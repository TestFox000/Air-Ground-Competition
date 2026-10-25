# -*- coding: utf-8 -*-
"""机载自主规划旁挂进程（Orin 上独立运行，不依赖 ROS）。

为什么需要它
--------------------------------
官方控制要求（docs/raw_txt/02_15空地协同侦排挑战赛.txt 第 2 页）：
    "比赛过程中，无人机的定位、建图、识别、规划与控制等实时任务
      须由机载平台自主完成"
而 canonical 链里唯一有版本控制的规划实现 `src/ag_integrate` 此前被定位成
车端（香橙派）包。本进程把规划层搬到机上：

    机载 dzuav(13 类 检测+PnP) ──本地 UDP:50000──→ 本进程
        ├─ 信息帧 0x4001 → WorldModel 态势归并（红靶粘性否决 / 障碍多帧确认）
        ├─ 车姿帧 0x4002 → 车真实朝向（供 reset 断言取最近轴）
        ├─ MatrixFusion  → 11×14 落格矩阵（滑窗投票）
        ├─ evaluate_gate → 发车门控（雷数 / 靶面）
        ├─ build_mission → 朝向感知 Dijkstra + Held-Karp + 车命令
        └─ 落盘 route_plan.json / cell_matrix.json / planned_path.txt
           + 广播 0x4003 车命令帧 → 车端薄接收 → 官方 path_executor

设计边界（刻意不做的事）
--------------------------------
  * 不复制规划源码：直接 sys.path 复用 src/ag_integrate，避免多副本漂移；
  * 不碰 C++：dzuav 一行不改（它已在向 50000 广播，本地监听即消费），
    因此不触发 orin_build_verify.sh；
  * 不装 ROS：只用标准库 + 纯 Python 规划核心（pipeline 无 ROS / 无 socket）；
  * 不下发「朝向对齐」：align 需要车端 /cmd_vel 与底盘契约，属执行侧，
    留在车端（本进程只负责把 reset 断言朝向算对后写进 0x4003）。

✅ 规划是唯一决策核心：本进程与车端 integrate_node（local 模式）**不可同时消费
   同一 UDP:50000**（同机第二份 bind 会 fail-fast 暴露）。上车时车端切 onboard 模式。

用法（机上）
--------------------------------
  python3 tools/orin_plan_sidecar.py --dry-run                       # 只落盘不发车命令
  python3 tools/orin_plan_sidecar.py --send                          # 落盘 + 广播 0x4003
  python3 tools/orin_plan_sidecar.py --plan-mode on-change --send    # 态势变化即重规划
  python3 tools/orin_plan_sidecar.py --duration-seconds 60           # 跑 60s 自动退出
  python3 tools/orin_plan_sidecar.py --params <yaml> --out <目录>     # 指定配置/产物目录

产物（--out，默认 /tmp/ag_plan_<时间戳>）：
  route_plan.json     路线 + 压缩点 + 车命令 + 校验结论（字段与 video_recon 同源）
  cell_matrix.json    11×14 落格矩阵 + 精确坐标明细
  planned_path.txt    执行路线格序列（人读）
"""
from __future__ import annotations

import argparse
import json
import re
import socket
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from math import gcd
from pathlib import Path

_REPO = Path(__file__).resolve().parents[1]
_SRC = _REPO / "src" / "ag_integrate"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from ag_integrate.car_commands import parse_cell  # noqa: E402
from ag_integrate.field_frame import LETTERS  # noqa: E402
from ag_integrate.field_matrix import (  # noqa: E402
    MatrixFusion, build_grid, render_report_from_spots,
)
from ag_integrate.heading_calibration import nearest_axis  # noqa: E402
from ag_integrate.intel_protocol import (  # noqa: E402
    MSG_ID_CAR_COMMAND, MSG_ID_CAR_POSE, CarCommandFrame,
    build_car_command, detect_msg_id, parse, parse_car_pose,
)
from ag_integrate.mission_router import _MID_BOUNDARY  # noqa: E402
from ag_integrate.pipeline import build_mission, evaluate_gate  # noqa: E402
from ag_integrate.route_progress import filter_swept_mines, route_progress  # noqa: E402
from ag_integrate.route_time import RouteTimeParams  # noqa: E402
from ag_integrate.world_model import WorldModel  # noqa: E402

MATRIX_LEGEND = "M=爆炸物 O=障碍 B=蓝靶(敌) R=红靶(友) A=桥洞 C=车 .=空"

# 格号正则（与 integrate_node._CELL_RE 同款）：两位行号分支在前，
# 否则 "N10" 会被 [1-9] 截成 "N1"。
_CELL_RE = re.compile(r"[A-N](?:1[01]|[1-9])")


# ---------------------------------------------------------------- 配置

@dataclass
class PlanConfig:
    """规划配置。来源是 canonical 的 src/ag_integrate/config/match_params.yaml
    （integrate_node 段），字段名与 ROS 参数一一对应，避免出现第二份默认值副本。"""

    udp_port: int = 50000
    start_cell: str = "N10"
    goal_cell: str = "N5"
    start_heading: str = "y"
    final_heading: str | None = "y"
    min_mines: int = 1
    require_face: bool = True
    red_veto_ttl_s: float = 5.0
    obstacle_confirm_frames: int = 2
    cross_gates: tuple[str, ...] = ()
    mid_boundary: int = _MID_BOUNDARY
    crossing_penalty: float = 0.0
    tunnel_dirs: dict[str, str] | None = None
    zone_order: tuple[str, ...] | None = None
    turn_45: float = 0.75
    allow_diagonal: bool = True
    heading_bias_source: str = "none"
    car_pose_ttl_s: float = 1.0
    target_track: tuple[str, str] | None = None
    target_blue_face: str | None = None
    # 2026-10-24 与 integrate_node 同步翻默认：重规划链开启（缺雷修复）。
    # 依据同 integrate_node 注释——车端 _install_route 进度清零从头执行 +
    # route_progress 起点换当前格 = 闭环安全；现场要一次性规划显式置 False。
    replan_on_update: bool = True
    time_soft_cap_s: float = 165.0
    time_cruise_mps: float = 0.25
    time_accel_overhead_s: float = 0.4
    time_mine_slow_s: float = 0.6
    time_shoot_stop_s: float = 4.0
    # 2026-10-25 综合规划增强项（语义见 mission_router._dijkstra docstring）：
    # 90° 内切守卫 / 实体挡板（K7-M7 几何债）/ 倒车挡位原语 / HK 朝向传递。
    # yaml 缺键时取此默认（与 integrate_node declare_parameter 默认一致）。
    turn90_guard: bool = True
    solid_barriers: bool = True
    reverse_gear: bool = True
    rev_mult: float = 1.5
    gear_switch_cost: float = 0.5
    heading_prop: bool = True
    # 倒车差异化计时（route_time）：None=关闭（yaml 置 null）。
    time_reverse_mps: float = 0.15
    time_gear_switch_s: float = 1.0
    matrix_enabled: bool = True
    matrix_fuse_window: int = 5
    matrix_fuse_confirm: int = 2
    # 2026-10-24 融合模式：matrix=现役二元票（默认，行为不变）；
    # evidential=置信度加权证据积累（协议 v2 conf + 负证据清除 + 粘性禁火，
    # 见 ag_integrate.evidential_fusion；低置信 0.4~0.6 真雷多帧可确认、
    # 单帧幽灵被清、已确认目标不闪断）。机上 C++ 发 v2 帧后全能力生效；
    # v1 帧下退化为"两次确认制"（仍优于整帧替换）。
    fusion_mode: str = "matrix"
    matrix_min_interval: float = 2.0

    @property
    def time_params(self) -> RouteTimeParams:
        return RouteTimeParams(
            cruise_mps=self.time_cruise_mps,
            accel_overhead_s=self.time_accel_overhead_s,
            mine_slow_s=self.time_mine_slow_s,
            shoot_stop_s=self.time_shoot_stop_s,
            reverse_mps=self.time_reverse_mps,
            gear_switch_s=self.time_gear_switch_s,
        )


def _as_tuple_str(value) -> tuple[str, ...]:
    if value is None:
        return ()
    if isinstance(value, str):
        value = [value]
    return tuple(str(v).strip() for v in value if str(v).strip())


def _as_tuple2(value) -> tuple[str, str] | None:
    items = _as_tuple_str(value)
    return (items[0].upper(), items[1].upper()) if len(items) == 2 else None


def _parse_tunnel_dirs(value) -> dict[str, str] | None:
    out: dict[str, str] = {}
    for item in _as_tuple_str(value):
        cell, sep, direction = item.partition(":")
        if sep:
            out[cell.strip()] = direction.strip()
    return out or None


def load_params(path: Path) -> dict:
    """读 match_params.yaml 的 integrate_node 段。

    优先 PyYAML（机上 python3-yaml 一般随 JetPack 预装）；缺失时退化为缩进解析的
    最小实现（与 tests/yaml_lite.py 同策略），绝不内置场地常量副本。
    """
    text = path.read_text(encoding="utf-8")
    try:
        import yaml  # noqa: PLC0415

        raw = yaml.safe_load(text) or {}
        body = raw.get("integrate_node", {})
        return body.get("ros__parameters", body) if isinstance(body, dict) else {}
    except ImportError:
        pass

    out: dict = {}
    in_section = in_params = False
    for line in text.splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        s = line.strip()
        indent = len(line) - len(line.lstrip())
        if indent == 0 and s.endswith(":"):
            in_section, in_params = s[:-1] == "integrate_node", False
            continue
        if in_section and indent == 2 and s == "ros__parameters:":
            in_params = True
            continue
        if in_section and in_params and indent >= 4 and ":" in s:
            key, _, value = s.partition(":")
            value = value.split("#")[0].strip()
            out[key.strip()] = _scalar(value)
    return out


def _scalar(value: str):
    if value.startswith("[") and value.endswith("]"):
        return [v.strip().strip("\"'") for v in value[1:-1].split(",") if v.strip()]
    if len(value) >= 2 and value[0] == value[-1] == '"':
        return value[1:-1]
    if value in ("true", "false"):
        return value == "true"
    try:
        return json.loads(value)
    except ValueError:
        return value


def config_from_params(raw: dict) -> PlanConfig:
    """yaml 参数字典 -> PlanConfig。缺项用 dataclass 默认（与 ROS declare_parameter
    默认同源），不做第二套场地常量。"""
    final = str(raw.get("final_heading", "y")).strip().lower()
    face = str(raw.get("target_blue_face", "")).strip().upper()
    return PlanConfig(
        udp_port=int(raw.get("udp_port", 50000)),
        start_cell=str(raw.get("start_cell", "N10")),
        goal_cell=str(raw.get("goal_cell", "N5")),
        start_heading=str(raw.get("start_heading", "y")).strip().lower(),
        final_heading=final if final in ("x", "y") else None,
        min_mines=int(raw.get("min_mines", 1)),
        require_face=bool(raw.get("require_face", True)),
        red_veto_ttl_s=float(raw.get("red_veto_ttl_s", 5.0)),
        obstacle_confirm_frames=max(1, int(raw.get("obstacle_confirm_frames", 2))),
        cross_gates=_as_tuple_str(raw.get("cross_gates")),
        mid_boundary=int(raw.get("mid_boundary", _MID_BOUNDARY)),
        crossing_penalty=float(raw.get("crossing_penalty", 0.0)),
        tunnel_dirs=_parse_tunnel_dirs(raw.get("tunnel_dirs")),
        zone_order=_as_tuple_str(raw.get("zone_order")) or None,
        turn_45=float(raw.get("turn_45", 0.75)),
        allow_diagonal=bool(raw.get("allow_diagonal", True)),
        heading_bias_source=str(raw.get("heading_bias_source", "none")).strip().lower(),
        car_pose_ttl_s=max(0.0, float(raw.get("car_pose_ttl_s", 1.0))),
        target_track=_as_tuple2(raw.get("target_track")),
        target_blue_face=face if face in ("S", "N", "E", "W") else None,
        replan_on_update=bool(raw.get("replan_on_update", True)),  # 2026-10-24 默认开（缺雷修复）
        time_soft_cap_s=float(raw.get("time_soft_cap_s", 165.0)),
        time_cruise_mps=float(raw.get("time_cruise_mps", 0.25)),
        time_accel_overhead_s=float(raw.get("time_accel_overhead_s", 0.4)),
        time_mine_slow_s=float(raw.get("time_mine_slow_s", 0.6)),
        time_shoot_stop_s=float(raw.get("time_shoot_stop_s", 4.0)),
        turn90_guard=bool(raw.get("turn90_guard", True)),          # 2026-10-25 综合规划
        solid_barriers=bool(raw.get("solid_barriers", True)),
        reverse_gear=bool(raw.get("reverse_gear", True)),
        rev_mult=float(raw.get("rev_mult", 1.5)),
        gear_switch_cost=float(raw.get("gear_switch_cost", 0.5)),
        heading_prop=bool(raw.get("heading_prop", True)),
        time_reverse_mps=float(raw.get("time_reverse_mps", 0.15)),
        time_gear_switch_s=float(raw.get("time_gear_switch_s", 1.0)),
        matrix_enabled=bool(raw.get("matrix_log", True)),
        matrix_fuse_window=int(raw.get("matrix_fuse_window", 5)),
        matrix_fuse_confirm=int(raw.get("matrix_fuse_confirm", 2)),
        fusion_mode=str(raw.get("fusion_mode", "matrix")).strip().lower(),
        matrix_min_interval=float(raw.get("matrix_min_interval", 2.0)),
    )


# ---------------------------------------------------------------- 规划核心（无 I/O，可单测）

def route_cells(route_text: str) -> list[str]:
    """从车端 route 命令串取回执行格序列（去掉 '*' 与尾部 ',x/,y' 朝向 token）。"""
    cells: list[str] = []
    for token in route_text.split():
        tok = token.strip().rstrip("*")
        tok = tok.split(",", 1)[0]      # 末段 ",x/,y/,-y" 是终点朝向 token
        if tok:
            cells.append(tok)
    return cells


def route_metrics(cells: list[str]) -> tuple[float, int]:
    """（对角步折算路线长度，转向次数）——只统计相邻格间的实际移动。"""
    def ref(name: str) -> tuple[int, int]:
        s = name.strip().upper()
        return LETTERS.index(s[0]), int(s[1:]) - 1

    length = 0.0
    turns = 0
    prev: tuple[int, int] | None = None
    last: tuple[int, int] | None = None
    for name in cells:
        cur = ref(name)
        if last is not None:
            d = (cur[0] - last[0], cur[1] - last[1])
            length += (d[0] ** 2 + d[1] ** 2) ** 0.5
            if prev is not None and (abs(d[0] - prev[0]) + abs(d[1] - prev[1])) in (1, 2):
                turns += 1
            prev = d
        last = cur
    return round(length, 2), turns


def snapshot_signature(snapshot) -> tuple:
    """重规划守卫的态势签名：雷 / 桥门 / 靶面 / 障碍 四类集合。

    与 integrate_node._snap_signature 同口径（容器内容变了才放行重规划）。
    """
    return (
        frozenset(c.name for c in snapshot.mines),
        frozenset(c.name for c in snapshot.passable),
        frozenset((t.cell.name, t.is_blue) for t in snapshot.targets),
        frozenset(c.name for c in snapshot.obstacles),
    )


def effective_start_heading(cfg: PlanConfig, car_yaw: float | None) -> str:
    """reset 断言朝向：align 模式由车端把车头摆到 start_heading（零残差）；
    udp 模式按无人机观测的真实朝向取最近坐标轴（残差 ≤45°）；无观测退回 start_heading。"""
    if cfg.heading_bias_source == "udp" and car_yaw is not None:
        return nearest_axis(car_yaw)
    return cfg.start_heading


@dataclass
class PlannerCore:
    """规划核心：吃情报帧/车姿帧，吐规划产物。无 socket、无 ROS，便于单测。"""

    cfg: PlanConfig
    out_dir: Path
    plan_mode: str = "once"          # once | on-change
    fusion: MatrixFusion = field(init=False)
    world: WorldModel = field(init=False)
    car_yaw: float | None = field(default=None, init=False)
    car_yaw_mono: float | None = field(default=None, init=False)
    car_xy: tuple[float, float] | None = field(default=None, init=False)
    last_route: str = field(default="", init=False)
    last_reset: str = field(default="", init=False)
    last_start_used: str = field(default="", init=False)
    last_plan_ts_ms: int = field(default=0, init=False)
    last_sig: tuple | None = field(default=None, init=False)
    last_matrix_sig: tuple | None = field(default=None, init=False)
    last_matrix_mono: float = field(default=-1e9, init=False)
    plan_count: int = field(default=0, init=False)
    last_report: str = field(default="", init=False)
    last_degraded: tuple[str, ...] = field(default=(), init=False)

    def __post_init__(self) -> None:
        self.out_dir = Path(self.out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        if self.cfg.start_heading not in ("x", "y"):
            raise ValueError(f"start_heading 只支持 'x'/'y'，收到 {self.cfg.start_heading!r}")
        parse_cell(self.cfg.start_cell)
        parse_cell(self.cfg.goal_cell)
        self.fusion_mode = self.cfg.fusion_mode
        if self.fusion_mode == "evidential":
            from ag_integrate.evidential_fusion import EvidentialFusion
            self.fusion = EvidentialFusion()
        else:
            self.fusion = MatrixFusion(window=self.cfg.matrix_fuse_window,
                                       confirm=self.cfg.matrix_fuse_confirm)
        self.world = WorldModel(red_veto_ttl_s=self.cfg.red_veto_ttl_s,
                                obstacle_confirm_frames=self.cfg.obstacle_confirm_frames)

    # -------- 输入 --------
    def feed_frame(self, frame, recv_mono: float | None = None) -> None:
        if recv_mono is None:
            recv_mono = time.monotonic()
        self.world.update(frame, recv_mono=recv_mono)
        if self.fusion_mode == "evidential":
            self.fusion.add_frame(frame, recv_mono=recv_mono)
        else:
            self.fusion.add_frame(frame)

    def feed_car_pose(self, pose, recv_mono: float | None = None) -> None:
        self.car_yaw = pose.yaw
        self.car_yaw_mono = time.monotonic() if recv_mono is None else recv_mono
        self.car_xy = (pose.x, pose.y)

    def _fresh_car_pose(self):
        """新鲜车位姿 (x, y, yaw)；缺失/过期返回 None（重规划进度用，绝不
        拿冻结观测当最新进度——过期位置会把起点换到车早已离开的格子）。"""
        xy = self.car_xy
        yaw = self._fresh_car_yaw()
        if xy is None or yaw is None:
            return None
        return (xy[0], xy[1], yaw)

    def _fresh_car_yaw(self) -> float | None:
        if self.car_yaw is None or self.car_yaw_mono is None:
            return None
        if self.cfg.car_pose_ttl_s > 0 and \
                time.monotonic() - self.car_yaw_mono > self.cfg.car_pose_ttl_s:
            return None       # 过期按"没有观测"处理（绝不用冻结 yaw 断言）
        return self.car_yaw

    # -------- 决策 --------
    def step(self) -> str | None:
        """一拍决策：门控 -> 规划 -> 落盘。返回新路线串（无更新返回 None）。

        规划触发：
          once      -> 首次规划成功后不再重算（对应 yaml replan_on_update=false）
          on-change -> 态势签名变化即重算（对应 replan_on_update=true；离线回放用）
        """
        # 2026-10-24：evidential 模式下规划吃融合快照（置信度加权、负证据
        # 清除、已确认不闪断）；matrix 模式保持 world_model 整帧替换（行为
        # 不变）。world 仍持续更新（红靶否决/车朝向消费者不受影响）。
        snapshot = getattr(self.fusion, "snapshot", None)             if self.fusion_mode == "evidential" else None
        if snapshot is None:
            snapshot = self.world.snapshot
        if snapshot is None:
            return None
        if self.cfg.matrix_enabled:
            # 矩阵只是可观测量，绝不能挡住决策链：写失败（磁盘满/权限/序列化异常）
            # 只降级为告警，门控与规划照常推进。2026-10-20 机上实测：矩阵签名
            # 的 TypeError 曾直接打死整个 sidecar（表现＝"矩阵在、规划不推进"）。
            try:
                self._write_matrix(interactive=True)
            except Exception as exc:            # noqa: BLE001 - 见上
                print(f"[plan] 矩阵落盘失败（已忽略，决策继续）: "
                      f"{type(exc).__name__}: {exc}", file=sys.stderr)

        gate = evaluate_gate(snapshot, self.cfg.min_mines, self.cfg.require_face)
        if not gate.ok:
            return None
        sig = snapshot_signature(snapshot)
        if self.last_route:
            if self.plan_mode == "once":
                return None
            if sig == self.last_sig:
                return None
        return self._plan(snapshot, sig)

    def _plan(self, snapshot, sig: tuple) -> str | None:
        import warnings

        # —— 重规划基于最新进度（2026-10-23，与车端 integrate_node 同口径）——
        # 首发按摆位起点；重规划时有新鲜 0x4002 车位姿则换到车的当前格 +
        # 真实朝向最近轴 + 剔除已扫雷。守卫签名（sig）由 step() 用全量快照
        # 算好，这里只剪喂给规划器的副本，不影响 on-change 触发口径。
        start_cell = self.cfg.start_cell
        heading = effective_start_heading(self.cfg, self._fresh_car_yaw())
        plan_snapshot = snapshot
        if self.last_route:
            pose = self._fresh_car_pose()
            progress = None
            if pose is not None:
                try:
                    progress = route_progress(self.last_route, *pose)
                except ValueError as exc:
                    print(f"[plan] 车位姿无法落格，重规划退回摆位起点: {exc}",
                          file=sys.stderr)
            if progress is not None:
                start_cell = progress.cell.name
                heading = progress.heading
                plan_snapshot = filter_swept_mines(
                    snapshot, progress.swept_cells)
                print(f"[plan] 重规划基于最新进度: 起点 {start_cell}"
                      f"（朝向 {heading}），已扫雷 "
                      f"{len(snapshot.mines) - len(plan_snapshot.mines)} 颗")
        self.last_start_used = start_cell
        try:
            with warnings.catch_warnings(record=True) as caught:
                warnings.simplefilter("always", RuntimeWarning)
                commands = build_mission(
                    plan_snapshot, start_cell, self.cfg.goal_cell,
                    heading, self.cfg.final_heading,
                    self.cfg.cross_gates, self.cfg.mid_boundary,
                    target_track=self.cfg.target_track,
                    target_blue_face=self.cfg.target_blue_face,
                    time_params=self.cfg.time_params,
                    soft_cap_s=self.cfg.time_soft_cap_s,
                    crossing_penalty=self.cfg.crossing_penalty,
                    tunnel_dirs=self.cfg.tunnel_dirs,
                    zone_order=self.cfg.zone_order,
                    turn_45=self.cfg.turn_45,
                    allow_diagonal=self.cfg.allow_diagonal,
                    turn90_guard=self.cfg.turn90_guard,      # 2026-10-25 综合规划
                    solid_barriers=self.cfg.solid_barriers,
                    reverse=self.cfg.reverse_gear,
                    rev_mult=self.cfg.rev_mult,
                    gear_switch=self.cfg.gear_switch_cost,
                    heading_prop=self.cfg.heading_prop,
                )
        except (ValueError, IndexError, KeyError) as exc:
            self.last_report = f"规划失败，等待态势更新: {exc}"
            return None
        self.last_degraded = tuple(str(w.message) for w in caught)
        self.last_sig = sig
        if commands.route == self.last_route:
            return None
        self.last_route = commands.route
        self.plan_count += 1
        self.last_reset = commands.reset
        self.last_plan_ts_ms = snapshot.timestamp_ms
        self._write_route_plan(snapshot, commands, heading)
        self.write_path()     # 三份产物在规划成功的那一刻齐备
        for text in self.last_degraded:
            print(f"[plan] 降级: {text}")
        print(f"[plan] #{self.plan_count} reset={commands.reset} | route={commands.route}")
        return commands.route

    # -------- 产物 --------
    def _write_route_plan(self, snapshot, commands, heading: str) -> None:
        cells = route_cells(commands.route)
        length, turns = route_metrics(cells)
        present = set(cells)
        mines = [c.name for c in snapshot.mines]
        missing = [c for c in mines if c not in present]
        obstacles = {c.name for c in snapshot.obstacles}
        viol = [c for c in cells if c in obstacles]

        payload = {
            "planner": "onboard_sidecar",
            "source": {
                "mines/obstacles/targets": "机载 dzuav 13 类识别（0x4001 实时帧 + 滑窗融合）",
                "start": self.last_start_used or self.cfg.start_cell,
                "goal": self.cfg.goal_cell,
                "reset_heading_assert": heading,
                "target_track": list(self.cfg.target_track) if self.cfg.target_track else [],
                "target_blue_face": self.cfg.target_blue_face,
            },
            "snapshot": {
                "mines": mines,
                "passable": [c.name for c in snapshot.passable],
                "obstacles": sorted(obstacles),
                "targets": [f"{'蓝' if t.is_blue else '红'}@{t.cell.name}" for t in snapshot.targets],
            },
            "route": cells,                       # 执行路线（= 压缩后，含射击停靠格）
            "route_len_cells": len(cells),
            "route_len_diag": length,
            "turns": turns,
            "car_cmds": {
                "reset": commands.reset, "route": commands.route,
                "shoot_cells": list(commands.shoot_cells),
            },
            "check": {
                "排雷覆盖": "全部覆盖" if not missing else f"缺失{missing}",
                "障碍避让": "通过" if not viol else f"穿障{viol}",
            },
            "est_seconds": round(commands.est_seconds, 1),
            "shed_shooting": commands.shed_shooting,
            "degraded": list(self.last_degraded),
            "timestamp_ms": snapshot.timestamp_ms,
        }
        self._atomic_write(self.out_dir / "route_plan.json",
                           json.dumps(payload, ensure_ascii=False, indent=2))
        self.last_report = (f"[plan] 执行路线 {len(cells)} 格 | 排雷覆盖 "
                            f"{'✓' if not missing else missing} | 避障 "
                            f"{'✓' if not viol else viol} | 预估 {commands.est_seconds:.0f}s")

    def write_matrix(self) -> None:
        """强制落一次矩阵产物（退出前收尾用）。"""
        if self.cfg.matrix_enabled and self.world.snapshot is not None:
            self._write_matrix(interactive=False, force=True)

    def _write_matrix(self, interactive: bool, force: bool = False) -> None:
        """矩阵产物：内容变化 + 节流（与 integrate_node 的打印策略同口径）。"""
        snapshot = self.world.snapshot
        if snapshot is None:
            return
        spots = self.fusion.fused_spots() + self.fusion.outside_spots()
        # 矩阵签名：出界目标的 col/row 是 None，同类里"一个出界 + 一个落格"时
        # 直接排序会撞 None < int 的 TypeError（2026-10-20 机上实跑踩到，进程
        # 当场崩）。签名只用于判"内容变没变"，用 -1 代表出界即可，不影响语义。
        sig = tuple(sorted(
            (s.kind.value, -1 if s.col is None else s.col,
             -1 if s.row is None else s.row)
            for s in spots))
        now = time.monotonic()
        if not force:
            if sig == self.last_matrix_sig:
                return
            if interactive and now - self.last_matrix_mono < self.cfg.matrix_min_interval:
                return
        self.last_matrix_sig = sig
        self.last_matrix_mono = now
        grid = build_grid(spots)
        payload = {
            "planner": "onboard_sidecar",
            "timestamp_ms": snapshot.timestamp_ms,
            "grid": grid,
            "legend": MATRIX_LEGEND,
            "fused": True,
            "spots": [
                {"kind": s.kind_name, "cell": s.cell,
                 "x": round(s.x, 3), "y": round(s.y, 3)}
                for s in spots
            ],
        }
        self._atomic_write(self.out_dir / "cell_matrix.json",
                           json.dumps(payload, ensure_ascii=False, indent=2))
        if interactive:
            print(render_report_from_spots(spots, snapshot.timestamp_ms, fused=True))

    def write_path(self) -> None:
        """planned_path.txt：执行路线格序列（人读；含 '*' 射击格）。"""
        if not self.last_route:
            return
        text = " ".join(route_cells(self.last_route)) or self.last_route
        self._atomic_write(self.out_dir / "planned_path.txt", text + "\n")

    @staticmethod
    def _atomic_write(path: Path, text: str) -> None:
        """原子落盘：先写 .tmp 再 replace，避免读侧读到半截 JSON。"""
        tmp = path.with_suffix(path.suffix + ".tmp")
        tmp.write_text(text, encoding="utf-8")
        tmp.replace(path)


# ---------------------------------------------------------------- 进程外壳（socket）

class _UdpLink:
    def __init__(self, port: int, broadcast_addr: str, send: bool) -> None:
        # 刻意不设 SO_REUSEADDR：重复 bind 即 fail-fast——机上同时跑两个决策核心
        # （sidecar 与车端 local 模式）会当场炸，而不是各收一份副本双决策。
        self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock.bind(("", port))
        self.recv_sock.settimeout(0.1)
        self.send = send
        self.addr = (broadcast_addr, port)
        self.send_sock = None
        # 收包计数：现场只看到"规划 0 次"时，要能立刻分清是相机没出帧
        # （intel=0）还是出了帧但没有可用目标（intel>0、plan=0）。
        self.intel_frames = 0
        self.car_poses = 0
        self.bad_packets = 0
        if send:
            self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.send_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    def drain(self, core: PlannerCore) -> None:
        """收 0x4001 / 0x4002；忽略自己广播出去的 0x4003（避免自激）。"""
        while True:
            try:
                data, _addr = self.recv_sock.recvfrom(65535)
            except socket.timeout:
                return
            except OSError as exc:
                print(f"[link] 收包失败: {type(exc).__name__}: {exc}", file=sys.stderr)
                return
            recv_mono = time.monotonic()
            msg_id = detect_msg_id(data)
            if msg_id is None or msg_id == MSG_ID_CAR_COMMAND:
                continue
            if msg_id == MSG_ID_CAR_POSE:
                try:
                    core.feed_car_pose(parse_car_pose(data), recv_mono)
                    self.car_poses += 1
                except ValueError as exc:
                    self.bad_packets += 1
                    print(f"[link] 丢弃坏车位姿帧: {exc}", file=sys.stderr)
                continue
            try:
                frame = parse(data)
            except ValueError as exc:
                self.bad_packets += 1
                print(f"[link] 丢弃坏情报帧: {exc}", file=sys.stderr)
                continue
            self.intel_frames += 1
            core.feed_frame(frame, recv_mono)

    def send_command(self, ts_ms: int, reset: str, route: str) -> None:
        if not self.send_sock or not route:
            return
        try:
            self.send_sock.sendto(
                build_car_command(CarCommandFrame(ts_ms, reset, route)), self.addr)
        except OSError as exc:
            print(f"[link] 车命令广播失败: {type(exc).__name__}: {exc}", file=sys.stderr)

    def close(self) -> None:
        for sock in (self.recv_sock, self.send_sock):
            if sock is not None:
                sock.close()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="机载自主规划旁挂进程（Orin，无 ROS）")
    ap.add_argument("--params", default=str(_SRC / "config" / "match_params.yaml"),
                    help="canonical 参数文件（默认 src/ag_integrate/config/match_params.yaml）")
    ap.add_argument("--out", default="", help="产物目录（默认 /tmp/ag_plan_<时间戳>）")
    ap.add_argument("--udp-port", type=int, default=0, help="覆盖 yaml 的 udp_port")
    ap.add_argument("--plan-mode", choices=("once", "on-change"), default="on-change",
                    help="on-change=态势签名变化即重算（2026-10-24 起默认，对齐"
                         " integrate_node replan_on_update=true 缺雷修复）；"
                         "once=首次成功后不再重算（离线回放/保守现场用）")
    ap.add_argument("--send", action="store_true",
                    help="广播 0x4003 车命令帧到车端（默认关闭=只落盘，安全）")
    ap.add_argument("--broadcast-addr", default="255.255.255.255")
    ap.add_argument("--duration-seconds", type=float, default=0.0,
                    help=">0 时跑够秒数自动退出（冒烟测试用）")
    ap.add_argument("--tick-hz", type=float, default=10.0)
    ap.add_argument("--resend-period-s", type=float, default=2.0,
                    help="最近一次车命令的周期重发间隔秒（0=关闭）。"
                         "车端可能晚于本进程启动、或中途重启；话题无滞留，"
                         "只发一次会让车永久收不到路线；车端按 route 文本去重，重发幂等")
    args = ap.parse_args(argv)

    out_dir = Path(args.out) if args.out else \
        Path("/tmp") / f"ag_plan_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    cfg = config_from_params(load_params(Path(args.params)))
    if args.udp_port:
        cfg.udp_port = args.udp_port

    core = PlannerCore(cfg, out_dir, plan_mode=args.plan_mode)
    link = _UdpLink(cfg.udp_port, args.broadcast_addr, args.send)

    print(f"[sidecar] 机载规划就绪: 监听 UDP:{cfg.udp_port} -> "
          f"{cfg.start_cell}->{cfg.goal_cell} | plan-mode={args.plan_mode} | "
          f"{'实发 0x4003' if args.send else 'dry-run（只落盘）'}")
    print(f"[sidecar] 产物目录: {out_dir}")

    start = time.monotonic()
    period = 1.0 / max(0.5, args.tick_hz)
    last_send_mono = 0.0
    try:
        while True:
            # 单拍异常绝不终止整局：rclpy 侧 integrate_node 的 _on_tick 同理
            # （"宁可丢一拍，不要丢一局"）。2026-10-20 机上实测：_write_matrix 的
            # TypeError 直接把进程打死，表现是"矩阵在、规划不动"，极难现场定位。
            try:
                link.drain(core)
                route = core.step()
            except Exception as exc:            # noqa: BLE001 - 见上
                print(f"[sidecar] 决策拍异常（已吞掉，下拍继续）: "
                      f"{type(exc).__name__}: {exc}", file=sys.stderr)
                route = None
            if route is not None:
                core.write_path()
                link.send_command(core.last_plan_ts_ms, core.last_reset, route)
                last_send_mono = time.monotonic()
            elif (args.send and core.last_route and args.resend_period_s > 0
                  and time.monotonic() - last_send_mono >= args.resend_period_s):
                # 周期重发最近命令：车端晚启动/重启时靠这条补上（车端按 route 去重）
                link.send_command(core.last_plan_ts_ms, core.last_reset, core.last_route)
                last_send_mono = time.monotonic()
            if args.duration_seconds > 0 and \
                    time.monotonic() - start >= args.duration_seconds:
                break
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n[sidecar] 收到中断，收尾落盘…")
    finally:
        for step in (core.step, core.write_path, core.write_matrix):
            try:
                step()
            except Exception as exc:            # noqa: BLE001 - 退出时不掩盖真实原因
                print(f"[sidecar] 收尾 {step.__name__} 失败: "
                      f"{type(exc).__name__}: {exc}", file=sys.stderr)
        link.close()
    print(f"[sidecar] 收帧：情报 {link.intel_frames} / 车位姿 {link.car_poses}"
          f" / 坏帧 {link.bad_packets}")
    print(f"[sidecar] 退出：规划 {core.plan_count} 次 | 产物 {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
