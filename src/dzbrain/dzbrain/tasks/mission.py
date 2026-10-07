# -*- coding: utf-8 -*-
"""MissionTask（10 Hz）—— 任务调度：出发 → 航点推进 → 停站三段 → 终点横幅。

对应 `docs/P1接口约定.md` §3.3；停站三段式与超时兜底复刻旧实现
`path_executor` 的打靶四阶段状态机（survey §4.3，旧 2034–2193），改动两处：

1. **三段而不是四段**：旧版 `SETTLE → ACQUIRE → STABLE → FIRE` 里的 `STABLE`
   （等瞄稳 40 px 连续 5 帧）在新版归 `AimTask`（云台自己判定"锁定"），
   Mission 只在 `ACQUIRE` 里等 `aim_status == TRACK` 且视觉有目标 → 直接进 `FIRE`。
2. **每段独立超时**：绝不允许任何一段把主循环卡住（旧版 `SHOOT_WAIT_TIMEOUT` 是死变量）。

铁律：**宁可漏打，不可卡死**。所以每一段都有"超时即放行"分支，且放行前一定
`publish_fire_gate(False)` 收尾（旧版 2181–2183 的同一条纪律）。

其他约定与出处：

- 出发：`mission.auto_start` + `mission.auto_start_delay_s` → `parse_reset('N10,y')` 复位 belief
  → `parse_mission(route.tokens)` → 写 `P_ROUTE` → `EV_ROUTE_LOADED` → 阶段 `DRIVE`（§3.3）。
- 开机跨帧一致性：`mission.scan_mode == 'continuous'` 时开局发 `publish_scan_request(2)`
  （**持续**扫描）。readme【十】第 31 条：绝不能在到点时发 `1` 把它降级成单趟。
- 终点：最后一个航点到达 → `FINISH`（**锁存**）、`publish_brake()`、`publish_scan_request(0)`、
  `EV_MISSION_FINISHED`、结束横幅（用时/发数/漂移）。
- 外部命令（`/nav/waypoint`）：`reset:BLOCK,HEAD` / `load:文件` / `单点 BLOCK,HEAD` / `多点 ...`
  全部支持；**未知命令只告警不崩**（旧版 `waypoint_callback` 的教训）。

★ 时间来源一律走构造注入的 `clock`（默认真实 `time.monotonic`），三段超时全部基于它，
  测试用 `FakeClock` 快进即可 —— 任务内**不得**直接调 `time.time()`。

⚠️ SCHEMA 缺键（本次未改 `core/config.py`，故用带默认值的 `cfg.get`）：
  `mission.shots_per_stop` / `mission.settle_time_s` / `mission.settle_eps_m` /
  `mission.settle_speed_mps` / `mission.lock_timeout_s` / `mission.shot_timeout_s` /
  `mission.scan_stop_timeout_s`。建议 P1 收口时补进 SCHEMA，否则 `Config.validate()`
  看不到它们（键拼错不会被"未知配置段"告警兜住）。
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any, Callable

from ..core import geometry as geo
from ..core.blackboard import (P_MISSION_PHASE, P_POSE, P_ROUTE, P_ROUTE_INDEX, P_ROUTE_S,
                               P_SAFETY, P_STOP_POINT, P_TELEMETRY, Blackboard)
from ..core.config import Config
from ..core.events import (EV_MISSION_FINISHED, EV_MISSION_STARTED, EV_ARRIVED_STOP_POINT,
                           EV_ROUTE_LOADED, EV_STOP_POINT_DONE, EV_WAYPOINT_REACHED, EventBus)
from ..core.logging import Throttle, banner
from ..core.state import SafetyState, StopKind, StopPhase, StopPointState
from ..core.task import TaskThread
from ..io.contract import AimStatus, MissionPhase
from ..pursuit.path import Path
from ..route.model import Route, Waypoint
from ..route.parser import parse_mission, parse_reset, parse_token

__all__ = ["MissionTask", "MISSION_STATE_KEY"]

#: 遥测键：Mission 的当前上下文（停站点/航点/用时）
MISSION_STATE_KEY = "mission_state"

#: 只有这些阶段允许"发车/推进"（★ 白名单：未知阶段一律不动）
_DRIVING_PHASES: tuple[str, ...] = (MissionPhase.DRIVE, MissionPhase.APPROACH)

#: 终局阶段：不再自动推进，也不再被外部命令复位
_TERMINAL_PHASES: tuple[str, ...] = (MissionPhase.FINISH, MissionPhase.SAFE_STOP)

_EPS = 1e-9


@dataclass
class _StopContext:
    """一个停站点的**可变**上下文（只在 Mission 线程内使用，不跨线程共享）。"""

    index: int = -1
    block: str = ""
    kind: str = StopKind.SHOOT
    phase: str = StopPhase.SETTLE
    started_at: float = 0.0
    phase_started_at: float = 0.0
    settle_armed_at: float | None = None
    last_x: float | None = None
    last_y: float | None = None
    shots_at_arrival: int = 0
    shots_needed: int = 3
    detail: str = ""

    @property
    def active(self) -> bool:
        return self.index >= 0 and self.phase != StopPhase.DONE


class MissionTask(TaskThread):
    """任务调度（10 Hz）。"""

    def __init__(self, bridge, bb: Blackboard, bus: EventBus, cfg: Config, *,
                 log: Callable[[str], None] = print, group=None,
                 clock: Callable[[], float] | None = None) -> None:
        super().__init__("MissionTask", 10.0, bus=bus, blackboard=bb, logger=log)
        self.bridge = bridge
        self.cfg = cfg
        self.group = group
        if clock is None:
            import time as _time
            clock = _time.monotonic
        self._clock = clock

        # ---- 配置（一次性读出；缺键全部走显式默认值，见模块 docstring 的 ⚠️） ----
        self._auto_start = bool(cfg.get("mission.auto_start"))
        self._auto_start_delay_s = float(cfg.get("mission.auto_start_delay_s") or 0.0)
        self._reset_spec = str(cfg.get("mission.reset") or "")
        self._scan_mode = str(cfg.get("mission.scan_mode") or "").strip().lower()
        self._shots_per_stop = int(cfg.get("mission.shots_per_stop", 3) or 3)
        self._settle_time_s = float(cfg.get("mission.settle_time_s", 0.8))
        self._settle_eps_m = float(cfg.get("mission.settle_eps_m", 0.02))
        self._settle_speed_mps = float(cfg.get("mission.settle_speed_mps", 0.02))
        self._lock_timeout_s = float(cfg.get("mission.lock_timeout_s", 18.0))
        self._shot_timeout_s = float(cfg.get("mission.shot_timeout_s", 30.0))
        self._scan_stop_timeout_s = float(cfg.get("mission.scan_stop_timeout_s", 8.0))
        self._settle_grace_s = max(0.4 * self._shot_timeout_s, 1.0)     # ★ 旧版 0.4×SHOOT_TIMEOUT
        self._arrival_m = float(cfg.get("pursuit.arrival_m"))
        self._stop_radius_m = float(cfg.get("pursuit.stop_radius_m"))
        self._approach_m = float(cfg.get("pursuit.approach_m"))
        self._rate_limit_s = float(cfg.get("logging.rate_limit_s") or 1.0)

        # ---- 任务态 ----
        self._created_at = self._clock()
        self._started_at: float | None = None
        self._phase = MissionPhase.IDLE
        self._route: Route | None = None
        self._path: Path | None = None
        self._route_name = ""
        self._waypoint_count = 0
        self._passed: list[bool] = []
        self._arrived = 0
        self._next_wp = 1
        self._handled_stops: set[int] = set()
        # ⚠️ 名字必须避开 `self._stop`：`threading.Thread` 内部有同名方法，
        #    被实例属性遮蔽后 `stop()/join()` 会在 `self._stop()` 处炸
        #    （`TypeError: 'NoneType' object is not callable`）—— 已实测踩过。
        self._stopctx: _StopContext | None = None
        self._last_stop_state = StopPointState()
        self._fire_gate_state: bool = False
        self._route_s: float | None = None
        self._dist_to_next_stop: float | None = None
        self._finish_latched = False
        self._finish_done = False
        self._finish_info: dict[str, Any] = {}
        self._scans: list[int] = []
        self._cmd_stats: dict[str, int] = {"total": 0, "rejected": 0}
        self._log_throttle = Throttle(self._rate_limit_s)

        # 停站点（含末点）的弧长前缀与目标索引，路线载入时算一次
        self._stop_s: dict[int, float] = {}
        self._stop_blocks: list[str] = []
        self._last_s = 0.0

    # ------------------------------------------------------------------ #
    #  只读接口（测试/遥测）
    # ------------------------------------------------------------------ #
    @property
    def phase(self) -> str:
        return self._phase

    @property
    def route(self) -> Route | None:
        return self._route

    @property
    def stop_context(self) -> _StopContext | None:
        return self._stopctx

    @property
    def handled_stops(self) -> set[int]:
        return set(self._handled_stops)

    def state(self) -> dict[str, Any]:
        """最近一拍的任务上下文（与 `P_TELEMETRY[MISSION_STATE_KEY]` 一致）。"""
        return {
            "phase": self._phase, "route_len": self._waypoint_count,
            "next_wp": self._next_wp, "arrived": self._arrived,
            "handled_stops": sorted(self._handled_stops),
            "stop_index": self._stopctx.index if self._stopctx else -1,
            "stop_block": self._stopctx.block if self._stopctx else "",
            "stop_phase": self._stopctx.phase if self._stopctx else StopPhase.DONE,
            "stop_detail": self._stopctx.detail if self._stopctx else "",
            "dist_to_next_stop": self._dist_to_next_stop,
            "finish": dict(self._finish_info),
            "commands": dict(self._cmd_stats),
        }

    # ------------------------------------------------------------------ #
    #  生命周期
    # ------------------------------------------------------------------ #
    def setup(self) -> None:
        self._write_phase(MissionPhase.IDLE)
        self._log(f"就绪：auto_start={self._auto_start} 延时={self._auto_start_delay_s:g}s "
                  f"复位={self._reset_spec!r} 扫描模式={self._scan_mode or '未设置'}")

    def teardown(self) -> None:
        try:
            self.bridge.publish_fire_gate(False)     # ★ 收尾关闸：绝不留一个开着的放行门
        except Exception:                            # noqa: BLE001
            pass
        try:
            self.bridge.publish_brake()
        except Exception:                            # noqa: BLE001
            pass

    # ------------------------------------------------------------------ #
    #  单拍
    # ------------------------------------------------------------------ #
    def tick(self) -> None:
        now = self._clock()
        self._drain_commands()

        safety: SafetyState | None = self.bb.get(P_SAFETY)
        if safety is not None and not safety.ok:
            self._enter_safe_stop()
            self._publish_state(now)
            return
        if self._phase == MissionPhase.SAFE_STOP:
            self._recover_from_safe_stop()

        self._maybe_auto_start()
        if self._phase == MissionPhase.FINISH:
            pass                                    # ★ 锁存：FINISH 后不再推进（也不重复发事件）
        elif self._phase in _DRIVING_PHASES:
            self._drive()
        elif self._phase == MissionPhase.STOP:
            self._run_stop_point()
        self._check_finish()
        self._publish_state(now)

    # ------------------------------------------------------------------ #
    #  出发
    # ------------------------------------------------------------------ #
    def _maybe_auto_start(self) -> None:
        if not self._auto_start or self._phase != MissionPhase.IDLE:
            return
        if (self._clock() - self._created_at) < self._auto_start_delay_s:
            return
        if self._route is not None:                 # 外部命令已经载入过路线
            return
        try:
            self._reset_belief(self._reset_spec)
            self._load_tokens(self.cfg.get("route.tokens"), source="route.tokens")
        except Exception as exc:                    # noqa: BLE001  配置错就停着，绝不带病发车
            if self._log_throttle.allow("auto_start_failed", self._clock()):
                self._log(f"自动出发失败（保持 IDLE）：{type(exc).__name__}: {exc}")
            return
        self._log(f"自动出发：延时 {self._auto_start_delay_s:g}s 到，复位 {self._reset_spec!r}")
        self._start_mission()

    def _start_mission(self) -> None:
        """路线就绪 → 阶段 DRIVE + 事件 + 持续扫描（见模块 docstring）。"""
        self.bus.publish(EV_MISSION_STARTED, source=self.name,
                         route=len(self._route) if self._route else 0)
        self._write_phase(MissionPhase.DRIVE)
        if self._scan_mode == "continuous":
            self._request_scan(2)                   # ★ 2 = 持续，绝不用 1（会被降级成单趟）
        else:
            self._request_scan(1 if self._scan_mode == "waypoint" else 0)
        self._banner_start()

    def _banner_start(self) -> None:
        route = self._route
        if route is None:
            return
        banner([
            f"【任务开始】路线 {len(route)} 点 / {route.path_length():.2f} m / 穿洞 {route.crossing}",
            f"停站点 {len(self._stop_s)} 个："
            + " ".join(f"{i}:{b}" for i, b in sorted(self._stop_s.items())),
            f"扫描模式 {self._scan_mode or '未设置'}（continuous → scan_request=2）",
        ], log=self._log)

    def _reset_belief(self, spec: str) -> None:
        """`reset:BLOCK,HEAD` / `mission.reset` → 把 belief 设为该格心 + 朝向。"""
        if not spec or not str(spec).strip():
            raise ValueError("复位串为空")
        block, yaw = parse_reset(spec)
        x, y = geo.block_to_meters(block)
        pose = geo.Pose(x, y, yaw)
        self.bb.update(self.name, **{P_POSE: pose})
        self.bb.update(self.name, **{P_ROUTE_INDEX: -1, P_ROUTE_S: 0.0})
        self._last_s = 0.0
        self._route_s = 0.0
        self._log(f"复位 belief：{block} ({x:.3f}, {y:.3f}) yaw={geo.deg(yaw):.1f}°")

    # ------------------------------------------------------------------ #
    #  路线载入
    # ------------------------------------------------------------------ #
    def _load_tokens(self, tokens, *, source: str = "route.tokens") -> int:
        if not tokens:
            raise ValueError(f"{source} 为空，无法载入路线")
        route = parse_mission(tokens, shoot_planner=self._shoot_planner, name=source)
        n = len(route)
        if n < 2:
            raise ValueError(f"路线只有 {n} 个航点（Path 至少需要 2 点）")
        self._install_route(route, source=source)
        self._started_at = self._clock()            # 计时起点 = 路线就绪（终点横幅用它算用时）
        self.bus.publish(EV_ROUTE_LOADED, source=self.name, route=n, blocks=list(route.blocks()))
        self._log(f"路线载入[{source}]：{route.describe()}")
        return n

    def _load_single_point(self, spec: str) -> int:
        """单点命令 `BLOCK,HEAD`（可带 `*` / `:SCAN` / `R:`）。

        ★ 单点**不经过 `parse_mission`**：只有 1 个航点，`Path` 会拒绝编译，
          而且 `parse_mission` 的"朝向后缀只允许出现在最后一个 token"对单点是恒真的，
          直接在末尾航点上落 `yaw` 更简单、也更贴合旧版 `send_waypoint.py` 的语义。
        """
        text = spec.strip()
        if not text:
            raise ValueError("单点命令缺少格名")
        heading = None
        if len(text) >= 2 and text[-2] == ",":
            heading = text[-1].lower()
            if heading not in ("x", "y"):
                raise ValueError(f"朝向后缀只能是 ,x 或 ,y：{spec!r}")
            text = text[:-2].strip()
        info = parse_token(text)
        block = info["block"]
        x, y = geo.block_to_meters(block)
        yaw = geo.heading_token_to_yaw(heading) if heading else geo.HEADING_X
        wp = Waypoint(block=block, x=x, y=y, yaw=yaw, reverse=info["reverse"],
                      shoot=info["shoot"], scan=info["scan"], token=info["token"])
        route = Route((wp,), name="单点命令")
        self._install_route(route, source="单点命令")
        self._started_at = self._clock()
        self.bus.publish(EV_ROUTE_LOADED, source=self.name, route=1, blocks=[block])
        self._log(f"单点命令：{info['token']}（单点不编译 Path；Tracker 侧投影不可用，"
                  f"只用于「停在该点」类调试）")
        return 1

    def _install_route(self, route: Route, *, source: str = "") -> None:
        self._route = route
        self._route_name = source or route.name
        # ★ 单点路线（`单点 BLOCK,HEAD`）只有 1 个航点，而 `Path` 要求 ≥2 点，
        #   所以这里 `_path = None`；推进/终点/停站判定全部对 `_path is None` 做防御。
        self._path = Path(route) if len(route) >= 2 else None
        self._waypoint_count = len(route)
        self._passed = [False] * len(route)
        self._arrived = 0
        self._next_wp = 1
        self._handled_stops = set()
        self._stopctx = None
        self._last_stop_state = StopPointState()
        self._route_s = 0.0
        self._last_s = 0.0
        self._dist_to_next_stop = None
        self._finish_info = {}
        self._stop_s = {}
        if self._path is not None:
            self._stop_s = {i: self._path.s_at_waypoint(i) for i, _ in route.stop_points()}
            if (len(route) - 1) not in self._stop_s:     # 末点一定是一个"停站点"（终点）
                self._stop_s[len(route) - 1] = self._path.total_length
        self._stop_blocks = [route[i].block for i in sorted(self._stop_s)]
        self.bb.update(self.name, **{P_ROUTE: route})
        self.bb.update(self.name, **{P_ROUTE_INDEX: 0, P_ROUTE_S: 0.0})

    # ------------------------------------------------------------------ #
    #  `SHOOT@<靶标>` 的候选链计算
    # ------------------------------------------------------------------ #
    def _shoot_planner(self, target: str) -> list[str]:
        """靶格 → 候选射击停站点（4 邻格中落在射击窗口内的）。

        语法：`SHOOT@G11` 是**指令不是坐标**（`route/parser.py`），由本回调展开成候选链。
        取窗口与候选保留数沿用现场键 `path_executor.shoot_distance_target` /
        `shoot_distance_min` / `shoot_distance_max`（缺省 0.35 / 0.20 / 0.45，与 map.yaml 一致）。
        """
        tx, ty = geo.block_to_meters(target)
        win_lo = float(self.cfg.get("path_executor.shoot_distance_min", 0.20))
        win_hi = float(self.cfg.get("path_executor.shoot_distance_max", 0.45))
        self.cfg.get("path_executor.shoot_distance_target", 0.35)
        keep = int(self.cfg.get("path_executor.shoot_candidates_keep", 3) or 3)

        row_idx, col_idx = geo.block_to_index(target)
        cands: list[tuple[float, str]] = []
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            r, c = row_idx + dr, col_idx + dc
            if not (0 <= r < geo.MAP_ROWS and 0 <= c < geo.MAP_COLS):
                continue
            block = geo.index_to_block(r, c)
            bx, by = geo.block_to_meters(block)
            dist = math.hypot(bx - tx, by - ty)
            if win_lo - _EPS <= dist <= win_hi + _EPS:
                cands.append((dist, block))
        cands.sort(key=lambda item: (item[0], item[1]))
        out = [block for _, block in cands[:keep]]
        return out or [target]                       # 兜底：靶标自身一格（旧实现的降级行为）

    # ------------------------------------------------------------------ #
    #  航点推进
    # ------------------------------------------------------------------ #
    def _current_s(self) -> float:
        """本拍沿程弧长。

        ★ 读 `P_ROUTE_S` 之前**必须验一致性**：该键可能是上一次任务残留的旧值
          （例如刚 `reset`、或本轮 Localization/Tracker 还没写过），
          拿它当"车已经走到哪"会让"到点/到站/终点"判定全部失真。
          验法是把它与"用当前位姿现算的弧长"比对：差得离谱就丢掉、用现算值，
          并把现算值写回（与 Localization 写的是**同一投影语义**，不会互相打架）。
        """
        path = self._path
        if path is None:                            # 单点路线：没有弧长概念
            return self._last_s
        pose = self.bb.get(P_POSE)
        cached = self.bb.get(P_ROUTE_S)
        s: float | None = None
        if pose is not None:
            proj = path.project(pose.x, pose.y, s_hint=self._last_s)
            if isinstance(cached, (int, float)) and abs(float(cached) - proj.s) <= self._arrival_m:
                s = float(cached)                   # 与位姿一致 → 沿用（Localization 的权威值）
            else:
                s = float(proj.s)
                self.bb.update(self.name, **{P_ROUTE_INDEX: int(proj.seg_index), P_ROUTE_S: s})
        elif isinstance(cached, (int, float)):
            s = float(cached)
        if s is None:
            return self._last_s
        self._last_s = s
        self._route_s = s
        return s

    def _advance_waypoints(self) -> None:
        route, path = self._route, self._path
        if route is None or path is None:
            return
        pose = self.bb.get(P_POSE)
        s = self._current_s()
        for i, wp in enumerate(route.waypoints):
            if self._passed[i]:
                continue
            if i == 0:
                self._passed[0] = True           # 起点不报"到达"
                continue
            reached = (s + self._arrival_m) >= path.s_at_waypoint(i)
            if not reached and pose is not None:
                reached = wp.distance_to(pose.x, pose.y) <= self._arrival_m
            if reached:
                self._passed[i] = True
                self._arrived += 1
                self._next_wp = i + 1
                self.bus.publish(EV_WAYPOINT_REACHED, source=self.name, index=i,
                                 block=wp.block, s=round(path.s_at_waypoint(i), 3))
                self._log(f"到点 {i}/{len(route) - 1} {wp.block}"
                          f"（s={path.s_at_waypoint(i):.2f} m）")

    def _next_stop_candidate(self) -> int | None:
        """下一个需要停车的停站点索引；**末点不算停站点**。

        ★ 末点虽然也在 `_stop_s` 里（它是"终点减速/终点判定"的锚），但它由 `FINISH`
          处理，绝不能进三段式停站流程 —— 否则终点会被当成一个打靶点停在那里。
        """
        last = self._waypoint_count - 1
        for idx in sorted(self._stop_s):
            if idx == last:
                continue
            if idx in self._handled_stops:
                continue
            if idx < len(self._passed) and self._passed[idx]:
                continue
            return idx
        return None

    def _dist_to_stop(self, idx: int, pose) -> float | None:
        if pose is None or self._route is None or idx >= len(self._route):
            return None
        wp = self._route[idx]
        return math.hypot(wp.x - pose.x, wp.y - pose.y)

    # ------------------------------------------------------------------ #
    #  驱动
    # ------------------------------------------------------------------ #
    def _drive(self) -> None:
        self._advance_waypoints()
        idx = self._next_stop_candidate()
        if idx is None:
            self._dist_to_next_stop = None
            self._write_phase(MissionPhase.DRIVE)
            return
        pose = self.bb.get(P_POSE)
        dist = self._dist_to_stop(idx, pose)
        self._dist_to_next_stop = dist
        s = self._current_s()
        if self._should_trigger_stop(idx, s, dist):
            self._enter_stop_point(idx, dist)
            return
        approach_m = self._approach_m
        self._write_phase(MissionPhase.APPROACH if (dist is not None and dist <= approach_m)
                          else MissionPhase.DRIVE)

    def _should_trigger_stop(self, idx: int, s: float, dist: float | None) -> bool:
        """到点判据：沿程已过停站点弧长，或几何距离进 `stop_radius_m`（旧版 0.17 m）。

        ★ 弧长与半径**取或**：弧长判据在"路线自交/投影跳段"时可能提前触发，
          半径判据在"车贴着点但投影没跟上"时兜底，两者互补（旧版只有半径 + 开过头补触发）。
        """
        path = self._path
        if path is not None and s + self._stop_radius_m >= self._stop_s.get(idx, math.inf):
            return True
        if dist is not None and dist <= max(self._stop_radius_m, self._arrival_m):
            return True
        return False

    # ------------------------------------------------------------------ #
    #  停站点三段式（每段独立超时，绝不阻塞）
    # ------------------------------------------------------------------ #
    def _enter_stop_point(self, idx: int, dist: float | None) -> None:
        route = self._route
        if route is None:
            return
        wp = route[idx]
        now = self._clock()
        kind = StopKind.SCAN if (wp.scan and not wp.shoot) else StopKind.SHOOT
        shots = int(self.bridge.shoot_count())
        self._stopctx = _StopContext(
            index=idx, block=wp.block, kind=kind, phase=StopPhase.SETTLE,
            started_at=now, phase_started_at=now,
            shots_at_arrival=shots,
            shots_needed=(self._shots_per_stop if kind == StopKind.SHOOT else 0),
            detail=f"到站（距离 {dist:.3f} m，累计发数 {shots}）" if dist is not None
                   else f"到站（累计发数 {shots}）",
        )
        self._write_phase(MissionPhase.STOP)
        self._publish_stop_state(now)
        self.bus.publish(EV_ARRIVED_STOP_POINT, source=self.name, index=idx, block=wp.block,
                         kind=kind, shots=shots)
        self._log(f"★ 停站点 {idx} {wp.block}（{kind}）{self._stopctx.detail}"
                  f"；三段超时 SETTLE>{self._settle_grace_s:.1f}s "
                  f"ACQUIRE>{self._lock_timeout_s:.1f}s FIRE>{self._shot_timeout_s:.1f}s")

    def _run_stop_point(self) -> None:
        stop = self._stopctx
        if stop is None:                            # 阶段与上下文不一致 → 安全回落
            self._write_phase(MissionPhase.DRIVE)
            return
        now = self._clock()
        if stop.kind == StopKind.SCAN:
            self._run_scan_stop(stop, now)
        elif stop.phase == StopPhase.SETTLE:
            self._phase_settle(stop, now)
        elif stop.phase == StopPhase.ACQUIRE:
            self._phase_acquire(stop, now)
        elif stop.phase == StopPhase.FIRE:
            self._phase_fire(stop, now)
        else:
            self._abandon_stop(stop, f"未知阶段 {stop.phase}")
        self._publish_stop_state(now)

    # ---- 段 1：SETTLE ----
    def _phase_settle(self, stop: _StopContext, now: float) -> None:
        pose = self.bb.get(P_POSE)
        moved = None
        if pose is not None and stop.last_x is not None and stop.last_y is not None:
            moved = math.hypot(pose.x - stop.last_x, pose.y - stop.last_y)
        if pose is not None:
            stop.last_x, stop.last_y = pose.x, pose.y

        speed = self._last_speed()
        # 停稳 = 位置连续不变 **或** 速度近零（两者任一成立就武装，避免过站慢速时白等）
        if (moved is not None and moved <= self._settle_eps_m) or \
                (speed is not None and abs(speed) <= self._settle_speed_mps):
            if stop.settle_armed_at is None:
                stop.settle_armed_at = now
                stop.detail = f"停稳计时开始（Δ={moved if moved is not None else float('nan'):.4f} m，"
                stop.detail += f"v={speed if speed is not None else float('nan'):.3f} m/s）"

        if stop.settle_armed_at is not None:
            if (now - stop.settle_armed_at) >= self._settle_time_s:
                self._goto_phase(stop, StopPhase.ACQUIRE, now,
                                 f"停稳 {(now - stop.settle_armed_at):.2f}s"
                                 f" ≥ {self._settle_time_s:.2f}s")
                return
        if (now - stop.phase_started_at) >= self._settle_grace_s:
            self._goto_phase(stop, StopPhase.ACQUIRE, now,
                             f"停稳超时 {(now - stop.phase_started_at):.2f}s"
                             f" ≥ {self._settle_grace_s:.2f}s → 降级放行")

    # ---- 段 2：ACQUIRE ----
    def _phase_acquire(self, stop: _StopContext, now: float) -> None:
        if stop.kind == StopKind.SCAN:
            self._goto_phase(stop, StopPhase.FIRE, now, "扫描停站：跳过锁定")
            return
        status = int(self.bridge.aim_status())
        obs = self.bridge.vision(0.5)
        has_target = bool(obs is not None and getattr(obs, "has_target", False))
        if status == AimStatus.TRACK and has_target:
            self._goto_phase(stop, StopPhase.FIRE, now, "云台 TRACK 且有目标")
            return
        if (now - stop.phase_started_at) >= self._lock_timeout_s:
            self._abandon_stop(stop, f"锁定超时 {(now - stop.phase_started_at):.2f}s"
                                     f" ≥ {self._lock_timeout_s:.2f}s"
                                     f"（aim={AimStatus.name(status)} 目标={has_target}）")

    # ---- 段 3：FIRE ----
    def _phase_fire(self, stop: _StopContext, now: float) -> None:
        if stop.kind == StopKind.SCAN:
            if (now - stop.phase_started_at) >= self._scan_stop_timeout_s:
                self._complete_stop(stop, f"扫描停站完成（{self._scan_stop_timeout_s:.1f}s）")
            return
        self._set_fire_gate(True)                   # ★ 放行门：进本段即打开（幂等）
        delta = int(self.bridge.shoot_count()) - stop.shots_at_arrival
        if delta >= stop.shots_needed:
            self._complete_stop(stop, f"发数增量 {delta} ≥ {stop.shots_needed} → 命中窗口结束")
            return
        if (now - stop.phase_started_at) >= self._shot_timeout_s:
            self._set_fire_gate(False)
            self._abandon_stop(stop, f"开火超时 {(now - stop.phase_started_at):.2f}s"
                                     f" ≥ {self._shot_timeout_s:.2f}s（增量 {delta}"
                                     f"/{stop.shots_needed}）")

    # ---- 扫描停站（`F2:SCAN_STOP`）：不停顿打靶，只保证到位停稳 ----
    def _run_scan_stop(self, stop: _StopContext, now: float) -> None:
        if stop.phase == StopPhase.SETTLE:
            self._phase_settle(stop, now)
            return
        if stop.phase in (StopPhase.ACQUIRE, StopPhase.FIRE):
            if stop.phase == StopPhase.ACQUIRE:
                stop.phase = StopPhase.FIRE
                stop.phase_started_at = now
            if (now - stop.phase_started_at) >= self._scan_stop_timeout_s:
                self._complete_stop(stop, f"扫描停站完成（{self._scan_stop_timeout_s:.1f}s）")
            return
        self._abandon_stop(stop, f"未知阶段 {stop.phase}")

    # ---- 三段收尾 ----
    def _goto_phase(self, stop: _StopContext, phase: str, now: float, detail: str) -> None:
        self._set_fire_gate(False)                  # ★ 每段结束都关闸（旧版 2181–2183 的纪律）
        stop.phase = phase
        stop.phase_started_at = now
        stop.settle_armed_at = None
        stop.detail = detail
        self._log(f"停站点 {stop.index} {stop.block}：{detail} → {phase}")

    def _complete_stop(self, stop: _StopContext, detail: str) -> None:
        self._set_fire_gate(False)
        stop.detail = detail
        self._log(f"停站点 {stop.index} {stop.block} 完成：{detail}")
        self._close_stop_point(stop)

    def _abandon_stop(self, stop: _StopContext, detail: str) -> None:
        self._set_fire_gate(False)
        stop.detail = detail
        self._log(f"停站点 {stop.index} {stop.block} 放弃：{detail}（放行下一个候选）")
        self._close_stop_point(stop)

    def _close_stop_point(self, stop: _StopContext) -> None:
        stop.phase = StopPhase.DONE
        self._handled_stops.add(stop.index)
        self._last_stop_state = self._build_stop_state(stop)
        self.bus.publish(EV_STOP_POINT_DONE, source=self.name, index=stop.index,
                         block=stop.block, kind=stop.kind, detail=stop.detail)
        self._stopctx = None
        self._write_phase(MissionPhase.DRIVE)

    def _publish_stop_state(self, now: float) -> None:
        stop = self._stopctx
        state = self._build_stop_state(stop) if stop is not None else self._last_stop_state
        self.bb.set(P_STOP_POINT, state, source=self.name)

    def _build_stop_state(self, stop: _StopContext) -> StopPointState:
        return StopPointState(
            index=stop.index, block=stop.block, kind=stop.kind, phase=stop.phase,
            started_at=stop.started_at, phase_started_at=stop.phase_started_at,
            shots_at_arrival=stop.shots_at_arrival, shots_needed=stop.shots_needed,
            detail=stop.detail,
        )

    def _last_speed(self) -> float | None:
        """读 Tracker 最近一拍的速度（只读遥测键，不跨任务调用）。"""
        snap = self.bb.get(P_TELEMETRY)
        if not isinstance(snap, dict):
            return None
        cmd = snap.get("tracker_cmd")
        if not isinstance(cmd, dict):
            return None
        value = cmd.get("speed")
        return float(value) if isinstance(value, (int, float)) else None

    # ------------------------------------------------------------------ #
    #  放行门 / 扫描 / 阶段 / 遥测
    # ------------------------------------------------------------------ #
    def _set_fire_gate(self, allow: bool) -> None:
        """放行门是**锁存式**的：只在变化时发（避免 10 Hz 刷屏，同时保证状态可追）。"""
        allow = bool(allow)
        if allow == self._fire_gate_state:
            return
        self.bridge.publish_fire_gate(allow)
        self._fire_gate_state = allow

    def _request_scan(self, mode: int) -> None:
        mode = int(mode)
        if mode == 1 and 2 in self._scans:
            # ★ readme【十】31：持续扫描期间绝不能再发 1（会被降级成单趟）
            if self._log_throttle.allow("scan_downgrade", self._clock()):
                self._log("拒绝 scan_request=1：当前是持续扫描（2），单趟会把它降级")
            return
        self.bridge.publish_scan_request(mode)
        self._scans.append(mode)

    def _write_phase(self, phase: str) -> None:
        if phase == self._phase:
            return
        old, self._phase = self._phase, phase
        self.bb.set(P_MISSION_PHASE, phase, source=self.name)
        if self._log_throttle.allow(f"phase:{old}->{phase}", self._clock()):
            self._log(f"阶段 {old} → {phase}")

    def _enter_safe_stop(self) -> None:
        reasons = self.bb.get(P_SAFETY)
        if self._phase != MissionPhase.SAFE_STOP:
            self._set_fire_gate(False)
            self._log(f"安全接管：{getattr(reasons, 'reasons', ())} → SAFE_STOP")
        self._write_phase(MissionPhase.SAFE_STOP)

    def _recover_from_safe_stop(self) -> None:
        """安全恢复后回到 DRIVE（停站点上下文若还在就继续那一段，不重来）。"""
        if self._stopctx is not None and self._stopctx.active:
            self._write_phase(MissionPhase.STOP)
        else:
            self._write_phase(MissionPhase.DRIVE)
        self._log("安全恢复 → 继续任务")

    def _publish_state(self, now: float) -> None:
        snap = self.bb.get(P_TELEMETRY)
        data = dict(snap) if isinstance(snap, dict) else {}
        data[MISSION_STATE_KEY] = self.state()
        self.bb.set(P_TELEMETRY, data, source=self.name)
        self._publish_stop_state(now)

    # ------------------------------------------------------------------ #
    #  终点（锁存）
    # ------------------------------------------------------------------ #
    def _check_finish(self) -> None:
        route, path = self._route, self._path
        if route is None or path is None or self._finish_latched:
            return
        if self._phase in (MissionPhase.IDLE, MissionPhase.SAFE_STOP):
            return
        last = len(route) - 1
        self._advance_waypoints()                   # 先补最后一次推进（末点也要报到达）
        s = self._current_s()
        if not (self._passed[last] or s + self._arrival_m >= path.s_at_waypoint(last)):
            return

        self._finish_latched = True
        elapsed = (self._clock() - self._started_at) if self._started_at is not None else 0.0
        drift = 0.0
        fix = self.bb.get("fix")
        if fix is not None:
            drift = float(getattr(fix, "max_drift_m", 0.0) or 0.0)
        shots = int(self.bridge.shoot_count())
        hits = int(self.bridge.take_hits())
        self._finish_info = {"elapsed_s": round(elapsed, 2), "shots": shots, "hits": hits,
                             "drift_m": round(drift, 3), "waypoints": len(route)}
        self._write_phase(MissionPhase.FINISH)
        self.bridge.publish_brake()
        self.bridge.publish_scan_request(0)         # 0 = 停扫
        self._scans.append(0)
        if not self._finish_done:
            self._finish_done = True
            self.bus.publish(EV_MISSION_FINISHED, source=self.name, **self._finish_info)
            banner([
                "【任务结束】" + (f"路线 {len(route)} 点 / {route.path_length():.2f} m"),
                f"用时 {elapsed:.1f} s（{self._fmt_duration(elapsed)}）",
                f"发数 {shots} / 命中 {hits} / 漂移 {drift:.3f} m",
                f"停站点 {len(self._stop_s)} 个（完成 {len(self._handled_stops)}）",
            ], log=self._log)

    @staticmethod
    def _fmt_duration(seconds: float) -> str:
        seconds = max(0.0, float(seconds))
        return f"{int(seconds // 60)}分{seconds % 60:.1f}秒"

    # ------------------------------------------------------------------ #
    #  外部命令（/nav/waypoint）：未知命令只告警不崩
    # ------------------------------------------------------------------ #
    def _drain_commands(self) -> None:
        try:
            commands = self.bridge.take_waypoint_commands(8)
        except Exception as exc:                    # noqa: BLE001  读命令失败不该杀任务
            if self._log_throttle.allow("cmd_read_failed", self._clock()):
                self._log(f"读取外部命令失败（忽略本拍）：{type(exc).__name__}: {exc}")
            return
        for raw in commands or []:
            self._cmd_stats["total"] += 1
            try:
                self._handle_command(raw)
            except Exception as exc:                # noqa: BLE001  ★ 一条坏命令不许崩
                self._cmd_stats["rejected"] += 1
                self._log(f"⚠️ 外部命令无效（已忽略）{raw!r}：{type(exc).__name__}: {exc}")

    def _handle_command(self, raw: Any) -> None:
        text = str(raw or "").strip()
        if not text:
            raise ValueError("空命令")
        low = text.lower()

        if low.startswith("reset:"):
            self._reset_belief(text[len("reset:"):].strip())
            self._clear_route("reset 命令")
            self._write_phase(MissionPhase.IDLE)    # 复位后回到 IDLE，等下一单点/多点/自动出发
            return

        if low.startswith("load:"):
            spec = text[len("load:"):].strip()
            if not spec:
                raise ValueError("load: 后面缺少文件名")
            self._load_tokens(spec, source=f"load:{spec}")
            self._start_mission()
            return

        if low.startswith("shoot@"):
            raise ValueError("SHOOT@ 只能作为路线 token 出现，不能单独下发给 /nav/waypoint")

        # `单点 BLOCK,HEAD` → 单点路线（显式前缀，便于日志区分）
        for prefix in ("单点", "single", "goto"):
            if low == prefix or low.startswith(prefix + " ") or low.startswith(prefix + ":"):
                rest = text[len(prefix):].lstrip(" :").strip()
                self._load_single_point(rest)
                self._start_mission()
                return

        # `多点 ...` → 多航点路线（前缀只为可读性，内容与裸 token 列表一致）
        for prefix in ("多点", "multi", "route"):
            if low == prefix or low.startswith(prefix + " ") or low.startswith(prefix + ":"):
                rest = text[len(prefix):].lstrip(" :").strip()
                if not rest:
                    raise ValueError(f"{prefix} 后面缺少航点")
                self._load_tokens(self._normalize_command_tokens(rest), source="多点命令")
                self._start_mission()
                return

        tokens = self._normalize_command_tokens(text)
        if len(tokens) == 1:
            self._load_single_point(tokens[0])      # 裸 `BLOCK,HEAD` 也当单点
            self._start_mission()
            return
        self._load_tokens(tokens, source="waypoint命令")   # 裸多航点
        self._start_mission()

    @staticmethod
    def _normalize_command_tokens(text: str) -> list[str]:
        """`BLOCK,HEAD` / `A B C,y`（空格/分号/竖线分隔）→ token 列表。

        旧版 `waypoint_callback` 兼容 `BLOCK,HEAD`（逗号分隔）与空格分隔的串。
        朝向后缀 `,x|,y` 只允许挂在**最后一个** token 上（`parse_mission` 的硬约束），
        所以这里把误写在中间的裸 `x`/`y` 挪到最后，而不是直接报错。
        """
        body = text.replace(";", " ").replace("|", " ").strip()
        pieces = [p.strip() for p in body.replace(",", " ").split() if p.strip()]
        if not pieces:
            raise ValueError("命令里没有可解析的 token")
        tokens = pieces
        heads = {"x", "y"}
        moved: list[str] = []
        for tok in tokens:
            low = tok.lower()
            if low in heads:
                moved.append(low)
        if moved:
            tokens = [t for t in tokens if t.lower() not in heads]
            if not tokens:
                raise ValueError("只有朝向 token，缺少格名")
            tokens = tokens[:-1] + [tokens[-1] + "," + moved[-1]]
        return tokens

    def _clear_route(self, why: str) -> None:
        self._route = None
        self._path = None
        self._waypoint_count = 0
        self._passed = []
        self._handled_stops = set()
        self._stopctx = None
        self._last_stop_state = StopPointState()
        self._stop_s = {}
        self._stop_blocks = []
        self._dist_to_next_stop = None
        self._finish_latched = False
        self._finish_done = False
        self._finish_info = {}
        self._last_s = 0.0
        self._route_s = 0.0
        self._set_fire_gate(False)
        self.bb.set(P_ROUTE, None, source=self.name)
        self.bb.update(self.name, **{P_ROUTE_INDEX: -1, P_ROUTE_S: 0.0})
        self._log(f"清空路线（{why}）")
