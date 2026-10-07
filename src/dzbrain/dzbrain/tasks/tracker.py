# -*- coding: utf-8 -*-
"""TrackerTask（50 Hz）—— 循迹：纯追踪 → `pursuitAngle`（底盘指令的唯一写入者）。

对应 `docs/P1接口约定.md` §3.2，旧实现血统 `path_executor.control_loop`（survey §4.2/§6.1）。

设计要点（逐条对齐约定与 survey）：

1. **前置门**：无位姿 / 安全非 OK / 遥控接管 / 阶段不允许 / 无路线 → `publish_brake()` 后 return。
   旧实现在"无定位"时整拍直接 return、**连零 Twist 都不发**（survey §4.2 第 1 条），
   而底盘有 1 s 看门狗：不发就是等它自己停 —— 新版一律显式刹车，且原因限流打一条。
2. **桥洞直行保护按航段索引**（survey §6.1，★ 不要用"离 K 列近"的几何判据）：
   旧版早期按几何距离判，导致 K 列附近 ±2 格全域锁航向 + 限速（"循迹僵直"）。
   这里复刻段级预计算：保护点 → 航点索引 → 航段集合 `[h-before, h+after]`，
   **集合只在路线变化时重建**（`_ensure_align_segments`），每拍只做一次集合查询。
3. **倒车段**：`path.segment_is_reverse(idx)` → `compute_command(reverse=True)`，
   `publish_motion_state(True/False)` **仅状态变化时发**（与旧版同语义）。
4. **限速**：接近未完成停站点 < `pursuit.approach_m` 时按 `stop_approach_speed_gain` 降速 ——
   这一段由 `pursuit.controller.speed_profile(dist_to_stop=...)` 实现，
   而 `_dist_to_stop()` 只遍历 `path.stop_indices` 且跳过 `i <= proj.seg_index` 的点，
   即"**未完成**停站点"语义天然成立；本任务只负责把正确的投影传进去。
5. **Path 缓存**：`Path(route)` 一次编译、路线变化才重建。旧实现每拍多次 O(N) 全扫
   （survey §8.1）；`Path.project(s_hint=...)` 的窗口搜索把每拍代价压到 O(窗口)。

★ 时间来源一律走构造注入的 `clock`（默认真实 `time.monotonic`）：限流日志、舵角速率限幅
   都依赖它，测试里用 `FakeClock` 快进即可，**任务内不得直接调 `time.time()`**。

⚠️ 与 Localization 的写入分工：`P_ROUTE_INDEX/P_ROUTE_S` 两边都可能写。约定是
   "**谁写了就写同一个投影结果**"——本任务读旧值只作为 `s_hint`（搜索窗口提示），
   不用它做控制决策；控制决策一律用本拍自己算出来的投影。
"""

from __future__ import annotations

import math
from typing import Any, Callable

from ..core import geometry as geo
from ..core.blackboard import (P_MISSION_PHASE, P_POSE, P_ROUTE, P_ROUTE_INDEX, P_ROUTE_S,
                               P_SAFETY, P_TELEMETRY, Blackboard)
from ..core.config import Config, lookahead_distance
from ..core.events import EventBus
from ..core.logging import Throttle
from ..core.state import SafetyState
from ..core.task import TaskThread
from ..io.contract import MissionPhase
from ..pursuit.controller import PursuitCommand, compute_command
from ..pursuit.path import Path, Projection
from ..route.model import Route

__all__ = ["TrackerTask", "TRACKER_CMD_KEY", "ACTIVE_PHASES", "BLOCKED_PHASES"]

#: 遥测键：Track 最近一拍的控制指令与判定原因（供 Safety/遥测/测试断言）。
#: `P_TELEMETRY` 的约定值是 dict，本任务只往里塞这一个子键，不覆盖别的任务的内容。
TRACKER_CMD_KEY = "tracker_cmd"

#: 允许循迹的阶段（★ 白名单而不是黑名单：将来新增的阶段默认**不动车**，失败偏安全）
ACTIVE_PHASES: tuple[str, ...] = (MissionPhase.DRIVE, MissionPhase.APPROACH)

#: 明令刹车的阶段（`P1接口约定.md` §3.2 列出的六个）
BLOCKED_PHASES: tuple[str, ...] = (MissionPhase.IDLE, MissionPhase.STOP, MissionPhase.SHOOT,
                                   MissionPhase.WAIT_SCAN, MissionPhase.FINISH,
                                   MissionPhase.SAFE_STOP)

_EPS = 1e-9


class TrackerTask(TaskThread):
    """循迹任务（50 Hz）。"""

    def __init__(self, bridge, bb: Blackboard, bus: EventBus, cfg: Config, *,
                 log: Callable[[str], None] = print, group=None,
                 clock: Callable[[], float] | None = None) -> None:
        super().__init__("TrackerTask", 50.0, bus=bus, blackboard=bb, logger=log)
        self.bridge = bridge
        self.cfg = cfg
        self.group = group
        if clock is None:
            import time as _time
            clock = _time.monotonic
        self._clock = clock

        # ---- 配置（一次性读出，避免 50 Hz 下反复查表） ----
        self._rate_limit_s = float(cfg.get("logging.rate_limit_s") or 1.0)
        self._align_kp = float(cfg.get("pursuit.align_kp"))
        self._align_steer_limit_deg = float(cfg.get("pursuit.align_steer_limit_deg"))
        self._align_speed_mps = float(cfg.get("pursuit.align_straight_speed_mps"))
        self._align_before = float(cfg.get("pursuit.align_straight_before") or 0.0)
        self._align_after = float(cfg.get("pursuit.align_straight_after") or 0.0)
        self._bridge_blocks = list(cfg.get("pursuit.bridge_blocks") or [])
        self._max_steer_deg = float(cfg.get("pursuit.max_steer_deg"))
        self._min_speed_mps = float(cfg.get("pursuit.min_speed_mps"))
        self._cruise_mps = float(cfg.get("pursuit.cruise_speed_mps"))
        self._tick_dt = 1.0 / self.rate_hz
        self._window_m = max(0.0, float(cfg.get("pursuit.approach_m")))

        # ---- 里程态 ----
        self._path: Path | None = None
        self._route_id: int | None = None
        self._seg_s: float | None = None            # 上一拍投影弧长（下一拍 s_hint）
        self._last_steer_deg: float | None = None
        self._last_tick_at: float | None = None
        self._reverse_state: bool | None = None     # ★ 仅变化时发 motion_state
        self._align_segs: frozenset[int] = frozenset()
        self._align_build_key: tuple | None = None
        self._last_cmd: dict[str, Any] = {}
        self._blocked_reason: str = ""
        self._log_throttle = Throttle(self._rate_limit_s)

    # ------------------------------------------------------------------ #
    #  只读接口（测试/遥测）
    # ------------------------------------------------------------------ #
    @property
    def path(self) -> Path | None:
        """当前缓存的 `Path`（★ 测试用它断言"没有被每拍重建"）。"""
        return self._path

    @property
    def last_align_segments(self) -> frozenset[int]:
        """当前桥洞保护航段集合（预计算结果，路线变化才重建）。"""
        return self._align_segs

    def telemetry(self) -> dict[str, Any]:
        """最近一拍的判定细节（与写进 `P_TELEMETRY[TRACKER_CMD_KEY]` 的内容一致）。"""
        return dict(self._last_cmd)

    # ------------------------------------------------------------------ #
    #  桥洞保护：航段索引集合（survey §6.1）
    # ------------------------------------------------------------------ #
    def _ensure_align_segments(self, route: Route) -> None:
        """路线变化时重建保护航段集合；同一路线不重复计算（50 Hz 下必须廉价）。

        ★ 判据是**航段索引**不是几何距离：只有真正穿洞的那几段受保护，
          K 列附近 ±2 格的其他段照常走纯追踪。
        """
        key = (id(route), len(route))
        if key == self._align_build_key:
            return
        self._align_build_key = key

        if not self._bridge_blocks or len(route) < 2:
            self._align_segs = frozenset()
            return

        # 单位换算：`align_straight_before/after` 的单位是**格**
        # （旧实现 `tol = 0.35 × 0.4`，survey §6.1 的 1938 行），取整时减 EPS 防止 2.0 → 3。
        before = max(0, int(math.ceil(self._align_before - _EPS)))
        after = max(0, int(math.ceil(self._align_after - _EPS)))
        last_seg = len(route) - 2                       # 段索引上限

        hits: set[int] = set()
        for i, wp in enumerate(route.waypoints):
            if wp.block in self._bridge_blocks:
                hits.add(i)
        segs: set[int] = set()
        for h in hits:
            for seg in range(h - before, h + after + 1):
                if 0 <= seg <= last_seg:
                    segs.add(seg)
        self._align_segs = frozenset(segs)

    def _align_active(self, seg_index: int) -> bool:
        return seg_index in self._align_segs

    # ------------------------------------------------------------------ #
    #  资源
    # ------------------------------------------------------------------ #
    def _resolve_path(self, route) -> Path:
        """取/建缓存 Path。接受 `Route`（约定）或已编译的 `Path`（Localization 可能先写过）。"""
        if isinstance(route, Path):
            path = route
            route_obj = route.route
        else:
            route_obj = route
            path = self._path if (self._path is not None and self._route_id == id(route_obj)) else None
            if path is None:
                path = Path(route_obj)
                self._path = path
                self._route_id = id(route_obj)
        self._ensure_align_segments(route_obj)
        return path

    # ------------------------------------------------------------------ #
    #  前置门
    # ------------------------------------------------------------------ #
    def _blocked_reason_now(self, pose, safety: SafetyState | None, phase: str | None,
                            route) -> str:
        """返回拦停原因（空串 = 可以动）。顺序 = 严重度顺序，便于日志定位主因。"""
        if pose is None:
            return "无位姿（P_POSE 缺失）"
        if safety is None:
            return "无安全状态（P_SAFETY 缺失，按不安全处理）"
        if not safety.ok:
            return "安全不 OK：" + ("," .join(safety.reasons) or "未给原因")
        if self.bridge.manual_override():
            return "遥控接管（manual_override）"
        if phase not in ACTIVE_PHASES:
            return f"阶段 {phase or '未设置'} 不允许循迹"
        if route is None or len(route) == 0:
            return "无路线（P_ROUTE 缺失）"
        return ""

    def _hold(self, reason: str) -> None:
        """刹车 + 限流记录原因（重复原因不刷屏，但每次变化立即打）。"""
        self._blocked_reason = reason
        self.bridge.publish_brake()
        self._last_cmd = {
            "blocked": True, "reason": reason, "speed": 0.0, "steer_deg": 0.0,
            "align": False, "reverse": False, "seg_index": -1, "s": 0.0,
            "dist_to_path": 0.0, "gate": "blocked", "at": self._clock(),
        }
        self._write_telemetry()
        if self._log_throttle.allow("blocked:" + reason, self._clock()):
            self._log(f"刹车：{reason}")

    # ------------------------------------------------------------------ #
    #  单拍
    # ------------------------------------------------------------------ #
    def tick(self) -> None:
        now = self._clock()
        dt = self._tick_dt if self._last_tick_at is None else (now - self._last_tick_at)
        self._last_tick_at = now
        if not (0.0 < dt <= 1.0):          # 时钟跳变/首拍 → 退回标称周期（速率限幅不做傻事）
            dt = self._tick_dt

        pose = self.bb.get(P_POSE)
        safety: SafetyState | None = self.bb.get(P_SAFETY)
        phase = self.bb.get(P_MISSION_PHASE)
        route = self.bb.get(P_ROUTE)

        reason = self._blocked_reason_now(pose, safety, phase, route)
        if reason:
            self._hold(reason)
            return

        path = self._resolve_path(route)

        # s_hint：优先本任务上一拍的值，其次 Localization 写进黑盒的值（同一投影语义）
        s_hint = self._seg_s
        if s_hint is None:
            prev = self.bb.get(P_ROUTE_S)
            s_hint = float(prev) if isinstance(prev, (int, float)) else None

        proj: Projection = path.project(pose.x, pose.y, s_hint=s_hint, window_m=self._window_m)
        self._seg_s = proj.s

        is_rev = path.segment_is_reverse(proj.seg_index)
        cmd: PursuitCommand = compute_command(pose, path, proj, self.cfg,
                                              reverse=is_rev,
                                              last_steer_deg=self._last_steer_deg,
                                              dt=dt)
        align = self._align_active(proj.seg_index)

        if align:
            # ---- 桥洞直行：关掉纯追踪，只做航向保持（survey §6.1 的 2480–2481 复刻） ----
            seg_heading = path.heading_at(proj.s)
            err_deg = geo.deg(geo.ang_diff(seg_heading, pose.yaw))
            steer = err_deg * self._align_kp / 10.0
            lim = self._align_steer_limit_deg
            steer = lim if steer > lim else (-lim if steer < -lim else steer)
            speed = self._align_speed_mps
            if is_rev:
                speed = -speed                     # ★ 倒车段桥洞直行同样取负（旧版同语义）
            reason_tag = "align"
        else:
            steer = cmd.steer_deg
            speed = cmd.linear_x
            reason_tag = cmd.speed_reason

        self._last_steer_deg = steer
        self.bridge.publish_command(speed, steer)

        # ---- 倒车状态：仅变化时发（旧版 1427–1435 同语义） ----
        if self._reverse_state is None or self._reverse_state != is_rev:
            self.bridge.publish_motion_state(is_rev)
            self._reverse_state = is_rev

        # ---- 路线位置回写：与 Localization 写同一个投影结果（不互相打架） ----
        self.bb.update("TrackerTask", **{P_ROUTE_INDEX: int(proj.seg_index),
                                         P_ROUTE_S: float(proj.s)})

        self._blocked_reason = ""
        self._last_cmd = {
            "blocked": False, "reason": "", "gate": "run",
            "speed": float(speed), "steer_deg": float(steer),
            "align": bool(align), "reverse": bool(is_rev),
            "seg_index": int(proj.seg_index), "s": float(proj.s),
            "dist_to_path": float(proj.dist), "curvature": float(cmd.curvature),
            "speed_reason": cmd.speed_reason, "reason_tag": reason_tag,
            "lookahead_m": float(lookahead_distance(self._cruise_mps, self.cfg)),
            "align_segments": sorted(self._align_segs),
            "at": now,
        }
        self._write_telemetry()

    def _write_telemetry(self) -> None:
        snap = self.bb.get(P_TELEMETRY)
        data = dict(snap) if isinstance(snap, dict) else {}
        data[TRACKER_CMD_KEY] = dict(self._last_cmd)
        self.bb.set(P_TELEMETRY, data, source="TrackerTask")

    # ------------------------------------------------------------------ #
    #  生命周期
    # ------------------------------------------------------------------ #
    def teardown(self) -> None:
        """退出时发零指令（零 Twist = 刹车）。"""
        try:
            self.bridge.publish_brake()
        except Exception:                                  # noqa: BLE001  收尾不许抛
            pass
