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

__all__ = ["TrackerTask", "TRACKER_CMD_KEY", "ACTIVE_PHASES", "BLOCKED_PHASES",
           "PIVOT_DONE_KEY", "PivotPhase"]


class PivotPhase:
    """PIVOT 原地转向的三段相位（并入 10-05 核心的已验证修复）。

    IDLE   : 未在 PIVOT（走纯追踪 / 桥洞直行）
    SETTLE : 已在顶点前触发，先刹车 `pursuit.pivot_wait_s` 让底盘惯性消失
    TURN   : 以 `pursuit.pivot_speed_mps` 蠕动、舵角指向目标航向，直到误差进入
             `pursuit.pivot_yaw_tol_deg`（或 `pursuit.pivot_timeout_s` 超时放行）
    """

    IDLE = "IDLE"
    SETTLE = "SETTLE"
    TURN = "TURN"

    ALL = (IDLE, SETTLE, TURN)

#: 遥测键：Track 最近一拍的控制指令与判定原因（供 Safety/遥测/测试断言）。
#: `P_TELEMETRY` 的约定值是 dict，本任务只往里塞这一个子键，不覆盖别的任务的内容。
TRACKER_CMD_KEY = "tracker_cmd"

#: ★PIVOT 完成/放弃通知键（Tracker 写、MissionTask 读）：值为**不可变三元组**
#:   `(顶点航点索引, 顶点弧长, 结局)`，结局 ∈ {"done", "abandon"}。
#: 为什么需要（bench 实测 E11/A6/B6）：`pursuit.pivot_dist_m`(0.25) > `pursuit.arrival_m`
#: (0.12) —— 车在距顶点 0.25 m 处原地掉头，掉头完成后**原路返回**，既不进 0.12 m 的
#: 到达圈、沿程弧长也永远不越过顶点 ⇒ MissionTask 的到点判定恒假 ⇒ 拐角绕圈兜底
#: 12 s 后把它当"不可达"强制跳过。PIVOT 触发本身已保证"车在顶点 pivot_dist_m 内、
#: 且掉头已完成（或超时放弃）"，把这个事实通知 MissionTask 判到达才是诚实语义。
#: MissionTask 在路线载入/清空时把该键复位为 None，防止跨轮次残留误判。
PIVOT_DONE_KEY = "pivot_done"

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
        self._max_steer_rate_deg_s = float(cfg.get("pursuit.max_steer_rate_deg_s"))
        self._min_speed_mps = float(cfg.get("pursuit.min_speed_mps"))
        self._cruise_mps = float(cfg.get("pursuit.cruise_speed_mps"))
        self._tick_dt = 1.0 / self.rate_hz
        self._window_m = max(0.0, float(cfg.get("pursuit.approach_m")))

        # ---- 里程态 ----
        self._path: Path | None = None
        self._route_id: int | None = None
        self._seg_s: float | None = None            # 上一拍投影弧长（下一拍 s_hint）
        self._last_steer_deg: float | None = None
        # ⚠️ 曾经有个 `_last_linear_x`（上一拍**命令**的线速度）被用来猜"行进方向"。
        #    那是个自指环：命令符号 → 选哪条重叠段 → is_reverse → 命令符号，
        #    在折返压线处每拍翻转（bench 实测 601 次）。现在行进方向取**实测位移**
        #    （见 `_tick` 里的长注释），该字段已删除 —— 别再加回来。
        self._last_tick_at: float | None = None
        self._reverse_state: bool | None = None     # ★ 仅变化时发 motion_state
        self._align_segs: frozenset[int] = frozenset()
        self._align_build_key: tuple | None = None

        # ---- ★PIVOT 原地转向（并入 10-05 核心的已验证修复）----
        #   纯追踪在大角度换向（现场图 F11 的 180° 折返）会画大弧：仿真实测 0.41 m 横向偏差，
        #   而那里是压雷段 —— 偏差 0.4 m 就可能漏压雷。参数扫描证明调参无解（结构性）。
        self._pivot_enabled = bool(cfg.get("pursuit.pivot_enabled"))
        self._pivot_angle_deg = float(cfg.get("pursuit.pivot_angle_deg"))
        self._pivot_dist_m = float(cfg.get("pursuit.pivot_dist_m"))
        self._pivot_wait_s = float(cfg.get("pursuit.pivot_wait_s"))
        self._pivot_speed_mps = float(cfg.get("pursuit.pivot_speed_mps"))
        self._pivot_yaw_tol_deg = float(cfg.get("pursuit.pivot_yaw_tol_deg"))
        self._pivot_timeout_s = float(cfg.get("pursuit.pivot_timeout_s"))  # ★旧版没有超时
        self._pivot_kp = float(cfg.get("pursuit.pivot_kp"))
        #: ★行进方向裁决：用**实际位移**算，位移小于门限时锁存上次可信值。
        #: 绝不能用"上一拍命令的速度符号"——那是自指环，见 `_tick` 里的长注释。
        self._motion_eps_m = float(cfg.get("pursuit.motion_heading_eps_m"))
        self._travel_heading: float | None = None      # 锁存的"车正在往哪边走"（地图系）
        self._last_motion_xy: tuple[float, float] | None = None
        #: ★投影单拍最大回退量：折返压线时两条共线段"等距、s 相差一整段"，
        #: 不限住就会在 0.0↔0.70 之间倒带（见 pursuit/path.py 的 back_m 说明）。
        self._rewind_m = float(cfg.get("pursuit.projection_rewind_m"))
        self._reverse_entry_m = float(cfg.get("pursuit.reverse_entry_m"))
        #: ★弧长地板：已走过的顶点的弧长，投影不允许退回到它之前（单调推进）。
        #: 与仿真的 `_project_with_floor` 同语义 —— 断"倒车段↔正向段交界处"的自指环。
        self._vertex_floor_m = float(cfg.get("pursuit.vertex_floor_m"))
        self._s_floor = 0.0
        #: ★投影"跟丢"判据：窗口内最近点离线超过它就全路线重投影（解开地板/窗口的死锁）。
        self._release_m = float(cfg.get("pursuit.projection_release_m"))
        self.projection_relocks = 0            # 跟丢自愈次数（遥测/测试）
        self._min_speed_mps = float(cfg.get("pursuit.min_speed_mps"))
        self._pivot_phase: str = PivotPhase.IDLE
        self._pivot_started_at: float = 0.0
        self._pivot_target_heading: float = 0.0
        self._pivot_vertex_s: float = 0.0
        #: 本次 PIVOT 的顶点 = 哪一号航点（= 进入 PIVOT 时 `proj.seg_index + 1`），
        #: 完成/放弃时随 `PIVOT_DONE_KEY` 一起通知 MissionTask。
        self._pivot_vertex_index: int = -1
        #: ★**已处理过**（完成 或 超时放弃）的顶点：投影弧长越过这个值之前不再对它触发 PIVOT。
        #: 为什么需要：处理完车还停在该顶点邻域内（< pivot_dist_m），下一拍条件依然成立 →
        #: **立刻再次触发**。超时分支一开始就有这个标记，但**成功分支漏了** —— 实闭环 bench
        #: 实测（`--bench --drive --seconds 240`）在 F11 的 180° 折返处变成
        #: 「进入 PIVOT → 0.5s 刹车 → 判定完成 → 下一拍再进入」的**无限循环**：
        #: 几百次循环、整场卡在 s≈4.4/34 m，车几乎不动（每拍先被刹车 0.5s）。
        self._pivot_abandon_until_s: float = 0.0
        self.pivot_count = 0                     # 完成次数（遥测/测试）
        self.pivot_abandon_count = 0             # 超时放弃次数（遥测/测试）

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
                # ★ 路线换了 → 弧长地板必须复位（旧路线的 s 对新路线毫无意义）
                self._s_floor = 0.0
                # ★ 同理复位"已处理顶点"标记：旧路线的顶点 s 对新路线毫无意义，
                #   不复位的话同几何重载会把新轮次的 PIVOT 整个抑制掉。
                self._pivot_abandon_until_s = 0.0
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

        # ★ 行进方向（地图系）：倒车时是"车头反向"。路线折返压在同一条线上时
        #   （现场图 F10→F11→E11→R:F11→F10，B6→A6→R:B6 也是），同一物理点落在两条航段上
        #   完全等距，位置裁决不了 —— 必须把"车正在往哪边走"告诉投影（path.project 的
        #   motion_heading）。
        #   ★★ 但这个方向**必须取实际位移**，不能取"我们上一拍命令的速度符号"：
        #     命令符号 → 决定选哪条重叠段 → 决定 segment_is_reverse → 决定命令符号，
        #     这是一个**自指环**，两个状态互相自洽 ⇒ 每拍翻转。实闭环 bench 实测
        #     （`--bench --drive --seconds 240`，F11 折返簇）：命令在 +0.120/-0.080 之间
        #     交替、段号在 11↔13 之间来回跳、车几乎不动，12s 后触发"拐角绕圈兜底"强制跳点
        #     （F8/B8/B6 三处，全在同一段重叠折返上）。
        #     三层仿真不中招，正是因为它用的是**真值速度**算方向（tools/sim.py:555）——
        #     也就是说：仿真与实车代码在这一点上语义不一致，仿真绿掩盖了实车会卡。
        #   ★ 车几乎不动时（|位移| < 门限）方向是噪声 → **锁存上一次可信方向**，
        #     而不是退回命令符号（否则又回到自指环）。
        travel_heading = self._travel_heading
        if self._last_motion_xy is not None:
            dx = pose.x - self._last_motion_xy[0]
            dy = pose.y - self._last_motion_xy[1]
        else:
            dx = dy = 0.0
        self._last_motion_xy = (pose.x, pose.y)
        if math.hypot(dx, dy) >= self._motion_eps_m:
            travel_heading = math.atan2(dy, dx)         # 实测位移方向（倒车时天然反向）
            self._travel_heading = travel_heading
        elif travel_heading is None:
            travel_heading = pose.yaw                    # 首拍/从未动过：退化为车头方向
            self._travel_heading = travel_heading

        proj: Projection = path.project(pose.x, pose.y, s_hint=s_hint,
                                        window_m=self._window_m,
                                        back_m=self._rewind_m,
                                        s_min=self._s_floor,
                                        motion_heading=travel_heading)

        # ---- ★投影"跟丢"自愈：地板/窗口把真实最近点排除掉了 → 必须能解开 ----
        #  没有这个出口会怎样（实闭环 bench 实测，最严重的后果）：
        #    tracker 的 s 冻在 **7.70**（第 16 段）而车实际已在 **16.30**（第 32 段）——
        #    同一个信念位姿、两个写入者差 **8.6 m**；tracker 于是追着幻影目标开，
        #    虚拟小车**最大横向偏差 4.101 m**、只走到第 18/34 点。
        #  机制：`s_hint` 每拍跟随上一拍的 s ⇒ 窗口（±0.6 m）+ 地板一起把错误的 s
        #  变成自洽的不动点，车走多远都拉不回来。
        #  仿真的 `_project_with_floor` 一直有同样的出口（`release_dist`），实车代码此前没有
        #  （与"弧长地板"同属"仿真有、实车缺"的一类缺口）。
        if proj.dist > self._release_m:
            proj_relock = path.project(pose.x, pose.y)        # 丢掉 hint 与地板，全路线重投影
            # ★ 只接受**向前**的修正（`s` 不许后退到地板之前）：
            #   · 地板过时（bench 实测：地板停在 7.70 而车已在 16.30）→ 重投影向前 ✓ 采用
            #   · 地板正确、只是车横向离线（F11 过渡期 0.5 m）→ 重投影会退到上一段 ✗ 拒绝
            #   后者如果也采用，就等于把"地板"整个绕过去了 —— test_tracker 第 9 组立刻红
            #   （翻转 2 → 11、s 只推进到 1.05）。
            if (proj_relock.dist < proj.dist - 1e-6
                    and proj_relock.s >= self._s_floor - 1e-9):
                proj = proj_relock
                self._s_floor = path.s_at_waypoint(proj.seg_index)   # 地板跟到新位置的本段起点
                if self._log_throttle.allow("projection_relock", now):
                    self._log(
                        f"⚠️ 投影跟丢自愈：窗口内最近点离线 {proj_relock.dist:.2f} m，"
                        f"全路线重投影 → 段{proj.seg_index} s={proj.s:.2f}"
                        f"（累计 {self.projection_relocks + 1} 次）")
                self.projection_relocks += 1
        self._seg_s = proj.s

        # ---- ★弧长地板：贴到顶点就算"这个顶点走过了"，此后投影不许退回去 ----
        #  为什么在"贴到"（而不是"越过"）时就抬升：越过顶点的那一拍，投影可能已经被
        #  另一条共线段抢走（s 很小的那一支），此时再抬升就晚了 —— 探针里车正是这样
        #  在 F11 处冻住（倒车段末端 ↔ 正向段起点，两候选的 s 都在顶点附近、Δs 几乎相同，
        #  连续性分不出 ⇒ 落到"行进方向"⇒ 而方向由命令符号决定 ⇒ 自指环）。
        if (proj.dist_to_vertex <= self._vertex_floor_m
                and proj.seg_index + 1 < path.segment_count):
            v_s = path.s_at_waypoint(proj.seg_index + 1)
            if v_s > self._s_floor:
                self._s_floor = v_s

        is_rev = path.segment_is_reverse(proj.seg_index)
        cmd: PursuitCommand = compute_command(pose, path, proj, self.cfg,
                                              reverse=is_rev,
                                              last_steer_deg=self._last_steer_deg,
                                              dt=dt)
        align = self._align_active(proj.seg_index)

        # ---- ★PIVOT 原地转向：优先级最高（它在"顶点前"接管，免得纯追踪画出 0.4 m 的大弧） ----
        pivot = self._pivot_update(pose, path, proj, now, dt)

        # ---- ★倒车段入口的"推一把"（否则永远进不去倒车段） ----
        #  为什么必须有：投影"换到下一段"靠的是**实测位移方向**（见本文件 travel_heading
        #  那段长注释），而"车真的往后动"又需要**先换到倒车段**才会发 v<0 —— 鸡生蛋。
        #  实闭环 bench 实测（现场 F10→F11→E11→R:F11→F10）：车到了 E11 顶点后
        #  s 冻在 4.20 不动、yaw 一直转、每 12s 触发一次「绕圈跳点」。
        #  条件收得很紧：**已经贴到顶点** + **下一段是倒车段** + **本段不是倒车段**，
        #  此时直接给一个小倒车速度把车"推"回倒车段；投影一旦落到下一段就自动接管。
        rev_entry = (pivot is None and not is_rev
                     and proj.seg_index + 1 < path.segment_count
                     and path.segment_is_reverse(proj.seg_index + 1)
                     and proj.dist_to_vertex <= self._reverse_entry_m)

        if pivot is not None:
            steer, speed, reason_tag = pivot.steer_deg, pivot.linear_x, pivot.speed_reason
        elif rev_entry:
            # 只做航向保持，别让"推一把"把车带偏（车头本来就该保持原朝向：倒车段车头与
            # 行进方向相反，而此刻车头的朝向正是下一段需要的车头朝向）
            seg_heading = path.car_heading_on(proj.seg_index)
            err_deg = geo.deg(geo.ang_diff(seg_heading, pose.yaw))
            steer = err_deg * self._align_kp / 10.0
            lim = self._align_steer_limit_deg
            steer = lim if steer > lim else (-lim if steer < -lim else steer)
            speed = -self._min_speed_mps
            reason_tag = "reverse_entry"
            # ★ 还要把投影**显式推进到下一段**：换段（连续性/实测位移）在这个时刻都指向
            #   前一段（车刚从顶点退开、行进方向还没翻），于是 s 会**倒退**而不是继续增长。
            #   两条折返段在顶点处代表的是**同一个物理点**（这里只差 <1mm），
            #   所以这不是"跳变"，而是把"这一段已经走完"这件事告诉投影。
            vertex_s = path.s_at_waypoint(proj.seg_index + 1)
            if self._seg_s is None or self._seg_s <= vertex_s:
                self._seg_s = vertex_s + 1e-3
        elif align:
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
            "align": bool(align), "pivot": pivot is not None,
            "pivot_phase": str(self._pivot_phase),
            "reverse": bool(is_rev),
            "seg_index": int(proj.seg_index), "s": float(proj.s),
            "dist_to_path": float(proj.dist), "curvature": float(cmd.curvature),
            "speed_reason": cmd.speed_reason, "reason_tag": reason_tag,
            "lookahead_m": float(lookahead_distance(self._cruise_mps, self.cfg)),
            "align_segments": sorted(self._align_segs),
            "at": now,
        }
        self._write_telemetry()

    # ------------------------------------------------------------------ #
    #  ★PIVOT 原地转向（并入 10-05 核心的已验证修复）
    # ------------------------------------------------------------------ #
    def _corner_ahead(self, path: Path, proj: Projection):
        """前方顶点信息 -> (顶点弧长, 换向角°, 下一段的**车头朝向**)；没有下一段则 None。

        ★★ 换向角用 `corner_turn_deg`（**车头**要转多少），不是 `corner_angle_deg`
          （**行进方向**夹角）；目标航向用 `car_heading_on`（车头朝向），不是 `heading_at`
          （行进方向）。下一段是倒车段（`R:`）时两者差 180°：车倒着原路退回**一度都不用转**，
          旧代码却据此把车头转反 180°，之后纯追踪自己跟自己打架（bench 实测 F11 簇）。
        """
        i = proj.seg_index
        if i + 1 >= path.segment_count:
            return None
        s_v = path.s_at_waypoint(i + 1)
        return s_v, path.corner_turn_deg(i), path.car_heading_on(i + 1)

    def _pivot_update(self, pose, path: Path, proj: Projection, now: float, dt: float):
        """PIVOT 状态机：返回要发布的指令（PursuitCommand）或 None（= 不接管）。

        为什么要有它（实测证据）：纯追踪在 F11 那个 180° 折返会画大弧，
        横向偏差 0.37~0.41 m，而 E11/F8 是必须压过的雷 —— 偏差 0.4 m 就可能漏压。
        9 组参数扫描（`tools/sim_param_sweep.py`）证明这是**几何结构性**的，调参无解，
        必须改用"停下来 + 低速蠕动转向"（10-05 核心的做法）。

        ★ 与旧版的差异：**补了硬超时**（`pursuit.pivot_timeout_s`）——旧版没有超时，
        是本项目自己标注的缺口（survey §7 风险 R2）；超时即放行，绝不卡死。
        ★ 越界保护：本分支不需要自己查边界 —— 新版把越界急停收敛到 `SafetyTask` 一处
        （它是唯一有权强制停车者），Tracker 的 `P_SAFETY` 门会把指令整体刹住。
        """
        if not self._pivot_enabled:
            return None

        if self._pivot_phase == PivotPhase.IDLE:
            info = self._corner_ahead(path, proj)
            if info is None:
                return None
            s_v, angle, next_heading = info
            dist_to_vertex = s_v - proj.s
            # ★ 已处理（完成或超时）过的顶点：先开过去再说
            #   （否则「超时→放行→再触发」以及「完成→再触发」都会变成死循环）
            if s_v <= self._pivot_abandon_until_s:
                return None
            if angle < self._pivot_angle_deg or dist_to_vertex > self._pivot_dist_m:
                return None
            # 桥洞保护段内绝不做原地转向（旧版显式排除过；桥洞里侧壁就在旁边）
            if self._align_active(proj.seg_index):
                return None
            self._pivot_phase = PivotPhase.SETTLE
            self._pivot_started_at = now
            self._pivot_target_heading = next_heading
            self._pivot_vertex_s = s_v
            self._pivot_vertex_index = proj.seg_index + 1
            self._log(f"★ 进入 PIVOT（原地转向）：顶点 s={s_v:.2f} 距 {dist_to_vertex:.2f}m / "
                      f"换向 {angle:.0f}° > 阈值 {self._pivot_angle_deg:.0f}°")

        elapsed = now - self._pivot_started_at
        # ---- ① 硬超时兜底（★旧版缺这个，是本项目标注的缺口）----
        if elapsed > self._pivot_timeout_s:
            self._log(f"⚠️ PIVOT 超时（{elapsed:.1f}s > {self._pivot_timeout_s:.1f}s）→ 放行交回纯追踪")
            # 记下"这个顶点我不再 pivot"，等车开过去再恢复（防"超时→放行→再触发"循环）
            self._pivot_abandon_until_s = self._pivot_vertex_s + max(0.35, self._pivot_dist_m * 2.0)
            self.pivot_abandon_count += 1
            # ★ 通知 MissionTask："这个顶点已在 PIVOT 内处理过（放弃）"——
            #   否则 MissionTask 的到点判定恒假，12 s 后拐角绕圈兜底强制跳过（bench E11/A6/B6）。
            self.bb.update(self.name, **{PIVOT_DONE_KEY:
                                         (self._pivot_vertex_index, self._pivot_vertex_s,
                                          "abandon")})
            self._reset_pivot()
            return None

        err_deg = geo.deg(geo.ang_diff(self._pivot_target_heading, pose.yaw))

        # ---- ② 先停稳（让底盘惯性消失，再开始蠕转）----
        if self._pivot_phase == PivotPhase.SETTLE:
            if elapsed >= self._pivot_wait_s:
                self._pivot_phase = PivotPhase.TURN
            else:
                return PursuitCommand(linear_x=0.0, steer_deg=0.0, curvature=0.0,
                                      speed_reason="pivot_settle", reverse=False)

        # ---- ③ 蠕动转向：舵角指向目标航向，到容差即完成 ----
        if abs(err_deg) <= self._pivot_yaw_tol_deg:
            self._log(f"✓ PIVOT 完成：航向误差 {err_deg:+.1f}° ≤ {self._pivot_yaw_tol_deg:.0f}°"
                      f"（用时 {elapsed:.1f}s）")
            self.pivot_count += 1
            # ★ 通知 MissionTask："这个顶点已到达"（原地掉头后车会原路返回，永远不会
            #   进到达圈/越过顶点弧长 ⇒ MissionTask 正常判据恒假，见 PIVOT_DONE_KEY 注释）。
            self.bb.update(self.name, **{PIVOT_DONE_KEY:
                                         (self._pivot_vertex_index, self._pivot_vertex_s,
                                          "done")})
            # ★ 完成后**同样**要标记"这个顶点已处理"（与超时分支对称）：
            #   否则车还停在顶点邻域内、换向角依旧 > 阈值 → 下一拍立刻重入 PIVOT →
            #   又是 0.5s 刹车 → 再"完成" → **无限循环、车几乎不动**。
            #   实闭环 bench 在 F11（180° 折返）处就是这么卡的：整场 240s 只走到 4.4/34 m。
            self._pivot_abandon_until_s = self._pivot_vertex_s + max(0.35, self._pivot_dist_m * 2.0)
            self._reset_pivot()
            return None

        steer = err_deg * self._pivot_kp
        lim = self._max_steer_deg
        steer = lim if steer > lim else (-lim if steer < -lim else steer)
        # 舵角速率限幅（与纯追踪同一条契约，避免"限幅只在纯追踪里生效"的暗坑）
        if self._last_steer_deg is not None:
            max_step = self._max_steer_rate_deg_s * dt
            delta = steer - self._last_steer_deg
            if abs(delta) > max_step:
                steer = self._last_steer_deg + (max_step if delta > 0 else -max_step)
        return PursuitCommand(linear_x=self._pivot_speed_mps, steer_deg=steer, curvature=0.0,
                              speed_reason="pivot_turn", reverse=False)

    def _reset_pivot(self) -> None:
        self._pivot_phase = PivotPhase.IDLE
        self._pivot_started_at = 0.0

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
