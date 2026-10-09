# -*- coding: utf-8 -*-
"""LocalizationTask —— 定位与锚定（20 Hz，架构 §3.1 的 T1）。

职责（docs/P1接口约定.md §3.1，逐条落地）：
1. 每拍读 `bridge.pose()`（io 层已把**格单位 + 原始车体 yaw** 换算成米制地图 Pose），
   `None` 或 `bridge.pose_age()` 为 inf → **不写 `P_POSE`**（宁愿让下游看到"没有位姿"，
   也不要用旧值冒充新值）、写 `P_ODOM_AGE = inf`。
2. 应用 `P_FIX`（`core.state.FixState`，P1 阶段全 0）：`FixState.corrected()` 之后再写 `P_POSE`；
   未修正的原值写 `P_POSE_RAW`（遥测/复盘要能看出"修正了多少"）。
3. 有 `P_ROUTE` 时投影：`Path(route).project(x, y, s_hint=上一拍的 s)` → 写 `P_ROUTE_INDEX`/`P_ROUTE_S`。
   ★ 必须带 `s_hint`：没有它每拍都是 O(整条路线) 全扫（旧 `find_nearest_segment` 的热点），
   而且路线自交时会误锁到另一条边。
   ★★ **卡死自愈**（`_heal_stuck`，2026-10-07 补）：带 `s_hint` 的投影有一个**吸收态** ——
   `Path.project(allow_backward=False)` 把窗口下界夹在 `s_hint` 上（`t_floor`），
   车的垂足一旦落到窗口**之前**，"窗口内最近点"就永远是下界点本身 ⇒ 返回的 `s` 恰好
   等于上一拍 hint ⇒ 下一拍 hint 不变 = 不动点，车继续走而上报弧长**永久停住**。
   实测（bench 日志 + 离线逐值复算）冻结值 `(12,4.41) (15,6.78) (19/20,9.10) (14,4.9)
   (10,3.73) (17/18,8.05)`，上报投影点离线 0.5~3.5 m 占 20~37% 拍；`tasks/fix.py`
   的段比值因此被污染成物理不可能的 0.12~0.23。对策：三条判据同时成立才做**一次**
   全路线重投影（不是每拍全扫）。
4. **到位吸附**（`fix.anchor_at_waypoint`）：车距"当前段末端航点 / 最近的未完成航点"
   小于 `pursuit.arrival_m` 时，把 `P_POSE` 的**位置**锚到该航点格心（yaw 不动），
   发 `EV_POSE_ANCHORED`，并把本次锚定前的偏差记进 `P_FIX` 的
   `anchor_count / last_anchor / drift_m / max_drift_m`。
   为什么要有这一步：纯轮式死推里程计会累积漂移，锚定把"每过一个已知航点"变成一次
   误差归零的机会；偏差不丢弃而是记进 `FixState`，P3 的 FixTask 才有估计素材。
5. 漂移告警：单段漂移 > `pursuit.drift_warn_m` → `EV_DRIFT_WARNING`（`core.logging.Throttle` 限流）。

铁律（core/task.py 文档 + P1 §4）：
- `tick()` 非阻塞、无长 I/O；
- **不 import rclpy**：只依赖注入的 `bridge` 鸭子类型（车端 `io.ros_io.RosBridge` / 测试 `fakes.FakeBridge`）；
- **不调 `time.time()/time.monotonic()`**：一律用注入的 `clock()`，否则测试无法快进时间。

本文件不改任何 core/route/pursuit/io 代码，只消费它们的公开接口。
"""

from __future__ import annotations

import math
from typing import Callable, Protocol

from ..core import geometry as geo
from ..core.blackboard import (P_FIX, P_ODOM_AGE, P_POSE, P_POSE_RAW, P_ROUTE,  # noqa: F401
                               P_ROUTE_INDEX, P_ROUTE_S, Blackboard)
from ..core.config import Config
from ..core.events import EV_DRIFT_WARNING, EV_POSE_ANCHORED, EventBus
from ..core.logging import Throttle
from ..core.state import FixState
from ..core.task import TaskThread
from ..pursuit.path import Path
from ..route.model import Route

__all__ = ["LocalizationTask", "PoseSource"]

_INF = float("inf")
#: 自愈判定的"有没有变"容差（米）：全扫与窗口投影的差异小于它就算一致（浮点噪声/顶点归属）。
_EPS_S = 1e-9


class PoseSource(Protocol):
    """`bridge` 只被用到的两个方法（鸭子类型下限，写在这里当契约）。"""

    def pose(self):                      # -> geo.Pose | None
        ...

    def pose_age(self) -> float:
        ...


class LocalizationTask(TaskThread):
    """把原始里程计变成"信念位姿 + 路线进度"，并在航点处吸附纠偏。"""

    def __init__(self, bridge, bb: Blackboard, bus: EventBus, cfg: Config, *,
                 log: Callable[[str], None] = print, group=None,
                 clock: Callable[[], float] = None) -> None:
        from time import monotonic as _monotonic
        super().__init__("Localization", 20.0, bus=bus, blackboard=bb, logger=log)
        self.bridge = bridge
        self.cfg = cfg
        self.group = group
        self.clock = _monotonic if clock is None else clock

        # 参数一次读齐（tick 里不再碰 cfg，避免每拍 20 次键查找 + 校验开销）
        self.pose_timeout_s = float(cfg.get("safety.pose_timeout_s") or 1.0)
        self.arrival_m = float(cfg.get("pursuit.arrival_m") or 0.12)
        self.drift_warn_m = float(cfg.get("pursuit.drift_warn_m") or 0.30)
        self.anchor_enabled = bool(cfg.get("fix.anchor_at_waypoint"))
        #: 漂移判定的"越段闸门"：车沿路线走到离顶点还剩 ≤ 这个距离就进入可判定窗口。
        #: 取 0.18 m ≈ arrival_m(0.12) 的一半再放宽 → 正常行驶（0.15 m/s @20 Hz = 7.5 mm/拍）
        #: 必然经过该窗口，不会"跳过去"；而 1.5 倍 arrival_m 又保证不会提前太久报警。
        #: ★ 该判据纯弧长、与横向偏差无关 —— 否则"漂得越狠越报不出来"。
        self.drift_seg_gate_m = float(cfg.get("pursuit.drift_seg_gate_m") or 0.18)
        #: ★ 卡死自愈的三条判据（`localization.stuck_*`，理由见 `_heal_stuck`）：
        #: · `stuck_ticks`：连续这么多拍 s 前进不足 `stuck_eps_m`；
        #: · `stuck_dist_m`：上报的投影点离车这么远（说明它已经不是车的位置）；
        #: · 位姿在这几拍里移动 > `arrival_m`（"贴住不动"必须配上"车确实在动"）。
        self.stuck_ticks = max(2, int(cfg.get("localization.stuck_ticks") or 5))
        self.stuck_eps_m = float(cfg.get("localization.stuck_eps_m") or 0.01)
        self.stuck_dist_m = float(cfg.get("localization.stuck_dist_m") or 0.20)
        self._warn = Throttle(float(cfg.get("logging.rate_limit_s") or 1.0))

        # 路线缓存：Route 不可变，用 `is` 判同一性即可
        self._route: Route | None = None
        self._path: Path | None = None
        self._s_hint: float | None = None
        self._last_seg: int = -1
        self._anchor_idx: int = -1          # 上一个"已吸附过"的航点索引（防同一航点反复锚）
        self._anchor_key: tuple | None = None   # 上次发过漂移告警的锚点（按航点，不按时间）
        self._last_odom_ts: float | None = None

        # 卡死判定的滑动窗口（起点：s 与位姿。s 一有前进就重新起算）
        self._stuck_s: float | None = None
        self._stuck_n: int = 0
        self._stuck_x: float = 0.0
        self._stuck_y: float = 0.0

        # 遥测/自检计数（测试直接断言这些，不靠解析日志）
        self.ticks = 0
        self.missing_pose_ticks = 0
        self.projected_ticks = 0
        self.anchored = 0
        self.drift_warnings = 0
        self.stuck_scans = 0        # 判定卡死 → 全路线重投影的次数（含"全局在身后"的）
        self.projection_heals = 0   # 其中真正改了上报值（全局投影更靠前）的次数

    # ------------------------------------------------------------------ #
    #  生命周期
    # ------------------------------------------------------------------ #
    def setup(self) -> None:
        self._log(f"定位：锚定={'ON' if self.anchor_enabled else 'OFF'}"
                  f" 到位半径={self.arrival_m:.3f}m 漂移告警={self.drift_warn_m:.3f}m"
                  f" 前向投影窗口={self.arrival_m:.3f}m 定位超时={self.pose_timeout_s:.2f}s"
                  f" 卡死自愈={self.stuck_ticks}拍/{self.stuck_eps_m:.3f}m/{self.stuck_dist_m:.2f}m")

    def tick(self) -> None:
        self.ticks += 1
        now = self.clock()

        # ---------------- ① 取位姿：拿不到就明确"没有"，绝不沿用旧值 ---------------- #
        pose = self.bridge.pose()
        age = _INF if pose is None else self._float_or_inf(self.bridge.pose_age())
        if pose is None or age == _INF:
            self.missing_pose_ticks += 1
            # ★ P_ODOM_AGE 每拍都要写（Safety 的第①道检查直接读它）
            self.bb.set(P_ODOM_AGE, _INF, source=self.name, ts=now)
            if self.missing_pose_ticks == 1 or self._warn.allow("pose_missing", now):
                self._log(f"定位缺失：pose={'None' if pose is None else '有'} age={age}"
                          f"（已 {self.missing_pose_ticks} 拍不写 P_POSE）")
            return
        self._last_odom_ts = now
        self.bb.set(P_ODOM_AGE, age, source=self.name, ts=now)

        # ---------------- ② 应用 P_FIX 修正 ---------------- #
        fix = self.bb.get(P_FIX)
        if not isinstance(fix, FixState):
            fix = FixState()
        cx, cy, cyaw = fix.corrected(pose.x, pose.y, pose.yaw)
        self.bb.set(P_POSE_RAW, pose, source=self.name, ts=now)

        # ---------------- ③ 路线投影（带 s_hint） ---------------- #
        proj = self._project(cx, cy)
        if proj is not None:
            self.projected_ticks += 1
            self.bb.set(P_ROUTE_INDEX, proj.seg_index, source=self.name, ts=now)
            self.bb.set(P_ROUTE_S, proj.s, source=self.name, ts=now)

        # ---------------- ④ 到位吸附 + 漂移判定 ---------------- #
        anchored_pose = self._anchor(cx, cy, cyaw, proj, now)
        if anchored_pose is None and proj is not None and self.anchor_enabled:
            gate = self._warning_gate(proj)
            if gate is not None:
                gidx, gwp = gate
                self._report_drift(gwp.block, gidx, gwp.distance_to(cx, cy), now,
                                   anchored=False)

        # 写 P_POSE 的必须是"最终信念"，且 P_ROUTE_S 要与之一致 →
        # 吸附把点挪了，就重新投影一次（单段内，代价可忽略）
        if anchored_pose is not None:
            self.bb.set(P_POSE, anchored_pose, source=self.name, ts=now)
            proj2 = self._project(anchored_pose.x, anchored_pose.y)
            if proj2 is not None:
                self.bb.set(P_ROUTE_INDEX, proj2.seg_index, source=self.name, ts=now)
                self.bb.set(P_ROUTE_S, proj2.s, source=self.name, ts=now)
        else:
            self.bb.set(P_POSE, geo.Pose(cx, cy, cyaw), source=self.name, ts=now)

    def teardown(self) -> None:
        self._log(f"定位退出：{self.ticks} 拍 / 缺位姿 {self.missing_pose_ticks} 拍 / "
                  f"吸附 {self.anchored} 次 / 漂移告警 {self.drift_warnings} 次 / "
                  f"投影卡死自愈 {self.projection_heals} 次（判定 {self.stuck_scans} 次）")

    # ------------------------------------------------------------------ #
    #  内部
    # ------------------------------------------------------------------ #
    @staticmethod
    def _float_or_inf(value) -> float:
        try:
            out = float(value)
        except (TypeError, ValueError):
            return _INF
        return _INF if math.isinf(out) or math.isnan(out) else out

    def _project(self, x: float, y: float):
        """用当前 s_hint 投影（无路线/路线太短 → None）。

        ★ 调用形状是**试出来的**，不是随手填的：`Path.project` 在 `allow_backward=True` 时把
          窗口下界 `s_hint - window_m` 折算成 `t_floor` 并夹住本段的 t。于是当 `window_m`
          大于"本拍到 hint 的距离"时（例如 window_m=0.45 m、车每拍走 0.05 m），
          **投影点每拍都被夹回 s_hint → s 永远停住**（实测：整条 0.35 m 路线走完，s 恒为 0）。

          正确形状 = `allow_backward=False` + `window_m = arrival_m`：
          · 前向窗口 [s_hint, s_hint + 0.12] —— 足以覆盖 20 Hz × 0.6 m/s 的正常位移
            （0.03 m/拍），并在噪声下仍不后退；
          · `allow_backward=False` 保证 `s` 单调（P1 §3.1 验收："P_ROUTE_S 单调"）。
          代价：倒车段（`R:`）投影只能停在最小 s 上 —— 倒车段要回退时改传 True，
          但那属于 Tracker/Mission 的用法，本任务只报"信念沿程位置"。
          ★★ 这套形状本身有个**吸收态**：车的垂足落到窗口之前时，返回的 s 恰好等于
          hint ⇒ 不动点。由 `_heal_stuck()` 兜底（判据与实证见那里），
          所以本函数的调用方不需要额外的看门狗。
        """
        route = self.bb.get(P_ROUTE)
        if not isinstance(route, Route) or len(route.waypoints) < 2:
            return None
        if self._route is not route:        # 换了路线：缓存与 hint 一起作废
            self._route = route
            self._path = Path(route)
            self._s_hint = None
            self._last_seg = -1
            self._anchor_idx = -1
            self._anchor_key = None
            self._clear_stuck()
            self._log(f"路线已装载：{len(route.waypoints)} 点 / {self._path.total_length:.2f} m")
        assert self._path is not None
        proj = self._path.project(x, y, s_hint=self._s_hint,
                                  window_m=self.arrival_m, allow_backward=False)
        proj = self._heal_stuck(proj, x, y)
        self._s_hint = proj.s
        self._last_seg = proj.seg_index
        return proj

    # ------------------------------------------------------------------ #
    #  ★ 投影卡死自愈（本文件 2026-10-07 新增的核心修复）
    # ------------------------------------------------------------------ #
    def _clear_stuck(self) -> None:
        self._stuck_s = None
        self._stuck_n = 0
        self._stuck_x = 0.0
        self._stuck_y = 0.0

    def _heal_stuck(self, proj, x: float, y: float):
        """投影"贴住不动"但车确实在动 → 做**一次**全路线重投影，把 hint 拉回真值。

        ## 为什么必须修（这是带 `s_hint` 的投影链的结构性吸收态，不是调参问题）

        `Path.project(allow_backward=False)` 把窗口下界夹在 `s_hint` 上：
        `lo = hint`、`t_floor = (lo - s_start[i_first]) / seg_len`，于是**首段**的 t 被夹到
        `t_floor`（= 弧长 `hint` 处那个点）。当车的垂足落在窗口**之前**时，
        窗口内最近点就永远是那个下界点 ⇒ `proj.s == hint`（逐位相等）⇒ 下一拍 hint 不变
        ⇒ **不动点，永不自行恢复** —— 因为本任务每拍都拿自己的上一拍当 hint。

        bench 实证（`_tmp_session/bench_*.txt` 的 `[T]` 行 + `Path` 全局投影对拍）：
        上报投影点离线 >0.5 m 占 20~37%、最大 3.53~3.67 m；冻结值集中在顶点吸引子
        `(12,4.41) (15,6.78) (19/20,9.10) (14,4.9) (10,3.73) (17/18,8.05)`；
        用本任务的调用形状（`window_m=0.12, allow_backward=False`）离线复算逐值命中。

        ## 判据（三条同时成立；每条都必要，见各自的理由）

        1. `proj.s` 相对窗口起点前进 < `stuck_eps_m` 且已持续 ≥ `stuck_ticks` 拍
           —— "贴住不动"。取 0.01 m：最低速 0.08 m/s @20 Hz 五拍也前进 20 mm，
           所以正常行驶（哪怕蠕动）**不可能**被误判；
        2. 位姿在同样这几拍里移动 > `arrival_m` —— "车确实在动"。没有它，停车等靶时
           的横向偏差会被当成卡死（那种情形 s 本来就该不动）；
        3. `proj.dist > stuck_dist_m` —— 上报的投影点已经**不是车的位置**了。取 0.20 m
           （与仿真实测的跟踪 RMS 同量级）：便宜地挡掉"投影贴住但确实就在车脚下"的
           情形（例如刚越过末点时 dist 会涨，那种 s 本来就该停住）。

        ## 方向：不做"只向前"限制（及为什么仍不破坏单调语义）

        全路线重投影给出的 `s` 可能**在冻结值身后**（车真的倒回去了/被推回了）。
        ⚠️ 实测结论（2026-10-07，合成路线 + bench 位姿）：**不能**限制方向 —— 车"切角
        之后往回退一点"时真值本来就在身后（本例 0.550 → 0.402），限制方向等于把错误值
        留在原地。所以改为靠判据 3 兜底：`proj.dist > stuck_dist_m` 保证"车已经不在
        路线上"，此时"贴着路线的单调"没有意义，报真值更重要；而**沿路线正常倒车**
        （dist ≈ 0 < 0.20）根本不会进自愈分支，`allow_backward=False` 的单调语义
        在正常情形下一字未改。

        ## 代价

        只在判定卡死时做一次全扫（33 段 ≈104 µs，正常每拍仍是窗口投影 ≈2.5 µs）；
        "判定卡死但全扫结果与窗口一致"时也重新起算窗口，所以最多每 `stuck_ticks` 拍
        一次全扫，不会退化成"每拍全路线扫描"。
        """
        if self._path is None or not self._stuck_check(proj, x, y):
            return proj
        self.stuck_scans += 1
        # ★ 全路线搜索，但**保留 hint**（只让它裁决等距候选）：
        #   `s_hint=None` 时 path.py 对共线折返段按"取更靠后的段"裁决，会把投影误锁到
        #   另一条边（"路线自交时会误锁到另一条边"是它 docstring 里既有的警告），
        #   s 会凭空前跳一整段（实测 +1.15 m）；而"取离 hint 最近的候选"才是连续性语义。
        #   窗口给到整条路线长度 = 全扫；候选集是窗口投影的**超集** ⇒ dist 只会变好不会变差。
        glob = self._path.project(x, y, s_hint=self._s_hint,
                                  window_m=self._path.total_length, allow_backward=True)
        if abs(glob.s - proj.s) > _EPS_S:
            self.projection_heals += 1
            self._stuck_s, self._stuck_x, self._stuck_y, self._stuck_n = glob.s, x, y, 0
            if self._warn.allow("proj_heal", self.clock()):
                self._log(f"⚠️ 投影卡死自愈：s 冻在 {proj.s:.3f}（第 {proj.seg_index} 段，"
                          f"离线 {proj.dist:.2f} m），车已在动 → 全路线重投影得 "
                          f"{glob.s:.3f}（第 {glob.seg_index} 段，离线 {glob.dist:.2f} m，"
                          f"{glob.s - proj.s:+.3f} m，累计自愈 {self.projection_heals} 次）")
            return glob
        # 全扫结果与窗口一致 ⇒ 冻住的值就是真最近点（车在等/在原地转）：只重新起算窗口
        self._stuck_s, self._stuck_x, self._stuck_y, self._stuck_n = proj.s, x, y, 0
        return proj

    def _stuck_check(self, proj, x: float, y: float) -> bool:
        """维护"贴住不动"的滑动窗口；返回 True 表示三条判据都成立。"""
        if proj.passed_end:                 # 已跑过末点：s 本来就该停在总长上
            self._stuck_s, self._stuck_x, self._stuck_y, self._stuck_n = proj.s, x, y, 0
            return False
        if self._stuck_s is None:           # 窗口起点（换路线/首拍/刚自愈过）
            self._stuck_s, self._stuck_x, self._stuck_y, self._stuck_n = proj.s, x, y, 0
            return False
        if proj.s - self._stuck_s >= self.stuck_eps_m:   # 有前进 → 重新起算
            self._stuck_s, self._stuck_x, self._stuck_y, self._stuck_n = proj.s, x, y, 0
            return False
        self._stuck_n += 1
        if self._stuck_n < self.stuck_ticks:
            return False
        moved = math.hypot(x - self._stuck_x, y - self._stuck_y)
        return moved > self.arrival_m and proj.dist > self.stuck_dist_m

    def _anchor(self, x: float, y: float, yaw: float, proj, now: float):
        """到位吸附。返回锚定后的 Pose；未触发返回 None。

        两件事分开判，不要混：
        · **吸附**只有一种触发条件 —— 车离目标航点 < `arrival_m`（"到位"就是到位）；
        · **漂移判定**在 `_warning_gate()` 处独立进行 —— 否则就有个结构性死角：
          `arrival_m`(0.12) 远小于 `drift_warn_m`(0.30)，横向漂了 0.5 m 的车
          永远进不了吸附半径 ⇒ 既不纠偏、也永远报不出漂移（漂移这件事"报不出来"）。
        """
        if proj is None or self._path is None or not self.anchor_enabled:
            return None
        candidates = self._anchor_candidates(proj, x, y)
        target_idx, target = -1, None
        for idx, wp in candidates:
            if wp.distance_to(x, y) < self.arrival_m:
                target_idx, target = idx, wp
                break
        if target is None:
            self._reset_anchor_latch(x, y, proj, now)
            return None
        if target_idx == self._anchor_idx:  # 同一航点只锚一次，不反复重锚（防"越锚越偏"）
            return geo.Pose(target.x, target.y, yaw)

        drift = math.hypot(x - target.x, y - target.y)
        self._anchor_idx = target_idx
        self.anchored += 1
        self._store_drift(target, drift, now)
        self.bus.publish(EV_POSE_ANCHORED, source=self.name, block=target.block,
                         index=target_idx, drift=round(drift, 4),
                         x=round(target.x, 4), y=round(target.y, 4))
        self._log(f"到位吸附：{target.block}（航点 #{target_idx}）偏差 {drift:.3f} m"
                  f" → 锚回格心 ({target.x:.3f}, {target.y:.3f})，yaw 不变 {yaw:+.3f} rad")
        self._report_drift(target.block, target_idx, drift, now, anchored=True)
        return geo.Pose(target.x, target.y, yaw)

    def _anchor_candidates(self, proj, x: float, y: float) -> tuple[tuple[int, object], ...]:
        """吸附候选，按优先级：① 当前段末端航点 ② 其后最近的未完成航点（兜底）。

        ★ 判据用**段索引**而不是"离最近格心 < 阈值"：后者会让车在 K 列附近任何位置都被吸附
          （旧代码桥洞保护踩过的"僵直"坑，survey §6.1 防误伤设计）。

        兜底候选（②）的准入 = 「正在趋近该顶点」：顶点尚未被甩到身后
        （`dist_to_vertex + drift_warn_m >= 0`，允许顶点略在身后），且横向偏差不超过
        `max(arrival_m, drift_warn_m)` —— 吸附半径是 `arrival_m`（0.12 m），
        这个准入只是"值得算一下距离"的必要条件，不构成吸附判据本身。
        """
        assert self._path is not None
        pts = self._path.waypoints
        n = len(pts)
        nxt = proj.seg_index + 1
        if proj.passed_end:
            return ((n - 1, pts[n - 1]),)
        if nxt >= n:
            return ()
        if proj.dist_to_vertex + self.drift_warn_m < 0.0:
            return ()
        if proj.dist > max(self.arrival_m, self.drift_warn_m):
            return ()
        out: list[tuple[int, object]] = [(nxt, pts[nxt])]
        if proj.dist <= self.drift_warn_m:      # 贴着折线才有资格补锚更远的点
            out.extend((idx, pts[idx]) for idx in range(nxt + 1, n))
        return tuple(out)

    # ------------------------------------------------------------------ #
    #  漂移判定（独立于吸附）
    # ------------------------------------------------------------------ #
    def _warning_gate(self, proj) -> tuple[int, object] | None:
        """当前是否处于"可判定漂移"的窗口：车正在接近某个顶点。

        ★ 主判据是**弧长**（沿路走到离顶点还剩 ≤ `pursuit.drift_seg_gate_m`），
          不是"离折线的横向距离"。这是刻意的：横向距离大的车恰恰是漂移最大的车，
          用它当门槛 = "漂得越狠越报不出来"。弧长判据与横向无关，任何方向漂移都进窗。
          辅以顶点半径判据，覆盖"刚越过顶点一点、横向还歪着"的情形。
        """
        assert self._path is not None
        pts = self._path.waypoints
        n = len(pts)
        if proj.passed_end:
            return (n - 1, pts[n - 1])
        idx = proj.seg_index + 1
        if idx >= n:
            return None
        wp = pts[idx]
        d_back = proj.dist_to_vertex
        d_vertex = wp.distance_to(proj.x, proj.y)
        in_seg_gate = (d_back >= -1e-9) and (d_back <= self.drift_seg_gate_m)
        if in_seg_gate or d_vertex <= self.drift_warn_m:
            return (idx, wp)
        return None

    def _reset_anchor_latch(self, x: float, y: float, proj, now: float) -> None:
        """本次没吸附成：判断是"还没到"还是"已经走过且干净"。

        ★ 只有"投影已经离开被闩住的航点"才解开闩锁。
          两个反面情形都要挡住：
          · 车还在半路（离下一航点 0.3 m > arrival 0.12 m）：此时若解锁，
            等真正到位时会**再锚一次** —— 同一航点反复吸附，越锚越偏；
          · 车刚走过航点但投影还停在它附近（例如 M10 → M9 的下一段）：
            此时若随便解锁，会把"刚刚锚过的点"再锚一遍。
          判据 = 投影的段索引已越过被闩住的航点，**或**该航点已不在可判定窗口里。
        """
        if self._anchor_idx < 0:
            return
        passed = (proj.seg_index + 1 > self._anchor_idx) or proj.passed_end
        if passed and self._warning_gate(proj) is None:
            if self._warn.allow("anchor_depart", now):
                self._log(f"已走完航点 #{self._anchor_idx}（投影已在其之后、且已离开判定窗口）"
                          f" → 下一个航点可再次吸附")
            self._anchor_idx = -1

    def _report_drift(self, block: str, index: int, drift: float, now: float,
                      *, anchored: bool) -> None:
        """漂移告警。★ 每个航点最多一条：按 `(block, index)` 去重，Throttle 只作兜底。

        时间限流下同一航点会按 20 Hz 刷屏；按航点去重则是"每航点至多一条" ——
        既保证异常逃不掉，又不会把日志冲掉。
        """
        if drift <= self.drift_warn_m:
            return
        key = (block, index)
        if self._anchor_key == key:
            return
        self._anchor_key = key
        self.drift_warnings += 1
        self.bus.publish(EV_DRIFT_WARNING, source=self.name, block=block, index=index,
                         drift=round(drift, 4), threshold=self.drift_warn_m,
                         anchored=anchored)
        self._log(f"⚠️ 漂移告警：{block} 偏差 {drift:.3f} m > {self.drift_warn_m:.3f} m"
                  f"（{'已锚回格心' if anchored else '未吸附（未进入到位半径）'}，"
                  f"累计 {self.drift_warnings} 次）")

    def _store_drift(self, target, drift: float, now: float) -> None:
        """把本次锚定偏差写进 P_FIX（P3 的 FixTask 用它做估计，不能丢）。"""
        fix = self.bb.get(P_FIX)
        if not isinstance(fix, FixState):
            fix = FixState()
        new_fix = FixState(
            dx=fix.dx, dy=fix.dy, dyaw=fix.dyaw, scale=fix.scale,
            anchor_count=fix.anchor_count + 1,
            last_anchor=target.block,
            drift_m=drift,
            max_drift_m=max(fix.max_drift_m, drift),
        )
        self.bb.set(P_FIX, new_fix, source=self.name, ts=now)

