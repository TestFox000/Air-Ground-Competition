# -*- coding: utf-8 -*-
r"""FixTask —— 误差修正（10 Hz），架构 §7.1 的 **L2 位置修正 · 估计侧**。

★ 分工（越界就是 bug，别抢别人的键）：
  · `LocalizationTask` = **执行侧**：到位吸附把 `P_POSE` 锚回格心，并把每次锚定的偏差
    记进 `P_FIX`（`FixState`：anchor_count / last_anchor / drift_m / max_drift_m）。
  · 本任务 = **估计侧**：只读事件与 `P_POSE`/`P_ROUTE_S`，把**估计量**写进 `P_FIX_EST`
    （同名类型 `FixState`）—— `scale`（系统性比例误差估计）、`drift_m`/`max_drift_m`
    （跨段汇总）、`anchor_count`（从事件累计）。
  · ★★ **本任务绝不改位姿**：不写 `P_POSE` / `P_POSE_RAW` / `P_FIX`。应用侧是
    Localization（或将来接入的 L1/L3/L4 汇合点），见 §7.1 的"修正量汇合点"设计。

## `scale` 的语义（★ 读之前先看这里，否则会反用）

`scale` = **信念位移 / 路线几何弧长** = "里程计相对路线报大了多少倍"。
  · `1.0`   → 里程计与路线几何一致（无系统误差）；
  · `1.18`  → 车以为自己走了 1.18 m，实际路线只给了 1.0 m ⇒ 里程**多报 18%**；
  · `0.93`  → 里程**少报** 7%。

★ 为什么不是"修正系数"：`1/scale` 才是乘到里程上的修正系数（1.18 的估计 → 乘 0.847）。
  两个约定在纸面上都讲得通，但 `P_FIX_EST.scale` 落的是**观测量**（测量比值），
  修正系数由应用侧取 `1/scale`。理由：估计器输出"我看到的事实"，应用方决定怎么用；
  并且 `scale_limit`（±0.10）夹的是同一个量，语义单一、不会两处各夹一次。
  ⚠️ 若将来要求 `scale = 1.0 表示不改` 的乘性系数语义，只需在应用处取倒数，
      **不要在估计器里再取一次倒数**（两处取倒数 = 误差被平方）。

## 估计方法（保守三板斧，防"误收敛"）

每完成一段（两次锚定之间）：
  1. 比较"信念位移沿路线累计的弧长 `Σ|Δpose|`"与"路线几何弧长 `ΣΔs`"→ 比值 `ratio`；
  2. 三重剔除：**异常漂移**（`|drift| > pursuit.drift_warn_m`）、**比值越界**
     （不在 `fix.ratio_bounds`）、**比值突跳**（与最近窗口均值差 > `fix.ratio_jump_max`）；
  3. 保守更新：单段步长 ≤ `fix.scale_step_max`、总量夹在 `1 ± fix.scale_limit`、
     样本数 < `fix.scale_min_samples` 不出估计值。

★ 为什么要"保守"：比例误差的观测本身带噪声（吸附位移、投影跳段、倒车段方向反转），
  一次误收敛会让整条路线的限速/到点判据系统性偏移——旧代码的教训是"宁可不动，不可乱动"。

## 输入守卫（`_settle_ds`，2026-10-07 补）

`P_ROUTE_S` 有**三个写入者**（Localization / Tracker / Mission），各维护自己的投影链。
任一处冻结或换手，相邻两拍就会给出物理上不可能的 `ds`（bench 实测：冻结期间 `P_ROUTE_S`
在 3.73↔4.20 之间来回跳）。原实现 `if ds > 0: geo_m += ds` 只挑正数不判别 ⇒ 来回跳全被
记成前进 ⇒ 段比值被推成 0.12~0.23（真值≈1），并把正常样本挤掉。
现在只接受 `0 < ds <= max(同一窗口的位姿位移 × fix.ds_max_ratio, fix.ds_floor_m)`：
越界的丢弃并计数（`ds_rejected` / `ds_rejected_m`，见 `stats()`），**不写任何新黑盒键**。
被判不可信的那一拍**不推进基线**，下一拍把落下的正常位移一起补回来（belief 与 geo 必须
覆盖同一段时间，否则比值会系统性偏大）；连续 `fix.ds_reject_max_run` 拍不可信就重新基线。

## 铁律

- `tick()` 非阻塞：只做 O(1) 累加 + 事件 drain，不做投影、不做 I/O；
- **不 import rclpy**；不直接调 `time.time()/time.monotonic()`（走注入的 `clock`）；
- 与 Localization 只通过 Blackboard/EventBus 通信，**禁止互相调用**。

配置键：本文件读的可选键**已全部进 `core/config.py` 的 SCHEMA**（`fix.seg_min_m` /
`fix.scale_min_samples` / `fix.drift_cull_m` / `fix.ratio_bounds` / `fix.ratio_jump_max` /
`fix.ratio_jump_min_prior` / `fix.ds_max_ratio` / `fix.ds_floor_m`），
所以 `Config.validate()` 能看见它们，键名拼错会被"未知配置段"告警兜住。
"""

from __future__ import annotations

import math
from typing import Any, Callable

from ..core.blackboard import P_FIX_EST, P_POSE, P_ROUTE_S, Blackboard
from ..core.config import Config
from ..core.events import EV_DRIFT_WARNING, EV_POSE_ANCHORED, EventBus
from ..core.logging import Throttle
from ..core.state import FixState
from ..core.task import TaskThread

__all__ = ["FixTask"]

#: 位姿突跳判定（米）：超过它认为"信念位姿被外力挪过"（吸附/复位），本拍不计入位移。
#: 0.20 m 的取法：吸附半径上限 `fix.anchor_max_offset_m`(0.08) 的 2.5 倍，
#: 远小于正常航段 0.35 m 的骨架位移 → 正常行驶绝不会被误判成突跳。
_TELEPORT_M = 0.20

_EPS = 1e-9


class FixTask(TaskThread):
    """系统性比例误差在线估计 + 跨段漂移统计（10 Hz，只写 `P_FIX_EST`）。"""

    def __init__(self, bridge, bb: Blackboard, bus: EventBus, cfg: Config, *,
                 log: Callable[[str], None] = print, group=None,
                 clock: Callable[[], float] | None = None) -> None:
        super().__init__("FixTask", 10.0, bus=bus, blackboard=bb, logger=log)
        self.bridge = bridge
        self.cfg = cfg
        self.group = group
        if clock is None:
            import time as _time
            clock = _time.monotonic
        self.clock = clock

        # ---------------- 配置（一次读齐；缺键走显式默认值，见模块 docstring ⚠️） ----
        self.est_enabled = bool(cfg.get("fix.scale_est_enabled"))
        self.step_max = float(cfg.get("fix.scale_step_max"))
        self.scale_limit = float(cfg.get("fix.scale_limit"))
        self.drift_warn_m = float(cfg.get("pursuit.drift_warn_m"))
        self.drift_cull_m = float(cfg.get("fix.drift_cull_m", self.drift_warn_m))
        self.seg_min_m = float(cfg.get("fix.seg_min_m", 0.05))
        self.min_samples = int(cfg.get("fix.scale_min_samples", 3))
        bounds = cfg.get("fix.ratio_bounds", (0.75, 1.35)) or (0.75, 1.35)
        self.ratio_min = float(bounds[0])
        self.ratio_max = float(bounds[1])
        self.ratio_jump_max = float(cfg.get("fix.ratio_jump_max", 0.15))
        self.ratio_jump_min_prior = int(cfg.get("fix.ratio_jump_min_prior", 3))
        self.window = max(1, int(cfg.get("fix.scale_window", 5)))
        # ★ 单拍位移守卫（`_accumulate`）：见 `_ds_limit()` 与模块 docstring ★P_ROUTE_S
        self.ds_max_ratio = float(cfg.get("fix.ds_max_ratio") or 2.0)
        self.ds_floor_m = float(cfg.get("fix.ds_floor_m") or 0.05)
        self.ds_reject_max_run = max(1, int(cfg.get("fix.ds_reject_max_run") or 3))
        self._rate_limit_s = float(cfg.get("logging.rate_limit_s") or 1.0)

        # ---------------- 估计状态 ----------------
        self.scale = 1.0                     # 当前落盘的估计（夹紧后）
        self.scale_raw = 1.0                 # 未夹紧的原始估计（诊断用，不落盘）
        self.segments_total = 0              # 见过的完整段数（含被剔除的）
        self.segments_accepted = 0
        self.segments_rejected = 0
        self.reject_reasons: dict[str, int] = {}
        self.steps = 0                       # 实际发生过的估计更新次数
        self._ratios: list[float] = []       # 最近 window 个被接受的比值
        self.ds_rejected = 0                 # ★ 物理不可能的 ds 被丢弃的拍数（可观测）
        self.ds_rejected_m = 0.0             # ★ 被丢弃的弧长总量（米，可观测）
        self._ds_run_m = 0.0                 # 本结算窗口内累计的位姿位移（配 ds 用）
        self._ds_reject_run = 0              # 连续被判不可信的拍数（防死循环）

        # ---------------- 漂移台账（从事件累计，P_FIX 里的是 Localization 的副本） ----
        self.anchor_count = 0
        self.last_anchor = ""
        self.drift_m = 0.0                   # 最近一段漂移
        self.max_drift_m = 0.0
        self.drift_warnings = 0

        # ---------------- 单段累加器 ----------------
        self._prev_pose = None               # 上一拍信念位姿（geo.Pose）
        self._prev_s: float | None = None    # 上一拍路线弧长
        self._geo_m = 0.0                    # 本段"路线几何弧长"（真值代理）
        self._belief_m = 0.0                 # 本段"信念位移累计"
        self._frames = 0
        self._rebaselined = False            # 本拍刚重设基线 → 本拍不累计位移

        self._sub = None
        self._warn = Throttle(self._rate_limit_s)

    # ------------------------------------------------------------------ #
    #  生命周期
    # ------------------------------------------------------------------ #
    def setup(self) -> None:
        if self._sub is None:
            self._sub = self.bus.subscribe(self.name, [EV_POSE_ANCHORED])
        self._publish(1.0, self.clock())
        self._log(f"误差修正（估计侧）：比例估计={'ON' if self.est_enabled else 'OFF'}"
                  f" 步长≤{self.step_max:g}/段 夹紧±{self.scale_limit:g}"
                  f" 样本门槛={self.min_samples}段 漂移剔除>{self.drift_cull_m:.3f}m"
                  f" 比值窗口={self.bounds_text()}")

    def teardown(self) -> None:
        if self._sub is not None:
            try:
                self._sub.unsubscribe()
            except Exception:                       # noqa: BLE001  退出路径不许抛
                pass
            self._sub = None
        self._log(f"退出：锚定 {self.anchor_count} 次 / 段 {self.segments_accepted}"
                  f"/{self.segments_total} 接受 / 估计 scale={self.scale:.5f}"
                  f"（原始 {self.scale_raw:.5f}）/ 漂移 max {self.max_drift_m:.3f} m"
                  f"/ 丢弃不可信 ds {self.ds_rejected} 拍（{self.ds_rejected_m:.3f} m）")

    # ------------------------------------------------------------------ #
    #  只读接口（测试/遥测/结束横幅）
    # ------------------------------------------------------------------ #
    @property
    def samples(self) -> int:
        """已接受的有效段数（< `fix.scale_min_samples` 时不出估计值）。"""
        return self.segments_accepted

    def bounds_text(self) -> str:
        return f"[{self.ratio_min:g}, {self.ratio_max:g}]"

    def stats(self) -> dict[str, Any]:
        """估计器健康度（测试直接断言，不靠解析日志）。"""
        return {
            "scale": self.scale, "scale_raw": self.scale_raw,
            "samples": self.segments_accepted, "segments": self.segments_total,
            "rejected": self.segments_rejected, "reasons": dict(self.reject_reasons),
            "steps": self.steps, "ratios": list(self._ratios),
            "ds_rejected": self.ds_rejected, "ds_rejected_m": self.ds_rejected_m,
        }

    def drift_report(self) -> dict[str, Any]:
        """漂移台账（结束横幅用）。"""
        return {
            "anchor_count": self.anchor_count, "last_anchor": self.last_anchor,
            "drift_m": round(self.drift_m, 4),
            "max_drift_m": round(self.max_drift_m, 4),
            "warnings": self.drift_warnings,
            "scale": round(self.scale, 5),
            "samples": self.segments_accepted,
        }

    def render_report(self) -> str:
        """一页纸（横幅/日志用，纯格式化）。"""
        return (f"误差修正：锚定 {self.anchor_count} 次，漂移 最近 {self.drift_m:.3f} m "
                f"/ 最大 {self.max_drift_m:.3f} m（告警 {self.drift_warnings} 次），"
                f"比例估计 {self.scale:.5f}（{self.segments_accepted} 段样本"
                f"，剔除 {self.segments_rejected}）")

    # ------------------------------------------------------------------ #
    #  单拍
    # ------------------------------------------------------------------ #
    def tick(self) -> None:
        now = self.clock()
        self._rebaselined = False
        # ① 先处理锚定事件：段边界在这里闭合，本拍的位移从**锚定后**重新起算
        self._drain_events(now)
        # ② 累计本段位移（信念 vs 路线几何）
        self._accumulate()
        # ③ 落盘估计（异常也不许让估计器把任务带崩）
        try:
            self._publish(self.scale, now)
        except Exception as exc:                    # noqa: BLE001
            if self._warn.allow("publish_failed", now):
                self._log(f"写 P_FIX_EST 失败（本拍跳过）：{type(exc).__name__}: {exc}")

    # ------------------------------------------------------------------ #
    #  事件：锚定 / 漂移
    # ------------------------------------------------------------------ #
    def _drain_events(self, now: float) -> None:
        if self._sub is None:
            return
        events = self._sub.poll(64)
        for ev in events:
            if ev.type == EV_POSE_ANCHORED:
                self._on_anchored(ev, now)

    def _on_anchored(self, ev, now: float) -> None:
        data = ev.data or {}
        block = str(data.get("block") or "")
        drift = self._as_float(data.get("drift"), 0.0)
        self.anchor_count += 1
        self.last_anchor = block
        self.drift_m = drift
        self.max_drift_m = max(self.max_drift_m, drift)

        # ---- 段闭合：只有"本段真的走过"才拿去估计 ----
        self.segments_total += 1
        ratio = self._segment_ratio()
        if ratio is None:
            self._reject("seg_too_short", f"段 {block} 位移不足"
                                          f"（几何 {self._geo_m:.3f} m / {self._frames} 拍）", now)
        elif drift > self.drift_cull_m:
            self._reject("drift_outlier",
                         f"段 {block} 漂移 {drift:.3f} m > {self.drift_cull_m:.3f} m", now)
        elif not (self.ratio_min <= ratio <= self.ratio_max):
            self._reject("ratio_out_of_range",
                         f"段 {block} 比值 {ratio:.3f} 超出 {self.bounds_text()}", now)
        elif (len(self._ratios) >= self.ratio_jump_min_prior
              and abs(ratio - self._window_mean()) > self.ratio_jump_max):
            self._reject("ratio_jump",
                         f"段 {block} 比值 {ratio:.3f} 突跳（窗口均值 "
                         f"{self._window_mean():.3f}，容差 {self.ratio_jump_max:.3f}）", now)
        else:
            self._accept(ratio, block, now)

        # ---- 漂移告警（限流；Localization 已按航点各发一条，这里只作兜底） ----
        if drift > self.drift_warn_m and self._warn.allow(f"drift:{block}", now):
            self.drift_warnings += 1
            self.bus.publish(EV_DRIFT_WARNING, source=self.name, block=block,
                             index=data.get("index"), drift=round(drift, 4),
                             threshold=self.drift_warn_m, by="fix_est")
            self._log(f"⚠️ 漂移告警（估计侧）：{block} {drift:.3f} m > "
                      f"{self.drift_warn_m:.3f} m（累计 {self.drift_warnings} 次）")

        # ---- 段边界：基线移到锚定后的位姿（必须重设，否则突跳会污染下一段） ----
        self._rebaseline()

    def _accept(self, ratio: float, block: str, now: float) -> None:
        self.segments_accepted += 1
        self._ratios.append(ratio)
        if len(self._ratios) > self.window:
            del self._ratios[:-self.window]
        if not self.est_enabled:
            # `fix.scale_est_enabled=False` → **只统计不改估计**（scale 恒 1.0）
            if self._warn.allow("est_off", now):
                self._log(f"比例估计已关闭：段 {block} 实测比值 {ratio:.4f}"
                          f"（{self.segments_accepted} 段样本，scale 保持 1.0）")
            return
        if self.segments_accepted < self.min_samples:
            return                                   # 样本不够，先看不动
        step = max(-self.step_max, min(self.step_max, ratio - self.scale))
        self.scale = self._clamp(self.scale + step)
        self.scale_raw = ratio                       # 原始观测量（诊断）
        self.steps += 1
        self._log(f"比例估计更新：段 {block} 比值 {ratio:.4f}，步长 {step:+.5f}"
                  f" → scale {self.scale:.5f}（{self.segments_accepted} 段，"
                  f"窗口 {self._window_mean():.4f}）")

    def _reject(self, reason: str, detail: str, now: float) -> None:
        self.segments_rejected += 1
        self.reject_reasons[reason] = self.reject_reasons.get(reason, 0) + 1
        if self._warn.allow(f"reject:{reason}", now):
            self._log(f"⚠️ 异常段剔除（{reason}）：{detail} → 本段不进估计"
                      f"（已剔除 {self.segments_rejected} 段）")
        self.bus.publish(EV_DRIFT_WARNING, source=self.name, block="",
                         reason=reason, detail=detail, threshold=self.drift_warn_m,
                         by="fix_est_reject")

    # ------------------------------------------------------------------ #
    #  位移累计
    # ------------------------------------------------------------------ #
    def _accumulate(self) -> None:
        pose = self.bb.get(P_POSE)
        if pose is None:
            return
        s_raw = self.bb.get(P_ROUTE_S)
        s = self._as_float(s_raw, None) if isinstance(s_raw, (int, float)) else None

        if self._prev_pose is None:
            self._rebaseline(pose=pose, s=s)
            return
        if not self._rebaselined:
            step = math.hypot(pose.x - self._prev_pose.x, pose.y - self._prev_pose.y)
            if step > _TELEPORT_M:
                # 信念位姿被外力挪过（吸附/复位/投影跳段）：本段作废重起，
                # ★ 不能把它当"位移"计入，否则比值会被系统性拉偏。
                if self._warn.allow("teleport", self.clock()):
                    self._log(f"位姿突跳 {step:.3f} m > {_TELEPORT_M:.2f} m"
                              f"（吸附/复位？）→ 本段重新累计")
                self._rebaseline(pose=pose, s=s)
                return
            self._belief_m += step
            self._frames += 1
            if s is not None:
                self._settle_ds(s, step)      # ★ 唯一改动 prev_s / _geo_m / 计数的地方
        elif s is not None:
            # 本拍是"基线刚重设后的第一拍"（锚定/突跳的下一拍）：只把弧长基线对齐到当前值，
            # 不累计几何 —— 与旧行为一致（旧代码每拍都同步 `_prev_s`），否则基线会停在
            # 重设前的位置，下一拍的 ds 会跨过整个重设窗口。
            self._prev_s = s
            self._ds_run_m = 0.0
        self._prev_pose = pose

    def _ds_limit(self, run_m: float) -> float:
        """本结算窗口允许计入的**最大弧长增量**（米）。

        ★ 为什么需要这道守卫：`P_ROUTE_S` 有**三个写入者**（Localization / Tracker /
          Mission），各维护自己的投影链；任一处冻结或换手，相邻两拍的 `ds` 就会出现
          物理上不可能的跳。原来的累加是 `if ds > 0: geo_m += ds` —— 只挑正数、不判别，
          于是"冻结→跳回→再冻结"的来回全被记成**前进**，段比值被推成 0.12~0.23
          （bench 实测：K9 0.23~0.46 / H10 0.12~0.22 / G10 1.46~1.57，真值≈1）。
          后果不是"多一条告警"：这些离谱比值会把**正常样本挤掉**，估计器的样本数与
          收敛都被牵连。

        判据 = `max(同一窗口的位姿位移 × fix.ds_max_ratio, fix.ds_floor_m)`，两个都必要：
          · `× 2` 利用"弧长 ≥ 直线位移"这一几何关系 —— 正常行驶 ds ≈ 位移，给 2 倍余量；
          · 绝对下限 `fix.ds_floor_m`(0.05 m ≈ 0.5 m/s × 0.1 s) 兜住"位姿位移很小"的拍：
            否则蠕行/定位抖动时 `位移×2` 会小到把**正常**的 ds 也拒掉（那会把 geo_m
            系统性算小，比不修更糟）。
        """
        return max(run_m * self.ds_max_ratio, self.ds_floor_m)

    def _settle_ds(self, s: float, step: float) -> None:
        """结算一次弧长增量。`ds` 与 `run_m` 覆盖**同一个时间窗口**，这是比值正确的前提。

        ★ 窗口语义（为什么被判不可信时**不**推进基线）：`ds = s - prev_s`，而 `prev_s`
          只在①结算成功 ②本窗口测不到几何（`ds ≤ 0`：停着/倒车/投影冻住）③连续
          `fix.ds_reject_max_run` 拍都不可信 这三种情况下前进。
          于是"某一拍被判不可信"不会**吞掉**那一拍的正常位移 —— 下一拍的 ds 会把它一起
          带回来（补账），belief 与 geo 仍然覆盖同一段时间，比值不会因为丢一拍而系统性偏大
          （旧写法"直接丢掉一拍"实测能把 1.18 的比值抬到 1.77，反而误杀正常段）。
          连续多拍不可信说明不是噪声而是"换了写入者的坐标系"→ 重新基线，避免死循环。
        """
        self._ds_run_m += step
        if self._prev_s is None:
            self._prev_s = s
            self._ds_run_m = 0.0
            return
        ds = s - self._prev_s
        if ds <= 0.0:
            # 没有前进：本窗口测不到几何（停着/倒车/被冻住）→ 基线跟上，位移作废
            self._prev_s = s
            self._ds_run_m = 0.0
            self._ds_reject_run = 0
            return

        limit = self._ds_limit(self._ds_run_m)
        if ds <= limit:
            self._geo_m += ds
            self._prev_s = s
            self._ds_run_m = 0.0
            self._ds_reject_run = 0
            return

        # ---- 物理不可能：丢弃（基线不动 → 下一拍补账），计数 + 限流日志 ----
        self.ds_rejected += 1
        self.ds_rejected_m += ds
        self._ds_reject_run += 1
        if self._warn.allow("ds_impossible", self.clock()):
            self._log(f"⚠️ 段位移不可信：本窗口 ds={ds:.3f} m 远超位姿位移 "
                      f"{self._ds_run_m:.3f} m（上限 {limit:.3f} m = 位移×"
                      f"{self.ds_max_ratio:g} 与 {self.ds_floor_m:.3f} m 取大）→ 不计入几何弧长"
                      f"（已丢弃 {self.ds_rejected} 拍 / {self.ds_rejected_m:.3f} m）")
        if self._ds_reject_run >= self.ds_reject_max_run:
            self._prev_s = s                  # 连续不可信 = 换了坐标系，重新基线
            self._ds_run_m = 0.0
            self._ds_reject_run = 0

    def _rebaseline(self, pose=None, s: float | None = None) -> None:
        self._geo_m = 0.0
        self._belief_m = 0.0
        self._frames = 0
        self._rebaselined = True
        self._ds_run_m = 0.0
        self._ds_reject_run = 0
        if pose is not None:
            self._prev_pose = pose
        if s is not None:
            self._prev_s = s

    # ------------------------------------------------------------------ #
    #  小工具
    # ------------------------------------------------------------------ #
    def _segment_ratio(self) -> float | None:
        if self._geo_m < self.seg_min_m or self._belief_m < self.seg_min_m:
            return None
        return self._belief_m / self._geo_m

    def _window_mean(self) -> float:
        if not self._ratios:
            return 1.0
        return sum(self._ratios) / len(self._ratios)

    def _clamp(self, value: float) -> float:
        lo = 1.0 - self.scale_limit
        hi = 1.0 + self.scale_limit
        return max(lo, min(hi, value))

    @staticmethod
    def _as_float(value: Any, default: float) -> float:
        try:
            out = float(value)
        except (TypeError, ValueError):
            return default
        return default if math.isnan(out) or math.isinf(out) else out

    def _publish(self, scale: float, now: float) -> None:
        """写 `P_FIX_EST`（★ 只写这一个键；位姿与本任务无关）。"""
        self.bb.set(P_FIX_EST, FixState(
            dx=0.0, dy=0.0, dyaw=0.0,             # ★ 估计侧不做平移/航向修正
            scale=scale,
            anchor_count=self.anchor_count,
            last_anchor=self.last_anchor,
            drift_m=self.drift_m,
            max_drift_m=self.max_drift_m,
        ), source=self.name, ts=now)
