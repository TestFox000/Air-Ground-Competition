# -*- coding: utf-8 -*-
"""AimTask（50 Hz）—— 云台瞄准 + 打击：轨道移动靶的运动估计、提前量、开火门控与弹药账本。

对应 `docs/新版架构设计.md` §五（移动轨道靶方案）与 `docs/survey_gimbal_fire.md` §10（15 条缺口）。

===============================================================================
★ 开火权：两把钥匙（**本任务只拿第二把，绝不碰第一把**）
===============================================================================
| 话题              | 谁写           | 语义                                             |
|-------------------|----------------|--------------------------------------------------|
| `/aim/fire_gate`  | **MissionTask**| 任务级：车已到站、已停稳、进入 FIRE 阶段          |
| `/aim/fire_ready` | **本任务**     | 瞄准级：目标确认、预测落点在开火框内、冷却已过、弹药未超限 |
云台 C++ 补丁只在**两者都为 1** 时点亮激光。所以本模块里出现 `publish_fire_gate`
就是 bug —— 测试第 9 组会断言 `bridge.fire_gate` **全程为空**。

===============================================================================
★ 旧版 15 条缺口 → 本任务的对应处置
===============================================================================
| 旧缺口（survey §10）                                  | 本实现                                        |
|---|---|
| ① 无速度估计，只有滤波后单帧差分（px/帧，无时间戳）    | ★ 1-D α-β 滤波，**dt 来自 `OffsetObservation.ts`** |
| ② 预判只有 0.5 帧 ≈16ms，链路延迟却有 0.5~0.9s        | `lead_px = v_dx × aim.lead_time_s` → `publish_lead` |
| ③ 开火要求"画面静止 ≤1.5px/帧 + 保持 5 帧" → 移动靶打不出去 | `fire_policy=moving_lead`：判据换成**预测落点在框内** |
| ④⑤ 无命中闭环 / shots_on_target 重锁清零             | `take_hits()` 记账，**命中即停火**（锁存）+ 许可/实发分账 |
| ⑥ TRACK 丢帧云台冻结最长 6s                          | 丢帧**立刻闭锁** `fire_ready=0`，并按 `aim.lost_frames_to_rescan` 计时 |
| ⑦ `scan_pitch_offset` 冲突 / ⑨ 无目标 ID / ⑩ 类别迟滞 | 不在本任务范围（分别属配置核对、视觉端、类别锁定已实现） |
| ⑪ 放行门是死线                                       | 两把钥匙（见上）；本任务只管 `fire_ready`      |
| ⑫ 无弹药闭环                                          | `bridge.ammo()` / `shoot_count()` / `take_hits()` 三路对账 |
| ⑮ 距离标定未用于提前量                                | ★ 框宽 → 开火框**线性插值**（近大远小），见 `fire_box_px()` |

===============================================================================
★ 折返（turnaround）为什么必须闭锁 —— 移动靶最容易打飞的地方
===============================================================================
本靶**无端点停顿**（瞬时折返，§五 实测），所以策略必须是"带提前量的运动中开火"。
但折返瞬间的滤波速度**必然**是错的（旧速度方向 + 新测量方向相反），若继续外推：
`lead_px = v_dx_old × 0.35` = +10.5px 而真实目标已反向 → 预测落点偏 21px > 开火框 → 必打飞。
因此本实现有**两道闸**：
1. **跳变门**（`aim.gate_*`）：残差 > 门限 → 估计器**重新初始化且 v 清零**，绝不外推旧速度；
2. **方向确认**（`aim.reset_confirm_frames`）：重新初始化后必须连续 N 帧同向才重新武装
   `fire_ready`（期间 `target.v` 可能已经很大，但**不允许开火**）。
数值证据见 `tests/test_aim.py` 第 3 组（折返后 `lead_px` 归零 → 6 拍闭锁 → 重新许可）。

⚠️ **本靶的真实物理上限不在滤波精度，而在 `lead_time_s` 的标定**：
`v=4cm/s × T=0.35s = 1.4cm`；提前量时间写错 0.2s，落点就偏 0.8cm（=0.35m 处约 11px）——
接近一个开火框宽。上线前必须用实测延迟（0.5~0.9s）核对 `aim.lead_time_s`。

★ 时间来源一律走构造注入的 `clock`（默认真实 `time.monotonic`）：冷却、丢靶计时、
   限流日志都依赖它；测试用 `FakeClock` 快进。**任务内不得直接调 `time.time()`**。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from ..core.blackboard import (P_AIM, P_SAFETY, P_TARGET, P_TELEMETRY, Blackboard)
from ..core.config import SCHEMA, Config
from ..core.events import (EV_AMMO_CHANGED, EV_FIRE_ALLOWED, EV_FIRE_BLOCKED,
                           EV_HIT_CONFIRMED, EV_TARGET_FIRST_SEEN, EV_TARGET_LOST, EventBus)
from ..core.logging import Throttle
from ..core.state import AimState, SafetyState, TargetState
from ..core.task import TaskThread
from ..io.contract import AimStatus, OffsetObservation, ShotClass

__all__ = ["AimTask", "RailEstimator", "fire_box_px", "AIM_TELEMETRY_KEY",
           "FirePolicy", "VISION_MAX_AGE_S", "REASON_*"]

#: 遥测键：本任务最近一拍的判定细节（塞进 `P_TELEMETRY` 的这一个子键，不覆盖别人）。
AIM_TELEMETRY_KEY = "aim_cmd"

#: 视觉新鲜度上限（秒）。约定值 0.5s：过期观测拿去开火就是打空气。
VISION_MAX_AGE_S = 0.5

#: 判定原因码（`AimState.last_reason` / 遥测 `blocked_reason` 用，测试按它断言）
REASON_OK = "ok"
REASON_NO_TARGET = "no_target"
REASON_TARGET_LOST = "target_lost"
REASON_SAFETY = "safety_not_ok"
REASON_MANUAL = "manual_gimbal"
REASON_CLASS_UNSTABLE = "cls_unstable"
REASON_CLASS_NOT_ALLOWED = "cls_not_allowed"
REASON_ESTIMATOR_WARMUP = "estimator_warmup"
REASON_ESTIMATOR_STALE = "estimator_stale"
REASON_OUT_OF_BOX = "out_of_box"
REASON_COOLDOWN = "cooldown"
REASON_HIT_LATCHED = "hit_latched"
REASON_AMMO_EMPTY = "ammo_empty"
REASON_BUDGET_PER_TARGET = "budget_per_target"
REASON_BUDGET_TOTAL = "budget_total"
REASON_POLICY_TURNAROUND = "policy_turnaround_no_window"
REASON_POLICY_STATIC = "policy_static_needs_still"
REASON_POLICY_UNKNOWN = "policy_unknown"

#: 每个原因码的日志措辞（限流后打人看的；测试断言的是原因码不是措辞）
_REASON_TEXT = {
    REASON_OK: "瞄准合格",
    REASON_NO_TARGET: "无目标观测",
    REASON_TARGET_LOST: "目标丢失（闭锁）",
    REASON_SAFETY: "安全不 OK",
    REASON_MANUAL: "遥控接管云台",
    REASON_CLASS_UNSTABLE: "类别未确认",
    REASON_CLASS_NOT_ALLOWED: "类别不允许（fire_only_blue）",
    REASON_ESTIMATOR_WARMUP: "速度估计收敛中（折返/重捕后闭锁）",
    REASON_ESTIMATOR_STALE: "观测断流（估计已失效）",
    REASON_OUT_OF_BOX: "预测落点在开火框外",
    REASON_COOLDOWN: "冷却未到",
    REASON_HIT_LATCHED: "已命中（本靶停火）",
    REASON_AMMO_EMPTY: "弹药为 0",
    REASON_BUDGET_PER_TARGET: "单靶许可数到上限",
    REASON_BUDGET_TOTAL: "全场发数到上限",
    REASON_POLICY_TURNAROUND: "turnaround 策略：本靶无端点停顿，等价不可用",
    REASON_POLICY_STATIC: "static 策略：要求目标静止",
    REASON_POLICY_UNKNOWN: "未知开火策略（按不开火处理）",
}


class FirePolicy:
    """开火策略名（与 `aim.fire_policy` 配置值一致）。"""

    MOVING_LEAD = "moving_lead"
    SWEEP = "sweep"
    TURNAROUND = "turnaround"
    STATIC = "static"

    ALL = (MOVING_LEAD, SWEEP, TURNAROUND, STATIC)


def fire_box_px(box_w: float, *, near_px: float, far_px: float,
                box_w_far: float, box_w_near: float) -> float:
    """框宽 → 开火框半宽（px）。**近大远小**：目标越近视差越大，框越宽。

    ★ 这是 survey §10 第 6 条的落地点：旧代码把 `box_w_far/near` 标定**只用于死区**，
      `laser_offset` 固定 0，`compute_comp_pixel_0` 是死代码 —— 距离信息完全没进提前量链路。
      这里做一次纯线性插值（无魔法数字，两端点全部来自配置/旧标定值）：

          t = clamp((box_w - box_w_far) / (box_w_near - box_w_far), 0, 1)
          box = far_px + t × (near_px - far_px)

    ⚠️ 极端/退化输入处理：
      · `box_w ≤ 0`（观测没给框宽）→ 取**最保守**的远端框 `far_px`；
      · `box_w_near == box_w_far`（标定未填）→ 同样取 `far_px`（近大远小的直觉宁可偏严）；
      · 超范围一律夹到端点，不外推（外推会得到一个"什么都能打中"的框，等于取消判据）。
    """
    far_px = float(far_px)
    near_px = float(near_px)
    span = float(box_w_near) - float(box_w_far)
    if box_w <= 0 or abs(span) < 1e-9:
        return far_px
    t = (float(box_w) - float(box_w_far)) / span
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    return far_px + t * (near_px - far_px)


# --------------------------------------------------------------------------- #
#  轨道运动估计（★ 核心）
# --------------------------------------------------------------------------- #
@dataclass
class RailEstimator:
    r"""一维 α-β 滤波：对 `dx`（画面水平偏移）估计速度 `v`（px/s），带跳变门 + 方向确认。

    模型：`x_{k+1} = x_k + v_k·dt`，`v_{k+1} = v_k`（匀速）；观测 `z_k = x_k + 噪声`。

    增益由**跟踪指数** ξ 解析给出（不是拍脑袋的调参）：
        α = 2(2ξ−1) / (ξ(ξ+1))，  β = 6 / (ξ(ξ+1))
    ξ=3.5（默认）→ α=0.7619、β=0.3810。实测（见 tests/test_aim.py）：
      · 恒速 30px/s 序列第 4 帧起 |v−30| ≤ 15%，第 20 帧误差 **0.00%**；
      · 1px 量化噪声下稳态 |v−30| ≤ **11.6 px/s**（× lead_time 0.35s = **4.1px** 提前量噪声，
        远小于开火框 16~38px —— 也就是说**允许开火**而不会因滤波噪声打飞）。

    ★ 三道防线（旧版一条都没有）：

    1. **dt 来自观测时间戳**（`OffsetObservation.ts`），不是固定 1/30s。
       旧版 `vx = dx_filt − dx_prev_filt` 是"px/帧"，帧率一变（视觉 30Hz → 丢帧 → 20Hz）
       速度含义就变了（survey §10 第 1 条）。
    2. **跳变门**：残差 > `max(jump_floor_px, gate_k·σ_meas + gate_vel_scale·|v|·dt)`
       → 判定"不是同一个运动"（目标被遮挡后重现、类别换靶、坐标跳变），
       **重新初始化且 v 清零** —— 绝不把跳变量当成 3σ 以上的速度。
    3. **方向确认 `warmup`**：重新初始化后 `warmup = reset_confirm_frames`，每来一帧减一；
       `warmup > 0` 期间**不允许开火**（`armed` 为假）。这是折返后"给收敛时间"的落地方式：
       折返瞬间 v 已清零、方向刚翻，此时提前量毫无意义。

    ⚠️ `on_no_frame()` 与"目标不可见"是两件事：
      · `on_no_frame()`（走 `_apply` 且本帧无有效观测）→ v 清零、标 stale：断流后**旧速度不可外推**；
      · 目标不可见但视觉仍在出帧（cls=0 心跳帧）→ 同样清零（不可见就没有可用速度）。
      两者都会让 `fire_ready=0`（`estimator_stale`），区别只在日志措辞。
    """

    xi: float = 3.5
    gate_k: float = 3.0
    gate_base_px: float = 3.0
    gate_vel_scale: float = 0.10
    jump_floor_px: float = 25.0
    reset_confirm_frames: int = 5
    dt_min_s: float = 0.002
    dt_max_s: float = 0.50
    max_speed_px_s: float = 420.0

    # ---- 状态（默认值即"未初始化"） ----
    x: float = 0.0
    v: float = 0.0
    samples: int = 0
    resets: int = 0
    jump_rejections: int = 0
    warmup: int = 0
    flips: int = 0
    armed_run: int = 0
    last_ts: float | None = None
    last_dt: float = 0.0
    stale: bool = True
    init: bool = False
    last_reason: str = "init"

    def __post_init__(self) -> None:
        self.alpha, self.beta = self.gains(self.xi)

    # ------------------------------------------------------------------ #
    @staticmethod
    def gains(xi: float) -> tuple[float, float]:
        """跟踪指数 ξ → (α, β)。ξ 越小越跟手、噪声越大。"""
        xi = max(0.5, float(xi))
        alpha = 2.0 * (2.0 * xi - 1.0) / (xi * (xi + 1.0))
        beta = 6.0 / (xi * (xi + 1.0))
        return alpha, beta

    @property
    def armed(self) -> bool:
        """估计是否可信到可以用来开火（初始化过 + 未断流 + 方向已确认）。"""
        return bool(self.init) and not self.stale and self.warmup <= 0

    # ------------------------------------------------------------------ #
    def _reinit(self, z: float, reason: str) -> None:
        """重新初始化：位置取测量值、**速度清零**、进入方向确认期。"""
        self.x = float(z)
        self.v = 0.0
        self.init = True
        self.stale = False
        self.warmup = max(0, int(self.reset_confirm_frames))
        self.armed_run = 0
        self.resets += 1
        self.last_reason = reason

    def _block_warmup(self, reason: str) -> None:
        """★ 折返处置：**速度清零 + 重新确认**，绝不把折返前的速度外推出去。

        为什么需要单独一条（跳变门管不住它）：
          本靶移速只有 4cm/s（≈1px/帧），折返是**平滑**的 —— 一帧残差只有 2px，
          远低于 `jump_floor_px=25` 的跳变门。若只靠跳变门，估计器会平滑地把 +30px/s
          翻成 −14px/s 而**不加任何闭锁**，此时 `lead = +?` 仍指向旧方向 → 必打飞。
          旧版云台连"折返"这个概念都没有（survey §10 第 5 条：无折返识别）。
        """
        self.v = 0.0
        self.warmup = max(0, int(self.reset_confirm_frames))
        self.armed_run = 0
        self.flips += 1
        self.last_reason = reason

    def update(self, dx: float, ts: float) -> bool:
        """喂一帧**有效**观测。返回是否被采纳（False = 重复/乱序帧或异常 dt）。"""
        z = float(dx)
        if self.last_ts is not None:
            dt = float(ts) - float(self.last_ts)
            if dt <= self.dt_min_s:
                self.last_reason = "dup_frame"          # 同帧重读/时钟不动 → 丢弃（不更新，也不清速度）
                return False
            if dt > self.dt_max_s:
                self._reinit(z, f"gap_dt={dt:.3f}s")    # 断流：旧速度不可外推
                self.last_ts = float(ts)
                self.last_dt = dt
                self.samples += 1
                return True
        else:
            dt = 0.0

        if not self.init:
            # ★ 第一帧只初始化位置，**不**从 0 推速度（否则第一拍会把"目标在 dx 处"当成巨大速度）
            self.x, self.v, self.init, self.stale = z, 0.0, True, False
            self.warmup = max(0, int(self.reset_confirm_frames))
            self.armed_run = 0
            self.last_ts = float(ts)
            self.last_dt = 0.0
            self.samples += 1
            self.last_reason = "first"
            return True

        # ---- 预测 ----
        x_pred = self.x + self.v * dt
        res = z - x_pred

        # ---- 跳变门（残差归一化：帧越长、估计越快，门越宽） ----
        gate = self.gate_k * self.gate_base_px + self.gate_vel_scale * abs(self.v) * dt
        gate = max(gate, self.jump_floor_px)
        if abs(res) > gate:
            self.jump_rejections += 1
            self._reinit(z, f"jump_res={res:.1f}px>gate={gate:.1f}px")
            self.last_ts = float(ts)
            self.last_dt = dt
            self.samples += 1
            return True

        # ---- 校正 ----
        v_prev = self.v
        self.x = x_pred + self.alpha * res
        v_new = v_prev + self.beta * res / dt

        # ---- ★ 折返：速度符号翻转 → 清零 + 重新确认（见 `_block_warmup` 的说明） ----
        if v_prev * v_new < 0.0:
            self._block_warmup(f"turnaround v {v_prev:+.1f}→{v_new:+.1f}px/s")
            self.last_ts = float(ts)
            self.last_dt = dt
            self.samples += 1
            return True

        # ---- 荒谬速度（噪声/误检）：宁可清零重估，也不外推一个不可能的速度 ----
        if abs(v_new) > self.max_speed_px_s:
            self._reinit(self.x, f"speed_clamp(|v|>{self.max_speed_px_s:.0f}px/s)")
            self.last_ts = float(ts)
            self.last_dt = dt
            self.samples += 1
            return True

        self.v = v_new
        # ---- 方向确认计数：只在"残差与速度同向"（跟踪平滑）时累加；折返/超调会打断它 ----
        smooth = abs(res) <= 1e-12 or (res > 0.0) == (v_new > 0.0)
        if smooth:
            self.armed_run += 1
            if self.warmup > 0:
                self.warmup -= 1
        else:
            self.armed_run = 0
        self.stale = False
        self.last_reason = "ok"
        self.last_ts = float(ts)
        self.last_dt = dt
        self.samples += 1
        return True

    def on_no_frame(self) -> None:
        """本拍没有有效观测（丢帧 / 不可见 / 重复帧）→ 速度清零并标 stale。"""
        if self.v != 0.0:
            self.v = 0.0
        self.stale = True
        self.armed_run = 0
        self.warmup = max(self.warmup, int(self.reset_confirm_frames))
        self.last_reason = "no_frame"


# --------------------------------------------------------------------------- #
#  开火许可账本（本靶 & 全场）
# --------------------------------------------------------------------------- #
@dataclass
class _Permit:
    """许可账本：★ 许可数分两套计数（旧版把"目标身份"和"计数"混在一起，是 survey §10 第 8 条的坑）。

    · `total`   —— **本任务生命周期**发出的许可次数（不做任何清零）→ `P_AIM.shots_fired`；
    · `window`  —— **本靶射击窗口**内的许可次数，命中后重新武装会清零 → 判 `max_shots_per_target`；
    · `actual`  —— 用 `shoot_count` 增量反推的"实际打了几发"（与许可分别记账，便于对账）。

    不变量：`window ≤ total`。
    """

    window: int = 0                 # 本靶窗口内的许可数（重新武装清零）
    total: int = 0                  # 生命周期许可总数（只增）
    hits: int = 0
    last_allow_at: float = 0.0
    hit_at: float = 0.0
    baseline_shots: int = 0
    actual_shots: int = 0
    rearmed_at: float = 0.0
    hit_latched: bool = False
    #: 本靶被判定的"首次见到"时刻（重新武装时刷新）
    first_seen_at: float = 0.0

    def note_hit(self, n: int, now: float) -> None:
        self.hits += int(n)
        self.hit_at = now
        self.hit_latched = True

    def allowed(self, now: float) -> None:
        self.window += 1
        self.total += 1
        self.last_allow_at = now

    def cool_ok(self, now: float, cooldown_s: float) -> bool:
        if self.last_allow_at <= 0.0:
            return True
        return (now - self.last_allow_at) >= cooldown_s

    def cooldown_left(self, now: float, cooldown_s: float) -> float:
        if self.last_allow_at <= 0.0:
            return 0.0
        return max(0.0, (self.last_allow_at + cooldown_s) - now)

    def rearm(self, now: float, shots_now: int, obs_ts: float) -> None:
        self.window = 0
        self.hit_latched = False
        self.last_allow_at = 0.0
        self.baseline_shots = int(shots_now)
        self.actual_shots = 0
        self.rearmed_at = now
        self.first_seen_at = obs_ts


class AimTask(TaskThread):
    """云台瞄准 + 打击任务（50 Hz）。

    构造签名照 `docs/P1接口约定.md` §0：`(bridge, bb, bus, cfg, *, log, group, clock)`。
    """

    #: 缺的 SCHEMA 键一律走 `cfg.get(key, default)`；`missing_schema_keys` 记录**实际读了默认值**的键，
    #: 供预检门禁/报告一次性列出"需要补进 SCHEMA 的键"（而不是散落各处静默回退）。
    #: `_opt()` 会把每次回退登记进 `_used_defaults`；`setup()` 里用 `_OPTIONAL_KEYS` 做完整性自检
    #: —— **新增一个可选键却忘了登记**会在启动日志里报出来，不会静默漏掉。
    _OPTIONAL_KEYS: tuple[str, ...] = (
        "aim.est_xi", "aim.est_gate_k", "aim.est_gate_base_px", "aim.est_gate_vel_scale",
        "aim.est_jump_floor_px", "aim.reset_confirm_frames", "aim.est_dt_min_s",
        "aim.est_dt_max_s", "aim.est_max_speed_px_s", "aim.lost_target_s",
        "aim.lost_zeros_velocity_s", "aim.rearm_after_lost_s", "aim.max_lead_factor",
        "aim.px_per_m_rail", "aim.fire_box_far_px", "aim.fire_box_near_px",
        "aim.box_w_far_px", "aim.box_w_near_px", "aim.cls_switch_confirm_frames",
        "aim.vision_nominal_fps", "aim.fire_only_blue", "aim.hit_hold_s",
        "aim.rescan_on_lost", "aim.lead_eps_px", "aim.hold_min_s",
    )

    def __init__(self, bridge, bb: Blackboard, bus: EventBus, cfg: Config, *,
                 log: Callable[[str], None] = print, group=None,
                 clock: Callable[[], float] | None = None) -> None:
        super().__init__("AimTask", 50.0, bus=bus, blackboard=bb, logger=log)
        self.bridge = bridge
        self.cfg = cfg
        self.group = group
        if clock is None:
            import time as _time
            clock = _time.monotonic
        self._clock = clock
        self._log_throttle = Throttle(float(cfg.get("logging.rate_limit_s") or 1.0))

        # ---- 配置（一次性读出，50 Hz 下不反复查表） ----
        #: 实际读了内置默认值（= 不在 SCHEMA/robot.yaml 里）的键，`_opt()` 自动登记
        self._used_defaults: set[str] = set()
        g = self._opt
        self._lead_time_s = float(cfg.get("aim.lead_time_s"))
        self._policy = str(cfg.get("aim.fire_policy") or FirePolicy.MOVING_LEAD)
        self._cooldown_s = float(cfg.get("aim.shot_cooldown_ms")) / 1000.0
        self._max_per_target = int(cfg.get("aim.max_shots_per_target"))
        self._max_total = int(cfg.get("aim.max_shots_total"))
        self._lost_frames = int(cfg.get("aim.lost_frames_to_rescan"))
        self._fps = max(1.0, float(g("aim.vision_nominal_fps", 30.0)))
        #: ★ 帧 → 秒换算：`lost_frames_to_rescan` 是**帧**数（视觉 ~30Hz），而本任务的时钟是秒。
        #:   旧版按帧判丢（180 帧≈6s）正是"丢靶期间云台冻结"的根源（survey §10 第 7 条）。
        self._lost_s = max(0.3, self._lost_frames / self._fps)
        self._lost_arm_s = float(g("aim.rearm_after_lost_s", max(0.3, self._lost_s * 0.5)))

        self._box = {
            "far_px": float(g("aim.fire_box_far_px", 14.0)),
            "near_px": float(g("aim.fire_box_near_px", 26.0)),
            # 框宽两端点标定：优先新键，其次旧 dzrknn 段真值（robot.yaml:120-121 = 30/120 px）
            "w_far": float(g("aim.box_w_far_px",
                             cfg.get("dzrknn_dataprocess.box_w_far", 30.0))),
            "w_near": float(g("aim.box_w_near_px",
                              cfg.get("dzrknn_dataprocess.box_w_near", 120.0))),
        }
        self._cls_confirm = max(1, int(g("aim.cls_switch_confirm_frames",
                                         cfg.get("dzrknn_dataprocess.cls_switch_confirm_frames",
                                                 5))))
        self._fire_only_blue = bool(g("aim.fire_only_blue",
                                      cfg.get("dzrknn_dataprocess.fire_only_blue", False)))
        self._hit_hold_s = float(g("aim.hit_hold_s", self._cooldown_s))
        self._rescan_on_lost = bool(g("aim.rescan_on_lost", False))
        self._lead_eps = float(g("aim.lead_eps_px", 0.5))

        rail_span = float(cfg.get("aim.rail_span_m"))
        #: px/m 换算（框宽标定缺失时的兜底）。默认 140 px/m ≈ 0.35m 处 45px 框宽
        #: （现场靶框实测 0.76~0.83 长宽比、框宽几十 px 量级）；它同时决定提前量限幅与
        #: `v_expected` 阈值，**现场标定后应写进配置**（见报告"需要补的 SCHEMA 键"）。
        px_per_m = float(g("aim.px_per_m_rail", 140.0))
        self._max_lead_factor = float(g("aim.max_lead_factor", 0.45))
        self._max_lead_px = max(1.0, self._max_lead_factor * rail_span * px_per_m)
        self._v_expected = float(cfg.get("aim.target_speed_mps")) * px_per_m

        self._estimator = RailEstimator(
            xi=float(g("aim.est_xi", 3.5)),
            gate_k=float(g("aim.est_gate_k", 3.0)),
            gate_base_px=float(g("aim.est_gate_base_px", 3.0)),
            gate_vel_scale=float(g("aim.est_gate_vel_scale", 0.10)),
            jump_floor_px=float(g("aim.est_jump_floor_px", 25.0)),
            reset_confirm_frames=int(g("aim.reset_confirm_frames", 5)),
            dt_min_s=float(g("aim.est_dt_min_s", 0.002)),
            dt_max_s=float(g("aim.est_dt_max_s", 0.50)),
            # ★ 速度夹紧只是"防荒谬值"的兜底（噪声/误检），**不**承担提前量限幅的职责：
            #   提前量由 `_max_lead_px`（轨道半宽）硬夹。把兜底设成 3.5×靶速会在靶看起来
            #   稍快时就反复重置估计器（每次都进 warmup → 闭锁），反而打不出去。取 5× 且 ≥300px/s。
            max_speed_px_s=float(g("aim.est_max_speed_px_s",
                                   max(300.0, 5.0 * self._v_expected))),
        )

        # ---- 运行时状态 ----
        self._permit = _Permit()
        self._last_frame_ts: float | None = None
        self._frame_dt = 1.0 / self._fps
        self._last_obs_ts: float = 0.0
        self._last_obs: OffsetObservation | None = None
        self._seen_prev = False
        self._lost_at: float | None = None
        self._last_class = 0
        self._cls_candidate = 0
        self._cls_run = 0
        self._locked_cls = 0
        self._fire_ready = False
        self._aim_status = AimStatus.IDLE
        self._ammo_prev: int | None = None
        self._shots_seen = 0
        self._hits_total = 0
        self._rescan_requested = False
        self._blocked_reason = REASON_NO_TARGET
        self._last_lead = 0.0
        self._last_tick_at: float | None = None
        self._last_cmd: dict[str, Any] = {}

    # ------------------------------------------------------------------ #
    #  只读接口（测试/遥测）
    # ------------------------------------------------------------------ #
    @property
    def estimator(self) -> RailEstimator:
        return self._estimator

    @property
    def fire_ready_state(self) -> bool:
        return self._fire_ready

    @property
    def permits(self) -> int:
        """本任务生命周期发出的许可总数（= `P_AIM.shots_fired`）。"""
        return self._permit.total

    @property
    def permits_window(self) -> int:
        """本靶射击窗口内的许可数（判 `max_shots_per_target`；命中重新武装会清零）。"""
        return self._permit.window

    @property
    def missing_schema_keys(self) -> list[str]:
        """本任务**实际读了内置默认值**的配置键（= 需要补进 `core/config.py` SCHEMA 的键）。"""
        return sorted(self._used_defaults)

    def _opt(self, key: str, default: Any) -> Any:
        """读一个**可能不在 SCHEMA 里**的可选键；回退到内置默认值时登记进 `_used_defaults`。

        ★ 判据是"既不在 robot.yaml（`_flat`）里、也不在 SCHEMA 里"：
          P2 已经进 SCHEMA 的键（`aim.lead_time_s` / `aim.fire_box_far_px` / …）有配置中心兜底，
          不算"缺键"；只有两处都没有的才是**真缺**（拼错了也不会被告警兜住）。
        """
        if key not in getattr(self.cfg, "_flat", {}) and key not in SCHEMA:
            self._used_defaults.add(key)
        return self.cfg.get(key, default)

    def telemetry(self) -> dict[str, Any]:
        """最近一拍的判定细节（与写进 `P_TELEMETRY[AIM_TELEMETRY_KEY]` 的内容一致）。"""
        return dict(self._last_cmd)

    # ------------------------------------------------------------------ #
    #  单拍
    # ------------------------------------------------------------------ #
    def tick(self) -> None:
        now = self._clock()
        dt = self.period if self._last_tick_at is None else (now - self._last_tick_at)
        if not (0.0 < dt <= 1.0):        # 时钟跳变/首拍 → 退回标称周期
            dt = self.period
        self._last_tick_at = now

        # ---- 1. 读观测（过期的返回 None） ----
        obs = self.bridge.vision(VISION_MAX_AGE_S)
        has_target = bool(obs is not None and _cls_of(obs) in (ShotClass.BLUE, ShotClass.RED))

        # ---- 2. 命中/弹药账本（每拍都读，不依赖是否有目标） ----
        self._account(now)

        # ---- 3. 运动估计（只在**新帧**上更新；同帧重读不重复喂） ----
        self._ingest(obs, has_target)

        # ---- 3b. 类别锁定（连续 N 帧同类才改锁；单帧抖动只吞当拍许可） ----
        self._update_class(obs, has_target)

        # ---- 4. 可见性台账（seen / lost_since / 事件） ----
        self._track_visibility(obs, has_target, now)

        # ---- 5. 安全门 ----
        safety: SafetyState | None = self.bb.get(P_SAFETY)
        manual = False
        try:
            manual = bool(self.bridge.manual_gimbal())
        except Exception:                       # noqa: BLE001  鸭子类型替身可能没实现
            manual = False
        self._aim_status = int(self.bridge.aim_status())

        # ---- 6. 开火判据 ----
        ready, reason = self._evaluate(now, has_target, safety, manual)

        # ---- 7. 提前量 → 云台补丁；开火许可 → 云台补丁（**绝不发 fire_gate**） ----
        lead = self._lead_px(has_target)
        self._publish_lead(lead)
        self._set_fire_ready(ready, now, reason)

        # ---- 8. 写黑盒 + 事件 + 遥测 ----
        self._write_target(now, obs, has_target)
        self._write_aim(now, obs)
        self._write_telemetry(now, lead, dt)

    # ------------------------------------------------------------------ #
    #  观测摄入
    # ------------------------------------------------------------------ #
    def _ingest(self, obs: OffsetObservation | None, has_target: bool) -> bool:
        """把新观测喂给估计器。返回"本拍是否吃到了新帧"。"""
        if not has_target or obs is None:
            self._estimator.on_no_frame()
            return False
        ts = float(_ts_of(obs))
        if self._last_frame_ts is not None and (ts - self._last_frame_ts) <= self._frame_dt * 0.25:
            # 同一帧被 50 Hz 循环读了两次（视觉 30 Hz）→ 不重复更新，也不清零
            return False
        self._estimator.update(float(obs.dx), ts)
        if self._last_frame_ts is not None:
            self._frame_dt = max(1e-3, ts - self._last_frame_ts)
        self._last_frame_ts = ts
        return True

    # ------------------------------------------------------------------ #
    #  可见性
    # ------------------------------------------------------------------ #
    def _track_visibility(self, obs: OffsetObservation | None, has_target: bool,
                          now: float) -> None:
        if has_target and obs is not None:
            self._last_obs = obs
            self._last_obs_ts = float(_ts_of(obs))
            if not self._seen_prev:
                # ★ 顺序要紧：先把"上次丢失了多久"算出来交给重新武装判据，再清 `_lost_at`。
                #   反过来写的话 `_maybe_rearm` 永远看到 `_lost_at is None` → 命中锁存永远解不开
                #   （现场表现：打中一块靶之后第二块靶一直不开火）。这个 bug 在自测里被抓到过一次。
                lost_for = (now - self._lost_at) if self._lost_at is not None else 0.0
                self._lost_at = None
                self._on_target_seen(now, lost_for)
            self._seen_prev = True
            return

        # ---- 不可见：立刻闭锁（旧版这里会冻结云台最长 6s，survey §10 第 7 条） ----
        if self._seen_prev:
            self._lost_at = now
            self._seen_prev = False
            self._rescan_requested = False
            self.bus.publish(EV_TARGET_LOST, source=self.name,
                             dx=int(getattr(self._last_obs, "dx", 0) or 0),
                             at=now)

    def _on_target_seen(self, now: float, lost_for: float = 0.0) -> None:
        """（重新）见到目标：刷新本靶射击窗口 + 触发重新武装判据。"""
        self._maybe_rearm(now, lost_for)
        if self._permit.first_seen_at <= 0.0:
            self._permit.first_seen_at = now
            self._permit.baseline_shots = self._shots_seen
        self.bus.publish(EV_TARGET_FIRST_SEEN, source=self.name,
                         cls=int(self._locked_cls or self._last_class), at=now)

    def _maybe_rearm(self, now: float, lost_for: float) -> None:
        """命中锁存后的重新武装（靶是"来回走的同一个"→ 默认不自动重新武装，见文件头）。

        条件全部满足才解除 `hit_latched`：
          ① `aim.hit_hold_s` 已过；
          ② 目标**重新出现**，且上次丢失时长达 `aim.rearm_after_lost_s`
             （"消失过一段时间又回来"= 新目标/新一块靶的判据）；
          ③ 本靶许可数未到上限（否则重新武装也没意义）。
        ⚠️ 现场"两块独立靶板交替出现"时，这条判据会自动把第二块当成新靶 —— 这是有意的：
          本靶打中后停火，第二块靶不该因为"第一块命中了"而永远打不了。
        """
        p = self._permit
        if not p.hit_latched:
            return
        if (now - p.hit_at) < self._hit_hold_s:
            return
        if lost_for < self._lost_arm_s:
            return
        if p.window >= self._max_per_target:
            return
        p.rearm(now, self._shots_seen, now)
        self._log(f"命中锁存解除：目标重新出现（丢失 {lost_for:.2f}s ≥ "
                  f"{self._lost_arm_s:.2f}s）→ 本靶许可窗口重新计数"
                  f"（基线发数 {p.baseline_shots}，生命周期许可累计 {p.total}）")

    # ------------------------------------------------------------------ #
    #  命中 / 弹药 / 发数对账
    # ------------------------------------------------------------------ #
    def _account(self, now: float) -> None:
        hits = int(self.bridge.take_hits() or 0)
        if hits > 0:
            self._hits_total += hits
            self._permit.note_hit(hits, now)         # ★ 命中即停火（锁存）
            self.bus.publish(EV_HIT_CONFIRMED, source=self.name,
                             hits=hits, total=self._hits_total, at=now)
            self._log(f"★ 命中确认 +{hits}（累计 {self._hits_total}）→ 本靶停火，省弹药")

        self._shots_seen = int(self.bridge.shoot_count() or 0)
        self._permit.actual_shots = max(0, self._shots_seen - self._permit.baseline_shots)

        ammo = self.bridge.ammo()
        if ammo != self._ammo_prev:
            # 只在"两个已知值之间变化"时报事件：首次拿到弹药值（None → 10）是**初始化**不是变化，
            # 否则每次起栈都会刷一条假告警。
            if self._ammo_prev is not None and ammo is not None:
                self.bus.publish(EV_AMMO_CHANGED, source=self.name,
                                 ammo=ammo, prev=self._ammo_prev, at=now)
                self._log(f"弹药变化：{self._ammo_prev} → {ammo}")
            self._ammo_prev = ammo

    # ------------------------------------------------------------------ #
    #  开火判据
    # ------------------------------------------------------------------ #
    def _fire_box_now(self, obs: OffsetObservation | None) -> float:
        box_w = float(getattr(obs, "box_w", 0) or 0)
        return fire_box_px(box_w, near_px=self._box["near_px"], far_px=self._box["far_px"],
                           box_w_far=self._box["w_far"], box_w_near=self._box["w_near"])

    def _lead_px(self, has_target: bool) -> float:
        """提前量 `lead_px = v_dx × lead_time_s`（限幅，防把目标外推进画面外）。"""
        if not has_target:
            return 0.0
        est = self._estimator
        if not est.init or est.stale:
            return 0.0                       # 断流/未初始化 → 不外推旧速度
        lead = est.v * self._lead_time_s
        lim = self._max_lead_px
        return max(-lim, min(lim, lead))

    def _evaluate(self, now: float, has_target: bool, safety: SafetyState | None,
                  manual: bool) -> tuple[bool, str]:
        """返回 `(fire_ready, reason)`。★ 任一硬门不满足 → 0（失败偏安全）。"""
        # ---- 硬门 1：安全（P_SAFETY 缺失按不安全处理） ----
        if safety is None:
            return False, REASON_SAFETY
        if not safety.ok:
            return False, REASON_SAFETY
        # ---- 硬门 2：遥控接管云台 ----
        if manual:
            return False, REASON_MANUAL
        # ---- 硬门 3：目标新鲜 ----
        if not has_target:
            return False, (REASON_TARGET_LOST if self._lost_at is not None else REASON_NO_TARGET)
        # ---- 硬门 4：类别确认（连续 cls_confirm 帧同类，吸收单帧抖动） ----
        if self._cls_run < self._cls_confirm:
            return False, REASON_CLASS_UNSTABLE
        if self._fire_only_blue and self._locked_cls != ShotClass.BLUE:
            return False, REASON_CLASS_NOT_ALLOWED
        # ---- 硬门 5：命中锁存（已中即停） ----
        if self._permit.hit_latched:
            return False, REASON_HIT_LATCHED
        # ---- 硬门 6：弹药 / 预算 ----
        if self._permit.window >= self._max_per_target:
            return False, REASON_BUDGET_PER_TARGET
        if self._shots_seen >= self._max_total:
            return False, REASON_BUDGET_TOTAL
        ammo = self._ammo_prev
        if ammo is not None and int(ammo) <= 0:
            return False, REASON_AMMO_EMPTY
        # ---- 硬门 7：速度估计可信（折返/重捕后要收敛时间） ----
        est = self._estimator
        if est.stale or not est.init:
            return False, REASON_ESTIMATOR_STALE
        if not est.armed:
            return False, REASON_ESTIMATOR_WARMUP
        # ---- 策略相关判据 ----
        obs = self._last_obs
        box = self._fire_box_now(obs)
        dx = float(getattr(obs, "dx", 0) or 0)
        dy = float(getattr(obs, "dy", 0) or 0)
        lead = self._lead_px(True)
        pred = dx + lead

        if self._policy == FirePolicy.MOVING_LEAD:
            if abs(pred) > box:
                return False, REASON_OUT_OF_BOX
        elif self._policy == FirePolicy.SWEEP:
            # 扫过式：要求"下一帧即将进入框内"（|pred| 在框内且提前量把落点拉向零点）
            if abs(pred) > box:
                return False, REASON_OUT_OF_BOX
            if abs(pred) >= abs(dx) and abs(est.v) > 1e-6:
                return False, REASON_OUT_OF_BOX
        elif self._policy == FirePolicy.TURNAROUND:
            # ★ 本靶无端点停顿（瞬时折返）→ 该策略等价不可用；只认 |v| 降到阈值以下
            if abs(est.v) > max(1.0, self._v_expected * 0.25):
                return False, REASON_POLICY_TURNAROUND
            if abs(pred) > box:
                return False, REASON_OUT_OF_BOX
        elif self._policy == FirePolicy.STATIC:
            if abs(est.v) > max(1.0, self._v_expected * 0.25):
                return False, REASON_POLICY_STATIC
            if abs(pred) > box:
                return False, REASON_OUT_OF_BOX
        else:
            return False, REASON_POLICY_UNKNOWN
        # 垂直方向用同一框（旧版同语义：动态框 x/y 同宽）
        if abs(dy) > box:
            return False, REASON_OUT_OF_BOX
        # ---- 硬门 8：冷却 ----
        if not self._permit.cool_ok(now, self._cooldown_s):
            return False, REASON_COOLDOWN
        return True, REASON_OK

    # ------------------------------------------------------------------ #
    #  写话题（唯一允许的"钥匙"是 fire_ready / lead）
    # ------------------------------------------------------------------ #
    def _publish_lead(self, lead: float) -> None:
        self._last_lead = float(lead)
        try:
            self.bridge.publish_lead(float(lead))
        except Exception:                      # noqa: BLE001  话题缺失不该杀任务
            pass

    def _set_fire_ready(self, ready: bool, now: float, reason: str) -> None:
        """★ 每拍都发（锁存 + 保活）：云台补丁重启/丢帧后不会卡在旧的许可上。

        ⚠️ 这里**只**碰 `fire_ready`。`fire_gate` 是 MissionTask 的钥匙，本任务绝不写。

        ★★ 许可计数发生在**冷却到点那一拍**，而不是"False→True 跳变那一拍"。

        为什么必须这样（否则单靶只能打 1 发）：本任务每拍都在发 `fire_ready`，
        云台侧只要收到 1 就会在条件满足时开火。如果许可只在跳变沿计数，
        第一发之后 `fire_ready` 一直是 1（只要目标还在框内），`_permit.window` 就永远停在 1 ——
        `max_shots_per_target=3` 形同虚设、且**第二发永远不会被授权**。
        改成"冷却到点即记一次许可"后：1 发 = 1 个 `shot_cooldown_ms` 窗口，
        该窗口内本任务不再重复计数，云台自己的冷却也正好用掉一个窗口。
        不变量：**一拍最多记一次许可**（记完 `cool_ok` 立即为假）。
        """
        ready = bool(ready)
        if not ready:
            self._blocked_reason = reason
            if self._fire_ready:
                self._fire_ready = False
                self.bus.publish(EV_FIRE_BLOCKED, source=self.name, reason=reason,
                                 lead_px=self._last_lead, at=now)
                self._log(f"闭锁 fire_ready=0：{_REASON_TEXT.get(reason, reason)}")
            elif self._blocked_reason != REASON_OK:
                if self._log_throttle.allow("blocked:" + reason, now):
                    self._log(f"不开火：{_REASON_TEXT.get(reason, reason)}")
            self._emit_ready(False)
            return

        # ---- ready=True ----
        first_permit = self._permit.last_allow_at <= 0.0
        grant = first_permit or self._permit.cool_ok(now, self._cooldown_s)
        if grant:
            first_for_this_target = first_permit
            self._permit.allowed(now)
            if first_for_this_target:
                # 本靶的**第一发**：一个独立事件（EV_FIRE_ALLOWED），便于复盘时间线
                self.bus.publish(EV_FIRE_ALLOWED, source=self.name, lead_px=self._last_lead,
                                 permits=self._permit.window, at=now)
            self._log(f"★ 瞄准合格 → fire_ready=1（许可 {self._permit.window}"
                      f"/{self._max_per_target}，累计 {self._permit.total}，"
                      f"lead {self._last_lead:+.1f}px）")
        elif not self._fire_ready:
            self._log(f"瞄准合格但冷却未过（剩 {self._permit.cooldown_left(now, self._cooldown_s):.2f}s）"
                      f"→ 保持 ready，不重复计数")
        self._fire_ready = True
        self._blocked_reason = REASON_OK
        self._emit_ready(True)

    def _emit_ready(self, ready: bool) -> None:
        try:
            self.bridge.publish_fire_ready(bool(ready))
        except Exception:                      # noqa: BLE001  话题缺失不该杀任务
            pass

    # ------------------------------------------------------------------ #
    #  写黑盒
    # ------------------------------------------------------------------ #
    def _write_target(self, now: float, obs: OffsetObservation | None,
                      has_target: bool) -> None:
        obs = obs if has_target else None
        half = self._max_lead_px / max(1e-6, float(self.cfg.get("aim.max_lead_factor", 0.45)))
        s_rail = None
        if obs is not None and half > 1e-6:
            s_rail = max(0.0, min(1.0, 0.5 + float(obs.dx) / (2.0 * half)))
        state = TargetState(
            seen=bool(has_target),
            dx=int(getattr(obs, "dx", 0) or 0),
            dy=int(getattr(obs, "dy", 0) or 0),
            cls=int(getattr(obs, "cls", 0) or 0),
            box_w=int(getattr(obs, "box_w", 0) or 0),
            seen_at=float(_ts_of(obs)) if obs is not None else self._last_obs_ts,
            lost_since=float(self._lost_at or 0.0),
            s_rail=s_rail,
            v_rail=float(self._estimator.v),
            samples=int(self._estimator.samples),
            notes=(f"v={self._estimator.v:+.1f}px/s lead={self._last_lead:+.1f}px "
                   f"warmup={self._estimator.warmup} n={int(self._estimator.samples)} "
                   f"resets={int(self._estimator.resets)} {self._estimator.last_reason}"),
        )
        self.bb.set(P_TARGET, state, source=self.name)

    def _write_aim(self, now: float, obs: OffsetObservation | None) -> None:
        cooled_until = (self._permit.last_allow_at + self._cooldown_s
                        if self._permit.last_allow_at > 0.0 else 0.0)
        state = AimState(
            status=int(self._aim_status),
            fire_gate=bool(self._fire_ready),        # P_AIM.fire_gate 语义 = "本任务已许可"
            lead_px=float(self._last_lead),
            shots_fired=int(self._permit.total),
            hits=int(self._hits_total),
            ammo=self._ammo_prev,
            cooled_until=float(cooled_until),
            last_reason=self._blocked_reason,
        )
        self.bb.set(P_AIM, state, source=self.name)

    def _write_telemetry(self, now: float, lead: float, dt: float) -> None:
        est = self._estimator
        self._last_cmd = {
            "ready": bool(self._fire_ready),
            "blocked_reason": self._blocked_reason,
            "policy": self._policy,
            "status": AimStatus.name(self._aim_status),
            "seen": bool(self._seen_prev),
            "lost_s": (now - self._lost_at) if self._lost_at is not None else 0.0,
            "dx": int(getattr(self._last_obs, "dx", 0) or 0) if self._seen_prev else 0,
            "dy": int(getattr(self._last_obs, "dy", 0) or 0) if self._seen_prev else 0,
            "cls": int(self._locked_cls),
            "cls_run": int(self._cls_run),
            "box_px": float(self._fire_box_now(self._last_obs if self._seen_prev else None)),
            "v_px_s": float(est.v),
            "lead_px": float(lead),
            "pred_px": float(lead + (getattr(self._last_obs, "dx", 0) or 0) if self._seen_prev else 0.0),
            "v_error": float(est.v - self._v_expected),
            "est_armed": bool(est.armed),
            "est_warmup": int(est.warmup),
            "est_armed_run": int(est.armed_run),
            "est_flips": int(est.flips),
            "est_samples": int(est.samples),
            "est_resets": int(est.resets),
            "est_jumps": int(est.jump_rejections),
            "est_reason": est.last_reason,
            "permits": int(self._permit.window),
            "permits_total": int(self._permit.total),
            "max_per_target": int(self._max_per_target),
            "shots_seen": int(self._shots_seen),
            "actual_shots": int(self._permit.actual_shots),
            "max_total": int(self._max_total),
            "hits": int(self._hits_total),
            "hit_latched": bool(self._permit.hit_latched),
            "ammo": self._ammo_prev,
            "cool_left_s": self._permit.cooldown_left(now, self._cooldown_s),
            "rescan_requested": bool(self._rescan_requested),
            "dt": float(dt),
            "at": now,
        }
        snap = self.bb.get(P_TELEMETRY)
        data = dict(snap) if isinstance(snap, dict) else {}
        data[AIM_TELEMETRY_KEY] = dict(self._last_cmd)
        self.bb.set(P_TELEMETRY, data, source=self.name)

        # ---- 丢靶回扫：默认**关**（理由见下），开着才发 ----
        if (self._lost_at is not None and not self._rescan_requested
                and (now - self._lost_at) >= self._lost_s):
            self._rescan_requested = True
            if self._rescan_on_lost:
                # ⚠️ 只发 2（持续），**绝不发 1**（readme【十】31：到点发 1 会把持续扫描降级成单趟）；
                #    发 0 会在目标短暂消失时彻底停扫，是更糟的失败方向。
                #    C++ 侧 TRACK 期间忽略 scan_request、SCAN 期间 ≥1 只改模式不重置进度（survey §2.2）
                #    → 发 2 无副作用；云台侧的 lost_frames_to_rescan 自己会处理回扫。
                try:
                    self.bridge.publish_scan_request(2)
                    self._log(f"丢靶 {now - (self._lost_at or now):.2f}s ≥ {self._lost_s:.2f}s"
                              f" → 重发 scan_request=2（持续扫描，不发 1）")
                except Exception:              # noqa: BLE001
                    pass

    # ------------------------------------------------------------------ #
    #  类别锁定
    # ------------------------------------------------------------------ #
    def _update_class(self, obs: OffsetObservation | None, has_target: bool) -> None:
        """连续 N 帧同类才改锁（旧版 `cls_switch_confirm_frames` 同语义，survey §10 第 10 条）。

        ⚠️ 单帧抖动**不**改 `_locked_cls`，但该帧的 `_cls_run` 也不增长
          （`_cls_run < _cls_confirm` 期间不允许开火）—— 这就是"1 帧变类不应立即闭锁、
          而连续 N 帧才改锁"的落地：抖动只吞掉当拍许可，后续帧回到原类即恢复。
        """
        cls = _cls_of(obs) if has_target and obs is not None else ShotClass.NONE
        if cls == ShotClass.NONE:
            # ★ 目标不可见**不清除** `_locked_cls`（靶还是那个靶），只把确认计数清零：
            #   重新出现后必须重新连续确认 N 帧，但不必重新"发现"类别。
            self._cls_candidate = ShotClass.NONE
            self._cls_run = 0
            self._last_class = cls
            return
        if cls == self._cls_candidate:
            self._cls_run += 1
        else:
            self._cls_candidate = cls
            self._cls_run = 1
        if self._cls_run >= self._cls_confirm:
            if self._locked_cls != cls:
                self._log(f"类别锁定改锁：{self._locked_cls} → {cls}"
                          f"（连续 {self._cls_run} 帧 ≥ {self._cls_confirm}）")
            self._locked_cls = cls
        self._last_class = cls

    # ------------------------------------------------------------------ #
    #  生命周期
    # ------------------------------------------------------------------ #
    def setup(self) -> None:
        blocks = 2.0 * (self._max_lead_px / max(1e-9, self._max_lead_factor))
        self._log(f"启动：策略 {self._policy}，提前量 {self._lead_time_s:g}s，"
                  f"α-β(ξ={self._estimator.xi:g}, α={self._estimator.alpha:.3f}, "
                  f"β={self._estimator.beta:.3f})，开火框 {self._box['far_px']:.0f}~"
                  f"{self._box['near_px']:.0f}px（框宽 {self._box['w_far']:.0f}~"
                  f"{self._box['w_near']:.0f}px），提前量限幅 ±{self._max_lead_px:.1f}px"
                  f"（≈轨道全宽 {blocks:.0f}px），单靶 {self._max_per_target} 发 / "
                  f"全场 {self._max_total} 发，冷却 {self._cooldown_s * 1000:.0f}ms，"
                  f"丢靶判定 {self._lost_s:.2f}s（{self._lost_frames} 帧 @ {self._fps:g}fps）")
        missing = self.missing_schema_keys
        if missing:
            self._log("⚠️ 以下配置键不在 SCHEMA 里（本任务读的是内置默认值，需补进 "
                      "core/config.py 的 SCHEMA）: " + ", ".join(missing))
        unregistered = [k for k in missing if k not in self._OPTIONAL_KEYS]
        if unregistered:
            self._log("⚠️ 上面的键里有未登记进 `AimTask._OPTIONAL_KEYS` 的（自检提示）: "
                      + ", ".join(unregistered))

    def teardown(self) -> None:
        """退出必闭锁（**关的是自己那把钥匙**，不碰 fire_gate）。"""
        try:
            self.bridge.publish_fire_ready(False)
        except Exception:                      # noqa: BLE001  收尾不许抛
            pass
        self._fire_ready = False
        try:
            self.bridge.publish_lead(0.0)
        except Exception:                      # noqa: BLE001
            pass


# --------------------------------------------------------------------------- #
#  小工具（鸭子类型替身可能不给 ts / cls 属性）
# --------------------------------------------------------------------------- #
def _ts_of(obs: OffsetObservation | None) -> float:
    return float(getattr(obs, "ts", 0.0) or 0.0)


def _cls_of(obs: OffsetObservation | None) -> int:
    if obs is None:
        return ShotClass.NONE
    try:
        return int(getattr(obs, "cls", ShotClass.NONE))
    except (TypeError, ValueError):
        return ShotClass.NONE
