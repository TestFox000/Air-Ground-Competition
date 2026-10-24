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
★★ 云台扫描开关的**单写者**（与锁止配套）
===============================================================================
| 话题                     | 唯一写者        | 语义                                                      |
|--------------------------|-----------------|-----------------------------------------------------------|
| `/nav/scan_request`      | **本任务**      | `aim.gimbal_lock_enabled=true`（默认）时**只有 AimTask 写** |
| `/aim/lead_x`            | 本任务          | 提前量（px）→ 云台补丁                                     |

- `aim.gimbal_lock_enabled=true`（默认）时：`MissionTask` **一个值都不发**
  （它的 `mission.scan_mode` 在该模式下 **inert**，只影响横幅文案；旧行为需
  `aim.gimbal_lock_enabled=false` 才回退）。为什么必须这样（run11 实证的顺序）：
  云台锁止 `0`（阶段=None）→ mission「扫描模式 continuous → `2`」→ 云台起转搜索 ——
  Mission 的 `2` 在**巡迹全程**压住了 Aim 的 `0`；而 C++ 侧 `scan_request` 是**电平保持**
  语义，Aim 又有 `_gimbal_cmd` 去重（值没变就不再重发 `0`）⇒ 云台实际一直在转。
- 锁止 = `scan_request=0` ⇒ C++ 进 `AIM_IDLE` ⇒ `CaremaMontorControl()` 每帧主动**回正到
  车头正前**（`Position_1 = center + 扫描行程中点`、`Position_0 = center + 基准俯仰`；
  证据：`src/dzrknn_dataprocess/src/dzrknn_dataprocess.cpp:662-684`，注释原文
  「旧版在 IDLE 时直接 return，于是云台【冻结在最后一次的任意姿态】上」）。
  ⇒ 用户要求「射击点之前/离开之后云台回正到向前看并保持」**由 C++ 的 IDLE 归位实现**，
  Python 侧只需保证**锁止边沿确实发了一次 0**（TRACK→锁止 那一次不能因为
  "值没变"而被去重吞掉，见 `_update_gimbal_gate` 的 `lock_edge`）。

===============================================================================
★ 旧版 15 条缺口 → 本任务的对应处置
===============================================================================
| 旧缺口（survey §10）                                  | 本实现                                        |
|---|---|
| ① 无速度估计，只有滤波后单帧差分（px/帧，无时间戳）    | ★ 时间戳驱动的**长基线端点差分**（0.4s 窗口），见 `RailEstimator` |
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
但折返瞬间的速度估计**必然**是错的（旧速度指向旧方向 + 新测量指向反方向），若继续外推：
`lead_px = v_dx_old × 0.35` = +10.5px 而真实目标已反向 → 预测落点偏 21px > 开火框 → 必打飞。
因此本实现有**三道闸**：
1. **跳变门**（`aim.est_gate_*` / `est_jump_floor_px`）：残差 > 门限 → 估计器**重建且 v 清零**；
2. **折返检测**（`v · Δz < 0`）：跳变门管不住平滑折返（本靶一帧只差 2px），所以单独判
   "新样本方向与当前速度相反" → **重建基线 + 位置取测量值 + v 清零 + `resets/flips` 计数**
   （`_block_warmup()`）。位置不重建的话，闭锁期结束时会给出"方向对、量级翻倍"的提前量
   （实测 −61px/s vs 正确 −30px/s）；
3. **方向确认**（`aim.reset_confirm_frames`）：重建后必须连续 N 帧同向才重新 `armed`
   （期间 `target.v` 可能已经很大，但**不允许开火**）。

数值证据见 `tests/test_aim.py` 第 3 组：折返那一拍 `resets+1`、`lead_px` 归零、
闭锁 5 拍（= `reset_confirm_frames`）后重新许可，且那时 `v≈−30px/s`（方向已翻、量级正确）。

★ **开火许可的两种"时间"语义**（都在 `_set_fire_ready()` 里，务必分清）：
 · `aim.shot_cooldown_ms`：两发许可之间的**最小间隔** —— 发数纪律，硬约束；
 · `aim.hold_min_s`：许可电平的**最短保持** —— 纯通信可靠性（云台 100Hz，只亮一拍的
   许可可能被漏采）。两者叠加后仍然严格满足"相邻许可 ≥ `shot_cooldown_ms`"。

⚠️ **本靶的真实物理上限不在滤波精度，而在 `lead_time_s` 的标定**：
`v=4cm/s × T=0.35s = 1.4cm`；提前量时间写错 0.2s，落点就偏 0.8cm（=0.35m 处约 11px）——
接近一个开火框宽。上线前必须用实测延迟（0.5~0.9s）核对 `aim.lead_time_s`。

★ 时间来源一律走构造注入的 `clock`（默认真实 `time.monotonic`）：冷却、丢靶计时、
   限流日志都依赖它；测试用 `FakeClock` 快进。**任务内不得直接调 `time.time()`**。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable

from ..core.blackboard import (P_AIM, P_AIM_RED, P_MISSION_PHASE, P_SAFETY,  # noqa: F401
                               P_STOP_POINT, P_TARGET, P_TELEMETRY, Blackboard)
from ..core.config import SCHEMA, Config
from ..core.events import (EV_AMMO_CHANGED, EV_FIRE_ALLOWED, EV_FIRE_BLOCKED,
                           EV_HIT_CONFIRMED, EV_TARGET_FIRST_SEEN, EV_TARGET_LOST, EventBus)
from ..core.logging import Throttle
from ..core.state import (AimState, SafetyState, StopKind, StopPhase,  # noqa: E402
                          StopPointState, TargetState)
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
    r"""一维运动估计：对 `dx`（画面水平偏移）估计速度 `v`（px/s），带跳变门 + 方向确认。

    ★ 位置走 α 滤波（`x = x_pred + α·res`），**速度走长基线端点差分**：
        v = (z_new − z_old) / (t_new − t_old)，基线窗口 `vel_window_s`（默认 0.40s）

    为什么速度不用 α-β 的 `v += β·res/dt`（上一版就是这么写的，被实测否掉）：
      · 本靶 4cm/s ÷ 30fps ≈ **0.19 px/帧** —— 单帧位移**小于 1px 量化步长**，`res/dt`
        的主要成分是量化噪声而不是运动；而 `β/dt` 高达 11.4 /s，噪声被放大成 ±5px/s：
        实测 20fps 台阶序列把 30px/s 估成 **25.31px/s（−15.6%）**，刚好越过容差。
      · 端点差分对**匀速运动零稳态误差**（无量化时精确等于真值，1e-9 级），
        对量化噪声按 1/W 抑制（W=12 帧 @30fps → ±0.3px/s），两个要求同时满足。
      · 代价是滞后 ≈ W/2 = 0.2s：本靶只在折返时改变速度，而折返有专门的检测与闭锁
        （见下），滞后可接受。

    位置支路增益仍由**跟踪指数** ξ 解析给出（不是拍脑袋的调参）：
        α = 2(2ξ−1) / (ξ(ξ+1))，  β = 6 / (ξ(ξ+1))（β 保留用于自检/兼容，速度不再用它）
    ξ=3.5（默认）→ α=0.7619、β=0.3810。

    ★ 四道防线（旧版一条都没有）：

    1. **dt 来自观测时间戳**（`OffsetObservation.ts`），不是固定 1/30s。
       旧版 `vx = dx_filt − dx_prev_filt` 是"px/帧"，帧率一变（视觉 30Hz → 丢帧 → 20Hz）
       速度含义就变了（survey §10 第 1 条）。
    2. **跳变门**：残差 > `max(jump_floor_px, gate_k·σ_meas + gate_vel_scale·|v|·dt)`
       → 判定"不是同一个运动"（目标被遮挡后重现、类别换靶、坐标跳变），
       **重新初始化且 v 清零** —— 绝不把跳变量当成 3σ 以上的速度。
    3. **方向确认 `warmup`**：重新初始化后 `warmup = reset_confirm_frames`，每来一帧减一；
       `warmup > 0` 期间**不允许开火**（`armed` 为假）。这是折返后"给收敛时间"的落地方式：
       折返瞬间 v 已清零、方向刚翻，此时提前量毫无意义。
    4. **零位移帧（Δz=0）单独一条支路**：本靶单帧位移 < 1px 量化步长，
       `dx` 连续好几帧一模一样是**常态**。把 Δz=0 当成"减速到 0"会让估计器永远停在 0；
       把 Δz=0 帧丢进端点差分又会让基线被"没动的那几帧"拉平。
       所以：**不动 v、不动基线，只当"目标还在、无反证"推进方向确认**。

    ⚠️ `on_no_frame()` 与"目标不可见"是两件事：
      · `on_no_frame()`（本帧无有效观测）→ v 清零、标 stale：断流后**旧速度不可外推**；
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
    #: ★ 速度基线窗口（秒）：越长噪声越小、滞后越大。0.40s @30fps ≈ 12 帧
    vel_window_s: float = 0.40
    #: ★ 折返判据的速度下限（px/s）：低于它的"反向台阶"算量化抖动，不触发折返闭锁
    flip_min_speed_px_s: float = 2.0

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
    #: 上一个**被采纳**的测量值（判"零位移帧"用；None = 还没吃到帧）
    last_z: float | None = None
    #: 速度基线 (ts, z)，长度受 `vel_window_s` 限制
    base: list = field(default_factory=list)

    def __post_init__(self) -> None:
        self.alpha, self.beta = self.gains(self.xi)

    # ------------------------------------------------------------------ #
    @staticmethod
    def gains(xi: float) -> tuple[float, float]:
        """跟踪指数 ξ → (α, β)。ξ 越小越跟手、噪声越大。（位置支路用 α；β 保留兼容）"""
        xi = max(0.5, float(xi))
        alpha = 2.0 * (2.0 * xi - 1.0) / (xi * (xi + 1.0))
        beta = 6.0 / (xi * (xi + 1.0))
        return alpha, beta

    @property
    def armed(self) -> bool:
        """估计是否可信到可以用来开火（初始化过 + 未断流 + 方向已确认）。"""
        return bool(self.init) and not self.stale and self.warmup <= 0

    # ------------------------------------------------------------------ #
    def _seed(self, z: float, ts: float) -> None:
        """把位置/速度基线重建到 `z`（速度清零、进入方向确认期）。**不计 resets**。"""
        self.x = float(z)
        self.v = 0.0
        self.init = True
        self.stale = False
        self.warmup = max(0, int(self.reset_confirm_frames))
        self.armed_run = 0
        self.last_z = float(z)
        self.base = [(float(ts), float(z))]

    def _reinit(self, z: float, ts: float, reason: str) -> None:
        """重新初始化（跳变/断流/荒谬速度）：位置取测量值、**速度清零**、重进确认期。"""
        self._seed(z, ts)
        self.resets += 1
        self.last_reason = reason

    def _block_warmup(self, z: float, ts: float, reason: str) -> None:
        """★ 折返处置：**速度清零 + 位置基线重建 + 重新确认**，绝不把折返前的速度外推。

        为什么需要单独一条（跳变门管不住它）：
          本靶移速只有 4cm/s（≈0.2px/帧），折返是**平滑**的 —— 一帧残差只有 2px，
          远低于 `jump_floor_px=25` 的跳变门。若只靠跳变门，估计器会平滑地把 +30px/s
          翻成 −14px/s 而**不加任何闭锁**，此时 `lead` 仍指向旧方向 → 必打飞。

        为什么位置也要重建（`x = z`）：
          只清 v 不清 x，滤波器会拿着"折返前那个偏高的 x"去追新测量，
          速度会一路冲到 **−61px/s**（实测）—— 闭锁期一结束就给出方向正确但**量级翻倍**的
          提前量。重建位置后同样条件下是 −30.1px/s（±0.3%）。

        `resets` 与 `flips` 都 +1：前者是"估计器重置"的统一证据字段（测试/遥测都用它），
        后者单独记折返次数。
        """
        self._seed(z, ts)
        self.resets += 1
        self.flips += 1
        self.last_reason = reason

    # ---- 速度基线（长基线端点差分） ----
    def _trim(self, ts: float) -> None:
        cut = float(ts) - max(1e-3, float(self.vel_window_s))
        while len(self.base) > 2 and self.base[0][0] < cut:
            self.base.pop(0)

    def _slope(self) -> float:
        if len(self.base) < 2:
            return 0.0
        t0, z0 = self.base[0]
        t1, z1 = self.base[-1]
        dt = t1 - t0
        if dt <= self.dt_min_s:
            return 0.0
        return (z1 - z0) / dt

    def _touch(self, ts: float, dt: float) -> None:
        self.last_ts = float(ts)
        self.last_dt = float(dt)
        self.samples += 1

    # ------------------------------------------------------------------ #
    def update(self, dx: float, ts: float) -> bool:
        """喂一帧观测。返回是否被**采纳**（False = 同帧重读/乱序 → 完全不动状态）。"""
        z = float(dx)
        ts = float(ts)
        if self.last_ts is not None:
            dt = ts - self.last_ts
            if dt <= self.dt_min_s:
                self.last_reason = "dup_frame"          # 同帧重读/时钟不动 → 丢弃
                return False
            if dt > self.dt_max_s:
                self._reinit(z, ts, f"gap_dt={dt:.3f}s")    # 断流：旧速度不可外推
                self._touch(ts, dt)
                return True
        else:
            dt = 0.0

        if not self.init:
            # ★ 第一帧只建立基线，**不**从 0 推速度（否则第一拍会把"目标在 dx 处"当成巨大速度）
            self._seed(z, ts)
            self._touch(ts, 0.0)
            self.last_reason = "first"
            return True

        # ---- ★ 零位移帧（第四道防线，见类文档） ----
        if self.last_z is not None and z == self.last_z:
            self.stale = False
            self.armed_run += 1
            if self.warmup > 0:
                self.warmup -= 1
            self.last_reason = "no_motion"
            self._touch(ts, dt)
            return True

        # ---- 预测 + 跳变门 ----
        x_pred = self.x + self.v * dt
        res = z - x_pred
        gate = self.gate_k * self.gate_base_px + self.gate_vel_scale * abs(self.v) * dt
        gate = max(gate, self.jump_floor_px)
        if abs(res) > gate:
            self.jump_rejections += 1
            self._reinit(z, ts, f"jump_res={res:.1f}px>gate={gate:.1f}px")
            self._touch(ts, dt)
            return True

        # ---- 位置：α 滤波；速度：长基线端点差分 ----
        self.x = x_pred + self.alpha * res
        self.base.append((ts, z))
        self._trim(ts)
        v_new = self._slope()

        # ---- ★ 折返：新样本的运动方向与当前速度相反 → 重建 + 重新确认 ----
        dz = z - float(self.last_z)
        if abs(self.v) >= self.flip_min_speed_px_s and self.v * dz < 0.0:
            self._block_warmup(z, ts, f"turnaround v {self.v:+.1f}->{v_new:+.1f}px/s")
            self._touch(ts, dt)
            return True

        # ---- 荒谬速度（噪声/误检）：宁可重建，也不外推一个不可能的速度 ----
        if abs(v_new) > self.max_speed_px_s:
            self._reinit(z, ts, f"speed_clamp(|v|>{self.max_speed_px_s:.0f}px/s)")
            self._touch(ts, dt)
            return True

        self.v = v_new
        self.last_z = z
        self.stale = False
        self.armed_run += 1
        if self.warmup > 0:
            self.warmup -= 1
        self.last_reason = "ok"
        self._touch(ts, dt)
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
        "aim.est_dt_max_s", "aim.est_max_speed_px_s", "aim.est_vel_window_s",
        "aim.est_flip_min_speed_px_s", "aim.rearm_after_lost_s", "aim.max_lead_factor",
        "aim.px_per_m_rail", "aim.fire_box_far_px", "aim.fire_box_near_px",
        "aim.box_w_far_px", "aim.box_w_near_px", "aim.cls_switch_confirm_frames",
        "aim.vision_nominal_fps", "aim.fire_only_blue", "aim.hit_hold_s",
        "aim.stop_on_hit",
        "aim.rescan_on_lost", "aim.lead_eps_px", "aim.hold_min_s",
        # 云台锁止四键已登记进 SCHEMA（core/config.py），这里同步登记以便自检一致
        "aim.gimbal_lock_enabled", "aim.gimbal_lock_scan_request",
        "aim.gimbal_seek_scan_request", "aim.gimbal_seek_on_shoot_stop_only",
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
        # ---- 红面早退（≥N 秒锁定红 ⇒ 置黑盒标志，Mission 提前离站）----
        #   为什么放在 Aim：类别锁定（`_update_class`）与"连续 N 帧同类"是这里独占的知识，
        #   Mission 只该看到"结论"（确认红面）而不是自己重算一遍类别抖动逻辑。
        #   阈值缺省 3.0 s = 用户口径；`run_s` 只在**射击停站的 ACQUIRE/FIRE 段**累计，
        #   离开停站立即归零（否则第二个停站一进去就被误判早退，见 `P_AIM_RED` 契约）。
        self._red_early_leave_s = max(0.0, float(g("aim.red_early_leave_s", 3.0)))
        self._red_run_s = 0.0               # 本次停站内"锁定红"累计秒数
        self._red_confirmed = False         # 是否已到达阈值（写 `P_AIM_RED`）
        self._hit_hold_s = float(g("aim.hit_hold_s", self._cooldown_s))
        # ★ 2026-10-24 用户口径：射击站要「打满 10 发」，命中不提前停火。
        #   false 时 `hit_latched` 只作遥测/事件记账，不再闭锁 `fire_ready`。
        self._stop_on_hit = bool(g("aim.stop_on_hit", True))
        self._rescan_on_lost = bool(g("aim.rescan_on_lost", False))
        # ---- 云台巡迹锁止（用户 2026-10-11 指令）----
        # 用户原话：「循迹期间云台保持不动（锁止），到达最佳射击点（停站）之后云台才开始旋转、
        # 寻找目标」。实车首跑失败形态：`mission.scan_mode='continuous'` 让 mission 开局就发
        # `scan_request=2`，整趟巡迹云台一直在转（瞄 SCAN 全程）→ 视觉不稳 + 到站 G10 打 3 发 0 命中。
        # 因此 aim（云台任务）改为**自己掌管扫描开关**：行进/寻迹一律下发"停扫"（锁止），
        # 只有进入 `StopKind.SHOOT` 停站（最佳射击点）才下发"持续扫描"（起转寻靶），
        # 离开停站（含 30s 超时放行、FINISH）立刻回到锁止。
        # 机械语义说明：本仓 bridge 没有"归中"指令，`scan_request=0` 就是"停扫/保持当前位"
        # —— 与 C++ 侧既有语义一致（`survey_gimbal_fire.md` §2.2），不新增机械动作。
        self.gimbal_lock_enabled = bool(g("aim.gimbal_lock_enabled", True))
        self.gimbal_lock_scan = int(g("aim.gimbal_lock_scan_request", 0))
        self.gimbal_seek_scan = int(g("aim.gimbal_seek_scan_request", 2))
        self.gimbal_seek_shoot_only = bool(g("aim.gimbal_seek_on_shoot_stop_only", True))
        self._lead_eps = float(g("aim.lead_eps_px", 0.5))
        #: ★ 许可最短保持时间：许可一旦成立，即使冷却已经开始计时，也再保持这么久。
        #:   为什么需要：`fire_ready` 是给云台（100Hz 硬实时）采的**电平**信号，
        #:   若许可只亮一拍（20ms）而云台那一拍正在处理别的分支，就会漏掉整次授权；
        #:   反过来电平常亮又会让"冷却"形同虚设（云台每拍都能看到 1）。
        #:   取 0.30s：既 > 云台一拍火控周期，又 < `shot_cooldown_ms`（默认 500ms）
        #:   —— 许可占空比 60%，冷却仍然真的在限制发数。
        self._hold_min_s = max(0.0, float(g("aim.hold_min_s", 0.30)))

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
            # ★ 速度基线窗口：0.40s 是"噪声 vs 滞后"的折中（见 RailEstimator 类文档）。
            #   现场若把视觉降到 15fps，窗口内样本数减半 → 噪声翻倍，需要同步放大这个值。
            vel_window_s=float(g("aim.est_vel_window_s", 0.40)),
            flip_min_speed_px_s=float(g("aim.est_flip_min_speed_px_s", 2.0)),
        )

        # ---- 运行时状态 ----
        self._permit = _Permit()
        self._last_frame_ts: float | None = None
        self._frame_dt = 1.0 / self._fps
        self._last_obs_ts: float = 0.0
        self._last_obs: OffsetObservation | None = None
        self._seen_prev = False
        #: ★ 最近一次"判丢"的时刻。**故意不在目标重现时清掉**：重新武装判据要的是
        #:   "目标离开过多久"，而丢靶检测本身有 `VISION_MAX_AGE_S`(0.5s) 的滞后 ——
        #:   只在重现那一拍算一次会把 0.2s 的检测差当成"只丢了 0.2s"，
        #:   于是"消失过一阵又回来"永远判不出来（5.7 组用例就是这个坑）。
        #:   它由 `_maybe_rearm()` 成功重新武装时清掉。
        self._lost_at: float | None = None
        self._last_class = 0
        self._cls_candidate = 0
        self._cls_run = 0
        self._locked_cls = 0
        self._fire_ready = False
        #: 许可保持窗口的截止时刻（见 `_hold_min_s`）
        self._hold_until: float = 0.0
        self._aim_status = AimStatus.IDLE
        self._ammo_prev: int | None = None
        self._shots_seen = 0
        self._hits_total = 0
        self._rescan_requested = False
        self._blocked_reason = REASON_NO_TARGET
        self._last_lead = 0.0
        self._last_tick_at: float | None = None
        self._last_cmd: dict[str, Any] = {}
        # ---- ★云台巡迹锁止的状态----
        self._gimbal_seek = False           # 当前是否允许转台搜索（= 在最佳射击点停站内）
        self._gimbal_cmd: int | None = None  # 上次下发的 scan_request（None = 还没下发过）
        self.gimbal_lock_ticks = 0          # 处于锁止的拍数（遥测/验收用）
        #: 进入锁止的**边沿**计数（每次 TRACK→锁止 都会 +1 并强制发一次 0 ⇒ C++ 回正）
        self.gimbal_home_cmds = 0
        self._gimbal_was_seeking = False     # 上一拍是否在"起转搜索"（用于识别锁止边沿）
        self._gimbal_last_home_at: float = 0.0
        self.gimbal_seek_ticks = 0          # 处于可起转搜索的拍数
        self.gimbal_switches = 0            # 实际下发开关切换的次数（应远小于拍数）

        # ---- ★ 构造即发布安全初值 ----
        # 为什么放在构造函数而不是 `setup()`：`setup()` 只在真线程 `start()` 时跑，
        # 而 Blackboard 是给**别的任务**读的 —— 别人（遥测/任务调度/测试夹具）在我们第一拍
        # 之前读 `P_AIM` 必须拿到"闭锁"而不是 None。旧写法会让 `bb.get(P_AIM).fire_gate`
        # 在第一拍前直接 AttributeError（4.1 组用例抓到的就是这个）。
        self._publish_initial()

    def _publish_initial(self) -> None:
        """构造时写一份"失败偏安全"的初值（fire_gate=False / seen=False / reason=no_target）。"""
        try:
            self.bb.set(P_TARGET, TargetState(), source=self.name)
            self.bb.set(P_AIM, AimState(status=int(self._aim_status), fire_gate=False,
                                        last_reason=REASON_NO_TARGET), source=self.name)
        except Exception:                       # noqa: BLE001  黑盒不可用不该拦住构造
            pass

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

        # ---- 3c. ★红面早退：锁定红持续 ≥ aim.red_early_leave_s ----
        self._update_red_confirmed(dt)

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

        # ---- 5b. ★云台巡迹锁止：行进中锁止，只有最佳射击点停站才起转 ----
        self._update_gimbal_gate(now)

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
                self._seen_prev = True
                self._on_target_seen(now)
            # ★ 重新武装判据**每拍都评估**（不只是"重现那一拍"）：
            #   条件是"目标离开过 ≥ rearm_after_lost_s"，而这个时长在重现之后仍在增长
            #   （`_lost_at` 有意保留到重新武装成功为止）。只判重现那一拍的话，
            #   0.5s 检测滞后会把 0.75s 的缺席算成 0.2s → 永远解不开锁存。
            self._maybe_rearm(now)
            return

        # ---- 不可见：立刻闭锁（旧版这里会冻结云台最长 6s，survey §10 第 7 条） ----
        if self._seen_prev:
            self._lost_at = now
            self._seen_prev = False
            self._rescan_requested = False
            self.bus.publish(EV_TARGET_LOST, source=self.name,
                             dx=int(getattr(self._last_obs, "dx", 0) or 0),
                             at=now)

    def _on_target_seen(self, now: float) -> None:
        """（重新）见到目标：刷新本靶射击窗口基准 + 发可见性事件。"""
        if self._permit.first_seen_at <= 0.0:
            self._permit.first_seen_at = now
            self._permit.baseline_shots = self._shots_seen
        self.bus.publish(EV_TARGET_FIRST_SEEN, source=self.name,
                         cls=int(self._locked_cls or self._last_class), at=now)

    def _maybe_rearm(self, now: float) -> None:
        """命中锁存后的重新武装（靶是"来回走的同一个"→ 默认不自动重新武装，见文件头）。

        条件全部满足才解除 `hit_latched`：
          ① `aim.hit_hold_s` 已过（命中后至少安静这么久）；
          ② 目标**离开过** `aim.rearm_after_lost_s` 以上（"消失过一阵又回来"
             = 新一块靶 / 同一块靶的下一轮；时长自 `_lost_at` 起算，含检测滞后）；
          ③ 目标当前**可见**（调用点已保证）。

        ⚠️ 现场"两块独立靶板交替出现"时，这条判据会自动把第二块当成新靶 —— 这是有意的：
          本靶打中后停火，第二块靶不该因为"第一块命中了"而永远打不了。

        ⚠️ 刻意**没有**"本靶许可数未到上限"这条（上一版有，是个自锁 bug）：
          `p.rearm()` 恰恰会把 `window` 清零 —— 用"window 已到上限"去否决一次会清零 window
          的重新武装，等于说"打满 3 发之后这个任务对任何新靶都不再开火"（实测表现：
          命中 + 长时间丢靶后 `hit_latched` 永远为真）。上限该由 `_evaluate` 的预算门管。
        """
        p = self._permit
        if not p.hit_latched:
            return
        if (now - p.hit_at) < self._hit_hold_s:
            return
        lost_for = (now - self._lost_at) if self._lost_at is not None else 0.0
        if lost_for < self._lost_arm_s:
            return
        p.rearm(now, self._shots_seen, now)
        self._lost_at = None                 # 本轮缺席结案，下轮重新计时
        self._log(f"命中锁存解除：目标离开过 {lost_for:.2f}s ≥ "
                  f"{self._lost_arm_s:.2f}s → 本靶许可窗口重新计数"
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
        """返回 `(fire_ready, reason)`。★ 任一硬门不满足 → 0（失败偏安全）。

        ★ 判据顺序 = **原因码的可解释性**（测试按原因码断言，遥测里人也要能一眼看懂）：

          安全 → 遥控 → 无目标 → 类别 → 命中锁存 → 预算/弹药 → **几何(策略/框)** →
          估计器(stale/warmup) → 冷却

        为什么"几何"要排在估计器**前面**：目标突然跳到画面边缘（`dx=400`）时，
        估计器会因为跳变门而重置（`warmup>0`）。若先判估计器，原因码就是 `estimator_warmup`，
        把"靶跑出开火框了"这个**真正的几何事实**掩盖掉；先判几何则原因码是 `out_of_box`，
        现场看日志能立刻分清"没瞄上"和"估计没收敛"。

        ⚠️ 顺序**不改变安全性**：两道门都是"不许可"，谁先谁后只影响原因码。

        ★ 冷却门有**迟滞**（`aim.hold_min_s`，见 `_hold_min_s` 的说明）：许可一旦成立，
        冷却期间也再保持 `hold_min_s`。所以这里只负责判"拍内是否合格"，
        迟滞由 `_set_fire_ready()` 统一施加（保持窗口内的原因码回写成 `ok`）。
        """
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
        # ---- 硬门 4：类别（见 `_update_class`：本拍类别必须等于**已锁定**的类别） ----
        cls_now = self._cls_of_locked()
        if cls_now == ShotClass.NONE or cls_now != self._locked_cls:
            return False, REASON_CLASS_UNSTABLE
        if self._fire_only_blue and self._locked_cls != ShotClass.BLUE:
            return False, REASON_CLASS_NOT_ALLOWED
        # ---- 硬门 5：命中锁存（已中即停；`aim.stop_on_hit=false` 时只记账不闭锁） ----
        if self._stop_on_hit and self._permit.hit_latched:
            return False, REASON_HIT_LATCHED
        # ---- 硬门 6：弹药 / 预算 ----
        if self._permit.window >= self._max_per_target:
            return False, REASON_BUDGET_PER_TARGET
        if self._shots_seen >= self._max_total:
            return False, REASON_BUDGET_TOTAL
        ammo = self._ammo_prev
        if ammo is not None and int(ammo) <= 0:
            return False, REASON_AMMO_EMPTY
        # ---- 硬门 7：几何（策略 + 开火框） ----
        est = self._estimator
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
            if abs(pred) >= abs(dx) and abs(est.v) > self._lead_eps:
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
        # ---- 硬门 8：速度估计可信（折返/重捕后要收敛时间） ----
        if est.stale or not est.init:
            return False, REASON_ESTIMATOR_STALE
        if not est.armed:
            return False, REASON_ESTIMATOR_WARMUP
        # ---- 硬门 9：冷却 ----
        if not self._permit.cool_ok(now, self._cooldown_s):
            return False, REASON_COOLDOWN
        return True, REASON_OK

    def _cls_of_locked(self) -> int:
        """本拍观测的类别（与 `_update_class()` 的判定保持一致）。"""
        return int(self._cls_candidate) if self._cls_candidate else ShotClass.NONE

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

        ★ 冷却 + 迟滞的关系（`aim.hold_min_s`）：许可电平在记完许可后再保持 `hold_min_s`
        （云台 100Hz 一定能采到），随后到冷却到点之前保持 0 —— 两者相加仍严格保证
        "两发许可之间 ≥ `shot_cooldown_ms`"，只是把这一拍的 1 铺成了一段可靠的高电平。
        """
        ready = bool(ready)
        # ---- ★ 许可迟滞：纯粹的"冷却未到"不立刻撤许可，再保持 `_hold_min_s` ----
        # 为什么：许可一旦成立，云台那一拍可能正在处理别的分支；只亮一拍的许可会被漏采。
        # 为什么只对"冷却"这一条迟滞：其余门（安全/丢失/命中/预算/几何/估计器）都是
        # **事实变了**，必须当拍闭锁；只有冷却是"我们自己刚刚计数"造成的，可以宽限。
        if not ready and reason == REASON_COOLDOWN and now < self._hold_until:
            ready = True
            reason = REASON_OK
        if not ready:
            self._blocked_reason = reason
            if self._fire_ready:
                self._fire_ready = False
                self._hold_until = 0.0
                self.bus.publish(EV_FIRE_BLOCKED, source=self.name, reason=reason,
                                 lead_px=self._last_lead, at=now)
                self._log(f"闭锁 fire_ready=0：{_REASON_TEXT.get(reason, reason)}")
            elif reason != REASON_OK:
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
            self._hold_until = now + self._hold_min_s
            if first_for_this_target:
                # 本靶的**第一发**：一个独立事件（EV_FIRE_ALLOWED），便于复盘时间线
                self.bus.publish(EV_FIRE_ALLOWED, source=self.name, lead_px=self._last_lead,
                                 permits=self._permit.window, at=now)
            self._log(f"★ 瞄准合格 → fire_ready=1（许可 {self._permit.window}"
                      f"/{self._max_per_target}，累计 {self._permit.total}，"
                      f"lead {self._last_lead:+.1f}px）")
        self._fire_ready = True
        self._blocked_reason = REASON_OK
        self._emit_ready(True)

    def _update_gimbal_gate(self, now: float) -> None:
        """★★云台巡迹锁止：**行进中不动，到最佳射击点才起转寻靶**。

        判据只看两样**已经存在**的事实（不新增状态机、不抢 Mission 的停站判定）：
          · `P_STOP_POINT`（Mission 写、本任务读）：`active` 且 `kind == StopKind.SHOOT`
            ⇒ "正在最佳射击点停站" ⇒ 允许转台搜索；
          · 其余全部情形（DRIVE 巡迹 / 到点未停稳 / SCAN 类停站 / 停站已 DONE / FINISH）
            ⇒ **锁止**（下发"停扫"，云台保持当前位不动）。

        三个"绝不破坏"：
          ① **停站判定仍归 Mission**（本方法只读 `P_STOP_POINT`，不写任何停站状态）；
          ② **30 s 超时放行**仍由 Mission 的 `shot_timeout_s/lock_timeout_s` 决定 —— 超时后
             `StopPointState` 被置为 DONE/离开停站 ⇒ 本方法自然回到锁止，不干预计时；
          ③ **发弹统计**（`_account`/`_shots_seen`/`_hits_total`/`permits`）完全不碰。

        下发策略：**只在取值变化时发**（`_gimbal_cmd` 去重）—— 20 Hz 每拍重发会把话题刷爆，
        而 C++ 侧对 `scan_request` 是"电平/保持"语义（发一次即生效）。
        性能：`aim.gimbal_lock_enabled=False` 时本方法**完全不动**（回到旧行为，便于对照）。
        """
        if not self.gimbal_lock_enabled:
            return
        stop = self.bb.get(P_STOP_POINT)
        phase = self.bb.get(P_MISSION_PHASE)
        seeking = False
        why = "巡迹/非射击停站"
        if isinstance(stop, StopPointState) and stop.active:
            kind = str(getattr(stop, "kind", "") or "")
            if self.gimbal_seek_shoot_only:
                seeking = (kind == StopKind.SHOOT)
            else:
                seeking = kind in (StopKind.SHOOT, StopKind.SCAN)
            if seeking:
                why = f"最佳射击点停站 {stop.block or '#' + str(stop.index)}（kind={kind}）"
        self._gimbal_seek = bool(seeking)
        if seeking:
            self.gimbal_seek_ticks += 1
        else:
            self.gimbal_lock_ticks += 1
        want = int(self.gimbal_seek_scan if seeking else self.gimbal_lock_scan)
        # ★★ **每次 TRACK→锁止的边沿必须再发一次 0**（车端 C++ 收到 0 → `AIM_IDLE`
        #   → `CaremaMontorControl()` 每帧主动归位到"扫描行程中点 + 基准俯仰"= 相对车头回正）。
        #   为什么不能被 `_gimbal_cmd` 值去重吞掉：TRACK→锁止 时 `want(0)` 与缓存里的锁止值相同
        #   （都是 0），按值去重就会漏发 ⇒ 云台停在 TRACK 结束时的目标方向（可能侧向大角度），
        #   正是用户明确禁止的"停在任意角度"。**只发边沿那一拍**（不是每拍）：
        #   每拍重发会把话题刷爆（与 1.3「20 拍只 1 条」的既有契约冲突），而 C++ IDLE 自己
        #   每帧都在归位，边沿发一次就足够。
        #   C++ 证据：`src/dzrknn_dataprocess/src/dzrknn_dataprocess.cpp:662-684`
        #   （IDLE 分支每帧下发 Position_1/Position_0：绝不能让云台冻结在最后一次姿态上）。
        lock_edge = self._gimbal_was_seeking and not seeking
        self._gimbal_was_seeking = bool(seeking)
        if lock_edge:
            self.gimbal_home_cmds += 1
            self._gimbal_last_home_at = now
        elif want == self._gimbal_cmd:
            return
        self._gimbal_cmd = want
        self.gimbal_switches += 1
        try:
            self.bridge.publish_scan_request(want)
        except Exception:                       # noqa: BLE001  话题缺失不该杀任务
            return
        self._log(f"云台{'起转搜索' if seeking else ('回正锁止' if lock_edge else '锁止')}"
                  f"：scan_request={want}"
                  f"（{why}；阶段={phase}；累计切换 {self.gimbal_switches} 次"
                  + (f"；★TRACK→锁止 边沿回正：C++ IDLE 每帧归位到车头正前"
                     f"（累计回正 {self.gimbal_home_cmds} 次）" if lock_edge else "") + "）")

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
            #: ⚠️ 这里是**丢了多久（秒）**，不是"丢靶时刻"这个绝对时间戳。
            #:   旧写法塞的是 `_lost_at`（单调钟的绝对秒数，实测 1001.68s 这种值），
            #:   下游拿去当"时长"用会得到毫无意义的巨大数；可见时恒为 0。
            lost_since=(float(now - self._lost_at)
                        if (self._lost_at is not None and not has_target) else 0.0),
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
            # 云台巡迹锁止状态（遥测/验收可读；`scan_request` 的当前下发值也在里面）
            "gimbal_seek": bool(self._gimbal_seek),
            "gimbal_cmd": self._gimbal_cmd,
            "gimbal_lock_ticks": int(self.gimbal_lock_ticks),
            "gimbal_seek_ticks": int(self.gimbal_seek_ticks),
            # 锁止边沿回正台账（每次 TRACK→锁止 强发一次 0；回正由 C++ IDLE 每帧执行）
            "gimbal_home_cmds": int(self.gimbal_home_cmds),
            "gimbal_last_home_at": float(self._gimbal_last_home_at),
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
            # **只有已到最佳射击点、云台允许起转搜索时**才重发扫描。
            #   巡迹途中丢靶重发 2 会把刚锁止的云台重新解锁 —— 正是用户要消灭的行为。
            if self._rescan_on_lost and self._gimbal_seek:
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
    #  ★ 红面早退
    # ------------------------------------------------------------------ #
    def _update_red_confirmed(self, dt: float) -> None:
        """锁定红**持续** ≥ `aim.red_early_leave_s` ⇒ 写 `P_AIM_RED`（Mission 据此早退）。

        只在**射击停站的 ACQUIRE / FIRE 段**累计：巡迹途中（无停站）看到红面不该有任何
        副作用，否则"路过时瞄到一眼红"就会把下一个停站的开火窗口提前掐掉。
        离开停站（`phase` 变成 SETTLE/DONE 或键消失）或类别不再是红 → 累计清零、标志撤下。
        """
        stop = self.bb.get(P_STOP_POINT)
        phase = str(getattr(stop, "phase", "") or "")
        kind = str(getattr(stop, "kind", "") or "")
        in_stop_fire = (kind == StopKind.SHOOT
                        and phase in (StopPhase.ACQUIRE, StopPhase.FIRE))
        red = bool(self._fire_only_blue and self._locked_cls == ShotClass.RED)

        if in_stop_fire and red:
            self._red_run_s += max(0.0, dt)
        else:
            self._red_run_s = 0.0

        confirmed = bool(in_stop_fire and red
                         and self._red_run_s >= self._red_early_leave_s - 1e-9)
        if confirmed and not self._red_confirmed:
            block = str(getattr(stop, "block", "") or "")
            self._log(f"红面确认：锁定红已持续 {self._red_run_s:.2f}s ≥ "
                      f"{self._red_early_leave_s:.1f}s（停站点 {block}）"
                      f" ⇒ 置红面标志，Mission 可提前离站")
        self._red_confirmed = confirmed
        self.bb.set(P_AIM_RED, {
            "confirmed": confirmed,
            "run_s": round(self._red_run_s, 4),
            "threshold_s": round(self._red_early_leave_s, 4),
            "cls": int(self._locked_cls),
            "block": str(getattr(stop, "block", "") or ""),
        }, source=self.name)

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
            # ⚠️ 措辞里**不许**出现 U+26A0/U+2705 这类非 GBK 字符：车端 `log=print`
            #    在 GBK 控制台上会抛 UnicodeEncodeError，把整拍 tick 打断在半路
            #    （日志没打出来，后半拍的发布也没执行）。本文件里这类符号只留在注释/文档串。
            self._log("[注意] 以下配置键不在 SCHEMA 里（本任务读的是内置默认值，需补进 "
                      "core/config.py 的 SCHEMA）: " + ", ".join(missing))
        unregistered = [k for k in missing if k not in self._OPTIONAL_KEYS]
        if unregistered:
            self._log("[注意] 上面的键里有未登记进 `AimTask._OPTIONAL_KEYS` 的（自检提示）: "
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
