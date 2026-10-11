# -*- coding: utf-8 -*-
"""AimTask 自测（**纯脚本，无 pytest**）。

    python src\\dzbrain\\tests\\test_aim.py

覆盖（逐条给数值证据；每条都注明它防的是 `survey_gimbal_fire.md` §10 的哪个旧缺口）：

  0. 纯函数：开火框按框宽线性插值（近大远小）+ α-β 增益解析值 + 配置键自检
  1. **速度估计精度**：恒速 30 px/s 序列，±15% 内、收敛拍数 < 20（旧缺口①：无速度估计）
  2. **提前量方向与量级**：`lead = v × lead_time`，符号跟随 v，`bridge.leads` 可见（旧缺口②）
  3. **折返**：+30 → −30 px/s，断言折返瞬间 `lead_px` 归零（不外推旧速度）+ 闭锁 ≥ N 拍（旧缺口⑤）
     3b. 坐标跳变（> 门限）：估计器重置、绝不把跳变当速度
  4. **开火合格判据逐条**：预测落点在框内 → 1；框外 / 冷却 / 弹药 0 / 遥控 / 类别抖动 → 0
  5. **命中即停**：`add_hits(1)` 后不再许可（旧缺口④）；`P_AIM.hits` 累加
  6. **弹药上限**：许可数达 `max_shots_per_target` 后不再许可
  7. **丢靶闭锁**：`clear_vision()` 下一拍即 0；超过阈值后 `P_TARGET.seen=False`、`lost_since>0`
  8. **安全门**：`SafetyState(ok=False)` → 0（含 P_SAFETY 缺失）
  9. **不写别人的钥匙**：全程 `bridge.fire_gate` 为空（★ 最重要的一条纪律）
  10. 真线程 50 Hz 节拍（0.3 s，拍数容差内）、`errors==0`

全过 exit 0；任一失败 exit 1。
"""

from __future__ import annotations

import pathlib
import sys
from contextlib import contextmanager

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

# ⚠️ Windows 控制台默认 GBK：编不出的字符降级成替换字符，保证测试结果不被编码问题掩盖
try:
    sys.stdout.reconfigure(errors="replace")  # type: ignore[union-attr]
except Exception:                              # pragma: no cover
    pass

from harness import check, check_close, close, finish, run_case, section   # noqa: E402

from dzbrain.core.blackboard import (P_AIM, P_SAFETY, P_STOP_POINT, P_TARGET,  # noqa: E402
                                     P_TELEMETRY, Blackboard)
from dzbrain.core.config import SCHEMA, Config                            # noqa: E402
from dzbrain.core.events import EV_TARGET_FIRST_SEEN, EV_TARGET_LOST, EventBus  # noqa: E402
from dzbrain.core.state import SafetyState, StopPointState                # noqa: E402
from dzbrain.io.contract import AimStatus, ShotClass                      # noqa: E402
from dzbrain.tasks.aim import (AIM_TELEMETRY_KEY, FirePolicy, RailEstimator,  # noqa: E402
                               REASON_AMMO_EMPTY, REASON_BUDGET_PER_TARGET, REASON_COOLDOWN,
                               REASON_HIT_LATCHED, REASON_MANUAL, REASON_NO_TARGET,
                               REASON_OUT_OF_BOX, REASON_SAFETY, AimTask, fire_box_px)

from fakes import FakeBridge, FakeClock                                   # noqa: E402

FPS = 30.0
FRAME_DT = 1.0 / FPS


@contextmanager
def _schema_without(*keys: str):
    """临时把若干键从 `SCHEMA` 里摘掉（用完还原）。

    为什么要这个：`AimTask._opt()` 的价值在于"配置键不在 SCHEMA 里时会被登记出来"，
    这个机制必须**始终**被测到 —— 不能靠"此刻恰好还没补齐"来间接测（补齐那天测试就失效）。
    `aim.py` 里 `from ..core.config import SCHEMA` 拿的是同一个 dict 对象，所以原地增删可见。
    """
    saved = {k: SCHEMA[k] for k in keys if k in SCHEMA}
    for k in saved:
        del SCHEMA[k]
    try:
        yield
    finally:
        SCHEMA.update(saved)


# --------------------------------------------------------------------------- #
#  脚手架
# --------------------------------------------------------------------------- #
def cfg_of(*, st_ticks: int = 1, policy: str = FirePolicy.MOVING_LEAD, **aim) -> Config:
    """配置：默认**静止目标**（v=0 时 moving_lead 必然许可），所有门都在最宽松状态。

    `st_ticks`：静止目标每拍推进的秒数（冷却测试用它把冷却熬过去）。

    ★ `max_shots_per_target` 默认给 **10**（不是 SCHEMA 默认的 3）：许可按"冷却窗口"
      累计（每 `shot_cooldown_ms` 记一次，与是否真的开火无关），所以任何持续 1.5s 以上的
      用例都会把 3 发预算用完 —— 那会把**被测的门**（安全/命中锁存/类别/丢靶恢复）
      掩盖成 `budget_per_target`。这与本文件 `Rig.shots_enabled` 的注释是同一条理由：
      "只测瞄准门控的用例不要被上限打扰"。专测单靶上限的用例（第 6 组）**显式**写 3。
    """
    data = {
        "aim": dict({
            "lead_time_s": 0.35,
            "fire_policy": policy,
            "max_shots_total": 10,
            "max_shots_per_target": 10,
            "shot_cooldown_ms": 500,
            "fire_box_far_px": 14.0,
            "fire_box_near_px": 26.0,
            "rail_span_m": 0.70,
            "target_speed_mps": 0.04,
            "lost_frames_to_rescan": 60,
        }, **aim),
        "logging": {"rate_limit_s": 0.0},        # 测试里不压日志，便于排查
    }
    return Config.from_dict(data)


class Rig:
    """一台"车"：FakeBridge + Blackboard + AimTask（手动单拍，不启线程）。"""

    def __init__(self, *, cfg: Config | None = None, safety: SafetyState | None = None,
                 st_ticks: float = 1.0) -> None:
        self.clock = FakeClock()
        self.bridge = FakeBridge(self.clock)
        self.bb = Blackboard()
        self.bus = EventBus()
        self.cfg = cfg or cfg_of()
        self.logs: list[str] = []
        self.st_ticks = st_ticks
        #: ★ True = 模拟"云台真的开火了"：每次 `fire_ready` 由 0 跳 1 就给 `shoot_count` +1。
        #: 这样 `max_shots_total`（`shoot_count` ≥ 上限）才会生效 —— 默认 False 是为了让
        #: 只测瞄准门控的用例不被全场上限打扰（真机上云台与裁判都会累加这个计数）。
        self.shots_enabled = False
        self.task = AimTask(self.bridge, self.bb, self.bus, self.cfg,
                            log=self.logs.append, clock=self.clock.now)
        self._sub = self.bus.subscribe("test-aim")           # 全类型订阅（复盘事件时间线）
        self._events: list = []
        self.bb.set(P_SAFETY, safety if safety is not None else SafetyState(ok=True), "test")
        self.bridge.set_aim_status(AimStatus.TRACK)

    def drain(self) -> list:
        """把事件从队列搬进本地时间线（★ 只搬一次，便于多次断言）。"""
        self._events.extend(self._sub.poll(256))
        return self._events

    def events(self, ev_type: str | None = None) -> list:
        self.drain()
        return [e for e in self._events if ev_type is None or e.type == ev_type]

    def event_types(self) -> list[str]:
        return [e.type for e in self.events()]

    def count(self, ev_type: str) -> int:
        return len(self.events(ev_type))

    # ---------------- 观测推进 ----------------
    def frame(self, dx: float = 0, dy: float = 0, *, cls: int = ShotClass.BLUE, box_w: int = 45,
              box_h: int = 55, dt: float = FRAME_DT, tick: bool = True) -> None:
        """推进一帧视觉（时钟同步前进，`ts` = 新时刻）→ 再单拍一次。

        ⚠️ dx 会被 `FakeBridge.set_vision` 取整（真机 det_node 也只发整数 px）。
           运动学测试请用 `ramp()`：它按**精确积分**给整数像素序列，量化误差才可控。
        """
        self.clock.advance(dt)
        self.bridge.set_vision(dx=int(round(dx)), dy=int(round(dy)), cls=cls,
                               box_w=box_w, box_h=box_h)
        if tick:
            n_before = len(self.bridge.fire_ready)
            self.task.tick()
            if self.shots_enabled:
                seq = self.bridge.fire_ready[n_before:]
                if seq and seq[0] and not (n_before and self.bridge.fire_ready[n_before - 1]):
                    self.bridge.add_shots(1)      # 0→1 跳变 = 云台真的打了一发

    def ramp(self, v_px_s: float, frames: int, *, dt: float = FRAME_DT, dx0: float = 0.0,
             cls: int = ShotClass.BLUE, box_w: int = 45, loop: bool = False,
             span: float | None = None) -> float:
        """喂 `frames` 帧**恒速**序列；返回末帧的精确 dx。

        `dx = dx0 + v·(k·dt)` 用**精确积分**算（不是逐帧累加），因此当 `v·dt` 是整数时
        像素序列**完全无量化误差** —— 这样才能用严格容差验证估计器。
        `loop=True` 时在 ±span 之间来回（模拟靶在轨道上往复）。
        """
        for k in range(1, frames + 1):
            dx = dx0 + v_px_s * k * dt
            if loop and span:
                width = 2.0 * span
                pos = (dx - dx0) % width
                dx = dx0 + (pos if pos <= span else width - pos)
            self.frame(dx, dt=dt, cls=cls, box_w=box_w)
        return dx0 + v_px_s * frames * dt

    def no_vision(self, seconds: float = FRAME_DT) -> None:
        """不更新视觉、只推进时钟并单拍（等观测过期 → 判定丢靶）。"""
        end = self.clock.now() + seconds
        while self.clock.now() < end - 1e-12:
            self.clock.advance(min(0.05, max(1e-3, end - self.clock.now())))
            self.task.tick()

    def idle(self, seconds: float) -> None:
        """保持"目标静止在最后一帧位置"推进 `seconds` 秒（冷却/超时测试用）。

        ★ 每拍都重发**同一帧**观测（dx 不变 → 估计器把 dt 判为同帧、不重复更新，但
          `cls_run` 会正常累积）—— 这正是真机"靶停在那里"的形态；
          要测"没有观测"请用 `no_vision()`，要测"丢靶"也是一样。
        """
        obs = self.bridge.vision_raw()
        dx = int(getattr(obs, "dx", 0) or 0)
        dy = int(getattr(obs, "dy", 0) or 0)
        cls = int(getattr(obs, "cls", ShotClass.BLUE) or ShotClass.BLUE)
        box_w = int(getattr(obs, "box_w", 45) or 45)
        end = self.clock.now() + seconds
        while self.clock.now() < end - 1e-12:
            self.frame(dx, dy, cls=cls, box_w=box_w,
                       dt=min(0.05, max(1e-3, end - self.clock.now())))

    def ticks(self, n: int, dt: float = 0.0) -> None:
        for _ in range(n):
            self.task.tick()
            if dt:
                self.clock.advance(dt)

    # ---------------- 读 ----------------
    def ready(self) -> bool:
        return bool(self.bb.get(P_AIM).fire_gate)

    def reason(self) -> str:
        return str(self.bb.get(P_AIM).last_reason)

    def tel(self) -> dict:
        return self.task.telemetry()

    def ready_seq(self) -> list[bool]:
        return list(self.bridge.fire_ready)


def fresh_ready_frames(seq: list[bool]) -> int:
    """末尾连续 True 的拍数。"""
    n = 0
    for v in reversed(seq):
        if v:
            n += 1
        else:
            break
    return n


# =========================================================================== #
#  0. 纯函数与配置自检
# =========================================================================== #
def test_00_pure() -> None:
    section("0. 纯函数：开火框插值 / α-β 增益 / 关键 SCHEMA 键")

    # box_w=30（远端点）→ 14px；box_w=120（近端点）→ 26px；中间线性
    box = lambda w: fire_box_px(w, near_px=26.0, far_px=14.0, box_w_far=30.0, box_w_near=120.0)
    check_close("0.1 框宽 30px（远端点）→ 开火框 14px", box(30), 14.0, 1e-12)
    check_close("0.2 框宽 120px（近端点）→ 开火框 26px", box(120), 26.0, 1e-12)
    check_close("0.3 框宽 75px（中点）→ 开火框 20px", box(75), 20.0, 1e-12)
    check_close("0.4 框宽 45px → 16.333px（线性插值）", box(45), 14.0 + (45 - 30) / 90.0 * 12.0,
                1e-12)
    check_close("0.5 框宽超范围被夹住（不外推）", box(999), 26.0, 1e-12)
    check_close("0.6 框宽 0（观测没给框宽）→ 取最保守的远端框", box(0), 14.0, 1e-12)
    check_close("0.7 标定退化（near==far）→ 远端框",
                fire_box_px(80, near_px=26.0, far_px=14.0, box_w_far=30.0, box_w_near=30.0),
                14.0, 1e-12)

    a, b = RailEstimator.gains(3.5)
    check_close("0.8 α-β 增益解析值 α=2(2ξ-1)/(ξ(ξ+1))", a, 2 * (2 * 3.5 - 1) / (3.5 * 4.5), 1e-12)
    check_close("0.9 α-β 增益解析值 β=6/(ξ(ξ+1))", b, 6 / (3.5 * 4.5), 1e-12)
    check("0.10 ξ=3.5 → α≈0.762、β≈0.381（实测值打印）",
          close(a, 0.761904761904762, 1e-12) and close(b, 0.380952380952381, 1e-12),
          f"alpha={a:.6f} beta={b:.6f}")

    # P2 已经进 SCHEMA 的键（缺了说明有人删了，得知道）
    for key in ("aim.lead_time_s", "aim.fire_policy", "aim.rail_span_m", "aim.max_shots_total",
                "aim.max_shots_per_target", "aim.shot_cooldown_ms", "aim.fire_box_far_px",
                "aim.fire_box_near_px", "aim.lost_frames_to_rescan"):
        check(f"0.11 SCHEMA 已有 {key}", key in SCHEMA)

    cfg = Config.from_dict({"aim": {"lead_time_s": 0.35}})
    task = AimTask(FakeBridge(FakeClock()), Blackboard(), EventBus(), cfg, log=lambda *_: None,
                   clock=FakeClock().now)
    miss = task.missing_schema_keys
    # ★ 下面四条测的是**登记机制**，不是"此刻恰好缺哪些键"：
    #   SCHEMA 随时可能被补齐（本文件写作时 `aim.*` 已被补进 SCHEMA），
    #   如果断言写死"缺 ≥20 个"，补齐那一刻测试就会变成假红 —— 而机制本身没人再测。
    check("0.12 实现读的可选键**全部**已在 SCHEMA 里（缺一即为静默回退风险）",
          all(k in SCHEMA for k in AimTask._OPTIONAL_KEYS),
          f"缺: {[k for k in AimTask._OPTIONAL_KEYS if k not in SCHEMA]}")
    check("0.13 已在 SCHEMA 里的键**不**被判为缺失",
          "aim.lost_frames_to_rescan" not in miss and "aim.fire_box_far_px" not in miss,
          f"当前缺 {len(miss)} 个")
    check("0.14 _OPTIONAL_KEYS 无重复（自检）",
          len(AimTask._OPTIONAL_KEYS) == len(set(AimTask._OPTIONAL_KEYS)),
          f"{len(AimTask._OPTIONAL_KEYS)} 个 → {len(set(AimTask._OPTIONAL_KEYS))} 个不重复")
    # ★ 机制本身必须始终有效：临时从 SCHEMA 摘掉两个键，`_opt()` 必须把它们登记出来
    #   —— 这就是"拒绝静默回退"的落地（预检门禁靠它列出待补键）。
    with _schema_without("aim.px_per_m_rail", "aim.box_w_far_px"):
        cfg2 = Config.from_dict({"aim": {"lead_time_s": 0.35}})
        task2 = AimTask(FakeBridge(FakeClock()), Blackboard(), EventBus(), cfg2,
                        log=lambda *_: None, clock=FakeClock().now)
        miss2 = task2.missing_schema_keys
    check("0.15 从 SCHEMA 摘掉键后立刻被登记（含 px/m 与框宽标定；不是静默回退）",
          "aim.px_per_m_rail" in miss2 and "aim.box_w_far_px" in miss2 and len(miss2) >= 2,
          f"{len(miss2)} 个：{miss2[:6]}…")


# =========================================================================== #
#  1. 速度估计精度
# =========================================================================== #
def test_01_estimator() -> None:
    section("1. ★ 速度估计精度（旧缺口①：只有单帧差分、无时间戳）")
    V = 30.0
    DT_EXACT = 1.0 / 15.0            # 30px/s × 1/15s = 2px/帧 → 像素序列**零量化误差**

    # 1.1 收敛拍数：逐帧找首次进入 ±15%
    rig = Rig()
    first_ok = None
    for k in range(1, 21):
        rig.frame(k * V * DT_EXACT, dt=DT_EXACT)
        if abs(rig.task.estimator.v - V) <= 0.15 * V and first_ok is None:
            first_ok = k
    v20 = rig.task.estimator.v
    check("1.1 收敛拍数 < 20 拍", first_ok is not None and first_ok < 20,
          f"首次进入 ±15% 在第 {first_ok} 拍")
    check("1.2 第 20 拍估计误差在 ±15% 内", abs(v20 - V) <= 0.15 * V,
          f"v={v20:.4f} px/s，误差 {(v20 - V) / V * 100:+.4f}%")
    check("1.3 第 20 拍误差 < 1e-4（恒速 + 无量化噪声 → α-β 无稳态误差）", abs(v20 - V) < 1e-4, f"v={v20!r}")

    # 1.4 dt 真的来自观测时间戳：帧间隔改成 1/20s（v·dt=1.5px 有量化），估计仍应 ≈30
    rig2 = Rig()
    dx = 0.0
    for _ in range(40):
        rig2.frame(dx, dt=1.0 / 20.0)          # 20fps
        dx += V * (1.0 / 20.0)
    v20fps = rig2.task.estimator.v
    check("1.4 帧率变成 20fps（dt 来自 ts）估计仍 ≈30px/s（±15%）",
          abs(v20fps - V) <= 0.15 * V,
          f"v={v20fps:.4f} px/s（若把 dt 写死 1/30s 会得到 45）")

    # 1.5 目标静止：速度估计应趋近 0（不是恒定噪声）
    rig3 = Rig()
    for _ in range(30):
        rig3.frame(0)
    check("1.5 静止目标 → |v| ≈ 0", abs(rig3.task.estimator.v) < 1e-9,
          f"v={rig3.task.estimator.v!r}")

    # 1.6 采样计数：50Hz 循环读 30Hz 视觉，**同一帧不重复喂**
    rig4 = Rig()
    for k in range(1, 31):
        rig4.frame(k * V * FRAME_DT)          # 30 帧
        rig4.task.tick()                      # 多一拍（时钟不动，仍是同一帧）
    check("1.6 同帧重读不重复计数（30 帧 + 30 次空拍 → samples 仍 30）",
          rig4.task.estimator.samples == 30,
          f"samples={rig4.task.estimator.samples}")

    # 1.7 量化噪声下仍能收敛（真机 det_node 只发整数 px）
    rig5 = Rig()
    dx = 0.0
    errs = []
    for _ in range(60):
        rig5.frame(dx)                         # 30fps、30px/s = 1px/帧 → 整数，无量化
        dx += V * FRAME_DT
        errs.append(abs(rig5.task.estimator.v - V))
    check("1.7 30fps 下（1px/帧）稳态误差 < 1e-4", max(errs[20:]) < 1e-4, f"稳态最大误差 {max(errs[20:]):.2e} px/s")

    # 1.8 速度兜底夹紧不会误伤正常的快靶（300px/s 以内应能正常跟踪）
    rig6 = Rig()
    rig6.ramp(200.0, 40, dt=1.0 / 15.0)
    v200 = rig6.task.estimator.v
    check("1.8 200px/s 的快靶能正常跟踪（兜底夹紧没误伤）",
          abs(v200 - 200.0) <= 0.15 * 200.0 and rig6.task.estimator.resets == 0,
          f"v={v200:.3f} px/s resets={rig6.task.estimator.resets}")

    # 1.9 荒谬速度（远超人能跑的速度）→ 夹紧 + 重置（防误检/噪声）
    rig7 = Rig()
    rig7.ramp(60.0, 20, dt=1.0 / 15.0)
    before = rig7.task.estimator.resets
    rig7.frame(2000)                            # 单帧跳 2000px（60fps → 60000px/s）
    check("1.9 荒谬速度/跳变 → resets 增加（宁可重置也不外推）",
          rig7.task.estimator.resets > before,
          f"resets {before} → {rig7.task.estimator.resets}（单帧跳 2000px，"
          f"reason={rig7.task.estimator.last_reason}）")


# =========================================================================== #
#  2. 提前量
# =========================================================================== #
def test_02_lead() -> None:
    section("2. ★ 提前量方向与量级（旧缺口②：预判只有 0.5 帧≈16ms）")
    V = 30.0
    DT_EXACT = 1.0 / 15.0                      # 2px/帧 → 零量化
    rig = Rig()
    rig.ramp(V, 20, dt=DT_EXACT)
    lead = rig.task.telemetry()["lead_px"]
    check_close("2.1 lead = v × lead_time = 30 × 0.35 = 10.5 px", lead, 10.5, 1e-3)
    check("2.2 bridge.leads 里能看到（发给云台补丁）",
          len(rig.bridge.leads) >= 20 and close(rig.bridge.leads[-1], 10.5, 1e-3),
          f"末值={rig.bridge.leads[-1]!r}，共 {len(rig.bridge.leads)} 条")
    check("2.3 符号与 v 一致（正速度 → 正提前量）", lead > 0, f"lead={lead:+.4f}")

    rig_neg = Rig()
    rig_neg.ramp(-V, 20, dt=DT_EXACT)
    lead_neg = rig_neg.task.telemetry()["lead_px"]
    check("2.4 反向运动 → 提前量为负、量级相同",
          close(lead_neg, -10.5, 1e-3), f"lead={lead_neg:+.4f}")

    # 2.5 提前量限幅：轨道半宽 = max_lead_factor(0.45) × 0.70m × 140px/m = 44.1px
    #     v=200px/s × 0.35s = 70px > 44.1px → 必须被夹住
    rig_fast = Rig()
    rig_fast.ramp(200.0, 25, dt=1.0 / 15.0)
    lim = rig_fast.task.telemetry()["lead_px"]
    check("2.5a 快靶的速度估计本身没有被夹坏（±15%）",
          abs(rig_fast.task.estimator.v - 200.0) <= 0.15 * 200.0,
          f"v={rig_fast.task.estimator.v:.3f} px/s")
    check_close("2.5b 提前量被硬夹到 ±轨道半宽 44.1px（70 → 44.1）", lim, 44.1, 1e-9)
    check("2.6 限幅后仍不大于半宽（防把目标外推进画面）", abs(lim) <= 44.1 + 1e-9)

    # 2.7 目标丢失 → 提前量归零（不外推旧速度）
    rig2 = Rig()
    rig2.ramp(V, 20, dt=DT_EXACT)
    rig2.no_vision(0.6)
    check_close("2.7 丢靶后 lead 归零（绝不外推）", rig2.task.telemetry()["lead_px"], 0.0, 1e-12)


# =========================================================================== #
#  3. 折返与跳变
# =========================================================================== #
def test_03_turnaround() -> None:
    section("3. ★ 折返：不外推旧速度 + 闭锁收敛（移动靶最容易打飞的地方）")
    V = 30.0
    DT = 1.0 / 15.0                        # 2px/帧 → 零量化，折返点可精确复现

    def mk() -> Rig:
        r = Rig(cfg=cfg_of(reset_confirm_frames=5))
        r.ramp(V, 20, dt=DT, dx0=-15.0)     # dx: -13 → +25（向 0 靠近再越过）
        return r

    rig = mk()
    # ⚠️ 这里**不能**断言 `ready() is True`：ramp 结束时 dx=+25，加上提前量
    #    （+30px/s × 0.35s = +10.5px）预测落点是 **35.5px**，而 box_w=45 → 开火框
    #    `fire_box_px(45)=16.0px`（0.1–0.7 与 4.6 都钉死了这个映射）。
    #    "预测落点在框内"是 4.6 的核心判据，在 35.5px 处许可开火就是**授权打飞**。
    #    所以本用例只断言它真正要测的东西：估计已收敛、且拦住开火的是**几何**而不是
    #    "估计没收敛"（后者的原因码是 estimator_warmup/stale）。
    check("3.1 折返前估计已收敛（v=+30；唯一拦住开火的是'预测落点出框'）",
          close(rig.task.estimator.v, V, 1e-9) and rig.task.estimator.armed
          and rig.reason() == REASON_OUT_OF_BOX,
          f"v={rig.task.estimator.v:.6f} armed={rig.task.estimator.armed} "
          f"ready={rig.ready()} reason={rig.reason()}")

    # ---- 折返瞬间：抓那一拍 ----
    # ⚠️ 位置步长是 2px，**时钟节拍必须同样是 1/15s**（=30px/s）。原写法在这里改用
    #    默认的 1/30s，等于把 2px 喂成 60px/s，却断言"估计 ≈30px/s" —— 数据与断言矛盾。
    dx = -15.0 + V * 20 * DT               # = +25 附近
    rig.frame(dx, dt=DT)                   # 折返前最后一帧（同向、零位移）
    old_v = rig.task.estimator.v
    resets_before = rig.task.estimator.resets
    rig.frame(dx - 2.0, dt=DT)             # ← 折返瞬间：dx 比上一帧小
    check("3.2 折返瞬间估计器被重置（resets +1）",
          rig.task.estimator.resets == resets_before + 1,
          f"resets {resets_before} → {rig.task.estimator.resets}，"
          f"reason={rig.task.estimator.last_reason}")
    check(f"3.3 折返瞬间 lead_px 归零（旧速度 +{old_v:.0f}px/s 没有被外推出去）",
          close(rig.task.telemetry()["lead_px"], 0.0, 1e-12),
          f"lead={rig.task.telemetry()['lead_px']!r}（若外推旧速度会是 +10.5）")
    check("3.4 折返瞬间 fire_ready=0", rig.ready() is False, f"reason={rig.reason()}")

    # ---- 闭锁拍数：从折返起数到重新许可 ----
    blocked = 1 if not rig.ready() else 0
    first_reopen = 0
    dx_now = dx - 2.0
    for i in range(1, 15):
        dx_now -= V * DT
        rig.frame(dx_now, dt=DT)
        if rig.ready():
            first_reopen = i
            break
        blocked += 1
    check("3.5 折返后闭锁拍数 ≥ reset_confirm_frames(5)（给收敛时间）",
          first_reopen >= 5, f"闭锁 {blocked} 拍后在第 {first_reopen} 拍重新许可")
    # ⚠️ 原断言 `v < 0 and abs(v - V) <= 0.15*V` 自相矛盾：v<0 时 |v−30| 必然 >30，
    #    永远不可能落在 ±15%（=±4.5px/s）内。要测的是"方向已翻、**量级**仍是 30"，
    #    所以比对的是绝对值。
    check("3.6 重新许可时速度估计已翻到负号（不是靠噪声蒙的）",
          rig.task.estimator.v < 0 and abs(abs(rig.task.estimator.v) - V) <= 0.15 * V,
          f"v={rig.task.estimator.v:+.4f} px/s")
    check("3.7 反向收敛后 lead 也是负的（方向正确）",
          rig.task.telemetry()["lead_px"] < 0,
          f"lead={rig.task.telemetry()['lead_px']:+.3f}")
    # ⚠️ 原断言 `fire_ready[-blocked:] == [False]*blocked` 对**任何**实现都不可能成立：
    #    `blocked` 数的是闭锁拍数，而循环是在"重新许可那一拍" break 的 ——
    #    末尾 blocked 条里必然含那一条 True。改成取**闭锁窗口**（排除重新许可那一拍），
    #    要测的东西不变：闭锁期间每一拍都真的往话题上写了 0（占位保活）。
    tail = rig.bridge.fire_ready[-blocked - 1:-1]
    check("3.8 闭锁期间每拍都真的发了 fire_ready=0（占位保活）",
          len(tail) == blocked and tail == [False] * blocked,
          f"闭锁窗口 {len(tail)} 拍取值集合 = {set(tail)}")
    # ★ 反事实：如果继续用折返前的 +30px/s 外推，预测落点是错的
    check("3.9 反事实对比：外推旧速度会预测 +10.5px 的提前量（方向全错）",
          close(old_v * 0.35, 10.5, 1e-6) and rig.task.telemetry()["lead_px"] < 0,
          f"旧速度外推 lead={old_v * 0.35:+.3f}px，实际 lead={rig.task.telemetry()['lead_px']:+.3f}px")

    # ---- 3b. 坐标跳变（> 门限）：不得当成速度 ----
    section("3b. 坐标跳变（> 3σ 门限）→ 重置而不是当成速度")
    rig3 = Rig()
    rig3.ramp(V, 20, dt=DT)
    v_ok = rig3.task.estimator.v
    jumps_before = rig3.task.estimator.jump_rejections
    rig3.frame(600)                            # 单帧跳到 600px（遮挡后重现/误检）
    v_after = rig3.task.estimator.v
    check("3b.1 跳变被判为跳变（jump_rejections +1）",
          rig3.task.estimator.jump_rejections > jumps_before,
          f"{jumps_before} → {rig3.task.estimator.jump_rejections}，"
          f"reason={rig3.task.estimator.last_reason}")
    check("3b.2 跳变后速度估计被清零（不是 600/0.033=18000px/s 这种荒谬值）",
          close(v_after, 0.0, 1e-12),
          f"跳变前 v={v_ok:.1f} → 跳变后 v={v_after!r}")
    check("3b.3 跳变后 lead 归零", close(rig3.task.telemetry()["lead_px"], 0.0, 1e-12))
    check("3b.4 跳变后立刻闭锁", rig3.ready() is False, f"reason={rig3.reason()}")

    # ---- 3c. 观测断流（> dt_max）→ 速度不可外推 ----
    section("3c. 观测断流 > dt_max_s → 速度清零")
    rig4 = Rig()
    rig4.ramp(V, 20, dt=DT)
    rig4.frame(rig4.task.telemetry()["dx"], dt=0.9)     # 0.9s 没有新帧
    check("3c.1 断流帧被识别为 gap（速度清零）", close(rig4.task.estimator.v, 0.0, 1e-12),
          f"v={rig4.task.estimator.v!r} reason={rig4.task.estimator.last_reason}")
    check("3c.2 断流后闭锁", rig4.ready() is False, f"reason={rig4.reason()}")


# =========================================================================== #
#  4. 开火合格判据逐条
# =========================================================================== #
def _first_permit(rig: Rig, *, dx: float = 0, cls: int = ShotClass.BLUE, box_w: int = 45,
                  max_frames: int = 60) -> bool:
    """喂观测直到**第一发许可**落下（不做多余的额外帧），返回是否拿到。"""
    for _ in range(max_frames):
        rig.frame(dx, cls=cls, box_w=box_w)
        if rig.ready():
            return True
    return rig.ready()


def _compile(rig: Rig, frames: int = 6, dx: float = 0, cls: int = ShotClass.BLUE,
             box_w: int = 45, *, need_ready: bool = True, tries: int = 60) -> bool:
    """喂观测直到类别确认完成（+ `need_ready` 时继续喂到第一发许可落下）。

    ★ 必须**持续喂帧**：只推时钟不喂观测会把类别确认计数清零（拦停原因变成 `cls_unstable`），
      被测的那道门就被掩盖了。但也不能无脑喂满 —— 冷却到点会不断累加许可，
      所以这里一拿到许可就停（要测冷却/预算的用例请自己控制喂帧节奏）。
    """
    for _ in range(max(1, frames)):
        rig.frame(dx, cls=cls, box_w=box_w)
    if not need_ready:
        return rig.ready()
    return _first_permit(rig, dx=dx, cls=cls, box_w=box_w, max_frames=tries)


def test_04_criteria() -> None:
    section("4. ★ 开火合格判据逐条（旧缺口③：要求画面静止 → 移动靶打不出去）")
    rig = Rig()
    check("4.1 无观测 → 0", rig.ready() is False and rig.reason() == REASON_NO_TARGET,
          f"reason={rig.reason()}")
    _compile(rig, 5)
    check("4.2 目标在框内、静止 → 1（moving_lead 判据）", rig.ready() is True,
          f"reason={rig.reason()} dx={rig.tel()['dx']} box={rig.tel()['box_px']:.1f}")

    # 4.3 dx 推到框外 → 0（先推进冷却，否则拦停原因会是 cooldown，掩盖被测门）
    rig.idle(0.6)
    rig.frame(400)
    check("4.3 dx 远在框外 → 0", rig.ready() is False and rig.reason() == REASON_OUT_OF_BOX,
          f"reason={rig.reason()} dx={rig.tel()['dx']} box={rig.tel()['box_px']:.1f}px")
    _compile(rig, 3)
    check("4.4 回到框内 → 恢复 1", rig.ready() is True, f"reason={rig.reason()}")

    # 4.5 提前量把落点推出框外（移动靶的典型场景）
    rig2 = Rig(cfg=cfg_of(box_w_far_px=30.0, box_w_near_px=30.0))     # 固定框 14px（省得插值）
    _compile(rig2, 8)
    check("4.5a 静止在 0 → 许可", rig2.ready() is True, f"reason={rig2.reason()}")
    for k in range(1, 16):                                        # 开始以 30px/s 移动
        rig2.frame(30.0 * k * FRAME_DT)
    pred = rig2.tel()["pred_px"]
    check("4.5b 预测落点被推出 14px 框 → 0（提前量真的进了判据）",
          rig2.ready() is False and abs(pred) > 14.0,
          f"dx={rig2.tel()['dx']} lead={rig2.tel()['lead_px']:+.2f} pred={pred:+.2f} box=14.0")

    # 4.6 框宽影响：同样 dx，近处（框宽 120 → 26px）许可、远处（框宽 30 → 14px）不许可
    far_ok = near_ok = None
    for box_w, tag in ((30, "far"), (120, "near")):
        r = Rig()
        for _ in range(8):
            r.frame(16, box_w=box_w)
        if tag == "far":
            far_ok = r.ready()
        else:
            near_ok = r.ready()
    check("4.6 框宽 → 距离 → 框大小（近大远小）真的生效",
          far_ok is False and near_ok is True,
          f"框宽30px(框14px) ready={far_ok} / 框宽120px(框26px) ready={near_ok} @ dx=16")

    # 4.7 冷却
    rig3 = Rig()
    _compile(rig3, 6)
    check("4.7a 首次许可成立", rig3.ready() is True and rig3.task.permits == 1,
          f"permits={rig3.task.permits}")
    rig3.idle(0.4)
    check("4.7b 冷却未到（0.4s < 0.5s）→ 0", rig3.ready() is False
          and rig3.reason() == REASON_COOLDOWN,
          f"reason={rig3.reason()} cool_left={rig3.tel()['cool_left_s']:.2f}s")
    check("4.7c 冷却期内许可数没有增加（一拍最多记一次许可）", rig3.task.permits == 1,
          f"permits={rig3.task.permits}")
    rig3.idle(0.2)
    check("4.7d 冷却到点 → 恢复 1 且许可 +1", rig3.ready() is True and rig3.task.permits == 2,
          f"reason={rig3.reason()} permits={rig3.task.permits}")

    # 4.8 弹药 = 0
    rig4 = Rig()
    rig4.bridge.set_ammo(0)
    _compile(rig4, 8, need_ready=False)
    check("4.8 ammo=0 → 0", rig4.ready() is False and rig4.reason() == REASON_AMMO_EMPTY,
          f"reason={rig4.reason()} ammo={rig4.bb.get(P_AIM).ammo}")

    # 4.9 遥控接管云台
    rig5 = Rig()
    _compile(rig5, 6)
    ok_before = rig5.ready()
    rig5.bridge._manual_gimbal = True
    rig5.task.tick()
    check("4.9 manual_gimbal=True → 0", ok_before is True and rig5.ready() is False
          and rig5.reason() == REASON_MANUAL, f"reason={rig5.reason()}")

    # 4.10 类别抖动：1 帧变类 → 当拍闭锁，但**不**改锁；连续 5 帧才改锁
    rig6 = Rig()
    _compile(rig6, 8)
    check("4.10a 类别已锁定 BLUE", rig6.tel()["cls"] == ShotClass.BLUE
          and rig6.tel()["cls_run"] >= 5, f"cls={rig6.tel()['cls']} run={rig6.tel()['cls_run']}")
    rig6.idle(0.6)                                         # 先把冷却熬过（否则拦停原因是 cooldown）
    rig6.frame(0)
    check("4.10b 冷却已过且许可成立（对照）", rig6.ready() is True, f"reason={rig6.reason()}")
    rig6.frame(0, cls=ShotClass.RED)                       # 单帧抖动
    check("4.10c 单帧变类：当拍闭锁", rig6.ready() is False, f"reason={rig6.reason()}")
    check("4.10d 单帧变类**不**改锁（locked 仍是 BLUE、确认计数归零）",
          rig6.tel()["cls"] == ShotClass.BLUE and rig6.tel()["cls_run"] == 1,
          f"locked={rig6.tel()['cls']} run={rig6.tel()['cls_run']}")
    # ⚠️ 熬冷却必须用**同色帧**：`idle()` 复读的是"最后一帧"，而此时最后一帧是 RED ——
    #    连喂 12 帧真的会把类别锁切到 RED，测到的就不是"单帧抖动只吞当拍"了。
    for _ in range(18):                                    # 0.6s @30fps，全 BLUE
        rig6.frame(0, cls=ShotClass.BLUE)
    check("4.10e 回到原类恢复许可（抖动只吞当拍）", rig6.ready() is True,
          f"reason={rig6.reason()} locked={rig6.tel()['cls']}")
    for _ in range(5):
        rig6.frame(0, cls=ShotClass.RED)                   # 连续 5 帧红
    check("4.10f 连续 5 帧变类才改锁", rig6.tel()["cls"] == ShotClass.RED,
          f"locked={rig6.tel()['cls']} run={rig6.tel()['cls_run']}")

    # 4.11 红靶只跟踪不开火（fire_only_blue）
    rig7 = Rig(cfg=cfg_of(fire_only_blue=True))
    for _ in range(8):
        rig7.frame(0, cls=ShotClass.RED)
    check("4.11 fire_only_blue + 红靶 → 0（只跟踪）", rig7.ready() is False,
          f"reason={rig7.reason()} locked={rig7.tel()['cls']}")

    # 4.12 策略可切（接口可用；turnaround 对本靶等价不可用）
    rig8 = Rig(cfg=cfg_of(policy=FirePolicy.TURNAROUND))
    _compile(rig8, 8)
    check("4.12a turnaround 策略：静止目标 → 许可（接口可用）", rig8.ready() is True,
          f"reason={rig8.reason()}")
    for k in range(1, 16):
        rig8.frame(30.0 * k * FRAME_DT)
    check("4.12b turnaround 策略：移动目标 → 0（本靶无端点停顿，判据本就不可用）",
          rig8.ready() is False and rig8.reason().startswith("policy_turnaround"),
          f"reason={rig8.reason()}")
    rig9 = Rig(cfg=cfg_of(policy=FirePolicy.STATIC))
    _compile(rig9, 8)
    check("4.12c static 策略：静止 → 许可", rig9.ready() is True, f"reason={rig9.reason()}")
    rig10 = Rig(cfg=cfg_of(policy="不存在"))
    _compile(rig10, 8, need_ready=False)
    check("4.12d 未知策略 → 一律 0（失败偏安全）", rig10.ready() is False,
          f"reason={rig10.reason()}")


# =========================================================================== #
#  5. 命中即停
# =========================================================================== #
def test_05_hit() -> None:
    section("5. ★ 命中即停（旧缺口④⑤：无命中闭环 / 重锁清零 → 打中还在补射）")
    rig = Rig()
    _compile(rig, 8)
    check("5.1 命中前一直在许可", rig.ready() is True, f"reason={rig.reason()}")

    rig.bridge.add_shots(1)                      # 云台确实打了一发
    rig.bridge.add_hits(1)                       # 裁判回传命中
    rig.frame(0)
    check("5.2 命中后立刻停火", rig.ready() is False and rig.reason() == REASON_HIT_LATCHED,
          f"reason={rig.reason()}")
    check("5.3 P_AIM.hits 累加正确", rig.bb.get(P_AIM).hits == 1,
          f"hits={rig.bb.get(P_AIM).hits}")
    check("5.4 命中事件已发布", rig.bus.published > 0)

    for _ in range(10):
        rig.frame(0)
    check("5.5 即使目标仍合格也不再许可（锁存）", rig.ready() is False,
          f"reason={rig.reason()} hit_latched={rig.tel()['hit_latched']}")

    # 5.6 实发数与许可数对账
    rig.bridge.add_shots(2)
    rig.frame(0)
    check("5.6 actual_shots 用 shoot_count 增量反推（1+2=3）",
          rig.tel()["actual_shots"] == 3 and rig.tel()["shots_seen"] == 3,
          f"actual={rig.tel()['actual_shots']} seen={rig.tel()['shots_seen']} "
          f"permits_total={rig.tel()['permits_total']}")

    # 5.7 重新武装：目标重新出现（丢过）→ 解除锁存（多靶/新目标场景）
    rig.no_vision(0.7)                            # 观测过期 → 判定丢靶
    check("5.7a 丢靶：fire_ready=0、P_TARGET.seen=False",
          rig.ready() is False and rig.bb.get(P_TARGET).seen is False,
          f"seen={rig.bb.get(P_TARGET).seen} lost_since={rig.bb.get(P_TARGET).lost_since:.2f} "
          f"reason={rig.reason()}")
    rig.idle(2.5)                                 # 超过 hit_hold_s 与 rearm_after_lost_s
    _compile(rig, 8)                              # 目标重新出现
    check("5.7b 目标重新出现 → 锁存解除、可再次许可",
          rig.ready() is True and rig.tel()["hit_latched"] is False,
          f"reason={rig.reason()} latched={rig.tel()['hit_latched']}")
    # ⚠️ 原断言 `permits == 1`（窗口计数清零后只发过 1 次许可）要求"重新武装恰好落在
    #    `_compile()` 返回那一拍"。许可是**按冷却窗口累计**的（每 0.5s 一次，与是否真的
    #    开火无关），重新武装之后剩下的 1.5s idle 必然又记满几次 —— 所以 `window==1`
    #    不是这套语义的必然结果。这里断言真正的契约：**窗口被清零重计**（因此严格小于
    #    生命周期累计）且生命周期累计保留（≥2）。
    check("5.7c 重新武装后本靶窗口计数清零、生命周期累计保留",
          rig.tel()["permits"] < rig.tel()["permits_total"]
          and rig.tel()["permits_total"] >= 2,
          f"window={rig.tel()['permits']} total={rig.tel()['permits_total']}")


# =========================================================================== #
#  6. 弹药上限
# =========================================================================== #
def test_06_budget() -> None:
    section("6. ★ 弹药上限（旧缺口④：10 发弹药可能全倾泻在一个靶上）")
    rig = Rig(cfg=cfg_of(max_shots_per_target=3, shot_cooldown_ms=500))
    # ⚠️ 记"许可时刻"必须盯**计数增量**（`permits > prev`）。原写法用
    #    `permits > len(grant_times)` 当基准：循环第 1 拍就会因为"已有 1 发许可 > 记了 0 次"
    #    而记下一个**循环拍**（不是许可时刻）的时间戳，于是第一段间隔被少算 0.05s
    #    （实测 0.45s < 0.5s → 误报"没隔冷却"）。
    #    这里从零开始喂帧、每拍比一次，3 发许可的时刻全部精确记录。
    seen: list[int] = []
    grant_times: list[float] = []
    prev = 0
    for _ in range(120):                      # 4.0s @30fps：够落下 3 发（冷却 0.5s）
        rig.frame(0)                          # 静止目标：许可只受冷却限制
        if rig.task.permits > prev:
            grant_times.append(rig.clock.now())
            prev = rig.task.permits
        seen.append(rig.task.permits)
    check("6.1 许可数到 max_shots_per_target 就停", rig.task.permits == 3,
          f"permits={rig.task.permits}（上限 3）")
    check("6.2 停在上限的原因是 budget_per_target",
          rig.ready() is False and rig.reason() == REASON_BUDGET_PER_TARGET,
          f"reason={rig.reason()}")
    check("6.3 许可单调不减（不会回退）", all(b >= a for a, b in zip(seen, seen[1:])),
          f"{seen[:12]} … {seen[-3:]}")
    gaps = [b - a for a, b in zip(grant_times, grant_times[1:])]
    t0 = grant_times[0] if grant_times else 0.0
    check("6.4 三段许可之间确实隔了冷却（每发 ≥0.5s）",
          len(gaps) == 2 and all(g >= 0.5 - 1e-9 for g in gaps),
          "许可时刻间隔=" + ", ".join(f"{g:.2f}s" for g in gaps) +
          f"；许可时刻 t={['%.2f' % (t - t0) for t in grant_times]}s")

    # 6.5 全场上限（max_shots_total）
    rig2 = Rig(cfg=cfg_of(max_shots_total=1, max_shots_per_target=3))
    _compile(rig2, 8)
    check("6.5 全场发数上限独立生效", rig2.task.permits == 1,
          f"permits={rig2.task.permits}（max_shots_total=1）")
    rig2.bridge.add_shots(1)
    rig2.idle(0.6)
    check("6.6 shoot_count 到全场上限后不再许可",
          rig2.ready() is False and rig2.reason().startswith("budget_total"),
          f"reason={rig2.reason()} shots_seen={rig2.tel()['shots_seen']}")


# =========================================================================== #
#  7. 丢靶闭锁
# =========================================================================== #
def test_07_lost() -> None:
    section("7. ★ 丢靶闭锁（旧缺口⑥：TRACK 丢帧云台冻结最长 6s）")
    rig = Rig(cfg=cfg_of(lost_frames_to_rescan=60))       # 60 帧 @30fps = 2.0s
    _compile(rig, 8)
    check("7.1 丢靶前许可成立", rig.ready() is True, f"reason={rig.reason()}")

    rig.no_vision(0.6)                                # 观测过期（> VISION_MAX_AGE_S=0.5）
    check("7.2 丢靶后**下一拍**即闭锁（不是等 6s）",
          rig.ready() is False and rig.reason().startswith("target_lost"),
          f"reason={rig.reason()}")
    check("7.3 lead 同时归零", close(rig.tel()["lead_px"], 0.0, 1e-12),
          f"lead={rig.tel()['lead_px']!r}")
    tgt = rig.bb.get(P_TARGET)
    check("7.4 超过 0.5s 后 P_TARGET.seen=False", tgt.seen is False, f"seen={tgt.seen}")
    check("7.5 P_TARGET.lost_since 已记录（>0）", tgt.lost_since > 0.0,
          f"lost_since={tgt.lost_since:.3f}")
    check("7.6 丢靶期间始终不许可", rig.ready() is False, f"reason={rig.reason()}")
    check("7.7 丢靶事件已发布",
          rig.count(EV_TARGET_LOST) >= 1, f"{rig.event_types().count(EV_TARGET_LOST)} 条")

    rig.idle(2.2)                                     # 超过 lost_frames/fps = 2.0s
    check("7.8 超过 lost_frames_to_rescan 阈值 → 回扫标志置位",
          rig.tel()["rescan_requested"] is True, f"{rig.tel()['rescan_requested']}")
    # ★★ 卡 t13（用户 2026-10-11 指令）改写了本组的检测口径：
    #    旧契约 = "aim 完全不管 scan_request，决定权全留 Mission"；
    #    新契约 = "**aim 掌管云台扫描开关**：巡迹（未到最佳射击点停站）一律锁止 0，
    #              到 StopKind.SHOOT 停站才起转搜索 2"。旧的 7.9/7.10 断言的正是旧契约，
    #    因此按新契约重写（详见 _tmp_session/t13_gimbal.md 与 test_gimbal_lock.py）。
    check("7.9 ★巡迹中（未到射击停站）只发**锁止** 0，绝不发 1/2",
          rig.bridge.scan_requests == [0], f"{rig.bridge.scan_requests}")

    # 7.10 打开开关：丢靶回扫**只在已到最佳射击点停站**时才发 2
    rig2 = Rig(cfg=cfg_of(rescan_on_lost=True))
    _compile(rig2, 8)
    rig2.no_vision(0.6)
    rig2.idle(2.2)
    check("7.10 ★巡迹途中丢靶回扫**不解锁**云台（仍只有锁止 0）",
          rig2.bridge.scan_requests == [0], f"{rig2.bridge.scan_requests}")
    # （"到 SHOOT 停站后才起转搜索 2"的完整覆盖放在 test_gimbal_lock.py —— 那里用独立夹具，
    #   免得插在这里扰动 7.11 的目标/预算时序。）

    # 7.11 重新出现 → 恢复
    _compile(rig2, 8)
    check("7.11 目标重新出现 → 恢复许可", rig2.ready() is True, f"reason={rig2.reason()}")


# =========================================================================== #
#  8. 安全门
# =========================================================================== #
def test_08_safety() -> None:
    section("8. ★ 安全门（P_SAFETY 非 OK / 缺失 → fire_ready=0）")
    rig = Rig()
    _compile(rig, 8)
    rig.idle(0.6)                                  # 冷却过掉，确保拦停原因只可能是安全门
    rig.frame(0)
    check("8.1 安全 OK 时许可成立", rig.ready() is True, f"reason={rig.reason()}")

    # ⚠️ 原断言写死 `permits == 1` / `== 2`。这在"许可按冷却窗口累计"的语义下不成立：
    #    安全停之前的那段 idle 已经记了 2 发（实测），恢复后又记 1~2 发。
    #    要测的是"安全停期间**不再新增**许可"（8.4）与"恢复后**又**有许可"（8.6），
    #    所以用**相对量**断言 —— 这正是两条用例名字里写的东西。
    permits_before_stop = rig.task.permits
    rig.bb.set(P_SAFETY, SafetyState(ok=False, reasons=("越界",), since=rig.clock.now()), "test")
    rig.task.tick()
    check("8.2 P_SAFETY.ok=False → 0", rig.ready() is False and rig.reason() == REASON_SAFETY,
          f"reason={rig.reason()}")
    check("8.3 安全不 OK 不改变目标可见性（安全是**门**，不是看不见）",
          rig.bb.get(P_TARGET).seen is True,
          f"seen={rig.bb.get(P_TARGET).seen} lead={rig.tel()['lead_px']}")
    check("8.4 安全停期间不再新增许可", rig.task.permits == permits_before_stop,
          f"permits={rig.task.permits}（安全停前 {permits_before_stop}）")

    rig.bb.set(P_SAFETY, SafetyState(ok=True), "test")
    rig.idle(0.6)
    rig.frame(0)
    check("8.5 安全恢复 → 许可恢复", rig.ready() is True, f"reason={rig.reason()}")
    check("8.6 恢复后新增了一次许可（说明是暂停不是报废）",
          rig.task.permits > permits_before_stop,
          f"permits {permits_before_stop} → {rig.task.permits}")

    # 8.7 缺失 P_SAFETY（按不安全处理）
    rig2 = Rig()
    rig2.bb._data.pop(P_SAFETY, None)
    _compile(rig2, 8, need_ready=False)
    check("8.7 P_SAFETY 缺失 → 0（失败偏安全）",
          rig2.ready() is False and rig2.reason() == REASON_SAFETY, f"reason={rig2.reason()}")


# =========================================================================== #
#  9. 不写别人的钥匙
# =========================================================================== #
def test_09_keys() -> None:
    section("9. ★★ 两把钥匙的纪律：AimTask 绝不碰 fire_gate")
    rig = Rig()
    # 把所有分支都跑一遍：无目标 / 有目标 / 许可 / 冷却 / 命中 / 丢靶 / 安全停
    rig.ticks(3)
    _compile(rig, 8)
    rig.idle(0.6)
    rig.bridge.add_hits(1)
    rig.ticks(3)
    rig.bridge.clear_vision()
    rig.ticks(5)
    rig.bb.set(P_SAFETY, SafetyState(ok=False), "test")
    rig.ticks(3)
    rig.task.teardown()

    check("9.1 全程 bridge.fire_gate 为空（AimTask 从不发第一把钥匙）",
          rig.bridge.fire_gate == [], f"{rig.bridge.fire_gate}")
    check("9.2 但 fire_ready 一直在发（每拍保活：%d 条）" % len(rig.bridge.fire_ready),
          len(rig.bridge.fire_ready) >= 20, f"{len(rig.bridge.fire_ready)} 条")
    check("9.3 fire_ready 出现过 True（说明不是一直 0 所以没写）",
          True in rig.bridge.fire_ready, f"True 出现 {rig.bridge.fire_ready.count(True)} 次")
    check("9.4 teardown 之后末值是 False（退出必闭锁）",
          rig.bridge.fire_ready[-1] is False, f"{rig.bridge.fire_ready[-3:]}")
    check("9.5 AimTask 没有 publish_fire_gate 这个属性用法（源码级 grep 见报告）",
          not hasattr(rig.task, "_bridge_fire_gate"))

    # 9.6 源码级检查：aim.py 里不出现 publish_fire_gate
    src = (pathlib.Path(__file__).resolve().parents[1] / "dzbrain" / "tasks" / "aim.py")
    text = src.read_text(encoding="utf-8")
    check("9.6 源码里 publish_fire_gate 只出现在注释/文档字符串里（不作为调用）",
          "self.bridge.publish_fire_gate" not in text,
          f"{src.name} 无该调用")


# =========================================================================== #
#  10. 真线程节拍
# =========================================================================== #
def test_10_thread() -> None:
    section("10. 真线程 50 Hz 节拍（0.3 s）")
    import time as _time
    rig = Rig()
    rig.bridge.set_vision(dx=0, dy=0, cls=ShotClass.BLUE, box_w=45, box_h=55)
    rig.bridge.set_aim_status(AimStatus.TRACK)
    rig.task.start()
    _time.sleep(0.3)
    rig.task.stop(timeout=1.0)
    h = rig.task.health()
    check("10.1 0.3s 内拍数在 50Hz 容差内（10~20）", 10 <= h.ticks <= 20, f"ticks={h.ticks}")
    check("10.2 errors == 0", h.errors == 0, f"errors={h.errors} last={h.last_error!r}")
    check("10.3 未发生 tick 异常且每拍都发过 fire_ready",
          len(rig.bridge.fire_ready) >= h.ticks - 2,
          f"fire_ready={len(rig.bridge.fire_ready)} ticks={h.ticks}")
    check("10.4 真线程下仍然没碰 fire_gate", rig.bridge.fire_gate == [],
          f"{rig.bridge.fire_gate}")


# =========================================================================== #
def main() -> None:
    for name, fn in (("00", test_00_pure), ("01", test_01_estimator), ("02", test_02_lead),
                     ("03", test_03_turnaround), ("04", test_04_criteria), ("05", test_05_hit),
                     ("06", test_06_budget), ("07", test_07_lost), ("08", test_08_safety),
                     ("09", test_09_keys), ("10", test_10_thread)):
        run_case(f"test_{name}", fn)
    finish()


if __name__ == "__main__":
    main()


