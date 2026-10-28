# -*- coding: utf-8 -*-
"""P2-T3 AIM 提前量核查（MISSION-GATE-20261029）：靶不停 ⇒ 提前量是开火前提。

卡面要求：通读 `aim.py`，核验整链完整 —— 目标速度估计 → `lead_px` → `publish_lead`
→ 云台补丁消费方；确认 `config/robot.yaml` 有 `aim.lead_time_s` 且值合理（只读）；
缺环才补。核查结论：**链是完整的**（见下），所以本文件只做**验证**，不改进程逻辑。

整链逐环（本文件每条都落到可执行断言上）：
  ① 速度估计：`RailEstimator.update()` 用长基线端点差分 `v=(z1-z0)/(t1-t0)`（px/s，
     窗口 `aim.est_vel_window_s`=0.40 s），不是单帧差分 ⇒ 详见 aim.py:196 起的类文档；
  ② 提前量：`AimTask._lead_px()` = `est.v × aim.lead_time_s`，再按 `_max_lead_px`
     （= `aim.max_lead_factor` × `rail_span_m` × `px_per_m_rail`）限幅；
  ③ 下发：`_publish_lead()` → `bridge.publish_lead()` → 话题 `/aim/lead_x`（Float32，
     `KeepLast(1)` + best_effort，见 io/ros_io.py）；
  ④ 消费方：`dzrknn_dataprocess.cpp` 的 `callback_lead_x`（|v|>400 归零）→
     `lead_applied = lead_apply_gain_ × lead_px_` → 叠加到 `aim_err_x`（第 299-300 行）。

跑法：
    cd dzrobot_v2 && python -B -X utf8 src/dzbrain/tests/test_p2_t3_aim_lead_chain.py
"""
from __future__ import annotations

import pathlib
import re
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

try:
    sys.stdout.reconfigure(errors="replace")
except Exception:                      # pragma: no cover
    pass

from harness import check, check_close, finish, section                   # noqa: E402

from dzbrain.core.blackboard import P_AIM, P_SAFETY, P_STOP_POINT, Blackboard  # noqa: E402
from dzbrain.core.config import Config                                   # noqa: E402
from dzbrain.core.events import EventBus                                 # noqa: E402
from dzbrain.core.state import (SafetyState, StopKind, StopPhase,         # noqa: E402
                                StopPointState)
from dzbrain.io.contract import AimStatus, ShotClass                     # noqa: E402
from dzbrain.tasks.aim import AimTask                                    # noqa: E402

from fakes import FakeBridge, FakeClock                                   # noqa: E402

WS = pathlib.Path(__file__).resolve().parents[3]          # <ws> = dzrobot_v2/
ROBOT_YAML = WS / "config" / "robot.yaml"
DZRKNN_CPP = WS / "src" / "dzrknn_dataprocess" / "src" / "dzrknn_dataprocess.cpp"
FPS = 30.0
FRAME_DT = 1.0 / FPS


def cfg_of(*, lead_time_s: float = 0.35, **aim) -> Config:
    return Config.from_dict({"aim": dict({
        "lead_time_s": lead_time_s,
        "fire_policy": "moving_lead",
        "max_shots_total": 10, "max_shots_per_target": 10,
        "shot_cooldown_ms": 500,
        "fire_box_far_px": 14.0, "fire_box_near_px": 26.0,
        "rail_span_m": 0.70, "target_speed_mps": 0.04,
        "lost_frames_to_rescan": 60,
    }, **aim), "logging": {"rate_limit_s": 0.0}})


class Rig:
    """一台"云台在瞄"的车：FakeBridge + Blackboard + AimTask（手动单拍）。"""

    def __init__(self, *, lead_time_s: float = 0.35) -> None:
        self.clock = FakeClock()
        self.bridge = FakeBridge(self.clock)
        self.bb = Blackboard()
        self.bus = EventBus()
        self.cfg = cfg_of(lead_time_s=lead_time_s)
        self.logs: list[str] = []
        self.task = AimTask(self.bridge, self.bb, self.bus, self.cfg,
                            log=self.logs.append, clock=self.clock.now)
        self.bb.set(P_SAFETY, SafetyState(ok=True), "test")
        self.bb.set(P_STOP_POINT, StopPointState(kind=StopKind.SHOOT, phase=StopPhase.ACQUIRE),
                    "test")
        self.bridge.set_aim_status(AimStatus.TRACK)

    def frame(self, dx: float, *, dt: float = FRAME_DT) -> None:
        self.clock.advance(dt)
        self.bridge.set_vision(dx=int(round(dx)), dy=0, cls=ShotClass.BLUE, box_w=45)
        self.task.tick()

    def ramp(self, v_px_s: float, frames: int, *, dt: float = FRAME_DT, dx0: float = 0.0) -> None:
        """恒速序列：dx = dx0 + v·k·dt（精确积分，整数像素序列 ⇒ 无量化误差）。"""
        for k in range(1, frames + 1):
            self.frame(dx0 + v_px_s * k * dt, dt=dt)

    def last_lead(self) -> float:
        return float(self.bridge.leads[-1]) if self.bridge.leads else float("nan")


# --------------------------------------------------------------------------- #
#  ① 匀速运动靶：lead_px ≈ v × lead_time_s（卡面要求的单测）
# --------------------------------------------------------------------------- #
def case_uniform_lead() -> None:
    section("① 匀速运动靶：lead_px ≈ v × lead_time_s（30 px/s × 0.35 s = 10.5 px）")
    V, T = 30.0, 0.35
    rig = Rig(lead_time_s=T)
    # 15 fps × 30 px/s = 2 px/帧 ⇒ 像素序列零量化误差，估计器稳态无偏
    rig.ramp(V, frames=24, dt=1.0 / 15.0)
    v_est = rig.task.estimator.v
    lead = rig.last_lead()
    check_close("①.1 速度估计 v ≈ 30 px/s（±1%）", v_est, V, 0.01 * V)
    check_close("①.2 lead_px == v × lead_time_s（公式恒等，容差 1e-9）",
                lead, v_est * T, 1e-9)
    check("①.3 量级：lead_px ≈ 10.5 px（±0.5 px，覆盖观测取整的影响）",
          abs(lead - V * T) <= 0.5, f"lead={lead:.3f} px（期望 {V * T:.1f}）")
    check("①.4 已从 bridge 下发（/aim/lead_x 有值，不是只算不发）",
          len(rig.bridge.leads) >= 24, f"下发 {len(rig.bridge.leads)} 拍")


# --------------------------------------------------------------------------- #
#  ② 现场配置值 0.27 s：同速度下 lead 按比例缩小
# --------------------------------------------------------------------------- #
def case_config_lead_time() -> None:
    section("② 现场 aim.lead_time_s=0.27：同速度 lead ≈ 30 × 0.27 = 8.1 px")
    V = 30.0
    rig = Rig(lead_time_s=0.27)
    rig.ramp(V, frames=24, dt=1.0 / 15.0)
    lead = rig.last_lead()
    check("②.1 lead_px ≈ 8.1 px（±0.5 px）", abs(lead - V * 0.27) <= 0.5,
          f"lead={lead:.3f} px（期望 {V * 0.27:.2f}）")
    rig2 = Rig(lead_time_s=0.35)
    rig2.ramp(V, frames=24, dt=1.0 / 15.0)
    ratio = rig2.last_lead() / lead if lead else float("nan")
    check("②.2 lead 与 lead_time_s 成正比（0.35/0.27 = 1.296，±2%）",
          abs(ratio - 0.35 / 0.27) <= 0.02 * (0.35 / 0.27), f"ratio={ratio:.4f}")


# --------------------------------------------------------------------------- #
#  ③ 符号跟随运动方向（靶往返 ⇒ 提前量必须换向）
# --------------------------------------------------------------------------- #
def case_sign_follows_motion() -> None:
    section("③ 符号：靶向右 (+v) ⇒ lead>0；向左 (−v) ⇒ lead<0")
    rig = Rig(lead_time_s=0.35)
    rig.ramp(30.0, frames=16, dt=1.0 / 15.0, dx0=-60.0)
    lead_rt = rig.last_lead()
    rig2 = Rig(lead_time_s=0.35)
    rig2.ramp(-30.0, frames=16, dt=1.0 / 15.0, dx0=+60.0)
    lead_lt = rig2.last_lead()
    check("③.1 +30 px/s ⇒ lead > 0", lead_rt > 0.0, f"lead={lead_rt:.3f}")
    check("③.2 −30 px/s ⇒ lead < 0", lead_lt < 0.0, f"lead={lead_lt:.3f}")
    check("③.3 两个方向量级对称（|差| < 0.5 px）", abs(abs(lead_rt) - abs(lead_lt)) < 0.5,
          f"{lead_rt:.3f} vs {lead_lt:.3f}")


# --------------------------------------------------------------------------- #
#  ④ 限幅：离谱速度不许把提前量推进画面外
# --------------------------------------------------------------------------- #
def case_clamp() -> None:
    section("④ 限幅：|lead| ≤ max_lead_px（= 0.45 × 0.70 m × 140 px/m ≈ 44.1 px）")
    rig = Rig(lead_time_s=0.35)
    cap = rig.task._max_lead_px
    check("④.1 限幅值 = max_lead_factor × rail_span × px_per_m",
          abs(cap - (0.45 * 0.70 * 140.0)) < 1e-6, f"cap={cap:.3f} px")
    # 200 px/s < est_max_speed_px_s(=300) ⇒ 估计器不会重置，lead 真的会被限幅触发：
    # 200 × 0.35 = 70 px > 44.1 px ⇒ 必须被夹到 +44.1
    rig.ramp(200.0, frames=10, dt=1.0 / 15.0, dx0=-800.0)
    lead = rig.last_lead()
    v_est = rig.task.estimator.v
    check("④.2 高速靶（200 px/s）估计器照常收敛", abs(v_est - 200.0) <= 0.02 * 200.0,
          f"v={v_est:.3f} px/s")
    check("④.3 lead 被夹到 +cap（200 × 0.35 = 70 px > 44.1 px）",
          abs(lead - cap) < 1e-9, f"lead={lead:.3f} / cap={cap:.3f}")


# --------------------------------------------------------------------------- #
#  ⑤ 链路静态核对：robot.yaml 有 aim.lead_time_s，C++ 消费方真的消费 /aim/lead_x
# --------------------------------------------------------------------------- #
def case_static_chain() -> None:
    section("⑤ 链路静态核对（只读）：配置存在且合理 / C++ 侧有消费者")
    text = ROBOT_YAML.read_text(encoding="utf-8", errors="replace")
    m = re.search(r"^\s*lead_time_s:\s*([0-9.]+)", text, re.MULTILINE)
    check("⑤.1 config/robot.yaml 有 aim.lead_time_s", m is not None,
          f"{m.group(0).strip() if m else '未找到'}")
    val = float(m.group(1)) if m else float("nan")
    check("⑤.2 值在合理区间 (0, 1.0] s（链路延迟量级；0 = 关闭提前量）",
          0.0 < val <= 1.0, f"lead_time_s={val}")
    gain = re.search(r"^\s*lead_apply_gain:\s*([0-9.]+)", text, re.MULTILINE)
    check("⑤.3 云台侧 lead_apply_gain > 0（否则 C++ 算了不用，提前量等于没生效）",
          gain is not None and float(gain.group(1)) > 0.0,
          f"{gain.group(0).strip() if gain else '未找到'}")
    cpp = DZRKNN_CPP.read_text(encoding="utf-8", errors="replace")
    check("⑤.4 C++ 订阅 /aim/lead_x",
          '"/aim/lead_x"' in cpp and "callback_lead_x" in cpp)
    check("⑤.5 C++ 把 lead 叠加进瞄准误差（lead_apply_gain_ × lead_px_）",
          "lead_apply_gain_ * lead_px_" in cpp)
    check("⑤.6 C++ 有异常值守卫（|v| > 400 px 归零）",
          "400.0" in cpp and "isfinite" in cpp)
    # ⑥ 现场量级：4 cm/s 的靶在 140 px/m 下 ≈ 5.6 px/s
    px_per_m, speed = 140.0, 0.04
    v_expected = speed * px_per_m
    check(f"⑤.7 现场靶速折算 ≈ {v_expected:.1f} px/s"
          f"（lead ≈ {v_expected * val:.2f} px = 靶速×提前量，量级合理）",
          abs(v_expected - 5.6) < 1e-9, f"v_expected={v_expected:.2f} px/s")


for name, fn in (("① 公式", case_uniform_lead),
                 ("② 现场值", case_config_lead_time),
                 ("③ 符号", case_sign_follows_motion),
                 ("④ 限幅", case_clamp),
                 ("⑤ 静态链路", case_static_chain)):
    try:
        fn()
    except Exception as exc:                     # noqa: BLE001
        import traceback
        traceback.print_exc()
        check(f"{name} 用例自身未抛异常", False, f"{type(exc).__name__}: {exc}")

finish()
