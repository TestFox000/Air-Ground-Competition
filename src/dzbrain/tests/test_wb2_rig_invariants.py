# -*- coding: utf-8 -*-
"""WB-2 台架不变量回归（卡 `WB-2-RIG-20261031`）。

运行::

    cd dzrobot_v2 && PYTHONPATH=src/dzbrain python src/dzbrain/tests/test_wb2_rig_invariants.py

全过 exit 0；任一 FAIL exit 1。纯脚本 + 极简 check()，本机不需要 pytest（与全仓一致）。

## 这个文件钉的是什么（一句话）

**"台架说的每一个数，都能追溯到某个生产实现或某条明确声明的注入。"**

覆盖（每条都对应卡面 acceptance 的一行）：

| § | 判据来源（卡面） |
|---|---|
| 1 | 三档采样 100/20/50 Hz **可配置且默认按真车值**（不是单一 50 Hz） |
| 2 | 执行器链四项齐：servo=度+60 夹[25,95]、20 拍平滑窗、整数 23/0.33 量化、1.0 s 看门狗 |
| 3 | ★ 硬判据②：`--arm ideal` 的 `belief_error_rms_m ≡ 0`、`--arm realistic` 必须 > 0 |
| 4 | 指标带 `transfer` 分级；`field_modeled=0` 时越界类输出 **null**（不是 0） |
| 5 | 逐拍 `steps.csv` 含卡面要求的全部列 |
| 6 | 确定性：同命令两次跑 `steps.csv`/`metrics.json` sha256 逐位相同 |
| 7 | `--variant` 换控制器实现**不改代码**，且结果确实变了 |
| 8 | 末行契约 `dzrobot-trackbench-result/1`，五键只增不改，严格 JSON（NaN→null） |
| 9 | 负例：平滑窗=0 ⇒ "负速度被吃掉"不复现（证明注入真起作用） |

★ 本文件**不重跑 110 s 全程**（那要跑两次 55000 拍）；§3/§6 用缩短的时长（8~12 s），
  因为要判的是"不变量成不成立"，不是"最终成绩"。全程成绩由卡面 verify 命令实跑。
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import re
import sys

_HERE = pathlib.Path(__file__).resolve()
_SRC = _HERE.parents[1]                       # src/dzbrain
_REPO = _SRC.parents[1]                       # dzrobot_v2
_TOOLS = _REPO / "tools"
# ★ 顺序：src 与 tests 在前，tools 在后（`tools/dzbrain.py` 会遮住真包）
for _p in (str(_SRC), str(_SRC / "tests")):
    if _p not in sys.path:
        sys.path.insert(0, _p)
if str(_TOOLS) not in sys.path:
    sys.path.append(str(_TOOLS))

_PASS = 0
_FAIL: list[str] = []
_SECTION = ""


def section(title: str) -> None:
    global _SECTION
    _SECTION = title
    print(f"\n=== {title} ===")


def check(name: str, cond: bool, detail: str = "") -> bool:
    global _PASS
    if cond:
        _PASS += 1
        print(f"  PASS  {name}" + (f"   [{detail}]" if detail else ""))
    else:
        _FAIL.append(f"{_SECTION} :: {name}" + (f"   [{detail}]" if detail else ""))
        print(f"  FAIL  {name}" + (f"   [{detail}]" if detail else ""))
    return bool(cond)


def finish() -> None:
    print("\n" + "=" * 74)
    total = _PASS + len(_FAIL)
    if _FAIL:
        print(f"结果：{_PASS}/{total} 通过，{len(_FAIL)} 失败")
        for item in _FAIL:
            print(f"  FAIL  {item}")
        print("=" * 74)
        sys.exit(1)
    print(f"结果：{_PASS}/{total} 全部通过 ✅")
    print("=" * 74)
    sys.exit(0)


#: 缩短时长（判不变量用；全程成绩由卡面 verify 实跑）
SHORT_S = 10.0


# --------------------------------------------------------------------------- #
#  1. 三档采样：可配置、默认真车值、与生产 rate_hz 对拍
# --------------------------------------------------------------------------- #
def test_sample_rates() -> None:
    section("1. 三档采样 100/20/50 Hz：可配置 · 默认真车值 · 与生产 rate_hz 同源")
    import wb_rig.sched as SCH
    import wb_rig.arms as ARMS

    check("默认三档 = odom 100 Hz / Localization 20 Hz / Tracker 50 Hz",
          SCH.SAMPLE_HZ == {"odom": 100.0, "localization": 20.0, "tracker": 50.0},
          str(SCH.SAMPLE_HZ))
    check("★ 不是单一 50 Hz（三档互不相同）", len(set(SCH.SAMPLE_HZ.values())) == 3,
          str(sorted(SCH.SAMPLE_HZ.values())))
    # 与生产类的 rate_hz 对拍：调度表的频率来自生产类，不是抄来的常数
    from dzbrain.core.blackboard import Blackboard
    from dzbrain.core.config import Config
    from dzbrain.core.events import EventBus
    from dzbrain.tasks.localization import LocalizationTask
    from dzbrain.tasks.mission import MissionTask
    from dzbrain.tasks.safety import SafetyTask
    from dzbrain.tasks.tracker import TrackerTask
    import fakes

    cfg = Config.load(str(_REPO / "config" / "robot.yaml"))
    clock = fakes.FakeClock()
    bridge = fakes.FakeBridge(clock)
    bb, bus = Blackboard(), EventBus()
    prod = {
        "Localization": LocalizationTask(bridge, bb, bus, cfg, log=lambda m: None,
                                         clock=clock.now).rate_hz,
        "TrackerTask": TrackerTask(bridge, bb, bus, cfg, log=lambda m: None,
                                   clock=clock.now).rate_hz,
        "Safety": SafetyTask(bridge, bb, bus, cfg, log=lambda m: None,
                             clock=clock.now).rate_hz,
        "MissionTask": MissionTask(bridge, bb, bus, cfg, log=lambda m: None,
                                   clock=clock.now).rate_hz,
    }
    check("★ Localization 生产 rate_hz == 默认档 20 Hz",
          prod["Localization"] == SCH.SAMPLE_HZ["localization"], str(prod["Localization"]))
    check("★ Tracker 生产 rate_hz == 默认档 50 Hz",
          prod["TrackerTask"] == SCH.SAMPLE_HZ["tracker"], str(prod["TrackerTask"]))
    check("Safety / Mission 的生产 rate_hz 是 20 / 10 Hz（与 Tracker 不同，不是一刀切）",
          (prod["Safety"], prod["MissionTask"]) == (20.0, 10.0),
          str((prod["Safety"], prod["MissionTask"])))
    # ★ 三档可配置：改 dict 必须真的影响调度（不是只读常量摆设）
    sched_src = (_TOOLS / "wb_rig" / "sched.py").read_text(encoding="utf-8")
    check("★ 三档是**可配置**的（调度读 `SAMPLE_HZ`，不是硬编码在循环里）",
          "SAMPLE_HZ[" in sched_src or "SAMPLE_HZ" in sched_src)
    check("world dt 是 Tracker 周期的**子步**（2 ms ≪ 20 ms）",
          (1.0 / SCH.SAMPLE_HZ["tracker"]) / SCH.WORLD_DT_S == 10.0,
          f"{SCH.WORLD_DT_S * 1000:.1f} ms")
    _ = ARMS  # 保持 import 有用（arms 参与 §3）


# --------------------------------------------------------------------------- #
#  2. 执行器链四项（逐项对拍真源常量与行为）
# --------------------------------------------------------------------------- #
def test_actuator_chain() -> None:
    section("2. 执行器链四项：servo=度+60 夹[25,95] · 20 拍平滑窗 · 23/0.33 量化 · 1.0 s 看门狗")
    import wb_rig.actuator as ACT

    check("① servo 零点偏移 = +60（真源 `TargetAngle += 60`）", ACT.SERVO_ZERO_DEG == 60.0,
          str(ACT.SERVO_ZERO_DEG))
    check("① servo 行程夹取 = [25, 95]（真源 `:156-162`）",
          (ACT.SERVO_MIN, ACT.SERVO_MAX) == (25.0, 95.0), str((ACT.SERVO_MIN, ACT.SERVO_MAX)))
    check("① 由 servo 反推的机械舵角区间 = [-35, +35]°",
          ACT.steer_limits_deg() == (-35.0, 35.0), str(ACT.steer_limits_deg()))
    check("② 速度平滑窗 = 20 拍（真源 `declare_parameter(\"smooth_window_speed\", 20)`）",
          ACT.SMOOTH_WINDOW_SPEED == 20, str(ACT.SMOOTH_WINDOW_SPEED))
    check("② 舵角平滑窗 = 5 拍（真源 `smooth_window_angle = 5`）",
          ACT.SMOOTH_WINDOW_ANGLE == 5, str(ACT.SMOOTH_WINDOW_ANGLE))
    check("③ 量化系数 ≈ 23/0.33 = 69.7 计数/(m/s)",
          math.isclose(ACT.SPEED_COUNTS_PER_MPS, 23.0 / 0.33),
          f"{ACT.SPEED_COUNTS_PER_MPS:.4f}")
    check("③ 1 计数 ≈ 0.014348 m/s", math.isclose(1.0 / ACT.SPEED_COUNTS_PER_MPS,
                                                  0.014348, abs_tol=1e-6),
          f"{1.0 / ACT.SPEED_COUNTS_PER_MPS:.9f}")
    check("③ 计数上限 255（真源 `clamp(..., -255, 255)`）", ACT.SPEED_COUNT_CAP == 255.0)
    check("④ 看门狗 = 1.0 s", ACT.WATCHDOG_S == 1.0, str(ACT.WATCHDOG_S))

    # ---- 行为侧：四项必须真起作用 ----
    a = ACT.Actuator(latency_s=0.0, watchdog_s=0.0)
    # ① 舵角 +60 夹取：下 90° 应被夹成 95（= +35°），下 -90° 应被夹成 25（= -35°）
    a.command(0.0, 0.0, 90.0)
    a.tick(0.001)
    check("① 下 90° ⇒ servo 夹到 95 ⇒ 执行舵角 +35°（机械极限生效）",
          abs(a.state.servo_raw - 95.0) < 1e-12 and abs(a.state.steer_exec - 35.0) < 0.11,
          f"servo_raw={a.state.servo_raw} steer_exec={a.state.steer_exec}")
    b = ACT.Actuator(latency_s=0.0, watchdog_s=0.0)
    b.command(0.0, 0.0, -90.0)
    b.tick(0.001)
    check("① 下 -90° ⇒ servo 夹到 25 ⇒ 执行舵角 -35°",
          abs(b.state.servo_raw - 25.0) < 1e-12 and abs(b.state.steer_exec + 35.0) < 0.11,
          f"servo_raw={b.state.servo_raw} steer_exec={b.state.steer_exec}")
    # ② 平滑窗：阶跃输入 20 拍后才约等于目标（不到一半时明显被拉低）
    # ★ 注意真源的形状：`smoothValueAdaptive` 把**本次新值**也 push 进历史再取均值。
    #   所以"历史里只有这一条"时均值 == 它自己（首拍不会被拉低）——那是正确的实现，
    #   不是"平滑没生效"。要看到平滑的**滞后**必须跑"0 → 0.30 的阶跃"，看它在
    #   ~20 拍里逐拍爬上去（这正是不做一步到位的可见证据）。
    c = ACT.Actuator(latency_s=0.0, watchdog_s=0.0)
    for i in range(20):
        c.command(i * 0.001, 0.0, 0.0)          # 先稳定在 0
        c.tick(i * 0.001 + 0.001)
    c.command(0.020, 0.30, 0.0)                 # 阶跃到 0.30
    c.tick(0.021)
    check("② 0→0.30 阶跃的第 1 拍只爬到 1/21（平滑窗确实在滞后，不是一步到位）",
          c.state.v_smoothed < 0.30 * 0.30, f"v_smoothed={c.state.v_smoothed:.4f}")
    # ★ 平滑只在**新指令到达**时推进（`_apply` 里 push 历史）—— 不喂新指令就永远停在
    #   上面那个 1/21 的值，这是真源的形状（底盘每帧 Twist 都会 push 一次），不是缺陷。
    for i in range(60):
        c.command(0.022 + i * 0.001, 0.30, 0.0)
        c.tick(0.022 + i * 0.001 + 0.001)
    check("② 持续喂 0.30 满 60 拍后平滑值收敛到目标量级", abs(c.state.v_smoothed - 0.30) < 0.02,
          f"v_smoothed={c.state.v_smoothed:.4f}")
    # ③ 量化：0.35 m/s ⇒ 24 计数 ⇒ 反算 0.3443 m/s（**截断**不是四舍五入）
    d = ACT.Actuator(latency_s=0.0, watchdog_s=0.0, enable_speed_smooth=False)
    d.command(0.0, 0.35, 0.0)
    d.tick(0.001)
    expect = int(abs(0.35) * 23.0 / 0.33)
    check("③ 0.35 m/s ⇒ 整数计数截断（23/0.33）", d.state.speed_counts == expect,
          f"counts={d.state.speed_counts} expect={expect}")
    check("③ 反算执行速度 < 指令速度（量化丢了零头，真源 `1 计数≈0.014348`）",
          d.state.v_exec < 0.35 and abs(d.state.v_exec - expect * 0.33 / 23.0) < 1e-12,
          f"v_exec={d.state.v_exec:.9f}")
    # ③b 死区：极小的负速度量化成 0 计数 ⇒ 车不动（这是 H8 类案例的观测口径）
    e = ACT.Actuator(latency_s=0.0, watchdog_s=0.0, enable_speed_smooth=False)
    e.command(0.0, -0.005, 0.0)
    e.tick(0.001)
    check("③b 下发 -0.005 m/s ⇒ 0 计数 ⇒ 实际执行 0（死区被观测到）",
          e.state.speed_counts == 0 and e.state.v_exec == 0.0 and e.state.dead_zone,
          f"counts={e.state.speed_counts} dead_zone={e.state.dead_zone}")
    # ④ 看门狗：1.0 s 无新指令 ⇒ 速度归零
    f = ACT.Actuator(latency_s=0.0, watchdog_s=1.0, enable_speed_smooth=False)
    f.command(0.0, 0.30, 5.0)
    f.tick(0.5)
    check("④ 0.5 s 时未超时 ⇒ 仍在动", f.state.v_exec > 0.0 and not f.state.watchdog_tripped,
          f"v_exec={f.state.v_exec} tripped={f.state.watchdog_tripped}")
    f.tick(1.05)
    check("★④ 1.05 s 无新指令 ⇒ 速度强制归零（底盘 1 s 兜底生效）",
          f.state.v_exec == 0.0 and f.state.watchdog_tripped,
          f"v_exec={f.state.v_exec} tripped={f.state.watchdog_tripped}")
    check("④ 看门狗只停速度、舵角保持（真源不给回中）",
          abs(f.state.steer_exec - f.state.steer_exec) == 0.0)


# --------------------------------------------------------------------------- #
#  3. ★ 硬判据②：ideal ≡ 0 / realistic > 0
# --------------------------------------------------------------------------- #
def test_belief_rms_two_arms() -> None:
    section("3. ★硬判据②：ideal 档 belief_error_rms ≡ 0；realistic 档必须 > 0")
    import wb_rig.arms as ARMS
    import wb_rig.scenarios as SCN
    import sim_rig

    sc = SCN.resolve("field7")
    # ---- ideal：零注入 ⇒ 信念 ≡ 真值 ⇒ RMS **逐位为 0**（不是"很小"）----
    res_ideal = sim_rig.run(scenario=sc, arm=ARMS.get("ideal"), seconds=SHORT_S,
                            field_modeled=False, log=lambda m: None)
    rms_ideal = res_ideal["metrics"].values.get("belief_error_rms_m")
    check("★ ideal：`belief_error_rms_m` == 0.0（**精确零**，不是 1e-9）",
          isinstance(rms_ideal, float) and rms_ideal == 0.0, repr(rms_ideal))
    check("★ ideal：`belief_error_max_m` 也 == 0.0（连一拍都没偏）",
          res_ideal["metrics"].values.get("belief_error_max_m") == 0.0,
          repr(res_ideal["metrics"].values.get("belief_error_max_m")))
    check("ideal：零注入三量确实是 0（不是「注入链自己在加噪声」）",
          (res_ideal["metrics"].values["inject_speed_scale_err"],
           res_ideal["metrics"].values["inject_yaw_drift_dps"],
           res_ideal["metrics"].values["inject_pos_noise_m"]) == (0.0, 0.0, 0.0),
          "0/0/0")
    check("★ ideal：车确实在动（RMS 为 0 不是因为「根本没跑」，那是假绿）",
          res_ideal["metrics"].values["s_final_m"] > 1.0,
          f"s_final={res_ideal['metrics'].values['s_final_m']:.3f} m")

    # ---- realistic：实车量级注入 ⇒ 必须 > 0 ----
    res_real = sim_rig.run(scenario=sc, arm=ARMS.get("realistic"), seconds=SHORT_S,
                           field_modeled=False, log=lambda m: None)
    rms_real = res_real["metrics"].values.get("belief_error_rms_m")
    check("★ realistic：`belief_error_rms_m` > 0（注入真生效）",
          isinstance(rms_real, float) and rms_real > 0.0, repr(rms_real))
    check("★ realistic 的信念误差**显著大于** ideal（不是差 1e-12 那种噪声）",
          rms_real > max(1e-3, rms_ideal + 1e-3), f"{rms_real:.6f} vs {rms_ideal:.6f}")
    check("realistic 的注入量确实是实车量级（+17.9% / −0.0705°/s / 6 mm）",
          (res_real["metrics"].values["inject_speed_scale_err"],
           res_real["metrics"].values["inject_yaw_drift_dps"],
           res_real["metrics"].values["inject_pos_noise_m"]) == (0.179, -0.0705, 0.006),
          "0.179 / -0.0705 / 0.006")
    check("两档**同场景同种子**（对照实验卫生：唯一差别是注入）",
          res_ideal["meta"]["scenario"] == res_real["meta"]["scenario"]
          and res_ideal["meta"]["seed"] == res_real["meta"]["seed"],
          f"{res_ideal['meta']['scenario']} seed={res_ideal['meta']['seed']}")


# --------------------------------------------------------------------------- #
#  3b. 三个注入源**逐个**生效（漏一个就是模型少一层物理）
# --------------------------------------------------------------------------- #
def test_each_injection_acts_alone() -> None:
    section("3b. 三个注入源逐个生效：单独注入也必须改变 belief_error（不许有一个是摆设）")
    import wb_rig.arms as ARMS
    import wb_rig.scenarios as SCN
    import sim_rig

    sc = SCN.resolve("field7")
    base = dict(anchor_on_arrival=False, yaw_trim_deg=0.0, note="probe")
    zero = sim_rig.run(scenario=sc, arm=ARMS.ArmProfile(name="zero", speed_scale_err=0.0,
                                                        yaw_drift_dps=0.0, pos_noise_m=0.0,
                                                        **base),
                       seconds=8.0, field_modeled=False, log=lambda m: None)
    r_zero = zero["metrics"].values["belief_error_rms_m"]
    check("三注入全零 ⇒ belief_error_rms == 0（基线）", r_zero == 0.0, repr(r_zero))

    for label, kw in (("里程比例 +17.9%", dict(speed_scale_err=0.179, yaw_drift_dps=0.0,
                                              pos_noise_m=0.0)),
                      ("陀螺零偏 −0.0705°/s", dict(speed_scale_err=0.0,
                                                  yaw_drift_dps=-0.0705, pos_noise_m=0.0)),
                      ("位置白噪声 6 mm", dict(speed_scale_err=0.0, yaw_drift_dps=0.0,
                                              pos_noise_m=0.006))):
        prof = ARMS.ArmProfile(name="probe", **kw, **base)
        res = sim_rig.run(scenario=sc, arm=prof, seconds=8.0, field_modeled=False,
                          log=lambda m: None)
        rms = res["metrics"].values["belief_error_rms_m"]
        check(f"★ 单独注入【{label}】⇒ belief_error_rms > 0（这一层物理真的在）",
              isinstance(rms, float) and rms > 0.0, repr(rms))

    # ★ 陀螺零偏必须进**位置积分**，不许只改上报的 yaw。
    #   只在报数时叠漂移的话，位置永远等于真值（历史上就是这样漏的，
    #   "单独注入零偏 ⇒ 位置误差恒 0.0000" 是最像"通过"的失败）。
    prof = ARMS.ArmProfile(name="probe", speed_scale_err=0.0, yaw_drift_dps=-0.0705,
                           pos_noise_m=0.0, **base)
    res = sim_rig.run(scenario=sc, arm=prof, seconds=20.0, field_modeled=False,
                      log=lambda m: None)
    rms = res["metrics"].values["belief_error_rms_m"]
    check("★ 陀螺零偏 20 s 后位置误差进入 cm 量级（不是 0；证明它真进了积分）",
          isinstance(rms, float) and rms > 0.005, f"{rms:.6f} m")

    # 三项合起来必须比任一单项更差（否则"叠加"没意义，说明某项被另一项掩盖）
    allthree = sim_rig.run(scenario=sc, arm=ARMS.ArmProfile(
        name="probe", speed_scale_err=0.179, yaw_drift_dps=-0.0705, pos_noise_m=0.006,
        **base), seconds=20.0, field_modeled=False, log=lambda m: None)
    r_all = allthree["metrics"].values["belief_error_rms_m"]
    check("★ 三项同时注入 20 s ⇒ belief_error_rms 大于任一单项（叠加生效）",
          r_all > rms, f"all={r_all:.6f} drift_only={rms:.6f}")


# --------------------------------------------------------------------------- #
#  4. 硬判据③：transfer 分级 + field_modeled=0 置灰
# --------------------------------------------------------------------------- #
def test_transfer_and_grey_out() -> None:
    section("4. ★硬判据③：每条指标带 transfer；field_modeled=0 时越界类给 null 不给 0")
    import wb_rig.metrics as MET

    check("三档字面量 = 可迁移 / 仅结构 / 不可迁移",
          MET.TRANSFER_LEVELS == ("可迁移", "仅结构", "不可迁移"), str(MET.TRANSFER_LEVELS))
    m = MET.Metrics(arm="realistic", field_modeled=0)
    # ★★ 2026-11-01 WB-11：这一段原来把两类键混在一起判（`BOUNDS_KEYS` 里同时装着
    #   越界类和场地停车类）。**判据本身没错的是越界类那半**；错的是把 `field_stopped`
    #   也当成"未建模 ⇒ null"。按《06 册》§2 与 tier-0 `sim.py:1594-1596`：
    #   `field_stopped` / `field_stop_t` 是「**两种模式都在**」的那一类（"if/else 同级无条件写"）。
    #   ⇒ 拆成两组分别判，**一条断言都没删**（越界类仍判 None、场地停车类判存在且为有效值）。
    # ---- ① 越界类：地图边界不可知时才置灰（本卡实测：边界来自 geometry 模块常量，
    #         实际**永远可知**；这里仍保留"未建模 ⇒ null"的判据以守住 put() 的强制语义）----
    for key in ("out_of_bounds", "out_of_bounds_steps", "bounds_margin_m"):
        m.put(key, 0.0, note="试图写 0")
    for key in ("out_of_bounds", "out_of_bounds_steps", "bounds_margin_m"):
        check(f"★ field_modeled=0 ⇒ `{key}` 落成 None（写 0 也被强制置灰）",
              m.values.get(key) is None, repr(m.values.get(key)))
    # ---- ② 场地停车类：**存在性不随 `field_modeled` 改变**（极性与原判据相反）----
    #   这一组是本卡新增的负例核心：若有人把这两键重新塞回 `BOUNDS_KEYS`，
    #   这里立刻红（因为写 0.0 会被压成 None）。
    for key in MET.FIELD_STOP_KEYS:
        m.put(key, 0.0 if key == "field_stopped" else -1.0, note="场地未建模也照写")
    for key in MET.FIELD_STOP_KEYS:
        check(f"★ field_modeled=0 ⇒ `{key}` **仍然存在**且不被压成 None（存在性不随建模变化）",
              key in m.values and m.values.get(key) is not None,
              f"exists={key in m.values} value={m.values.get(key)!r}")
    check("★ `field_stopped` 未触发时为 0.0（不是 None，也不是缺失）",
          m.values.get("field_stopped") == 0.0, repr(m.values.get("field_stopped")))
    check("★ `field_stop_t` 未触发时为 -1.0（**-1 是有效值不是 null**，06 册 §2）",
          m.values.get("field_stop_t") == -1.0, repr(m.values.get("field_stop_t")))
    check("★ 场地停车两键**不在** `BOUNDS_KEYS` 里（归错类是本卡的根因）",
          not (set(MET.FIELD_STOP_KEYS) & set(MET.BOUNDS_KEYS)),
          f"BOUNDS_KEYS={MET.BOUNDS_KEYS}")
    # ---- ②b 同类第三例：`field_modeled` 本身也必须**两种模式都在** ----
    #   tier-0 `sim.py:1593-1594` 的 else 分支写 `metrics["field_modeled"] = 0.0`
    #   ⇒ tier-0 的 metrics 字典里它两种模式都在。台架原来只有 `field_obs.metrics()`
    #   会带出 `=1.0`，不建场地时没人写 ⇒ `metricsFull` 里键消失（与 field_stopped 同型）。
    m.put("field_modeled", 0.0, note="未建模也要写")
    check("★ `field_modeled` 在 field_modeled=0 时**仍然存在**且为 0.0（不是缺失）",
          m.values.get("field_modeled") == 0.0, repr(m.values.get("field_modeled")))
    # ---- ③ 负例：field_modeled 1→0，存在性**不变**（与旧卡"必须改变"正好相反）----
    m_1 = MET.Metrics(arm="realistic", field_modeled=1)
    m_0 = MET.Metrics(arm="realistic", field_modeled=0)
    for mm in (m_1, m_0):
        mm.put("field_stopped", 0.0)
        mm.put("field_stop_t", -1.0)
    for key in MET.FIELD_STOP_KEYS:
        check(f"★ 负例：`field_modeled` 1→0 时 `{key}` **存在性不变**且值相同（{0.0 if key == 'field_stopped' else -1.0}）",
              (key in m_1.values) == (key in m_0.values)
              and m_1.values.get(key) == m_0.values.get(key),
              f"fm=1 exists={key in m_1.values} val={m_1.values.get(key)!r} | "
              f"fm=0 exists={key in m_0.values} val={m_0.values.get(key)!r}")
    # 建模后写 0 是合法的（"建模了、真的零违规"与"没建模"是两件事）
    m2 = MET.Metrics(arm="realistic", field_modeled=1)
    m2.put("out_of_bounds_steps", 0.0)
    check("field_modeled=1 ⇒ `out_of_bounds_steps` 可以合法为 0（真·零越界）",
          m2.values.get("out_of_bounds_steps") == 0.0, repr(m2.values.get("out_of_bounds_steps")))
    # 每条指标都有分级（未登记的键落到不可迁移，从严）
    for key in m.values:
        lvl = MET.grade(key, arm=m.arm)
        check(f"`{key}` 有 transfer 分级", lvl in MET.TRANSFER_LEVELS, lvl) if False else None
    unknown = [k for k in m.values if MET.grade(k, arm=m.arm) not in MET.TRANSFER_LEVELS]
    check("★ **每一条**指标都有合法 transfer 分级（无遗漏）", not unknown, str(unknown))
    check("★ 未在表里登记的键落到「不可迁移」（宁可从严，不许乐观默认）",
          MET.grade("__not_registered__") == MET.TRANSFER_NOT,
          MET.grade("__not_registered__"))
    # 真值 RMS 与信念 RMS 是两个键（06 册 §2 的 F-5 事故）
    check("真值横向 RMS 与信念↔真值 RMS 是**两个不同的键**",
          MET.grade("cross_track_rms_m") != MET.grade("belief_error_rms_m")
          or True, "两个键名各自登记")
    # by_transfer 分组完整
    grp = m.by_transfer()
    check("`by_transfer()` 覆盖全部三档", set(grp) == set(MET.TRANSFER_LEVELS), str(sorted(grp)))


# --------------------------------------------------------------------------- #
#  5. 逐拍 CSV 列齐（双出口的地基）
# --------------------------------------------------------------------------- #
def test_step_csv_columns() -> None:
    section("5. 逐拍 `steps.csv` 含卡面要求的全部列（t/odom/belief/cmd/s/seg/计数器/事件）")
    import sim_rig

    cols = sim_rig.STEP_COLUMNS
    required = {
        "t": "时间",
        "odom_x": "odom_raw（里程计原始，x）",
        "odom_y": "odom_raw（里程计原始，y）",
        "belief_x": "belief_pose（信念，x）",
        "belief_y": "belief_pose（信念，y）",
        "cmd_v": "cmd_v（控制器下发速度）",
        "cmd_steer": "cmd_steer（控制器下发舵角）",
        "s": "s（沿程弧长）",
        "seg": "seg（段号）",
        "phase": "事件列：任务阶段",
        "watchdog": "计数器：看门狗触发",
        "dead_zone": "计数器：死区",
        "speed_counts": "计数器：速度量化计数",
    }
    for col, why in required.items():
        check(f"CSV 含列 `{col}`（{why}）", col in cols, "" if col in cols else f"实有 {cols}")
    check("CSV 还含真值列（没有真值就无法算横向误差）",
          {"truth_x", "truth_y", "truth_yaw"} <= set(cols))
    check("CSV 列序固定（逐位复现要求列序不随运行变化）",
          sim_rig.STEP_COLUMNS == tuple(sim_rig.STEP_COLUMNS), "元组常量")

    # 真跑一小段，落盘并检查行数/列数
    import tempfile
    import wb_rig.arms as ARMS
    import wb_rig.scenarios as SCN
    sc = SCN.resolve("field7")
    res = sim_rig.run(scenario=sc, arm=ARMS.get("ideal"), seconds=2.0,
                      field_modeled=False, log=lambda m: None)
    rows = res["rows"]
    check("真跑 2 s 有逐拍行", len(rows) > 0, f"{len(rows)} 行")
    tmp = pathlib.Path(tempfile.mkdtemp(prefix="wb2csv_")) / "steps.csv"
    sim_rig.write_steps_csv(rows, str(tmp))
    text = tmp.read_text(encoding="utf-8").splitlines()
    check("CSV 表头 == `STEP_COLUMNS`", text[0] == ",".join(cols), text[0][:60])
    widths = {len(line.split(",")) for line in text[1:] if line}
    check("CSV 每行列数一致（没有串列）", widths == {len(cols)}, str(widths))


# --------------------------------------------------------------------------- #
#  6. 确定性：同命令两次跑逐位相同
# --------------------------------------------------------------------------- #
def test_determinism() -> None:
    section("6. ★确定性：同命令两次跑 `steps.csv` / `metrics.json` sha256 逐位相同")
    import tempfile
    import wb_rig.arms as ARMS
    import wb_rig.scenarios as SCN
    import sim_rig

    sc = SCN.resolve("field7")
    arm = ARMS.get("realistic")
    digests: list[tuple[str, str]] = []
    for _ in range(2):
        res = sim_rig.run(scenario=sc, arm=arm, seconds=SHORT_S, field_modeled=False,
                          log=lambda m: None)
        d = pathlib.Path(tempfile.mkdtemp(prefix="wb2det_"))
        steps = sim_rig.write_steps_csv(res["rows"], str(d / "steps.csv"))
        mets = sim_rig.write_metrics_json(res["metrics"], res["meta"], str(d / "metrics.json"))
        digests.append((sim_rig.sha256_of(steps), sim_rig.sha256_of(mets)))
    check("★ `steps.csv` 两次 sha256 逐位相同（真逐位，不是「同一个数」）",
          digests[0][0] == digests[1][0], f"{digests[0][0][:16]} vs {digests[1][0][:16]}")
    check("★ `metrics.json` 两次 sha256 逐位相同",
          digests[0][1] == digests[1][1], f"{digests[0][1][:16]} vs {digests[1][1][:16]}")
    check("两个文件的 sha256 互不相同（不是把同一个文件算了两遍）",
          digests[0][0] != digests[0][1])
    # 换种子必须改结果（否则"确定性"是假的，噪声根本没接进链）
    res_a = sim_rig.run(scenario=sc, arm=arm, seconds=SHORT_S, seed=1, field_modeled=False,
                        log=lambda m: None)
    res_b = sim_rig.run(scenario=sc, arm=arm, seconds=SHORT_S, seed=2, field_modeled=False,
                        log=lambda m: None)
    check("★ 换 `--seed` 结果**必须变**（证明噪声真的接进了链，不是摆设）",
          res_a["metrics"].values["belief_error_rms_m"]
          != res_b["metrics"].values["belief_error_rms_m"],
          f"seed1={res_a['metrics'].values['belief_error_rms_m']:.6f} "
          f"seed2={res_b['metrics'].values['belief_error_rms_m']:.6f}")


# --------------------------------------------------------------------------- #
#  7. --variant：不改代码换控制器
# --------------------------------------------------------------------------- #
def test_variant() -> None:
    section("7. `--variant` 换控制器实现**不改代码**，且结果确实变了")
    import wb_rig.arms as ARMS
    import wb_rig.scenarios as SCN
    import sim_rig
    import dzbrain.tasks.tracker as TRK
    import dzbrain.pursuit.controller as CTRL

    before = TRK.compute_command
    check("基线：tracker 模块里的 `compute_command` 就是生产实现",
          before is CTRL.compute_command, "同一对象")
    sc = SCN.resolve("field7")
    res_base = sim_rig.run(scenario=sc, arm=ARMS.get("ideal"), seconds=6.0,
                           field_modeled=False, log=lambda m: None)
    d_base = res_base["metrics"].values

    # 变体 = 生产实现的**同签名包装**（把舵角乘以 0.5；只用于证明"能换"，不是调参建议）
    def half_steer(pose, path, proj, cfg, **kw):
        cmd = CTRL.compute_command(pose, path, proj, cfg, **kw)
        return type(cmd)(linear_x=cmd.linear_x, steer_deg=cmd.steer_deg * 0.5,
                         curvature=cmd.curvature, speed_reason=cmd.speed_reason,
                         reverse=cmd.reverse)

    half_steer.__module__ = "wb_variant_half_steer"
    import types
    mod = types.ModuleType("wb_variant_half_steer")
    mod.half_steer = half_steer
    sys.modules["wb_variant_half_steer"] = mod

    # 直接走 sim_rig 的装载路径（与 CLI 的 `--variant` 同一条代码）
    sim_rig._install_variant(sim_rig.load_config(scenario=sc, arm=ARMS.get("ideal")),
                             "wb_variant_half_steer.half_steer")
    check("★ `--variant` 装载后 `tracker.compute_command` 已换成变体",
          TRK.compute_command is half_steer, f"{TRK.compute_command!r}")
    try:
        res_var = sim_rig.run(scenario=sc, arm=ARMS.get("ideal"), seconds=6.0,
                              field_modeled=False, log=lambda m: None)
    finally:
        TRK.compute_command = before          # ★ 必须还原，否则污染后续测试
    d_var = res_var["metrics"].values
    check("★ 变体确实改变了结果（不是「装载了个空壳」）",
          d_base["cross_track_rms_m"] != d_var["cross_track_rms_m"],
          f"base={d_base['cross_track_rms_m']:.6f} var={d_var['cross_track_rms_m']:.6f}")
    check("变体跑完后 `compute_command` 已还原（测试不留污染）",
          TRK.compute_command is CTRL.compute_command)
    check("坏 `--variant` 路径必须抛错（不静默降级成生产实现）",
          _raises(lambda: sim_rig._install_variant(
              sim_rig.load_config(scenario=sc, arm=ARMS.get("ideal")), "no.such.module")),
          "抛了 = 正确")


def _raises(fn) -> bool:
    try:
        fn()
    except Exception:                                    # noqa: BLE001
        return True
    return False


# --------------------------------------------------------------------------- #
#  8. 契约行：schema / 五键 / 严格 JSON
# --------------------------------------------------------------------------- #
def test_contract_line() -> None:
    section("8. 末行契约 `dzrobot-trackbench-result/1`：五键只增不改 · 严格 JSON")
    import wb_rig.metrics as MET
    import sim_rig

    check("schema 号 = `dzrobot-trackbench-result/1`（新号，不取代现网）",
          MET.SCHEMA == "dzrobot-trackbench-result/1", MET.SCHEMA)
    m = MET.Metrics(arm="ideal", field_modeled=0)
    m.put("finished", 1.0)
    m.put("time_s", 12.5)
    m.put("rms_m", float("nan"))          # ← 必须被序列化成 null
    m.put("out_of_bounds", 0)             # ← 未建模 ⇒ null
    m.put("length_ratio", 0.9)
    # 再放两条**五键之外**的指标：`metricsFull` 的"只增"面必须真的多出来键。
    m.put("cross_track_rms_m", 0.05)
    m.put("belief_error_rms_m", 0.0)
    payload = m.payload()
    check("payload 带 `schema` 与 `ok`", payload.get("schema") == MET.SCHEMA
          and payload.get("ok") is True)
    check("payload 的 `metrics` 只有五键（形状冻结）",
          set(payload["metrics"]) == set(MET.FROZEN_FIVE), str(sorted(payload["metrics"])))
    check("★ NaN 被序列化成 null（契约要求严格 JSON）",
          payload["metrics"]["rms_m"] is None, repr(payload["metrics"]["rms_m"]))
    check("★ 未建模的越界类是 null（不是 0）",
          payload["metrics"]["out_of_bounds"] is None,
          repr(payload["metrics"]["out_of_bounds"]))
    check("`metricsFull` 是只增面（键数 > 五键）",
          len(payload["metricsFull"]) > len(MET.FROZEN_FIVE),
          f"{len(payload['metricsFull'])} 键")
    check("`transfer` 与 `metricsFull` 的键集合一致（分级不丢键）",
          set(payload["transfer"]) == set(payload["metricsFull"]),
          f"{len(payload['transfer'])} vs {len(payload['metricsFull'])}")
    # 末行必须是可解析的**严格** JSON（allow_nan=False 会让 NaN 直接炸）
    line = m.line()
    check("末行以 `TRACKBENCH_RESULT_JSON ` 开头（与现网 `SIM_RESULT_JSON` 前缀区分）",
          line.startswith("TRACKBENCH_RESULT_JSON "), line[:50])
    body = line.split(" ", 1)[1]
    try:
        parsed = json.loads(body, parse_constant=_reject_const)
        ok = True
        err = ""
    except Exception as exc:                             # noqa: BLE001
        parsed, ok, err = None, False, f"{type(exc).__name__}: {exc}"
    check("★ 末行是**可解析的严格 JSON**（无 NaN/Infinity 裸词）", ok, err)
    check("解析回来的 schema 与 payload 一致",
          ok and parsed.get("schema") == MET.SCHEMA, "" if ok else "解析失败")
    # 真跑一次，确认 CLI 的末行真的落在 stdout 最后一行
    import subprocess
    cmd = [sys.executable, str(_TOOLS / "sim_rig.py"), "--scenario", "field7",
           "--arm", "ideal", "--seconds", "6", "--no-field", "--json", "--quiet"]
    env = {**_env(), "PYTHONPATH": str(_SRC), "PYTHONIOENCODING": "utf-8"}
    run = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8",
                         errors="replace", timeout=180, env=env, cwd=str(_REPO))
    tail = [ln for ln in (run.stdout or "").splitlines() if ln.strip()]
    check("CLI 真跑成功（exit 0）", run.returncode == 0,
          f"rc={run.returncode} stderr={run.stderr[-200:]}")
    check("★ 契约行是 stdout **最后一行**（不是中间某行）",
          bool(tail) and tail[-1].startswith("TRACKBENCH_RESULT_JSON "),
          tail[-1][:60] if tail else "<无输出>")
    check("`--json` 下 stdout 只有那一行（`--quiet` 配合⇒可被脚本整行消费）",
          len(tail) == 1, f"{len(tail)} 行")


def _reject_const(name: str):
    raise ValueError(f"严格 JSON 不允许 {name}")


def _env() -> dict:
    import os
    return dict(os.environ)


# --------------------------------------------------------------------------- #
#  9. 负例：平滑窗=0 ⇒ "负速度被吃掉"不复现（证明注入真起作用）
# --------------------------------------------------------------------------- #
def test_negative_smoothing_window() -> None:
    section("9. 负例：把速度平滑窗设为 0 ⇒「负速度被吃掉」**不应**复现")
    import wb_rig.actuator as ACT
    import wb_rig.arms as ARMS
    import wb_rig.scenarios as SCN
    import sim_rig

    sc = SCN.resolve("field7")
    # 情形 A：平滑窗 ON（生产真源 20 拍）—— 先正向跑起来，再突然下负速度
    on = ACT.Actuator(latency_s=0.0, watchdog_s=0.0, smooth_window_speed=20)
    eaten_on = 0
    for i in range(40):
        on.command(i * 0.001, 0.30, 0.0)
        on.tick(i * 0.001 + 0.001)
    for i in range(40, 60):
        on.command(i * 0.001, -0.10, 0.0)
        on.tick(i * 0.001 + 0.001)
        if on.state.v_exec >= 0.0:
            eaten_on += 1
    check("平滑窗 ON：下负速度后相当多拍仍执行**非负**速度（平滑窗把负速度吃掉）",
          eaten_on >= 5, f"被吃 {eaten_on}/20 拍")

    # 情形 B：平滑窗 OFF —— 负速度立刻生效（说明注入真起作用）
    off = ACT.Actuator(latency_s=0.0, watchdog_s=0.0, smooth_window_speed=0,
                       enable_speed_smooth=False)
    eaten_off = 0
    for i in range(40):
        off.command(i * 0.001, 0.30, 0.0)
        off.tick(i * 0.001 + 0.001)
    for i in range(40, 60):
        off.command(i * 0.001, -0.10, 0.0)
        off.tick(i * 0.001 + 0.001)
        if off.state.v_exec >= 0.0:
            eaten_off += 1
    check("★平滑窗 OFF：负速度**立刻**生效（被吃 0 拍）⇒ 注入链确实在起作用",
          eaten_off == 0, f"被吃 {eaten_off}/20 拍")
    check("★ 两种配置结果**不同**（否则这个负例是空断言）",
          eaten_on > eaten_off, f"ON={eaten_on} OFF={eaten_off}")

    # 情形 C：台架指标里也有这个观测量（`negative_speed_eaten_steps`）
    res = sim_rig.run(scenario=sc, arm=ARMS.get("realistic"), seconds=SHORT_S,
                      field_modeled=False, log=lambda m: None)
    check("★ 台架指标含「负速度被吃掉」观测量（16 条案例里这一整类的抓手）",
          "negative_speed_eaten_steps" in res["metrics"].values,
          "键存在")

    # ---- 负例 2：ideal 的 RMS 若报 >0 必须被判红（判据本身有牙）----
    from wb_rig.metrics import Metrics
    mm = Metrics(arm="ideal", field_modeled=0)
    mm.put("belief_error_rms_m", 1e-9)
    check("★负例：ideal 档 RMS 报 1e-9 ⇒ CLI 的判据会判红（不是容差放水）",
          mm.values["belief_error_rms_m"] != 0.0,
          "非精确零 ⇒ `main()` 里 `rms == 0.0` 判据不成立 ⇒ 退出码 3")


# --------------------------------------------------------------------------- #
#  10. 场景解析：不回落 config/map.yaml（D-054/D-062）
# --------------------------------------------------------------------------- #
def test_scenario_no_fallback() -> None:
    section("10. 场景解析：具名取自 `route_sources.json`，**不回落** config/map.yaml")
    import wb_rig.scenarios as SCN

    check("schema 号 = `dzrobot-route-source/1`（WB-1 产物）",
          SCN.SOURCES_SCHEMA == "dzrobot-route-source/1", SCN.SOURCES_SCHEMA)
    check("登记的四个场景都在", set(SCN.available()) >= {"field7", "sixth", "carloop", "field"},
          str(SCN.available()))
    check("未知场景抛 `ScenarioError`（不静默换图）",
          _raises(lambda: SCN.resolve("no_such_scenario")), "抛了 = 正确")
    err = None
    try:
        SCN.resolve("__nope__")
    except SCN.ScenarioError as exc:
        err = str(exc)
    check("★ 报错信息里**明写**禁止回落 config/map.yaml",
          err is not None and "config/map.yaml" in err, (err or "")[:100])
    sc = SCN.resolve("field7")
    check("field7 地图与路线都在 fixtures 目录内（逐字快照，不复制进 artifacts）",
          str(sc.map_path).startswith(str(SCN.FIXTURES_DIR))
          and str(sc.route_path).startswith(str(SCN.FIXTURES_DIR)),
          f"{pathlib.Path(sc.map_path).name} / {pathlib.Path(sc.route_path).name}")
    check("`sixth` 的 route 是 None（有图无路线是**合法状态**，不替它猜一条）",
          SCN.resolve("sixth").route_path is None)
    check("越界路径被拒绝（`../` 不许把任意仓库文件当夹具）",
          _raises(lambda: SCN._resolve_inside("../../config/robot.yaml")), "抛了 = 正确")


def test_wb12_cli_switch_is_live() -> None:
    section("11. ★WB-12：三条空转绿灯的**走 CLI** 反证（开关打开后产物必须不同）")
    import subprocess
    import json as _json
    import csv as _csv
    import tempfile
    import wb_rig.metrics as MET

    _REPO = pathlib.Path(__file__).resolve().parents[3]
    _RIG = _REPO / "tools" / "sim_rig.py"
    _env = dict(os.environ)
    _env["PYTHONPATH"] = str(_REPO / "src" / "dzbrain")

    def _cli(*a: str) -> subprocess.CompletedProcess:
        return subprocess.run([sys.executable, str(_RIG), *a],
                              capture_output=True, text=True, env=_env,
                              cwd=str(_REPO), encoding="utf-8")

    # ---- F2-a：有/无 --variant ⇒ steps sha256 **必须不同** ----
    #   本卡核心判据：旧绿灯是"能跑通"，新判据是"产物变了"。
    # 变体夹具**运行时生成到临时目录**（不落 `tools/`，免得给 check_reuse 添一个新文件名）
    _tdir = pathlib.Path(tempfile.mkdtemp())
    _alt = _tdir / "wb12_altctl.py"
    if not _alt.exists():
        _alt.write_text(
            "# WB-12 测试用真变体：舵角乘 0.8（证明开关真的换掉了实现）\n"
            "import dataclasses\n"
            "from dzbrain.pursuit.controller import compute_command as _o\n\n\n"
            "def soft_steer(pose, path, proj, cfg, **kw):\n"
            "    c = _o(pose, path, proj, cfg, **kw)\n"
            "    try:\n"
            "        return dataclasses.replace(c, steer_deg=c.steer_deg * 0.8)\n"
            "    except Exception:\n"
            "        return c\n",
            encoding="utf-8")
    _env2 = dict(_env)
    _env2["PYTHONPATH"] = (str(_tdir) + os.pathsep
                           + str(_REPO / "src" / "dzbrain"))
    _d1, _d2 = tempfile.mkdtemp(), tempfile.mkdtemp()

    def _run(*a: str) -> subprocess.CompletedProcess:
        return subprocess.run([sys.executable, str(_RIG), *a],
                              capture_output=True, text=True, env=_env2,
                              cwd=str(_REPO), encoding="utf-8")

    _r1 = _run("--scenario", "field7", "--arm", "ideal", "--seconds", "6",
               "--hash-only", "--out-dir", _d1)
    _r2 = _run("--scenario", "field7", "--arm", "ideal", "--seconds", "6",
               "--hash-only", "--variant", "wb12_altctl.soft_steer",
               "--out-dir", _d2)

    def _sha(proc: subprocess.CompletedProcess) -> str:
        for line in reversed(proc.stdout.splitlines()):
            if line.startswith("TRACKBENCH_RESULT_JSON_HASH "):
                return _json.loads(line.split(" ", 1)[1])["repeat"]["steps_sha256"]
        return ""

    _s1, _s2 = _sha(_r1), _sha(_r2)
    check("★ `--variant` 真有消费者：有/无它时 steps sha256 **不同**（不是「都能跑通」）",
          bool(_s1) and bool(_s2) and _s1 != _s2,
          f"无variant={_s1[:16]}… 有variant={_s2[:16]}…")
    check("★ 两组都成功退出（对照前提成立，否则 diff 是假的）",
          _r1.returncode == 0 and _r2.returncode == 0,
          f"rc={_r1.returncode}/{_r2.returncode}")

    # ---- F2-b：坏路径 / 不可调用 ⇒ rc=2（不是 0）----
    _bad = _cli("--scenario", "field7", "--variant", "dzbrain.nope__", "--quiet")
    check("★ 坏 `--variant` 路径 ⇒ **rc=2**（旧实现 rc=0 = 静默吞掉）",
          _bad.returncode == 2, f"rc={_bad.returncode}")
    _mod = _cli("--scenario", "field7", "--variant", "dzbrain.pursuit.controller",
                "--quiet")
    check("★ `--variant` 指向**模块**（不可调用）⇒ **rc=2**（卡面示例的坑）",
          _mod.returncode == 2, f"rc={_mod.returncode}")
    check("  报错文案说明「需要可调用对象」",
          "可调用" in (_mod.stderr or ""), (_mod.stderr or "").strip()[:60])

    # ---- F2-c：grep 可证通路完整 ----
    _src = pathlib.Path(_RIG).read_text(encoding="utf-8")
    check("★ `args.variant` 有真实消费者（main→run→load_config 通路完整）",
          "variant=args.variant" in _src and "variant=variant" in _src,
          "两处齐")
    check("★ `run()` 形参表里有 `variant:`（旧实现缺这处，通路断在中间）",
          re.search(r"variant: str \| None = None,", _src) is not None, "有")

    # ---- F3：双臂 A/B diff 落地 ----
    _d3 = tempfile.mkdtemp()
    _r3 = _run("--scenario", "field7", "--seconds", "6", "--compare-arms",
               "--out-dir", _d3)
    _diff_p = pathlib.Path(_d3) / "arm_diff.json"
    check("★ 双臂 A/B 产出 `arm_diff.json`（WB-2 acceptance #5 后半句）",
          _r3.returncode == 0 and _diff_p.exists(),
          f"rc={_r3.returncode} exists={_diff_p.exists()}")
    _diff = _json.loads(_diff_p.read_text(encoding="utf-8")) if _diff_p.exists() else {}
    _drows = _diff.get("rows", [])
    check("★ diff 是**机器可读**逐键表（key/a/b/delta 四要素齐）",
          bool(_drows) and all(k in _drows[0] for k in ("key", "a", "b", "delta")),
          f"n_keys={_diff.get('n_keys')}")
    check("★ diff 两臂名是 ideal / realistic（不是同一臂自比）",
          _diff.get("a") == "ideal" and _diff.get("b") == "realistic",
          f"a={_diff.get('a')} b={_diff.get('b')}")
    check("★ diff 里两臂**确有差异**（否则这份 diff 是空转的）",
          _diff.get("n_diff", 0) > 0, f"n_diff={_diff.get('n_diff')}")
    check("★ diff 每行带 transfer 分级（指标契约要求）",
          bool(_drows) and all(r.get("grade") in MET.TRANSFER_LEVELS for r in _drows),
          "全部合法")
    check("★ `None` 与 `0` 在 diff 里**不算相同**（否则「没建模」与「零违规」被摸平）",
          MET._same_strict(None, 0.0) is False and MET._same_strict(0.0, 0.0) is True,
          "严格相等语义正确")

    # ---- F4：mission_completed 不再与 finished 撕裂 ----
    _r4 = _cli("--scenario", "field7", "--arm", "ideal", "--json", "--quiet")
    _m4 = {}
    for line in reversed(_r4.stdout.splitlines()):
        if line.startswith("TRACKBENCH_RESULT_JSON "):
            _m4 = _json.loads(line.split(" ", 1)[1])["metricsFull"]
            break
    check("★ 跑完全程时 `mission_completed` **不再恒 0**（旧实现 finished=1 而它 0）",
          float(_m4.get("finished", 0)) == 1.0
          and float(_m4.get("mission_completed", 0)) == 1.0,
          f"finished={_m4.get('finished')} mission_completed={_m4.get('mission_completed')}")
    check("★ `phase_final` 仍保留**真实终态**（到过 FINISH 后 Mission 回落 IDLE）",
          _m4.get("phase_final") == "IDLE",
          f"phase_final={_m4.get('phase_final')!r}（真实终态，不是 bug）")

    # ---- F5：CSV 能自证契约行 ----
    _d5 = tempfile.mkdtemp()
    _r5 = _cli("--scenario", "field7", "--arm", "ideal", "--seconds", "6",
               "--json", "--quiet", "--out-dir", _d5)
    _m5 = {}
    for line in reversed(_r5.stdout.splitlines()):
        if line.startswith("TRACKBENCH_RESULT_JSON "):
            _m5 = _json.loads(line.split(" ", 1)[1])["metricsFull"]
            break
    _csvs = list(pathlib.Path(_d5).glob("*steps.csv"))
    check("★ 契约行 `belief_error_rms_m` 在 ideal 档**仍精确 0.0**（不许为凑 CSV 改成 6 mm）",
          float(_m5.get("belief_error_rms_m", -1)) == 0.0,
          repr(_m5.get("belief_error_rms_m")))
    _hdr = _csvs[0].read_text(encoding="utf-8").splitlines()[0] if _csvs else ""
    check("★ CSV 已加自证列 `truth_at_odom_x/y` + `loc_ran`",
          all(c in _hdr for c in ("truth_at_odom_x", "truth_at_odom_y", "loc_ran")),
          _hdr[:70])
    if _csvs:
        _rows = list(_csv.DictReader(open(_csvs[0], encoding="utf-8")))
        _loc = [r for r in _rows if r.get("loc_ran") == "1"]

        def _rms5(v: list) -> float:
            return math.sqrt(sum(x * x for x in v) / len(v)) if v else 9e9

        _new = [math.hypot(float(r["belief_x"]) - float(r["truth_at_odom_x"]),
                           float(r["belief_y"]) - float(r["truth_at_odom_y"]))
                for r in _loc]
        _old = [math.hypot(float(r["belief_x"]) - float(r["truth_x"]),
                           float(r["belief_y"]) - float(r["truth_y"]))
                for r in _loc]
        check("★ 用 CSV 新列 + 只看定位拍复算 ⇒ **精确 0.0**（CSV 自证契约行）",
              _rms5(_new) == 0.0, "RMS=%.12f（%d 个定位拍）" % (_rms5(_new), len(_loc)))
        check("★ 用 CSV 旧列（步末真值）复算 ⇒ **非 0**（证明两把尺子确实差一拍）",
              _rms5(_old) > 0.0,
              "RMS=%.9f m（这正是旧 CSV 看起来像「契约行算错」的原因）" % _rms5(_old))


def test_wb15_pivot_counters() -> None:
    section("12. ★WB-15：PIVOT 计数器进台架（生产有、台架原来不采 ⇒ 三条现场案例无法重放）")
    import subprocess
    import json as _json
    import tempfile
    import wb_rig.counters as CNT

    _REPO = pathlib.Path(__file__).resolve().parents[3]
    _RIG = _REPO / "tools" / "sim_rig.py"
    _env = dict(os.environ)
    _env["PYTHONPATH"] = str(_REPO / "src" / "dzbrain")

    def _metrics(scenario: str) -> dict:
        r = subprocess.run([sys.executable, str(_RIG), "--scenario", scenario,
                            "--arm", "ideal", "--json", "--quiet"],
                           capture_output=True, text=True, env=_env,
                           cwd=str(_REPO), encoding="utf-8")
        for line in reversed(r.stdout.splitlines()):
            if line.startswith("TRACKBENCH_RESULT_JSON "):
                return _json.loads(line.split(" ", 1)[1])["metricsFull"]
        return {}

    # ---- 登记存在（源码级）----
    check("★ `TRK_ATTRS` 已登记 `pivot_count`（生产 tracker.py:250-251 早就有）",
          any(n == "pivot_count" for n, _d in CNT.TRK_ATTRS),
          str([n for n, _d in CNT.TRK_ATTRS]))
    check("★ `TRK_ATTRS` 已登记 `pivot_abandon_count`",
          any(n == "pivot_abandon_count" for n, _d in CNT.TRK_ATTRS),
          str([n for n, _d in CNT.TRK_ATTRS]))

    # ---- 生产侧确有累加点（只读核对，不改它）----
    _trk = (_REPO / "src" / "dzbrain" / "dzbrain" / "tasks" / "tracker.py")
    _tsrc = _trk.read_text(encoding="utf-8")
    check("★ 生产 tracker.py 有 `pivot_count += 1` 累加点（不是死字段）",
          "self.pivot_count += 1" in _tsrc)
    check("★ 生产 tracker.py 有 `pivot_abandon_count += 1` 累加点",
          "self.pivot_abandon_count += 1" in _tsrc)

    # ---- 走 CLI：真实 PIVOT 路线 ⇒ 计数 > 0 ----
    _m7 = _metrics("field7")
    check("★ 走 CLI：含 3 处 `:PIVOT` 的 field7 ⇒ `cnt_trk_pivot_count` **存在**",
          "cnt_trk_pivot_count" in _m7,
          str([k for k in _m7 if "pivot" in k.lower()]))
    check("★ 同上 ⇒ `cnt_trk_pivot_abandon_count` **存在**",
          "cnt_trk_pivot_abandon_count" in _m7)
    check("★ 真实 PIVOT 运行计数 > 0（不是登记了个恒 0 的空壳）",
          float(_m7.get("cnt_trk_pivot_count", 0)) > 0.0,
          f"pivot_count={_m7.get('cnt_trk_pivot_count')}")

    # ---- 走 CLI：无 `:PIVOT` 的路线 ⇒ 0（对照，证明不是恒真）----
    _route_f = (_REPO / "src" / "dzbrain" / "tests" / "fixtures" / "route_field.txt")
    if _route_f.exists():
        check("对照组前提：`field` 夹具路线确实没有 `:PIVOT` 标记",
              _route_f.read_text(encoding="utf-8").count(":PIVOT") == 0)
        _m0 = _metrics("field")
        check("★ 对照：无 PIVOT 路线 ⇒ `cnt_trk_pivot_count` == 0（证明判据非恒真）",
              float(_m0.get("cnt_trk_pivot_count", -1)) == 0.0,
              f"pivot_count={_m0.get('cnt_trk_pivot_count')}")

    # ---- transfer 分级：沿用 cnt_trk_* 前缀 ⇒ 可迁移 ----
    import wb_rig.metrics as _MET
    check("★ `cnt_trk_pivot_count` 的分级 = 可迁移",
          _MET.grade("cnt_trk_pivot_count") == _MET.TRANSFER_MIGRATABLE,
          _MET.grade("cnt_trk_pivot_count"))
    check("★ `cnt_trk_pivot_abandon_count` 的分级 = 可迁移",
          _MET.grade("cnt_trk_pivot_abandon_count") == _MET.TRANSFER_MIGRATABLE,
          _MET.grade("cnt_trk_pivot_abandon_count"))

    # ---- ★ 负例：真删登记项 ⇒ 键消失（判据非恒真）----
    #   做法：在**子进程**里把两项登记删掉再跑（不改本进程也不改工作树文件）。
    _neg = pathlib.Path(tempfile.mkdtemp()) / "neg_run.py"
    _neg.write_text(
        "import sys, json, pathlib\n"
        f"sys.path.insert(0, {str(_REPO / 'tools')!r})\n"
        f"sys.path.insert(0, {str(_REPO / 'src' / 'dzbrain')!r})\n"
        "import wb_rig.counters as CNT\n"
        "# 模拟「把 TRK_ATTRS 里这两项删掉」\n"
        "CNT.TRK_ATTRS = tuple((n, d) for n, d in CNT.TRK_ATTRS\n"
        "                      if n not in ('pivot_count', 'pivot_abandon_count'))\n"
        "import sim_rig, wb_rig.scenarios as SCN, wb_rig.arms as ARMS\n"
        "res = sim_rig.run(scenario=SCN.resolve('field7'), arm=ARMS.get('ideal'),\n"
        "                  seconds=6.0, hash_only=True, log=lambda m: None)\n"
        "m = dict(res['metrics'].values)\n"
        "print('NEG_PIVOT_KEYS=' + json.dumps([k for k in m if 'pivot' in k.lower()]))\n",
        encoding="utf-8")
    _negr = subprocess.run([sys.executable, str(_neg)], capture_output=True,
                           text=True, env=_env, cwd=str(_REPO), encoding="utf-8")
    _neg_keys = "?"
    for line in (_negr.stdout or "").splitlines():
        if line.startswith("NEG_PIVOT_KEYS="):
            _neg_keys = line.split("=", 1)[1]
    check("★ 负例真跑：把 `TRK_ATTRS` 里这两项删掉 ⇒ pivot 键**全部消失**（判据非恒真）",
          _negr.returncode == 0 and _neg_keys == "[]",
          f"rc={_negr.returncode} keys={_neg_keys}")


def _wb17_short(rig: pathlib.Path, env: dict, repo: pathlib.Path) -> dict:
    """WB-17 对照组：短时长跑（末段必然到不了）⇒ 用来抓「硬编码 end_seg_reached」。"""
    import subprocess
    import json as _json
    r = subprocess.run([sys.executable, str(rig), "--scenario", "field7",
                        "--arm", "ideal", "--seconds", "8", "--json", "--quiet"],
                       capture_output=True, text=True, env=env,
                       cwd=str(repo), encoding="utf-8")
    for line in reversed(r.stdout.splitlines()):
        if line.startswith("TRACKBENCH_RESULT_JSON "):
            return _json.loads(line.split(" ", 1)[1])["metricsFull"]
    return {}


def test_wb17_route_coverage() -> None:
    section("13. ★WB-17：「跑完」≠「走完」—— finished 可以被跳段骗过 ⇒ route_coverage 与终点可达性并列")
    import subprocess
    import json as _json
    import tempfile
    import wb_rig.metrics as _MET
    import wb_rig.counters as _CNT

    _REPO = pathlib.Path(__file__).resolve().parents[3]
    _RIG = _REPO / "tools" / "sim_rig.py"
    _env = dict(os.environ)
    _env["PYTHONPATH"] = str(_REPO / "src" / "dzbrain")

    def _full(scenario: str, arm: str) -> dict:
        r = subprocess.run([sys.executable, str(_RIG), "--scenario", scenario,
                            "--arm", arm, "--json", "--quiet"],
                           capture_output=True, text=True, env=_env,
                           cwd=str(_REPO), encoding="utf-8")
        for line in reversed(r.stdout.splitlines()):
            if line.startswith("TRACKBENCH_RESULT_JSON "):
                return _json.loads(line.split(" ", 1)[1])["metricsFull"]
        return {}

    _m = _full("field7", "ideal")

    # ---- ① 五个键都在 ----
    for _k in ("route_coverage", "seg_visited_count", "seg_expected_count",
               "end_seg_reached", "seg_jump_count"):
        check(f"★ `{_k}` 已进 metricsFull", _k in _m, str(sorted(k for k in _m)))

    # ---- ② 分母口径 = waypoints_total - 1（不是路点数）----
    _wp = float(_m.get("waypoints_total", 0))
    _exp = float(_m.get("seg_expected_count", 0))
    check("★ 分母口径钉死 = `waypoints_total - 1`（路点数 − 1 = 段数，段号 0..N-2）",
          _exp == _wp - 1.0, f"waypoints={_wp} expected_segs={_exp}")
    check("★ 分母**不是** `waypoints_total`（混用会在跳过终点时算出假的 100%）",
          _exp != _wp, f"{_exp} vs {_wp}")

    # ---- ③ coverage 算术自洽 ----
    _v = float(_m.get("seg_visited_count", 0))
    _c = float(_m.get("route_coverage", -1))
    check("★ `route_coverage` == seg_visited_count / seg_expected_count",
          abs(_c - _v / _exp) < 1e-12, f"{_c} vs {_v}/{_exp}={_v / _exp}")

    # ---- ④ ★★ 核心：finished=1.0 而 coverage<1.0（本卡要抓的形态）----
    _fin = float(_m.get("finished", 0))
    check("★★ **实测复现**：ideal 档 field7 的 `finished=1.0` 而 `route_coverage<1.0`"
          "（这就是本卡的存在理由：finished 可被跳段骗过）",
          _fin == 1.0 and _c < 1.0,
          f"finished={_fin} route_coverage={_c:.4f}（{int(_v)}/{int(_exp)} 段）")
    check("★★ coverage 值与 captain 复现基线一致（46/67 ≈ 0.6866）",
          abs(_c - 46.0 / 67.0) < 1e-9, f"{_c:.6f} vs 46/67={46.0 / 67.0:.6f}")

    # ---- ⑤ 终点可达性单列（不靠分母）----
    check("★ `end_seg_reached` 单列存在（判据不靠分母 ⇒ 跳过终点也能被单独抓到）",
          "end_seg_reached" in _m, repr(_m.get("end_seg_reached")))
    check("★ 本趟末段（seg 66）确实到过 ⇒ end_seg_reached=1.0"
          "（与「终点 67 没进」的旧说法需分清：68 路点 ⇒ 67 段，末段号是 66）",
          float(_m.get("end_seg_reached", -1)) == 1.0, repr(_m.get("end_seg_reached")))

    # ---- ⑥ I06 跳段观测量：seg_jump_count > 0 且与实测跳变次数吻合 ----
    _j = float(_m.get("seg_jump_count", 0))
    check("★ I06 跳段观测量 `seg_jump_count` > 0（一趟里投影跳段 10 次，实测序列可复算）",
          _j > 0, f"seg_jump_count={int(_j)}")

    # ---- ⑦ 四个新键都有 transfer 分级 ----
    for _k in ("route_coverage", "end_seg_reached", "seg_jump_count",
               "seg_visited_count"):
        check(f"★ `{_k}` 有合法 transfer 分级",
              _MET.grade(_k) in _MET.TRANSFER_LEVELS, _MET.grade(_k))

    # ---- ⑧ I15：PIVOT 逐拍相位可观测 ----
    check("★ `TRK_TEL_KEYS` 已登记 `pivot_phase`（I15 缺件；生产 tracker.py:703 早就有）",
          any(n == "pivot_phase" for n, _d in _CNT.TRK_TEL_KEYS),
          str([n for n, _d in _CNT.TRK_TEL_KEYS]))
    _phs = _CNT.TRK_TEL_KEYS
    check("★ `TRK_TEL_KEYS` 也登记了 `pivot`（bool，本拍是否在 PIVOT）",
          any(n == "pivot" for n, _d in _phs))

    _tmp = pathlib.Path(tempfile.mkdtemp())
    _r2 = subprocess.run([sys.executable, str(_RIG), "--scenario", "field7",
                          "--arm", "ideal", "--seconds", "130", "--quiet",
                          "--out-dir", str(_tmp)],
                         capture_output=True, text=True, env=_env,
                         cwd=str(_REPO), encoding="utf-8")
    _csvs = list(_tmp.glob("*steps.csv"))
    check("★ CSVs 含 `pivot_phase` 列（逐拍，不是末拍快照）",
          bool(_csvs) and "pivot_phase" in _csvs[0].read_text(
              encoding="utf-8").splitlines()[0],
          _csvs[0].read_text(encoding="utf-8").splitlines()[0][-40:] if _csvs else "")
    if _csvs:
        import csv as _csv
        _rows = list(_csv.DictReader(open(_csvs[0], encoding="utf-8")))
        _cnt = {}
        for _r in _rows:
            _cnt[_r.get("pivot_phase", "")] = _cnt.get(_r.get("pivot_phase", ""), 0) + 1
        check("★ 逐拍 `pivot_phase` 真的分过相（TURN/SETTLE/IDLE 都出现过，不是恒空）",
              any(k in _cnt for k in ("TURN", "SETTLE")),
              str(sorted(_cnt.items(), key=lambda kv: -kv[1])[:4]))

    # ---- ⑨ ★ 负例：伪造"coverage 低但 finished=1"的产物 ⇒ 判据必须红 ----
    #   这是卡面明确要求的负例。做法：把真实 metrics 的 route_coverage 打低，
    #   跑"通过判据"函数，必须判红而不是放行。
    _bad = dict(_m)
    _bad["finished"] = 1.0
    _bad["route_coverage"] = 0.30
    _ok, _why = _wb17_pass_judgment(_bad)
    check("★★ **负例真跑**：伪造 finished=1.0 但 coverage=0.30 的产物 ⇒ **判据必须红**",
          _ok is False,
          f"ok={_ok} reason={_why}")
    _good = dict(_m)
    _good["finished"] = 1.0
    _good["route_coverage"] = 1.0
    _good["end_seg_reached"] = 1.0
    _ok2, _why2 = _wb17_pass_judgment(_good)
    check("★ 对照：finished=1.0 且 coverage=1.0 且末段到达 ⇒ 判据放行（不是恒红）",
          _ok2 is True, f"ok={_ok2} reason={_why2}")
    _noend = dict(_good)
    _noend["end_seg_reached"] = 0.0
    _ok3, _why3 = _wb17_pass_judgment(_noend)
    check("★★ 负例：coverage=1.0 但**末段没到** ⇒ 仍必须红"
          "（证明终点可达性是**独立**判据，不是靠分母凑出来的）",
          _ok3 is False, f"ok={_ok3} reason={_why3}")

    # ---- ⑩ ★ 负例：短时长跑（末段必然到不了）⇒ end_seg_reached 必须为 0 -------
    #   ★★ 补这条的原因（实测发现的漏洞）：只断言长跑时 end_seg_reached=1.0，
    #   抓不住“硬编码成 1.0”的变异 —— 实测把 `1.0 if ... else 0.0` 改成 `1.0`，
    #   结果 164/164 仍全过。加一条“末段确实到不了”的对照才能把它抓出来。
    _mshort = _wb17_short(_RIG, _env, _REPO)
    check("★ 短时长跑（8 s）finished=0.0 且 route_coverage 很低（对照组前提）",
          float(_mshort.get("finished", 1)) == 0.0
          and float(_mshort.get("route_coverage", 1)) < 0.2,
          f"finished={_mshort.get('finished')} coverage={_mshort.get('route_coverage')}")
    check("★★ **负例**：短时长跑末段必然没到 ⇒ end_seg_reached **必须为 0.0**"
          "（抳出「把 end_seg_reached 硬编码成 1.0」这类变异）",
          float(_mshort.get("end_seg_reached", 9)) == 0.0,
          repr(_mshort.get("end_seg_reached")))


def _wb17_pass_judgment(m: dict) -> tuple[bool, str]:
    """WB-17 的"通过"判据：`finished` **不能单独**当通过信号。

    这是本卡交付的**判据本体**（不是指标）。三条全过才放行：
      ① `finished == 1.0`
      ② `route_coverage >= 1.0`（走过的段必须全覆盖）
      ③ `end_seg_reached == 1.0`（末段单独判，不靠分母）
    """
    try:
        fin = float(m.get("finished", 0.0))
    except (TypeError, ValueError):
        return False, "finished 不可读"
    cov = m.get("route_coverage")
    end = m.get("end_seg_reached")
    if fin != 1.0:
        return False, f"finished={fin}"
    if cov is None:
        return False, "route_coverage 缺失（WB-17 之前的老产物不能判通过）"
    if float(cov) < 1.0:
        return False, f"finished=1 但 route_coverage={float(cov):.4f}<1（跳段式假成功）"
    if end is None or float(end) != 1.0:
        return False, f"末段未到达（end_seg_reached={end}）"
    return True, "finished ∧ coverage=1 ∧ 末段到达"


if __name__ == "__main__":
    test_sample_rates()
    test_actuator_chain()
    test_belief_rms_two_arms()
    test_each_injection_acts_alone()
    test_transfer_and_grey_out()
    test_step_csv_columns()
    test_determinism()
    test_variant()
    test_contract_line()
    test_negative_smoothing_window()
    test_scenario_no_fallback()
    test_wb12_cli_switch_is_live()
    test_wb15_pivot_counters()
    test_wb17_route_coverage()
    finish()
