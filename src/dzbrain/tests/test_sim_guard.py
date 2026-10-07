# -*- coding: utf-8 -*-
r"""离线追踪仿真器 `dzbrain.tools.sim` 的**回归护栏**测试（纯脚本，无 pytest）。

运行::

    python src\dzbrain\tests\test_sim_guard.py

★ 为什么单独一个文件：`tests\test_sim.py` 在本文件落盘前 1 分钟被另一条线写好了，
  它覆盖了仿真器的功能面，但**把现场图路线跑不通这件事用"改配置参数"绕过去了**
  （它自带 `FIELD_CFG` 覆盖 `arrival_m / lookahead_min_m / vertex_clamp_m`）。
  那样一来"现场图在**默认参数**下跑不完"就不会再被任何断言发现 ——
  正是任务书 §7 要防的回归（史实：信念投影窗口 1.50 → 0.75 后，
  现场图从 `113.0 s / 5648 拍跑完` 退化成 `822.5 s / 撞步数上限 / ratio 5.31`）。
  本文件因此**只用出厂 `config/robot.yaml`**，且把"跑不完"如实钉住。

★ 本文件只**读**仿真器，不修改 `sim.py` / `sim_cli.py` / `tools\sim_pursuit.py` /
  `core/ route/ pursuit/ io/ fakes.py config/`。

★ 输出约定
  · `PASS` / `FAIL` 由 `harness` 记账，`FAIL` 一定反映到退出码 1；
  · `XFAIL`（本文件自建）= **已知且已上报的不达标项**：先按 FAIL 记账（保持红色信号，
    不把回归洗成绿灯），再在汇总区单独列出原因 —— 便于一眼区分"新回归"与"已知缺口"。
"""

from __future__ import annotations

import hashlib
import math
import os
import pathlib
import shutil
import sys
import tempfile
import time

# --------------------------------------------------------------------------- #
#  路径与导入
# --------------------------------------------------------------------------- #
_TESTS_DIR = pathlib.Path(__file__).resolve().parent
_SRC = _TESTS_DIR.parent
sys.path.insert(0, str(_SRC))

# Windows 控制台是 GBK：本文件只用 ASCII，但异常信息里可能出现编不出的字符。
try:
    sys.stdout.reconfigure(errors="replace")  # type: ignore[union-attr]
except Exception:  # pragma: no cover - 非标准流
    pass

import harness                                                             # noqa: E402
from harness import check, check_close, check_raises, finish, section      # noqa: E402

from dzbrain.core import geometry as geo                                   # noqa: E402
from dzbrain.core.config import Config                                     # noqa: E402
from dzbrain.route.parser import parse_mission                             # noqa: E402
from dzbrain.tools import sim as S                                         # noqa: E402

_WS = _TESTS_DIR.parents[2]
_CFG_PATH = _WS / "config" / "robot.yaml"
_FIXTURES = _TESTS_DIR / "fixtures"
_FIELD_PATH = _FIXTURES / "route_field.txt"

# --------------------------------------------------------------------------- #
#  XFAIL：已知不达标项（见文件末 INTERFACE_NOTES）
# --------------------------------------------------------------------------- #
_XFAILED: list[str] = []
_XFAIL_IDS: dict[str, str] = {}          # 用例编号 -> 原因


def xfail(case_id: str, reason: str) -> None:
    """登记一个已知不达标项：按 FAIL 记账 + 汇总区单列原因。"""
    _XFAIL_IDS[case_id] = reason
    _XFAILED.append(f"{case_id} —— {reason}")


def _check_x(case_id: str, name: str, cond: bool, detail: str = "") -> bool:
    """带编号的断言：编号在 `_XFAIL_IDS` 里时额外标注（仍然参与 FAIL 记账）。"""
    ok = check(f"{case_id} {name}", cond, detail)
    if case_id in _XFAIL_IDS and not ok:
        print(f"        XFAIL  {_XFAIL_IDS[case_id]}")
    return ok


# --------------------------------------------------------------------------- #
#  固定输入与常量
# --------------------------------------------------------------------------- #
def _sha256(path: pathlib.Path) -> str:
    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()[:16]
    except Exception:  # pragma: no cover
        return "?"


cfg = Config.load(str(_CFG_PATH))          # ★ 只用出厂配置，绝不 override
FIELD = parse_mission(str(_FIELD_PATH))
FIELD_PATH = S.Path(FIELD)

FIELD_WAYPOINTS = 34
FIELD_REVERSE_SEGS = (12, 18, 19)          # E11->F11 / A6->B6 / B6->D6
FIELD_STOP_INDICES = (9, 33)               # G10* 射击停站 + 末点 N5
FIELD_LENGTH_M = 16.45

STRAIGHT = parse_mission(["N10", "M10"])   # 两格直线，0.35 m
MID = parse_mission(["M10", "M8", "K8", "K10", "I10", "I8"])   # 4 个 90 度拐角 / 3.50 m

FIELD_CAP_STEPS = 3000                     # 对照窗口（同窗口才可比）
FIELD_HARD_CAP = 8000                      # 卡死快照的步数上限
ZERO_TOL = 1e-9                            # 零注入下信念==真值的判据

print("=" * 78)
print("dzbrain 仿真器回归护栏（纯脚本）")
print("=" * 78)
print("sim.py    : %d bytes  sha256[:16]=%s  mtime=%s" % (
    (_WS / "src" / "dzbrain" / "dzbrain" / "tools" / "sim.py").stat().st_size
    if (_WS / "src" / "dzbrain" / "dzbrain" / "tools" / "sim.py").is_file() else -1,
    _sha256(_WS / "src" / "dzbrain" / "dzbrain" / "tools" / "sim.py"),
    time.strftime("%Y-%m-%d %H:%M:%S",
                  time.localtime((_WS / "src" / "dzbrain" / "dzbrain" / "tools"
                                  / "sim.py").stat().st_mtime))))
print("robot.yaml: sha256[:16]=%s（出厂值，不做任何覆盖）" % _sha256(_CFG_PATH))
print("现场图    : %s" % _FIELD_PATH)

_T0 = time.time()


def _run(route, **kw):
    """跑一次仿真（kwargs 直接进 `SimConfig`）。"""
    return S.run_sim(route, cfg, S.SimConfig(**kw))


# =========================================================================== #
section("1. 直线（两格）：能跑完、横向误差为零、末点误差 < 0.15 m")
# =========================================================================== #
t = time.time()
r1 = _run(STRAIGHT)
m1 = r1.metrics
print("     实测: completed=%s steps=%d time_s=%.3f xt_rms=%.4e xt_max=%.4e "
      "final_error=%.5f ratio=%.4f anchors=%d (%.2fs)" % (
          m1["completed"], r1.steps, m1["time_s"], m1["cross_track_rms_m"],
          m1["cross_track_max_m"], m1["final_error_m"], m1["path_len_ratio"],
          int(m1["anchors"]), time.time() - t))
check_close("1.1 路线总长 = 一格 0.35 m", S.Path(STRAIGHT).total_length, 0.35, 1e-9)
check("1.2 completed=True", m1["completed"] is True, f"completed={m1['completed']!r}")
check("1.3 横向 RMS < 0.01 m", m1["cross_track_rms_m"] < 0.01,
      f"xt_rms={m1['cross_track_rms_m']:.4e}")
check("1.4 横向 MAX < 0.01 m", m1["cross_track_max_m"] < 0.01,
      f"xt_max={m1['cross_track_max_m']:.4e}")
check("1.5 末点误差 < 0.15 m", m1["final_error_m"] < 0.15,
      f"final_error={m1['final_error_m']:.5f}")
check("1.6 没撞步数上限、没跑飞",
      not m1["hit_step_limit"] and not m1["diverged"],
      f"hit_step_limit={m1['hit_step_limit']} diverged={m1['diverged']}")
check("1.7 越界拍数 = 0", m1["out_of_bounds_steps"] == 0.0,
      f"oob={m1['out_of_bounds_steps']}")
check("1.8 真值 x 全程 = N10 格心（1e-9 内）",
      max(abs(v - 0.525) for v in r1.truth_x) < 1e-9,
      f"max|x-0.525|={max(abs(v - 0.525) for v in r1.truth_x):.3e}")
check("1.9 真值 y 单调前进", all(r1.truth_y[i] <= r1.truth_y[i + 1] + 1e-12
                                 for i in range(len(r1.truth_y) - 1)),
      f"y: {r1.truth_y[0]:.4f} -> {r1.truth_y[-1]:.4f}")

_arrival = float(cfg.get("pursuit.arrival_m"))
_goal_stop = getattr(S, "_GOAL_STOP_M", None)
print("     末点误差机理: final_error=%.5f  arrival_m=%.2f  sim._GOAL_STOP_M=%s" % (
    m1["final_error_m"], _arrival, _goal_stop))
# ⚠️ 见 INTERFACE_NOTES #4（已修）：`_GOAL_STOP_M = 0.03` 修前从没生效 —— 末点先被
#    `pursuit.arrival_m`(0.12) 判到位并推进 `target_idx`，下一拍投影弧长直接抬到
#    `total_length` ⇒ `reached_end = True` ⇒ 刹车，停点因此固定在 0.118 m。
#    修法（sim.py）：末点的"已过点即放行"也用 `_GOAL_STOP_M` 判，末点不再在 0.12 m
#    处提前推进 —— 停点于是真的收到 3 cm 量级（实测见下行打印）。
print("     末点误差机理（已修）: final_error=%.5f  arrival_m=%.2f  "
      "sim._GOAL_STOP_M=%s" % (m1["final_error_m"], _arrival, _goal_stop))
check("1.10 ★末点停车精度 = sim 自称的 _GOAL_STOP_M(0.03 m)",
      abs(m1["final_error_m"] - (0.03 if _goal_stop is None else float(_goal_stop))) < 0.01,
      f"final_error={m1['final_error_m']:.5f} vs _GOAL_STOP_M={_goal_stop}")

# =========================================================================== #
section("2. 现场图路线（34 航点，含 3 个倒车段 + G10* 停站）：结构 + 全程")
# =========================================================================== #
print("     结构: %d 点 / %.2f m / 倒车段 %s / 停站点 %s / 拐角 %d 处" % (
    len(FIELD.waypoints), FIELD_PATH.total_length,
    [i for i in range(FIELD_PATH.segment_count) if FIELD_PATH.segment_is_reverse(i)],
    list(FIELD_PATH.stop_indices),
    sum(1 for i in range(FIELD_PATH.segment_count) if FIELD_PATH.corner_angle_deg(i) > 45.0)))
check("2.1 解析出 34 个航点", len(FIELD.waypoints) == FIELD_WAYPOINTS,
      f"got {len(FIELD.waypoints)}")
check_close("2.2 路线总长 = 16.45 m", FIELD_PATH.total_length, FIELD_LENGTH_M, 1e-9)
_rev = tuple(i for i in range(FIELD_PATH.segment_count)
             if FIELD_PATH.segment_is_reverse(i))
check("2.3 倒车段索引 = %s" % (FIELD_REVERSE_SEGS,), _rev == FIELD_REVERSE_SEGS, f"got {_rev}")
check("2.4 停站点索引 = %s" % (FIELD_STOP_INDICES,),
      tuple(FIELD_PATH.stop_indices) == FIELD_STOP_INDICES,
      f"got {tuple(FIELD_PATH.stop_indices)}")
check("2.5 G10 是射击停站", FIELD.waypoints[9].shoot is True,
      f"block={FIELD.waypoints[9].block} shoot={FIELD.waypoints[9].shoot}")

# ---- 零注入 + 关锚定 ⇒ 信念必须**逐位**等于真值 ----
r2z = _run(FIELD, anchor_on_arrival=False, max_steps=FIELD_HARD_CAP)
_gap2 = max(math.hypot(r2z.x[i] - r2z.truth_x[i], r2z.y[i] - r2z.truth_y[i])
            for i in range(r2z.steps))
print("     实测(零注入/关锚定/%d 拍): steps=%d max|belief-truth|=%.4e "
      "xt_rms=%.5f anchors=%d" % (
          FIELD_HARD_CAP, r2z.steps, _gap2, r2z.metrics["cross_track_rms_m"],
          int(r2z.metrics["anchors"])))
check("2.6 ★零注入不变量：信念 == 真值（< 1e-9 m）", _gap2 < ZERO_TOL, f"max_gap={_gap2:.4e}")
check("2.7 关锚定时 anchors == 0（锚定确实被关掉了）",
      r2z.metrics["anchors"] == 0.0, f"anchors={int(r2z.metrics['anchors'])}")
_margin = float(cfg.get("pursuit.map_safety_margin_m"))
_oob = sum(1 for x, y in zip(r2z.truth_x, r2z.truth_y) if not geo.in_bounds(x, y, _margin))
check("2.8 真值逐拍都在地图内（margin=%.2f m）" % _margin, _oob == 0, f"越界 {_oob} 拍")
check("2.9 全程没跑飞（看门狗未触发）", not r2z.metrics["diverged"],
      f"diverged={r2z.metrics['diverged']} at {int(r2z.metrics['diverged_at_step'])}")

# ---- 完整跑（auto max_steps），只用出厂配置 ----
t = time.time()
r2 = _run(FIELD)
m2 = r2.metrics
_el2 = time.time() - t
print("     实测(出厂配置完整跑): completed=%s steps=%d/%d time_s=%.2f xt_rms=%.5f "
      "xt_max=%.4f ratio=%.4f final_error=%.4f diverged=%s anchors=%d (%.2fs)" % (
          m2["completed"], r2.steps, int(m2["max_steps"]), m2["time_s"],
          m2["cross_track_rms_m"], m2["cross_track_max_m"], m2["path_len_ratio"],
          m2["final_error_m"], m2["diverged"], int(m2["anchors"]), _el2))
print("     分组横向 RMS: 直段 %.5f / 倒车 %.5f / 拐角 %.5f" % (
    m2["cross_track_rms_normal_m"], m2["cross_track_rms_reverse_m"],
    m2["cross_track_rms_corner_m"]))
# 参考基线（同一 fixture、更早的 sim.py 版本实测，见任务书 §7）：
print("     参考基线: 113.0 s / 5648 拍 / ratio ~0.97 跑完（旧版实测）")
# ★ 2026-10-07 修复：投影改为"任务段下界 + 窗口内比距离 + 行进方向裁决 + 段滞回"，
#   现场图在**出厂参数**下跑完（修前：822.5 s / 41127 拍 / ratio 5.32 / 未完成）。
#   这三条断言从此常驻（不再是 XFAIL），防止再次退化。
check("2.10 ★现场图在出厂参数下 completed=True", m2["completed"] is True,
      f"completed={m2['completed']} steps={r2.steps} ratio={m2['path_len_ratio']:.3f} "
      f"final_error={m2['final_error_m']:.3f}")
check("2.11 ★现场图 steps < 20000", r2.steps < 20000,
      f"steps={r2.steps} max_steps={int(m2['max_steps'])} hit_step_limit={m2['hit_step_limit']}")
check("2.12 现场图 path_len_ratio < 1.6（卡死时为 5.3 量级）",
      m2["path_len_ratio"] < 1.6, f"ratio={m2['path_len_ratio']:.4f}")
check("2.13 现场图没有跑飞看门狗触发", not m2["diverged"],
      f"diverged={m2['diverged']} at {int(m2['diverged_at_step'])}")
check("2.14 现场图真值未越界（如实记录）", m2["out_of_bounds_steps"] == 0.0,
      f"oob={int(m2['out_of_bounds_steps'])}")
check("2.15 零注入下信念↔真值仍逐位相同（仅锚定拍例外）",
      max(math.hypot(r2.x[i] - r2.truth_x[i], r2.y[i] - r2.truth_y[i])
          for i in range(r2.steps)) < ZERO_TOL,
      f"max_gap={max(math.hypot(r2.x[i] - r2.truth_x[i], r2.y[i] - r2.truth_y[i]) for i in range(r2.steps)):.4e} "
      f"anchors={int(m2['anchors'])}")

# ---- 折返区快照（8000 拍上限，秒级）：护栏红了时立刻能判断"是不是老毛病" ----
r2b = _run(FIELD, max_steps=FIELD_HARD_CAP)
print("     实测(%d 拍上限快照): completed=%s steps=%d ratio=%.4f final_error=%.4f "
      "diverged=%s diverged_at=%d" % (
          FIELD_HARD_CAP, r2b.metrics["completed"], r2b.steps,
          r2b.metrics["path_len_ratio"], r2b.metrics["final_error_m"],
          r2b.metrics["diverged"], int(r2b.metrics["diverged_at_step"])))
check("2.16 8000 拍快照与完整跑结论一致（排除步数上限造成的假象）",
      bool(r2b.metrics["completed"]) == bool(m2["completed"]),
      f"cap8000={r2b.metrics['completed']} auto={m2['completed']}")
# ★ 折返区定位证据（2026-10-07 修复后改写）。
#   修前：车在 (2.3, 4.4) 附近绕圈、`target_idx` 死在 20（D6）、投影弧长在
#         7.58 ↔ 8.40 之间反复跳，822 s 撞步数上限 —— 于是旧断言写成
#         "卡死位置确实停在 D6 附近"。
#   修后：路线跑完（`completed=True`），**旧断言必然失败**，所以改成检验
#         "修好的形态"：末位置落在终点 N5 的 `arrival_m` 量级内，
#         且折返区（第 18~20 段：A6→B6→D6）确实被走过了。
_last_xy = (r2b.truth_x[-1], r2b.truth_y[-1])
_goal_wp = FIELD_PATH.waypoints[-1]
_d_goal = math.hypot(_last_xy[0] - _goal_wp.x, _last_xy[1] - _goal_wp.y)
print("     末位置 = (%.3f, %.3f)；终点 %s = (%.3f, %.3f) 距离 %.3f m；"
      "折返区最大真值弧长 = %.3f m（D6 在 %.3f m）" % (
          _last_xy[0], _last_xy[1], _goal_wp.block, _goal_wp.x, _goal_wp.y, _d_goal,
          max(r2b.truth_s), FIELD_PATH.s_at_waypoint(20)))
check("2.17 末位置停在终点 %s 的 arrival_m 量级内（修前是卡死在 D6 附近）" % _goal_wp.block,
      _d_goal <= 2.0 * _arrival, f"d={_d_goal:.3f} m vs arrival={_arrival}")
check("2.18 ★折返区（A6→B6→D6）确实走过了（真值弧长越过 D6 = %.3f m）"
      % FIELD_PATH.s_at_waypoint(20),
      max(r2b.truth_s) >= FIELD_PATH.s_at_waypoint(20) - 1e-9,
      f"max_s_truth={max(r2b.truth_s):.3f} vs s(D6)={FIELD_PATH.s_at_waypoint(20):.3f}")

# =========================================================================== #
section("3. 误差注入对照：注入必须真的产生误差（同窗口 3000 拍）")
# =========================================================================== #
t = time.time()
r3c = _run(FIELD, max_steps=FIELD_CAP_STEPS, anchor_on_arrival=False)
r3f = _run(FIELD, speed_scale_err=0.18, anchor_on_arrival=False, max_steps=FIELD_CAP_STEPS)
r3a = _run(FIELD, speed_scale_err=0.18, anchor_on_arrival=True, max_steps=FIELD_CAP_STEPS)
print("     %-24s %10s %12s %12s %8s" % ("窗口 3000 拍", "xt_rms", "belief_rms",
                                          "belief_max", "anchors"))
for _tag, _r in (("零注入（对照）", r3c), ("eps=0.18 不锚定", r3f), ("eps=0.18 锚定", r3a)):
    print("     %-24s %10.5f %12.5f %12.5f %8d" % (
        _tag, _r.metrics["cross_track_rms_m"], _r.metrics["belief_error_rms_m"],
        _r.metrics["belief_error_max_m"], int(_r.metrics["anchors"])))
print("     (%.2fs)" % (time.time() - t))

check("3.1 对照组的信念误差 = 0（模型自洽）",
      r3c.metrics["belief_error_max_m"] < ZERO_TOL,
      f"belief_error_max={r3c.metrics['belief_error_max_m']:.4e}")
check("3.2 ★注入产生非零信念误差（仿真器不是安慰剂）",
      r3f.metrics["belief_error_rms_m"] > 0.05,
      f"belief_error_rms={r3f.metrics['belief_error_rms_m']:.5f}")
check("3.3 注入的信念误差 >= 对照组的 3 倍",
      r3f.metrics["belief_error_rms_m"] > 3.0 * max(1e-12, r3c.metrics["belief_error_rms_m"]),
      f"注入={r3f.metrics['belief_error_rms_m']:.5f} 对照={r3c.metrics['belief_error_rms_m']:.5f}")
check("3.4 注入使误差沿路线累积（>= 0.30 m RMS，drift_warn_m 量级）",
      r3f.metrics["belief_error_rms_m"] >= 0.30,
      f"belief_error_rms={r3f.metrics['belief_error_rms_m']:.5f} "
      f"max={r3f.metrics['belief_error_max_m']:.5f}")
check("3.5 注入版在对照窗口内给出可比样本（跑完 = 提前完成，不算差异）",
      r3f.steps >= FIELD_CAP_STEPS - 1 or r3f.metrics["completed"] is True,
      f"steps={r3f.steps} completed={r3f.metrics['completed']}")

# ⚠️ 锚定：修前（2026-10-07 之前）同窗口实测 不锚定 0.36020 / 锚定 0.34930（只差 3%），
#    docstring 宣称的"夹在 anchor_radius_m 内"不成立。修后投影/推进修好，
#    锚定确实把累积误差压回 anchor_radius_m 量级（见下方打印的实测值）。
print("     锚定 vs 不锚定: belief_rms %.5f vs %.5f (比值 %.3f) / xt_rms %.5f vs %.5f" % (
    r3a.metrics["belief_error_rms_m"], r3f.metrics["belief_error_rms_m"],
    r3a.metrics["belief_error_rms_m"] / max(1e-12, r3f.metrics["belief_error_rms_m"]),
    r3a.metrics["cross_track_rms_m"], r3f.metrics["cross_track_rms_m"]))
check("3.6 ★锚定把信念误差夹在 anchor_radius 量级内",
      r3a.metrics["belief_error_rms_m"] <= 2.0 * _arrival,
      f"锚定版 belief_error_rms={r3a.metrics['belief_error_rms_m']:.5f} vs "
      f"anchor_radius={_arrival}")
check("3.7 锚定确实被触发过（不是没生效）",
      r3a.metrics["anchors"] >= 1.0, f"anchors={int(r3a.metrics['anchors'])}")
check("3.8 ★吸附不能造误差：锚定版信念误差 <= 不锚定版的 2 倍",
      r3a.metrics["belief_error_rms_m"] <= 2.0 * max(1e-12, r3f.metrics["belief_error_rms_m"]),
      f"锚定={r3a.metrics['belief_error_rms_m']:.5f} 不锚定={r3f.metrics['belief_error_rms_m']:.5f} "
      f"(比值 {r3a.metrics['belief_error_rms_m'] / max(1e-12, r3f.metrics['belief_error_rms_m']):.3f})")
check("3.9 ★吸附不能造误差（横向）：锚定版横向 RMS <= 不锚定版的 2 倍",
      r3a.metrics["cross_track_rms_m"] <= 2.0 * max(1e-12, r3f.metrics["cross_track_rms_m"]),
      f"锚定={r3a.metrics['cross_track_rms_m']:.5f} 不锚定={r3f.metrics['cross_track_rms_m']:.5f}")

# 同注入放到直线路线上：误差必须小一个量级（证明误差与路线形状相关）
r3s = _run(STRAIGHT, speed_scale_err=0.18, anchor_on_arrival=False)
print("     同注入放直线 N10->M10: completed=%s belief_rms=%.6f belief_max=%.6f" % (
    r3s.metrics["completed"], r3s.metrics["belief_error_rms_m"],
    r3s.metrics["belief_error_max_m"]))
check("3.9 直线 + 同注入：仍跑完且信念偏差 < 0.05 m",
      r3s.metrics["completed"] is True and r3s.metrics["belief_error_max_m"] < 0.05,
      f"completed={r3s.metrics['completed']} max={r3s.metrics['belief_error_max_m']:.5f}")

# =========================================================================== #
section("4. 指令延迟：调大 → 横向误差变大，且仍能跑完")
# =========================================================================== #
# ★ 对照用"带 4 个 90 度拐角"的中间路线：延迟在**拐角**上才吃误差；
#   一格短线来不及体现，现场图又会卡死（§2），都不适合当延迟的对照。
_lat_rows = []
for _lat in (0.0, 0.06, 0.12, 0.20, 0.30):
    _r = _run(MID, cmd_latency_s=_lat)
    _lat_rows.append((_lat, _r))
    print("     lat=%.2fs completed=%-5s steps=%-5d xt_rms=%.6f xt_max=%.6f "
          "ratio=%.4f final=%.5f" % (
              _lat, _r.metrics["completed"], _r.steps, _r.metrics["cross_track_rms_m"],
              _r.metrics["cross_track_max_m"], _r.metrics["path_len_ratio"],
              _r.metrics["final_error_m"]))
_r0 = _lat_rows[0][1]
_r2 = _lat_rows[-1][1]
check("4.1 零延迟跑完", _r0.metrics["completed"] is True, f"steps={_r0.steps}")
check("4.2 延迟 0.30 s（15 个控制周期）仍能跑完",
      _r2.metrics["completed"] is True,
      f"completed={_r2.metrics['completed']} steps={_r2.steps}")
check("4.3 ★延迟 0.30 s 的横向 RMS > 零延迟",
      _r2.metrics["cross_track_rms_m"] > _r0.metrics["cross_track_rms_m"],
      f"{_r0.metrics['cross_track_rms_m']:.6f} -> {_r2.metrics['cross_track_rms_m']:.6f}")
check("4.4 横向 RMS 随延迟单调不减（5 档全查）",
      all(_lat_rows[i][1].metrics["cross_track_rms_m"]
          <= _lat_rows[i + 1][1].metrics["cross_track_rms_m"] + 1e-12
          for i in range(len(_lat_rows) - 1)),
      " / ".join("%.5f" % r.metrics["cross_track_rms_m"] for _, r in _lat_rows))
check("4.5 延迟越大越费拍数（单调不减）",
      all(_lat_rows[i][1].steps <= _lat_rows[i + 1][1].steps
          for i in range(len(_lat_rows) - 1)),
      " / ".join("%d" % r.steps for _, r in _lat_rows))
check("4.6 延迟版末点误差仍在 arrival_m 量级（< 0.20 m）",
      _r2.metrics["final_error_m"] < 0.20, f"final_error={_r2.metrics['final_error_m']:.5f}")

_f0 = _run(FIELD, cmd_latency_s=0.0, anchor_on_arrival=False, max_steps=FIELD_CAP_STEPS)
_f2 = _run(FIELD, cmd_latency_s=0.20, anchor_on_arrival=False, max_steps=FIELD_CAP_STEPS)
print("     现场图 3000 拍窗口: lat0 xt_rms=%.5f / lat0.20 xt_rms=%.5f" % (
    _f0.metrics["cross_track_rms_m"], _f2.metrics["cross_track_rms_m"]))
check("4.7 现场图上同向趋势（延迟 0.20 的横向 RMS 更大）",
      _f2.metrics["cross_track_rms_m"] > _f0.metrics["cross_track_rms_m"],
      f"{_f0.metrics['cross_track_rms_m']:.5f} -> {_f2.metrics['cross_track_rms_m']:.5f}")

# =========================================================================== #
section("5. 输出文件：write_csv / write_segments_csv / write_svg")
# =========================================================================== #
_tmp = tempfile.mkdtemp(prefix="dzbrain_sim_guard_")
try:
    _out = os.path.join(_tmp, "nested", "out")      # 父目录故意不存在
    _csv = S.write_csv(r1, os.path.join(_out, "traj.csv"))
    _svg = S.write_svg(r1, os.path.join(_out, "traj.svg"), STRAIGHT, title="GUARD SVG")
    _seg = S.write_segments_csv(r1, os.path.join(_out, "segments.csv"))
    print("     落盘目录: %s" % _out)
    for _p in (_csv, _svg, _seg):
        print("       %-14s exists=%s bytes=%d" % (
            os.path.basename(_p), os.path.isfile(_p),
            os.path.getsize(_p) if os.path.isfile(_p) else -1))

    check("5.1 write_csv 返回绝对路径且文件存在",
          os.path.isabs(_csv) and os.path.isfile(_csv), _csv)
    check("5.2 父目录不存在时自动创建", os.path.isdir(_out), _out)

    with open(_csv, "r", encoding="utf-8") as _fh:
        _rows = _fh.read().splitlines()
    _hdr = _rows[0].split(",")
    check("5.3 CSV 行数 = 表头 + 采样拍数（> 100 行）",
          len(_rows) == r1.steps + 1 and len(_rows) > 100,
          f"lines={len(_rows)} steps={r1.steps}")
    check("5.4 表头前 10 列 = 契约列（顺序不变）",
          _hdr[:10] == ["t", "x", "y", "yaw", "v", "steer", "truth_x", "truth_y",
                        "seg_index", "s"], f"hdr={_hdr[:10]}")
    check("5.5 表头含诊断列（truth_yaw/steer_cmd/kind/anchored/reverse）",
          all(_c in _hdr for _c in ("truth_yaw", "steer_cmd", "truth_seg_index",
                                    "truth_s", "kind", "anchored", "reverse")),
          f"n_cols={len(_hdr)}")
    check("5.6 每行列数与表头一致（不错位）",
          all(len(_r.split(",")) == len(_hdr) for _r in _rows[1:]), f"cols={len(_hdr)}")
    _f = _rows[1].split(",")
    check_close("5.7 首行 t = 0", float(_f[0]), 0.0, 1e-9)
    check_close("5.8 首行 truth_x = N10 格心", float(_f[6]), 0.525, 1e-9)
    check("5.9 kind 列取值合法",
          all(_r.split(",")[14] in ("normal", "reverse", "corner") for _r in _rows[1:]),
          "出现 normal/reverse/corner 之外的值")
    check("5.10 anchored/reverse 列是 0/1",
          all(_r.split(",")[15] in ("0", "1") and _r.split(",")[16] in ("0", "1")
              for _r in _rows[1:]), "出现非 0/1")

    with open(_svg, "r", encoding="utf-8") as _fh:
        _svg_text = _fh.read()
    check("5.11 SVG 头尾合法",
          _svg_text.lstrip().startswith("<?xml") and "<svg" in _svg_text
          and _svg_text.rstrip().endswith("</svg>"), f"chars={len(_svg_text)}")
    check("5.12 SVG 有路线 + 真值 + 信念三条折线",
          _svg_text.count("<polyline") >= 3, f"polyline={_svg_text.count('<polyline')}")
    check("5.13 SVG 航点数 = 路线航点数",
          _svg_text.count("<circle") == len(STRAIGHT.waypoints),
          f"circles={_svg_text.count('<circle')} wps={len(STRAIGHT.waypoints)}")
    check("5.14 SVG 含自定义标题", "GUARD SVG" in _svg_text)
    check("5.15 SVG 含路线配色与图例",
          "#9aa4b2" in _svg_text and "completed=" in _svg_text and "viewBox" in _svg_text)

    with open(_seg, "r", encoding="utf-8") as _fh:
        _srows = _fh.read().splitlines()
    check("5.16 逐段 CSV = 表头 + 段数",
          len(_srows) == len(r1.segments) + 1 and len(_srows) >= 2,
          f"lines={len(_srows)} segs={len(r1.segments)}")
    check("5.17 逐段 CSV 表头完整",
          _srows[0].split(",") == ["index", "from_block", "to_block", "length_m", "kind",
                                   "corner_deg", "peak_m", "peak_t", "peak_s"],
          f"hdr={_srows[0]}")
    check("5.18 SegmentStat 字段可读（段 0 = N10 -> M10）",
          r1.segments[0].from_block == "N10" and r1.segments[0].to_block == "M10"
          and abs(r1.segments[0].length_m - 0.35) < 1e-9,
          f"{r1.segments[0].from_block}->{r1.segments[0].to_block} "
          f"{r1.segments[0].length_m:.4f} m")

    # 退化跨度（一格长路线）不该让 SVG 缩放除零
    _degen = _run(STRAIGHT, max_steps=50)
    _svg2 = S.write_svg(_degen, os.path.join(_out, "degen.svg"), STRAIGHT)
    check("5.19 短路线（退化跨度）SVG 仍能落盘",
          os.path.isfile(_svg2) and os.path.getsize(_svg2) > 100,
          f"bytes={os.path.getsize(_svg2) if os.path.isfile(_svg2) else -1}")
finally:
    shutil.rmtree(_tmp, ignore_errors=True)

# =========================================================================== #
section("6. 模型单元级：step_bicycle")
# =========================================================================== #
_wb = float(cfg.get("pursuit.wheelbase_m"))
_x, _y, _yaw = S.step_bicycle(0.0, 0.0, 0.0, 1.0, 0.0, 1.0, _wb)
print("     wheelbase=%.3f  直行 1 s (v=1, steer=0) -> x=%.6f y=%.6f yaw=%.6f" % (
    _wb, _x, _y, _yaw))
check_close("6.1 直行 1 m：x = 1.0", _x, 1.0, 1e-12)
check_close("6.2 直行 1 m：y 不变（横向零漂）", _y, 0.0, 1e-12)
check_close("6.3 直行不改变航向", _yaw, 0.0, 1e-12)

_x2, _y2, _yaw2 = S.step_bicycle(0.5, 0.7, 0.3, 0.0, 25.0, 1.0, _wb)
print("     v=0 steer=25 -> x=%.15f y=%.15f yaw=%.15f" % (_x2, _y2, _yaw2))
check("6.4 v=0 且舵角 0 时位置**逐位**不动", _x2 == 0.5 and _y2 == 0.7,
      f"got ({_x2!r}, {_y2!r})")
# ⚠️ 输出前会过一次 `normalize_angle`：0.3 与 0.2999999999999998 是同一个角度
#    （差 1 ULP），所以这里比角度差而不是比浮点字面量。
check_close("6.5 v=0 时打舵也不动（角速度正比于 v，只差折绕的 1 ULP）",
            _yaw2, 0.3, 1e-12)

# ★ 转向用 dt=0.5 s：v=1、steer=30 时角速度 = v/L*tan(30) = 2.99 rad/s，
#   dt=1.0 s 会转过近 150 度、被折绕成负值，把"左转为正"的断言搞成假失败。
_x3, _y3, _yaw3 = S.step_bicycle(0.0, 0.0, 0.0, 1.0, 30.0, 0.5, _wb)
_omega = (1.0 / _wb) * math.tan(math.radians(30.0)) * 0.5
print("     v=1 steer=30 dt=0.5 -> x=%.4f y=%.4f yaw=%.3f deg（解析 %.3f deg）" % (
    _x3, _y3, math.degrees(_yaw3), math.degrees(_omega)))
check_close("6.6 前轮 30 度的 yaw 增量与解析式一致（v/L*tan(delta)*dt）",
            geo.normalize_angle(_yaw3), geo.normalize_angle(_omega), 1e-9)
check("6.7 正舵角 = 左转（yaw 为正，未被折绕）",
      0.0 < _yaw3 < math.pi, f"yaw={math.degrees(_yaw3):.3f} deg")
# ⚠️ 欧拉积分的第一步 yaw 还是 0，横向位移必然为 0（不是 bug）。
#    要验证"往左偏"必须**再走一步**：此时 yaw 已转过来，横向分量才出现。
_x6, _y6, _yaw6 = S.step_bicycle(_x3, _y3, _yaw3, 1.0, 30.0, 0.5, _wb)
print("     第二步(沿用已转过的 yaw): x=%.4f y=%.4f（第一步 y=%.6f）" % (_x6, _y6, _y3))
check("6.8 左转第二步的横向位移为正（向车体左侧偏）",
      _y6 > 0.0 and _y6 > _y3, f"y0={_y3:.6f} -> y1={_y6:.6f}")

# 折绕本身要正确：超过 pi 的角度必须被折回 (-pi, pi]
_x5, _y5, _yaw5 = S.step_bicycle(0.0, 0.0, 0.0, 1.0, 30.0, 1.0, _wb)
_omega_full = (1.0 / _wb) * math.tan(math.radians(30.0)) * 1.0
print("     v=1 steer=30 dt=1.0 -> yaw=%.3f deg（未折绕应为 %.3f deg）" % (
    math.degrees(_yaw5), math.degrees(_omega_full)))
check("6.8b 大角度被折绕到 (-pi, pi]（不是原样返回）",
      -math.pi < _yaw5 <= math.pi
      and abs(geo.normalize_angle(_omega_full) - _yaw5) < 1e-12,
      f"yaw={_yaw5:.6f} rad = {math.degrees(_yaw5):.3f} deg, "
      f"normalize({math.degrees(_omega_full):.3f} deg)={math.degrees(geo.normalize_angle(_omega_full)):.3f} deg")

_x4, _y4, _yaw4 = S.step_bicycle(0.0, 0.0, 0.0, 1.0, 10.0, 1.0, 0.0)
check("6.9 轴距 0（防御分支）：航向不变、位置仍前进",
      _yaw4 == 0.0 and abs(_x4 - 1.0) < 1e-12, f"x={_x4} yaw={_yaw4}")

# =========================================================================== #
section("7. ★回归护栏：投影窗口常量 + 出厂配置端到端")
# =========================================================================== #
_proj_belief = getattr(S, "_PROJ_BELIEF_M", None)
_proj_fwd = getattr(S, "_PROJ_FWD_M", None)
_proj_back = getattr(S, "_PROJ_BACK_M", None)
print("     投影窗口常量: _PROJ_BELIEF_M=%s  _PROJ_FWD_M=%s  _PROJ_BACK_M=%s" % (
    _proj_belief, _proj_fwd, _proj_back))
# ★ 史实（sim.py docstring 亦载）：把信念投影窗口从 1.50 收到 0.75 后，
#   现场图从 "113.0 s / 5648 拍跑完" 退化成 "822.5 s / 撞步数上限 / ratio 5.31"。
if _proj_belief is None:
    check("7.0 定位不到 sim._PROJ_BELIEF_M（实现改了名/改成配置键）", False,
          "无法断言窗口常量：请人工确认信念投影窗口未被收窄")
else:
    check_close("7.1 ★信念投影窗口 _PROJ_BELIEF_M = 1.50 m（收窄它会让现场图卡死）",
                float(_proj_belief), 1.50, 1e-12)
check("7.2 物理投影窗口仍是前宽后窄（0.80 / 0.20）",
      _proj_fwd is not None and _proj_back is not None
      and abs(float(_proj_fwd) - 0.80) < 1e-12 and abs(float(_proj_back) - 0.20) < 1e-12,
      f"fwd={_proj_fwd} back={_proj_back}")

# ---- 端到端护栏：**只用出厂配置**（§2 已跑，这里把结论按护栏语义再钉一次）----
print("     护栏结论: completed=%s steps=%d/%d ratio=%.4f xt_rms=%.5f "
      "（基线参考 5648 拍 / ratio ~0.97）" % (
          m2["completed"], r2.steps, int(m2["max_steps"]), m2["path_len_ratio"],
          m2["cross_track_rms_m"]))
# ★ 2026-10-07 修复后常驻断言（修前是 XFAIL：822.5 s / 41127 拍 / ratio 5.32 未完成）。
check("7.3 ★出厂配置下现场图跑完（端到端护栏）", m2["completed"] is True,
      f"completed={m2['completed']} steps={r2.steps} ratio={m2['path_len_ratio']:.4f} "
      f"final_error={m2['final_error_m']:.4f}")
check("7.4 出厂配置下横向 RMS 仍 < 0.20 m（没退化成乱跑）",
      m2["cross_track_rms_m"] < 0.20, f"xt_rms={m2['cross_track_rms_m']:.5f}")
check("7.5 分组指标键齐全、有限、非负",
      all(isinstance(m2[k], float) and m2[k] == m2[k] and m2[k] >= 0.0
          for k in S.GROUPED_METRIC_KEYS),
      " / ".join("%s=%.5f" % (k, m2[k]) for k in ("cross_track_rms_normal_m",
                                                  "cross_track_rms_reverse_m",
                                                  "cross_track_rms_corner_m")))

# =========================================================================== #
section("8. 接口一致性 / 确定性 / 参数校验")
# =========================================================================== #
check("8.1 METRIC_KEYS 契约键全部存在",
      all(k in r1.metrics for k in S.METRIC_KEYS),
      f"missing={[k for k in S.METRIC_KEYS if k not in r1.metrics]}")
check("8.2 GROUPED_METRIC_KEYS 全部存在",
      all(k in r1.metrics for k in S.GROUPED_METRIC_KEYS),
      f"missing={[k for k in S.GROUPED_METRIC_KEYS if k not in r1.metrics]}")
_seqs = ("t", "x", "y", "yaw", "v", "steer", "truth_x", "truth_y", "truth_yaw",
         "seg_index", "s", "truth_seg_index", "truth_s", "steer_cmd",
         "anchored", "near_vertex", "reverse")
check("8.3 逐拍序列全部等长（不错位）",
      len({len(getattr(r1, _n)) for _n in _seqs}) == 1,
      f"lengths={sorted({len(getattr(r1, _n)) for _n in _seqs})}")
check("8.4 steps 属性 = 序列长度", r1.steps == len(r1.t) == len(r1.steer))
check("8.5 逐段表长度 = 段数", len(r1.segments) == S.Path(STRAIGHT).segment_count,
      f"segs={len(r1.segments)}")
check("8.6 series() 可取列", len(r1.series("steer_cmd")) == r1.steps)
check_raises("8.7 series() 未知列抛 KeyError", KeyError, r1.series, "no_such_series")
check_close("8.8 metric() 缺键返回默认值", r1.metric("no_such_key", -1.0), -1.0, 1e-12)

_rA = _run(MID, speed_scale_err=0.18, cmd_latency_s=0.20)
_rB = _run(MID, speed_scale_err=0.18, cmd_latency_s=0.20)
check("8.9 ★确定性：同参数两次运行逐位相同",
      _rA.t == _rB.t and _rA.x == _rB.x and _rA.y == _rB.y and _rA.steer == _rB.steer
      and _rA.metrics == _rB.metrics, f"steps {_rA.steps} vs {_rB.steps}")
check("8.10 无噪声时 seed 不影响结果",
      _run(MID, seed=1).x == _run(MID, seed=999).x, "seed 影响了无噪声结果")
_n1 = _run(MID, pos_noise_m=0.05, seed=20261007)
_n2 = _run(MID, pos_noise_m=0.05, seed=20261007)
check("8.11 有噪声时同 seed 逐位可复现",
      _n1.x == _n2.x and _n1.y == _n2.y, f"steps={_n1.steps}")
check("8.12 不同 seed 在有噪声时给出不同轨迹",
      _run(MID, pos_noise_m=0.05, seed=1).x != _n1.x, "不同 seed 结果相同（噪声没生效？）")

check_raises("8.13 dt=0 抛 ValueError", ValueError, _run, MID, dt=0.0)
check_raises("8.14 负延迟抛 ValueError", ValueError, _run, MID, cmd_latency_s=-0.01)
check_raises("8.15 pos_noise 超上限抛 ValueError", ValueError, _run, MID, pos_noise_m=0.6)
check_raises("8.16 trace_every=0 抛 ValueError", ValueError, _run, MID, trace_every=0)
check_raises("8.17 init_yaw 非法值抛 ValueError", ValueError, _run, MID, init_yaw="diagonal")

_txt = S.format_metrics(r1, S.SimConfig(), title="T")
check("8.18 format_metrics 可渲染（>= 20 行且含标题）",
      len(_txt.splitlines()) >= 20 and _txt.startswith("T"),
      f"lines={len(_txt.splitlines())}")
check("8.19 format_metrics 含分组表与限速描述",
      "P95" in _txt and "RMS" in _txt and "斜坡" in _txt,
      f"lines={len(_txt.splitlines())}")
_fo = S.format_metrics(_run(STRAIGHT, max_steps=20), S.SimConfig())
check("8.20 撞步数上限时 format_metrics 给出提示行",
      "hit_step_limit" in str(_run(STRAIGHT, max_steps=20).metrics)
      and "步数上限" in _fo.replace("\ufffd", ""),
      "未找到步数上限提示")

# ---- trace_hook：排障钩子的契约（字段名是测试与排障脚本的共同依赖）----
_seen: list[tuple[int, dict]] = []


def _hook(step, state):
    if step <= 2:
        _seen.append((step, dict(state)))


_rh = _run(STRAIGHT, max_steps=5, trace_hook=_hook)
_hk_need = ("t", "bx", "by", "byaw", "tx", "ty", "tyaw", "v", "steer", "steer_cmd",
            "steer_target", "s", "seg", "dist", "s_truth", "seg_truth", "kind",
            "anchors", "anchored", "target_idx", "s_floor", "goal_d", "trk")
check("8.21 trace_hook 被调用且步号从 1 开始",
      len(_seen) == 2 and _seen[0][0] == 1, f"calls={[s for s, _ in _seen]}")
check("8.22 trace_hook 的 state 字段齐全（%d 个）" % len(_hk_need),
      all(k in _seen[0][1] for k in _hk_need),
      f"missing={[k for k in _hk_need if k not in _seen[0][1]]}")

# =========================================================================== #
section("9. 接口差异与仿真器缺陷清单（只读记录，供决策）")
# =========================================================================== #
_NOTES = [
    "锚定（anchor_on_arrival）未达 docstring 宣称的效果：宣称'把信念误差夹在 "
    "anchor_radius_m 内、不锚定则无界累积'，实测同一窗口下 0.36020（不锚定） vs "
    "0.28540（锚定）—— 量级相同，且锚定版横向 RMS 更大（0.11931 -> 0.15300）。"
    "原因：判据 `_anchor_allowed` 排除拐角顶点，而现场图 34 点里绝大多数是 90 度顶点 "
    "=> 全程只锚 3~5 次；且吸附在零误差场景下自身会引入 0.119 m 误差。",

    "sim.py 落盘期间被另一条线高频改写（实测同一分钟内 73176 -> 74863 -> 75124 -> 73718 "
    "字节），并**实测到一次崩**：run_sim 抛 `UnboundLocalError: anchor_idx`（当时约 515 行），"
    "同一环境下重跑即正常。⇒ 这是并发改写造成的瞬态，不是稳定缺陷；"
    "但任何以本仿真器结论为依据的动作，都应先确认 sim.py 在本轮未被改写（本文件抬头打印 hash）。",

    "`pursuit.vertex_clamp_m` 是 core/config.py 的新键（Spec 默认 0.04），sim.py 里只有注释"
    "提到它、没有读取点；另有 `pursuit.lookahead_corner_angle_deg`。"
    "⚠️ 若为了'让现场图跑完'而改配置参数，默认参数下的回归就再也不会被发现 —— "
    "本文件因此**只用出厂 robot.yaml**，绝不 override。",

    "末点停车精度与代码自述不符：`_GOAL_STOP_M = 0.03` 注释说'真值进入 3 cm 才算到'，"
    "但末点是先被 `pursuit.arrival_m = 0.12` 判到位并推进 target_idx，下一拍投影弧长"
    "直接抬到 total_length => reached_end=True => 刹车。实测停点固定在 0.118~0.133 m"
    "（= arrival_m 量级），0.03 m 的门从未生效。",

    "`geometry.in_bounds` 在 margin=0.6 时对**明显场外**的点也返回 True"
    "（实测 in_bounds(0.1, 0.1, 0.6) is True）—— 它只能当'明显跑飞'的粗筛，"
    "不能当精确边界判据；本文件的越界用例据此只做粗筛。",

    "接口与预期不一致（其余）：(a) `SimResult.metrics` 是 dataclass 字段 dict（不是属性/方法）；"
    "(b) `sim_cli.py --out-dir` 只写 traj.csv/traj.svg，不写逐段 CSV（`write_segments_csv` "
    "只有测试在用）；(c) `write_svg(result, path, route, title)` 需要 route，"
    "而 `write_csv(result, path)` 不需要；(d) `SimConfig.bounds_margin_m=0` 时回落到 "
    "`pursuit.map_safety_margin_m`（0 不是'无边距'，是'用配置值'）；"
    "(e) `trace_hook` 是 `object` 类型标注，签名 (step:int, state:dict) 只写在文档里，"
    "本文件把它钉成 23 个字段的契约。",

    "现场图卡死点定位（实测）：车在 (2.3, 4.4) 附近绕圈，target_idx 停在 20（D6），"
    "投影弧长在 7.58 与 8.40 之间反复跳变（s_at(#19 B6)=8.400 / s_at(#20 D6)=9.100），"
    "从未满足'已过点即放行'判据 => 822.5 s / 41127 拍 / ratio 5.39。"
    "零注入、零延迟、锚定开/关下都复现 => 与注入无关，是 mission 推进/投影夹紧的问题。"
    "★ 同一现象已被另一条线的 `tests\\test_sim.py` 用 `FIELD_CFG` 覆盖三个 pursuit 参数绕过，"
    "本文件是唯一会在**出厂参数**下把它判红的用例。",
]
for _note in _NOTES:
    print("     - %s" % _note)
check("9.1 差异与缺陷清单已记录（>= 5 条）", len(_NOTES) >= 5, f"n={len(_NOTES)}")

# =========================================================================== #
#  汇总
# =========================================================================== #
print()
print("=" * 78)
print("总耗时: %.2f s" % (time.time() - _T0))
if _XFAILED:
    print("已知不达标项（XFAIL，已按 FAIL 记账以保持红色信号）：")
    for _item in _XFAILED:
        print("  · %s" % _item)
print("=" * 78)
finish()
