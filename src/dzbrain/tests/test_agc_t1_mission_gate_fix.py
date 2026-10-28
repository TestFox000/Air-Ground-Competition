# -*- coding: utf-8 -*-
"""AGC-T1 补丁回归：`MissionTask._current_s` 的几何可信门 + 读侧不回写。

补丁真源见卡 AGC-T1-PROGRESS-20261024 §4A.10（Lead 2026-10-24 批准）：
  (a) 采纳判据加**几何可信门** —— `proj.dist > pursuit.drift_warn_m(0.30)` 时先做
      无 hint 全路线重投影（`Path.project` 的 hint=None 分支，与 `localization._heal_stuck`
      / tracker relock 同语义）；
  (b) **读侧不回写** `P_ROUTE_INDEX/P_ROUTE_S`（Mission 只算不写，键的唯一常规写者
      回到 Localization；改键只走授权路径 `_reset_belief` / `_install_route`）。

本文件断言“修好之后”的语义；“修复前”的现场指纹由 `test_agc_t1_progress_contract.py`
在 legacy（门限=inf）模式下继续钉着，两者用同一份代码、只差 `_drift_gate_m` 一个属性。

补丁未落地时（无 `_drift_gate_m` 属性）本文件整体 SKIP 并返回 0，不制造红。

跑法（v1 根目录）：
  python -B -X utf8 dzrobot_v2/src/dzbrain/tests/test_agc_t1_mission_gate_fix.py
"""
from __future__ import annotations

import inspect
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from harness import check, finish, section                            # noqa: E402

from dzbrain.core.blackboard import P_ROUTE_INDEX, P_ROUTE_S          # noqa: E402
from dzbrain.tasks.mission import MissionTask                         # noqa: E402
from test_agc_t1_progress_contract import (F9_APPROACH, F9_STOP,      # noqa: E402
                                           Rig)

#: run2-mission.log 第 449/452/459 行（F9→D10→C10 途中）的原样位姿
FINGERPRINT_POSES = ((0.606, 3.783, 136.5), (0.553, 3.846, 123.0), (0.514, 3.920, 111.3))
#: 实跑那三拍打印出来的 段/s（修复后不应再出现）
LOG_FINGERPRINT = ((6, 2.18), (6, 2.13), (5, 2.10))


def patched() -> bool:
    return hasattr(Rig().mission, "_drift_gate_m")


# --------------------------------------------------------------------------- #
#  G1 F9 停站：旧 hint 2.42 一拍自愈到真值 4.40（门开/门关只差一个属性）
# --------------------------------------------------------------------------- #
def test_g1_f9_heal() -> None:
    section("G1 F9 停站：hint 驻留旧段 2.42 → 几何可信门一拍把它拉回 段11/4.40")
    rig = Rig()
    rig.pose(*F9_STOP)
    rig.loc.tick()                                   # Localization 先写一版权威值
    s_loc = float(rig.bb.get(P_ROUTE_S))
    rig.mission._last_s = 2.42                       # 旧 hint（实跑同态）
    s_fix = rig.mission._current_s()
    check("G1.1 门开：Mission 自己算到 4.402（不再驻留 2.48）",
          abs(s_fix - 4.402) < 5e-3, f"s={s_fix:.4f}")
    check(f"G1.2 自愈后 _last_lat_m 只有 {rig.mission._last_lat_m * 100:.1f} cm"
          "（门用的就是同一份 proj.dist）",
          rig.mission._last_lat_m <= 0.05, f"_last_lat_m={rig.mission._last_lat_m:.4f}")
    check("G1.3 与 Localization 的权威值差 < 2 cm（两层投影口径一致）",
          abs(s_fix - s_loc) < 0.02, f"mission={s_fix:.4f} loc={s_loc:.4f}")

    legacy = Rig(legacy=True)                        # 同一份代码，只把门限抬到 inf
    legacy.pose(*F9_STOP)
    legacy.loc.tick()
    legacy.mission._last_s = 2.42
    s_old = legacy.mission._current_s()
    check("G1.4 门关（= 修复前）：同一姿态回到 2.483 ⇒ 门是唯一变量",
          abs(s_old - 2.483) < 5e-3 and abs(s_old - s_fix) > 1.9,
          f"legacy={s_old:.4f} fixed={s_fix:.4f}")


# --------------------------------------------------------------------------- #
#  G2 现场指纹回归：日志 449/452/459 三拍不再回退到 段6/段5
# --------------------------------------------------------------------------- #
def test_g2_fingerprint() -> None:
    section("G2 现场指纹：日志 449/452/459 三拍修复后跟车（段14~16 / 5.7~5.9），不再是 段6/段5")
    rig = Rig()
    rig.mission._last_s = 2.483                      # F9 停站时 Mission 自己的值
    got = []
    for (x, y, yaw) in FINGERPRINT_POSES:
        rig.pose(x, y, yaw)
        rig.tracker.tick()
        s_m = rig.mission._current_s()
        seg_truth = rig.mission._path.project(x, y).seg_index    # 无 hint = 几何真值段
        got.append((seg_truth, round(s_m, 2), round(float(rig.tracker.telemetry()["s"]), 2)))
    check("G2.1 三拍都跟到车实际所在（段14~16、s 5.6~5.9）",
          all(14 <= g[0] <= 16 and 5.6 <= g[1] <= 5.9 for g in got), f"got {got}")
    check("G2.2 与 Tracker 同拍差 < 2 cm（两层投影一致，没有第二次驻留）",
          all(abs(g[1] - g[2]) < 0.02 for g in got), f"got {got}")
    check("G2.3 实跑的 段6/2.18、段5/2.10 两处回退不再出现（差 > 3.0 m）",
          all(g[1] - s > 3.0 for g, (_i, s) in zip(got, LOG_FINGERPRINT)),
          f"log={LOG_FINGERPRINT} now={[(g[0], g[1]) for g in got]}")
    check("G2.4 段号不再随 s 落到段界而减一：2.100 是旧 hint 窗口的产物，真值在 5.8",
          got[0][0] >= 14 and got[-1][0] >= 14, f"got {[g[0] for g in got]}")


# --------------------------------------------------------------------------- #
#  G3 读侧不回写：`_current_s` 一次都不写黑板
# --------------------------------------------------------------------------- #
def test_g3_no_writeback() -> None:
    section("G3 读侧不回写：_current_s 只读黑板（写计数 +0），改键只走授权路径")
    rig = Rig()
    rig.pose(*F9_STOP)
    rig.loc.tick()
    before = rig.bb.writes
    src_before = rig.bb.entry(P_ROUTE_S).source
    for hint in (2.42, 4.39, 0.0, 99.0, 2.48):
        rig.mission._last_s = hint
        rig.mission._current_s()
    check("G3.1 五拍（hint=2.42/4.39/0.0/99/2.48）bb.writes 增量为 0",
          rig.bb.writes == before, f"delta={rig.bb.writes - before}")
    check("G3.2 键的来源仍是上一个写者（Localization），没被 Mission 抢走",
          rig.bb.entry(P_ROUTE_S).source == src_before == "Localization",
          f"src={rig.bb.entry(P_ROUTE_S).source!r}")
    code = inspect.getsource(MissionTask._current_s)
    check("G3.3 静态核对：_current_s 源码里没有任何 bb.update( / bb.set( 调用",
          ("bb.update(" not in code) and ("bb.set(" not in code),
          "still writes" if ("bb.update(" in code or "bb.set(" in code) else "clean")
    check("G3.4 门本身也不写键：自愈那一路只改本地 proj/_last_lat_m",
          rig.mission._last_lat_m is not None and rig.bb.writes == before,
          f"_last_lat_m={rig.mission._last_lat_m!r}")


# --------------------------------------------------------------------------- #
#  G4/G5 采纳判据不变：一致就采纳缓存，冲突就用自身值（都不写）
# --------------------------------------------------------------------------- #
def _case(cached_s: float, cached_idx: int, hint_s: float, legacy: bool = False):
    rig = Rig(legacy=legacy)
    rig.pose(*F9_STOP)
    rig.bb.set(P_ROUTE_S, cached_s, "Localization", ts=rig.clock.now())
    rig.bb.set(P_ROUTE_INDEX, cached_idx, "Localization", ts=rig.clock.now())
    rig.mission._last_s = hint_s
    out = rig.mission._current_s()
    return rig, out


def test_g4_g5_adoption() -> None:
    section("G4/G5 采纳判据未被改动：|缓存-自身|<=arrival_m 采纳缓存，否则用自身（都不写键）")
    rig, out = _case(4.40, 11, 4.39)
    check("G4.1 缓存 4.40 与自身 4.402 差 0.002 → 采纳缓存 4.40",
          abs(out - 4.40) < 1e-9, f"out={out!r}")
    check("G4.2 且不写键（来源仍是 Localization）",
          rig.bb.entry(P_ROUTE_S).source == "Localization",
          f"src={rig.bb.entry(P_ROUTE_S).source!r}")

    rig, out = _case(2.42, 6, 4.39)
    check("G5.1 缓存旧 2.42、自身 4.402 → 采用自身 4.402（不再被缓存拖住）",
          abs(out - 4.402) < 5e-3, f"out={out!r}")
    check("G5.2 键保持 段6/2.42（等 Localization 下一拍自己修，不由 Mission 代劳）",
          rig.bb.get(P_ROUTE_INDEX) == 6 and abs(float(rig.bb.get(P_ROUTE_S)) - 2.42) < 1e-9,
          f"key=({rig.bb.get(P_ROUTE_INDEX)},{rig.bb.get(P_ROUTE_S)!r})")
    rig.loc.tick()
    check("G5.3 Localization 下一拍就把键修回 段11/4.40（20 Hz，最迟 50 ms）",
          rig.bb.get(P_ROUTE_INDEX) == 11 and abs(float(rig.bb.get(P_ROUTE_S)) - 4.40) < 0.05,
          f"key=({rig.bb.get(P_ROUTE_INDEX)},{rig.bb.get(P_ROUTE_S)!r})")


# --------------------------------------------------------------------------- #
#  G6 授权路径仍然能写键（补丁只删“读侧回写”，没删授权写）
# --------------------------------------------------------------------------- #
def test_g6_authorized_writes() -> None:
    section("G6 授权写没被误删：reset / 换路 仍然改键（补丁只移除读侧回写）")
    rig = Rig()
    rig.pose(*F9_STOP)
    rig.loc.tick()
    rig.mission._install_route(rig.route, source="换路", keep_progress=True)
    check("G6.1 keep_progress=True → 段落/s 原样保留",
          rig.bb.get(P_ROUTE_INDEX) == 11 and abs(float(rig.bb.get(P_ROUTE_S)) - 4.40) < 0.05,
          f"key=({rig.bb.get(P_ROUTE_INDEX)},{rig.bb.get(P_ROUTE_S)!r})")
    rig.mission._install_route(rig.route, source="重新载入")
    check("G6.2 普通载入 → 段0/s0.0（source=MissionTask）",
          rig.bb.get(P_ROUTE_INDEX) == 0 and float(rig.bb.get(P_ROUTE_S)) == 0.0
          and rig.bb.entry(P_ROUTE_S).source == "MissionTask",
          f"key=({rig.bb.get(P_ROUTE_INDEX)},{rig.bb.get(P_ROUTE_S)!r})")
    rig.mission._reset_belief("N10,y")
    check("G6.3 reset:BLOCK,HEAD → 段-1/s0.0（source=MissionTask）",
          rig.bb.get(P_ROUTE_INDEX) == -1 and float(rig.bb.get(P_ROUTE_S)) == 0.0
          and rig.bb.entry(P_ROUTE_S).source == "MissionTask",
          f"key=({rig.bb.get(P_ROUTE_INDEX)},{rig.bb.get(P_ROUTE_S)!r})")


# --------------------------------------------------------------------------- #
#  G7 健康路径上零影响（门是幂等的，不引入抖动）
# --------------------------------------------------------------------------- #
def test_g7_idempotent() -> None:
    section("G7 健康路径零影响：hint 本来就贴着车时，门开/门关结果逐位相同（不抖）")
    for (x, y, yaw) in (F9_APPROACH, F9_STOP):
        vals = []
        for legacy in (False, True):
            rig = Rig(legacy=legacy)
            rig.pose(x, y, yaw)
            rig.loc.tick()
            rig.mission._last_s = float(rig.bb.get(P_ROUTE_S))    # 健康 hint
            vals.append(rig.mission._current_s())
        check(f"G7 位姿({x},{y})：门开 {vals[0]:.4f} == 门关 {vals[1]:.4f}",
              abs(vals[0] - vals[1]) < 1e-9, f"on={vals[0]!r} off={vals[1]!r}")


def main() -> int:
    if not patched():
        print("\n[SUSPEND] 生产补丁未落地（MissionTask 无 _drift_gate_m）：本文件跳过，不算红。"
              "\n         见卡 AGC-T1-PROGRESS-20261024 §4A.10 与 _tmp_session/agc-1024/t1/patch/。")
        finish()
        return 0
    test_g1_f9_heal()
    test_g2_fingerprint()
    test_g3_no_writeback()
    test_g4_g5_adoption()
    test_g6_authorized_writes()
    test_g7_idempotent()
    finish()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
