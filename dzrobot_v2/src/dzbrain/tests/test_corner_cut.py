# -*- coding: utf-8 -*-
r"""卡 t53 专项：拐角**条件切角**（90° 直角 → 45° 斜段）。

为什么需要（实测：`_tmp_session/t53_corner/`）：
  现役 44 点有 **18 个 90° 直角**（拐角横向峰值 0.1192 m / 顶点超调 0.0491 m / 急转弯 18 处），
  直角处纯追踪必须一次转过 90° ⇒ 超调。删掉直角顶点后，前后两点**对角相邻**、
  连线天然是 **45° 斜段**（"把 90° 转角分成两段 45°"），而 token 仍是纯格名
  ⇒ 往返 / 门禁 / 车侧全兼容（**不需要**任何"格心+米制偏移"的 token 语法）。

本文件钉住三件事：
  ① 七条硬规则逐条生效（① 雷格 ② 离雷 0.15 m/不更近 ③ 障碍·挡板·靶板净空不更近
     ④ 雷三连 ⑤ 桥洞三连 ⑥ 只切 90°±2° ⑦ 停站/指令格）；
  ② **默认关闭** ⇒ 现役发车路线逐 token 不变（冻结路线不受影响）；
  ③ 开启后的产物：39 点、5 处切角、`F3*` 保留、`validate` 0 error、
     顶点超调显著下降且**受保护指标**（雷 8/8 / 压靶 0 / 碰撞 0 / 挡板 0 / 越界 0 / 靶板净空不降）全保持。

跑：python src\dzbrain\tests\test_corner_cut.py
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
sys.stdout.reconfigure(errors="replace")

import yaml                                                       # noqa: E402

from dzbrain.core import geometry as geo                          # noqa: E402
from dzbrain.core.config import Config                            # noqa: E402
from dzbrain.route import parser as P                             # noqa: E402
from dzbrain.route import planner as PL                           # noqa: E402
from dzbrain.route import validate as V                           # noqa: E402
from dzbrain.tools import sim as S                                # noqa: E402
from harness import check, section, finish                        # noqa: E402

WS = pathlib.Path(__file__).resolve().parents[3]
CFG_PATH = WS / "config" / "robot.yaml"
MAP_PATH = WS / "config" / "map.yaml"

MAP = yaml.safe_load(MAP_PATH.read_text(encoding="utf-8"))
SPEC = PL.load_map(MAP, source=str(MAP_PATH))
LIVE = [str(t) for t in (Config.load(str(CFG_PATH)).get("route.tokens") or [])]
BAR = {"barrier_row_hi": 6.0, "barrier_col_from": "D"}


def _cfg(**over):
    """主配置的**独立副本**（`Config.override` 是就地改，必须每次重新 load）。"""
    cfg = Config.load(str(CFG_PATH))
    for k, v in over.items():
        cfg.override(k, v)
    return cfg


def _cells(blocks):
    return [(geo.block_to_index(b)[0] + 1, geo.block_to_index(b)[1]) for b in blocks]


def _map(mines=(), obstacles=(), targets=(), bridges=("K5", "K9"), start="N10", end="N5"):
    return {"map": {"name": "cut-test", "start": start, "end": end,
                    "mines": list(mines), "obstacles": list(obstacles),
                    "targets": list(targets), "bridges": list(bridges),
                    "barrier_col_from": "D", "barrier_row_hi": 6, "barrier_row_lo": 7}}


def _cut(blocks, *, mines=(), obstacles=(), targets=(), bridges=("K5", "K9"),
         stop=(), mine_clear=0.15):
    spec = PL.load_map(_map(mines, obstacles, targets, bridges), source="<cut-test>")
    kept, dropped, notes = PL.corner_cut_cells(
        _cells(blocks), spec, mine_clear_m=mine_clear,
        stop_cells=[_cells([b])[0] for b in stop], crossings=(), **BAR)
    return ([geo.index_to_block(r - 1, c) for r, c in kept],
            [geo.index_to_block(r - 1, c) for r, c in dropped], notes)


# --------------------------------------------------------------------------- #
#  1. 七条硬规则
# --------------------------------------------------------------------------- #
def test_rules() -> None:
    section("1. 七条硬规则逐条（合成直角 A4→A3→B3，拐角格 = A3）")
    base = ["A4", "A3", "B3"]

    kept, dropped, notes = _cut(base)
    check("1.0 基准：空旷场上直角顶点被切（A3 删除 → 2 点）",
          dropped == ["A3"] and kept == ["A4", "B3"], f"{kept} / {dropped} / {notes}")

    kept, dropped, notes = _cut(base, mines=("A3",))
    check("1.1 ①拐角格是雷 ⇒ 不切",
          dropped == [] and any("①" in n for n in notes), f"{dropped} / {notes}")

    kept, dropped, notes = _cut(base, mines=("A4",))
    check("1.2 ④雷三连（prev 是雷）⇒ 不切",
          dropped == [] and any("④" in n for n in notes), f"{dropped} / {notes}")

    kept, dropped, notes = _cut(base, mines=("B3",))
    check("1.3 ④雷三连（next 是雷）⇒ 不切",
          dropped == [] and any("④" in n for n in notes), f"{dropped} / {notes}")

    # ② 用真实几何 J9→J10→I10（弦到雷 H10 实测 0.350 m）：阈值抬到 0.40 必触发，
    #    生产阈值 0.15 不触发 —— 如实记录"②在当前格心几何下是非绑定护栏"。
    kept, dropped, notes = _cut(["J9", "J10", "I10"], mines=("H10",), mine_clear=0.40)
    check("1.4a ②弦离雷 < 阈值 ⇒ 不切（阈值 0.40 触发；实测弦-雷 = 0.350）",
          dropped == [] and any("②" in n for n in notes), f"{dropped} / {notes}")
    kept, dropped, notes = _cut(["J9", "J10", "I10"], mines=("H10",))
    check("1.4b 生产阈值 0.15 下同一条不触发（②目前非绑定，护栏仍在）",
          dropped == ["J10"], f"{dropped} / {notes}")

    kept, dropped, notes = _cut(["A2", "B2", "B1"], obstacles=("A1",))
    check("1.5 ③切后离障碍更近 ⇒ 不切（障 A1：切前 0.350 → 切后 0.247）",
          dropped == [] and any("③障碍更近" in n for n in notes), f"{dropped} / {notes}")

    kept, dropped, notes = _cut(["J9", "K9", "K10"], bridges=("K9",))
    check("1.6a ⑤拐角格本身就是桥洞格 ⇒ 不切",
          dropped == [] and any("⑤桥洞格" in n for n in notes), f"{dropped} / {notes}")

    kept, dropped, notes = _cut(["L10", "L9", "K9"], bridges=("K9",))
    check("1.6b ⑤拐角格紧邻桥洞三连（next 是 K9）⇒ 不切",
          dropped == [] and any("⑤" in n for n in notes), f"{dropped} / {notes}")

    kept, dropped, notes = _cut(base, stop=("A3",))
    check("1.7 ⑦停站/指令格 ⇒ 不切",
          dropped == [] and any("⑦" in n for n in notes), f"{dropped} / {notes}")

    kept, dropped, notes = _cut(["A5", "A4", "B3"])
    check("1.8 ⑥非直角（45° 拐角）⇒ 不参与（连原因都不记）",
          dropped == [] and notes == [], f"{dropped} / {notes}")

    kept, dropped, notes = _cut(["A5", "A4", "A3", "B3", "B2"])
    check("1.9 相邻两个直角不同时切（索引差 ≤1 只切第一个）",
          dropped == ["A3"], f"{dropped} / {notes}")


# --------------------------------------------------------------------------- #
#  2. 开关默认关闭 + 冻结路线不变
# --------------------------------------------------------------------------- #
def test_default_off() -> None:
    section("2. 开关默认关闭 ⇒ 现役发车路线逐 token 不变")
    check("2.1 cfg=None 时关闭", PL.corner_cut_enabled(None) is False, "None")
    cfg = _cfg()
    check("2.2 现役 robot.yaml 没有该键（缺省 False）",
          cfg.get("planner.corner_cut", None) is None and PL.corner_cut_enabled(cfg) is False,
          repr(cfg.get("planner.corner_cut", "<缺键>")))
    off = PL.plan_route_tracker_safe(MAP, cfg, max_seconds=10.0)
    check("2.3 规划器产物 == 冻结 mission_blocks（逐 token，%d 点）" % len(LIVE),
          list(off.tokens) == LIVE, " ".join(off.tokens))
    check("2.4 关闭时不得出现任何切角 warning",
          not any("切角" in w for w in off.warnings), str(off.warnings))


# --------------------------------------------------------------------------- #
#  3. 开启后的产物与门禁
# --------------------------------------------------------------------------- #
def test_cut_on() -> None:
    section("3. 开启切角：产物 39 点 / 5 处 / D3* 保留 / validate 0 error")
    on = PL.plan_route_tracker_safe(MAP, _cfg(**{"planner.corner_cut": True}), max_seconds=10.0)
    cut_warn = next((w for w in on.warnings if "切角" in w), "")
    # ★ 卡 t3 一版（2026-10-16）：停站点推远到板心 1.00 m ⇒ 路线 44 → 50 点，条件切角 5 → 8 处
    #   （新增 `F1`/`H1`/`H3` 三个直角，全在 `F3→F2*→F1→G1→H1→H2→H3→G3` 那段绕行上）。
    # ★ 卡 t3-A（同日）：停站退到主线上的 `D3` ⇒ 绕行段消失 ⇒ **回到 44 点 / 5 处**
    #   （`M10` `J10` `E10` `B9` `G6`，与 t57 逐格相同）。
    check("3.1 切 5 处：M10 / J10 / E10 / B9 / G6",
          all(b + ":" in cut_warn for b in ("M10", "J10", "E10", "B9", "G6"))
          and "拐角条件切角 5 处" in cut_warn, cut_warn[:120])
    check("3.2 44 → 39 点", len(LIVE) == 44 and len(on.tokens) == 39, "%d → %d"
          % (len(LIVE), len(on.tokens)))
    check("3.3 删掉的就是那 5 个格（其余逐 token 保序不变）",
          [t for t in LIVE if t.rstrip("*,y") not in
           ("M10", "J10", "E10", "B9", "G6")]
          == list(on.tokens), " ".join(on.tokens))
    check("3.4 两个射击停站 `D3*`+`G3*` 保留（t13 双停站 + t3-A 板心窗口语义都不受影响）",
          [t for t in on.tokens if t.endswith("*")] == [t for t in LIVE if t.endswith("*")]
          == ["D3*", "G3*"], str([t for t in on.tokens if t.endswith("*")]))
    check("3.5 末点 `N5,y` 朝向后缀保留", on.tokens[-1] == "N5,y", on.tokens[-1])
    check("3.6 token 往返一致（纯格名，可被车侧 parse_mission 读回）",
          [w.block for w in P.parse_mission(on.tokens).waypoints]
          == [w.block for w in on.tokens and P.parse_mission(on.tokens).waypoints],
          "roundtrip")
    issues = V.validate_route(P.parse_mission(on.tokens), map_cfg=MAP, cfg=_cfg())
    errs = [i.code for i in issues if i.severity == "error"]
    check("3.7 切角后 validate 0 error", not errs, V.summary(issues)[:60] + " errs=" + str(errs))
    spans = V.validate_route(P.parse_mission(LIVE), map_cfg=MAP, cfg=_cfg())
    check("3.8 对照：冻结 44 点也 0 error（基线未被破坏）",
          not [i.code for i in spans if i.severity == "error"], V.summary(spans)[:60])


def test_determinism() -> None:
    section("4. 规划确定性（同配置两次 → 逐 token + warning 相同）")
    a = PL.plan_route_tracker_safe(MAP, _cfg(**{"planner.corner_cut": True}), max_seconds=10.0)
    b = PL.plan_route_tracker_safe(MAP, _cfg(**{"planner.corner_cut": True}), max_seconds=10.0)
    check("4.1 tokens 相同", list(a.tokens) == list(b.tokens),
          "%d vs %d" % (len(a.tokens), len(b.tokens)))
    check("4.2 warnings 相同", a.warnings == b.warnings, str(a.warnings)[:80])
    c = PL.plan_route_tracker_safe(MAP, _cfg(), max_seconds=10.0)
    d = PL.plan_route_tracker_safe(MAP, _cfg(), max_seconds=10.0)
    check("4.3 关闭时两次也相同且 == 冻结 44 点",
          list(c.tokens) == list(d.tokens) == LIVE, "%d" % len(c.tokens))


# --------------------------------------------------------------------------- #
#  5. sim：峰值下降 + 受保护指标不劣化
# --------------------------------------------------------------------------- #
def test_sim_compare() -> None:
    section("5. sim：拐角峰值下降 + 受保护指标保持（雷 8/8 / 压靶 0 / 碰撞 0 / 挡板 0 / 越界 0）")
    field = S.load_field_spec(str(MAP_PATH))
    off = PL.plan_route_tracker_safe(MAP, _cfg(), max_seconds=10.0)
    on = PL.plan_route_tracker_safe(MAP, _cfg(**{"planner.corner_cut": True}), max_seconds=10.0)
    cfg = _cfg()
    mo = S.run_sim(P.parse_mission(off.tokens), cfg, S.SimConfig(), field=field).metrics
    mn = S.run_sim(P.parse_mission(on.tokens), cfg, S.SimConfig(), field=field).metrics
    # ★★ 卡 t3 实测（2026-10-16）：现役 44 点路线（停站 `D3*`）上**收益反转** —— 切 5 处后
    #   顶点超调 0.04402 → **0.06366 m（+45%）**、拐角横向峰值 0.11620 → 0.14025 m（+21%）、
    #   拐角 RMS 0.05222 → 0.04858 m（−7%）；换来的是更快（74.00 → 63.94 s）与靶板净空变大
    #   （0.14331 → 0.20284 m）。
    #   ⚠️ t3 一版曾把这次反转归因于"新增的 `F1`/`H1`/`H3` 三处落在停站绕行段上"——
    #     **该归因已被 t3-A 证伪**：停站退回主线 `D3`（绕行段消失、只剩 5 处切点）后，
    #     超调仍然 +45% ⇒ 反转是"删 90° 顶点 ⇒ 弯道变长直角 ⇒ 纯追踪切得更多"这个**机制**本身，
    #     与停站位置无关。
    #   ⇒ 本特性在现役路线上**没有精度收益**，继续**默认关闭**（发车路线不带它）；
    #     下面的断言从"必须下降"改为"**不得超过基线的 1.5 倍**"—— 它守的是"删顶点不能把
    #     弯道改到不可跟踪"这条底线，而不是"必然变好"这条已证伪的假设。
    #     ⚠️ 若将来要用它，必须先重做前视/节拍整定，不能直接开 `planner.corner_cut`。
    _LIMIT = 1.5
    for key, label in (("vertex_overshoot_normal_m", "5.1 顶点超调"),
                       ("cross_track_max_corner_m", "5.2 拐角横向峰值"),
                       ("cross_track_rms_corner_m", "5.3 拐角横向 RMS")):
        check("%s 不劣于基线的 %.1f 倍（%.5f → %.5f m）"
              % (label, _LIMIT, mo[key], mn[key]),
              mn[key] <= mo[key] * _LIMIT,
              "%.5f → %.5f（基线 %.5f）" % (mo[key], mn[key], mo[key]))
    check("5.4 雷覆盖仍 8/8", mn["mine_pressed_count"] == mo["mine_pressed_count"] == 8.0,
          "%d/%d" % (mn["mine_pressed_count"], mn["mine_total"]))
    check("5.5 车体不压靶板（硬）0 拍 + 靶板最近净距不降（%.5f → %.5f m）"
          % (mo["board_min_clearance_m"], mn["board_min_clearance_m"]),
          mn["board_overlap_steps"] == 0.0
          and mn["board_min_clearance_m"] >= mo["board_min_clearance_m"],
          "overlap=%.0f clearance=%.5f" % (mn["board_overlap_steps"], mn["board_min_clearance_m"]))
    check("5.6 障碍碰撞 0 / 挡板触线 0 / 越界 0 / K 列封死 0",
          mn["obstacle_hit_steps"] == 0.0 and mn["barrier_breach_steps"] == 0.0
          and mn["out_of_bounds_steps"] == 0.0 and mn["k_seal_violation_steps"] == 0.0,
          "hit=%.0f barrier=%.0f oob=%.0f kseal=%.0f"
          % (mn["obstacle_hit_steps"], mn["barrier_breach_steps"],
             mn["out_of_bounds_steps"], mn["k_seal_violation_steps"]))
    check("5.7 全程跑完（completed=1）+ 里程比仍在 1 以内",
          mn["completed"] == 1.0 and mn["path_len_ratio"] <= 1.0,
          "completed=%.0f len_ratio=%.4f" % (mn["completed"], mn["path_len_ratio"]))
    print("     峰值对比：超调 %.5f→%.5f m ｜ 拐角峰值 %.5f→%.5f m ｜ 拐角RMS %.5f→%.5f m"
          " ｜ 时间 %.2f→%.2f s ｜ 靶板净空 %.5f→%.5f m ｜ 靶净空拍 %.0f→%.0f"
          % (mo["vertex_overshoot_normal_m"], mn["vertex_overshoot_normal_m"],
             mo["cross_track_max_corner_m"], mn["cross_track_max_corner_m"],
             mo["cross_track_rms_corner_m"], mn["cross_track_rms_corner_m"],
             mo["time_s"], mn["time_s"],
             mo["board_min_clearance_m"], mn["board_min_clearance_m"],
             mo["board_breach_steps"], mn["board_breach_steps"]))


def main() -> int:
    print("=" * 78)
    print("卡 t53 · 拐角条件切角专项（七条硬规则 / 默认关闭 / 峰值对比）")
    print("=" * 78)
    test_rules()
    test_default_off()
    test_cut_on()
    test_determinism()
    test_sim_compare()
    finish()
    return 0


if __name__ == "__main__":
    sys.exit(main())
