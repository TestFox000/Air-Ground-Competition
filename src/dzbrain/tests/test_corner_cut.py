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
  ② **默认关闭** ⇒ 产物逐 token 不变（第六图重钉：整线无解 ⇒ "冻结现役路线"改钉在
     **B3 增量形态**上：开关缺省时产物 == B3 基线、无切角 warning）；
  ③ 开启后的产物：第六图 B3 形态上三处直角全被硬规则拦下（⑦停站 B4 / ③靶板 E4 /
     ②雷 E5）⇒ 0 处切角、逐 token 不变、validate 0 error；
     另加**合成图端到端正证**（K10 桥 + 7 雷 + D5/D6 靶板，tracker-safe 可解）：
     56 → 50 点、6 处切角、`B4*` 保留、validate 0 error —— 特性本身仍会切、规则放行时
     真的切；sim 两臂对照受保护指标不劣化。

★★ 第六图重钉（2026-10-24）：tracker-safe 对第六图整线结构性无解（K7 死锁）⇒
   本文件 §2~§5 的"现役路线两臂"全部改用 `from_cell="B3" + already_covered=全部 8 雷`
   的可行增量形态（15 token；详见 test_incremental_plan.py 文件头）。

跑：python src\dzbrain\tests\test_corner_cut.py
"""

from __future__ import annotations

import pathlib
import sys
from collections import Counter

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
#  2. 开关默认关闭 + 产物不变（第六图：整线无解 ⇒ B3 增量形态上钉"逐 token 不变"）
# --------------------------------------------------------------------------- #
MINES = list(SPEC.mines)


def _plan6(cfg, **kw):
    """第六图唯一可行形态：B3 + 全部雷已压（详见 test_incremental_plan.py）。"""
    return PL.plan_route_tracker_safe(MAP, cfg, max_seconds=30.0,
                                      from_cell="B3", already_covered=MINES, **kw)


def test_default_off() -> None:
    section("2. 开关默认关闭 ⇒ 产物逐 token 不变（第六图重钉：整线无解 → B3 增量基底）")
    check("2.1 cfg=None 时关闭", PL.corner_cut_enabled(None) is False, "None")
    cfg = _cfg()
    check("2.2 现役 robot.yaml 没有该键（缺省 False）",
          cfg.get("planner.corner_cut", None) is None and PL.corner_cut_enabled(cfg) is False,
          repr(cfg.get("planner.corner_cut", "<缺键>")))
    # ★★ 第六图重钉（2026-10-24）：默认整线（不带 from_cell）结构性无解（K7 死锁）——
    #   旧 2.3「规划器产物 == 冻结 mission_blocks」在第六图上无从谈起，改钉两件事：
    #   (a) 默认整线 = PlannerError + 归因（不因切角开关而变）；
    #   (b) "缺省关闭 ⇒ 产物不变"在可行形态（B3 增量）上保住：产物 == B3 基线逐 token，
    #       且无任何切角 warning。
    try:
        PL.plan_route_tracker_safe(MAP, cfg, max_seconds=10.0)
        e_msg = ""
    except PL.PlannerError as exc:
        e_msg = str(exc)
    check("2.3 默认整线 ⇒ PlannerError + 归因（第六图结构性死锁，与切角开关无关）",
          "无解" in e_msg and "归因" in e_msg, e_msg.splitlines()[0][:80] or "居然成功了")
    off = _plan6(cfg)
    base_tokens = [str(t) for t in off.tokens]
    check("2.4 缺省关闭 + B3 增量形态 ⇒ 产物 == B3 基线（15 token）且无任何切角 warning",
          len(base_tokens) == 15 and not any("切角" in w for w in off.warnings),
          "%d token / warnings=%s" % (len(base_tokens),
                                      [w for w in off.warnings if "切角" in w]))


# --------------------------------------------------------------------------- #
#  3. 开启后的产物与门禁（第六图：0 处可切 + 合成图端到端正证）
# --------------------------------------------------------------------------- #
def test_cut_on() -> None:
    section("3. 开启切角：第六图 B3 形态 0 处可切（三规则拦下）+ 合成图 56→50 端到端")
    on = _plan6(_cfg(**{"planner.corner_cut": True}))
    cut_warn = next((w for w in on.warnings if "切角" in w), "")
    # ---- 第六图真几何逐格证据：三处直角全被硬规则拦下（corner_cut_cells 直查）----
    cells6 = [str(t).rstrip("*,y") for t in on.tokens]
    kept, dropped, notes = PL.corner_cut_cells(
        _cells(cells6), SPEC, mine_clear_m=0.15,
        stop_cells=_cells(["B4"]), crossings=("K5",), **BAR)
    check("3.1 第六图 B3 形态三处直角全被拦：B4=⑦停站 / E4=③靶板更近 / E5=②雷更近"
          "（逐条原因钉死，规则真的在第六图几何上生效）",
          dropped == [] and notes == ["B4:⑦停站/指令格", "E4:③靶板更近（0.350→0.247）",
                                      "E5:②切后比切前更近雷（0.350→0.247）"],
          "dropped=%s notes=%s" % (dropped, notes))
    kept_ns, dropped_ns, _notes_ns = PL.corner_cut_cells(
        _cells(cells6), SPEC, mine_clear_m=0.15, stop_cells=(), crossings=("K5",), **BAR)
    check("3.2 反证：去掉⑦停站保护 ⇒ B4 立刻可切（⑦ 正是保护停站的那条规则）",
          [geo.index_to_block(r - 1, c) for r, c in dropped_ns] == ["B4"],
          "dropped=%s" % [geo.index_to_block(r - 1, c) for r, c in dropped_ns])
    check("3.3 端到端：corner_cut=True 的产物与关闭时**逐 token 相同**（0 处切角 ⇒ "
          "无切角 warning；开关只增不删，被拦下就原样放行）",
          list(on.tokens) == [t for t in _plan6(_cfg()).tokens] and cut_warn == "",
          "%d token / cut_warn=%r" % (len(on.tokens), cut_warn[:60]))
    check("3.4 射击停站 `B4*` 保留 + 末点 `N5,y` 朝向后缀保留",
          [t for t in on.tokens if str(t).endswith("*")] == ["B4*"]
          and on.tokens[-1] == "N5,y",
          "%s / %s" % ([t for t in on.tokens if str(t).endswith("*")], on.tokens[-1]))
    iss = V.validate_route(P.parse_mission(list(on.tokens)), map_cfg=MAP, cfg=_cfg(),
                           expect_halves=("upper",), already_covered=MINES)
    check("3.5 切角（0 处）后 validate 增量口径 0 error（只剩 R002 warn）",
          not [i.code for i in iss if i.level == "error"],
          V.summary(iss)[:60])
    spans = V.validate_route(P.parse_mission(LIVE), map_cfg=MAP, cfg=_cfg())
    check("3.6 对照：冻结现役 68 token 也 0 error（基线未被破坏）",
          not [i.code for i in spans if i.level == "error"], V.summary(spans)[:60])
    # ---- 合成图端到端正证：规则放行时特性真的会切（第六图整线无解 ⇒ 借合成图）----
    #   K10 桥（离隔板两行，无 K7 类死锁）+ 7 雷 + D5/D6 靶板 ⇒ tracker-safe 可解。
    #   ⚠️ 合成路线有重复格（G4/H4/I4/I5 各两访）⇒ 删除对照必须用**多重集差 + 保序
    #   子序列**判据（按名字的集合过滤会把两访格一次全删，误判）。
    syn = _map(mines=("B10", "E8", "I11", "A2", "F4", "H4", "G5"),
               targets=("D5", "D6"), bridges=("K5", "K10"))
    syn_off = PL.plan_route_tracker_safe(syn, _cfg(), max_seconds=20.0)
    syn_on = PL.plan_route_tracker_safe(syn, _cfg(**{"planner.corner_cut": True}),
                                        max_seconds=20.0)
    syn_cut = next((w for w in syn_on.warnings if "切角" in w), "")
    _off_b = [str(t).rstrip("*,y") for t in syn_off.tokens]
    _on_b = [str(t).rstrip("*,y") for t in syn_on.tokens]
    _diff = Counter(_off_b) - Counter(_on_b)
    _it = iter(_off_b)
    _subseq = all(b in _it for b in _on_b)          # 保序子序列（只删不换）
    check("3.7 合成图正证：切 6 处（F11/D10/C1/C3/I6/I5）、56 → 50 点、"
          "on 是 off 的保序删除（多重集差恰 = 6 个切点，各 1 次）",
          "拐角条件切角 6 处" in syn_cut
          and all(b + ":" in syn_cut for b in ("F11", "D10", "C1", "C3", "I6", "I5"))
          and len(_off_b) == 56 and len(_on_b) == 50 and _subseq
          and sorted(_diff.elements()) == ["C1", "C3", "D10", "F11", "I5", "I6"],
          "%d → %d / 多重集差=%s / 保序=%s" % (len(_off_b), len(_on_b),
                                              sorted(_diff.elements()), _subseq))
    iss_syn = V.validate_route(P.parse_mission(list(syn_on.tokens)), map_cfg=syn, cfg=_cfg())
    check("3.8 合成图切角后 validate 0 error（特性产物本身过门禁）",
          not [i.code for i in iss_syn if i.level == "error"], V.summary(iss_syn)[:60])


def test_determinism() -> None:
    section("4. 规划确定性（同配置两次 → 逐 token + warning 相同）")
    a = _plan6(_cfg(**{"planner.corner_cut": True}))
    b = _plan6(_cfg(**{"planner.corner_cut": True}))
    check("4.1 tokens 相同", list(a.tokens) == list(b.tokens),
          "%d vs %d" % (len(a.tokens), len(b.tokens)))
    check("4.2 warnings 相同", a.warnings == b.warnings, str(a.warnings)[:80])
    c = _plan6(_cfg())
    d = _plan6(_cfg())
    check("4.3 关闭时两次也相同且 == B3 基线 15 token",
          list(c.tokens) == list(d.tokens) and len(c.tokens) == 15, "%d" % len(c.tokens))


# --------------------------------------------------------------------------- #
#  5. sim：两臂对照 + 受保护指标不劣化（第六图 B3 形态，两臂同路线）
# --------------------------------------------------------------------------- #
def test_sim_compare() -> None:
    section("5. sim：B3 两臂对照（0 处切角 ⇒ 指标逐位相同）+ 受保护指标全保持")
    field = S.load_field_spec(str(MAP_PATH))
    off = _plan6(_cfg())
    on = _plan6(_cfg(**{"planner.corner_cut": True}))
    cfg = _cfg()
    mo = S.run_sim(P.parse_mission(off.tokens), cfg, S.SimConfig(), field=field).metrics
    mn = S.run_sim(P.parse_mission(on.tokens), cfg, S.SimConfig(), field=field).metrics
    # ★★ 第六图重钉（2026-10-24）：B3 形态 0 处可切（§3.1）⇒ 两臂是**同一条路线**，
    #   指标必须逐位相同 —— 本条守的是"开关只增不删：被规则拦下就原样放行、
    #   不许有隐藏副作用"。第六图前的旧断言（超调 1.5 倍上界 / 收益反转记录）随
    #   第五图 44 点整线两臂退役；特性真实收益的 sim 对照需等有可切路线时再钉。
    _SAME = ("completed", "path_len_ratio", "time_s", "vertex_overshoot_normal_m",
             "cross_track_max_corner_m", "cross_track_rms_corner_m",
             "mine_pressed_count", "board_overlap_steps", "board_min_clearance_m",
             "obstacle_hit_steps", "barrier_breach_steps", "out_of_bounds_steps",
             "k_seal_violation_steps", "final_error_m")
    check("5.1 两臂 14 项契约指标逐位相同（0 处切角 = 同一条路线，无隐藏副作用）",
          all(mo[k] == mn[k] for k in _SAME),
          " | ".join("%s: %s→%s" % (k, mo[k], mn[k]) for k in _SAME
                     if mo[k] != mn[k]) or "14/14 逐位相同")
    check("5.2 受保护指标全保持：雷覆盖不降（段内 1/8，另 7 颗在 already_covered 台账）"
          " / 压靶 0 / 碰撞 0 / 挡板 0 / 越界 0 / K 列封死 0",
          mn["mine_pressed_count"] == mo["mine_pressed_count"]
          and mn["board_overlap_steps"] == 0.0
          and mn["obstacle_hit_steps"] == 0.0 and mn["barrier_breach_steps"] == 0.0
          and mn["out_of_bounds_steps"] == 0.0 and mn["k_seal_violation_steps"] == 0.0,
          "mines=%.0f/%.0f overlap=%.0f hit=%.0f barrier=%.0f oob=%.0f kseal=%.0f"
          % (mn["mine_pressed_count"], mn["mine_total"], mn["board_overlap_steps"],
             mn["obstacle_hit_steps"], mn["barrier_breach_steps"],
             mn["out_of_bounds_steps"], mn["k_seal_violation_steps"]))
    check("5.3 全程跑完（completed）+ 里程比仍在 1 以内",
          bool(mn["completed"]) and mn["path_len_ratio"] <= 1.0,
          "completed=%s len_ratio=%.4f" % (mn["completed"], mn["path_len_ratio"]))
    print("     峰值对比：超调 %.5f→%.5f m ｜ 拐角峰值 %.5f→%.5f m ｜ 拐角RMS %.5f→%.5f m"
          " ｜ 时间 %.2f→%.2f s ｜ 靶板净空 %.5f→%.5f m"
          % (mo["vertex_overshoot_normal_m"], mn["vertex_overshoot_normal_m"],
             mo["cross_track_max_corner_m"], mn["cross_track_max_corner_m"],
             mo["cross_track_rms_corner_m"], mn["cross_track_rms_corner_m"],
             mo["time_s"], mn["time_s"],
             mo["board_min_clearance_m"], mn["board_min_clearance_m"]))


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
