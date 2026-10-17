# -*- coding: utf-8 -*-
r"""卡 t9（G4）专项：**增量续算入口** `plan_route_tracker_safe(from_cell=…, already_covered=…)`。

为什么需要（docs/黑图探索与动态规划_决策层方案.md §6 第 6 条 + §11 E4 + 裁决 R6）：
  现役规划器**硬性要求**起点是地图起点（且"在桥洞以东"）⇒ 车一旦过洞就无法"从当前格续算"，
  黑图探索的"点亮 → 重规划 → 循迹"循环只能整线重算 + 前缀裁剪。本卡给规划器加**两个入参**：

    · `from_cell`        —— 当前格（本轮从哪儿接着算）；
    · `already_covered`  —— 已压格集合（只对**雷**生效：写进来的雷不再要求重压）。

  并落实**起点硬门降级规则**：原硬门「起点在洞东、必须穿一次洞」只在"车还没过洞"时保留；
  车已在洞西 / 已在上半场 ⇒ 该半场穿越视为**已完成**（本轮允许 0 次），只要求压完剩余雷/靶、
  收在终点。**不做隐式回填**：若剩余雷落在已封闭的一侧 ⇒ 如实 `PlannerError`，不偷偷放行两次穿越。

本文件钉住六组（本卡验收 ① ②）：
  ① **缺省逐 token 不变**：不带新入参时与 `robot.yaml` 的 44 点路线**逐 token 相同**；
  ② `from_cell` 续算：`M9`（洞东）/ `B8`（洞西=已过洞）/ `K9`（正站在洞上）；
  ③ `already_covered` 避让：已压雷从必访集里剔除、未压雷一个不许漏；
  ④ 起点硬门拒绝：障碍格 / 非法格名 / 剩余雷在对侧 / 非法 covered 格名；
  ⑤ **R-5**：改 `MapSpec.start` 绕过的老路**仍然失败**（E4 原样），且增量不改地图；
  ⑥ 边界：续算段**不是整线**——整线门禁（`validate_route` 的 R002/R005/R007）会照规矩拒收，
     本卡只保证"段内结构自检 + 语法/几何合法"，合成整线是决策层（t5c/t5e）的事。

跑：python src\dzbrain\tests\test_incremental_plan.py
"""

from __future__ import annotations

import copy
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
sys.stdout.reconfigure(errors="replace")

import yaml                                                        # noqa: E402

from dzbrain.core import geometry as geo                           # noqa: E402
from dzbrain.core.config import Config, SCHEMA                     # noqa: E402
from dzbrain.route import parser as P                              # noqa: E402
from dzbrain.route import planner as PL                            # noqa: E402
from dzbrain.route import validate as V                           # noqa: E402
from harness import check, check_raises, finish, section           # noqa: E402

WS = pathlib.Path(__file__).resolve().parents[3]
CFG_PATH = WS / "config" / "robot.yaml"
MAP_PATH = WS / "config" / "map.yaml"

MAP = yaml.safe_load(MAP_PATH.read_text(encoding="utf-8"))
SPEC = PL.load_map(MAP, source=str(MAP_PATH))
CFG = Config.load(str(CFG_PATH))
LIVE = [str(t) for t in (CFG.get("route.tokens") or [])]
MINES = list(SPEC.mines)
TARGETS = list(SPEC.targets)

TS = dict(max_seconds=30.0)          # 调试/回归都给足预算（本机实测 <1.5 s）


def plan(spec=MAP, **kw):
    """一次增量/整线规划（`spec` 是 dict，与 CLI/车端同形）。"""
    return PL.plan_route_tracker_safe(spec, Config.load(str(CFG_PATH)),
                                      name=kw.pop("name", "t9"), **TS, **kw)


def blocks(tokens):
    return [str(t).rstrip("*,y") for t in tokens]


def cross_runs(tokens):
    return list(V.crossing_entries(blocks(tokens)))                # noqa: SLF001


def validate(tokens):
    return V.validate_route(P.parse_mission(list(tokens)), map_cfg=MAP,
                            cfg=Config.load(str(CFG_PATH)))


def codes(tokens, level="error"):
    return [i.code for i in validate(tokens) if i.level == level]


def main() -> int:
    # ================================================================== #
    section("1 缺省入参：逐 token 不变（本卡验收 ①）")
    base = plan()
    check("1.1 不带新入参 ⇒ 与 robot.yaml 的 mission_blocks 逐 token 相同（44 点）",
          list(base.tokens) == LIVE and len(base.tokens) == 44,
          "%d token vs 落盘 %d" % (len(base.tokens), len(LIVE)))
    check("1.2 缺省产物 start = N10（地图起点），且**没有**『增量续算』warning",
          base.start == SPEC.start == "N10"
          and not any("增量续算" in w for w in base.warnings),
          "start=%s warnings=%d" % (base.start, len(base.warnings)))
    again = plan()
    check("1.3 缺省两次调用逐 token 相同（确定性未被入参默认值破坏）",
          list(again.tokens) == list(base.tokens), "%d token" % len(again.tokens))
    cov_on_route = plan(already_covered=["C1"])
    miss_c1 = [m for m in MINES if m not in blocks(cov_on_route.tokens)]
    check("1.4 只传 `already_covered=[C1]`（该雷在主线上）⇒ 起点/终点/门禁口径不变，"
          "且 C1 之外 7 颗雷仍全压（路线可变短，C1 只在'必访集'里才必须经过）",
          cov_on_route.start == "N10" and cov_on_route.tokens[-1].startswith("N5")
          and miss_c1 == ["C1"] and len(cov_on_route.tokens) <= len(LIVE),
          "%d token ≤ %d / 漏压 %s" % (len(cov_on_route.tokens), len(LIVE), miss_c1))

    # ================================================================== #
    section("2 `from_cell` 续算：洞东 / 洞西（已过洞）/ 站在洞上")
    r_m9 = plan(from_cell="M9")
    check("2.1 from_cell=M9（下半场洞东）⇒ 首点=M9、start 字段=M9、终点仍是 N5",
          r_m9.tokens[0] == "M9" and r_m9.start == "M9"
          and r_m9.tokens[-1].startswith("N5"),
          "首点=%s start=%s 末点=%s" % (r_m9.tokens[0], r_m9.start, r_m9.tokens[-1]))
    check("2.2 from_cell=M9：下半场仍要穿一次 K9（向西），故比整线短但仍走洞",
          "K9" in r_m9.tokens and len(r_m9.tokens) < len(LIVE),
          "%d token（整线 %d）" % (len(r_m9.tokens), len(LIVE)))
    r_b8 = plan(from_cell="B8")
    b8 = r_b8.tokens
    check("2.3 from_cell=B8（下半场洞西 ⇒ 已过洞）⇒ 本轮**不再穿 K9**",
          "K9" not in b8, "tokens 含 K9" if "K9" in b8 else "无 K9 ✅")
    check("2.4 from_cell=B8：上半场仍恰穿一次 K5，且方向**向东**（J5 -> K5 -> L5）",
          "K5" in b8 and b8.index("J5") < b8.index("K5") < b8.index("L5"),
          " ".join(b8[b8.index("J5") - 1:b8.index("L5") + 1]) if "K5" in b8 else "无 K5")
    miss = [m for m in MINES if m not in blocks(b8)]
    check("2.5 from_cell=B8：8 颗雷仍全部压过（续算不许漏）", not miss,
          "漏 " + " ".join(miss) if miss else "8/8")
    r_k9 = plan(from_cell="K9")
    check("2.6 from_cell=K9（正站在洞上）⇒ 视为已进洞，本轮不重复穿 K9 之外的下半场洞",
          r_k9.tokens[0] == "K9" and r_k9.tokens.count("K9") >= 1
          and r_k9.tokens[-1].startswith("N5"),
          "%d token / 首点=%s / K9 出现 %d 次"
          % (len(r_k9.tokens), r_k9.tokens[0], r_k9.tokens.count("K9")))
    check("2.7 续算产物带『增量续算』warning 且写明 from_cell 名（现场/日志可识别）",
          any("增量续算" in w and "M9" in w for w in r_m9.warnings)
          and any("增量续算" in w and "B8" in w for w in r_b8.warnings),
          str([w for w in r_m9.warnings if "增量" in w])[:90])

    # --- 段内结构自检（用与规划器同一套降级参数复算一遍，防"规划器自己放过自己"）---
    probs = PL.check_route_structure(
        P.parse_mission(list(r_b8.tokens)), SPEC,
        PL.Grid(SPEC, bridge_rows=(9, 5), allow_reverse=False), PL._limits(CFG),  # noqa: SLF001
        PL.shoot_distance_window(PL._shoot_accept(CFG)),                          # noqa: SLF001
        expected_start="B8",
        already_covered=(),
        crossing_halves=(False, True))
    check("2.8 B8 续算段独立复算结构自检：0 问题（起点放宽=B8 / 下半场穿越已完成）",
          not probs, "；".join(probs[:3]) if probs else "0 问题")
    probs_std = PL.check_route_structure(
        P.parse_mission(list(base.tokens)), SPEC,
        PL.Grid(SPEC, bridge_rows=(9, 5), allow_reverse=False), PL._limits(CFG),  # noqa: SLF001
        PL.shoot_distance_window(PL._shoot_accept(CFG)))                          # noqa: SLF001
    check("2.9 同一自检对**整线**（缺省参数）也是 0 问题（未因新入参放松底线）",
          not probs_std, "；".join(probs_std[:3]) if probs_std else "0 问题")

    # ================================================================== #
    section("3 `already_covered` 避让（本卡验收 ② 主体）")
    cov = ["B7", "D9", "H10", "C1", "A3"]
    r_c3 = plan(from_cell="C3", already_covered=cov)
    c3 = blocks(r_c3.tokens)
    check("3.1 已压的 5 格**一个都不出现在路线里**（不再绕回去补压）",
          not [c for c in cov if c in c3],
          "仍出现 " + " ".join(c for c in cov if c in c3) if [c for c in cov if c in c3] else "0 个")
    check("3.2 未压的 3 颗雷（E3 / G4 / I6）一个不漏", all(m in c3 for m in ("E3", "G4", "I6")),
          " ".join(m for m in ("E3", "G4", "I6") if m not in c3) or "3/3")
    r_all = plan(from_cell="C3", already_covered=MINES)
    check("3.3 全部雷都已压 ⇒ 续算更短（只压靶 + 穿洞 + 到终点）",
          len(r_all.tokens) < len(r_c3.tokens) and r_all.tokens[-1].startswith("N5"),
          "%d token < %d token" % (len(r_all.tokens), len(r_c3.tokens)))
    check("3.4 warning 里写明已压格数（现场可核对台账）",
          any("已压 5 格" in w for w in r_c3.warnings),
          str([w for w in r_c3.warnings if "增量" in w])[:100])
    r_nonmine = plan(already_covered=["M9"])
    check("3.5 `already_covered` 里塞**非雷格**（M9）不炸、也不影响雷必压集（仍 8/8）",
          not [m for m in MINES if m not in blocks(r_nonmine.tokens)]
          and list(r_nonmine.tokens) == LIVE,
          "%d token" % len(r_nonmine.tokens))
    check_raises("3.6 `already_covered` 里有非法格名 ⇒ 抛 PlannerError（不静默吞）",
                 PL.PlannerError, plan, already_covered=["Z9"])

    # ================================================================== #
    section("4 起点硬门：该拒的必须拒（且文案可判读）")
    def err_of(**kw):
        try:
            plan(**kw)
        except PL.PlannerError as exc:
            return str(exc)
        return ""

    e_obs = err_of(from_cell="E2")
    check("4.1 from_cell=障碍格（E2）⇒ PlannerError『不可通行』",
          "不可通行" in e_obs, e_obs[:80] or "没有抛")
    e_bad = err_of(from_cell="Z9")
    check("4.2 from_cell=非法格名（Z9）⇒ PlannerError『不是合法格名』",
          "不是合法格名" in e_bad, e_bad[:80] or "没有抛")
    e_low = err_of(from_cell="C3")
    check("4.3 from_cell 在上半场但**下半场还有未压雷** ⇒ 拒（不许靠再穿一次洞兜底）",
          "下半场仍有未压雷" in e_low, e_low[:110] or "没有抛")
    e_east = err_of(from_cell="N2", already_covered=["B7", "D9", "H10", "C1", "A3"])
    check("4.4 from_cell 在上半场**洞东**但上半场还有未压雷 ⇒ 拒（回去必再穿一次）",
          "已在上半场桥洞以东" in e_east, e_east[:110] or "没有抛")

    # ================================================================== #
    section("5 R-5：改 `MapSpec.start` 绕不过去（t5 §E4 的老路仍然失败）")
    def with_start(cell):
        spec2 = copy.deepcopy(MAP)
        spec2["map"]["start"] = cell
        return spec2

    for cell in ("E10", "F2", "B8"):
        try:
            plan(spec=with_start(cell))
            msg = ""
        except PL.PlannerError as exc:
            msg = str(exc)
        check("5.1 改 map.start=%s 后走 tracker-safe（不带 from_cell）⇒ 仍然不过" % cell,
              bool(msg) and ("无解" in msg or "起点" in msg), msg.splitlines()[0][:80] or "居然过了")
    try:
        PL.plan_route(with_start("E10"), Config.load(str(CFG_PATH)))
        tour_msg = ""
    except PL.PlannerError as exc:
        tour_msg = str(exc)
    check("5.4 tour 引擎：改 map.start=E10 ⇒ 仍报『不在下半场桥洞以东』（E4 原文）",
          "以东" in tour_msg, tour_msg.splitlines()[0][:90] or "居然过了")
    check("5.5 `from_cell` 路径**不写地图**：规划后 spec.start 仍是 N10，而产物 start=B8",
          MAP["map"]["start"] == "N10" and r_b8.start == "B8",
          "map.start=%s / res.start=%s" % (MAP["map"]["start"], r_b8.start))
    r_hack = plan(spec=with_start("M9"))
    check("5.6 对照：改 map.start=M9（洞东）虽能过，但**起点语义被写进地图**（res.start=M9）"
          "⇒ 这正是必须用 from_cell 的理由",
          r_hack.start == "M9" and MAP["map"]["start"] == "N10",
          "res.start=%s / 磁盘 spec.start=%s" % (r_hack.start, MAP["map"]["start"]))

    # ================================================================== #
    section("6 边界：续算段**不是整线** —— 整线门禁照规矩拒收（口径必须写进决策层）")
    check("6.1 M9 续算段：整线校验对『起点不是地图起点』只到 **warn**（R002），"
          "另有 0 error ⇒ 该段在整线口径下仍可读，但必须由决策层合成",
          "R002" in codes(r_m9.tokens, level="warn") and not codes(r_m9.tokens),
          "error=%s warn=%s" % (codes(r_m9.tokens), codes(r_m9.tokens, "warn")))
    check("6.2 B8 续算段：validate 报 R007（下半场 0 次穿越）—— 预期行为",
          "R007" in codes(r_b8.tokens), str(codes(r_b8.tokens)))
    check("6.3 C3+已压 5 雷：validate 报 R005（雷覆盖 3/8）—— 预期行为（台账里那 5 颗已压）",
          "R005" in codes(r_c3.tokens), str(codes(r_c3.tokens)))
    check("6.4 缺省整线路线仍是 0 error（门禁底线未被本卡动摇）",
          not codes(LIVE), str(codes(LIVE)))

    # ================================================================== #
    section("7 配置真源：SCHEMA 登记即生效（本卡验收 ④ 的防回归）")
    #   背景：`Config.validate()` 原来用一张**手抄**的段前缀白名单判断"未知配置段"，
    #   漏了 `planner.` / `vehicle.` / `field.` 三段 —— 而这 20 个键**早就在 SCHEMA 里**，
    #   于是 config_selfcheck 每次都白报 20 条"未知配置段"，把真告警淹掉。
    #   修法 = 段前缀由 SCHEMA 派生（登记即生效）；本段钉住"白报消失 + 真拼错仍被抓"。
    cfg_live = Config.load(str(CFG_PATH))
    cfg_live.validate()
    unknown = [i for i in cfg_live.issues if i.level == "warn" and "未知配置段" in i.msg]
    check("7.1 现役 robot.yaml 的 validate() 不再报『未知配置段』（旧：20 条）",
          not unknown,
          ("%d 条：%s" % (len(unknown), [i.key for i in unknown[:5]])) if unknown else "0 条")
    must = ("planner.engine", "planner.footprint_gate", "planner.tracker_max_seconds",
            "planner.auto_plan_enabled", "planner.map_path", "planner.fallback_to_tokens",
            "planner.allow_reverse", "planner.max_seconds", "planner.pivot_penalty",
            "planner.mine_turn_penalty", "planner.obstacle_margin_m",
            "vehicle.length_m", "vehicle.width_m", "vehicle.coverage_radius_m",
            "vehicle.ref_offset_m", "field.obstacle_radius_m", "field.mine_pad_m",
            "field.barrier_width_m", "field.barrier_cols", "field.bridge_entry_redundancy_m")
    missing = []
    unknown_keys = {i.key for i in unknown}
    for k in must:
        if k in unknown_keys:                    # 仍被判"未知配置段" ⇒ 没登记
            missing.append(k)
            continue
        try:
            cfg_live.get(k)                      # 能被 Config 正常读到（不抛 ConfigError）
        except Exception:                        # noqa: BLE001
            missing.append(k)
    check("7.2 这 20 个键逐条：不再被判『未知配置段』且能被 Config 正常读到",
          not missing, "有问题 " + " ".join(missing) if missing else "20/20")
    prefixes = {k.split(".", 1)[0] for k in SCHEMA if "." in k}
    check("7.3 段前缀由 SCHEMA 派生（新登记键自带它的段，不再依赖手抄表）",
          {"planner", "vehicle", "field", "localization"} <= prefixes, str(sorted(prefixes)))
    typo = Config({"planner.nope": 1.0}, source="t9-guard")
    typo.validate(strict=False)          # 只为看告警，不触发关键键缺失的 error
    hits = [i for i in typo.issues
            if i.key == "planner.nope" and ("未登记的配置键" in i.msg or "未知配置段" in i.msg)]
    check("7.4 真·拼错的键仍会被抓（`planner.nope` ⇒ 『未登记的配置键』warn；防'改宽了')",
          bool(hits), "%d issues / 命中 %d" % (len(typo.issues), len(hits)))
    from dzbrain.core.config import INTENTIONALLY_UNREGISTERED
    exempt = Config({"localization.yaw_trim_deg": -0.6}, source="t9-exempt")
    exempt.validate(strict=False)
    check("7.5 唯一豁免键 `localization.yaw_trim_deg` 既不报未知、也**不在** SCHEMA"
          "（±10° 护栏归 calib_backfill，见 test_calib_backfill 2.3）",
          "localization.yaw_trim_deg" in INTENTIONALLY_UNREGISTERED
          and "localization.yaw_trim_deg" not in SCHEMA
          and not [i for i in exempt.issues if i.key == "localization.yaw_trim_deg"],
          "豁免集合 %s" % sorted(INTENTIONALLY_UNREGISTERED))

    finish()
    return 0


if __name__ == "__main__":
    sys.exit(main())
