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

★★★ 第六图重钉（2026-10-24，整套重基底）：
  第六图（2026-10-17 换图）对 tracker-safe **结构性无解**——K7 桥洞北壁与隔板共线
  （洞在隔板相邻行，行 7∈(6,7)），行内 90° 进场被『隔板相邻行转向禁令』杀、进洞邻格
  又禁换行 ⇒ 凡 `need_lower=True`（还要求穿下半场洞）的规划**全部无解**，PlannerError
  消息带归因（"归因：…结构性死锁…"）。因此：
    · §1 缺省整线 / §2 下半场起点（M9/B8/K9）⇒ 钉「PlannerError + 归因」（老的无解断言口径）；
    · 可行的增量形态只剩**上半场起点 + already_covered 覆盖全部下半场雷**：
      `from_cell="B3" + already_covered=全部 8 雷` ⇒ 15 token 成功且过双门禁
      （结构自检 0 问题；validate 增量口径 `expect_halves=("upper",)` + `already_covered` 0 error）。
      原语义（不重压已压雷 / 未压雷一颗不漏 / 增量 warning / 非法格抛错 / 起点硬门拒绝）
      全部在这个形态上保住。

本文件钉住六组（本卡验收 ① ②）：
  ① **缺省整线无解**：不带新入参 ⇒ `PlannerError`（带归因），且带任意 `already_covered`
     也救不了（K7 进不去与压没压雷无关）；
  ② `from_cell` 续算：下半场（M9/B8/K9）⇒ 无解+归因；上半场（B3）+ 全压雷 ⇒ 成功；
  ③ `already_covered` 避让：已压雷从必访集里剔除、未压雷一个不许漏（漏了就无解，不悄悄丢雷）；
  ④ 起点硬门拒绝：障碍格 / 非法格名 / 剩余雷在对侧 / 非法 covered 格名；
  ⑤ **R-5**：改 `MapSpec.start` 绕过的老路**仍然失败**（E4 原样，第六图死得更透），且增量不改地图；
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
    # ★★ 第六图重钉（2026-10-24）：缺省整线（从 N10 出发、need_lower=True）
    #   结构性无解（K7 桥洞北壁与隔板共线）⇒ §1 从「逐 token == robot.yaml」改为
    #   「PlannerError + 归因」。原验收 ①（不带新入参 = 旧行为）在新图上的"旧行为"
    #   就是如实失败，语义保住：缺省路径一个字节都没改，只是地图换了。
    section("1 缺省入参：第六图整线结构性无解（本卡验收 ①，第六图重钉）")
    def err_of(**kw):
        try:
            plan(**kw)
        except PL.PlannerError as exc:
            return str(exc)
        return ""
    e0 = err_of()
    check("1.1 不带新入参 ⇒ PlannerError（tracker-safe 整线无解）",
          bool(e0), e0[:80] or "居然成功了")
    check("1.2 ★消息带归因：『归因：…结构性死锁…』且点名隔板相邻行（行 7）"
          "——消息缺归因也要红（现场靠它分辨'图死了'还是'参数死了'）",
          "归因" in e0 and "结构性死锁" in e0 and "行 7" in e0,
          e0[:120])
    e0b = err_of()
    check("1.3 缺省两次调用归因消息逐字相同（确定性未被入参默认值破坏）",
          e0 == e0b and bool(e0), "len=%d/%d" % (len(e0), len(e0b)))
    e_cov = err_of(already_covered=["I11"])
    check("1.4 只传 `already_covered=[I11]`（真雷）也救不了：K7 进不去与压没压雷无关，"
          "仍无解且带归因（不因 covered 而放松报文）",
          "无解" in e_cov and "归因" in e_cov, e_cov[:90] or "居然成功了")

    # ================================================================== #
    section("2 `from_cell` 续算：下半场无解+归因 / 上半场 B3 全压雷成功（第六图重钉）")
    # ---- 下半场起点：need_lower=True ⇒ K7 结构性死锁（§见文件头）----
    for cell, why in (("M9", "下半场洞东 ⇒ 仍要求穿 K7"),
                      ("B8", "下半场洞西 ⇒ 东侧雷 D11/I11 已封死，绕不回去"),
                      ("K9", "站在 K 列下半场 ⇒ 同样被 K7 死锁咬住")):
        e_c = err_of(from_cell=cell)
        check("2.1 from_cell=%s（%s）⇒ PlannerError + 归因（第六图结构性死锁）" % (cell, why),
              "无解" in e_c and "归因" in e_c and "结构性死锁" in e_c,
              e_c[:80] or "居然成功了")
    # ---- 上半场起点：唯一可行形态 = B3 + 全部雷已压 ----
    r_b3 = plan(from_cell="B3", already_covered=MINES)
    b3 = blocks(r_b3.tokens)
    check("2.2 from_cell=B3+全压雷 ⇒ 首点=B3、start 字段=B3、终点仍是 N5（15 token）",
          r_b3.tokens[0] == "B3" and r_b3.start == "B3"
          and r_b3.tokens[-1].startswith("N5") and len(r_b3.tokens) == 15,
          "首点=%s start=%s 末点=%s %d token"
          % (r_b3.tokens[0], r_b3.start, r_b3.tokens[-1], len(r_b3.tokens)))
    check("2.3 from_cell=B3：下半场**不再穿 K7**（已过洞），上半场恰穿一次 K5 且方向向东"
          "（J5 -> K5 -> L5）",
          "K7" not in b3 and b3.count("K5") == 1
          and b3.index("J5") < b3.index("K5") < b3.index("L5"),
          " ".join(b3[b3.index("K5") - 1:b3.index("L5") + 1]))
    check("2.4 续算产物带『增量续算』warning 且写明 from_cell 名 + 跳过下半场穿洞"
          "（现场/日志可识别）",
          any("增量续算" in w and "B3" in w and "跳过穿洞=下半场" in w
              for w in r_b3.warnings),
              str([w for w in r_b3.warnings if "增量" in w])[:100])

    # --- 段内结构自检（用与规划器同一套降级参数复算一遍，防"规划器自己放过自己"）---
    probs = PL.check_route_structure(
        P.parse_mission(list(r_b3.tokens)), SPEC,
        PL.Grid(SPEC, bridge_rows=(7, 5), allow_reverse=False), PL._limits(CFG),  # noqa: SLF001
        PL.shoot_distance_window(PL._shoot_accept(CFG)),                          # noqa: SLF001
        expected_start="B3",
        already_covered=MINES,
        crossing_halves=(False, True))
    check("2.5 B3 续算段独立复算结构自检：0 问题（起点放宽=B3 / 下半场穿越已完成）",
          not probs, "；".join(probs[:3]) if probs else "0 问题")
    probs_live = PL.check_route_structure(
        P.parse_mission(LIVE), SPEC,
        PL.Grid(SPEC, bridge_rows=(7, 5), allow_reverse=False), PL._limits(CFG),  # noqa: SLF001
        PL.shoot_distance_window(PL._shoot_accept(CFG)))                          # noqa: SLF001
    check("2.6 同一自检对**整线现役 68 token**也是 0 问题（未因新入参放松底线）",
          not probs_live, "；".join(probs_live[:3]) if probs_live else "0 问题")

    # ================================================================== #
    section("3 `already_covered` 避让（本卡验收 ② 主体，第六图重钉到 B3 形态）")
    off_path = [m for m in MINES if m != "G5"]      # 不在 B3→B4*→…→K5→N5 主线上的 7 颗
    check("3.1 已压的 7 颗离线雷**一个都不出现在路线里**（不再绕回去补压；"
          "G5 是唯一例外——它恰在第 5 行主线上，属顺路而非绕行）",
          not [c for c in off_path if c in b3] and "G5" in b3,
          "仍出现 " + " ".join(c for c in off_path if c in b3)
          if [c for c in off_path if c in b3] else "7 颗离线雷 0 出现 / G5 顺路")
    rem_g5 = ["E8", "B10", "D11", "I11", "A2", "F4", "H4"]     # = MINES − G5
    r_g5 = plan(from_cell="B3", already_covered=rem_g5)
    check("3.2 未压的雷一颗不许漏：只留 G5 未压 ⇒ G5 真的压过（第 5 行主线）",
          "G5" in blocks(r_g5.tokens) and r_g5.tokens[-1].startswith("N5"),
          "%d token / 含 G5=%s" % (len(r_g5.tokens),
                                   "G5" in blocks(r_g5.tokens)))
    e_miss = err_of(from_cell="B3", already_covered=["E8", "B10", "D11", "I11"])
    check("3.3 ★少压了 A2/F4/H4 任一颗 ⇒ 无解+归因（不许悄悄丢雷：上半场口袋雷从 B3 "
          "结构上收不进来，如实失败）",
          "无解" in e_miss and "归因" in e_miss, e_miss[:90] or "居然成功了")
    check("3.4 全压 ⇒ 续算段只干三件事（压靶停站 B4* / 穿一次 K5 / 收终点 N5），"
          "比整线 68 token 短得多（15 << 68；同口径 C3=16 / B8=20，起点离靶越远越长）",
          len(r_b3.tokens) == 15 < len(LIVE), "%d vs 整线 %d" % (len(r_b3.tokens), len(LIVE)))
    check("3.5 warning 里写明已压格数（现场可核对台账：7 格 / 8 格两档）",
          any("已压 7 格" in w for w in r_g5.warnings)
          and any("已压 8 格" in w for w in r_b3.warnings),
          str([w for w in r_g5.warnings if "增量" in w])[:100])
    r_nonmine = plan(from_cell="B3", already_covered=MINES + ["M9"])
    check("3.6 `already_covered` 里塞**非雷格**（M9）不炸、也不影响雷必压集（token 逐位同）",
          list(r_nonmine.tokens) == list(r_b3.tokens),
          "%d token / 与 3.2 基线%s" % (len(r_nonmine.tokens),
                                       "逐位相同" if list(r_nonmine.tokens) == list(r_b3.tokens) else "不同"))
    check_raises("3.7 `already_covered` 里有非法格名 ⇒ 抛 PlannerError（不静默吞）",
                 PL.PlannerError, plan, from_cell="B3",
                 already_covered=MINES + ["Z9"])

    # ================================================================== #
    section("4 起点硬门：该拒的必须拒（且文案可判读；障碍格第六图用 F3）")
    e_obs = err_of(from_cell="F3")
    check("4.1 from_cell=障碍格（F3）⇒ PlannerError『不可通行』",
          "不可通行" in e_obs, e_obs[:80] or "没有抛")
    e_bad = err_of(from_cell="Z9")
    check("4.2 from_cell=非法格名（Z9）⇒ PlannerError『不是合法格名』",
          "不是合法格名" in e_bad, e_bad[:80] or "没有抛")
    e_low = err_of(from_cell="C3")
    check("4.3 from_cell 在上半场但**下半场还有未压雷** ⇒ 拒（不许靠再穿一次洞兜底）",
          "已在上半场" in e_low and "下半场仍有未压雷" in e_low, e_low[:110] or "没有抛")
    e_east = err_of(from_cell="N2",
                    already_covered=["E8", "B10", "D11", "I11"])
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
    check("5.2 tour 引擎：改 map.start=E10 ⇒ 仍报『不在下半场桥洞以东』（E4 原文）",
          "以东" in tour_msg, tour_msg.splitlines()[0][:90] or "居然过了")
    check("5.3 `from_cell` 路径**不写地图**：规划后 spec.start 仍是 N10，而产物 start=B3",
          MAP["map"]["start"] == "N10" and r_b3.start == "B3",
          "map.start=%s / res.start=%s" % (MAP["map"]["start"], r_b3.start))
    e_hack = err_of(spec=with_start("M9"))
    check("5.4 对照：改 map.start=M9（洞东）在第六图**连 E4 老路也过不去**"
          "（整线无解+归因，见 §1）——双保险都拦，更没有『起点语义被写进地图』的风险",
          "无解" in e_hack and "归因" in e_hack,
          e_hack.splitlines()[0][:80] or "居然过了")

    # ================================================================== #
    section("6 边界：续算段**不是整线** —— 整线门禁照规矩拒收（口径必须写进决策层）")
    check("6.1 B3 续算段：整线校验对『起点不是地图起点』只到 **warn**（R002），"
          "另如实报 R005（雷覆盖 1/8）+ R007（下半场 0 次穿越）——该段在整线口径下必须拒收，"
          "由决策层合成",
          "R002" in codes(r_b3.tokens, level="warn")
          and set(codes(r_b3.tokens)) >= {"R005", "R007"},
          "error=%s warn=%s" % (codes(r_b3.tokens), codes(r_b3.tokens, "warn")))
    inc_issues = V.validate_route(P.parse_mission(list(r_b3.tokens)),           # noqa: SLF001
                                  map_cfg=MAP, cfg=Config.load(str(CFG_PATH)),
                                  expect_halves=("upper",), already_covered=MINES)
    inc_err = [i.code for i in inc_issues if i.level == "error"]
    check("6.2 B3 续算段换**增量口径**（expect_halves=upper + already_covered）：0 error"
          "（只剩 R002 warn）——决策层合成时按此口径放行（t12 三处同契约）",
          not inc_err and "R002" in [i.code for i in inc_issues if i.level == "warn"],
          "error=%s warn=%s" % (inc_err, [i.code for i in inc_issues if i.level == "warn"]))
    check("6.3 缺省整线路线（robot.yaml 68 token）仍是 0 error（门禁底线未被本卡动摇）",
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
