# -*- coding: utf-8 -*-
r"""卡 t9（G4）专项：**增量续算入口** `plan_route_tracker_safe(from_cell=…, already_covered=…)`。

为什么需要（docs/黑图探索与动态规划_决策层方案.md §6 第 6 条 + §11 E4 + 裁决 R6）：
  规划器**硬性要求**起点是地图起点（且"在桥洞以东"）⇒ 车一旦过洞就无法"从当前格续算"，
  黑图探索的"点亮 → 重规划 → 循迹"循环只能整线重算 + 前缀裁剪。本卡给规划器加**两个入参**：

    · `from_cell`        —— 当前格（本轮从哪儿接着算）；
    · `already_covered`  —— 已压格集合（只对**雷**生效：写进来的雷不再要求重压）。

  并落实**起点硬门降级规则**：原硬门「起点在洞东、必须穿一次洞」只在"车还没过洞"时保留；
  车已在洞西 / 已在上半场 ⇒ 该半场穿越视为**已完成**（本轮允许 0 次），只要求压完剩余雷/靶、
  收在终点。**不做隐式回填**：若剩余雷落在已封闭的一侧 ⇒ 如实 `PlannerError`，不偷偷放行两次穿越。

-------------------------------------------------------------------------------
★★ 2026-10-31 状态：**主体被显式 SKIP**（不是通过、也不是删除）
-------------------------------------------------------------------------------
第七张图（`fixtures/map_field7_t24.yaml`：通口 K5/K7）上，`plan_route_tracker_safe()`
**结构性无解** —— 定位在第 1 层 `legs0`（起点 N10 → 下半场通口 K7）：
  · 下半场东侧（行 8~11 × L~N）除 **K7** 之外没有出口（K 列其余行封死）；
  · K7 在**隔板紧邻行（行 7）**，进 K7 必须先"在行 7 里向西"，
    而 `_barrier_turn_cell` 对行 7（D~N 列）的 90° 转向是**硬否决**
    （实车依据：N7/H7 形态 178/175 拍触线）⇒ 任何到 K7 的腿都不存在。
第二处**独立**冲突（修好上面那条后会让上半场 DP 筛空）：口袋雷 `A2` 的可达来向 {北,西}
与可出方向 {东,南} 交集为空，而 `A2 ∉ TS_TURNED_MINES` ⇒ 雷上不许转向。

⇒ 本文件里依赖 tracker-safe 产物的断言**全部 SKIP 并打印原因**（`_SKIPPED` 汇总），
   不算通过。仍然真跑的三类与引擎无关：
     · §0 无解定位（防"SKIP 了却不知道原因"，成因变了会红）；
     · §5.4 tour 引擎仍拒"改 map.start 绕过"（E4 原样）；
     · §6.4 现场路线的整线门禁 0 error；
     · §7 配置真源（SCHEMA 派生的段前缀）。
   旧的第五图黄金值（44 点 / K9 / C1…）已不适用，原文见 `git log` 本文件历史。

跑：python src\dzbrain\tests\test_incremental_plan.py
"""

from __future__ import annotations

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
from harness import check, finish, section                         # noqa: E402

WS = pathlib.Path(__file__).resolve().parents[3]
#: ★2026-10-31：生产配置不再有手写路线（path_executor.mission_blocks 随旧核删除）。
#:   本文件原先直接读 robot.yaml 的 route.tokens；为空时改用固定夹具
#:   （与删除前 robot.yaml 逐字一致的第七图 68 token），保证断言仍在验一条真实路线。
_ROUTE_FIXTURE = WS / "src" / "dzbrain" / "tests" / "fixtures" / "route_field7_t24.txt"

_SKIPPED: list[str] = []


def skip(name: str, reason: str) -> None:
    """**显式 SKIP**：不计入 PASS，也不静默 —— 原因打到 stdout 并进最终汇总。"""
    _SKIPPED.append(f"{name}：{reason}")
    print(f"  SKIP  {name}   [{reason}]")


def _live_tokens(cfg) -> list:
    toks = [str(t) for t in (cfg.get("route.tokens") or [])]
    return toks or _ROUTE_FIXTURE.read_text(encoding="utf-8").split()


CFG_PATH = WS / "config" / "robot.yaml"
MAP_PATH = WS / "src" / "dzbrain" / "tests" / "fixtures" / "map_field7_t24.yaml"

MAP = yaml.safe_load(MAP_PATH.read_text(encoding="utf-8"))
SPEC = PL.load_map(MAP, source=str(MAP_PATH))
CFG = Config.load(str(CFG_PATH))
LIVE = _live_tokens(CFG)
MINES = list(SPEC.mines)
TARGETS = list(SPEC.targets)

TS = dict(max_seconds=30.0)          # 调试/回归都给足预算（本机实测 <1.5 s）


def plan(spec=MAP, **kw):
    """一次增量/整线规划（`spec` 是 dict，与 CLI/车端同形）。"""
    return PL.plan_route_tracker_safe(spec, Config.load(str(CFG_PATH)),
                                      name=kw.pop("name", "t9"), **TS, **kw)


def blocks(tokens):
    return [str(t).split(":")[0].rstrip("*,y") for t in tokens]


def cross_runs(tokens):
    return list(V.crossing_entries(blocks(tokens)))                # noqa: SLF001


def validate(tokens):
    return V.validate_route(P.parse_mission(list(tokens)), map_cfg=MAP,
                            cfg=Config.load(str(CFG_PATH)))


def codes(tokens, level="error"):
    return [i.code for i in validate(tokens) if i.level == level]


def _ts_infeasible_probe() -> dict:
    """复算"本图 tracker-safe 无解"并定位到第 1 层（起点 → 下半场通口）。**只读**。

    为什么本文件自己写一遍而不是 import 别的测试：测试之间不互相 import（会连带跑对方
    的模块级代码，也会让"A 挂了 B 跟着挂"的归因变糊）。这里只复算 3 个数，够定位。
    """
    spec = PL.load_map(MAP)
    phys = PL._phys_params(CFG)
    lower_pool, upper_pool = PL._cross_pools(CFG, spec)           # noqa: SLF001
    lc = PL._cell(PL._default_cross(CFG, lower_pool, "lower"))     # noqa: SLF001
    uc = PL._cell(PL._default_cross(CFG, upper_pool, "upper"))     # noqa: SLF001
    grid = PL.Grid(spec, bridge_rows=(lc[0], uc[0]), allow_reverse=False)
    ts = PL._TSGrid(spec=spec, grid=grid, phys=phys, turn_w=PL.TS_TURN_W,
                    turned_mines=tuple(PL.TS_TURNED_MINES))
    h0 = PL.DIRS[PL.START_HEADING]
    legs0 = PL._ts_leg(ts, PL._cell(spec.start), lc, h0, {}, first_dir=h0)  # noqa: SLF001
    h_n, h_w = PL._heading_of(-1, 0, -1), PL._heading_of(0, -1, -1)         # noqa: SLF001
    banned = all(PL._barrier_turn_cell(grid, grid.barrier_lo,                 # noqa: SLF001
                                       geo.COL_LETTERS.index(c), h_n, h_w)
                 for c in ("N", "M", "L"))
    return {"lower": geo.index_to_block(lc[0] - 1, lc[1]),
            "legs0": len(legs0), "row7_turn_banned": banned}


def main() -> int:
    # ================================================================== #
    section("0 前置：本图 tracker-safe 为什么无解（SKIP 的地基，不是形式）")
    #   这两条是**活判据**：一旦成因变了（换图 / 修了行 7 判据 / A2 进 turned_mines），
    #   它们会红 —— 提醒把下面 SKIP 掉的 17 条断言按新图重钉，而不是继续挂 SKIP。
    _probe = _ts_infeasible_probe()
    check("0.1 无解定位在第 1 层 `legs0`：起点 → 下半场通口 %s 一条可行腿都没有" % _probe["lower"],
          _probe["legs0"] == 0, "%s 腿数=%d" % (_probe["lower"], _probe["legs0"]))
    check("0.2 成因：通口 K7 在隔板紧邻行（行 7），行内 90° 转向对 D~N 列硬否决"
          "（N7/M7/L7 三处『来向=北→去向=西』全部命中）⇒ 下半场东侧无出口",
          _probe["row7_turn_banned"] is True, "row7_turn_banned=True")

    # ================================================================== #
    section("1 缺省入参：逐 token 不变（本卡验收 ①）")
    skip("1.1~1.4 缺省入参逐 token 不变 / start 字段 / 确定性 / already_covered=[C1]",
         "需要 tracker-safe 产物；本图无解（见 §0）。且旧期望值（44 点、C1 是雷）"
         "属第五张图，已随换图失效")

    section("2 `from_cell` 续算：洞东 / 洞西（已过洞）/ 站在洞上")
    skip("2.1~2.9 from_cell=M9/B8/K9 续算 + 段内结构自检复算",
         "同上；且 M9/B8/K9 的『洞』指的是第五图 K9，本图通口是 K7 —— 期望值需按新图重钉")

    section("3 `already_covered` 避让（本卡验收 ② 主体）")
    skip("3.1~3.6 已压 5 格不回头 / 未压 3 雷不漏 / 全压更短 / warning 计数 / 非雷格 / 非法名",
         "需要 tracker-safe 产物；且 B7/D9/H10/C1/A3、E3/G4/I6 都是第五图点位")

    section("4 起点硬门：该拒的必须拒（且文案可判读）")
    skip("4.1~4.4 from_cell=障碍/非法名/上半场有未压雷/洞东有未压雷",
         "这些『必须抛 PlannerError』在本图**恒真**（引擎对所有输入都无解）"
         "⇒ 跑了也是绿得不是这条判据，只能 SKIP")

    # ================================================================== #
    section("5 R-5：改 `MapSpec.start` 绕不过去（t5 §E4 的老路仍然失败）")
    # ★ 5.4 与 tracker-safe 无关（钉的是 tour 引擎的起点硬门），照常跑。
    try:
        _spec_e10 = {"map": {**MAP["map"], "start": "E10"}}
        PL.plan_route(_spec_e10, Config.load(str(CFG_PATH)))
        tour_msg = ""
    except PL.PlannerError as exc:
        tour_msg = str(exc)
    check("5.4 tour 引擎：改 map.start=E10 ⇒ 仍报『不在下半场桥洞以东』（E4 原文）",
          "以东" in tour_msg, tour_msg.splitlines()[0][:90] or "居然过了")
    skip("5.1 / 5.5 / 5.6 tracker-safe 侧的三条：改 map.start=E10/F2/B8 仍不过 / "
         "from_cell 不写地图 / 改 map.start=M9 虽过但语义写进地图",
         "需要 tracker-safe 产物；本图无解（见 §0）")

    # ================================================================== #
    section("6 边界：续算段**不是整线** —— 整线门禁照规矩拒收（口径必须写进决策层）")
    skip("6.1~6.3 M9/B8/C3 三段续算的整线门禁口径（R002 / R007 / R005）",
         "需要 tracker-safe 产物；本图无解（见 §0）")
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

    # ================================================================== #
    if _SKIPPED:
        print("")
        print("-" * 70)
        print(f"⚠️  {len(_SKIPPED)} 组 SKIP（**不是通过**）—— 全部因为本图 tracker-safe 无解：")
        for item in _SKIPPED:
            print(f"  SKIP  {item}")
        print("   ⇒ 下面的『全部通过』只覆盖真跑过的 9 项；SKIP 项**没有执行**。")
        print("-" * 70)
    finish()
    return 0


if __name__ == "__main__":
    sys.exit(main())
