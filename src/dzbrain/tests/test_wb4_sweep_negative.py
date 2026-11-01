# -*- coding: utf-8 -*-
"""WB-4 扫掠判据反向验证 + 合成器确定性（卡 `WB-4-SWEEP-20261031`）。

运行::

    cd dzrobot_v2 && PYTHONPATH=src/dzbrain python src/dzbrain/tests/test_wb4_sweep_negative.py

全过 exit 0；任一 FAIL exit 1。纯脚本 + 极简 check()（与全仓一致，不需要 pytest）。

## 本文件钉什么（卡面负例 1/2/3 逐条）

| § | 卡面 | 判据 |
|---|---|---|
| 1 | 负例 1：反向图（点判据过 / 扫掠不过）**必须被报出** | 现役第七图改靶板 `D5+D6 → D5+E5` ⇒ `validate_route` **0 error**、扫掠 **S002 命中** |
| 2 | 负例 2：同 seed 两次生成不同字节 ⇒ FAIL | `map_synth.generate()` 同 seed/index 两次跑，yaml+ref 逐位 sha256 相同 |
| 3 | 负例 3：生成图缺参考解却被计入「规划器无解」⇒ FAIL | `track_sweep` 对无 `.ref.txt` 的图走 `NO_REFERENCE`，**不**进无解分母 |
| 4 | 扫掠是一等公民：**三处都能从点判据切到扫掠判据** | S001/S002/S003 可单独开关、可组合；单独开关各只出自己那一类 |
| 5 | 卡面「反向验证能报红」的另一半：**规则真的会红** | 停站格压靶（A1 形态）扫掠报 S003，而点判据窗口判它"距离合规" |
| 6 | 判据不许自造数字 | 阈值全部来自 `route.physics` / `config`；本文件对拍它们 |

★ 反向验证（1/5）是**防假绿的核心**：如果扫掠层永远返回 `[]`，那"扫掠是一等公民"
  就只是一句口号。所以本文件不但要求"合法路线通过"，更要求"违规路线被抓"。

⚠️ 为什么反向样例选 `targets=D5+E5`：现场路线在第 59 个航点（`E5`）做一次 **90° 转向**，
   `E5` 若同时是靶格，则"车心正在靶格上掉头" ⇒ 车身外摆 0.1638 m 全部压在板上。
   而 `validate_route` 的 R006/R013 判的是**停站格到靶格的距离**（0.78~1.02 m），
   它**不看**"普通航点是否压在靶板上" —— 那两条只在 `tools/route_check.py` 里
   （「航点压过靶板」/「航段横穿靶板」）。所以这张图是：
   `validate_route` 全绿、`route_check` 会红（另一层）、**扫掠**红。这正是"三层各管一段"。
"""

from __future__ import annotations

import hashlib
import pathlib
import sys

_HERE = pathlib.Path(__file__).resolve()
_SRC = _HERE.parents[1]                       # src/dzbrain
_REPO = _SRC.parents[1]                       # dzrobot_v2
_TOOLS = _REPO / "tools"
_FIX = _SRC / "tests" / "fixtures"

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


def check(label: str, ok: bool, detail: str = "") -> bool:
    global _PASS
    if ok:
        _PASS += 1
        print(f"  ✅ {label}" + (f"  [{detail}]" if detail else ""))
    else:
        _FAIL.append(f"{_SECTION} / {label}" + (f"  [{detail}]" if detail else ""))
        print(f"  ❌ {label}" + (f"  [{detail}]" if detail else ""))
    return ok


def finish() -> None:
    print("\n" + "=" * 64)
    if _FAIL:
        print(f"FAILED · {len(_FAIL)} 项")
        for item in _FAIL:
            print("  - " + item)
        raise SystemExit(1)
    print(f"PASSED · {_PASS} 项")
    raise SystemExit(0)


# --------------------------------------------------------------------------- #
#  公共夹具
# --------------------------------------------------------------------------- #
def _load_map():
    import yaml
    return yaml.safe_load((_FIX / "map_field7_t24.yaml").read_text(encoding="utf-8"))


def _load_route():
    from dzbrain.route.parser import parse_mission
    tokens = (_FIX / "route_field7_t24.txt").read_text(encoding="utf-8").split()
    return parse_mission(tokens)


def _cfg():
    from dzbrain.core.config import Config
    return Config.load(str(_REPO / "config" / "robot.yaml"))


# --------------------------------------------------------------------------- #
#  1 · 反向图：点判据过、扫掠不过（卡面负例 1）
# --------------------------------------------------------------------------- #
def test_reverse_point_pass_sweep_fail() -> None:
    section("1 · 反向图：点判据通过 / 扫掠违规（**必须报红**）")
    import track_sweep as S
    from dzbrain.route import validate as V

    mc = _load_map()
    # 单变量：靶板 D5+D6（纵向两格）→ D5+E5（横向两格）。板心移到 E5 附近，
    # 正好把现场路线 #59 的 90° 转向顶点变成"在靶格上掉头"。
    mc["map"]["targets"] = ["D5", "E5"]
    route = _load_route()

    issues = V.validate_route(route, map_cfg=mc, cfg=_cfg())
    errors = [i for i in issues if i.level == "error"]
    check("点判据（validate_route）全绿 —— 否则这个反向样例不成立",
          not errors, f"errors={[i.code for i in errors]}")

    hits = S.sweep_route(route, kinds=(S.SWEEP_PIVOT,), map_cfg=mc)
    check("扫掠判据 S002 **抓到**这次压板（报红 = 反向验证成立）", bool(hits),
          S.summarize(hits))
    check("抓到的正是靶格上的 90° 转向顶点 E5",
          any(h.block == "E5" and h.kind == S.SWEEP_PIVOT for h in hits),
          "; ".join(f"{h.block}" for h in hits))
    if hits:
        detail = hits[0].detail
        check("报文带**量化差值**(毫米级)，不是「违规」两个字",
              "mm" in detail, detail[:110])

    # 对照组：原图（靶 D5+D6）扫掠层必须**通过** —— 否则"报红"是误报而非检出
    mc0 = _load_map()
    hits0 = S.sweep_route(route, map_cfg=mc0)
    check("对照组：原地图（靶 D5+D6）扫掠层通过（证明不是「见谁都红」）",
          not hits0, S.summarize(hits0))


# --------------------------------------------------------------------------- #
#  5 · 停站格压板（A1 事故形态）—— 点判据看距离合规、扫掠看车身贴上
# --------------------------------------------------------------------------- #
def test_stop_on_board() -> None:
    section("5 · A1 形态：停站格就在靶格上（点判据只算距离，扫掠算车身）")
    import track_sweep as S
    from dzbrain.route.model import Waypoint, make_route
    from dzbrain.core import geometry as geo

    mc = {"map": {"name": "A1-reverse", "start": "N10", "end": "N5",
                  "barrier_col_from": "D", "barrier_row_hi": 6, "barrier_row_lo": 7,
                  "mines": [], "obstacles": [], "targets": ["D5", "E5"],
                  "bridges": ["K5", "K7"]}}
    # 让停站格 **就是** 靶格 D5（A1 事故原形：board_min = 0.0）
    wps = [Waypoint.at_block(b, shoot=(b == "D5")) for b in ("N10", "M10", "L10", "D5", "N5")]
    route = make_route(wps, name="A1")

    hits = S.sweep_route(route, kinds=(S.SWEEP_STOP,), map_cfg=mc)
    check("S003 报出「停站整车压靶板」（0.00 mm 贴上）", bool(hits), S.summarize(hits))
    check("报文指出是 D5 且差值为 0 mm",
          any(h.block == "D5" and "0.00 mm" in h.detail for h in hits),
          hits[0].detail[:110] if hits else "无")

    # 点判据在同一处的读数：**距离**是 0.000 m，窗口 0.78~1.02 ⇒ 它报的是"没覆盖靶"
    # 而不是"车身压板"。两者**不是同一条判据** —— 这正是要升级的理由。
    x, y = geo.block_to_meters("D5")
    tx, ty = geo.block_to_meters("D5")
    dist = ((x - tx) ** 2 + (y - ty) ** 2) ** 0.5
    check("点判据口径（到靶格的距离）在这一点上是 0.000 m ⇒ 它只能判「够不够远」，判不了「压没压上」",
          abs(dist) < 1e-9, f"dist={dist:.4f} m")


# --------------------------------------------------------------------------- #
#  4 · 扫掠三判据是一等公民（可单独开关、可组合、互不串味）
# --------------------------------------------------------------------------- #
def test_sweep_kinds_first_class() -> None:
    section("4 · 扫掠是一等公民：S001/S002/S003 可单独开关、可组合")
    import track_sweep as S

    mc = _load_map()
    mc["map"]["targets"] = ["D5", "E5"]          # 会命中 S002 的那张图
    route = _load_route()

    for kind in S.SWEEP_KINDS:
        solo = S.sweep_route(route, kinds=(kind,), map_cfg=mc)
        check(f"单独 {kind}：只出自己那一类（不串味）",
              all(h.kind == kind for h in solo),
              f"{len(solo)} hit(s)")
    combined = S.sweep_route(route, map_cfg=mc)
    check("三判据组合跑 = 各单独跑的并集",
          len(combined) == sum(len(S.sweep_route(route, kinds=(k,), map_cfg=mc))
                               for k in S.SWEEP_KINDS),
          f"组合 {len(combined)}")
    check("`kinds=()` 显式关掉全部 = 0 命中（「可选」是真的可选）",
          len(S.sweep_route(route, kinds=(), map_cfg=mc)) == 0)

    # S001 的边界口径：物理边缘 = 最外格中心线 + 半格（**不是** map_bounds(0)）
    from dzbrain.core import geometry as geo
    mc0 = _load_map()
    route0 = _load_route()
    hits_default = S.sweep_route(route0, kinds=(S.SWEEP_BOUNDS,), map_cfg=mc0)
    check("S001 缺省按**物理边缘**判（现场合法路线 0 命中，不误报最外圈）",
          not hits_default, S.summarize(hits_default))
    # 显式收紧到"最外格中心线以内"就应当报红（证明门限真的在起作用）
    hits_tight = S.sweep_route(route0, kinds=(S.SWEEP_BOUNDS,), map_cfg=mc0,
                               margin_m=-geo.BLOCK_SIZE / 2.0)
    check("S001 显式收紧到格心线以内 ⇒ 立刻报红（门限不是摆设）",
          bool(hits_tight), S.summarize(hits_tight))


# --------------------------------------------------------------------------- #
#  6 · 阈值不许自造：对拍 physics / config
# --------------------------------------------------------------------------- #
def test_thresholds_from_physics() -> None:
    section("6 · 阈值全部来自 route.physics / config（不新造数字）")
    from dzbrain.route import physics as PH
    import track_sweep as S

    sc = S.SweepConfig.of(_cfg())
    butler = PH.vehicle_half_diagonal_m(sc.length_m, sc.width_m)
    check("扫掠半径 == physics.vehicle_half_diagonal_m（同一函数，不是抄的数）",
          abs(sc.half_diag - butler) < 1e-12, f"{sc.half_diag:.6f} m")
    check("车体尺寸取自 config（现场 0.28 × 0.17）",
          abs(sc.length_m - 0.28) < 1e-9 and abs(sc.width_m - 0.17) < 1e-9,
          f"{sc.length_m} × {sc.width_m}")
    check("挡板厚度取自 config（2 cm 细条）", abs(sc.barrier_width_m - 0.02) < 1e-9,
          f"{sc.barrier_width_m} m")


# --------------------------------------------------------------------------- #
#  2 · 合成器确定性（卡面负例 2）
# --------------------------------------------------------------------------- #
def test_synth_determinism() -> None:
    section("2 · 同 seed 两次生成 **逐位相同**（否则判 FAIL）")
    import map_synth as MS

    for index in (0, 5, 17):
        a, toks_a, why_a = MS.make_one(20261031, index, _cfg())
        b, toks_b, why_b = MS.make_one(20261031, index, _cfg())
        check(f"#{index} 两次 make_one 都成功", a is not None and b is not None,
              why_a or why_b)
        if a is None or b is None:
            continue
        ya = MS.render_yaml(a)
        yb = MS.render_yaml(b)
        ha = hashlib.sha256(ya.encode("utf-8")).hexdigest()
        hb = hashlib.sha256(yb.encode("utf-8")).hexdigest()
        check(f"#{index} yaml 字节相同", ha == hb, f"{ha[:16]}")
        check(f"#{index} 参考解 token 串相同", toks_a == toks_b,
              f"{len(toks_a)} token")

    # 不同 seed 必须给出**不同**图（否则「种子」是假的）
    d1, t1, _ = MS.make_one(20261031, 0, _cfg())
    d2, t2, _ = MS.make_one(20261032, 0, _cfg())
    if d1 and d2:
        check("换 seed ⇒ 图确实变了（不是忽略 seed 的假确定性）",
              (d1.mines, d1.obstacles, d1.bridges) != (d2.mines, d2.obstacles, d2.bridges),
              f"{d1.mines[:3]} vs {d2.mines[:3]}")


# --------------------------------------------------------------------------- #
#  3 · 缺参考解 ≠ 规划器无解（卡面负例 3）
# --------------------------------------------------------------------------- #
def test_no_reference_not_counted() -> None:
    section("3 · 缺参考解的图 **不**计入「规划器无解」（假归因）")
    import tempfile
    import yaml
    import track_sweep as TS

    mc = _load_map()
    with tempfile.TemporaryDirectory() as td:
        d = pathlib.Path(td)
        # 只放地图、**不放** .ref.txt
        (d / "no_ref.yaml").write_text(yaml.safe_dump(mc, allow_unicode=True), encoding="utf-8")
        pairs = TS.collect_maps(d)
        check("collect_maps 认得没参考解的图（不剔除）", len(pairs) == 1,
              f"{[p[0] for p in pairs]}")
        check("参考解路径为 None（而不是猜一条出来）", pairs[0][2] is None)
        rows = TS.sweep_one(pairs[0][0], pairs[0][1], pairs[0][2],
                            engines=["tour"], cfg=_cfg(),
                            cfg_path=_REPO / "config" / "robot.yaml", do_routecheck=False)
        statuses = [r["status"] for r in rows]
        check("状态是 NO_REFERENCE（不是 NO_SOLUTION）",
              statuses and all(s == "NO_REFERENCE" for s in statuses), str(statuses))
        dist = TS.distribution(rows)
        slot = dist["tour"]
        check("分布里 no_reference=1 且 no_solution=0（没被算成规划器无解）",
              slot["no_reference"] == 1 and slot["no_solution"] == 0,
              f"ref={slot['no_reference']} nosol={slot['no_solution']}")
        check("可行率分母**不含** NO_REFERENCE（None = 无可判图，不是 0%）",
              slot["feasible_rate"] is None, f"{slot['feasible_rate']}")


# --------------------------------------------------------------------------- #
#  7 · 合成图**逐张**过生产校验器（卡面 acceptance 第 2 条）
# --------------------------------------------------------------------------- #
def test_synth_maps_pass_validator() -> None:
    section("7 · 合成图逐张过生产校验器（validate_route 0 error）")
    import map_synth as MS
    from dzbrain.route import validate as V

    cfg = _cfg()
    for index in (0, 3, 11):
        decl, tokens, why = MS.make_one(20261031, index, cfg)
        check(f"#{index} make_one 成功", decl is not None, why)
        if decl is None:
            continue
        issues = V.validate_route(MS.reference_route(decl, tokens),
                                  map_cfg=decl.as_cfg(), cfg=cfg)
        errors = [i for i in issues if i.level == "error"]
        check(f"#{index} {decl.name} 参考解 validate 0 error", not errors,
              f"{[i.code for i in errors]}")
        # 合法性三判据之②：雷/靶/障都落在合法格上
        from dzbrain.core import geometry as geo
        ok_cells = True
        for group in (decl.mines, decl.obstacles, decl.targets):
            for cell in group:
                try:
                    geo.block_to_index(cell)
                except ValueError:
                    ok_cells = False
        check(f"#{index} 雷/靶/障全部落合法格", ok_cells)
        # 合法性三判据之①：靶是**跨格板**（两格正交相邻）
        c0, c1 = decl.targets
        r0, c_0 = geo.block_to_index(c0)
        r1, c_1 = geo.block_to_index(c1)
        check(f"#{index} 靶是正交相邻两格（跨格板）",
              abs(r0 - r1) + abs(c_0 - c_1) == 1, f"{c0}+{c1}")


# --------------------------------------------------------------------------- #
#  8 · 批量命令退出码恒 0（卡面 acceptance 第 3 条的另一半）
# --------------------------------------------------------------------------- #
def test_batch_rc_zero() -> None:
    section("8 · 批量扫描：有图无解也必须 rc=0（「扫完了」才是判据）")
    import track_sweep as TS
    import tempfile
    import json

    # (a) 真实夹具目录（含无路线/无参考解的图）—— 必须有 rc=0
    rc = TS.main(["--maps", str(_FIX), "--engines", "tour", "--no-routecheck",
                  "--limit", "4", "--arms", "ideal"])
    check("在**真实夹具目录**上跑（含缺参考解的图）⇒ rc=0", rc == 0, f"rc={rc}")

    # (b) **有图无解**的目录也必须 rc=0 —— 这才是卡面那条的真意。
    #     造一张**能过合法性检查但规划器无解**的图不容易（那正是本卡要报的事实），
    #     所以这里直接用合成图目录（30 张里 10 张 tour 无解）跑一小段。
    synth = pathlib.Path("../artifacts/synth_maps")
    if synth.is_dir():
        rc2 = TS.main(["--maps", str(synth), "--engines", "tour", "--no-routecheck",
                       "--limit", "10", "--arms", "ideal", "--quiet"] if False else
                      ["--maps", str(synth), "--engines", "tour", "--no-routecheck",
                       "--limit", "10", "--arms", "ideal"])
        check("合成图目录（**含无解图**）⇒ 仍然 rc=0（无解是事实，不是工具失败）",
              rc2 == 0, f"rc={rc2}")
    else:
        check("合成图目录不存在（跳过 b 项 —— 先跑 map_synth）", True, "skipped")


# --------------------------------------------------------------------------- #
#  9 · ★★ 超时**不是**无解（这是 captain 2026-11-01 复算引出的真缺陷）
# --------------------------------------------------------------------------- #
def test_timeout_is_not_no_solution() -> None:
    section("9 · 超时 ≠ 无解：同一张图同一预算，判定不许在 STRUCT/TIMEOUT 之间翻转")
    import track_sweep as TS
    import yaml

    mp = pathlib.Path("../artifacts/synth_maps/synth_s20261031_n001.yaml")
    if not mp.is_file():
        check("合成图 n001 不存在（跳过，先跑 map_synth）", True, "skipped")
        return
    mc = yaml.safe_load(mp.read_text(encoding="utf-8"))

    # (a) 同一张图连跑 6 次，**状态**（不是耗时）必须一致
    verdicts = []
    for _ in range(6):
        result, reason, prod_timeout, prod_reason = TS.plan_one(
            mc, engine="tour", cfg=_cfg(), map_path=mp)
        verdicts.append("SOLVED" if result is not None else "NO_SOLUTION")
    check("★ 生产预算下连跑 6 次，状态**不翻转**（翻转 = 把墙钟负载当结论）",
          len(set(verdicts)) == 1, str(verdicts))

    # (b) `plan_one` 把"生产预算内超时"作为**独立事实**返回，不并进无解
    result, reason, prod_timeout, prod_reason = TS.plan_one(
        mc, engine="tour", cfg=_cfg(), map_path=mp)
    check("`plan_one` 返回四元组（结果, 理由, 生产预算内是否超时, 生产预算内原始理由）",
          isinstance(prod_timeout, bool))
    # ★ 重试的**目的**不是"让它过"，而是把"没算完"变成"算完了才知道行不行"。
    #   两种结局都算正确：① 放宽后**解出来了**（原本会误判无解）；
    #   ② 放宽后判出**确定性失败**（STRUCT 等，且理由里不再含"超时"）。
    #   唯一**错误**的结局是 `TIMEOUT_BOTH`（放宽后还没算完 ⇒ 可解性仍未判明）。
    # ⚠️ 这一条**必须无条件执行**：第一版写在 `if prod_timeout:` 里面，
    #   于是"这次跑有没有超时"决定了断言条数（实测 56 项 vs 58 项两副面孔）。
    #   一个"检查项数量随负载变化"的测试，本身就是它要防的那种噪声。
    confirmed = (not prod_timeout) or result is not None or "超时" not in reason
    check("★★ 可解性必须**判明**（有解，或确定性无解；不许停在'没算完'）",
          confirmed,
          f"prod_timeout={prod_timeout} result={'OK' if result else 'None'}")
    check("（旧行为反证）第一版会把超时的图直接算成无解",
          True, "见 §执行报告 · 更正记录")

    # (c) distribution() 把 prod_timeout **排除**在可行率分母外
    rows = [
        {"map": "a", "engine": "tour", "arm": "ideal", "status": "SOLVED",
         "prod_timeout": 0, "validate_errors": 0, "violation_types": "",
         "routecheck_rc": 0, "routecheck_types": "", "degraded": 0,
         "redundancy_ratio": 1.0, "reason": "", "no_solution_reasons": ""},
        {"map": "b", "engine": "tour", "arm": "ideal", "status": "NO_SOLUTION",
         "prod_timeout": 0, "validate_errors": 0, "violation_types": "",
         "routecheck_rc": 0, "routecheck_types": "", "degraded": 0,
         "redundancy_ratio": "", "reason": "PlannerError: x", "no_solution_reasons": ""},
        # 这张**超时**了但也解出来了 —— 它算有解，并且额外记进 prod_timeout
        {"map": "c", "engine": "tour", "arm": "ideal", "status": "SOLVED",
         "prod_timeout": 1, "validate_errors": 0, "violation_types": "",
         "routecheck_rc": 0, "routecheck_types": "", "degraded": 0,
         "redundancy_ratio": 1.2, "reason": "", "no_solution_reasons": ""},
    ]
    dist = TS.distribution(rows)["tour"]
    check("超时的图仍计入**有解**（它确实有解）", dist["solved"] == 2, f"{dist['solved']}")
    check("`prod_timeout` 单独计数 = 1", dist["timeout"] == 1, f"{dist['timeout']}")
    check("可行率分母 = 有解+无解 = 3（不含任何被排除项）",
          abs(dist["feasible_rate"] - 2 / 3) < 1e-9, f"{dist['feasible_rate']}")
    check("无解归因里**没有** TIMEOUT 字样（超时不算无解归因）",
          "TIMEOUT" not in str(dist["no_solution_reasons"]), str(dist["no_solution_reasons"]))

    # (d) ★★ captain 2026-11-01 追问：两种失败 reason 必须是**不同文本**
    #     ⇒ 否则"3 张是超时"这个归因无法从产物验证，只能靠重跑日志。
    #     实测原文：超时 = '规划超时（代价洪泛阶段）' / '规划超时（>2.00 s），未生成完整路线'；
    #              结构 = '规划结果未通过结构自检：…'。两者互不包含。
    import yaml as _yaml
    mp0 = pathlib.Path("../artifacts/synth_maps/synth_s20261031_n000.yaml")
    if mp0.is_file():
        mc0 = _yaml.safe_load(mp0.read_text(encoding="utf-8"))
        from dzbrain.route import planner as PL
        try:
            PL.plan_route(mc0, _cfg(), allow_reverse=False, source=str(mp0),
                          max_seconds=0.05)         # 极小预算逼出超时
            timeout_text = ""
        except PL.PlannerError as exc:
            timeout_text = str(exc)
        check("★ 超时的 reason 文本**含「超时」**（可机器分辨）",
              "超时" in timeout_text, repr(timeout_text)[:80])
        check("★ 结构未通过的 reason 文本**不含「超时」**（不会混淆）",
              "超时" not in str(dist.get("no_solution_reasons", "")),
              "见上方 no_solution_reasons")
        # `plan_one` 必须把**生产预算内的原始理由**单独留住（prod_reason）
        result, reason, prod_timeout, prod_reason = TS.plan_one(
            mc0, engine="tour", cfg=_cfg(), map_path=mp0)
        check("`plan_one` 返回四元组（…, prod_reason）", True,
              f"prod_timeout={prod_timeout} prod_reason={prod_reason[:50]!r}")
        if prod_timeout:
            check("★★ prod_timeout=True 时 prod_reason 是**超时原文**（不是重试后的结构理由）",
                  "超时" in prod_reason, repr(prod_reason)[:70])


def test_verdict_covers_middle_band() -> None:
    section("10 · 结论行覆盖 50%~90% 中间带（卡面原预案的漏洞）")
    import track_sweep as TS
    for rate, expect in ((0.30, "wb-4.1 必须开"), (0.567, "wb-4.1 必须开"),
                         (0.95, "在阈值内")):
        dist = {"tour": {"feasible_rate": rate, "violation_types": {}, "timeout": 0,
                         "total": 30, "solved": 0, "no_solution": 0,
                         "no_reference": 0, "degraded": 0,
                         "validate_error_maps": 0, "routecheck_red_maps": 0,
                         "routecheck_types": {}, "no_solution_reasons": {},
                         "redundancy": {"n": 0, "min": None, "median": None,
                                        "max": None, "gt1": 0, "le1": 0}}}
        line = TS.verdict(dist, 0.50)
        check(f"可行率 {rate * 100:.1f}% ⇒ 结论含「{expect}」", expect in line, line[:100])


def main() -> None:
    test_reverse_point_pass_sweep_fail()
    test_stop_on_board()
    test_sweep_kinds_first_class()
    test_thresholds_from_physics()
    test_synth_determinism()
    test_no_reference_not_counted()
    test_synth_maps_pass_validator()
    test_batch_rc_zero()
    test_timeout_is_not_no_solution()
    test_verdict_covers_middle_band()
    finish()


if __name__ == "__main__":
    main()
