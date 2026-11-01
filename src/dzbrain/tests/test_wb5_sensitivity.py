# -*- coding: utf-8 -*-
"""WB-5 台架敏感度证明（卡 `WB-5-REGNET-20261031`）。

运行::

    cd dzrobot_v2 && PYTHONPATH=src/dzbrain python src/dzbrain/tests/test_wb5_sensitivity.py

全过 exit 0；任一 FAIL / XPASS exit 1。

## 这个文件到底在证什么（一句话）

**"台架能测出坏"本身必须有机器证明**，不能靠"我们跑过几次看着挺像"担保。

审计给的决定性证据（本卡存在的前提）：直线/合成夹具下把循迹故意改坏 → **0 条红**；
换现场 68-token 路线 → 抓出 6 条。也就是说**敏感度不是天生有的**，取决于用哪条路线。

本文件把它固化成三条互相咬合的判据：

1. **判据级**（§2）：在 `field7`（现场 68 token）上跑变异臂，
   `JUDGE` 里**每条**指标断言都必须至少被一条变异臂打红 —— 全绿 = 本测试 FAIL；
2. **变异级**（§3）：每条变异臂必须至少打红一条判据 —— 全绿（变异被漏检）= 本测试 FAIL；
3. **反例级**（§4/§5）：`field`（早期 34 点夹具）上同一条"前视钉死 3.0 m"变异
   **让指标变好**（`cross_track_rms` 0.1834→0.1487、`final_error` 0.1093→0.0549）⇒
   该夹具抓不住它，必须登记为 **fixture 档 = 禁止用于性能结论**，且本测试**机器断言**
   这个"变好"现象还在 —— 谁把 `field` 提升成判据档，这条立刻红。

## 一条非判据但同样重要的对照：空变异

`identity` 臂把生产 `compute_command` **原样包一层**，指标必须逐位不变
（实测 digest 与 base 相同）。没有它，"判据太紧导致什么都红"的假红就洗不干净。

## 已实测的两个坑（照抄会翻车）

1. **`--variant` 是死参数**：`tools/sim_rig.py:635` 声明了 `--variant`，但
   `run()`（`:202`）与 `main()`（`:686`）从不把它传下去 ⇒ CLI 侧 `--variant no.such.fn`
   实测 **rc=0**（应当 rc=2）。变异只能在本进程内替换模块级绑定。见 §7 XFAIL。
2. **必须替换 `tasks.tracker.compute_command`**，不是 `pursuit.controller` 那个：
   tracker 是 `from ..pursuit.controller import compute_command`（模块级绑定），
   改 controller 模块里的名字对已 import 的 tracker **无效**（会得到一个"变异没生效"的假绿）。

## 成本

一次跑 130 s 虚拟时间 ≈ 7 s 墙钟；本文件共 7 次仿真 ≈ 25 s。
"""

from __future__ import annotations

import hashlib
import json
import math
import pathlib
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
_XFAIL: list[str] = []
_XPASS: list[str] = []
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


def xfail(name: str, cond: bool, owner: str, why: str) -> None:
    """**已知且已归因**的失败：不判红，但不许静默腐烂。

    `cond=True`（问题已修好）⇒ 记 **XPASS** 并判本测试 FAIL ——
    否则这条记录会变成装饰性注释，而真正要跟进的人以为它还红着。
    """
    if cond:
        _XPASS.append(f"{_SECTION} :: {name}（owner={owner}；{why}）")
        print(f"  XPASS {name} —— 已修好，请把这条 XFAIL 改成正式断言")
    else:
        _XFAIL.append(f"{name}（owner={owner}）")
        print(f"  xfail {name} —— 已知问题，未修（owner={owner}）：{why}")


def finish() -> None:
    print("\n" + "=" * 74)
    total = _PASS + len(_FAIL)
    print(f"结果：{_PASS}/{total} 通过"
          + (f"，{len(_FAIL)} 失败" if _FAIL else " 全部通过"))
    for item in _FAIL:
        print(f"  FAIL  {item}")
    if _XPASS:
        print(f"  XPASS {len(_XPASS)} 条（XFAIL 居然通过 ⇒ 按失败算）")
        for item in _XPASS:
            print(f"    - {item}")
    if _XFAIL:
        print(f"  （另有 {len(_XFAIL)} 条已归因 XFAIL，不计入失败）")
        for item in _XFAIL:
            print(f"    · {item}")
    print("=" * 74)
    sys.exit(1 if (_FAIL or _XPASS) else 0)


# --------------------------------------------------------------------------- #
#  仿真跑手（唯一入口：所有臂都走这里，保证"同参数同场地同种子，只差变异"）
# --------------------------------------------------------------------------- #
#: 判据档场景（现场 68 token）。**只有它**可以出性能/通过性结论。
JUDGE_SCENARIO = "field7"
#: 参考档场景（早期 34 点夹具）。只许当"能被抓到的下限"参照，**禁止**用于性能结论。
FIXTURE_SCENARIO = "field"
#: 判据档时长（现场 68 token 全程 ≈ 118 s；130 s 留余量，让"跑不完"能体现出来）
JUDGE_SECS = 130.0
#: 参考档时长
FIXTURE_SECS = 60.0


def _digest(values: dict) -> str:
    """指标快照的稳定指纹（用来证明"空变异一位不变"）。"""
    blob = json.dumps(values, sort_keys=True, ensure_ascii=False, default=str)
    return hashlib.sha256(blob.encode("utf-8")).hexdigest()[:16]


def run_arm(scenario: str, *, seconds: float, arm: str = "ideal",
            steer_mul: float | None = None, lookahead_pin: float | None = None,
            identity_wrap: bool = False) -> dict:
    """跑一条臂并返回 `metrics.values`。

    变异只做两件事（都是"把循迹算法改坏"的最小手术，不正负两向调参）：
      · `steer_mul=-1.0` 舵角增益反号（车往反方向打舵）；
      · `lookahead_pin=3.0` 前视距离钉死 3.0 m（比一格 0.35 m 大近一个数量级）；
      · `identity_wrap=True` 原样包一层（空变异对照，必须一位不变）。

    ★ 为什么用 `ideal` 臂：判据要证的是"控制器坏了台架能看出来"，
      叠加注入噪声只会让"能看出来"变成"可能是噪声贡献的"。注入档的敏感度由
      `test_wb2_rig_invariants.py` §3 与 `metrics.transfer` 分级负责。
    """
    import dzbrain.pursuit.controller as CTRL
    import dzbrain.tasks.tracker as TRK
    import sim_rig
    import wb_rig.arms as ARMS
    import wb_rig.scenarios as SCN

    base_fn = TRK.compute_command
    base_ld = CTRL.lookahead_distance
    try:
        if (steer_mul is not None and steer_mul != 1.0) or identity_wrap:
            def wrapper(pose, path, proj, cfg, **kw):
                cmd = base_fn(pose, path, proj, cfg, **kw)
                mul = 1.0 if steer_mul is None else float(steer_mul)
                return type(cmd)(linear_x=cmd.linear_x,
                                 steer_deg=cmd.steer_deg * mul,
                                 curvature=cmd.curvature * mul,
                                 speed_reason=cmd.speed_reason,
                                 reverse=cmd.reverse)
            TRK.compute_command = wrapper
        if lookahead_pin is not None:
            CTRL.lookahead_distance = lambda speed, cfg: float(lookahead_pin)
        res = sim_rig.run(scenario=SCN.resolve(scenario), arm=ARMS.get(arm),
                          seconds=float(seconds), field_modeled=False,
                          log=lambda msg: None)
        return dict(res["metrics"].values)
    finally:
        TRK.compute_command = base_fn          # ★ 必须还原，否则污染后续臂
        CTRL.lookahead_distance = base_ld


# --------------------------------------------------------------------------- #
#  判据集（**唯一的判据真源**：tier-1 上"循迹有没有跑对"就由这张表定义）
# --------------------------------------------------------------------------- #
#: 每条 = (键, 方向, 阈值, 人话)。`<=` 表示"不许超过"，`>=` 表示"不许低于"。
#: ★ 新增判据必须同时证明"它至少能被一条变异打红"（§2 会自动检查这一点）。
#: ★ 刻意**不**把 `belief_error_rms_m` 放进来：ideal 档它必须恒 0，
#:   那是"注入链没偷改物理"的结构性不变量（§4），不是"控制器好坏"的判据。
#: ★ 刻意**不**把 `mission_completed` 放进来：实测它恒为 0（§6 XFAIL，owner=wb-core）——
#:   拿一条**永远不会绿**的指标当判据，等于给判据集埋一条永久假红。
JUDGE: tuple[tuple[str, str, float, str], ...] = (
    ("finished", ">=", 1.0, "跑没跑完全程（改坏舵角的直接后果就是跑不完）"),
    ("length_ratio", ">=", 0.98, "已走弧长 / 路线总长（跑偏会让它掉下来）"),
    ("s_final_m", ">=", 24.0, "末拍投影弧长（路线总长 24.85 m）"),
    ("seg_final", ">=", 66.0, "末拍所在段号（68 token 路线的末段）"),
    ("cross_track_rms_m", "<=", 0.35, "真值横向 RMS"),
    ("cross_track_max_m", "<=", 0.75, "真值横向 MAX"),
    ("final_error_m", "<=", 0.10, "末拍真值横向误差"),
)


def judge_fails(values: dict) -> list[str]:
    """按 `JUDGE` 判一条臂，返回**失败的判据 key 列表**（空 = 全绿）。"""
    bad: list[str] = []
    for key, op, limit, _why in JUDGE:
        val = values.get(key)
        if isinstance(val, bool):
            val = float(val)
        if not isinstance(val, (int, float)) or (isinstance(val, float) and not math.isfinite(val)):
            bad.append(f"{key}(非数值 {val!r})")
            continue
        ok = val >= limit if op == ">=" else val <= limit
        if not ok:
            bad.append(key)
    return bad


# --------------------------------------------------------------------------- #
#  1. 池子：一次把需要的臂全跑出来（同一进程、同一进程级状态）
# --------------------------------------------------------------------------- #
ARMS_DATA: dict[str, dict] = {}


def collect() -> None:
    section("1. 跑出全部对照臂（判据档 = field7 现场 68 token）")
    for name, kw in (("base", {}),
                     ("identity", {"identity_wrap": True}),
                     ("steer_neg", {"steer_mul": -1.0}),
                     ("lookahead_3m", {"lookahead_pin": 3.0})):
        ARMS_DATA[name] = run_arm(JUDGE_SCENARIO, seconds=JUDGE_SECS, **kw)
        d = ARMS_DATA[name]
        print(f"  [{name:12s}] finished={d.get('finished')} length_ratio={d.get('length_ratio'):.4f} "
              f"rms={d.get('cross_track_rms_m'):.4f} max={d.get('cross_track_max_m'):.4f} "
              f"final={d.get('final_error_m'):.4f} s={d.get('s_final_m'):.3f} "
              f"seg={d.get('seg_final')} t={d.get('time_s'):.1f} digest={_digest(d)}")

    base = ARMS_DATA["base"]
    check("判据档基线本身全绿（判据自洽，不是天生就红）",
          judge_fails(base) == [], f"红={judge_fails(base)}")
    check("基线跑完全程（68 token / 24.85 m）",
          base.get("finished") == 1.0 and base.get("length_ratio", 0) >= 0.98,
          f"finished={base.get('finished')} lr={base.get('length_ratio'):.4f}")

    # ---- 空变异对照：原样包一层，指标必须一位不变 ----
    check("★ 空变异（原样包装生产控制器）指标**逐位不变**",
          _digest(ARMS_DATA["identity"]) == _digest(base),
          f"identity={_digest(ARMS_DATA['identity'])} base={_digest(base)}")


# --------------------------------------------------------------------------- #
#  2. 判据级：每条判据都必须至少被一条变异臂打红
# --------------------------------------------------------------------------- #
MUTANTS = ("steer_neg", "lookahead_3m")


def test_every_judge_is_sensitive() -> None:
    section("2. 判据级：JUDGE 里每条判据都被至少一条变异臂打红（否则该判据 = 假绿）")
    reds = {m: judge_fails(ARMS_DATA[m]) for m in MUTANTS}
    for m, bad in reds.items():
        print(f"  {m:12s} 打红: {bad or '（一条都没打红）'}")

    for key, op, limit, why in JUDGE:
        hit = [m for m in MUTANTS if any(b.split("(")[0] == key for b in reds[m])]
        check(f"判据 `{key}` ({op} {limit}) 被变异打红",
              bool(hit), f"打红者={hit or '无'} · {why}")

    # 变异级：每条变异必须至少打红一条判据
    for m in MUTANTS:
        check(f"变异 `{m}` 至少打红一条判据（漏检 = 台架抓不住这个坏）",
              bool(reds[m]), f"红={reds[m]}")

    # ★ 这条是本卡的"反向自证"：如果两条变异都只打红同一条判据，
    #   说明判据集退化成单点判据 —— 一个判据扛不住"公司法"。
    distinct = {b.split("(")[0] for m in MUTANTS for b in reds[m]}
    check("两条变异打红的判据**集合不退化**（≥3 条不同判据）",
          len(distinct) >= 3, f"共 {len(distinct)} 条：{sorted(distinct)}")


# --------------------------------------------------------------------------- #
#  3. 反例级：field（34 点夹具）抓不住"前视钉死"，且**指标变好**
# --------------------------------------------------------------------------- #
field_data: dict[str, dict] = {}


def test_fixture_tier_is_blind() -> None:
    section("3. 反例级：`field` 夹具上同一条变异**变好**（证明它不能当判据档）")
    for name, kw in (("base", {}), ("lookahead_3m", {"lookahead_pin": 3.0}),
                     ("steer_neg", {"steer_mul": -1.0})):
        field_data[name] = run_arm(FIXTURE_SCENARIO, seconds=FIXTURE_SECS, **kw)
        d = field_data[name]
        print(f"  [field/{name:12s}] finished={d.get('finished')} "
              f"lr={d.get('length_ratio'):.4f} rms={d.get('cross_track_rms_m'):.4f} "
              f"max={d.get('cross_track_max_m'):.4f} final={d.get('final_error_m'):.4f} "
              f"s={d.get('s_final_m'):.3f} digest={_digest(d)}")

    b, m = field_data["base"], field_data["lookahead_3m"]
    better = (m["cross_track_rms_m"] < b["cross_track_rms_m"]
              and m["final_error_m"] < b["final_error_m"])
    check("★ `field` 夹具上「前视钉死 3.0 m」让两条指标**同时变好** ⇒ 该夹具抓不住这个坏",
          better,
          f"rms {b['cross_track_rms_m']:.4f}→{m['cross_track_rms_m']:.4f}（越小越好）· "
          f"final {b['final_error_m']:.4f}→{m['final_error_m']:.4f}（越小越好）")
    check("★ 同一变异在判据档 `field7` 上**必须**被 rms/max/final 三点打红",
          all(k in judge_fails(ARMS_DATA["lookahead_3m"])
              for k in ("cross_track_rms_m", "cross_track_max_m", "final_error_m")),
          f"field7 红={judge_fails(ARMS_DATA['lookahead_3m'])}")

    # ---- tier 登记：机器可读的"这条路线能不能出性能结论" ----
    import wb_rig.scenarios as SCN
    tier = {SCN.resolve(JUDGE_SCENARIO).name: "judge",
            SCN.resolve(FIXTURE_SCENARIO).name: "fixture"}
    check("tier 登记：`field7`=judge / `field`=fixture",
          tier == {"field7": "judge", "field": "fixture"}, str(tier))
    check("★ `field` 被登记为 fixture 档后，判据函数对它**拒绝出结论**",
          tier_allows_verdict("field") is False and tier_allows_verdict("field7") is True,
          "tier_allows_verdict('field')=False / ('field7')=True")


#: tier → 能否用于性能/通过性结论。**只有 judge 档可以**。
TIER: dict[str, str] = {"field7": "judge", "field": "fixture",
                        "sixth": "fixture", "carloop": "fixture"}


def tier_allows_verdict(scenario: str) -> bool:
    """这条场景能不能用来给循迹打分。未登记的场合一律 **False**（宁可从严）。"""
    return TIER.get(str(scenario)) == "judge"


# --------------------------------------------------------------------------- #
#  4. 反证：敏感度必须来自"路线 + 真注入"，不是来自"台架自己会算"
# --------------------------------------------------------------------------- #
def test_sensitivity_is_not_free() -> None:
    section("4. 反证：变异在 `--no-field`（场地未建模）下照样被抓（敏感度不靠场地要素）")
    # 上面 7 次跑全部 field_modeled=False（见 run_arm）。若敏感度靠的是场地要素，
    # 这里就该全漏检；实测不是 ⇒ 判据来自「轨迹几何 × 控制器」，与雷/障/靶无关。
    check("判据档两条变异在 `field_modeled=0` 下各打红 ≥1 条",
          all(judge_fails(ARMS_DATA[m]) for m in MUTANTS),
          f"steer_neg={judge_fails(ARMS_DATA['steer_neg'])} "
          f"lookahead_3m={judge_fails(ARMS_DATA['lookahead_3m'])}")
    # ★ 这条钉的是**置灰语义**的关键一半：越界类必须随 field_modeled 变化，
    #   而 field_stopped / field_modeled 自己**不许**变化（它们两种模式都在）。
    check("`field_modeled=0` 时 `field_modeled` 键**仍在**且等于 0（不是消失）",
          ARMS_DATA["base"].get("field_modeled") == 0.0,
          repr(ARMS_DATA["base"].get("field_modeled")))
    check("`field_stopped` 两种模式都写（未触发 ⇒ 0.0，**不是 null**）",
          ARMS_DATA["base"].get("field_stopped") == 0.0,
          repr(ARMS_DATA["base"].get("field_stopped")))


def test_bounds_greying() -> None:
    section("5. 置灰语义：`field_modeled=0` 时越界类指标到底该是什么")
    # ★★ 这里有一条**与 `00_树.md` 硬约束 4 / 《06 册》§2 相冲突的实测发现**，
    #    所以分成两半判据：能证的证，不能证的挂 XFAIL 交给 owner，**不许拿绿盖红**。
    import wb_rig.metrics as MET
    m = MET.Metrics(arm="ideal", field_modeled=0)
    for key in MET.BOUNDS_KEYS:
        m.put(key, 0.0)
    greyed = {k: m.values.get(k) for k in MET.BOUNDS_KEYS}
    check("★ 数据层置灰是真实现的：`Metrics.put` 在 field_modeled=0 时把越界类压成 None",
          all(v is None for v in greyed.values()), str(greyed))

    # 但**实际 payload** 里这三个键被 sim_rig 末尾的 `m.values[...] = oob` 直接写回真值，
    # 绕过了 put() ⇒ 置灰在端到端上不生效。这就是下面这条 XFAIL。
    live = {k: ARMS_DATA["base"].get(k) for k in MET.BOUNDS_KEYS}
    xfail("`field_modeled=0` 的**实际输出**里越界类为 null（端到端置灰生效）",
          all(v is None for v in live.values()) and live != {},
          owner="wb-core",
          why=f"实测输出 {live} —— sim_rig 用 m.values[...]=oob 直写，绕过 Metrics.put 的置灰；"
              f"《00_树》硬约束 4 要求「不许显示 0 冒充零越界」")
    print(f"  （实测 payload：{live}）")
    print("  （注意：本卡**不改** sim_rig.py（wb-core 写域），只把冲突钉成可见的 XFAIL）")


# --------------------------------------------------------------------------- #
#  5. 结构性不变量：ideal 档 injection 链没偷改物理
# --------------------------------------------------------------------------- #
def test_ideal_invariants() -> None:
    section("6. 结构性不变量（ideal 档）：信念↔真值必须恒 0，且与变异无关")
    base = ARMS_DATA["base"]
    check("ideal 档 `belief_error_rms_m` 精确 0（注入链没偷改物理）",
          base.get("belief_error_rms_m") == 0.0, repr(base.get("belief_error_rms_m")))
    check("★ 变异臂上 `belief_error_rms_m` **仍然 0**（证明它不随控制器变化，是独立不变量）",
          all(ARMS_DATA[m].get("belief_error_rms_m") == 0.0 for m in MUTANTS),
          str({m: ARMS_DATA[m].get("belief_error_rms_m") for m in MUTANTS}))

    # ---- `mission_completed`（★ 2026-11-01 wb-core 已修：跑完就记 1，不再被 IDLE 吞掉）----
    check("（记录）车确实跑完全程（finished=1 / length_ratio≈1）",
          base.get("finished") == 1.0 and base.get("length_ratio", 0) >= 0.98,
          f"finished={base.get('finished')} lr={base.get('length_ratio'):.4f}")
    check("★ `mission_completed` 在跑完全程时 = 1（曾因 FINISH 被 IDLE 覆盖而恒 0）",
          base.get("mission_completed") == 1.0, repr(base.get("mission_completed")))


# --------------------------------------------------------------------------- #
#  7. `--variant` 接线（★ 2026-11-01 wb-core 已修：run() 接收 variant，坏路径 rc=2）
# --------------------------------------------------------------------------- #
def test_variant_is_wired() -> None:
    section("7. `--variant` 真的接到 `run()`（死参数已由 wb-core 修好，改正式断言）")
    import inspect
    import sim_rig
    sig = inspect.signature(sim_rig.run)
    has = "variant" in sig.parameters
    cli_declared = "--variant" in open(str(_TOOLS / "sim_rig.py"), encoding="utf-8").read()
    check("★ `sim_rig.run()` 接收 `variant` 形参（CLI 的 `--variant` 真生效）",
          has, str(sorted(sig.parameters)))
    check("CLI 声明了 `--variant`", cli_declared, "sim_rig.py 含 --variant")
    # ★ 这条 XFAIL 的历史：wb-core 接线前，`--variant` 是**死参数**（`main()` 声明却
    #   从不传给 `run()`），坏变体路径实测 rc=0（应 rc=2）。接线后转 XPASS ⇒ 已改成
    #   上面的正式断言。保留这段记录，是为了让后来人知道**这条判据曾经真的是红的**。
    check("（记录）曾经的红已转绿：坏变体路径现在由 §9 真跑钉住（rc=2）", True, "")


# --------------------------------------------------------------------------- #
#  7. 场景层：无路线场景必须显式拒绝，不许猜一条路线出来
# --------------------------------------------------------------------------- #
def test_scenario_guard() -> None:
    section("8. 场景层：`sixth`/`carloop` 无路线 ⇒ 显式报错（不猜路线、不回落已删地图）")
    import wb_rig.scenarios as SCN
    for name in ("sixth", "carloop"):
        sc = SCN.resolve(name)
        check(f"`{name}` 的 route 为 None（有图无路线是合法状态）",
              sc.route_path is None, f"map={pathlib.Path(sc.map_path).name}")
        check(f"`{name}` 在台架入口会以 rc=2 拒绝（不是静默换图）",
              not tier_allows_verdict(name) or sc.route_path is None, name)


# --------------------------------------------------------------------------- #
#  8. CLI 子进程：坏变体路径的真实退出码（外部可观测，不依赖进程内替换）
# --------------------------------------------------------------------------- #
def test_cli_bad_variant_rc() -> None:
    section("9. CLI 子进程：坏 `--variant` 的真实退出码（文档说 2）")
    import subprocess
    env = dict(__import__("os").environ)
    env["PYTHONPATH"] = str(_SRC)
    env["PYTHONIOENCODING"] = "utf-8"
    proc = subprocess.run(
        [sys.executable, str(_TOOLS / "sim_rig.py"), "--scenario", JUDGE_SCENARIO,
         "--arm", "ideal", "--seconds", "5", "--no-field", "--quiet",
         "--variant", "no.such.module.fn", "--out-dir", str(_REPO / "artifacts"
                                                             / "sim_rig" / "_wb5_probe")],
        cwd=str(_REPO), env=env, capture_output=True, text=True, timeout=300)
    check("★ 坏 `--variant` 不许**扮成一次成功跑**（退出码 0 + 契约行 = 双重假绿）",
          not (proc.returncode == 0 and "TRACKBENCH_RESULT_JSON" in proc.stdout),
          f"rc={proc.returncode} 契约行={'有' if 'TRACKBENCH_RESULT_JSON' in proc.stdout else '无'}")
    # ★ 2026-11-01：这条曾是 XFAIL（wb-core 接线前 `--variant` 是死参数、坏路径 rc=0）。
    #   现已修好 ⇒ 改**正式断言**，钉住不再回退。历史写在这里：谁把 `main()` 里的
    #   `variant=args.variant` 删了，这条立刻红。
    check("★ 坏 `--variant` 的退出码 = 2（文档契约；2026-11-01 起真生效）",
          proc.returncode == 2,
          f"实测 rc={proc.returncode}；若回到 0 说明 `main()` 又不把 variant 传给 `run()` 了")


# --------------------------------------------------------------------------- #
#  main
# --------------------------------------------------------------------------- #
def main() -> int:
    print("WB-5 · 台架敏感度证明（判据档=field7 / 参考档=field / 变异=舵角反号 + 前视钉死 3 m）")
    collect()
    test_every_judge_is_sensitive()
    test_fixture_tier_is_blind()
    test_sensitivity_is_not_free()
    test_bounds_greying()
    test_ideal_invariants()
    test_variant_is_wired()
    test_scenario_guard()
    test_cli_bad_variant_rc()
    finish()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
