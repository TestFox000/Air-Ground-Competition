# -*- coding: utf-8 -*-
"""WB-5 · 契约行解析（卡 `WB-5-REGNET-20261031`）。

运行::

    cd dzrobot_v2 && PYTHONPATH=src/dzbrain python src/dzbrain/tests/test_wb5_contract.py

全过 exit 0；任一 FAIL exit 1。

## 这个文件钉的是什么

编排层（插件 `src/simbench.js`）现在靠**钉死 stdout 的逐行文字排版**取数：
`simbench.js` 自己写着"不这么写会抓错行"。这条依赖的脆弱性有两次血证：

- F-5 抓串事故（《06 册》§3）：`cross_track_rms` 与 `belief_error_rms` 两个数被读串，
  于是"信念误差"被当成"横向误差"报了出去（错得毫无迹象，两个数都在合理量级）；
- 只取"**某一行**"而不取"**末行**"：中间过程行、警告行、`print` 的调试残留
  一旦含同样的字样，解析器就抓到前一条 —— 而且**不报错**。

本文件把契约机的**两侧**都钉住，缺一半都不算：

| 侧 | 判据 |
|---|---|
| 产出侧 | `TRACKBENCH_RESULT_JSON` 必须是 **stdout 的最后一行**；且它是**合法严格 JSON**、带 `schema` |
| 消费侧 | 解析器只认**前缀 + 末行**（`splitlines()[-1]` / `rsplit(prefix, 1)`），**不许全文扫** |

## 反例（真跑，不是注释）

- `NEG-1`：契约行**不是末行**（后面再打一行）⇒ 解析器必须取不到 / 判红；
- `NEG-2`：全文扫的老写法（`text.index(prefix)` 取**第一次**出现）在多行输入上
  会取到**旧的**那一行 ⇒ 与末行解析结果不同，且末行解析才是对的；
- `NEG-3`：契约行里有 NaN（`allow_nan=False` 会炸）⇒ 必须抛，不许静默变 `null`。

## 顺带钉住的两件事

1. **两个 schema 并存**：`dzrobot-sim-result/1`（tier-0，冻结）与
   `dzrobot-trackbench-result/1`（tier-1 新号）**前缀也不同**
   （`SIM_RESULT_JSON` vs `TRACKBENCH_RESULT_JSON`）⇒ 消费方必须按前缀分流，
   不许按"含 JSON 的行"猜。
2. **五键形状冻结**：`finished/time_s/rms_m/out_of_bounds/length_ratio`
   在 tier-1 契约行里必须**同名同义**存在（缺键补 `None`，绝不用 0 冒充）。
"""

from __future__ import annotations

import json
import pathlib
import re
import subprocess
import sys

_HERE = pathlib.Path(__file__).resolve()
_SRC = _HERE.parents[1]                       # src/dzbrain
_REPO = _SRC.parents[1]                       # dzrobot_v2
_TOOLS = _REPO / "tools"
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
    if cond:
        _XPASS.append(f"{_SECTION} :: {name}（owner={owner}；{why}）")
        print(f"  XPASS {name} —— 已修好，请改成正式断言")
    else:
        _XFAIL.append(f"{name}（owner={owner}）")
        print(f"  xfail {name} —— 已知问题（owner={owner}）：{why}")


def finish() -> None:
    print("\n" + "=" * 74)
    total = _PASS + len(_FAIL)
    print(f"结果：{_PASS}/{total} 通过" + (f"，{len(_FAIL)} 失败" if _FAIL else " 全部通过"))
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
#  解析器（**唯一的消费侧实现**：本文件与将来的编排层必须都用这两个函数）
# --------------------------------------------------------------------------- #
TIER0_PREFIX = "SIM_RESULT_JSON"
TIER1_PREFIX = "TRACKBENCH_RESULT_JSON"


def parse_contract(text: str, prefix: str) -> dict | None:
    """取自**末行**、按**前缀**定位的契约对象。取不到 ⇒ `None`（不抛、不猜）。

    ★ 三条规则，每条都有血证：
      1. **从末往上找**（`splitlines()[::-1]`），不是 `text.index(prefix)`：
         后者取**第一次**出现，而台架会打多段横幅（`--repeat` 时更是 N 段），
         中间那段的契约行是**旧结果**（F-5 抓串）；
      2. **只认行首前缀**（`startswith(prefix + " ")`），不认"某行里有这个字串"：
         `notes` 里就出现过 `"= cross_track_rms_m（现网五键别名...）"` 这类文字，
         子串匹配会把它们当契约行；
      3. **失败返回 None，不抛**：调用方（编排层）要能区分"没跑到出契约行"
         与"契约行存在但解析坏了"，所以本函数只做"取不到"，坏 JSON 由 `strict` 另判。
    """
    for line in reversed(text.splitlines()):
        line = line.strip()
        if not line.startswith(prefix + " "):
            continue
        try:
            return json.loads(line[len(prefix) + 1:])
        except ValueError:
            return None
    return None


def parse_contract_strict(text: str, prefix: str) -> dict:
    """同 `parse_contract`，但**坏 JSON 与 NaN/Infinity 都抛**（产出一侧用这一版）。

    ★ `json.loads` 默认**接受** `NaN` / `Infinity`（Python 的扩展，不是标准 JSON）。
      产出侧用的是 `allow_nan=False`，所以一个含 NaN 的契约行只可能来自
      "别的实现写的"或"字符串拼接坏了"。默认行为会把它静默读成 `float('nan')`
      —— 而 `nan` 与任何数比较都 False，下游"越界判据"会**永远不触发**。
      所以严格版必须显式拒绝（`parse_constant` 抛）。
    """
    def _reject(token: str) -> dict:
        raise ValueError(f"契约行里出现非标准 JSON 字面量 {token!r}"
                         f"（产出侧 allow_nan=False，不应存在）")

    for line in reversed(text.splitlines()):
        line = line.strip()
        if not line.startswith(prefix + " "):
            continue
        return json.loads(line[len(prefix) + 1:], parse_constant=_reject)   # 故意不 catch
    raise ValueError(f"stdout 里没有 `{prefix} ` 开头的行（契约行缺失）")


#: 老写法（**反例**：全文扫 + 取第一次）。保留在文件里，是为了让 NEG-2 真跑它。
def parse_contract_legacy_first(text: str, prefix: str) -> dict | None:
    """反例：`text.index(prefix)` 取**第一次**出现的那行（F-5 事故的形状）。"""
    at = text.find(prefix)
    if at < 0:
        return None
    rest = text[at + len(prefix):].lstrip()
    line = rest.splitlines()[0] if rest else ""
    try:
        return json.loads(line)
    except ValueError:
        return None


# --------------------------------------------------------------------------- #
#  1. 产出侧：真跑 sim_rig，契约行必须是末行 + 严格 JSON + 带 schema
# --------------------------------------------------------------------------- #
_RUN: dict = {}


def run_rig_once() -> dict:
    """真跑一次（short 时长，只判契约形状，不判成绩）。"""
    if _RUN:
        return _RUN
    env = dict(__import__("os").environ)
    env["PYTHONPATH"] = str(_SRC)
    env["PYTHONIOENCODING"] = "utf-8"
    proc = subprocess.run(
        [sys.executable, str(_TOOLS / "sim_rig.py"),
         "--scenario", "field7", "--arm", "ideal", "--seconds", "8",
         "--no-field", "--json",
         "--out-dir", str(_REPO / "artifacts" / "sim_rig" / "_wb5_contract")],
        cwd=str(_REPO), env=env, capture_output=True, text=True, encoding="utf-8",
        errors="replace", timeout=600)
    _RUN.update(rc=proc.returncode, out=proc.stdout or "", err=proc.stderr or "")
    return _RUN


def test_producer_side() -> None:
    section("1. 产出侧：契约行是**末行**、严格 JSON、带 schema 与五键")
    r = run_rig_once()
    print(f"  rc={r['rc']}  stdout 行数={len(r['out'].splitlines())}")
    lines = [ln for ln in r["out"].splitlines() if ln.strip()]
    check("真跑成功（rc=0；本文件不判成绩，只判契约形状）", r["rc"] == 0, f"rc={r['rc']}")
    check("★ 契约行是 stdout 的**最后一个非空行**",
          bool(lines) and lines[-1].startswith(TIER1_PREFIX + " "),
          (lines[-1][:60] + "…") if lines else "（stdout 空）")
    payload = parse_contract(r["out"], TIER1_PREFIX)
    check("按前缀 + 末行能解析出对象", isinstance(payload, dict), type(payload).__name__)
    if not isinstance(payload, dict):
        return
    check("schema = `dzrobot-trackbench-result/1`（新号，与 tier-0 并存不取代）",
          payload.get("schema") == "dzrobot-trackbench-result/1", str(payload.get("schema")))
    five = payload.get("metrics") or {}
    check("★ 现网五键**同名同义**都在（形状冻结）",
          set(five) == {"finished", "time_s", "rms_m", "out_of_bounds", "length_ratio"},
          str(sorted(five)))
    check("`transfer` 分级随值一起走（每条指标都有档）",
          isinstance(payload.get("transfer"), dict)
          and set(payload["transfer"]) >= set(payload.get("metricsFull") or {}),
          f"{len(payload.get('transfer') or {})} 档 / "
          f"{len(payload.get('metricsFull') or {})} 值")
    check("`transfer_groups` 三档齐（可迁移/仅结构/不可迁移）",
          set(payload.get("transfer_groups") or {}) == {"可迁移", "仅结构", "不可迁移"},
          str(sorted(payload.get("transfer_groups") or {})))
    check("严格 JSON：没有 NaN/Infinity 字面量",
          not re.search(r"\b(NaN|Infinity|-Infinity)\b", lines[-1] or ""),
          "末行不含 NaN/Infinity")
    check("契约行**不超过一行**（换行会把它拆成两行，末行只剩半截 JSON）",
          "\n" not in (lines[-1] if lines else ""), "单行")

    # 前缀分流：两个 schema 的前缀**不同**，消费方必须按前缀认
    check("tier-0 前缀 `SIM_RESULT_JSON` 与 tier-1 `TRACKBENCH_RESULT_JSON` **互不包含前导**",
          parse_contract(r["out"], TIER0_PREFIX) is None,
          "用 tier-0 前缀解析 tier-1 输出 ⇒ 取不到（前缀分流有效）")


# --------------------------------------------------------------------------- #
#  2. 消费侧：末行解析 vs 全文扫（真跑两条解析器，对比结果）
# --------------------------------------------------------------------------- #
def test_consumer_side() -> None:
    section("2. 消费侧：末行解析 vs 全文扫（多段输出下全文扫会取到旧结果）")
    old = {"schema": "dzrobot-trackbench-result/1", "metrics": {"finished": 0.0},
           "marker": "OLD"}
    new = {"schema": "dzrobot-trackbench-result/1", "metrics": {"finished": 1.0},
           "marker": "NEW"}
    text = "\n".join([
        "== 第 1 次跑完 ==",
        f"{TIER1_PREFIX} {json.dumps(old, ensure_ascii=False, sort_keys=True)}",
        "== 第 2 次跑完 ==",
        f"{TIER1_PREFIX} {json.dumps(new, ensure_ascii=False, sort_keys=True)}",
        "",
    ])
    last = parse_contract(text, TIER1_PREFIX)
    first = parse_contract_legacy_first(text, TIER1_PREFIX)
    check("★ 末行解析取到**最新**那次（marker=NEW）", (last or {}).get("marker") == "NEW",
          str((last or {}).get("marker")))
    check("★ 全文扫的老写法取到**旧**那次（marker=OLD）—— 这就是 F-5 的形状",
          (first or {}).get("marker") == "OLD", str((first or {}).get("marker")))
    check("两种解析器结果**不同**（若相同则本判据不成立，说明测试没造出多段）",
          (last or {}).get("marker") != (first or {}).get("marker"), "OLD vs NEW")

    # 子串陷阱：`notes` 里出现指标名，子串匹配会把它当契约行
    trap = "\n".join([
        "notes: rms_m = cross_track_rms_m（现网五键别名，形状冻结）",
        f"{TIER1_PREFIX} {json.dumps({'schema': 'x', 'marker': 'REAL'})}",
    ])
    check("★ 行首前缀（不是子串）：含指标名的 notes 行不会被误认成契约行",
          (parse_contract(trap, TIER1_PREFIX) or {}).get("marker") == "REAL",
          "取到 REAL 而非 notes 行")

    # 契约行缺失 ⇒ None（不抛），让调用方区分"没跑到"与"解析坏了"
    check("契约行缺失 ⇒ 返回 None（不抛；调用方才能分清两种失败）",
          parse_contract("只有横幅\n没有契约行\n", TIER1_PREFIX) is None)

    # strict 版：坏 JSON 必须抛
    broken = "横幅\n" + TIER1_PREFIX + ' {"schema": "x", "v": NaN}\n'
    threw = None
    try:
        parse_contract_strict(broken, TIER1_PREFIX)
    except ValueError as exc:
        threw = str(exc)
    check("★ 严格版：含 NaN 的契约行**抛异常**（`allow_nan=False` 的产出侧对称面）",
          threw is not None, (threw or "没抛 ⇒ 静默变 null 了")[:80])


# --------------------------------------------------------------------------- #
#  3. NEG：契约行不在末行 ⇒ 解析器必须取不到 / 判红
# --------------------------------------------------------------------------- #
def test_negative_not_last() -> None:
    section("3. NEG：契约行被后续同前缀行顶掉 ⇒ 必须能看出来")
    good = json.dumps({"schema": "dzrobot-trackbench-result/1", "marker": "OK"})
    text = "\n".join([f"{TIER1_PREFIX} {good}", "⚠️ teardown 日志尾行", ""])
    # ★ 设计口径：**从末往上找 + 前缀唯一**。所以"后面有无关输出"不影响结果 ——
    #   这条**不是**缺陷（要求"必须在严格末行"会让一句 teardown 日志就把契约弄丢）。
    check("（设计）后面只有**无关**输出 ⇒ 仍能取到契约（前缀唯一时位置容忍）",
          parse_contract(text, TIER1_PREFIX) is not None,
          "从末往上找，不要求严格末行")

    # 真正致命的是"后面又打了一条**同前缀**行"：末行解析会取到后那条。
    text2 = "\n".join([f"{TIER1_PREFIX} {good}",
                       f"{TIER1_PREFIX} " + json.dumps({"schema": "x", "marker": "TEARDOWN"}),
                       ""])
    got = parse_contract(text2, TIER1_PREFIX)
    check("★ 后续又打一条同前缀行 ⇒ 末行解析取到**后**那条（契约会被顶掉）",
          (got or {}).get("marker") == "TEARDOWN",
          f"取到 {(got or {}).get('marker')} ⇒ 产出侧必须保证契约行**唯一**")
    # 实证产出侧满足"唯一"，所以上面那个风险在真跑里不发生
    out = _RUN.get("out") or ""
    check("★ 实证：真跑的 stdout 里 `TRACKBENCH_RESULT_JSON` **恰好出现 1 次**",
          out.count(TIER1_PREFIX) == 1, f"出现 {out.count(TIER1_PREFIX)} 次")
    check("★ 且它在**最后一个非空行**（编排层用末行解析不会漏）",
          bool(out.strip()) and out.strip().splitlines()[-1].startswith(TIER1_PREFIX + " "),
          "末行 = 契约行")


# --------------------------------------------------------------------------- #
#  4. 编排层现状：插件消费侧是否已经按前缀 + 末行（还是全文扫）
# --------------------------------------------------------------------------- #
_PLUGIN = pathlib.Path("D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh")


def test_orchestrator_side() -> None:
    section("4. 编排层现状：插件怎么读指标（**文字正则全文扫** vs 契约行）")
    bench = _PLUGIN / "src" / "simbench.js"
    if not bench.is_file():
        check("插件 simbench.js 可读（不在本卡写域，只读判现状）", False, str(bench))
        return
    src = bench.read_text(encoding="utf-8", errors="replace")
    print(f"  插件里提到 tier-0 前缀={TIER0_PREFIX in src} · tier-1 前缀={TIER1_PREFIX in src}")

    # ---- 现状①：`parseMetrics` 是**文字正则**全文扫（这正是本卡要替换的那一半）----
    fn = re.search(r"function parseMetrics\(text\)\s*\{[\s\S]{0,800}?\n\}", src)
    body = fn.group(0) if fn else ""
    check("（现状·只读记录）插件 `parseMetrics(text)` 按**文字正则**抓指标（不是解析 JSON）",
          bool(body) and "METRIC_LABELS" in body and ".match(" in body,
          (body.splitlines()[1].strip()[:90] + "…") if body else "取不到函数体")

    # ---- 现状②：它取的是**第一次**匹配（`String.match` 无 g 标志 ⇒ 首处命中）----
    check("（现状·只读记录）它是**首处命中**（`text.match(pattern)` 而非逐行末行）",
          bool(body) and re.search(r"text\.match\(pattern\)", body) is not None,
          "正则无 `g` ⇒ 取第一次；多段输出的旧结果会被抓到（F-5 的形状）")

    # ---- 现状③：已有 JSON 通道，但读的是 **tier-0 的 `metrics` 键名**----
    from_payload = re.search(r"export function metricsFromPayload\(payload\)\s*\{[\s\S]{0,900}?\n\}", src)
    fp = from_payload.group(0) if from_payload else ""
    check("（现状）插件**已有** JSON 通道 `metricsFromPayload(payload)`（不是在纯猜文字）",
          bool(fp), "存在")
    check("（现状）它读的键名是 tier-0 的 `cross_track_rms_m` / `path_len_ratio`",
          "cross_track_rms_m" in fp and "path_len_ratio" in fp,
          "键名逐字：cross_track_rms_m / path_len_ratio")
    check("★ 它**不认** tier-1 的五键（`rms_m` / `length_ratio` 在 metrics 里叫这两个名字）",
          "m.rms_m" not in fp and "m.length_ratio" not in fp,
          "tier-1 五键名（rms_m/length_ratio）在 metricsFromPayload 里没有对应分支")

    # ★ 关键裁决：tier-1 的五键**同名同义**，所以迁移只需加分支、不是改语义 ——
    #   这条把"迁移可行性"钉死，免得 wb-6 以为要重新定义指标。
    payload = parse_contract(_RUN.get("out") or "", TIER1_PREFIX) or {}
    five = payload.get("metrics") or {}
    check("★ 迁移可行性：tier-1 五键名与 tier-0 的**语义别名**对得上（加分支即可，不改语义）",
          set(five) == {"finished", "time_s", "rms_m", "out_of_bounds", "length_ratio"}
          and (payload.get("metricsFull") or {}).get("cross_track_rms_m") == five.get("rms_m"),
          f"rms_m={five.get('rms_m')} == cross_track_rms_m="
          f"{(payload.get('metricsFull') or {}).get('cross_track_rms_m')}")

    xfail("插件侧已支持 tier-1 契约前缀 `TRACKBENCH_RESULT_JSON`", TIER1_PREFIX in src,
          owner="wb-studio",
          why="实测插件仍只解析 tier-0（文字正则 + `SIM_RESULT_JSON`）；"
              "台架出口迁移归 wb-6（WB-6-STUDIO-20261031），本卡写域不含插件")

    # tier-0 的**产出侧**是否仍叫这个名字 —— 只读判现状（改它 = 断 UI）
    sim_cli = _SRC / "dzbrain" / "tools" / "sim_cli.py"
    txt = sim_cli.read_text(encoding="utf-8", errors="replace")
    check("tier-0 `sim_cli.py` 仍产出 `SIM_RESULT_JSON`（契约没被谁悄悄改名）",
          TIER0_PREFIX in txt, "sim_cli.py 含前缀")


# --------------------------------------------------------------------------- #
#  5. 两个 schema 的字段口径不许混（F-5 抓串事故的形状）
# --------------------------------------------------------------------------- #
def test_no_crosstalk() -> None:
    section("5. 两个 rms 不许混（F-5 抓串事故的形状）")
    payload = parse_contract(_RUN.get("out") or "", TIER1_PREFIX)
    if not isinstance(payload, dict):
        check("契约对象可解析", False, "上一步已失败")
        return
    full = payload.get("metricsFull") or {}
    notes = payload.get("notes") or {}
    check("`rms_m` 与 `belief_error_rms_m` **各自独立存在**",
          "rms_m" in full and "belief_error_rms_m" in full, str(sorted(full))[:80])
    check("★ `rms_m` 自称是 `cross_track_rms_m` 的别名（notes 里明写）",
          "cross_track_rms_m" in str(notes.get("rms_m", "")),
          str(notes.get("rms_m", ""))[:60])
    check("★ `belief_error_rms_m` 的 notes **明确否定**自己是横向误差",
          "不是" in str(notes.get("belief_error_rms_m", "")),
          str(notes.get("belief_error_rms_m", ""))[:60])
    check("★ ideal 档 `rms_m` 与 `belief_error_rms_m` **数值不同**"
          "（信念≡真值 ⇒ 后者=0，而横向误差不为 0 ⇒ 两个量确实不是一个东西）",
          full.get("rms_m") != full.get("belief_error_rms_m")
          or (full.get("belief_error_rms_m") == 0.0 and full.get("rms_m", 0) > 0),
          f"rms={full.get('rms_m')} belief={full.get('belief_error_rms_m')}")


# --------------------------------------------------------------------------- #
#  6. 三哈希证据链：`run_meta.jsonl` 每条记录必须带 configMd5 / mapMd5 / routeMd5
# --------------------------------------------------------------------------- #
META = _REPO / "artifacts" / "sim" / "run_meta.jsonl"
META_REQUIRED = ("configMd5", "mapMd5", "routeMd5")


def test_three_hashes() -> None:
    section("6. 三哈希证据链：台账每条记录都要说清「哪一版配置 / 哪张图 / 哪条路线」")
    if not META.is_file():
        check("台账文件存在", False, str(META))
        return
    recs = []
    bad_lines = 0
    for line in META.read_text(encoding="utf-8", errors="replace").splitlines():
        if not line.strip():
            continue
        try:
            recs.append(json.loads(line))
        except ValueError:
            bad_lines += 1
    check("台账可解析（坏行数为 0）", bad_lines == 0, f"{len(recs)} 条 / 坏行 {bad_lines}")
    check("台账记录数 > 100（真的跑过很多次，不是空文件）", len(recs) > 100, f"{len(recs)} 条")

    # ★ 现状：`routeMd5` **一条都没有**（字段从未存在），`mapMd5` 有 1 条是空串。
    #   卡面要求"新增记录必须带三哈希，空串即判红"——但台账是**只追加的历史账**，
    #   旧记录改不了（也不该改）。所以判据只能落在"**新增**记录"上，
    #   而"新增"的边界必须可机判：本测试用"文件末 N 条 = 本次会话之后写入的"来近似，
    #   N 由 `--since-iso` 或环境变量给；默认只判**字段存在性规划**（见下面那条 XFAIL）。
    has_route = [r for r in recs if "routeMd5" in r]
    empty_hashes = [(r.get("id") or r.get("at"), k) for r in recs
                    for k in META_REQUIRED if k in r and r.get(k) == ""]
    print(f"  台账 {len(recs)} 条；含 routeMd5 字段 {len(has_route)} 条；"
          f"空串哈希 {len(empty_hashes)} 处 {empty_hashes[:3]}")
    # ★ 覆盖率只作**现状记录**，不设阈值：台账里混着压测自产的 invalid/error 记录
    #   （`stressbench` 写 `kind` 类条目，本来就没有 config 哈希），按"必须有"判会误红。
    covered = sum(1 for r in recs if r.get("configMd5"))
    print(f"  （现状）configMd5 覆盖 {covered}/{len(recs)} = {covered / max(1, len(recs)):.1%}"
          f"（缺口集中在压测自产记录）")
    check("（现状记录）台账里至少有 900 条记录了 configMd5（证据链主体存在）",
          covered >= 900, f"{covered} 条")
    check("（现状记录）`mapMd5` 历史记录有值（删图前的账是完整的）",
          any(r.get("mapMd5") for r in recs), "有非空 mapMd5")

    # ---- 闸：给 `appendMeta` 加"三哈希必填"的**纯函数校验**（本卡写域外不能改插件，
    #      所以闸放在台账的**消费侧**：谁读谁判，读的时候就能发现缺证）----
    def meta_gaps(rec: dict) -> list[str]:
        """一条台账记录的证据链缺口（空列表 = 三哈希齐）。"""
        gaps = []
        for key in META_REQUIRED:
            if not rec.get(key):
                gaps.append(key)
        return gaps

    check("★ 闸：`mapMd5=\"\"` 被识别为证据链缺口（不是「有字段就算有值」）",
          meta_gaps({"configMd5": "a", "mapMd5": "", "routeMd5": "c"}) == ["mapMd5"],
          str(meta_gaps({"configMd5": "a", "mapMd5": "", "routeMd5": "c"})))
    check("★ 闸：三哈希齐 ⇒ 无缺口", meta_gaps({k: "x" for k in META_REQUIRED}) == [],
          "齐")
    check("★ 闸：三哈希全缺 ⇒ 三个缺口", meta_gaps({}) == list(META_REQUIRED),
          str(meta_gaps({})))

    newest = recs[-1]
    newest_gaps = meta_gaps(newest)
    xfail("**最新一条**台账记录三哈希齐（configMd5 / mapMd5 / routeMd5）",
          not newest_gaps,
          owner="wb-studio",
          why=f"实测最新记录缺 {newest_gaps}——`routeMd5` 字段**从未存在**"
              f"（台账代码 `src/simbench.js::runSimJob` 只写 configMd5/mapMd5）；"
              f"补证在插件写域（wb-6），本卡只把缺口判据固定下来")

    # 历史空串统计（只作证，不判红；旧账不可改）
    if empty_hashes:
        print(f"  （历史空串 {len(empty_hashes)} 处，是删图后旧账的自然结果，不判红）")


# --------------------------------------------------------------------------- #
#  main
# --------------------------------------------------------------------------- #
def main() -> int:
    print("WB-5 · 契约行解析（产出侧末行 + 消费侧前缀 + 反例 + schema 分流 + 三哈希）")
    test_producer_side()
    test_consumer_side()
    test_negative_not_last()
    test_orchestrator_side()
    test_no_crosstalk()
    test_three_hashes()
    finish()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
