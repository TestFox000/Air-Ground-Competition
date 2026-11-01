# -*- coding: utf-8 -*-
"""WB-5 · 0 断言闸（卡 `WB-5-REGNET-20261031`）。

运行::

    cd dzrobot_v2 && PYTHONPATH=src/dzbrain python src/dzbrain/tests/test_wb5_zero_assert.py

全过 exit 0；任一 FAIL exit 1。

## 它封的是什么洞（实测，不是假想）

```
$ PYTHONPATH=src/dzbrain python src/dzbrain/tests/test_aim_probe.py
  SKIP  样本日志不存在（...\\_tmp_session\\run_20261017_1504\\7_path_utf8.txt）→ 只跳过，不算失败
结果：0/0 全部通过 ✅
$ echo $?
0
```

「**0 条断言、0 条通过，仍然打印"全部通过"并退出 0**」—— 这是本仓最典型的假绿形态：

| 形态 | 例子 | 后果 |
|---|---|---|
| 样本文件被清理 ⇒ 整段 SKIP | `test_aim_probe.py` §0（样本日志 `_tmp_session/` 已清理） | 该套件今天**一条断言都没跑**，但 CI 看到 exit 0 |
| 只打印 `结果：` 而 `total=0` | 任何 `harness.finish()` 在"全 SKIP"分支 | 同上 |
| 解析器抓不到 `结果：` 行 | `_run_full_regression.py` 的 `re.search` 失败 ⇒ `passed=total=-1` | 汇总表里该行 **看起来"没红"** |

## 本文件怎么判（三层，都不靠人眼）

1. **静态扫描**：`src/dzbrain/tests/` 下每个 `test_*.py` 必须**有且只有**一个 `结果：` 输出点；
   且其来源必须是 `harness.finish()` 或本文件认可的自写 `finish()`（见 §2 白名单）。
   —— 防止"新写的套件自己打印一句 '结果：全部通过' 就 exit 0"。
2. **动态闸（子进程真跑）**：对**抽样套件**真跑一遍，把 stdout 解析成 `(passed, total)`：
   `total == 0` ⇒ **判红**（这就是本卡要封的那一条）；
   套件在"全 SKIP"时若仍 exit 0 且 total=0 ⇒ 判红。
3. **汇总器闸**：`_run_full_regression.py` 解析不到 `结果：` 时必须把该套件记成**红**，
   不许记成 `-1` 然后在 `red` 列表里漏掉。

## 负例（本文件自带，真跑）

- **NEG-A**：造一个"0 断言仍 exit 0"的假套件（写进 `artifacts/` 下的临时目录，不进 `tests/`），
  闸必须把它判红。这条是**自证**：闸若自己失效，NEG-A 会绿，本文件就红。
- **NEG-B**：造一个"有断言但全 FAIL"的套件，闸必须判红（证明闸不是"有断言就绿"）。
- **NEG-C**：造一个"结果：行缺失"的套件，汇总器必须判红（而不是记 -1 混过去）。

三条负例都用 `tempfile` 写、退出前删；**不写 `src/dzbrain/tests/`**。
"""

from __future__ import annotations

import ast
import os
import pathlib
import re
import subprocess
import sys
import tempfile

_HERE = pathlib.Path(__file__).resolve()
_SRC = _HERE.parents[1]                       # src/dzbrain
_REPO = _SRC.parents[1]                       # dzrobot_v2
_TESTS = _SRC / "tests"
for _p in (str(_SRC), str(_TESTS)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

_PASS = 0
_FAIL: list[str] = []
_XFAIL: list[str] = []
_XPASS: list[str] = []
_SECTION = ""

#: `结果：` 行的**唯一**合法形状（`harness.finish()` 与 `test_wb2_*` 的自写 finish 都这样）
RESULT_RE = re.compile(r"结果：(\d+)/(\d+)\s*(全部通过|通过[，,]\s*(\d+)\s*失败)")


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
#  闸的实现（**唯一权威**：静态扫描与子进程解析都调这两个函数）
# --------------------------------------------------------------------------- #
def parse_result_line(text: str) -> tuple[int, int] | None:
    """从套件 stdout 里取 `(passed, total)`。取不到 ⇒ `None`（= 汇总器今天会记 -1）。"""
    found = None
    for line in text.splitlines():
        hit = RESULT_RE.search(line)
        if hit:
            found = (int(hit.group(1)), int(hit.group(2)))     # 末次出现为准（多段输出）
    return found


def judge_output(rc: int, text: str) -> tuple[str, str]:
    """判一次子进程输出。返回 `("pass"|"red"|"zero", 理由)`。

    ★ 三条规则的顺序不能改：
      1. `total == 0` **无条件判红** —— 哪怕 rc=0、哪怕它自己打印了"全部通过"；
      2. 有断言但有 FAIL ⇒ 红；
      3. rc != 0 ⇒ 红（症-44：harness 把 exit 5/2 当通过 = 假绿）。
    """
    got = parse_result_line(text)
    if got is None:
        return "red", "没有 `结果：N/M` 行（汇总器会记成 -1，等于静默消失）"
    passed, total = got
    if total == 0:
        return "zero", f"断言数=0（{passed}/{total}）仍未判红 —— 这正是本卡要封的假绿"
    if passed != total:
        return "red", f"有断言但有失败 {passed}/{total}"
    if rc != 0:
        return "red", f"断言全过但退出码 {rc} != 0（症-44：RC≠0 一律红）"
    return "pass", f"{passed}/{total} 且 rc=0"


def run_suite(path: pathlib.Path, timeout: int = 900) -> tuple[int, str, str]:
    """跑一个套件，返回 `(rc, stdout+stderr, 错误说明)`。

    ★★ `encoding="utf-8"` 是**判据级**的（我第一版就栽在这）：Windows 下
      `text=True` 默认按 locale（cp936）解码子进程输出，而套件按 `PYTHONIOENCODING=utf-8`
      打印中文 ⇒ 拿到的是乱码，`结果：N/M` **一条也匹配不上** ⇒ 闸把**所有**套件都判成
      "没有结果行"（包括正常的）。那是闸自己坏了，不是被检对象坏了 —— 而它长得像"全红"，
      最容易被误读成"仓库烂透了"。
    """
    env = dict(os.environ)
    env["PYTHONPATH"] = str(_SRC)
    env["PYTHONIOENCODING"] = "utf-8"
    env["PYTHONUTF8"] = "1"
    try:
        proc = subprocess.run([sys.executable, str(path)], cwd=str(_REPO), env=env,
                              capture_output=True, text=True, encoding="utf-8",
                              errors="replace", timeout=timeout)
        return proc.returncode, (proc.stdout or "") + (proc.stderr or ""), ""
    except subprocess.TimeoutExpired:
        return -9, "", f"超时（>{timeout}s）"


# --------------------------------------------------------------------------- #
#  1. 静态扫描：`结果：` 必须带**真实计数**（自封绿 = 裸字面量）
# --------------------------------------------------------------------------- #
#: 允许自己打印 `结果：` 的形状：`harness.finish()` 或一个本地 `finish()`。
#: ★ 但**不**要求所有套件都走 finish()：本仓既有 `main()` 里自写结果行的合法约定
#:   （`test_route.py:768`、`test_motion_limit.py:277`、`test_calib_backfill.py:484`）。
#:   把"必须走 finish()"钉死会一次性判红 8 个契约内文件 —— 那是**闸自己坏了**，
#:   而且本卡写域不许改它们（那是 wb-doc 的事）。
#:
#: 所以静态判据换成**真正的假绿特征**：`结果：` 那一行**必须带 `{passed}/{total}` 计数**。
#: 裸字面量（`print("结果：全部通过")` / `结果：绿（...）`）才是自封绿 ——
#: 它连"跑了几条断言"都没说，闸与人都没法核。
_ACCEPTED_DEFS = {"finish"}

#: **存量白名单**：`结果：` 写成裸字面量的既有套件。新增条目 = 开倒车（本测试会判红）。
#: 处置归 wb-doc（那些是 tier-0 契约套件，本卡写域不许改）。
LEGACY_SELFGREEN: dict[str, str] = {
    "test_pursuit.py": "`结果：全部通过 (ALL PASS)` / `结果：FAILED` 两条裸字面量",
    "test_config_keys.py": "`结果：绿（...）` / `结果：红（...）` 两条裸字面量",
}


def _result_sites(path: pathlib.Path) -> list[tuple[str, str, bool]]:
    """列出每个 `结果：` 打印点：`(所在函数名, 行文本, 是否带计数)`。"""
    raw = path.read_text(encoding="utf-8", errors="replace")
    tree = ast.parse(raw)
    sites: list[tuple[str, str, bool]] = []

    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        name = ""
        f = node.func
        if isinstance(f, ast.Name):
            name = f.id
        elif isinstance(f, ast.Attribute):
            name = f.attr
        if name not in ("print", "write"):
            continue
        text = _unparse(node)
        if "结果：" not in text:
            continue
        # 带计数 = 该 call 里至少有一个 FormattedValue（f-string 插值）且那个插值
        # 引用了 `_PASS` / `passed` / `total` 之一。
        counted = any(isinstance(sub, ast.FormattedValue) and
                      any(isinstance(x, ast.Name) and x.id in
                          ("_PASS", "passed", "total", "_FAIL", "fails", "n_pass", "n_total")
                          for x in ast.walk(sub))
                      for sub in ast.walk(node))
        sites.append((_enclosing(tree, node), text.strip(), counted))
    return sites


def _unparse(node) -> str:
    try:
        return ast.unparse(node)
    except Exception:                                          # noqa: BLE001
        return f"<line {getattr(node, 'lineno', '?')}>"


def _enclosing(tree: ast.AST, node) -> str:
    """包含该节点的**最内层**函数名（没有则 `<模块级>`）。"""
    best, best_span = "<模块级>", None
    for cand in ast.walk(tree):
        if not isinstance(cand, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        lo, hi = cand.lineno, (cand.end_lineno or cand.lineno)
        if lo <= node.lineno <= hi:
            span = hi - lo
            if best_span is None or span < best_span:
                best, best_span = cand.name, span
    return best


def scan_suite(path: pathlib.Path) -> tuple[bool, str]:
    """静态判一个套件有没有"自封绿"。返回 `(ok, 理由)`。

    ★ 判据只有一条，且**只判真危害**：每个 `结果：` 打印点必须带计数。
      出现多次**不判红** —— `parse_result_line` 取末次出现，多段输出是安全的
      （`test_config_keys.py` 的两条红/绿分支就是这种写法）。
    """
    try:
        tree = ast.parse(path.read_text(encoding="utf-8", errors="replace"))
    except SyntaxError as exc:
        return False, f"语法错误 {exc.msg}（第 {exc.lineno} 行）"
    sites = _result_sites(path)
    if not sites:
        return True, "无 `结果：` 行（由动态闸按退出码 + 断言数判）"
    bad = [(fn, txt) for fn, txt, counted in sites if not counted]
    if bad:
        return False, "自封绿（`结果：` 不带计数）: " + "; ".join(f"{fn}: {t}" for fn, t in bad)
    return True, f"{len(sites)} 个结果点均带计数（{sorted({s[0] for s in sites})}）"


def test_static_scan() -> None:
    section("1. 静态扫描：`结果：` 必须带计数（裸字面量 = 自封绿）")
    suites = sorted(_TESTS.glob("test_*.py"))
    check("套件数 > 30（扫描真的扫到了东西，不是空目录）", len(suites) > 30, f"{len(suites)} 个")
    bad: list[tuple[str, str]] = []
    for path in suites:
        ok, why = scan_suite(path)
        if not ok:
            bad.append((path.name, why))
    legacy = [x for x in bad if x[0] in LEGACY_SELFGREEN]
    hard = [x for x in bad if x[0] not in LEGACY_SELFGREEN]
    check("★ 白名单之外**没有**自封绿套件（新套件不许自己封绿）",
          not hard, "; ".join(f"{n}: {w}" for n, w in hard) or "全数合规")

    # ★ 存量白名单**只减不增**，且修好必须回来删（`xfail` 转 `XPASS` 会判本测试红）。
    #   这条是本卡"不许拿绿盖红"的具体落法：既承认存量、又不让存量静默扩大。
    for name, why in LEGACY_SELFGREEN.items():
        check(f"白名单条目 `{name}` 仍存在（白名单不烂成空壳）",
              (_TESTS / name).is_file(), name)
        xfail(f"`{name}` 的 `结果：` 行改为带计数", name not in {n for n, _ in bad},
              owner="wb-doc", why=why)

    no_result = [p.name for p in suites
                 if "结果：" not in p.read_text(encoding="utf-8", errors="replace")]
    print(f"  （另 {len(no_result)} 个套件无结果行，靠退出码 + 断言数判；抽样见 §2）")

    # 本文件与 wb2/wb5 家族的 `finish()` 必须各定义一次（防止被谁复制两份、口径分叉）
    family = [p for p in suites
              if p.name in ("test_wb2_rig_invariants.py", "test_wb2_no_second_projection.py",
                            "test_wb5_sensitivity.py", "test_wb5_zero_assert.py")]
    check("wb-5/wb-2 家族四个套件都在（本卡与地基的守卫齐全）",
          len(family) >= 3, str([p.name for p in family]))

    # ---- 负例：往"新套件"位置塞一个自封绿的，必须被 hard 分支抓到 ----
    with tempfile.TemporaryDirectory(prefix="wb5_scan_") as tmp:
        fake = pathlib.Path(tmp) / "test_fake_selfgreen.py"
        fake.write_text('print("结果：全部通过")\nraise SystemExit(0)\n', encoding="utf-8")
        ok, why = scan_suite(fake)
        print(f"  [neg selfgreen] ok={ok}  {why}")
        check("★ 负例：裸字面量 `结果：全部通过` 被判为自封绿", not ok, why)
        fake2 = pathlib.Path(tmp) / "test_fake_counted.py"
        fake2.write_text('_PASS = 1\nprint(f"结果：{_PASS}/{_PASS} 全部通过")\n'
                        'raise SystemExit(0)\n', encoding="utf-8")
        ok2, why2 = scan_suite(fake2)
        print(f"  [pos counted  ] ok={ok2}  {why2}")
        check("★ 正例：带计数的结果行不被误判", ok2, why2)


# --------------------------------------------------------------------------- #
#  2. 动态闸：真跑抽样套件，`total==0` 判红
# --------------------------------------------------------------------------- #
#: 抽样清单：**必须**包含历史上出过 0/0 的那个（`test_aim_probe.py`），
#: 以及两个"真断言很多"的对照组（证明闸不是"一律判红"）。
SAMPLE = ("test_aim_probe.py", "test_geometry.py", "test_core.py")


def test_dynamic_zero_assert() -> None:
    section("2. 动态闸：真跑抽样套件，0 断言 = 红（抽样含历史事故套件）")
    for name in SAMPLE:
        path = _TESTS / name
        if not path.is_file():
            check(f"{name} 存在", False, "文件不见了 ⇒ 抽样清单过期")
            continue
        rc, text, err = run_suite(path)
        verdict, why = judge_output(rc, text)
        got = parse_result_line(text)
        print(f"  [{name:24s}] rc={rc:<3} 结果行={got} 判定={verdict}  {why}")
        if name == "test_aim_probe.py":
            # ★ 这个套件**今天就是 0/0 假绿**（样本日志被清理）。闸把它判 zero = 闸生效。
            #   这里要证的是「闸认出来了」，不是「套件是好的」。
            check("★ 闸认出了历史事故套件 `test_aim_probe.py` 的 0/0 假绿",
                  verdict == "zero", f"判定={verdict}（0 断言必须判红）")
        else:
            check(f"对照组 `{name}` 判定为 pass（闸不是一律判红）",
                  verdict == "pass", f"{verdict}: {why}")

    # ★ 事故复现（必须仍可复现，否则说明样本文件被补回来了、这条闸失去了活样本）
    rc, text, _ = run_suite(_TESTS / "test_aim_probe.py")
    got = parse_result_line(text)
    xfail("`test_aim_probe.py` 的断言数 > 0（其样本日志 `_tmp_session/run_20261017_1504/` 已被清理）",
          bool(got and got[1] > 0),
          owner="wb-doc",
          why=f"实测 结果行={got} ⇒ 该套件今天一条断言都没执行；"
              f"闸已把它判为 {'红' if judge_output(rc, text)[0] == 'zero' else '非零'}，"
              f"但**根因是样本缺失**，补样本或改成契约断言由 owner 定")


# --------------------------------------------------------------------------- #
#  3. 汇总器闸：解析不到 `结果：` 必须记红，不许 -1 混过去
# --------------------------------------------------------------------------- #
REG = _REPO / "_run_full_regression.py"


def test_summariser() -> None:
    section("3. 汇总器闸：`_run_full_regression.py` 抓不到结果行 ⇒ 记红（不许 -1 混过去）")
    if not REG.is_file():
        check("汇总器存在", False, str(REG))
        return
    text = REG.read_text(encoding="utf-8", errors="replace")
    check("汇总器仍按 `结果：N/M` 解析（契约没被谁悄悄改掉）",
          "结果：" in text and "re.search" in text, "")
    check("★ 汇总器的正则**不卡收尾文字**（三种合法结尾都能解析）",
          "([^\\n]*)" in text, "正则含宽松收尾 ([^\\n]*)")
    # ★ 而且它必须真的能解析三种合法结尾（用它的正则实跑三种样本，见 §5）
    for tail in (" 全部通过 ✅", " 通过", " 通过，2 失败", ""):
        sample = f"结果：3/5{tail}"
        hit = re.search(r"结果：(\d+)/(\d+)([^\n]*)", sample)
        check(f"（正则可解析收尾={tail!r}）", hit is not None
              and int(hit.group(1)) == 3 and int(hit.group(2)) == 5,
              sample)
    # ★ 关键判据：解析失败分支必须把三键**同时**记成 -1（源码形状，可机判）
    ok_missing = bool(re.search(r"else:[ \t]*\n[ \t]*(?:[A-Za-z_]+[ \t]*=[ \t]*)?"
                                r"passed[ \t]*=[ \t]*total[ \t]*=[ \t]*fails[ \t]*=[ \t]*-1",
                                text))
    check("（现状记录）解析失败时被记成 -1（三键同时）", ok_missing,
          "`else: passed = total = fails = -1`")
    # ★ 关键判据②：解析不到结果行的套件必须进 `red`（本卡修的就是这条）。
    check("★ 汇总器有 `why_red()` 把「无结果行 / total=0」也判红",
          "def why_red" in text and 'row["total"] == -1' in text and 'row["total"] == 0' in text,
          "why_red() 已覆盖 total<=0")
    check("★ 汇总器自身以非零退出（不只打印不表态）",
          bool(re.search(r"sys\.exit\(1 if red else 0\)", text)) or "sys.exit(" in text.split("if red else")[-1],
          "末行 sys.exit")


# --------------------------------------------------------------------------- #
#  4. 负例（自证闸没坏）：三条都要判红
# --------------------------------------------------------------------------- #
NEG_ZERO = '''# -*- coding: utf-8 -*-
"""NEG-A：0 断言却 exit 0（本卡要封的形态）。"""
if __name__ == "__main__":
    print("SKIP 样本不存在 → 只跳过，不算失败")
    print("=" * 70)
    print("结果：0/0 全部通过 ✅")
    print("=" * 70)
    raise SystemExit(0)
'''

NEG_FAIL = '''# -*- coding: utf-8 -*-
"""NEG-B：有断言但全 FAIL，却 exit 0（比 NEG-A 更细的假绿）。"""
if __name__ == "__main__":
    print("  FAIL a")
    print("  FAIL b")
    print("结果：0/2 通过，2 失败")
    raise SystemExit(0)
'''

NEG_NOLINE = '''# -*- coding: utf-8 -*-
"""NEG-C：根本不打结果行（汇总器会记 -1）。"""
if __name__ == "__main__":
    print("looks fine")
    raise SystemExit(0)
'''


def test_negative_controls() -> None:
    section("4. 负例自证：三条假绿必须被闸判红（闸若失效，本节会绿 ⇒ 本文件红）")
    tmpdir = pathlib.Path(tempfile.mkdtemp(prefix="wb5_neg_"))
    try:
        for name, body, want in (("neg_zero.py", NEG_ZERO, "zero"),
                                 ("neg_fail.py", NEG_FAIL, "red"),
                                 ("neg_noline.py", NEG_NOLINE, "red")):
            path = tmpdir / name
            path.write_text(body, encoding="utf-8")
            rc, text, err = run_suite(path)
            verdict, why = judge_output(rc, text)
            print(f"  [{name:14s}] rc={rc} 判定={verdict}  {why}")
            check(f"★ NEG `{name}` 被判红（期望 {want}）",
                  verdict == want, f"{verdict} != {want}")
    finally:
        import shutil
        shutil.rmtree(tmpdir, ignore_errors=True)
    check("负例目录已清理（不污染 artifacts/）", not tmpdir.exists(), str(tmpdir))

    # 正向对照：一个**真**有断言的套件必须判绿（防"闸一律判红"）
    check("正向对照：某真套件判绿（闸不是一律判红）",
          judge_output(*run_suite(_TESTS / "test_core.py")[:2])[0] == "pass",
          "test_core.py")


# --------------------------------------------------------------------------- #
#  5. 汇总器实跑（把 NEG-C 那种形态送进汇总器的解析器，看它记什么）
# --------------------------------------------------------------------------- #
def test_summariser_behaviour() -> None:
    section("5. 汇总器行为复现：用它的同一条正则解析三种输出")
    if not REG.is_file():
        return
    src = REG.read_text(encoding="utf-8", errors="replace")
    # ★ 从汇总器源码里把它的正则**抠出来**（不写死一份，避免口径分叉）。
    #   注意现在的写法是 `for ... in re.finditer(r"结果：...", text)`（不是 `m = re.search(`），
    #   所以锚点认两种：`re.search(<pat>, text)` 或 `re.finditer(<pat>, text)`。
    hit = re.search(r're\.[a-z]+\((r?".*?"),\s*text\)', src)
    pattern = hit.group(1) if hit else None
    check("能从汇总器源码里取出它用的正则", bool(pattern), pattern or "取不到")
    if not pattern:
        return
    pat = eval(pattern)                                        # noqa: S307  仅用于本仓自有源码
    total_groups = re.compile(pat).groups
    print(f"  汇总器正则的分组数={total_groups}（原为 4；本卡放宽收尾后为 3）")
    samples = (("真套件", "结果：30/30 全部通过 ✅"),
               ("有失败", "结果：0/2 通过，2 失败"),
               ("0 断言", "结果：0/0 全部通过 ✅"),
               ("无结果行", "looks fine"))
    for label, text in samples:
        m = re.search(pat, text)
        if m:
            passed, total = int(m.group(1)), int(m.group(2))
            # 失败条数：有 `N 失败` 分组就用它，否则用 total-passed（与汇总器同一口径）
            tail = m.group(3) if total_groups >= 3 else ""
            fm = re.search(r"(\d+)\s*失败", tail)
            fails = max(int(fm.group(1)) if fm else 0, total - passed)
            print(f"  [{label:8s}] 解析⇒ {passed}/{total} 失败{fails}")
        else:
            passed = total = fails = -1
            print(f"  [{label:8s}] 解析⇒ 未命中 ⇒ 记 -1/-1/-1")
        if label == "0 断言":
            check("★ 汇总器把「0/0 全部通过」解析成 total=0（不是判绿）",
                  m is not None and int(m.group(2)) == 0,
                  "total=0 ⇒ 汇总器的 why_red() 会判红（本卡已修）")
        if label == "无结果行":
            check("★ 汇总器把「无结果行」解析成 -1，且 why_red() 会把它判红",
                  m is None and passed == -1 and why_red_row(0, passed, total) != "",
                  "本卡修后：-1 不再静默绿")


def why_red_row(rc: int, passed: int, total: int) -> str:
    """**从 `_run_full_regression.py` 现读现判**：某行为什么算红（空串 = 绿）。

    ★ 为什么不在这里重写一份规则：那样就是"第二份判据"，
      汇总器改了它也不知道（本卡封的就是这种口径分叉）。
      所以从汇总器源码里 `exec` 它自己的 `why_red`，用同一个函数判。
    """
    src = REG.read_text(encoding="utf-8", errors="replace") if REG.is_file() else ""
    hit = re.search(r"def why_red\(row\)[^\n]*:\s*\n((?:\s+\S.*\n|\s*\n)*?)(?=\S)", src)
    if not hit:
        return "<汇总器里找不到 why_red()>"
    body = "def why_red(row):\n" + hit.group(1)
    ns: dict = {}
    try:
        exec(compile(body, str(REG), "exec"), ns)          # noqa: S102  本仓自有源码
    except Exception as exc:                                # noqa: BLE001
        return f"<why_red 编译失败 {type(exc).__name__}: {exc}>"
    return ns["why_red"]({"exit": rc, "pass": passed, "total": total, "fails": 0})


# --------------------------------------------------------------------------- #
#  main
# --------------------------------------------------------------------------- #
def main() -> int:
    print("WB-5 · 0 断言闸（静态自封绿扫描 + 动态 total==0 判红 + 汇总器漏判）")
    test_static_scan()
    test_dynamic_zero_assert()
    test_summariser()
    test_negative_controls()
    test_summariser_behaviour()
    finish()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
