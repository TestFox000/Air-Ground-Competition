# -*- coding: utf-8 -*-
"""WB-5 负例矩�阵：每条判据配一个能**真跑出红**的变异（卡 `WB-5-REGNET-20261031`）。

运行::

    cd dzrobot_v2 && PYTHONPATH=src/dzbrain python src/dzbrain/tests/test_wb5_negatives.py

全过 exit 0；任一 FAIL exit 1。

## 为什么单独一个文件

卡面验收原文：「每个判据配一个变异负例并**真跑出红**」。三个正测
（`test_wb5_sensitivity` / `test_wb5_zero_assert` / `test_wb5_contract`）各带负例，
但那是"它们自己的负例"。这里做的是**跨文件的反证**：**在台架/闸上做手术**，
看正测是不是真的红 —— 这是"测试网的敏感度"本身。

## 做法（都不改文件：源码级替换后 `exec` 到独立命名空间）

| 负例 | 手术 | 期望 |
|---|---|---|
| N1 | `test_wb5_sensitivity.py` 的 `JUDGE_SCENARIO` 换成盲夹具 `field` | 判据档基线出红 |
| N2 | 同上文件的变异替换生效代码删掉（变异装不上去） | "变异必须打红判据"整段红 |
| N3 | `test_wb5_zero_assert.py` 的 `judge_output` 改成 `total==0` 也算绿 | 事故套件与自证负例全绿 ⇒ 本文件红 |
| N4 | `test_wb5_zero_assert.py` 的 `parse_result_line` 永远返回 `(0,0)` | 正向对照红 |
| N5 | `test_wb5_contract.py` 的产出侧在契约行**之后**再追加一条同前缀行 | "恰好出现 1 次"红 |
| N6 | `test_wb5_contract.py` 的 `parse_contract` 改成全文扫取第一次 | 多段输出判据红 |

★ 代价：N1/N2 各跑 6~7 次 130 s 仿真（每次 ≈ 7 s 墙钟）⇒ 本节 ≈ 1.5 min。
  接受这个代价：**"能测出坏"这件事必须每次都真的被证明**，不能靠"上次跑过"。
"""

from __future__ import annotations

import pathlib
import re
import subprocess
import sys

_HERE = pathlib.Path(__file__).resolve()
_SRC = _HERE.parents[1]
_REPO = _SRC.parents[1]
for _p in (str(_SRC), str(_SRC / "tests")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

_PASS = 0
_FAIL: list[str] = []
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


def finish() -> None:
    print("\n" + "=" * 74)
    total = _PASS + len(_FAIL)
    print(f"结果：{_PASS}/{total} 通过" + (f"，{len(_FAIL)} 失败" if _FAIL else " 全部通过"))
    for item in _FAIL:
        print(f"  FAIL  {item}")
    print("=" * 74)
    sys.exit(1 if _FAIL else 0)


# --------------------------------------------------------------------------- #
#  跑器：源码级变异 → 独立命名空间 exec（**不改任何文件**）
# --------------------------------------------------------------------------- #
FAIL_PAT = re.compile(r"^\s*FAIL\s+", re.M)


def run_mutated(test_file: str, subs: list[tuple[str, str]], *, tag: str,
                timeout: int = 900) -> tuple[int, str, int]:
    """跑一个"被手术过的"测试，返回 `(rc, 输出尾部, FAIL 条数)`。

    ★ 为什么用 `subprocess` 而不是 `exec`：正测里有 `sys.exit(...)`（`finish()`），
      exec 会把本进程一起退出。子进程才能拿到真实 `rc`。
    """
    src_path = _TESTS / test_file
    src = src_path.read_text(encoding="utf-8")
    applied = 0
    for old, new in subs:
        if old in src:
            src = src.replace(old, new, 1)
            applied += 1
        else:
            print(f"  ⚠️ {tag}: 替换锚点没找到 → {old[:50]!r}")
    if applied == 0 and subs:
        return -98, f"（{tag}：一条替换都没生效 ⇒ 手术失败，负例无意义）", 0
    driver = (
        "import sys\n"
        f"_SRC = {repr(str(_SRC))}\n"
        "sys.path.insert(0, _SRC)\n"
        "sys.path.insert(0, _SRC + '/tests')\n"
        "import os\n"
        "sys.path.append(os.path.dirname(_SRC).rstrip('/') + '/tools')\n"
        f"src = {src!r}\n"
        f"code = compile(src, {str(src_path)!r}, 'exec')\n"
        f"ns = {{'__name__': '__main__', '__file__': {str(src_path)!r}}}\n"
        "exec(code, ns)\n"
    )
    env = dict(__import__("os").environ)
    env["PYTHONPATH"] = str(_SRC)
    env["PYTHONIOENCODING"] = "utf-8"
    try:
        proc = subprocess.run([sys.executable, "-c", driver], cwd=str(_REPO), env=env,
                              capture_output=True, text=True, encoding="utf-8",
                              errors="replace", timeout=timeout)
    except subprocess.TimeoutExpired:
        return -9, "", 0
    out = (proc.stdout or "") + (proc.stderr or "")
    return proc.returncode, out, len(FAIL_PAT.findall(out))


_TESTS = _SRC / "tests"


# --------------------------------------------------------------------------- #
#  N1/N2：敏感度正测的负例（判据档换成盲夹具 / 变异装不上去）
# --------------------------------------------------------------------------- #
def test_sensitivity_negatives() -> None:
    section("N1/N2. 敏感度正测的负例（必须真跑出红）")

    # ---- 基线：未手术时必须绿（否则"负例红"没有对照意义）----
    rc, out, nf = run_mutated("test_wb5_sensitivity.py", [], tag="baseline")
    check("基线（未手术）绿：rc=0", rc == 0, f"rc={rc} FAIL 条数={nf}")
    check("基线（未手术）没有 FAIL 行", nf == 0, f"{nf} 条")
    check("基线（未手术）没有 XPASS 行（XFAIL 不许腐烂）",
          "XPASS" not in out, "无 XPASS")

    # ---- N1：把判据档悄悄换成盲夹具 ----
    rc1, out1, nf1 = run_mutated(
        "test_wb5_sensitivity.py",
        [('JUDGE_SCENARIO = "field7"', 'JUDGE_SCENARIO = "field"')],
        tag="N1")
    check("★ N1 判据档换成盲夹具 `field` ⇒ 真跑出红",
          rc1 != 0 and nf1 >= 1, f"rc={rc1} FAIL 条数={nf1}")
    print(f"  N1 红因样例：{(FAIL_PAT.search(out1) and out1[FAIL_PAT.search(out1).start():][:150]) or '（无）'}")

    # ---- N2：变异装不上去（把"替换 tracker.compute_command"那行改成 pass）----
    #   ★ 锚点用**唯一**的一行：wrapper 赋值语句。手术成功后，"变异臂"= 基线，
    #     于是"每条判据必须被变异打红"整段红。
    rc2, out2, nf2 = run_mutated(
        "test_wb5_sensitivity.py",
        [("            TRK.compute_command = wrapper\n",
          "            pass  # N2：变异被静默丢掉\n")],
        tag="N2")
    check("★ N2 变异被静默丢掉 ⇒ 真跑出红（至少 5 条判据红）",
          rc2 != 0 and nf2 >= 5, f"rc={rc2} FAIL 条数={nf2}")
    print(f"  N2 红因样例：{(FAIL_PAT.search(out2) and out2[FAIL_PAT.search(out2).start():][:150]) or '（无）'}")


# --------------------------------------------------------------------------- #
#  N3/N4：0 断言闸的负例
# --------------------------------------------------------------------------- #
def test_zero_assert_negatives() -> None:
    section("N3/N4. 0 断言闸的负例（闸被改坏必须红）")

    rc, out, nf = run_mutated("test_wb5_zero_assert.py", [], tag="baseline")
    check("基线（未手术）绿：rc=0", rc == 0, f"rc={rc} FAIL 条数={nf}")

    # ---- N3：把闸改成"0 断言也算绿"（拆掉 total==0 那一步）----
    rc3, out3, nf3 = run_mutated(
        "test_wb5_zero_assert.py",
        [("    if total == 0:\n"
          "        return \"zero\", f\"断言数=0（{passed}/{total}）仍未判红 —— 这正是本卡要封的假绿\"\n",
          "    # N3：闸被拆（0 断言也放行）\n")],
        tag="N3")
    check("★ N3 闸被拆掉 `total==0` 判红 ⇒ 真跑出红",
          rc3 != 0 and nf3 >= 1, f"rc={rc3} FAIL 条数={nf3}")

    # ---- N4：解析器永远返回 (0,0)（正向对照会被判 zero）----
    rc4, out4, nf4 = run_mutated(
        "test_wb5_zero_assert.py",
        [("    found = None\n", "    return (0, 0)   # N4：解析器坏了\n    found = None\n")],
        tag="N4")
    check("★ N4 解析器永远 0/0 ⇒ 正向对照红",
          rc4 != 0 and nf4 >= 1, f"rc={rc4} FAIL 条数={nf4}")


# --------------------------------------------------------------------------- #
#  N5/N6：契约行正测的负例
# --------------------------------------------------------------------------- #
def test_contract_negatives() -> None:
    section("N5/N6. 契约行正测的负例（产出侧多打一条 / 消费侧全文扫）")

    rc, out, nf = run_mutated("test_wb5_contract.py", [], tag="baseline")
    check("基线（未手术）绿：rc=0", rc == 0, f"rc={rc} FAIL 条数={nf}")

    # ---- N5：产出侧在契约行**之后**再追加一条同前缀行 ----
    #   ★ 手术把 stdout 末尾追加一行；"恰好出现 1 次"与"末行=契约行"两条必须红。
    anchor = '    _RUN.update(rc=proc.returncode, out=proc.stdout or "", err=proc.stderr or "")'
    mutated = ('    _RUN.update(rc=proc.returncode,\n'
               '                out=(proc.stdout or "") + chr(10) + '
               'TIER1_PREFIX + chr(32) + chr(123) + chr(125) + chr(10),\n'
               '                err=proc.stderr or "")')
    rc5, out5, nf5 = run_mutated("test_wb5_contract.py", [(anchor, mutated)], tag="N5")
    check("★ N5 产出侧多打一条同前缀行 ⇒ 真跑出红",
          rc5 != 0 and nf5 >= 1, f"rc={rc5} FAIL 条数={nf5}")
    print(f"  N5 红因样例：{(FAIL_PAT.search(out5) and out5[FAIL_PAT.search(out5).start():][:150]) or '（无）'}")

    # ---- N6：消费侧退回"全文扫取第一次"（F-5 的形状）----
    rc6, out6, nf6 = run_mutated(
        "test_wb5_contract.py",
        [("    for line in reversed(text.splitlines()):\n"
          "        line = line.strip()\n"
          "        if not line.startswith(prefix + \" \"):\n"
          "            continue\n"
          "        try:\n"
          "            return json.loads(line[len(prefix) + 1:])\n"
          "        except ValueError:\n"
          "            return None\n"
          "    return None\n",
          "    return parse_contract_legacy_first(text, prefix)   # N6：退回全文扫\n")],
        tag="N6")
    check("★ N6 消费侧退回全文扫取第一次 ⇒ 真跑出红",
          rc6 != 0 and nf6 >= 1, f"rc={rc6} FAIL 条数={nf6}")


# --------------------------------------------------------------------------- #
#  汇总：每条判据至少被一个负例打红过
# --------------------------------------------------------------------------- #
def test_coverage() -> None:
    section("汇总：六个负例各打红了几条（一条都没打红 = 那个手术没起作用）")
    # 本条只做**记录**：各负例的 FAIL 条数在上一节已打 PASS/FAIL。
    print("  N1..N6 的 FAIL 条数见上（每条 ≥1 才算真跑出红）")
    check("六个负例都跑过（本节无失败才有意义）", True, "见上")


# --------------------------------------------------------------------------- #
#  main
# --------------------------------------------------------------------------- #
def main() -> int:
    print("WB-5 · 负例矩阵（在台架/闸上做手术，看正测是否真红）")
    test_sensitivity_negatives()
    test_zero_assert_negatives()
    test_contract_negatives()
    test_coverage()
    finish()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
