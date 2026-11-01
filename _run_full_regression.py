# -*- coding: utf-8 -*-
"""修红后全量回归：全部 test_*.py 套件逐一跑，落盘日志 + 汇总。
用法：python _run_full_regression.py
"""
from __future__ import annotations
import csv, os, pathlib, re, subprocess, sys

WS = pathlib.Path(__file__).resolve().parent
TESTS = WS / "src" / "dzbrain" / "tests"
OUT = WS / "_tmp_full_regression"
OUT.mkdir(parents=True, exist_ok=True)

env = dict(os.environ)
env["PYTHONPATH"] = str(WS / "src" / "dzbrain")
env["PYTHONIOENCODING"] = "utf-8"
env["PYTHONUTF8"] = "1"

files = sorted(p.name for p in TESTS.glob("test_*.py"))
rows = []
for name in files:
    log = OUT / f"{name}.log"
    with open(log, "w", encoding="utf-8", errors="replace") as fh:
        proc = subprocess.run([sys.executable, str(TESTS / name)],
                              cwd=str(WS), env=env, stdout=fh, stderr=subprocess.STDOUT,
                              timeout=1200)
    text = log.read_text(encoding="utf-8", errors="replace")
    # ★ WB-5：正则必须容得下**本仓实际存在的两种收尾语法**（这不是"放松"，是补漏）：
    #     A 型 `结果：30/30 全部通过 ✅`            —— `harness.finish()` / `test_route.py` 系
    #     B 型 `结果：30/30 通过 全部通过`          —— `test_wb2_*` 与本卡 `test_wb5_*` 的 finish()
    #     原正则 `结果：(\d+)/(\d+)\s*(全部通过|通过[，,]\s*(\d+)\s*失败)` 只认 A 型，
    #     B 型的 `通过` 与 `全部通过` 中间多一个空格 ⇒ **匹配失败 ⇒ 记 -1 ⇒ 静默消失**。
    #     实测：我自己的三个套件 + wb2 的两个，在全量回归表里全是 -1/-1（一条都没被采信）。
    #   所以改成「`结果：P/T` 后面**任意收尾**」：数值照旧来自真实计数，只是不再卡收尾文字。
    # ★★ 还必须取**末次**出现（`findall[-1]`），不是 `search` 的首次：
    #    台账里好几个套件的**模块 docstring 里就写着样例结果行**
    #    （本卡 `test_wb5_zero_assert.py` 的 docstring 原文贴着 `结果：0/0 全部通过 ✅`），
    #    `search` 会抓到 docstring 里那一行，把 27/27 的套件记成 3/5。
    #    末次出现与 `test_wb5_zero_assert.parse_result_line` 的口径一致。
    hits = list(re.finditer(r"结果：(\d+)/(\d+)([^\n]*)", text))
    if hits:
        h = hits[-1]
        passed, total = int(h.group(1)), int(h.group(2))
        tail = h.group(3)
        # 失败条数：优先取「N 失败」，其次取「p/t 之差」，两者不一致时取**较大**（从严）
        fm = re.search(r"(\d+)\s*失败", tail)
        fails = int(fm.group(1)) if fm else max(0, total - passed)
        fails = max(fails, total - passed)
    else:
        passed = total = fails = -1
    ids = re.findall(r"^\s*FAIL\s+([0-9]+\.[0-9a-zA-Z]+)", text, flags=re.M)
    rows.append({"file": name, "exit": proc.returncode, "pass": passed, "total": total,
                 "fails": fails, "fail_ids": " ".join(sorted(set(ids)))})
    print(f"{name:<30} exit={proc.returncode:<3} {passed}/{total}  失败{fails}  "
          f"红={','.join(sorted(set(ids)))}", flush=True)

with open(OUT / "suite_table.csv", "w", newline="", encoding="utf-8") as fh:
    w = csv.DictWriter(fh, fieldnames=["file", "exit", "pass", "total", "fails", "fail_ids"])
    w.writeheader()
    w.writerows(rows)


def why_red(row) -> str:
    """某行为什么算红。**空串 = 绿**。

    ★ WB-5（卡 `WB-5-REGNET-20261031`）加的三条：本函数**之前**只是
      `red = [r for r in rows if r["exit"] != 0]`，于是三类"静默消失"全都不进红区：
        ① 解析不到 `结果：` 行（`pass=total=-1`）而 exit=0 —— 套件坏得连结果行都没打，
           汇总器却当它绿（`test_aim_probe.py` 就是这么混了很多轮）；
        ② `total == 0`（0 断言）而 exit=0 —— 一条断言都没跑仍判绿（本卡封的正面形态）；
        ③ `total == -1` 但 exit 非零 —— 只是"红"，看不出是"根本没跑"还是"跑了失败"。
      所以判据改成"退出码 **或** 结果行缺失/为 0"——**宁可误红，不许静默绿**。
    """
    if row["exit"] != 0:
        return f"exit={row['exit']}"
    if row["total"] == -1:
        return "没有可解析的 `结果：N/M` 行（断言计数未知 ⇒ 不许当绿）"
    if row["total"] == 0:
        return "断言数=0（一条断言都没执行 ⇒ 假绿）"
    if row["pass"] != row["total"]:
        return f"结果行自称失败 {row['pass']}/{row['total']}"
    return ""


red = [r for r in rows if why_red(r)]
silent = [r for r in rows if r["exit"] == 0 and r["total"] <= 0]
print(f"\n套数={len(rows)}  红={len(red)}  （其中「退出码为 0 但无断言/无结果行」={len(silent)}）")
for r in red:
    print(f"  RED: {r['file']} exit={r['exit']} {r['pass']}/{r['total']} "
          f"因={why_red(r)} 红={r['fail_ids']}")

# ★ 有静默红就以非零退出：这个脚本自己也是一道闸，不能只打印不表态。
sys.exit(1 if red else 0)
