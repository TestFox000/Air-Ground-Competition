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
    m = re.search(r"结果：(\d+)/(\d+)\s*(全部通过|通过[，,]\s*(\d+)\s*失败)", text)
    if m:
        passed, total = int(m.group(1)), int(m.group(2))
        fails = int(m.group(4)) if m.group(4) else 0
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

red = [r for r in rows if r["exit"] != 0]
print(f"\n套数={len(rows)}  非零退出={len(red)}")
for r in red:
    print(f"  RED: {r['file']} exit={r['exit']} {r['pass']}/{r['total']} 红={r['fail_ids']}")
