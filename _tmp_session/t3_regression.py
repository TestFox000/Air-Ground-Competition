# -*- coding: utf-8 -*-
"""t3 独立全量回归（只读）：逐个跑 src/dzbrain/tests/test_*.py，记录 rc 与末行结果。"""
import glob, os, re, subprocess, sys, io, json

ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2"
TESTS = sorted(glob.glob(os.path.join(ROOT, "src", "dzbrain", "tests", "test_*.py")))
env = dict(os.environ, PYTHONIOENCODING="utf-8")
rows = []
for p in TESTS:
    name = os.path.basename(p)
    r = subprocess.run([sys.executable, p], cwd=ROOT, capture_output=True, env=env)
    out = (r.stdout.decode("utf-8", "replace") + r.stderr.decode("utf-8", "replace"))
    lines = [l for l in out.splitlines() if l.strip()]
    tail = lines[-1] if lines else ""
    ratio = ""
    m = re.search(r"(\d+)\s*/\s*(\d+)", out)
    if m:
        ratio = "%s/%s" % (m.group(1), m.group(2))
    fails = re.findall(r"^\s*(?:✗|FAIL)\s+(.*)$", out, re.M)
    rows.append(dict(name=name, rc=r.returncode, ratio=ratio, tail=tail[:110],
                     nfail=len(fails), tier1=fails[:6]))
    print("%-28s rc=%d %-12s %s" % (name, r.returncode, ratio, tail[:90]))
    for f in fails[:6]:
        print("        FAIL:", f[:120])

print()
red = [r for r in rows if r["rc"] != 0]
print("套件总数 %d  绿 %d  红 %d" % (len(rows), len(rows) - len(red), len(red)))
for r in red:
    print("  红:", r["name"], "rc=%d" % r["rc"], r["tail"][:80])
io.open(r"D:\User\Desktop\比赛存档\v1\_tmp_session\t3_regression.json", "w",
        encoding="utf-8").write(json.dumps(rows, ensure_ascii=False, indent=1))
