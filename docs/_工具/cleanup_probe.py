
# -*- coding: utf-8 -*-
import csv, os, sys
sys.stdout.reconfigure(encoding="utf-8")
ROOT = r"D:\User\Desktop\比赛存档\v1"
WD = os.path.join(ROOT,"docs","_work")
led = list(csv.DictReader(open(os.path.join(WD,"cleanup_ledger.csv"), encoding="utf-8-sig")))
inv = {r["rel_path"]: r for r in csv.DictReader(open(os.path.join(WD,"inventory.csv"), encoding="utf-8-sig"))}
print("ledger:", len(led))
tgt = "_tmp_session/TRACK-BASE-20261021/snapshot/docs/StepLoop_实车第二轮_20261018.md"
for r in led:
    if r["rel_path"] == tgt:
        print("ledger row:", r)
        print("kept exists:", os.path.isfile(os.path.join(ROOT, r["kept_path"])), r["kept_path"])
        break
else:
    print("NOT in ledger:", tgt)
# 检查 docs/_archive 里 dzrobot_v2 目录
for p in ["docs/_archive/dzrobot_v2", "docs/_archive/dzrobot_v2/docs", "docs/_archive/dzsmartscout"]:
    ap = os.path.join(ROOT, p)
    if os.path.isdir(ap):
        n = sum(1 for dp,dn,fn in os.walk(ap) for f in fn if f.lower().endswith(".md"))
        print(p, "md=", n)
    else:
        print(p, "MISSING")
# 列出 docs/_archive 顶层与各子项 md 数
base = os.path.join(ROOT, "docs", "_archive")
for d in sorted(os.listdir(base)):
    dp = os.path.join(base, d)
    if os.path.isdir(dp):
        n = sum(1 for a,b,c in os.walk(dp) for f in c if f.lower().endswith(".md"))
        print("  archive/%-22s md=%d" % (d, n))
