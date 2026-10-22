
# -*- coding: utf-8 -*-
import os, sys, csv
sys.stdout.reconfigure(encoding="utf-8")
ROOT = r"D:\User\Desktop\比赛存档\v1"
LIB = os.path.join(ROOT,"docs")
def count(base, skip_lib=False):
    n=0
    for dp,dn,fn in os.walk(base):
        dn[:] = [d for d in dn if d not in (".git","node_modules","__pycache__")]
        if skip_lib and os.path.abspath(dp).startswith(LIB + os.sep): continue
        n += sum(1 for f in fn if f.lower().endswith(".md"))
    return n
lib = count(LIB)
out = count(ROOT, skip_lib=True)
led = list(csv.DictReader(open(os.path.join(LIB,"_work","cleanup_ledger.csv"), encoding="utf-8-sig")))
print("库内 md:", lib)
print("库外 md:", out)
print("ledger rows:", len(led), " by phase:", {p: sum(1 for r in led if r["phase"]==p) for p in set(r["phase"] for r in led)})
snap = os.path.join(LIB,"_archive","snapshots")
tot = sum(os.path.getsize(os.path.join(snap,f)) for f in os.listdir(snap) if f.endswith(".tar.gz"))
print("snapshots: %d tar, %.1f MB" % (sum(1 for f in os.listdir(snap) if f.endswith(".tar.gz")), tot/1048576))
