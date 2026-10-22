
# -*- coding: utf-8 -*-
import csv, os, hashlib, sys, collections
sys.stdout.reconfigure(encoding="utf-8")
ROOT = r"D:\User\Desktop\比赛存档\v1"
WD = os.path.join(ROOT,"docs","_work")
inv = list(csv.DictReader(open(os.path.join(WD,"inventory.csv"), encoding="utf-8-sig")))
def sha(p):
    h=hashlib.sha256()
    with open(p,'rb') as f:
        for c in iter(lambda: f.read(1<<20), b''): h.update(c)
    return h.hexdigest()
row = [r for r in inv if r["rel_path"].endswith("_step_placeholder")][:0]
a_rows = [r for r in inv if r["class"]=="A"]
print("A rows:", len(a_rows), "| sample:", a_rows[0]["rel_path"], "->", a_rows[0]["kept_path"])
ok = miss = 0
bad = []
for r in a_rows:
    p = os.path.join(ROOT, r["kept_path"])
    if os.path.isfile(p) and sha(p) == r["sha256"]: ok += 1
    else: bad.append((r["rel_path"], r["kept_path"]))
print("A 副本仍在且同哈希:", ok, " 不一致:", len(bad))
for b in bad[:10]: print("  ", b)
# B rows
b_rows = [r for r in inv if r["class"]=="B"]
ok2 = 0; bad2 = []
for r in b_rows:
    p = os.path.join(ROOT, r["rel_path"])
    if os.path.isfile(p): continue  # 保留份自己还在，说明没删
    kp = os.path.join(ROOT, r["kept_path"])
    if os.path.isfile(kp) and sha(kp) == r["sha256"]: ok2 += 1
    else: bad2.append((r["rel_path"], r["kept_path"]))
print("B 已删且保留份同哈希:", ok2, " 不一致:", len(bad2))
for b in bad2[:10]: print("  ", b)
