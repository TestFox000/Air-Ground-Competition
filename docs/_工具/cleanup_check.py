
# -*- coding: utf-8 -*-
import csv, os, hashlib, collections
ROOT = r"D:\User\Desktop\比赛存档\v1"
WD = os.path.join(ROOT,"docs","_work")
inv = list(csv.DictReader(open(os.path.join(WD,"inventory.csv"), encoding="utf-8-sig")))
led = list(csv.DictReader(open(os.path.join(WD,"cleanup_ledger.csv"), encoding="utf-8-sig")))
print("ledger rows =", len(led))
gone = [r for r in inv if not os.path.isfile(os.path.join(ROOT, r["rel_path"]))]
print("inventory files now missing =", len(gone))
for r in gone:
    if r["class"] not in ("A","B"):
        print("  !! 非A/B却已消失:", r["class"], r["rel_path"])
# 那个报错文件是否真的丢了内容
tgt = "dzsmartscout/dsh-plugin-dzdrone-ssh/novnc/vendor/pako/README.md"
row = [r for r in inv if r["rel_path"]==tgt]
if row:
    h = row[0]["sha256"]
    same = [r["rel_path"] for r in inv if r["sha256"]==h and os.path.isfile(os.path.join(ROOT,r["rel_path"]))]
    print("同哈希现存副本:", same)
le = [r for r in led if r["rel_path"]==tgt]
print("ledger 记录:", le)
