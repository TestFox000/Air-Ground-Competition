
# -*- coding: utf-8 -*-
import csv, os, collections
ROOT = r"D:\User\Desktop\比赛存档\v1"
WD = os.path.join(ROOT,"docs","_work")
inv = list(csv.DictReader(open(os.path.join(WD,"inventory.csv"), encoding="utf-8-sig")))
by_hash = collections.defaultdict(list)
for r in inv:
    by_hash[r["sha256"]].append(r)
miss = [r for r in inv if not os.path.isfile(os.path.join(ROOT, r["rel_path"]))]
print("missing:", len(miss))
rec, lost = [], []
for r in miss:
    sur = [x["rel_path"] for x in by_hash[r["sha256"]]
           if os.path.isfile(os.path.join(ROOT, x["rel_path"]))]
    # 库内同哈希
    libdir = os.path.join(ROOT, "docs")
    lib = None
    if sur:
        rec.append((r["rel_path"], sur[0]))
    else:
        lost.append((r["rel_path"], r["sha256"], r["bytes"]))
print("可复原(有同哈希现存副本):", len(rec))
print("无副本:", len(lost))
for x in lost: print("  LOST", x[0], x[1][:12], x[2])
json_out = os.path.join(WD, "lost_dzsmartscout.json")
import json
json.dump({"recover": rec, "lost": lost}, open(json_out,"w",encoding="utf-8"), ensure_ascii=False, indent=1)
print("written", json_out)
