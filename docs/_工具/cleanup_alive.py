
# -*- coding: utf-8 -*-
import os, csv, sys, collections
sys.stdout.reconfigure(encoding="utf-8")
ROOT = r"D:\User\Desktop\比赛存档\v1"
WD = os.path.join(ROOT,"docs","_work")
inv = list(csv.DictReader(open(os.path.join(WD,"inventory.csv"), encoding="utf-8-sig")))
led = {r["rel_path"] for r in csv.DictReader(open(os.path.join(WD,"cleanup_ledger.csv"), encoding="utf-8-sig"))}
alive = [r for r in inv if r["rel_path"] not in led]
print("库外现存 md:", len(alive))
groups = collections.Counter()
for r in alive:
    parts = r["rel_path"].split("/")
    key = "/".join(parts[:2]) if len(parts) > 2 else "/".join(parts[:1])
    groups[key] += 1
for k, v in groups.most_common(60):
    print("  %-52s %d" % (k, v))
