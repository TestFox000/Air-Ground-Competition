
# -*- coding: utf-8 -*-
import os, csv, sys
sys.stdout.reconfigure(encoding="utf-8")
ROOT = r"D:\User\Desktop\比赛存档\v1"
WD = os.path.join(ROOT,"docs","_work")
SNAP = os.path.join(ROOT,"docs","_archive","snapshots")
salv = os.path.join(SNAP,"_salvage")
inv = list(csv.DictReader(open(os.path.join(WD,"inventory.csv"), encoding="utf-8-sig")))
led = {r["rel_path"] for r in csv.DictReader(open(os.path.join(WD,"cleanup_ledger.csv"), encoding="utf-8-sig"))}
sep = ("_tmp_session/gitee_air_ground/", "_tmp_session/yb_tree/", "_tmp_session/gitee_main_tree/",
       "_tmp_session/carsnap_tree/", "_tmp_session/TRACK-FIELD-R3-20261020-01/")
exp = [r["rel_path"] for r in inv
       if r["rel_path"].startswith("_tmp_session/") and r["rel_path"] not in led
       and not any(r["rel_path"].startswith(s) for s in sep)]
got = set()
for dp,dn,fn in os.walk(salv):
    for f in fn:
        if f.lower().endswith(".md"):
            got.add(os.path.relpath(os.path.join(dp,f), salv).replace("\\","/"))
miss = [e for e in exp if e not in got]
print("期望 md（已扣除已删）:", len(exp), "解出:", len(got), "缺:", len(miss))
for m in miss: print("  MISS", m)
