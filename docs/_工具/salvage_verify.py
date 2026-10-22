
# -*- coding: utf-8 -*-
import os, csv, sys, tarfile
sys.stdout.reconfigure(encoding="utf-8")
ROOT = r"D:\User\Desktop\比赛存档\v1"
WD = os.path.join(ROOT,"docs","_work")
SNAP = os.path.join(ROOT,"docs","_archive","snapshots")
salv = os.path.join(SNAP,"_salvage")
inv = list(csv.DictReader(open(os.path.join(WD,"inventory.csv"), encoding="utf-8-sig")))
led = {r["rel_path"] for r in csv.DictReader(open(os.path.join(WD,"cleanup_ledger.csv"), encoding="utf-8-sig"))}
# 期望：_tmp_session 下未单独打包的 md（排除 4 棵树 + R3 目录）
sep = ("_tmp_session/gitee_air_ground/", "_tmp_session/yb_tree/", "_tmp_session/gitee_main_tree/",
       "_tmp_session/carsnap_tree/", "_tmp_session/TRACK-FIELD-R3-20261020-01/")
exp = [r["rel_path"] for r in inv if r["rel_path"].startswith("_tmp_session/")
       and not any(r["rel_path"].startswith(s) for s in sep)]
got = []
for dp,dn,fn in os.walk(salv):
    for f in fn:
        if f.lower().endswith(".md"):
            got.append(os.path.relpath(os.path.join(dp,f), salv).replace("\\","/"))
miss = [e for e in exp if e not in got]
extra = [g for g in got if g not in exp]
print("期望 md:", len(exp), " 解出 md:", len(got), " 缺:", len(miss), " 多:", len(extra))
for m in miss[:15]: print("  MISS", m)
for e in extra[:5]: print("  EXTRA", e)
