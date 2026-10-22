
# -*- coding: utf-8 -*-
import csv, os, hashlib, sys, collections, json
sys.stdout.reconfigure(encoding="utf-8")
ROOT = r"D:\User\Desktop\比赛存档\v1"
WD = os.path.join(ROOT,"docs","_work")
inv = list(csv.DictReader(open(os.path.join(WD,"inventory.csv"), encoding="utf-8-sig")))
def sha(p):
    h=hashlib.sha256()
    with open(p,'rb') as f:
        for c in iter(lambda: f.read(1<<20), b''): h.update(c)
    return h.hexdigest()
ds = [r for r in inv if r["rel_path"].startswith("dzsmartscout/")]
still = [r for r in ds if not os.path.isfile(os.path.join(ROOT, r["rel_path"]))]
print("dzsmartscout: 原 %d, 仍缺 %d, 已恢复 %d" % (len(ds), len(still), len(ds)-len(still)))
# 全仓索引同哈希（排除 .git）
idx = collections.defaultdict(list)
for dp,dn,fn in os.walk(ROOT):
    dn[:] = [d for d in dn if d != ".git"]
    for f in fn:
        p=os.path.join(dp,f)
        try:
            if os.path.getsize(p) > 4*1024*1024: continue
            idx[sha(p)].append(os.path.relpath(p,ROOT).replace("\\","/"))
        except OSError: pass
rec, lost = [], []
for r in still:
    c = [x for x in idx.get(r["sha256"], []) if x != r["rel_path"]]
    if c: rec.append((r["rel_path"], c[0]))
    else: lost.append(r["rel_path"])
print("可从他处复原: %d" % len(rec))
for x in rec[:10]: print("  REC", x[0], "<=", x[1])
print("仍无副本: %d" % len(lost))
for x in lost: print("  LOST", x)
json.dump({"rec":rec,"lost":lost}, open(os.path.join(WD,"ds_gap.json"),"w",encoding="utf-8"), ensure_ascii=False, indent=1)
