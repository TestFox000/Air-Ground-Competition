
# -*- coding: utf-8 -*-
import json, os, hashlib, sys
sys.stdout.reconfigure(encoding="utf-8")
ROOT = r"D:\User\Desktop\比赛存档\v1"
V0 = r"D:\User\Desktop\比赛存档\v0"
WD = os.path.join(ROOT,"docs","_work")
gap = json.load(open(os.path.join(WD,"ds_gap.json"), encoding="utf-8"))
lost = gap["lost"]
inv = {r["rel_path"]: r for r in __import__("csv").DictReader(open(os.path.join(WD,"inventory.csv"), encoding="utf-8-sig"))}
def sha(p):
    h=hashlib.sha256()
    with open(p,'rb') as f:
        for c in iter(lambda: f.read(1<<20), b''): h.update(c)
    return h.hexdigest()
hit, miss = [], []
for rel in lost:
    p = os.path.join(V0, rel)
    if os.path.isfile(p):
        ok = sha(p) == inv[rel]["sha256"]
        hit.append((rel, ok))
    else:
        miss.append(rel)
print("v0 中存在: %d（哈希一致 %d）" % (len(hit), sum(1 for _,o in hit if o)))
print("v0 中也缺: %d" % len(miss))
for rel, ok in hit:
    if not ok: print("  HASH-DIFF", rel)
for m in miss: print("  MISS", m)
json.dump({"hit":[h[0] for h in hit if h[1]], "diff":[h[0] for h in hit if not h[1]], "miss":miss},
          open(os.path.join(WD,"ds_v0.json"),"w",encoding="utf-8"), ensure_ascii=False, indent=1)
