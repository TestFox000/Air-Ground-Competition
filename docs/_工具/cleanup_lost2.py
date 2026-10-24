
# -*- coding: utf-8 -*-
import csv, os, hashlib, collections, json, sys
sys.stdout.reconfigure(encoding="utf-8")
ROOT = r"D:\User\Desktop\比赛存档\v1"
WD = os.path.join(ROOT,"docs","_work")
inv = list(csv.DictReader(open(os.path.join(WD,"inventory.csv"), encoding="utf-8-sig")))

def sha(p):
    h = hashlib.sha256()
    with open(p,"rb") as f:
        for c in iter(lambda: f.read(1<<20), b""): h.update(c)
    return h.hexdigest()

# 库内哈希
lib = collections.defaultdict(list)
for dp, dns, fns in os.walk(os.path.join(ROOT,"docs")):
    for fn in fns:
        p = os.path.join(dp, fn)
        try: lib[sha(p)].append(os.path.relpath(p, ROOT).replace("\\","/"))
        except OSError: pass

# 库外现存哈希
out = collections.defaultdict(list)
for r in inv:
    p = os.path.join(ROOT, r["rel_path"])
    if os.path.isfile(p): out[r["sha256"]].append(r["rel_path"])

miss = [r for r in inv if not os.path.isfile(os.path.join(ROOT, r["rel_path"]))]
ds = [r for r in miss if r["rel_path"].startswith("dzsmartscout/")]
mine = [r for r in miss if not r["rel_path"].startswith("dzsmartscout/")]
print("missing total=%d  (dzsmartscout=%d, 我删的A/B=%d)" % (len(miss), len(ds), len(mine)))

# 我删的 A/B 是否都能在库里/存活副本找到
bad = []
for r in mine:
    if r["sha256"] in lib: continue
    if r["sha256"] in out and out[r["sha256"]]: continue
    bad.append(r["rel_path"])
print("我删的A/B中找不到副本的:", len(bad))
for b in bad[:10]: print("   ", b)

# dzsmartscout 153 的可复原性
rec_lib, rec_out, gone = [], [], []
for r in ds:
    if r["sha256"] in lib: rec_lib.append((r["rel_path"], lib[r["sha256"]][0]))
    elif out.get(r["sha256"]): rec_out.append((r["rel_path"], out[r["sha256"]][0]))
    else: gone.append((r["rel_path"], r["sha256"]))
print("dzsmartscout: 库里同哈希=%d  库外同哈希=%d  两者都无=%d" % (len(rec_lib), len(rec_out), len(gone)))
json.dump({"rec_lib":rec_lib,"rec_out":rec_out,"gone":gone}, open(os.path.join(WD,"ds_recover.json"),"w",encoding="utf-8"), ensure_ascii=False, indent=1)
for g in gone[:25]: print("   GONE", g[0])
print("... gone 共", len(gone))
