
# -*- coding: utf-8 -*-
import csv, os, tarfile, hashlib, sys
sys.stdout.reconfigure(encoding="utf-8")
ROOT = r"D:\User\Desktop\比赛存档\v1"
WD = os.path.join(ROOT,"docs","_work")
SNAP = os.path.join(ROOT,"docs","_archive","snapshots")
inv = list(csv.DictReader(open(os.path.join(WD,"inventory.csv"), encoding="utf-8-sig")))
def sha_file(p):
    h=hashlib.sha256()
    with open(p,'rb') as f:
        for c in iter(lambda: f.read(1<<20), b''): h.update(c)
    return h.hexdigest()

b_rows = [r for r in inv if r["class"]=="B" and not os.path.isfile(os.path.join(ROOT, r["rel_path"]))]
need = {}
for r in b_rows:
    kp = r["kept_path"]
    if os.path.isfile(os.path.join(ROOT, kp)):
        if sha_file(os.path.join(ROOT, kp)) == r["sha256"]: continue
    need.setdefault(r["sha256"], []).append(kp)
print("B 已删且需在 tar 内验证:", len(b_rows), "唯一哈希:", len(need))
found = {}
for fn in sorted(os.listdir(SNAP)):
    if not fn.endswith(".tar.gz"): continue
    tp = os.path.join(SNAP, fn)
    with tarfile.open(tp, "r:gz") as tf:
        for m in tf:
            if not m.isfile(): continue
            name = m.name.replace("\\","/")
            cands = [h for h, paths in need.items() if h not in found and any(p.replace("\\","/") == name or p.replace("\\","/").endswith("/"+name) or name.endswith("/"+p.replace("\\","/")) for p in paths)]
            if not cands: continue
            f = tf.extractfile(m)
            if f is None: continue
            hh = hashlib.sha256(f.read()).hexdigest()
            if hh in cands and hh not in found:
                found[hh] = (fn, name)
print("tar 内找到同哈希:", len(found), "/", len(need))
missing = [ (h, need[h][0]) for h in need if h not in found ]
for h, p in missing: print("  NOTFOUND", p)
json.dump({k: v for k, v in found.items()}, open(os.path.join(WD,"b_tar_verify.json"),"w",encoding="utf-8"), ensure_ascii=False, indent=1)
