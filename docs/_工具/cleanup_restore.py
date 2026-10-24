
# -*- coding: utf-8 -*-
import csv, os, subprocess, hashlib, sys, json
sys.stdout.reconfigure(encoding="utf-8")
ROOT = r"D:\User\Desktop\比赛存档\v1"
G = r"D:\User\Desktop\比赛存档\public_v1.git"
V0 = r"D:\User\Desktop\比赛存档\v0"
WD = os.path.join(ROOT,"docs","_work")
inv = {r["rel_path"]: r for r in csv.DictReader(open(os.path.join(WD,"inventory.csv"), encoding="utf-8-sig"))}
gap = json.load(open(os.path.join(WD,"ds_gap.json"), encoding="utf-8"))
v0j = json.load(open(os.path.join(WD,"ds_v0.json"), encoding="utf-8"))
targets = gap["lost"] + v0j["diff"]

def sha_bytes(b): return hashlib.sha256(b).hexdigest()

restored, diffs, failed = [], [], []
for rel in targets:
    want = inv[rel]["sha256"]
    dst = os.path.join(ROOT, rel)
    src = None
    # 1) git HEAD
    r = subprocess.run(["git","--git-dir="+G,"show","HEAD:"+rel], capture_output=True)
    if r.returncode == 0 and sha_bytes(r.stdout) == want:
        src = ("git", r.stdout)
    else:
        # 2) v0
        p = os.path.join(V0, rel)
        if os.path.isfile(p):
            b = open(p,"rb").read()
            if sha_bytes(b) == want: src = ("v0", b)
    if src:
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        open(dst,"wb").write(src[1])
        restored.append((rel, src[0]))
    else:
        # 3) 退而求其次：git 有但版本不同 / v0 有但版本不同 → 记录不落盘
        gi = subprocess.run(["git","--git-dir="+G,"show","HEAD:"+rel], capture_output=True)
        pv = os.path.join(V0, rel)
        alt = None
        if gi.returncode == 0: alt = ("git-旧版", len(gi.stdout))
        elif os.path.isfile(pv): alt = ("v0-旧版", os.path.getsize(pv))
        (diffs if alt else failed).append((rel, alt))

print("已精确恢复: %d" % len(restored))
for r in restored: print("  OK", r[1], r[0])
print("只有旧版本(未落盘): %d" % len(diffs))
for d in diffs: print("  OLD", d[0], d[1])
print("完全无源: %d" % len(failed))
for f in failed: print("  NONE", f[0])
json.dump({"restored":restored,"old":diffs,"none":failed},
          open(os.path.join(WD,"ds_restore.json"),"w",encoding="utf-8"), ensure_ascii=False, indent=1)
