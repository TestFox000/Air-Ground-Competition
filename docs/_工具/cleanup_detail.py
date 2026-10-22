
# -*- coding: utf-8 -*-
import csv, os, collections, subprocess
ROOT = r"D:\User\Desktop\比赛存档\v1"
rows = list(csv.DictReader(open(os.path.join(ROOT,"docs","_work","inventory.csv"), encoding="utf-8-sig")))
def depth2(p):
    parts = p.split("/")
    return "/".join(parts[:2]) if len(parts) > 2 else p
for cls in ("A","B","C"):
    sub = [r for r in rows if r["class"]==cls]
    print("=== %s (%d) ===" % (cls, len(sub)))
    for k,v in collections.Counter(depth2(r["rel_path"]) for r in sub).most_common(12):
        print("  %-46s %d" % (k, v))
# git tracked check for A + B-delete
def tracked(repo, rel):
    r = subprocess.run(["git","-C",os.path.join(ROOT,repo),"ls-files","--error-unmatch",rel],
                       capture_output=True, text=True)
    return r.returncode == 0
cand = [r for r in rows if r["class"]=="A" or r["class"]=="B"]
print("\ncandidates=%d" % len(cand))
tr = []
for r in cand:
    p = r["rel_path"]
    if p.startswith("dzrobot_v2/"):
        if tracked("dzrobot_v2", p[len("dzrobot_v2/"):]):
            tr.append(p)
print("tracked_by_git=%d" % len(tr))
for p in tr[:20]: print("  TRACKED", p)
# symlink check
syms = [r["rel_path"] for r in cand if os.path.islink(os.path.join(ROOT, r["rel_path"]))]
print("symlinks=%d" % len(syms))
