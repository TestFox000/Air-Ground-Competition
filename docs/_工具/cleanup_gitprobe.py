
# -*- coding: utf-8 -*-
import csv, os, subprocess, hashlib, sys, json
sys.stdout.reconfigure(encoding="utf-8")
ROOT = r"D:\User\Desktop\比赛存档\v1"
G = r"D:\User\Desktop\比赛存档\public_v1.git"
WD = os.path.join(ROOT,"docs","_work")
inv = {r["rel_path"]: r for r in csv.DictReader(open(os.path.join(WD,"inventory.csv"), encoding="utf-8-sig"))}
gap = json.load(open(os.path.join(WD,"ds_gap.json"), encoding="utf-8"))
targets = gap["lost"] + json.load(open(os.path.join(WD,"ds_v0.json"), encoding="utf-8"))["diff"]
exact, diff, absent, nofile = [], [], [], []
for rel in targets:
    r = subprocess.run(["git","--git-dir="+G,"show","HEAD:"+rel], capture_output=True)
    if r.returncode != 0:
        absent.append(rel); continue
    h = hashlib.sha256(r.stdout).hexdigest()
    if h == inv[rel]["sha256"]: exact.append(rel)
    else: diff.append((rel, len(r.stdout), inv[rel]["bytes"]))
print("目标 %d：git HEAD 精确同哈希 %d，版本不同 %d，git 里没有 %d" % (len(targets), len(exact), len(diff), len(absent)))
for x in diff: print("  DIFF", x[0], x[1], "->", x[2])
for x in absent: print("  ABSENT", x)
json.dump({"exact":exact,"diff":[d[0] for d in diff],"absent":absent},
          open(os.path.join(WD,"ds_git.json"),"w",encoding="utf-8"), ensure_ascii=False, indent=1)
