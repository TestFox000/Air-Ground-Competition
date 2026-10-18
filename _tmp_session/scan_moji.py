# -*- coding: utf-8 -*-
import os, sys
root = r"D:\User\Desktop\比赛存档\v1"
markers = "锛銆鐨涓鏂鏈鍒鐩璺闂鎬鏄濡缁鏄庣‘"
dirs = [os.path.join(root,"docs"), os.path.join(root,"dzrobot_v2","docs"), os.path.join(root,"dzsmartscout","docs")]
paths = []
for base in dirs:
    for dp,dn,fn in os.walk(base):
        if "_tmp_session" in dp: continue
        for f in fn:
            if f.endswith(".md"): paths.append(os.path.join(dp,f))
paths = sorted(set(paths))
damaged = []
for p in paths:
    try:
        t = open(p, encoding="utf-8").read()
    except Exception as e:
        print("READ_ERR", p, str(e)[:60]); continue
    lines = t.splitlines()
    bad = [i for i,ln in enumerate(lines) if any(c in markers for c in ln)]
    if bad:
        damaged.append((p, len(bad), len(lines)))
print("scanned", len(paths), "files, damaged:", len(damaged))
for p,b,n in damaged:
    print(f"{b:5d}/{n:5d}  {p}")
print("--- non-md files under docs/merge:")
for f in sorted(os.listdir(os.path.join(root,"docs","merge"))):
    print(" ", f)
