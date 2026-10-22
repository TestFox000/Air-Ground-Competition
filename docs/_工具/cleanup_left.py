
# -*- coding: utf-8 -*-
import os, sys, collections
sys.stdout.reconfigure(encoding="utf-8")
ROOT = r"D:\User\Desktop\比赛存档\v1"
LIB = os.path.join(ROOT,"docs")
out=[]
for dp,dn,fn in os.walk(ROOT):
    dn[:] = [d for d in dn if d not in (".git","node_modules","__pycache__")]
    if os.path.abspath(dp) == LIB or os.path.abspath(dp).startswith(LIB + os.sep): continue
    for f in fn:
        if f.lower().endswith(".md"):
            out.append(os.path.relpath(os.path.join(dp,f), ROOT).replace("\\","/"))
print("库外现存 md:", len(out))
g = collections.Counter("/".join(p.split("/")[:2]) if p.count("/")>=1 else p for p in out)
for k,v in sorted(g.items(), key=lambda x:-x[1]):
    print("  %-40s %d" % (k, v))
