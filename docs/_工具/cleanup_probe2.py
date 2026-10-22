
# -*- coding: utf-8 -*-
import os, hashlib, sys, json
sys.stdout.reconfigure(encoding="utf-8")
ROOT = r"D:\User\Desktop\比赛存档\v1"
target = "6673b4015d23bdc63982d92fa675a40ae9f38609523ff74916fe05c762722382"
def sha(p):
    h=hashlib.sha256()
    with open(p,'rb') as f:
        for c in iter(lambda: f.read(1<<20), b''): h.update(c)
    return h.hexdigest()
hits=[]
for dp,dn,fn in os.walk(os.path.join(ROOT,"docs")):
    for f in fn:
        p=os.path.join(dp,f)
        try:
            if sha(p)==target: hits.append(os.path.relpath(p,ROOT))
        except OSError: pass
print("docs hits:", hits)
# 名字搜
for dp,dn,fn in os.walk(ROOT):
    if ".git" in dp: continue
    for f in fn:
        if "StepLoop" in f:
            p=os.path.join(dp,f)
            print("NAME", os.path.relpath(p,ROOT), os.path.getsize(p))
