
# -*- coding: utf-8 -*-
import os, tarfile, sys, shutil
sys.stdout.reconfigure(encoding="utf-8")
SNAP = r"D:\User\Desktop\比赛存档\v1\docs\_archive\snapshots"
for fn in sorted(os.listdir(SNAP)):
    if not fn.endswith(".tar.gz"): continue
    p = os.path.join(SNAP, fn)
    try:
        n = 0; files = 0
        with tarfile.open(p, "r|gz") as tf:
            for m in tf:
                n += 1
                if m.isfile(): files += 1
        print("OK  %-45s members=%-6d files=%-6d %6.1fMB" % (fn, n, files, os.path.getsize(p)/1048576))
    except Exception as e:
        print("ERR %-45s %s" % (fn, e))
salv = os.path.join(SNAP,"_salvage")
if os.path.isdir(salv):
    shutil.rmtree(salv); print("salvage dir removed")
