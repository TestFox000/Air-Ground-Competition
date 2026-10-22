
# -*- coding: utf-8 -*-
import os, tarfile, sys
sys.stdout.reconfigure(encoding="utf-8")
SNAP = r"D:\User\Desktop\比赛存档\v1\docs\_archive\snapshots"
for fn in sorted(os.listdir(SNAP)):
    if not fn.endswith(".tar.gz"): continue
    p = os.path.join(SNAP, fn)
    try:
        n = 0
        with tarfile.open(p, "r|gz") as tf:
            for m in tf:
                n += 1
        print("OK  %-45s members=%d" % (fn, n))
    except Exception as e:
        print("ERR %-45s %s: %s" % (fn, type(e).__name__, e))
