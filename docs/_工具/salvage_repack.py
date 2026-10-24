
# -*- coding: utf-8 -*-
import os, tarfile, sys, shutil
sys.stdout.reconfigure(encoding="utf-8")
SNAP = r"D:\User\Desktop\比赛存档\v1\docs\_archive\snapshots"
salv = os.path.join(SNAP,"_salvage")
out = os.path.join(SNAP,"process_drafts_20261022.tar.gz")
tmpout = os.path.join(SNAP,"process_drafts_20261022.repack.tar.gz")
if os.path.exists(tmpout): os.remove(tmpout)
n = 0
with tarfile.open(tmpout, "w:gz") as tf:
    for dp, dn, fn in os.walk(salv):
        for f in fn:
            full = os.path.join(dp, f)
            arc = os.path.relpath(full, salv).replace("\\","/")
            tf.add(full, arcname=arc)
            n += 1
print("repacked files:", n)
# 校验
m = 0
with tarfile.open(tmpout, "r|gz") as tf:
    for x in tf: m += 1
print("verify members:", m)
if m >= n:
    if os.path.exists(out): os.remove(out)
    os.rename(tmpout, out)
    print("replaced corrupt archive")
else:
    print("KEEP tmp, count mismatch")
