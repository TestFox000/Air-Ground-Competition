
# -*- coding: utf-8 -*-
import os, tarfile, sys, shutil, json
sys.stdout.reconfigure(encoding="utf-8")
SNAP = r"D:\User\Desktop\比赛存档\v1\docs\_archive\snapshots"
p = os.path.join(SNAP, "process_drafts_20261022.tar.gz")
out = os.path.join(SNAP, "_salvage")
if os.path.isdir(out): shutil.rmtree(out)
os.makedirs(out)
ok = 0; bad = []
with tarfile.open(p, "r|gz") as tf:
    while True:
        try:
            m = tf.next()
        except Exception as e:
            print("list stop:", type(e).__name__)
            break
        if m is None: break
        try:
            tf.extract(m, out, filter="data")
            ok += 1
        except Exception as e:
            bad.append((m.name, type(e).__name__, str(e)[:60]))
print("extracted members:", ok, "failed:", len(bad))
for b in bad: print("  FAILED", b)
json.dump({"ok": ok, "failed": bad}, open(os.path.join(SNAP, "_salvage_report.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
