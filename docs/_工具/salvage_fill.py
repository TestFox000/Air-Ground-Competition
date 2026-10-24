
# -*- coding: utf-8 -*-
import os, hashlib, sys, json
sys.stdout.reconfigure(encoding="utf-8")
ROOT = r"D:\User\Desktop\比赛存档\v1"
V0 = r"D:\User\Desktop\比赛存档\v0"
SNAP = os.path.join(ROOT,"docs","_archive","snapshots")
salv = os.path.join(SNAP,"_salvage")
miss = ["_tmp_session/agent_overview_20261020/car.md","_tmp_session/agent_overview_20261020/drone.md",
        "_tmp_session/agent_overview_20261020/history.md","_tmp_session/agent_overview_20261020/link.md",
        "_tmp_session/aim_audit/体检报告_20261018_1741.md",
        "_tmp_session/audit_20261018/AUDIT_20261018.md",
        "_tmp_session/cleanup_20261018/REVIEW.md",
        "_tmp_session/d13_verify/切换记录_20261018.md"]
def sha(p):
    h=hashlib.sha256()
    with open(p,'rb') as f:
        for c in iter(lambda: f.read(1<<20), b''): h.update(c)
    return h.hexdigest()
found, notfound = [], []
for rel in miss:
    cand = os.path.join(V0, rel)
    if os.path.isfile(cand):
        os.makedirs(os.path.dirname(os.path.join(salv, rel)), exist_ok=True)
        open(os.path.join(salv, rel),"wb").write(open(cand,"rb").read())
        found.append((rel,"v0"))
    else:
        notfound.append(rel)
print("从 v0 补回:", len(found))
for f in found: print("  ", f[0])
print("仍缺:", len(notfound))
for n in notfound: print("  ", n)
