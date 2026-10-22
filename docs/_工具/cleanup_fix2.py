
# -*- coding: utf-8 -*-
import os, subprocess, hashlib, json, sys, csv
sys.stdout.reconfigure(encoding="utf-8")
ROOT = r"D:\User\Desktop\比赛存档\v1"
WD = os.path.join(ROOT,"docs","_work")

def sha(p):
    h=hashlib.sha256()
    with open(p,'rb') as f:
        for c in iter(lambda: f.read(1<<20), b''): h.update(c)
    return h.hexdigest()

# 1) 从兄弟插件目录补回 pako README（同哈希）
src = os.path.join(ROOT, "dsh-dzairground-ssh/novnc/vendor/pako/README.md")
dst = os.path.join(ROOT, "dzsmartscout/dsh-plugin-dzdrone-ssh/novnc/vendor/pako/README.md")
if os.path.isfile(src) and not os.path.isfile(dst):
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    open(dst,"wb").write(open(src,"rb").read())
    print("restored pako README from sibling")

# 2) link_status.md：只有旧版，落盘并标注
rel = "dzsmartscout/tools/planner/EVIDENCE/link_status.md"
p = os.path.join(ROOT, rel)
if not os.path.isfile(p):
    r = subprocess.run(["git","show","HEAD:"+rel], capture_output=True, cwd=ROOT)
    if r.returncode == 0:
        os.makedirs(os.path.dirname(p), exist_ok=True)
        open(p,"wb").write(r.stdout)
        print("restored link_status.md from git HEAD (older revision, %d bytes)" % len(r.stdout))

# 3) 全量对账：inventory 中所有文件是否存在且同哈希
inv = list(csv.DictReader(open(os.path.join(WD,"inventory.csv"), encoding="utf-8-sig")))
ledger = {r["rel_path"]: r for r in csv.DictReader(open(os.path.join(WD,"cleanup_ledger.csv"), encoding="utf-8-sig"))}
missing, mismatched = [], []
for r in inv:
    if r["rel_path"] not in ledger:   # 应该都还在
        fp = os.path.join(ROOT, r["rel_path"])
        if not os.path.isfile(fp): missing.append(r["rel_path"])
        elif sha(fp) != r["sha256"]: mismatched.append(r["rel_path"])
print("清单内应存在但缺失:", len(missing))
for m in missing[:20]: print("   MISS", m)
print("清单内存在但哈希变化:", len(mismatched))
for m in mismatched[:10]: print("   DIFF", m)
