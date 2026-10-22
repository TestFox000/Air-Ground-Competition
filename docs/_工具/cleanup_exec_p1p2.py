
# -*- coding: utf-8 -*-
"""P1+P2：删 A 类（库内已有同哈希）+ B 类（库外重复的非首份）。
安全闸：1) git 跟踪文件跳过 2) 删除前再校验哈希 3) 先落 ledger 再删"""
import csv, os, hashlib, subprocess, collections

ROOT = r"D:\User\Desktop\比赛存档\v1"
WD = os.path.join(ROOT, "docs", "_work")
inv = list(csv.DictReader(open(os.path.join(WD, "inventory.csv"), encoding="utf-8-sig")))
LEDGER = os.path.join(WD, "cleanup_ledger.csv")

def sha(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for c in iter(lambda: f.read(1 << 20), b""):
            h.update(c)
    return h.hexdigest()

def git_tracked(rel):
    if not rel.startswith("dzrobot_v2/"): return False
    r = subprocess.run(["git", "-C", os.path.join(ROOT, "dzrobot_v2"),
                        "ls-files", "--error-unmatch", rel[len("dzrobot_v2/"):]],
                       capture_output=True, text=True, encoding="utf-8", errors="replace")
    return r.returncode == 0

skip, todo = [], []
for r in inv:
    if r["class"] not in ("A", "B"): continue
    if git_tracked(r["rel_path"]):
        skip.append((r["rel_path"], "git 跟踪，不删")); continue
    todo.append(r)

bad = []
ok = []
for r in todo:
    p = os.path.join(ROOT, r["rel_path"])
    if not os.path.isfile(p):
        bad.append((r["rel_path"], "文件不存在")); continue
    if r["class"] == "A":
        # 库内同哈希文件必须存在（kept_path 已由 inventory 记为库内相对路径）
        kp = os.path.join(ROOT, r["kept_path"])
        if not os.path.isfile(kp):
            bad.append((r["rel_path"], "库内副本丢失: " + r["kept_path"])); continue
        if sha(kp) != r["sha256"]:
            bad.append((r["rel_path"], "库内副本哈希不符: " + r["kept_path"])); continue
    else:
        kp = os.path.join(ROOT, r["kept_path"])
        if not os.path.isfile(kp):
            bad.append((r["rel_path"], "保留份丢失: " + r["kept_path"])); continue
        if sha(kp) != r["sha256"]:
            bad.append((r["rel_path"], "保留份哈希不符: " + r["kept_path"])); continue
    ok.append(r)

new = not os.path.exists(LEDGER)
with open(LEDGER, "a", newline="", encoding="utf-8-sig") as f:
    w = csv.writer(f)
    if new:
        w.writerow(["phase", "rel_path", "sha256", "bytes", "class", "kept_path", "note"])
    for r in ok:
        w.writerow(["P1" if r["class"] == "A" else "P2", r["rel_path"], r["sha256"],
                    r["bytes"], r["class"],
                    "docs 内同哈希副本" if r["class"] == "A" else r["kept_path"], ""])

deleted = collections.Counter(); errs = []
for r in ok:
    p = os.path.join(ROOT, r["rel_path"])
    try:
        os.remove(p); deleted[r["class"]] += 1
    except OSError as e:
        errs.append((r["rel_path"], str(e)))

print("计划删除 A=%d B=%d；跳过(git)=%d；校验失败=%d" % (
    sum(1 for r in ok if r["class"] == "A"), sum(1 for r in ok if r["class"] == "B"),
    len(skip), len(bad)))
print("实际删除 A=%d B=%d；失败=%d" % (deleted["A"], deleted["B"], len(errs)))
for x in skip: print("  SKIP", x[0], "-", x[1])
for x in bad[:10]: print("  BAD ", x[0], "-", x[1])
for x in errs[:10]: print("  ERR ", x[0], "-", x[1])
