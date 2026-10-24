
# -*- coding: utf-8 -*-
"""库外文档清理 P0：全量清单 + 三分类（A可证冗余/B内部重复/C唯一）"""
import os, sys, hashlib, csv, json, collections

ROOT = os.path.abspath(r"D:\User\Desktop\比赛存档\v1")
LIB = os.path.join(ROOT, "docs")
SKIP_DIRS = {".git", "node_modules", "__pycache__", ".venv", "venv"}
OUT = os.path.join(LIB, "_work", "inventory.csv")

def md_files(base, skip_lib=False):
    out = []
    for dp, dns, fns in os.walk(base):
        dns[:] = [d for d in dns if d not in SKIP_DIRS]
        if skip_lib:
            dns[:] = [d for d in dns if os.path.abspath(os.path.join(dp, d)) != LIB]
        for fn in fns:
            if fn.lower().endswith(".md"):
                p = os.path.join(dp, fn)
                try:
                    out.append(os.path.abspath(p))
                except OSError:
                    pass
    return out

def sha(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for c in iter(lambda: f.read(1 << 20), b""):
            h.update(c)
    return h.hexdigest()

# 库内全部文件哈希（不只 md：C2 快照可能和库里非 md 撞）
lib_hashes = {}
lib_count = 0
for dp, dns, fns in os.walk(LIB):
    dns[:] = [d for d in dns if d not in SKIP_DIRS]
    for fn in fns:
        p = os.path.join(dp, fn)
        try:
            lib_hashes.setdefault(sha(p), []).append(os.path.relpath(p, ROOT))
            lib_count += 1
        except OSError:
            pass

files = md_files(ROOT, skip_lib=True)
recs = []
for p in files:
    try:
        st = os.stat(p); h = sha(p)
    except OSError:
        continue
    recs.append({"abs": p, "rel": os.path.relpath(p, ROOT).replace("\\", "/"),
                 "size": st.st_size, "sha": h})

groups = collections.defaultdict(list)
for r in recs:
    groups[r["sha"]].append(r)

rows = []
for r in recs:
    g = groups[r["sha"]]
    if r["sha"] in lib_hashes:
        cls = "A"; kept = lib_hashes[r["sha"]][0]; gs = len(g)
        reason = "与库内同哈希：" + kept
    elif len(g) > 1:
        keep = sorted(g, key=lambda x: (len(x["rel"]), x["rel"]))[0]
        if keep["abs"] == r["abs"]:
            cls = "C"; kept = ""; gs = len(g)
            reason = "库外 %d 份重复，本份为保留份" % len(g)
        else:
            cls = "B"; kept = keep["rel"]; gs = len(g)
            reason = "库外 %d 份重复，保留 %s" % (len(g), kept)
    else:
        cls = "C"; kept = ""; gs = 1
        reason = "全仓唯一"
    top = r["rel"].split("/")[0]
    rows.append([r["rel"], top, r["size"], r["sha"], cls, gs, kept, reason])

rows.sort(key=lambda x: (x[4], x[0]))
os.makedirs(os.path.dirname(OUT), exist_ok=True)
with open(OUT, "w", newline="", encoding="utf-8-sig") as f:
    w = csv.writer(f)
    w.writerow(["id", "rel_path", "top_dir", "bytes", "sha256", "class", "group_size", "kept_path", "reason"])
    for i, r in enumerate(rows, 1):
        w.writerow([i] + r)

cnt = collections.Counter(r[4] for r in rows)
print("lib_files=%d  outside_md=%d" % (lib_count, len(rows)))
print("A=%d B=%d C=%d" % (cnt["A"], cnt["B"], cnt["C"]))
print("--- A/B by top ---")
for k in ("A", "B"):
    d = collections.Counter(r[1] for r in rows if r[4] == k)
    print(k, dict(d))
print("--- by top (all) ---")
for k, v in collections.Counter(r[1] for r in rows).most_common():
    print("  %-22s %d" % (k, v))
json.dump({"lib": lib_count, "outside": len(rows), "A": cnt["A"], "B": cnt["B"], "C": cnt["C"]},
          open(os.path.join(LIB, "_work", "inventory_summary.json"), "w"), indent=1)
