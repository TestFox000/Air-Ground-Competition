"""构建 cell_classifier v4c 训练集 = v4b 成分 + 四个val批次帧人工标注128张。

破局死循环: v4a 只认124批次(跨批次11.5%), 自举只能标认识批次 -> 人工标注4个val批次代表帧:
  frame_00170(55) + frame_00099(33) + frame_00250(22) + frame_00020(18) = 128 张 (hi+mid)
新增成分: 原图×6 + 40强域增广变体 -> aerial_newf/ + aerial_newf_aug/
val 保持与 v4b 完全一致 (12航拍+9裁判+156合成=177, 可与v4a/v4b对比)

成分(train): 124原图420 + 124增广2800 + 新帧原图768 + 新帧增广5120 + 裁判870 + 合成2000 = 11978
输出: datasets/cell_labels/aerial_newf/ + aerial_newf_aug/ + merged_v4c.csv
"""
from __future__ import annotations

import csv
import json
import random
import shutil
import sys
from collections import Counter
from pathlib import Path

from PIL import Image

ROOT = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground")
TOOLS_ML = ROOT / "tools" / "ml"
sys.path.insert(0, str(TOOLS_ML))
from gen_cell_synth import apply_domain_randomization  # noqa: E402

LABEL_DIR = ROOT / r"runs\cell_recog_render\label"
DATASET = ROOT / r"datasets\cell_labels"
NEWF_OUT = DATASET / "aerial_newf"
NEWF_AUG = DATASET / "aerial_newf_aug"
AERIAL_OUT = DATASET / "aerial_124"
AUG_OUT = DATASET / "aerial_124_aug"
SYNTH_B = DATASET / "synth_b"
AUG_PER_IMG = 40
NEW_FRAMES = ["frame_00170", "frame_00099", "frame_00250", "frame_00020"]

random.seed(42)
NEWF_OUT.mkdir(parents=True, exist_ok=True)
NEWF_AUG.mkdir(parents=True, exist_ok=True)

# ---------- 1. 新四帧标注 (128张): 复制原图 + 增广 ----------
newf = []  # (rel_path, cell_id)
for frame in NEW_FRAMES:
    labels = json.loads((LABEL_DIR / frame / "labels_final.json").read_text(encoding="utf-8"))
    for k, v in sorted(labels.items(), key=lambda x: int(x[0])):
        if v["quality"] not in ("hi", "mid"):
            continue
        cid = v["cell_id"]
        src = LABEL_DIR / frame / "crops" / f"crop_{int(k):02d}.png"
        dst = NEWF_OUT / f"{frame}_f{int(k):02d}_{cid}.png"
        shutil.copy2(src, dst)
        newf.append((f"aerial_newf/{dst.name}", cid))
print(f"新帧标注: {len(newf)} 张 "
      f"({', '.join(f'{f}:{sum(1 for r, c in newf if f in r)}' for f in NEW_FRAMES)})")

aug_rows = []
for rel, cid in newf:
    with Image.open(DATASET / rel) as im:
        base = im.convert("RGB").copy()
    stem = Path(rel).stem
    for v in range(AUG_PER_IMG):
        out = NEWF_AUG / f"aug_{stem}_{v:02d}.png"
        apply_domain_randomization(base, 96).save(out)
        aug_rows.append({"crop_path": f"aerial_newf_aug/{out.name}", "split": "train",
                         "cell_id": cid, "column": cid[0], "row": cid[1:], "status": "labeled"})
print(f"新帧增广: {len(aug_rows)} 张")

# ---------- 2. 124帧成分 (与 v4b 相同 val 划分) ----------
labels = json.loads((LABEL_DIR / "labels_124_final.json").read_text(encoding="utf-8"))
aerial = [(int(k), v["cell_id"], v["quality"]) for k, v in labels.items() if v["quality"] != "bad"]
by_letter = {}
for idx, cid, q in aerial:
    by_letter.setdefault(cid[0], []).append((idx, cid, q))
val_keys = set()
for letter, items in sorted(by_letter.items()):
    if letter == "A":
        continue
    items.sort(key=lambda x: (x[2] != "hi", x[1]))
    val_keys.add(items[0][0])
if len(val_keys) > 12:
    val_keys = set(sorted(val_keys)[:12])
train_aerial = [(i, c) for i, c, _ in aerial if i not in val_keys]

rows = []
for idx, cid in train_aerial:
    rel = f"aerial_124/a124_{idx:02d}_{cid}.png"
    for _ in range(6):
        rows.append({"crop_path": rel, "split": "train", "cell_id": cid,
                     "column": cid[0], "row": cid[1:], "status": "labeled"})
for idx in sorted(val_keys):
    cid = next(c for i, c, _ in aerial if i == idx)
    rows.append({"crop_path": f"aerial_124/a124_{idx:02d}_{cid}.png", "split": "val",
                 "cell_id": cid, "column": cid[0], "row": cid[1:], "status": "labeled"})

# 124帧已有增广图直接复用
n_aug124 = 0
for fp in sorted(AUG_OUT.glob("aug_*.png")):
    cid = fp.stem.split("_")[2]
    rows.append({"crop_path": f"aerial_124_aug/{fp.name}", "split": "train",
                 "cell_id": cid, "column": cid[0], "row": cid[1:], "status": "labeled"})
    n_aug124 += 1
print(f"124帧增广复用: {n_aug124} 张")

# 新帧原图 ×6
for rel, cid in newf:
    for _ in range(6):
        rows.append({"crop_path": rel, "split": "train", "cell_id": cid,
                     "column": cid[0], "row": cid[1:], "status": "labeled"})

# ---------- 3. 裁判图 + 合成 (与 v4b 相同) ----------
v2 = list(csv.DictReader((DATASET / "merged_v2.csv").open(encoding="utf-8-sig")))
referee_seen = {}
for r in v2:
    if r["crop_path"].startswith("crops/"):
        referee_seen.setdefault(r["crop_path"], r)
ref_keys = sorted(referee_seen)
random.shuffle(ref_keys)
ref_val = set(ref_keys[:9])
for key in ref_keys:
    r = referee_seen[key]
    split = "val" if key in ref_val else "train"
    reps = 1 if split == "val" else 6
    for _ in range(reps):
        rows.append({"crop_path": key, "split": split, "cell_id": r["cell_id"],
                     "column": r["column"], "row": r["row"], "status": "labeled"})

synth_rows = []
with (SYNTH_B / "annotations.csv").open(encoding="utf-8-sig") as f:
    for r in csv.DictReader(f):
        synth_rows.append({"crop_path": f"synth_b/{r['crop_path']}", "split": r["split"],
                           "cell_id": r["cell_id"], "column": r["column"],
                           "row": r["row"], "status": "labeled"})
random.shuffle(synth_rows)
synth_train = [r for r in synth_rows if r["split"] == "train"][:2000]
synth_val = [r for r in synth_rows if r["split"] == "val"][:156]

# ---------- 4. 合并写出 ----------
rows += aug_rows + synth_train + synth_val
random.shuffle(rows)
out_csv = DATASET / "merged_v4c.csv"
with out_csv.open("w", encoding="utf-8", newline="") as f:
    w = csv.DictWriter(f, fieldnames=["crop_path", "split", "cell_id", "column", "row", "status"])
    w.writeheader()
    w.writerows(rows)

tr, va = Counter(), Counter()
for r in rows:
    tag = ("newf_aug" if "newf_aug" in r["crop_path"] else
           "newf" if "aerial_newf/" in r["crop_path"] else
           "aerial_aug" if "aug_" in r["crop_path"] else
           "aerial" if r["crop_path"].startswith("aerial") else
           "referee" if r["crop_path"].startswith("crops") else "synth")
    (tr if r["split"] == "train" else va)[tag] += 1
print(f"\ntrain: {dict(tr)}  总={sum(tr.values())}")
print(f"val:   {dict(va)}  总={sum(va.values())}")
letters = set(r["column"] for r in rows if r["split"] == "train")
nums = set(r["row"] for r in rows if r["split"] == "train")
print(f"train字母覆盖: {sorted(letters)}  行覆盖: {sorted(nums, key=int)}")
assert len(letters) == 14 and len(nums) == 11, "类别覆盖不全!"
print(f"\n已写 {out_csv} ({len(rows)} 行)")
