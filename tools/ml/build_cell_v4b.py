"""构建 cell_classifier v4b 训练集：真实航拍裁剪强域增广 + 裁判图 + 新合成图。

v4a 诊断结论: 过拟合单帧(训练帧97%, 其他帧11.5%), 错误集中在行号2/3/5混淆
(低分辨率模糊). v4b 对策:
  - 70张train裁剪 × 40强域增广变体(模糊/降分辨率/透视/旋转/JPEG) = 2800
  - 合成图用加重域随机化的新生成器(synth_b)
  - val 保持与v4相同的12张航拍原图 + 9裁判 + 新合成156 (可对比v4a)

成分(train): 原图420 + 增广2800 + 裁判870 + 合成2000 = 6090
输出: datasets/cell_labels/aerial_124_aug/ + synth_b/ + merged_v4b.csv
"""
from __future__ import annotations

import csv
import json
import random
import subprocess
import sys
from collections import Counter
from pathlib import Path

from PIL import Image

ROOT = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground")
TOOLS_ML = ROOT / "tools" / "ml"
sys.path.insert(0, str(TOOLS_ML))
from gen_cell_synth import apply_domain_randomization  # noqa: E402

LABEL_DIR = ROOT / r"runs\cell_recog_render\label"
CROPS_SRC = LABEL_DIR / "crops_124"
DATASET = ROOT / r"datasets\cell_labels"
AERIAL_OUT = DATASET / "aerial_124"
AUG_OUT = DATASET / "aerial_124_aug"
SYNTH_B = DATASET / "synth_b"
AUG_PER_IMG = 40

random.seed(42)

# ---------- 1. 载入已验证标签 (与 v4 相同的 val 划分) ----------
labels = json.loads((LABEL_DIR / "labels_124_final.json").read_text(encoding="utf-8"))
aerial = []  # (idx, cell_id, quality)
for k, v in labels.items():
    if v["quality"] == "bad":
        continue
    aerial.append((int(k), v["cell_id"], v["quality"]))

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
print(f"航拍: {len(aerial)} 张 (train {len(train_aerial)} / val {len(val_keys)})")

# ---------- 2. 增广变体: 70张 train 裁剪 × 40 ----------
AUG_OUT.mkdir(parents=True, exist_ok=True)
aug_rows = []
for idx, cid in train_aerial:
    src = AERIAL_OUT / f"a124_{idx:02d}_{cid}.png"
    with Image.open(src) as im:
        base = im.convert("RGB").copy()
    for v in range(AUG_PER_IMG):
        out = AUG_OUT / f"aug_{idx:02d}_{cid}_{v:02d}.png"
        apply_domain_randomization(base, 96).save(out)
        aug_rows.append({"crop_path": f"aerial_124_aug/{out.name}", "split": "train",
                         "cell_id": cid, "column": cid[0], "row": cid[1:], "status": "labeled"})
print(f"增广变体: {len(aug_rows)} 张 -> {AUG_OUT}")

# ---------- 3. 新合成图 (加重域随机化) ----------
if not (SYNTH_B / "annotations.csv").is_file():
    subprocess.run([sys.executable, str(TOOLS_ML / "gen_cell_synth.py"),
                    "--per-cell", "14", "--out", str(SYNTH_B)], check=True)
synth_rows = []
with (SYNTH_B / "annotations.csv").open(encoding="utf-8-sig") as f:
    for r in csv.DictReader(f):
        synth_rows.append({"crop_path": f"synth_b/{r['crop_path']}", "split": r["split"],
                           "cell_id": r["cell_id"], "column": r["column"],
                           "row": r["row"], "status": "labeled"})
random.shuffle(synth_rows)
synth_train = [r for r in synth_rows if r["split"] == "train"][:2000]
synth_val = [r for r in synth_rows if r["split"] == "val"][:156]
print(f"新合成: train {len(synth_train)} / val {len(synth_val)}")

# ---------- 4. 原始航拍 + 裁判图 ----------
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

# ---------- 5. 合并写出 ----------
rows += aug_rows + synth_train + synth_val
random.shuffle(rows)
out_csv = DATASET / "merged_v4b.csv"
with out_csv.open("w", encoding="utf-8", newline="") as f:
    w = csv.DictWriter(f, fieldnames=["crop_path", "split", "cell_id", "column", "row", "status"])
    w.writeheader()
    w.writerows(rows)

tr, va = Counter(), Counter()
for r in rows:
    tag = ("aerial_aug" if "aug_" in r["crop_path"] else
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
