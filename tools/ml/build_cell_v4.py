"""构建 cell_classifier v4 训练集：真实航拍裁剪 + 裁判图 + 合成图降采样。

成分(train):
  - 航拍 frame_00124 已验证裁剪 70张 × 15 上采样 = 1050
  - 裁判图 crops 145张 × 6 上采样 = 870
  - 合成图 ~2000 (从9240降采样, 每图1次)
成分(val, 每图仅1次):
  - 航拍 12 + 裁判 9 + 合成 200
输出: datasets/cell_labels/aerial_124/ (裁剪图) + merged_v4.csv
"""
from __future__ import annotations

import csv
import json
import random
import shutil
from pathlib import Path

ROOT = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground")
LABEL_DIR = ROOT / r"runs\cell_recog_render\label"
CROPS_SRC = LABEL_DIR / "crops_124"
DATASET = ROOT / r"datasets\cell_labels"
AERIAL_OUT = DATASET / "aerial_124"
AERIAL_OUT.mkdir(parents=True, exist_ok=True)

random.seed(42)

# 1. 载入已验证标签
labels = json.loads((LABEL_DIR / "labels_124_final.json").read_text(encoding="utf-8"))
aerial = []  # (src, cell_id, quality)
for k, v in labels.items():
    if v["quality"] == "bad":
        continue
    aerial.append((int(k), v["cell_id"], v["quality"]))
print(f"可用航拍裁剪: {len(aerial)} (hi={sum(1 for a in aerial if a[2]=='hi')}, mid={sum(1 for a in aerial if a[2]=='mid')})")

# 2. 航拍 val 选择: 每字母最多1张(排除A保train覆盖), 尽量选高置信、行号分散
by_letter = {}
for idx, cid, q in aerial:
    by_letter.setdefault(cid[0], []).append((idx, cid, q))
val_keys = set()
for letter, items in sorted(by_letter.items()):
    if letter == "A":
        continue  # A只有A5, 必须留在train
    # 优先选 hi 质量
    items.sort(key=lambda x: (x[2] != "hi", x[1]))
    pick = items[0]
    val_keys.add(pick[0])
# 控制val数量 ~12
if len(val_keys) > 12:
    val_keys = set(sorted(val_keys)[:12])
print(f"航拍 val: {sorted(val_keys)} -> {[cid for i,cid,q in aerial if i in val_keys]}")

# 3. 复制裁剪图
rows = []
for idx, cid, q in aerial:
    src = CROPS_SRC / f"crop_{idx:02d}.png"
    dst = AERIAL_OUT / f"a124_{idx:02d}_{cid}.png"
    shutil.copy2(src, dst)
    rel = f"aerial_124/{dst.name}"
    split = "val" if idx in val_keys else "train"
    reps = 1 if split == "val" else 15
    for _ in range(reps):
        rows.append({"crop_path": rel, "split": split, "cell_id": cid,
                     "column": cid[0], "row": cid[1:], "status": "labeled"})

# 4. 裁判图: 从 merged_v2.csv 提取 crops/ 行 (154张, 3080行=20x上采样) -> 改为 6x/1x
v2 = list(csv.DictReader((DATASET / "merged_v2.csv").open(encoding="utf-8-sig")))
referee_seen = {}
for r in v2:
    if not r["crop_path"].startswith("crops/"):
        continue
    key = r["crop_path"]
    referee_seen.setdefault(key, r)
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
print(f"裁判图: {len(ref_keys)} 张 (val {len(ref_val)})")

# 5. 合成图: 从 9240 行降采样 2200 (train 2000 + val 200)
synth_rows = [r for r in v2 if r["crop_path"].startswith("synth/")]
random.shuffle(synth_rows)
for r in synth_rows[:2000]:
    r2 = dict(r); r2["split"] = "train"
    rows.append(r2)
for r in synth_rows[2000:2200]:
    r2 = dict(r); r2["split"] = "val"
    rows.append(r2)

# 6. 汇总写出
random.shuffle(rows)
out_csv = DATASET / "merged_v4.csv"
with out_csv.open("w", encoding="utf-8", newline="") as f:
    w = csv.DictWriter(f, fieldnames=["crop_path", "split", "cell_id", "column", "row", "status"])
    w.writeheader()
    w.writerows(rows)

from collections import Counter
tr = Counter()
va = Counter()
src_of = Counter()
for r in rows:
    (tr if r["split"] == "train" else va)[
        "aerial" if r["crop_path"].startswith("aerial") else
        ("referee" if r["crop_path"].startswith("crops") else "synth")] += 1
print(f"\ntrain: {dict(tr)}  总={sum(tr.values())}")
print(f"val:   {dict(va)}  总={sum(va.values())}")

# 检查train的字母/行覆盖
letters = set(r["column"] for r in rows if r["split"] == "train")
nums = set(r["row"] for r in rows if r["split"] == "train")
print(f"train字母覆盖: {sorted(letters)}  行覆盖: {sorted(nums, key=int)}")
assert len(letters) == 14 and len(nums) == 11, "类别覆盖不全!"
print(f"\n已写 {out_csv} ({len(rows)} 行)")
