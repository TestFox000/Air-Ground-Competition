# -*- coding: utf-8 -*-
"""合并 supplemented + obstacle_aug（去重），增加训练数据和 car 样本。"""
from pathlib import Path
import shutil

SRC1 = Path(r"E:\ml_datasets\aerial_field_7cls_supplemented")
SRC2 = Path(r"E:\ml_datasets\aerial_field_7cls_obstacle_aug")
OUT = Path(r"E:\ml_datasets\aerial_field_7cls_combined")

for sub in ["images", "labels"]:
    for split in ["train", "val"]:
        (OUT / sub / split).mkdir(parents=True, exist_ok=True)

# 复制 supplemented 的 train + val
for split in ["train", "val"]:
    for f in (SRC1 / "images" / split).glob("*.jpg"):
        shutil.copy2(f, OUT / "images" / split / f.name)
        lbl = SRC1 / "labels" / split / (f.stem + ".txt")
        if lbl.exists():
            shutil.copy2(lbl, OUT / "labels" / split / lbl.name)

# 复制 obstacle_aug 的 train（跳过已存在的）
n_added = 0
for f in (SRC2 / "images" / "train").glob("*.jpg"):
    if not (OUT / "images" / "train" / f.name).exists():
        shutil.copy2(f, OUT / "images" / "train" / f.name)
        lbl = SRC2 / "labels" / "train" / (f.stem + ".txt")
        if lbl.exists():
            shutil.copy2(lbl, OUT / "labels" / "train" / lbl.name)
        n_added += 1

print(f"从 obstacle_aug 新增: {n_added} 张")

# 统计
n_train = len(list((OUT / "images" / "train").glob("*.jpg")))
n_val = len(list((OUT / "images" / "val").glob("*.jpg")))
car_train = sum(
    1 for lbl in (OUT / "labels" / "train").glob("*.txt")
    for line in lbl.read_text().strip().splitlines() if line.split()[0] == "1"
)
print(f"训练集: {n_train}, val: {n_val}, car: {car_train}")

# data.yaml
(OUT / "data.yaml").write_text(f"""path: {OUT.as_posix()}
train: images/train
val: images/val
nc: 7
names:
  0: mine
  1: car
  2: arrow
  3: target_red
  4: cell_label
  5: target_blue
  6: obstacle
""", encoding="utf-8")
