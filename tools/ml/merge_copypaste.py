# -*- coding: utf-8 -*-
"""合并原始7cls训练集 + copy-paste增强集，创建新的data.yaml。"""
from pathlib import Path
import shutil

SRC_IMG = Path(r"E:\ml_datasets\aerial_field_7cls\images")
SRC_LBL = Path(r"E:\ml_datasets\aerial_field_7cls\labels")
CP_IMG = Path(r"E:\ml_datasets\aerial_field_7cls_copypaste\images")
CP_LBL = Path(r"E:\ml_datasets\aerial_field_7cls_copypaste\labels")
DST = Path(r"E:\ml_datasets\aerial_field_7cls_merged")

# 清空目标
if DST.exists():
    shutil.rmtree(DST)

for split in ["train", "val"]:
    (DST / "images" / split).mkdir(parents=True, exist_ok=True)
    (DST / "labels" / split).mkdir(parents=True, exist_ok=True)

# 复制原始数据
for split in ["train", "val"]:
    for f in (SRC_IMG / split).glob("*"):
        shutil.copy2(f, DST / "images" / split / f.name)
    for f in (SRC_LBL / split).glob("*"):
        shutil.copy2(f, DST / "labels" / split / f.name)
    print(f"{split}: 原始 {len(list((SRC_IMG/split).glob('*')))} 张")

# 追加copy-paste到train
n_cp = 0
for f in (CP_IMG / "train").glob("*"):
    shutil.copy2(f, DST / "images" / "train" / f.name)
    n_cp += 1
for f in (CP_LBL / "train").glob("*"):
    shutil.copy2(f, DST / "labels" / "train" / f.name)
print(f"copy-paste追加: {n_cp} 张")

# 写data.yaml
yaml = f"""path: {DST.as_posix()}
train: images/train
val: images/val

names:
  0: mine
  1: car
  2: arrow
  3: target_red
  4: cell_label
  5: target_blue
  6: obstacle
"""
(DST / "data.yaml").write_text(yaml, encoding="utf-8")
print(f"\n合并完成 → {DST}")
print(f"train: {len(list((DST/'images'/'train').glob('*')))} 张")
print(f"val: {len(list((DST/'images'/'val').glob('*')))} 张")
