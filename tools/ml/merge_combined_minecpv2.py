# -*- coding: utf-8 -*-
"""合并 combined 数据集(2087 train) + mine_cp_v2(1000 train)，针对 mine FN=38 瓶颈。

mine_cp_v2 直接针对 15 个 explosive_* 域差异 mine FN + 23 个航拍域 mine FN。
用法: python tools/ml/merge_combined_minecpv2.py
"""
from pathlib import Path
import shutil

COMBINED = Path(r"E:\ml_datasets\aerial_field_7cls_combined")
MINE_CP_V2 = Path(r"E:\ml_datasets\aerial_field_7cls_mine_cp_v2")
DST = Path(r"E:\ml_datasets\aerial_field_7cls_combined_minecpv2")

# 清空目标
if DST.exists():
    shutil.rmtree(DST)

for split in ["train", "val"]:
    (DST / "images" / split).mkdir(parents=True, exist_ok=True)
    (DST / "labels" / split).mkdir(parents=True, exist_ok=True)

# 复制 combined 全部数据
n_combined = 0
for split in ["train", "val"]:
    for f in (COMBINED / "images" / split).glob("*"):
        shutil.copy2(f, DST / "images" / split / f.name)
        n_combined += 1
    for f in (COMBINED / "labels" / split).glob("*"):
        shutil.copy2(f, DST / "labels" / split / f.name)
print(f"combined 复制: {n_combined} 张")

# 追加 mine_cp_v2 到 train
n_minecp = 0
for f in (MINE_CP_V2 / "images" / "train").glob("*"):
    shutil.copy2(f, DST / "images" / "train" / f.name)
    n_minecp += 1
for f in (MINE_CP_V2 / "labels" / "train").glob("*"):
    shutil.copy2(f, DST / "labels" / "train" / f.name)
print(f"mine_cp_v2 追加: {n_minecp} 张")

# 写 data.yaml
yaml = f"""path: {DST.as_posix()}
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
"""
(DST / "data.yaml").write_text(yaml, encoding="utf-8")

train_count = len(list((DST / "images" / "train").glob("*")))
val_count = len(list((DST / "images" / "val").glob("*")))
print(f"\n合并完成 -> {DST}")
print(f"train: {train_count} 张")
print(f"val: {val_count} 张")
