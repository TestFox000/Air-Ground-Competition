# -*- coding: utf-8 -*-
"""合并原始7cls训练集 + mine copy-paste + obstacle copy-paste，创建新data.yaml。

针对 obstacle FN=30 瓶颈：在原始数据基础上追加障碍copy-paste增强样本。
用法: python tools/ml/merge_copypaste_obstacle.py
"""
from pathlib import Path
import shutil

SRC_IMG = Path(r"E:\ml_datasets\aerial_field_7cls\images")
SRC_LBL = Path(r"E:\ml_datasets\aerial_field_7cls\labels")
MINE_CP_IMG = Path(r"E:\ml_datasets\aerial_field_7cls_copypaste\images\train")
MINE_CP_LBL = Path(r"E:\ml_datasets\aerial_field_7cls_copypaste\labels\train")
OBS_CP_IMG = Path(r"E:\ml_datasets\aerial_field_7cls_obstacle_cp\images\train")
OBS_CP_LBL = Path(r"E:\ml_datasets\aerial_field_7cls_obstacle_cp\labels\train")
DST = Path(r"E:\ml_datasets\aerial_field_7cls_obstacle_aug")

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

# 追加mine copy-paste到train（如存在）
n_mine_cp = 0
if MINE_CP_IMG.exists():
    for f in MINE_CP_IMG.glob("*"):
        shutil.copy2(f, DST / "images" / "train" / f.name)
        n_mine_cp += 1
    for f in MINE_CP_LBL.glob("*"):
        shutil.copy2(f, DST / "labels" / "train" / f.name)
    print(f"mine copy-paste追加: {n_mine_cp} 张")

# 追加obstacle copy-paste到train
n_obs_cp = 0
for f in OBS_CP_IMG.glob("*"):
    shutil.copy2(f, DST / "images" / "train" / f.name)
    n_obs_cp += 1
for f in OBS_CP_LBL.glob("*"):
    shutil.copy2(f, DST / "labels" / "train" / f.name)
print(f"obstacle copy-paste追加: {n_obs_cp} 张")

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

train_count = len(list((DST / "images" / "train").glob("*")))
val_count = len(list((DST / "images" / "val").glob("*")))
print(f"\n合并完成 → {DST}")
print(f"train: {train_count} 张")
print(f"val: {val_count} 张")
