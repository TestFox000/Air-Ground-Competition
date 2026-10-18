# -*- coding: utf-8 -*-
"""car 类上采样：复制含 car 的图片多份，增加 car 类出现频率，不引入假阳性。
对比 copy-paste：上采样是真实样本重复，car 特征真实，不会引入合成伪影。
"""
from pathlib import Path
import shutil

SRC = Path(r"E:\ml_datasets\aerial_field_7cls_supplemented")
OUT = Path(r"E:\ml_datasets\aerial_field_7cls_car_upsampled")
REPEAT = 8  # 含 car 的图片复制 8 份

for split in ["train", "val"]:
    for sub in ["images", "labels"]:
        (OUT / sub / split).mkdir(parents=True, exist_ok=True)

train_img = SRC / "images" / "train"
train_lbl = SRC / "labels" / "train"
out_img = OUT / "images" / "train"
out_lbl = OUT / "labels" / "train"

# 先复制所有原始训练图
n_orig = 0
for f in train_img.glob("*.jpg"):
    shutil.copy2(f, out_img / f.name)
    lbl = train_lbl / (f.stem + ".txt")
    if lbl.exists():
        shutil.copy2(lbl, out_lbl / lbl.name)
    n_orig += 1

# 找含 car 的图片，复制 REPEAT 份
car_files = []
for lbl in train_lbl.glob("*.txt"):
    has_car = any(line.split()[0] == "1" for line in lbl.read_text().strip().splitlines() if line)
    if has_car:
        car_files.append(lbl.stem)

print(f"原始训练图: {n_orig}, 含car: {len(car_files)}")

n_aug = 0
for stem in car_files:
    for r in range(REPEAT):
        new_name = f"{stem}_up{r}"
        src_img = train_img / f"{stem}.jpg"
        src_lbl = train_lbl / f"{stem}.txt"
        if src_img.exists():
            shutil.copy2(src_img, out_img / f"{new_name}.jpg")
            shutil.copy2(src_lbl, out_lbl / f"{new_name}.txt")
            n_aug += 1

print(f"上采样生成: {n_aug} 张")

# 复制 val
for f in (SRC / "images" / "val").glob("*.jpg"):
    shutil.copy2(f, OUT / "images" / "val" / f.name)
    lbl = SRC / "labels" / "val" / (f.stem + ".txt")
    if lbl.exists():
        shutil.copy2(lbl, OUT / "labels" / "val" / lbl.name)

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

# 统计
car_count = 0
for lbl in (OUT / "labels" / "train").glob("*.txt"):
    for line in lbl.read_text().strip().splitlines():
        if line.split()[0] == "1":
            car_count += 1
print(f"增强后训练集 car 标注数: {car_count}")
