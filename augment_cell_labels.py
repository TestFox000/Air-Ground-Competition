"""对 train 集格号图做离线增强，每张生成 20 个版本，扩充训练数据。"""
import csv
from pathlib import Path
import random
from PIL import Image, ImageEnhance, ImageFilter
import numpy as np

BASE = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\datasets\cell_labels")
CSV = BASE / "annotations.csv"
CROPS = BASE / "crops"
AUG = BASE / "aug_crops"
AUG.mkdir(exist_ok=True)

# 读取原始记录
records = []
with CSV.open("r", encoding="utf-8-sig", newline="") as f:
    for row in csv.DictReader(f):
        records.append(row)

random.seed(42)
new_records = []
aug_count = 0

for r in records:
    if r["split"] != "train":
        continue
    src = CROPS / f"{r['cell_id']}.png"
    im = Image.open(src).convert("RGB")
    # 原图保留
    new_records.append({**r, "crop_path": f"crops/{r['cell_id']}.png"})
    # 生成 20 个增强版本
    for i in range(20):
        aug = im.copy()
        # 旋转 ±30 度
        angle = random.uniform(-30, 30)
        aug = aug.rotate(angle, resample=Image.BICUBIC, fillcolor=(114, 114, 114))
        # 缩放 0.85-1.15
        scale = random.uniform(0.85, 1.15)
        w, h = aug.size
        nw, nh = int(w * scale), int(h * scale)
        aug = aug.resize((nw, nh), Image.BICUBIC)
        # 裁剪/填充回 96
        canvas = Image.new("RGB", (96, 96), (114, 114, 114))
        ox, oy = (96 - nw) // 2, (96 - nh) // 2
        canvas.paste(aug, (ox, oy))
        aug = canvas
        # 亮度
        aug = ImageEnhance.Brightness(aug).enhance(random.uniform(0.7, 1.3))
        # 对比度
        aug = ImageEnhance.Contrast(aug).enhance(random.uniform(0.7, 1.3))
        # 模糊（概率 20%）
        if random.random() < 0.2:
            aug = aug.filter(ImageFilter.GaussianBlur(radius=random.uniform(0.5, 1.5)))
        # 加噪声
        if random.random() < 0.3:
            arr = np.array(aug).astype(np.float32)
            noise = np.random.normal(0, random.uniform(5, 15), arr.shape)
            arr = np.clip(arr + noise, 0, 255).astype(np.uint8)
            aug = Image.fromarray(arr)
        # 保存
        aug_name = f"{r['cell_id']}_aug{i:02d}.png"
        aug.save(AUG / aug_name)
        new_records.append({
            "crop_path": f"aug_crops/{aug_name}",
            "split": "train",
            "cell_id": r["cell_id"],
            "column": r["column"],
            "row": r["row"],
            "status": "labeled",
        })
        aug_count += 1

# val 记录直接保留（用原图）
for r in records:
    if r["split"] == "val":
        new_records.append({**r, "crop_path": f"crops/{r['cell_id']}.png"})

# 写新 CSV
new_csv = BASE / "annotations_aug.csv"
with new_csv.open("w", encoding="utf-8-sig", newline="") as f:
    w = csv.DictWriter(f, fieldnames=["crop_path", "split", "cell_id", "column", "row", "status"])
    w.writeheader()
    w.writerows(new_records)

print(f"原始 train: 132, 增强后 train: {132 + aug_count}, val: 22")
print(f"总样本: {len(new_records)}")
print(f"增强 CSV: {new_csv}")
