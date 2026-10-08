"""验证 CellClassifier 训练数据：看看原始裁剪图和增强后的图。"""
import sys
sys.path.insert(0, r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\reference\onboard\dzuav\src")
from pathlib import Path
from cell_classifier.dataset import load_records, build_transform, CellLabelDataset
from PIL import Image
import torch
import numpy as np

CSV = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\datasets\cell_labels\annotations.csv")
records = load_records(CSV, "train")
print(f"train records: {len(records)}")

# 看前 5 张原图
for r in records[:5]:
    im = Image.open(r.image_path)
    print(f"{r.cell_id}: size={im.size}, path={r.image_path.name}")

# 看增强后的图
transform = build_transform(96, training=True)
ds = CellLabelDataset(records, transform)
print(f"\n增强后样本数: {len(ds)}")

# 保存增强前后对比
OUT = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\datasets\cell_labels\debug")
OUT.mkdir(exist_ok=True)

LETTERS = "ABCDEFGHIJKLMN"
for i in range(min(8, len(ds))):
    tensor, li, ri = ds[i]
    # 反归一化
    mean = torch.tensor([0.485, 0.456, 0.406]).view(3, 1, 1)
    std = torch.tensor([0.229, 0.224, 0.225]).view(3, 1, 1)
    arr = ((tensor * std + mean) * 255).clamp(0, 255).byte().permute(1, 2, 0).numpy()
    aug = Image.fromarray(arr)
    aug.save(OUT / f"aug_{i}_{LETTERS[li]}{ri+1}.png")

print(f"增强样本已保存到 {OUT}")
