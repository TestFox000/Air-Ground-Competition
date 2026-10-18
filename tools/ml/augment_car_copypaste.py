# -*- coding: utf-8 -*-
"""car 类 copy-paste 增强：从含 car 的图裁剪 car，贴到无 car 背景图上，生成合成样本。
目标：把 car 训练样本从 82 扩增到 ~500，缓解类别极度不平衡。
"""
from pathlib import Path
from PIL import Image
import random, shutil

SRC = Path(r"E:\ml_datasets\aerial_field_7cls_supplemented")
OUT = Path(r"E:\ml_datasets\aerial_field_7cls_car_aug")
NAMES = {0:"mine",1:"car",2:"arrow",3:"target_red",4:"cell_label",5:"target_blue",6:"obstacle"}

random.seed(42)

# 复制原始数据到输出目录
for split in ["train", "val"]:
    for sub in ["images", "labels"]:
        src_dir = SRC / sub / split
        out_dir = OUT / sub / split
        out_dir.mkdir(parents=True, exist_ok=True)
        if src_dir.exists():
            for f in src_dir.iterdir():
                shutil.copy2(f, out_dir / f.name)

# 收集 car 裁剪
car_crops = []  # (PIL.Image, w, h)
train_lbl = SRC / "labels" / "train"
train_img = SRC / "images" / "train"
for lbl_file in train_lbl.glob("*.txt"):
    img_file = train_img / (lbl_file.stem + ".jpg")
    if not img_file.exists():
        continue
    img = Image.open(img_file).convert("RGB")
    W, H = img.size
    for line in lbl_file.read_text().strip().splitlines():
        parts = line.split()
        cls = int(parts[0])
        if cls != 1:
            continue
        cx, cy, w, h = map(float, parts[1:5])
        x1 = int((cx - w/2) * W); y1 = int((cy - h/2) * H)
        x2 = int((cx + w/2) * W); y2 = int((cy + h/2) * H)
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(W, x2), min(H, y2)
        if x2 - x1 < 10 or y2 - y1 < 10:
            continue
        crop = img.crop((x1, y1, x2, y2))
        car_crops.append(crop)

print(f"收集到 {len(car_crops)} 个 car 裁剪")

# 收集无 car 的背景图（训练集）
bg_files = []
for lbl_file in train_lbl.glob("*.txt"):
    has_car = False
    for line in lbl_file.read_text().strip().splitlines():
        if line.split()[0] == "1":
            has_car = True
            break
    if not has_car:
        img_file = train_img / (lbl_file.stem + ".jpg")
        if img_file.exists():
            bg_files.append((img_file, lbl_file))

print(f"无 car 背景图: {len(bg_files)} 张")

# 对每张背景图贴 1-3 个 car，生成合成样本
out_img = OUT / "images" / "train"
out_lbl = OUT / "labels" / "train"
aug_count = 0

for bg_img_path, bg_lbl_path in bg_files:
    bg = Image.open(bg_img_path).convert("RGB")
    W, H = bg.size
    # 读取已有标注
    existing = []
    for line in bg_lbl_path.read_text().strip().splitlines():
        parts = line.split()
        if parts:
            existing.append((int(parts[0]), *map(float, parts[1:5])))
    
    # 随机贴 1-3 个 car
    n_cars = random.randint(1, 3)
    new_lines = [f"{c} {cx} {cy} {w} {h}" for c, cx, cy, w, h in existing]
    
    for _ in range(n_cars):
        crop = random.choice(car_crops)
        cw, ch = crop.size
        # 随机缩放 0.7-1.3
        scale = random.uniform(0.7, 1.3)
        nw, nh = int(cw * scale), int(ch * scale)
        if nw < 10 or nh < 10:
            continue
        crop_r = crop.resize((nw, nh), Image.BICUBIC)
        # 随机位置
        px = random.randint(0, max(0, W - nw))
        py = random.randint(0, max(0, H - nh))
        bg.paste(crop_r, (px, py))
        # YOLO 标注
        cx = (px + nw / 2) / W
        cy = (py + nh / 2) / H
        w = nw / W
        h = nh / H
        new_lines.append(f"1 {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")
    
    # 保存
    out_name = f"aug_{aug_count:04d}"
    bg.save(out_img / f"{out_name}.jpg", quality=90)
    (out_lbl / f"{out_name}.txt").write_text("\n".join(new_lines) + "\n")
    aug_count += 1

print(f"生成 {aug_count} 张合成样本")

# 写 data.yaml
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

# 统计新数据集 car 数量
car_count = 0
for lbl in (OUT / "labels" / "train").glob("*.txt"):
    for line in lbl.read_text().strip().splitlines():
        if line.split()[0] == "1":
            car_count += 1
print(f"增强后训练集 car 标注数: {car_count}")
