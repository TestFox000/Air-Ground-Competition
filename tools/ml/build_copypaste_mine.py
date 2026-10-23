# -*- coding: utf-8 -*-
"""官方雷图copy-paste增强：把官方真实手雷裁成航拍小框尺寸，回贴到航拍背景。

解决域差异：让检测器直接学习"航拍视角下的真实手雷纹理"。
输出：新的训练图片+标签，追加到7cls数据集。
"""
from pathlib import Path
import random
import numpy as np
from PIL import Image, ImageFilter

OFFICIAL_IMG = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\runs\eval_official74\images")
OFFICIAL_LBL = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\runs\eval_official74\labels")
AERIAL_IMG = Path(r"E:\ml_datasets\aerial_field_7cls\images\train")
AERIAL_LBL = Path(r"E:\ml_datasets\aerial_field_7cls\labels\train")
OUT_IMG = Path(r"E:\ml_datasets\aerial_field_7cls_copypaste\images\train")
OUT_LBL = Path(r"E:\ml_datasets\aerial_field_7cls_copypaste\labels\train")
OUT_IMG.mkdir(parents=True, exist_ok=True)
OUT_LBL.mkdir(parents=True, exist_ok=True)

NAMES7 = {0: "mine", 1: "car", 2: "arrow", 3: "target_red",
          4: "cell_label", 5: "target_blue", 6: "obstacle"}

random.seed(42)


def crop_mines_from_official():
    """从官方74张精确标注图中裁出真实雷区域。"""
    mines = []
    for lbl_file in sorted(OFFICIAL_LBL.glob("*.txt")):
        img_file = OFFICIAL_IMG / (lbl_file.stem + ".jpg")
        if not img_file.exists():
            continue
        img = Image.open(img_file).convert("RGB")
        W, H = img.size
        for line in lbl_file.read_text().strip().splitlines():
            parts = line.split()
            if not parts or int(parts[0]) != 0:
                continue
            cx, cy, w, h = map(float, parts[1:5])
            pad = 0.1
            x1 = int((cx - w / 2 - pad * w) * W)
            y1 = int((cy - h / 2 - pad * h) * H)
            x2 = int((cx + w / 2 + pad * w) * W)
            y2 = int((cy + h / 2 + pad * h) * H)
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(W, x2), min(H, y2)
            if x2 - x1 < 10 or y2 - y1 < 10:
                continue
            crop = img.crop((x1, y1, x2, y2))
            mines.append(crop)
    print(f"从官方图裁出 {len(mines)} 个真实雷")
    return mines


def get_aerial_backgrounds():
    """获取航拍背景图（优先选mine少的图作为背景）。"""
    bgs = []
    for img_file in sorted(AERIAL_IMG.glob("*.jpg")):
        lbl_file = AERIAL_LBL / (img_file.stem + ".txt")
        mine_count = 0
        if lbl_file.exists():
            for line in lbl_file.read_text().strip().splitlines():
                if line and int(line.split()[0]) == 0:
                    mine_count += 1
        bgs.append((img_file, mine_count))
    # 按雷数量排序，雷少的优先作为背景
    bgs.sort(key=lambda x: x[1])
    return bgs


def paste_mine_on_bg(bg_img, mine_crop, target_size_ratio=0.01):
    """把雷缩放到航拍尺寸后随机贴到背景上。
    target_size_ratio: 雷占画面的面积比例（航拍雷约0.085%~0.5%）
    """
    W, H = bg_img.size
    bg_area = W * H
    # 目标面积 = bg_area * ratio
    target_area = bg_area * target_size_ratio
    # 按雷的原始宽高比缩放
    mw, mh = mine_crop.size
    aspect = mw / mh
    new_h = int(np.sqrt(target_area / aspect))
    new_w = int(new_h * aspect)
    new_w = max(8, min(new_w, W // 2))
    new_h = max(8, min(new_h, H // 2))

    mine_resized = mine_crop.resize((new_w, new_h), Image.LANCZOS)
    # 轻微模糊模拟航拍距离感
    if random.random() < 0.5:
        mine_resized = mine_resized.filter(ImageFilter.GaussianBlur(radius=random.uniform(0.3, 1.0)))

    # 随机位置（避开边缘）
    margin = 10
    x = random.randint(margin, W - new_w - margin)
    y = random.randint(margin, H - new_h - margin)

    # 简单alpha混合（边缘羽化）
    bg_pasted = bg_img.copy()
    # 用paste直接覆盖（航拍雷是卡片，贴在地面上）
    bg_pasted.paste(mine_resized, (x, y))

    # YOLO标签
    cx = (x + new_w / 2) / W
    cy = (y + new_h / 2) / H
    w = new_w / W
    h = new_h / H
    return bg_pasted, (cx, cy, w, h)


def build_copypaste_dataset(n_samples=500):
    """构建copy-paste增强数据集。"""
    mines = crop_mines_from_official()
    bgs = get_aerial_backgrounds()
    print(f"航拍背景图: {len(bgs)} 张")

    count = 0
    for i in range(n_samples):
        # 选背景图
        bg_file, _ = random.choice(bgs)
        bg_img = Image.open(bg_file).convert("RGB")
        W, H = bg_img.size

        # 读取原有标签
        lbl_file = AERIAL_LBL / (bg_file.stem + ".txt")
        existing_labels = []
        if lbl_file.exists():
            for line in lbl_file.read_text().strip().splitlines():
                if line:
                    existing_labels.append(line)

        # 贴3~8个雷
        n_mines = random.randint(3, 8)
        new_mine_labels = []
        for _ in range(n_mines):
            mine_crop = random.choice(mines)
            # 随机尺寸（航拍雷大小范围）
            size_ratio = random.uniform(0.0008, 0.005)
            bg_img, (cx, cy, w, h) = paste_mine_on_bg(bg_img, mine_crop, size_ratio)
            new_mine_labels.append(f"0 {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")

        # 保存
        out_name = f"cp_{i:04d}.jpg"
        bg_img.save(OUT_IMG / out_name, quality=90)
        all_labels = existing_labels + new_mine_labels
        (OUT_LBL / (Path(out_name).stem + ".txt")).write_text("\n".join(all_labels) + "\n")
        count += 1
        if (i + 1) % 100 == 0:
            print(f"  已生成 {i+1}/{n_samples}")

    print(f"\n完成！生成 {count} 张copy-paste增强图 → {OUT_IMG}")


if __name__ == "__main__":
    build_copypaste_dataset(n_samples=500)
