# -*- coding: utf-8 -*-
"""箭头copy-paste增强：把训练集中的箭头裁出，回贴到航拍训练背景。

解决 arrow 召回率极低问题（val 11个GT仅检出2个，召回18%）。
箭头为桥洞通行方向标识，需保留方向性，旋转以90°增量为主。

输出：新的训练图片+标签，追加到7cls数据集train。
用法: python tools/ml/build_copypaste_arrow.py [--n 500]
"""
from pathlib import Path
import argparse
import random
import numpy as np
from PIL import Image, ImageFilter, ImageEnhance

TRAIN_IMG = Path(r"E:\ml_datasets\aerial_field_7cls\images\train")
TRAIN_LBL = Path(r"E:\ml_datasets\aerial_field_7cls\labels\train")
OUT_IMG = Path(r"E:\ml_datasets\aerial_field_7cls_arrow_cp\images\train")
OUT_LBL = Path(r"E:\ml_datasets\aerial_field_7cls_arrow_cp\labels\train")

ARROW_CLS = 2  # 7cls中arrow的类号

random.seed(42)


def crop_arrows_from_train():
    """从训练帧裁出箭头区域（带padding）。"""
    arrows = []
    for img_file in sorted(TRAIN_IMG.glob("*.jpg")):
        lbl_file = TRAIN_LBL / (img_file.stem + ".txt")
        if not lbl_file.exists():
            continue
        img = Image.open(img_file).convert("RGB")
        W, H = img.size
        for line in lbl_file.read_text().strip().splitlines():
            parts = line.split()
            if not parts or int(parts[0]) != ARROW_CLS:
                continue
            cx, cy, w, h = map(float, parts[1:5])
            pad = 0.2  # 多裁20%边缘，保留箭头周围上下文
            x1 = int((cx - w / 2 - pad * w) * W)
            y1 = int((cy - h / 2 - pad * h) * H)
            x2 = int((cx + w / 2 + pad * w) * W)
            y2 = int((cy + h / 2 + pad * h) * H)
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(W, x2), min(H, y2)
            if x2 - x1 < 10 or y2 - y1 < 10:
                continue
            crop = img.crop((x1, y1, x2, y2))
            arrows.append(crop)
    print(f"从训练帧裁出 {len(arrows)} 个箭头")
    return arrows


def get_train_backgrounds():
    """获取训练背景图（优先选arrow少的图作为背景）。"""
    bgs = []
    for img_file in sorted(TRAIN_IMG.glob("*.jpg")):
        lbl_file = TRAIN_LBL / (img_file.stem + ".txt")
        arr_count = 0
        if lbl_file.exists():
            for line in lbl_file.read_text().strip().splitlines():
                if line and int(line.split()[0]) == ARROW_CLS:
                    arr_count += 1
        bgs.append((img_file, arr_count))
    bgs.sort(key=lambda x: x[1])  # arrow少的优先
    return bgs


def augment_crop(crop):
    """对箭头裁剪做几何+颜色增强，保留方向性（90°旋转为主）。"""
    # 90°旋转（箭头天然四方向）
    if random.random() < 0.6:
        k = random.choice([0, 1, 2, 3])
        crop = crop.rotate(90 * k, expand=True)
    # 小角度扰动 ±10°
    if random.random() < 0.4:
        angle = random.uniform(-10, 10)
        crop = crop.rotate(angle, expand=True, fillcolor=(0, 0, 0))
    # 随机缩放 0.6~1.4
    if random.random() < 0.6:
        scale = random.uniform(0.6, 1.4)
        nw = max(10, int(crop.width * scale))
        nh = max(10, int(crop.height * scale))
        crop = crop.resize((nw, nh), Image.LANCZOS)
    # 颜色抖动
    if random.random() < 0.5:
        crop = ImageEnhance.Color(crop).enhance(random.uniform(0.7, 1.3))
    if random.random() < 0.5:
        crop = ImageEnhance.Brightness(crop).enhance(random.uniform(0.8, 1.2))
    if random.random() < 0.5:
        crop = ImageEnhance.Contrast(crop).enhance(random.uniform(0.7, 1.3))
    # 轻微模糊
    if random.random() < 0.3:
        crop = crop.filter(ImageFilter.GaussianBlur(radius=random.uniform(0.2, 1.0)))
    return crop


def paste_arrow_on_bg(bg_img, arr_crop, target_area_ratio=0.003):
    """把箭头缩放到航拍尺寸后随机贴到背景上。
    target_area_ratio: 箭头占画面面积比例（航拍箭头较小，约0.1%~1%）。
    """
    W, H = bg_img.size
    bg_area = W * H
    target_area = bg_area * target_area_ratio
    ow, oh = arr_crop.size
    aspect = ow / oh
    new_h = int(np.sqrt(target_area / aspect))
    new_w = int(new_h * aspect)
    new_w = max(10, min(new_w, W // 4))
    new_h = max(10, min(new_h, H // 4))
    arr_resized = arr_crop.resize((new_w, new_h), Image.LANCZOS)

    # 随机位置（避开边缘）
    margin = 15
    x = random.randint(margin, max(margin, W - new_w - margin))
    y = random.randint(margin, max(margin, H - new_h - margin))

    bg_pasted = bg_img.copy()
    bg_pasted.paste(arr_resized, (x, y))

    cx = (x + new_w / 2) / W
    cy = (y + new_h / 2) / H
    w = new_w / W
    h = new_h / H
    return bg_pasted, (cx, cy, w, h)


def build_copypaste_dataset(n_samples=500):
    """构建箭头copy-paste增强数据集。"""
    arrows = crop_arrows_from_train()
    if not arrows:
        print("未裁到任何箭头，退出")
        return
    bgs = get_train_backgrounds()
    print(f"训练背景图: {len(bgs)} 张")

    OUT_IMG.mkdir(parents=True, exist_ok=True)
    OUT_LBL.mkdir(parents=True, exist_ok=True)

    count = 0
    for i in range(n_samples):
        bg_file, _ = random.choice(bgs)
        bg_img = Image.open(bg_file).convert("RGB")

        # 读取背景原有标签
        lbl_file = TRAIN_LBL / (bg_file.stem + ".txt")
        existing_labels = []
        if lbl_file.exists():
            for line in lbl_file.read_text().strip().splitlines():
                if line:
                    existing_labels.append(line)

        # 贴1~3个箭头
        n_arr = random.randint(1, 3)
        new_arr_labels = []
        for _ in range(n_arr):
            arr_crop = random.choice(arrows)
            arr_crop = augment_crop(arr_crop)
            # 箭头面积比例范围（航拍箭头较小）
            area_ratio = random.uniform(0.001, 0.01)
            bg_img, (cx, cy, w, h) = paste_arrow_on_bg(bg_img, arr_crop, area_ratio)
            new_arr_labels.append(f"{ARROW_CLS} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")

        out_name = f"arr_cp_{i:04d}.jpg"
        bg_img.save(OUT_IMG / out_name, quality=90)
        all_labels = existing_labels + new_arr_labels
        (OUT_LBL / (Path(out_name).stem + ".txt")).write_text("\n".join(all_labels) + "\n")
        count += 1
        if (i + 1) % 100 == 0:
            print(f"  已生成 {i+1}/{n_samples}")

    print(f"\n完成！生成 {count} 张箭头copy-paste增强图 → {OUT_IMG}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=500, help="生成样本数")
    args = ap.parse_args()
    build_copypaste_dataset(n_samples=args.n)
