# -*- coding: utf-8 -*-
"""官方雷图copy-paste增强 v2 — 针对 explosive_* 域差异（val-B mine召回仅37.5%）。

v1 (build_copypaste_mine.py) 仅用 eval_official74 的74张官方图，把雷贴到航拍背景。
问题：val-B 11张 explosive_* 帧是官方近景原图（背景+尺度与航拍CP不同），模型仍漏检。

v2 改进：
1. 数据源扩大：全部157张官方雷图 - 11张val帧 = 146张（防止数据泄漏）
2. 双域背景：
   - 域A（航拍域）：雷贴到航拍背景，航拍小尺寸（0.085%~0.5%面积）→ 匹配比赛航拍
   - 域B（官方近景域）：雷贴到官方近景背景，近景大尺寸（5%~30%面积）→ 匹配explosive_* val
3. 样本量：域A 500张 + 域B 500张 = 1000张（v1仅500张航拍域）

输出：E:\\ml_datasets\\aerial_field_7cls_mine_cp_v2\\images\\train + labels\\train
供4090恢复后训练使用。
"""
from pathlib import Path
import random
import numpy as np
from PIL import Image, ImageFilter

OFFICIAL_IMG = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\capture\mine_official\images")
OFFICIAL_LBL = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\capture\mine_official\labels")
AERIAL_IMG = Path(r"E:\ml_datasets\aerial_field_7cls\images\train")
AERIAL_LBL = Path(r"E:\ml_datasets\aerial_field_7cls\labels\train")
OUT_IMG = Path(r"E:\ml_datasets\aerial_field_7cls_mine_cp_v2\images\train")
OUT_LBL = Path(r"E:\ml_datasets\aerial_field_7cls_mine_cp_v2\labels\train")
OUT_IMG.mkdir(parents=True, exist_ok=True)
OUT_LBL.mkdir(parents=True, exist_ok=True)

# val-B 中的11张 explosive 帧，必须排除防止数据泄漏
VAL_EXPLOSIVE = {
    "explosive_006", "explosive_008", "explosive_016", "explosive_036",
    "explosive_045", "explosive_056", "explosive_065", "explosive_071",
    "explosive_086", "explosive_107", "explosive_120",
}

NAMES7 = {0: "mine"}

random.seed(2026)


def get_official_images():
    """获取所有官方雷图（排除val-B的11张）。"""
    imgs = []
    for f in sorted(OFFICIAL_IMG.glob("*.jpg")):
        if f.stem in VAL_EXPLOSIVE:
            continue
        imgs.append(f)
    print(f"官方雷图（排除val-B后）: {len(imgs)} 张")
    return imgs


def crop_mines_from_official(official_imgs):
    """从官方图裁出真实雷区域。标签在 OFFICIAL_LBL 目录。"""
    mines = []
    for img_file in official_imgs:
        lbl_file = OFFICIAL_LBL / (img_file.stem + ".txt")
        if not lbl_file.exists():
            continue
        img = Image.open(img_file).convert("RGB")
        W, H = img.size
        for line in lbl_file.read_text(encoding="utf-8").strip().splitlines():
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
            mines.append(img.crop((x1, y1, x2, y2)))
    print(f"从官方图裁出 {len(mines)} 个真实雷")
    return mines


def get_aerial_backgrounds():
    """获取航拍背景图。"""
    bgs = list(AERIAL_IMG.glob("*.jpg"))
    print(f"航拍背景图: {len(bgs)} 张")
    return bgs


def paste_mine(bg_img, mine_crop, size_ratio_range):
    """把雷缩放到指定尺寸范围后随机贴到背景上。"""
    W, H = bg_img.size
    bg_area = W * H
    target_area = bg_area * random.uniform(*size_ratio_range)
    mw, mh = mine_crop.size
    aspect = mw / mh
    new_h = int(np.sqrt(target_area / aspect))
    new_w = int(new_h * aspect)
    new_w = max(8, min(new_w, W - 2))
    new_h = max(8, min(new_h, H - 2))

    mine_resized = mine_crop.resize((new_w, new_h), Image.LANCZOS)
    if random.random() < 0.5:
        mine_resized = mine_resized.filter(ImageFilter.GaussianBlur(radius=random.uniform(0.3, 1.0)))

    margin = max(10, new_w // 10)
    x = random.randint(margin, W - new_w - margin) if W - new_w - margin > margin else 0
    y = random.randint(margin, H - new_h - margin) if H - new_h - margin > margin else 0

    bg_pasted = bg_img.copy()
    bg_pasted.paste(mine_resized, (x, y))

    cx = (x + new_w / 2) / W
    cy = (y + new_h / 2) / H
    w = new_w / W
    h = new_h / H
    return bg_pasted, (cx, cy, w, h)


def build_domain_aerial(mines, bgs, n=500):
    """域A：雷贴到航拍背景，航拍小尺寸。"""
    count = 0
    for i in range(n):
        bg_file = random.choice(bgs)
        bg_img = Image.open(bg_file).convert("RGB")
        lbl_file = AERIAL_LBL / (bg_file.stem + ".txt")
        existing = []
        if lbl_file.exists():
            existing = [l for l in lbl_file.read_text(encoding="utf-8").strip().splitlines() if l]
        n_mines = random.randint(3, 8)
        new_labels = []
        for _ in range(n_mines):
            mine_crop = random.choice(mines)
            bg_img, (cx, cy, w, h) = paste_mine(bg_img, mine_crop, (0.0008, 0.005))
            new_labels.append(f"0 {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")
        out_name = f"cp_aerial_{i:04d}.jpg"
        bg_img.save(OUT_IMG / out_name, quality=90)
        (OUT_LBL / (Path(out_name).stem + ".txt")).write_text(
            "\n".join(existing + new_labels) + "\n", encoding="utf-8")
        count += 1
    print(f"域A（航拍域）生成 {count} 张")
    return count


def build_domain_official(mines, official_imgs, n=500):
    """域B：雷贴到官方近景背景，近景大尺寸（匹配explosive_* val分布）。"""
    count = 0
    for i in range(n):
        bg_file = random.choice(official_imgs)
        bg_img = Image.open(bg_file).convert("RGB")
        # 官方近景背景不保留原有标签（避免重复标注），仅保留新贴的雷
        n_mines = random.randint(1, 4)
        new_labels = []
        for _ in range(n_mines):
            mine_crop = random.choice(mines)
            bg_img, (cx, cy, w, h) = paste_mine(bg_img, mine_crop, (0.05, 0.30))
            new_labels.append(f"0 {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")
        out_name = f"cp_official_{i:04d}.jpg"
        bg_img.save(OUT_IMG / out_name, quality=90)
        (OUT_LBL / (Path(out_name).stem + ".txt")).write_text(
            "\n".join(new_labels) + "\n", encoding="utf-8")
        count += 1
    print(f"域B（官方近景域）生成 {count} 张")
    return count


def main():
    official_imgs = get_official_images()
    mines = crop_mines_from_official(official_imgs)
    if not mines:
        print("错误：未裁到任何雷，请检查官方图标签")
        return
    bgs = get_aerial_backgrounds()
    n_a = build_domain_aerial(mines, bgs, n=500)
    n_b = build_domain_official(mines, official_imgs, n=500)
    print(f"\n完成！共生成 {n_a + n_b} 张copy-paste增强图 → {OUT_IMG}")
    print(f"  域A（航拍域）: {n_a} 张 → 提升航拍域mine召回")
    print(f"  域B（官方近景域）: {n_b} 张 → 提升explosive_*域mine召回")


if __name__ == "__main__":
    main()
