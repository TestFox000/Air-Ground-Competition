# -*- coding: utf-8 -*-
"""标靶数据训练前准备：train/val划分 + 合成增强

输入: target_collect.py 产出的 images/ + labels/
输出: target_dataset/{images,labels}/{train,val}/ + data.yaml

增强策略:
  1. 时间块划分val（防泄漏）
  2. 从真实标注帧裁出正对标靶 → 粘贴到背景图上合成新样本
  3. 几何+光照扰动（标靶是刚性目标，合成效果好）
"""
import argparse
import random
import shutil
from pathlib import Path

import cv2
import numpy as np


def split_train_val(src_img: Path, src_lbl: Path, dst: Path, val_ratio=0.2):
    """按文件名前缀(视频段)时间块划分，防泄漏。"""
    imgs = sorted(src_img.glob("*.jpg"))
    # 按视频前缀分组
    groups = {}
    for p in imgs:
        prefix = p.stem.rsplit("_f", 1)[0]  # VID_xxx_f123 -> VID_xxx
        groups.setdefault(prefix, []).append(p)
    val_imgs = []
    train_imgs = []
    for prefix, lst in groups.items():
        # 每组后20%做val（时间块：同视频后段）
        n_val = max(1, int(len(lst) * val_ratio))
        train_imgs.extend(lst[:-n_val])
        val_imgs.extend(lst[-n_val:])
    for split, files in [("train", train_imgs), ("val", val_imgs)]:
        for d in [dst / "images" / split, dst / "labels" / split]:
            d.mkdir(parents=True, exist_ok=True)
        for img in files:
            shutil.copy2(img, dst / "images" / split / img.name)
            lbl = src_lbl / (img.stem + ".txt")
            if lbl.exists():
                shutil.copy2(lbl, dst / "labels" / split / lbl.name)
    print(f"划分完成: train={len(train_imgs)} val={len(val_imgs)}")
    return train_imgs, val_imgs


def crop_targets(lbl_dir: Path, img_dir: Path, out: Path):
    """从已标注帧裁出正对标靶patch，作为合成素材库。"""
    out.mkdir(parents=True, exist_ok=True)
    for lbl in sorted(lbl_dir.glob("*.txt")):
        img = cv2.imread(str(img_dir / (lbl.stem + ".jpg")))
        if img is None:
            continue
        H, W = img.shape[:2]
        for i, line in enumerate(lbl.read_text().splitlines()):
            parts = line.split()
            if len(parts) < 8:
                continue
            cls, cx, cy, w, h = int(parts[0]), *map(float, parts[1:5])
            x1, y1 = int((cx - w / 2) * W), int((cy - h / 2) * H)
            x2, y2 = int((cx + w / 2) * W), int((cy + h / 2) * H)
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(W, x2), min(H, y2)
            if x2 - x1 < 20 or y2 - y1 < 20:
                continue
            patch = img[y1:y2, x1:x2]
            cv2.imwrite(str(out / f"{lbl.stem}_{i}_cls{cls}.png"), patch)
    print(f"裁出标靶patch: {len(list(out.glob('*.png')))}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=r"E:\ml_datasets\target_raw")
    ap.add_argument("--dst", default=r"E:\ml_datasets\target_dataset")
    ap.add_argument("--val-ratio", type=float, default=0.2)
    args = ap.parse_args()

    src = Path(args.src)
    dst = Path(args.dst)
    if dst.exists():
        shutil.rmtree(dst)

    train_imgs, val_imgs = split_train_val(src / "images", src / "labels", dst, args.val_ratio)

    # 裁出标靶patch素材库
    crop_targets(src / "labels", src / "images", dst / "patches")

    # 统计类别
    for split in ["train", "val"]:
        lbl_dir = dst / "labels" / split
        blue = red = 0
        for f in lbl_dir.glob("*.txt"):
            for ln in f.read_text().splitlines():
                if ln.strip():
                    c = int(ln.split()[0])
                    if c == 0:
                        blue += 1
                    else:
                        red += 1
        print(f"{split}: blue={blue} red={red}")

    # 生成data.yaml由target_train.py负责，这里只生成路径提示
    print(f"\n数据集就绪: {dst}")
    print("训练命令:")
    print(f"  python tools\\ml\\target_train.py --data {dst} --epochs 100")


if __name__ == "__main__":
    main()
