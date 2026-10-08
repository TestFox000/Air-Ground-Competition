#!/usr/bin/env python3
"""合成格号训练数据：用 PIL 渲染格号 + 航拍域随机化，解决 154 张真实样本过拟合问题。

真实航拍格号与裁判系统截图的域差异：
  - 透视变形（无人机非正射）
  - 旋转（无人机偏航）
  - 尺度（高度变化）
  - 运动模糊 / 高斯模糊
  - 光照 / 阴影 / 对比度
  - 背景纹理（草地、塑胶、灰尘）
  - JPEG 压缩伪影

用法：
  python tools/ml/gen_cell_synth.py --per-cell 80 --out datasets/cell_labels/synth
"""

from __future__ import annotations

import argparse
import csv
import io
import random
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageEnhance, ImageFilter, ImageFont

LETTERS = "ABCDEFGHIJKLMN"
ROWS = list(range(1, 12))

# 候选字体：以 Arial 为主（匹配裁判系统渲染风格）
FONT_CANDIDATES = [
    "C:/Windows/Fonts/arial.ttf",
    "C:/Windows/Fonts/arialbd.ttf",
    "C:/Windows/Fonts/calibri.ttf",
    "C:/Windows/Fonts/tahoma.ttf",
    "C:/Windows/Fonts/verdana.ttf",
]

# 真实格号颜色：浅灰背景 + 深蓝文字（裁判系统截图采样）
BG_COLORS = [(232, 232, 232), (228, 230, 235), (236, 236, 238), (224, 226, 230)]
TEXT_COLORS = [(40, 50, 90), (35, 45, 85), (50, 60, 100), (30, 40, 80)]


def load_font(size: int) -> ImageFont.FreeTypeFont:
    """随机选一个可用字体（Arial 优先）。"""
    random.shuffle(FONT_CANDIDATES)
    for path in FONT_CANDIDATES:
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            continue
    return ImageFont.load_default()


def make_background(size: int) -> Image.Image:
    """浅灰背景 + 轻微噪声（匹配裁判系统格子底色）。"""
    bg = random.choice(BG_COLORS)
    arr = np.full((size, size, 3), bg, dtype=np.uint8)
    noise = np.random.randint(-8, 9, (size, size, 3), dtype=np.int16)
    arr = np.clip(arr.astype(np.int16) + noise, 0, 255).astype(np.uint8)
    return Image.fromarray(arr)


def render_label(letter: str, number: int, canvas: int) -> Image.Image:
    """渲染格号：浅灰背景 + 深蓝文字 + 左上角位置（匹配真实裁判系统风格）。"""
    img = make_background(canvas)
    draw = ImageDraw.Draw(img)
    # 字号：真实格号文字较小，约占格子高度 28%~40%
    font_size = int(canvas * random.uniform(0.26, 0.40))
    font = load_font(font_size)
    text = f"{letter}{number}"
    bbox = draw.textbbox((0, 0), text, font=font)
    # 文字在左上角，留边距
    margin_x = random.randint(3, 8)
    margin_y = random.randint(2, 6)
    x = margin_x - bbox[0]
    y = margin_y - bbox[1]
    color = random.choice(TEXT_COLORS)
    draw.text((x, y), text, font=font, fill=color)
    return img


def apply_domain_randomization(img: Image.Image, canvas: int) -> Image.Image:
    """模拟航拍域差异：透视、旋转、模糊、降分辨率、光照、压缩。"""
    bg_gray = random.choice(BG_COLORS)
    # 1. 透视变形（无人机非正射，幅度适中）
    if random.random() < 0.6:
        w, h = img.size
        max_shift = int(canvas * 0.12)
        shifts = [random.randint(-max_shift, max_shift) for _ in range(4)]
        src = [(0, 0), (w, 0), (w, h), (0, h)]
        dst = [
            (shifts[0], shifts[1]),
            (w - shifts[2], shifts[3]),
            (w - shifts[1], h - shifts[0]),
            (shifts[3], h - shifts[2]),
        ]
        coeffs = _find_coeffs(src, dst)
        img = img.transform((w, h), Image.PERSPECTIVE, coeffs, Image.BICUBIC)

    # 2. 旋转（无人机偏航，格号大致水平，角度较小）
    angle = random.uniform(-20, 20)
    img = img.rotate(angle, resample=Image.BICUBIC, expand=False, fillcolor=bg_gray)

    # 3. 尺度抖动（裁剪回 96）
    scale = random.uniform(0.85, 1.15)
    new_size = int(canvas * scale)
    img = img.resize((new_size, new_size), Image.BICUBIC)
    if new_size > canvas:
        left = (new_size - canvas) // 2
        top = (new_size - canvas) // 2
        img = img.crop((left, top, left + canvas, top + canvas))
    else:
        bg = make_background(canvas)
        bg.paste(img, ((canvas - new_size) // 2, (canvas - new_size) // 2))
        img = bg

    # 3.5 降分辨率模拟（不同飞行高度: 先缩小到40-72px再放大回96, 产生真实模糊）
    if random.random() < 0.6:
        small = random.randint(40, 72)
        img = img.resize((small, small), Image.BICUBIC).resize((canvas, canvas), Image.BICUBIC)

    # 4. 高斯模糊（无人机移动/失焦, 加重以匹配真实航拍模糊度）
    if random.random() < 0.7:
        img = img.filter(ImageFilter.GaussianBlur(radius=random.uniform(0.5, 2.2)))

    # 5. 光照 / 对比度
    img = ImageEnhance.Brightness(img).enhance(random.uniform(0.7, 1.25))
    img = ImageEnhance.Contrast(img).enhance(random.uniform(0.75, 1.25))
    img = ImageEnhance.Sharpness(img).enhance(random.uniform(0.7, 1.3))

    # 6. JPEG 压缩伪影
    if random.random() < 0.5:
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=random.randint(60, 90))
        buf.seek(0)
        img = Image.open(buf).convert("RGB")

    return img


def _find_coeffs(src: list, dst: list) -> list[float]:
    """计算透视变换系数（PIL PERSPECTIVE 用 8 个系数）。"""
    matrix = []
    for (x, y), (X, Y) in zip(src, dst):
        matrix.append([x, y, 1, 0, 0, 0, -X * x, -X * y, X])
        matrix.append([0, 0, 0, x, y, 1, -Y * x, -Y * y, Y])
    A = np.array(matrix, dtype=np.float64)
    b = A[:, -1]
    A = A[:, :-1]
    coeffs = np.linalg.solve(A, b)
    return coeffs.tolist()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--per-cell", type=int, default=80, help="每个格号生成样本数")
    parser.add_argument("--out", type=Path, default=Path("datasets/cell_labels/synth"))
    parser.add_argument("--canvas", type=int, default=96)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    random.seed(args.seed)
    np.random.seed(args.seed)

    args.out.mkdir(parents=True, exist_ok=True)
    crop_dir = args.out / "crops"
    crop_dir.mkdir(exist_ok=True)

    rows = []
    idx = 0
    for letter in LETTERS:
        for number in ROWS:
            cell_id = f"{letter}{number}"
            for i in range(args.per_cell):
                img = render_label(letter, number, args.canvas)
                img = apply_domain_randomization(img, args.canvas)
                fname = f"{cell_id}_syn{i:03d}.png"
                img.save(crop_dir / fname)
                split = "train" if (idx % 10) < 8 else "val"
                rows.append({
                    "crop_path": f"crops/{fname}",
                    "split": split,
                    "cell_id": cell_id,
                    "column": letter,
                    "row": str(number),
                    "status": "labeled",
                })
                idx += 1

    csv_path = args.out / "annotations.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["crop_path", "split", "cell_id", "column", "row", "status"])
        writer.writeheader()
        writer.writerows(rows)

    print(f"生成 {len(rows)} 张合成样本 -> {args.out}")
    train_n = sum(1 for r in rows if r["split"] == "train")
    print(f"train={train_n} val={len(rows) - train_n}")


if __name__ == "__main__":
    main()
