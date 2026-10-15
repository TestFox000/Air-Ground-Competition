"""从train帧裁剪GT格号框，拼成带编号的contact sheet供人工读图标注。

输出:
  runs/cell_recog_render/label/crops_124/crop_{i:02d}.png   原生分辨率裁剪
  runs/cell_recog_render/label/sheet_124_{k}.png            拼接大图(带编号)
  runs/cell_recog_render/label/boxes_124.json               框坐标(供lattice拟合)
"""
from __future__ import annotations

import json
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground")
IMG = ROOT / r"capture\datasets\aerial_field_mixed\images\train"
LBL = ROOT / r"capture\datasets\aerial_field_mixed\labels\train"
OUT = ROOT / r"runs\cell_recog_render\label"
CROPS = OUT / "crops_124"
CROPS.mkdir(parents=True, exist_ok=True)

FRAME = "frame_00124"
CELL_CLS = 1
PAD = 1.3          # 裁剪留边比例
SHEET_CELL = 170   # contact sheet 中每格显示尺寸
COLS = 5

try:
    font_big = ImageFont.truetype(r"C:\Windows\Fonts\arialbd.ttf", 22)
    font_idx = ImageFont.truetype(r"C:\Windows\Fonts\arialbd.ttf", 30)
except OSError:
    # 非 Windows / 无该字体时回退 Pillow 默认字体，避免直接崩溃
    font_big = font_idx = ImageFont.load_default()

img = Image.open(IMG / f"{FRAME}.jpg").convert("RGB")
W, H = img.size

boxes = []
for line in (LBL / f"{FRAME}.txt").open(encoding="utf-8"):
    p = line.split()
    if not p or int(p[0]) != CELL_CLS:
        continue
    cx, cy, w, h = map(float, p[1:5])
    boxes.append({"cx": cx * W, "cy": cy * H, "w": w * W, "h": h * H})

boxes.sort(key=lambda b: (b["cy"], b["cx"]))  # 按行列排序，方便读图
print(f"{FRAME}: {len(boxes)} 格号, 原图 {W}x{H}")

crops = []
for i, b in enumerate(boxes):
    side = max(b["w"], b["h"]) * PAD
    x1 = max(0, int(b["cx"] - side / 2))
    y1 = max(0, int(b["cy"] - side / 2))
    x2 = min(W, int(b["cx"] + side / 2))
    y2 = min(H, int(b["cy"] + side / 2))
    crop = img.crop((x1, y1, x2, y2))
    crop.save(CROPS / f"crop_{i:02d}.png")
    crops.append(crop)
    b["side_px"] = side

# 拼contact sheet
rows_per_sheet = 6
per_sheet = COLS * rows_per_sheet
n_sheets = (len(crops) + per_sheet - 1) // per_sheet
for s in range(n_sheets):
    idxs = list(range(s * per_sheet, min((s + 1) * per_sheet, len(crops))))
    sheet = Image.new("RGB", (COLS * SHEET_CELL, rows_per_sheet * (SHEET_CELL + 34)), (30, 30, 30))
    d = ImageDraw.Draw(sheet)
    for j, i in enumerate(idxs):
        r, c = divmod(j, COLS)
        cell = crops[i].resize((SHEET_CELL - 8, SHEET_CELL - 8), Image.LANCZOS)
        sheet.paste(cell, (c * SHEET_CELL + 4, r * (SHEET_CELL + 34) + 34))
        d.text((c * SHEET_CELL + 6, r * (SHEET_CELL + 34) + 4),
               f"#{i}", fill=(255, 255, 0), font=font_big)
    sheet.save(OUT / f"sheet_124_{s}.png")
    print(f"sheet_124_{s}.png: {len(idxs)} 格 (#~{idxs[-1]})")

(OUT / "boxes_124.json").write_text(json.dumps(
    {"frame": FRAME, "W": W, "H": H, "boxes": boxes}), encoding="utf-8")
print(f"框坐标已存 boxes_124.json")
