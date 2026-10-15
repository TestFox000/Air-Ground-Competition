"""多帧版: 裁剪指定帧的GT格号框, 拼带编号contact sheet供人工读图标注。

用法: python make_sheets_multi.py frame_00170 val
输出: runs/cell_recog_render/label/{frame}/crops/crop_{i:02d}.png
      runs/cell_recog_render/label/{frame}/sheet_{k}.png
      runs/cell_recog_render/label/{frame}/boxes.json
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground")

FRAME = sys.argv[1] if len(sys.argv) > 1 else "frame_00170"
SPLIT = sys.argv[2] if len(sys.argv) > 2 else "val"
IMG = ROOT / rf"capture\datasets\aerial_field_mixed\images\{SPLIT}"
LBL = ROOT / rf"capture\datasets\aerial_field_mixed\labels\{SPLIT}"
OUT = ROOT / "runs/cell_recog_render/label" / FRAME
CROPS = OUT / "crops"
CROPS.mkdir(parents=True, exist_ok=True)

CELL_CLS = 1
PAD = 1.3
SHEET_CELL = 200   # 200px 大格, 便于人工读图
COLS = 5
ROWS_PER_SHEET = 6

try:
    font_big = ImageFont.truetype(r"C:\Windows\Fonts\arialbd.ttf", 26)
    font_idx = ImageFont.truetype(r"C:\Windows\Fonts\arialbd.ttf", 34)
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

boxes.sort(key=lambda b: (b["cy"], b["cx"]))
print(f"{FRAME}: {len(boxes)} 格号, 原图 {W}x{H}, 输出 {OUT}")

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

per_sheet = COLS * ROWS_PER_SHEET
n_sheets = (len(crops) + per_sheet - 1) // per_sheet
for s in range(n_sheets):
    idxs = list(range(s * per_sheet, min((s + 1) * per_sheet, len(crops))))
    sheet = Image.new("RGB", (COLS * SHEET_CELL, ROWS_PER_SHEET * (SHEET_CELL + 40)), (30, 30, 30))
    d = ImageDraw.Draw(sheet)
    for j, i in enumerate(idxs):
        r, c = divmod(j, COLS)
        cell = crops[i].resize((SHEET_CELL - 8, SHEET_CELL - 8), Image.LANCZOS)
        sheet.paste(cell, (c * SHEET_CELL + 4, r * (SHEET_CELL + 40) + 40))
        d.text((c * SHEET_CELL + 6, r * (SHEET_CELL + 40) + 4),
               f"#{i}", fill=(255, 255, 0), font=font_big)
    sheet.save(OUT / f"sheet_{s}.png")
    print(f"sheet_{s}.png: #~{idxs[-1]}")

(OUT / "boxes.json").write_text(json.dumps(
    {"frame": FRAME, "W": W, "H": H, "boxes": boxes}), encoding="utf-8")
