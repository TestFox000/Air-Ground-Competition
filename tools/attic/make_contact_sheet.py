"""把 154 张裁剪图拼成一张大图，检查对齐情况。"""
from pathlib import Path
from PIL import Image
import numpy as np

CROPS = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\datasets\cell_labels\crops")
LETTERS = "ABCDEFGHIJKLMN"
COLS, ROWS = 14, 11
CELL = 96

canvas = Image.new("RGB", (COLS * CELL, ROWS * CELL), (128, 128, 128))
for row in range(ROWS):
    for col in range(COLS):
        cell_id = f"{LETTERS[col]}{row + 1}"
        p = CROPS / f"{cell_id}.png"
        if p.exists():
            im = Image.open(p).resize((CELL, CELL))
            canvas.paste(im, (col * CELL, row * CELL))

out = r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\datasets\cell_labels\contact_sheet.png"
canvas.save(out)
print(f"拼图已保存: {out}")
print(f"尺寸: {canvas.size}")
