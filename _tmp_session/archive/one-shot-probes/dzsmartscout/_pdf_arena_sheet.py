import os
from PIL import Image, ImageDraw

d = r"D:\User\Desktop\dzsmartscout\fc-audit\arena_pdf_img"
# (page, idx, caption)
items = [
    (7, 1, "P7 Fig5: obstacles = water-filled colored buckets"),
    (8, 1, "P8 Fig6: simulated bridge / tunnel structure"),
    (4, 1, "P4 Fig2: randomized layout example"),
    (9, 1, "P9 Fig7: passable-arrow on bridge top"),
]

CELL = 1000
GAP = 16
cols, rows = 2, 2
W = cols * CELL + (cols + 1) * GAP
H = rows * CELL + (rows + 1) * GAP
sheet = Image.new("RGB", (W, H), (255, 255, 255))
dr = ImageDraw.Draw(sheet)

for k, (pg, idx, cap) in enumerate(items):
    p = os.path.join(d, "p%02d_%d.png" % (pg, idx))
    im = Image.open(p).convert("RGB")
    im.thumbnail((CELL - 10, CELL - 40), Image.LANCZOS)
    cx = k % cols
    cy = k // cols
    x = GAP + cx * (CELL + GAP) + (CELL - im.width) // 2
    y = GAP + cy * (CELL + GAP) + 34 + (CELL - 40 - im.height) // 2
    sheet.paste(im, (x, y))
    dr.text((GAP + cx * (CELL + GAP) + 6, GAP + cy * (CELL + GAP) + 10),
            cap, fill=(180, 0, 0))

op = os.path.join(d, "_sheet_key.png")
sheet.save(op)
print(op, sheet.size, os.path.getsize(op))
