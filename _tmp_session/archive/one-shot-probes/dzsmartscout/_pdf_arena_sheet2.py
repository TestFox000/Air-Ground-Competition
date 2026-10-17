import os
from PIL import Image, ImageDraw

d = r"D:\User\Desktop\dzsmartscout\fc-audit\arena_pdf_img"
items = [
    (3, 1, "P3 Fig1: arena panorama", None),
    (8, 1, "P8 Fig6: bridge structure", None),
    (9, 2, "P9 Fig8: arrow example B", None),
    (4, 1, "P4 Fig2: judge-system UI", None),
]

CELL = 1010
GAP = 14
cols, rows = 2, 2
W = cols * CELL + (cols + 1) * GAP
H = rows * CELL + (rows + 1) * GAP
sheet = Image.new("RGB", (W, H), (255, 255, 255))
dr = ImageDraw.Draw(sheet)

for k, (pg, idx, cap, _) in enumerate(items):
    p = os.path.join(d, "p%02d_%d.png" % (pg, idx))
    im = Image.open(p).convert("RGB")
    box = (CELL - 8, CELL - 38)
    if im.width < box[0] and im.height < box[1]:
        f = min(box[0] / im.width, box[1] / im.height)
        im = im.resize((int(im.width * f), int(im.height * f)), Image.LANCZOS)
    else:
        im.thumbnail(box, Image.LANCZOS)
    cx, cy = k % cols, k // cols
    ox = GAP + cx * (CELL + GAP)
    oy = GAP + cy * (CELL + GAP)
    x = ox + (CELL - im.width) // 2
    y = oy + 32 + (CELL - 38 - im.height) // 2
    sheet.paste(im, (x, y))
    dr.text((ox + 6, oy + 10), cap, fill=(180, 0, 0))

op = os.path.join(d, "_sheet_key2.png")
sheet.save(op)
print(op, sheet.size, os.path.getsize(op))
