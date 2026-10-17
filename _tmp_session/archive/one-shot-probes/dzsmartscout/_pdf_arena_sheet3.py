import os
from PIL import Image, ImageDraw

d = r"D:\User\Desktop\dzsmartscout\fc-audit\arena_pdf_img"
items = [
    (5, 1, "P5 Fig3: mines = flat 12cm pictures"),
    (6, 1, "P6 Fig4: movable electronic targets"),
    (11, 1, "P11 Fig10: white adhesive strip A"),
    (10, 1, "P10 Fig9: start/finish zones"),
]
CELL = 1010
GAP = 14
cols, rows = 2, 2
W = cols * CELL + (cols + 1) * GAP
H = rows * CELL + (rows + 1) * GAP
sheet = Image.new("RGB", (W, H), (255, 255, 255))
dr = ImageDraw.Draw(sheet)
for k, (pg, idx, cap) in enumerate(items):
    im = Image.open(os.path.join(d, "p%02d_%d.png" % (pg, idx))).convert("RGB")
    box = (CELL - 8, CELL - 38)
    if im.width < box[0] and im.height < box[1]:
        f = min(box[0] / im.width, box[1] / im.height)
        im = im.resize((int(im.width * f), int(im.height * f)), Image.LANCZOS)
    else:
        im.thumbnail(box, Image.LANCZOS)
    cx, cy = k % cols, k // cols
    ox, oy = GAP + cx * (CELL + GAP), GAP + cy * (CELL + GAP)
    sheet.paste(im, (ox + (CELL - im.width) // 2,
                     oy + 32 + (CELL - 38 - im.height) // 2))
    dr.text((ox + 6, oy + 10), cap, fill=(180, 0, 0))
op = os.path.join(d, "_sheet_key3.png")
sheet.save(op)
print(op, sheet.size, os.path.getsize(op))
