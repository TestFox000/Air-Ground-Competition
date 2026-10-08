"""生成存疑+被剔除框的高分辨率复核大图(300px/格, 4列)。"""
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

OUT = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\runs\cell_recog_render\label")
CROPS = OUT / "crops_124"

SUSPECTS = [8, 11, 12, 13, 18, 22, 25, 30, 31, 35, 42, 44, 47, 52, 53, 54,
            55, 63, 64, 65, 72, 73, 75, 76, 78, 80, 81, 84, 85, 86, 87]

SHEET_CELL = 300
COLS = 4
font = ImageFont.truetype(r"C:\Windows\Fonts\arialbd.ttf", 40)

per_sheet = 16  # 4x4
n_sheets = (len(SUSPECTS) + per_sheet - 1) // per_sheet
for s in range(n_sheets):
    ids = SUSPECTS[s * per_sheet:(s + 1) * per_sheet]
    rows = (len(ids) + COLS - 1) // COLS
    sheet = Image.new("RGB", (COLS * SHEET_CELL, rows * (SHEET_CELL + 52)), (25, 25, 25))
    d = ImageDraw.Draw(sheet)
    for j, i in enumerate(ids):
        r, c = divmod(j, COLS)
        crop = Image.open(CROPS / f"crop_{i:02d}.png")
        crop.thumbnail((SHEET_CELL - 10, SHEET_CELL - 10), Image.LANCZOS)
        sheet.paste(crop, (c * SHEET_CELL + 5, r * (SHEET_CELL + 52) + 52))
        d.text((c * SHEET_CELL + 8, r * (SHEET_CELL + 52) + 4),
               f"#{i}", fill=(255, 255, 0), font=font)
    sheet.save(OUT / f"recheck_{s}.png")
    print(f"recheck_{s}.png: {ids}")
