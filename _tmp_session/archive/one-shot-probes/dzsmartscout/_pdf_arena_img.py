import sys, io, os
sys.path.insert(0, r"D:\User\Desktop\dzsmartscout\.pdfdeps")
from pypdf import PdfReader
from PIL import Image

src = r"D:\User\Desktop\Date\附加资源\场地布置说明书V2-空地协同侦排挑战赛.pdf"
outdir = r"D:\User\Desktop\dzsmartscout\fc-audit\arena_pdf_img"
os.makedirs(outdir, exist_ok=True)

r = PdfReader(src)
rows = []
for i, p in enumerate(r.pages):
    for j, im in enumerate(p.images):
        try:
            img = im.image
        except Exception as e:
            print("ERR p%d #%d %s: %s" % (i + 1, j, im.name, e))
            continue
        w, h = img.size
        op = os.path.join(outdir, "p%02d_%d.png" % (i + 1, j))
        if img.mode not in ("RGB", "RGBA"):
            img = img.convert("RGB")
        img.save(op)
        rows.append((i + 1, j, w, h, op, im.name))

print("%-4s %-4s %-12s %-10s %s" % ("page", "#", "size", "bytes", "out"))
for pg, j, w, h, op, nm in rows:
    print("%-4d %-4d %-12s %-10d %s" % (pg, j, "%dx%d" % (w, h), os.path.getsize(op), os.path.basename(op)))
print("total images:", len(rows))
