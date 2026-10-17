import sys, io, os
sys.path.insert(0, r"D:\User\Desktop\dzsmartscout\.pdfdeps")
from pypdf import PdfReader

src = r"D:\User\Desktop\Date\附加资源\场地布置说明书V2-空地协同侦排挑战赛.pdf"
outdir = r"D:\User\Desktop\dzsmartscout\.pdfdeps\txt"
os.makedirs(outdir, exist_ok=True)

r = PdfReader(src)
parts = []
for i, p in enumerate(r.pages):
    t = p.extract_text() or ""
    try:
        imgs = len(p.images)
    except Exception as e:
        imgs = "err:" + type(e).__name__
    parts.append("===== PAGE %d (imgs=%s) =====\n%s\n" % (i + 1, imgs, t))

full = "\n".join(parts)
op = os.path.join(outdir, "arena_v2.txt")
with io.open(op, "w", encoding="utf-8") as fh:
    fh.write(full)
print("pages=", len(r.pages), "chars=", len(full), "->", op)
