import sys, io
sys.path.insert(0, r"D:\User\Desktop\dzsmartscout\.pdfdeps")
from pypdf import PdfReader
import os
files = [
    ("ar5", r"D:\User\Desktop\Date\附加资源\1-无人机操作指南\SmartScout-AR5_高级功能用户手册202609.pdf"),
    ("ar250", r"D:\User\Desktop\Date\附加资源\1-无人机操作指南\SmartScout-AR250_快速使用与安全指南v3(1).pdf"),
]
outdir = r"D:\User\Desktop\dzsmartscout\.pdfdeps\txt"
os.makedirs(outdir, exist_ok=True)
for name, f in files:
    r = PdfReader(f)
    parts = []
    for i, p in enumerate(r.pages):
        t = p.extract_text() or ""
        parts.append(f"===== PAGE {i+1} =====\n{t}\n")
    full = "\n".join(parts)
    op = os.path.join(outdir, name + ".txt")
    with io.open(op, "w", encoding="utf-8") as fh:
        fh.write(full)
    print(name, "pages=", len(r.pages), "chars=", len(full), "->", op)
