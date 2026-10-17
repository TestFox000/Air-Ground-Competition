import sys
sys.path.insert(0, r"D:\User\Desktop\dzsmartscout\.pdfdeps")
from pypdf import PdfReader
import os
files = [
    r"D:\User\Desktop\Date\附加资源\1-无人机操作指南\SmartScout-AR5_高级功能用户手册202609.pdf",
    r"D:\User\Desktop\Date\附加资源\1-无人机操作指南\SmartScout-AR250_快速使用与安全指南v3(1).pdf",
]
for f in files:
    r = PdfReader(f)
    print("="*80)
    print("FILE:", os.path.basename(f))
    print("PAGES:", len(r.pages))
    for i, p in enumerate(r.pages):
        t = p.extract_text() or ""
        t = t.strip()
        print(f"--- PAGE {i+1} ({len(t)} chars) ---")
        print(t[:3000])
