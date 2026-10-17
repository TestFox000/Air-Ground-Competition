import pdfplumber, sys, os

paths = [
    r"D:\User\Desktop\Date\附加资源\1-无人机操作指南\SmartScout-AR5_高级功能用户手册202609.pdf",
    r"D:\User\Desktop\Date\附加资源\1-无人机操作指南\SmartScout-AR250_快速使用与安全指南v3(1).pdf",
]

out_dir = r"D:\User\Desktop\Date\Save\dzsmartscout\_pdf_text"
os.makedirs(out_dir, exist_ok=True)

for p in paths:
    name = os.path.splitext(os.path.basename(p))[0]
    with pdfplumber.open(p) as pdf:
        n = len(pdf.pages)
        print(f"== {name}  pages={n}")
        txt = []
        for i, page in enumerate(pdf.pages):
            t = page.extract_text() or ""
            txt.append(f"\n----- PAGE {i+1} -----\n" + t)
    with open(os.path.join(out_dir, name + ".txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(txt))
    print(f"written: {name}.txt")
