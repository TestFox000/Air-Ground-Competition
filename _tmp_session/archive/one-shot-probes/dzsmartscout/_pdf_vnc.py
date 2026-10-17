import pdfplumber, re, os
p = r"D:\User\Desktop\Date\附加资源\VNC远程桌面使用方法.pdf"
out = r"D:\User\Desktop\Date\Save\dzsmartscout\_pdf_text\VNC远程桌面使用方法.txt"
with pdfplumber.open(p) as pdf:
    txt = []
    for i, page in enumerate(pdf.pages):
        txt.append(f"\n----- PAGE {i+1} -----\n" + (page.extract_text() or ""))
text = "\n".join(txt)
with open(out, "w", encoding="utf-8") as f:
    f.write(text)
# print IP-like and network keywords
for line in text.splitlines():
    if re.search(r"\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}|IP|地址|端口|VNC|ssh|用户名|密码|网络|WiFi|热点|192|10\.|172\.", line, re.I):
        print(line)
print("=== written", out, "pages=", len(pdf.pages) if False else "?")
