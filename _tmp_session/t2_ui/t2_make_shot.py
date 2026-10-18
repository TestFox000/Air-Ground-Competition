import re, os, json
from PIL import Image, ImageDraw, ImageFont

TMP = os.path.dirname(os.path.abspath(__file__))   # 证据自包含：与脚本同目录的 SVG/PNG
S = 6                      # 放大倍数
CROP = (96, 96, 244, 210)  # 取板周围 D4..G6 区域（真实屏幕坐标）
W = (CROP[2]-CROP[0])*S
H = (CROP[3]-CROP[1])*S

def load(name):
    return open(os.path.join(TMP, name), encoding="utf-8").read()

def raster(svg, fill_bg=(255,255,255)):
    img = Image.new("RGB", (W, H), fill_bg)
    d = ImageDraw.Draw(img)
    try:
        font = ImageFont.truetype("msyh.ttc", 11*S//2)
    except Exception:
        font = ImageFont.load_default()
    def T(x, y):
        return ((x-CROP[0])*S, (y-CROP[1])*S)
    # 背景
    for m in re.finditer(r'<rect x="([-\d.]+)" y="([-\d.]+)" width="([-\d.]+)" height="([-\d.]+)" fill="([^"]+)"([^>]*)/>', svg):
        x,y,w,h = map(float, m.group(1,2,3,4)); fill = m.group(5); rest = m.group(6)
        if fill in ("none",): continue
        if x+w < CROP[0] or x > CROP[2] or y+h < CROP[1] or y > CROP[3]: continue
        stroke = (201,206,216); width = 1
        ms = re.search(r'stroke="(#\w+)"', rest)
        if ms: stroke = ms.group(1)
        mw = re.search(r'stroke-width="([\d.]+)"', rest)
        if mw: width = max(1, int(float(mw.group(1)))*2)
        d.rectangle([T(x,y), T(x+w,y+h)], fill=fill, outline=stroke, width=width)
    for m in re.finditer(r'<line x1="([-\d.]+)" y1="([-\d.]+)" x2="([-\d.]+)" y2="([-\d.]+)"([^>]*)/>', svg):
        x1,y1,x2,y2 = map(float, m.group(1,2,3,4)); rest = m.group(5)
        ms = re.search(r'stroke="(#\w+)"', rest); col = ms.group(1) if ms else "#000000"
        d.line([T(x1,y1), T(x2,y2)], fill=col, width=max(2, S//2))
    for m in re.finditer(r'<polyline points="([^"]+)"([^>]*)>', svg):
        pts = [tuple(map(float, p.split(","))) for p in m.group(1).split()]
        if len(pts) > 1: d.line([T(*p) for p in pts], fill="#2f7fd0", width=2)
    for m in re.finditer(r'<circle cx="([-\d.]+)" cy="([-\d.]+)" r="([-\d.]+)" fill="([^"]+)"([^>]*)/>', svg):
        cx,cy,r,fill = float(m.group(1)), float(m.group(2)), float(m.group(3)), m.group(4)
        if cx+r < CROP[0] or cx-r > CROP[2] or cy+r < CROP[1] or cy-r > CROP[3]: continue
        d.ellipse([T(cx-r,cy-r), T(cx+r,cy+r)], fill=fill if fill != "none" else None)
    for m in re.finditer(r'<text x="([-\d.]+)" y="([-\d.]+)" font-size="([\d.]+)" fill="([^"]+)"([^>]*)>([^<]*)</text>', svg):
        x,y,fs,fill = float(m.group(1)), float(m.group(2)), float(m.group(3)), m.group(4)
        body = m.group(6)
        if x < CROP[0]-40 or x > CROP[2]+40 or y < CROP[1]-20 or y > CROP[3]+20: continue
        anchor = re.search(r'text-anchor="(\w+)"', m.group(5))
        txt_w = len(body)*fs*0.95
        tx = x - txt_w/2 if anchor and anchor.group(1) == "middle" else x
        d.text(T(tx, y-fs), body, fill=fill, font=font)
    return img

h_svg = load("sim_map_target_t2_horizontal.svg")
v_svg = load("sim_map_target_t2_vertical.svg")

# 旧 t14 版式（按旧代码语义重绘：两块 34×34 逐格方块，E5=蓝 / F5=红）
old_svg = re.sub(r'<rect x="136" y="136" width="68" height="17" fill="#2f7fd0" pointer-events="none"/>'
                 r'<rect x="136" y="153" width="68" height="17" fill="#d64545" pointer-events="none"/>'
                 r'<line[^/]*y1="153"[^/]*/>',
                 '<rect x="136" y="136" width="34" height="34" fill="#2f7fd0" pointer-events="none"/>'
                 '<rect x="170" y="136" width="34" height="34" fill="#d64545" pointer-events="none"/>', h_svg)

panels = [("新 t2：沿长边对半（E5+F5 横板 · 上蓝下红）", raster(h_svg)),
          ("新 t2：旋转成竖板（E5+E6 · 左蓝右红，不翻面）", raster(v_svg)),
          ("旧 t14（对照）：逐格两块 34×34 · E5 蓝 / F5 红", raster(old_svg))]

PAD, TITLE = 12, 26
width = sum(p.width for _, p in panels) + PAD*(len(panels)+1)
height = max(p.height for _, p in panels) + TITLE + PAD*2
out = Image.new("RGB", (width, height), (245,247,250))
d = ImageDraw.Draw(out)
try: tf = ImageFont.truetype("msyh.ttc", 16)
except Exception: tf = ImageFont.load_default()
x = PAD
for title, im in panels:
    d.text((x, 6), title, fill=(20,30,45), font=tf)
    out.paste(im, (x, TITLE))
    d.rectangle([x-1, TITLE-1, x+im.width, TITLE+im.height], outline=(180,188,198))
    x += im.width + PAD
p = os.path.join(TMP, "t2_target_board_compare.png")
out.save(p)
print(p, out.size)