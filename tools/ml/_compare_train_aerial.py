"""对比：训练样本的格号图 vs 航拍裁剪图。"""
from __future__ import annotations

import base64
import io
from pathlib import Path
from PIL import Image

ROOT = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground")
TRAIN_CROPS = ROOT / r"datasets\cell_labels\crops"
AERIAL_DIAG = ROOT / r"runs\cell_recog_render\diag"
OUT = ROOT / r"runs\cell_recog_render"

def img2b64(path, size=96):
    with Image.open(path) as img:
        img = img.convert("RGB").resize((size, size), Image.BICUBIC)
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    return base64.b64encode(buf.getvalue()).decode()

def img2b64_pil(img, size=96):
    img = img.convert("RGB").resize((size, size), Image.BICUBIC)
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    return base64.b64encode(buf.getvalue()).decode()

html = ['<!DOCTYPE html><html><head><meta charset="utf-8"><title>训练样本 vs 航拍裁剪</title>',
        '<style>body{font-family:system-ui;background:#1a1a1a;color:#ddd;margin:20px}',
        'h2{font-size:16px;margin:16px 0 8px}',
        '.row{display:flex;flex-wrap:wrap;gap:14px;margin-bottom:24px}',
        '.cell{background:#262626;border-radius:6px;padding:8px;text-align:center;width:120px}',
        '.cell img{width:96px;height:96px;image-rendering:pixelated;border:1px solid #444}',
        '.small{font-size:11px;color:#999}</style></head><body>']

# 训练样本：取 20 张
html.append('<h2>训练样本（datasets/cell_labels/crops/）</h2><div class="row">')
samples = sorted(TRAIN_CROPS.glob("*.png"))[:24]
for p in samples:
    # 看原始尺寸
    with Image.open(p) as im:
        w, h = im.size
    b64 = img2b64(p)
    html.append(f'<div class="cell"><img src="data:image/png;base64,{b64}">'
                f'<div class="small">{p.stem}</div>'
                f'<div class="small">{w}x{h}</div></div>')
html.append('</div>')

# 航拍裁剪图
html.append('<h2>航拍裁剪图（YOLO 检出后裁剪）</h2><div class="row">')
aerial = sorted(AERIAL_DIAG.glob("crop_*_raw.png"))[:24]
for p in aerial:
    with Image.open(p) as im:
        w, h = im.size
    b64 = img2b64(p)
    html.append(f'<div class="cell"><img src="data:image/png;base64,{b64}">'
                f'<div class="small">{p.stem}</div>'
                f'<div class="small">{w}x{h}</div></div>')
html.append('</div>')

html.append('</body></html>')
(OUT / "compare_train_vs_aerial.html").write_text("\n".join(html), encoding="utf-8")
print(f"对比网页：{OUT / 'compare_train_vs_aerial.html'}")
