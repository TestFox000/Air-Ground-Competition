import base64, os, math
from PIL import Image
import io

review_dir = r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\capture\mine_official\review"
labels_dir = r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\capture\mine_official\labels"
out_dir = r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\capture\mine_official"

items = []
files = sorted([f for f in os.listdir(review_dir) if f.endswith(".jpg")])
for fn in files:
    stem = fn[:-4]
    label_path = os.path.join(labels_dir, stem + ".txt")
    nbox = 0
    if os.path.exists(label_path):
        with open(label_path) as f:
            nbox = sum(1 for line in f if line.strip())
    if nbox == 0:
        continue
    img = Image.open(os.path.join(review_dir, fn))
    img.thumbnail((120, 120))
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=50)
    b64 = base64.b64encode(buf.getvalue()).decode()
    items.append({"name": stem, "b64": b64, "nbox": nbox})

PER = 15
n_pages = math.ceil(len(items) / PER)

def make_widget(page_items, page_idx, total_pages):
    cells = "".join(
        f'<div class="cell"><div class="cap">{it["name"]}</div>'
        f'<img src="data:image/jpeg;base64,{it["b64"]}">'
        f'<div class="meta">{it["nbox"]} 框</div></div>'
        for it in page_items
    )
    total_box = sum(it["nbox"] for it in items)
    return f'''<style>
:root{{--text:#1f2328;--muted:#6e7781;--border:#d0d7de;--surface:#f6f8fa;--smuted:#eaeef2}}
:root[data-widget-theme="dark"]{{--text:#e6edf3;--muted:#8b949e;--border:#30363d;--surface:#0d1117;--smuted:#161b22}}
.w{{font-family:system-ui;color:var(--text);padding:14px;background:var(--surface);border-radius:12px;width:100%;box-sizing:border-box}}
.h{{font-size:14px;font-weight:600;margin-bottom:2px}}
.sub{{font-size:12px;color:var(--muted);margin-bottom:10px}}
.grid{{display:grid;grid-template-columns:repeat(5,1fr);gap:8px}}
.cell{{background:var(--smuted);border:1px solid var(--border);border-radius:8px;padding:6px;text-align:center}}
.cap{{font-size:11px;font-weight:600;margin-bottom:4px}}
.meta{{font-size:10px;color:var(--muted);margin-top:3px}}
img{{width:100%;height:auto;display:block;border-radius:4px}}
</style>
<div class="w" data-dynamic-ui-widget data-template="gallery">
<div class="h">官方爆炸物标注目检 — 第 {page_idx+1}/{total_pages} 批（共 {len(items)} 张有标注，{total_box} 框）</div>
<div class="sub">绿框 = X-AnyLabeling 真值标注，确认框位是否准确</div>
<div class="grid">{cells}</div>
</div>'''

for i in range(n_pages):
    slice_items = items[i*PER:(i+1)*PER]
    html = make_widget(slice_items, i, n_pages)
    out_path = os.path.join(out_dir, f"_widget_batch_{i+1}.html")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"batch {i+1}: {len(slice_items)} images, {len(html)} chars -> {out_path}")
