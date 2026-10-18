import base64, json, os
from PIL import Image
import io

review_dir = r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\capture\mine_official\review"
labels_dir = r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\capture\mine_official\labels"

items = []
files = sorted([f for f in os.listdir(review_dir) if f.endswith(".jpg")])
for fn in files:
    stem = fn[:-4]
    img = Image.open(os.path.join(review_dir, fn))
    img.thumbnail((96, 96))  # smaller
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=40)
    b64 = base64.b64encode(buf.getvalue()).decode()
    label_path = os.path.join(labels_dir, stem + ".txt")
    nbox = 0
    if os.path.exists(label_path):
        with open(label_path) as f:
            nbox = sum(1 for line in f if line.strip())
    items.append({"name": stem, "b64": b64, "nbox": nbox})

print(f"total b64 size: {sum(len(x['b64']) for x in items)} chars")
print(f"avg per image: {sum(len(x['b64']) for x in items)//len(items)} chars")

# Build widget
data_js = "[" + ",".join(
    f'{{n:"{it["name"]}",b:"{it["b64"]}",c:{it["nbox"]}}}' for it in items
) + "]"

html = f'''<style>
:root{{--text:#1f2328;--muted:#6e7781;--border:#d0d7de;--surface:#f6f8fa;--brand:#1a7f37;--smuted:#eaeef2}}
:root[data-widget-theme="dark"]{{--text:#e6edf3;--muted:#8b949e;--border:#30363d;--surface:#0d1117;--brand:#3fb950;--smuted:#161b22}}
.w{{font-family:system-ui;color:var(--text);padding:14px;background:var(--surface);border-radius:12px;width:100%;box-sizing:border-box}}
.h{{font-size:14px;font-weight:600;margin-bottom:2px}}
.sub{{font-size:12px;color:var(--muted);margin-bottom:10px}}
.ctrl{{display:flex;gap:8px;align-items:center;margin-bottom:10px;flex-wrap:wrap}}
.btn{{font-size:12px;padding:5px 12px;border:1px solid var(--border);border-radius:6px;background:var(--smuted);color:var(--text);cursor:pointer}}
.btn:disabled{{opacity:.4;cursor:default}}
.info{{font-size:12px;color:var(--muted)}}
.grid{{display:grid;grid-template-columns:repeat(4,1fr);gap:8px}}
.cell{{background:var(--smuted);border:1px solid var(--border);border-radius:8px;padding:6px;text-align:center}}
.cap{{font-size:11px;font-weight:600;margin-bottom:4px}}
.meta{{font-size:10px;color:var(--muted);margin-top:3px}}
img{{width:100%;height:auto;display:block;border-radius:4px}}
</style>
<div class="w" data-dynamic-ui-widget data-template="paginated-gallery">
<div class="h">官方爆炸物标注目检（绿框 = 你标的 mine）— 全部 157 张</div>
<div class="sub">确认标注框位置是否准确，绿框为 X-AnyLabeling 真值标注</div>
<div class="ctrl">
<button class="btn" id="prev">上一页</button>
<span class="info" id="pageInfo">1 / 1</span>
<button class="btn" id="next">下一页</button>
<span class="info" id="statInfo"></span>
</div>
<div class="grid" id="grid"></div>
</div>
<script>
const DATA = {data_js};
const PER = 12;
let page = 0;
const pages = Math.ceil(DATA.length / PER);
const grid = document.getElementById("grid");
const pageInfo = document.getElementById("pageInfo");
const statInfo = document.getElementById("statInfo");
const prev = document.getElementById("prev");
const next = document.getElementById("next");
function render(){{
  const start = page*PER;
  const slice = DATA.slice(start, start+PER);
  grid.innerHTML = slice.map(it => 
    `<div class="cell"><div class="cap">${{it.n}}</div><img src="data:image/jpeg;base64,${{it.b}}"><div class="meta">${{it.c}} 框</div></div>`
  ).join("");
  pageInfo.textContent = (page+1) + " / " + pages + " （" + (start+1) + "-" + Math.min(start+PER, DATA.length) + "）";
  const withBox = DATA.filter(d=>d.c>0).length;
  statInfo.textContent = `共 ${{DATA.length}} 张，其中 ${{withBox}} 张有标注`;
  prev.disabled = page===0;
  next.disabled = page>=pages-1;
}}
prev.addEventListener("click",()=>{{if(page>0){{page--;render()}}}});
next.addEventListener("click",()=>{{if(page<pages-1){{page++;render()}}}});
render();
</script>'''

# 与 _gen_widget_all.py 同款：拼好的 html 要落盘，否则只 print 没法用
out = r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\capture\mine_official\_widget_all2.html"
with open(out, "w", encoding="utf-8") as f:
    f.write(html)
print(f"widget saved: {out}, size={len(html)} chars")
