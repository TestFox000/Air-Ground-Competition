"""Generate widget HTML from _sample_b64.json"""
import json, os

b64_path = r"E:\OneDrive - mails.qust.edu.cn\桌面\air_ground\tools\ml\_sample_b64.json"
with open(b64_path, encoding="utf-8") as f:
    samples = json.load(f)

style = """<style>
:root{--bg:#0f1117;--card:#1a1d27;--txt:#e4e6eb;--sub:#8b8fa3;--border:#2a2d3a}
body{margin:0;padding:16px;background:var(--bg);color:var(--txt);font-family:system-ui,sans-serif}
.wrap{max-width:980px;margin:0 auto}
h2{font-size:18px;margin:0 0 4px}
.sub{color:var(--sub);font-size:13px;margin-bottom:12px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}
.tile{background:var(--card);border:1px solid var(--border);border-radius:10px;overflow:hidden}
.tile img{width:100%;display:block}
.tile .cap{padding:8px 10px;font-size:12px;color:var(--sub);border-top:1px solid var(--border)}
.legend{margin-top:14px;display:flex;flex-wrap:wrap;gap:10px;font-size:12px}
.legend span{display:inline-flex;align-items:center;gap:5px}
.dot{width:12px;height:12px;border-radius:3px;display:inline-block}
</style>"""

items = ""
for title, b64 in samples:
    # break base64 into chunks so Read tool can parse lines
    chunks = [b64[i:i+900] for i in range(0, len(b64), 900)]
    b64_split = "\n".join(chunks)
    items += f'<div class="tile"><img src="data:image/jpeg;base64,\n{b64_split}\n"><div class="cap">{title}</div></div>'

legend = """<div class="legend">
<span><i class="dot" style="background:#0f0"></i>green target</span>
<span><i class="dot" style="background:#f00"></i>target</span>
<span><i class="dot" style="background:#ffa500"></i>red bucket</span>
<span><i class="dot" style="background:#064aff"></i>blue bucket</span>
<span><i class="dot" style="background:#00c8c8"></i>green bucket</span>
<span><i class="dot" style="background:#ff0"></i>typo (r'r / RR)</span>
</div>"""

html = f'<div data-dynamic-ui-widget="sample-tiles"><div class="wrap">{style}<h2>标靶标注样例图 — 类别映射确认</h2><div class="sub">Wave2 (manual_hq_5) 中 green target(160框) 及 typo(r\'r/RR) 需确认映射到哪个类别</div><div class="grid">{items}</div>{legend}</div></div>'

out = r"E:\OneDrive - mails.qust.edu.cn\桌面\air_ground\tools\ml\_widget_samples.html"
with open(out, "w", encoding="utf-8") as f:
    f.write(html)
print(f"Widget HTML saved: {len(html)} chars")
