import json

d = json.load(open("capture/mine_official/_b64_small.json"))
parts = ['<style>:root{--text:#1f2328;--text-muted:#6e7781;--border:#d0d7de;--surface:#f6f8fa;--brand:#1a7f37;--surface-muted:#eaeef2}:root[data-widget-theme="dark"]{--text:#e6edf3;--text-muted:#8b949e;--border:#30363d;--surface:#0d1117;--brand:#3fb950;--surface-muted:#161b22}.w{font-family:system-ui;color:var(--text);padding:16px;background:var(--surface);border-radius:12px;width:100%;box-sizing:border-box}.h{font-size:14px;font-weight:600;margin-bottom:4px}.sub{font-size:12px;color:var(--text-muted);margin-bottom:12px}.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}.cell{background:var(--surface-muted);border:1px solid var(--border);border-radius:8px;padding:8px;text-align:center}.cap{font-size:12px;font-weight:600;margin-bottom:6px}.meta{font-size:11px;color:var(--text-muted);margin-top:4px}img{width:100%;height:auto;display:block;border-radius:4px}</style>']
parts.append('<div class="w" data-dynamic-ui-widget data-template="compact-table-visual"><div class="h">官方爆炸物标注 — 真实图片目检（绿框 = 你标的 mine）</div><div class="sub">绿框为 X-AnyLabeling 标注框，确认框位是否准确</div><div class="grid">')
for n, (b64, cnt) in d.items():
    parts.append('<div class="cell"><div class="cap">' + n + '</div><img src="data:image/jpeg;base64,' + b64 + '"/><div class="meta">' + str(cnt) + ' 框</div></div>')
parts.append('</div></div>')
html = "".join(parts)
open("capture/mine_official/_widget_small.html", "w", encoding="utf-8").write(html)
print(len(html))
