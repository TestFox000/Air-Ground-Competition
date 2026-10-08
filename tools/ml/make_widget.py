"""把 lattice 渲染图打包成 inline widget HTML 片段。"""
import base64
import io
from pathlib import Path

from PIL import Image

ROOT = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground")
OUT = ROOT / "runs\lattice_ransac"
frames = [
    ("lattice_frame_00175.jpg", "frame_00175：覆盖 65.9%→78.0%（lattice 补全/纠正 10 格）"),
    ("lattice_frame_00168.jpg", "frame_00168：覆盖 85.5%→91.3%（69 格全场视角）"),
]
for fn, caption in frames:
    img = Image.open(OUT / fn)
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=46)
    b64 = base64.b64encode(buf.getvalue()).decode("ascii")
    html = (
        '<div data-dynamic-ui-widget data-template="explanation-panel" data-mounted="true" '
        'style="font-family:system-ui,sans-serif">'
        f'<div style="font-size:13px;color:#888;margin:4px 0">{caption}</div>'
        f'<img src="data:image/jpeg;base64,{b64}" '
        'style="max-width:100%;border-radius:8px;display:block">'
        '<div style="font-size:12px;color:#888;margin:6px 0 2px">'
        "绿=v4c确认 蓝◆=lattice补全 黄✚=lattice纠正 橙⚠=冲突保留v4c 灰=重复 红=剔除"
        "</div></div>"
    )
    out = OUT / ("_widget_" + fn.replace(".jpg", ".html"))
    out.write_text(html, encoding="utf-8")
    print(f"{out.name}: {len(html) // 1024} KB")
