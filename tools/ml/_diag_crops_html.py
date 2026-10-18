"""生成裁剪 ROI 诊断网页：显示裁剪图 + 识别结果 + 各置信度。"""
from __future__ import annotations

import base64
import io
import sys
from pathlib import Path

from PIL import Image
from ultralytics import YOLO

ROOT = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground")
YOLO_WEIGHTS = ROOT / r"runs\detect\runs\detect\aerial_field\weights\best.pt"
CELL_WEIGHTS = ROOT / r"runs\cell_classifier_v3\best.pt"
VAL_DIR = ROOT / r"capture\datasets\aerial_field_mixed\images\val"
OUT = ROOT / r"runs\cell_recog_render"

sys.path.insert(0, str(ROOT / "reference" / "onboard" / "dzuav" / "src"))
from cell_classifier.predictor import CellIdRecognizer  # noqa: E402


def square_crop(img, x1, y1, x2, y2, pad=1.15):
    w = x2 - x1
    h = y2 - y1
    side = max(w, h) * pad
    cx = (x1 + x2) / 2
    cy = (y1 + y2) / 2
    sx1 = max(0, int(cx - side / 2))
    sy1 = max(0, int(cy - side / 2))
    sx2 = min(img.width, int(cx + side / 2))
    sy2 = min(img.height, int(cy + side / 2))
    return img.crop((sx1, sy1, sx2, sy2))


def img2b64(img, size=96):
    img = img.resize((size, size), Image.BICUBIC)
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    return base64.b64encode(buf.getvalue()).decode()


yolo = YOLO(str(YOLO_WEIGHTS))
rec = CellIdRecognizer(str(CELL_WEIGHTS), device="cuda:0", half=True,
                       rotation_tta=True, confidence_threshold=0.0)

# 处理 4 张代表性帧
frames = ["frame_00016.jpg", "frame_00103.jpg", "frame_00168.jpg", "frame_00255.jpg"]
html = ['<!DOCTYPE html><html><head><meta charset="utf-8"><title>格号裁剪诊断</title>',
        '<style>body{font-family:system-ui;background:#1a1a1a;color:#ddd;margin:20px}',
        '.row{display:flex;flex-wrap:wrap;gap:14px;margin-bottom:24px}',
        '.cell{background:#262626;border-radius:6px;padding:8px;text-align:center;width:140px}',
        '.cell img{width:96px;height:96px;image-rendering:pixelated;border:1px solid #444}',
        '.ok{color:#1eb43c}.bad{color:#dc3232}',
        'h2{font-size:16px;margin:16px 0 8px}',
        '.small{font-size:11px;color:#999}</style></head><body>']

for fname in frames:
    img_path = VAL_DIR / fname
    if not img_path.is_file():
        continue
    img = Image.open(img_path).convert("RGB")
    results = yolo.predict(source=str(img_path), conf=0.25, iou=0.45,
                           classes=[1], verbose=False, device=0)
    boxes = results[0].boxes
    crops = []
    for box in boxes[:20]:
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        crops.append(square_crop(img, x1, y1, x2, y2, pad=1.15))
    preds = rec.predict(crops)

    html.append(f'<h2>{fname}（{len(crops)} 个格号）</h2><div class="row">')
    for i, (crop, p) in enumerate(zip(crops, preds)):
        b64 = img2b64(crop)
        if p is None:
            label = '<span class="bad">None</span>'
            meta = ""
        else:
            ok = p.joint_confidence >= 0.45
            cls = "ok" if ok else "bad"
            label = f'<span class="{cls}">{p.cell_id}</span>'
            meta = (f'<div class="small">L:{p.column_confidence:.2f} '
                    f'R:{p.row_confidence:.2f} J:{p.joint_confidence:.2f}</div>')
        html.append(f'<div class="cell"><img src="data:image/png;base64,{b64}">'
                    f'<div>{label}</div>{meta}</div>')
    html.append('</div>')

html.append('</body></html>')
(OUT / "diag_crops.html").write_text("\n".join(html), encoding="utf-8")
print(f"诊断网页：{OUT / 'diag_crops.html'}")
