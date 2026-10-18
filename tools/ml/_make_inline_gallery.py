# -*- coding: utf-8 -*-
"""生成 inline 展示用的紧凑标注图：frame_00170 全场概览 + 左上高密特写。
输出两段 base64 到 runs/cell_recog_render/_inline_parts.json（供 PureShowWidget 组装）。"""
import base64
import io
import json
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont
from ultralytics import YOLO

ROOT = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground")
sys.path.insert(0, str(ROOT / "reference" / "onboard" / "dzuav" / "src"))
from cell_classifier.predictor import CellIdRecognizer

FRAME = ROOT / "capture/datasets/aerial_field_mixed/images/val/frame_00170.jpg"
V2 = ROOT / "runs/detect/runs/detect/aerial_field_finetuned_v2/weights/best.pt"
V4C = ROOT / "runs/cell_classifier_v4c/best.pt"
OUT = ROOT / "runs/cell_recog_render/_inline_parts.json"

yolo = YOLO(str(V2))
rec = CellIdRecognizer(str(V4C), device="cpu", half=False, rotation_tta=True,
                       confidence_threshold=0.45)
font = ImageFont.truetype(r"C:\Windows\Fonts\msyhbd.ttc", 15) if Path(r"C:\Windows\Fonts\msyhbd.ttc").is_file() else ImageFont.load_default()

img = Image.open(FRAME).convert("RGB")
W, H = img.size
r = yolo.predict(str(FRAME), conf=0.25, iou=0.45, classes=[1], verbose=False, device="cpu")[0]
boxes = [(b.xyxy[0].tolist(), float(b.conf[0])) for b in r.boxes] if r.boxes is not None else []
print(f"cell_label 检出 {len(boxes)}")

def square_crop(im, x1, y1, x2, y2):
    side = max(x2 - x1, y2 - y1) * 1.15
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    sx1, sy1 = max(0, int(cx - side / 2)), max(0, int(cy - side / 2))
    sx2, sy2 = min(im.width, int(cx + side / 2)), min(im.height, int(cy + side / 2))
    crop = im.crop((sx1, sy1, sx2, sy2))
    s = max(crop.width, crop.height)
    canvas = Image.new("RGB", (s, s), (232, 232, 232))
    canvas.paste(crop, ((s - crop.width) // 2, (s - crop.height) // 2))
    return canvas

crops = [square_crop(img, *xyxy) for xyxy, _ in boxes]
preds = rec.predict(crops) if crops else []

# --- 图1：全场概览（缩 900 宽，只画框：绿=识别成功 红=低置信弃用） ---
scale = 900 / W
overview = img.resize((900, int(H * scale)))
d = ImageDraw.Draw(overview)
n_ok = 0
for (xyxy, _), pred in zip(boxes, preds):
    x1, y1, x2, y2 = [v * scale for v in xyxy]
    ok = pred is not None
    n_ok += ok
    d.rectangle([x1, y1, x2, y2], outline=(30, 180, 60) if ok else (220, 50, 50), width=2)

# --- 图2：左上高密特写（crop 后缩 1200 宽，框+格号文字） ---
CX1, CY1, CX2, CY2 = 0, 0, 1500, 1200
crop_img = img.crop((CX1, CY1, CX2, CY2))
crop_img = crop_img.resize((1200, int(1200 * (CY2 - CY1) / (CX2 - CX1))))
ds = 1200 / (CX2 - CX1)
d2 = ImageDraw.Draw(crop_img)
for (xyxy, _), pred in zip(boxes, preds):
    x1, y1, x2, y2 = xyxy
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    if not (CX1 <= cx <= CX2 and CY1 <= cy <= CY2):
        continue
    ok = pred is not None
    color = (30, 180, 60) if ok else (220, 50, 50)
    x1s, y1s, x2s, y2s = (x1 - CX1) * ds, (y1 - CY1) * ds, (x2 - CX1) * ds, (y2 - CY1) * ds
    d2.rectangle([x1s, y1s, x2s, y2s], outline=color, width=2)
    label = f"{pred.cell_id} {pred.joint_confidence:.2f}" if ok else "??"
    bb = d2.textbbox((x1s, y1s - 16), label, font=font)
    d2.rectangle([bb[0] - 2, bb[1] - 2, bb[2] + 2, bb[3] + 2], fill=color)
    d2.text((x1s, y1s - 16), label, fill=(255, 255, 255), font=font)

def b64(im, q=55):
    buf = io.BytesIO()
    im.save(buf, format="JPEG", quality=q)
    return base64.b64encode(buf.getvalue()).decode()

ov, cp = b64(overview, 45), b64(crop_img, 45)
OUT_DIR = ROOT / "runs/cell_recog_render"
(OUT_DIR / "_widget_overview.html").write_text(
    f'<div data-dynamic-ui-widget data-template="explanation-panel" data-mounted="true" '
    f'style="text-align:center"><img style="max-width:100%;border-radius:8px" '
    f'alt="frame_00170 全场格号识别概览" src="data:image/jpeg;base64,{ov}"></div>', encoding="utf-8")
(OUT_DIR / "_widget_crop.html").write_text(
    f'<div data-dynamic-ui-widget data-template="explanation-panel" data-mounted="true" '
    f'style="text-align:center"><img style="max-width:100%;border-radius:8px" '
    f'alt="frame_00170 左上区域格号识别特写" src="data:image/jpeg;base64,{cp}"></div>', encoding="utf-8")
print(f"检出={len(boxes)} 识别成功={n_ok}")
print("overview widget:", round(len(ov) / 1024), "KB  crop widget:", round(len(cp) / 1024), "KB")
