"""诊断：保存裁剪的格号 ROI + 打印置信度分布。"""
from __future__ import annotations

import sys
from pathlib import Path

from PIL import Image
from ultralytics import YOLO

ROOT = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground")
YOLO_WEIGHTS = ROOT / r"runs\detect\runs\detect\aerial_field\weights\best.pt"
CELL_WEIGHTS = ROOT / r"runs\cell_classifier_v3\best.pt"
VAL_DIR = ROOT / r"capture\datasets\aerial_field_mixed\images\val"
OUT = ROOT / r"runs\cell_recog_render\diag"
OUT.mkdir(parents=True, exist_ok=True)

sys.path.insert(0, str(ROOT / "reference" / "onboard" / "dzuav" / "src"))
from cell_classifier.predictor import CellIdRecognizer  # noqa: E402
from cell_classifier.constants import LETTERS, ROWS  # noqa: E402


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


yolo = YOLO(str(YOLO_WEIGHTS))
rec = CellIdRecognizer(str(CELL_WEIGHTS), device="cuda:0", half=True,
                       rotation_tta=True, confidence_threshold=0.0)  # 阈值设为0看所有结果

# 取第一张有较多格号的图
img_path = VAL_DIR / "frame_00016.jpg"
img = Image.open(img_path).convert("RGB")
results = yolo.predict(source=str(img_path), conf=0.25, iou=0.45,
                       classes=[1], verbose=False, device=0)
boxes = results[0].boxes

crops = []
for i, box in enumerate(boxes[:15]):
    x1, y1, x2, y2 = box.xyxy[0].tolist()
    crop = square_crop(img, x1, y1, x2, y2, pad=1.15)
    crop.save(OUT / f"crop_{i:02d}_raw.png")
    # 也保存 resize 到 96 的版本，看最终输入长什么样
    crop96 = crop.resize((96, 96), Image.BICUBIC)
    crop96.save(OUT / f"crop_{i:02d}_96.png")
    crops.append(crop)

preds = rec.predict(crops)
for i, p in enumerate(preds):
    if p is None:
        print(f"crop {i}: None")
    else:
        print(f"crop {i}: {p.cell_id}  col_conf={p.column_confidence:.3f}  row_conf={p.row_confidence:.3f}  joint={p.joint_confidence:.3f}")

print(f"\n裁剪图已保存到 {OUT}")
