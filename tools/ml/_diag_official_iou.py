"""验证官方图低置信检出框的位置正确性: 与GT框IoU匹配。

若 conf=0.05 的检出框与GT框 IoU>=0.3 匹配率高 => 纯置信度校准问题(降阈值即可)。
"""
from __future__ import annotations

from pathlib import Path

from ultralytics import YOLO

ROOT = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground")
IMG = ROOT / "capture/mine_official/images"
LBL = ROOT / "capture/mine_official/labels"


def iou(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
    ua = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - inter
    return inter / ua if ua > 0 else 0.0


model = YOLO(str(ROOT / "runs/detect/runs/detect/aerial_field_finetuned_b2/weights/best.pt"))

# 检查训练集官方图: 模型是否至少拟合了训练分布
import sys
mode = sys.argv[1] if len(sys.argv) > 1 else "official"
if mode == "train":
    IMG2 = ROOT / "capture/datasets/aerial_field_mixed/images/train"
    LBL2 = ROOT / "capture/datasets/aerial_field_mixed/labels/train"
    gt_files = [lp for lp in sorted(LBL2.glob("explosive_*.txt"))
                if any(l.split() and l.split()[0] == "0" for l in lp.open(encoding="utf-8"))]
    img_dir = IMG2
else:
    gt_files = [lp for lp in sorted(LBL.glob("*.txt"))
                if any(l.strip() for l in lp.open(encoding="utf-8"))]
    img_dir = IMG

matched = half = missed = 0
for lp in gt_files:
    gts = []
    for line in lp.open(encoding="utf-8"):
        p = line.split()
        if p and p[0] == "0":
            cx, cy, w, h = map(float, p[1:5])
            gts.append((cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2))
    if not gts:
        continue
    jp = img_dir / (lp.stem + ".jpg")
    if not jp.is_file():
        continue
    r = model.predict(str(jp), conf=0.05, imgsz=1280,
                      verbose=False, device=0)[0]
    preds = []
    for box in r.boxes:
        if int(box.cls) == 0:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            preds.append((x1 / r.orig_shape[1], y1 / r.orig_shape[0],
                          x2 / r.orig_shape[1], y2 / r.orig_shape[0]))
    if any(max(iou(p, g) for g in gts) >= 0.3 for p in preds):
        matched += 1
    elif preds:
        half += 1  # 有检出但框不匹配
    else:
        missed += 1

n = len(gt_files)
print(f"GT图 {n} 张 (conf=0.05, IoU>=0.3):")
print(f"  框位置匹配: {matched} ({matched/n:.1%})")
print(f"  有检出但位置偏: {half}")
print(f"  完全无检出: {missed}")
