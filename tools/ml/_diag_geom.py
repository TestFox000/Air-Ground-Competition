"""数值诊断官方图预测框 vs GT框几何关系: 中心距 / IoU / 大小比。"""
from __future__ import annotations

from pathlib import Path

from ultralytics import YOLO

ROOT = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground")
IMG = ROOT / "capture/mine_official/images"
LBL = ROOT / "capture/mine_official/labels"


def iou(a, b):
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
    ua = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
    return inter / ua if ua > 0 else 0.0


model = YOLO(str(ROOT / "runs/detect/runs/detect/aerial_field_finetuned_b2/weights/best.pt"))
gt_files = [lp for lp in sorted(LBL.glob("*.txt"))
            if any(l.strip() for l in lp.open(encoding="utf-8"))]

stats = []
for lp in gt_files:
    gts = []
    for line in lp.open(encoding="utf-8"):
        p = line.split()
        if p and p[0] == "0":
            cx, cy, w, h = map(float, p[1:5])
            gts.append((cx-w/2, cy-h/2, cx+w/2, cy+h/2))
    if not gts:
        continue
    r = model.predict(str(IMG / (lp.stem + ".jpg")), conf=0.05, imgsz=1280,
                      verbose=False, device=0)[0]
    preds = []
    for box in r.boxes:
        if int(box.cls) == 0:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            preds.append((x1/r.orig_shape[1], y1/r.orig_shape[0],
                          x2/r.orig_shape[1], y2/r.orig_shape[0], float(box.conf)))
    if not preds:
        continue
    # 每图取 conf 最高的预测, 与每个GT算几何
    best = max(preds, key=lambda p: p[4])
    best_iou = max(iou(best[:4], g) for g in gts)
    # 中心距(相对图尺寸)
    pcx, pcy = (best[0]+best[2])/2, (best[1]+best[3])/2
    dmin = min(((pcx-(g[0]+g[2])/2)**2 + (pcy-(g[1]+g[3])/2)**2) ** 0.5 for g in gts)
    # 大小比(面积)
    parea = (best[2]-best[0])*(best[3]-best[1])
    garea = max((g[2]-g[0])*(g[3]-g[1]) for g in gts)
    stats.append((lp.stem, best[4], best_iou, dmin, parea/garea if garea > 0 else 0))

stats.sort(key=lambda s: -s[1])
print(f"{'图':<16}{'conf':>6}{'IoU':>7}{'中心距':>8}{'面积比':>8}")
for s, c, i, d, ar in stats[:25]:
    print(f"{s:<16}{c:>6.3f}{i:>7.3f}{d:>8.3f}{ar:>8.2f}")
print("...")
for s, c, i, d, ar in stats[50:60]:
    print(f"{s:<16}{c:>6.3f}{i:>7.3f}{d:>8.3f}{ar:>8.2f}")

import statistics as st
print(f"\nIoU中位: {st.median(s[2] for s in stats):.3f}")
print(f"中心距中位: {st.median(s[3] for s in stats):.3f}")
print(f"面积比中位: {st.median(s[4] for s in stats):.2f}")
print(f"中心距<0.1 的图: {sum(1 for s in stats if s[3] < 0.1)}/{len(stats)}")
print(f"面积比>3 或 <0.3 的图: {sum(1 for s in stats if s[4] > 3 or s[4] < 0.3)}/{len(stats)}")
