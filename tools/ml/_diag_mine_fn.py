# -*- coding: utf-8 -*-
"""mine FN=38 漏检根因诊断
判断每个 mine FN 属于:
  A. 完全漏检 (附近无任何预测, IoU<0.1)
  B. 误分类 (附近有非mine预测, IoU>=0.1)
  C. 定位偏差 (附近有mine预测但IoU<0.5)
输出各类数量 + 误分类目标类别分布
"""
from pathlib import Path
import cv2

GT_DIR = Path(r"E:\ml_datasets\aerial_field_7cls\labels\val")
IMG_DIR = Path(r"E:\ml_datasets\aerial_field_7cls\images\val")
PRED_DIR = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\runs\detect\runs\detect\runs\detect\aerial_field_7cls_combined\pred_val\labels")
NAMES = ["mine", "car", "arrow", "target_red", "cell_label", "target_blue", "obstacle"]
MINE_CLS = 0

def load_labels(path, w, h):
    items = []
    if not path.exists():
        return items
    for line in path.read_text().strip().split("\n"):
        if not line.strip():
            continue
        p = line.split()
        cls = int(p[0])
        cx, cy, bw, bh = map(float, p[1:5])
        items.append((cls, cx * w, cy * h, bw * w, bh * h))
    return items

def iou(a, b):
    ax1, ay1 = a[0] - a[2] / 2, a[1] - a[3] / 2
    ax2, ay2 = a[0] + a[2] / 2, a[1] + a[3] / 2
    bx1, by1 = b[0] - b[2] / 2, b[1] - b[3] / 2
    bx2, by2 = b[0] + b[2] / 2, b[1] + b[3] / 2
    ix = max(0, min(ax2, bx2) - max(ax1, bx1))
    iy = max(0, min(ay2, by2) - max(ay1, by1))
    inter = ix * iy
    return inter / (a[2] * a[3] + b[2] * b[3] - inter + 1e-9)

catA = 0  # 完全漏检
catB = 0  # 误分类
catC = 0  # 定位偏差
miscls_dist = {}
fn_details = []

for img_f in sorted(IMG_DIR.glob("*.jpg")):
    img = cv2.imread(str(img_f))
    if img is None:
        continue
    h, w = img.shape[:2]
    stem = img_f.stem
    gt = load_labels(GT_DIR / f"{stem}.txt", w, h)
    pred = load_labels(PRED_DIR / f"{stem}.txt", w, h)
    mine_gt = [g for g in gt if g[0] == MINE_CLS]
    mine_pred = [p for p in pred if p[0] == MINE_CLS]
    other_pred = [p for p in pred if p[0] != MINE_CLS]

    matched = [False] * len(mine_pred)
    for g in mine_gt:
        # 找最佳 mine 预测
        best_iou, best_j = 0, -1
        for j, p in enumerate(mine_pred):
            if matched[j]:
                continue
            v = iou(g[1:], p[1:])
            if v > best_iou:
                best_iou, best_j = v, j
        if best_j >= 0 and best_iou >= 0.5:
            matched[best_j] = True
            continue  # TP
        # FN: 判断类别
        if best_iou >= 0.1:
            catC += 1  # 有mine预测但IoU不足
            fn_details.append((stem, "C", best_iou, NAMES[MINE_CLS]))
        else:
            # 检查附近是否有非mine预测
            best_other_iou, best_other_cls = 0, -1
            for p in other_pred:
                v = iou(g[1:], p[1:])
                if v > best_other_iou:
                    best_other_iou, best_other_cls = v, p[0]
            if best_other_iou >= 0.1:
                catB += 1
                miscls_dist[NAMES[best_other_cls]] = miscls_dist.get(NAMES[best_other_cls], 0) + 1
                fn_details.append((stem, "B", best_other_iou, NAMES[best_other_cls]))
            else:
                catA += 1
                fn_details.append((stem, "A", 0.0, ""))

print("=" * 60)
print("mine FN=38 漏检根因诊断")
print("=" * 60)
print(f"  A. 完全漏检 (附近无任何预测, IoU<0.1) : {catA}")
print(f"  B. 误分类 (附近有非mine预测, IoU>=0.1): {catB}")
print(f"  C. 定位偏差 (有mine预测但IoU<0.5)    : {catC}")
print(f"  合计 FN                              : {catA + catB + catC}")
print()
if miscls_dist:
    print("误分类目标类别分布:")
    for cls, cnt in sorted(miscls_dist.items(), key=lambda x: -x[1]):
        print(f"  {cls:12s}: {cnt}")
print()
print("FN 明细 (前20条):")
for stem, cat, iou_v, cls in fn_details[:20]:
    print(f"  {stem}  cat={cat}  iou={iou_v:.3f}  pred_cls={cls}")
