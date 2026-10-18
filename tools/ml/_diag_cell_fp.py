# -*- coding: utf-8 -*-
"""cell_label FP 分析 + 阈值扫描
cell_label: TP=1023 FP=80 FN=9, 检查 FP 模式和阈值优化空间
"""
from pathlib import Path
import cv2
from collections import Counter

GT_DIR = Path(r"E:\ml_datasets\aerial_field_7cls\labels\val")
IMG_DIR = Path(r"E:\ml_datasets\aerial_field_7cls\images\val")
PRED_DIR = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\runs\detect\runs\detect\runs\detect\aerial_field_7cls_combined\pred_val\labels")
CELL_CLS = 4  # cell_label

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
        items.append((cls, cx * w, cy * h, bw * w, bh * h, cx, cy, bw * bh))
    return items

def iou(a, b):
    ax1, ay1 = a[0] - a[2] / 2, a[1] - a[3] / 2
    ax2, ay2 = a[0] + a[2] / 2, a[1] + a[3] / 2
    bx1, by1 = b[0] - b[2] / 2, b[1] - b[3] / 2
    bx2, by2 = b[0] + b[2] / 2, b[1] + b[3] / 2
    ix = max(0, min(ax2, bx2) - max(ax1, bx1))
    iy = max(0, min(ay2, by2) - max(ay1, by1))
    return ix * iy / (a[2] * a[3] + b[2] * b[3] - ix * iy + 1e-9)

# 1. cell_label FP 分布
fp_list = []
for img_f in sorted(IMG_DIR.glob("*.jpg")):
    img = cv2.imread(str(img_f))
    if img is None:
        continue
    h, w = img.shape[:2]
    stem = img_f.stem
    gt = [g for g in load_labels(GT_DIR / f"{stem}.txt", w, h) if g[0] == CELL_CLS]
    pred = [p for p in load_labels(PRED_DIR / f"{stem}.txt", w, h) if p[0] == CELL_CLS]
    matched = [False] * len(pred)
    for g in gt:
        best_iou, best_j = 0, -1
        for j, p in enumerate(pred):
            if matched[j]:
                continue
            v = iou(g[1:5], p[1:5])
            if v > best_iou:
                best_iou, best_j = v, j
        if best_j >= 0 and best_iou >= 0.5:
            matched[best_j] = True
    for j, p in enumerate(pred):
        if not matched[j]:
            fp_list.append((stem, p[5], p[6], p[7]))

print(f"cell_label FP 总数: {len(fp_list)}")
print()
print("=== FP 位置分布 (x 分桶) ===")
x_buckets = Counter()
for _, cx, _, _ in fp_list:
    b = int(cx * 10)
    x_buckets[min(b, 9)] += 1
for b in range(10):
    bar = "#" * x_buckets.get(b, 0)
    print(f"  x[{b*0.1:.1f}-{(b+1)*0.1:.1f}]: {x_buckets.get(b, 0):>3} {bar}")

print()
print("=== FP 面积分布 ===")
area_buckets = Counter()
for _, _, _, a in fp_list:
    if a < 0.0005:
        area_buckets["<0.0005"] += 1
    elif a < 0.001:
        area_buckets["0.0005-0.001"] += 1
    elif a < 0.002:
        area_buckets["0.001-0.002"] += 1
    else:
        area_buckets[">=0.002"] += 1
for k, v in sorted(area_buckets.items()):
    print(f"  area {k:>12s}: {v}")

print()
print("=== FP 帧分布 ===")
frame_cnt = Counter(f[0] for f in fp_list)
for stem, cnt in frame_cnt.most_common(10):
    print(f"  {stem}: {cnt}")

# 2. cell_label 面积过滤扫描
print()
print("=" * 70)
print("cell_label 面积过滤扫描")
print("=" * 70)

def eval_cell(min_area):
    tp = fp = fn = 0
    for img_f in sorted(IMG_DIR.glob("*.jpg")):
        img = cv2.imread(str(img_f))
        if img is None:
            continue
        h, w = img.shape[:2]
        stem = img_f.stem
        gt = [g for g in load_labels(GT_DIR / f"{stem}.txt", w, h) if g[0] == CELL_CLS]
        pred = [p for p in load_labels(PRED_DIR / f"{stem}.txt", w, h) if p[0] == CELL_CLS and p[7] >= min_area]
        matched = [False] * len(pred)
        ctp = 0
        for g in gt:
            bi, bj = 0, -1
            for j, p in enumerate(pred):
                if matched[j]:
                    continue
                v = iou(g[1:5], p[1:5])
                if v > bi:
                    bi, bj = v, j
            if bj >= 0 and bi >= 0.5:
                matched[bj] = True
                ctp += 1
        tp += ctp
        fp += len(pred) - ctp
        fn += len(gt) - ctp
    acc = tp / max(1, tp + fp + fn)
    return acc, tp, fp, fn

print(f"{'min_area':>12} {'cell_acc':>10} {'TP':>6} {'FP':>6} {'FN':>6}")
for ma in [0, 0.0003, 0.0005, 0.0008, 0.001, 0.0015, 0.002]:
    acc, tp, fp, fn = eval_cell(ma)
    label = f"{ma:.4f}" if ma > 0 else "0"
    print(f"{label:>12} {acc:>10.4f} {tp:>6} {fp:>6} {fn:>6}")
