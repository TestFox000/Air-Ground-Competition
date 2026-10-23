# -*- coding: utf-8 -*-
"""mine FP 空间分布分析: 检查 73 个 mine FP 是否集中在特定区域(可过滤)
输出: FP 的位置分布 (x/y 分桶) + 尺寸分布
"""
from pathlib import Path
import cv2
from collections import Counter

GT_DIR = Path(r"E:\ml_datasets\aerial_field_7cls\labels\val")
IMG_DIR = Path(r"E:\ml_datasets\aerial_field_7cls\images\val")
PRED_DIR = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\runs\detect\runs\detect\runs\detect\aerial_field_7cls_combined\pred_val\labels")
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
        items.append((cls, cx * w, cy * h, bw * w, bh * h, cx, cy, bw, bh))
    return items

def iou(a, b):
    ax1, ay1 = a[0] - a[2] / 2, a[1] - a[3] / 2
    ax2, ay2 = a[0] + a[2] / 2, a[1] + a[3] / 2
    bx1, by1 = b[0] - b[2] / 2, b[1] - b[3] / 2
    bx2, by2 = b[0] + b[2] / 2, b[1] + b[3] / 2
    ix = max(0, min(ax2, bx2) - max(ax1, bx1))
    iy = max(0, min(ay2, by2) - max(ay1, by1))
    return ix * iy / (a[2] * a[3] + b[2] * b[3] - ix * iy + 1e-9)

fp_list = []  # (frame, cx_norm, cy_norm, area_norm)
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
    matched = [False] * len(mine_pred)
    for g in mine_gt:
        best_iou, best_j = 0, -1
        for j, p in enumerate(mine_pred):
            if matched[j]:
                continue
            v = iou(g[1:5], p[1:5])
            if v > best_iou:
                best_iou, best_j = v, j
        if best_j >= 0 and best_iou >= 0.5:
            matched[best_j] = True
    for j, p in enumerate(mine_pred):
        if not matched[j]:
            # FP
            cx_n, cy_n = p[5], p[6]
            area_n = p[7] * p[8]
            fp_list.append((stem, cx_n, cy_n, area_n))

print(f"mine FP 总数: {len(fp_list)}")
print()
print("=== FP 位置分布 (x 分桶, 归一化坐标) ===")
x_buckets = Counter()
for _, cx, _, _ in fp_list:
    b = int(cx * 10)
    x_buckets[min(b, 9)] += 1
for b in range(10):
    bar = "#" * x_buckets.get(b, 0)
    print(f"  x[{b*0.1:.1f}-{(b+1)*0.1:.1f}]: {x_buckets.get(b, 0):>3} {bar}")

print()
print("=== FP 位置分布 (y 分桶, 归一化坐标) ===")
y_buckets = Counter()
for _, _, cy, _ in fp_list:
    b = int(cy * 10)
    y_buckets[min(b, 9)] += 1
for b in range(10):
    bar = "#" * y_buckets.get(b, 0)
    print(f"  y[{b*0.1:.1f}-{(b+1)*0.1:.1f}]: {y_buckets.get(b, 0):>3} {bar}")

print()
print("=== FP 面积分布 (归一化面积 w*h) ===")
area_buckets = Counter()
for _, _, _, a in fp_list:
    if a < 0.001:
        area_buckets["<0.001"] += 1
    elif a < 0.005:
        area_buckets["0.001-0.005"] += 1
    elif a < 0.01:
        area_buckets["0.005-0.01"] += 1
    elif a < 0.05:
        area_buckets["0.01-0.05"] += 1
    else:
        area_buckets[">=0.05"] += 1
for k, v in sorted(area_buckets.items()):
    print(f"  area {k:>12s}: {v}")

print()
print("=== FP 帧分布 (前10) ===")
frame_cnt = Counter(f[0] for f in fp_list)
for stem, cnt in frame_cnt.most_common(10):
    print(f"  {stem}: {cnt}")

# 检查是否有大量 FP 集中在图像中心区域 (可能是场地中心标记误检)
center_fp = sum(1 for _, cx, cy, _ in fp_list if 0.3 < cx < 0.7 and 0.3 < cy < 0.7)
print(f"\n中心区域 (0.3<x<0.7, 0.3<y<0.7) FP: {center_fp}/{len(fp_list)} = {center_fp/len(fp_list)*100:.1f}%")
