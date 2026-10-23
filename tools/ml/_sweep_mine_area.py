# -*- coding: utf-8 -*-
"""mine 面积过滤扫描: 测试最小面积阈值对综合精度的影响
51/80 mine FP 面积<0.001, 检查面积过滤能否净增益
"""
from pathlib import Path
import cv2

GT_DIR = Path(r"E:\ml_datasets\aerial_field_7cls\labels\val")
IMG_DIR = Path(r"E:\ml_datasets\aerial_field_7cls\images\val")
PRED_DIR = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\runs\detect\runs\detect\runs\detect\aerial_field_7cls_combined\pred_val\labels")
NAMES = ["mine", "car", "arrow", "target_red", "cell_label", "target_blue", "obstacle"]
EXCLUDE_CLS = {1}
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
        if cls in EXCLUDE_CLS:
            continue
        cx, cy, bw, bh = map(float, p[1:5])
        items.append((cls, cx * w, cy * h, bw * w, bh * h, bw * bh))
    return items

def iou(a, b):
    ax1, ay1 = a[0] - a[2] / 2, a[1] - a[3] / 2
    ax2, ay2 = a[0] + a[2] / 2, a[1] + a[3] / 2
    bx1, by1 = b[0] - b[2] / 2, b[1] - b[3] / 2
    bx2, by2 = b[0] + b[2] / 2, b[1] + b[3] / 2
    ix = max(0, min(ax2, bx2) - max(ax1, bx1))
    iy = max(0, min(ay2, by2) - max(ay1, by1))
    return ix * iy / (a[2] * a[3] + b[2] * b[3] - ix * iy + 1e-9)

def area_filter(items, min_area):
    """过滤归一化面积 < min_area 的 mine 检测"""
    kept = []
    for it in items:
        if it[0] == MINE_CLS and it[5] < min_area:
            continue
        kept.append(it)
    return kept

def eval_all(frames, min_area):
    total_tp = total_fp = total_fn = 0
    for img_f in frames:
        img = cv2.imread(str(img_f))
        if img is None:
            continue
        h, w = img.shape[:2]
        stem = img_f.stem
        gt = load_labels(GT_DIR / f"{stem}.txt", w, h)
        pred = area_filter(load_labels(PRED_DIR / f"{stem}.txt", w, h), min_area)
        matched = [False] * len(pred)
        tp = 0
        for g in gt:
            best_iou, best_j = 0, -1
            for j, p in enumerate(pred):
                if matched[j] or p[0] != g[0]:
                    continue
                v = iou(g[1:5], p[1:5])
                if v > best_iou:
                    best_iou, best_j = v, j
            if best_j >= 0 and best_iou >= 0.5:
                matched[best_j] = True
                tp += 1
        total_tp += tp
        total_fp += len(pred) - tp
        total_fn += len(gt) - tp
    return total_tp / max(1, total_tp + total_fp + total_fn), total_tp, total_fp, total_fn

all_frames = sorted(IMG_DIR.glob("*.jpg"))
frame_frames = [f for f in all_frames if f.stem.startswith("frame")]

print("=" * 80)
print("mine 面积过滤阈值扫描 (1% edge filter 已应用)")
print("=" * 80)
print(f"{'min_area':>12} {'全部34帧':>12} {'航拍23帧':>12} {'mine_FP':>8} {'mine_TP':>8} {'mine_FN':>8}")
print("-" * 80)

best_area = 0
best_acc = 0
for area_pct in [0, 0.0005, 0.001, 0.0015, 0.002, 0.003, 0.004, 0.005, 0.007, 0.01]:
    acc_all, tp, fp, fn = eval_all(all_frames, area_pct)
    acc_frame, _, _, _ = eval_all(frame_frames, area_pct)
    mine_tp = mine_fp = mine_fn = 0
    for img_f in all_frames:
        img = cv2.imread(str(img_f))
        if img is None:
            continue
        h, w = img.shape[:2]
        stem = img_f.stem
        gt = [g for g in load_labels(GT_DIR / f"{stem}.txt", w, h) if g[0] == MINE_CLS]
        pred = [p for p in area_filter(load_labels(PRED_DIR / f"{stem}.txt", w, h), area_pct) if p[0] == MINE_CLS]
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
        mine_tp += ctp
        mine_fp += len(pred) - ctp
        mine_fn += len(gt) - ctp
    label = f"{area_pct:.4f}" if area_pct > 0 else "0(无过滤)"
    print(f"{label:>12} {acc_all*100:>11.2f}% {acc_frame*100:>11.2f}% {mine_fp:>8} {mine_tp:>8} {mine_fn:>8}")
    if acc_all > best_acc:
        best_acc = acc_all
        best_area = area_pct

print()
print(f"最优 min_area = {best_area} (全部34帧综合精度 {best_acc*100:.2f}%)")
acc_0, _, _, _ = eval_all(all_frames, 0)
print(f"无面积过滤: {acc_0*100:.2f}%")
print(f"增益: +{(best_acc - acc_0)*100:.2f} pp")
