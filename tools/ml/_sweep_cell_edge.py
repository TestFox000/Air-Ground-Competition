# -*- coding: utf-8 -*-
"""cell_label 专用边缘过滤扫描
cell_label FP 56% 在 x>0.8, 尝试仅对 cell_label 做边缘过滤
"""
from pathlib import Path
import cv2

GT_DIR = Path(r"E:\ml_datasets\aerial_field_7cls\labels\val")
IMG_DIR = Path(r"E:\ml_datasets\aerial_field_7cls\images\val")
PRED_DIR = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\runs\detect\runs\detect\runs\detect\aerial_field_7cls_combined\pred_val\labels")
NAMES = ["mine", "car", "arrow", "target_red", "cell_label", "target_blue", "obstacle"]
EXCLUDE_CLS = {1}
CELL_CLS = 4
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
        items.append((cls, cx * w, cy * h, bw * w, bh * h, cx, cy))
    return items

def iou(a, b):
    ax1, ay1 = a[0] - a[2] / 2, a[1] - a[3] / 2
    ax2, ay2 = a[0] + a[2] / 2, a[1] + a[3] / 2
    bx1, by1 = b[0] - b[2] / 2, b[1] - b[3] / 2
    bx2, by2 = b[0] + b[2] / 2, b[1] + b[3] / 2
    ix = max(0, min(ax2, bx2) - max(ax1, bx1))
    iy = max(0, min(ay2, by2) - max(ay1, by1))
    return ix * iy / (a[2] * a[3] + b[2] * b[3] - ix * iy + 1e-9)

def cell_edge_filter(items, thr):
    """仅对 cell_label 做边缘过滤"""
    kept = []
    for it in items:
        if it[0] == CELL_CLS:
            cx, cy = it[5], it[6]
            if cx < thr or cx > 1 - thr or cy < thr or cy > 1 - thr:
                continue
        kept.append(it)
    return kept

def eval_all(frames, cell_thr):
    total_tp = total_fp = total_fn = 0
    for img_f in frames:
        img = cv2.imread(str(img_f))
        if img is None:
            continue
        h, w = img.shape[:2]
        stem = img_f.stem
        gt = load_labels(GT_DIR / f"{stem}.txt", w, h)
        pred = cell_edge_filter(load_labels(PRED_DIR / f"{stem}.txt", w, h), cell_thr)
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
print("cell_label 专用边缘过滤扫描 (mine 仍用 1% edge filter)")
print("=" * 80)
print(f"{'cell_thr':>10} {'全部34帧':>12} {'航拍23帧':>12} {'cell_TP':>8} {'cell_FP':>8} {'cell_FN':>8}")
print("-" * 80)

best_thr = 0
best_acc = 0
for thr_pct in range(1, 16):
    thr = thr_pct / 100.0
    acc_all, tp, fp, fn = eval_all(all_frames, thr)
    acc_frame, _, _, _ = eval_all(frame_frames, thr)
    cell_tp = cell_fp = cell_fn = 0
    for img_f in all_frames:
        img = cv2.imread(str(img_f))
        if img is None:
            continue
        h, w = img.shape[:2]
        stem = img_f.stem
        gt = [g for g in load_labels(GT_DIR / f"{stem}.txt", w, h) if g[0] == CELL_CLS]
        pred = [p for p in cell_edge_filter(load_labels(PRED_DIR / f"{stem}.txt", w, h), thr) if p[0] == CELL_CLS]
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
        cell_tp += ctp
        cell_fp += len(pred) - ctp
        cell_fn += len(gt) - ctp
    print(f"{thr_pct:>9}% {acc_all*100:>11.2f}% {acc_frame*100:>11.2f}% {cell_tp:>8} {cell_fp:>8} {cell_fn:>8}")
    if acc_all > best_acc:
        best_acc = acc_all
        best_thr = thr_pct

print()
print(f"最优 cell_label edge_thr = {best_thr}% (全部34帧综合精度 {best_acc*100:.2f}%)")
acc_1, _, _, _ = eval_all(all_frames, 0.01)
print(f"1% (默认): {acc_1*100:.2f}%")
print(f"增益: +{(best_acc - acc_1)*100:.2f} pp")
