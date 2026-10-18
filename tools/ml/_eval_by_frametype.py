# -*- coding: utf-8 -*-
# ⚠️ 留档脚本：7cls/旧线，2026-10-16 起已切换 D13 十三类统一模型（hbb_d13_best.pt），仅供追溯，勿用于现役训练/评估 ⚠️
"""分帧类型评估: 分别计算 frame_*(航拍) 和 explosive_*(官方图) 的综合识别精度
判断域差异对 95% 硬指标的实际影响
"""
from pathlib import Path
import cv2

GT_DIR = Path(r"E:\ml_datasets\aerial_field_7cls\labels\val")
IMG_DIR = Path(r"E:\ml_datasets\aerial_field_7cls\images\val")
PRED_DIR = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\runs\detect\runs\detect\runs\detect\aerial_field_7cls_combined\pred_val\labels")
NAMES = ["mine", "car", "arrow", "target_red", "cell_label", "target_blue", "obstacle"]
EXCLUDE_CLS = {1}  # car

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
        items.append((cls, cx * w, cy * h, bw * w, bh * h))
    return items

def iou(a, b):
    ax1, ay1 = a[0] - a[2] / 2, a[1] - a[3] / 2
    ax2, ay2 = a[0] + a[2] / 2, a[1] + a[3] / 2
    bx1, by1 = b[0] - b[2] / 2, b[1] - b[3] / 2
    bx2, by2 = b[0] + b[2] / 2, b[1] + b[3] / 2
    ix = max(0, min(ax2, bx2) - max(ax1, bx1))
    iy = max(0, min(ay2, by2) - max(ay1, by1))
    return ix * iy / (a[2] * a[3] + b[2] * b[3] - ix * iy + 1e-9)

def eval_set(frames):
    total_tp = total_fp = total_fn = 0
    per_cls = {i: [0, 0, 0] for i in range(len(NAMES))}
    for img_f in frames:
        img = cv2.imread(str(img_f))
        if img is None:
            continue
        h, w = img.shape[:2]
        stem = img_f.stem
        gt = load_labels(GT_DIR / f"{stem}.txt", w, h)
        pred = load_labels(PRED_DIR / f"{stem}.txt", w, h)
        # overall
        matched = [False] * len(pred)
        tp = 0
        for g in gt:
            best_iou, best_j = 0, -1
            for j, p in enumerate(pred):
                if matched[j] or p[0] != g[0]:
                    continue
                v = iou(g[1:], p[1:])
                if v > best_iou:
                    best_iou, best_j = v, j
            if best_j >= 0 and best_iou >= 0.5:
                matched[best_j] = True
                tp += 1
        total_tp += tp
        total_fp += len(pred) - tp
        total_fn += len(gt) - tp
        # per class
        for i in range(len(NAMES)):
            if i in EXCLUDE_CLS:
                continue
            g_i = [g for g in gt if g[0] == i]
            p_i = [p for p in pred if p[0] == i]
            m = [False] * len(p_i)
            ctp = 0
            for g in g_i:
                bi, bj = 0, -1
                for j, p in enumerate(p_i):
                    if m[j]:
                        continue
                    v = iou(g[1:], p[1:])
                    if v > bi:
                        bi, bj = v, j
                if bj >= 0 and bi >= 0.5:
                    m[bj] = True
                    ctp += 1
            per_cls[i][0] += ctp
            per_cls[i][1] += len(p_i) - ctp
            per_cls[i][2] += len(g_i) - ctp
    overall = total_tp / max(1, total_tp + total_fp + total_fn)
    return overall, total_tp, total_fp, total_fn, per_cls

all_frames = sorted(IMG_DIR.glob("*.jpg"))
frame_frames = [f for f in all_frames if f.stem.startswith("frame")]
explosive_frames = [f for f in all_frames if f.stem.startswith("explosive")]

print(f"总帧数: {len(all_frames)}  (frame_*={len(frame_frames)}, explosive_*={len(explosive_frames)})")
print()

for label, frames in [("全部 34 帧 (val-B)", all_frames),
                      ("frame_* 仅航拍帧", frame_frames),
                      ("explosive_* 仅官方图帧", explosive_frames)]:
    overall, tp, fp, fn, per_cls = eval_set(frames)
    print("=" * 60)
    print(f"{label}  ({len(frames)} 帧)")
    print("=" * 60)
    for i, n in enumerate(NAMES):
        if i in EXCLUDE_CLS:
            continue
        t, f, nn = per_cls[i]
        if t + f + nn == 0:
            continue
        acc = t / max(1, t + f + nn)
        print(f"  {n:12s}: acc={acc:.4f}  (TP={t} FP={f} FN={nn})")
    print("-" * 60)
    print(f"  综合识别精度: {overall*100:.2f}%  (TP={tp} FP={fp} FN={fn})")
    gap = (0.95 - overall) * 100
    if gap > 0:
        print(f"  距 95%: 还差 {gap:.2f} pp")
    else:
        print(f"  ✅ 已达标, 超出 {-gap:.2f} pp")
    print()
