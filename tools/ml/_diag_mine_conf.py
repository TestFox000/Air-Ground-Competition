# -*- coding: utf-8 -*-
"""低阈值推理诊断: 检查 mine FN=38 中有多少可在 conf<0.25 时检出
推理 combined best.pt at conf=0.05, 然后对 38 个 FN mine 统计
在不同 conf 档位下的检出率, 找到最优 mine 阈值
"""
from pathlib import Path
import subprocess, sys, cv2
import numpy as np

REPO = Path(r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground")
PYTHON = r"C:\Users\a\venvs\air_ground\Scripts\python.exe"
BEST_PT = REPO / "runs" / "detect" / "runs" / "detect" / "runs" / "detect" / "aerial_field_7cls_combined" / "weights" / "best.pt"
VAL_IMG = Path(r"E:\ml_datasets\aerial_field_7cls\images\val")
VAL_GT = Path(r"E:\ml_datasets\aerial_field_7cls\labels\val")
OUT_DIR = REPO / "runs" / "detect" / "runs" / "detect" / "runs" / "detect" / "aerial_field_7cls_combined" / "pred_val_conf005"
NAMES = ["mine", "car", "arrow", "target_red", "cell_label", "target_blue", "obstacle"]
MINE_CLS = 0

def load_labels(path, w, h, with_conf=False):
    items = []
    if not path.exists():
        return items
    for line in path.read_text().strip().split("\n"):
        if not line.strip():
            continue
        p = line.split()
        cls = int(p[0])
        cx, cy, bw, bh = map(float, p[1:5])
        conf = float(p[5]) if with_conf and len(p) > 5 else 1.0
        items.append((cls, cx * w, cy * h, bw * w, bh * h, conf))
    return items

def iou(a, b):
    ax1, ay1 = a[0] - a[2] / 2, a[1] - a[3] / 2
    ax2, ay2 = a[0] + a[2] / 2, a[1] + a[3] / 2
    bx1, by1 = b[0] - b[2] / 2, b[1] - b[3] / 2
    bx2, by2 = b[0] + b[2] / 2, b[1] + b[3] / 2
    ix = max(0, min(ax2, bx2) - max(ax1, bx1))
    iy = max(0, min(ay2, by2) - max(ay1, by1))
    return ix * iy / (a[2] * a[3] + b[2] * b[3] - ix * iy + 1e-9)

# 1. 推理 (conf=0.05, save_conf=True) — 若已存在则跳过
PRED_EXISTS = (OUT_DIR / "labels").exists() and any((OUT_DIR / "labels").glob("*.txt"))
if PRED_EXISTS:
    print(f"跳过推理 (pred_labels 已存在: {OUT_DIR / 'labels'})")
else:
    print("推理 combined best.pt at conf=0.05 ...")
    infer_cmd = [
        PYTHON, "-c",
        (
            "from ultralytics import YOLO; "
            f"m=YOLO(r'{BEST_PT}'); "
            f"m.predict(source=r'{VAL_IMG}', imgsz=1280, conf=0.05, iou=0.45, "
            f"save_txt=True, save_conf=True, project=r'{OUT_DIR.parent}', "
            f"name='{OUT_DIR.name}', exist_ok=True, verbose=False, device=0)"
        ),
    ]
    r = subprocess.run(infer_cmd, cwd=str(REPO), capture_output=True, text=True, timeout=1800)
    if r.returncode != 0:
        print(f"推理失败: {r.stderr[-2000:]}")
        sys.exit(1)
    print("推理完成")

# 2. mine 边缘过滤
pred_labels = OUT_DIR / "labels"
edge_thr = 0.01
removed = 0
for lbl_f in pred_labels.glob("*.txt"):
    lines = lbl_f.read_text().strip().splitlines()
    kept = []
    for line in lines:
        if not line.strip():
            continue
        parts = line.split()
        cls = int(parts[0])
        if cls == MINE_CLS:
            cx, cy = float(parts[1]), float(parts[2])
            if cx < edge_thr or cx > 1 - edge_thr or cy < edge_thr or cy > 1 - edge_thr:
                removed += 1
                continue
        kept.append(line)
    lbl_f.write_text("\n".join(kept) + ("\n" if kept else ""))
print(f"mine 边缘过滤: 移除 {removed} 个")

# 3. 统计不同 conf 档位下 mine 类的 TP/FP/FN
thresholds = [0.05, 0.10, 0.15, 0.20, 0.25, 0.30]
print("\n" + "=" * 70)
print("mine 类不同置信度阈值下的 TP/FP/FN (IoU>=0.5)")
print("=" * 70)
print(f"{'conf':>6} {'TP':>5} {'FP':>5} {'FN':>5} {'acc':>8} {'precision':>10} {'recall':>8}")

for thr in thresholds:
    tp = fp = fn = 0
    for img_f in sorted(VAL_IMG.glob("*.jpg")):
        img = cv2.imread(str(img_f))
        if img is None:
            continue
        h, w = img.shape[:2]
        stem = img_f.stem
        gt = load_labels(VAL_GT / f"{stem}.txt", w, h)
        pred = load_labels(pred_labels / f"{stem}.txt", w, h, with_conf=True)
        mine_gt = [g for g in gt if g[0] == MINE_CLS]
        mine_pred = [p for p in pred if p[0] == MINE_CLS and p[5] >= thr]
        matched = [False] * len(mine_pred)
        frame_tp = 0
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
                frame_tp += 1
        tp += frame_tp
        fn += len(mine_gt) - frame_tp
        fp += len(mine_pred) - frame_tp
    acc = tp / max(1, tp + fp + fn)
    prec = tp / max(1, tp + fp)
    rec = tp / max(1, tp + fn)
    print(f"{thr:>6.2f} {tp:>5} {fp:>5} {fn:>5} {acc:>8.4f} {prec:>10.4f} {rec:>8.4f}")
