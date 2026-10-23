# -*- coding: utf-8 -*-
"""mine FN 帧分布分析: 统计 38 个 FN mine 分别在 explosive_*(官方图) 和 frame_*(航拍图) 中的分布
同时计算两类帧各自的 mine 准确率, 判断域差异影响
"""
from pathlib import Path
import cv2

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

groups = {"explosive": {"gt": 0, "tp": 0, "fp": 0, "fn": 0},
          "frame": {"gt": 0, "tp": 0, "fp": 0, "fn": 0}}
fn_by_frame = {}

for img_f in sorted(IMG_DIR.glob("*.jpg")):
    img = cv2.imread(str(img_f))
    if img is None:
        continue
    h, w = img.shape[:2]
    stem = img_f.stem
    grp = "explosive" if stem.startswith("explosive") else "frame"
    gt = load_labels(GT_DIR / f"{stem}.txt", w, h)
    pred = load_labels(PRED_DIR / f"{stem}.txt", w, h)
    mine_gt = [g for g in gt if g[0] == MINE_CLS]
    mine_pred = [p for p in pred if p[0] == MINE_CLS]
    matched = [False] * len(mine_pred)
    frame_tp = 0
    for g in mine_gt:
        best_iou, best_j = 0, -1
        for j, p in enumerate(mine_pred):
            if matched[j]:
                continue
            v = iou(g[1:], p[1:])
            if v > best_iou:
                best_iou, best_j = v, j
        if best_j >= 0 and best_iou >= 0.5:
            matched[best_j] = True
            frame_tp += 1
    frame_fn = len(mine_gt) - frame_tp
    frame_fp = len(mine_pred) - frame_tp
    groups[grp]["gt"] += len(mine_gt)
    groups[grp]["tp"] += frame_tp
    groups[grp]["fp"] += frame_fp
    groups[grp]["fn"] += frame_fn
    if frame_fn > 0:
        fn_by_frame[stem] = (len(mine_gt), frame_tp, frame_fn, frame_fp)

print("=" * 70)
print("mine FN 帧类型分布分析")
print("=" * 70)
for grp in ["explosive", "frame"]:
    g = groups[grp]
    acc = g["tp"] / max(1, g["tp"] + g["fp"] + g["fn"])
    rec = g["tp"] / max(1, g["gt"])
    print(f"\n[{grp}_* 帧]  GT={g['gt']}  TP={g['tp']}  FP={g['fp']}  FN={g['fn']}")
    print(f"  mine 准确率={acc:.4f}  召回率={rec:.4f}")

print("\n" + "=" * 70)
print("含 mine FN 的帧明细")
print("=" * 70)
print(f"{'帧名':<18} {'mine_GT':>8} {'TP':>5} {'FN':>5} {'FP':>5}")
for stem, (ngt, tp, fn, fp) in sorted(fn_by_frame.items()):
    print(f"{stem:<18} {ngt:>8} {tp:>5} {fn:>5} {fp:>5}")

print(f"\n含 FN 的帧数: {len(fn_by_frame)}")
print(f"explosive 帧 FN 占比: {groups['explosive']['fn']}/{groups['explosive']['fn']+groups['frame']['fn']} = {groups['explosive']['fn']/(groups['explosive']['fn']+groups['frame']['fn'])*100:.1f}%")
