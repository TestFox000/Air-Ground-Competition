# -*- coding: utf-8 -*-
# ⚠️ 留档脚本：7cls/旧线，2026-10-16 起已切换 D13 十三类统一模型（hbb_d13_best.pt），仅供追溯，勿用于现役训练/评估 ⚠️
"""mine 联合集成：distill + combined 双模型 mine 预测取并集(最大化召回)，
经 IoU 去重 + 边缘过滤 + mine_cls_v5 二次确认。
obstacle/arrow/cell_label/target_red/target_blue 取自 combined。

用法:
    python tools/ml/_ensemble_mine_union.py
"""
import subprocess
import sys
from pathlib import Path

import numpy as np
from PIL import Image
from ultralytics import YOLO

REPO = Path(__file__).resolve().parents[2]
PYTHON = r"C:\Users\a\venvs\air_ground\Scripts\python.exe"

COMBINED_LABELS = REPO / "runs" / "detect" / "runs" / "detect" / "runs" / "detect" / "aerial_field_7cls_combined" / "pred_val" / "labels"
DISTILL_LABELS = REPO / "runs" / "detect" / "runs" / "detect" / "runs" / "detect" / "aerial_field_7cls" / "pred_valB" / "labels"
MERGED_DIR = REPO / "runs" / "detect" / "runs" / "detect" / "runs" / "detect" / "aerial_field_7cls_combined" / "pred_val_mine_union" / "labels"

VAL_IMG = Path(r"E:/ml_datasets/aerial_field_7cls/images/val")
VAL_GT = Path(r"E:/ml_datasets/aerial_field_7cls/labels/val")
CLASSIFIER = Path(r"E:/ml_datasets/downloaded/4090_batch2/mine_cls_v5_best.onnx")

NAMES = "mine,car,arrow,target_red,cell_label,target_blue,obstacle"
EXCLUDE = "car"

EDGE_THR = 0.01
MINE_CLS = 0
CLS_THRESH = 0.9
IOU_THR = 0.5  # mine 去重 IoU 阈值
NON_MINE_CLASSES = {2, 3, 4, 5, 6}


def iou_xywh(a, b):
    ax1, ay1 = a[0] - a[2] / 2, a[1] - a[3] / 2
    ax2, ay2 = a[0] + a[2] / 2, a[1] + a[3] / 2
    bx1, by1 = b[0] - b[2] / 2, b[1] - b[3] / 2
    bx2, by2 = b[0] + b[2] / 2, b[1] + b[3] / 2
    ix = max(0, min(ax2, bx2) - max(ax1, bx1))
    iy = max(0, min(ay2, by2) - max(ay1, by1))
    inter = ix * iy
    return inter / (a[2] * a[3] + b[2] * b[3] - inter + 1e-9)


def main():
    if not CLASSIFIER.exists():
        print(f"mine_cls_v5 不存在: {CLASSIFIER}", file=sys.stderr)
        return 1

    MERGED_DIR.mkdir(parents=True, exist_ok=True)
    print(f"加载 mine_cls_v5: {CLASSIFIER}")
    cls_model = YOLO(str(CLASSIFIER))
    mine_idx = 0 if cls_model.names[0] == "mine" else 1

    # 收集所有有预测的帧
    all_frames = set()
    if COMBINED_LABELS.exists():
        all_frames.update(p.stem for p in COMBINED_LABELS.glob("*.txt"))
    if DISTILL_LABELS.exists():
        all_frames.update(p.stem for p in DISTILL_LABELS.glob("*.txt"))

    total_edge = 0
    total_cls_kept = 0
    total_cls_rem = 0
    total_dedup = 0

    for stem in sorted(all_frames):
        merged = []

        # combined 非 mine 类
        comb_path = COMBINED_LABELS / f"{stem}.txt"
        if comb_path.exists():
            for line in comb_path.read_text().strip().split("\n"):
                if not line.strip():
                    continue
                cls = int(line.strip().split()[0])
                if cls in NON_MINE_CLASSES:
                    merged.append(line.strip())

        # 收集 mine 预测：combined + distill
        mine_boxes = []  # (cx, cy, w, h, source)
        for src_dir in [COMBINED_LABELS, DISTILL_LABELS]:
            p = src_dir / f"{stem}.txt"
            if not p.exists():
                continue
            for line in p.read_text().strip().split("\n"):
                if not line.strip():
                    continue
                parts = line.strip().split()
                if int(parts[0]) == MINE_CLS:
                    cx, cy, w, h = map(float, parts[1:5])
                    # 边缘过滤
                    if cx < EDGE_THR or cx > 1 - EDGE_THR or cy < EDGE_THR or cy > 1 - EDGE_THR:
                        total_edge += 1
                        continue
                    mine_boxes.append((cx, cy, w, h))

        # IoU 去重：保留先出现的（combined 优先）
        deduped = []
        for box in mine_boxes:
            dup = False
            for kept in deduped:
                if iou_xywh(box, kept) >= IOU_THR:
                    dup = True
                    total_dedup += 1
                    break
            if not dup:
                deduped.append(box)

        # mine_cls_v5 二次确认
        img_path = VAL_IMG / f"{stem}.jpg"
        if img_path.exists() and deduped:
            img = Image.open(img_path).convert("RGB")
            W, H = img.size
            for cx, cy, w, h in deduped:
                x1 = (cx - w / 2) * W
                y1 = (cy - h / 2) * H
                x2 = (cx + w / 2) * W
                y2 = (cy + h / 2) * H
                xi1, yi1 = max(0, int(x1)), max(0, int(y1))
                xi2, yi2 = min(W, int(x2)), min(H, int(y2))
                if xi2 - xi1 < 5 or yi2 - yi1 < 5:
                    total_cls_rem += 1
                    continue
                crop = img.crop((xi1, yi1, xi2, yi2)).resize((96, 96))
                cls_res = cls_model(np.array(crop), verbose=False, device=0)[0]
                probs = cls_res.probs.data.cpu().numpy()
                mine_prob = probs[mine_idx]
                if mine_prob < CLS_THRESH:
                    total_cls_rem += 1
                    continue
                merged.append(f"{MINE_CLS} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f} {float(mine_prob):.6f}")
                total_cls_kept += 1

        out = MERGED_DIR / f"{stem}.txt"
        out.write_text("\n".join(merged) + ("\n" if merged else ""), encoding="utf-8")

    del cls_model
    print(f"\nmine 联合集成完成: {len(all_frames)} 帧")
    print(f"  边缘过滤移除: {total_edge}")
    print(f"  IoU 去重移除: {total_dedup}")
    print(f"  mine_cls_v5 保留: {total_cls_kept}, 移除: {total_cls_rem}")

    eval_script = REPO / "tools" / "ml" / "eval_comprehensive.py"
    cmd = [
        PYTHON, str(eval_script),
        "--pred-dir", str(MERGED_DIR),
        "--gt-dir", str(VAL_GT),
        "--img-dir", str(VAL_IMG),
        "--names", NAMES,
        "--exclude", EXCLUDE,
    ]
    print("\n运行 eval_comprehensive.py ...")
    r = subprocess.run(cmd, capture_output=True, text=True)
    print(r.stdout)
    if r.stderr:
        print(r.stderr, file=sys.stderr)
    return r.returncode


if __name__ == "__main__":
    sys.exit(main())
