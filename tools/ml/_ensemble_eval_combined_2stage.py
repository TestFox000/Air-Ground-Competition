# -*- coding: utf-8 -*-
# ⚠️ 留档脚本：7cls/旧线，2026-10-16 起已切换 D13 十三类统一模型（hbb_d13_best.pt），仅供追溯，勿用于现役训练/评估 ⚠️
"""combined 全类别 + mine_cls_v5 二次确认 mine。

策略（服务 M0-3 省赛95%硬指标）：
- 所有类别取自 combined 预测（combined 根治了 obstacle FN=30 和 arrow 召回低）
- 仅对 mine(cls=0) 应用 mine_cls_v5 分类器二次确认(cls_thresh=0.9) + 1% 边缘过滤
- car 排除

用法:
    python tools/ml/_ensemble_eval_combined_2stage.py
"""
import subprocess
import sys
from pathlib import Path

import numpy as np
from PIL import Image
from ultralytics import YOLO

REPO = Path(__file__).resolve().parents[2]
PYTHON = r"C:\Users\a\venvs\air_ground\Scripts\python.exe"

RUN_DIR = REPO / "runs" / "detect" / "runs" / "detect" / "runs" / "detect" / "aerial_field_7cls_combined"
COMBINED_LABELS = RUN_DIR / "pred_val" / "labels"
MERGED_DIR = RUN_DIR / "pred_val_combined_2stage" / "labels"

VAL_IMG = Path(r"E:/ml_datasets/aerial_field_7cls/images/val")
VAL_GT = Path(r"E:/ml_datasets/aerial_field_7cls/labels/val")
CLASSIFIER = Path(r"E:/ml_datasets/downloaded/4090_batch2/mine_cls_v5_best.onnx")

NAMES = "mine,car,arrow,target_red,cell_label,target_blue,obstacle"
EXCLUDE = "car"

EDGE_THR = 0.01
MINE_CLS = 0
CLS_THRESH = 0.9


def main():
    if not COMBINED_LABELS.exists():
        print(f"combined 预测不存在: {COMBINED_LABELS}", file=sys.stderr)
        return 1
    if not CLASSIFIER.exists():
        print(f"mine_cls_v5 模型不存在: {CLASSIFIER}", file=sys.stderr)
        return 1

    MERGED_DIR.mkdir(parents=True, exist_ok=True)

    print(f"加载 mine_cls_v5 分类器: {CLASSIFIER}")
    cls_model = YOLO(str(CLASSIFIER))
    mine_idx = 0 if cls_model.names[0] == "mine" else 1
    print(f"  mine 类索引 = {mine_idx}")

    total_edge_removed = 0
    total_cls_kept = 0
    total_cls_removed = 0
    frames = sorted(COMBINED_LABELS.glob("*.txt"))
    print(f"处理 {len(frames)} 帧 ...")

    for f in frames:
        stem = f.stem
        lines = f.read_text().strip().split("\n")
        merged = []
        mine_lines = []
        for line in lines:
            parts = line.strip().split()
            if not parts:
                continue
            cls = int(parts[0])
            if cls == MINE_CLS:
                mine_lines.append(line.strip())
            else:
                merged.append(line.strip())

        # mine 边缘过滤
        mine_kept = []
        for line in mine_lines:
            parts = line.strip().split()
            cx, cy = float(parts[1]), float(parts[2])
            if cx < EDGE_THR or cx > 1 - EDGE_THR or cy < EDGE_THR or cy > 1 - EDGE_THR:
                total_edge_removed += 1
                continue
            mine_kept.append(line.strip())

        # mine_cls_v5 二次确认
        img_path = VAL_IMG / f"{stem}.jpg"
        if img_path.exists() and mine_kept:
            img = Image.open(img_path).convert("RGB")
            W, H = img.size
            for line in mine_kept:
                parts = line.strip().split()
                cx, cy, w, h = map(float, parts[1:5])
                x1 = (cx - w / 2) * W
                y1 = (cy - h / 2) * H
                x2 = (cx + w / 2) * W
                y2 = (cy + h / 2) * H
                xi1, yi1 = max(0, int(x1)), max(0, int(y1))
                xi2, yi2 = min(W, int(x2)), min(H, int(y2))
                if xi2 - xi1 < 5 or yi2 - yi1 < 5:
                    total_cls_removed += 1
                    continue
                crop = img.crop((xi1, yi1, xi2, yi2)).resize((96, 96))
                cls_res = cls_model(np.array(crop), verbose=False, device=0)[0]
                probs = cls_res.probs.data.cpu().numpy()
                mine_prob = probs[mine_idx]
                if mine_prob < CLS_THRESH:
                    total_cls_removed += 1
                    continue
                merged.append(f"{MINE_CLS} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f} {float(mine_prob):.6f}")
                total_cls_kept += 1
        elif mine_kept:
            merged.extend(mine_kept)
            total_cls_kept += len(mine_kept)

        out = MERGED_DIR / f.name
        out.write_text("\n".join(merged) + ("\n" if merged else ""), encoding="utf-8")

    del cls_model

    print(f"\n完成: {len(frames)} 帧")
    print(f"  mine 边缘过滤移除: {total_edge_removed}")
    print(f"  mine_cls_v5 保留: {total_cls_kept}, 移除: {total_cls_removed}")

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
