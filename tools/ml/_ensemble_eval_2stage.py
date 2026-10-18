# -*- coding: utf-8 -*-
"""2stage 集成检测评估：combined(obstacle+arrow+其他) + final_v2(mine w/ mine_cls_v5)。

策略（服务 M0-3 省赛95%硬指标）：
- obstacle(cls=6) + arrow(cls=2) + cell_label(cls=4) + target_red(cls=3) + target_blue(cls=5)
  取自 combined 预测（combined 根治了 obstacle FN=30 和 arrow 召回低）
- mine(cls=0) 取自 final_v2 预测，经 mine_cls_v5 分类器二次确认(cls_thresh=0.9)
  + 1% 边缘过滤（final_v2+mine_cls_v5 即 91.07% 基线的 mine 方案）
- car 排除

用法:
    python tools/ml/_ensemble_eval_2stage.py
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
FINALV2_LABELS = RUN_DIR / "pred_val_final_v2" / "labels"
MERGED_DIR = RUN_DIR / "pred_val_ensemble_2stage" / "labels"

VAL_IMG = Path(r"E:/ml_datasets/aerial_field_7cls/images/val")
VAL_GT = Path(r"E:/ml_datasets/aerial_field_7cls/labels/val")
CLASSIFIER = Path(r"E:/ml_datasets/downloaded/4090_batch2/mine_cls_v5_best.onnx")

NAMES = "mine,car,arrow,target_red,cell_label,target_blue,obstacle"
EXCLUDE = "car"

# mine 边缘过滤：中心点距边缘 < 1% 则移除
EDGE_THR = 0.01
MINE_CLS = 0
CLS_THRESH = 0.9  # mine_cls_v5 分类阈值（与 91.07% 基线一致）

NON_MINE_CLASSES = {2, 3, 4, 5, 6}  # arrow, target_red, cell_label, target_blue, obstacle


def apply_mine_edge_filter(lines, edge_thr=0.01, mine_cls=0):
    """过滤中心点距图像边缘 < edge_thr 的 mine 检测框。"""
    kept = []
    removed = 0
    for line in lines:
        parts = line.strip().split()
        if not parts:
            continue
        cls = int(parts[0])
        cx, cy = float(parts[1]), float(parts[2])
        if cls == mine_cls and (cx < edge_thr or cx > 1 - edge_thr or cy < edge_thr or cy > 1 - edge_thr):
            removed += 1
            continue
        kept.append(line.strip())
    return kept, removed


def main():
    if not COMBINED_LABELS.exists():
        print(f"combined 预测不存在: {COMBINED_LABELS}", file=sys.stderr)
        return 1
    if not FINALV2_LABELS.exists():
        print(f"final_v2 预测不存在: {FINALV2_LABELS}", file=sys.stderr)
        return 1
    if not CLASSIFIER.exists():
        print(f"mine_cls_v5 模型不存在: {CLASSIFIER}", file=sys.stderr)
        return 1

    MERGED_DIR.mkdir(parents=True, exist_ok=True)

    # 加载 mine_cls_v5 分类器
    print(f"加载 mine_cls_v5 分类器: {CLASSIFIER}")
    cls_model = YOLO(str(CLASSIFIER))
    mine_idx = 0 if cls_model.names[0] == "mine" else 1
    print(f"  mine 类索引 = {mine_idx}, names = {cls_model.names}")

    total_edge_removed = 0
    total_cls_kept = 0
    total_cls_removed = 0
    frames = sorted(FINALV2_LABELS.glob("*.txt"))
    print(f"处理 {len(frames)} 帧 ...")

    for f in frames:
        stem = f.stem
        merged = []

        # combined 预测：取 obstacle(6) + arrow(2) + cell_label(4) + target_red(3) + target_blue(5)
        comb_path = COMBINED_LABELS / f.name
        if comb_path.exists():
            comb_lines = comb_path.read_text().strip().split("\n")
            for line in comb_lines:
                parts = line.strip().split()
                if not parts:
                    continue
                cls = int(parts[0])
                if cls in NON_MINE_CLASSES:
                    merged.append(line.strip())

        # final_v2 预测：取 mine(0)，应用边缘过滤 + mine_cls_v5 分类器
        fv2_lines = f.read_text().strip().split("\n")
        mine_lines = [l for l in fv2_lines if l.strip() and int(l.strip().split()[0]) == MINE_CLS]
        mine_kept, edge_removed = apply_mine_edge_filter(mine_lines, EDGE_THR, MINE_CLS)
        total_edge_removed += edge_removed

        # 对每个 mine 候选框用分类器二次确认
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
                # 保留（原预测无 conf 列，写 6 列带 fused=mine_prob 以便追溯）
                merged.append(f"{MINE_CLS} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f} {float(mine_prob):.6f}")
                total_cls_kept += 1
        elif mine_kept:
            # 无原图则直接保留（不应发生）
            merged.extend(mine_kept)
            total_cls_kept += len(mine_kept)

        out = MERGED_DIR / f.name
        out.write_text("\n".join(merged) + ("\n" if merged else ""), encoding="utf-8")

    del cls_model

    print(f"\n集成完成: {len(frames)} 帧")
    print(f"  mine 边缘过滤移除: {total_edge_removed} 个框")
    print(f"  mine_cls_v5 保留: {total_cls_kept}, 移除: {total_cls_removed}")
    print(f"合并标签 → {MERGED_DIR}")

    # 运行 eval_comprehensive.py
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
