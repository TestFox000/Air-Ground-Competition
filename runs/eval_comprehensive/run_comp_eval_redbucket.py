# -*- coding: utf-8 -*-
"""多检测器融合综合评估：7cls_final_v2 + red_bucket检测器补充obstacle召回
目标：降低 obstacle FN=30（当前召回74.82%），提升省赛综合识别精度
策略：
  1. 用7cls_final_v2基线预测（conf=0.25）
  2. 额外运行red_bucket检测器，对不与已有obstacle重叠(IoU<0.5)的红桶检测补充为obstacle
  3. 评估综合精度
"""
import os, sys
from pathlib import Path
import numpy as np
from PIL import Image
from ultralytics import YOLO

REPO = r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground"
VAL_IMG = rf"{REPO}\capture\datasets\aerial_field\images\val"
OUT = rf"{REPO}\runs\eval_comprehensive"

BASELINE_PRED = rf"{OUT}\pred_7cls_final_v2_4090"
RED_BUCKET = r"E:\ml_datasets\downloaded\red_bucket_best.onnx"

NAMES7 = ["mine", "car", "arrow", "target_red", "cell_label", "target_blue", "obstacle"]
OBSTACLE_CLS = 6

def load_yolo_lines(path):
    if not path.exists():
        return []
    lines = []
    for line in path.read_text().strip().split("\n"):
        if not line.strip():
            continue
        p = line.split()
        cls = int(p[0])
        cx, cy, w, h = map(float, p[1:5])
        conf = float(p[5]) if len(p) > 5 else 1.0
        lines.append((cls, cx, cy, w, h, conf))
    return lines

def iou_xywh(a, b):
    ax1, ay1 = a[0]-a[2]/2, a[1]-a[3]/2
    ax2, ay2 = a[0]+a[2]/2, a[1]+a[3]/2
    bx1, by1 = b[0]-b[2]/2, b[1]-b[3]/2
    bx2, by2 = b[0]+b[2]/2, b[1]+b[3]/2
    ix = max(0, min(ax2,bx2)-max(ax1,bx1))
    iy = max(0, min(ay2,by2)-max(ay1,by1))
    inter = ix*iy
    return inter / (a[2]*a[3] + b[2]*b[3] - inter + 1e-9)

def merge_red_bucket(baseline_dir, red_bucket_model, img_dir, out_dir, rb_conf=0.25):
    """合并red_bucket检测到baseline预测中"""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    rb_det = YOLO(red_bucket_model)

    img_files = sorted(Path(img_dir).glob("*.jpg"))
    for img_path in img_files:
        stem = img_path.stem
        # 加载基线预测
        base = load_yolo_lines(Path(baseline_dir) / f"{stem}.txt")
        # 运行red_bucket检测器
        results = rb_det(str(img_path), imgsz=1280, conf=rb_conf, iou=0.45,
                         verbose=False, device=0, max_det=500)[0]
        boxes = results.boxes
        img = Image.open(img_path).convert("RGB")
        W, H = img.size
        added = 0
        if len(boxes) > 0:
            xyxy = boxes.xyxy.cpu().numpy()
            confs = boxes.conf.cpu().numpy()
            for box, c in zip(xyxy, confs):
                x1, y1, x2, y2 = box
                cx = (x1 + x2) / 2 / W
                cy = (y1 + y2) / 2 / H
                w = (x2 - x1) / W
                h = (y2 - y1) / H
                rb_box = (cx, cy, w, h)
                # 检查是否与已有obstacle重叠
                overlap = False
                for b in base:
                    if b[0] == OBSTACLE_CLS:
                        if iou_xywh(rb_box, (b[1], b[2], b[3], b[4])) >= 0.5:
                            overlap = True
                            break
                if not overlap:
                    base.append((OBSTACLE_CLS, cx, cy, w, h, float(c)))
                    added += 1
        # 保存
        lines = [f"{c} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f} {cf:.6f}" for c, cx, cy, w, h, cf in base]
        (out / f"{stem}.txt").write_text("\n".join(lines) + "\n" if lines else "")
    del rb_det

def run_eval_comprehensive(pred_dir, gt_dir, img_dir, names, exclude="car"):
    sys.path.insert(0, rf"{REPO}\tools\ml")
    from eval_comprehensive import main as eval_main
    sys.argv = ["eval_comprehensive.py",
                "--pred-dir", pred_dir,
                "--gt-dir", gt_dir,
                "--img-dir", img_dir,
                "--names", ",".join(names),
                "--exclude", exclude]
    return eval_main()

if __name__ == "__main__":
    gt_7dir = rf"{OUT}\gt_7cls"

    print(f"{'='*60}\n基线: 7cls_final_v2 单阶段 conf=0.25\n{'='*60}")
    run_eval_comprehensive(BASELINE_PRED, gt_7dir, VAL_IMG, NAMES7, exclude="car")

    for rb_c in [0.15, 0.25, 0.35]:
        tag = f"7cls_final_v2_redbucket_rbconf{rb_c}"
        out_dir = rf"{OUT}\pred_{tag}"
        print(f"\n{'='*60}\n融合: 7cls_final_v2 + red_bucket(rb_conf={rb_c})\n{'='*60}")
        merge_red_bucket(BASELINE_PRED, RED_BUCKET, VAL_IMG, out_dir, rb_conf=rb_c)
        run_eval_comprehensive(out_dir, gt_7dir, VAL_IMG, NAMES7, exclude="car")
