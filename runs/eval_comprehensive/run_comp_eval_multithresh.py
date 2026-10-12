# -*- coding: utf-8 -*-
"""多阈值两阶段综合评估：det_conf=0.15 + 分类别后处理
策略：
  - mine: conf>=0.15 + mine_cls_v5(cls_thresh=0.9) 过滤FP
  - obstacle: conf>=0.15 最大化召回
  - cell_label/arrow/others: conf>=0.25 控制FP
目标：降低 obstacle FN=30 同时控制其他类FP
"""
import os, sys
from pathlib import Path
import numpy as np
from PIL import Image
from ultralytics import YOLO

REPO = r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground"
VAL_IMG = rf"{REPO}\capture\datasets\aerial_field\images\val"
OUT = rf"{REPO}\runs\eval_comprehensive"

DETECTOR = rf"E:\ml_datasets\downloaded\4090_batch2\7cls_final_v2_best.onnx"
CLASSIFIER = rf"E:\ml_datasets\downloaded\4090_batch2\mine_cls_v5_best.onnx"

NAMES7 = ["mine", "car", "arrow", "target_red", "cell_label", "target_blue", "obstacle"]
# 分类别conf阈值: mine和obstacle用低阈值(0.15)，其他用0.25
CLASS_CONF = {0: 0.15, 1: 0.25, 2: 0.25, 3: 0.25, 4: 0.25, 5: 0.25, 6: 0.15}

def predict_multi_thresh(detector, classifier, img_dir, pred_dir, cls_thresh=0.9):
    pred = Path(pred_dir)
    pred.mkdir(parents=True, exist_ok=True)
    det = YOLO(detector)
    cls_model = YOLO(classifier)

    img_files = sorted(Path(img_dir).glob("*.jpg"))
    for img_path in img_files:
        results = det(str(img_path), imgsz=1280, conf=0.15, iou=0.45,
                      verbose=False, device=0, max_det=1000)[0]
        img = Image.open(img_path).convert("RGB")
        W, H = img.size
        boxes = results.boxes
        lines = []
        if len(boxes) > 0:
            xyxy = boxes.xyxy.cpu().numpy()
            confs = boxes.conf.cpu().numpy()
            clss = boxes.cls.cpu().numpy().astype(int)
            for box, c, s in zip(xyxy, confs, clss):
                # 分类别conf阈值
                if c < CLASS_CONF.get(s, 0.25):
                    continue
                x1, y1, x2, y2 = box
                cx = (x1 + x2) / 2 / W
                cy = (y1 + y2) / 2 / H
                w = (x2 - x1) / W
                h = (y2 - y1) / H
                if s == 0:  # mine -> 分类器
                    xi1, yi1 = max(0, int(x1)), max(0, int(y1))
                    xi2, yi2 = min(W, int(x2)), min(H, int(y2))
                    if xi2 - xi1 < 5 or yi2 - yi1 < 5:
                        continue
                    crop = img.crop((xi1, yi1, xi2, yi2)).resize((96, 96))
                    cls_res = cls_model(np.array(crop), verbose=False, device=0)[0]
                    probs = cls_res.probs.data.cpu().numpy()
                    mine_prob = probs[0] if cls_model.names[0] == "mine" else probs[1]
                    if mine_prob < cls_thresh:
                        continue
                    fused = float(c) * float(mine_prob)
                    lines.append(f"0 {cx:.6f} {cy:.6f} {w:.6f} {h:.6f} {fused:.6f}")
                else:
                    lines.append(f"{s} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f} {float(c):.6f}")
        (pred / f"{img_path.stem}.txt").write_text("\n".join(lines) + "\n" if lines else "")
    del det, cls_model

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
    run_eval_comprehensive(rf"{OUT}\pred_7cls_final_v2_4090", gt_7dir, VAL_IMG, NAMES7, exclude="car")

    tag = "7cls_final_v2_multithresh_mine015_obs015_other025"
    pred_dir = rf"{OUT}\pred_{tag}"
    print(f"\n{'='*60}\n多阈值: mine/obstacle conf>=0.15, 其他>=0.25, mine+cls_v5(0.9)\n{'='*60}")
    predict_multi_thresh(DETECTOR, CLASSIFIER, VAL_IMG, pred_dir, cls_thresh=0.9)
    run_eval_comprehensive(pred_dir, gt_7dir, VAL_IMG, NAMES7, exclude="car")
