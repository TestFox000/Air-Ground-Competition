# -*- coding: utf-8 -*-
"""两阶段综合识别精度评估：YOLO粗定位 + mine_cls_v5雷分类器二次确认
目标：降低 mine 类 FP（当前FP=119），提升省赛综合识别精度（距95%差4.76pp）

策略：
  1. YOLO(7cls_final_v2) 用低conf(0.15)检测，提高mine召回
  2. 对所有mine类候选框裁剪后送入 mine_cls_v5 分类器
  3. 仅保留 mine_prob >= cls_thresh 的框（融合置信度 = yolo_conf * mine_prob）
  4. 非mine类保持原样
  5. 按省赛口径（类别+IoU>=0.5全对）评估综合精度
"""
import os, sys, shutil
from pathlib import Path
import numpy as np
from PIL import Image
from ultralytics import YOLO

REPO = r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground"
VAL_IMG = rf"{REPO}\capture\datasets\aerial_field\images\val"
VAL_LBL = rf"{REPO}\capture\datasets\aerial_field\labels\val"
OUT = rf"{REPO}\runs\eval_comprehensive"

DETECTOR = rf"E:\ml_datasets\downloaded\4090_batch2\7cls_final_v2_best.onnx"
CLASSIFIER = rf"E:\ml_datasets\downloaded\4090_batch2\mine_cls_v5_best.onnx"

NAMES7 = ["mine", "car", "arrow", "target_red", "cell_label", "target_blue", "obstacle"]
GT_TO_7 = {0:0, 1:4, 2:2, 3:6, 4:1, 5:1, 6:6, 7:6, 8:6, 9:6, 10:6}

def map_gt_labels(src_dir, dst_dir):
    dst = Path(dst_dir)
    dst.mkdir(parents=True, exist_ok=True)
    for f in Path(src_dir).glob("*.txt"):
        lines = []
        for line in f.read_text().strip().split("\n"):
            if not line.strip():
                continue
            p = line.split()
            cls = int(p[0])
            if cls in GT_TO_7:
                lines.append(f"{GT_TO_7[cls]} {p[1]} {p[2]} {p[3]} {p[4]}")
        (dst / f.name).write_text("\n".join(lines) + "\n" if lines else "")

def predict_two_stage(detector, classifier, img_dir, pred_dir,
                      det_conf=0.15, cls_thresh=0.5):
    """两阶段检测：YOLO低conf + 分类器过滤mine假阳性"""
    pred = Path(pred_dir)
    pred.mkdir(parents=True, exist_ok=True)

    det = YOLO(detector)
    cls_model = YOLO(classifier)

    img_files = sorted(Path(img_dir).glob("*.jpg"))
    for img_path in img_files:
        img = Image.open(img_path).convert("RGB")
        W, H = img.size
        results = det(np.array(img), imgsz=1280, conf=det_conf, iou=0.45,
                      verbose=False, device=0, max_det=1000)[0]
        boxes = results.boxes
        lines = []
        if len(boxes) > 0:
            xyxy = boxes.xyxy.cpu().numpy()
            confs = boxes.conf.cpu().numpy()
            clss = boxes.cls.cpu().numpy().astype(int)
            for box, c, s in zip(xyxy, confs, clss):
                x1, y1, x2, y2 = box
                cx = (x1 + x2) / 2 / W
                cy = (y1 + y2) / 2 / H
                w = (x2 - x1) / W
                h = (y2 - y1) / H
                if s == 0:  # mine类 -> 分类器二次确认
                    xi1, yi1 = max(0, int(x1)), max(0, int(y1))
                    xi2, yi2 = min(W, int(x2)), min(H, int(y2))
                    if xi2 - xi1 < 5 or yi2 - yi1 < 5:
                        continue
                    crop = img.crop((xi1, yi1, xi2, yi2)).resize((96, 96))
                    cls_res = cls_model(np.array(crop), verbose=False, device=0)[0]
                    probs = cls_res.probs.data.cpu().numpy()
                    mine_prob = probs[0] if cls_model.names[0] == "mine" else probs[1]
                    if mine_prob < cls_thresh:
                        continue  # 分类器判定为非雷，过滤
                    # 融合置信度
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
    os.makedirs(OUT, exist_ok=True)
    gt_7dir = rf"{OUT}\gt_7cls"
    if not Path(gt_7dir).exists():
        map_gt_labels(VAL_LBL, gt_7dir)
    print(f"[1/3] GT标签已映射到7类: {gt_7dir}")

    # 基线：单阶段 conf=0.25
    print(f"\n{'='*60}")
    print(f"基线: 7cls_final_v2 单阶段 conf=0.25")
    print(f"{'='*60}")
    base_dir = rf"{OUT}\pred_7cls_final_v2_4090"
    run_eval_comprehensive(base_dir, gt_7dir, VAL_IMG, NAMES7, exclude="car")

    # 两阶段：不同分类器阈值
    for cls_t in [0.3, 0.5, 0.7]:
        tag = f"7cls_final_v2_two_stage_cls{cls_t}"
        pred_dir = rf"{OUT}\pred_{tag}"
        print(f"\n{'='*60}")
        print(f"两阶段: 7cls_final_v2 det_conf=0.15 + mine_cls_v5 cls_thresh={cls_t}")
        print(f"{'='*60}")
        predict_two_stage(DETECTOR, CLASSIFIER, VAL_IMG, pred_dir,
                          det_conf=0.15, cls_thresh=cls_t)
        run_eval_comprehensive(pred_dir, gt_7dir, VAL_IMG, NAMES7, exclude="car")
