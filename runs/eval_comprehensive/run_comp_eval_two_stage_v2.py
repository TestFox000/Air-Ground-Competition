# -*- coding: utf-8 -*-
"""两阶段综合识别精度评估 v2：保持det_conf=0.25基线 + 仅mine类用分类器过滤
验证 mine_cls_v5 对航拍域 mine FP 的过滤效果
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

def predict_two_stage_v2(detector, classifier, img_dir, pred_dir,
                         det_conf=0.25, cls_thresh=0.5):
    """保持det_conf=0.25，仅对mine类候选框用分类器过滤"""
    pred = Path(pred_dir)
    pred.mkdir(parents=True, exist_ok=True)

    det = YOLO(detector)
    cls_model = YOLO(classifier)

    img_files = sorted(Path(img_dir).glob("*.jpg"))
    for img_path in img_files:
        # 用文件路径做检测（与基线一致，避免np.array像素差异）
        results = det(str(img_path), imgsz=1280, conf=det_conf, iou=0.45,
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

    # 基线
    print(f"{'='*60}\n基线: 7cls_final_v2 单阶段 conf=0.25\n{'='*60}")
    run_eval_comprehensive(rf"{OUT}\pred_7cls_final_v2_4090", gt_7dir, VAL_IMG, NAMES7, exclude="car")

    # 两阶段 v2: det_conf=0.25 + 分类器过滤
    for cls_t in [0.5, 0.7, 0.9]:
        tag = f"7cls_final_v2_2stage_v2_cls{cls_t}"
        pred_dir = rf"{OUT}\pred_{tag}"
        print(f"\n{'='*60}\n两阶段v2: det_conf=0.25 + mine_cls_v5 cls_thresh={cls_t}\n{'='*60}")
        predict_two_stage_v2(DETECTOR, CLASSIFIER, VAL_IMG, pred_dir,
                             det_conf=0.25, cls_thresh=cls_t)
        run_eval_comprehensive(pred_dir, gt_7dir, VAL_IMG, NAMES7, exclude="car")
