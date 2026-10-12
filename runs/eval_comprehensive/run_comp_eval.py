# -*- coding: utf-8 -*-
"""综合识别精度评估：7cls模型预测 + GT标签映射 + eval_comprehensive.py
评估省赛口径：类别+格位(IoU>=0.5)全对才算正确
"""
import os, sys, shutil
from pathlib import Path
import numpy as np
import cv2
from ultralytics import YOLO

REPO = r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground"
VAL_IMG = rf"{REPO}\capture\datasets\aerial_field\images\val"
VAL_LBL = rf"{REPO}\capture\datasets\aerial_field\labels\val"
OUT = rf"{REPO}\runs\eval_comprehensive"

# 7cls模型类别
NAMES7 = ["mine", "car", "arrow", "target_red", "cell_label", "target_blue", "obstacle"]

# GT 11类 -> 7cls映射
# 0:mine->0, 1:cell_label->4, 2:arrow->2, 3:bridge->6(obstacle),
# 4:car_start->1(car), 5:car_end->1(car), 6:uav_start->6(obstacle),
# 7:uav_end->6(obstacle), 8:bucket_red->6, 9:bucket_green->6, 10:bucket_cyan->6
GT_TO_7 = {0:0, 1:4, 2:2, 3:6, 4:1, 5:1, 6:6, 7:6, 8:6, 9:6, 10:6}

def map_gt_labels(src_dir, dst_dir):
    """将11类GT标签映射为7类"""
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
                new_cls = GT_TO_7[cls]
                lines.append(f"{new_cls} {p[1]} {p[2]} {p[3]} {p[4]}")
        (dst / f.name).write_text("\n".join(lines) + "\n" if lines else "")

def predict(model_path, img_dir, pred_dir, conf=0.25):
    """用YOLO模型预测，保存YOLO格式txt"""
    pred = Path(pred_dir)
    pred.mkdir(parents=True, exist_ok=True)
    model = YOLO(model_path)
    results = model.predict(source=img_dir, imgsz=1280, conf=conf, iou=0.45,
                            save_txt=True, save_conf=False, verbose=False,
                            project=str(pred), name="preds", exist_ok=True)
    # ultralytics saves to pred/preds/labels/, move up
    label_dir = pred / "preds" / "labels"
    if label_dir.exists():
        for f in label_dir.glob("*.txt"):
            shutil.move(str(f), str(pred / f.name))
        shutil.rmtree(pred / "preds", ignore_errors=True)

def run_eval_comprehensive(pred_dir, gt_dir, img_dir, names, exclude="car"):
    """调用eval_comprehensive.py"""
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
    # 1. 映射GT标签到7类
    gt_7dir = rf"{OUT}\gt_7cls"
    map_gt_labels(VAL_LBL, gt_7dir)
    print(f"[1/3] GT标签已映射到7类: {gt_7dir}")

    models = {
        "distill_published": rf"{REPO}\models\release\hbb_detector_7cls_distill_20261011.onnx",
        "7cls_clean_4090": rf"E:\ml_datasets\downloaded\4090_batch2\7cls_clean_best.onnx",
        "7cls_final_v2_4090": rf"E:\ml_datasets\downloaded\4090_batch2\7cls_final_v2_best.onnx",
    }
    for tag, mpath in models.items():
        if not os.path.exists(mpath):
            print(f"\n[SKIP] {tag}: {mpath} not found")
            continue
        print(f"\n{'='*60}")
        print(f"评估模型: {tag}")
        print(f"{'='*60}")
        pred_dir = rf"{OUT}\pred_{tag}"
        predict(mpath, VAL_IMG, pred_dir, conf=0.25)
        print(f"[2/3] 预测完成: {pred_dir}")
        print(f"[3/3] 综合识别精度:")
        run_eval_comprehensive(pred_dir, gt_7dir, VAL_IMG, NAMES7, exclude="car")
