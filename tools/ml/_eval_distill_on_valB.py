# -*- coding: utf-8 -*-
"""蒸馏模型 (aerial_field_7cls, mAP50=0.7879) 在 val-B 上评估。

用法:
    python tools/ml/_eval_distill_on_valB.py
"""
import subprocess
import sys
from pathlib import Path

from PIL import Image
from ultralytics import YOLO

REPO = Path(__file__).resolve().parents[2]
PYTHON = r"C:\Users\a\venvs\air_ground\Scripts\python.exe"

DISTILL_PT = REPO / "runs" / "detect" / "runs" / "detect" / "runs" / "detect" / "aerial_field_7cls" / "weights" / "best.pt"
VAL_IMG = Path(r"E:/ml_datasets/aerial_field_7cls/images/val")
VAL_GT = Path(r"E:/ml_datasets/aerial_field_7cls/labels/val")
PRED_DIR = REPO / "runs" / "detect" / "runs" / "detect" / "runs" / "detect" / "aerial_field_7cls" / "pred_valB" / "labels"

NAMES = "mine,car,arrow,target_red,cell_label,target_blue,obstacle"
EXCLUDE = "car"

EDGE_THR = 0.01
MINE_CLS = 0


def main():
    PRED_DIR.mkdir(parents=True, exist_ok=True)
    print(f"加载蒸馏模型: {DISTILL_PT}")
    model = YOLO(str(DISTILL_PT))

    img_files = sorted(VAL_IMG.glob("*.jpg"))
    print(f"预测 {len(img_files)} 帧 (conf=0.25) ...")
    edge_removed = 0
    for img_path in img_files:
        results = model(str(img_path), imgsz=1280, conf=0.25, iou=0.45,
                        verbose=False, device=0, max_det=2000)[0]
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
                if s == MINE_CLS and (cx < EDGE_THR or cx > 1 - EDGE_THR or cy < EDGE_THR or cy > 1 - EDGE_THR):
                    edge_removed += 1
                    continue
                lines.append(f"{s} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f} {float(c):.6f}")
        (PRED_DIR / f"{img_path.stem}.txt").write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")
    del model
    print(f"mine 边缘过滤移除: {edge_removed}")

    eval_script = REPO / "tools" / "ml" / "eval_comprehensive.py"
    cmd = [
        PYTHON, str(eval_script),
        "--pred-dir", str(PRED_DIR),
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
