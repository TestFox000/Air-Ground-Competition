# -*- coding: utf-8 -*-
# ⚠️ 留档脚本：7cls/旧线，2026-10-16 起已切换 D13 十三类统一模型（hbb_d13_best.pt），仅供追溯，勿用于现役训练/评估 ⚠️
"""在 val-A (capture/datasets/aerial_field) 上评估 combined 模型，与 91.07% 基线公平对比。

用法:
    python tools/ml/_eval_combined_on_valA.py
"""
import subprocess
import sys
from pathlib import Path

from ultralytics import YOLO

REPO = Path(__file__).resolve().parents[2]
PYTHON = r"C:\Users\a\venvs\air_ground\Scripts\python.exe"

COMBINED_PT = REPO / "runs" / "detect" / "runs" / "detect" / "runs" / "detect" / "aerial_field_7cls_combined" / "weights" / "best.pt"
VAL_IMG = Path(r"capture/datasets/aerial_field/images/val")
VAL_GT = Path(r"capture/datasets/aerial_field/labels/val")
PRED_DIR = REPO / "runs" / "detect" / "runs" / "detect" / "runs" / "detect" / "aerial_field_7cls_combined" / "pred_valA_combined" / "labels"

NAMES = "mine,car,arrow,target_red,cell_label,target_blue,obstacle"
EXCLUDE = "car"


def main():
    if not COMBINED_PT.exists():
        print(f"combined best.pt 不存在: {COMBINED_PT}", file=sys.stderr)
        return 1

    PRED_DIR.mkdir(parents=True, exist_ok=True)

    print(f"加载 combined 模型: {COMBINED_PT}")
    model = YOLO(str(COMBINED_PT))

    img_files = sorted(VAL_IMG.glob("*.jpg"))
    print(f"预测 {len(img_files)} 帧 ...")
    for img_path in img_files:
        results = model(str(img_path), imgsz=1280, conf=0.25, iou=0.45,
                        verbose=False, device=0, max_det=1000)[0]
        from PIL import Image
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
                lines.append(f"{s} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f} {float(c):.6f}")
        (PRED_DIR / f"{img_path.stem}.txt").write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")

    del model

    # 运行 eval_comprehensive.py
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
