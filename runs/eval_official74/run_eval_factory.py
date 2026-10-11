# -*- coding: utf-8 -*-
"""厂家出厂 hbb_detector.onnx（机载现役 7 类）在两个口径下 vs 自训 v2（11 类）对照。

口径 1: 官方 74 张精确标注（runs/eval_official74）
口径 2: 航拍场地 val 帧（aerial_field_mixed val split, 43 帧）
"""
import sys

from ultralytics import YOLO

REPO = r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground"
OFFICIAL = rf"{REPO}\runs\eval_official74\data.yaml"
AERIAL = rf"{REPO}\capture\datasets\aerial_field_mixed\data.yaml"
FACTORY = rf"{REPO}\models\release\hbb_detector.onnx"
V2 = (rf"{REPO}\runs\detect\runs\detect\aerial_field_finetuned_v2\weights\best.pt")

for tag, model in (("厂家hbb(机载现役)", FACTORY), ("v2自训(彩排用)", V2)):
    m = YOLO(model)
    print(f"\n===== {tag} =====", flush=True)
    r = m.val(data=OFFICIAL, imgsz=1280, conf=0.25, iou=0.45, batch=8,
              workers=0, device=0, plots=False, verbose=False, amp=False)
    print(f"[官方74精确] mine  P={r.box.mp:.4f}  R={r.box.mr:.4f}  "
          f"mAP50={r.box.map50:.4f}  mAP50-95={r.box.map:.4f}", flush=True)
    r2 = m.val(data=AERIAL, imgsz=1280, conf=0.25, iou=0.45, batch=8,
               workers=0, device=0, plots=False, verbose=False, amp=False)
    print(f"[航拍val]    mine  P={r2.box.mp:.4f}  R={r2.box.mr:.4f}  "
          f"mAP50={r2.box.map50:.4f}  mAP50-95={r2.box.map:.4f}", flush=True)
sys.exit(0)
