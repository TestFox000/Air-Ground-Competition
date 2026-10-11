# -*- coding: utf-8 -*-
"""官方 74 张精确标注 IoU 对照评估：YOLO v1 vs v2（X-AnyLabeling 逐雷精确口径）。"""
import sys

from ultralytics import YOLO

RUNS = (r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\runs\detect\runs\detect")
DATA = r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\runs\eval_official74\data.yaml"

for tag, name in (("v1", "aerial_field_finetuned"), ("v2", "aerial_field_finetuned_v2")):
    m = YOLO(rf"{RUNS}\{name}\weights\best.pt")
    r = m.val(data=DATA, imgsz=1280, conf=0.25, iou=0.45, batch=8,
              workers=0, device=0, plots=False, verbose=False, amp=False)
    print(f"[{tag}] mine  P={r.box.mp:.4f}  R={r.box.mr:.4f}  "
          f"mAP50={r.box.map50:.4f}  mAP50-95={r.box.map:.4f}", flush=True)
sys.exit(0)
