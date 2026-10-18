# -*- coding: utf-8 -*-
"""frame_00106 全帧缩略：厂家靶框(绿) + 人工格号框(黄) + 人工桶框(紫红)，看靶框落在场地哪个格。"""
import json
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(r"E:/OneDrive - mails.qust.edu.cn/桌面/air_ground")
DATASET = ROOT / "capture/datasets/aerial_field_mixed"
records = json.load(open(ROOT / "runs/distill_7cls/_factory_boxes.json", encoding="utf-8"))

for fname in ["frame_00106.jpg", "frame_00116.jpg"]:
    img = cv2.imdecode(np.fromfile(str(DATASET / "images/train" / fname), np.uint8), cv2.IMREAD_COLOR)
    H, W = img.shape[:2]
    scale = 1280 / W
    small = cv2.resize(img, (1280, int(H * scale)))
    # 人工标签
    lbl = DATASET / "labels/train" / (Path(fname).stem + ".txt")
    for line in open(lbl, encoding="utf-8"):
        parts = line.split()
        if not parts:
            continue
        cls = int(parts[0])
        if cls == 1:   # cell_label
            cx, cy, w, h = [float(v) for v in parts[1:5]]
            x1, y1 = (cx - w / 2) * W, (cy - h / 2) * H
            p1 = (int(x1 * scale), int(y1 * scale))
            p2 = (int((x1 + w * W) * scale), int((y1 + h * H) * scale))
            cv2.rectangle(small, p1, p2, (0, 255, 255), 1)
        elif cls in (8, 9, 10):   # 桶
            cx, cy, w, h = [float(v) for v in parts[1:5]]
            x1, y1 = (cx - w / 2) * W, (cy - h / 2) * H
            p1 = (int(x1 * scale), int(y1 * scale))
            p2 = (int((x1 + w * W) * scale), int((y1 + h * H) * scale))
            cv2.rectangle(small, p1, p2, (128, 0, 128), 2)
    # 厂家靶/car 框
    for cls, conf, xyxy in records[fname]:
        if cls in (1, 3, 5):
            p1 = (int(xyxy[0] * scale), int(xyxy[1] * scale))
            p2 = (int(xyxy[2] * scale), int(xyxy[3] * scale))
            cv2.rectangle(small, p1, p2, (0, 255, 0), 2)
            cv2.putText(small, f"tgt/car {conf:.2f}", (p1[0], max(20, p1[1] - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    out = ROOT / "runs/distill_7cls" / f"_full_{Path(fname).stem}.jpg"
    cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 88])[1].tofile(str(out))
    print("saved:", out)
