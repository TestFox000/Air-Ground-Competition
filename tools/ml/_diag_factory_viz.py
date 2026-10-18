# -*- coding: utf-8 -*-
"""可视化：厂家模型在航拍帧上的 obstacle/car/靶 检出内容确认。
挑 car 置信度最高帧、obstacle 最多帧、target_red 最高帧各画一张。"""
import json
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

ROOT = Path(r"E:/OneDrive - mails.qust.edu.cn/桌面/air_ground")
DATASET = ROOT / "capture/datasets/aerial_field_mixed"
FACTORY = ROOT / "models/release/hbb_detector.onnx"
OUT = ROOT / "runs/distill_7cls"

NAMES = ["mine", "car", "arrow", "target_red", "cell_label", "target_blue", "obstacle"]
COLORS = [(0, 0, 255), (255, 0, 255), (0, 165, 255), (0, 0, 255), (0, 255, 255), (255, 0, 0), (128, 0, 128)]

model = YOLO(str(FACTORY))
frames = sorted((DATASET / "images/train").glob("*.jpg"))

# 第一遍：全量推理记录各类最高置信帧
best = {}  # cls -> (conf, path, n_extra)
records = {}  # path -> results boxes
for img in frames:
    r = model.predict(str(img), imgsz=1280, conf=0.2, verbose=False, device=0)[0]
    boxes = [(int(c), float(f), [float(v) for v in b]) for c, f, b in
             zip(r.boxes.cls, r.boxes.conf, r.boxes.xyxy)]
    records[img.name] = boxes
    per_cls = {}
    for c, f, _ in boxes:
        per_cls.setdefault(c, []).append(f)
    for c, confs in per_cls.items():
        top = max(confs)
        n = len(confs)
        cur = best.get(c)
        if c in (1, 3, 5) and (cur is None or top > cur[0]):   # car/靶：最高置信帧
            best[c] = (top, img.name, n)
        if c == 6 and (cur is None or n > cur[2]):             # obstacle：最多帧
            best[6] = (top, img.name, n)

print("best frames:", {NAMES[c]: v[1] for c, v in best.items()})

OUT.mkdir(parents=True, exist_ok=True)  # 推迟到首次写文件前建目录，import 期无副作用
with open(OUT / "_factory_boxes.json", "w") as fp:
    json.dump(records, fp)

# 第二遍：画图
for c, (conf, name, n) in best.items():
    if c not in (1, 3, 5, 6):
        continue
    img = cv2.imdecode(np.fromfile(str(DATASET / "images/train" / name), np.uint8), cv2.IMREAD_COLOR)
    for cls, conf_, xyxy in records[name]:
        if cls in (0, 4):   # 雷和格号不画（太多），只画要确认的类 + arrow 参考
            continue
        x1, y1, x2, y2 = xyxy
        cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), COLORS[cls], 3)
        cv2.putText(img, f"{NAMES[cls]} {conf_:.2f}", (int(x1), max(20, int(y1) - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLORS[cls], 2)
    out_path = OUT / f"_viz_{NAMES[c]}_{name}"
    cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tofile(str(out_path))
    print("saved:", out_path)
