# -*- coding: utf-8 -*-
"""定位场上蓝靶/红靶的实际外观：用厂家低置信检出框位置裁剪 + HSV 蓝色分析。"""
import json
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(r"E:/OneDrive - mails.qust.edu.cn/桌面/air_ground")
DATASET = ROOT / "capture/datasets/aerial_field_mixed"
OUT = ROOT / "runs/distill_7cls"

records = json.load(open(ROOT / "runs/distill_7cls/_factory_boxes.json", encoding="utf-8"))
NAMES = ["mine", "car", "arrow", "target_red", "cell_label", "target_blue", "obstacle"]

# 找出场地帧上的靶检出（conf 0.2+，不管厂家置信度多低，位置线索即可）
for name, boxes in sorted(records.items()):
    if name.startswith("explosive"):
        continue
    for cls, conf, xyxy in boxes:
        if cls in (3, 5):
            cx, cy = (xyxy[0] + xyxy[2]) / 2, (xyxy[1] + xyxy[3]) / 2
            print(f"{NAMES[cls]:<12} {name} conf={conf:.3f} center=({cx:.0f},{cy:.0f}) "
                  f"size={xyxy[2]-xyxy[0]:.0f}x{xyxy[3]-xyxy[1]:.0f}")

# 裁剪 frame_00106 的 target_blue 框附近区域（放大 4 倍看靶长什么样）
cands = [(n, c, x) for n, bs in records.items() for c, x in
         [(b[0], b[2]) for b in bs if b[0] == 5]]
print("\n蓝靶框:", cands)
for i, (name, xyxy) in enumerate([(n, x) for n, _, x in cands][:4]):
    img = cv2.imdecode(np.fromfile(str(DATASET / "images/train" / name), np.uint8), cv2.IMREAD_COLOR)
    H, W = img.shape[:2]
    x1, y1, x2, y2 = [int(v) for v in xyxy]
    pad = 120
    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
    r = max(x2 - x1, y2 - y1) // 2 + pad
    crop = img[max(0, cy - r):min(H, cy + r), max(0, cx - r):min(W, cx + r)]
    crop = cv2.resize(crop, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
    p = OUT / f"_crop_blue_{i}_{name}"
    cv2.imencode(".jpg", crop, [cv2.IMWRITE_JPEG_QUALITY, 92])[1].tofile(str(p))
    print("saved:", p)

# 全图 HSV 蓝色检测：看场上蓝色元素的分布（靶 vs 格号文字 vs 其他）
img = cv2.imdecode(np.fromfile(str(DATASET / "images/train/frame_00106.jpg"), np.uint8), cv2.IMREAD_COLOR)
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
for tag, lo, hi in [("蓝(H100-135)", (100, 80, 60), (135, 255, 255)), ("深蓝文字(H90-115)", (90, 40, 30), (115, 200, 200))]:
    mask = cv2.inRange(hsv, lo, hi)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    areas = sorted([(s[4], s[0], s[1], s[2], s[3]) for s in stats[1:]], reverse=True)[:12]
    print(f"\n[{tag}] 连通域 top12 (area,x,y,w,h):")
    for a in areas:
        print(f"   area={a[0]:<7} x={a[1]:<5} y={a[2]:<5} w={a[3]:<4} h={a[4]}")
