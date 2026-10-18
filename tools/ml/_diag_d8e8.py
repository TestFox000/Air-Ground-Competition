# -*- coding: utf-8 -*-
"""frame_00170（全场视野+格号真值）裁剪 D8-E8 轨道区域，确认移动靶是否在训练视频中存在。"""
import json
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(r"E:/OneDrive - mails.qust.edu.cn/桌面/air_ground")
GT = ROOT / "runs/cell_recog_render/label/frame_00170"
img = cv2.imdecode(np.fromfile(str(ROOT / "capture/datasets/aerial_field_mixed/images/val/frame_00170.jpg"), np.uint8), cv2.IMREAD_COLOR)
H, W = img.shape[:2]
boxes = json.load(open(GT / "boxes.json", encoding="utf-8"))
labels = json.load(open(GT / "labels_final.json", encoding="utf-8"))

cells = {}
for i, b in enumerate(boxes["boxes"]):
    key = str(i)
    if key in labels and labels[key].get("cell_id"):
        cells[labels[key]["cell_id"]] = b["cx"], b["cy"]

print("有格号真值的格:", sorted(cells.keys())[:80])
for c in ["C8", "D8", "E8", "F8", "D7", "E7", "D9", "E9", "C7", "F7", "C9", "F9"]:
    print(f"  {c}: {cells.get(c)}")

# 用 D8..F9 的范围裁剪（靶轨道 D8<->E8，靶可能移动到任何位置，裁大点）
xs, ys = [], []
for c in ["C7", "C8", "C9", "D7", "D8", "D9", "E7", "E8", "E9", "F7", "F8", "F9"]:
    if c in cells:
        xs.append(cells[c][0]); ys.append(cells[c][1])
if xs:
    x1, x2 = max(0, int(min(xs) - 150)), min(W, int(max(xs) + 150))
    y1, y2 = max(0, int(min(ys) - 200)), min(H, int(max(ys) + 200))
    crop = img[y1:y2, x1:x2].copy()
    for c, (cx, cy) in cells.items():
        if c[0] in "CDEF" and c[1] in "789":
            px, py = int(cx - x1), int(cy - y1)
            cv2.circle(crop, (px, py), 4, (0, 255, 255), -1)
            cv2.putText(crop, c, (px + 4, py - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    out = ROOT / "runs/distill_7cls/_crop_D8E8_frame_00170.jpg"
    cv2.imencode(".jpg", crop, [cv2.IMWRITE_JPEG_QUALITY, 92])[1].tofile(str(out))
    print("saved:", out, f"crop=({x1},{y1})-({x2},{y2})")
else:
    print("无 C7-F9 格号真值，改裁场地中央")
