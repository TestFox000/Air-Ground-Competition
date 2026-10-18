# -*- coding: utf-8 -*-
"""导出第一轮 best.pt 为 ONNX。"""
from ultralytics import YOLO

model = YOLO(r"runs/detect/runs/detect/runs/detect/aerial_field_7cls_local/weights/best.pt")
model.export(
    format="onnx",
    imgsz=1280,
    simplify=True,
    opset=12,
)
print("ONNX 导出完成")
