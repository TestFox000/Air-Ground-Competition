# -*- coding: utf-8 -*-
"""本地 7cls 训练（car 增强版）：用 aerial_field_7cls_car_aug 数据集（car=2104）。
从上一轮 best.pt 继续训，重点提升 car 类。
"""
from ultralytics import YOLO

DATA = r"E:/ml_datasets/aerial_field_7cls_car_aug/data.yaml"
PRETRAINED = r"runs/detect/runs/detect/runs/detect/aerial_field_7cls_local/weights/best.pt"

model = YOLO(PRETRAINED)
model.train(
    data=DATA,
    imgsz=1280,
    batch=2,
    workers=0,
    amp=False,
    device=0,
    epochs=50,
    patience=20,
    project=r"runs/detect/runs/detect",
    name="aerial_field_7cls_car_aug",
    exist_ok=True,
    seed=42,
    verbose=True,
)
