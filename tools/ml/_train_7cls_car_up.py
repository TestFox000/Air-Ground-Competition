# -*- coding: utf-8 -*-
"""car 上采样版训练（car=738，仅复制真实样本，不引入假阳性）。
从第一轮 best.pt 继续训（car P=0.914 已能识别，上采样提升召回）。
"""
from ultralytics import YOLO

DATA = r"E:/ml_datasets/aerial_field_7cls_car_upsampled/data.yaml"
PRETRAINED = r"runs/detect/runs/detect/runs/detect/aerial_field_7cls_local/weights/best.pt"

model = YOLO(PRETRAINED)
model.train(
    data=DATA,
    imgsz=1280,
    batch=2,
    workers=0,
    amp=False,
    device=0,
    epochs=40,
    patience=15,
    project=r"runs/detect/runs/detect",
    name="aerial_field_7cls_car_up",
    exist_ok=True,
    seed=42,
    verbose=True,
)
