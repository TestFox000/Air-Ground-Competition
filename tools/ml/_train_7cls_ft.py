# -*- coding: utf-8 -*-
"""从第一轮 best.pt 微调：极低学习率 + 关闭 mosaic，避免破坏已有性能。
目标：提升 car 召回率，同时保持其他类别精度。
"""
from ultralytics import YOLO

DATA = r"E:/ml_datasets/aerial_field_7cls_supplemented/data.yaml"
PRETRAINED = r"runs/detect/runs/detect/runs/detect/aerial_field_7cls_local/weights/best.pt"

model = YOLO(PRETRAINED)
model.train(
    data=DATA,
    imgsz=1280,
    batch=2,
    workers=0,
    amp=False,
    device=0,
    epochs=30,
    patience=15,
    lr0=0.001,        # 极低学习率微调
    lrf=0.01,
    momentum=0.937,
    weight_decay=0.0005,
    warmup_epochs=2,
    warmup_momentum=0.8,
    warmup_bias_lr=0.05,
    box=7.5,
    cls=0.5,
    dfl=1.5,
    hsv_h=0.015,
    hsv_s=0.7,
    hsv_v=0.4,
    degrees=0.0,
    translate=0.1,
    scale=0.5,
    shear=0.0,
    perspective=0.0,
    flipud=0.0,
    fliplr=0.5,
    mosaic=0.0,       # 关闭 mosaic，避免小目标被缩小
    mixup=0.0,
    copy_paste=0.0,
    project=r"runs/detect/runs/detect",
    name="aerial_field_7cls_ft",
    exist_ok=True,
    seed=42,
    verbose=True,
)
