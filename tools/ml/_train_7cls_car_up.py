# -*- coding: utf-8 -*-
# ⚠️ 留档脚本：7cls/旧线，2026-10-16 起已切换 D13 十三类统一模型（hbb_d13_best.pt），仅供追溯，勿用于现役训练/评估 ⚠️
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
