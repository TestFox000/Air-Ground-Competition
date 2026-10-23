# -*- coding: utf-8 -*-
"""本地 7cls 训练（RTX 4060 8GB）。
数据 = aerial_field_7cls_supplemented（car=82，各数据集中最多）。
预训练 = aerial_field_finetuned_v2/best.pt（航拍域骨干，nc=11→7 自动重建 cls 头）。
8GB 显存：batch 2 / imgsz 1280 / workers 0 / amp False。
"""
from ultralytics import YOLO
import sys

DATA = r"E:/ml_datasets/aerial_field_7cls_supplemented/data.yaml"
PRETRAINED = r"runs/detect/runs/detect/aerial_field_finetuned_v2/weights/best.pt"

model = YOLO(PRETRAINED)
model.train(
    data=DATA,
    imgsz=1280,
    batch=2,
    workers=0,
    amp=False,
    device=0,
    epochs=60,
    patience=25,
    project=r"runs/detect/runs/detect",
    name="aerial_field_7cls_local",
    exist_ok=True,
    seed=42,
    verbose=True,
)
