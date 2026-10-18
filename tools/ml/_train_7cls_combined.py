# -*- coding: utf-8 -*-
# ⚠️ 留档脚本：7cls/旧线，2026-10-16 起已切换 D13 十三类统一模型（hbb_d13_best.pt），仅供追溯，勿用于现役训练/评估 ⚠️
"""合并数据集训练（2087图，186 car）。
从预训练模型训，更多数据 + 更多 car 样本。
"""
from ultralytics import YOLO

DATA = r"E:/ml_datasets/aerial_field_7cls_combined/data.yaml"
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
    lr0=0.01,
    lrf=0.01,
    momentum=0.937,
    weight_decay=0.0005,
    warmup_epochs=3,
    warmup_momentum=0.8,
    warmup_bias_lr=0.1,
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
    mosaic=1.0,
    mixup=0.0,
    copy_paste=0.0,
    project=r"runs/detect/runs/detect",
    name="aerial_field_7cls_combined",
    exist_ok=True,
    seed=42,
    verbose=True,
)
