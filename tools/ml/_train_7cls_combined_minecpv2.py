# -*- coding: utf-8 -*-
"""combined + mine_cp_v2 微调训练（3087 train）。

从 combined best.pt (val-B综合90.53%, obstacle FN=4, arrow召回100%) 微调，
追加 mine_cp_v2 (1000张双域) 专门解决 mine FN=38 瓶颈。
低学习率 + close_mosaic 防止 obstacle/arrow 灾难性遗忘。

4060配置: batch=2 / imgsz=1280 / workers=0 / amp=False
"""
from ultralytics import YOLO

DATA = r"E:/ml_datasets/aerial_field_7cls_combined_minecpv2/data.yaml"
PRETRAINED = r"runs/detect/runs/detect/runs/detect/aerial_field_7cls_combined/weights/best.pt"

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
    lr0=0.003,          # 低学习率微调，防遗忘
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
    close_mosaic=10,     # 最后10epoch关mosaic，稳定mine召回
    project=r"runs/detect/runs/detect/runs/detect",
    name="aerial_field_7cls_combined_minecpv2",
    exist_ok=True,
    seed=42,
    verbose=True,
)
