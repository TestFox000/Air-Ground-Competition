# -*- coding: utf-8 -*-
"""本地训练脚本：obstacle_aug 数据集（RTX 4060 8GB）。

针对 M0-3 省赛95%硬指标的两大瓶颈：
  1. obstacle FN=30（四模型共漏，纯数据域差异）
  2. arrow 召回低（val 11GT仅检出2，召回18%）

数据集：aerial_field_7cls_obstacle_aug（1716 train / 34 val）
  - 原始216 train + mine CP 500 + obstacle CP 500 + arrow CP 500
  - obstacle=6087 / arrow=1812 / mine=56859（合并后类别分布）

基座：7cls蒸馏版 best.pt（mAP50=0.7879，当前综合精度91.07%）
策略：低学习率微调 + close_mosaic，避免 catastrophic forgetting

本地配置：RTX 4060 8GB，batch=2，imgsz=1280，amp=False，workers=0
"""
from ultralytics import YOLO

# 本地路径
DATA = r"E:/ml_datasets/aerial_field_7cls_obstacle_aug/data.yaml"
PRETRAINED = r"runs/detect/runs/detect/runs/detect/aerial_field_7cls/weights/best.pt"

model = YOLO(PRETRAINED)
model.train(
    data=DATA,
    imgsz=1280,
    batch=2,             # 4060 8GB
    workers=0,           # Windows 本地
    amp=False,           # 4060 8GB 防OOM
    device=0,
    epochs=80,
    patience=20,
    # 微调超参：低学习率防止遗忘标靶/雷
    lr0=0.003,
    lrf=0.01,
    momentum=0.937,
    weight_decay=0.0005,
    warmup_epochs=3,
    warmup_momentum=0.8,
    warmup_bias_lr=0.05,
    box=7.5,
    cls=0.5,              # 降低分类损失权重，避免过拟合小样本
    dfl=1.5,
    label_smoothing=0.0,
    nbs=64,
    # 数据增强：保留适度增强，最后10epoch关mosaic
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
    close_mosaic=10,      # 最后10epoch关闭mosaic，提升最终精度
    auto_augment="randaugment",
    erasing=0.4,
    project=r"runs/detect",
    name="aerial_field_7cls_obstacle_aug",
    exist_ok=True,
    seed=42,
    verbose=True,
)
