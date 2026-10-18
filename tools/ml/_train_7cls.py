# -*- coding: utf-8 -*-
# ⚠️ 留档脚本：7cls/旧线，2026-10-16 起已切换 D13 十三类统一模型（hbb_d13_best.pt），仅供追溯，勿用于现役训练/评估 ⚠️
"""蒸馏 7 类模型训练：官方 7 类本体（与 C++ className 契约一致）。
数据 = 航拍人工真值(mine/cell_label/arrow/bucket→obstacle) + 厂家伪标注(car/target_red/obstacle) + 官方74精确雷。
从 v2 best.pt 起训（骨干已航拍域适配，nc=11→7 时 ultralytics 自动重建 cls 头、迁移骨干+box 头；
不走 GitHub 下载 yolo11s.pt——本环境访问卡顿）。
8GB 显存适配：batch 4（batch 8 在 TaskAlignedAssigner OOM）/ imgsz 1280 / workers 0 / amp False。"""
from ultralytics import YOLO

model = YOLO(r"runs/detect/runs/detect/aerial_field_finetuned_v2/weights/best.pt")
model.train(
    data=r"E:/ml_datasets/aerial_field_7cls/data.yaml",
    imgsz=1280,
    batch=4,
    workers=0,
    amp=False,
    device=0,
    epochs=75,
    patience=30,
    project=r"runs/detect/runs/detect",
    name="aerial_field_7cls",
    exist_ok=True,
    seed=42,
    verbose=True,
)
