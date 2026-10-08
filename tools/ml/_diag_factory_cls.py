# -*- coding: utf-8 -*-
"""诊断：厂家 7 类模型在航拍数据集上的各类检出统计。
目的：决定蒸馏伪标注的类别与置信度阈值（靶/车/障碍类自建标注没有，全靠厂家伪标注）。"""
import collections
from pathlib import Path

from ultralytics import YOLO

ROOT = Path(r"E:/OneDrive - mails.qust.edu.cn/桌面/air_ground")
DATASET = ROOT / "capture/datasets/aerial_field_mixed"
FACTORY = ROOT / "models/release/hbb_detector.onnx"

# C++ className 顺序：0 mine/1 car/2 arrow/3 target_red/4 cell_label/5 target_blue/6 obstacle
NAMES = ["mine", "car", "arrow", "target_red", "cell_label", "target_blue", "obstacle"]

model = YOLO(str(FACTORY))
stats = collections.defaultdict(lambda: collections.defaultdict(list))  # split -> cls -> [conf]
for split in ["train", "val"]:
    images = sorted((DATASET / "images" / split).glob("*.jpg"))
    print(f"[{split}] {len(images)} images")
    for img in images:
        result = model.predict(str(img), imgsz=1280, conf=0.2, verbose=False, device=0)[0]
        for box in result.boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            stats[split][cls].append(conf)

for split in ["train", "val"]:
    print(f"\n===== {split} =====")
    for cls in sorted(stats[split]):
        confs = sorted(stats[split][cls])
        n = len(confs)
        # 置信度分档：>=0.5 / >=0.35 / >=0.25
        hi = sum(c >= 0.5 for c in confs)
        mid = sum(c >= 0.35 for c in confs)
        print(f"  {NAMES[cls]:<12} total={n:<5} conf>=0.5:{hi:<5} >=0.35:{mid:<5} "
              f"median={confs[n // 2]:.3f} max={confs[-1]:.3f}")
