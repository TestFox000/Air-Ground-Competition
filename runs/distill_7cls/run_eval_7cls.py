# -*- coding: utf-8 -*-
"""蒸馏 7 类模型 vs 厂家机载现役模型，双口径对照评估。

口径 1: 官方 74 张精确标注（runs/eval_official74，mine 人工真值）
口径 2: 7cls 数据集 val（43 帧 = 35 场地帧 + 8 官方图）
        - mine/cell_label/arrow 为人工真值，可直接对照厂家
        - obstacle/car 的 GT 含厂家伪标注（自标自评，指标仅表示"与厂家的一致率"）
注意: 训练结束后先跑本脚本，达标后再 release_models.py --hbb 发布。
"""
import sys

from ultralytics import YOLO

REPO = r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground"
OFFICIAL = rf"{REPO}\runs\eval_official74\data.yaml"
VAL7 = r"E:\ml_datasets\aerial_field_7cls\data.yaml"
FACTORY = rf"{REPO}\models\release\hbb_detector.onnx"
DISTILL = rf"{REPO}\runs\detect\runs\detect\runs\detect\aerial_field_7cls\weights\best.pt"

NAMES7 = {0: "mine", 1: "car", 2: "arrow", 3: "target_red",
          4: "cell_label", 5: "target_blue", 6: "obstacle"}

for tag, path in (("厂家hbb(机载现役)", FACTORY), ("7cls蒸馏(本次)", DISTILL)):
    m = YOLO(path)
    print(f"\n===== {tag} =====", flush=True)
    r = m.val(data=OFFICIAL, imgsz=1280, conf=0.25, iou=0.45, batch=4,
              workers=0, device=0, plots=False, verbose=False, amp=False)
    print(f"[官方74精确] mine  P={r.box.mp:.4f}  R={r.box.mr:.4f}  "
          f"mAP50={r.box.map50:.4f}  mAP50-95={r.box.map:.4f}", flush=True)
    r2 = m.val(data=VAL7, split="val", imgsz=1280, conf=0.25, iou=0.45, batch=4,
               workers=0, device=0, plots=False, verbose=False, amp=False)
    print(f"[7cls val 43帧] all  P={r2.box.mp:.4f}  R={r2.box.mr:.4f}  "
          f"mAP50={r2.box.map50:.4f}", flush=True)
    per = r2.box.maps  # 每类 mAP50-95；p/r 需从 results dict 取
    for c in range(7):
        key = f"metrics/maps_{c}(B)"
        mp = r2.results_dict.get(f"metrics/precision_{c}(B)", float("nan"))
        print(f"    cls{c} {NAMES7[c]:<11} mAP50-95={per[c]:.4f}", flush=True)
    # 每类 P/R 从 confusion 或者直接再跑单类……ultralytics 不直接给每类 P/R，用 ap50 近似参考
sys.exit(0)
