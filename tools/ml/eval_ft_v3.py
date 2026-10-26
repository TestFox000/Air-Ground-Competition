# -*- coding: utf-8 -*-
"""ft_v3 门禁评估（训练完成后运行）：
  G1 官方爆炸物图级雷召回——留出57(诚实口径) + 全157(连续口径)，ft_v3/ft_v1/d13 三模型同集对照
  G2 data_14 val 逐类 mAP50 —— ft_v3 对 ft_v1 不回退（val 集跨版一致）
  G3 红面 168 帧包 —— 在 ONNX 导出后另跑 eval_ft_video.py，不在本脚本内

用法: python eval_ft_v3.py [best.pt 路径]
产出: gates_ft_v3.json + 控制台报告
"""
from __future__ import annotations

import json
import os
import sys

import cv2
import numpy as np

WORK = r"C:\Users\29212\Documents\Trae-CN\default\ft_v3_official_20261026"
MANIFEST = os.path.join(WORK, "manifest.json")
DATA14_YAML = r"C:\Users\29212\Documents\Trae-CN\default\data_14_20261025\data_14.yaml"
FTV1_PT = r"C:\Users\29212\Documents\Trae-CN\default\data_14_20261025\runs\ft_v1\weights\best.pt"
D13_PT = r"C:\Users\29212\Documents\Trae-CN\default\dk_train_pack_20261016\models\hbb_d13_best.pt"
GATE_CONF = 0.25

OFFICIAL_DIR = (r"D:\人工智能算法精英大赛-空地协同侦排挑战赛"
                r"\手雷图片+通行通道图片\1-爆炸物\B-爆炸物")


def official_recall(model, files):
    hit = 0
    for f in files:
        img = cv2.imdecode(np.fromfile(os.path.join(OFFICIAL_DIR, f),
                                       dtype=np.uint8), cv2.IMREAD_COLOR)
        r = model.predict(img, conf=GATE_CONF, classes=[0], imgsz=1280,
                          device=0, verbose=False)[0]
        if len(r.boxes):
            hit += 1
    return hit, len(files)


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    ft3_pt = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        WORK, "runs", "ft_v3", "weights", "best.pt")
    man = json.load(open(MANIFEST, encoding="utf-8"))
    holdout = man["holdout"]
    all_files = sorted(man["train"] + holdout)
    from ultralytics import YOLO

    out = {"models": {}}
    for tag, pt in (("ft_v3", ft3_pt), ("ft_v1", FTV1_PT), ("d13", D13_PT)):
        m = YOLO(pt)
        h57, n57 = official_recall(m, holdout)
        f157, n157 = official_recall(m, all_files)
        v = m.val(data=DATA14_YAML, imgsz=1280, batch=6, device=0,
                  verbose=False, plots=False)
        ap50 = {v.names[int(c)]: round(float(v.box.all_ap[i][0]), 4)
                for i, c in enumerate(v.box.ap_class_index)}
        out["models"][tag] = {
            "official_holdout57": {"hit": h57, "n": n57, "recall": round(h57 / n57, 4)},
            "official_full157": {"hit": f157, "n": n157, "recall": round(f157 / n157, 4)},
            "map50": round(float(v.box.map50), 4),
            "map50_95": round(float(v.box.map), 4),
            "ap50_per_class": ap50,
        }
        print(f"{tag}: 留出57 {h57}/{n57}={h57/n57:.1%} | 全157 {f157}/{n157}={f157/n157:.1%} "
              f"| mAP50 {v.box.map50:.4f} | mAP50-95 {v.box.map:.4f}", flush=True)

    with open(os.path.join(WORK, "gates_ft_v3.json"), "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
    print("gates_ft_v3.json 写出")


if __name__ == "__main__":
    main()
