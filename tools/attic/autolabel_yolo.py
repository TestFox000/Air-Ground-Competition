# -*- coding: utf-8 -*-
"""YOLO 自动预标注：用训练好的检测模型对图片批量推理，生成 YOLO 格式标签。

用法:
  python tools/autolabel_yolo.py --weights runs/detect/.../best.pt --source runs/capture_dataset/images --out runs/capture_dataset/labels --conf 0.25

  # 只保留特定类别（如只要 mine 和 cell_label）
  python tools/autolabel_yolo.py --weights ... --source ... --classes 0,1

  # 指定类别数（自动从模型读取，也可手动覆盖）
  python tools/autolabel_yolo.py --weights ... --source ... --nc 11
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from ultralytics import YOLO


def main() -> int:
    ap = argparse.ArgumentParser(description="YOLO 自动预标注")
    ap.add_argument("--weights", required=True, help="YOLO 权重路径 (.pt)")
    ap.add_argument("--source", required=True, help="图片目录")
    ap.add_argument("--out", required=True, help="标签输出目录")
    ap.add_argument("--conf", type=float, default=0.25, help="置信度阈值")
    ap.add_argument("--iou", type=float, default=0.45, help="NMS IoU 阈值")
    ap.add_argument("--imgsz", type=int, default=1280, help="推理尺寸")
    ap.add_argument("--classes", default="", help="只保留的类别 id，逗号分隔，如 0,1")
    ap.add_argument("--nc", type=int, default=0, help="类别数（0=从模型自动读取）")
    ap.add_argument("--device", default="0", help="推理设备")
    args = ap.parse_args()

    src = Path(args.source)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    classes = set()
    if args.classes:
        classes = set(int(x) for x in args.classes.split(","))

    model = YOLO(args.weights)
    nc = args.nc or model.model.nc
    print(f"模型: {args.weights}, 类别数={nc}, 只保留类别={classes or '全部'}")

    exts = {".jpg", ".jpeg", ".png", ".bmp"}
    images = sorted(p for p in src.iterdir() if p.suffix.lower() in exts)
    print(f"待标注图片: {len(images)} 张")

    n_labeled = 0
    n_empty = 0
    for i, img_path in enumerate(images):
        results = model.predict(
            source=str(img_path),
            conf=args.conf,
            iou=args.iou,
            imgsz=args.imgsz,
            device=args.device,
            verbose=False,
        )

        label_lines = []
        for r in results:
            if r.boxes is None or len(r.boxes) == 0:
                continue
            boxes = r.boxes
            for j in range(len(boxes)):
                cls_id = int(boxes.cls[j].item())
                if classes and cls_id not in classes:
                    continue
                # xywh 归一化
                x, y, w, h = boxes.xywhn[j].tolist()
                label_lines.append(f"{cls_id} {x:.6f} {y:.6f} {w:.6f} {h:.6f}")

        label_path = out / f"{img_path.stem}.txt"
        with open(label_path, "w", encoding="utf-8") as f:
            f.write("\n".join(label_lines))

        if label_lines:
            n_labeled += 1
        else:
            n_empty += 1

        if (i + 1) % 50 == 0:
            print(f"  [{i+1}/{len(images)}] 有标签={n_labeled}, 空标签={n_empty}")

    print(f"\n标注完成: {n_labeled} 张有标签, {n_empty} 张空标签")
    print(f"标签目录: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
