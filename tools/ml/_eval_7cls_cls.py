# -*- coding: utf-8 -*-
"""评估 7cls 模型各类 mAP，重点看 car 类。"""
from ultralytics import YOLO
import sys

MODEL = sys.argv[1] if len(sys.argv) > 1 else r"runs/detect/runs/detect/runs/detect/aerial_field_7cls_local/weights/best.pt"
DATA = r"E:/ml_datasets/aerial_field_7cls_supplemented/data.yaml"


def main():
    model = YOLO(MODEL)
    metrics = model.val(data=DATA, imgsz=1280, batch=2, device=0, workers=0, verbose=False)

    names = model.names
    ap50_per_class = metrics.box.ap50  # 各类 mAP50
    ap_per_class = metrics.box.ap      # 各类 mAP50-95
    p_per_class = metrics.box.p if hasattr(metrics.box, 'p') else [0]*len(names)
    r_per_class = metrics.box.r if hasattr(metrics.box, 'r') else [0]*len(names)

    print(f"\n{'='*60}")
    print(f"模型: {MODEL}")
    print(f"{'='*60}")
    print(f"{'类别':<15} {'P':>8} {'R':>8} {'mAP50':>8} {'mAP50-95':>10}")
    print("-" * 55)
    for i, name in names.items():
        p = p_per_class[i] if i < len(p_per_class) else 0
        r = r_per_class[i] if i < len(r_per_class) else 0
        a50 = ap50_per_class[i] if i < len(ap50_per_class) else 0
        a95 = ap_per_class[i] if i < len(ap_per_class) else 0
        print(f"{name:<15} {p:>8.4f} {r:>8.4f} {a50:>8.4f} {a95:>10.4f}")

    print(f"{'='*60}")
    print(f"整体 mAP50={metrics.box.map50:.4f}  mAP50-95={metrics.box.map:.4f}")


if __name__ == '__main__':
    main()
