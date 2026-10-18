"""训练自研移动靶 YOLO11-pose 模型（Ultralytics 薄封装，不重造训练框架）。

为什么选 YOLO11-pose：官方模型是 YOLOv8-pose 一代架构；YOLO11 在同参数量下
骨干/颈部更高效（C3k2 + C2PSA），同量级精度与速度均优于 v8 —— 这是
「比官方性能更好」的合理赌注，且输出仍是标准 pose 布局，后续转 RKNN 的
工具链完全复用。

用法：
    pip install ultralytics                      # 首次
    python tools/ml/train_pose.py --data datasets/field/data.yaml
    python tools/ml/train_pose.py --data datasets/field/data.yaml \
        --model yolo11s-pose.pt --epochs 150 --imgsz 640 --batch 16
训练完成后自动导出 ONNX（固定输入形状，供 RKNN 转换）。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--data", required=True, help="build_dataset 生成的 data.yaml")
    parser.add_argument("--model", default="yolo11s-pose.pt",
                        help="预训练权重（默认 yolo11s-pose.pt；算力紧张用 yolo11n-pose.pt）")
    parser.add_argument("--epochs", type=int, default=150)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--device", default="0", help="0=GPU；cpu=无显卡时")
    parser.add_argument("--project", default="runs/pose")
    parser.add_argument("--name", default="move_target")
    args = parser.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        print("缺少 ultralytics，请先执行: python -m pip install ultralytics",
              file=sys.stderr)
        return 2

    model = YOLO(args.model)
    results = model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        project=args.project,
        name=args.name,
        # 小数据集的稳妥默认：早停 + 适度增强；不开启过强的 mosaic 混标
        patience=30,
        mosaic=0.5,
        close_mosaic=10,
        cos_lr=True,
        exist_ok=True,
    )

    run_dir = Path(results.save_dir) if hasattr(results, "save_dir") else \
        Path(args.project) / args.name
    best = run_dir / "weights" / "best.pt"
    if not best.exists():
        print(f"未找到 {best}，请检查训练输出", file=sys.stderr)
        return 3

    # 导出固定形状 ONNX（RKNN 转换需要静态输入）
    export = YOLO(str(best))
    onnx_path = export.export(format="onnx", imgsz=args.imgsz, simplify=True, opset=12)
    print("=" * 60)
    print(f"训练完成。best.pt: {best}")
    print(f"ONNX: {onnx_path}")
    print("下一步（香橙派）：tools/ml/rknn_convert.py 转 .rknn")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
