"""训练第二阶段：爆炸物卡片内容分类器（Ultralytics 分类，薄封装）。

输入：build_card_dataset.py 输出的「文件夹即类别」数据集
（train/<子类>/*.jpg、val/<子类>/*.jpg）。模型默认 yolo11s-cls，裁剪图
内容是印刷照片、类别不多，s 级足够；分类比检测简单，CPU/Orin 推理都快。

用法：
    python tools/ml/train_card_classifier.py --data datasets/card_cls
    python tools/ml/train_card_classifier.py --data ... --model yolo11n-cls.pt
训练完成自动导出 ONNX（供 dzuav/车端第二阶段推理）。
退出码：0 成功；2 依赖；3 产物缺失。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--data", required=True)
    parser.add_argument("--model", default="yolo11s-cls.pt")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=160)
    parser.add_argument("--batch", type=int, default=128)
    parser.add_argument("--device", default="0")
    parser.add_argument("--project", default="runs/cls")
    parser.add_argument("--name", default="card_content")
    args = parser.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        print("缺少 ultralytics：python -m pip install ultralytics", file=sys.stderr)
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
        patience=30,
        cos_lr=True,
        exist_ok=True,
    )

    run_dir = Path(results.save_dir) if hasattr(results, "save_dir") else \
        Path(args.project) / args.name
    best = run_dir / "weights" / "best.pt"
    if not best.exists():
        print(f"未找到 {best}", file=sys.stderr)
        return 3

    export = YOLO(str(best))
    onnx_path = export.export(format="onnx", imgsz=args.imgsz,
                              simplify=True, opset=12)
    print("=" * 60)
    print(f"训练完成。best.pt: {best}")
    print(f"ONNX: {onnx_path}")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
