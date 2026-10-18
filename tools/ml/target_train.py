# -*- coding: utf-8 -*-
"""标靶识别模型训练脚本 — YOLO-pose，对齐车端C++契约 [1,9,8400]

车端期望输出: 4 bbox + 2 cls(blue=0, red=1) + 3 kpt(x, y, conf) = 9通道

数据格式:
    target_dataset/
    ├── images/{train,val}/
    └── labels/{train,val}/  每行: cls cx cy w h kx ky 2  (2=可见关键点)

用法:
    python target_train.py --data E:\\ml_datasets\\target_dataset --epochs 100
"""
import argparse
from pathlib import Path
from ultralytics import YOLO


def build_yaml(dataset_dir: Path) -> Path:
    """生成data.yaml（pose任务需指定kpt_shape和flip_idx）。"""
    yaml = dataset_dir / "data.yaml"
    content = (
        f"path: {dataset_dir.as_posix()}\n"
        "train: images/train\n"
        "val: images/val\n"
        "nc: 2\n"
        "names:\n"
        "  0: blue\n"
        "  1: red\n"
        "kpt_shape: [1, 3]\n"        # 1个关键点，3个值(x,y,conf)
        "flip_idx: [0]\n"             # 关键点翻转索引
    )
    yaml.write_text(content, encoding="utf-8")
    return yaml


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=r"E:\ml_datasets\target_dataset")
    ap.add_argument("--model", default="yolo11n-pose.pt",
                    help="预训练pose权重")
    ap.add_argument("--epochs", type=int, default=100)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--device", default="0")
    ap.add_argument("--out", default=r"e:\OneDrive - mails.qust.edu.cn\桌面\air_ground\runs\target_pose")
    args = ap.parse_args()

    data_dir = Path(args.data)
    yaml = build_yaml(data_dir)

    m = YOLO(args.model)
    results = m.train(
        data=str(yaml),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=0,
        amp=False,                       # 4060 8GB显存安全
        patience=30,
        seed=42,
        project=args.out,
        name="train",
        # 标靶是小目标，加强相关增强
        scale=0.5,
        fliplr=0.5,
        mosaic=1.0,
        close_mosaic=10,
        hsv_h=0.015,
        hsv_s=0.5,
        hsv_v=0.3,
    )
    print(f"\n训练完成: best={results.best}")
    # 导出ONNX验证契约
    best_pt = Path(args.out) / "train" / "weights" / "best.pt"
    if best_pt.exists():
        print(f"\n导出ONNX验证契约...")
        m2 = YOLO(str(best_pt))
        onnx = m2.export(format="onnx", imgsz=args.imgsz, opset=12, simplify=True)
        print(f"ONNX: {onnx}")


if __name__ == "__main__":
    main()
