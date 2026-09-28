#!/usr/bin/env python3
"""YOLOv8-pose 训练包装（需要 ultralytics + torch，见 requirements-train.txt）。

    python tools/ml/train_pose.py --data /path/data.yaml --epochs 120 --batch 16

开训前会做三件**必须**的检查，任何一条不过就直接退出（宁可不训，也不能训出个上不了车的模型）：
    1. data.yaml 里的 names 顺序必须是 0=blue_target、1=red_target
    2. kpt_shape 必须是 [1, 3]
    3. 数据集必须先过 tools/ml/dataset_check.py

训练结束后自动在 val 上跑一次，并把 best.pt 路径打印出来供 export_rknn.py 使用。
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ML = Path(__file__).resolve().parent

#: 与车端 postprocess.h 绑死的契约
EXPECT_NAMES = ("blue_target", "red_target")
EXPECT_KPT_SHAPE = (1, 3)


def load_yaml(path: Path) -> dict:
    try:
        import yaml
    except ImportError:
        print("需要 PyYAML：pip install pyyaml", file=sys.stderr)
        raise SystemExit(2)
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def check_data_yaml(cfg: dict) -> None:
    names = cfg.get("names")
    if isinstance(names, dict):
        pairs = sorted((int(k), str(v)) for k, v in names.items())
        ordered = tuple(v for _, v in pairs)
    elif isinstance(names, list):
        ordered = tuple(names)
    else:
        raise SystemExit("data.yaml 里缺少 names")

    if tuple(n.lower() for n in ordered) != EXPECT_NAMES:
        raise SystemExit(
            "类别顺序不对！必须是 0=blue_target(敌方) 1=red_target(友方)，"
            f"当前是 {ordered}。\n"
            "顺序反了会把友方当敌方打（每次 −5 分，最多 −15）。"
        )

    kpt = tuple(cfg.get("kpt_shape") or ())
    if kpt != EXPECT_KPT_SHAPE:
        raise SystemExit(
            f"kpt_shape 必须是 {list(EXPECT_KPT_SHAPE)}（车端 KPT_NUM=1、KPT_DIM=3），"
            f"当前是 {list(kpt)}"
        )
    print("[OK] data.yaml 契约检查通过（2 类 / 1 关键点，类别顺序正确）")


def run_dataset_check(data_yaml: Path) -> None:
    """训练前强制体检数据集。"""
    path = data_yaml.parent if data_yaml.parent.name != "" else data_yaml
    base = load_yaml(data_yaml).get("path")
    target = Path(base).expanduser() if base else path
    cmd = [sys.executable, str(ML / "dataset_check.py"), "--root", str(target)]
    print(f"\n$ {' '.join(cmd)}")
    r = subprocess.run(cmd)
    if r.returncode != 0:
        raise SystemExit("数据集体检未通过，先修数据再训练（--skip-check 可跳过，不建议）")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="YOLOv8-pose 训练（带车端契约检查）")
    ap.add_argument("--data", required=True, help="data.yaml 路径")
    ap.add_argument("--model", default="yolov8n-pose.pt", help="预训练权重")
    ap.add_argument("--epochs", type=int, default=120)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--device", default="0", help="GPU 编号，'cpu' 也行")
    ap.add_argument("--project", default=str(ROOT / "runs" / "pose"))
    ap.add_argument("--name", default="air_ground_pose")
    ap.add_argument("--skip-check", action="store_true", help="跳过数据集体检（不建议）")
    ap.add_argument("--dry-run", action="store_true", help="只做检查不训练")
    args = ap.parse_args(argv)

    data_yaml = Path(args.data).expanduser().resolve()
    if not data_yaml.is_file():
        raise SystemExit(f"找不到 {data_yaml}")
    check_data_yaml(load_yaml(data_yaml))
    if not args.skip_check:
        run_dataset_check(data_yaml)
    if args.dry_run:
        print("\n[dry-run] 检查全部通过，未开始训练")
        return 0

    try:
        from ultralytics import YOLO
    except ImportError:
        print("需要 ultralytics：pip install -r requirements-train.txt", file=sys.stderr)
        return 2

    print(f"\n开始训练：{args.model} -> {args.project}/{args.name}")
    model = YOLO(args.model)
    model.train(
        data=str(data_yaml),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        project=args.project,
        name=args.name,
        # 小目标 + 单一目标：开强增强但关掉会改变类别的增强
        mosaic=1.0,
        fliplr=0.5,
        flipud=0.0,          # 上下翻转会让"靶心在下方"这样的先验失效，关掉
        degrees=0.0,         # 靶标是人为摆正的，旋转交给云台
        hsv_h=0.015, hsv_s=0.7, hsv_v=0.4,
        patience=40,
        seed=0,
        exist_ok=True,
    )

    best = Path(args.project) / args.name / "weights" / "best.pt"
    print("\n训练完成。下一步：")
    print(f"  python tools/ml/export_rknn.py --weights {best} --data {data_yaml}")
    print("  导出前会再校验一次输出形状必须为 [1, 9, 8400]（车端 det_node 的契约）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
