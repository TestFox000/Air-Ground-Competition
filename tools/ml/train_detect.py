"""训练航拍场地 11 类检测模型（Ultralytics 薄封装），替代官方 hbb。

模型：YOLO11s / 1280 输入（2026-10-02 用户确认）。
    - 与官方 hbb 同为 s 量级，Orin Nano TensorRT FP16 帧率相当；
    - YOLO11 骨干在同参数下精度/效率优于官方 hbb 的 YOLOv8；
    - 1280 是高空看清小卡片/格号文字的必要尺寸。
11 类：mine cell_label arrow bridge car_start car_end uav_start uav_end
       bucket_red bucket_green bucket_cyan
数据：split_capture.py 从「校正后的 capture 帧」生成（时间块切分防泄漏）。

用法：
    python tools/ml/train_detect.py --data datasets/aerial_field/data.yaml
    python tools/ml/train_detect.py --data ... --model yolo11s.pt --epochs 200
训练完成后自动导出固定形状 ONNX（Orin TensorRT 转换用）。
退出码：0 成功；2 依赖；3 训练产物缺失。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--data", default=None,
                        help="split_capture 生成的 data.yaml（resume 时可省）")
    parser.add_argument("--model", default="yolo11s.pt",
                        help="预训练权重（默认 yolo11s.pt）")
    parser.add_argument("--epochs", type=int, default=200)
    parser.add_argument("--imgsz", type=int, default=1280)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--device", default="0", help="0=GPU；cpu=无显卡")
    parser.add_argument("--workers", type=int, default=0,
                        help="dataloader 进程数；5070Ti+cuDNN9 子进程会崩，默认 0")
    parser.add_argument("--amp", action="store_true", default=False,
                        help="开启混合精度（默认关，Blackwell+cuDNN9 不稳定）")
    parser.add_argument("--project", default="runs/detect")
    parser.add_argument("--name", default="aerial_field")
    parser.add_argument("--resume", action="store_true",
                        help="从 last.pt 恢复训练（断电/重启后）")
    args = parser.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        print("缺少 ultralytics：python -m pip install ultralytics", file=sys.stderr)
        return 2

    # resume 时从 last.pt 加载，且只传 resume=True（其他参数 Ultralytics 会从 ckpt 恢复）
    if args.resume:
        # 搜索 last.pt（save_dir 可能因项目嵌套而路径不同）
        candidates = [
            Path(args.project) / args.name / "weights" / "last.pt",
            Path.cwd() / args.project / args.name / "weights" / "last.pt",
        ]
        # 也扫一下 args.project 下所有 last.pt 取最新
        search_root = Path.cwd() / args.project
        if search_root.exists():
            candidates += list(search_root.rglob("last.pt"))
        last_pt = None
        for c in candidates:
            if c.exists():
                if last_pt is None or c.stat().st_mtime > last_pt.stat().st_mtime:
                    last_pt = c
        if last_pt is None:
            print(f"resume 失败：未找到 last.pt（搜索 {search_root}）", file=sys.stderr)
            return 3
        print(f"resume from: {last_pt}")
        model = YOLO(str(last_pt))
        results = model.train(resume=True)
    else:
        if not args.data:
            print("非 resume 模式需要 --data", file=sys.stderr)
            return 2
        model = YOLO(args.model)
        results = model.train(
            data=args.data,
            epochs=args.epochs,
            imgsz=args.imgsz,
            batch=args.batch,
            device=args.device,
            workers=args.workers,
            amp=args.amp,
            project=args.project,
            name=args.name,
            patience=40,
            mosaic=0.5,
            close_mosaic=15,
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
    onnx_path = export.export(
        format="onnx", imgsz=args.imgsz, simplify=True, opset=12
    )
    print("=" * 60)
    print(f"训练完成。best.pt: {best}")
    print(f"ONNX: {onnx_path}")
    print("下一步：tools/orin_convert_engines.sh 转 Orin TensorRT FP16 引擎")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
