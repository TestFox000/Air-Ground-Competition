#!/usr/bin/env python3
"""`.pt` -> ONNX -> `.rknn` 导出流水线（带三重契约检查）。

    # 1) 只导 ONNX（本机就能做，Windows/Linux 均可）
    python tools/ml/export_rknn.py --weights runs/pose/air_ground_pose/weights/best.pt --onnx-only

    # 2) ONNX -> RKNN（rknn-toolkit2 只在 Linux x86 / 板端可用）
    python tools/ml/export_rknn.py --weights best.pt --quant-images /path/to/imgs.txt

为什么每一步都要检查：车端 `det_node` 的解码写死了
    输出 [1, 9, 8400]、box 单位=输入像素、cls 已激活、kpt 也是像素
导出参数稍有不同（比如带上解码前的 64 通道、或把 sigmoid 拿掉），
形状或数值语义就变了，而**上车才发现**。所以：
    * 导出 ONNX 后 → tools/ml/verify_contract.py 查形状
    * 转 RKNN 后   → 再查一次（量化会改变输入/输出 dtype）
    * 上车前       → tools/verify_decode.py 用真图实测 cls_activation / kpt_unit
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ML = Path(__file__).resolve().parent
IMG_SIZE = 640


def run(cmd) -> None:
    print(f"\n$ {' '.join(str(c) for c in cmd)}")
    r = subprocess.run([str(c) for c in cmd])
    if r.returncode != 0:
        raise SystemExit(f"上一步失败（退出码 {r.returncode}）")


def verify(model: Path, nc: int, nkpt: int) -> None:
    run([sys.executable, str(ML / "verify_contract.py"),
         "--model", str(model), "--nc", str(nc), "--nkpt", str(nkpt), "--kpt-dim", "3"])


def export_onnx(weights: Path, out_dir: Path) -> Path:
    try:
        from ultralytics import YOLO
    except ImportError:
        raise SystemExit("导出 ONNX 需要 ultralytics：pip install -r requirements-train.txt")
    model = YOLO(str(weights))
    path = model.export(
        format="onnx",
        imgsz=IMG_SIZE,
        opset=12,
        simplify=True,      # 去掉冗余算子，NPU 转换更顺
        dynamic=False,      # 固定 1x3x640x640，车端按固定尺寸喂
    )
    onnx_path = Path(path)
    print(f"ONNX 已生成: {onnx_path}")
    return onnx_path


def export_rknn(onnx_path: Path, out_path: Path, quant_images: Path,
                nc: int, quantized: bool) -> Path:
    try:
        from rknn.api import RKNN
    except ImportError:
        raise SystemExit(
            "rknn-toolkit2 只在 Linux x86 上可用（板端也能跑但要装 toolkit，不是 lite）。\n"
            "  pip install rknn-toolkit2    # 或按官方文档装 wheel\n"
            f"已生成好的 ONNX 可以直接拿到 Linux 上转：{onnx_path}")

    rknn = RKNN(verbose=True)
    rknn.config(
        mean_values=[[0, 0, 0]],        # YOLO 输入归一化：pixel/255
        std_values=[[255, 255, 255]],
        target_platform="rk3588",
        quantized_dtype="asymmetric_quantized-8" if quantized else "float16",
        # DFL 的 softmax 在 NPU 上不友好，交给 CPU 或让转换器融合
        optimization_level=3,
    )
    if rknn.load_onnx(model=str(onnx_path)) != 0:
        raise SystemExit("load_onnx 失败")
    print("构建中（量化需要几分钟）…")
    if rknn.build(do_quantization=quantized,
                  dataset=str(quant_images) if quantized else None) != 0:
        raise SystemExit("build 失败")
    if rknn.export_rknn(str(out_path)) != 0:
        raise SystemExit("export_rknn 失败")
    rknn.release()
    print(f"RKNN 已生成: {out_path}")
    return out_path


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="pt -> onnx -> rknn 导出（带契约检查）")
    ap.add_argument("--weights", required=True, help="训练得到的 best.pt")
    ap.add_argument("--out-dir", default=str(ROOT / "models"))
    ap.add_argument("--name", default="best_new", help="输出文件名（不含扩展名）")
    ap.add_argument("--quant-images", default=None,
                    help="量化校准图片清单（每行一个绝对路径，30~100 张真图最佳）")
    ap.add_argument("--no-quant", action="store_true", help="不做量化（float16，精度稳但更慢）")
    ap.add_argument("--nc", type=int, default=2)
    ap.add_argument("--nkpt", type=int, default=1)
    ap.add_argument("--onnx-only", action="store_true", help="只导 ONNX（本机即可完成）")
    args = ap.parse_args(argv)

    weights = Path(args.weights).expanduser().resolve()
    if not weights.is_file():
        raise SystemExit(f"找不到 {weights}")
    out_dir = Path(args.out_dir).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)

    onnx_path = export_onnx(weights, out_dir)
    verify(onnx_path, args.nc, args.nkpt)          # ① 形状契约
    if args.onnx_only:
        print("\n下一步（在 Linux x86 上）：")
        print(f"  python tools/ml/export_rknn.py --weights {weights} --quant-images imgs.txt")
        return 0

    if not args.quant_images and not args.no_quant:
        raise SystemExit("量化需要 --quant-images（每行一个真图绝对路径）；"
                         "或加 --no-quant 走 float16")
    rknn_path = export_rknn(onnx_path, out_dir / f"{args.name}.rknn",
                            Path(args.quant_images) if args.quant_images else Path("."),
                            args.nc, quantized=not args.no_quant)
    verify(rknn_path, args.nc, args.nkpt)          # ② 量化后再查一次

    print("\n最后三步（都要在板端/车端做）：")
    print(f"  1. 拷到车上: ~/dzrobot/src/rknn_pt/model/{args.name}.rknn")
    print("  2. 数值语义实测: python tools/verify_decode.py --model <rknn> --image <真图>")
    print("     把结论写进 params.yaml 的 cls_activation / kpt_unit")
    print("  3. det_node.cc 里的模型路径要指到新文件，再 ros2 launch dzactuator det_node.launch.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
