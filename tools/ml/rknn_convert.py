"""把训练导出的 ONNX 转成 rk3588 可用的 RKNN 模型（完整版 rknn-toolkit2）。

运行位置：装了完整版 rknn-toolkit2 的机器（rk3588 aarch64 官方 whl，或 x86
开发机）。注意与「rknn-toolkit-lite2（车端只推理）」区分。

两种产物：
    FP16（--no-quant，默认）：不量化，精度损失最小、操作最简单
    INT8（默认行为，需校准图）：NPU 全精度加速，需提供 50~200 张代表性
        训练图做量化校准（dataset.txt，每行一张图的绝对路径）

用法：
    python tools/ml/rknn_convert.py --onnx runs/pose/move_target/weights/best.onnx \
        --out models/move_target.rknn --dataset calibration/dataset.txt
    python tools/ml/rknn_convert.py --onnx ... --out ... --no-quant
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--onnx", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--dataset", help="INT8 校准清单 txt（每行一张图）")
    parser.add_argument("--no-quant", action="store_true",
                        help="不做 INT8 量化（FP16 混合，精度优先）")
    parser.add_argument("--target", default="rk3588")
    args = parser.parse_args()

    if not args.no_quant and not args.dataset:
        print("INT8 量化必须提供 --dataset；或显式加 --no-quant", file=sys.stderr)
        return 2

    try:
        from rknn.api import RKNN
    except ImportError:
        print("缺少完整版 rknn-toolkit2（不是 lite2）：按 Rockchip 官方 whl 安装",
              file=sys.stderr)
        return 2

    rknn = RKNN(verbose=False)
    # 输入已经是 0~1（模型内 /255 语义由导出 ONNX 承接），mean/std 保持恒等。
    rknn.config(
        mean_values=[[0, 0, 0]],
        std_values=[[255, 255, 255]],
        target_platform=args.target,
    )

    if rknn.load_onnx(model=args.onnx) != 0:
        print("load_onnx 失败", file=sys.stderr)
        return 3

    do_quant = not args.no_quant
    if rknn.build(do_quantization=do_quant,
                  dataset=args.dataset if do_quant else None) != 0:
        print("build 失败", file=sys.stderr)
        return 3

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    if rknn.export_rknn(str(out_path)) != 0:
        print("export 失败", file=sys.stderr)
        return 3
    rknn.release()

    mode = "INT8 量化" if do_quant else "FP16（未量化）"
    print("=" * 60)
    print(f"RKNN 导出完成（{mode}）: {out_path.resolve()}")
    print("下一步：tools/ml/compare_models.py 与官方模型同集对比")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
