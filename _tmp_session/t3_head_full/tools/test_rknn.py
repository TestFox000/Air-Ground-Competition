#!/usr/bin/env python3
"""检查 RKNN 模型能否加载并可选执行一次图片推理。"""

import argparse
from pathlib import Path

import cv2
import numpy as np
from rknn.api import RKNN

WORKSPACE_ROOT = Path(__file__).resolve().parent
DEFAULT_MODEL = WORKSPACE_ROOT / "src" / "det_node" / "model" / "light_det.rknn"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", nargs="?", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--image", type=Path, help="用于试推理的图片；省略时只检查模型")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.model.is_file():
        print(f"Model file not found: {args.model}")
        return 1

    rknn = RKNN(verbose=True)
    try:
        print("=== load_rknn ===")
        result = rknn.load_rknn(str(args.model))
        print("load_rknn ret =", result)
        if result != 0:
            print("model file load failed or model is corrupted")
            return 1

        print("=== init_runtime ===")
        result = rknn.init_runtime()
        print("init_runtime ret =", result)
        if result != 0:
            print("NPU runtime init failed")
            return 1

        print("=== inputs ===")
        print(rknn.get_input_attrs())
        print("=== outputs ===")
        print(rknn.get_output_attrs())

        if args.image:
            image = cv2.imread(str(args.image))
            if image is None:
                print(f"Image could not be read: {args.image}")
                return 1
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image = cv2.resize(image, (640, 640))
            output = rknn.inference(inputs=[image])[0]
            print("output[0] shape:", np.asarray(output).shape)
            print("output[0] max:", np.max(output))
            print("output[0] min:", np.min(output))
    finally:
        rknn.release()

    print("OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
