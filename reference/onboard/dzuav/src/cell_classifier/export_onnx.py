#!/usr/bin/env python3
"""Export the dual-head classifier to ONNX for TensorRT deployment."""

from __future__ import annotations

import argparse
from pathlib import Path

import torch

from .model import load_checkpoint_model


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--weights", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--image-size", type=int)
    parser.add_argument("--opset", type=int, default=17)
    parser.add_argument("--static-batch", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    model, checkpoint = load_checkpoint_model(args.weights, "cpu")
    image_size = int(args.image_size or checkpoint.get("config", {}).get("image_size", 96))
    example = torch.randn(1, 3, image_size, image_size)
    dynamic_axes = None if args.static_batch else {
        "images": {0: "batch"},
        "letter_logits": {0: "batch"},
        "row_logits": {0: "batch"},
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(
        model,
        example,
        args.output,
        input_names=["images"],
        output_names=["letter_logits", "row_logits"],
        dynamic_axes=dynamic_axes,
        opset_version=args.opset,
        do_constant_folding=True,
    )
    print(f"exported {args.output.resolve()}")


if __name__ == "__main__":
    main()
