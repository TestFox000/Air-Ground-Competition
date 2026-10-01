#!/usr/bin/env python3
"""Run batched cell-ID inference and print or save JSON Lines results."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .predictor import CellIdRecognizer


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--weights", type=Path, required=True)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--rotation-tta", action="store_true")
    parser.add_argument("--half", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args()


def collect_images(source: Path) -> list[Path]:
    if source.is_file():
        return [source]
    if not source.is_dir():
        raise FileNotFoundError(source)
    extensions = {".png", ".jpg", ".jpeg", ".bmp", ".webp"}
    return sorted(path for path in source.rglob("*") if path.suffix.lower() in extensions)


def main() -> None:
    args = parse_args()
    paths = collect_images(args.source)
    recognizer = CellIdRecognizer(
        args.weights,
        device=args.device,
        half=args.half,
        rotation_tta=args.rotation_tta,
    )
    lines = []
    for start in range(0, len(paths), args.batch_size):
        batch_paths = paths[start : start + args.batch_size]
        predictions = recognizer.predict_paths(batch_paths)
        for path, prediction in zip(batch_paths, predictions):
            record = {"image": str(path.resolve()), **prediction.to_dict()}
            lines.append(json.dumps(record, ensure_ascii=False))
    text = "\n".join(lines) + ("\n" if lines else "")
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
        print(f"saved {len(lines)} predictions to {args.output}")
    else:
        print(text, end="")


if __name__ == "__main__":
    main()
