#!/usr/bin/env python3
"""Evaluate a checkpoint and save auditable per-image and per-class results."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import torch

from .constants import LETTERS, ROWS
from .dataset import load_records
from .metrics import classification_scores
from .predictor import CellIdRecognizer


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--weights", type=Path, required=True)
    parser.add_argument("--csv", type=Path, default=Path("datasets/编号/annotations.csv"))
    parser.add_argument("--split", choices=("train", "val"), default="val")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--batch-size", type=int, default=512)
    parser.add_argument("--rotation-tta", action="store_true")
    return parser.parse_args()


def save_confusion(path: Path, labels: tuple[str, ...], matrix: torch.Tensor) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["actual\\predicted", *labels])
        for label, values in zip(labels, matrix.tolist()):
            writer.writerow([label, *values])


def main() -> None:
    args = parse_args()
    records = load_records(args.csv, args.split)
    recognizer = CellIdRecognizer(
        args.weights,
        device=args.device,
        half=True,
        rotation_tta=args.rotation_tta,
    )
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    letter_confusion = torch.zeros(len(LETTERS), len(LETTERS), dtype=torch.long)
    row_confusion = torch.zeros(len(ROWS), len(ROWS), dtype=torch.long)
    joint_correct = 0
    rows = []

    for start in range(0, len(records), args.batch_size):
        batch = records[start : start + args.batch_size]
        predictions = recognizer.predict_paths(record.image_path for record in batch)
        for record, prediction in zip(batch, predictions):
            predicted_letter = LETTERS.index(prediction.column)
            predicted_row = ROWS.index(prediction.row)
            letter_confusion[record.letter_index, predicted_letter] += 1
            row_confusion[record.row_index, predicted_row] += 1
            correct = prediction.cell_id == record.cell_id
            joint_correct += int(correct)
            rows.append(
                {
                    "crop_path": record.crop_path,
                    "actual_cell_id": record.cell_id,
                    "predicted_cell_id": prediction.cell_id,
                    "actual_column": LETTERS[record.letter_index],
                    "predicted_column": prediction.column,
                    "actual_row": ROWS[record.row_index],
                    "predicted_row": prediction.row,
                    "column_confidence": prediction.column_confidence,
                    "row_confidence": prediction.row_confidence,
                    "joint_confidence": prediction.joint_confidence,
                    "correct": correct,
                }
            )

    prediction_path = output / f"{args.split}_predictions.csv"
    with prediction_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    save_confusion(output / "letter_confusion.csv", LETTERS, letter_confusion)
    save_confusion(output / "row_confusion.csv", ROWS, row_confusion)

    summary = {
        "split": args.split,
        "samples": len(records),
        "rotation_tta": args.rotation_tta,
        "joint_accuracy": joint_correct / len(records),
        "letter": classification_scores(letter_confusion),
        "row": classification_scores(row_confusion),
        "errors": len(records) - joint_correct,
    }
    (output / "evaluation_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
