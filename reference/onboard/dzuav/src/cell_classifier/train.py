#!/usr/bin/env python3
"""Train and evaluate the lightweight dual-head cell-label classifier."""

from __future__ import annotations

import argparse
import csv
import json
import random
import time
from pathlib import Path
from typing import Any

import numpy as np
import torch
from torch import nn
from torch.optim import AdamW
from torch.optim.lr_scheduler import CosineAnnealingLR
from torch.utils.data import DataLoader

from .constants import LETTERS, ROWS
from .dataset import CellLabelDataset, build_transform, class_weights, load_records
from .metrics import DualHeadMetrics
from .model import build_model


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, default=Path("datasets/编号/annotations.csv"))
    parser.add_argument("--output", type=Path, default=Path("runs/cell_classifier_v1"))
    parser.add_argument("--image-size", type=int, default=96)
    parser.add_argument("--epochs", type=int, default=80)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--weight-decay", type=float, default=1e-4)
    parser.add_argument("--label-smoothing", type=float, default=0.05)
    parser.add_argument("--class-weight-power", type=float, default=0.5)
    parser.add_argument("--patience", type=int, default=15)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--resume", type=Path)
    parser.add_argument("--pretrained", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--amp", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args()


def seed_everything(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.benchmark = True


def make_loader(
    dataset: CellLabelDataset,
    batch_size: int,
    workers: int,
    shuffle: bool,
    device: torch.device,
) -> DataLoader:
    return DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=shuffle,
        num_workers=workers,
        pin_memory=device.type == "cuda",
        persistent_workers=workers > 0,
        drop_last=False,
    )


def run_epoch(
    model: nn.Module,
    loader: DataLoader,
    device: torch.device,
    letter_loss_fn: nn.Module,
    row_loss_fn: nn.Module,
    optimizer: AdamW | None,
    scaler: torch.amp.GradScaler,
    amp: bool,
) -> dict[str, float]:
    training = optimizer is not None
    model.train(training)
    metrics = DualHeadMetrics(len(LETTERS), len(ROWS))
    loss_sum = letter_loss_sum = row_loss_sum = 0.0
    sample_count = 0

    for images, letter_targets, row_targets in loader:
        images = images.to(device, non_blocking=True)
        letter_targets = letter_targets.to(device, non_blocking=True)
        row_targets = row_targets.to(device, non_blocking=True)
        batch_size = images.shape[0]

        if training:
            optimizer.zero_grad(set_to_none=True)
        with torch.set_grad_enabled(training):
            with torch.amp.autocast(device_type=device.type, enabled=amp):
                letter_logits, row_logits = model(images)
                letter_loss = letter_loss_fn(letter_logits, letter_targets)
                row_loss = row_loss_fn(row_logits, row_targets)
                loss = letter_loss + row_loss
            if training:
                scaler.scale(loss).backward()
                scaler.unscale_(optimizer)
                torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=5.0)
                scaler.step(optimizer)
                scaler.update()

        metrics.update(letter_logits, row_logits, letter_targets, row_targets)
        loss_sum += loss.item() * batch_size
        letter_loss_sum += letter_loss.item() * batch_size
        row_loss_sum += row_loss.item() * batch_size
        sample_count += batch_size

    result = metrics.compute()
    result.update(
        {
            "loss": loss_sum / sample_count,
            "letter_loss": letter_loss_sum / sample_count,
            "row_loss": row_loss_sum / sample_count,
        }
    )
    return result


def checkpoint_payload(
    model: nn.Module,
    optimizer: AdamW,
    scheduler: CosineAnnealingLR,
    scaler: torch.amp.GradScaler,
    epoch: int,
    best_joint_accuracy: float,
    config: dict[str, Any],
) -> dict[str, Any]:
    return {
        "model_state": model.state_dict(),
        "optimizer_state": optimizer.state_dict(),
        "scheduler_state": scheduler.state_dict(),
        "scaler_state": scaler.state_dict(),
        "epoch": epoch,
        "best_joint_accuracy": best_joint_accuracy,
        "config": config,
        "letters": LETTERS,
        "rows": ROWS,
    }


def main() -> None:
    args = parse_args()
    seed_everything(args.seed)
    if args.device.startswith("cuda") and not torch.cuda.is_available():
        raise RuntimeError("CUDA requested but unavailable; pass --device cpu for a smoke test")
    device = torch.device(args.device)
    amp = bool(args.amp and device.type == "cuda")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)

    train_records = load_records(args.csv, "train")
    val_records = load_records(args.csv, "val")
    train_dataset = CellLabelDataset(train_records, build_transform(args.image_size, training=True))
    val_dataset = CellLabelDataset(val_records, build_transform(args.image_size, training=False))
    train_loader = make_loader(train_dataset, args.batch_size, args.workers, True, device)
    val_loader = make_loader(val_dataset, args.batch_size, args.workers, False, device)

    model = build_model(pretrained=args.pretrained and args.resume is None).to(device)
    parameter_count = sum(parameter.numel() for parameter in model.parameters())
    letter_weights = class_weights(
        train_records, len(LETTERS), "letter", args.class_weight_power
    ).to(device)
    row_weights = class_weights(train_records, len(ROWS), "row", args.class_weight_power).to(device)
    letter_loss_fn = nn.CrossEntropyLoss(
        weight=letter_weights, label_smoothing=args.label_smoothing
    )
    row_loss_fn = nn.CrossEntropyLoss(weight=row_weights, label_smoothing=args.label_smoothing)
    optimizer = AdamW(model.parameters(), lr=args.learning_rate, weight_decay=args.weight_decay)
    scheduler = CosineAnnealingLR(optimizer, T_max=args.epochs, eta_min=args.learning_rate * 0.02)
    scaler = torch.amp.GradScaler("cuda", enabled=amp)

    config = vars(args).copy()
    config["csv"] = str(args.csv.resolve())
    config["output"] = str(output)
    config["resume"] = str(args.resume.resolve()) if args.resume else None
    config.update(
        {
            "train_samples": len(train_records),
            "val_samples": len(val_records),
            "parameter_count": parameter_count,
            "torch_version": torch.__version__,
            "cuda_device": torch.cuda.get_device_name(device) if device.type == "cuda" else None,
        }
    )
    (output / "config.json").write_text(
        json.dumps(config, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    start_epoch = 1
    best_joint_accuracy = -1.0
    epochs_without_improvement = 0
    if args.resume:
        checkpoint = torch.load(args.resume, map_location=device, weights_only=False)
        model.load_state_dict(checkpoint["model_state"])
        optimizer.load_state_dict(checkpoint["optimizer_state"])
        scheduler.load_state_dict(checkpoint["scheduler_state"])
        scaler.load_state_dict(checkpoint["scaler_state"])
        start_epoch = int(checkpoint["epoch"]) + 1
        best_joint_accuracy = float(checkpoint.get("best_joint_accuracy", -1.0))

    metrics_path = output / "metrics.csv"
    metric_names = [
        "loss",
        "letter_loss",
        "row_loss",
        "letter_accuracy",
        "letter_macro_precision",
        "letter_macro_recall",
        "letter_macro_f1",
        "row_accuracy",
        "row_macro_precision",
        "row_macro_recall",
        "row_macro_f1",
        "joint_accuracy",
    ]
    fieldnames = ["epoch", "lr", "seconds"] + [
        f"{split}_{name}" for split in ("train", "val") for name in metric_names
    ]
    if not metrics_path.exists() or start_epoch == 1:
        with metrics_path.open("w", encoding="utf-8", newline="") as handle:
            csv.DictWriter(handle, fieldnames=fieldnames).writeheader()

    print(
        f"device={device} amp={amp} parameters={parameter_count:,} "
        f"train={len(train_records)} val={len(val_records)} output={output}",
        flush=True,
    )
    for epoch in range(start_epoch, args.epochs + 1):
        started = time.time()
        train_metrics = run_epoch(
            model,
            train_loader,
            device,
            letter_loss_fn,
            row_loss_fn,
            optimizer,
            scaler,
            amp,
        )
        with torch.inference_mode():
            val_metrics = run_epoch(
                model,
                val_loader,
                device,
                letter_loss_fn,
                row_loss_fn,
                None,
                scaler,
                amp,
            )
        scheduler.step()
        elapsed = time.time() - started
        row = {
            "epoch": epoch,
            "lr": optimizer.param_groups[0]["lr"],
            "seconds": elapsed,
            **{f"train_{key}": value for key, value in train_metrics.items()},
            **{f"val_{key}": value for key, value in val_metrics.items()},
        }
        with metrics_path.open("a", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writerow(row)

        improved = val_metrics["joint_accuracy"] > best_joint_accuracy
        if improved:
            best_joint_accuracy = val_metrics["joint_accuracy"]
            epochs_without_improvement = 0
        else:
            epochs_without_improvement += 1
        payload = checkpoint_payload(
            model, optimizer, scheduler, scaler, epoch, best_joint_accuracy, config
        )
        torch.save(payload, output / "last.pt")
        if improved:
            torch.save(payload, output / "best.pt")

        print(
            f"epoch={epoch:03d}/{args.epochs} time={elapsed:.1f}s "
            f"train_loss={train_metrics['loss']:.4f} "
            f"val_loss={val_metrics['loss']:.4f} "
            f"letter_acc={val_metrics['letter_accuracy']:.4f} "
            f"row_acc={val_metrics['row_accuracy']:.4f} "
            f"joint_acc={val_metrics['joint_accuracy']:.4f} "
            f"best={best_joint_accuracy:.4f}",
            flush=True,
        )
        if epochs_without_improvement >= args.patience:
            print(f"early_stopping epoch={epoch} patience={args.patience}", flush=True)
            break

    print(f"training_complete best_joint_accuracy={best_joint_accuracy:.6f}", flush=True)


if __name__ == "__main__":
    main()
