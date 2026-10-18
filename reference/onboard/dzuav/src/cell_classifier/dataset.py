"""CSV-backed dataset and augmentation pipeline for cell-label crops."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

from PIL import Image
import torch
from torch.utils.data import Dataset
from torchvision import transforms
from torchvision.transforms import InterpolationMode

from .constants import IMAGENET_MEAN, IMAGENET_STD, LETTER_TO_INDEX, ROW_TO_INDEX


@dataclass(frozen=True)
class CellLabelRecord:
    image_path: Path
    crop_path: str
    split: str
    cell_id: str
    letter_index: int
    row_index: int


def load_records(csv_path: Path, split: str) -> list[CellLabelRecord]:
    """Load only user-approved rows from annotations.csv."""
    csv_path = csv_path.resolve()
    dataset_root = csv_path.parent
    records: list[CellLabelRecord] = []

    with csv_path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        required = {"crop_path", "split", "cell_id", "column", "row", "status"}
        missing = required.difference(reader.fieldnames or ())
        if missing:
            raise ValueError(f"CSV missing required fields: {sorted(missing)}")

        for line_number, row in enumerate(reader, start=2):
            if row["split"].strip() != split or row["status"].strip() != "labeled":
                continue

            cell_id = row["cell_id"].strip().upper()
            letter = row["column"].strip().upper()
            number = row["row"].strip()
            if letter not in LETTER_TO_INDEX or number not in ROW_TO_INDEX:
                raise ValueError(f"Invalid label at CSV line {line_number}: {cell_id!r}")
            if cell_id != f"{letter}{number}":
                raise ValueError(
                    f"Inconsistent cell_id/column/row at CSV line {line_number}: "
                    f"{cell_id!r}, {letter!r}, {number!r}"
                )

            relative_path = Path(row["crop_path"].strip())
            image_path = (dataset_root / relative_path).resolve()
            try:
                image_path.relative_to(dataset_root)
            except ValueError as error:
                raise ValueError(f"Unsafe crop path at CSV line {line_number}") from error
            if not image_path.is_file():
                raise FileNotFoundError(image_path)

            records.append(
                CellLabelRecord(
                    image_path=image_path,
                    crop_path=relative_path.as_posix(),
                    split=split,
                    cell_id=cell_id,
                    letter_index=LETTER_TO_INDEX[letter],
                    row_index=ROW_TO_INDEX[number],
                )
            )

    if not records:
        raise ValueError(f"No labeled records found for split={split!r} in {csv_path}")
    return records


def build_transform(image_size: int, training: bool) -> transforms.Compose:
    """Create transforms; rotations model arbitrary drone yaw without mirroring text."""
    if training:
        return transforms.Compose(
            [
                transforms.Resize((image_size, image_size), interpolation=InterpolationMode.BICUBIC),
                transforms.RandomAffine(
                    degrees=(-25.0, 25.0),
                    translate=(0.04, 0.04),
                    scale=(0.85, 1.10),
                    shear=(-4.0, 4.0),
                    interpolation=InterpolationMode.BICUBIC,
                    fill=(114, 114, 114),
                ),
                transforms.RandomPerspective(
                    distortion_scale=0.10,
                    p=0.15,
                    interpolation=InterpolationMode.BICUBIC,
                    fill=114,
                ),
                transforms.ColorJitter(
                    brightness=0.35,
                    contrast=0.35,
                    saturation=0.15,
                    # The source is nearly monochrome, so hue jitter adds
                    # little value and breaks on some old torchvision/NumPy
                    # combinations.
                    hue=0.0,
                ),
                transforms.RandomGrayscale(p=0.08),
                transforms.RandomApply(
                    [transforms.GaussianBlur(kernel_size=3, sigma=(0.1, 1.2))], p=0.15
                ),
                transforms.RandomAutocontrast(p=0.10),
                transforms.ToTensor(),
                transforms.Normalize(IMAGENET_MEAN, IMAGENET_STD),
                transforms.RandomErasing(
                    p=0.08,
                    scale=(0.01, 0.05),
                    ratio=(0.5, 2.0),
                    value="random",
                ),
            ]
        )

    return transforms.Compose(
        [
            transforms.Resize((image_size, image_size), interpolation=InterpolationMode.BICUBIC),
            transforms.ToTensor(),
            transforms.Normalize(IMAGENET_MEAN, IMAGENET_STD),
        ]
    )


class CellLabelDataset(Dataset):
    def __init__(self, records: list[CellLabelRecord], transform: transforms.Compose):
        self.records = records
        self.transform = transform

    def __len__(self) -> int:
        return len(self.records)

    def __getitem__(self, index: int) -> tuple[torch.Tensor, int, int]:
        record = self.records[index]
        with Image.open(record.image_path) as image:
            tensor = self.transform(image.convert("RGB"))
        return tensor, record.letter_index, record.row_index


def class_weights(
    records: list[CellLabelRecord], num_classes: int, target: str, power: float = 0.5
) -> torch.Tensor:
    """Return gently inverse-frequency-weighted loss weights with mean one."""
    if target not in {"letter", "row"}:
        raise ValueError(target)
    counts = torch.zeros(num_classes, dtype=torch.float64)
    for record in records:
        index = record.letter_index if target == "letter" else record.row_index
        counts[index] += 1
    if torch.any(counts == 0):
        raise ValueError(f"Training split has an empty {target} class: {counts.tolist()}")
    weights = counts.pow(-power)
    return (weights / weights.mean()).float()
