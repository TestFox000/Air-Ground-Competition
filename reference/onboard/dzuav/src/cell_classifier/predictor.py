"""Reusable GPU/CPU inference interface for the trained classifier."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

from PIL import Image
import torch

from .constants import LETTERS, ROWS
from .dataset import build_transform
from .model import load_checkpoint_model


@dataclass(frozen=True)
class CellIdPrediction:
    cell_id: str
    column: str
    row: str
    column_confidence: float
    row_confidence: float
    joint_confidence: float

    def to_dict(self) -> dict[str, str | float]:
        return asdict(self)


class CellIdRecognizer:
    """Batch-capable recognizer intended for use after YOLO label cropping."""

    def __init__(
        self,
        weights: Path | str,
        device: str = "cuda:0",
        image_size: int | None = None,
        half: bool = True,
        rotation_tta: bool = False,
    ) -> None:
        if device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError("CUDA requested but no CUDA device is available")
        self.device = torch.device(device)
        self.model, checkpoint = load_checkpoint_model(weights, self.device)
        config = checkpoint.get("config", {})
        self.image_size = int(image_size or config.get("image_size", 96))
        self.transform = build_transform(self.image_size, training=False)
        self.use_half = bool(half and self.device.type == "cuda")
        self.rotation_tta = rotation_tta
        if self.use_half:
            self.model.half()

    @torch.inference_mode()
    def predict(self, images: Iterable[Image.Image]) -> list[CellIdPrediction]:
        image_list = [image.convert("RGB") for image in images]
        if not image_list:
            return []
        batch = torch.stack([self.transform(image) for image in image_list]).to(self.device)
        if self.use_half:
            batch = batch.half()

        rotations = (0, 1, 2, 3) if self.rotation_tta else (0,)
        letter_probabilities = 0.0
        row_probabilities = 0.0
        for turns in rotations:
            inputs = torch.rot90(batch, turns, dims=(-2, -1)) if turns else batch
            letter_logits, row_logits = self.model(inputs)
            letter_probabilities = letter_probabilities + letter_logits.float().softmax(dim=1)
            row_probabilities = row_probabilities + row_logits.float().softmax(dim=1)
        letter_probabilities = letter_probabilities / len(rotations)
        row_probabilities = row_probabilities / len(rotations)

        letter_confidence, letter_index = letter_probabilities.max(dim=1)
        row_confidence, row_index = row_probabilities.max(dim=1)
        predictions = []
        for li, ri, lc, rc in zip(
            letter_index.tolist(),
            row_index.tolist(),
            letter_confidence.tolist(),
            row_confidence.tolist(),
        ):
            column = LETTERS[li]
            row = ROWS[ri]
            predictions.append(
                CellIdPrediction(
                    cell_id=f"{column}{row}",
                    column=column,
                    row=row,
                    column_confidence=lc,
                    row_confidence=rc,
                    joint_confidence=lc * rc,
                )
            )
        return predictions

    def predict_paths(self, paths: Iterable[Path | str]) -> list[CellIdPrediction]:
        images = []
        for path in paths:
            with Image.open(path) as image:
                images.append(image.convert("RGB"))
        return self.predict(images)

