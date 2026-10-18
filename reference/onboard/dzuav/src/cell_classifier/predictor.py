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
        rotation_tta: bool = True,
        confidence_threshold: float = 0.45,
    ) -> None:
        if device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError("CUDA requested but no CUDA device is available")
        self.device = torch.device(device)
        self.model, checkpoint = load_checkpoint_model(weights, self.device)
        config = checkpoint.get("config", {})
        self.image_size = int(image_size or config.get("image_size", 96))
        self.transform = build_transform(self.image_size, training=False)
        self.use_half = bool(half and self.device.type == "cuda")
        # 小角度 TTA：对文字用 ±10° 投票（90° 旋转会让文字侧过来，反而降精度）
        self.rotation_tta = rotation_tta
        self.tta_angles = (-10, -5, 0, 5, 10) if rotation_tta else (0,)
        # 置信度阈值：低于此值的预测返回 None，宁缺勿错（PnP 少几个点没事，错点才致命）
        self.confidence_threshold = confidence_threshold
        if self.use_half:
            self.model.half()

    @torch.inference_mode()
    def predict(self, images: Iterable[Image.Image]) -> list[CellIdPrediction | None]:
        image_list = [image.convert("RGB") for image in images]
        if not image_list:
            return []

        letter_probabilities = 0.0
        row_probabilities = 0.0
        for angle in self.tta_angles:
            if angle == 0:
                rotated = image_list
            else:
                rotated = [
                    image.rotate(angle, resample=Image.BICUBIC, fillcolor=(232, 232, 232))
                    for image in image_list
                ]
            batch = torch.stack([self.transform(image) for image in rotated]).to(self.device)
            if self.use_half:
                batch = batch.half()
            letter_logits, row_logits = self.model(batch)
            letter_probabilities = letter_probabilities + letter_logits.float().softmax(dim=1)
            row_probabilities = row_probabilities + row_logits.float().softmax(dim=1)
        letter_probabilities = letter_probabilities / len(self.tta_angles)
        row_probabilities = row_probabilities / len(self.tta_angles)

        letter_confidence, letter_index = letter_probabilities.max(dim=1)
        row_confidence, row_index = row_probabilities.max(dim=1)
        predictions: list[CellIdPrediction | None] = []
        for li, ri, lc, rc in zip(
            letter_index.tolist(),
            row_index.tolist(),
            letter_confidence.tolist(),
            row_confidence.tolist(),
        ):
            # 置信度不足则返回 None，上游跳过该格号（宁缺勿错）
            if lc * rc < self.confidence_threshold:
                predictions.append(None)
                continue
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

    def predict_paths(self, paths: Iterable[Path | str]) -> list[CellIdPrediction | None]:
        images = []
        for path in paths:
            with Image.open(path) as image:
                images.append(image.convert("RGB"))
        return self.predict(images)

