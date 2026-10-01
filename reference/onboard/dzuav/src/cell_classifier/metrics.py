"""Dependency-free classification metrics for both heads and their joint result."""

from __future__ import annotations

from dataclasses import dataclass

import torch


def classification_scores(confusion: torch.Tensor) -> dict[str, float]:
    confusion = confusion.double()
    true_positive = confusion.diag()
    predicted = confusion.sum(dim=0)
    actual = confusion.sum(dim=1)
    precision = true_positive / predicted.clamp_min(1.0)
    recall = true_positive / actual.clamp_min(1.0)
    f1 = 2.0 * precision * recall / (precision + recall).clamp_min(1e-12)
    total = confusion.sum().clamp_min(1.0)
    return {
        "accuracy": (true_positive.sum() / total).item(),
        "macro_precision": precision.mean().item(),
        "macro_recall": recall.mean().item(),
        "macro_f1": f1.mean().item(),
    }


@dataclass
class DualHeadMetrics:
    letter_classes: int
    row_classes: int

    def __post_init__(self) -> None:
        self.letter_confusion = torch.zeros(self.letter_classes, self.letter_classes, dtype=torch.long)
        self.row_confusion = torch.zeros(self.row_classes, self.row_classes, dtype=torch.long)
        self.joint_correct = 0
        self.total = 0

    def update(
        self,
        letter_logits: torch.Tensor,
        row_logits: torch.Tensor,
        letter_targets: torch.Tensor,
        row_targets: torch.Tensor,
    ) -> None:
        letter_predictions = letter_logits.argmax(dim=1).detach().cpu()
        row_predictions = row_logits.argmax(dim=1).detach().cpu()
        letter_targets = letter_targets.detach().cpu()
        row_targets = row_targets.detach().cpu()
        self.letter_confusion += torch.bincount(
            letter_targets * self.letter_classes + letter_predictions,
            minlength=self.letter_classes**2,
        ).reshape(self.letter_classes, self.letter_classes)
        self.row_confusion += torch.bincount(
            row_targets * self.row_classes + row_predictions,
            minlength=self.row_classes**2,
        ).reshape(self.row_classes, self.row_classes)
        self.joint_correct += int(
            ((letter_predictions == letter_targets) & (row_predictions == row_targets)).sum()
        )
        self.total += int(letter_targets.numel())

    def compute(self) -> dict[str, float]:
        letter = classification_scores(self.letter_confusion)
        row = classification_scores(self.row_confusion)
        result = {f"letter_{key}": value for key, value in letter.items()}
        result.update({f"row_{key}": value for key, value in row.items()})
        result["joint_accuracy"] = self.joint_correct / max(self.total, 1)
        return result
