"""MobileNetV3-Small backbone with shared embedding and two classification heads."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import torch
from torch import nn
from torchvision.models import MobileNet_V3_Small_Weights, mobilenet_v3_small

from .constants import LETTERS, ROWS


class DualHeadCellClassifier(nn.Module):
    """Predict column letter and row number from one shared visual embedding."""

    def __init__(self, pretrained: bool = True, embedding_dim: int = 256, dropout: float = 0.20):
        super().__init__()
        weights = MobileNet_V3_Small_Weights.IMAGENET1K_V1 if pretrained else None
        backbone = mobilenet_v3_small(weights=weights)
        feature_dim = backbone.classifier[0].in_features
        self.features = backbone.features
        self.pool = backbone.avgpool
        self.embedding = nn.Sequential(
            nn.Linear(feature_dim, embedding_dim, bias=False),
            nn.LayerNorm(embedding_dim),
            nn.Hardswish(inplace=True),
            nn.Dropout(p=dropout),
        )
        self.letter_head = nn.Linear(embedding_dim, len(LETTERS))
        self.row_head = nn.Linear(embedding_dim, len(ROWS))

        nn.init.trunc_normal_(self.embedding[0].weight, std=0.02)
        nn.init.trunc_normal_(self.letter_head.weight, std=0.02)
        nn.init.zeros_(self.letter_head.bias)
        nn.init.trunc_normal_(self.row_head.weight, std=0.02)
        nn.init.zeros_(self.row_head.bias)

    def forward(self, images: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        features = self.features(images)
        features = self.pool(features)
        features = torch.flatten(features, 1)
        embedding = self.embedding(features)
        return self.letter_head(embedding), self.row_head(embedding)


def build_model(pretrained: bool = True) -> DualHeadCellClassifier:
    return DualHeadCellClassifier(pretrained=pretrained)


def load_checkpoint_model(
    checkpoint_path: Path | str,
    device: torch.device | str = "cpu",
) -> tuple[DualHeadCellClassifier, dict[str, Any]]:
    checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=False)
    model = build_model(pretrained=False)
    state_dict = checkpoint.get("model_state", checkpoint)
    model.load_state_dict(state_dict, strict=True)
    model.to(device)
    model.eval()
    return model, checkpoint

