# -*- coding: utf-8 -*-
r"""把感知结果画回图像（人工复核用，不参与任何判据）。

产物用于"人眼确认管线没跑偏"：网格线、检出的格、车体框、箭头框、以及每格的类名。
只依赖 Pillow。
"""

from __future__ import annotations

import numpy as np
from PIL import Image, ImageDraw

from .colors import as_rgb_array
from .grid import ArenaRegistration

__all__ = ["CLASS_COLORS", "draw_result", "draw_registration"]

CLASS_COLORS: dict[str, tuple[int, int, int]] = {
    "mine": (60, 110, 255),
    "obstacle": (255, 60, 60),
    "target": (255, 90, 220),
    "bridge": (40, 190, 120),
    "arrow": (0, 200, 90),
    "car": (255, 150, 30),
    "car_nose": (160, 90, 240),
    "deck": (230, 200, 40),
}


def _rgba(img: np.ndarray) -> Image.Image:
    return Image.fromarray(as_rgb_array(img)).convert("RGBA")


def draw_registration(rgb, reg: ArenaRegistration, *, grid: bool = True) -> Image.Image:
    """只画网格线 + 格名（每 2 格标一次，避免糊成一片）。"""
    base = _rgba(rgb)
    layer = Image.new("RGBA", base.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    spec = reg.spec
    if grid:
        for block in spec.all_blocks():
            poly = [tuple(p) for p in reg.cell_polygon_px(block)]
            d.polygon(poly, outline=(0, 120, 255, 110))
        for r in range(1, spec.rows + 1):
            for c in range(0, spec.cols, 2):
                block = f"{spec.col_letters[c]}{r}"
                try:
                    u, v = reg.cell_center_px(block)
                except Exception:                       # noqa: BLE001
                    continue
                d.text((u - 6, v - 5), block, fill=(0, 90, 220, 230))
    return Image.alpha_composite(base, layer)


def draw_result(rgb, res, *, grid: bool = True, labels: bool = True) -> Image.Image:
    """画一帧 ``PerceptionResult``（对象/车/箭头 + 网格）。"""
    base = draw_registration(rgb, res.registration, grid=grid)
    layer = Image.new("RGBA", base.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    reg = res.registration

    if res.objects is not None:
        for det in res.objects.detections:
            color = CLASS_COLORS.get(det.cls, (255, 255, 0)) + (70,)
            d.polygon([tuple(p) for p in reg.cell_polygon_px(det.block)], fill=color)
            if labels:
                u, v = reg.cell_center_px(det.block)
                d.text((u - 10, v - 5), f"{det.cls[:3]} {det.block}",
                       fill=(20, 20, 20, 255))

    for arr in res.arrows:
        u, v = reg.cell_center_px(arr.block)
        d.ellipse([u - 14, v - 14, u + 14, v + 14], outline=(0, 160, 60, 240), width=2)
        if labels:
            d.text((u - 24, v + 15), f"arr {arr.block} {arr.dir_deg_map:.0f}deg",
                   fill=(0, 110, 40, 255))

    for car in res.cars:
        u, v = reg.cell_center_px(car.block)
        d.rectangle([u - 12, v - 12, u + 12, v + 12], outline=(255, 140, 0, 255), width=2)
        if labels:
            d.text((u - 20, v - 26), f"car {car.block} {car.yaw_deg:.0f}deg",
                   fill=(200, 90, 0, 255))

    for call in res.bridge_calls:
        u, v = reg.cell_center_px(call.block)
        d.rectangle([u - 16, v - 16, u + 16, v + 16], outline=(0, 160, 60, 255), width=3)
        if labels:
            d.text((u - 18, v + 18), f"{call.half}->{call.block}",
                   fill=(0, 110, 40, 255))
    return Image.alpha_composite(base, layer)
