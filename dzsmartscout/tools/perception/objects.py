# -*- coding: utf-8 -*-
r"""场地目标（雷 / 障 / 靶区）→ 格名列表 → ``field_verify{...}`` / ``target_zone{...}``。

## 两套几何口径（**必须区分，否则面积判据全错**）

| 口径 | 雷 | 障 | 靶 |
|---|---|---|---|
| **图纸**（``config/map.yaml`` 识图） | 0.9 格**黄方块** | 满格**红方块** | 品红方块 |
| **实物**（说明书 p5/p6/p7） | **直径 12 cm 彩色打印圆片**平贴地面 | **彩色带盖水桶**（约 30 cm，有高度） | **低矮黑色装置**（形态未确认） |

所以本模块的判据全部用**"面积 / 该格图像面积"**与**填充率**表达，并给两套预设：
:data:`MAP_IMAGE_SPECS`（图纸，方块）与 :data:`FIELD_SPECS`（实物，圆盘/桶）。
12 cm 圆片在 35 cm 格里只占 ``π·6²/35² ≈ 9.2%``——**用图纸的"接近满格"去套实物必然漏检**。

## 颜色：只有箭头的绿色有实测证据

说明书只说"**彩色**地雷图片""**彩色**带盖水桶"，没有给可用颜色；``p09_1.png`` 里
红/青/绿三种桶同框出现 ⇒ **水桶颜色不唯一，单色阈值不可靠**。因此：

* 本模块不猜颜色，颜色由 ``Palette`` 决定（现场标定）；
* 全部格名都带 ``confidence`` 与 ``evidence``，下游可以按阈值取舍；
* ``target_zone`` 只输出**格名列表**（与车侧 ``drone.py:292-304`` 一致：它拿
  ``blocks`` 与本地 ``map.yaml.targets`` 做差集核对，不解释几何）。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, Sequence

import numpy as np

from .colors import Palette, as_rgb_array, blob_masks
from .grid import ArenaRegistration, GridSpec

__all__ = [
    "ObjectClassSpec",
    "ObjectDetection",
    "FieldObjects",
    "MAP_IMAGE_SPECS",
    "FIELD_SPECS",
    "detect_field_objects",
    "group_blocks",
]


@dataclass(frozen=True)
class ObjectClassSpec:
    """一类目标的几何判据（面积与填充率都以**该格的图像面积**归一）。"""

    cls: str
    min_cell_frac: float
    max_cell_frac: float
    fill_range: tuple[float, float] = (0.0, 1.01)
    k_open: int = 0
    k_close: int = 1
    min_pixels: int = 8
    priority: int = 0          # 同格冲突时大者胜
    max_dark_frac: float | None = None   # bbox 内"深色像素"占比上限（挡带文字的标注框）
    dark_luma: float = 150.0
    note: str = ""


#: 图纸口径：色块 ≈ 0.9 格方块（实测 59 px / 64 px 格 ⇒ cell_frac≈0.85，fill≈1.0）
#:
#: ★ 雷的 ``max_dark_frac`` 来自**实测**（``map_src.png``，同色黄块逐个量）：
#:   8 个真雷的 bbox 内深色像素（luma<150）占比 ≤0.0014（≤5 px）；
#:   3 个"起终点/无人机点"黄色**标注框**（框内印字）占比 0.086~0.105（352~422 px）；
#:   ⇒ 阈值取 0.02 有 60 倍余量。这就是 t28 识图脚本"证据 C"的口径
#:   （``identify_t28_v2.py:117-131`` 用块内深色像素区分"雷"与"标注框"）。
MAP_IMAGE_SPECS: tuple[ObjectClassSpec, ...] = (
    ObjectClassSpec("obstacle", 0.55, 1.25, (0.78, 1.01), priority=3,
                    note="图纸：满格红块"),
    ObjectClassSpec("mine", 0.55, 1.25, (0.78, 1.01), priority=2,
                    max_dark_frac=0.02,
                    note="图纸：0.9 格黄块；带文字的黄框（起终点/无人机点）按深色像素占比剔除"),
    ObjectClassSpec("target", 0.55, 1.25, (0.78, 1.01), priority=1,
                    note="图纸：品红靶区；相邻两格=一块跨格靶板"),
    ObjectClassSpec("bridge", 0.55, 1.25, (0.78, 1.01), priority=0,
                    note="图纸：绿色『通』格 = 桥洞可通行格（map.yaml.bridges）"),
)

#: 实物口径：雷=12 cm 圆片（cell_frac≈0.09）、桶≈30 cm（≈0.57）、靶形态未知
FIELD_SPECS: tuple[ObjectClassSpec, ...] = (
    ObjectClassSpec("obstacle", 0.25, 0.95, (0.55, 1.01), priority=3, k_close=1,
                    note="实物：水桶俯视近似圆，直径约 30 cm ⇒ cell_frac≈0.57"),
    ObjectClassSpec("mine", 0.03, 0.30, (0.55, 0.98), priority=2, k_close=1,
                    note="实物：12 cm 圆片 ⇒ cell_frac≈0.092"),
    ObjectClassSpec("target", 0.03, 1.25, (0.05, 1.01), priority=1,
                    note="实物：低矮黑色装置，形态未确认 ⇒ 判据放宽"),
)


@dataclass
class ObjectDetection:
    cls: str
    block: str
    row_idx: int
    col_idx: int
    half: str
    x_m: float
    y_m: float
    area_px: int
    area_frac: float
    fill: float
    confidence: float
    evidence: dict = field(default_factory=dict)

    def to_json(self) -> dict:
        return {"cls": self.cls, "block": self.block, "half": self.half,
                "x_m": round(self.x_m, 3), "y_m": round(self.y_m, 3),
                "area_px": self.area_px, "area_frac": round(self.area_frac, 3),
                "fill": round(self.fill, 3),
                "confidence": round(self.confidence, 3),
                "evidence": dict(self.evidence)}


@dataclass
class FieldObjects:
    """一帧的场地目标集合（格名一律大写，按类名分组）。"""

    by_class: dict[str, list[str]] = field(default_factory=dict)
    target_groups: list[list[str]] = field(default_factory=list)
    detections: list[ObjectDetection] = field(default_factory=list)
    rejected: list[dict] = field(default_factory=list)
    conflicts: list[str] = field(default_factory=list)

    # ---- 便捷视图（车侧协议只用到前三个）----
    @property
    def mines(self) -> list[str]:
        return self.by_class.get("mine", [])

    @property
    def obstacles(self) -> list[str]:
        return self.by_class.get("obstacle", [])

    @property
    def targets(self) -> list[str]:
        return self.by_class.get("target", [])

    @property
    def bridges(self) -> list[str]:
        return self.by_class.get("bridge", [])

    def blocks_of(self, cls: str) -> list[str]:
        return self.by_class.get(cls, [])

    def to_json(self) -> dict:
        return {
            "by_class": {k: sorted(v) for k, v in sorted(self.by_class.items())},
            "target_groups": [sorted(g) for g in self.target_groups],
            "detections": [d.to_json() for d in self.detections],
            "rejected": list(self.rejected), "conflicts": list(self.conflicts),
        }


def _mid_confidence(value: float, lo: float, hi: float) -> float:
    """落在区间中部给 1.0，贴边给 0.4。"""
    if hi <= lo:
        return 1.0
    pos = (value - lo) / (hi - lo)
    pos = min(1.0, max(0.0, pos))
    return float(0.4 + 0.6 * (1.0 - abs(pos - 0.5) * 2.0))


def group_blocks(blocks: Iterable[str], grid: GridSpec | None = None) -> list[list[str]]:
    """把相邻（4 邻接）格名聚成组 —— 规则 p6「相连的两个横向或纵向单元」。"""
    grid = grid or GridSpec()
    cells = {}
    for b in blocks:
        r, c = grid.block_to_index(b)
        cells[(r, c)] = b.upper()
    groups: list[list[str]] = []
    seen: set[tuple[int, int]] = set()
    for start in list(cells):
        if start in seen:
            continue
        stack, comp = [start], []
        seen.add(start)
        while stack:
            r, c = stack.pop()
            comp.append(cells[(r, c)])
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nb = (r + dr, c + dc)
                if nb in cells and nb not in seen:
                    seen.add(nb)
                    stack.append(nb)
        groups.append(sorted(comp))
    groups.sort(key=lambda g: (-len(g), g))
    return groups


def detect_field_objects(rgb, reg: ArenaRegistration, palette: Palette,
                         specs: Sequence[ObjectClassSpec] = FIELD_SPECS,
                         *, min_confidence: float = 0.30) -> FieldObjects:
    """颜色掩膜 → 连通域 → 面积/填充率判据 → 格名（含冲突消解）。"""
    arr = as_rgb_array(rgb)
    masks = palette.masks(arr)
    luma = arr.astype(np.float64) @ np.array([0.299, 0.587, 0.114])
    out = FieldObjects()
    for spec in specs:
        if spec.cls not in masks:
            continue
        blobs = blob_masks(masks[spec.cls], min_pixels=spec.min_pixels,
                           k_open=spec.k_open, k_close=spec.k_close, rgb=arr)
        for blob in blobs:
            if blob.pixels is None:
                continue
            pts = reg.px_to_meters(blob.pixels[:, ::-1].astype(np.float64))
            centroid = pts.mean(axis=0)
            hit = reg.meters_to_hit(float(centroid[0]), float(centroid[1]))
            cell_area = reg.cell_area_px(hit.block)
            area_frac = blob.area / cell_area if cell_area > 0 else 0.0
            fill = blob.fill
            dark_frac = 0.0
            if spec.max_dark_frac is not None:
                sub = luma[blob.y0:blob.y1 + 1, blob.x0:blob.x1 + 1]
                dark_frac = float((sub < spec.dark_luma).mean())
            if not (spec.min_cell_frac <= area_frac <= spec.max_cell_frac):
                out.rejected.append({
                    "cls": spec.cls, "block": hit.block,
                    "reason": f"面积比 {area_frac:.3f} 不在 "
                              f"[{spec.min_cell_frac}, {spec.max_cell_frac}]",
                    "area_px": int(blob.area)})
                continue
            if not (spec.fill_range[0] <= fill <= spec.fill_range[1]):
                out.rejected.append({
                    "cls": spec.cls, "block": hit.block,
                    "reason": f"填充率 {fill:.3f} 不在 {list(spec.fill_range)}",
                    "area_px": int(blob.area)})
                continue
            if spec.max_dark_frac is not None and dark_frac > spec.max_dark_frac:
                out.rejected.append({
                    "cls": spec.cls, "block": hit.block,
                    "reason": f"bbox 内深色像素占比 {dark_frac:.4f} > "
                              f"{spec.max_dark_frac}（判为带文字的标注框，非目标）",
                    "area_px": int(blob.area), "dark_frac": round(dark_frac, 4)})
                continue
            conf = 0.5 * _mid_confidence(area_frac, spec.min_cell_frac, spec.max_cell_frac) \
                + 0.5 * _mid_confidence(fill, spec.fill_range[0], spec.fill_range[1])
            if not hit.in_arena:
                conf *= 0.5
            out.detections.append(ObjectDetection(
                cls=spec.cls, block=hit.block, row_idx=hit.row_idx,
                col_idx=hit.col_idx, half=hit.half, x_m=hit.x_m, y_m=hit.y_m,
                area_px=int(blob.area), area_frac=float(area_frac), fill=float(fill),
                confidence=float(conf),
                evidence={"bbox": [blob.x0, blob.y0, blob.x1, blob.y1],
                          "cell_px_area": round(cell_area, 1),
                          "dark_frac": round(dark_frac, 4),
                          "in_arena": hit.in_arena, "note": spec.note}))

    # ---- 同格冲突：优先级 → 面积 → 置信度 ----
    prio = {s.cls: s.priority for s in specs}
    best: dict[str, ObjectDetection] = {}
    for det in sorted(out.detections,
                      key=lambda d: (-prio.get(d.cls, 0), -d.area_px, -d.confidence)):
        if det.block in best:
            keep = best[det.block]
            out.conflicts.append(
                f"{det.block}: 保留 {keep.cls}(prio={prio.get(keep.cls, 0)},"
                f"area={keep.area_px}) / 丢弃 {det.cls}(prio={prio.get(det.cls, 0)},"
                f"area={det.area_px})")
            continue
        best[det.block] = det
    out.detections = [d for d in out.detections if best.get(d.block) is d]

    keep = [d for d in out.detections if d.confidence >= min_confidence]
    grouped: dict[str, list[str]] = {}
    for det in keep:
        grouped.setdefault(det.cls, []).append(det.block)
    out.by_class = {k: sorted(v) for k, v in grouped.items()}
    out.target_groups = group_blocks(out.targets, reg.spec)
    return out
