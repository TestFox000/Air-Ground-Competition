# -*- coding: utf-8 -*-
r"""无人机侦察视场模型（步进循环协议 v2 §4.1）。

物理模型（**仿真假设，非现场标定值**）：
- 四旋翼在 **1.5 m** 高度俯视，地面足迹 = **4 格 x 2 格** = 1.40 m x 0.70 m
  （`BLOCK_SIZE=0.35`，与 `core.geometry` 同一真源）；
- 航向 0°（+x）时足迹为 **2 列 x 4 行**；航向 90°（+y）时旋转为 **4 列 x 2 行**；
- 任意 yaw：足迹是绕 pose 旋转的矩形，取**格心严格落在矩形内**的格
  （`|u| < 0.7` 且 `|v| < 0.35`，u=沿航向、v=横向）。

★ 为什么是"格心严格在内"而不是"矩形与格有交"：
  1. 与 `route/planner.py` 的格心判据同源（车/机同一口径，避免半格歧义）；
  2. 只有 pose 落在**格点**（0.35 m 整数倍）时，0°/90° 才**恰好** 8 格 ——
     这正是 `frames.py` 把帧位放在格点上的原因（也是"2x4=8 格"成立的唯一姿态）；
  3. pose 落在格心时，4x2 的矩形边界穿过相邻格心，会变成 15 格（5x3）或 9 格，
     与协议"2x4=8 格"不符。

姿态与坐标：`Pose=(x, y, yaw)`，米制/弧度，遵循 `core.geometry`：
  x=(10.5-行索引)*0.35、y=(13.5-列索引)*0.35；yaw 0=+x，yaw pi/2=+y。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from functools import lru_cache
from typing import Iterable

from ..core import geometry as geo

#: 视场沿航向的格数（4 格 = 1.40 m）
FOV_LENGTH_CELLS = 4
#: 视场横向的格数（2 格 = 0.70 m）
FOV_WIDTH_CELLS = 2
FOV_LENGTH_M = FOV_LENGTH_CELLS * geo.BLOCK_SIZE
FOV_WIDTH_M = FOV_WIDTH_CELLS * geo.BLOCK_SIZE
FOV_HALF_LENGTH_M = FOV_LENGTH_M / 2.0
FOV_HALF_WIDTH_M = FOV_WIDTH_M / 2.0
#: 侦察高度假设（m）。本模型不据此推导张角，只作为协议/报告的显式假设。
SCOUT_ALTITUDE_M = 1.5

_EPS = 1e-9


@dataclass(frozen=True)
class Pose:
    """无人机姿态（米制/弧度）。`block` 只用于日志与"最近格"展示。"""

    x: float
    y: float
    yaw: float

    @property
    def block(self) -> str:
        return geo.meters_to_block(self.x, self.y)

    @property
    def yaw_deg(self) -> float:
        return round(math.degrees(self.yaw), 3)

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.yaw)

    def moved(self, dx: float = 0.0, dy: float = 0.0,
              dyaw: float = 0.0) -> "Pose":
        return Pose(self.x + dx, self.y + dy, self.yaw + dyaw)


@lru_cache(maxsize=1)
def cell_centers() -> tuple[tuple[str, float, float], ...]:
    """全部 154 格的 `(格名, 格心 x, 格心 y)`，行优先（行 1..11 x 列 A..N）。"""
    out: list[tuple[str, float, float]] = []
    for row in range(1, geo.MAP_ROWS + 1):
        for col in geo.COL_LETTERS:
            block = f"{col}{row}"
            x, y = geo.block_to_meters(block)
            out.append((block, x, y))
    return tuple(out)


@lru_cache(maxsize=1)
def all_blocks() -> tuple[str, ...]:
    """全部格名（行优先）。"""
    return tuple(b for b, _, _ in cell_centers())


def block_key(block: str) -> tuple[int, int]:
    """行优先排序键：先行号（1-based 升序）再列索引。"""
    row_idx, col_idx = geo.block_to_index(str(block).strip().upper())
    return (row_idx + 1, col_idx)


def sorted_blocks(blocks: Iterable[str]) -> list[str]:
    """去重 + 行优先排序（布局报文的确定性顺序）。"""
    return sorted({str(b).strip().upper() for b in blocks if str(b).strip()},
                  key=block_key)


def in_bounds(row_idx: int, col_idx: int) -> bool:
    return 0 <= row_idx < geo.MAP_ROWS and 0 <= col_idx < geo.MAP_COLS


def cells_in_view(pose: Pose) -> tuple[str, ...]:
    """返回该姿态视场覆盖的格名（行优先、去重）。

    判据：格心到 pose 的局部坐标 `(u, v)` 满足 `|u| < 0.7, |v| < 0.35`。
    0°/90° + 格点姿态 => 恰好 8 格；边界处会被场地裁剪（这是物理事实，不补格）。
    """
    hx, hy = math.cos(pose.yaw), math.sin(pose.yaw)
    rx, ry = -hy, hx
    out: list[str] = []
    for block, cx, cy in cell_centers():
        dx, dy = cx - pose.x, cy - pose.y
        u = dx * hx + dy * hy
        v = dx * rx + dy * ry
        if abs(u) < FOV_HALF_LENGTH_M - _EPS and abs(v) < FOV_HALF_WIDTH_M - _EPS:
            out.append(block)
    return tuple(out)


#: 兼容别名：语义就是"这一帧看到了哪些格"。
view_blocks = cells_in_view


def footprint_corners(pose: Pose) -> tuple[tuple[float, float], ...]:
    """视场矩形的 4 个角（米制，逆时针；用于重叠面积计算/绘图）。"""
    hx, hy = math.cos(pose.yaw), math.sin(pose.yaw)
    rx, ry = -hy, hx
    pts: list[tuple[float, float]] = []
    for a, b in ((FOV_HALF_LENGTH_M, FOV_HALF_WIDTH_M),
                 (-FOV_HALF_LENGTH_M, FOV_HALF_WIDTH_M),
                 (-FOV_HALF_LENGTH_M, -FOV_HALF_WIDTH_M),
                 (FOV_HALF_LENGTH_M, -FOV_HALF_WIDTH_M)):
        pts.append((pose.x + a * hx + b * rx, pose.y + a * hy + b * ry))
    return tuple(pts)


def polygon_area(points: Iterable[tuple[float, float]]) -> float:
    """多边形面积（鞋带公式，绝对值）。"""
    pts = list(points)
    if len(pts) < 3:
        return 0.0
    total = 0.0
    for i, (x1, y1) in enumerate(pts):
        x2, y2 = pts[(i + 1) % len(pts)]
        total += x1 * y2 - x2 * y1
    return abs(total) / 2.0


def _inside(p: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> bool:
    return ((b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])) >= 0.0


def _intersect(p: tuple[float, float], q: tuple[float, float],
               a: tuple[float, float], b: tuple[float, float]) -> tuple[float, float]:
    x1, y1 = p
    x2, y2 = q
    x3, y3 = a
    x4, y4 = b
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(den) < 1e-12:
        return q
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
    return (x1 + t * (x2 - x1), y1 + t * (y2 - y1))


def clip_convex(subject: Iterable[tuple[float, float]],
                clipper: Iterable[tuple[float, float]]) -> list[tuple[float, float]]:
    """Sutherland-Hodgman：两个凸多边形求交（这里只用于视场矩形）。"""
    out = list(subject)
    clip = list(clipper)
    if not out or len(clip) < 3:
        return []
    for i in range(len(clip)):
        a, b = clip[i], clip[(i + 1) % len(clip)]
        inp, out = out, []
        if not inp:
            break
        s = inp[-1]
        for p in inp:
            if _inside(p, a, b):
                if not _inside(s, a, b):
                    out.append(_intersect(s, p, a, b))
                out.append(p)
            elif _inside(s, a, b):
                out.append(_intersect(s, p, a, b))
            s = p
    return out


def overlap_fraction(a: Pose, b: Pose) -> float:
    """相邻帧足迹的**物理面积重叠率** = 交集面积 / 单帧面积（0..1）。"""
    area_a = polygon_area(footprint_corners(a))
    if area_a <= 0.0:
        return 0.0
    inter = clip_convex(footprint_corners(a), footprint_corners(b))
    return polygon_area(inter) / area_a


def cell_overlap_fraction(a: Pose, b: Pose) -> float:
    """相邻帧**格集合**的 Jaccard（诊断用；协议判据用物理面积）。"""
    sa, sb = set(cells_in_view(a)), set(cells_in_view(b))
    union = sa | sb
    if not union:
        return 0.0
    return len(sa & sb) / len(union)


def is_node(pose: Pose, eps: float = 1e-6) -> bool:
    """姿态是否落在格点上（0.35 m 整数倍）—— 0°/90° 恰好 8 格的前提。"""
    for value in (pose.x, pose.y):
        q = value / geo.BLOCK_SIZE
        if abs(q - round(q)) > eps / geo.BLOCK_SIZE:
            return False
    return True


__all__ = [
    "FOV_LENGTH_CELLS", "FOV_WIDTH_CELLS", "FOV_LENGTH_M", "FOV_WIDTH_M",
    "FOV_HALF_LENGTH_M", "FOV_HALF_WIDTH_M", "SCOUT_ALTITUDE_M",
    "Pose", "cell_centers", "all_blocks", "block_key", "sorted_blocks",
    "in_bounds", "cells_in_view", "view_blocks", "footprint_corners",
    "polygon_area", "clip_convex", "overlap_fraction",
    "cell_overlap_fraction", "is_node",
]
