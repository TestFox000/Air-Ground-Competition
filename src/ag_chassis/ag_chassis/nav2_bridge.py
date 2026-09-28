"""格号 <-> Nav2 目标位姿 的换算（无 ROS 依赖，可单测）。

为什么需要：比赛车的底盘由 Nav2 驱动（激光雷达 → AMCL → Nav2 → 底盘），
Nav2 要的是 map 坐标系下的目标位姿，而任务规划给的是格号（如 "C3"）。
两者之间差一个「栅格坐标系 → map 坐标系」的刚体变换：

    x_map = ox + (cosθ * x_grid - sinθ * y_grid)
    y_map = oy + (sinθ * x_grid + cosθ * y_grid)

四个数（ox, oy, theta, 以及栅格 cell_size）必须在现场标定一次：
让车站在 A1 格中心，读 AMCL 的 /amcl_pose，与栅格原点 (0.175, 0.175) 对比即可。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

__all__ = ["FrameAlign", "cell_center_xy", "cell_to_map_pose"]


def cell_center_xy(cell: str, cell_size: float = 0.35, cols: str = "ABCDEFGHIJKLMNOPQRSTUVWXYZ") -> Tuple[float, float]:
    """格号 -> 栅格坐标系下的中心 (x, y)，单位米。x 沿列（A→），y 沿行（1→）。"""
    s = cell.strip().upper()
    i = 0
    while i < len(s) and s[i].isalpha():
        i += 1
    letters, digits = s[:i], s[i:]
    if not letters or not digits:
        raise ValueError(f"无法解析格号: {cell!r}")
    col = 0
    for ch in letters:
        col = col * 26 + (cols.index(ch) + 1)
    col -= 1
    row = int(digits) - 1
    return ((col + 0.5) * cell_size, (row + 0.5) * cell_size)


@dataclass
class FrameAlign:
    """栅格坐标系 -> map 坐标系的对齐参数。"""

    origin_x: float = 0.0        # 栅格原点在 map 中的 x
    origin_y: float = 0.0        # 栅格原点在 map 中的 y
    yaw_rad: float = 0.0         # 栅格坐标系相对 map 的旋转
    cell_size: float = 0.35
    scale: float = 1.0           # 栅格米制与 map 米制的比例，通常为 1

    def to_map(self, x_grid: float, y_grid: float) -> Tuple[float, float]:
        c, s = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        x = self.scale * (c * x_grid - s * y_grid)
        y = self.scale * (s * x_grid + c * y_grid)
        return (self.origin_x + x, self.origin_y + y)

    def cell_to_map(self, cell: str) -> Tuple[float, float]:
        x, y = cell_center_xy(cell, self.cell_size)
        return self.to_map(x, y)


def cell_to_map_pose(cell: str, align: FrameAlign, yaw: float = 0.0) -> Tuple[float, float, float]:
    """格号 -> (x_map, y_map, yaw)。返回的三元组可直接填进 PoseStamped。"""
    x, y = align.cell_to_map(cell)
    return (x, y, yaw)
