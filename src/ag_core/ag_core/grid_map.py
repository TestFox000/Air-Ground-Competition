"""比赛场地网格模型：格号 <-> 米制坐标 <-> 图像像素。

场地实测（来自《场地布置说明书V2》的图 2，并经车端代码复核）：
    场地 4 m x 5 m，单元格 35 cm x 35 cm
    **14 列（字母 A~N，沿 5 m 边，14*0.35 = 4.90 m）x 11 行（数字 1~11，沿 4 m 边，3.85 m）**

依据（当初"11 列 A~K x 14 行"的推断是错的，已被实机代码推翻）：
    - 车端 `~/dzrobot/src/car_localization/.../path_planner.py` 与 `path_executor.py`
      都写死 `COL_LETTERS='ABCDEFGHIJKLMN'`(14) / `MAP_COLS=14` / `MAP_ROWS=11`；
    - 该实现 `block_to_meters('M10')` → x=(10.5-row)*0.35、y=(13.5-col)*0.35，
      即 **x 沿数字行方向、y 沿字母列方向**，原点在 N11 格的右下角；
    - 说明书原文出现过的 `K14`、`N10` 之类格号在"1~14 行"下本就无法自洽
      （图 2 里可读到的最大行号是 11）。

格号示例（说明书原文）：D11 / B7 / F7 / D4（地雷）、D2+E2（敌我区）、K4 / K9（桥洞通行标识）。

⚠️ 本模块内部约定：x 沿**列**方向、y 沿**行**方向（与图像/单应标定一致）。
   与车端 `/nav/waypoint` 的 x/y 轴是对调的，转换统一走
   `ag_car.route_builder.cell_to_car_xy()`，不要在别处手写。
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np

__all__ = ["FieldGrid", "CellID"]

_LETTERS = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"


@dataclass(frozen=True)
class CellID:
    """格号，例如 'D11' -> row=10(0基), col=3(0基)。"""

    row: int  # 0 基行号（打印时 +1）
    col: int  # 0 基列号

    @property
    def name(self) -> str:
        return f"{_LETTERS[self.col]}{self.row + 1}"

    @classmethod
    def parse(cls, name: str) -> "CellID":
        s = name.strip().upper()
        col = 0
        i = 0
        while i < len(s) and s[i].isalpha():
            i += 1
        letters = s[:i]
        digits = s[i:]
        for ch in letters:
            col = col * 26 + (_LETTERS.index(ch) + 1)
        col -= 1
        return cls(row=int(digits) - 1, col=col)

    @classmethod
    def from_index(cls, row: int, col: int) -> "CellID":
        return cls(row=row, col=col)

    def __str__(self) -> str:  # pragma: no cover - 简单转发
        return self.name


class FieldGrid:
    """占据栅格 + 格号换算 + 路径搜索。"""

    FREE = 0
    BLOCKED = 1
    UNKNOWN = 2

    def __init__(
        self,
        cols: int = 14,
        rows: int = 11,
        cell_size: float = 0.35,
        turn_radius: float = 0.26,
    ) -> None:
        self.cols = cols
        self.rows = rows
        self.cell_size = cell_size
        self.turn_radius = turn_radius  # SmartMaster 最小转弯半径 240~260 mm
        self.grid = np.zeros((rows, cols), dtype=np.uint8)
        self._H: Optional[np.ndarray] = None  # 像素 -> 场地米制 的单应矩阵

    def contains(self, cid: CellID) -> bool:
        """格号是否落在本场地内。

        现场最常见的错误就是把行号写到 12~14（那是旧版推断的产物），
        这种格号能解析、能打印，但车端 `path_executor` 会直接拒收整条路线，
        所以宁可在这里显式拦下。
        """
        return 0 <= cid.row < self.rows and 0 <= cid.col < self.cols

    # ---------------- 坐标换算 ----------------
    @property
    def width_m(self) -> float:
        return self.cols * self.cell_size

    @property
    def length_m(self) -> float:
        return self.rows * self.cell_size

    def cell_center(self, cid: CellID) -> Tuple[float, float]:
        """返回格中心 (x, y)，单位米。x 沿列方向，y 沿行方向。"""
        return ((cid.col + 0.5) * self.cell_size, (cid.row + 0.5) * self.cell_size)

    def cell_of_point(self, x: float, y: float) -> Optional[CellID]:
        # 加 1e-9 容差：单应反算在边界上会产生 -1e-16 级误差，
        # 直接用 // 会把格中心/角点判到界外。
        c = int(np.floor(x / self.cell_size + 1e-9))
        r = int(np.floor(y / self.cell_size + 1e-9))
        if 0 <= r < self.rows and 0 <= c < self.cols:
            return CellID(r, c)
        return None

    def in_bounds(self, cid: CellID) -> bool:
        return 0 <= cid.row < self.rows and 0 <= cid.col < self.cols

    def is_free(self, cid: CellID) -> bool:
        return self.in_bounds(cid) and self.grid[cid.row, cid.col] == self.FREE

    # ---------------- 占据状态 ----------------
    def set_blocked(self, cid: CellID) -> None:
        if self.in_bounds(cid):
            self.grid[cid.row, cid.col] = self.BLOCKED

    def set_free(self, cid: CellID) -> None:
        if self.in_bounds(cid):
            self.grid[cid.row, cid.col] = self.FREE

    def reset(self, value: int = FREE) -> None:
        self.grid[:] = value

    def blocked_cells(self) -> List[str]:
        r, c = np.nonzero(self.grid == self.BLOCKED)
        return [CellID(int(a), int(b)).name for a, b in zip(r, c)]

    # ---------------- 单应矩阵（无人机高空视角 -> 场地坐标） ----------------
    def set_homography(self, H: np.ndarray) -> None:
        H = np.asarray(H, dtype=np.float64)
        if H.shape != (3, 3):
            raise ValueError("H 必须是 3x3")
        self._H = H

    @property
    def has_homography(self) -> bool:
        return self._H is not None

    def pixel_to_field(self, u: float, v: float) -> Tuple[float, float]:
        """图像像素 -> 场地米制坐标。需先 set_homography。"""
        if self._H is None:
            raise RuntimeError("未标定单应矩阵，请先 set_homography()")
        p = np.array([u, v, 1.0], dtype=np.float64)
        q = self._H @ p
        if abs(q[2]) < 1e-12:
            raise ValueError("单应变换退化")
        return float(q[0] / q[2]), float(q[1] / q[2])

    def pixel_to_cell(self, u: float, v: float) -> Optional[CellID]:
        x, y = self.pixel_to_field(u, v)
        return self.cell_of_point(x, y)

    # ---------------- 路径搜索 ----------------
    def neighbors(self, cid: CellID, diagonal: bool = False) -> List[CellID]:
        steps = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        if diagonal:
            steps += [(1, 1), (1, -1), (-1, 1), (-1, -1)]
        out = []
        for dr, dc in steps:
            n = CellID(cid.row + dr, cid.col + dc)
            if self.is_free(n):
                out.append(n)
        return out

    def bfs_path(self, start: CellID, goal: CellID, diagonal: bool = False) -> Optional[List[CellID]]:
        """最短格数路径（忽略转弯代价）。起点不可达时返回 None。"""
        if not self.is_free(goal):
            return None
        if start == goal:
            return [start]
        prev: Dict[CellID, Optional[CellID]] = {start: None}
        q: Deque[CellID] = deque([start])
        while q:
            cur = q.popleft()
            for n in self.neighbors(cur, diagonal):
                if n in prev:
                    continue
                prev[n] = cur
                if n == goal:
                    path: List[CellID] = []
                    node: Optional[CellID] = n
                    while node is not None:
                        path.append(node)
                        node = prev[node]
                    return path[::-1]
                q.append(n)
        return None

    def path_length_m(self, path: List[CellID]) -> float:
        """按实际步长累加：对角步是 √2 格，不能一律按 1 格算。

        现在 bfs_path 只走四邻域，但这条公式一旦被用在 diagonal=True 的路径上，
        就会把路径长低估最多 29%（时间预算跟着失真）——必须按步算。
        """
        total = 0.0
        for a, b in zip(path[:-1], path[1:]):
            dr = abs(a.row - b.row)
            dc = abs(a.col - b.col)
            total += self.cell_size * math.hypot(dr, dc)
        return total

    def count_turns(self, path: List[CellID]) -> int:
        if len(path) < 3:
            return 0
        turns = 0
        for a, b, c in zip(path[:-2], path[1:-1], path[2:]):
            d1 = (b.row - a.row, b.col - a.col)
            d2 = (c.row - b.row, c.col - b.col)
            if d1 != d2:
                turns += 1
        return turns

    def turn_feasible(self, path: List[CellID]) -> bool:
        """连续直角转弯需要 2*R 的通道宽度；通道不足时判定不可行。

        SmartMaster 最小转弯半径 240~260 mm，而单元格仅 350 mm，
        因此「连续两个反向直角弯」在单格通道内基本无法完成。
        """
        if len(path) < 3:
            return True
        need = 2.0 * self.turn_radius
        for a, b, c in zip(path[:-2], path[1:-1], path[2:]):
            d1 = (b.row - a.row, b.col - a.col)
            d2 = (c.row - b.row, c.col - b.col)
            if d1 == d2:
                continue
            # 直角转弯：通道沿两个方向都必须是单格（<2*R 的余量极小）
            if abs(d1[0]) + abs(d1[1]) == 1 and abs(d2[0]) + abs(d2[1]) == 1:
                corridor = min(self.cell_size, self.cell_size)
                if corridor < need:
                    # 单格通道内的直角弯：仅当两侧有额外一格余量才算可行
                    if not self._has_turn_margin(b, d1, d2):
                        return False
        return True

    def _has_turn_margin(self, at: CellID, d1: Tuple[int, int], d2: Tuple[int, int]) -> bool:
        """转弯处的外侧是否还有空格可作为转弯余量。"""
        for dr, dc in (d1, d2):
            side1 = CellID(at.row + dc, at.col + dr)
            side2 = CellID(at.row - dc, at.col - dr)
            if self.is_free(side1) or self.is_free(side2):
                return True
        return False

    # ---------------- 批量导入 ----------------
    def apply_blocked_names(self, names: List[str]) -> None:
        for n in names:
            self.set_blocked(CellID.parse(n))

    def to_dict(self) -> dict:
        return {
            "cols": self.cols,
            "rows": self.rows,
            "cell_size": self.cell_size,
            "blocked": self.blocked_cells(),
        }
