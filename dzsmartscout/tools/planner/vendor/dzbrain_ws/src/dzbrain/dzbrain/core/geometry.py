# -*- coding: utf-8 -*-
"""坐标系与几何工具 —— 新版唯一坐标真源。

★ 本文件是**不可退让的契约**：公式与旧代码逐字一致，任何改动都会让车全盘错位。

坐标约定（与旧 `path_executor.block_to_meters` / `serial_reader` 完全一致）：
- 场地：14 列 A~N × 11 行 1~11，格边 BLOCK_SIZE = 0.35 m
- 米制原点在地图 N11 格的角上：
      x = (10.5 - row_idx) * 0.35      row 11 → 0.175, row 1 → 3.675
      y = (13.5 - col_idx) * 0.35      col N  → 0.175, col A → 4.725
  其中 row_idx = 行号-1, col_idx = 列字母索引（A=0 … N=13）
  即：N11 中心 = (0.175, 0.175)，A1 中心 = (3.675, 4.725)，N10 中心 = (0.525, 0.175)
- **格单位（block unit）**：米制 ÷ 0.35。`/odom/localization` 发的就是格单位！
      N10 = 格 (1.5, 0.5)  ← 里程计起点
- 航向：`/odom/localization` 的四元数带的是**原始车体 yaw**，
      map_yaw = raw_yaw + RAW_YAW_TO_MAP_YAW_OFFSET(+π/2)
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# --------------------------------------------------------------------------- #
#  常量
# --------------------------------------------------------------------------- #
COL_LETTERS = "ABCDEFGHIJKLMN"      # 14 列
MAP_COLS = 14
MAP_ROWS = 11
BLOCK_SIZE = 0.35                   # 格边（m）

MAP_X_MIN = 0.175                   # N/A 列中心线（米制）
MAP_X_MAX = 3.675
MAP_Y_MIN = 0.175
MAP_Y_MAX = 4.725

# 隔板（挡板）：第 6 行与第 7 行之间、D~N 列封死，只有 A/B/C 列能上下穿行。
# ★★ **全仓唯一真源**：`route/validate.py`、`route/planner.py`（经 `geo.`）、
#    `tools/{global_planner,route_check,shoot_planner}.py`（经 `tools/_field_const.py`）
#    一律从这里取值。改值只改这一处；任何工具/模块都不得再抄一份数字。
BARRIER_ROW_HI = 6                  # 挡板上侧行号（第 6 行）
BARRIER_ROW_LO = 7                  # 挡板下侧行号（第 7 行）
BARRIER_COL_FROM = "D"              # 从 D 列开始封死（含 D 列，一直到 N）

RAW_YAW_TO_MAP_YAW_OFFSET = math.pi / 2.0   # 原始 yaw → 地图 yaw（旧代码常量）

HEADING_X = 0.0                     # 朝 X+（行号增大方向）
HEADING_Y = math.pi / 2.0           # 朝 Y+（列号减小方向）


# --------------------------------------------------------------------------- #
#  格 ↔ 米
# --------------------------------------------------------------------------- #
def block_to_index(block: str) -> tuple[int, int]:
    """'K9' -> (row_idx, col_idx)，均 0-based。非法格名抛 ValueError。"""
    if not isinstance(block, str) or len(block) < 2:
        raise ValueError(f"非法格名: {block!r}")
    letter = block[0].upper()
    if letter not in COL_LETTERS:
        raise ValueError(f"非法列字母: {block!r}（应为 A~N）")
    try:
        number = int(block[1:])
    except ValueError as exc:
        raise ValueError(f"非法行号: {block!r}") from exc
    if not 1 <= number <= MAP_ROWS:
        raise ValueError(f"行号越界: {block!r}（应为 1~{MAP_ROWS}）")
    return number - 1, COL_LETTERS.index(letter)


def index_to_block(row_idx: int, col_idx: int) -> str:
    """(row_idx, col_idx) 0-based -> 'K9'（越界夹紧）。"""
    row_idx = max(0, min(MAP_ROWS - 1, int(row_idx)))
    col_idx = max(0, min(MAP_COLS - 1, int(col_idx)))
    return f"{COL_LETTERS[col_idx]}{row_idx + 1}"


def block_to_meters(block: str) -> tuple[float, float]:
    """格名 -> 该格中心的米制坐标 (x, y)。"""
    row_idx, col_idx = block_to_index(block)
    x = (10.5 - row_idx) * BLOCK_SIZE
    y = (13.5 - col_idx) * BLOCK_SIZE
    return x, y


def meters_to_block(x: float, y: float) -> str:
    """米制坐标 -> 最近的格名。"""
    row_idx = round(10.5 - x / BLOCK_SIZE)
    col_idx = round(13.5 - y / BLOCK_SIZE)
    return index_to_block(row_idx, col_idx)


def block_to_units(block: str) -> tuple[float, float]:
    """格名 -> 格单位坐标（= 米制 / 0.35）。用于与 /odom/localization 对齐。"""
    x, y = block_to_meters(block)
    return x / BLOCK_SIZE, y / BLOCK_SIZE


def units_to_meters(u: float, v: float) -> tuple[float, float]:
    """格单位 -> 米制。"""
    return u * BLOCK_SIZE, v * BLOCK_SIZE


def meters_to_units(x: float, y: float) -> tuple[float, float]:
    """米制 -> 格单位。"""
    return x / BLOCK_SIZE, y / BLOCK_SIZE


def map_bounds(margin: float = 0.0) -> tuple[float, float, float, float]:
    """(x_min, x_max, y_min, y_max)，可选外扩 margin（米）。"""
    return (MAP_X_MIN - margin, MAP_X_MAX + margin,
            MAP_Y_MIN - margin, MAP_Y_MAX + margin)


def in_bounds(x: float, y: float, margin: float = 0.0) -> bool:
    x_min, x_max, y_min, y_max = map_bounds(margin)
    return x_min <= x <= x_max and y_min <= y <= y_max


# --------------------------------------------------------------------------- #
#  角度
# --------------------------------------------------------------------------- #
def normalize_angle(angle: float) -> float:
    """折到 [-π, π)。"""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def ang_diff(a: float, b: float) -> float:
    """a-b 的最小角差，落在 [-π, π)。"""
    return normalize_angle(a - b)


def deg(rad: float) -> float:
    return math.degrees(rad)


def rad(degree: float) -> float:
    return math.radians(degree)


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """平面 yaw -> (qx, qy, qz, qw)（仅 z/w 非零）。"""
    half = yaw / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """四元数 -> 平面 yaw（与旧 `euler_from_quaternion` 等价）。"""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def heading_token_to_yaw(token: str) -> float:
    """',' 后缀航向 token -> yaw。'x'=X+ / 'y'=Y+ / 兜底 X+。"""
    token = (token or "").strip().lower()
    if token == "y":
        return HEADING_Y
    return HEADING_X


def heading_of(dx: float, dy: float, fallback: float = 0.0) -> float:
    """向量方向 -> yaw；零向量返回 fallback。"""
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return fallback
    return math.atan2(dy, dx)


# --------------------------------------------------------------------------- #
#  位姿
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Pose:
    """信念位姿（米制 + 地图 yaw）。"""

    x: float
    y: float
    yaw: float

    def block(self) -> str:
        return meters_to_block(self.x, self.y)

    def offset_to(self, x: float, y: float) -> tuple[float, float, float]:
        """到 (x, y) 的相对向量与距离 -> (dx, dy, dist)。"""
        dx, dy = x - self.x, y - self.y
        return dx, dy, math.hypot(dx, dy)

    def translated(self, dx: float, dy: float) -> "Pose":
        return Pose(self.x + dx, self.y + dy, self.yaw)

    def rotated(self, dyaw: float) -> "Pose":
        return Pose(self.x, self.y, normalize_angle(self.yaw + dyaw))
