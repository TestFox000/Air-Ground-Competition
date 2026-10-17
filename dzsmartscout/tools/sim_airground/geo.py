# -*- coding: utf-8 -*-
"""格名 ↔ 米制坐标（**逐字沿用**车侧 ``dzbrain.core.geometry`` 的公式）。

真源：``dzrobot_v2/src/dzbrain/dzbrain/core/geometry.py``（与 t2 ``perception/grid.py``
文件头的坐标语义一致）：

    x = (10.5 - row_idx) * 0.35      row 11 → 0.175, row 1 → 3.675
    y = (13.5 - col_idx) * 0.35      col N  → 0.175, col A → 4.725

原点 = N11 格外角；+x 指向行号减小（下半场 → 上半场）；+y 指向列号减小（N → A）。
"""

from __future__ import annotations

COL_LETTERS = "ABCDEFGHIJKLMN"
MAP_ROWS = 11
MAP_COLS = 14
BLOCK_M = 0.35


def block_to_index(block: str) -> tuple[int, int]:
    """'K9' → (row_idx 0-based, col_idx 0-based)；非法格名抛 ``ValueError``。"""
    text = str(block).strip().upper()
    if not text or len(text) < 2:
        raise ValueError(f"非法格名：{block!r}")
    col = COL_LETTERS.index(text[0])
    row = int(text[1:]) - 1
    if not (0 <= row < MAP_ROWS and 0 <= col < MAP_COLS):
        raise ValueError(f"格名出界：{block!r}")
    return row, col


def index_to_block(row_idx: int, col_idx: int) -> str:
    """(row_idx, col_idx) 0-based → 'K9'（越界夹紧，与车侧 index_to_block 同）。"""
    r = max(0, min(MAP_ROWS - 1, int(row_idx)))
    c = max(0, min(MAP_COLS - 1, int(col_idx)))
    return f"{COL_LETTERS[c]}{r + 1}"


def block_to_meters(block: str) -> tuple[float, float]:
    """格心米制坐标（地图系）。"""
    row_idx, col_idx = block_to_index(block)
    return (10.5 - row_idx) * BLOCK_M, (13.5 - col_idx) * BLOCK_M


def meters_to_block(x: float, y: float) -> str:
    """米制坐标 → 所在格名（含边界夹紧）。"""
    row_idx = int(round((10.5 - x / BLOCK_M)))
    col_idx = int(round((13.5 - y / BLOCK_M)))
    return index_to_block(row_idx, col_idx)


def block_distance_cells(a: str, b: str) -> tuple[int, int]:
    """两格的 (Δ行, Δ列)。"""
    ra, ca = block_to_index(a)
    rb, cb = block_to_index(b)
    return abs(ra - rb), abs(ca - cb)
