"""dzuav 世界坐标（米）与场地格号的双向换算。

对应 dzuav MapLocalizer.world()（reference/onboard/dzuav/src/MapLocalizer.cpp）：

    world_x = 0.065 + col * 0.35          col: 0=A .. 13=N
    world_y = 0.295 + (11 - row_disp) * 0.35
           = 0.295 + (10 - row) * 0.35    row: 0=第1行 .. 10=第11行

原点/偏移是机上定义，与旧 ag_core.grid_map 的栅格原点不同，因此本模块独立实现，
新链路不再依赖 ag_core。
"""

from __future__ import annotations

from dataclasses import dataclass

CELL_SIZE = 0.35
COLS = 14   # A..N
ROWS = 11   # 1..11
X0 = 0.065
Y0 = 0.295

LETTERS = "ABCDEFGHIJKLMN"


@dataclass(frozen=True)
class CellRef:
    col: int   # 0 基
    row: int   # 0 基

    @property
    def name(self) -> str:
        return f"{LETTERS[self.col]}{self.row + 1}"

    def validate(self) -> None:
        if not (0 <= self.col < COLS and 0 <= self.row < ROWS):
            raise ValueError(f"cell out of field: col={self.col} row={self.row}")


def cell_to_world(cell: CellRef) -> tuple[float, float]:
    cell.validate()
    return (X0 + cell.col * CELL_SIZE, Y0 + (ROWS - 1 - cell.row) * CELL_SIZE)


def world_to_cell(x: float, y: float, max_residual: float = 0.5) -> CellRef:
    """把世界坐标吸附到最近格；离格心过远或出界则抛 ValueError。"""
    col_f = (x - X0) / CELL_SIZE
    row_f = (ROWS - 1) - (y - Y0) / CELL_SIZE
    col = round(col_f)
    row = round(row_f)
    cell = CellRef(col, row)
    cell.validate()
    if abs(col_f - col) > max_residual or abs(row_f - row) > max_residual:
        raise ValueError(f"point too far from a cell center: x={x} y={y}")
    return cell
