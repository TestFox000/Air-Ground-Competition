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


#: 落格残差上限（以格为单位）。0.5 = 正好压在两格边界上。
#: 原默认值 0.5 使下面这条检查成为**死代码**：col = round(col_f) 天然保证
#: abs(col_f - col) <= 0.5，永远不可能 > 0.5，于是"离格心过远就拒绝"从来没
#: 生效过 —— 距格心 0.49 格（17cm，比半个格还远）的检测照样被吸附进最近格。
#: 收到 0.35（≈12cm）：超过这个残差，"落在哪一格"已接近抛硬币，与其把
#: 一个可能错格的雷/障碍/红靶喂给规划器，不如拒收让上游的粘性否决与多帧
#: 确认接手。⚠️ 现场务必用 tools/monitor_link.py 观察拒收率再决定是否回调。
MAX_CELL_RESIDUAL = 0.35


def world_to_cell(x: float, y: float,
                  max_residual: float = MAX_CELL_RESIDUAL) -> CellRef:
    """把世界坐标吸附到最近格；离格心过远或出界则抛 ValueError。"""
    col_f = (x - X0) / CELL_SIZE
    row_f = (ROWS - 1) - (y - Y0) / CELL_SIZE
    col = round(col_f)
    row = round(row_f)
    cell = CellRef(col, row)
    cell.validate()
    d_col, d_row = abs(col_f - col), abs(row_f - row)
    if d_col > max_residual or d_row > max_residual:
        raise ValueError(
            f"point too far from a cell center: x={x:.3f} y={y:.3f} "
            f"-> {cell.name} residual=({d_col:.3f},{d_row:.3f})cell "
            f"({max(d_col, d_row) * CELL_SIZE * 100:.1f}cm) "
            f"> {max_residual:.2f}cell")
    return cell
