"""禁火保险：停靠格正交相邻蓝色敌方靶时才允许开火。

官方车端 det_node 不分红蓝、path_executor 见到 '*' 就开火；
误击红色友方每计数 −5（最多 −15）。本模块是开火前的最后一道独立闸门。

车不踩靶格（靶格是物理靶体，路线生成时已设为不可通行），
而是在靶的正交邻格停靠、车头正对靶心开火。因此本模块的判定是：
停靠格的正交四邻里必须有蓝靶；若同时邻到红靶（夹缝场景）则拒绝，
宁可不打也不能冒误伤友方的风险。
"""

from __future__ import annotations

from dataclasses import dataclass

from .field_frame import COLS, LETTERS, ROWS, CellRef
from .world_model import TargetRef, WorldSnapshot


@dataclass(frozen=True)
class FireDecision:
    allowed: bool
    reason: str

    @property
    def denied(self) -> bool:
        return not self.allowed


def neighbor_names(name: str) -> tuple[str, ...]:
    """格号正交四邻的格号（越界忽略）。"""
    col = LETTERS.index(name[0].upper())
    row = int(name[1:]) - 1
    out = []
    for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        c, r = col + dc, row + dr
        if 0 <= c < COLS and 0 <= r < ROWS:
            out.append(f"{LETTERS[c]}{r + 1}")
    return tuple(out)


def _track_fire_cells(t: TargetRef) -> tuple[CellRef, ...]:
    """移动靶的合法开火格：轨道两端在蓝面朝向一侧的正交邻格（越界剔除）。

    轨道水平（两端同行）时 blue_face='S'(row+1)/'N'(row-1)；
    垂直（两端同列）时 'E'(col+1)/'W'(col-1)。
    """
    a, b = t.track
    if a.row == b.row:       # 水平轨道
        dr = 1 if t.blue_face == "S" else -1
        cells = (CellRef(a.col, a.row + dr), CellRef(b.col, b.row + dr))
    else:                    # 垂直轨道
        dc = 1 if t.blue_face == "E" else -1
        cells = (CellRef(a.col + dc, a.row), CellRef(b.col + dc, b.row))
    return tuple(c for c in cells if 0 <= c.col < COLS and 0 <= c.row < ROWS)


def blue_targets_hit(snapshot: WorldSnapshot, shoot_cell: str) -> tuple[TargetRef, ...]:
    """停靠在 shoot_cell 开火会打中的蓝靶集合（与 decide_fire 同一套几何）。

    固定靶：停靠格正交邻靶格；移动靶：停靠格是轨道端点在蓝面朝向侧的
    正交邻格——与靶当前瞬时在轨位置无关（轨道几何判定，不是格名邻接）。
    """
    name = shoot_cell.strip().upper()
    around = set(neighbor_names(name))
    out = []
    for t in snapshot.targets:
        if not t.is_blue:
            continue
        if t.track is not None:
            if any(name == c.name for c in _track_fire_cells(t)):
                out.append(t)
        elif t.cell.name in around:
            out.append(t)
    return tuple(out)


def decide_fire(snapshot: WorldSnapshot, shoot_cell: str) -> FireDecision:
    """停靠在 shoot_cell 时是否允许开火。

    固定靶：正交邻有蓝靶且不邻红靶。
    移动靶（全场仅一个）：红面可见一律禁火；蓝面可见时，
    停靠格必须是轨道端点在蓝面朝向一侧的正交邻格（正对端点）。
    """
    name = shoot_cell.strip().upper()
    around = set(neighbor_names(name))
    has_blue = bool(blue_targets_hit(snapshot, name))
    has_red = False
    for t in snapshot.targets:
        if t.is_blue:
            continue
        if t.track is not None:
            has_red = True        # 红面可见：友方，禁火（不论停在轨道何处）
        elif t.cell.name in around:
            has_red = True
    if has_blue and has_red:
        return FireDecision(False, "mixed_targets_risk")
    if has_blue:
        return FireDecision(True, "enemy_blue_adjacent")
    if has_red:
        return FireDecision(False, "friendly_red_adjacent")
    return FireDecision(False, "no_target_adjacent")
