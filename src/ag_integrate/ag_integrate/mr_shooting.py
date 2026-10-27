# mr_shooting —— 由 mission_router.py 拆出（射击站选择：环带停站/LOS遮挡/移动靶候选）。与原文件逐行一致（AST 校验过）。
from __future__ import annotations

import heapq
import warnings
from itertools import count, product
from math import gcd

from .field_frame import CellRef, ROWS, COLS, LETTERS
from .fire_guard import blue_targets_hit, decide_fire
from .world_model import WorldSnapshot
from .mr_nav import _ORTHO_STEPS
from .mr_params import _DIRS

_FACE_OPP = {"S": 6, "N": 2, "E": 4, "W": 0}   # 蓝面朝向 → 车需正对的方向（_DIRS 索引）
def _face_heading(frm: CellRef, to: CellRef) -> int:
    """frm→to 的正交朝向索引（射击停靠后车头应正对的朝向，_DIRS 索引）。"""
    dc = (to.col > frm.col) - (to.col < frm.col)
    dr = (to.row > frm.row) - (to.row < frm.row)
    return _DIRS.index((dc, dr))
_SHOOT_COMBO_CAP = 64  # 蓝靶射击位组合枚举上限，超过则退化为逐靶就近
def _shoot_candidates(
    target: CellRef,
    blocked: frozenset[CellRef],
    target_cells: dict[str, CellRef],
) -> list[CellRef]:
    """固定靶的候选停靠射击格：正交相邻格（车端正对靶心开火最准），
    排除障碍/桥洞墙/其他靶格。"""
    out = []
    for dc, dr in _ORTHO_STEPS:
        c = CellRef(target.col + dc, target.row + dr)
        if not (0 <= c.col < COLS and 0 <= c.row < ROWS):
            continue
        if c in blocked or c.name in target_cells:
            continue
        out.append(c)
    return sorted(out, key=lambda c: c.name)
_STANDOFF_OFFSETS = ((1, 2), (2, 1), (2, 2), (-1, 2), (-2, 1), (-2, 2),
                     (1, -2), (2, -1), (2, -2), (-1, -2), (-2, -1), (-2, -2))
# ★ 2026-10-26 吸收车端官方标定：射击停站与靶心距离须在 0.78~1.02 m
#   （route_check 第④层"射击点距离"红线；车端 fire_dz 25~26px 开火框按此
#   站距标定）。0.78/0.35=2.229、1.02/0.35=2.914 格 ⇒ 合法偏移只有
#   (±1,±2)/(±2,±1)（√5=2.236 格=0.783 m）与 (±2,±2)（√8=2.828 格=0.990 m）；
#   轴向 2 格=0.70 m 太近、3 格=1.05 m 太远，都在窗外。
#   旧口径（贴轨正交邻格=0.35 m）在 64 图头对头基准里造成 5 张停站失败、
#   官方六层校验"靶覆盖 0/2"❌，实锤为口径债而非搜索缺陷。


def _standoff_serves(c: CellRef, rail: CellRef) -> bool:
    """c 是否在 rail 的 0.78~1.02 m 标定窗内（窗内整数格偏移恰为 √5/√8 两档）。"""
    dc, dr = c.col - rail.col, c.row - rail.row
    return dc * dc + dr * dr in (5, 8)
def _los_clear(stop: CellRef, rail: CellRef,
               blocked: frozenset[CellRef]) -> bool:
    """停站→轨端连线是否不被障碍方格遮挡（激光是光路，挡住即白停）。

    采样线段两格心连线（步长 0.1 格），任一采样点落入障碍格的 0.35 m
    方内（|dc|<0.5 且 |dr|<0.5）即遮挡。车端 route_check 不查这条
    （2026-10-26 交叉验证在 11/64 图抓到 51 对擦边，最重 0.0 m 正穿），
    我方比它严一档：候选优先无遮挡，端点全遮挡才降级用带遮挡候选。
    """
    if not blocked:
        return True
    blocked_idx = {c.row * COLS + c.col for c in blocked}
    steps = 10
    for i in range(steps + 1):
        t = i / steps
        col = rail.col + (stop.col - rail.col) * t
        row = rail.row + (stop.row - rail.row) * t
        ci = int(col + 0.5)          # 四舍五入到最近格
        ri = int(row + 0.5)
        if ri * COLS + ci in blocked_idx:
            return False
    return True
def _shoot_candidates_moving(
    t: "TargetRef",
    blocked: frozenset[CellRef],
) -> list[CellRef]:
    """移动靶的候选停靠射击格：轨道两端在蓝面朝向一侧的标定站距环带。

    候选 = 各轨端 + _STANDOFF_OFFSETS 斜偏移，须落在蓝面一侧（红面侧不可
    见蓝面、不可能停）、界内、未被障碍/桥墙/轨道占用；对最近轨端的距离
    天然在 0.78~1.02 m 标定窗内（见 _STANDOFF_OFFSETS 注释）。
    轨道水平（两端同行）时 blue_face='S'(row+1)/'N'(row-1)；
    垂直（两端同列）时 'E'(col+1)/'W'(col-1)。
    """
    a, b = t.track
    vertical = a.row != b.row      # 行不同=垂直轨道（同列）；行相同=水平轨道
    out: set[CellRef] = set()
    for rail in (a, b):
        for dc, dr in _STANDOFF_OFFSETS:
            c = CellRef(rail.col + dc, rail.row + dr)
            if not (0 <= c.col < COLS and 0 <= c.row < ROWS):
                continue
            if vertical:            # 垂直轨道：蓝面侧判定
                if t.blue_face == "E" and c.col <= a.col:
                    continue
                if t.blue_face == "W" and c.col >= a.col:
                    continue
            else:                   # 水平轨道：蓝面侧判定
                if t.blue_face == "S" and c.row <= a.row:
                    continue
                if t.blue_face == "N" and c.row >= a.row:
                    continue
            if c in blocked or c == a or c == b:
                continue
            out.add(c)
    return sorted(out, key=lambda c: c.name)
