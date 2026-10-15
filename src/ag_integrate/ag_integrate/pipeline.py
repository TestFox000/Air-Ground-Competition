"""端到端纯管线：世界态势 -> 出发门控 -> 车端命令（无 ROS、无 socket）。"""

from __future__ import annotations

from dataclasses import dataclass, replace

from .car_adapter import CarCommands, adapt
from .field_frame import LETTERS, CellRef
from .mission_router import _MID_BOUNDARY, plan_route
from .world_model import WorldSnapshot

# 门控 reason 是黑匣子事件流契约（route.blocked 的 reason 字段），字面量
# 必须保持稳定，语义对应 ag_core.blackbox.REASON.MIN_MINES / FACE_UNKNOWN。
# REASON.MIN_MINES 现值 "min_mines" 与本契约不一致：按"事件流字面量不变"
# 原则以本包现值为准，ag_core 常量由下一波 ag_core 修复对齐（本包不动它）。
REASON_NOT_ENOUGH_MINES = "not_enough_mines"
REASON_FACE_UNKNOWN = "face_unknown"


@dataclass(frozen=True)
class Gate:
    ok: bool
    reason: str = ""


def evaluate_gate(
    snapshot: WorldSnapshot,
    min_mines: int,
    require_face: bool,
) -> Gate:
    """情报是否足以发车。与"有没有敌方"无关：红面也要能去排雷/回终点。"""
    if len(snapshot.mines) < min_mines:
        return Gate(False, REASON_NOT_ENOUGH_MINES)
    if require_face and not snapshot.targets:
        return Gate(False, REASON_FACE_UNKNOWN)
    return Gate(True)


def build_mission(
    snapshot: WorldSnapshot,
    start_cell: str,
    goal_cell: str,
    start_heading: str,
    final_heading: str | None,
    cross_gates: tuple[str, ...] = (),
    mid_boundary: int = _MID_BOUNDARY,
    *,
    target_track: tuple[str, str] | None = None,
    target_blue_face: str | None = None,
) -> CarCommands:
    """态势 -> reset + route 两条命令。

    target_track/target_blue_face：移动靶（全场仅一个）的轨道两端格号与
    蓝面朝向。这是裁判系统每局固定的先验，无人机只识别瞬时哪面可见，
    因此由本节点参数注入，而不是从识别帧推导。
    """
    start = _parse_name(start_cell)
    goal = _parse_name(goal_cell)
    gates = frozenset(_parse_name(name) for name in cross_gates) or None
    if target_track is not None:
        snapshot = _inject_track(snapshot, target_track, target_blue_face)
    ordered = plan_route(
        snapshot, start, goal, gates, mid_boundary,
        start_heading=start_heading, final_heading=final_heading,
    )
    return adapt(snapshot, ordered, start_cell, start_heading, final_heading)


def _inject_track(
    snapshot: WorldSnapshot,
    track: tuple[str, str],
    blue_face: str | None,
) -> WorldSnapshot:
    """给识别到的移动靶补轨道先验（前提：全场仅一个移动靶，轨道两端由
    裁判系统地图固定，无人机识别只有瞬时落点与哪面可见）。

    只对落在轨道上的靶注入（落点格在轨道线段上，即距线段 <0.5 格）；
    固定红/蓝靶不注入——否则固定红靶会被当成移动靶触发全局禁火，
    固定蓝靶的停靠射击位会被轨道几何算错。
    """
    a, b = _parse_name(track[0]), _parse_name(track[1])
    targets = tuple(
        replace(t, track=(a, b), blue_face=blue_face) if _on_track(t.cell, a, b) else t
        for t in snapshot.targets
    )
    return replace(snapshot, targets=targets)


def _on_track(cell: CellRef, a: CellRef, b: CellRef) -> bool:
    """落点格是否"在轨道上"：格心到轨道线段 ab 的距离 < 0.5 格。"""
    dx, dy = b.col - a.col, b.row - a.row
    if dx == 0 and dy == 0:
        return cell == a
    # 格心到线段 ab 的垂距（格单位），t 截到 [0,1] 得线段最近点
    t = ((cell.col - a.col) * dx + (cell.row - a.row) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    dc = cell.col - (a.col + t * dx)
    dr = cell.row - (a.row + t * dy)
    return dc * dc + dr * dr < 0.25


def _parse_name(name: str) -> CellRef:
    """格号 -> CellRef；非法（空串/字母越界/行号越界）抛 ValueError。"""
    s = name.strip().upper()
    if len(s) < 2 or s[0] not in LETTERS or not s[1:].isdigit():
        raise ValueError(f"非法格号 {name!r}：应为 字母A~N + 数字1~11")
    cell = CellRef(LETTERS.index(s[0]), int(s[1:]) - 1)
    cell.validate()
    return cell
