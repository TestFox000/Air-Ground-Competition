"""端到端纯管线：世界态势 -> 出发门控 -> 车端命令（无 ROS、无 socket）。"""

from __future__ import annotations

from dataclasses import dataclass, replace

from .car_adapter import CarCommands, adapt
from .field_frame import CellRef
from .mission_router import plan_route
from .world_model import WorldSnapshot


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
        return Gate(False, "not_enough_mines")
    if require_face and not snapshot.targets:
        return Gate(False, "face_unknown")
    return Gate(True)


def build_mission(
    snapshot: WorldSnapshot,
    start_cell: str,
    goal_cell: str,
    start_heading: str,
    final_heading: str | None,
    cross_gates: tuple[str, ...] = (),
    mid_boundary: int = 6,
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
    """给识别到的靶补上轨道先验（全场仅一个移动靶，识别只有瞬时哪面可见）。"""
    a, b = _parse_name(track[0]), _parse_name(track[1])
    targets = tuple(
        replace(t, track=(a, b), blue_face=blue_face)
        for t in snapshot.targets
    )
    return replace(snapshot, targets=targets)


def _parse_name(name: str) -> CellRef:
    from .field_frame import LETTERS
    s = name.strip().upper()
    return CellRef(LETTERS.index(s[0]), int(s[1:]) - 1)
