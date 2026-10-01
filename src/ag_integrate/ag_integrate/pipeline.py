"""端到端纯管线：世界态势 -> 出发门控 -> 车端命令（无 ROS、无 socket）。"""

from __future__ import annotations

from dataclasses import dataclass

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
    """情报是否足以发车。与“有没有敌方”无关：红面也要能去排雷/回终点。"""
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
) -> CarCommands:
    start = _parse_name(start_cell)
    goal = _parse_name(goal_cell)
    ordered = plan_route(snapshot, start, goal)
    return adapt(snapshot, ordered, start_cell, start_heading, final_heading)


def _parse_name(name: str) -> CellRef:
    from .field_frame import LETTERS
    s = name.strip().upper()
    return CellRef(LETTERS.index(s[0]), int(s[1:]) - 1)
