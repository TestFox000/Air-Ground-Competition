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
    mid_boundary: int = 5,
    *,
    target_track: tuple[str, str] | None = None,
    target_blue_face: str | None = None,
    time_params: "RouteTimeParams | None" = None,
    soft_cap_s: float | None = None,
) -> CarCommands:
    """态势 -> reset + route 两条命令。

    target_track/target_blue_face：移动靶（全场仅一个）的轨道两端格号与
    蓝面朝向。这是裁判系统每局固定的先验，无人机只识别瞬时哪面可见，
    因此由本节点参数注入，而不是从识别帧推导。

    时间预算门控（soft_cap_s 传入才启用，官方硬顶 180s、同分比用时）：
    规划出解后用 route_time.estimate_seconds 折算执行秒数（写入
    commands.est_seconds）；超软预算且有射击位时，砍掉射击停靠重规划
    （打击是唯一带 −5 负分风险的可砍任务），并置 commands.shed_shooting。
    纯排雷链超预算不自动砍雷——少排几颗雷是人的决策，不是规划器的。
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
    commands = adapt(snapshot, ordered, start_cell, start_heading, final_heading)

    if soft_cap_s is None or soft_cap_s <= 0:
        return commands

    from .route_time import RouteTimeParams, estimate_seconds
    params = time_params or RouteTimeParams()
    mine_names = {c.name for c in snapshot.mines}
    est = estimate_seconds(ordered, shoot_cells=commands.shoot_cells,
                           mine_cells=mine_names, params=params)
    commands = replace(commands, est_seconds=est.seconds)
    if est.seconds <= soft_cap_s or not commands.shoot_cells:
        return commands

    # 超软预算：砍射击位重规划（靶格物理仍在、仍不可踩，只是不停靠开火）
    shed_snapshot = replace(snapshot, targets=tuple(
        replace(t, is_blue=False) for t in snapshot.targets))
    try:
        ordered2 = plan_route(
            shed_snapshot, start, goal, gates, mid_boundary,
            start_heading=start_heading, final_heading=final_heading,
        )
    except ValueError:
        return commands   # 同一地图可达性不变，理论上不会到这里；防御性保底
    commands2 = adapt(shed_snapshot, ordered2, start_cell, start_heading, final_heading)
    est2 = estimate_seconds(ordered2, shoot_cells=commands2.shoot_cells,
                            mine_cells=mine_names, params=params)
    if est2.seconds < est.seconds:
        return replace(commands2, est_seconds=est2.seconds, shed_shooting=True)
    return commands


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
