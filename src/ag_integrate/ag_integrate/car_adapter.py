"""车适配器：世界态势 + 已排好序的格序列 -> 官方 path_executor 两条命令。

本模块不决定“先去哪格、怎么绕障”（那是任务排序模块的事）；
它只负责把有序路线翻译成车端认的 reset + route，并用禁火保险过滤 '*'。
"""

from __future__ import annotations

from dataclasses import dataclass

from .car_commands import RoutePlan, reset_command, route_command
from .fire_guard import decide_fire
from .world_model import WorldSnapshot


@dataclass(frozen=True)
class CarCommands:
    reset: str
    route: str
    shoot_cells: tuple[str, ...]


def adapt(
    snapshot: WorldSnapshot,
    ordered_cells: tuple[str, ...],
    start_cell: str,
    start_heading: str,
    final_heading: str | None = None,
) -> CarCommands:
    # 先用禁火保险过滤候选停靠点：只有蓝敌格保留 '*'
    shoot: list[str] = []
    for cell in ordered_cells:
        decision = decide_fire(snapshot, cell)
        if decision.allowed:
            shoot.append(cell.strip().upper())

    plan = RoutePlan(tuple(ordered_cells), frozenset(shoot), final_heading)
    return CarCommands(
        reset_command(start_cell, start_heading),
        route_command(plan),
        tuple(shoot),
    )
