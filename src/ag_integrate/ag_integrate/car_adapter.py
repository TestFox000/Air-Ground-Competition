"""车适配器：世界态势 + 已排好序的格序列 -> 官方 path_executor 两条命令。

本模块不决定“先去哪格、怎么绕障”（那是任务排序模块的事）；
它只负责把有序路线翻译成车端认的 reset + route，并用禁火保险过滤 '*'。
"""

from __future__ import annotations

from dataclasses import dataclass

from .car_commands import RoutePlan, reset_command, route_command
from .fire_guard import blue_targets_hit, decide_fire
from .world_model import TargetRef, WorldSnapshot


@dataclass(frozen=True)
class CarCommands:
    reset: str
    route: str
    shoot_cells: tuple[str, ...]
    # 时间预算（route_time 门控，默认未启用时为 -1/False，不影响旧调用方）：
    est_seconds: float = -1.0      # 路线预估执行秒数；-1 = 未估算
    shed_shooting: bool = False    # True = 超软预算已砍射击位重规划


def _target_key(t: TargetRef) -> tuple:
    """靶身份键：移动靶以轨道为身份（全场仅一个，靶在轨上换格不换身份）；
    固定靶以落点格为身份。served 去重必须按身份，按格名会让移动靶
    换格后被重复记/漏记。"""
    if t.track is not None:
        return ("track", t.track)
    return ("fixed", t.cell.name)


def _target_key(t: TargetRef) -> tuple:
    """靶身份键：移动靶以轨道为身份（全场仅一个，靶在轨上换格不换身份）；
    固定靶以落点格为身份。served 去重必须按身份，按格名会让移动靶
    换格后被重复记/漏记。"""
    if t.track is not None:
        return ("track", t.track)
    return ("fixed", t.cell.name)


def adapt(
    snapshot: WorldSnapshot,
    ordered_cells: tuple[str, ...],
    start_cell: str,
    start_heading: str,
    final_heading: str | None = None,
) -> CarCommands:
    # 禁火保险过滤 + 每靶一次：只有蓝敌格保留 '*'，且每个蓝靶
    # 只在其首个合法开火格开火一次（顺路再次经过不重复开火）。
    # 命中判定复用 decide_fire 的同一套几何（blue_targets_hit）：
    # 移动靶按轨道端点+蓝面朝向判定，不按瞬时落点的格名邻接。
    shoot: list[str] = []
    shoot_set: set[str] = set()
    served: set[tuple] = set()
    for cell in ordered_cells:
        name = cell.strip().upper()
        if decide_fire(snapshot, name).denied:
            continue
        fresh = [t for t in blue_targets_hit(snapshot, name)
                 if _target_key(t) not in served]
        if not fresh:
            continue
        served.update(_target_key(t) for t in fresh)
        if name not in shoot_set:
            shoot_set.add(name)
            shoot.append(name)

    plan = RoutePlan(tuple(ordered_cells), frozenset(shoot), final_heading)
    return CarCommands(
        reset_command(start_cell, start_heading),
        route_command(plan),
        tuple(shoot),
    )
