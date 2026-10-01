"""官方 path_executor 的 /nav/waypoint 命令编码器（纯算法，无 ROS）。

语法严格对应车端 path_executor（见 reference/onboard 与旧 ag_car.route_builder，
本模块在新链路中独立实现，不再依赖 ag_car/ag_core）：

    "reset:N10,y"            里程计锚定：格号 + 摆放朝向（每局先发一次）
    "G8 A10 D2* E2 ,y"       路线：空格分隔格号；`*`=停靠射击；末尾 ,x/,y=终点朝向

非法 token 车端会整条丢弃，所以所有输入在此先校验，宁错不发。
"""

from __future__ import annotations

from dataclasses import dataclass

COL_LETTERS = "ABCDEFGHIJKLMN"
MAP_COLS = 14
MAP_ROWS = 11


class CommandError(ValueError):
    """格号/语法不合法。"""


@dataclass(frozen=True)
class RoutePlan:
    cells: tuple[str, ...]
    shoot_at: frozenset[str] = frozenset()
    final_heading: str | None = None   # 'x' / 'y' / None


def parse_cell(cell: str) -> str:
    """校验并返回规范格号 'D11'；非法抛 CommandError。"""
    s = (cell or "").strip().upper().rstrip("*")
    if len(s) < 2 or s[0] not in COL_LETTERS or not s[1:].isdigit():
        raise CommandError(f"非法格号 {cell!r}：应为 字母A~N + 数字1~11")
    row = int(s[1:]) - 1
    col = COL_LETTERS.index(s[0])
    if not (0 <= row < MAP_ROWS):
        raise CommandError(f"格号 {cell!r} 行号超出 1~{MAP_ROWS}")
    if not (0 <= col < MAP_COLS):
        raise CommandError(f"格号 {cell!r} 列号超出 A~{COL_LETTERS[-1]}")
    return s


def reset_command(start_cell: str, heading: str) -> str:
    cell = parse_cell(start_cell)
    h = (heading or "").strip().lower()
    if h not in ("x", "y"):
        raise CommandError(f"朝向只能是 'x'/'y'，收到 {heading!r}")
    return f"reset:{cell},{h}"


def route_command(plan: RoutePlan) -> str:
    if plan.final_heading is not None and plan.final_heading not in ("x", "y"):
        raise CommandError(f"终点朝向只能是 'x'/'y'，收到 {plan.final_heading!r}")

    ordered: list[str] = []
    present: set[str] = set()
    for raw in plan.cells:
        name = parse_cell(raw)
        if ordered and ordered[-1] == name:   # 连续去重
            continue
        ordered.append(name)
        present.add(name)

    if not ordered:
        raise CommandError("路线为空")

    shoot = {s for s in (parse_cell(x) for x in plan.shoot_at) if s in present}

    tokens = [f"{c}*" if c in shoot else c for c in ordered]
    if plan.final_heading:
        tokens.append(f",{plan.final_heading}")
    return " ".join(tokens).replace(" ,", ",")
