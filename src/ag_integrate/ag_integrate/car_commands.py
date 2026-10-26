"""官方 path_executor 的 /nav/waypoint 命令编码器（纯算法，无 ROS）。

语法严格对应车端 path_executor（见 reference/onboard 与旧 ag_car.route_builder，
本模块在新链路中独立实现，不再依赖 ag_car/ag_core）：

    "reset:N10,x"            里程计锚定：格号 + 摆放朝向（每局先发一次）
    "G8 A10 D2* E2,x"        路线：空格分隔格号；`*`=停靠射击；末尾 ,x/,y=终点朝向

非法 token 车端会整条丢弃，所以所有输入在此先校验，宁错不发。
⚠️ 朝向 token 在这里从本机坐标架翻译成车端坐标架（两架差 90°，见
_CAR_HEADING）：上面两行是**上线的字节**，配置里写的 start_heading/final_heading
是**本机 token**，两者不一定同字。
"""

from __future__ import annotations

from dataclasses import dataclass

COL_LETTERS = "ABCDEFGHIJKLMN"
MAP_COLS = 14
MAP_ROWS = 11

# ---- 朝向 token 翻译（本机架 -> 车端架）----
# ag_integrate 与车端（官方 path_executor / dznav）的 'x'/'y' **不是同一套坐标架**，
# 直接透传会让车的航向信念整体差 90°（表现为纯追踪绕目标画圆）：
#
#   本机（field_frame.cell_to_world，同 dzuav MapLocalizer::world）
#       world_x = 0.065 + col*0.35       -> +x 指向列号增大（A→N）
#       world_y = 0.295 + (10-row)*0.35  -> +y 指向行号减小（第11行→第1行）
#   车端（dznav contract.block_to_meters，逐字对齐官方 car_localization）
#       x = (10.5 - row_idx)*0.35        -> +x 指向行号减小（第11行→第1行）
#       y = (13.5 - col_idx)*0.35        -> +y 指向列号减小（N→A）
#
# 车端 +x ∥ 本机 +y、车端 +y ∥ 本机 −x，两架相差 −90°（θ_车 = θ_机 − π/2）。
# 按物理方向配对：本机 'x'(A→N) = 车端 '-y'；本机 'y'(向第1行) = 车端 'x'。
#
# 本模块就是"官方 path_executor 命令编码器"，说车端方言是它的职责，所以翻译
# 放在这一层：match_params.yaml、mission_router、heading_calibration 一律继续
# 用本机 token，配置里 x(东)/y(北) 的含义不变，操作手不需要改任何配置。
# 车端两套实现都接受 -x/-y（dznav RE_HEADING、官方 HEADING_VECTORS）。
#
# 推导不靠注释口说：tests/test_heading_frame.py 直接从两个包的"格号 -> 坐标"
# 函数重算这张表，任一侧改动坐标架都会当场红。
_CAR_HEADING = {"x": "-y", "y": "x"}


def to_car_heading(heading: str) -> str:
    """本机朝向 token -> 车端朝向 token（见 _CAR_HEADING 推导）。"""
    return _CAR_HEADING[heading]


class CommandError(ValueError):
    """格号/语法不合法。"""


@dataclass(frozen=True)
class RoutePlan:
    cells: tuple[str, ...]
    shoot_at: frozenset[str] = frozenset()
    final_heading: str | None = None   # 本机 token 'x'/'y'/None（编码时翻译）


def parse_cell(cell: str) -> str:
    """校验并返回规范格号 'D11'（前导零归一：'A01' -> 'A1'）；非法抛 CommandError。"""
    s = (cell or "").strip().upper().rstrip("*")
    if len(s) < 2 or s[0] not in COL_LETTERS or not s[1:].isdigit():
        raise CommandError(f"非法格号 {cell!r}：应为 字母A~N + 数字1~11")
    row = int(s[1:]) - 1
    col = COL_LETTERS.index(s[0])
    if not (0 <= row < MAP_ROWS):
        raise CommandError(f"格号 {cell!r} 行号超出 1~{MAP_ROWS}")
    if not (0 <= col < MAP_COLS):
        raise CommandError(f"格号 {cell!r} 列号超出 A~{COL_LETTERS[-1]}")
    return f"{s[0]}{row + 1}"


def reset_command(start_cell: str, heading: str) -> str:
    cell = parse_cell(start_cell)
    h = (heading or "").strip().lower()
    if h not in ("x", "y"):
        raise CommandError(f"朝向只能是 'x'/'y'，收到 {heading!r}")
    return f"reset:{cell},{to_car_heading(h)}"


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
    # :PIVOT 声明（官方 route_check 第⑥层口径：车头转角 ≥135° 的顶点须声明，
    # 虚报=❌、漏报=⚠️）。按下标记、不按格名——同名格重访时直行通过的那次
    # 不能被别处的 180° 折返连坐成虚报。与 '*' 叠加形如 'F2*:PIVOT'，
    # 对应车端解析器先剥 :PIVOT 再剥 * 的顺序。
    for i in _pivot_indices(ordered):
        tokens[i] += ":PIVOT"
    if plan.final_heading:
        tokens.append(f",{to_car_heading(plan.final_heading)}")
    return " ".join(tokens).replace(" ,", ",")


def _pos(name: str) -> tuple[int, int]:
    return (COL_LETTERS.index(name[0]), int(name[1:]) - 1)


def _pivot_indices(cells: list[str]) -> list[int]:
    """相邻三格的车头转角 ≥135° 的中间格下标（含 180° 原路折返）。

    转角 = 相邻两段方向向量夹角，对角步按实际向量算，阈值与官方
    route_check.PIVOT_DECL_DEG=135.0 一致。零向量（重复格，防御）跳过。
    """
    import math
    out: list[int] = []
    for i in range(1, len(cells) - 1):
        c0, r0 = _pos(cells[i - 1])
        c1, r1 = _pos(cells[i])
        c2, r2 = _pos(cells[i + 1])
        v1 = (c1 - c0, r1 - r0)
        v2 = (c2 - c1, r2 - r1)
        n1 = math.hypot(*v1)
        n2 = math.hypot(*v2)
        if not n1 or not n2:
            continue
        cos = (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)
        if math.degrees(math.acos(max(-1.0, min(1.0, cos)))) >= 135.0:
            out.append(i)
    return out
