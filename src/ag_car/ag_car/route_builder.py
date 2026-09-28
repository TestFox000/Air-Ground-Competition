"""任务序列 -> 车端 `/nav/waypoint` 路线字符串（纯算法，无 ROS）。

为什么需要这一层：
    渡众 SmartMaster 车端**已经自带**任务执行器
    （`~/dzrobot/src/car_localization/car_localization/path_executor.py`），
    它订阅 `/nav/waypoint`（std_msgs/String），自己完成：
        格号 -> 米制坐标 -> 纯跟踪控制 -> dzactuator(cmd_vel/pursuitAngle)
        停靠点等待 `/shoot_count` 打满 -> 继续
        逐点吸附定位、转弯、终点朝向
    所以车端**不需要**我们再做底盘驱动、瞄准或开火——只需要把
    「格号序列 + 哪个格要停靠打击」拼成它认的字符串。

路线语法（来自 path_executor 源码，逐条核对过）：
    "G8 A10 D2* E2,y"
      - 空格分隔格号，格号 = 字母 A~N + 数字 1~11
      - `*` 后缀 = 在该格**停靠并射击**，射满次数（默认 5 发）后继续
      - 末尾 `,x` / `,y` = 强制终点朝向（x = 面向 X+，y = 面向 Y+）
    "reset:N10,y"   —— 单独一条命令，把里程计锚定到指定格号+朝向
      ⚠️ 车是**被人搬到起点**的，里程计不知道自己在哪，所以每局
         必须先发 reset，否则整条路线会整体偏移。

坐标约定（车端 block_to_meters）：
    x = (10.5 - row_idx) * 0.35      row_idx = 数字 - 1
    y = (13.5 - col_idx) * 0.35      col_idx = 字母序号(A=0)
    原点在 N11 格右下角；x 沿数字行方向，y 沿字母列方向。
⚠️ 与 ag_core.FieldGrid 的 x/y 是**对调**的，转换只走 cell_to_car_xy()。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

__all__ = [
    "CarGrid",
    "RouteError",
    "RouteSpec",
    "build_route",
    "cell_to_car_xy",
    "reset_command",
]

#: 车端 path_executor 的写死常量，改这里必须同步改车端
COL_LETTERS = "ABCDEFGHIJKLMN"
MAP_COLS = 14
MAP_ROWS = 11
BLOCK_SIZE = 0.35


class RouteError(ValueError):
    """格号/语法不合法。车端遇到非法 token 会**整条丢弃**，所以宁可提前报错。"""


@dataclass(frozen=True)
class CarGrid:
    """车端栅格常量（与 path_executor 一致，不要各写一份）。"""

    cols: int = MAP_COLS
    rows: int = MAP_ROWS
    cell_size: float = BLOCK_SIZE

    def parse(self, cell: str) -> Tuple[int, int]:
        """格号 -> (row_idx, col_idx)，0 基。非法格号抛 RouteError。"""
        s = (cell or "").strip().upper().rstrip("*")
        if len(s) < 2 or s[0] not in COL_LETTERS or not s[1:].isdigit():
            raise RouteError(f"非法格号 {cell!r}：应为 字母A~N + 数字1~11，例如 'D2'")
        col = COL_LETTERS.index(s[0])
        row = int(s[1:]) - 1
        if not (0 <= row < self.rows):
            raise RouteError(
                f"格号 {cell!r} 的行号超出范围（本场地 1~{self.rows}）"
                f"：注意场地是 {self.cols} 列 A~N x {self.rows} 行 1~{self.rows}，"
                "不是 11 列 x 14 行"
            )
        if not (0 <= col < self.cols):
            raise RouteError(f"格号 {cell!r} 的列号超出范围（本场地 A~{COL_LETTERS[self.cols - 1]}）")
        return row, col

    def to_car_xy(self, cell: str) -> Tuple[float, float]:
        """格号 -> 车端米制坐标 (x, y)。"""
        row, col = self.parse(cell)
        x = (self.rows - 0.5 - row) * self.cell_size
        y = (self.cols - 0.5 - col) * self.cell_size
        return x, y

    def name(self, row: int, col: int) -> str:
        if not (0 <= row < self.rows and 0 <= col < self.cols):
            raise RouteError(f"格号越界: row={row} col={col}")
        return f"{COL_LETTERS[col]}{row + 1}"


def cell_to_car_xy(cell: str, grid: Optional[CarGrid] = None) -> Tuple[float, float]:
    """格号 -> 车端米制坐标（x 沿数字行方向、y 沿字母列方向）。"""
    return (grid or CarGrid()).to_car_xy(cell)


@dataclass
class RouteSpec:
    """一条路线命令的解析结果，便于单测与日志。"""

    cells: List[str] = field(default_factory=list)
    shoot_at: List[str] = field(default_factory=list)   # 需要停靠打击的格号
    final_heading: Optional[str] = None                 # 'x' / 'y' / None

    def tokens(self) -> List[str]:
        out: List[str] = []
        for c in self.cells:
            out.append(f"{c}*" if c in self.shoot_at else c)
        if self.final_heading:
            out.append(f",{self.final_heading}")
            # 车端用 rsplit(',', 1) 取朝向，因此 ',' 必须**紧贴**最后一个格号
        return out

    def command(self) -> str:
        toks = self.tokens()
        if not toks:
            raise RouteError("空路线")
        # 朝向要拼成 "...E2,y"，不能是 "... E2 ,y"
        s = " ".join(toks)
        return s.replace(" ,", ",")


def build_route(
    cells: Sequence[str],
    shoot_at: Iterable[str] = (),
    final_heading: Optional[str] = None,
    dedupe_consecutive: bool = True,
    grid: Optional[CarGrid] = None,
) -> str:
    """把格号序列拼成车端路线命令。

    cells            任务顺序（格号），如 ["G8", "A10", "D2", "N11"]
    shoot_at         其中需要停靠射击的格号（会加 `*`）
    final_heading    'x' / 'y'，强制终点朝向；None = 由车端按末段方向自行决定
    dedupe_consecutive 连续重复的同一格只保留一次（车端也会去重，这里提前做掉，
                     免得日志里出现 'D2* D2' 这种容易误读的命令）
    """
    g = grid or CarGrid()
    shoot_flags = {str(s).strip().upper() for s in shoot_at}

    if final_heading is not None and final_heading not in ("x", "y"):
        raise RouteError(f"终点朝向只能是 'x' 或 'y'，收到 {final_heading!r}")

    spec = RouteSpec(final_heading=final_heading)
    seen: set = set()
    for raw in cells:
        name = str(raw or "").strip().upper()
        if not name:
            continue
        g.parse(name)  # 合法性校验：越界/写错会在这里直接抛
        name = name.rstrip("*")
        if dedupe_consecutive and spec.cells and spec.cells[-1] == name:
            continue
        spec.cells.append(name)
        seen.add(name)

    if not spec.cells:
        raise RouteError("路线为空（cells 里没有有效格号）")

    # 射击标记只对路线里真实存在的格生效，避免把 'Z9*' 这类误报塞进去
    spec.shoot_at = sorted(shoot_flags & seen, key=spec.cells.index)
    return spec.command()


def reset_command(cell: str, heading: str, grid: Optional[CarGrid] = None) -> str:
    """生成里程计锚定命令：车被搬到 `cell` 并朝 `heading` 摆放。

    每局开始、以及任何一次人工挪车之后都必须发。
    """
    g = grid or CarGrid()
    g.parse(cell)
    h = (heading or "").strip().lower()
    if h not in ("x", "y"):
        raise RouteError(f"朝向只能是 'x' 或 'y'，收到 {heading!r}")
    return f"reset:{cell.strip().upper()},{h}"


def route_summary(cells: Sequence[str], shoot_at: Iterable[str] = (),
                  final_heading: Optional[str] = None) -> Dict[str, object]:
    """给日志/自检用的一览（不抛异常，非法格号记进 invalid）。"""
    g = CarGrid()
    bad: List[str] = []
    ok: List[str] = []
    for c in cells:
        try:
            g.parse(c)
            ok.append(str(c).strip().upper().rstrip("*"))   # 抹掉 '*'，只留格号
        except RouteError:
            bad.append(str(c))
    out: Dict[str, object] = {
        "cells": ok,
        "invalid": bad,
        "shoot_at": [s for s in shoot_at if s in ok],
        "length": len(ok),
        "final_heading": final_heading,
    }
    if ok:
        out["route"] = build_route(ok, shoot_at, final_heading)
        xs = [g.to_car_xy(c) for c in ok]
        out["bbox_m"] = [
            round(min(p[0] for p in xs), 3), round(min(p[1] for p in xs), 3),
            round(max(p[0] for p in xs), 3), round(max(p[1] for p in xs), 3),
        ]
    return out
