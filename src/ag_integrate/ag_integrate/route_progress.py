# -*- coding: utf-8 -*-
"""路线执行进度推断（纯算法，无 ROS）——重规划"基于最新进度"的公共底座。

背景（2026-10-23）
------------------
重规划此前永远从**原始起点**重算整条路线，靠 path_executor 的"最近航段"
接续。这有两个实际故障面：
  1. 若新路线的排序把某个未完成的必经点（雷/桥门）排到了续接点后方，
     执行器跳段前进时会**静默漏掉**它（路线文本看似完整，任务实际缺项）；
  2. 时间预算按全程估算，中途重规划会高估剩余时间，可能误触发"砍射击位"。

本模块用 0x4002 车位姿 × 当前路线折线，推断"车在哪、已经过哪些格"，
供重规划把起点换到车的当前格、并剔除已扫过的雷。

口径（保守优先：宁可少剔除，不可误剔除）
--------------------------------------
  * 进度点 = 路线折线上离车最近的**显式航点**（压缩后的格）；
  * 已扫集 = 该航点**之前**的全部显式航点格（当前航点本身不算——车可能
    还差半格才到，把它算成"已过"会导致漏排雷）；
  * 对角段的隐式经过格不参与判定：雷/桥门在压缩时受 mission_keep 保护、
    必为显式航点，按显式航点判定即无遗漏；
  * 车位姿超出 car_pose_ttl（由调用方判定）或落格残差过大（world_to_cell
    抛 ValueError）时，调用方按"无进度"处理、退回原始起点——绝不拿
    过期/离谱的观测当进度（宁可保守，不可错删任务项）。
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Iterable

from .car_commands import parse_cell
from .field_frame import (CELL_SIZE, COLS, ROWS, X0, Y0, CellRef, LETTERS,
                          cell_to_world)
from .heading_calibration import nearest_axis


@dataclass(frozen=True)
class RouteProgress:
    """一次进度推断的结果。"""

    cell: CellRef                     # 车当前所在格（吸附 + 残差防线）
    heading: str                      # 车真实朝向最近轴（'x'/'y'）
    swept_cells: frozenset[str]       # 已过的显式航点格名（不含当前格）


def _cell_of(name: str) -> CellRef:
    return CellRef(LETTERS.index(name[0]), int(name[1:]) - 1)


def route_cells_of(route_text: str) -> list[str]:
    """车端路线命令串 -> 显式航点格名序列（去 '*'、去末尾朝向 token）。

    ★ 2026-10-24 修复：末尾朝向的标准写法是**连写**在最后一个格号上的
    （"… N5,y"，mission_router final_heading / dzbrain parse_mission 同一
    方言），旧实现只认空格分隔的独立 ",y" token ⇒ parse_cell("N5,y") 抛
    CommandError ⇒ 上层 except ValueError 捕获后**静默退回摆位起点**——
    "重规划基于最新进度"在所有带朝向后缀的路线上从未生效（起点回摆位、
    已扫雷不剔），正是 2026-10-23 想修的两个故障面原样复活。
    """
    out: list[str] = []
    for tok in (route_text or "").split():
        if tok.startswith(","):       # 独立 ",x" 朝向 token（兼容旧写法）
            break
        if "," in tok:                # 连写 "N5,y"：朝向后缀必在最后一个 token
            out.append(parse_cell(tok.split(",", 1)[0]))
            break
        out.append(parse_cell(tok))
    return out


def route_progress(route_text: str, x: float, y: float, yaw: float) -> RouteProgress:
    """由最新车位姿与当前路线推断执行进度。

    route_text：当前正在执行的路线命令串（car_commands 方言，可含 '*' 与
                末尾朝向 token）；
    x/y/yaw：   0x4002 观测的车世界坐标与朝向（调用方负责新鲜度）。

    车位姿出界时抛 ValueError（调用方按"无进度"降级）。落格用**包围盒
    界内 + 最近格吸附**，不用 world_to_cell 的残差防线——那是给"检测
    分类"用的（落错格=认错目标，宁可拒收）；车自身位置跨格边界（离两
    格心各 0.5）是正常行驶状态，拒收会让进度推断在航段中段全部失效。
    """
    cells = route_cells_of(route_text)
    if not cells:
        raise ValueError("route_text has no waypoints")
    col_f = (x - X0) / CELL_SIZE
    row_f = (ROWS - 1) - (y - Y0) / CELL_SIZE
    # 界内判定：允许半格跨界（车在边格边缘仍有效），出界即拒。
    if not (-0.5 <= col_f < COLS - 0.5 and -0.5 <= row_f < ROWS - 0.5):
        raise ValueError(
            f"车位姿出界: x={x:.3f} y={y:.3f} "
            f"(col_f={col_f:.2f}, row_f={row_f:.2f})")
    cell = CellRef(int(round(col_f)), int(round(row_f)))
    cell.validate()

    best_i = 0
    best_d2 = float("inf")
    for i, name in enumerate(cells):
        wx, wy = cell_to_world(_cell_of(name))
        d2 = (wx - x) ** 2 + (wy - y) ** 2
        if d2 < best_d2 - 1e-12:
            best_d2 = d2
            best_i = i

    return RouteProgress(
        cell=cell,
        heading=nearest_axis(yaw),
        swept_cells=frozenset(cells[:best_i]),
    )


def filter_swept_mines(snapshot, swept: Iterable[str]):
    """从态势快照中剔除已扫过的雷（dataclasses.replace，其余字段原样）。

    只剔雷：雷被车碾过即完成，语义无歧义。桥门/靶位不剔——桥门是否"已
    穿越"取决于路线走向而非物理经过，靶是否"已打击"执行侧才清楚，误剔
    的代价（漏任务）远大于多跑一格的代价。
    """
    swept = frozenset(swept)
    if not swept:
        return snapshot
    return replace(snapshot, mines=tuple(
        m for m in snapshot.mines if m.name not in swept))
