"""任务路由器：世界态势 -> 绕障的有序格序列。

输入 world_model.WorldSnapshot，输出有序格号（tuple[str]），交给 car_adapter。

做法（保持简单、可测）：
    - 障碍格为 blocked；其余 free；
    - 必经点 = 雷 + 可通行桥洞格，按“离当前位置最近”贪心选序；
    - 相邻两点间用 4 邻域 BFS 求最短 free 路径；
    - 最后到终点；某必经点不可达则跳过，终点不可达则报错。

已知简化（现场核实后再加强，不臆造拓扑）：
    中场挡板未默认建模——只把识别到的障碍当作 blocked；可通行格作为必经点
    自然纳入。现场证实“只能从可通行格穿越半场”后，向 plan_route 传入
    cross_gates（允许穿越的格）即可在 BFS 中强制该约束；默认不启用。
"""

from __future__ import annotations

from collections import deque

from .field_frame import CellRef, ROWS, COLS
from .world_model import WorldSnapshot


def _crosses_mid(row_a: int, row_b: int, boundary: int) -> bool:
    """一条竖向边是否跨越中场分界（boundary 指上半区行数，如 5 = row4↔row5 之间）。"""
    return (row_a < boundary <= row_b) or (row_b < boundary <= row_a)


def _bfs(
    start: CellRef,
    goal: CellRef,
    blocked: frozenset[CellRef],
    cross_gates: frozenset[CellRef] | None = None,
    mid_boundary: int = ROWS // 2,
) -> list[CellRef]:
    if start == goal:
        return [start]
    prev: dict[CellRef, CellRef] = {start: start}
    q: deque[CellRef] = deque([start])
    while q:
        cur = q.popleft()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nxt = CellRef(cur.col + dc, cur.row + dr)
            if not (0 <= nxt.col < COLS and 0 <= nxt.row < ROWS):
                continue
            if nxt in blocked or nxt in prev:
                continue
            # 中场穿越约束：跨越中线的边必须贴着某个可通行 gate 格。
            if (
                cross_gates is not None
                and dr != 0
                and _crosses_mid(cur.row, nxt.row, mid_boundary)
                and cur not in cross_gates
                and nxt not in cross_gates
            ):
                continue
            prev[nxt] = cur
            if nxt == goal:
                path: list[CellRef] = []
                node = nxt
                while node != start:
                    path.append(node)
                    node = prev[node]
                path.append(start)
                return path[::-1]
            q.append(nxt)
    return []


def _flatten(segments: list[list[CellRef]]) -> tuple[str, ...]:
    names: list[str] = []
    for seg in segments:
        for cell in seg:
            name = cell.name
            if not names or names[-1] != name:
                names.append(name)
    return tuple(names)


def plan_route(
    snapshot: WorldSnapshot,
    start: CellRef,
    goal: CellRef,
    cross_gates: frozenset[CellRef] | None = None,
    mid_boundary: int = ROWS // 2,
) -> tuple[str, ...]:
    blocked = frozenset(snapshot.obstacles)
    # 必经点 = 雷 + 可通行桥洞格 + 蓝色敌靶格（要把车开过去才能开火；红靶不去）
    required = [
        *snapshot.mines,
        *snapshot.passable,
        *[t.cell for t in snapshot.targets if t.is_blue],
    ]

    ordered: list[CellRef] = []
    current = start
    remaining = set(required)
    segments: list[list[CellRef]] = []

    while remaining:
        nxt = min(remaining, key=lambda c: (abs(c.row - current.row) + abs(c.col - current.col)))
        seg = _bfs(current, nxt, blocked, cross_gates, mid_boundary)
        if not seg:
            remaining.discard(nxt)   # 不可达，跳过
            continue
        segments.append(seg)
        ordered.append(nxt)
        current = nxt
        remaining.discard(nxt)

    tail = _bfs(current, goal, blocked, cross_gates, mid_boundary)
    if not tail:
        raise ValueError("goal cell unreachable from last waypoint")
    segments.append(tail)

    if not segments:
        return (start.name,)
    return _flatten(segments)
