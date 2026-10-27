# mr_board —— 由 mission_router.py 拆出（场地模型：格点/边界/挡板/邻接表/转向侧表及其构建）。与原文件逐行一致（AST 校验过）。
from __future__ import annotations

import heapq
import warnings
from itertools import count, product
from math import gcd

from .field_frame import CellRef, ROWS, COLS, LETTERS
from .fire_guard import blue_targets_hit, decide_fire
from .world_model import WorldSnapshot
from .mr_params import _DIRS, _SQRT2

# 中场横向挡板：位于第 6/7 行的边界线上（row5↔row6 之间），
# 向上占第6行底部1/4、向下占第7行顶部1/4，总高半格。
# 第6行、第7行格子本身可通行，但跨边界的竖向移动被阻挡，桥洞缺口除外。
# 2026-10-15 终裁：挡板实体在 D7~N7 格带（第 6/7 行边界）——车上官方地图真源
# + 现场核实（I6→I7/K6→K7/L6→L7 全封死）双重确认，mid_boundary=6。
# 注意：旧"决定性证据"（goal=N6+heading=y 只在 =5 可行）建立在旧门模型上，
# 与现场物理冲突的归一问题见 docs/分区规划与感知约束-20261015.md 问题1。
_MID_BOUNDARY = 6  # 唯一默认值来源：row5(第6行) ↔ row6(第7行) 之间
# 横向挡板覆盖 D(3)~N(13) 列，A、B、C 列无挡板可自由穿越。
# 桥洞列由 passable（ARROW）动态确定，每局随机。
_BARRIER_COLS: frozenset[int] = frozenset(range(3, COLS))  # D..N
# ---- 静态几何表（模块级一次性构建）：整数编码 (格, 航向) 状态空间 ----
# 格索引 ci = row*COLS + col；状态索引 si = ci*8 + h。
# Dijkstra 内循环只碰整数与预构边表，不再逐邻居构造 CellRef/元组哈希
# （2026-10-14 沙箱审计：旧实现单次 ~60ms，其中大半是 CellRef.__init__/__hash__）。
_N_CELLS = ROWS * COLS
_CELLS: tuple[CellRef, ...] = tuple(
    CellRef(c, r) for r in range(ROWS) for c in range(COLS)
)
# 8×8 航向差（几个 45°）查表
_TURNS: tuple[tuple[int, ...], ...] = tuple(
    tuple(min(abs(a - b), 8 - abs(a - b)) for b in range(8)) for a in range(8)
)
def _build_edges() -> tuple[tuple[tuple[int, int, float, bool, int, int, int, int], ...], ...]:
    """_EDGES[ci] = ((n_ci, nh, step, diag, o1_ci, o2_ci, dr, n_row), ...)

    o1/o2 为对角移动防切角的两侧正交格（非对角时为 -1）；dr/n_row 供中场
    挡板跨界判定（mid_boundary 是运行期参数，不能烧进表里）。
    """
    edges = []
    for r in range(ROWS):
        for c in range(COLS):
            lst = []
            for nh, (dc, dr) in enumerate(_DIRS):
                nc, nr = c + dc, r + dr
                if not (0 <= nc < COLS and 0 <= nr < ROWS):
                    continue
                diag = dc != 0 and dr != 0
                o1 = r * COLS + nc if diag else -1   # (nc, r)
                o2 = nr * COLS + c if diag else -1   # (c, nr)
                lst.append((nr * COLS + nc, nh, _SQRT2 if diag else 1.0,
                            diag, o1, o2, dr, nr))
            edges.append(tuple(lst))
    return tuple(edges)
_EDGES = _build_edges()
def _build_inedges() -> tuple[tuple[tuple[int, int, float, bool, int, int, int, int], ...], ...]:
    """_INEDGES[ci] = ((p_ci, trav, step, diag, o1, o2, dr, p_row), ...)

    倒车镜像边表：前进边 (p,h_dep)→(ci,trav) 的倒车遍历是 (ci,trav,R)→(p,h_dep,R)
    ——同一条物理弧、扫掠同一片空间，因此边级过滤（防切角/桥门/隧道）与前进边
    完全同判（2026-10-25 倒车挡位原语）。trav = p→ci 的行驶方向索引。
    """
    ine: list[list[tuple[int, int, float, bool, int, int, int, int]]] = [
        [] for _ in range(_N_CELLS)
    ]
    for r in range(ROWS):
        for c in range(COLS):
            ci = r * COLS + c
            for nci, nh, step, diag, o1, o2, dr, _nrow in _EDGES[ci]:
                ine[nci].append((ci, nh, step, diag, o1, o2, dr, r))
    return tuple(tuple(x) for x in ine)
_INEDGES = _build_inedges()
def _build_turn_side_tables() -> tuple[tuple[tuple[int, ...], ...], tuple[tuple[int, ...], ...]]:
    """转弯内侧正交方向 _INNER 与内切对角方向 _IDIAG 的 8×8 查表（-1=共线）。

    与 turn_feasibility_v2._inside_dir/_clearance_inside 同一几何口径：
    左转（cross>0）取 v_in 逆时针 90°，右转取顺时针 90°；内切对角 =
    sign(v_in + v_out)。守卫与实体挡板探测共用（2026-10-25 综合规划）。
    """
    idx = {d: i for i, d in enumerate(_DIRS)}
    inner: list[list[int]] = [[-1] * 8 for _ in range(8)]
    idiag: list[list[int]] = [[-1] * 8 for _ in range(8)]
    for h, vh in enumerate(_DIRS):
        for nh, vo in enumerate(_DIRS):
            cross = vh[0] * vo[1] - vh[1] * vo[0]
            if cross > 0:
                inner[h][nh] = idx.get((-vh[1], vh[0]), -1)
            elif cross < 0:
                inner[h][nh] = idx.get((vh[1], -vh[0]), -1)
            sx, sy = vh[0] + vo[0], vh[1] + vo[1]
            if sx or sy:
                idiag[h][nh] = idx.get((1 if sx > 0 else -1, 1 if sy > 0 else -1), -1)
    return tuple(tuple(r) for r in inner), tuple(tuple(r) for r in idiag)
_INNER, _IDIAG = _build_turn_side_tables()
# _SIDES[h][nh] = 转弯需探测的方向元组（内侧正交+内切对角，共线=空）——
# 守卫热路径查表用，避免每次转弯现算 min(genexpr)（实测占增强模式 60%+ 耗时）。
_SIDES: tuple[tuple[tuple[int, ...], ...], ...] = tuple(
    tuple(
        tuple(d for d in (_INNER[h][nh], _IDIAG[h][nh]) if d >= 0)
        for nh in range(8)
    )
    for h in range(8)
)
# 实体挡板侵入深度：中场挡板总高半格（175mm），上下各伸进邻格 1/4 格。
# 格占用模型看不见这部分几何（K7 贴板死锁 / M7 转弯 1.2mm 余量的直接成因），
# solid_barriers=True 时进搜索内判可行性。
_BOARD_INTRUDE_MM = 87.5
def _build_solid_barriers(
    mid_boundary: int, bridge_open, board_intrude_mm: float = _BOARD_INTRUDE_MM,
) -> tuple[dict[int, list[float]] | None, tuple]:
    """实体挡板探测表：{格索引: 8 向可用横向空间 mm}（575=开阔哨兵）。

    中场挡板（D~N 列）实体侵入边界两侧格：上侧格（第 mid_boundary 行 1 基）
    朝南、下侧格朝北的探测值 = 侵入深度（默认 87.5mm）。桥洞开口列无挡板。
    返回 (map 或 None, 缓存键)；map 供 _dijkstra 查询，键供 _cached_dijkstra
    进缓存键（dict 不可哈希）。
    """
    holes = {c.col for c in bridge_open}
    smap: dict[int, list[float]] = {}
    for col in _BARRIER_COLS:
        if col in holes:
            continue
        # 上侧格（0 基 row=mid-1）朝南探测（_DIRS[2]=(0,1)，行号增=南）；
        # 下侧格（row=mid）朝北（_DIRS[6]=(0,-1)）。首版曾写反——
        # _DIRS 的 y 轴向下，(0,-1) 是北不是南。
        for row, d_idx in ((mid_boundary - 1, 2), (mid_boundary, 6)):
            # 行越界（mid_boundary 异常时）防御跳过
            if 0 <= row < ROWS:
                ci = row * COLS + col
                arr = smap.setdefault(ci, [575.0] * 8)
                arr[d_idx] = board_intrude_mm
    key = tuple(sorted((ci, tuple(a)) for ci, a in smap.items()))
    return (smap or None), key
def _turn_units(h1: int, h2: int) -> int:
    """两个航向之间相差几个 45°。"""
    d = abs(h1 - h2)
    return min(d, 8 - d)
def _zone_of(cell: CellRef, mid_boundary: int) -> str:
    """格号所属功能区（2026-10-15 现场确认的三区划分）。

    西区 = A~C 通长走廊（自由穿行）；东南区 = D~N 第 1~6 行；东北区 = D~N 第 7~11 行。
    """
    if cell.col <= 2:
        return "西"
    return "东南" if cell.row < mid_boundary else "东北"
def _crosses_mid(row_a: int, row_b: int, boundary: int) -> bool:
    """一条竖向边是否跨越中场分界（boundary 指上半区行数，如 6 = row5↔row6 之间）。"""
    return (row_a < boundary <= row_b) or (row_b < boundary <= row_a)
