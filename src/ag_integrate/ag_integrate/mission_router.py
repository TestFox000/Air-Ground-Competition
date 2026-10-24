"""任务路由器：世界态势 -> 绕障的有序格序列。

输入 world_model.WorldSnapshot，输出有序格号（tuple[str]），交给 car_adapter。

运动模型（对齐官方 path_executor 路线语法"直线连接、可对角"）：
    - 8 邻域移动：正交 1.0 格、对角 √2 格；对角移动要求两侧正交格均非障碍（防切角）；
    - 转向代价：阿克曼底盘（轴距 150mm、最小转弯半径≈0.34m），每 45° 转向
      ≈ 0.27m 弧长 ≈ 0.76 格，默认 turn_45=0.75（设 0 则退化为纯最短路）；
    - 朝向：start_heading / final_heading 支持 'x'（东）/ 'y'（北），
      对应车端 reset 断言朝向与路线末尾 ",x/,y"；中途朝向自动链式传递。

做法（近似全局最优）：
    - 障碍格为 blocked；必经点 = 雷 + 可通行桥洞格 + 蓝色敌靶格（开过去才能打击）；
    - 各必经点间用朝向感知 Dijkstra（状态 = 格 + 航向）求最省代价路径作边权，
      Held-Karp 位压 DP 求访问顺序（边权对"进入朝向"取乐观界；n 超过
      _TSP_EXACT_MAX_NODES 时退化为多起点最近邻，避免 2^n 卡死）；
    - 枚举所有达到乐观最优的顺序，逐段按真实朝向链拼接、顺路避让已踩格，
      取（真实代价, 重复踩格）最小者；
    - 不可达必经点剔除（跳过）；终点不可达报错并附连通性诊断
      （封锁点分类 + 开放桥门，2026-10-17）。

中场挡板：默认不建模；现场证实"只能从可通行格穿越半场"后传入 cross_gates
（允许穿越的格）。桥洞一格宽，穿越中线的移动限定正交且贴着 gate 格。
"""

from __future__ import annotations

import heapq
import warnings
from itertools import count, product
from math import gcd

from .field_frame import CellRef, ROWS, COLS, LETTERS
from .fire_guard import blue_targets_hit, decide_fire
from .world_model import WorldSnapshot

_SQRT2 = 2.0 ** 0.5
_INF = float("inf")
_BIG_INT = 1 << 30      # 累计转向/步数的"未触达"哨兵（平局裁决数组）
_EPS = 1e-6

# Held-Karp 精确解的必经点数上限。DP 是 2^n × n 状态：本机实测
# n=12 → 0.03s、n=14 → 0.17s、n=16 → 0.86s，再往上按 4 倍/点增长
# （n=20 约 4 分钟、2^20×20 ≈ 2000 万个表项，几百 MB，直接 OOM）。
# 正常对局走 plan_route_zoned 按区分桶，单次 _plan_core 的 n 通常 ≤8；
# 超过 14 说明感知把大片格误判成必经点，此时卡死 = 整局 0 分，
# 宁可退化为最近邻次优顺序（见 _greedy_orders）。
_TSP_EXACT_MAX_NODES = 14

# ---- 转角物理分级（移植自 V1019 路径规划交付包，原作者 yubinghuan，2026-10-20）----
# 物理前提（2026-10-16 现场确认）：阿克曼前轮转向 δ_max=28°、R_min=260mm、
# 车 278×140mm、格 350mm、距障碍边缘 ≥50mm。
#   45° 弧线格内可完成（need≈146mm）；90° 临界（need≈330mm，一格只给 225mm，
#   需占用内切对角格）；135°+ 单段绝无可能（need≈514mm）→ 只能多点掉头。
# 精确判据与逐转角体检见 turn_feasibility_v2.py（同批移植）；
# 方案与实测：docs/路径规划优化-转角物理分级-20261020.md

# 转角硬件守卫（opt-in，默认关）：90° 弯需内切对角格空置，被桶/挡板占用则删边。
# 实测无条件启用会让 test_router_fuzz::test_random_maps_invariants 失败
# （删边破坏随机图上的不变量），故默认关；需要时置 True 或运行时
# `mission_router._TURN_GUARD = True`。135°+ 不在此删边（真实场地存在必须
# 多点掉头的情形），由 turn_feasibility_v2 事后告警。
_TURN_GUARD = False

# 转角分级代价（默认启用）：索引 = 45° 单位数（0..4）。
#   45° 永远可行 → ×1；90° 临界 → ×2；135° 硬 → ×10；**180° 按倒车计 → ×1**。
# 180° 为什么便宜（2026-10-21 用户指出 + 实测）：阿克曼车可以**倒车**，回头不必
# 原地掉头——沿原路倒回去只花距离、不花转向。原值 ×10 等于把"回头"当多段掉头，
# 于是规划器被迫绕大圈去避免回头（实测 41 格/44.6/28 转向单元），改成"倒车"口径后
# 同一态势自己就走出用户手推的那条走法：33 格/38.0/22 单元（距离 −15%、转向 −21%）。
#   放宽版实测（局部图重建，终点不约束朝向）：43 格、需掉头 1、135°=0、雷 8/8
#   —— 相比 (1,1,8,50,50) 的 46 格/需掉头2/135°×1 是全面更优（Pareto 改善）。
# 回退：改回 (1.0, 1.0, 1.0, 1.0, 1.0) 即恢复纯最短路径行为；
#       把末位改回 10.0 即恢复"禁止倒车"的旧口径。
# ⚠️ 前提：车端能执行"沿原路倒回"（用户确认可倒车）。路点序列里的回头步由
#    path_executor 按最短路径跟随；若实测发现车会画圈而非倒车，把末位改回 10.0。
_TURN_GRADE = (1.0, 1.0, 2.0, 10.0, 1.0)

# 空间缺口代价（opt-in，默认 0 = 行为不变）：
# 代价 += _SPACE_W × max(0, need_mm − avail_mm)，need/avail 与
# turn_feasibility_v2 同一物理判据（转弯内侧侧向投影）——
# "优化目标 = 验收指标"，守卫/体检/规划三者用同一个量。
# 建议启用值：0.5（每 1mm 空间缺口 ≈ 0.5 格当量）。
_SPACE_W = 0.0

# 转角硬件需求（mm），索引 = 45° 单位数（0..4）：
#   0/146.2(45°)/330(90°)/513.8(135°)/578(180°)
_NEED_MM = (0.0, 146.2, 330.0, 513.8, 578.0)

# 航向索引（顺时针，0=正东）：车端 'x'=东(0)、'y'=北(6)
_DIRS = ((1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1))
_HEADING_IDX = {"x": 0, "y": 6}
_ALL_HEADINGS = tuple(range(8))

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


def _dijkstra(
    start: CellRef,
    blocked: frozenset[CellRef],
    cross_gates: frozenset[CellRef] | None,
    mid_boundary: int,
    entry_headings: tuple[int, ...],
    turn_45: float,
    allow_diagonal: bool,
    terminal: CellRef | None = None,
    cross_penalty: float = 0.0,
    tunnel_axes: frozenset[int] | None = None,
    tunnel_req: dict[int, int] | None = None,
) -> tuple[dict, dict, dict]:
    """(格, 航向) 状态空间的 Dijkstra；返回 (dist, prev, aux)。

    aux[(cell, h)] = (累计转向 45° 单位数, 累计步数)——同代价平局裁决用：
    代价相同时偏好更平直（转向少）再步数少的路线（步数/姿态平衡）。

    cross_penalty：每次跨越挡板边界附加的代价当量（默认 0=关闭，向后兼容），
    分区策略软约束——"一区扫完再转场"天然优于反复跨区。
    tunnel_axes：桥洞隧道格集合——进出必须同行正交（沿隧道轴东西向）、
    洞内禁止转向；一格宽桥洞斜进斜出 = 撞桥边（阿克曼底盘物理约束）。
    tunnel_req：隧道单向约束 {格索引: 行驶朝向}（0=向东 4=向西），
    通行方向=桥顶箭头指向，逆向入洞禁边。依据 docs/分区规划与感知约束-20261015.md。

    entry_headings 里的朝向以 0 代价起步（自由进入 = 不罚第一步转向）。
    terminal 指定时，该格成为终点态：进入即停、不许穿过再离开
    （避免“路过目标又折返”的路径）。

    平局裁决（2026-10-17 步数-姿态平衡）：代价、转向、步数三者沿路径
    可加且非负 → (代价, 转向, 步数) 字典序 Dijkstra 仍最优。turn_45=0
    （纯距离最短）时所有同距离形态全平局，本裁决保证取"转向最少 →
    步数最少"的最平直形态，不出锯齿路线；turn_45>0 时转向已计价，
    裁决只在精确补偿平局（如 Δ转向=4×0.75 ↔ Δ正交步=3）中生效。

    实现：整数态 (si = ci*8+h) + 预构边表 + 位图障碍，出口处物化为
    原 {(CellRef, h): ...} 字典接口（消费方 _arrival_state/_trace_states 不变）。
    """
    blocked_bits = bytearray(_N_CELLS)
    for c in blocked:
        blocked_bits[c.row * COLS + c.col] = 1
    gates_bits: bytearray | None = None
    if cross_gates is not None:
        gates_bits = bytearray(_N_CELLS)
        for c in cross_gates:
            gates_bits[c.row * COLS + c.col] = 1
    # 进洞邻格（隧道格东西两侧一格）集合：dzbrain 实测依据（L10→L9 横向进=124 拍蹭墙、
    # M10→M9→L9→K9 沿轴进=0 拍）——桥室冗余墙向进洞侧突出 ~0.135m，换行（含对角）
    # 进出该格的车身余量仅 ~2cm。2026-10-24 移植为边级硬规则：禁换行触碰进洞邻格。
    entry_nbr: frozenset[int] | None = None
    if tunnel_axes:
        _en = set()
        for _ci in tunnel_axes:
            _r, _c = divmod(_ci, COLS)
            for _c2 in (_c - 1, _c + 1):
                if 0 <= _c2 < COLS:
                    _en.add(_r * COLS + _c2)
        entry_nbr = frozenset(_en) if _en else None
    term_ci = terminal.row * COLS + terminal.col if terminal is not None else -1
    start_ci = start.row * COLS + start.col

    dist_i = [_INF] * (_N_CELLS * 8)
    turns_i = [_BIG_INT] * (_N_CELLS * 8)   # 累计转向 45° 单位（平局裁决）
    steps_i = [_BIG_INT] * (_N_CELLS * 8)    # 累计步数（平局裁决）
    prev_i = [-1] * (_N_CELLS * 8)
    settled = bytearray(_N_CELLS * 8)
    tick = count()
    pq: list[tuple[float, int, int, int, int]] = []
    for h in entry_headings:
        si = start_ci * 8 + h
        dist_i[si] = 0.0
        turns_i[si] = 0
        steps_i[si] = 0
        heapq.heappush(pq, (0.0, 0, 0, next(tick), si))
    push, pop = heapq.heappush, heapq.heappop
    _grade, _sw = _TURN_GRADE, _SPACE_W   # 热路径预取（转角分级，V1019 移植）
    while pq:
        d, t, s, _, si = pop(pq)
        if settled[si]:
            continue          # 字典序堆：首次出堆即最优，后续为陈旧副本
        settled[si] = 1
        if dist_i[si] < d - 1e-12:
            continue
        ci = si >> 3
        if ci == term_ci:
            continue  # 终点态：不再向外扩展
        h = si & 7
        row = ci // COLS
        for nci, nh, step, diag, o1, o2, dr, nrow in _EDGES[ci]:
            if blocked_bits[nci]:
                continue
            if diag:
                if not allow_diagonal:
                    continue
                # 防切角：对角移动要求两侧正交格均非障碍
                if blocked_bits[o1] or blocked_bits[o2]:
                    continue
            if (
                dr and gates_bits is not None
                and ((row < mid_boundary <= nrow) or (nrow < mid_boundary <= row))
            ):
                # 桥洞一格宽：只能正交穿越，且必须贴着 gate 格
                if diag or not (gates_bits[ci] or gates_bits[nci]):
                    continue
            # 桥洞隧道轴向约束：斜进斜出/洞内转向=撞桥边（物理必修）
            if tunnel_axes is not None:
                if nci in tunnel_axes and dr != 0:
                    continue            # 进出隧道格只允许同行正交（东西向）
                if ci in tunnel_axes:
                    rev = (h + 4) % 8
                    if dr != 0 or (nh != h and nh != rev):
                        continue        # 出洞只许直行穿洞或原路倒出（倒车=4×45°转向代价）
            # 隧道单向：逆向入洞禁边（规则方向=桥顶箭头指向）
            if tunnel_req is not None:
                req_h = tunnel_req.get(nci, -1)
                if req_h >= 0 and nh != req_h:
                    continue
            # 进洞邻格禁换行（2026-10-24，dzbrain 实测移植）：换行/对角进出
            # 进洞邻格 = 蹭桥室冗余墙（余量 ~2cm）；沿行（东西向）进出不受限。
            if entry_nbr is not None and dr and (ci in entry_nbr or nci in entry_nbr):
                continue
            # 转角硬件守卫（opt-in，见 _TURN_GUARD，V1019 移植）：90° 弯的
            # 内切对角格 = 当前格 + dir(h) + dir(nh)；越界或被桶/挡板占用则此边不存在。
            if _TURN_GUARD and _TURNS[h][nh] == 2:
                _cr = row + _DIRS[h][1] + _DIRS[nh][1]
                _cc = ci % COLS + _DIRS[h][0] + _DIRS[nh][0]
                if not (0 <= _cr < ROWS and 0 <= _cc < COLS):
                    continue
                if blocked_bits[_cr * COLS + _cc]:
                    continue
            # 穿越挡板边界附加代价：分区策略软约束（docs/分区规划与感知约束）
            extra = 0.0
            if cross_penalty and dr and (
                    (row < mid_boundary <= nrow) or (nrow < mid_boundary <= row)):
                extra = cross_penalty
            _tu = _TURNS[h][nh]
            # 转角分级代价（V1019 移植）：90°×2、135°+×10（见 _TURN_GRADE 注释）。
            # _grade/_sw 循环外预取：内循环热路径尽量只碰局部名（求解器路径
            # O(n²) 次规划对每一点开销都敏感，见 test_worst_case_solver_planning_speed）。
            nd = d + step + _tu * turn_45 * _grade[_tu] + extra
            if _sw > 0.0 and _tu >= 2:
                # 空间缺口代价：内切对角格被桶/挡板占用 → avail 仅 225mm
                # （半格 175 + 桶边 50）；空着 → 575mm。与 turn_feasibility_v2 同判据。
                _cr2 = row + _DIRS[h][1] + _DIRS[nh][1]
                _cc2 = ci % COLS + _DIRS[h][0] + _DIRS[nh][0]
                _av = 225.0
                if 0 <= _cr2 < ROWS and 0 <= _cc2 < COLS and not blocked_bits[_cr2 * COLS + _cc2]:
                    _av = 575.0
                _def = _NEED_MM[_tu] - _av
                if _def > 0.0:
                    nd += _sw * _def
            nt = t + _tu
            nst = s + 1
            nsi = nci * 8 + nh
            cur_d = dist_i[nsi]
            # 主序：代价更小直接胜出；同代价平局比 (转向, 步数) 字典序
            better = nd < cur_d - 1e-12 or (
                abs(nd - cur_d) <= 1e-12
                and (nt, nst) < (turns_i[nsi], steps_i[nsi])
            )
            if better:
                dist_i[nsi] = nd
                turns_i[nsi] = nt
                steps_i[nsi] = nst
                prev_i[nsi] = si
                push(pq, (nd, nt, nst, next(tick), nsi))

    dist: dict[tuple[CellRef, int], float] = {}
    prev: dict[tuple[CellRef, int], tuple[CellRef, int]] = {}
    aux: dict[tuple[CellRef, int], tuple[int, int]] = {}
    for si in range(_N_CELLS * 8):
        dd = dist_i[si]
        if dd < _INF:
            st = (_CELLS[si >> 3], si & 7)
            dist[st] = dd
            aux[st] = (turns_i[si], steps_i[si])
            p = prev_i[si]
            if p >= 0:
                prev[st] = (_CELLS[p >> 3], p & 7)
    return dist, prev, aux


def _arrival_state(
    dist: dict, cell: CellRef, want: int | None,
    aux: dict | None = None,
) -> tuple[float, tuple[CellRef, int] | None]:
    """到达 cell 的最省（代价, 终态）；want 指定到达航向。

    多个到达朝向同代价时用 aux[(cell,h)]=(累计转向, 步数) 平局裁决：
    取更平直（转向少）再步数少的到达态——等时路线里姿态更稳、
    航段更少（步数-姿态平衡，2026-10-17）。
    """
    if want is not None:
        return dist.get((cell, want), _INF), (cell, want)
    best = (_INF, None)
    best_key: tuple[int, int] = (_BIG_INT, _BIG_INT)
    for h in _ALL_HEADINGS:
        d = dist.get((cell, h), _INF)
        if d >= _INF:
            continue
        key = aux.get((cell, h), (_BIG_INT, _BIG_INT)) if aux is not None else (_BIG_INT, _BIG_INT)
        if (
            d < best[0] - 1e-12
            or (abs(d - best[0]) <= 1e-12 and key < best_key)
        ):
            best = (d, (cell, h))
            best_key = key
    return best


def _trace_states(prev: dict, end_state: tuple[CellRef, int]) -> list[CellRef]:
    cells: list[CellRef] = []
    state: tuple[CellRef, int] | None = end_state
    while state is not None:
        cells.append(state[0])
        state = prev.get(state)
    cells.reverse()
    return cells


def _cached_dijkstra(
    cache: dict | None,
    start: CellRef,
    blocked: frozenset[CellRef],
    cross_gates: frozenset[CellRef] | None,
    mid_boundary: int,
    entry_headings: tuple[int, ...],
    turn_45: float,
    allow_diagonal: bool,
    terminal: CellRef | None = None,
    cross_penalty: float = 0.0,
    tunnel_axes: frozenset[int] | None = None,
    tunnel_req: dict[int, int] | None = None,
) -> tuple[dict, dict, dict]:
    """_dijkstra 的缓存封装：键 = (起点, 进入朝向集, 终点态, 障碍集)。

    cross_gates/mid_boundary/turn_45/allow_diagonal 在一次 plan_route 调用内
    不变，由调用方保证；避让重算的 blocked 会变，故必须进键。
    cross_penalty/tunnel_axes/tunnel_req 同属本约定（一次调用内不变），不进键。
    """
    key = (start, entry_headings, terminal, blocked)
    if cache is not None and key in cache:
        return cache[key]
    result = _dijkstra(start, blocked, cross_gates, mid_boundary, entry_headings, turn_45, allow_diagonal, terminal=terminal,
                       cross_penalty=cross_penalty, tunnel_axes=tunnel_axes, tunnel_req=tunnel_req)
    if cache is not None:
        cache[key] = result
    return result


def _route_leg(
    frm: CellRef,
    to: CellRef,
    blocked: frozenset[CellRef],
    used: set[CellRef],
    cross_gates: frozenset[CellRef] | None,
    mid_boundary: int,
    entry: tuple[int, ...],
    want_arrival: int | None,
    turn_45: float,
    allow_diagonal: bool,
    cache: dict | None = None,
    cross_penalty: float = 0.0,
    tunnel_axes: frozenset[int] | None = None,
    tunnel_req: dict[int, int] | None = None,
    face_idx: int | None = None,
) -> tuple[list[CellRef] | None, int | None, float]:
    """frm→to 单段路由：优先走避开已踩格子的等代价路径。返回 (cells, 到达航向, 代价)。

    face_idx：射击停靠段的正对朝向（_DIRS 索引）。给出时按
    "到达朝向 + 停靠转身"联合最省选择到达态（朝向零转向准则：
    绕一格正对到达优于直进后原地 90° 转身），代价含转身。

    性能（2026-10-14 沙箱审计修复）：等最优顺序枚举最多 256 条，每条 9+ 段，
    逐段两次 Dijkstra 曾导致单局规划 30s+（ARM 车端会放大到分钟级，直接
    吃掉 180s 计时）。三处提速：
      1. Dijkstra 结果按 (frm, entry, terminal, blocked) 缓存——跨顺序/跨组合
         大量重复；中间段一律用全场图（terminal=None），使同一起点+进入朝向的
         所有段共享同一次计算；
      2. want_arrival=None 时不指定 terminal 是安全的：最省到达态的回溯链
         不可能中途先穿过 to（穿过意味着存在更便宜的到达，与"最省"矛盾），
         只有末段 h_final 约束才需要终点态防折返；
      3. 自由路径若本就零重复踩格，跳过避让重算（多数段的实际情况）。
    """
    if frm == to:
        h0 = entry[0] if len(entry) == 1 else (want_arrival if want_arrival is not None else 0)
        return [frm], h0, 0.0

    term = to if want_arrival is not None else None
    d_free, p_free, a_free = _cached_dijkstra(cache, frm, blocked, cross_gates, mid_boundary, entry, turn_45, allow_diagonal, terminal=term,
                                      cross_penalty=cross_penalty, tunnel_axes=tunnel_axes, tunnel_req=tunnel_req)

    def _pick(dist_map, aux_map) -> tuple[float, tuple[CellRef, int] | None]:
        if face_idx is not None and want_arrival is None:
            # 停靠段：全部到达朝向按"路程+停靠转身"联合计价取最省
            # （朝向零转向准则：绕一格正对到达优于直进后原地 90° 转身）
            best = None
            best_end: tuple[CellRef, int] | None = None
            best_key: tuple[int, int] = (_BIG_INT, _BIG_INT)
            for hh in _ALL_HEADINGS:
                c0 = dist_map.get((to, hh), _INF)
                if c0 >= _INF:
                    continue
                c1 = c0 + _TURNS[hh][face_idx] * turn_45
                # 同代价平局：行进过程更平直（转向少）/步数少的到达态优先
                key = aux_map.get((to, hh), (_BIG_INT, _BIG_INT)) if aux_map is not None else (_BIG_INT, _BIG_INT)
                if (
                    best is None or c1 < best - 1e-12
                    or (abs(c1 - best) <= 1e-12 and key < best_key)
                ):
                    best = c1
                    best_key = key
                    best_end = (to, hh)
            if best is not None and best_end is not None:
                return best, best_end
        return _arrival_state(dist_map, to, want_arrival, aux_map)

    cost, end = _pick(d_free, a_free)
    if end is None or cost >= _INF:
        return None, None, _INF

    avoid_cells = used - {frm, to}
    free_cells = _trace_states(p_free, end)
    if avoid_cells and not (set(free_cells) & avoid_cells):
        # 自由路径没有踩任何已踩格：避让重算的结果必然相同，直接返回
        return free_cells, end[1], cost

    avoid = frozenset(blocked | avoid_cells)
    if avoid != blocked:
        d_av, p_av, _ = _cached_dijkstra(cache, frm, avoid, cross_gates, mid_boundary, entry, turn_45, allow_diagonal, terminal=term,
                                  cross_penalty=cross_penalty, tunnel_axes=tunnel_axes, tunnel_req=tunnel_req)
        c_av, e_av = _pick(d_av, None)
        if e_av is not None and c_av <= cost + _EPS:
            return _trace_states(p_av, e_av), e_av[1], c_av
    return free_cells, end[1], cost


def _flatten(segments: list[list[CellRef]]) -> tuple[str, ...]:
    names: list[str] = []
    for seg in segments:
        for cell in seg:
            name = cell.name
            if not names or names[-1] != name:
                names.append(name)
    return tuple(names)


def _reconstruct_order(parent: list[list[int]], mask: int, last: int) -> list[int]:
    """按 DP 父指针回溯出一条最优访问顺序（保底用）。"""
    order: list[int] = []
    while last != -1:
        order.append(last)
        prev = parent[mask][last]
        mask ^= 1 << last
        last = prev
    order.reverse()
    return order


def _enumerate_optimal_orders(
    dp: list[list[float]],
    W: list[list[float]],
    dgoal: list[float],
    n: int,
    best_cost: float,
    call_cap: int = 20000,
    order_cap: int = 256,
) -> list[list[int]]:
    """枚举所有达到（乐观）最优代价的访问顺序（带上限，防对称场景组合爆炸）。"""
    orders: list[list[int]] = []
    budget = [call_cap]

    def rec(mask: int, last: int, suffix: tuple[int, ...]) -> None:
        if budget[0] <= 0 or len(orders) >= order_cap:
            return
        budget[0] -= 1
        if mask == (1 << last):  # 回到初始态：该顺序从起点直接过来
            orders.append([last, *suffix])
            return
        pmask = mask ^ (1 << last)
        row = dp[pmask]
        target = dp[mask][last]
        for i in range(n):
            if not (pmask >> i) & 1:
                continue
            if row[i] < _INF and abs(row[i] + W[i + 1][last + 1] - target) <= _EPS:
                rec(pmask, i, (last, *suffix))
                if len(orders) >= order_cap:
                    return

    full_mask = (1 << n) - 1
    for last in range(n):
        if dp[full_mask][last] < _INF and abs(dp[full_mask][last] + dgoal[last + 1] - best_cost) <= _EPS:
            rec(full_mask, last, ())
    return orders


def _assemble(
    order: list[int],
    nodes: list[CellRef],
    goal: CellRef,
    blocked: frozenset[CellRef],
    cross_gates: frozenset[CellRef] | None,
    mid_boundary: int,
    entry0: tuple[int, ...],
    h_final: int | None,
    turn_45: float,
    allow_diagonal: bool,
    cache: dict | None = None,
    fallback_any_heading: bool = False,
    cross_penalty: float = 0.0,
    tunnel_axes: frozenset[int] | None = None,
    tunnel_req: dict[int, int] | None = None,
    face_req: dict[str, int] | None = None,
    align_degraded: set[str] | None = None,
) -> tuple[tuple[str, ...] | None, float]:
    """按给定顺序逐段拼接（真实朝向链 + 顺路避让），返回 (路线, 真实总代价)。

    face_req：射击停靠格的"正对朝向"要求（格名 -> _DIRS 朝向索引）。
    靶向对齐（2026-10-23）：停靠段**强制先试**"到达朝向 = 正对朝向"（一次
    到位）——官方 path_executor 只跟折线、不知道靶在哪，开火瞬间的车头
    朝向只能由进段几何保证；终点态约束下的 Dijkstra 自动绕/曲进对齐
    （"逐渐调整"的几何形态由此涵盖：进段可多折几手 45° 小弯逐步转过来），
    到达即正对、无停靠转身，代价口径也更真实。对齐到达不可行（对侧进段
    被堵）才退回"代价择优 + 停靠转身"假设（残余偏差交执行器停转-爬行/
    云台消化），并把格名记入 align_degraded 供上层一次性告警。

    fallback_any_heading：终点朝向约束（h_final）物理不可达时
    （例：终点格是口袋入口，朝向到达须先踩终点），允许以任意朝向进终点——
    放弃终点朝向分，保住完赛。严格模式（False）下不可达即整段返回 None。
    """
    used: set[CellRef] = {nodes[0]}
    segments: list[list[CellRef]] = []
    total = 0.0
    cur, entry = nodes[0], entry0
    for i in order:
        w = nodes[i + 1]
        fidx = face_req.get(w.name) if face_req else None
        seg = None
        h = None
        c = _INF
        if fidx is not None:
            # 一次到位：终点态约束（到达朝向=正对朝向）的进段。
            seg, h, c = _route_leg(cur, w, blocked, used, cross_gates, mid_boundary,
                                   entry, fidx, turn_45, allow_diagonal, cache,
                                   cross_penalty, tunnel_axes, tunnel_req)
        if seg is None:
            # 普通段（fidx=None）的主路径；射击段则是"逐渐调整"兜底——对齐
            # 到达不可行（对侧进段被堵），退回代价择优 + 停靠转身假设，并
            # 记录降级格供上层一次性告警。
            seg, h, c = _route_leg(cur, w, blocked, used, cross_gates, mid_boundary,
                                   entry, None, turn_45, allow_diagonal, cache,
                                   cross_penalty, tunnel_axes, tunnel_req, face_idx=fidx)
            if seg is not None and fidx is not None and align_degraded is not None:
                align_degraded.add(w.name)
        if seg is None:
            return None, _INF
        segments.append(seg)
        used.update(seg)
        total += c
        # 射击停靠后车头已转向正对朝向（打完后从该朝向继续行驶）
        cur = w
        entry = (face_req[w.name],) if (face_req and w.name in face_req) else (h,)
    seg, _h, c = _route_leg(cur, goal, blocked, used, cross_gates, mid_boundary, entry, h_final, turn_45, allow_diagonal, cache,
                               cross_penalty, tunnel_axes, tunnel_req)
    if seg is None and fallback_any_heading and h_final is not None:
        seg, _h, c = _route_leg(cur, goal, blocked, used, cross_gates, mid_boundary, entry, None, turn_45, allow_diagonal, cache,
                               cross_penalty, tunnel_axes, tunnel_req)
    if seg is None:
        return None, _INF
    segments.append(seg)
    total += c
    return _flatten(segments), total


def _classify_blocked(cell: CellRef, diag: dict | None) -> str:
    """封锁格归类（诊断用）：障碍/靶体/轨道/桥墙/封锁。"""
    if diag:
        if cell.name in diag.get("obstacles", ()):
            return "障碍"
        if cell.name in diag.get("targets", ()):
            return "靶体"
        if cell.name in diag.get("tracks", ()):
            return "轨道"
        if cell.col in diag.get("bridge_cols", ()):
            return "桥墙"
    return "封锁"


def _unreachable_detail(start: CellRef, goal: CellRef, dist0: dict,
                        blocked: frozenset[CellRef],
                        cross_gates: frozenset[CellRef] | None,
                        mid_boundary: int, diag: dict | None) -> str:
    """不可达诊断（2026-10-17 修复）：起点连通域规模 + 封锁点分类 +
    开放桥门——现场拿到报错就知道该查哪块板/哪个桶。

    封锁点 = 与起点连通域 8 邻接的 blocked 格；连通域取自 Dijkstra
    dist0（任意朝向可达即算）。"""
    reach = {c for (c, _h), d in dist0.items() if d < _INF} or {start}
    sealing: dict[str, set[str]] = {}
    for c in reach:
        for dc in (-1, 0, 1):
            for dr in (-1, 0, 1):
                if dc == dr == 0:
                    continue
                col, row = c.col + dc, c.row + dr
                if not (0 <= col < COLS and 0 <= row < ROWS):
                    continue
                n = CellRef(col, row)
                if n in reach or n not in blocked:
                    continue
                sealing.setdefault(_classify_blocked(n, diag), set()).add(n.name)
    parts = [f"goal {goal.name} 不可达：起点 {start.name} 连通域仅 {len(reach)} 格"]
    for kind in ("障碍", "靶体", "轨道", "桥墙", "封锁"):
        if sealing.get(kind):
            names = sorted(sealing[kind])
            parts.append(f"{kind}[{','.join(names[:8])}{'…' if len(names) > 8 else ''}]")
    if diag:
        open_gates = sorted(diag.get("open_gates", ()))
        if open_gates:
            parts.append(f"桥门开放[{','.join(open_gates)}]")
        elif diag.get("bridge_cols"):
            parts.append("桥门全部关闭（箭头格缺失或被实物占用）")
    if cross_gates is not None:
        parts.append(f"中线穿越口 {len(cross_gates)} 格")
    parts.append("排查：门格是否被桶占用、箭头格是否漏检、障碍是否误检堵死唯一通道")
    return "；".join(parts)


def _greedy_orders(W: list[list[float]], n: int) -> list[list[int]]:
    """多起点最近邻访问顺序（Held-Karp 超限时用）。

    W 下标与 DP 同口径：0=起点，k+1=必经点 k。每条 O(n^2)，n 条共 O(n^3)，
    n=20 也是微秒级。多起点比单起点稳——单起点在"起点位于图边缘"时容易
    一路走回头路，而这里只能拼一次（不像精确解有等代价顺序可枚举）。
    """
    orders: list[list[int]] = []
    for first in range(n):
        unvisited = set(range(n))
        unvisited.discard(first)
        order = [first]
        cur = first
        while unvisited:
            nxt = min(unvisited, key=lambda j: (W[cur + 1][j + 1], j))
            order.append(nxt)
            unvisited.discard(nxt)
            cur = nxt
        orders.append(order)
    return orders


def _held_karp_orders(W: list[list[float]], dgoal: list[float],
                      n: int) -> list[list[int]]:
    """Held-Karp 精确排序 -> 候选访问顺序（保底顺序 + 全部等代价顺序）。

    dp[mask][i] = 从起点出发、恰好经过 mask 内必经点、当前停在 i 的最少代价。
    必经点编号 0..n-1 对应 nodes[1..n]；W/dgoal 下标 0 = 起点。
    调用方须先保证 n <= _TSP_EXACT_MAX_NODES：状态数是 2^n × n。
    """
    full = 1 << n
    dp = [[_INF] * n for _ in range(full)]
    parent = [[-1] * n for _ in range(full)]
    for i in range(n):
        dp[1 << i][i] = W[0][i + 1]
    for mask in range(1, full):
        row = dp[mask]
        for i in range(n):
            cur = row[i]
            if cur >= _INF or not (mask >> i) & 1:
                continue
            base = W[i + 1]
            for j in range(n):
                if (mask >> j) & 1:
                    continue
                nm = mask | (1 << j)
                nd = cur + base[j + 1]
                if nd < dp[nm][j] - 1e-12:
                    dp[nm][j] = nd
                    parent[nm][j] = i

    best_cost, best_last = _INF, -1
    for i in range(n):
        c = dp[full - 1][i] + dgoal[i + 1]
        if c < best_cost:
            best_cost, best_last = c, i
    if best_last < 0:
        # 全集没有任何可行顺序（单向隧道等方向约束把所有排列都堵死）。
        # 此时 best_last=-1 会回溯出**空顺序**，拼装成"起点直奔终点、
        # 跳过全部雷与桥洞"——正是最坏的静默灾难，改走贪心候选并告警。
        warnings.warn(
            f"{n} 个必经点无可行访问顺序（乐观代价全为不可达），退化为"
            "最近邻顺序——检查单向隧道方向是否标反",
            RuntimeWarning, stacklevel=4,
        )
        return _greedy_orders(W, n)
    return ([_reconstruct_order(parent, full - 1, best_last)]
            + _enumerate_optimal_orders(dp, W, dgoal, n, best_cost))


def _plan_core(
    start: CellRef,
    goal: CellRef,
    blocked: frozenset[CellRef],
    required: dict[str, CellRef],
    cross_gates: frozenset[CellRef] | None,
    mid_boundary: int,
    entry0: tuple[int, ...],
    h_final: int | None,
    turn_45: float,
    allow_diagonal: bool,
    cache: dict | None = None,
    cross_penalty: float = 0.0,
    tunnel_axes: frozenset[int] | None = None,
    tunnel_req: dict[int, int] | None = None,
    face_req: dict[str, int] | None = None,
    diag: dict | None = None,
    align_degraded: set[str] | None = None,
) -> tuple[tuple[str, ...], float]:
    """给定障碍集与必经点，输出（最优路线, 真实总代价）。
    diag：不可达诊断上下文（障碍/靶/轨道名集 + 桥列/开放门），由 plan_route 提供。
    align_degraded：靶向对齐降级格收集集（见 _assemble），None 表示不需要收集。"""
    dist0, _, _ = _cached_dijkstra(cache, start, blocked, cross_gates, mid_boundary, _ALL_HEADINGS, turn_45, allow_diagonal,
                     cross_penalty=cross_penalty, tunnel_axes=tunnel_axes, tunnel_req=tunnel_req)
    if _arrival_state(dist0, goal, None)[0] >= _INF:
        raise ValueError(_unreachable_detail(
            start, goal, dist0, blocked, cross_gates, mid_boundary, diag))
    # 注意：可达性检查必须按"任意朝向"——朝向到达不可行由拼装层降级兜底，
    # 若在此按 h_final 判不可达，会把"丢 5 分朝向"误升级成"整局崩溃"。

    # 剔除不可达必经点（与旧语义一致：跳过）；nodes[0] 恒为起点。
    # 但不再静默：逐格告警并按功能分区标注——整区被跳过大概率是唯一通道
    # （桥洞/走廊口）被障碍误检堵死（交界格误检是 documented 顽疾）。
    reachable_required = []
    dropped = []
    for c in required.values():
        (reachable_required if _arrival_state(dist0, c, None)[0] < _INF else dropped).append(c)
    if dropped:
        by_zone: dict[str, list[str]] = {}
        for c in dropped:
            by_zone.setdefault(_zone_of(c, mid_boundary), []).append(c.name)
        detail = "；".join(f"{z}区[{','.join(sorted(ns))}]" for z, ns in sorted(by_zone.items()))
        warnings.warn(
            f"{len(dropped)} 个必经点不可达被跳过：{detail}。",
            RuntimeWarning, stacklevel=3,
        )
    nodes = [start] + reachable_required
    n = len(nodes) - 1
    if n == 0:
        seg, _h, _c = _route_leg(start, goal, blocked, set(), cross_gates, mid_boundary, entry0, h_final, turn_45, allow_diagonal, cache,
                             cross_penalty, tunnel_axes, tunnel_req)
        if seg is None and h_final is not None:
            # 终点朝向到达不可行（口袋地形）：降级任意朝向进终点
            warnings.warn(
                f"goal {goal.name} 以朝向到达不可行，降级为任意朝向进终点"
                "（终点朝向 5 分不保）",
                RuntimeWarning, stacklevel=3,
            )
            seg, _h, _c = _route_leg(start, goal, blocked, set(), cross_gates, mid_boundary, entry0, None, turn_45, allow_diagonal, cache,
                                 cross_penalty, tunnel_axes, tunnel_req)
        if seg is None:
            raise ValueError(_unreachable_detail(
                start, goal, dist0, blocked, cross_gates, mid_boundary, diag))
        return _flatten([seg]), _c

    # 每个节点做一次自由进入朝向的 Dijkstra，得两两边权（乐观界）。
    # aux 供等代价边权平局裁决（姿态更平直的到达态优先）。
    _maps = [
        _cached_dijkstra(cache, c, blocked, cross_gates, mid_boundary, _ALL_HEADINGS, turn_45, allow_diagonal,
                          cross_penalty=cross_penalty, tunnel_axes=tunnel_axes, tunnel_req=tunnel_req)
        for c in [start] + nodes[1:]
    ]
    dists = [m[0] for m in _maps]
    auxs = [m[2] for m in _maps]
    W = [[_INF] * (n + 1) for _ in range(n + 1)]
    for i in range(n + 1):
        W[i][i] = 0.0
        for j in range(n + 1):
            if j != i:
                W[i][j] = _arrival_state(dists[i], nodes[j], None, auxs[i])[0]
    dgoal = [_arrival_state(dists[i], goal, h_final)[0] for i in range(n + 1)]
    if h_final is not None and all(d >= _INF for d in dgoal):
        # 终点朝向到达全局不可行（口袋地形）：TSP 排序改用任意朝向界，
        # 朝向降级由拼装层兜底——否则 dgoal 全 INF 会导致回溯出空顺序，
        # 退化成"起点直奔终点、跳过全部雷"的灾难路线。
        dgoal = [_arrival_state(dists[i], goal, None)[0] for i in range(n + 1)]

    # 访问顺序候选：n 在上限内走 Held-Karp 精确解，超限退化为最近邻。
    if n > _TSP_EXACT_MAX_NODES:
        warnings.warn(
            f"必经点 {n} 个超过精确 TSP 上限 {_TSP_EXACT_MAX_NODES}，退化为"
            "最近邻顺序（路线次优）——检查感知是否把大片格误判为必经点",
            RuntimeWarning, stacklevel=3,
        )
        candidates = _greedy_orders(W, n)
    else:
        candidates = _held_karp_orders(W, dgoal, n)

    def assemble(order: list[int]) -> tuple[tuple[str, ...] | None, float]:
        return _assemble(order, nodes, goal, blocked, cross_gates, mid_boundary, entry0, h_final, turn_45, allow_diagonal, cache,
                         cross_penalty=cross_penalty, tunnel_axes=tunnel_axes, tunnel_req=tunnel_req, face_req=face_req,
                         align_degraded=align_degraded)

    # 保底顺序 + 枚举其余等代价顺序，取（真实代价, 重复踩格, 姿态形态）最小者。
    # 形态键 = (总转向 45° 单位, 总步数)：等时代价里更平直、航段更少者优先
    # （步数-姿态平衡，2026-10-17）。拼接数硬上限：对称场景下等代价顺序
    # 可达 256 条；64 次足以覆盖真实的并列分支（实测场景 <10 次即收敛）。
    best_route: tuple[str, ...] | None = None
    best_cost_true: float = _INF
    best_dup = 1 << 30
    best_shape: tuple[int, int] = (_BIG_INT, _BIG_INT)
    for order in candidates[:64]:
        route, cost_true = assemble(order)
        if route is None:
            continue
        dup = len(route) - len(set(route))
        shape = _route_shape(route)
        key_better = cost_true < best_cost_true - _EPS or (
            abs(cost_true - best_cost_true) <= _EPS
            and (dup, shape) < (best_dup, best_shape)
        )
        if best_route is None or key_better:
            best_route, best_cost_true, best_dup, best_shape = route, cost_true, dup, shape
    if best_route is None and h_final is not None:
        # 终点朝向到达物理不可行（如终点格是口袋入口，朝向到达须先踩终点）。
        # 降级：任意朝向进终点——放弃终点朝向分，保住完赛（崩溃 = 整局 0 分）。
        warnings.warn(
            f"goal {goal.name} 以朝向到达不可行，降级为任意朝向进终点"
            "（终点朝向 5 分不保）",
            RuntimeWarning, stacklevel=3,
        )
        for order in candidates[:64]:
            route, cost_true = _assemble(
                order, nodes, goal, blocked, cross_gates, mid_boundary,
                entry0, h_final, turn_45, allow_diagonal, cache,
                fallback_any_heading=True,
                cross_penalty=cross_penalty, tunnel_axes=tunnel_axes, tunnel_req=tunnel_req,
                face_req=face_req, align_degraded=align_degraded,
            )
            if route is None:
                continue
            dup = len(route) - len(set(route))
            shape = _route_shape(route)
            key_better = cost_true < best_cost_true - _EPS or (
                abs(cost_true - best_cost_true) <= _EPS
                and (dup, shape) < (best_dup, best_shape)
            )
            if best_route is None or key_better:
                best_route, best_cost_true, best_dup, best_shape = route, cost_true, dup, shape
    if best_route is None:
        # 所有候选顺序都拼不出完整路线。原来是 `assert best_route is not None`：
        # `python -O` 会把 assert 整条删掉，None 便顺着 plan_route 流进
        # ensure_mine_center_pass 再到 commands.route，现场表现为"车收到一条
        # 莫名其妙的路线"而不是一个报错。抛 ValueError 则能被 plan_route 的
        # 射击位组合循环 `except ValueError: continue` 接住，换下一组合重试。
        raise ValueError(
            f"{len(candidates[:64])} 条候选访问顺序均无法拼接完整路线："
            f"start={start.name} goal={goal.name} 必经点={n} "
            f"障碍={len(blocked)}。终点本身可达（前置检查已过），"
            "多半是必经点之间的朝向/隧道约束把每一段都堵死；"
            "排查单向桥洞方向是否标反、必经点是否落在不可踩格上"
        )
    return best_route, best_cost_true


_ORTHO_STEPS = ((1, 0), (-1, 0), (0, 1), (0, -1))


def _pos(name: str) -> tuple[int, int]:
    """格号 -> (col, row)（0 基）。"""
    return LETTERS.index(name[0]), int(name[1:]) - 1


# 单位位移 -> 航向索引（_route_shape 用；只覆盖 8 邻域单位步）
_DIR_INDEX: dict[tuple[int, int], int] = {d: i for i, d in enumerate(_DIRS)}


def _route_shape(route: tuple[str, ...]) -> tuple[int, int]:
    """路线形态键 (总转向 45° 单位数, 总步数)——等代价平局裁决用。

    从相邻格位移推航向链、统计相邻航向差。首步的起步转向不计入
    （起点朝向对各候选一致，口径公平）。射击停靠格上的原地转身
    不在几何航向链里，但同局各候选的停靠集相同，不影响比较结论。
    """
    turns = 0
    prev_h = -1
    for i in range(1, len(route)):
        a, b = _pos(route[i - 1]), _pos(route[i])
        h = _DIR_INDEX.get((b[0] - a[0], b[1] - a[1]), -1)
        if prev_h >= 0 and h >= 0 and h != prev_h:
            turns += _TURNS[prev_h][h]
        if h >= 0:
            prev_h = h
    return turns, max(len(route) - 1, 0)


def ensure_mine_center_pass(
    route: tuple[str, ...],
    mine_names: frozenset[str] | set[str],
    blocked: frozenset[CellRef],
    mid_boundary: int = _MID_BOUNDARY,
    cross_gates: frozenset[CellRef] | None = None,
    tunnel_names: frozenset[str] | None = None,
) -> tuple[str, ...]:
    """雷格过心约束：>45° 转向处的雷格补一格"过心穿越点"。

    排雷判据是"任意轮中心进入雷标 12cm 圆"（docs/执行层深优化-20261016
    问题 3）。executor 的 arrival_threshold ≈ 0.157m：雷格处于路线拐点时，
    车在距雷心 15.7cm 处即宣布到点并切向下一段，物理轨迹切角——轮中心
    可能从未进入 12cm 圆，排雷失败。直线途经的雷格天然过心（纯追踪贴线），
    45° 及以下拐角由拐角保护（lookahead 钳顶点）自然贴顶点过弯（主证明
    路线实测），只有 >45° 拐角触发 PIVOT 停转-爬行、顶点不再保证到达
    ——本函数只修 >45°（PIVOT 阈值同口径）。

    修复：雷格处转向 >45° 时，沿进入方向在雷后补一格过心穿越点——雷格
    变成直线段中间点（必过心），转向推迟到过心穿越点（普通格，无判分风险）。

    补点等效改写两条边（雷→补点、补点→原后继），必须逐条过 Dijkstra
    同款边界规则，任何一条不过就放弃补点（该雷退化为拐点，尽力而为）：
    - 越界 / 落在障碍格上；
    - 补点或雷格落在 K 列（桥体结构 + 车端 BRIDGE_BLOCKS 强制直行
      保护：在 K 列补点 = 以非轴向进桥/在保护点上转向，SIL 实测引发
      直行接管后失控出图）；
    - 对角边两侧正交格须非障碍（防切角）；
    - 跨挡板边界：只许正交竖向且端点有 gate（无 gate 建模时不查）——
      典型反例 K7 贴挡板雷，向北补点会穿挡板，必须拒绝；
    - 补点/原后继落在桥洞隧道格时只许东西向进出（洞内转向=撞桥边）。
    补点→原后继只接受 8 邻接几何（保持路线逐格 8 邻接约定，fuzz 不变量
    锁死）；骑士步/2 格长弦一概拒绝。

    逐格左→右扫描：补点本身也可能是雷（连环雷），其"实际进入方向"以补点
    为前驱计算，天然收敛（每次补点只增加一格）。
    """
    if len(route) < 3:
        return tuple(route)
    mines = {m.strip().upper() for m in mine_names}
    if not mines:
        return tuple(route)
    blocked_names = {c.name for c in blocked}
    gates_names = ({g.name for g in cross_gates} if cross_gates is not None
                   else None)
    tunnel = tunnel_names or frozenset()
    # 桥体列（K，col 10）：场地固定结构，2026-10-15 现场核实。
    _BRIDGE_COL = LETTERS.index("K")

    def _cell_ok(c: CellRef) -> bool:
        return (0 <= c.col < COLS and 0 <= c.row < ROWS
                and c.name not in blocked_names)

    def _diag_sides_ok(a: CellRef, b: CellRef) -> bool:
        """对角边 a→b 的防切角：两侧正交格均须在界内且非障碍。"""
        dc, dr = b.col - a.col, b.row - a.row
        for s in (CellRef(a.col + dc, a.row), CellRef(a.col, a.row + dr)):
            if not _cell_ok(s):
                return False
        return True

    def _leg_ok(a: CellRef, b: CellRef) -> bool:
        """单条 8 邻接边 a→b 过 Dijkstra 同款边界规则。"""
        if not (_cell_ok(a) and _cell_ok(b)):
            return False
        dc, dr = b.col - a.col, b.row - a.row
        diag = dc != 0 and dr != 0
        if diag and not _diag_sides_ok(a, b):
            return False
        # 跨挡板：正交竖向 + 端点是 gate（无 gate 建模时挡板不存在，不查）
        if (cross_gates is not None and dr != 0
                and _crosses_mid(a.row, b.row, mid_boundary)):
            if diag:
                return False
            if not (a.name in gates_names or b.name in gates_names):
                return False
        # 隧道格：只许东西向进出（含洞内直行），竖向=撞桥边
        if dr != 0 and (a.name in tunnel or b.name in tunnel):
            return False
        return True

    work = [c.strip().upper() for c in route]
    out: list[str] = [work[0]]
    for i in range(1, len(work)):
        cur = work[i]
        if cur in mines and i + 1 < len(work):
            mine = CellRef(*_pos(cur))
            nxt = CellRef(*_pos(work[i + 1]))
            d_in = _norm_dir(_pos(out[-1]), _pos(cur))
            d_out = _norm_dir(_pos(cur), _pos(work[i + 1]))
            # 只有相邻单步方向的转向角才有定义；跨格跳线不做补点
            if (d_in in _DIRS and d_out in _DIRS and d_in != (0, 0)
                    and d_in != d_out
                    and _turn_units(_DIRS.index(d_in), _DIRS.index(d_out)) >= 2
                    and mine.col != _BRIDGE_COL):
                ov = CellRef(mine.col + d_in[0], mine.row + d_in[1])
                if not _cell_ok(ov) or ov.col == _BRIDGE_COL:
                    out.append(cur)
                    continue
                # 补点→原后继只接受 8 邻接几何（保持路线逐格 8 邻接约定，
                # fuzz 不变量锁死）；骑士步/2 格长弦一概拒绝（该雷退化为拐点）
                vc, vr = nxt.col - ov.col, nxt.row - ov.row
                mid_ok = (abs(vc) <= 1 and abs(vr) <= 1 and (vc, vr) != (0, 0))
                if mid_ok and _leg_ok(mine, ov) and _leg_ok(ov, nxt):
                    out.append(cur)              # 雷格（直线段中间点）
                    out.append(ov.name)          # 过心穿越点：转向推迟到这
                    continue
        out.append(cur)
    return tuple(out)


_FACE_OPP = {"S": 6, "N": 2, "E": 4, "W": 0}   # 蓝面朝向 → 车需正对的方向（_DIRS 索引）


def _face_heading(frm: CellRef, to: CellRef) -> int:
    """frm→to 的正交朝向索引（射击停靠后车头应正对的朝向，_DIRS 索引）。"""
    dc = (to.col > frm.col) - (to.col < frm.col)
    dr = (to.row > frm.row) - (to.row < frm.row)
    return _DIRS.index((dc, dr))
_SHOOT_COMBO_CAP = 64  # 蓝靶射击位组合枚举上限，超过则退化为逐靶就近


def _shoot_candidates(
    target: CellRef,
    blocked: frozenset[CellRef],
    target_cells: dict[str, CellRef],
) -> list[CellRef]:
    """固定靶的候选停靠射击格：正交相邻格（车端正对靶心开火最准），
    排除障碍/桥洞墙/其他靶格。"""
    out = []
    for dc, dr in _ORTHO_STEPS:
        c = CellRef(target.col + dc, target.row + dr)
        if not (0 <= c.col < COLS and 0 <= c.row < ROWS):
            continue
        if c in blocked or c.name in target_cells:
            continue
        out.append(c)
    return sorted(out, key=lambda c: c.name)


def _shoot_candidates_moving(
    t: "TargetRef",
    blocked: frozenset[CellRef],
) -> list[CellRef]:
    """移动靶的候选停靠射击格：轨道两端在蓝面朝向一侧的正交邻格。

    车停这类格上，靶移动到正对端点时恰好正交相邻（距离 1 格），
    车头正对靶心开火最准；红面朝向另一侧，不可能误伤。
    轨道水平（两端同行）时 blue_face='S'(row+1)/'N'(row-1)；
    垂直（两端同列）时 'E'(col+1)/'W'(col-1)。
    """
    a, b = t.track
    if a.row == b.row:  # 水平轨道
        dr = 1 if t.blue_face == "S" else -1
        cells = (CellRef(a.col, a.row + dr), CellRef(b.col, b.row + dr))
    else:               # 垂直轨道
        dc = 1 if t.blue_face == "E" else -1
        cells = (CellRef(a.col + dc, a.row), CellRef(b.col + dc, b.row))
    out = []
    for c in cells:
        if not (0 <= c.col < COLS and 0 <= c.row < ROWS):
            continue
        if c in blocked:
            continue
        out.append(c)
    return sorted(out, key=lambda c: c.name)


def plan_route(
    snapshot: WorldSnapshot,
    start: CellRef,
    goal: CellRef,
    cross_gates: frozenset[CellRef] | None = None,
    mid_boundary: int = _MID_BOUNDARY,
    *,
    start_heading: str | None = None,
    final_heading: str | None = None,
    turn_45: float = 0.75,
    allow_diagonal: bool = True,
    crossing_penalty: float = 0.0,
    tunnel_dirs: dict[str, str] | None = None,
    require_gates: bool = True,
) -> tuple[str, ...]:
    """输出有序格号；转向按 turn_45/45° 计费，起点/终点朝向可约束。

    crossing_penalty：每次跨越挡板边界的附加代价（格当量，默认 0=关闭）。
    tunnel_dirs：桥洞隧道方向约束 {格号: 'E'/'W'/'EW'}——规则方向=桥顶箭头
    指向（E=西向东）；'EW'=方向未知时仍强制轴向（东西直行、洞内不转向）。
    默认 None=对 snapshot.passable 的全部隧道格自动施加 'EW' 轴向约束
    （物理必修：斜进一格宽桥洞会撞桥边）；传 {} 可完全关闭（遗留行为）。

    标靶格（红/蓝）物理上立着靶体，一律不可踩；每个蓝色敌方靶必须在
    其正交邻格停靠射击（车头正对靶心，UDP 朝向对准即打得准），
    停靠格作为必经点参与全局排序；红靶只挡路、不停靠、不打。
    发现态靶标（is_blue=None，红蓝未辨）以确认点（距靶 1 格的正交
    邻格）为必经点——路过即近距复判，face 分辨后由 replan 守卫接手。

    雷格过心约束（ensure_mine_center_pass）：处于路线拐点的雷格沿进入
    方向补一格"过心穿越点"，保证轮中心进入 12cm 判分圆（直线途经天然
    过心，只有拐点雷有切角风险）。

    require_gates：桥门格（snapshot.passable）是否作为**必经点**。默认 True
    ——规则里"桥洞通道协同穿梭"每段 10 分，正式任务路线必须穿门。
    传 False 时桥门只保留"桥墙开口"的连通性语义、不再强制绕行：
    供纯移动代价测量（pipeline 的 leg_s）使用。两种角色必须能分开，
    否则每段点对点代价都被迫绕经全部门格（见 pipeline._solve_with_joint_solver）。
    """
    if start_heading is not None and start_heading not in _HEADING_IDX:
        raise ValueError(f"start_heading 只支持 'x'/'y'，收到 {start_heading!r}")
    if final_heading is not None and final_heading not in _HEADING_IDX:
        raise ValueError(f"final_heading 只支持 'x'/'y'，收到 {final_heading!r}")
    entry0 = (_HEADING_IDX[start_heading],) if start_heading else _ALL_HEADINGS
    h_final = _HEADING_IDX[final_heading] if final_heading else None

    # 桥门占用过滤（2026-10-17 修复）：门格被实物（障碍/靶/轨道）占用 → 门失效并告警。
    # 物理优先：桶压在桥门上就是过不去；旧逻辑把 passable 格从障碍集豁免，
    # 会让路线从实物上开过去。桥列仍按原始 passable 推导——门全关时桥体仍是墙。
    raw_passable = tuple(snapshot.passable)
    bridge_cols = {c.col for c in raw_passable}
    occupied = {c.name for c in snapshot.obstacles}
    occupied |= {t.cell.name for t in snapshot.targets}
    occupied |= {c.name for t in snapshot.targets if t.track for c in t.track}
    gate_hits = sorted(c.name for c in raw_passable if c.name in occupied)
    if gate_hits:
        warnings.warn(
            f"桥门被实物占用已禁用: {','.join(gate_hits)}"
            f"（该格按障碍处理；若确认误检，从 obstacles 移除后重规划）",
            RuntimeWarning, stacklevel=2)
    passable_cells = tuple(c for c in raw_passable if c.name not in occupied)

    # 隧道约束解析：方向 'E'/'W'（单向）或 'EW'（双向但强制轴向）。
    # 默认对所有 passable 隧道格强制 'EW'（物理上斜进必撞边，无需等规则方向）。
    tunnel_req: dict[int, int] | None = None
    tunnel_axes: frozenset[int] | None = None
    if tunnel_dirs is None:
        axes = {c.row * COLS + c.col for c in passable_cells}
        tunnel_axes = frozenset(axes) if axes else None
    elif tunnel_dirs:
        axes = set()
        tunnel_req = {}
        for name, d in tunnel_dirs.items():
            cn = name.strip().upper()
            dd = str(d).strip().upper()
            if dd not in ("E", "W", "EW"):
                raise ValueError(f"tunnel_dirs[{name!r}] 只支持 'E'/'W'/'EW'，收到 {d!r}")
            cell = CellRef(LETTERS.index(cn[0]), int(cn[1:]) - 1)
            if not (0 <= cell.row < ROWS and 0 <= cell.col < COLS):
                raise ValueError(f"tunnel_dirs 格号越界: {name!r}")
            ci = cell.row * COLS + cell.col
            axes.add(ci)
            if dd != "EW":
                tunnel_req[ci] = 0 if dd == "E" else 4
        tunnel_axes = frozenset(axes)
        if not tunnel_req:
            tunnel_req = None
    else:
        tunnel_axes = None     # 显式 {} = 全关（遗留行为，不推荐）

    cross_penalty = crossing_penalty  # 内部链路简称（对外参数 crossing_penalty）

    # 桥洞列推导：passable（ARROW）所在列即桥洞列。
    # 桥洞列物理上每格都有缺口，但规则上只有带箭头的格子允许通过，
    # 其余格视为墙。箭头位置每场随机。多个箭头之间不默认连通。
    # bridge_cols / passable_cells 已在占用过滤处算好（门被占用时桥列不变）。
    passable_names = {c.name for c in passable_cells}
    bridge_open: set[CellRef] = set()       # 桥洞列中可通行的格
    bridge_walls: set[CellRef] = set()
    if not bridge_cols:
        # 2026-10-24：侦察未出箭头（漏检/未识别）时，桥体结构在 K 列是**场地固定设施**
        # ——整列按墙处理（路线只走 A/B/C 走廊），箭头识别后 replan 守卫换门。
        # 旧版空 passable ⇒ 桥列凭空消失，早发路线可斜穿 K 列任意格（K8 直穿实测）。
        bridge_cols = {LETTERS.index("K")}
    for col in bridge_cols:
        # 2026-10-15 现场模型修正：桥体其余格全部是结构体（不可踩），
        # 不再设"引道"——引道概念属于旧的"桥洞格南北穿越中线"模型，
        # 实测隧道是东西向穿越桥体（docs/分区规划与感知约束-20261015.md）。
        open_rows = {c.row for c in passable_cells if c.col == col}
        for row in range(ROWS):
            c = CellRef(col, row)
            if row in open_rows:
                bridge_open.add(c)
            else:
                bridge_walls.add(c)
    bridge_walls = frozenset(bridge_walls)
    # 标靶格一律不可踩（物理靶体）；移动靶的轨道两端同样不可踩（轨道本体）。
    target_cells = {t.cell.name: t.cell for t in snapshot.targets}
    track_cells = {c.name: c for t in snapshot.targets if t.track for c in t.track}
    blocked = (
        frozenset(snapshot.obstacles) | bridge_walls
        | frozenset(target_cells.values()) | frozenset(track_cells.values())
    )
    # 桥洞可通行格优先于桥洞墙（同一格不可能既是缺口又是墙）。
    blocked = frozenset(c for c in blocked if c.name not in passable_names)
    # 起点/终点格免疫封锁：车物理上就站在起点，终点是固定方框——
    # 这两格上的"障碍"只能是误检（边缘误检是 documented 顽疾），
    # 信任它会让规划器在自家脚下看到一堵墙。
    blocked = frozenset(c for c in blocked if c != start and c != goal)

    # 中场横向挡板的穿越缺口：
    # 1. 挡板范围外的列（A、B、C）可自由穿越边界；
    # 2. 桥洞列中位于边界附近的可通行格（箭头+引道）可穿越。
    # 若调用方未显式指定 cross_gates，则自动推导。
    if cross_gates is None:
        gates: set[CellRef] = set()
        for col in range(COLS):
            if col not in _BARRIER_COLS:
                for row in (mid_boundary - 1, mid_boundary):
                    gates.add(CellRef(col, row))
        # 桥洞列中边界附近的可通行格
        for c in bridge_open:
            if c.row in (mid_boundary - 1, mid_boundary):
                gates.add(c)
        if gates:
            cross_gates = frozenset(gates)

    # 雷为普通必经点；蓝靶的必经点是其停靠射击格（每靶一组候选）。
    # cache 在本次 plan_route 内共享：blocked/cross_gates 已定型，
    # 等最优顺序枚举与射击位组合会大量重复同一段 Dijkstra。
    cache: dict = {}
    # 不可达诊断上下文：报错时能直接指出封锁点类别与开放桥门
    diag_ctx = {
        "obstacles": {c.name for c in snapshot.obstacles},
        "targets": {t.cell.name for t in snapshot.targets},
        "tracks": {c.name for t in snapshot.targets if t.track for c in t.track},
        "bridge_cols": bridge_cols,
        "open_gates": passable_names,
    }

    base_required: dict[str, CellRef] = {}
    for c in snapshot.mines:
        base_required.setdefault(c.name, c)
    # 可通行桥洞格也是必经点：规则里"桥洞通道协同穿梭"每段 10 分（一区域 5 选 1、
    # 二区域 6 选 1，每区只计一次）——无人机读到的箭头格不穿过去就是白丢分。
    # 此前代码漏了这条（docstring 写了但实现没加），2026-10-15 随机图复盘发现补上。
    # 被实物占用的门格不是必经点（门已失效，见占用过滤）。
    if require_gates:
        for c in passable_cells:
            base_required.setdefault(c.name, c)
    # 发现态靶标（红蓝未辨，is_blue=None）的确认点：距靶 1 格的正交邻格
    # 作为必经点（docs/分区规划与感知约束-20261015 问题 4 两段式）。
    # 车到确认点即近距离复判：蓝→replan 守卫加射击位，红→粘性否决绕行；
    # fire_guard 对未辨靶一律禁火（fail-safe），本机制只负责"保证路过近处"。
    for t in snapshot.targets:
        if t.is_blue is not None:
            continue
        for cand in _shoot_candidates(t.cell, blocked, target_cells):
            base_required.setdefault(cand.name, cand)
            break

    cand_lists: list[list[CellRef]] = []
    cand_faces: list[dict[str, int]] = []   # 与 cand_lists 平行：停靠格 -> 需正对朝向
    for t in snapshot.targets:
        if not t.is_blue:
            continue   # 红面可见（友方）或红靶：不打不停靠
        if t.track is not None:
            cands = _shoot_candidates_moving(t, blocked)
            faces = {s.name: _FACE_OPP[t.blue_face] for s in cands}
        else:
            cands = _shoot_candidates(t.cell, blocked, target_cells)
            faces = {s.name: _face_heading(s, t.cell) for s in cands}
        if cands:
            cand_lists.append(cands)
            cand_faces.append(faces)
        # 无候选 = 该靶被围死无处停靠，跳过（与不可达必经点语义一致）

    # 射击位候选按"起点可达性"预过滤：不可达候选直接移出组合枚举。
    # 否则某组合的候选不可达会被当成"该靶无可停靠"静默跳过，
    # 可能挤掉另一组合中可达候选的路线——白丢整组打击分。
    dist0 = None
    if cand_lists:
        dist0, _, _ = _cached_dijkstra(cache, start, blocked, cross_gates, mid_boundary, _ALL_HEADINGS, turn_45, allow_diagonal,
                     cross_penalty=cross_penalty, tunnel_axes=tunnel_axes, tunnel_req=tunnel_req)
        cand_lists = [
            [c for c in cl if _arrival_state(dist0, c, None)[0] < _INF]
            for cl in cand_lists
        ]
        cand_lists = [cl for cl in cand_lists if cl]

    if not cand_lists:
        route, _ = _plan_core(start, goal, blocked, base_required, cross_gates, mid_boundary, entry0, h_final, turn_45, allow_diagonal, cache,
                              cross_penalty=cross_penalty, tunnel_axes=tunnel_axes, tunnel_req=tunnel_req,
                              diag=diag_ctx)
        return ensure_mine_center_pass(
            route, {c.name for c in snapshot.mines}, blocked,
            mid_boundary=mid_boundary, cross_gates=cross_gates,
            tunnel_names=({_CELLS[ci].name for ci in tunnel_axes}
                          if tunnel_axes else None))

    total = 1
    for cl in cand_lists:
        total *= len(cl)
    if total <= _SHOOT_COMBO_CAP:
        combos: list[tuple[CellRef, ...]] = list(product(*cand_lists))
    else:
        # 退化：逐靶选离起点最近的候选（大场景保护，实际靶数少不会触发）
        combos = [tuple(
            min(cl, key=lambda c: (_arrival_state(dist0, c, None)[0], c.name)) for cl in cand_lists
        )]

    best_route: tuple[str, ...] | None = None
    best_cost = _INF
    best_dup = 1 << 30
    align_degraded: set[str] = set()   # 靶向对齐降级格（跨组合累积，选定后过滤告警）
    for combo in combos:
        req = dict(base_required)
        for c in combo:
            req.setdefault(c.name, c)
        # 该组合停靠格的朝向要求表：对齐到达优先（2026-10-23），对齐不可行的
        # 停靠格退回停靠转身假设并记入 align_degraded
        face_req = {c.name: cf[c.name] for cf in cand_faces for c in combo if c.name in cf}
        try:
            route, cost = _plan_core(start, goal, blocked, req, cross_gates, mid_boundary, entry0, h_final, turn_45, allow_diagonal, cache,
                                     cross_penalty=cross_penalty, tunnel_axes=tunnel_axes, tunnel_req=tunnel_req,
                                     face_req=face_req, diag=diag_ctx, align_degraded=align_degraded)
        except ValueError:
            continue
        dup = len(route) - len(set(route))
        better = cost < best_cost - _EPS or (abs(cost - best_cost) <= _EPS and dup < best_dup)
        if best_route is None or better:
            best_route, best_cost, best_dup = route, cost, dup
    if best_route is None:
        # 所有射击位组合都不可达：给出带封锁点分类的诊断而非裸报错
        dist_all, _, _ = _cached_dijkstra(cache, start, blocked, cross_gates, mid_boundary, _ALL_HEADINGS, turn_45, allow_diagonal,
                                          cross_penalty=cross_penalty, tunnel_axes=tunnel_axes, tunnel_req=tunnel_req)
        raise ValueError(_unreachable_detail(
            start, goal, dist_all, blocked, cross_gates, mid_boundary, diag_ctx))
    degraded_hit = sorted(align_degraded & set(best_route))
    if degraded_hit:
        # 只对**选定路线**上的降级格告警（跨组合尝试的中间结果不刷屏）。
        warnings.warn(
            "靶标停靠格 {} 无法以正对朝向到达（对侧进段被堵），改为停靠转身对齐"
            "（逐渐调整）——开火瞬间车头与靶心存在残余夹角，请确认执行器停转-"
            "爬行可用/云台覆盖足够".format(",".join(degraded_hit)),
            RuntimeWarning, stacklevel=2,
        )
    return ensure_mine_center_pass(
        best_route, {c.name for c in snapshot.mines}, blocked,
        mid_boundary=mid_boundary, cross_gates=cross_gates,
        tunnel_names=({_CELLS[ci].name for ci in tunnel_axes}
                      if tunnel_axes else None))


_ZONE_NAMES = ("西", "东南", "东北")
_CORRIDOR_CROSS = ("A6", "A7", "B6", "B7", "C6", "C7")  # 挡板穿越可用的走廊格（6/7 行界两侧）


def _dist(a: CellRef, b: CellRef) -> int:
    """格间曼哈顿距离（转场启发式用，仅作 tie-break）。"""
    return abs(a.col - b.col) + abs(a.row - b.row)


def _transit_blocked(snapshot: WorldSnapshot) -> frozenset[CellRef]:
    """全局 blocked 集（供中转 Dijkstra 用）：障碍 + 桥洞墙 + 靶格 + 轨道格。

    passable（桥洞隧道）格不封——中转可能途经隧道；起点/终点格免疫。
    与 plan_route 内部 blocked 计算同口径，保证中转代价与实际行驶一致。
    """
    passable_cells = tuple(snapshot.passable)
    passable_names = {c.name for c in passable_cells}
    bridge_cols = {c.col for c in passable_cells}
    if not bridge_cols:
        bridge_cols = {LETTERS.index("K")}  # 2026-10-24：无箭头时 K 列桥体仍按墙（同 plan_route）
    bridge_walls: set[CellRef] = set()
    for col in bridge_cols:
        open_rows = {c.row for c in passable_cells if c.col == col}
        for row in range(ROWS):
            if row not in open_rows:
                bridge_walls.add(CellRef(col, row))
    target_cells = {t.cell for t in snapshot.targets}
    track_cells = {c for t in snapshot.targets if t.track for c in t.track}
    blocked = (frozenset(snapshot.obstacles) | frozenset(bridge_walls)
               | frozenset(target_cells) | frozenset(track_cells))
    return frozenset(c for c in blocked if c.name not in passable_names)


def mission_keep(snapshot: WorldSnapshot, route) -> set[str]:
    """压缩保护集：雷 / 桥门 / 靶体与轨道 / 路线上所有可开火格。

    compress_route 会删掉同向共线的中间格——射击停靠格一旦被删，
    adapt 就看不到它，'*' 标记丢失 = 漏打（2026-10-17 实拍复盘踩中）。
    本函数把"绝不能删的格"集中在一处推导，调用方不再手工拼 KEEP。

    ⚠ 全仓**只能有这一个** keep 推导。曾经的 _required_keep(snapshot) 少了
    轨道格与可开火格，且只认 snapshot 不认 route；car_adapter 修好后
    plan_route_zoned 仍走旧的那个，等于同一个 bug 在分区规划路径上活着。
    """
    keep = {c.name for c in snapshot.mines}
    keep |= {c.name for c in snapshot.passable}
    keep |= {t.cell.name for t in snapshot.targets}
    keep |= {c.name for t in snapshot.targets if t.track for c in t.track}
    for cell in route:
        name = cell.strip().upper()
        if not decide_fire(snapshot, name).denied and blue_targets_hit(snapshot, name):
            keep.add(name)          # 潜在开火格：宁可多保，不可漏保
    return keep


def _derive_cross_gates(snapshot: WorldSnapshot, mid_boundary: int) -> frozenset[CellRef]:
    """推导中场挡板穿越缺口（与 plan_route 内部逻辑同口径）。

    始终包含西区走廊格（A~C 列 6/7 行）——挡板范围外的列可自由穿越；
    桥洞列中边界附近的可通行格也作为缺口。无 passable 时仅返回走廊格，
    保证中转 Dijkstra 与 plan_route 的挡板约束一致。
    """
    gates: set[CellRef] = set()
    for col in range(COLS):
        if col not in _BARRIER_COLS:
            for row in (mid_boundary - 1, mid_boundary):
                gates.add(CellRef(col, row))
    passable_cells = tuple(snapshot.passable)
    for c in passable_cells:
        if c.row in (mid_boundary - 1, mid_boundary):
            gates.add(c)
    return frozenset(gates)


def _pick_transit_dijkstra(
    cur: CellRef,
    cur_zone: str,
    next_z: str | None,
    goal: CellRef,
    mine_z: dict[str, list[CellRef]],
    pas_z: dict[str, list[CellRef]],
    mid_boundary: int,
    blocked: frozenset[CellRef],
    cross_gates: frozenset[CellRef] | None,
    turn_45: float,
    allow_diagonal: bool,
    cross_penalty: float,
    tunnel_axes: frozenset[int] | None,
    tunnel_req: dict[int, int] | None,
) -> CellRef:
    """中转目标格选择：用 Dijkstra 真实路径代价（替代曼哈顿距离）。

    场景：
    - next_z=None → 直接去 goal（最后一区或无后续区）；
    - 东南↔东北 → 枚举西区走廊格，选 cur→走廊→下一区最近必经点 总代价最小者；
    - 涉西转场 → 直奔下一区最近必经点（Dijkstra 代价最小）；
    - 下一区无已知必经点 → 去 goal（等无人机侦察后重规划接手）。

    平局裁决（2026-10-17 步数-姿态平衡）：同代价时先比 (累计转向, 步数)
    ——等时转场里更平直者优先；曼哈顿距离再作末级 tie-break（避免抖动）。
    """
    if next_z is None:
        return goal
    next_req = mine_z.get(next_z, []) + pas_z.get(next_z, [])

    def _cost(a: CellRef, b: CellRef) -> tuple[float, int, int]:
        """a→b 的 (代价, 累计转向 45° 单位, 步数)。

        代价平局时姿态更平直/步数更少者优先（步数-姿态平衡，2026-10-17）；
        不可达返回 (_INF, ...)。
        """
        if a == b:
            return 0.0, 0, 0
        d, _, aux = _cached_dijkstra(None, a, blocked, cross_gates, mid_boundary,
                                _ALL_HEADINGS, turn_45, allow_diagonal,
                                cross_penalty=cross_penalty,
                                tunnel_axes=tunnel_axes, tunnel_req=tunnel_req)
        cost, end = _arrival_state(d, b, None, aux)
        if end is None or cost >= _INF:
            return _INF, _BIG_INT, _BIG_INT
        t, s = aux.get(end, (_BIG_INT, _BIG_INT))
        return cost, t, s

    # 下一区无已知任务：直奔 goal（部分场域下未知区按可通行处理，
    # 无人机随车侦察后 replan 守卫会自动补路线）
    if not next_req:
        return goal

    # 东南↔东北：必须经西区走廊格转场（D7~N7 挡板不能直穿）
    if cur_zone != "西" and next_z != "西":
        best: tuple[tuple, int, str, CellRef] | None = None
        for name in _CORRIDOR_CROSS:
            cc = CellRef(LETTERS.index(name[0]), int(name[1:]) - 1)
            k1 = _cost(cur, cc)
            if k1[0] >= _INF:
                continue
            k2_candidates = [k for k in (_cost(cc, r) for r in next_req) if k[0] < _INF]
            if not k2_candidates:
                continue
            k2 = min(k2_candidates)
            total = (k1[0] + k2[0], k1[1] + k2[1], k1[2] + k2[2])
            key = (total, _dist(cur, cc) + min(_dist(cc, r) for r in next_req), name, cc)
            if best is None or key < best:
                best = key
        if best is not None:
            return best[3]
        # 全部走廊格不可达（极端障碍堵死走廊）：降级直奔 goal
        warnings.warn(
            f"西区走廊格全部不可达，{cur_zone}区→{next_z}区转场降级直奔终点",
            RuntimeWarning, stacklevel=3,
        )
        return goal

    # 涉西转场（C|D 无墙）：直奔下一区 Dijkstra 代价最小的必经点
    best = None
    for r in next_req:
        k = _cost(cur, r)
        if k[0] >= _INF:
            continue
        key = (k, _dist(cur, r), r.name, r)
        if best is None or key < best:
            best = key
    if best is not None:
        return best[3]
    # 下一区所有必经点不可达：直奔 goal
    return goal


def _verify_zone_covered(
    route: tuple[str, ...],
    zone: str,
    mine_z: dict[str, list[CellRef]],
    pas_z: dict[str, list[CellRef]],
) -> None:
    """每区完赛校验：该区所有已知雷/桥洞格必须出现在路线中。

    plan_route 内部已对不可达必经点告警并跳过；此处兜底校验可达点是否
    被 TSP 遗漏（理论不应发生，发生则是 bug）。告警只报一次文本。
    """
    route_set = set(route)
    missing: list[str] = []
    for c in mine_z.get(zone, []):
        if c.name not in route_set:
            missing.append(c.name)
    for c in pas_z.get(zone, []):
        if c.name not in route_set:
            missing.append(c.name)
    if missing:
        warnings.warn(
            f"{zone}区完赛校验：{len(missing)} 个必经点未在路线中 "
            f"（可能不可达已被 plan_route 跳过）: {sorted(missing)}",
            RuntimeWarning, stacklevel=3,
        )


def plan_route_zoned(
    snapshot: WorldSnapshot,
    start: CellRef,
    goal: CellRef,
    zone_order: tuple[str, ...] | None,
    **kwargs,
) -> tuple[str, ...]:
    """分区排序规划：按指定区域顺序逐区执行，再缝合为一条路线。

    语义（docs/分区规划与感知约束-20261015.md）：
    - zone_order=None 时退化为全局 plan_route（向后兼容）；
    - 区域名限 "西"/"东南"/"东北"（2026-10-15 现场确认的三区）；
    - 每区内部仍用 plan_route 求该区最优（含雷/隧道/射击位）；
    - 东南↔东北 之间只经西区走廊格（A6~C7）转场；西区进出自由（C|D 无墙）；
    - 终点所在区自动排到最后执行（车必须停在终点）；

    可靠性硬化（2026-10-16 部分场域适配）：
    - zone_order 未覆盖含必经点的区时**自动追加到末尾**（不抛 ValueError），
      部分场域下无人机只侦察到 1/3 区域也能正常规划；
    - 中转目标格用 **Dijkstra 真实路径代价**选择（替代曼哈顿距离），
      避开障碍/挡板导致的绕弯与折返；
    - 空区（无已知必经点）**跳过不穿越**，避免白跑；
    - 每区规划后做**完赛校验**，必经点遗漏即告警；
    - 下一区无已知任务时直奔 goal，未知区按可通行处理，replan 守卫随车侦察补全。
    """
    if not zone_order:
        return plan_route(snapshot, start, goal, **kwargs)
    mid_boundary = kwargs.get("mid_boundary", _MID_BOUNDARY)
    unknown = set(zone_order) - set(_ZONE_NAMES)
    if unknown:
        raise ValueError(f"zone_order 含未知区域名 {sorted(unknown)}，只支持 {_ZONE_NAMES}")

    turn_45 = kwargs.get("turn_45", 0.75)
    allow_diagonal = kwargs.get("allow_diagonal", True)
    cross_penalty = kwargs.get("crossing_penalty", 0.0)
    tunnel_dirs = kwargs.get("tunnel_dirs", None)

    # 必经点按区分桶（障碍物全局保留——物理墙处处生效）
    mine_z: dict[str, list[CellRef]] = {}
    pas_z: dict[str, list[CellRef]] = {}
    tgt_z: dict[str, list] = {}
    for c in snapshot.mines:
        mine_z.setdefault(_zone_of(c, mid_boundary), []).append(c)
    for c in snapshot.passable:
        pas_z.setdefault(_zone_of(c, mid_boundary), []).append(c)
    for t in snapshot.targets:
        tgt_z.setdefault(_zone_of(t.cell, mid_boundary), []).append(t)

    have_req = set(mine_z) | set(pas_z) | set(tgt_z)

    # 可靠性：zone_order 漏覆盖含任务的区时自动追加（不抛异常）
    # 部分场域下无人机只侦察到 1/3 区域，配置的 zone_order 可能不全
    missing = have_req - set(zone_order)
    if missing:
        warnings.warn(
            f"zone_order 未覆盖区域 {sorted(missing)}，自动追加到末尾执行",
            RuntimeWarning, stacklevel=2,
        )
        zone_order = tuple(zone_order) + tuple(sorted(missing))

    # 只排有任务的区；终点所在区强制排最后（车必须停在终点）
    order = [z for z in zone_order if z in have_req]
    goal_zone = _zone_of(goal, mid_boundary)
    if goal_zone in order and order[-1] != goal_zone:
        order.append(order.pop(order.index(goal_zone)))
        warnings.warn(f"zone_order 已将终点所在区({goal_zone}区)调整到最后执行",
                      RuntimeWarning, stacklevel=2)

    # 无任务区：直奔终点（部分场域下只有 1 区有数据的场景）
    if not order:
        try:
            return plan_route(snapshot, start, goal, **kwargs)
        except ValueError:
            # 终点不可达：返回仅含起点的退化路线（保住节点不崩，等下拍重规划）
            warnings.warn(
                f"无任务区且终点 {goal.name} 不可达，返回起点退化路线",
                RuntimeWarning, stacklevel=2,
            )
            return (start.name,)

    # 中转 Dijkstra 用的全局 blocked / cross_gates / tunnel 约束
    blocked = _transit_blocked(snapshot)
    gates_param = kwargs.get("cross_gates", None)
    cross_gates = gates_param if gates_param is not None else _derive_cross_gates(snapshot, mid_boundary)
    # 隧道约束解析（与 plan_route 同口径）
    tunnel_axes: frozenset[int] | None = None
    tunnel_req: dict[int, int] | None = None
    if tunnel_dirs is None:
        axes = {c.row * COLS + c.col for c in snapshot.passable}
        tunnel_axes = frozenset(axes) if axes else None
    elif tunnel_dirs:
        axes = set()
        tunnel_req = {}
        for name, d in tunnel_dirs.items():
            cn = name.strip().upper()
            dd = str(d).strip().upper()
            cell = CellRef(LETTERS.index(cn[0]), int(cn[1:]) - 1)
            ci = cell.row * COLS + cell.col
            axes.add(ci)
            if dd in ("E", "W"):
                tunnel_req[ci] = 0 if dd == "E" else 4
        tunnel_axes = frozenset(axes) if axes else None

    parts: list[tuple[str, ...]] = []
    cur = start
    for i, z in enumerate(order):
        last = i == len(order) - 1
        next_z = order[i + 1] if not last else None
        sub_goal = goal if (last and z == goal_zone) else _pick_transit_dijkstra(
            cur, z, next_z, goal, mine_z, pas_z, mid_boundary, blocked, cross_gates,
            turn_45, allow_diagonal, cross_penalty, tunnel_axes, tunnel_req,
        )
        sub = WorldSnapshot(
            snapshot.timestamp_ms,
            tuple(mine_z.get(z, [])),
            tuple(pas_z.get(z, [])),
            snapshot.obstacles,
            tuple(tgt_z.get(z, [])),
        )
        try:
            r = plan_route(sub, cur, sub_goal, **kwargs)
        except ValueError:
            # 该区到中转点不可达（障碍堵死）：降级——跳过该区任务直奔终点，
            # 保住完赛（崩溃 = 整局 0 分，跳过区 = 丢该区分数但能完赛）
            warnings.warn(
                f"{z}区规划失败（到 {sub_goal.name} 不可达），"
                f"降级跳过该区任务直奔终点",
                RuntimeWarning, stacklevel=2,
            )
            r = plan_route(
                WorldSnapshot(snapshot.timestamp_ms, (), (), snapshot.obstacles, ()),
                cur, goal, **kwargs,
            )
            parts.append(r if not parts else r[1:])
            cur = CellRef(LETTERS.index(r[-1][0]), int(r[-1][1:]) - 1)
            break   # 已直奔终点，不再处理后续区
        # 每区完赛校验：该区雷/桥洞必经点必须出现在路线中
        _verify_zone_covered(r, z, mine_z, pas_z)
        parts.append(r if not parts else r[1:])
        cur = CellRef(LETTERS.index(r[-1][0]), int(r[-1][1:]) - 1)  # 路线格名 -> CellRef

    # 终点不在任何已执行的区（终点区无任务）→ 补最后一程
    if cur != goal:
        try:
            tail = plan_route(
                WorldSnapshot(snapshot.timestamp_ms, (), (), snapshot.obstacles, ()),
                cur, goal, **kwargs,
            )
        except ValueError:
            # 终点不可达：返回已规划的部分（至少完成了已扫区）
            warnings.warn(
                f"终点 {goal.name} 不可达，返回已规划部分路线",
                RuntimeWarning, stacklevel=2,
            )
            full = tuple(c for part in parts for c in part)
            return compress_route(full, mission_keep(snapshot, full))
        parts.append(tail[1:])
    full = tuple(c for part in parts for c in part)
    return compress_route(full, mission_keep(snapshot, full))


def _norm_dir(a: tuple[int, int], b: tuple[int, int]) -> tuple[int, int]:
    """a→b 位移归一化到最简整数方向，用于共线判断。"""
    dx, dy = b[0] - a[0], b[1] - a[1]
    g = gcd(abs(dx), abs(dy)) or 1
    return (dx // g, dy // g)


def compress_route(
    route: tuple[str, ...],
    keep: frozenset[str] | set[str] = frozenset(),
) -> tuple[str, ...]:
    """共线压缩：删掉"进入方向 == 离开方向"的中间格。

    车端按"相邻格号之间连直线段"行驶（官方协议"直线连接、可对角"），
    同向共线的中间格删掉后物理轨迹完全不变，命令 token 更短。
    永不删除：首尾格、所有拐点（方向变化处）、keep 中的格
    （雷/桥洞/靶等必经点与射击格必须保留为显式途经点）。
    """
    if len(route) < 3:
        return tuple(route)
    keep = frozenset(k.strip().upper() for k in keep)

    def pos(name: str) -> tuple[int, int]:
        return LETTERS.index(name[0]), int(name[1:]) - 1

    out: list[str] = [route[0]]
    for i in range(1, len(route) - 1):
        cur = route[i]
        if cur not in keep and _norm_dir(pos(route[i - 1]), pos(cur)) == _norm_dir(pos(cur), pos(route[i + 1])):
            continue  # 同向共线中间格：直线段自然经过其格心，无需单列
        out.append(cur)
    out.append(route[-1])
    return tuple(out)
