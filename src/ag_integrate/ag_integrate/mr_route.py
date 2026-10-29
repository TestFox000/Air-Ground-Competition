# mr_route —— 由 mission_router.py 拆出（公开路线 API：plan_route/plan_route_zoned/mission_keep/compress_route）。与原文件逐行一致（AST 校验过）。
from __future__ import annotations

import heapq
import warnings
from itertools import count, product
from math import gcd

from .field_frame import CellRef, ROWS, COLS, LETTERS
from .fire_guard import blue_targets_hit, decide_fire
from .world_model import WorldSnapshot
from .mr_board import _BARRIER_COLS, _CELLS, _MID_BOUNDARY, _build_solid_barriers, _zone_of
from .mr_nav import _arrival_state, _cached_dijkstra, _norm_dir, _plan_core, _unreachable_detail, ensure_mine_center_pass
from .mr_params import _ALL_HEADINGS, _BIG_INT, _EPS, _GEAR_SWITCH, _HEADING_IDX, _INF, _REV_MULT
from .mr_shooting import _FACE_OPP, _SHOOT_COMBO_CAP, _face_heading, _los_clear, _shoot_candidates, _shoot_candidates_moving, _standoff_serves

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
    tunnel_corridor: int = 1,
    require_gates: bool = True,
    turn90_guard: bool = False,
    solid_barriers: bool = False,
    reverse: bool = False,
    rev_mult: float = _REV_MULT,
    gear_switch: float = _GEAR_SWITCH,
    heading_prop: bool = False,
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

    # 2026-10-25 综合规划增强项：实体挡板探测表（solid_barriers=True 时启用）。
    # 挡板侵入边界两侧格各 1/4 格（87.5mm），桥洞开口列豁免；转角可用空间
    # 在搜索内按物理判据（need/avail）删边，K7 贴板死锁/M7 1.2mm 余量这类
    # "格占用模型看不见"的几何债由此出清。turn90_guard/reverse/heading_prop
    # 的语义见 _dijkstra/_held_karp_orders_heading docstring。
    solid_map, solid_key = (
        _build_solid_barriers(mid_boundary, bridge_open) if solid_barriers else (None, None))
    # tunnel_corridor：过洞直线走廊长度（两侧各 N 格禁换行，默认 1=历史行为；
    # 2026-10-29 用户需求——桥洞有真实体积，corridor=2 时过 K2 强制 J2→K2→L2→M2
    # 全直行，转向最早在洞口外第 3 格）。走廊越长隧道越难用：走廊格被桶占/误检
    # 堵死时该隧道整条弃用（不可达告警口径），现场按需回调。
    if tunnel_corridor < 1:
        raise ValueError(f"tunnel_corridor 须 >=1，收到 {tunnel_corridor}")
    _aenh = dict(turn90_guard=turn90_guard, solids=solid_map, solid_key=solid_key,
                 reverse=reverse, rev_mult=rev_mult, gear_switch=gear_switch,
                 tunnel_corridor=tunnel_corridor)

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
            # 按轨端分组必经（2026-10-26 吸收车端 route_check 第④层语义：
            # 每个轨端都要有停站在 0.78~1.02 m 窗内，单格可同时服务两端，
            # 如 D5 距 E7=√5 格、距 F7=√8 格；只到访"任一环带格"会漏掉
            # 另一端——官方六层校验"靶覆盖 n/2"❌ 的根因）。
            # 组内优选无视线遮挡候选（_los_clear）；全遮挡才降级用全组
            # （best-effort：可见端打不到也别整靶放弃）。
            los_blocked = frozenset(
                c for c in blocked if c.name not in target_cells
                and c.name not in track_cells)
            end_groups = []
            for rail in t.track:
                g = [c for c in cands if _standoff_serves(c, rail)]
                g_clear = [c for c in g
                           if _los_clear(c, rail, los_blocked)]
                g = g_clear or g
                if g:
                    end_groups.append(g)
            for g in end_groups:
                cand_lists.append(g)
                cand_faces.append({s.name: _FACE_OPP[t.blue_face] for s in g})
        else:
            cands = _shoot_candidates(t.cell, blocked, target_cells)
            faces = {s.name: _face_heading(s, t.cell) for s in cands}
            if cands:
                cand_lists.append(cands)
                cand_faces.append(faces)
        # 无候选 = 该靶被围死无处停靠，跳过（与不可达必经点语义一致）
        # 部分轨端无组 = 该端放弃、其余端仍要停（best-effort 同上）。

    # 射击位候选按"起点可达性"预过滤：不可达候选直接移出组合枚举。
    # 否则某组合的候选不可达会被当成"该靶无可停靠"静默跳过，
    # 可能挤掉另一组合中可达候选的路线——白丢整组打击分。
    dist0 = None
    if cand_lists:
        dist0, _, _ = _cached_dijkstra(cache, start, blocked, cross_gates, mid_boundary, _ALL_HEADINGS, turn_45, allow_diagonal,
                     cross_penalty=cross_penalty, tunnel_axes=tunnel_axes, tunnel_req=tunnel_req, **_aenh)
        cand_lists = [
            [c for c in cl if _arrival_state(dist0, c, None)[0] < _INF]
            for cl in cand_lists
        ]
        cand_lists = [cl for cl in cand_lists if cl]

    if not cand_lists:
        route, _ = _plan_core(start, goal, blocked, base_required, cross_gates, mid_boundary, entry0, h_final, turn_45, allow_diagonal, cache,
                              cross_penalty=cross_penalty, tunnel_axes=tunnel_axes, tunnel_req=tunnel_req,
                              diag=diag_ctx, heading_prop=heading_prop, **_aenh)
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
                                     face_req=face_req, diag=diag_ctx, align_degraded=align_degraded,
                                     heading_prop=heading_prop, **_aenh)
        except ValueError:
            continue
        dup = len(route) - len(set(route))
        better = cost < best_cost - _EPS or (abs(cost - best_cost) <= _EPS and dup < best_dup)
        if best_route is None or better:
            best_route, best_cost, best_dup = route, cost, dup
    if best_route is None:
        # 所有射击位组合都不可达：给出带封锁点分类的诊断而非裸报错
        dist_all, _, _ = _cached_dijkstra(cache, start, blocked, cross_gates, mid_boundary, _ALL_HEADINGS, turn_45, allow_diagonal,
                                          cross_penalty=cross_penalty, tunnel_axes=tunnel_axes, tunnel_req=tunnel_req, **_aenh)
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
