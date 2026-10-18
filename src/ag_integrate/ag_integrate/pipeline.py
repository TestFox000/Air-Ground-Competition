"""端到端纯管线：世界态势 -> 出发门控 -> 车端命令（无 ROS、无 socket）。"""

from __future__ import annotations

from dataclasses import dataclass, replace

from .car_adapter import CarCommands, adapt
from .field_frame import LETTERS, CellRef
from .mission_router import _MID_BOUNDARY, plan_route, plan_route_zoned
from .world_model import WorldSnapshot

# 门控 reason 是黑匣子事件流契约（route.blocked 的 reason 字段），字面量
# 必须保持稳定，语义对应 ag_core.blackbox.REASON.MIN_MINES / FACE_UNKNOWN。
# REASON.MIN_MINES 现值 "min_mines" 与本契约不一致：按"事件流字面量不变"
# 原则以本包现值为准，ag_core 常量由下一波 ag_core 修复对齐（本包不动它）。
REASON_NOT_ENOUGH_MINES = "not_enough_mines"
REASON_FACE_UNKNOWN = "face_unknown"


@dataclass(frozen=True)
class Gate:
    ok: bool
    reason: str = ""


def evaluate_gate(
    snapshot: WorldSnapshot,
    min_mines: int,
    require_face: bool,
) -> Gate:
    """情报是否足以发车。与"有没有敌方"无关：红面也要能去排雷/回终点。"""
    if len(snapshot.mines) < min_mines:
        return Gate(False, REASON_NOT_ENOUGH_MINES)
    if require_face and not snapshot.targets:
        return Gate(False, REASON_FACE_UNKNOWN)
    return Gate(True)


def build_mission(
    snapshot: WorldSnapshot,
    start_cell: str,
    goal_cell: str,
    start_heading: str,
    final_heading: str | None,
    cross_gates: tuple[str, ...] = (),
    mid_boundary: int = _MID_BOUNDARY,
    *,
    target_track: tuple[str, str] | None = None,
    target_blue_face: str | None = None,
    time_params: "RouteTimeParams | None" = None,
    soft_cap_s: float | None = None,
    crossing_penalty: float = 0.0,
    tunnel_dirs: dict[str, str] | None = None,
    zone_order: tuple[str, ...] | None = None,
    turn_45: float = 0.75,
    allow_diagonal: bool = True,
    # 以下均为关键字参数（原有关键字区）：
    use_ortools_solver: bool = False,
    ortools_drop_policy: str = "shooting_only",
    ortools_backend: str = "auto",
) -> CarCommands:
    """态势 -> reset + route 两条命令。

    target_track/target_blue_face：移动靶（全场仅一个）的轨道两端格号与
    蓝面朝向。这是裁判系统每局固定的先验，无人机只识别瞬时哪面可见，
    因此由本节点参数注入，而不是从识别帧推导。

    turn_45/allow_diagonal：路径代价平衡（步数 vs 姿态）——对角步按 √2
    计价天然"距离更短走对角线"；turn_45 是每 45° 转向的代价当量，
    0.75=阿克曼底盘物理标定，0=纯最短路+最平直平局，越大越姿态优先。

    时间预算门控（soft_cap_s 传入才启用，官方硬顶 180s、同分比用时）：
    规划出解后用 route_time.estimate_seconds 折算执行秒数（写入
    commands.est_seconds）。超软预算时按序尝试两级策略：
      1) use_ortools_solver=True：联合求解器（ortools_solver.solve_visit_plan）
         一步解出"保留哪些任务 + 访问顺序"（得分字典序优先、耗时次之）。
         drop_policy="shooting_only"（默认）保持"少排雷是人的决策"血线
         哲学、只是细化整砍；"any_task" 允许舍雷（V3 升级口径）。
         求解失败/未安装自动退回 2)。
      2) 现行启发式：有射击位时整砍射击位重规划（打击是唯一带 −5 负分
         风险的可砍任务），并置 commands.shed_shooting。纯排雷链超预算
         不自动砍雷。
    """
    start = _parse_name(start_cell)
    goal = _parse_name(goal_cell)
    gates = frozenset(_parse_name(name) for name in cross_gates) or None
    if target_track is not None:
        snapshot = _inject_track(snapshot, target_track, target_blue_face)
    ordered = plan_route_zoned(
        snapshot, start, goal, zone_order, cross_gates=gates, mid_boundary=mid_boundary,
        start_heading=start_heading, final_heading=final_heading,
        crossing_penalty=crossing_penalty, tunnel_dirs=tunnel_dirs,
        turn_45=turn_45, allow_diagonal=allow_diagonal,
    )
    commands = adapt(snapshot, ordered, start_cell, start_heading, final_heading)

    if soft_cap_s is None or soft_cap_s <= 0:
        return commands

    from .route_time import RouteTimeParams, estimate_seconds
    params = time_params or RouteTimeParams()
    mine_names = {c.name for c in snapshot.mines}
    est = estimate_seconds(ordered, shoot_cells=commands.shoot_cells,
                           mine_cells=mine_names, params=params)
    commands = replace(commands, est_seconds=est.seconds)
    if est.seconds <= soft_cap_s:
        return commands

    # 一级策略：联合求解器（U1）——排序 + 取舍一步最优，失败自动退回启发式
    if use_ortools_solver:
        solved = _solve_with_joint_solver(
            snapshot, commands, est.seconds,
            start_cell=start_cell, goal_cell=goal_cell,
            start_heading=start_heading, final_heading=final_heading,
            gates=gates, mid_boundary=mid_boundary,
            crossing_penalty=crossing_penalty, tunnel_dirs=tunnel_dirs,
            zone_order=zone_order, turn_45=turn_45,
            allow_diagonal=allow_diagonal,
            params=params, soft_cap_s=soft_cap_s,
            drop_policy=ortools_drop_policy, backend=ortools_backend,
        )
        if solved is not None:
            return solved

    # 二级策略（现行启发式兜底）：整砍射击位重规划
    if not commands.shoot_cells:
        return commands

    # 超软预算：砍射击位重规划（靶格物理仍在、仍不可踩，只是不停靠开火）
    shed_snapshot = replace(snapshot, targets=tuple(
        replace(t, is_blue=False) for t in snapshot.targets))
    try:
        ordered2 = plan_route_zoned(
            shed_snapshot, start, goal, zone_order, cross_gates=gates, mid_boundary=mid_boundary,
            start_heading=start_heading, final_heading=final_heading,
            crossing_penalty=crossing_penalty, tunnel_dirs=tunnel_dirs,
            turn_45=turn_45, allow_diagonal=allow_diagonal,
        )
    except ValueError:
        return commands   # 同一地图可达性不变，理论上不会到这里；防御性保底
    commands2 = adapt(shed_snapshot, ordered2, start_cell, start_heading, final_heading)
    est2 = estimate_seconds(ordered2, shoot_cells=commands2.shoot_cells,
                            mine_cells=mine_names, params=params)
    if est2.seconds < est.seconds:
        return replace(commands2, est_seconds=est2.seconds, shed_shooting=True)
    return commands


def _solve_with_joint_solver(
    snapshot, commands, est_seconds, *, start_cell, goal_cell,
    start_heading, final_heading, gates, mid_boundary,
    crossing_penalty, tunnel_dirs, zone_order, turn_45,
    allow_diagonal, params, soft_cap_s, drop_policy, backend,
):
    """U1 联合求解路径：解出保留集后按保留集重规划。

    返回 None = 不可用/不占优，调用方退回现行启发式（降级契约）。
    得分口径：踩雷 +5/颗、射击位命中期望 +5/位（与 route_time 计时模型一致）。
    """
    from .ortools_solver import (
        STATUS_FEASIBLE, STATUS_OPTIMAL, VisitTask, solve_visit_plan,
    )
    from .route_time import estimate_seconds

    # 点对点 leg 用**真实路由**估计（纯场地：去掉雷/靶必经点，只留障碍与桥门），
    # 而不是直线距离——挡板/桥墙切分的场地上直线会严重低估绕行耗时，导致
    # 求解器按错误模型弃任务（实测教训：弃 5 雷后实路由仍超预算）。
    field_snap = replace(snapshot, mines=(), targets=())

    def leg_s(a: str, b: str) -> float:
        try:
            route = plan_route(field_snap, _parse_name(a), _parse_name(b))
        except ValueError:
            return float("inf")
        # plan_route 返回格名序列（字符串），直接喂给 estimate_seconds
        return estimate_seconds(tuple(route), params=params).seconds

    tasks = [VisitTask(m.name, "mine", 5.0, params.mine_slow_s)
             for m in snapshot.mines]
    tasks += [VisitTask(s, "shoot", 5.0, params.shoot_stop_s)
              for s in commands.shoot_cells]
    try:
        plan = solve_visit_plan(
            start_cell, goal_cell, tasks, leg_s=leg_s,
            time_budget_s=soft_cap_s, drop_policy=drop_policy,
            backend=backend,
        )
    except Exception:
        return None
    if plan.status not in (STATUS_OPTIMAL, STATUS_FEASIBLE):
        return None

    keep = set(plan.order)
    mines2 = tuple(m for m in snapshot.mines if m.name in keep)
    shoot_kept = any(s in keep for s in commands.shoot_cells)
    # 射击位被舍：靶格物理仍在、仍不可踩，只是不停靠开火（与启发式同语义）
    targets2 = snapshot.targets if shoot_kept else tuple(
        replace(t, is_blue=False) for t in snapshot.targets)
    snapshot2 = replace(snapshot, mines=mines2, targets=targets2)
    try:
        ordered3 = plan_route_zoned(
            snapshot2, _parse_name(start_cell), _parse_name(goal_cell),
            zone_order, cross_gates=gates, mid_boundary=mid_boundary,
            start_heading=start_heading, final_heading=final_heading,
            crossing_penalty=crossing_penalty, tunnel_dirs=tunnel_dirs,
            turn_45=turn_45, allow_diagonal=allow_diagonal,
        )
    except ValueError:
        return None
    commands3 = adapt(snapshot2, ordered3, start_cell, start_heading, final_heading)
    est3 = estimate_seconds(ordered3, shoot_cells=commands3.shoot_cells,
                            mine_cells={m.name for m in mines2}, params=params)
    if est3.seconds >= est_seconds:
        return None  # 求解器口径占优但实路由不占优：退回启发式
    return replace(
        commands3,
        est_seconds=est3.seconds,
        shed_shooting=(not shoot_kept and bool(commands.shoot_cells)),
        dropped_tasks=plan.dropped,
    )


def _inject_track(
    snapshot: WorldSnapshot,
    track: tuple[str, str],
    blue_face: str | None,
) -> WorldSnapshot:
    """给识别到的移动靶补轨道先验（前提：全场仅一个移动靶，轨道两端由
    裁判系统地图固定，无人机识别只有瞬时落点与哪面可见）。

    只对落在轨道上的靶注入（落点格在轨道线段上，即距线段 <0.5 格）；
    固定红/蓝靶不注入——否则固定红靶会被当成移动靶触发全局禁火，
    固定蓝靶的停靠射击位会被轨道几何算错。
    """
    a, b = _parse_name(track[0]), _parse_name(track[1])
    targets = tuple(
        replace(t, track=(a, b), blue_face=blue_face) if _on_track(t.cell, a, b) else t
        for t in snapshot.targets
    )
    return replace(snapshot, targets=targets)


def _on_track(cell: CellRef, a: CellRef, b: CellRef) -> bool:
    """落点格是否"在轨道上"：格心到轨道线段 ab 的距离 < 0.5 格。"""
    dx, dy = b.col - a.col, b.row - a.row
    if dx == 0 and dy == 0:
        return cell == a
    # 格心到线段 ab 的垂距（格单位），t 截到 [0,1] 得线段最近点
    t = ((cell.col - a.col) * dx + (cell.row - a.row) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    dc = cell.col - (a.col + t * dx)
    dr = cell.row - (a.row + t * dy)
    return dc * dc + dr * dr < 0.25


def _parse_name(name: str) -> CellRef:
    """格号 -> CellRef；非法（空串/字母越界/行号越界）抛 ValueError。"""
    s = name.strip().upper()
    if len(s) < 2 or s[0] not in LETTERS or not s[1:].isdigit():
        raise ValueError(f"非法格号 {name!r}：应为 字母A~N + 数字1~11")
    cell = CellRef(LETTERS.index(s[0]), int(s[1:]) - 1)
    cell.validate()
    return cell
