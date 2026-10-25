"""端到端纯管线：世界态势 -> 出发门控 -> 车端命令（无 ROS、无 socket）。"""

from __future__ import annotations

import warnings
from dataclasses import dataclass, replace

from .car_adapter import CarCommands, adapt
from .car_commands import RoutePlan, route_command
from .field_frame import LETTERS, CellRef
from .mission_router import _MID_BOUNDARY, plan_route, plan_route_zoned
from .world_model import WorldSnapshot

# 门控 reason 是黑匣子事件流契约（route.blocked 的 reason 字段），字面量
# 必须保持稳定，语义对应 ag_core.blackbox.REASON.MIN_MINES / FACE_UNKNOWN。
# REASON.MIN_MINES 现值 "min_mines" 与本契约不一致：按"事件流字面量不变"
# 原则以本包现值为准，ag_core 常量由下一波 ag_core 修复对齐（本包不动它）。
REASON_NOT_ENOUGH_MINES = "not_enough_mines"
REASON_FACE_UNKNOWN = "face_unknown"


def _audit_solid(snapshot: WorldSnapshot, mid_boundary: int, solid_barriers: bool):
    """体检用实体挡板表（{格名: 8 向 mm}）；solid_barriers 关闭时 None（旧口径）。"""
    if not solid_barriers:
        return None
    from .mission_router import _build_solid_barriers, _CELLS

    smap, _ = _build_solid_barriers(mid_boundary, snapshot.passable)
    return {_CELLS[ci].name: row for ci, row in smap.items()} if smap else None


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


def _post_plan_audit(snapshot: WorldSnapshot, ordered, solid=None) -> None:
    """规划后只读体检（V1019 移植，2026-10-20）：转角物理可行性 + 现场规则。

    纯告警不改路线、异常不影响出车：
    - 转角判据（转弯内侧侧向投影，ok/tight/pivot/forbid）见 turn_feasibility_v2；
    - 规则校验（雷只在 A~I / 雷数≤8 / 隧道直穿）见 field_rules；
    - 方案与实测：docs/路径规划优化-转角物理分级-20261020.md。

    solid（2026-10-25）：实体挡板探测表（{格名: 8 向 mm}）——solid_barriers
    开启时由 build_mission 传入，体检与搜索内守卫看同一片挡板几何，
    "守卫按 A 优化、体检按 B 打分"的口径分裂不再发生。
    """
    try:
        from .turn_feasibility_v2 import check_route, report_lines, summarize

        chk = check_route(ordered, [c.name for c in snapshot.obstacles], solid)
        rep = summarize(chk)
        if rep["pivot_needed"]:
            warnings.warn(
                f"转角体检：{rep['turns']} 个转角中需掉头/不可行 {rep['pivot_needed']} 个"
                f"（135°+ {rep['forbid135']}，最小余量 {rep['min_margin_mm']:.0f}mm）"
                "——按明细减速/多点掉头：\n  " + "\n  ".join(report_lines(chk, 4)),
                RuntimeWarning, stacklevel=3)
    except Exception:                      # 体检永不影响规划结果
        pass
    try:
        from .field_rules import validate

        rv = validate(list(ordered),
                      [c.name for c in snapshot.mines],
                      [c.name for c in snapshot.passable])
        if not rv["ok"]:
            issues = list(rv["tunnel"]["issues"])
            issues += [f"雷越界 {m}：规则限定雷只在 A~I 列（疑误检）"
                       for m in rv["mines"]["out_of_region"]]
            if not rv["mines_count"]["ok"]:
                issues.append(f"雷 {rv['mines_count']['count']} 颗 > 上限 8（必有误检，建议人工复核）")
            warnings.warn("规则校验：" + "；".join(issues), RuntimeWarning, stacklevel=3)
    except Exception:                      # 规则校验失败也不影响出车
        pass


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
    # 2026-10-25 综合规划增强项（透传 plan_route；语义见 mission_router._dijkstra）：
    turn90_guard: bool = False,
    solid_barriers: bool = False,
    reverse: bool = False,
    rev_mult: float = 1.5,
    gear_switch: float = 0.5,
    heading_prop: bool = False,
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
    _motion = dict(turn90_guard=turn90_guard, solid_barriers=solid_barriers,
                   reverse=reverse, rev_mult=rev_mult, gear_switch=gear_switch,
                   heading_prop=heading_prop)
    ordered = plan_route_zoned(
        snapshot, start, goal, zone_order, cross_gates=gates, mid_boundary=mid_boundary,
        start_heading=start_heading, final_heading=final_heading,
        crossing_penalty=crossing_penalty, tunnel_dirs=tunnel_dirs,
        turn_45=turn_45, allow_diagonal=allow_diagonal, **_motion,
    )
    commands = adapt(snapshot, ordered, start_cell, start_heading, final_heading)
    _post_plan_audit(snapshot, ordered, _audit_solid(snapshot, mid_boundary, solid_barriers))

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
            motion=_motion,
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
            turn_45=turn_45, allow_diagonal=allow_diagonal, **_motion,
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
    allow_diagonal, params, soft_cap_s, drop_policy, backend, motion,
):
    """U1 联合求解路径：解出保留集后按保留集重规划。

    返回 None = 不可用/不占优，调用方退回现行启发式（降级契约）。
    得分口径：踩雷 +5/颗、射击位命中期望 +5/位（与 route_time 计时模型一致）。

    三条不变量（各自都曾被违反，见下方实测数字）：
      1. leg 代价只测"移动"，不含计分绕行 —— require_gates=False；
      2. 执行 == 决策 == 记录 —— 被弃的射击位必须真的从命令串里去掉 '*'；
      3. 返回前用**实路由**复核预算，模型乐观就收紧重解，仍超则告警。
    """
    from .ortools_solver import (
        STATUS_FEASIBLE, STATUS_OPTIMAL, VisitTask, solve_visit_plan,
    )
    from .route_time import estimate_seconds

    # 点对点 leg 用**真实路由**估计（纯场地：去掉雷/靶必经点，只留障碍与桥门），
    # 而不是直线距离——挡板/桥墙切分的场地上直线会严重低估绕行耗时，导致
    # 求解器按错误模型弃任务（实测教训：弃 5 雷后实路由仍超预算）。
    #
    # ⚠ 但桥门只能保留"桥墙开口"语义，不能同时当必经点（require_gates=False）。
    # 门格是计分项（"桥洞通道协同穿梭"每段 10 分），正式路线必须穿门；
    # 可一旦让每段 leg 也强制穿门，同一对格子的代价会被撑到 5×：
    #   B9->A2（同在无挡板的 A/B 列，直线 8 格）
    #     require_gates=True  : 37 格 / 57.1s（东行到 K9 折返、穿挡板、再到 K4 折返）
    #     require_gates=False :  8 格 / 11.2s
    # 9 段 leg 累加后整条任务的模型估值 404s，而真实连续路线只要 71.5s（5.7×）。
    # 真实 recon 夹具（docs/成员交接/yu-binghuan/20261018-uav-video-recon）
    # 实测通胀 3.3×~5.0×。后果按 drop_policy 分两种，都很糟：
    #   shooting_only（默认）：雷不可舍 -> 模型地板恒超预算 -> 恒 infeasible
    #                          -> U1 从不生效，静默退回启发式
    #   any_task           ：按通胀代价弃任务 -> cap=68s 时弃 7/8 雷 + 射击位，
    #                          实踩雷 1 颗；而真实规划器只需弃 2 雷（J8/A2）
    #                          即可 65.1s 进预算、保住 6 雷 30 分 —— 白丢 25 分
    field_snap = replace(snapshot, mines=(), targets=())
    _leg_cache: dict[tuple[str, str], float] = {}

    def leg_s(a: str, b: str) -> float:
        key = (a, b)
        if key in _leg_cache:
            return _leg_cache[key]
        try:
            route = plan_route(field_snap, _parse_name(a), _parse_name(b),
                               require_gates=False)
        except ValueError:
            _leg_cache[key] = float("inf")
        else:
            # plan_route 返回格名序列（字符串），直接喂给 estimate_seconds
            _leg_cache[key] = estimate_seconds(tuple(route), params=params).seconds
        return _leg_cache[key]

    tasks = [VisitTask(m.name, "mine", 5.0, params.mine_slow_s)
             for m in snapshot.mines]
    tasks += [VisitTask(s, "shoot", 5.0, params.shoot_stop_s)
              for s in commands.shoot_cells]

    budget = soft_cap_s
    for attempt in (0, 1):
        try:
            plan = solve_visit_plan(
                start_cell, goal_cell, tasks, leg_s=leg_s,
                time_budget_s=budget, drop_policy=drop_policy,
                backend=backend,
            )
        except Exception:
            return None
        if plan.status not in (STATUS_OPTIMAL, STATUS_FEASIBLE):
            return None

        keep = set(plan.order)
        mines2 = tuple(m for m in snapshot.mines if m.name in keep)
        shoot_kept = any(s in keep for s in commands.shoot_cells)
        # 射击位全被舍：靶格物理仍在、仍不可踩，只是不停靠开火（与启发式同语义）
        targets2 = snapshot.targets if shoot_kept else tuple(
            replace(t, is_blue=False) for t in snapshot.targets)
        snapshot2 = replace(snapshot, mines=mines2, targets=targets2)
        try:
            ordered3 = plan_route_zoned(
                snapshot2, _parse_name(start_cell), _parse_name(goal_cell),
                zone_order, cross_gates=gates, mid_boundary=mid_boundary,
                start_heading=start_heading, final_heading=final_heading,
                crossing_penalty=crossing_penalty, tunnel_dirs=tunnel_dirs,
                turn_45=turn_45, allow_diagonal=allow_diagonal, **motion,
            )
        except ValueError:
            return None
        commands3 = adapt(snapshot2, ordered3, start_cell, start_heading, final_heading)

        # 部分射击位被舍：adapt 只看 snapshot 几何，会把求解器已弃的站重新标出 '*'。
        # 执行必须与决策一致 —— 否则 dropped_tasks 记的是没发生的事，
        # 黑匣子里"已弃 X 站"而车其实停了 X 站（时间也就没省下来）。
        kept_shoot = tuple(c for c in commands3.shoot_cells if c in keep)
        if len(kept_shoot) != len(commands3.shoot_cells):
            commands3 = replace(
                commands3,
                route=route_command(RoutePlan(tuple(ordered3),
                                              frozenset(kept_shoot), final_heading)),
                shoot_cells=kept_shoot,
            )

        est3 = estimate_seconds(ordered3, shoot_cells=commands3.shoot_cells,
                                mine_cells={m.name for m in mines2}, params=params)

        # VisitTask 以**格名**为身份：同一格既是雷又是射击位时（蓝靶的停靠格
        # 正好压着雷，8雷2靶的场地上约 10% 概率），两种任务是两个节点、
        # 却塌缩成 keep 里同一个名字。求解器可以"弃射击留排雷"，而车反正要
        # 去那格排雷、停靠开火照旧 —— 省不下来的 shoot_stop_s 会让预算收敛不了。
        # 记录必须只写**真的没去**的项，否则黑匣子说省了、实路由没省。
        ineffective = tuple(c for c in plan.dropped if c in keep)
        if ineffective:
            warnings.warn(
                f"联合求解器弃了 {list(ineffective)}，但这些格另有保留任务"
                f"（同格既是雷又是射击位），车仍会停靠——省不出对应时间；"
                f"任务身份按格名塌缩，需要按 (格,任务类型) 区分才能真正弃掉",
                RuntimeWarning, stacklevel=2,
            )
        dropped_eff = tuple(c for c in plan.dropped if c not in keep)

        if est3.seconds <= soft_cap_s or attempt:
            break
        # leg-sum 模型对真实连续路线系统性略乐观（真实 recon 夹具实测
        # 模型/真实 = 0.91~0.96×）：模型说进预算了、实路由没进。
        # 按实测偏差收紧预算再解一次，而不是把超预算的结果当成功返回。
        # 此分支内必有 ratio > 1（plan.est_seconds <= budget 且 est3 > soft_cap_s）。
        ratio = est3.seconds / plan.est_seconds if plan.est_seconds > 0 else 1.0
        budget = soft_cap_s / max(ratio, 1.0)
        warnings.warn(
            f"联合求解器时间模型乐观 {ratio:.2f}×：模型 {plan.est_seconds:.0f}s "
            f"实路由 {est3.seconds:.0f}s 超软预算 {soft_cap_s:.0f}s，"
            f"收紧预算至 {budget:.0f}s 重解",
            RuntimeWarning, stacklevel=2,
        )

    if est3.seconds >= est_seconds:
        return None  # 求解器口径占优但实路由不占优：退回启发式
    if est3.seconds > soft_cap_s:
        # 收紧一轮后仍超预算：返回它（确实比原方案好），但必须喊出来。
        # 否则 integrate_node 只在 shed_shooting / >180s 两种情况下出声，
        # soft_cap < est <= 180 且未砍射击位时全程静默。
        warnings.warn(
            f"联合求解器重解后仍超软预算：{est3.seconds:.0f}s > {soft_cap_s:.0f}s"
            f"（已弃 {len(dropped_eff)} 项：{list(dropped_eff)}）",
            RuntimeWarning, stacklevel=2,
        )
    return replace(
        commands3,
        est_seconds=est3.seconds,
        shed_shooting=bool(commands.shoot_cells) and any(
            s not in keep for s in commands.shoot_cells),
        dropped_tasks=dropped_eff,
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
