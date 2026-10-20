"""route_time 路线时间预算器测试。"""

import math

from ag_integrate.field_frame import CELL_SIZE, CellRef, cell_to_world
from ag_integrate.intel_protocol import IntelFrame, IntelObject, ObjectType
from ag_integrate.mission_router import compress_route, plan_route
from ag_integrate.route_time import (
    MATCH_LIMIT_S,
    SOFT_CAP_S,
    RouteTimeParams,
    estimate_seconds,
)
from ag_integrate.world_model import WorldModel


def _obj(cell, t):
    x, y = cell_to_world(cell)
    return IntelObject(t, x, y, 0.35, 0.35)


def _snap(objects):
    return WorldModel().update(IntelFrame(1, tuple(objects)))


def test_straight_line_distance_and_overheads():
    # N10 -> N9 -> N8：同列连续三格 = 0.7m 里程、**一条**直线段。
    # 旧断言写 n_segments == 2（按格数算），把 accel_overhead_s 变成"每格附加"，
    # 与它自己 docstring 里"每条直线段"的定义矛盾。
    p = RouteTimeParams(cruise_mps=0.5, accel_overhead_s=1.0)
    b = estimate_seconds(("N10", "N9", "N8"), params=p)
    assert b.distance_m == 2 * CELL_SIZE
    assert b.n_segments == 1
    assert b.seconds == (0.7 / 0.5) + 1 * 1.0


def test_turn_starts_a_new_segment():
    """转向必须另算一段：那才是真正需要减速再加速的地方。"""
    p = RouteTimeParams(cruise_mps=1.0, accel_overhead_s=1.0)
    # N10 -> N9 -> M9：先向南一格再向西北转向，两段
    b = estimate_seconds(("N10", "N9", "M9"), params=p)
    assert b.n_segments == 2


def test_u_turn_along_same_line_counts_as_new_segment():
    """同线折返不是"继续直行"：车得停稳再起步，必须另计一段。

    只用叉积判方向会把 (0,-1) 后的 (0,+1) 当成同一段（cross=0）。
    """
    p = RouteTimeParams(cruise_mps=1.0, accel_overhead_s=1.0)
    b = estimate_seconds(("N10", "N9", "N10"), params=p)
    assert b.n_segments == 2


def test_budget_is_invariant_to_route_encoding():
    """同一条物理路线，逐格喂和压缩喂必须给出**完全相同**的预算。

    这是本模块的核心契约：pipeline.py 用 plan_route 的逐格序列，
    tests/test_full_match_route_within_soft_cap 用 compress_route 的压缩序列。
    旧实现下两者相差 13.6s（59 格全场路线 120.2s vs 106.6s，里程同为 23.05m），
    等于生产门限比测试门限严 13s —— 幻影开销足以把一条真实 158s 的路线判成
    超 165s 软顶，触发不必要的弃靶/弃雷（每弃一项 −5 分）。
    """
    p = RouteTimeParams(cruise_mps=0.5, accel_overhead_s=0.4)
    per_cell = ("N10", "N9", "N8", "N7", "M7", "M8", "M9")
    compressed = ("N10", "N7", "M7", "M9")     # 同一条折线的拐点表达
    b1 = estimate_seconds(per_cell, params=p)
    b2 = estimate_seconds(compressed, params=p)
    assert b1.distance_m == b2.distance_m, "两种表达的里程必须一致"
    assert b1.n_segments == b2.n_segments == 3, (
        f"段数应同为 3（南下/东转/北上），实得 {b1.n_segments} vs {b2.n_segments}")
    assert b1.seconds == b2.seconds, (
        f"预算随表达方式漂移：逐格 {b1.seconds:.2f}s vs 压缩 {b2.seconds:.2f}s")


def test_repeated_cell_is_charged_per_pass():
    """同一格被路线经过两次，减速/停靠就该计两次（旧实现用 set 只算一次）。"""
    p = RouteTimeParams(cruise_mps=1.0, accel_overhead_s=0.0,
                        mine_slow_s=0.5, shoot_stop_s=3.0)
    b = estimate_seconds(("A1", "A2", "A1", "A2"),
                         mine_cells={"A2"}, shoot_cells={"A1"}, params=p)
    assert b.n_mines == 2 and b.n_shoots == 2


def test_diagonal_distance_is_sqrt2():
    p = RouteTimeParams(cruise_mps=1.0, accel_overhead_s=0.0)
    b = estimate_seconds(("N10", "M9"), params=p)   # 对角一格
    assert b.distance_m == CELL_SIZE * math.sqrt(2)
    assert b.seconds == b.distance_m


def test_mine_and_shoot_surcharges():
    p = RouteTimeParams(cruise_mps=1.0, accel_overhead_s=0.0,
                        mine_slow_s=0.5, shoot_stop_s=3.0)
    b = estimate_seconds(
        ("A1", "A2", "A3"),
        mine_cells={"A2"},
        shoot_cells={"A3"},
        params=p,
    )
    assert b.n_mines == 1 and b.n_shoots == 1
    assert b.seconds == (2 * CELL_SIZE) + 0.5 + 3.0


def test_command_suffixes_are_tolerated():
    # 车端命令串的 "*" 射击标记与 ",x" 终点朝向后缀不影响解析
    b1 = estimate_seconds(("N10", "N9", "N8"))
    b2 = estimate_seconds(("N10", "N9*", "N8,x"))
    assert b1.distance_m == b2.distance_m and b1.seconds == b2.seconds


def test_budget_flags():
    assert not estimate_seconds(("A1", "A2")).over_soft_cap
    slow = RouteTimeParams(cruise_mps=0.001)
    b = estimate_seconds(("A1", "N11"), params=slow)
    assert b.over_soft_cap and b.over_match_limit
    assert SOFT_CAP_S < MATCH_LIMIT_S


def test_full_match_route_within_soft_cap():
    """代表性全局任务（8 雷跨两场 + K4/K9 桥洞 + 蓝靶）在默认参数下 < 软预算。"""
    mines = [CellRef(*c) for c in
             ((0, 1), (3, 10), (6, 0), (1, 8), (12, 2), (13, 5), (9, 7), (5, 9))]
    objects = [_obj(m, ObjectType.PAPER) for m in mines]
    objects += [_obj(CellRef(10, 3), ObjectType.PAPER),    # K4 桥洞
                _obj(CellRef(10, 8), ObjectType.PAPER)]    # K9 桥洞
    objects += [_obj(CellRef(3, 1), ObjectType.TARGET_BLUE)]  # D2 敌靶
    snap = _snap(objects)
    route = plan_route(snap, CellRef(13, 9), CellRef(13, 10))  # N10 -> N11
    compressed = compress_route(route, keep={m.name for m in mines})
    b = estimate_seconds(
        compressed,
        mine_cells={m.name for m in mines},
        shoot_cells=set(),  # 射击位由 plan 决定，这里仅粗估
    )
    assert b.seconds < SOFT_CAP_S, (
        f"全局任务预算 {b.seconds:.0f}s 超软上限 {SOFT_CAP_S}s——"
        f"里程 {b.distance_m:.1f}m / {b.n_segments} 段，需提速或压缩任务"
    )


def test_worst_case_planning_speed():
    """P0 回归守卫：最坏密度场景（8雷+双桥洞+蓝靶）单次全局规划 < 3s。

    2026-10-14 沙箱审计发现：等最优顺序枚举(≤256)×逐段两次 Dijkstra，
    单局规划 33.7s；ARM 车端放大到分钟级，直接吃掉 180s 计时。
    修复（整数态 Dijkstra + 全场图共享缓存 + 拼接上限 64）后 x86 实测 ~1.2s。
    """
    import time

    mines = [CellRef(*c) for c in
             ((7, 5), (0, 5), (13, 7), (2, 9), (13, 4), (0, 8), (0, 2), (13, 3))]
    objects = [_obj(m, ObjectType.PAPER) for m in mines]
    objects += [_obj(CellRef(10, 4), ObjectType.PAPER),
                _obj(CellRef(10, 6), ObjectType.PAPER)]
    objects += [_obj(CellRef(2, 6), ObjectType.TARGET_BLUE)]
    snap = _snap(objects)
    t0 = time.monotonic()
    route = plan_route(snap, CellRef(13, 9), CellRef(13, 10))
    dt = time.monotonic() - t0
    assert route[0] == "N10" and route[-1] == "N11"
    assert dt < 3.0, f"最坏场景规划 {dt:.1f}s——性能退化（P0 阈值 3s）"
