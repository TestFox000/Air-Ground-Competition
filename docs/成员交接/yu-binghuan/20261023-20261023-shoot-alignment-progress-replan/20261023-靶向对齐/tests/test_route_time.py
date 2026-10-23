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
    # N10 -> N8：正交两格 = 0.7m，2 条段
    p = RouteTimeParams(cruise_mps=0.5, accel_overhead_s=1.0)
    b = estimate_seconds(("N10", "N9", "N8"), params=p)
    assert b.distance_m == 2 * CELL_SIZE
    assert b.n_segments == 2
    assert b.seconds == (0.7 / 0.5) + 2 * 1.0


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
    objects = [_obj(m, ObjectType.MINE) for m in mines]
    objects += [_obj(CellRef(10, 3), ObjectType.ARROW),    # K4 桥洞
                _obj(CellRef(10, 8), ObjectType.ARROW)]    # K9 桥洞
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
    objects = [_obj(m, ObjectType.MINE) for m in mines]
    objects += [_obj(CellRef(10, 4), ObjectType.ARROW),
                _obj(CellRef(10, 6), ObjectType.ARROW)]
    objects += [_obj(CellRef(2, 6), ObjectType.TARGET_BLUE)]
    snap = _snap(objects)
    t0 = time.monotonic()
    route = plan_route(snap, CellRef(13, 9), CellRef(13, 10))
    dt = time.monotonic() - t0
    assert route[0] == "N10" and route[-1] == "N11"
    assert dt < 3.0, f"最坏场景规划 {dt:.1f}s——性能退化（P0 阈值 3s）"
