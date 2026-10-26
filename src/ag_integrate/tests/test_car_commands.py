# -*- coding: utf-8 -*-
"""car_commands :PIVOT 声明护栏（2026-10-26）：官方 route_check 第⑥层口径。

车头转角 ≥135° 才声明（虚报=❌ 漏报=⚠️），与 '*' 叠加为 'F2*:PIVOT'，
按下标记不按格名（同名格重访的直行段不连坐）。
"""
from __future__ import annotations

from ag_integrate.car_commands import RoutePlan, route_command


def test_right_angle_no_pivot():
    assert route_command(RoutePlan(cells=("A1", "A2", "B2", "B3"))) == "A1 A2 B2 B3"


def test_180_reversal_declared():
    out = route_command(RoutePlan(cells=("A1", "A2", "A1", "B1")))
    assert out == "A1 A2:PIVOT A1 B1"


def test_exact_135_declared_borderline():
    # A1→B2 对角、B2→A2 反向水平：夹角恰 135.0°，≥ 阈值须声明
    out = route_command(RoutePlan(cells=("A1", "B2", "A2")))
    assert out == "A1 B2:PIVOT A2"


def test_under_135_not_declared():
    # A1→C2 (2,1)、C2→A3 (-2,1)：夹角 ≈126.87° < 135°，不声明
    out = route_command(RoutePlan(cells=("A1", "C2", "A3")))
    assert out == "A1 C2 A3"


def test_star_and_pivot_stack():
    out = route_command(RoutePlan(cells=("A1", "F2", "A1"),
                                  shoot_at=frozenset({"F2"})))
    assert out == "A1 F2*:PIVOT A1"


def test_revisit_straight_leg_not_polluted():
    # J4 第一次经过是 180° 折返（I3→J4→I3）须声明；I3 处同样折返也声明；
    # J4 第二次经过（I3→J4→K5 对角直行，转角 0°）**不得**被别处的 J4 折返
    # 连坐标记——按下标记而非按格名的意义就在这。
    out = route_command(RoutePlan(cells=("I3", "J4", "I3", "J4", "K5")))
    assert out == "I3 J4:PIVOT I3:PIVOT J4 K5"


def test_final_heading_appends():
    out = route_command(RoutePlan(cells=("A1", "B1"), final_heading="x"))
    assert out == "A1 B1,-y"   # 本机 x -> 车端 -y（坐标架翻译不变）


def test_diagonal_180_declared():
    # 对角原路折返：A1→B2→A1 夹角恰 180°
    out = route_command(RoutePlan(cells=("A1", "B2", "A1")))
    assert out == "A1 B2:PIVOT A1"


def test_two_cell_route_no_pivot_no_crash():
    assert route_command(RoutePlan(cells=("A1", "A2"))) == "A1 A2"


def test_pivot_with_final_heading_combo():
    # F2: 180° 折返+射击；A1: F2→A1(-1,-1) 转 A1→B1(1,0) 恰 135° 也须声明
    out = route_command(RoutePlan(cells=("A1", "F2", "A1", "B1"),
                                  shoot_at=frozenset({"F2"}),
                                  final_heading="y"))
    assert out == "A1 F2*:PIVOT A1:PIVOT B1,x"
