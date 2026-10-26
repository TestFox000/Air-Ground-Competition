# -*- coding: utf-8 -*-
"""track_debug 纯几何核心测试：路线展开 / 横向误差 / 航点到达 / 倒车判别 / 分段速度。"""
import importlib.util
import math
import pathlib
import sys

_TOOL = pathlib.Path(__file__).resolve().parents[3] / "tools" / "track_debug.py"
_spec = importlib.util.spec_from_file_location("track_debug", _TOOL)
mod = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = mod
_spec.loader.exec_module(mod)


def _traj_along(pts, dt=0.2):
    """沿规划点列匀速采样的"完美轨迹"。"""
    traj = []
    t = 0.0
    for p in pts:
        traj.append((t, p[1], p[2], 0.0))
        t += dt
    return traj


def test_route_points_expands_and_marks_explicit():
    pts = mod.route_points("A1 C3* N5,y")
    assert [p[0] for p in pts] == ["A1", "B2", "C3", "N5"]
    assert [p[3] for p in pts] == [True, False, True, True]   # B2 是隐式途经格


def test_perfect_traj_zero_error_all_arrived():
    pts = mod.route_points("A1 C3 A5")
    mod._plan_pts = pts
    traj = _traj_along(pts)
    lat = mod.lateral_errors(traj)
    assert max(r[4] for r in lat) < 1e-9
    ap = mod.waypoint_approach(traj, pts)
    assert len(ap) == 3 and all(a["arrived"] for a in ap)


def test_offset_traj_fails_arrival():
    pts = mod.route_points("A1 A5")           # 纵向直线段
    mod._plan_pts = pts
    d = 0.30                                   # 偏 30cm（> 到点阈值 15.7cm）
    traj = [(i * 0.5, pts[0][1] + d, pts[0][2], 0.0) for i in range(10)]
    lat = mod.lateral_errors(traj)
    assert all(abs(r[4] - d) < 1e-6 for r in lat)
    ap = mod.waypoint_approach(traj, pts)
    assert not any(a["arrived"] for a in ap)   # 横向 30cm 永远不"到达"


def test_reverse_episodes_detects_backtrack():
    # 车头朝东（yaw=0）先前进 0.3m 再沿原路倒回 0.3m
    traj = [(0.0, 0.0, 0.0, 0.0)]
    for k in range(1, 7):
        traj.append((k * 0.5, 0.05 * k, 0.0, 0.0))
    for k in range(1, 7):
        traj.append((3.0 + k * 0.5, 0.3 - 0.05 * k, 0.0, 0.0))
    # 前进 6 步 5cm 到 0.30，再倒回 6 步 5cm 到 0.00
    eps = mod.reverse_episodes(traj)
    assert len(eps) == 1
    assert abs(eps[0]["dist"] - 0.30) < 1e-6

    fwd_only = [(k * 0.5, 0.05 * k, 0.0, 0.0) for k in range(10)]
    assert mod.reverse_episodes(fwd_only) == []


def test_segment_speeds_one_cell_per_second():
    pts = mod.route_points("A1 A2 A3")
    mod._plan_pts = pts
    traj = [(0.0, pts[0][1], pts[0][2], 0.0),
            (1.0, pts[1][1], pts[1][2], 0.0),
            (2.0, pts[2][1], pts[2][2], 0.0)]
    ap = mod.waypoint_approach(traj, pts)
    speeds = mod.segment_speeds(traj, pts, ap)
    assert len(speeds) == 2
    assert all(abs(s["speed_mps"] - 0.35) < 0.01 for s in speeds)
    assert all(not s["diag"] for s in speeds)
