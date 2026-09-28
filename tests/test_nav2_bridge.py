import math

import pytest

from ag_chassis.nav2_bridge import FrameAlign, cell_center_xy, cell_to_map_pose


def test_cell_center_a1():
    x, y = cell_center_xy("A1", 0.35)
    assert (x, y) == pytest.approx((0.175, 0.175))


def test_cell_center_k14():
    x, y = cell_center_xy("K14", 0.35)
    assert x == pytest.approx(10 * 0.35 + 0.175)
    assert y == pytest.approx(13 * 0.35 + 0.175)


def test_cell_center_case_insensitive():
    assert cell_center_xy("c3") == cell_center_xy("C3")


def test_bad_cell_raises():
    for bad in ("", "3", "A", "!!"):
        with pytest.raises(ValueError):
            cell_center_xy(bad)


def test_identity_align_passthrough():
    align = FrameAlign(cell_size=0.35)
    x, y = align.cell_to_map("A1")
    assert (x, y) == pytest.approx((0.175, 0.175))


def test_origin_offset():
    align = FrameAlign(origin_x=2.0, origin_y=1.0, cell_size=0.35)
    x, y = align.cell_to_map("A1")
    assert (x, y) == pytest.approx((2.175, 1.175))


def test_rotation_90_degrees():
    """场地相对 map 转了 90 度：栅格 x 轴应映射到 map 的 y 轴。"""
    align = FrameAlign(yaw_rad=math.pi / 2, cell_size=0.35)
    x, y = align.to_map(1.0, 0.0)
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(1.0, abs=1e-9)


def test_scale_applies():
    align = FrameAlign(scale=2.0, cell_size=0.35)
    assert align.to_map(1.0, 1.0) == pytest.approx((2.0, 2.0))


def test_cell_to_map_pose_returns_yaw():
    align = FrameAlign(cell_size=0.35)
    x, y, yaw = cell_to_map_pose("B2", align, yaw=0.5)
    assert yaw == 0.5
    assert (x, y) == pytest.approx(align.cell_to_map("B2"))


def test_roundtrip_of_two_cells_preserves_distance():
    """相邻两格中心距离应恒为 cell_size（刚体变换不改变距离）。"""
    align = FrameAlign(origin_x=3.0, origin_y=-1.0, yaw_rad=0.7, scale=1.0, cell_size=0.35)
    ax, ay = align.cell_to_map("C3")
    bx, by = align.cell_to_map("C4")
    assert math.hypot(bx - ax, by - ay) == pytest.approx(0.35, abs=1e-9)
