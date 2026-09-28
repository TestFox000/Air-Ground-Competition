import numpy as np
import pytest

from ag_core.grid_map import CellID, FieldGrid
from ag_core.homography import (
    field_corners,
    homography_from_correspondences,
    reprojection_error,
)


def _sim_transform(scale=0.003, tx=0.2, ty=0.15):
    """一个已知的「像素 -> 米」相似变换，用于构造合成数据。"""
    return np.array([[scale, 0.0, tx], [0.0, scale, ty], [0.0, 0.0, 1.0]])


def test_recovers_known_transform():
    H_true = _sim_transform()
    src = np.array([[100.0, 200.0], [900.0, 180.0], [880.0, 700.0], [120.0, 690.0]])
    dst = np.hstack([src, np.ones((4, 1))]) @ H_true.T
    dst = dst[:, :2] / (np.hstack([src, np.ones((4, 1))]) @ H_true.T)[:, 2:3]

    H = homography_from_correspondences(src, dst)
    assert reprojection_error(H, src, dst) < 1e-6


def test_four_corners_map_to_grid_extent():
    grid = FieldGrid(cols=14, rows=11, cell_size=0.35)
    corners = field_corners(grid.cols, grid.rows, grid.cell_size)
    # 假设相机与场地严格相似：A1->像素(0,0)，N11->像素(1400,1100)
    src = np.array([[0.0, 0.0], [1400.0, 0.0], [1400.0, 1100.0], [0.0, 1100.0]])
    H = homography_from_correspondences(src, corners)
    grid.set_homography(H)

    assert grid.pixel_to_cell(*src[0]).name == "A1"
    # N11 是角点，落在边界上，取内部一点更稳妥
    assert grid.pixel_to_cell(1390.0, 1090.0).name == "N11"
    assert grid.pixel_to_cell(700.0, 550.0) is not None


def test_cell_center_roundtrip():
    """格中心 -> 米制 -> 像素 -> 反算回格号，应闭合。"""
    grid = FieldGrid()
    src = np.array([[0.0, 0.0], [1300.0, 0.0], [1300.0, 1000.0], [0.0, 1000.0]])
    H = homography_from_correspondences(src, field_corners(grid.cols, grid.rows, grid.cell_size))
    grid.set_homography(H)
    H_inv = np.linalg.inv(H)

    for name in ("A1", "D7", "N11", "B2"):
        cid = CellID.parse(name)
        x, y = grid.cell_center(cid)
        q = H_inv @ np.array([x, y, 1.0])
        u, v = q[0] / q[2], q[1] / q[2]
        assert grid.pixel_to_cell(u, v) == cid, name


def test_requires_at_least_four_points():
    with pytest.raises(ValueError):
        homography_from_correspondences([[0, 0], [1, 1], [2, 2]], [[0, 0], [1, 1], [2, 2]])


def test_mismatched_sizes_raises():
    with pytest.raises(ValueError):
        homography_from_correspondences(
            [[0, 0], [1, 1], [2, 2], [3, 3]], [[0, 0], [1, 1], [2, 2]]
        )


def test_pixel_to_field_without_homography_raises():
    grid = FieldGrid()
    with pytest.raises(RuntimeError):
        grid.pixel_to_field(10.0, 10.0)


def test_reprojection_error_detects_bad_point():
    H_true = _sim_transform()
    src = np.array([[0.0, 0.0], [1000.0, 0.0], [1000.0, 1300.0], [0.0, 1300.0]])
    dst = (np.hstack([src, np.ones((4, 1))]) @ H_true.T)[:, :2]
    assert reprojection_error(H_true, src, dst) < 1e-9
    dst_bad = dst.copy()
    dst_bad[2] += 0.5  # 第三个角点标错 0.5 m
    assert reprojection_error(H_true, src, dst_bad) > 0.1
