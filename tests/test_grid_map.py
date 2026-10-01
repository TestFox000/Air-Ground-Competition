import math

import numpy as np
import pytest

from ag_core.grid_map import CellID, FieldGrid


def test_cell_name_roundtrip():
    for name in ("A1", "D11", "K4", "K9", "B7", "F7", "E2"):
        cid = CellID.parse(name)
        assert cid.name == name


def test_cell_parse_values():
    cid = CellID.parse("D11")
    assert cid.row == 10
    assert cid.col == 3


def test_cell_center_meters():
    g = FieldGrid()
    x, y = g.cell_center(CellID.parse("A1"))
    assert pytest.approx(x, abs=1e-9) == 0.175
    assert pytest.approx(y, abs=1e-9) == 0.175
    x, y = g.cell_center(CellID.parse("D11"))
    assert pytest.approx(x, abs=1e-9) == 3.5 * 0.35
    assert pytest.approx(y, abs=1e-9) == 10.5 * 0.35


def test_point_to_cell_inverse():
    g = FieldGrid()
    cid = CellID.parse("F7")
    x, y = g.cell_center(cid)
    assert g.cell_of_point(x, y).name == "F7"


def test_field_size_matches_rule():
    g = FieldGrid()
    # 字母列沿 5 m 边（14 列 x 0.35 = 4.9 m），数字行沿 4 m 边（11 行 x 0.35 = 3.85 m）
    assert 4.5 <= g.width_m <= 5.0      # 5 m 边（A~N）
    assert 3.5 <= g.length_m <= 4.0     # 4 m 边（1~11）


def test_grid_extent_matches_car():
    """与车端 path_executor 的常量保持一致：14 列 A~N、11 行 1~11。"""
    g = FieldGrid()
    assert (g.cols, g.rows) == (14, 11)
    assert CellID.parse("N11").name == "N11"
    assert g.contains(CellID.parse("N11"))
    assert not g.contains(CellID.parse("K14"))   # 旧推断里的"终点"根本不在场上


def test_bfs_shortest_path():
    g = FieldGrid()
    p = g.bfs_path(CellID.parse("A1"), CellID.parse("N11"))
    assert p is not None
    assert len(p) == 13 + 10 + 1  # 13 列 + 10 行 步数，再加起点本身
    assert p[0].name == "A1"
    assert p[-1].name == "N11"


def test_blocked_cells_force_detour():
    g = FieldGrid()
    # 在第 5 行整行设障碍，只留最右一列通过
    for c in range(g.cols - 1):
        g.set_blocked(CellID(4, c))
    p = g.bfs_path(CellID.parse("A1"), CellID.parse("N11"))
    assert p is not None
    assert CellID(4, g.cols - 1) in p


def test_unreachable_when_fully_blocked():
    g = FieldGrid()
    for c in range(g.cols):
        g.set_blocked(CellID(4, c))
    assert g.bfs_path(CellID.parse("A1"), CellID.parse("N11")) is None


def test_blocked_names_roundtrip():
    g = FieldGrid()
    g.apply_blocked_names(["B2", "C3", "D4"])
    assert sorted(g.blocked_cells()) == ["B2", "C3", "D4"]


def test_homography_pixel_to_cell():
    g = FieldGrid()
    # 构造一个 100 px = 1 m、原点在图像左上 的线性单应
    H = np.array([[0.01, 0.0, 0.0], [0.0, 0.01, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    g.set_homography(H)
    cid = g.pixel_to_cell(360.0, 360.0)   # (3.6 m, 3.6 m)，避开整除边界
    assert cid is not None
    assert cid.name == "K11"              # col=10(K), row=10 -> 11

    # 超出场地范围时应返回 None
    assert g.pixel_to_cell(360.0, 700.0) is None


def test_path_length_counts_diagonal_steps():
    """对角步是 √2 格，不能一律按 1 格算（否则时间预算低估最多 29%）。"""
    g = FieldGrid()
    diag = g.bfs_path(CellID.parse("A1"), CellID.parse("B2"), diagonal=True)
    assert g.path_length_m(diag) == pytest.approx(g.cell_size * math.sqrt(2))
    straight = g.bfs_path(CellID.parse("A1"), CellID.parse("A2"))
    assert g.path_length_m(straight) == pytest.approx(g.cell_size)
