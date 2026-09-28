"""车端路线字符串测试：逐条对齐 path_executor 的实际语法。"""

import pytest

from ag_car.route_builder import (
    CarGrid,
    RouteError,
    build_route,
    cell_to_car_xy,
    reset_command,
    route_summary,
)


# ---------------------------------------------------------------- 坐标换算
def test_car_xy_matches_path_executor():
    """与车端 block_to_meters 的实测值逐字对齐（这些数字来自源码注释）。"""
    g = CarGrid()
    assert g.to_car_xy("N10") == pytest.approx((0.525, 0.175))    # 起点，block(1.5, 0.5)
    assert g.to_car_xy("A1") == pytest.approx((3.675, 4.725))     # row1->3.675, colA->4.725
    assert g.to_car_xy("N11") == pytest.approx((0.175, 0.175))    # 原点格
    assert cell_to_car_xy("K4") == pytest.approx(g.to_car_xy("K4"))


def test_car_xy_axes_are_swapped_vs_fieldgrid():
    """车端 x 沿数字行、y 沿字母列；ag_core.FieldGrid 是反的，不能混用。"""
    from ag_core.grid_map import CellID, FieldGrid

    fg = FieldGrid()
    cx, cy = fg.cell_center(CellID.parse("A1"))
    car_x, car_y = cell_to_car_xy("A1")
    assert (cx, cy) == pytest.approx((0.175, 0.175))              # 列方向在前
    assert (car_x, car_y) == pytest.approx((3.675, 4.725))        # 行方向在前
    assert car_x != cx and car_y != cy


def test_grid_extent_is_14x11():
    g = CarGrid()
    assert (g.cols, g.rows) == (14, 11)
    assert g.name(0, 0) == "A1"
    assert g.name(10, 13) == "N11"


# ---------------------------------------------------------------- 格号校验
@pytest.mark.parametrize("bad", ["K14", "N12", "Z9", "D0", "", "12", "AB"])
def test_invalid_cells_are_rejected(bad):
    with pytest.raises(RouteError):
        CarGrid().parse(bad)


def test_row_overflow_message_mentions_real_layout():
    with pytest.raises(RouteError) as e:
        CarGrid().parse("K14")
    assert "14" in str(e.value) and "11" in str(e.value)


# ---------------------------------------------------------------- 路线拼装
def test_basic_route():
    assert build_route(["G8", "A10", "D2", "N11"]) == "G8 A10 D2 N11"


def test_shoot_stop_marked_with_star():
    assert build_route(["G8", "D2", "N11"], shoot_at=["D2"]) == "G8 D2* N11"


def test_final_heading_is_glued_to_last_token():
    """车端用 rsplit(',', 1) 取朝向，所以逗号前不能有空格。"""
    cmd = build_route(["G8", "N11"], final_heading="y")
    assert cmd == "G8 N11,y"
    assert " ," not in cmd


def test_bad_final_heading_rejected():
    with pytest.raises(RouteError):
        build_route(["G8"], final_heading="z")


def test_consecutive_duplicates_collapsed():
    assert build_route(["D2", "D2", "G8", "G8"]) == "D2 G8"


def test_shoot_flag_on_deduped_cell_still_applies():
    """'D2 D2*' 在车端等价于 'D2*'：同一个格第二次出现只是补上打击标记。"""
    assert build_route(["G8", "D2", "D2"], shoot_at=["D2"]) == "G8 D2*"


def test_shoot_flag_outside_route_is_ignored():
    assert build_route(["G8", "N11"], shoot_at=["Z9", "A1"]) == "G8 N11"


def test_empty_route_rejected():
    with pytest.raises(RouteError):
        build_route([])
    with pytest.raises(RouteError):
        build_route(["", "  "])


def test_illegal_cell_aborts_whole_route():
    """车端会因一个非法 token 丢掉整条路线，所以必须整体失败而不是跳过。"""
    with pytest.raises(RouteError):
        build_route(["G8", "K14", "N11"])


def test_star_suffix_tolerated_in_input():
    assert build_route(["D2*", "N11"]) == "D2 N11"
    assert build_route(["D2*", "N11"], shoot_at=["D2"]) == "D2* N11"


# ---------------------------------------------------------------- reset
def test_reset_command():
    assert reset_command("N10", "y") == "reset:N10,y"
    assert reset_command("n10", "Y") == "reset:N10,y"


@pytest.mark.parametrize("heading", ["z", "", None, "xy"])
def test_reset_rejects_bad_heading(heading):
    with pytest.raises(RouteError):
        reset_command("N10", heading)


# ---------------------------------------------------------------- 一览
def test_route_summary_reports_invalid_and_bbox():
    s = route_summary(["N10", "K14", "D2*", "N11"], shoot_at=["D2"])
    assert s["invalid"] == ["K14"]
    assert s["cells"] == ["N10", "D2", "N11"]      # '*' 已归一化掉
    assert s["shoot_at"] == ["D2"]
    bbox = s["bbox_m"]
    assert len(bbox) == 4 and bbox[0] <= bbox[2] and bbox[1] <= bbox[3]


def test_route_summary_marks_shoot_points():
    s = route_summary(["G8", "D2", "N11"], shoot_at=["D2"])
    assert s["shoot_at"] == ["D2"]
    assert "*" in str(s["route"])
