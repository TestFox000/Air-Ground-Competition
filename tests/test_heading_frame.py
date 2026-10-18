# -*- coding: utf-8 -*-
"""朝向 token 的跨坐标架一致性（P0：两架相差 90°，透传会让车绕圆）。

本机（ag_integrate / dzuav）与车端（dznav / 官方 path_executor）各自定义了一套
"格号 -> 米"坐标架，且 **x/y 轴指向的物理方向不同**。'x'/'y' 朝向 token 是两套
坐标架共用的字面量，直接透传就会让车端把车头信念整体转 90°。

本文件不写死"应该是多少"，而是从两个包各自的 cell->world 函数**重新推导**
物理方向，再断言 car_commands._CAR_HEADING 与推导结果一致。任一侧改了坐标架
（换原点、换轴向、改 block_to_meters），这里当场红。
"""
from __future__ import annotations

import math

import pytest

from ag_integrate import car_commands
from ag_integrate.field_frame import CellRef, LETTERS, cell_to_world
from ag_integrate.mission_router import _DIRS, _HEADING_IDX
from dznav.contract import HEADING_YAWS, RE_HEADING, block_to_meters

# 取一个内部格作基准，保证 8 邻域都落在场内
_BASE = CellRef(LETTERS.index("G"), 5)          # G6
_NEIGHBOURS = tuple(_DIRS)                        # ((Δcol, Δrow), ...) 8 个


def _uav_xy(col: int, row: int) -> tuple[float, float]:
    return cell_to_world(CellRef(col, row))


def _car_xy(col: int, row: int) -> tuple[float, float]:
    return block_to_meters(f"{LETTERS[col]}{row + 1}")


def _neighbour_angles(to_xy) -> dict[tuple[int, int], float]:
    """某坐标架下，8 个邻格方向各自的角度（弧度）。"""
    x0, y0 = to_xy(_BASE.col, _BASE.row)
    out = {}
    for dc, dr in _NEIGHBOURS:
        x1, y1 = to_xy(_BASE.col + dc, _BASE.row + dr)
        out[(dc, dr)] = math.atan2(y1 - y0, x1 - x0)
    return out


_UAV_ANGLES = _neighbour_angles(_uav_xy)
_CAR_ANGLES = _neighbour_angles(_car_xy)


def _grid_dir(theta: float, angles: dict[tuple[int, int], float]) -> tuple[int, int]:
    """把一个朝向角吸附到最近的邻格方向，得到与坐标架无关的 (Δcol, Δrow)。"""
    best, best_d = None, None
    for d, a in angles.items():
        diff = abs((theta - a + math.pi) % (2 * math.pi) - math.pi)
        if best_d is None or diff < best_d:
            best, best_d = d, diff
    assert best_d < math.pi / 8, f"{theta} 离最近的 45° 网格方向太远（{best_d}）"
    return best


def _derived_car_heading() -> dict[str, str]:
    """从两架的几何重新推导 本机 token -> 车端 token 的映射。"""
    out = {}
    for token, idx in _HEADING_IDX.items():
        physical = _grid_dir(_UAV_ANGLES[_DIRS[idx]], _UAV_ANGLES)
        for car_token, car_theta in HEADING_YAWS.items():
            if _grid_dir(car_theta, _CAR_ANGLES) == physical:
                out[token] = car_token
                break
        else:                                    # pragma: no cover - 断言失败路径
            pytest.fail(f"车端词表里没有 token 对应物理方向 {physical}")
    return out


# ---- 两架确实不是同一套坐标（本文件存在的前提）----

def test_frames_are_rotated_by_minus_90_degrees():
    """车端 +x ∥ 本机 +y、车端 +y ∥ 本机 −x，即 θ_car = θ_uav − π/2。"""
    for d, uav_theta in _UAV_ANGLES.items():
        car_theta = _CAR_ANGLES[d]
        delta = (car_theta - uav_theta + math.pi) % (2 * math.pi) - math.pi
        assert delta == pytest.approx(-math.pi / 2, abs=1e-9), (
            f"邻格方向 {d}: 本机 {math.degrees(uav_theta):.1f}° vs "
            f"车端 {math.degrees(car_theta):.1f}°，差 {math.degrees(delta):.1f}°")


def test_same_token_means_different_physical_direction():
    """字面量相同的 token 在两架下指向不同物理方向——透传必然错 90°。

    这条是"为什么需要翻译"的实锤：如果哪天两架对齐了，本测试会红，
    那时应该删掉翻译层而不是留着它把对的改错。
    """
    for token in ("x", "y"):
        uav_dir = _grid_dir(_UAV_ANGLES[_DIRS[_HEADING_IDX[token]]], _UAV_ANGLES)
        car_dir = _grid_dir(HEADING_YAWS[token], _CAR_ANGLES)
        assert uav_dir != car_dir, (
            f"token {token!r} 在两架下已指向同一物理方向 {uav_dir}，"
            "翻译层应随之删除")


# ---- 翻译表与推导一致 ----

def test_car_heading_table_matches_derivation():
    assert car_commands._CAR_HEADING == _derived_car_heading()


def test_car_heading_table_is_a_bijection():
    table = car_commands._CAR_HEADING
    assert sorted(table) == ["x", "y"]
    assert len(set(table.values())) == len(table)


@pytest.mark.parametrize("token", ["x", "y"])
def test_translated_token_is_accepted_by_car(token):
    """车端两套实现都收 -x/-y：dznav 的正则 + 官方 path_executor 的词表口径。"""
    out = car_commands.to_car_heading(token)
    assert RE_HEADING.match(out), f"{out!r} 不被 dznav RE_HEADING 接受"
    assert out in HEADING_YAWS


@pytest.mark.parametrize("token", ["x", "y"])
def test_translation_preserves_physical_direction(token):
    """翻译前后指向同一个物理方向（这是唯一真正的正确性判据）。"""
    uav = _grid_dir(_UAV_ANGLES[_DIRS[_HEADING_IDX[token]]], _UAV_ANGLES)
    car = _grid_dir(HEADING_YAWS[car_commands.to_car_heading(token)], _CAR_ANGLES)
    assert uav == car


# ---- 编码器出口确实翻译了 ----

def test_reset_command_emits_car_token():
    assert car_commands.reset_command("N10", "y") == "reset:N10,x"
    assert car_commands.reset_command("N10", "x") == "reset:N10,-y"


def test_reset_command_still_rejects_bad_heading():
    with pytest.raises(car_commands.CommandError):
        car_commands.reset_command("N10", "-y")
    with pytest.raises(car_commands.CommandError):
        car_commands.reset_command("N10", "north")


def test_route_command_emits_car_token():
    plan = car_commands.RoutePlan(("N10", "N8", "N5"), frozenset({"N8"}), "y")
    assert car_commands.route_command(plan) == "N10 N8* N5,x"


def test_route_command_without_final_heading_is_untouched():
    plan = car_commands.RoutePlan(("N10", "N5"))
    assert car_commands.route_command(plan) == "N10 N5"


def test_route_command_rejects_bad_final_heading():
    with pytest.raises(car_commands.CommandError):
        car_commands.route_command(
            car_commands.RoutePlan(("N10",), frozenset(), "-x"))


# ---- 本机侧内部自洽（防止只改一半）----

def test_planner_heading_index_matches_uav_frame_angles():
    """mission_router 的 _HEADING_IDX 与 integrate_node 的 _HEADING_YAW 同架。

    对齐闭环拿无人机 yaw 去比 _HEADING_YAW，规划器拿 _HEADING_IDX 约束末段
    进入朝向；两者必须是同一个物理方向，否则"对齐成功"和"路线要求"打架。
    """
    from ag_integrate.integrate_node import _HEADING_YAW

    for token, idx in _HEADING_IDX.items():
        physical = _grid_dir(_HEADING_YAW[token], _UAV_ANGLES)
        assert physical == _DIRS[idx], (
            f"token {token!r}: _HEADING_YAW 指向 {physical}，"
            f"规划器 _DIRS[{idx}] 是 {_DIRS[idx]}")
