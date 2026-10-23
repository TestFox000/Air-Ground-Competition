# -*- coding: utf-8 -*-
r"""网格注册单测：与车侧 ``geometry.py`` 的坐标契约对齐 + 单应往返 + 手性拦截。"""

from __future__ import annotations

import numpy as np

from ..grid import (COL_LETTERS, CORNER_ORDER, GridSpec, RegistrationError,
                    arena_corner_meters, register, register_from_corners,
                    register_from_named_corners)
from .synth import make_camera

# 车侧真源（dzrobot_v2/src/dzbrain/dzbrain/core/geometry.py:9-12 原文引用）
#   x = (10.5 - row_idx) * 0.35      row 11 → 0.175, row 1 → 3.675
#   y = (13.5 - col_idx) * 0.35      col N  → 0.175, col A → 4.725


def test_grid_spec_matches_car_side_geometry():
    spec = GridSpec()
    # 车侧 docstring 里逐字给出的三个锚点
    assert np.allclose(spec.block_to_meters("N11"), (0.175, 0.175), atol=1e-12)
    assert np.allclose(spec.block_to_meters("A1"), (3.675, 4.725), atol=1e-12)
    assert np.allclose(spec.block_to_meters("N10"), (0.525, 0.175), atol=1e-12)
    # K 列 index=10 ⇒ y=(13.5-10)*0.35=1.225
    assert np.allclose(spec.block_to_meters("K9"), (0.875, 1.225), atol=1e-12)
    # 场地尺寸：14 列 × 11 行 × 0.35
    assert abs(spec.width_m - 4.90) < 1e-12
    assert abs(spec.height_m - 3.85) < 1e-12
    assert abs(spec.barrier_x_m - 2.10) < 1e-12


def test_all_cells_round_trip():
    spec = GridSpec()
    assert len(spec.all_blocks()) == 154
    for block in spec.all_blocks():
        x, y = spec.block_to_meters(block)
        assert spec.meters_to_block(x, y) == block, block
        r, c = spec.block_to_index(block)
        assert spec.index_to_block(r, c) == block
    # 边界：格心 ±0.174 m 仍在同格；±0.176 m 落到邻格
    x, y = spec.block_to_meters("H5")
    assert spec.meters_to_block(x + 0.174, y) == "H5"
    assert spec.meters_to_block(x + 0.176, y) == "H4"     # +x = 行号减小
    assert spec.meters_to_block(x, y + 0.174) == "H5"
    assert spec.meters_to_block(x, y + 0.176) == "G5"     # +y = 列号减小


def test_half_division_follows_barrier_rows():
    spec = GridSpec()
    assert spec.half_of_block("K5") == "upper"      # 上半场（map.yaml bridges）
    assert spec.half_of_block("K10") == "lower"     # 下半场
    assert spec.half_of_block("A1") == "upper"
    assert spec.half_of_block("N11") == "lower"
    assert len(spec.blocks_of_half("upper")) == 6 * 14
    assert len(spec.blocks_of_half("lower")) == 5 * 14


def test_arena_corner_meters():
    spec = GridSpec()
    # 注意用 allclose：11×0.35 在二进制下是 3.8499999999999996，不是精确 3.85
    assert np.allclose(arena_corner_meters("N11", spec), (0.0, 0.0), atol=1e-12)
    assert np.allclose(arena_corner_meters("N1", spec), (3.85, 0.0), atol=1e-12)
    assert np.allclose(arena_corner_meters("A1", spec), (3.85, 4.90), atol=1e-12)
    assert np.allclose(arena_corner_meters("A11", spec), (0.0, 4.90), atol=1e-12)
    # 外角必须是"离场地中心最远"的那个角 —— 用格心距离交叉验证
    for tag, cell in (("N11", "N11"), ("N1", "N1"), ("A1", "A1"), ("A11", "A11")):
        cx, cy = spec.block_to_meters(cell)
        cxx, cyy = arena_corner_meters(tag, spec)
        assert abs(abs(cxx - cx) - spec.cell_m / 2) < 1e-12
        assert abs(abs(cyy - cy) - spec.cell_m / 2) < 1e-12
    try:
        arena_corner_meters("B7", spec)
    except RegistrationError:
        pass
    else:
        raise AssertionError("非法角点标签应报错")


def _check_camera(rot_deg: float, tilt: float, px_per_m: float = 64.0) -> dict:
    spec = GridSpec()
    H_true, size = make_camera(spec, px_per_m=px_per_m, rot_deg=rot_deg, tilt=tilt)
    corners = [tuple(H_true.apply([arena_corner_meters(t, spec)])[0])
               for t in CORNER_ORDER]
    reg = register_from_corners(corners, spec)
    # 1) 所有格心：像素 → 格名 必须闭合
    worst = 0.0
    for block in spec.all_blocks():
        x, y = spec.block_to_meters(block)
        u, v = H_true.apply([(x, y)])[0]
        hit = reg.px_to_hit(float(u), float(v))
        assert hit.block == block, f"rot={rot_deg} tilt={tilt} {block}→{hit.block}"
        assert hit.in_arena
        worst = max(worst, float(np.hypot(hit.x_m - x, hit.y_m - y)))
    # 2) 拟合出的 H 与真值 H 在格心上的差（像素）
    px_err = 0.0
    for block in spec.all_blocks():
        x, y = spec.block_to_meters(block)
        a = H_true.apply([(x, y)])[0]
        b = reg.meters_to_px([(x, y)])[0]
        px_err = max(px_err, float(np.hypot(*(a - b))))
    return {"worst_m": worst, "px_err": px_err, "size": size,
            "residual": reg.residual_px, "diagnostics": reg.diagnostics()}


def test_homography_round_trip_under_rotation_and_tilt():
    for rot in (0.0, 17.0, 90.0, 180.0, -33.0):
        for tilt in (0.0, 0.15):
            info = _check_camera(rot, tilt)
            # 4 点 DLT 在无噪声下应当重建到亚像素
            assert info["residual"] < 1e-6, info
            assert info["px_err"] < 1e-6, info
            assert info["worst_m"] < 1e-6, info


def test_registration_recovers_true_geometry_after_rotation():
    """旋转 90° 后，同一物理格必须仍解析成同一个格名（格名是场地属性，不是图像属性）。"""
    spec = GridSpec()
    H0, _ = make_camera(spec, px_per_m=50.0)
    H1, _ = make_camera(spec, px_per_m=50.0, rot_deg=90.0)
    reg0 = register_from_corners([tuple(H0.apply([arena_corner_meters(t, spec)])[0])
                                  for t in CORNER_ORDER], spec)
    reg1 = register_from_corners([tuple(H1.apply([arena_corner_meters(t, spec)])[0])
                                  for t in CORNER_ORDER], spec)
    for block in ("N10", "K5", "K10", "A1", "B11", "C11"):
        x, y = spec.block_to_meters(block)
        u0, v0 = H0.apply([(x, y)])[0]
        u1, v1 = H1.apply([(x, y)])[0]
        assert reg0.px_to_block(float(u0), float(v0)) == block
        assert reg1.px_to_block(float(u1), float(v1)) == block


def test_mirrored_corners_are_rejected():
    """角点顺序被打乱成镜像 ⇒ 必须报错（否则格名系统性错位）。"""
    spec = GridSpec()
    H_true, _ = make_camera(spec, px_per_m=40.0)
    corners = [tuple(H_true.apply([arena_corner_meters(t, spec)])[0])
               for t in CORNER_ORDER]
    mirrored = [corners[1], corners[0], corners[3], corners[2]]      # N1,N11,A11,A1
    try:
        register_from_corners(mirrored, spec)
    except RegistrationError as exc:
        assert "手性" in str(exc)
    else:
        raise AssertionError("镜像角点应被 RegistrationError 拦下")


def test_named_corners_and_extra_correspondences():
    spec = GridSpec()
    H_true, _ = make_camera(spec, px_per_m=48.0, rot_deg=12.0)
    px_of = {t: tuple(H_true.apply([arena_corner_meters(t, spec)])[0])
             for t in ("N11", "N1", "A1", "A11")}
    reg = register_from_named_corners(px_of, spec)
    assert reg.self_check()["ok"]
    # 追加 6 个格心对应点（>4 点最小二乘），结果应更稳且仍然闭合
    pairs = [(arena_corner_meters(t, spec), px_of[t]) for t in CORNER_ORDER]
    for block in ("N10", "K5", "K10", "M8", "B11", "C11"):
        x, y = spec.block_to_meters(block)
        pairs.append(((x, y), tuple(H_true.apply([(x, y)])[0])))
    reg2 = register(pairs, spec)
    assert reg2.self_check()["ok"]
    assert len(reg2.pairs) == 10
    assert reg2.diagnostics()["condition"] < 1e6


def test_cell_hit_offsets_and_area():
    spec = GridSpec()
    H_true, _ = make_camera(spec, px_per_m=60.0)
    reg = register_from_corners([tuple(H_true.apply([arena_corner_meters(t, spec)])[0])
                                 for t in CORNER_ORDER], spec)
    x, y = spec.block_to_meters("H5")
    hit = reg.meters_to_hit(x, y)
    assert hit.block == "H5"
    assert abs(hit.fx - 0.5) < 1e-9 and abs(hit.fy - 0.5) < 1e-9
    assert abs(hit.offset_from_center_m[0]) < 1e-9
    # 朝 +x（行号减小）= 行 5 → 行 4 方向
    hit2 = reg.meters_to_hit(x + 0.1, y)
    assert hit2.block == "H5" and hit2.fx > 0.5
    assert abs(hit2.offset_from_center_m[0] - 0.1) < 1e-9
    # 朝 +y（列号减小）= H → G
    hit3 = reg.meters_to_hit(x, y + 0.1)
    assert hit3.fy > 0.5 and abs(hit3.offset_from_center_m[1] - 0.1) < 1e-9
    # 透视下格面积随位置变化，但都 > 0
    areas = [reg.cell_area_px(b) for b in ("A1", "N11", "H5", "K10")]
    assert all(a > 0 for a in areas)


def test_bad_inputs_raise():
    spec = GridSpec()
    for bad in ("", "Z1", "A0", "A12", "5", "AA1"):
        try:
            spec.block_to_index(bad)
        except RegistrationError:
            continue
        raise AssertionError(f"{bad!r} 应报错")
    try:
        register_from_corners([(0, 0), (1, 0), (1, 1)], spec)
    except RegistrationError:
        pass
    else:
        raise AssertionError("3 点应报错（少于 4 组对应点）")
    # 共线四点
    try:
        register_from_corners([(0, 0), (10, 0), (20, 0), (30, 0)], spec)
    except RegistrationError:
        pass
    else:
        raise AssertionError("共线应报错")


def test_column_letters_and_corner_order_are_documented_values():
    assert COL_LETTERS == "ABCDEFGHIJKLMN"
    assert CORNER_ORDER == ("N11", "N1", "A1", "A11")
