# -*- coding: utf-8 -*-
r"""车顶标记 → ``block + yaw`` 单测（含透视/旋转、双标记解歧义、置信度闸门）。

**实测精度**（合成、无噪声；车顶板 0.22×0.10 m，车头标记 0.06 m 方块、前移 0.16 m，
两者不重叠）：

| 配置 | 方向误差 max / mean |
|---|---|
| 64 px/m，正俯视（rot=0,tilt=0） | 4.63° / 1.74° |
| 64 px/m，rot=90°+tilt=0.30（强斜视） | 6.15° / 2.28° |
| 150 px/m（≈2.3 mm/px，对应无人机 2 m 高度） | 2.07° / 0.91° |
| 200 px/m | 1.62° / 0.73° |

**格名在全部 50 组配置里 100% 正确**。误差主要来自像素栅格化（64 px/m 时板只有
14×6 px），所以下面的容差按分辨率分别给：64 px/m → 8°，150 px/m → 3°。
⚠️ 车头标记若与车顶板**重叠**，会把板的一端遮掉、PCA 主轴被拉偏（实测 rot=90/tilt=0.3
时误差从 6.2° 涨到 12.6°）——这是标记布置问题，**安装时要让两者分开**。
"""

from __future__ import annotations

import numpy as np
from PIL import Image, ImageDraw

from ..car_pose import CarSpec, best_car, detect_car
from ..grid import CORNER_ORDER, GridSpec, register_from_corners
from .synth import SYNTH_FIELD_PALETTE, board_polygon_meters, render_arena

YAW_TOL_COARSE = 8.0    # 64 px/m（实测最差 6.15°）
YAW_TOL_FINE = 3.0      # 150 px/m（实测最差 2.07°）
NOSE_OFFSET = 0.16      # 与主板不重叠的最小前移量（板半长 0.11 + 标记半宽 0.03）


def _setup(px_per_m: float = 64.0, **kw):
    spec = GridSpec()
    arena = render_arena(spec, px_per_m=px_per_m, **kw)
    reg = register_from_corners([arena.corners_px[t] for t in CORNER_ORDER], spec)
    return arena, reg


def test_car_pose_with_nose_is_unambiguous_in_flat_frame():
    for yaw in (0.0, 30.0, 45.0, 90.0, 135.0, 180.0, -90.0, 175.0, -175.0):
        arena, reg = _setup(car=("K5", yaw), car_nose_offset=NOSE_OFFSET)
        cars = detect_car(arena.image, reg, SYNTH_FIELD_PALETTE,
                          CarSpec(cls="car", nose_cls="car_nose"))
        assert cars, f"yaw={yaw} 没检到车"
        c = cars[0]
        assert c.yaw_ambiguous is False and c.yaw_source == "board_axis+nose"
        err = (c.yaw_deg - yaw + 180.0) % 360.0 - 180.0
        assert abs(err) < YAW_TOL_COARSE, (yaw, c.yaw_deg, err)
        assert c.block == "K5" and c.half == "upper"
        assert 0.5 < c.confidence <= 1.0


def test_car_pose_survives_rotation_and_tilt():
    for rot, tilt in ((25.0, 0.25), (180.0, 0.0), (90.0, 0.30), (-40.0, 0.15)):
        for yaw in (0.0, 45.0, 90.0, 200.0):
            arena, reg = _setup(rot_deg=rot, tilt=tilt, car=("M10", yaw),
                                car_nose_offset=NOSE_OFFSET)
            cars = detect_car(arena.image, reg, SYNTH_FIELD_PALETTE,
                              CarSpec(cls="car", nose_cls="car_nose"))
            assert cars, (rot, tilt, yaw)
            c = cars[0]
            err = (c.yaw_deg - yaw + 180.0) % 360.0 - 180.0
            assert abs(err) < YAW_TOL_COARSE, (rot, tilt, yaw, c.yaw_deg, err)
            assert c.block == "M10" and c.half == "lower"


def test_car_pose_accuracy_at_realistic_ground_sample_distance():
    """150 px/m ≈ 2.3 mm/px（对应 t1 报告里无人机 2 m 高度、1920 档的 GSD 量级）。"""
    for rot, tilt in ((0.0, 0.0), (25.0, 0.25), (90.0, 0.30)):
        for yaw in (0.0, 30.0, 90.0, 180.0, -90.0, 200.0):
            arena, reg = _setup(px_per_m=150.0, rot_deg=rot, tilt=tilt,
                                car=("K5", yaw), car_nose_offset=NOSE_OFFSET)
            c = detect_car(arena.image, reg, SYNTH_FIELD_PALETTE,
                           CarSpec(cls="car", nose_cls="car_nose"))[0]
            err = (c.yaw_deg - yaw + 180.0) % 360.0 - 180.0
            assert abs(err) < YAW_TOL_FINE, (rot, tilt, yaw, c.yaw_deg, err)
            assert c.block == "K5"


def test_single_marker_is_ambiguous_mod_180():
    arena, reg = _setup(car=("K9", 300.0))          # 无车头标记
    cars = detect_car(arena.image, reg, SYNTH_FIELD_PALETTE, CarSpec(cls="car"))
    assert cars
    c = cars[0]
    assert c.yaw_ambiguous is True and c.yaw_source == "pca"
    # 只能定到 mod 180：与 300-180=120 或 300 相差 180 的整数倍
    diff = abs(((c.yaw_deg - 300.0) + 90.0) % 180.0 - 90.0)
    assert diff < YAW_TOL_COARSE, (c.yaw_deg, diff)
    # 歧义 ⇒ 置信度减半（同场景带 nose 时约 0.7）
    assert c.confidence < 0.5


def test_expect_yaw_resolves_ambiguity():
    arena, reg = _setup(car=("K9", 300.0))
    cars = detect_car(arena.image, reg, SYNTH_FIELD_PALETTE,
                      CarSpec(cls="car", expect_yaw_deg=300.0))
    assert abs(((cars[0].yaw_deg - 300.0) + 180.0) % 360.0 - 180.0) < YAW_TOL_COARSE
    assert cars[0].yaw_ambiguous is True          # 仍是"靠先验解出来的"，诚实标注


def test_yaw_offset_compensation():
    arena, reg = _setup(car=("K5", 90.0), car_nose_offset=NOSE_OFFSET)
    c = detect_car(arena.image, reg, SYNTH_FIELD_PALETTE,
                   CarSpec(cls="car", nose_cls="car_nose", yaw_offset_deg=90.0))[0]
    assert abs(((c.yaw_deg - 0.0) + 180.0) % 360.0 - 180.0) < YAW_TOL_COARSE


def test_no_car_returns_empty_and_best_car_gate():
    arena, reg = _setup(mines=["B1"])
    assert detect_car(arena.image, reg, SYNTH_FIELD_PALETTE,
                      CarSpec(cls="car")) == []
    assert best_car(arena.image, reg, SYNTH_FIELD_PALETTE, CarSpec(cls="car")) is None
    # 单标记场景：置信度低于闸门时 best_car 返回 None，但 detect_car 仍给出候选
    a2, r2 = _setup(car=("K9", 10.0))
    assert best_car(a2.image, r2, SYNTH_FIELD_PALETTE, CarSpec(cls="car"),
                    min_confidence=0.9) is None
    assert detect_car(a2.image, r2, SYNTH_FIELD_PALETTE, CarSpec(cls="car"))


def test_two_cars_sorted_by_confidence_and_off_cell_rejected():
    arena, reg = _setup(car=("K9", 45.0), car_nose_offset=NOSE_OFFSET)
    # 手工在 H2 再画一块同样的橙板
    img = Image.fromarray(arena.image.copy())
    d = ImageDraw.Draw(img)
    cx, cy = reg.spec.block_to_meters("H2")
    poly = reg.meters_to_px(board_polygon_meters((cx, cy), 100.0))
    d.polygon([tuple(p) for p in poly], fill=(240, 130, 30))
    cars = detect_car(np.asarray(img), reg, SYNTH_FIELD_PALETTE, CarSpec(cls="car"))
    assert {c.block for c in cars} == {"K9", "H2"}
    assert cars[0].confidence >= cars[1].confidence
    # 面积闸门：把 max_cell_frac 压到 0.05 时应全部被拒
    assert detect_car(np.asarray(img), reg, SYNTH_FIELD_PALETTE,
                      CarSpec(cls="car", max_cell_frac=0.05)) == []


def test_car_detection_is_deterministic():
    arena, reg = _setup(car=("N10", 90.0), car_nose_offset=NOSE_OFFSET)
    spec = CarSpec(cls="car", nose_cls="car_nose")
    a = [c.to_json() for c in detect_car(arena.image, reg, SYNTH_FIELD_PALETTE, spec)]
    b = [c.to_json() for c in detect_car(arena.image, reg, SYNTH_FIELD_PALETTE, spec)]
    assert a == b
    # 车在起点 N10、朝 +y（= 图左）——与 map.yaml:18 "mission.reset=N10,y" 一致
    assert a[0]["block"] == "N10" and abs(a[0]["yaw_deg"] - 90.0) < YAW_TOL_COARSE
