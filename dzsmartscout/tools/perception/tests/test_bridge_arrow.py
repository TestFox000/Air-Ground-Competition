# -*- coding: utf-8 -*-
r"""桥顶箭头 → 半场通行格 单测（含方向估计器在**合成**与**实物照片**上的验证）。

方向估计器实测（本文件里的合成箭头是**实心**掩膜，与真实 blob 同构）：

| 场景 | 结果 |
|---|---|
| 合成实心箭头，14 个角度（0…179、-30/-90/-135） | 误差 ≤1.0°，置信度 1.00 |
| 点抖动 σ=0.005 m（箭头长 0.2 m 的 2.5%） | 误差 −12°，置信度 1.00 |
| 点抖动 σ=0.01 m（5%） | 误差 −4°，置信度 1.00 |
| 点抖动 σ≥0.02 m（≥10%） | **可能翻 90°+ 而置信度仍 0.9~1.0** ⇒ 已知失败模式，见特征化测试 |
| 正方形（各向同性） | 置信度 0.00（找不到"箭杆+张开"结构） |
| **实物照片** `arena_pdf_img/p09_1.png` 的绿箭头 | 估计 −96°，真值"指向图像上方"=−90° ⇒ 误差 6° |
"""

from __future__ import annotations

import pathlib

import numpy as np
from PIL import Image

from ..bridge_arrow import (ArrowSpec, detect_arrows, direction_from_points,
                            infer_bridge_passable)
from ..colors import PALETTES, as_rgb_array, blob_masks
from ..grid import CORNER_ORDER, GridSpec, register_from_corners
from .synth import SYNTH_FIELD_PALETTE, arrow_polygon_meters, render_arena

DIR_TOL = 5.0
#: 实物照片（只读引用；不在 perception 目录里，缺失时跳过该测试）
REAL_ARROW_IMG = (pathlib.Path(__file__).resolve().parents[4] / "dzsmartscout"
                  / "fc-audit" / "arena_pdf_img" / "p09_1.png")
REAL_ARROW_BBOX = (634, 534, 714, 591)      # 实测 bbox（px）


def _setup(px_per_m: float = 64.0, **kw):
    spec = GridSpec()
    arena = render_arena(spec, px_per_m=px_per_m, deck_col="K", **kw)
    reg = register_from_corners([arena.corners_px[t] for t in CORNER_ORDER], spec)
    return arena, reg


def _filled_arrow_points(deg: float, step: float = 0.0012) -> np.ndarray:
    """把箭头多边形栅格化成**实心**点集（模拟真实掩膜，不是只有轮廓）。"""
    poly = arrow_polygon_meters((0.0, 0.0), deg, 0.20)
    xs = np.arange(poly[:, 0].min(), poly[:, 0].max(), step)
    ys = np.arange(poly[:, 1].min(), poly[:, 1].max(), step)
    grid_x, grid_y = np.meshgrid(xs, ys)
    inside = np.zeros(grid_x.shape, dtype=bool)
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        cond = ((y1 > grid_y) != (y2 > grid_y))
        with np.errstate(divide="ignore", invalid="ignore"):
            xint = (x2 - x1) * (grid_y - y1) / (y2 - y1) + x1
        inside ^= cond & (grid_x < xint)
    return np.stack([grid_x[inside], grid_y[inside]], axis=1)


def test_direction_estimator_on_clean_filled_arrows():
    for deg in (0.0, 15.0, 30.0, 45.0, 60.0, 90.0, 120.0, 135.0, 150.0, 165.0, 179.0,
                -30.0, -90.0, -135.0):
        res = direction_from_points(_filled_arrow_points(deg))
        assert res["dir_xy"] is not None, deg
        err = (res["dir_deg"] - deg + 180.0) % 360.0 - 180.0
        assert abs(err) <= 2.0, (deg, res["dir_deg"], res["evidence"])
        assert res["confidence"] >= 0.9, (deg, res["confidence"], res["evidence"])
        assert res["evidence"]["n_agree"] >= 2
        assert res["evidence"]["shaft_len"] >= 2 and res["evidence"]["mono_len"] >= 2


def test_pca_axis_alone_would_be_wrong_for_real_glyph():
    """把"为什么不用 PCA"钉成回归测试：实物箭头的头部比身长，PCA 主轴是横向的。"""
    if not REAL_ARROW_IMG.is_file():
        return                                    # 只读文件不在 ⇒ 跳过
    img = as_rgb_array(Image.open(REAL_ARROW_IMG))
    mask = PALETTES["field"].enable_provisional().masks(img)["arrow"]
    blob = [b for b in blob_masks(mask, min_pixels=40, k_close=1)
            if (b.x0, b.y0, b.x1, b.y1) == REAL_ARROW_BBOX][0]
    res = direction_from_points(blob.pixels[:, ::-1].astype(float))
    pca = res["evidence"]["pca_axis_deg"]
    assert min(abs(pca - 0.0), abs(pca - 180.0)) < 20.0        # PCA 说"横向"
    assert abs(abs(res["axis_deg"]) - 90.0) < 20.0             # 结构法说"竖向"
    assert abs(res["dir_deg"] + 90.0) < 15.0, res["evidence"]  # 真值：指向图像上方
    assert res["confidence"] >= 0.5


def test_direction_estimator_rejects_symmetric_shape():
    """各向同性的形状（正方形）方向无意义 ⇒ 置信度 0（连结构都找不到）。"""
    yy, xx = np.mgrid[0:20, 0:20]
    sq = np.stack([xx.ravel() * 0.01, yy.ravel() * 0.01], axis=1)
    res = direction_from_points(sq)
    assert res["confidence"] == 0.0
    assert res["evidence"]["anisotropy"] < 0.05
    assert res["evidence"].get("axis_score") is None


def test_direction_is_exact_clean_and_close_under_mild_jitter():
    clean = direction_from_points(_filled_arrow_points(30.0))
    assert abs((clean["dir_deg"] - 30.0 + 180.0) % 360.0 - 180.0) <= 1.0
    rng = np.random.default_rng(7)
    base = _filled_arrow_points(30.0)
    for sigma in (0.005, 0.01):
        res = direction_from_points(base + rng.normal(0, sigma, base.shape))
        err = abs((res["dir_deg"] - 30.0 + 180.0) % 360.0 - 180.0)
        assert err < 15.0, (sigma, res["dir_deg"], res["evidence"])


def test_direction_failure_mode_under_heavy_jitter_is_documented():
    """**特征化测试**：点抖动 ≥ 箭头长的 10% 时方向可能翻 90°+ 而置信度仍 ≥0.9。

    对策已在设计里：默认 ``infer_mode="position"``（H1）只用**位置**定通行格，
    方向只作证据输出；``n_agree`` 与 ``axis_score`` 都留在 evidence 里供人复核。
    估计器若改进，本测试会失败并提示同步更新文档。
    """
    rng = np.random.default_rng(7)
    base = _filled_arrow_points(30.0)
    flips = []
    for sigma in (0.02, 0.03, 0.04):
        res = direction_from_points(base + rng.normal(0, sigma, base.shape))
        err = abs((res["dir_deg"] - 30.0 + 180.0) % 360.0 - 180.0)
        if err > 60.0:
            flips.append((sigma, res["confidence"]))
    assert flips, "预期失败模式未复现：估计器行为已变，请复核文档"
    assert any(c > 0.5 for _, c in flips), flips


# --------------------------------------------------------------------------- #
#  端到端：箭头 → bridge_passable
# --------------------------------------------------------------------------- #
def test_bridge_passable_position_mode_both_halves():
    arena, reg = _setup(arrows=[("K5", 90.0), ("K10", -90.0)])
    arrows = detect_arrows(arena.image, reg, SYNTH_FIELD_PALETTE, ArrowSpec())
    assert {a.block for a in arrows} == {"K5", "K10"}
    for a in arrows:
        # p09_1.png 实测：箭头尖头沿**穿洞方向**（列方向）⇒ along_tunnel
        assert a.dir_kind == "along_tunnel", (a.block, a.dir_deg_map, a.evidence)
        err = abs((a.dir_deg_map - (90.0 if a.block == "K5" else -90.0) + 180.0)
                  % 360.0 - 180.0)
        assert err < 10.0, (a.block, a.dir_deg_map)
    calls = {c.half: c for c in infer_bridge_passable(arrows, ArrowSpec())}
    assert calls["upper"].block == "K5"
    assert calls["lower"].block == "K10"
    assert all(c.confidence >= ArrowSpec().min_conf for c in calls.values())


def test_bridge_along_strip_direction_is_flagged():
    """箭头若指向桥带长轴（±x），应被标成 along_strip —— H1 语义下这属于异常证据。"""
    arena, reg = _setup(arrows=[("K5", 0.0)])
    arrows = detect_arrows(arena.image, reg, SYNTH_FIELD_PALETTE, ArrowSpec())
    assert arrows[0].dir_kind == "along_strip"


def test_bridge_direction_mode_h2():
    """H2（方向）判据：箭头在 K10、指向 +x（行号减小）⇒ 通行格 K9。"""
    arena, reg = _setup(arrows=[("K10", 0.0)])
    arrows = detect_arrows(arena.image, reg, SYNTH_FIELD_PALETTE, ArrowSpec())
    assert arrows and arrows[0].confidence >= ArrowSpec().min_conf, arrows
    calls = infer_bridge_passable(arrows, ArrowSpec(infer_mode="direction",
                                                    direction_steps=1))
    assert len(calls) == 1 and calls[0].half == "lower"
    assert calls[0].block == "K9"                    # K10 沿 +x 一格 → K9
    ev = " ".join(calls[0].evidence)
    assert "H2" in ev and "K9" in ev


def test_arrow_detection_resolution_behaviour():
    """分辨率效应（记录在案）：0.20 m 箭头在 64 px/m 下只有 ~13 px、旋+斜视后 ~7 px，
    结构判据（箭杆 2 段 + 张开 2 段）给不出 0.30 闸门以上的置信度 ⇒ 不出结论。
    150 px/m（≈2.3 mm/px，对应无人机 2 m 高度）旋+斜视后仍有 0.6 置信度。"""
    cases = [(64.0, 0.0, 0.0, 2), (64.0, 18.0, 0.20, 0), (150.0, 18.0, 0.20, 2)]
    for ppm, rot, tilt, expect_calls in cases:
        arena, reg = _setup(px_per_m=ppm, rot_deg=rot, tilt=tilt,
                            arrows=[("K5", 90.0), ("K10", -90.0)])
        arrows = detect_arrows(arena.image, reg, SYNTH_FIELD_PALETTE, ArrowSpec())
        calls = infer_bridge_passable(arrows, ArrowSpec())
        assert len(calls) == expect_calls, (ppm, rot, tilt,
                                           [(a.block, a.confidence) for a in arrows])
        for c in calls:
            assert c.block in ("K5", "K10")


def test_arrow_outside_bridge_column_is_ignored_by_default():
    arena, reg = _setup(arrows=[("J5", 90.0)])
    arrows = detect_arrows(arena.image, reg, SYNTH_FIELD_PALETTE, ArrowSpec())
    assert arrows and arrows[0].block == "J5"
    assert infer_bridge_passable(arrows, ArrowSpec()) == []
    calls = infer_bridge_passable(arrows, ArrowSpec(allow_off_column=True))
    assert len(calls) == 1 and calls[0].block == "J5"


def test_no_arrow_no_call_and_low_confidence_gated():
    arena, reg = _setup(arrows=[])
    arrows = detect_arrows(arena.image, reg, SYNTH_FIELD_PALETTE, ArrowSpec())
    assert arrows == []
    assert infer_bridge_passable(arrows, ArrowSpec()) == []
    # 抬高 confidence 闸门 ⇒ 有箭头也不出结论（宁可不报）
    arena2, reg2 = _setup(arrows=[("K5", 90.0)])
    arrows2 = detect_arrows(arena2.image, reg2, SYNTH_FIELD_PALETTE, ArrowSpec())
    assert infer_bridge_passable(arrows2, ArrowSpec(min_conf=0.999)) == []


def test_tiny_blob_is_filtered_by_area_gate():
    arena, reg = _setup(arrows=[])
    img = arena.image.copy()
    u, v = reg.cell_center_px("K5")
    ui, vi = int(round(u)), int(round(v))
    img[vi - 1:vi + 2, ui - 1:ui + 2] = (30, 170, 70)
    arrows = detect_arrows(img, reg, SYNTH_FIELD_PALETTE, ArrowSpec())
    assert arrows == []


def test_arrow_detection_is_deterministic():
    arena, reg = _setup(px_per_m=150.0, rot_deg=35.0, tilt=0.2,
                        arrows=[("K5", 90.0), ("K10", -90.0)])
    a = [x.to_json() for x in detect_arrows(arena.image, reg, SYNTH_FIELD_PALETTE,
                                            ArrowSpec())]
    b = [x.to_json() for x in detect_arrows(arena.image, reg, SYNTH_FIELD_PALETTE,
                                            ArrowSpec())]
    assert a == b
    assert {x["block"] for x in a} == {"K5", "K10"}
    for x in a:
        assert x["dir_kind"] == "along_tunnel"
