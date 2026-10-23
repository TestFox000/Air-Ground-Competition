# -*- coding: utf-8 -*-
r"""颜色分类 / 形态学 / 连通域单测（合成图，确定性）。"""

from __future__ import annotations

import numpy as np

from ..colors import (ColorRule, FIELD_DEFAULT, MAP_IMAGE, Palette, as_rgb_array,
                      blob_masks, classify, close_, connected_components, dilate,
                      erode, hsv_to_rgb, open_, rgb_to_hsv, sample_patch_stats)


def test_rgb_to_hsv_known_values():
    px = np.array([[[255, 0, 0], [0, 255, 0], [0, 0, 255],
                    [255, 255, 0], [0, 255, 255], [255, 0, 255],
                    [128, 128, 128], [255, 255, 255]]], dtype=np.uint8)
    hsv = rgb_to_hsv(px)[0]
    h = hsv[:, 0]
    assert abs(h[0] - 0.0) < 1e-9 or abs(h[0] - 360.0) < 1e-9
    assert abs(h[1] - 120.0) < 1e-9
    assert abs(h[2] - 240.0) < 1e-9
    assert abs(h[3] - 60.0) < 1e-9
    assert abs(h[4] - 180.0) < 1e-9
    assert abs(h[5] - 300.0) < 1e-9
    # 灰度：S=0
    assert abs(hsv[6, 1]) < 1e-9 and abs(hsv[6, 2] - 128 / 255) < 1e-9
    assert abs(hsv[7, 1]) < 1e-9 and abs(hsv[7, 2] - 1.0) < 1e-9


def test_hsv_rgb_round_trip():
    rgb = np.array([[[255, 0, 0], [0, 255, 0], [12, 200, 240], [250, 250, 250],
                     [0, 0, 0], [123, 45, 67]]], dtype=np.uint8)
    back = hsv_to_rgb(rgb_to_hsv(rgb))
    assert np.abs(back.astype(np.int16) - rgb.astype(np.int16)).max() <= 1


def test_wrap_around_red_rule():
    rule = ColorRule("obstacle", hsv_lo=(345.0, 0.35, 0.30), hsv_hi=(12.0, 1.0, 1.0))
    px = np.array([[[255, 0, 0], [200, 30, 30], [0, 255, 0],
                    [255, 200, 0], [255, 255, 255]]], dtype=np.uint8)
    m = rule.mask(px)
    assert m.tolist() == [[True, True, False, False, False]]


def test_rgb_box_rule_equivalence_to_evidence_thresholds():
    """MAP_IMAGE 的 RGB 框必须与识图脚本原文阈值等价（逐字抄的，要能证明）。"""
    rng = np.random.default_rng(0)
    px = rng.integers(0, 256, size=(4000, 1, 3), dtype=np.uint8)
    r, g, b = (px[..., 0].astype(np.int16), px[..., 1].astype(np.int16),
               px[..., 2].astype(np.int16))
    # 原文：identify_t28_v2.py:58-63
    expect = {
        "obstacle": (r > 170) & (g < 110) & (b < 110),
        "mine": (r > 190) & (g > 160) & (b < 130),
        "target": (r > 170) & (g < 130) & (b > 150),
        "bridge": (r < 140) & (g > 120) & (b < 150),
    }
    masks = MAP_IMAGE.masks(px)
    for cls, exp in expect.items():
        got = masks[cls].reshape(-1)
        assert np.array_equal(got, exp.reshape(-1)), f"{cls} 阈值与原文不一致"
        assert int(got.sum()) > 0, f"{cls} 在 4000 个随机色上命中为 0，测试无意义"


def test_field_palette_provisional_rules_are_off_by_default():
    """没有实测证据的规则默认不参与分类 —— 这是防"猜错阈值静默出错"的闸门。"""
    assert "deck" in FIELD_DEFAULT.classes()
    assert "arrow" in FIELD_DEFAULT.classes()
    assert "mine" not in FIELD_DEFAULT.classes()
    assert "obstacle" not in FIELD_DEFAULT.classes()
    on = FIELD_DEFAULT.enable_provisional()
    assert "mine" in on.classes() and "obstacle" in on.classes() and "car" in on.classes()
    assert "mine" not in on.disable_provisional().classes()


def test_palette_json_round_trip(tmp_path=None):
    p = FIELD_DEFAULT.enable_provisional()
    d = p.to_json()
    q = Palette.from_json(d)
    assert q.name == p.name and len(q.rules) == len(p.rules)
    assert q.allow_provisional is True
    px = np.array([[[240, 205, 40], [30, 170, 70], [220, 40, 40]]], dtype=np.uint8)
    a, b = p.masks(px), q.masks(px)
    assert set(a) == set(b)
    for k in a:
        assert np.array_equal(a[k], b[k])


def test_with_rule_replaces_same_class():
    p = FIELD_DEFAULT.with_rule(ColorRule("deck", hsv_lo=(0.0, 0.0, 0.0),
                                          hsv_hi=(360.0, 1.0, 1.0)))
    assert p.masks(np.array([[[1, 2, 3]]], dtype=np.uint8))["deck"].all()
    assert sum(1 for r in p.rules if r.cls == "deck") == 1


def test_morphology_open_close_dilate_erode():
    m = np.zeros((11, 11), dtype=bool)
    m[5, 5] = True                       # 孤立噪点
    m[2:5, 2:5] = True                   # 3×3 方块
    assert connected_components(m, min_pixels=1)[0].area == 10          # 9 + 1
    assert connected_components(open_(m, 3), min_pixels=1)[0].area == 9  # 噪点被开掉
    hole = np.ones((5, 5), dtype=bool)
    hole[2, 2] = False
    assert close_(hole, 3).all()
    assert erode(hole, 3).shape == hole.shape
    big = dilate(np.eye(5, dtype=bool), 3)
    assert big.sum() > np.eye(5, dtype=bool).sum()
    assert erode(np.ones((5, 5), bool), 3).all()      # 全 1 腐蚀后仍全 1（内部）
    try:
        dilate(m, 4)
    except ValueError:
        pass
    else:
        raise AssertionError("偶数结构元应报错")


def test_connected_components_4_vs_8_connectivity():
    m = np.zeros((10, 10), dtype=bool)
    m[1:4, 1:4] = True
    m[6:9, 6:9] = True
    two = connected_components(m, min_pixels=1)
    assert len(two) == 2 and two[0].area == 9
    assert (two[0].x0, two[0].y0, two[0].x1, two[0].y1) == (1, 1, 3, 3)
    assert abs(two[0].cx - 2.0) < 1e-9 and abs(two[0].cy - 2.0) < 1e-9
    diag = np.zeros((5, 5), dtype=bool)
    diag[1, 1] = True
    diag[2, 2] = True                     # 对角相接
    assert len(connected_components(diag, min_pixels=1, connectivity=8)) == 1
    assert len(connected_components(diag, min_pixels=1, connectivity=4)) == 2
    assert connected_components(m, min_pixels=10) == []
    assert connected_components(np.zeros((3, 3), bool)) == []


def test_blob_moments_and_shapes():
    m = np.zeros((40, 60), dtype=bool)
    m[10:13, 5:35] = True                 # 宽 30、高 3 的长条
    b = connected_components(m, min_pixels=1)[0]
    assert b.area == 90 and b.w == 30 and b.h == 3
    assert b.elongation > 0.9
    assert min(b.axis_deg, 180 - b.axis_deg) < 1.0       # 主轴沿水平
    sq = np.zeros((20, 20), dtype=bool)
    sq[5:15, 5:15] = True
    b2 = connected_components(sq, min_pixels=1)[0]
    assert b2.elongation < 0.05 and b2.fill > 0.99
    # 圆盘填充率明显小于方块 —— objects.py 的 fill_range 判据就靠这条区分"圆片/方块"。
    # 离散栅格化会让实测值比连续值 πr²/(2r+1)² 再低约 2%（=0.018），所以容差取 0.025。
    r = 12
    yy, xx = np.mgrid[0:40, 0:40]
    disc = ((yy - 20.0) ** 2 + (xx - 20.0) ** 2) <= r ** 2
    b3 = connected_components(disc, min_pixels=1)[0]
    cont = np.pi * r ** 2 / b3.bbox_area
    assert abs(b3.fill - cont) < 0.025, (b3.fill, cont)
    assert 0.68 < b3.fill < 0.80
    sq_fill = connected_components(np.ones((30, 30), bool), min_pixels=1)[0].fill
    assert sq_fill > 0.99 and b3.fill < sq_fill - 0.20


def test_blob_mean_rgb_and_pixels():
    img = np.zeros((6, 6, 3), dtype=np.uint8)
    img[1:3, 1:3] = (10, 20, 30)
    blobs = connected_components(img[..., 0] == 10, min_pixels=1, rgb=img)
    assert len(blobs) == 1
    assert np.allclose(blobs[0].mean_rgb, (10, 20, 30))
    assert blobs[0].pixels.shape == (4, 2)


def test_blob_masks_chain_and_classify_alias():
    img = np.zeros((20, 20, 3), dtype=np.uint8)
    img[5:10, 5:10] = (255, 0, 0)        # 5×5 = 25 px 目标
    img[15, 15] = (255, 0, 0)            # 远处 1 px 椒盐噪点
    masks = classify(img, MAP_IMAGE)
    raw = blob_masks(masks["obstacle"], min_pixels=1, rgb=img)
    assert len(raw) == 2 and raw[0].area == 25
    op = blob_masks(masks["obstacle"], min_pixels=4, k_open=3, rgb=img)
    assert len(op) == 1 and op[0].area == 25      # 噪点被开掉、方块不受影响


def test_sample_patch_stats():
    img = np.zeros((10, 10, 3), dtype=np.uint8)
    img[:, :] = (200, 30, 30)
    st = sample_patch_stats(img, 5, 5, radius=2)
    assert st["n"] == 25
    assert abs(st["median_rgb"][0] - 200) < 1e-9
    assert st["median_hsv"][1] > 0.8
    try:
        sample_patch_stats(img, 100, 100)
    except ValueError:
        pass
    else:
        raise AssertionError("越界取样应报错")


def test_as_rgb_array_normalizes_inputs():
    from PIL import Image
    rgba = np.zeros((4, 4, 4), dtype=np.uint8)
    rgba[..., 3] = 255
    assert as_rgb_array(rgba).shape == (4, 4, 3)
    assert as_rgb_array(Image.new("L", (2, 2))).shape == (2, 2, 3)
    assert as_rgb_array(np.full((2, 2, 3), 300.0)).max() == 255
    try:
        as_rgb_array(np.zeros((2, 2)))
    except Exception as exc:
        assert "RGB" in str(exc) or "shape" in str(exc)
    else:
        raise AssertionError("二维数组应报错")
