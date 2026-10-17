# -*- coding: utf-8 -*-
r"""端到端单测：一帧合成俯视图 → 车侧报文（含退化特征化）。

覆盖：
* 全要素场景（8 雷 + 8 障 + 靶 + 桥面双箭头 + 车）→ 5 种报文与格名全对；
* 旋转 / 透视 / 像素噪声下的稳定性；
* 图纸口径（当前 ``config/map.yaml``）端到端复现；
* **只看得到就只说得到**（部分可见时不许补全）；
* **退化特征化**：模糊 / 噪声 / 低分辨率各自先坏哪一路 —— 这条最重要，
  因为它直接回答"摄像头糊了还能用吗"。
"""

from __future__ import annotations

import json

import numpy as np
from PIL import Image, ImageFilter

from ..car_pose import CarSpec
from ..grid import CORNER_ORDER, GridSpec
from ..objects import FIELD_SPECS
from ..pipeline import PerceptionConfig, PerceptionPipeline, config_for
from .synth import SYNTH_FIELD_PALETTE, render_arena

# 当前地图真值（dzrobot_v2/config/map.yaml:49-55 原文）
MAP_MINES = ["B1", "G2", "H3", "A4", "D8", "F9", "I10", "E11"]
MAP_OBSTACLES = ["D1", "F4", "A5", "H5", "B7", "C9", "E10", "G11"]
MAP_TARGETS = ["B11", "C11"]
MAP_BRIDGES = ["K5", "K10"]

FIELD_CFG = PerceptionConfig(palette=SYNTH_FIELD_PALETTE, object_specs=FIELD_SPECS,
                             car_spec=CarSpec(cls="car", nose_cls="car_nose"))


def _corners(arena):
    return [arena.corners_px[t] for t in CORNER_ORDER]


def _scene(**kw):
    kw.setdefault("mines", MAP_MINES)
    kw.setdefault("obstacles", MAP_OBSTACLES)
    kw.setdefault("targets", MAP_TARGETS)
    kw.setdefault("deck_col", "K")
    kw.setdefault("arrows", [("K5", 90.0), ("K10", -90.0)])
    kw.setdefault("car", ("N10", 90.0))
    kw.setdefault("car_nose_offset", 0.16)
    return render_arena(GridSpec(), **kw)


def test_end_to_end_full_scene():
    arena = _scene()
    res = PerceptionPipeline(FIELD_CFG).process(arena.image, corners_px=_corners(arena))
    assert [m["type"] for m in res.messages] == [
        "heartbeat", "bridge_passable", "bridge_passable", "target_zone", "field_verify"]
    bridges = {(m["half"], m["block"]) for m in res.messages
               if m["type"] == "bridge_passable"}
    assert bridges == {("upper", "K5"), ("lower", "K10")}
    fv = next(m for m in res.messages if m["type"] == "field_verify")
    assert fv["mines"] == sorted(MAP_MINES) and fv["obstacles"] == sorted(MAP_OBSTACLES)
    tz = next(m for m in res.messages if m["type"] == "target_zone")
    assert tz["blocks"] == sorted(MAP_TARGETS)
    assert res.objects.target_groups == [["B11", "C11"]]
    assert res.car is not None and res.car.block == "N10"
    assert abs(((res.car.yaw_deg - 90.0) + 180.0) % 360.0 - 180.0) < 8.0
    # 报文自检 + 车侧解析镜像：必须"会被正常处理"，不能落进 missing/unknown
    for m in res.messages:
        assert res.diagnostics["validate"][m["type"]] == []
    for parsed in res.diagnostics["mirror_car_parse"]:
        assert parsed["raw_ok"] and "missing_fields" not in parsed["action"]
        assert "未知类型" not in parsed["action"]
    assert res.objects.conflicts == []


def test_end_to_end_is_deterministic():
    """同一帧、同一标定 ⇒ 结论逐字相同。

    比较前要剥掉 ``ts``（墙上时间）与 ``seq``（单调序号）——这两个字段**本来就该不同**：
    ``ts`` 是发包时刻，``seq`` 每次递增（车侧靠它回 ack）。感知结论必须完全一致。
    """
    arena = _scene()
    pipe = PerceptionPipeline(FIELD_CFG)

    def strip(res):
        out = []
        for m in res.messages:
            body = {k: v for k, v in m.items() if k not in ("ts", "seq")}
            out.append(json.dumps(body, ensure_ascii=False, sort_keys=True))
        return out

    r1 = pipe.process(arena.image, corners_px=_corners(arena))
    r2 = pipe.process(arena.image, corners_px=_corners(arena))
    assert strip(r1) == strip(r2)
    assert r1.to_json()["objects"] == r2.to_json()["objects"]
    assert r1.to_json()["cars"] == r2.to_json()["cars"]
    # seq 必须递增（否则车侧 ack 会张冠李戴）
    assert [m["seq"] for m in r2.messages] == [m["seq"] + 5 for m in r1.messages]


def test_end_to_end_survives_rotation_tilt_and_noise():
    """150 px/m（≈2.3 mm/px）= t1 报告里无人机 2 m 高度的 GSD 量级。

    ⚠️ 不要用 64 px/m 跑这个场景：那时 0.20 m 的箭头只有 13 px，旋 18° 后约 7 px，
    结构判据给不出闸门以上的置信度（见 ``test_bridge_arrow`` 的分辨率测试）。
    """
    arena = _scene(px_per_m=150.0, rot_deg=18.0, tilt=0.20, noise_sigma=8.0, seed=5)
    res = PerceptionPipeline(FIELD_CFG).process(arena.image, corners_px=_corners(arena))
    bridges = {(m["half"], m["block"]) for m in res.messages
               if m["type"] == "bridge_passable"}
    assert bridges == {("upper", "K5"), ("lower", "K10")}
    fv = next(m for m in res.messages if m["type"] == "field_verify")
    assert fv["mines"] == sorted(MAP_MINES) and fv["obstacles"] == sorted(MAP_OBSTACLES)
    assert res.car is not None and res.car.block == "N10"


def test_end_to_end_on_current_map_drawing():
    """图纸口径（drawing 风格 + MAP_IMAGE 阈值）：复现当前 config/map.yaml 的雷/障/靶/通。"""
    arena = render_arena(GridSpec(), style="drawing", mines=MAP_MINES,
                         obstacles=MAP_OBSTACLES, targets=MAP_TARGETS,
                         bridges=MAP_BRIDGES, draw_border=False)
    cfg = config_for("map_image")
    res = PerceptionPipeline(cfg).process(arena.image, corners_px=_corners(arena))
    assert res.objects.mines == sorted(MAP_MINES)
    assert res.objects.obstacles == sorted(MAP_OBSTACLES)
    assert res.objects.bridges == sorted(MAP_BRIDGES)
    fv = next(m for m in res.messages if m["type"] == "field_verify")
    tz = next(m for m in res.messages if m["type"] == "target_zone")
    assert fv["mines"] == sorted(MAP_MINES) and tz["blocks"] == sorted(MAP_TARGETS)
    # 图纸口径没有箭头/车顶标记 ⇒ 不该凭空产出 bridge_passable
    assert "bridge_passable" not in [m["type"] for m in res.messages]


def test_no_hallucination_when_only_part_of_field_is_visible():
    """只画 3 个雷 ⇒ field_verify 里只能有这 3 个（宁可漏，不可编）。"""
    arena = _scene(mines=["B1", "G2", "H3"], obstacles=["D1"], targets=[],
                   arrows=[], car=None)
    res = PerceptionPipeline(FIELD_CFG).process(arena.image, corners_px=_corners(arena))
    fv = next(m for m in res.messages if m["type"] == "field_verify")
    assert fv["mines"] == ["B1", "G2", "H3"] and fv["obstacles"] == ["D1"]
    assert "target_zone" not in [m["type"] for m in res.messages]
    assert "bridge_passable" not in [m["type"] for m in res.messages]
    assert res.car is None


def test_degradation_characterization_blur_noise_resolution():
    """**特征化测试**：把"先坏哪一路"钉死，供可行性结论直接引用。

    实测（64 px/m，合成全要素场景，PIL 高斯模糊，半径单位=px）：

    | 退化 | 结果 |
    |---|---|
    | 干净 | 8 雷 + 8 障 + 靶 + 两半场箭头 + 车，全对 |
    | 模糊 r=1 | **全对**（箭头能扛住 1 px 模糊） |
    | 模糊 r=2 | 雷 7/8；箭头、障、靶、车全对 |
    | 模糊 r=3 | **雷 0/8 + 箭头全丢 + 车丢**；障 8/8、靶仍在 |
    | 模糊 r=4 | 障也丢（0/8）；靶仍在 |
    | 模糊 r=8 | 靶也丢（只剩 heartbeat） |
    | 噪声 σ=20/255 | 全对 |
    | 噪声 σ=30/255 | 障剩 4/8；箭头两半场仍在 |
    | 噪声 σ=45/255 | 雷 4/8、障 0/8、只剩上半场箭头 |
    | 分辨率 32 px/m | 雷 7/8，其余全对（箭头仍在） |
    | 分辨率 16 px/m | 雷 5/8、靶与箭头全丢 |

    ⇒ 结论：**12 cm 地雷与桥顶箭头是最脆的两路**，比 30 cm 水桶先失效；
    整格大小的靶区最耐糊。
    """
    def run(img, arena_ok):
        res = PerceptionPipeline(FIELD_CFG).process(img, corners_px=_corners(arena_ok))
        bridges = {(m["half"], m["block"]) for m in res.messages
                   if m["type"] == "bridge_passable"}
        return {"bridge": bridges,
                "mines": len(set(res.objects.mines) & set(MAP_MINES)),
                "obstacles": len(set(res.objects.obstacles) & set(MAP_OBSTACLES)),
                "targets": res.objects.targets,
                "car": None if res.car is None else res.car.block}

    def blur(radius, **kw):
        base = _scene(**kw)
        img = np.asarray(Image.fromarray(base.image).filter(
            ImageFilter.GaussianBlur(radius)))
        return base, img

    clean_arena = _scene()
    clean = run(clean_arena.image, clean_arena)
    assert clean["bridge"] == {("upper", "K5"), ("lower", "K10")}
    assert clean["mines"] == 8 and clean["obstacles"] == 8
    assert clean["car"] == "N10"

    for radius, exp in ((1.0, dict(mines=8, obstacles=8, bridge=2, car="N10")),
                        (2.0, dict(mines=7, obstacles=8, bridge=2, car="N10")),
                        (3.0, dict(mines=0, obstacles=8, bridge=0, car=None)),
                        (4.0, dict(mines=0, obstacles=0, bridge=0, car=None)),
                        (8.0, dict(mines=0, obstacles=0, bridge=0, car=None))):
        base, img = blur(radius)
        r = run(img, base)
        assert r["mines"] == exp["mines"], (radius, r)
        assert r["obstacles"] == exp["obstacles"], (radius, r)
        assert len(r["bridge"]) == exp["bridge"], (radius, r)
        assert r["car"] == exp["car"], (radius, r)
        # 靶区（整格色块）在 r≤4 时必须还在 —— 最耐糊的一路
        if radius <= 4.0:
            assert r["targets"] == sorted(MAP_TARGETS), (radius, r)
        else:
            assert r["targets"] == []

    n20_arena = _scene(noise_sigma=20.0, seed=3)
    n20 = run(n20_arena.image, n20_arena)
    assert n20["mines"] == 8 and n20["obstacles"] == 8 and len(n20["bridge"]) == 2
    n30_arena = _scene(noise_sigma=30.0, seed=3)
    n30 = run(n30_arena.image, n30_arena)
    assert n30["obstacles"] < 8 or len(n30["bridge"]) < 2, n30

    for ppm, exp_mines, exp_bridge, exp_targets in (
            (32.0, 7, 2, sorted(MAP_TARGETS)), (16.0, 5, 0, [])):
        arena = render_arena(GridSpec(), px_per_m=ppm, mines=MAP_MINES,
                             obstacles=MAP_OBSTACLES, targets=MAP_TARGETS,
                             deck_col="K", arrows=[("K5", 90.0), ("K10", -90.0)],
                             car=("N10", 90.0), car_nose_offset=0.16)
        res = PerceptionPipeline(FIELD_CFG).process(arena.image,
                                                    corners_px=_corners(arena))
        assert len(set(res.objects.mines) & set(MAP_MINES)) == exp_mines, ppm
        assert res.objects.targets == exp_targets, ppm
        assert len([m for m in res.messages
                    if m["type"] == "bridge_passable"]) == exp_bridge
