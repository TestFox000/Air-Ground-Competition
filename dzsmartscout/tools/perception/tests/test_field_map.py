# -*- coding: utf-8 -*-
r"""t59 L1 出图单测：感知输出 → ``FieldMap`` → 与 ``config/map.yaml`` 同 schema 的 JSON/YAML。

确定性（无随机）；沿用 t2 的合成场地发生器，另加"官方图纸型"（外框+灰线）输入的自动标定。
"""

from __future__ import annotations

import json
import pathlib
import re

import numpy as np
from PIL import Image

from ..field_map import (MAP_SCHEMA_KEYS, BlueFaceEstimate, build_field_map,
                         build_field_map_from_result, confidence_thresholds,
                         estimate_blue_face, motion_budget, parse_map_yaml,
                         reconcile_with_map_yaml, write_field_map)
from ..grid import (CORNER_ORDER, GridSpec, register_from_arena_frame,
                    register_from_corners)
from ..objects import FIELD_SPECS
from ..pipeline import PerceptionPipeline
from ..car_pose import CarSpec
from .synth import SYNTH_FIELD_PALETTE, render_arena

V1 = pathlib.Path(__file__).resolve().parents[4]
MAP_YAML = V1 / "dzrobot_v2" / "config" / "map.yaml"
OFFICIAL = V1 / "_tmp_session" / "t39_map_official" / "map_official.png"

CFG = None  # 延迟构造（避免导入期副作用）
TRUTH_OFFICIAL = {
    "mines": ["C1", "A3", "E3", "G4", "I6", "B7", "D9", "H10"],
    "obstacles": ["A1", "E2", "H4", "D8", "I8", "G9", "B11", "F11"],
    "targets": ["E5", "F5"],
    "bridges": ["K5", "K9"],
}


def _pipeline():
    from ..pipeline import PerceptionConfig
    return PerceptionPipeline(PerceptionConfig(
        palette=SYNTH_FIELD_PALETTE, object_specs=FIELD_SPECS,
        car_spec=CarSpec(cls="car", nose_cls="car_nose")))


def _scene(**kw):
    kw.setdefault("mines", ["C1", "A3", "E3"])
    kw.setdefault("obstacles", ["A1", "E2"])
    kw.setdefault("targets", ["E5", "F5"])
    kw.setdefault("deck_col", "K")
    kw.setdefault("arrows", [("K5", 90.0)])
    kw.setdefault("car", ("N10", 90.0))
    kw.setdefault("car_nose_offset", 0.16)
    return render_arena(GridSpec(), **kw)


# --------------------------------------------------------------------------- #
#  ① schema 对齐 / 出图
# --------------------------------------------------------------------------- #
def test_map_dict_keys_match_map_yaml_schema():
    arena = _scene()
    res = _pipeline().process(arena.image, corners_px=[arena.corners_px[t] for t in CORNER_ORDER])
    fm = build_field_map_from_result(res, name="单测图", start="N10", end="N5",
                                     barrier_col_from="D")
    d = fm.to_map_dict()
    assert set(MAP_SCHEMA_KEYS) <= set(d), (set(d), set(MAP_SCHEMA_KEYS))
    assert d["mines"] == ["A3", "C1", "E3"] and d["obstacles"] == ["A1", "E2"]
    assert d["targets"] == ["E5", "F5"] and d["bridges"] == ["K5"]
    assert isinstance(d["barrier_row_hi"], int) and d["start"] == "N10"
    assert "cell_m" not in d                      # 默认不塞额外键，保证与 map.yaml 可直接 diff


def test_json_and_yaml_round_trip_are_parsable():
    arena = _scene()
    res = _pipeline().process(arena.image, corners_px=[arena.corners_px[t] for t in CORNER_ORDER])
    fm = build_field_map_from_result(res, name="出图", start="N10", end="N5",
                                    barrier_col_from="D")
    js = json.loads(fm.to_json_text())
    assert set(js) == {"map", "perception"}
    assert js["perception"]["car"]["block"] == "N10"
    # YAML 文本必须能被"纯正则解析器"读回同一份 map 块（不依赖 PyYAML）
    back = parse_map_yaml(fm.to_yaml_text())
    for k in MAP_SCHEMA_KEYS:
        assert str(back[k]) == str(fm.to_map_dict()[k]), (k, back[k])
    assert len([l for l in fm.to_yaml_text().splitlines() if l.startswith("map:")]) == 1


def test_write_files(tmp_path=None):
    arena = _scene()
    res = _pipeline().process(arena.image, corners_px=[arena.corners_px[t] for t in CORNER_ORDER])
    fm = build_field_map_from_result(res)
    out = pathlib.Path(__file__).resolve().parent / "_out_t59"
    jp, yp = write_field_map(fm, out, stem="unit")
    try:
        assert jp.is_file() and yp.is_file()
        assert json.loads(jp.read_text(encoding="utf-8"))["map"]["targets"] == ["E5", "F5"]
    finally:
        for p in (jp, yp):
            p.unlink(missing_ok=True)
        try:
            out.rmdir()
        except OSError:
            pass


def test_confidence_threshold_keeps_low_confidence_out():
    arena = _scene()
    res = _pipeline().process(arena.image, corners_px=[arena.corners_px[t] for t in CORNER_ORDER])
    strict = build_field_map_from_result(res, name="x")
    strict_accept = {c: 0.999 for c in ("mine", "obstacle", "target", "bridge")}
    fm_strict = build_field_map(res.objects, res.registration, name="x", accept=strict_accept)
    assert strict.mines and not fm_strict.mines          # 阈值拉满 ⇒ 全被刷掉
    assert all(v >= confidence_thresholds("mine") for v in strict.confidence["mine"].values())


# --------------------------------------------------------------------------- #
#  ② 与真值对账（合成图用注入真值；官方图用 config/map.yaml）
# --------------------------------------------------------------------------- #
def test_reconcile_detects_exact_and_diff():
    arena = _scene(mines=["C1", "A3", "E3"], obstacles=["A1", "E2"],
                   targets=["E5", "F5"], deck_col="K", arrows=[])
    res = _pipeline().process(arena.image, corners_px=[arena.corners_px[t] for t in CORNER_ORDER])
    fm = build_field_map_from_result(res)
    truth = {"mines": ["A3", "C1", "E3"], "obstacles": ["A1", "E2"],
             "targets": ["E5", "F5"], "bridges": []}
    rec = fm.reconcile(truth)
    assert rec.ok and rec.n_diffs == 0, rec.report()
    bad = fm.reconcile(dict(truth, mines=["A3", "C1", "Z9"]))
    assert not bad.ok and bad.n_diffs == 2, bad.report()
    assert bad.per_class["mine"]["missing"] == ["Z9"] and bad.per_class["mine"]["extra"] == ["E3"]


def test_parse_map_yaml_reads_current_truth():
    if not MAP_YAML.is_file():
        return
    d = parse_map_yaml(MAP_YAML.read_text(encoding="utf-8"))
    assert d["start"] == "N10" and d["end"] == "N5"
    assert d["barrier_col_from"] == "D" and d["barrier_row_hi"] == 6 and d["barrier_row_lo"] == 7
    for k in ("mines", "obstacles", "targets", "bridges"):
        assert isinstance(d[k], list) and d[k], k
    GridSpec().block_to_index(d["mines"][0])          # 格名合法


# --------------------------------------------------------------------------- #
#  ③ 官方图纸：自动标定 + 逐格对账（真图，0 差异）
# --------------------------------------------------------------------------- #
def test_official_map_auto_registration():
    if not OFFICIAL.is_file():
        return
    img = np.asarray(Image.open(OFFICIAL).convert("RGB"))
    reg = register_from_arena_frame(img)
    assert reg.self_check()["ok"]
    assert reg.residual_px < 0.01
    assert "自动标定" in reg.notes[-1]
    # 四个起终点/无人机标注框落在 map.yaml 自记的格上（独立于感知的几何证据）
    for block in ("N10", "N5", "N8", "M2"):
        c = reg.cell_center_px(block)
        assert reg.px_to_block(*c) == block


def test_official_map_field_map_matches_config_map_yaml():
    """**真图端到端**：官方贴图 → 感知 → FieldMap → 与 config/map.yaml 逐条对账。"""
    if not (OFFICIAL.is_file() and MAP_YAML.is_file()):
        return
    img = np.asarray(Image.open(OFFICIAL).convert("RGB"))
    reg = register_from_arena_frame(img)
    # 官方图纸口径：色块 = 近满格方块（与 map_src.png 同类），用图纸调色板 + 图纸判据
    from ..colors import MAP_IMAGE
    from ..objects import MAP_IMAGE_SPECS, detect_field_objects
    objs = detect_field_objects(img, reg, MAP_IMAGE, MAP_IMAGE_SPECS)
    truth = parse_map_yaml(MAP_YAML.read_text(encoding="utf-8"))
    fm = build_field_map(objs, reg, name="official", start=truth["start"], end=truth["end"],
                         barrier_col_from=truth["barrier_col_from"],
                         barrier_row_hi=int(truth["barrier_row_hi"]),
                         barrier_row_lo=int(truth["barrier_row_lo"]),
                         image=OFFICIAL.name)
    rec = fm.reconcile(truth)
    assert rec.ok, rec.report()
    # 逐类都要有量（防止"两边都空所以相等"的假通过）
    for cls, key in (("mine", "mines"), ("obstacle", "obstacles"),
                     ("target", "targets"), ("bridge", "bridges")):
        assert len(fm.confidence.get(cls, {})) == len(truth[key]) > 0, (cls, fm.confidence)


# --------------------------------------------------------------------------- #
#  ④ 蓝面朝向 / 拖影预算 / 锚点
# --------------------------------------------------------------------------- #
def test_blue_face_unseen_on_nadir_view_is_unknown():
    """俯视帧里看不到竖直靶面 ⇒ 必须如实返回 unknown（不能猜）。"""
    arena = _scene(targets=["E5", "F5"])
    reg = register_from_corners([arena.corners_px[t] for t in CORNER_ORDER], GridSpec())
    est = estimate_blue_face(arena.image, reg, ["E5", "F5"])
    assert est.facing == "unknown" and est.confidence == 0.0
    assert "reason" in est.evidence


def test_blue_face_detects_when_face_is_visible():
    """合成一张"蓝面正对"的图 ⇒ 判 toward_us；红面 ⇒ away；两者都不足 ⇒ unknown。"""
    img = np.full((60, 60, 3), 245, dtype=np.uint8)
    img[20:40, 20:40] = (40, 90, 220)                 # 蓝面（H≈223°）
    arena = _scene(targets=["E5"], mines=[], obstacles=[], arrows=[], car=None)
    reg = register_from_corners([arena.corners_px[t] for t in CORNER_ORDER], GridSpec())
    tiny = np.asarray(Image.open(OFFICIAL).convert("RGB")) if OFFICIAL.is_file() else None
    # 直接构造"某格 ROI 内大片蓝色"的最小场景：把蓝块贴到 E5 的像素位置
    if tiny is not None:
        base = tiny.copy()
    else:
        base = np.full((520, 654, 3), 245, dtype=np.uint8)
    x0, y0, x1, y1 = (int(v) for v in (reg.cell_polygon_px("E5")[:, 0].min(),
                                       reg.cell_polygon_px("E5")[:, 1].min(),
                                       reg.cell_polygon_px("E5")[:, 0].max(),
                                       reg.cell_polygon_px("E5")[:, 1].max()))
    base[y0:y1, x0:x1] = (40, 90, 220)
    est = estimate_blue_face(base, reg, ["E5", "F5"])
    assert est.facing == "toward_us" and est.confidence > 0
    base[y0:y1, x0:x1] = (220, 50, 50)
    assert estimate_blue_face(base, reg, ["E5", "F5"]).facing == "away"
    base[y0:y1, x0:x1] = (245, 245, 245)
    assert estimate_blue_face(base, reg, ["E5", "F5"]).facing == "unknown"


def test_blue_face_guard_rejects_arena_frame_color():
    """★t59 实测踩到的假阳性回归：**场地外框是深蓝的**（官方图 (48,80,144) ⇒ H≈220°，
    正好落在"蓝面"色带里）。紧邻靶格的蓝框会被误判成"蓝面正对" ⇒ 守卫必须挡住：

    * ROI **不外扩**（``margin_frac=0``）；
    * 蓝/红块的**质心必须落在 ROI 中央 80%**，贴边的"蓝条"（外框）一律不算。
    """
    arena = _scene(targets=["E5"], mines=[], obstacles=[], arrows=[], car=None)
    reg = register_from_corners([arena.corners_px[t] for t in CORNER_ORDER], GridSpec())
    poly = reg.cell_polygon_px("E5")
    x0, y0 = int(poly[:, 0].min()), int(poly[:, 1].min())
    x1, y1 = int(poly[:, 0].max()), int(poly[:, 1].max())
    # ① 靶格上沿外侧贴一条"外框深蓝"带（+若 ROI 外扩就会吃到）⇒ 必须 unknown
    img = np.full_like(arena.image, 245)
    img[max(0, y0 - 12):y0, x0:x1] = (48, 80, 144)
    est = estimate_blue_face(img, reg, ["E5"], margin_frac=0.15)   # 显式外扩，考验质心守卫
    assert est.facing == "unknown", (est.facing, est.blue_frac)
    # ② 同一张图：把蓝块放到格子正中 ⇒ 必须判 toward_us（守卫不能把真信号也挡掉）
    img2 = np.full_like(arena.image, 245)
    img2[y0 + 8:y1 - 8, x0 + 8:x1 - 8] = (48, 80, 144)
    assert estimate_blue_face(img2, reg, ["E5"]).facing == "toward_us"


def test_motion_budget_numbers():
    b = motion_budget(0.12, 30.0, 0.002)
    assert abs(b["m_per_frame"] - 0.004) < 1e-9
    assert abs(b["px_per_frame"] - 2.0) < 1e-6
    b2 = motion_budget(0.12, 30.0, 0.0077)          # 官方图纸 1 px ≈ 7.7 mm
    assert 0.4 < b2["px_per_frame"] < 0.6


def test_anchors_and_car_pose_flow_into_json():
    arena = _scene()
    res = _pipeline().process(arena.image, corners_px=[arena.corners_px[t] for t in CORNER_ORDER])
    fm = build_field_map_from_result(res, anchors=["N10", "N5"])
    js = fm.to_json_dict()
    assert js["perception"]["anchors"] == ["N10", "N5"]
    assert js["perception"]["car"]["block"] == "N10"
    assert abs(js["perception"]["car"]["yaw_deg"] - 90.0) < 10.0
    assert js["perception"]["cell_confidence"]["N10"] > 0
