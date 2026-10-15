"""track_diag 纯函数单测 + 合成场景端到端（不开相机、不连 SSH）。

端到端思路：用 cv2.aruco.generateImageMarker 生成 id0~4 标记，按已知场地位姿
经合成单应 warp 到空白画面上（模拟俯拍相机），再跑"检测→H→车位姿解算"管线，
断言恢复的 (x, y, yaw) 与合成真值误差 <1cm/1°。
"""

from __future__ import annotations

import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
import cv2.aruco as aruco  # noqa: E402

REPO = Path(__file__).resolve().parents[1]


def load_module(name: str, file: Path):
    spec = importlib.util.spec_from_file_location(name, file)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


td = load_module("track_diag", REPO / "tools" / "track_diag.py")

STD_CORNERS_TEXT = "1:0,0 2:3.85,0 3:3.85,4.9 4:0,4.9"


# ---------------------------------------------------------------- 角度 / 格位

def test_wrap_angle():
    assert td.wrap_angle(0.3) == pytest.approx(0.3)
    assert td.wrap_angle(3.5) == pytest.approx(3.5 - 2 * math.pi)
    assert td.wrap_angle(-3.5) == pytest.approx(-3.5 + 2 * math.pi)
    assert td.wrap_angle(math.pi * 2) == pytest.approx(0.0)


def test_cell_to_field_contract():
    # 场地契约：N10 → x=(10.5-9)*0.35=0.525, y=(13.5-13)*0.35=0.175
    assert td.cell_to_field("N10") == pytest.approx((0.525, 0.175))
    assert td.cell_to_field("A1") == pytest.approx((3.675, 4.725))
    assert td.cell_to_field("N11") == pytest.approx((0.175, 0.175))
    with pytest.raises(ValueError):
        td.cell_to_field("Z9")
    with pytest.raises(ValueError):
        td.cell_to_field("A12")


def test_field_to_cell_roundtrip():
    for cell in ("N10", "A1", "G6", "B11"):
        x, y = td.cell_to_field(cell)
        assert td.field_to_cell(x, y) == cell


# ---------------------------------------------------------------- 角点配置

def test_parse_corners_text():
    got = td.parse_corners_text(STD_CORNERS_TEXT)
    assert got == td.DEFAULT_CORNERS
    with pytest.raises(ValueError):
        td.parse_corners_text("1:0,0 bad")


def test_load_corner_config(tmp_path):
    p = tmp_path / "c.json"
    p.write_text(json.dumps({"corners": {"1": [0, 0], "2": [3.85, 0],
                                         "3": [3.85, 4.9], "4": [0, 4.9]}}),
                 encoding="utf-8")
    assert td.load_corner_config(p) == td.DEFAULT_CORNERS


def test_validate_corners_standard_ok():
    assert td.validate_corners(td.DEFAULT_CORNERS) == []


def test_validate_corners_rejects_non_rectangle():
    skewed = {1: (0.0, 0.0), 2: (3.85, 0.0), 3: (3.0, 4.9), 4: (0.0, 4.9)}
    warns = td.validate_corners(skewed)
    assert warns  # 非矩形必须报警
    with pytest.raises(ValueError):
        td.validate_corners(skewed, strict=True)
    # 缺角也必报
    assert td.validate_corners({1: (0.0, 0.0)})


# ---------------------------------------------------------------- 单应 / yaw

def test_homography_fit_apply_rectangle():
    img = [(100, 100), (500, 100), (500, 700), (100, 700)]
    fld = [(0, 0), (3.85, 0), (3.85, 4.9), (0, 4.9)]
    H = td.homography_fit(img, fld)
    # 画面中心 → 场地中心
    assert td.homography_apply(H, (300, 400)) == pytest.approx((1.925, 2.45))
    # 四角还原（findHomography 内部有 float32 求解，精度给 1e-4 足够）
    for p, want in zip(img, fld):
        assert td.homography_apply(H, p) == pytest.approx(want, abs=1e-4)
    with pytest.raises(ValueError):
        td.homography_fit(img[:3], fld[:3])


def _img_quad(center, uvec, vvec, half):
    """TL,TR,BR,BL 顺序（与 detectMarkers 输出同序）的局部四边形。"""
    c = np.asarray(center, float)
    u, v = np.asarray(uvec, float), np.asarray(vvec, float)
    return np.array([c - u * half + v * half, c + u * half + v * half,
                     c + u * half - v * half, c - u * half - v * half])


def test_marker_yaw_from_corners():
    # 各向同性映射 img = field*100 + (50,50)
    H = td.homography_fit([(50, 50), (150, 50), (150, 150), (50, 150)],
                          [(0, 0), (1, 0), (1, 1), (0, 1)])
    yaw0 = math.radians(30)
    vvec = (math.cos(yaw0), math.sin(yaw0))            # 顶边指向（=场地 yaw 方向）
    uvec = (math.sin(yaw0), -math.cos(yaw0))           # 顶边向右
    c_field = np.array([0.5, 0.5])
    c_img = c_field * 100 + 50
    quad = _img_quad(c_img, uvec, vvec, half=10.0)
    assert td.marker_yaw_from_corners(H, quad) == pytest.approx(yaw0, abs=1e-6)
    # solve_car_pose 带贴歪修正
    x, y, yaw = td.solve_car_pose(H, quad, yaw_offset_deg=-30.0)
    assert (x, y) == pytest.approx((0.5, 0.5))
    assert yaw == pytest.approx(0.0, abs=1e-6)


# ---------------------------------------------------------------- Δ / reset 解析 / 采样器行

def test_compute_delta():
    d = td.compute_delta((1.0, 2.0, 0.0), (1.35, 2.0, math.radians(80)))
    assert d["dpos_m"] == pytest.approx(0.35)
    assert d["dpos_blocks"] == pytest.approx(1.0)
    assert d["dyaw_deg"] == pytest.approx(80.0)
    # yaw 环绕：真值 -170° vs 信念 +170° → Δ=+20°
    d2 = td.compute_delta((0, 0, math.radians(170)), (0, 0, math.radians(-170)))
    assert d2["dyaw_deg"] == pytest.approx(20.0)


def test_parse_reset():
    assert td.parse_reset("reset:N10,y") == ("N10", "y")
    assert td.parse_reset("reset: B3,x") == ("B3", "x")
    assert td.parse_reset("G8 A10 D2* E2,y") is None
    assert td.parse_reset("") is None
    assert td.parse_reset(None) is None


def test_parse_sampler_line_good_and_error():
    line = json.dumps({"t": 1.5, "belief": [1, 2, 0.5], "route": "reset:N10,y",
                       "pursuit": [0.1, 0.2], "cmdvel": None})
    obj = td.parse_sampler_line(line)
    assert obj["t"] == 1.5
    assert obj["belief"] == (1.0, 2.0, 0.5)
    assert obj["route"] == "reset:N10,y"
    assert obj["pursuit"] == (0.1, 0.2)
    assert obj["cmdvel"] is None and obj["error"] is None
    err = td.parse_sampler_line(json.dumps({"error": "rclpy 导入失败: x"}))
    assert err["error"].startswith("rclpy 导入失败")


def test_parse_sampler_line_tolerates_bad_rows():
    assert td.parse_sampler_line("") is None
    assert td.parse_sampler_line("not json at all") is None
    assert td.parse_sampler_line("{bad json") is None
    assert td.parse_sampler_line("[1,2,3]") is None
    # belief 形状不对 → belief 置 None，行仍可用
    obj = td.parse_sampler_line(json.dumps({"t": 1, "belief": [1, 2]}))
    assert obj is not None and obj["belief"] is None


# ---------------------------------------------------------------- 信念复算

def test_belief_tracker_reset_recompute():
    tr = td.BeliefTracker()
    cx, cy = td.cell_to_field("N10")
    tr.update(0.0, (1.5, 0.5, 0.10), "reset:N10,y")
    snap = tr.snapshot()
    assert snap["reset"] == ("N10", "y")
    # 复算信念 = 锚格中心 + 断言向
    assert snap["est"] == pytest.approx((cx, cy, math.pi / 2))
    # 裸信念 = block×0.35、yaw+π/2
    assert snap["raw"] == pytest.approx((0.525, 0.175, 0.10 + math.pi / 2))
    # 同一 reset 话题是 latch 重发，不得重复锚定；位移增量叠加
    tr.update(1.0, (2.5, 0.9, 0.15), "reset:N10,y")
    snap = tr.snapshot()
    assert snap["est"] == pytest.approx((cx + 0.35, cy + 0.4 * 0.35, math.pi / 2))
    # 普通路线命令不动锚
    tr.update(2.0, (3.0, 0.9, 0.2), "G8 A10,y")
    snap = tr.snapshot()
    assert snap["route"] == "G8 A10,y"
    assert snap["est"] == pytest.approx((cx + 1.5 * 0.35, cy + 0.4 * 0.35,
                                         math.pi / 2))


def test_belief_tracker_new_reset_reanchors():
    tr = td.BeliefTracker()
    tr.update(0.0, (1.0, 1.0, 0.0), "reset:N10,y")
    tr.update(1.0, (2.0, 1.0, 0.0), "reset:B3,x")  # 新 reset → 重新锚定
    cx, cy = td.cell_to_field("B3")
    snap = tr.snapshot()
    assert snap["est"] == pytest.approx((cx, cy, 0.0))


# ---------------------------------------------------------------- 规则引擎

def _drive(engine, samples, dt=0.1):
    """按脚本化时间序列驱动规则引擎，返回全部告警。"""
    fired = []
    for i, (belief, truth) in enumerate(samples):
        t = i * dt
        delta = td.compute_delta(belief, truth) \
            if belief is not None and truth is not None else None
        fired.extend(engine.update(t, belief, truth, delta))
    return fired


def test_rule_heading_bias_fires_once_with_cooldown():
    e = td.RuleEngine()
    samples = [((0, 0, 0.0), (0, 0, math.radians(20)))] * 65  # 6.5s 持续 20°
    fired = _drive(e, samples)
    hb = [a for a in fired if a["rule"] == "HEADING_BIAS"]
    # 2s 处命中一次；冷却 5s 内不重复（第 2 次最早 t=7s，超出窗口）
    assert len(hb) == 1
    assert hb[0]["t"] == pytest.approx(2.0, abs=0.15)
    assert "画圆前兆" in hb[0]["msg"]


def test_rule_heading_bias_not_fired_when_small():
    e = td.RuleEngine()
    samples = [((0, 0, 0.0), (0, 0, math.radians(5)))] * 40
    assert _drive(e, samples) == []


def test_rule_pos_drift():
    e = td.RuleEngine()
    # 真值比信念偏 0.5m ≈ 1.43 格，持续 3s
    samples = [((0, 0, 0.0), (0.5, 0, 0.0))] * 30
    rules = {a["rule"] for a in _drive(e, samples)}
    assert "POS_DRIFT" in rules
    # 偏差 0.1m ≈ 0.29 格则不报
    e2 = td.RuleEngine()
    samples2 = [((0, 0, 0.0), (0.1, 0, 0.0))] * 30
    assert _drive(e2, samples2) == []


def test_rule_odom_stall():
    e = td.RuleEngine()
    # 真值以 0.1m/s 前进，信念钉死不动
    samples = [((0, 0, 0.0), (0.1 * i * 0.1, 0, 0.0)) for i in range(30)]
    rules = {a["rule"] for a in _drive(e, samples)}
    assert "ODOM_STALL" in rules
    assert "ACTUATOR_DEAD" not in rules


def test_rule_actuator_dead():
    e = td.RuleEngine()
    # 信念在动（0.1m/s），真值钉死
    samples = [((0.1 * i * 0.1, 0, 0.0), (0, 0, 0.0)) for i in range(30)]
    rules = {a["rule"] for a in _drive(e, samples)}
    assert "ACTUATOR_DEAD" in rules
    assert "ODOM_STALL" not in rules


def test_rule_circling():
    e = td.RuleEngine()
    # 真值每拍转 35°、位置基本不动（1mm 抖动），信念静止
    samples = [((0, 0, 0.0), (0.001 * (i % 2), 0, math.radians(35) * i))
               for i in range(30)]
    rules = {a["rule"] for a in _drive(e, samples)}
    assert "CIRCLING" in rules


def test_rule_no_signal():
    e = td.RuleEngine()
    fired = _drive(e, [(None, None)] * 40)  # 4s 全静默
    rules = {a["rule"] for a in fired}
    assert rules == {"NO_TRUTH", "NO_BELIEF"}


def test_rule_no_signal_respects_expect_flags():
    e = td.RuleEngine()
    e.expect_belief = False  # --no-car 时不该报 NO_BELIEF
    fired = _drive(e, [(None, (0, 0, 0.0))] * 40)
    assert fired == []


def test_rule_healthy_tracking_silent():
    e = td.RuleEngine()
    # 信念与真值一起正常前进 4s → 零告警
    samples = [((0.05 * i * 0.1, 0, 0.0), (0.05 * i * 0.1 + 0.02, 0, 0.01))
               for i in range(40)]
    assert _drive(e, samples) == []


# ---------------------------------------------------------------- 合成场景端到端

def _render_marker(canvas, Hinv, mid, x, y, yaw, side_m):
    """把带安静区的标记按场地位姿 (x,y,yaw) warp 到画面上（模拟相机成像）。"""
    d = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    core, quiet = 200, 50
    tex = aruco.generateImageMarker(d, mid, core)
    tex = cv2.copyMakeBorder(tex, quiet, quiet, quiet, quiet,
                             cv2.BORDER_CONSTANT, value=255)
    tex = cv2.cvtColor(tex, cv2.COLOR_GRAY2BGR)
    th, tw = tex.shape[:2]
    half = side_m * (1 + 2 * quiet / core) / 2  # 含安静区的半边长（米）
    vvec = np.array([math.cos(yaw), math.sin(yaw)])   # 顶边指向 = 标记 yaw
    uvec = np.array([math.sin(yaw), -math.cos(yaw)])  # 顶边向右
    quad_field = _img_quad(np.array([x, y]), uvec, vvec, half)

    def to_img(p):
        v = Hinv @ np.array([p[0], p[1], 1.0])
        return v[:2] / v[2]

    quad_img = np.array([to_img(p) for p in quad_field], np.float32)
    src = np.array([[0, 0], [tw, 0], [tw, th], [0, th]], np.float32)
    M = cv2.getPerspectiveTransform(src, quad_img)
    size = (canvas.shape[1], canvas.shape[0])
    warped = cv2.warpPerspective(tex, M, size)
    mask = cv2.warpPerspective(np.full((th, tw), 255, np.uint8), M, size)
    canvas[mask > 0] = warped[mask > 0]


def test_endtoend_synthetic_scene():
    # 画面 1450x1750，四角标记贴标准场角（像素位置带透视感）
    img_pts = {1: (120, 1620), 2: (1330, 1590), 3: (1290, 110), 4: (90, 140)}
    H_exact = td.homography_fit([img_pts[i] for i in (1, 2, 3, 4)],
                                [td.DEFAULT_CORNERS[i] for i in (1, 2, 3, 4)])
    Hinv = np.linalg.inv(H_exact)
    canvas = np.full((1750, 1450, 3), 255, np.uint8)
    for i in (1, 2, 3, 4):
        fx, fy = td.DEFAULT_CORNERS[i]
        _render_marker(canvas, Hinv, i, fx, fy, yaw=0.3 * i, side_m=0.15)
    car_x, car_y, car_yaw = 1.2, 2.0, math.radians(35)
    _render_marker(canvas, Hinv, 0, car_x, car_y, car_yaw, side_m=0.18)
    _render_marker(canvas, Hinv, 5, td.FIELD_W_M / 2, td.FIELD_H_M / 2,
                   yaw=0.0, side_m=0.15)

    # 管线：检测（自动探字典）→ H → 车位姿
    dets, locked = td.detect_with_autodict(canvas)
    assert locked == "DICT_4X4_50"
    assert set(dets) >= {0, 1, 2, 3, 4, 5}
    H = td.corners_homography(dets, td.DEFAULT_CORNERS)
    assert H is not None
    x, y, yaw = td.solve_car_pose(H, dets[0])
    assert math.hypot(x - car_x, y - car_y) < 0.01          # <1cm
    assert abs(math.degrees(td.wrap_angle(yaw - car_yaw))) < 1.0  # <1°
    # 中心校验标记投影误差也应 <1cm
    cx, cy = td.homography_apply(H, dets[5].mean(axis=0))
    assert math.hypot(cx - td.FIELD_W_M / 2, cy - td.FIELD_H_M / 2) < 0.01


# ---------------------------------------------------------------- print-markers 回读

def test_print_markers_roundtrip(tmp_path):
    rc = td.main(["print-markers", str(tmp_path)])
    assert rc == 0
    d = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    det = aruco.ArucoDetector(d, aruco.DetectorParameters())
    found = set()
    pngs = sorted(tmp_path.glob("*.png"))
    assert len(pngs) == 6
    for png in pngs:
        img = cv2.imread(str(png), cv2.IMREAD_GRAYSCALE)
        assert img is not None
        _, ids, _ = det.detectMarkers(img)
        assert ids is not None and len(ids) == 1
        found.add(int(ids.ravel()[0]))
    assert found == {0, 1, 2, 3, 4, 5}


def test_main_rejects_unknown_subcommand():
    assert td.main(["bogus"]) == 2
