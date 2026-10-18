# -*- coding: utf-8 -*-
"""dznav 单元测试（纯算法，Windows 无 ROS 直接跑）。"""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))   # ros_stub 所在目录
import ros_stub  # noqa: E402  必须先于 dznav 导入

from dznav.contract import (BLOCK, block_to_meters, wrap_pi, in_field,  # noqa: E402
                            RE_BLOCK, RE_HEADING)
from dznav.servo import ServoModel  # noqa: E402
from dznav.refpath import (Waypoint, build_reference, PathOptions)  # noqa: E402
from dznav.pursuit import Tracker, TrackerConfig  # noqa: E402
from dznav.fusion import Localizer  # noqa: E402


# ================================================================ contract
class TestContract:
    def test_block_to_meters_n10(self):
        # N10 = 起点（1.5, 0.5 格）
        assert block_to_meters("N10") == pytest.approx(
            (1.5 * BLOCK, 0.5 * BLOCK), abs=1e-9)

    def test_block_to_meters_a1(self):
        # A1 = 对角格（10.5, 13.5 格）
        assert block_to_meters("A1") == pytest.approx(
            (10.5 * BLOCK, 13.5 * BLOCK), abs=1e-9)

    def test_invalid_block(self):
        with pytest.raises(ValueError):
            block_to_meters("Z1")
        with pytest.raises(ValueError):
            block_to_meters("A12")   # 行越界

    def test_grammar(self):
        assert RE_BLOCK.match("D2*").group(3) == "*"
        assert RE_BLOCK.match("M10").group(3) == ""
        assert RE_HEADING.match("-y")
        assert not RE_HEADING.match("z")

    def test_in_field(self):
        assert in_field(1.0, 1.0)
        assert not in_field(-0.61, 1.0, margin=0.60)
        assert in_field(-0.59, 1.0, margin=0.60)


# ================================================================ servo
class TestServo:
    def test_ideal_roundtrip(self):
        sv = ServoModel(steer_limit_deg=30.0, wheelbase=0.157)
        for d in (-25.0, -10.0, 0.0, 10.0, 25.0):
            k = sv.kappa_for_command(d)
            assert sv.command_for_kappa(k) == pytest.approx(d, abs=1e-6)

    def test_fxxy_asymmetry(self):
        sv = ServoModel(mode="fxxy", steer_limit_deg=30.0, center_deg=-1.0)
        assert sv.kappa_for_command(30.0) == pytest.approx(6.6, abs=0.01)
        assert sv.kappa_for_command(-30.0) == pytest.approx(-2.3, abs=0.01)
        assert sv.kappa_for_command(-1.0) == pytest.approx(0.0, abs=1e-9)
        # 右转 κ=-2.0 的前馈应远比左转同幅值更"满舵"
        assert sv.command_for_kappa(-2.0) < -1.0 - 2.0 / 6.6 * 31.0
        assert sv.kappa_max(-1.0) == pytest.approx(2.3, abs=0.01)
        assert sv.kappa_max(+1.0) == pytest.approx(6.6, abs=0.01)

    def test_command_clamp(self):
        sv = ServoModel(steer_limit_deg=30.0)
        assert abs(sv.command_for_kappa(100.0)) <= 90.0   # atan 有界


# ================================================================ refpath
def _corner_path(sign=+1.0, dev=0.08):
    """1m 直行 -> 90° 弯 -> 1m 直行。sign=+1 左转（+y），-1 右转。"""
    sv = ServoModel(steer_limit_deg=30.0)
    wps = [Waypoint(0.0, 0.0), Waypoint(1.0, 0.0),
           Waypoint(1.0, sign * 1.0)]
    return build_reference(wps, sv, PathOptions(dev_cap=dev)), sv


class TestRefPath:
    def test_90deg_fillet_geometry(self):
        rp, sv = _corner_path(+1.0)
        cs = [c for c in rp.corners if c.mode == "fillet"]
        assert len(cs) == 1
        c = cs[0]
        assert c.turn_sign > 0
        # 物理下限生效：R >= R_phys（30° 舵角 κ_max=3.677 → R>=0.272）
        assert c.radius >= 1.0 / sv.kappa_max(+1.0) - 1e-9
        # 顶点偏差 = R(1/cos45° - 1) ≤ 失格线 17.5cm
        assert c.dev < 0.175
        # 弧长坐标单调
        ss = [s for s, *_ in rp.samples]
        assert all(b > a for a, b in zip(ss, ss[1:]))
        # 总长 = 2*(1 - t) + R*π/2（t = R·tan45° = R）
        expect = 2 * (1 - c.radius) + c.radius * math.pi / 2
        assert rp.length == pytest.approx(expect, abs=0.05)
        # 弧上 κ 符号 = 左转正
        karc = [k for _s, _x, _y, _th, k in rp.samples if abs(k) > 1e-6]
        assert karc and all(k > 0 for k in karc)

    def test_vertex_pass_through_deviation(self):
        rp, _ = _corner_path(-1.0)
        c = [x for x in rp.corners if x.mode == "fillet"][0]
        arc_pts = [(x, y) for _s, x, y, _th, k in rp.samples if abs(k) > 1e-6]
        assert arc_pts, "应有圆弧样本"

        def seg_dist(px, py, ax, ay, bx, by):
            dx, dy = bx - ax, by - ay
            t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy)
                             / (dx * dx + dy * dy)))
            return math.hypot(px - ax - t * dx, py - ay - t * dy)

        # 1) 弧到顶点最近距离 ≈ c.dev（切角量，保守度量）
        dmin = min(math.hypot(x - 1.0, y - 0.0) for x, y in arc_pts)
        assert dmin == pytest.approx(c.dev, abs=0.02)
        # 2) 对理想折线（两段腿）的最大横向偏差 < 17.5cm 失格线
        dev_line = max(min(seg_dist(x, y, 0, 0, 1, 0),
                           seg_dist(x, y, 1, 0, 1, -1))
                       for x, y in arc_pts)
        assert dev_line < 0.175
        assert dev_line == pytest.approx(
            c.radius * (1 - math.cos(math.pi / 4)), abs=0.02)

    def test_straight_through_dedupe(self):
        sv = ServoModel(steer_limit_deg=30.0)
        wps = [Waypoint(0, 0), Waypoint(0.5, 0), Waypoint(1.0, 0.0)]
        rp = build_reference(wps, sv, PathOptions())
        assert all(c.mode == "pass" for c in rp.corners)
        assert rp.length == pytest.approx(1.0, abs=0.02)

    def test_arc_fallback_u_turn(self):
        sv = ServoModel(steer_limit_deg=30.0)
        # 180° 掉头：右转
        wps = [Waypoint(0, 0), Waypoint(1, 0), Waypoint(0, 0)]
        rp = build_reference(wps, sv, PathOptions())
        arcs = [c for c in rp.corners if c.mode == "arc"]
        assert len(arcs) == 1
        a = arcs[0]
        assert math.degrees(a.angle) > 170
        # 出口点在第二段上（从 (1,0) 回向 (0,0) 方向）
        assert a.exit_x < 1.0 and abs(a.exit_y) < 1e-9

    def test_critical_waypoint_tightens(self):
        sv = ServoModel(steer_limit_deg=35.0)   # κ_max=4.30
        # 45° 弯：dev 预算可满足（R_dev(3cm, 45°) = 0.03/0.0824 = 0.364 > R_phys）
        wps = [Waypoint(0, 0), Waypoint(1, 0, critical=True),
               Waypoint(1.5, 0.5)]
        rp = build_reference(wps, sv, PathOptions(dev_cap=0.08,
                                                  dev_cap_critical=0.03))
        c = [x for x in rp.corners if x.mode == "fillet"][0]
        # 关键点弯：R 被 3cm 预算限制（0.364）或腿长限制
        assert c.radius <= 0.364 + 1e-6

    def test_project_cte_sign(self):
        rp, _ = _corner_path(+1.0)
        # 车在路径左侧 0.1m（首直段中点）
        pr = rp.project(0.4, 0.1, 0.0)
        assert pr["cte"] == pytest.approx(0.1, abs=0.02)
        # 右侧为负
        pr = rp.project(0.4, -0.1, 0.0)
        assert pr["cte"] == pytest.approx(-0.1, abs=0.02)


# ================================================================ pursuit
def _straight_tracker():
    sv = ServoModel(steer_limit_deg=30.0)
    wps = [Waypoint(0, 0), Waypoint(3, 0)]
    rp = build_reference(wps, sv, PathOptions())
    return Tracker(rp, sv, TrackerConfig()), sv


class TestPursuit:
    def test_steer_signs_on_straight(self):
        tr, _sv = _straight_tracker()
        # 车在左侧 0.1m、朝向正 → 应右打（负）
        _c, ev = tr.update(0.3, 0.1, 0.0, 0.02)
        assert _c.steer_deg < 0
        # 车朝向偏右（th=-0.2）→ 应左打（正）
        tr2, _ = _straight_tracker()
        _c, _ = tr2.update(0.3, 0.0, -0.2, 0.02)
        assert _c.steer_deg > 0

    def test_speed_ramp_and_arrival(self):
        tr, _ = _straight_tracker()
        v0 = None
        cmd, ev = tr.update(0.05, 0.0, 0.0, 0.02)
        v0 = cmd.v
        assert v0 > 0
        # 终点渐近：接近末尾速度下降并到点
        v_near = None
        for _ in range(4000):
            x = tr.s_prog + 0.05
            cmd, ev = tr.update(min(x, 2.95), 0.0, 0.0, 0.02)
            if any(e["ev"] == "arrived" for e in ev):
                break
            if tr.s_prog > 2.5:
                v_near = cmd.v
        assert tr.arrived
        assert v_near is None or v_near <= tr.cfg.v_max

    def test_corner_feedforward_left(self):
        sv = ServoModel(steer_limit_deg=30.0)
        wps = [Waypoint(0, 0), Waypoint(1, 0), Waypoint(1, 1)]
        rp = build_reference(wps, sv, PathOptions())
        tr = Tracker(rp, sv, TrackerConfig())
        # 车从起点沿路径推进到弯道入口（切点前），参考曲率>0 → 前馈为正
        x = 0.0
        saw_pos_steer = False
        for _ in range(60):
            x = min(x + 0.03, 1.0 - 0.10)
            cmd, _ev = tr.update(x, 0.0, 0.0, 0.02)
            if x > 1.0 - 0.35 and cmd.steer_deg > 0:
                saw_pos_steer = True
                break
        assert saw_pos_steer, "接近左弯入口应出现正向前馈"

    def test_shoot_stop_pause_resume(self):
        sv = ServoModel(steer_limit_deg=30.0)
        wps = [Waypoint(0, 0), Waypoint(1.5, 0, label="D2", shoot_stop=True),
               Waypoint(3, 0)]
        rp = build_reference(wps, sv, PathOptions())
        tr = Tracker(rp, sv, TrackerConfig())
        saw_stop = False
        for _ in range(3000):
            cmd, ev = tr.update(tr.s_prog + 0.05, 0.0, 0.0, 0.02)
            if any(e["ev"] == "shoot_stop" for e in ev):
                saw_stop = True
                break
        assert saw_stop
        assert tr.paused
        cmd, _ = tr.update(1.5, 0.0, 0.0, 0.02)
        assert cmd.v == 0.0
        tr.resume()
        assert not tr.paused

    def test_rebuild_on_huge_cte(self):
        tr, _ = _straight_tracker()
        # 车横偏 0.5m（> recover_cte 0.3）→ 重建事件
        _c, ev = tr.update(0.5, 0.5, 0.0, 0.02)
        assert any(e["ev"] == "rebuild" for e in ev)
        assert tr.rebuilds == 1
        assert tr.path.length < 3.0    # 重建为当前位置到终点直连

    def test_arc_mechanism(self):
        sv = ServoModel(steer_limit_deg=30.0)
        wps = [Waypoint(0, 0), Waypoint(1, 0), Waypoint(0, 0)]   # 180°掉头
        rp = build_reference(wps, sv, PathOptions())
        tr = Tracker(rp, sv, TrackerConfig())
        tr.set_travel(0.0)
        # 车"到达"掉头顶点 (1,0)，朝向 +x；掉头机动应把车转到 -x（exit_th=π）
        corner = [c for c in rp.corners if c.mode == "arc"][0]
        tr._enter_arc(corner, 1.0, 0.0, 0.0, 0.0, [], at_vertex=True)
        assert tr.mode == "arc"
        th = 0.0
        exited = False
        for i in range(1, 600):
            # 满舵 28°、0.1 m/s 的名义角速度（理想模型）旋转航向
            w = 0.10 / sv.wheelbase * math.tan(math.radians(28.0))
            th += w * 0.02
            tr.set_travel(0.10 * 0.02 * i)
            _cmd, ev = tr.update(1.0, 0.0, th, 0.02, now=i * 0.02)
            if any(e["ev"] == "arc_exit" for e in ev):
                exited = True
                assert tr.mode == "track"
                # 途经点推进到掉头顶点之后
                assert tr.passed >= corner.idx
                break
        assert exited, "ARC 机动应按 yaw 判据完成"


# ================================================================ fusion
class TestFusion:
    def test_dr_straight(self):
        loc = Localizer()
        # IMU: 车真朝 +x（东）。yaw_rel 满足 th = yaw_rel + π/2 = 0 → yaw_rel = -π/2
        loc.on_imu(-math.pi / 2)
        for i in range(1, 101):
            loc.on_odom_raw(i * 0.01, 0.0)
        x, y, th = loc.pose
        # 起点在 N10 格心 (0.525, 0.175)，东行 1m
        assert x == pytest.approx(0.525 + 1.0, abs=0.01)
        assert y == pytest.approx(0.175, abs=1e-6)
        assert abs(th) < 1e-6

    def test_yaw_out_contract(self):
        loc = Localizer()
        loc.on_imu(0.3)
        # 下游 yaw_out + π/2 == 内部 th
        assert wrap_pi(loc.yaw_out() + math.pi / 2) == pytest.approx(
            loc.th, abs=1e-9)

    def test_anchor_soft_pull(self):
        loc = Localizer()
        loc.on_imu(-math.pi / 2)
        for i in range(1, 51):
            loc.on_odom_raw(i * 0.01, 0.0)
        x0, _y0, _ = loc.pose
        # 软锚点拉回一半（blend=0.5）
        loc.anchor(x0 + 0.10, 0.0, 0.0, blend=0.5, learn=False)
        x1, _y1, _ = loc.pose
        assert x1 == pytest.approx(x0 + 0.05, abs=1e-6)

    def test_reset_hard(self):
        loc = Localizer()
        loc.on_imu(0.1)
        loc.reset(1.0, 2.0, math.pi / 2)
        x, y, th = loc.pose
        assert (x, y) == (1.0, 2.0)
        assert th == pytest.approx(math.pi / 2, abs=1e-9)
        # reset 后 IMU 读数不变 → 信念航向不变；物理转动 → 信念跟随
        loc.on_imu(0.1)
        assert loc.th == pytest.approx(math.pi / 2, abs=1e-9)
        loc.on_imu(0.3)
        assert loc.th == pytest.approx(math.pi / 2 + 0.2, abs=1e-9)

    def test_stationary_heading_hold(self):
        loc = Localizer()
        t = 0.0
        loc.on_imu(-math.pi / 2)
        loc.on_odom_raw(0.0, 0.0)
        loc.on_odom_raw(0.05, 0.0)     # 动一下
        loc.tick(t)
        th0 = loc.th
        # 静止后 IMU 缓慢漂移（陀螺零漂）：保持观测把航向锚住
        for i in range(1, 200):
            t += 0.02
            loc.tick(t)
            loc.on_imu(-math.pi / 2 + 0.0001 * i)
        assert abs(loc.th - th0) < 0.01

    def test_scale_learning_smoke(self):
        loc = Localizer()
        loc.on_imu(-math.pi / 2)
        # 模拟编码器尺度偏小 2%：真走 0.98m，raw 只记 0.9604
        raw = 0.0
        for i in range(1, 99):
            raw = 0.98 * i * 0.01
            loc.on_odom_raw(raw, 0.0)
        # 锚点告诉它真实位置：残差 ~2% → enc_scale 上调（慢增益）
        loc.anchor(0.525 + 0.98, 0.175, 0.0, blend=0.0, learn=True)
        assert loc.enc_scale > 1.0
        assert loc.enc_scale < 1.03    # 慢增益 + 钳位


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
