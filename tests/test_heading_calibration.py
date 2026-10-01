"""heading_calibration：固定朝向偏置的探测与闭环修正。"""

import math

import pytest

from ag_integrate.heading_calibration import (
    ProbeResult,
    closed_loop_rate,
    corrected_heading,
    estimate_bias,
    wrap_pi,
    yaw_from_quaternion,
)


def test_wrap_pi():
    assert wrap_pi(0.0) == 0.0
    assert wrap_pi(3 * math.pi) == pytest.approx(math.pi)
    assert wrap_pi(-3 * math.pi) == pytest.approx(math.pi)
    assert wrap_pi(2 * math.pi + 0.5) == pytest.approx(0.5)
    assert wrap_pi(-2 * math.pi - 0.5) == pytest.approx(-0.5)
    assert -math.pi < wrap_pi(5.0) <= math.pi


def test_yaw_from_quaternion_known_angles():
    # yaw = +90°：绕 z 轴的四元数 z=w=sin/cos(pi/4)
    s = math.sin(math.pi / 4)
    assert yaw_from_quaternion(0.0, 0.0, s, s) == pytest.approx(math.pi / 2)
    assert yaw_from_quaternion(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0)
    assert yaw_from_quaternion(0.0, 0.0, -s, s) == pytest.approx(-math.pi / 2)


def test_estimate_bias_real_68_6_case():
    # reset 断言朝向 +y = 90°，车实际朝 158.6°（实测 δ=+68.6°）
    expected = math.radians(90.0)
    true = math.radians(158.6)
    distance = 0.3
    start = (1.0, 2.0)
    end = (start[0] + distance * math.cos(true), start[1] + distance * math.sin(true))

    result = estimate_bias(expected, start, end, min_distance=0.2)
    assert isinstance(result, ProbeResult)
    assert math.degrees(result.bias) == pytest.approx(68.6, abs=1e-9)
    assert result.distance == pytest.approx(distance)


def test_estimate_bias_negative_offset():
    expected = math.radians(90.0)
    true = math.radians(21.4)   # δ = -68.6°
    p0 = (0.0, 0.0)
    p1 = (0.3 * math.cos(true), 0.3 * math.sin(true))
    result = estimate_bias(expected, p0, p1)
    assert result is not None
    assert math.degrees(result.bias) == pytest.approx(-68.6, abs=1e-9)


def test_short_probe_is_rejected():
    # 只动了 5 cm：不可信，返回 None 而不是硬给一个补偿
    assert estimate_bias(1.0, (0.0, 0.0), (0.05, 0.0), min_distance=0.2) is None


def test_corrected_heading_cancels_bias():
    bias = math.radians(68.6)
    turn = corrected_heading(bias)
    assert math.degrees(turn) == pytest.approx(-68.6)
    # 叠加后残差为 0
    assert wrap_pi(bias + turn) == pytest.approx(0.0, abs=1e-12)


def test_closed_loop_rate_sign_capping_and_deadzone():
    target = math.radians(90.0)
    # 偏 +68.6°：需要正角速度，且不超过 max_rate
    current = math.radians(21.4)
    rate = closed_loop_rate(current, target, max_rate=0.5, tolerance=math.radians(3))
    assert rate > 0
    assert rate <= 0.5
    # 偏 -90°：大误差时限幅
    rate = closed_loop_rate(math.radians(180.0), target, max_rate=0.5, tolerance=0.05)
    assert rate == pytest.approx(-0.5)
    # 进入容差区：停止
    rate = closed_loop_rate(math.radians(89.0), target, max_rate=0.5, tolerance=math.radians(3))
    assert rate == 0.0


def test_full_calibration_converges():
    """端到端：测出偏置 -> 反转 -> 车头指向断言朝向。"""
    expected = math.radians(90.0)
    true_heading = math.radians(158.6)
    p0 = (0.0, 0.0)
    p1 = (0.3 * math.cos(true_heading), 0.3 * math.sin(true_heading))
    probe = estimate_bias(expected, p0, p1)
    assert probe is not None

    target = true_heading + corrected_heading(probe.bias)
    # 模拟闭环若干拍：每拍按 rate 转向
    yaw = true_heading
    for _ in range(200):
        rate = closed_loop_rate(yaw, target, max_rate=0.3, tolerance=math.radians(2))
        if rate == 0.0:
            break
        yaw = wrap_pi(yaw + rate * 0.1)
    assert abs(wrap_pi(yaw - expected)) <= math.radians(2)
