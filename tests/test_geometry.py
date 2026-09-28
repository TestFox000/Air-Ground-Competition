import math

import pytest

from ag_core.geometry import clamp_gimbal, focal_pixels, pixel_to_gimbal_delta


def test_focal_pixels_matches_fov():
    f = focal_pixels(640, 60.0)
    assert f == pytest.approx((640 / 2.0) / math.tan(math.radians(30.0)))


def test_center_pixel_gives_zero_delta():
    yaw, pitch = pixel_to_gimbal_delta(320.0, 240.0, 640, 480, 60.0)
    assert yaw == pytest.approx(0.0, abs=1e-9)
    assert pitch == pytest.approx(0.0, abs=1e-9)


def test_right_of_center_needs_positive_yaw():
    yaw, _ = pixel_to_gimbal_delta(640.0, 240.0, 640, 480, 60.0)
    assert yaw > 0.0


def test_left_of_center_needs_negative_yaw():
    yaw, _ = pixel_to_gimbal_delta(0.0, 240.0, 640, 480, 60.0)
    assert yaw < 0.0


def test_upper_half_needs_positive_pitch():
    _, pitch = pixel_to_gimbal_delta(320.0, 0.0, 640, 480, 60.0)
    assert pitch > 0.0


def test_lower_half_needs_negative_pitch():
    _, pitch = pixel_to_gimbal_delta(320.0, 480.0, 640, 480, 60.0)
    assert pitch < 0.0


def test_edge_of_fov_is_half_fov():
    """画面最右侧的偏角应接近半个水平视场角。"""
    yaw, _ = pixel_to_gimbal_delta(640.0, 240.0, 640, 480, 60.0)
    assert yaw == pytest.approx(30.0, abs=0.5)


def test_narrower_fov_gives_smaller_delta():
    """窄视场 = 长焦：同样的像素偏移对应的角度增量更小。"""
    yaw_wide, _ = pixel_to_gimbal_delta(400.0, 240.0, 640, 480, 60.0)
    yaw_narrow, _ = pixel_to_gimbal_delta(400.0, 240.0, 640, 480, 40.0)
    assert 0.0 < yaw_narrow < yaw_wide


def test_clamp_gimbal_limits():
    assert clamp_gimbal(200.0, 0.0) == (180.0, 0.0)
    assert clamp_gimbal(-999.0, 0.0) == (-180.0, 0.0)
    assert clamp_gimbal(0.0, 90.0) == (0.0, 60.0)
    assert clamp_gimbal(0.0, -90.0) == (0.0, -30.0)


def test_clamp_gimbal_keeps_in_range_values():
    assert clamp_gimbal(15.0, 12.0) == (15.0, 12.0)


def test_clamp_gimbal_custom_limits():
    assert clamp_gimbal(50.0, 0.0, pan_limit=30.0) == (30.0, 0.0)
