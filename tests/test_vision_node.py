"""vision_node 的图像解码测试（用 ros_stub，无需 ROS / cv2）。"""

import numpy as np
import pytest

import ros_stub  # noqa: F401
from sensor_msgs.msg import Image  # noqa: E402

from ag_perception.vision_node import image_to_bgr  # noqa: E402


def img(encoding: str, data: bytes, w: int, h: int) -> Image:
    return Image(encoding=encoding, data=data, width=w, height=h)


def test_bgr8_passthrough():
    data = bytes([1, 2, 3, 4, 5, 6])
    out = image_to_bgr(img("bgr8", data, 2, 1))
    assert out.shape == (1, 2, 3)
    assert out[0, 0].tolist() == [1, 2, 3]


def test_rgb8_is_converted_to_bgr():
    data = bytes([1, 2, 3])          # 一个 RGB 像素
    out = image_to_bgr(img("rgb8", data, 1, 1))
    assert out[0, 0].tolist() == [3, 2, 1]


def test_mono8_becomes_three_channels():
    out = image_to_bgr(img("mono8", bytes([10, 20]), 2, 1))
    assert out.shape == (1, 2, 3)
    assert out[0, 1].tolist() == [20, 20, 20]


def test_yuyv_shape_and_range():
    """YUYV 每 4 字节描述 2 个像素：Y0 U Y1 V。"""
    w, h = 4, 1
    packed = bytes([100, 128, 150, 128] * (w // 2))
    out = image_to_bgr(img("yuyv", packed, w, h))
    assert out.shape == (h, w, 3)
    assert out.dtype == np.uint8
    # U=V=128 时为灰度，三通道应相等
    assert out[0, 0, 0] == out[0, 0, 1] == out[0, 0, 2]


def test_yuyv_luma_order_preserved():
    w = 2
    packed = bytes([50, 128, 200, 128])
    out = image_to_bgr(img("yuyv", packed, w, 1))
    assert out[0, 0, 2] < out[0, 1, 2]  # 第一个像素更暗


def test_unsupported_encoding_raises():
    with pytest.raises(ValueError):
        image_to_bgr(img("bayer_grbg8", bytes([0, 0]), 2, 1))


def test_empty_encoding_raises():
    with pytest.raises(ValueError):
        image_to_bgr(img("", bytes([0, 0, 0]), 1, 1))
