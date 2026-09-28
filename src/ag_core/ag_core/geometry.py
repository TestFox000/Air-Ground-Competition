"""云台瞄准几何：像素偏差 -> 云台角度增量。

SmartMaster 云台相机参数（硬件操作指南）：
    Sensor 1/2.6"，焦距 5 mm，FOV 60°，分辨率最高 1920x1080
    云台电机 12 V，12 位磁编码，360°，额定 52 rpm
"""

from __future__ import annotations

import math
from typing import Tuple

__all__ = ["pixel_to_gimbal_delta", "clamp_gimbal", "focal_pixels"]


def focal_pixels(img_w: int, fov_h_deg: float = 60.0) -> float:
    """由水平视场角反算等效焦距（像素）。"""
    return (img_w / 2.0) / math.tan(math.radians(fov_h_deg) / 2.0)


def pixel_to_gimbal_delta(
    pixel_x: float,
    pixel_y: float,
    img_w: int,
    img_h: int,
    fov_h_deg: float = 60.0,
) -> Tuple[float, float]:
    """目标像素偏离画面中心 -> (yaw_delta, pitch_delta)，单位度。

    yaw 正值表示需向右转，pitch 正值表示需向上抬。
    """
    f = focal_pixels(img_w, fov_h_deg)
    dx = pixel_x - img_w / 2.0
    dy = pixel_y - img_h / 2.0
    # 垂直方向焦距按像素宽高比折算（多数 sensor 像素为正方形）
    f_v = f
    yaw = math.degrees(math.atan2(dx, f))
    pitch = math.degrees(math.atan2(-dy, f_v))
    return yaw, pitch


def clamp_gimbal(
    pan: float,
    tilt: float,
    pan_limit: float = 180.0,
    tilt_up: float = 60.0,
    tilt_down: float = -30.0,
) -> Tuple[float, float]:
    pan = max(-pan_limit, min(pan_limit, pan))
    tilt = max(tilt_down, min(tilt_up, tilt))
    return pan, tilt
