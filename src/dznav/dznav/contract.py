# -*- coding: utf-8 -*-
"""dznav 契约层：场地/底盘常量、坐标变换、话题与命令语法。

坐标约定（与发学员/官方 car_localization 完全一致，见
reference/car_src_fxxy_20261017/car_localization_rect）：
- 地图 14 列(A~N) x 11 行(1~11)，格边 0.35 m；
- 原点在 N11 格右下角，X+ 指向行 11->1，Y+ 指向列 N->A；
- /odom/localization 位置以"格"为单位发布，四元数载 raw yaw，
  下游消费方按 yaw + pi/2 得地图航向（RAW_YAW_TO_MAP_YAW_OFFSET）；
- pursuitAngle.angular.z = 舵机角度(度)，linear.x = m/s（dzactuator 契约）。

dznav 定位融合内部维护真实地图航向 th；对外发布 yaw_out = th - pi/2，
使下游 yaw_out + pi/2 == th 恒成立（对外信念 = 内部信念，无双重误差）。
"""

from __future__ import annotations

import math
import re

# ---- 场地 ----
BLOCK = 0.35                 # 格边长 m
COL_LETTERS = "ABCDEFGHIJKLMN"
MAP_COLS = 14
MAP_ROWS = 11
FIELD_X_MAX = MAP_ROWS * BLOCK    # 3.85 m
FIELD_Y_MAX = MAP_COLS * BLOCK    # 4.90 m
#: 越界急停余量：信念位姿出"场地外扩该值"的矩形即刹停（对齐现役 0.60 m）
OOB_MARGIN = 0.60

# ---- 底盘（dzactuator 实测契约，SIL ChassisSim 同源）----
WHEELBASE = 0.157            # m（现役标定；发学员版为 0.17，配置可覆盖）
STEER_LIMIT_DEG = 35.0       # 舵机行程 ±35°（TargetAngle 钳位）
MAX_SPEED = 0.33             # m/s 物理上限
#: dzactuator ACTIVITY_TIMEOUT=0.5s：pursuitAngle 停发 0.5s 即失活丢模式
CMD_KEEPALIVE_S = 0.2

# ---- 话题 ----
TOPIC_WAYPOINT = "/nav/waypoint"           # String：reset:/load:/路线串（下发）
TOPIC_ODOM_RAW = "/odom"                   # Odometry：dzactuator 原始里程（米）
TOPIC_IMU = "/imu_data"                    # Imu：官方 serial_reader 同名话题
TOPIC_ODOM_LOC = "/odom/localization"      # Odometry：信念位姿（格），dznav 自产
TOPIC_PURSUIT = "pursuitAngle"             # Twist：舵机角(度)/速度
TOPIC_STOP_SIGNAL = "/move_base/stop_signal"   # UInt8：1=放行云台瞄准射击
TOPIC_SHOOT_COUNT = "/shoot_count"         # Int32：累计已发弹数
TOPIC_ESTOP = "/estop"                     # Bool：外部急停（TRANSIENT_LOCAL 闩锁）

#: RAW_YAW_TO_MAP_YAW_OFFSET：raw yaw + 该偏移 = 地图航向（历史契约）
RAW_YAW_TO_MAP_YAW_OFFSET = math.pi / 2

HEADING_YAWS = {
    "x": 0.0,
    "-x": math.pi,
    "y": math.pi / 2,
    "-y": -math.pi / 2,
}

# ---- 命令语法（/nav/waypoint，与发学员版超集兼容）----
#  'reset:N10,y'                     重锚位姿（对齐完成后下达）
#  'load:/path/to/file.txt'          载入路径文件(x,y,z,qx,qy,qz,qw 每行)
#  'M10 L9 K7,y'                     途经格序列 + 可选终姿朝向
#  'D2*'                             '*' = 射击停站格
RE_HEADING = re.compile(r"^(x|-x|y|-y)$")
RE_BLOCK = re.compile(r"^([A-N])([0-9]{1,2})(\*?)$")


def block_to_meters(block: str) -> tuple[float, float]:
    """格号 -> 地图坐标（米）。例：N10 -> (1.5*0.35, 0.5*0.35)。"""
    m = RE_BLOCK.match(block.strip().upper())
    if not m:
        raise ValueError(f"非法格号: {block!r}")
    letter, number = m.group(1), int(m.group(2))
    col_idx = COL_LETTERS.index(letter)
    row_idx = number - 1
    if not (0 <= col_idx < MAP_COLS and 0 <= row_idx < MAP_ROWS):
        raise ValueError(f"格号越界: {block!r}")
    return (10.5 - row_idx) * BLOCK, (13.5 - col_idx) * BLOCK


def wrap_pi(a: float) -> float:
    """归一化到 (-pi, pi]。"""
    w = (a + math.pi) % (2.0 * math.pi) - math.pi
    return w if w > -math.pi else math.pi


def deg(a: float) -> float:
    return a * 180.0 / math.pi


def rad(a: float) -> float:
    return a * math.pi / 180.0


def in_field(x: float, y: float, margin: float = 0.0) -> bool:
    """位姿是否在场地外扩 margin 的矩形内。"""
    return (-margin <= x <= FIELD_X_MAX + margin
            and -margin <= y <= FIELD_Y_MAX + margin)
