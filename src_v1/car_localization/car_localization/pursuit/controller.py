"""pursuit/controller.py — 纯追踪舵角/速度计算 + 限幅 + 倒车符号翻转。

从 path_executor.py V1 拆出：
  - pure_pursuit_control L974-1026：舵角公式（atan(L*sinα/d)）+ 转弯减速
  - L1320-1348：末端减速 + 弯前减速
  - 保留 V1 的两套 gain 系数行为（gain=1.0 逐字等于 V1 旧行为）
  - 新增舵角速率限幅（V2 新增功能，V1 无此限制）

重要说明：V1 的 pure_pursuit_control 使用 gain=1.0（隐含在公式中）：
  steer = atan(L * sin(alpha) / max(d, 0.10))
V2 在公式中引入 gain 因子，gain=0.5 时输出与 V1 相同。

设计模式：纯函数，输入 Pose + Config → 输出 PursuitCommand。
"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple

from ..core.geometry import Pose, normalize_angle


@dataclass(frozen=True)
class PursuitCommand:
    """纯追踪控制指令。

    V1 返回 (linear_vel, steer_deg) 的元组；本类封装为命名字段。
    """
    linear_x: float      # 线速度（m/s）；负值 = 倒车
    steer_deg: float     # 舵角（度），正值=左转，负值=右转
    curvature: float = 0.0   # 曲率（1/m）
    speed_reason: str = ''   # 速度限速原因（调试用）
    reverse: bool = False    # 是否倒车


# V1 的 gain 系数（隐含在公式中，本模块提供显式两套）
GAIN_V1 = 1.0         # 等同于 V1 原始行为
GAIN_V2_HALF = 0.5    # V2 新增：gain=0.5 时与 V1 逐位一致（2*L*s/d * 0.5 = L*s/d）


def compute_command(
    pose_x: float, pose_y: float, pose_yaw: float,
    target_x: float, target_y: float,
    wheelbase: float = 0.157,
    max_steer_deg: float = 30.0,
    gain: float = 1.0,
    speed: float = 0.3,
    dist_to_final: Optional[float] = None,
    corner_angle_deg: float = 0.0,
    dist_to_vertex: Optional[float] = None,
    last_steer_deg: float = 0.0,
    dt: float = 0.05,
    max_steer_rate_deg_s: float = 0.0,
    # 减速参数
    end_slow_dist: float = 0.5,
    end_slow_factor: float = 0.35,
    end_slow_floor: float = 0.1,
    corner_slow_dist: float = 0.45,
    corner_slow_speed: float = 0.12,
    corner_angle_thr_deg: float = 45.0,
    # 速度衰减
    steer_factor_max: float = 30.0,
    speed_reduction: float = 0.5,
    speed_floor: float = 0.1,
) -> PursuitCommand:
    """纯追踪舵角 + 速度计算（纯函数）。

    对应 V1 pure_pursuit_control L974-1026 的完整语义，加上
    L1320-1348 的末端减速和弯前减速。

    Args:
        pose_x, pose_y, pose_yaw: 当前位姿（V1 self.current_x/y/yaw）
        target_x, target_y: 瞄准点坐标
        wheelbase: 轴距 L（V1 self.wheelbase = 0.157）
        max_steer_deg: 舵角硬限幅（V1 ±30°）
        gain: 公式增益（V1 隐含 1.0；V2 gain=0.5 等价 V1）
        speed: 巡航速度（V1 self.linear_speed = 0.3）
        dist_to_final: 到终点距离（用于末端减速）
        corner_angle_deg: 弯角角度（用于弯前减速，0 = 无弯角）
        dist_to_vertex: 距顶点距离（用于弯前减速，None = 不减速）
        last_steer_deg: 上一拍舵角（用于速率限幅）
        dt: 控制周期（秒，V1 0.05s = 20Hz）
        max_steer_rate_deg_s: 最大舵角速率（deg/s，0 = 不限幅）
        end_slow_dist: 末端减速距离阈值（V1 0.5m）
        end_slow_factor: 末端减速系数（V1 0.35）
        end_slow_floor: 末端减速下限（V1 0.1）
        corner_slow_dist: 弯前减速距离阈值（V1 0.45m）
        corner_slow_speed: 弯前减速上限（V1 0.12）
        corner_angle_thr_deg: 弯前减速角度阈值（V1 45°）
        steer_factor_max: 速度衰减因子（V1 30.0）
        speed_reduction: 转弯减速比例（V1 0.5）
        speed_floor: 速度下限（V1 0.1）

    Returns:
        PursuitCommand 对象
    """
    # ── 舵角计算：V1 L986-1017 ──────────────────────────────

    # 目标相对位置（地图帧）
    dx = target_x - pose_x
    dy = target_y - pose_y

    # 世界→车体系旋转（V1 L993-996）
    cos_yaw = math.cos(-pose_yaw)
    sin_yaw = math.sin(-pose_yaw)
    rel_x = dx * cos_yaw - dy * sin_yaw  # 前方分量
    rel_y = dx * sin_yaw + dy * cos_yaw  # 左侧分量

    # 距离
    dist = math.sqrt(dx * dx + dy * dy)

    if dist < 0.01:  # V1 L1001-1002
        return PursuitCommand(linear_x=0.0, steer_deg=0.0, speed_reason='no_target')

    # 纯追踪舵角公式：V1 L1006-1015
    # steer = atan(L * sin(alpha) / max(d, 0.10))
    # V2: steer = atan(gain * 2 * L * sin(alpha) / denom)
    #   gain=0.5 时 0.5*2=1，与 V1 逐位一致
    alpha = math.atan2(rel_y, rel_x)   # 瞄准点方位角
    denom = max(dist, 0.10)            # V1: max(dist, 0.10) 防近距离增益爆炸

    # 使用 gain 因子
    if abs(gain - 0.5) < 1e-12:
        # gain=0.5 → gain*2 = 1.0，直接等同于 V1 公式
        steer_deg = math.degrees(
            math.atan(wheelbase * math.sin(alpha) / denom)
        )
    else:
        steer_deg = math.degrees(
            math.atan(gain * 2.0 * wheelbase * math.sin(alpha) / denom)
        )

    # 硬限幅 ±max_steer_deg（V1 L1017）
    steer_deg = max(-max_steer_deg, min(max_steer_deg, steer_deg))

    # ── 舵角速率限幅（V2 新增，V1 无此限制）──────────────────
    if max_steer_rate_deg_s > 0:
        max_delta = max_steer_rate_deg_s * dt
        delta = steer_deg - last_steer_deg
        if delta > max_delta:
            steer_deg = last_steer_deg + max_delta
        elif delta < -max_delta:
            steer_deg = last_steer_deg - max_delta

    # ── 速度计算 ────────────────────────────────────────────
    # V1 L1019-1024: 转弯减速（turn_factor）
    steer_abs = abs(steer_deg)
    turn_factor = steer_abs / steer_factor_max  # 舵角比率 0..1
    linear_vel = speed * (1.0 - speed_reduction * turn_factor)
    linear_vel = max(speed_floor, linear_vel)   # V1 L1024: 下限 0.1 m/s

    speed_reason = 'cruise'

    # V1 L1323-1324: 末端减速
    if dist_to_final is not None and dist_to_final < end_slow_dist:
        limit_v = max(end_slow_floor, end_slow_factor * dist_to_final)
        if limit_v < linear_vel:
            linear_vel = limit_v
            speed_reason = 'end_approach'

    # V1 L1330-1348: 弯前减速
    if (corner_angle_deg > corner_angle_thr_deg
            and dist_to_vertex is not None
            and dist_to_vertex < corner_slow_dist):
        if corner_slow_speed < linear_vel:
            linear_vel = corner_slow_speed
            speed_reason = 'corner_approach'

    # 曲率（调试用）
    if dist > 0.01:
        curvature = 2.0 * math.sin(alpha) / denom
    else:
        curvature = 0.0

    return PursuitCommand(
        linear_x=linear_vel,
        steer_deg=steer_deg,
        curvature=curvature,
        speed_reason=speed_reason,
        reverse=(linear_vel < 0),
    )


def brake_command() -> PursuitCommand:
    """零速刹车指令。对应 V1 Twist() 零命令。"""
    return PursuitCommand(linear_x=0.0, steer_deg=0.0, speed_reason='brake')


# ── V1 兼容函数 ──────────────────────────────────────────────

def pure_pursuit_control_v1(
    target_point: dict,
    current_x: float, current_y: float, current_yaw: float,
    wheelbase: float = 0.157,
    linear_speed: float = 0.3,
) -> Tuple[float, float]:
    """V1 pure_pursuit_control 的纯函数版本（独立于 self）。

    完全复制 V1 L974-1026 逻辑，输入无状态参数。

    Args:
        target_point: V1 格式瞄准点 dict {'x','y',...}
        current_x, current_y, current_yaw: 当前位姿
        wheelbase: 轴距
        linear_speed: 巡航速度

    Returns:
        (linear_vel, steer_deg) 与 V1 完全相同
    """
    if target_point is None:
        return 0.0, 0.0

    dx = target_point['x'] - current_x
    dy = target_point['y'] - current_y

    cos_yaw = math.cos(-current_yaw)
    sin_yaw = math.sin(-current_yaw)
    rel_x = dx * cos_yaw - dy * sin_yaw
    rel_y = dx * sin_yaw + dy * cos_yaw

    dist = math.sqrt(dx * dx + dy * dy)

    if dist < 0.01:
        return 0.0, 0.0

    alpha = math.atan2(rel_y, rel_x)
    steer_deg = math.degrees(
        math.atan(wheelbase * math.sin(alpha) / max(dist, 0.10))
    )
    steer_deg = max(-30.0, min(30.0, steer_deg))

    turn_factor = abs(steer_deg) / 30.0
    linear_vel = linear_speed * (1.0 - 0.5 * turn_factor)
    linear_vel = max(0.1, linear_vel)

    return linear_vel, steer_deg
