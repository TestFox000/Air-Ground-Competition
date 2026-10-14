"""起步朝向自校准：用一小段直行探测真实朝向，消除 reset 断言的固定 yaw 偏置。

背景
    reset:<起点>,<朝向> 会盲目断言车头朝向；车实际是被人搬上起点的，断言与真实
    朝向差一个固定角 δ 时，纯追踪必然绕目标画圆（2026-10-01 实测 δ≈68.6°）。

校准流程（现场上车后由节点在正式路线前执行一次）
    1. 记录独立位姿 p0 —— ⚠️ 必须是**独立信息源**（官方 EKF / 外部定位），
       不能用产生偏差的那套死推算里程计：它报告的位移方向恒等于它自己的
       信念朝向，永远量不出偏差；
    2. 控制车沿 reset 断言的朝向直行一小段（机体系，建议 0.2~0.4 m）；
    3. 记录 p1，estimate_bias() 由位移方向反推 δ；
    4. 原地反转 −δ 把车头摆正，closed_loop_rate() 给闭环角速度；
    5. 再发正式任务路线。补偿只在此处做一次，路线/坐标语义不做第二处旋转。

本模块只含纯数学，不碰 ROS：角度一律弧度、逆时针为正、atan2 约定。
"""

from __future__ import annotations

import math
from dataclasses import dataclass


def wrap_pi(angle: float) -> float:
    """归一化到 (-pi, pi]。"""
    wrapped = (angle + math.pi) % (2 * math.pi) - math.pi
    return wrapped if wrapped > -math.pi else math.pi


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """四元数 -> 绕 z 轴偏航角（标准 ROS REP-103 约定）。"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass(frozen=True)
class ProbeResult:
    """一次直行探测的结果。"""

    bias: float
    distance: float


def estimate_bias(
    expected_heading: float,
    start: tuple[float, float],
    end: tuple[float, float],
    min_distance: float = 0.2,
) -> ProbeResult | None:
    """由实测位移反推固定偏置 δ = 真实行进方向 − reset 断言朝向。

    返回 None 表示位移太短、结果不可信（车轮打滑/没动），调用方应放弃自动
    校准回退到人工对齐，而不是用一个噪声值补偿。
    """
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    distance = math.hypot(dx, dy)
    if distance < min_distance:
        return None
    observed_heading = math.atan2(dy, dx)
    return ProbeResult(wrap_pi(observed_heading - expected_heading), distance)


def nearest_axis(yaw: float) -> str:
    """真实 yaw 最近的坐标轴断言：'x'(东,0) 或 'y'(北,π/2)。

    path_executor 的 reset 只能断言 x/y 两个**方向**；本函数把任意真实朝向
    吸附到误差最小的那个。注意必须按环绕角比较：旧实现
    `yaw∈[45°,135°)→y 否则 x` 在车朝西（135°~225°）时会错选 x，
    残差最高 135°（画圆根因同款）。

    ⚠️ 操作约束：断言集合只有 {0°, 90°} 两个方向，残差 ≤45° 只在车头
    落在东/北半圆（-45°~135°）时成立。摆车必须让车头大致朝东或朝北；
    朝南/朝西的车，最优残差也有 45°~135°，必须先人工转车或走 align 闭环。
    """
    d_x = abs(wrap_pi(yaw))
    d_y = abs(wrap_pi(yaw - math.pi / 2))
    return "x" if d_x <= d_y else "y"


def corrected_heading(bias: float) -> float:
    """需要原地反转的角度（把固定偏置抵消）。"""
    return wrap_pi(-bias)


def closed_loop_rate(
    current_yaw: float,
    target_yaw: float,
    max_rate: float = 0.5,
    tolerance: float = 0.05,
) -> float:
    """闭环原地转向角速度：符号给转向方向，进入容差区返回 0。"""
    error = wrap_pi(target_yaw - current_yaw)
    if abs(error) <= tolerance:
        return 0.0
    return math.copysign(min(max_rate, abs(error)), error)
