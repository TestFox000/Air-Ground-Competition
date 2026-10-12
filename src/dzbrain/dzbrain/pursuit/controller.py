# -*- coding: utf-8 -*-
"""舵角与速度剖面 —— 纯追踪的"输出层"（阿克曼自行车模型，输出**度**）。

旧实现（`path_executor.pure_pursuit_control` 1736–1809）的问题与保留项：
- 🔧 修掉：舵角公式**缺教科书因子 2**（`δ = atan(L·sinα/d)`，等效增益只有一半）；
  现在 `δ = atan(gain · 2L·sinα / l_d)`，`gain = pursuit.curvature_gain`，
  **`gain = 0.5` 逐位等于旧行为**（`0.5 × 2 = 1` 在 IEEE754 里是精确的），
  所以现场要不要吃那半个增益，是一个 YAML 键的事，不用改代码。
- 🔧 修掉：前视距离硬编码 0.15（注释还写 0.2）、舵角无速率限制（可一帧跳 60°）、
  速度分层散落在控制环里（终点 0.35 / 射击 0.30 / 转角 0.12 三处各写一遍）。
- ✅ 保留：**倒车舵角同式**（只乘方向系数、公式不变）、
  **零 Twist = 刹车**（`brake_command()`：`linear_x = 0` 且 `steer_deg = 0`）、
  分母下限防"目标贴脸时增益爆炸"（`pursuit.lookahead_floor_m`，旧硬编码 0.10）。

★ 不变量
1. `steer_deg` 单位是**度**，正 = 左转（与下位机 `pursuitAngle.angular.z` 契约一致）。
2. `|steer_deg| <= pursuit.max_steer_deg`，且相邻两拍变化 <= `max_steer_rate_deg_s × dt`
   （速率限幅作用在**最终输出**上，倒车符号翻转也算在内）。
3. `linear_x < 0` ⟺ 倒车；倒车速度 = 前进速度 × `reverse_speed_ratio`，下界 `min_speed_mps`。
4. `curvature = 2·sinα / l_d`（**不含 gain**的目标几何曲率，正 = 左弯）：
   限速用"路线要求转多少"，与转向增益解耦 —— 调 gain 不会顺带改速度。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..core.config import Config, curve_speed_limit, lookahead_distance
from ..core.blackboard import P_MOTION_LIMIT
from .lookahead import find_lookahead_point
from .path import Path, Projection

__all__ = ["PursuitCommand", "compute_command", "speed_profile", "brake_command",
           "motion_limit_speed_limit"]

_CHORD_EPS = 1e-6       # 车到前视点的弦长下限（米）：低于它就认为"目标贴在车上"，方位角无意义


@dataclass(frozen=True)
class PursuitCommand:
    """一拍控制指令（不可变）。`linear_x` 单位 m/s，`steer_deg` 单位度（正=左转）。"""

    linear_x: float
    steer_deg: float
    curvature: float        # 目标几何曲率 1/m（正 = 左弯）
    speed_reason: str       # 限速主因：cruise / curve / stop_approach / corner / min_speed / brake
    reverse: bool


# --------------------------------------------------------------------------- #
#  ★ 限速请求读侧（t20）：`motion_limit` 黑板键 → 线速度上限
# --------------------------------------------------------------------------- #
def motion_limit_speed_limit(bb) -> float | None:
    """读黑板键 `P_MOTION_LIMIT` → 线速度上限（m/s）；**无信号一律返回 `None`（读侧 no-op）**。

    契约（写侧原文见 `core/blackboard.py` 的 `P_MOTION_LIMIT` 注释）：
      · 键不存在 / 值为 `None` / 载荷不是 dict / 缺 `speed_mps` / 不是数字 / NaN / ±inf /
        负数 → **全部当"无信号"**，读侧行为与没有这个键时**逐位一致**；
      · 合法值 `speed_mps ≥ 0` → 把线速度**钳到该值以内（只降不升，倒车按绝对值保号）**；
      · `speed_mps = 0.0` = "请求停车"，必须真的给 0（不受 `min_speed_mps` 蠕动下限兜底）。

    ★ 本函数在 50 Hz 控制环里被调用，**任何情况都不抛异常**（黑板实现被换/被关掉 → 当无信号）。
    """
    if bb is None:
        return None
    try:
        payload = bb.get(P_MOTION_LIMIT)
    except Exception:                       # 读黑板失败 = 无信号（宁可跑原速，也不打断循迹）
        return None
    if not isinstance(payload, dict):
        return None
    value = payload.get("speed_mps")
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    value = float(value)
    if not math.isfinite(value) or value < 0.0:
        return None
    return value


# --------------------------------------------------------------------------- #
#  速度剖面
# --------------------------------------------------------------------------- #
def speed_profile(*, curvature: float, dist_to_stop: float | None,
                  corner_dist: float | None, speed_scale: float,
                  cfg: Config, motion_limit_mps: float | None = None,
                  last_speed_mps: float | None = None,
                  dt: float | None = None,
                  cruise_mps: float | None = None) -> tuple[float, str]:
    """前进速度剖面：`min(巡航×scale, 曲率限速, 停站减速, 转角减速) × 量化补偿`，下界 `min_speed`。

    - 停站减速：`v = min(v, max(min_speed, stop_approach_speed_gain × d))`（旧 0.30·d）
    - 转角减速：距尖角顶点 < `corner_slow_dist_m` → 压到 `corner_speed_mps`（旧固定 0.12）
    - `dist_to_stop` / `corner_dist` 传 `None` 表示"这一层不参与"
    - `motion_limit_mps`：外部限速请求（`None` = 无信号 → **完全 no-op**）。它是**最后一层**，
      **在 `min_speed` 下限之后**施加，所以 `0.0` 能真正压到 0（"请求停车"不被蠕动下限顶回去）。

    ★★ t30 新增：**加减速斜坡**（"直线上猛加速、拐角照样减速"的落地件）
      · 两个点号键（**缺省 0.0 = 本层完全不参与**，行为与历史逐位一致）：
          `pursuit.accel_up_mps2`   —— 升速上限（m/s²），建议 0.6~0.8
          `pursuit.accel_down_mps2` —— 降速上限（m/s²），建议 1.0~1.2
      · `last_speed_mps` = **上一拍控制器输出的速度**（命令量，不是里程计实测）；
        `dt` = 控制周期。两个都给了、且至少一个限值 > 0 时，本层才参与：
            `v_hi = last + accel_up·dt`、`v_lo = last − accel_down·dt`（`v_lo` 再被 `min_speed` 兜底）
        目标速度 `v` 超过 `v_hi` 就压到 `v_hi`（`reason="ramp_up"`），低于 `v_lo` 就抬到 `v_lo`
        （`reason="ramp_down"`）。
      · **首拍/无历史（`last_speed_mps is None`）或没给 `dt` ⇒ 本层不参与**（返回 `reason` 不变）。
        为什么不是"当成 0"：调用方若不传历史，每一拍都会被当成首拍 ⇒ 速度被永久压在
        `accel_up·dt`（0.7×0.02 = 0.014 m/s）⇒ **车根本不会动**。所以"无历史 = 不限"是唯一安全语义，
        并且**调用方有责任播种历史**（`sim.py` 已在本卡里接上；真车侧见
        `_tmp_session/t30/patch.md` 的接线清单）。
      · 施加位置：**在 `min_speed` 下限之前**（先斜坡、再兜底），因此
        "运动中速度 ≥ min_speed" 这条既有不变量不变；而 `motion_limit`（安全层）仍在**最后**，
        所以"请求停车 = 0"不会被斜坡或下限顶回去。

    ★★ t36 新增：`cruise_mps` —— **动态档**的注入点。默认 `None` = 用
      `pursuit.cruise_speed_mps`（与历史**逐位一致**）；给值就只换"巡航那一层"的基准速度，
      其余层（曲率/停站/转角/斜坡/下限/motion_limit）一字不改。

    ★ 返回的 `reason` 是**真正起作用的那一层**，便于日志定位"为什么这么慢"。
    """
    cruise = cfg.get("pursuit.cruise_speed_mps") if cruise_mps is None else float(cruise_mps)
    min_speed = cfg.get("pursuit.min_speed_mps")

    v = cruise * max(0.0, speed_scale)
    reason = "cruise"

    curve_v = curve_speed_limit(curvature, cfg)
    if curve_v < v:
        v, reason = curve_v, "curve"

    if dist_to_stop is not None:
        stop_v = max(min_speed, cfg.get("pursuit.stop_approach_speed_gain") * dist_to_stop)
        if stop_v < v:
            v, reason = stop_v, "stop_approach"

    if corner_dist is not None and corner_dist < cfg.get("pursuit.corner_slow_dist_m"):
        corner_v = cfg.get("pursuit.corner_speed_mps")
        if corner_v < v:
            v, reason = corner_v, "corner"

    v *= cfg.get("pursuit.speed_quant_comp")

    # ---- ★ t30 加减速斜坡（缺省 0.0 ⇒ 整层 no-op） ----
    accel_up = cfg.get("pursuit.accel_up_mps2", 0.0) or 0.0
    accel_down = cfg.get("pursuit.accel_down_mps2", 0.0) or 0.0
    if last_speed_mps is not None and dt is not None and dt > 0.0 \
            and (accel_up > 0.0 or accel_down > 0.0):
        last = max(0.0, float(last_speed_mps))          # 命令量恒 ≥ 0（倒车在调用侧另取绝对值）
        v_hi = last + accel_up * dt if accel_up > 0.0 else float("inf")
        v_lo = max(min_speed, last - accel_down * dt) if accel_down > 0.0 else min_speed
        if v > v_hi:
            v, reason = v_hi, "ramp_up"
        elif v < v_lo:
            v, reason = v_lo, "ramp_down"

    if v < min_speed:       # ★ 下界最后兜（保证任何分支都不会低于蠕动速度）
        v, reason = min_speed, "min_speed"
    if motion_limit_mps is not None and motion_limit_mps < v:   # ★ 限速请求：只降不升
        v, reason = float(motion_limit_mps), "motion_limit"
    return v, reason


def _dist_to_stop(pose, path: Path, proj: Projection, cfg: Config) -> float | None:
    """前方最近的停站/终点距离；超过 `approach_m` 返回 None（不做无谓减速）。

    ★ 只遍历 `path.stop_indices`（射击点/扫描点/末点），不做全表扫描 ——
      旧实现每拍遍历全部航点 3 次（survey §8.1）。
    """
    limit = cfg.get("pursuit.approach_m")
    best: float | None = None
    points = path.waypoints
    for i in path.stop_indices:
        if i <= proj.seg_index:         # 已经过的点不再减速（含当前段的起点）
            continue
        w = points[i]
        d = math.hypot(w.x - pose.x, w.y - pose.y)
        if d <= limit and (best is None or d < best):
            best = d
    return best


def _corner_dist(path: Path, proj: Projection, cfg: Config) -> float | None:
    """若本段末端是"尖角"（夹角 > `corner_angle_deg`）→ 返回到顶点的弧长距离，否则 None。"""
    if path.corner_angle_deg(proj.seg_index) <= cfg.get("pursuit.corner_angle_deg"):
        return None
    return proj.dist_to_vertex


def dynamic_gear_speed(cfg: Config, dist_to_cross_m: float | None) -> float | None:
    """★ t36 动态档：按"离下一个桥洞（K 列格）的距离"在**近/远两档**间选巡航速度。

    **默认关闭**（`pursuit.dynamic_gear_enabled=false`）⇒ 本函数返回 `None`，
    调用方（`tasks/tracker.py`）就不给 `compute_command` 传覆盖值 ⇒ 与历史逐位一致。

    键与阈值（全部登记在 `core/config.py` 的 SCHEMA 里，`config/robot.yaml` **零改动**）：
      · `pursuit.dynamic_gear_enabled`     bool，默认 **False** —— 总闸
      · `pursuit.dynamic_gear_near_dist_m` float，默认 **1.05 m**（= 3 格 × 0.35 m）—— "近洞"阈值
      · `pursuit.dynamic_gear_near_mps`    float，默认 **0.35 m/s** —— 距洞 ≤ 阈值时的档
      · `pursuit.dynamic_gear_far_mps`     float，默认 **0.40 m/s** —— 距洞 > 阈值时的档
    `dist_to_cross_m is None`（路线里没有"前方桥洞"）= **远档**（离洞远 ⇒ 放开跑）。

    :returns: 生效的巡航速度（m/s）；**关闭时返回 `None`**（调用方必须把它原样当"不覆盖"）。
    """
    if not bool(cfg.get("pursuit.dynamic_gear_enabled", False)):
        return None
    near_mps = float(cfg.get("pursuit.dynamic_gear_near_mps"))
    far_mps = float(cfg.get("pursuit.dynamic_gear_far_mps"))
    if dist_to_cross_m is None:
        return far_mps
    threshold = float(cfg.get("pursuit.dynamic_gear_near_dist_m"))
    return near_mps if float(dist_to_cross_m) <= threshold else far_mps


# --------------------------------------------------------------------------- #
#  舵角
# --------------------------------------------------------------------------- #
def compute_command(pose, path: Path, proj: Projection, cfg: Config, *,
                    reverse: bool = False, last_steer_deg: float | None = None,
                    dt: float = 0.02, bb=None,
                    motion_limit_mps: float | None = None,
                    last_speed_mps: float | None = None,
                    cruise_mps: float | None = None) -> PursuitCommand:
    """纯追踪一拍：位姿 + 路径投影 → (速度, 舵角, 曲率, 限速主因)。

    `pose` 只需有 `.x/.y/.yaw`（`core.geometry.Pose`）；`last_steer_deg=None` 表示首拍、不做速率限幅。

    ★ 倒车判定 = 显式 `reverse` 参数 **或** 该段带 `R:` 标记（取或，两者都不会漏）；
      倒车只做两件事：速度取负 × `reverse_speed_ratio`、舵角 × `reverse_steer_sign`。
    ★ 前视距离由**巡航速度**决定（不随瞬时速度变化）：否则"减速 → 前视变短 → 转向变猛"
      会形成正反馈，低速贴站时反而摆头。
    ★ `motion_limit_mps` / `bb`（t20 读侧）：限速请求来源二选一 —— 显式传 `motion_limit_mps`
      优先；否则从黑板 `bb` 读 `P_MOTION_LIMIT`。**两者都没有信号时逐位等价于旧行为**
      （`speed_profile` 收到 `None` 就整层不参与）。
    ★★ t30：`last_speed_mps` = 上一拍**控制器输出**的速度（命令量）。传了就启用加减速斜坡
      （限值来自 `pursuit.accel_up_mps2` / `pursuit.accel_down_mps2`，**缺省 0.0 = 不参与**）；
      `None` = 首拍/无历史 ⇒ 不参与（原因见 `speed_profile` 的"为什么不是当成 0"）。
      `dt` 同时是斜坡的时间基准（默认 0.02 s）。
    ★★ t36：`cruise_mps` = **动态档**的巡航速度覆盖（`None` = 不覆盖 ⇒ 逐位同历史；
      只换 `speed_profile` 的巡航那一层，**前视距离仍按配置巡航速度**算，几何不变）。
    """
    limit = motion_limit_mps if motion_limit_mps is not None else motion_limit_speed_limit(bb)
    wheelbase = cfg.get("pursuit.wheelbase_m")
    gain = cfg.get("pursuit.curvature_gain")
    ld = lookahead_distance(cfg.get("pursuit.cruise_speed_mps"), cfg)
    denom = max(ld, cfg.get("pursuit.lookahead_floor_m"))

    target = find_lookahead_point(path, proj, ld, cfg)

    # ---- 车体系分解：前向 / 左向分量（与旧实现同一旋转写法） ----
    dx = target.x - pose.x
    dy = target.y - pose.y
    cos_neg = math.cos(-pose.yaw)
    sin_neg = math.sin(-pose.yaw)
    rel_x = dx * cos_neg - dy * sin_neg
    rel_y = dx * sin_neg + dy * cos_neg
    chord = math.hypot(rel_x, rel_y)
    # ⚠️ 目标点贴在车上时 atan2 全是噪声：取 0（分母下限已挡住增益爆炸，不必像旧代码那样急刹）
    alpha = 0.0 if chord < _CHORD_EPS else math.atan2(rel_y, rel_x)
    sin_a = math.sin(alpha)

    # ---- 舵角：δ = atan(gain · 2L·sinα / l_d) ----
    steer_deg = math.degrees(math.atan(gain * 2.0 * wheelbase * sin_a / denom))
    max_steer = cfg.get("pursuit.max_steer_deg")
    if steer_deg > max_steer:
        steer_deg = max_steer
    elif steer_deg < -max_steer:
        steer_deg = -max_steer

    # ---- 倒车：方向系数 ----
    rev = bool(reverse) or path.segment_is_reverse(proj.seg_index)
    if rev:
        steer_deg *= cfg.get("pursuit.reverse_steer_sign")

    # ---- 速率限幅（对最终输出，含倒车符号翻转） ----
    rate = cfg.get("pursuit.max_steer_rate_deg_s")
    if last_steer_deg is not None and dt > 0.0:
        step = rate * dt
        delta = steer_deg - last_steer_deg
        if delta > step:
            delta = step
        elif delta < -step:
            delta = -step
        steer_deg = last_steer_deg + delta
        if steer_deg > max_steer:       # 兜底：上一个值本身越界时不放大
            steer_deg = max_steer
        elif steer_deg < -max_steer:
            steer_deg = -max_steer

    # ---- 速度剖面 ----
    curvature = 2.0 * sin_a / denom
    speed, reason = speed_profile(
        curvature=curvature,
        dist_to_stop=_dist_to_stop(pose, path, proj, cfg),
        corner_dist=_corner_dist(path, proj, cfg),
        speed_scale=path.segment_speed_scale(proj.seg_index),
        cfg=cfg,
        motion_limit_mps=limit,
        last_speed_mps=last_speed_mps,      # ★ t30：上一拍**命令**速度（None = 首拍/无历史 ⇒ 不参与）
        dt=dt,
        cruise_mps=cruise_mps,              # ★ t36：动态档（None = 不覆盖 ⇒ 逐位同历史）
    )
    if rev:
        min_speed = cfg.get("pursuit.min_speed_mps")
        speed = -max(min_speed, speed * cfg.get("pursuit.reverse_speed_ratio"))
        # ★ 限速请求对**倒车**同样有效：符号翻转后再按绝对值钳（保号）。
        #   必须放在翻转**之后**：否则 `speed=0` 会被上面的 `max(min_speed, …)` 顶成蠕动速度，
        #   "请求停车"就失效了。`0.0` 用正零，避免下游把 `-0.0` 当特殊值。
        if limit is not None and abs(speed) > limit:
            speed = 0.0 if limit == 0.0 else math.copysign(float(limit), speed)
            reason = "motion_limit"

    return PursuitCommand(linear_x=speed, steer_deg=steer_deg, curvature=curvature,
                          speed_reason=reason, reverse=rev)


def brake_command(cfg: Config) -> PursuitCommand:
    """零速刹车：**零 Twist = 刹车**（下位机看到 0 速度即停，不需要额外停机协议）。

    ★ `steer_deg` 也必须是 0：下位机 `servo = angular.z + 60`，
      发一个非零舵角会让轮子保持打死的姿态停车（下次起步会带偏）。
    """
    return PursuitCommand(linear_x=0.0, steer_deg=0.0, curvature=0.0,
                          speed_reason="brake", reverse=False)
