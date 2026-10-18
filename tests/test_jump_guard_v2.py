#!/usr/bin/env python3
"""jump guard v2 决策逻辑等价验证（与 px4_odom_convert.cpp 逐行对应）。

本地无 ROS2 环境，用 Python 复刻三级防护状态机（L1 insane 熔断 / L2 bump
节流 / L3 EKF reset 风暴熔断 + 常规 hold/confirm 路径），回放 2026-10-16~
10-17 node_odom.log 实测场景做行为断言：

  1) 正常悬停：直通、零 hold、零 bump
  2) SLAM 疯跳（实测 26~585m/帧）：L1 锁定输出、零 bump、恢复原位后接管
  3) 真搬运（disarmed 2m/s）：1 次 bump 后放行跟随
  4) armed 疯跳：L1 锁定、hold 到底、永不 bump
  5) EKF reset 风暴：>5 次/60s 置位风暴、accept 冻结、窗口滑过自动解除
  6) 节流窗口内不前移判定基准：连续确认每 0.2s 复查、10s 后才放行

v2.1（2026-10-17）扩展：状态机同时复刻 px4_odom_convert 的可观测性层——
事件 blackbox（writeBlackbox 等价，g.rec 列表，每条含 4 行全部字段）+
3s 质量窗口统计（win_* 字段，健康行数据源）。决策逻辑零改动，原有 6 场景
断言不变；blackbox/健康行专项断言见 test_blackbox_health_v2.py，
watchdog 侧见 test_localization_watchdog.py。
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


@dataclass
class GuardState:
    """与 px4_odom_convert.hpp 成员一一对应（默认值=declare_parameter 默认）。"""
    # 参数
    jump_hold_m: float = 0.5
    jump_yaw_rad: float = 0.8
    jump_confirm_n: int = 8
    slam_insane_m: float = 2.0
    slam_recover_n: int = 86
    slam_recover_m: float = 0.1
    slam_recover_yaw_rad: float = 0.1
    bump_min_interval_s: float = 10.0
    ekf_storm_window_s: float = 60.0
    ekf_storm_max_resets: int = 5
    # 常规 guard 状态
    gate_has_prev: bool = False
    gate_hold_count: int = 0
    gate_prev: tuple = (0.0, 0.0, 0.0, 1.0)   # x, y, yaw, qw（yaw 用标量等价四元数）
    gate_out: tuple = (0.0, 0.0, 0.0, 1.0)
    # L1
    slam_bad: bool = False
    slam_recover_count: int = 0
    slam_has_prev_raw: bool = False
    slam_prev_raw: tuple = (0.0, 0.0, 0.0, 1.0)
    # L2
    has_last_bump: bool = False
    last_bump_time: float = 0.0
    # L3
    reset_times: list = field(default_factory=list)
    ekf_storm: bool = False
    # 输出
    ev_reset_counter: int = 0
    # 统计
    stats: dict = field(default_factory=lambda: {
        "hold_frames": 0, "bumps": 0, "insane_events": 0,
        "storm_events": 0, "passthrough_frames": 0})
    # ---- v2.1 可观测性等价（对应 cpp writeBlackbox / onHealthTimer）----
    aligned: int = 0                    # 对齐完成（ALIGN_BUMP 后为 1）
    last_gate_holding: bool = False     # 最近一帧 guard 是否 hold（guard= 数据源）
    l2_throttling: bool = False         # 节流期标志（L2_THROTTLED 只记首帧）
    total_l1: int = 0                   # L1 触发累计
    total_bumps: int = 0                # bump 累计（含对齐首锚）
    total_storm_on: int = 0             # L3 风暴置位累计
    total_ekf_resets: int = 0           # EKF reset 事件累计
    last_l1_t: float = -1.0             # 最近 L1 触发时刻（-1 = 从未）
    last_storm_on_t: float = -1.0       # 最近 L3 置位时刻
    last_storm_clear_t: float = -1.0    # 最近 L3 解除时刻
    # 3s 质量窗口累计（健康行输出后由 health_reset_windows 清零）
    win_frames: int = 0
    win_hold: int = 0
    win_stable: int = 0
    win_jump_sum: float = 0.0
    win_jump_max: float = 0.0
    # blackbox 事件记录（writeBlackbox 等价产物，键集与 C++ 4 行一致）
    rec: list = field(default_factory=list)

    def dpos(self, a, b) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def dyaw(self, a, b) -> float:
        return abs(wrap_pi(a[2] - b[2]))


def _bb(g, t, event, trans, dpos, dyaw, frm, to, armed) -> None:
    """writeBlackbox 等价：一条记录含 C++ 4 行全部字段（键集与 cpp
    writeBlackbox 对齐；cov_max/xy_cnt/hdg_cnt 为 C++ 独有数据源，二维
    回放中恒 0；delta_ned/from_ned/to_ned 的 z 分量恒 0）。
    frm/to 取 (x, y, yaw, qw) 元组；调用时序与 C++ 一致——状态更新后、
    锚点前移前，frm 为旧锚点、to 为本帧原始位姿。"""
    g.rec.append({
        "event": event, "t": round(t, 3), "trans": trans,
        "armed": 1 if armed else 0, "aligned": g.aligned,
        "dpos_m": round(dpos, 3), "dyaw_deg": round(math.degrees(dyaw), 1),
        "delta_ned": (round(to[0] - frm[0], 3), round(to[1] - frm[1], 3), 0.0),
        "from_ned": (round(frm[0], 3), round(frm[1], 3), 0.0),
        "to_ned": (round(to[0], 3), round(to[1], 3), 0.0),
        "ev_reset": g.ev_reset_counter % 256,
        "resets_60s": len(g.reset_times),
        "resets_total": g.total_ekf_resets,
        "storm": int(g.ekf_storm), "ekf_valid": 1, "xy_cnt": 0, "hdg_cnt": 0,
        "win_max_m": round(g.win_jump_max, 3),
        "win_mean_m": (round(g.win_jump_sum / (g.win_frames - 1), 3)
                       if g.win_frames > 1 else 0.0),
        "win_frames": g.win_frames,
        "stable_now": g.slam_recover_count, "cov_max": 0.0,
    })


def guard_feed(g: GuardState, t: float, x: float, y: float, yaw: float,
               armed: bool = False) -> tuple:
    """等价于 publishVehicleVisualOdometry 内 jump guard v2 主体的决策路径。

    返回 (out_x, out_y, out_yaw, holding, bumped)。
    EKF reset 事件单独用 on_ekf_reset() 注入（对应 onEkfLocalPosition）。
    v2.1 包装层：hold 窗口计数（win_hold）与最近一帧 guard 状态
    （last_gate_holding）在此单点维护，决策本身全在 _guard_feed_impl。
    """
    out = _guard_feed_impl(g, t, x, y, yaw, armed)
    if out[3]:
        g.win_hold += 1
    g.last_gate_holding = out[3]
    return out


def published_vio_fields(g: GuardState, holding: bool) -> dict:
    """发布层质量字段等价映射；reset_counter 在 hold/正常帧均沿用状态值。"""
    variance = 100.0 if holding else 0.01
    return {
        "position_variance": (variance, variance, variance),
        "orientation_variance": (variance, variance, variance),
        "quality": 1 if holding else 100,
        "reset_counter": g.ev_reset_counter,
    }


def _guard_feed_impl(g: GuardState, t: float, x: float, y: float, yaw: float,
                     armed: bool = False) -> tuple:
    cur = (x, y, yaw, 1.0)
    gate_holding = False
    bumped = False

    # SLAM 原始相邻帧跟踪 + 3s 质量窗口累计（v2.1，对应 cpp win_* 统计）
    raw_stable = False
    if g.slam_has_prev_raw:
        raw_dp = g.dpos(cur, g.slam_prev_raw)
        raw_stable = (raw_dp < g.slam_recover_m and
                      g.dyaw(cur, g.slam_prev_raw) < g.slam_recover_yaw_rad)
        g.win_jump_sum += raw_dp
        g.win_jump_max = max(g.win_jump_max, raw_dp)
        if raw_stable:
            g.win_stable += 1
    g.win_frames += 1
    g.slam_prev_raw = cur
    g.slam_has_prev_raw = True

    if not g.gate_has_prev:
        g.gate_prev = cur
        g.gate_out = cur
        g.gate_has_prev = True
        g.stats["passthrough_frames"] += 1
        return x, y, yaw, False, False

    if g.slam_bad:
        # ---- L1 熔断中 ----
        g.slam_recover_count = g.slam_recover_count + 1 if raw_stable else 0
        if g.slam_recover_count >= g.slam_recover_n:
            d_out = g.dpos(cur, g.gate_out)
            bump_ok = (not g.has_last_bump or
                       t - g.last_bump_time >= g.bump_min_interval_s)
            if d_out < g.jump_hold_m:
                # 原位恢复：直接接管
                g.slam_bad = False
                g.slam_recover_count = 0
                g.gate_hold_count = 0
                _bb(g, t, "L1_RECOVER_INPLACE", "L1_LOCK->OK", d_out,
                    abs(wrap_pi(cur[2] - g.gate_out[2])), g.gate_out, cur, armed)
                g.gate_prev = cur
                g.gate_out = cur
                g.stats["passthrough_frames"] += 1
                return x, y, yaw, False, False
            if (not armed and not g.ekf_storm and bump_ok):
                # 新位置恢复 + 放行条件满足：一次 bump 平滑重锚
                g.slam_bad = False
                g.slam_recover_count = 0
                g.gate_hold_count = 0
                g.ev_reset_counter += 1
                g.last_bump_time = t
                g.has_last_bump = True
                g.total_bumps += 1
                _bb(g, t, "L1_RECOVER_REANCHOR", "L1_LOCK->REANCHOR", d_out,
                    abs(wrap_pi(cur[2] - g.gate_out[2])), g.gate_out, cur, armed)
                g.gate_prev = cur
                g.gate_out = cur
                g.stats["bumps"] += 1
                return x, y, yaw, False, True
            # 其余（armed / storm / 节流）：保持熔断 hold，每帧复查放行条件
        g.stats["hold_frames"] += 1
        o = g.gate_out
        return o[0], o[1], o[2], True, False

    # ---- 常规/hold/confirm/L1 触发 ----
    gate_dp = g.dpos(cur, g.gate_prev)
    gate_dy = g.dyaw(cur, g.gate_prev)

    if gate_dp > g.slam_insane_m or gate_dy > 2.0 * g.jump_yaw_rad:
        # L1 触发
        g.slam_bad = True
        g.slam_recover_count = 0
        g.gate_hold_count = 0
        g.total_l1 += 1
        g.last_l1_t = t
        g.l2_throttling = False
        _bb(g, t, "L1_TRIGGER", "OK->L1_LOCK", gate_dp, gate_dy,
            g.gate_prev, cur, armed)
        g.stats["insane_events"] += 1
        g.stats["hold_frames"] += 1
        o = g.gate_out
        return o[0], o[1], o[2], True, False

    if gate_dp > g.jump_hold_m or gate_dy > g.jump_yaw_rad:
        g.gate_hold_count += 1
        confirmed = (not armed and not g.ekf_storm and
                     g.gate_hold_count >= g.jump_confirm_n)
        if confirmed:
            bump_ok = (not g.has_last_bump or
                       t - g.last_bump_time >= g.bump_min_interval_s)
            g.gate_hold_count = 0
            if bump_ok:
                g.ev_reset_counter += 1
                g.last_bump_time = t
                g.has_last_bump = True
                g.total_bumps += 1
                g.l2_throttling = False
                _bb(g, t, "L2_ACCEPT", "HOLD->ACCEPT", gate_dp, gate_dy,
                    g.gate_out, cur, armed)
                g.gate_prev = cur
                g.gate_out = cur
                g.stats["bumps"] += 1
                bumped = True
                return x, y, yaw, False, True
            # 节流窗口内：hold 输出，gate_prev 不动（blackbox 只记首帧）
            if not g.l2_throttling:
                g.l2_throttling = True
                _bb(g, t, "L2_THROTTLED", "HOLD->HOLD_THROTTLE", gate_dp,
                    gate_dy, g.gate_out, cur, armed)
            g.stats["hold_frames"] += 1
            o = g.gate_out
            return o[0], o[1], o[2], True, False
        g.stats["hold_frames"] += 1
        o = g.gate_out
        return o[0], o[1], o[2], True, False

    # 正常放行
    g.gate_hold_count = 0
    g.l2_throttling = False
    g.gate_prev = cur
    g.gate_out = cur
    g.stats["passthrough_frames"] += 1
    return x, y, yaw, False, False


def on_ekf_reset(g: GuardState, t: float, armed: bool = False) -> None:
    """等价于 onEkfLocalPosition 的 L3 检测与解除逻辑（v2.1 含 blackbox）。"""
    g.reset_times.append(t)
    while g.reset_times and t - g.reset_times[0] > g.ekf_storm_window_s:
        g.reset_times.pop(0)
    g.total_ekf_resets += 1
    if not g.ekf_storm and len(g.reset_times) > g.ekf_storm_max_resets:
        g.ekf_storm = True
        g.stats["storm_events"] += 1
        g.total_storm_on += 1
        g.last_storm_on_t = t
        _bb(g, t, "L3_STORM_ON", "OK->STORM", 0.0, 0.0,
            (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0), armed)
    elif g.ekf_storm:
        pass  # 风暴中继续 reset，保持置位
    if g.ekf_storm:
        # 解除检查（每次回调都执行）
        while g.reset_times and t - g.reset_times[0] > g.ekf_storm_window_s:
            g.reset_times.pop(0)
        if len(g.reset_times) <= g.ekf_storm_max_resets:
            g.ekf_storm = False
            g.last_storm_clear_t = t
            _bb(g, t, "L3_STORM_CLEAR", "STORM->OK", 0.0, 0.0,
                (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0), armed)


def align_bump(g: GuardState, t: float, yaw_offset: float = 0.0) -> None:
    """对齐完成的首次 bump（computeAlignment / 直通对齐路径）。
    v2.1：同时记 ALIGN_BUMP blackbox（对应 cpp 两处发射点）。"""
    g.ev_reset_counter = 1
    g.last_bump_time = t
    g.has_last_bump = True
    g.aligned = 1
    g.total_bumps += 1
    _bb(g, t, "ALIGN_BUMP", "UNALIGNED->ALIGNED", 0.0, yaw_offset,
        (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0), False)


# ============================================================================
# 发布层契约
# ============================================================================

def test_vio_quality_and_variance_follow_hold_state():
    g = GuardState()
    align_bump(g, 0.0)
    normal = published_vio_fields(g, holding=False)
    assert normal["quality"] == 100
    assert normal["position_variance"] == (0.01, 0.01, 0.01)
    assert normal["orientation_variance"] == (0.01, 0.01, 0.01)

    reset_counter = g.ev_reset_counter
    hold = published_vio_fields(g, holding=True)
    assert hold["quality"] == 1
    assert hold["position_variance"] == (100.0, 100.0, 100.0)
    assert hold["orientation_variance"] == (100.0, 100.0, 100.0)
    assert hold["reset_counter"] == reset_counter


# ============================================================================
# 场景回放断言
# ============================================================================

def test_1_normal_hover_passthrough():
    """正常悬停：armed + 平稳微动 -> 全程直通，零 hold 零 bump。"""
    g = GuardState()
    align_bump(g, 0.0)
    rng = random.Random(42)
    out_jumps = 0
    for i in range(4300):  # 100s @43Hz
        t = i / 43.0
        x = 0.01 * math.sin(t * 0.5) + rng.gauss(0, 0.002)
        y = 0.01 * math.cos(t * 0.5) + rng.gauss(0, 0.002)
        ox, oy, _, holding, bumped = guard_feed(g, t, x, y, 0.0, armed=True)
        assert not holding and not bumped
        assert abs(ox - x) < 1e-9 and abs(oy - y) < 1e-9
        out_jumps += 0
    assert g.stats["bumps"] == 0
    assert g.stats["hold_frames"] == 0
    assert g.stats["insane_events"] == 0
    assert g.ev_reset_counter == 1  # 只有对齐那一次


def test_2_slam_insane_lock_zero_bump():
    """SLAM 疯跳（回放实测模式 26~585m/帧，disarmed）：L1 锁定、零 bump。"""
    g = GuardState()
    align_bump(g, 0.0)
    # 先 2s 正常
    for i in range(86):
        guard_feed(g, i / 43.0, 0.0, 0.0, 0.0, armed=False)
    # 疯跳 60s：实测模式 —— ICP 摇摆，帧间大跳
    rng = random.Random(7)
    bumps_before = g.stats["bumps"]
    for i in range(86, 86 + 2580):
        t = i / 43.0
        x = rng.uniform(-300, 300)
        y = rng.uniform(-300, 300)
        yaw = rng.uniform(-math.pi, math.pi)
        ox, oy, _, holding, bumped = guard_feed(g, t, x, y, yaw, armed=False)
        assert holding, "疯跳期间必须 hold"
        assert not bumped, "疯跳期间绝不 bump"
        assert abs(ox) < 1e-9 and abs(oy) < 1e-9, "输出必须锁定在最后可信位姿(0,0)"
    assert g.stats["insane_events"] >= 1
    assert g.stats["bumps"] == bumps_before, "L1 熔断期间零 bump"
    # SLAM 恢复：先稳定在原点 2s -> 原位接管
    for i in range(86 + 2580, 86 + 2580 + 200):
        t = i / 43.0
        guard_feed(g, t, 0.005, 0.005, 0.0, armed=False)
    assert not g.slam_bad, "SLAM 稳定 2s 后应解除熔断"
    assert g.stats["bumps"] == bumps_before, "原位恢复不产生 bump"


def test_3_smooth_move_passthrough_then_relocate_bump():
    """平滑搬运 + 重定位跳变：连续移动天然直通（零 bump）；SLAM 重定位
    瞬跳 1.5m 到新锚点后稳定 -> 8 帧确认 + 恰好 1 次 bump + 放行跟随。
    对齐 bump 设在 -100s：真实开机对齐与飞行中重定位间隔远超节流窗口。"""
    g = GuardState()
    align_bump(g, -100.0)
    for i in range(86):
        guard_feed(g, i / 43.0, 0.0, 0.0, 0.0, armed=False)
    # 搬运 2m/s 移动 2.5s 到 5m 外：43Hz 下帧间 0.047m << 0.5m -> 全程直通零 bump
    for i in range(86, 86 + 108):
        t = i / 43.0
        x = 2.0 * (t - 2.0)
        ox, oy, _, holding, bumped = guard_feed(g, t, x, 0.0, 0.0, armed=False)
        assert not holding and not bumped, "平滑移动必须直通"
        assert abs(ox - x) < 1e-9
    assert g.stats["bumps"] == 0
    assert g.stats["hold_frames"] == 0
    # 停稳后 SLAM 全局重定位：坐标基准瞬跳 1.5m（0.5m < 1.5m < 2m，走
    # confirm 路径而非 L1）后稳定在新位置：
    for i in range(86 + 108, 86 + 108 + 400):
        t = i / 43.0
        ox, oy, _, holding, bumped = guard_feed(g, t, 5.0 + 1.5, 0.0, 0.0, armed=False)
    assert g.stats["bumps"] == 1, f"重定位瞬跳应恰好 1 次 bump，实际 {g.stats['bumps']}"
    assert abs(ox - 6.5) < 1e-9, "最终应放行到新锚点"
    assert not holding
    # 再稳 20s：不再有任何新 bump / 新 hold（8 帧确认期的 hold 是预期防护）
    base_holds = g.stats["hold_frames"]
    for i in range(86 + 508, 86 + 508 + 860):
        t = i / 43.0
        ox, oy, _, holding, bumped = guard_feed(g, t, 6.5, 0.0, 0.0, armed=False)
        assert not holding and not bumped, "稳定期必须直通"
    assert g.stats["bumps"] == 1
    assert g.stats["hold_frames"] == base_holds, "稳定期不得新增 hold"
    # 确认期 hold 帧数应约为 jump_confirm_n（8 帧观察期）
    assert base_holds <= 10, f"重定位确认期 hold 异常偏多: {base_holds}"


def test_4_armed_insane_hold_forever():
    """armed 疯跳：L1 锁定 hold 到底、永不 bump（飞行中 SLAM 跟丢兜底）。"""
    g = GuardState()
    align_bump(g, 0.0)
    for i in range(86):
        guard_feed(g, i / 43.0, 0.0, 0.0, 0.0, armed=False)
    bumps_before = g.stats["bumps"]
    rng = random.Random(3)
    for i in range(86, 86 + 4300):  # 100s 疯跳
        t = i / 43.0
        x, y = rng.uniform(-50, 50), rng.uniform(-50, 50)
        ox, oy, _, holding, bumped = guard_feed(g, t, x, y, 0.0, armed=True)
        assert holding and not bumped
        assert abs(ox) < 1e-9 and abs(oy) < 1e-9
    assert g.stats["bumps"] == bumps_before


def test_5_ekf_storm_freeze_and_clear():
    """EKF reset 风暴：>5 次/60s 置位；期间 confirmed accept 被冻结；窗口滑过解除。"""
    g = GuardState()
    align_bump(g, 0.0)
    for i in range(86):  # 初始：SLAM 稳定在原点建立基准
        guard_feed(g, i / 43.0, 0.0, 0.0, 0.0, armed=False)
    # 每秒 1 次 reset，6 次后应触发风暴
    for i in range(6):
        on_ekf_reset(g, i + 3.0)
        if i < 5:
            assert not g.ekf_storm, "前 5 次不应触发"
    assert g.ekf_storm, "第 6 次 reset 应触发风暴"

    # 风暴中：SLAM 瞬跳 1.5m 后稳定（< insane 但 > hold）——
    # 即使 8 帧确认成立，accept 也必须被冻结（hold 输出）
    for i in range(86):
        t = 10.0 + i / 43.0
        ox, oy, _, holding, bumped = guard_feed(g, t, 1.5, 0.0, 0.0, armed=False)
        assert holding, "风暴期间必须 hold（含确认前的观察期）"
        assert not bumped
    assert g.stats["bumps"] == 0

    # 65s 后窗口滑过 -> 解除
    on_ekf_reset(g, 75.0)
    assert not g.ekf_storm, "窗口滑过后应解除风暴"
    # 解除后：正常 confirm 路径恢复（8 帧确认 + 节流窗口已过 -> bump）
    for i in range(43):
        t = 80.0 + i / 43.0
        ox, oy, _, holding, bumped = guard_feed(g, t, 1.5, 0.0, 0.0, armed=False)
    assert g.stats["bumps"] == 1, "解除后应通过正常路径重锚"
    assert abs(ox - 1.5) < 1e-9


def test_6_throttle_holds_output_full_window():
    """节流：上次 bump（对齐 t=102）后 10s 窗口覆盖跳变期，输出保持 hold，
    窗口滑过（t>=112）后才 bump 放行。"""
    g = GuardState()
    align_bump(g, 102.0)  # 对齐 bump 在 t=102 -> 节流窗口至 t=112
    for i in range(86):
        guard_feed(g, 100.0 + i / 43.0, 0.0, 0.0, 0.0, armed=False)
    # SLAM 瞬移 1.2m 后保持稳定（< insane 2m，> hold 0.5m）
    first_accept_t = None
    for i in range(2000):  # ~46s
        t = 110.0 + i / 43.0
        ox, oy, _, holding, bumped = guard_feed(g, t, 1.2, 0.0, 0.0, armed=False)
        if bumped:
            first_accept_t = t
            break
        assert holding, "节流窗口内必须 hold"
        assert abs(ox) < 1e-9, "hold 输出必须停在原点"
    assert first_accept_t is not None, "窗口过后必须放行"
    assert first_accept_t >= 112.0, f"放行不得早于节流窗口结束 112s（实际 {first_accept_t:.2f}）"
    assert g.stats["bumps"] == 1


if __name__ == "__main__":
    for name, fn in sorted({k: v for k, v in globals().items()
                            if k.startswith("test_")}.items()):
        fn()
        print(f"[PASS] {name}")
    print("\njump guard v2 全部 6 个场景回放断言通过")
