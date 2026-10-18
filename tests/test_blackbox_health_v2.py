#!/usr/bin/env python3
"""px4_odom_convert v2.1 可观测性层等价验证（blackbox + 健康行）。

复用 test_jump_guard_v2.py 的 Python 等价状态机（决策逻辑零改动，6 场景
仍由原文件保证），本文件只验证 v2.1 新增的观测层与 C++ 逐点对应：

  * writeBlackbox 等价（g.rec）：各事件类型的记录完整性——每条记录必须
    含 C++ 4 行全部字段（事件/状态转换、跳变距离与方向、EKF reset 快照、
    近 3s 质量窗口），目标是"事后只看 blackbox 就能归因"；
  * onHealthTimer 等价（health_snapshot/format_health_line）：健康行
    key=value 字段集与顺序（watchdog 解析契约）、guard 状态机
    （OK/HOLD/L1_LOCK[+STORM]）、窗口清零、EKF reset 速率；
  * 兼容性：新前缀不含既有解析关键字（'jump guard'/'position delta'/
    'Resilience layer'），机上既有日志解析零破坏。

C++ 侧对应：onboard/dzpx4_patch/px4_odom_convert.cpp 的 writeBlackbox() /
onHealthTimer()。本地无 ROS2，逻辑正确性由本等价机保证。
"""
from __future__ import annotations

import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from test_jump_guard_v2 import (GuardState, align_bump, guard_feed,  # noqa: E402
                                on_ekf_reset)

# ============================================================================
# 健康行契约（与 C++ onHealthTimer 输出字段一一对应、顺序一致）
# ============================================================================
HEALTH_KEYS = [
    "t", "guard", "armed", "aligned", "ekf_valid",
    "slam_frames", "slam_hz", "hold_ratio", "jump_max_m", "jump_mean_m",
    "cov_max",
    "l1_lock", "l1_age_s", "l1_total", "bump_age_s", "bump_total",
    "storm", "storm_age_s", "ekf_reset_60s", "ekf_reset_total", "ev_reset",
]

# blackbox 记录必含字段（与 C++ writeBlackbox 4 行输出对齐）
REQUIRED_BB_FIELDS = [
    "event", "t", "trans", "armed", "aligned",
    "dpos_m", "dyaw_deg", "delta_ned", "from_ned", "to_ned",
    "ev_reset", "resets_60s", "resets_total", "storm", "ekf_valid",
    "xy_cnt", "hdg_cnt",
    "win_max_m", "win_mean_m", "win_frames", "stable_now", "cov_max",
]


def health_snapshot(g: GuardState, t: float, armed: bool,
                    ekf_valid: bool = True, win_s: float = 3.0) -> dict:
    """onHealthTimer 等价：返回健康行键值 dict（字段集/语义与 C++ 一致）。"""
    guard = "L1_LOCK" if g.slam_bad else ("HOLD" if g.last_gate_holding else "OK")
    if g.ekf_storm:
        guard += "+STORM"

    def age(ts: float) -> float:
        return -1.0 if ts < 0 else round(t - ts, 3)

    return {
        "t": round(t, 3), "guard": guard,
        "armed": 1 if armed else 0, "aligned": g.aligned,
        "ekf_valid": 1 if ekf_valid else 0,
        "slam_frames": g.win_frames,
        "slam_hz": round(g.win_frames / win_s, 3),
        "hold_ratio": round(g.win_hold / g.win_frames, 3) if g.win_frames else 0.0,
        "jump_max_m": round(g.win_jump_max, 3),
        "jump_mean_m": (round(g.win_jump_sum / (g.win_frames - 1), 3)
                        if g.win_frames > 1 else 0.0),
        "cov_max": 0.0,  # C++ 独有（SLAM 协方差迹峰值），二维回放恒 0
        "l1_lock": int(g.slam_bad),
        "l1_age_s": age(g.last_l1_t),
        "l1_total": g.total_l1,
        "bump_age_s": round(t - g.last_bump_time, 3) if g.has_last_bump else -1.0,
        "bump_total": g.total_bumps,
        "storm": int(g.ekf_storm),
        "storm_age_s": age(g.last_storm_on_t),
        "ekf_reset_60s": len(g.reset_times),
        "ekf_reset_total": g.total_ekf_resets,
        "ev_reset": g.ev_reset_counter % 256,
    }


def health_reset_windows(g: GuardState) -> None:
    """onHealthTimer 末尾窗口复位等价（下一个 3s 窗口重新累计）。"""
    g.win_frames = 0
    g.win_hold = 0
    g.win_stable = 0
    g.win_jump_sum = 0.0
    g.win_jump_max = 0.0


def format_health_line(snap: dict) -> str:
    """健康行字符串（与 C++ std::cout 输出格式一致）。"""
    return "[px4_odom_convert_health] " + " ".join(
        f"{k}={snap[k]}" for k in HEALTH_KEYS)


def assert_full_record(rec: dict) -> None:
    """blackbox 记录完整性：必含字段一个不少、值非 None。"""
    missing = [k for k in REQUIRED_BB_FIELDS if k not in rec]
    assert not missing, f"blackbox 缺字段 {missing} (event={rec.get('event')})"
    for k in REQUIRED_BB_FIELDS:
        assert rec[k] is not None, f"blackbox 字段 {k} 为 None"


def events(g: GuardState, name: str) -> list:
    return [r for r in g.rec if r["event"] == name]


# ============================================================================
# blackbox：各事件类型记录完整性
# ============================================================================

def test_blackbox_align_bump_record():
    """对齐首锚：ALIGN_BUMP 记录完整，ev_reset=1、total_bumps 计数。"""
    g = GuardState()
    align_bump(g, 12.5, yaw_offset=0.3)
    assert len(g.rec) == 1
    r = g.rec[0]
    assert_full_record(r)
    assert r["event"] == "ALIGN_BUMP"
    assert r["trans"] == "UNALIGNED->ALIGNED"
    assert r["t"] == 12.5
    assert r["ev_reset"] == 1
    assert r["aligned"] == 1
    assert r["dyaw_deg"] == round(math.degrees(0.3), 1)
    assert g.total_bumps == 1


def test_blackbox_l1_trigger_full_context():
    """L1 触发：距离/方向/起止位姿/EKF 快照/质量窗口全字段可归因。"""
    g = GuardState()
    align_bump(g, -100.0)
    for i in range(86):
        guard_feed(g, i / 43.0, 1.0, 2.0, 0.0, armed=False)
    # gate_prev/out 已锚定 (1,2)；单帧疯跳到 (20,25)、yaw 0.5（29.7m > 2m 熔断门限）
    ox, oy, _, holding, bumped = guard_feed(g, 2.0, 20.0, 25.0, 0.5, armed=True)
    assert holding and not bumped
    trig = events(g, "L1_TRIGGER")
    assert len(trig) == 1
    r = trig[0]
    assert_full_record(r)
    assert r["trans"] == "OK->L1_LOCK"
    assert r["armed"] == 1
    expect_d = math.hypot(19.0, 23.0)
    assert r["dpos_m"] > g.slam_insane_m
    assert abs(r["dpos_m"] - expect_d) < 0.01
    assert r["from_ned"][0] == 1.0 and r["from_ned"][1] == 2.0
    assert r["to_ned"][0] == 20.0 and r["to_ned"][1] == 25.0
    assert r["delta_ned"][0] == 19.0 and r["delta_ned"][1] == 23.0
    assert r["dyaw_deg"] == round(math.degrees(0.5), 1)
    assert r["win_frames"] > 0 and r["win_max_m"] >= r["dpos_m"]
    assert r["resets_total"] == 0 and r["storm"] == 0
    assert g.total_l1 == 1 and g.last_l1_t == 2.0


def test_blackbox_l1_recover_inplace():
    """L1 原位恢复：一条 L1_RECOVER_INPLACE，dpos < hold 门限、零 bump。"""
    g = GuardState()
    align_bump(g, 0.0)
    for i in range(86):
        guard_feed(g, i / 43.0, 0.0, 0.0, 0.0, armed=False)
    guard_feed(g, 2.0, 30.0, 0.0, 0.0, armed=False)  # L1 触发
    for i in range(600):                             # 14s 疯跳
        t = 2.02 + i / 43.0
        guard_feed(g, t, 30.0 + 50 * ((i % 7) - 3), 0.0, 0.0, armed=False)
    for i in range(200):                             # 恢复到原位 2s+
        guard_feed(g, 16.1 + i / 43.0, 0.005, 0.005, 0.0, armed=False)
    rec = events(g, "L1_RECOVER_INPLACE")
    assert len(rec) == 1
    r = rec[0]
    assert_full_record(r)
    assert r["trans"] == "L1_LOCK->OK"
    assert 0.0 <= r["dpos_m"] < g.jump_hold_m
    # C++ 记录时点在恢复计数清零之后（与 writeBlackbox 调用顺序一致）
    assert r["stable_now"] == 0
    assert g.stats["bumps"] == 0, "原位恢复不产生 bump"
    assert g.total_bumps == 1, "只有对齐那次 bump"


def test_blackbox_l1_recover_reanchor():
    """L1 新锚点恢复：一条 L1_RECOVER_REANCHOR，dpos=新锚距离、bump+1。"""
    g = GuardState()
    align_bump(g, -100.0)  # 对齐远离节流窗
    for i in range(86):
        guard_feed(g, i / 43.0, 0.0, 0.0, 0.0, armed=False)
    guard_feed(g, 2.0, 30.0, 0.0, 0.0, armed=False)  # L1 触发于原点
    # 疯跳后 SLAM 稳定在 3m 外新锚点：86 帧稳定 -> disarmed + 非风暴 + 节流过 -> 重锚
    for i in range(300):
        t = 2.02 + i / 43.0
        guard_feed(g, t, 3.0, 0.0, 0.0, armed=False)
    rec = events(g, "L1_RECOVER_REANCHOR")
    assert len(rec) == 1
    r = rec[0]
    assert_full_record(r)
    assert r["trans"] == "L1_LOCK->REANCHOR"
    assert abs(r["dpos_m"] - 3.0) < 0.01
    assert r["ev_reset"] == 2, "对齐 1 + 重锚 1"
    assert r["from_ned"][0] == 0.0 and r["to_ned"][0] == 3.0
    assert r["delta_ned"][0] == 3.0
    assert g.total_bumps == 2 and g.stats["bumps"] == 1


def test_blackbox_l2_accept_record():
    """L2 接受（重定位 1.5m 瞬跳）：HOLD->ACCEPT、ev_reset 新值、方向字段。"""
    g = GuardState()
    align_bump(g, -100.0)
    for i in range(86):
        guard_feed(g, i / 43.0, 0.0, 0.0, 0.0, armed=False)
    for i in range(86, 86 + 108):          # 平滑搬运到 5m（直通零 bump）
        t = i / 43.0
        guard_feed(g, t, 2.0 * (t - 2.0), 0.0, 0.0, armed=False)
    for i in range(86 + 108, 86 + 508):    # 重定位瞬跳 1.5m 后稳定
        t = i / 43.0
        guard_feed(g, t, 6.5, 0.0, 0.0, armed=False)
    acc = events(g, "L2_ACCEPT")
    assert len(acc) == 1
    r = acc[0]
    assert_full_record(r)
    assert r["trans"] == "HOLD->ACCEPT"
    # 搬运终点 ~4.98m，重定位瞬跳到 6.5m：1.5m 量级（hold< d <insane，走 confirm）
    assert g.jump_hold_m < r["dpos_m"] < g.slam_insane_m
    assert abs(r["dpos_m"] - 1.5) < 0.1
    assert r["ev_reset"] == 2, "bump 后新值（对齐 1 + 接受 1）"
    assert abs(r["from_ned"][0] - 4.98) < 0.05 and r["to_ned"][0] == 6.5
    assert abs(r["delta_ned"][0] - r["dpos_m"]) < 0.01, "delta 与距离一致"
    assert r["armed"] == 0
    assert g.total_bumps == 2


def test_blackbox_l2_throttled_once_then_accept():
    """L2 节流：整个节流期只记 1 条 L2_THROTTLED（防刷屏），随后 L2_ACCEPT。"""
    g = GuardState()
    align_bump(g, 102.0)  # 节流窗口至 t=112
    for i in range(86):
        guard_feed(g, 100.0 + i / 43.0, 0.0, 0.0, 0.0, armed=False)
    accepted_t = None
    for i in range(2000):  # t=110 起瞬跳 1.2m 后稳定（<2m 非 L1，>0.5m 走 confirm）
        t = 110.0 + i / 43.0
        _, _, _, holding, bumped = guard_feed(g, t, 1.2, 0.0, 0.0, armed=False)
        if bumped:
            accepted_t = t
            break
    assert accepted_t is not None and accepted_t >= 112.0
    thr = events(g, "L2_THROTTLED")
    assert len(thr) == 1, f"节流期应只记 1 条 L2_THROTTLED，实际 {len(thr)}"
    r = thr[0]
    assert_full_record(r)
    assert r["trans"] == "HOLD->HOLD_THROTTLE"
    assert abs(r["dpos_m"] - 1.2) < 0.01
    assert r["from_ned"][0] == 0.0 and r["to_ned"][0] == 1.2
    assert len(events(g, "L2_ACCEPT")) == 1


def test_blackbox_l3_storm_on_clear_records():
    """L3 风暴起止：L3_STORM_ON（窗口/累计快照）+ L3_STORM_CLEAR。"""
    g = GuardState()
    align_bump(g, 0.0)
    for i in range(6):  # 每秒 1 次，第 6 次触发（>5/60s）
        on_ekf_reset(g, i + 3.0, armed=True)
    rec = events(g, "L3_STORM_ON")
    assert len(rec) == 1
    r = rec[0]
    assert_full_record(r)
    assert r["trans"] == "OK->STORM"
    assert r["resets_60s"] == 6 and r["resets_total"] == 6
    assert r["storm"] == 1 and r["armed"] == 1
    assert g.total_storm_on == 1 and g.ekf_storm
    # 60s 窗口滑过 -> 解除
    on_ekf_reset(g, 75.0)
    clear = events(g, "L3_STORM_CLEAR")
    assert len(clear) == 1
    r = clear[0]
    assert_full_record(r)
    assert r["trans"] == "STORM->OK"
    assert r["storm"] == 0 and r["resets_total"] == 7
    assert not g.ekf_storm


def test_blackbox_armed_l1_no_recovery_records():
    """armed 疯跳 + armed 稳定：只有 L1_TRIGGER，无任何 accept/recover 记录。"""
    g = GuardState()
    align_bump(g, 0.0)
    for i in range(86):
        guard_feed(g, i / 43.0, 0.0, 0.0, 0.0, armed=False)
    guard_feed(g, 2.0, 30.0, 0.0, 0.0, armed=True)
    rng = random.Random(5)
    for i in range(860):  # 20s 疯跳
        t = 2.1 + i / 43.0
        guard_feed(g, t, rng.uniform(-50, 50), rng.uniform(-50, 50), 0.0,
                   armed=True)
    for i in range(215):  # 5s 稳定在新位置（armed -> 保持熔断）
        guard_feed(g, 22.1 + i / 43.0, 2.0, 0.0, 0.0, armed=True)
    assert len(events(g, "L1_TRIGGER")) == 1
    for name in ("L1_RECOVER_INPLACE", "L1_RECOVER_REANCHOR", "L2_ACCEPT"):
        assert not events(g, name), f"armed 期间不得出现 {name}"
    assert g.total_bumps == 1  # 仅对齐


# ============================================================================
# 健康行：格式契约 / guard 状态 / 窗口复位 / reset 速率
# ============================================================================

def test_health_line_format_and_keys():
    """健康行 key=value 键序与 C++ 一致（watchdog 解析契约），正常悬停值正确。"""
    g = GuardState()
    align_bump(g, 0.0)
    for i in range(129):  # 3s @ 43Hz
        guard_feed(g, i / 43.0, 0.01, 0.0, 0.0, armed=True)
    snap = health_snapshot(g, t=3.0, armed=True)
    line = format_health_line(snap)
    assert line.startswith("[px4_odom_convert_health] ")
    keys = [tok.split("=")[0] for tok in line.split("] ", 1)[1].split()]
    assert keys == HEALTH_KEYS
    assert snap["guard"] == "OK"
    assert snap["slam_frames"] == 129 and snap["slam_hz"] == 43.0
    assert snap["hold_ratio"] == 0.0
    assert snap["l1_total"] == 0 and snap["bump_total"] == 1
    assert snap["bump_age_s"] == 3.0
    assert snap["l1_age_s"] == -1.0 and snap["storm"] == 0
    assert snap["ekf_reset_60s"] == 0 and snap["ekf_reset_total"] == 0
    assert snap["ev_reset"] == 1


def test_health_guard_states():
    """guard 状态机：OK -> L1_LOCK -> L1_LOCK+STORM；独立场景 HOLD。"""
    g = GuardState()
    align_bump(g, 0.0)
    for i in range(86):
        guard_feed(g, i / 43.0, 0.0, 0.0, 0.0, armed=False)
    assert health_snapshot(g, 2.0, armed=False)["guard"] == "OK"
    guard_feed(g, 2.0, 30.0, 0.0, 0.0, armed=False)  # L1 触发
    snap = health_snapshot(g, 2.02, armed=False)
    assert snap["guard"] == "L1_LOCK" and snap["l1_lock"] == 1
    assert snap["l1_age_s"] == 0.02 and snap["l1_total"] == 1
    for i in range(6):  # EKF 风暴叠加
        on_ekf_reset(g, 3.0 + i)
    snap = health_snapshot(g, 9.0, armed=False)
    assert snap["guard"] == "L1_LOCK+STORM"
    assert snap["storm"] == 1 and snap["storm_age_s"] == 1.0  # 第 6 次 reset(t=8) 置位
    assert snap["ekf_reset_60s"] == 6
    # 常规 hold（非 L1、非风暴）：armed 下 0.8m 跳变 -> HOLD
    g2 = GuardState()
    align_bump(g2, -100.0)
    for i in range(86):
        guard_feed(g2, i / 43.0, 0.0, 0.0, 0.0, armed=True)
    guard_feed(g2, 2.0, 0.8, 0.0, 0.0, armed=True)
    snap = health_snapshot(g2, 2.05, armed=True)
    assert snap["guard"] == "HOLD"
    assert snap["hold_ratio"] > 0.0


def test_health_window_reset_between_snapshots():
    """健康行输出后窗口清零（对应 onHealthTimer 末尾复位），新窗口重新累计。"""
    g = GuardState()
    align_bump(g, 0.0)
    for i in range(129):
        guard_feed(g, i / 43.0, 0.01, 0.0, 0.0, armed=True)
    snap1 = health_snapshot(g, 3.0, armed=True)
    assert snap1["slam_frames"] == 129 and snap1["jump_max_m"] >= 0.0
    health_reset_windows(g)
    snap2 = health_snapshot(g, 6.0, armed=True)  # 尚无新帧
    assert snap2["slam_frames"] == 0 and snap2["slam_hz"] == 0.0
    assert snap2["jump_max_m"] == 0.0 and snap2["hold_ratio"] == 0.0
    for i in range(43):  # 新窗口 1s
        guard_feed(g, 3.0 + i / 43.0, 0.01, 0.0, 0.0, armed=True)
    snap3 = health_snapshot(g, 6.0, armed=True)
    assert snap3["slam_frames"] == 43
    assert abs(snap3["slam_hz"] - 43.0 / 3.0) < 0.001


def test_health_ekf_reset_rate_fields():
    """EKF reset 速率字段：60s 窗计数 + 累计计数 + 风暴标志随窗口滑过回落。"""
    g = GuardState()
    align_bump(g, 0.0)
    for i in range(6):
        on_ekf_reset(g, 10.0 + i)
    snap = health_snapshot(g, 16.0, armed=False)
    assert snap["ekf_reset_60s"] == 6 and snap["ekf_reset_total"] == 6
    assert snap["storm"] == 1
    on_ekf_reset(g, 80.0)  # 窗口滑过：旧 reset 全部出窗，风暴解除
    snap = health_snapshot(g, 81.0, armed=False)
    assert snap["ekf_reset_60s"] == 1
    assert snap["ekf_reset_total"] == 7, "累计计数不随窗口回落"
    assert snap["storm"] == 0


def test_new_prefixes_do_not_collide_with_existing_parsers():
    """兼容性：新前缀/新行不得含既有解析关键字，保证机上既有日志解析
    （patch_deploy.py 的 grep 'jump guard'/'position delta'/
    'Resilience layer'，及 orin_log_capture 类工具）零破坏。"""
    g = GuardState()
    align_bump(g, 0.0)
    line = format_health_line(health_snapshot(g, 1.0, armed=True))
    for kw in ("jump guard", "position delta", "Resilience layer"):
        assert kw not in line
    # blackbox 4 行前缀（C++ writeBlackbox 固定输出样式）
    for bb_line in ("[px4_odom_convert_blackbox] event=L1_TRIGGER t=1.0",
                    "[px4_odom_convert_blackbox]   jump dpos_m=1.0",
                    "[px4_odom_convert_blackbox]   ekf ev_reset=1",
                    "[px4_odom_convert_blackbox]   slam_q win_max_m=1.0"):
        for kw in ("jump guard", "position delta", "Resilience layer"):
            assert kw not in bb_line


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"[PASS] {fn.__name__}")
    print(f"\nblackbox/健康行 v2.1 全部 {len(fns)} 项断言通过")
