#!/usr/bin/env python3
"""localization_watchdog.py 等价验证（可模拟部分全链路）。

watchdog 是机上被动观测脚本（onboard/dzpx4_patch/localization_watchdog.py），
本测试在本地无 ROS2 环境下验证其核心链路：

  * 解析层：健康行 key=value round-trip、blackbox 4 行 -> 事件 dict 聚合
  * 判定层：L1 持续/短暂分级告警、EKF 风暴、SLAM 断流、hold 占比、
    bump 速率、健康行停更（HEALTH_STALE/NODE_DOWN）
  * 去重/升级/恢复：repeat 窗口内不重发、WARN->ERROR 升级即发、
    条件消失补"已恢复"
  * 端到端：等价状态机（test_jump_guard_v2.py）回放"悬停->疯跳 L1->
    疯跳持续->EKF 风暴->SLAM 断流"完整序列，健康行与 blackbox 按真实
    日志流喂入，断言醒目 ERROR 告警按预期产出
"""
from __future__ import annotations

import os
import random
import sys

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, "onboard", "dzpx4_patch"))
sys.path.insert(0, TESTS)

import localization_watchdog as lw  # noqa: E402
from test_jump_guard_v2 import (GuardState, align_bump, guard_feed,  # noqa: E402
                                on_ekf_reset)
from test_blackbox_health_v2 import (  # noqa: E402
    format_health_line, health_reset_windows, health_snapshot)


class FakeClock:
    """可注入 watchdog 的假时钟（秒）。"""

    def __init__(self, t0=1_800_000_000.0):
        self.t = t0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def base_snap(**over):
    """基础健康行 dict（正常悬停值），按需覆盖。"""
    snap = {
        "t": 100.0, "guard": "OK", "armed": 0, "aligned": 1, "ekf_valid": 1,
        "slam_frames": 129, "slam_hz": 43.0, "hold_ratio": 0.0,
        "jump_max_m": 0.01, "jump_mean_m": 0.004, "cov_max": 0.0,
        "l1_lock": 0, "l1_age_s": -1.0, "l1_total": 0,
        "bump_age_s": 10.0, "bump_total": 1, "storm": 0, "storm_age_s": -1.0,
        "ekf_reset_60s": 0, "ekf_reset_total": 0, "ev_reset": 1,
    }
    snap.update(over)
    return snap


def blackbox_lines(rec: dict):
    """把等价状态机的 blackbox dict 渲染成 C++ writeBlackbox 4 行样式。"""
    return [
        f"[px4_odom_convert_blackbox] event={rec['event']} t={rec['t']} "
        f"trans={rec['trans']} armed={rec['armed']} aligned={rec['aligned']}",
        f"[px4_odom_convert_blackbox]   jump dpos_m={rec['dpos_m']} "
        f"dyaw_deg={rec['dyaw_deg']} "
        f"delta_ned=[{rec['delta_ned'][0]}, {rec['delta_ned'][1]}, "
        f"{rec['delta_ned'][2]}] from_ned=[{rec['from_ned'][0]}, "
        f"{rec['from_ned'][1]}, 0.0] to_ned=[{rec['to_ned'][0]}, "
        f"{rec['to_ned'][1]}, 0.0]",
        f"[px4_odom_convert_blackbox]   ekf ev_reset={rec['ev_reset']} "
        f"resets_60s={rec['resets_60s']} resets_total={rec['resets_total']} "
        f"storm={rec['storm']} ekf_valid={rec['ekf_valid']} "
        f"xy_cnt={rec['xy_cnt']} hdg_cnt={rec['hdg_cnt']}",
        f"[px4_odom_convert_blackbox]   slam_q win_max_m={rec['win_max_m']} "
        f"win_mean_m={rec['win_mean_m']} win_frames={rec['win_frames']} "
        f"stable_now={rec['stable_now']} cov_max={rec['cov_max']}",
    ]


# ============================================================================
# 解析层
# ============================================================================

def test_parse_health_line_roundtrip():
    """等价状态机生成的健康行 -> watchdog 解析 -> 字段逐一一致。"""
    g = GuardState()
    align_bump(g, 5.0)
    for i in range(129):
        guard_feed(g, 5.0 + i / 43.0, 0.01, 0.0, 0.0, armed=True)
    snap = health_snapshot(g, 8.0, armed=True)
    parsed = lw.parse_health_line(format_health_line(snap))
    for k, v in snap.items():
        assert parsed.get(k) == v, f"{k}: {parsed.get(k)!r} != {v!r}"
    assert lw.parse_health_line("[px4_odom_convert] position delta NED") is None


def test_blackbox_stream_grouping():
    """blackbox 4 行 -> 一条完整事件 dict（jump/ekf/slam_q 三个 section）。"""
    g = GuardState()
    align_bump(g, -100.0)
    for i in range(86):
        guard_feed(g, i / 43.0, 1.0, 2.0, 0.0, armed=False)
    guard_feed(g, 2.0, 20.0, 25.0, 0.5, armed=True)
    rec = [r for r in g.rec if r["event"] == "L1_TRIGGER"][-1]
    stream = lw.BlackboxStream()
    out = None
    for line in blackbox_lines(rec):
        out = stream.feed(line)
    assert out is not None, "4 行喂完必须聚合出事件"
    assert out["event"] == "L1_TRIGGER"
    assert out["trans"] == "OK->L1_LOCK"
    assert out["jump"]["dpos_m"] == rec["dpos_m"]
    assert out["ekf"]["resets_total"] == rec["resets_total"]
    assert out["slam_q"]["win_frames"] == rec["win_frames"]
    assert lw.parse_health_line("[px4_odom_convert_blackbox] x") is None


# ============================================================================
# 判定层（健康行持续性条件）
# ============================================================================

def test_l1_sustained_alert_and_recovery():
    """L1 短暂(<10s)=WARN；持续(>=10s)=ERROR 升级即发；恢复=INFO 已恢复。"""
    clk = FakeClock()
    wd = lw.LocalizationWatchdog(wall=clk)
    lines = wd.feed_line(format_health_line(
        base_snap(guard="L1_LOCK", l1_lock=1, l1_age_s=2.0, l1_total=1,
                  hold_ratio=1.0, jump_max_m=26.3)))
    assert any("WARN" in l and "[L1_LOCK]" in l for l in lines)
    assert not any("ERROR" in l for l in lines)
    # 10s 后仍未恢复：升级 ERROR（升级立即发，不等 repeat 间隔）
    clk.advance(10.0)
    lines = wd.feed_line(format_health_line(
        base_snap(guard="L1_LOCK", l1_lock=1, l1_age_s=12.0, l1_total=1,
                  hold_ratio=1.0, jump_max_m=26.3)))
    assert any("ERROR" in l and "[L1_LOCK]" in l and "12s" in l for l in lines)
    # 恢复：INFO 已恢复
    lines = wd.feed_line(format_health_line(base_snap()))
    assert any("INFO" in l and "[L1_LOCK]" in l and "已恢复" in l for l in lines)
    assert not any("ERROR" in l for l in lines)


def test_storm_alert_and_clear():
    """storm=1 / guard 含 +STORM -> ERROR EKF_STORM_ACTIVE；解除 -> 已恢复。"""
    clk = FakeClock()
    wd = lw.LocalizationWatchdog(wall=clk)
    lines = wd.feed_line(format_health_line(
        base_snap(guard="L1_LOCK+STORM", storm=1, ekf_reset_60s=8)))
    assert any("ERROR" in l and "[EKF_STORM_ACTIVE]" in l and "8" in l
               for l in lines)
    lines = wd.feed_line(format_health_line(base_snap()))
    assert any("INFO" in l and "[EKF_STORM_ACTIVE]" in l and "已恢复" in l
               for l in lines)


def test_reset_elevated_warn():
    """未到风暴线但 reset 偏多（>=3/60s）-> WARN 预警。"""
    wd = lw.LocalizationWatchdog(wall=FakeClock())
    lines = wd.feed_line(format_health_line(base_snap(ekf_reset_60s=3)))
    assert any("WARN" in l and "[EKF_RESET_ELEVATED]" in l for l in lines)


def test_slam_stream_low_alert():
    """slam_hz=0（窗口无帧）-> ERROR SLAM_STREAM_LOW。"""
    wd = lw.LocalizationWatchdog(wall=FakeClock())
    lines = wd.feed_line(format_health_line(base_snap(slam_frames=0, slam_hz=0.0)))
    assert any("ERROR" in l and "[SLAM_STREAM_LOW]" in l for l in lines)


def test_hold_ratio_warn():
    """hold 占比 >50% -> WARN HOLD_RATIO_HIGH。"""
    wd = lw.LocalizationWatchdog(wall=FakeClock())
    lines = wd.feed_line(format_health_line(base_snap(hold_ratio=0.8)))
    assert any("WARN" in l and "[HOLD_RATIO_HIGH]" in l for l in lines)


def test_bump_rate_warn():
    """bump_total 差分速率 >6/min -> WARN BUMP_RATE_HIGH。"""
    clk = FakeClock()
    wd = lw.LocalizationWatchdog(wall=clk)
    wd.feed_line(format_health_line(base_snap(bump_total=1)))
    clk.advance(3.0)  # 3s 内又 bump 2 次 -> 40/min
    lines = wd.feed_line(format_health_line(base_snap(bump_total=3)))
    assert any("WARN" in l and "[BUMP_RATE_HIGH]" in l for l in lines)


# ============================================================================
# 停更检测（poll）
# ============================================================================

def test_health_stale_then_recover():
    """健康行停更 >10s -> ERROR HEALTH_STALE；恢复更新 -> INFO 已恢复。"""
    clk = FakeClock()
    wd = lw.LocalizationWatchdog(wall=clk)
    wd.feed_line(format_health_line(base_snap()))
    clk.advance(11.0)
    lines = wd.poll()
    assert any("ERROR" in l and "[HEALTH_STALE]" in l for l in lines)
    # repeat 窗口内不重发
    clk.advance(5.0)
    assert wd.poll() == []
    # 超过 repeat 间隔重发
    clk.advance(11.0)
    lines = wd.poll()
    assert any("[HEALTH_STALE]" in l for l in lines)
    # 恢复更新：新健康行喂入的瞬间即产生"已恢复"通知（后续 poll 无输出）
    clk.advance(1.0)
    lines = wd.feed_line(format_health_line(base_snap()))
    assert any("INFO" in l and "[HEALTH_STALE]" in l and "已恢复" in l
               for l in lines)
    assert wd.poll() == []


def test_health_stale_from_start():
    """从未收到健康行：启动 10s 后同样告警（节点未起/日志切走）。"""
    clk = FakeClock()
    wd = lw.LocalizationWatchdog(wall=clk)
    assert wd.poll() == []
    clk.advance(11.0)
    lines = wd.poll()
    assert any("[HEALTH_STALE]" in l for l in lines)


# ============================================================================
# blackbox 即时告警 + 去重 + 端到端
# ============================================================================

def test_blackbox_immediate_alerts():
    """L1_TRIGGER/L3_STORM_ON blackbox -> 立即 ERROR；恢复/接受 -> INFO。"""
    wd = lw.LocalizationWatchdog(wall=FakeClock())
    base = [
        "[px4_odom_convert_blackbox] event={ev} t=8123.456 trans={tr} "
        "armed=1 aligned=1",
        "[px4_odom_convert_blackbox]   jump dpos_m=26.352 dyaw_deg=143.2 "
        "delta_ned=[25.9, 4.3, 0.0] from_ned=[1.0, 2.0, 0.1] "
        "to_ned=[26.9, 6.3, 0.1]",
        "[px4_odom_convert_blackbox]   ekf ev_reset=1 resets_60s=6 "
        "resets_total=3764 storm=1 ekf_valid=1 xy_cnt=255 hdg_cnt=0",
        "[px4_odom_convert_blackbox]   slam_q win_max_m=26.352 "
        "win_mean_m=0.87 win_frames=129 stable_now=0 cov_max=0.0123",
    ]

    def feed_ev(ev, tr):
        lines = []
        for l in base:
            lines.extend(wd.feed_line(l.format(ev=ev, tr=tr)))
        return lines

    lines = feed_ev("L1_TRIGGER", "OK->L1_LOCK")
    assert any("ERROR" in l and "[L1_TRIGGERED]" in l and "26.352" in l
               for l in lines)
    lines = feed_ev("L3_STORM_ON", "OK->STORM")
    assert any("ERROR" in l and "[EKF_STORM_ONSET]" in l for l in lines)
    lines = feed_ev("L1_RECOVER_INPLACE", "L1_LOCK->OK")
    assert any("INFO" in l and "[L1_RECOVERED]" in l for l in lines)
    lines = feed_ev("L3_STORM_CLEAR", "STORM->OK")
    assert any("INFO" in l and "[EKF_STORM_CLEARED]" in l for l in lines)


def test_repeat_dedup_window():
    """同一持续条件：首帧发出；repeat 窗口内静默；超窗重发。"""
    clk = FakeClock()
    wd = lw.LocalizationWatchdog(wall=clk)
    line = format_health_line(base_snap(guard="OK+STORM", storm=1,
                                        ekf_reset_60s=7))
    lines = wd.feed_line(line)
    assert len([l for l in lines if "EKF_STORM_ACTIVE" in l]) == 1
    clk.advance(5.0)
    assert wd.feed_line(line) == []
    clk.advance(11.0)
    lines = wd.feed_line(line)
    assert len([l for l in lines if "EKF_STORM_ACTIVE" in l]) == 1


def test_end_to_end_log_replay():
    """端到端：等价状态机回放 悬停->L1 疯跳->持续熔断->EKF 风暴->SLAM 断流，
    真实日志流（健康行 + blackbox 4 行）喂入 watchdog，醒目告警按预期产出。"""
    clk = FakeClock()
    wd = lw.LocalizationWatchdog(wall=clk)
    g = GuardState()
    align_bump(g, 0.0)

    # 阶段 1：正常悬停 3s + 健康行 -> 零告警
    for i in range(129):
        guard_feed(g, i / 43.0, 0.01, 0.0, 0.0, armed=True)
    assert wd.feed_line(format_health_line(health_snapshot(g, 3.0, armed=True))) == []
    health_reset_windows(g)

    # 阶段 2：单帧疯跳 -> L1 blackbox -> ERROR L1_TRIGGERED
    guard_feed(g, 3.02, 30.0, 0.0, 0.0, armed=True)
    trig = [r for r in g.rec if r["event"] == "L1_TRIGGER"][-1]
    lines = []
    for l in blackbox_lines(trig):
        lines.extend(wd.feed_line(l))
    assert any("ERROR" in l and "[L1_TRIGGERED]" in l for l in lines)

    # 阶段 3：armed 疯跳持续 12s -> 健康行 ERROR L1_LOCK（持续未恢复）
    rng = random.Random(1)
    for i in range(516):
        t = 3.05 + i / 43.0
        guard_feed(g, t, rng.uniform(-50, 50), rng.uniform(-50, 50), 0.0,
                   armed=True)
    snap = health_snapshot(g, 15.2, armed=True)
    assert snap["guard"] == "L1_LOCK" and snap["l1_age_s"] >= 10.0
    lines = wd.feed_line(format_health_line(snap))
    assert any("ERROR" in l and "[L1_LOCK]" in l for l in lines)

    # 阶段 4：EKF reset 风暴叠加 -> L3 blackbox + 健康行 ERROR EKF_STORM_ACTIVE
    for i in range(6):
        on_ekf_reset(g, 16.0 + i, armed=True)
    storm_rec = [r for r in g.rec if r["event"] == "L3_STORM_ON"][-1]
    lines = []
    for l in blackbox_lines(storm_rec):
        lines.extend(wd.feed_line(l))
    assert any("ERROR" in l and "[EKF_STORM_ONSET]" in l for l in lines)
    snap = health_snapshot(g, 22.5, armed=True)
    assert "+STORM" in snap["guard"]
    lines = wd.feed_line(format_health_line(snap))
    assert any("ERROR" in l and "[EKF_STORM_ACTIVE]" in l for l in lines)

    # 阶段 5：SLAM 断流（窗口清零后无新帧）-> ERROR SLAM_STREAM_LOW
    health_reset_windows(g)
    snap = health_snapshot(g, 25.5, armed=True)
    assert snap["slam_hz"] == 0.0
    lines = wd.feed_line(format_health_line(snap))
    assert any("ERROR" in l and "[SLAM_STREAM_LOW]" in l for l in lines)

    # 全程告警行均为醒目格式：[localization_watchdog] ... LEVEL [CODE] ...
    for l in wd.emitted:
        assert l.startswith("[localization_watchdog] ")
        assert " [" in l and "]" in l


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"[PASS] {fn.__name__}")
    print(f"\nlocalization_watchdog 全部 {len(fns)} 项断言通过")
