"""orin_watchdog 规则引擎单测（零硬件：不碰 ROS，纯快照驱动）。

覆盖：armed/空中门控、断流介入、OFFBOARD 丢失持续判定、位置跳变连击、
倾覆 KILL 授权闸门、低电加固、failsafe 加固、冷却与每段飞行一次性、
上锁复位后可再次介入。
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]


def load_module(name: str, file: Path):
    spec = importlib.util.spec_from_file_location(name, file)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


wd = load_module("orin_watchdog", REPO / "tools" / "orin_watchdog.py")


def snap(t, **kw):
    """空中飞行中的基准快照（armed、OFFBOARD、0.5m 高度、电量充足）。"""
    s = {"t": float(t), "armed": True, "nav_state": 14, "failsafe": False,
         "z": -0.5, "roll_deg": 2.0, "pitch_deg": -1.0, "batt_remaining": 0.9,
         "last_seen_t": float(t), "pos_jump": False}
    s.update(kw)
    return s


def actions_of(rules, s):
    return [a["action"] for a in rules.update(s)]


# ---------------------------------------------------------------- 门控：不上锁不动作

def test_disarmed_never_acts():
    r = wd.WatchdogRules(enable_kill=True)
    for t in range(20):
        s = snap(t, armed=False, nav_state=2, z=0.0, failsafe=True,
                 batt_remaining=0.01, last_seen_t=t - 100, pos_jump=True,
                 roll_deg=80.0)
        assert r.update(s) == []


def test_ground_z_means_not_airborne():
    r = wd.WatchdogRules()
    # armed 但 z 在地面：断流/掉模式/低电都不介入（地面处置留给人）
    s = snap(0.0, z=-0.05, nav_state=2, batt_remaining=0.05, last_seen_t=-100)
    assert r.update(s) == []


def test_blind_never_seen_no_action():
    r = wd.WatchdogRules()
    # last_seen_t=None：栈还没起完/桥没通，不能上来就 LAND
    s = snap(10.0, last_seen_t=None)
    assert r.update(s) == []


# ---------------------------------------------------------------- BLIND 断流

def test_blind_fires_land_once_per_flight():
    r = wd.WatchdogRules(blind_timeout_s=1.5)
    a1 = r.update(snap(10.0, last_seen_t=8.0))   # 断流 2s
    assert [a["action"] for a in a1] == ["LAND"]
    assert a1[0]["rule"] == "BLIND"
    # 同段飞行内不重复
    assert r.update(snap(11.0, last_seen_t=8.0)) == []


def test_blind_within_timeout_quiet():
    r = wd.WatchdogRules(blind_timeout_s=1.5)
    assert r.update(snap(10.0, last_seen_t=9.0)) == []


# ---------------------------------------------------------------- OFFBOARD_LOST

def test_offboard_loss_needs_sustain():
    r = wd.WatchdogRules(offboard_loss_sustain_s=1.5)
    assert r.update(snap(10.0, nav_state=2)) == []          # 掉出瞬间不介入
    assert r.update(snap(11.0, nav_state=2)) == []          # 1.0s 未到阈
    a = r.update(snap(11.6, nav_state=2))                   # 1.6s 到阈
    assert [x["action"] for x in a] == ["LAND"]
    assert a[0]["rule"] == "OFFBOARD_LOST"


def test_offboard_glitch_recovers_quietly():
    r = wd.WatchdogRules(offboard_loss_sustain_s=1.5)
    r.update(snap(10.0, nav_state=2))
    r.update(snap(10.5, nav_state=14))                      # 抖动回去
    assert r.update(snap(12.0, nav_state=14)) == []


def test_landing_states_not_treated_as_loss():
    r = wd.WatchdogRules()
    for nav in (8, 5, 10):                                  # LAND/RTL/PRECLAND
        for t in (10.0, 12.0):
            assert r.update(snap(t, nav_state=nav)) == []


def test_offboard_loss_disabled_by_flag():
    r = wd.WatchdogRules(land_on_offboard_loss=False)
    for t in (10.0, 12.0, 14.0):
        assert r.update(snap(t, nav_state=2)) == []


# ---------------------------------------------------------------- POS_JUMP 位置跳变

def test_pos_jump_needs_streak():
    r = wd.WatchdogRules()
    assert r.update(snap(10.0, pos_jump=True)) == []        # 单帧跳变不介入
    a = r.update(snap(10.1, pos_jump=True))                 # 连击第 2 帧
    assert [x["action"] for x in a] == ["LAND"]
    assert a[0]["rule"] == "POS_JUMP"


def test_pos_jump_streak_broken_by_clean_frame():
    r = wd.WatchdogRules()
    r.update(snap(10.0, pos_jump=True))
    r.update(snap(10.1))                                    # 干净帧清零
    assert r.update(snap(10.2, pos_jump=True)) == []


# ---------------------------------------------------------------- TILT_RUNAWAY 倾覆

def test_tilt_alert_without_kill_authorization():
    r = wd.WatchdogRules(enable_kill=False, tilt_sustain_s=0.8)
    assert r.update(snap(10.0, roll_deg=70.0)) == []
    a = r.update(snap(11.0, roll_deg=70.0))                 # 持续 1.0s > 0.8s
    assert [x["action"] for x in a] == ["ALERT"]            # 未授权 KILL 只告警
    assert a[0]["rule"] == "TILT_RUNAWAY"


def test_tilt_kill_with_authorization():
    r = wd.WatchdogRules(enable_kill=True, tilt_sustain_s=0.8)
    r.update(snap(10.0, pitch_deg=-65.0))
    a = r.update(snap(10.9, pitch_deg=-65.0))
    assert [x["action"] for x in a] == ["KILL"]


def test_tilt_spike_shorter_than_sustain_quiet():
    r = wd.WatchdogRules(enable_kill=True, tilt_sustain_s=0.8)
    r.update(snap(10.0, roll_deg=70.0))
    r.update(snap(10.5, roll_deg=2.0))                      # 瞬时大角度回落
    assert r.update(snap(11.0, roll_deg=2.0)) == []


# ---------------------------------------------------------------- BATT_CRIT / FAILSAFE

def test_batt_crit_fires_land_airborne():
    r = wd.WatchdogRules(batt_crit=0.10)
    a = r.update(snap(10.0, batt_remaining=0.09))
    assert [x["action"] for x in a] == ["LAND"]
    assert a[0]["rule"] == "BATT_CRIT"


def test_failsafe_sustained_fires_land():
    r = wd.WatchdogRules(failsafe_sustain_s=0.5)
    assert r.update(snap(10.0, failsafe=True)) == []
    a = r.update(snap(10.6, failsafe=True))
    assert [x["action"] for x in a] == ["LAND"]
    assert a[0]["rule"] == "FAILSAFE"


# ---------------------------------------------------------------- 复位：上锁后重新武装

def test_disarm_rearms_rules_for_next_flight():
    r = wd.WatchdogRules(blind_timeout_s=1.5)
    assert [a["action"] for a in r.update(snap(10.0, last_seen_t=8.0))] == ["LAND"]
    assert r.update(snap(11.0, last_seen_t=8.0)) == []      # 同段飞行不重复
    r.update(snap(12.0, armed=False, z=0.0))                # 落地
    assert [a["action"] for a in r.update(snap(13.0, last_seen_t=8.0))] == ["LAND"]
