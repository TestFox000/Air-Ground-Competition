"""uav_link 纯函数单测（零硬件：不开 SSH、不碰 rclpy、不连无人机）。

覆盖：NDJSON 采样行解析（含坏行容错）、黑匣子写读往返、QoS 选择、限速器、
四元数→欧拉角数值、告警规则状态机逐条触发/不触发、ctl 安全闸门与倒计时、
命令映射表渲染、机载采样器源码契约、report 对构造 jsonl 的分级输出。
"""

from __future__ import annotations

import importlib.util
import json
import math
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]


def load_module(name: str, file: Path):
    spec = importlib.util.spec_from_file_location(name, file)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


ul = load_module("uav_link", REPO / "tools" / "uav_link.py")


# ---------------------------------------------------------------- 四元数 → 欧拉角

def test_quat_to_euler_identity():
    assert ul.quat_to_euler_deg(1.0, 0.0, 0.0, 0.0) == pytest.approx((0.0, 0.0, 0.0))


def test_quat_to_euler_single_axes():
    c, s = math.cos(math.radians(45)), math.sin(math.radians(45))
    # 绕 z 转 90° → yaw=90
    assert ul.quat_to_euler_deg(c, 0.0, 0.0, s) == pytest.approx((0.0, 0.0, 90.0))
    # 绕 x 转 30° → roll=30
    c2, s2 = math.cos(math.radians(15)), math.sin(math.radians(15))
    assert ul.quat_to_euler_deg(c2, s2, 0.0, 0.0) == pytest.approx((30.0, 0.0, 0.0))
    # 绕 y 转 -45° → pitch=-45
    c3, s3 = math.cos(math.radians(-22.5)), math.sin(math.radians(-22.5))
    assert ul.quat_to_euler_deg(c3, 0.0, s3, 0.0) == pytest.approx((0.0, -45.0, 0.0))


def test_quat_to_euler_near_gimbal_lock_no_crash():
    # pitch=+90° 万向锁附近不得数值越界崩溃
    c, s = math.cos(math.radians(45)), math.sin(math.radians(45))
    r, p, _y = ul.quat_to_euler_deg(c, 0.0, s, 0.0)
    assert p == pytest.approx(90.0)
    assert math.isfinite(r)


def test_wrap_deg():
    assert ul._wrap_deg(190.0) == pytest.approx(-170.0)
    assert ul._wrap_deg(-190.0) == pytest.approx(170.0)
    assert ul._wrap_deg(10.0) == pytest.approx(10.0)


def test_percentile():
    assert ul._percentile([0.0, 1.0, 2.0, 3.0, 4.0], 95) == pytest.approx(3.8)
    assert ul._percentile([0.0, 1.0, 2.0, 3.0, 4.0], 50) == pytest.approx(2.0)
    assert ul._percentile([5.0], 95) == 5.0
    with pytest.raises(ValueError):
        ul._percentile([], 95)


# ---------------------------------------------------------------- NDJSON 采样行解析

def test_parse_sampler_line_sample():
    obj = ul.parse_sampler_line(json.dumps(
        {"t": 1.5, "topic": "/fmu/out/vehicle_attitude", "ch": "attitude",
         "hz": 10.0, "data": {"rpy_deg": [1, 2, 3]}}))
    assert obj["kind"] == "sample"
    assert obj["t"] == 1.5 and obj["ch"] == "attitude" and obj["hz"] == 10.0
    assert obj["data"] == {"rpy_deg": [1, 2, 3]}


def test_parse_sampler_line_classifies_without_ch():
    # 采样器没给 ch 时笔记本端按话题名分类
    obj = ul.parse_sampler_line(json.dumps(
        {"t": 1.0, "topic": "/fmu/out/vehicle_attitude_setpoint", "data": {}}))
    assert obj["ch"] == "attitude_target"
    obj2 = ul.parse_sampler_line(json.dumps(
        {"t": 1.0, "topic": "/fmu/in/vehicle_command", "data": {}}))
    assert obj2["ch"] is None


def test_parse_sampler_line_hello_error_ctl():
    h = ul.parse_sampler_line(json.dumps(
        {"hello": "uav_sampler", "topics_found": ["/a", "/b"]}))
    assert h["kind"] == "hello" and h["topics_found"] == ["/a", "/b"]
    e = ul.parse_sampler_line(json.dumps({"error": "rclpy 导入失败: x"}))
    assert e["kind"] == "error" and e["error"].startswith("rclpy 导入失败")
    c = ul.parse_sampler_line(json.dumps({"ctl": "arm", "ok": True, "detail": "d"}))
    assert c["kind"] == "ctl" and c["action"] == "arm" and c["ok"] is True
    c2 = ul.parse_sampler_line(json.dumps({"src": "ctl", "action": "land",
                                           "ok": False, "detail": "x"}))
    assert c2["kind"] == "ctl" and c2["action"] == "land" and c2["ok"] is False
    # 采样器醒目提示行（M1 直发警告等，执行前打印）
    w = ul.parse_sampler_line(json.dumps({"warn": "直发飞控，绕过状态机防护"}))
    assert w["kind"] == "warn" and "直发" in w["msg"]


def test_parse_sampler_line_tolerates_bad_rows():
    assert ul.parse_sampler_line("") is None
    assert ul.parse_sampler_line("not json at all") is None
    assert ul.parse_sampler_line("{bad json") is None
    assert ul.parse_sampler_line("[1,2,3]") is None
    assert ul.parse_sampler_line(json.dumps({"foo": 1})) is None
    # data 不是字典 → 置空字典，行仍可用
    obj = ul.parse_sampler_line(json.dumps({"topic": "/x", "data": [1, 2]}))
    assert obj is not None and obj["data"] == {}
    # t/hz 非法 → None，不崩
    obj2 = ul.parse_sampler_line(json.dumps({"topic": "/x", "t": "abc"}))
    assert obj2 is not None and obj2["t"] is None


# ---------------------------------------------------------------- 话题分类（顺序敏感）

def test_classify_topic():
    assert ul.classify_topic("/fmu/out/vehicle_attitude") == "attitude"
    # setpoint 名字包含 attitude：必须先中 attitude_target
    assert ul.classify_topic("/fmu/out/vehicle_attitude_setpoint") == "attitude_target"
    assert ul.classify_topic("/fmu/out/vehicle_angular_velocity") == "rates"
    assert ul.classify_topic("/fmu/out/vehicle_local_position") == "position"
    assert ul.classify_topic("/fmu/out/battery_status") == "battery"
    assert ul.classify_topic("/fmu/out/vehicle_status") == "status"
    assert ul.classify_topic("/fmu/out/estimator_status_flags") == "estimator"
    assert ul.classify_topic("/fmu/out/sensor_combined") == "imu"
    assert ul.classify_topic("/fmu/in/vehicle_command") is None
    assert ul.classify_topic("/odom/localization") is None


# ---------------------------------------------------------------- 黑匣子写读往返

def test_blackbox_roundtrip(tmp_path):
    p = tmp_path / "bb.jsonl"
    box = ul.BlackboxWriter(p)
    box.write({"kind": "sample", "topic": "/x", "data": {"a": 1}}, recv=100.0)
    box.write({"hello": "uav_sampler", "topics_found": ["/x"]}, recv=100.1)
    box.close()
    # 混入坏行（Ctrl-C 截尾/链路毛边），读取必须容错跳过
    with open(p, "a", encoding="utf-8", newline="\n") as fh:
        fh.write("{truncated json\n\nnot json\n")
    rows = ul.read_blackbox(p)
    assert len(rows) == 2
    assert rows[0]["recv"] == 100.0 and rows[0]["topic"] == "/x"
    assert rows[1]["topics_found"] == ["/x"]


def test_blackbox_write_oserror_disables_not_kills(tmp_path, capsys):
    """M2：写盘失败（盘满/权限）响亮警告一次后停用黑匣子，遥测流程不被杀死
    （对齐 orin_flight_blackbox"落盘失败绝不拖死节点"语义）。"""
    p = tmp_path / "bb.jsonl"
    box = ul.BlackboxWriter(p)
    box.write({"a": 1}, recv=1.0)

    class _BadFH:   # 模拟盘满：write/flush 都抛 OSError
        def write(self, _s):
            raise OSError(28, "No space left on device")

        def flush(self):
            raise OSError(28, "No space left on device")

        def close(self):
            pass

    box._fh.close()                    # 换掉真句柄，避免 Windows 文件锁残留
    box._fh = _BadFH()
    box.write({"b": 2}, recv=2.0)      # 不抛，响亮警告一次
    box.write({"c": 3}, recv=3.0)      # 已停用：静默空操作，不再刷屏
    box.close()                        # 停用后 close 不抛
    err = capsys.readouterr().err
    assert err.count("黑匣子写盘失败") == 1
    assert "遥测继续" in err
    # 失败前的行还在盘上，失败行没写进去也没炸
    rows = ul.read_blackbox(p)
    assert len(rows) == 1 and rows[0]["a"] == 1


def test_blackbox_append_mode_keeps_history(tmp_path):
    p = tmp_path / "bb.jsonl"
    ul.BlackboxWriter(p).close()
    box = ul.BlackboxWriter(p, append=True)
    box.write({"src": "ctl", "action": "arm", "ok": True})
    box.close()
    rows = ul.read_blackbox(p)
    assert len(rows) == 1 and rows[0]["src"] == "ctl"
    # recv 由写入器盖上
    assert rows[0]["recv"] > 0


# ---------------------------------------------------------------- QoS 选择

def test_choose_qos():
    # 空列表 = DDS 发现未跑完的竞态态：订阅端按 best_effort 永远兼容
    # （reliable 订阅收不到 best_effort 发布——2026-10-16 实机静默踩坑）
    assert ul.choose_qos([]) == "best_effort"
    assert ul.choose_qos(["reliable"]) == "reliable"
    assert ul.choose_qos(["system_default"]) == "reliable"
    assert ul.choose_qos(["best_effort"]) == "best_effort"
    # 任一发布端 best_effort，订阅端必须同档（track_diag 真车踩过的坑）
    assert ul.choose_qos(["reliable", "best_effort"]) == "best_effort"
    assert ul.choose_qos(["BEST_EFFORT"]) == "best_effort"


# ---------------------------------------------------------------- 限速器

def test_rate_limiter_per_topic_cap():
    lim = ul.RateLimiter({"attitude": 10.0}, default_hz=1.0)
    assert lim.allow("attitude", 0.0)          # 首帧必过
    assert not lim.allow("attitude", 0.05)     # 10Hz → 间隔 0.1s
    assert lim.allow("attitude", 0.1)
    assert lim.allow("status", 0.15)           # 不同话题独立计时
    assert not lim.allow("status", 0.5)        # status 默认 1Hz
    assert lim.allow("status", 1.2)


def test_rate_limiter_on_change_bypass():
    lim = ul.RateLimiter({}, default_hz=1.0)
    assert lim.allow("status", 0.0)
    # 周期内，但状态变了（on-change）→ 直通
    assert lim.allow("status", 0.1, changed=True)
    assert not lim.allow("status", 0.2)


# ---------------------------------------------------------------- 告警规则引擎

def _drive(engine, states, dt=0.1):
    """按脚本化时间序列驱动规则引擎，返回全部告警。"""
    fired = []
    for i, st in enumerate(states):
        fired.extend(engine.update(i * dt, st))
    return fired


def _quiet_engine():
    e = ul.UavRuleEngine()
    e.expect_dzuav = False     # 非 dzuav 用例别被 DZUAV_SILENT 干扰
    return e


def test_rule_failsafe_fires_immediately():
    e = _quiet_engine()
    fired = _drive(e, [{"failsafe": True}] * 30)   # 3s 持续 failsafe
    fs = [a for a in fired if a["rule"] == "FAILSAFE"]
    assert len(fs) == 1                            # 立即报一次，冷却内不重复
    assert fs[0]["t"] == pytest.approx(0.0)
    # failsafe 消失 → 不报
    e2 = _quiet_engine()
    assert _drive(e2, [{"failsafe": False}] * 30) == []


def test_rule_ekf_degraded_needs_sustain():
    e = _quiet_engine()
    fired = _drive(e, [{"estimator_ok": False}] * 30)   # 3s 异常
    ekf = [a for a in fired if a["rule"] == "EKF_DEGRADED"]
    assert len(ekf) == 1
    assert ekf[0]["t"] == pytest.approx(2.0, abs=0.15)
    # 只异常 1s 就恢复 → 不报
    e2 = _quiet_engine()
    seq = [{"estimator_ok": False}] * 10 + [{"estimator_ok": True}] * 20
    assert _drive(e2, seq) == []
    # 没给 estimator 数据（None）→ 不评不炸
    e3 = _quiet_engine()
    assert _drive(e3, [{}] * 30) == []


def test_estimator_flags_ok_ignores_solution_status():
    """S2：solution_status_flags 是"哪些状态输出有效"的位掩码（健康飞行恒非零），
    绝不参与零判定；ok 只由 filter_fault_flags + innovation_check_flags 决定。"""
    # 健康飞行假消息：solution_status_flags 非零、坏 flag 全零 → 健康
    healthy = {"filter_fault_flags": 0, "innovation_check_flags": 0,
               "solution_status_flags": 0x1FF, "control_mode_flags": 0x7FFF}
    assert ul.estimator_flags_ok(healthy) is True
    # 坏 flag 任一非零 → 坏（即使 solution_status_flags 也在）
    assert ul.estimator_flags_ok({**healthy, "filter_fault_flags": 4}) is False
    assert ul.estimator_flags_ok({**healthy, "innovation_check_flags": 1}) is False
    # 只有 solution_status_flags（没有坏 flag 字段）→ 信息不足 None，不评
    assert ul.estimator_flags_ok({"solution_status_flags": 0}) is None
    assert ul.estimator_flags_ok({}) is None
    # 采样器源码必须同源（内嵌副本，无 rclpy 跑不了 slim 单测）
    assert "estimator_flags_ok 同源" in ul.SAMPLER_SRC or \
        "solution_status_flags" in ul.SAMPLER_SRC


def test_ekf_healthy_flight_flags_no_alert_via_feed():
    """S2 整合：健康飞行的 estimator 采样行（solution_status_flags 非零）
    经 TelemetryState.feed → 引擎，不得触发 EKF_DEGRADED。"""
    st = ul.TelemetryState()
    e = _quiet_engine()
    fired = []
    for i in range(40):
        t = i * 0.1
        st.feed(t, {"kind": "sample", "t": t, "topic": "/fmu/out/estimator_status",
                    "ch": "estimator", "hz": 1.0,
                    "data": {"flags": {"filter_fault_flags": 0,
                                       "innovation_check_flags": 0,
                                       "solution_status_flags": 479},
                             "ok": True}})
        fired.extend(e.update(t, st.engine_input()))
    assert [a for a in fired if a["rule"] == "EKF_DEGRADED"] == []
    assert st.estimator_ok is True
    # 对照：filter_fault_flags 非零持续 2s → 报
    st2 = ul.TelemetryState()
    e2 = _quiet_engine()
    fired2 = []
    for i in range(40):
        t = i * 0.1
        st2.feed(t, {"kind": "sample", "t": t, "topic": "/fmu/out/estimator_status",
                     "ch": "estimator", "hz": 1.0,
                     "data": {"flags": {"filter_fault_flags": 8,
                                        "innovation_check_flags": 0,
                                        "solution_status_flags": 479},
                              "ok": True}})   # ok 字段说谎也要被 flags 纠正
        fired2.extend(e2.update(t, st2.engine_input()))
    ekf = [a for a in fired2 if a["rule"] == "EKF_DEGRADED"]
    assert len(ekf) == 1 and st2.estimator_ok is False


def test_rule_att_diverge():
    e = _quiet_engine()
    st = {"att_deg": (0.0, 0.0, 0.0), "att_target_deg": (20.0, 0.0, 0.0)}
    fired = _drive(e, [st] * 20)                        # 偏差 20° 持续 2s
    ad = [a for a in fired if a["rule"] == "ATT_DIVERGE"]
    assert len(ad) == 1
    assert ad[0]["t"] == pytest.approx(1.0, abs=0.15)
    # 偏差 10° → 不报
    e2 = _quiet_engine()
    st2 = {"att_deg": (0.0, 0.0, 0.0), "att_target_deg": (10.0, 0.0, 0.0)}
    assert _drive(e2, [st2] * 30) == []
    # 没有姿态目标 → 永不评（不报）
    e3 = _quiet_engine()
    st3 = {"att_deg": (0.0, 0.0, 0.0)}
    assert _drive(e3, [st3] * 30) == []


def test_rule_rate_sat():
    e = _quiet_engine()
    fired = _drive(e, [{"rate_dps": (350.0, 0.0, 0.0)}] * 20)
    rs = [a for a in fired if a["rule"] == "RATE_SAT"]
    assert len(rs) == 1
    assert "350" in rs[0]["msg"]
    # 角速度正常 → 不报
    e2 = _quiet_engine()
    assert _drive(e2, [{"rate_dps": (100.0, 50.0, 0.0)}] * 30) == []


def test_rule_vibe_clip_rising():
    e = _quiet_engine()
    seq = [{"clip_count": 0}] * 5 + [{"clip_count": 3}] * 5
    fired = _drive(e, seq)
    vc = [a for a in fired if a["rule"] == "VIBE_CLIP"]
    assert len(vc) == 1 and "0→3" in vc[0]["msg"]
    # 计数不变 → 不报
    e2 = _quiet_engine()
    assert _drive(e2, [{"clip_count": 5}] * 30) == []
    # 计数回落（飞控重启）→ 静默重设基线，不报
    e3 = _quiet_engine()
    seq3 = [{"clip_count": 10}] * 5 + [{"clip_count": 2}] * 5
    assert _drive(e3, seq3) == []


def test_rule_bat_low():
    e = _quiet_engine()
    fired = _drive(e, [{"battery_remaining": 0.15}] * 30)
    bl = [a for a in fired if a["rule"] == "BAT_LOW"]
    assert len(bl) == 1 and "15%" in bl[0]["msg"]
    # 恰好 20% 不报（阈值是 <20%），50% 更不报
    e2 = _quiet_engine()
    assert _drive(e2, [{"battery_remaining": 0.20}] * 30) == []
    e3 = _quiet_engine()
    assert _drive(e3, [{"battery_remaining": 0.5}] * 30) == []


def test_rule_dzuav_silent():
    e = ul.UavRuleEngine()                      # expect_dzuav 默认 True
    fired = _drive(e, [{"dzuav_t": None}] * 70)   # 7s 无健康行
    dz = [a for a in fired if a["rule"] == "DZUAV_SILENT"]
    assert len(dz) == 1
    assert dz[0]["t"] == pytest.approx(5.0, abs=0.2)
    # 健康行持续刷新 → 不报
    e2 = ul.UavRuleEngine()
    seq = [{"dzuav_t": i * 0.1} for i in range(70)]
    assert _drive(e2, seq) == []
    # --no-dzuav：不评
    e3 = _quiet_engine()
    assert _drive(e3, [{"dzuav_t": None}] * 70) == []


def test_dzuav_null_rows_trigger_silent_and_null_alert():
    """S1 整合：采样器每秒都发 dzuav.health 行（即使 data=null）——feed 必须只在
    data 非空时刷新活跃时间戳，否则 dzuav 真挂了告警永不触发；同时 data 持续
    null 要有独立的 DZUAV_NULL 告警（dzuav 活着但健康源缺席，宁可误报不可漏报）。"""
    st = ul.TelemetryState()
    e = ul.UavRuleEngine()
    fired = []
    for i in range(9):                          # 9 秒 null 心跳（1Hz，同采样器）
        t = float(i)
        st.feed(t, {"kind": "sample", "t": t, "topic": "dzuav.health",
                    "ch": "dzuav", "hz": 1.0, "data": {}})
        fired.extend(e.update(t, st.engine_input()))
    rules = {a["rule"] for a in fired}
    assert "DZUAV_SILENT" in rules              # null 行不得糊住 SILENT
    assert "DZUAV_NULL" in rules                # 独立告警：通道在、数据空
    # 对照：data 非空的正常健康行 → 两条都不报
    st2 = ul.TelemetryState()
    e2 = ul.UavRuleEngine()
    fired2 = []
    for i in range(9):
        t = float(i)
        st2.feed(t, {"kind": "sample", "t": t, "topic": "dzuav.health",
                     "ch": "dzuav", "hz": 1.0,
                     "data": {"localized": True, "landmarks": 3}})
        fired2.extend(e2.update(t, st2.engine_input()))
    assert fired2 == []
    assert st2.dzuav_null_since is None and st2.dzuav_t == 8.0
    # null → 恢复：null 计时清零，告警停
    st3 = ul.TelemetryState()
    st3.feed(0.0, {"kind": "sample", "t": 0.0, "topic": "dzuav.health",
                   "ch": "dzuav", "hz": 1.0, "data": {}})
    st3.feed(1.0, {"kind": "sample", "t": 1.0, "topic": "dzuav.health",
                   "ch": "dzuav", "hz": 1.0, "data": {"localized": False}})
    assert st3.dzuav_null_since is None and st3.dzuav_t == 1.0


def test_rule_link_lag():
    # 声明 10Hz（期望间隔 0.1s），实际 0.25s 一帧 → p95 超 2 倍期望 → 报
    e = _quiet_engine()
    fired = []
    for i in range(14):
        t = i * 0.25
        e.note_sample(t, "/fmu/out/vehicle_attitude", 10.0)
        fired.extend(e.update(t, {}))
    lag = [a for a in fired if a["rule"] == "LINK_LAG"]
    assert len(lag) == 1 and "2.5" in lag[0]["msg"]
    # 间隔正常 → 不报
    e2 = _quiet_engine()
    fired2 = []
    for i in range(60):
        t = i * 0.1
        e2.note_sample(t, "/fmu/out/vehicle_attitude", 10.0)
        fired2.extend(e2.update(t, {}))
    assert fired2 == []
    # 低 hz 通道不参与 LINK_LAG
    e3 = _quiet_engine()
    fired3 = []
    for i in range(20):
        t = i * 1.0
        e3.note_sample(t, "/fmu/out/vehicle_status", 1.0)
        fired3.extend(e3.update(t, {}))
    assert fired3 == []


def test_rule_healthy_flight_silent():
    e = _quiet_engine()
    st = {"failsafe": False, "estimator_ok": True,
          "att_deg": (1.0, -2.0, 80.0), "att_target_deg": (2.0, -2.0, 80.0),
          "rate_dps": (10.0, 5.0, 3.0), "clip_count": 0,
          "battery_remaining": 0.8}
    fired = []
    for i in range(40):
        t = i * 0.1
        e.note_sample(t, "/fmu/out/vehicle_attitude", 10.0)
        fired.extend(e.update(t, st))
    assert fired == []


# ---------------------------------------------------------------- ctl 闸门 / 倒计时 / 通道

def test_gate_requires_yes_for_high_risk():
    # kill 已升到最高风险级（S3）：--yes 之外另有全词确认，见 test_kill_word_confirm
    for a in ("arm", "takeoff", "stack-restart", "kill"):
        assert ul.check_gate(a, False) is not None   # 无 --yes 拒
        assert ul.check_gate(a, True) is None        # 有 --yes 放行
    # disarm/land/return 免 --yes（靠倒计时兜底），stack-status 只读
    for a in ("disarm", "land", "return", "stack-status"):
        assert ul.check_gate(a, False) is None
    # kill 的拒绝文本要点名第二道闸门
    assert "kill" in ul.check_gate("kill", False)


def test_kill_word_confirm():
    """S3：kill 在 --yes 之外必须逐字输入全词 "kill"（对齐 flight_console 强度：
    shell 历史里 `ctl land --yes` 与 `ctl kill --yes` 只差一个上翻键）。"""
    lines = []
    # 逐字全词 → 放行；警告文本点名动作语义
    assert ul.confirm_word("kill", ask=lambda _p: "kill",
                           out=lines.append, is_tty=True) is True
    assert any("最后手段" in ln for ln in lines)
    # 前后空白宽容，错词/缩写/空输入 → 拒
    assert ul.confirm_word("kill", ask=lambda _p: " kill ",
                           out=lines.append, is_tty=True) is True
    for bad in ("k", "KILL", "kil", "y", ""):
        assert ul.confirm_word("kill", ask=lambda _p, b=bad: b,
                               out=lines.append, is_tty=True) is False
    # Ctrl-C / EOF 中断 → 拒
    def _interrupt(_p):
        raise KeyboardInterrupt
    assert ul.confirm_word("kill", ask=_interrupt, out=lines.append,
                           is_tty=True) is False
    # 无交互终端 → 拒（默认什么都不做）
    assert ul.confirm_word("kill", ask=lambda _p: "kill",
                           out=lines.append, is_tty=False) is False
    # 非 WORD_CONFIRM 动作直通
    assert ul.confirm_word("land", is_tty=False) is True
    assert ul.confirm_word("arm", is_tty=False) is True


def test_gate_blocks_kill_via_main(capsys):
    # kill 没 --yes：碰网络之前就拒（回归：旧版 kill 只靠倒计时，--yes 直接跳）
    assert ul.main(["ctl", "kill"]) == 2
    assert "--yes" in capsys.readouterr().err


def test_gate_blocks_arm_via_main(capsys):
    # 默认什么都不做：闸门在碰网络之前就拒
    assert ul.main(["ctl", "arm"]) == 2
    assert "--yes" in capsys.readouterr().err
    assert ul.main(["ctl", "takeoff"]) == 2
    assert ul.main(["ctl", "stack-restart"]) == 2


def test_countdown_skipped_with_yes():
    slept, lines = [], []
    ul.countdown("land", yes=True, out=lines.append, sleep=slept.append)
    assert slept == [] and lines == []


def test_countdown_runs_without_yes():
    slept, lines = [], []
    ul.countdown("land", yes=False, out=lines.append, sleep=slept.append)
    assert slept == [1.0, 1.0, 1.0]                # 3s 逐秒倒数
    assert any("land" in ln for ln in lines)       # 醒目警告里点名动作


def test_countdown_not_for_kill():
    # S3：kill 已移出倒计时动作（改 --yes + 全词确认双闸门）
    slept, lines = [], []
    ul.countdown("kill", yes=False, out=lines.append, sleep=slept.append)
    assert slept == [] and lines == []
    assert "kill" not in ul.COUNTDOWN_ACTIONS
    assert ul.WORD_CONFIRM.get("kill") == "kill"


def test_countdown_not_for_arm():
    # arm 不走倒计时（它走 --yes 闸门）
    slept, lines = [], []
    ul.countdown("arm", yes=False, out=lines.append, sleep=slept.append)
    assert slept == [] and lines == []


def test_find_command_channel():
    found = ["/fmu/out/vehicle_status", "/fmu/in/vehicle_command",
             "/fmu/out/vehicle_attitude"]
    assert ul.find_command_channel(found) == "/fmu/in/vehicle_command"
    assert ul.find_command_channel(["/fmu/out/vehicle_status"]) is None
    assert ul.find_command_channel([]) is None


def test_choose_command_route_split():
    """M1 映射分路：发现 /px4_control/command 时优先走状态机防护的安全通道；
    找不到才直发 /fmu/in/vehicle_command；kill/return 恒直发；都没有→None。"""
    both = ["/fmu/out/vehicle_status", "/px4_control/command",
            "/fmu/in/vehicle_command"]
    for a in ("arm", "disarm", "takeoff", "land"):
        assert ul.choose_command_route(a, both) == ("px4_control",
                                                    "/px4_control/command")
    # kill（穿透状态机）与 return（无对应码）恒直发，即使安全通道在场
    assert ul.choose_command_route("kill", both) == ("fmu_direct",
                                                     "/fmu/in/vehicle_command")
    assert ul.choose_command_route("return", both) == ("fmu_direct",
                                                       "/fmu/in/vehicle_command")
    # 安全通道不在场 → 有映射的动作回退直发（笔记本端会醒目提示绕过防护）
    direct = ["/fmu/in/vehicle_command", "/fmu/out/vehicle_status"]
    assert ul.choose_command_route("arm", direct) == ("fmu_direct",
                                                      "/fmu/in/vehicle_command")
    # 两个通道都没有 → None（响亮失败，绝不盲发）
    assert ul.choose_command_route("arm", ["/fmu/out/vehicle_status"]) is None
    assert ul.choose_command_route("arm", []) is None
    # 采样器内嵌同款分路逻辑（同源契约）
    assert "choose_command_route" in ul.SAMPLER_SRC
    assert "px4_control/command" in ul.SAMPLER_SRC
    # 数据驱动契约：ctrl 码只能是 None 或 px4_control 已核实码（2/3/4）
    for a, spec in ul.COMMAND_MAP.items():
        assert spec["ctrl"] in (None, 2, 3, 4), a
    assert ul.COMMAND_MAP["kill"]["ctrl"] is None
    assert ul.COMMAND_MAP["return"]["ctrl"] is None


def test_clamp_alt():
    """--alt 夹到 0.5~10m（过低撞地效区、过高超侦察剖面）。"""
    assert ul.clamp_alt(2.0) == 2.0
    assert ul.clamp_alt(0.1) == 0.5
    assert ul.clamp_alt(0.0) == 0.5
    assert ul.clamp_alt(99.0) == 10.0


def test_render_channel_failure_lists_found_topics():
    text = ul.render_channel_failure("arm", ["/fmu/out/vehicle_status"])
    assert "拒绝盲发" in text                       # 响亮失败
    assert "/fmu/out/vehicle_status" in text        # 列出实际发现帮助改映射
    empty = ul.render_channel_failure("arm", [])
    assert "MicroXRCEAgent" in empty


# ---------------------------------------------------------------- 命令映射表渲染（数据驱动）

def test_render_command_map():
    text = ul.render_command_map()
    for action, spec in ul.COMMAND_MAP.items():
        assert action in text
        assert f"cmd={spec['cmd']}" in text
        assert spec["desc"] in text
    assert "stack-status" in text and "vehicle_command" in text
    # 数据驱动契约：每项必须有 cmd:int / params:dict / desc:str
    for spec in ul.COMMAND_MAP.values():
        assert isinstance(spec["cmd"], int)
        assert isinstance(spec["params"], dict)
        assert spec["desc"]
    assert set(ul.ALL_ACTIONS) == set(ul.COMMAND_MAP) | set(ul.STACK_ACTIONS)
    # MAVLink 命令号锚定（改错了真机上就是另一条命令）
    assert ul.COMMAND_MAP["arm"]["cmd"] == 400
    assert ul.COMMAND_MAP["takeoff"]["cmd"] == 22
    assert ul.COMMAND_MAP["land"]["cmd"] == 21
    assert ul.COMMAND_MAP["return"]["cmd"] == 20
    # kill 与 flight_console 同语义：400 + 强制断解锁魔法字 param2=21196
    # （185 DO_FLIGHTTERMINATION 受 CBRK_FLIGHTTERM 影响可能被降级，2026-10-16 修正）
    assert ul.COMMAND_MAP["kill"]["cmd"] == 400
    assert ul.COMMAND_MAP["kill"]["params"].get("param1") == 0.0
    assert ul.COMMAND_MAP["kill"]["params"].get("param2") == 21196.0
    # 采样器内嵌映射表必须同源（文件头警告：改一边同步另一边）
    assert '"kill":    {"cmd": 400' in ul.SAMPLER_SRC
    assert "21196" in ul.SAMPLER_SRC


# ---------------------------------------------------------------- 机载采样器源码契约

def test_sampler_launch_cmd_sources_domain():
    cmd = ul.sampler_launch_cmd()
    # 真机教训回归：必须先 source 机载域配置 dzros.env（ROS_DOMAIN_ID=118），
    # 文件不存在时静默跳过；再 humble，再工作空间（px4_msgs 在 dzpx4），最后 setsid
    assert "[ -f ~/dzrobot/dzros.env ] && source ~/dzrobot/dzros.env || true" in cmd
    assert cmd.index("dzros.env") < cmd.index("/opt/ros/humble/setup.bash") \
        < cmd.index("dzpx4/install/setup.bash") < cmd.index("setsid python3")
    # 外层 bash -lc 单引号包裹，内层不得再出现单引号
    assert cmd.startswith("bash -lc '") and cmd.endswith("'")
    assert cmd.count("'") == 2
    assert ul.SAMPLER_REMOTE in cmd


def test_sampler_src_self_describing():
    src = ul.SAMPLER_SRC
    # 话题名无法离线验证：必须运行期自描述枚举 + QoS 跟随发布端
    assert "get_topic_names_and_types" in src
    assert "get_publishers_info_by_topic" in src
    # 启动先报 hello 让笔记本端知道发现了什么
    assert '"hello": "uav_sampler"' in src
    # 无 rclpy 时响亮报错非零退出
    assert "rclpy 导入失败" in src and "sys.exit(3)" in src
    # 找不到命令通道时响亮失败（exit 4），绝不盲发
    assert "sys.exit(4)" in src
    # 命令映射表与笔记本端同源（改一边必须同步另一边）
    for action, spec in ul.COMMAND_MAP.items():
        assert f'"{action}"' in src
        assert f'"cmd": {spec["cmd"]}' in src
    # dzuav 健康通道：jsonl 与 tmux capture-pane 双路；jsonl 只读尾 64KB（M3）
    assert "capture-pane" in src and "dzuav-*.jsonl" in src
    assert "65536" in src
    # 内嵌 RateLimiter 的 cap<=0 除零守卫与笔记本端同步
    assert "period = 1.0 / cap if cap > 0 else 0.0" in src
    # M1：采样器端同款命令分路（优先 px4_control 安全通道，找不到才直发）
    assert "choose_command_route" in src and "px4_control/command" in src
    # S2：estimator ok 只由故障/创新检验 flag 决定，solution_status_flags 仅留痕
    assert "filter_fault_flags" in src and "innovation_check_flags" in src
    # 限速档位与笔记本端一致
    assert '"attitude": 10.0' in src and '"position": 5.0' in src


# ---------------------------------------------------------------- 遥测状态聚合 / 仪表盘

def test_telemetry_state_feed_and_dashboard():
    st = ul.TelemetryState()

    def row(ch, data, t=99.9, topic=None):
        return {"kind": "sample", "t": t, "topic": topic or f"/fmu/out/{ch}",
                "ch": ch, "hz": 1.0, "data": data}

    st.feed(100.0, row("status", {"armed": True, "nav_state": 5, "failsafe": False}))
    st.feed(100.0, row("attitude", {"rpy_deg": [1, 2, 3]}))
    st.feed(100.0, row("battery", {"voltage": 22.4, "remaining": 0.65}))
    st.feed(100.0, row("dzuav", {"localized": True, "landmarks": 3},
                       topic="dzuav.health"))
    ei = st.engine_input()
    assert ei["failsafe"] is False
    assert ei["att_deg"] == (1.0, 2.0, 3.0)
    assert ei["battery_remaining"] == pytest.approx(0.65)
    assert ei["dzuav_t"] == 100.0
    assert st.last_lag_ms == pytest.approx(100.0)   # 链路滞后 = recv - 采样t
    line = ul.render_dashboard(st, "无", now=100.0)
    assert "ARMED" in line and "nav=5" in line
    assert "22.4V" in line and "65%" in line
    assert "dzuav:loc=1" in line and "lm=3" in line


# ---------------------------------------------------------------- 主机缓存 / CLI 骨架

def test_host_cache_roundtrip(tmp_path):
    p = tmp_path / "uav_link.local.env"
    assert ul.load_cached_host(p) is None
    ul.save_cached_host("10.0.0.8", p)
    assert ul.load_cached_host(p) == "10.0.0.8"


def test_host_cache_flag_wired(tmp_path):
    """--host-cache 不再是死旋钮：CLI 接线后 resolve_host 读指定路径。"""
    p = tmp_path / "custom.env"
    ul.save_cached_host("10.9.9.9", p)
    args = ul.build_parser().parse_args(["run", "--host-cache", str(p)])
    assert args.host_cache == str(p)
    assert ul.resolve_host(args) == "10.9.9.9"
    # 命令行 --host 仍优先于缓存
    args2 = ul.build_parser().parse_args(["run", "--host", "10.1.1.1",
                                          "--host-cache", str(p)])
    assert ul.resolve_host(args2) == "10.1.1.1"


def test_main_rejects_unknown_subcommand():
    assert ul.main(["bogus"]) == 2


def test_main_run_without_host(monkeypatch, tmp_path, capsys):
    # 没 --host 也没缓存：清晰报错返回 2，不碰网络
    monkeypatch.setattr(ul, "HOST_CACHE", tmp_path / "nonexistent.env")
    monkeypatch.delenv("AG_ORIN_PASS", raising=False)
    assert ul.main(["run"]) == 2
    assert "--host" in capsys.readouterr().err


def test_help_all_subcommands():
    with pytest.raises(SystemExit) as e:
        ul.main(["--help"])
    assert e.value.code == 0
    for c in ("run", "ctl", "logs", "report"):
        with pytest.raises(SystemExit) as e2:
            ul.main([c, "--help"])
        assert e2.value.code == 0


# ---------------------------------------------------------------- report 离线复盘

def _sample(t, ch, data, topic=None):
    """构造一条黑匣子采样行（形状与 cmd_run 落盘一致）。"""
    return {"recv": t, "t": t, "topic": topic or f"/fmu/out/{ch}",
            "ch": ch, "hz": 1.0, "data": data}


def _mk_blackbox_rows():
    """构造一段"飞控不稳"的黑匣子：解锁→起飞态→failsafe→链路空洞→上锁。"""
    rows = [{"recv": 0.0, "hello": "uav_sampler",
             "topics_found": ["/fmu/out/vehicle_status",
                              "/fmu/in/vehicle_command"]}]
    rows.append(_sample(0.0, "status",
                        {"armed": False, "nav_state": 0, "failsafe": False}))
    rows.append(_sample(1.0, "status",
                        {"armed": True, "nav_state": 5, "failsafe": False}))
    for i in range(10):  # 姿态 10° vs 目标 40°（偏差 30°）、低电 15%
        t = 1.0 + 0.1 * i
        rows.append(_sample(t, "attitude", {"rpy_deg": [10.0, -5.0, 80.0]}))
        rows.append(_sample(t, "attitude_target", {"rpy_deg": [40.0, -5.0, 80.0]}))
        rows.append(_sample(t, "rates", {"dps": [50.0, -20.0, 10.0]}))
        rows.append(_sample(t, "battery", {"voltage": 21.5, "remaining": 0.15}))
    rows.append(_sample(1.0, "imu", {"clip": 0}))
    rows.append(_sample(1.9, "imu", {"clip": 4}))          # 削波 +4
    rows.append(_sample(2.0, "status",
                        {"armed": True, "nav_state": 5, "failsafe": True}))
    rows.append(_sample(2.5, "status",
                        {"armed": True, "nav_state": 5, "failsafe": False}))
    # 2.5 → 7.0：4.5s 链路空洞
    rows.append(_sample(7.0, "status",
                        {"armed": False, "nav_state": 0, "failsafe": False}))
    rows.append(_sample(7.5, "dzuav", None, topic="dzuav.health"))
    rows.append({"recv": 3.0, "src": "ctl", "action": "arm", "ok": True,
                 "detail": "已向 /fmu/in/vehicle_command 连发 10 帧"})
    return rows


def test_build_report_extracts_events():
    rep = ul.build_report(_mk_blackbox_rows())
    # armed 时间线：disarmed → ARMED → disarmed
    assert rep["armed_timeline"] == [(0.0, False, 0), (1.0, True, 5),
                                     (7.0, False, 0)]
    assert rep["mode_switches"] == [(1.0, 5), (7.0, 0)]
    # failsafe False→True 跳变一次
    assert rep["failsafe_events"] == [(2.0, 5)]
    # 链路空洞一段，4.5s
    assert len(rep["link_gaps"]) == 1
    assert rep["link_gaps"][0][2] == pytest.approx(4.5)
    # 削波累计 +4
    assert rep["vibe_clip_total"] == 4
    # 姿态-目标偏差 30°，一段
    assert len(rep["att_divergence"]) == 1
    assert rep["att_divergence"][0][2] == pytest.approx(30.0)
    # 电池最低
    assert rep["battery"]["min_remaining"] == pytest.approx(0.15)
    assert rep["battery"]["min_voltage"] == pytest.approx(21.5)
    # dzuav 通道一行且为 null
    assert rep["dzuav"] == {"rows": 1, "null_rows": 1}
    # ctl 回执
    assert len(rep["ctl_receipts"]) == 1 and rep["ctl_receipts"][0]["ok"] is True
    # 姿态统计原料
    assert len(rep["att_abs_roll"]) == 10
    assert max(rep["rate_max"]) == pytest.approx(50.0)


def test_render_report_severity_grading():
    rep = ul.build_report(_mk_blackbox_rows())
    lines = ul.render_report(rep)
    crit = [ln for ln in lines if ln.startswith("[严重]")]
    warn = [ln for ln in lines if ln.startswith("[注意]")]
    info = [ln for ln in lines if ln.startswith("[信息]")]
    # 严重：FAILSAFE
    assert any("FAILSAFE" in ln for ln in crit)
    # 注意：链路空洞 / 低电 / 削波 / 姿态-目标偏差 / dzuav 无数据
    assert any("链路空洞" in ln for ln in warn)
    assert any("电池" in ln for ln in warn)
    assert any("削波" in ln for ln in warn)
    assert any("姿态-目标偏差" in ln for ln in warn)
    assert any("dzuav" in ln for ln in warn)
    # 信息：时间线 + 姿态统计（|roll| 全 10 → mean 10.00）
    assert any("armed 时间线" in ln and "ARMED" in ln for ln in info)
    assert any("|roll|" in ln and "mean=10.00" in ln for ln in info)
    assert any("角速度峰值" in ln and "max=50.00" in ln for ln in info)
    # 严重必须排在注意之前（分级输出顺序）
    idx = [i for i, ln in enumerate(lines) if ln.startswith("[严重]")] + \
          [i for i, ln in enumerate(lines) if ln.startswith("[注意]")]
    assert idx == sorted(idx)


def test_report_healthy_blackbox_no_critical():
    rows = []
    for i in range(30):
        t = i * 0.1
        rows.append(_sample(t, "status",
                            {"armed": True, "nav_state": 5, "failsafe": False}))
        rows.append(_sample(t, "battery",
                            {"voltage": 24.0, "remaining": 0.9}))
        rows.append(_sample(t, "attitude", {"rpy_deg": [1.0, 2.0, 80.0]}))
    rep = ul.build_report(rows)
    findings = ul.build_findings(rep)
    assert not [f for f in findings if f["severity"] == "critical"]
    assert not [f for f in findings if f["severity"] == "warn"]
    lines = ul.render_report(rep)
    assert not any(ln.startswith("[严重]") for ln in lines)


def test_report_t_zero_not_confused_with_recv():
    # 回归：t=0.0 是合法时刻，不得被 or 退化到 recv（epoch 时刻会污染时间线/时长）
    rows = [
        {"recv": 1792144013.0, "t": 0.0, "topic": "/fmu/out/vehicle_status",
         "ch": "status", "hz": 1.0,
         "data": {"armed": False, "nav_state": 0, "failsafe": False}},
        {"recv": 1792144014.0, "t": 1.0, "topic": "/fmu/out/vehicle_status",
         "ch": "status", "hz": 1.0,
         "data": {"armed": True, "nav_state": 5, "failsafe": False}},
    ]
    rep = ul.build_report(rows)
    assert rep["armed_timeline"][0][0] == 0.0
    assert rep["span_s"] == pytest.approx(1.0)


def test_cmd_report_endtoend(tmp_path, capsys):
    p = tmp_path / "bb.jsonl"
    box = ul.BlackboxWriter(p)
    for r in _mk_blackbox_rows():
        box.write(r)
    box.close()
    assert ul.main(["report", str(p)]) == 0
    out = capsys.readouterr().out
    assert "[严重] FAILSAFE" in out and "[注意]" in out and "[信息]" in out
    # 不存在的文件 → 清晰报错
    assert ul.main(["report", str(tmp_path / "nope.jsonl")]) == 2


# ================================================================ flight_bb 双格式
# 并行会话机载黑匣子 tools/orin_flight_blackbox.py 的 JSONL（ev 键判别）。
# 行形：{"ts","mono","seq","node":"flight_bb","ev","lvl","session",<事件字段>}

FB_T0 = 1792144000.0   # 样例 ts 基准（report 统一时间归零，断言用相对秒）


def _fb(t, ev, lvl="info", **fields):
    """构造一条 flight_bb 行（形状与 _emit 落盘一致）。"""
    row = {"ts": FB_T0 + t, "mono": t, "seq": 0, "node": "flight_bb",
           "ev": ev, "lvl": lvl, "session": "pytest"}
    row.update(fields)
    return row


def test_classify_blackbox_row():
    assert ul.classify_blackbox_row({"ev": "flight.att", "node": "flight_bb"}) \
        == "flight_bb"
    assert ul.classify_blackbox_row({"topic": "/x"}) == "sample"
    # 两键都在时 ev+node 优先（实际不存在，但判别顺序要锚定）
    assert ul.classify_blackbox_row({"ev": "x", "topic": "/y",
                                     "node": "flight_bb"}) == "flight_bb"
    # node 条件防误判（2026-10-17 审查修正）：ag_core 黑匣子行同样带 ev 键，
    # node 不是 flight_bb 的一律不当 flight_bb 解析
    assert ul.classify_blackbox_row({"ev": "x"}) == "other"
    assert ul.classify_blackbox_row({"ev": "session.begin", "node": "ag_core"}) \
        == "other"
    assert ul.classify_blackbox_row({"hello": 1}) == "other"
    assert ul.classify_blackbox_row({}) == "other"


def test_flight_bb_status_timeline_onchange_heartbeat():
    """flight.status 是 on-change + 每秒保底心跳：时间线按变化段，心跳不产生新条目。"""
    rows = [
        _fb(0.0, "flight.status", arming=1, arming_name="STANDBY", nav=0,
            nav_name="MANUAL", failsafe=0, prev=None),
        _fb(1.0, "flight.status", lvl="warn", arming=2, arming_name="ARMED",
            nav=0, nav_name="MANUAL", failsafe=0, prev={"arming": 1, "nav": 0,
                                                        "failsafe": 0}),
        _fb(2.0, "flight.status", lvl="warn", arming=2, nav=14,
            nav_name="OFFBOARD", failsafe=0),
        # 保底心跳：同值 info，不得进时间线
        _fb(3.0, "flight.status", arming=2, nav=14, nav_name="OFFBOARD", failsafe=0),
        _fb(4.0, "flight.status", arming=2, nav=14, nav_name="OFFBOARD", failsafe=0),
        _fb(5.0, "flight.status", lvl="warn", arming=2, nav=14,
            nav_name="OFFBOARD", failsafe=1),
        _fb(6.0, "flight.status", arming=2, nav=14, nav_name="OFFBOARD", failsafe=1),
        _fb(7.0, "flight.status", lvl="warn", arming=2, nav=14,
            nav_name="OFFBOARD", failsafe=0),
        _fb(9.0, "flight.status", lvl="warn", arming=1, arming_name="STANDBY",
            nav=0, nav_name="MANUAL", failsafe=0),
    ]
    rep = ul.build_report(rows)
    # 解锁时间线只记 armed 变化段（首帧 + 1s 解锁 + 9s 上锁），心跳/模式变化不进
    assert rep["armed_timeline"] == [(0.0, False, "MANUAL"), (1.0, True, "MANUAL"),
                                     (9.0, False, "MANUAL")]
    assert rep["mode_switches"] == [(2.0, "OFFBOARD"), (9.0, "MANUAL")]
    # failsafe 0→1 跳变一次（心跳重复 failsafe=1 不重复计）
    assert rep["failsafe_events"] == [(5.0, "OFFBOARD")]
    # warn 行（变化行）5 条，心跳（info）不计
    assert rep["status_flips"] == 5
    findings = ul.build_findings(rep)
    assert any(f["severity"] == "critical" and "FAILSAFE" in f["title"]
               for f in findings)


def test_flight_bb_att_stats_and_target_pairing():
    """flight.att（弧度）→ 姿态统计（度）；与 fmu.target 配对（1s 窗口）出偏差段。"""
    rows = []
    for i in range(7):  # att yaw=80° vs target yaw=50° → 偏差 30° 持续 3s
        t = i * 0.5
        rows.append(_fb(t, "flight.att", q=[1, 0, 0, 0],
                        roll=math.radians(5), pitch=math.radians(-3),
                        yaw=math.radians(80)))
        rows.append(_fb(t, "fmu.target", target_x=0.0, target_y=0.0,
                        target_z=-2.0, yaw=math.radians(50)))
    rep = ul.build_report(rows)
    assert rep["att_abs_roll"][0] == pytest.approx(5.0)     # 弧度→度
    assert rep["att_abs_pitch"][0] == pytest.approx(3.0)
    assert len(rep["att_divergence"]) == 1
    assert rep["att_divergence"][0][2] == pytest.approx(30.0, abs=0.01)
    findings = ul.build_findings(rep)
    assert any("姿态-目标偏差" in f["title"] for f in findings)


def test_flight_bb_pair_window_respected():
    """att 与 target 相隔 >1s 不配对：不出偏差段。"""
    rows = [
        _fb(0.0, "flight.att", roll=0.0, pitch=0.0, yaw=math.radians(80)),
        _fb(2.5, "fmu.target", target_x=None, target_y=None, target_z=None,
            yaw=math.radians(10)),
    ]
    rep = ul.build_report(rows)
    assert rep["att_divergence"] == []


def test_flight_bb_pos_speed_stats_and_pos_err():
    """flight.pos → 速度/高度统计；与 fmu.target 位置配对出三维误差。"""
    rows = []
    for i in range(6):
        t = i * 0.2
        rows.append(_fb(t, "flight.pos", x=1.0, y=2.0, z=-3.0,
                        vx=3.0, vy=4.0, vz=0.0, yaw=0.0, xy_valid=1, z_valid=1))
        rows.append(_fb(t, "fmu.target", target_x=2.0, target_y=2.0,
                        target_z=-2.0, yaw=None))
    rep = ul.build_report(rows)
    assert rep["speed"][0] == pytest.approx(5.0)            # 3-4-5 三角形
    assert rep["alt"] == [-3.0] * 6
    # (1,2,-3) vs (2,2,-2)：dx=1, dz=-1 → sqrt(2)
    assert rep["pos_err"][0] == pytest.approx(math.sqrt(2.0))
    lines = ul.render_report(rep)
    assert any("速度(m/s)" in ln for ln in lines)
    assert any("位置-目标误差(m)" in ln for ln in lines)


def test_flight_bb_cmd_audit_merged_into_timeline():
    """ctrl.cmd/fmu.cmd/goto.state 合并进解锁时间线（cmd: 前缀，按时间交错）。"""
    rows = [
        _fb(0.0, "flight.status", arming=1, nav=0, nav_name="MANUAL", failsafe=0),
        _fb(1.0, "ctrl.cmd", lvl="warn", cmd=2, name="ARM",
            dx=None, dy=None, dz=None),
        _fb(1.2, "flight.status", lvl="warn", arming=2, nav=0,
            nav_name="MANUAL", failsafe=0),
        _fb(5.0, "fmu.cmd", lvl="warn", command=21, name="NAV_LAND",
            p1=None, p2=None, p3=None, p4=None, p7=None),
        _fb(6.0, "goto.state", lvl="warn", enabled=1),
    ]
    rep = ul.build_report(rows)
    labels = [la for _t, la in rep["cmd_audit"]]
    assert labels == ["ctrl.cmd ARM", "fmu.cmd NAV_LAND", "goto ON"]
    lines = ul.render_report(rep)
    tl = [ln for ln in lines if "armed 时间线" in ln][0]
    assert "cmd:ctrl.cmd ARM" in tl and "cmd:fmu.cmd NAV_LAND" in tl
    assert "cmd:goto ON" in tl
    assert any("命令审计 3 条" in ln for ln in lines)
    # GOTO 翻转（lvl=warn 留痕语义）→ [注意]
    findings = ul.build_findings(rep)
    assert any(f["severity"] == "warn" and "GOTO" in f["title"] for f in findings)


def test_flight_bb_kill_is_critical():
    """KILL（400 p1=0 p2=21196，lvl=error）→ [严重]；普通 arm/disarm 不算。"""
    rows = [
        _fb(0.0, "flight.status", arming=2, nav=14, nav_name="OFFBOARD", failsafe=0),
        _fb(1.0, "fmu.cmd", lvl="error", command=400, name="ARM_DISARM",
            p1=0.0, p2=21196.0, p3=None, p4=None, p7=None),
        _fb(2.0, "fmu.cmd", lvl="warn", command=400, name="ARM_DISARM",
            p1=1.0, p2=None, p3=None, p4=None, p7=None),
    ]
    rep = ul.build_report(rows)
    assert len(rep["kill_events"]) == 1
    findings = ul.build_findings(rep)
    crit = [f for f in findings if f["severity"] == "critical"]
    assert any("KILL" in f["title"] for f in crit)
    # KILL 不得再被 error_rows 聚合重复报一次
    assert not [f for f in crit if f["title"].startswith("error 级事件")]
    # 普通 ARM_DISARM（p1=1，warn 级）只进审计不进结论
    assert "ARM_DISARM(p1=1)" in [la for _t, la in rep["cmd_audit"]][1]
    # 分级顺序：严重必须在注意之前
    lines = ul.render_report(rep)
    idx = [i for i, ln in enumerate(lines)
           if ln.startswith("[严重]") or ln.startswith("[注意]")]
    assert idx == sorted(idx) and any(ln.startswith("[严重]") for ln in lines)


def test_flight_bb_batt_error_critical_no_duplicate():
    """flight.batt lvl=error（remaining<15%）→ [严重]，与最低电量规则去重。"""
    rows = [
        _fb(0.0, "flight.batt", lvl="error", remaining=0.12, voltage=20.1,
            warning=3),
        _fb(5.0, "flight.batt", remaining=0.11, voltage=20.0),
    ]
    rep = ul.build_report(rows)
    findings = ul.build_findings(rep)
    batt = [f for f in findings if "电池" in f["title"]]
    assert len(batt) == 1 and batt[0]["severity"] == "critical"
    assert rep["battery"]["min_remaining"] == pytest.approx(0.11)


def test_flight_bb_slam_gap_no_vibe_no_exit():
    """armed 期间 >5s 无 slam.odom → [注意] SLAM 断流；无振动通道明示；无 bb.exit 提醒。"""
    rows = [
        _fb(0.0, "flight.status", arming=2, nav=14, nav_name="OFFBOARD",
            failsafe=0),
        _fb(1.0, "slam.odom", x=0, y=0, z=0, qw=1, qx=0, qy=0, qz=0),
        _fb(9.5, "slam.odom", x=0.1, y=0, z=0, qw=1, qx=0, qy=0, qz=0),
    ]
    rep = ul.build_report(rows)
    assert len(rep["slam_gaps"]) == 1
    assert rep["slam_gaps"][0][2] == pytest.approx(8.5)
    findings = ul.build_findings(rep)
    assert any(f["severity"] == "warn" and "SLAM 断流" in f["title"]
               for f in findings)
    assert any("bb.exit" in f["title"] + f["detail"] for f in findings)
    lines = ul.render_report(rep)
    assert any("未含振动通道" in ln for ln in lines)


def test_flight_bb_slam_gap_closed_by_disarm():
    """回归：断流段以 disarm 为界闭合（armed 中断流 6s 后 disarm、slam 未恢复）。

    旧实现只在下一条 slam.odom 到达时用"当前 armed"判定——disarm 后 slam 才来
    会漏报（2026-10-16 手写样例冒烟抓到）。
    """
    rows = [
        _fb(0.0, "flight.status", arming=2, nav=14, nav_name="OFFBOARD",
            failsafe=0),
        _fb(1.0, "slam.odom", x=0, y=0, z=0, qw=1, qx=0, qy=0, qz=0),
        # 1.0 之后 slam 再没来过；7.5 disarm（armed 期间断流 6.5s）
        _fb(7.5, "flight.status", lvl="warn", arming=1, nav=0,
            nav_name="MANUAL", failsafe=0),
        _fb(9.0, "slam.odom", x=0, y=0, z=0, qw=1, qx=0, qy=0, qz=0),
    ]
    rep = ul.build_report(rows)
    assert len(rep["slam_gaps"]) == 1
    assert rep["slam_gaps"][0] == pytest.approx((1.0, 7.5, 6.5))
    findings = ul.build_findings(rep)
    assert any("SLAM 断流" in f["title"] for f in findings)
    # disarm 后的 1.5s（7.5→9.0）不算断流，不得多出第二段
    assert len(rep["slam_gaps"]) == 1


def test_flight_bb_rearm_resets_slam_timer():
    """M5 回归：armed False→True（rearm）时 last_slam_t 重置为 rearm 时刻——
    注释承诺"rearm 后重新连续 5s"，上一个架次的静默期不得带进新架次。

    无此修复时：disarm(10s) 把 last_slam_t 归 10，rearm(12s) 后 slam 16s 才来，
    断流被算成 10→16=6s>5s 且阈值跨越点 15s 已 armed → 误报。
    """
    rows = [
        _fb(0.0, "flight.status", arming=1, nav=0, nav_name="MANUAL", failsafe=0),
        _fb(1.0, "flight.status", lvl="warn", arming=2, nav=14,
            nav_name="OFFBOARD", failsafe=0),          # 解锁
        _fb(2.0, "slam.odom", x=0, y=0, z=0, qw=1, qx=0, qy=0, qz=0),
        _fb(5.0, "slam.odom", x=0, y=0, z=0, qw=1, qx=0, qy=0, qz=0),
        _fb(9.0, "slam.odom", x=0, y=0, z=0, qw=1, qx=0, qy=0, qz=0),
        _fb(10.0, "flight.status", lvl="warn", arming=1, nav=0,
            nav_name="MANUAL", failsafe=0),            # disarm（此架次无断流）
        _fb(12.0, "flight.status", lvl="warn", arming=2, nav=14,
            nav_name="OFFBOARD", failsafe=0),          # rearm：last_slam_t 归 12
        _fb(16.0, "slam.odom", x=0, y=0, z=0, qw=1, qx=0, qy=0, qz=0),
        # rearm 后仅 4s 无 slam → 不算断流
    ]
    rep = ul.build_report(rows)
    assert rep["slam_gaps"] == []
    # 对照：rearm 后 7s 无 slam → 断流从 rearm 时刻（12s）起算
    rows2 = rows[:-1] + [
        _fb(19.0, "slam.odom", x=0, y=0, z=0, qw=1, qx=0, qy=0, qz=0)]
    rep2 = ul.build_report(rows2)
    assert len(rep2["slam_gaps"]) == 1
    assert rep2["slam_gaps"][0] == pytest.approx((12.0, 19.0, 7.0))


def test_flight_bb_no_slam_at_all_while_armed():
    """全程一条 slam.odom 都没有且解锁过 → [注意]（FAST-LIO 没起/话题名变了）。"""
    rows = [
        _fb(0.0, "flight.status", arming=2, nav=14, nav_name="OFFBOARD",
            failsafe=0),
        _fb(1.0, "flight.att", roll=0.0, pitch=0.0, yaw=0.0),
        _fb(2.0, "bb.exit", force=True, counts={}),
    ]
    rep = ul.build_report(rows)
    findings = ul.build_findings(rep)
    assert any("全程无 slam.odom" in f["title"] for f in findings)
    # 有 bb.exit → 不报"未正常收尾"
    assert not any("bb.exit" in f["detail"] for f in findings)


def test_flight_bb_bb_hb_health_summary():
    rows = [
        _fb(0.0, "flight.att", roll=0.0, pitch=0.0, yaw=0.0),
        _fb(3.0, "bb.hb", up=30.0,
            counts={"flight.pos": 150, "flight.att": 60, "slam.odom": 150,
                    "fmu.cmd": 2, "ctrl.cmd": 1}),
        _fb(3.5, "bb.exit", counts={}),
    ]
    rep = ul.build_report(rows)
    lines = ul.render_report(rep)
    hb = [ln for ln in lines if "链路健康(bb.hb)" in ln]
    assert hb and "pos=5.0Hz" in hb[0] and "att=2.0Hz" in hb[0]
    assert "fmu.cmd=2" in hb[0]


def test_report_mixed_formats():
    """双格式混合文件：采样行与 flight_bb 行共用时间线与统计维度。"""
    rows = [
        {"recv": FB_T0, "t": FB_T0, "topic": "/fmu/out/vehicle_status",
         "ch": "status", "hz": 1.0,
         "data": {"armed": True, "nav_state": 5, "failsafe": False}},
        _fb(1.0, "flight.status", arming=2, nav=14, nav_name="OFFBOARD",
            failsafe=0),
        _fb(2.0, "flight.att", roll=0.1, pitch=0.0, yaw=0.0),
        {"recv": FB_T0 + 3, "t": FB_T0 + 3, "topic": "/fmu/out/battery_status",
         "ch": "battery", "hz": 1.0, "data": {"voltage": 23.0, "remaining": 0.8}},
    ]
    rep = ul.build_report(rows)
    assert rep["formats"]["sample"] == 2
    assert rep["formats"]["flight_bb"] == 2
    # 时间线跨格式合并（时间已归零）
    assert rep["armed_timeline"][0] == (0.0, True, 5)
    assert rep["armed_timeline"][1] == (1.0, True, "OFFBOARD")
    assert rep["att_abs_roll"]           # flight.att（弧度）进了姿态统计
    assert rep["battery"]["min_voltage"] == pytest.approx(23.0)
    lines = ul.render_report(rep)
    assert any("识别格式" in ln for ln in lines)


def test_flight_bb_lvl_respect_error_aggregated():
    """lvl=error 的非 KILL/非电池事件 → [严重] 聚合，不静默。"""
    rows = [
        _fb(0.0, "flight.att", roll=0.0, pitch=0.0, yaw=0.0),
        _fb(1.0, "flight.status", lvl="error", arming=2, nav=17,
            nav_name="LANDENGFAIL", failsafe=1),
        _fb(2.0, "bb.exit", counts={}),
    ]
    rep = ul.build_report(rows)
    findings = ul.build_findings(rep)
    crit_titles = [f["title"] for f in findings if f["severity"] == "critical"]
    assert any("error 级事件" in t for t in crit_titles)
    # failsafe=1 同样走 FAILSAFE 专条
    assert any("FAILSAFE" in t for t in crit_titles)
