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
    assert ul.choose_qos([]) == "reliable"               # 无发布端信息按默认
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
    for a in ("arm", "takeoff", "stack-restart"):
        assert ul.check_gate(a, False) is not None   # 无 --yes 拒
        assert ul.check_gate(a, True) is None        # 有 --yes 放行
    # disarm/land/return/kill 免 --yes（靠倒计时兜底），stack-status 只读
    for a in ("disarm", "land", "return", "kill", "stack-status"):
        assert ul.check_gate(a, False) is None


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
    ul.countdown("kill", yes=False, out=lines.append, sleep=slept.append)
    assert slept == [1.0, 1.0, 1.0]                # 3s 逐秒倒数
    assert any("kill" in ln for ln in lines)       # 醒目警告里点名动作


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
    assert ul.COMMAND_MAP["kill"]["cmd"] == 185


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
    # dzuav 健康通道：jsonl 与 tmux capture-pane 双路
    assert "capture-pane" in src and "dzuav-*.jsonl" in src
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
