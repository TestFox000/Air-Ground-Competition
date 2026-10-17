"""flight_console 键处理状态机 + 镜像过滤单测（零硬件：不碰 SSH/终端/真网卡）。

覆盖审查实锤的安全链回归：
  * 确认链：L/X/!/1 后输确认词，动作必须真正执行；确认中误触任意键
    不得关掉面板（旧实现 confirm_key 恒返回 True，run_panel 把它当退出——
    按 L 再敲 y，面板直接关闭，LAND 永远发不出去）；
  * MOVE/YAW 单发：px4_control handle_move 是累加语义（target_ += dx），
    三发兜底会让一次 W 实走 1.5m 而非标称 0.5m；
  * UDP 镜像过滤：非 --host 来源 / 白名单外事件的伪造包必须丢弃并计数；
  * KeyReader 延迟创建：--stop/--fetch 路径不开面板，终端不能被弄进 cbreak。
"""

from __future__ import annotations

import argparse
import importlib.util
import sys
from pathlib import Path

import pytest

pytest.importorskip("paramiko")   # flight_console 顶层经 remote_run import paramiko

REPO = Path(__file__).resolve().parents[1]


def load_module(name: str, file: Path):
    spec = importlib.util.spec_from_file_location(name, file)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


fc = load_module("flight_console", REPO / "tools" / "flight_console.py")


def make_console():
    """构造不碰硬件的面板：镜像端口 0（OS 分配）、发包函数换成记录器。"""
    args = argparse.Namespace(host="127.0.0.1", user="u", password="p",
                              port=22, mirror_port=0)
    con = fc.FlightConsole(args)
    sent: list[tuple] = []
    con.send_ctrl_async = lambda cmd, name, dx=0.0, dy=0.0, dz=0.0: \
        sent.append((cmd, name, dx, dy, dz))
    kills: list[str] = []
    con.send_kill_async = lambda: kills.append("KILL")
    return con, sent, kills


def type_str(con, text: str):
    """模拟逐字符键入（返回每个键的 handle_key 返回值列表）。"""
    return [con.handle_key(ch) for ch in text]


# ---------------------------------------------------------------- 确认链（审查#1）

def test_land_confirm_flow_executes_action():
    """L -> y -> 回车：动作必须真正发出去，且全程没有任何键导致退出。"""
    con, sent, _ = make_console()
    assert con.handle_key("l") is False
    assert con.mode == "confirm"
    assert con.handle_key("y") is False
    assert con.handle_key("\r") is False
    assert sent == [(fc.CMD_LAND, "LAND", 0.0, 0.0, 0.0)]
    assert con.mode == "view"


def test_stray_keys_in_confirm_mode_never_exit():
    """确认中误触任何键：不退出、不执行，只是进了确认缓冲区。"""
    con, sent, _ = make_console()
    con.handle_key("l")
    for ch in "abcdef123!":
        assert con.handle_key(ch) is False
        assert con.mode == "confirm"
    assert sent == []


def test_esc_cancels_confirm_without_exit():
    con, sent, _ = make_console()
    con.handle_key("l")
    assert con.handle_key("\x1b") is False
    assert con.mode == "view"
    assert sent == []


def test_wrong_confirm_word_cancels():
    """DISARM 需要输全词：输错回车 = 取消，动作不得发出。"""
    con, sent, _ = make_console()
    con.handle_key("x")
    type_str(con, "nope")
    assert con.handle_key("\r") is False
    assert sent == []
    assert con.mode == "view"


def test_kill_requires_full_word():
    """KILL 确认词是 'kill'：少一个字母都不能执行（最后手段的闸门）。"""
    con, _, kills = make_console()
    con.handle_key("!")
    type_str(con, "kil")
    con.handle_key("\r")
    assert kills == []
    con.handle_key("!")
    type_str(con, "kill")
    con.handle_key("\r")
    assert kills == ["KILL"]


def test_q_exits_only_in_view_mode():
    """q 只在 view 模式退出：move 模式它是下降键，confirm 模式它是取消。"""
    con, sent, _ = make_console()
    con.handle_key("m")
    assert con.handle_key("q") is False          # move 模式：下降 0.1m
    assert sent[-1][0] == fc.CMD_MOVE and sent[-1][4] == -fc.STEP_Z
    con.handle_key("m")                          # 退回 view 模式
    assert con.mode == "view"
    con.handle_key("l")                          # 进确认
    assert con.handle_key("q") is False          # confirm 模式：取消而非退出
    assert con.mode == "view"
    assert con.handle_key("q") is True           # view 模式：退出面板


def test_backspace_edits_confirm_buffer_posix_and_windows():
    """退格两种编码都认：\\x7f（POSIX）与 \\x08（Windows conhost）。"""
    con, sent, _ = make_console()
    con.handle_key("x")                          # DISARM，确认词 disarm
    type_str(con, "disarX")
    con.handle_key("\x08")                       # Windows 退格删掉 X
    con.handle_key("m")
    type_str(con, "zz")
    con.handle_key("\x7f")                       # POSIX 退格
    con.handle_key("\x7f")
    assert con.confirm_buf == "disarm"
    assert con.handle_key("\r") is False
    assert sent == [(fc.CMD_DISARM, "DISARM", 0.0, 0.0, 0.0)]


# ---------------------------------------------------------------- MOVE/YAW 单发（审查#2）

def test_move_and_yaw_commands_are_single_shot():
    """累加语义命令只发一帧；幂等命令保留三发兜底（DDS 发现竞态）。"""
    con, _, _ = make_console()
    for cmd in (fc.CMD_MOVE, fc.CMD_YAW):
        assert "$(seq 1)" in con._ctrl_pub_cmd(cmd, 0.5, 0, 0)
    for cmd in (fc.CMD_OFFBOARD, fc.CMD_ARM, fc.CMD_DISARM, fc.CMD_LAND,
                fc.CMD_GOTO_ENABLE, fc.CMD_GOTO_DISABLE):
        assert "$(seq 3)" in con._ctrl_pub_cmd(cmd, 0, 0, 0)


def test_move_mode_key_sends_exactly_one_command():
    con, sent, _ = make_console()
    con.handle_key("m")
    assert con.handle_key("w") is False
    assert sent == [(fc.CMD_MOVE, "move(w)", fc.STEP_XY, 0, 0)]


# ---------------------------------------------------------------- 镜像过滤（审查#11）

def make_listener(ip="10.0.0.5"):
    return fc.MirrorListener(0, allowed_ip=ip)


def test_mirror_accepts_packet_from_host_ip():
    lis = make_listener()
    lis.handle_packet(b'{"ev":"flight.pos","x":1.5}\n', ("10.0.0.5", 9999))
    assert lis.latest["flight.pos"]["x"] == 1.5
    assert lis.dropped == 0


def test_mirror_drops_foreign_source():
    """来源 IP 不是 --host 解析值：整包丢弃并计数（伪造遥测不得上面板）。"""
    lis = make_listener()
    lis.handle_packet(b'{"ev":"flight.pos","x":1.5}\n', ("10.0.0.5", 9999))
    lis.handle_packet(b'{"ev":"flight.pos","x":-99.0}\n', ("9.9.9.9", 9999))
    assert lis.latest["flight.pos"]["x"] == 1.5     # 没被伪造包覆盖
    assert lis.dropped == 1
    assert lis.flow and "丢弃非法镜像包" in lis.flow[0][2]


def test_mirror_drops_unknown_event_names():
    """白名单外的事件名（伪造者最容易瞎编的入口）丢弃并计数。"""
    lis = make_listener()
    lis.handle_packet(b'{"ev":"evil.fake","x":1}\n', ("10.0.0.5", 9999))
    assert "evil.fake" not in lis.latest
    assert lis.dropped == 1


def test_mirror_drop_warning_is_throttled():
    """伪造包洪泛时告警节流：100 包只产生第一条节流窗内的告警。"""
    lis = make_listener()
    for _ in range(100):
        lis.handle_packet(b'{"ev":"evil.fake"}\n', ("10.0.0.5", 9999))
    assert lis.dropped == 100
    assert sum("丢弃非法镜像包" in t for _ts, _lvl, t in lis.flow) == 1


# ---------------------------------------------------------------- KeyReader 延迟创建（审查#6）

def test_keyreader_created_lazily():
    """构造面板不得碰终端（--stop/--fetch 路径提前 return 的坑）。"""
    con, _, _ = make_console()
    assert con.keys is None
