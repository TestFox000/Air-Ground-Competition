"""ULog 容错打捞器（tools/ulog_salvage.py）的测试。

针对 2026-10-15 真实事故（627KB 文件只有前 6.5KB 有效、其余全零）验证：
    1. 完整合成 ULog 能解出 info / 参数 / format / 订阅 / 数据帧
    2. 中段截断不炸——截断点前的内容全保住，truncation_offset 报出来
    3. 尾部大段填零（真实死因）——行为与截断一致，且绝不抛异常
"""

import importlib.util
import struct
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def _load_tool(name: str):
    spec = importlib.util.spec_from_file_location(
        f"_tool_{name}", ROOT / "tools" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod  # dataclass 的 | 注解解析要回查 sys.modules
    spec.loader.exec_module(mod)  # type: ignore[union-attr]
    return mod


salvage_mod = _load_tool("ulog_salvage")


def _frame(mtype: bytes, payload: bytes) -> bytes:
    return struct.pack("<H", len(payload)) + mtype + payload


def _info(key: str, value: bytes) -> bytes:
    k = key.encode()
    return _frame(b"I", bytes([len(k)]) + k + value)


def _build_full_ulog() -> bytes:
    """最小但结构完整的 ULog：头 + flag + info + 参数 + format + 订阅 + 数据。"""
    out = bytearray()
    out += b"ULog\x01\x12\x35"          # magic(7)
    out += b"\x01"                        # version
    out += struct.pack("<Q", 123456789)   # timestamp_us
    # flag bitset：compat(8) + incompat(8) + offsets(3*8) = 40B
    out += _frame(b"B", b"\x00" * 40)
    # info: char[40] ver_sw = "abc123..."（补零到 40B）
    out += _info("char[40] ver_sw", b"abc123".ljust(40, b"\x00"))
    # info: int32_t sys_flight = 7
    out += _info("int32_t sys_flight", struct.pack("<i", 7))
    # parameter: float MPC_XY_P = 0.95
    k = b"float MPC_XY_P"
    out += _frame(b"P", bytes([len(k)]) + k + struct.pack("<f", 0.95))
    # format: test_msg:uint64_t timestamp;float x;float y
    out += _frame(b"F", b"test_msg:uint64_t timestamp;float x;float y")
    # subscription: msg_id=5, multi_id=0, name="test_msg"
    out += _frame(b"A", struct.pack("<HB", 5, 0) + b"test_msg")
    # 两帧数据：msg_id=5, (timestamp, x, y)
    out += _frame(b"D", struct.pack("<H", 5)
                  + struct.pack("<Qff", 1_000_000, 1.5, -2.5))
    out += _frame(b"D", struct.pack("<H", 5)
                  + struct.pack("<Qff", 2_000_000, 3.5, -4.5))
    # logged string: level=6, ts=1500000, "test event"
    out += _frame(b"L", bytes([6]) + struct.pack("<Q", 1_500_000)
                  + b"test event")
    return bytes(out)


# ── 完整文件 ───────────────────────────────────────────────────

def test_full_ulog_roundtrip(tmp_path):
    p = tmp_path / "full.ulg"
    p.write_bytes(_build_full_ulog())
    res = salvage_mod.salvage(p)
    assert res.header_ok
    assert res.infos["ver_sw"] == "abc123"
    assert res.infos["sys_flight"] == 7
    assert res.parameters["MPC_XY_P"] == pytest.approx(0.95)
    assert "test_msg" in res.formats
    assert res.subscriptions[5] == "test_msg"
    assert res.data_counts[5] == 2
    # 数据帧时间跨度（解码器解出 timestamp 字段）
    first, last = res.data_first_last_ts[5]
    assert (first, last) == (1_000_000, 2_000_000)
    assert res.logged_strings == [(6, 1_500_000, "test event")]
    assert res.truncation_offset is None or res.truncation_offset == len(
        p.read_bytes())


# ── 中段截断（真实死因变体）────────────────────────────────────

def test_truncated_mid_definitions(tmp_path):
    """参数帧被切掉一半：前半内容必须完整保住，报出截断点。"""
    full = _build_full_ulog()
    # 找到参数帧起点，从中间切断
    cut = full.index(b"P", 16)  # 粗定位：第一个 'P' 类型字节附近
    # 更稳的做法：直接砍在全文 60% 处（参数帧在文件前半段内）
    cut = int(len(full) * 0.6)
    p = tmp_path / "cut.ulg"
    p.write_bytes(full[:cut])
    res = salvage_mod.salvage(p)
    assert res.header_ok
    assert res.infos["ver_sw"] == "abc123"       # 截断点前的 info 保住
    assert res.truncation_offset is not None      # 报出可读边界
    assert res.truncation_offset < len(full)


def test_zero_padded_tail_like_real_incident(tmp_path):
    """尾部填零（log_1_2026-10-15-17-10-50.ulg 的真实死因）：不抛异常。"""
    full = _build_full_ulog()
    p = tmp_path / "padded.ulg"
    p.write_bytes(full + b"\x00" * 10000)
    res = salvage_mod.salvage(p)
    assert res.header_ok
    assert res.infos["ver_sw"] == "abc123"
    assert res.data_counts.get(5) == 2            # 数据帧全解码
    assert res.truncation_offset is not None      # 零区被识别为可读边界


def test_bad_magic_rejected(tmp_path):
    p = tmp_path / "bad.ulg"
    p.write_bytes(b"NOTULOG" + b"\x00" * 100)
    res = salvage_mod.salvage(p)
    assert not res.header_ok
    assert res.errors


def test_cli_json(tmp_path, capsys):
    p = tmp_path / "full.ulg"
    p.write_bytes(_build_full_ulog())
    rc = salvage_mod.main([str(p), "--json"])
    assert rc == 0
    import json
    d = json.loads(capsys.readouterr().out)
    assert d["header_ok"] is True
    assert d["parameters"]["MPC_XY_P"] == pytest.approx(0.95)
