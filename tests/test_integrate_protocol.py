"""ag_integrate intel_protocol 二进制 :50000 协议解析测试。"""

import random
import struct

import pytest

from ag_integrate.intel_protocol import (
    HEADER_SIZE,
    OBJECT_SIZE,
    ObjectType,
    IntelFrame,
    IntelObject,
    build,
    crc16,
    parse,
)


def make_object(type_index: int = 0) -> IntelObject:
    return IntelObject(ObjectType(type_index), 0.12 * type_index, -0.34, 0.35, 0.35)


def test_roundtrip_empty_and_many():
    for count in (0, 1, 7):
        frame = IntelFrame(
            1_700_000_000_123,
            tuple(make_object(i % 7) for i in range(count)),
        )
        raw = build(frame)
        assert len(raw) == HEADER_SIZE + OBJECT_SIZE * count + 2 + 4
        decoded = parse(raw)
        assert decoded.timestamp_ms == frame.timestamp_ms
        assert len(decoded.objects) == count
        for got, exp in zip(decoded.objects, frame.objects):
            assert got.type == exp.type
            assert got.x == pytest.approx(exp.x)
            assert got.y == pytest.approx(exp.y)
            assert got.length == pytest.approx(exp.length, abs=1e-6)
            assert got.width == pytest.approx(exp.width, abs=1e-6)


def test_fields_are_preserved():
    obj = IntelObject(ObjectType.TARGET_BLUE, 1.25, 2.75, 0.40, 0.30)
    frame = IntelFrame(999, (obj,))
    decoded = parse(build(frame))
    assert decoded.timestamp_ms == 999
    got = decoded.objects[0]
    assert got.type is ObjectType.TARGET_BLUE
    assert got.x == pytest.approx(1.25)
    assert got.y == pytest.approx(2.75)


def test_crc16_known_value():
    # CRC-16/Modbus check value for "123456789" is 0x4B37
    assert crc16(b"123456789") == 0x4B37


def _crc16_bitwise(data: bytes) -> int:
    """逐位参考实现（与机上 UdpBroadcaster.crc 的写法一一对应）。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def test_crc16_table_matches_bitwise_reference():
    # 查表实现必须与逐位参考在任意输入上逐位等价
    rng = random.Random(20261001)
    for length in (0, 1, 2, 7, 21, 100, 546):
        blob = bytes(rng.randrange(256) for _ in range(length))
        assert crc16(blob) == _crc16_bitwise(blob), f"mismatch at length {length}"


def test_crc16_accepts_memoryview_zero_copy_span():
    raw = build(IntelFrame(1, (make_object(0), make_object(5))))
    mv = memoryview(raw)
    # 与切片拷贝结果一致
    assert crc16(mv[4:52]) == crc16(raw[4:52])
    # 空区间
    assert crc16(mv[4:4]) == 0xFFFF


def _corrupt(raw: bytes, **changes) -> bytes:
    raw = bytearray(raw)
    if "magic" in changes:
        raw[0] = 0x00
    if "version" in changes:
        raw[4] = 0x11
    if "crc" in changes:
        raw[-6] ^= 0xFF
    if "suffix" in changes:
        raw[-1] = 0x00
    if "length" in changes:
        struct.pack_into("<I", raw, 15, changes["length"])
    if "size" in changes:
        del raw[-1]
    return bytes(raw)


@pytest.mark.parametrize(
    "change",
    ["magic", "version", "crc", "suffix", "length", "size"],
)
def test_malformed_frames_rejected(change):
    raw = build(IntelFrame(1, (make_object(0), make_object(2))))
    bad = _corrupt(raw, **{change: True})
    with pytest.raises(ValueError):
        parse(bad)


def test_truncated_and_nan_rejected():
    with pytest.raises(ValueError):
        parse(b"\xef" * 10)

    raw = bytearray(build(IntelFrame(1, (make_object(0),))))
    # overwrite x (float64 at first object) with NaN
    struct.pack_into("<d", raw, HEADER_SIZE + 1, float("nan"))
    with pytest.raises(ValueError):
        parse(bytes(raw))
