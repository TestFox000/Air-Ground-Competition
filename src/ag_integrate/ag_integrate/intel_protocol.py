"""dzuav UdpBroadcaster 二进制 UDP 帧解析（端口 50000）。

字节布局（全部小端），严格对应
reference/onboard/dzuav/src/UdpBroadcaster.cpp：

    magic      4 × 0xef
    version    1    0x10
    timestamp  uint64   机上毫秒时间戳
    msg_id     uint16   0x4001
    length     uint32   = 2 + 25 * count
    count      uint16   目标数量
    objects    count * 25 字节，每目标：
                   type    uint8
                   centerx float64
                   centery float64
                   length  float32
                   width   float32
    crc16      uint16   CRC-16/Modbus，覆盖从 version 到最后一个目标
    suffix     4 × 0xfe

本模块只做“线上字节 <-> 结构化帧”，不含 ROS、不做坐标变换。
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from enum import IntEnum

MAGIC = b"\xef\xef\xef\xef"
VERSION = 0x10
MSG_ID = 0x4001
SUFFIX = b"\xfe\xfe\xfe\xfe"

HEADER_SIZE = 21          # magic4 + version1 + ts8 + msgid2 + length4 + count2
OBJECT_SIZE = 25          # type1 + x8 + y8 + length4 + width4
CRC_SIZE = 2
TAIL_SIZE = 4             # suffix


class ObjectType(IntEnum):
    """目标类别，顺序必须与 dzuav main.cpp 的 className 一致。"""

    MINE = 0          # mine 雷
    CAR = 1           # car
    ARROW = 2         # arrow 桥顶箭头/可通行
    TARGET_RED = 3    # target_red 友方
    CELL_LABEL = 4    # cell_label 地面格号
    TARGET_BLUE = 5   # target_blue 敌方
    OBSTACLE = 6      # obstacle 障碍


@dataclass(frozen=True)
class IntelObject:
    type: ObjectType
    x: float
    y: float
    length: float
    width: float


@dataclass(frozen=True)
class IntelFrame:
    timestamp_ms: int
    objects: tuple[IntelObject, ...]


def _build_crc_table() -> tuple[int, ...]:
    """预生成 256 项查表（多项式 0xa001，与逐位计算数学等价）。"""
    table = []
    for byte in range(256):
        crc = byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
        table.append(crc)
    return tuple(table)


_CRC_TABLE = _build_crc_table()


def crc16(data: bytes | memoryview) -> int:
    """CRC-16/Modbus 查表实现：与机上 UdpBroadcaster 逐位算法完全等价（"123456789"→0x4B37）。

    接受 memoryview 以便调用方零拷贝地校验帧内区间。
    """
    crc = 0xFFFF
    for byte in data:
        crc = (crc >> 8) ^ _CRC_TABLE[(crc ^ byte) & 0xFF]
    return crc & 0xFFFF


def _expected_size(count: int) -> int:
    return HEADER_SIZE + OBJECT_SIZE * count + CRC_SIZE + TAIL_SIZE


def parse(data: bytes) -> IntelFrame:
    """解析一帧；任何不一致都抛 ValueError（不返回半成品）。"""
    if len(data) < HEADER_SIZE + CRC_SIZE + TAIL_SIZE:
        raise ValueError(f"frame too short: {len(data)} bytes")
    if data[0:4] != MAGIC:
        raise ValueError("bad magic prefix")
    if data[4] != VERSION:
        raise ValueError(f"unsupported version byte: 0x{data[4]:02x}")

    timestamp_ms, msg_id, declared_len, count = struct.unpack_from("<QHIH", data, 5)
    if msg_id != MSG_ID:
        raise ValueError(f"unexpected message id: 0x{msg_id:04x}")

    expected_declared = 2 + OBJECT_SIZE * count
    if declared_len != expected_declared:
        raise ValueError(
            f"declared length {declared_len} != 2 + 25*{count} ({expected_declared})"
        )

    expected_size = _expected_size(count)
    if len(data) != expected_size:
        raise ValueError(f"frame size {len(data)} != expected {expected_size}")

    objects: list[IntelObject] = []
    offset = HEADER_SIZE
    for _ in range(count):
        type_index, x, y, length, width = struct.unpack_from("<Bddff", data, offset)
        if not ObjectType.MINE <= type_index <= ObjectType.OBSTACLE:
            raise ValueError(f"unknown object type: {type_index}")
        if not all(math.isfinite(v) for v in (x, y, length, width)):
            raise ValueError("object contains NaN/Inf")
        objects.append(IntelObject(ObjectType(type_index), x, y, length, width))
        offset += OBJECT_SIZE

    body_end = HEADER_SIZE + OBJECT_SIZE * count
    (received_crc,) = struct.unpack_from("<H", data, body_end)
    actual_crc = crc16(memoryview(data)[4:body_end])
    if received_crc != actual_crc:
        raise ValueError(f"crc mismatch: frame=0x{received_crc:04x} calc=0x{actual_crc:04x}")

    if data[body_end + CRC_SIZE : body_end + CRC_SIZE + TAIL_SIZE] != SUFFIX:
        raise ValueError("bad suffix")

    return IntelFrame(timestamp_ms, tuple(objects))


def build(frame: IntelFrame) -> bytes:
    """按机上格式编码一帧（主要用于测试/回放注入）。"""
    count = len(frame.objects)
    out = bytearray()
    out += MAGIC
    out.append(VERSION)
    out += struct.pack("<QHIH", frame.timestamp_ms, MSG_ID, 2 + OBJECT_SIZE * count, count)
    for obj in frame.objects:
        out += struct.pack(
            "<Bddff", int(obj.type), obj.x, obj.y, obj.length, obj.width
        )
    body_end = HEADER_SIZE + OBJECT_SIZE * count
    out += struct.pack("<H", crc16(memoryview(out)[4:body_end]))
    out += SUFFIX
    return bytes(out)
