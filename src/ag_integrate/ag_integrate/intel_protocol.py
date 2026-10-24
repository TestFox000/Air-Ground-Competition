"""空地 UDP 帧编解码（端口 50000）。

三条消息（msg_id 区分）：
    0x4001  机载 dzuav → 车端   情报帧：13 类检测目标 + 世界米制坐标
    0x4002  机载 dzuav → 车端   车真实位姿：无人机观测的车 x/y/yaw
    0x4003  机载规划   → 车端   车命令帧：reset + route（官方 path_executor 语法）

0x4001 字节布局（全部小端），严格对应
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
VERSION_V2 = 0x11   # 2026-10-24：目标段尾部追加 1 字节 conf（uint8，conf*255）
MSG_ID = 0x4001
MSG_ID_CAR_POSE = 0x4002   # 无人机观测的车真实位姿（位置+朝向），用于根除绕圈问题
MSG_ID_CAR_COMMAND = 0x4003  # 机载规划下发的车命令（reset + route），见文件末尾
SUFFIX = b"\xfe\xfe\xfe\xfe"

HEADER_SIZE = 21          # magic4 + version1 + ts8 + msgid2 + length4 + count2 (0x4001)
HEADER_SIZE_NO_COUNT = 19  # magic4 + version1 + ts8 + msgid2 + length4 (0x4002，无 count)
OBJECT_SIZE = 25          # type1 + x8 + y8 + length4 + width4   (v1/0x10)
OBJECT_SIZE_V2 = 26       # v1 + conf1 (uint8 = round(conf*255)) (v2/0x11)
CRC_SIZE = 2
TAIL_SIZE = 4             # suffix

CAR_POSE_SIZE = 20        # x8 + y8 + yaw4


class ObjectType(IntEnum):
    """目标类别，顺序必须与 dzuav main.cpp 的 className 一致。

    2026-10-16 起切换 D 方案 13 类统一模型（hbb_d13_best.pt）。
    PAPER 是场地纸片（雷图卡/桥顶箭头标志），语义由 world_model 按落格判定：
    普通格 PAPER=雷；K 列桥洞区 PAPER=通行箭头。
    """

    PAPER = 0         # paper 场地纸片（雷/箭头，落格判语义）
    CELL_LABEL = 1    # cell_label 地面格号
    CAR = 2           # car 行驶车辆（车位姿链）
    TARGET_RED = 3    # target_red 友方（禁火）
    TARGET_BLUE = 4   # target_blue 敌方
    OBSTACLE = 5      # obstacle 障碍
    BUCKET_RED = 6    # bucket_red 红桶（禁火）
    BUCKET_GREEN = 7  # bucket_green 绿桶
    BUCKET_CYAN = 8   # bucket_cyan 青桶
    CAR_START = 9     # car_start 智能车起点
    CAR_END = 10      # car_end 智能车终点
    UAV_START = 11    # uav_start 无人机起点
    UAV_END = 12      # uav_end 无人机终点


@dataclass(frozen=True)
class IntelObject:
    type: ObjectType
    x: float
    y: float
    length: float
    width: float
    # 2026-10-24 v2(0x11) 起随帧携带 D13 置信度；v1(0x10) 帧解析时填 1.0
    # （= 满置信，与旧"二元票/整帧替换"语义完全等价——老发射端零破坏）。
    conf: float = 1.0


@dataclass(frozen=True)
class IntelFrame:
    timestamp_ms: int
    objects: tuple[IntelObject, ...]


@dataclass(frozen=True)
class CarPose:
    """无人机观测到的车的真实位姿（msg_id=0x4002），用于根除绕圈问题。"""
    timestamp_ms: int
    x: float          # 车的世界坐标 x（米）
    y: float          # 车的世界坐标 y（米）
    yaw: float        # 车的真实朝向（弧度，场地坐标系）


def detect_msg_id(data: bytes) -> int | None:
    """快速判断帧的 msg_id（不做完整校验）；格式不对返回 None。"""
    if len(data) < 15 or data[0:4] != MAGIC or data[4] not in (VERSION, VERSION_V2):
        return None
    msg_id = struct.unpack_from("<H", data, 13)[0]
    return msg_id


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
    version = data[4]
    if version not in (VERSION, VERSION_V2):
        raise ValueError(f"unsupported version byte: 0x{version:02x}")
    obj_size = OBJECT_SIZE_V2 if version == VERSION_V2 else OBJECT_SIZE

    timestamp_ms, msg_id, declared_len, count = struct.unpack_from("<QHIH", data, 5)
    if msg_id != MSG_ID:
        raise ValueError(f"unexpected message id: 0x{msg_id:04x}")

    expected_declared = 2 + obj_size * count
    if declared_len != expected_declared:
        raise ValueError(
            f"declared length {declared_len} != 2 + {obj_size}*{count} ({expected_declared})"
        )

    expected_size = HEADER_SIZE + obj_size * count + CRC_SIZE + TAIL_SIZE
    if len(data) != expected_size:
        raise ValueError(f"frame size {len(data)} != expected {expected_size}")

    objects: list[IntelObject] = []
    offset = HEADER_SIZE
    for _ in range(count):
        type_index, x, y, length, width = struct.unpack_from("<Bddff", data, offset)
        if not ObjectType.PAPER <= type_index <= ObjectType.UAV_END:
            raise ValueError(f"unknown object type: {type_index}")
        if not all(math.isfinite(v) for v in (x, y, length, width)):
            raise ValueError("object contains NaN/Inf")
        conf = 1.0
        if version == VERSION_V2:
            (c8,) = struct.unpack_from("<B", data, offset + OBJECT_SIZE)
            conf = min(1.0, c8 / 255.0)
            if not math.isfinite(conf):
                raise ValueError("object conf not finite")
        objects.append(IntelObject(ObjectType(type_index), x, y, length, width, conf))
        offset += obj_size

    body_end = HEADER_SIZE + obj_size * count
    (received_crc,) = struct.unpack_from("<H", data, body_end)
    actual_crc = crc16(memoryview(data)[4:body_end])
    if received_crc != actual_crc:
        raise ValueError(f"crc mismatch: frame=0x{received_crc:04x} calc=0x{actual_crc:04x}")

    if data[body_end + CRC_SIZE : body_end + CRC_SIZE + TAIL_SIZE] != SUFFIX:
        raise ValueError("bad suffix")

    return IntelFrame(timestamp_ms, tuple(objects))


def build(frame: IntelFrame, *, version: int = VERSION_V2) -> bytes:
    """按机上格式编码一帧（测试/回放注入）。

    version=VERSION_V2（默认，0x11）：目标段带 conf 字节；
    version=VERSION（0x10）：发老格式（conf 丢弃）——兼容性测试用。
    """
    count = len(frame.objects)
    obj_size = OBJECT_SIZE_V2 if version == VERSION_V2 else OBJECT_SIZE
    out = bytearray()
    out += MAGIC
    out.append(version)
    out += struct.pack("<QHIH", frame.timestamp_ms, MSG_ID, 2 + obj_size * count, count)
    for obj in frame.objects:
        out += struct.pack(
            "<Bddff", int(obj.type), obj.x, obj.y, obj.length, obj.width
        )
        if version == VERSION_V2:
            c = min(1.0, max(0.0, getattr(obj, "conf", 1.0)))
            out.append(int(round(c * 255)) & 0xFF)
    body_end = HEADER_SIZE + obj_size * count
    out += struct.pack("<H", crc16(memoryview(out)[4:body_end]))
    out += SUFFIX
    return bytes(out)


# ---- msg_id=0x4002：车真实位姿（根除绕圈问题）----

_CAR_POSE_FRAME_SIZE = HEADER_SIZE_NO_COUNT + CAR_POSE_SIZE + CRC_SIZE + TAIL_SIZE


def parse_car_pose(data: bytes) -> CarPose:
    """解析车位姿帧（msg_id=0x4002）；任何不一致抛 ValueError。"""
    if len(data) != _CAR_POSE_FRAME_SIZE:
        raise ValueError(f"car_pose frame size {len(data)} != expected {_CAR_POSE_FRAME_SIZE}")
    if data[0:4] != MAGIC:
        raise ValueError("bad magic prefix")
    if data[4] != VERSION:
        raise ValueError(f"unsupported version byte: 0x{data[4]:02x}")

    timestamp_ms, msg_id, declared_len = struct.unpack_from("<QHI", data, 5)
    if msg_id != MSG_ID_CAR_POSE:
        raise ValueError(f"unexpected message id: 0x{msg_id:04x}")
    if declared_len != CAR_POSE_SIZE:
        raise ValueError(f"declared length {declared_len} != {CAR_POSE_SIZE}")

    x, y, yaw = struct.unpack_from("<ddf", data, HEADER_SIZE_NO_COUNT)
    if not all(math.isfinite(v) for v in (x, y, yaw)):
        raise ValueError("car pose contains NaN/Inf")

    body_end = HEADER_SIZE_NO_COUNT + CAR_POSE_SIZE
    (received_crc,) = struct.unpack_from("<H", data, body_end)
    actual_crc = crc16(memoryview(data)[4:body_end])
    if received_crc != actual_crc:
        raise ValueError(f"crc mismatch: frame=0x{received_crc:04x} calc=0x{actual_crc:04x}")

    if data[body_end + CRC_SIZE : body_end + CRC_SIZE + TAIL_SIZE] != SUFFIX:
        raise ValueError("bad suffix")

    return CarPose(timestamp_ms, x, y, yaw)


def build_car_pose(pose: CarPose) -> bytes:
    """编码车位姿帧（msg_id=0x4002），用于测试/回放。"""
    out = bytearray()
    out += MAGIC
    out.append(VERSION)
    out += struct.pack("<QHI", pose.timestamp_ms, MSG_ID_CAR_POSE, CAR_POSE_SIZE)
    out += struct.pack("<ddf", pose.x, pose.y, pose.yaw)
    body_end = HEADER_SIZE_NO_COUNT + CAR_POSE_SIZE
    out += struct.pack("<H", crc16(memoryview(out)[4:body_end]))
    out += SUFFIX
    return bytes(out)


# ---- msg_id=0x4003：机载规划 -> 车端 车命令帧 ----
# 官方控制要求「无人机的定位、建图、识别、规划与控制等实时任务须由机载平台自主
# 完成」：规划层在机上。机载规划进程与车端不再同机，reset/route 不能再靠车端本机
# 的 /nav/waypoint ROS 话题传递。本帧沿用既有空地 UDP 广播链路把两条命令原样运到
# 车端，车端薄接收后转发到 /nav/waypoint（官方 path_executor 消费）。
#
# 字节布局（小端）：
#     magic     4 × 0xef
#     version   1    0x10
#     timestamp uint64   机载毫秒时间戳
#     msg_id    uint16   0x4003
#     length    uint32   = 2+len(reset) + 2+len(route)
#     reset_len uint16   | reset bytes(ASCII)
#     route_len uint16   | route bytes(ASCII)
#     crc16     uint16   CRC-16/Modbus，覆盖 version 到最后一个命令字节
#     suffix    4 × 0xfe
#
# 命令串是 car_commands 编码后的**车端方言**字面量（朝向 token 已在 car_commands
# 翻译成本机->车端架），本模块只做字节搬运与结构校验，绝不改写命令内容。

_CMD_LEN_SIZE = 2
_CMD_ASCII = "ascii"
_CAR_CMD_MIN_SIZE = HEADER_SIZE_NO_COUNT + _CMD_LEN_SIZE * 2 + CRC_SIZE + TAIL_SIZE


@dataclass(frozen=True)
class CarCommandFrame:
    """机载规划产出的车命令（msg_id=0x4003）。

    reset: "reset:<格号>,<车端朝向 token>"；空串 = 本局不下发 reset。
    route: 格号序列（空格分隔，`*`=停靠射击，末尾 ,x/,y=终点朝向）；不能为空。
    """

    timestamp_ms: int
    reset: str
    route: str


def _validate_reset(reset: str) -> str:
    """空 reset 合法（本局不发锚定）；非空必须是 reset 命令，防脏串误发上底盘。"""
    if reset == "":
        return reset
    if not reset.startswith("reset:"):
        raise ValueError(f"reset 命令必须以 'reset:' 开头，收到 {reset!r}")
    return reset


def _validate_route(route: str) -> str:
    if not route.strip():
        raise ValueError("route 命令不能为空")
    return route


def _encode_cmd_text(text: str, what: str) -> bytes:
    """命令串 -> 严格 ASCII 字节（非 ASCII 会让车端解析整条丢弃，宁错不发）。"""
    raw = text.encode(_CMD_ASCII)
    if len(raw) > 0xFFFF:
        raise ValueError(f"{what} 过长: {len(raw)} 字节 > 65535")
    return raw


def build_car_command(frame: CarCommandFrame) -> bytes:
    """编码车命令帧（msg_id=0x4003），用于机载下发与测试/回放。"""
    reset_b = _encode_cmd_text(_validate_reset(frame.reset), "reset")
    route_b = _encode_cmd_text(_validate_route(frame.route), "route")
    body = (struct.pack("<H", len(reset_b)) + reset_b
            + struct.pack("<H", len(route_b)) + route_b)
    out = bytearray()
    out += MAGIC
    out.append(VERSION)
    out += struct.pack("<QHI", frame.timestamp_ms, MSG_ID_CAR_COMMAND, len(body))
    out += body
    body_end = HEADER_SIZE_NO_COUNT + len(body)
    out += struct.pack("<H", crc16(memoryview(out)[4:body_end]))
    out += SUFFIX
    return bytes(out)


def parse_car_command(data: bytes) -> CarCommandFrame:
    """解析车命令帧（msg_id=0x4003）；任何不一致都抛 ValueError（不返回半成品）。"""
    if len(data) < _CAR_CMD_MIN_SIZE:
        raise ValueError(f"car command frame too short: {len(data)} bytes")
    if data[0:4] != MAGIC:
        raise ValueError("bad magic prefix")
    if data[4] != VERSION:
        raise ValueError(f"unsupported version byte: 0x{data[4]:02x}")

    timestamp_ms, msg_id, declared_len = struct.unpack_from("<QHI", data, 5)
    if msg_id != MSG_ID_CAR_COMMAND:
        raise ValueError(f"unexpected message id: 0x{msg_id:04x}")

    total = HEADER_SIZE_NO_COUNT + declared_len + CRC_SIZE + TAIL_SIZE
    if len(data) != total:
        raise ValueError(f"frame size {len(data)} != declared {total}")

    body_end = HEADER_SIZE_NO_COUNT + declared_len
    offset = HEADER_SIZE_NO_COUNT
    (reset_len,) = struct.unpack_from("<H", data, offset)
    offset += _CMD_LEN_SIZE
    if offset + reset_len + _CMD_LEN_SIZE > body_end:
        raise ValueError("reset 长度越界")
    reset_b = data[offset:offset + reset_len]
    offset += reset_len
    (route_len,) = struct.unpack_from("<H", data, offset)
    offset += _CMD_LEN_SIZE
    if offset + route_len != body_end:
        raise ValueError("route 长度与声明长度不符")
    route_b = data[offset:body_end]

    try:
        reset = reset_b.decode(_CMD_ASCII)
        route = route_b.decode(_CMD_ASCII)
    except UnicodeDecodeError as exc:
        raise ValueError(f"命令串非 ASCII: {exc}") from exc
    if "\x00" in reset or "\x00" in route:
        raise ValueError("命令串含 NUL 字节")
    _validate_reset(reset)
    _validate_route(route)

    (received_crc,) = struct.unpack_from("<H", data, body_end)
    actual_crc = crc16(memoryview(data)[4:body_end])
    if received_crc != actual_crc:
        raise ValueError(f"crc mismatch: frame=0x{received_crc:04x} calc=0x{actual_crc:04x}")

    if data[body_end + CRC_SIZE : body_end + CRC_SIZE + TAIL_SIZE] != SUFFIX:
        raise ValueError("bad suffix")

    return CarCommandFrame(timestamp_ms, reset, route)
