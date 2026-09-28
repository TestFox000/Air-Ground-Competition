"""渡众 SmartMaster 裁判系统串口协议（**实机真实帧格式**，非推测）。

来源：车端源码 `~/dzrobot/src/dzjudgment/dzjudgment/src/judgment.cpp`
      （已取回本地：`reference/car_src/dzjudgment/src/judgment.cpp`）
      —— 这是官方随车提供的实现，字段偏移直接照抄，不再靠猜。

帧格式：
    B5 5B <len> <cmd> <data...>
      len   = cmd + data 的字节数（车端读 `len` 个字节到 charArray[3..]）
      cmd   = 0x02 / 0x03 / 0x04 / 0x07（见 CMD）
    车端在流里用「搜 0xB5 再取下一字节」的方式重同步，本模块同样实现了逐字节状态机。

cmd 含义：
    0x02  全部物资队列   上位机 读
    0x03  对方（敌方）物资/受击信息
    0x04  己方物资/受击信息
    0x07  血量与受击状态：data = [targethp, self_hp, Bing_hit, Shooting_Count, ammo]
          - targethp      靶标剩余血量
          - self_hp       己方剩余血量
          - Bing_hit      0x01 左 / 0x02 前 / 0x03 右 / 0x04 后 / 0x10 未被击打
          - Shooting_Count 已射击次数
          - ammo          当前可射击弹数

下发只有一条：**射击指令**（车端源码写死 6 字节）
    B5 5B 03 21 01 25
    其中 0x21 = 射击命令字，0x25 = 校验位。

⚠️ 与 `referee_protocol.py` 的关系：
    `referee_protocol.py` 是「官方包不可用时的自研兜底」（0xAA55 帧头 + crc8）。
    实机上应优先使用官方路径：`ros2 launch dzactuator judgment.launch.py`
    （`dzjudgment` 节点自己解串口，把结果发布成 ROS 话题，见 `referee_node`
     的 `source=dz` 模式），本模块用于需要直接抓串口/离线复现的场合。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

__all__ = [
    "SOF",
    "CMD",
    "HIT_NONE",
    "HIT_DIR",
    "SHOT_FRAME",
    "DzFrame",
    "DzState",
    "DzFrameParser",
    "parse_state_payload",
]

SOF = (0xB5, 0x5B)


class CMD:
    ALL_MATERIAL = 0x02     # 全部物资
    ENEMY_MATERIAL = 0x03   # 对方物资
    SELF_MATERIAL = 0x04    # 己方物资
    HP_AND_HIT = 0x07       # 血量 + 受击 + 射击数 + 弹药
    SHOT = 0x21             # 上位机 -> 裁判板：射击


#: 受击方向
HIT_NONE = 0x10
HIT_DIR = {0x01: "left", 0x02: "front", 0x03: "right", 0x04: "back", HIT_NONE: "none"}

#: 标准射击帧（裁判系统操作指南 + dzjudgment 源码：B5 5B 03 21 01 25）
SHOT_FRAME = bytes([0xB5, 0x5B, 0x03, CMD.SHOT, 0x01, 0x25])


@dataclass
class DzFrame:
    """一帧原始数据。"""

    cmd: int
    payload: bytes               # cmd 之后的数据字节

    @property
    def materials(self) -> List[int]:
        """物资队列（0x02/0x03/0x04）。

        注意：车端实现从 payload 起点取 `len - 2` 个字节，即**丢掉最后一个字节**，
        这里保持一致，免得和实机对不上。
        """
        return list(self.payload[: max(0, len(self.payload) - 1)])


@dataclass
class DzState:
    """0x07 帧解析结果。"""

    targethp: int = 0
    self_hp: int = 0
    hit: int = HIT_NONE
    shots: int = 0
    ammo: int = 0

    @property
    def hit_dir(self) -> Optional[str]:
        return None if self.hit == HIT_NONE else HIT_DIR.get(self.hit, "unknown")

    @property
    def under_fire(self) -> bool:
        return self.hit != HIT_NONE


def parse_state_payload(payload: bytes) -> DzState:
    """解析 0x07 帧的 payload -> DzState。"""
    if len(payload) < 5:
        raise ValueError(f"0x07 帧数据不足：需要 5 字节，收到 {len(payload)}")
    return DzState(
        targethp=payload[0],
        self_hp=payload[1],
        hit=payload[2],
        shots=payload[3],
        ammo=payload[4],
    )


class DzFrameParser:
    """逐字节状态机，抗丢字节/半帧（与车端 recvESP32Info 同样的重同步策略）。"""

    def __init__(self) -> None:
        self._buf = bytearray()
        self.dropped = 0     # 不是帧头而被丢弃的字节数，用于现场判断线序/波特率

    def feed(self, data: bytes) -> List[DzFrame]:
        self._buf.extend(data)
        out: List[DzFrame] = []
        while True:
            # 1) 找帧头
            i = self._buf.find(bytes(SOF))
            if i < 0:
                # 只保留最后一个字节（可能是被截断的 0xB5）
                if len(self._buf) > 1:
                    self.dropped += len(self._buf) - 1
                    del self._buf[:-1]
                break
            if i > 0:
                self.dropped += i
                del self._buf[:i]
            # 2) 等长度字节
            if len(self._buf) < 3:
                break
            length = self._buf[2]
            total = 3 + length
            if length < 1:
                # 长度不合理：丢掉这个帧头重新同步
                self.dropped += 1
                del self._buf[0]
                continue
            if len(self._buf) < total:
                break   # 半帧，等后续数据
            body = bytes(self._buf[3:total])
            del self._buf[:total]
            out.append(DzFrame(cmd=body[0], payload=body[1:]))
        return out

    @staticmethod
    def encode_shot() -> bytes:
        """需要直接控串口时的射击指令。"""
        return SHOT_FRAME

    @staticmethod
    def encode_frame(cmd: int, payload: bytes = b"") -> bytes:
        """构造一帧（主要用于自测/回放）。"""
        body = bytes([cmd]) + bytes(payload)
        return bytes(SOF) + bytes([len(body)]) + body

    @staticmethod
    def state_frame(state: DzState) -> bytes:
        return DzFrameParser.encode_frame(
            CMD.HP_AND_HIT,
            bytes([state.targethp, state.self_hp, state.hit, state.shots, state.ammo]),
        )


@dataclass
class DzMaterial:
    """物资队列快照（0x02/0x03/0x04）。"""

    all_items: List[int] = field(default_factory=list)
    enemy_items: List[int] = field(default_factory=list)
    self_items: List[int] = field(default_factory=list)

    def update(self, frame: DzFrame) -> bool:
        items = frame.materials
        if frame.cmd == CMD.ALL_MATERIAL:
            self.all_items = items
        elif frame.cmd == CMD.ENEMY_MATERIAL:
            self.enemy_items = items
        elif frame.cmd == CMD.SELF_MATERIAL:
            self.self_items = items
        else:
            return False
        return True


def split_frames(stream: bytes) -> Tuple[List[DzFrame], int]:
    """一次性解析字节流，返回 (帧列表, 丢弃字节数)。"""
    p = DzFrameParser()
    frames = p.feed(stream)
    return frames, p.dropped
