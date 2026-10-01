"""车载裁判系统串口协议编解码。

已知事实（《SmartMaster 裁判系统操作指南》《硬件操作指南》）：
    * OrangePi 通过 USB 串口与车尾裁判板通信
    * 上位机可读取：靶子血量、已抢占物资队列、己方/对方血量、
      靶子是否被激光照射（0x01~0x04 前后左右，0x10 未照射）、
      已射击次数、当前可射击弹数
    * 上位机只能下发 **射击指令**

⚠️ 本文件是**自研兜底**实现（0xAA55 + crc8），只在官方包不可用时才用。
   实机的真实帧格式已经拿到（渡众 `dzjudgment` 源码），见
   **`ag_core/dz_referee.py`**（B5 5B 帧头、cmd 0x02/0x03/0x04/0x07、
   射击指令 `B5 5B 03 21 01 25`）。
   实机推荐直接跑官方的 `dzjudgment` 节点，`referee_node` 用 `source=dz` 对接。

帧格式：
    [0] 0xAA  [1] 0x55  [2] cmd  [3] len  [4..4+len) payload  [4+len] crc8
    CRC8: poly 0x07, init 0x00（对 cmd+len+payload 计算）
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

__all__ = ["RefereeState", "RefereeCodec", "CMD", "SOF"]

SOF = (0xAA, 0x55)


class CMD:
    STATE = 0x01   # 裁判板 -> 上位机：状态上报
    SHOT = 0x02    # 上位机 -> 裁判板：射击指令
    ACK = 0x03
    RESET = 0x04


@dataclass
class RefereeState:
    ammo: int = 10                 # 当前可射击弹数
    shots: int = 0                 # 已射击次数
    enemy_count: int = 0           # 敌方受击侧计数器
    friend_count: int = 0          # 友方受击侧计数器
    illuminated: int = 0x10        # 0x01~0x04 方向，0x10 未被照射
    hp: int = 10                   # 满格 10
    seq: int = 0

    @property
    def illuminated_dir(self) -> Optional[int]:
        return None if self.illuminated == 0x10 else self.illuminated

    @classmethod
    def from_bytes(cls, payload: bytes) -> "RefereeState":
        if len(payload) < 7:
            raise ValueError(f"状态帧 payload 长度不足: {len(payload)}")
        return cls(
            ammo=payload[0],
            shots=payload[1],
            enemy_count=payload[2],
            friend_count=payload[3],
            illuminated=payload[4],
            hp=payload[5],
            seq=payload[6] | (payload[7] << 8) if len(payload) >= 8 else 0,
        )

    def to_bytes(self) -> bytes:
        return bytes(
            [
                self.ammo & 0xFF,
                self.shots & 0xFF,
                self.enemy_count & 0xFF,
                self.friend_count & 0xFF,
                self.illuminated & 0xFF,
                self.hp & 0xFF,
                self.seq & 0xFF,
                (self.seq >> 8) & 0xFF,
            ]
        )


def crc8(data: bytes, poly: int = 0x07, init: int = 0x00) -> int:
    crc = init
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc & 0xFF


class RefereeCodec:
    """带缓冲的帧编解码器，可直接喂串口字节流。"""

    def __init__(self) -> None:
        self._buf = bytearray()

    # ---- 编码 ----
    @staticmethod
    def encode(cmd: int, payload: bytes = b"") -> bytes:
        head = bytes([SOF[0], SOF[1], cmd & 0xFF, len(payload) & 0xFF])
        body = head + bytes(payload)
        return body + bytes([crc8(head[2:] + bytes(payload))])

    @staticmethod
    def encode_shot(count: int = 1, duration_ms: int = 200) -> bytes:
        payload = bytes([count & 0xFF, duration_ms & 0xFF, (duration_ms >> 8) & 0xFF])
        return RefereeCodec.encode(CMD.SHOT, payload)

    @staticmethod
    def encode_reset() -> bytes:
        return RefereeCodec.encode(CMD.RESET)

    # ---- 解码 ----
    def feed(self, data: bytes) -> List[tuple]:
        """喂入字节流，返回 [(cmd, payload), ...] 完整帧列表。"""
        self._buf.extend(data)
        frames: List[tuple] = []
        while True:
            start = self._find_sof()
            if start < 0:
                # ⚠️ 末尾可能留着半个帧头（0xAA 是缓冲区最后一个字节）：
                #    它和**下一包**开头的 0x55 才拼得出 SOF。整段清掉会把
                #    这个 0xAA 也丢掉，于是跨包对齐的帧**整帧丢失**
                #    （串口长会话里必然发生，只是概率低所以难抓）。
                if self._buf and self._buf[-1] == SOF[0]:
                    del self._buf[:-1]
                else:
                    self._buf.clear()
                break
            if start > 0:
                del self._buf[:start]
            if len(self._buf) < 5:
                break
            length = self._buf[3]
            total = 4 + length + 1
            if len(self._buf) < total:
                break
            frame = bytes(self._buf[:total])
            cmd = frame[2]
            payload = frame[4 : 4 + length]
            if crc8(frame[2 : 4 + length]) != frame[4 + length]:
                del self._buf[:1]  # 校验失败，滑过一个字节重新同步
                continue
            del self._buf[:total]
            frames.append((cmd, payload))
        return frames

    def _find_sof(self) -> int:
        for i in range(len(self._buf) - 1):
            if self._buf[i] == SOF[0] and self._buf[i + 1] == SOF[1]:
                return i
        return -1

    @staticmethod
    def parse_state(payload: bytes) -> RefereeState:
        return RefereeState.from_bytes(payload)
