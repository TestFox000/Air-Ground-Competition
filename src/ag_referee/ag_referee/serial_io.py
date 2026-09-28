"""裁判板串口收发封装（无 ROS 依赖，可用假串口单测）。

约定：
    * 状态帧由裁判板主动上报（CMD.STATE），上位机只解析
    * 上位机只能下发「射击指令」（CMD.SHOT），不得下发其它控制
    * 帧格式与 CRC 见 ag_core.referee_protocol

波特率/端口以实机为准；香橙派上通常是 /dev/ttyUSB0 @ 115200。
"""

from __future__ import annotations

from typing import List, Optional, Protocol

from ag_core.referee_protocol import CMD, RefereeCodec, RefereeState

__all__ = ["PortLike", "RefereeSerial"]


class PortLike(Protocol):  # 只用到这三个成员，便于用假对象替换
    def read(self, n: int) -> bytes: ...

    def write(self, data: bytes) -> int: ...

    @property
    def in_waiting(self) -> int: ...


class RefereeSerial:
    """把串口对象包成「喂字节 -> 拿状态」的形态。"""

    def __init__(self, port: Optional[PortLike] = None, read_chunk: int = 256) -> None:
        self.port = port
        self.read_chunk = read_chunk
        self.codec = RefereeCodec()
        self.last_state: Optional[RefereeState] = None
        self.stats = {"rx_frames": 0, "rx_dropped": 0, "has_state": False}

    # ---------------- 接收 ----------------
    def poll(self) -> Optional[RefereeState]:
        """读一次串口，返回最新状态（无新帧返回 None）。"""
        if self.port is None:
            return None
        waiting = int(getattr(self.port, "in_waiting", 0) or 0)
        if waiting <= 0:
            return None
        raw = self.port.read(min(waiting, self.read_chunk))
        return self.feed(raw)

    def feed(self, raw: bytes) -> Optional[RefereeState]:
        """喂入原始字节，返回解析出的最新状态。可用于回放抓包。"""
        frames = self.codec.feed(raw)
        if not frames:
            return None
        latest: Optional[RefereeState] = None
        for cmd, payload in frames:
            if cmd == CMD.STATE:
                try:
                    latest = RefereeCodec.parse_state(payload)
                except ValueError:
                    self.stats["rx_dropped"] += 1
                    continue
            self.stats["rx_frames"] += 1
        if latest is not None:
            self.last_state = latest
        self.stats["has_state"] = self.last_state is not None
        return latest

    # ---------------- 发送 ----------------
    def send_shot(self, count: int = 1, duration_ms: int = 200) -> bool:
        if self.port is None:
            return False
        try:
            self.port.write(RefereeCodec.encode_shot(count, duration_ms))
            return True
        except Exception:
            return False

    def send_reset(self) -> bool:
        if self.port is None:
            return False
        try:
            self.port.write(RefereeCodec.encode_reset())
            return True
        except Exception:
            return False

    # ---------------- 统计 ----------------
    @property
    def has_state(self) -> bool:
        return self.last_state is not None


def open_serial(port: str, baud: int = 115200, timeout: float = 0.05):
    """真实串口打开（仅板端/PC 有 pyserial 时可用）。"""
    import serial  # 局部导入，避免无 pyserial 环境导入失败

    return serial.Serial(port, baudrate=baud, timeout=timeout)


def simulate_state_stream(state: RefereeState) -> bytes:
    """把状态对象编码成一帧，用于离线回放与单测。"""
    return RefereeCodec.encode(CMD.STATE, state.to_bytes())


def decode_stream(raw: bytes) -> List[tuple]:
    """便捷函数：一次性解出所有帧。"""
    return RefereeCodec().feed(raw)
