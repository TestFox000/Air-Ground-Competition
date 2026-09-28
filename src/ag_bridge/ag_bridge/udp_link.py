"""UDP 链路封装（无 ROS 依赖，可在本机回环单测）。

一帧 = 4 字节长度前缀 + JSON + 4 字节 crc32（网络序），由 LinkCodec 提供。
本模块只负责 socket 收发与统计。
"""

from __future__ import annotations

import socket
from typing import Optional, Tuple

from ag_core.link_protocol import FieldIntel, LinkCodec, MSG

__all__ = ["UdpLink", "LinkStats"]


class LinkStats:
    def __init__(self) -> None:
        self.tx = 0
        self.rx = 0
        self.bad_crc = 0
        self.unknown = 0

    def as_dict(self) -> dict:
        return {"tx": self.tx, "rx": self.rx, "bad_crc": self.bad_crc, "unknown": self.unknown}


class UdpLink:
    """支持「发送广播」与「接收单播/广播」的最小封装。"""

    def __init__(
        self,
        bind_port: int = 9901,
        peer: Tuple[str, int] = ("255.255.255.255", 9901),
        broadcast: bool = True,
        timeout_s: float = 0.05,
    ) -> None:
        self.peer = peer
        self.stats = LinkStats()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if broadcast:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.settimeout(timeout_s)
        self.sock.bind(("", bind_port))

    # ---------------- 发送 ----------------
    def send_intel(self, intel: FieldIntel) -> int:
        data = LinkCodec.encode(intel)
        n = self.sock.sendto(data, self.peer)
        self.stats.tx += 1
        return n

    def send_heartbeat(self, seq: int = 0, role: str = "air") -> int:
        self.stats.tx += 1
        return self.sock.sendto(LinkCodec.encode_heartbeat(seq, role), self.peer)

    def send_status(self, seq: int, stage: str, score: int, note: str = "") -> int:
        self.stats.tx += 1
        return self.sock.sendto(LinkCodec.encode_status(seq, stage, score, note), self.peer)

    # ---------------- 接收 ----------------
    def recv(self, bufsize: int = 4096) -> Optional[dict]:
        """收一帧，返回解析后的 dict；超时/损坏返回 None。"""
        try:
            data, _addr = self.sock.recvfrom(bufsize)
        except (socket.timeout, BlockingIOError):
            return None
        d = LinkCodec.decode(data)
        if d is None:
            self.stats.bad_crc += 1
            return None
        self.stats.rx += 1
        if d.get("t") == MSG.INTEL:
            return d
        self.stats.unknown += 1
        return d

    def recv_intel(self, bufsize: int = 4096) -> Optional[FieldIntel]:
        d = self.recv(bufsize)
        if d is None or d.get("t") != MSG.INTEL:
            return None
        return FieldIntel.from_dict(d)

    def close(self) -> None:
        try:
            self.sock.close()
        except Exception:
            pass
