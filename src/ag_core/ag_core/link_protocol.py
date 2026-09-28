"""空地协同链路协议：无人机 -> 地面车的侦察情报。

通信约束（《15空地协同侦排挑战赛》）：
    * 禁止公网通信
    * 允许自带路由器布设无线局域网 / 数传 / 蓝牙 / 图传 / Lora
    * 现场多场地同时比赛，需考虑同频干扰 -> 协议带 seq + crc32，可丢弃乱序/损坏帧

这里用「JSON over UDP」便于调试；如需更低带宽可换成 msgpack/CBOR，接口不变。
"""

from __future__ import annotations

import json
import struct
import time
import zlib
from dataclasses import dataclass, field
from typing import List, Optional

__all__ = ["FieldIntel", "LinkCodec", "MSG"]

VERSION = 1


class MSG:
    INTEL = "intel"        # 无人机 -> 车：侦察情报
    HEARTBEAT = "heartbeat"
    ACK = "ack"
    STATUS = "status"      # 车 -> 无人机：执行状态


@dataclass
class FieldIntel:
    mines: List[str] = field(default_factory=list)            # 8 个地雷格号
    obstacles: List[str] = field(default_factory=list)        # 8 个障碍物格号
    bridge_a: Optional[str] = None                            # 桥洞一 可通行格（5 选 1）
    bridge_b: Optional[str] = None                            # 桥洞二 可通行格（6 选 1）
    target_cells: List[str] = field(default_factory=list)     # 敌我识别区（相连两格）
    target_face: Optional[str] = None                         # 'blue' / 'red' / None
    ts: float = field(default_factory=time.time)
    seq: int = 0

    def to_dict(self) -> dict:
        return {
            "v": VERSION,
            "t": MSG.INTEL,
            "seq": self.seq,
            "ts": self.ts,
            "mines": self.mines,
            "obstacles": self.obstacles,
            "bridge_a": self.bridge_a,
            "bridge_b": self.bridge_b,
            "target_cells": self.target_cells,
            "target_face": self.target_face,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "FieldIntel":
        return cls(
            mines=list(d.get("mines", [])),
            obstacles=list(d.get("obstacles", [])),
            bridge_a=d.get("bridge_a"),
            bridge_b=d.get("bridge_b"),
            target_cells=list(d.get("target_cells", [])),
            target_face=d.get("target_face"),
            ts=float(d.get("ts", time.time())),
            seq=int(d.get("seq", 0)),
        )

    def is_complete(self) -> bool:
        return bool(self.mines) and (self.bridge_a is not None or self.bridge_b is not None)


class LinkCodec:
    """编码为 4 字节长度前缀 + JSON + 4 字节 crc32（网络序）。"""

    @staticmethod
    def encode(obj) -> bytes:
        d = obj.to_dict() if hasattr(obj, "to_dict") else obj
        body = json.dumps(d, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        crc = zlib.crc32(body) & 0xFFFFFFFF
        return struct.pack("!I", len(body)) + body + struct.pack("!I", crc)

    @staticmethod
    def decode(buf: bytes) -> Optional[dict]:
        if len(buf) < 8:
            return None
        (n,) = struct.unpack("!I", buf[:4])
        if len(buf) < 8 + n:
            return None
        body = buf[4 : 4 + n]
        (crc,) = struct.unpack("!I", buf[4 + n : 8 + n])
        if zlib.crc32(body) & 0xFFFFFFFF != crc:
            return None
        return json.loads(body.decode("utf-8"))

    @staticmethod
    def encode_heartbeat(seq: int = 0, role: str = "car") -> bytes:
        return LinkCodec.encode({"v": VERSION, "t": MSG.HEARTBEAT, "seq": seq, "role": role, "ts": time.time()})

    @staticmethod
    def encode_status(seq: int, stage: str, score: int, note: str = "") -> bytes:
        return LinkCodec.encode(
            {"v": VERSION, "t": MSG.STATUS, "seq": seq, "stage": stage, "score": score, "note": note, "ts": time.time()}
        )
