import pytest

from ag_core.referee_protocol import CMD, RefereeCodec, RefereeState
from ag_referee.serial_io import RefereeSerial


class FakePort:
    """最小串口替身：write 进 tx，read 从 rx 取。"""

    def __init__(self) -> None:
        self.rx = bytearray()
        self.tx = bytearray()

    @property
    def in_waiting(self) -> int:
        return len(self.rx)

    def read(self, n: int) -> bytes:
        n = min(n, len(self.rx))
        out = bytes(self.rx[:n])
        del self.rx[:n]
        return out

    def write(self, data: bytes) -> int:
        self.tx.extend(data)
        return len(data)


def _state_frame(st: RefereeState) -> bytes:
    return RefereeCodec.encode(CMD.STATE, st.to_bytes())


def test_feed_parses_state_frame():
    port = FakePort()
    link = RefereeSerial(port)
    st = RefereeState(ammo=7, shots=3, enemy_count=1, friend_count=0, hp=9, seq=42)
    port.rx.extend(_state_frame(st))
    got = link.poll()
    assert got is not None
    assert (got.ammo, got.shots, got.enemy_count, got.seq) == (7, 3, 1, 42)


def test_partial_bytes_are_buffered():
    port = FakePort()
    link = RefereeSerial(port)
    frame = _state_frame(RefereeState(ammo=5, seq=1))
    port.rx.extend(frame[:3])
    assert link.poll() is None
    port.rx.extend(frame[3:])
    assert link.poll() is not None


def test_corrupt_frame_is_dropped():
    port = FakePort()
    link = RefereeSerial(port)
    frame = bytearray(_state_frame(RefereeState(ammo=5, seq=1)))
    frame[-1] ^= 0xFF  # 破坏 crc
    port.rx.extend(bytes(frame))
    assert link.poll() is None


def test_garbage_prefix_then_valid_frame():
    """帧前有乱码时应能重新同步。"""
    port = FakePort()
    link = RefereeSerial(port)
    port.rx.extend(b"\x00\x01\x02")
    port.rx.extend(_state_frame(RefereeState(ammo=9, seq=2)))
    got = link.poll()
    assert got is not None and got.ammo == 9


def test_send_shot_encodes_command():
    port = FakePort()
    link = RefereeSerial(port)
    assert link.send_shot(1, 200) is True
    frame = bytes(port.tx)
    assert frame[:2] == bytes([0xAA, 0x55])
    assert frame[2] == CMD.SHOT
    assert link.stats["rx_frames"] == 0  # 发送不计入接收统计


def test_no_port_returns_none():
    link = RefereeSerial(port=None)
    assert link.poll() is None
    assert link.send_shot() is False
    assert link.send_reset() is False


def test_stats_counts_frames_and_drops():
    port = FakePort()
    link = RefereeSerial(port)
    port.rx.extend(_state_frame(RefereeState(seq=1)))
    port.rx.extend(_state_frame(RefereeState(seq=2)))
    link.poll()
    assert link.stats["rx_frames"] == 2
    assert link.stats["has_state"] is True


if __name__ == "__main__":
    pytest.main([__file__])
