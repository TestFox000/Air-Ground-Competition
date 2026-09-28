import socket

import pytest

from ag_core.link_protocol import FieldIntel
from ag_bridge.udp_link import UdpLink


def _free_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("", 0))
    port = s.getsockname()[1]
    s.close()
    return port


@pytest.fixture()
def pair():
    a_port, b_port = _free_port(), _free_port()
    air = UdpLink(bind_port=a_port, peer=("127.0.0.1", b_port), broadcast=False, timeout_s=0.5)
    car = UdpLink(bind_port=b_port, peer=("127.0.0.1", a_port), broadcast=False, timeout_s=0.5)
    yield air, car
    air.close()
    car.close()


def test_intel_roundtrip(pair):
    air, car = pair
    intel = FieldIntel(mines=["C3", "D4"], obstacles=["J7"], bridge_a="K4", bridge_b="K9",
                       target_cells=["D2", "E2"], target_face="blue", seq=7)
    air.send_intel(intel)
    got = car.recv_intel()
    assert got is not None
    assert got.mines == ["C3", "D4"]
    assert got.bridge_a == "K4"
    assert got.seq == 7
    assert air.stats.tx == 1
    assert car.stats.rx == 1


def test_heartbeat_is_not_intel(pair):
    air, car = pair
    air.send_heartbeat(seq=3, role="air")
    d = car.recv()
    assert d is not None
    assert d["t"] == "heartbeat"
    assert car.recv_intel() is None


def test_corrupted_frame_counts_bad_crc(pair):
    air, car = pair
    good = FieldIntel(mines=["B2"], seq=1)
    air.send_intel(good)
    # 手工构造一帧并破坏最后一个字节
    from ag_core.link_protocol import LinkCodec

    raw = bytearray(LinkCodec.encode(FieldIntel(mines=["Z9"], seq=2)))
    raw[-1] ^= 0xFF
    air.sock.sendto(bytes(raw), air.peer)
    car.recv_intel()  # 先吃掉正常帧
    assert car.recv_intel() is None
    assert car.stats.bad_crc == 1


def test_status_message_roundtrip(pair):
    air, car = pair
    air.send_status(seq=5, stage="mine", score=30, note="C3 done")
    d = car.recv()
    assert d is not None
    assert d["t"] == "status"
    assert d["score"] == 30


if __name__ == "__main__":
    pytest.main([__file__])
