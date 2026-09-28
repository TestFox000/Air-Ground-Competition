import pytest

from ag_core.link_protocol import FieldIntel, LinkCodec
from ag_core.referee_protocol import CMD, RefereeCodec, RefereeState, crc8


# ---------------- 裁判系统串口协议 ----------------
def test_crc8_reference():
    assert crc8(b"\x01\x00") == crc8(b"\x01\x00")
    assert crc8(b"\x01\x00") != crc8(b"\x02\x00")


def test_state_frame_roundtrip():
    st = RefereeState(ammo=7, shots=3, enemy_count=2, friend_count=0, illuminated=0x10, hp=9, seq=258)
    frame = RefereeCodec.encode(CMD.STATE, st.to_bytes())
    codec = RefereeCodec()
    frames = codec.feed(frame)
    assert len(frames) == 1
    cmd, payload = frames[0]
    assert cmd == CMD.STATE
    back = RefereeState.from_bytes(payload)
    assert back == st


def test_shot_command_encodes():
    f = RefereeCodec.encode_shot(count=1, duration_ms=200)
    assert f[0] == 0xAA and f[1] == 0x55
    assert f[2] == CMD.SHOT
    assert f[3] == 3


def test_feed_tolerates_garbage_prefix():
    codec = RefereeCodec()
    st = RefereeState(ammo=5, seq=1)
    frame = RefereeCodec.encode(CMD.STATE, st.to_bytes())
    frames = codec.feed(b"\x00\xff\x13" + frame)
    assert len(frames) == 1
    assert frames[0][0] == CMD.STATE


def test_corrupt_frame_is_rejected():
    codec = RefereeCodec()
    st = RefereeState(ammo=5)
    frame = bytearray(RefereeCodec.encode(CMD.STATE, st.to_bytes()))
    frame[-1] ^= 0xFF  # 破坏 crc
    assert codec.feed(bytes(frame)) == []


def test_partial_frame_buffers():
    codec = RefereeCodec()
    st = RefereeState(ammo=4)
    frame = RefereeCodec.encode(CMD.STATE, st.to_bytes())
    assert codec.feed(frame[:3]) == []
    assert len(codec.feed(frame[3:])) == 1


def test_illuminated_dir():
    assert RefereeState(illuminated=0x10).illuminated_dir is None
    assert RefereeState(illuminated=0x02).illuminated_dir == 0x02


# ---------------- 空地链路协议 ----------------
def test_intel_roundtrip():
    intel = FieldIntel(
        mines=["D11", "B7", "F7", "D4"],
        obstacles=["C3", "H9"],
        bridge_a="K4",
        bridge_b="K9",
        target_cells=["D2", "E2"],
        target_face="blue",
        seq=12,
    )
    buf = LinkCodec.encode(intel)
    d = LinkCodec.decode(buf)
    assert d is not None
    back = FieldIntel.from_dict(d)
    assert back.mines == intel.mines
    assert back.bridge_a == "K4"
    assert back.target_face == "blue"
    assert back.seq == 12


def test_decode_rejects_bad_crc():
    intel = FieldIntel(mines=["A1"])
    buf = bytearray(LinkCodec.encode(intel))
    buf[-1] ^= 0xFF
    assert LinkCodec.decode(bytes(buf)) is None


def test_decode_requires_complete_buffer():
    intel = FieldIntel(mines=["A1", "B2"])
    buf = LinkCodec.encode(intel)
    assert LinkCodec.decode(buf[: len(buf) - 2]) is None


def test_heartbeat_and_status():
    assert LinkCodec.decode(LinkCodec.encode_heartbeat(seq=3))["t"] == "heartbeat"
    s = LinkCodec.decode(LinkCodec.encode_status(seq=4, stage="mine", score=20))
    assert s["stage"] == "mine" and s["score"] == 20


def test_intel_completeness():
    assert not FieldIntel().is_complete()
    assert FieldIntel(mines=["A1"], bridge_a="K4").is_complete()
