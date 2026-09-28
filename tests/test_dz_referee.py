"""实机裁判帧测试：帧格式全部照抄车端 dzjudgment 源码，逐字节核对。"""

import pytest

from ag_core.dz_referee import (
    CMD,
    HIT_NONE,
    SHOT_FRAME,
    DzFrameParser,
    DzMaterial,
    DzState,
    parse_state_payload,
    split_frames,
)


# ---------------------------------------------------------------- 帧常量
def test_shot_frame_is_byte_exact():
    """官方源码写死的 6 字节射击指令，一个字节都不能差。"""
    assert SHOT_FRAME == bytes([0xB5, 0x5B, 0x03, 0x21, 0x01, 0x25])
    assert SHOT_FRAME[:2] == b"\xb5\x5b"


def test_sof_and_cmd_values():
    assert CMD.ALL_MATERIAL == 0x02
    assert CMD.ENEMY_MATERIAL == 0x03
    assert CMD.SELF_MATERIAL == 0x04
    assert CMD.HP_AND_HIT == 0x07
    assert CMD.SHOT == 0x21


# ---------------------------------------------------------------- 收帧
def test_parse_state_frame():
    frame = DzFrameParser.state_frame(DzState(targethp=6, self_hp=10, hit=0x02, shots=4, ammo=6))
    frames, dropped = split_frames(frame)
    assert dropped == 0 and len(frames) == 1
    assert frames[0].cmd == CMD.HP_AND_HIT
    st = parse_state_payload(frames[0].payload)
    assert (st.targethp, st.self_hp, st.hit, st.shots, st.ammo) == (6, 10, 0x02, 4, 6)
    assert st.hit_dir == "front"
    assert st.under_fire is True


def test_hit_direction_mapping():
    for raw, name in ((0x01, "left"), (0x02, "front"), (0x03, "right"), (0x04, "back")):
        assert DzState(hit=raw).hit_dir == name
    assert DzState(hit=HIT_NONE).hit_dir is None
    assert DzState(hit=HIT_NONE).under_fire is False
    assert DzState(hit=0x09).hit_dir == "unknown"


def test_state_payload_too_short():
    with pytest.raises(ValueError):
        parse_state_payload(b"\x01\x02\x03")


def test_resync_after_garbage():
    """线上混入噪声后必须能重新找到帧头（车端也是这么做的）。"""
    stream = b"\x00\xff\x12" + SHOT_FRAME + b"\xb5"
    frames, dropped = split_frames(stream)
    assert dropped == 3
    assert [f.cmd for f in frames] == [CMD.SHOT]


def test_partial_frame_waits_for_more_bytes():
    p = DzFrameParser()
    frame = SHOT_FRAME
    assert p.feed(frame[:2]) == []          # 只有帧头
    assert p.feed(frame[2:4]) == []         # 长度字节到，但数据还没到齐
    out = p.feed(frame[4:])
    assert len(out) == 1 and out[0].cmd == CMD.SHOT
    assert p.dropped == 0


def test_multiple_frames_in_one_buffer():
    buf = SHOT_FRAME + DzFrameParser.state_frame(DzState(ammo=3)) + SHOT_FRAME
    frames, dropped = split_frames(buf)
    assert dropped == 0
    assert [f.cmd for f in frames] == [CMD.SHOT, CMD.HP_AND_HIT, CMD.SHOT]


def test_zero_length_frame_resyncs():
    """长度字段不可信时不能瞎读，丢掉帧头继续找，且不影响后续正常帧。"""
    p = DzFrameParser()
    assert p.feed(bytes([0xB5, 0x5B, 0x00])) == []
    assert p.dropped >= 1
    # 紧接着来一帧合法的，必须还能正确收下
    out = p.feed(SHOT_FRAME)
    assert [f.cmd for f in out] == [CMD.SHOT]


def test_encode_frame_matches_parser():
    raw = DzFrameParser.encode_frame(CMD.SELF_MATERIAL, b"\x01\x02\x03")
    assert raw[:2] == b"\xb5\x5b"
    assert raw[2] == 1 + 3                   # len = cmd + data
    frames, _ = split_frames(raw)
    assert frames[0].cmd == CMD.SELF_MATERIAL
    assert frames[0].payload == b"\x01\x02\x03"


# ---------------------------------------------------------------- 物资
def test_materials_drop_last_byte_like_car_code():
    """车端从 payload 起点只取 len-2 个字节，最后一个是校验/占位，保持一致。"""
    frame = DzFrameParser.encode_frame(CMD.ALL_MATERIAL, b"\x05\x07\x09\x0b")
    frames, _ = split_frames(frame)
    assert frames[0].materials == [5, 7, 9]


def test_material_snapshot_update():
    m = DzMaterial()
    frames, _ = split_frames(
        DzFrameParser.encode_frame(CMD.ALL_MATERIAL, b"\x01\x02\x00")
        + DzFrameParser.encode_frame(CMD.ENEMY_MATERIAL, b"\x03\x00")
        + DzFrameParser.encode_frame(CMD.HP_AND_HIT, b"\x01\x02\x10\x03\x04")
    )
    assert m.update(frames[0]) is True
    assert m.update(frames[1]) is True
    assert m.update(frames[2]) is False      # 0x07 不是物资帧
    assert m.all_items == [1, 2]
    assert m.enemy_items == [3]
    assert m.self_items == []
