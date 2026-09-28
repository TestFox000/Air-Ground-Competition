import time

import pytest

from ag_core.link_protocol import FieldIntel
from ag_mission.field_fusion import IntelVoteConfig, IntelVoter


def _intel(seq, mines, obstacles=None, bridge_a="K4", bridge_b="K9", face="blue"):
    return FieldIntel(
        mines=list(mines),
        obstacles=list(obstacles or []),
        bridge_a=bridge_a,
        bridge_b=bridge_b,
        target_cells=["D2", "E2"],
        target_face=face,
        seq=seq,
    )


def test_single_frame_not_accepted_with_min_votes_two():
    v = IntelVoter(IntelVoteConfig(min_votes=2))
    v.push(_intel(1, ["C3", "D4"]))
    assert v.stable.mines == []
    assert v.is_confident is False


def test_repeated_cells_become_stable():
    v = IntelVoter(IntelVoteConfig(min_votes=2))
    v.push(_intel(1, ["C3", "D4"]))
    v.push(_intel(2, ["C3", "D4"]))
    st = v.stable
    assert st.mines == ["C3", "D4"]
    assert v.is_confident is True


def test_one_off_noise_is_filtered():
    v = IntelVoter(IntelVoteConfig(min_votes=2))
    for i in range(1, 6):
        v.push(_intel(i, ["C3"]))
    v.push(_intel(6, ["C3", "Z9"]))  # 偶发误报
    assert v.stable.mines == ["C3"]


def test_out_of_order_and_duplicate_frames_dropped():
    v = IntelVoter(IntelVoteConfig(min_votes=1))
    assert v.push(_intel(5, ["C3"])) is True
    assert v.push(_intel(5, ["D4"])) is False   # 重复 seq
    assert v.push(_intel(3, ["E5"])) is False   # 乱序
    assert v.frame_count == 1
    assert v.stable.mines == ["C3"]


def test_sliding_window_expires_old_frames():
    v = IntelVoter(IntelVoteConfig(min_votes=2, ttl_s=1.0))
    now = time.time()
    v.push(_intel(1, ["C3"]), now=now)
    v.push(_intel(2, ["C3"]), now=now)
    assert v.stable.mines == ["C3"]
    # 时间推进超过 ttl，两帧都过期
    v.push(_intel(3, ["D4"]), now=now + 5.0)
    assert v.frame_count == 1
    assert v.stable.mines == []


def test_bridge_below_min_votes_is_none():
    v = IntelVoter(IntelVoteConfig(min_votes=2))
    v.push(_intel(1, [], bridge_a="K4", bridge_b="K9", face="blue"))
    v.push(_intel(2, [], bridge_a="K5", bridge_b="K9", face="red"))
    st = v.stable
    assert st.bridge_a is None   # K4/K5 各 1 票，未达 min_votes
    assert st.bridge_b == "K9"   # 2 票
    assert st.target_face is None


def test_bridge_and_face_use_mode():
    v = IntelVoter(IntelVoteConfig(min_votes=2))
    v.push(_intel(1, [], bridge_a="K4", bridge_b="K9", face="blue"))
    v.push(_intel(2, [], bridge_a="K5", bridge_b="K9", face="red"))
    v.push(_intel(3, [], bridge_a="K4", bridge_b="K9", face="blue"))
    st = v.stable
    assert st.bridge_a == "K4"   # K4 2 票 > K5 1 票
    assert st.bridge_b == "K9"
    assert st.target_face == "blue"


def test_reset_clears_state():
    v = IntelVoter(IntelVoteConfig(min_votes=1))
    v.push(_intel(1, ["C3"]))
    v.reset()
    assert v.frame_count == 0
    assert v.stable is None
    assert v.is_confident is False


def test_max_mines_caps_output():
    cfg = IntelVoteConfig(min_votes=1, max_mines=3)
    v = IntelVoter(cfg)
    v.push(_intel(1, ["A1", "B2", "C3", "D4", "E5"]))
    assert len(v.stable.mines) == 3


def test_progressive_stream_converges():
    """无人机先逐帧揭示、侦察完成后每帧发全量：窗口滚过早期帧后应收敛到 8 颗雷。

    注意 min_ratio 的作用：窗口里若仍混着早期不完整帧，后揭示的雷占比会被拉低。
    因此现场应把 max_history / ttl_s 调到「侦察稳定后窗口内基本都是全量帧」的量级。
    """
    all_mines = ["B2", "C3", "D4", "E5", "F6", "G7", "H8", "I9"]
    v = IntelVoter(IntelVoteConfig(min_votes=2, ttl_s=100.0, max_history=12))
    seq = 0

    # 阶段一：逐帧揭示（模拟无人机边飞边看）
    for k in range(2, len(all_mines) + 1):
        seq += 1
        v.push(_intel(seq, all_mines[:k]))
    early = v.stable
    assert set(early.mines).issubset(set(all_mines))
    assert len(early.mines) < len(all_mines)   # 尚未收敛

    # 阶段二：侦察完成，持续下发全量情报
    for _ in range(12):
        seq += 1
        v.push(_intel(seq, all_mines))

    st = v.stable
    assert st.mines == all_mines
    assert st.bridge_a == "K4" and st.bridge_b == "K9"
    assert st.target_face == "blue"


if __name__ == "__main__":
    pytest.main([__file__])
