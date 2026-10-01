import time

import pytest

from ag_perception.intel_accumulator import KIND, IntelAccumulator


def _acc(**kw) -> IntelAccumulator:
    return IntelAccumulator(**kw)


def test_below_min_votes_not_reported():
    acc = _acc(min_votes=3)
    acc.observe(KIND.MINE, "C3")
    acc.observe(KIND.MINE, "C3")
    assert acc.snapshot().mines == []
    assert acc.is_ready is False


def test_reaching_min_votes_reports():
    acc = _acc(min_votes=3)
    for _ in range(3):
        acc.observe(KIND.MINE, "C3")
    st = acc.snapshot()
    assert st.mines == ["C3"]
    assert acc.is_ready is True


def test_mines_sorted_and_capped():
    acc = _acc(min_votes=1, max_mines=3)
    for cell in ["E5", "A1", "C3", "D4"]:
        acc.observe(KIND.MINE, cell)
    assert acc.snapshot().mines == ["A1", "C3", "D4"]


def test_bridges_split_into_a_and_b():
    """两处桥洞按**区域**分配：行号小 → A 段、行号大 → B 段，与票数无关。

    旧实现按票数先后直接排，K9 票数高时会被错标成 bridge_a（A 段），
    车端桥洞校验（strict 模式）会判定"情报不符"而拒绝发车。
    """
    acc = _acc(min_votes=1)
    acc.observe(KIND.BRIDGE, "K4")
    acc.observe(KIND.BRIDGE, "K9")
    acc.observe(KIND.BRIDGE, "K9")  # K9 票数更高，但它是 B 段（行号大）
    st = acc.snapshot()
    assert st.bridge_a == "K4"   # A 段 = 行号小，不看票数
    assert st.bridge_b == "K9"


def test_single_bridge_only_sets_a():
    acc = _acc(min_votes=1)
    acc.observe(KIND.BRIDGE, "K4")
    st = acc.snapshot()
    assert st.bridge_a == "K4"
    assert st.bridge_b is None


def test_target_face_picks_stronger_side():
    acc = _acc(min_votes=1)
    acc.observe(KIND.TARGET_BLUE, "D2")
    acc.observe(KIND.TARGET_BLUE, "E2")
    acc.observe(KIND.TARGET_BLUE, "E2")
    acc.observe(KIND.TARGET_RED, "D2")  # 同一格只有 1 票红面
    st = acc.snapshot()
    assert st.target_face == "blue"
    assert st.target_cells == ["D2", "E2"]


def test_target_face_red_when_red_dominates():
    acc = _acc(min_votes=1)
    for _ in range(3):
        acc.observe(KIND.TARGET_RED, "D2")
        acc.observe(KIND.TARGET_RED, "E2")
    st = acc.snapshot()
    assert st.target_face == "red"
    assert st.target_cells == ["D2", "E2"]


def test_old_observations_expire():
    acc = _acc(min_votes=2, ttl_s=5.0)
    now = time.time()
    acc.observe(KIND.MINE, "C3", now=now)
    acc.observe(KIND.MINE, "C3", now=now)
    assert acc.snapshot(now=now).mines == ["C3"]
    assert acc.snapshot(now=now + 10.0).mines == []


def test_observe_many_counts():
    acc = _acc(min_votes=1)
    n = acc.observe_many(KIND.MINE, ["A1", "B2", ""])
    assert n == 2


def test_unknown_kind_raises():
    acc = _acc()
    with pytest.raises(ValueError):
        acc.observe("ufo", "A1")


def test_reset_clears_everything():
    acc = _acc(min_votes=1)
    acc.observe_many(KIND.MINE, ["A1", "B2"])
    acc.reset()
    assert acc.snapshot().mines == []
    assert acc.counts() == {}


def test_counts_reflect_votes():
    acc = _acc(min_votes=1)
    acc.observe(KIND.MINE, "C3")
    acc.observe(KIND.MINE, "C3")
    acc.observe(KIND.MINE, "D4")
    assert acc.counts()[KIND.MINE] == {"C3": 2, "D4": 1}


def test_noise_cell_drops_out_after_window():
    """偶发误检只出现一帧，min_votes 下不应进入情报。"""
    acc = _acc(min_votes=3)
    for _ in range(3):
        acc.observe_many(KIND.MINE, ["B2", "C3"])
    acc.observe(KIND.MINE, "Z9")  # 单帧误检
    assert acc.snapshot().mines == ["B2", "C3"]


if __name__ == "__main__":
    pytest.main([__file__])
