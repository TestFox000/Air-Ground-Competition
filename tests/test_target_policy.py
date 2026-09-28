import numpy as np
import pytest

from ag_core.postprocess import Detection
from ag_core.target_policy import CLASS_BLUE, CLASS_RED, FireAction, TargetPolicy


def _det(cls: int, score: float = 0.92, kpt=(320.0, 240.0, 0.9)) -> Detection:
    return Detection(
        cls=cls,
        score=score,
        x1=300.0, y1=220.0, x2=340.0, y2=260.0,
        kpts=np.array([list(kpt)], dtype=np.float32),
    )


def test_blue_is_enemy_and_can_fire():
    p = TargetPolicy(consensus_frames=3, conf_thres=0.5)
    for _ in range(2):
        assert p.update([_det(CLASS_BLUE)]).action == FireAction.HOLD
    d = p.update([_det(CLASS_BLUE)])
    assert d.action == FireAction.FIRE
    assert d.cls == CLASS_BLUE
    assert "敌方" in d.reason


def test_red_is_friend_and_never_fires():
    p = TargetPolicy(consensus_frames=2, conf_thres=0.5)
    for _ in range(5):
        assert p.update([_det(CLASS_RED)]).action == FireAction.AVOID
    assert p.shots_fired == 0


def test_low_score_holds():
    p = TargetPolicy(consensus_frames=1, conf_thres=0.9)
    assert p.update([_det(CLASS_BLUE, score=0.5)]).action == FireAction.NO_TARGET


def test_low_kpt_conf_blocks_aim():
    p = TargetPolicy(consensus_frames=1, conf_thres=0.5, kpt_conf_thres=0.8)
    d = p.update([_det(CLASS_BLUE, kpt=(320.0, 240.0, 0.2))])
    assert d.action == FireAction.HOLD
    assert "关键点" in d.reason


def test_ammo_exhausted_holds():
    p = TargetPolicy(consensus_frames=1, conf_thres=0.5, ammo_total=2)
    p.on_shot_result(hit=True, on_enemy=True)
    p.on_shot_result(hit=True, on_enemy=True)
    assert p.ammo_left == 0
    assert p.update([_det(CLASS_BLUE)]).action == FireAction.HOLD


def test_two_hits_make_one_count():
    p = TargetPolicy()
    for _ in range(2):
        p.on_shot_result(hit=True, on_enemy=True)
    assert p.enemy_count == 1
    assert p.expected_score == 5


def test_destroy_after_three_counts():
    p = TargetPolicy()
    for _ in range(6):
        p.on_shot_result(hit=True, on_enemy=True)
    assert p.enemy_count == 3
    assert p.target_destroyed is True
    assert p.expected_score == 15
    # 击毁后不再计分
    p.on_shot_result(hit=True, on_enemy=True)
    assert p.expected_score == 15


def test_friend_mis_hit_penalty():
    p = TargetPolicy()
    for _ in range(4):
        p.on_shot_result(hit=True, on_enemy=False)
    assert p.friend_count == 2
    assert p.expected_score == -10


def test_destroyed_target_blocks_fire():
    p = TargetPolicy(consensus_frames=1, conf_thres=0.5)
    for _ in range(6):
        p.on_shot_result(hit=True, on_enemy=True)
    d = p.update([_det(CLASS_BLUE)])
    assert d.action == FireAction.HOLD
    assert "失效" in d.reason


def test_reset_episode():
    p = TargetPolicy()
    p.on_shot_result(hit=True, on_enemy=True)
    p.reset_episode()
    assert p.shots_fired == 0 and p.enemy_hits == 0


def test_off_center_target_needs_gimbal_move():
    p = TargetPolicy(consensus_frames=1, conf_thres=0.5, aim_tolerance_px=10.0)
    d = p.update([_det(CLASS_BLUE, kpt=(500.0, 240.0, 0.9))])
    assert d.action == FireAction.HOLD
    assert abs(d.yaw_delta) > 5.0
