"""开火闸门测试。

背景（决定比赛得分的一条）：官方 `dzrknn_dataprocess` 的开火判定是
`is_target_cls = (data[2]==1) || (data[2]==2)` —— **蓝红都打**，
而车端那道"停车才允许开火"的 `/move_base/stop_signal` 是死代码（无订阅者）。
所以"绝不误击友方"这件事只能由我们这层闸门保证。
"""

import pytest

import ros_stub  # noqa: F401  必须先安装替身
from std_msgs.msg import Int32MultiArray  # noqa: E402

from ag_car.aim_guard_node import AimGuardNode  # noqa: E402

IN = "/offset_center"
OUT = "/offset_center_guarded"

BLUE = [120, -30, 1, 80, 82, 118, -28, 95]     # cls=1 蓝靶（敌方）
RED = [-273, 41, 2, 94, 99, -270, 44, 88]      # cls=2 红靶（友方）—— 实测拍到的就是这个
NONE = [0, 0, 0, 0, 0, 0, 0, 0]


def _node(**kw):
    ros_stub.set_overrides(**kw)
    try:
        return AimGuardNode()
    finally:
        ros_stub.clear_overrides()


def _feed(node, data):
    m = Int32MultiArray()
    m.data = list(data)
    node.deliver(IN, m)


def _last(node):
    return list(node.topic(OUT).published[-1].data)


# ---------------------------------------------------------------- 默认行为
def test_blue_target_passes_through():
    n = _node()
    _feed(n, BLUE)
    assert _last(n) == BLUE
    assert n.passed == 1 and n.blocked == 0


def test_red_target_is_blocked_and_blanked():
    """最重要的一条：红面（友方）必须被拦下，且输出"无目标"帧。"""
    n = _node()
    _feed(n, RED)
    assert _last(n) == [0] * 8
    assert n.blocked == 1 and n.passed == 0


def test_red_block_is_logged():
    n = _node(publish_blocked_log_period_s=0.0)   # 不限流，立刻出日志
    _feed(n, RED)
    assert any("红面" in r[1] for r in n.logger.records)


def test_no_target_passes_through_unchanged():
    n = _node()
    _feed(n, NONE)
    assert _last(n) == NONE
    assert n.blank == 1


def test_unknown_class_is_blocked_by_default():
    """类别 3+（将来加物资类别时）默认不放行 —— 宁可漏打，不可错打。"""
    n = _node()
    _feed(n, [10, 10, 3, 50, 50, 10, 10, 90])
    assert _last(n) == [0] * 8
    assert n.blocked == 1


# ---------------------------------------------------------------- 总闸
def test_master_switch_blocks_everything():
    n = _node(enable_fire=False)
    _feed(n, BLUE)
    _feed(n, RED)
    _feed(n, [10, 10, 3, 50, 50, 10, 10, 90])
    assert _last(n) == [0] * 8
    assert n.passed == 0 and n.blocked == 3
    assert any("总闸关闭" in r[1] for r in n.logger.records)


def test_master_switch_still_forwards_no_target_frames():
    """总闸关掉只是不锁定目标，但"没有目标"的帧照旧透传，免得官方节点卡在跟踪态。"""
    n = _node(enable_fire=False)
    _feed(n, NONE)
    assert _last(n) == NONE
    assert n.blank == 1


# ---------------------------------------------------------------- 白名单
def test_empty_whitelist_allows_all_classes():
    """allow_classes 留空 = 全放行（危险，仅供对照/调试）。"""
    n = _node(allow_classes="")
    _feed(n, BLUE)
    _feed(n, RED)
    assert n.passed == 2 and n.blocked == 0
    assert _last(n) == RED


def test_multi_class_whitelist():
    n = _node(allow_classes="1,3")
    _feed(n, BLUE)
    _feed(n, [1, 2, 3, 4, 5, 6, 7, 8])
    _feed(n, RED)
    assert n.passed == 2 and n.blocked == 1


def test_whitelist_with_spaces():
    n = _node(allow_classes=" 1 , 2 ")
    _feed(n, RED)
    assert n.passed == 1


# ---------------------------------------------------------------- 健壮性
@pytest.mark.parametrize("bad", [[], [0, 0, 1], [1] * 7])
def test_short_message_is_treated_as_no_target(bad):
    n = _node()
    _feed(n, bad)
    assert _last(n) == [0] * 8
    assert n.passed == 0


def test_long_message_passes_through_extra_fields():
    """官方以后在末尾追加字段（比如关键点置信度）时，我们把整条原样转发，不截断。"""
    longer = BLUE + [42, 43]
    n = _node()
    _feed(n, longer)
    assert _last(n) == longer


def test_status_counters_accumulate():
    n = _node(publish_blocked_log_period_s=0.0)
    for _ in range(3):
        _feed(n, BLUE)
    for _ in range(2):
        _feed(n, RED)
    _feed(n, NONE)
    assert (n.passed, n.blocked, n.blank) == (3, 2, 1)
    assert any("累计拦截 2 次" in r[1] for r in n.logger.records)
