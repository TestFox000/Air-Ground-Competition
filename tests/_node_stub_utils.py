"""节点级测试的公共桩工具：参数覆盖 + tick 驱动 + 发布/订阅存取。

根 conftest.py 的 _FakeNode 只把 declare_parameter 的默认值原样返回、
get_parameter 恒回 None，且 create_timer 只收集 (period, callback) 元组
不自动调度——而被测节点（IntegrateNode / DzbrainBridgeNode）在构造期把
所有参数 declare 一遍、再经 get_parameter 逐个读回。这里用 monkeypatch
把这对方法包装成"declare 时记录默认值（可覆盖）+ 查表读回"，等价于旧
ros_stub 的 set_overrides 机制，但作用域仅限当前测试（monkeypatch 自动
还原），不再污染进程级单例。

时间驱动：tick() 先把假时钟推进 dt 再触发全部 timer 回调。两个节点的
决策逻辑都只在本拍里读一次 now()，所以第 k 拍的时钟读数 = k*dt，
周期断言（心跳 1Hz / 刷新 2s / 对齐超时）据此排拍。
"""

from __future__ import annotations

import types


def install_param_overrides(monkeypatch, cls, overrides: dict) -> None:
    """把 cls 的 declare_parameter/get_parameter 换成带覆盖表的实现。

    被测节点都是"先 declare 全部参数、再 get_parameter 读回"的写法，
    所以只需在这对方法间维护一张 captured 表；BlackBox.from_params 读
    log_dir/log_enabled/log_level 也走同一张表。
    """

    captured: dict = {}

    def _declare(self, name, value=None, descriptor=None):
        if name in overrides:
            value = overrides[name]
        captured[name] = value
        return type("P", (), {"value": value})()

    def _get(self, name):
        return type("P", (), {"value": captured[name]})()

    monkeypatch.setattr(cls, "declare_parameter", _declare)
    monkeypatch.setattr(cls, "get_parameter", _get)


def make_node(monkeypatch, cls, **overrides):
    """按覆盖参数构造一个节点（参数机制见 install_param_overrides）。"""
    install_param_overrides(monkeypatch, cls, overrides)
    return cls()


def tick(node, dt: float = 0.1) -> None:
    """推进假时钟并触发全部 timer 回调（默认一拍 0.1s，对应 10Hz 决策）。"""
    node.get_clock().advance(dt)
    for _period, cb in node.timers:
        cb()


def pub(node, topic: str):
    """取 topic 上的假发布器（.sent 全量 / .last 最新 / .clear() 清空）。"""
    return node.publishers[topic]


def published_data(node, topic: str) -> list:
    return [m.data for m in node.publishers[topic].sent]


def inject_str(node, topic: str, data) -> None:
    """向 topic 的订阅回调投递一条 std_msgs/String 风格消息（.data=data）。"""
    node.inject(topic, types.SimpleNamespace(data=data))
