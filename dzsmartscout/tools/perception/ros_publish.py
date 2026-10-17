# -*- coding: utf-8 -*-
r"""可选：把报文真正发到 DDS（**只能在无人机上跑**，本机没有 rclpy）。

为什么不放在 ``pipeline`` 里默认执行：本工具链必须在**本机 Windows + numpy/Pillow**
下能跑测试，而 ``rclpy`` 只在无人机的 ROS 2 Humble 里存在。所以这里做成懒加载，
只有显式调用才 import rclpy。

## 唯一不能配错的东西：QoS

车侧（``dzrobot_v2/src/dzbrain/dzbrain/io/contract.py:40-41,94-96``）对
``/dzlink/drone_to_car`` 用的是 **depth 10 + BEST_EFFORT**。跨机 ROS 2 里
"可靠 vs 尽力" 不匹配会**静默收不到**（RSM 规则下 RELIABLE pub 不会匹配
BEST_EFFORT sub）。所以这里把 QoS 写死成与车侧一致，并提供一个 ``listen_acks``
用来拿"车确实收到了"的硬证据（车侧对带 ``seq`` 的报文会回 ``ack``）。
"""

from __future__ import annotations

import time
from typing import Iterable

from .protocol import CAR_TO_DRONE_TOPIC, DRONE_TO_CAR_TOPIC, encode

__all__ = ["ros_available", "publish_messages", "listen_acks"]


def ros_available() -> bool:
    try:
        import rclpy                                          # noqa: F401
    except Exception:                                          # noqa: BLE001
        return False
    return True


def _qos(depth: int = 10, reliability: str = "best_effort"):
    from rclpy.qos import (QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy)
    return QoSProfile(
        depth=int(depth),
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=(QoSReliabilityPolicy.BEST_EFFORT
                     if reliability == "best_effort" else QoSReliabilityPolicy.RELIABLE),
    )


def publish_messages(messages: Iterable[dict], *, topic: str = DRONE_TO_CAR_TOPIC,
                     depth: int = 10, reliability: str = "best_effort",
                     spin_s: float = 0.3, node_name: str = "dz_perception_tx") -> dict:
    """发布一组报文，返回 ``{"sent": n, "subscribers": m, "encoded": [...]}``。

    ``subscribers`` 是发送瞬间的订阅者匹配数 —— **0 就说明没配上**（先查 QoS 与
    ``ROS_DOMAIN_ID``，别怀疑报文）。
    """
    try:
        import rclpy
        from std_msgs.msg import String
    except Exception as exc:                                   # noqa: BLE001
        raise RuntimeError(
            "本机没有 rclpy：本函数只能在无人机（ROS 2 Humble）上运行；"
            f"原错误：{type(exc).__name__}: {exc}") from exc

    bodies = [encode(m) for m in messages]
    rclpy.init()
    node = rclpy.create_node(node_name)
    pub = node.create_publisher(String, topic, _qos(depth, reliability))
    sent = 0
    subs = 0
    try:
        deadline = time.monotonic() + max(0.0, spin_s)
        while time.monotonic() < deadline:                     # 等发现
            rclpy.spin_once(node, timeout_sec=0.05)
            subs = pub.get_subscription_count()
        for body in bodies:
            msg = String()
            msg.data = body
            pub.publish(msg)
            sent += 1
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.02)
        subs = pub.get_subscription_count()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return {"sent": sent, "subscribers": int(subs), "topic": topic,
            "qos": {"depth": depth, "reliability": reliability}, "encoded": bodies}


def listen_acks(duration_s: float = 5.0, *,
                topic: str = CAR_TO_DRONE_TOPIC, depth: int = 10,
                reliability: str = "best_effort",
                node_name: str = "dz_perception_rx") -> list[str]:
    """听车侧回传（``ack`` / ``car_state``），默认 5 s。返回收到的原始 JSON 串。

    这是"车确实收到了"的唯一硬证据：收到 ``{"type":"ack","of":<我们发的 seq>}``
    就说明该条情报已被车侧消费。
    """
    try:
        import rclpy
        from std_msgs.msg import String
    except Exception as exc:                                   # noqa: BLE001
        raise RuntimeError(
            "本机没有 rclpy：本函数只能在无人机（ROS 2 Humble）上运行；"
            f"原错误：{type(exc).__name__}: {exc}") from exc

    got: list[str] = []
    rclpy.init()
    node = rclpy.create_node(node_name)
    node.create_subscription(String, topic, lambda m: got.append(m.data),
                             _qos(depth, reliability))
    try:
        end = time.monotonic() + max(0.0, duration_s)
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return got
