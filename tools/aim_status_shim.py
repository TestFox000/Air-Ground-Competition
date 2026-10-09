#!/usr/bin/env python3
"""自瞄状态类型兼容转发：/aim/status (Int32 -> UInt8)。

背景（2026-10-06 实车定位）：
    dzrknn_dataprocess 以 std_msgs/Int32 发布 /aim/status，
    但网页端 cam_viewer 以 std_msgs/UInt8 订阅同名话题。
    ROS 2 按【类型】匹配端点 → 网页端一个包都收不到，
    网页"云台状态"永远显示"空闲"，现场会误判成"云台没在扫描"。

本节点做的事：
    订阅 Int32 的 /aim/status → 按【同名话题】以 UInt8 再发一次。
    DDS 按类型匹配，因此：
      · path_executor（Int32 订阅）继续直接消费云台原始消息，链路不受影响；
      · cam_viewer（UInt8 订阅）匹配本节点的消息，网页状态恢复显示。

★ 为什么要用两个 context（重要，别改回单节点）：
    rcl 禁止【同一个 context】里同一话题名挂两种类型，实测报错：
      create_subscription() called for existing topic name rt/aim/status
      with incompatible type std_msgs::msg::dds_::Int32_
    即使拆成两个 Node 也一样（registry 是 context 级、不是 node 级）。
    因此这里用两个独立 rclpy.Context（各自一个节点 + 一个 executor 线程），
    状态值通过带锁的共享变量传递。

状态取值：0 = IDLE 空闲 / 1 = SCAN 扫描 / 2 = TRACK 锁定

用法：
    python3 -u tools/aim_status_shim.py      # 已由 start_all.sh 自动拉起
"""

import threading
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Int32, UInt8

TOPIC = '/aim/status'
POLL_SEC = 0.1      # 状态变化检测周期
KEEPALIVE_SEC = 1.0  # 状态不变时的保活周期（后启动的网页端也能读到）


class _SharedState:
    """带锁的状态槽：订阅端写、发布端读。"""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._value = None

    def set(self, value: int) -> None:
        with self._lock:
            self._value = value

    def get(self):
        with self._lock:
            return self._value


class _SourceNode(Node):
    """context A：订阅云台发布的 Int32 原始状态。"""

    def __init__(self, context: Context, shared: _SharedState) -> None:
        super().__init__('aim_status_shim_in', context=context)
        self._shared = shared
        self.create_subscription(Int32, TOPIC, self._on_status, 10)
        self.get_logger().info(f'订阅 {TOPIC} (std_msgs/Int32) 已就绪')

    def _on_status(self, msg: Int32) -> None:
        raw = msg.data
        if raw < 0 or raw > 255:
            # 状态机只可能取 0/1/2；越界说明上游语义变了，钳位并告警而不是崩溃
            self.get_logger().warning(f'收到越界状态 {raw}，已钳位到 UInt8 范围')
        self._shared.set(max(0, min(255, raw)))


class _SinkNode(Node):
    """context B：以 UInt8 在同名话题回发，供网页端读取。"""

    def __init__(self, context: Context, shared: _SharedState) -> None:
        super().__init__('aim_status_shim_out', context=context)
        self._shared = shared
        self._last = None
        self._last_pub = 0.0
        self._pub = self.create_publisher(UInt8, TOPIC, 10)
        self.create_timer(POLL_SEC, self._tick)
        self.get_logger().info(f'回发 {TOPIC} (std_msgs/UInt8) 已就绪（供网页端读取）')

    def _tick(self) -> None:
        value = self._shared.get()
        if value is None:
            return
        now = time.monotonic()
        changed = value != self._last
        if changed or now - self._last_pub >= KEEPALIVE_SEC:
            msg = UInt8()
            msg.data = value
            self._pub.publish(msg)
            self._last = value
            self._last_pub = now


def main() -> None:
    shared = _SharedState()
    ctx_in, ctx_out = Context(), Context()
    ctx_in.init()
    ctx_out.init()

    source = _SourceNode(ctx_in, shared)
    sink = _SinkNode(ctx_out, shared)

    ex_in = SingleThreadedExecutor(context=ctx_in)
    ex_out = SingleThreadedExecutor(context=ctx_out)
    ex_in.add_node(source)
    ex_out.add_node(sink)

    threads = [
        threading.Thread(target=ex_in.spin, name='shim-in', daemon=True),
        threading.Thread(target=ex_out.spin, name='shim-out', daemon=True),
    ]
    for t in threads:
        t.start()

    try:
        while all(t.is_alive() for t in threads):
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        ex_in.shutdown()
        ex_out.shutdown()
        for t in threads:
            t.join(timeout=2.0)
        source.destroy_node()
        sink.destroy_node()
        ctx_in.shutdown()
        ctx_out.shutdown()


if __name__ == '__main__':
    main()
