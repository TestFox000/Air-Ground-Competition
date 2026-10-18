#!/usr/bin/env python3

import threading

import rclpy
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient
from rclpy.node import Node


class DztrackPathBridge(Node):

    def __init__(self) -> None:
        super().__init__('dztrack_path_bridge')

        self._action_client = ActionClient(self, FollowPath, 'follow_path')

        self._plan_sub = self.create_subscription(
            Path,
            'allplan',
            self.plan_callback,
            10
        )

        self._lock = threading.Lock()
        self._current_goal_handle = None
        self._has_active_goal = False

        self._last_allplan: Path | None = None
        self._last_sent_signature: str | None = None

        self.get_logger().info('dztrack_path_bridge 已启动，等待 controller_server...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('等待 /follow_path 超时，后续收到路径时会继续尝试发送')
        else:
            self.get_logger().info('已连接到 /follow_path')

    def _path_signature(self, path: Path) -> str:
        if not path.poses:
            return "empty"
        n = len(path.poses)
        first = path.poses[0].pose.position
        last = path.poses[-1].pose.position
        return f"{n}_{first.x:.3f}_{first.y:.3f}_{last.x:.3f}_{last.y:.3f}"

    def plan_callback(self, msg: Path) -> None:
        if not msg.poses:
            self.get_logger().warn('收到空的 allplan，忽略')
            return

        if not self._action_client.server_is_ready():
            self.get_logger().warn('controller_server(/follow_path) 未就绪，暂不发送')
            return

        sig = self._path_signature(msg)

        with self._lock:
            if self._has_active_goal:
                return
            if self._last_sent_signature == sig:
                return

        self._last_allplan = msg
        self._send_path(msg, sig)

    def _send_path(self, path: Path, signature: str) -> None:
        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = 'FollowPath'

        self.get_logger().info(f'发送 allplan 到 controller_server: poses={len(path.poses)}')
        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

        with self._lock:
            self._last_sent_signature = signature

    def goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('FollowPath goal 被拒绝')
            return

        self.get_logger().info('FollowPath goal 已接受')
        with self._lock:
            self._current_goal_handle = goal_handle
            self._has_active_goal = True

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg) -> None:
        # 不处理反馈，仅保持接口
        return

    def result_callback(self, future) -> None:
        with self._lock:
            self._current_goal_handle = None
            self._has_active_goal = False

        try:
            _ = future.result().result
            self.get_logger().info('FollowPath 结束，已到达目标点')
        except Exception as e:
            self.get_logger().warn(f'FollowPath result 回调异常: {e}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DztrackPathBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

