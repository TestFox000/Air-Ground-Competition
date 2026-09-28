"""底盘执行节点：/mission/waypoints -> 行进 -> /mission/task_done。

闭环里的最后一环：mission_node 只管「下一步去哪」，走的事交给本节点，
走完回报格号，mission_node 再推下一段。

    /mission/waypoints (String, JSON) ──▶ chassis_node
                                              │ 行进
    /mission/task_done (String) ──────────────┘
    /chassis/state     (String, JSON)  当前状态，供地面站观察

实机对接：实现 chassis_link.ChassisLink._send_path / _poll_arrived，
          并把 simulate 置 false。在此之前用 simulate:=true 跑通逻辑。
"""

from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from ag_core.blackbox import BlackBox

from .chassis_link import ChassisLink, DriveTask
from .nav2_bridge import FrameAlign, cell_to_map_pose

WAYPOINT_TOPIC = "/mission/waypoints"
TASK_DONE_TOPIC = "/mission/task_done"
STATE_TOPIC = "/chassis/state"


class ChassisNode(Node):
    def __init__(self) -> None:
        super().__init__("chassis_node")

        self.declare_parameter("simulate", True)
        self.declare_parameter("time_scale", 1.0)     # >1 表示快进，用于仿真
        self.declare_parameter("tick_period_s", 0.1)
        self.declare_parameter("waypoints_topic", WAYPOINT_TOPIC)
        self.declare_parameter("task_done_topic", TASK_DONE_TOPIC)
        self.declare_parameter("state_topic", STATE_TOPIC)
        self.declare_parameter("min_eta_s", 0.5)      # 仿真下给每段一个最短耗时

        # ---- 导航对接 ----
        self.declare_parameter("mode", "direct")      # direct（自驱/仿真）| nav2（交给 Nav2）
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("origin_x", 0.0)       # 栅格原点在 map 中的位姿
        self.declare_parameter("origin_y", 0.0)
        self.declare_parameter("yaw_deg", 0.0)
        self.declare_parameter("cell_size", 0.35)
        self.declare_parameter("scale", 1.0)
        # ---- 黑匣子 ----
        self.declare_parameter("log_enabled", True)
        self.declare_parameter("log_dir", "")
        self.declare_parameter("log_level", "info")

        p = lambda k: self.get_parameter(k).value  # noqa: E731

        self.link = ChassisLink(time_scale=float(p("time_scale")), simulate=bool(p("simulate")))
        self.tick_period = float(p("tick_period_s"))
        self.min_eta_s = float(p("min_eta_s"))
        self.mode = str(p("mode"))
        self.frame_id = str(p("frame_id"))
        self.align = FrameAlign(
            origin_x=float(p("origin_x")),
            origin_y=float(p("origin_y")),
            yaw_rad=math.radians(float(p("yaw_deg"))),
            cell_size=float(p("cell_size")),
            scale=float(p("scale")),
        )

        self.pub_done = self.create_publisher(String, p("task_done_topic"), 10)
        self.pub_state = self.create_publisher(String, p("state_topic"), 10)
        self.pub_goal = self.create_publisher(PoseStamped, p("goal_topic"), 10)
        self.create_subscription(String, p("waypoints_topic"), self.cb_waypoint, 10)
        self.create_timer(self.tick_period, self.cb_tick)

        self.bb = BlackBox.from_params(self)

        self.get_logger().info(
            f"chassis_node 就绪: simulate={self.link.simulate} time_scale={self.link.time_scale}"
        )

    # ---------------- 路点 ----------------
    def cb_waypoint(self, msg: String) -> None:
        try:
            wp = json.loads(msg.data or "{}")
        except json.JSONDecodeError:
            self.get_logger().warning("路点不是合法 JSON，已忽略")
            return

        if wp.get("done"):
            self.link.abort("mission done")
            self._publish_state()
            return
        cell = wp.get("cell") or wp.get("type") or ""
        if not cell:
            return
        task = DriveTask(
            type=wp.get("type", ""),
            cell=str(cell),
            path=list(wp.get("path") or []),
            eta_s=max(self.min_eta_s, float(wp.get("eta_s", 0.0) or 0.0)),
            index=int(wp.get("index", 0)),
        )
        if not self._start(task):
            return  # 正在走，忽略重复路点
        if self.mode == "nav2" and task.cell:
            self._publish_goal(task.cell)
        self._publish_state()

    def _publish_goal(self, cell: str) -> None:
        """把格号换算成 map 坐标系目标点发给 Nav2。"""
        try:
            x, y, yaw = cell_to_map_pose(cell, self.align)
        except ValueError as e:
            self.get_logger().warning(f"格号无法换算为目标点: {e}")
            return
        goal = PoseStamped()
        goal.header.frame_id = self.frame_id
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub_goal.publish(goal)

    def _start(self, task: DriveTask) -> bool:
        """下发行程；底盘协议未对接时回退为计时推进，保证链路不卡死。"""
        try:
            return self.link.start(task)
        except NotImplementedError as e:
            self.get_logger().warning(f"{e}；回退为计时推进（simulate）")
            self.link.simulate = True
            return self.link.start(task)

    # ---------------- 推进 ----------------
    def cb_tick(self) -> None:
        try:
            arrived = self.link.tick(self.tick_period)
        except NotImplementedError as e:
            # 实机协议未对接：回退为计时推进，保证链路不卡死
            self.get_logger().warning(f"{e}；本次回退为计时推进")
            self.link.simulate = True
            arrived = self.link.tick(self.tick_period)
        if arrived:
            out = String()
            out.data = arrived
            self.pub_done.publish(out)
        self._publish_state()

    def _publish_state(self) -> None:
        s = self.link.state
        out = String()
        out.data = json.dumps(
            {
                "status": s.status,
                "cell": s.cell,
                "progress": round(s.progress, 3),
                "elapsed_s": round(s.elapsed_s, 2),
                "note": s.note,
            },
            ensure_ascii=False,
        )
        self.pub_state.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ChassisNode()
    try:
        rclpy.spin(node)
    finally:
        node.bb.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
