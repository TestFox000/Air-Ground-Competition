"""任务规划节点（车端）：情报 -> 排雷/过桥/打击顺序 -> 逐段路点。

数据流向：
    /link/field_intel (FieldIntelMsg) ──▶ IntelVoter ──▶ MissionPlanner
                                                            │
    /mission/task_done (String) ────────────────────────────┘
                                                            ▼
    /mission/plan (String, JSON 全量计划)
    /mission/waypoints (String, JSON 当前任务 + 路径格号)
    /mission/status (String, JSON 预估分/耗时/进度)

底盘只需要订阅 /mission/waypoints 并逐个走格号即可。
"""

from __future__ import annotations

import json
from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ag_core.blackbox import EV, BlackBox
from ag_core.grid_map import CellID, FieldGrid
from ag_core.link_protocol import FieldIntel
from ag_core.mission_planner import MissionPlanner, Task, TaskType
from ag_interfaces.msg import FieldIntelMsg

from .field_fusion import IntelVoteConfig, IntelVoter

PLAN_TOPIC = "/mission/plan"
WAYPOINT_TOPIC = "/mission/waypoints"
STATUS_TOPIC = "/mission/status"


class MissionNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_node")

        # ---- 场地 ----
        # 14 列（A~N，沿 5 m 边）x 11 行（1~11，沿 4 m 边），与车端 path_executor 常量一致
        self.declare_parameter("cols", 14)
        self.declare_parameter("rows", 11)
        self.declare_parameter("cell_size", 0.35)
        self.declare_parameter("start_cell", "N10")
        self.declare_parameter("goal_cell", "N11")

        # ---- 规划 ----
        self.declare_parameter("speed_mps", 0.55)
        self.declare_parameter("budget_s", 180.0)
        self.declare_parameter("goal_margin_s", 12.0)

        # ---- 融合 ----
        self.declare_parameter("min_votes", 2)
        self.declare_parameter("vote_window", 24)
        self.declare_parameter("vote_ttl_s", 20.0)

        # ---- 行为 ----
        self.declare_parameter("replan_period_s", 1.0)
        self.declare_parameter("intel_topic", "/link/field_intel")
        self.declare_parameter("task_done_topic", "/mission/task_done")
        # ---- 黑匣子 ----
        self.declare_parameter("log_enabled", True)
        self.declare_parameter("log_dir", "")
        self.declare_parameter("log_level", "info")

        p = lambda k: self.get_parameter(k).value  # noqa: E731

        self.grid = FieldGrid(
            cols=int(p("cols")), rows=int(p("rows")), cell_size=float(p("cell_size"))
        )
        self.start = CellID.parse(p("start_cell"))
        self.goal = CellID.parse(p("goal_cell"))

        self.planner = MissionPlanner(
            grid=self.grid,
            start=self.start,
            goal=self.goal,
            speed_mps=float(p("speed_mps")),
            budget_s=float(p("budget_s")),
            goal_margin_s=float(p("goal_margin_s")),
        )
        self.voter = IntelVoter(
            IntelVoteConfig(
                min_votes=int(p("min_votes")),
                max_history=int(p("vote_window")),
                ttl_s=float(p("vote_ttl_s")),
            )
        )

        self._plan: List[Task] = []
        self._index = 0
        self._intel_stamp = 0.0

        self.pub_plan = self.create_publisher(String, PLAN_TOPIC, 10)
        self.pub_wp = self.create_publisher(String, WAYPOINT_TOPIC, 10)
        self.pub_status = self.create_publisher(String, STATUS_TOPIC, 10)

        self.sub_intel = self.create_subscription(
            FieldIntelMsg, p("intel_topic"), self.cb_intel, 10
        )
        self.sub_done = self.create_subscription(
            String, p("task_done_topic"), self.cb_task_done, 10
        )
        self.timer = self.create_timer(float(p("replan_period_s")), self.cb_timer)

        self.bb = BlackBox.from_params(self)
        # 自研底盘这条路没有 car_node，由 mission_node 自己标记"一局开始"，
        # 复盘工具才能正确切局（只统计最后一局，不算上上一轮的残留）。
        self.bb.log(EV.SESSION_BEGIN, start_cell=self.start.name, goal_cell=self.goal.name,
                    cols=self.grid.cols, rows=self.grid.rows,
                    cell_size=self.grid.cell_size, speed_mps=self.planner.speed_mps,
                    budget_s=self.planner.budget_s, min_votes=self.voter.cfg.min_votes,
                    path="mission_node（自研/仿真底盘）")
        self._last_stable: Optional[tuple] = None
        self._last_wp_log: Optional[tuple] = None
        self._last_plan_log: Optional[tuple] = None

        self.get_logger().info(
            f"mission_node 就绪: {self.start.name} -> {self.goal.name}, "
            f"预算 {self.planner.budget_s:.0f}s"
        )

    # ---------------- 情报 ----------------
    def cb_intel(self, msg: FieldIntelMsg) -> None:
        intel = FieldIntel(
            mines=list(msg.mines),
            obstacles=list(msg.obstacles),
            bridge_a=msg.bridge_a or None,
            bridge_b=msg.bridge_b or None,
            target_cells=list(msg.target_cells),
            target_face=msg.target_face or None,
            seq=int(msg.seq),
        )
        ok = self.voter.push(intel)
        self.bb.log(
            EV.INTEL_RECV, ok=ok, why="" if ok else "乱序或重复（投票窗口已丢弃）",
            seq=int(intel.seq), mines=list(intel.mines),
            obstacles=list(intel.obstacles),
            bridge_a=intel.bridge_a, bridge_b=intel.bridge_b,
            target_cells=list(intel.target_cells), target_face=intel.target_face,
        )
        if not ok:
            return  # 乱序 / 重复帧
        if self.voter.is_confident:
            self._replan()

    def _replan(self) -> None:
        st = self.voter.stable
        if st is None:
            return
        # 与 car_node 同款：内容没变就不重复写（情报收敛后每帧都会走到这里）
        snap = (tuple(st.mines), tuple(st.obstacles), st.bridge_a, st.bridge_b,
                tuple(st.target_cells), st.target_face)
        if snap != self._last_stable:
            self._last_stable = snap
            self.bb.log(EV.INTEL_STABLE, mines=list(st.mines), obstacles=list(st.obstacles),
                        bridge_a=st.bridge_a, bridge_b=st.bridge_b,
                        target_cells=list(st.target_cells), target_face=st.target_face,
                        frames=int(self.voter.frame_count))
        self.planner.set_intel(
            mines=st.mines,
            obstacles=st.obstacles,
            bridge_a=st.bridge_a,
            bridge_b=st.bridge_b,
            target_cell=st.target_cells[0] if st.target_cells else None,
        )
        done = self._plan[: self._index]
        # 回到终点即本局结束：此后到达的新情报只更新地图，不再往计划里塞任务，
        # 否则会出现「已经回终点了又跑出去排雷」这种顺序倒挂。
        if any(t.type == TaskType.GOAL for t in done):
            self._plan = done
            self._log_plan()
            self._publish_plan()
            self._publish_waypoint()
            return

        new_plan = self.planner.full_plan()
        # 已执行的任务保持不变，只在剩余段上重排（按 类型+格号 判重）
        done_keys = {(t.type, t.cell) for t in done}
        self._plan = done + [t for t in new_plan if (t.type, t.cell) not in done_keys]
        self._log_plan()
        self._publish_plan()
        self._publish_waypoint()

    def _log_plan(self) -> None:
        """计划变了才记一条（`_replan` 每次情报变化都会重算）。"""
        stops = [t.cell.name for t in self._plan
                 if t.type == TaskType.TARGET and t.cell is not None]
        key = (len(self._plan), tuple(stops))
        if key == self._last_plan_log:
            return
        self._last_plan_log = key
        self.bb.log(EV.PLAN_BUILT, plan="mission_node（自研/仿真底盘）",
                    tasks=len(self._plan), shoot_stops=stops,
                    eta_s=round(self.planner.estimate_time(self._plan), 1),
                    score=self.planner.estimate_score(self._plan),
                    cells=[f"{t.type.value}:{t.cell.name if t.cell else '-'}"
                           for t in self._plan])

    # ---------------- 执行推进 ----------------
    def cb_task_done(self, msg: String) -> None:
        token = (msg.data or "").strip().upper()
        if not token:
            # 空消息**不能**推进任务：契约里 chassis_node 只发格号/任务类型，
            # 空消息没有合法来源；任何一个写错的发布者都能让任务整段跳掉。
            self.get_logger().warning("收到空的 task_done，已忽略（应发布格号或任务类型）")
            return
        cur = self.current_task
        if cur is not None and (token == (cur.cell.name if cur.cell else "") or token == cur.type.value.upper()):
            self._advance()
        else:
            self.get_logger().debug(f"忽略无关完成信号: {token}")

    def _advance(self) -> None:
        if self._index < len(self._plan):
            self._index += 1
        self._publish_waypoint()

    def cb_timer(self) -> None:
        self._publish_status()

    # ---------------- 查询 ----------------
    @property
    def current_task(self) -> Optional[Task]:
        if self._index < len(self._plan):
            return self._plan[self._index]
        return None

    # ---------------- 发布 ----------------
    def _publish_plan(self) -> None:
        out = String()
        out.data = json.dumps(
            {
                "tasks": [t.to_dict() for t in self._plan],
                "score": self.planner.estimate_score(self._plan),
                "time_s": round(self.planner.estimate_time(self._plan), 2),
            },
            ensure_ascii=False,
        )
        self.pub_plan.publish(out)

    def _publish_waypoint(self) -> None:
        t = self.current_task
        # 只在"当前任务变了"时记一条：`_replan` 每次收敛都会重发路点，
        # 不去重的话同一段任务会被记几十遍，时间轴上就看不出真实的推进节奏。
        key = (self._index, t.type.value if t else "done", t.cell.name if t and t.cell else "")
        if key != self._last_wp_log:
            self._last_wp_log = key
            self.bb.log(
                EV.MISSION_DONE if t is None else EV.MISSION_TASK,
                index=self._index, total=len(self._plan),
                type=t.type.value if t else "done",
                cell=(t.cell.name if t and t.cell else None),
                score=(t.score if t else 0),
                eta_s=round(t.total_s, 1) if t else 0.0,
                path=[c.name for c in (t.path if t else [])],
                is_target=bool(t is not None and t.type == TaskType.TARGET),
                score_total=self.planner.estimate_score(self._plan),
            )
        out = String()
        if t is None:
            out.data = json.dumps({"done": True, "index": self._index}, ensure_ascii=False)
        else:
            out.data = json.dumps(
                {
                    "done": False,
                    "index": self._index,
                    "type": t.type.value,
                    "cell": t.cell.name if t.cell else None,
                    "path": [c.name for c in t.path],
                    "score": t.score,
                    "eta_s": round(t.total_s, 2),
                },
                ensure_ascii=False,
            )
        self.pub_wp.publish(out)

    def _publish_status(self) -> None:
        remaining = self._plan[self._index :]
        out = String()
        out.data = json.dumps(
            {
                "index": self._index,
                "total": len(self._plan),
                "frames": self.voter.frame_count,
                "confident": self.voter.is_confident,
                "remaining_score": sum(t.score for t in remaining),
                "remaining_time_s": round(sum(t.total_s for t in remaining), 2),
            },
            ensure_ascii=False,
        )
        self.pub_status.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    finally:
        node.bb.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
