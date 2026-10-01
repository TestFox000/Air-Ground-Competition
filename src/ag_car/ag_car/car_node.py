"""车端对接节点：空中情报 -> 任务排序 -> 车端 `/nav/waypoint` 路线。

跑在渡众 SmartMaster 上时，这个节点**替代** mission_node + chassis_node：

    机载端 FieldIntelMsg
        └─▶ car_node（本节点）
               ├─ IntelVoter      多帧投票，抗链路误码
               ├─ MissionPlanner  排雷/桥洞/打击的任务顺序
               ├─ RouteBuilder    格号序列 -> 路线字符串
               └─▶ /nav/waypoint  "reset:N10,y" 然后 "G8 D10* A2 N11,y"
                                       │
                       渡众既有 path_executor（车端自带，不要重写）
                                       ├─ 纯跟踪控制 -> dzactuator -> cmd_vel/pursuitAngle
                                       ├─ 到 `*` 格停靠，等 /shoot_count 打满再走
                                       └─ dzrknn_dataprocess 负责瞄准与开火

发布：
    /nav/waypoint (String)  —— reset 命令 + 路线命令（本节点唯一的"动作"输出）
    /car/route_status (String) —— JSON，含当前路线/已确认情报/打击计数，便于现场观察

订阅：
    /link/field_intel (FieldIntelMsg)
    /shoot_count (Int32) —— dzrknn_dataprocess 上报的累计发射数

安全约束：
    - 格号先本地校验，非法格号**不发任何命令**（车端遇到非法 token 会整条丢弃，
      如果还先发了 reset，车就会"知道自己在起点但没路线"，白跑一局）
    - reset 只在真正要发第一条路线前发一次；重新规划不会重复 reset
    - 已经发过终点就不再追加新任务（否则会出现"回到终点又跑出去排雷"）
"""

from __future__ import annotations

import json
from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

from ag_core.blackbox import EV, REASON, BlackBox
from ag_core.bridge_zone import BridgeZones, check_bridge_pair
from ag_core.grid_map import CellID, FieldGrid
from ag_core.link_protocol import FieldIntel
from ag_core.mission_planner import MissionPlanner, Task, TaskType
from ag_mission.field_fusion import IntelVoteConfig, IntelVoter
from ag_interfaces.msg import FieldIntelMsg

from .route_builder import CarGrid, RouteError, build_route, reset_command

ROUTE_TOPIC = "/nav/waypoint"
STATUS_TOPIC = "/car/route_status"


class CarNode(Node):
    def __init__(self) -> None:
        super().__init__("car_node")

        # ---- 场地 ----
        self.declare_parameter("cols", 14)
        self.declare_parameter("rows", 11)
        self.declare_parameter("cell_size", 0.35)
        self.declare_parameter("start_cell", "N10")
        self.declare_parameter("goal_cell", "N11")
        self.declare_parameter("start_heading", "y")   # 车被摆放时的朝向
        self.declare_parameter("final_heading", "y")   # 终点朝向，'' = 交给车端

        # ---- 规划 ----
        self.declare_parameter("speed_mps", 0.55)
        self.declare_parameter("budget_s", 180.0)
        self.declare_parameter("goal_margin_s", 12.0)
        self.declare_parameter("min_votes", 2)
        self.declare_parameter("vote_window", 24)
        self.declare_parameter("vote_ttl_s", 20.0)
        self.declare_parameter("replan_period_s", 1.0)

        # ---- 话题 ----
        self.declare_parameter("intel_topic", "/link/field_intel")
        self.declare_parameter("waypoint_topic", ROUTE_TOPIC)
        self.declare_parameter("status_topic", STATUS_TOPIC)
        self.declare_parameter("shoot_count_topic", "/shoot_count")

        # ---- 现场开关 ----
        self.declare_parameter("dry_run", True)        # true = 只算不发（第一次上车必须先用它验证）
        self.declare_parameter("publish_reset", True)  # false = 自己已手动 reset 过

        # ---- 出发前置条件（「情报收敛」而非「必须有打击点」）----
        # ⚠️ 早先用「至少 1 个打击点」当出发条件，结果是：无人机若正确地识别出
        #    靶是**友方（红面）**，车端就永远不发路线 —— 白丢终点/排雷/桥洞分。
        self.declare_parameter("min_mines", 1)               # 至少确认几颗雷才出发
        self.declare_parameter("require_target_face", True)  # 敌我未判决前不发车
        self.declare_parameter("min_shoot_stops", 0)         # 额外要求（默认 0 = 不要求）

        # ---- 打击停靠的硬约束（车端实机参数，见 docs/实机接口对照.md §6/§11）----
        # 车端 path_executor：到 '*' 格停车，等 /shoot_count 打满 SHOOT_COUNT_TARGET(=10)
        # 或 150s 超时才继续；而整车弹药只有 10 发 → **只允许一个停靠点**。
        self.declare_parameter("max_shoot_stops", 1)
        self.declare_parameter("shoot_wait_budget_s", 60.0)   # 愿意为打击等待的时间预算
        self.declare_parameter("car_shoot_wait_timeout_s", 150.0)  # 车端超时上限
        self.declare_parameter("fire_only_on_enemy", True)    # 红面（友方）绝不标 '*'

        # ---- 桥洞布点校验（推断值，现场核实后改成严格模式）----
        # 规则：两处桥洞各「5 选 1」「6 选 1」；说明书图 7/8 的标识在 K4 / K9，
        # 而 K 列共 11 行 = 5 + 6 → 推断两处桥洞都沿 K 列排、按行切成 5 格 + 6 格。
        self.declare_parameter("bridge_zone_col", "K")
        self.declare_parameter("bridge_zone_split", 5)
        self.declare_parameter("strict_bridge_zone", False)   # true = 情报不符就拒绝发车

        # ---- 黑匣子（现场复盘，见 docs/调试日志.md）----
        # 默认开：现场"跑了一圈却不知道发生了什么"的代价远大于写几个文件。
        # log_dir 留空 = $AG_LOG_DIR 或系统临时目录/ag_logs（**不写车端家目录**）。
        self.declare_parameter("log_enabled", True)
        self.declare_parameter("log_dir", "")
        self.declare_parameter("log_level", "info")

        p = lambda k: self.get_parameter(k).value  # noqa: E731

        self.grid = FieldGrid(cols=int(p("cols")), rows=int(p("rows")),
                              cell_size=float(p("cell_size")))
        self.car_grid = CarGrid(cols=int(p("cols")), rows=int(p("rows")),
                                cell_size=float(p("cell_size")))
        self.start_cell = str(p("start_cell")).upper()
        self.goal_cell = str(p("goal_cell")).upper()
        self.start_heading = str(p("start_heading")).lower()
        final = str(p("final_heading")).strip().lower()
        self.final_heading: Optional[str] = final if final in ("x", "y") else None
        self.dry_run = bool(p("dry_run"))
        self.publish_reset = bool(p("publish_reset"))
        self.min_mines = max(0, int(p("min_mines")))
        self.require_target_face = bool(p("require_target_face"))
        self.min_shoot_stops = max(0, int(p("min_shoot_stops")))
        self.max_shoot_stops = max(0, int(p("max_shoot_stops")))
        self.shoot_wait_budget_s = max(0.0, float(p("shoot_wait_budget_s")))
        self.car_shoot_wait_timeout_s = max(0.0, float(p("car_shoot_wait_timeout_s")))
        self.fire_only_on_enemy = bool(p("fire_only_on_enemy"))
        self.bridge_zones = BridgeZones(
            col=str(p("bridge_zone_col")),
            split=max(1, int(p("bridge_zone_split"))),
            rows=self.grid.rows,
        )
        self.strict_bridge_zone = bool(p("strict_bridge_zone"))
        self.bridge_check = None

        self.planner = MissionPlanner(
            grid=self.grid,
            start=CellID.parse(self.start_cell),
            goal=CellID.parse(self.goal_cell),
            speed_mps=float(p("speed_mps")),
            budget_s=float(p("budget_s")),
            goal_margin_s=float(p("goal_margin_s")),
        )
        self.voter = IntelVoter(IntelVoteConfig(
            min_votes=int(p("min_votes")),
            max_history=int(p("vote_window")),
            ttl_s=float(p("vote_ttl_s")),
        ))

        self.reset_sent = False
        self.last_route = ""
        self.shots = 0
        #: 实际下发的打击停靠点（可能被时间预算砍掉；cb_timer/复盘用它，别用计划里的 TARGET 任务）
        self._last_shoot_stops: List[str] = []
        self._plan: List[Task] = []
        self.face: Optional[str] = None      # 最近一次稳定的敌我判决（blue=敌 / red=友）
        #: 最近一次"没发车"的原因，以及**已经写进日志的那一条**（避免同一原因刷满日志）
        self._block: Optional[tuple] = None
        self._block_logged: Optional[tuple] = None
        self._last_stable: Optional[tuple] = None

        # ---- 黑匣子 ----
        # session.begin = "一局开始"：复盘工具据此只统计最后一局，
        # 顺带把影响决策的参数全部落进同一行，复现问题时不用再问"当时参数是多少"。
        self.bb = BlackBox.from_params(self)
        self.bb.log(
            EV.SESSION_BEGIN,
            start_cell=self.start_cell, goal_cell=self.goal_cell,
            cols=self.grid.cols, rows=self.grid.rows,
            cell_size=float(p("cell_size")), speed_mps=float(p("speed_mps")),
            budget_s=float(p("budget_s")), goal_margin_s=float(p("goal_margin_s")),
            dry_run=self.dry_run, publish_reset=self.publish_reset,
            min_mines=self.min_mines, require_target_face=self.require_target_face,
            min_shoot_stops=self.min_shoot_stops, max_shoot_stops=self.max_shoot_stops,
            shoot_wait_budget_s=self.shoot_wait_budget_s,
            fire_only_on_enemy=self.fire_only_on_enemy,
            strict_bridge_zone=self.strict_bridge_zone,
            bridge_zones=self.bridge_zones.describe(),
        )

        self.pub_route = self.create_publisher(String, str(p("waypoint_topic")), 10)
        self.pub_status = self.create_publisher(String, str(p("status_topic")), 10)
        self.create_subscription(FieldIntelMsg, str(p("intel_topic")), self.cb_intel, 10)
        self.create_subscription(Int32, str(p("shoot_count_topic")), self.cb_shots, 10)
        self.create_timer(float(p("replan_period_s")), self.cb_timer)

        self.get_logger().info(
            f"car_node 就绪: {self.start_cell}->{self.goal_cell} "
            f"{self.grid.cols}x{self.grid.rows} 格, dry_run={self.dry_run}"
            + ("（dry_run=true：只计算不发命令）" if self.dry_run else "")
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
            EV.INTEL_RECV, level="info",
            ok=ok, why="" if ok else "乱序或重复（投票窗口已丢弃）",
            seq=int(intel.seq), mines=list(intel.mines),
            obstacles=list(intel.obstacles),
            bridge_a=intel.bridge_a, bridge_b=intel.bridge_b,
            target_cells=list(intel.target_cells), target_face=intel.target_face,
        )
        if not ok:
            return  # 乱序 / 重复
        if self.voter.is_confident:
            self.replan()

    def cb_shots(self, msg: Int32) -> None:
        value = int(msg.data)
        if value != self.shots:
            # 只在变化时记：车端每打一发上报一次，正是复盘打击效果的关键证据
            self.bb.log(EV.SHOOT_COUNT, value=value, prev=self.shots)
        self.shots = value

    # ---------------- 规划 -> 路线 ----------------
    def _log_blocked(self, reason: str, detail: str) -> None:
        """记一条「为什么没发车」。

        同一原因只记一次：`replan` 在情报收敛后**每来一帧就会被调用一次**，
        不去重的话日志会被同一个原因刷满，复盘时反而看不出"是哪一刻变的"。
        """
        key = (reason, detail)
        if key == self._block_logged:
            return
        self._block_logged = key
        st = self.voter.stable
        self.bb.log(EV.ROUTE_BLOCKED, level="warn", reason=reason, detail=detail,
                    face=self.face, mines=len(st.mines) if st else 0)

    def replan(self) -> None:
        st = self.voter.stable
        if st is None:
            return
        self.face = st.target_face

        # 情报收敛后**每一帧**都会走到这里（`replan` 由 cb_intel 触发）。
        # 内容没变就不重复写：实测 45 秒能刷出 88 条一模一样的"收敛"，
        # 真正的变化（雷数变了、敌我判决变了）反而被埋在中间看不见。
        snap = (tuple(st.mines), tuple(st.obstacles), st.bridge_a, st.bridge_b,
                tuple(st.target_cells), st.target_face)
        if snap != self._last_stable:
            self._last_stable = snap
            self.bb.log(EV.INTEL_STABLE, mines=list(st.mines), obstacles=list(st.obstacles),
                        bridge_a=st.bridge_a, bridge_b=st.bridge_b,
                        target_cells=list(st.target_cells), target_face=st.target_face)

        # 桥洞布点自检：只在推断的区域内才算"合理"，区域外八成是侦察错了
        # （或我们对场地的推断不对）。默认只告警，不阻断。
        check = check_bridge_pair(st.bridge_a, st.bridge_b, self.bridge_zones,
                                  self.grid, strict=self.strict_bridge_zone)
        self.bridge_check = check
        if check.level == "warn":
            self.get_logger().warning(f"桥洞情报可疑：{check.message()}")
        elif check.level == "reject":
            self.get_logger().error(f"桥洞情报不符（严格模式），拒绝发车：{check.message()}")
            self._log_blocked(REASON.BRIDGE_REJECT, check.message())
            return

        # 只有「蓝面 = 敌方」才值得专程去打击；红面（友方）不排打击任务，
        # 打它要倒扣分（每计数 −5，最多 −15）。
        enemy_target = None
        if self.face == "blue" and st.target_cells:
            enemy_target = st.target_cells[0]
        self.planner.set_intel(
            mines=st.mines,
            obstacles=st.obstacles,
            bridge_a=st.bridge_a,
            bridge_b=st.bridge_b,
            target_cell=enemy_target,
        )
        self._plan = self.planner.full_plan()
        route = self.build_command(st)
        if route is None:
            if self._block is not None:
                self._log_blocked(*self._block)
                self._block = None
            return
        if route == self.last_route:
            return  # 路线没变，不重复下发（车端会重新从头跑）

        stops = list(self._last_shoot_stops)   # 实际下发的停靠点（可能被时间预算砍掉）
        eta_s = round(self.planner.estimate_time(self._plan), 1)
        self.bb.log(EV.PLAN_BUILT, route=route, tasks=len(self._plan),
                    shoot_stops=stops, eta_s=eta_s, face=self.face,
                    dry_run=self.dry_run)

        if not self.dry_run:
            if self.publish_reset and not self.reset_sent:
                cmd = reset_command(self.start_cell, self.start_heading, self.car_grid)
                self._publish(cmd)
                self.reset_sent = True
                self.bb.log(EV.ROUTE_PUBLISH, kind="reset", cmd=cmd, dry_run=False)
            self._publish(route)
            self.bb.log(EV.ROUTE_PUBLISH, kind="route", cmd=route, tasks=len(self._plan),
                        shoot_stops=stops, eta_s=eta_s, dry_run=False)
        self.last_route = route
        self.get_logger().info(
            f"路线[{len(self._plan)} 段] {route}"
            + ("  (dry_run)" if self.dry_run else "")
        )

    def build_command(self, st=None) -> Optional[str]:
        """任务序列 -> 车端路线字符串；不满足出发条件时返回 None（不下发）。

        返回 None 时会把原因写进 `self._block`（`REASON` 常量），
        由 `replan` 统一落到黑匣子 —— 现场复盘第一问就是"为什么没发车"。
        """
        st = st if st is not None else self.voter.stable
        self._block = None
        # 状态里要报的是"实际下发的停靠点"，不是"计划里的 TARGET 任务"——
        # 时间不够砍掉 '*' 之后，两者就不一致了，照计划报会误导现场
        self._last_shoot_stops = []

        # ---- 出发条件：情报已收敛（与「有没有打击点」无关）----
        n_mines = len(st.mines) if st else 0
        if n_mines < self.min_mines:
            self.get_logger().warning(
                f"只确认了 {n_mines} 颗雷（要求 ≥{self.min_mines}），暂不发车"
            )
            self._block = (REASON.MIN_MINES,
                           f"只确认了 {n_mines} 颗雷（要求 ≥{self.min_mines}）")
            return None
        if self.require_target_face and self.face not in ("blue", "red"):
            self.get_logger().warning("敌我尚未判决（蓝/红都没认出来），暂不发车")
            self._block = (REASON.FACE_UNKNOWN,
                           f"target_face={self.face}（要求 blue 或 red）")
            return None

        cells: List[str] = []
        shoot: List[str] = []
        for t in self._plan:
            if t.type == TaskType.LEAVE_START:
                continue  # 车本来就在起点，不必额外发一个"去起点"
            if t.cell is None:
                continue
            # ⚠️ 必须下发 BFS 绕障路径的中间格，不能只发任务格：
            #    纯追踪在相邻路点间走**直线**，而水桶障碍就立在格中央，
            #    只发任务格会让车直接横穿障碍格（比如 C3->K4 一路扫过 8 格）。
            #    task.path 是「上一任务格 -> 本任务格」的完整路径（含起点），
            #    去掉首格避免与上一段的末格重复（build_route 也会再去重）。
            if t.path:
                cells.extend(c.name for c in t.path[1:])
            else:
                cells.append(t.cell.name)   # 兜底：路径缺失时退回任务格
            if t.type == TaskType.TARGET:
                shoot.append(t.cell.name)
        if not cells:
            self._block = (REASON.NO_CELLS, "任务序列为空（情报里既没有雷也没有终点）")
            return None

        # 友方绝不标 '*'（双保险：规划层已经不会把红面排进来）
        if self.fire_only_on_enemy and self.face != "blue" and shoot:
            self.get_logger().warning(
                f"敌我判决为 {self.face}，取消 {len(shoot)} 个打击停靠点（避免误击）"
            )
            shoot = []

        # 弹药只有 10 发 → 一个停靠点就会打光，第二个点只会白等 150s
        if len(shoot) > self.max_shoot_stops:
            self.get_logger().warning(
                f"打击停靠点 {len(shoot)} 个超过上限 {self.max_shoot_stops}"
                f"（整车弹药有限），只保留前 {self.max_shoot_stops} 个"
            )
            shoot = shoot[: self.max_shoot_stops]

        if shoot and not self._fits_shoot_wait():
            self.get_logger().warning("时间不够，取消打击停靠点，优先保证回到终点")
            shoot = []
        self._last_shoot_stops = list(shoot)

        if len(shoot) < self.min_shoot_stops:
            self.get_logger().warning(
                f"路线里只有 {len(shoot)} 个打击点（要求 ≥{self.min_shoot_stops}），暂不下发"
            )
            self._block = (REASON.MIN_SHOOT_STOPS,
                           f"只有 {len(shoot)} 个打击点（要求 ≥{self.min_shoot_stops}）")
            return None

        try:
            return build_route(cells, shoot, self.final_heading, grid=self.car_grid)
        except RouteError as e:
            self.get_logger().error(f"路线非法，已拒绝下发: {e}")
            self._block = (REASON.ROUTE_ERROR, str(e))
            return None

    def _fits_shoot_wait(self) -> bool:
        """判断"行驶 + 打击等待"能否塞进时间预算。

        车端 `path_executor` 在 `*` 格会一直等到 `/shoot_count` 打满 10 发，
        最长 150 s —— 这段时间不产生位移。宁可放弃 15 分打击分，
        也不能把「回终点 +5 分」和计时一起拖垮。
        """
        t_drive = self.planner.estimate_time(self._plan)
        wait = min(self.shoot_wait_budget_s, self.car_shoot_wait_timeout_s)
        limit = self.planner.budget_s - self.planner.goal_margin_s
        return (t_drive + wait) <= limit

    # ---------------- 发布 ----------------
    def _publish(self, text: str) -> None:
        m = String()
        m.data = text
        self.pub_route.publish(m)

    def cb_timer(self) -> None:
        st = self.voter.stable
        stops = list(self._last_shoot_stops)   # 与 build_command 实际下发的 '*' 一致
        s = String()
        s.data = json.dumps({
            "dry_run": self.dry_run,
            "reset_sent": self.reset_sent,
            "route": self.last_route,
            "tasks": len(self._plan),
            "shots": self.shots,
            "mines": list(st.mines) if st else [],
            "bridge_a": st.bridge_a if st else None,
            "bridge_b": st.bridge_b if st else None,
            "target_cells": list(st.target_cells) if st else [],
            "target_face": st.target_face if st else None,
            # 打击决策的可观测性：现场只要看这里就知道"为什么没标 *"
            "face_decided": self.face,
            "shoot_stops": stops,
            "shoot_wait_budget_s": min(self.shoot_wait_budget_s,
                                       self.car_shoot_wait_timeout_s),
            "fits_shoot_wait": self._fits_shoot_wait(),
            "drive_eta_s": round(self.planner.estimate_time(self._plan), 1),
            # 桥洞布点自检（推断值；现场核实后可切 strict_bridge_zone）
            "bridge_zones": self.bridge_zones.describe(),
            "bridge_check_ok": (self.bridge_check.ok if self.bridge_check else None),
            "bridge_problems": (self.bridge_check.problems if self.bridge_check else []),
        }, ensure_ascii=False)
        self.pub_status.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = CarNode()
    try:
        rclpy.spin(node)
    finally:
        node.bb.close()      # 先把缓冲刷盘，再拆节点
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
