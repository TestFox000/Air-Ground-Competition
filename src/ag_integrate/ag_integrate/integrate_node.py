"""车端集成节点：接收 dzuav 二进制 UDP:50000 -> 官方 path_executor。

    UDP 广播 (dzuav UdpBroadcaster)
        -> 后台收包线程（只做 收包/解析/态势归并）
        -> 最新帧队列（容量 1，决策慢时自动丢旧帧——态势数据只要最新）
        -> ROS timer 回调（executor 线程）：门控 -> 路由+适配 -> 发布
        -> /nav/waypoint  先发一次 reset，再发路线（官方 path_executor 消费）

线程模型：规划与发布都在 ROS executor 线程的 timer 回调里做，
后台线程只把最新 WorldSnapshot 递进来，不跨线程调用 ROS API。

安全默认：dry_run=True（只计算、打日志，不发命令）；上车验证后再置 false。
"""

from __future__ import annotations

import math
import queue
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from .field_matrix import MatrixFusion, render_report_from_spots
from .heading_calibration import closed_loop_rate, nearest_axis, wrap_pi
from .intel_protocol import detect_msg_id, parse, parse_car_pose
from .pipeline import build_mission, evaluate_gate
from .world_model import WorldModel

# reset 断言朝向对应的场地 yaw（弧度，0=东、π/2=北，与 dzuav 0x4002 同架）
_HEADING_YAW = {"x": 0.0, "y": math.pi / 2}

# 黑匣子（软依赖）：ag_core 未装时静默关闭，不影响主流程。
try:
    from ag_core.blackbox import BlackBox, EV
    _HAS_BB = True
except ImportError:
    _HAS_BB = False


class IntegrateNode(Node):
    def __init__(self) -> None:
        super().__init__("integrate_node")

        self.declare_parameter("udp_port", 50000)
        self.declare_parameter("waypoint_topic", "/nav/waypoint")
        self.declare_parameter("start_cell", "N10")
        self.declare_parameter("goal_cell", "N11")
        self.declare_parameter("start_heading", "y")
        self.declare_parameter("final_heading", "y")
        self.declare_parameter("dry_run", True)
        self.declare_parameter("publish_reset", True)
        self.declare_parameter("min_mines", 1)
        self.declare_parameter("require_face", True)
        self.declare_parameter("cross_gates", [""])
        # 中场横向挡板位置：必须与 mission_router._MID_BOUNDARY 一致
        # （挡板在第 6/7 行之界，=6）。test_field_config_consistency 会校验。
        self.declare_parameter("mid_boundary", 5)
        # 外部朝向补偿（根除绕圈问题）：
        #   heading_bias_source="none"  → 不补偿，保持现状（默认，安全）
        #   heading_bias_source="udp"   → 监听 dzuav 的 0x4002 车位姿帧，
        #                                  用无人机观测的真实朝向修正 reset 断言
        #                                  （取最近坐标轴，残差 ≤ 45°，不挪车）
        #   heading_bias_source="align" → 用 0x4002 真实朝向闭环原地转向，
        #                                  把车头转到与 start_heading 断言一致
        #                                  （零残差，需发 /cmd_vel 角速度）
        #   heading_bias（弧度）         → 手动偏置（=真实朝向 - 断言朝向），
        #                                  优先级低于 udp/align 自动补偿
        self.declare_parameter("heading_bias_source", "none")
        self.declare_parameter("heading_bias", 0.0)
        # ---- 朝向对齐（align 模式）参数 ----
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("heading_align_tolerance", 0.05)   # rad ≈ 3°
        self.declare_parameter("heading_align_max_rate", 0.5)    # rad/s
        self.declare_parameter("heading_align_timeout", 15.0)    # s
        self.declare_parameter("heading_align_settle_ticks", 3)  # 连续 N tick 在容差内才算对齐
        # 移动靶先验（全场仅一个，裁判系统每局固定）：
        #   target_track = ["D8", "E8"]  轨道两端格号
        #   target_blue_face = "S"      蓝面朝向 S/N(水平轨道)/E/W(垂直轨道)
        # 无人机只识别瞬时哪面可见（TARGET_BLUE/RED），轨道与朝向由本参数注入。
        self.declare_parameter("target_track", [""])
        self.declare_parameter("target_blue_face", "")
        # 中途重规划开关（默认关）：path_executor 对"执行中途换路线"的
        # 行为未验证（从头执行？当前段接续？），实车确认前保持一次性规划。
        # 置 true 后：态势变化导致路线变化时自动重发新路线。
        self.declare_parameter("replan_on_update", False)
        # 态势矩阵输出（教员建议的矩阵形式）：
        # 每收到一帧识别结果，把爆炸物/障碍物/靶等落成 11×14 场地矩阵并
        # 打印精确坐标明细。可靠性设计：
        #   - 多帧滑窗融合（window 票 confirm 才上矩阵）：抑制单帧漏检
        #     闪断与单帧误检幽灵目标，坐标输出窗口内中值（更稳）；
        #   - 仅矩阵内容变化时打印，且最少间隔 matrix_min_interval 秒。
        self.declare_parameter("matrix_log", True)
        self.declare_parameter("matrix_fuse_window", 5)
        self.declare_parameter("matrix_fuse_confirm", 2)
        self.declare_parameter("matrix_min_interval", 2.0)
        # ---- 黑匣子（ag_core.blackbox）----
        self.declare_parameter("log_enabled", True)
        self.declare_parameter("log_dir", "")
        self.declare_parameter("log_level", "info")

        p = lambda k: self.get_parameter(k).value  # noqa: E731
        self.start_cell = str(p("start_cell"))
        self.goal_cell = str(p("goal_cell"))
        self.start_heading = str(p("start_heading"))
        fh = str(p("final_heading")).strip().lower()
        self.final_heading = fh if fh in ("x", "y") else None
        self.dry_run = bool(p("dry_run"))
        self.publish_reset = bool(p("publish_reset"))
        self.min_mines = int(p("min_mines"))
        self.require_face = bool(p("require_face"))
        self.cross_gates = tuple(n.strip() for n in p("cross_gates") if n.strip())
        self.mid_boundary = int(p("mid_boundary"))
        self.heading_bias_source = str(p("heading_bias_source")).strip().lower()
        self.heading_bias = float(p("heading_bias"))
        self.cmd_vel_topic = str(p("cmd_vel_topic"))
        self.heading_align_tolerance = float(p("heading_align_tolerance"))
        self.heading_align_max_rate = float(p("heading_align_max_rate"))
        self.heading_align_timeout = float(p("heading_align_timeout"))
        self.heading_align_settle_ticks = int(p("heading_align_settle_ticks"))
        track = tuple(str(n).strip().upper() for n in p("target_track") if str(n).strip())
        self.target_track = track if len(track) == 2 else None
        bf = str(p("target_blue_face")).strip().upper()
        self.target_blue_face = bf if bf in ("S", "N", "E", "W") else None
        self.replan_on_update = bool(p("replan_on_update"))
        self.matrix_log = bool(p("matrix_log"))
        self._matrix_sig: tuple | None = None  # 上次打印的矩阵签名
        self._matrix_last_ts: float = -1e9     # 上次打印时刻（节流）
        self._fusion = MatrixFusion(
            window=int(p("matrix_fuse_window")),
            confirm=int(p("matrix_fuse_confirm")),
        )
        self._matrix_min_interval = float(p("matrix_min_interval"))

        self.model = WorldModel()
        self.reset_sent = False
        self.last_route = ""
        # 无人机观测到的车真实朝向（0x4002 帧），None 表示尚未收到
        self._car_true_yaw: float | None = None

        # ---- 朝向对齐状态机（align 模式）----
        # _align_state: None=未启动, "aligning"=转向中, "aligned"=已对齐, "failed"=超时/无朝向
        self._align_state: str | None = None
        self._align_settle_count = 0
        self._align_start_time: float | None = None

        self.pub = self.create_publisher(String, str(p("waypoint_topic")), 10)
        # 朝向对齐时直接发底盘角速度（path_executor 无路线时不占 /cmd_vel）
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # 收包线程 -> 决策回调的最新帧交接：容量 1，满了丢旧帧。
        # 态势数据只要最新值，决策慢于到达率时自动降级为"跳帧"，不积压。
        self._frames: queue.Queue = queue.Queue(maxsize=1)

        # 决策/发布都在 executor 线程（timer 回调）里做，收包线程不碰 ROS。
        self.create_timer(0.1, self._on_tick)

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._receive, args=(int(p("udp_port")),), daemon=True)
        self._thread.start()

        # 黑匣子挂载 + 关键参数快照（复盘一局"为什么这么走"的依据）
        self.bb = BlackBox.from_params(self) if _HAS_BB else None
        self._blog(EV.NODE_ARGS if _HAS_BB else "", dry_run=self.dry_run,
                   start=self.start_cell, goal=self.goal_cell,
                   heading=self.start_heading, final=self.final_heading,
                   udp_port=int(p("udp_port")), bias=self.heading_bias_source,
                   track=self.target_track, replan=self.replan_on_update,
                   log_file=str(self.bb.path) if self.bb else "")

        mode = "DRY_RUN（只算不发）" if self.dry_run else "实发"
        self.get_logger().info(
            f"integrate_node 就绪: 监听 UDP:{p('udp_port')} -> "
            f"{self.start_cell}->{self.goal_cell}, {mode}"
        )

    # ---------------- 黑匣子 ----------------
    def _blog(self, ev: str, level: str = "info", **fields) -> None:
        """黑匣子软接口：没挂上（ag_core 缺失/写盘失败）就是空操作。"""
        if self.bb is not None and ev:
            self.bb.log(ev, level=level, **fields)

    # ---------------- UDP 接收（后台线程，只解析不入 ROS） ----------------
    def _receive(self, port: int) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", port))
        sock.settimeout(0.5)
        while not self._stop.is_set():
            try:
                data, _addr = sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            self._process_packet(data)
        sock.close()

    def _process_packet(self, data: bytes) -> None:
        """处理一帧 UDP 数据：区分 0x4001（态势）和 0x4002（车位姿）。"""
        msg_id = detect_msg_id(data)
        if msg_id is None:
            return
        if msg_id == 0x4002:
            # 车位姿帧：无人机观测的车真实朝向，用于修正 reset 断言
            try:
                pose = parse_car_pose(data)
            except ValueError as exc:
                self.get_logger().warning(f"丢弃坏车位姿帧: {exc}")
                self._blog(EV.LINK_DROP, level="warn", msg=f"car_pose: {exc}")
                return
            self._car_true_yaw = pose.yaw
            self.get_logger().debug(
                f"收到车位姿: x={pose.x:.2f} y={pose.y:.2f} yaw={pose.yaw:.3f}"
            )
            return
        # 默认：态势帧（0x4001）
        try:
            frame = parse(data)
        except ValueError as exc:
            self.get_logger().warning(f"丢弃坏帧: {exc}")
            self._blog(EV.LINK_DROP, level="warn", msg=str(exc))
            return
        snapshot = self.model.update(frame)
        if self.bb is not None and self.bb.throttled("intel", 5.0):
            self._blog(EV.INTEL_RECV, n=len(frame.objects),
                       ts=frame.timestamp_ms,
                       mines=len(snapshot.mines) if snapshot else 0)
        if self.matrix_log:
            self._log_matrix(frame)
        self._submit(snapshot)

    def _log_matrix(self, frame) -> None:
        """态势矩阵输出：多帧融合确认目标 + 精确坐标。

        打印条件（AND）：矩阵内容变化 且 距上次打印 ≥ matrix_min_interval 秒。
        移动靶高频移动时靠节流兜底，避免刷屏。
        """
        import time as _time
        spots = self._fusion.add_frame(frame) + self._fusion.outside_spots()
        sig = tuple(sorted((s.kind.value, s.col, s.row) for s in spots))
        now = _time.monotonic()
        if sig == self._matrix_sig:
            return  # 矩阵无变化，不重复打印
        if now - self._matrix_last_ts < self._matrix_min_interval:
            return  # 变了但太快，等下个变化窗口
        self._matrix_sig = sig
        self._matrix_last_ts = now
        self.get_logger().info("\n" + render_report_from_spots(
            spots, frame.timestamp_ms, fused=True))

    def _submit(self, snapshot) -> None:
        """把最新态势放入交接队列；满了先丢旧帧再放（单生产者，无竞态）。"""
        try:
            self._frames.put_nowait(snapshot)
        except queue.Full:
            try:
                self._frames.get_nowait()
                self._frames.put_nowait(snapshot)
            except (queue.Empty, queue.Full):
                pass

    # ---------------- 决策（executor 线程） ----------------
    def _on_tick(self) -> None:
        # 朝向对齐不等情报门控：无人机一看到车（0x4002）立即原地对齐，
        # 与空中侦察并行，不把对齐耗时（上限 heading_align_timeout 秒）
        # 压在发车关键路径上——比赛 180s 硬顶，同分比用时。
        if (self._needs_heading_alignment() and self._align_state is None
                and self._car_true_yaw is not None):
            self._start_heading_alignment()
        try:
            snapshot = self._frames.get_nowait()
        except queue.Empty:
            snapshot = None
        if snapshot is not None:
            self._handle(snapshot)
        # 朝向对齐需要每 tick 持续发 /cmd_vel；放在 _handle 之后，
        # 这样 _handle 启动对齐的同一 tick 就会发出第一帧角速度。
        if self._align_state == "aligning":
            self._do_heading_alignment()

    def _effective_start_heading(self) -> str:
        """根据无人机观测的真实朝向，选最近的坐标轴方向作为 reset 断言。

        path_executor 的 reset 只支持 'x'(0°)/'y'(90°)，无法传任意偏置。
        - udp 模式：选最近坐标轴，残差 ≤ 45°，不挪车；
        - align 模式：已通过 _do_heading_alignment 把车头转到 start_heading，
          直接用 start_heading（零残差）；
        - none：退回 start_heading（人工摆车对齐）。
        """
        src = self.heading_bias_source
        if src == "align":
            # 对齐成功：车头已物理转到 start_heading 断言方向，零残差
            if self._align_state == "aligned":
                return self.start_heading
            # 对齐失败/未完成：退回最近坐标轴（与 udp 模式同逻辑）
            if self._car_true_yaw is not None:
                return nearest_axis(self._car_true_yaw)
            return self.start_heading
        if src == "udp" and self._car_true_yaw is not None:
            # 环绕角取最近轴：任意朝向残差恒 ≤45°（朝西车旧逻辑会错选 x）
            nearest = nearest_axis(self._car_true_yaw)
            if nearest != self.start_heading:
                self.get_logger().info(
                    f"外部朝向修正: 断言 {self.start_heading} -> {nearest} "
                    f"(观测 yaw={math.degrees(self._car_true_yaw):.1f}°)"
                )
            return nearest
        return self.start_heading

    # ---------------- 朝向对齐（align 模式） ----------------
    def _needs_heading_alignment(self) -> bool:
        """是否需要执行朝向对齐。"""
        return self.heading_bias_source == "align"

    def _start_heading_alignment(self) -> None:
        """启动朝向对齐：把车头原地转到 start_heading 断言方向。"""
        if self._align_state in ("aligning", "aligned"):
            return
        self._align_state = "aligning"
        self._align_settle_count = 0
        self._align_start_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(
            f"朝向对齐启动: 目标 {self.start_heading} "
            f"({math.degrees(_HEADING_YAW[self.start_heading]):.1f}°)"
        )

    def _do_heading_alignment(self) -> None:
        """每 tick 执行：读无人机真实 yaw，闭环发角速度直到对齐。"""
        if self._align_state != "aligning":
            return

        now = self.get_clock().now().nanoseconds / 1e9
        # 超时保护
        if (self._align_start_time is not None
                and now - self._align_start_time > self.heading_align_timeout):
            self._align_state = "failed"
            self._stop_cmd_vel()
            self.get_logger().warning(
                f"朝向对齐超时 ({self.heading_align_timeout}s)，"
                f"退回 udp 最近轴断言"
            )
            return

        if self._car_true_yaw is None:
            # 还没收到车位姿帧，保持停止等待
            self._stop_cmd_vel()
            return

        target_yaw = _HEADING_YAW[self.start_heading]
        rate = closed_loop_rate(
            self._car_true_yaw, target_yaw,
            max_rate=self.heading_align_max_rate,
            tolerance=self.heading_align_tolerance,
        )
        if rate == 0.0:
            self._align_settle_count += 1
            self._stop_cmd_vel()
            if self._align_settle_count >= self.heading_align_settle_ticks:
                self._align_state = "aligned"
                self.get_logger().info(
                    f"朝向对齐完成: 观测 "
                    f"{math.degrees(self._car_true_yaw):.1f}° ≈ 目标 "
                    f"{math.degrees(target_yaw):.1f}°"
                )
        else:
            self._align_settle_count = 0
            self._publish_cmd_vel_angular(rate)

    def _publish_cmd_vel_angular(self, rate: float) -> None:
        msg = Twist()
        msg.angular.z = rate
        self.cmd_vel_pub.publish(msg)

    def _stop_cmd_vel(self) -> None:
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

    def _handle(self, snapshot) -> None:
        gate = evaluate_gate(snapshot, self.min_mines, self.require_face)
        if not gate.ok:
            self._log_hold_once(gate.reason)
            self._blog(EV.ROUTE_BLOCKED, level="warn", reason=gate.reason)
            return

        # 朝向对齐（align 模式）：发车前先把车头摆正，消除 reset 断言偏差。
        if self._needs_heading_alignment() and self._align_state is None:
            self._start_heading_alignment()
        if self._align_state == "aligning":
            return   # 对齐中，不发路线
        if self._align_state == "failed":
            # 对齐失败：退回 udp 最近轴，仍尝试发车（比完全不动好）
            pass

        # 默认关：首条路线发出后不再重规划（一次性规划模式），
        # 跳过 build_mission 也省掉每帧规划开销。
        if self.last_route and not self.replan_on_update:
            return

        start_heading = self._effective_start_heading()
        commands = build_mission(
            snapshot, self.start_cell, self.goal_cell,
            start_heading, self.final_heading,
            self.cross_gates, self.mid_boundary,
            target_track=self.target_track,
            target_blue_face=self.target_blue_face,
        )
        if commands.route == self.last_route:
            return
        self.last_route = commands.route
        self._blog(EV.PLAN_BUILT, reset=commands.reset, route=commands.route,
                   shoot=",".join(commands.shoot_cells),
                   heading=start_heading)

        if self.dry_run:
            self.get_logger().info(
                f"[dry_run] {commands.reset} | {commands.route}"
            )
            return

        if self.publish_reset and not self.reset_sent:
            self._publish(commands.reset)
            self.reset_sent = True
        self._publish(commands.route)
        self._blog(EV.ROUTE_PUBLISH, cmd=commands.route, kind="route",
                   reset_sent=self.reset_sent)
        self.get_logger().info(f"已下发路线: {commands.route}")

    def _log_hold_once(self, reason: str) -> None:
        if reason != getattr(self, "_last_hold", None):
            self.get_logger().info(f"暂不发车：{reason}")
            self._last_hold = reason

    def _publish(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.pub.publish(msg)

    def destroy_node(self) -> bool:
        self._stop.set()
        self._thread.join(timeout=2)
        # 安全：关闭时若正在对齐，发零角速度让底盘停转
        if self._align_state == "aligning":
            try:
                self._stop_cmd_vel()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IntegrateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
