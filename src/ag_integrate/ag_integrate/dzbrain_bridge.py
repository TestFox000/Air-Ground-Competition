"""dzbrain 桥接节点：机载 dzuav 二进制 UDP 情报 -> dzbrain JSON 话题。

模式定位：车上现役默认决策核心是官方 dzbrain（只读快照在
reference/car_src_20261015/dzbrain/），它的无人机情报接口是 ROS 话题
JSON 报文（std_msgs/String），不再是 integrate_node -> path_executor 的
legacy 路线串路径。本节点就是 dzbrain 模式下的链路胶水。**两模式互斥**：
dzbrain 模式起本节点，legacy 模式起 integrate_node，launch 侧二选一，
绝不同时跑（同抢 UDP:50000，且决策核心只能有一个）。互斥由工程防线兜底：
两边都不设 SO_REUSEADDR，重复 bind 同一端口时第二个节点构造即抛
（UDP 无 TIME_WAIT，进程死掉端口即释放，重启不受阻）。

    UDP 广播 (dzuav UdpBroadcaster, 0x4001 态势帧)
        -> 后台收包线程（只做 收包/解析/态势归并入 WorldModel）
        -> 最新帧队列（容量 1，决策慢时自动丢旧帧——态势数据只要最新）
        -> ROS timer 回调（executor 线程，10Hz）：心跳 + 变化检测/周期刷新
        -> /dzlink/drone_to_car（std_msgs/String，JSON 文本）

报文（字段以 dzbrain/tasks/drone.py DroneTask 的解析逻辑为真源）：
    heartbeat       {"type":"heartbeat","ts":...,"seq":N}
                    1Hz 恒定发（无论有无情报）；DroneTask 丢失 >3s 判离线
    bridge_passable {"type":"bridge_passable","half":"lower"|"upper",
                     "block":"K9","confidence":1.0,"ts":...,"seq":N}
                    桥洞可通行格（ARROW，无人机是权威）。按行分半场：
                    行7~11→lower、行1~6→upper（挡板在 6/7 行界），每半场一条；
                    变化即发 + 无变化每 refresh_period_s 刷新一次
                    （dzbrain info_stale_s=30s 兜底，刷新周期须远小于它）。
                    confidence 恒 1.0：我们识别即确认；带 seq 让车端可 ack。
    target_zone     {"type":"target_zone","blocks":["E5",...]}
                    靶区格号列表（红+蓝都报，车上与本地地图核对用）；
                    空列表会触发 DroneTask 缺字段告警，故无靶时整条不发。
    field_verify    {"type":"field_verify","mines":[...],"obstacles":[...]}
                    雷/障格号核对；空列表是有效情报（"全场干净"），两键恒带。

订阅 /dzlink/car_to_drone：ack/car_state 记 debug 日志，坏消息 warn 不崩。
本节点**不做订阅者闸门**：情报该发就发，不等车端订阅上线（心跳同理）。

线程模型：与 integrate_node 相同——发布都在 executor 线程的 timer 回调里，
收包线程只解析入队，不跨线程调用 ROS API。

安全默认：本节点不驱动车（不发路线、不发 /cmd_vel，选格决策是 dzbrain
MissionTask 的事），故无需 dry_run。任何异常（socket/序列化/ROS）都兜底
记日志，绝不让节点崩——DroneTask 对坏消息同样只记日志不崩，两端对称。
"""

from __future__ import annotations

import json
import queue
import socket
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .intel_protocol import MSG_ID, detect_msg_id, parse
from .mission_router import _MID_BOUNDARY
from .world_model import WorldModel

# 黑匣子（软依赖）：ag_core 未装时静默关闭，不影响主流程（同 integrate_node）。
try:
    from ag_core.blackbox import BlackBox, EV
    _HAS_BB = True
except ImportError:
    _HAS_BB = False

    class _StubNS:
        """ag_core 缺失时的占位命名空间：任何属性名都解析为空串。

        收包线程/timer 回调里 EV.XXX 作为实参直接求值，没有占位命名空间
        会在 ag_core 缺失时 NameError 杀线程/崩节点；空串事件名被 _blog
        直接丢弃，等价于黑匣子关闭。
        """

        def __getattr__(self, _name: str) -> str:
            return ""

    EV = _StubNS()  # type: ignore[assignment]
    BlackBox = None  # type: ignore[assignment]

# dzbrain 桥专用事件名：ag_core.blackbox.EV 没有对应常量，按约定用字符串
# 字面量（bb 为 None 时 _blog 整体空操作，字面量与 EV 常量两种模式行为一致）。
_EV_SEND = "dzbrain.send"   # 向 drone_to_car 发出一条 JSON 报文
_EV_RECV = "dzbrain.recv"   # 从 car_to_drone 收到一条 JSON 报文

_BRIDGE_HALVES = ("lower", "upper")

# 周期比较 epsilon：吸收浮点周期（如 0.3）与整数 ns 时钟换算的表示误差
# （1.1+0.3=1.4000000000000001 > 时钟读数 1.4，没它会晚一拍才发）。
# 1ns 远小于任何有意义的周期，只兜底表示误差，不改变调度语义。
_TIME_EPS = 1e-9


class DzbrainBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("dzbrain_bridge")

        self.declare_parameter("udp_port", 50000)
        self.declare_parameter("drone_to_car_topic", "/dzlink/drone_to_car")
        self.declare_parameter("car_to_drone_topic", "/dzlink/car_to_drone")
        # 心跳周期：DroneTask 心跳超时默认 3s，1Hz 有两拍丢包余量
        self.declare_parameter("heartbeat_period_s", 1.0)
        # 无变化刷新周期：须远小于 DroneTask info_stale_s（默认 30s）兜底
        self.declare_parameter("refresh_period_s", 2.0)
        # 红靶粘性否决 TTL（秒）透传 WorldModel，语义同 integrate_node
        self.declare_parameter("red_veto_ttl_s", 5.0)
        # ---- 黑匣子（ag_core.blackbox）----
        self.declare_parameter("log_enabled", True)
        self.declare_parameter("log_dir", "")
        self.declare_parameter("log_level", "info")

        p = lambda k: self.get_parameter(k).value  # noqa: E731
        self.drone_to_car_topic = str(p("drone_to_car_topic"))
        self.car_to_drone_topic = str(p("car_to_drone_topic"))
        self.heartbeat_period_s = float(p("heartbeat_period_s"))
        self.refresh_period_s = float(p("refresh_period_s"))
        # 周期参数 fail-fast 校验（对齐 integrate_node 启动自检范式）：
        # <=0 会让心跳/刷新每拍连发刷爆话题，且 DroneTask 离线判定/刷新兜底
        # 的时序假设全部失效——非法值构造即抛，不留到赛中
        if self.heartbeat_period_s <= 0:
            raise ValueError(
                f"heartbeat_period_s 必须 >0，收到 {self.heartbeat_period_s}")
        if self.refresh_period_s <= 0:
            raise ValueError(
                f"refresh_period_s 必须 >0，收到 {self.refresh_period_s}")

        self.model = WorldModel(red_veto_ttl_s=float(p("red_veto_ttl_s")))

        self.pub = self.create_publisher(String, self.drone_to_car_topic, 10)
        self.create_subscription(
            String, self.car_to_drone_topic, self._on_car_msg, 10)

        # 收包线程 -> 决策回调的最新帧交接：容量 1，满了丢旧帧。
        # 态势数据只要最新值，决策慢于到达率时自动降级为"跳帧"，不积压。
        self._frames: queue.Queue = queue.Queue(maxsize=1)

        # UDP socket 在构造函数里先 bind 好再交给收包线程：端口被占/权限
        # 不足时在启动时直接抛错（fail-fast，同 integrate_node 范式），
        # 而不是收包线程静默死亡、节点照常"就绪"。
        # ⚠️ 刻意不设 SO_REUSEADDR：重复 bind 即 fail-fast 是与 integrate_node
        # 互斥的防线——两节点同抢 UDP:50000，文件头承诺的"绝不同时跑"靠的是
        # 第二个 bind 当场炸（Linux 下 SO_REUSEADDR 会让双方重复 bind 且各收
        # 一份副本，双决策核心并行驱动车）。UDP 无 TIME_WAIT，进程死掉端口
        # 即释放，去掉它重启不受阻（2026-10-17 审查修正）。
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._sock.bind(("", int(p("udp_port"))))
        except OSError as exc:
            self._sock.close()
            fatal = getattr(self.get_logger(), "fatal", self.get_logger().error)
            fatal(f"UDP 端口 {p('udp_port')} 绑定失败: {exc}")
            raise
        self._sock.settimeout(0.5)

        # 发送状态：kind -> (签名, 发出时刻)。签名变化立即发，
        # 无变化每 refresh_period_s 刷新一次。
        self._sent: dict[str, tuple[object, float]] = {}
        self._seq = 0
        self._next_heartbeat_at: float | None = None

        # 决策/发布都在 executor 线程（timer 回调）里做，收包线程不碰 ROS。
        self.create_timer(0.1, self._on_tick)

        # 黑匣子挂载 + 关键参数快照（复盘一局"为什么这么发"的依据）。
        # 必须在收包线程启动之前挂好：收包线程的 _process_packet/_blog 引用
        # self.bb，线程先起跑会在早窗口收包时 AttributeError（2026-10-17 审查修正）。
        self.bb = BlackBox.from_params(self) if _HAS_BB else None
        self._blog(EV.SESSION_BEGIN, mode="dzbrain_bridge",
                   udp_port=int(p("udp_port")))
        self._blog(EV.NODE_ARGS, udp_port=int(p("udp_port")),
                   drone_to_car=self.drone_to_car_topic,
                   car_to_drone=self.car_to_drone_topic,
                   heartbeat_period_s=self.heartbeat_period_s,
                   refresh_period_s=self.refresh_period_s,
                   log_file=str(self.bb.path) if self.bb else "")

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._receive, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f"dzbrain_bridge 就绪: 监听 UDP:{p('udp_port')} -> "
            f"{self.drone_to_car_topic}（dzbrain 模式，不驱动车）"
        )

    # ---------------- 黑匣子 ----------------
    def _blog(self, ev: str, level: str = "info", **fields) -> None:
        """黑匣子软接口：没挂上（ag_core 缺失/写盘失败）就是空操作。"""
        if self.bb is None:
            return
        if ev:
            self.bb.log(ev, level=level, **fields)

    # ---------------- UDP 接收（后台线程，只解析不入 ROS） ----------------
    def _receive(self) -> None:
        sock = self._sock   # 构造函数里已 bind 好；这里只收包
        while not self._stop.is_set():
            try:
                data, _addr = sock.recvfrom(65535)  # UDP 报文上限，绝不截断
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                self._process_packet(data)
            except Exception as exc:  # noqa: BLE001  收包线程不许死
                self.get_logger().warning(
                    f"处理 UDP 包异常（已忽略）: {type(exc).__name__}: {exc}")
        sock.close()

    def _process_packet(self, data: bytes) -> None:
        """处理一帧 UDP 数据：只认 0x4001 态势帧。

        0x4002 车位姿帧静默忽略：dzbrain 自行定位/观测，本桥只报场地情报。
        """
        msg_id = detect_msg_id(data)
        if msg_id is None or msg_id != MSG_ID:
            return
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
        self._submit(snapshot)

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
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_tick(self) -> None:
        now = self._now()
        self._maybe_heartbeat(now)
        # 交接队列只是"新帧到达"的交接信号，排空即可；决策源取 WorldModel
        # 当前快照（不可变对象，收包线程写、本线程读，靠 GIL 单属性赋值的
        # 原子性，不加锁）——刷新周期到点但没有新帧时也要能补发，
        # 不能只看队列里有没有新东西。
        try:
            self._frames.get_nowait()
        except queue.Empty:
            pass
        snapshot = self.model.snapshot
        if snapshot is None:
            return
        try:
            self._publish_intel(snapshot, now)
        except Exception as exc:  # noqa: BLE001  兜底：异常不许穿透 timer
            self.get_logger().error(
                f"情报发布异常（下拍重试）: {type(exc).__name__}: {exc}")
            self._blog(EV.NODE_ERROR, level="error",
                       where="publish_intel", msg=str(exc))

    # ---- 心跳：1Hz 恒定发，无论有无情报 ----
    def _maybe_heartbeat(self, now: float) -> None:
        if (self._next_heartbeat_at is not None
                and now + _TIME_EPS < self._next_heartbeat_at):
            return
        if self._send({
            "type": "heartbeat", "ts": round(now, 3), "seq": self._next_seq(),
        }):
            self._next_heartbeat_at = now + self.heartbeat_period_s

    # ---- 情报：变化即发 + 周期刷新 ----
    def _publish_intel(self, snapshot, now: float) -> None:
        # 桥洞可通行格（ARROW，无人机是权威）：按行分半场，行7~11→lower、
        # 行1~6→upper。挡板在 6/7 行界，_MID_BOUNDARY 是 0 基的第 7 行下标
        # （mission_router/场地 yaml/本节点三处口径一致）。
        halves: dict[str, list[str]] = {h: [] for h in _BRIDGE_HALVES}
        for cell in snapshot.passable:
            half = "lower" if cell.row >= _MID_BOUNDARY else "upper"
            halves[half].append(cell.name)
        for half in _BRIDGE_HALVES:
            blocks = sorted(halves[half])
            if not blocks:
                continue
            # 每半场只发一条（DroneTask 每半场只存一格，多条会互相覆盖）。
            # 一局物理上只有一个桥洞格；识别出多格时取排序首格并记告警。
            if len(blocks) > 1:
                self.get_logger().warning(
                    f"{half} 半场识别到多个桥洞格 {blocks}，只上报 {blocks[0]}")
            kind = f"bridge:{half}"
            sig = (half, blocks[0])
            if not self._due(kind, sig, now):
                continue
            if self._send({
                "type": "bridge_passable", "half": half, "block": blocks[0],
                "confidence": 1.0, "ts": round(now, 3), "seq": self._next_seq(),
            }):
                self._sent[kind] = (sig, now)

        # 靶区格号（红+蓝都报，车上与本地地图核对用；空列表会触发
        # DroneTask 缺字段告警，所以无靶时整条不发）
        targets = sorted({t.cell.name for t in snapshot.targets})
        if targets:
            self._send_typed("target_zone", tuple(targets),
                             {"type": "target_zone", "blocks": targets}, now)

        # 雷/障核对：空列表也是有效情报（"全场干净"，DroneTask 照常 _diff，
        # 只有缺键才告警），所以两键恒带
        mines = sorted({c.name for c in snapshot.mines})
        obstacles = sorted({c.name for c in snapshot.obstacles})
        self._send_typed("field_verify", (tuple(mines), tuple(obstacles)),
                         {"type": "field_verify",
                          "mines": mines, "obstacles": obstacles}, now)

    def _due(self, kind: str, sig, now: float) -> bool:
        """该类报文签名变化了，或距上次发出已达 refresh_period_s。"""
        last = self._sent.get(kind)
        if last is None:
            return True
        return sig != last[0] or now - last[1] + _TIME_EPS >= self.refresh_period_s

    def _send_typed(self, kind: str, sig, payload: dict, now: float) -> None:
        if not self._due(kind, sig, now):
            return
        if self._send(payload):
            self._sent[kind] = (sig, now)

    def _next_seq(self) -> int:
        self._seq += 1
        return self._seq

    def _send(self, payload: dict) -> bool:
        """序列化并发布一条 JSON；任何异常记日志返回 False（不许崩节点）。"""
        try:
            body = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
            msg = String()
            msg.data = body
            self.pub.publish(msg)
        except Exception as exc:  # noqa: BLE001  序列化/ROS 全部兜底
            self.get_logger().warning(
                f"报文发送失败（下拍重试）: {type(exc).__name__}: {exc}")
            return False
        if payload.get("type") == "heartbeat":
            # 心跳 1Hz 恒定发，全量进黑匣子会刷屏：限流记录（首次必放行）
            if self.bb is not None and self.bb.throttled("hb", 5.0):
                self._blog(_EV_SEND, body=body)
        else:
            self._blog(_EV_SEND, body=body)
        return True

    # ---------------- 收车端回报（/dzlink/car_to_drone） ----------------
    def _on_car_msg(self, msg) -> None:
        """解析车端 JSON：ack/car_state 记 debug；坏消息 warn 不崩。

        ★ 与 DroneTask._handle_raw 对称：任何一条坏消息都只记日志，
          绝不上抛（"一条坏消息杀一个回调"是旧代码教训）。
        """
        try:
            raw = getattr(msg, "data", None)
            if not isinstance(raw, str):
                self.get_logger().warning(
                    f"车端消息类型非字符串（丢弃）: {type(raw).__name__}")
                return
            try:
                payload = json.loads(raw)
            except Exception as exc:  # noqa: BLE001  非法 JSON
                self.get_logger().warning(
                    f"车端消息非法 JSON（丢弃）: {type(exc).__name__}: "
                    f"{raw[:80]!r}")
                return
            if not isinstance(payload, dict):
                self.get_logger().warning(
                    f"车端消息不是 JSON 对象（丢弃）: {type(payload).__name__}")
                return
            msg_type = str(payload.get("type") or "").strip()
            if msg_type == "ack":
                self.get_logger().debug(f"收到车端回执: ack of={payload.get('of')}")
            elif msg_type == "car_state":
                self.get_logger().debug(
                    f"车端状态: block={payload.get('block')} "
                    f"yaw_deg={payload.get('yaw_deg')} "
                    f"phase={payload.get('phase')}")
            else:
                self.get_logger().warning(
                    f"未知车端消息类型 {msg_type!r}（忽略）: {raw[:80]!r}")
            self._blog(_EV_RECV, body=raw[:200])
        except Exception as exc:  # noqa: BLE001  回调兜底：绝不外抛
            self.get_logger().warning(
                f"车端消息处理异常（已忽略）: {type(exc).__name__}: {exc}")

    def destroy_node(self) -> bool:
        self._stop.set()
        self._thread.join(timeout=2)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DzbrainBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
