"""开火闸门：把 `det_node` 的检测结果按敌我过滤后再喂给官方的瞄准节点。

## 为什么必须有这个节点

车端源码实测（`path_executor.py` 与全仓 grep）：

    `/move_base/stop_signal`（注释写着 "1 = car stopped, allow gimbal scan + shooting"）
    **只有 path_executor 一个发布者，没有任何订阅者** —— 那道"停车才开火"的闸门是**死代码**。

而 `dzrknn_dataprocess` 的开火条件里**没有敌我判断**：

    const bool is_target_cls = (data[2] == 1) || (data[2] == 2);   // 蓝 或 红 都算靶
    ...
    if (in_fire_zone && aim_ready && ammo_ok && cooldown_ok && target_confirmed) fire();

结论：**只要 `dzrknn_dataprocess` 在跑，它就会对视野内任何靶开火，包括红面（友方）**。
规则对误击友方的判罚是每个计数 −5、最多 −15，而且会把 10 发弹药全部浪费掉。

## 怎么解决（不改官方代码）

`det_node` 发 `/offset_center`（`[dx, dy, cls, w, h, kx, ky, kconf]`，其中 **cls: 1=蓝(敌) 2=红(友) 0=无目标**）。
我们在这条链路上插一层：

    det_node ─/offset_center─▶ aim_guard_node ─/offset_center_guarded─▶ dzrknn_dataprocess

启官方瞄准节点时做话题重映射即可（见 `ag_bringup/launch/gimbal_guarded.launch.py`）：

    ros2 run dzrknn_dataprocess dzactuator_node --ros-args -r offset_center:=/offset_center_guarded

红面（友方）会被替换成**"无目标"帧**（`data` 全 0），官方节点于是回到扫描状态、绝不锁定、绝不发射；
同时它还会因为 `vision_timeout` 之外的输入持续存在而不会卡死在跟踪态。

## 两道闸

1. `allow_classes`（默认只放行蓝靶）：类别白名单
2. `enable_fire`（默认 True）：总闸，现场可用 `ros2 param set` 动态关掉

任一不满足就转发"无目标"帧 —— 对齐车端"看到 0 就不动作"的既有行为。
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

from ag_core.blackbox import AIM_BLOCK_WHY, EV, BlackBox

MSG_LEN = 8           # [dx, dy, cls, w, h, kx, ky, kconf*100]
IDX_CLS = 2


class AimGuardNode(Node):
    def __init__(self) -> None:
        super().__init__("aim_guard_node")

        self.declare_parameter("input_topic", "/offset_center")
        self.declare_parameter("output_topic", "/offset_center_guarded")
        # 注意：这里用的是 **det_node 输出的类别号**（已 +1），不是训练时的 0/1
        self.declare_parameter("class_blue", 1)     # 1 = 蓝面 = 敌方 = 可打
        self.declare_parameter("class_red", 2)      # 2 = 红面 = 友方 = 禁止
        self.declare_parameter("allow_classes", "1")   # 类别白名单，逗号分隔；空=全部放行
        self.declare_parameter("enable_fire", True)    # 总闸
        self.declare_parameter("publish_blocked_log_period_s", 2.0)
        # ---- 黑匣子（现场复盘：这一局到底拦了什么、放行了什么）----
        self.declare_parameter("log_enabled", True)
        self.declare_parameter("log_dir", "")
        self.declare_parameter("log_level", "info")

        p = lambda k: self.get_parameter(k).value  # noqa: E731

        raw = str(p("allow_classes")).strip()
        self.allow = {int(v) for v in raw.split(",") if v.strip() != ""} if raw else None
        # 两个类别号都要存：`cb` 里判断"被拦的是敌是友"时要用，
        # 早先只存了 class_red，白名单一旦排除蓝面就会 AttributeError 崩在回调里。
        self.class_blue = int(p("class_blue"))
        self.class_red = int(p("class_red"))
        self.enable_fire = bool(p("enable_fire"))
        self.log_period = float(p("publish_blocked_log_period_s"))

        self.pub = self.create_publisher(Int32MultiArray, str(p("output_topic")), 1)
        self.create_subscription(Int32MultiArray, str(p("input_topic")), self.cb, 1)

        self.blocked = 0
        self.passed = 0
        self.blank = 0
        # 日志限流用单调时钟即可（不需要 ROS 时间，也便于单测）。
        # 初值 0 是故意的：**第一次拦截必定打印**，现场才能确认闸门在工作。
        self._last_log_t = 0.0

        self.bb = BlackBox.from_params(self)
        self.bb.log(EV.NODE_ARGS, allow_classes=sorted(self.allow) if self.allow else None,
                    class_blue=int(p("class_blue")), class_red=self.class_red,
                    enable_fire=self.enable_fire, input_topic=str(p("input_topic")),
                    output_topic=str(p("output_topic")))

        self.get_logger().info(
            f"aim_guard 就绪: {p('input_topic')} -> {p('output_topic')} | "
            f"白名单={sorted(self.allow) if self.allow else '全部'} "
            f"红面类={self.class_red} 总闸={'开' if self.enable_fire else '关'}"
        )

    # ---------------- 主逻辑 ----------------
    def cb(self, msg: Int32MultiArray) -> None:
        data = list(msg.data) if msg.data else []
        if len(data) < MSG_LEN:
            # 长度不对就当"无目标"，绝不放行不确定的数据
            if self.bb.throttled("short_frame", self.log_period):
                self.bb.log(EV.AIM_BLOCK, level="warn", why=AIM_BLOCK_WHY.SHORT_FRAME,
                            cls=None, length=len(data),
                            blocked_total=self.blocked, passed_total=self.passed)
            self._publish_blank()
            return

        cls = int(data[IDX_CLS])

        if cls == 0:
            self.blank += 1
            self._sample_pass(cls)
            self._publish(data)          # 本来就无目标，原样转发
            return

        if not self.enable_fire:
            self.blocked += 1
            self._log_blocked(f"总闸关闭时收到 cls={cls}", AIM_BLOCK_WHY.GATE_OFF, cls)
            self._publish_blank()
            return

        if self.allow is not None and cls not in self.allow:
            self.blocked += 1
            if cls == self.class_red:
                tag = "红面（友方）"
            elif cls == self.class_blue:
                tag = "蓝面（敌方）"
            else:
                tag = f"未知类别 {cls}"
            self._log_blocked(f"{tag} 不在白名单 {sorted(self.allow)}，已拦截",
                              AIM_BLOCK_WHY.NOT_ALLOWED, cls)
            self._publish_blank()
            return

        self.passed += 1
        self._sample_pass(cls)
        self._publish(data)

    # ---------------- 输出 ----------------
    def _publish(self, data) -> None:
        out = Int32MultiArray()
        out.data = [int(v) for v in data]
        self.pub.publish(out)

    def _publish_blank(self) -> None:
        """转发"无目标"帧：官方瞄准节点看到 cls=0 会回到扫描/停止跟踪。"""
        self._publish([0] * MSG_LEN)

    def _log_blocked(self, why: str, code: str, cls: int) -> None:
        now = time.monotonic()
        if now - self._last_log_t >= self.log_period:
            self._last_log_t = now
            self.get_logger().warning(
                f"{why}（累计拦截 {self.blocked} 次 / 放行 {self.passed} 次）"
            )
            # 结构化那条与文本警告同一时刻，复盘时能对齐
            self.bb.log(EV.AIM_BLOCK, level="warn", why=code, cls=cls,
                        blocked_total=self.blocked, passed_total=self.passed)

    def _sample_pass(self, cls: int) -> None:
        """放行是**持续状态**，按周期采样记一条即可。

        这个回调跟着检测帧跑（30fps 量级），逐帧写会把盘写满、
        也会把时间轴淹掉，反而看不见拦截事件。
        """
        if self.bb.throttled("aim_pass", self.log_period):
            self.bb.log(EV.AIM_PASS, cls=cls, blocked_total=self.blocked,
                        passed_total=self.passed)


def main(args=None):
    rclpy.init(args=args)
    node = AimGuardNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
