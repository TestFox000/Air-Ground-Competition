"""循迹链路软件在环仿真内核（纯 Python，不依赖 ROS）。

链路：integrate_node（真码）-> /nav/waypoint -> path_executor（reference 真码，
importlib 只读加载）-> pursuitAngle -> DzActuatorSim -> ChassisSim（真值）
-> SerialReaderSim -> /odom/localization -> 回到 path_executor。
无人机侧：DronePoseFeed 按 10Hz 给 integrate_node 喂 0x4002 车位姿帧。

=== 保真边界（先读我）===
本仿真验证的是"契约 + 控制逻辑"，不是动力学参数辨识：
- 底盘为理想 Ackermann 运动学（轴距 L=0.157m，与 path_executor 标定一致），
  无滑移、无电机响应滞后、速度瞬时到位（物理上限 0.33m/s）；
- serial_reader 侧理想定位：IMU 无漂移、编码器无量化误差（留 noise 口，
  默认全零）；
- dzactuator 只复刻"话题 -> 执行量"的译码与双通道模式仲裁
  （dzactuator.cpp 实测契约）；串口 23 字节帧打包、smoothValueAdaptive
  速度平滑不建模；
- 时钟为步进仿真时钟（ros_stub.SIM）：节点里所有 get_clock() 读同一
  全局时间，测试按 0.05s 节拍快进。

能抓的 bug 类：reset 断言偏置导致的画圆、dzactuator 模式仲裁互斥吞命令、
对齐状态机不收敛/超时、路线命令语义错位。抓不到的：动力学参数错、
滑移、通信抖动、串口丢帧。

=== 画圆机理（2026-10-01 事故复现，δ=68.6°）===
车 IMU 开机时朝向 θ_boot（≈90°，+Y），被搬到起点后真实朝向 158.6°：
- serial_reader：yaw_rel = θ_true − θ_boot，位置沿"yaw_rel + π/2"航位推算
  —— θ_boot=90° 时位置积分方向恰好等于真实位移方向；
- path_executor：reset:N10,y 把信念航向锚到 90°（snap_yaw_off），
  信念航向 = yaw_rel + π/2 + snap = θ_true − 68.6°。
=> 信念系里"车朝哪"与"车在往哪挪"差恒定 68.6°（双重误差），纯追踪等效
操控一辆恒定侧滑 68.6° 的车，永远进不了到点圆盘 —— 绕目标画圈。
align 修复的本质：发车前闭环把车真实转到断言朝向（yaw_rel->0），
snap->0，双重误差同时归零。这也解释了为什么必须仿到 serial_reader 的
"yaw_rel + π/2"位置积分这一层，只仿 heading 是看不出画圈的。
"""

from __future__ import annotations

import importlib.util
import math
import random
from pathlib import Path

import ros_stub
from ros_stub import BUS, SIM, Odometry, String, Twist
from std_msgs.msg import Int32, UInt8  # 经 ros_stub 安装的桩模块

from ag_integrate.intel_protocol import (
    CarPose, IntelFrame, build, build_car_pose,
)

# ---------------------------------------------------------------- 常量（真源标定值）
BLOCK = 0.35            # 格边长 m（path_executor BLOCK_SIZE）
WHEELBASE = 0.157       # 底盘轴距 m（path_executor 编码器标定值）
CARL = 0.15             # dzactuator 里 cmd_vel 分支用的轴距宏（#define CARL 0.15）
MAX_SPEED = 0.33        # 底盘物理速度上限 m/s（满字节）
STEER_LIMIT_DEG = 35.0  # 舵机行程：TargetAngle=w+60 钳 [25,95] -> ±35°
ACTIVITY_TIMEOUT = 0.5  # dzactuator ACTIVITY_TIMEOUT：0.5s 无消息判通道不活跃
PURSUIT_LOW_TIMEOUT = 3.0  # dzactuator PURSUIT_LOW_TIMEOUT
MIN_STEP_M = 0.0005     # serial_reader 最小积分步长

_DT = 0.05              # 仿真节拍 50ms（path_executor 控制环标称 20Hz）


def wrap_pi(a: float) -> float:
    """归一化到 (-pi, pi]。"""
    w = (a + math.pi) % (2 * math.pi) - math.pi
    return w if w > -math.pi else math.pi


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _cxx_round(x: float) -> int:
    """C++ round()：half-away-from-zero（Python 自带 round 是银行家舍入）。"""
    return int(math.copysign(math.floor(abs(x) + 0.5), x))


def _quat_from_yaw(yaw: float):
    """平面 yaw -> (qx, qy, qz, qw)。"""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


# ---------------------------------------------------------------- 底盘真值
class ChassisSim:
    """Ackermann 运动学积分器：输入执行量 (v, steer_deg)，输出真位姿。

    yaw_rate = v/L·tan(steer)（与 path_executor 标定 L=0.157 一致）。
    速度截 ±0.33m/s、舵角截 ±35°（物理上限）。
    """

    def __init__(self, x: float, y: float, theta: float) -> None:
        self.x = x
        self.y = y
        self.theta = theta          # 真朝向（场地系，rad，0=东 π/2=北）
        self.yaw_traveled = 0.0     # 有符号累计转角（展开不归一）：画圈检测
        self.dist_traveled = 0.0    # 累计里程 m

    def step(self, v: float, steer_deg: float, dt: float = _DT) -> None:
        v = _clamp(v, -MAX_SPEED, MAX_SPEED)
        steer = math.radians(_clamp(steer_deg, -STEER_LIMIT_DEG, STEER_LIMIT_DEG))
        w = v / WHEELBASE * math.tan(steer)
        th_mid = self.theta + 0.5 * w * dt   # 中点积分，减小大步长误差
        self.x += v * math.cos(th_mid) * dt
        self.y += v * math.sin(th_mid) * dt
        self.theta = wrap_pi(self.theta + w * dt)
        self.yaw_traveled += w * dt
        self.dist_traveled += abs(v) * dt


# ---------------------------------------------------------------- dzactuator 契约
class DzActuatorSim:
    """dzactuator.cpp 实测契约替身（两话题译码 + current_mode_ 仲裁）。

    两段式与真码一致：
    - 消息回调：无条件刷新该通道"最近消息时间"，只有当前模式匹配才改写
      执行量 —— 模式不匹配时消息被静默丢弃，旧执行量一直挂着（真 STM32
      保持最后一帧）；
    - step() 等价 modeTimerCallback（真码 10Hz，仿真每拍评估）：
      活跃 = 0.5s 内有消息；只 pursuit->2，只 cmd_vel->1；都活跃时
      pursuit.linear.y>=0.5->1，否则维持（含 pursuit 低速超 3s 转 2 的分支）。
    """

    def __init__(self) -> None:
        self.mode = 0               # 0=未定 1=cmd_vel(DWA) 2=pursuit(Pursuit)
        self.out_v = 0.0            # 当前执行线速度 m/s（符号=方向）
        self.out_steer_deg = 0.0    # 当前执行舵角 deg
        self._cv_msg: Twist | None = None
        self._cv_t: float | None = None
        self._pu_msg: Twist | None = None
        self._pu_t: float | None = None
        # 真码构造时把 last_pursuit_high_time_ 初始化为当前时间（非 0）
        self._pu_high_t: float | None = None

    def on_cmd_vel(self, msg: Twist, t: float) -> None:
        if self._pu_high_t is None:
            self._pu_high_t = t
        self._cv_msg, self._cv_t = msg, t
        if self.mode != 1:
            return
        v, w = msg.linear.x, msg.angular.z
        if v == 0:
            # v==0 -> TargetAngle=0（舵机回中）、速度 0，车不动
            self.out_v, self.out_steer_deg = 0.0, 0.0
        else:
            # w 语义是 yaw 角速度 rad/s：TargetAngle=round(atan(CARL*w/v)*57.3)
            steer = _cxx_round(math.degrees(math.atan(CARL * w / v)))
            self.out_steer_deg = _clamp(steer, -STEER_LIMIT_DEG, STEER_LIMIT_DEG)
            # TargetSpeed=|v*23/0.33| 截 ±255；字节->物理速度的上限由底盘截
            self.out_v = v

    def on_pursuit(self, msg: Twist, t: float) -> None:
        if self._pu_high_t is None:
            self._pu_high_t = t
        self._pu_msg, self._pu_t = msg, t
        if msg.linear.y >= 0.5:
            self._pu_high_t = t
        if self.mode != 2:
            return
        # angular.z 直接是舵机角度（度），±35 打死
        self.out_steer_deg = _clamp(msg.angular.z, -STEER_LIMIT_DEG, STEER_LIMIT_DEG)
        self.out_v = msg.linear.x

    def step(self, t: float) -> None:
        """模式仲裁（modeTimerCallback 等价）。"""
        if self._pu_high_t is None:
            self._pu_high_t = t
        dwa = self._cv_t is not None and (t - self._cv_t) < ACTIVITY_TIMEOUT
        pur = self._pu_t is not None and (t - self._pu_t) < ACTIVITY_TIMEOUT
        if not dwa and not pur:
            return                      # 都不活跃：维持现状（含初始 mode 0）
        if pur and not dwa:
            self.mode = 2
            return
        if dwa and not pur:
            self.mode = 1
            return
        # 都活跃：pursuit.linear.y>=0.5 -> 切 cmd_vel；否则维持，
        # 仅当 pursuit 低值超 3s 且 pursuit 速度更大时才让出给 pursuit
        if self._pu_msg.linear.y >= 0.5:
            self.mode = 1
            return
        if (self.mode == 1
                and self._pu_msg.linear.x > self._cv_msg.linear.x
                and (t - self._pu_high_t) >= PURSUIT_LOW_TIMEOUT):
            self.mode = 2


# ---------------------------------------------------------------- serial_reader 语义
class SerialReaderSim:
    """serial_reader 语义替身：真位姿 -> /odom/localization（block 单位）。

    按任务书语义实现，不订阅 reset（位置/航向重锚由 path_executor 的
    snap 完成）：
    - yaw_rel = θ_true − θ_boot（IMU 相对开机朝向的转角），四元数只载 yaw_rel；
    - 位置 = "步长 × 信念航向" 航位推算，信念航向 = yaw_rel + π/2
      （内部以米积分，发布时 ÷0.35 转 block）。
    默认理想无噪声；noise_* 参数口留着做灵敏度实验。
    """

    def __init__(self, theta_boot: float, start_block=(1.5, 0.5),
                 noise_yaw_rad: float = 0.0, seed: int = 1) -> None:
        self.theta_boot = theta_boot
        self.dr_x_m = start_block[0] * BLOCK   # 约定起点 N10 中心 (1.5, 0.5) block
        self.dr_y_m = start_block[1] * BLOCK
        self.noise_yaw_rad = noise_yaw_rad
        self._rng = random.Random(seed)
        self._yaw_rel = 0.0

    def yaw_rel(self, theta_true: float) -> float:
        return wrap_pi(theta_true - self.theta_boot)

    def step(self, v: float, dt: float, theta_true: float) -> None:
        self._yaw_rel = self.yaw_rel(theta_true)
        d = abs(v) * dt     # 真码步长取自编码器位移模长（恒非负；无倒车场景）
        if d < MIN_STEP_M:
            return
        h = wrap_pi(self._yaw_rel + math.pi / 2)   # 信念航向（raw_map_yaw）
        self.dr_x_m += d * math.cos(h)
        self.dr_y_m += d * math.sin(h)

    def make_odom(self) -> Odometry:
        msg = Odometry()
        msg.pose.pose.position.x = self.dr_x_m / BLOCK
        msg.pose.pose.position.y = self.dr_y_m / BLOCK
        yaw = self._yaw_rel
        if self.noise_yaw_rad:
            yaw += self._rng.uniform(-self.noise_yaw_rad, self.noise_yaw_rad)
        qx, qy, qz, qw = _quat_from_yaw(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        return msg


# ---------------------------------------------------------------- 无人机车位姿帧源
class DronePoseFeed:
    """10Hz 0x4002 车位姿帧源：延迟线（默认 0.2s）+ 均匀噪声（默认 ±1°）。

    延迟线模拟无人机检测链路的处理/传输滞后：t 时刻发出的观测值来自
    t-delay 的采样。噪声用固定 seed 的均匀分布，保证测试可复现。
    """

    def __init__(self, delay: float = 0.2, noise_deg: float = 1.0,
                 period: float = 0.1, seed: int = 42) -> None:
        self.delay = delay
        self.noise_deg = noise_deg
        self.period = period
        self._rng = random.Random(seed)
        self._buf: list[tuple[float, float]] = []
        self._next = 0.0

    def emit(self, t: float, x: float, y: float, yaw: float) -> bytes | None:
        """每拍缓存真值；到 10Hz 节拍才产出一帧（采样自 t-delay）。"""
        self._buf.append((t, yaw))
        if t + 1e-9 < self._next:
            return None
        self._next = t + self.period
        tq = t - self.delay
        sample = self._buf[0][1]
        for ts, yw in self._buf:
            if ts <= tq + 1e-9:
                sample = yw
            else:
                break
        # 裁剪 2s 以前的老样本，缓冲不膨胀
        self._buf = [(ts, yw) for ts, yw in self._buf if ts >= tq - 2.0]
        yaw_obs = sample + math.radians(
            self._rng.uniform(-self.noise_deg, self.noise_deg))
        return build_car_pose(CarPose(int(t * 1000), x, y, yaw_obs))


# ---------------------------------------------------------------- 云台/射击握手
class GimbalSim:
    """云台自瞄/射击握手替身（挂在 rig 节点上，走总线）。

    - /nav/scan_request -> +0.2s 发 aim=1(SCAN)，+0.5s 回 aim=0(IDLE)，
      正好踩中 path_executor 扫描停站放行条件②（见过 SCAN 后回 IDLE）；
    - path_executor 进入 shoot_wait -> +0.5s 补 10 发 /shoot_count。
    """

    def __init__(self, rig: "ros_stub.FakeNode", pe) -> None:
        self._pe = pe
        self._aim_pub = rig.create_publisher(Int32, "/aim/status", 10)
        self._shoot_pub = rig.create_publisher(Int32, "/shoot_count", 10)
        self._events: list[tuple[float, object]] = []
        self._shoot_pending = False
        rig.create_subscription(UInt8, "/nav/scan_request", self._on_scan_req, 10)

    def _on_scan_req(self, _msg) -> None:
        t = SIM.t
        self._events.append((t + 0.2, lambda: self._pub(self._aim_pub, 1)))
        self._events.append((t + 0.5, lambda: self._pub(self._aim_pub, 0)))

    @staticmethod
    def _pub(pub, data: int) -> None:
        pub.publish(Int32(data=data))

    def tick(self, t: float) -> None:
        if self._pe.shoot_wait and not self._shoot_pending:
            self._shoot_pending = True
            n = self._pe.shoot_count_at_arrival + self._pe.SHOOT_COUNT_TARGET
            self._events.append((t + 0.5, lambda: self._pub(self._shoot_pub, n)))
        if not self._pe.shoot_wait:
            self._shoot_pending = False
        due = [e for e in self._events if e[0] <= t]
        self._events = [e for e in self._events if e[0] > t]
        for _tf, fn in due:
            fn()


# ---------------------------------------------------------------- reference 真码加载
_PE_PATH = (Path(__file__).resolve().parents[1]
            / "reference" / "car_src" / "car_localization_full"
            / "car_localization" / "car_localization" / "path_executor.py")


def load_path_executor_module():
    """从 reference 只读加载真 path_executor；模块级 import 由 ros_stub 满足。

    每次调用重新 exec_module，测试间互不留状态。
    """
    spec = importlib.util.spec_from_file_location("ref_path_executor", _PE_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# ---------------------------------------------------------------- 场景容器
class TrackingWorld:
    """把底盘/执行器/serial_reader/无人机馈源与 ros_stub 总线、仿真时钟
    串起来的场景容器。

    两阶段启动语义（对齐 tools/car_match_bringup.sh）：
    - dzactuator（执行器）从 t=0 在线 —— align 阶段的 cmd_vel 靠它执行；
    - 车端栈（serial_reader + path_executor）由 start_car_stack() 拉起 ——
      此时 /nav/waypoint 才出现订阅者，integrate_node 的闸门才放行。
    """

    DT = _DT

    def __init__(self, start_cell: str = "N10", true_yaw_deg: float = 158.6,
                 theta_boot_deg: float = 90.0, drone_delay: float = 0.2,
                 drone_noise_deg: float = 1.0, seed: int = 42) -> None:
        BUS.reset()
        SIM.enable(0.0)
        pe_mod = load_path_executor_module()
        self._pe_cls = pe_mod.PathExecutorNode
        self.block_to_meters = pe_mod.block_to_meters

        x0, y0 = self.block_to_meters(start_cell)
        self.start_cell = start_cell
        self.chassis = ChassisSim(x0, y0, math.radians(true_yaw_deg))
        self.actuator = DzActuatorSim()
        self.theta_boot = math.radians(theta_boot_deg)
        self.drone: DronePoseFeed | None = DronePoseFeed(
            delay=drone_delay, noise_deg=drone_noise_deg, seed=seed)

        # rig 节点：承载仿真侧所有总线 pub/sub（执行器订阅 t=0 就在）
        self.rig = ros_stub.FakeNode("sim_rig")
        self.rig.create_subscription(
            Twist, "/cmd_vel", lambda m: self.actuator.on_cmd_vel(m, SIM.t), 10)
        self.rig.create_subscription(
            Twist, "pursuitAngle", lambda m: self.actuator.on_pursuit(m, SIM.t), 10)
        # 直发 /nav/waypoint 用（pivot 场景不经 integrate_node）
        self.waypoint_pub = self.rig.create_publisher(String, "/nav/waypoint", 10)

        # 车端栈（两阶段启动，start_car_stack 拉起）
        self.serial: SerialReaderSim | None = None
        self.pe = None
        self.gimbal: GimbalSim | None = None
        self._odom_pub = None

        # integrate_node 侧
        self.ig = None
        self._intel_objects = None
        self._next_ig = 0.0
        self._next_intel = 0.0

        # 指标
        self.t = 0.0
        self.steps = 0
        self.goal_xy: tuple[float, float] | None = None
        self.pivot_seen = False
        self.align_done_t: float | None = None
        self.route_pub_t: float | None = None
        self.arrived_t: float | None = None

    # ---- 装配 ----
    def attach_integrate(self, node, intel_objects) -> None:
        """挂上真 integrate_node，并设定周期重喂的情报帧内容。"""
        self.ig = node
        self._intel_objects = tuple(intel_objects)
        self.goal_xy = self.block_to_meters(node.goal_cell)

    def start_car_stack(self) -> None:
        """两阶段启动的第二阶段：serial_reader + path_executor 上线。"""
        assert self.serial is None, "车端栈只能起一次"
        self.serial = SerialReaderSim(self.theta_boot)
        self._odom_pub = self.rig.create_publisher(
            Odometry, "/odom/localization", 10)
        self.pe = self._pe_cls()
        self.gimbal = GimbalSim(self.rig, self.pe)

    # ---- 主循环 ----
    def step(self) -> None:
        t = SIM.advance(self.DT)
        self.t = t
        # --- 车端栈 20Hz：定位 -> 控制环 -> 云台握手 ---
        # 注意顺序：定位帧必须先于 integrate_node 的决策拍 ——
        # 真车两阶段启动时 serial_reader 的高频定位流本就先于任何
        # reset 命令到达 path_executor；若让 reset 抢在首帧定位前，
        # snap 会按 raw=0 算出一格双倍的偏置（与实车不符）。
        if self.serial is not None:
            self._odom_pub.publish(self.serial.make_odom())
            self.pe.control_loop()
            self.gimbal.tick(t)
        # --- 10Hz：无人机 0x4002 + 情报帧 1Hz 重喂 + integrate_node 决策 ---
        if self.ig is not None and t >= self._next_ig - 1e-9:
            self._next_ig = t + 0.1
            if self.drone is not None:
                data = self.drone.emit(
                    t, self.chassis.x, self.chassis.y, self.chassis.theta)
                if data is not None:
                    self.ig._process_packet(data)
            if self._intel_objects is not None and t >= self._next_intel - 1e-9:
                self._next_intel = t + 1.0
                frame = IntelFrame(int(t * 1000), self._intel_objects)
                self.ig._process_packet(build(frame))
            self.ig.tick()
        # --- 执行器仲裁 -> 底盘积分 -> DR 积分（本拍执行量） ---
        self.actuator.step(t)
        v, steer = self.actuator.out_v, self.actuator.out_steer_deg
        self.chassis.step(v, steer, self.DT)
        if self.serial is not None:
            self.serial.step(v, self.DT, self.chassis.theta)
        # --- 指标 ---
        self.steps += 1
        if self.pe is not None:
            if self.pe.corner_mode:
                self.pivot_seen = True
            if self.pe.arrived and self.arrived_t is None:
                self.arrived_t = t
        if self.ig is not None:
            if self.align_done_t is None and self.ig._align_state == "aligned":
                self.align_done_t = t
            if (self.route_pub_t is None
                    and len(self.ig.topic(self.ig.waypoint_topic).published) >= 2):
                self.route_pub_t = t

    def run_until(self, cond, budget_s: float) -> bool:
        """快进仿真直到 cond() 为真或预算耗尽；返回 cond() 最终值。"""
        end = self.t + budget_s
        while not cond() and self.t < end - 1e-12:
            self.step()
        return bool(cond())

    # ---- 查询 ----
    def dist_to_goal(self) -> float:
        assert self.goal_xy is not None
        return math.hypot(self.chassis.x - self.goal_xy[0],
                          self.chassis.y - self.goal_xy[1])

    def yaw_err_to(self, deg: float) -> float:
        """真朝向与目标角度（度）的短弧差（度）。"""
        return math.degrees(wrap_pi(self.chassis.theta - math.radians(deg)))

    def close(self) -> None:
        if self.ig is not None:
            self.ig.destroy_node()
        if self.pe is not None:
            self.pe.destroy_node()
