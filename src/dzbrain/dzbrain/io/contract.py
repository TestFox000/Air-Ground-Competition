# -*- coding: utf-8 -*-
r"""ROS 契约集中层（**纯数据，无 rclpy 依赖，可本地单测**）。

为什么单独一层：旧代码里最容易出错的不是算法，而是**单位与坐标系约定**散落在各处：
  · `/odom/localization` 的 position 是【格】不是米（readme【一】3 明确警告"别搞混"）
  · `pursuitAngle.angular.z` 是【度】舵角，而 `cmd_vel.angular.z` 是【弧度/秒】
  · 四元数里的 yaw 是原始车体角，要 +π/2 才是地图 yaw
这些约定一旦写错，表现是"车莫名跑偏"而不是报错。所以把它们钉在这一处，
`tasks/` 只允许通过本层访问话题（禁止再手写话题名/单位换算）。

`io/ros_io.py` 负责把这些定义接到 rclpy 上（编不出来也不会影响本文件的单测）。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from ..core import geometry as geo

# --------------------------------------------------------------------------- #
#  QoS 契约（改错会导致丢消息或收不到，务必与订阅端一致）
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class QosSpec:
    depth: int = 10
    reliability: str = "reliable"      # reliable | best_effort
    durability: str = "volatile"       # volatile | transient_local

    def describe(self) -> str:
        return (f"KEEP_LAST({self.depth})/{self.reliability.upper()}/"
                f"{self.durability.upper()}")


#: ★ 底盘对 `pursuitAngle` 的订阅是 depth 1 / BEST_EFFORT（见 survey §2.6、L776-782）
#:   换成 RELIABLE 会让底盘收到慢/旧指令，表现为"车一顿一顿"。
QOS_PURSUIT_ANGLE = QosSpec(depth=1, reliability="best_effort")
QOS_ODOM = QosSpec(depth=10, reliability="best_effort")
QOS_SENSOR_FRAME = QosSpec(depth=10, reliability="best_effort")
QOS_EVENT = QosSpec(depth=10, reliability="reliable")
QOS_LINK = QosSpec(depth=10, reliability="best_effort")     # 无人机链路允许丢旧帧


# --------------------------------------------------------------------------- #
#  话题表
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Topic:
    name: str
    msg_type: str
    qos: QosSpec = QOS_EVENT
    unit: str = ""
    note: str = ""
    direction: str = "sub"             # sub | pub | both


#: 与**既有 C++/底盘节点**的接口（不可退让，改动需同步车端）
TOPICS: dict[str, Topic] = {
    # ---------------- 输入（车 → 我们不，是别人发给我们） ----------------
    "odom": Topic("/odom/localization", "nav_msgs/Odometry", QOS_ODOM,
                  "格（×0.35=米）；yaw=原始车体角(+π/2=地图航向)", "sub",
                  "★ 发布者是 serial_reader（车端 install 副本）"),
    "imu": Topic("/imu_data", "sensor_msgs/Imu", QOS_SENSOR_FRAME, "rad/s, m/s²", "sub"),
    "offset_center": Topic("/offset_center", "std_msgs/Int32MultiArray", QOS_EVENT,
                           "px / 类别 / 框宽高", "sub",
                           "字段=[0]dx [1]dy [2]cls+1 [3]框宽 [4]框高 [5..7]关键点"),
    "aim_status": Topic("/aim/status", "std_msgs/Int32", QOS_EVENT, "0=IDLE 1=SCAN 2=TRACK", "sub"),
    "shoot_count": Topic("/shoot_count", "std_msgs/Int32", QOS_EVENT, "累计发数", "sub"),
    "manual_override": Topic("/nav/manual_override", "std_msgs/UInt8", QOS_EVENT,
                             "1=遥控接管底盘", "sub"),
    "manual_gimbal": Topic("nav/manual_gimbal", "std_msgs/UInt8", QOS_EVENT,
                           "1=遥控接管云台（注意：历史话题名没有前导斜杠）", "sub"),
    "waypoint_cmd": Topic("/nav/waypoint", "std_msgs/String", QOS_EVENT,
                          "外部命令：路线/复位/单点", "sub"),

    # ---------------- 输出（我们 → 别人） ----------------
    "pursuit_angle": Topic("pursuitAngle", "geometry_msgs/Twist", QOS_PURSUIT_ANGLE,
                           "linear.x=m/s；★angular.z=舵角【度】(正=左转)", "pub",
                           "★ 零 Twist = 刹车；50Hz 保活（底盘 1s 看门狗）"),
    "scan_request": Topic("/nav/scan_request", "std_msgs/UInt8", QOS_EVENT,
                          "0=停扫 1=单趟 2=持续", "pub"),
    "aim_fire_gate": Topic("/aim/fire_gate", "std_msgs/UInt8", QOS_EVENT,
                           "1=允许开火（★新增，修旧版死线）", "pub"),
    "stop_signal": Topic("/move_base/stop_signal", "std_msgs/UInt8", QOS_EVENT,
                         "1=允许云台扫描+射击（历史话题，云台侧原先无人订阅）", "pub"),
    "motion_state": Topic("/nav/motion_state", "std_msgs/UInt8", QOS_EVENT,
                          "0 前进 / 1 倒车（仅变化时发）", "pub"),
    "voice_switch": Topic("/voice_switch", "std_msgs/UInt8", QOS_EVENT, "语音 ID", "pub"),

    # ---------------- 新版本新增（与无人机/裁判/云台补丁对齐） ----------------
    "drone_in": Topic("/dzlink/drone_to_car", "std_msgs/String", QOS_LINK,
                      "JSON 情报：桥洞/靶区/雷障核对/心跳", "sub"),
    "drone_out": Topic("/dzlink/car_to_drone", "std_msgs/String", QOS_LINK,
                       "JSON：车状态/ack", "pub"),
    "referee_hit": Topic("/referee/hit", "std_msgs/Int32", QOS_EVENT,
                         "命中确认（来自 dzjudgment 裁判回传帧解析）", "sub"),
    "referee_ammo": Topic("/referee/ammo", "std_msgs/Int32", QOS_EVENT,
                          "剩余弹药（裁判回传）", "sub"),
    "aim_lead": Topic("/aim/lead_x", "std_msgs/Float32", QOS_EVENT,
                      "★提前量（px，云台补丁订阅；旧版无此能力）", "pub"),
}


def topic(key: str) -> Topic:
    """按键取话题定义（拼错即报错，避免手写话题名）。"""
    if key not in TOPICS:
        raise KeyError(f"未登记的话题键: {key!r}（可选: {', '.join(sorted(TOPICS))}）")
    return TOPICS[key]


# --------------------------------------------------------------------------- #
#  单位换算（只在这里做，别处不许再写）
# --------------------------------------------------------------------------- #
def odom_to_pose(pos_x_units: float, pos_y_units: float,
                 quat: tuple[float, float, float, float]) -> geo.Pose:
    """`/odom/localization` 一帧 → 米制地图 Pose。

    ★ 两道容易错的换算，集中在此：
      ① position 是**格单位** → 乘 0.35 得米
      ② 四元数带的是**原始车体 yaw** → 加 +π/2 得地图 yaw
    """
    x, y = geo.units_to_meters(pos_x_units, pos_y_units)
    raw_yaw = geo.quaternion_to_yaw(*quat)
    return geo.Pose(x, y, geo.normalize_angle(raw_yaw + geo.RAW_YAW_TO_MAP_YAW_OFFSET))


def pose_to_pursuit_twist(linear_x_mps: float, steer_deg: float) -> dict[str, float]:
    """我们的控制量 → `pursuitAngle` 的 Twist 语义（返回 dict，便于本地测试）。

    ★ angular.z 是**度**舵角，不是角速度。底盘侧会 `servo = angular.z + 60`。
    """
    return {"linear_x": float(linear_x_mps), "angular_z_deg": float(steer_deg)}


def clamp_steer_for_chassis(steer_deg: float, cfg) -> float:
    """按底盘实际能力再夹一次（底盘 clamp 到 servo∈[25,95] 即舵角 ∈[-35,+35]）。"""
    limit = min(cfg.get("pursuit.max_steer_deg"), 35.0)
    return max(-limit, min(limit, float(steer_deg)))


def chassis_speed_counts(linear_x_mps: float) -> int:
    """底盘把 m/s 量化成整数计数：|v·23/0.33| 截断（1 计数 ≈ 0.0144 m/s）。

    返回**下位机实际执行的计数**，供补偿项参考（旧代码 0.15 → 10，少 4.3%）。
    """
    return int(abs(linear_x_mps) * 23.0 / 0.33)


# --------------------------------------------------------------------------- #
#  指令安全（"零 Twist = 刹车"是硬契约）
# --------------------------------------------------------------------------- #
ZERO_COMMAND: dict[str, Any] = {"linear_x": 0.0, "angular_z_deg": 0.0}
ZERO_COMMAND["note"] = "零 Twist = 刹车；底盘 1s 无指令也会强制停（最后一道兜底）"


def is_brake_command(cmd: dict[str, Any]) -> bool:
    return abs(cmd.get("linear_x", 0.0)) < 1e-9 and abs(cmd.get("angular_z_deg", 0.0)) < 1e-9


# --------------------------------------------------------------------------- #
#  统一枚举（供状态机与日志共用，避免各处写魔法数字）
# --------------------------------------------------------------------------- #
class AimStatus:
    IDLE = 0
    SCAN = 1
    TRACK = 2

    NAMES = {0: "IDLE", 1: "SCAN", 2: "TRACK"}

    @classmethod
    def name(cls, value: int) -> str:
        return cls.NAMES.get(int(value), f"UNKNOWN({value})")


class MissionPhase:
    IDLE = "IDLE"
    DRIVE = "DRIVE"
    APPROACH = "APPROACH"
    STOP = "STOP"
    SHOOT = "SHOOT"
    WAIT_SCAN = "WAIT_SCAN"
    FINISH = "FINISH"
    SAFE_STOP = "SAFE_STOP"

    ALL = (IDLE, DRIVE, APPROACH, STOP, SHOOT, WAIT_SCAN, FINISH, SAFE_STOP)


class ShotClass:
    """`/offset_center[2]` 的类别编码（旧代码：0=无目标，1=蓝，2=红）。"""

    NONE = 0
    BLUE = 1
    RED = 2


# --------------------------------------------------------------------------- #
#  报文解码（纯函数：本地可测，且**带长度守卫**）
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class OffsetObservation:
    """一帧视觉观测。字段含义来自 det_node（survey_gimbal_fire §6）。"""

    dx: int = 0            # 靶心相对画面中心水平偏移（px）
    dy: int = 0            # 垂直偏移（px）
    cls: int = 0           # 0 无目标 / 1 蓝 / 2 红
    box_w: int = 0         # 框宽（px）→ 距离代理
    box_h: int = 0         # 框高（px）
    ts: float = 0.0

    @property
    def has_target(self) -> bool:
        return self.cls in (ShotClass.BLUE, ShotClass.RED)

    @property
    def bearing_px(self) -> int:
        return self.dx


def decode_offset_center(data, ts: float = 0.0) -> OffsetObservation | None:
    """`std_msgs/Int32MultiArray` 的 data → OffsetObservation。

    ★ 长度守卫：旧代码 `dzrknn_dataprocess.cpp` 的 `offset_center` 回调曾因
      `size < 5` 越界读未定义内存（survey_chassis_vision 记录为健壮性缺口）。
      这里不足 5 个字段一律返回 None（视为"无观测"），绝不猜。
    """
    if data is None:
        return None
    try:
        vals = list(data)
    except TypeError:
        return None
    if len(vals) < 5:
        return None
    return OffsetObservation(dx=int(vals[0]), dy=int(vals[1]), cls=int(vals[2]),
                             box_w=int(vals[3]), box_h=int(vals[4]), ts=ts)


def decode_aim_status(value) -> str:
    """`/aim/status`(Int32) → 可读状态名。"""
    try:
        return AimStatus.name(int(value))
    except (TypeError, ValueError):
        return "UNKNOWN"


@dataclass(frozen=True)
class ContractSummary:
    """给启动横幅/文档用的一页纸摘要。"""

    topics: tuple[str, ...] = field(default_factory=lambda: tuple(sorted(TOPICS)))

    def render(self) -> list[str]:
        out = ["ROS 契约（新版本）:"]
        for key in sorted(TOPICS):
            t = TOPICS[key]
            out.append(f"  [{t.direction:3}] {t.name:26} {t.msg_type:26} {t.qos.describe()}"
                       + (f"  {t.unit}" if t.unit else ""))
        return out
