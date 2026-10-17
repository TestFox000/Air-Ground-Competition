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
    "aim_fire_ready": Topic("/aim/fire_ready", "std_msgs/UInt8", QOS_EVENT,
                            "1=瞄准合格（预测落点在开火框内）—— 与 fire_gate 构成"
                            "**两键开火**：云台补丁只在两者都为 1 时开火", "pub"),
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
    "referee_hp": Topic("/HpAndHitmsg", "std_msgs/UInt8MultiArray", QOS_EVENT,
                        unit="裁判回传帧（★★命中闭环真源：dzjudgment 收帧即发 + 1Hz 心跳）",
                        note="字段=[0]对方HP [1]己方HP [2]被击打方位(0x10=未被击打) "
                             "[3]硬件累计发数 [4]剩余弹药；解析见 io/referee.py",
                        direction="sub"),
    "referee_hit": Topic("/referee/hit", "std_msgs/Int32", QOS_EVENT,
                         "命中确认（★车端**无生产者**：实际命中由 referee_hp 解析；"
                         "本话题仅作未来桥接兜底，且只在从未收到合法帧时才认）", "sub"),
    "referee_ammo": Topic("/referee/ammo", "std_msgs/Int32", QOS_EVENT,
                          "剩余弹药（★同上：兜底源，referee_hp 的 [4] 才是真源）", "sub"),
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
    #: 桥前等待（到达 L10 决策点后、两半场情报未齐时**保持**）。
    #:   语义 = "Mission 让位、车停着等情报"：不在 `tracker.ACTIVE_PHASES` 里
    #:   ⇒ TrackerTask 每拍走它自己的 `_hold()` 分支（`publish_brake()`），
    #:   不需要第二个驱动指令写者（arbitration 见 `tasks/mission.py` 的桥前等待段）。
    HOLD = "HOLD"
    FINISH = "FINISH"
    SAFE_STOP = "SAFE_STOP"

    ALL = (IDLE, DRIVE, APPROACH, STOP, SHOOT, WAIT_SCAN, HOLD, FINISH, SAFE_STOP)


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


# --------------------------------------------------------------------------- #
#  无人机覆盖语义（coverage / slice / scan_id）—— G5-2 车侧契约
# --------------------------------------------------------------------------- #
#: `coverage.mode` 白名单（与机侧 `dzsmartscout/tools/perception/protocol.py`
#: 的 `COVERAGE_MODES` 同源；车侧只做透传/校验，不用它改变判据）。
COVERAGE_MODES: tuple[str, ...] = ("strip", "point", "full")

#: `scan_id` 长度上限（与机侧 `protocol.SCAN_ID_MAX_LEN` 同源）。
SCAN_ID_MAX_LEN = 64


@dataclass(frozen=True)
class SliceInfo:
    """片标识（幂等键的一半）：`{id,index,of,route}`。

    它是"这条 field_verify 属于哪一片"的自描述；与 `scan_id` 合成
    `(scan_id, slice.id)` 去重键 —— BEST_EFFORT 链路上同一片必须周期重发，
    车侧必须幂等（决策层方案 §5.1 第 4 条）。
    """

    id: str = ""
    index: int = 0
    of: int = 0
    route: str = ""

    @property
    def valid(self) -> bool:
        return bool(self.id) and 1 <= self.index <= self.of

    def to_dict(self) -> dict:
        out = {"id": self.id, "index": int(self.index), "of": int(self.of)}
        if self.route:
            out["route"] = self.route
        return out


@dataclass(frozen=True)
class Coverage:
    """一条 `field_verify` 的片级覆盖（负证据语义的唯一载体）。

    ★ 语义边界（与决策层方案 §4.2 / 机侧 `coverage.py` 逐字对齐）：

    - `cells` = 本片**判读范围**（格名，已归一化去重，按 payload 顺序保留）；
    - `complete=True`  ⇒ 片内 `cells` 已判读完毕，**未在 mines/obstacles/targets
      里出现的格 = ABSENT**（这是「扫过且为空」的唯一来源）；
    - `complete=False` ⇒ 只报看得见的，片内其余仍是 **UNKNOWN**，**不得**当 ABSENT；
    - 缺 `complete` 键 ⇒ 按 `False`（UNKNOWN）处理，绝不猜成"空的"。

    `problems` 只是"结构有问题、车侧已降级处理"的诊断，不表示整条报文非法
    （非法形状由 :func:`decode_coverage` 返回 `None`）。
    """

    cells: tuple[str, ...] = ()
    complete: bool = False
    mode: str = "strip"
    scan_id: str = ""
    slice: SliceInfo | None = None
    dropped: int = 0                 # 畸形/越界/重复格名被剔除的个数
    problems: tuple[str, ...] = ()

    @property
    def key(self) -> tuple[str, str] | None:
        """幂等键 `(scan_id, slice.id)`；任一为空 ⇒ `None`（无法去重）。"""
        if self.scan_id and self.slice is not None and self.slice.id:
            return (self.scan_id, self.slice.id)
        return None

    def to_dict(self) -> dict:
        out = {"cells": list(self.cells), "complete": bool(self.complete),
               "mode": self.mode, "n_cells": len(self.cells)}
        if self.scan_id:
            out["scan_id"] = self.scan_id
        if self.slice is not None:
            out["slice"] = self.slice.to_dict()
        if self.dropped:
            out["dropped"] = int(self.dropped)
        if self.problems:
            out["problems"] = list(self.problems)
        return out


def _norm_coverage_cells(items) -> tuple[tuple[str, ...], int]:
    """覆盖格清单归一化：大写去空格、去重、丢越界/非字符串（`dropped` 计数）。

    ★ 与 `drone.py` 对雷/障格名的"不校验是否在地图内"原则**不同**：
      coverage 是**本车网格**的覆盖进度，分母固定 154 格；放进 Z9 这种越界名
      会把比例算到 >1。所以这里按 `geometry.block_to_index` 校验本地网格。
    """
    out: list[str] = []
    seen: set[str] = set()
    dropped = 0
    for item in items:
        if not isinstance(item, str):
            dropped += 1
            continue
        text = item.strip().upper()
        if not text:
            dropped += 1
            continue
        try:
            geo.block_to_index(text)
        except (TypeError, ValueError):
            dropped += 1
            continue
        if text in seen:
            dropped += 1
            continue
        seen.add(text)
        out.append(text)
    return tuple(out), dropped


def decode_slice_info(value) -> SliceInfo | None:
    """`slice:{id,index,of,route?}` → :class:`SliceInfo`；形状非法返回 `None`。"""
    if not isinstance(value, dict):
        return None
    sid = str(value.get("id") or "").strip().upper()
    if not sid or len(sid) > 16:
        return None
    index, of = value.get("index"), value.get("of")
    if isinstance(index, bool) or isinstance(of, bool):
        return None
    if not isinstance(index, int) or not isinstance(of, int):
        return None
    if not (1 <= index <= of):
        return None
    route = str(value.get("route") or "").strip().upper()
    return SliceInfo(id=sid, index=index, of=of, route=route)


def decode_coverage(value, *, scan_id=None, slice_info=None) -> Coverage | None:
    """`field_verify.coverage` → :class:`Coverage`；旧报文（无该键）/形状非法返回 `None`。

    ★ 向后兼容：旧机侧不发 coverage ⇒ 本函数不被调用（或返回 `None`），
      车侧行为与加此特性前**逐位一致**（见 `test_drone.py` ⑬ 的 A/B 证据）。

    ★ 容错优先级（宁可保守，不可把"没扫"当"空的"）：
      1. `cells` 必须是 list/tuple，否则整条 coverage 视为非法（`None`）；
      2. 缺/非 bool 的 `complete` ⇒ `False`（UNKNOWN）+ 记 problems；
      3. `mode` 非法 ⇒ 回退 `strip` + 记 problems；
      4. `scan_id` 超长 ⇒ 丢弃去重键 + 记 problems（不丢数据）；
      5. `slice` 形状非法 ⇒ `slice=None` + 记 problems（不丢数据）。
    """
    if not isinstance(value, dict):
        return None
    raw_cells = value.get("cells")
    if not isinstance(raw_cells, (list, tuple)):
        return None
    cells, dropped = _norm_coverage_cells(raw_cells)

    problems: list[str] = []
    complete_raw = value.get("complete")
    if isinstance(complete_raw, bool):
        complete = complete_raw
    else:
        complete = False
        problems.append("complete 缺失/非 bool → 按 UNKNOWN（不产生 ABSENT）")

    mode = str(value.get("mode") or "").strip().lower()
    if mode not in COVERAGE_MODES:
        if mode:
            problems.append(f"mode 非法({mode!r}) → 按 strip")
        mode = "strip"

    sid = str(scan_id or "").strip()
    if len(sid) > SCAN_ID_MAX_LEN:
        problems.append(f"scan_id 超长({len(sid)}>{SCAN_ID_MAX_LEN}) → 只丢弃去重键")
        sid = ""

    sl = None
    if slice_info is not None:
        sl = decode_slice_info(slice_info)
        if sl is None:
            problems.append("slice 形状非法 → 无去重键（数据仍保留）")

    if not cells:
        problems.append("cells 为空 → 不产生负证据")
    if dropped:
        problems.append(f"剔除 {dropped} 个畸形/越界/重复格名")

    return Coverage(cells=cells, complete=complete, mode=mode, scan_id=sid,
                    slice=sl, dropped=dropped, problems=tuple(problems))


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
