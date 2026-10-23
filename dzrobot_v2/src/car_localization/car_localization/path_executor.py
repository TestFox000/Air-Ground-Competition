# -*- coding: utf-8 -*-
"""
Path tracking executor node (path_executor) - reads path and drives the car along it.

Parses route commands (e.g. 'G8 A10 D2* E2') or path files into a metric waypoint
sequence, then uses the Pure Pursuit algorithm to publish pursuitAngle (Twist)
control commands at 50Hz, stopping at mid-route shoot points and the final point
to notify the gimbal to scan and fire.

Path file format (per line):
  x, y, z, qx, qy, qz, qw

where x, y are in meters and qx~qw are the quaternion attitude.

Route commands support a '*' suffix for mid-route shoot stops, e.g. 'N10 L10 D2* E2 N5'
stops at D2, fires 10 pellets, then continues.

Usage:
  python3 path_executor.py [path_file]

=== Complete data flow (who feeds this node, and who it feeds, incl. serial & STM32) ===

  -- Upstream localization chain (where data comes from) ---
  [STM32] --encoder/IMU data--> serial /dev/ttyUSB0 @115200bps
           (serial port opened by dzactuator: params serial_port_name/serial_baud_rate)
  dzactuator: parses odom raw data from serial --/odom(Odometry)--> serial_reader
  serial_reader: heading from IMU (/imu_data, 2s timeout fallback to odom), position
                 via "step x map heading" dead reckoning --/odom/localization(block units)--> this node
  send_waypoint.py --/nav/waypoint(String route command)--> this node
  dzrknn_dataprocess --/shoot_count(Int32 cumulative shot count)--> this node

  -- Downstream control chain (where Twist goes, how it becomes motor speed) ---
  this node --pursuitAngle(Twist)--> dzactuator's callback_movebase_angle
        linear.x  = target linear velocity m/s  (0..~0.3)
        angular.z = front wheel steer angle (deg) (0=straight, max +/-30)
      dzactuator conversion: TargetSpeed = |linear.x*23/0.33| clamp +/-255 -> frame buf[6]
                       TargetAngle = angular.z+60 clamp [25,95] -> x10 -> frame buf[3..4]
                       SpeedDirection (0x00 forward / 0x01 reverse) -> frame buf[5]
  dzactuator --23-byte serial frame--> STM32:
      0xA5 0x5A | len | servo angle (0.1 deg, int16 big-endian) | direction | speed | gimbal | checksum
  STM32: servo angle -> front steer servo PWM;
         speed byte -> drive wheel motor PID closed loop (encoder feedback) -> PWM duty
  this node --/move_base/stop_signal(UInt8)--> dzrknn_dataprocess
        1 = car stopped, allow gimbal scan + shooting
        0 = shooting done (or timeout), continue route

=== Control pipeline (control_loop, 20ms per tick, 50Hz) ====

  1. Waiting for shooting? -> stop, publish zero Twist until 10 shots or 150s timeout
  2. No loaded path? -> publish empty Twist (keeps dzactuator Pursuit mode alive)
  3. Find nearest segment -> project car onto remaining path
  4. Find lookahead aim point -> point on path 0.2m ahead
  5. Near a shoot waypoint? -> snap to waypoint center, stop, allow shooting
  6. Reached the end? -> snap, stop, allow shooting, never move again
  7. Sharp corner ahead? -> three-phase in-place turn: stop / wait servo 0.5s / creep
  8. Otherwise -> pure pursuit steering + speed shaping

=== Module outline (sections of this file, their functions and responsibilities) ===

  (1) Imports & global constants  - rclpy/msg imports; CMD_QOS (QoS config); map constants
                          (14 cols x 11 rows, 0.35m per block); RAW_YAW_TO_MAP_YAW_OFFSET
  (2) Math utility functions      - euler_from_quaternion(quaternion->yaw),
                                    normalize_angle(angle to +/-pi),
                                    block_to_meters(block name->meters),
                                    heading_to_quaternion(heading letter->quaternion)
  (3) __init__ (node constructor)  - path/vehicle/sharp corner PIVOT/mid-route shoot/snap
                                    state vars; nav params (lookahead/wheelbase/arrival
                                    threshold/approach segment); subscribers & publishers;
                                    50Hz control timer; publisher health check timer
  (4) Path loading & command parse - check_odom_publishers(loc topic publisher check),
                                    load_path(read path file), waypoint_callback(cmd entry:
                                    load/reset/single target/multi-waypoint dispatch),
                                    follow_route_command(route parsing)
  (5) Sensor input               - shoot_count_cb(cumulative shot count cache),
                                    odom_callback(localization conversion: block->meters,
                                    raw yaw->map frame, apply snap offset)
  (6) Stop decision               - enter_shoot_stop(mid-route shoot stop: zero Twist brake
                                    + pose snap + signal)
  (7) Pure pursuit geometry       - find_nearest_segment(nearest seg + projection),
                                    find_lookahead_point(lookahead aim + corner clamp),
                                    pure_pursuit_control(steer angle + linear velocity)
  (8) Main control loop          - control_loop(priority state machine: shoot wait / idle
                                    keep-alive / shoot point detect / end arrival / sharp
                                    corner PIVOT three-phase / normal pure pursuit + slowdown)
  (9) main (entry point)         - rclpy init, node spin, Ctrl-C exit publishes zero Twist brake
========================================================================
"""

import math                     # math library: trig/sqrt/angle conversions (pure pursuit geometry)
import sys                      # read command-line args sys.argv (optional path file)
import os                       # (kept import, not directly used currently)
import rclpy                    # ROS2 Python client library entry (init/spin/shutdown)
from rclpy.node import Node     # ROS2 node base class, this node inherits it
from rclpy.qos import QoSProfile, ReliabilityPolicy  # QoS config: queue depth & reliability policy
from nav_msgs.msg import Odometry            # odometry message (localization pose: position + quaternion)
from geometry_msgs.msg import Twist          # control command message (linear.x=lin vel, angular.z=steer angle)
from std_msgs.msg import String, UInt8, Int32, Int32MultiArray  # + /offset_center 视觉偏移

# QoS config matches dzactuator's subscriber end (KeepLast(1) + best_effort).
# BEST_EFFORT + depth 1: control commands are "fire and forget", only the latest
# frame matters; with reliable transport, old commands would pile up when the
# subscriber stalls and replay in a burst on recovery, causing latency jitter.
CMD_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

# ---- Map constants ----
COL_LETTERS = 'ABCDEFGHIJKLMN'  # 14 column letters, A (left/west) .. N (right/east)
MAP_COLS = 14                   # map column count
MAP_ROWS = 11                   # map row count, row 1 (far end) .. row 11 (near end)
BLOCK_SIZE = 0.35  # block edge length (meters), 14x11 grid map

# ★P0-3 越界保护：里程计是纯轮式死推算，会漂移。一旦信念位姿偏出地图，
#   纯追踪会把车越带越远(实测跑出地图 12 米仍未停)，必须有硬边界急停兜底。
MAP_X_MIN = 0.5 * BLOCK_SIZE     # row 11 中心
MAP_X_MAX = 10.5 * BLOCK_SIZE    # row 1  中心
MAP_Y_MIN = 0.5 * BLOCK_SIZE     # col N  中心
MAP_Y_MAX = 13.5 * BLOCK_SIZE    # col A  中心
MAP_SAFETY_MARGIN = 0.60         # 允许越界的余量(m)，超此即急停

# serial_reader publishes /odom/localization in "block" units and forwards raw odom heading.
# Raw odom frame: car heading = X+; map frame: startup heading = Y+. So raw yaw needs an offset:
#   e.g. raw yaw=0 (car facing X+) -> map yaw=+90 deg (i.e. Y+ direction);
#        raw yaw=-90 deg (right turn 90 deg) -> map yaw=0 (X+ direction).
RAW_YAW_TO_MAP_YAW_OFFSET = math.pi / 2  # fixed offset raw yaw -> map yaw (+90 deg)


# ##### 任务配置（赛程路线，直接写死在脚本里）#####
# 赛题：从智能车起点出发走完全程，依次经过两个"通"（桥洞可通行处），
#       并压过全部 8 个"雷"，避开 8 个"障"与绿线挡板，最终抵达智能车终点。
#
# 地图：14 列(A~N) x 11 行(1~11)，每格 0.35m；场地 4m x 5m
#   雷(8)：A2  G3  C4  I5  E8  D9  B10 I10
#   通(2)：K4  K11                     <- 两个桥洞，均在 K 列
#   障(8)：E1  H2  B4  G5  A8  I9  D11 E11
#   靶  ：G7+G8（相连两格，打击区）
#   起终点：智能车起点 N10 -> 终点 N5
#
# ★★★ 关键约束：绿线挡板 ★★★
#   挡板位于第 6 行与第 7 行之间，横跨 D 列到 N 列。
#   上下半场之间【只有 A / B / C 三列可以穿行】，D 列及以东全部封死。
#   本路线选择【B 列】过界（离挡板西缘 0.525m 余量，最稳）。
MISSION_BLOCKS = [
    # ---- 下半场（行 7~11）----
    'K7',    # 通：可通行桥洞
    'K9',    # 中转：沿 K 列南下，绕开 J7 障碍物
    'J9',    # 雷
    'D10',   # 雷
    'B9',    # 雷
    'E7',    # 雷
    'B7',    # 中转：沿第 7 行西行到 B 列 —— 挡板封死 D~N，必须回到 A/B/C 才能上行
    'B5',    # 中转：沿 B 列北上，穿过挡板缺口（离挡板边缘 0.525m）
    # ---- 穿过挡板缺口，进入上半场（行 1~6）----
    'C5',    # 雷
    'B3',    # 中转：绕开 C3 障碍物
    'B1',    # 中转：沿 B 列北上
    'A1',    # 雷（沿第 1 行西行）
    'D2',    # 中转：绕开 A2 / C3 障碍物，走第 2 行
    'E4',    # 中转：对齐第 4 行
    'F4',    # 雷（从西侧 E4 进入，避开其正上方 F3 障碍）
    'J5',    # 雷
    'K1',    # 通：可通行桥洞
    # ---- 终点 ----
    'N5',    # 智能车终点
]

# 启动时是否先把里程计重锚到某个点位。
# 车是"放"上去的（不是开上去的），重锚能让死推算从已知点起步，精度好得多。
# 智能车起点是 N10，开局朝 Y+（地图 +Y 方向）。None = 不重锚。
MISSION_RESET = 'M10,y'     # 智能车起点，朝 Y+

# 启动后是否自动下发 MISSION_BLOCKS 并开始循迹（等驱动都初始化完）
AUTO_START = True           # 想纯手动就改成 False
AUTO_START_DELAY = 8.0      # 秒；给底盘/相机/视觉/云台留初始化时间


# ★P1-1 漂移预算告警阈值(m)：单段航程结束时，信念位姿与目标点中心的偏差
#   超过这个值就告警。到达判定阈值是 0.157m，一旦单段漂移逼近它，
#   说明定位已经开始不可信，继续跑会"越锚越偏"。默认 0.30m 留一倍余量。
DRIFT_WARN_M = 0.30

# ★完成提示音：全程跑完时通过 /voice_switch 播放的语音 ID。
#   0 = 不播（默认）。
#   ⚠️ 各 ID 对应哪句话我没有实机依据 —— 板子上只有 1(切换自主模式) 和 15(cmd_vel)
#      两处用法可考。请先在车上跑  tools/test_voice.sh 把 ID 试出来，
#      再把满意的那句填到这里。
#   注意：底盘有"同一 ID 至少间隔 2s 才重播"的闸门，连播同一个 ID 不会响第二次。
FINISH_VOICE_ID = 0

# ★★★ 倒车模式（R: 前缀）★★★
#   global_planner 输出的阿克曼路径里，某些航段需要倒车到位（例如窄巷道掉头、
#   贴近射击点的微调）。路线里写成 'R:B5'，表示"倒着走到 B5"。
REVERSE_SPEED_RATIO = 0.6      # 倒车速度 = 巡航速度 x 该比例（倒车要慢）
# ★ 倒车转向方向：倒车时舵角对车头朝向的作用与前进相反（v<0 使航向角速度反号）。
#   本实现按"让【车尾】对准目标"推导，理论上是对的，但**没有实车依据**。
#   上车若发现倒车时车尾偏向了反方向，把这里改成 -1.0 即可，改完不用编译。
REVERSE_STEER_SIGN = 1.0

# ★★★ 最佳射击点与打靶执行 ★★★
#   配合 tools/shoot_planner.py 使用：规划器负责"停哪一格"（粗定位），
#   视觉云台负责"30cm±10cm 内的精瞄"。本段参数控制打靶执行本身。
SHOOT_DISTANCE_TARGET = 0.30   # 理想射击距离(m)。太近云台视场压不住靶，太远精度差
SHOOT_DISTANCE_MIN = 0.20      # 候选点距离下限(m)
SHOOT_DISTANCE_MAX = 0.45      # 候选点距离上限(m)
SHOOT_MAX_RETRY = 3            # ★每个靶最多尝试几个候选点（失败自动换下一个）
SHOOT_TIMEOUT = 20.0           # ★单个候选点的硬超时(s)。超时立刻换点/放行，绝不卡死

# ---- 开火前稳定判据（"底盘速度≈0 且连续稳定 N 帧"）----
SHOOT_SETTLE_SPEED = 0.02      # 底盘速度小于此值视为"停住"(m/s)
SHOOT_STABLE_FRAMES = 8        # 视觉偏移需连续稳定多少帧才允许开火(5~10)
SHOOT_STABLE_OFFSET_PX = 25    # 视作"稳定"的偏移量容差(px)
SHOOT_STABLE_TIMEOUT = 6.0     # 等稳定的最长时间(s)；等不到就"降级放行"而不是死等

# ---- 射击点评分权重（供 shoot_planner 使用，这里只做透传/展示）----
SHOOT_SCORE_W_GIMBAL = 0.6     # 云台偏转角代价权重
SHOOT_SCORE_W_DETOUR = 0.5     # 路径绕行代价权重
# ★★★ 打靶状态机（到达射击点后）★★★
#   旧逻辑：刹车 -> 直接开火窗口，等到打满 N 发或超时。
#   问题：车刚停还在惯性前冲、云台也还没锁定，这时就允许开火 ——
#         打出去的弹是浪费的，而且"开了几发"的计数把无效弹也算进去了。
#   新逻辑分三段，每段都有独立超时兜底，任何一段卡住都不会堵死路线：
#        0 SETTLE : 持续刹车，等底盘真正停稳（位置连续 SHOOT_SETTLE_TIME 秒不变）
#        1 SCAN   : 请求云台全向扫描，等它锁定(TRACK=2)；扫完没找到也放行
#        2 FIRE   : 已锁定且底盘静止 -> 才是真正的开火窗口，等打满 SHOOT_COUNT_TARGET
SHOOT_SETTLE_TIME = 0.8        # 位置需连续多少秒不变才算"停稳"(s)
# ★射击点必须连做【两轮水平扫描】（场地规则要求）
#   一轮 = scan_passes(2) 趟 = 左一趟 + 右一趟。
#   实现方式：收到"一轮扫完"的信号后，主动再发一次 /nav/scan_request，
#   而不是靠云台自己连续扫 —— 这样"两轮"的边界是明确的、可验证的。
SHOOT_SCAN_ROUNDS = 2

# ★★★ 扫描范围与"开过头"防护 ★★★
#  云台水平扫描范围（相对车头）：实配 scan_yaw_min_deg=-160° ~ scan_yaw_max_deg=+10°
#  （见 config/robot.yaml /dzrknn_dataprocess 段）。★注意这与 readme【六】
#  早期写的"±90°"不一致 —— 实际行为以配置为准，改扫描范围请改那两个参数。
#  这意味着：
#    · 车一旦【冲过射击点】，靶就落到扫描范围之外 —— 再也扫不到，这一枪必丢。
#    · 而且几乎没有补救机会（只有 10 发弹，且路线不支持任意倒车）。
#  所以接近射击点必须【提前减速】并【容忍轻微过冲】：
#    SHOOT_OVERSHOOT_TOL  —— 车越过射击点后，仍在此距离内就补触发停站
#                            （原来是"必须在半径 0.17m 内"，冲过就永远不触发）
#    SHOOT_SLOWDOWN_M     —— 进入此距离开始线性减速，防止冲过头
SHOOT_OVERSHOOT_TOL = 0.30     # 过冲容忍(m)：越过射击点后仍在此范围内就停
SHOOT_SLOWDOWN_M = 0.60        # 提前减速距离(m)
SHOOT_LOCK_TIMEOUT = 18.0      # 等云台锁定的最长时间(s)，超时放行，绝不卡路线

# ★★★ 桥洞强制直行（ALIGN_STRAIGHT）★★★
#   桥洞两边是实墙，Pure Pursuit 在洞口前为了对准会有小幅摆头，
#   宽度余量不足时会蹭墙。所以进入"桥洞前 N 格 + 桥洞内"这一段时，
#   直接关掉 Pure Pursuit 转向，改用 IMU 锁定航向、低速直冲。
ALIGN_STRAIGHT_BEFORE = 2.0    # 桥洞前多少格开始强制直行
ALIGN_STRAIGHT_AFTER = 1.0     # 桥洞后多少格结束强制直行
ALIGN_STRAIGHT_SPEED = 0.10    # 强制直行时的速度(m/s)
ALIGN_KP = 12.0                # 航向保持的比例增益（度/度）
ALIGN_STEER_LIMIT = 8.0        # 强制直行时舵角限幅(度) —— 小舵角微调，防止摆动

# 任务扫描模式：continuous=持续横扫(边走边扫) / waypoint=点位停站扫描 / none=不自动扫
MISSION_SCAN_MODE = 'continuous'

# 桥洞点位（强制直行保护点），由配置中心 bridge_blocks 覆盖
BRIDGE_BLOCKS = ['K7', 'K1']

# 障碍占格：射击点规划器用它算视线(LOS)遮挡。空列表 = 不做遮挡剔除。
# 由配置中心 obstacle_blocks 覆盖。
OBSTACLE_BLOCKS = []

# MISSION_BLOCKS 校验结果（由配置中心加载时填充）
MISSION_BLOCKS_VALID = True     # 路线 token 是否全部合法
MISSION_BLOCKS_PROBLEMS = []    # 不合法项的描述
MISSION_BLOCKS_STATS = {'total': 0, 'reverse': 0, 'shoot': 0, 'normal': 0}

# 基础巡航速度(m/s)。降速会让画面更稳，更容易满足云台开火前的"画面稳定"判据
LINEAR_SPEED = 0.15
# ★单靶打满即走：走到最佳射击点后打满这么多发就收站去下一个点。
#   必须 <= 云台侧 max_shots_per_target(robot.yaml /dzrknn_dataprocess 段，默认 3)，
#   否则云台会先一步停火，循迹只能干等 shoot_timeout，白白浪费每点 30 秒。
SHOOT_COUNT_TARGET = 3


# =============================================================================
#  配置中心接入  (P0 之后的质量底座之一)
# -----------------------------------------------------------------------------
#  读取  ~/dzrobot/config/robot.yaml  的 path_executor 段，覆盖上面的硬编码默认值。
#  全车只有一处需要改参数，且改完【不需要重新编译】，重启本脚本即生效。
#
#  ★ 失败时【静默回退】到上面的内置默认值 —— 配置中心本身坏了也不能让车跑不起来，
#    现场安全优先。启动日志会打印配置来源，可据此确认是否真的读到了。
# =============================================================================
CONFIG_FILE = os.environ.get(
    'ROBOT_CONFIG',
    os.path.expanduser('~/dzrobot/config/robot.yaml'))


def _load_config_center():
    """读取配置中心的 path_executor 段；任何异常都回退为空字典(用内置默认值)。"""
    try:
        import yaml
    except ImportError:
        return {}
    try:
        with open(CONFIG_FILE, 'r', encoding='utf-8') as fp:
            data = yaml.safe_load(fp) or {}
    except Exception:
        return {}
    if not isinstance(data, dict):
        return {}
    # 兼容两种写法：'/path_executor:' 或 'path_executor: ros__parameters:'
    for key in ('/path_executor', 'path_executor'):
        sec = data.get(key)
        if isinstance(sec, dict):
            return sec.get('ros__parameters', sec) or {}
    return {}


# -----------------------------------------------------------------------------
#  MISSION_BLOCKS 规范化与校验
# -----------------------------------------------------------------------------
#  从 robot.yaml 读进来的每个 token 都要能带后缀，且后缀可以叠加：
#      'K7'       普通航点
#      'R:K7'     倒车到位（global_planner 输出）
#      'F2*'      射击停站
#      'R:B5*'    倒车到位 + 射击停站
#      'N5,y'     仅【最后一个】token 可以带终点朝向（逗号后缀）
#
#  ★ 为什么要在加载时校验而不是等 follow_route_command 报错：
#    路线是整车任务的入口，一旦某个 token 拼错（比如把 'R:K7' 写成 'R K7'
#    或 'RK7'），follow_route_command 会直接 return 整条路线都不下发 ——
#    车会一动不动地停在起点，而日志只有一行容易被刷屏淹没的 error。
#    在这里一次性把问题点名报出来，出车前就能发现。
# -----------------------------------------------------------------------------
def normalize_mission_token(tok):
    """把一个路线 token 规范成大写、去空格的标准形式（保留 R: 与 * 后缀）。"""
    t = str(tok).strip()
    if not t:
        return ''
    # 只在【去除后缀】的状态下做大写化，避免把用户写的 ',x' 变成 ',X'
    # （follow_route_command 里对朝向做的是小写比较）
    if ',' in t:
        body, tail = t.rsplit(',', 1)
        return body.strip().upper() + ',' + tail.strip().lower()
    return t.upper()


def validate_mission_blocks(blocks):
    """校验路线 token 列表。返回 (是否全部合法, 问题清单, 统计信息)。

    这里刻意做【宽松但完整】的检查：只报错、不改写 ——
    真正的解析仍然由 follow_route_command 负责，避免两处逻辑各说各话。
    """
    problems = []
    n_rev = n_shoot = n_normal = 0

    for i, tok in enumerate(blocks):
        t = str(tok).strip()
        if not t:
            problems.append(f"#{i} 空 token")
            continue

        # ★'SHOOT@<靶标>' 是【指令】不是坐标点：它会让 follow_route_command
        #   现场算出射击点再插进路线。校验器必须认识它，否则合法的自动选点
        #   路线会被误报成"非法坐标"。
        if t.upper().startswith('SHOOT@'):
            tgt = t[6:].strip().upper()
            # 本模块没有 parse_block（那是 shoot_planner 里的函数），
            # 所以这里内联做格式校验，别去调用不存在的名字。
            if (len(tgt) >= 2 and tgt[0] in COL_LETTERS and tgt[1:].isdigit()
                    and 1 <= int(tgt[1:]) <= MAP_ROWS):
                n_shoot += 1
            else:
                problems.append(f"#{i} '{t}' 的靶标名不合法（应为 A~N + 数字）")
            continue

        body = t
        is_last = (i == len(blocks) - 1)

        # 终点朝向后缀只允许出现在最后一个 token 上
        if ',' in body:
            if not is_last:
                problems.append(
                    f"#{i} '{t}' 带 ',x/,y' 朝向后缀，但它不是最后一个航点 "
                    f"（朝向只对终点有意义）")
            body = body.rsplit(',', 1)[0].strip()

        # 剥 R: 前缀（必须在前）
        if body.upper().startswith('R:'):
            body = body[2:].strip()
            n_rev += 1
        # 剥 ':SHOOT_STOP' / ':SCAN_STOP' 标签（与 follow_route_command 的解析顺序一致）
        if ':' in body:
            _head, _tail = body.split(':', 1)
            _tu = _tail.strip().upper()
            if _tu in ('SHOOT_STOP', 'SHOOT'):
                body = _head.strip()
                n_shoot += 1
            elif _tu in ('SCAN_STOP', 'SCAN'):
                body = _head.strip()
            # 其它后缀保持原样，交给下面的检查报错
        # 再剥 * 后缀（必须在后）。唯一合法组合是 'R:B5*' 或 'R:B5:SHOOT_STOP'
        if body.endswith('*'):
            body = body[:-1].strip()
            n_shoot += 1

        body_u = body.upper()
        if (len(body_u) < 2 or body_u[0] not in COL_LETTERS
                or not body_u[1:].isdigit()):
            problems.append(
                f"#{i} '{t}' 不是合法坐标（应为 字母A~N + 数字，"
                f"可带 R: 前缀与 * 后缀）")
            continue

        row = int(body_u[1:])
        if not (1 <= row <= MAP_ROWS):
            problems.append(f"#{i} '{t}' 行号 {row} 越界（应为 1~{MAP_ROWS}）")
            continue

        if not (t.upper().startswith('R:') or t.endswith('*')):
            n_normal += 1

    stats = {'total': len(blocks), 'reverse': n_rev,
             'shoot': n_shoot, 'normal': n_normal}
    return (len(problems) == 0), problems, stats


_CFG = _load_config_center()
CFG_SOURCE = CONFIG_FILE if _CFG else '<内置默认值，未读到配置中心>'
if _CFG:
    if isinstance(_CFG.get('mission_blocks'), list) and _CFG['mission_blocks']:
        # 规范化保留 R: / * 后缀（旧代码的 .upper() 会保留它们，但不会校验）
        MISSION_BLOCKS = [b for b in
                          (normalize_mission_token(x)
                           for x in _CFG['mission_blocks'])
                          if b]
        _mb_ok, _mb_problems, _mb_stats = validate_mission_blocks(MISSION_BLOCKS)
        MISSION_BLOCKS_VALID = _mb_ok
        MISSION_BLOCKS_PROBLEMS = _mb_problems
        MISSION_BLOCKS_STATS = _mb_stats
    if _CFG.get('mission_reset'):
        MISSION_RESET = str(_CFG['mission_reset'])
    if 'auto_start' in _CFG:
        AUTO_START = bool(_CFG['auto_start'])
    if 'auto_start_delay' in _CFG:
        AUTO_START_DELAY = float(_CFG['auto_start_delay'])
    if 'linear_speed' in _CFG:
        LINEAR_SPEED = float(_CFG['linear_speed'])
    if 'shoot_count_target' in _CFG:
        SHOOT_COUNT_TARGET = int(_CFG['shoot_count_target'])
    if _CFG.get('mission_scan_mode'):
        MISSION_SCAN_MODE = str(_CFG['mission_scan_mode'])
    if 'map_safety_margin' in _CFG:
        MAP_SAFETY_MARGIN = float(_CFG['map_safety_margin'])
    if 'drift_warn_m' in _CFG:
        DRIFT_WARN_M = float(_CFG['drift_warn_m'])
    if 'finish_voice_id' in _CFG:
        FINISH_VOICE_ID = int(_CFG['finish_voice_id'])
    for _k in ('shoot_distance_target', 'shoot_distance_min', 'shoot_distance_max',
               'shoot_settle_speed', 'shoot_stable_offset_px',
               'shoot_stable_timeout', 'shoot_overshoot_tol',
               'shoot_slowdown_m', 'shoot_score_w_gimbal',
               'shoot_score_w_detour', 'shoot_timeout',
               'shoot_settle_time', 'shoot_lock_timeout',
               'reverse_speed_ratio', 'reverse_steer_sign',
               'align_straight_before', 'align_straight_after',
               'align_straight_speed', 'align_yaw_tol_deg',
               'align_kp', 'align_steer_limit'):
        if _k in _CFG:
            globals()[_k.upper()] = float(_CFG[_k])
    # 桥洞点位：字符串列表，如 ['K7', 'K1']。强制直行保护点。
    if isinstance(_CFG.get('bridge_blocks'), list):
        BRIDGE_BLOCKS = [str(b).strip().upper() for b in _CFG['bridge_blocks']]
    if isinstance(_CFG.get('obstacle_blocks'), list):
        OBSTACLE_BLOCKS = [str(b).strip().upper() for b in _CFG['obstacle_blocks']]
    for _k in ('shoot_max_retry', 'shoot_stable_frames', 'shoot_candidates_keep'):
        if _k in _CFG:
            globals()[_k.upper()] = int(_CFG[_k])

# ##### Math utility functions #####
# All pure math conversions live here: quaternion->yaw, angle normalization,
# block name<->meters, heading letter->quaternion. Reused by command parsing,
# localization callback, and pure pursuit.

def euler_from_quaternion(x, y, z, w):
    """Quaternion -> yaw angle (radians)."""
    # Standard yaw extraction from ZYX Euler angles. For pure Z-axis rotation
    # (only qz, qw) this simplifies to yaw = 2*atan2(qz, qw); the general form
    # tolerates small residual x/y components from IMU filter output.
    siny_cosp = 2 * (w * z + x * y)      # numerator = sin(yaw)*cos(pitch)
    cosy_cosp = 1 - 2 * (y * y + z * z)  # denominator = cos(yaw)*cos(pitch)
    yaw = math.atan2(siny_cosp, cosy_cosp)  # atan2(num, den) -> yaw in (-pi, pi]
    return yaw  # return yaw (radians)


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    # Called whenever two yaws are differenced, ensuring the short arc is taken
    # (e.g. a 350 deg error folds to -10 deg instead of spinning a full turn).
    while angle > math.pi:      # above +pi:
        angle -= 2 * math.pi    # subtract a full turn toward (-pi, pi]
    while angle < -math.pi:     # below -pi:
        angle += 2 * math.pi    # add a full turn toward [-pi, pi)
    return angle                # return normalized angle


def block_to_meters(block_name):
    """Block name -> metric coordinates.

    e.g. M10 -> (x, y) meters
    M is the 13th letter (column index 12, 0-based)
    10 is row 10
    """
    letter = block_name[0].upper()   # column letter A..N (upper-cased)
    number = int(block_name[1:])     # row number 1..11

    # Column index (A=0, B=1, ..., N=13)
    col_idx = COL_LETTERS.index(letter)
    # Row index (1=0, 2=1, ..., 11=10)
    row_idx = number - 1

    # Convert to metric coordinates.
    # Map origin is at the bottom-right corner of block N11.
    # X: row 11 (row_idx=10) -> ~0 m, row 1 -> 10*0.35 = 3.5 m
    # Y: column N (col_idx=13) -> ~0 m, column A -> 13*0.35 = 4.55 m
    # +0.5 offset places the coordinate at block center, not corner.
    # N11 center = (0.175, 0.175), A1 center = (3.675, 4.725).
    # serial_reader's dead-reckoning output uses the same convention (start N10 = block (1.5, 0.5)).
    x = (10.5 - row_idx) * BLOCK_SIZE   # row 11 -> 0.175 m, row 1 -> 3.675 m
    y = (13.5 - col_idx) * BLOCK_SIZE   # col N -> 0.175 m, col A -> 4.725 m

    return x, y  # return metric coords (x, y) of this block's center


def heading_to_quaternion(heading_dir):
    """Heading letter -> quaternion.

    heading_dir: 'x' = facing X+, 'y' = facing Y+
    returns: (qx, qy, qz, qw)
    """
    if heading_dir == 'y':       # facing Y+:
        yaw = math.pi / 2        # corresponds to yaw +90 deg
    elif heading_dir == 'x':     # facing X+:
        yaw = 0.0                # corresponds to yaw 0 deg
    else:                        # other input:
        yaw = 0.0                # fallback: treat as X+

    # Planar rotation yaw -> quaternion: only qz and qw are non-zero.
    # qz=sin(yaw/2), qw=cos(yaw/2)
    qz = math.sin(yaw / 2)       # quaternion Z component
    qw = math.cos(yaw / 2)       # quaternion W component
    return 0.0, 0.0, qz, qw      # return (qx=0, qy=0, qz, qw)


# ##### Node class #####
# PathExecutorNode: path tracking executor body. The constructor sets up all
# state and topic plumbing; other functions cooperate as
# "command parse -> sensor input -> geometry -> main control loop".

class _GatedCmdPub:
    """速度指令闸门 —— 手动接管期间丢弃自动指令。

    ★ 为什么需要它：
      循迹的 control_loop 以 50Hz 持续向 pursuitAngle 发追求角，
      而浏览器遥控器也往同一个话题发 —— 两边不同频、不同值，
      下位机拿到的是互相打架的指令，车会一顿一顿地抽。
      接管期间这里直接吞掉自动指令，把控制权让给遥控器；释放后自动恢复。

    ★ 为什么用包装器而不是在每个 publish 点加 if：
      本文件有 15 处 self.cmd_pub.publish(...)，逐个加判断既啰嗦又容易漏。
      包一层只有一个入口，将来新增发布点也自动受管辖。
    """

    __slots__ = ("_pub", "_node")

    def __init__(self, pub, node):
        self._pub = pub
        self._node = node

    def publish(self, msg):
        if getattr(self._node, "manual_override", False):
            return              # 接管中：自动循迹闭嘴
        self._pub.publish(msg)


class PathExecutorNode(Node):
    def __init__(self, path_file=None):
        super().__init__('path_executor_node')  # init ROS2 node, name path_executor_node

        # Suppress routine logs: only warn/error level is printed.
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)

        # Path data
        # path_points: list of waypoints, each a dict {'x','y','z','qx','qy','qz','qw'}, meters.
        #   In route-command mode, point i's quaternion stores "the heading the car should
        #   have when arriving at that point" (= direction of the segment leaving it;
        #   the last point stores the final heading).
        # current_target_idx: index of the "segment start" the controller is currently on
        #   (advanced forward by find_nearest_segment).
        self.path_points = []        # waypoint list (metric coords)
        self.current_target_idx = 0  # current target segment start index
        self.path_loaded = False     # whether a valid path is loaded

        # Current state (all in map frame: meters / radians)
        self.current_x = 0.0   # belief position x (m) = raw dead-reckoning + snap offset
        self.current_y = 0.0   # belief position y (m)
        self.current_yaw = 0.0 # belief yaw (radians)
        self.has_odom = False         # set True after first /odom/localization frame
        self.arrived = False          # latched True at end (never moves again)
        self.arrive_logged = False    # guard: arrival log + stop_signal run once

        # Sharp corner (turn > 45 deg) in-place turn (PIVOT) state machine.
        # Pure pursuit traces an "S" trajectory at 90 deg corners: front wheel
        # first reverses then corrects, body visibly fishtails.
        # Alternative:
        #   stop before vertex -> steer servo to max -> creep at 0.08 m/s ->
        #   heading aligned within 15 deg -> resume pure pursuit.
        # Diagonal segments (turn < 45 deg) still use pure pursuit.
        self.corner_mode = False       # True = in-place turn in progress
        self.corner_target_yaw = 0.0   # exit segment heading (radians)
        self.corner_turn_dir = 0       # +1 left, -1 right (from cross product sign)
        self.corner_phase = 0          # phase: 0=stop, 1=wait servo, 2=creep
        self.corner_phase_t = 0.0      # phase start time (seconds, wall clock)
        self.corner_seg_idx = -1       # segment index to resume after in-place turn

        # Navigation params
        self.lookahead_dist = 0.15
        self.linear_speed = LINEAR_SPEED  # m/s, 基础巡航速度(来自配置中心，默认 0.15)
        # 降速的附带作用：画面更稳，更容易满足云台开火前的 aim_ready 稳定判据，
        # 从而真正做到"先完成瞄准、再执行开火"
        self.max_angular_speed = 1.5  # rad/s (kept for compatibility; servo angle interface does no angular-rate clamp)
        # Effective wheelbase from dzactuator's encoder calibration:
        # ticksPer2PI/(2*pi*ticksPerMeter) = 3236/(2*pi*3290) = 0.157 m.
        # PDF nominal 0.150 m; 0.157 calibration matches the odom the controller actually receives.
        self.wheelbase = 0.157  # m, wheelbase L in pure pursuit formula
        # Arrival threshold is intentionally set to "stop 1/4 wheel circumference short"
        # (the car will coast past the exact point; tightening the threshold stops it closer).
        # Wheel diameter D from dzactuator calibration: encoder 13 PPR x gear ratio 29 x 2 (AB quadrature) = 754 tick/rev,
        # ticksPerMeter=3290 -> circumference C = 754/3290 = 0.229 m -> D = 0.073 m.
        # 1/4 rev = pi*D/4 = ~0.0573 m.
        # So the car declares arrival ~0.157 m from block center, then the snap logic below
        # pulls the "belief pose" straight to the center.
        self.arrival_threshold = 0.10 + (math.pi * 0.073 / 4.0)  # ~0.157 m
        self.approach_dist = 0.6  # m, straight approach segment length before the end

        # ---- Mid-route shoot stop params ----------------------------------
        # Waypoints with '*' in the route command (e.g. 'D2*') make the car stop there,
        # opening gimbal scan + shooting via stop_signal, then waiting for /shoot_count
        # to reach SHOOT_COUNT_TARGET before continuing.
        self.shoot_flags = []            # shoot-stop flag array parallel to path_points
        # ★倒车标志：reverse_flags[i]=True 表示"第 i 个航点要【倒着】到达"，
        #   即第 (i-1 -> i) 段是倒车段。与 path_points 平行。
        self.reverse_flags = []
        # ★桥洞强制直行状态
        self.align_active = False        # 当前是否处于强制直行区
        self.align_locked_yaw = 0.0      # 进入强制直行时锁定的航向(rad)
        self.align_bridge_name = ''      # 当前保护的桥洞名（仅用于日志）
        # ★需要强制直行的【航段索引集合】（路线装载时预计算，见 _rebuild_align_segments）
        self._align_segs = set()
        self.shoot_wait = False          # True = stopped, waiting for shooting to finish
        self.shoot_wait_start = 0.0      # wait start time (seconds)
        self.shoot_count_at_arrival = 0  # /shoot_count snapshot at arrival
        # ★打靶三段式状态机（0=等停稳 1=等锁定 2=开火）
        self.shoot_phase = 0
        self.shoot_phase_t = 0.0         # 本阶段开始时刻
        self.shoot_settle_x = 0.0        # 停稳判定基准位置
        self.shoot_settle_y = 0.0
        self.shoot_settle_since = 0.0    # 位置最近一次变化(或首次稳定)的时刻
        self.shoot_lock_seen = False     # 本次停站中是否见过云台扫描(SCAN)
        # ★水平扫描轮次计数（场地规则：射击点必须连做两轮）
        self.scan_rounds_done = 0
        self.scan_round_active = False
        self.SHOOT_COUNT_TARGET = SHOOT_COUNT_TARGET   # 来自 config/robot.yaml
        self.SHOOT_WAIT_TIMEOUT = 150.0  # seconds; on timeout give up and continue
        self.SHOOT_STOP_RADIUS = 0.17    # m; stop judgment radius around a shoot waypoint

        # ---- 点位全向扫描（与 dzrknn_dataprocess 的自瞄状态机握手）---------
        # 循迹是最高优先级：下面所有等待都带【硬超时】，绝不允许卡住路线。
        #   话题约定
        #     /aim/status       0=IDLE(未锁定) 1=SCAN(点位扫描中) 2=TRACK(已锁定/边走边跟随)
        #     /nav/scan_request  1 -> 请求云台做一次全向扫描（左一趟 + 右一趟）
        self.aim_status = 0                  # 最近一次 /aim/status
        # ★视觉偏移状态（开火稳定判据的数据源）
        self.det_dx = 0                      # 目标框中心横向偏移(px)
        self.det_dy = 0                      # 纵向偏移(px)
        self.det_cls = 0                     # 0=无目标 1=蓝靶 2=红靶
        self.det_t = 0.0                     # 最近一次收到视觉结果的时刻
        self.det_box_w = 0                   # 框宽（估距离用）
        # ★靶标候选射击点链：每个靶标由 shoot_planner 算出多个候选点，
        #   第一个打不中就自动换下一个 —— "宁可漏打，不可卡死"。
        self.shoot_chain = {}                # {靶标格: [候选格, ...]}
        self.shoot_wp_idx = -1               # 当前正在处理的射击点索引
        self.chassis_speed = 0.0             # ★底盘实际线速度(m/s)，由里程计差分算出
        self._prev_pose = None               # 上一次位姿（算速度用）
        self._prev_pose_t = 0.0
        self.stable_count = 0                # ★连续"稳定"帧计数（开火门控）
        self.shoot_attempt = 0               # 当前靶标已尝试到第几个候选点
        self.shoot_resume_gid = 0            # 打中后需要取消的候选组号
        self.shoot_chain_meta = {}           # 调试信息（距离/评分）
        self.scan_flags = []                 # 需要"停站扫描"的点位标记（与 path_points 平行）
        self.scan_wait = False               # True = 已停车，正在等云台扫描结果
        self.scan_wait_start = 0.0           # 等待开始时刻（秒）
        self.scan_seen_scanning = False      # 本次等待中是否已看到 status==1（用于判定"扫完没找到"）
        self.SCAN_WAIT_TIMEOUT = 8.0         # 硬超时：最多等这么久，之后无条件继续循迹
        self.SCAN_STOP_RADIUS = 0.20         # m；触发扫描停站的点位判定半径
        # 任务扫描模式：'continuous'=持续横扫(边走边扫,默认) / 'waypoint'=点位停站扫描 / 'none'=不自动扫
        self.mission_scan_mode = MISSION_SCAN_MODE

        # ★障碍表：射手动点规划器需要它算 LOS 遮挡。
        #   优先读配置中心 obstacle_blocks；没有就留空（规划器仍可工作，
        #   只是不会因障碍剔点 —— 地图数据不全比算不出来强）。
        self._obstacle_blocks = list(OBSTACLE_BLOCKS)

        # ★桥洞点坐标（强制直行保护点）：
        #   桥洞两边是实墙，Pure Pursuit 在洞口前为了对准会有小幅摆头，
        #   宽度余量不足时会蹭墙。所以把桥洞点预先转成米制坐标缓存下来。
        self._bridge_points = []
        try:
            for _bn in BRIDGE_BLOCKS:
                _bx, _by = block_to_meters(_bn)
                self._bridge_points.append((_bx, _by, _bn))
        except Exception as _e:
            self.get_logger().warn(f"桥洞点解析失败({_e})，强制直行保护将不生效")
        if self._bridge_points:
            self.get_logger().info(
                "桥洞强制直行保护点: " +
                " ".join(f"{n}({x:.2f},{y:.2f})" for x, y, n in self._bridge_points) +
                f" | 洞前 {ALIGN_STRAIGHT_BEFORE} 格起锁航向、限速 {ALIGN_STRAIGHT_SPEED} m/s")

        # Odom snap offset: at each waypoint the "belief pose" is re-anchored to
        # the block center + commanded heading, limiting dead-reckoning (DR) drift
        # to within a single segment.
        # Belief pose = raw DR pose + offset. Snap only modifies the offset (never
        # touches raw data); every subsequent /odom/localization frame is shifted
        # by the same amount. Drift thus cannot accumulate across segments:
        # it is reset at every waypoint.
        self.snap_x_off = 0.0   # position x snap offset (m)
        self.snap_y_off = 0.0   # position y snap offset (m)
        self.snap_yaw_off = 0.0 # yaw snap offset (radians)
        self.raw_x = 0.0        # raw DR position x (m), without offset
        self.raw_y = 0.0        # raw DR position y (m)
        self.raw_map_yaw = 0.0  # raw DR yaw (map frame), without offset

        # Subscribe to localization topic (the only source of localization data for this node; full chain):
        #   [STM32] --encoder/IMU data--> serial (dzactuator opens /dev/ttyUSB0, 115200 bps)
        #   dzactuator: parses serial -> publishes /odom(Odometry)
        #   serial_reader: subscribes /odom + /imu_data, heading from IMU (2s timeout
        #     fallback to odom), position via "step x map heading" dead reckoning
        #     -> publishes /odom/localization
        #   frame (Odometry): position.x/y = block-unit coords (1 block = 0.35 m),
        #     orientation quaternion = raw yaw_rel (this node adds pi/2 to map to map frame)
        # serial_reader publishes "block"-unit coords (x=0..10.5, y=0..13)
        self.odom_sub = self.create_subscription(
            Odometry,               # msg type: nav_msgs/Odometry (position + quaternion pose)
            '/odom/localization',   # topic: localization published by serial_reader
            self.odom_callback,     # each localization frame enters odom_callback for conversion
            10                      # queue depth 10
        )

        # Subscribe to waypoint commands (trigger topic)
        # Supports: 'M10,y' single target | 'G8 A10 D2*' multi-waypoint route |
        #           'load:/path/file.txt' load path file | 'reset:N10,y' re-anchor odom
        self.waypoint_sub = self.create_subscription(
            String,                 # msg type: std_msgs/String (text command)
            '/nav/waypoint',        # topic: send_waypoint.py and similar tools send routes
            self.waypoint_callback, # command parse callback
            10                      # queue depth 10
        )

        # Publish control commands (this node's only control output; both motor
        # speed and steering leave here). This Twist is subscribed by dzactuator's
        # callback_movebase_angle (KeepLast(1) + best_effort):
        #   linear.x (m/s)  -> TargetSpeed = |v*23/0.33| clamp +/-255 -> serial frame buf[6] (speed byte)
        #   angular.z (deg) -> TargetAngle = value+60 clamp [25,95] -> x10 -> frame buf[3..4] (servo angle)
        #   v>0: SpeedDirection=0x00 -> frame buf[5]; v<0: 0x01
        #   (0.33 is calibrated full-speed m/s, 23 is full-speed PWM equivalent;
        #    the whole frame is packed with 0xA5 0x5A header and sent to STM32)
        # ★手动接管标志：由 /nav/manual_override 置位。
        #   必须在创建 cmd_pub 之前初始化 —— 闸门第一次 publish 就会读它。
        self.manual_override = False

        self._cmd_pub_raw = self.create_publisher(
            Twist,            # msg type: geometry_msgs/Twist
            'pursuitAngle',   # topic: dzactuator subscribes by this name
            CMD_QOS           # QoS: matches dzactuator subscriber (depth 1, BEST_EFFORT)
        )
        # 外面套一层闸门：接管期间自动指令全部被吞掉（见 _GatedCmdPub）
        self.cmd_pub = _GatedCmdPub(self._cmd_pub_raw, self)

        # Publish stop signal to dzrknn_dataprocess (1 = allow gimbal scan + shooting, 0 = forbid).
        # Sent once at each mid-route shoot stop, and once more on final arrival.
        self.stop_signal_pub = self.create_publisher(
            UInt8,                     # msg type: std_msgs/UInt8 (single-byte flag)
            '/move_base/stop_signal',  # topic
            10                         # queue depth 10
        )

        # dzrknn_dataprocess shot count (one msg per shot fired, cumulative value)
        self.shoot_count_sub = self.create_subscription(
            Int32, '/shoot_count', self.shoot_count_cb, 10)  # subscribe cumulative shot count topic
        self.shoot_count = 0  # locally cached latest cumulative shot count

        # 自瞄状态订阅：0=IDLE 1=SCAN 2=TRACK
        # 云台以 100Hz 持续上报，所以即使本节点晚启动也能立刻读到当前状态。
        # ★手动接管开关（浏览器遥控器用）
        #   1 = 接管：本节点停止下发自动指令，控制权交给遥控器
        #   0 = 释放：恢复自动循迹
        #   ★安全性：接管期间本节点完全静默；若遥控器也停了，
        #     dzactuator 的 1 秒看门狗会自动刹停 —— 三层保险里的最后一层。
        self.manual_override_sub = self.create_subscription(
            UInt8, '/nav/manual_override', self.manual_override_cb, 10)

        self.aim_status_sub = self.create_subscription(
            Int32, '/aim/status', self.aim_status_cb, 10)

        # 扫描请求发布：到达点位且尚未锁定时，请求云台做一次全向扫描
        self.scan_req_pub = self.create_publisher(UInt8, '/nav/scan_request', 10)

        # ★运动状态上报（供浏览器给轨迹分色）
        #   0 = 前进 / 1 = 倒车。只在变化时发，不给总线添压力。
        self.motion_pub = self.create_publisher(UInt8, '/nav/motion_state', 10)
        self._motion_last = -1

        # ★视觉检测结果订阅（dzrknn_dataprocess <- det_node）
        #   data = [dx, dy, cls, box_w, box_h, kpt_x, kpt_y, kpt_conf]
        #     dx/dy   : 目标框中心相对画面中心的偏移(px)
        #     cls     : 0=无目标 1=蓝靶 2=红靶
        #   打靶前必须靠它判断"到底有没有看到靶、瞄得稳不稳"——
        #   否则只能"到达即放行开火"，白白浪费弹药（全场只有 10 发）。
        self.offset_sub = self.create_subscription(
            Int32MultiArray, '/offset_center', self.offset_center_cb, 10)

        # ★完成提示音：底盘 dzactuator 订阅 /voice_switch(UInt8)，直接写入
        #   moveBaseControl.VoiceSwitch 触发播报。ID 含义见 FINISH_VOICE_ID 注释。
        self.voice_pub = self.create_publisher(UInt8, '/voice_switch', 10)

        # ★P1-1 漂移预算统计（每次到位都会累加，结束横幅里汇总）
        self.drift_max = 0.0        # 单段最大位置漂移(m)
        self.drift_yaw_max = 0.0    # 单段最大航向漂移(deg)
        self.drift_legs = 0         # 统计了多少段
        self.drift_warn_count = 0   # 超阈值次数

        # Control timer (50 Hz)  ← 0.02s/tick
        self.control_timer = self.create_timer(0.02, self.control_loop)

        # One-shot health check: /odom/localization must have exactly one publisher.
        # Two publishers with different frames would alternate and corrupt the pose
        # (this once caused 90 deg heading/position jumps).
        self.create_timer(3.0, self.check_odom_publishers)  # health check once 3 s after startup

        # 自动任务：启动 AUTO_START_DELAY 秒后自动下发内置路线（MISSION_BLOCKS）
        # 一次性定时器，触发后立即 cancel，避免重复下发。
        self._auto_started = False
        self._auto_start_timer = None
        if AUTO_START:
            self._auto_start_timer = self.create_timer(
                AUTO_START_DELAY, self._auto_start_mission)

        # Load path file (if given via command-line arg)
        if path_file:                  # if a path file arg was given at startup
            self.load_path(path_file)  # load it immediately

        self.get_logger().info("=" * 50)  # separator
        self.get_logger().info("Path Executor Node Initialized")  # node init done
        self.get_logger().info("Commands:")  # supported commands below
        self.get_logger().info("  'M10,y' -> Go to M10, axis-aligned final approach")  # single target: go to M10, last segment axis-aligned
        self.get_logger().info("  'G8 A10' -> route: straight lines between")  # multi-waypoint route: straight lines between
        self.get_logger().info("          waypoints, DIAGONALS allowed, the")  # diagonals allowed, and each
        self.get_logger().info("          turn angle at each waypoint is")     # turn angle at each waypoint
        self.get_logger().info("          computed automatically")             # is auto-computed
        self.get_logger().info("  'G8 A10 C2,y' -> route, final stop facing Y+")  # route + forced final heading Y+
        self.get_logger().info("  'G8 D2* E2' -> route, STOP and fire 10")  # route + mid-route stop at D2
        self.get_logger().info("          shots at D2, then continue")      # fire 10 then continue
        self.get_logger().info("  'A10' -> direct line from current position")  # straight line from current position to A10
        self.get_logger().info("  'load:/path/to/file.txt' -> Load path file")  # load path file
        self.get_logger().info("  'reset:L9,x' -> re-anchor odometry after")  # after manually moving
        self.get_logger().info("          manually moving/rotating the car")  # the car, re-anchor odom pose
        self.get_logger().info("=" * 50)  # closing separator
        self.get_logger().info("Note: modeTimerCallback will auto-switch to Pursuit mode")  # note: dzactuator auto-switches
        self.get_logger().info("when pursuitAngle is the only active topic")  # to Pursuit when only pursuitAngle is active

    # ##### Publisher health check #####
    # Called once 3 s after startup: confirm /odom/localization has exactly one
    # publisher, preventing two localization nodes from corrupting the pose.

    def check_odom_publishers(self):
        """Check whether /odom/localization has multiple publishers."""
        try:
            count = self.count_publishers('/odom/localization')  # query topic publisher count
            if count > 1:
                # Two dead-reckoning nodes running at once = poses from two
                # frames alternate; the controller chases garbage data. Once
                # caused 90 deg heading jumps, so report an error here.
                self.get_logger().error(
                    f"{count} publishers on /odom/localization! "
                    "Kill the extra node (e.g. old C++ car_localization_node), "
                    "otherwise the pose is corrupted"
                )
            elif count == 0:
                # Early in startup serial_reader may not be up yet; normal.
                self.get_logger().warning(
                    "No publisher on /odom/localization yet"
                )
            else:
                self.get_logger().info(
                    "Localization OK: 1 publisher on /odom/localization"
                )
        except Exception:
            pass  # silently ignore health-check failure (does not affect main flow)

    # ##### Path file loading #####
    # Parse "x, y, z, qx, qy, qz, qw" text line by line into the path_points list.

    def load_path(self, file_path):
        """Load path from file."""
        try:
            self.path_points = []            # clear old path first
            with open(file_path, 'r') as f:  # open path file
                # File format: one point per line
                # "x, y, z, qx, qy, qz, qw" (meters + quaternion)
                for line in f:              # read line by line
                    line = line.strip()     # strip leading/trailing whitespace
                    if not line:            # empty line:
                        continue            # skip
                    parts = line.split(',')  # split by comma
                    if len(parts) >= 7:     # need at least 7 fields to be valid
                        point = {           # build waypoint dict (all to float)
                            'x': float(parts[0]),   # x coord (m)
                            'y': float(parts[1]),   # y coord (m)
                            'z': float(parts[2]),   # z coord (m, always 0 on planar map)
                            'qx': float(parts[3]),  # quaternion x
                            'qy': float(parts[4]),  # quaternion y
                            'qz': float(parts[5]),  # quaternion z (stores heading)
                            'qw': float(parts[6])   # quaternion w (stores heading)
                        }
                        self.path_points.append(point)  # append to waypoint list
            if self.path_points:                       # parsed valid points:
                self.path_loaded = True                # mark path loaded
                self.current_target_idx = 0            # target segment from start
                # File-loaded paths have no mid-route shoot stops.
                self.shoot_flags = [False] * len(self.path_points)  # all non-shoot
                # 文件路径点很密，逐个停站扫描会把路线拖得很长 —— 默认不触发扫描
                self.scan_flags = [False] * len(self.path_points)
                self._start_mission_scan()
                self.get_logger().info(
                    f"Loaded {len(self.path_points)} path points from {file_path}"
                )
            else:                                      # no valid points at all:
                self.get_logger().warning(f"No valid points found in {file_path}")

        except FileNotFoundError:  # file does not exist:
            self.get_logger().error(f"Path file not found: {file_path}")
        except Exception as e:     # other parse/read exceptions:
            self.get_logger().error(f"Failed to load path: {e}")

    # ##### Command entry (/nav/waypoint callback) #####
    # Dispatches four command types: load: path file / reset: re-anchor odom /
    # single target 'M10,y' / multi-waypoint route 'G8 A10 D2*' (-> follow_route_command).

    def waypoint_callback(self, msg):
        """Handle waypoint command."""
        try:
            data = msg.data.strip()  # extract command text, strip whitespace

            # Check load command
            # 'load:/path/to/file.txt' -> switch to path-file mode
            if data.startswith('load:'):      # load: prefix:
                file_path = data[5:].strip()  # take file path after the colon
                self.load_path(file_path)     # call file loader
                return                        # done

            # Manual odometry re-anchor: 'reset:block,heading'
            # Used after manually moving/rotating the car. Must be sent before
            # each route: the car is "placed" (not driven) to the start, so dead
            # reckoning has no idea where it is. This command forces the belief
            # pose to the specified block + heading. e.g. 'reset:N10,y' = "I'm at N10, facing Y+".
            # serial_reader also listens and resets its own DR heading integral (v4-dr-reset version).
            if data.startswith('reset:'):     # reset: prefix:
                try:
                    rp = data[6:].split(',')  # split into "block,heading"
                    rblock = rp[0].strip().upper()   # block name, e.g. 'N10'
                    rhead = rp[1].strip().lower()    # heading, 'x' or 'y'
                    tx, ty = block_to_meters(rblock)          # target position (m)
                    tqx, tqy, tqz, tqw = heading_to_quaternion(rhead)  # target pose quaternion
                    tyaw = euler_from_quaternion(tqx, tqy, tqz, tqw)   # target yaw
                    # Offset = target - raw, so raw + offset == target.
                    # Takes effect permanently in odom_callback.
                    self.snap_x_off = tx - self.raw_x   # reset x snap offset
                    self.snap_y_off = ty - self.raw_y   # reset y snap offset
                    self.snap_yaw_off = normalize_angle(tyaw - self.raw_map_yaw)  # reset yaw offset
                    self.current_x = tx   # belief position immediately equals target
                    self.current_y = ty
                    self.current_yaw = tyaw
                    # Discard prior route/state; the car was just re-placed.
                    self.path_points = []     # clear old path
                    self.shoot_flags = []     # clear shoot flags
                    self.scan_flags = []      # clear scan flags
                    self.scan_wait = False    # 退出扫描停站状态
                    self.path_loaded = False  # mark no path
                    self.arrived = False      # reset arrival latch
                    self.arrive_logged = False
                    self.shoot_wait = False   # exit shoot wait
                    self.corner_mode = False  # exit in-place turn
                    self.corner_phase = 0     # reset phase
                    self.reverse_flags = []   # ★清空倒车标记
                    self.align_active = False # ★退出桥洞强制直行
                    self.get_logger().info(
                        f"Odometry reset to {rblock} facing {rhead}"
                    )
                except Exception as e:
                    self.get_logger().error(f"Failed to reset: {e}")
                return  # reset done

            # ---- Command dispatch ----------------------------------------
            # 'G8 A10' / 'G8 D2* E2,y' -> route command (straight lines between
            #     waypoints, diagonals allowed, turn angles auto-computed; '*' = mid-route shoot stop)
            # 'M10,y'                  -> traditional single-target command
            # Dispatch rule: single-target has exactly one comma with no space
            # before it ('M10,y'); everything else is treated as a route.
            parts = data.split(',')                       # split by comma
            if len(parts) != 2 or ' ' in parts[0]:        # not single-target shape:
                self.follow_route_command(data)           # route command parse
                return                                    # done

            # Traditional single-target command: "M10,y" or "M7,x"
            # (trailing '*' is also accepted, equivalent to a normal end stop -
            #   end arrival opens gimbal + shooting anyway)
            block_name = parts[0].strip().upper().rstrip('*')  # block name (strip '*', upper-case)
            heading_dir = parts[1].strip().lower()             # heading letter (lower-case)

            if heading_dir not in ['x', 'y']:  # invalid heading:
                self.get_logger().error(f"Invalid heading: '{heading_dir}'. Use 'x' or 'y'")
                return                         # reject

            # Block name -> metric coords
            target_x, target_y = block_to_meters(block_name)   # target block center
            qx, qy, qz, qw = heading_to_quaternion(heading_dir)  # target heading quaternion

            # Build path: insert a straight approach segment along the specified
            # axis before the end so the car arrives already facing the specified
            # direction (pure pursuit naturally straightens within the last 0.6 m).
            if heading_dir == 'y':           # approach along Y+:
                axis_x, axis_y = 0.0, 1.0    # approach axis direction vector
            else:                            # approach along X+:
                axis_x, axis_y = 1.0, 0.0    # approach axis direction vector

            # Approach point = target stepped back 0.6 m along approach axis
            # (no lower than 0.05 m, to stay inside the map boundary).
            app_x = max(0.05, target_x - axis_x * self.approach_dist)  # approach point x
            app_y = max(0.05, target_y - axis_y * self.approach_dist)  # approach point y

            # Only add the approach point if the car is not on the approach line,
            # or is still behind the approach point. Both "along-axis position"
            # and "lateral offset" must be checked: checking only one would let a
            # car approaching from the side skip the alignment segment and end
            # with an (incomplete) sharp corner.
            rel_x = self.current_x - app_x     # car - approach point vector x
            rel_y = self.current_y - app_y     # car - approach point vector y
            along = rel_x * axis_x + rel_y * axis_y     # dot product = along-axis projection
            lateral = abs(rel_x * axis_y - rel_y * axis_x)  # |cross| = lateral offset
            points = []                        # new waypoint list
            if not (along >= -0.1 and lateral <= 0.15):  # car not on approach line / not past approach point:
                points.append({                # add approach point first
                    'x': app_x, 'y': app_y, 'z': 0.0,
                    'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw
                })
            points.append({                    # then add target end
                'x': target_x, 'y': target_y, 'z': 0.0,
                'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw
            })
            self.path_points = points          # commit new path
            self.current_target_idx = 0        # target segment from start
            self.shoot_flags = [False] * len(points)  # single target has no mid-route shoot
            # 单点目标命令不支持倒车（语法里没有 R: 的位置），一律正向
            self.reverse_flags = [False] * len(points)
            # 单点目标：只对真正的目标点做一次全向扫描，插入的接近段起点不扫描
            self.scan_flags = [True] * len(points)
            if len(self.scan_flags) > 1:
                self.scan_flags[0] = False
            self._start_mission_scan()
            self.path_loaded = True            # path ready
            self.arrived = False               # reset arrival latch
            self.arrive_logged = False         # reset arrival log guard
            self.shoot_wait = False            # exit shoot wait
            self.corner_mode = False           # exit in-place turn
            self.corner_phase = 0              # reset phase

            self.get_logger().info(
                f"Navigating to {block_name} ({target_x:.2f}m, {target_y:.2f}m), facing {heading_dir}"
            )
            self.get_logger().info(
                f"Current: ({self.current_x:.2f}m, {self.current_y:.2f}m), "
                f"yaw={self.current_yaw:.2f}rad"
            )

        except Exception as e:  # any parse exception:
            self.get_logger().error(f"Failed to parse command: {e}")

    # ##### Route command parsing #####
    # Expand multi-waypoint commands like 'G8 A10 D2* E2,y' into a straight-line
    # waypoint sequence, auto-compute each point's heading, dedupe, optionally
    # insert an axis-aligned approach segment before the end.

    def follow_route_command(self, data):
        """Drive through a sequence of straight-line-connected waypoints in order.

        'G8 A10'      -> straight line G8->A10 (diagonals allowed); turn angle
                         at each waypoint auto-computed (pure pursuit transitions
                         to the next segment automatically)
        'G8 D2* E2'   -> same, but stop at D2, fire SHOOT_COUNT_TARGET shots, then continue
        'G8 A10 C2,y' -> same, final stop facing Y+ axis
        'A10'         -> single waypoint: drive straight from current position
        """
        # Optional trailing ',x' / ',y' forces the final heading.
        final_heading = None  # default: no force
        body = data           # command body (after stripping trailing heading)
        if ',' in data:                       # comma present in command:
            body, tail = data.rsplit(',', 1)  # split off the final heading field from the right
            final_heading = tail.strip().lower()  # 'x' or 'y'
            if final_heading not in ('x', 'y'):   # invalid heading:
                self.get_logger().error(
                    f"Invalid heading: '{tail}'. Use 'x' or 'y'")
                return                            # reject
        tokens = body.split()      # split waypoints by whitespace
        if not tokens:             # empty waypoints:
            self.get_logger().error(f"Empty route command: '{data}'")
            return                 # reject

        points = []          # waypoint list (metric)
        shoot_flags = []     # True = this point is a shoot stop
        scan_flags = []      # True = 到达该点位时触发一次全向扫描
        reverse_flags = []   # ★True = 倒着走到这个点（R: 前缀）
        shoot_groups = []    # ★同一靶标的候选射击点共用一个组号；
        for tok in tokens:                       # process each waypoint token:
            t = tok.strip()                      # strip whitespace
            if not t:                            # 空 token（多余空格）:
                continue                         # 跳过
            # ★★ SHOOT@<靶标> 指令：为靶标自动计算最佳射击点并插入路线 ★★
            #   例：'SHOOT@F1' -> 现场按 shoot_planner 的评分插进最优射击点，
            #       并把备选点也一并插进去（打不中就自动换点）。
            #   这是"最佳射击点自主计算"的入口。
            if t.upper().startswith('SHOOT@'):
                tgt_name = t[6:].strip().upper()
                # 内联校验（本模块没有 parse_block，它在 shoot_planner 里）
                if not (len(tgt_name) >= 2 and tgt_name[0] in COL_LETTERS
                        and tgt_name[1:].isdigit()
                        and 1 <= int(tgt_name[1:]) <= MAP_ROWS):
                    self.get_logger().error(
                        f"SHOOT@ 后面不是合法靶标名: '{tgt_name}'")
                    return
                chain = self.compute_shoot_chain(tgt_name)
                # 整条候选链都插进去，共用同一个 group id；
                # 实际执行时打中一个就跳过同组剩余点。
                grp = getattr(self, '_shoot_group_seq', 0) + 1
                self._shoot_group_seq = grp
                for cn in chain:
                    px, py = block_to_meters(cn)     # 格名 -> 米制
                    points.append({'x': px, 'y': py, 'z': 0.0,
                                   'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0})
                    shoot_flags.append(True)
                    scan_flags.append(True)
                    reverse_flags.append(False)
                    shoot_groups.append(grp)
                self.get_logger().warn(
                    f"SHOOT@{tgt_name}: 已插入 {len(chain)} 个候选射击点 "
                    f"[组 {grp}] {' '.join(chain)}")
                continue

            # ★倒车前缀 'R:'：'R:B5' = 倒着走到 B5。
            #   与射击后缀 '*' 可叠加，但【只有一种合法顺序】：'R:B5*'
            #   （R: 必须在前，* 必须在后 —— 与路径规划器的输出格式一致）
            rev = False
            if t.upper().startswith('R:'):
                rev = True
                t = t[2:].strip()
            # ★':SHOOT_STOP' 标签写法：'F2:SHOOT_STOP' 等价于 'F2*'，
            #   但可读性好得多，也更适合放在配置中心里人工维护。
            #   （顺手删掉了一段永远不成立的 '*R:B5' 解析死代码 ——
            #     唯一合法顺序是 R: 在前、* 在后。）
            if ':' in t:
                _head, _tail = t.split(':', 1)
                _tail_u = _tail.strip().upper()
                if _tail_u in ('SHOOT_STOP', 'SHOOT'):
                    t = _head.strip() + '*'      # 归一成内部统一的 '*' 表示
                elif _tail_u in ('SCAN_STOP', 'SCAN'):
                    t = _head.strip()            # 扫描停站：只扫描不射击
                # 其它 :xxx 后缀保持原样，交给下面的合法性检查去报错
            shoot = t.endswith('*')              # 'D2*' -> stop & shoot at D2
            b = (t[:-1] if shoot else t).upper() # block name without '*'
            # Valid block = letter A-N + digits, e.g. 'G8', 'D2', 'N10'
            if (len(b) < 2 or b[0] not in COL_LETTERS
                    or not b[1:].isdigit()):     # invalid format:
                self.get_logger().error(
                    f"Invalid waypoint '{tok}'. Use blocks like 'G8' / 'D2*' "
                    "('*' = mid-route shoot stop) / 'R:B5' ('R:' = 倒车到位), "
                    "e.g. 'G8 R:B5 D2* E2 C2,y'")
                return                           # reject
            px, py = block_to_meters(b.upper())  # block name -> metric coords
            # Dedup: a repeated block ('D2 D2*') does not create a second point,
            # it just marks the existing one as a shoot stop.
            # ★注意：若重复点的 R: 标记不同（'B5 R:B5'），说明是"先正向后反向
            #   经过同一点"，这种情况【不合并】—— 合并会让倒车段丢失。
            same_as_prev = (points
                            and abs(points[-1]['x'] - px) < 1e-9
                            and abs(points[-1]['y'] - py) < 1e-9)
            if same_as_prev and reverse_flags[-1] == rev:   # 同坐标且同模式:
                if shoot:                        # and has '*':
                    shoot_flags[-1] = True       # mark previous as shoot point
                continue                         # do not add a new point
            points.append({'x': px, 'y': py, 'z': 0.0,   # new waypoint (pose placeholder)
                           'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0})
            shoot_flags.append(shoot)            # record whether shoot stop
            scan_flags.append(True)              # 路线点位一律触发一次全向扫描
            reverse_flags.append(rev)            # ★记录是否倒车到位
            # 显式写出的射击点各自成组（组号 0 = 不参与"打中即跳过同组"逻辑）
            shoot_groups.append(0)

        if not points:  # no valid waypoints:
            self.get_logger().error(f"No valid waypoints in '{data}'")
            return          # reject

        # Last point heading: use trailing heading if present; otherwise use the
        # natural direction of the last segment (single-waypoint routes keep the
        # current heading).
        if len(points) >= 2:  # multiple points: heading = direction of last segment
            last_yaw = math.atan2(points[-1]['y'] - points[-2]['y'],
                                  points[-1]['x'] - points[-2]['x'])
        else:                 # single point: keep current heading
            last_yaw = self.current_yaw
        if final_heading == 'y':   # force Y+:
            final_yaw = math.pi / 2
        elif final_heading == 'x': # force X+:
            final_yaw = 0.0
        else:                      # not forced: use what was computed above
            final_yaw = last_yaw

        # Only insert an axis-aligned end approach segment when both "a final
        # heading is forced" and "the last segment does not arrive along that axis".
        # cos(angle between segments) < 0.999 = off-axis by more than ~2.6 deg.
        if final_heading is not None \
                and abs(math.cos(last_yaw - final_yaw)) < 0.999:  # last segment not along target axis:
            axis_x, axis_y = ((1.0, 0.0) if final_heading == 'x'
                              else (0.0, 1.0))   # approach axis direction vector
            tx, ty = points[-1]['x'], points[-1]['y']  # end coords
            # Step back 0.6 m from the end along the forced axis to get the
            # approach point (skip if it would leave the map).
            app_x = max(0.05, tx - axis_x * self.approach_dist)  # approach point x
            app_y = max(0.05, ty - axis_y * self.approach_dist)  # approach point y
            if abs(app_x - tx) > 1e-9 or abs(app_y - ty) > 1e-9:  # approach point != end:
                # Insert before the end (shoot flag stays on the end; approach
                # point is never a shoot point).
                points.insert(len(points) - 1,
                              {'x': app_x, 'y': app_y, 'z': 0.0,
                               'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0})
                shoot_flags.insert(len(shoot_flags) - 1, False)  # insert matching non-shoot flag
                scan_flags.insert(len(scan_flags) - 1, False)    # 插入的接近段起点不扫描
                # 接近段本身永远正向行驶；原终点保持它自己的倒车标记
                reverse_flags.insert(len(reverse_flags) - 1, False)
                shoot_groups.insert(len(shoot_groups) - 1, 0)

        # Each point's quaternion encodes "the heading the car should have at
        # that point": intermediate points = direction of the leaving segment,
        # end point = final heading. Pure pursuit does not use the quaternions
        # while driving (it steers by position), but arrival snap and logs read them.
        for i, p in enumerate(points):            # backfill quaternions per point:
            if i + 1 < len(points):               # intermediate: heading = toward next point
                yw = math.atan2(points[i + 1]['y'] - p['y'],
                                points[i + 1]['x'] - p['x'])
            else:                                 # last point: heading = final heading
                yw = final_yaw
            p['qz'] = math.sin(yw / 2.0)          # quaternion Z
            p['qw'] = math.cos(yw / 2.0)          # quaternion W

        # Commit the new route, clearing all one-shot task state.
        self.path_points = points            # waypoint sequence
        self.current_target_idx = 0          # target segment from start
        self.shoot_flags = shoot_flags       # shoot-stop flags
        self.scan_flags = scan_flags         # 待扫描点位标记
        self.reverse_flags = reverse_flags   # ★倒车段标记
        self.shoot_groups = shoot_groups     # ★射击点分组（同组 = 同一靶的候选）
        self._rebuild_align_segments()       # ★预计算桥洞直行保护航段
        n_rev = sum(1 for f in reverse_flags if f)
        if n_rev:
            self.get_logger().warn(
                f"路线含 {n_rev} 个倒车航点（R: 前缀），"
                f"倒车速度 = 巡航 x {REVERSE_SPEED_RATIO}")
        self._start_mission_scan()        # 按任务扫描模式启动云台扫描
        self.path_loaded = True           # path ready
        self.arrived = False              # reset arrival latch
        self.arrive_logged = False        # reset arrival log guard
        self.shoot_wait = False           # exit shoot wait
        # Reset any in-place turn state left over from the previous route,
        # to avoid starting stuck in corner phase 0/1.
        self.corner_mode = False
        self.corner_phase = 0
        self.get_logger().info(
            f"Route ({len(points)} pts): {' '.join(tokens)} | "
            f"straight lines between waypoints, diagonals allowed")
        self.get_logger().info(
            f"Current: ({self.current_x:.2f}m, {self.current_y:.2f}m), "
            f"yaw={self.current_yaw:.2f}rad")

    # ##### Sensor input #####
    # shoot_count_cb: cache cumulative shot count; odom_callback: convert
    # serial_reader's block-unit localization to meters and apply the snap offset
    # to produce the "belief pose" used by the controller.

    def shoot_count_cb(self, msg):
        """Track dzrknn_dataprocess cumulative shot count."""
        # dzrknn publishes a cumulative value (never resets), so the wait logic
        # compares against the "snapshot at arrival".
        self.shoot_count = int(msg.data)  # cache latest cumulative shot count

    # ##### 底盘速度估计（开火门控要用）#####
    #  只有 IMU + 轮速计，没有激光。速度就用里程计位置差分算 ——
    #  对"车是否真的停住了"这个判断来说，位置差分比轮速更可信
    #  （轮速在轮子打滑时会有读数，而位置不动就是不动）。
    def _update_chassis_speed(self):
        now_t = self.get_clock().now().nanoseconds / 1e9
        if self._prev_pose is not None and self._prev_pose_t > 0:
            dt = now_t - self._prev_pose_t
            if dt > 1e-3:
                d = math.hypot(self.current_x - self._prev_pose[0],
                               self.current_y - self._prev_pose[1])
                inst = d / dt
                # 一阶低通，避免单帧里程计抖动让速度忽大忽小
                self.chassis_speed = 0.7 * self.chassis_speed + 0.3 * inst
        self._prev_pose = (self.current_x, self.current_y)
        self._prev_pose_t = now_t

    def offset_center_cb(self, msg):
        """缓存视觉检测结果（打靶稳定判据的唯一数据源）。"""
        try:
            d = list(msg.data)
            if len(d) < 3:
                return
            self.det_dx = int(d[0])
            self.det_dy = int(d[1])
            self.det_cls = int(d[2])
            self.det_box_w = int(d[3]) if len(d) > 3 else 0
            self.det_t = self.get_clock().now().nanoseconds / 1e9
        except Exception:
            pass

    def aim_status_cb(self, msg):
        """Cache the gimbal aim state machine status (0=IDLE 1=SCAN 2=TRACK)."""
        self.aim_status = int(msg.data)

    def manual_override_cb(self, msg):
        """手动接管开关：1=接管（停止自动下发），0=释放（恢复自动循迹）。"""
        on = (int(msg.data) != 0)
        if on == self.manual_override:
            return
        self.manual_override = on
        if on:
            self.get_logger().warn(
                "★ 收到【手动接管】—— 循迹暂停下发指令，控制权交给浏览器遥控器")
        else:
            self.get_logger().warn(
                "★ 收到【手动释放】—— 恢复自动循迹（下一条指令即接管控制）")

    def enter_scan_stop(self, wp_idx):
        """在某点位停车，并请求云台做一次全向扫描（左一趟 + 右一趟）。

        循迹优先级最高：这里只短暂停车，并且 control_loop 里有硬超时兜底 ——
        即使云台异常/丢包，超时后也会无条件继续循迹，绝不会卡住路线。
        """
        self.cmd_pub.publish(Twist())             # 零 Twist = 刹车
        self.scan_wait = True
        self.scan_wait_start = self.get_clock().now().seconds_nanoseconds()[0]
        self.scan_seen_scanning = False
        self.scan_flags[wp_idx] = False           # 一次性触发，避免重复停站
        self.scan_req_pub.publish(UInt8(data=1))  # 请求云台全向扫描
        self.get_logger().info(
            f"点位 #{wp_idx} 停车，已请求云台全向扫描（最多等 {self.SCAN_WAIT_TIMEOUT:.0f}s）"
        )

    def _banner(self, lines):
        """打印带边框的横幅，方便在滚屏日志里一眼看到。"""
        bar = "=" * 64
        self.get_logger().warn("\n" + bar + "\n" + "\n".join(lines) + "\n" + bar)

    def _print_start_banner(self):
        self._banner([
            "  >>>  任 务 开 始  <<<",
            f"  航点数     : {len(self.path_points)}",
            f"  扫描模式   : {self.mission_scan_mode}",
            f"  巡航速度   : {getattr(self, 'linear_speed', 0):.2f} m/s",
            f"  桥洞保护   : {' '.join(n for _, _, n in self._bridge_points) or '(未配置)'}"
            f"  洞前{ALIGN_STRAIGHT_BEFORE}格起锁航向直行",
        ])

    def _print_finish_banner(self):
        """★ 运行完成提示符：一眼确认全程已跑完。"""
        elapsed = 0.0
        t0 = getattr(self, 'mission_start_time', None)
        if t0 is not None:
            elapsed = self.get_clock().now().nanoseconds / 1e9 - t0
        shots = getattr(self, 'shoot_count', 0) or 0

        # ★P1-1 漂移汇总：整趟跑下来的定位可信度一目了然
        legs = getattr(self, 'drift_legs', 0)
        if legs > 0:
            drift_line = (f"  定位漂移   : 最大 {self.drift_max:.3f}m / "
                          f"航向 {self.drift_yaw_max:.1f}deg（{legs} 段）")
            if self.drift_warn_count:
                drift_line += f"  ⚠️ 超阈值 {self.drift_warn_count} 次"
            else:
                drift_line += "  ✅ 全段正常"
        else:
            drift_line = "  定位漂移   : (无到位记录)"

        self._banner([
            "  ★★★  全 程 已 执 行 完 毕  ★★★",
            f"  总航点数   : {len(self.path_points)}",
            f"  行驶用时   : {elapsed:.1f} s",
            f"  累计开火   : {shots} 发",
            drift_line,
            "  车辆状态   : 已停车；云台正在做终点扫描",
            "  重跑       : ./start_all.sh restart",
            "  停止       : ./start_all.sh stop",
        ])

        # ★完成提示音（默认关闭，见 FINISH_VOICE_ID 注释）
        if FINISH_VOICE_ID > 0:
            try:
                self.voice_pub.publish(UInt8(data=FINISH_VOICE_ID))
                self.get_logger().info(
                    f"已播放完成提示音 (voice id={FINISH_VOICE_ID})")
            except Exception as exc:
                self.get_logger().warn(f"完成提示音发布失败: {exc}")

    def _publish_motion(self, reversing):
        """把当前运动状态发出去（只在变化时发）。"""
        v = 1 if reversing else 0
        if v != getattr(self, '_motion_last', -1):
            self._motion_last = v
            try:
                self.motion_pub.publish(UInt8(data=v))
            except Exception:
                pass

    def _start_mission_scan(self):
        """任务加载后，按 mission_scan_mode 决定云台的扫描方式。

        'continuous'（默认）= 立即让云台持续左右横扫，行进途中不停车 —— 边走边扫
        'waypoint'   = 保持"到达每个点位停车扫描"的旧握手
        'none'       = 不自动扫描
        """
        # 记录任务起始时刻，供结束横幅算总用时
        self.mission_start_time = self.get_clock().now().nanoseconds / 1e9
        self._print_start_banner()

        if self.mission_scan_mode == 'continuous':
            self.scan_flags = [False] * len(self.path_points)  # 不停站
            self.scan_req_pub.publish(UInt8(data=2))           # 请求持续扫描
            self.get_logger().info("任务扫描模式：持续横扫（边走边扫）")
        elif self.mission_scan_mode == 'waypoint':
            self.scan_req_pub.publish(UInt8(data=0))           # 先停掉可能残留的持续扫描
            self.get_logger().info("任务扫描模式：点位停站扫描")
        else:
            self.scan_flags = [False] * len(self.path_points)
            self.scan_req_pub.publish(UInt8(data=0))
            self.get_logger().info("任务扫描模式：不自动扫描")

    def _auto_start_mission(self):
        """启动延时到点后自动下发内置路线（只执行一次）。

        走的是和外部 /nav/waypoint 命令完全相同的路径（reset -> follow_route_command），
        所以行为一致，只是不需要人为触发。
        注意：本节点把日志级别压到了 WARN，这里用 warn() 输出关键信息以便现场确认。
        """
        if self._auto_started:
            return
        self._auto_started = True
        if self._auto_start_timer is not None:
            try:
                self._auto_start_timer.cancel()  # 一次性定时器，触发后立即取消
            except Exception:
                pass

        if not MISSION_BLOCKS:
            self.get_logger().warn("MISSION_BLOCKS 为空，未自动下发路线")
            return

        # 1) 可选：先把里程计重锚到已知点位
        if MISSION_RESET:
            m = String()
            m.data = 'reset:' + MISSION_RESET
            self.waypoint_callback(m)
            self.get_logger().warn(f"已重锚里程计到 {MISSION_RESET}")

        # 2) 下发内置路线（与外部命令同一条代码路径）
        # ★路线合法性：不合法就【明确报错但不阻止下发】——
        #   follow_route_command 自己会拒绝非法 token，车不会带着错路线乱跑；
        #   这里的作用是让问题在日志里【点名可见】，而不是淹没在一行 error 里。
        if not MISSION_BLOCKS_VALID:
            self.get_logger().error(
                "❌ MISSION_BLOCKS 存在 " + str(len(MISSION_BLOCKS_PROBLEMS)) +
                " 处问题，路线可能无法下发：")
            for _p in MISSION_BLOCKS_PROBLEMS:
                self.get_logger().error("     " + _p)
            self._banner(
                ["  ★★★  路 线 配 置 有 误  ★★★"] +
                ["     " + p for p in MISSION_BLOCKS_PROBLEMS] +
                ["  请检查 config/robot.yaml 的 path_executor.mission_blocks",
                 "  合法格式: K7 / R:K7(倒车) / F2*(射击) / N5,y(终点朝向)"])

        _st = MISSION_BLOCKS_STATS
        self.get_logger().warn(
            f"路线构成：共 {_st['total']} 点 | 普通 {_st['normal']} | "
            f"倒车(R:) {_st['reverse']} | 射击(*) {_st['shoot']}")

        route = ' '.join(MISSION_BLOCKS)
        self.get_logger().warn(
            f"自动出发：{len(MISSION_BLOCKS)} 个坐标点 -> {route}")
        self.follow_route_command(route)
        self.get_logger().warn(
            f"任务扫描模式 = {self.mission_scan_mode}"
            "（continuous = 云台边走边扫，循迹全程不停车）")

    def enter_shoot_stop(self, wp_idx):
        """Stop at a mid-route shoot waypoint, allow gimbal + shooting."""
        # Zero Twist = immediate stop: linear.x=0 -> dzactuator TargetSpeed=0
        # -> serial frame speed byte 0 -> STM32 motor PID target speed 0, closed-loop brake.
        cmd = Twist()                 # all-zero control command
        self.cmd_pub.publish(cmd)     # publish stop command

        b = self.path_points[wp_idx]   # shoot waypoint (stop here)

        # Snap belief pose to block center, heading to the entering segment direction,
        # so the gimbal faces the driving direction.
        # (Same anchoring logic as end arrival: both position and heading are forced,
        #   drift is cleared during the stop.)
        a = self.path_points[max(0, wp_idx - 1)]                 # previous waypoint (entering segment start)
        leg_yaw = math.atan2(b['y'] - a['y'], b['x'] - a['x'])   # entering segment heading
        drift_x = self.current_x - b['x']   # belief - block center (x drift)
        drift_y = self.current_y - b['y']   # belief - block center (y drift)
        self.snap_x_off -= drift_x          # fold x drift into snap offset
        self.snap_y_off -= drift_y          # fold y drift into snap offset
        self.current_x = b['x']             # force belief position = block center
        self.current_y = b['y']
        yaw_drift_deg = math.degrees(
            normalize_angle(self.current_yaw - leg_yaw))         # heading drift (deg)
        self.snap_yaw_off = normalize_angle(
            self.snap_yaw_off - math.radians(yaw_drift_deg))     # fold into yaw offset
        self.current_yaw = leg_yaw          # force belief heading = entering segment direction

        # One-shot: clear the flag to prevent re-triggering.
        self.shoot_flags[wp_idx] = False  # this shoot point is handled

        # Enter wait state (handled at the top of control_loop).
        self.shoot_wait = True            # set wait flag
        now_t = self.get_clock().now().seconds_nanoseconds()[0]
        self.shoot_wait_start = now_t     # record start time
        self.shoot_count_at_arrival = self.shoot_count  # snapshot current cumulative shots

        # ★打靶三段式状态机复位：从"等停稳"开始。
        #   旧逻辑一到这里就直接放行开火，车还在惯性前冲、云台还没锁定，
        #   打出去的弹基本是浪费的。
        self.shoot_phase = 0
        self.shoot_phase_t = now_t
        self.shoot_settle_x = self.current_x
        self.shoot_settle_y = self.current_y
        self.shoot_settle_since = now_t
        self.shoot_lock_seen = False
        self.scan_rounds_done = 0        # ★两轮扫描计数复位
        self.scan_round_active = False

        # ★这里【刻意不放行开火】。
        #   旧逻辑一到点就发 stop_signal=1，结果是"车还在惯性前冲、云台还没转到位"
        #   就已经允许射击 —— 全场只有 10 发弹，这样打出去的基本是浪费。
        #   新逻辑把放行推迟到 shoot_phase==3，由 control_loop 在
        #   "底盘停稳 + 看到靶 + 瞄准稳定" 三个条件同时成立后才发。
        #   先收回许可，确保上一站残留的状态不会漏过来。
        sig = UInt8()
        sig.data = 0                   # 0 = 暂不许可
        self.stop_signal_pub.publish(sig)

        # ★记录本次停站的候选点信息（供换点重试与同组取消用）
        self.shoot_wp_idx = wp_idx
        self.shoot_attempt += 1
        self.stable_count = 0          # 稳定帧计数清零
        self._shoot_allow_sent = False # 本候选点是否已放过开火许可

        _gid = 0
        if wp_idx < len(self.shoot_groups):
            _gid = self.shoot_groups[wp_idx]
        self.shoot_resume_gid = _gid
        _cand_info = (f"，候选组 {_gid}" if _gid else "")
        self.get_logger().info(
            f"射击停站 wp#{wp_idx}{_cand_info}（第 {self.shoot_attempt} 次尝试）: "
            f"停稳->找靶->稳定{SHOOT_STABLE_FRAMES}帧->开火；"
            f"单点硬超时 {SHOOT_TIMEOUT:.0f}s，失败自动换下一个候选点"
        )

    def odom_callback(self, msg):
        """Update current position."""
        # Data origin (full path serial -> this callback):
        #   STM32 reports encoder odom via /dev/ttyUSB0 (@115200, opened by dzactuator);
        #   dzactuator parses and publishes /odom;
        #   serial_reader subscribes /odom+/imu_data: heading from IMU (2s timeout
        #   fallback to odom), position via "step x map heading" dead reckoning,
        #   publishes /odom/localization;
        #   its position is in "block" units -> here x0.35 converts to meters (1 block = 0.35 m).
        # This is the raw dead-reckoning value; the belief pose the controller sees
        # also includes the snap offset.
        self.raw_x = msg.pose.pose.position.x * BLOCK_SIZE  # block coord x -> m
        self.raw_y = msg.pose.pose.position.y * BLOCK_SIZE  # block coord y -> m
        self.current_x = self.raw_x + self.snap_x_off  # belief x = raw + snap offset
        self.current_y = self.raw_y + self.snap_y_off  # belief y = raw + snap offset
        self.has_odom = True  # localization received, control loop may run

        q = msg.pose.pose.orientation                       # quaternion pose
        raw_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)  # parse raw yaw
        # Raw chassis yaw -> map frame (raw X+ front = map Y+)
        self.raw_map_yaw = normalize_angle(raw_yaw + RAW_YAW_TO_MAP_YAW_OFFSET)
        # Then add the heading snap offset (set by reset / waypoint arrival)
        self.current_yaw = normalize_angle(self.raw_map_yaw + self.snap_yaw_off)

    # ##### Pure pursuit geometry 1: nearest segment & projection #####

    def find_nearest_segment(self):
        """Find the nearest path segment + the car's projection onto it.

        Searches only forward from the current target segment (never looks back
        at segments already traveled, to avoid re-locking onto a segment behind the car).

        Returns dict {'idx','t','px','py','dist'} or None:
          idx   = index of segment start p1 (segment i = p1->p2)
          t     = projection parameter of the car on the segment, clamped to [0,1] (0=at p1, 1=at p2)
          px/py = projection point coords (point on path closest to the car)
          dist  = perpendicular distance from car to segment (lateral error)
        """
        pts = self.path_points              # waypoint list
        if not self.path_loaded or len(pts) < 2:  # no path or too few points:
            return None                     # cannot form a segment

        best = None  # best segment record
        # Search from current_target_idx: only look at remaining segments ahead.
        for i in range(self.current_target_idx, len(pts) - 1):  # iterate remaining segments
            p1 = pts[i]    # segment start
            p2 = pts[i + 1]  # segment end
            sx = p2['x'] - p1['x']   # segment vector AB x component
            sy = p2['y'] - p1['y']   # segment vector AB y component
            seg2 = sx * sx + sy * sy          # |AB|^2 (segment length squared)
            if seg2 < 1e-12:                  # zero-length segment:
                continue                      # skip
            # Vector projection: t = dot(AP, AB) / |AB|^2, clamped to segment ends -
            # "behind p1" counts as t=0, "past p2" counts as t=1.
            t = ((self.current_x - p1['x']) * sx
                 + (self.current_y - p1['y']) * sy) / seg2  # projection parameter
            t = max(0.0, min(1.0, t))         # clamp to [0,1]
            qx = p1['x'] + t * sx             # nearest point on segment (projection) x
            qy = p1['y'] + t * sy             # nearest point y
            d = math.hypot(self.current_x - qx, self.current_y - qy)  # car-to-projection distance
            if best is None or d < best['dist']:  # closer segment:
                best = {'idx': i, 't': t, 'px': qx, 'py': qy, 'dist': d}  # record it
        return best  # return nearest segment (or None)

    # ##### Pure pursuit geometry 2: lookahead aim point #####

    def find_lookahead_point(self, seg):
        """From the car's projection on the nearest segment, take the aim point on the path exactly lookahead_dist ahead.

        Corner protection: when the angle between the current segment and the next
        exceeds CORNER_ANGLE_RAD, the lookahead point is clamped to the current
        segment's end (the vertex). Prevents the car from cutting the corner
        before reaching the vertex. For near-linear transitions (small angle) the
        lookahead point slides into the next segment normally.
        """
        pts = self.path_points                    # waypoint list
        if len(pts) < 2:                          # too few points:
            return pts[-1] if pts else None       # aim straight at the last point

        def _pt(x, y):
            # Synthesize a path point (pure pursuit does not use the quaternion).
            return {'x': x, 'y': y, 'z': 0.0,
                    'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}

        # Angle between two segments above this counts as a "corner" (soft guard;
        # the hard threshold for in-place turn in control_loop is 45 deg).
        CORNER_ANGLE_RAD = math.radians(20.0)     # corner judgment angle (20 deg)

        # Lookahead budget: how many meters still to travel along the current segment.
        remaining = self.lookahead_dist   # 0.2 m
        idx = seg['idx']                  # current segment index
        p1 = pts[idx]                     # segment start
        p2 = pts[idx + 1]                 # segment end
        seg_len = math.hypot(p2['x'] - p1['x'], p2['y'] - p1['y'])  # segment length
        # How much is left on the current segment (from projection to segment end)
        left = (1.0 - seg['t']) * seg_len if seg_len > 1e-9 else 0.0

        # Corner check: is the angle between the next segment and current segment sharp?
        corner_ahead = False                       # default: no corner ahead
        if idx + 2 < len(pts):                     # only check if a next segment exists:
            d1x = p2['x'] - p1['x']   # current segment direction x
            d1y = p2['y'] - p1['y']   # current segment direction y
            p3 = pts[idx + 2]
            d2x = p3['x'] - p2['x']   # next segment direction x
            d2y = p3['y'] - p2['y']   # next segment direction y
            cross = d1x * d2y - d1y * d2x   # cross product z (sign = turn direction)
            dot = d1x * d2x + d1y * d2y     # dot product
            ang = abs(math.atan2(cross, dot))  # signed angle -> take abs
            if ang > CORNER_ANGLE_RAD:         # angle above threshold:
                corner_ahead = True            # corner ahead

        if corner_ahead and left < 0.04:  # corner and less than 4 cm to vertex:
            # Clamp only within the last 4 cm before the vertex. control_loop's
            # in-place turn takes over at 0.25 m with a standard stop-steer-creep;
            # this only prevents last-moment corner cutting.
            return p2                     # aim point = vertex

        if seg_len > 1e-9 and left >= remaining:
            # Case 1: the whole lookahead distance fits in the current segment.
            # Aim point = projection point + remaining x unit direction vector.
            ux = (p2['x'] - p1['x']) / seg_len  # segment unit vector x
            uy = (p2['y'] - p1['y']) / seg_len  # segment unit vector y
            return _pt(seg['px'] + ux * remaining,
                       seg['py'] + uy * remaining)  # advance projection by remaining
        remaining -= left   # current segment contributes all its remaining length, deduct from budget

        # When the current segment cannot hold the full lookahead, continue across
        # subsequent waypoints.
        j = idx + 1                                    # start from the next segment
        while j < len(pts) - 1 and remaining > 1e-9:   # while budget remains and segments exist:
            p1 = pts[j]      # segment start
            p2 = pts[j + 1]  # segment end
            sl = math.hypot(p2['x'] - p1['x'], p2['y'] - p1['y'])  # segment length
            if sl > 1e-9 and sl >= remaining:          # this segment holds the remaining budget:
                ux = (p2['x'] - p1['x']) / sl          # segment unit vector x
                uy = (p2['y'] - p1['y']) / sl          # segment unit vector y
                return _pt(p1['x'] + ux * remaining,
                           p1['y'] + uy * remaining)   # advance from segment start by remaining
            remaining -= sl  # consume the whole segment, keep deducting budget
            j += 1           # next segment
        # Total path length is less than the lookahead distance: aim straight at the end.
        return pts[-1]

    # ##### Pure pursuit geometry 3: steer angle & linear velocity (core output formula) #####

    def pure_pursuit_control(self, target_point, reverse=False):
        """Compute control output via the Pure Pursuit algorithm.

        Important: dzactuator treats angular.z as a servo angle (deg), not an
        angular velocity:
          servo angle = angular.z + 60, clamped to [25, 95]
          -> angular.z = 0 means straight, valid range [-35, +35]
        Consistent with dztrack's sLineControl() (which returns degrees).

        reverse=True 表示这一段要【倒着走】（路线里 'R:B5' 那种）：
          返回的 linear_vel 为负 —— dzactuator 侧 linear.x<0 会置
          SpeedDirection=0x01（倒车方向位），电机反转。
        """
        if target_point is None:     # no aim point:
            return 0.0, 0.0          # straight stop safe output

        # Relative position of the aim point (map frame)
        dx = target_point['x'] - self.current_x  # aim - car x diff
        dy = target_point['y'] - self.current_y  # aim - car y diff

        # Transform to body frame (rel_x=forward, rel_y=left)
        # Rotate the world vector by -yaw: if the aim point is dead ahead,
        # rel_y=0 -> no steering.
        cos_yaw = math.cos(-self.current_yaw)  # cos(-yaw)
        sin_yaw = math.sin(-self.current_yaw)  # sin(-yaw)
        rel_x = dx * cos_yaw - dy * sin_yaw    # body-frame forward component
        rel_y = dx * sin_yaw + dy * cos_yaw    # body-frame left component

        # Distance to target (world-frame magnitude)
        dist = math.sqrt(dx * dx + dy * dy)    # Euclidean distance (lookahead chord)

        if dist < 0.01:              # aim point is right at the car:
            return 0.0, 0.0          # no steering

        # Front wheel steer angle: pure pursuit formula. Wheelbase (0.157 m,
        # encoder-calibrated) and lookahead distance (0.2 m) jointly set the gain.
        # alpha = aim point bearing in body frame.
        #   alpha = 0            -> straight
        #   alpha > 0 (on left)  -> turn left (positive angle)
        #   alpha < 0 (on right) -> turn right (negative angle)
        # Geometry: steer = atan(L * sin(alpha) / d).
        # The max(d, 0.10) floor prevents gain blow-up when the aim point is very close.
        alpha = math.atan2(rel_y, rel_x)          # aim point bearing
        steer_deg = math.degrees(
            math.atan(self.wheelbase * math.sin(alpha) / max(dist, 0.10))
        )   # pure pursuit steer formula -> degrees
        # Hard clamp; dzactuator further clamps to +/-35 deg (servo [25,95]).
        steer_deg = max(-30.0, min(30.0, steer_deg))

        # Linear velocity (slow down in sharp turns)
        # turn_factor 0..1 -> speed scales 1.0..0.5 (0.3 -> 0.15 m/s),
        # 0.1 m/s floor ensures the car does not stop in a turn.
        turn_factor = abs(steer_deg) / 30.0                    # steer ratio 0..1

        # ===== ★倒车段（R: 前缀）=====
        # 舵角公式【与前进完全相同】，只是把速度取负：
        #   自行车模型 θ̇ = (v / L)·tan(δ)
        #   倒车时 v<0 会让 θ̇ 自动反号 —— 也就是说"打同样的舵，车头朝反方向转"，
        #   于是车尾正好朝目标那一侧摆过去，这正是倒车对准想要的。
        #   （用两个例子验算过：目标在正后方 -> δ=0 笔直倒退；
        #     目标在右后方 -> δ<0，车尾确实往右摆。）
        #   这里保留一个方向系数 REVERSE_STEER_SIGN：实车若发现倒车时车尾
        #   偏向反了，把它改成 -1.0 即可（配置中心可改，不用编译）。
        if reverse:
            steer_deg *= REVERSE_STEER_SIGN
            # 倒车要慢：巡航速度 x REVERSE_SPEED_RATIO，转弯再降一半，并保底不熄火
            rev_speed = self.linear_speed * REVERSE_SPEED_RATIO * (1.0 - 0.5 * turn_factor)
            rev_speed = max(0.08, rev_speed)
            return -rev_speed, steer_deg   # ★负线速度 = 倒车

        linear_vel = self.linear_speed * (1.0 - 0.5 * turn_factor)  # slow down in turns
        linear_vel = max(0.1, linear_vel)                      # no lower than 0.1 m/s

        return linear_vel, steer_deg  # return (linear velocity m/s, steer angle deg)

    # ##### ★最佳射击点自主计算（调用 tools/shoot_planner.py）#####
    def _load_shoot_planner(self):
        """懒加载 shoot_planner 模块。

        懒加载的原因：shoot_planner 是独立工具，万一它不在（比如只拷了
        src 没拷 tools），整个循迹节点也绝不能因此起不来 ——
        降级成"靶标格自身当射击点"继续跑，比赛里这比崩掉强得多。
        """
        if getattr(self, '_sp_mod', None) is not None:
            return self._sp_mod
        try:
            import importlib.util
            here = os.path.dirname(os.path.abspath(__file__))
            path = os.path.normpath(os.path.join(here, '..', '..', '..',
                                                 'tools', 'shoot_planner.py'))
            if not os.path.exists(path):
                self.get_logger().warn(f"未找到 {path}，射击点退化为靶标自身")
                self._sp_mod = False
                return False
            spec = importlib.util.spec_from_file_location("shoot_planner", path)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            self._sp_mod = mod
            self.get_logger().info(f"已加载射击点规划器: {path}")
            return mod
        except Exception as exc:
            self.get_logger().warn(f"射击点规划器加载失败({exc})，退化为靶标自身")
            self._sp_mod = False
            return False

    def compute_shoot_chain(self, target_name):
        """为一个靶标算出【候选射击点链】（按评分从优到劣）。

        返回 [格名, ...]。第一个打不中就自动换下一个 ——
        "宁可漏打，不可卡死"，也避免一个点废掉整个靶。

        失败时降级返回 [target_name] 自身（虽然车不会骑到靶上，
        但至少能让路线继续跑完，而不是报错卡住）。
        """
        mod = self._load_shoot_planner()
        tgt = str(target_name).strip().upper()
        if not mod:
            return [tgt]

        try:
            grid = mod.MapGrid(self._obstacle_blocks)
            pose = {'x': self.current_x, 'y': self.current_y,
                    'yaw': self.current_yaw}
            plans = mod.plan_shoot_points(
                [tgt], grid, current_pose=pose,
                dist_target=SHOOT_DISTANCE_TARGET,
                dist_min=SHOOT_DISTANCE_MIN,
                dist_max=SHOOT_DISTANCE_MAX,
                w_gimbal=SHOOT_SCORE_W_GIMBAL,
                w_detour=SHOOT_SCORE_W_DETOUR,
                keep_per_target=max(1, SHOOT_MAX_RETRY),
                verbose=False)
            if plans and plans[0]['candidates']:
                chain = [c['name'] for c in plans[0]['candidates']]
                self.shoot_chain_meta[tgt] = plans[0]['candidates']
                self.get_logger().info(
                    f"靶 {tgt} 候选射击点链（最多试 {SHOOT_MAX_RETRY} 个）: " +
                    " -> ".join("%s(%.2fm)" % (c['name'], c['dist'])
                                for c in plans[0]['candidates']))
                return chain[:max(1, SHOOT_MAX_RETRY)]
        except Exception as exc:
            self.get_logger().warn(f"靶 {tgt} 射击点计算失败({exc})，退化为自身")

        return [tgt]

    # ##### ★是否已越过某航点（"开过头"判据）#####
    def _passed_waypoint(self, wp_idx, wp):
        """车是否已经【越过】该航点。

        判据：车->航点 的向量 与 当前所在航段的前进方向 的点积为负，
              说明航点落在车的后方 —— 已经冲过去了。

        ★为什么重要：云台水平扫描只有 ±90°。车一旦冲过射击点，靶就落到
          扫描范围之外，这一枪必丢。所以要能识别"冲过去了"并及时补救。
        """
        try:
            seg = self.find_nearest_segment()
            if seg is None:
                return False
            a = self.path_points[seg['idx']]
            b = self.path_points[min(seg['idx'] + 1, len(self.path_points) - 1)]
            fx, fy = b['x'] - a['x'], b['y'] - a['y']   # 本段前进方向
            if abs(fx) < 1e-9 and abs(fy) < 1e-9:
                return False
            # 若航点就是本段终点，用"车到段起点的投影"判断更稳
            tx, ty = wp['x'] - self.current_x, wp['y'] - self.current_y
            return (tx * fx + ty * fy) < 0
        except Exception:
            return False

    # ##### 倒车段判定 #####
    def _is_reverse_segment(self, seg):
        """本段（seg['idx'] -> seg['idx']+1）是否为倒车段。

        reverse_flags[i]=True 表示"第 i 个航点要倒着到达"，
        所以段 idx 的倒车属性记在端点 reverse_flags[idx+1] 上。

        防御式写法：数组缺失或不够长都返回 False ——
        这样 path 文件加载（load_path）等老路径不会因为没这个字段而崩。
        """
        if seg is None or not self.reverse_flags:
            return False
        i = seg['idx'] + 1
        return i < len(self.reverse_flags) and bool(self.reverse_flags[i])

    # ##### ★桥洞强制直行（ALIGN_STRAIGHT）#####
    #  为什么需要：桥洞两边是实墙，而 Pure Pursuit 在逼近航点时为了让车压到
    #  路径线上，会做小幅"摆头"修正。平时无所谓，但在洞里宽度余量很小时，
    #  这点摆头就足以蹭墙。所以进入保护段后【直接关掉 Pure Pursuit 转向】，
    #  改用 IMU 航向锁定 + 小舵角微调 + 低速直冲。
    def _rebuild_align_segments(self):
        """预计算"需要强制直行的航段索引集合"。

        ★ 为什么必须在【航段】层面预计算，而不是每帧用几何距离找最近保护点：
          保护点覆盖整列 K1~K11 时，"离最近保护点 < 2 格"这个判据会在
          车开到 K 列附近 ±2 格的【任何位置】都成立 —— 当前航段明明要拐弯，
          也会被强制锁航向 + 限速 0.10 m/s，表现为"循迹僵直、不听路线"。
          改成按航段索引后，只有真正穿过桥洞的那几段才受保护，其余照常循迹。
        """
        self._align_segs = set()
        if not self._bridge_points or len(self.path_points) < 2:
            return
        tol = BLOCK_SIZE * 0.4
        hit = []
        for idx, p in enumerate(self.path_points):
            for (bx, by, _n) in self._bridge_points:
                if abs(p['x'] - bx) < tol and abs(p['y'] - by) < tol:
                    hit.append(idx)
                    break
        approach = max(1, int(round(ALIGN_STRAIGHT_BEFORE)))
        exit_n = max(0, int(round(ALIGN_STRAIGHT_AFTER)))
        last = len(self.path_points) - 2          # 最后一根航段的索引
        for h in hit:
            for k in range(max(0, h - approach), min(last, h + exit_n) + 1):
                self._align_segs.add(k)
        if hit:
            self.get_logger().info(
                f"桥洞直行保护已装载：{len(hit)} 个航点在桥洞上，"
                f"保护航段 {sorted(self._align_segs)}（其余航段正常循迹）")

    def _align_straight_update(self):
        """维护桥洞强制直行状态。返回 (是否激活, 锁定航向rad, 桥洞名)。

        判据：当前所在的【航段索引】是否落在预计算的保护集合里。
        锁定航向：取该航段自身的走向 —— 这正是"直直地穿过去"需要的方向。
        """
        if not self._align_segs or len(self.path_points) < 2:
            return False, 0.0, ''

        seg = self.find_nearest_segment()
        if seg is None:
            return False, 0.0, ''

        idx = seg['idx']
        if idx not in self._align_segs:
            if self.align_active:
                self.get_logger().info(
                    f"★驶离桥洞直行保护（航段 #{idx} 已超出保护范围）")
                self.align_active = False
                self.align_bridge_name = ''
            return False, 0.0, ''

        # 锁定航向 = 本航段的方向（直穿桥洞需要的正是它）
        a = self.path_points[idx]
        b = self.path_points[min(idx + 1, len(self.path_points) - 1)]
        diag = math.hypot(b['x'] - a['x'], b['y'] - a['y'])
        if diag < 1e-6:
            return False, 0.0, ''
        locked = math.atan2(b['y'] - a['y'], b['x'] - a['x'])

        if not self.align_active:
            self.align_active = True
            self.align_bridge_name = f"航段#{idx}"
            self.get_logger().warn(
                f"★进入桥洞强制直行保护 {self.align_bridge_name}"
                f"（锁航向 {math.degrees(locked):.1f}deg，"
                f"限速 {ALIGN_STRAIGHT_SPEED} m/s，关纯追踪转向）")

        return True, locked, self.align_bridge_name


    # ##### Main control loop (50 Hz priority state machine) #####
    # Each tick checks in priority order: shoot wait > no-path keep-alive >
    # nearest segment/aim point > mid-route shoot point > end arrival > sharp
    # corner PIVOT > normal pure pursuit. Each branch ultimately boils down to
    # "publish one Twist": linear.x becomes the motor speed byte, angular.z
    # becomes the servo angle byte, packed by dzactuator into an A5 5A frame
    # and sent to STM32.

    def control_loop(self):
        """Main control loop (50 Hz)."""
        # No localization yet -> do nothing.
        # (Never publish commands early, otherwise steering is impossible and
        # mode switching may be disrupted.)
        if not self.has_odom:   # no localization data:
            return              # skip this tick

        # ★底盘速度估计：每个 tick 都更新一次，保证定点狙击阶段读到的是最新值。
        #   放在最前面是因为射击状态机、以及将来的任何"等车停稳"逻辑都要用它。
        self._update_chassis_speed()

        cmd = Twist()   # reused zero command (linear=0, angular=0)
        # linear.x=0 -> dzactuator TargetSpeed=0 -> STM32 motor PID target 0 (brake)

        # ---- Branch 1: shoot wait: stop and wait for N shots ----------------
        # Priority 1: during the wait, publish zero Twist every tick so dzactuator's
        # Pursuit watchdog keeps seeing us (mode 2 is maintained).
        # ---- Branch 1: 定点狙击状态机 --------------------------------------
        # 战术原则：★宁可漏打，不可卡死★
        #   每一段等不到预期结果都会超时降级放行，绝不把整条路线堵死。
        #   全场只有 10 发弹，所以只在"底盘停住 + 看到靶 + 瞄准稳了"三个条件
        #   同时成立时才真正放行开火 —— 避免无效弹，也避免把有限的弹药
        #   浪费在车还没停稳的那几帧上。
        #
        #   0 SETTLE : 刹车，等底盘速度降到 SHOOT_SETTLE_SPEED 以下
        #   1 ACQUIRE: 请求云台全向扫描，等视觉报告"看到靶"(det_cls != 0)
        #   2 STABLE : 等偏移连续 SHOOT_STABLE_FRAMES 帧落在容差带内
        #   3 FIRE   : 放行开火(stop_signal=1)，等打满 N 发或超时
        if self.shoot_wait:                  # waiting for shooting:
            self.cmd_pub.publish(cmd)  # zero Twist keep-alive (motor target 0 = brake)
            now_t = self.get_clock().now().seconds_nanoseconds()[0]
            self._update_chassis_speed()

            cand_t = now_t - self.shoot_wait_start   # 本候选点已耗时(s)

            # ---- 视觉状态判据 ----
            det_age = (now_t - self.det_t) if self.det_t else 9999.0
            has_target = (self.det_cls != 0) and (det_age < 1.0)
            aimed = (abs(self.det_dx) <= SHOOT_STABLE_OFFSET_PX
                     and abs(self.det_dy) <= SHOOT_STABLE_OFFSET_PX)
            still = (self.chassis_speed < SHOOT_SETTLE_SPEED)

            resume = False        # 本拍是否结束停站
            reason = ''

            # ===== 阶段 0：等底盘停稳 =====
            if self.shoot_phase == 0:
                if not still:
                    self.shoot_settle_since = now_t   # 还在动 -> 持续刷新
                elif now_t - self.shoot_settle_since >= SHOOT_SETTLE_TIME:
                    self.shoot_phase = 1
                    self.shoot_phase_t = now_t
                    # ★向云台请求全向扫描（复用既有 /nav/scan_request 接口）
                    # ★不再发扫描请求！
                    #   云台从任务开始就处于【持续扫描】(data=2)，一直独立地找目标。
                    #   原来这里发 data=1 会把持续扫描【降级成单次】，扫完就停 ——
                    #   这正是"循迹限制了云台找目标"的根源。
                    #   现在循迹只负责停车，找靶完全交给云台自己（真正的双线程）。
                    self.get_logger().info(
                        f"候选点 wp#{self.shoot_wp_idx} 底盘已停稳"
                        f"（v={self.chassis_speed:.3f}m/s）-> 云台持续扫描中，等待锁定")
                elif cand_t > SHOOT_TIMEOUT * 0.4:
                    # 兜底：万一速度估计一直降不到阈值（抖动/坡度），
                    # 不能死等，直接进入找靶阶段
                    self.shoot_phase = 1
                    self.shoot_phase_t = now_t
                    self.get_logger().warn(
                        f"底盘停稳判据未达阈值(v={self.chassis_speed:.3f})，"
                        f"按超时降级进入找靶阶段（云台持续扫描中）")

            # ===== 阶段 1：找靶（请求扫描后等视觉报告）=====
            elif self.shoot_phase == 1:
                if self.aim_status == 1:
                    self.shoot_lock_seen = True
                    # ★置位"一轮正在进行"，才能在后半段识别出"这一轮扫完了"
                    self.scan_round_active = True
                if has_target:
                    self.shoot_phase = 2
                    self.shoot_phase_t = now_t
                    self.stable_count = 0
                    self.get_logger().info(
                        f"已看到靶（cls={self.det_cls} 偏移={self.det_dx:+d},"
                        f"{self.det_dy:+d}px）-> 等待瞄准稳定")
                elif self.scan_round_active and self.aim_status == 0:
                    # ★云台结束了一轮扫描（回到 IDLE）。
                    #   在持续扫描模式下它会自动重新开始，所以这里【不发请求】，
                    #   只记录轮次。等够 SHOOT_SCAN_ROUNDS 轮仍无靶才放弃。
                    self.scan_round_active = False
                    self.scan_rounds_done += 1
                    if self.scan_rounds_done >= SHOOT_SCAN_ROUNDS:
                        resume = True
                        reason = (f'已连续完成 {SHOOT_SCAN_ROUNDS} 轮扫描，视野内无靶'
                                  f'（云台独立扫描，循迹未干预）')
                    else:
                        self.get_logger().info(
                            f"第 {self.scan_rounds_done} 轮扫描结束，"
                            f"云台持续扫描中（第 {self.scan_rounds_done + 1} 轮）")
                elif now_t - self.shoot_phase_t > SHOOT_LOCK_TIMEOUT:
                    resume = True
                    reason = f'找靶超时({SHOOT_LOCK_TIMEOUT:.0f}s)'

            # ===== 阶段 2：等瞄准稳定 =====
            elif self.shoot_phase == 2:
                if has_target and aimed and still:
                    self.stable_count += 1
                elif not has_target:
                    # 靶从视野里消失了 -> 退回找靶
                    self.stable_count = 0
                    if cand_t < SHOOT_TIMEOUT * 0.6:
                        self.shoot_phase = 1
                        self.shoot_phase_t = now_t
                        self.get_logger().warn("目标脱离视野，重新找靶")
                else:
                    self.stable_count = 0     # 瞄偏了 / 车在动，重新计数

                if self.stable_count >= SHOOT_STABLE_FRAMES:
                    self.shoot_phase = 3
                    self.shoot_phase_t = now_t
                    self.shoot_count_at_arrival = self.shoot_count
                    # ★放行开火：底盘停住 + 看到靶 + 连续稳定 N 帧，三者齐备
                    sig = UInt8()
                    sig.data = 1
                    self.stop_signal_pub.publish(sig)
                    self._shoot_allow_sent = True
                    self.get_logger().warn(
                        f"★瞄准稳定 {self.stable_count} 帧"
                        f"（偏移 {self.det_dx:+d},{self.det_dy:+d}px，"
                        f"v={self.chassis_speed:.3f}m/s）-> 放行开火，"
                        f"本候选点目标 {self.SHOOT_COUNT_TARGET} 发")
                elif now_t - self.shoot_phase_t > SHOOT_STABLE_TIMEOUT:
                    # 等不到"完美稳定"就降级放行 —— 宁可打个不太准的，
                    # 也不能因为一直等稳定而把整条路线堵住。
                    self.shoot_phase = 3
                    self.shoot_phase_t = now_t
                    self.shoot_count_at_arrival = self.shoot_count
                    sig = UInt8()
                    sig.data = 1
                    self.stop_signal_pub.publish(sig)
                    self._shoot_allow_sent = True
                    self.get_logger().warn(
                        f"等待稳定超时({SHOOT_STABLE_TIMEOUT:.0f}s，"
                        f"当前稳定计数 {self.stable_count}/{SHOOT_STABLE_FRAMES})"
                        f"-> 降级放行开火")

            # ===== 阶段 3：开火窗口 =====
            # 打满目标发数 = 这个候选点成功
            if self.shoot_phase == 3 and \
                    self.shoot_count - self.shoot_count_at_arrival >= self.SHOOT_COUNT_TARGET:
                self.get_logger().info(
                    f"本候选点已发射 {self.SHOOT_COUNT_TARGET} 发 -> 靶标命中判定完成，"
                    f"跳过同组其余候选点")
                # ★打中就取消同一靶标的其它候选点，不再去那些位置
                gid = self.shoot_resume_gid
                if gid:
                    n_skip = 0
                    for k in range(len(self.shoot_flags)):
                        if (k < len(self.shoot_groups)
                                and self.shoot_groups[k] == gid
                                and self.shoot_flags[k]):
                            self.shoot_flags[k] = False
                            n_skip += 1
                    if n_skip:
                        self.get_logger().info(
                            f"已取消同组 {n_skip} 个剩余候选射击点")
                resume = True
                reason = 'success'

            # ===== 硬超时：本候选点用掉 SHOOT_TIMEOUT 还没结果就换点 =====
            if not resume and cand_t > SHOOT_TIMEOUT:
                fired = self.shoot_count - self.shoot_count_at_arrival
                resume = True
                reason = f'本候选点硬超时({SHOOT_TIMEOUT:.0f}s，已打 {fired} 发)'

            if resume:
                # 收回开火许可（云台停止射击，路线继续）
                sig = UInt8()
                sig.data = 0
                self.stop_signal_pub.publish(sig)
                self.shoot_wait = False
                self.shoot_phase = 0
                if reason == 'success':
                    self.get_logger().info("射击完成，恢复循迹")
                else:
                    # 失败也要坚决往下走：路线里下一个候选点会自动再试，
                    # 全部候选都用完还没打中就直接继续后续航点 —— 绝不卡死。
                    self.get_logger().warn(
                        f"本候选点未完成（{reason}）-> 转移至下一候选点/继续循迹")
            return               # during wait, this tick ends here

        # ---- 点位扫描停站等待 -------------------------------------------
        # 停车等云台扫完两趟。三种放行条件，任一满足就走：
        #   ① /aim/status == 2  云台已锁定目标 -> 立刻进入"边走边跟随"
        #   ② 看到过 SCAN(1) 之后回到 IDLE(0)  说明两趟扫完没找到 -> 去下一个点位
        #   ③ 硬超时            兜底，绝不卡住循迹
        if self.scan_wait:
            self.cmd_pub.publish(cmd)   # 停车保活（零 Twist = 刹车）
            now_t = self.get_clock().now().seconds_nanoseconds()[0]
            elapsed = now_t - self.scan_wait_start
            if self.aim_status == 1:
                self.scan_seen_scanning = True

            if self.aim_status == 2:
                self.scan_wait = False
                self.get_logger().info("云台已锁定目标，恢复循迹（边走边跟随）")
            elif self.scan_seen_scanning and self.aim_status == 0:
                self.scan_wait = False
                self.get_logger().info("云台扫描完成、未发现目标，继续前往下一个点位")
            elif elapsed > self.SCAN_WAIT_TIMEOUT:
                self.scan_wait = False
                self.get_logger().warn(
                    f"扫描等待超时({self.SCAN_WAIT_TIMEOUT:.0f}s)，强制继续循迹")
            return

        # Priority 2: no route -> publish an empty zero Twist. This is what keeps
        # dzactuator's modeTimerCallback in Pursuit mode (mode 2); if we stay
        # silent for more than 2 s, another mode may seize control.
        if not self.path_loaded or not self.path_points:  # no path:
            # Even without a path, publish an idle command to keep pursuit_active
            # This keeps dzactuator in Pursuit mode
            self.cmd_pub.publish(cmd)   # zero Twist = stationary but keep-alive
            return                      # tick ends

        # Priority 3: nearest segment + aim point on the path exactly lookahead_dist ahead
        # (steering gain is constant regardless of segment length/direction)
        seg = self.find_nearest_segment()  # nearest segment + projection
        if seg is None:                    # degenerate path (single point):
            target_point = self.path_points[-1] if self.path_points else None  # aim straight at last point
        else:                              # normal case:
            self.current_target_idx = seg['idx']       # advance target segment index
            target_point = self.find_lookahead_point(seg)  # get lookahead aim point

        if target_point is None:           # not even an aim point:
            self.cmd_pub.publish(cmd)      # publish zero Twist keep-alive
            return                         # tick ends

        # === 点位全向扫描停站检测 =========================================
        # 到达"待扫描"点位时：
        #   云台已锁定(status==2) -> 不停车，直接标记跳过（边走边跟随）
        #   未锁定                -> 停车并发扫描请求
        for wp_idx in range(len(self.path_points)):
            if (wp_idx >= len(self.scan_flags)
                    or not self.scan_flags[wp_idx]):     # 不是待扫描点：
                continue                                 # 跳过
            wp = self.path_points[wp_idx]                # 该点位
            d = math.hypot(wp['x'] - self.current_x,
                           wp['y'] - self.current_y)     # 车到点位距离
            if d <= self.SCAN_STOP_RADIUS:               # 进入停站半径：
                if self.aim_status == 2:
                    # 已锁定目标，不必停车，标记跳过即可
                    self.scan_flags[wp_idx] = False
                    self.get_logger().info(
                        f"点位 #{wp_idx} 云台已锁定目标，不停车继续")
                else:
                    self.enter_scan_stop(wp_idx)         # 停车 + 请求扫描
                    return                               # 本 tick 结束

        # === Mid-route shoot stop detection ===============================
        # Scan all waypoints whose shoot_flags are still True. Independent of
        # segment index, so shoot points at the start/end/middle can all trigger.
        for wp_idx in range(len(self.path_points)):      # iterate all waypoints
            if (wp_idx >= len(self.shoot_flags)
                    or not self.shoot_flags[wp_idx]):    # not a pending shoot point:
                continue                                 # skip
            wp = self.path_points[wp_idx]                # this shoot waypoint
            d = math.hypot(wp['x'] - self.current_x,
                           wp['y'] - self.current_y)     # car-to-point distance
            # ★"开过头"补触发（云台只有 ±90°，冲过就再也扫不到靶）
            #   原来只认"半径 0.17m 以内"，一旦冲过去这个点就永远不触发、
            #   整条路线会静默跳过这次射击。现在补一条"已越过但还没跑远"的判据。
            passed = self._passed_waypoint(wp_idx, wp)
            if d <= self.SHOOT_STOP_RADIUS or (passed and d <= SHOOT_OVERSHOOT_TOL):
                if passed and d > self.SHOOT_STOP_RADIUS:
                    self.get_logger().warn(
                        f"⚠️ 射击点 #{wp_idx} 被冲过（还差 {d:.2f}m），"
                        f"已在 {SHOOT_OVERSHOOT_TOL:.2f}m 容忍范围内补触发停站"
                        f" —— 云台扫描仅 ±90°，再往前就永远扫不到靶了")
                self.enter_shoot_stop(wp_idx) # enter shoot stop
                return                        # tick ends

        # Priority 4: check end arrival
        final_point = self.path_points[-1]                # end
        dx = final_point['x'] - self.current_x            # end - car x diff
        dy = final_point['y'] - self.current_y            # end - car y diff
        dist_to_final = math.sqrt(dx * dx + dy * dy)      # distance to end

        # Once arrived, "arrived" is latched forever. A car that overshoots and
        # re-navigates would oscillate back and forth. To move again, a new
        # waypoint command must be issued.
        if dist_to_final < self.arrival_threshold or self.arrived:  # within threshold or latched:
            self.arrived = True   # latch

            # Stop right at the block center.
            # No forced heading change: the straight approach segment already
            # aligned the car to the specified heading.
            self.cmd_pub.publish(cmd)   # zero Twist = brake (speed byte 0, motor closed-loop brake)

            if not self.arrive_logged:  # one-shot block, runs once per arrival:
                # Log + snap + allow shooting, runs only once per arrival.
                self.arrive_logged = True
                target_yaw = euler_from_quaternion(          # end expected heading
                    final_point['qx'], final_point['qy'],
                    final_point['qz'], final_point['qw']
                )
                drift_x = self.current_x - final_point['x']  # belief position error x
                drift_y = self.current_y - final_point['y']  # belief position error y
                yaw_drift_deg = math.degrees(
                    normalize_angle(self.current_yaw - target_yaw)  # heading drift (deg)
                )

                # Snap odom to block center + commanded heading.
                # This limits DR drift to within a single segment: the next
                # segment starts from a "perfect pose" (in belief; the small
                # residual on the real car is absorbed by the 0.157 m arrival threshold).
                self.snap_x_off -= drift_x   # fold x drift into snap offset
                self.snap_y_off -= drift_y   # fold y drift into snap offset
                self.snap_yaw_off = normalize_angle(
                    self.snap_yaw_off - math.radians(yaw_drift_deg)  # fold heading drift
                )
                self.current_x = final_point['x']   # belief position = end center
                self.current_y = final_point['y']
                self.current_yaw = target_yaw       # belief heading = commanded heading

                self.get_logger().info(
                    f"ARRIVED! dist={dist_to_final:.3f}m, "
                    f"drift=({drift_x:+.2f}m, {drift_y:+.2f}m), "
                    f"yaw_drift={yaw_drift_deg:+.1f}deg, stopped"
                )
                self.get_logger().info(
                    "Odometry re-anchored to block center + commanded heading"
                )
                # ★P1-1 漂移预算监控：把本段漂移计入统计，超阈值立即告警。
                #   这里衡量的是"上一段航程里死推算累积了多少误差"——
                #   因为每段结束都会把信念位姿锚回点位中心，段内漂移不会累积到下一段，
                #   所以这个值就是单段误差的直接读数，是定位可信度的最佳指标。
                leg_drift = math.hypot(drift_x, drift_y)
                self.drift_legs += 1
                self.drift_max = max(self.drift_max, leg_drift)
                self.drift_yaw_max = max(self.drift_yaw_max, abs(yaw_drift_deg))
                if leg_drift > DRIFT_WARN_M:
                    self.drift_warn_count += 1
                    self.get_logger().warn(
                        f"⚠️ 定位漂移偏大：本段 {leg_drift:.3f}m（阈值 {DRIFT_WARN_M:.2f}m），"
                        f"yaw {yaw_drift_deg:+.1f}deg，第 {self.drift_warn_count} 次。"
                        f"到达判定阈值仅 {self.arrival_threshold:.3f}m，"
                        f"漂移逼近它就会'越锚越偏'，建议检查轮径/地面打滑/IMU。"
                    )
                # Notify dzrknn_dataprocess to open gimbal scan + shooting at the end stop.
                sig = UInt8()                  # build signal
                sig.data = 1                   # 1 = allow shooting
                self.stop_signal_pub.publish(sig)  # publish stop signal
                self.get_logger().info(
                    "Published stop_signal=1 (enable gimbal+shooting)")
                # 终点也请求一次全向扫描：云台扫完两趟后按开火判据自主射击
                self.scan_req_pub.publish(UInt8(data=1))
                self.get_logger().info("已请求云台在终点执行全向扫描")
                # ★★★ 运行完成提示符 ★★★
                self._print_finish_banner()
            return      # arrival branch ends here

        # === Sharp corner in-place turn (PIVOT) ==========================
        # When the next vertex has a sharp corner (>45 deg) and the car enters
        # the trigger distance, switch to three-phase turning:
        #   0: stop, steer servo to max in the turn direction
        #   1: hold steering for 0.5 s, wait for the servo to physically reach
        #   2: creep at 0.08 m/s until heading aligns with the next segment (within 15 deg)
        # Then resume pure pursuit on the next segment.
        SHARP_CORNER_RAD = math.radians(45.0)  # min turn angle to trigger in-place turn
        CORNER_TRIGGER_M = 0.25    # how close to vertex to start turning
        CORNER_ALIGN_RAD = math.radians(15.0)  # heading alignment tolerance
        CORNER_SLOW_M = 0.45       # range before the trigger point to start slowing

        # ---- PIVOT entry check (only when not already turning) -----------
        # ★align_active 时禁止进入 PIVOT：桥洞里绝不做原地转向。
        #   先算一次桥洞保护状态，供 PIVOT 入口判断与后面的直行接管共用。
        seg_rev = self._is_reverse_segment(seg)
        self._publish_motion(seg_rev)   # ★上报前进/倒车，供浏览器轨迹分色
        align_on, align_yaw, align_name = self._align_straight_update()

        if seg is not None and not self.corner_mode and not align_on \
                and seg['idx'] + 2 < len(self.path_points):  # a next-next point must exist to form a corner:
            vertex = self.path_points[seg['idx'] + 1]   # corner vertex
            dvx = vertex['x'] - self.current_x          # car->vertex x
            dvy = vertex['y'] - self.current_y          # car->vertex y
            dist_to_vertex = math.hypot(dvx, dvy)       # distance to vertex
            p1 = self.path_points[seg['idx']]       # current segment start
            p2 = self.path_points[seg['idx'] + 1]   # vertex
            p3 = self.path_points[seg['idx'] + 2]   # next segment end
            d1x = p2['x'] - p1['x']   # entering segment direction x
            d1y = p2['y'] - p1['y']
            d2x = p3['x'] - p2['x']   # exiting segment direction x
            d2y = p3['y'] - p2['y']
            cross = d1x * d2y - d1y * d2x   # cross: >0 left, <0 right
            dot = d1x * d2x + d1y * d2y     # dot
            corner_ang = abs(math.atan2(cross, dot))  # abs angle between segments
            if corner_ang > SHARP_CORNER_RAD and dist_to_vertex < CORNER_TRIGGER_M:
                # Enter in-place turn mode
                self.corner_mode = True          # set turn flag
                self.corner_phase = 0            # start from phase 0
                self.corner_phase_t = self.get_clock().now().seconds_nanoseconds()[0]  # record time
                self.corner_seg_idx = seg['idx'] + 1  # resume from this segment when done
                self.corner_target_yaw = math.atan2(d2y, d2x)  # exit segment heading
                self.corner_turn_dir = 1 if cross > 0 else -1  # left +1 / right -1
                self.get_logger().info(
                    f"[PIVOT] sharp corner at vertex #{seg['idx']+1}, "
                    f"target_yaw={math.degrees(self.corner_target_yaw):.0f}deg "
                    f"dir={'L' if self.corner_turn_dir>0 else 'R'}")

        # ---- PIVOT state machine (runs every tick while corner_mode is on) ----
        if self.corner_mode:   # in-place turn in progress:
            now_t = self.get_clock().now().seconds_nanoseconds()[0]  # current time
            # Error relative to exit heading (short-arc direction)
            yaw_err = normalize_angle(self.corner_target_yaw - self.current_yaw)

            if self.corner_phase == 0:   # phase 0:
                # Stop + steer to max
                # 30 deg x direction = servo full deflection (servo angle 60 +/-30).
                cmd.linear.x = 0.0                        # speed 0 = motor brake
                cmd.angular.z = 30.0 * self.corner_turn_dir  # servo full deflection (left +, right -)
                self.cmd_pub.publish(cmd)                 # publish stop + steer command
                self.corner_phase = 1          # advance to phase 1 immediately
                self.corner_phase_t = now_t    # record phase time
                self.get_logger().info("[PIVOT] phase 1: stop + max steer")
                return                         # tick ends

            elif self.corner_phase == 1:   # phase 1:
                # Hold steering for 0.5 s, wait for servo to reach position
                if now_t - self.corner_phase_t < 0.5:     # not yet 0.5 s:
                    cmd.linear.x = 0.0         # car stationary, wheels held at full steer
                    cmd.angular.z = 30.0 * self.corner_turn_dir
                    self.cmd_pub.publish(cmd)  # keep publishing stop + full steer
                    return                     # tick ends
                self.corner_phase = 2          # 0.5 s elapsed -> enter phase 2
                self.corner_phase_t = now_t    # record time
                self.get_logger().info("[PIVOT] phase 2: creep forward")
                # fall through to phase 2 logic

            # Phase 2: hold full steer and creep until heading aligns
            if abs(yaw_err) < CORNER_ALIGN_RAD:   # heading error < 15 deg:
                # Aligned: resume pure pursuit on the next segment
                self.corner_mode = False                 # exit turn mode
                self.current_target_idx = self.corner_seg_idx  # switch to resume segment
                self.get_logger().info(
                    f"[PIVOT] aligned (yaw_err={math.degrees(yaw_err):.1f}deg), "
                    f"resume pure pursuit on seg {self.corner_seg_idx}")
                # fall through to normal pure pursuit below

            else:   # not aligned:
                # Hold full steer, creep at 0.08 m/s. Ackermann geometry turns
                # the car gradually toward corner_target_yaw.
                cmd.linear.x = 0.08                         # creep speed 0.08 m/s
                cmd.angular.z = 30.0 * self.corner_turn_dir  # servo held at full deflection
                self.cmd_pub.publish(cmd)                    # publish creep command
                self.arrive_logged = False   # re-arm "arrival log once" guard
                # Log once every 20 ticks (~1 s) to avoid spam.
                if not hasattr(self, '_log_count'):   # first use:
                    self._log_count = 0               # init counter
                self._log_count += 1                  # count +1
                if self._log_count % 20 == 0:         # every 20 ticks:
                    self.get_logger().info(
                        f"[PIVOT] creeping yaw_err={math.degrees(yaw_err):.1f}deg "
                        f"yaw={math.degrees(self.current_yaw):.1f}deg "
                        f"target={math.degrees(self.corner_target_yaw):.1f}deg")
                return   # while creeping, tick ends here

        # === ★桥洞强制直行接管（ALIGN_STRAIGHT）=========================
        # 放在这里的原因：
        #   · 在所有"停车类"判定（射击停站/扫描停站）之后 —— 那些优先级更高，
        #     桥洞保护只接管【转向】，不该阻止该停的站。
        #   · 在 PIVOT 之后 —— 进洞前的最后摆正仍交给 PIVOT，那之后才直冲。
        if align_on and not self.corner_mode:
            # IMU 航向锁定：用当前航向与锁定航向的误差做小舵角微调。
            # 舵角限幅 ±ALIGN_STEER_LIMIT 度，避免为了修几度的偏差打出大舵角，
            # 反而在洞里摆动。
            yaw_err = math.degrees(normalize_angle(align_yaw - self.current_yaw))
            steer_deg = max(-ALIGN_STEER_LIMIT,
                            min(ALIGN_STEER_LIMIT, yaw_err * ALIGN_KP / 10.0))

            # 速度：低速直冲；倒车段则反向
            spd = ALIGN_STRAIGHT_SPEED
            if seg_rev:
                spd = -max(0.08, spd * REVERSE_SPEED_RATIO)

            # 越界急停依然最高优先（复用同一套判据，不能因为直行保护而绕过）
            if not (MAP_X_MIN - MAP_SAFETY_MARGIN <= self.current_x <= MAP_X_MAX + MAP_SAFETY_MARGIN
                    and MAP_Y_MIN - MAP_SAFETY_MARGIN <= self.current_y <= MAP_Y_MAX + MAP_SAFETY_MARGIN):
                self.cmd_pub.publish(Twist())
                return

            cmd.linear.x = spd
            cmd.angular.z = steer_deg
            self.cmd_pub.publish(cmd)

            self._align_log = getattr(self, '_align_log', 0) + 1
            if self._align_log % 10 == 1:
                self.get_logger().info(
                    f"[ALIGN] {align_name} 洞内直行: yaw_err={yaw_err:+.1f}deg "
                    f"steer={steer_deg:+.1f}deg v={spd:+.2f}m/s "
                    f"pos=({self.current_x:.2f},{self.current_y:.2f})")
            return   # 本 tick 结束，不走纯追踪

        # === Normal pure pursuit =========================================
        # Compute control output
        # seg_rev：本段是否倒车（路线里的 'R:' 前缀），已在上面算好
        linear_vel, steer_deg = self.pure_pursuit_control(target_point, reverse=seg_rev)

        # ---- ★P0-3 越界急停：信念位姿出地图即刹车，绝不继续循迹 ----
        if not (MAP_X_MIN - MAP_SAFETY_MARGIN <= self.current_x <= MAP_X_MAX + MAP_SAFETY_MARGIN
                and MAP_Y_MIN - MAP_SAFETY_MARGIN <= self.current_y <= MAP_Y_MAX + MAP_SAFETY_MARGIN):
            self.cmd_pub.publish(Twist())        # 零 Twist = 立即刹车
            if not getattr(self, '_oof_warned', False):
                self._oof_warned = True
                self.get_logger().error(
                    f"★★★ 越界急停！信念位姿 ({self.current_x:.2f}, {self.current_y:.2f}) "
                    f"超出地图 [{MAP_X_MIN:.2f}~{MAP_X_MAX:.2f}] x "
                    f"[{MAP_Y_MIN:.2f}~{MAP_Y_MAX:.2f}]，已刹车。"
                    f"里程计可能已漂移，请检查后重跑。")
                self._banner([
                    "  ★★★  越 界 急 停  ★★★",
                    f"  信念位姿 : ({self.current_x:.2f}, {self.current_y:.2f})",
                    f"  地图范围 : x {MAP_X_MIN:.2f}~{MAP_X_MAX:.2f}  y {MAP_Y_MIN:.2f}~{MAP_Y_MAX:.2f}",
                    "  原因     : 里程计漂移导致位姿跑出地图，已强制停车",
                ])
            return
        # Slow down near the end to avoid overshoot: within 0.5 m limit speed to
        # 0.35 x distance (0.5 m -> 0.175 m/s, hits the 0.1 m/s floor at 0.29 m),
        # forming a smooth deceleration ramp.
        if dist_to_final < 0.5:  # less than 0.5 m to end:
            linear_vel = min(linear_vel, max(0.1, 0.35 * dist_to_final))  # limit by distance ratio

        # ★射击点提前减速（云台±90°，冲过头就再也扫不到靶）
        for _i, _f in enumerate(self.shoot_flags):
            if not _f:
                continue
            _wp = self.path_points[_i]
            _d = math.hypot(_wp['x'] - self.current_x, _wp['y'] - self.current_y)
            if _d < SHOOT_SLOWDOWN_M:
                # 0.60m 处开始线性减速到 0.08 m/s，保证能在 0.17m 内停住
                _lim = max(0.08, 0.30 * _d)
                linear_vel = min(linear_vel, _lim)

        # Slow down before a corner: when a sharp corner enters the CORNER_SLOW_M
        # range, limit speed so the car can stop at the trigger point instead of
        # blowing past the vertex.
        # (Same geometry as the entry check above, just a larger radius and only limits speed.)
        if seg is not None and not self.corner_mode \
                and seg['idx'] + 2 < len(self.path_points):   # a corner may be ahead:
            vertex = self.path_points[seg['idx'] + 1]   # vertex
            dvx = vertex['x'] - self.current_x          # car->vertex x
            dvy = vertex['y'] - self.current_y          # car->vertex y
            dist_to_vertex = math.hypot(dvx, dvy)       # distance to vertex
            p1 = self.path_points[seg['idx']]           # current segment start
            p2 = self.path_points[seg['idx'] + 1]       # vertex
            p3 = self.path_points[seg['idx'] + 2]       # next segment end
            d1x = p2['x'] - p1['x']                     # entering segment direction
            d1y = p2['y'] - p1['y']
            d2x = p3['x'] - p2['x']                     # exiting segment direction
            d2y = p3['y'] - p2['y']
            cross = d1x * d2y - d1y * d2x               # cross
            dot = d1x * d2x + d1y * d2y                 # dot
            corner_ang = abs(math.atan2(cross, dot))    # angle between segments
            if corner_ang > SHARP_CORNER_RAD \
                    and dist_to_vertex < CORNER_SLOW_M:  # sharp corner and within slow zone:
                linear_vel = min(linear_vel, 0.12)   # limit to 0.12 m/s

        # Final output. Note the unit convention with dzactuator:
        #   linear.x  = m/s        (converted via 23/0.33 to a 0..255 speed byte)
        #   angular.z = servo angle (deg) (servo angle = value + 60)
        cmd.linear.x = linear_vel    # target linear velocity (m/s) -> dzactuator converts to speed byte buf[6]
        cmd.angular.z = steer_deg    # steer angle (deg) -> dzactuator +60 -> servo byte buf[3..4]

        self.cmd_pub.publish(cmd)    # publish: dzactuator packs an A5 5A frame and sends it over serial to STM32
        self.arrive_logged = False   # re-arm "arrival log once" guard

        # Periodic status log
        if not hasattr(self, '_log_count'):   # first use:
            self._log_count = 0               # init counter
        self._log_count += 1                  # count +1

        if self._log_count % 40 == 0:  # every 2 s (40 x 50 ms)
            seg_disp = str(seg['idx']) if seg else '-'  # segment index display ('-' if no segment)
            self.get_logger().info(
                f"pos=({self.current_x:.2f}m, {self.current_y:.2f}m) "
                f"seg={seg_disp}/{len(self.path_points)} "
                f"dist={dist_to_final:.2f}m "
                f"v={linear_vel:.2f} steer={steer_deg:.1f}deg "
                f"yaw={self.current_yaw:.2f}rad"
            )


# ##### Program entry #####
# Parse the optional path file arg -> init ROS2 -> create node and spin;
# on Ctrl-C exit, publish a zero Twist (brake + center servo) before destroying the node.

def main(args=None):
    path_file = None  # optional path file

    # Optional command-line arg: preload a path file at startup
    if len(sys.argv) > 1:        # extra arg present:
        path_file = sys.argv[1]  # first arg is the path file

    rclpy.init(args=args)          # init ROS2
    node = PathExecutorNode(path_file)  # create executor node (with preloaded path)

    try:
        rclpy.spin(node)  # enter event loop, process callbacks and timers
    except KeyboardInterrupt:  # user pressed Ctrl-C:
        node.get_logger().info("Node stopped by user")
        # Publish stop command (zero Twist -> brake, servo centered)
        stop_msg = Twist()              # all-zero command
        node.cmd_pub.publish(stop_msg)  # speed byte 0 = brake, servo angle 60 = centered
    finally:
        node.destroy_node()  # destroy node
        rclpy.shutdown()     # shutdown ROS2


if __name__ == "__main__":  # when this file is run directly:
    main()                  # call entry
