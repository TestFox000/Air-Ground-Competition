---
来源: 本次代码级技术审计（2026-10-23，子智能体）
取证: 实
更新: 2026-10-23
status: 现行
---

# 无人机（dzsmartscout）技术全景文档

> **用途**：给没有参与过前序工作的人（人或 agent）在 10~30 分钟内搞清无人机侧现在
> 是什么、代码怎么跑、坏在哪里、下一步做什么。本文是**代码级审计**，所有结论均带
> `文件:行号` 或测试名，不确定一律写「未核实」。
>
> **审计范围**：`dzsmartscout` 的**自研部分**。第三方包（`livox_ros_driver2`、
> `FAST_LIO_LOCALIZATION_ROS2`、`px4_msgs`、`px4_ros_com`、`mavros`、`ego-planner`、
> `hba`、`FASTLIO2_ROS2`、`gst_camera`）只在其**边界与生效配置**上说明，不逐行审计。
>
> **审计方式**：纯静态阅读（未 SSH、未执行任何机上命令、未改动任何源码或配置）。
> 唯一写入目标是本文档。
>
> **关键事实（先记住）**：这套无人机**从来不是"自主飞行"系统**。它是
> **「人先遥操飞一遍 → FAST-LIO 记录轨迹 → 存成 txt → 之后飞机沿该轨迹回放」**的
> 回放式路径跟随系统，外加一条**向小车发航点/情报**的链路。
> 截至本轮，**飞控板从未连接**（无 `/dev/ttyACM*`），因此**从未自主飞行过**；
> 唯一在真机闭环过的是**无人机 → 小车**这条链路。

---

## §0 导读（只看这一节也能干活）

### 0.1 一页速览

| 面 | 状态 | 一句话证据 |
|---|---|---|
| 感知+定位 | ✅ 通到 `/fmu/in/vehicle_visual_odometry` | 2026-10-06 实测 42.4 Hz（历史记录） |
| 飞控链路 | ❌ 飞控板从未接 | 无 `/dev/ttyACM*`、无 `/dev/ttyUSB*`、`lsusb` 无 FMU |
| 路径跟随 | ⚠️ 代码完整、从未真机飞 | 依赖飞控板，本轮无法验证 |
| 无人机→小车 | ✅ 已端到端闭环 | 2026-10-07 车侧订阅数=2（硬证据） |
| 相机 | ✅ 已接且通（`/image_raw` 29.2 Hz），❌ 画面糊 | 拉普拉斯方差 46.2（糊区间） |
| 桥顶箭头识别 | ❌ 无法满足赛规 | 相机糊 + 无机载识别算法 |
| 比赛合规 | ❌ p8 强制项不满足 | 必须无人机高空侦察桥顶标识 |

### 0.2 系统是什么

```text
人遥操飞 → FAST-LIO 记录轨迹 → 存 TXT → 之后自动沿该轨迹回放
        └──────────────────────┘
外加：无人机 → /nav/waypoint 或 /dzlink/drone_to_car → 小车
```

- **没有**自主避障、**没有**自主建图决策、**没有**视觉识别（机载 `dzuav` 有检测，但与飞控无关、从未在真机跑过）。
- **有**回放式路径跟随（`px4_path`）、Offboard 控制（`px4_control`）、
  定位进 PX4（`px4_odom_convert`）、跨机情报（`dzlink_supervisor` / `dz_send_waypoint`）。

### 0.3 三条独立链路（判断"我卡在哪一步"）

| # | 链路 | 起点→终点 | 状态 | 是否需飞控板 |
|---|---|---|---|---|
| A | 感知→PX4 | 雷达 → FAST-LIO → `/fmu/in/vehicle_visual_odometry` | ✅ 42.4 Hz 实测 | 发出侧不需要；PX4 收需要 |
| B | 路径录制/跟随 | `/localization` ↔ `px4_path` ↔ `/px4_control/goto` | ⚠️ 代码就绪，依赖 A+飞控 | **是** |
| C | 无人机→小车 | `dz_send_waypoint` / `dzlink_supervisor` → `/nav/waypoint`、`/dzlink/drone_to_car` | ✅ 端到端实测通过 | **否** |

> **C 是当前唯一"不依赖飞控板就能完整验证"的链路**，也是 2026-10-07 唯一取得硬证据的闭环。

### 0.4 当前最重要的四个坑（先记住，别再踩）

1. **`ros2 launch px4_odom_convert px4_odom_convert.launch.py` 会静默不发数据**。
   默认 `mode=localization` 订阅一个**从不存在**的话题。**必须加 `mode:=odom`**
   （补丁 01 已产出，未应用；`fc-audit/fixes/01-odom-source/APPLY.md`）。
2. **`px4_path` 有"录了新路径却飞旧路径"的契约断裂**。
   跟随读 `recorded_path_2.txt`、录制写 `recorded_path.txt`（补丁 02 已修，未应用）。
3. **跨机 ROS 靠 `ROS_DOMAIN_ID=118` 对齐**。
   唯一真相源是 `dzlink.env` / `dzros.env`。域号不一致时**ping 通、SSH 通、ROS 互相看不见且零报错**。
4. **跟随器是"锁存许可 + 无上限接驳段"，起节点即可能自动开飞**。
   补丁 03 已加"显式 OFF→ON 边沿"闸门，**未应用**。

### 0.5 导航（按任务类型查）

| 我想…… | 去看 |
|---|---|
| 起整套系统 / 启动顺序 | §2.9 + §4 |
| 找某个参数叫什么、现在是多少 | §5（配置键汇总） |
| 搞清坐标为什么这样转 | §2.2.5、§2.3.3 |
| 知道"现在到底能不能飞" | §0.1 + §7（未核实清单） |
| 找某个函数/类在哪个文件、什么签名 | §3（函数级索引大表） |
| 找 bug / 坑 | §2.10（缺陷清单） |
| 改代码 / 出补丁 | §2.8（fc-audit 补丁区） |
| 给小车发航点 | §2.7 |
| 搞清楚以前踩过什么坑 | §2.8 + §6 |

---

## §1 模块地图

### 1.1 目录树（自研部分，含行数）

```text
dzsmartscout/
├── WORKFLOW.md                      (57)   工作约定：本地改→用户手动替换+编译；SSH 只读
├── fc-audit/
│   ├── 启动手册.md                   (240) 实测版启动手册（含三处差异）
│   ├── 无人机技术文档.md             (954) 无人机全景文档（历史，本文档取代其"全景"角色）
│   ├── 飞控代码整理.md               (561) 飞控代码审计整理
│   ├── 场地规则解读与雷达应用.md     (139) 规则与雷达可见性矩阵
│   ├── notes/{camera,fastlio,px4_path}.md  (108/464/602) 专题审计笔记
│   ├── fixes/01~06/*                六个补丁（files/ + APPLY.md）
│   ├── verify_frames.py             (210) 坐标链路数值验证脚本
│   └── src-mirror/                  与机上 ~/dzpx4/src 逐文件 SHA256 一致的镜像
├── src/
│   ├── px4_control/                 自研：Offboard 控制 + 键盘遥操（662+177 行 C++）
│   ├── px4_path/                    自研：路径录制 + 双模式跟随 + 轨迹生成（约 2100 行）
│   ├── px4_odom_convert/            自研：定位 → PX4 外部视觉里程计（978 行）
│   ├── px4_imu_bridge/              自研：PX4 SensorCombined → ROS Imu（148 行）
│   ├── uav_camera_code/onboard/dzuav/  自研：机载视觉侦察栈（C++，约 800 行）
│   │   └── tools/                   标定/数据集/预检/远端运行/仿真匹配工具
│   ├── uav_camera_code/docs/        接手说明.md、无人机工作流程.md
│   └── [第三方] livox_ros_driver2 / FAST_LIO_LOCALIZATION_ROS2 / px4_msgs / px4_ros_com /
│                                        mavros / ego-planner / hba / FASTLIO2_ROS2 / gst_camera
├── tools/
│   ├── perception/                  自研：俯瞰场地感知管线（纯 Python，numpy+Pillow）
│   ├── link/                        自研：dzlink_supervisor + dz_send_waypoint + 部署/探针
│   ├── drone/                       无人机诊断/桥接/相机/雷达脚本
│   └── [planner/sim_airground/reports] 规划/仿真/证据（车端为主，本文档只交叉引用）
├── drone-scripts/                   start-lidar.sh、start-lidar-on-link.sh
└── [根级] find_drone.py / setup_drone_access.py / 各类 *.py、*.mjs、*.ps1（网络接入阶段）
```

### 1.2 五个自研飞控核心包（依赖方向单一）

```text
px4_path → px4_control → PX4（/fmu/in/*）
px4_odom_convert → PX4（/fmu/in/vehicle_visual_odometry）
px4_imu_bridge → ROS（/fmu/out/imu_data）
dzuav → UDP :50000 → 车端（不依赖飞控）
dzlink_supervisor / dz_send_waypoint → 小车（不依赖飞控）
```

| 包 | 版本 | 代码量 | 一句话职责 |
|---|---|---|---|
| `px4_control` | 0.1.0 | 662+177 行 C++ | Offboard 心跳/解锁/起飞/悬停/降落 + Goto 门控 + 键盘遥操 |
| `px4_path` | 0.1.0 | 约 2100 行 | 路径录制、加载、坐标转换、平滑、waypoint/trajectory 双跟随 |
| `px4_odom_convert` | — | 978 行 | 定位(ENU/FLU) → PX4 外部视觉里程计(NED/FRD) |
| `px4_imu_bridge` | — | 148 行 | PX4 `SensorCombined` → ROS `sensor_msgs/Imu` |
| `dzuav` | — | 约 800 行 | 相机→去畸变→HBB 检测→格号分类→PnP 定位→UDP 广播 |

> `px4_control` **不认识**路径文件、不做规划；`px4_path` **不碰** Offboard/解锁/起降。

### 1.3 第三方包边界（说明其存在与生效配置，不逐行审计）

| 包 | 体积 | 状态 | 生效配置 / 边界 |
|---|---|---|---|
| `livox_ros_driver2` | — | **在用** | `msg_MID360_launch.py`：`xfer_format=1`（CustomMsg，不可改）、`publish_freq=10.0`、`frame_id=livox_frame`、`cmdline_bd_code=47MCN850037605` |
| `FAST_LIO_LOCALIZATION_ROS2` | 348 MB（doc 占 128 MB） | **在用但 4 节点 3 死** | launch 默认 `config_file=mid360_test.yaml`；唯一活节点 `fastlio_mapping` 发 `/Odometry` ~44 Hz |
| `px4_msgs` | 1.9 MB | **version 2.0.1** | ≈PX4 v1.15+，与固件强耦合 |
| `px4_ros_com` | 1.1 MB | 只用 `frame_transforms.h` | `enu_to_ned_local_frame` / `ros_to_px4_orientation` |
| `mavros` | 13 MB | 编译了，**手册从未使用** | 走 uXRCE-DDS |
| `ego-planner` | 7.7 MB | **未接入** | 发 `quadrotor_msgs/PositionCommand`，与真飞控链不通；`MotionTrajectory` 非其 `UniformBspline` |
| `gst_camera` | 24 KB | 4 目录 0 文件（空壳） | 补丁 04 走 UVC/v4l2 路线 |

> ⚠️ **整个工作空间不是 git 仓库**（`~/dzpx4`、本地 `dzsmartscout` 均 `NO_GIT`），
> **没有版本历史可回滚**，只能靠 `fc-audit/fixes/` 的备份。

### 1.4 机上非代码资产

| 路径 | 内容 |
|---|---|
| `~/dzpx4/1.txt` | 原始运行手册（14 条命令，**有 3 处错**，见 §2.8.1 差异表） |
| `~/dzpx4/src/px4_path/src/path/*.txt` | 3 条录制轨迹：`recorded_path.txt`(104 行)/`_1.txt`(120)/`_2.txt`(92) |
| `~/dzpx4/src/maps/*.pcd` | 3 张先验地图（`map.pcd`/`map_compile_4cm.pcd`/`output_voxel_0.05.pcd`） |
| `~/Fast-DDS-2.14.5` / `~/Micro-XRCE-DDS-Agent-2.4.3` | DDS / uXRCE；Agent 已装 `/usr/local/bin/MicroXRCEAgent` |
| `~/dzuav`（4.9 GB） | Jetson 上的视觉识别程序（HBB+TensorRT），与飞控无关 |
| `~/dzlink_tools/` | 补丁 05/06 部署的链路工具目录 |


---

## §2 逐模块详解

### 2.1 px4_control —— Offboard 唯一入口 + Goto 门控

> 目标包：`src/px4_control/`。版本 0.1.0。README 明确："`px4_control` 只负责键盘命令、
> 外部 Goto 准入和 PX4 Offboard 控制。路径文件、录制、优化与进度均由 `px4_path` 管理，
> 依赖方向只有 `px4_path -> px4_control`"（`README.md:3-5`）。

#### 2.1.1 节点与接口

| 节点 | 源文件 | 职责 |
|---|---|---|
| `px4_control_node` | `src/px4_control_node.cpp`（662 行） | Offboard 心跳、解锁、起飞、悬停、降落、Goto 门控、`TrajectorySetpoint` 下发 |
| `px4_teleop_node` | `src/px4_teleop_node.cpp`（177 行） | 读终端键盘，只发布 `/px4_control/command` |

**订阅**（`px4_control_node.cpp:46-64`）：

| 话题 | 类型 | QoS |
|---|---|---|
| `/fmu/out/vehicle_local_position` | `px4_msgs/VehicleLocalPosition` | `QoS(KeepLast(5)).best_effort()` |
| `/fmu/out/vehicle_status` | `px4_msgs/VehicleStatus` | 同上 |
| `/px4_control/command` | `px4_control/msg/ControlCommand` | depth 10 默认 |
| `/px4_control/goto` | `px4_control/msg/GotoTarget` | depth 10 默认 |

**发布**（`px4_control_node.cpp:66-74`）：

| 话题 | 类型 | QoS |
|---|---|---|
| `/fmu/in/offboard_control_mode` | `OffboardControlMode` | depth 10 |
| `/fmu/in/trajectory_setpoint` | `TrajectorySetpoint` | depth 10 |
| `/fmu/in/vehicle_command` | `VehicleCommand` | depth 10 |
| `/px4_control/goto_enabled` | `std_msgs/Bool` | **`QoS(1).reliable().transient_local()`（锁存）** |

**定时器**：`loop_rate_hz`（YAML 20）驱动 `on_timer`（`:75-78`），每 tick 无条件发心跳+设定值。

#### 2.1.2 状态机（`FlightState`，`px4_control_node.hpp:21-28`）

```text
DISARMED ──按0(Offboard)+按1(ARM)──► ARMING ──PX4报ARMED──► TAKEOFF
   ▲                                                          │ |z-目标z|<arrive_tol
   │                                                          ▼
   │◄── 离开Offboard(RC接管/failsafe) ──────────────────── HOVER
   │                                                       │ 按3
   │                                                       ▼
   └────── LANDED ◄──PX4报STANDBY──── LANDING ◄────────────┘
```

- `is_offboard()`：`nav_state == 14`（OFFBOARD），需 `status_received_`（`:98-100`）。
- `is_armed()`：`arming_state == 2`（ARMED）（`:102-104`）。
- `is_on_ground()`：disarmed 时恒真；armed 时用 `|z|<0.20` 且 `|vz|<0.15`（`:106-117`）。
- `set_state()` 只在状态变化时打日志（`:137-144`）。

#### 2.1.3 命令处理（`on_command` 分派，`:253-283`）

`ControlCommand.msg` 定义了 9 个命令常量（`msg/ControlCommand.msg:7-15`）：

```text
CMD_NONE=0 CMD_OFFBOARD=1 CMD_ARM=2 CMD_DISARM=3 CMD_LAND=4 CMD_MOVE=5
CMD_YAW=6 CMD_GOTO_ENABLE=7 CMD_GOTO_DISABLE=8
```

| 命令 | 处理函数 | 关键门槛（代码行） |
|---|---|---|
| `CMD_OFFBOARD` | `handle_offboard` `:335-346` | 需 `heartbeat_count_ >= heartbeat_before_offboard(20)`，否则拒绝；发 `VC_DO_SET_MODE(p1=1.0, p2=6.0=OFFBOARD)` |
| `CMD_ARM` | `handle_arm` `:348-380` | 仅 DISARMED/LANDED；`is_on_ground()`；`is_offboard()`；`local_pos_valid_`；设 `target_.z=-takeoff_height`；发 `VC_COMPONENT_ARM_DISARM(1.0)` |
| `CMD_DISARM` | `handle_disarm` `:382-396` | 非地面时**拒绝**并 `disable_goto_and_hold`；否则发 `VC_COMPONENT_ARM_DISARM(0.0)` |
| `CMD_LAND` | `handle_land` `:398-429` | `land_control=true` → 发 `VC_NAV_LAND`；false → 冻结 XY、`target_.z=current_.z` 开始自主下降 |
| `CMD_MOVE` | `handle_move` `:431-446` | 仅 HOVER；**goto_enabled_ 时拒绝** |
| `CMD_YAW` | `handle_yaw` `:448-466` | 仅 HOVER；goto 使能时拒绝；yaw 归一到 `[-π,π]` |
| `CMD_GOTO_ENABLE` | `handle_goto_enable` `:468-484` | **仅 HOVER 且 local_pos_valid**；置 `goto_enabled_=true`、`pursuit_active_=false`、清前馈 |
| `CMD_GOTO_DISABLE` | `handle_goto_disable` `:486-488` | 调 `disable_goto_and_hold("keyboard 5")` |

**常量定义**（`px4_control_node.cpp:12-24`）：`NAV_STATE_OFFBOARD=14`、`ARMING_STATE_STANDBY=1`、`ARMING_STATE_ARMED=2`、`VC_DO_SET_MODE=176`、`VC_NAV_LAND=21`、`VC_COMPONENT_ARM_DISARM=400`。

#### 2.1.4 Goto 目标准入（`on_goto`，`:285-333`）—— 核心门控

```cpp
if (!m->active) { disable_goto_and_hold("GotoTarget(active=false)"); return; }  // 无条件关许可
if (!goto_enabled_) { return; }              // 许可关闭 → 静默忽略
if (state_ != HOVER || !local_pos_valid_) return;
// 11 个字段逐一 isfinite，任一 NaN/Inf → 报错 + 关许可
target_ = m->x,y,z,yaw; pursuit_feedforward_ = m->v/a/yawspeed;
pursuit_active_ = true; last_goto_stamp_ = now;
```

- **`active=false` 不看当前许可状态，无条件关许可**（`:288-291`）——这是跟随节点唯一能"夺走"许可的手段。
- 超时兜底：`on_timer` 里 `now - last_goto_stamp_ > pursuit_goto_timeout(1.0s)` → 关许可悬停（`:579-586`）。
- `disable_goto_and_hold()`（`:152-171`）清 `goto_enabled_/pursuit_active_/前馈`，HOVER 下把 `target_=current_` 钉住。

#### 2.1.5 发布设定值（`publish_trajectory_setpoint`，`:505-546`）—— NaN vs 0 的坑

```cpp
const float NaN = quiet_NaN();
m.velocity = (LANDING&&!land_control) ? {NaN,NaN,descent_vz}
            : (pursuit_active_&&HOVER) ? {ff.vx,ff.vy,ff.vz}
            : {NaN,NaN,NaN};
m.acceleration = (pursuit_active_&&HOVER) ? {ff.ax,ff.ay,ff.az} : {NaN,NaN,NaN};
```

> 注释（`:506-509`）明确："PX4 v1.14+ expects unused setpoint components filled with NaN.
> A zero in velocity/acceleration/jerk is treated as a feed-forward, which fights the
> position controller and causes overshoot / bounce on takeoff."

**这就是 R-07 缺陷的直接来源**：`path_follow_waypoint_node` 显式发 v/a/yawspeed=0（`path_follow_waypoint_node.cpp:184-191`），与这里"0=前馈会打架"的结论自相矛盾。

#### 2.1.6 自主降落（`LANDING`，`on_timer :610-646`）

- `land_control=true`：等 `arming_state==STANDBY && is_on_ground()` → LANDED（`:614-616`）。
- `land_control=false`：`v = agl>land_slow_alt_ ? land_fast_speed_(0.4) : land_slow_speed_(0.2)`；`target_.z += v*dt`，上限 `land_touch_z_+0.30 = 0.35`（NED 下行 z 增大）。触地判断完全交给 PX4 land detector + `COM_DISARM_LAND`。

#### 2.1.7 键盘遥操（`px4_teleop_node.cpp`）

| 键 | 作用 | 代码行 |
|---|---|---|
| `0` | Offboard | `:106-109` |
| `1` | ARM + 起飞 | `:110-113` |
| `2` | DISARM | `:114-117` |
| `3` | LAND | `:118-121` |
| W/S/A/D | ±X/±Y 0.5 m（NED） | `:124-135` |
| Q/E | 上/下 0.1 m | `:136-141` |
| L/R | yaw ±0.15 rad | `:146-151` |
| `4` | 使能外部 GOTO | `:153-156` |
| `5` | 撤销 GOTO 并保持 | `:157-161` |

- **必须在真 TTY 跑**：`tcgetattr(STDIN_FILENO, ...)` + `raw.c_lflag &= ~(ICANON|ECHO)`（`:46-58`）。用 `setsid`/管道启动会 `tcgetattr failed`，键盘静默失效。
- 单线程 `keyboard_loop`，`VMIN=0, VTIME=1`（100 ms 超时读）（`:54-56`）。

#### 2.1.8 参数（`config/params.yaml`）

| 键 | YAML | 代码默认 | 说明 |
|---|---|---|---|
| `takeoff_height` | **0.5** | 1.2 | 起飞目标高度 m。⚠️ YAML 把代码默认 1.2 压到 0.5（贴近地面） |
| `step_xy` / `step_z` | 0.5 / 0.1 | 同 | WASDQE 单次步进 |
| `loop_rate_hz` | **20** | 50 | Offboard 心跳/设定值频率 |
| `arrive_tol` | 0.15 | 同 | 起飞到位判据 m |
| `heartbeat_before_offboard` | 20 | 同 | 按 0 前需已发够心跳（20Hz 下 = 1s） |
| `land_control` | false | false | false=自主下降，true=PX4 NAV_LAND |
| `land_fast/slow_speed` | 0.4 / 0.2 | 同 | 降落分段速度 |
| `land_slow_alt` | 0.5 | 同 | 切换慢速的 AGL 阈值 |
| `land_touch_z` | 0.05 | 同 | 触地判据 |
| `yaw_step_rad` | 0.15 | 同 | L/R 单次偏航 |
| `pursuit_goto_timeout` | **1.0** | 1.0 | Goto 看门狗秒数 |

#### 2.1.9 关键缺陷（详见 §2.10）

- R-07：waypoint 前馈 0 vs trajectory NaN 自相矛盾（`path_follow_waypoint_node.cpp:184-191` vs `px4_control_node.cpp:505-509`）。
- R-03：`pursuit_goto_timeout=1.0s` 偏紧；`start_trajectory()` 在许可回调里同步构建样条，耗时未知。
- R-10：自主降落依赖 land detector + `COM_DISARM_LAND`（未验证），无超时兜底。

### 2.2 px4_path —— 录制 + 双模式跟随 + 轨迹生成

> 目标包：`src/px4_path/`。版本 0.1.0。约 2100 行 C++。

#### 2.2.1 职责与依赖

**一句话**：把 FAST-LIO 的 SLAM 轨迹录成文本、读回来转成 PX4 NED，并变成一串外部位置参考
（`px4_control/msg/GotoTarget`）通过 `/px4_control/goto` 发给 `px4_control`；自己**不碰** Offboard/解锁/起降/`/fmu/in/*`。

```text
FAST-LIO(/localization, nav_msgs/Odometry)
      │（录制时）
      ▼
path_record_node ──► TXT (x y z qx qy qz qw, SLAM 原始坐标)

path_follow_node ──► /px4_control/goto (px4_control/msg/GotoTarget, NED)
      ▲
      └── /px4_control/goto_enabled (std_msgs/Bool, 锁存) ◄── px4_control
      └── /fmu/out/vehicle_local_position ◄── PX4
```

- 上游输入：`/fmu/out/vehicle_local_position`、`/px4_control/goto_enabled`、`/localization`（仅录制）。
- 下游输出：`/px4_control/goto`（唯一控制输出）、`/path_record/visualization`（仅 RViz）。
- 编译期依赖：`px4_control`（只要 `msg/GotoTarget`）、`px4_msgs`、`geometry_msgs`、`nav_msgs`、`rclcpp`、`std_msgs`。

#### 2.2.2 CMake 编译真相：2 个可执行，7 个 .cpp 全参与

| 产物 | 类型 | 编译的源文件 |
|---|---|---|
| `px4_path_core` | 静态库 | `motion_trajectory.cpp`、`path_loader.cpp`（被 `path_follow_node` 链接） |
| `path_record_node` | 可执行 | `path_record_node.cpp`、`path_record_process.cpp`（**不链接 core**） |
| `path_follow_node` | 可执行 | `path_follow.cpp`、`path_follow_waypoint_node.cpp`、`path_follow_trajectory_node.cpp`（链接 core） |

**机上 `install/` 只有 `path_record_node` 与 `path_follow_node` 两个可执行**，与上表完全吻合。

#### 2.2.3 录制链路（`path_record_node.cpp`）

```text
① 节点构造即 recording_=true（:106-107），先发空 Path 清 RViz（:210）
② 每帧 /localization（slam_topic，默认 /localization）:
   frame_id 检查（:392-408）
   → NaN/Inf + 四元数归一（:421-425）
   → 原始跳变检查 raw_step > max_neighbor_dist_m → 丢帧（:429-438）
   → 可选在线滑动均值 filter()（:442-443）
   → 关键点判定（:446-483）：水平≥min_xy 或竖直≥min_z 才记录
③ Ctrl+C → 析构 → stop_recording（:253-293）
   → append_final_pose 补末点（:224-251）
   → finalize = 中心滑动平均 + 近点压缩（path_record_process.cpp:74-79）
   → write_path_file：相邻距离终检 → 写 .tmp → flush/good → close → fs::rename 原子替换（:295-361）
```

- **只在析构（正常退出）时落盘**（`:114-118`）。`kill -9` / 断电 = 整段录制丢失。
- 文件格式（`:324-341`）：8 行 `#` 头 + 每行 `x y z qx qy qz qw`（fixed 9 位小数），**保存 SLAM 原始坐标**，不做 ENU/NED。
- **无定时器**，全部由里程计回调驱动。
- 参数校验失败 → 失效安全：保留节点运行 + 错误日志，但关闭录制（`:121-163`）。

#### 2.2.4 加载链路（`path_loader.cpp`）—— 唯一转换点

```text
SLAM → ENU   (x,y,z) → (-x,-y,z)                path_loader.cpp:30-32
ENU → NED    (x,y,z) → (y,x,-z)                 path_loader.cpp:34
yaw_slam = atan2(2(qw qz + qx qy), 1-2(qy²+qz²))   :119-120
yaw_enu  = wrap_pi(yaw_slam + π)                      :121
yaw_ned  = wrap_pi(π/2 - yaw_enu)                     :122
```

- 逐行 `trim` → 跳空行/`#` → 必须**正好 7 个浮点**，多一个 token 即整条失败（`:76-82`，防手误）。
- z 越界只 warning（`:89-94`）；`|q|` 越界 warning 但仍归一化（`:96-115`）。
- 点数 < min_points 整条失败（`:132-138`）；相邻点 > max_neighbor 整条失败（`:142-154`）。
- 加载结果校验**与 `px4_odom_convert` 的坐标链路逐项一致**（已用 `verify_frames.py` 2000 组随机位姿数值验证，见 §6）。

#### 2.2.5 两种跟随模式对照（最关键的算法选择点）

| | **waypoint（当前生效）** | **trajectory** |
|---|---|---|
| 实现 | `path_follow_waypoint_node.cpp`（295 行） | `path_follow_trajectory_node.cpp`（590 行） |
| 机制 | 离散航点逐点推进游标 | 曲率感知自然三次样条（三对角解 + 均匀弧长重采样） |
| 到位判据 | `switch_tol 0.10 m` 切下一点 | `terminal_tol 0.15 m` 进 HOLD |
| 单步限幅 | `max_step_dist 1.5 m`（`clamp_step()` :104-117） | 辅助点间距 1.5 m |
| 前馈 | **速度/加速度/yawspeed 全 0** | 位置+速度+加速度+yaw+yawspeed 全套 |
| 动力学约束 | 无（交给 PX4 位置环） | 0.8 m/s、1.0 m/s²、jerk 2.5、向心 0.35 |
| 误差保护 | 无 | 1.00 m 降速 → 1.50 m 暂停 → 5.00 m 中止 |
| 航向模式 | 仅 `fixed` | `fixed` / `recorded` / `tangent` |
| 急弯处理 | 无 | `segment_split_turn_deg 140°` 自动分段 |

**waypoint 游标推进逻辑**（`:260-293`）：

```cpp
while (cursor_+1 < size && dist(current_pos_, points_[cursor_]) < switch_tol_) ++cursor_;
const Waypoint raw = points_[cursor_];
const Waypoint sent = clamp_step(current_pos_, raw);   // 限制单步 ≤ max_step_dist
publish_goto(sent);
if (cursor_+1 == size && dist(current_pos_, points_.back()) < terminal_tol_)
    set_state(HOLD, "reached terminal point");
```

**trajectory 跟踪器**（`on_timer :450-588`）逐步：

1. `!local_pos_valid_` → 直接 return，不发任何消息（`:451-453`）。
2. 状态必须是 PURSUING 且 `active_segment_index_ < trajectories_.size()`（`:455-462`）。
3. 算 `timer_dt`，若 `≤0` 或 `>0.5s` 用标称周期（`:464-469`）。
4. 用当前参考点算三维误差 `current_error`（`:471-481`）。
5. **参考时钟缩放**：误差 ≥ `tracking_pause_error_m` → `scale=0`（冻结）；介于 slowdown 与 pause → 线性降到 0；否则 1（`:483-490`）。变化率受 `max_time_scale_rate_per_s` 限制（`:491-494`）。
6. **重定时下的加速度可行域夹紧**（`:496-523`）：链式法则 `a_retimed = a·k² + v·k̇`，把 `k̇` 限制在使 `|a_retimed|≤max_accel` 的区间（解二次不等式；判别式<0 → `k̇=0`）。
7. 推进 `trajectory_elapsed_s_ += 0.5(k_prev+k_new)·dt`（`:524-530`）。
8. 采样并算真实跟踪误差；超 `max_tracking_error_m` → 报错、退回 LOADED、发 `active=false`（`:532-554`）。
9. 段末且误差 < `terminal_tol` → 要求 `current_speed_mps_ ≤ segment_switch_speed_mps` 才切段（`:556-576`）；否则 HOLD（`:577-580`）。

> **结论**：本包**没有任何反馈控制器**。位置环完全由 PX4 承担；本包只提供 (p,v,a,yaw,yawspeed) 参考，并用标量参考时钟缩放做"沿轨迹方向的追赶调制"。它**不能做横向纠偏**（无横向误差项进入输出）。

#### 2.2.6 轨迹生成（`motion_trajectory.cpp`，527 行）—— 插值型自然三次样条

**不是**多项式最优控制、不是 B 样条、不是采样式。步骤：

1. **预处理去重**（`:245-268`）：相邻距离 < `kMinPointDistanceM=1e-2`（`:11`）的点合并（后者 yaw 覆盖前者），保证节点严格递增。
2. **空间样条**（`:270-307`）：以**累积弦长** `knots[i]=knots[i-1]+|P_i-P_{i-1}|`（`:290-293`）为参数，对 x(s)/y(s)/z(s) 各建 `NaturalCubicSpline::build`（自然边界 M''=0，Thomas 算法解三对角，`:67-119`）。yaw 单独建**解缠后**的样条（`:284-289`）。位置 **C² 连续**。
3. **曲率限速**（`:312-350`）：在 `profile_count = max(2, 2N, ceil(len/profile_resolution_m)+1)` 个均匀弧长采样点上算
   `κ = |P'×P''| / |P'|³`（`:334-335`），令 `v ≤ sqrt(max_centripetal_accel / κ)`（`:336-339`）。`|P'|<1e-6` → 判退化失败（`:328-332`）。
4. **前向/后向加速度约束**（`:352-363`）：首末速度强制 0；前向 `v_i = min(v_i, sqrt(v_{i-1}² + 2a·Δs_i))`；后向同式反向。Δs 用弦长（≤真实弧长 → 保守）。
5. **时间参数化**（`:365-379`）：梯形速度假设 `Δt_i = 2Δs_i/(v_i+v_{i+1})`；速度和过小时退化 `sqrt(2Δs/a)`（`:369-372`），每段时长下限夹到 1ms（`:378`）。
6. **时间样条 s(t)**（`:473-480`）：`build_clamped(knots=t_i, values=s_i, 首斜率=0, 末斜率=0)`，保证首末速度 0；`duration_s_=t_last`。
7. **全量限值测量**（`:482-518`）：在 `max(2000, 40·节点数)` 个时间采样点上用链式法则算
   `v=P'(s)·ṡ`、`a=P''(s)·ṡ²+P'(s)·s̈`、`j=P'''(s)·ṡ³+3P''(s)·ṡ·s̈+P'(s)·s⃛`（**jerk 只用于校验，不发布**），并对每点重算向心加速度 `κ·|v|²`。负速度 → 四个限值置 +inf 并提前返回（`:492-499`）。
8. **全局时间拉伸**（`:393-409`）：`scale = max(1, v_ratio, sqrt(a_ratio), cbrt(j_ratio), sqrt(acent_ratio)) × kLimitSafetyScale(1.001)`，把所有时间节点乘 scale 后重建样条并重测。数学上正确：时间乘 k ⇒ 速度÷k、加速度÷k²、jerk÷k³、向心÷k²。

**急弯分割 `find_path_turn_splits`**（`:28-65`）：对每个内部点算入/出方向夹角的补角 `angle=acos(clamp(cos)) ≥ threshold_deg` 记切点。`threshold_deg≤0` 或 >180 或点数<3 → 返回空。**一次 `start_trajectory` 里被调用两次**（`:204`、`:289`），纯重复计算。

**时间参数化与约束汇总**：

| 约束 | 参数 | 强制点 |
|---|---|---|
| 速度上限 | `max_speed_mps` | 初始化 + 拉伸校验 |
| 加速度上限 | `max_accel_mps2` | 前向/后向两遍 + 拉伸校验 |
| jerk 上限 | `max_jerk_mps3` | 只测量 → 参与拉伸 → **不发给 PX4** |
| 向心加速度上限 | `max_centripetal_accel_mps2` | 曲率限速 + 拉伸校验 |
| 首末速度 0 | 硬编码 | `:352-353` |

> **验证缺口**：拉伸后**没有再断言**测量值是否真的落在限值内（`:408` 调了 `measure_limits()` 但结果只写入成员、无人校验）。

#### 2.2.7 状态机与暂停恢复

```text
INIT ──首帧 local_position + 路径加载成功──► LOADED
LOADED ──goto_enabled=true─────────────────► PURSUING
PURSUING ──goto_enabled=false────────────────► PAUSED（冻结游标/轨迹时钟）
PURSUING ──到达终点(t<terminal_tol)─────────► HOLD（持续重发终点，防看门狗超时）
任意态 ──内部错误────────────────────────────► 发 active=false，控制节点关 Goto
```

- 跟随节点订阅**锁存**话题 `/px4_control/goto_enabled`（两侧 QoS 均为 `QoS(1).reliable().transient_local()`）。
- `false` → 进入 PAUSED 冻结进度；`true` → 首次启动或从 PAUSED 恢复。
- trajectory 恢复时**只重置 timer 基准，不重建轨迹、不累计暂停时长**（`on_goto_enabled :442-444`）。
- **路径只在节点构造时读一次**（`reload_path()` 只在 `:72`/`:51` 调用）。日志里 "press 4 after creating a valid path" **是错的**——`'4'` 不重新读文件。
- 两个跟随节点约 150 行模板代码逐字重复（state_name/set_state/distance/reload_path/on_goto_enabled），建议抽 `PathFollowBase`。

#### 2.2.8 参数（`config/params.yaml`）—— 录制/跟随/轨迹共用一份

**公共（`path_follow_node`）**：

| 键 | 值 | 行 | 说明 |
|---|---|---|---|
| `path_dir` | `/home/duzhong/dzpx4/src/px4_path/src/path` | 21/114 | ⚠️ 硬编码绝对路径，换机必改；空值=拒绝 |
| `path_file` | `recorded_path_2.txt` ⚠️ | 25 | 与录制段 `output_file` 不一致（R-04） |
| `pursuit_rate` | 20.0 Hz | 29 | GotoTarget 发布频率 |
| `switch_tol` | 0.10（waypoint） | 32 | 切下一点的三维距离 |
| `terminal_tol` | 0.15 | 35 | 进 HOLD 的距离 |
| `max_step_dist` | 1.5 | 39 | waypoint 单步限幅 / trajectory 连接段步长 |
| `max_neighbor_dist_m` | 5.0 | 43 | 加载相邻点硬门限 |

**trajectory 专用**：`max_speed_mps 0.8`、`max_accel_mps2 1.0`、`max_jerk_mps3 2.5`、`max_centripetal_accel_mps2 0.35`、`profile_resolution_m 0.12`、`segment_split_turn_deg 140.0`、`segment_switch_speed_mps 0.10`、`tracking_slowdown_error_m 1.00`、`tracking_pause_error_m 1.50`、`max_time_scale_rate_per_s 1.2`、`max_tracking_error_m 5.0`、`yaw_mode fixed`、`yaw_lookahead_s 0.5`、`max_yaw_rate_rps 0.5`。

**录制（`path_record_node`）**：`slam_topic /localization` ⚠️（不存在）、`expected_frame_id ""`、`visualization_topic /path_record/visualization`、`path_dir`、`output_file recorded_path.txt` ⚠️、`overwrite_existing false` ⚠️（代码默认 true）、`enable_filter_smoothing true`、`min_xy_m 0.12`、`min_z_m 0.10`、`max_neighbor_dist_m 5.0`、`min_record_interval_s 0.05`、`filter_window 5`、`smoothing_window 3`、`min_smoothed_point_dist_m 0.05`、`min_points 2`。

> **两套默认值陷阱**：YAML 与代码内默认值不一致，YAML 缺失时会静默用代码值（waypoint：YAML `pursuit_rate 20.0`/代码 `10.0`；trajectory：YAML `50.0`/代码 `10.0` 等）。

#### 2.2.9 launch（`launch/px4_path.launch.py`）

- `record_mode` 从 YAML 读（`:11-27`），**没有同名 launch 参数**。
- `record_mode=true` → 只起 `path_record_node`；`false` → 只起 `path_follow_node`（再由 `follow_mode` 二选一）。
- `follow_mode` 由进程入口 `read_follow_mode()` 读（`src/path_follow.cpp:11-14`）。

### 2.3 px4_odom_convert —— 定位进 PX4 的最后一公里

> 目标包：`src/px4_odom_convert/`。增强版 v2（2026-10-17），头注释见 `include/px4_odom_convert.hpp:3-27`。

#### 2.3.1 三种模式与现状

| `mode` | 数据来源 | 频率 | 现状 |
|---|---|---|---|
| `odom` | `/Odometry`（fastlio_mapping 直接发） | 跟随输入（~44 Hz） | ✅ **当前唯一可用** |
| `localization` | `/localization` | 50 Hz | ❌ 订阅不存在的话题 |
| `tf` | TF 树 `map → lidar` | `loop_rate_hz 50` | 未验证 |

- **代码默认值本来就是 `odom`**（`px4_odom_convert.cpp:43`），是 **launch 第 35 行把它覆盖成 `localization`**。
- 补丁 01 修的就是这一行（`fc-audit/fixes/01-odom-source/files/px4_odom_convert.launch.py:54-57`）。
- 为什么 `mode:=odom` 与 `localization` 数值等价：`transform_fusion.py:54` 在没收到 `/map_to_odom` 时用 `np.eye(4)`，于是 `/localization == /Odometry`。

#### 2.3.2 坐标转换链路

```cpp
R_enu_slam_ << -1,0,0,  0,-1,0,  0,0,1;   // = Rz(180°)   (:51-54)
t_enu_slam_ << 0,0,0;                       (:56-59)
position_slam = (p.x, p.y, p.z - z_offset_m);
position_enu  = R_enu_slam_ * position_slam + t_enu_slam_;
q_enu  = R_enu_slam_ * q_slam.toRotationMatrix();
q_px4  = ft::ros_to_px4_orientation(q_enu);
position_ned = ft::enu_to_ned_local_frame(position_enu);
```

- 与 `path_loader` 的坐标链路**逐项一致**（见 §2.2.4）。
- ⚠️ **唯一耦合条件**：`z_offset_m` 必须保持 0（`loader` 里没有对应项）。一旦非 0，路径高度整体差该偏移。

#### 2.3.3 自动对齐（auto-align）

- `enable_auto_align=true`：SLAM 位姿直通 + **一次** `reset_counter` bump 重锚 EKF 到 SLAM 帧。
- `align_yaw_to_ekf=false`（默认）：对齐完成条件 `align_sample_count_ >= 30`（`:566-579`）。
- `align_yaw_to_ekf=true`：对 SLAM yaw 求 ~1s 平均后冻结（`:542-565`）。
- 对齐完成后对位置施加 `Rz(yaw_offset_)` 旋转（`:582-589`）。
- **z 轴换源**（`:592-599`）：用 EKF 反馈 z 替换 SLAM z（SLAM z 漂 5.56/8.9 m，EKF z 由气压计主导）。

#### 2.3.4 jump guard v2 —— 三级防护（本包技术含量最高）

> 背景：FAST-LIO ICP 摇摆时 `/localization` 持续输出 26~585m 跳变；旧版"连续 8 帧即接受"把疯跳当真实位移放行且每次 bump，EKF xy_reset 每秒+1（累计 3764 次）。

| 级别 | 触发 | 行为 |
|---|---|---|
| **L1 insane 熔断** | 单帧位移 > `slam_insane_m_(2.0)` 或 yaw > `2*jump_yaw_rad_` | 判定 SLAM 无效：hold 到底、**绝不 bump**；恢复需 SLAM 相邻帧稳定 `slam_recover_n_(86 帧≈2s)` |
| **L2 bump 节流** | 真实大位移接受后 | bump 最小间隔 `bump_min_interval_s_(10s)`；窗口内 `gate_prev_` 与 `gate_out_` 保持不动 |
| **L3 EKF reset 风暴熔断** | 60s 滑动窗内 reset > `ekf_storm_max_resets_(5)` | `ekf_storm_=true`，冻结一切 accept/bump，hold 输出保 EKF，风暴缓解自动恢复 |

**armed 门控**：`fcu_armed_` 时 jump 永不接受大位移（物理不可能）——仅地面待命允许"被搬运"式重锚（`:752-756`）。

**常规路径**（`:726-823`）：

```cpp
gate_dp = |position_ned - gate_prev_pos_|;
if (gate_dp > slam_insane_m_ || gate_dy > 2*jump_yaw_rad_) { slam_bad_=true; ... }   // L1
else if (gate_dp > jump_hold_m_ || gate_dy > jump_yaw_rad_) {                       // 大位移
   confirmed = !fcu_armed_ && !ekf_storm_ && gate_hold_count_ >= jump_confirm_n_;
   if (confirmed) { bump_ok = ...间隔...; if (bump_ok) { ++ev_reset_counter_; ... } else { hold } }
   else { hold previous pose }
}
```

**hold 期间方差放大两个数量级**（`:860-870`）：`gate_pos_var = gate_holding ? 100.0 : 0.01`，EKF 大幅降低该测量权重。

#### 2.3.5 可观测性（v2.1）

- **事件级 blackbox** `writeBlackbox()`（`:350-398`）：4 行完整上下文（event/t/trans/armed/aligned；jump；ekf；slam_q）。
- **3s 周期健康行** `onHealthTimer()`（`:400-450`）：`[px4_odom_convert_health] t=... guard=... slam_frames=... slam_hz=... hold_ratio=... l1_lock=... bump_age_s=... storm=... ekf_reset_60s=...`，机上 `localization_watchdog.py` 据此感知定位质量。
- **防双实例**：启动时检查 VIO 话题已有发布者（`:65-75`），`>0` 抛异常拒绝启动；运行期 10s 定时器检查双 VIO / 双 SLAM 源（`:183-209`）。

#### 2.3.6 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `source_mode` | `odom`（代码）/ `localization`（launch） | 见 §2.3.1 |
| `odom_topic` | `/Odometry` | odom 模式订阅 |
| `localization_topic` | `/localization` | localization 模式订阅 |
| `tf_map_frame` / `tf_lidar_frame` | `map` / `lidar` | tf 模式 |
| `z_offset_m` | 0.0 | 雷达安装高度补偿，三种模式均生效 |
| `loop_rate_hz` | 50.0 | tf 模式查询频率 |
| `enable_auto_align` | true | 自动对齐开关 |
| `align_yaw_to_ekf` | false | 是否旋转对齐 EKF 航向 |
| `slam_insane_m` | 2.0 | L1 熔断门限 |
| `slam_recover_n` | 86 | L1 恢复所需稳定帧数 |
| `bump_min_interval_s` | 10.0 | L2 bump 最小间隔 |
| `ekf_storm_window_s` | 60.0 | L3 滑动窗 |
| `ekf_storm_max_resets` | 5 | L3 风暴阈值 |

> ⚠️ **幽灵参数**：launch 传了 `max_position_jump_m 0.5` / `max_attitude_jump_rad 0.5`（`launch:18-19`），但节点**从未声明**这两个参数 → **位置/姿态跳变保护在代码里根本不存在**（见 §2.10 R-12）。

### 2.4 px4_imu_bridge —— PX4 SensorCombined → ROS Imu

> 目标包：`src/px4_imu_bridge/`。

- 订阅 `/fmu/out/sensor_combined`（`px4_msgs/SensorCombined`，SensorDataQoS）→ 发布 `/fmu/out/imu_data`（`sensor_msgs/Imu`）。
- **FRD→FLU 转换**（`:42-60`）：X 不变，Y/Z 取反。`convert_frd_to_flu` 可关。
- **时间戳用 PX4 开机时间**（`:104-109`），不是 ROS 时间 → 与其他节点时间轴不一致。
- **方位角无效**：`orientation_covariance[0] = -1.0` 标记（`:112-118`）。
- 周期统计日志 `timing_log_period_ms`（默认 1000ms）。
- launch 参数：`input_topic`、`output_topic`、`frame_id`、`convert_frd_to_flu`、`log_conversion_time`、`timing_log_period_ms`。

### 2.5 dzuav —— 机载视觉侦察栈（C++，TensorRT）

> 目标包：`src/uav_camera_code/onboard/dzuav/`。单目标可执行 `dzuav`（CMakeLists.txt:18-33）。

#### 2.5.1 职责边界

机载**没有**飞控/建图/定位/自主飞行代码——那些全部由官方 `dzpx4` 栈负责。我们维护的只有
**机载侦察栈 `onboard/dzuav`**（C++）：把相机画面变成目标世界坐标，通过 UDP 发给车。

```
[机载 Orin Nano]
  ┌─ dzpx4（官方，直接用，不改）────────────────────────────┐
  │  MID360 激光 → FASTLIO2 建图/定位 → PX4 飞控              │
  │  px4_control / px4_path 执行航线（起飞/巡航/航线/返航）     │
  └──────────────────────────────────────────────────────────┘
  ┌─ dzuav（我们维护的侦察栈，C++）──────────────────────────┐
  │  AR0234 下视相机 → 去畸变 → HbbDetector(7类检测)           │
  │    → CellClassifier(格号识别) → MapLocalizer(PnP 自定位)   │
  │    → UdpBroadcaster（二进制 UDP :50000 广播）              │
  └──────────────────────────────────────────────────────────┘
            │ UDP :50000
            ▼
[车端香橙派] 官方 dzrobot：path_executor / dztrack / Nav2 负责开车
            我们的 ag_integrate：收 :50000 → 世界模型 → 官方车命令 + 禁火保险
```

#### 2.5.2 单帧处理流水线（`processFrame`，`src/main.cpp:70-188`）

```text
① MapLocalizer.undistort(frame)   去畸变（映射预计算，同分辨率只算一次）
② HbbDetector.infer(image)        TensorRT 推理，1280×1280，7 类目标
③ 遍历检测结果：
   ├─ CellLabel 类 → 裁剪 ROI → CellClassifier.infer(roi) → 格号 CellId
   │  （收集 landmarks: (CellId, 像素中心点)；贴边半标签丢弃 :84-88）
   └─ 其它类暂存
④ MapLocalizer.solve(landmarks, size)  PnP 自定位（需 ≥4 个有效格号）
⑤ 定位成功 → MapLocalizer.project(像素中心) → 场地世界坐标（米）→ WorldObject
⑥ UdpBroadcaster.send(timestamp, objects)   二进制 UDP 广播 :50000
⑦ stdout: localized=1/0
```

**车朝向估计**（`estimateCarYaw`，`main.cpp:43-68`）：检测 `ObjectClass::Car` 的框，
在框内找红色方向标记质心，经 `map.project` 投到世界坐标取 `atan2`；输出帧 `msg_id=0x4002`。
车端 `integrate_node`（`heading_bias_source=udp`）据此修正 reset 断言，根除绕圈。

#### 2.5.3 类别定义（`include/dzuav/types.hpp:11-20`）

```cpp
enum class ObjectClass : int {
    Mine=0, Car=1, Arrow=2, TargetRed=3, CellLabel=4, TargetBlue=5, Obstacle=6
};
```

- `CellId`（`:23-38`）：列字母 A~N + 行号 1~11，`valid()` 校验范围。
- `Detection`：像素框 + 置信度。
- `WorldObject`：世界坐标 + length/width。

> ⚠️ 顺序必须与 `main.cpp className()` 表（`main.cpp:33-38`）及训练标签严格一致。

#### 2.5.4 TensorRT 引擎加载（`TensorRtEngine.hpp`）

- 序列化 .engine → `createInferRuntime` → `deserializeCudaEngine` → `createExecutionContext` → CUDA stream（`:21-43`）。
- 析构：`delete context_/engine_/runtime_` + `cudaStreamDestroy`（`:45-53`）。TRT 8.x 起 `destroy()` 已废弃、10.x 移除。

#### 2.5.5 HbbDetector（`src/HbbDetector.cpp`，91 行）

- 输入 `images` shape `{1,3,1280,1280}`，输出 `output0`（`11×33600` float，即 7 类 × 33600 检测框）（`:11-14`）。
- 页锁定主机缓冲 H2D/D2H DMA 直传（`:16-18`）；张量地址构造期绑定一次（`:25-27`）。
- `infer()`（`:29-90`）：resize+pad 到 1280² → `blobFromImage` → `enqueueV3` → 解析每点 7 类最大分数 → 置信度过滤 → **逐类 NMS**。
- 构造参数：`confidence=0.25, nms=0.45`（`main.cpp:276/344`）。

#### 2.5.6 CellClassifier（`src/CellClassifier.cpp`，85 行）

- 双头 MobileNetV3-Small：`letter_logits`（14 个列字母 A~N）+ `row_logits`（11 个行号）（`:9-20`）。
- 输入 96×96，**±10° TTA**（`-10,-5,0,5,10`），每角度独立推理后对 softmax 概率取平均再 argmax（`:22-28,36-76`）。
- 置信度 = `min(letter_prob, row_prob)`（`:83`）。
- 主流程调用阈值 `id.confidence > 0.45F`（`main.cpp:90`）；文档另说 0.55（`docs/无人机工作流程.md:77`）。

#### 2.5.7 MapLocalizer（`src/MapLocalizer.cpp`，153 行）—— PnP 自定位

**内参写死**（`:25-26`）：
```cpp
K_ = (995.153295, 0, 1010.494099; 0, 997.758431, 481.641588; 0,0,1);
dist_ = (-0.041172, -0.003300, -0.008916, 0.000707, 0);
```

**格号世界坐标**（`world()`，`:42-45`）：
```text
world_x = 0.065 + (column - 'A') × 0.35     column: 'A'=0 .. 'N'=13
world_y = 0.295 + (11 - row) × 0.35         row: 1..11
world_z = 0                                  地面平面
```

**solve()**（`:46-146`）：
1. 有效格号 < 4 → 返回 false。
2. 格号凸包面积 < 2% 画面 → false（`:62-64`）。
3. **主路径 PnP**：`cv::solvePnPRansac(..., SOLVEPNP_P3P)` + 重投影中位误差 ≤10px 质检（`:74-104`）。
4. **兜底单应拟合**：`findHomography(RANSAC, 5.0)` + 中位误差 ≤15px（`:119-141`）。
5. 成功后计算 `Hinv = H.inv()`（像素→世界）。

**project()**（`:147-152`）：`[X,Y,1]^T = Hinv × [u,v,1]^T`，返回 `X/Z, Y/Z`。

**去畸变缓存**（`undistort()`，`:28-41`）：同分辨率只算一次映射，每帧仅 `remap`，输出与逐帧 `undistort` 逐位等价。

#### 2.5.8 UdpBroadcaster（`src/UdpBroadcaster.cpp`，101 行）—— 私有二进制协议

帧布局（`UdpBroadcaster.hpp:11-13`）：
```text
0     4   magic       0xef 0xef 0xef 0xef
4     1   version     0x10
5     8   timestamp   uint64，机上毫秒时间戳
13    2   msg_id      0x4001（目标帧）/ 0x4002（车位姿帧）
15    4   length      uint32 = 2 + 25 × count
19    2   count       uint16
21    25N objects     type(u8) + x(f64) + y(f64) + length(f32) + width(f32)
21+25N 2  crc16       uint16，CRC-16/Modbus（多项式 0xa001），覆盖 version 到最后一个目标
+2    4   suffix      0xfe 0xfe 0xfe 0xfe
```

- `send()`（`:57-76`）：目标帧。
- `sendCarPose()`（`:78-91`）：车位姿帧，body = `x(8) + y(8) + yaw(4)`，共 45 字节。
- CRC 查表（`:14-30`）：`crc = (c>>8) ^ table[(c ^ d[i]) & 0xff]`；`"123456789" → 0x4B37`。
- 广播地址 `INADDR_BROADCAST`（`:98`）。

> ⚠️ **`dzlink_supervisor` 不解析这个私有帧**（它只吃 JSON）。见 §2.7.4。

#### 2.5.9 相机管道与运行模式（`main.cpp`）

| 模式 | 命令 | 管道 |
|---|---|---|
| USB UVC | `dzuav uvc:/dev/video0 ... 1920x1080@30` | `v4l2src → image/jpeg → jpegdec → videoconvert → BGR → appsink`（MJPEG）；失败回退 raw YUY2（5fps）（`:206-218,262-268`） |
| MIPI CSI | `dzuav argus:0 ...` | `nvarguscamerasrc → nvvidconv → BGR → appsink`（`:191-202`） |
| 离线图片/视频 | `dzuav test.jpg ... out_dir/` | `cv::imread` / `VideoCapture`，可落盘可视化 |

- 环境变量：`DZUAV_PREVIEW=1`（开实时预览窗口，按 q 退出）、`DZUAV_FORCE_UDP`（定位失败也广播）。
- 默认 UDP 端口 50000，引擎 `~/dzuav/engines/hbb_detector_fp16.engine`、`cell_classifier_fp16.engine`。

#### 2.5.10 配套工具（`src/uav_camera_code/tools/`）

| 脚本 | 作用 |
|---|---|
| `orin_build_verify.sh`（280 行） | 机上一键编译 + L0~L3 分层验证（L3 用**车端同一个 `intel_protocol.parse`** 校验 UDP 帧） |
| `orin_run_dzuav.sh`（77 行） | 启动 dzuav（camera/image/video），默认 `~/ag_test/dzuav` + `~/dzuav/engines` |
| `preflight.py`（246 行） | 赛前三端体检（本地/车端/机载），`--match` 比赛模式 |
| `release_models.py` | 导出 ONNX/TRT + 登记表 md5 |
| `calibrate_camera.py` | 标定板生成/采集/标定/注入 `MapLocalizer.cpp` 内参 |
| `sim_match.py` | 仿真匹配（世界坐标 + 内参） |
| `remote_run.py` / `orin_log_capture.py` / `capture_dataset_local.py` / `orin_capture_dataset.py` / `train_on_5090.py` / `autolabel_yolo.py` | 远端执行 / 日志采集 / 数据集 / 训练 |

> ⚠️ **dzuav 从未在真机跑过**（`docs/接手说明.md §2.6`）：Orin 实机编译、格号识别率、PnP 稳定性、帧率**全部未测**。

### 2.6 tools/perception —— 俯瞰场地感知管线（纯 Python）

> 目标：`dzsmartscout/tools/perception/`。版本 1.2.0（t5a 片级覆盖/负证据）。
> 依赖 **numpy + Pillow**（`cv2` 仅可选）；`rclpy` 只在真发 DDS 时用。

**模块职责**（`__init__.py:6-16`）：

| 模块 | 作用 | 主要产出 |
|---|---|---|
| `colors` | 颜色分类（HSV/RGB 规则 + 调色板）、形态学、连通域 | cls→掩膜、blob 统计 |
| `grid` | 四角点→单应→任意像素→格名 A1..N11 | `ArenaRegistration` |
| `objects` | 雷/障/靶区（+图纸"通"格）→ 格名 | 格名列表 + 置信度 + 拒收理由 |
| `car_pose` | 车顶标记→block+yaw（透视校正） | `CarPose` |
| `bridge_arrow` | 桥顶箭头→方向→半场通行格 | `BridgeCall` |
| `protocol` | 车侧契约 JSON（`drone.py` 口径） | 4 种报文 + 校验 + 车侧解析镜像 + 覆盖/负证据 |
| `coverage` | 片（slice）与覆盖清单（t5a） | 7 片航点、`coverage{complete,cells}` |
| `field_map` | 出图：感知输出→与 `config/map.yaml` 同 schema | `FieldMap`（JSON/YAML + 对账 + 置信度） |
| `pipeline` | 总装（一帧图→一组报文） | `PerceptionResult` |

#### 2.6.1 坐标语义（与车侧逐字一致）

真源：`dzrobot_v2/src/dzbrain/dzbrain/core/geometry.py:6-16,69-81`（`README.md:36-53`）：

```text
x = (10.5 - row_idx) × 0.35      row 11 → 0.175, row 1 → 3.675
y = (13.5 - col_idx) × 0.35      col N  → 0.175, col A → 4.725
```

| 概念 | 值 |
|---|---|
| 场地 | 14 列 A~N × 11 行 1~11，格边 0.35 m ⇒ 4.90 m × 3.85 m |
| 原点 | **N11 格外角**（不是格心） |
| +x | 行号**减小**方向（下半场→上半场） |
| +y | 列号**减小**方向（N→A） |
| 半场 | `x > 6×0.35 = 2.10` = upper（行 1~6）；否则 lower（行 7~11） |
| 四角点 | `N11=(0,0)`、`N1=(3.85,0)`、`A1=(3.85,4.90)`、`A11=(0,4.90)` |
| yaw | `atan2(dy,dx)` 度：0=朝 +x；+90=朝 +y |

⚠️ **手性**：场地米制系右手系（z 朝上），图像数组 `(u 右, v 下)` 左手系 ⇒ 正常俯视像素有向面积必为负。`register()` 默认直接报错，`expect_flip=False` 才放行。

⚠️ 车侧 `geometry.py:39` 注释与 `:72` 公式矛盾（行号方向），**以公式为准**（本工具只记录不改车侧）。

#### 2.6.2 grid.py —— 网格注册

- `register_from_corners` / `register_from_named_corners`：DLT 最小二乘（Hartley 归一化），**支持 4 点以上**。
- `reg.px_to_hit(u,v) → CellHit(block, row_idx, col_idx, fx, fy, in_arena, half, offset_from_center_m)`。
- `reg.self_check()`：154 格逐格"格心→像素→米→格名"闭合自检；`diagnostics()` 给残差与条件数。
- 官方图纸标定链：`sx=45.5857 px`、`sy=45.5073 px`，四角点解析拟合残差 **0.000 px**。

#### 2.6.3 colors.py + objects.py —— 分类与落格

- 调色板是**数据**：`Palette` 可 `to_json/from_json`。两个内置：`MAP_IMAGE`（图纸）、`FIELD_DEFAULT`（实物，仅 `deck` 与 `arrow` 有实测证据，其余 `provisional=True` 默认关闭）。
- ⚠️ **实物颜色只有两条有证据**；水桶颜色不唯一（红/青/绿三种），单色阈值不可靠。
- 几何判据两套口径（图纸方块 vs 实物圆片/桶），**12 cm 圆片在 35 cm 格里只占 9.2%**。
- **"深色像素占比"判据是实测出来的**（阈值 0.02，60 倍余量）。
- 同格冲突按 `priority`（障 > 雷 > 靶）消解，每条拒收留 `reason`。

#### 2.6.4 car_pose.py —— 车位姿

- 输入是"车顶贴一块纯色板"的**己方假设**（规则全文无车顶标记）→ 颜色必须现场标定 `CarSpec`。
- 像素先经单应映到场地米制系再算主轴 ⇒ yaw 是**透视校正后**的。
- **单块只能定到 mod 180°**（`yaw_ambiguous=True`，置信度减半）；再贴车头小标记才能定向。
- 合成实测（150 px/m）：方向误差 max **2.07°** / mean **0.91°**；格名 50 组配置 **100% 正确**。

#### 2.6.5 bridge_arrow.py —— 桥顶箭头

- 现场形态：黄色桥面板上一张白纸+绿箭头，箭头尖头指向穿洞方向。
- 两种判据：**H1 位置**（默认，箭头在哪块板上 ⇒ 该 K 列格是该半场通行格）、**H2 方向**（`infer_mode="direction"`）。
- 方向估计**不用 PCA**（实物箭头头宽 80px、整箭才 58px 高，PCA 主轴横向差 87°）；改用**扫描候选轴 + 结构打分**（一端等宽低平台=箭杆，另一端单调张开=头部）。
- 合成实心箭头 14 个角度：误差 **≤1.0°**；实物照片（`p09_1.png`）：估计 **−96°** vs 真值 −90°（误差 6°）。

#### 2.6.6 protocol.py —— 车侧契约

**四种报文**（消费端 `dzrobot_v2/src/dzbrain/dzbrain/tasks/drone.py`）：

| 报文 | 车侧要求的字段 | 车侧行为 |
|---|---|---|
| `heartbeat` | `type` | 刷新心跳；>3s 无心跳 → `online=False` |
| `bridge_passable` | `half`∈{lower,upper}、`block` 非空、`confidence` | 写 `P_DRONE`，`bridge_of(half)`；>30s 过期返回 None |
| `target_zone` | `blocks`（非空 list） | 与本地 `map.yaml.targets` 做差集 → `field_diff` |
| `field_verify` | `mines`/`obstacles`（list） | 同上（雷/障差异） |
| 任意带 `seq` | — | 回 `{"type":"ack","of":<seq>}` 到 `/dzlink/car_to_drone` |

**四个坑**（`protocol.py:17-31`）：
1. `half` 被 `.strip().lower()` 校验，`block` 被 `.strip().upper()`；**不校验格名是否在地图内**。
2. **载荷里的 `ts` 车侧不使用**，过期从**车收到**算起 → 周期重发同一结论是有效续期。
3. `target_zone.blocks` 空列表 = 车侧记一次 `missing_fields` = 白发一条。
4. 未知 `type` 只记日志不崩 → 新增字段安全、新增类型不安全。

**覆盖/负证据（t5a）**：`build_field_verify` 新增 `coverage`、`slice`、`scan_id` 三个 additive 键；`scan_id` 是幂等键（BEST_EFFORT 必须周期重发同一片）；空片（扫过且为空）必须走 `mines=[] + coverage{complete:true}`。

**话题与 QoS**：`/dzlink/drone_to_car` / `std_msgs/String` / **depth 10 · BEST_EFFORT**（`contract.py:40-41,94-96`）。

#### 2.6.7 能力盘点（`CAPABILITY_L1.md`）

| 能力 | 状态 | 证据 |
|---|---|---|
| 官方贴图→出图（雷/障/靶/桥） | ✅ 逐格一致 0 差异 | mine 8/8、obstacle 8/8、target 2/2、bridge 2/2；P=R=1.000 |
| 与 `map.yaml` 同 schema 的 JSON/YAML | ✅ | `field_map_official.{json,yaml}` |
| 每格置信度 | ✅ | mine 0.80~0.84、obstacle 0.66~0.69、target 0.66~0.67、bridge 0.74~0.75 |
| 车位姿 | ✅（合成确定性验证） | `perception.car_pose` |
| 桥顶箭头→通行格 | ✅ | 实拍 −96° vs 真值 −90° |
| **蓝面朝向** | ⚠️ 无人机俯视判不了（几何原因） | 真图如实返回 `unknown` |
| **片级覆盖/负证据** | ✅ 报文结构+合成场景；⚠️ 现场视场未标定 | `tests/test_coverage.py`（14 项） |
| **实拍相机帧上检出** | ❌ 无相机可用 | cam_probe 5 帧拉普拉斯方差 1.37~49.1（全糊） |
| 起终点/无人机点**语义** | ⚠️ 半自动 | 框内印字未 OCR，语义取自参考真值 |

#### 2.6.8 入口与测试

```bash
cd dzsmartscout/tools
python -m perception.cli selftest            # 66 项（不含最慢的 e2e）
python -m perception.cli selftest --with-e2e # 72 项
python -m perception.run_real_checks         # 真图核查
python -m perception.cli detect --image 图.png --corners "..."
```

`run_tests.py`：`cd dzsmartscout/tools && python -m perception.run_tests`（99 项）。

### 2.7 tools/link —— 空地链路常驻节点与发送器

#### 2.7.1 dzlink_supervisor.py（2014 行）—— 无人机侧空地链路常驻节点

> 一架无人机只要挂着**这一个进程**，就完成了「感知 → 下发 → 闭环」三件事。
> 单文件，纯 stdlib + rclpy，**不需要编译**；部署到 `~/dzlink_tools/dzlink_supervisor.py`。

**四项职责**（`dzlink_supervisor.py:5-11`）：

| # | 职责 | 话题 | QoS |
|---|---|---|---|
| ① | 1 Hz 心跳 | `/dzlink/drone_to_car` | BEST_EFFORT depth10 |
| ② | 感知结果转发（bridge/target/field） | `/dzlink/drone_to_car` | BEST_EFFORT depth10 |
| ③ | 订阅车回传 car_state / ack | `/dzlink/car_to_drone` | BEST_EFFORT |
| ④ | 路线/复位下发 | `/nav/waypoint` | RELIABLE depth10 |

**QoS 逐条对着车侧订阅端设死**（读车侧源码核实，不是猜的）：

| 话题 | 类型 | QoS | 车侧订阅者 |
|---|---|---|---|
| `/nav/waypoint` | `std_msgs/String` | **RELIABLE** / VOLATILE / depth10 | `path_executor_node`、`serial_reader_node` |
| `/dzlink/drone_to_car` | `std_msgs/String`(JSON) | **BEST_EFFORT** / VOLATILE / depth10 | dzbrain `DroneTask` |
| `/dzlink/car_to_drone` | `std_msgs/String`(JSON) | BEST_EFFORT | 车发 `car_state`/`ack` |

> QoS 不匹配的表现是「完全收不到**且不报错**」，所以这块不允许自由发挥（`:17-21`）。

**域号唯一真相源**：import 阶段读 `~/dzlink_tools/dzlink.env`（与车侧 `~/dzrobot/dzros.env` 逐字节对应的 **118**）。必须在 `rclpy.init()` **之前**生效，所以放在模块导入期做（`:22-23`，`bootstrap_domain` `:155-213`）。

**车侧契约**（`CAR_CONTRACT`，`:116-124`，2026-10-12 t16 按车侧真值对齐）：

| 参数 | 值 | 含义 |
|---|---|---|
| `info_stale_s` | 30.0 | 情报超过 30s 车侧当 None |
| `mission_wait_s` | **2.0** | Mission 最多等 2s，等不到用默认穿洞格 |
| `auto_start_delay_s` | **45.0** | 起栈 45s 后车自动出发 |
| `car_state_period_s` | 0.7 | 车侧 car_state 上报周期 |
| `heartbeat_offline_s` | 3.0 | 3s 没心跳车侧判无人机离线 |
| `bridge_default_lower` / `_upper` | `K10` / `K5` | 车侧兜底穿洞格 |

> ★★ **等情报窗口只有 2 s**，且只覆盖「两半场相隔 ≤2s 到达」。超过 2s 才到的那个半场只进遥测，路线已冻结。
> ⇒ **两半场必须成对发出**。本节点已保证：任一新鲜桥洞半场到点，就把所有新鲜桥洞半场一起发（`tick()` 的 `bridge_round_due`，`:1173-1193`）。
> ⇒ 情报**必须在车起栈前备好**（`--cmd-file` 预置 / `once`）。

**架构：纯逻辑层与 rclpy 层分离**（`:13-16`）：

| 层 | 类/函数 | rclpy 依赖 |
|---|---|---|
| 决策核心（纯函数） | `SupervisorCore`、`decide()`、`canonical_text()` | 无（Windows 本机可跑 `--dry-run`/单测） |
| DDS 层 | `SupervisorNode`（唯一 import rclpy 处） | 有 |
| 输入层 | `FileTailer`/`GlobTailer`（文件尾随）、`UdpJsonSource`（UDP JSON）、ROS 话题 | 部分 |
| 命令文件 | `CommandFile`（JSONL 追加即生效） | 无 |
| 日志 | `JsonlLogger`（JSONL，每次 flush） | 无 |

**核心类 `SupervisorCore`**（`:976-1223`）：

- `ingest_sample()`（`:1040-1080`）：`decide()` 翻译 → 落 `records` 台账 → 内容变化时立即重发（不等周期）。
- `tick()`（`:1131-1194`）决策顺序：**⑤ 过期撤回 → ④ 路线优先 → ① 心跳 → ② 情报（按槽位、固定顺序 bridge→target→field）**。
- `_next_intel[slot]` 记录下次重发时刻；`period_for()`（`:969-973`）返回各类型周期（bridge 默认 2s）。
- `status()`（`:1197-1220`）汇总 stats / intel / ack_ledger / pending_routes。

**`decide()` 纯函数**（`:327-392`）：一份感知样本 → `[(event_type, canonical_payload)]`。只做翻译，不做"该不该发"。认不出来返回空列表（绝不猜）。支持外层包裹 `payload`/`data`/`intel`/`result`/`msg`。

**桥洞两半场配对**（`:1165-1193`）：`fresh_bridge` = 所有 age ≤ ttl 的 bridge 槽；`bridge_round_due` = 任一槽到期；到期时**把所有新鲜桥洞半场一起发**。

**路线校验**（`validate_route()` `:563-600`、`_route_token_problems()` `:603-627`、`route_blocks()` `:630-641`）：

- 语法：裸 token 列表 / `多点`/`multi`/`route` 前缀 / `单点`/`single`/`goto` / `reset:N10,y` / `load:文件` / `G10*`（停车打靶）/ `R:F11`（倒车）/ `SHOOT@G11` / `:SCAN_STOP` / `N5,y`（朝向后缀只在最后一个 token）。
- **10/11 列（`K10`/`K11`）曾被本地校验误判非法，已修为回归测试**（`test_grid_columns_10_and_11_are_accepted`）。

**感知输入三选一（可叠加）**（`README.md:162-171`）：

| 方式 | 参数 |
|---|---|
| 文件尾随 | `--perception-glob` '.../out/*.jsonl' 或 `--perception-file` x.jsonl |
| ROS 话题 | `--perception-topic` `/dzlink/perception` |
| UDP JSON | `--perception-udp` 39042 |

默认从尾读；要把历史行也补上加 `--perception-from-start`。命令文件同理。

**接受的 JSON 形态**（`README.md:176-187`）：`bridge_passable` / `bridge`（方向词自动映射）/ `target_zone` / `field_verify` / `mines` / `obstacles` / `heartbeat` / 外层包裹。

**★ 两条必须知道的边界**（`README.md:189-201`）：
1. **不解析 dzuav 的私有 UDP 二进制帧**（端口 50000，`ef ef ef ef 10` + u64 ts + 25B/目标 + CRC16 + `fe fe fe fe`）——帧格式属于感知层，本节点**只吃 JSON**。
2. **`direction`→`half` 是"约定假设"，不是源码事实**：默认 `left→lower`、`right→upper`。写错的表现是**车穿错洞而且不报错**。每次用到方向词都会打印映射表；可用 `--bridge-left-halves` 反转；最稳做法是感知侧直接写 canonical `half`。
3. 桥面是 z=0.35 的**第二个平面**，地面格号单应对桥顶不适用；换算**由感知侧负责**，本节点只把 `{half,block}` 原样下发。

**运行模式**（`README.md:99-158`）：

| 模式 | 用途 |
|---|---|
| `run` | 常驻：1Hz 心跳 + 尾随感知 + 监听车回传 + 吃命令文件（比赛主用） |
| `once` | 只跑一次，把当前情报发一遍（默认 3 遍） |
| `route` / `reset` | 下发路线 / 复位 |
| `contract` | 打印契约一页纸（含当前域号） |
| `selftest` | 纯逻辑自检（不需要 ROS） |
| `dry-run` | 只本地拼报文，不发 |
| `listen` | 只监听车回传 |

**退出码**：`0` 正常 / `2` 没有 ROS 环境 / `3` 发出去了但没有任何车侧订阅者 / `1` 参数错。

**日志 kind**（`README.md:282-295`）：`intel_in`/`intel_emit`/`intel_stale` / `route_queued`/`route_emit` / `car_ack`/`car_state`/`car_other` / `sample_rejected`/`sample_bad_line` / `cmd_in`/`cmd_rejected` / `subs` / `status`/`final_status`。

#### 2.7.2 dz_send_waypoint.py（452 行）—— 一次性发送器

> 单次发送路线/情报/心跳；退出码 0 正常 / 2 无 ROS 环境 / 3 发出去了但无订阅者。

**子命令**：`route` / `bridge` / `heartbeat` / `target` / `field` / `listen` / `contract`。

**格名拆分**（`split_blocks()` `:101-117`，t12/D4 修复点）：分隔符 `[空格 , ; | 、]`，去空 → 大写 → 按序去重。
- 修复前：`--mines A3,B4` 会发出**畸形格名** `"A3,B4"`（车侧 `_norm_blocks` 不拆逗号）且**全程不报错**。
- `target` 新增 `--blocks` 别名（修复前 `target --blocks G10,H10` 会 argparse 报 unrecognized arguments → exit 2）。
- `bridge <half>=` 只允许一个格名（`lower=K9,K4` 明确报错）。

**t18/F2 文案修正**：报错提示改为**本发送器可执行**的写法（`bridge lower=K9 upper=K4` 位置参数并列，或分两次调用），并注明 `--bridge` 是 `dzlink_supervisor.py` 的参数、本发送器不认。

**报文构造**（纯函数，可 `--dry-run`）：

| 函数 | 产物 |
|---|---|
| `route_payload(text)` | 裸文本路线 |
| `bridge_payload(half, block, confidence, seq)` | `{"type":"bridge_passable",...}` |
| `heartbeat_payload(seq)` | `{"type":"heartbeat",...}` |
| `blocks_payload(kind, blocks, seq)` | `{"type":"target_zone",...}` |
| `field_payload(mines, obstacles, seq)` | `{"type":"field_verify",...}` |

**发送**（`send()` `:189-234`）：`make_qos(reliability)`（`:80-87`）设 depth10；先等 DDS 匹配（`wait_match_s` 默认 3s）→ 再给 1.5s 让计数收敛 → 打印订阅者匹配数 → 按 repeat 发。匹配数 0 时仍按 repeat 发（供判断）。

**监听**（`listen(duration)` `:237-268`）：订阅 `/dzlink/car_to_drone`（BEST_EFFORT），统计按 type 的计数。

**热点话题与 QoS**：`TOPIC_ROUTE=/nav/waypoint`（reliable）、`TOPIC_TO_CAR=/dzlink/drone_to_car`（best_effort）、`TOPIC_FROM_CAR=/dzlink/car_to_drone`（best_effort）（`:66-68`）。

#### 2.7.3 部署与验证

- **部署**：`bash dzsmartscout/tools/link/deploy_supervisor.sh`（上传+md5核对+远端自检+真机冒烟；`--no-smoke`/`--dry-run`）。默认目标 `duzhong@10.29.122.104:~/dzlink_tools/`（`:17`）。
- **L1 本机纯逻辑**：`python -m unittest discover -s dzsmartscout/tools/link/tests -p "test_*.py"`（70 项）+ `selftest`（`README.md:207-214`）。
- **L2 真机 ROS**：`bash ~/dzlink_tools/smoke_ros.sh`（不依赖小车，用本机临时订阅者冒充小车）。
- **L3 端到端**：`once --bridge lower=K9 upper=K4` + 车侧 `ROS_DOMAIN_ID=118 ros2 topic info /nav/waypoint -v`。

**反面判据（最先排的三种）**：匹配数 0 / 车没起栈 / 组播被 AP 拦。域 d 的发现端口 = `7400 + 250·d`（域 118 → 36900）。

#### 2.7.4 关联节点（tools/drone/）

| 脚本 | 作用 |
|---|---|
| `dzlink_livox_bridge.py`（172 行） | `/livox/lidar`(CustomMsg) → `/dzlink/points`(PointCloud2) + `/dzlink/seq`(Int64 序号信标)；RELIABLE/VOLATILE/depth10 |
| `dzlink_seq_beacon.py`（74 行） | 50Hz 单调递增 Int64 信标（跨机丢包精确对账） |
| `dzlink_seq_check.py`（110 行） | 对端丢包对账：`span = max-min+1`、`丢包率 = 1 - recv/span` |
| `dzlink_points_check.py`（100 行） | 点云消费自检（字段/首点/非零点比例） |
| `probe_lidar.py` / `pc2_check.py` / `xtest_type.py` / `lidar_char.py` | 雷达/点云诊断 |
| `cam_ros_snap.py` / `cam_ros_view.py` / `cam_sharpness.py` / `cam_probe_*.py` | 相机诊断与抓帧 |

**点云桥接字段**（`dzlink_livox_bridge.py:43-49`）：`x,y,z,intensity` 各 float32，point_step=16；`is_dense=False`（MID-360 有无效点）。

#### 2.7.5 域号问题（★ 全车唯一真相源）

| 机器 | 文件 | 内容 |
|---|---|---|
| 无人机 | `~/dzlink_tools/dzlink.env`（仓库内 `tools/link/dzlink.env`） | `export ROS_DOMAIN_ID=118`、`ROS_LOCALHOST_ONLY=0` |
| 小车 | `~/dzrobot/dzros.env`（仓库内 `tools/link/dzros.env`） | `DZ_ROS_DOMAIN=118`，`export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-$DZ_ROS_DOMAIN}` |

**为什么必须钉进脚本**（`dzlink.env:6-13`）：交互式终端起栈会读 `~/.bashrc` → 118；而 systemd/cron/`ssh host 'cmd'` **不读** → 默认 0。两种起法域号不同 ⇒ **ping 通、SSH 通、网页台 :8080 通，ROS 死活看不见对方，且一个错都不报**。

**命令行速查**：域 d 的组播发现端口 = `7400 + 250·d`；用户端口 = `7401 + 250·d`。域 118 → `239.255.0.1:36900`。在 7400 上嗅探**永远**看不到域 118 的对端。

**风险**：空地两机**共用同一 ROS 2 域**。`/odom`、`/imu_data`、`/cmd_vel` 这类通用话题名在两机上都可能出现，跨机可见会造成**误订阅/串扰**（`notes/camera.md:89-95`）。

### 2.8 fc-audit —— 补丁区、启动手册与专题笔记

> 目标：`dzsmartscout/fc-audit/`。工作流铁律（`WORKFLOW.md:1-14`）：**本地改 → 用户手动替换+编译**；助手对无人机 SSH **只读**。每个补丁目录必须有 `files/`（完整文件）+ `APPLY.md`（路径/编译/验证/回退）。

#### 2.8.1 启动手册（`启动手册.md`，240 行）—— 原始 1.txt 的实测校正版

**每个新终端**：`source /opt/ros/humble/setup.bash` + `source ~/dzpx4/install/setup.bash`（少一句就 `Package not found`）。

**四阶段**：

| 阶段 | 内容 | 状态 |
|---|---|---|
| A 感知层 | `ros2 launch livox_ros_driver2 msg_MID360_launch.py`、`ros2 launch fast_lio_localization velodyne_localization.launch.py rviz:=false` | ✅ 可跑 |
| B 飞控链路 | `MicroXRCEAgent serial --dev /dev/ttyACM0 -b 921600`、`px4_odom_convert mode:=odom`、`px4_imu_bridge` | ❌ 需飞控板 |
| C 验证 | `ros2 topic list | grep fmu`、`ros2 topic hz /fmu/in/vehicle_visual_odometry` | ❌ 依赖 B1 |
| D 控制层 | `px4_control`、`px4_teleop_node`（真 TTY）、`px4_path` | ⚠️ 有执行能力 |

**与原始 `1.txt` 的三处差异**（`启动手册.md:168-193`）：

| # | 差异 | 原因 |
|---|---|---|
| 1 | B2 必须加 `mode:=odom` | launch 默认 `localization` 订阅不存在的话题 |
| 2 | `ros2 launch px4_path` 跑哪个节点由 **YAML** 决定 | `record_mode` 无同名 launch 参数，改 YAML 要重装 |
| 3 | 阶段 C 的 5 条 echo 在飞控板没插时**全部卡住** | 无 `/fmu/out/*`，不是软件故障 |

**停止**：`pkill -f`（不是 `-x`，名字超过 15 字符）。`path_record_node` **析构时才落盘**，`kill -9` 会丢整段录制。

#### 2.8.2 补丁清单（`fixes/README.md`）

| # | 目录 | 改什么 | 要不要编译 | 风险 | 状态 |
|---|---|---|---|---|---|
| 01 | 01-odom-source | `px4_odom_convert` 定位来源 `/localization` → `/Odometry` | 要（或零编译 `mode:=odom`） | 极低 | 已产出，**未应用** |
| 02 | 02-px4-path-config | 路径文件契约修复（录制写==跟随读）+ 补丁 03 配套参数 | 要 | 低 | 已产出，**未应用** |
| 03 | 03-follow-safety | 显式开始闸门 + 接驳段长度上限 + HOLD 空轨迹防御 | **要**（C++） | 中 | 已产出，**未应用** |
| 04 | 04-camera-bringup | USB/UVC 相机点亮 + 出图验证（新增工具包） | **不要** | 极低 | ✅ **已真机验证通过** |
| 05 | 05-drone-to-car-waypoint | 无人机→小车航点/情报发送器（小车零改动） | **不要** | 极低 | ✅ **已真机验证：车侧收 2** |
| 06 | 06-car-wait-for-drone | **车侧**：起栈后原地待命等无人机指令 | 不要（须同步 install 副本） | 低 | ✅ **已应用并实测** |

**补丁 01 要点**（`01-odom-source/APPLY.md`）：
- 根因：`transform_fusion.py` 是 `/localization` 唯一发布者，`import tf_transformations` 失败 → 进程启动即死 → `/localization` 从未发布。
- 代码默认 `odom`，是 launch 第 35 行覆盖成 `localization`。
- 数值等价：`transform_fusion.py:54` 在没收到 `/map_to_odom` 时用 `np.eye(4)`。
- **顺带记录（本补丁未改）**：launch 第 18–19 行传了 `max_position_jump_m 0.5` / `max_attitude_jump_rad 0.5`，但节点从未声明 → **跳变保护根本不存在**。

**补丁 02 要点**（`02-px4-path-config/APPLY.md`）：
- 原值：`path_follow_node.path_file=recorded_path_2.txt`（:25）vs `path_record_node.output_file=recorded_path.txt`（:119）；`overwrite_existing=false`（:123）。
- 叠加行为：`overwrite_existing=false` → 每次录制自动改名 `_1/_2/_3…` → 下次写 `recorded_path_3.txt`，跟随仍读 `_2` → **录新路径飞旧路径，且无任何报错**。
- 新值：`path_file=recorded_path.txt`、`overwrite_existing=true`；另含补丁 03 的 `require_fresh_enable: true`、`max_join_length_m: 0.0`。

**补丁 03 要点**（`03-follow-safety/APPLY.md`）：
- **显式开始闸门**：`/px4_control/goto_enabled` 是 transient_local 锁存。新增 `require_fresh_enable`（默认 true）——第一帧只当基线，只有观测到一次真正的 `false → true` 跳变才允许开飞。操作手势：先按 `5` 再按 `4`。
- **接驳段长度上限**：`max_join_length_m`（默认 0=关闭，建议首飞前设 5~10 m）。
- **HOLD 空容器防御** + **日志文案纠错**（"press 4 after creating a valid path" → "this node reads the path file only once at startup, so fix the file and restart the node"）。
- **没修的**：R-02 追赶速度不受 `max_speed_mps` 约束（需台架/系留实测）。

**补丁 05 要点**（`05-drone-to-car-waypoint/APPLY.md`）：
- 结论：**不用写桥接，链路早就通了**（2026-10-06 两端 DDS 直连 LINK_OK）。缺的是话题名 + QoS + 报文格式对齐。
- **★ 更正**：车上跑的是**旧核** `path_executor`，**根本没有 dzbrain**。A 通道旧核就吃；B 通道当前部署下**没人吃**（旧核全文 `grep -i dzlink` = 0 命中）→ 想让无人机影响穿洞选择，**只能走 A 通道：把选定的洞直接写进路线 token**。
- 时序口径已更新（2026-10-12 t16）：`bridge_info_wait_s` 20→**2.0 s**、`auto_start_delay_s` 10→**45.0 s**、默认穿洞格 K9/K4 → **K10(下)/K5(上)**。
- **废弃说明**（`DEPRECATED_dz_send_waypoint.md`）：本目录 `files/dz_send_waypoint.py` 已废弃，唯一真源是 `tools/link/dz_send_waypoint.py`（md5 `EB41DBF4B944…`）；旧副本 md5 `B3AA2ED999CD…` 已分叉。

**补丁 06 要点**（`06-car-wait-for-drone/APPLY.md`）：
- 根因：`path_executor.py:849` `if AUTO_START: create_timer(AUTO_START_DELAY, _auto_start_mission)`，起栈后自动把内置路线当外部指令下发。
- 改法：`auto_start: false` + `DZ_AUTO_START` 环境变量覆盖（`1/true/yes/on` 强制开，`0/false/no/off` 强制关）+ 启动横幅。
- **★ 关键发现**：`ros2 run` 跑的是 **install 副本**不是 `src/`（纯 Python 包 colcon build 就是逐字节拷贝）→ 只改 `src/` 行为零变化。

#### 2.8.3 专题笔记

| 笔记 | 行数 | 核心结论 |
|---|---|---|
| `notes/camera.md` | 108 | 2026-10-07 只读实测：机上**没有相机硬件**、软件栈已齐、`gst_camera` 空壳。⚠️ 已被补丁 04 之后的状态取代（相机已接但糊） |
| `notes/fastlio.md` | 464 | FAST-LIO 定位链路：实际生效 config=`mid360_test.yaml`；`/initialpose` VOLATILE 首发竞态不可自愈；ICP 失败静默；输出帧非 ENU、偏航被下游写死 180° |
| `notes/px4_path.md` | 602 | px4_path 全量审计：R-01~R-18 风险清单、编译真相、坐标一致性 |

#### 2.8.4 uXRCE-DDS 与 921600

- 机载电脑**没有 PX4 固件**。PX4 跑在单独飞控板，通过 USB 串口 + `MicroXRCEAgent`（uXRCE-DDS）把 PX4 uORB 映射成 ROS 2 的 `/fmu/in/*` 与 `/fmu/out/*`（`无人机技术文档.md:103-107`）。
- 启动：`MicroXRCEAgent serial --dev /dev/ttyACM0 -b 921600`（**921600** 波特率）。
- **飞控板不插 ⇒ 没有 `/fmu/*` ⇒ 所有 Offboard 控制无法进行**（2026-06 至 10-08 从未插过）。
- `px4_msgs 2.0.1` ≈ PX4 v1.15+，与固件强耦合（`px4_msgs/package.xml:5`）。
- 前置检查：`ls -l /dev/ttyACM*`、`lsusb | grep -iE "3dr|px4|pixhawk|hex"`。

#### 2.8.5 verify_frames.py —— 坐标链路数值验证

- 复刻 `px4_odom_convert.cpp:121-179` + `frame_transforms.h:173,180,316-349` + `frame_transforms.cpp:157-198`（`verify_frames.py:4-8`）。
- 结论（`notes/px4_path.md §6.2`）：位置与姿态共用同一个 `R_total = R_NED_ENU · R_ENU_SLAM`；`NED_ENU_Q` 旋转矩阵恰为 `[[0,1,0],[1,0,0],[0,0,-1]]`（det=+1 正当旋转）；`path_loader` 的 yaw 转换对任意姿态精确（非小角度近似）。

#### 2.8.6 场地规则与雷达可见性（`场地规则解读与雷达应用.md`）

| 元素 | 物理形态 | 3D 雷达 | 视觉相机 |
|---|---|---|---|
| 地雷（8） | 直径 12cm 彩色图片平贴地面 | ❌ | ✅ 唯一手段（规则明写） |
| 彩桶（8） | 装满水彩色带盖桶 | ✅ 强项 | ✅ |
| 桥洞结构 | 35cm 挡板架空 | ✅ 可测净空 | ❌ 看不到顶面 |
| **桥顶箭头** | 贴桥面顶部绿色箭头贴纸 | ❌ | ✅ **必须无人机** |
| 黄色隔离挡板 | 立式挡板墙 | ✅ | ✅ |
| 白色胶条 | 平贴地面 mm 级 | ❌（噪声量级） | ✅ |
| 起终点黄块 | 地面印刷 | ❌ | ✅ |
| 场地边界 | 刀刮布边缘，无围挡 | ⚠️ 勉强 | ✅ |

> **一句话：雷达看不见"图案"，但看得见"结构"。** 目标识别 100% 是视觉的活；几何测量才是雷达的活。

**两个必须实飞前解决的风险**：
- **风险 A · 白色无特征地垫 + 无围挡 → FAST-LIO 漂移**：`blind: 0.8` 把 0.8m 内点全丢，低空视野里大部分是近场白地垫。
- **风险 B · 18% 的 (0,0,0) 无回波点**：怀疑机身/桨叶遮挡近场。

**不需飞控板就能做的三个实验**：桥洞通道分辨 / 彩桶点云特征 / **白色地垫漂移**（实验 3 最容易被忽略、也最可能致命）。

### 2.9 launch / config / 脚本

#### 2.9.1 launch 文件

| 文件 | 行数 | 要点 |
|---|---|---|
| `px4_control/launch/px4_bringup.launch.py` | 41 | 起 `px4_control_node`；`land_control` 默认 false |
| `px4_control/launch/px4_control.launch.py` | 42 | 包装 bringup |
| `px4_control/launch/px4_teleop.launch.py` | 35 | 起 `px4_teleop_node`；**必须在真 TTY** |
| `px4_path/launch/px4_path.launch.py` | 55 | 读 YAML `record_mode` 决定跑 `path_record_node` 或 `path_follow_node` |
| `px4_odom_convert/launch/px4_odom_convert.launch.py` | 58 | `mode` 默认 `localization`（补丁 01 改 `odom`）；传 `max_position_jump_m`/`max_attitude_jump_rad`（幽灵参数） |
| `px4_imu_bridge/launch/sensor_combined_to_imu.launch.py` | 54 | 6 个 launch 参数 |
| `livox_ros_driver2/launch_ROS2/msg_MID360_launch.py` | 54 | `xfer_format=1`、`publish_freq=10.0`、`frame_id=livox_frame`、`cmdline_bd_code=47MCN850037605` |
| `FAST_LIO_LOCALIZATION_ROS2/launch/velodyne_localization.launch.py` | 106 | 并行拉 4 节点 + rviz；`config_file` 默认 `mid360_test.yaml` |
| `FAST_LIO_LOCALIZATION_ROS2/launch/mapping.launch.py` | — | 只跑 `fastlio_mapping` + rviz |
| `uav_camera_code/tools/orin_*.sh` | — | 编译/运行 dzuav |

#### 2.9.2 config 文件

| 文件 | 行数 | 要点 |
|---|---|---|
| `px4_control/config/params.yaml` | 23 | `takeoff_height 0.5`、`loop_rate_hz 20`、`pursuit_goto_timeout 1.0` |
| `px4_path/config/params.yaml` | 159 | 录制/跟随/轨迹共用一份（见 §2.2.8） |
| `px4_odom_convert/launch/px4_odom_convert.launch.py` 内联参数 | — | `loop_rate_hz 50.0`、`max_position_jump_m 0.5`（幽灵）、`max_attitude_jump_rad 0.5`（幽灵） |
| `FAST_LIO_LOCALIZATION_ROS2/config/mid360_test.yaml` | — | **实际生效**；含 `/global_localization`、`/transform_fusion`、`/global_map_publisher` + `/**` |
| `FAST_LIO_LOCALIZATION_ROS2/config/mid360.yaml` | — | 纯 FAST-LIO；`map_file_path` 指向**不存在**的 `buliding_a_floor1.pcd`（拼写错误） |
| `FAST_LIO_LOCALIZATION_ROS2/config/{avia,horizon,ouster64,velodyne,velodyne_test}.yaml` | — | 其他雷达型号 |
| `livox_ros_driver2/config/MID360_config.json` | — | `lidar_type: 8`（MID360 专属）、雷达 IP `192.168.3.105`、主机 `192.168.3.189`（eno1） |

> ⚠️ **FAST-LIO 参数双份配置**：`mid360_test.yaml:16`（精确节点名）与 `:29`（`/**` 通配）都写 `map_file_path`；ROS 2 对优先级**没有可依赖的规范** → 隐患（`notes/fastlio.md §7-R5`）。

#### 2.9.3 drone-scripts 与根级脚本

| 脚本 | 作用 |
|---|---|
| `drone-scripts/start-lidar.sh` | 检查 `eno1 carrier` → 拉网口 → ping 雷达 192.168.3.105 → 启动 `livox_ros_driver2`（前台，日志 `/tmp/livox_launch.log`） |
| `drone-scripts/start-lidar-on-link.sh` | 轮询 carrier 最多 30 分钟，链路恢复后后台启动驱动 |
| `find_drone.py` | /24 扫描 :22 + mDNS `duzhong.local` |
| `setup_drone_access.py` | 无人机统一接入：补 nsswitch mdns 模块 + 安装本机 SSH 公钥（duzhong + root） |
| `install_drone_key.py` / `test_drone_users.py` / `ssh_connect.py` | SSH 免密 / 用户探测 |
| `enable_mdns.py` / `fix_drone_avahi.py` / `remote_fix_drone_avahi.sh` / `fix_mdns_nsswitch.py` | Avahi/mDNS 配置 |
| `rename_drone_hostname.py` | 主机名改 `dzsmart` |
| `probe_drone_dirs.py` / `probe_drone_vnc.py` / `crack_vnc_passwd.py` | 环境探测 |
| `sweep_current.py` / `verify_new_ip.py` | 定位无人机 IP |
| `adapt_drone_plugin.mjs` / `vnc_handshake.ps1` | 插件适配 / VNC 握手 |
| `_pdf_log.txt` / `ar5_p7_8.txt` / `qsg_p4_6.txt` | 厂商手册文本证据 |
| `WORKFLOW.md` | 工作约定真源（本地改→用户替换编译；SSH 只读） |

> ⚠️ 根级脚本属"网络接入"阶段产物。**飞控代码类改动一律走 `fc-audit/fixes/` 流程**，不再用这些脚本推送。

### 2.10 缺陷清单（现象 → 触发 → 后果 → 修法）

> 分级：🔴 阻断 · 🟠 高危 · 🟡 中等 · ⚪ 卫生。

#### 🔴 阻断

| # | 一句话 | 位置 |
|---|---|---|
| R-01 | **定位链路整体不可用**：3 个 Python 节点 import 即崩，`/localization` 从未发布 | `notes/fastlio.md` §4.2 |
| R-02 | **飞控板从未连接**，PX4 参数零备份（全盘搜 `EKF2_*`/`COM_OBL` 零命中），路径跟随无法真机验证 | 全盘搜索 |
| R-03 | `pursuit_goto_timeout=1.0s` 偏紧；`start_trajectory()` 在许可回调里同步构建样条，耗时未知 | `px4_control_node.cpp:579-586` |

#### 🟠 高危

| # | 一句话 | 位置 |
|---|---|---|
| R-04 | **录了新路径却飞旧路径**（文件契约断裂 + `overwrite_existing=false` 自动改名） | `px4_path/config/params.yaml:25` vs `:119` |
| R-05 | **起节点即自动开飞**：许可是 transient_local 锁存，跟随器在回调里直接进 PURSUING；接驳段总长无上限 | `path_follow_trajectory_node.cpp:447, 210-229` |
| R-06 | **误差纠正不受轨迹限速约束**：参考冻结时发"1.5m 外的位置目标 + 前馈全 0"，实际速度由 `MPC_XY_VEL_MAX` 决定 | `:483-490, 341-346` |
| R-07 | **waypoint 前馈与 PX4 打架**：显式发 v/a/yawspeed=0，而 `px4_control` 自己的注释写明"0=前馈会打架"，非追踪分支特意用 NaN | `path_follow_waypoint_node.cpp:184-191` vs `px4_control_node.cpp:505-509` |
| R-08 | **`/initialpose` 是 VOLATILE 且只发一次**，首发竞态不可自愈，ICP 失败静默且无超时 | `notes/fastlio.md` §5、§7-R1/R2 |
| R-09 | SLAM 世界系**不对齐重力、不对齐偏航**，先验地图来源不明（mtime 与源码树相同 → 随项目拷入） | `IMU_Processing.hpp:200` |
| R-10 | 自主降落依赖 land detector + `COM_DISARM_LAND`（未验证），无超时兜底 | `px4_control_node.cpp:617-646` |
| R-11 | 跨机共用 ROS 域 + 通用话题名 → **误订阅/串扰风险** | `notes/camera.md` §6 |

#### 🟡 中等

| # | 一句话 | 位置 |
|---|---|---|
| R-12 | **幽灵参数**：launch 传 `max_position_jump_m`/`max_attitude_jump_rad`，节点只声明 7 个参数 → 跳变保护**根本不存在** | `px4_odom_convert.launch.py:18-19` |
| R-13 | 参数无边界：`pursuit_rate > 1000` → 定时器周期截成 **0 ms**；`NaN` → `std::max` 行为未定义 | `path_follow_waypoint_node.cpp:39-40` |
| R-14 | `LoadOptions` 的 `z_min_enu/z_max_enu/quat_norm_*` 是**硬编码默认值**，YAML 改不动；且 z 与四元数检查只是软警告 | `path_loader.hpp:27-30` |
| R-15 | 录制只在进程正常退出时落盘，SIGKILL/断电全丢 | `path_record_node.cpp:114-118` |
| R-16 | `path_dir` 硬编码；README 仍写开发机路径 `/home/lyl/code/...` 与 params.yaml 的 `/home/duzhong/...` **两个用户名冲突** | `params.yaml:21,114` / `README.md:65` |
| R-17 | RViz 配置名拼错：`fastlio_localiztion.rviz` 不存在 | `velodyne_localization.launch.py:16` |
| R-18 | `px4_msgs 2.0.1` 与飞控固件强耦合，机上无固件版本证据 | `px4_msgs/package.xml:5` |
| R-19 | 两个跟随节点约 150 行**逐字重复**，应抽 `PathFollowBase` | 两文件对照 |
| R-20 | **白色地垫 + 无围挡 → FAST-LIO 漂移**（风险 A，未实测） | §2.8.6 |

#### ⚪ 卫生

| # | 一句话 |
|---|---|
| R-21 | `on_goto()` 静默丢弃外部目标（无日志）；`handle_move` 无活动半径限制 |
| R-22 | 死代码：`CMakeLists.txt:73-77` 的 `install(DIRECTORY path)` **恒假**（真实目录是 `src/path/`）；`test_py` 假里程计 |
| R-23 | `install/` 装进手册完全用不到的一整套：`mavros`(13MB)、`ego-planner` 派生的十多个包、`FASTLIO2_ROS2`/`hba` |
| R-24 | **整个工作空间无版本控制**（`~/dzpx4`、本地 `dzsmartscout` 全部非 git），没有可回滚历史 |
| R-25 | Python stdout 重定向到文件是**块缓冲**，不满 4KB 不落盘 ⇒ `7_path.log` 0 字节 ≠ 节点没起来 |

#### 补充缺陷（本轮新发现）

| # | 现象 | 位置 | 说明 |
|---|---|---|---|
| N-01 | `SupervisorNode._on_perception` 里 `return` 落在 `diag(...)` 之后、`ingest_sample` 之前 | `dzlink_supervisor.py:1283-1286` | 见 §7 未核实清单（代码路径可达性需确认） |
| N-02 | `MotionTrajectory::sample()` 在 `!valid_` 时返回全 0 样本（= EKF 原点） | `motion_trajectory.cpp:520-525` | 当前不可达；一旦引入会把飞机指向原点 |
| N-03 | 拉伸后未再断言测量值是否落在限值内 | `motion_trajectory.cpp:408` | 数学推导正确但代码层无校验 |
| N-04 | `find_path_turn_splits` 双调用、`measure_limits` 双调用（每次 `max(2000,40N)` 点 × 4 次样条求值） | `path_follow_trajectory_node.cpp:204,289`、`motion_trajectory.cpp:385,408` | 纯重复计算 |
| N-05 | `average_poses` 会 `throw std::invalid_argument`，rclcpp 回调里无 catch → `std::terminate` | `path_record_process.cpp:111-113` | 触发条件：磁盘慢/满或改动 `smooth_path` 边界 |

## §3 函数级索引大表

> 格式：`文件:行号` | 签名 | 一句话职责。尽量穷尽自研包的函数/类/成员。`main` 与构造/析构单列。

### 3.1 px4_control（src/px4_control/）

| 位置 | 签名 | 职责 |
|---|---|---|
| `src/px4_control_node.cpp:27` | `ControlNode::ControlNode()` | 声明参数、创建 4 订阅/4 发布/1 定时器；启动即 `publish_goto_enabled(false)` |
| `:94` | `uint64_t now_us()` | 当前微秒时间戳 |
| `:98` | `bool is_offboard() const` | `status_received_ && nav_state==14` |
| `:102` | `bool is_armed() const` | `arming_state==2` |
| `:106` | `bool is_on_ground() const` | disarmed 恒真；armed 用 `|z|<0.20 && |vz|<0.15` |
| `:119` | `const char* state_name(FlightState)` | 状态名 |
| `:137` | `void set_state(FlightState, const char*)` | 状态迁移 + 日志 |
| `:146` | `void publish_goto_enabled()` | 发 `/px4_control/goto_enabled` |
| `:152` | `void disable_goto_and_hold(const char*)` | 清许可/前馈，HOVER 下钉住 target |
| `:177` | `void on_local_position(SharedPtr)` | 更新 current_、local_pos_valid_；DISARMED/LANDED 时钉 target |
| `:200` | `void on_vehicle_status(SharedPtr)` | 检测 STANDBY 完成降落；Offboard 丢失退 DISARMED |
| `:253` | `void on_command(SharedPtr)` | 按 `m->cmd` 分派 9 种命令 |
| `:285` | `void on_goto(SharedPtr)` | Goto 准入门控（active/许可/HOVER/有限性） |
| `:335` | `void handle_offboard()` | 攒够心跳后发 DO_SET_MODE(Offboard) |
| `:348` | `void handle_arm()` | 地面+Offboard+位置有效才 arm；设 target_.z=-takeoff_height |
| `:382` | `void handle_disarm()` | 地面才放行；否则 disable_goto_and_hold |
| `:398` | `void handle_land()` | NAV_LAND 或自主下降 |
| `:431` | `void handle_move(float,float,float)` | HOVER 下平移 target；goto 使能时拒绝 |
| `:448` | `void handle_yaw(float)` | HOVER 下偏航；归一到 [-π,π] |
| `:468` | `void handle_goto_enable()` | HOVER+位置有效 → goto_enabled_=true |
| `:486` | `void handle_goto_disable()` | disable_goto_and_hold |
| `:494` | `void publish_offboard_control_mode()` | 只置 position=true 的心跳 |
| `:505` | `void publish_trajectory_setpoint()` | 组装 NED 位置 + 前馈（NaN 约定） |
| `:548` | `void publish_vehicle_command(uint16_t,float×7)` | 组装 VehicleCommand（from_external=true） |
| `:573` | `void on_timer()` | 心跳 + Goto 超时 + 状态推进（ARMING/TAKEOFF/LANDING） |
| `:656` | `int main(int,char**)` | init/spin/shutdown |

**teleop**：`TeleopNode::TeleopNode`（:12）、`enable_raw_mode`（:46）、`restore_terminal`（:60）、`publish_cmd`（:67）、`publish_move`（:76）、`publish_yaw`（:85）、`keyboard_loop`（:94）、`main`（:171）。

### 3.2 px4_path（src/px4_path/）

**motion_trajectory.cpp**

| 位置 | 签名 | 职责 |
|---|---|---|
| `:15` | `double norm3(array<double,3>)` | 三维范数 |
| `:20` | `std::array<double,3> cross3(...)` | 三维叉积 |
| `:28` | `std::vector<PathTurnSplit> find_path_turn_splits(points, threshold_deg)` | 急弯切点检测 |
| `:67` | `NaturalCubicSpline::build(knots,values)` | 自然三次样条（Thomas 三对角） |
| `:121` | `NaturalCubicSpline::build_clamped(knots,values,first,last)` | 夹紧三次样条（首末斜率指定） |
| `:176` | `std::array<double,4> evaluate(double knot)` | 样条求值 {value,first,second,third} |
| `:208` | `double MotionTrajectory::wrap_pi(double)` | 角度归一 |
| `:218` | `bool MotionTrajectory::build(points,limits,error)` | 完整轨迹生成（去重→空间样条→曲率限速→加减速→时间样条→测量→拉伸） |
| `:421` | `SpatialSample sample_spatial(double path_s) const` | 空间样条求值 |
| `:437` | `TrajectorySample sample_kinematics(ProgressSample) const` | 链式法则求 v/a/j |
| `:458` | `ProgressSample sample_progress(double time_s) const` | 时间样条求值 s/ṡ/s̈/s⃛ |
| `:473` | `bool rebuild_progress_spline()` | 夹紧时间样条 + duration |
| `:482` | `void measure_limits()` | 全量限值测量（max(2000,40N) 点） |
| `:520` | `TrajectorySample sample(double time_s) const` | 对外采样入口（!valid_ 返回全 0 ⚠️） |

**path_loader.cpp**

| 位置 | 签名 | 职责 |
|---|---|---|
| `:11` | `double wrap_pi(double a)` | 角度归一 |
| `:30` | `Position3 slam_to_enu(x,y,z)` | SLAM→ENU（绕 Z 转 180°） |
| `:34` | `Position3 enu_to_ned(enu)` | ENU→NED |
| `:36` | `std::string trim(std::string)` | 去空白 |
| `:47` | `LoadResult load_path(path, opts)` | 读文件 + 校验 + 转换（唯一转换点） |

**path_record_node.cpp**

| 位置 | 签名 | 职责 |
|---|---|---|
| `:10` | `bool is_finite_pose(SlamPathPose)` | NaN/Inf 检查 |
| `:17` | `bool normalize_quaternion(SlamPathPose&)` | 四元数归一 |
| `:30` | `double position_distance(a,b)` | 三维距离 |
| `:38` | `double horizontal_distance(a,b)` | 水平距离 |
| `:43` | `double vertical_distance(a,b)` | 竖直距离 |
| `:49` | `PathRecordNode::PathRecordNode()` | 声明参数、订阅/发布、启动录制 |
| `:121` | `bool validate_parameters()` | 参数校验（失效安全） |
| `:165` | `bool prepare_output_path()` | 建目录 + 解析输出路径（overwrite=false 时找 _1/_2…） |
| `:200` | `void start_recording()` | 清缓存 + 发空 Path + recording_=true |
| `:224` | `void append_final_pose()` | 补末点（跳变 > max_neighbor 则丢弃） |
| `:253` | `void stop_recording(const char*)` | 落盘流程（补末点→finalize→终检→原子写） |
| `:295` | `bool write_path_file(poses)` | 相邻距离终检 + 写 .tmp + rename |
| `:363` | `void publish_visualization(poses)` | 发 nav_msgs/Path |
| `:387` | `void on_slam_odom(SharedPtr)` | 帧检查→跳变检查→滤波→关键点判定 |
| `:488` | `int main(int,char**)` | init/spin |

**path_record_process.cpp**

| 位置 | 签名 | 职责 |
|---|---|---|
| `:11` | `PathRecordProcessor(rclcpp::Node&)` | 声明 filter/smoothing 参数 |
| `:19` | `bool validate_parameters() const` | 窗口 1~101 校验 |
| `:41` | `void log_configuration() const` | 配置日志 |
| `:55` | `void write_metadata(ostream&) const` | 写文件头元数据 |
| `:63` | `void reset()` | 清滤波缓冲 |
| `:65` | `SlamPathPose filter(pose)` | 在线滑动均值（窗口 filter_window） |
| `:75` | `std::vector<SlamPathPose> finalize(poses) const` | 平滑 + 近点压缩 |
| `:81` | `bool normalize_quaternion(pose&)` | 四元数归一 |
| `:95` | `double position_distance(a,b)` | 三维距离 |
| `:103` | `double quaternion_dot(a,b)` | 四元数点积（定符号） |
| `:109` | `SlamPathPose average_poses(poses,begin,end)` | 区间平均（四元数取同号） |
| `:149` | `std::vector<SlamPathPose> smooth_path(poses,window)` | 中心滑动平均（首尾不动） |
| `:169` | `std::vector<SlamPathPose> remove_close_points(poses,min_dist)` | 近重复点压缩（首末恒保留） |

**path_follow_waypoint_node.cpp**

| 位置 | 签名 | 职责 |
|---|---|---|
| `:12` | `PathFollowWaypointNode::PathFollowWaypointNode()` | 声明参数、订阅/发布/定时器、加载路径 |
| `:71` | `state_name(WaypointFollowState)` | 状态名 |
| `:87` | `void set_state(...)` | 状态迁移 + 日志 |
| `:97` | `double distance(a,b)` | 三维距离 |
| `:104` | `Waypoint clamp_step(from,target) const` | 单步限幅（≤max_step_dist） |
| `:119` | `bool prepare_path_directory()` | 建目录 + 解析文件路径 |
| `:143` | `bool reload_path()` | 加载 + 校验（只在构造时调用一次） |
| `:178` | `void publish_goto(Waypoint)` | 发 GotoTarget（v/a/yawspeed=0 ⚠️） |
| `:195` | `void publish_stop()` | 发 active=false |
| `:201` | `void activate_if_ready()` | 许可+位置有效+LOADED → PURSUING |
| `:227` | `void on_local_position(SharedPtr)` | 更新 current；首帧 → LOADED |
| `:241` | `void on_goto_enabled(SharedPtr)` | 锁存许可处理（PAUSED 恢复） |
| `:260` | `void on_timer()` | 游标推进 + clamp_step + HOLD |

**path_follow_trajectory_node.cpp**

| 位置 | 签名 | 职责 |
|---|---|---|
| `:12` | `PathFollowTrajectoryNode::PathFollowTrajectoryNode()` | 声明参数、订阅/发布/定时器、加载路径 |
| `:97` | `state_name(TrajectoryFollowState)` | 状态名 |
| `:113` | `void set_state(...)` | 状态迁移 + 日志 |
| `:123` | `double distance(a,b)` | 三维距离 |
| `:131` | `double wrap_pi(double)` | 角度归一 |
| `:141` | `bool prepare_path_directory()` | 建目录 + 解析文件路径 |
| `:165` | `bool reload_path()` | 加载 + 校验（只在构造时调用一次） |
| `:197` | `make_execution_segments() const` | 接驳段（当前→首点）+ 急弯切段 |
| `:249` | `bool start_trajectory()` | 参数总检 + 分段 + 每段 build + 打日志 |
| `:336` | `void publish_reference(sample,dt,scale,scale_rate)` | 重定时 + yaw 模式 + 组装 GotoTarget |
| `:387` | `void publish_stop()` | 发 active=false |
| `:393` | `void activate_if_ready()` | start_trajectory → PURSUING |
| `:409` | `void on_local_position(SharedPtr)` | 更新 current/speed；首帧 → LOADED |
| `:430` | `void on_goto_enabled(SharedPtr)` | 锁存许可处理（PAUSED 只重置 timer 基准） |
| `:450` | `void on_timer()` | 误差→scale→夹紧→推进→采样→中止判定→发布→切段 |

**path_follow.cpp**

| 位置 | 签名 | 职责 |
|---|---|---|
| `:11` | `std::string read_follow_mode()` | 读 `follow_mode` 参数 |
| `:18` | `int main(int,char**)` | 按 mode 实例化 waypoint 或 trajectory |

### 3.3 px4_odom_convert（src/px4_odom_convert/）

| 位置 | 签名 | 职责 |
|---|---|---|
| `src/px4_odom_convert.cpp:12` | `double wrapPi(double a)` | 角度归一 |
| `:25` | `double yawFromQuat(Quaterniond)` | 四元数提取 yaw |
| `:32` | `Quaterniond quatFromYaw(double)` | yaw 建四元数 |
| `:40` | `Px4OdomConvert::Px4OdomConvert()` | 声明参数、防双实例、订阅/发布、健康定时器 |
| `:212` | `uint64_t nowUs()` | 微秒时间戳 |
| `:217` | `void odomCallback(SharedPtr)` | 收满 20 帧后开始发布 |
| `:251` | `void onFcuStatus(SharedPtr)` | 记录 armed 状态 |
| `:257` | `void onEkfLocalPosition(SharedPtr)` | EKF 位置 + reset 风暴检测/解除 |
| `:350` | `void writeBlackbox(...)` | 4 行事件级 blackbox |
| `:400` | `void onHealthTimer()` | 3s 周期健康行 + 窗口统计复位 |
| `:452` | `void computeAlignment(slam_yaw,ekf_heading)` | 对齐完成 + bump reset_counter |
| `:470` | `void publishVehicleVisualOdometry(odom)` | 转换 + auto-align + z 换源 + jump guard + 发布 |
| `:911` | `void lookupTfAndPublish()` | tf 模式：查 TF 并发布 |
| `:944` | `void run()` | odom/localization → spin；tf → 50Hz 循环 |
| `:960` | `int main(int,char**)` | try 构造（防双实例）+ run |

### 3.4 px4_imu_bridge（src/px4_imu_bridge/）

| 位置 | 签名 | 职责 |
|---|---|---|
| `src/sensor_combined_to_imu.cpp:14` | `SensorCombinedToImu::SensorCombinedToImu()` | 声明参数、订阅/发布 |
| `:42` | `std::array<double,3> convertVectorFrdToRosFlu(input)` | FRD→FLU（Y/Z 取反） |
| `:62` | `void recordConversionTime(double us)` | 周期统计日志 |
| `:95` | `void sensorCombinedCallback(SharedPtr)` | 组装 Imu + 发布 |
| `:141` | `int main(int,char**)` | init/spin |

### 3.5 dzuav（src/uav_camera_code/onboard/dzuav/）

**main.cpp**

| 位置 | 签名 | 职责 |
|---|---|---|
| `:28` | `void handleSignal(int)` | SIGINT/SIGTERM → g_running=false |
| `:33` | `const char* className(ObjectClass)` | 类别名表（与 types.hpp 顺序一致） |
| `:43` | `bool estimateCarYaw(image,box,map,out_x,out_y,out_yaw)` | 车头朝向估计（红标记质心） |
| `:70` | `void processFrame(frame,hbb,cell,map,udp,output,preview)` | 单帧流水线 |
| `:191` | `std::string argusPipeline(sensor_id,w,h,fps)` | MIPI CSI GStreamer 管道 |
| `:206` | `std::string uvcPipeline(device,w,h,fps,mjpeg)` | USB UVC GStreamer 管道 |
| `:221` | `void parseSpec(spec,width,height,fps)` | 解析 `1920x1080@30` |
| `:232` | `int runCamera(source,hbb,cell,port,spec)` | 相机实时模式主循环 |
| `:311` | `int main(int,char**)` | 参数分派（argus/uvc/文件） |

**HbbDetector.cpp**

| 位置 | 签名 | 职责 |
|---|---|---|
| `:8` | `HbbDetector::HbbDetector(path,confidence,nms)` | 绑定 1280² 输入、11×33600 输出 |
| `:29` | `std::vector<Detection> infer(const cv::Mat&)` | resize→blob→enqueue→解析→逐类 NMS |

**CellClassifier.cpp**

| 位置 | 签名 | 职责 |
|---|---|---|
| `:7` | `CellClassifier::CellClassifier(path)` | 绑定 96² 输入、双头输出 |
| `:30` | `CellId infer(const cv::Mat& crop)` | ±10° TTA + softmax 平均 + argmax |

**MapLocalizer.cpp**

| 位置 | 签名 | 职责 |
|---|---|---|
| `:14` | `cv::Mat scaledIntrinsics(base,sz)` | 按分辨率缩放内参 |
| `:23` | `MapLocalizer::MapLocalizer()` | 写死 K_ / dist_ |
| `:28` | `cv::Mat undistort(const cv::Mat&) const` | 去畸变（映射缓存） |
| `:42` | `cv::Point3f world(const CellId&) const` | 格号世界坐标 |
| `:46` | `bool solve(landmarks,size)` | PnP 主路径 + 单应兜底 |
| `:147` | `cv::Point2d project(const cv::Point2f&) const` | 像素→世界 |

**UdpBroadcaster.cpp**

| 位置 | 签名 | 职责 |
|---|---|---|
| `:14` | `const std::array<uint16_t,256>& crcTable()` | CRC-16/Modbus 查表 |
| `:31` | `uint16_t crc(const uint8_t*,size_t)` | CRC 计算 |
| `:46` | `UdpBroadcaster::UdpBroadcaster(uint16_t p)` | 建 socket + SO_BROADCAST |
| `:57` | `void send(uint64_t,vector<WorldObject>)` | 目标帧 |
| `:78` | `void sendCarPose(uint64_t,double,double,float)` | 车位姿帧 |
| `:93` | `void sendBuf()` | sendto 广播 |

**TensorRtEngine.hpp**

| 位置 | 签名 | 职责 |
|---|---|---|
| `:21` | `TensorRtEngine(engine_path)` | 读文件 + 建 runtime/engine/context/stream |
| `:45` | `~TensorRtEngine()` | delete 三对象 + 销毁 stream |
| `:61` | `class Logger : public ILogger` | TensorRT 日志（≤WARNING 打 stderr） |

### 3.6 dzlink_supervisor.py（tools/link/）

| 位置 | 签名 | 职责 |
|---|---|---|
| `:138` | `_parse_env_file(path)` | 解析 env 文件 |
| `:155` | `bootstrap_domain(env_file,argv,quiet)` | **导入期**设 ROS_DOMAIN_ID（早于 rclpy.init） |
| `:215` | `default_log_path()` | 默认日志路径 |
| `:221` | `expand(path)` | ~ 展开 |
| `:225` | `_force_utf8(stream)` | stdout 强制 utf-8 |
| `:244` | `diag(msg)` | 诊断打印 |
| `:253` | `class IntelRecord` + `:263 age(now)` | 情报记录 + 时效 |
| `:268` | `class OutEvent` | 待发事件 |
| `:282` | `class CarMsg` | 车回传消息 |
| `:290` | `class CoreStats` + `:305 snapshot(now)` | 统计 + 快照 |
| `:327` | `decide(sample, left_halves)` | **纯函数**：样本→[(type,payload)] |
| `:395` | `_norm_block(value)` | 格名归一（含 SHOOT@ 剥离） |
| `:410` | `_block_list(sample,keys)` | 从多键取格名列表 |
| `:438` | `_bridge_payload(sample,left_halves)` | 桥洞解析（half/direction 映射） |
| `:473` | `_direction_map(left_halves)` | left/right → half 映射 |
| `:479` | `canonical_text(event_type,payload,seq,ts)` | 组装车侧 JSON 文本 |
| `:512` | `intel_key(etype,payload)` | 情报内容指纹 |
| `:525` | `slot_sort_key(slot)` | 槽位排序 |
| `:535` | `slot_key(etype,payload)` | 槽位名 |
| `:563` | `validate_route(text)` | 路线形状校验 |
| `:603` | `_route_token_problems(tok,idx,last)` | 单个 token 校验 |
| `:630` | `route_blocks(text)` | 从路线抽格名 |
| `:644` | `reset_command(block,yaw)` | 组装 reset 命令 |
| `:654` | `route_payload(text)` | 校验 + 规范化路线 |
| `:665` | `class FileTailer` + `:672 __init__`、`:683 poll`、`:721 lines_read` | 文件增量尾随（含截断/半行） |
| `:725` | `class GlobTailer` + `:728 __init__`、`:732 match`、`:735 poll` | glob 多文件尾随 |
| `:746` | `sample_from_file_line(line)` | JSONL 行解析 |
| `:757` | `class UdpJsonSource` + `:766 __init__`、`:776 poll`、`:797 close` | UDP JSON 输入 |
| `:804` | `parse_car_message(raw,ts)` | 车回传解析 |
| `:819` | `class AckLedger` + `:831 __init__`、`:842 note_sent`、`:851 note_ack`、`:866 pending`、`:869 summary` | ack 台账 |
| `:889` | `class JsonlLogger` + `:892 __init__`、`:904 log`、`:923 _rotate`、`:934 close` | JSONL 日志（轮转） |
| `:946` | `class CoreConfig` + `:958 normalise`、`:969 period_for` | 配置 + 归一 + 周期 |
| `:976` | `class SupervisorCore` + `:979 __init__` | 决策核心 |
| `:996` | `next_seq()` | 单调序号 |
| `:1000` | `submit_route(text,repeat,now,source)` | 路线入队（校验） |
| `:1016` | `submit_reset(block,yaw,repeat,now)` | reset 入队 |
| `:1027` | `submit_heartbeat()` | 安排立即心跳 |
| `:1032` | `request_stop()` / `:1036 stopping` | 停止请求 |
| `:1040` | `ingest_sample(sample,source,now)` | 吃感知样本 → 台账 |
| `:1082` | `ingest_car_raw(raw,now)` | 吃车回传 |
| `:1112` | `_stale_slots(now)` | 过期槽位 |
| `:1116` | `_make_intel_event(slot,now,note)` | 造情报事件 |
| `:1131` | `tick(now)` | **决策主循环**（过期→路线→心跳→情报配对） |
| `:1197` | `status(now)` / `:1222 log_status` | 状态汇总 |
| `:1226` | `_short(obj,limit)` | 短化 JSON |
| `:1237` | `make_qos(reliability)` | QoS 构造 |
| `:1248` | `class SupervisorNode` + `:1251 __init__` | DDS 层 |
| `:1276` | `_on_car(msg)` | 车回传回调 |
| `:1280` | `_on_perception(msg)` | 感知话题回调 ⚠️ return 位置（N-01） |
| `:1288` | `_settle(timeout_s)` | spin 等待 |
| `:1294` | `_subscription_count(pub)` | 订阅者计数 |
| `:1300` | `dispatch(events)` | 发送一批事件（重试/匹配数） |
| `:1333` | `_subscriptions_note(topic,matched)` | 订阅者变化日志 |
| `:1339` | `spin_step(timeout_s)` | 单步 spin |
| `:1343` | `maybe_status(now)` | 周期状态日志 |
| `:1355` | `destroy()` | 销毁节点 |
| `:1365` | `class CommandFile` + `:1368 __init__`、`:1372 poll` | 命令文件尾随 |
| `:1390` | `CommandFile.execute(cmd,core)` | 执行一条命令（坏命令跳过） |
| `:1440` | `build_argparser()` | 构建参数解析器 |
| `:1549` | `_core_from_args(args)` | 从 args 造核心 |
| `:1568` | `_add_samples_from_args(core,args)` | 注入 --bridge/--target/--mines/--obstacles |
| `:1589` | `cmd_contract()` | 契约输出 |
| `:1626` | `cmd_selftest()` | 纯逻辑自检 |
| `:1725` | `cmd_dry_run(args)` | dry-run |
| `:1745` | `_run_ros(args,core,mode)` | **常驻主循环** |
| `:1881` | `main(argv)` | 入口分派 |
| `:1947` | `_run_ros_once(args,core,events,only_route)` | 一次性发送 |
| `:1993` | `_install_signal_handlers()` | SIGINT/SIGTERM → KeyboardInterrupt |

### 3.7 dz_send_waypoint.py（tools/link/）

| 位置 | 签名 | 职责 |
|---|---|---|
| `:80` | `make_qos(reliability)` | depth10 + VOLATILE + KEEP_LAST |
| `:90` | `now_s()` | 秒级时间 |
| `:101` | `split_blocks(tokens)` | 格名拆分（t12/D4） |
| `:120` | `had_separator(tokens)` | 检测非空格分隔符 |
| `:125` | `note_split(raw,blocks,label)` | 打印拆分提示 |
| `:135` | `route_payload(text)` | 裸文本路线 |
| `:143` | `bridge_payload(half,block,confidence,seq)` | 桥洞 JSON |
| `:158` | `heartbeat_payload(seq)` | 心跳 JSON |
| `:165` | `blocks_payload(kind,blocks,seq)` | 靶区 JSON |
| `:174` | `field_payload(mines,obstacles,seq)` | 雷障 JSON |
| `:189` | `send(topic,reliability,texts,repeat,period,wait_match_s,label)` | 发送 + DDS 匹配等待 |
| `:237` | `listen(duration)` | 监听车回传 |
| `:272` | `parse_kv(pairs)` | key=value 解析 |
| `:282` | `main(argv)` | 入口分派 |

### 3.8 perception（tools/perception/）

**protocol.py**

| 位置 | 签名 | 职责 |
|---|---|---|
| `:97` | `_now(ts)` | 时间 |
| `:101` | `_norm_scan_id(scan_id)` | scan_id 校验 |
| `:113` | `make_scan_id(ts,token)` | 生成扫描号 |
| `:126` | `_norm_cells(cells,spec,key)` | 覆盖格校验 |
| `:150` | `_norm_coverage(coverage,spec)` | coverage 校验 |
| `:165` | `_norm_slice(slice_info,spec)` | slice 校验 |
| `:196` | `_norm_confidence(confidence,spec)` | 置信度校验 |
| `:220` | `_norm_blocks(blocks,spec,strict)` | 格名归一 |
| `:245` | `build_heartbeat(ts,seq)` | 心跳 |
| `:253` | `build_bridge_passable(half,block,confidence,ts,seq)` | 桥洞 |
| `:269` | `build_target_zone(blocks,spec,ts,seq)` | 靶区 |
| `:281` | `build_field_verify(mines,obstacles,spec,diff,coverage,slice,scan_id,confidence,ts,seq)` | 雷障 + 覆盖 |
| `:327` | `encode(payload)` | JSON 编码 |
| `:332` | `encode_all(payloads)` | 批量编码 |
| `:336` | `validate_payload(payload,spec)` | 发送端自检 |
| `:418` | `mirror_car_side_parse(raw,spec)` | 车侧解析镜像 |
| `:490` | `coverage_absent_cells(coverage,reported,spec)` | 负证据推导 |
| `:513` | `field_verify_absent(payload,spec)` | field_verify 负证据 |
| `:527` | `recommended_cadence(heartbeat_hz)` | 建议发送节奏 |

**grid.py**

| 位置 | 签名 | 职责 |
|---|---|---|
| `:84` | `class GridSpec` | 网格规格（列/行/格边/原点） |
| `:181` | `arena_corner_meters(tag,spec)` | 角点米坐标 |
| `:203` | `class Homography` | 单应封装 |
| `:231` | `_normalize(pts)` | Hartley 归一化 |
| `:243` | `fit_homography(src_xy,dst_uv)` | DLT 拟合 + 残差 |
| `:275` | `_signed_area(pts)` | 有向面积（手性判据） |
| `:285` | `class CellHit` | 格命中结果 |
| `:310` | `class ArenaRegistration` | 注册器（px_to_hit/cell_polygon/self_check/diagnostics） |
| `:411` | `register(pairs,spec)` | 通用注册 |
| `:449` | `register_from_corners(corners_px,spec)` | 四角点注册 |
| `:466` | `register_from_named_corners(px_of,spec)` | 标签化角点注册 |
| `:486` | `_runs_1d(mask)` | 1D 连续段 |
| `:500` | `register_from_arena_frame(rgb,spec,...)` | 从场地外框自动注册 |

**colors.py**

| 位置 | 签名 | 职责 |
|---|---|---|
| `:48` | `class ImageShapeError(ValueError)` | 形状异常 |
| `:55` | `as_rgb_array(image)` | 转 RGB numpy |
| `:73` | `rgb_to_hsv(rgb)` | RGB→HSV |
| `:96` | `hsv_to_rgb(hsv)` | HSV→RGB |
| `:116` | `class ColorRule` | 颜色规则（HSV/RGB 阈值） |
| `:173` | `class Palette` | 调色板（to_json/from_json） |
| `:322` | `classify(rgb,palette)` | 逐类分类 |
| `:330` | `_offsets(k)` | 结构元素偏移 |
| `:338` | `dilate/erode/open_/close_` | 形态学 |
| `:386` | `class Blob` | 连通域（矩/质心/bbox） |
| `:453` | `connected_components(mask,min_pixels)` | run-based union-find |
| `:555` | `blob_masks(mask,min_pixels,k_open)` | 形态学后取 blob |
| `:571` | `sample_patch_stats(rgb,x,y,radius)` | 局部统计 |

**objects.py / car_pose.py / bridge_arrow.py**

| 位置 | 签名 | 职责 |
|---|---|---|
| `objects.py:48` | `class ObjectClassSpec` | 单类几何判据 |
| `objects.py:95` | `class ObjectDetection` | 单目标检测结果 |
| `objects.py:119` | `class FieldObjects` | 一帧全部目标 |
| `objects.py:157` | `_mid_confidence(value,lo,hi)` | 置信度折算 |
| `objects.py:166` | `group_blocks(blocks,grid)` | 按区域分组 |
| `objects.py:193` | `detect_field_objects(rgb,reg,palette,...)` | 雷/障/靶检测 + 冲突消解 |
| `car_pose.py:37` | `class CarSpec` | 车顶板/车头标记规格 |
| `car_pose.py:53` | `class CarPose` | 车位姿结果 |
| `car_pose.py:89` | `_axis_angle_meters(pts_m)` | 主轴 + 角度 + 米 |
| `car_pose.py:108` | `_wrap180(deg)` | 归一到 ±180 |
| `car_pose.py:112` | `_blob_points_meters(blob,reg)` | blob → 米坐标 |
| `car_pose.py:120` | `detect_car(rgb,reg,palette,...)` | 车检测 |
| `car_pose.py:203` | `best_car(rgb,reg,palette,...)` | 多候选择优 |
| `bridge_arrow.py:63` | `class ArrowSpec` | 箭头规格 |
| `bridge_arrow.py:81` | `class ArrowDetection` | 箭头检测结果 |
| `bridge_arrow.py:109` | `class BridgeCall` | 桥洞判定结果 |
| `bridge_arrow.py:132` | `_profile_along(points_centered,u)` | 沿轴宽度剖面 |
| `bridge_arrow.py:156` | `_const_run(seq,w_max,low_frac)` | 等宽平台判定 |
| `bridge_arrow.py:169` | `_mono_run(seq,w_max,tol_frac)` | 单调张开判定 |
| `bridge_arrow.py:187` | `_axis_evidence(p,u,n_bins)` | 单轴证据 |
| `bridge_arrow.py:247` | `_rel(a,b)` | 相对宽度 |
| `bridge_arrow.py:252` | `_tip_side(u,ev,points_centered)` | 尖端侧判定 |
| `bridge_arrow.py:281` | `direction_from_points(points_xy,n_bins,angle_step)` | 方向估计（扫描候选轴） |
| `bridge_arrow.py:393` | `_dir_kind(deg)` | 方向归类 |
| `bridge_arrow.py:399` | `detect_arrows(rgb,reg,palette,...)` | 箭头检测 |
| `bridge_arrow.py:438` | `infer_bridge_passable(arrows,spec)` | 桥洞通行判定 |

**coverage.py / field_map.py / pipeline.py / cli.py / ros_publish.py**

| 位置 | 签名 | 职责 |
|---|---|---|
| `coverage.py:104` | `_cell_centers(spec)` | 格心 |
| `coverage.py:110` | `_point_segment_distance(pts,a,b)` | 点到线段距离 |
| `coverage.py:121` | `cells_of_route(route,spec)` | 路线覆盖格 |
| `coverage.py:159` | `class Slice` | 单片 |
| `coverage.py:176` | `class SliceCoverage` | 片覆盖结果 |
| `coverage.py:189` | `class SlicePlan` | 片计划 |
| `coverage.py:255` | `_routes_to_slices(routes)` | 路线→片 |
| `coverage.py:262` | `plan_for(kind,spec,...)` | 预设片计划 |
| `coverage.py:278` | `parse_slice_spec(text,spec,...)` | 解析片规格 |
| `coverage.py:318` | `as_plan(obj,spec,...)` | 通用解析 |
| `field_map.py:82` | `confidence_thresholds(cls)` | 建议阈值 |
| `field_map.py:95` | `class BlueFaceEstimate` | 蓝面估计 |
| `field_map.py:115` | `_face_mask(rgb,spec)` | 面掩膜 |
| `field_map.py:123` | `estimate_blue_face(rgb,reg,cells,...)` | 蓝面朝向 |
| `field_map.py:201` | `motion_budget(speed_mps,fps)` | 运动预算 |
| `field_map.py:219` | `class FieldMap` | 场地地图 |
| `field_map.py:339` | `class Reconciliation` | 对账结果 |
| `field_map.py:373` | `build_field_map(objects,...)` | 出图 |
| `field_map.py:455` | `build_field_map_from_result(result,...)` | 从结果出图 |
| `field_map.py:476` | `write_field_map(fm,out_dir,stem)` | 写 JSON/YAML |
| `field_map.py:489` | `parse_map_yaml(text)` | 解析真源 map.yaml |
| `field_map.py:510` | `reconcile_with_map_yaml(fm,truth)` | 与真源对账 |
| `pipeline.py:61` | `class PerceptionConfig` | 管线配置 |
| `pipeline.py:81` | `config_for(kind,**overrides)` | 预设配置 |
| `pipeline.py:100` | `class PerceptionResult` | 管线结果 |
| `pipeline.py:132` | `class PerceptionPipeline` | 总装 |
| `cli.py:72` | `parse_corners(text,order)` | 角点解析 |
| `cli.py:91` | `parse_hsv_rule(cls,text)` | 颜色规则解析 |
| `cli.py:108` | `_load_image(path)` | 读图 |
| `cli.py:115` | `_build_config(args)` | 建配置 |
| `cli.py:141` | `_slice_plan(args)` | 片计划 |
| `cli.py:154` | `_scan_token(image_path,plan)` | 单图扫描 |
| `cli.py:171` | `_print_slices(res)` | 打印切片 |
| `cli.py:190` | `_write_coverage_report(path,res,plan)` | 写覆盖报告 |
| `cli.py:199-350` | `cmd_selftest/detect/coverage/calibrate/synth/arrow/topics` | 各子命令 |
| `cli.py:355` | `build_parser()` | 参数解析器 |
| `cli.py:440` | `main(argv)` | 入口 |
| `ros_publish.py:27` | `ros_available()` | rclpy 可用性 |
| `ros_publish.py:35` | `_qos(depth,reliability)` | QoS |
| `ros_publish.py:45` | `publish_messages(messages,topic,...)` | 真发 DDS |
| `ros_publish.py:87` | `listen_acks(duration_s,...)` | 听 ack |

### 3.9 tools/drone 关联节点

| 位置 | 签名 | 职责 |
|---|---|---|
| `dzlink_livox_bridge.py:52` | `class LivoxBridge(Node)` + `:92 on_seq`、`:99 on_cloud` | CustomMsg→PointCloud2 + 序号信标 |
| `dzlink_seq_beacon.py:22` | `class SeqBeacon(Node)` + `:44 tick` | 50Hz 序号信标 |
| `dzlink_seq_check.py:23` | `class SeqCheck(Node)` + `:38 cb` | 丢包对账 |
| `dzlink_points_check.py:22` | `class Check(Node)` + `:38 cb_pc`、`:57 cb_seq` | 点云消费自检 |
| `pc2_check.py:20` | `class Check(Node)` + `:32 cb` | PointCloud2 自检 |
| `probe_lidar.py:8` | `main()` | 雷达点数探针 |
| `xtest_type.py:25` / `lidar_char.py:34` / `cam_ros_snap.py:28` / `cam_ros_view.py:31` / `cam_sharpness.py:29` | 各类 Node/main | 专项诊断 |

## §4 数据流与线程模型

### 4.1 端到端数据流（感知 → 飞控 → 小车）

```text
[MID-360 雷达 192.168.3.105 ⇄ eno1 主机 192.168.3.189]
        │ /livox/lidar (CustomMsg 10Hz, 19968点)  /livox/imu (200.9Hz)
        ▼
[livox_ros_driver2 / fastlio_mapping]
        │ /Odometry ~44Hz (frame=camera_init→body)
        ├──────────────────────────────────────────┐
        ▼                                          ▼
[px4_odom_convert mode:=odom]            [path_record_node slam_topic=/localization] ❌死
   │ 1) R_enu_slam=diag(-1,-1,1) 2) ENU→NED      │（/localization 从未发布）
   │ 3) auto-align 4) z 换源 EKF               ▼
   │ 5) jump guard L1/L2/L3                路径 TXT（SLAM 原始坐标）
   ▼
[/fmu/in/vehicle_visual_odometry ~42Hz] ──► [PX4 EKF2] ──► [PX4 飞控板] ❌未插
                                                              │ /fmu/out/*
                                                              ▼
[px4_control_node] ◄── /px4_control/goto_enabled (锁存) ◄── [px4_teleop_node] 键盘
        │ 心跳 20Hz
        ▼
[/fmu/in/offboard_control_mode + trajectory_setpoint]
        │
        ▼
[PX4] ──► 自主飞行（从未发生）

[px4_path/path_follow_node] ──/px4_control/goto──► px4_control_node

[dzuav] 相机→HbbDetector→CellClassifier→MapLocalizer(PnP)→UdpBroadcaster
        │ 二进制 UDP :50000 ──► [车端 ag_integrate]（私有协议，supervisor 不解析）

[dzlink_supervisor / dz_send_waypoint] ──/nav/waypoint (RELIABLE)──► [车 path_executor]
        └──────────────/dzlink/drone_to_car (BEST_EFFORT JSON)──► [车 DroneTask]
                                                                  └──/dzlink/car_to_drone (ack)──┘
```

### 4.2 线程模型（每个进程）

| 进程 | 执行器 | 线程/回调 | 说明 |
|---|---|---|---|
| `px4_control_node` | `rclcpp::spin`（默认单线程 executor） | `on_local_position`、`on_vehicle_status`、`on_command`、`on_goto`、`on_timer` | **全部共享 `std::mutex mtx_`**（`:179,202,254,286,574`）→ 线程安全 |
| `px4_teleop_node` | `spin` + `std::thread kb_thread_` | `keyboard_loop` 读 STDIN；发布线程 spin | `running_` 是 `std::atomic<bool>`；析构 join + restore_terminal |
| `path_follow_node`（waypoint/trajectory） | `spin`（单线程） | `on_local_position`、`on_goto_enabled`、`on_timer` | **无锁**，靠单线程 executor 不变式（R-17） |
| `path_record_node` | `spin`（单线程） | `on_slam_odom` | 无定时器，纯事件驱动 |
| `px4_odom_convert` | `spin`（非 tf）/ `spin_some`+Rate（tf） | `odomCallback`、`onEkfLocalPosition`、`onFcuStatus`、`onHealthTimer`、10s `dup_check_timer` | 无显式锁；单线程 executor |
| `px4_imu_bridge` | `spin`（单线程） | `sensorCombinedCallback` | 无锁 |
| `dzuav` | **单线程 main 循环**（无 ROS） | `processFrame` | GStreamer appsink 取帧；`g_running` atomic |
| `dzlink_supervisor` | **自有 while 循环** + `rclpy.spin_once` | 文件尾随 / UDP / stdin 线程 / ROS 回调 | `SupervisorCore` 无锁；`_run_ros` 主循环驱动 |
| `dz_send_waypoint` | `spin_once` 内联 | `send`/`listen` | 单次进程 |
| `dzlink_livox_bridge` | `spin` | `on_cloud`、`on_seq`(timer) | 单线程 |
| FAST-LIO `fastlio_mapping` | **MultiThreadedExecutor(3)** | IMU / LiDAR / 映射三组互斥回调 | `IMU_SUBSCRIPTION_DEPTH=500`；`prop_kf`/传播历史独立锁（`MODIFICATIONS.md #6`） |

### 4.3 关键时序与锁存语义

- `/px4_control/goto_enabled` 是 **transient_local 锁存**：新订阅者一建立就收到上次发布的值。**后果**：若许可锁存为 true，重启 `path_follow_node` 会立刻进 PURSUING（R-05）。补丁 03 用「显式 OFF→ON 边沿」修复。
- `GotoTarget` 走**单话题保序**：目标与停止消息同 topic，保证顺序（`GotoTarget.msg:5`）。
- `active=false` **不看当前许可**，无条件关许可（`px4_control_node.cpp:288-291`）。
- Goto 超时 `pursuit_goto_timeout=1.0s`：跟随节点崩了 → 1s 后 control 关许可悬停（`px4_control_node.cpp:579-586`）。
- 桥洞两半场必须**同轮成对**发出（`dzlink_supervisor.py:1165-1193`），因车侧等情报窗口只有 2s。
- 情报有效期 30s（车收到时刻算起）→ **周期重发是有效续期**（`protocol.py:21-23`）。

### 4.4 坐标系流转（关键）

```text
SLAM (FAST-LIO camera_init, Z向上, 偏航任意)
   │ slam_to_enu: (x,y,z)→(-x,-y,z)          [path_loader.cpp:30-32]
   │                                            [px4_odom_convert.cpp:51-54 R_enu_slam=diag(-1,-1,1)]
   ▼
ENU (假设)
   │ enu_to_ned: (x,y,z)→(y,x,-z)             [path_loader.cpp:34]
   │                                            [px4_odom_convert.cpp:531-534 enu_to_ned_local_frame]
   ▼
NED (PX4)
```

- yaw：`yaw_slam → +π → π/2 − yaw_enu`（`path_loader.cpp:119-122`）。
- ⚠️ **前置假设**：`map` 帧 +X 必须恰好与真实东向相差 180°（`notes/fastlio.md §4.2`）。若不成立，EKF2 拿到的**绝对航向系统性错误**。
- `px4_path` 录制保存 **SLAM 原始坐标**，加载时才转 NED（`path_record_node.cpp:48`、`path_loader.cpp:47`）。

### 4.5 任务时序（人做了什么 → 飞机怎么动）

```text
① YAML 改 record_mode=true → 重编译 → 起 px4_path（path_record_node）
② 人遥操飞完整航线 → Ctrl+C（★ 只有正常退出才落盘）
③ YAML 改 record_mode=false, path_file=刚录的文件 → 重编译 → 起 px4_path（path_follow_node）
④ 起 px4_control + px4_teleop（真 TTY）
⑤ 按 0（Offboard）→ 攒够 20 心跳 → 按 1（ARM + 起飞）→ 到 takeoff_height 进 HOVER
⑥ 按 4（GOTO 使能）→ 跟随器 start_trajectory → 20Hz 发 GotoTarget → 飞机沿路径回放
⑦ 按 5 → PAUSED 冻结；再按 4 → 从冻结处续跑
⑧ 到终点 → HOLD 持续重发终点（防看门狗超时）
⑨ 按 3 → LAND
```

---

## §5 配置键汇总

### 5.1 px4_control/config/params.yaml（23 行）

| 键 | 值 | 代码默认 | 说明 |
|---|---|---|---|
| `takeoff_height` | 0.5 | 1.2 | ⚠️ YAML 把代码默认压到 0.5（贴近地面） |
| `step_xy` / `step_z` | 0.5 / 0.1 | 同 | WASDQE 步进 |
| `loop_rate_hz` | 20 | 50 | Offboard 心跳/设定值频率 |
| `arrive_tol` | 0.15 | 0.15 | 起飞到位判据 |
| `heartbeat_before_offboard` | 20 | 20 | 按 0 前需已发够心跳 |
| `land_control` | false | false | false=自主下降，true=NAV_LAND |
| `land_fast_speed` / `land_slow_speed` | 0.4 / 0.2 | 同 | 降落分段速度 |
| `land_slow_alt` | 0.5 | 同 | 切换慢速 AGL 阈值 |
| `land_touch_z` | 0.05 | 同 | 触地判据 |
| `yaw_step_rad` | 0.15 | 0.15 | L/R 偏航 |
| `pursuit_goto_timeout` | 1.0 | 1.0 | Goto 看门狗 |

### 5.2 px4_path/config/params.yaml（159 行）

**px4_path_launch**：`record_mode: false`（**无同名 launch 参数**，改了要重编译）。

**path_follow_node**（公共 + trajectory）：

| 键 | 值 | 代码默认(wp/traj) | 说明 |
|---|---|---|---|
| `follow_mode` | waypoint | trajectory / (n/a) | **文档默认值与 YAML 相反** |
| `path_dir` | `/home/duzhong/dzpx4/src/px4_path/src/path` | `""` | ⚠️ 硬编码；空=拒绝 |
| `path_file` | `recorded_path_2.txt` ⚠️ | recorded_path.txt | ⚠️ 与录制段不一致 |
| `pursuit_rate` | 20.0 | 10.0 / 50.0 | GotoTarget 频率 |
| `switch_tol` | 0.10 | 0.30 | 仅 waypoint |
| `terminal_tol` | 0.15 | 0.20 | 进 HOLD |
| `max_step_dist` | 1.5 | 1.5 | waypoint 单步 / traj 连接段步长 |
| `max_neighbor_dist_m` | 5.0 | 5.0 | 加载相邻点硬门限 |
| `max_speed_mps` | 0.8 | 0.5 | trajectory 参考速度 |
| `max_accel_mps2` | 1.0 | 0.5 | trajectory 加速度 |
| `max_jerk_mps3` | 2.5 | 1.0 | 只测量，不发布 |
| `max_centripetal_accel_mps2` | 0.35 | 0.35 | 弯道降速 |
| `profile_resolution_m` | 0.12 | 0.10 | 曲率采样间隔 |
| `segment_split_turn_deg` | 140.0 | 150.0 | 急弯拆段（度） |
| `segment_switch_speed_mps` | 0.10 | 0.15 | 切段前速度上限 |
| `tracking_slowdown_error_m` | 1.00 | 0.30 | 降参考时钟 |
| `tracking_pause_error_m` | 1.50 | 0.70 | 冻结参考时钟 |
| `max_time_scale_rate_per_s` | 1.2 | 1.0 | 缩放变化率上限 |
| `max_tracking_error_m` | 5.0 | 1.0 | 超限中止（≤0 关闭） |
| `yaw_mode` | fixed | fixed | fixed/recorded/tangent |
| `yaw_lookahead_s` | 0.5 | 0.5 | 仅 tangent |
| `max_yaw_rate_rps` | 0.5 | 0.5 | 航向发布变化率 |

**path_record_node**：

| 键 | 值 | 代码默认 | 说明 |
|---|---|---|---|
| `slam_topic` | `/localization` | 同 | ⚠️ 该话题不存在 |
| `expected_frame_id` | `""` | 同 | 空=用首帧 |
| `visualization_topic` | `/path_record/visualization` | 同 | RViz |
| `path_dir` | `/home/duzhong/.../src/path` | `""` | ⚠️ 必须与跟随段一致 |
| `output_file` | `recorded_path.txt` | 同 | ⚠️ 与跟随段不一致 |
| `overwrite_existing` | false | **true** | ⚠️ 自动改名 _1/_2/_3 |
| `enable_filter_smoothing` | true | 同 | 滑动均值+平滑+压缩 |
| `min_xy_m` / `min_z_m` | 0.12 / 0.10 | 0.10 / 0.10 | 关键点阈值 |
| `max_neighbor_dist_m` | 5.0 | 同 | 跳变门限 |
| `min_record_interval_s` | 0.05 | 0.10 | 时间限频 |
| `filter_window` | 5 | 5 | 在线均值窗口 1~101 |
| `smoothing_window` | 3 | 3 | 保存前平滑窗口 |
| `min_smoothed_point_dist_m` | 0.05 | 0.05 | 近点压缩阈值 |
| `min_points` | 2 | 2 | 最少点数 |

### 5.3 px4_odom_convert launch 参数

| 键 | 默认 | 说明 |
|---|---|---|
| `mode` | `localization`（代码 `odom`） | ⚠️ 补丁 01 改 `odom` |
| `odom_topic` | `/Odometry` | odom 模式订阅 |
| `localization_topic` | `/localization` | localization 模式订阅 |
| `tf_map_frame` / `tf_lidar_frame` | `map` / `lidar` | tf 模式 |
| `z_offset_m` | 0.0 | 雷达安装高度补偿 |
| `loop_rate_hz` | 50.0 | tf 模式查询频率 |
| `enable_auto_align` | true | 自动对齐 |
| `align_yaw_to_ekf` | false | 是否旋转对齐 EKF 航向 |
| `slam_insane_m` | 2.0 | L1 熔断门限 |
| `slam_recover_n` | 86 | L1 恢复帧数 |
| `bump_min_interval_s` | 10.0 | L2 bump 间隔 |
| `ekf_storm_window_s` | 60.0 | L3 滑动窗 |
| `ekf_storm_max_resets` | 5 | L3 风暴阈值 |
| `max_position_jump_m` | 0.5（launch） | ⚠️ **幽灵参数**，节点未声明 |
| `max_attitude_jump_rad` | 0.5（launch） | ⚠️ **幽灵参数**，节点未声明 |

### 5.4 dzlink_supervisor.py 参数

| 键 | 默认 | 说明 |
|---|---|---|
| `--perception-glob` / `--perception-file` | — | 文件尾随输入 |
| `--perception-topic` | — | ROS 话题输入 |
| `--perception-udp` / `--perception-udp-host` | — / 0.0.0.0 | UDP JSON 输入 |
| `--cmd-file` | — | 命令文件（JSONL 追加即生效） |
| `--cmd-from-start` | false | 命令文件从头读 |
| `--perception-from-start` | false | 感知文件从头读 |
| `--stdin` | false | 开启 JSONL stdin |
| `--heartbeat-hz` | 1.0 | 心跳频率 |
| `--bridge-period` | 2.0 | 桥洞重发周期 |
| `--target-period` / `--field-period` | 0.0（变了才发） | 靶区/雷障周期 |
| `--info-ttl` | 30.0 | 情报有效期 |
| `--intel-repeat` / `--route-repeat` | 3 / 3 | 重复发送遍数 |
| `--loop-hz` | 20.0 | 主循环频率 |
| `--status-period` | 30.0 | 状态日志周期 |
| `--bridge-left-halves` | lower | 反转方向词映射 |
| `--domain` / `--no-domain-override` | 118 / — | 域号（默认读 env 文件） |
| `--duration` | 0（常驻） | 运行时长 |
| `--log-file` | — | 日志路径 |
| `--echo-car` | true | 回显车消息 |

### 5.5 dz_send_waypoint.py 参数

| 键 | 默认 | 说明 |
|---|---|---|
| `--repeat` | 3 | 重复发送遍数 |
| `--period` | 0.5 | 遍间隔秒 |
| `--wait-match` | 3.0 | 等 DDS 匹配秒数 |
| `--dry-run` | false | 只打印不发 |
| `--confidence` | 0.9 | 桥洞置信度 |
| `--seq` | None | 带上 seq 会车回 ack |
| `--duration` / `--hz` | 30.0 / 1.0 | heartbeat/listen 时长与频率 |
| `--blocks` | — | target 别名 |
| `--mines` / `--obstacles` | — | field 子命令 |

### 5.6 FAST-LIO 生效配置（mid360_test.yaml）

| 键 | 值 | 行 | 说明 |
|---|---|---|---|
| `lid_topic` / `imu_topic` | `/livox/lidar` / `/livox/imu` | 36–37 | 与驱动一致 |
| `lidar_type` | 1（AVIA，MID-360 兼容） | 38 | 走 CustomMsg 分支 |
| `scan_line` | 4 | 39 | MID-360 等效线数 |
| `blind` | 0.8 | 40 | ⚠️ 0.8m 内点全丢 |
| `det_range` | 40.0 | 50 | 10% 反射率量程 |
| `extrinsic_T` | `[-0.011,-0.02329,0.04412]` | 52 | 与 Livox 出厂标定吻合 |
| `filter_size_surf/map` | 0.18 | 24–25 | 体素 |
| `map_file_path` | `/home/duzhong/dzpx4/src/maps/output_voxel_0.05.pcd` | 16, 29 | ⚠️ 双份配置 + 绝对路径 |
| `/global_localization.localization_th` | 0.85 | 6 | ⚠️ ICP 判据偏高 |
| `/global_localization.scan/map_voxel_size` | 0.18 | 3–4 | 与 filter 一致 |
| `/global_localization.acc/gyr_cov` | 0.8 | 45–46 | 比标准值大 8 倍 |
| `pcd_save_en` | false | 66 | 定位模式不保存 |
| `/transform_fusion.freq_pub_localization` | 50.0 | 10–12 | ⚠️ **死参数**（节点未声明） |

### 5.7 dzuav 参数

| 参数/环境变量 | 值 | 说明 |
|---|---|---|
| `argc<4` 用法 | `dzuav <image 或 video> <hbb_engine> <cell_engine> [udp_port] [output_dir]` | 离线模式 |
| `argus:<sensor-id>` / `uvc:<device>` | — | 相机源 |
| UDP 端口 | 50000（默认） | 车端约定 |
| 分辨率/帧率 | `1920x1080@30`（默认） | USB MJPEG |
| `DZUAV_PREVIEW=1` | — | 开实时预览窗口 |
| `DZUAV_FORCE_UDP` | — | 定位失败也广播 |
| `HbbDetector(conf=0.25, nms=0.45)` | — | 检测阈值 |
| `CellClassifier` 调用阈值 | 0.45（主流程）/ 0.55（文档） | ⚠️ 文档与代码不一致 |

### 5.8 车侧契约（跨机必读）

| 参数 | 值 | 位置 |
|---|---|---|
| `drone.info_stale_s` | 30.0 | `robot.yaml` |
| `mission.bridge_info_wait_s` | **2.0**（旧 20） | `robot.yaml:546` |
| `mission.auto_start_delay_s` | **45.0**（旧 10） | `robot.yaml:525` |
| `route.cross_lower_default` | `K10` | `robot.yaml:520` |
| `route.cross_upper_default` | `K5` | `robot.yaml:521` |
| `drone.heartbeat_offline_s` | 3.0 | `robot.yaml` |
| 车侧 car_state 周期 | 0.7s | `dzlink_supervisor.py:120` |

## §6 测试与证据

### 6.1 本机纯逻辑测试（不需要 ROS/硬件）

| 测试套件 | 命令 | 覆盖 | 证据 |
|---|---|---|---|
| link 单测 | `python -m unittest discover -s dzsmartscout/tools/link/tests -p "test_*.py"` | 70 项：心跳节律 / 情报翻译与时效 / 报文契约字段 / 路线形状校验 / ack 台账 / 文件尾随（半行、截断）/ 命令文件 / UDP / 端到端（感知文件→报文→模拟 ack） | `tools/link/README.md:207-214`、`tools/link/tests/test_dzlink_supervisor.py`、`test_dz_send_waypoint.py` |
| link selftest | `python dzlink_supervisor.py selftest` | 精简版纯逻辑自检 | 同上 |
| perception 自检 | `python -m perception.cli selftest` | 66 项（不含最慢的 e2e） | `tools/perception/README.md:10-12` |
| perception e2e | `python -m perception.cli selftest --with-e2e` | 72 项 | 同上 |
| perception 全量 | `cd dzsmartscout/tools && python -m perception.run_tests` | 99 项（t5a 前 85 + 覆盖/负证据 14） | `tools/perception/CAPABILITY_L1.md:6`、`tools/perception/run_tests.py` |
| 官方贴图出图 | `python -m perception.run_field_map_check --all` | mine 8/8、obstacle 8/8、target 2/2、bridge 2/2，P=R=1.000 | `tools/perception/reports/field_map_check.md`、`field_map_official.{json,yaml}` |
| 真图核查 | `python -m perception.run_real_checks` | 图纸回归 + cam_probe + 实物箭头 | `tools/perception/reports/real_checks.md/json` |
| 桥洞箭头 | `python -m perception.cli arrow --image p09_1.png` | 实物 −96° vs 真值 −90°（误差 6°） | `tools/perception/CAPABILITY_L1.md:18` |
| 车位姿合成 | `python -m perception.cli synth` | 50 组配置格名 100% 正确；方向误差 ≤2.07° | `tools/perception/README.md:121-131` |
| 坐标链路验证 | `python fc-audit/verify_frames.py` | 2000 组随机位姿验证 `px4_odom_convert` 与 `path_loader` 坐标链路逐项一致 | `fc-audit/verify_frames.py`、`notes/px4_path.md §6.2` |
| dzuav 编译验证 | `bash tools/orin_build_verify.sh --run ~/engines ~/test/field.jpg` | L0 环境 / L1 编译 / L2 二进制自检 / L3 真实推理 + UDP 协议闭环（车端 parse） | `tools/orin_build_verify.sh` |

### 6.2 真机验证记录（历史，本轮未在线复核）

| 项 | 结果 | 证据 |
|---|---|---|
| 雷达驱动 | ✅ `/livox/lidar` 10.000 Hz、19968 点/帧；`/livox/imu` 200.9 Hz | 2026-10-06 实测（`fc-audit/启动手册.md:24-25`） |
| FAST-LIO | ✅ imu=200.0 Hz、replay≈165 Hz、odom=43~45 Hz、invalid_gap=0 | 同上 |
| `/localization` | ❌ 话题不存在（3 个 Python 节点全死） | `启动手册.md:26-27,44-72` |
| `/Odometry` | ✅ 44.1 Hz | `启动手册.md:28` |
| `px4_odom_convert mode:=odom` | ✅ `/fmu/in/vehicle_visual_odometry` 42.4 Hz | `启动手册.md:29` |
| PX4 飞控板 | ❌ 无 `/dev/ttyACM*`、`lsusb` 无 FMU | `启动手册.md:30` |
| 无人机→小车 | ✅ 车侧订阅数=2（硬证据），`/nav/waypoint` 双向通 | 2026-10-07 20:08（`无人机技术文档.md §7.4`） |
| 相机点亮 | ✅ `/image_raw` 29.245 Hz，640×480 rgb8，JPEG 头 `ff d8 ff` | 2026-10-07（`04-camera-bringup/APPLY.md`） |
| 相机画质 | ❌ 拉普拉斯方差 46.2（糊区间），对焦控制 no-op | `无人机技术文档.md §8` |
| 补丁 06（车侧待命） | ✅ 起栈 35 s 未出发、发 2 收 2、回滚逐字节还原 | `06-car-wait-for-drone/APPLY.md` |

### 6.3 证据目录索引

| 目录 | 内容 |
|---|---|
| `tools/reports/t4-evidence/` | QoS 矩阵、链路验证报告（t4） |
| `tools/reports/t12-evidence/` | 发送器/监听端到端证据（t12） |
| `tools/reports/t16-evidence/` | 桥洞成对发送实测（三种注入场景，相邻报文最大间隔 1.13 s） |
| `tools/reports/t18-evidence/` | 部署哈希、复核整改说明（t18） |
| `tools/perception/reports/` | 出图/核查/覆盖报告与叠加图 |
| `fc-audit/arena_pdf_img/` | 厂商手册提取的场地图 |

### 6.4 未验证清单（不能当结论用）

> 详细见 §7。

| # | 未验证项 | 怎么验 |
|---|---|---|
| 1 | 机上 `params.yaml` 与本地副本是否一致 | 登机 `md5sum ~/dzpx4/src/px4_path/config/params.yaml` 对比 |
| 2 | 是否 `--symlink-install` 工作空间 | `ls -l install/*/share/*/config/*.yaml` 看是否软链 |
| 3 | `pursuit_goto_timeout=1.0s` 是否够（样条构建耗时） | 打点测 `start_trajectory()` 耗时 |
| 4 | `pursuit_rate > 1000` 时 0ms 定时器行为 | 查 rclcpp 版本行为 |
| 5 | PX4 对**显式 0 前馈**的处理 | 真机飞（需飞控板） |
| 6 | 白色地垫上的 FAST-LIO 漂移 | §2.8.6 实验 3 |
| 7 | PX4 固件版本与 `px4_msgs 2.0.1` 是否匹配 | 飞控板上电后查 `px4_firmware_version` |
| 8 | 机上能否访问 PyPI（决定 `open3d` 能否装） | 登机 `pip3 download --no-deps open3d -d /tmp/x` |
| 9 | dzuav 全流程（相机→检测→PnP→UDP） | 无人机到货后按 §2.5.10 顺序联调 |
| 10 | `max_tracking_error_m` 中止门是否曾经触发 | 需机上日志 |
| 11 | 机器人侧是否还有 `recorded_path_3.txt` | 本次只读本地镜像，未 SSH |

---

## §7 未核实清单

> 原则：不确定一律写「未核实」，不许凭记忆编造。下列条目本轮**未能确认**，引用时必须标注。

### 7.1 本轮未能在线复核的机上事实

| 项 | 说明 |
|---|---|
| 机上 `~/dzpx4/src/` 与本地 `fc-audit/src-mirror/` 是否仍逐文件一致 | 历史记录（2026-10-08 前后）说一致；本轮未 SSH，未复核 |
| 机上 `config/params.yaml` 实际值 | 本地副本是 `fc-audit` 镜像；机上可能已被补丁或手改 |
| 机上是否已存在 `recorded_path_3.txt` | 补丁 02 的 APPLY 说「未确认现场当前是否已存在」 |
| 机上 `px4_odom_convert` 当前是代码默认 `odom` 还是 launch 覆盖的 `localization` | 取决于补丁 01 是否已应用（记录为「未应用」） |
| 机上 `follow_mode` 是否仍是 `waypoint` | 补丁 02 明说「不动这个值」；但未复核 |
| 飞控板是否已插上 | 历史记录「从未连接」；本轮未复核 |
| `px4_msgs 2.0.1` 与机上 PX4 固件是否匹配 | 机上无固件版本证据 |

### 7.2 代码层面未能确认的细节

| # | 项 | 说明 |
|---|---|---|
| U-1 | `SupervisorNode._on_perception` 中 `return` 的可达性（N-01） | 第 1283-1286 行：`if obj is None: diag(...); return`，紧接着 `self.core.ingest_sample(...)` 在 return 之后**不可达**。这段代码路径在感知话题收到非 JSON 时会打 diag 后 return（正常）；但 `ingest_sample` 那行永远不执行——**这是缺陷还是笔误需人工确认**（按现状：感知话题输入永远不会被处理）。 |
| U-2 | PX4 对「显式 0 速度/加速度前馈」的处理 | `px4_control_node.cpp:505-509` 的注释只代表作者当时结论；无 PX4 固件源码在审计范围 |
| U-3 | rclcpp 对 `0 ms` 周期定时器的行为 | 取决于 rclcpp 版本；范围外 |
| U-4 | `MotionTrajectory` 拉伸后是否真落在限值内 | 数学推导正确，但代码未断言（N-03） |
| U-5 | `dz_send_waypoint.py` 是否读 `dzlink.env` | 文档说「域号仍由调用方内联」；代码层面 `grep dzlink.env` 需确认 |
| U-6 | 感知侧（dzuav）输出是否已能转成 `dzlink_supervisor` 吃的 JSON | supervisor 明确**不解析私有 UDP 帧**；需一个薄适配器，当前状态未核实 |
| U-7 | 车侧当前部署是否已有 dzbrain | 补丁 05 更正：车上是旧核 path_executor，没有 dzbrain；是否已升级未核实 |
| U-8 | `follow_mode: waypoint` 时 R-02/R-06 是否生效 | 补丁 03 说「只要 follow_mode 保持 waypoint，R-02 不生效（waypoint 有 clamp_step 兜底）」；但 `waypoint` 仍有 R-07 前馈矛盾 |
| U-9 | 官方 FAST-LIO 的 3 个死节点是否已装好依赖 | 补丁区说「装包由用户执行」；本轮未核实 |
| U-10 | 机上 `install/` 是否是 symlink-install | 决定补丁 01/02/03 是否需要编译；未核实 |

### 7.3 第三方依赖边界（只说明存在，不审计）

| 包 | 边界 |
|---|---|
| `livox_ros_driver2` | 只读配置生效（`msg_MID360_launch.py`）；`xfer_format` 不可改 |
| `FAST_LIO_LOCALIZATION_ROS2` | 只审计生效 config 与 4 节点死亡原因；算法细节（ikd-Tree、IEKF）不逐行审计 |
| `px4_msgs` / `px4_ros_com` / `mavros` | 只记录版本与所用接口 |
| `ego-planner` / `hba` / `FASTLIO2_ROS2` | 仅确认未接入 |
| `gst_camera` | 空壳（4 目录 0 文件） |

### 7.4 审计方法与限制

- 方式：**纯静态阅读**（未 SSH、未执行机上命令、未改动任何源码/配置）。
- 覆盖：自研 C++ 包 5 个、dzuav、perception、link、fc-audit、launch/config/脚本，共约 40 个源文件 + 30 份文档。
- 未做：未连无人机、未连小车、未编译、未跑机上测试。
- 唯一写入目标：本文档。未修改、未创建其它任何文件。

---

## 附：快速命令速查

```bash
# —— 环境（每个新终端）——
source /opt/ros/humble/setup.bash
source ~/dzpx4/install/setup.bash
source ~/dzlink_tools/dzlink.env          # 域号唯一真相源 = 118

# —— 感知层 ——
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch fast_lio_localization velodyne_localization.launch.py rviz:=false

# —— 飞控链路（需飞控板）——
MicroXRCEAgent serial --dev /dev/ttyACM0 -b 921600
ros2 launch px4_odom_convert px4_odom_convert.launch.py mode:=odom   # ★ 必须 mode:=odom
ros2 launch px4_imu_bridge sensor_combined_to_imu.launch.py

# —— 验证 ——
ros2 topic list | grep fmu
ros2 topic hz /fmu/in/vehicle_visual_odometry

# —— 控制层 ——
ros2 launch px4_control px4_control.launch.py
ros2 run px4_control px4_teleop_node --params-file ~/dzpx4/install/px4_control/share/px4_control/config/params.yaml
ros2 launch px4_path px4_path.launch.py

# —— 向小车发航点/情报（不依赖飞控板）——
python3 ~/dzlink_tools/dz_send_waypoint.py route "N10 M10 L10 L9 K9 J9 J10 I10 H10 G10* F10 F11 E11 N5"
python3 ~/dzlink_tools/dz_send_waypoint.py bridge lower=K9 upper=K4 --seq 1
python3 ~/dzlink_tools/dz_send_waypoint.py heartbeat --duration 120
python3 ~/dzlink_tools/dz_send_waypoint.py target G10 H10
python3 ~/dzlink_tools/dz_send_waypoint.py field --mines A3 B4 --obstacles K9
python3 ~/dzlink_tools/dz_send_waypoint.py listen --duration 30

# —— 停止 ——
pkill -f livox_ros_driver2_node
pkill -f fastlio_mapping
pkill -f px4_odom_convert
pkill -f sensor_combined_to_imu
pkill -f px4_control_node
pkill -f px4_teleop_node
pkill -f path_follow_node
pkill -f path_record_node
pkill -f MicroXRCEAgent
```

> ⚠️ `path_record_node` 必须 `Ctrl+C` 正常退出才落盘；`px4_teleop_node` 必须在真 TTY。

---

*本文档为「无人机 dzsmartscout」技术全景的唯一写入目标，基于 2026-10-23 代码级审计。所有结论附证据；未核实项集中在 §7。*





