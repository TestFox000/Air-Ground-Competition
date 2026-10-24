---
来源: 本次代码级技术审计（2026-10-23，子智能体）
取证: 实
更新: 2026-10-23
status: 现行
---

# 技术全景 C · 车端硬件与配置

> 赛事：全球校园人工智能算法精英大赛 · 空地协同侦排挑战赛
> 范围：dzrobot_v2 车端——硬件链路、串口协议、定位与循迹、云台 PID、开火链路、配置中心、部署对账、冻结区边界。
> 方法：本报告所有结论均来自本地代码逐行精读，每条给出 路径:行号。凡未能在源码中确证者，一律标注「未核实」。本报告只读审计，未修改任何源码或配置。

---

## §0 导读

### 0.1 一句话结论

车端是一个 STM32 下位机 + RK/Jetson 类上位机 + ROS 2 Humble 的阿克曼小车：上位机（dzrobot_v2）通过两个串口与下位机通信——**底盘串口** `/dev/dzactuator`（ttyUSB0，115200）与**裁判/激光串口** `/dev/dzjudgment`（ttyACM1，115200）——完成「识别 → 云台伺服 → 开火」闭环，并通过 serial_reader → path_executor 的轮式死推算完成场地定位与循迹。

### 0.2 阅读顺序

| 想看什么 | 直接跳 |
|---|---|
| 硬件接线、两个串口的帧格式 | §2.2（底盘）与 §2.4（裁判） |
| 云台到底怎么转、有哪几套 PID | §2.3，重点 2.3.3 / 2.3.4 |
| 什么时候开火、弹药怎么算 | §2.3.5 与 §2.1.3（循迹停站） |
| 配置中心改什么、什么不能改 | §2.6 与 §5 |
| 上车部署怎么对账、冻结区边界 | §2.7 与 §2.7.6 |
| 想找某个函数在哪个文件哪一行 | §3 索引大表 |
| 已知 bug / 坑 | §2.8（汇总） |
| 没查清楚的事 | §7 |

### 0.3 证据与口径声明

- 所有行号以本次审计时刻的工作区快照为准（工作区根 `D:\\User\\Desktop\\比赛存档\\v1`）。
- 代码中大量注释带有踩坑说明，本报告保留其关键结论并标注出处行号；这些注释是作者自述，属于证据而非不可挑战的真理。凡涉及实测/现场的判断，若本报告未能独立复核，均在 §7 标注「未核实」。
- `config/**`、`src/dzrknn_dataprocess/**`、`tools/deploy_to_car.py`、`src/dzbrain/**` 属**冻结区**（`tools/wf/frozen.json:3-8`），本报告只读不改。

---

## §1 模块地图

### 1.1 目录结构与角色

    dzrobot_v2/
    ├── config/
    │   ├── robot.yaml           # 全车唯一参数真源（968 行，7 个节点段）
    │   ├── map.yaml             # 场地要素（雷/障/靶/通/起终）——当前第六张图
    │   ├── map_v3.yaml          # 第四张图（历史备份，见 §2.6.4）
    │   └── robot.yaml.*bak      # 多份回滚备份（t39 / yawfloor / corrupt_20261017）
    ├── dzros.env                # ROS 域号唯一真源（DZ_ROS_DOMAIN=118）
    ├── start_all.sh             # 一键起栈 / 停 / 重启（7 步）
    ├── src/
    │   ├── car_localization/    # 定位 + 循迹（Python）
    │   │   ├── generate_path.py       # 生成测试路径文件（纯工具）
    │   │   ├── path_executor.py       # 循迹单体（2622 行，LEGACY_PATH=1 时启用）
    │   │   ├── serial_reader.py       # 死推算定位（block 单位）
    │   │   ├── send_waypoint.py       # 发一个航点命令（一次性）
    │   │   └── launch/localization.launch.py
    │   ├── dzactuator/           # 底盘串口 + TF + IMU（C++）
    │   │   ├── src/dzactuator.cpp     # 1447 行：串口帧打包/解析、100Hz 控制环
    │   │   ├── src/Quaternion_Solution.cpp  # IMU 四元数解算 + 滤波
    │   │   ├── include/dzactuator.h   # 结构体 / 宏 / 参数声明
    │   │   ├── launch/bringup.launch.py     # 底盘 + static TF
    │   │   ├── launch/camera.launch.py      # usb_cam（respawn）
    │   │   ├── launch/judgment.launch.py    # 裁判启动（与 dzjudgment 包重复，见 §2.8）
    │   │   ├── launch/bringup_all.launch.py # 全栈（含 auto_start，勿用）
    │   │   ├── launch_auto/auto_sl_s3e.launch.py  # AMCL 栈（历史，引用已不存在的 launch）
    │   │   └── params/*.yaml           # amcl / amcl / imu / bmi / mapping 等
    │   ├── dzrknn_dataprocess/  # 云台自瞄 + 开火判定（C++）
    │   │   ├── src/dzrknn_dataprocess.cpp  # 1725 行：5 套 PID 口径、扫描状态机、开火链
    │   │   ├── include/dzrknn_dataprocess.h  # 成员/默认值/枚举
    │   │   ├── launch/dzrknn_dataprocess.launch.py  # executable=dzactuator_node
    │   │   ├── offcenter_table.txt       # 视差查表（当前全 0）
    │   │   └── CMakeLists.txt
    │   ├── dzjudgment/           # 裁判串口 + 激光开火（C++）
    │   │   ├── src/judgment.cpp         # 401 行：串口帧 B5 5B … + 状态上报
    │   │   ├── src/dzjudgment.cpp       # main
    │   │   └── launch/judgment.launch.py
    │   ├── rknn_pt/              # RKNN 视觉检测 det_node（C++）
    │   │   ├── src/det_node.cc          # 订阅 /image_raw → /offset_center
    │   │   ├── src/det/{rkpt,preprocess,postprocess}.cc
    │   │   ├── include/{rknnPool,ThreadPool,coreNum}.hpp
    │   │   ├── include/det/{rkpt,preprocess,postprocess}.h
    │   │   ├── include/common.h         # DetectionBox / DetectResultsGroup
    │   │   ├── model/light_det.rknn     # YOLOv8-pose（蓝/红靶）
    │   │   ├── lib/{librknnrt,librga}.so
    │   │   └── launch/det_node.launch.py
    │   └── dzbrain/              # 新决策层（本报告不展开，见 §1.4）
    ├── tools/
    │   ├── deploy_to_car.py      # 唯一部署入口（SSH/SCP + 哈希对账）
    │   ├── restart_node.sh       # 单节点重启（8 个槽位）
    │   ├── healthcheck.sh        # 出车体检（6 节）
    │   ├── config_selfcheck.py   # 配置/路线门禁（部署 verify 已不再调用）
    │   └── wf/{check,check_reuse,check_links,repo_index}.py + frozen.json
    └── install/                  # colcon 产物（车上的已安装副本）

### 1.2 话题 / 串口 / 进程总览

| 层 | 名称 | 类型 | 生产者 | 消费者 | 备注 |
|---|---|---|---|---|---|
| 串口 | `/dev/dzactuator` (ttyUSB0) | 23B 下行 / 34B 上行 | dzactuator | STM32 | 波特 115200；§2.2.2 |
| 串口 | `/dev/dzjudgment` (ttyACM1) | 6B 下行 `B5 5B 03 21 01 25` | dzjudgment | 激光/裁判模块 | §2.4.2 |
| 相机 | `/image_raw` | sensor_msgs/Image | usb_cam_node | det_node | 640x480 yuyv 30fps |
| 视觉 | `/offset_center` | Int32MultiArray[8] | det_node | dzrknn_dataprocess | §2.5.2 |
| 视觉 | `/imu_data` | sensor_msgs/Imu | dzactuator | serial_reader、dzrknn_dataprocess | 滤波+零偏 |
| 里程 | `/odom` | nav_msgs/Odometry | dzactuator | serial_reader | 原始死推算 |
| 里程 | `/odom/localization` | nav_msgs/Odometry | serial_reader | path_executor / dzbrain | block 单位 |
| 控制 | `/pursuitAngle` | geometry_msgs/Twist | path_executor | dzactuator | linear.x=m/s, angular.z=舵角 deg |
| 控制 | `/cmd_vel` | geometry_msgs/Twist | dzbrain | dzactuator | DWA 模式（current_mode_==1） |
| 云台 | `/CameMo_Conl` | Int32MultiArray[4] | dzrknn_dataprocess | dzactuator | Position0/1, Speed0/1 |
| 反馈 | `/camemo_feedback` | Int32MultiArray[6] | dzactuator | dzrknn_dataprocess | 位置/速度/负载 |
| 云台 | `/aim/status` | Int32 (0/1/2) | dzrknn_dataprocess | path_executor | 网页端曾按 UInt8 订阅，已下线 shim（§2.8） |
| 云台 | `/nav/scan_request` | UInt8 | path_executor | dzrknn_dataprocess | 1=单扫 / 2=持续扫 / 0=停 |
| 云台 | `/nav/manual_gimbal` | UInt8 | 遥控器 | dzrknn_dataprocess | 1=接管 |
| 云台 | `/nav/manual_override` | UInt8 | 遥控器 | path_executor | 1=循迹闭嘴 |
| 云台 | `/imu_data` | Imu | dzactuator | dzrknn_dataprocess | 车体转动前馈 |
| 开火 | `/LaserShot_Command` | UInt8 | dzrknn_dataprocess | dzjudgment | 1=开火 |
| 开火 | `/aim/fire_gate` / `/aim/fire_ready` | UInt8 | dzbrain | dzrknn_dataprocess | 两键开火（默认开） |
| 开火 | `/aim/lead_x` | Float32 | dzbrain | dzrknn_dataprocess | 提前量 px |
| 弹药 | `/shoot_count` | Int32 | dzrknn_dataprocess | path_executor | 累计发数 |
| 弹药 | `/shoot_reset` | UInt8 | 网页/工具 | dzrknn_dataprocess | 重置计数 |
| 激光 | `/laser_switch` | UInt8 | 遥控器 | dzactuator | buf[7] 常亮开关 |
| 语音 | `/voice_switch` | UInt8 | dzrknn_dataprocess / path_executor | dzactuator | 播报 ID |
| 导航 | `/nav/waypoint` | String | send_waypoint.py / dzbrain | path_executor + serial_reader | 路线 / reset |
| 状态 | `/move_base/stop_signal` | UInt8 | path_executor | dzbrain（旧） | 1=允许射击 |
| 运动 | `/nav/motion_state` | UInt8 | path_executor | 浏览器 | 0=前进 1=倒车 |

### 1.3 启动时序（start_all.sh）

start_all.sh 共 7 步，每步 sleep 3（云台 2s），用 setsid 让每个节点自成进程组：

| 步 | 日志名 | 命令 | 说明 |
|---|---|---|---|
| 1/7 | 1_chassis | `ros2 launch dzactuator bringup.launch.py` | 底盘串口 + TF（dzactuator.cpp:1078） |
| 2/7 | 2_camera | `ros2 launch dzactuator camera.launch.py` | usb_cam，respawn=True |
| 3/7 | 3_localization | `ros2 launch car_localization localization.launch.py` | serial_reader → /odom/localization |
| 4/7 | 4_det | `ros2 launch det_node det_node.launch.py` | DISPLAY=$GUI_DISPLAY（默认 :1 VNC） |
| 5/7 | 5_gimbal | `ros2 launch dzrknn_dataprocess dzrknn_dataprocess.launch.py` | 云台自瞄 |
| 6/7 | 6_judgment | `ros2 launch dzjudgment judgment.launch.py` | 射击驱动，缺了激光永不亮 |
| 7/7 | 7_path | `python3 -u -m dzbrain.main … --force` | 新决策层；LEGACY_PATH=1 回退 path_executor_node |

证据：`dzrobot_v2/start_all.sh:168-221`。第 8 步（浏览器控制台 cam_viewer）已于 2026-10-09 下线（`start_all.sh:224-232`）。

### 1.4 与 dzbrain 的关系（边界）

`start_all.sh:205-221` 默认走**新决策层 dzbrain**，旧单体 `path_executor.py`（2622 行）只在 `LEGACY_PATH=1` 时启用。两者共用 `config/robot.yaml` 的 path_executor 段与 map.yaml。本报告按任务范围覆盖 car_localization（旧核）与车端硬件包；dzbrain 的 8 任务线程内部不在本次审计范围，其关键键在 §5 中一并列出以便对照。

## §2 逐模块详解

### 2.1 car_localization（定位 + 循迹）

### 2.1.1 坐标系与场地模型

场地为 **14 列 × 11 行** 网格，每格 **0.35 m**。原点在 **N11 格右下角**。

- 列字母 `A…N`（A 最左/西，N 最右/东），列索引 0..13。
- X 轴正向：行 11 → 行 1（下→上）；Y 轴正向：列 N → 列 A（右→左）。
- 起点智能车 `N10`，朝向 **Y+**；终点 `N5`。

换算公式（三处实现必须一致）：

    x = (10.5 - row_idx) * 0.35     # row 11 → 0.175 m，row 1 → 3.675 m
    y = (13.5 - col_idx) * 0.35     # col N → 0.175 m，col A → 4.725 m

证据：
- `path_executor.py:106-116`（常量）
- `path_executor.py:501-526` `block_to_meters()`
- `serial_reader.py:38-48`（常量与 `HEADING_YAWS`）
- `generate_path.py:17-32` `block_to_meters()`

> ⚠ **潜在不一致（未核实）**：`generate_path.py:22` 的列字母表是 `'ABCDEFGHIJKLMN'`（14 个），`get_block_name` 在 `serial_reader.py:68-75` 用 `round(10.5 - x)` / `round(13.5 - y)`，而 `path_executor.py` 用 int 语义。本报告未逐格穷举验证边界格，仅确认三者坐标公式一致。

### 2.1.2 generate_path.py（路径生成工具）

- `block_to_meters(block_name)`：`generate_path.py:20-32`，与 §2.1.1 同公式。
- `yaw_to_quaternion(yaw)`：`generate_path.py:35-39`，只算 `qz=sin(yaw/2)`、`qw=cos(yaw/2)`。
- `generate_straight_path(...)`：`generate_path.py:42-64`，按 step（默认 0.05 m）生成直线点。
- `generate_waypoint_path(start_block, waypoints, step=0.05)`：`generate_path.py:67-106`，多点折线；输出格式 `x,y,z,qx,qy,qz,qw,`（**每行末尾带逗号**，`generate_path.py:62,101`）。

> ⚠ **坑**：输出行末尾多一个逗号，而 `path_executor.load_path()` 用 `line.split(',')` 且要求 `len(parts) >= 7`（`path_executor.py:919-920`）。末尾逗号会产生第 8 个空字段，`len(parts)==8` 仍可通过，但 `parts[7]` 为空串、未被使用——**当前能工作但格式不干净**（未核实是否有下游依赖精确 7 字段）。

- 路径文件格式（被 `path_executor.load_path` 消费）：`x, y, z, qx, qy, qz, qw`（`path_executor.py:908-948`）。

### 2.1.3 path_executor.py（循迹单体，2622 行）

**职责**：解析路线命令或路径文件 → 在 50 Hz 控制环里用 Pure Pursuit 发布 `pursuitAngle`（Twist），在射击点/终点停车并交还开火权。

#### (a) 模块级常量（`path_executor.py:105-275`）

| 常量 | 值 | 行号 |
|---|---|---|
| COL_LETTERS | ABCDEFGHIJKLMN | 106 |
| BLOCK_SIZE | 0.35 | 109 |
| MAP_X_MIN/MAX | 0.175 / 3.675 | 113-114 |
| MAP_Y_MIN/MAX | 0.175 / 4.725 | 115-116 |
| MAP_SAFETY_MARGIN | 0.60 | 117 |
| RAW_YAW_TO_MAP_YAW_OFFSET | π/2 | 123 |
| MISSION_BLOCKS | 内置 20 点路线（K7…N5） | 141-163 |
| MISSION_RESET | 'M10,y' | 168 |
| AUTO_START | True | 171 |
| AUTO_START_DELAY | 8.0 | 172 |
| DRIFT_WARN_M | 0.30 | 178 |
| FINISH_VOICE_ID | 0 | 186 |
| REVERSE_SPEED_RATIO | 0.6 | 191 |
| REVERSE_STEER_SIGN | 1.0 | 195 |
| SHOOT_DISTANCE_TARGET/MIN/MAX | 0.30 / 0.20 / 0.45 | 200-202 |
| SHOOT_MAX_RETRY | 3 | 203 |
| SHOOT_TIMEOUT | 20.0 | 204 |
| SHOOT_SETTLE_SPEED | 0.02 | 207 |
| SHOOT_STABLE_FRAMES | 8 | 208 |
| SHOOT_STABLE_OFFSET_PX | 25 | 209 |
| SHOOT_STABLE_TIMEOUT | 6.0 | 210 |
| SHOOT_SETTLE_TIME | 0.8 | 223 |
| SHOOT_SCAN_ROUNDS | 2 | 228 |
| SHOOT_OVERSHOOT_TOL | 0.30 | 241 |
| SHOOT_SLOWDOWN_M | 0.60 | 242 |
| SHOOT_LOCK_TIMEOUT | 18.0 | 243 |
| ALIGN_STRAIGHT_BEFORE/AFTER/SPEED | 2.0 / 1.0 / 0.10 | 249-251 |
| ALIGN_KP / ALIGN_STEER_LIMIT | 12.0 / 8.0 | 252-253 |
| MISSION_SCAN_MODE | 'continuous' | 256 |
| BRIDGE_BLOCKS | ['K7','K1'] | 259 |
| OBSTACLE_BLOCKS | [] | 263 |
| LINEAR_SPEED | 0.15 | 271 |
| SHOOT_COUNT_TARGET | 3 | 275 |

**配置中心覆盖**（`path_executor.py:287-472`）：`_load_config_center()` 读 `~/dzrobot/config/robot.yaml` 的 `path_executor` 段（兼容 `/path_executor` 前缀，`path_executor.py:306-310`）；任何异常**静默回退**内置默认值（`path_executor.py:293,301-304`）。覆盖键见 `path_executor.py:424-472`（含 `linear_speed`、`auto_start`、`mission_blocks`、`bridge_blocks`、`obstacle_blocks` 等）。

> ⚠ **冲突（重要）**：代码内置 `MISSION_BLOCKS` 是 **20 个点**的旧路线（`path_executor.py:141-163`），而 `config/robot.yaml:532-600` 的 `mission_blocks` 是 **68 token**（第六张图路线）。默认 `AUTO_START=True` 时，`_auto_start_mission()`（`path_executor.py:1460-1514`）会下发配置中心的 68 点路线。两者只在 `AUTO_START=False` 时才会用到内置 20 点。**上车执行以配置中心为准**。

#### (b) 数学工具

- `euler_from_quaternion(x,y,z,w)`：`path_executor.py:479-487`，`yaw=atan2(2(wz+xy), 1-2(y²+z²))`。
- `normalize_angle(angle)`：`path_executor.py:490-498`，归一到 (-π, π]。
- `block_to_meters(block_name)`：`path_executor.py:501-526`。
- `heading_to_quaternion(heading_dir)`：`path_executor.py:529-546`，`'y'→yaw=π/2`、`'x'→yaw=0`。

#### (c) _GatedCmdPub（手动接管闸门）

`path_executor.py:554-577`。包装 `pursuitAngle` 发布器，`manual_override=True` 时吞掉所有自动 Twist，避免与遥控器抢同一个话题。动机注释：本文件有 15 处 `self.cmd_pub.publish(...)`，包一层是唯一入口（`path_executor.py:563-565`）。

#### (d) 构造器状态（`path_executor.py:581-874`）

关键状态：
- 路径：`path_points`、`current_target_idx`、`path_loaded`（594-596）。
- 信念位姿：`current_x/y/yaw`、`raw_x/y/map_yaw`、`snap_x_off/snap_y_off/snap_yaw_off`（599-601, 731-736）。
- 循迹几何：`lookahead_dist=0.15`（621）、`wheelbase=0.157`（629）、`arrival_threshold≈0.157`（637）、`approach_dist=0.6`（638）。
- 打靶状态机：`shoot_wait`、`shoot_phase`（0 SETTLE / 1 ACQUIRE / 2 STABLE / 3 FIRE）、`shoot_settle_since`、`scan_rounds_done`、`SHOOT_WAIT_TIMEOUT=150`、`SHOOT_STOP_RADIUS=0.17`（654-669）。
- 扫描握手：`aim_status`、`scan_wait`、`SCAN_WAIT_TIMEOUT=8.0`、`SCAN_STOP_RADIUS=0.20`（676, 695-699）。
- 视觉缓存：`det_dx/dy/cls/box_w/t`（678-682）。
- 漂移统计：`drift_max`、`drift_yaw_max`、`drift_legs`、`drift_warn_count`（832-835）。
- 控制定时器 50 Hz（838）；发布者健康检查 3 s 后跑一次（843）。
- 订阅/发布清单：`/odom/localization`(747)、`/nav/waypoint`(757)、`pursuitAngle`(776)、`/move_base/stop_signal`(786)、`/shoot_count`(793)、`/nav/manual_override`(804)、`/aim/status`(807)、`/nav/scan_request`(811)、`/nav/motion_state`(815)、`/offset_center`(824)、`/voice_switch`(829)。

> ⚠ **单位约定（关键）**：`/odom/localization` 的位置是 **block 单位**，`path_executor` 乘 `BLOCK_SIZE` 得米（`path_executor.py:1602-1603`）；四元数携带的是 **raw yaw_rel**，加 `RAW_YAW_TO_MAP_YAW_OFFSET=π/2` 转 map 系（`path_executor.py:1609-1613`）。

#### (e) 命令解析

`waypoint_callback(msg)`（`path_executor.py:954-1097`）支持：
- `load:/path/file.txt` → `load_path`（961-964）
- `reset:BLOCK,HEAD` → 重锚里程计（966-996 区间）
- `M10,y` 单点 / `G8 A10 D2*` 多点 / `A10` 单点 → `follow_route_command`

`follow_route_command(data)`（`path_executor.py:1098-1295`）解析规则：
- 末尾 `,x` / `,y` 强制终点朝向（1108-1117）。
- `SHOOT@<靶标>` → 调 `compute_shoot_chain()` 插候选点（1136-1161）。
- `R:B5` 倒车前缀（1167-1169）；`:SHOOT_STOP`/`:SCAN_STOP` 标签（1174-1181）；`*` 射击后缀（1182-1183）。
- 重复点去重：同坐标且同倒车标记时合并，只把前一个标为射击点（1196-1203）。
- 终点前 0.6 m 插入轴对齐接近段（1234-1253）。
- 回填每点四元数 = 离开航段方向（1259-1266）。
- 提交后 `_rebuild_align_segments()`（1275）+ `_start_mission_scan()`（1281）。

#### (f) 传感器回调

- `shoot_count_cb`（`path_executor.py:1302-1306`）：缓存累计发数。
- `_update_chassis_speed()`（`path_executor.py:1312-1323`）：位姿差分 + 一阶低通 `0.7*旧+0.3*新`。
- `offset_center_cb`（`path_executor.py:1325-1337`）：缓存 `det_dx/dy/cls/box_w`（要求 `len>=3`）。
- `aim_status_cb`（`path_executor.py:1339-1341`）、`manual_override_cb`（1343-1354）。

#### (g) 停站与射击

- `enter_scan_stop(wp_idx)`（`path_executor.py:1356-1370`）：零 Twist 刹车 + 发 `/nav/scan_request=1`，等待最多 `SCAN_WAIT_TIMEOUT=8 s`。
- `enter_shoot_stop(wp_idx)`（`path_executor.py:1516-1589`）：零 Twist + 把信念位姿吸附到格心、航向吸附到进入段方向（1529-1541）；清 `shoot_flags[wp_idx]`；`shoot_phase=0`；**先发 `stop_signal=0` 收回许可**（1570-1572）。
- 打靶四阶段（`control_loop` 分支，`path_executor.py:2034-2193`）：
  - **0 SETTLE**：等 `chassis_speed < 0.02` 且位置连续 `0.4 s` 不变（`path_executor.py:2051-2074`；配置 `shoot_settle_time` 在 robot.yaml:439 为 0.4）。
  - **1 ACQUIRE**：等 `det_cls != 0` 且 det 新鲜 <1 s；或云台扫完 `SHOOT_SCAN_ROUNDS=2` 轮仍无靶则放弃（2076-2105）。
  - **2 STABLE**：偏移落在 ±`SHOOT_STABLE_OFFSET_PX`（配置 40 px）且连续 `SHOOT_STABLE_FRAMES`（配置 5）帧 → 放行（2108-2148）。
  - **3 FIRE**：等 `shoot_count` 增量达 `SHOOT_COUNT_TARGET`（配置 10）→ 成功；否则 `SHOOT_TIMEOUT=30 s` 硬超时换点（2150-2193）。
- `_passed_waypoint(wp_idx, wp)`（`path_executor.py:1882-1904`）：判断是否冲过射击点（点积为负），用于「过冲容忍」补触发。

#### (h) Pure Pursuit 几何

- `find_nearest_segment()`（`path_executor.py:1617-1653`）：从 `current_target_idx` 向后搜索，投影参数 `t = clamp(dot(AP,AB)/|AB|², 0, 1)`。
- `find_lookahead_point(seg)`（`path_executor.py:1657-1732`）：前视 `lookahead_dist=0.15` m；转角 > 20° 且距顶点 < 4 cm 时钳到顶点（1677, 1702-1706）。
- `pure_pursuit_control(target_point, reverse=False)`（`path_executor.py:1736-1809`）：

    alpha      = atan2(rel_y, rel_x)
    steer      = degrees(atan(wheelbase * sin(alpha) / max(dist, 0.10)))
    steer      = clamp(steer, -30, +30)
    turn_factor = |steer| / 30
    forward: linear = linear_speed * (1 - 0.5*turn_factor), 下限 0.1 m/s
    reverse: steer *= REVERSE_STEER_SIGN
             rev = linear_speed*0.6*(1-0.5*turn_factor), 下限 0.08; 返回 (-rev, steer)

  证据：`path_executor.py:1778-1809`。

#### (i) 主控制环（50 Hz）

`control_loop()`（`path_executor.py:2005-2592`）优先级：

1. **无里程计** → 什么都不做（2010-2011）。
2. **shoot_wait** → 零 Twist 保活 + 四阶段状态机（2034-2193）。
3. **scan_wait** → 零 Twist 保活 + 三种放行（aim_status==2 / 见过 SCAN 后回 IDLE / 8 s 超时）（2200-2217）。
4. **无路径** → 零 Twist 保活（2222-2226）。
5. 最近航段 + 前视点（2230-2239）。
6. **点位扫描停站检测**：`d <= SCAN_STOP_RADIUS(0.20)` 且未锁定 → `enter_scan_stop`（2245-2260）。
7. **射击停站检测**：`d <= SHOOT_STOP_RADIUS(0.17)` 或「已越过且 `d <= SHOOT_OVERSHOOT_TOL(0.30)`」→ `enter_shoot_stop`（2265-2283）。
8. **终点到达**：`dist_to_final < arrival_threshold(≈0.157)` → 锁存 `arrived`、吸附、漂移统计、发 `stop_signal=1`、请求终点扫描、打印完成横幅（2285-2363）。
9. **急转弯 PIVOT**：转角 > 45° 且距顶点 < 0.25 m → 三段原地转（停 0.5 s → 满舵 → 0.08 m/s 缓行到航向差 < 15°）（2365-2468）。
10. **桥洞强制直行**：`align_on` → 锁航向 + 小舵角 + 0.10 m/s（2470-2504）。
11. **正常 Pure Pursuit**：越界急停（2511-2528）→ 终点前 0.5 m 减速（2532-2533）→ 射击点前 0.6 m 减速（2536-2544）→ 转角前减速到 0.12 m/s（2549-2568）→ 发布（2573-2576）。

#### (j) 入口

`main(args=None)`（`path_executor.py:2599-2618`）：可选第 1 个 argv 作为路径文件；Ctrl-C 时发布零 Twist 刹车。

### 2.1.4 serial_reader.py（死推算定位）

**策略**（`serial_reader.py:13-27`）：航向取 `/imu_data`（2 s 超时回退 `/odom`）；位置用**死推算**——因为 `dzactuator` 积分的位姿在「编码器-航向」坐标系下旋转不正确（作者实测 2026-09-06：同一次 90° 右转，两次跑偏到相反方向并单调外扩），故只取**样本间位移距离**（旋转不变）沿 IMU map yaw 积分；残余误差由 `path_executor` 的逐角点吸附消除。

- 常量：`BLOCK_SIZE=0.35`、`START_X=1.5`、`START_Y=0.5`、`IMU_TIMEOUT=2.0`、`MIN_STEP_M=0.0005`、`FLIP_RAW_YAW_SIGN=False`、`NODE_VERSION="v4-dr-reset"`（`serial_reader.py:38-65`）。
- `get_block_name(x,y)`（`serial_reader.py:68-75`）。
- `euler_yaw` / `normalize_angle`（78-89）。
- 订阅 `/odom`、`/imu_data`、`/nav/waypoint`；发布 `/odom/localization`（`serial_reader.py:96-108`）。
- `imu_callback`（146-154）：记录 `imu_yaw_rel`（相对首次）。
- `waypoint_callback`（156-187）：只处理 `reset:`，重新基准 DR 方向（`dr_yaw_off`），位置重锚留在 `path_executor`（避免双重修正，见 100-103）。
- `odom_callback`（189-255）：

    yaw_rel = IMU(新鲜) ? imu_yaw_rel : odom四元数yaw
    step = hypot(Δx, Δy)
    if step >= 0.0005:
        map_yaw = normalize(yaw_rel + dr_yaw_off)
        dr_x_m += step*cos(map_yaw); dr_y_m += step*sin(map_yaw)
    发布位置 = (dr_x_m/0.35, dr_y_m/0.35)   # block 单位
    四元数 = yaw_rel 的四元数              # 供 path_executor 加 π/2

  证据：`serial_reader.py:214-244`。

> ⚠ **健康检查依赖**：`path_executor.check_odom_publishers()`（`path_executor.py:880-903`）要求 `/odom/localization` 恰好 1 个发布者，否则报「两个死推算节点交替 → 90° 跳变」。`localization.launch.py:5-9` 明确注释不要起第二个发布者。

### 2.1.5 send_waypoint.py（一次性航点发送）

`send_waypoint.py:17-35`：创建 `WaypointSender`，0.1 s 定时器发一次 `/nav/waypoint`（String），发完 `shutdown()`。用法 `python3 send_waypoint.py M10,y`（`send_waypoint.py:39-45`）。

### 2.1.6 launch 与打包

- `localization.launch.py:10-19`：只起 `serial_reader_node`。
- `setup.py:22-31`：注册两个 console_scripts——`serial_reader_node` 与 `path_executor_node`（2026-10-03 补充；注释强调 `start_all.sh` 仍直跑、不要贸然改）。
- `start_path_executor.sh:23-35`：先生成 test_path.txt 再起 `path_executor.py`，然后 `ros2 topic pub /nav/waypoint`。

### 2.2 dzactuator（底盘串口 + TF + IMU）

#### 2.2.1 概述与线程模型

dzactuator.cpp 是整车最底层的 C++ 节点：它独占底盘串口，以 100 Hz 循环完成「读下位机数据 → 解算里程计/IMU → 发布 ROS 话题 → 打包下行控制帧 → 写串口」。节点构造时先 declare_parameter 再 get_parameter，参数全部来自 config/robot.yaml 的 /dzactuator 段（dzactuator.cpp:1121-1142）。

线程模型：单线程 main → node->Control()（dzactuator.cpp:20-28），Control() 内每拍 rclcpp::spin_some(shared_from_this()) 处理回调（dzactuator.cpp:1273），然后同步读串口、同步发串口。没有独立的控制线程——所有回调与主循环共享同一线程，因此回调里不能做长阻塞。

关键参数声明（dzactuator.cpp:1121-1142）：

- serial_port_name 默认 /dev/ttyUSB0（配置中心改为 /dev/dzactuator）
- serial_baud_rate 默认 115200
- odom_frame_id=odom、robot_frame_id=base_link、gyro_frame_id=imu_link
- calibrate_lineSpeed 默认 1、ticksPerMeter 默认 1000.0、ticksPer2PI 默认 3200.0
- smooth_window_speed=20、smooth_window_angle=5

⚠ 默认值与配置中心不一致：代码内置 ticksPerMeter=1000 / ticksPer2PI=3200（dzactuator.cpp:1127-1128），而 config/robot.yaml:932,941 已改为 3280 / 2680。若 launch 没把 params-file 正确喂进去（类型不匹配），会静默回落到 1000/3200，里程计随之错误。bringup.launch.py:6-16 记录了这一坑。

#### 2.2.2 底盘串口协议（核心）

##### (a) 下行帧：ROS → STM32（23 字节）

打包函数 sendCarInfoKernel()（dzactuator.cpp:376-449），写入 unsigned char buf[23]：

| 偏移 | 长度 | 字段 | 来源 / 语义 |
|---|---|---|---|
| buf[0] | 1 | 0xA5 | 帧头 |
| buf[1] | 1 | 0x5A | 帧头 |
| buf[2] | 1 | 20 | 数据长度（dzactuator.cpp:394） |
| buf[3..4] | 2 | int16_t(TargetAngle*10) 大端 | 舵机角度，0.1 度为单位（396-398） |
| buf[5] | 1 | SpeedDirection | 0x00=前进，0x01=倒车（104-108） |
| buf[6] | 1 | TargetSpeed | 速度 0..255，abs(clamp(v*23/0.33, -255, 255))（78） |
| buf[7] | 1 | LaserGunSwitch | 激光常亮开关 0 关 / 1 开，由 /laser_switch 控制（405-408） |
| buf[8..9] | 2 | Position_0 大端 | 云台电机 0 位置（410-411） |
| buf[10..11] | 2 | Position_1 大端 | 云台电机 1 位置（412-413） |
| buf[12..13] | 2 | Speed_0 大端 | 云台电机 0 速度（415-416） |
| buf[14..15] | 2 | Speed_1 大端 | 云台电机 1 速度（418-419） |
| buf[16] | 1 | Time_0 | 云台电机 0 时间（422） |
| buf[17] | 1 | Time_1 | 云台电机 1 时间（424） |
| buf[18] | 1 | Fun | 功能模式（426） |
| buf[19] | 1 | Orin_ID | 使用 ID（428） |
| buf[20] | 1 | Set_ID_Num | 重置 ID（430） |
| buf[21] | 1 | VoiceSwitch | 语音播报 ID（434） |
| buf[22] | 1 | 校验和 | 累加和：sum = Σ buf[i], i=2..21（436-439） |

注意：

- 帧头是 0xA5 0x5A（不是 0x7B/0x7D）。0x7B 是上行帧头。
- 校验和是累加和（dzactuator.cpp:437 注释虽写 BCC/XOR，但实现是加法求和）。头文件注释称 BCC 校验（dzactuator.h:245），与实现不符——注释误导。
- TargetAngle = angular.z + 60，再被夹到 [25,95]（dzactuator.cpp:87,93-101）。dzactuator.cpp:87 对 DWA 路径不执行 +60（见下）。
- 云台位置/速度来自 callback_CameMo_Conl（dzactuator.cpp:270-283，读 msg->data[0..3]）——即 dzrknn_dataprocess 发布的 /CameMo_Conl。注意 Time_0/Time_1 被硬编码为 0（dzactuator.cpp:281-282）。
- 数据平滑：TargetAngle 经 smoothValue(angle_history_, …, smooth_window_angle=5) 滑动平均（dzactuator.cpp:387-388）；TargetSpeed 的平滑被注释掉（dzactuator.cpp:384-385）。

##### (b) 上行帧：STM32 → ROS（34 字节）

读取函数 Get_Sensor_Data_New()（dzactuator.cpp:883-1039）。帧头 0x7B，第二字节 0x1F，随后 RECEIVE_DATA_SIZE - 2 = 32 字节（dzactuator.h:64）。总长 34。

| 偏移 | 长度 | 字段 | 解析 |
|---|---|---|---|
| rx[0] | 1 | 0x7B | 帧头（939） |
| rx[1] | 1 | 0x1F | 长度标记（940） |
| rx[2..3] | 2 | Odom_Trans 大端 short | 左轮速度（968） |
| rx[4..5] | 2 | 同上 | 右轮速度（969） |
| rx[6..7] | 2 | 大端 short | 电池电压 mV → /1000.0 得 V（973-975） |
| rx[8..9] | 2 | IMU_Trans | 加速度 X（978） |
| rx[10..11] | 2 | IMU_Trans | 加速度 Y（979） |
| rx[12..13] | 2 | IMU_Trans | 加速度 Z（980, 982 重复写） |
| rx[14..15] | 2 | IMU_Trans | 角速度 X（985） |
| rx[16..17] | 2 | IMU_Trans | 角速度 Y（986, 989 重复写） |
| rx[18..19] | 2 | IMU_Trans | 角速度 Z（987, 990 重复写） |
| rx[20..21] | 2 | Odom_Trans | 云台电机 0 位置（1003） |
| rx[22..23] | 2 | Odom_Trans | 云台电机 0 速度（1005） |
| rx[24..25] | 2 | Odom_Trans | 云台电机 0 负载（1007） |
| rx[26..27] | 2 | Odom_Trans | 云台电机 1 位置（1011） |
| rx[28..29] | 2 | Odom_Trans | 云台电机 1 速度（1013） |
| rx[30..31] | 2 | Odom_Trans | 云台电机 1 负载（1015） |
| rx[32] | 1 | 预留位 | 跳变检测（1021-1035） |

数据换算：

- 加速度：* 24 / 32768.0f * 9.8（dzactuator.cpp:993-995）
- 角速度：* 1000 / 32768.0f / 57.2957795f（997-999）
- 电池：mV / 1000.0 → V（975）

⚠ 坑 1（已修，保留说明）：Get_Sensor_Data_New() 开头等待帧头用的是 while (++guard < 200000)（dzactuator.cpp:897-903）。旧代码是 while(1) 死等，下位机掉线时 read() 抛 serial::SerialException（父类，旧代码只 catch IOException 子类）→ 异常逃逸 → SIGABRT。现行为：限次循环 + catch SerialException 返回 false（dzactuator.cpp:909-913），并在 Control() 调用点再兜一层（dzactuator.cpp:1276-1286）。

⚠ 坑 2：rx[12..13]、rx[16..17]、rx[18..19] 各被重复赋值两次（dzactuator.cpp:980 与 982、986 与 989、987 与 990）。冗余但无害——只要两次取的源相同。

⚠ 坑 3：上行帧没有校验。原代码注释掉的校验（dzactuator.cpp:957-962）比较 rx[33] 与 Check_Sum(33, READ_DATA_CHECK)，但实际未启用。Check_Sum() 定义在 dzactuator.cpp:857-876 却只在析构函数里被使用（dzactuator.cpp:1242）。

⚠ 坑 4：等待帧头时 head_Receive_Data 若非 0x7B，函数返回 false（dzactuator.cpp:904-907），此时本周期不做任何更新，主循环继续——不会死等。

#### 2.2.3 回调与控制模式

##### callback_cmd_vel_angle（dzactuator.cpp:59-110）

订阅 /cmd_vel（DWA 模式）。重要：current_mode_ != 1 时直接 return（66）；control_mode==1 时执行。

- v = twist.linear.x（m/s）、w = twist.angular.z（rad/s）
- TargetSpeed = abs(clamp(v * 23 / 0.33, -255, 255))（78）——0.33 是标定满速 m/s，23 是满速 PWM 当量
- TargetAngle = round(atan(CARL * w / v) * 57.3) + 60，CARL=0.15 轴距（81, 87）
- 夹到 [25,95]（93-101）
- 方向：v>0 → 0x00，v<0 → 0x01（103-108）

⚠ 单位陷阱（作者用注释标红）：/cmd_vel 的 angular.z 是 rad/s，而 /pursuitAngle 的同名字段是度。dzactuator.cpp:72-74 明确标注，勿混。

##### callback_movebase_angle（dzactuator.cpp:112-186）

订阅 /pursuitAngle（循迹 Pursuit 模式）。current_mode_ != 2 时 return（123）。

- v = linear.x（m/s）、w = angular.z（度）
- v 经 smoothValueAdaptive(speed_history_, v, smooth_window_speed=20)（149）
- TargetSpeed = abs(clamp(v*23/0.33, -255, 255))（151）
- TargetAngle = w + 60，夹 [25,95]（152-163）
- 方向同上（165-170）
- 上报 /current_vel_state：linear.y = 档位（9=D，2=R，3=N），linear.z=1 表示非平行驾驶（172-183）

smoothValueAdaptive（dzactuator.cpp:335-373）：根据 |v| 动态扩窗（<0.3 → w；<0.5 → w+5；<1.0 → w+10；否则 w+15），但缓存上限仍是 w+15，然后取最近 temp_window 个值求平均。

##### callback_logi_control（dzactuator.cpp:188-239）

订阅 /logicontrol。linear.z > 1.0 → control_mode=2（平行驾驶），否则 1（195-202）。切换时发 /navi_cmd=1（204-210）。control_mode_count 用于超时回退：callback_movebase_angle 中 control_mode_count++，>40 则切回自动模式并发 /navi_cmd=1（129-140）。

##### 其他回调

| 回调 | 行号 | 话题 | 作用 |
|---|---|---|---|
| callback_voice_switch | 241-246 | /voice_switch | 写 moveBaseControl.VoiceSwitch |
| callback_laser_switch | 248-257 | /laser_switch | 写 LaserGunSwitch（0/1） |
| callback_vehicle_state | 258-262 | /vehicle_state | voice_nums[0] |
| callback_vehicle_Drivestate | 264-268 | /vehicle_Drivestate | voice_nums[1] |
| callback_CameMo_Conl | 270-283 | /CameMo_Conl | 云台位置/速度（4 字段） |
| callback_stopCMD | 285-291 | /StopCmd | stop_cmd（10 拍内清速） |

switchvoice()（dzactuator.cpp:293-319）：同一语音 ID 至少间隔 2 s 才重播（306-309）。

#### 2.2.4 里程计积分

Control() 主循环（dzactuator.cpp:1256-1338）：

    linear_Speed  = (leftSpeed + rightSpeed)/2        # leftSpeed = Left/ticksPerMeter
    ThetaSpeed    = (Right - Left) * 2π / ticksPer2PI
    if |linear_Speed| > 1e-5:
        Robot_Pos.Z += ThetaSpeed; normalize(-π,π]
        Robot_Pos.X += linear_Speed*cos(Z)
        Robot_Pos.Y += linear_Speed*sin(Z)
    else: montion_flag = false
    Publish_Odom(); Publish_CameMo(); Publish_ImuSensor(); ...
    checkCommandWatchdog(); sendCarInfoKernel()

证据：dzactuator.cpp:1289-1334。

- ticksPerMeter=3280.0、ticksPer2PI=2680.0（robot.yaml:932,941）→ 几何等价轮距 2680/(2π×3280) = 0.130 m（robot.yaml:940）。
- Publish_Odom()（dzactuator.cpp:646-703）发布 /odom（Odometry）：位置 X/Y、z=0.2、yaw 由 Z 转四元数；速度 linear.x = linear_Speed*50、angular.z = ThetaSpeed*50（乘 50 的单位换算，669-671）。协方差按「轮速为 0 → 更信编码器」选两组矩阵（674-681）。
- Publish_CameMo()（dzactuator.cpp:823-845）发布 /camemo_feedback（Int32MultiArray[6]）：Position_0/1, Speed_0/1, Load_0/1。

⚠ 坑：Publish_Odom 里 Robot_Pos.Z 的 normalizeAngle 在 X/Y 累加之后才调用（dzactuator.cpp:1302），即先积分再归一，逻辑正确但写法反直觉。另：Robot_Pos.X/Y 初值在 Control() 开头被清零（1259-1261），但 Robot_Pos.Z 也在开头清零（1261）——每次重启里程计归零。

#### 2.2.5 IMU 与四元数

- Publish_ImuSensor()（dzactuator.cpp:576-634）：发布 /raw（原始 IMU）与 /imu_msg_valid；协方差用 Allan 标定结果硬编码（603-610）。
- Publish_ImuSensor_filter()（dzactuator.cpp:501-569）：发布 /imu_data（滤波+零偏后）。
- 零偏校正 imu_bias_correction()（dzactuator.cpp:451-499）：前 BIAS_CALIBRATION_SAMPLES=200 帧累积零偏（dzactuator.h:294），之后减去；accel_bias_z 额外减 9.81（479）。零偏未完成时输出全 0（485-486）。
- Quaternion_Solution()（Quaternion_Solution.cpp:214-280）：经典 Madgwick 风格一阶互补滤波，twoKp=1.0、twoKi=0.0（210-211），用 InvSqrt 快逆平方根（192-205）。积分步长固定 1/100（255-257）。
- filter_imu_data()（Quaternion_Solution.cpp:162-186）：滑动平均预滤波（ENABLE_SMOOTH_PREFILTER=true，169-173）→ 二阶 Butterworth 低通（GYRO_CUTOFF_FREQ=10Hz，177-179）→ 加速度卡尔曼（183-185）。
- detect_motion()（Quaternion_Solution.cpp:148-157）：|gyro|>0.1 rad/s 或 |accel|-9.81 > 1.0 → 运动。

⚠ 坑：MOTION_ADAPTIVE_FILTER=true 时运动中 R 放大 5 倍、静止时缩小 0.5 倍（Quaternion_Solution.cpp:56-62）。协方差矩阵在 Publish_ImuSensor 里与 montion_flag 相关（536-565），而 montion_flag 只在 Control() 里更新（1300, 1310）——/imu_data 的协方差反映的是上一拍的运动状态。

#### 2.2.6 电池电量

Publish_Battery_Percentage()（dzactuator.cpp:728-816）：

- x = clamp((v-Power_min)/(Power_max-Power_min), 0, 1)，Power_max=12, Power_min=10（1096-1097, 735-746）
- 10 点滑动平均（727, 748-768）
- 下降门槛 10 次、上升门槛 100 次（773-802），保证电量只降难升

#### 2.2.7 指令看门狗

checkCommandWatchdog()（dzactuator.cpp:1346-1374）：/pursuitAngle 与 /cmd_vel 两路都超过 CMD_WATCHDOG_S=1.0 s 无消息 → TargetSpeed=0, SpeedDirection=0x00（只停速度，不动云台角度）。在 Control() 里于 sendCarInfoKernel() 之前调用（1332）。

modeTimerCallback()（dzactuator.cpp:1376-1448），100 ms 定时器：

- ACTIVITY_TIMEOUT=0.5 s（dzactuator.h:320）判断 DWA / Pursuit 是否活跃。
- 单源活跃：只有 pursuit → current_mode_=2；只有 dwa → current_mode_=1。
- 双源：pursuit.linear.y >= 0.5 → DWA（1423-1431）；否则若 current_mode_==1 且 pursuit 速度 > dwb 速度且 PURSUIT_LOW_TIMEOUT=3.0 s 已过 → 切 Pursuit（1434-1447）。

⚠ 坑：callback_movebase_angle 里 current_mode_ 的判断在 control_mode==1 分支之前（123-124），即 current_mode_ != 2 时整个回调直接 return——即使 control_mode 是 2（平行驾驶）也不会执行。这是平行驾驶时 pursuitAngle 无效的根因。

#### 2.2.8 launch 与 CMake

- bringup.launch.py:39-61：两个 static_transform_publisher（base_link→imu_link 0.08 m 高；base_link→laser 15 cm 前、绕 X 180°），然后起 dzactuator_node（包 dzactuator）。参数来自 ROBOT_CONFIG（默认 ~/dzrobot/config/robot.yaml）；文件缺失时传空参数列表、节点用内置默认值（26-32）。
- CMakeLists.txt:87：add_executable(dzactuator_node src/dzactuator.cpp src/Quaternion_Solution.cpp)；安装到 lib/dzactuator（92-97）。
- package.xml：42 行（依赖 rclcpp/nav_msgs/sensor_msgs/tf2/tf2_ros/serial/geometry_msgs/Boost）。

⚠ 坑：launch_auto/auto_sl_s3e.launch.py:49-53 引用 rplidar_s3e_launch.py 与 amcl.launch.py，但 dzactuator/launch/ 下没有这两个文件（本次审计时 launch/ 只有 bringup/bringup_all/camera/judgment）。该 launch 一旦执行会失败——属于死引用。同时 bringup_all.launch.py:22-23 注释说旧避障雷达分支已移除。

#### 2.2.9 dzactuator/params 下的 YAML

| 文件 | 段 | 用途 |
|---|---|---|
| vehicle_client_params.yaml | vehicle_client_node | 车型客户端参数 |
| amcl.yaml | amcl / amcl_map_client / amcl_rclcpp_node | AMCL 定位（历史栈） |
| imu_calib.yaml | SM / bias | IMU 标定（陀螺/加速度零偏） |
| bmi_imu_param.yaml | type: IMU / name: bmi / Gyr / Acc | BMI088 原始参数 |
| mapping.yaml / slam_gmapping.yaml / self_amcl.yaml / group_amcl_sl.yaml / robot4_map.yaml | 各节点 | SLAM/AMCL 历史配置 |

⚠ 未核实：以上 params 文件当前没有任何 launch 引用（bringup.launch.py 只读 robot.yaml）。它们属于被 auto_sl_s3e/AMCL 栈引用的历史配置，处于冻结但无活引用状态。

### 2.3 dzrknn_dataprocess（云台自瞄 + 开火判定）

#### 2.3.1 概述与线程模型

这是整车最复杂的节点（1725 行）。它订阅视觉偏移，做位置伺服把靶拉到画面中心，同时维护 IDLE/SCAN/TRACK 状态机，并在满足条件时发布开火指令。

线程模型：单线程 main → node->Control()（dzrknn_dataprocess.cpp:14-22）。Control() 每拍 rclcpp::spin_some(shared_from_this())（1691），然后同步跑 CaremaMontorControl() 状态机（1693），最后发布 CameMo_Conl 与 aim/status（1696-1720），按状态选频率 sleep（1723）。

自适应频率：TRACK 时 loop_hz_track_（配置 100 Hz），其余 loop_hz_idle_（50 Hz）（1723）。注意丢帧计数 target_seen_count_/lost_target_count_ 都在视觉回调里按 30 Hz 视觉帧自增，与本循环频率无关（1686-1688）。

手动接管：/nav/manual_gimbal=1 时 manual_override_=true，Control() 不再发 CameMo_Conl 的位置字段（1702-1707），但 LaserShot_Command / aim_status 照发。理由：本节点以 50~100 Hz 持续发 CameMo_Conl，与遥控器抢同一话题会导致云台疯狂来回抽（dzrknn_dataprocess.h:109-113）。

参数：全部 declare_parameter + get_parameter（dzrknn_dataprocess.cpp:1305-1528），默认值在 dzrknn_dataprocess.h。配置中心 robot.yaml 的 /dzrknn_dataprocess 段覆盖。

#### 2.3.2 自瞄状态机（AIM_IDLE / AIM_SCAN / AIM_TRACK）

枚举（dzrknn_dataprocess.h:134）：AIM_IDLE=0、AIM_SCAN=1、AIM_TRACK=2。

- setAimState(int s)（dzrknn_dataprocess.cpp:843-855）：状态变化时立即 publishAimStatus() 并打日志。
- publishAimStatus()（857-862）：发 /aim/status（Int32）。
- 状态上报保活：状态未变化时每 0.5 s 补发一次（1714-1720），按时间而非循环次数（否则降频后保活周期变长）。

#### 2.3.3 视觉回调 callback_offset_center（24-474，核心）

输入：std_msgs/Int32MultiArray，字段定义（det_node 发布，det_node.cc:220-222）：

- data[0] = centerx 偏移（px）
- data[1] = centery 偏移（px）
- data[2] = 类别（0=无目标，1=蓝靶，2=红靶）
- data[3] = 框宽，data[4] = 框高
- data[5..7] = 关键点偏移与置信度（备用）

处理流程：

1. 长度守卫（30-36）：要求 data.size() >= 5，否则丢弃并 5 s 节流告警。动机：std::vector operator[] 不检查边界，异常帧会读栈垃圾。
2. 刷新视觉心跳 last_vision_time_（37）。
3. 纯 IDLE 忽略视觉 TRACK（47-50）：若 aim_state_==AIM_IDLE 且不在归位段 scan_homing_，直接 return。这是需求 3（用户 2026-10-16）：防止跑完全程后云台被反复从 IDLE 拽回 TRACK 造成跑完乱动。
4. 目标筛选（53）：is_target = is_target_cls && aspectRatioOk(msg)。is_target_cls 为 data[2]∈{1,2}（40）；aspectRatioOk（582-596）要求宽高比在 [aspect_min, aspect_max]（配置 0.60/1.60）。
5. 进入 TRACK（57-77）：
   - 重置 fire_hold_count_=0、filt_inited_=false、target_seen_count_=0、fire_cleared_=false、shots_on_target_=0（61-65）
   - 不清零类别锁 locked_cls_/cls_streak_（66-73 注释）：旧行为每次重新锁定都归零，目标闪一下就得从 5 帧重新累计 → F9 停站 30 s 内反复类别未确认/丢失 → 永不开火。
   - setAimState(AIM_TRACK)、resetPid()（74-75）。
6. 扫描确认（83-89）：target_seen_count_++，达到 target_confirm_frames_（配置 2）才算扫描确认，才允许开火。
7. 一阶低通 EMA（91-109）：
   - ema_a = track_mode_=="v1" ? v1_ema_alpha_ : ema_alpha_（93）
   - dx_filt_ = ema_a*dx_raw + (1-ema_a)*dx_filt_，dy 同理（108-109）
   - 首次初始化时同步复位 dx_prev_filt_/dy_prev_filt_（96-107）
8. 跟踪死区（111-122）：dz = track_mode_=="v1" ? v1_deadzone_px_ : track_deadzone_px_；|dx_filt_|>dz 时 eff_x = dx_filt_ ± dz（符号保持）。
9. 位置增量 + 预判（124-202）：
   - vx = dx_filt_ - dx_prev_filt_（像素/帧），dx_prev_filt_ = dx_filt_（126-129）
   - 车体转动前馈 ff_x（135-148）：

       ff_x = -imu_omega_z_ * 57.2957795 * yaw_ticks_per_deg_ * dt * gyro_ff_gain_ * gyro_ff_sign_

     仅在 gyro_ff_gain_!=0 且 IMU 新鲜（< imu_ff_timeout_ms_=200 ms）时计算。配置 gyro_ff_gain=0.8、gyro_ff_sign=1.0（robot.yaml:242-243）。
   - 俯仰 pitch_delta 分三套口径（157-171，详见 2.3.4）
   - pitch_inc = lround(pitch_delta * pitch_deg_sign_)（172）
   - 俯仰符号修复（172-182 注释）：2026-10-17 修复追踪时总朝目标上方看。原式多一个负号。实测锚点（2026-10-15 现场）：pitch_deg_sign=+1、scan_pitch_offset_deg=+10° → tick +113.78 → 云台向下压（低头）⇒ 本车 tick 增大 = 低头。追踪需求：dy>0（目标在画面下方）⇒ 需要低头 ⇒ pitch_delta>0 ⇒ pitch_inc 必须为正。
   - 俯仰钳位改为机械限位（183-202）：跟踪走 clampPitchTick（950~2400），不再走 clampPitchToScan。原因：旧 clampPitchToScan 的允许区间 = offset ± margin，本车 scan_pitch_offset_deg=6.0、scan_pitch_levels=[0.0]、margin=6.0 → 区间 [0°,12°] → 换算刻度仅 [2047,2183]，跟踪时俯仰被焊死在水平到微抬头，地面靶需要低头约 40° 根本看不到。
   - apply_pitch 闸门：track_mode_=="pid" 且 pid_enable_ 时要求 |pitch_inc| >= pid_min_step_（196-197）。
   - moveBaseControl.Position_0 = clampPitchTick(curYuntai_feedback_data.Position_0 + pitch_inc)（200-201）。
10. 水平轴（204-235）：
    - yaw_delta/yaw_lead 同样分三套口径（208-224）
    - inc_x = lround(yaw_delta + yaw_lead + ff_x)（225）
    - apply_yaw 闸门（227）
    - v1 模式且 v1_yaw_full_range_ 时用整量程 clampYawTick，其余用 clampYawToScan（231-234）
    - 注意：即使 eff_x==0，只要车在转（ff_x≠0）也要下发，否则转弯中目标被甩出画面（204-205 注释）
11. 速度下发（237-259）：
    - pid：pidSpeedFor(pid_rate_pitch_, eff_y)（245-246）
    - v1：v1TrackSpeedFor(...)（250-251）
    - tuned：trackSpeedFor(...)（255-256）
    - 死区内 eff==0 → 速度 0（彻底消除小幅来回摆动）
12. 开火判定（261-392，详见 2.3.5）
13. 目标丢失处理（410-472）：
    - target_seen_count_ 衰减而非清零（414-417）
    - lost_target_count_++（422）
    - 去抖滞回（423-440）：需要连续未见 >= max(lost_frames_to_rescan_, rescan_min_frames_) 帧 且 时长 >= max(1, rescan_debounce_ms_) ms
    - 满足后清类别锁（443-445），scan_continuous_ 时 startScan(true, home=false) 接续扫掠（446-454）；否则 setAimState(AIM_IDLE)（456-459）
    - 发布 LaserShot_Command=0 与 voice_switch=111（463-470）
14. 物资点播报（395-408）：data[2] > 2 时按类别 ID 发 /voice_switch。

#### 2.3.4 五套 PID / 跟踪口径并存

这是本模块最需要警惕的地方。代码里同时存在五套互斥的跟踪公式，由 track_mode_ 与 pid_enable_ 选择。当前配置 track_mode: pid、pid_enable: true（robot.yaml:141,166）。

| # | 名称 | 启用条件 | 位置增量公式 | 速度公式 | 行号 |
|---|---|---|---|---|---|
| 1 | tuned（旧式） | track_mode != "v1" 且非 pid | pitch_delta = eff_y*track_inc_gain_ + vy*predict_gain_；yaw_delta = eff_x*track_inc_gain_ | trackSpeedFor：kp*abs_err + kd*d_abs_err，地板 track_speed_floor_，上限 trackSpeedCap() | 170, 222, 255-256；函数 604-634 |
| 2 | v1（上游复刻） | track_mode == "v1" | pitch_delta = eff_y*v1_inc_gain_ + vy*v1_predict_；yaw_delta = eff_x*v1_inc_gain_；yaw_lead = vx*v1_predict_ | v1TrackSpeedFor：kp*abs_err + kd*d_abs_err，地板 v1_speed_floor_，上限 v1SpeedCap() | 157-161, 208-212, 248-252；函数 748-772 |
| 3 | pid 增量式 | pid_enable_ && track_mode=="pid" | u = pidStep(eff_y, dt, false)；pitch_delta = u*dt + vy*predict_gain_；yaw 同理 | pidSpeedFor(pid_rate_*, eff)：速率幅值，地板/上限同 trackSpeedCap() | 162-167, 213-219, 243-247；函数 666-731 |
| 4 | 俯仰符号 / 钳位口径 | 所有模式 | pitch_inc = lround(pitch_delta * pitch_deg_sign_)；跟踪用 clampPitchTick（950~2400）；扫描用 clampPitchToScan | — | 172, 200-202, 183-194；函数 477-543 |
| 5 | 车体前馈口径 | gyro_ff_gain_ != 0 | ff_x = -ωz*57.3*yaw_ticks_per_deg_*dt*gain*sign，叠加进 inc_x | — | 141-148, 225 |

pidStep 详解（666-710）：输出是速率 u（tick/s），调用方再做 Δθ = u·dt。

- dt 兜底：dt<=1e-6 或 dt>0.5 → dt=0.0333（668-671）
- D 项对滤波后误差做差分并过一阶低通：dfilt = pid_d_alpha_*dfilt + (1-pid_d_alpha_)*d_raw（688-689）
- u_pd = pid_kp_*e + pid_kd_*dfilt（691）
- 条件积分：仅当 |u_pd| < 0.95*cap 时才累积 integ += pid_ki_*e*dt，并限幅 ±pid_i_clamp_（694-706）
- 返回 u_pd + integ（709）

trackSpeedCap()（637-642）：track_speed_max_ > 0 时用绝对值，否则 patrolSpeed()*track_speed_max_ratio_。这是位置增量与下发速度共用的唯一限速源（注释强调避免两处限速打架）。

resetPid()（645-653）：进入 TRACK 时清零所有 PID 状态与 inited 标志。

⚠ 口径陷阱：旧式 tuned 里位置增量按 inc_gain、速度按 kp/kd 是两套口径，位置先到、速度还在冲（dzrknn_dataprocess.h:243-249 注释）。pid 模式让二者同源。

#### 2.3.5 开火判定（261-392）

开火需同时满足：

1. in_fire_zone（302）：
   - fire_dz = fireDeadzonePx(box_w)（265）——动态框：框宽在 [box_w_far_=30, box_w_near_=120] 线性映射到 [fire_dz_far_px_=25, fire_dz_near_px_=26]（776-799）。注意配置里 far/near 几乎相等（25/26），动态框基本退化为固定框。
   - aim_err_x = |dx_filt_ + lead_applied - laser_offset_x_|；aim_err_y = |dy_filt_ - laser_offset_y_|
   - lead_applied = lead_apply_gain_ * lead_px_（299）；lead_px_ 来自 /aim/lead_x（回调 1169-1178，范围守卫 |v|>400 或非有限 → 0）
   - laser_offset_x_/y_ 来自配置（默认 0，robot.yaml:212-213）
2. aim_ready_effective（281-291）：
   - fire_use_image_stable_（配置 true）：aim_stable = |vx|<=fire_stable_px_per_frame_(1.5) 且 |vy|<=1.5（279-280）——画面稳定，移动中也能开火
   - 否则用旧判据 motor_still：下位机回传速度 < fire_real_speed_max_(20) 且指令速度 < fire_speed_max_(50)（268-273）
   - use_external_fire_gate_（配置 true）时改为两键：external_fire_ok = external_gate_ && external_ready_（290），即 /aim/fire_gate 与 /aim/fire_ready 都为 1
3. 类别锁（303-333）：cls_switch_confirm_frames_（配置 5）帧迟滞。raw_cls==locked_cls_ → 候选作废；raw_cls==cls_candidate_ → streak++，达阈值改锁并打日志；否则换候选、streak=1。
   - cls_fire_ok = !fire_only_blue_ || locked_cls_==1（333）；配置 fire_only_blue: true
4. 弹药（352-358）：
   - per_target_ok：shots_on_target_ < max_shots_per_target_（配置 10）
   - ammo_ok：max_shots_>0 且 shots_fired_ < max_shots_（配置 10，硬上限 AIM_MAX_SHOTS=10，dzrknn_dataprocess.h:294）
   - cooldown_ok：距上次开火 >= shot_cooldown_ms_（500）
5. fire_hold_frames_（360-391）：连续满足 fire_hold_frames_（配置 5）帧才真正开火。开火时连发 LaserShot_Command=1 再 =0（365-369），shots_fired_++、shots_on_target_++、发 /shoot_count（377-379）。

失败/丢失时：fire_hold_count_=0；若 isTracking() 但 !cls_fire_ok，1 s 限流打诊断日志（334-351）。

⚠ 坑：dzactuator.cpp:79 的 if (!v == 0) 恒真（!v 是布尔，与 0 比较恒 true）。这是 DWA 回调里的 bug——TargetAngle 分支因此永远走 atan 分支，else（TargetAngle=0）永远不会执行。

#### 2.3.6 扫描状态机

- startScan(continuous, home=true)（877-905）：
  - home=true（默认）：重置 scan_yaw_idx_=0、scan_yaw_dir_=1、scan_pitch_idx_=0、scan_pass_done_=0、scan_cycles_done_=0；设 scan_homing_=true、scan_home_deadline_ = now + max(1, scan_home_timeout_ms_)/1000（1500 ms）。先归位再开扫。
  - home=false（t42 重扫接续）：不重跑归位段，保留扫掠进度，重置 scan_cycles_done_=0，直接 enterScanPhase()（880-889）。
- scanHomingStep()（910-933）：到位判据 d_yaw/d_pitch <= scan_arrive_tol_(30)；或超时。返回 true 即归位完成。
- enterScanPhase()（936-942）：scan_homing_=false，applyScanTarget() 下发第一个扫描点，setAimState(AIM_SCAN)。
- callback_scan_request（951-975）：
  - AIM_TRACK 时忽略（不打断跟随）
  - data==0 → scan_continuous_=false，setAimState(AIM_IDLE)
  - AIM_SCAN 时只更新模式，不重置进度（避免反复请求把扫描拽回起点）
  - 否则 startScan(data>=2)
- CaremaMontorControl()（977-1101）：
  - TRACK：视觉静默 > vision_timeout_ms_(1000) → 清 lost_target_count_，scan_continuous_ 时 startScan(true, false) 接续扫掠，否则 setAimState(AIM_IDLE)（984-1001）
  - IDLE：每帧下发归位目标（yaw=行程中点、pitch=scanPitchDeg），速度 = patrolSpeed()*idle_homing_speed_ratio_(0.5)（1013-1034）。这是改参数立刻可见的关键——旧版 IDLE 时直接 return，云台冻结在任意姿态，导致调 scan_pitch_offset_deg 无效。
  - SCAN：每 scan_dwell_ms_(300) ms 检查是否到位/超时 → advanceScanIndex()（1040-1098）。超时兜底 = max(scan_dwell_ms_*3, traverse_ms*1.5)（1051-1052）。扫完一轮：连续模式且 scan_cycles_done_ >= scan_continuous_cycles_(3) → 回 IDLE（1068-1074）；否则立刻从头再来并打进度日志（1076-1088）；单次模式 → setAimState(AIM_IDLE)（1092-1094）
- advanceScanIndex()（819-838）：scan_yaw_dir_ *= -1；每 scan_passes_ 趟换 pitch 层；扫满 scan_passes_*pitchLevelCount() 趟 → scan_finished_=true，scan_cycles_done_++。
- applyScanTarget()（802-815）：yaw_tick = center + lround(scanYawDeg()*yaw_ticks_per_deg_*yaw_deg_sign_)；pitch_tick 同理；速度 = patrolSpeed() = gimbal_max_speed_*patrol_speed_ratio_（配置 3700*0.30=1110）。

#### 2.3.7 车体转动前馈（惯性凝视稳定）

- callback_imu（1136-1140）：imu_omega_z_ = msg->angular_velocity.z（rad/s）。
- 前馈公式（147）：ff_x = -imu_omega_z_ * 57.2957795 * yaw_ticks_per_deg_ * dt * gyro_ff_gain_ * gyro_ff_sign_。
- 配置：gyro_ff_gain=0.8、gyro_ff_sign=1.0、imu_ff_timeout_ms=200（robot.yaml:242-244）。
- 注释说明：增益 0 时完全关闭，行为与旧版一致（134）。

#### 2.3.8 Control() 主循环

循环体（1689-1724）：

1. spin_some
2. CaremaMontorControl()（状态机）
3. 组装 Int32MultiArray[4] = {Position_0, Position_1, Speed_0, Speed_1}（1696-1701）
4. 若 !manual_override_ 则发 /CameMo_Conl（1704-1707）
5. 状态上报保活（1714-1720）
6. 按状态选频率 sleep（1723）

#### 2.3.9 launch / CMake / 查表

- dzrknn_dataprocess.launch.py:42-48：Node(package='dzrknn_dataprocess', executable='dzactuator_node', name='dzrknn_dataprocess', parameters=[CONFIG_FILE])。
- CMakeLists.txt:91：add_executable(dzactuator_node src/dzrknn_dataprocess.cpp)；安装到 lib/dzrknn_dataprocess（101-106）。offcenter_table.txt 装到 share（71-72）。
- loadLookupTable()（1194-1241）：逐行 stoi 解析 offcenter_table.txt；当前文件 23 行全 0（offcenter_table.txt:1-23）。
- compute_comp_pixel_0(tar_dis)（1180-1191）：tar_dis/0.25 取索引查表；未使用（预留）。
- 模型路径解析：ament_index_cpp::get_package_share_directory("dzrknn_dataprocess") + "/offcenter_table.txt"，失败回退源码绝对路径（1660-1672）。

⚠ 可执行名歧义（P1-2）：dzrknn_dataprocess 包安装的可执行叫 dzactuator_node（CMakeLists.txt:91），与 dzactuator 包的可执行同名（dzactuator/CMakeLists.txt:87）。两者都装到各自的 lib/<pkg>/ 下，因此 ros2 run dzactuator_node 会歧义——必须写 ros2 run dzrknn_dataprocess dzactuator_node 或用 launch。

### 2.4 dzjudgment（裁判串口 + 激光开火）

#### 2.4.1 概述

裁判/激光驱动节点。它订阅开火指令，经串口写 6 字节帧到激光模块，同时接收裁判板状态帧并转发为 ROS 话题。这是开火链路唯一真正点亮激光的一环——start_all.sh:194-202 有醒目注释：以前只起 dzrknn_dataprocess，导致 /LaserShot_Command 的 Subscription count = 0，开火指令全进空话题，激光永不亮且无任何报错。

线程模型：单线程 main → node->run()（dzjudgment.cpp:15-22；run 见 judgment.cpp:119-160）。run() 以 100 Hz 循环（int run_rate = 100;，judgment.cpp:127）每拍 spin_some + recvESP32Info + 每 100 拍 publish_referee_state。原为 10 Hz，是按下开火后延迟很大的主因（judgment.cpp:121-125）。

参数：mcubaudrate（默认 115200）、mcuserialport（默认 /dev/ttyACM0；配置中心为 /dev/dzjudgment → ttyACM1）（judgment.cpp:5-11；robot.yaml:290-291）。

#### 2.4.2 串口协议

##### (a) 下行：开火帧（6 字节）

sendLaserShot()（judgment.cpp:358-401）：

    buf[0]=0xB5  buf[1]=0x5B  buf[2]=0x03  buf[3]=0x21  buf[4]=0x01  buf[5]=0x25

固定帧，无参数。成功时打 WARN 日志「★ 开火指令已下发 (B5 5B 03 21 01 25)」（390-391）——作者强调开火是一次性动作，成功必须留痕，否则无法区分「指令没送到」与「送到但硬件没响应」。

##### (b) 下行：通用指令通道

/referee_cmd（UInt8MultiArray）→ 原样写串口（judgment.cpp:85-117）。用于下发开启/关闭裁判系统、重置计数等代码里不知道的协议帧（judgment.cpp:54-58 注释）。空帧被忽略（87-91）。

##### (c) 上行：裁判板状态帧

recvESP32Info()（judgment.cpp:205-283）。帧头 0xB5 0x5B，第三字节 dataLength，随后 dataLength 字节帧体。

- 有界非阻塞：每次最多处理 8 帧（guard<8，228），ser.available()<=0 立即 return（230-232）。
- 半帧状态靠成员 previousHeader2 跨调用保留（237-256）。
- 帧体不足时 return 等下个周期（263-273）。

processData()（judgment.cpp:285-356）按 charArray[3]（帧体首字节）分流：

| charArray[3] | 语义 | 处理 | 行号 |
|---|---|---|---|
| 0x02 | 全场物资 | 清零并填 all_Material_Number，发 /all_Material_Number | 302-310 |
| 0x03 | 敌方物资 | 同上，发 /enemy_Material_Number | 311-319 |
| 0x04 | 己方物资 | 同上，发 /self_Material_Number | 320-328 |
| 0x07 | 血量/击打/弹药 | targethp=charArray[4]、self_hp=[5]、Bing_hit=[6]、Shooting_Count=[7]、ammo=[8]，发 /HpAndHitmsg | 329-351 |

⚠ 长度守卫（code_review C5）：case 0x07 要求 dataLength >= 6 才读 charArray[4..8]（333-339），否则丢弃——否则会读越界栈内存。

##### (d) 状态上报

publish_referee_state()（judgment.cpp:193-203）：每 100 拍（约 1 s）发 /HpAndHitmsg，字段 [对方血量, 己方血量, 被击打方位, 累计射击次数, 剩余弹药]。

##### (e) 掉线重连

reconnectSerial()（judgment.cpp:162-191）：关闭后重试 3 次，间隔 1 s，失败返回 false。所有串口操作 catch serial::SerialException（judgment.cpp:137-147 注释：IOException 是子类，只 catch 子类会漏掉父类异常 → SIGABRT）。

#### 2.4.3 launch 与 CMake

- dzjudgment/launch/judgment.launch.py:50-57：Node(package='dzjudgment', executable='dzjudgment', parameters=[CONFIG_FILE])。注释完整描述了开火链路与历史坑。
- dzactuator/launch/judgment.launch.py:6-16：另一份副本，硬编码 mcubaudrate=115200、mcuserialport=/dev/dzjudgment，不走配置中心。⚠ 两份 launch 同名不同包，start_all.sh:202 用的是 dzjudgment 包那份。
- CMakeLists.txt:44：add_executable(dzjudgment src/dzjudgment.cpp src/judgment.cpp)；依赖 rclcpp/Boost/serial/geometry_msgs/nav2_msgs/tf2（45）。

⚠ 坑：dzjudgment.cpp:3 与 13 各 #include 一次 judgment.h（重复包含，靠头文件 guard 兜住）。dzjudgment.cpp:3-12 有一段被注释掉的旧 main。

### 2.5 rknn_pt / det_node（RKNN 视觉检测）

#### 2.5.1 概述

det_node 订阅相机 /image_raw，用 RKNN NPU 跑 YOLOv8-pose 模型，输出 8 字段的 /offset_center。模型类别 2（0=blue_target、1=red_target，postprocess.cc:8），关键点 1 个（KPT_NUM=1、KPT_DIM=3，postprocess.h:22-23）。

线程模型：DetNode 构造时起一个 wall timer，周期 = max(33 ms, 1000/max_fps)（det_node.cc:67-74）。processTimer() 里做 put/get 推理池。模型推理在 rknnPool 的线程池里跑（rknnPool.hpp:53-71），主线程只提交与取结果。

关键参数（det_node.cc:37-47）：
- model_path（默认空 → 用包内 share/det_node/model/light_det.rknn）
- draw（默认 true，false 省掉 30 Hz imshow+waitKey 开销）
- thread_num（默认 4；配置中心已改为 2）
- max_fps（默认 30.0）
- hot_reload（默认 true）

⚠ 实测（AIM-LAT-20261022，robot.yaml:305-307）：8 线程 30.4 Hz 但流水线滞后 8 帧约 264 ms；2 线程 30.4 Hz、滞后约 2 帧 66 ms、CPU 160→113%；1 线程吞吐不足掉到 17 Hz。故 thread_num 定为 2。

#### 2.5.2 /offset_center 消息格式（det_node.cc:215-259）

    [0] centerx 偏移   [1] centery 偏移   [2] 类别(1=蓝 2=红 0=无)
    [3] 框宽           [4] 框高           [5] 关键点x偏移   [6] 关键点y偏移   [7] 关键点置信度*100

计算（244-258）：

    center_x = res.box.x + res.box.width/2
    center_y = res.box.y + res.box.height/2
    offset_center_x = center_x - width/2
    offset_center_y = center_y - height/2
    data[2] = res.cls + 1          # 0=blue 1=red → 发布 1=blue 2=red
    data[5] = res.kpt_x - width/2
    data[6] = res.kpt_y - height/2
    data[7] = res.kpt_conf * 100

⚠ 只发布最后一个检测结果（for 循环覆盖同一 msg），多目标时只剩分数最高的一个。

#### 2.5.3 推理流水线

1. imageCallback（det_node.cc:184-195）：cv_bridge::toCvShare → clone，加 img_mtx_ 锁。
2. processTimer（197-275）：checkHotReload → 取帧（空则等待提示）→ detectPool_->put(frame, frames_) → detectPool_->get(results_group) → 后处理填 msg → publish → 可选 draw。
   - 限帧由定时器周期完成（注释说明：以前在回调里卡时间戳，定时器 33 ms + 阈值 33.3 ms 的边界效应会让频率腰斩到约 15 Hz，实测 30.3→15.6）（det_node.cc:62-66）。
   - 热加载 checkHotReload（147-182）：每 1 s 最多 stat 一次 mtime，变化则 loadPool()；loadPool 先建新池、成功才替换旧池（118-141）。
3. RkPt::infer（rkpt.cc:205-298）：
   - BGR→RGB（209）
   - 用 RGA 硬件 resize（resize_rga，preprocess.cc:28-53；失败只打 stderr 不中断）
   - rknn_inputs_set → rknn_run → rknn_outputs_get（want_float=1，250-260）
   - 从张量属性推 num_preds（最大维）与 num_cols = n_elems/num_preds（268-279）
   - post_process(...)（281-284）
   - 输出 shape [1, num_cols, num_preds]，取值 output[channel*num_preds + index]
4. post_process（postprocess.cc:73-169）：
   - 类别分数取最大（96-106），低于 conf_threshold(BOX_THRESH=0.4) 跳过
   - cx,cy,w,h → 左上角+宽高（113-127）；bw<=1 或 bh<=1 跳过
   - 关键点 kpt_shape [1,3] → kx,ky,kconf（132-143）
   - nms（39-66）：按分数降序，同类 IoU > nms_threshold(0.45) 抑制
   - 映射回原图：clamp 到 [0, model_in_w/h] 后 /scale_w/scale_h（154-163）；kpt 同理
5. 线程池：rknnPool（rknnPool.hpp）持有 threadNum 个模型实例，每个绑定不同 NPU 核（rkpt.cc:112-126 通过 get_core_num() 轮转 RKNN_NPU_CORE_0/1/2；coreNum.hpp:12-22）；父 context 通过 rknn_dup_context 复用权重（rkpt.cc:102-105）。put/get 用同一把 queueMtx（rknnPool.hpp:91-111）。ThreadPool.hpp 是通用池，worker 空闲 2 s 后回收（WAIT_SECONDS=2，150）。

#### 2.5.4 launch / CMake

- det_node.launch.py:28-34：Node(package='det_node', executable='det_node', parameters=[CONFIG_FILE])。注释说明已移除三个从未 declare 的参数（robot_color / tracking.* / show_results，ROS 2 会静默丢弃）。
- CMakeLists.txt:18-23：add_executable(det_node src/det_node.cc + det/{postprocess,preprocess,rkpt}.cc)；链接 OpenCV、librknnrt.so（必需）、librga.so（可选）；RPATH 指向源码 lib（59-62）；安装 lib/ 下的 .so（68-71）、launch（73-74）、param（76-80）、model（82-86）。

⚠ 坑：det_node.cc:100-108 优先从 ament share 目录解析模型路径，失败回退 /home/duzhong/dzrobot/src/rknn_pt/model/light_det.rknn（硬编码用户名）。源码撤离车上后此回退会失效。

### 2.6 配置中心（config/robot.yaml）

#### 2.6.1 顶层结构（ROS params 文件约束）

robot.yaml 共 968 行，7 个段：/dzrknn_dataprocess（31）、/usb_cam_node（250）、/dzjudgment（288）、/det_node（297）、path_executor（319）、/dzbrain（613）、/dzactuator（907）。顶层裸键会让 --params-file 解析失败，导致 usb_cam/det_node/dzrknn_dataprocess 三节点同时 abort(exit -6)（healthcheck.sh:109-137 有专门校验）。

#### 2.6.2 /dzrknn_dataprocess 段（31-245）

关键键：

| 键 | 值 | 行号 | 说明 |
|---|---|---|---|
| scan_yaw_min_deg / scan_yaw_max_deg | -150.0 / 150.0 | 37-38 | t25 用户指令，左右各 150°；同时被 TRACK 的 clampYawToScan 当跟踪水平允许范围 |
| scan_pitch_offset_deg | +10.0 | 50 | t52；2026-10-15 实测 + 才向下压、- 向上抬，符号约定更正 |
| scan_pitch_levels | [0.0] | 65 | 只做水平扫描 |
| scan_passes | 2 | 68 | 一轮 = 左一趟 + 右一趟 |
| scan_continuous_cycles | 3 | 72 | 连续扫描最多 3 轮 |
| scan_home_timeout_ms | 1500 | 75 | 进扫描前归位段超时 |
| patrol_speed_ratio | 0.30 | 81 | 扫描速度 = 最大转速 × 0.30 |
| gimbal_max_speed | 3700 | 82 | 最大转速 |
| track_speed_max | 2000 | 83 | 跟踪速度上限（与 patrol 解耦） |
| track_speed_max_ratio | 0.63 | 86 | 仅 track_speed_max=0 时生效 |
| loop_hz_track / loop_hz_idle | 100 / 50 | 94-95 | 自适应主循环频率 |
| gimbal_center_tick | 2047 | 100 | 零位刻度 |
| yaw_ticks_per_deg / pitch_ticks_per_deg | 11.378 | 101-102 | 4096/360 |
| yaw_deg_sign / pitch_deg_sign | 1.0 / 1.0 | 103-104 | 方向符号 |
| tracking_pitch_margin_deg | 6.0 | 111 | 扫描俯仰区间外的微调余量 |
| idle_homing_speed_ratio | 0.5 | 114 | 空闲归位速度比例 |
| gimbal_yaw_tick_min/max | 100 / 3995 | 115-116 | 水平轴硬件限位 |
| gimbal_pitch_tick_min/max | 950 / 2400 | 117-118 | 竖直轴下位机钳位 |
| ema_alpha | 0.45 | 123 | 视觉低通 |
| track_deadzone_px | 5 | 124 | 须 < fire_dz_far_px |
| track_kp / track_kd | 20.0 / 15.0 | 125-126 | 旧式 PD（pid 模式下不生效） |
| track_inc_gain | 1.2 | 127 | 旧式位置增益（pid 模式下不生效） |
| track_speed_floor | 30 | 128 | 速度下限 |
| fine_zone_px | 40 | 129 | 精调档阈值 |
| fine_kp_scale / fine_kd_scale | 0.5 / 2.0 | 130-131 | 精调档缩放 |
| predict_gain | 0.5 | 132 | 目标移动预判强度 |
| track_mode | pid | 141 | 当前生效模式 |
| v1_inc_gain / v1_predict | 0.5 / 0.5 | 144-145 | v1 位置增量 |
| v1_kp / v1_kd | 8.0 / 4.5 | 146-147 | v1 速度 PD |
| v1_ema_alpha / v1_deadzone_px | 0.3 / 12 | 148-149 | v1 滤波/死区 |
| v1_fine_zone_px / v1_fine_kp_scale / v1_fine_kd_scale | 20 / 0.5 / 2.0 | 150-152 | v1 精调档 |
| v1_speed_floor / v1_speed_max_ratio | 100 / 0.63 | 153-154 | v1 地板/上限比例 |
| v1_speed_max_abs | 1700 | 155 | v1 绝对上限 |
| v1_yaw_full_range | true | 157 | v1 水平整量程 |
| pid_enable | true | 166 | 当前生效开关 |
| pid_kp / pid_ki / pid_kd | 12.0 / 0.5 / 2.4 | 167-169 | 用户 2026-10-18 指定 |
| pid_i_clamp | 600.0 | 170 | 积分限幅 |
| pid_d_alpha | 0.3 | 171 | D 项低通 |
| pid_min_step | 1 | 172 | 最小位置步长 |
| fire_dz_dynamic | true | 177 | 动态开火框 |
| fire_deadzone_px | 8 | 178 | 仅动态关时使用 |
| fire_dz_far_px / fire_dz_near_px | 25.0 / 26.0 | 179,184 | 远/近开火死区 |
| box_w_far / box_w_near | 30.0 / 120.0 | 185-186 | 框宽标定 |
| fire_hold_frames | 5 | 187 | 连续多少帧才开火 |
| fire_use_image_stable | true | 188 | 画面稳定判据 |
| fire_stable_px_per_frame | 1.5 | 189 | 画面稳定阈值 |
| fire_real_speed_max | 20 | 190 | 电机静止阈值（旧判据） |
| fire_speed_max | 50 | 191 | 开火时指令速度上限 |
| target_confirm_frames | 2 | 192 | 扫描确认帧数 |
| cls_switch_confirm_frames | 5 | 193 | 类别改锁帧数 |
| shot_cooldown_ms | 500 | 197 | 两发间隔 |
| fire_only_blue | true | 202 | 只打蓝靶 |
| max_shots | 10 | 203 | 全程弹药 |
| max_shots_per_target | 10 | 207 | t26 改为 10 |
| laser_offset_x / laser_offset_y | 0.0 / 0.0 | 212-213 | 激光同轴标定 |
| use_external_fire_gate | true | 227 | 两键开火 |
| lead_apply_gain | 0.5 | 228 | 提前量施加增益 |
| aspect_min / aspect_max | 0.60 / 1.60 | 234-235 | 目标框长宽比过滤 |
| lost_frames_to_rescan | 180 | 236 | 丢帧回扫描阈值 |
| vision_timeout_ms | 1000 | 237 | 视觉静默超时 |
| gyro_ff_gain / gyro_ff_sign | 0.8 / 1.0 | 242-243 | 车体转动前馈 |
| imu_ff_timeout_ms | 200 | 244 | IMU 失联阈值 |

⚠ 三条硬性不变量（healthcheck.sh:82-100）：
1. track_deadzone_px < fire_dz_far_px（否则目标停在死区里，一发打不出）
2. fire_dz_far_px > 机械精度约 12 px（t43 曾放宽注释，t53 又改回）
3. lost_frames_to_rescan >= 60 帧（约 2 s，覆盖视觉抖动周期）

当前值 5 < 25 ✅、25 > 12 ✅、180 >= 60 ✅。

#### 2.6.3 其余段

- /usb_cam_node（250-260）：video_device=/dev/video0、640×480、framerate 30、yuyv、mmap、brightness 0（曾因 50 过曝）。
- /dzjudgment（288-291）：mcuserialport=/dev/dzjudgment、mcubaudrate=115200。
- /det_node（297-311）：model_path=""、draw=true、thread_num=2、max_fps=30.0、hot_reload=true。
- path_executor（319-600）：
  - linear_speed: 0.35（333）；注释详述 0.15→0.40 的 t38 提速档历史
  - auto_start: true（336）；auto_start_delay: 45.0（337）
  - mission_scan_mode: continuous（338）；mission_reset: "N10,y"（339）
  - reverse_speed_ratio 0.6、reverse_steer_sign 1.0（343-344）
  - bridge_blocks: []（355）；bridge_order（361-363）
  - align_straight_*（364-368）
  - shoot_distance_target 0.35（373）、shoot_distance_min 0.78 / shoot_distance_max 1.02（399-400）、shoot_max_retry 3（408）、shoot_score_w_gimbal 0.6 / w_detour 0.5（409-410）
  - shoot_count_target 10（423）、shoot_timeout 30.0（424）、shoot_settle_speed 0.02（426）、shoot_stable_frames 5（427）、shoot_stable_offset_px 40.0（428）、shoot_stable_timeout 3.0（429）
  - obstacle_blocks（432）
  - shoot_settle_time 0.4（439）、shoot_lock_timeout 6.0（440）、shoot_overshoot_tol 0.30（441）、shoot_slowdown_m 0.60（443）
  - map_safety_margin 0.60（450）、drift_warn_m 0.30（456）、finish_voice_id 0（459）
  - mission_blocks：68 token（532-600），含 I4:PIVOT / G4:PIVOT / A1:PIVOT / B4* / N5,y
- /dzbrain（613-900）：pursuit 段（wheelbase_m 0.157、max_steer_deg 30、max_steer_rate_deg_s 120、curvature_gain 1.0、lookahead_min_m 0.36、lookahead_max_m 0.40、lookahead_k_v 1.2、cruise_speed_mps 0.35）、map_safety_margin_m 0.60、drift_warn_m 0.30、drift_seg_gate_m 0.18、bridge_blocks []、cross_lower/upper_candidates [K7]/[K5]、shoot_distance_m 0.90 / shoot_distance_tol_m 0.12、auto_start false（703）、auto_start_delay_s 10.0（704）、reset "N10,y"、scan_mode continuous、shots_per_stop 10、shoot_timeout_s 30.0、lock_timeout_s 6.0、settle_time_s 0.4、engine tour、fallback_to_tokens true、allow_reverse false、max_seconds 2.0、pivot_penalty 5.0、mine_turn_penalty 2.0、obstacle_margin_m 0.02、lead_time_s 0.27、fire_policy moving_lead、rail_span_m 0.70、target_speed_mps 0.04、px_per_m_rail 140.0、max_shots_total 10、max_shots_per_target 10、shot_cooldown_ms 500、lost_frames_to_rescan 60、scan_pitch_offset_deg +10.0、anchor_at_waypoint true、anchor_max_pull_m 0.12、drift_slow_after 3、drift_slow_speed_mps 0.08、drift_stop_after 8、drift_clear_ticks 5、boundary_guard_m 0.25、boundary_stop_m 0.10、yaw_trim_deg -0.6、heartbeat_timeout_s 3.0、info_stale_s 30.0、pose_timeout_s 1.5、task_stall_s 1.5、startup_grace_s 8.0、brake_on_fault true；以及 /dzactuator 段的 vehicle（length_m 0.28、width_m 0.135、coverage_radius_m 0.035、ref_offset_m 0.075）与 field（obstacle_radius_m 0.1275、mine_pad_m 0.12、barrier_width_m 0.02、barrier_cols "D..N"）。
- /dzactuator（907-968）：serial_port_name=/dev/dzactuator、serial_baud_rate=115200、odom/robot/gyro frame id、calibrate_lineSpeed=0、ticksPerMeter=3280.0、ticksPer2PI=2680.0、vehicle/field 物理几何、smooth_window_speed 20、smooth_window_angle 5。

⚠ 未核实：/dzbrain 段的键与 dzbrain 代码 SCHEMA 的对应关系未在本报告中逐键核对（超出本次车端硬件范围）。

#### 2.6.4 map.yaml 与 map_v3.yaml

- config/map.yaml（41 行）：第六张图（2026-10-17 用户现场换图）。雷 A2 F4 H4 G5 E8 B10 D11 I11；障 B1 D1 F3 I6 H7 A8 C9 E9；靶 D5+D6（纵向跨格）；通口 K5（上半场）/K7（下半场）；起终 N10/N5。barrier_col_from=D、barrier_row_hi=6、barrier_row_lo=7。
- config/map_v3.yaml（46 行）：第四张图（2026-10-11），历史备份。雷 B2 H2 F3 C4 G4 I7 D8 E10；障 A5 G5 B6 D7 C8 E9 I9 H10；靶 E4+E5；通口 K1/K11。
- ⚠ robot.yaml 的 obstacle_blocks（432）与 map.yaml 的 obstacles 需保持一致（route_check.py 会校验）。本报告只确认两文件当前都指向第六张图。

#### 2.6.5 备份文件

- robot.yaml.bak / robot.yaml.t39bak / robot.yaml.bak_yawfloor_20261018 / robot.yaml.corrupt_20261017_161718。⚠ 未核实各备份的时间点与差异。

### 2.7 部署与运维工具

#### 2.7.1 deploy_to_car.py（470 行，唯一部署入口）

流程：probe → backup → push → build（all = backup→push→build）；verify 为独立可选子命令（不包含在 all 里）。

- 常量（48-52）：CAR_HOST=duzhong.local、CAR_USER=duzhong、CAR_WS=/home/duzhong/dzrobot、SSH_OPTS 含 BatchMode=yes / StrictHostKeyChecking=no / ConnectTimeout=10。
- PUSH_FILES（55-74）：src/dzbrain（整包目录）、dzrknn_dataprocess 的 .h/.cpp、config/robot.yaml、tools/config_selfcheck.py、tools/dzbrain.py、tools/healthcheck.sh。不推 config/map.yaml（车机不留地图文件，布局由无人机 field_layout 下发，见 62-65）。
- BACKUP_PATHS（77-83）：config/robot.yaml、start_all.sh、src/dzbrain、src/dzrknn_dataprocess。
- BACKUP_ROOT=/home/duzhong/_backups（87）：必须放在工作区之外，否则 colcon 报 Duplicate package names（85-86）。
- BUILD_PACKAGES=["dzbrain","dzrknn_dataprocess"]（89）。

关键技术点：
- 目录推送必须用 tar 流就地替换（234-247）：先解到 <remote>.dznew，再 rm -rf + mv 原子顶掉。注释说明 scp -r 会把源目录塞进目标目录变成嵌套，导致推送成功但一个文件都没更新（2026-10-08 真机踩过）。
- 推送后哈希对账（306-328）：逐项比对本地/车端 sha256；missing 或 mismatch 即 bad，返回 1 中止编译。
- 单文件 push 用 scp（250-253）；二进制 tar 走 stdin，绝不经 PowerShell 管道（132-143）。
- normalize_auto_start（177-179）：把 auto_start: true 归一为 false。默认 push 不碰车端 auto_start；只有 --force-no-autostart 才改写（370-386），且对账基准同步用改写后内容（expected_remote_bytes，191-208）。
- cmd_build（389-402）：构建 dzbrain 前 rm -rf build/dzbrain install/dzbrain（防旧布局残留），colcon build --packages-select … --cmake-args -DCMAKE_BUILD_TYPE=Release。
- cmd_verify（405-420）：只做两项瞬时检查——dzbrain 包/入口存在性（ros2 pkg executables/prefix）、云台补丁符号 grep -c use_external_fire_gate。
- 安全约定（18-28）：绝不在验证阶段启动 ROS 栈；push 前自动 backup；默认不碰 auto_start。

⚠ 坑：PUSH_FILES 只覆盖 5 条路径，但备份 BACKUP_PATHS 也是 4 条；start_all.sh 不在 push 清单里（只在 backup 里）。若本地改了 start_all.sh，部署不会同步上车。

#### 2.7.2 start_all.sh（248 行）

已在 §1.3 覆盖启动时序。补充运维要点：

- 进程管理（35-92）：spawn() 用 setsid 起进程组并写 PIDFILE（47-59）；kill_group() 对整个进程组先 TERM 后 KILL（61-72）；cleanup_strays() 扫 /proc 清掉属于本工作区但不在 PID 记录里的残留（74-92）。注释强调不再用 pkill -f（会误伤、会漏杀，已出两次事故：12 个 static_transform_publisher 残留、相机抢不到 /dev/video0 abort exit -6）。
- 域号（114-126）：source dzros.env（全车唯一真相源），DZ_ROS_DOMAIN 兜底 118，外部已 export 的 ROS_DOMAIN_ID 优先。
- set -u 与 ROS setup.bash 冲突：source 期间临时 set +u（128-136）。
- 配置中心 YAML 可解析守卫（145-156）：解析失败只提示不拦截。
- GUI_DISPLAY 默认 :1（30）；4_det 步骤显式 env DISPLAY（52-54）。
- stop/restart 分支（138-141）；restart 先 stop_all 再 sleep 2。

#### 2.7.3 restart_node.sh（116 行）

8 个槽位：gimbal / camera / vision / path / judgment / chassis / loc（node_spec，28-48）。按 PIDFILE 精确停旧实例（进程组回收），再 setsid 起新实例（74-102）。path 槽位必须与 start_all.sh 保持一致（默认 dzbrain，LEGACY_PATH=1 回退 path_executor_node，33-42 注释强调旧版写死直跑 path_executor.py 会悄悄切回旧核）。vision 槽位自动加 DISPLAY（96）。gimbal 重启后回显 瞄准:/射击: 参数横幅（108-111）。

#### 2.7.4 healthcheck.sh（220+ 行）

6 节体检：设备（/dev/video0、ttyACM*/ttyUSB*）、配置中心与不变量、params 结构、declare/get 配对、狙击链路、模型文件。退出码 0=全过、1=有 ❌、2=只有 ⚠️。

- 不变量校验（61-100）：Python 读 YAML 逐键比对 track_deadzone_px / fire_dz_far_px / lost_frames_to_rescan / patrol_speed_ratio / scan_pitch_offset_deg / fire_only_blue / max_shots_per_target。
- params 结构校验（115-137）：顶层每个键必须是 dict 且含 ros__parameters——这是 yaml.safe_load 查不出来的 ROS 约束。
- declare/get 配对（139-179）：正则扫 declare_parameter("x") 与 get_parameter("x")，报出声明了却没取的键。⚠ 注释说明：scan_pitch_offset_deg 曾声明了却从未 get，导致改了三次毫无效果。源码撤离车上后此项退化为提示。
- 狙击链路（181-201）：检查 /dev/dzjudgment 存在、dzjudgment 节点在跑、/LaserShot_Command 的 Subscription count >= 1。
- 模型文件（203-211）：优先 install/det_node/share/det_node/model/light_det.rknn。

#### 2.7.5 dzros.env（18 行）

DZ_ROS_DOMAIN 默认 118；export ROS_DOMAIN_ID（外部已 export 的优先）、ROS_LOCALHOST_ONLY 默认 0。注释强调交互式终端读 ~/.bashrc 得 118，而 systemd/cron/ssh 一行命令不读 → 默认 0，症状是 ping/SSH/网页都通但 ROS 互相看不见且不报错（2026-10-07 无人机↔小车联调事故）。无人机侧对应 ~/dzlink_tools/dzlink.env，两边必须一致。

#### 2.7.6 冻结区边界

tools/wf/frozen.json:3-8 定义冻结区：src/dzbrain/dzbrain/**、src/dzrknn_dataprocess/**、config/**、tools/deploy_to_car.py。in_flight 记录 4 项在途改动（9-37）：

| 路径 | 卡号 | owner | 理由 | 过期 |
|---|---|---|---|---|
| src/dzbrain/dzbrain/tasks/tracker.py | TRACK-R1-20261020-01 | implementer | Tracker 最终发布限速统一钳位 | 2026-10-27 |
| src/dzbrain/dzbrain/route/planner.py | TRACK-PLAN-B1-20261021 | implementer | T-B1 目标函数换口径 + 第七图双停站 | 2026-11-04 |
| src/dzbrain/dzbrain/tools/sim_cli.py | TOOLS-SIM-STUDIO-20261023 | lead | 新增 --json 机器可读契约 | 2026-10-30 |
| config/robot.yaml | AIM-LAT-20261022 | lead | det_node.thread_num 8→2（滞后 264→66 ms，CPU 160→113%） | 2026-10-29 |

⚠ 本报告只读，未改动冻结区任何文件。

### 2.8 已知 bug / 坑汇总

| # | 现象 | 根因 | 位置 | 状态 |
|---|---|---|---|---|
| B1 | DWA 模式下 TargetAngle 永不为 0 | if (!v == 0) 恒真 | dzactuator.cpp:79 | 未修 |
| B2 | 上行帧无校验 | 校验代码被注释掉 | dzactuator.cpp:957-962 | 未修 |
| B3 | Check_Sum 注释说 XOR 实际是累加和 | 实现与注释不符 | dzactuator.cpp:436-439；dzactuator.h:245 | 未修（注释误导） |
| B4 | rx 加速度/角速度各被重复赋值 | 冗余代码 | dzactuator.cpp:980/982、986/989、987/990 | 无害 |
| B5 | 静态参数声明不生效 | declare 了没 get | healthcheck.sh:139-179 会报 | 机制已加 |
| B6 | LaunchConfiguration 传字符串导致类型不匹配静默失败 | launch 传参方式 | 各 launch 文件头注释 | 已改用 params-file |
| B7 | 云台俯仰符号反了 | 多一个负号 | dzrknn_dataprocess.cpp:172（pitch_inc）；修复注释 173-182 | 已修（本地真源） |
| B8 | 跟踪俯仰被钳死 | clampPitchToScan 区间宽度为 0 | dzrknn_dataprocess.cpp:183-202 | 已修（改 clampPitchTick） |
| B9 | 扫描会无限往复 | scan_continuous_ 无轮数上限 | dzrknn_dataprocess.cpp:1068-1074 | 已修（scan_continuous_cycles） |
| B10 | 归位段空转 | 每次丢靶都重跑 1500 ms 归位 | dzrknn_dataprocess.cpp:873-889 | 已修（home=false 接续） |
| B11 | 掉靶高频抖动 | min(180,10)=10 帧即重扫 | dzrknn_dataprocess.cpp:435-439 | 已修（max + 去抖） |
| B12 | 目标闪一下就得重新累计类别锁 | 每次进 TRACK 清类别锁 | dzrknn_dataprocess.cpp:66-73 | 已修 |
| B13 | 位置与速度两套限速打架 | 位置按 inc_gain、速度按 kp/kd | dzrknn_dataprocess.cpp:636-642 | 已修（pid 同源） |
| B14 | 停机后云台被视觉拽回 TRACK | 纯 IDLE 也采信视觉 | dzrknn_dataprocess.cpp:47-50 | 已修 |
| B15 | 开火指令进空话题，激光永不亮 | dzjudgment 没被启动 | start_all.sh:194-202；judgment.cpp | 已修（start_all 第 6 步） |
| B16 | 裁判串口空闲时 CPU 102%、回调全失效 | while(true) 死等帧头 | judgment.cpp:205-283 | 已修（有界非阻塞） |
| B17 | 裁判 case 0x07 帧体过短越界读栈 | 无条件读 charArray[4..8] | judgment.cpp:333-339 | 已修（长度守卫） |
| B18 | 开火延迟大 | run() 是 10 Hz，回调排队 100 ms | judgment.cpp:121-127 | 已修（100 Hz） |
| B19 | /aim/status 类型不匹配 | 网页按 UInt8 订阅，云台发 Int32 | dzrknn_dataprocess.cpp:1261；start_all.sh:188-192 | 网页已下线，shim 已移除 |
| B20 | det_node 频率腰斩 | 定时器+阈值边界效应 | det_node.cc:62-74 | 已修（周期=1/max_fps） |
| B21 | 流水线滞后 8 帧 | thread_num=8 排队 | robot.yaml:305-307 | 已修（thread_num=2） |
| B22 | 相机进程退出后无信号 | /dev/video0 未释放 | camera.launch.py:30-35 | 已修（respawn=True） |
| B23 | 里程计虚增 17.9% | ticksPerMeter 标定错 | robot.yaml:925-932 | 已修（3280） |
| B24 | 航向系统性偏大约 3% | ticksPer2PI 插值尺度约 0.97 | robot.yaml:933-941 | 已改 2680，风险记录待仲裁 |
| B25 | 倒车时车尾方向可能反 | REVERSE_STEER_SIGN 无实车依据 | path_executor.py:192-195；robot.yaml:344-346 | 待实车验证 |
| B26 | 倒车段丢失 | 重复点合并 | path_executor.py:1196-1203 | 已处理（标记不同不合并） |
| B27 | 桥洞保护锁错航向 | 保护窗含 90° 拐 | robot.yaml:351-355 | 已关闭 bridge_blocks |
| B28 | 两个发布者交替导致 90° 跳变 | 两个死推算节点并存 | path_executor.py:880-903 | 机制已加（健康检查） |
| B29 | generate_path 输出行尾多逗号 | 格式不干净 | generate_path.py:62,101 | 未修（当前可用） |
| B30 | 可执行名歧义 | dzrknn 与 dzactuator 都叫 dzactuator_node | dzrknn CMakeLists.txt:91 vs dzactuator:87 | 未修（P1-2） |

### 2.9 调参与生效路径（现场操作速查）

本节把「改哪个文件 → 重启哪个节点 → 怎么验证」串起来，避免重复踩「改了没生效」。

| 想改什么 | 改哪里 | 重启方式 | 验证 |
|---|---|---|---|
| 云台扫描角度/速度/俯仰/开火框/只打蓝靶 | config/robot.yaml 的 /dzrknn_dataprocess 段 | tools/restart_node.sh gimbal | 日志横幅「瞄准:」「射击:」；healthcheck.sh |
| 巡航速度/路线/急停余量（旧核） | config/robot.yaml 的 path_executor 段 | tools/restart_node.sh path | 7_path.log；healthcheck.sh |
| 相机曝光/分辨率/帧率 | config/robot.yaml 的 /usb_cam_node 段 | tools/restart_node.sh camera | 页面画面 |
| 识别模型 | config/robot.yaml 的 /det_node.model_path 或覆盖 share/det_node/model/light_det.rknn | 不需重启（hot_reload=true，1 s 内生效） | 4_det.log「模型热加载完成」 |
| 识别线程数/帧率 | /det_node.thread_num / max_fps | tools/restart_node.sh vision | 4_det.log 横幅；ros2 topic hz /offset_center |
| 开火链路（裁判串口） | /dzjudgment.mcuserialport / mcubaudrate | tools/restart_node.sh judgment | 6_judgment.log；ros2 topic info /LaserShot_Command（Subscription >= 1） |
| 底盘里程计标定 | /dzactuator.ticksPerMeter / ticksPer2PI | tools/restart_node.sh chassis | 直线跑 2.10 m 后 /odom/localization 位移 |
| ROS 域号 | dzros.env（DZ_ROS_DOMAIN） | 重启起栈脚本 | start_all.sh 横幅「ROS 链路域号」 |
| 地图要素 | config/map.yaml（不放 robot.yaml！） | 重新部署 + 重启 dzbrain | route_check.py / config_selfcheck.py |

生效规则：

1. **C++ 参数**（dzrknn_dataprocess / dzactuator / dzjudgment / det_node）：改 robot.yaml 后必须重启对应节点；只有重启才从参数服务器重新读取。
2. **Python 参数**（path_executor）：改 robot.yaml 后重启 path_executor_node；它启动时一次性 _load_config_center()（path_executor.py:292-310），运行中不重读。
3. **dzbrain**：改 robot.yaml 后重启 dzbrain.main；它启动时读 --config。
4. **类型必须匹配**：declare 的 double 收 double、int 收 int、bool 收 bool。用 LaunchConfiguration 传字符串会静默失败（各 launch 头注释）。
5. **declare/get 必须配对**：只 declare 不 get 的参数改了不生效（healthcheck.sh:139-179 会报）。
6. **顶层结构**：robot.yaml 顶层只允许 <节点名>: ros__parameters: 结构；地图等数据放 config/map.yaml（healthcheck.sh:115-137）。
7. **A/B 对比**：ROBOT_CONFIG=/path/to/other.yaml ros2 launch ...（各 launch 文件均支持）。
8. **冻结区**：config/**、src/dzrknn_dataprocess/**、src/dzbrain/**、tools/deploy_to_car.py 属冻结区（tools/wf/frozen.json:3-8）；实验一律进 _tmp_session/ 或 feature flag。

常见「改了没生效」的四种原因（按概率）：

- 参数键名拼错（YAML 不报错，静默忽略）→ 用 healthcheck.sh 的运行时比对或 ros2 param get 核对。
- 节点没重启（还在跑旧 ELF）→ 用 restart_node.sh 对应槽位。
- declare 了没 get → healthcheck.sh 的 declare/get 配对检查。
- 该参数属于另一套口径（如 pid 模式下调 track_kp 无效）→ 先确认 track_mode / pid_enable。

---

## §3 函数级索引大表

> 「文件:行」为本次审计快照中的行号。C++ 函数名为类成员时标注类名；Python 标注类/模块。

### 3.1 car_localization

| 函数 / 符号 | 文件:行 | 签名 | 职责 |
|---|---|---|---|
| block_to_meters | generate_path.py:20 | (str) -> (float,float) | 格名 → 米制坐标 |
| yaw_to_quaternion | generate_path.py:35 | (float) -> (qx,qy,qz,qw) | yaw → 四元数 |
| generate_straight_path | generate_path.py:42 | (x,y,len,yaw,step) -> [str] | 生成直线路径点 |
| generate_waypoint_path | generate_path.py:67 | (block,wps,step) -> [str] | 生成折线路径点 |
| main | generate_path.py:109 | () -> None | 入口，写 test_path.txt |
| WaypointSender.__init__ | send_waypoint.py:18 | (cmd) | 创建发布器与 0.1 s 定时器 |
| WaypointSender.send_once | send_waypoint.py:25 | () | 发一次 /nav/waypoint 后 shutdown |
| main | send_waypoint.py:38 | () -> None | 入口 |
| euler_from_quaternion | path_executor.py:479 | (x,y,z,w) -> float | 四元数 → yaw(rad) |
| normalize_angle | path_executor.py:490 | (float) -> float | 归一到 (-π,π] |
| block_to_meters | path_executor.py:501 | (str) -> (float,float) | 格名 → 米 |
| heading_to_quaternion | path_executor.py:529 | (str) -> (qx,qy,qz,qw) | x/y → 四元数 |
| _GatedCmdPub.__init__ | path_executor.py:570 | (pub,node) | 包装发布器 |
| _GatedCmdPub.publish | path_executor.py:574 | (msg) | manual_override 时吞掉 |
| PathExecutorNode.__init__ | path_executor.py:581 | (path_file=None) | 初始化全部状态与话题 |
| check_odom_publishers | path_executor.py:880 | () | /odom/localization 发布者数量检查 |
| load_path | path_executor.py:908 | (file_path) | 解析路径文件 |
| waypoint_callback | path_executor.py:954 | (msg) | 命令入口分发 |
| follow_route_command | path_executor.py:1098 | (data) | 路线解析与提交 |
| shoot_count_cb | path_executor.py:1302 | (msg) | 缓存累计发数 |
| _update_chassis_speed | path_executor.py:1312 | () | 位姿差分 + 低通估速 |
| offset_center_cb | path_executor.py:1325 | (msg) | 缓存视觉检测 |
| aim_status_cb | path_executor.py:1339 | (msg) | 缓存 aim 状态 |
| manual_override_cb | path_executor.py:1343 | (msg) | 手动接管开关 |
| enter_scan_stop | path_executor.py:1356 | (wp_idx) | 停车并请求扫描 |
| _banner | path_executor.py:1372 | (lines) | 打印边框横幅 |
| _print_start_banner | path_executor.py:1377 | () | 任务开始横幅 |
| _print_finish_banner | path_executor.py:1387 | () | 完成横幅 + 漂移汇总 |
| _publish_motion | path_executor.py:1427 | (reversing) | 发布运动状态（变化才发） |
| _start_mission_scan | path_executor.py:1437 | () | 按 mission_scan_mode 启动扫描 |
| _auto_start_mission | path_executor.py:1460 | () | 延时自动下发内置路线 |
| enter_shoot_stop | path_executor.py:1516 | (wp_idx) | 射击停站：吸附 + 状态机复位 |
| odom_callback | path_executor.py:1591 | (msg) | block→米、yaw 换算、叠加 snap |
| find_nearest_segment | path_executor.py:1617 | () -> dict|None | 最近航段 + 投影 |
| find_lookahead_point | path_executor.py:1657 | (seg) -> dict | 前视点 + 转角钳位 |
| pure_pursuit_control | path_executor.py:1736 | (tp,reverse) -> (v,steer) | 纯追踪核心输出 |
| _load_shoot_planner | path_executor.py:1812 | () -> module|False | 懒加载 shoot_planner |
| compute_shoot_chain | path_executor.py:1841 | (target_name) -> [str] | 候选射击点链 |
| _passed_waypoint | path_executor.py:1882 | (wp_idx,wp) -> bool | 是否冲过航点 |
| _is_reverse_segment | path_executor.py:1907 | (seg) -> bool | 本段是否倒车 |
| _rebuild_align_segments | path_executor.py:1926 | () | 预计算桥洞保护航段 |
| _align_straight_update | path_executor.py:1956 | () -> (bool,yaw,name) | 维护强制直行状态 |
| control_loop | path_executor.py:2005 | () | 50 Hz 优先级状态机 |
| main | path_executor.py:2599 | (args=None) | 入口，Ctrl-C 发零 Twist |
| get_block_name | serial_reader.py:68 | (x,y) -> str | 米 → 格名 |
| euler_yaw | serial_reader.py:78 | (x,y,z,w) -> float | 四元数 → yaw |
| normalize_angle | serial_reader.py:84 | (float) -> float | 归一 |
| SerialReaderNode.__init__ | serial_reader.py:93 | () | 订阅 /odom /imu_data /nav/waypoint |
| imu_callback | serial_reader.py:146 | (msg) | 记录 IMU yaw_rel |
| waypoint_callback | serial_reader.py:156 | (msg) | reset 重新基准 DR 方向 |
| odom_callback | serial_reader.py:189 | (msg) | 死推算 + 发布 /odom/localization |
| main | serial_reader.py:258 | (args=None) | 入口 |

### 3.2 dzactuator

| 函数 / 符号 | 文件:行 | 签名 | 职责 |
|---|---|---|---|
| main | dzactuator.cpp:20 | (argc,argv) | 入口，main→Control() |
| IMU_Trans | dzactuator.cpp:35 | (hi,lo) -> short | 大端 → short |
| Odom_Trans | dzactuator.cpp:43 | (hi,lo) -> float | 大端 → float |
| callback_cmd_vel_angle | dzactuator.cpp:59 | (Twist) | /cmd_vel → TargetSpeed/Angle（DWA） |
| callback_movebase_angle | dzactuator.cpp:112 | (Twist) | /pursuitAngle → 控制帧（Pursuit） |
| callback_logi_control | dzactuator.cpp:188 | (Twist) | /logicontrol → 平行/非平行切换 |
| callback_voice_switch | dzactuator.cpp:241 | (UInt8) | 语音 ID |
| callback_laser_switch | dzactuator.cpp:248 | (UInt8) | 激光常亮开关 |
| callback_vehicle_state | dzactuator.cpp:258 | (UInt8) | voice_nums[0] |
| callback_vehicle_Drivestate | dzactuator.cpp:264 | (UInt8) | voice_nums[1] |
| callback_CameMo_Conl | dzactuator.cpp:270 | (Int32MultiArray) | 云台位置/速度 |
| callback_stopCMD | dzactuator.cpp:285 | (UInt8) | StopCmd |
| switchvoice | dzactuator.cpp:293 | () | 语音播报节流 |
| smoothValue | dzactuator.cpp:321 | (deque,new,win) -> float | 滑动平均 |
| smoothValueAdaptive | dzactuator.cpp:335 | (deque,new,win) -> float | 自适应窗滑动平均 |
| sendCarInfoKernel | dzactuator.cpp:376 | () | 打包 23B 下行帧并写串口 |
| imu_bias_correction | dzactuator.cpp:451 | (Imu,&g,&a) | 零偏校正（200 帧） |
| Publish_ImuSensor_filter | dzactuator.cpp:501 | () | 发 /imu_data（滤波+零偏） |
| Publish_ImuSensor | dzactuator.cpp:576 | () | 发 /raw + /imu_msg_valid |
| getYaw | dzactuator.cpp:636 | (PoseStamped) -> double | tf2::getYaw |
| Publish_Odom | dzactuator.cpp:646 | () | 发 /odom + /odom_msg_valid |
| Publish_Voltage | dzactuator.cpp:710 | () | 发 /PowerVoltage |
| Publish_Battery_Percentage | dzactuator.cpp:728 | () | 发 /Battery_Percentage |
| Publish_CameMo | dzactuator.cpp:823 | () | 发 /camemo_feedback |
| Check_Sum | dzactuator.cpp:857 | (n,mode) -> unsigned char | 累加和（注释误称 XOR） |
| Get_Sensor_Data_New | dzactuator.cpp:883 | () -> bool | 读 34B 上行帧并解析 |
| detect_toggle | dzactuator.cpp:1041 | (uint8) -> int | rx[32] 跳变检测 |
| normalizeAngle | dzactuator.cpp:1065 | (double) -> double | 归一到 (-π,π] |
| turn_on_robot::turn_on_robot | dzactuator.cpp:1078 | (name) | 构造：参数、话题、串口、定时器 |
| ~turn_on_robot | dzactuator.cpp:1223 | () | 析构：发零速帧、关串口 |
| Control | dzactuator.cpp:1256 | () | 100 Hz 主循环 |
| checkCommandWatchdog | dzactuator.cpp:1346 | () | 1 s 无速度指令强制停车 |
| modeTimerCallback | dzactuator.cpp:1376 | () | DWA/Pursuit 模式仲裁（100 ms） |
| Quaternion_Solution | Quaternion_Solution.cpp:214 | (g,a) -> Quaternion | Madgwick 风格互补滤波 |
| InvSqrt | Quaternion_Solution.cpp:192 | (float) -> float | 快逆平方根 |
| kalman_filter_optimized | Quaternion_Solution.cpp:46 | (z,&x,&P,motion) -> double | 加速度卡尔曼 |
| calculate_butterworth_coefficients | Quaternion_Solution.cpp:84 | () | 二阶 Butterworth 系数 |
| butterworth_filter_optimized | Quaternion_Solution.cpp:103 | (x,buf,ybuf) -> double | Butterworth 低通 |
| smooth_prefilter | Quaternion_Solution.cpp:129 | (input,buf) -> float | 滑动平均预滤波 |
| detect_motion | Quaternion_Solution.cpp:148 | (g,a) -> bool | 运动状态检测 |
| filter_imu_data | Quaternion_Solution.cpp:162 | (g&,a&) | 完整 IMU 滤波链 |

### 3.3 dzrknn_dataprocess

| 函数 / 符号 | 文件:行 | 签名 | 职责 |
|---|---|---|---|
| main | dzrknn_dataprocess.cpp:14 | (argc,argv) | 入口 |
| callback_offset_center | dzrknn_dataprocess.cpp:24 | (Int32MultiArray) | 视觉伺服 + 开火判定（核心） |
| clampYawTick | 477 | (int) -> int | 水平刻度钳位 [100,3995] |
| clampPitchTick | 491 | (int) -> int | 竖直刻度钳位 [950,2400] |
| clampYawToScan | 507 | (int) -> int | 钳到扫描水平区间 |
| clampPitchToScan | 519 | (int) -> int | 钳到扫描俯仰区间 ± margin |
| scanYawDeg | 549 | () -> double | 当前扫描水平角 |
| pitchLevelCount | 555 | () -> int | 俯仰层数 |
| scanPitchDeg | 563 | () -> double | 当前扫描俯仰角 |
| patrolSpeed | 575 | () -> int | 巡逻速度 = max×ratio |
| aspectRatioOk | 582 | (msg) -> bool | 长宽比过滤 |
| trackSpeedFor | 604 | (abs_err,d_abs_err,eff) -> int | tuned 速度律 |
| trackSpeedCap | 637 | () -> int | 位置/速度共用限速源 |
| resetPid | 645 | () | 进入 TRACK 清 PID 状态 |
| pidStep | 666 | (e,dt,yaw_axis) -> double | 增量式 PID 单步（速率输出） |
| pidSpeedFor | 714 | (rate_abs,eff) -> int | PID 速率 → 下发速度 |
| v1SpeedCap | 735 | () -> int | v1 速度上限 |
| v1TrackSpeedFor | 748 | (abs_err,d_abs_err,eff) -> int | v1 速度律 |
| fireDeadzonePx | 776 | (box_w) -> int | 动态开火死区 |
| applyScanTarget | 802 | () | 下发当前扫描目标 |
| advanceScanIndex | 819 | () | 扫描推进/反向/换层 |
| setAimState | 843 | (int) | 切状态并上报 |
| publishAimStatus | 857 | () | 发 /aim/status |
| startScan | 877 | (continuous,home) | 启动扫描（含归位段） |
| scanHomingStep | 910 | () -> bool | 归位段一拍 |
| enterScanPhase | 936 | () | 真正进入扫描阶段 |
| callback_scan_request | 951 | (UInt8) | /nav/scan_request 解析 |
| CaremaMontorControl | 977 | () | IDLE/SCAN/TRACK 状态机 |
| callback_camemo_feedback | 1105 | (Int32MultiArray) | 云台反馈缓存 |
| callback_shoot_reset | 1120 | (UInt8) | /shoot_reset 重置弹药 |
| callback_imu | 1136 | (Imu) | 车体角速度 |
| callback_fire_gate | 1145 | (UInt8) | /aim/fire_gate |
| callback_fire_ready | 1157 | (UInt8) | /aim/fire_ready |
| callback_lead_x | 1169 | (Float32) | /aim/lead_x |
| compute_comp_pixel_0 | 1180 | (tar_dis) -> int | 视差查表（预留） |
| loadLookupTable | 1194 | (filePath) -> bool | 解析 offcenter_table.txt |
| turn_on_robot::turn_on_robot | 1247 | (name) | 构造：参数/话题/查表 |
| ~turn_on_robot | 1676 | () | 析构 |
| Control | 1680 | () | 主循环（自适应频率） |

### 3.4 dzjudgment

| 函数 / 符号 | 文件:行 | 签名 | 职责 |
|---|---|---|---|
| main | dzjudgment.cpp:15 | (argc,argv) | 入口 → run() |
| dzjudgment::dzjudgment | judgment.cpp:3 | () | 构造：串口、话题 |
| ~dzjudgment | judgment.cpp:68 | () | 关串口 |
| callback_LaserShot_Command | judgment.cpp:77 | (UInt8) | data==1 → sendLaserShot |
| callback_referee_cmd | judgment.cpp:85 | (UInt8MultiArray) | 原样写串口 |
| run | judgment.cpp:119 | () | 100 Hz 主循环 |
| reconnectSerial | judgment.cpp:162 | () -> bool | 掉线重连（3 次） |
| publish_referee_state | judgment.cpp:193 | () | 1 s 上报 /HpAndHitmsg |
| recvESP32Info | judgment.cpp:205 | () | 有界非阻塞读裁判帧 |
| processData | judgment.cpp:285 | (arr,len) | 按 0x02/0x03/0x04/0x07 分流 |
| sendLaserShot | judgment.cpp:358 | () | 写 6B 开火帧 |

### 3.5 rknn_pt / det_node

| 函数 / 符号 | 文件:行 | 签名 | 职责 |
|---|---|---|---|
| DetNode::DetNode | det_node.cc:27 | () | 构造：参数、订阅、推理池、定时器 |
| resolveModelPath | det_node.cc:91 | (cfg) -> string | 模型路径解析（ament→源码回退） |
| loadPool | det_node.cc:118 | () -> bool | 建新池并替换 |
| checkHotReload | det_node.cc:147 | () | mtime 热加载 |
| imageCallback | det_node.cc:184 | (Image) | 帧拷贝（加锁） |
| processTimer | det_node.cc:197 | () | 定时推理 + 后处理 + 发布 |
| main | det_node.cc:300 | (argc,argv) | 入口 |
| RkPt::RkPt | rkpt.cc:89 | (path) | 保存路径与阈值 |
| RkPt::init | rkpt.cc:96 | (ctx_in,share_weight) -> int | 加载模型、绑核、查输入输出属性 |
| RkPt::get_pctx | rkpt.cc:200 | () -> rknn_context* | 父 context |
| RkPt::infer | rkpt.cc:205 | (img,frame_id) -> DetectResultsGroup | 预处理→推理→后处理 |
| ~RkPt | rkpt.cc:300 | () | 释放 context/内存 |
| load_model | rkpt.cc:66 | (filename,&size) -> uchar* | 读模型文件 |
| load_data | rkpt.cc:37 | (fp,ofst,sz) -> uchar* | 分段读 |
| dump_tensor_attr | rkpt.cc:20 | (attr) | 调试打印（已注释） |
| post_process | postprocess.cc:73 | (out,np,nc,...) -> int | YOLOv8-pose 后处理 |
| nms | postprocess.cc:39 | (boxes,threshold) | 贪心 NMS（同类） |
| CalculateOverlap | postprocess.cc:19 | (8 floats) -> float | IoU |
| clamp | postprocess.cc:14 | (val,min,max) -> int | 夹取 |
| initializeRandColors | postprocess.cc:172 | () | 随机颜色初始化 |
| show_draw_results | postprocess.cc:183 | (group) | 画框/点 |
| letterbox | preprocess.cc:9 | (img,padded,pads,scale,size,color) | letterbox 缩放填充 |
| resize_rga | preprocess.cc:28 | (src,dst,img,resized,size) -> int | RGA 硬件 resize |
| rknnPool::init | rknnPool.hpp:49 | () -> int | 建线程池 + threadNum 个模型 |
| rknnPool::put | rknnPool.hpp:91 | (input,frame_id) -> int | 提交推理 |
| rknnPool::get | rknnPool.hpp:102 | (output&) -> int | 取结果 |
| rknnPool::getModelId | rknnPool.hpp:74 | () -> int | 轮转模型实例 |
| ~rknnPool | rknnPool.hpp:114 | () | drain 在途任务 |
| ThreadPool::submit | ThreadPool.hpp:59 | (func,...) -> future | 提交任务 |
| ThreadPool::worker | ThreadPool.hpp:97 | () | 工作线程 |
| get_core_num | coreNum.hpp:12 | () -> int | 轮转 NPU 核 0/1/2 |

### 3.6 配置与工具

| 函数 / 符号 | 文件:行 | 签名 | 职责 |
|---|---|---|---|
| deploy_to_car.py: sh | 96 | (argv,dry,input) -> int | 执行子进程 |
| resolve_ip | 109 | (dry) -> str | mDNS 解析车端 IP（最多 5 次） |
| ssh_run | 126 | (ip,cmd,dry,label) -> int | ssh 执行 |
| sh_bytes | 132 | (argv,data,dry,label) -> int | 二进制 stdin 执行 |
| iter_payload | 153 | (root) | 列推送文件（跳过 __pycache__） |
| sha256_file / sha256_bytes | 165/173 | (path/data) -> str | 哈希 |
| normalize_auto_start | 177 | (text) -> str | auto_start 归一化 |
| expected_remote_bytes | 191 | (local,raw,apply_transform) -> bytes | 对账基准 |
| local_manifest | 211 | (local,apply_transform) -> dict | 本地 sha256 清单 |
| make_tarball | 224 | (local) -> bytes | 打 tar.gz（相对路径） |
| dir_swap_cmd | 234 | (remote) -> str | 车端原子替换脚本 |
| push_dir | 243 | (ip,local,remote,dry) -> int | tar 流推送目录 |
| push_file | 250 | (ip,local,remote,dry) -> int | scp 推单文件 |
| remote_manifest | 280 | (ip,remote,is_dir,dry) -> dict | 车端 sha256 清单 |
| diff_manifest | 298 | (lm,rm) -> (missing,extra,mismatch) | 清单差异 |
| verify_pushed | 306 | (ip,items,dry,apply_transform) -> int | 对账闸门 |
| cmd_probe | 333 | (ip,args) | probe 子命令 |
| cmd_backup | 344 | (ip,args) | backup 子命令 |
| cmd_push | 357 | (ip,args) -> int | push 子命令 |
| cmd_build | 389 | (ip,args) -> int | build 子命令 |
| cmd_verify | 405 | (ip,args) -> int | verify 子命令 |
| main | 424 | (argv) -> int | argparse 入口 |
| start_all.sh: spawn | 47 | (name,cmd...) | setsid 起进程组 + 写 PID |
| start_all.sh: kill_group | 61 | (pid) | 进程组 TERM→KILL |
| start_all.sh: cleanup_strays | 74 | () | 清工作区残留 |
| start_all.sh: stop_all | 94 | () | 停全部受管节点 |
| restart_node.sh: node_spec | 28 | (name) -> "log|cmd" | 槽位表 |
| healthcheck.sh: ok/bad/warn/sec | 30-33 | (str) | 打印辅助 |
| healthcheck.sh: 不变量校验块 | 61-107 | (CONFIG) | Python 内联校验 |
| healthcheck.sh: params 结构校验块 | 115-137 | (CONFIG) | 顶层结构校验 |
| healthcheck.sh: declare/get 配对块 | 146-168 | (WS) | 正则配对检查 |

## §4 数据流与线程模型

### 4.1 完整数据流（车端）

    STM32 下位机
      ├─ 底盘串口 /dev/dzactuator @115200
      │    下行 23B: 0xA5 0x5A | 舵角(0.1°,int16 BE) | 方向 | 速度 | 激光 | 云台Pos0/1 | 云台Spd0/1 | Time0/1 | Fun | ID | 重置 | 语音 | 累加和
      │    上行 34B: 0x7B 0x1F | 左/右轮速 | 电池mV | 加速度xyz | 角速度xyz | 云台Pos0/1 Spd0/1 Load0/1 | rx32
      │
      └─ 裁判/激光串口 /dev/dzjudgment @115200
           下行 6B: B5 5B 03 21 01 25（开火）
           上行: 0xB5 0x5B len payload（0x02/0x03/0x04/0x07）

    上位机 ROS 2 Humble
      usb_cam_node ──/image_raw──► det_node ──/offset_center[8]──► dzrknn_dataprocess
                                                                         │
      dzactuator ──/odom,/imu_data,/camemo_feedback──► serial_reader ──/odom/localization──► path_executor/dzbrain
                                                                         │
      dzrknn_dataprocess ──/CameMo_Conl[4]──► dzactuator ──23B 下行帧──► STM32（云台伺服）
      dzrknn_dataprocess ──/LaserShot_Command──► dzjudgment ──6B 开火帧──► 激光模块
      path_executor ──/pursuitAngle──► dzactuator ──23B 下行帧──► STM32（底盘）
      path_executor ──/nav/scan_request,/aim/status 握手──► dzrknn_dataprocess

### 4.2 线程与频率模型

| 进程 | 线程模型 | 主循环频率 | 关键定时器/回调频率 |
|---|---|---|---|
| dzactuator | 单线程（回调与主循环同线程） | 100 Hz（Control） | mode_timer 100 ms |
| dzrknn_dataprocess | 单线程（回调与主循环同线程） | 自适应 100/50 Hz | 视觉回调由 det_node 30 Hz 驱动 |
| dzjudgment | 单线程 | 100 Hz（run） | 每 100 拍（1 s）上报状态 |
| det_node | 主线程 + rknnPool 线程池（thread_num=2） | 定时器周期 max(33ms, 1000/30) | 30 Hz |
| serial_reader | 单线程 rclpy::spin | 由 /odom 频率驱动 | — |
| path_executor | 单线程 rclpy::spin + 50 Hz 定时器 | 50 Hz（control_loop） | 3 s 发布者健康检查 |

### 4.3 关键握手时序

1. 起栈：start_all.sh 依序起 1→7 步，每步 sleep 3 s（云台 2 s）。
2. 发车：path_executor（或 dzbrain）在 auto_start_delay 后发 reset:N10,y → 再发路线 → _start_mission_scan() 发 /nav/scan_request=2（continuous）。
3. 扫描：dzrknn_dataprocess 收到 scan_request=2 → startScan(true) → 归位段（≤1500 ms）→ enterScanPhase → SCAN 状态。
4. 锁定：视觉连续 target_confirm_frames_(2) 帧命中 → TRACK，同时类别锁需 cls_switch_confirm_frames_(5) 帧。
5. 停站：path_executor 到达射击点 → enter_shoot_stop → 四阶段（SETTLE 0.4 s → ACQUIRE 等锁定 → STABLE 5 帧 → FIRE）。
6. 开火：dzrknn_dataprocess 判 in_fire_zone && aim_ready && ammo && cooldown && target_confirmed，连续 fire_hold_frames_(5) 帧 → 发 /LaserShot_Command=1 → dzjudgment 写 6B 帧。
7. 收尾：打满 SHOOT_COUNT_TARGET(10) 或 30 s 超时 → 停站结束 → 循迹继续。

### 4.4 失效保护层

1. **指令看门狗**：dzactuator checkCommandWatchdog，1 s 无速度指令强制停车（只停速度）。
2. **手动接管闸门**：path_executor._GatedCmdPub 吞掉自动 Twist；dzrknn_dataprocess manual_override 停发 CameMo_Conl 位置。
3. **越界急停**：path_executor 信念位姿超出地图 +0.60 m 即零 Twist 刹车。
4. **漂移降速/停车**：dzbrain 的 drift_slow_after 3 拍降速 0.08 m/s、drift_stop_after 8 拍停车（robot.yaml:871-874）。
5. **边界守卫**：boundary_guard_m 0.25 限速、boundary_stop_m 0.10 请求停车。
6. **串口异常兜底**：dzactuator 三层 catch SerialException + 限次循环；dzjudgment 有界非阻塞 + 重连。

## §5 配置键汇总

### 5.1 键 → 消费方对照（重点）

| 配置键 | 段 | 行号 | 消费方 | 备注 |
|---|---|---|---|---|
| scan_yaw_min_deg / scan_yaw_max_deg | /dzrknn | 37-38 | dzrknn | 同时影响 TRACK 的 clampYawToScan |
| scan_pitch_offset_deg | /dzrknn | 50 | dzrknn | 符号约定 +向下压 |
| scan_pitch_levels | /dzrknn | 65 | dzrknn | 俯仰层 |
| scan_passes / scan_continuous_cycles / scan_home_timeout_ms | /dzrknn | 68,72,75 | dzrknn | 扫描趟数/轮数上限/归位超时 |
| patrol_speed_ratio / gimbal_max_speed | /dzrknn | 81-82 | dzrknn | 巡逻速度 |
| track_speed_max / track_speed_max_ratio | /dzrknn | 83,86 | dzrknn | 跟踪限速（同源） |
| loop_hz_track / loop_hz_idle | /dzrknn | 94-95 | dzrknn | 自适应频率 |
| gimbal_*_tick_min/max / center_tick | /dzrknn | 100-118 | dzrknn | 刻度限位 |
| yaw_deg_sign / pitch_deg_sign | /dzrknn | 103-104 | dzrknn | 方向符号 |
| tracking_pitch_margin_deg | /dzrknn | 111 | dzrknn | 扫描俯仰余量 |
| ema_alpha | /dzrknn | 123 | dzrknn | 视觉滤波 |
| track_deadzone_px | /dzrknn | 124 | dzrknn | 须 < fire_dz_far_px |
| track_kp/kd/inc_gain/speed_floor | /dzrknn | 125-128 | dzrknn（tuned 模式） | pid 模式下不生效 |
| fine_* | /dzrknn | 129-131 | dzrknn | 精调档 |
| predict_gain | /dzrknn | 132 | dzrknn | 移动预判 |
| track_mode | /dzrknn | 141 | dzrknn | v1/tuned/pid 选择 |
| v1_* | /dzrknn | 144-157 | dzrknn（v1 模式） | 上游复刻 |
| pid_enable/kp/ki/kd/i_clamp/d_alpha/min_step | /dzrknn | 166-172 | dzrknn（pid 模式） | 当前生效 |
| fire_dz_dynamic / fire_deadzone_px / fire_dz_far_px / fire_dz_near_px | /dzrknn | 177-184 | dzrknn + Python aim.py | 开火框 |
| box_w_far / box_w_near | /dzrknn | 185-186 | dzrknn | 框宽标定 |
| fire_hold_frames | /dzrknn | 187 | dzrknn | 连续帧 |
| fire_use_image_stable / fire_stable_px_per_frame | /dzrknn | 188-189 | dzrknn | 画面稳定判据 |
| fire_real_speed_max / fire_speed_max | /dzrknn | 190-191 | dzrknn（旧判据） | 电机静止阈值 |
| target_confirm_frames | /dzrknn | 192 | dzrknn | 扫描确认 |
| cls_switch_confirm_frames | /dzrknn | 193 | dzrknn | 类别迟滞 |
| shot_cooldown_ms | /dzrknn | 197 | dzrknn | 两发间隔 |
| fire_only_blue | /dzrknn | 202 | dzrknn | 只打蓝靶 |
| max_shots / max_shots_per_target | /dzrknn | 203,207 | dzrknn | 弹药 |
| laser_offset_x / laser_offset_y | /dzrknn | 212-213 | dzrknn | 激光同轴 |
| use_external_fire_gate / lead_apply_gain | /dzrknn | 227-228 | dzrknn | 两键开火 + 提前量 |
| aspect_min / aspect_max | /dzrknn | 234-235 | dzrknn | 长宽比过滤 |
| lost_frames_to_rescan | /dzrknn | 236 | dzrknn | 丢帧阈值（须 ≥60） |
| vision_timeout_ms | /dzrknn | 237 | dzrknn | 视觉静默 |
| gyro_ff_gain / gyro_ff_sign / imu_ff_timeout_ms | /dzrknn | 242-244 | dzrknn | 车体前馈 |
| video_device / image_width / image_height / framerate / pixel_format / io_method / brightness | /usb_cam_node | 252-258 | usb_cam_node | 相机 |
| mcuserialport / mcubaudrate | /dzjudgment | 290-291 | dzjudgment | 裁判串口 |
| model_path / draw / thread_num / max_fps / hot_reload | /det_node | 299-311 | det_node | 视觉检测 |
| linear_speed | path_executor | 333 | path_executor + dzbrain | 巡航速度 |
| auto_start / auto_start_delay | path_executor | 336-337 | path_executor | 自动出发 |
| mission_scan_mode / mission_reset | path_executor | 338-339 | path_executor | 扫描模式/重锚 |
| reverse_speed_ratio / reverse_steer_sign | path_executor | 343-344 | path_executor | 倒车 |
| bridge_blocks / bridge_order / align_straight_* | path_executor | 355-368 | path_executor | 桥洞保护 |
| shoot_distance_target/min/max | path_executor | 373,399-400 | path_executor + shoot_planner | 射击距离窗口 |
| shoot_max_retry | path_executor | 408 | path_executor | 候选点上限 |
| shoot_score_w_gimbal / w_detour | path_executor | 409-410 | shoot_planner | 评分权重 |
| shoot_count_target / shoot_timeout | path_executor | 423-424 | path_executor | 每点发数/超时 |
| shoot_settle_speed / shoot_stable_frames / shoot_stable_offset_px / shoot_stable_timeout | path_executor | 426-429 | path_executor | 稳定判据 |
| obstacle_blocks | path_executor | 432 | shoot_planner | 障碍表 |
| shoot_settle_time / shoot_lock_timeout / shoot_overshoot_tol / shoot_slowdown_m | path_executor | 439-443 | path_executor | 停站参数 |
| map_safety_margin / drift_warn_m | path_executor | 450,456 | path_executor | 越界/漂移 |
| finish_voice_id | path_executor | 459 | path_executor | 完成语音 |
| mission_blocks | path_executor | 532-600 | path_executor | 路线真源（68 token） |
| pursuit.* | /dzbrain | 616-623 | dzbrain | 纯追踪 |
| cruise_speed_mps | /dzbrain | 625 | dzbrain | 巡航速度 |
| auto_start / auto_start_delay_s / reset | /dzbrain | 703-705 | dzbrain | 出发控制 |
| scan_mode | /dzbrain | 706 | dzbrain | 扫描模式 |
| shots_per_stop / shoot_timeout_s / lock_timeout_s / settle_time_s | /dzbrain | 713-725 | dzbrain | 停站 |
| engine / fallback_to_tokens / allow_reverse / max_seconds / pivot_penalty / mine_turn_penalty | /dzbrain | 749,786-792 | dzbrain | 规划器 |
| lead_time_s / fire_policy / rail_span_m / target_speed_mps / px_per_m_rail | /dzbrain | 833-843 | dzbrain | 提前量/轨道靶 |
| max_shots_total / max_shots_per_target / shot_cooldown_ms | /dzbrain | 846-847 | dzbrain | 弹药 |
| lost_frames_to_rescan | /dzbrain | 848 | dzbrain | 丢帧 |
| scan_pitch_offset_deg | /dzbrain | 849 | dzbrain | 与 /dzrknn 对齐 |
| anchor_* / scale_est_* | /dzbrain | 854-870 | dzbrain | 吸附/尺度估计 |
| drift_slow_after / drift_slow_speed_mps / drift_stop_after / drift_clear_ticks | /dzbrain | 871-874 | dzbrain | 漂移降级 |
| boundary_guard_m / boundary_guard_speed_mps / boundary_stop_m | /dzbrain | 875-877 | dzbrain | 边界守卫 |
| yaw_trim_deg / yaw_rate_floor_dps | /dzbrain | 882,886 | dzbrain | 航向修正 |
| heartbeat_timeout_s / info_stale_s / pose_timeout_s / task_stall_s / startup_grace_s / brake_on_fault | /dzbrain | 890-897 | dzbrain | 看门狗 |
| serial_port_name / serial_baud_rate | /dzactuator | 910-911 | dzactuator | 底盘串口 |
| odom/robot/gyro_frame_id | /dzactuator | 914-916 | dzactuator | TF |
| calibrate_lineSpeed / ticksPerMeter / ticksPer2PI | /dzactuator | 920-941 | dzactuator | 里程计标定 |
| vehicle.* / field.* | /dzactuator | 944-964 | dzbrain route/physics | 物理几何 |
| smooth_window_speed / smooth_window_angle | /dzactuator | 967-968 | dzactuator | 平滑窗口 |

### 5.2 双真源 / 冗余键

| 键 | 出现位置 | 说明 |
|---|---|---|
| scan_pitch_offset_deg | robot.yaml:50（/dzrknn）与 robot.yaml:849（/dzbrain） | 两处必须一致（healthcheck 不查，靠人工/卡对齐） |
| lost_frames_to_rescan | robot.yaml:236（/dzrknn=180）与 robot.yaml:848（/dzbrain=60） | ⚠ **数值不一致**：180 vs 60。dzrknn 注释说 180 是旧值 60 收紧……实际两段不同值，需确认哪个是真源 |
| max_shots_per_target | robot.yaml:207（/dzrknn=10）与 robot.yaml:846（/dzbrain=10） | 一致 |
| shot_cooldown_ms | 207 附近与 847 | 一致（500） |
| shoot_distance_* | robot.yaml:399-400（path_executor，靶格心）与 :687-688（/dzbrain，板心） | 语义不同但本次同值 [0.78,1.02] |
| cruise_speed_mps | robot.yaml:625（/dzbrain=0.35）与 linear_speed :333（path_executor=0.35） | 一致 |
| auto_start | :336（path_executor=true）与 :703（/dzbrain=false） | ⚠ **不一致**：true vs false。path_executor 段注释说「本轮现场发车翻 true 跑完回 false」，dzbrain 段是 false。默认走 dzbrain，故实际 auto_start=false |
| auto_start_delay | :337（path_executor=45.0）与 :704（/dzbrain=10.0） | 不一致（45 vs 10） |
| bridge_blocks | :355（path_executor=[]）与 :636（/dzbrain=[]） | 一致（都关闭） |

⚠ 以上「不一致」项需在收口时由队长裁决；本报告只记录事实。

## §6 测试与证据

### 6.1 本次审计的取证方式

- 全量 glob 清点 dzrobot_v2/src 下 5 个目标包与 config/、tools/ 的 11946 行代码（wc -l 汇总）。
- 逐文件精读：dzactuator.cpp(1447)、dzactuator.h(341)、dzrknn_dataprocess.cpp(1725)、dzrknn_dataprocess.h(372)、judgment.cpp(401)、det_node.cc(307)、postprocess.cc(209)、rkpt.cc(311)、path_executor.py(2622)、serial_reader.py(271)、robot.yaml(968)、deploy_to_car.py(470)、start_all.sh(248)、restart_node.sh(116)、healthcheck.sh(220+)。
- 未执行任何车端命令，未 SSH 连车，未修改任何文件。

### 6.2 静态检查工具（仓库自带）

| 工具 | 命令 | 覆盖 |
|---|---|---|
| 文档守卫 | python docs/_工具/check_doc_policy.py | 库外 .md / 新库 / 未登记 |
| 索引生成 | python docs/_工具/gen_index.py --ns <ns> | 目录索引增量更新 |
| 收口门禁 | cd dzrobot_v2 && python tools/wf/check.py | 冻结区脏文件（frozen.json in_flight 必须登记且未过期）、REQUIRED_DOCS |
| 复用检查 | tools/wf/check_reuse.py | 复用清单 / 能力索引 |
| 断链检查 | tools/wf/check_links.py | docs/** 与 history/INDEX 链接 |
| 目录索引 | tools/wf/repo_index.py --write | 索引标注.yaml → 目录索引 |

### 6.3 配置中心自检

- config_selfcheck.py：车上自检脚本，但 deploy_to_car.py verify 已不再调用它（deploy_to_car.py:11,405-420 注释）。
- healthcheck.sh 是当前唯一的出车体检入口（start_all.sh:13,244）。

### 6.4 关键证据锚点（可复跑）

| 结论 | 证据位置 |
|---|---|
| 下行帧 23B 布局 | dzactuator.cpp:391-443 |
| 上行帧 34B 布局 | dzactuator.cpp:883-1039 |
| 俯仰符号修复 | dzrknn_dataprocess.cpp:172-182 |
| 五套 PID 口径分支 | dzrknn_dataprocess.cpp:157-171, 208-224, 243-257 |
| 开火判定 | dzrknn_dataprocess.cpp:261-392 |
| 扫描状态机 | dzrknn_dataprocess.cpp:877-1101 |
| 打靶四阶段 | path_executor.py:2034-2193 |
| 裁判帧协议 | judgment.cpp:358-401 |
| /offset_center 格式 | det_node.cc:215-259 |
| 部署哈希对账 | deploy_to_car.py:306-328 |
| 冻结区定义 | tools/wf/frozen.json:3-8 |

⚠ 未核实：以上「可复跑」指本地源码阅读可复现；本报告未在车上执行任何命令。

## §7 未核实清单

| # | 未核实项 | 原因 | 建议 |
|---|---|---|---|
| U1 | 车端实际运行的 robot.yaml 与本地 config/robot.yaml 是否一致 | 需 SSH 上车比对 | deploy_to_car.py verify 或手动 sha256 比对 |
| U2 | 车端当前是否已部署 HEAD e40b9a1（v3.8.1）还是 2523845（v3.2） | 历史记忆称落后 9 个提交，本次未连车 | 上车核对 git log / 版本文件 |
| U3 | 各 robot.yaml.*bak 的时间点与差异 | 未逐一 diff | 需要时 diff 对照 |
| U4 | generate_path.py 行尾逗号是否被下游严格依赖 | 未找到精确 7 字段断言 | grep 下游解析 |
| U5 | get_block_name 与 block_to_meters 在边界格是否严格互逆 | 未逐格穷举 | 跑 test_coords.py |
| U6 | /dzbrain 段键与 dzbrain SCHEMA 的完整对应 | 超出车端硬件范围 | 由 dzbrain 负责人核对 |
| U7 | lost_frames_to_rescan 180 vs 60 谁是真源 | 两段不一致 | 队长裁决 |
| U8 | auto_start true vs false、delay 45 vs 10 的最终生效值 | 两段不一致 | 队长裁决 |
| U9 | REVERSE_STEER_SIGN 倒车方向是否正确 | 无实车依据（代码注释自述） | 实车验证 |
| U10 | ticksPer2PI=2680 的 ~3% 航向偏差是否可接受 | 代码注释自述待仲裁 | 实车到位吸附验证 |
| U11 | 平行驾驶（control_mode==2）下 pursuitAngle 是否真的无效 | 代码逻辑推断（123-124 return），未实车 | 实车验证 |
| U12 | 上行帧校验被注释掉后，帧错误率是否可接受 | 未实测 | 日志统计 |
| U13 | /aim/status 移除 shim 后，是否还有 UInt8 订阅者 | 网页已下线，但未全仓 grep 确认 | grep 全仓 |
| U14 | PUSH_FILES 未含 start_all.sh，车端是否已用最新版 | 部署清单不含 | 确认上车脚本版本 |
| U15 | dzactuator/params/*.yaml 是否有活引用 | 未找到 launch 引用 | 全仓 grep |
| U16 | auto_sl_s3e.launch.py 引用的两个 launch 是否已删除 | 本次审计 launch/ 下确实缺失 | 确认是否应清理 |
| U17 | det_node 只发布最高分目标是否影响多靶场景 | 代码逻辑推断 | 实车/仿真验证 |
| U18 | 轨迹靶 rail 参数（rail_span_m 0.70、target_speed_mps 0.04、px_per_m_rail 140）与实际靶是否匹配 | 配置值，未实测 | 现场标定 |

---

## 附录 A：关键常量速查

| 常量 | 值 | 位置 |
|---|---|---|
| FRAME_HEADER（上行） | 0x7B | dzactuator.h:62 |
| FRAME_TAIL（上行） | 0x7D | dzactuator.h:63 |
| 下行帧头 | 0xA5 0x5A | dzactuator.cpp:392-393 |
| RECEIVE_DATA_SIZE | 34 | dzactuator.h:64 |
| SEND_DATA_SIZE | 11 | dzactuator.h:65 |
| CARL（轴距） | 0.15 | dzactuator.h:68 |
| CARW（轮距） | 0.157 | dzactuator.h:69 |
| ACTIVITY_TIMEOUT | 0.5 s | dzactuator.h:320 |
| PURSUIT_LOW_TIMEOUT | 3.0 s | dzactuator.h:321 |
| CMD_WATCHDOG_S | 1.0 s | dzactuator.h:322 |
| BIAS_CALIBRATION_SAMPLES | 200 | dzactuator.h:294 |
| AIM_IDLE/SCAN/TRACK | 0/1/2 | dzrknn_dataprocess.h:134 |
| AIM_MAX_SHOTS | 10 | dzrknn_dataprocess.h:294 |
| OBJ_CLASS_NUM | 2 | postprocess.h:20 |
| KPT_NUM / KPT_DIM | 1 / 3 | postprocess.h:22-23 |
| NMS_THRESH | 0.45 | postprocess.h:25 |
| BOX_THRESH | 0.4 | postprocess.h:26 |
| COL_LETTERS | ABCDEFGHIJKLMN（14） | path_executor.py:106 |
| MAP_COLS / MAP_ROWS | 14 / 11 | path_executor.py:107-108 |
| BLOCK_SIZE | 0.35 | path_executor.py:109 |
| RAW_YAW_TO_MAP_YAW_OFFSET | π/2 | path_executor.py:123 |
| 裁判开火帧 | B5 5B 03 21 01 25 | judgment.cpp:360-366 |
| ROBOT 域号 | 118 | dzros.env:16 |

## 附录 B：术语

- **刻度（tick）**：云台编码器单位，4096/360 ≈ 11.378 tick/度；零位 2047。
- **block**：场地网格单元，0.35 m。
- **DR（dead reckoning）**：轮式里程计死推算。
- **两键开火**：/aim/fire_gate（任务级：到站停稳）+ /aim/fire_ready（瞄准级：预测落点在框内）同时为 1 才开火。
- **PIVOT**：急转弯原地三段转（停→满舵 0.5 s→缓行）。
- **ALIGN_STRAIGHT**：桥洞强制直行保护（当前已关闭）。
- **tuned / v1 / pid**：三套互斥的云台跟踪算法。

— 报告完 —
