---
来源: dzrobot_v2/src/dzbrain/dzbrain/io/contract.py, dzrobot_v2/src/dzbrain/dzbrain/io/ros_io.py, dzrobot_v2/src/dzbrain/dzbrain/io/referee.py, dzrobot_v2/src/dzbrain/dzbrain/main.py, dzrobot_v2/src/dzbrain/dzbrain/tasks/*.py, dzrobot_v2/src/dzbrain/dzbrain/core/blackboard.py, dzrobot_v2/src/dzbrain/dzbrain/core/events.py, dzrobot_v2/src/dzbrain/dzbrain/core/geometry.py
取证: 实
更新: 2026-10-30
status: 现行
取代: 无
---

# dzbrain 接口契约表（对外/对内 ROS 接口 · 人读对照版）

> 机器可读同源版本：`dzrobot_v2/docs/contracts/dzbrain_topics.yaml`。本文是它的**人读对照表**；
> 两边字段一一对应，改一处必须同步改另一处。

## TL;DR（给 agent 看的）

- **一个 Python 进程 = 8 个任务线程 + 主线程里的步进循环 + 一个 4 线程 ROS executor**。线程之间**不发 ROS 话题**，只走黑板（Blackboard）+ 事件总线；跨进程话题只有 **21 条 = 12 入 / 9 出**。
- **话题的唯一真源是契约表**：`dzrobot_v2/src/dzbrain/dzbrain/io/contract.py:58` 的 `TOPICS` dict。装配接线在 `io/ros_io.py` 的 `_create_subs`（`:256`）与 `_create_pubs`（`:386`）；启动对账在 `main.py` 的 `startup_contract`（`:2247`）。
- **本文新增、`13_模块协调与链路展开.md` 没写的**：**每条话题的 QoS 三值**（RELIABLE / BEST_EFFORT + depth + durability）+ 单位 + 字段语义 + 生产/消费者，以及**三个最容易踩的静默失败**。QoS 是跨机经典静默失败的根子，`13` 只列了谁写谁读，没列 QoS。
- **行号只对 dzrobot_v2 提交 `ff231e1e`（2026-10-29）成立**。代码属冻结区（D-008）：可以照着读，不代表可以照着改。

## 0. 三个最容易踩的静默失败（先看这条）

**"静默"的意思是：跨进程/跨机配错不会报错，也不会在日志里留痕迹——只表现为"车没反应"或"看起来在走其实是错的"。**

| # | 名字 | 一句话 | 证据 |
|---|---|---|---|
| SF-1 | **QoS 不匹配 ⇒ 无端点、无报错** | `pursuitAngle` 是 `depth=1 + BEST_EFFORT`。任一侧改成 `RELIABLE`，发布/订阅不互通，rclpy 不报错，但底盘收不到指令（车不动）或收到慢/旧指令（车一顿一顿） | `contract.py:37`、`contract.py:36`、`ros_io.py:169` |
| SF-2 | **单位/坐标系三处漂移 ⇒ 车动但走错、打飞** | ① odom `position` 是**格**，×0.35=米；② `pursuitAngle` `angular.z` 是**度**舵角，不是 rad/s；③ odom 四元数 yaw 是原始车体角，+π/2=地图 yaw。三处都集中在 `contract.odom_to_pose` 与 `publish_command`，别处手写换算即埋雷 | `contract.py:145`、`ros_io.py:499`、`geometry.py:45` |
| SF-3 | **ROS_DOMAIN_ID 分裂 ⇒ 部分话题通、部分收不到** | `env_problems` 判 error 的就是域号不一致。一旦域分裂：DDS 图裂成两半，跨机 `/dzlink` 可能还通、同机 `/odom/localization` 却收不到 ⇒ 车不动。症状是"某些话题有、某些没有"，极易误判为对端没起 | `ros_io.py:103`、`ros_io.py:111` |
| SF-4 | **刹车静默失效 ⇒ 日志自证安全、实际一条零速都没发** | 旧写法 `_safe_call(publish_brake)` 把 `RCLError` 吞成 None，照样打"已发零速指令"。真刹只剩底盘 1s 看门狗。现 `publish_brake` 显式判 `rclpy.ok()`，发不出去就抛异常 | `ros_io.py:509`、`safety.py:461` |

> SF-1 的机理：ROS2 的 QoS **RELIABLE 与 BEST_EFFORT 不互通**（reliability 必须完全一致才建链）；durability `TRANSIENT_LOCAL` 与 `VOLATILE` 不互通同理。`ros_io._qos()`（`:169`）按 `QosSpec` 生成 profile；`describe()` 输出 `"KEEP_LAST(depth)/RELIABILITY/DURABILITY"`（`contract.py:24`）。

## 1. QoS 常量（唯一真源）

`contract.py:37-41`，五个常量，被 21 条话题各自引用：

| 常量 | reliability | depth | durability | 用在哪 |
|---|---|---|---|---|
| `QOS_PURSUIT_ANGLE` | **best_effort** | 1 | volatile | `pursuitAngle`（★唯一 depth=1，底盘要求最新一帧，绝不留旧指令） |
| `QOS_ODOM` | best_effort | 10 | volatile | `/odom/localization` |
| `QOS_SENSOR_FRAME` | best_effort | 10 | volatile | `/imu_data` |
| `QOS_EVENT` | **reliable** | 10 | volatile | 其余 17 条（瞄准/命中/命令类） |
| `QOS_LINK` | best_effort | 10 | volatile | `/dzlink/*`（无人机链路允许丢旧帧） |

证据：`dzrobot_v2/src/dzbrain/dzbrain/io/contract.py:37-41`、`QosSpec`/`describe()` 在 `:24-32`。

## 2. 话题契约总表（21 条 = 12 入 / 9 出）

真源 `contract.py:58-110`；QoS 与行号均来自该表，装配/回调行号来自 `ros_io.py`。**direction 是 dzbrain 视角**（sub=我们订，pub=我们发）。

### 2.1 车外输入（12 条，sub）

| # | key | 话题名 | 类型 | QoS | 单位 / 关键语义 | 生产者 → 消费者 | 契约行 |
|---|---|---|---|---|---|---|---|
| 1 | `odom` | `/odom/localization` | `nav_msgs/Odometry` | **BEST_EFFORT**/10 | position **格**（×0.35=米）；四元数 yaw=原始车体角(+π/2=地图航向) | `serial_reader`（车端 install 副本）→ RosBridge/Localization | `:60` |
| 2 | `imu` | `/imu_data` | `sensor_msgs/Imu` | BEST_EFFORT/10 | rad/s, m/s² | `dzactuator`（底盘串口）→ RosBridge/Localization | `:63` |
| 3 | `offset_center` | `/offset_center` | `std_msgs/Int32MultiArray` | RELIABLE/10 | px/类别/框宽高；字段=[0]dx [1]dy [2]cls+1 [3]框宽 [4]框高 [5..7]关键点 | `det_node`(rknn_pt) → RosBridge/Aim | `:64` |
| 4 | `aim_status` | `/aim/status` | `std_msgs/Int32` | RELIABLE/10 | 0=IDLE 1=SCAN 2=TRACK | `dzrknn_dataprocess` → RosBridge/Aim/Mission | `:67` |
| 5 | `shoot_count` | `/shoot_count` | `std_msgs/Int32` | RELIABLE/10 | 累计发数（与 `referee_hp[3]` 交叉校验） | `dzrknn_dataprocess` → RosBridge/Aim | `:68` |
| 6 | `manual_override` | `/nav/manual_override` | `std_msgs/UInt8` | RELIABLE/10 | 1=遥控接管底盘 | 遥控接收端 → RosBridge/Safety/StepLoop(硬拦①) | `:69` |
| 7 | `manual_gimbal` | `nav/manual_gimbal` | `std_msgs/UInt8` | RELIABLE/10 | 1=遥控接管云台；★**历史相对名，无前导斜杠**，改名断链 | 遥控云台 → RosBridge/Aim | `:71` |
| 8 | `waypoint_cmd` | `/nav/waypoint` | `std_msgs/String` | RELIABLE/10 | 外部命令：`reset:BLOCK,HEAD` / `load:文件` / `单点` / `多点` | `serial_reader`（只为 reset 重锚）/操作员 → RosBridge/Mission | `:73` |
| 9 | `drone_in` | `/dzlink/drone_to_car` | `std_msgs/String` | BEST_EFFORT/10 | JSON 情报：`heartbeat`/`bridge_passable`/`target_zone`/`field_verify`/**`field_layout`**(规划真源) | `dzsmartscout` → RosBridge/DroneTask | `:94` |
| 10 | `referee_hp` | `/HpAndHitmsg` | `std_msgs/UInt8MultiArray` | RELIABLE/10 | ★★**命中闭环真源帧**；=[0]对方HP [1]己方HP [2]被击打方位(0x10=未) [3]硬件发数 [4]剩余弹药 | `dzjudgment` → RosBridge/RefereeTracker/Aim | `:98` |
| 11 | `referee_hit` | `/referee/hit` | `std_msgs/Int32` | RELIABLE/10 | 命中确认（★车端**无生产者**，纯兜底，valid_seen 后忽略防双计） | （无）→ RosBridge | `:103` |
| 12 | `referee_ammo` | `/referee/ammo` | `std_msgs/Int32` | RELIABLE/10 | 剩余弹药（★兜底源；真源是 `referee_hp[4]`） | （无）→ RosBridge | `:106` |

### 2.2 车外输出（9 条，pub）

| # | key | 话题名 | 类型 | QoS | 单位 / 关键语义 | 生产者 → 消费者 | 契约行 |
|---|---|---|---|---|---|---|---|
| 13 | `pursuit_angle` | `pursuitAngle` | `geometry_msgs/Twist` | **BEST_EFFORT**/**1** | `linear.x`=m/s；★`angular.z`=**舵角【度】**(正=左转)；**零 Twist=刹车**；50Hz 保活(底盘 1s 看门狗) | Tracker(唯一非零写者)/Safety/Mission/StepLoop(零) → `dzactuator` | `:77` |
| 14 | `scan_request` | `/nav/scan_request` | `std_msgs/UInt8` | RELIABLE/10 | 0=停扫 1=单趟 2=持续扫描 | Aim(★单写者) → `dzrknn_dataprocess` | `:80` |
| 15 | `aim_fire_gate` | `/aim/fire_gate` | `std_msgs/UInt8` | RELIABLE/10 | 1=允许开火（★新增，修旧版死线；随发一份到 stop_signal 便于对拍） | Mission → `dzrknn_dataprocess` | `:82` |
| 16 | `aim_fire_ready` | `/aim/fire_ready` | `std_msgs/UInt8` | RELIABLE/10 | 1=瞄准合格（预测落点在开火框内）；与 fire_gate 构成**两键开火** | Aim → `dzrknn_dataprocess` | `:84` |
| 17 | `stop_signal` | `/move_base/stop_signal` | `std_msgs/UInt8` | RELIABLE/10 | 1=允许云台扫描+射击（历史话题，云台侧原先无人订阅） | Mission（随 fire_gate 同步发） → （历史无订阅） | `:87` |
| 18 | `motion_state` | `/nav/motion_state` | `std_msgs/UInt8` | RELIABLE/10 | 0 前进 / 1 倒车（仅变化时发） | Tracker → 机/地面站/上位 | `:89` |
| 19 | `voice_switch` | `/voice_switch` | `std_msgs/UInt8` | RELIABLE/10 | 语音 ID | （装配侧 `publish_voice`） → 语音播报节点 | `:91` |
| 20 | `drone_out` | `/dzlink/car_to_drone` | `std_msgs/String` | BEST_EFFORT/10 | JSON：`car_state`{type,block,yaw_deg,phase,ts,seq}+段级字段 / `ack`{type,of:seq} | DroneTask → `dzsmartscout` | `:96` |
| 21 | `aim_lead` | `/aim/lead_x` | `std_msgs/Float32` | RELIABLE/10 | 提前量（px），`lead_px = v_dx × aim.lead_time_s` | Aim → `dzrknn_dataprocess` | `:108` |

装配证据：订阅 12 条 `ros_io.py:256-384`（含 3 条 referee）；发布 9 条 `ros_io.py:386-408`；`REQUIRED_SUBSCRIPTIONS` 9 条 `ros_io.py:30`；启动契约表 `main.py:2247-2291`；`_CONTRACT_ROWS` 见 `main.py:2269`。

## 3. 三类最容易写错的单位/坐标系（SF-2 展开）

| 位置 | 正确口径 | 错在哪 |
|---|---|---|
| `/odom/localization` 的 `position` | **格单位**，`×0.35 = 米`（`geometry.BLOCK_SIZE`） | 当米直接用 ⇒ 全图坐标差 2.86 倍 |
| `/odom/localization` 的四元数 yaw | **原始车体角**，`+π/2 = 地图 yaw`（`RAW_YAW_TO_MAP_YAW_OFFSET`） | 直接当地图 yaw ⇒ 车头方向偏 90° |
| `pursuitAngle` 的 `angular.z` | **舵角【度】**（正=左转） | 当 rad/s ⇒ 打满也就 3° |
| `pursuitAngle` 的 `linear.x` | m/s | 当格/s |

换算**只许在** `contract.odom_to_pose()`（`:145-155`）与 `ros_io.publish_command()`（`:499-507`）里做，别处手写即埋雷。

## 4. 车内通信（非 ROS，供对照）

线程之间**不发话题**，走黑板（事实）+ 事件总线（通知）。**它们不是 ROS 接口**，本文只为完整对照列出；`13_模块协调与链路展开.md` §2 已有详细卡片，此处只给键/事件清单。

### 4.1 黑板键（`core/blackboard.py:21-95`，键名唯一拼写源）

`pose` / `pose_raw` / `odom_age` / `route` / `route_index` / `route_s` / `mission_phase` / `stop_point` / `target` / `aim` / `aim_red_face` / `drone` / `field_layout` / `layout_arrivals` / `field_verify` / `fix` / `fix_est` / `safety` / `telemetry` / `pursuit_gear` / `motion_limit` / `car_state`（22 个常量）。

读写契约要点（`13` §5 已细讲，勿重开）：`P_POSE` 有 3 个写者（Localization/Mission/Drone）；`P_SAFETY.ok` 是唯一强制停车者；`P_PURSUIT_GEAR`/`P_MOTION_LIMIT` 是**观测/请求**，`None` 即 no-op 逐位一致。

### 4.2 事件类型（`core/events.py:24-48`，21 个）

`MISSION_STARTED` / `MISSION_FINISHED` / `ROUTE_LOADED` / `WAYPOINT_REACHED` / `ARRIVED_STOP_POINT` / `STOP_POINT_DONE` / `TARGET_FIRST_SEEN` / `TARGET_LOST` / `FIRE_ALLOWED` / `FIRE_BLOCKED` / `HIT_CONFIRMED` / `AMMO_CHANGED` / `POSE_ANCHORED` / `DRIFT_WARNING` / `DRONE_ONLINE` / `DRONE_OFFLINE` / `BRIDGE_INFO` / `FIELD_VERIFY` / `SAFETY_STOP` / `SAFETY_CLEAR` / `TASK_FAULT`。

## 5. 两把钥匙 / 单写者契约（动了就破约）

- **两键开火**：`/aim/fire_gate` ← Mission（任务级：到站停稳）；`/aim/fire_ready` ← Aim（瞄准级：目标确认+落点合格+冷却过）。云台补丁只在**两者都为 1** 时开火；任一任务误判都不会误开火（`ros_io.py:530-550`）。
- **`/nav/scan_request` 单写者**：`aim.gimbal_lock_enabled=true`（默认）时只有 AimTask 写，Mission 一个值都不发（`aim.py:19-36`）。
- **非零驱动指令单写者**：只有 Tracker 发非零 `pursuitAngle`；零 Twist 有 4 个写者（Safety/Tracker/Mission/StepLoop）。
- **`P_ROUTE` 唯一装载者**：Mission `_load_tokens`（`mission.py:2941`）。

## 6. 未核实清单（本篇未取证据，别当成已确认）

1. `voice_switch` 的实际订阅节点与取值编码 —— 未见消费方源码。
2. `stop_signal` 当前是否被云台补丁真正订阅 —— `contract.py:88` docstring 记"原先无人订阅"，补丁版是否已订阅未取证。
3. `dzsmartscout` 端 `heartbeat`/`bridge_passable`/`target_zone` 的**精确 JSON 字段** —— 仅据车端 `DRONE_PAYLOAD_TYPES`（`drone.py:146`）与 `normalize_field_layout`（`drone.py:254`）校验推断，未读机端 `protocol.py`。
4. `path_executor`（旧核）与 dzbrain 同时跑时的 `pursuitAngle` 抢写 —— 本文未验证二选一装配是否真的互斥。

## 7. 机器可读版与本文的同步

`dzrobot_v2/docs/contracts/dzbrain_topics.yaml` 是本文的 JSON/YAML 同源版（含 QoS 三值、field_semantics、evidence、`silent_failure_modes`、`unverified`）。**改话题一律先改 `contract.py` 的 `TOPICS`**（唯一真源），再同步两份文档；`canonical_topic_name()` 已由 `test_contract.py` 钉死两条历史相对名不改名。

## 8. 相关文档

- `docs/20_资料索引库/车端/13_模块协调与链路展开.md` —— 模块卡片、线程图、旧核分叉（本篇的兄弟篇，补 QoS）。
- `docs/20_资料索引库/车端/15_dzbrain零基础源码导读.md` —— 源码导读。
- `dzrobot_v2/docs/contracts/dzbrain_topics.yaml` —— 机器可读契约表。