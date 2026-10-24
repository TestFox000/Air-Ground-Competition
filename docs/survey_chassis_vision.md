# 底盘驱动 / 里程计 / 辅助接口 —— 只读调研报告

- 调研对象：`D:\User\Desktop\dzrobot_v2`（本工作区副本）
- 性质：**只读分析**。除本文件外未创建/修改任何文件，未 SSH，未执行任何改变车辆状态的命令。
- 全部结论均可按 `文件:行号` 复核；不确定处显式标 **未确认**。

## 0. 范围与方法

| 项 | 内容 |
|---|---|
| 逐行通读 | `src/dzactuator/src/dzactuator.cpp`(1448 行)、`include/dzactuator.h`(341)、`src/Quaternion_Solution.cpp`(280)、`include/Quaternion_Solution.h`、3 个 launch、`src/car_localization/car_localization/serial_reader.py`(271)、`car_localization` 的 package.xml/setup.py/localization.launch.py、`src/naviPointPath/*`、`tools/{global_planner.py,route_check.py,shoot_planner.py,param_spec.py,healthcheck.sh,rc_control.py}`、`config/robot.yaml`、`config/map.yaml`、`start_all.sh` |
| 按需读关键段 | `car_localization/car_localization/path_executor.py`(2622 行)：话题/发布段、纯追踪公式段、接管闸门段（未逐行通读全文） |
| 未读 | `dzrknn_dataprocess` / `dzjudgment` / `rknn_pt` 的 package.xml、STM32 固件源码（`src/history/附加资源/DZVCU程序/` 内有工程，未打开） |
| 未做 | 未 SSH，故**本副本与车上 `~/dzrobot` 的差异未核对** |

---

## 1. 底盘节点 dzactuator

- 编译目标：`CMakeLists.txt:87` `add_executable(dzactuator_node src/dzactuator.cpp src/Quaternion_Solution.cpp)`；入口 `dzactuator.cpp:20-28`，节点名 `dzactuator`（`:23`）。
- 主循环：`Control()` `dzactuator.cpp:1256-1338`；`rclcpp::Rate rate(100)`（`:1270`，注释写 `// Loop rate of 50Hz` —— **注释与代码矛盾**）；`rclcpp::spin_some()` 在循环内（`:1273`），无多线程 executor。
- **架构要点**：所有发布 + 所有下行指令都在 `if (sensor_ok)` 分支内（`:1287-1335`）。即「串口读不到一帧完整数据 → 本周期既不发布也不下发指令」。

### 1.1 订阅话题（声明处 `dzactuator.cpp:1160-1171`）

| 话题 | 类型 | QoS | 单位约定 | 作用 / 行号 |
|---|---|---|---|---|
| `cmd_vel` | geometry_msgs/Twist | KeepLast(100) best_effort | linear.x **m/s**；angular.z **rad/s** | DWA 分支；`TargetAngle=atan(CARL·w/v)·57.3`（`:72-101`） |
| `pursuitAngle` | geometry_msgs/Twist | KeepLast(1) best_effort | linear.x **m/s**；angular.z **度（舵角）** | 循迹主通道（`:143-184`）；linear.y 兼作模式档位 |
| `logicontrol` | Twist | KeepLast(1) best_effort | linear.z>1 → 平行驾驶 | 手动驾驶分支（`:188-239`）；**本工作区无任何发布者** |
| `voice_switch` | UInt8 | KeepLast(1) best_effort | 语音 ID | `:241-246` |
| `/laser_switch` | UInt8 | 可靠 depth 10 | 0/1 | 写下行帧 buf[7]（`:248-257`、`:408`） |
| `vehicle_state` | UInt8 | KeepLast(1) best_effort | 状态码 | 存 `voice_nums[0]`（`:258-262`） |
| `vehicle_Drivestate` | UInt8 | 同上 | 状态码 | 存 `voice_nums[1]`（`:264-268`） |
| `CameMo_Conl` | Int32MultiArray | 同上 | data[0..3]=云台位置/速度刻度 | 直写下行帧（`:270-283`） |
| `StopCmd` | UInt8 | 同上 | 非 0=停 | `:285-291` + `:379-382`；**本工作区无发布者** |

**单位坑（全项目最易踩）**：两个回调的形参都叫 `angular.z`，含义不同 ——
`cmd_vel` 是 **rad/s**（`:72-74` 注释 + `:81` 的 `·57.3` 换算）；`pursuitAngle` 是 **度**（`:147-148`）。
`path_executor.py` 发布 `pursuitAngle` 时确实给的是度（`path_executor.py:1779-1783`，公式见 §6）。

**速度映射**：`TargetSpeed = |clamp(v·23/0.33, ±255)|`（`:78` / `:151` / `:213`）。
⇒ 1 速度计数 ≈ 0.01435 m/s；巡航 0.15 m/s → 10.45 → 被 `int` 字段截断成 **10**（实得 ≈0.1435 m/s，−4.3%）。
上限 255（≈3.66 m/s）远超「23 = 满速」的口径，实际不构成限速。

**舵角映射**：`TargetAngle = 舵角 + 60`，再夹到 `[25,95]`（`:87-101` / `:152-163`）⇒ 伺服字节 60=直行，行程 ±35°；`path_executor` 侧已先夹到 ±30°（`path_executor.py:1783`）。

### 1.2 发布话题（声明处 `dzactuator.cpp:1146-1156`）

| 话题 | 类型 | depth | 内容 | 行号 |
|---|---|---|---|---|
| `odom` | nav_msgs/Odometry | 50 | 轮式死推算位姿 + ×50 的 twist | `:1148`、`:646-703` |
| `raw` | sensor_msgs/Imu | 20 | 原始 IMU + 四元数 | `:1149`、`:576-634` |
| `imu_data` | sensor_msgs/Imu | 1 | 零偏校正+滤波后 IMU（**serial_reader 的航向源**） | `:1152`、`:501-569` |
| `PowerVoltage` | Float32 | 10 | 电池电压 V | `:1146`、`:710-716` |
| `Battery_Percentage` | Float32 | 10 | 0..1，带单调性迟滞 | `:1147`、`:728-816` |
| `odom_msg_valid` / `imu_msg_valid` | UInt8 | 10 | 数据有效标志 | `:1150-1151` |
| `navi_cmd` | UInt8 | 10 | 平行→自主切换通知（=1） | `:1153`、`:137-139` |
| `current_vel_state` | Twist | 10 | 档位/驾驶模式回显 | `:1154`、`:174-183` |
| `wakeup_signal` | Int32 | 10 | 上行帧 rx[32] 跳变触发 | `:1155`、`:1026-1035` |
| `camemo_feedback` | Int32MultiArray | 10 | 云台反馈 6 元组 | `:1156`、`:823-845` |

**频率**：发布与主循环同频 = `min(100 Hz, 下位机帧率)`（`:1270`）。实际值 **未确认**（未 SSH；车上可 `ros2 topic hz /odom` 核对）。

**TF**：`odom→base_link` **未发布** —— `sendTransform` 被注释（`:702`），但 `tf_broadcaster_` 仍被构造（`:1175`）。launch 只发 `base_link→imu_link`(0,0,0.08) 与 `base_link→laser`（`bringup.launch.py:39-51`）。当前栈无人消费 TF，但任何引入 nav2/rviz 的方案会缺这条链。

---

## 2. 里程计实现

核心在 `dzactuator.cpp:1289-1311`：

```
leftDistance  += Robot_Vel.Left            # 累积原始计数（仅供日志）
leftSpeed      = Robot_Vel.Left  / ticksPerMeter      # :1292
rightSpeed     = Robot_Vel.Right / ticksPerMeter      # :1293
linear_Speed   = (leftSpeed+rightSpeed)/2             # :1295
ThetaSpeed     = (R-L)·2π / ticksPer2PI               # :1296
Robot_Pos.Z   += ThetaSpeed  → normalizeAngle         # :1301-1302
Robot_Pos.X   += linear_Speed·cos(Z)                  # :1303
Robot_Pos.Y   += linear_Speed·sin(Z)                  # :1304
```

- `Robot_Vel.Left/Right` 实际是**本帧计数增量**（`Odom_Trans` 只做 int16 大端拼接，不除 dt，`:43-52`），所以 `leftSpeed` 的单位是「**米/帧**」、`ThetaSpeed` 是「**弧度/帧**」。
- 因此积分**故意不乘 dt**（与「每帧增量」口径自洽），但隐含「每帧时间恒定」假设 ⇒ **下位机帧率变化即比例失真**。
- 只在 `|linear_Speed|>1e-5` 时积分（`:1298-1307`），静止时 `montion_flag=False` 且位姿冻结。
- **航向来源 = 轮差积分，IMU 完全不参与里程计**（IMU 只用于发布 `/raw`、`/imu_data`）⇒ 航向随轮径误差/打滑持续漂移。
- **本节点无漂移补偿**。唯一的 IMU 处理是 `imu_bias_correction`（`:451-499`）：开机前 200 帧取均值做零偏（≈2 s，隐含「静止且水平」假设），此后仅做减法。
- 姿态解算 `Quaternion_Solution.cpp:214-280` 是 Mahony 型互补滤波：**只有加速度计校正、无磁力计**（`:221-253`），且 `twoKi=0`（`:211`）⇒ roll/pitch 收敛，**yaw 是纯陀螺积分、必然慢漂**。积分步长硬编码 `1/100`（`:255-257`）与主循环 100 Hz 对应，但同文件 `IMU_SAMPLING_FREQ = 50.0f`（`Quaternion_Solution.cpp:10`）自相矛盾。

**`ticksPerMeter` / `ticksPer2PI` 使用点（唯一两处）**：`dzactuator.cpp:1292`、`:1293`（ticksPerMeter）、`:1296`（ticksPer2PI）。取值来自配置中心 `robot.yaml:385`(=3290.0)、`:392`(=3236.0)。
> ⚠️ `robot.yaml:379` 注释称 `calibrate_lineSpeed: 1=用原始值 / 0=用标定值换算`，但代码里该参数**只控制是否打印 100 Hz 调试日志**（`:1317`），换算恒定使用两个标定值 ⇒ **注释与代码不符**，属「改了以为生效」的静默坑。

**twist 上报疑似 2 倍偏差**：`odom.twist.twist.linear.x = linear_Speed * 50`、`angular.z = ThetaSpeed * 50`（`:669`、`:671`）。×50 是按 50 Hz 假设写的，而主循环声明 100 Hz（`:1270`）。若真按 100 Hz 运行，上报速度是实际的 2 倍 —— 这会影响任何基于 `twist` 的判据（例如「车是否停稳」）。**建议上车用 `ros2 topic echo /odom` 对实测车速核对**。

其他细节：`odom.pose.pose.position.z = 0.2` 硬编码（`:665`）；协方差矩阵（`dzactuator.h:75-102`）给轮式 yaw 的方差仅 `1e-3`/`1e-6` 量级，**与实际漂移特性不符**（若将来接 EKF 会过度信任轮式航向）。

### 2.1 漂移实际在哪里被补偿

dead-reckoning 的补偿**不在底盘，而在下游两级**：

1. `serial_reader.py` —— **丢掉底盘的旋转航向，只取「相邻采样点距离」**（旋转不变量），沿 IMU 航向做步长积分（`serial_reader.py:214-225`；文件头 `:14-23` 记录了实测依据：同一 90° 右转在两次运行里轨迹向相反侧发散）。航向优先 IMU、2 s 无数据回退 odom（`:197-212`、`IMU_TIMEOUT=2.0` `:62`）；位置换算成「格」单位后发布 `/odom/localization`（`:227-244`）。
2. `path_executor.py` —— **每次到位把残余漂移折进吸附偏移**（`snap_x/y/yaw_off`，`:1533-1540`、`:2319-2322`），并用 `drift_warn_m=0.30`（`robot.yaml:319`）做单段漂移告警。

即：**底盘输出的 `/odom` 其旋转分量已被下游判定为不可用**；可用的是「距离」。这是当前架构最重要的隐含契约，新版本必须显式处理。

### 2.2 坐标系与单位契约（已核对，三处一致）

- 米制：`x = (10.5 − row0)·0.35`、`y = (13.5 − col0)·0.35`（row0/col0 为 0 基下标）（`path_executor.py:523-524`）。
- 格制：`/odom/localization` 用**格**为单位，x∈[0.5,10.5]、y∈[0.5,13.5]（`serial_reader.py:227-234`；`path_executor.py:746`、`:1602-1603` 乘 0.35 转米）。
- 反查：`row = round(10.5 − x_blocks)`、`col = round(13.5 − y_blocks)`（`serial_reader.py:68-75`）。
- 地图范围 x∈[0.175, 3.675] m（11 行）、y∈[0.175, 4.725] m（14 列）；起点 N10 = (0.525, 0.175) m = (1.5, 0.5) 格。
- 航向：`HEADING_YAWS = {x:0, -x:π, y:+π/2, -y:−π/2}`（`serial_reader.py:48`）；**IMU 偏航 CCW 为正**，`FLIP_RAW_YAW_SIGN=False`（`:50-53`）；raw→map 另加固定 +π/2（`path_executor.py:123`）。

---

## 3. 看门狗与失效保护

### 3.1 时限常量（`dzactuator.h:320-322`）

| 常量 | 值 | 用途 |
|---|---|---|
| `ACTIVITY_TIMEOUT` | 0.5 s | 模式仲裁判「某源是否活跃」 |
| `PURSUIT_LOW_TIMEOUT` | 3.0 s | DWA→Pursuit 回切去抖 |
| `CMD_WATCHDOG_S` | 1.0 s | **无速度指令强制停车** |

### 3.2 底盘侧机制

- **指令看门狗** `checkCommandWatchdog()`（`:1346-1374`）：`t = min(t_pursuit, t_dwa)`（`:1361`，「任一源新鲜即算新鲜」，逻辑正确）；`t ≥ 1 s` → `TargetSpeed=0`、`SpeedDirection=0x00`（`:1367-1373`），**只停速度、不动舵角**。
  - ⚠️ **只监视 `pursuitAngle` 与 `cmd_vel`**：`logicontrol` 不更新时间戳（`:188-239`）⇒ 若仅靠 `logicontrol` 驾驶，看门狗每周期都会把速度清零。（该通道在当前工作区无发布者，属潜在风险。）
  - ✅ 浏览器遥控走的是 **`pursuitAngle`**（`rc_control.py:89`），因此被看门狗正确覆盖。
- **模式仲裁** `modeTimerCallback()`（`:1376-1448`），10 Hz 定时器（`:1214-1217`）：单源活跃即切该源；双源时 `pursuit.linear.y ≥ 0.5` → DWA，否则若 pursuit 已从高值跌落 ≥3 s 且其速度大于 DWA → Pursuit。`cmd_vel` 无发布者 ⇒ 常态 `current_mode_=2`（Pursuit）。
- **StopCmd 通道**（`:285-291`、`:379-382`）：收到非 0 后连续 10 个周期把速度置 0。**本工作区无发布者**。

### 3.3 急停 / 手动接管（在 path_executor + rc_control 侧）

- `path_executor` 的 `_GatedCmdPub`（`path_executor.py:568-577`）：`manual_override=True` 时**吞掉全部自动指令**；用「包一层唯一出口」而非逐点加 `if`，避免漏判（该文件有 15 处 publish）。回调 `:1343-1354`，开关话题 `/nav/manual_override`（`:804-805`）。
- `rc_control.py`（浏览器遥控服务端）：
  - 接管时发 `/nav/manual_override=1` + `/nav/manual_gimbal=1`（`:157-158`），先刹一脚（`:162`）。
  - **急停粘性**：`_estopped` 置位后看门狗不释放接管、持续补发零速，**只有显式「解除急停/释放控制」才恢复**（`:347-371`、`:166-172`、`:390-392`）。注释 `:350-359` 记录了踩过的坑（最初非粘性 = 「停了 2 秒」）。
  - 接管看门狗：0.5 s 轮询，静默 > 2 s 自动刹车并释放（`:376-402`，`WATCHDOG_S=2.0` `:63`）。
  - 四层失效保护（文件头 `:22-30`）：按住才动 / 数值钳制 / 接管看门狗 2 s / **底盘看门狗 1 s（最底层兜底）**。
  - 安全包络：`MAX_SPEED=0.30`、`MAX_STEER=40°`、云台 `PITCH 950~2400`、`YAW 1022~3071` 刻度（`:51-61`）。

### 3.4 缺口（重要）

1. **传感路径门控指令路径**：读串口失败即完全不发帧（`:1287`、`:1334`）。若下位机「只静默不宕机」，dzactuator 既不跑看门狗也不发停车帧 ⇒ **下位机保留最后一条速度指令**。
2. **帧头等待循环形同阻塞**：`while (++guard < 200000)`（`:897`）配合每次读 30 ms 超时（`:1196`）⇒ 最坏约 `200000×30 ms ≈ 100 分钟`才放弃。注释自称「不再无限死转」，实际在「端口活着但下位机不吐帧」这一模式下等价于长时间阻塞。
3. 看门狗本身只在 `sensor_ok` 分支内被调用（`:1332`）——与 1、2 叠加放大后果。

---

## 4. 串口协议（与下位机 STM32）

- 设备/波特率：`/dev/dzactuator`（udev 符号链接）、115200（`robot.yaml:370-371`）；读超时 `simpleTimeout(30)`（`:1196`）。

### 4.1 下行帧（ROS→STM32，23 字节，每周期 1 帧）—— `sendCarInfoKernel()` `:376-449`

| 偏移 | 含义 | 行号 |
|---|---|---|
| [0][1] | 帧头 `0xA5 0x5A` | `:392-393` |
| [2] | 长度 `20`（注释写「22−3=19 字节」，**注释与值不符**） | `:394` |
| [3][4] | 舵角 int16 大端 = `TargetAngle×10`（0.1° 单位） | `:396-398` |
| [5] | 方向 `0x00` 前进 / `0x01` 倒退 | `:400`、`:103-108` |
| [6] | 速度字节（0..255，23 ≈ 0.33 m/s） | `:403` |
| [7] | **激光常亮 `LaserGunSwitch`** | `:408` |
| [8..15] | 云台 位置0/1、速度0/1，各 int16 大端 | `:410-420` |
| [16][17] | 云台 时间0/1 | `:422-424` |
| [18][19][20] | `Fun` / `Orin_ID` / `Set_ID_Num`（功能、使用 ID、重置 ID） | `:426-430` |
| [21] | `VoiceSwitch`（语音 ID，预留位） | `:434` |
| [22] | 校验 = `sum(buf[2..21])`（累加和，**非 BCC**） | `:436-439` |

> 头文件另定义了 `FRAME_HEADER 0x7B / FRAME_TAIL 0x7D / SEND_DATA_SIZE 11` 与 XOR 校验 `Check_Sum`（`dzactuator.h:60-65`、`:245`、`:857-876`），但**下行完全不用**——只在析构函数里发一次「旧格式 11 字节停车帧」（`:1227-1246`）。即项目内**并存两套协议**，生效的是 23 字节 `0xA5 0x5A` 帧。

### 4.2 上行帧（STM32→ROS，34 字节 = `0x7B` + `0x1F` + 32 字节载荷）—— `Get_Sensor_Data_New()` `:883-1039`

| rx 下标 | 含义 | 行号 |
|---|---|---|
| [2][3] / [4][5] | 左/右轮计数增量 int16 | `:968-969` |
| [6][7] | 电池电压（mV，`/1000` 转 V） | `:973-975` |
| [8..13] | 三轴加速度 int16 | `:978-980` |
| [14..19] | 三轴角速度 int16 | `:985-987` |
| [20..31] | 云台 位置/速度/负载 ×2 | `:1003-1015` |
| [32] | 跳变位 → `/wakeup_signal` | `:1021-1035` |
| [33] | 校验位（**未校验**） | `:958-962` 已被注释 |

- 换算：加速度 `raw·24/32768·9.8`（`:993-995`）；角速度 `raw·1000/32768/57.2957795` → rad/s（`:997-999`）。
- ⚠️ **上行 BCC 校验被注释掉**（`:958-962`）⇒ 帧头/长度之外无完整性检查，错帧会直接进里程计与安全判据。
- ⚠️ 长度字段判 `0x1F`(=31) 但随后读 **32** 字节（`:928` vs `:930`）——约定可疑，**未确认**（需对照 STM32 工程，工作区内有 `src/history/附加资源/DZVCU程序/`）。

---

## 5. 辅助包接口（相机 → 视觉 → 云台 → 射击 → 循迹）

| 节点 | 订阅 | 发布 |
|---|---|---|
| `usb_cam`（`/usb_cam_node`） | — | `/image_raw`（640×480@30, yuyv；`robot.yaml:166-176`） |
| `det_node`（源码在 `src/rknn_pt`） | `/image_raw` | **`offset_center`**（Int32MultiArray depth1）（`det_node.cc:30-32`）；参数 `model_path/draw/thread_num/max_fps/hot_reload`（`:37-41`）；模型 `src/rknn_pt/model/light_det.rknn`（8.1 MB） |
| `dzrknn_dataprocess`（云台自瞄） | `offset_center`、`camemo_feedback`、`/nav/scan_request`、`/nav/manual_gimbal`、`/imu_data`、`/shoot_reset`（`dzrknn_dataprocess.cpp:844-863`） | `LaserShot_Command`、`CameMo_Conl`、`voice_switch`、`/shoot_count`、`/aim/status`（`:830-840`）；30+ 可调参数（`:871-935`） |
| `dzjudgment`（射击驱动） | `LaserShot_Command`、`/referee_cmd`（`judgment.cpp:51-59`） | `HpAndHitmsg`、`all/enemy/self_Material_Number`（`:62-65`）；串口 `/dev/dzjudgment` 115200，写 6 字节帧 `B5 5B 03 21 01 25` |
| `path_executor`（循迹主控） | `/odom/localization`、`/nav/waypoint`、`/shoot_count`、`/nav/manual_override`、`/aim/status`、`/offset_center`（`path_executor.py:747-825`） | **`pursuitAngle`**、`/move_base/stop_signal`、`/nav/scan_request`、`/nav/motion_state`、`/voice_switch`（`:776-829`）；50 Hz 控制定时器（`:838`） |
| `serial_reader` | `/odom`、`/imu_data`、`/nav/waypoint` | `/odom/localization`（**格单位**）（`serial_reader.py:96-108`） |

- `offset_center` 数据布局：`[dx, dy, cls, box_w, box_h, kpt_x, kpt_y, kpt_conf]`，`cls 0=无目标 1=蓝靶 2=红靶`（`path_executor.py:818-825`）。
- 开火链路完整说明见 `robot.yaml:179-202`（记录了「dzjudgment 未启动 → `/LaserShot_Command` Publisher 3 / Subscription 0，开火进空话题」的事故）。**该节点必须常驻**。
- 遥控开火会同时点亮激光常亮 3 s（因为判罚板脉冲肉眼不可见，`rc_control.py:245-262`、`:337-339`）。

---

## 6. 导航 / 规划工具链

### 6.1 `global_planner.py`（926 行，**免 ROS，可离线跑**）

- 作用：地图要素（起/终/雷/障/靶/桥）→ 网格建模 → 任务排序 → 阿克曼 A* → **输出一条路线串**（头部文档 `:3-54`）。
- 用法：`python3 tools/global_planner.py --config config/robot.yaml --emit-route --quiet`；argparse 在 `:776-792`：`--config`（默认 `os.path.expanduser("~/dzrobot/config/robot.yaml")` `:780` —— **Windows/异目录必须显式传**）、`--start/--end/--mines/--obstacles/--targets/--bridges/--start-heading(n|w|s|e)/--emit-route/--quiet`。
- 输入：`robot.yaml` 顶层 `map:` 段（`:728-737`），否则 `$ROBOT_MAP` → 配置同目录 `map.yaml`（`:686-713`）；射击距离窗从 `path_executor.shoot_distance_min/max` 读（`:755-768`，默认 0.20/0.45）。
- 输出：**只打印，不写文件**。`--emit-route` 打印路线串（`:881-883`）；退出码 0/1/2（`:815/:818/:852/:922`）。
- 算法要点：动作代价 前1.0/转1.6/倒1.9/倒转2.4（`:84-87`）；方向罚 0.55（`:90`）；跨区罚 4.0（`:92`）；8 雷全排列 → 初筛 `TOP_K_LEI_ORDERS=240` 再精算（`:94`、`:564-582`）；A* 状态 `(行,列,朝向)`（`:223-274`）；折返 >150° 自动转倒车段（`:328-380`、`:372`）；射击候选点朝向罚 同列0/同行0.8/对角2.4（`:490-509`）。
- **token 格式**（`:647-680` docstring，与 `follow_route_command` 兼容）：普通 `B5`；倒车 `R:B5`；射击停站 `B5*`（**只在首次经过时标**，`:670-672`，避免同一位置停两次浪费弹药）；末尾朝向 `,x` / `,y`。
- 地图真源：`config/map.yaml:5-15`（`map.name/start/end/mines/obstacles/targets/bridges`）。

### 6.2 `route_check.py`（880 行，**免 ROS**；缺 rclpy 时自动降级）

- 作用：出车前对「即将执行的路线」做 **5 层校验**（`:34` 起）。
- 用法：`python3 tools/route_check.py`（不带参数 = 校验 `robot.yaml` 的 `mission_blocks`，`:864-869`）；或 `--route "…"`、`--from-planner`（`:810-824`）。
- 五层：
  1. **语法**：token 合法性 —— 优先调用 `path_executor.validate_mission_blocks`（`:433-434`，标注「权威实现」），不可用时用内置 `fallback_validate`（`:174-214`）；
  2. **解析**：`parse_route` 模拟 `follow_route_command` 的顺序（`R:`→标签→`*`→`,朝向`）（`:217-255`、`:449-456`）；
  3. **几何**：越界（`MAP_X/Y_MIN/MAX` + 安全边距 0.60，`:91-95`、`:459-470`）；挡板穿越余量 ≥ `0.10 m`（`:85`、`:261-284`）；障碍距离 < `0.30 m` 仅告警（`:86`、`:287-306`）；>45° 急转告警（`:89`、`:354-366`）；
  4. **任务**：雷覆盖（容差 0.6 格 = 0.21 m，`:387-402`）；桥洞水平直穿分组（`:311-344`）；靶覆盖与射击距离窗 `[0.20−0.03, 0.45+0.06]`（`:578-582`、`:615-618`）；
  5. **安全**：总里程（info）、倒车分组与长度告警（`:646-683`）。
- 输出：仅 stdout；退出码 `0` 全过/仅告警、`1` 有 FAIL、`2` 参数或文件问题（`summarise()` `:698`）。
- **与 global_planner 的关系**：planner 生成 → 人工/脚本写回 `robot.yaml` 的 `mission_blocks` → route_check 校验。**生成与校验分离**（planner 自身无校验）。
  - `route_check` 的 `run_planner()`（`:780-807`）可直接跑 planner 并取 stdout，但**不与 `robot.yaml` 里现行的 `mission_blocks` 比对** ⇒ 「配置是否已同步最新规划」它查不出来。
  - `bridge_order`（`robot.yaml:258-260`）在 route_check 中**零引用** ⇒ 桥洞穿越**方向/顺序**未校验。
  - `robot.yaml:299` 注释宣称「route_check.py 会校验 obstacle_blocks 与 map.yaml 一致性」——**该检查在 route_check.py 中不存在**（我已核对其键读取：`:751-774` 只读 obstacles/bridges/mission_blocks/shoot 距离）。
- **真源判定**：生成真源 = planner 输出；**运行真源 = `robot.yaml` 的 `mission_blocks`**（`path_executor.py:425-434` 会用配置覆盖代码内置常量，并在启动横幅打印校验结果）。

### 6.3 `shoot_planner.py`（641 行，**免 ROS**；既是 CLI 也是被 import 的库）

- 作用：给定靶标格 + 障碍表，用 14×11 栅格做「距离 / 视线 / 阿克曼可达」三层**硬筛选**，再按 4 项代价**打分排序**，输出每靶的候选射击停点链。
- 用法：`python3 tools/shoot_planner.py --targets G11 --json`；argparse `:539-554`（`--config/--targets/--obstacles/--pose/--dist-target/--dist-min/--dist-max/--keep/--json/--quiet`）。**不写任何文件**；退出码：无靶标 → 2（`:566-568`），否则 0。
- **评分输入（这是本报告要求的重点，逐项给行号；分数越低越好）**：

| # | 代价项 | 公式 | 权重 / 常数 | 计算行 |
|---|---|---|---|---|
| ① | 距离误差 | `\|d − dist_target\| / dist_sigma` | `w_distance=1.0`（`:316`）；`dist_target` 默认 0.30（`:314`），运行时取 `robot.yaml:270` 的 **0.35**；`dist_sigma=0.10` 恒为 0.10（`:315`） | `:346` |
| ② | 云台偏角 | `deg(yaw_err)/90`（车头朝向与「看向靶」之差） | `w_gimbal=0.6`（`:317`；`robot.yaml:277` 同名可配） | `:357` |
| ③ | 路径绕行 | 直线可达 `straight/0.35`；不通时取切比雪夫上界 | `w_detour=0.5`（`:318`；`robot.yaml:278`） | `:365` |
| ④ | 轴向对齐 | 同列 `0.0` / 同行 `0.8` / 对角 `2.4`（`:378-382`） | `w_axis=1.5` —— **硬编码，CLI 与 path_executor 都不传 ⇒ 不可配置**（`:319`、`:473-476`） | `:377-387` |

  聚合：`total = 1.0·dist + 0.6·gimbal + 0.5·detour + 1.5·axis`（`:384-387`）→ 升序 → 取前 `keep`（默认 3）。
- **硬约束（不进分数）**：距离窗 0.20~0.45 m（`:219-220`、`:261-262`）；候选格可通行（`:251`）；视线 Bresenham（`:156-199`，含挡板约束）；阿克曼可达 ≥2 个正交方向（`:291-305`）；严格池为空时兜底取最近 4 个（`:268-275`）。障碍/挡板**只作硬约束**。
- **未参与评分的输入（逐项确认）**：**雷（`map.yaml:12`）、桥（`map.yaml:15`、`robot.yaml:252`）、倒车段、时间/耗时、弹药余量、靶标优先级**。`R:` 前缀仅在 `parse_block`（`:70-73`）被剥掉，无代价项。
- 消费方式：`path_executor` 以 importlib **懒加载**（`path_executor.py:1812-1850`），失败降级为「靶自身当射击点」（宁可漏打不可卡死）；`SHOOT@<靶>` 指令会现场插点（`path_executor.py:1132-1177`）。

### 6.4 `healthcheck.sh`（365 行）

- 用法：`./healthcheck.sh`（完整）/ `./healthcheck.sh --pre`（只静态）；退出码 全过 `0` / 仅告警 `2` / 有 ❌ `1`（`:354-365`）。
- 六段：① 设备（`/dev/video0`、`/dev/ttyACM*`）`：43-50`；② 配置中心 + 三条硬性不变量 + ROS params 结构 + declare/get 静态配对 + 狙击链路 + 模型文件 `：55-217`；③ 运行时参数比对（`ros2 param dump`，MISS/DIFF）`：222-313`；④ 节点与话题速率 `：318-332`；⑤ 8080 端口与 tf 残留 `：337-348`；⑥ 结论。
- 面向**新核心**（检查 `path_executor_node`/`serial_reader`，`:323`）。
- ⚠️ **一处假绿**：`det_node` 的 declare/get 配对检查按「目录路径含包名」过滤（`:148-151`），而 `det_node` 的 C++ 在 `src/rknn_pt/src/det_node.cc`（路径不含 `det_node` 字样）⇒ 命中 0 个文件，脚本**仍打印 ✅**（`:173`）。`dzrknn_dataprocess` 那一半正常。

### 6.5 `param_spec.py`（226 行）

- 调参网页的**参数白名单规范表**：`NODES` 别名 3 条（`:27-31`）、`PARAM_GROUPS` 8 组共 34 项（`:35-201`，含 `mission_blocks` 的 list 型条目 `:194`）。
- **自身不做 IO、不做校验**（无 `import yaml`）；只提供 `iter_items/find_item/node_of_group`（`:204-226`），仅被 `tools/cam_viewer.py` 导入。真正的校验与落盘（白名单过滤、类型/范围、`.bak` + 回读回滚）在 `cam_viewer.py:919-959`。
- 条目**无 `default` 字段** ⇒ 默认值即 `robot.yaml` 现值（设计声明 `:7-9`）。

### 6.6 `src/naviPointPath`（纯数据包）

- 声明即自述：「Navigation waypoint and map data … **Pure data package, no sources**」（`package.xml:6`）；`CMakeLists.txt:7-8` 安装 `forwardcar_paths/backcar_paths/map`。
- 关键文件：`map/1.txt`(117 行)、`mapping.txt`(214 行)、`forwardcar_paths/1.txt`、`backcar_paths/1.txt`(6183 B)。
- 文件内容：每行 7 列 `x,y,z,qx,qy,qz,qw`（位姿轨迹点，见 `map/1.txt:1`）。对照《案例10：全局路径录制与纯追踪导航》，是**旧方案录制的路径文件**。
- **引用情况：全工作区无任何 `.py`/launch 引用该包**（grep `naviPointPath` 命中 0）⇒ 当前新核心不使用，属遗留资产。

---

## 7. 只读发现的问题清单（按严重度）

| # | 级别 | 现象（含行号） | 后果 | 建议 |
|---|---|---|---|---|
| 1 | **P0** | twist 乘 50 但主循环声明 100 Hz（`dzactuator.cpp:669,671` vs `:1270`） | 上报速度≈实际 2× ⇒ 任何「停稳/速度」判据失真 | 按实测 dt 计算，或统一到 50 Hz 并核对 `ros2 topic echo /odom` |
| 2 | **P0** | 帧头等待 `guard<200000` × 30 ms 超时（`:897`、`:1196`）＋ 全部收发门控于 `sensor_ok`（`:1287-1335`） | 下位机「静默不宕机」时主循环可阻塞约 100 分钟，期间不发停车帧、不跑看门狗 ⇒ 车保持最后指令 | 有限次短超时 + 显式失联处理（发停车帧/告警/降级） |
| 3 | **P0** | 上行 BCC 校验被注释（`:958-962`） | 错帧直接进里程计与安全判据（含电压、跳变位） | 恢复校验并统计错误率 |
| 4 | P1 | `calibrate_lineSpeed` 只控制日志（`:1317`），但 `robot.yaml:379` 宣称切换换算方式 | 「改了不生效」静默坑；标定流程可信度受损 | 改注释或实现该分支 |
| 5 | P1 | 速度字节截断 + 分辨率 0.0144 m/s（`:78,151,213` → `:403`）；0.15→10（−4.3%） | 低速精度差、系统性偏慢 | 显式四舍五入 + 提高映射分辨率或改双字节 |
| 6 | P1 | `/odom` 航向为轮差积分、无 IMU 融合、无零偏在线估计（`:1296,1301-1304`；`Quaternion_Solution.cpp:211` twoKi=0） | 长距离航向漂移；现由下游打补丁（`serial_reader.py:214-225`、`path_executor.py:1533-1540`） | 底盘层提供可用航向/诚实协方差，或明确降级为「串口+距离源」 |
| 7 | P1 | 里程计协方差与事实不符（`dzactuator.h:75-102`，yaw 方差 1e-3/1e-6） | 将来接 EKF 会过度信任轮式 yaw | 按实测漂移重标 |
| 8 | P2 | `odom→base_link` TF 未发布（`:702` 注释）、`tf_broadcaster_` 白构造（`:1175`） | nav2/rviz 类组件不可用（当前无消费者） | 需要时开启 |
| 9 | P2 | `logicontrol` 无发布者且不受看门狗时间戳保护（`:188-239` vs `:1351-1361`） | 一旦有人接上该话题手动驾驶，1 s 后必被强制停车 | 纳入时间戳 |
| 10 | P2 | 硬编码绝对路径：`bringup.launch.py:20`、`camera.launch.py:16`（`~/dzrobot/config/robot.yaml`）、`bringup_all.launch.py:87`、`start_all.sh:16` | 换目录/换机即失效（本工作区名为 `dzrobot_v2`） | 改为相对/环境变量解析 |
| 11 | P2 | 双协议并存：23 B `0xA5` 帧生效，11 B `0x7B` 帧仅析构用（`:1227-1246`）；头文件宏对下行无效（`dzactuator.h:60-65`） | 阅读与维护误导 | 清理旧宏/旧帧 |
| 12 | P2 | 注释与代码不符：`:1270`（50 vs 100 Hz）、`:394`（长度 19 vs 20）、`:437`（注释 buf[2..18] vs 循环 2..21）、`Quaternion_Solution.cpp:10`（50 Hz vs 积分 1/100）、`robot.yaml:379` | 排障误导 | 修正注释 |
| 13 | P2 | `shoot_planner` 评分缺雷/桥/倒车项，`w_axis` 硬编码（`:319`、`:377-387`） | 射击点选择忽略过桥/吃雷/倒车代价 | 补齐代价项并全部走配置 |
| 14 | P2 | `route_check` 不与现行 `mission_blocks` 比对规划器输出；`bridge_order` 零校验（`:780-807`、`:864-869`） | 「配置未同步」与「桥洞顺序错」查不出来 | 增加一致性/顺序校验 |
| 15 | P2 | `healthcheck.sh:148-151` 过滤条件导致 `det_node` 空扫，仍打印 ✅（`:173`） | 参数未配对的静默失败漏检 | 按源码实际路径（`src/rknn_pt`）匹配 |
| 16 | P2 | `robot.yaml:299` 注释宣称的「obstacle_blocks ↔ map.yaml 一致性」检查并不存在 | 假承诺 | 要么实现，要么改注释 |
| 17 | P3 | 上行 `rx[12][13]`、`rx[16..19]` 被重复赋值两次（`:982`、`:989-990`）；`Publish_Battery_Percentage` 中 `count_B>200` 是死判据（`:788`） | 无害但显示复制粘贴/逻辑冗余 | 清理 |
| 18 | P3 | `dzactuator/params/*.yaml`（amcl/self_amcl/slam_gmapping/mapping/vehicle_client_params/map/robot4_map/group_amcl_sl）只被 `launch_auto/auto_sl_s3e.launch.py` 引用，而该 launch 引用的 `rplidar_s3e_launch.py`(`:51`)、`amcl.launch.py`(`:57`)**均不存在** | 死 launch + 一批遗留参数文件 | 归档/删除 |
| 19 | P3 | `install/` 有 13 个目录而 `build/` 仅 5 个（`det_node`、`fake_amcl_node`、`nav2_common`、`nav2_velocity_smoother`、`nav_2d_msgs`、`openslam_gmapping`、`robot_localization`、`rplidar_ros` 无对应 build） | 与历史「幽灵包导致 `source install/setup.bash` 报错」同源 | 投产前清理一次 install |
| 20 | P3 | `package.xml:26` 依赖写作 `rclcib`（拼写错误，不存在该包） | 依赖声明不可解析 | 删除或改正 |
| 21 | P3 | `car_localization/package.xml:16` 声明 `pyserial`，但 `serial_reader.py` 未 import serial | 冗余依赖 | 清理 |

---

## 8. 面向新版「通用纯追踪 + 多线程任务架构」的复用建议

| 组件 | 结论 | 理由（关键行号） |
|---|---|---|
| 串口收发 + 23 B/34 B 帧格式 + 云台/激光字段 | **直接留用** | 这是硬件契约，重写零收益；字段映射已完整（`dzactuator.cpp:376-449`、`:883-1039`）。改动点是 §7#2/#3 的健壮性，不是协议 |
| `rc_control.py` 的接管/急停四层保护 | **直接留用（思路与实现都对）** | 粘性急停 + 接管看门狗 + 闸门 + 底盘兜底，`rc_control.py:22-30`、`:347-402`。**建议把「接管闸门」从 `path_executor` 内部类（`path_executor.py:568-577`）提升为所有自动发布者共用的中间件**，否则新架构每加一个发布者就会绕过闸门 |
| `serial_reader.py` | **改（不重写）** | 格单位/坐标/重置语义的契约正确且被 path_executor 依赖（`serial_reader.py:227-234`、`path_executor.py:746,1602-1603`）；但它的定位算法是「IMU yaw + 步长死推算 + 到位吸附」的补丁式方案（`:14-23`、`:214-225`）。新架构应换成可观测的航向源（真融合 yaw，或里程计+IMU 的 EKF），并把 `reset` 从「重锚 DR 方向」（`:156-187`）升级为「重锚位姿 + 协方差」。保留 `NODE_VERSION` 标签机制（`:59`）——它让「装的是哪一版」在日志里可见，是很好的实践 |
| `dzactuator` 的里程计与 twist | **重写** | 轮差积分 + 帧增量当固定周期 + ×50 + 协方差失真（`:1289-1311`、`:669-671`、`dzactuator.h:75-102`）。建议：以真实 dt 积分的两轮差速模型 + 可选 IMU yaw 融合 + 诚实协方差；或把底盘节点降级为「串口网桥」，定位独立成包 |
| `dzactuator` 的看门狗/模式仲裁 | **改** | 1 s 指令看门狗是安全刚需，保留（`:1346-1374`）；但 (a) 把 `logicontrol` 纳入时间戳，(b) **与传感失联解耦**（不要放在 `sensor_ok` 分支内），(c) DWA/Pursuit 双源仲裁在「cmd_vel 无发布者」的现实下是纯负担（`:1376-1448`）——新架构若只保留单一速度入口，应删掉仲裁 |
| `path_executor.py` 的纯追踪核心 | **改后留用（最有价值的资产）** | 纯追踪几何 `steer=atan(L·sin α/d)`（`:1776-1809`）；倒车舵角的自行车模型推导与 `REVERSE_STEER_SIGN` 开关（`:1790-1804`）；前瞻点带拐角钳制（`:1657-1731`）；急转三段式 PIVOT；到位吸附 + 漂移预算（`:1533-1540`、`:2319-2322`）。**要改的是「单线程 50 Hz 大循环里塞满状态机」**（`control_loop` `:2005` 起，全文 2622 行）——应拆成「定位 / 任务调度 / 控制」三条线程或三个节点，控制环只读共享状态，射击停站/扫描请求/超时兜底独立成任务队列 |
| 射击链路（`det_node` + `dzrknn_dataprocess` + `dzjudgment`） | **直接留用，仅规范化接口** | 话题契约清晰（§5 表）；`dzjudgment` 必须常驻（`robot.yaml:191-202` 记录了「未启动 → 开火进空话题」的事故） |
| 配置中心 `robot.yaml` + `healthcheck.sh` + `param_spec.py` + `cam_viewer` 调参 | **直接留用（好设计）** | 「全车唯一改参处 + YAML 原生类型 + 启动自检 + 静态不变量 + 运行时比对」（`robot.yaml:1-24`、`healthcheck.sh:55-217`、`param_spec.py:35-201`、`cam_viewer.py:919-959`）。需补：修 §7#15 空扫；把「配置 ↔ 代码默认值」一致性纳入构建期检查 |
| `global_planner.py` + `route_check.py` | **改后留用** | 离线、免 ROS、可测、生成与校验分离 —— 工程实践正确。需改：(a) planner 增加 `--write`（现在只能手抄回 `robot.yaml`，`:881-883`）；(b) route_check 增加「配置 vs 规划器输出一致」与 `bridge_order` 顺序校验（§7#14）；(c) 把 route_check 的检查函数做成**可 import 的库**，让新架构启动时自检（现在只能当脚本跑） |
| `shoot_planner.py` | **改** | 评分只有 4 项、缺雷/桥/倒车、`w_axis` 不可配（`:316-319`、`:377-387`）。若新架构把「射击点选择」纳入任务规划，应补齐代价项并全部走配置 |
| `naviPointPath` / `dzactuator/params/*.yaml` / `launch_auto` | **删除或归档** | 无引用、协议已废（§6.6、§7#18） |
| `install/` 中的 nav2 / robot_localization / slam_gmapping / rplidar 残留 | **清理** | §7#19；与车上出现过的「幽灵包致 `source` 报错」是同一现象 |
| 旧激光导航栈（navigation2 / amcl / slam_gmapping / rplidar / robot_localization） | **不复活** | `bringup_all.launch.py:22-23` 已明确本车无激光雷达、该栈已移出；新架构的工控需求由「格点地图 + 纯追踪」满足 |

---

## 9. 依赖与构建

### 9.1 `package.xml` 摘要（本次读到的）

- **dzactuator**（`ament_cmake`）：`rclcpp`、`sensor_msgs`、`nav_msgs`、`tf`、`tf2`、`tf2_msgs`、`tf2_ros`、`actionlib(_msgs)`、`geometry_msgs`、`message_generation`、`rclpy`、`std_msgs`、`tf2_geometry_msgs`；**`rclcib`（拼写错误，:26）**。`exec_depend`：`joint_state_publisher(_gui)`、`robot_state_publisher`、`rviz`（当前流程均未使用）。CMake 侧另需 `serial`、`Boost(thread atomic system regex)`（`CMakeLists.txt:27,31`），`C++14`（`:11`）。
- **car_localization**（`ament_python`）：`rclpy`、`nav_msgs`、`geometry_msgs`、`tf2_ros`、`pyserial`（冗余）；`console_scripts`：`serial_reader_node`、`path_executor_node`（`setup.py:24,30`；后者是 2026-10-03 补的调试入口，`start_all.sh` 仍走源码直跑）。
- **naviPointPath**（`ament_cmake`）：纯数据，无依赖。
- **未读**：`dzrknn_dataprocess`、`dzjudgment`、`rknn_pt` 的 `package.xml`（超出本次指定范围）。

### 9.2 构建方式与耗时线索

- 方式：colcon（4 个 C++ 包 + 1 个数据包 + 1 个 ament_python 包）。
- 实测耗时（来自本工作区 `log/build_*/events.log` 的 `JobEnded` 时间戳）：
  - 冷构建 4 个 C++ 包 + `naviPointPath`：**墙钟 ≈ 41.0 s**（并行）；其中 `dzactuator` 单独 **35.7 s**、`dzrknn_dataprocess` **30.8 s**、`dzjudgment` 4.5 s、`naviPointPath` 3.0 s。
  - `car_localization`（ament_python）：**≈ 1.8 s**（`log/build_2026-10-03_10-37-30/events.log`，该次因 `install/.../resource_index` 缺失以 `rc=1` 结束 —— 环境问题，非代码问题）。
- 结论：**C++ 单包迭代 30~40 s**（大头是 RKNN/OpenCV 头文件），Python 包约 2 s；`launch/` 与 `config/` 改动**无需编译**（bringup 走 params-file，`bringup.launch.py:53-61`）。
- 构建残留：`build/` 5 包 vs `install/` 13 包（§7#19）。

---

## 10. 未确认项（未当结论使用）

1. `/odom`、`/imu_data` 等话题的**实测发布频率**（需 SSH 或 `ros2 topic hz`）。
2. 主循环实际频率：声明 100 Hz，但受下位机帧率约束，真实值未测。
3. 上行 34 字节帧「长度字段 0x1F vs 实际 32 字节载荷」的约定（需对照 STM32 工程 `src/history/附加资源/DZVCU程序/`）。
4. `dzactuator.h:64-65` 的 `RECEIVE_DATA_SIZE 34 / SEND_DATA_SIZE 11` 与生效协议（23 B 下行）的关系是否有其他消费者。
5. 本工作区副本与车上 `~/dzrobot` 的**文件级差异**（未 SSH）。
6. 车上是否存在与本工作区不同的 `mission_blocks`（本工作区为 28 个航点，`robot.yaml:329-358`）。
7. `route_check.py` 内置 `fallback_validate`（`:174-214`）与车上权威 `path_executor.validate_mission_blocks`（`path_executor.py:342`）的逐条等价程度。
8. `config/map.yaml:9-11` 的 `barrier_col_from/barrier_row_hi/lo` 未被两个工具读取（均在代码里硬编码）—— 是有意还是疏漏。
9. `route_check` 射击距离容差 `+0.06`（上限放宽到 0.51 m）会放对角射击点（0.495 m）通过（`:578-582`、`:615-618`）—— 有意容差还是疏漏。
10. `healthcheck.sh` 在旧核心树/归档树上的具体误报集合。
