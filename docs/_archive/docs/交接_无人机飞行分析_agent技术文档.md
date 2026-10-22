# 无人机飞行数据分析 · Agent 技术交接文档

> 更新：2026-10-18 · 由前任 lead 会话整理。目标读者：接手的 AI agent。
> 配套人类侧文档：docs/交接_无人机飞行分析_人类操作手册.md（操作口径以它为准，本文不重复人类侧内容）。

## 0. TL;DR 与职责边界

**目标**：人类手动飞 PX4 无人机；agent 录 rosbag（外环数据）+ 收集飞控 SD 卡 ULog（内环数据），做"摇杆→指令→电机"闭环分析，产出**飞行指纹卡**，一轮一参数迭代，把飞机调稳。

**Agent 职责白名单**：SSH 只读体检；起/停 rosbag 录制（写入仅限机上 `~/drone_bags/`）；经用户同意后拉起感知栈；读取/分析 ULog 与 bag；产出报告与参数建议。

**禁区**：解锁/起飞/发控制指令；改 PX4 参数；重启飞控；未经同意杀/起机上进程；代跑有执行能力的节点（px4_control / teleop —— 那类决定必须由人在座做出）。

## 1. 环境与拓扑事实

| 项 | 值 | 备注 |
|---|---|---|
| 飞控 | PX4（具体版本未确认，ULog 里有） | 串口连 Orin |
| 机载电脑 | Jetson Orin Nano，Ubuntu 22.04.5，6 核，RAM 7.6G（可用约 2~3G），eMMC 空闲 ~190G | 主机名 **dzsmart**（mDNS） |
| SSH | `duzhong@dzsmart.local`，**免密已配**（Windows 公钥已装入） | 历史 IP：10.240.174.104 / 10.35.165.104（网络变过，优先 hostname，ARP 兜底） |
| VNC 密码 | duzhong | 备用远程桌面入口，一般用不到 |
| 激光雷达 | Livox MID-360：雷达 192.168.3.105，Orin 网口 eno1=192.168.3.50 | 点云 10Hz/19968 点，IMU 200Hz |
| ROS | Humble，**domain 0**，FastDDS 默认 | 非交互 SSH 需手动 source |
| 机上工作区 | `~/dzpx4`（含 install/setup.bash） | 包：livox_ros_driver2、fast_lio_localization、px4_odom_convert、px4_imu_bridge、px4_control、px4_path |
| 飞控串口 | **ttyACM1 是 XRCE 活口**（0x7E 帧）；ttyACM0 静默 | `ls /dev/ttyACM*` + 读原始字节鉴别 |
| 场地 | 12×14 m 房间 = 赛场，内含 4×5 m 比赛垫 | 场地帧标定未做（做自动任务前需要） |

## 2. 软件链路与数据流

```
Livox MID-360 ─10Hz→ livox_ros_driver2 ─→ FAST-LIO ─/Odometry ~45Hz─→ px4_odom_convert 【mode:=odom 必须】
     ─~43.8Hz→ /fmu/in/vehicle_visual_odometry ─→ PX4 EKF2（位置/速度/偏航均接受）

飞控 USB(ttyACM1, 921600) ─→ MicroXRCEAgent serial --dev /dev/ttyACM1 -b 921600 ─→ DDS /fmu/out/*(8) + /fmu/in/*(13)

飞控内部 logger ─→ SD 卡 .ulog（黑匣子：内环全量）
```

**EV 已验证结论（2026-10-17）**：PX4 接受并融合 EV——eph 0.032 m / epv 0.029 m、`heading_good_for_control=true`、`dead_reckoning=false`、timesync RTT ≈ 3 ms。EV 不含速度字段（NaN），PX4 位置差分自算，低速悬停无碍，高速机动再议。

## 3. DDS 数据面（2026-10-18 实测，重启后探得）

### /fmu/out（8 路，有数据）
| 话题 | 实测频率 | 内容 |
|---|---|---|
| vehicle_local_position | **102 Hz** | x/y/z + v + eph/epv + reset_counter |
| vehicle_attitude | **133 Hz** | 四元数 |
| sensor_combined | **142 Hz** | 陀螺 + 加速度积分 |
| vehicle_odometry | 有 | EKF 融合位姿 |
| vehicle_status / vehicle_control_mode / failsafe_flags / position_setpoint_triplet / timesync_status | 低频 | 状态语境/故障体检 |

### /fmu/in（13 通道，飞控在等数据）
vehicle_visual_odometry、trajectory_setpoint、offboard_control_mode、vehicle_command、vehicle_rates_setpoint、vehicle_attitude_setpoint、obstacle_distance、onboard_computer_status、sensor_optical_flow、telemetry_status、vehicle_mocap_odometry、trajectory_bezier、trajectory_waypoint。
→ **offboard 注入入口全开**（含角速率直通内环，可做激励注入实验）。

### DDS 里没有的（固件编译期定死）→ 只在 ULog
rc_channels、manual_control_setpoint、vehicle_rates_setpoint(out)、vehicle_angular_velocity、actuator_outputs、actuator_controls、battery_status、estimator_innovations、vehicle_imu(高率)、esc_status、gps、land_detected。

## 4. 关键坑清单（现象/根因/对策/状态）

| # | 坑 | 对策 | 状态 |
|---|---|---|---|
| 1 | `px4_odom_convert` launch 默认 `mode:=localization`，订阅已死话题（transform_fusion.py ImportError → /localization 不存在），**EV 永不出且表面一切正常** | 必须 `mode:=odom`（数值上与 localization 模式逐位相同） | ✅ 已验证，启动手册已写 |
| 2 | 串口二义性：看门狗曾写死 ttyACM0（静默口），每 15s 静默重启死循环 | Agent 必须 `--dev /dev/ttyACM1`；杀旧看门狗再手动起 | ⚠️ 10-18 晨重启后 agent 已以 `--dev ttyACM1` 出现，疑似已修，**待读脚本确认** |
| 3 | 机上 start_drone.sh 曾写死 ttyACM0 且第 4 步缺 mode:=odom | 同上 | ⚠️ 待确认 |
| 4 | 节点 duplicate-instance 保护：旧实例不退，新实例 FATAL | `pkill -TERM` 后确认干净再启 | ✅ 已验证 |
| 5 | 无人机多次重启（10-17 21:10、10-18 08:17，原因未确认），**感知栈不自启**，/tmp 清空 | 每次重启按 §6 恢复 | ⚠️ 重启原因待问用户 |
| 6 | 非交互 SSH 无 ROS 环境 | 每条命令前 `source /opt/ros/humble/setup.bash; source ~/dzpx4/install/setup.bash`，或包装成脚本上传执行 | ✅ |
| 7 | PowerShell 发 SSH 时双引号被吃 | grep 用无引号单词模式；复杂命令写成 .sh 传上去跑 | ✅ |
| 8 | /tmp 易失 + RAM 仅 2~3G | 录包写 `~/drone_bags`（eMMC），不写 /tmp | ✅ |
| 9 | local_position 的 reset_counter 频繁（观测 31 次） | 分析按重置点分段，防把跳变算成漂移 | ✅ |
| 10 | 未 source px4_msgs 时 `ros2 topic hz` 报"消息类型无效" | source ~/dzpx4/install/setup.bash | ✅ |
| 11 | EV 无速度字段 | 低速无碍；高速机动时考虑补 | 留观 |
| 12 | 起飞前 failsafe：manual_control_signal_lost=true（无 RC）、home/global invalid（无 GPS 属预期） | 人类用真 RC，起飞前核对 failsafe_flags 全清（local 系全 false） | ✅ |

## 5. 接手体检（每次先跑这个）

从 Windows 侧（pwsh）建议把下面存成 .sh 传上机执行，避免引号问题：

```bash
uptime; ls -l /dev/ttyACM* 2>/dev/null
pgrep -af MicroXRCEAgent || echo AGENT_DOWN
pgrep -af fast_lio; pgrep -af px4_odom_convert; pgrep -af livox || echo STACK_DOWN
df -h ~ | tail -1; free -g | head -2
```

ROS 侧（每条命令先 source 两行）：

```bash
source /opt/ros/humble/setup.bash; source ~/dzpx4/install/setup.bash
ros2 topic list | grep fmu          # 有 /fmu/out/* ⇒ uXRCE 通
ros2 topic hz /fmu/out/vehicle_attitude --window 50
ros2 topic echo /fmu/out/vehicle_local_position --once   # 看 xy_valid / reset_counter / eph
ros2 topic echo /fmu/out/failsafe_flags --once
```

判读：agent 在 ttyACM1 + /fmu/out 有数据 ⇒ 链路 OK；STACK_DOWN ⇒ EV 没喂（§6 恢复）；failsafe 里 local_*_invalid 应为 false。

## 6. 启动/恢复手册（重启后按序执行）

细节见 [dzsmartscout/fc-audit/启动手册.md](../dzsmartscout/fc-audit/启动手册.md)。核心命令：

```bash
# 每个终端先：
source /opt/ros/humble/setup.bash; source ~/dzpx4/install/setup.bash

# A 感知层
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch fast_lio_localization velodyne_localization.launch.py rviz:=false

# B 飞控链路（注意 --dev ttyACM1 和 mode:=odom）
MicroXRCEAgent serial --dev /dev/ttyACM1 -b 921600
ros2 launch px4_odom_convert px4_odom_convert.launch.py mode:=odom
ros2 launch px4_imu_bridge sensor_combined_to_imu.launch.py

# C 验证：ros2 topic list | grep fmu；hz /fmu/in/vehicle_visual_odometry 应 ~43Hz
```

注意：①先确认旧 agent/看门狗不在跑（抢串口）；②px4_odom_convert 有 duplicate-instance 保护；③**px4_control / teleop 属"有执行能力"节点，需人在座、真 TTY，agent 不代跑**。

## 7. 数据采集规范

### 7.1 rosbag（外环，agent 直接负责）
```bash
mkdir -p ~/drone_bags && cd ~/drone_bags
ros2 bag record -o <YYYYMMDD_HHMM>_<科目> \
  /fmu/out/vehicle_local_position /fmu/out/vehicle_attitude /fmu/out/sensor_combined \
  /fmu/out/vehicle_odometry /fmu/out/vehicle_status /fmu/out/vehicle_control_mode \
  /fmu/out/failsafe_flags /fmu/out/timesync_status \
  /Odometry /fmu/in/vehicle_visual_odometry --storage sqlite3
```
- 大小约 **5 MB/分钟**；可选加 /livox/lidar（+180 MB/min，默认不加）；
- 一次飞行会话录一个包（起飞前开、落地后 SIGINT 停），段内切分靠 arming_state/control_mode/reset_counter；
- setsid 后台跑，记 PID，用 `kill -INT` 停（别 SIGKILL，会损坏 sqlite）；
- **端到端实战尚未跑过一次完整飞行**——工具本身验证过，接手后第一次飞行务必走通全流程。

### 7.2 ULog（内环，人类拔卡提供）
- 位置：飞控 SD 卡，PX4 默认 SDLOG_MODE=1（解锁后记录）大概率已开；目录 /log/*.ulog；
- 获取：无 MAVLink 链路 ⇒ QGC 下载不可用 ⇒ 物理拔卡读卡器 → Windows 指定目录（会话里建"飞行日志"文件夹）；
- 分析：Windows 侧 `py -m pip install --user pyulog`（**尚未安装**），或机上 python3；
- 重点 topics：rc_channels、manual_control_setpoint、actuator_controls、vehicle_attitude_setpoint、vehicle_rates_setpoint、vehicle_angular_velocity、actuator_outputs、battery_status、estimator_innovations、vehicle_imu。

## 8. 分析方法论与指纹卡

### 8.1 分段规则（先于一切计算）
按 arming_state 只取解锁段 → 按 control_mode 贴语境标签 → 位置按 reset_counter 切段 → timesync 偏移对齐 FC/Orin 时钟 → 剔除起降过渡段。

### 8.2 四层推导
| 层 | 数据 | 指标（定义） |
|---|---|---|
| L1 悬停基线 | bag | 陀螺 std + Welch PSD 振动峰（对齐转频=桨/机架问题）；四元数→欧拉的 roll/pitch 静态偏移（重心）；xy/z 漂移速率=线性拟合斜率（m/min）+方向；vel std；eph 中位数 vs 实际漂移；yaw 漂移率（°/min） |
| L2 动态响应 | bag（部分）+ ULog（全） | 阶跃四件套：上升时间/超调%/回调次数/稳态误差（ULog 有杆量边沿）；**自由衰减法（bag-only 也能做）**：停杆后衰减振荡，对数衰减率 δ=ln(x1/x2)，阻尼比 ζ=δ/√(4π²+δ²)，ωn 由峰间距反推——ζ 低 ω 高 = 速率环 P 偏高指纹；互相关 argmax 测杆→姿态总延迟 |
| L3 电机动力 | ULog only | 悬停油门%（>55% 偏重）；四通道 hover 均值 max-min（对称性）；饱和时间占比；起飞段 output↔az 拟合推力曲线；电压 sag |
| L4 跨飞行 | 台账 | 每次飞行一行指纹；一轮一参数；同科目对比建因果 |

### 8.3 指纹卡字段（bag 侧可先行）
hover_s、drift_xy_rate_m_min(+方向)、drift_z_rate、pos_std_xy_cm、pos_std_z_cm、vel_std_xy、gyro_std_xyz、vib_peak_hz、roll/pitch_bias_deg、yaw_drift_deg_min、eph_median、ev_rate_mean、ev_gap_max_s、reset_events；（ULog 到位后补）hover_throttle_pct、motor_symmetry_pct、saturation_pct、battery sag、各轴阶跃四件套、自由衰减 ζ/ω。

### 8.4 症状→参数对照（以机载固件实际参数表为准，版本间命名有差异）
| 症状 | 参数 |
|---|---|
| 横滚/俯仰反应慢或振荡 | MC_ROLLRATE_P/I/D、MC_PITCHRATE_P/I/D、MC_ROLL_P、MC_PITCH_P |
| 停杆弹跳、抖 | 速率环 D、IMU_GYRO_CUTOFF |
| 速度猛/肉 | MPC_XY_VEL_P、MPC_ACC_HOR、MPC_XY_VEL_MAX |
| 高度忽上忽下 | MPC_Z_VEL_P/I、MPC_THR_HOVER |
| yaw 走偏 | MC_YAWRATE_P/I、罗盘 |
| 手感延迟大 | 滤波截止频率、MPC_TILTMAX_AIR |

## 9. 飞行科目协议

| 科目 | 动作 | 数据目的 |
|---|---|---|
| F0 | 悬停 60s，手离杆 | 基线：漂移/振动/重心/油门 |
| F1 | 前后左右单杆满杆阶跃 ×3，间隔 3s+ | 阶跃响应（自稳模式最好） |
| F2 | 半杆连续机动 30s | 跟随性、振荡激励 |
| F3 | 起降 ×3 | 推力曲线、定高 |

人类落地后只报"飞了 F 几"；精确分段由 arming_state 时间戳完成。模式（自稳/定点）必须记录进台账。

## 10. 人机协作协议

- **沟通模板**：起飞前人类"准备飞了"→ agent 体检+开录→"录上了，可以飞"；落地后人类"飞完了，F 几"→ agent 停录、验 bag 完整性、先出 bag 侧指纹卡初版；拔卡后人类"卡拷好了"→ agent 出全量报告；
- **报告口径**：给人类看的一律自然语言（参考 wx_tools/REPORT_STYLE.md 的风格守则），参数建议 ≤1~2 个/轮；
- **安全红线**：见 §0 禁区。人类感觉不对先落地，数据永远第二。

## 11. 未决问题

1. 无人机两次重启原因未确认（10-17 21:10、10-18 08:17）——问人类是否人为断电；
2. 看门狗 / start_drone.sh 的 ACM1+mode:=odom 修复状态待读机上脚本确认；
3. pyulog 未安装；ULog→分析链路未实战过；
4. 场地帧标定未做（做自动任务前需要：原点/轴向对 35cm 格网/垫子四角）；
5. RC 满足 failsafe 的实机确认（插 RC 后 manual_control_signal_lost 应转 false）；
6. 首次完整飞行的 rosbag 端到端验证。

## 12. 产物与文件索引（均已核实存在）

| 文件 | 说明 |
|---|---|
| [dzsmartscout/fc-audit/启动手册.md](../dzsmartscout/fc-audit/启动手册.md) | 启动/恢复命令权威版（含阶段 A~D 与 teleop 按键表） |
| [_tmp_session/drone_ev/EV验证_20261017.md](_tmp_session/drone_ev/EV验证_20261017.md) | EV 链路验证全记录（43.9Hz、EKF 接受证据、4 问题清单） |
| dzsmartscout/tools/drone/check_lidar.sh、check_imu.sh、check_lidar_rate.sh | 雷达/IMU 一键复检 |
| dzsmartscout/tools/link/ | 空地协同信标（与本项目弱相关） |
| drone_recordings/ | 此前相机录制产物（非 bag） |
| docs/交接_无人机飞行分析_人类操作手册.md | 本文档的人类侧配套 |

## 13. 接手检查清单（30 分钟）

- [ ] §5 体检跑通，确认 agent 在 ttyACM1、/fmu/out 8 路有数据
- [ ] 判读 failsafe_flags 当前值并记录
- [ ] 若 STACK_DOWN → 经用户同意按 §6 拉起，验证 EV ~43Hz 且 eph<0.1
- [ ] 建立 Windows 侧"飞行日志"目录并告知人类
- [ ] 机上建 ~/drone_bags 并空跑一次 1 分钟录包验证（§7.1）
- [ ] py -m pip install --user pyulog（Windows 侧）
- [ ] 向人类复述：科目 F0~F3 怎么飞、拔卡流程、一轮一参数铁律
- [ ] 问清两件未决事：重启原因、看门狗/启动脚本是否已修
