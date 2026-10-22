---
来源: dzsmartscout/docs/drone/uXRCE恢复手册.md, dzsmartscout/docs/drone/域号切换单_118.md, dzsmartscout/docs/drone/无人机架构总览.md, docs/交接_无人机飞行分析_agent技术文档.md, dzsmartscout/tools/reports/t4-链路验证报告.md
取证: 实/档
更新: 2026-10-21
取代: 无（原文仍为现场一手材料，本册为提炼 + 状态更新）
status: 现行
---

# 飞控与uXRCE域号

## TL;DR（给 agent 看的）

- 飞控这条链有**两个独立故障**，别混：①uXRCE 会话死（10-17 起每 61 s 被看门狗重启、Publisher=0）②域号分裂（机 0 / 车 118，静默失联）。
- ①的解法是**物理动作**：飞控断电 5 s 再上电。②要改 PX4 参数 = 需 MAVLink 通道 = **当前不存在**，所以切换单写了但没执行。
- **10-18 起 ① 已缓解**：8 路 /fmu/out 有数据（102/133/142 Hz，agent 走 ttyACM1），但会话脆弱，Orin 单独重启会复发。
- 最大坑：ros2 topic list 里出现 fmu 话题**不是判据**（看门狗订阅也会让它存在）；唯一硬指标是 Publisher count ≥ 1。

## 1. 飞控硬件与串口

| 项 | 值 | 证据 |
|---|---|---|
| 飞控板 | Matek / HKUST_UAV_NxtPX4（USB 1b8c:0036） | dzsmartscout/docs/drone/无人机架构总览.md:15 |
| 桥 | uXRCE-DDS：MicroXRCEAgent serial --dev /dev/ttyACM<?> -b **921600** ⇄ PX4 uxrce_dds_client | dzsmartscout/docs/drone/uXRCE恢复手册.md:31 |
| 固件版本 | **未知**（只能从 ULog 读） | docs/交接_无人机飞行分析_agent技术文档.md:18 |
| 地面站 | **无 MAVLink 通道** ⇒ QGC 不可用、PX4 参数不可读不可改 | uXRCE恢复手册:247、:269 |

**串口归属有 A/B 两版一手记录，必须上机现判**：

| 版本 | 说法 | 出处 |
|---|---|---|
| A | ttyACM0 = NxtPX4（udevadm 属性），ttyACM1 = CH340（1a86:55d3） | dzsmartscout/docs/drone/无人机架构总览.md:15（10-17 实测） |
| B | **ttyACM1 才是 XRCE 活口**（读到 0x7E 帧），ttyACM0 静默 | docs/交接_无人机飞行分析_agent技术文档.md:25、§4 坑 2（10-18） |

判定：lsusb 看 VID/PID + fuser -v /dev/ttyACM* 看谁在占。**别背端口号**——看门狗曾写死 ACM0（静默口），每 15 s 静默重启死循环（交接:64）。

## 2. uXRCE 会话死（P0-1）

| 项 | 内容 |
|---|---|
| 症状 | agent 每 ~61 s 被看门狗重启；日志停在 TermiosAgentLinux init running；/fmu/out/vehicle_local_position **Publisher count = 0** |
| 周期成因 | 61 s = 60 s 冷却 + 1 s 杀/起（uXRCE恢复手册:169 实测） |
| 根因（指向） | **Orin 重启而飞控未断电** ⇒ 串口会话状态不一致 |
| 恢复 | 飞控**断电 5 s** 再上电 → 等 30 s → 重起 agent（三步卡：uXRCE恢复手册:16-80） |
| 10-18 状态 | **已缓解**：8 路 /fmu/out 有数据（vehicle_local_position 102 Hz / attitude 133 Hz / sensor_combined 142 Hz），agent 以 --dev ttyACM1 出现（交接:43-50、:64） |
| 仍脆弱 | Orin 再单独重启即复发；FC_OFFLINE 自动恢复补丁**已写未部署**（uXRCE恢复手册:173） |

**恢复后只读验证三件（缺一不算通）**：

| # | 命令 | 通过标准 |
|---|---|---|
| 1 | ros2 topic info -v /fmu/out/vehicle_local_position | Publisher count ≥ 1 |
| 2 | ros2 topic hz /fmu/out/vehicle_attitude | 稳定 ~133 Hz（不只是「有数」） |
| 3 | ps -o etimes= -p $(pgrep -f MicroXRCEAgent) | 启动时间 > 61 s 且持续增长（不在重启循环） |

## 3. 域号分裂（P0-2）

| 侧 | 值 | 证据 |
|---|---|---|
| 机侧全部（节点/agent/看门狗） | **0** | grep ~/.bashrc 无 DOMAIN 命中；~/.bash_history 中 ROS_DOMAIN_ID 出现 0 次（架构总览:370-380） |
| 机侧唯一 118 | ~/dzlink_tools/dzlink.env（被 dzlink_supervisor.py:157-209 在 import 阶段读取并覆盖） | 同上 |
| 车侧 | 118（dzros.env / start_all.sh:123） | dzsmartscout/docs/空地路线下发契约_v2.md:44-46；t4:70 |
| PX4 UXRCE_DDS_DOM_ID | **未知**（默认 0，无 MAVLink 读不到） | dzsmartscout/docs/drone/无人机架构总览.md:400 |

**症状**：ping/SSH 全通、ROS 零报错、彼此看不见。机制：域号决定 DDS 端口（7400+250×domain ⇒ 118 = **36900**，组播 239.255.0.1），跨域连发现包都收不到（dzsmartscout/tools/reports/t4-链路验证报告.md:86-93）。

**为什么没切**：切换单要求 PX4 侧设 UXRCE_DDS_DOM_ID=118，而改 PX4 参数必须有 MAVLink/QGC ⇒ 该通道不可用 ⇒ **前置闸不满足**（dzsmartscout/docs/drone/域号切换单_118.md:39-40、:213）。单侧切换比现状更糟：车在 118 等，机被切到 118 而 PX4 仍在 0 ⇒ 飞控侧直接断。

**切换单写了什么（四步，未执行）**：①PX4 设 UXRCE_DDS_DOM_ID=118 ②Orin 三处 export（.bashrc / start_drone.sh / orin_agent_watchdog.py）③重启全栈 ④两端 ROS_DOMAIN_ID 校验 + 双向 topic 对比。回滚 = 全部改回 0 + 重启。**切换窗口内禁止飞行**（切换单:214）。

## 4. 为什么两个 P0 都卡在「现场动作」

| 障碍 | 后果 |
|---|---|
| 无 MAVLink 通道 | 改不了 PX4 参数、QGC 不能救场、只能物理拔 SD 卡读 ULog |
| 部署三件套不在仓库（start_drone.sh / autostart_agup.sh / orin_agent_watchdog.py） | 域号切换与看门狗修复本地**改不了也 review 不了**，只能上机手改 |
| 意图边界：agent 对机只读 | 断电重启飞控、改配置都必须人在场 |

## 5. 安全红线（照抄执行）

- **禁止 cat /dev/ttyACM0**：会把字节从 agent 手里抢走、**自己制造**会话死（uXRCE恢复手册:313-315）。
- **禁止双 agent**：同一串口两个 MicroXRCEAgent = 互抢帧。
- 禁止代跑 px4_control / px4_teleop（有执行能力，须人在座、真 TTY）。
- 域号切换窗口内禁止飞行。
- 判活别看日志：stdout 块缓冲，日志 0 字节 ≠ 进程死（R-25）。

## 6. 与「能不能飞」直接相关的未修隐患

| 项 | 影响 | 出处 |
|---|---|---|
| P1-1 目标只查 11 个 isfinite，无 z 上下限/水平半径/单帧位移 | 任何能发 /px4_control/goto 的进程可把飞机指到界外或地下 | px4_control_node.cpp:300-315 |
| z 换源在 jump guard 之前 | armed 下 EKF z 阶跃 > 0.5 m ⇒ 永久 hold，x/y 外部视觉同时冻结（**代码推演，未实测**） | px4_odom_convert.cpp:597-599 vs :750-756、:806 |
| R-05 goto_enabled 是 transient_local 锁存 | 跟随节点晚启动也会立刻收到 true 并直接开跟，无二次人工确认（补丁 03 未应用） | path_follow_trajectory_node.cpp:393-403、:430-448 |
| R-07 前馈语义相反 | waypoint 发 v/a/yawspeed=0，控制节点把 0 当真前馈；自主降落注释「不碰速度」却发了 {NaN,NaN,descent_vz} | path_follow_waypoint_node.cpp:184-191；px4_control_node.cpp:505-509、:514-522 |
| R-21 Goto 静默丢弃 | 未使能/非 HOVER 时 on_goto 直接 return 且无日志 | px4_control_node.cpp:293-299 |
| R-12 幽灵参数 | launch 传 max_position_jump_m 但节点从未声明；真阈值硬编码在 hpp | launch/px4_odom_convert.launch.py:18-19；include/px4_odom_convert.hpp:148-150 |

## 相关文件

- [01_架构总览.md](01_架构总览.md)、[03_感知与视觉侦察.md](03_感知与视觉侦察.md)、[05_现场操作与脚本.md](05_现场操作与脚本.md)、[06_已取证与未验证.md](06_已取证与未验证.md)
- dzsmartscout/docs/drone/uXRCE恢复手册.md（三步卡 + 61 s 时间线 + 判活命令）
- dzsmartscout/docs/drone/域号切换单_118.md（四步 + 回滚 + 前置闸）
- dzsmartscout/docs/drone/无人机架构总览.md §3（域号审计原始 grep 输出）
- dzsmartscout/tools/reports/t4-链路验证报告.md（域 118 端口/组播推导、8 格 QoS 矩阵）
- docs/交接_无人机飞行分析_agent技术文档.md §1、§4（10-18 串口与链路恢复一手记录）
