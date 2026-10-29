---
来源: dzrobot_v2/src/dzbrain/dzbrain/main.py, dzrobot_v2/src/dzbrain/dzbrain/blackboard.py, dzrobot_v2/src/dzbrain/dzbrain/io/contract.py, dzrobot_v2/src/dzbrain/dzbrain/core/config.py, dzrobot_v2/src/dzbrain/dzbrain/tasks/safety.py
取证: 实
更新: 2026-10-29
status: 现行
---
# dzbrain 零基础深挖报告

> **读者**：刚接触项目、不懂 ROS / 不懂线程 / 没写过车控的入门者。
> **真源**：本报告所有行号都指向 `dzrobot_v2/src/dzbrain/dzbrain/` 下的源码，你可以逐个去翻。
> **范围**：只讲 dzbrain 这一个包（车上的"大脑"）。不涉及底盘驱动、云台固件、无人机本体。

---

## 0. 一句话定位：dzbrain 是"想"的那一半

一辆比赛小车，物理上分成两半：

| 半 | 干什么 | 谁负责 | 你改不改它 |
|---|---|---|---|
| **执行** | 电机怎么转、舵打多少度、激光打不打 | `dzactuator`、`dzjudgment`、`dzrknn_dataprocess` 等 C++ 节点 | 一般不碰 |
| **决策** | 我在哪、要去哪、走哪条线、现在该走多远、要不要打靶、有危险就刹停 | **`dzbrain`**（Python 包） | 今天只看它 |

`dzbrain` **不直接控制电机**。它只算出"下一步该干什么"，然后通过 ROS 话题（比如 `pursuitAngle`）告诉底盘节点；底盘节点再告诉电机。这条"决策→执行"的分工，是所有比赛车代码的第一原则。

包的位置：`dzrobot_v2/src/dzbrain/dzbrain/`。**注意这里有两个 dzbrain**：
- 外层 `dzrobot_v2/src/dzbrain/` —— ROS 2 的包壳（`package.xml`、`setup.py`，装包用的）；
- 内层 `.../dzbrain/dzbrain/` —— **真正的代码**，下面所有行号都指这一层。

---

## 1. 它有多大（先有个体量概念）

实际统计（`wc -l`，含注释）：

| 子包 | 行数 | 一句人话 |
|---|---|---|
| `route/planner.py` | **5050** | 路线规划器，全场最大 |
| `tasks/mission.py` | **4268** | 段执行与步进循环 |
| `main.py` | **3306** | 装配层：把 8 个线程组装起来跑 |
| `tasks/drone.py` | 1486 | 无人机情报接收/校验 |
| `tasks/localization.py` | 1490 | 定位 + 航向守卫 |
| `tasks/aim.py` | 1372 | 云台自瞄 |
| `tasks/tracker.py` | 911 | 循迹跟踪 |
| `tasks/safety.py` | 529 | 安全兜底 |
| `core/config.py` | 945 | 配置中心（SCHEMA） |
| 其余（io/core/pursuit/scout/tools） | 约 5000 | 通讯、几何、仿真 |
| **合计** | **约 22,500** | 注意：这**不含**测试（`tests/` 另有一大批） |

> 数字和注释各占多少？**注释占了大约 2/3**。这是好事——它把每一条设计决定的理由都写在了代码里。你今天要读的"说明书"，其实就在代码注释里，我这份报告只是把它串起来。

---

## 2. 八个子包，各管一摊

`main.py` 里 `build_tasks()` 只有一句总纲（`main.py:2511`）：
```python
group = TaskGroup(bus, bb, logger=log)
```
然后一行一个 `add(...)`，把 8 个任务线程装进一个 `TaskGroup`（`main.py:2531-2540`）。

```dsh-ui
{"title":"八个子包分工","items":[{"type":"table","columns":["子包/文件","干什么","人话类比"],"rows":[["main.py","把 8 个线程组装起来、跑步进循环、处理退出信号","总指挥：只发号施令，不自己动手"],["core/","黑板、事件总线、任务线程基类、配置中心、几何、日志","公共地基：谁都要用的工具箱"],["io/","话题表 + QoS 契约、ROS 桥、裁判帧解析","外交官：跟外面世界打交道"],["tasks/","8 个任务线程（定位/安全/循迹/任务/云台/误差/无人机/遥测）","8 个专员：各盯一件活"],["pursuit/","循迹控制（纯追踪、前视、路径、速度规划）","司机：怎么把车沿曲线开"],["route/","地图模型、解析、物理净空、规划器（两个引擎）","导航员：先想好走哪条线"],["scout/","无人机侧场地布局解析（视野、帧、心跳、参考）","侦察员助手：整理无人机给的图"],["tools/","离线仿真 sim.py / sim_cli.py","训练场：不上车先练"]]}]}
```

---

## 3. 一次启动，发生了什么（`main.py` 主流程）

`main.py:3223` 是入口 `def main(...)`。走法（`main.py:1-68` 的模块注释原文可查）：

1. **解析命令行参数**：`--dry-run`（只做门禁）、`--bench --seconds N`（无 ROS 的联调）、`--map <图>`（仅开发入口）、`--domain` 等。
2. **启动门禁**（`--dry-run` 或正常启动前都跑）：
   - 配置校验（`Config.validate()`，`core/config.py`）
   - 路线校验（`route/validate.py`）
   - **不过就拒绝启动**，不静默回退。这是硬规则：旧代码"配置错了照样跑"是最大的坑来源（`main.py:8`）。
3. **建 ROS 桥**（`io/ros_io.py`）：一个进程、一个 ROS 节点、一个 `MultiThreadedExecutor`。**ROS 回调只在线程池里跑，回调只做"解码+缓冲"，不做决策**（`main.py:5-6`）。
4. **打印启动横幅**（`start_banner()`，`main.py:2544`）：配置文件、地图来源、路线来源、巡航速度、自动出发、开火策略、**每个任务线程的频率**。这行横幅是你排查的第一步——它把当前生效的配置原样打印出来。
5. **`build_tasks()` 装 8 个线程**（`main.py:2508`）。
6. **进入主循环**：要么"规划一次"（旧路径），要么"步进循环 `StepLoop`"（新路径，见 §9）。
7. **退出必刹车**：无论正常退出、异常、Ctrl-C，先发一条"速度=0"，再收摊（`main.py:9`）。

### 3.1 启动横幅长什么样（`main.py:2572-2582`）

```
dzbrain 启动 · 多线程任务架构
  配置文件   : ...
  地图       : 无人机 field_layout（车端不留地图文件）
  路线来源   : 步进循环（无人机布局 → 段规划）
  路线       : 运行期等布局（version 大者胜）→ 每段从当前格规划到 target_cell → ...
  巡航速度   : 0.35 m/s
  自动出发   : true（延时 3.0s）
  越界余量   : 0.215 m
  开火策略   : ...（提前量 ...s）
  任务线程   : Localization@...Hz Safety@...Hz ...
```

---

## 4. 核心机制：一个进程、八个线程、一块黑板、一个总线

这是整个 dzbrain 的心脏。四个词解释清楚：

### 4.1 一个进程，不新开进程
整个 dzbrain 就是**一个 Python 进程**。好处：不额外占资源、好管理、日志集中。坏处（也就是为什么需要多线程）：单进程里不同任务会互相等待，必须靠线程隔离。

### 4.2 一个线程一个任务：`core/task.py` 的 `TaskThread`
`TaskThread` 是线程基类（`core/task.py:59`）。每个任务继承它，只要实现三个方法：
- `setup()`：启动前调用一次，订阅话题、建发布者（`task.py:90`）；
- `tick()`：**单拍逻辑，子类必须实现**（`task.py:93`）；
- `teardown()`：退出时调用一次，发零指令、释放资源（`task.py:97`）。

框架替你保证四件事（`task.py:2-14`）：
| 保证 | 含义 | 为什么重要 |
|---|---|---|
| **固定频率** | 按绝对时基排下一拍，补偿执行耗时，避免累积漂移；超时记 `overrun` | 不会"越跑越慢" |
| **异常隔离** | 单拍异常**不杀线程**；连续异常超阈值 → 进入 `FAULT` 并上报 | 一个任务坏了，车不至于立即失控 |
| **心跳** | `last_tick_at / ticks / errors / overruns` 随时可查 | `SafetyTask` 就靠这个判断"谁没在跳" |
| **优雅停止** | `stop()` 置位 + `join`，`teardown()` 保证被调用 | 干净退出，不残留僵尸线程 |

**健康度状态机**（`task.py:30-35`）：
```
INIT → RUNNING → DEGRADED（有异常但仍在跑）→ FAULT（连续异常超阈值，已停 tick）→ STOPPED
```
连续异常达到 `max_consecutive_errors = 5` 次即判 `FAULT`（`task.py:63`）。

**写任务的铁律**（`task.py:12-14`，原文）：
- `tick()` 必须**非阻塞、无长 I/O、不持锁等待别的任务**；重活放到独立进程（RKNN/底盘/云台）。
- 任务之间**只能**通过 `Blackboard`（高频状态）与 `EventBus`（低频事件）通信，**禁止直接互相调用**。

### 4.3 黑板 `Blackboard`：线程间的"共享记事本"（`core/blackboard.py`）

```dsh-ui
{"title":"黑板是什么","items":[{"type":"table","columns":["设计要点","人话","原文行"],"rows":[["值必须不可变","写入即替换，不做原地修改；frozen dataclass / tuple / 数字 / 字符串","blackboard.py:5"],["每项带时间戳与来源","读取方能判断"新鲜度"（get_fresh），避免拿过期数据决策","blackboard.py:6"],["一把 RLock 保护","临界区只有 dict 存取，禁止持锁期间做 I/O 或外部回调","blackboard.py:7"],["高频走黑板，低频走事件","位姿、视觉帧这种每秒几十次的走黑板；事件这种偶发的走总线","blackboard.py:8"]]}]}
```

**所有标准键名集中在文件顶部**（`blackboard.py:21-62`），这样就不会出现"A 线程写 `pose`、B 线程读 `Pose`"的拼写漂移。主要键：

| 键常量 | 值 | 谁写 | 谁读 |
|---|---|---|---|
| `P_POSE` | 信念位姿（米 + 地图 yaw） | Localization | 全都要 |
| `P_POSE_RAW` | 未加修正的原始融合位姿 | Localization | 诊断 |
| `P_ODOM_AGE` | 里程计最近一帧距今秒数 | Localization | Safety |
| `P_ROUTE` | 当前路线（不可变） | main | Mission/Tracker |
| `P_MISSION_PHASE` | `IDLE/DRIVE/APPROACH/STOP/WAIT/FINISH/SAFE` | Mission | 全部 |
| `P_STOP_POINT` | 当前停站点上下文 | Mission | Aim |
| `P_TARGET` | 移动靶估计状态 | Aim | 规划 |
| `P_AIM` | 云台与开火状态 | Aim | Safety |
| `P_AIM_RED` | 红面早退（见下） | **Aim 写** | **Mission 读（唯一消费者）** |
| `P_DRONE` | 无人机情报 | Drone | Mission/规划 |
| `P_FIELD_LAYOUT` | 无人机下发的完整场地布局 | Drone | main |
| `P_LAYOUT_ARRIVALS` | 已接受的布局帧计数（单调递增） | Drone | StepLoop |
| `P_FIX` | 误差修正量（吸附/漂移台账） | Localization | Mission |
| `P_SAFETY` | 安全状态 | Safety | 全部 |
| `P_TELEMETRY` | 遥测快照 | Telemetry | 诊断 |

**关键设计：每个共享键都带"读写契约"注释**。以 `P_AIM_RED` 为例（`blackboard.py:32-47`）——它把物理事实（靶板一面蓝一面红，蓝面随机但恒定）和取值语义写得明明白白：
> 键不存在/`None` → **no-op**，Mission 行为与未加本特性时**逐位一致**；
> `{"confirmed": False, ...}` → 只是观测，**不得据此早退**；
> `{"confirmed": True, ...}` → 锁定红连续累计 ≥ `aim.red_early_leave_s` ⇒ Mission 提前离站。

这就是为什么 dzbrain 能**逐位兼容旧行为**：新功能全是"有数据才生效，没数据就当没这回事"。

还有 `P_MOTION_LIMIT`（`blackboard.py:64-95`）——一个"提前一脚刹车"的键：把线速度**只降不升**钳到 `speed_mps` 以内；`speed_mps=0.0` 是"请求停车"，读侧必须真给 0，**不受 `pursuit.min_speed_mps` 蠕动下限兜底**。同时它**不取代 Safety**：`P_SAFETY` 仍是唯一强制停车者。

### 4.4 事件总线 `EventBus`（`core/events.py`）
低频事件（故障上报、布局到达、停车请求）走这里。`EV_TASK_FAULT` 就是一个事件：任务线程报错时 `SafetyTask` 听到它就去停车。

---

## 5. 对外通讯：话题表 `io/contract.py`

`io/contract.py:58-110` 是 `TOPICS` 表——**所有跟外面世界打交道的话题都在这里登记**，不许到处散写字符串。分三块：

### 5.1 输入（别人发给我们）
| 话题 | 消息类型 | 单位/含义 | 备注 |
|---|---|---|---|
| `/odom/localization` | `nav_msgs/Odometry` | **格**（×0.35 = 米）；yaw=原始车体角(+π/2=地图航向) | 发布者是 `serial_reader`（车端 install 副本） |
| `/imu_data` | `sensor_msgs/Imu` | rad/s, m/s² | |
| `/offset_center` | `std_msgs/Int32MultiArray` | px / 类别 / 框宽高 | 字段=[0]dx [1]dy [2]cls+1 [3]框宽 [4]框高 [5..7]关键点 |
| `/aim/status` | `std_msgs/Int32` | 0=IDLE 1=SCAN 2=TRACK | |
| `/shoot_count` | `std_msgs/Int32` | 累计发数 | |
| `/nav/manual_override` | `std_msgs/UInt8` | 1=遥控接管底盘 | |
| `nav/manual_gimbal` | `std_msgs/UInt8` | 1=遥控接管云台 | **注意历史话题名没有前导斜杠** |
| `/nav/waypoint` | `std_msgs/String` | 外部命令：路线/复位/单点 | 你之前"发多点指令"就发到这里 |

### 5.2 输出（我们发给别人）
| 话题 | 消息类型 | 含义 | 备注 |
|---|---|---|---|
| `pursuitAngle` | `geometry_msgs/Twist` | linear.x=m/s；**angular.z=舵角【度】(正=左转)** | **★零 Twist = 刹车；50Hz 保活（底盘 1s 看门狗）** |
| `/nav/scan_request` | `std_msgs/UInt8` | 0=停扫 1=单趟 2=持续 | |
| `/aim/fire_gate` | `std_msgs/UInt8` | 1=允许开火 | ★新增，修旧版死线 |
| `/aim/fire_ready` | `std_msgs/UInt8` | 1=瞄准合格（预测落点在开火框内） | 与 fire_gate 构成**两键开火**：云台补丁只在两者都为 1 时开火 |
| `/move_base/stop_signal` | `std_msgs/UInt8` | 1=允许云台扫描+射击（历史话题，云台侧原先无人订阅） | |
| `/nav/motion_state` | `std_msgs/UInt8` | 0 前进 / 1 倒车（仅变化时发） | |
| `/voice_switch` | `std_msgs/UInt8` | 语音 ID | |
| `/aim/lead_x` | `std_msgs/Float32` | ★提前量（px，云台补丁订阅；旧版无此能力） | |

### 5.3 新版本新增（与无人机/裁判/云台补丁对齐）
| 话题 | 含义 |
|---|---|
| `/dzlink/drone_to_car` | JSON 情报：桥洞/靶区/雷障核对/心跳 |
| `/dzlink/car_to_drone` | JSON：车状态/ack |
| `/HpAndHitmsg` | **★★命中闭环真源**：裁判回传帧，字段=[0]对方HP [1]己方HP [2]被击打方位 [3]硬件累计发数 [4]剩余弹药；解析见 `io/referee.py` |
| `/referee/hit`、`/referee/ammo` | 兜底源，**只有在从未收到合法裁判帧时才认** |

**QoS 契约**（`contract.py:22-43`，改错会导致丢消息或收不到）：
- ★ 底盘对 `pursuitAngle` 的订阅是 **depth 1 / BEST_EFFORT**；**换成 RELIABLE 会让底盘收到慢/旧指令，表现为"车一顿一顿"**。
- 每个 `Topic` 还带 `unit` 和 `note`，单位换算**只在这里做一次**，别处不许再写（`contract.py:142-144`）。

---

## 6. 八个任务线程逐个深挖

### 6.1 `LocalizationTask`（定位）—— `tasks/localization.py` 1490 行
**干什么**：回答"我在哪"。这是所有决策的地基——定位错了，后面全错。

每拍逻辑（`localization.py:5-31`）：
1. 读 `bridge.pose()`（io 层已把**格单位 + 原始车体 yaw** 换算成米制地图 Pose）；
2. 航点吸附：当实际位置距某航点格心小于 `pursuit.arrival_m` 时，把 `P_POSE` 的**位置**锚到该格心（**yaw 不动**）；
3. 横向重锚：上限 0.35 m + `anchor_scale_max` 8% 闸（比云端更细）；
4. **航向完整性守卫**（`_update_yaw_guard`）—— 见 §11。

### 6.2 `SafetyTask`（安全）—— `tasks/safety.py` 529 行
**干什么**：兜底刹车。它是**唯一**能强制停车的角色。

关键点（`safety.py:97-110`）：
- 有个"启动宽限"：构造时刻起给 3.0 s，**等定位数据中**（`DEGRADED`，不是 `SAFE_STOP`）；
- **为什么需要它**（`safety.py:100-108`，实测教训）：两个任务对"有没有定位"的判断**天然差一拍**——Localization 先 tick 还没拿到里程计 → 把 `P_ODOM_AGE` 写成 `inf`；Safety 后 tick → 桥已收到帧（`pose_age` 有限）。旧判据因此绕开启动宽限、直接落到 `pose_lost` → 锁存 `SAFE_STOP`，而安全**刻意不自动恢复** ⇒ 车一动不动。
- 日志特征：只有一条"启动宽限（0.0x/3.0s）"，没有"启动宽限期已过"，却出现 `pose_lost` → **一眼认出这个坑**。

`SAFE_STOP` 是**锁存**的：一旦锁存，要显式清场才能动。这是设计选择——宁可多拦一次，不可放过一次。

### 6.3 `TrackerTask`（循迹）—— `tasks/tracker.py` 911 行
**干什么**：把规划好的路径变成底盘的目标速度/转向。

它把档值直接喂给 `pursuit.controller.compute_command(cruise_mps=...)`，因为**驱动指令只有 Tracker 一个写者**（`blackboard.py:67-69`）——这是防止"两个模块同时改速度打架"的关键设计。

### 6.4 `MissionTask`（任务调度）—— `tasks/mission.py` 4268 行
**干什么**：一段一段地走完整条路线。全场第二大文件。

关键机制（`main.py:44-68` 与 `mission.py` 头部）：
- `MissionTask.request_segment()`：线程安全地载入段并发车（10 Hz tick 里）；
- 段末停在 `target_cell`；`P_CAR_STATE` 回报 `phase/segment_id/known_count/target_cell/block`；
- 到 end 格 ⇒ `FINISH`（主循环退出码 0）。
- 红面早退：在射击停站内，若 `P_AIM_RED.confirmed=True` 且锁定红连续累计 ≥ `aim.red_early_leave_s` ⇒ 提前离站（`ACQUIRE`/`FIRE` 两段）。

### 6.5 `AimTask`（云台自瞄）—— `tasks/aim.py` 1372 行
**干什么**：云台锁定 + 开火。

两键开火（`contract.py:82-86`）：
- `fire_gate=1` 允许开火 + `fire_ready=1` 瞄准合格（预测落点在开火框内）→ 云台补丁才开火。
- **靶未锁只记不拦**；是否开火由联锁决定。

### 6.6 `FixTask`（误差修正）—— `tasks/fix.py` 461 行
**干什么**：把偏差补回去。`P_FIX`（吸附/漂移台账，Localization 写）与 `P_FIX_EST`（**估计量**，FixTask 写：比例误差估计/漂移统计）是两个不同键，别混。

### 6.7 `DroneTask`（无人机）—— `tasks/drone.py` 1486 行
**干什么**：接收并校验无人机下发的 `field_layout`，写进 `P_FIELD_LAYOUT`。

**关键**：**没有布局就不规划、不出发**。`DroneTask` 等布局超时 → 退出码 3。窗口耗尽照常出发（用已得情报 + 默认格）；**无 `P_DRONE` 时不等、不换，行为与接线前逐字一致**（`mission.py:106`）。

### 6.8 `TelemetryTask`（遥测）—— `tasks/telemetry.py` 295 行
**干什么**：把车状态打包上报。`P_TELEMETRY` 快照供诊断/复盘用。

---

## 7. 步进循环 `StepLoop` 与 T27（`main.py:1003`）

这是**目前车端真正在跑的模式**（`main.py:44-68`）：

```
布局 v2 到达（version 大者胜）
   → 记下 P_FIELD_LAYOUT；DRIVING 期间只记不换段（换段只在段边界）
   → 段规划：MapSpec{未知格=不可通行}，from_cell=当前格，target=layout.target_cell
       规划失败/无 target/无位姿 ⇒ WAIT_INFO，不退出
   → 段执行：MissionTask.request_segment()，段末停 target_cell
   → P_CAR_STATE 回报
   → 到 end 格 ⇒ FINISH（退出码 0）
   → 安全：实际 belief 连续 3 拍落入未知格 ⇒ SAFE_STOP（停段 + 刹车，不再自动恢复）
```

**为什么引入 `P_LAYOUT_ARRIVALS`**（`blackboard.py:50-57`）：无人机每 0.5 s 重发同版本心跳，而 `P_FIELD_LAYOUT` 的值不变 ⇒ StepLoop 无法从"值"判断这是新到的一帧还是手里捂着的旧值；只有**版本递增**才刷新新鲜度 ⇒ 发射器 12 s 一版时每段白等 11 s。所以加了"布局帧到达计数"，StepLoop 只认计数变化。

---

## 8. 路线规划器 `route/planner.py`（5050 行，全场最大）

**两个引擎**：

### 8.1 旧引擎 `plan_route()`（`planner.py:3001`）
用"原地转向 + 软惩罚"绕开死角，代价里掺大量绕远惩罚。**产生 `:PIVOT` 折返**。

### 8.2 tracker-safe 引擎 `plan_route_tracker_safe()`（`planner.py:4785`）
为什么另起一个引擎（`planner.py:3777-3796`，全部有实测数字）：
1. **原地转向（PIVOT）不可仿真**：旧引擎用 180° 折返 + `:PIVOT` 处理死角，但 `tools/sim.py` 与真车循迹层都**不建模 PIVOT** ⇒ 真值轨迹只能画一个大弧兜回去：实测峰值横向误差可达 **0.4292 m**。
2. **紧凑 U 形折返会被整段切掉**：两条反向腿之间只隔 1 格时，前视 0.36~0.40 m ≥ 单格 0.35 m ⇒ 车提前瞄到后面；实测车心离顶点最近只有 **0.240 m**。
3. **雷在顶点被拐过去 = 漏压**：纯追踪在顶点切内，实测漏压 `B2`（0.0364 m，阈值 0.035）。

**本引擎的硬规则**（全部可枚举、可复现）：
- 只走**单格**（正交）；
- **禁 180°**（转角 ≤ 90°）⇒ 路线里**不含 `:PIVOT`**；
- **禁紧凑 U 形折返**；
- **雷必须直线穿心**（转角 ≤ 10°），"口袋雷"允许 90° 拐过并给重罚；
- 净空用**物理口径**（障碍段 ≥ 0.215 / 拐角顶点 ≥ 0.3029 / 靶板 ≥ 0.0675 / 挡板细条 ≥ 0.0775）。

搜索算法：**朝向感知 Dijkstra**（状态 = 格 × 本腿方向 × 上一腿方向）+ 分阶段顺序 DP。

### 8.3 拐角条件切角（`planner.py:4458-4473`）
把 90° 直角 → 45° 斜段。为什么是"删顶点"不是"插半格偏移点"：全库**没有**"格心+米制偏移"的 token 语法（`parser.normalize_token()` 要求格名后全是数字，`F3+0.175` 直接 `ValueError`），而路线是**以 token 串**传给车侧的 ⇒ 偏移点要么解析失败、要么被静默丢回格心。实测（现役 44 点）：切 6 个角 → 38 点，顶点超调 0.04912→0.03887 m（−20.9%）、急转弯 18→11 处、靶板净空不足拍 49→0。**默认关闭**（`planner.corner_cut`，缺省 False）⇒ 现役发车路线逐 token 不变。

### 8.4 增量续算 G4（`planner.py:4625-4640`）
为什么必须**新增入参**而不是"把 `MapSpec.start` 改成当前格"：`MapSpec` 是**地图**（map.yaml 解析产物），起点进地图 ⇒ 任何同时读 `spec.start` 的地方都会跟着漂。且实测把 start 改到别处后规划器直接 `PlannerError: 起点不在下半场桥洞以东`——那条路**走不通**，也不该走通。

---

## 9. 安全：三道防线

### 9.1 AGC-RECTIFY 运行面闸门分级（`main.py:31-42`）

运行面（发车前 + 步进循环）的**拦停型互锁一律降级 warn + 日志**，**只留 3 条硬拦**：
1. 硬件急停 / 遥控接管（`hw_estop`）；
2. 激光 / 开火联锁（含红蓝面未定，`fire_interlock`）；
3. 软件边界保护（边界 = 地面线，`field_boundary`）。

每条 warn 打印 `GATE[warn] key=<键> 字段=值 …`。**离线交付面（门禁/check.py/单测）不降级。**

生效口径 `step.gate_policy`（缺省 = `auto`）：
- `auto`：`step.layout_ttl_s <= 0` ⇒ `warn`，否则 `hard`；
- `warn`/`degrade`：运行面降级（目标态）；
- `hard`/`legacy`：旧的拦停语义（只给既有回归断言用）。

### 9.2 航向完整性守卫（`localization.py`）

完整参数表（`localization.py:95-115`）：

| 判据 | 阈值 | 说明 |
|---|---|---|
| 静止期 yaw 率 `yaw_static_dps` | **45 °/s** | `v < yaw_min_speed_mps` 就算静止；车没动却被"转"了 —— 只可能是被搬/被转或 yaw 链坏了（事故静止段实测 169.2°/s） |
| 运动期包络 | `max(yaw_diff_spin_dps, yaw_diff_gain_dps_per_mps × v)` | 下限 60°/s，随速度增益 525°/s per m/s（A2 实测，名义阿克曼极限 210.7°/s per m/s，实车含滑移/有效轴距） |
| 单帧跳变 `yaw_jump_deg` | **30°** | 立即判故障（重定位/坐标跳变） |
| IMU 矛盾 `yaw_imu_check_dps` | 默认 20.0 | 与 IMU 对比 |
| 连续拍数 `yaw_bad_ticks` | 默认 3（新帧） | **单帧毛刺不刹，必须连续 N 个新帧都超限** |
| 自动复位 `yaw_clear_ticks` | 默认 20（≈ 5Hz 下 4 s） | 连续可信新帧后守卫自动复位（Safety 另清） |

判了之后：经 `P_MOTION_LIMIT` + `EV_TASK_FAULT` 请求停车（`localization.py:30-31`），Safety 仍锁存。

**背景**（`localization.py:98-99`）：K9 事故 IMU 地图 yaw 1.8 s 内 +90°→−78°（峰值 120.3°/s @ 0.112 m/s）；旧版把 yaw 当"直接可用观测" ⇒ 下游一路用坏航向直到出界。

### 9.3 退出码全表（`main.py:64-67`）

| 退出码 | 含义 |
|---|---|
| **0** | 正常：跑到终点 FINISH |
| **3** | 无布局 / 装配失败（最常见：等无人机布局超时） |
| **4** | 布局规划失败 |
| **5** | ROS 环境 / 位姿管线未就绪（域号冲突 `DZ_ROS_DOMAIN≠ROS_DOMAIN_ID`、重复 dzbrain 实例、`/odom/localization` 无首帧、运行期断流） |
| **6** | 启动契约不通过（`startup_contract()` 表里必需 sub/pub 端点缺失或未匹配） |

5 的现场判据与三条照抄命令见 `main.py` 里的 `_ROS_DIAG_COMMANDS`。

---

## 10. 配置中心：`core/config.py`（945 行）

**干什么**：一元化读取 + **显式默认值** + 启动期校验（`config.py:1-10`）。

三条规则：
1. 所有键集中在 `SCHEMA`，带类型/范围/单位/说明；未登记的键**记录告警**而不是默默忽略；
2. 缺键 → 用 SCHEMA 里的显式默认值，并在 `issues` 里留一条 `default-used` 记录（可查）；
3. `validate()` 用**两级判据**（`config.py:854-876`）：① 键在 SCHEMA 里 ⇒ 合法（**登记即生效**）；② 键的**段**在 SCHEMA 里但键本身没登记 ⇒ 报"未登记的配置键"（拼错仍能抓到）。

还支持 `section(prefix)` 按段取整块配置（`config.py:812`）——这是未来"配置分域"改造的现成基础。

**`INTENTIONALLY_UNREGISTERED` 白名单**（`config.py:620-622`）：故意**不**登记进 SCHEMA 的键，逐条有出处。为什么故意？因为 `test_calib_backfill.py:2.3` 明确把"SCHEMA 里没有它"当作设计约束钉住（防止两套判据各说各话）。

---

## 11. 离线验证：`tools/`（不进车的训练场）

| 文件 | 行数 | 干什么 |
|---|---|---|
| `sim.py` | 145003 字节（约 3500 行） | 纯 Python 仿真：无 ROS、无车，跑真实任务线程看结果 |
| `sim_cli.py` | 13590 字节 | 命令行入口 |

用法（`main.py:14`）：`python -m dzbrain.main --bench --seconds 5` —— 无车/无 ROS 的联调，在 `FakeBridge` 上跑真实任务线程。

**为什么重要**：算法/控制类改动，**单元测试绿不算过**——要跑仿真闭环看退出码 + 指标表；实车链路改动要跑实车闭环（bench 日志 + 视频）。

---

## 12. 你之前碰到的现象，现在都能对上代码了

| 现象 | 对应机制 | 源码位置 |
|---|---|---|
| 发多点指令没反应 | 布局超时退出码 3，进程不在了 | `main.py:27-28` |
| 车不动 | ① `cam_viewer` 抢 `/pursuitAngle` 发全零；② dzbrain 起在"文件模式"没进布局循环 | `contract.py:77-79` / `main.py:16-23` |
| yaw 安全阈值 | 静止 45°/s + 连续 3 拍 | `localization.py:114/220` |
| 车一顿一顿 | QoS 从 BEST_EFFORT 换成 RELIABLE | `contract.py:35-36` |
| 假崩溃 | 车端 rclpy 默认接走 SIGTERM，只 `rcl_shutdown` 不退进程 | `main.py:95-99` |
| 启动宽限被绕过 | 两个任务对"有没有定位"判断天然差一拍 | `safety.py:100-108` |

---

## 13. 可以删 / 可以优化（针对 dzbrain 本身）

**⚠️ 注意：`src/dzbrain/dzbrain/**` 是冻结区**（AGENTS.md §1.3）——以下只是建议，落地必须走"先出 patch 设计稿 → 你裁决"的流程。

### 13.1 三个真正的病灶（不是包结构，而是这三样）

| 病灶 | 现状 | 建议 |
|---|---|---|
| **`main.py` 是政策枢纽** | 3306 行、39 个 def，压着步进循环本体、`WAIT_INFO`、布局检查、`SAFE_STOP` | 把步进循环抽进一个独立的新步进模块（暂名 stepper，放 tasks 层），`main.py` 退化成纯装配。**判据**：main.py 的 def 数从 39 降到装配级别（预计 ≤15） |
| **一个全局配置桶** | `robot.yaml` 只 1 个顶层 key，`config.py` 945 行 | 按 `safety:/stepper:/localization:/pursuit:` 分域。`Config.section()` 已支持，机械上能拆。**判据**：改安全层参数不碰定位参数 |
| **`SAFE_STOP` 散落 6 个文件** | main.py(9)、safety.py(8)、localization.py(7)、mission.py(6)、io/contract.py(3)、tracker.py(1) 各写各的 | 抽一个统一策略模块（暂名 policy，放 core 层）定义三档 DEV/AUTO/RACE，18 条检查各自声明行为。**判据**：改一次档位从"翻 6 个文件"变成"改 1 个文件" |

### 13.2 可以删的

- **注释约占 2/3**。其中大量是历史沿革（"t25 改的""2026-10-11 从 0.30 改 0.35"）。**但不要现在删**——这些注释是"为什么这么写"的唯一证据，抽到 `docs/` 后再瘦身，别直接抹掉。
- **冗余/未接线代码**：如 `io/contract.py:87-88` 明确写着 `stop_signal` 是"历史话题，云台侧原先无人订阅"；`/referee/hit`、`/referee/ammo` 标注为兜底源。逐条清点可瘦身，但要小心它们可能是兼容旧报文的关键。

### 13.3 判断"模块化够不够"的四把尺子

1. **改一处行为要改几个文件？** 今天改档位 = 6 个。目标 ≤2。
2. **能不能只加载一部分就跑？** 今天不能，main.py 装配全部。
3. **配置能不能按模块分开？** 今天不能，978 行一个桶。
4. **测试能不能针对单模块跑？** 今天能。

---

## 14. 怎么开始学（给零基础的建议路线）

1. **先跑一次**：`python -m dzbrain.main --dry-run`（只做门禁，不起线程）→ 看横幅和校验结果，理解"配置 → 校验 → 拒绝/放行"。
2. **再看启动横幅**：`start_banner()` 打印的每个字段，就是当前车上的真值。
3. **读 `blackboard.py` 顶部**：所有键名和它们的"读写契约"——这是理解线程间如何协作的最快路径。
4. **读 `io/contract.py` 的 `TOPICS`**：这是"车和外界的全部对话清单"，每个话题带单位、方向和备注。
5. **挑一个任务深读**：建议从 `tasks/safety.py`（529 行）开始——最短，且最能体现"宁可多拦不可放过"的设计哲学。
6. **最后读 `main.py` 顶部注释**（1-68 行）：那本身就是一份完整的架构设计文档。

---

*本报告全程只读，未修改 `dzbrain/**` 任何文件；所有行号均为 `git` 当前工作区实测。*