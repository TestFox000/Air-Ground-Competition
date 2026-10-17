# 小车侧负载级验证结果（由「小车负责人」填写）

> **执行时间**：2026-10-06 19:46 → 19:49（车侧本地时间 CST，UTC+8）
> **执行人**：小车负责人（DSH 会话，工作区 `D:\User\Desktop\dzrobot_v10.05`）
> **被测对象**：无人机 `dzsmart.local`（10.240.174.104）上的 `/livox_lidar_publisher`
> **车侧身份**：`duzhong` / `10.240.174.188` / ROS 2 Humble / `ROS_DOMAIN_ID` 未设置(=0) / RMW 默认(FastDDS)
> **改动**：**无**（只读命令 + 你提供的 `/tmp/beacon.sh`）

## 0. 一个必须先说的前提：车侧 `dzsmart.local` 解析不了

```
$ ping -c 3 dzsmart.local
ping: dzsmart.local: 名称或服务未知          ← 车侧解析失败
$ getent hosts dzsmart.local
（无输出，失败）
$ systemctl is-active avahi-daemon
active                                       ← avahi 在跑，但 glibc 侧走不通 .local
$ grep -c dzsmart /etc/hosts
0                                            ← /etc/hosts 无条目
```

**这不影响本次验证**：ROS 2 / DDS 靠**组播发现**，不依赖 DNS/mDNS。下面所有结果都是纯 DDS 层拿到的，
没有一条以域名解析为前提。（若以后要从车上 `ssh`/`scp` 到无人机，需要另配解析 —— 见第 6 节，我没动。）

## 1. 发现层：车侧看得到无人机的节点与话题

```
$ ros2 node list
/livox_lidar_publisher

$ ros2 topic list -t | grep -i livox
/livox/imu [sensor_msgs/msg/Imu]
/livox/lidar [livox_ros_driver2/msg/CustomMsg]

$ ros2 pkg list | grep -i livox
（无）                                        ← 车上没装 livox_ros_driver2
```

**强证据**：车上**根本没有** `livox_ros_driver2`，所以 `/livox_lidar_publisher` 与这两个话题
**不可能是本机产生的** —— 它们在 DDS 图里出现，只能来自跨机的无人机。

### 1.1 端点与 QoS（`ros2 topic info -v`）

| 话题 | 类型 | Publisher | 端点节点 | GID（前缀） | QoS | Subscription |
|---|---|---|---|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | 1 | `livox_lidar_publisher` | `01.0f.93.c9.32.26.02.eb …13.03` | RELIABLE / VOLATILE | 0 |
| `/livox/imu` | `sensor_msgs/msg/Imu` | 1 | `livox_lidar_publisher` | `01.0f.93.c9.32.26.02.eb …12.03` | RELIABLE / VOLATILE | 0 |

- 两个 GID 共享同一 participant 前缀 `01.0f.93.c9.32.26.02.eb` → **同一远端 participant 的两个端点**，与「一个驱动节点发两路」一致；
- QoS 与你侧自报（RELIABLE + VOLATILE）**逐项吻合** → 车侧用 `hz/echo` 默认档即可，无需额外参数（实测已证）。

## 2. 负载层（标准类型，开箱可用）：`/livox/imu`

### 2.1 频率

```
$ timeout 12 stdbuf -oL ros2 topic hz /livox/imu | tail -3
average rate: 200.016
	min: 0.001s max: 0.014s std dev: 0.00213s window: 2007
```

| 项 | 车侧实测 | 你侧自报 | 判定 |
|---|---|---|---|
| 平均频率 | **200.016 Hz**（窗口 2007 条） | 200.1 Hz | 偏差 **0.04%** |
| 到达抖动 | 1 ms ~ 14 ms（std 2.1 ms） | — | WiFi 抖动存在，但**无停顿/无堆积** |

> 说明：IMU 消息**无序号字段**，因此这是「频率一致性」而非「逐条丢包统计」。不过 12s 窗口内
> 车侧 2007 条 vs 你侧 200.1Hz 预期 ≈2001 条，量级吻合，未见系统性丢失。

### 2.2 真实数值样本

```
$ timeout 10 ros2 topic echo /livox/imu --once
header:
  stamp: {sec: 1791287261, nanosec: 231373661}
  frame_id: livox_frame
orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
angular_velocity: {x: -0.01102255, y: 0.00408349, z: -0.00968659}
angular_velocity_covariance: [0.0, …]        ← 与你侧驱动行为一致
```

- **非零、非恒定**的真实角速度（约 ±0.01 rad/s 量级的静止零偏抖动）→ 是活的传感器流，不是空帧；
- `frame_id: livox_frame`、`orientation` 全零 + w=1（MID-360 驱动不提供姿态，符合预期）。

### 2.3 新鲜度（时延上界）

| 量 | 值 |
|---|---|
| 该消息 stamp | `1791287261.231373661`（无人机时钟） |
| 车侧打印完成时刻 | `1791287261.444813586`（车侧时钟） |
| → 消息龄上界 | **≤ 0.213 s**（含链路 + CLI 反序列化 + 打印开销 + 两端时钟差） |

车侧系统时间与你侧一致（均为 2026-10-06 19:4x），未见明显时钟失步；0.21s 是**上界**而非纯网络时延。

## 3. `/livox/lidar`：发现层通过，消费层差一个消息包

```
$ timeout 8 ros2 topic hz /livox/lidar
The message type 'livox_ros_driver2/msg/CustomMsg' is invalid      ← 预期内，非链路故障
```

| 层 | 车侧结果 |
|---|---|
| 发现层（不反序列化） | ✅ 类型名、publisher、GID、QoS 全部可见（见 §1.1） |
| 消费层（反序列化） | ❌ 车未装 `livox_ros_driver2`，无法接收点云 |

**结论**：正如你预警的，这条报错**不代表链路有问题**。但从「车要真正用点云」的角度，
这是一个**待办**：车侧需要 `livox_ros_driver2` 消息包（或由你侧转成 `sensor_msgs/PointCloud2`）。

> ⚠️ 我**没有**独立测到点云的 10 Hz / 19968 点 —— 没有消息包就无法反序列化，
> 所以那两项数据**以你侧自报为准**，我不冒充实测。

## 4. 同时刻共存复测：字符串信标仍然在跑

在完成上述负载验证的同一时段（19:48:19 CST），我又跑了一次你给的信标：

```
$ bash /tmp/beacon.sh 15 duzhong | tail -8
== 收到本机信标: 28 条（本机自发自收正常性检查）
== 收到对端信标: 28 条
== 对端样本: data: HELLO from dzsmart ×3
==> RESULT: LINK_OK
```

| 项 | 值 |
|---|---|
| 15s × 2Hz 预期 | ≈30 条 |
| 本机 / 对端实测 | **28 / 28**（约 93%） |
| 脚本指纹复核 | 2234 字节 / sha256 前16位 **`ef1a8d81446b0684`** ← 与首轮、与你侧完全一致（未被人替换） |

**意义**：同一条物理链路上，字符串信标（跨机 pub/sub）与 **200 Hz IMU 流**并存工作，
互不干扰 —— 说明这不是「刚好能发现」的侥幸，而是**可持续的双机 DDS 数据面**。

## 5. 逐条回答你要求的四项

| # | 你的验证项 | 车侧结果 | 判定 |
|---|---|---|---|
| 1 | `ros2 node list \| grep livox` → 应见 `/livox_lidar_publisher` | 见 `/livox_lidar_publisher` | ✅ |
| 2 | `ros2 topic list -t \| grep livox` → 应见 `/livox/lidar` 与 `/livox/imu` | 两话题类型均正确列出 | ✅ |
| 3 | `timeout 12 ros2 topic hz /livox/imu` → 应约 200 Hz | **200.016 Hz**（2007 条窗口） | ✅ |
| 4 | `timeout 8 ros2 topic echo /livox/imu --once` → 真实角速度/加速度 | 收到真实非零角速度，stamp 新鲜度 ≤0.21s | ✅ |
| 附 | `/livox/lidar` 只做发现层检查 | `topic info -v` 全通过；消费需装消息包 | ✅（如你预警） |

**没有用任何额外参数**：QoS 默认档直接匹配 RELIABLE+VOLATILE，与你侧 §5 的判断一致。

## 6. 车侧环境与我的改动（如实）

```
改动：无。
- 未改 /etc/hosts、未装任何包、未动 ~/.bashrc、未动 ~/dzrobot、未落盘 DDS profile；
- 只执行只读的 ros2 查询命令，以及你提供的 /tmp/beacon.sh（脚本自身写 /tmp/dzlink_duzhong.log）。
```

补充环境事实（供你后续设计参考）：

- 车侧 `ROS_DOMAIN_ID` 未设置(=0)、`RMW_IMPLEMENTATION` 未设置(默认 FastDDS)、Humble —— 与首轮一致；
- 验证期间车侧**应用栈（dzrobot 节点）未运行**，`ros2 node list` 只有你那个远端节点 → 结果干净；
- 车侧 `.local` 解析不通（§0）。如果你希望车上也能直接 `ssh dzsmart.local`，
  两条路可选（**都需要你或用户点头我才做**）：① 在车 `/etc/hosts` 加 `10.240.174.104 dzsmart.local`（最小改动）；
  ② 装 `libnss-mdns` 让 mDNS 生效（更"正确"，但要装包）。**DDS 不需要它**。

## 7. 结论与未决项

**结论：负载级验证通过** —— 跨机 DDS 数据面在**标准消息类型的高频负载（200 Hz）**下实测可用，
且与字符串信标同时共存；自定义类型（点云）在发现层可见、消费层待补消息包。

未决项（我不含糊）：

1. **点云 10 Hz / 19968 点未在车侧实测**（缺消息包）→ 若要车侧直接吃点云，先定：装 `livox_ros_driver2` 还是你在机上转 `PointCloud2`？
2. **丢包未做序号级统计**（IMU 无序号）→ 若要比对真实交付率，需要你侧在点云/自定义消息里带递增序号，或由车侧订阅后计数并与你侧发送计数对账。
3. **未测长时稳定性**（本次仅 12s 频率窗口 + 8s 单帧）→ 需要更长窗口（≥5 min）的话说一声，我可以挂后台跑。
4. 车侧 `.local` 解析不通（§0）—— 与数据面无关，但会影响「车 → 无人机」的 ssh/scp 类操作。

---

**一句话交接**：车侧确认 `/livox_lidar_publisher` 发现正常、`/livox/imu` **200.016 Hz** 实时到车、
QoS 默认档直通、字符串信标同时在线；**点云需要消息包才能消费**，其余待你定方向。
