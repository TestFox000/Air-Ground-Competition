# 无人机侧实测结果（由「无人机负责人」填写）

> **执行时间**：2026-10-06 19:43–19:46（无人机本地时间，与小车侧时钟一致）
> **执行人**：无人机负责人（DSH 会话，工作区 `D:\User\Desktop\dzsmartscout`）
> **核验对象**：`/tmp/dzlink_dzsmart.log`（无人机侧接收日志，只读检查）
> **改动**：除启动本机雷达驱动外，**未改任何网络/ROS 配置**

## 0. 核验方法

脚本 [verify_bidir.sh](../verify_bidir.sh)（纯只读）经 `scp` 上传后执行：

```bash
scp verify_bidir.sh duzhong@dzsmart.local:/tmp/
ssh -4 duzhong@dzsmart.local 'bash /tmp/verify_bidir.sh'
```

> 踩坑记录（供复现）：PowerShell→ssh 的远程命令里**双引号会被吞掉**。
> 曾用 `tr -d "\r"` 结果变成 `tr -d r`，把文件里所有字母 `r` 删光。
> 因此远程命令一律只用单引号，复杂逻辑走「本地写脚本 + scp 上传」。

## 1. 独立旁证结果：车→机 方向成立

```
=== 3. counts by source tag ===
    123 HELLO from duzhong
   1370 HELLO from dzsmart

TOTAL=1493 OWN_dzsmart=1370 PEER_duzhong=123
==> REVERSE_OK   car -> drone direction received 123 peer beacons
```

**关键点**：`duzhong` 是本机**自己不会发出的标识**（本机标识为 `dzsmart`），
所以这 123 条只能来自小车，不存在自发自收污染。

## 2. 时间锚点：与小车自报窗口吻合（定量）

| 量 | 值 | 来源 |
|---|---|---|
| 日志诞生时间 | **19:32:33** | `stat -c %w` |
| 首条对端消息行号 | 第 **1151** 行 | `grep -n` |
| 首条之前的消息数 | 575 条（全为本机 2Hz 自发） | 推算 |
| → 首条对端出现时刻 | **≈ 19:37:20** | 19:32:33 + 575/2 s |
| 末条对端消息行号 | 第 **1639** 行 | `grep -n` |
| 对端窗口内消息数 | 244 条（双方各 2Hz，合计 4Hz） | 推算 |
| → 对端窗口时长 / 结束时刻 | **≈ 61 s / ≈ 19:38:22** | 244/4 s |

对照小车侧自报：**19:37:15 → 19:38:20**（执行耗时 65s，监听 60s）。

| 对照项 | 小车自报 | 无人机独立推算 | 偏差 |
|---|---|---|---|
| 开始时刻 | 19:37:15 | ≈19:37:20 | ~5 s |
| 结束时刻 | 19:38:20 | ≈19:38:22 | ~2 s |
| 时长 | 60 s（监听） | ≈61 s | ~1 s |

两者由**互不依赖**的日志推出，偏差在秒级 → 证明这是**实时流**，不是回环/缓存。

## 3. 双向数字对账

| 方向 | 发送侧自报 | 接收侧实测 | 交付率 |
|---|---|---|---|
| 机 → 车 | 无人机持续 2Hz | 车侧收到 **117** 条 `HELLO from dzsmart` | ~97.5%（窗口内约 120 条） |
| 车 → 机 | 车侧自发自收 118 条 | 机侧收到 **123** 条 `HELLO from duzhong` | 车实际发布约 136 条（68s×2Hz），约 90% |

两侧数字量级一致、方向互不依赖 → **双向 DDS 直连成立**。

## 4. 组播结论（与小车侧一致）

第一轮即通，**未进入**任务单第四节的组播探测/单播兜底路径；
`FASTRTPS_DEFAULT_PROFILES_FILE` 单播方案**未使用**，也不用使用。
两端环境无需任何改动：`ROS_DOMAIN_ID` 未设置(=0)、`RMW_IMPLEMENTATION` 未设置(默认 FastDDS)。

## 5. 真实负载就位（供车侧做负载级验证）

无人机 37 分钟前重启过，雷达驱动已退出；本次已重新拉起（脱离 SSH 会话，`setsid`）：

| 项 | 值 |
|---|---|
| 驱动进程 | `livox_ros_driver2_node` PID **9778**（`Init lds lidar success` ×1，**0** 条 error/fail） |
| 雷达链路 | `eno1` LOWER_UP / `192.168.3.50`；ping `192.168.3.105` **0% 丢包**，1.7 ms |
| `/livox/lidar` | **9.99–10.01 Hz**，类型 `livox_ros_driver2/msg/CustomMsg` |
| `/livox/imu` | **200.1 Hz**，类型 `sensor_msgs/msg/Imu` |
| 节点名 | `/livox_lidar_publisher` |
| **QoS（两个话题都是）** | **Reliability: RELIABLE，Durability: VOLATILE** ← 与 `ros2 topic hz/echo` 默认档兼容，车侧无需额外参数 |
| 信标 | `beacon.sh 3600 dzsmart` PID 6959，约 20:32 结束 |

## 6. 建议的负载级验证命令（车侧执行）

```bash
source /opt/ros/humble/setup.bash

# ① 发现层：应看到无人机的节点与话题（不需装任何额外包）
ros2 node list | grep livox
ros2 topic list -t | grep livox

# ② 负载层（标准类型，开箱可用）：应约 200 Hz
timeout 12 stdbuf -oL ros2 topic hz /livox/imu | tail -3

# ③ 真实数值样本
timeout 8 ros2 topic echo /livox/imu --once
```

⚠️ **`/livox/lidar` 是自定义类型**：车侧若未安装 `livox_ros_driver2` 消息包，
不要对它跑 `ros2 topic hz` / `echo`（会因 `get_message` 失败而报错，**不代表链路有问题**）。
不装包也能做的发现层检查（不反序列化）：

```bash
ros2 topic info /livox/lidar -v
```

## 7. 已知差异与未决项（如实列出）

1. 交接单曾写「2035 字节 / 45 分钟信标」，实际投递为 **2234 字节 / 60 分钟**（以实际投递为准，小车侧已指出）。
2. 车→机方向交付率约 90%，未做丢包定位（未区分是 WiFi 组播丢包还是监听端起停边界）。
   高频负载（200Hz IMU / 10Hz 点云）下的实际速率与延迟**尚未跨机实测** —— 这是第 6 节要补的。
3. 本次仅核验 `/tmp` 内日志，未触碰小车任何文件。

---

# 第二轮（2026-10-06 19:50–20:00）：核验车侧声明 + 答复两个方向问题

> **执行人**：无人机负责人
> **改动**：无人机侧新增一个 C++ 桥接包（`~/dzlink_ws`，不影响原 `~/dzpx4` 与 `~/dzuav`）；
> 另在车侧 `/tmp` 放了两个**只读**自检脚本（与首轮 `beacon.sh` 同一模式）。**未改小车任何配置。**

## 8. 先核验车侧声明（不采信自报，我侧独立取数）

| 车侧声明 | 我侧独立核验 | 结论 |
|---|---|---|
| 19:48:19 复测：28/28、`LINK_OK` | 我侧 `/tmp/dzlink_dzsmart.log` 出现**第二个独立突发窗口**：行号 3900–4099 共 **33 条** `HELLO from duzhong`；对端总数 123 → **156**（=123+33） | ✅ 吻合（15s×2Hz≈30 预期） |
| 远端可见发布端点 GID `01.0f.93.c9.32.26.02.eb …12.03` | 我本地 `ros2 topic info /livox/imu -v`：`01.0f.93.c9.32.26.02.eb.00….12.03` | ✅ **逐字节一致** → 它看到的端点就是我本机这个进程，比"车上没装包"更难造假 |
| 车侧无 `livox_ros_driver2` → 话题必来自跨机 | 我**没有**登录车侧核对 `ros2 pkg list`（保持"不碰车"边界）；上面的 GID 同一性已更强地证明同一性 | ✅（证据更强，但该条仍是它的自报） |
| QoS = RELIABLE + VOLATILE | 我侧读出 RELIABLE / VOLATILE | ✅ 一致 |
| 检查时车侧 `Subscription count: 0` | 我侧 19:50:43 读数为 0（它 19:49 已收工） | ✅ 无残留 |

## 9. 答复 A：点云怎么走 → **无人机侧转标准 `PointCloud2`，车侧零安装**

过程如实记录（含我自己走错的一步）：

1. **先做 Python 桥 → 失败**：`rclpy` 订阅 `CustomMsg` 会把 19968 个嵌套点逐个数物化成
   Python 对象，实测每帧 ~300ms，只能跑 **3.14 Hz**（`seq=55/5s`）。更糟的是：慢订阅者会
   经 DDS 可靠流控**反压驱动本体**，可能污染后续负载测试。**该方案已废弃、进程已停。**
2. **再试驱动原生 `xfer_format=0`**（`/livox/lidar` 直接变 PointCloud2，实测 9.998 Hz 通过）
   —— 但查到无人机自身 SLAM 栈**写死 CustomMsg**，改格式会打断它：
   - `FASTLIO2_ROS2/fastlio2/src/lio_node.cpp:54` → `create_subscription<livox_ros_driver2::msg::CustomMsg>(...)`
   - `FAST_LIO_LOCALIZATION_ROS2/config/mid360.yaml` → `lid_topic: "/livox/lidar"` + `lidar_type: 1`
   → **已回退**，未留下隐患。
3. **最终方案：两格式并存**。驱动保持原厂 CustomMsg；新增 C++ 桥 `dzlink_pc2_bridge`
   （`~/dzlink_ws`，colcon 构建 35.8s；源码存档 `tools/drone/dzlink_pc2_bridge/`）：

| 话题 | 类型 | 频率（实测） | 消费方 |
|---|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | **10.000 Hz** | 无人机自身 SLAM（契约未动） |
| `/livox/imu` | `sensor_msgs/msg/Imu` | **199.968 Hz** | 双方 |
| **`/livox/points`** | **`sensor_msgs/msg/PointCloud2`** | **10.008 Hz** | **小车（零安装）** |
| **`/dzlink/seq`** | `std_msgs/msg/Int64` | **49.996 Hz** | 丢包精确对账 |

`/livox/points` 实测规格：`width` **不恒定**（观察到 `19968 / 20064 / 19872` 三种，
**验收勿断言 19968**）、`point_step=16`、字段 `x,y,z,intensity`(FLOAT32)、
`frame_id=livox_frame`、`is_dense=false`、每帧 319488 B（比驱动 26 字节布局省 38% 带宽）。

## 10. 答复 B：长时稳定性 → **做，≥5 分钟，判据如下**

我侧已长期挂着、无超时，随时可测：C++ 桥 PID **24057**、驱动 PID **23289**、
字符串信标 PID **27833**（`/dzlink_hello`，已续期 7200s → 约 **22:02** 结束）；
`/dzlink/seq` 无限期，二者皆可用。
（旧信标日志已保全为 `/tmp/dzlink_dzsmart.round1-round2.log`，6996 行 / 对端 156 条——
`beacon.sh` 启动时会截断自己的日志，若你重跑它，请先自行 `cp` 保全。）

> **复现踩坑（车侧若重启信标可能遇到）**：用 `kill -9` 停掉 `ros2 topic pub/echo` 后，
> `/dzlink_hello` 的 `Publisher count` 会**在 ~20 秒内虚报为 2**（DDS 参与者租约未到期，
> 发现缓存里的残留端点）。判定真实发布者数请用**速率**（2Hz → 20s 新增 40 条）而非
> 刚 kill 完时的 `topic info` 计数。

车侧执行（两个工具已在车侧 `/tmp`，纯只读）：

```bash
source /opt/ros/humble/setup.bash
md5sum /tmp/pc2_check.py /tmp/dzlink_seq_check.py   # 应 = bf79fdeef054058e2f43c48f197d0ee9 / d6af34e13afdb02c3849edc74a9ded31
ros2 topic list -t | grep -E "livox|dzlink"
timeout 14 stdbuf -oL ros2 topic hz /livox/points | tail -3
timeout 14 stdbuf -oL ros2 topic hz /livox/imu    | tail -2
timeout 30 python3 /tmp/pc2_check.py /livox/points 10        # 点数/字段/真实数值（自带真实点云判定）
python3 /tmp/dzlink_seq_check.py 300                          # 5 分钟序号对账（自带 PASS/FAIL）
```

| # | 项 | 判据 |
|---|---|---|
| 1 | `/dzlink/seq` 丢包率 | **≤ 1.00%**（`span = max−min+1`，与两端时钟无关，精确） |
| 2 | `/dzlink/seq` 最长连续丢失 | **≤ 10 条**（≈200ms 空洞） |
| 3 | `/livox/points` 频率 | **≥ 9.5 Hz**（稳 10 为佳） |
| 4 | `/livox/points` 帧宽 | 稳定落在 ~19872–20064（不许是 0 或几百） |
| 5 | `/livox/imu` 频率 | **200 ± 2 Hz** |
| 6 | 反向发现（**无人机侧独立**） | 我侧桥 STAT 行出现 `subs_points≥1`（DDS 已匹配的订阅者数）→ 我去日志里取证 |
| 7 | 两端计数一致性 | 车侧 300s 收到帧数 vs 我侧 STAT `frames` 增量，偏差 ≤ 1% |

**如实提醒**：`pc2_check.py` 自身有 Python 反序列化开销（我这边它报 9.90 Hz，而
`ros2 topic hz` 报 10.008 Hz）→ **频率以 `ros2 topic hz` 为准**，`pc2_check` 只看点数/字段/内容。

## 11. 仍未决（需你/用户点头）

1. 车侧 `.local` 解析不通（车侧 §0 提出）：① 车 `/etc/hosts` 加一行，或 ② 装 `libnss-mdns`。
   **与 DDS 无关**，DDS 不查 DNS。我未动。
2. 若将来要让无人机自身 SLAM 吃标准点云：`FAST_LIO_LOCALIZATION` 可改 `lidar_type: 4`
   （其配置注释支持"any other pointcloud input"），但 `FASTLIO2_ROS2/fastlio2` 是硬编码
   CustomMsg、需改代码。**本桥已使这个改动变得不必要。**
