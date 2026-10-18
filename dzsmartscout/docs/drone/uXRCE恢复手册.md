# uXRCE-DDS 会话恢复手册

> **适用对象**：机载 Jetson（`dzsmart`）上的 `MicroXRCEAgent` 与 PX4 飞控板之间的 uXRCE 会话。
> **本轮定位**：2026-10-17 19:03 Orin 重启后，agent 每 ~61 s 被看门狗重启一次，**会话始终没建立**（只有 `TermiosAgentLinux init running`，无任何 `ProxyClient create_publisher`），`/fmu/out/*` Publisher count = 0。**19:24 复查仍在同一循环。**
> **证据**：`~/drone_logs/agent_watchdog_agent.log`、`~/drone_logs/node_wdog.log`、`ros2 topic info -v`、`/proc/<pid>/environ`。
> **边界**：本文只给**恢复步骤与判据**，不代替现场操作。所有机端命令均为只读/诊断性质；标「未验证」的表示本轮没有条件实测。

---

## 0. 飞控恢复三步卡（现场先跑这个）

> 适用：`/fmu/out/*` 无发布者、agent 日志只有 `TermiosAgentLinux init running`。
> 19:24 复查：仍在"每 ~61 s 杀 agent 重启"的死循环（会话未建立）。
> 时间预算：**Step 1 ≈ 5 min · Step 2 ≈ 1 min · Step 3 ≈ 10 min（可选）**。

### Step 1 · 给飞控断电重启（唯一可靠手段）

```bash
# 1.0 先停看门狗，避免它按 61 s 节奏插手（可选但推荐，省得日志混在一起）
pkill -f orin_agent_watchdog

# 1.1 物理：断开飞控供电（拔电池 / 断 USB 供电），保持 5 秒，再上电
#     只重启 Orin 没用——飞控一直带电时 PX4 的 uxrce client 不会重连

# 1.2 等串口重新枚举
ls -l /dev/ttyACM0            # 必须重新出现
udevadm info -q property -n /dev/ttyACM0 | grep ID_MODEL    # HKUST_UAV_NxtPX4

# 1.3 重启 agent（旧句柄已随枚举失效）
pkill -f "MicroXRCEAgent serial"; sleep 2
setsid nohup MicroXRCEAgent serial --dev /dev/ttyACM0 -b 921600 \
  >> ~/drone_logs/agent_manual.log 2>&1 &
sleep 10

# 1.4 看会话是否建立
sed "s/\x1b\[[0-9;]*m//g" ~/drone_logs/agent_manual.log \
  | grep -aE "create_(client|publisher|datawriter)" | tail -3
```

**通过**：10 s 内出现 `create_publisher … publisher created` → 会话活了，进 Step 2。
**没过**：只有 `init running` → **再等满 3 min**（PX4 重连可能慢）；仍无 → 查本手册 §3 的 L0 三项（双实例 / 串口占用 / 域号），必要时再断电一次。

### Step 2 · 验会话（4 条命令，缺一不可）

```bash
source /opt/ros/humble/setup.bash && source ~/dzpx4/install/setup.bash

# V1 发布者存在（最强判据）
ros2 topic info -v /fmu/out/vehicle_local_position --no-daemon | grep "Publisher count"   # >= 1

# V2 有数据流
timeout 10 ros2 topic hz /fmu/out/vehicle_local_position                                  # 稳定频率

# V3 PX4 状态可读
timeout 10 ros2 topic echo /fmu/out/vehicle_status --once                                 # 有输出

# V4 日志侧确认
grep -ac "create_publisher" ~/drone_logs/agent_manual.log                                 # >0
```

**全过** → 飞控链路恢复，可以起 `px4_odom_convert` / `px4_control`。
**V1=0 但 agent 活着** → 依次排除：① 两个 agent 抢串口（`fuser -v /dev/ttyACM0`）；② 看门狗还在旧节奏（`pgrep -af orin_agent_watchdog`）；③ **域号不一致**（`ROS_DOMAIN_ID` 与 PX4 的 `UXRCE_DDS_DOM_ID` 不等，见 Step 3）。
**注意**：`ros2 topic list | grep fmu` **不能当判据**——看门狗的订阅会让话题出现在列表里（详见 §0.1）。

### Step 3 · 决定是否切 118（可选，默认先不切）

| 你的需求 | 选择 | 动作 |
|---|---|---|
| 只做感知/定位/飞控本地验证，暂时不与小车互联 | **保持 0** | 什么都不用做；所有命令用默认域（不 export） |
| 要与小车同域（`/nav/*`、`/aim/*`、点云桥直连） | **切 118** | 按 [域号切换单_118.md](域号切换单_118.md) 执行：**先 PX4 参数 → 再 Orin 脚本 → 重启 agent → 复验** |
| 不确定 | **保持 0** | 切域会让飞控再重启一次，等于把 Step 1 重做一遍 |

**切 118 的前置**：必须先有一条能改 PX4 参数的通道（QGC 数传 / TELEM 口 USB-TTL）。没有这条通道就**不要切**——只改 Orin 一侧会直接失联。
**切完的验证**：`ROS_DOMAIN_ID=118 ros2 topic list | grep '^/fmu/out'` 有输出，`ROS_DOMAIN_ID=0 …` 无输出（判读矩阵见切换单 §5）。

### 三步卡的停止条件

- Step 1 连续 **3 轮**断电重启都建不起会话 → 停止盲目重试，按本手册 §3 L0 排雷（串口/双实例/域号），或换一台已知能连的机载电脑交叉验证；
- 任何时候出现"能连上但 30 s 内又掉" → 先查 USB 线/供电，再查 `dmesg` 枚举记录；
- **不要在会话死时反复手动 `pkill` agent**：旧看门狗已经把这个动作自动化成 61 s 死循环了；新版看门狗会在 3 轮后转 `FC_OFFLINE` 并停手（见 `_tmp_session/drone_arch/看门狗补丁说明.md`）。

---

## 0.1 30 秒判据表

| 判据 | 会话**活** | 会话**死** |
|---|---|---|
| agent 日志 | 有 `ProxyClient.cpp … create_publisher … publisher created`（**且时间在最近几秒**） | 只有 `TermiosAgentLinux.cpp init running` + `Root.cpp set_verbose_level logger setup` |
| DDS 图 | `ros2 topic info -v /fmu/out/vehicle_local_position` → **Publisher count ≥ 1** | **Publisher count = 0**（Subscription 可能是看门狗自己，别被骗） |
| 数据流 | `ros2 topic hz /fmu/out/vehicle_local_position` 有稳定频率（~10~50 Hz） | `hz` 无输出（挂住） |
| 时间戳 | agent 日志最后一个 `create_publisher` 距今 < 10 s | 最后一个 `create_publisher` 是几小时前 |
| 进程 | `MicroXRCEAgent serial` 1 个 | 0 个，或 >1 个（互抢串口） |

> ⚠️ **`ros2 topic list | grep fmu` 不是判据**：看门狗订阅了 `/fmu/out/vehicle_local_position`，即使 0 个发布者这个话题也会出现在列表里（本轮实测）。旧文档里"有 /fmu/out/* ⇒ 通了"的说法**不成立**。

---

## 1. 两个关键日志标记（看懂 agent 输出）

agent 日志是 eProsima 风格，**无人类可读时间**，时间戳是 Unix epoch 秒：

```
[1792228181.586811] info | TermiosAgentLinux.cpp | init | running... | fd: 3
[1792228181.588501] info | Root.cpp           | set_verbose_level | logger setup | verbose_level: 4
```

`date -d @1792228181 "+%F %T"` → `2026-10-17 17:09:41`

| 标记 | 含义 | 判读 |
|---|---|---|
| `TermiosAgentLinux.cpp … init … running...` | **串口打开了**（agent 侧成功） | 只说明 `/dev/ttyACM0` 能开，**不代表 PX4 在说话** |
| `Root.cpp … set_verbose_level … logger setup` | 启动参数生效 | 每次启动必有，无信息量 |
| `ProxyClient.cpp … create_client` | PX4 的 uXRCE client 建了会话 | **强正信号** |
| `ProxyClient.cpp … create_publisher / create_datawriter` | PX4 把 uORB 话题发布到 DDS | **强正信号**，会话活的铁证 |
| `ProxyClient.cpp … create_subscription / create_datareader` | PX4 订阅了 ROS 侧话题（如视觉里程计） | 强正信号 |
| `Error / timeout / session` 关键字 | 会话层错误 | 用 `grep -aiE "error|timeout"` 看 |

### 1.1 正/负样本（同一份日志里都能找到）

**活**（`~/drone_logs/node_agent.log`，2026-10-17 13:51 那一段）：

```
[1792216267.570884] | ProxyClient.cpp | create_publisher | publisher created | client_key: 0x00000001, publisher_id: 0x0DB(3)
[1792216267.571337] | ProxyClient.cpp | create_datawriter | datawriter created | …
[1792216267.607841] | ProxyClient.cpp | create_topic | topic created | …
```

**死**（本轮每次重启后都是这样，没有后续行）：

```
19:10:38 [1792235438.780024] | TermiosAgentLinux.cpp | init | running... | fd: 3
19:10:38 [1792235438.780459] | Root.cpp | set_verbose_level | logger setup | verbose_level: 4
（之后 61 秒无任何 ProxyClient 行，然后被看门狗杀掉重启）
```

### 1.2 本轮时间线（证据）

| 时间 | 事件 | 来源 |
|---|---|---|
| 19:03:30 | Orin 重启完成（uptime 起点） | `uptime -s` |
| 19:05:31 | 看门狗：遥测静默 60 s，重启 agent（第 2 次） | `node_wdog.log` |
| 19:06:32 / 19:07:34 / 19:08:35 / 19:09:36 / 19:10:37 / 19:11:39 | 每 ~61 s 一次，同样的"静默 60 s → 重启" | `node_wdog.log` |
| 14:59:38 | `agent_watchdog_agent.log` 里**最后一次** `create_publisher`（此后无会话） | `date -d @1792220378` |
| 13:51:07 | `node_agent.log` 里最后一次 `create_publisher` | `date -d @1792216267` |
| 19:10 实测 | `Publisher count: 0`，Subscription 1（`/fmu_agent_watchdog`） | `ros2 topic info -v` |

**结论**：agent 每轮都成功打开了串口（`init running`），但 **PX4 端从未发起/重建 uXRCE 会话**。这不是 agent 起不来，是**飞控侧客户端没连**。

---

> **勘误（19:2x）**：曾按 `node_agent.log` 里 16:51 的两行判断"当时会话是活的"——那两行其实是 `TermiosAgentLinux init running` + `Root.cpp logger setup`，**没有任何 `ProxyClient` 行**。全文件最后一次 `create_publisher` 是 **14:59:38**（`agent_watchdog_agent.log`，epoch 1792220378）与 **13:51:07**（`node_agent.log`，epoch 1792216267）。判断会话必须看 `create_publisher/datawriter`，不是看 `init running`。
>
> **待查线索**：14:59:38 这个时刻，车侧投影恰好在 14:59:38.2~39.2 跳变。两台机器独立供电，理论上不相关，但不能用"理论上"排除——现场请顺手确认当时是否有掉电/拔线/USB 重插（排查命令见 [域号切换单_118.md](域号切换单_118.md) §1.1）。

---

## 2. 为什么是 61 秒一轮（看懂看门狗）

`~/dzuav_deploy/air_ground/tools/orin_agent_watchdog.py` 逻辑：

```
每 2 s 巡检：
  设备(/dev/ttyACM0)不在      → 跳过（不空转重启）
  Agent 进程死了              → 立刻拉起
  遥测静默 > silence_sec(15)  → 重启 Agent
  距上次重启 < cooldown(60)   → 跳过（冷却）
```

重启完成后 `last_msg` 被重置 → 静默从 0 重新计时；但**冷却 60 s** 先到，所以实际周期 = **60 s 冷却 + 1 s 杀/起 ≈ 61 s**（日志里"遥测静默 60 s"是冷却结束那一刻的静默值，不是判据阈值 15 s）。

**含义**：只要 PX4 侧不重连，**旧版**看门狗会永远每 61 s 重启一次 agent。这不是故障源，是"会话未建立"的**持续表现**。每次重启都给 PX4 一个 60 s 的重连窗口。

> **已出补丁**：`_tmp_session/drone_arch/orin_agent_watchdog_fc_offline.py` 在连续 3 轮（约 3 min）仍无遥测后转 `FC_OFFLINE`、停止重启、只每 5 min 告警一次。说明与自测见同目录 `看门狗补丁说明.md`（未部署）。

---

## 3. 恢复阶梯（从最轻到最重）

### L0 · 只读确认（30 秒，先做）

```bash
# 1) 设备在不在
ls -l /dev/ttyACM* ; udevadm info -q property -n /dev/ttyACM0 | grep ID_MODEL

# 2) agent 是不是只有 1 个、谁占着串口
pgrep -af MicroXRCEAgent
fuser -v /dev/ttyACM0 2>/dev/null

# 3) 会话活没活（两种看法，缺一不可）
tail -5 ~/drone_logs/agent_watchdog_agent.log | sed "s/\x1b\[[0-9;]*m//g"
source /opt/ros/humble/setup.bash && source ~/dzpx4/install/setup.bash
timeout 12 ros2 topic info -v /fmu/out/vehicle_local_position --no-daemon | grep -E "Publisher count|Subscription count"
```

> 若 `fuser` 显示**两个**进程占 `/dev/ttyACM0`，先解决抢占（杀到只剩一个 agent），再谈恢复。

### L1 · 软重启 agent（不碰飞控）

```bash
# 先停看门狗，避免 61 s 后再被它重启（否则你观察到的是它的节奏，不是你的）
pkill -f orin_agent_watchdog

pkill -f "MicroXRCEAgent serial"
sleep 2
setsid nohup MicroXRCEAgent serial --dev /dev/ttyACM0 -b 921600 \
  >> ~/drone_logs/agent_manual.log 2>&1 &

# 观察 10 s
sleep 10
sed "s/\x1b\[[0-9;]*m//g" ~/drone_logs/agent_manual.log | grep -aE "create_(client|publisher|datawriter)" | tail -5
```

**判**：10 s 内出现 `create_publisher` → 会话恢复；只有 `init running` → 进 L2。
**为什么先停看门狗**：L1 与看门狗同时跑，日志会被反复截断，且 60 s 后它的重启会掩盖你的操作结果。

### L2 · 给会话更长窗口（仍然不碰飞控）

有时 PX4 的重连重试比较慢，60 s 窗口不够：

```bash
pkill -f orin_agent_watchdog
pkill -f "MicroXRCEAgent serial" ; sleep 3
setsid nohup MicroXRCEAgent serial --dev /dev/ttyACM0 -b 921600 >> ~/drone_logs/agent_manual.log 2>&1 &
# 等满 3 分钟，期间只观察
sleep 180
grep -ac "create_publisher" ~/drone_logs/agent_manual.log
```

**判**：180 s 内仍 0 条 → **PX4 侧客户端已经卡死，必须动飞控**（L3/L4）。

### L3 · PX4 侧软重启（需要 MAVLink 通道）

在 PX4 的 **MAVLink Shell / QGC MAVLink Console** 里执行（**本板未实测，命令语义来自 PX4 官方文档**）：

```text
uxrce_dds_client status
uxrce_dds_client stop
uxrce_dds_client start -t serial -d /dev/ttyACM0 -b 921600      # USB CDC 的情形
# 或走以太网（Orin 与飞控同网段时）：
uxrce_dds_client start -t udp -p 8888 -h <Orin_IP>
```

- PX4 官方文档明确 `uxrce_dds_client start` 可在 MAVLink Shell 里执行（示例 `uxrce_dds_client start -t udp -p 8888 -h 192.168.0.100 -n drone`）。
- 串口设备名 `/dev/ttyACM0`（PX4/NuttX 侧 USB CDC）**需要按本板固件的串口映射确认**——标「未验证」。
- 相关参数：`UXRCE_DDS_CFG`（把某个串口分配给 uXRCE-DDS，改完通常要重启 PX4）、`UXRCE_DDS_DOM_ID`（域号，**必须与 agent 的 `ROS_DOMAIN_ID` 一致**）。

> ⚠️ **本板现实约束**：USB CDC 已被 uXRCE 占用，PX4 的默认 MAVLink 很可能不在 USB 上（在某个 TELEM 口）。**如果没接数传/FTDI，就没有 MAVLink Shell 可用** → L3 走不通，直接 L4。这一点标「未验证」（无 MAVLink 通道无法确认 PX4 的 `MAV_0_CONFIG`）。

### L4 · 飞控断电重启（最可靠，也是本轮该做的）

**顺序很重要**：

1. 断飞控电源（拔电池 / 断 USB 供电），**等 5 s**；
2. 重新上电，等 USB 重新枚举（Orin 上 `ls -l /dev/ttyACM*` 应再次出现，`dmesg` 会有新的 `cdc_acm` 记录——本轮 `dmesg` 被内核权限挡住，未取到）；
3. Orin 侧重启 agent（`pkill -f "MicroXRCEAgent serial"`；看门狗若在跑会自动拉起，也可 `autostart_agup.sh start`）；
4. 10 s 内查 `create_publisher` 与 `Publisher count`。

> 依据（来自看门狗源码注释，10-16 实机结论）：**NxtPX4 断电重上电后串口会重新枚举，agent 持有的旧句柄失效 → 必须重启 agent 才能重建会话**。反过来同样成立：**Orin 重启而飞控没断电 → PX4 的 USB 侧状态残留 → 只有飞控断电才能清**。

---

## 4. 什么情况**必须**给飞控断电

| 场景 | 为什么必须断电 |
|---|---|
| **伴飞电脑重启，飞控一直带电**（本轮就是这个） | USB CDC 链路断开又重连，PX4 的 uxrce client 不会自动重建会话；agent 重启多少次都没用 |
| 改过 `UXRCE_DDS_CFG` / `UXRCE_DDS_DOM_ID` | 参数只在启动时生效，官方建议改完重启 PX4 |
| 两个 agent 抢过同一个串口 | 串口状态被搞乱，PX4 侧可能停在半连接 |
| PX4 侧无 MAVLink 通道（本板常态） | 软重启手段不可用，只剩断电 |
| agent 连续 3 轮重启（>3 min）都只有 `init running` | 软手段已证明无效 |

**不需要断电的情况**：只是 agent 进程被杀/崩了（`pgrep` 为空），重启 agent 即可；或者只是域号不一致（改环境变量重启节点即可）。

---

## 5. 恢复后的验证（必须全过）

```bash
source /opt/ros/humble/setup.bash && source ~/dzpx4/install/setup.bash

# V1 会话建立（最强证据）
sed "s/\x1b\[[0-9;]*m//g" ~/drone_logs/agent_watchdog_agent.log | grep -ac "create_publisher"   # >0 且时间在最近

# V2 发布者存在
ros2 topic info -v /fmu/out/vehicle_local_position --no-daemon | grep "Publisher count"          # >= 1

# V3 有数据流（期望 ~10~50 Hz，取决于 PX4 发布率）
timeout 10 ros2 topic hz /fmu/out/vehicle_local_position

# V4 PX4 状态可读（解锁/模式判断的基础）
timeout 10 ros2 topic echo /fmu/out/vehicle_status --once

# V5 外部视觉是否被接受（需 odom 桥在跑）
ros2 topic hz /fmu/in/vehicle_visual_odometry                                                    # 42~44 Hz
```

| 检查 | 通过标准 | 失败含义 |
|---|---|---|
| V1 | 最近 10 s 内有 `create_publisher` | PX4 侧仍没连 → L4 |
| V2 | Publisher count ≥1 | agent 在但 PX4 没发 → L4 |
| V3 | 有稳定频率输出 | 有发布者但无数据 → 域号/QoS 不一致或 PX4 卡死 |
| V4 | 打印出 `arming_state`/`nav_state` | — |
| V5 | 42~44 Hz | 桥没起 / mode 选错（看启动日志 `Mode:` 行） |

**再往后**（控制层）：`ros2 topic hz /fmu/in/offboard_control_mode` 20 Hz → 按 `0` 看是否切 Offboard → 按 `1` 看是否 ARM。

---

## 6. 不要做的事

| 不要 | 为什么 |
|---|---|
| `cat /dev/ttyACM0` / `head -c` 读串口"看有没有数据" | 会把字节从 agent 手里抢走，**制造**会话异常；串口是独占的 |
| 同时跑两个 agent | 抢串口，PX4 侧状态错乱（10-16 已实锤"双栈并行 93 s"事故） |
| `pkill -f MicroXRCEAgent` 后不管 | 看门狗 60 s 后会拉起新实例；若你在手工调试，先停看门狗 |
| 用 `ros2 topic list \| grep fmu` 判断链路 | 看门狗订阅会制造"话题存在的假象" |
| 改完域号只重启一半节点 | agent/watchdog/ROS 节点必须同域；只改一半 = 话题分裂 |
| 在 agent 活着时改 `b-` 波特率参数重连 | `-b 921600` 必须与 PX4 侧一致；不一致表现就是"串口开着但没会话" |

---

## 7. 与本轮结论的关系（交接说明）

- **现象**：19:03 Orin 重启 → 19:05 起看门狗每 61 s 重启 agent → 会话始终未建立 → `/fmu/out/vehicle_local_position` Publisher count 0。
- **已排除**：串口设备不存在（`/dev/ttyACM0` 在，`ID_MODEL=HKUST_UAV_NxtPX4`）、agent 进程起不来（每轮都 `init running`）、看门狗没跑（在跑，且计数在涨）。
- **最可能根因**：**飞控没断电**，PX4 的 uxrce client 停在旧 USB CDC 状态，不会自动重连。恢复动作 = L4（断电重启飞控）+ 重启 agent。
- **修复方向（不改机端，供决策）**：
  1. 把"Orin 重启后自动给飞控断电/重连"做成硬流程（人操作或可控 USB hub 断电）；
  2. 给 PX4 配一个可用的 MAVLink 通道（TELEM 口 + 数传/FTDI），让 L3 可用；
  3. 看门狗增加"连续 N 轮未建会话 → 停止重启并报警"（避免 61 s 无效循环掩盖真实故障）；
  4. 域号统一（见架构总览 §6），避免 PX4 `UXRCE_DDS_DOM_ID` 与 agent 域号错配。

---

## 8. 未验证项

1. PX4 固件版本、`UXRCE_DDS_CFG`/`UXRCE_DDS_DOM_ID` 实际值（无 MAVLink 通道，读不到）。
2. `uxrce_dds_client start -t serial -d /dev/ttyACM0` 在本板的设备名是否就是 `/dev/ttyACM0`。
3. USB 被 uXRCE 占用后 MAVLink 是否还能复用（因此 L3 是否可行）。
4. `dmesg` 中 USB 重新枚举的可见性（本轮 `dmesg` 无输出，疑似内核权限限制）。
5. 断电重启飞控后会话恢复的**本轮实测**（本轮按只读边界未执行，交由现场操作）。
