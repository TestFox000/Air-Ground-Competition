# 离线测试指南（不接无人机，模拟侦察数据）

本文说明如何**在没有真无人机**的情况下，驱动车端的排雷、桥洞穿越、
云台打击等功能，以便在实验室里反复验证。

---

## 1. 原理

```
  你的地图 YAML
        │
        ▼
  tools/drone_sim.py  ── 用 test/air_protocol.py 编码（与 C++ 侧逐字节一致）
        │
        ▼  UDP（空地协同协议 v2：A5 C3 | VER TYP SEQ LEN | PAYLOAD | CRC16-CCITT）
        │
        ▼
  车端 CmdLink（transport: "udp"）
        │
        ▼  /mission_cmd（尾部透传原始载荷）
        ▼
  MissionManager ── 领域校验 ──▶ FieldGrid / MinePlanner / BridgeTracker / TargetAimer
```

**为什么编码要复用 `test/air_protocol.py`**：它是与 C++ 侧做过
逐字节对撞的参考实现（golden 向量见 `test/air_protocol_golden_check.py`）。
这样模拟器发出的帧与真无人机**完全一致**，不会出现"模拟器通过、真机不通"。

---

## 2. 快速开始

### 2.0 一键启动（推荐）

车端**另开一个终端**启动，然后跑：

```bash
# 终端 A：车端
source /opt/ros/humble/setup.bash && source ~/ws/install/setup.bash
ros2 launch dzmission bringup_mission.launch.py

# 终端 B：假传感器 + 地图模拟器（一键）
bash "/mnt/c/Users/Ran/Desktop/无人车/src-car/offline_test.sh"
```

`offline_test.sh` 会：检查环境 → 起 `fake_sensors.py` → 等 2s → 起 `drone_sim.py` →
前台保持运行（Ctrl-C 一起停掉）。两个子进程日志在 `/tmp/fake_sensors.log`、
`/tmp/drone_sim.log`。

**可选参数**（环境变量）：

```bash
FACE=1 bash offline_test.sh          # 靶标显示红面（测禁火）
DRIVE=1 bash offline_test.sh         # 让"车"按 /cmd_vel 真的动（推进里程）
PERIOD=2 bash offline_test.sh        # 靶标往复更快（测提前量）
bash offline_test.sh ~/ws/src/dzmission/tools/maps/example_redblue.yaml
```

### 2.0.1 为什么需要"假传感器"

车端的设计是**失明即停**：

```
没有 IMU/odom  ->  定位不健康   ->  ESTOP
没有视觉       ->  近场感知超时 ->  按急停处理
```

纯离线时车端会**一直停在 ESTOP**，看不到排雷规划等输出。
`fake_sensors.py` 发布三类话题把它"喂"到健康状态：

| 话题 | 类型 | 频率 | 车端用途 |
|---|---|---|---|
| `/imu_data` | `sensor_msgs/Imu` | 100 Hz | ESKF 预测 + **地图偏航来源** |
| `/odom` | `nav_msgs/Odometry` | 50 Hz | **距离**（沿偏航积分）+ 堵转判定 |
| `/offset_center` | `std_msgs/Int32MultiArray` | 15 Hz | 近场避碰 + 云台瞄准 + 面/红点 |

**车端定位原理**（决定必须发什么）：

```
位置 = odom 相邻采样距离，沿 IMU 地图偏航角积分
```

所以 `odom.twist.twist.linear.x` 与 `imu.orientation` 是**必需**的。

#### ★★ 还有一个隐藏前提：车必须"在动" ★★

车端的定位补偿有这一步（`localization_compensator.cpp` 的 `odomCallback`）：

```cpp
step = |odom 相邻采样位移|
if (step >= dr_min_step_m(0.5mm) && step <= dr_max_step_m(0.5m) && yaw_offset_valid_)
    -> pose_valid_ = true        // 任务层据此认为"定位可信"
```

**即：位置只有检测到「有效位移」才会被置为可信。**

因此如果假传感器让位置**恒定不动**（`step` 恒为 0）：

```
pose_valid_ 永远 false  ->  localization unhealthy  ->  一直 ESTOP
->  拒绝所有 NAV_TO / ENGAGE
```

**这正是第一版的表现**：四类侦察数据都收到了（日志里有"地雷列表：收到 8 条"），
但车端始终 `ESTOP 中拒绝 NAV_TO`。

**所以 `fake_sensors.py` 默认开一个很小的「怠速运动」**：

| 参数 | 默认 | 效果 |
|---|---|---|
| `--idle-v` | 0.05 m/s | 50Hz 下每 tick 位移 **1.0 mm**，稳过 0.5mm 门槛 |
| `--yaw-rate-deg-s` | 2.0 °/s | 缓慢偏航，让航向也在变 |

位置飘出 2 m 会自动掉头，不会跑远。

**这不是 bug，是必需的** —— 要模拟"完全静止"用 `--no-idle-motion`
（那时车端会停在 ESTOP，属预期，可用于验证急停逻辑）。

**手动启动**（不想用一键脚本时）：

```bash
python3 ~/ws/src/dzmission/tools/fake_sensors.py --face 2
python3 ~/ws/src/dzmission/tools/drone_sim.py --map <地图>
```

> ★ **与真机的差异**：这里发的是理想值（无零偏、无噪声、里程精确、
> bbox 按几何精确算出）。所以本工具验证**逻辑正确性**（状态机、规划、
> 联锁、协议），**不能**验证精度与抗噪能力。

### 2.1 车端改成 UDP 模式

编辑 `dzmission/params/dzmission.yaml`：

```yaml
cmd:
  transport: "udp"          # ← 原来是 "loopback"
  udp_bind_host: "0.0.0.0"
  udp_bind_port: 9100       # 车端监听端口
  udp_peer_host: "127.0.0.1"  # 模拟器所在机器（本机测试就填 127.0.0.1）
  udp_peer_port: 9101
```

> ⚠️ **要改 Windows 侧的文件**（`src-car/dzmission/params/dzmission.yaml`）。
> `build_ws.sh` 会从那里拷贝到 `~/ws/src/`，只改 WSL 里的会被下次编译覆盖。
>
> ⚠️ `loopback` 模式**不接收外部数据**，只用内部合成的假场景。
> 要收模拟器的数据，**必须改成 `udp`**（或 `serial`）。

### 2.2 启动车端

```bash
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash
ros2 launch dzmission bringup_mission.launch.py
```

启动横幅里会打印自检提示，确认：
- `cmd.transport` 是 `udp`
- **未启动** `dzrknn_dataprocess`（它会抢云台/抢激光）

### 2.3 先干跑（强烈建议）

**先不发送**，只看编码结果对不对：

```bash
python3 ~/ws/install/dzmission/share/dzmission/tools/drone_sim.py \
    --map ~/ws/install/dzmission/share/dzmission/tools/maps/example_standard.yaml \
    --dry-run --verbose
```

应该看到每帧的十六进制 + `解码 OK`（解码会校验 CRC、版本、长度）。

### 2.4 实际发送

```bash
python3 ~/ws/install/dzmission/share/dzmission/tools/drone_sim.py \
    --map .../tools/maps/example_standard.yaml
```

### 2.5 观察车端反应

另开终端：

```bash
# 看收到的原始帧（含 v2 帧头）
ros2 topic echo /mission_cmd_raw

# 看解析后的指令（前 16 字节是标量，[16..] 是侦察载荷）
ros2 topic echo /mission_cmd

# 看车端日志（排雷规划、门位、红蓝联锁都在这里打印）
ros2 launch dzmission bringup_mission.launch.py     # 或直接看该终端
```

---

## 3. 地图文件怎么写

**你只需要改坐标和参数。** 完整示例见 `tools/maps/` 下三个文件。

### 3.1 坐标约定

| 项 | 说明 |
|---|---|
| **列 col** | `0=A, 1=B, … 10=K(桥洞列), 12=M(起点), 13=N(终点)` |
| **行 row** | `0=最北, 5=隔离线上方, 6=隔离线下方, 10=最南` |
| **单位** | 一律**格坐标**，不是米（脚本内部换算） |

例：`K4` 门位 → `row=3`；`M11` 起点 → `col=12, row=10`。

### 3.2 字段一览

```yaml
name: "场景名称（仅用于显示）"

field:                      # 场地固定结构；可省略（用 v5 默认值）
  cols: 14
  rows: 11
  cell_cm: 35
  barrier_up_row: 5         # 隔离线在 row6/row7 之间
  barrier_col_lo: 3         # 隔离线覆盖列 D..N
  barrier_col_hi: 13
  bridge_col: 10            # 桥洞列 K
  bridge_a: [0, 5]          # 区段一 row 0..5
  bridge_b: [6, 10]         # 区段二 row 6..10
  start: [12, 10]           # 起点 M11
  goal: [13, 5]             # 终点 N6
  team_color: 1             # 抽签决定

mines:                      # 地雷 [col, row, conf%]    ★ 每颗 5 分，共 40
  - [3, 2, 90]

obstacles:                  # 障碍 [col, row, color]
  - [1, 1, 1]

targets:                    # 打击区 [col, row, face, conf]
  - [6, 4, 2, 95]           # face: 0未知 1红/友 2蓝/敌

gates:                      # 桥洞门位 [bridge_id, gate_row, conf, flags]
  - [1, 3, 100, 1]          # 区段一 K4      ★ 每区段 10 分，共 20
  - [2, 8, 100, 1]          # 区段二 K9

commands:                   # 任务指令序列（按 t 秒下发）
  - {t: 2.0, cmd: nav_to, col: 3, row: 2}
  - {t: 8.0, cmd: engage, col: 6, row: 4, face: 2}
```

**可选时间参数**（不写就用默认）：`t_mines` `t_obstacles` `t_targets`
`t_gates` `t_map_full` `heartbeat_interval_s`。

### 3.3 任务指令可选值

| cmd | 需要字段 | 说明 |
|---|---|---|
| `nav_to` | `col, row`（可选 `timeout_s`） | 前往某格 |
| `engage` | `col, row`（可选 `face`） | 进入交战 |
| `hold` | — | 原地待命 |
| `stop` | — | 立即停车 |
| `return_base` | — | 返航 |
| `bridge_cross` | `bridge_id` | 跨桥（★ 车端自主判定过门，该码只记日志） |
| `nop` | — | 空操作 |

---

## 4. 命令行选项

```
--map PATH        地图文件（必需）
--host IP         车端 IP，默认 127.0.0.1
--port N          车端 UDP 端口，默认 9100
--dry-run         只打印不发送（先确认数据对不对）
--loop            循环重发（车端掉线重连时有用）
--speed 2.0       时间轴倍速（默认 1.0 按地图里的 t 秒发）
--verbose         打印每帧完整十六进制
```

---

## 5. 测试建议顺序

| 顺序 | 地图 | 验证什么 | 期望 |
|---|---|---|---|
| 1 | `example_standard.yaml` | 全链路基本通 | 14 帧全收到；排雷计划生成；两个门都认出 |
| 2 | **`example_redblue.yaml`** | ★ **红蓝面禁火** | `face=1` 时**一发不打** |
| 3 | `example_edge.yaml` | 健壮性 | 越界被拒、单门拒绝跨桥 + 每 2s 上报 `NEED_GATE` |

### ★ 第 2 项最重要

`example_redblue.yaml` 里把 `targets` 的 `face` 在 `0/1/2` 之间切换，
**每种都要确认车端行为**：

| face | 含义 | 车端必须 |
|---|---|---|
| `2` | 蓝面（敌） | **开火** |
| `1` | 红面（友） | **绝对禁火**（打了扣 5 分/次，3 次判毁） |
| `0` | 未知 | 按 `aim.face_unknown_policy`（默认 `0` = 禁火） |

**这是本赛题唯一"判错就扣分"的功能**，务必优先测。

---

## 6. 常见问题

### 车端收不到数据

1. **`cmd.transport` 还是 `loopback`？** → 改成 `udp`
2. **端口不对？** 模拟器 `--port` 必须等于车端 `cmd.udp_bind_port`
3. **防火墙 / 跨机测试？** `--host` 填车端实际 IP
4. **看车端日志有没有** `[CmdLink] LOOPBACK mode` —— 有的话就是还在 loopback

### 帧被拒（`bad air frame`）

模拟器 `--dry-run --verbose` 会自己解码一遍。**若 dry-run 显示"解码 OK"，
说明帧合法**，那问题在车端参数或接收路径。

### 想让车端"以为"没收过某类数据

直接在地图里**删掉那一节**（例如删 `gates:`）。车端会走到"信息不完整"分支。

---

## 7. 与真无人机的差异

| 项 | 模拟器 | 真无人机 |
|---|---|---|
| 传输 | UDP | 取决于 H1 决定（UDP / 数传串口） |
| 链路层 | 无心跳重传（只发一帧心跳） | 有心跳、ACK、重发 |
| 数据真实性 | 你指定的 | 视觉识别结果 |

**因此模拟器不能替代**：
- 链路质量测试（丢包、延迟、重传）
- 视觉识别准确率
- 无人机端 H1/H2/H3 的传输配置

这些必须等真机联调。
