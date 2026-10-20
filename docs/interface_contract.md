# 车机接口契约

> **Agent-1 车机协同工作任务协同 出品**。
> 本文档定义无人机（dzuav）与无人车（ag_integrate）之间的所有接口，确保双方对数据格式、语义、时序理解一致。
> **任何接口变更必须在此文档同步更新，并通知对方。**

---

## 1. 通信链路

```
[无人机 Orin]  dzuav ──UDP:50000──→ 机载自主规划 orin_plan_sidecar（同机消费，不出机）
                                        │
                                        └── 0x4003 车命令 ──UDP:50000──→ ag_integrate
                                                                          [无人车 香橙派]
                                                                          └→ /nav/waypoint
```

| 项 | 值 |
|---|---|
| 协议 | 二进制 UDP |
| 端口 | 50000 |
| 方向 | 无人机 → 无人车（单向广播） |
| 频率 | ~10 Hz（每帧处理完即发） |
| 字节序 | 小端（little-endian） |
| 消息 | `0x4001` 情报帧 / `0x4002` 车位姿帧（dzuav 发）；`0x4003` 车命令帧（**机载规划**发） |

> **规划位置（2026-10-20 起）**：官方控制要求「无人机的定位、建图、识别、规划与控制等实时
> 任务须由机载平台自主完成」→ 规划层在机上跑，产物为 `route_plan.json / cell_matrix.json /
> planned_path.txt`，车命令经 `0x4003` 广播到车端。车端 `integrate_node` 的
> `car_command_source=onboard` 即薄接收模式（默认 `local` 为本机自行规划，二者不可同时消费
> 同一 UDP:50000）。

---

## 2. UDP 帧格式

### 2.1 情报帧（msg_id = 0x4001）

无人机检测到的目标列表（含地雷、靶标、格号、车等）。

```
偏移  大小  字段        说明
─────────────────────────────────────────
0     4     magic       0xefefefef
4     1     version     0x10
5     8     timestamp   uint64，机上毫秒时间戳
13    2     msg_id      0x4001
15    4     length      uint32，= 2 + 25 * count
19    2     count       uint16，目标数量
21    25*N  objects     目标数组（见 §2.3）
21+25N 2   crc16       CRC-16/Modbus，覆盖 version 到最后一个目标
...   4     suffix      0xfefefefe
```

### 2.2 车位姿帧（msg_id = 0x4002）

无人机观测到的车的真实位姿，用于车端朝向对齐（根除画圆）。

```
偏移  大小  字段        说明
─────────────────────────────────────────
0     4     magic       0xefefefef
4     1     version     0x10
5     8     timestamp   uint64，机上毫秒时间戳
13    2     msg_id      0x4002
15    4     length      uint32，= 20（固定）
19    8     x           float64，车的世界坐标 x（米）
27    8     y           float64，车的世界坐标 y（米）
35    4     yaw         float32，车的真实朝向（弧度，场地坐标系）
39    2     crc16       CRC-16/Modbus
41    4     suffix      0xfefefefe
```

### 2.3 目标对象（25 字节）

```
偏移  大小  字段        说明
─────────────────────────────────────────
0     1     type        uint8，目标类别（见 §2.4）
1     8     x           float64，世界坐标 x（米）
9     8     y           float64，世界坐标 y（米）
17    4     length      float32，像素语义：检测框宽（并非米；车端暂不使用）
21    4     width       float32，像素语义：检测框高（并非米；车端暂不使用）
```

### 2.4 目标类别枚举（13 类，canonical）

⚠️ **2026-10-16 起为 D 方案 13 类统一模型**（`hbb_d13_best.pt`，17 通道引擎）。
本表顺序**必须**与 `onboard/dzuav/include/dzuav/types.hpp` 的 `ObjectClass`
及 `src/ag_integrate/ag_integrate/intel_protocol.py` 的 `ObjectType` 逐项一致；
改动任一侧都必须三处同改，否则会出现「索引错位 → 把障碍当蓝靶开火」这类事故。

| 值 | 名称 | 说明 | 规划侧处理 |
|---|---|---|---|
| 0 | PAPER | 场地纸片（雷图卡 / 桥顶箭头） | **落格判语义**：K 列 = 通行箭头，其余格 = 地雷 |
| 1 | CELL_LABEL | 地面格号 | PnP 定位用，不进态势 |
| 2 | CAR | 无人车 | 车位姿链（0x4002），不进态势 |
| 3 | TARGET_RED | 红色友方靶 | **禁火**（误击 −5），粘性否决 TTL 内持续参与 |
| 4 | TARGET_BLUE | 蓝色敌方靶 | **打击目标**（标 `*`） |
| 5 | OBSTACLE | 障碍物 | 路径避障（多帧确认 N=2） |
| 6 | BUCKET_RED | 红桶 | 展示/避让，禁火 |
| 7 | BUCKET_GREEN | 绿桶 | 展示/避让 |
| 8 | BUCKET_CYAN | 青桶 | 展示/避让 |
| 9 | CAR_START | 智能车起点 | 展示 |
| 10 | CAR_END | 智能车终点 | 展示 |
| 11 | UAV_START | 无人机起点 | 展示 |
| 12 | UAV_END | 无人机终点 | 展示 |

### 2.5 车命令帧（msg_id = 0x4003）

**机载规划 → 车端**。规划层在机上（官方控制要求），reset/route 不能再靠车端本机的
`/nav/waypoint` 传递，故沿用既有空地 UDP 广播把两条命令原样运到车端；车端
`integrate_node`（`car_command_source=onboard`）校验后转发到 `/nav/waypoint`。

```
偏移  大小  字段        说明
─────────────────────────────────────────
0     4     magic       0xefefefef
4     1     version     0x10
5     8     timestamp   uint64，机载毫秒时间戳
13    2     msg_id      0x4003
15    4     length      uint32，= (2+len(reset)) + (2+len(route))
19    2     reset_len   uint16
21    R     reset       ASCII，车端方言（"reset:<格号>,<车端朝向 token>"；空串=本局不发锚定）
21+R  2     route_len   uint16
23+R  L     route       ASCII，空格分隔格号，`*`=停靠射击，末尾 ,x/,y=终点朝向
...   2     crc16       CRC-16/Modbus，覆盖 version 到最后一个命令字节
...   4     suffix      0xfefefefe
```

结构约束（`parse_car_command` 全量校验，任一不符整条丢弃，绝不半成品上底盘）：
`length` 与实际字节一致；两段长度自洽；**纯 ASCII**（含非 ASCII 或 NUL 即拒绝）；
`reset` 为空或必须以 `reset:` 开头；`route` 非空；CRC 与 suffix 正确。

---

## 3. 坐标系约定

| 项 | 定义 |
|---|---|
| 原点 | 场地角点 **N11 = (0.5, 0.5)**（格单位） |
| 单位 | **米**（UDP 中）/ **格**（车端内部，1格=0.35m） |
| x 轴 | `x = (11 − 行号) + 0.5`，范围 [0.5, 10.5] |
| y 轴 | `y = (14 − 列序号) − 0.5`，范围 [0.5, 13.5] |
| 朝向 | yaw 弧度，场地坐标系；0 = 朝 +x，π/2 = 朝 +y |
| 格号世界坐标 | `x = 0.065 + (col-'A')*0.35`, `y = 0.295 + (11-row)*0.35`, `z=0` |

---

## 4. 车端命令格式

车端通过 `/nav/waypoint` 话题向官方 `path_executor` 发送字符串命令。

### 4.1 复位命令

```
reset:<格号>,<朝向>
```
- 例：`reset:N10,y`
- 格号：字母 A~N + 数字 1~11
- 朝向：`x` 或 `y`（车的初始朝向）
- **每局发车前必须先发一次**

### 4.2 路线命令

```
<格号1> <格号2> ... <格号N>,<终点朝向>
```
- 例：`G8 A10 D2* E2,y`
- 空格分隔格号
- `*` 后缀 = 停靠射击位（到达后停车瞄准开火）
- 末尾 `,x` / `,y` = 终点朝向，**逗号必须紧贴最后一个格号**；
  逗号前多留空格会被车端判非法、整条丢弃（探测记录证实）

---

## 5. 时序约定

### 5.1 正常流程

```
无人机                    无人车
  │                          │
  │── UDP 0x4002 (车位姿) ──→│  align模式：原地转向到start_heading
  │                          │  （零残差后停止转向）
  │                          │
  │── UDP 0x4001 (目标情报) ─→│  解析目标→世界模型→路径规划
  │                          │
  │                          │── /nav/waypoint "reset:N10,y"
  │                          │── /nav/waypoint "G8 A10 D2* E2,y"
  │                          │
  │                          │  车开始行驶
```

### 5.2 朝向对齐（align 模式）时序

```
无人车启动 integrate_node（heading_bias_source=align）
  │
  ├─ ⚠️ 前置条件（2026-10-15 源码实测契约）：align 必须在 path_executor
  │   启动之前完成。path_executor 一旦运行即以 20Hz 在 pursuitAngle 上
  │   保活，dzactuator 模式仲裁被钉死在模式2(Pursuit)，/cmd_vel 回调
  │   首行静默丢弃（current_mode_!=2 时 pursuit 同理）；且 Ackermann 车
  │   v==0 时舵机直接回中——原地转向命令必须带爬行速度。
  │   标准流程用 tools/car_match_bringup.sh 两阶段启动。
  │
  ├─ 门控（雷/靶面就绪）通过后启动朝向对齐，目标 = start_heading 断言方向
  │
  ├─ 每 100ms tick：读最新 0x4002 帧 yaw，闭环发布 /cmd_vel
  │   Twist{linear.x=align_creep_speed(默认0.05m/s), angular.z=w(rad/s)}——
  │   dzactuator 模式1下按 atan(CARL·w/v) 映射舵机，实际 yaw 率即 w
  │   （最大角速度 0.5 rad/s，对齐容差 0.05 rad ≈ 3°；
  │     尚未收到 0x4002 帧时保持停车等待，无单独的等待超时）
  │
  ├─ 连续 3 tick 进入容差 → 对齐完成，停止转向（此后不再发 /cmd_vel，
  │   0.5s 后 pursuit 通道独占、模式自动切回 2）
  │
  ├─ wait_waypoint_consumer（默认开）：等到 /nav/waypoint 出现订阅者
  │   （path_executor 上线）才发 reset + 路线——话题无滞留，发早整条丢
  │
  └─ 对齐总耗时超过 heading_align_timeout（默认 15s）→ 见 §5.3
```

### 5.3 超时保护

align 模式只有**一个**超时参数 `heading_align_timeout`（默认 15s，
`config/match_params.yaml` 同值），覆盖"等 0x4002 + 闭环转向"全过程。

| 情况 | 实际行为 |
|---|---|
| 对齐 15s 内未完成（含迟迟收不到 0x4002） | 停止转向、报警，**退回最近轴断言**（取 0x4002 观测朝向最近的 x/y 轴，与 udp 模式同逻辑；一帧都没收到过则退回 start_heading），仍尝试发车 |
| UDP 0x4001 情报帧中断 | world_model **无限沿用最后一帧快照**（无陈旧度上限），不主动停车；建议现场关注链路中断时的车端日志 |

---

## 6. 禁火保险

- **只有 TARGET_BLUE（类号 4）才允许开火**
- TARGET_RED（类号 3）绝对禁止开火（误击 -5 分）
- OBSTACLE 是类号 **5**（不是蓝靶）；类号一律以 §2.4 表与
  `onboard/dzuav/include/dzuav/types.hpp` / `intel_protocol.py` 的 `ObjectClass`
  枚举为准。本行原写"TARGET_BLUE（类号5）"是 7 类时代的残留索引，
  照它实现会把障碍物当蓝靶打。
- 无靶标时不下发开火命令
- 代码位置：`src/ag_integrate/ag_integrate/fire_guard.py`

---

## 7. 路径约束

| 约束 | 说明 |
|---|---|
| 中场挡板 | 位于第 6/7 行之间的边界线上（`_MID_BOUNDARY=6`，0 基 row5↔row6；上半场=行1~6、下半场=行7~11），只覆盖 D~N 列；**A/B/C 列无挡板，可自由穿越半场** |
| 桥洞列动态推导 | 桥洞列**不写死**：由 ARROW（可通行标记）识别结果动态推导，每局随机；桥洞列内只有箭头格 + 朝向挡板一侧的 1 格引道可通行，其余格视为墙（桥洞列南北不连通，跨半场只能走桥洞缺口或 A/B/C 列） |
| 桥洞正交贴门 | 穿越中线的移动限定正交且贴着可穿越格，禁止斜穿（防止剐蹭挡板） |
| 靶格不可踩 | 需在正交相邻格停靠瞄准射击 |
| 移动靶轨道 | D8↔E8，蓝面恒定不翻面 |
| 路径点压缩 | 删除同向共线中间格，保留拐点和必经点 |

> ⚠️ **cross_gates 陷阱**：`cross_gates` 一旦显式传入（非空），自动推导
> 整体关闭——**A/B/C 列的自由穿越也一并失效**，只有列出的 gate 格允许
> 穿越中线。留空（默认 `[""]`）才是自动推导：A/B/C 列 + 桥洞缺口都可穿越。

---

## 8. 接口变更记录

| 日期 | 变更 | 影响 | 通知对象 |
|---|---|---|---|
| 2026-10-09 | 初始版本 | 定义全部接口 | Agent-3, Agent-4 |
| 2026-10-14 | 文档对齐代码现状：路线命令逗号紧贴格号（§4.2）、§7 桥洞拓扑按 mission_router 重写（含 cross_gates 陷阱）、§5.2/§5.3 对齐超时改为单一 15s、§2.3 length/width 改注像素语义 | 仅文档修正，字节布局与命令语法不变 | Agent-3, Agent-4 |
| 2026-10-15 | §5.2 循迹契约修正：align 必须先于 path_executor 完成（dzactuator 模式仲裁实测）、对齐命令带爬行速度（v≠0 舵机才生效）、新增 wait_waypoint_consumer 闸门 | 车端行为变化：对齐走 cmd_vel 爬行转向；两阶段启动 tools/car_match_bringup.sh；字节布局不变 | Kimi Code |
| 2026-10-15 | §7 中场挡板行号修正：第 5/6 行 → 第 6/7 行（0 基 row5↔row6，上半场行1~6、下半场行7~11），按车上官方地图真源；代码侧见提交 3e87f95 | 仅文档修正，接口不变 | Agent-3, Agent-4 |
| 2026-10-15 | 新增 §9：车上现役默认核心 dzbrain 的无人机情报接口（/dzlink JSON 双话题）与我方桥接节点 dzbrain_bridge | 新增可选链路（mode:=dzbrain）；legacy 链路不变 | Kimi Code |

---

## 9. dzbrain 链路（车上现役默认核心的情报接口）

> 2026-10-15 起入库。真源：车上 `~/dzrobot/src/dzbrain/dzbrain/tasks/drone.py`
> （只读快照 `reference/car_src_20261015/dzbrain/`）。legacy 链路（§1~§8）不变，
> 两模式互斥：launch 侧 `mode:=legacy`（默认）或 `mode:=dzbrain` 二选一。

**话题**（std_msgs/String，JSON 文本）：

| 方向 | 话题 | 说明 |
|---|---|---|
| 我方 → 车 | `/dzlink/drone_to_car` | 无人机情报（dzbrain DroneTask 消费） |
| 车 → 我方 | `/dzlink/car_to_drone` | ack 回执 + car_state 周期上报（1~2Hz） |

**报文**（字段以 DroneTask 解析逻辑为准；带 `seq` 的报文会收到 `{"type":"ack","of":N}` 回执）：

| type | 载荷 | 说明 |
|---|---|---|
| `heartbeat` | `{type, ts, seq}` | 心跳；丢失 >3s 判离线（`drone.heartbeat_timeout_s`）。我方 1Hz 恒定发 |
| `bridge_passable` | `{type, half, block, confidence, ts, seq}` | 桥洞可通行格（ARROW，无人机是权威）；`half`：lower=行7~11、upper=行1~6（挡板在 6/7 行界） |
| `target_zone` | `{type, blocks:[...]}` | 靶区格号列表（红+蓝都报），车上与本地 map.yaml 核对 |
| `field_verify` | `{type, mines:[...], obstacles:[...]}` | 雷/障格号核对；空列表="全场干净"（有效情报） |

车上消费端韧性约定：未知 type/非法 JSON/缺字段一律记日志不崩；情报过期
>`drone.info_stale_s`（30s）自动失效（`bridge_of()` 返回 None）。
我方实现：`src/ag_integrate/ag_integrate/dzbrain_bridge.py`（变化即发 + 2s 周期刷新；
任何异常不崩；不驱动车、无 dry_run 概念）。车→机回包（ack/car_state）当前只记日志。

---

*最后更新：2026-10-15 双 §8 去重（变更记录保全量表，§8 复位到 §9 之前）；无接口变更*
