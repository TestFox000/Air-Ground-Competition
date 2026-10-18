# 实车验证清单（分阶段，每步都有"期望值"和"不对怎么查"）

> 适用：v5 拓扑、`dzmission` 编排层、移动靶方案 α+β。
> 全部数值来自代码与离线验证脚本，**不是估值**；标注"待实测"的除外。
>
> ★ 使用方式：**从上往下逐阶段做，不要跳步**。
>   每个阶段都有"通过标准"；不通过就停在该阶段，别往下走 ——
>   因为下层依赖上层的正确性，越往下越难定位。

---

## 阶段 0：编译与离线自检（不需要硬件，10 分钟）

### 0.1 编译

```bash
cd ~/your_ws
colcon build --packages-select dzmission --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

**通过标准**：无 error。警告可暂时忽略。

**★ 若编译失败，优先看这几处**（最近改动集中在这里）：

| 报错位置 | 可能原因 |
|---|---|
| `mission_manager.cpp` 的 `air::enc::cellDecode` / `air::enc::event` | 命名空间或参数个数 |
| `target_aimer.cpp` 的 `evaluateLeadFire` | `last_track_velocity_px_s_` / `vel_hist_` 定义顺序 |
| `bridge_tracker.cpp` 的 `selfCheck()` | 返回类型/头文件 include |
| `field_grid.cpp` 的 `markMineByRecon` 等 | `kBlocked` / `kMine` / `kTarget` 枚举名 |

### 0.2 离线自检（全部必过）

```bash
# C++ 自测：桥洞过门判定（不需要 ROS、不需要硬件）
ros2 run dzmission bridge_selftest

# 协议自测
ros2 run dzmission air_protocol_selftest

# Python 验证脚本（17 个，全过才算数）
cd src/dzmission/test
for f in *.py; do echo "--- $f"; python3 $f || echo "FAIL: $f"; done
```

**通过标准**：
- `bridge_selftest` 打印 `通过 N，失败 0`，且 30 种门位组合全过
- 17 个 Python 脚本全部 exit 0

**★ 关键**：这一步能在**完全没有硬件**的情况下验证最危险的逻辑（过门判定、红蓝联锁、协议编解码）。**不要跳过。**

---

## 阶段 1：单节点启动（确认没有"起不来"）

### 1.1 只起编排层

```bash
ros2 launch dzmission bringup_mission.launch.py use_hardware:=false
```

**通过标准**：
```
[Mission] BridgeTracker 自检通过（30 种门位组合）
```

**★ 为什么先看这一行**：它证明 `FieldGrid` + `BridgeTracker` 的规格自洽。
若这里失败，后面所有规划都不可信。

### 1.2 确认它是唯一的指令发布者

```bash
ros2 topic info /cmd_vel --verbose     # 发布者应只有 /dzmission
ros2 topic info /pursuitAngle --verbose
```

**通过标准**：各只有 1 个发布者。

**★ 若有两个**（例如 `dztrack` 或 `dzrknn_dataprocess`）：
→ **不要继续**。两个节点抢同一执行器会表现为"抖动/有时不动"，
现场极难定位。先解决归属问题（见 `doc/INTEGRATION.md`）。

---

## 阶段 2：视觉链路（不需要动，只需看数据）

### 2.1 相机 → 检测

```bash
ros2 topic hz /image_raw               # 期望 ~30 Hz
ros2 topic hz /offset_center           # 期望 ~15 Hz
ros2 topic echo /offset_center --once
```

**通过标准**：`/offset_center` 的 `data` 有 **10 个整数**：

| 下标 | 含义 | 期望 |
|---|---|---|
| `[0]` | 靶标中心相对画面中心的 x 偏移 | 有靶时在 ±320 内 |
| `[1]` | 同上 y | 在 ±240 内 |
| `[2]` | 类别 | 检测到靶标时 **> 0** |
| `[3]` | 距离（cm） | **0 = 未知**（正常，模型不输出距离） |
| `[4..7]` | bbox x/y/w/h | w、h 均 > 0 |
| `[8]` | **可见面** | **0=不确定 1=红(友) 2=蓝(敌)** |
| `[9]` | **激光红点** | 0 或 1 |

**★ 若只有 3 个元素** → 说明跑的是旧版 `det_node`，红蓝联锁与红点检测**全部失效**。
**必须重新 build `rknn_pt`**。

### 2.2 ★★ 红蓝判色标定（**现场必做，5 分钟**）

这是**唯一一条"判错就扣分"的标定**（打友方扣 5 分/次，且友方计数到 3 会让整个靶标失效）。

```
1. 把靶标红面朝向相机
   ros2 topic echo /offset_center
   -> 看 [8] 是否为 1
2. 转 180°（蓝面朝向相机）
   -> 看 [8] 是否为 2
3. 判成 0（不确定）：把 det_node 的 face_min_ratio 调小（0.12 -> 0.08）
4. 红蓝判反：把 face_red_bias 调小（1.25 -> 1.1）
5. ★ 在强光/逆光下重复 1~4，取"最差条件下仍能判对"的阈值
```

**通过标准**：红面稳定报 1、蓝面稳定报 2，且**两种光照下都对**。

**★ 兜底**：无论怎么标定，`[8] == 0`（不确定）时车端**默认禁火**。
宁可漏打，不可误打友方。

### 2.3 激光红点（可选，但有价值）

打一发激光，看 `[9]` 是否变 1。

- **能看到** → 红点联锁可用（能确认落点在靶上）
- **看不到** → 把 `aim.require_laser_dot` 设为 `false`
  （否则"打过一发后"会因为看不到红点而**再也不开火**）

---

## 阶段 3：底盘与云台（能量出"走不准"的边界）

### 3.1 直行标定

```bash
ros2 topic echo /odometry/filtered --field pose.pose.position
```

让车直行 1m（3 格），看位置变化。

**通过标准**：误差 < 10cm。

**★ 已知问题**：底盘积分位姿**不可用**（实测同一次 90° 右转两次偏向相反两侧）。
所以位置用的是**航位推算**：`步长 = 相邻采样点距离`（旋转不变量，准确）
沿 **IMU 地图偏航**积分。

**要验证的是**：
```bash
ros2 topic echo /localization/dr_step_cm    # 单步距离（应平滑，无跳变）
ros2 topic echo /localization/yaw_deg       # 地图偏航（应单调跟随转向）
```

### 3.2 ★ 原地转向（`PivotController`）

命令一次 90° 原地转向，看：
```bash
ros2 topic echo /arbiter_steer_deg
```

**通过标准**：
- 停在 `STOP -> WAIT_SERVO -> CREEP -> DONE` 四段内完成
- 最终偏航误差在 `pivot.tol_deg`(6°) 内
- 总耗时 < `pivot.total_timeout_s`(6s)

**★ 为什么重要**：v5 拓扑下**东侧窄条（L/M/N）单圆弧 90° 转弯不可行**
（扫掠半径 44.6cm，格心距场边仅 17.5cm）。
原地转向扫掠半径仅 **18.6cm**，是窄条内唯一可行的转向方式。
**若原地转向不准，窄条内的所有规划都执行不了。**

---

## 阶段 4：定位与格子对齐

### 4.1 anchor 注入

```bash
ros2 topic pub --once /nav/anchor std_msgs/msg/String \
  "{data: 'anchor:M,11,-90'}"
```

（格式：`anchor:列字母,行号(1-based),朝向角(度)`）

**通过标准**：
```bash
ros2 topic echo /mission_debug      # 应能看到当前格号 = M11
```

### 4.2 ★ 走一圈看格号是否漂

让车沿已知路径走（例如 M11 -> M10 -> M9 -> L9 -> K9），沿途看格号。

**通过标准**：格号与实际格一致，误差 **< 半格（17.5cm）**。

**★ 若漂移明显**：这是本项目最需要关注的风险（位置靠 IMU 偏航积分）。
- 检查 `loc.initial_map_yaw_deg` 是否为 `-90.0`
- 检查陀螺零偏是否已收敛（`/localization/gyro_bias_z` 应稳定）
- 每过一个门/已知点，可以用 `anchor` **重新对齐**（这是兜底手段）

---

## 阶段 5：排雷（40 分，最大单块）

### 5.1 先验规划（不起车）

```bash
# 造一个已知地雷布局，看规划器输出的路径
cd src/dzmission/test
python3 mine_planner_check.py
```

**通过标准**（离线已过，这里确认现场 `FieldGrid` 规格一致）：
- 覆盖 8/8
- 路径 27~37 格
- **0 处不可行转角**

### 5.2 ★ 实车单颗雷

放 **1 颗**地雷（格心），让车去压。

**通过标准**：车经过后，轮心轨迹确实进入了 12cm 圆。

**★ 判定口径提醒**（规则）："小车任意一轮中心进入地雷标识圆形区域内部"。
地雷图直径 12cm、**圆心 = 格心**、半径 6cm。
轮距 11cm → 轮心偏 5.5cm < 6cm。
所以**必须沿格线直穿**，斜穿会压缩容差。

### 5.3 实车多颗（4 颗 → 8 颗）

**通过标准**：
- 每颗都被压到
- 紧转角处车能通过（`turn_min_clearance_cm` 默认 0.5，紧转角会很多）
- **不撞固定场地超 10 秒**（否则裁判切手动）

**★ 现场调参顺序**：
1. 若车在紧转角卡住 → 加大 `pivot` 的使用（原地转向代替圆弧）；
   ★ 注意 `turn_min_clearance_cm` 是 **C++ 常量，现场改不了**（见附录 A.2）
2. 若路径太长超时 → 检查 `MinePlanner` 是否走了区域分组
3. 若压不到某颗 → 看日志里那颗的 `makePass` 选向原因

---

## 阶段 6：桥洞（20 分）

### 6.1 门位注入

```bash
# 手动发 BRIDGE_GATE（或用 mission_cmd 话题）
# 区间一 K4(row=3)、区间二 K9(row=8)
```

**期望日志**：
```
[Mission] 桥洞门位：bridge_id=1 row=K4 conf=100 ...
[Mission] ★ 桥洞就绪：K4 + K9（可跨桥）
```

**★ 必测的 3 个异常**：

| 测试 | 发送 | 期望 |
|---|---|---|
| 越界 | `row=11`（K12） | `GATE_INVALID` + 告警，**不规划** |
| 同段 | 两个都在区段一 | `GATE_INVALID` + 告警 |
| 只发一个 | 只发区段一 | 状态变 `kPartial`，**每 2s 发 `NEED_GATE`** |

### 6.2 ★ 实车穿门

让车实际穿过 K4（J4 -> K4 -> L4）。

**期望日志**：
```
[Mission] ★ 穿过桥洞区段一（K4）—— 该区段 10 分
```

**通过标准**：
- **一次穿门只上报一次**（穿过两次也只算一次，规则"每个桥洞通道只计一次"）
- 两个区段都过后打印"桥洞穿梭完成（共 20 分）"

**★ 最容易实现错的一点**：判定必须看"**进入 + 穿出**成对出现"，
不能只看"到达门格" —— 否则路过门前也会被算通过。
这一步用 `bridge_selftest` 已离线验证，实车确认"实际过门能被检出"即可。

---

## 阶段 7：敌我识别与打击（15 分，但**扣分风险最高**）

### 7.1 ★★ 红面禁火（**必须先测这个**）

把靶标**红面**朝向车，让车进入交战。

**通过标准**：
- **绝对不开火**
- 日志出现 `★ 可见面为红色(友方) -> 禁止开火`

**★ 这是最重要的一条测试。** 规则：友方受击计数每 +1 扣 5 分，
且友方计数到 3（= 6 次误击）会让**整个靶标失效** ——
此时敌方计数还没满，那 15 分也拿不到了。

### 7.2 蓝面开火

把靶标**蓝面**朝向车。

**通过标准**：开火，日志出现：
```
[Aimer] FIRE #1(local 1) cx=... cy=... face=2 dot=... | beta-lead v=... lead=... T_total=380ms
```

**★ 关键看点**：
- `face=2`（蓝面识别正确）
- `beta-lead` 还是 `alpha-dwell`：
  - `beta-lead` → 走了提前量路径
  - `alpha-dwell` → 退回换向点（也正常，说明 β 条件不满足）

### 7.3 ★ 移动靶

靶标在轨道上匀速往复，看车能否打中。

**分档验证**：

| 档 | 配置 | 目的 |
|---|---|---|
| **α 档** | `aim.lead_enable: false` | 验证"只在换向点开火"能否打中 |
| **β 档** | `aim.lead_enable: true` | 验证提前量是否更准 |

**★ 先测 α 档。** 理由：α 在 v≈0 时开火，**不受 T_total 标定误差影响**。
若 α 都打不中，说明问题在别处（判色、云台跟瞄、站位），
**不是提前量的问题** —— 这时候调 β 是浪费时间。

### 7.4 β 的三个标定量（**只能实测**）

| 参数 | 怎么测 | 不确定时的安全做法 |
|---|---|---|
| `lead_servo_ms` | 让云台做小角度阶跃，看 `camemo_feedback` 到位时间 | **先设 0** |
| `lead_fire_ms` | 发一次开火，用高速摄像或声音测延迟 | **先设 0** |
| `lead_exposure_ms` | 查相机驱动配置的曝光时间 | 保守取 33（1/30s） |

**★ 标定顺序（安全）**：
```
1. 先全部设 0（只补偿流水线 267ms）
2. 打几发，看脱靶方向
3. 若总是"落后于靶标" -> 加大 T_total（先加 servo，再加 fire）
4. 每次只改一个参数，打 3 发确认
```

**★ 脱靶方向的含义**：
- **激光落在靶标后面** → 提前量不够 → 加大 `lead_servo_ms` / `lead_fire_ms`
- **激光落在靶标前面** → 提前量过多 → 减小

### 7.5 弹量纪律

```bash
ros2 topic echo /referee/ammo      # 应从 10 递减
```

**通过标准**：本地记账（日志 `local N`）与裁判板计数一致。

**★ 必测**：拔掉裁判板链路，看是否按 `ammo_link_lost_policy` 降级
（默认 2 = 折中：允许再打 3 发且需反馈）。

---

## 阶段 8：端到端演练（**准备阶段 2 分钟的真实模拟**）

规则：**准备阶段只有 2 分钟**，允许启动程序与调参，**禁止更换硬件**。

### 8.1 ★ 计时演练

```
计时开始
  0:00  ros2 launch dzmission bringup_mission.launch.py
  0:30  确认所有节点起来（见 8.2）
  0:45  确认 /offset_center 10 个元素、15Hz
  1:00  确认 /pursuitAngle 数值范围 ±35（度，不是 rad/s）
  1:15  确认 /referee/ammo = 10
  1:30  把车放到起点
  1:50  停手（规则：结束前 20 秒交遥控器）
计时结束
```

**通过标准**：能在 **90 秒**内完成全部检查（留 30s 余量）。

**★ 为什么留余量**：现场会有意外（电脑慢、USB 没识别、等等）。

### 8.2 上场前 5 项必查（打印在 launch 启动横幅里）

```bash
1) ros2 topic echo /offset_center --once
   -> 应有 10 个整数，[8]=面(0/1/2) [9]=激光红点(0/1)
2) ros2 topic hz /offset_center            -> 应约 15 Hz
3) ros2 topic echo /pursuitAngle           -> angular.z 范围 ±35（度，非 rad/s）
4) ros2 topic echo /referee/ammo           -> 应为 10
5) 确认未启动 dzrknn_dataprocess           -> 否则抢云台/抢激光
```

### 8.3 全场任务演练

按顺序跑一遍完整流程，记录各阶段耗时：

| 阶段 | 期望 | 实际 | 备注 |
|---|---|---|---|
| 等待无人机侦察 | < 60s | | `BRIDGE_GATE` 就绪前车停在起点 |
| 桥洞穿梭 | < 40s | | 2 门 × 10 分 |
| 排雷 8 颗 | < 60s | | 40 分，路径 27~37 格 |
| 敌我识别打击 | < 40s | | 15 分 |
| 回到终点 | < 20s | | 5 分 |
| **合计** | **< 180s** | | 超时则按 180s 计 |

---

## 附录 A：参数速查（现场只改这些）

> ★ **注意有两类"参数"，改动方式完全不同**：
>   · **YAML 参数**（`dzmission/params/dzmission.yaml`、`rknn_pt/launch/det_node.launch.py`）
>     → 改文件后**重启节点**即可，现场可调
>   · **C++ Spec 常量**（`field_grid.hpp` 的 `struct Spec`）
>     → **必须重新编译**，现场**改不了**。所以这些值必须**赛前定死**

### A.1 YAML / launch 参数（现场可调）

| 参数 | 默认 | 什么时候改 |
|---|---|---|
| `det_node.face_min_ratio` | 0.12 | 判色总是"不确定" → 调小（★ 在 `rknn_pt/launch/det_node.launch.py`）|
| `det_node.face_red_bias` | 1.25 | 红蓝判反 → 调小（★ 同上）|
| `aim.require_laser_dot` | true | 看不到红点 → 改 false |
| `aim.fire_cooldown_ms` | 3000 | 规则靶标 1s 恢复；不建议低于 1500 |
| `aim.engage_standoff_cells` | 1 | 检测框 < 40px 宽 → 加大到 2 |
| `aim.require_turn_around` | true | A/B 对比时用（false = 随时可打）|
| `aim.lead_enable` | true | 退回纯 α 时设 false |
| `aim.lead_servo_ms` / `lead_fire_ms` | 60 / 20 | 按脱靶方向标定；不确定先设 0 |
| `loc.initial_map_yaw_deg` | -90.0 | 起点车头朝 M10（行号减小）|
| `cmd.transport` | loopback | 联调时改 `udp` 或 `serial` |
| `cmd.udp_bind_port` / `cmd.udp_peer_host` / `cmd.udp_peer_port` | — | 按 H2 的约定填 |
| `pivot.tol_deg` | 6 | 原地转向精度不够 → 适当放宽（但会影响窄条内转向成功率）|
| `pivot.total_timeout_s` | 6 | 转向总超时 |

### A.2 ★ C++ Spec 常量（**现场改不了，必须赛前定死**）

修改位置：`dzmission/include/dzmission/field_grid.hpp` 的 `struct Spec`。
**改完必须 `colcon build`。**

| 常量 | 默认 | 说明 |
|---|---|---|
| `cols` / `rows` | 14 / 11 | 场地格数（A–N × 1–11）|
| `cell_size` | 0.35 | 格边长 35cm |
| `barrier_up_row` | 5 | 水平隔离线在 row6/row7 之间（0-based 的 5）|
| `barrier_col_lo` / `barrier_col_hi` | 3 / 13 | 隔离线覆盖 D..N 全宽 |
| `bridge_col` | 10 | 桥洞在 K 列 |
| `bridge_a_lo/hi` | 0 / 5 | 区段一 K1..K6 |
| `bridge_b_lo/hi` | 6 / 10 | 区段二 K7..K11 |
| `start_col` / `start_row` | 12 / 10 | 起点 M11 |
| `goal_col` / `goal_row` | 13 / 5 | 终点 N6 |
| `car_width_cm` / `car_length_cm` | 14.0 / 27.8 | 车体尺寸（现场实测）|
| `wheelbase_cm` | 15.0 | 轴距（★ 与学员用的 0.17 差 13%，务必核实）|
| `track_cm` | 11.0 | 轮距 |
| `min_turn_radius_cm` | 26.0 | 最小转弯半径 240~260mm |
| **`turn_min_clearance_cm`** | **0.5** | ★ 转弯安全余量。**取 7.0（半车宽）会把格内转弯全拒掉**（假阳性 684 vs 真值 0），所以取 0.5 |
| `pivot_min_gap_cm` | 0.0 | 原地转向最小间隙 |
| `steer_trim_deg` | -1.0 | 舵机机械左偏补偿 |
| `front_overhang_cm` / `rear_overhang_cm` | 17.2 / 10.6 | 前后悬（决定原地转向扫掠半径 18.6cm）|

**★ 若场地实测与上表不符**，改 `Spec` 后**必须重跑全部 17 个验证脚本**
（它们会检查 `Spec` 与拓扑 v5 的一致性）。

## 附录 B：已知风险与兜底手段

| 风险 | 症状 | 兜底 |
|---|---|---|
| **位置漂移** | 格号逐渐偏 | 用 `/nav/anchor` 在已知点重新对齐 |
| **撞场地卡住** | 不动且电机堵转 | `SafetyMonitor` 堵转检测 -> 脱困倒车；**务必确认 10s 内脱离**（超时切手动） |
| **判色失败** | `[8]==0` | 默认禁火（不扣分，但丢 15 分） |
| **门位缺失** | 车停在起点不动 | 每 2s 发 `NEED_GATE`；★ 同时向裁判确认桥洞是否真的有可通行格 |
| **云台跟丢移动靶** | 画面里靶标追不上 | 加大 `engage_standoff_cells`；或依赖 α 的换向点 |
| **红点看不到** | 打过一发后不开火 | 设 `aim.require_laser_dot: false` |

## 附录 C：验证记录表（现场填写）

| 阶段 | 项目 | 通过? | 实际值/备注 |
|---|---|---|---|
| 0 | 编译 | ☐ | |
| 0 | `bridge_selftest` | ☐ | |
| 0 | 17 个 Python 脚本 | ☐ | |
| 1 | `BridgeTracker` 自检日志 | ☐ | |
| 1 | `/cmd_vel` 单发布者 | ☐ | |
| 2 | `/offset_center` 10 元素 | ☐ | |
| 2 | 红蓝判色（红=1） | ☐ | |
| 2 | 红蓝判色（蓝=2） | ☐ | |
| 2 | 强光下判色 | ☐ | |
| 2 | 激光红点可见 | ☐ | |
| 3 | 直行 1m 误差 | ☐ | |
| 3 | 原地转向 90° | ☐ | |
| 4 | anchor 注入 | ☐ | |
| 4 | 走一圈格号不漂 | ☐ | |
| 5 | 单颗雷 | ☐ | |
| 5 | 8 颗雷 | ☐ | |
| 6 | 门位注入 | ☐ | |
| 6 | 门位异常 3 项 | ☐ | |
| 6 | 实车穿门 | ☐ | |
| 7 | **红面禁火** | ☐ | |
| 7 | 蓝面开火 | ☐ | |
| 7 | α 档打中 | ☐ | |
| 7 | β 档打中 | ☐ | |
| 7 | 弹量纪律 | ☐ | |
| 8 | 2 分钟计时演练 | ☐ | |
| 8 | 全场任务演练 | ☐ | |
