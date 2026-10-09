# 云台瞄准与射击链路 · 只读代码调研报告

- 调研对象：`D:\User\Desktop\dzrobot_v2`（对应车上 `~/dzrobot`，v2 新核心）
- 调研方式：**纯只读**，未修改任何既有文件（仅新增本报告）
- 调研时间点：以工作区当前副本为准（`path_executor.py` = 151006 B 的 10-03 新核心）

## 0. 调研文件与规模

| 文件 | 行数 | 角色 |
|---|---|---|
| `src/dzrknn_dataprocess/src/dzrknn_dataprocess.cpp` | 1154 | 云台自瞄主节点（重点） |
| `src/dzrknn_dataprocess/include/dzrknn_dataprocess.h` | 281 | 状态机/参数/成员声明 |
| `src/dzrknn_dataprocess/launch/dzrknn_dataprocess.launch.py` | 50 | 只装载 `config/robot.yaml` |
| `src/dzjudgment/src/judgment.cpp` / `judgment.h` / `src/dzjudgment.cpp` | 402 / 96 / 22 | 射击驱动（裁判串口） |
| `src/dzjudgment/launch/judgment.launch.py` | 59 | 射击驱动启动 |
| `src/rknn_pt/src/det_node.cc` | 307 | 视觉检测节点（发布 `offset_center`） |
| `src/rknn_pt/src/det/{rkpt.cc,postprocess.cc,preprocess.cc}` | 311 / 209 / — | RKNN 推理与 YOLOv8-pose 后处理 |
| `src/rknn_pt/include/{common.h,det/postprocess.h,rknnPool.hpp}` | 55 / 42 / 123 | 数据结构、阈值、推理线程池 |
| `config/robot.yaml` | 396 | 唯一参数真源（本文引用为部署实值） |

---

## 1. 链路拓扑（实测代码级）

```
 usb_cam_node(/image_raw,640x480@30)
        │
        ▼
 det_node ──offset_center(Int32MultiArray,8字段,~30Hz)──► dzrknn_dataprocess（云台自瞄）
        ▲                                                        │
        │                                                        ├─ CameMo_Conl(Int32MultiArray[4]) ─► dzactuator ─串口A5 5A...─► 下位机云台
        │                                                        ├─ LaserShot_Command(UInt8=1) ─► dzjudgment ─串口 B5 5B 03 21 01 25 ─► 激光/裁判模块
        │                                                        ├─ /shoot_count(Int32) ─► path_executor
        │                                                        └─ /aim/status(Int32) ─► path_executor / cam_viewer
        │
        └◄──────────────────────────────────────────────────────── dzactuator 发布 camemo_feedback(位置/速度回传)
 path_executor ──/nav/scan_request(UInt8)──► dzrknn_dataprocess
 path_executor ──/move_base/stop_signal(UInt8)──► **无任何 C++ 订阅者（见 §7.4，链路缺口）**
 tools/rc_control.py ──/nav/manual_gimbal──► dzrknn_dataprocess（手动接管）
```

注意名称陷阱：`dzrknn_dataprocess` 的可执行文件名是 **`dzactuator_node`**（`CMakeLists.txt:91`、`dzrknn_dataprocess.launch.py:44`），与 `dzactuator` 包的可执行文件**同名**（`src/dzactuator/CMakeLists.txt:87`）。用 `ros2 run` 时极易起错节点。

---

## 2. 云台节点状态机

### 2.1 状态枚举（`dzrknn_dataprocess.h:110`）

| 值 | 名称 | 语义 |
|---|---|---|
| 0 | `AIM_IDLE` | 空闲：不下发扫描，云台**主动归位**到正前方+基准俯仰 |
| 1 | `AIM_SCAN` | 全向扫描：水平连续匀速往复（可跨俯仰层） |
| 2 | `AIM_TRACK` | 已锁定：视觉伺服持续修正，边走边跟随 |

开机**直接进入 SCAN 且 continuous=true**（`cpp:1016-1032`），不再依赖 `/nav/scan_request` 时序 —— 这是 v2 的关键行为变更：上电即扫，只有收到 `/nav/scan_request=0` 才停。

### 2.2 转移条件

| 迁移 | 触发条件 | 位置 |
|---|---|---|
| SCAN/IDLE → TRACK | 首帧合格目标（cls∈{1,2} 且长宽比合格） | `cpp:45-61` |
| TRACK → SCAN | 连续丢帧 ≥ `lost_frames_to_rescan_`(部署 180 帧≈6s) **且** `scan_continuous_` | `cpp:291-312` |
| TRACK → IDLE | 同上但 `scan_continuous_=false` | `cpp:309-311` |
| TRACK → SCAN/IDLE | **视觉完全静默** > `vision_timeout_ms_`(1000ms，按"收到 offset_center"计) | `cpp:629-642` |
| SCAN → IDLE | 扫满 `scan_passes_×层数` 趟且非连续 | `cpp:694-713` |
| 任意 → IDLE | `/nav/scan_request = 0` | `cpp:604-610` |
| IDLE/SCAN → SCAN | `/nav/scan_request ≥1` 启动；`≥2` 为连续；已在 SCAN 时**只改模式不重置进度** | `cpp:612-621` |
| TRACK 期间 | `/nav/scan_request` 一律忽略，不打断跟随 | `cpp:599-602` |

**IDLE 行为**（`cpp:646-668`）：每帧持续下发归位指令（yaw 取扫描范围中点、pitch 取 `scanPitchDeg()`），速度 = `patrolSpeed()×idle_homing_speed_ratio_`。注释明确说明：旧版 IDLE 直接 return 导致"云台冻结在任意姿态、改俯仰参数看不出变化"。

**TRACK 期间丢帧的行为**（`cpp:627-644`）：只要还有视觉心跳（哪怕是"无目标"帧），TRACK 分支**立即 return，云台纹丝不动**——最多冻结约 6 秒（180 帧）才重新扫描。对来回移动的靶，这是明确的浪费窗口（见 §9.7）。

---

## 3. 扫描策略

| 项 | 实现 | 位置/实值 |
|---|---|---|
| 扫描形态 | **连续匀速往复**，不再逐点步进停留；目标点恒取当前方向端点 | `cpp:398-404`、`518-531` |
| 水平范围 | `scan_yaw_min_deg_ ~ scan_yaw_max_deg_` | 代码默认 −90~+90；**部署 −160 ~ +10**（`robot.yaml:36-37`） |
| 俯仰层 | `scan_pitch_levels`（列表，逐层叠加 `scan_pitch_offset_deg`） | 代码默认 `[0.0]`；**部署 `[0.0]` + 偏移 +6.0**（`robot.yaml:44,54`） |
| 换层规则 | 每扫满 `scan_passes_`(2) 趟换一层 | `cpp:540-544` |
| 一轮结束 | `scan_pass_done_ ≥ scan_passes_×层数` | `cpp:546-549` |
| 扫描速度 | `gimbal_max_speed × patrol_speed_ratio` | 代码 3700×0.35=1295；**部署 3700×0.10=370 刻度/s**（`robot.yaml:63-64`） |
| 到位判据 | 两轴反馈与指令差 ≤ `scan_arrive_tol_`(30 刻度) | `cpp:676-678` |
| 超时兜底 | `max(3×dwell, 1.5×整程时间)`，防半路误反向 | `cpp:679-689` |
| 持续模式 | 一轮扫完立刻重来（不进入 IDLE），日志"一轮扫描结束未发现目标，继续扫描" | `cpp:698-706` |

**时序估算（部署值）**：水平跨度 `|10−(−160)|=170°` → 1934 刻度；370 刻度/s → **单趟约 5.2s，一轮（2 趟）约 10.5s**。若把俯仰层改回多层（如 3 层），一轮约 31s。

**扫描俯仰 = 0 + 6.0 = +6°（抬头）**。按 `robot.yaml:46-48` 与 `cpp:140-151` 的实测记录，0.35m 处的地面靶需要约 −40° 俯角 —— 即**当前部署配置下扫描平面看不见地面靶**。`robot.yaml:42-43` 自己写明："若扫描时看不到地面靶，把本值改回负数（−20 ~ −26 之间试）"。这是出车前最该确认的一个参数。

---

## 4. 视觉伺服（跟踪控制）

### 4.1 处理流水线（`cpp:74-170`）

1. **EMA 低通**：`dx_filt = α·raw + (1−α)·dx_filt`，α=`ema_alpha_`（部署 0.55，`cpp:89-90`）。
2. **跟踪死区**：|滤波值| ≤ `track_deadzone_px_`(部署 4px) → 有效偏差 0，完全不动作（`cpp:92-103`）。
3. **位置增量**（**增量式 P + 单帧预判**，`cpp:138-162`）：
   - 俯仰：`inc = −(eff_y/2 + vy·predict_gain) · pitch_deg_sign`，钳位走 **`clampPitchTick`（机械限位 950~2400 ≈ −96°~+31°）**（`cpp:152-153`，理由见 `cpp:140-151`）。
   - 水平：`inc = eff_x/2 + vx·predict_gain + ff_x`，钳位走 **`clampYawToScan`（被限制在扫描范围）**（`cpp:157-162`）。
4. **PD 速度**（`cpp:164-170` → `trackSpeedFor` `cpp:455-487`）：
   - `sp = kp·|err| + kd·Δ|err|`，死区内直接 0；`sp<100` 抬到 100（电机起步下限）；上限 = `track_speed_max_`（>0 时）否则 `patrolSpeed×track_speed_max_ratio`。
   - 精调档：`|err| < fine_zone_px_`(20) 时 `kp×0.5`、`kd×2.0`（`cpp:464-469`）。
5. **车体转动前馈**：`ff_x = −ωz·57.3·yaw_ticks_per_deg·dt·gain·sign`，`gyro_ff_gain_` **默认 0（关闭）**、IMU 超时 200ms 暂停（`cpp:112-129`、`cpp:754-758`）。

### 4.2 钳位函数族

| 函数 | 语义 | 位置 |
|---|---|---|
| `clampYawTick` | 硬件极限 100~3995 | `cpp:329-340` |
| `clampPitchTick` | 下位机机械限位 950~2400 | `cpp:343-354` |
| `clampYawToScan` | **扫描范围**（±deg × 刻度/度 + 中心） | `cpp:359-368` |
| `clampPitchToScan` | 扫描俯仰范围 **再外扩 `tracking_pitch_margin_deg_`** | `cpp:371-395`（外扩见 `385-386`） |

⚠ 一致性风险：**跟踪的水平轴用 `clampYawToScan`**，部署范围是 [−160°, +10°]。车头**右侧超过 10°** 的目标永远追不到（会顶在 2161 刻度上）。而 `tools/rc_control.py:54-55` 的手动接管限幅是按 ±90° 写成 `YAW_MIN/YAW_MAX = 1022/3071` —— 两处对"可用水平范围"的假设不一致。

### 4.3 增益与消抖一览（代码默认 / 部署实值）

| 参数 | 默认 | 部署 | 说明 |
|---|---|---|---|
| `track_kp` / `track_kd` | 8.0 / 3.0 | 同 | PD 增益（`cpp:905-906`） |
| `fine_kp_scale` / `fine_kd_scale` | 0.5 / 2.0 | 同 | 精调档（`cpp:903-904`） |
| `ema_alpha` | 0.3 | **0.55** | 越大越跟手、抖动越大 |
| `track_deadzone_px` | 12 | **4** | 必须 < `fire_dz_far_px`（`robot.yaml:105` 注释自证） |
| `predict_gain` | 0.5 | 0.5 | 单位是**帧**，见 §9.2 |
| `track_speed_max` / `_ratio` | 0 / 0.6 | 0 / **0.63** | 部署派生上限 = 370×0.63 ≈ **233 刻度/s** |
| `gyro_ff_gain` | 0.0 | **0.0（关闭）** | 转弯凝视稳定实际未启用 |

---

## 5. 参数键清单（全部读取的参数）

节点在 `cpp:871-935` 声明、`cpp:937-1000` 读取（成对）。按 `robot.yaml` 段落 `/dzrknn_dataprocess` 装载（`launch.py:47`）。

| 组 | 键名 |
|---|---|
| 扫描姿态 | `scan_yaw_min_deg`, `scan_yaw_max_deg`, `scan_pitch_levels`, `scan_pitch_offset_deg`, `scan_yaw_dirs`(仅兼容), `scan_passes`, `scan_dwell_ms`, `scan_arrive_tol` |
| 刻度标定 | `gimbal_center_tick`, `yaw_ticks_per_deg`, `pitch_ticks_per_deg`, `yaw_deg_sign`, `pitch_deg_sign`, `gimbal_yaw_tick_min/max`, `gimbal_pitch_tick_min/max`, `gimbal_max_speed`, `tracking_pitch_margin_deg` |
| 速度 | `patrol_speed_ratio`, `track_speed_max`, `track_speed_max_ratio`, `idle_homing_speed_ratio`, `loop_hz_track`, `loop_hz_idle` |
| 伺服 | `ema_alpha`, `track_deadzone_px`, `track_kp`, `track_kd`, `fine_zone_px`, `fine_kp_scale`, `fine_kd_scale`, `predict_gain` |
| 开火 | `fire_dz_dynamic`, `fire_deadzone_px`, `fire_dz_far_px`, `fire_dz_near_px`, `box_w_far`, `box_w_near`, `fire_hold_frames`, `fire_use_image_stable`, `fire_stable_px_per_frame`, `fire_real_speed_max`, `fire_speed_max`, `target_confirm_frames`, `cls_switch_confirm_frames`, `shot_cooldown_ms` |
| 弹药/筛选 | `fire_only_blue`, `max_shots`, `max_shots_per_target`, `aspect_min`, `aspect_max` |
| 激光标定 | `laser_offset_x`, `laser_offset_y` |
| 状态机 | `lost_frames_to_rescan`, `vision_timeout_ms` |
| 前馈 | `gyro_ff_gain`, `gyro_ff_sign`, `imu_ff_timeout_ms` |

**硬编码上限**：`AIM_MAX_SHOTS = 10`（`h:211`），`max_shots` 超出/非法一律强制为 10（`cpp:1008-1014`）。

`dzjudgment` 段参数仅 2 个：`mcuserialport`（部署 `/dev/dzjudgment`）、`mcubaudrate`(115200)（`robot.yaml:204-207`、`judgment.cpp:8-11`）。

---

## 6. 开火链路

### 6.1 "锁定 → 发射"的全部必要条件（`cpp:172-263`）

| # | 条件 | 代码 | 部署阈值 |
|---|---|---|---|
| 1 | 目标类别已确认 | `target_confirmed = target_seen_count_ ≥ target_confirm_frames_` | 8 帧（≈0.27s@30Hz） |
| 2 | 在（动态）开火框内 | `aim_err_x/y ≤ fireDeadzonePx(box_w)`，零点取**激光落点** `laser_offset_x/y` | 远 14px ~ 近 26px（部署） |
| 3 | 瞄准稳定 | `fire_use_image_stable_ ? (|vx|,|vy| ≤ fire_stable_px_per_frame_) : motor_still` | 1.5 px/帧（画面稳定模式） |
| 4 | 弹药允许 | `max_shots_>0 && shots_fired_<max_shots_ && shots_on_target_<max_shots_per_target_` | 全程 10 发 / 单目标 3 发 |
| 5 | 类别允许 | `cls_fire_ok = !fire_only_blue_ || locked_cls_==1` | 仅蓝靶 |
| 6 | 冷却 | `now − last_shot_time_ ≥ shot_cooldown_ms_` | 500ms |
| 7 | 连续保持 | `fire_hold_count_ ≥ fire_hold_frames_` | 5 帧（≈0.17s） |

任一不满足 → `fire_hold_count_=0`（`cpp:260-263`）。

### 6.2 发射动作（`cpp:236-257`）

```cpp
shotdata.data = 1; pub_LaserShot_Command->publish(shotdata);   // cpp:238-239
shotdata.data = 0; pub_LaserShot_Command->publish(shotdata);   // cpp:240-241
shots_fired_++; shots_on_target_++; last_shot_time_ = now; fire_hold_count_ = 0;
pub_shot_count->publish(shots_fired_);                          // cpp:249-251
```

话题 `LaserShot_Command`（UInt8，队列 10）：**先发 1 再立刻发 0**；无目标时该话题被以 ~30Hz 持续发 0（`cpp:315-317`）。`/shoot_count` 只在开火与重置时发布（非周期）。

### 6.3 dzjudgment 串口帧与日志

| 项 | 内容 | 位置 |
|---|---|---|
| 开火帧 | `B5 5B 03 21 01 25`（6 字节，固定；`buf[2]=0x03` 长度、`buf[5]=0x25` 校验） | `judgment.cpp:358-366` |
| 触发 | 订阅 `LaserShot_Command`，`data==1` 即写串口（`data==0` 忽略） | `judgment.cpp:77-83` |
| 成功日志 | `[dzjudgment-->] ★ 开火指令已下发 (B5 5B 03 21 01 25)` | `judgment.cpp:390-391` |
| 主循环 | **100Hz**（原 10Hz 导致 ≤100ms 排队延迟） | `judgment.cpp:119-128` |
| 收帧 | 非阻塞有界：无数据立即让出，每轮最多 8 帧；`B5 5B len payload` | `judgment.cpp:228-282` |
| 帧类型 | `payload[0]`：0x02 全部物资 / 0x03 敌方 / 0x04 己方 / **0x07 血量弹药** | `judgment.cpp:300-353` |
| 0x07 解析 | `[4]对方HP [5]己方HP [6]被击打方位 [7]累计射击数 [8]剩余弹药` | `judgment.cpp:340-349` |
| 回传话题 | `HpAndHitmsg = [targethp, self_hp, Bing_hit, Shooting_Count, ammo]`，1Hz 心跳 + 收帧即发 | `judgment.cpp:193-203`、`152-156` |
| 通用指令 | `/referee_cmd`(UInt8MultiArray) 原样写串口（用于开启/复位等未知协议帧） | `judgment.cpp:85-117` |

⚠ **闭环缺口**：`dzrknn_dataprocess` **不订阅** `HpAndHitmsg`（构造器订阅仅 6 条，`cpp:838-863`）。软件侧 `shots_fired_` 统计的是"已发指令数"，既不与裁判回传的 `ammo` 对账，也不利用 `Bing_hit`/命中信息停火。

### 6.4 射击许可门（缺口）

`path_executor` 的三段式状态机（`path_executor.py:2030-2193`）会在"底盘停稳 + 看到靶 + 连续稳定 8 帧"后发布 `sig.data=1` 到 **`/move_base/stop_signal`**（`path_executor.py:2128`），文件头注释写"→ dzrknn_dataprocess"（`path_executor.py:43`）。但全仓 C++ 搜索 `stop_signal` **零命中**（唯一相邻的 `StopCmd` 是 dzactuator 的底盘停车，`dzactuator.cpp:1171`）。结论：**这个"放行开火"门在 v2 里没有接收方**，云台完全按自己的判据自主开火，`path_executor` 的许可/收回只是一个本地状态与日志。

---

## 7. 云台手动接管机制

| 环节 | 实现 |
|---|---|
| 开关话题 | `nav/manual_gimbal`（UInt8，BEST_EFFORT KeepLast1，`cpp:850-861`，lambda 回调） |
| 置位动作 | `manual_override_ = true` → **主循环不再发布 `CameMo_Conl`**（`cpp:1133-1136`） |
| 不影响的 | 开火、`/aim/status`、`/shoot_count`、状态上报照常（`h:85-89`、`cpp:1131-1136`） |
| 发布方 | `tools/rc_control.py:105`（同时自己发 `CameMo_Conl`，`rc_control.py:90,228`），并以 `_on_gimbal`(123-129) 镜像云台指令位置做接管起点 |
| 限幅 | 遥控侧 `PITCH 950~2400`、`YAW 1022~3071`（`rc_control.py:54-57,116-121`） |
| 设计原因 | 本节点 50~100Hz 持续发位置指令，与遥控器同话题不同频会"疯狂来回抽"（`h:86-88`） |

注意：接管只停"位置下发"。**云台在 SCAN 状态仍在推进扫描索引**（`CaremaMontorControl` 照常执行 `applyScanTarget()`，只是最后不 publish），释放接管后会从推进后的索引继续，不是原地续接。

---

## 8. ROS 接口清单（dzrknn_dataprocess）

### 8.1 订阅

| 话题 | 类型 | QoS | 来源 | 频率 |
|---|---|---|---|---|
| `offset_center` | Int32MultiArray | BEST_EFFORT KeepLast1 | det_node | ~30Hz |
| `camemo_feedback` | Int32MultiArray | BEST_EFFORT KeepLast1 | dzactuator | 100Hz 主循环 |
| `/nav/scan_request` | UInt8 | 默认可靠 depth10 | path_executor | 事件 |
| `nav/manual_gimbal` | UInt8 | BEST_EFFORT KeepLast1 | rc_control | 事件 |
| `/shoot_reset` | UInt8 | 默认 depth10 | rc_control | 事件 |
| `/imu_data` | sensor_msgs/Imu | depth1 | dzactuator | 高频 |

`camemo_feedback` 字段（`cpp:723-734`）：`[0]Position_0(俯仰) [1]Position_1(水平) [2]Speed_0 [3]Speed_1 [4]Load_0 [5]Load_1`（≥6 字段才采信）。

### 8.2 发布

| 话题 | 类型 | 频率/时机 | 内容 |
|---|---|---|---|
| `CameMo_Conl` | Int32MultiArray[4] | 主循环 50/100Hz（TRACK 100） | `[Pitch, Yaw, Speed_0, Speed_1]`（`cpp:1125-1136`） |
| `LaserShot_Command` | UInt8 | 开火时 1→0；无目标时 ~30Hz 发 0 | 1=开火 |
| `voice_switch` | UInt8 | 无目标时发 111；cls>2 时发类别号 | 语音 |
| `/shoot_count` | Int32 | 开火/重置时 | 累计发数 |
| `/aim/status` | Int32 | 变化即发 + 0.5s 保活 | 0/1/2 |

下位机链路（`dzactuator.cpp:391-439`）：`CameMo_Conl` → 23 字节目标帧 `A5 5A 14 ...`，其中 `buf[8..9]=Pitch`、`buf[10..11]=Yaw`、`buf[12..15]=Speed` 均为**大端 16 位**；`buf[7]` 是激光常亮开关（由 `/laser_switch` 控制）。

---

## 9. 视觉端（det_node）输出契约

### 9.1 `offset_center` 字段（`det_node.cc:215-222`，含注释）

| 下标 | 含义 | 备注 |
|---|---|---|
| [0] | 框中心 x − 画面中心 x（px） | 右为正 |
| [1] | 框中心 y − 画面中心 y（px） | 下为正 |
| [2] | **类别+1**：1=蓝靶，2=红靶，0=无目标 | `res.cls+1`，`det_node.cc:252` |
| [3] | 框宽（px） | 用于长宽比过滤 + 距离估计 |
| [4] | 框高（px） | 长宽比过滤 |
| [5] | 靶心关键点 x 偏移（px） | **当前云台代码未使用** |
| [6] | 靶心关键点 y 偏移（px） | 未使用 |
| [7] | 关键点置信度 ×100 | 未使用 |

`layout.dim[0].size = 8`，label `"light center coordinate"`（`det_node.cc:216-219`）。

### 9.2 频率与流水线

- 定时器周期 = `max(33ms, 1000/max_fps)` → 部署 `max_fps=30`、`thread_num=8`（`robot.yaml:221-222`），实测日志"推理周期 33ms"。
- 发布条件：`results_group.cur_img` 非空即发布（**无目标也发心跳帧**，全 0），`det_node.cc:240,260`。
- **流水线深度 = `thread_num`**：`put()` 每拍入队一个 future（`rknnPool.hpp:96`），`get()` 取**队首最老**结果（`rknnPool.hpp:108`），且前 `thread_num` 拍不取（`det_node.cc:234`）→ 队列稳定在 8 帧，**视觉结果约滞后 8×33ms ≈ 0.26s**。
- **每帧只发一个目标**：`for (res : dets)` 循环里反复覆盖同一个 `msg.data`，循环外只 publish 一次（`det_node.cc:242-260`）→ 同帧多个目标时**只有最后一个存活**，且无目标 ID。
- 阈值：`NMS_THRESH 0.45`、`BOX_THRESH 0.4`（`det/postprocess.h:25-26`）；类别数 2（blue/red）、关键点 1×3（`postprocess.h:20-23`）。
- 每帧 `orig_img.clone()`（`rkpt.cc:292`）+ `imshow/waitKey`（`draw:true` 时）是 ARM 上可观的开销（`det_node.cc:262-270`、`robot.yaml:218-220`）。

---

## 10. 面向"2 格轨道上来回移动的靶"的缺口（逐条 + 改动点）

> 场景：靶在 2 个格（0.7m）间往复，云台从某个停站点射击。以下按影响排序。

1. **没有目标运动估计**（只有单帧差分）。`vx = dx_filt − dx_prev_filt`（`cpp:107-108`）是**滤波后的一帧差分**，量纲 px/帧，无时间戳、无平滑、无速度状态保持。改动点：在 `callback_offset_center` 内（`cpp:105-110`）用 `this->now()` 差分改为 **px/s**，并维护 `vx_filt_/vy_filt_`（同 EMA 结构）与方向符号。
2. **预判量小且单位是"帧"**：`predict_gain_=0.5`（`cpp:919`、`robot.yaml:111`）乘 px/帧 → 仅提前 0.5 帧 ≈ 16ms。而端到端延迟 = 视觉流水线 0.26s + EMA(α=0.55 约 2~3 帧) + 串口/下位机 + 冷却 500ms ≈ **0.5~0.9s**。改动点：`cpp:139/157` 改为按时间提前量 `v_px_s × T_lead`，`T_lead` 新增参数（按链路实测标定）。
3. **纯位置增量控制，无目标角速度前馈**：`Position += eff/2 + vx·gain`（`cpp:157`），对匀速运动目标存在**稳态滞后**（滞后 ≈ 目标角速度/Kp）。车体转动前馈 `ff_x` 已具备同构结构（`cpp:125-129`）但 `gyro_ff_gain_=0` 关闭。改动点：复用 `ff_x` 的形式，在 `cpp:157` 增加 `+ k_tgt_ff · vx_px_s · dt`，并同步给 `Speed_1`（`cpp:168`）做速度前馈。
4. **开火判据要求"画面几乎静止"**：`aim_stable = |vx|,|vy| ≤ 1.5 px/帧`（`cpp:190-192`、`robot.yaml:124`）。移动靶需要云台持续运动，任何跟随残差都会打破该判据；再叠加 `fire_hold_frames_=5`（`cpp:232-235`）的连续保持要求，实际开火窗口极窄。改动点：把判据从"画面稳定"改为"**预测命中误差**"（用 §10.1 的速度预测命中点与 `laser_offset` 比较），或在 `cpp:190` 引入按目标速度自适应的阈值（速度越大允许残差越大）。
5. **无折返点/减速段识别**：靶在 2 格轨道两端必然减速换向，这是最可靠的开火窗口，代码完全没有利用。改动点：在 `cpp:107-110` 检测 `vx` 符号翻转（或 |vx| 连续下降），置"折返窗口"标志，在 `cpp:232` 的 AND 条件里放宽 2/3 两项。
6. **距离标定未用于提前量/视差**：`box_w → 开火死区` 的线性映射只服务死区（`cpp:491-515`）；`laser_offset_x/y` 是固定 0（`cpp:920-921`）；`compute_comp_pixel_0()` + `lookup_table` 是**死代码**（`cpp:759-770`，表文件 `offcenter_table.txt` 24 行全 0）。改动点：抽出 `boxWidthToDistance()`（复用 `box_w_far_/near_` 标定），同时供提前量换算（px↔m↔角度）与动态视差补偿。
7. **丢帧判定按"帧数"而非时间，且 TRACK 期间云台冻结**：`lost_frames_to_rescan_=180`（部署，`robot.yaml:152`）≈6s；这 6s 内 `CaremaMontorControl` 在 TRACK 直接 return（`cpp:627-644`），云台一动不动 —— 移动靶一旦滑出画面就是纯浪费。改动点：改为按时间阈值（如 300ms）判丢；TRACK 丢帧时进入"沿最后运动方向的小幅搜索扫"而不是冻结。
8. **单目标连发计数会被"重新锁定"清零**：`shots_on_target_ = 0` 只在进入 TRACK 时重置（`cpp:55`）。移动靶反复出/入视场 → 上限失效，10 发弹药可能全倾泻在一个靶上；反之，若靶 6s 内一直判为丢失，云台无法继续射击。改动点：把"目标身份"与"目标计数"解耦（见第 9 条），或按时间窗（如 10s 内对同类靶累计）计发数。
9. **无多目标/无目标 ID**：视觉每帧只发一个框（`det_node.cc:242-260`），云台侧无关联逻辑。若场上同时有 2 个靶（例如"一块还是两块靶板"的现场歧义 + 干扰靶），无法稳定锁定同一个。改动点：`det_node.cc:242-260` 改为逐目标发布（数组分段或自定义 msg 带 id/序号），云台侧增加 id 关联与"最近目标优先"策略。
10. **类别迟滞 + 仅蓝靶叠加阻塞**：`cls_switch_confirm_frames_=5`（`cpp:126,209`）+ `fire_only_blue_=true`（`cpp:215,223`）→ 移动靶经过反光/遮挡时类别抖动会整段掐断开火。改动点：把 `locked_cls_` 的判定从"连续 5 帧相同"改为"滑动窗口多数票"，并在持续锁定同一 id 时锁定类别不再改（`cpp:198-219`）。
11. **射击许可门缺失**（§6.4）：`/move_base/stop_signal` 无人订阅，云台的行进中开火不会被循迹的"停稳"约束。改动点：在 `cpp:838-863` 增加 `sub_stop_signal`（UInt8），把 `stop_signal==1` 作为 `cpp:232` 开火 AND 条件之一；同时保持"无许可时不开火但继续跟踪"。
12. **无弹药/命中闭环**：不订阅 `HpAndHitmsg`（`judgment.cpp:202` 发布）。改动点：订阅后以 `data[4]`(ammo) 做弹药对账、`data[3]`(Shooting_Count) 与软件侧发数交叉校验、**`data[0]`(targethp) 下降量作命中判据**。
    ⚠️ **本条的早期写法有语义错误，勿照抄**：`data[2]`(`Bing_hit`) 是"**我方**被击打方位"（0x10=未被击打，见 `judgment.h:89`），拿它做"命中即停"会把**我们挨打**当成打中靶 —— 每挨一发就停火一次，而真正打中时毫无反应。命中真信号只有 `data[0]` 下降。
    ⚠️ 另注：该话题 1 Hz 心跳与实帧**值完全相同**，闭环必须按**状态变化**计数，不能按"收到帧"计数。
    ✅ 新版**已在纯 Python 侧落地**（不改 C++、车端无需重编译）：见 `docs/新版架构设计.md` §5.3 与 `src/dzbrain/dzbrain/io/referee.py`。
13. **视觉流水线 0.26s 陈旧**（§9.2）：`thread_num=8` 是延迟主因。改动点：`thread_num` 降到 2~3，或把 `rknnPool::get()`（`rknnPool.hpp:102-111`）改为"取最新已完成的结果、丢弃过期 future"。
14. **水平跟踪被锁在扫描区间**（§4.2）：部署 [−160°,+10°]（`robot.yaml:36-37`）；若靶在车右前方，跟踪不可达。改动点：跟踪与扫描使用**两套独立限位**（跟踪走 `clampYawTick` 机械限位 + 软限位参数），或在 `cpp:160` 使用独立的 `track_yaw_min/max_deg`。
15. **扫描俯仰 +6° 看不见地面靶**（§3）：`robot.yaml:44`（+6.0）与 `cpp:140-151` 的实测结论冲突。改动点（现场优先）：把 `scan_pitch_offset_deg` 改到 −20~−26 并复测"扫描能否看到靶"；或恢复多层 `scan_pitch_levels`（代价：一轮扫描时间 ×层数）。

---

## 11. 复杂度热点与拆分建议（为新版本重写准备）

| 热点 | 规模 | 问题 | 拆分建议 |
|---|---|---|---|
| `callback_offset_center` | `cpp:24-326`，约 300 行 | 解析+守卫+滤波+伺服+类别迟滞+弹药+开火+语音全塞在一个回调 | 拆 `VisionFrame{解析}` → `Tracker{EMA/速度估计/丢帧}` → `Servo{PD/钳位/前馈}` → `FireController{许可链/冷却/弹药}`，各自可单测 |
| 节点类成员 | `h:71-279`，60+ 成员 | 参数、运行时状态、缓冲区混在一起 | 提 `GimbalParams` / `TrackerState` / `FireState` 三个 POD，参数由 YAML→结构体一次装载（或 `generate_parameter_library`） |
| 参数声明/读取 | `cpp:871-1000`，约 130 行样板 | 声明与 get 两段手工同步，漏一处即"参数静默失效"（`launch.py:9-16` 记录了历史教训） | 用参数结构体 + 表驱动装载，保留 healthcheck 的"declare/get 成对"检查 |
| `CaremaMontorControl` | `cpp:623-719` | 三状态 + 归位 + 扫描推进 + 超时兜底耦合，扫描推进嵌在时间判断里 | 状态模式：每状态一个 `tick()`，扫描推进独立成 `ScanPlanner` |
| `Control()` 主循环 | `cpp:1109-1153` | spin/控制/发布/保活/变频 混在一起 | 拆 `spinOnce()`、`publishCommand()`、`publishStatusKeepalive()`、`rateFor(state)` |
| 死代码/遗留 | `compute_comp_pixel_0`+`lookup_table`（`cpp:759-770,1101`）、`scan_yaw_dirs_`（`h:147`）、`scan_yaw_idx_`（`h:221`）、`aim_status_tick_`（`h:113`）、`cls>2` 语音分支（`cpp:267-280`）、`h:43-59` 中本节点未用的字段（TargetAngle/SpeedDirection/LaserGunSwitch） | 增加阅读与重写成本 | 重写时删除；`offcenter_table.txt` 若确认无用则移出安装项（`CMakeLists.txt:71-72`） |
| 线程/延迟模型 | `det_node.cc:228-238` + `rknnPool.hpp:91-111` | FIFO 流水线把延迟固定在 `thread_num` 帧，且 `get()` 阻塞在单线程 executor 里 | 重写为"最新帧优先 + 有界队列 + 显式延迟记账" |
| 依赖卫生 | `CMakeLists.txt:24-49`（tf2/geometry_msgs/Boost 全列，代码基本不用） | 编译期噪音与升级风险 | 按实际 include 收敛依赖 |

---

## 12. 结论要点（供出车决策）

1. 云台节点**功能完整、可运行**：三状态机、连续往复扫描、PD 视觉伺服、双层钳位、弹药/冷却/类别门、手动接管、状态上报齐备。
2. 开火链路**在代码上是通的**：`LaserShot_Command=1` → `dzjudgment` → 串口 `B5 5B 03 21 01 25`，成功有明确日志（`judgment.cpp:390-391`）；`dzjudgment` 必须被启动（`start_all.sh` 历史坑见 `robot.yaml:191-202`）。
3. 打移动靶的主要短板集中在**"没有运动学"**：无速度估计（只有单帧差分）、预判仅 0.5 帧、无折返点策略、开火要求画面 1.5px/帧 内稳定、TRACK 丢帧冻结最长 6s。
4. **三个必须先核实的部署项**：① `scan_pitch_offset_deg=+6.0` 与"地面靶需低头 ~40°"冲突（`robot.yaml:44`）；② 跟踪水平限位被扫描范围压到 [−160°,+10°]（`robot.yaml:36-37` + `cpp:160`）；③ `/move_base/stop_signal` 放行门无订阅者（`path_executor.py:2128` 发 → 无人接）。
5. 视觉端每帧只报一个目标、且结果滞后约 8 帧，是"多靶/移动靶"场景下最容易被忽略的两个前提条件。
