---
来源: 本次代码级技术审计（2026-10-23，子智能体）
取证: 实
更新: 2026-10-23
status: 现行
---

# 技术全景 B：车端任务层与感知（tasks/* + scout/*）

> 审计对象：dzrobot_v2/src/dzbrain/dzbrain/ 下 8 个任务线程 + 5 个 scout 感知模块。
> 审计方式：逐文件精读源码（非文档转述），每条结论附 路径:行号 或测试名；无法证实者一律写「未核实」。
> 本文档只读代码，未修改任何源码或配置；唯一写入目标是本文件。

---

## §0 导读

### 0.1 这份文档讲什么

车端 dzbrain 是一个单进程、多线程的决策层：8 个 TaskThread 子类各自跑固定频率，通过
Blackboard（高频共享状态）与 EventBus（低频事件）通信，禁止任务之间直接互相调用
（core/task.py:13-14）。本文档覆盖其中承担「动作执行 + 感知」的 8 个任务与机侧侦察模型 scout/。

一句话总览（按数据流方向）：

    bridge.pose()        --> LocalizationTask --> P_POSE / P_ROUTE_S / P_MOTION_LIMIT
    bridge.vision()      --> AimTask           --> P_TARGET / P_AIM / P_AIM_RED / fire_ready / lead
    bridge.take_drone_...--> DroneTask         --> P_DRONE / P_FIELD_LAYOUT / P_FIELD_VERIFY / P_DRONE_COVERAGE
                                              MissionTask  --> P_MISSION_PHASE / P_STOP_POINT / P_ROUTE / fire_gate
                                                    |--> TrackerTask --> 底盘指令（唯一写入者）/ P_TELEMETRY
                                                    |--> FixTask     --> P_FIX_EST
                                                    |--> TelemetryTask --> P_TELEMETRY 子键
                                         SafetyTask --> P_SAFETY（唯一强制停车者）

### 0.2 频率速查（构造时写死，见 TaskThread.__init__）

| 任务 | 频率 | 代码位置 |
|---|---|---|
| TrackerTask | 50 Hz | tasks/tracker.py:126 |
| AimTask | 50 Hz | tasks/aim.py:514 |
| LocalizationTask | 20 Hz | tasks/localization.py:162 |
| SafetyTask | 20 Hz | tasks/safety.py:56 |
| MissionTask | 10 Hz | tasks/mission.py:1482 |
| FixTask | 10 Hz | tasks/fix.py:90 |
| DroneTask | 5 Hz | tasks/drone.py:534 |
| TelemetryTask | 2 Hz（线程）/ 0.5 s 打印 | tasks/telemetry.py:212, :222 |

注意：任务线程频率与「实际业务节拍」并不总是相等。Telemetry 线程 2 Hz，但 period_s 可配
（默认 0.5 s = 2 Hz，telemetry.py:222）；Safety 20 Hz 但有启动宽限 _startup_grace_s = 3.0 s
（safety.py:76, 108-113）。

### 0.3 关键数字（本次审计直接读到的）

| 项 | 值 | 证据 |
|---|---|---|
| 任务线程总数 | 8 | tasks/*.py 中 8 个 TaskThread 子类 |
| 单拍异常阈值（连续） | 5 拍 -> FAULT | core/task.py:63, 136-138 |
| 地图格数 | 14 列 × 11 行 = 154 | scout/fov.py:71-78, scout/layout.py:330 |
| 格边长 BLOCK_SIZE | 0.35 m | scout/fov.py:7 |
| 车体尺寸默认 | 长 0.28 m / 宽 0.135 m / 参考点偏移 0.0 m | tasks/mission.py:709-712 |
| 到达半径 pursuit.arrival_m | 0.12 m | tasks/mission.py:1539, config.py:155 |
| 末点停止半径 _GOAL_STOP_M | 0.03 m | tasks/mission.py:222, 3102 |
| PIVOT 触发角 / 距离 / 容差 / 超时 | 135° / 0.25 m / 15° / 14 s | config.py:92-116 |
| 停站三段超时 | SETTLE 0.8 s（兜底 0.4×30=12 s）/ ACQUIRE 18 s / FIRE 30 s | tasks/mission.py:1520-1538 |
| 到站级硬死线 _stop_deadline_s | 默认 = mission.shoot_timeout_s = 30 s | tasks/mission.py:1526-1537 |
| 拐角绕圈兜底 | 12 s 停滞 或 累计转 360° | tasks/mission.py:1544-1545, config.py:361-364 |
| 静止闸 | 10 拍位移 < 0.01 m 且 v < 0.03 m/s | tasks/mission.py:229-231, 3229 |
| 全场弹药 / 单靶上限 | 10 / 3 发 | config.py:376-377 |
| 两发最小间隔 | 500 ms | config.py:378 |
| 许可最短保持 hold_min_s | 0.30 s | config.py:418, tasks/aim.py:584 |
| 视觉新鲜度上限 VISION_MAX_AGE_S | 0.5 s | tasks/aim.py:107 |
| 丢靶回扫帧阈值 | 180 帧（@30 fps = 6 s） | config.py:381 |
| 靶轨道行程 / 移速 | 0.70 m / 0.04 m/s | config.py:374-375 |
| 提前量 lead_time_s 默认 | 0.0 s（SCHEMA 默认值，现场需设） | config.py:370 |
| 侦察视场 | 4 格 × 2 格 = 1.40 m × 0.70 m（格点姿态恰好 8 格） | scout/fov.py:5-16 |
| 布局心跳周期 | 1 s | scout/heartbeat.py:22 |

### 0.4 阅读约定与免责

- 「当前 SCHEMA」指 core/config.py 中登记的键；文中凡说「未登记进 SCHEMA」均指
  core/config.py 的 SCHEMA 表里没有该键，而代码用 cfg.get(key, default) 兜底。
- 本审计的 路径:行号 相对仓库根 dzrobot_v2/，如 src/dzbrain/dzbrain/tasks/mission.py:1482；
  正文中为简洁写 tasks/mission.py:1482。
- 文档中大量引用了源码 docstring 里的「为什么」说明（开发过程中记录的坑与实测证据）。
  它们是设计说明而非本审计独立复现的证据；凡属 docstring 断言、本审计未复跑者，均在 §7 标注「未核实」。

---

## §1 模块地图

### 1.1 目录与职责（本次审计逐文件数出）

dzbrain/ 下共 35 个 .py（审计期间 route/planner.py 已存在），按目录分组：

| 目录 | 文件数 | 职责（一句话） |
|---|---|---|
| core/ | 8 | 基础设施：Blackboard、Config、状态类型、事件总线、几何、任务线程基类、日志 |
| io/ | 4 | 桥接：ROS 话题、契约常量、裁判接口 |
| pursuit/ | 4 | 循迹算法：纯追踪控制器、前视距离、路径投影 |
| route/ | 5 | 路线：模型、解析器、物理几何、规划器、校验器 |
| tasks/ | 9 | 本文档主战场：8 个任务线程 + __init__.py |
| scout/ | 6 | 本文档主战场：机侧侦察模型 + __init__.py |
| tools/ | 3 | 仿真与 CLI |
| 顶层 | 2 | __init__.py、__main__.py |

### 1.2 本次审计覆盖的文件清单（行数实测）

| 文件 | 行数 | 类型 |
|---|---|---|
| tasks/aim.py | 1368 | 任务线程 |
| tasks/drone.py | 1196 | 任务线程 |
| tasks/localization.py | 1444 | 任务线程 |
| tasks/mission.py | 4007 | 任务线程 |
| tasks/tracker.py | 892 | 任务线程 |
| tasks/safety.py | 529 | 任务线程 |
| tasks/fix.py | 461 | 任务线程 |
| tasks/telemetry.py | 295 | 任务线程 |
| tasks/__init__.py | 2 | 包标记 |
| scout/layout.py | 704 | 感知模型 |
| scout/frames.py | 250 | 感知模型 |
| scout/fov.py | 225 | 感知模型 |
| scout/heartbeat.py | 114 | 感知模型 |
| scout/reference.py | 90 | 感知模型 |
| scout/__init__.py | 130 | 一站式入口 |
| 合计 | 11 707 | — |

### 1.3 任务线程一张表

| # | 任务 | 频率 | 线程内写的关键黑板键 | 订阅的事件 | 下游消费者 |
|---|---|---|---|---|---|
| T1 | LocalizationTask | 20 Hz | P_POSE P_POSE_RAW P_ODOM_AGE P_ROUTE_INDEX P_ROUTE_S P_FIX P_MOTION_LIMIT | — | Safety / Tracker / Mission / Fix / Telemetry |
| T2 | SafetyTask | 20 Hz | P_SAFETY | EV_TASK_FAULT | Tracker / Mission / Aim |
| T3 | TrackerTask | 50 Hz | P_TELEMETRY[tracker_cmd] P_ROUTE_INDEX P_ROUTE_S P_PURSUIT_GEAR | — | Mission / Telemetry / Safety |
| T4 | MissionTask | 10 Hz | P_MISSION_PHASE P_ROUTE P_ROUTE_INDEX P_ROUTE_S P_STOP_POINT P_TELEMETRY[mission_state] | EV_BRIDGE_INFO | Tracker / Aim / Telemetry / Drone |
| T5 | AimTask | 50 Hz | P_TARGET P_AIM P_AIM_RED P_TELEMETRY[aim_cmd] | — | Mission / Telemetry |
| T6 | FixTask | 10 Hz | P_FIX_EST | EV_POSE_ANCHORED | Localization / Telemetry |
| T7 | DroneTask | 5 Hz | P_DRONE P_FIELD_LAYOUT P_FIELD_VERIFY P_DRONE_COVERAGE | EV_FIELD_VERIFY | Mission / main |
| T8 | TelemetryTask | 2 Hz | P_TELEMETRY[telemetry] P_TELEMETRY[telemetry_line] | — | 外部（日志/地面站） |

注：P_TELEMETRY 是约定共享 dict，各任务只往里塞自己的子键，绝不整键覆盖
（telemetry.py:11-18, tracker.py:70-72, mission.py:213-214, aim.py:103-104）。

### 1.4 支撑类型（本审计只读）

- core/state.py:17 SafetyState(ok, reasons, since, stops)
- core/state.py:45 StopPointState(index, block, kind, phase, group, started_at, ...)
- core/state.py:68 TargetState(seen, dx, dy, cls, box_w, seen_at, lost_since, s_rail, v_rail, ...)
- core/state.py:86 AimState(status, fire_gate, lead_px, shots_fired, hits, ammo, cooled_until, last_reason)
- core/state.py:103 DroneState(online, last_heartbeat, bridge_half, bridge_block, bridge_confidence, bridge_ts, field_diff, ...)
- core/state.py:127 FixState(dx, dy, dyaw, scale, anchor_count, last_anchor, drift_m, max_drift_m)
- core/state.py:151 Telemetry(phase, block, x, y, yaw_deg, speed, steer_deg, seg_index, s, route_len, aim, target, shots, hits, ammo, safety_ok, drone, extra)
- io/contract.py:172 AimStatus ∈ {IDLE=0, SCAN=1, TRACK=2}
- io/contract.py:184 MissionPhase ∈ {IDLE, DRIVE, APPROACH, STOP, SHOOT, WAIT_SCAN, HOLD, FINISH, SAFE_STOP}
- io/contract.py:202 ShotClass ∈ {NONE=0, BLUE=1, RED=2}
- core/task.py:30 TaskState ∈ {INIT, RUNNING, DEGRADED, FAULT, STOPPED}

---


## §2 逐模块详解

### §2.1 tasks/localization.py —— LocalizationTask（20 Hz，T1）

#### 2.1.1 职责

把原始里程计变成「信念位姿 + 路线进度」，并在航点处吸附纠偏（模块 docstring，localization.py:2-39）。
本文档只依据源码行为，docstring 的「为什么」标注为设计说明。

#### 2.1.2 构造与配置（localization.py:158-348）

| 参数 | 键 | 默认 | 位置 |
|---|---|---|---|
| pose_timeout_s | safety.pose_timeout_s | 1.0 s | :169 |
| arrival_m | pursuit.arrival_m | 0.12 m | :170 |
| drift_warn_m | pursuit.drift_warn_m | 0.30 m | :171 |
| anchor_enabled | fix.anchor_at_waypoint | bool | :172 |
| drift_seg_gate_m | pursuit.drift_seg_gate_m | 0.18 m | :177 |
| stuck_ticks / stuck_eps_m / stuck_dist_m | localization.stuck_* | 5 / 0.01 / 0.20 | :182-184 |
| stuck_trust_dist_m | 常量 _STUCK_TRUST_DIST_M | 0.45 m | :188 |
| yaw_guard_enabled | localization.yaw_guard | bool | :192 |
| yaw_diff_gain_dps_per_mps | localization.yaw_diff_gain_dps_per_mps | 525.0 | :194-196 |
| yaw_diff_spin_dps | localization.yaw_diff_spin_dps | 60.0 | :197-198 |
| yaw_jump_deg | localization.yaw_jump_deg | 30.0 | :199-200 |
| yaw_clear_ticks | localization.yaw_clear_ticks | 20 | :201-202 |
| yaw_imu_check_dps | localization.yaw_imu_check_dps | 30.0 | :203-204 |
| yaw_static_dps | localization.yaw_static_dps | 45.0 | :207-208 |
| yaw_min_speed_mps | localization.yaw_min_speed_mps | 0.03 m/s | :209-210 |
| yaw_bad_ticks | localization.yaw_bad_ticks | 3 | :211-212 |
| yaw_plausible_speed_mps | localization.yaw_plausible_speed_mps | 0.70 m/s | :214-215 |
| yaw_max_dt_s | localization.yaw_max_dt_s | 0.60 s | :217-218 |
| yaw_imu_mismatch_ratio | localization.yaw_imu_mismatch_ratio | 0.35 | :219-220 |
| anchor_max_pull_m | localization.anchor_max_pull_m | = arrival_m | :250-251 |
| anchor_reject_eps_m | localization.anchor_reject_eps_m | 0.005 m | :254 |
| drift_slow_after / drift_slow_speed_mps | localization.drift_slow_* | 3 / 0.08 | :256-257 |
| drift_stop_after | localization.drift_stop_after | 8 | :259 |
| drift_clear_ticks | localization.drift_clear_ticks | 5 | :261 |
| boundary_guard_m / boundary_guard_speed_mps | localization.boundary_guard_* | 0.10 / 0.08 | :263-265 |
| boundary_stop_m | localization.boundary_stop_m | 0.03 | :267 |
| reanchor_pull_max_m | localization.reanchor_pull_max_m | 0.35 | :271 |
| anchor_scale_max | localization.anchor_scale_max | 0.08 | :279 |
| implied_scale_window | localization.implied_scale_window | 4 | :281 |
| motion_heading_eps_m | pursuit.motion_heading_eps_m | 0.001 | :311 |
| yaw_trim_deg | localization.yaw_trim_deg | 0.0 | :299 |

#### 2.1.3 tick 主流程（localization.py:359-446，逐行）

1. pose = self.bridge.pose()；age = inf if pose is None else self._float_or_inf(self.bridge.pose_age())（:364-365）。
2. 拿不到位姿就明确「没有」：P_ODOM_AGE = inf，不写 P_POSE，返回（:366-373）。missing_pose_ticks 计数：首拍必打一条日志，之后限流（:370）。
3. 有位姿：写 P_ODOM_AGE = age（:375）。
4. 航向完整性守卫 _update_yaw_guard 先于任何用 yaw 的计算（:377-380）。
5. 行进方向 _update_travel_heading（:382-383）。
6. 应用 P_FIX：cx, cy, cyaw = fix.corrected(...)；若 yaw_trim_deg 非零再叠加 normalize_angle(cyaw + radians(yaw_trim_deg))；写 P_POSE_RAW（:386-392）。
7. 路线投影 _project(cx, cy, cyaw)，写 P_ROUTE_INDEX/P_ROUTE_S（:394-399）。
8. 到位吸附 _anchor(...)；未吸附则 _reanchor(...)；仍失败则按顶点漂移判定 unhealthy（:401-429）。
9. 信念漂移护栏 _update_drift_guard(belief, unhealthy=unhealthy)（:431-435）。
10. 写 P_POSE：吸附后重新投影，保证 P_ROUTE_S 与信念一致（:437-446）。

#### 2.1.4 投影与自愈（localization.py:473-659）

_project 的关键调用形状（:521-523）：

    proj = self._path.project(x, y, s_hint=self._s_hint,
                              window_m=self.arrival_m, allow_backward=False,
                              motion_heading=heading)

docstring 说明（:476-489）：allow_backward=True 会把窗口下界折算成 t_floor 并夹住本段 t，当 window_m 大于「本拍到 hint 的距离」时投影每拍都被夹回 s_hint，形成吸收态。本任务用 window_m = arrival_m + allow_backward=False 保证 P_ROUTE_S 单调（P1 §3.1 验收）。

_heal_stuck 的三条判据（:570-579）：
1. proj.s 相对窗口起点前进 < stuck_eps_m 且持续 ≥ stuck_ticks 拍；
2. 位姿在这几拍移动 > arrival_m；
3. proj.dist > stuck_dist_m。

全路线重投影的裁决顺序（:600-615）：有 _travel_heading 时不传 s_hint 按方向重扫；否则退回「保留 hint 连续性」。大跳跃可信闸（:622-631）：

    if abs(jump) > _STUCK_MAX_JUMP_M (0.50) and glob.dist > self.stuck_trust_dist_m (0.45):
        拒绝（stuck_heal_refused++），保留窗口投影

_stuck_check（:644-659）维护滑动窗口；passed_end 直接复位。

方向一致性闸 _opposes_travel（:922-940）：投影段几何方向与 _travel_heading 夹角 > 150° ⇒ 拒绝本次推进，沿用上一拍 _last_proj（direction_rejects++）。阈值 150° 而非 90° 的理由：90° 直角弯、135° PIVOT 换向都属正常（docstring :929-931，设计说明）。

#### 2.1.5 航向完整性守卫（localization.py:664-844）

判据链（:664-760）：

1. 帧间隔 dt = frame_t - prev[0]：dt <= 1e-6 跳过（同帧重复读）；dt > yaw_max_dt_s 跳过并清 _yaw_bad_run（:707-713）。
2. 位姿连续：v = hypot(Δx, Δy)/dt > yaw_plausible_speed_mps ⇒ 跳过（:714-718）。
3. 差速包络（:762-767）：
    运动（v >= yaw_min_speed_mps）：omega_max = max(yaw_diff_spin_dps, yaw_diff_gain_dps_per_mps * v)
    静止（v < yaw_min_speed_mps）：omega_max = yaw_static_dps
4. 单帧跳变 |Δyaw| > yaw_jump_deg ⇒ 立即判故障（:751-756）。
5. IMU 交叉核对 _yaw_imu_mismatch（:769-776）：rate >= yaw_imu_check_dps 且 imu_rate < rate * ratio ⇒ 矛盾。
6. 连续 yaw_bad_ticks 个新帧才判故障（:758-760）。

手动旋转/遥控接管不判故障（但 IMU 矛盾不豁免，:731-740）。

故障闩锁与恢复（:778-844）：判故障 → yaw_untrusted=True，发 EV_TASK_FAULT（where="heading_untrusted"）+ 后续每拍经 P_MOTION_LIMIT{speed_mps:0} 请求停车；连续 yaw_clear_ticks 个可信新帧后自动复位；也可 reset_yaw_guard() 人工复位。Safety 的 SAFE_STOP 仍需显式 clear_safety()。

#### 2.1.6 到位吸附三道闸（localization.py:942-1028）

_anchor 的触发条件（:962-975）：候选 = _anchor_candidates；末点半径收紧到 _GOAL_STOP_M(0.03)，非末点用 arrival_m(0.12)。找到 < radius 的航点后：

- 闸① 位移上限：drift > anchor_max_pull_m ⇒ _refuse_anchor，本航点不再尝试（:981-985）。
- 闸② 误差闸：吸附后离线距离不得变大（_error_shrinks，:987-991）。
- 闸③ 尺度误差闸：_scale_suspected 非 None ⇒ 拒绝（:993-1001）。

通过三闸后：anchored++、drift_run = 0、记录隐含尺度、写 P_FIX（_store_drift）、发 EV_POSE_ANCHORED、_report_drift(anchored=True)（:1003-1028）。

#### 2.1.7 尺度误差的判别（localization.py:1030-1117）

- _error_shrinks（:1030-1044）：after.dist <= proj.dist + anchor_reject_eps_m。
- _along_ds（:1046-1054）：after.s - proj.s（带符号沿程修正量）。
- _scale_suspected（:1056-1095）：连续 ≥3 次吸附同号 + 幅度中位数 ≥ 0.5×arrival_m + 隐含尺度 |est| ≥ anchor_scale_max(8%)。
- _implied_scale（:1097-1117）：ds / (proj.s - last_anchor_s)，base < 0.10 时不判。

#### 2.1.8 漂移护栏与边界护栏（localization.py:1230-1328）

_distance_to_boundary（:1230-1240）用与 Safety 同源的 geo.map_bounds(margin)。

_update_drift_guard 的优先级（:1279-1300）：

    level = none / slow / stop
    drift_run >= drift_stop_after(8)      -> stop, 0.0, "drift_stop"
    drift_run >= drift_slow_after(3)      -> slow, drift_slow_speed_mps(0.08), "drift"
    boundary == stop                      -> stop, 0.0, "boundary_stop"
    boundary == guard (且非 stop)         -> slow, min(boundary_guard_speed_mps, speed)
    yaw_untrusted                         -> stop, 0.0, yaw_reason   # 最高优先级
    healthy_run >= drift_clear_ticks 且 drift_run==0 -> clear（写 None）

状态变化才写 P_MOTION_LIMIT（:1300-1328）；payload 字段（speed_mps, reason, drift_run, anchor_refused, distance_to_boundary_m, implied_scale_median, scale_suspect, yaw_untrusted, yaw_guard_recoveries, yaw_rate_dps, yaw_limit_dps, since）。

#### 2.1.9 关键不变量与故障闩锁清单

- P_ROUTE_S 单调性由 allow_backward=False + _opposes_travel 保证（:481-484, 922-940）。
- 卡死自愈「大跳跃」被拒时不抬 _s_hint（:630），避免把车瞬移到错误支。
- 锚定失败显式记账 anchor_refused（:1128-1132），绝不静默。
- 边界告警去重：进入护栏区打一条、离开复位（:1263-1277）。

#### 2.1.10 潜在问题（本次审计发现）

1. U+26A0 / U+2705 / U+2713 / U+1F9E9 / U+1F6D1 / U+21A9 / U+26D4 在 GBK 控制台不可编码：
   本文件多处 self._log(...) 字面量含这些字符（localization.py:457 ✅/❌、:535 ⛔、:626 ⛔、:635 ⚠️、:810 🧭❌、:827 ✅、:1131 ⛔、:1223 ↩️、:1265 ⚠️、:1307 ✅、:1326 🛑、:1427 ⚠️）。
   core/task.py:82 的默认 logger 是 print，在 Windows GBK 控制台上会抛 UnicodeEncodeError，把整拍 tick 打断在半路（日志没打出来、后半拍发布也没执行）。
   证据：本审计实测 chr(0x26A0).encode('gbk') 抛 UnicodeEncodeError；chr(0x2605)（★）可编码，chr(0x2705)（✅）不可。
   属系统性问题，非本文件独有（见 §2.10 汇总）。
2. _implied_scale 的分母可能为 0/负：base < 0.10 已挡（:1115），但 proj.s - last_anchor_s 在吸附把 last_anchor_s 推到 proj.s 之后，下一次 base 可能极小（依赖 _last_anchor_s 的更新时机，:1016-1020），已用 < 0.10 兜住，未核实边界情形。

---

### §2.2 tasks/safety.py —— SafetyTask（20 Hz，T2）★ 唯一强制停车者

#### 2.2.1 定位

模块 docstring 明确：刹车决策只在这一个文件里做，判据只读 Blackboard/事件，与车正在执行哪个状态机无关（safety.py:2-9）。恢复语义刻意不做自动恢复，必须显式 clear_safety()（:20-22）。

#### 2.2.2 配置（safety.py:62-77）

| 参数 | 键 | 默认 | 位置 |
|---|---|---|---|
| pose_timeout_s | safety.pose_timeout_s | 1.0 s | :63 |
| task_stall_s | safety.task_stall_s | 1.0 s | :64 |
| brake_on_fault | safety.brake_on_fault | bool | :65 |
| brake_fail_alarm_after | safety.brake_fail_alarm_after | 3 | :71 |
| map_margin_m | pursuit.map_safety_margin_m | 0.0 | :72 |
| startup_grace_s | safety.startup_grace_s | 3.0 s | :76 |
| 限流 | logging.rate_limit_s | 1.0 s | :77 |

#### 2.2.3 tick 主流程（safety.py:143-177）

1. _pump_events()（订阅 EV_TASK_FAULT，:132-133, 495-509）。
2. 取 P_POSE；reasons = _evaluate(pose, now)（:147-151）。
3. 有 reasons → _update(reasons, now)（写 P_SAFETY.ok=False + 逐拍发零速）。
4. 无 reasons 但 stopped → 记 auto_recover_count（不自动恢复，:155-165）。
5. 健康 → 仅当 P_SAFETY 不存在/非 OK 时写一次 OK（:166-177）。

#### 2.2.4 五道检查（safety.py:221-328）

| 编号 | 判据 | 代码 |
|---|---|---|
| ① 定位失联 | 启动期从未见过新鲜位姿且过 startup_grace_s；运行期 pose_age==inf 或 odom_inf_long（持续 > pose_timeout_s）或 pose_age > timeout 或 odom_age > timeout；Localization 心跳 age > 2×timeout | :235-314 |
| ② 越界 | pose is not None and not geo.in_bounds(x, y, map_margin_m) | :316-320 |
| ③ 任务健康 | group 中任一任务 state==FAULT 或心跳 age > task_stall_s（跳过自己） | :330-382 |
| ④ EV_TASK_FAULT | _fault_reasons 去重后全部上报 | :325-326 |
| ⑤ 遥控接管 | manual_override() 为真不算故障，只记录 | :224-233 |

关键细节：
- 键缺失（None）与值是 inf 必须区分（:236-239）。
- _ever_had_pose 记录「本进程见过定位」，避免启动竞态（:256-260）。
- _odom_inf_since 持续判据（:263-273）。
- 心跳 age 用注入时钟 _task_age，加 _CLOCK_SKEW_S=0.05（:384-389）。
- ③ 冷启动宽限：not h.last_tick_at and (now - _started_at) <= startup_grace_s（:372-377），判据不要求 state==INIT（docstring 说明 start() 先置 RUNNING 再跑首拍）。

#### 2.2.5 状态迁移与刹车（safety.py:403-476）

- _update：原因集合去重保序；集合变化才发一条 EV_SAFETY_STOP（:412-428）。
- first = not self.stopped 时 forced_stops++、auto_recover_count=0（:432-436）。
- 刹车每拍都发（不能被日志限流短路），成功才 brakes++（:441-451）。
- 连续 brake_fail_alarm_after 拍失败 → 独立 alarm（:452-459）；_publish_brake 如实返回成败（:461-475）。
- _state：since 只在「进入/离开故障」时更新，不每拍跳动（:477-490）。

#### 2.2.6 显式清场（safety.py:190-214）

clear_safety(operator, source)：幂等；清原因、发 EV_SAFETY_CLEAR、打横幅；返回新 SafetyState。

#### 2.2.7 潜在问题

1. 同 §2.1.10 的 GBK 炸弹：safety.py:163 banner 里的 ⚠️、:210 的 ✅、:424 的 🛑 在 GBK 下会炸。本审计实测 ⚠️/✅ 不可 GBK 编码。
2. _check_group 在 group is None 时跳过检查③并只打一条日志（:342-346）——若装配侧忘传 TaskGroup，任务健康看门狗整体失效，属静默降级（设计取舍，docstring 已注明）。

---

### §2.3 tasks/tracker.py —— TrackerTask（50 Hz，T3）★ 底盘指令唯一写入者

#### 2.3.1 职责与前置门（tracker.py:1-118, 326-354）

模块 docstring：前置门顺序 = 无位姿 → 安全非 OK → 遥控接管 → 阶段不允许 → 无路线；任一命中即 _hold() → publish_brake()。阶段白名单（:84-92）：

    ACTIVE_PHASES  = (DRIVE, APPROACH)
    BLOCKED_PHASES = (IDLE, STOP, SHOOT, WAIT_SCAN, HOLD, FINISH, SAFE_STOP)

_blocked_reason_now（:326-341）按严重度排序返回原因串。

#### 2.3.2 配置（tracker.py:135-236）

| 键 | 用途 | 位置 |
|---|---|---|
| logging.rate_limit_s | 限流 | :136 |
| pursuit.align_kp / align_steer_limit_deg / align_straight_speed_mps | 桥洞直行 | :137-139 |
| pursuit.align_straight_before / after | 桥洞直行前后格数 | :140-141 |
| pursuit.bridge_blocks | 桥洞格集合 | :142 |
| pursuit.max_steer_deg / max_steer_rate_deg_s | 舵角限幅 | :143-144 |
| pursuit.min_speed_mps / cruise_speed_mps | 速度 | :145-146 |
| pursuit.dynamic_gear_enabled | 动态档开关（默认 False） | :149 |
| pursuit.approach_m | 接近停站距离 | :152 |
| pursuit.pivot_enabled / pivot_angle_deg / pivot_dist_m / pivot_wait_s / pivot_speed_mps / pivot_yaw_tol_deg / pivot_timeout_s / pivot_kp | PIVOT | :190-197 |
| pursuit.motion_heading_eps_m | 行进方向位移门限 | :200 |
| pursuit.projection_rewind_m / reverse_entry_m / vertex_floor_m / projection_release_m | 投影参数 | :205-212 |

#### 2.3.3 tick 主流程（tracker.py:359-705）

1. dt = _tick_dt 或实测帧间隔；not (0 < dt <= 1.0) 退回标称（:361-364）。
2. 读 P_POSE/P_SAFETY/P_MISSION_PHASE/P_ROUTE，前置门（:366-374）。
3. path = _resolve_path(route)（带缓存，路线变化才重建 Path，:296-321）。
4. s_hint 取本任务上一拍 _seg_s，否则黑盒 P_ROUTE_S（:378-382）。
5. 行进方向取实测位移 atan2(dy, dx)；位移 < _motion_eps_m 时锁存上次可信值（:384-410）。docstring 明确禁止用「上一拍命令速度符号」（自指环，实测每拍翻转）。
6. proj = path.project(..., window_m, back_m=_rewind_m, s_min=_s_floor, motion_heading=...)（:412-416）。
7. 投影跟丢自愈（:418-449）：proj.dist > _release_m 且非推车时全路线重投影；只接受向前修正（proj_relock.s >= _s_floor）且 trusted（|jump| <= _RELOCK_MAX_JUMP_M=0.50 或 proj_relock.dist <= _RELOCK_TRUST_DIST_M=0.45）。
8. 弧长地板（:457-480）：贴到顶点（dist_to_vertex <= _vertex_floor_m）且下一段不是倒车段、且未过冲超限（_pre_refuse）时抬 _s_floor。
9. is_rev = path.segment_is_reverse(proj.seg_index)；gear = _gear_cruise(...)（动态档，默认关闭）。
10. cmd = compute_command(pose, path, proj, cfg, reverse=is_rev, ..., bb=self.bb, last_speed_mps=..., cruise_mps=gear)（:490-495）；_last_cmd_speed = abs(cmd.linear_x)（:499）。
11. align = _align_active(proj.seg_index)（:500）。
12. PIVOT：pivot = _pivot_update(...)（:502-503）。
13. 倒车入口闭环推车状态机（:505-599）：
    - 进入条件：not _rev_push_active and not _rev_push_gave_up and now >= cooldown and pivot is None and not is_rev and rev_next；_over 先算（过顶点用位移在段方向投影判），_over <= 0 时看 dist_to_vertex <= _reverse_entry_m；_over > _REV_PUSH_OVERSHOOT_M(0.65) ⇒ rev_push_refused++，不推。
    - 保持：每拍 -min_speed；退出：is_rev、或实测位移沿段方向反向 ≥ 2 cm、或 3 s 超时。
    - 超时：冷却 ≥ 2 s；连续 2 次超时 ⇒ 放弃闩锁（_rev_push_gave_up）。
14. 优先级选指令（:601-651）：PIVOT > rev_refuse（刹车）> give_up 闩锁（刹车）> 冷却窗口（刹车）> 推车（航向保持 + motion_limit_speed_limit 钳制）> align（桥洞直行）> 纯追踪 cmd。
15. 发布前最终限速 _mlim_final（:653-665）：所有分支汇入唯一 publish_command，用同一份读侧再钳一次；0 → 真 0，L>0 → 只降不升、|v|<=L、倒车保号。
16. publish_command(speed, steer)；倒车 motion_state 仅变化时发（:667-673）。
17. 写 P_ROUTE_INDEX/P_ROUTE_S（与 Localization 同一投影语义，:675-677）。
18. 写 P_TELEMETRY[tracker_cmd]（:680-705）。

#### 2.3.4 桥洞直行保护（tracker.py:258-291）

_ensure_align_segments 用航段索引集合而非几何距离（docstring 明确避免旧版「K 列附近 ±2 格全域锁航向」的僵直问题）。命中 K 列航点 h 后生成 [h-before, h+after] 段集合；单位是格，取整时减 EPS。

#### 2.3.5 PIVOT 原地掉头状态机（tracker.py:771-870）

PivotPhase = IDLE / SETTLE / TURN（:55-68）。

进入条件（:787-813）：
- _pivot_enabled；IDLE 态；_corner_ahead 非 None；
- s_v > _pivot_abandon_until_s（已处理顶点先开过去）；
- angle >= _pivot_angle_deg 且 dist_to_vertex <= pivot_dist_eff(...)（速度自适应触发距离）；
- not _align_active(...)（桥洞保护段内绝不做原地转向）。

_corner_ahead（:710-722）返回 (顶点弧长, corner_turn_deg(i), car_heading_on(i+1))——用车头转角与车头朝向，而不是行进方向夹角（docstring :711-717）。

完成（:842-858）：|err_deg| <= _pivot_yaw_tol_deg ⇒ pivot_count++，写 PIVOT_DONE_KEY = (顶点航点索引, 顶点弧长, "done")，_pivot_abandon_until_s = s_v + max(0.35, 2×触发距离)，_reset_pivot()。

超时（:817-829）：elapsed > _pivot_timeout_s ⇒ pivot_abandon_count++，同样写 PIVOT_DONE_KEY = (..., "abandon")，标记 _pivot_abandon_until_s，放行交回纯追踪。

TURN 段（:860-870）：steer = err_deg × _pivot_kp，限幅 _max_steer_deg，再做舵角速率限幅（与纯追踪同一条契约）。

关键修复：完成分支同样要标记 _pivot_abandon_until_s（否则车停在顶点邻域内下一拍立刻重入 PIVOT → 无限循环，实闭环 bench 在 F11 实测整场 240 s 只走到 4.4/34 m，见 docstring :851-856）。该断言属 docstring 证据，本审计未复跑 bench（§7）。

#### 2.3.6 动态档（tracker.py:745-769）

默认关闭（_gear_enabled=False）时 _gear_cruise 恒返回 None，compute_command(cruise_mps=None) 与历史逐位一致。开启时 _dist_to_next_cross 算车到前方最近 K 列格的直线距离（:724-743），dynamic_gear_speed(cfg, dist) 选 0.35/0.40，写 P_PURSUIT_GEAR 观测键。

#### 2.3.7 倒车入口推车闭环（tracker.py:105-117, 505-599）

三个常量：_REV_PUSH_REVERSED_M=0.02、_REV_PUSH_OVERSHOOT_M=0.65、_REV_PUSH_TIMEOUT_S=3.0、_REV_PUSH_COOLDOWN_S=2.0、_REV_PUSH_GIVEUP_STREAK=2。

docstring（:105-117, 505-517）给出的动机：底盘速度平滑窗约 0.4 s，旧「推一把」只推 2 拍，负速度还没填满平滑窗车就继续前冲 0.44 m ⇒ 过冲后判据永不成立 ⇒ s 冻死、mission 每 12 s 跳点。判据用实测位移而非命令符号（自指环）。

#### 2.3.8 超时 / 看门狗 / 闩锁

- PIVOT 硬超时 pivot_timeout_s（默认 14 s）。
- 推车 3 s 超时 → 2 s 冷却 → 连续 2 次 → 放弃闩锁。
- 无额外 watchdog；越界急停全部收敛到 SafetyTask。

#### 2.3.9 潜在问题

1. GBK 炸弹：tracker.py:540 ✓、:552 ⛔、:556 ⚠️、:578 ⛔、:818 ⚠️、:843 ✓ 均在 self._log(...) 字面量中；实测 ✓(U+2713)/⛔(U+26D4)/⚠️(U+26A0) 均不可 GBK 编码。
2. _pre_refuse 与弧长地板的交互（:464-477）：过冲超限时不抬地板，否则下一拍会被推到倒车段。该分支有测试覆盖（tests/test_tracker.py），本审计未逐条复跑（§7）。
3. _gear_cruise 每拍写黑盒（开启时）：P_PURSUIT_GEAR 是唯一观测键，实际生效路径是 cruise_mps 参数直传（:61, 765-768），读写语义已在 docstring 约定。


### §2.4 tasks/mission.py —— MissionTask（10 Hz，T4）★ 任务调度总纲

#### 2.4.1 职责与状态机

模块 docstring（mission.py:2-164）是本文件最长的自述，核心口径：

- 旧版打靶四阶段 SETTLE → ACQUIRE → STABLE → FIRE；新版把 STABLE 归 AimTask，Mission 只做三段式 SETTLE → ACQUIRE → FIRE（:6-10）。
- 每段独立超时，绝不阻塞主循环（:10-13）。
- 铁律：宁可漏打，不可卡死（:12-13）。
- 阶段集合：_DRIVING_PHASES = (DRIVE, APPROACH)、_TERMINAL_PHASES = (FINISH, SAFE_STOP)（:1445-1449）。

主循环（mission.py:2546-2582）：

    _drain_commands()                    # 外部 /nav/waypoint 命令
    _process_segment_requests()          # T27 步进循环段请求
    _refresh_intel()                     # 桥洞情报
    if P_SAFETY.ok == False: _enter_safe_stop(); return
    if phase == SAFE_STOP: _recover_from_safe_stop()
    _maybe_auto_start()
    if phase == FINISH:    延迟一拍复位回 IDLE
    elif phase in _DRIVING_PHASES:  _drive(); _maybe_enter_bridge_hold()
    elif phase == HOLD: _run_bridge_hold()
    elif phase == STOP: _run_stop_point()
    _check_finish()
    _publish_state(now)

#### 2.4.2 配置（mission.py:1491-1559）

| 键 | 默认 | 位置 |
|---|---|---|
| mission.auto_start | bool | :1492 |
| mission.auto_start_delay_s | 0.0 s | :1493 |
| mission.reset | "" | :1494 |
| mission.scan_mode | "" | :1495 |
| aim.gimbal_lock_enabled | True | :1507 |
| mission.shots_per_stop | 3 | :1508 |
| aim.red_early_leave_s | 3.0 s | :1516 |
| mission.skip_second_stop_after_hit | True | :1517-1518 |
| mission.settle_time_s / settle_eps_m / settle_speed_mps | 0.8 / 0.02 / 0.02 | :1520-1522 |
| mission.lock_timeout_s | 18.0 s | :1523 |
| mission.shot_timeout_s | 30.0 s | :1524 |
| mission.scan_stop_timeout_s | 8.0 s | :1525 |
| mission.stop_deadline_s（可选覆盖） | = shot_timeout_s | :1532-1537 |
| pursuit.arrival_m / stop_radius_m / approach_m | 0.12 / 0.17 / 0.60 | :1539-1541 |
| mission.stall_timeout_s / stall_yaw_deg | 12.0 s / 360° | :1544-1545 |
| planner.auto_plan_enabled / map_path / fallback_to_tokens / allow_reverse / max_seconds / engine / tracker_max_seconds | 见 §5 | :1547-1561 |
| drone.enabled / drone.info_stale_s | True / 30.0 | :1565-1569 |
| mission.bridge_info_wait_s / bridge_wait_at_start / bridge_wait_max_s / race_budget_s / bridge_reroute_max | 见 §5 | :1571-1591 |

#### 2.4.3 自动规划入口（mission.py:941-1071）

plan_route_from_config(cfg, *, map_path, log, cross_lower, cross_upper) -> PlanOutcome：

- 引擎分派：resolve_plan_engine（:421-436，planner.engine 别名容错，未知值回退 tour 并告警）；plan_budget_seconds（:439-461，tour 用 planner.max_seconds=2.0，tracker-safe 用 planner.tracker_max_seconds 缺省 5.0 / planner.TS_MAX_SECONDS）。
- 调 route.planner.plan_route(...)（tour）或 plan_route_tracker_safe(...)（tracker-safe）。
- 双射击停站收口（:1003-1022）：调 planner.ensure_second_shoot_stop(tokens, ...)，保证靶板两侧各留一个射击停站（靶板一面蓝一面红，蓝面每局随机）。
- 只有 tracker-safe 才跑双门禁 check_planned_route（:1031-1038）；tour 恒「不跑门禁 = True」（历史行为逐字节不变）。
- 足迹诊断对两种引擎都算（:1039-1047）。
- 失败语义由 planner.fallback_to_tokens 决定（:2640-2657）：true → warn 后回退 route.tokens 并出发；false → error 并抛 AutoPlanBlocked → 停在 IDLE。
- 全程宽捕获 Exception，绝不把 Mission 线程炸掉（:964-965, 1061-1071）。

PlanOutcome（:840-884）字段：ok/tokens/elapsed_ms/error/map_path/allow_reverse/route_len_m/reverse_segments/cost/engine/gate_ok/gate_detail/footprint/footprint_ok/cross_lower/cross_upper/shoot_stops/second_stop_added。

#### 2.4.4 双门禁（mission.py:464-640）

check_planned_route(tokens, map_cfg, cfg, log, route_check_module) -> GateResult：

- ① 内包 route.validate.validate_route（:588-598）；error 与 warn 分开收集。
- ② 用户侧 tools/route_check.py（:600-621）：找不到/加载失败不算失败（车端不部署 tools/），记 notes；执行时用本次规划真正用的地图要素覆盖它的几何输入（:543-567）。
- ③ 足迹诊断 footprint_diagnostic（:737-832）：折线口径、按 _FOOTPRINT_SAMPLE_M=0.05 采样；planner.footprint_gate 决定 warn（默认，只报）/ error（硬拦）（:633-639）。
- GateResult.ok = not validate_errors and not validate_warns and route_check_rc in (None, 0)（:631-632）。

vehicle_geometry（:682-712）的键路径坑：真源在 dzactuator.vehicle.*（robot.yaml 的 ros__parameters: vehicle: 段），SCHEMA 规范名是 vehicle.*；先用带段前缀的键再退规范名。

#### 2.4.5 段规划（mission.py:1172-1442）

plan_segment_route(map_cfg, cfg, *, from_cell, target_cell, heading_dir, engine, blocked_extra, source) -> SegmentPlan：

- 状态 = (格, 到达朝向, 上一腿朝向)，朝向感知 Dijkstra（:1326-1370）。
- 三条硬约束：物理净空（_TSGrid.move_ok）、禁 >90° 转角、禁紧凑 U 形折返（:1350-1356）。
- 整线引擎的策略性硬否决降级为代价 _SEG_SOFT_BAN_COST_M=2.0（:1357-1361）。
- 起点车头与唯一可行首步相反（180°）时允许「首步任意方向」重搜一次并留 warning（:1372-1380）。
- 段末停站类型 stop_kind ∈ {shoot, mine, path, end}（:1098-1114），shoot 时末 token 带 *（:1393-1397）。
- 自检：非正交相邻 / 进入 unknown 格 / move_ok 复核失败 ⇒ error（:1400-1412）。

#### 2.4.6 无人机桥洞情报 → 穿洞格（mission.py:296-302, 1937-2220）

BridgeIntel（:304-341）字段 half/cell/default/source/raw/reason/age_s；source ∈ {drone, default, stale, missing, offline, disabled, no_link, rejected}。

情报唯一真源 = EV_BRIDGE_INFO 事件流（:2125-2154）；P_DRONE 只判链路。关键动机（docstring :51-67, 1962-1975）：DroneState 只有一组 bridge 槽，_on_bridge_passable 覆盖式赋值，两半场稳态下槽里恒是 upper，只看快照会丢 lower。

采纳顺序（:2156-2193）：drone.enabled → 无 P_DRONE → 离线 → 从未收到 → 过期 → 格非法（_intel_cell_ok，:2044-2076：K 列 + 本半场行段，池外但在行段内也采纳、纠正过期地图）→ 采用。

换洞改写 rewrite_crossing_tokens（:363-418）：只认「进洞口 - K 列格 - 出洞口」同行三连，洞格全列表只出现一次；用词边界正则 retag_block_token（:344-351）避免 K1 命中 K10。_swapped_tokens_ok（:2474-2500）必须 validate_route 0 error 才放行；地图不可用则拒换。

#### 2.4.7 L10 桥前等待 + 迟到改道（mission.py:2267-2414）

- 决策点 _decision_point（:2276-2298）= 第一个「洞口尚未过」的穿洞三连的前一航点（默认 L10/J5）。
- _maybe_enter_bridge_hold（:2307-2338）：进 HOLD 条件 = 决策点已过 + 洞未决策过 + 两半场未齐 + 有链路且在线 + cap = min(bridge_wait_max_s, 预算余量) > 0。
- _run_bridge_hold（:2340-2355）：两半场齐或到点必发车（绝不卡死）。
- _release_bridge_hold → _reroute_at_decision（:2373-2414）：_install_route(..., keep_progress=True) 保留段索引，绝不把车送回起点；全程 ≤ mission.bridge_reroute_max（默认 1）。

#### 2.4.8 路线载入与停站台账（mission.py:2814-2995）

_install_route（:2857-2923）：单点路线 _path=None 防御；_stop_s = 所有 stop_points 的弧长 + 末点；_intel_used 路线载入即冻结；清掉残留 PIVOT_DONE_KEY。
_classify_stops（:2925-2951）：末点无 * 记 END，带 * 记 SHOOT；其余 shoot/scan。
stop_ledger / _stop_ledger_line（:2957-2995）按 射击/扫描/终点 分列计数（t7 收口，避免现场把终点误读成第二个射击停站）。

#### 2.4.9 航点推进与到达判据（mission.py:3031-3161）

_current_s（:3031-3060）：把 P_ROUTE_S 与当前位姿现算弧长比对，差 > arrival_m 则丢弃并写回。

_advance_waypoints（:3062-3161）逐航点判据：

    s_wp   = path.s_at_waypoint(i)
    over   = s + arrival - s_wp            # 末点 arrival = _GOAL_STOP_M(0.03)
    s_reached = over >= 0
    dist   = wp.distance_to(pose)
    记录 _min_dist[i] = min(dist)
    near   = (lat <= 0.30) or dist is None or dist <= 2.0 * arrival
    if (s_reached and near) or dist <= arrival:      -> 到达（"s+近路径" / "距格心"）
    elif _pivot_notice_accepts(...):                  -> 到达（"PIVOT 通知"），_passed_by_pivot.add(i)
    elif s_reached and _min_dist[i] <= _close_m:      -> 到达（"最近接近 X m"）
    elif s_reached and over >= _skip_over_m:          -> 显式记账「越窗待定」（不置位）

_close_m = max(arrival_m, 0.5×BLOCK_SIZE) = 0.175（:1735）；_skip_over_m = max(4×arrival_m, BLOCK_SIZE) ≈ 0.48（:1739）。

#### 2.4.10 停滞兜底与静止闸（mission.py:3163-3334）

_detect_stall_and_skip：任一成立即跳点——① stall_timeout_s(12 s) 内前进不足 arrival_m；② 累计转满 stall_yaw_deg(360°)。例外：Tracker 正在执行 PIVOT 时整体不计停滞（_pivot_in_progress，:3250-3266，读 P_TELEMETRY[tracker_cmd].pivot_phase）。

静止闸（:3224-3236）：_still_n >= _STILL_TICKS(10) ⇒ 禁止跳点，stall_skip_blocked++，如实上报 stall（H8 卡死修复：s 冻死时每 12 s 跳一个点，车一步没动却报 FINISH = 假完赛）。

_update_still_window（:3299-3327）：位移 ≤ 0.01 m/拍 且 v ≤ 0.03 m/s ⇒ +1，否则清零。

#### 2.4.11 PIVOT 完成通知（mission.py:3268-3297）

_pivot_notice 读 PIVOT_DONE_KEY = (顶点索引, 顶点弧长, 结局)；_pivot_notice_accepts 三道护栏：索引逐位对上、弧长逐位对上、车仍在顶点附近（s_wp - s <= _pivot_accept_m 且 dist <= _pivot_accept_m，_pivot_accept_m = max(3×pivot_dist_eff, 6×arrival_m)）。

#### 2.4.12 停站三段式（mission.py:3418-3653）

_enter_stop_point（:3418-3443）→ _run_stop_point（:3445-3494）：

    ① 到站级硬死线（先于一切）：now - started_at >= _stop_deadline_s -> _abandon_stop
    ② 红面早退（仅 ACQUIRE/FIRE 段）：P_AIM_RED.confirmed 且 dwell >= red_early_leave_s -> _abandon_stop
    ③ SCAN 停站 -> _run_scan_stop（SETTLE + FIRE 两段，超时即完成）
    ④ SHOOT：SETTLE -> ACQUIRE -> FIRE

- _phase_settle（:3497-3523）：位置连续不变 或 速度近零 ⇒ 武装；超 settle_time_s 或 settle_grace_s = max(0.4×shot_timeout_s, 1.0) ⇒ 降级放行。
- _phase_acquire（:3526-3539）：aim_status()==TRACK and vision().has_target ⇒ FIRE；否则 lock_timeout_s(18 s) 超时放弃。
- _phase_fire（:3542-3559）：_set_fire_gate(True)；发数增量 ≥ shots_needed ⇒ 完成；shot_timeout_s(30 s) 超时 ⇒ 关闸放弃。
- _goto_phase / _complete_stop / _abandon_stop / _close_stop_point（:3576-3629）：每段结束都 _set_fire_gate(False)（旧版纪律）。
- _set_fire_gate（:3658-3664）：锁存式，只在变化时发 publish_fire_gate。
- _skip_following_shoot_stop（:3595-3614）：本停站已打出去 ⇒ 跳过下一个射击停站。

#### 2.4.13 红面早退与弹药语义

P_AIM_RED 契约见 core/blackboard.py:32-47：AimTask 写、MissionTask 读；只在 ACQUIRE/FIRE 段生效；离开停站立即归零（mission.py:3465-3482）。

#### 2.4.14 终点与收尾清账（mission.py:3721-3881）

_check_finish（:3721-3823）：

- 末点带 * 时先跑停站三段式再判完赛（:3727-3731）。
- 末点判据收紧到 _GOAL_STOP_M(0.03)：_passed[last] or s + 0.03 >= s_at_waypoint(last)（:3737）。
- 置 _finish_latched、_auto_start_consumed=True、_close_ledger()；置 FINISH、刹车；lock 模式下不发 scan_request(0)（单写者铁律，:3780-3786）。
- 发 EV_MISSION_FINISHED + 结束横幅（用时/发数/命中/漂移/停站/到点率/显式跳过）。
- 若不变量被破坏（unexplained 非空）打 ❌ 横幅（:3805-3811）。

_close_ledger（:3842-3876）：收尾时把仍悬着的航点显式记账，保证 accounting()['unexplained'] 恒为空。

#### 2.4.15 外部命令与复位（mission.py:3886-4007）

_handle_command 支持 reset:BLOCK,HEAD / load:文件 / 单点/single/goto / 多点/multi/route / 裸多航点；未知命令只告警不崩（:3901-3952）。_normalize_command_tokens（:3955-3978）把误写在中间的裸 x/y 挪到最后。

_reset_for_next_run（:3825-3840）：只回阶段 + 重开终点锁存，不清路线数据（保留成绩遥测）。

#### 2.4.16 潜在问题（本次审计发现）

1. docstring 与 SCHEMA 已不一致：mission.py:46-49 说 planner.* 未登记进 SCHEMA，:141-146 说 mission.shot_timeout_s 仍缺；但本次审计实测 core/config.py:234-253 已登记 planner.*，:336 已登记 mission.shoot_timeout_s。属文档过期，非代码缺陷。
2. GBK 炸弹：mission.py 有 20+ 处 self._log(...)/log(...) 字面量含 ⚠️/⛔/✅/▶（:433, 460, 612, 629, 1019, 1047, 1881, 1991, 2146, 2151, 2391-2396, 2402, 2457, 2463, 2706, 3161, 3233, 3243, 3813, 3874, 3899），另 :679 footprint summary 的 ✅。实测 ⚠️/✅/⛔/▶(U+25BA) 均不可 GBK 编码。
3. _pivot_in_progress 读的是 Tracker 的遥测快照（:3260-3266）：若 Tracker 未运行或遥测未更新，Mission 会按停滞兜底误跳点（依赖 Tracker 50 Hz 写遥测）。属耦合，未核实失败模式。
4. _check_finish 里 _close_ledger() 在 _finish_latched 置位之后调用（:3740-3742），若 _close_ledger 抛异常会中断 FINISH 结算；代码未见 try 包裹，未核实。

---

### §2.5 tasks/aim.py —— AimTask（50 Hz，T5）★ 云台瞄准与开火门

#### 2.5.1 两把钥匙与单写者（aim.py:6-36）

| 话题 | 唯一写者 | 语义 |
|---|---|---|
| /aim/fire_gate | MissionTask | 任务级：到站停稳进入 FIRE |
| /aim/fire_ready | AimTask | 瞄准级：目标确认、落点在框内、冷却、弹药 |
| /nav/scan_request | AimTask（默认） | 锁止 0 / 起转 2（aim.gimbal_lock_enabled=True 时 Mission 一个值都不发） |
| /aim/lead_x | AimTask | 提前量（px） |

模块内出现 publish_fire_gate 即 bug（:13-14）；测试第 9 组断言全程 bridge.fire_gate 为空。

#### 2.5.2 FirePolicy 与开火框（aim.py:152-190）

FirePolicy ∈ {moving_lead, sweep, turnaround, static}（:152-161）。

fire_box_px(box_w, *, near_px, far_px, box_w_far, box_w_near)（:163-189）——近大远小线性插值：

    t    = clamp((box_w - box_w_far) / (box_w_near - box_w_far), 0, 1)
    box  = far_px + t × (near_px - far_px)

退化输入：box_w ≤ 0 或两端点相等 ⇒ 取 far_px（保守）。默认 far_px=14、near_px=26、w_far=30、w_near=120（SCHEMA config.py:379-380, 388-389）。

#### 2.5.3 RailEstimator（aim.py:192-429）★ 核心

位置走 α 滤波，速度走长基线端点差分：

    v = (z_new - z_old) / (t_new - t_old)     # 基线窗口 vel_window_s = 0.40 s
    α = 2(2ξ−1) / (ξ(ξ+1))    β = 6 / (ξ(ξ+1))    # ξ=3.5 ⇒ α=0.7619, β=0.3810

为什么不用 α-β 的 v += β·res/dt（docstring :200-213）：本靶 4 cm/s ÷ 30 fps ≈ 0.19 px/帧，单帧位移小于 1 px 量化步长，β/dt ≈ 11.4/s 把噪声放大成 ±5 px/s，实测 20 fps 把 30 px/s 估成 25.31 px/s（−15.6%）。端点差分对匀速零稳态误差、对量化噪声按 1/W 抑制。

四道防线：
1. dt 来自观测时间戳（OffsetObservation.ts）；dt <= dt_min_s(0.002) 视为同帧重读丢弃，dt > dt_max_s(0.50) 视为断流重建（:352-360）。
2. 跳变门：gate = max(gate_k×gate_base_px + gate_vel_scale×|v|×dt, jump_floor_px)；|res| > gate ⇒ 重建 + v 清零（:382-390）。
3. 方向确认 warmup：重建后 warmup = reset_confirm_frames(5)，每帧减一，期间不允许开火（:294, 374-377）。
4. 零位移帧（Δz=0）单独支路：不动 v、不动基线，只推进方向确认（:371-379）。

折返处置 _block_warmup（:305-324）：|v| >= flip_min_speed_px_s(2.0) 且 v × dz < 0 ⇒ 重建基线（x=z）+ v 清零 + resets/flips 各 +1。只清 v 不清 x 会让速度冲到 −61 px/s（实测），重建后是 −30.1 px/s（docstring :311-320）。

荒谬速度兜底：|v_new| > max_speed_px_s ⇒ 重建（:406-409）。

#### 2.5.4 许可账本 _Permit（aim.py:434-485）

- window：本靶射击窗口内许可数（重新武装清零）→ 判 max_shots_per_target；
- total：生命周期许可总数（只增）→ P_AIM.shots_fired；
- actual：用 shoot_count 增量反推的实际发数。
- 不变量 window ≤ total。

#### 2.5.5 tick 主流程（aim.py:715-764）

1. dt 计算（时钟跳变退回 period）。
2. obs = bridge.vision(VISION_MAX_AGE_S=0.5)；has_target = cls ∈ {BLUE, RED}。
3. _account(now)（命中/弹药/发数）。
4. _ingest(obs, has_target)（新帧才喂估计器；同帧重读不重复，:769-782）。
5. _update_class（连续 N 帧同类才改锁）。
6. _update_red_confirmed(dt)。
7. _track_visibility（丢靶立刻闭锁 + 发 EV_TARGET_LOST）。
8. 安全门 + bridge.aim_status()。
9. _update_gimbal_gate（云台锁止）。
10. ready, reason = _evaluate(...)。
11. lead = _lead_px(...)、_publish_lead、_set_fire_ready。
12. 写 P_TARGET / P_AIM / P_TELEMETRY[aim_cmd]。

#### 2.5.6 开火硬门顺序（aim.py:894-984）

安全 → 遥控 → 无目标 → 类别 → 命中锁存 → 弹药/预算 → 几何(策略/框) → 估计器(stale/warmup) → 冷却。
几何排在估计器之前的理由：dx=400 跳变时估计器会重置，若先判估计器会把「靶跑出开火框」这个几何事实掩盖成 estimator_warmup（docstring :900-906）。

策略判据：
- moving_lead：|pred| <= box（pred = dx + lead）。
- sweep：|pred| <= box 且 |pred| < |dx|（或 |v| <= lead_eps）。
- turnaround / static：|v| <= max(1.0, v_expected×0.25) 且 |pred| <= box。
- 垂直方向用同一框（:973-975）。

#### 2.5.7 许可迟滞与计数时机（aim.py:1000-1057）★

_set_fire_ready 的三个关键语义：

1. 许可计数发生在「冷却到点那一拍」，不是 False→True 跳变那一拍（:1005-1013）——否则单靶只能打 1 发（fire_ready 一直是 1，window 永远停在 1）。
2. 许可迟滞 hold_min_s(0.30 s)：纯粹的「冷却未到」不立刻撤许可，再保持 hold_min_s（云台 100 Hz 可能漏采只亮一拍的许可）；其余门必须当拍闭锁（:1015-1026）。
3. 不变量：一拍最多记一次许可；许可保持 + 冷却 ≥ shot_cooldown_ms。

#### 2.5.8 云台巡迹锁止（aim.py:1059-1125）★

判据只看两个已存在事实：P_STOP_POINT.active and kind == SHOOT ⇒ 起转搜索；其余 ⇒ 锁止。
下发策略：只在取值变化时发（_gimbal_cmd 去重）；但 TRACK→锁止 的边沿必须再强制发一次 0（lock_edge = _gimbal_was_seeking and not seeking，:1107-1111），否则 C++ 侧（电平保持语义 + 每帧 IDLE 归位）会停在最后一次目标方向。

#### 2.5.9 类别锁定与红面早退（aim.py:1252-1315）

- _update_class（:1252-1277）：单帧抖动不改 _locked_cls，但 _cls_run 也不增长（该帧不允许开火）；连续 cls_switch_confirm_frames(5) 帧同类才改锁；不可见不清除锁。
- _update_red_confirmed（:1282-1315）：只在射击停站 ACQUIRE/FIRE 段累计；离开停站或类别不再是红 ⇒ 累计清零、标志撤下；锁定红 ≥ red_early_leave_s(3.0 s) ⇒ 写 P_AIM_RED{confirmed:True}。

#### 2.5.10 丢靶回扫（aim.py:1231-1247）

默认 aim.rescan_on_lost=False（关）。开着且已到最佳射击点（_gimbal_seek）才重发 scan_request=2（持续），绝不发 1（会把持续扫描降级成单趟）。

#### 2.5.11 构造即发布安全初值（aim.py:655-669）

_publish_initial 在构造函数里写 P_TARGET=TargetState() 与 P_AIM=AimState(fire_gate=False, ...)，避免别的任务在第一拍前读 P_AIM 崩溃。

#### 2.5.12 潜在问题（本次审计发现）

1. SCHEMA 缺键自检机制：_OPTIONAL_KEYS（:497-509）登记了 23 个 aim.* 键；_opt（:697-706）只在「既不在 robot.yaml 也不在 SCHEMA」时登记 _used_defaults；setup() 打印缺失清单（:1330-1340）。但本审计未逐键核对这 23 个键是否都已进 core/config.py SCHEMA——本次只抽查到 aim.est_* / aim.px_per_m_rail 等已登记（config.py:370-427），逐键对账未做，标「未核实」。
2. 许可迟滞与冷却的语义耦合（:1015-1018）：hold_min_s 只对「冷却」这一条迟滞，代码靠 reason == REASON_COOLDOWN 判断（:1024）；若其他原因恰好也等于 REASON_COOLDOWN 会误迟滞，属边界，未核实。
3. _write_aim 里 P_AIM.fire_gate = bool(self._fire_ready)（:1168）：AimState.fire_gate 语义在本任务里是「本任务已许可」，与话题 /aim/fire_gate 的 Mission 语义不同名同义，容易误读。
4. GBK：aim.py 本文件的 self._log 已把 ⚠️ 全部改成 [注意]（:1335-1340），原因码不含非 GBK 字符；已无 GBK 炸弹。属唯一干净的任务文件（对照 §2.1/§2.2/§2.3/§2.4）。


### §2.6 tasks/fix.py —— FixTask（10 Hz，T6）★ 误差修正估计侧

#### 2.6.1 分工（fix.py:1-60）

- LocalizationTask = 执行侧：吸附把 P_POSE 锚回格心，把偏差写进 P_FIX（FixState）。
- FixTask = 估计侧：只读事件与 P_POSE/P_ROUTE_S，把估计量写进 P_FIX_EST（同名类型 FixState）。
- 本任务绝不改位姿：不写 P_POSE / P_POSE_RAW / P_FIX（:10-11）。

scale 语义（:13-25）：scale = 信念位移 / 路线几何弧长。1.0 = 无系统误差；1.18 = 里程多报 18%；
修正系数是 1/scale。

估计方法三板斧（:27-37）：每完成一段比较 Σ|Δpose| 与 ΣΔs → ratio；三重剔除（异常漂移、
比值越界、比值突跳）；保守更新（单段步长 ≤ scale_step_max、总量夹在 1±scale_limit、
样本数 < scale_min_samples 不出估计值）。

#### 2.6.2 配置（fix.py:99-117）

| 键 | 默认 | 位置 |
|---|---|---|
| fix.scale_est_enabled | bool | :100 |
| fix.scale_step_max | (SCHEMA) | :101 |
| fix.scale_limit | (SCHEMA) | :102 |
| pursuit.drift_warn_m | 0.30 | :103 |
| fix.drift_cull_m | = drift_warn_m | :104 |
| fix.seg_min_m | 0.05 | :105 |
| fix.scale_min_samples | 3 | :106 |
| fix.ratio_bounds | (0.75, 1.35) | :107-109 |
| fix.ratio_jump_max | 0.15 | :110 |
| fix.ratio_jump_min_prior | 3 | :111 |
| fix.scale_window | 5 | :112 |
| fix.ds_max_ratio | 2.0 | :114 |
| fix.ds_floor_m | 0.05 | :115 |
| fix.ds_reject_max_run | 3 | :116 |

#### 2.6.3 tick 与事件（fix.py:217-278）

tick（:217-229）：_drain_events → _accumulate → _publish(scale)。

_on_anchored（:242-281）：段闭合；ratio = _segment_ratio()；四路剔除：
seg_too_short / drift_outlier / ratio_out_of_range / ratio_jump；否则 _accept。
漂移告警走 _warn 限流并带 by="fix_est"（:271-278）。段边界后 _rebaseline()（:281）。

#### 2.6.4 位移累计与 ds 守卫（fix.py:317-412）★

_accumulate（:317-347）：pose 为 None 返回；首拍 _rebaseline；否则 step = hypot(Δx,Δy)，
step > _TELEPORT_M(0.20) ⇒ 本段作废重起（位姿被外力挪过）；否则 _belief_m += step、
_settle_ds(s, step)。

_ds_limit（:349-366）：max(位姿位移 × ds_max_ratio, ds_floor_m)。理由：P_ROUTE_S 有三个写入者
（Localization/Tracker/Mission），冻结/换手会产生物理不可能的 ds，旧写法「只挑正数」把来回跳
全记成前进，段比值被推成 0.12~0.23（真值≈1）。

_settle_ds（:368-412）：ds <= 0 ⇒ 基线跟上、位移作废；ds <= limit ⇒ 计入 _geo_m；
否则 ds_rejected++、基线不动（下一拍补账）、连续 ds_reject_max_run 拍不可信 ⇒ 重新基线。

#### 2.6.5 保守更新（fix.py:283-312, 429-442）

_accept（:283-302）：segments_accepted++、窗口裁剪；est_enabled=False 只统计不改；
样本 < min_samples 不动；step = clamp(ratio - scale, ±step_max)；scale = _clamp(scale + step)；
scale_raw = ratio。

_reject（:304-312）：segments_rejected++、reject_reasons 计数、限流日志、发 EV_DRIFT_WARNING
(by="fix_est_reject")。

_segment_ratio / _window_mean / _clamp（:429-442）。

#### 2.6.6 潜在问题

1. GBK：fix.py:277 ⚠️、:308 ⚠️、:405 ⚠️ 在 self._log 字面量中；实测 ⚠️ 不可 GBK 编码。
2. _TELEPORT_M=0.20 是吸附半径上限（0.08）的 2.5 倍（docstring :77-78），但若 SCHEMA 的
   fix.anchor_max_offset_m 被调大到 > 0.20，吸附会被判成「突跳」整段作废，未核实是否互斥。
3. est_enabled=False 时仍 _publish(1.0, ...)（:157），保证 P_FIX_EST 恒存在（遥测依赖）。

---

### §2.7 tasks/drone.py —— DroneTask（5 Hz，T7）★ 无人机链路与布局真源

#### 2.7.1 职责（drone.py:1-87）

收 heartbeat / bridge_passable / target_zone / field_verify / field_layout；发 car_state / ack。
无人机是硬输入：只有它能从空中看到桥顶标识与全场布置。**不做路线决策**（选格是 Mission 的事）。

- 失效兜底：心跳丢失 > heartbeat_timeout_s(3 s) ⇒ online=False + 一次告警；情报过期 > info_stale_s(30 s)
  ⇒ bridge_of() 返回 None。任务不阻塞、不改变默认分支/默认地图（:38-42）。
- drone.enabled=False ⇒ 整任务空转：不读 bridge、不发 bridge、不写 P_DRONE（:42）。

#### 2.7.2 覆盖语义与核对消费方（drone.py:18-36, 297-519）

- CoverageLedger（:314-402）：按 (scan_id, slice.id) 幂等去重；只有 complete=True 的片才把 cells 记入
  cells_done；比例分母 = MAP_COLS×MAP_ROWS = 154；_done() 判定某次扫描全部片到齐且 complete。
- FieldVerifyConsumer（:405-519）：订阅 EV_FIELD_VERIFY；tag 分类 target/hazard/other；
  summary() 给 verdict/severity(risk/warn/info)/hold/alert；policy ∈ off/record/alert/hold
  （默认 record = 只记录不阻断，:305-311）。
- P_FIELD_VERIFY / P_DRONE_COVERAGE 由 DroneTask 统一写（:956-962, 942-954）。

⚠️ 两键未登记 SCHEMA（docstring :35-36, 83-84）：drone.field_verify_policy、drone.coverage_enabled。

#### 2.7.3 本地地图与布局（drone.py:73-81, 1152-1183, 842-889）

本地地图来源优先级：构造参数 local_map → cfg.get("drone.map_path") → 仓库默认 config/map.yaml
（上溯 4 层，:120-133）。都没有 ⇒ map_loaded=False，核对降级为「不核对」。

field_layout（:842-889）：normalize_field_layout（:181-294）纯内存校验；
FIELD_LAYOUT_VERSION = 2（:145）；version 单调（旧版拒收 layout_stale；同版异内容拒收）；
v1 无 known 键 = 整图已知（归一化后**不补键**，车端据此区分 v1/v2，:217-229）；
特征必须 ⊆ known（v2，:231-239）；bridges 至少 1 个；bad 报文整份拒收、保留上一份好布局（:844-850）。

#### 2.7.4 tick 主流程（drone.py:714-722）

_drain_inbox → _drain_field_verify → _check_heartbeat → _maybe_send_car_state → _publish。
enabled=False 时直接 return（一个 bridge 调用都不发，:716-717）。

#### 2.7.5 桥洞情报（drone.py:808-824）

_on_bridge_passable：half/block 校验；**刻意不校验格名是否在地图内**（穿洞格每局随机，无人机才是权威，
docstring :815-816）；写槽 + 发 EV_BRIDGE_INFO（ts=now，与 P_DRONE.bridge_ts 同源）。

#### 2.7.6 心跳（drone.py:779-806, 1061-1076）

_on_heartbeat：先算 silent 再刷新 last_heartbeat（顺序要紧，否则恢复事件的 silent_s 恒为 0）；
上线发 EV_DRONE_ONLINE。_check_heartbeat 只在状态变化时发一次 EV_DRONE_OFFLINE。

#### 2.7.7 车状态上报（drone.py:1085-1144）★

_car_state_fields（:1085-1105）：★D1 段级真相在 P_CAR_STATE（"car_state"），StepLoop 在场时以它为准；
键名写字面量不 import main（main → tasks 单向依赖，反向会成环）。
_maybe_send_car_state（:1107-1119）：签名 (phase, segment_id, target_cell, stop_kind) 变化即发；
_send_car_state（:1121-1144）带 seq 递增。

#### 2.7.8 潜在问题

1. GBK：drone.py 有 20+ 处 self._log 字面量含 ⚠️（:479, 621, 623, 742, 749, 755, 774, 848, 858, 868,
   876, 940, 953, 984, 1011, 1026, 1058, 1071, 1182）；实测 ⚠️ 不可 GBK 编码。
2. **字段_layout 与 car_state 是两条独立链路**：field_layout 走 drone_in 话题解析；car_state 走
   drone_out。文档未说明机端如何把车回报（car_state.block/stop_kind）喂回 LayoutStream.note_segment_done
   （scout/layout.py:643-656），装配侧接线**未核实**。
3. _take_coverage 在 shape 非法时返回 (None, False)，其余字段照旧处理（:937-941）；
   contract.decode_coverage 的具体校验口径在 io/contract.py，本审计未逐行读（§7）。

---

### §2.8 tasks/telemetry.py —— TelemetryTask（2 Hz，T8）

#### 2.8.1 职责（telemetry.py:1-39）

只做两件事：把 Blackboard 上的一页纸状态汇总成 Telemetry 并落 P_TELEMETRY；按 telemetry.period_s
（默认 0.5 s）打印一行紧凑状态。不做任何决策。

★ 不能整键覆盖 P_TELEMETRY：它是约定共享 dict（含 tracker_cmd / mission_state），
本任务只 read-modify-write 自己的两个子键（telemetry / telemetry_line，:11-18, 62-64, 289-295）。

#### 2.8.2 数据来源（telemetry.py:20-30, 96-163）

build_telemetry(bb, now, drone_stale_s=30.0) 纯读取：phase/block/x/y/yaw_deg ← P_MISSION_PHASE/P_POSE；
seg_index/s/speed/steer_deg ← P_ROUTE_INDEX/P_ROUTE_S/P_TELEMETRY[tracker_cmd]；
aim/target/shots/hits/ammo ← P_AIM/P_TARGET；safety_ok ← P_SAFETY；drone ← P_DRONE；
route_len/extra.stop_point/extra.fix_est ← P_ROUTE/P_STOP_POINT/P_FIX_EST。缺键显示为未知，不崩。

#### 2.8.3 渲染（telemetry.py:73-93）

render_line 是纯函数（同一 Telemetry 永远同一行文本），字段顺序固定「位置 → 阶段 → 控制 → 进度 →
打击 → 安全 → 链路」。

#### 2.8.4 tick（telemetry.py:246-263）

节流（now < _next_at 返回）；build_telemetry 失败限流跳过不杀线程；写 P_TELEMETRY 子键并打印。

#### 2.8.5 潜在问题

1. GBK：telemetry.py:256 self._log 含 ⚠️；实测不可 GBK 编码。
2. 打印用 self._log（默认 print），与 §2.1-§2.4 同类 GBK 风险。
3. drone_stale_s 默认 30.0（:223），与 drone.info_stale_s 默认一致但**键名不同**（drone.info_stale_s），
   两处默认值可能漂移（SCHEMA 有 drone.info_stale_s，telemetry.period_s 已登记 config.py:561）。


### §2.9 scout/* —— 机侧侦察仿真模型（T28/T29，纯函数、无 ROS/DDS）

> 重要前提：scout/ 是机侧（无人机）侦察仿真模型，不是车端任务线程。它不 import rclpy，
> 不读车端实时状态；simulate() 读 config/map.yaml 真值图，生成帧位时间线 + 布局 v2 序列 +
> 心跳计划 + 对照指标（scout/__init__.py:1-19, 85-110）。车端只消费它产出的 field_layout 报文
> （经 drone.py 校验后写 P_FIELD_LAYOUT）。

#### 2.9.1 scout/fov.py —— 视场模型（225 行）

物理假设（scout/fov.py:3-20，仿真假设、非现场标定值）：四旋翼 1.5 m 高度俯视，
地面足迹 = 4 格 × 2 格 = 1.40 m × 0.70 m；航向 0°（+x）时足迹为 2 列 × 4 行。

- FOV_LENGTH_CELLS=4 / FOV_WIDTH_CELLS=2（:32-34）；FOV_LENGTH_M=1.40 / FOV_WIDTH_M=0.70（:35-36）；
  SCOUT_ALTITUDE_M=1.5（:40）。
- Pose(x, y, yaw)（:45-66）；cell_centers() 全部 154 格（:69-78）；block_key 行优先排序键（:87-90）；
  sorted_blocks 去重+行优先（:93-96）。
- cells_in_view(pose)（:103-118）：格心局部坐标 (u,v)，|u| < 0.7 且 |v| < 0.35（半长 0.70/半宽 0.35）。
  「格心严格在内」而非「矩形与格有交」（:11-16，理由：与 route/planner.py 格心判据同源、
  只有 pose 落在格点时 0°/90° 恰好 8 格）。
- footprint_corners（:125-135）；polygon_area 鞋带公式（:138-147）；
  clip_convex Sutherland-Hodgman（:167-188）；overlap_fraction 物理面积重叠率（:191-197）；
  cell_overlap_fraction Jaccard（:200-206）；is_node 是否落在格点（:209-215）。

#### 2.9.2 scout/frames.py —— 帧位生成（250 行）

- DEFAULT_FRAME_HZ = 2.0（协议 §4.1 上限，:29）。
- SCAN_LINE_YS = 0.35, 1.05, ..., 4.55（7 条纵向条带，:33-34）。
- LOWER_SCAN_XS = (1.05, 0.35)（:37）；UPPER_SCAN_XS = (1.40, 2.10, 2.80, 3.50)（:39）。
- ScoutFrame(index, t, pose, cells)（:42-60）。
- scan_poses()（:74-117）：下半场 7 线（先 N/M 侧再向西）→ 沿 y=4.55 横移到上半场 → 上半场 7 线回扫；
  转角处补「同位置换航向」帧保证相邻帧重叠 ≥ 50%（协议 ≥ 30%）。
- frames_from_poses / generate_snake_timeline / generate_timeline（:120-140）。
- frames_from_cell_route（:148-187）：格心 → 该格右下角格点（+0.175 m），每 stride 格取一帧，
  方向变化处补进/出双帧。
- coverage / adjacent_overlaps / cell_overlaps / timeline_duration_s / timeline_stats（:190-242）。

#### 2.9.3 scout/heartbeat.py —— 布局下发节奏（114 行）

协议两条硬语义（:7-10）：布局有变化就发；即使没变化也每秒重发最新版。

- heartbeat_schedule(change_times, t_end, interval_s=1.0)（:26-54）：纯函数；每个变化时刻立即发一次，
  之间每 interval_s 重发最新序号；同一时刻只保留一条；只输出 t <= t_end。
- heartbeat_messages(layouts, t_end, interval_s)（:57-69）：t_end 缺省 = 最后一份 ts。
- to_json（:72-74）；LayoutFeed（:77-110）：push 只接受更新版本；poll(now) 返回要发的 dict 或 None。

#### 2.9.4 scout/layout.py —— 布局 v2 与目标格决策（704 行）★

- TruthMap（:70-137）：真值图（机侧仿真口径）；from_map / load；all_cells / bridge_rows /
  blocked_for_driving。
- step_ok（:153-178）四类硬约束（与 route/planner.Grid.move_ok 对齐）：
  ① 目标格已知且可站；② 隔板只有列 < barrier_col_from 能穿；③ K 列内不得上下挪格；
  ④ 只能在桥洞行进出 K 列。
- reachable_cells（:181-201）BFS；frontier_cells（:204-213）。
- MissionTarget（:219-230）：block + kind(mine/stop/end/frontier) + engine + route。
- known_map_cfg（:233-253）：未知格塞进 obstacles（不可通行，不是物理水桶）；from_block 强制算已知。
- plan_mission_route（:268-326）：只有剩余雷+终点都已在 known 才尝试 planner；依次 tracker-safe(from_cell)
  → tracker-safe → tour。
- mission_checkpoints（:329-356）：未压雷 / 未做射击停站 / 终点。
- plan_mission_target（:359-392）：任务目标优先。
- frontier_target（:401-429）：回退——朝任务目标方向的最远可达前沿格。
- choose_mission_target / choose_target_cell（:432-461）。
- build_layout（:467-503）：输出可直接下发的 field_layout v2 dict。
- LayoutStream（:506-695）：feed / retarget / note_segment_done / observe_car_block / mark_covered /
  layouts_for / layouts_for_timeline；known 单调增长，变化才产生新 version。

#### 2.9.5 scout/reference.py —— 用户手画路线对照（90 行）

USER_ROUTE_BLOCKS（:31-39）：从 _tmp_session/step_loop/scout_route_user.png 还原的 66 格路线；
起点 M8、终点 M2；只扫 6 条纵向条带+M 列，N 列有缺口。
user_route_timeline / compare_routes（:51-86）：生成 vs 手绘（stride=1 与 stride=2 两组），
输出覆盖率/帧数/时长 delta。

#### 2.9.6 scout/__init__.py —— 一站式入口（130 行）

ScoutResult（:45-82）；simulate(map_path, start_t=-30.0, frame_hz=2.0, car_block, heartbeat_interval_s=1.0,
t_end, cfg)（:85-110）：TruthMap → generate_snake_timeline → LayoutStream.layouts_for →
heartbeat_messages → compare_routes。

#### 2.9.7 潜在问题

1. scout 的「T30 任务目标优先」与车端 Mission 的停止策略是两套独立实现：scout 用
   route/planner 规划检查点，车端 Mission 用 route.tokens 的 * 标记。两者对「射击停站」的判据
   必须一致（都指向 planner.shoot_distance_window），但一致性未核实（§7）。
2. LayoutStream.retarget / observe_car_block 需要车端回报驱动，车端 DroneTask 只发 car_state，
   装配侧是否把 car_state 喂进 LayoutStream 未核实。
3. USER_ROUTE_BLOCKS 的图源路径 _tmp_session/step_loop/scout_route_user.png 是否存在未核实
   （该目录可能已被清理）。

---

## §3 函数级索引大表（尽量穷尽）

> 本表按模块列出公开/半公开函数/方法、签名要点、行号、一句话职责。§ 列即本文档小节。

### 3.1 tasks/localization.py（1444 行）

| 函数/方法 | 行 | 签名要点 | 职责 |
|---|---|---|---|
| _cfg_float | 121 | (cfg, key, fallback) -> float | 只有键缺失才用兜底（0 是合法值） |
| PoseSource (Protocol) | 145 | pose()/pose_age() | bridge 鸭子类型契约 |
| LocalizationTask.__init__ | 158 | (bridge, bb, bus, cfg, *, log, group, clock) | 20 Hz；一次读齐全部配置 |
| setup | 353 | () | 打启动横幅 |
| tick | 359 | () | 10 步主流程 |
| teardown | 448 | () | 退出台账 |
| _float_or_inf | 466 | (value) -> float | inf/nan 归 inf |
| _project | 473 | (x, y, yaw=None) | 带 s_hint 窗口投影 + 方向一致性闸 |
| _clear_stuck | 548 | () | 复位卡死窗口 |
| _heal_stuck | 554 | (proj, x, y) | 三条判据的全路线重投影自愈 |
| _stuck_check | 644 | (proj, x, y) -> bool | 维护滑动窗口 |
| _update_yaw_guard | 664 | (pose, age, now) | 航向完整性守卫 |
| _yaw_limit_dps | 762 | (v) -> float | 差速包络 |
| _yaw_imu_mismatch | 769 | (rate) -> bool | IMU 交叉核对 |
| _trip_yaw_untrusted | 778 | (v, rate, limit, now, *, jump_deg, mismatch) | 判故障 + 发事件 |
| _clear_yaw_untrusted | 818 | (now) | 自动复位 |
| reset_yaw_guard | 831 | (now=None) | 人工复位 |
| _imu_yaw_rate_dps | 846 | () -> float或None | IMU gz 绝对值 |
| _manual_override | 867 | () -> bool | 遥控接管 |
| _update_travel_heading | 879 | (pose) | 实测位移方向裁决 |
| _opposes_travel | 922 | (proj) -> bool | 折返压线方向闸（150°） |
| _anchor | 942 | (x, y, yaw, proj, now) | 到位吸附三道闸 |
| _error_shrinks | 1030 | (target, proj) -> bool | 闸② |
| _along_ds | 1046 | (target, proj) -> float或None | 沿程修正量 |
| _scale_suspected | 1056 | (target, proj) -> tuple或None | 闸③ |
| _implied_scale | 1097 | (target, proj) -> float或None | ds / base |
| _refuse_anchor | 1119 | (block, index, drift, now, reason) | 显式记账 |
| guard_state | 1134 | () -> dict | 只读护栏快照 |
| _reanchor | 1188 | (proj, yaw, gate, now) | 顶点窗口横向重锚 |
| _distance_to_boundary | 1230 | (pose) -> float或None | 到安全边界距离 |
| _update_drift_guard | 1242 | (pose, *, unhealthy, now) | 写 P_MOTION_LIMIT |
| _log_throttled | 1330 | (key, now, text) | 限流日志 |
| _anchor_candidates | 1334 | (proj, x, y) | 吸附候选 |
| _warning_gate | 1366 | (proj) -> tuple或None | 顶点窗口 |
| _reset_anchor_latch | 1390 | (x, y, proj, now) | 解锁锚定闩锁 |
| _report_drift | 1410 | (block, index, drift, now, *, anchored) | 每航点至多一条告警 |
| _store_drift | 1431 | (target, drift, now) | 写 P_FIX |

### 3.2 tasks/safety.py（529 行）

| 函数/方法 | 行 | 签名要点 | 职责 |
|---|---|---|---|
| SafetyTask.__init__ | 52 | (bridge, bb, bus, cfg, *, log, group, clock) | 20 Hz；五道检查配置 |
| reasons / degraded (property) | 120-127 | () | 只读 |
| setup | 132 | () | 订阅 EV_TASK_FAULT + 横幅 |
| tick | 143 | () | 评估→迁移→健康写 OK |
| teardown | 179 | () | 退出台账 |
| clear_safety | 190 | (operator, source) -> SafetyState | 唯一恢复入口 |
| _evaluate | 221 | (pose, now) -> list[str] | 五道检查 |
| _check_group | 330 | (now) -> list[str] | 检查③（跳过自己） |
| _task_age | 384 | (h, now) -> float或None | 心跳年龄 |
| _heartbeat_age | 391 | (name, now) -> float或None | 指定任务心跳 |
| _update | 403 | (reasons, now) | 状态迁移 + 逐拍刹车 |
| _publish_brake | 461 | () -> bool | 如实返回成败 |
| _state | 477 | (*, ok, now) -> SafetyState | 组装 P_SAFETY |
| _pump_events | 495 | () | drain EV_TASK_FAULT |
| on_task_fault | 502 | (ev) | 记 task_fault_event |
| _finite / _safe_call | 514-529 | static | 工具 |

### 3.3 tasks/tracker.py（892 行）

| 函数/方法 | 行 | 签名要点 | 职责 |
|---|---|---|---|
| PivotPhase | 55 | IDLE/SETTLE/TURN | PIVOT 相位 |
| TrackerTask.__init__ | 123 | (bridge, bb, bus, cfg, *, log, group, clock) | 50 Hz |
| path / last_align_segments (property) | 241-249 | () | 只读 |
| telemetry | 251 | () -> dict | 最近一拍判定 |
| _ensure_align_segments | 258 | (route) | 桥洞航段集合 |
| _align_active | 290 | (seg_index) -> bool | 集合查询 |
| _resolve_path | 296 | (route) -> Path | 缓存 Path |
| _blocked_reason_now | 326 | (pose, safety, phase, route) -> str | 前置门 |
| _hold | 343 | (reason) | 刹车 + 记录 |
| tick | 359 | () | 主循环 |
| _corner_ahead | 710 | (path, proj) | 前方顶点信息 |
| _dist_to_next_cross | 724 | (pose, route, proj) | 到最近 K 列格距离 |
| _gear_cruise | 745 | (pose, route, proj, now) | 动态档 |
| _pivot_update | 771 | (pose, path, proj, now, dt) | PIVOT 状态机 |
| _reset_pivot | 872 | () | 复位 PIVOT |
| _write_telemetry | 876 | () | 写 P_TELEMETRY |
| teardown | 885 | () | 发零指令 |

### 3.4 tasks/mission.py（4007 行）

| 函数/方法 | 行 | 签名要点 | 职责 |
|---|---|---|---|
| BridgeIntel | 304 | frozen dataclass | 半场情报结论 |
| retag_block_token | 344 | (token, old, new) -> str | 词边界换格名 |
| _token_block | 354 | (token) -> str或None | token→格名 |
| rewrite_crossing_tokens | 363 | (tokens, half, old, new) -> list或None | 穿洞三连改写 |
| resolve_plan_engine | 421 | (cfg, log) -> str | 引擎别名容错 |
| plan_budget_seconds | 439 | (cfg, engine, log) -> float | 规划预算 |
| GateResult | 464 | dataclass | 双门禁结果 |
| route_check_script | 502 | (cfg) -> Path或None | 定位 route_check.py |
| load_route_check_module | 515 | (cfg) -> module或None | 加载门禁模块 |
| _run_route_check | 536 | (mod, tokens, map_cfg, cfg) | 本进程跑门禁 |
| check_planned_route | 570 | (tokens, map_cfg, cfg, log, mod) -> GateResult | 双门禁总入口 |
| FootprintReport | 643 | dataclass | 足迹诊断 |
| vehicle_geometry | 682 | (cfg) -> dict | 车体尺寸（键路径坑） |
| _map_geometry | 715 | (map_cfg) -> dict | 兼容两种地图写法 |
| _merged_target_rect | 721 | (geo_map) -> tuple或None | 跨格靶板合并矩形 |
| footprint_diagnostic | 737 | (tokens, map_cfg, cfg) -> FootprintReport | 折线口径足迹 |
| _state (nested) | 780 | (px, py, yaw, where) | 采样辅助 |
| AutoPlanBlocked | 835 | RuntimeError | 停在 IDLE |
| PlanOutcome | 839 | dataclass | 规划结果 |
| workspace_root | 887 | () -> Path | parents[4] |
| config_dir | 899 | (cfg) -> Path或None | 配置文件目录 |
| resolve_map_path | 907 | (cfg, spec) -> Path | 地图路径解析 |
| _read_map_yaml | 929 | (path) -> dict | 读地图 |
| plan_route_from_config | 941 | (cfg, *, map_path, log, cross_lower, cross_upper) -> PlanOutcome | 自动规划入口 |
| _stop_kind_with_spec | 1117 | (spec, cell, cfg) -> tuple | 停站分类内核 |
| stop_kind_for_cell | 1150 | (map_cfg, cell, cfg, *, end_cell) -> tuple | 段目标格分类 |
| SegmentPlan | 1172 | dataclass | 段规划结果 |
| heading_dir_from_yaw | 1209 | (yaw) -> tuple | yaw→栅格方向 |
| _seg_turn_deg / _seg_unit | 1218-1226 | static | 段规划几何 |
| plan_segment_route | 1229 | (map_cfg, cfg, *, from_cell, target_cell, heading_dir, engine, blocked_extra, source) -> SegmentPlan | 段规划 |
| _StopContext | 1454 | dataclass | 停站点可变上下文 |
| MissionTask.__init__ | 1479 | (bridge, bb, bus, cfg, *, log, group, clock) | 10 Hz |
| phase / route / stop_context / handled_stops (property) | 1744-1758 | () | 只读 |
| state | 1760 | () -> dict | 遥测上下文 |
| request_segment | 1789 | (tokens, *, segment_id, target_cell, known_count, source, stop_kind) -> dict | T27 段请求 |
| stop_segment | 1820 | (reason) -> dict | 停段 |
| segment_status | 1827 | () -> dict | 段状态 |
| reset_pose | 1840 | (spec) | 复位 belief |
| _process_segment_requests | 1844 | () | tick 开头处理段请求 |
| accounting | 1892 | () -> dict | 逐点到达台账 |
| closest_approaches | 1925 | () -> dict | 历史最近距离 |
| bridge_state | 1937 | () -> dict | 桥洞情报遥测 |
| _drone_state | 1962 | () -> Any | 读 P_DRONE |
| _intel_map_cfg | 1977 | () -> dict或None | 本地地图缓存 |
| _map_bridges | 1995 | () -> tuple | map.bridges K 列格 |
| _half_candidates | 2009 | (half) -> tuple | 候选收窄 |
| _bridge_pool | 2024 | (half) -> tuple | 合法穿洞格集合 |
| _intel_cell_ok | 2044 | (half, cell) -> tuple | 情报采纳门 |
| _bridge_subscription | 2078 | () -> Subscription | 惰性订阅 |
| _event_ts | 2093 | (data, now) -> float | 载荷 ts |
| _record_intel | 2109 | (half, block, ts) -> bool | 按半场独立记录 |
| _drain_bridge_events | 2125 | (now, max_items) -> int | 消费事件 |
| _intel_of | 2156 | (half) -> BridgeIntel | 单半场结论 |
| _refresh_intel | 2195 | (*, log) -> dict | 刷新两半场 |
| _intel_complete / _has_fresh_half | 2221-2232 | () -> bool | 等待判据 |
| _should_wait_for_intel | 2234 | (elapsed) -> bool | 起点等待窗口 |
| _bridge_wait_cap | 2267 | () -> float | 等待上限 |
| _decision_point | 2276 | () -> tuple或None | 决策点 |
| _link_online | 2300 | () -> bool | 链路在线 |
| _maybe_enter_bridge_hold | 2307 | () | 进 HOLD |
| _run_bridge_hold | 2340 | () | HOLD 循环 |
| _release_bridge_hold | 2357 | (reason) | 放行 |
| _reroute_at_decision | 2373 | (half, *, why) -> bool | 决策点改道 |
| _apply_intel_to_tokens | 2416 | (tokens, snap) -> tuple | 情报换洞 |
| _swapped_tokens_ok | 2474 | (tokens, half, old, new) -> tuple | 换洞校验 |
| _fmt_age | 2502 | (age) -> str | 格式化 |
| setup / teardown | 2509-2541 | () | 生命周期 |
| tick | 2546 | () | 主循环 |
| _maybe_auto_start | 2587 | () | 自动出发 |
| _publish_intel_note | 2633 | (*, waiting) | 遥测 note |
| _load_startup_route | 2640 | () | 路线来源分派 |
| _start_mission | 2722 | () | DRIVE + 扫描下发 |
| _engine_banner_text | 2737 | () -> str | 引擎横幅行 |
| _cross_source_line | 2745 | () -> str | 穿洞来源行 |
| _intel_summary_line | 2765 | () -> str | 情报摘要 |
| _banner_start | 2773 | () | 开始横幅 |
| _reset_belief | 2798 | (spec) | 复位 belief |
| _load_tokens | 2814 | (tokens, *, source) -> int | 载入多航点 |
| _load_single_point | 2827 | (spec) -> int | 载入单点 |
| _install_route | 2857 | (route, *, source, keep_progress) | 唯一路线真源 |
| _classify_stops | 2925 | (route) -> dict | 停站分类 |
| stop_kinds / stop_ledger | 2953-2976 | () | 台账 |
| _stop_ledger_line | 2978 | () -> str | 横幅行 |
| _shoot_planner | 3000 | (target) -> list[str] | SHOOT@ 候选链 |
| _current_s | 3031 | () -> float | 本拍弧长 |
| _advance_waypoints | 3062 | () | 航点推进 |
| _detect_stall_and_skip | 3163 | () | 停滞兜底 |
| _pivot_in_progress | 3250 | () -> bool | 读 Tracker 遥测 |
| _pivot_notice | 3271 | () -> tuple或None | 读 PIVOT_DONE_KEY |
| _pivot_notice_accepts | 3278 | (notice, idx, s_wp, s, dist) -> bool | PIVOT 判达护栏 |
| _update_still_window | 3299 | (pose) | 静止闸窗口 |
| _reset_stall | 3329 | () | 复位停滞 |
| _next_stop_candidate | 3336 | () -> int或None | 下一停站点 |
| _dist_to_stop | 3369 | (idx, pose) -> float或None | 到停站点距离 |
| _drive | 3378 | () | 驱动分支 |
| _should_trigger_stop | 3397 | (idx, s, dist) -> bool | 到点判据 |
| _enter_stop_point | 3418 | (idx, dist) | 进入停站 |
| _run_stop_point | 3445 | () | 停站主循环 |
| _phase_settle | 3497 | (stop, now) | 段1 |
| _phase_acquire | 3526 | (stop, now) | 段2 |
| _phase_fire | 3542 | (stop, now) | 段3 |
| _run_scan_stop | 3562 | (stop, now) | 扫描停站 |
| _goto_phase | 3576 | (stop, phase, now, detail) | 段间迁移 |
| _complete_stop | 3584 | (stop, detail) | 完成停站 |
| _skip_following_shoot_stop | 3595 | (done) | 跳过下一射击停站 |
| _abandon_stop | 3616 | (stop, detail) | 放弃停站 |
| _close_stop_point | 3622 | (stop) | 关闭停站点 |
| _publish_stop_state | 3631 | (now) | 写 P_STOP_POINT |
| _build_stop_state | 3636 | (stop) -> StopPointState | 组装 |
| _last_speed | 3644 | () -> float或None | 读 Tracker 速度 |
| _set_fire_gate | 3658 | (allow) | 放行门（锁存） |
| _request_scan | 3666 | (mode) | 扫描下发（单写者） |
| _write_phase | 3688 | (phase) | 写 P_MISSION_PHASE |
| _enter_safe_stop | 3696 | () | 安全接管 |
| _recover_from_safe_stop | 3703 | () | 安全恢复 |
| _publish_state | 3711 | (now) | 写 mission_state |
| _check_finish | 3721 | () | 终点结算 |
| _reset_for_next_run | 3825 | () | 复位回 IDLE |
| _close_ledger | 3842 | () | 收尾清账 |
| _fmt_duration | 3878 | (seconds) -> str | 格式化 |
| _drain_commands | 3886 | () | 外部命令 |
| _handle_command | 3901 | (raw) | 命令分发 |
| _normalize_command_tokens | 3954 | (text) -> list[str] | token 规范化 |
| _clear_route | 3980 | (why) | 清空路线 |


### 3.5 tasks/aim.py（1368 行）

| 函数/方法 | 行 | 签名要点 | 职责 |
|---|---|---|---|
| FirePolicy | 152 | moving_lead/sweep/turnaround/static | 开火策略名 |
| fire_box_px | 163 | (box_w, *, near_px, far_px, box_w_far, box_w_near) -> float | 框宽到开火框半宽 |
| RailEstimator | 196 | dataclass（xi, gate_*, jump_floor_px, ...） | 一维运动估计 |
| RailEstimator.__post_init__ | 270 | () | 解析 alpha/beta |
| RailEstimator.gains | 275 | (xi) -> (alpha, beta) | 解析增益 |
| RailEstimator.armed (property) | 282 | () -> bool | 是否可开火 |
| RailEstimator._seed | 288 | (z, ts) | 重建基线（不计 resets） |
| RailEstimator._reinit | 299 | (z, ts, reason) | 重建 + resets++ |
| RailEstimator._block_warmup | 305 | (z, ts, reason) | 折返处置 |
| RailEstimator._trim | 327 | (ts) | 裁剪速度基线 |
| RailEstimator._slope | 332 | () -> float | 端点差分速度 |
| RailEstimator._touch | 342 | (ts, dt) | 采样计数 |
| RailEstimator.update | 348 | (dx, ts) -> bool | 喂一帧 |
| RailEstimator.on_no_frame | 421 | () | 断流清零 |
| _Permit | 434 | dataclass | 许可账本 |
| _Permit.note_hit / allowed / cool_ok / cooldown_left / rearm | 457-484 | ... | 账本操作 |
| AimTask.__init__ | 511 | (bridge, bb, bus, cfg, *, log, group, clock) | 50 Hz |
| _publish_initial | 662 | () | 构造即写安全初值 |
| estimator / fire_ready_state / permits / permits_window (property) | 674-690 | () | 只读 |
| missing_schema_keys (property) | 692 | () -> list[str] | 缺键自检 |
| _opt | 697 | (key, default) -> Any | 可选键读取 + 登记 |
| telemetry | 708 | () -> dict | 最近一拍判定 |
| tick | 715 | () | 主循环 |
| _ingest | 769 | (obs, has_target) -> bool | 喂估计器 |
| _track_visibility | 787 | (obs, has_target, now) | 可见性台账 |
| _on_target_seen | 811 | (now) | 刷新本靶基准 |
| _maybe_rearm | 819 | (now) | 命中锁存解除 |
| _account | 853 | (now) | 命中/弹药/发数 |
| _fire_box_now | 878 | (obs) -> float | 当拍开火框 |
| _lead_px | 883 | (has_target) -> float | 提前量（限幅） |
| _evaluate | 894 | (now, has_target, safety, manual) -> (bool, str) | 开火硬门 |
| _cls_of_locked | 986 | () -> int | 本拍类别 |
| _publish_lead | 993 | (lead) | 发 lead |
| _set_fire_ready | 1000 | (ready, now, reason) | 许可迟滞 + 计数 |
| _update_gimbal_gate | 1059 | (now) | 云台巡迹锁止 |
| _emit_ready | 1126 | (ready) | 发 fire_ready |
| _write_target | 1135 | (now, obs, has_target) | 写 P_TARGET |
| _write_aim | 1163 | (now, obs) | 写 P_AIM |
| _write_telemetry | 1178 | (now, lead, dt) | 写 P_TELEMETRY |
| _update_class | 1252 | (obs, has_target) | 类别锁定 |
| _update_red_confirmed | 1282 | (dt) | 红面早退 |
| setup | 1320 | () | 启动横幅 + 缺键自检 |
| teardown | 1342 | () | 闭锁 + lead 归零 |
| _ts_of | 1358 | (obs) -> float | 观测时间戳 |
| _cls_of | 1362 | (obs) -> int | 观测类别 |

### 3.6 tasks/fix.py（461 行）

| 函数/方法 | 行 | 签名要点 | 职责 |
|---|---|---|---|
| FixTask.__init__ | 87 | (bridge, bb, bus, cfg, *, log, group, clock) | 10 Hz |
| setup | 154 | () | 订阅 EV_POSE_ANCHORED + publish(1.0) |
| teardown | 163 | () | 退出台账 |
| samples (property) | 178 | () -> int | 已接受段数 |
| bounds_text | 183 | () -> str | 比值区间文本 |
| stats | 186 | () -> dict | 估计器健康度 |
| drift_report | 196 | () -> dict | 漂移台账 |
| render_report | 207 | () -> str | 一页纸 |
| tick | 217 | () | 事件到累计到落盘 |
| _drain_events | 234 | (now) | drain EV_POSE_ANCHORED |
| _on_anchored | 242 | (ev, now) | 段闭合 + 四路剔除 |
| _accept | 283 | (ratio, block, now) | 保守更新 |
| _reject | 304 | (reason, detail, now) | 剔除记账 |
| _accumulate | 317 | () | 位移累计 |
| _ds_limit | 349 | (run_m) -> float | ds 上限 |
| _settle_ds | 368 | (s, step) | 结算弧长增量 |
| _rebaseline | 414 | (pose=None, s=None) | 重设基线 |
| _segment_ratio | 429 | () -> float或None | 段比值 |
| _window_mean | 434 | () -> float | 窗口均值 |
| _clamp | 439 | (value) -> float | 夹紧 scale |
| _as_float | 444 | (value, default) -> float | static |
| _publish | 452 | (scale, now) | 写 P_FIX_EST |

### 3.7 tasks/drone.py（1196 行）

| 函数/方法 | 行 | 签名要点 | 职责 |
|---|---|---|---|
| DRONE_PAYLOAD_TYPES | 112 | tuple | 已知报文类型 |
| default_map_path | 120 | () -> Path | 仓库默认地图 |
| FIELD_LAYOUT_VERSION | 145 | int = 2 | 布局协议版本 |
| _layout_cells | 156 | (value, key, *, allow_empty) -> tuple | 格名列表归一化 |
| normalize_field_layout | 181 | (payload) -> (dict或None, str) | 布局校验 |
| CoverageLedger | 314 | class | 片级覆盖账本 |
| CoverageLedger.__init__ / update / _done / summary | 328-402 | ... | 覆盖语义 |
| FieldVerifyConsumer | 405 | class | 核对消费方 |
| FieldVerifyConsumer.__init__ / drain / _consume / _kind_of / summary / unsubscribe | 423-519 | ... | 汇总 |
| DroneTask.__init__ | 530 | (bridge, bb, bus, cfg, *, log, group, clock, local_map) | 5 Hz |
| setup | 610 | () | 载地图 + 建消费方 |
| teardown | 633 | () | 退出台账 |
| state | 654 | () -> DroneState | 快照 |
| bridge_of | 664 | (half, *, now) -> str或None | 转发 bridge_of |
| field_verify_summary | 669 | () -> dict或None | 核对汇总 |
| field_verify_gate | 688 | () -> dict | 处置建议 |
| coverage_summary | 707 | () -> dict或None | 覆盖汇总 |
| tick | 714 | () | 主循环 |
| _drain_inbox | 727 | (now) | 收消息 |
| _handle_raw | 737 | (raw, now) | 单条报文解析 |
| _on_heartbeat | 779 | (now) | 心跳 |
| _maybe_ack | 790 | (payload, now) | 回执 |
| _on_bridge_passable | 808 | (payload, now) | 桥洞情报 |
| _on_target_zone | 827 | (payload, now) | 靶区核对 |
| _on_field_layout | 842 | (payload, now) | 布局校验落盒 |
| _on_field_verify | 892 | (payload, now) | 雷/障/靶核对 |
| _take_coverage | 922 | (payload, now) -> tuple | 覆盖解析 |
| _drain_field_verify | 956 | (now) | 消费事件 |
| _diff | 965 | (tag, got, local, now, *, coverage) | 差异登记 |
| _record_diff | 1028 | (text) | 截断保留 |
| _norm_blocks | 1033 | (blocks) -> tuple | 格名归一化 |
| _warn_missing | 1056 | (msg_type, what, payload, now) | 缺字段日志 |
| _check_heartbeat | 1061 | (now) | 心跳超时 |
| _silent_s / _fmt_age | 1075-1080 | ... | 工具 |
| _car_state_fields | 1085 | () -> tuple | 段级状态 |
| _maybe_send_car_state | 1107 | (now) | 周期上报 |
| _send_car_state | 1121 | (now, phase, extra) -> bool | 上报 |
| _publish | 1149 | (now) | 写 P_DRONE |
| _load_local_map | 1152 | () | 本地地图 |
| _read_yaml / _block_set / _as_float | 1174-1196 | ... | 工具 |

### 3.8 tasks/telemetry.py（295 行）

| 函数/方法 | 行 | 签名要点 | 职责 |
|---|---|---|---|
| render_line | 73 | (tel) -> str | 纯函数渲染一行 |
| build_telemetry | 96 | (bb, *, now, drone_stale_s=30.0) -> Telemetry | 汇总 |
| _num | 166 | (value, default) -> float | 工具 |
| _target_text | 173 | (target) -> str | 靶摘要 |
| _drone_text | 184 | (drone, *, now, stale_s) -> str | 链路摘要 |
| TelemetryTask.__init__ | 209 | (bridge, bb, bus, cfg, *, log, group, clock) | 2 Hz |
| setup | 236 | () | 首拍不印 |
| teardown | 240 | () | 退出 |
| tick | 246 | () | 节流 + 汇总 + 打印 |
| snapshot | 268 | () -> Telemetry或None | 只读 |
| json_line | 272 | () -> str | JSON 行 |
| _write | 289 | (tel, line, now) | read-modify-write |

### 3.9 scout/* 函数索引

| 模块 | 函数/类 | 行 | 签名要点 | 职责 |
|---|---|---|---|---|
| fov.py | Pose | 45 | frozen dataclass | 无人机姿态 |
| fov.py | cell_centers / all_blocks | 69-84 | () | 154 格中心 |
| fov.py | block_key / sorted_blocks / in_bounds | 87-100 | ... | 排序/边界 |
| fov.py | cells_in_view | 103 | (pose) -> tuple | 视场覆盖格 |
| fov.py | footprint_corners / polygon_area | 125-147 | ... | 足迹几何 |
| fov.py | clip_convex / overlap_fraction / cell_overlap_fraction / is_node | 167-215 | ... | 重叠/节点 |
| frames.py | ScoutFrame | 42 | frozen dataclass | 一帧侦察 |
| frames.py | scan_poses | 74 | () -> list[Pose] | 蛇形帧位 |
| frames.py | frames_from_poses / generate_snake_timeline / generate_timeline | 120-140 | ... | 时间线 |
| frames.py | frames_from_cell_route | 148 | (blocks, *, start_t, frame_hz, stride) | 手绘路线转帧位 |
| frames.py | coverage / adjacent_overlaps / cell_overlaps / timeline_duration_s / timeline_stats | 190-242 | ... | 指标 |
| heartbeat.py | heartbeat_schedule | 26 | (change_times, t_end, *, interval_s) -> list | 心跳计划 |
| heartbeat.py | heartbeat_messages | 57 | (layouts, *, t_end, interval_s) | 心跳报文 |
| heartbeat.py | to_json / LayoutFeed | 72-110 | ... | JSON / 状态机 |
| layout.py | TruthMap | 70 | frozen dataclass | 真值图 |
| layout.py | TruthMap.from_map / load | 85-123 | classmethod | 构造/读取 |
| layout.py | all_cells / bridge_rows / blocked_for_driving | 126-137 | property/方法 | 图属性 |
| layout.py | _neighbours | 143 | (block) -> tuple | 四邻 |
| layout.py | step_ok | 153 | (truth, known, cur, nxt) -> bool | 四类硬约束 |
| layout.py | reachable_cells / frontier_cells | 181-213 | ... | BFS/前沿 |
| layout.py | MissionTarget | 219 | frozen dataclass | 目标决策结果 |
| layout.py | known_map_cfg | 233 | (truth, known, *, mines, from_block) -> dict | planner 输入 |
| layout.py | _ensure_second | 256 | (tokens, map_cfg, cfg, source) -> list | 补第二停站 |
| layout.py | plan_mission_route | 268 | (known, truth, *, from_block, covered_mines, done_stops, cfg, max_stops, max_seconds) -> tuple或None | 任务路线规划 |
| layout.py | mission_checkpoints | 329 | (tokens, truth, *, covered_mines, done_stops, max_stops) -> list | 检查点序列 |
| layout.py | plan_mission_target | 359 | (known, truth, from_block, *, ...) -> MissionTarget或None | 任务目标优先 |
| layout.py | _euclid / frontier_target | 395-429 | ... | 回退前沿 |
| layout.py | choose_mission_target / choose_target_cell | 432-461 | ... | 决策入口 |
| layout.py | build_layout | 467 | (truth, known, *, version, ts, car_block, segment_id, target_cell, covered_mines, done_stops, cfg, max_stops) -> dict | v2 报文 |
| layout.py | LayoutStream | 506 | class | 增量布局流 |
| layout.py | LayoutStream.__init__ / known_count / target_cell / target_kind | 522-550 | ... | 状态 |
| layout.py | LayoutStream._reachable / _target_valid / _recompute_target / _emit | 552-619 | ... | 目标重算 |
| layout.py | LayoutStream.feed / retarget / note_segment_done / observe_car_block / mark_covered / layouts_for / layouts_for_timeline | 622-695 | ... | 增量驱动 |
| reference.py | USER_ROUTE_BLOCKS / USER_ROUTE_SOURCE | 31-48 | 常量 | 手绘路线 |
| reference.py | user_route_timeline | 51 | (*, stride, start_t, frame_hz) -> list | 手绘帧位 |
| reference.py | compare_routes | 58 | (generated, *, start_t, frame_hz) -> dict | 对照 |
| __init__.py | ScoutResult | 45 | dataclass | 仿真产物 |
| __init__.py | simulate | 85 | (*, map_path, start_t, frame_hz, car_block, heartbeat_interval_s, t_end, cfg) -> ScoutResult | 一站式仿真 |


---

## §4 数据流与线程模型

### 4.1 线程模型

- 每个任务 = 1 个 threading.Thread（daemon）+ 固定频率（core/task.py:59-105）。
- 绝对时基排下一拍（next_deadline += period），补偿执行耗时；落后太多重置时基避免追赶式连拍
  （core/task.py:146-154）。
- 单拍异常隔离；连续 5 拍异常 -> FAULT + EV_TASK_FAULT + fault_hook（:127-138, 165-177）。
- setup() 在 run() 内、循环前调用；异常 -> 直接 FAULT（:111-115）。
- TaskGroup.start_all/stop_all：逆序停止，stop(timeout=2.0)（core/task.py:209-221）。

### 4.2 通信契约

- Blackboard：RLock + 每项 Entry(value, source, ts)；set/get/age/get_fresh/snapshot/describe
  （core/blackboard.py:97-162）。值必须不可变。
- EventBus：低频事件；Subscription.poll(n) 非阻塞、dropped 计数（core/events.py）。

### 4.3 关键数据流（车端运行时，ASCII 表示）

    bridge.pose()
      -> LocalizationTask._project
           |- 写 P_POSE / P_POSE_RAW / P_ODOM_AGE
           |- 写 P_ROUTE_INDEX / P_ROUTE_S（与 Tracker 同投影语义）
           |- P_MOTION_LIMIT（漂移/边界/航向失可信）
           |- P_FIX（anchor_count/drift/max_drift）
           +- EV_POSE_ANCHORED
                 |                          |
                 v                          v
          MissionTask._current_s       FixTask._on_anchored
                 |                          |
                 v                          v
          P_ROUTE_INDEX/P_ROUTE_S       P_FIX_EST
                 |
                 v
      TrackerTask._resolve_path -> compute_command
                 |
                 |- publish_command（底盘指令唯一写入者）
                 |- P_TELEMETRY[tracker_cmd]
                 +- P_PURSUIT_GEAR

    bridge.vision()
      -> AimTask._ingest -> RailEstimator
           |- P_TARGET（seen/dx/dy/cls/s_rail/v_rail/lost_since）
           |- P_AIM（fire_ready 语义/lead_px/shots/hits/ammo/last_reason）
           |- P_AIM_RED（红面早退）
           |- publish_lead / publish_fire_ready
           +- P_TELEMETRY[aim_cmd]

    bridge.take_drone_messages()
      -> DroneTask
           |- P_DRONE（online/bridge_*）
           |- P_FIELD_LAYOUT（规划真源）
           |- P_FIELD_VERIFY / P_DRONE_COVERAGE
           |- publish_drone（car_state / ack）
           +- EV_BRIDGE_INFO -> MissionTask._drain_bridge_events

    MissionTask.tick
      |- EV_BRIDGE_INFO -> 穿洞格（P_ROUTE.crossing）
      |- 写 P_MISSION_PHASE / P_STOP_POINT / P_ROUTE
      |- publish_fire_gate（FIRE 阶段）
      |- P_TELEMETRY[mission_state]
      +- EV_* 事件

### 4.4 跨任务的键读写契约（本次审计从 docstring 读出，标注为设计约定）

| 键 | 写者 | 读者 | 契约要点 | 证据 |
|---|---|---|---|---|
| P_POSE | Localization | Safety/Tracker/Mission/Fix/Telemetry | 信念位姿；缺失=没有 | localization.py:366-373, 440-446 |
| P_ODOM_AGE | Localization | Safety | 每拍都写；inf=无数据 | localization.py:368-375 |
| P_ROUTE_INDEX / P_ROUTE_S | Localization + Tracker | Mission/Fix/Telemetry | 同投影语义；谁写都写同一投影 | localization.py:398-399; tracker.py:675-677 |
| P_SAFETY | Safety | Tracker/Mission/Aim | 唯一强制停车者；不自动恢复 | safety.py:20-22, 438-439 |
| P_ROUTE | Mission | Localization/Tracker | 不可变 Route | mission.py:2909 |
| P_MISSION_PHASE | Mission | Tracker/Aim/Drone/Telemetry | 白名单驱动 | mission.py:3688-3694; tracker.py:84-92 |
| P_STOP_POINT | Mission | Aim | 停站上下文；active 且 kind==SHOOT 才起转 | aim.py:1060-1092 |
| P_TARGET | Aim | Mission/Telemetry | lost_since 是「丢了多久」 | aim.py:1135-1161 |
| P_AIM | Aim | Mission/Telemetry | fire_gate 字段在本任务语义=已许可 | aim.py:1163-1176 |
| P_AIM_RED | Aim | Mission | 只在 ACQUIRE/FIRE 段累计 | blackboard.py:32-47; aim.py:1282-1315 |
| P_FIX | Localization | Fix/Telemetry | 执行侧偏差台账 | localization.py:1431-1443 |
| P_FIX_EST | Fix | Localization/Telemetry | 估计侧 scale | fix.py:452-461 |
| P_MOTION_LIMIT | Localization | pursuit/controller | None=完全 no-op | blackboard.py:69-87; localization.py:1242-1328 |
| P_DRONE | Drone | Mission | 只判链路；情报真源是事件流 | drone.py:1149-1150; mission.py:1962-1975 |
| P_FIELD_LAYOUT | Drone | main | 规划真源 | drone.py:878-881 |
| P_TELEMETRY | Tracker/Mission/Aim/Telemetry | Safety/各任务 | 约定共享 dict，只塞子键 | telemetry.py:11-18 |
| P_PURSUIT_GEAR | Tracker | 遥测/诊断 | 仅观测；实际生效靠 cruise_mps 直传 | blackboard.py:57-67; tracker.py:745-769 |
| PIVOT_DONE_KEY | Tracker | Mission | (顶点索引, 顶点弧长, done/abandon) | tracker.py:74-82; mission.py:3268-3297 |

### 4.5 故障与恢复模型

| 故障 | 触发 | 动作 | 恢复 |
|---|---|---|---|
| 单拍异常 | 任意 tick 抛异常 | 隔离 + 计数 | 下一拍正常 |
| 连续 5 拍异常 | TaskThread | FAULT + EV_TASK_FAULT + fault_hook | 人工/重启 |
| 任务心跳停滞 | Safety ③ | SAFE_STOP | 显式 clear_safety() |
| 定位失联/越界 | Safety ①② | SAFE_STOP + 逐拍零速 | 显式 clear_safety() |
| 航向失可信 | Localization | P_MOTION_LIMIT(0) + EV_TASK_FAULT | 连续 yaw_clear_ticks 帧自动复位 + Safety 清场 |
| 漂移/边界 | Localization | P_MOTION_LIMIT(降速/停车) | healthy_run 达标自动撤除 |
| 无人机离线 | Drone | online=False，不阻塞 | 心跳恢复自动上线 |
| PIVOT 超时 | Tracker | 放行纯追踪 + PIVOT_DONE_KEY=abandon | 标记 until_s |
| 倒车推车超时 | Tracker | 冷却 2 s；连续 2 次放弃闩锁 | 换段/离开后解除 |
| 停滞绕圈 | Mission | 强制跳点（静止闸拦） | s 前进自动复位 |
| 停站超时 | Mission | 关 fire_gate + 放弃停站 | 进下一停站点 |


---

## §5 配置键汇总

### 5.1 本文档覆盖代码实际读取的键（按模块）

**localization.py**：safety.pose_timeout_s / pursuit.arrival_m / pursuit.drift_warn_m /
fix.anchor_at_waypoint / pursuit.drift_seg_gate_m / localization.stuck_ticks /
localization.stuck_eps_m / localization.stuck_dist_m / localization.yaw_guard /
localization.yaw_diff_gain_dps_per_mps / localization.yaw_diff_spin_dps / localization.yaw_jump_deg /
localization.yaw_clear_ticks / localization.yaw_imu_check_dps / localization.yaw_static_dps /
localization.yaw_min_speed_mps / localization.yaw_bad_ticks / localization.yaw_plausible_speed_mps /
localization.yaw_max_dt_s / localization.yaw_imu_mismatch_ratio / localization.anchor_max_pull_m /
localization.anchor_reject_eps_m / localization.drift_slow_after / localization.drift_slow_speed_mps /
localization.drift_stop_after / localization.drift_clear_ticks / localization.boundary_guard_m /
localization.boundary_guard_speed_mps / localization.boundary_stop_m / localization.reanchor_pull_max_m /
localization.anchor_scale_max / localization.implied_scale_window / localization.yaw_trim_deg /
pursuit.motion_heading_eps_m / pursuit.map_safety_margin_m / logging.rate_limit_s

**safety.py**：safety.pose_timeout_s / safety.task_stall_s / safety.brake_on_fault /
safety.brake_fail_alarm_after / pursuit.map_safety_margin_m / safety.startup_grace_s /
logging.rate_limit_s

**tracker.py**：logging.rate_limit_s / pursuit.align_kp / pursuit.align_steer_limit_deg /
pursuit.align_straight_speed_mps / pursuit.align_straight_before / pursuit.align_straight_after /
pursuit.bridge_blocks / pursuit.max_steer_deg / pursuit.max_steer_rate_deg_s / pursuit.min_speed_mps /
pursuit.cruise_speed_mps / pursuit.dynamic_gear_enabled / pursuit.dynamic_gear_near_dist_m /
pursuit.approach_m / pursuit.pivot_enabled / pursuit.pivot_angle_deg / pursuit.pivot_dist_m /
pursuit.pivot_wait_s / pursuit.pivot_speed_mps / pursuit.pivot_yaw_tol_deg / pursuit.pivot_timeout_s /
pursuit.pivot_kp / pursuit.motion_heading_eps_m / pursuit.projection_rewind_m / pursuit.reverse_entry_m /
pursuit.vertex_floor_m / pursuit.projection_release_m

**mission.py**：mission.auto_start / mission.auto_start_delay_s / mission.reset / mission.scan_mode /
aim.gimbal_lock_enabled / mission.shots_per_stop / aim.red_early_leave_s /
mission.skip_second_stop_after_hit / mission.settle_time_s / mission.settle_eps_m /
mission.settle_speed_mps / mission.lock_timeout_s / mission.shot_timeout_s / mission.scan_stop_timeout_s /
mission.stop_deadline_s / pursuit.arrival_m / pursuit.stop_radius_m / pursuit.approach_m /
mission.stall_timeout_s / mission.stall_yaw_deg / planner.auto_plan_enabled / planner.map_path /
planner.fallback_to_tokens / planner.allow_reverse / planner.max_seconds / planner.engine /
planner.tracker_max_seconds / planner.pivot_penalty / drone.enabled / drone.info_stale_s /
mission.bridge_info_wait_s / mission.bridge_wait_at_start / mission.bridge_wait_max_s /
mission.race_budget_s / mission.bridge_reroute_max / route.cross_lower_default / route.cross_upper_default /
route.cross_lower_candidates / route.cross_upper_candidates / route.tokens / mission_blocks /
path_executor.shoot_distance_min / path_executor.shoot_distance_max / path_executor.shoot_candidates_keep /
planner.footprint_gate / planner.obstacle_margin_m / planner.mine_turn_penalty

**aim.py**：aim.lead_time_s / aim.fire_policy / aim.shot_cooldown_ms / aim.max_shots_per_target /
aim.max_shots_total / aim.lost_frames_to_rescan / aim.vision_nominal_fps / aim.fire_box_far_px /
aim.fire_box_near_px / aim.box_w_far_px / aim.box_w_near_px / aim.cls_switch_confirm_frames /
aim.fire_only_blue / aim.red_early_leave_s / aim.hit_hold_s / aim.rescan_on_lost / aim.rail_span_m /
aim.target_speed_mps / aim.px_per_m_rail / aim.max_lead_factor / aim.max_lead_px / aim.rearm_after_lost_s /
aim.lead_eps_px / aim.hold_min_s / aim.gimbal_lock_enabled / aim.gimbal_lock_scan_request /
aim.gimbal_seek_scan_request / aim.gimbal_seek_on_shoot_stop_only / aim.est_xi / aim.est_gate_k /
aim.est_gate_base_px / aim.est_gate_vel_scale / aim.est_jump_floor_px / aim.reset_confirm_frames /
aim.est_dt_min_s / aim.est_dt_max_s / aim.est_max_speed_px_s / aim.est_vel_window_s /
aim.est_flip_min_speed_px_s / aim.scan_request_on_start / aim.scan_pitch_offset_deg / logging.rate_limit_s

**fix.py**：fix.scale_est_enabled / fix.scale_step_max / fix.scale_limit / pursuit.drift_warn_m /
fix.drift_cull_m / fix.seg_min_m / fix.scale_min_samples / fix.ratio_bounds / fix.ratio_jump_max /
fix.ratio_jump_min_prior / fix.scale_window / fix.ds_max_ratio / fix.ds_floor_m /
fix.ds_reject_max_run / fix.anchor_at_waypoint / fix.anchor_max_offset_m / fix.anchor_gain / logging.rate_limit_s

**drone.py**：drone.enabled / drone.heartbeat_timeout_s / drone.info_stale_s /
drone.car_state_period_s / drone.ack_enabled / drone.field_diff_max / drone.coverage_enabled /
drone.field_verify_policy / drone.map_path / logging.rate_limit_s

**telemetry.py**：telemetry.period_s / drone.info_stale_s / logging.rate_limit_s

### 5.2 SCHEMA 状态（本次审计抽查 core/config.py）

已登记（config.py 中存在）：pursuit.*（:65-186）、planner.*（:234-253）、localization.*（:266-290）、
route.*（:292-302）、mission.*（:319-341, 356-364）、aim.*（:368-441）、fix.*（:444-473）、
localization.stuck_*（:483-488）、localization.yaw_*（:499-521）、drone.*（:526-548）、
safety.*（:552-560）、logging.rate_limit_s（:560）、telemetry.period_s（:561）。

未登记进 SCHEMA（代码用 cfg.get(key, default) 兜底）：

| 键 | 代码位置 | 说明 |
|---|---|---|
| mission.stop_deadline_s | mission.py:1533 | 本审计 grep config.py 无此键；缺省回退 mission.shot_timeout_s |
| drone.coverage_enabled | drone.py:554 | docstring 明说未登记（drone.py:35, 83） |
| drone.field_verify_policy | drone.py:555 | docstring 明说未登记 |

结论：mission.py:141-146 的 docstring 说「只剩 mission.shot_timeout_s 仍缺」，但本次审计实测
config.py:336 已登记 mission.shot_timeout_s；mission.stop_deadline_s 才是真正未登记的键。
即 mission.py 的 SCHEMA 缺键自述**已过期**（文档与代码不一致，§7）。

### 5.3 配置键的「0 是合法值」陷阱

多处用 cfg.get(key) or default 的写法会把显式 0 当成缺失：

- mission.py:1493 float(cfg.get("mission.auto_start_delay_s") or 0.0) —— 0 合法，碰巧同默认，无实际影响；
- mission.py:1571 max(0.0, float(cfg.get("mission.bridge_info_wait_s") or 0.0)) —— 0 合法，碰巧同默认；
- localization.py:250 float(cfg.get("localization.anchor_max_pull_m") or self.arrival_m) —— 若显式配 0，会被 arrival_m(0.12) 顶掉；
- drone.py:546 float(cfg.get("drone.heartbeat_timeout_s") or 3.0) —— 若显式配 0，会被 3.0 顶掉。

localization.py:121 的 _cfg_float 是正确写法（只有 None 才兜底）。本审计实测代码中确有此模式，
但是否引发过现场问题未核实。

---

## §6 测试与证据

### 6.1 测试文件清单（src/dzbrain/tests/，40 个 .py = 37 个 test_*.py + 3 个支撑文件，共 23492 行）

| 测试 | 行数 | check() 计数（grep -c 'check(' 静态计数） |
|---|---|---|
| test_aim.py | 1368 | 111 |
| test_aim_probe.py | — | — |
| test_autoplan.py | — | — |
| test_bridge_gate.py | — | — |
| test_calib_backfill.py | — | — |
| test_car_state_report.py | — | — |
| test_contract.py / test_core.py / test_geometry.py | — | — |
| test_corner_cut.py | 271 | — |
| test_deploy.py | 344 | — |
| test_drift.py | 484 | 30 |
| test_drone.py | 1035 | 173 |
| test_dynamic_gear.py | 259 | — |
| test_fix.py | 414 | 65 |
| test_gimbal_lock.py | 458 | 49 |
| test_goal_stop.py | 234 | — |
| test_incremental_plan.py | 298 | — |
| test_link_e2e.py | 656 | — |
| test_localization.py | 1142 | 155 |
| test_mission.py | 2314 | 244 |
| test_motion_limit.py | 252 | 0（无 check( 字样，可能用别的断言风格） |
| test_path_overlap.py | 162 | — |
| test_physics.py | 434 | — |
| test_pivot.py | 660 | 55 |
| test_planner.py | 1304 | — |
| test_pursuit.py | 947 | — |
| test_referee.py | 365 | — |
| test_route.py | 760 | — |
| test_safety.py | 824 | 185 |
| test_scout.py | 390 | 78 |
| test_shutdown.py | 464 | — |
| test_sim.py | 946 | — |
| test_sim_guard.py | 887 | — |
| test_stepper.py | 1203 | 115 |
| test_telemetry.py | 312 | 62 |
| test_tracker.py | 1237 | 92 |

> check() 计数来自 grep -c 'check('，是静态计数，不等于实际断言执行数；
> test_motion_limit.py 计数为 0 说明它用的是别的断言宏（本审计未细读，§7）。

### 6.2 本次审计实际跑过的验证

| 验证 | 命令 | 结果 |
|---|---|---|
| GBK 编码测试 | python -c "chr(0x26A0).encode('gbk')" | 抛 UnicodeEncodeError（U+26A0 不可 GBK） |
| GBK 编码测试 | python -c "chr(0x2705).encode('gbk')" | 抛 UnicodeEncodeError（U+2705 不可 GBK） |
| GBK 编码测试 | python -c "chr(0x2713).encode('gbk')" | 抛 UnicodeEncodeError |
| GBK 编码测试 | python -c "chr(0x26D4).encode('gbk')" | 抛 UnicodeEncodeError |
| GBK 编码测试 | python -c "chr(0x1F9E9).encode('gbk')" | 抛 UnicodeEncodeError |
| GBK 编码测试 | python -c "chr(0x1F6D1).encode('gbk')" | 抛 UnicodeEncodeError |
| GBK 编码测试 | python -c "chr(0x21A9).encode('gbk')" | 抛 UnicodeEncodeError |
| GBK 编码测试 | python -c "chr(0x25BA).encode('gbk')" | 抛 UnicodeEncodeError |
| GBK 编码测试 | python -c "chr(0x2605).encode('gbk')" | OK（★ 可用） |
| GBK 编码测试 | python -c "chr(0x25CF)/chr(0x25A0)/chr(0x2192)/chr(0x25BC)" | OK |
| SCHEMA 键核对 | grep -n "stop_deadline" core/config.py | 无命中（未登记） |
| SCHEMA 键核对 | grep -n "mission.shot_timeout_s" core/config.py | :336 命中（已登记） |
| SCHEMA 键核对 | grep planner.* core/config.py | :234-253 命中（已登记） |

### 6.3 未在本审计中执行的验证（诚实声明）

本审计没有运行任何测试套件、bench、仿真或车上实跑。因此：

- §0.3、§2 中所有数值均为从源码读出，未通过运行复核；
- §6.1 的 check() 计数是静态 grep，不是执行结果；
- §2 中标注「实测」的数值均来自源码 docstring（如 tracker.py:100-102 的 bench 数字、
  localization.py:71-76 的实车事故），本审计未复现。


---

## §7 未核实清单

> 本节是本审计的边界声明。凡属下列情况者，均不能当作「已确认」：
> 需要 ROS/车端实机、需要复跑 bench/仿真/测试、需要装配侧代码、或仅凭 docstring 断言。

### 7.1 需要实机/ROS 环境才能核实

| 项 | 为什么未核实 | 关联位置 |
|---|---|---|
| 所有 tick 的真实时序、死锁、竞态 | 需在车上多线程实跑 | 全部任务 |
| GBK 炸弹在实车上是否真的打断 tick | 需 Windows GBK 控制台实跑（本审计仅静态确认字符不可 GBK 编码） | §2.1.10 等 |
| 云台 C++ 侧 IDLE 归位语义 | 需 src/dzrknn_dataprocess C++ 源码/实机 | aim.py:30-36 |
| 裁判/弹药计数回路（shoot_count/take_hits/ammo） | 需裁判/IO 层实跑 | aim.py:853-873; mission.py:3748-3749 |
| PIVOT 现场真实触发 | 需现场路线 + 实车 | tracker.py:771-870 |
| 倒车推车闭环真实进入/退出 | 需现场倒车段 + 实车 | tracker.py:505-599 |
| drone ↔ Mission 情报链路端到端 | 需机端 T29 + 车端 | drone.py:808-824; mission.py:2125-2154 |
| field_layout 的规划真源被 main 消费 | 需 main.py 装配代码（不在本文档范围） | drone.py:44-49 |
| 段执行（T27 StepLoop ↔ Mission.request_segment） | 需 main.py 的 StepLoop | mission.py:1789-1891 |

### 7.2 需要复跑 bench / 仿真 / 测试才能核实

| 项 | 建议复跑命令 | 关联位置 |
|---|---|---|
| PIVOT 到达判定（E11/A6/B6） | python tools/dzbrain.py --bench --drive --seconds 240 | mission.py:3268-3297; tracker.py:825-858 |
| 停滞兜底 / 静止闸 | 同上 + grep 兜底次数 | mission.py:3163-3334 |
| 投影卡死自愈 | bench + 离线逐值复算 | localization.py:554-642 |
| 航向守卫误报率 | test_localization.py ⑮ | localization.py:664-844 |
| 漂移护栏双带牙 | test_localization.py | localization.py:1242-1328 |
| Aim 18 组语义 | python src/dzbrain/tests/test_aim.py | aim.py |
| 双射击停站补齐 | test_autoplan / plan_route_from_config | mission.py:1003-1022 |
| 段规划策略性转弯降级 | test_stepper.py | mission.py:1229-1442 |
| scout 覆盖率 / 对照 | python -m dzbrain.tools.sim_cli / test_scout.py | scout/* |
| 全量测试 | python src/dzbrain/tests/test_*.py | tests/* |
| 仿真 | python -m dzbrain.tools.sim_cli | tools/sim.py |

### 7.3 仅凭 docstring 断言、未独立复现

| 项 | docstring 位置 | 状态 |
|---|---|---|
| 20 fps 把 30 px/s 估成 25.31 px/s（−15.6%） | aim.py:200-213 | 未复现 |
| 折返后速度冲到 −61 px/s、重建后 −30.1 px/s | aim.py:305-324 | 未复现 |
| bench 实测 F11 180° 折返无限 PIVOT 循环（240 s 只走到 4.4/34 m） | tracker.py:851-856 | 未复现 |
| 2026-10-17 14:59 K9 事故（s 1.288→14.112 m） | localization.py:71-76; tracker.py:98-103 | 未复现 |
| 实车 17.9% 里程虚增、13/13 吸附 0 告警 | localization.py:271-279 | 未复现 |
| 倒车推车：车继续前冲 0.44 m、s 冻在 2.800 | tracker.py:105-117 | 未复现 |
| run11 日志：Mission 的 scan_request=2 压住 Aim 的 0 | aim.py:24-29; mission.py:1496-1506 | 未复现 |
| D3：DroneState 单槽覆盖导致 lower 丢失 | mission.py:56-67 | 未复现 |

### 7.4 需要其它代码才能核实

| 项 | 说明 |
|---|---|
| io/ros_io.py 的 bridge 接口实现 | 本文档只读了任务侧对 bridge 的调用 |
| io/contract.py 的 OffsetObservation/Coverage/SliceInfo | 只读了枚举与字段名 |
| pursuit/controller.py 的 compute_command / speed_profile | 只读了调用点 |
| pursuit/path.py 的 project / corner_turn_deg / car_heading_on | 只读了调用点 |
| route/parser.py 的 parse_mission/parse_reset/parse_token/detect_crossing | 只读了调用点 |
| route/planner.py 的 plan_route / plan_route_tracker_safe / ensure_second_shoot_stop | 只读了调用点 |
| route/validate.py 的 validate_route / CROSS_* | 只读了调用点 |
| route/model.py 的 Route/Waypoint | 只读了字段用法 |
| main.py 的 build_tasks / StepLoop / plan_route_from_field_layout | 不在本文档范围 |
| config/robot.yaml 的实际取值 | 本审计只核对 core/config.py SCHEMA，未逐键读 robot.yaml |
| core/config.py 的 Config.validate / _flatten | 只读了 SCHEMA 与个别键 |

### 7.5 本审计明确「未核实」的具体结论

1. mission.py:141-146 说 mission.shot_timeout_s 仍缺 SCHEMA —— 已实测过期（config.py:336 已登记）。
2. mission.py:46-49 说 planner.* 未登记 SCHEMA —— 已实测过期（config.py:234-253 已登记）。
3. drone.py:35-36、:83-84 说 drone.coverage_enabled / drone.field_verify_policy 未登记 —— 本次抽查
   config.py 未见，**与 docstring 一致**（未登记属实）。
4. mission.stop_deadline_s 未登记（config.py 无此键）—— 与 docstring 描述一致，且 docstring 未把它列为缺键。
5. §2.1.10-2 的 _implied_scale 分母边界情形 —— 未核实。
6. §2.4.16-3 的 _pivot_in_progress 耦合失败模式 —— 未核实。
7. §2.4.16-4 的 _close_ledger 异常是否中断 FINISH —— 未核实。
8. §2.5.12-1 的 aim._OPTIONAL_KEYS 23 个键是否全部已进 SCHEMA —— 未逐键核对。
9. §2.5.12-2 的 hold_min_s 误迟滞边界 —— 未核实。
10. §2.6.6-2 的 fix.anchor_max_offset_m 与 _TELEPORT_M 的互斥 —— 未核实。
11. §2.7.8-2 的 car_state → LayoutStream 装配 —— 未核实。
12. §2.7.8-3 的 contract.decode_coverage 校验口径 —— 未逐行读。
13. §2.9.7 的 scout/车端射击停站判据一致性 —— 未核实。
14. §2.9.7-3 的 USER_ROUTE_BLOCKS 图源文件是否存在 —— 未核实。

---

## 附：本次审计的写入与合规声明

- 唯一写入目标：D:/User/Desktop/比赛存档/v1/docs/00_项目总览/技术全景_B_车端任务层.md
- 未修改任何源码、配置、测试或其它文档；未执行 git 提交/推送。
- 审计方法：glob/grep/read 逐文件精读（tasks/*.py 8 个、scout/*.py 5 个），辅以 config.py SCHEMA 抽查、
  GBK 编码实测、行数统计。
- 证据等级标注：「实测」= 本次审计直接读到或运行过；「docstring」= 源码注释断言、未独立复现；
  「未核实」= 见 §7。









