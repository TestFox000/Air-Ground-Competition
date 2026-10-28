---
来源: 本次代码级技术审计（2026-10-23，子智能体 auditor）
取证: 实
更新: 2026-10-23
status: 现行
---

# 技术全景 F · 全仓代码审计报告

> 任务卡：task-6（全仓代码审计报告）。审计人：auditor（子智能体）。
> 工作区根：D:/User/Desktop/比赛存档/v1。
> 纪律：**只读代码**，未修改任何源码/配置/测试；唯一写入目标为本文档。
> 取证方式：glob/grep/read 逐文件精读 + git 只读命令（status/diff/log/ls-files/cat-file）+ 门禁脚本只读复跑。

## 严重级定义

| 级 | 判据 |
|---|---|
| **blocker** | 仓库外发即车机被接管、凭据/私钥外泄、冻结区被无登记改动、核心功能必然崩溃或静默失效 |
| **high** | 特定输入/竞态下崩溃、安全绕行、判据失真、配置静默漂移、看门狗失效 |
| **medium** | 逻辑可疑但影响受限、重复代码、文档与代码不符、常量多真源 |
| **low** | 风格/可维护性/低风险改进 |

## §0 导读

### 0.1 审计范围与方法

- **对象**：整个工作区 D:/User/Desktop/比赛存档/v1，含两个 git 仓：
  - 根仓 v1（HEAD bc185b4，远端 github / gitee / origin / public）；
  - 子仓 dzrobot_v2（HEAD 11d9999，远端 origin=gitee.com/circle-of-joy/air_ground.git）；
  - dzsmartscout **不是独立 git 仓**（无 .git，其文件被根仓跟踪）。
- **方法**：先 glob/grep 摸清文件清单与行数，再按「安全 → 冻结区 → 逻辑缺陷 → 重复/多真源 → 测试 → 配置漂移 → 文档一致性」七线逐文件精读。所有结论附 路径:行号 或测试名；无法证实的一律标「未核实」。
- **只读保证**：全程未执行任何写操作；check.py / pytest 均以 PYTHONDONTWRITEBYTECODE=1 只读复跑。

### 0.2 头条结论（按严重级）

1. **【blocker】明文口令（值已脱敏）硬编码进 git 历史并已推远端**。至少 14 个 dzsmartscout 脚本 + 2 个 shell + 1 个 PowerShell 明文写入 SSH/sudo/VNC 口令；setup_drone_access.py 把本机公钥追加进无人机 root 与 duzhong 的 authorized_keys；crack_vnc_passwd.py 用 sudo -S 读 ~/.vnc/passwd。根仓 origin/main 与 public/main 均含 dzsmartscout/setup_drone_access.py（git cat-file -e 命中）。
2. **【high】冻结区登记漂移**：tools/wf/check.py 判 GREEN（3 个脏文件均已登记），但 frozen.json 中 src/dzbrain/dzbrain/tasks/tracker.py 的登记**已失效**（文件已不脏），check.py 已 WARN。
3. **【high】auto_start 双真源互相矛盾**：path_executor.auto_start: true（robot.yaml:336）与 mission.auto_start: false（robot.yaml:703）同时存在，注释却自称「一致」。
4. **【high】auto_start_delay 双真源矛盾**：path_executor.auto_start_delay: 45.0（:337）与 mission.auto_start_delay_s: 10.0（:704），注释写「与 mission.auto_start_delay_s=45.0 一致」。
5. **【high】lost_frames_to_rescan 三处取值分叉**：C++ 默认 60（dzrknn_dataprocess.cpp:1335）、/dzrknn_dataprocess 段 180（robot.yaml:236）、/dzbrain.aim 段 60（robot.yaml:848）。
6. **【high】靶速口径四方矛盾**：aim.target_speed_mps: 0.04（robot.yaml:836）vs 注释「4 cm/s」vs TEAM_STATUS「D-031=1 m/s」vs 决策提交「D-033 靶速改判 0.3 m/s」。
7. **【high】scan_pitch_offset_deg 三处不一致**：SCHEMA 默认 +6.0（config.py:383）、/dzrknn_dataprocess +10.0（robot.yaml:50）、/dzbrain.aim +10.0（robot.yaml:849），注释自相矛盾。
8. **【high】Config.get 默认值分支缺陷**：缺键且调用方传了 default 时返回调用方默认而非 SCHEMA 默认；_schema_hits 一旦标记缺键，后续 override() 不再校验。
9. **【medium】dzactuator 里程计除零无守护**：ticksPerMeter 默认 1000.0 与配置 3280.0 不一致，leftSpeed = Robot_Vel.Left / ticksPerMeter（dzactuator.cpp:1292）无零值保护。
10. **【medium】三套地图口径并存**：config/map.yaml（第六张图，2026-10-17）、config/map_v3.yaml（第三次识图，2026-10-11）、文档「第七张图」（docs/20_资料索引库/历史/03:36）。**2026-10-27 状态更新**：已按用户裁决收敛为「仓库不存现役地图、每局真图」——`config/map.yaml` 与 `config/map_v3.yaml`〔已删〕 均已移出工作树，第六/第四图快照归档于 `artifacts/maps/`。

### 0.3 证据约定

- 正文 路径:行号 均为相对工作区根或相对 dzrobot_v2/ 的 POSIX 路径；跨仓处显式写仓名。
- 标注「〔档〕」= 文档/档案口径，非代码实测；「未核实」= 本次未能证实。
- 所有「现场实测」字样均照抄源码注释，未独立复测。

## §1 模块地图

### 1.1 顶层结构

| 顶层 | 定性 | 规模 | 证据 |
|---|---|---|---|
| dzrobot_v2/ | 智能车（主项目，独立 git 仓） | HEAD 11d9999，tag 至 v4.0.0 | dzrobot_v2: git log/tag |
| dzsmartscout/ | 无人机（非 git 仓，被根仓跟踪） | 2170 文件（历史口径） | dzsmartscout: git ls-files |
| docs/ | 唯一长期文档库（四库） | 00/10/20/30 + history_cache | docs/README.md |
| _tmp_session、_archive、backup | 过程稿/归档/备份（豁免区） | — | AGENTS.md §8.6 |

### 1.2 dzrobot_v2 代码地图（实读行数）

| 包 | 语言 | 规模 | 角色 |
|---|---|---|---|
| src/dzbrain/dzbrain/ | Python | 42 文件 / 27678 行 | ★现役决策层（8 线程） |
| src/dzactuator/ | C++ | dzactuator.cpp 1447 + dzactuator.h 341 | 底盘/串口/里程计/IMU |
| src/dzrknn_dataprocess/ | C++ | .cpp 1725 + .h 372 | 云台自瞄三状态机 + 开火门控 |
| src/dzjudgment/ | C++ | 约 523 行 | 裁判串口/激光 |
| src/rknn_pt/（包名 det_node） | C++17 | 约 4565 行 | RKNN 视觉检测 30 Hz |
| src/car_localization/ | Python | 约 2981 行 | 旧循迹核 path_executor.py |
| config/ | YAML | robot.yaml 968 / map.yaml 41 / map_v3.yaml 46 | 配置真源（冻结区） |
| tools/ | Python/Shell | 26 文件 | 部署/规划/自检/门禁 |
| src/dzbrain/tests/ | Python | 37~38 文件 / 108 个 def test_ | 回归套件 |

### 1.3 dzbrain 文件规模（wc -l 实测，降序）

| 文件 | 行数 | 角色 |
|---|---|---|
| route/planner.py | 4848 | 规划引擎（tour / tracker-safe） |
| tasks/mission.py | 4007 | 任务状态机 + 停站 |
| tools/sim.py | 2309 | 离线仿真 |
| main.py | 2264 | 装配/发车/看门狗 |
| tasks/localization.py | 1444 | 定位/吸附/漂移护栏 |
| tasks/aim.py | 1368 | 云台瞄准/开火 |
| tasks/drone.py | 1196 | 无人机链路 |
| tasks/tracker.py | 892 | 循迹控制（冻结区，R1 在途） |
| core/config.py | 889 | 配置中心 + SCHEMA |
| route/validate.py | 772 | 路线校验 |
| scout/layout.py | 704 | 布局解析 |
| route/physics.py | 678 | 物理判据 |
| io/ros_io.py | 637 | ROS 桥 |
| tasks/safety.py | 529 | ★唯一强制停车者 |
| route/parser.py | 485 | token → Path |
| tasks/fix.py | 461 | 里程比例修正 |
| pursuit/path.py | 456 | 路径投影 |
| io/contract.py | 456 | 契约 |
| pursuit/controller.py | 325 | 速度剖面 |
| tasks/telemetry.py | 295 | 遥测 |
| tools/sim_cli.py | 289 | 仿真 CLI（冻结区，TOOLS-SIM-STUDIO 在途） |
| scout/frames.py | 250 | 帧处理 |
| core/task.py | 240 | 任务线程基类 |
| scout/fov.py | 225 | 视场 |
| route/model.py | 205 | Waypoint/Route |
| io/referee.py | 204 | 裁判 |
| core/geometry.py | 191 | ★坐标/几何唯一真源 |
| core/state.py | 170 | 状态 |
| core/blackboard.py | 162 | 黑板 |
| core/events.py | 140 | 事件总线 |
| scout/__init__.py | 130 | 侦察聚合 |
| scout/heartbeat.py | 114 | 心跳 |
| pursuit/lookahead.py | 100 | 前视 |
| scout/reference.py | 90 | 参考 |
| core/logging.py | 80 | 日志 |
| 其余 __init__/__main__ | ≤21 | 包入口 |

## §2 逐模块详解

### 2.0 模块依赖总览

```
main.py（装配/发车/看门狗）
  ├─ core/  config · geometry · blackboard · events · state · task · logging
  ├─ route/ planner · parser · validate · physics · model
  ├─ tasks/ localization · tracker · mission · aim · drone · fix · safety · telemetry
  ├─ pursuit/ controller · path · lookahead
  ├─ scout/  layout · frames · fov · reference · heartbeat
  └─ io/     ros_io · contract · referee
```

数据流主干：ROS 话题 → io/ros_io.py（桥）→ Blackboard（高频状态）/ EventBus（低频事件）→ 各 TaskThread.tick() → 黑板/事件 → io 发布回 ROS。任务之间**只**经黑板与事件通信（core/task.py:14 明文约束）。

### 2.1 core/ —— 基础设施层

#### 2.1.1 geometry.py（191 行）★坐标唯一真源

- 常量：COL_LETTERS="ABCDEFGHIJKLMN"（:27）、MAP_COLS=14 / MAP_ROWS=11（:28-29）、BLOCK_SIZE=0.35（:30）。
- 米制原点在 N11 格角：x=(10.5-row_idx)*0.35、y=(13.5-col_idx)*0.35（block_to_meters :77-82）。
- 隔板真源：BARRIER_ROW_HI=6 / BARRIER_ROW_LO=7 / BARRIER_COL_FROM="D"（:41-43），注释声明「全仓唯一真源」（:38-40）。
- 函数：block_to_index :54、index_to_block :70、block_to_meters :77、meters_to_block :85、block_to_units :92、units_to_meters :98、meters_to_units :103、map_bounds :108、in_bounds :114、normalize_angle :122、ang_diff :127、deg :132、rad :136、yaw_to_quaternion :140、quaternion_to_yaw :146、heading_token_to_yaw :153、heading_of :161、Pose :171。
- **发现 F-G01（low）**：normalize_angle 用 % 折到 [-π,π)（:124）；对 -π 输入返回 -π（符合 docstring），边界自洽。无缺陷。
- **发现 F-G02（medium）**：index_to_block 对越界输入做**夹紧**（:72-73），而 block_to_index 对非法输入**抛 ValueError**（:56-66）。同一模块内「读格名严格、写格名宽松」的不对称，会把越界坐标静默映射到边缘格，掩盖上游 bug。证据：geometry.py:70-74 vs :54-67。

#### 2.1.2 config.py（889 行）★配置中心

- Spec dataclass（:31-52）：check() 做类型 + 范围校验；bool 与 int/float 显式区分（:43-46）。
- SCHEMA（:63-562）：约 150 个键，覆盖 pursuit / vehicle / field / planner / localization / route / mission / aim / fix / drone / step / safety / logging / telemetry。
- INTENTIONALLY_UNREGISTERED（:570-572）：仅 localization.yaw_trim_deg，理由见 :564-569。
- SECTION_PREFIX（:579-593）：/dzbrain 与 dzbrain 映射为空前缀（展平成 pursuit.* 等），其余节点段保留前缀。
- Config.load（:639-656）：读 YAML → 展平 → _migrate_legacy。
- _migrate_legacy（:665-724）：旧键→新键映射表；**只补不删**（:666）。含物理几何真源迁移（:702-715），修复「规范名 vehicle.* 恒吃 SCHEMA 默认」的 P0（注释 :702-707）。
- Config.get（:727-739）：见 F-C01。
- Config.validate（:773-847）：两级判据（键在 SCHEMA → 合法；段在 SCHEMA 但键未登记 → warn；段不认识 → 透传），见 :797-822。
- 模块级函数：lookahead_distance :861、curve_speed_limit :869、pivot_dist_eff :878。

**发现 F-C01（high）· Config.get 默认值分支缺陷**

- 位置：src/dzbrain/dzbrain/core/config.py:727-739。
- 代码：
  ```python
  def get(self, key, default=None):
      if key in self._flat:
          value = self._flat[key]
          self._check_against_schema(key, value)
          return value
      spec = SCHEMA.get(key)
      if spec is not None:
          self._schema_hits.add(key)
          if spec.critical: raise ConfigError(...)
          if default is None:
              return spec.default
      return default
  ```
- 问题：当键缺失、spec 存在、且调用方显式传了 default（非 None）时，返回的是**调用方默认**，而非 SCHEMA 的 spec.default。调用方若写了一个与 SCHEMA 兜底不同的默认值，就会静默偏离「登记的兜底」。
- 影响：SHEMAS 里明确写「default 取代码兜底值」的语义（见 :231-233 注释）被削弱；default 参数成为第二个真源。
- 建议：缺键时应优先 spec.default，仅在 spec 不存在时用调用方 default；或在文档中明示「显式 default 覆盖 SCHEMA 兜底」。

**发现 F-C02（medium）· _schema_hits 一旦标记缺键，后续 override 不再校验**

- 位置：config.py:732-734（self._schema_hits.add(key)）、:747-754（_check_against_schema 首行 if spec is None or key in self._schema_hits: return）。
- 问题：键缺失时 get() 把 key 加入 _schema_hits；此后即便 override(key, bad_value) 写入 _flat，再 get(key) 也因 key in self._schema_hits 跳过 spec.check。
- 影响：bench/测试用 override() 注入非法值时不会被 SCHEMA 范围校验拦截。
- 建议：override() 后清掉该 key 的 _schema_hits 标记，或让 _check_against_schema 独立于 hits 集合。

**发现 F-C03（medium）· _migrate_legacy 迁移键覆盖顺序**

- 位置：config.py:717-724。
- 问题：if new_key in self._flat: continue —— 只要新键已存在就跳过迁移。若 YAML 同时写了新键与旧键且值不同，**新键胜出**，旧键被忽略（无告警）。
- 影响：双真源并存时静默取新键，运维以为改了旧键却无效。
- 建议：迁移时若新旧键同时存在且不等，记一条 warn。

#### 2.1.3 blackboard.py / events.py / state.py

- blackboard.py（162 行）：高频状态键常量（P_POSE / P_ODOM_AGE / P_SAFETY / P_ROUTE_S 等）+ 读写接口。
- events.py（140 行）：EventBus + Subscription；事件类型常量（EV_TASK_FAULT / EV_SAFETY_STOP / EV_SAFETY_CLEAR / EV_ROUTE_LOADED / EV_WAYPOINT_REACHED / EV_MISSION_FINISHED 等）。
- state.py（170 行）：SafetyState 等冻结 dataclass。
- **发现 F-B01（low）**：P_ROUTE_S 有**三个写入者**（Localization / Tracker / Mission），见 config.py:464-468 注释与 tasks/fix.py。属已知设计，非新缺陷，但任何一处冻结都污染段比值估计。

#### 2.1.4 task.py（240 行）★任务线程基类

- TaskThread.run（:106-163）：固定频率绝对时基排拍（:118-154）；单拍异常隔离（:127-138）；连续异常达 max_consecutive_errors=5（:63）判 FAULT。
- TaskHealth.age（:54-56）：未 tick 过返回 inf。
- TaskGroup（:195-239）：顺序 start、逆序 stop。
- **发现 F-T01（medium）· 心跳 age 与「从未 tick」的竞态**
  - 位置：core/task.py:54-56、tasks/safety.py:372-377。
  - 问题：TaskHealth.age() 对 last_tick_at==0 返回 inf；Safety 的 _check_group 用「not h.last_tick_at + 仍在宽限期内」豁免（:372-373）。但宽限期过后，一个**从未 tick 过但 state=RUNNING** 的任务会被判 stalled（注释 :361-368 已说明此坑并修复）。
  - 残余风险：宽限期 = safety.startup_grace_s（默认 3.0 s，config.py:553）。若某任务 setup() 阻塞 > 3 s（如订阅 ROS 话题超时），宽限过后立即被判 stalled 并触发 SAFE_STOP，且安全**不自动恢复**。
  - 建议：宽限期应按「任务各自周期」而非统一 3 s；或对 setup 阶段单独设更长的宽限。

**发现 F-T02（low）· teardown 异常被吞**
  - 位置：core/task.py:156-159：try: self.teardown() except Exception as exc: self._log(...)。
  - 影响：teardown 失败只记日志，线程仍置 STOPPED。若 teardown 负责发布零速/释放资源，失败会静默。
  - 建议：teardown 失败应上报事件（如 EV_TASK_FAULT 的 teardown 变体）。

### 2.2 route/ —— 规划与校验

#### 2.2.1 planner.py（4848 行）★规划引擎

- 双引擎：tour（历史，逐字节不变）与 tracker-safe（新引擎）。入口 plan_route（约 :2890）。
- 关键函数：check_route_structure（:1635）、_insert_shoot_sided（:2689）、_insert_second_sided（:2693）、plan_route（:2890）、_insert_shoot（:3354）、shoot_distance_window、covering_stops、_TSGrid（tracker-safe 网格）。
- **冻结区在途改动（TRACK-PLAN-B1-20261021）**：git diff 显示 55 行改动（+45/-10），核心是「任务优先口径下允许缺停站」：
  - check_route_structure 新增参数 allow_no_stop=False（:1638）；
  - 停站数 0 时仅在 allow_no_stop and not stops 时放行（:1721-1724）；
  - plan_route 新增 no_shoot_possible 标志（:2907）与「射击窗口内只有起点/终点格」剔除逻辑（:3055-3063）；
  - _insert_shoot 对起点/终点格 continue（:3391-3394）；
  - legacy 口径**不再追加**新候选（:3132-3134）。
- **发现 F-P01（medium）· 在途改动使「缺停站」从 error 降级为可放行**
  - 位置：planner.py:1721-1724（diff 后）。
  - 问题：原判据「射击停站应为 1~2 个」是硬结构校验；改动后 allow_no_stop=True 时 0 停站被放行。若现场靶板确实无法形成合法停站，路线会**无射击**跑完 —— 射击分全丢且无硬报错。
  - 影响：与项目总览「R3 射击停站发 0 中 0」风险叠加；建议至少在 warnings 里给出可 grep 的固定串（已有「缺停站」），并补一条回归断言。
- **发现 F-P02（low）· _insert_shoot 起点/终点 continue 后 detour 未初始化**
  - 位置：planner.py:3388-3394（diff 后）。
  - 代码：if pos in (0, len(order)-1): continue 后 detour = 0.0。逻辑正确（continue 跳过赋值），但 detour 在循环体内被后续使用前必被赋值，无未初始化风险。仅可读性。

#### 2.2.2 physics.py（678 行）★物理判据

- body_corners :171、mine_covered :204、required_obstacle_center_distance_m :227、required_barrier_distance_m :286、bridge_entry_walls。
- 默认参数硬编码：wheelbase_m=0.157 :171、coverage_radius_m=0.035 :204、obstacle_radius_m=0.1275 :227、barrier_width_m=0.02 :286。
- **发现 F-PH01（medium）· 物理判据默认值与 SCHEMA/YAML 三处并行**
  - 位置：physics.py:171,204,227,286 vs config.py:65,196,206,210 vs planner.py:647-653、sim.py:335-340。
  - 问题：同一物理量在 4 处有默认字面量（0.157 / 0.035 / 0.1275 / 0.02）。注释 :235 明言「绝不在本模块写常量」，但默认参数本身就是常量。
  - 影响：改 SCHEMA 不改函数默认，缺键时行为分叉。
  - 建议：函数默认改为 None，内部回退到 SCHEMA 或模块级单一常量。

#### 2.2.3 validate.py / parser.py / model.py

- validate.py（772 行）：MIN_SEGMENT_M = 0.02（:52）等；路线校验。
- parser.py（485 行）：parse_mission() 把 token 编译成 Path。
- model.py（205 行）：Waypoint / Route frozen dataclass。
- **发现 F-V01（low）**：validate.py:52 的 MIN_SEGMENT_M 与 sim.py:626 的 _SEG_STICKY_M=0.02 同值但分属两模块，属平行常量。

### 2.3 tasks/ —— 8 线程任务层

#### 2.3.1 safety.py（529 行）★唯一强制停车者

- 五道检查（:11-18）：定位失联 / 越界 / 任务健康 / EV_TASK_FAULT / 遥控接管。
- _evaluate（:221-328）、_check_group（:330-382）、_update（:403-459）、_publish_brake（:461-475）、_state（:477-490）。
- brake_fail_alarm_after（:71）：连续 N 拍零速发不出去触发独立 alarm。
- **发现 F-S01（high）· brake_fail_alarm_after 未登记进 SCHEMA**
  - 位置：tasks/safety.py:71 读取 safety.brake_fail_alarm_after，但 config.py SCHEMA 的 safety.* 段只有 pose_timeout_s / startup_grace_s / task_stall_s / brake_on_fault（:552-557）。
  - 影响：该键缺失时走 or 3 兜底（:71），但 Config.validate 的两级判据会报「未登记的配置键」warn（因为 safety. 段在 SCHEMA 里）。实际效果：现场无法通过 YAML 调整该阈值而不触发 warn。
  - 建议：登记进 SCHEMA，或改用显式默认并加注释。
- **发现 F-S02（medium）· _finite 不处理 inf**
  - 位置：tasks/safety.py:514-521：return out if not math.isnan(out) else _INF。
  - 问题：inf 原样返回（not isnan(inf) 为 True）。调用方用 == _INF 比较（:244,261,289）恰好匹配，故行为正确；但 _finite 的 docstring 说「None/NaN/inf 一律返回 inf」，实际 inf 透传。属文档与实现轻微不符。
  - 建议：docstring 改为「NaN→inf，inf 透传」。
- **发现 F-S03（low）· _check_group 用 group.health() 每次重建快照**
  - 位置：tasks/safety.py:348 for h in self.group.health().values()。
  - 影响：每次 tick 为每个任务构造 TaskHealth（health() 走 vars() 拷贝，core/task.py:190-192）。20 Hz × N 任务，开销可忽略但非零。

#### 2.3.2 tracker.py（892 行）★循迹（冻结区 R1 在途）

- **冻结区登记状态**：frozen.json 登记 tasks/tracker.py（TRACK-R1-20261020-01，expires 2026-10-27），但 git status 显示 tracker.py **已不脏** → check.py WARN「登记已失效」。
- **发现 F-TR01（medium）· 冻结登记与工作树状态漂移**
  - 位置：tools/wf/frozen.json in_flight[0] vs git status --porcelain。
  - 问题：登记存在但文件已 clean，说明改动**已提交或已回滚**，而登记未清理。check.py 已 WARN（GREEN + WARN）。
  - 影响：登记表不再反映真实在途改动；「过期即红」的机制被绕过（因为文件不脏就不报过期）。
  - 建议：Lead 清理该条登记，或把 tracker.py 的改动补回工作树。
- R1 目标（frozen.json）：Tracker 最终发布限速统一钳位，ALIGN/PIVOT 绕过 P_MOTION_LIMIT。

#### 2.3.3 mission.py（4007 行）★任务状态机

- 停站三段式：SETTLE → ACQUIRE → FIRE（见 test_mission.py docstring）。
- 到站级死线 _stop_deadline_s（:708-710 注释）：发数增量 ≥10 或 30s 硬死线。
- mission.skip_second_stop_after_hit（config.py:400-401）：第一停站发数达标则跳过后续。
- **发现 F-M01（medium）· skip_second_stop_after_hit 与「双面覆盖」目标冲突**
  - 位置：config.py:400-401 + planner.py 在途改动（双停站）。
  - 问题：规划器现在刻意插入**两个**射击停站（板两侧），但 Mission 一旦第一停站发数达标就跳过第二个。两者方向相反：规划器多建停站，执行层提前丢弃。
  - 影响：第二停站永远不会被使用（除非第一停站 0 发）。
  - 建议：明确「双停站」策略下 skip_second_stop_after_hit 的语义（是「本靶打完」还是「全场打完」）。

#### 2.3.4 aim.py（1368 行）★云台瞄准

- aim.lead_time_s（config.py:370-371）：默认 0.0，注释说明「先归零」。
- aim.target_speed_mps（config.py:375）：默认 0.04。
- aim.gimbal_lock_* 四键（config.py:428-441）：单写者锁止。
- **发现 F-A01（high）· lead_time_s 默认 0 与注释「旧 0.35 未对准」并存，但 YAML 写 0.27**
  - 位置：config.py:370-371 vs robot.yaml dzbrain.aim 段（约 :825-835）lead_time_s: 0.27。
  - 问题：SCHEMA 默认 0.0，YAML 0.27；注释说「先归零让 pred=dx 直接瞄准」后又「实车回标 0.27」。语义反复。
  - 影响：改错键名或删 YAML 段会静默回退到 0.0，提前量消失。
  - 建议：在 YAML 段加醒目注释或在 SCHEMA desc 里写「现役值 0.27」。

#### 2.3.5 localization.py / fix.py / drone.py / telemetry.py

- localization.py（1444 行）：吸附/重锚/漂移护栏/边界护栏 + _heal_stuck + _update_yaw_guard（config.py:477-523 注释）。
- fix.py（461 行）：里程比例在线估计；fix.ds_max_ratio / ds_floor_m / ds_reject_max_run（config.py:468-475）。
- drone.py（1196 行）：无人机链路；drone.* 键（config.py:526-549）。
- telemetry.py（295 行）：遥测快照。
- **发现 F-L01（low）· step.pose_ready_wait_s / step.odom_watch_s 与 drone.* 混段**
  - 位置：config.py:536-546：step.wait_timeout_s / step.pose_ready_wait_s / step.odom_watch_s 夹在 drone.* 键之间。
  - 影响：可读性；step. 段只有 3 个键，且没有 step 前缀的节点段。

### 2.4 pursuit/ —— 循迹控制

- controller.py（325 行）：速度剖面 speed_profile()（config.py:80 注释引为真源）；动态档 dynamic_gear_speed()。
- path.py（456 行）：路径投影；projection_rewind_m / vertex_floor_m / projection_release_m 的消费方。
- lookahead.py（100 行）：前视点。
- **发现 F-PU01（medium）· 速度剖面五层与 motion_limit 的施加顺序依赖**
  - 位置：robot.yaml:630 注释：施加顺序 = 四层 min → ×quant → 斜坡 → min_speed → motion_limit（安全层最后）。
  - 问题：该顺序只在注释里，代码里若有人调整层序，限速语义会静默改变。R1 的目标是把 ALIGN/PIVOT 分支纳入同一钳位。
  - 建议：把顺序写成常量/断言，或在 controller.py 里加注释引用行号。

### 2.5 io/ —— ROS 桥与契约

- ros_io.py（637 行）：桥；大量 except Exception 吞异常（:189,568,576,580,584,600）。
- contract.py（456 行）：契约。
- referee.py（204 行）：裁判。
- **发现 F-IO01（medium）· ros_io 的 except Exception 吞掉发布失败**
  - 位置：io/ros_io.py:568-600 连续多个 except Exception。
  - 问题：桥层吞异常后返回 None/默认，上层（Safety 已单独修 _publish_brake）无法区分「发布失败」与「无数据」。
  - 影响：与 safety.py 的 brake_fail_alarm 同源问题；其他发布路径（如 /cmd_vel）可能静默失败。
  - 建议：区分「可恢复（无订阅者）」与「异常」，对后者计数并上报。

### 2.6 scout/ —— 无人机侦察

- layout.py（704 行）：布局解析；except Exception 换引擎（:324）、坏 token 跳过（:344）。
- frames.py（250 行）、fov.py（225 行）、reference.py（90 行）、heartbeat.py（114 行）、__init__.py（130 行）。
- **发现 F-SC01（low）· layout.py 三处 except Exception 静默降级**
  - 位置：scout/layout.py:66,264,324,344。
  - 影响：引擎失败时换下一个引擎或跳过坏 token，若全部失败会返回空布局，Mission 可能用兜底穿洞格继续跑。
  - 建议：全部失败时发 EV 事件或置 P_LAYOUT_INVALID。

### 2.7 tools/ —— 工具与门禁

- deploy_to_car.py（冻结区）：SSH/SCP 部署；PUSH_FILES / BACKUP_PATHS / BUILD_PACKAGES（:54-95）。
- wf/check.py：收口门禁（REQUIRED_DOCS、冻结区、复用、断链）。
- wf/frozen.json：冻结区 + in_flight 登记。
- config_selfcheck.py、route_check.py、plan_route.py、sim_map_render.py、sim_param_sweep.py、global_planner.py、shoot_planner.py 等。
- **发现 F-TO01（high）· deploy_to_car.py 依赖 BatchMode 免密，但无凭据兜底说明**
  - 位置：tools/deploy_to_car.py:49-51（CAR_HOST/CAR_USER/CAR_WS）、:58（SSH_OPTS 含 BatchMode=yes）。
  - 问题：部署走 ssh duzhong@duzhong.local 且 BatchMode=yes（不用口令）。若车端未装本机公钥，部署直接失败且无重试提示。
  - 影响：部署唯一入口；失败时排障成本高。
  - 建议：probe 阶段检测 BatchMode 失败并给出「先跑 install 脚本」的提示。
- **发现 F-TO02（medium）· deploy_to_car.py 推送清单不含 map.yaml（刻意）**
  - 位置：tools/deploy_to_car.py:67-72 注释：不推 config/map.yaml（D-019）。
  - 影响：车端无地图文件，规划依据完全依赖无人机 field_layout。若无人机链路失效，车端无法自规划。
  - 建议：文档中明确「无地图 + 无无人机 = 无法规划」的降级路径。

### 2.8 dzrobot_v2 其他包

#### 2.8.1 dzactuator（C++，冻结区外）

- dzactuator.cpp（1447 行）：串口读写、里程计积分、IMU、看门狗。
- dzactuator.h（341 行）：帧协议常量（FRAME_HEADER 0X7B :62、FRAME_TAIL 0X7D :63、RECEIVE_DATA_SIZE 34 :64、SEND_DATA_SIZE 11 :65）、CARL 0.15（:68）、CARW 0.157（:69）、ACTIVITY_TIMEOUT 0.5（:320）、CMD_WATCHDOG_S 1.0（:322）。
- 里程计积分（:1289-1311）：leftDistance/rightDistance 累加；linear_Speed = (left+right)/2；ThetaSpeed = (R-L)*2π/ticksPer2PI；X += v*cos(Z)、Y += v*sin(Z)。
- **发现 F-AC01（high）· ticksPerMeter / ticksPer2PI 除零无守护**
  - 位置：dzactuator.cpp:1292-1296。
  - 问题：leftSpeed = Robot_Vel.Left / ticksPerMeter；ticksPerMeter 来自参数（:1139），默认 1000.0（:1127）。若参数被设为 0 或参数文件缺失导致 get_parameter 失败（保留 :1086 的 0 初值），除零产生 inf/nan，里程计瞬间发散。
  - 影响：车端定位崩溃 → Safety 判 pose_stale → SAFE_STOP（安全侧有兜底，但排查困难）。
  - 建议：在 :1292 前加 `if (ticksPerMeter <= 0.0) ticksPerMeter = 1000.0;` 之类的守护，或启动时校验。
- **发现 F-AC02（medium）· 参数默认值与配置真源不一致**
  - 位置：dzactuator.cpp:1127-1128（默认 1000.0 / 3200.0）vs robot.yaml:932-941（3280.0 / 2680.0）。
  - 问题：默认值 1000/3200 与现场标定 3280/2680 差 3.28 倍 / 0.84 倍。若参数文件未加载，里程计比例严重失真。
  - 影响：静默的单位/比例错误。
  - 建议：默认值改成与 SCHEMA 一致，或启动时打 WARN 说明「正在用默认值」。
- **发现 F-AC03（medium）· CARL 0.15 vs 配置 wheelbase 0.157**
  - 位置：dzactuator.h:68-69（CARL 0.15 轴距、CARW 0.157 轮距）vs config.py:65（pursuit.wheelbase_m=0.157）。
  - 问题：CARL 0.15 与 0.157 不一致；注释 :69 说「本常量当前无代码引用」，但 CARL 语义是轴距，与 wheelbase_m 冲突。
  - 建议：删除或对齐 CARL。
- **发现 F-AC04（low）· 看门狗只清零 TargetSpeed，不清方向/角度**
  - 位置：dzactuator.cpp:1367-1373。
  - 问题：看门狗触发时只置 TargetSpeed=0、SpeedDirection=0x00，不动云台角度（注释 :1345 说明「只停速度」）。若底盘有惯性，仍会滑行。
  - 影响：低；Safety 层另有越界急停。

#### 2.8.2 dzrknn_dataprocess（C++，冻结区）

- dzrknn_dataprocess.cpp（1725 行）：云台状态机 IDLE/SCAN/TRACK + 开火门控。
- 关键：use_external_fire_gate（:290-291,1386）、lost_frames_to_rescan（:435,1335,1443）、aim_state_（:47,457,961,997,1003,1070,1092）、/nav/scan_request（:951,1268）、/aim/fire_ready（:1157,1292-1294）。
- **发现 F-RK01（medium）· 外部开火门控默认 false，YAML 开 true**
  - 位置：dzrknn_dataprocess.cpp:1386（declare_parameter("use_external_fire_gate", false)）vs robot.yaml（dzrknn 段 use_external_fire_gate: true，约 :244）。
  - 问题：C++ 默认 false，YAML true；若参数文件未加载，开火回到旧行为（仅 aim_ready），与车端 AimTask 的 fire_ready 语义不一致。
  - 影响：射击门控静默回退。
  - 建议：默认值与 YAML 现役值对齐，或启动时 WARN。
- **发现 F-RK02（high）· lost_frames_to_rescan 三处取值分叉（重复 F-头版 5）**
  - 位置：dzrknn_dataprocess.cpp:1335（默认 60）vs robot.yaml:236（/dzrknn_dataprocess 段 180）vs robot.yaml:848（/dzbrain.aim 段 60）。
  - 影响：同一语义三个值；/dzbrain 段注释说「此键在 dzbrain 中无消费者（仅 schema 登记），真源是 dzrknn 段」（:849-850），但两段值不同（180 vs 60），对齐失败。
  - 建议：只保留 dzrknn 段一处，删除 dzbrain 段重复键。

#### 2.8.3 dzjudgment / rknn_pt / car_localization

- dzjudgment（约 523 行）：裁判串口/激光。
- rknn_pt（det_node，约 4565 行）：RKNN 检测；thread_num 参数（robot.yaml:305）。
- car_localization（约 2981 行）：旧循迹核 path_executor.py；技术全景_C 指出内置 MISSION_BLOCKS 20 点 vs 配置 68 token（docs/00_项目总览/技术全景_C_车端硬件与配置.md:231）。
- **发现 F-CL01（medium）· 旧核与新核双轨并存**
  - 位置：src/car_localization/car_localization/path_executor.py vs src/dzbrain/。
  - 问题：robot.yaml:321-332 注释明确「legacy 段与 dzbrain 段双真源必须对齐」，且 `LEGACY_PATH=1` 可切旧核。两套代码同时维护，配置双写。
  - 影响：改一处漏一处；auto_start / linear_speed 双值矛盾即由此产生（见 §5）。
  - 建议：明确退役时间表，或用 config 迁移强制单真源。

### 2.9 dzsmartscout（无人机）

- 顶层脚本（19 个 .py/.sh/.ps1）：setup_drone_access.py、install_drone_key.py、fix_mdns_nsswitch.py、fix_drone_avahi.py、run_fix_drone_avahi.py、rename_drone_hostname.py、enable_mdns.py、probe_drone_vnc.py、probe_drone_dirs.py、verify_new_ip.py、ssh_connect.py、test_drone_users.py、find_drone.py、crack_vnc_passwd.py、vnc_handshake.ps1、remote_fix_drone_avahi.sh 等。
- drone-scripts/：start-lidar.sh、start-lidar-on-link.sh。
- src/：px4_path、FASTLIO2_ROS2、mavros（第三方）、uav_camera_code 等。
- tools/：link/、drone/、perception/、planner/、sim_airground/、reports/。
- **发现 F-DS01（blocker）· 明文口令（值已脱敏）硬编码（详见 §8 F-SEC 系列）**
- **发现 F-DS02（high）· 无人机从未自主飞行（文档口径）**
  - 位置：docs/00_项目总览/项目总览.md（无人机线：uXRCE 会话卡死、域号 0 vs 118、从未自主飞行）。
  - 影响：规则强制项（读桥顶箭头）无法交付。
- **发现 F-DS03（medium）· dzsmartscout 根目录非 git，凭据脚本靠根仓跟踪**
  - 位置：dzsmartscout 无 .git；`git -C dzsmartscout rev-parse` 返回 v1 根。
  - 影响：dzsmartscout 的文件历史完全由根仓决定，无法独立回滚/审计。

## §3 函数级索引大表（重点 · 尽量穷尽）

> 由 rg -n "^(def|class|async def) " src/dzbrain/dzbrain --glob "*.py" 实测生成（共 429 项）。签名省略参数，仅列名称与位置；角色与风险见 §2 与 §8。

| 位置 | 种类 | 名称 |
|---|---|---|
| src/dzbrain/dzbrain/route/model.py:21 | class | Waypoint |
| src/dzbrain/dzbrain/route/model.py:64 | class | Route |
| src/dzbrain/dzbrain/route/model.py:139 | class | RouteBuilder |
| src/dzbrain/dzbrain/route/model.py:198 | def | make_route |
| src/dzbrain/dzbrain/io/contract.py:25 | class | QosSpec |
| src/dzbrain/dzbrain/io/contract.py:48 | class | Topic |
| src/dzbrain/dzbrain/io/contract.py:113 | def | topic |
| src/dzbrain/dzbrain/io/contract.py:123 | def | odom_to_pose |
| src/dzbrain/dzbrain/io/contract.py:136 | def | pose_to_pursuit_twist |
| src/dzbrain/dzbrain/io/contract.py:144 | def | clamp_steer_for_chassis |
| src/dzbrain/dzbrain/io/contract.py:150 | def | chassis_speed_counts |
| src/dzbrain/dzbrain/io/contract.py:165 | def | is_brake_command |
| src/dzbrain/dzbrain/io/contract.py:172 | class | AimStatus |
| src/dzbrain/dzbrain/io/contract.py:184 | class | MissionPhase |
| src/dzbrain/dzbrain/io/contract.py:202 | class | ShotClass |
| src/dzbrain/dzbrain/io/contract.py:214 | class | OffsetObservation |
| src/dzbrain/dzbrain/io/contract.py:233 | def | decode_offset_center |
| src/dzbrain/dzbrain/io/contract.py:252 | def | decode_aim_status |
| src/dzbrain/dzbrain/io/contract.py:272 | class | SliceInfo |
| src/dzbrain/dzbrain/io/contract.py:297 | class | Coverage |
| src/dzbrain/dzbrain/io/contract.py:341 | def | _norm_coverage_cells |
| src/dzbrain/dzbrain/io/contract.py:372 | def | decode_slice_info |
| src/dzbrain/dzbrain/io/contract.py:390 | def | decode_coverage |
| src/dzbrain/dzbrain/io/contract.py:445 | class | ContractSummary |
| src/dzbrain/dzbrain/route/validate.py:75 | def | _fallback_cfg |
| src/dzbrain/dzbrain/route/validate.py:83 | def | _cfg_get |
| src/dzbrain/dzbrain/route/validate.py:93 | def | _window |
| src/dzbrain/dzbrain/route/validate.py:101 | def | _as_list |
| src/dzbrain/dzbrain/route/validate.py:111 | def | _cross_rows |
| src/dzbrain/dzbrain/route/validate.py:148 | def | _map_section |
| src/dzbrain/dzbrain/route/validate.py:171 | def | _cells |
| src/dzbrain/dzbrain/route/validate.py:178 | def | _rc |
| src/dzbrain/dzbrain/route/validate.py:184 | def | _sign |
| src/dzbrain/dzbrain/route/validate.py:188 | def | _blocks |
| src/dzbrain/dzbrain/route/validate.py:201 | def | _barrier_crossing |
| src/dzbrain/dzbrain/route/validate.py:227 | def | _crossing_row_of_cell |
| src/dzbrain/dzbrain/route/validate.py:238 | def | _is_cross |
| src/dzbrain/dzbrain/route/validate.py:275 | def | _crossing_row |
| src/dzbrain/dzbrain/route/validate.py:297 | def | _crossing_cell |
| src/dzbrain/dzbrain/route/validate.py:312 | def | _crossing_at_column |
| src/dzbrain/dzbrain/route/validate.py:345 | def | crossing_entries |
| src/dzbrain/dzbrain/route/validate.py:379 | class | Issue |
| src/dzbrain/dzbrain/route/validate.py:391 | def | validate_route |
| src/dzbrain/dzbrain/route/validate.py:731 | def | _euc |
| src/dzbrain/dzbrain/route/validate.py:736 | def | _stop_ok |
| src/dzbrain/dzbrain/route/validate.py:742 | def | _arrives_reverse |
| src/dzbrain/dzbrain/route/validate.py:748 | def | _total_length |
| src/dzbrain/dzbrain/route/validate.py:760 | def | summary |
| src/dzbrain/dzbrain/tools/sim.py:138 | class | FieldSpec |
| src/dzbrain/dzbrain/tools/sim.py:157 | def | _strip_comment |
| src/dzbrain/dzbrain/tools/sim.py:169 | def | _parse_inline_list |
| src/dzbrain/dzbrain/tools/sim.py:184 | def | parse_map_yaml |
| src/dzbrain/dzbrain/tools/sim.py:245 | def | _repo_root |
| src/dzbrain/dzbrain/tools/sim.py:259 | def | load_field_spec |
| src/dzbrain/dzbrain/tools/sim.py:295 | def | body_corners |
| src/dzbrain/dzbrain/tools/sim.py:304 | def | body_point_clearance_m |
| src/dzbrain/dzbrain/tools/sim.py:323 | class | FieldObs |
| src/dzbrain/dzbrain/tools/sim.py:557 | class | SegmentStat |
| src/dzbrain/dzbrain/tools/sim.py:693 | class | SimConfig |
| src/dzbrain/dzbrain/tools/sim.py:750 | class | SimResult |
| src/dzbrain/dzbrain/tools/sim.py:804 | def | step_bicycle |
| src/dzbrain/dzbrain/tools/sim.py:821 | def | _dist_point_seg |
| src/dzbrain/dzbrain/tools/sim.py:833 | def | _percentile |
| src/dzbrain/dzbrain/tools/sim.py:851 | def | _ramp_towards |
| src/dzbrain/dzbrain/tools/sim.py:861 | def | _cross_track_m |
| src/dzbrain/dzbrain/tools/sim.py:872 | def | _rms |
| src/dzbrain/dzbrain/tools/sim.py:878 | def | auto_max_steps |
| src/dzbrain/dzbrain/tools/sim.py:892 | def | run_sim |
| src/dzbrain/dzbrain/tools/sim.py:1534 | def | _prev_steer |
| src/dzbrain/dzbrain/tools/sim.py:1538 | def | _record_final |
| src/dzbrain/dzbrain/tools/sim.py:1567 | def | _project_with_seg |
| src/dzbrain/dzbrain/tools/sim.py:1580 | def | _candidates |
| src/dzbrain/dzbrain/tools/sim.py:1588 | def | _pick_candidate |
| src/dzbrain/dzbrain/tools/sim.py:1635 | def | _project_sticky |
| src/dzbrain/dzbrain/tools/sim.py:1688 | def | _window_segments |
| src/dzbrain/dzbrain/tools/sim.py:1709 | def | _seg_index_of_s |
| src/dzbrain/dzbrain/tools/sim.py:1719 | def | _projection_at |
| src/dzbrain/dzbrain/tools/sim.py:1740 | def | _clamp_forward |
| src/dzbrain/dzbrain/tools/sim.py:1795 | def | _anchor_allowed |
| src/dzbrain/dzbrain/tools/sim.py:1815 | def | _advance_anchor |
| src/dzbrain/dzbrain/tools/sim.py:1829 | def | _schedule |
| src/dzbrain/dzbrain/tools/sim.py:1837 | def | _take |
| src/dzbrain/dzbrain/tools/sim.py:1853 | class | _Rng |
| src/dzbrain/dzbrain/tools/sim.py:1973 | def | _fmt_row |
| src/dzbrain/dzbrain/tools/sim.py:1984 | def | format_metrics |
| src/dzbrain/dzbrain/tools/sim.py:2031 | def | rate_desc |
| src/dzbrain/dzbrain/tools/sim.py:2038 | def | format_segment_table |
| src/dzbrain/dzbrain/tools/sim.py:2062 | def | write_csv |
| src/dzbrain/dzbrain/tools/sim.py:2095 | def | kind_of |
| src/dzbrain/dzbrain/tools/sim.py:2104 | def | write_segments_csv |
| src/dzbrain/dzbrain/tools/sim.py:2125 | def | _traj_color |
| src/dzbrain/dzbrain/tools/sim.py:2134 | def | write_svg |
| src/dzbrain/dzbrain/tools/sim.py:2307 | def | _xml_escape |
| src/dzbrain/dzbrain/tools/sim_cli.py:50 | def | build_parser |
| src/dzbrain/dzbrain/tools/sim_cli.py:103 | def | _resolve_route |
| src/dzbrain/dzbrain/tools/sim_cli.py:116 | def | _parse_set_overrides |
| src/dzbrain/dzbrain/tools/sim_cli.py:143 | def | _apply_overrides |
| src/dzbrain/dzbrain/tools/sim_cli.py:154 | def | _json_sanitize |
| src/dzbrain/dzbrain/tools/sim_cli.py:167 | def | _emit_json_error |
| src/dzbrain/dzbrain/tools/sim_cli.py:174 | def | main |
| src/dzbrain/dzbrain/io/referee.py:74 | class | RefereeFrame |
| src/dzbrain/dzbrain/io/referee.py:95 | class | RefereeDelta |
| src/dzbrain/dzbrain/io/referee.py:107 | class | RefereeTracker |
| src/dzbrain/dzbrain/main.py:85 | def | install_stop_signals |
| src/dzbrain/dzbrain/main.py:114 | def | ros_context_alive |
| src/dzbrain/dzbrain/main.py:130 | def | default_config_path |
| src/dzbrain/dzbrain/main.py:142 | def | load_and_check |
| src/dzbrain/dzbrain/main.py:214 | def | load_field_layout |
| src/dzbrain/dzbrain/main.py:231 | def | plan_route_from_field_layout |
| src/dzbrain/dzbrain/main.py:318 | def | plan_startup_route |
| src/dzbrain/dzbrain/main.py:362 | def | _step_cells |
| src/dzbrain/dzbrain/main.py:387 | def | normalize_step_layout |
| src/dzbrain/dzbrain/main.py:455 | def | load_step_layout |
| src/dzbrain/dzbrain/main.py:466 | def | _step_mode_enabled |
| src/dzbrain/dzbrain/main.py:471 | def | all_map_cells |
| src/dzbrain/dzbrain/main.py:477 | def | _segment_map_cfg |
| src/dzbrain/dzbrain/main.py:497 | def | classify_segment_target |
| src/dzbrain/dzbrain/main.py:519 | def | plan_segment_from_layout |
| src/dzbrain/dzbrain/main.py:579 | def | _find_task |
| src/dzbrain/dzbrain/main.py:587 | class | StepLoop |
| src/dzbrain/dzbrain/main.py:1238 | def | _run_step_loop |
| src/dzbrain/dzbrain/main.py:1250 | def | _layout_mode_enabled |
| src/dzbrain/dzbrain/main.py:1263 | def | _start_layout_listener |
| src/dzbrain/dzbrain/main.py:1294 | def | _ros_diagnosis |
| src/dzbrain/dzbrain/main.py:1365 | def | _contract_skip_requested |
| src/dzbrain/dzbrain/main.py:1370 | def | _endpoint_counts |
| src/dzbrain/dzbrain/main.py:1402 | def | _subscription_topics_of |
| src/dzbrain/dzbrain/main.py:1419 | def | startup_contract |
| src/dzbrain/dzbrain/main.py:1469 | def | _format_contract_table |
| src/dzbrain/dzbrain/main.py:1486 | def | _log_subscriptions |
| src/dzbrain/dzbrain/main.py:1501 | def | _startup_self_check |
| src/dzbrain/dzbrain/main.py:1527 | def | _duplicate_dzbrain_instances |
| src/dzbrain/dzbrain/main.py:1560 | def | _wait_for_pose_ready |
| src/dzbrain/dzbrain/main.py:1617 | def | _wait_for_field_layout |
| src/dzbrain/dzbrain/main.py:1667 | def | build_tasks |
| src/dzbrain/dzbrain/main.py:1703 | def | start_banner |
| src/dzbrain/dzbrain/main.py:1745 | def | _tele_field |
| src/dzbrain/dzbrain/main.py:1754 | def | finish_report |
| src/dzbrain/dzbrain/main.py:1774 | def | run_bench |
| src/dzbrain/dzbrain/main.py:1933 | def | run_plan_only |
| src/dzbrain/dzbrain/main.py:1973 | def | _live_loop |
| src/dzbrain/dzbrain/main.py:2003 | def | run_live |
| src/dzbrain/dzbrain/main.py:2187 | def | main |
| src/dzbrain/dzbrain/route/planner.py:286 | class | PlannerError |
| src/dzbrain/dzbrain/route/planner.py:294 | class | MapSpec |
| src/dzbrain/dzbrain/route/planner.py:310 | def | map_section |
| src/dzbrain/dzbrain/route/planner.py:315 | def | _as_cells |
| src/dzbrain/dzbrain/route/planner.py:320 | def | _checked |
| src/dzbrain/dzbrain/route/planner.py:329 | def | load_map |
| src/dzbrain/dzbrain/route/planner.py:370 | class | Grid |
| src/dzbrain/dzbrain/route/planner.py:536 | def | _barrier_turn_cell |
| src/dzbrain/dzbrain/route/planner.py:614 | def | _door_turn_margin_m |
| src/dzbrain/dzbrain/route/planner.py:632 | def | _phys_params |
| src/dzbrain/dzbrain/route/planner.py:664 | def | _door_turn_required_margin_m |
| src/dzbrain/dzbrain/route/planner.py:686 | def | _turn_deg_from_dirs |
| src/dzbrain/dzbrain/route/planner.py:695 | def | _door_turn_margin_m |
| src/dzbrain/dzbrain/route/planner.py:712 | def | _obstacle_turn_clearance_m |
| src/dzbrain/dzbrain/route/planner.py:734 | def | _required_obstacle_clearance_m |
| src/dzbrain/dzbrain/route/planner.py:745 | def | _mine_pressed_at |
| src/dzbrain/dzbrain/route/planner.py:760 | def | _edge_mine_turn_gap |
| src/dzbrain/dzbrain/route/planner.py:775 | def | _edge_mine_cells |
| src/dzbrain/dzbrain/route/planner.py:786 | def | _moves |
| src/dzbrain/dzbrain/route/planner.py:820 | def | flood |
| src/dzbrain/dzbrain/route/planner.py:874 | def | _reconstruct |
| src/dzbrain/dzbrain/route/planner.py:905 | def | _flood_value |
| src/dzbrain/dzbrain/route/planner.py:922 | def | _heading_of |
| src/dzbrain/dzbrain/route/planner.py:935 | def | _path_reverse |
| src/dzbrain/dzbrain/route/planner.py:943 | def | stop_distance_m |
| src/dzbrain/dzbrain/route/planner.py:952 | def | turn_angle_deg |
| src/dzbrain/dzbrain/route/planner.py:964 | def | plate_center_m |
| src/dzbrain/dzbrain/route/planner.py:988 | def | shoot_distance_window |
| src/dzbrain/dzbrain/route/planner.py:1001 | def | plate_bisector_axis_m |
| src/dzbrain/dzbrain/route/planner.py:1019 | def | plate_face_normal_m |
| src/dzbrain/dzbrain/route/planner.py:1035 | def | barrier_sight_blocked |
| src/dzbrain/dzbrain/route/planner.py:1088 | def | shoot_candidates |
| src/dzbrain/dzbrain/route/planner.py:1158 | def | accept_window |
| src/dzbrain/dzbrain/route/planner.py:1168 | def | covering_stops |
| src/dzbrain/dzbrain/route/planner.py:1189 | def | plate_side_of |
| src/dzbrain/dzbrain/route/planner.py:1216 | def | second_shoot_stop |
| src/dzbrain/dzbrain/route/planner.py:1251 | def | shoot_stop_distance |
| src/dzbrain/dzbrain/route/planner.py:1263 | def | ensure_second_shoot_stop |
| src/dzbrain/dzbrain/route/planner.py:1338 | class | _Stop |
| src/dzbrain/dzbrain/route/planner.py:1360 | def | _collapse |
| src/dzbrain/dzbrain/route/planner.py:1413 | def | _split_long_hops |
| src/dzbrain/dzbrain/route/planner.py:1446 | def | _fix_uturn_reverse |
| src/dzbrain/dzbrain/route/planner.py:1475 | def | _merge_duplicate_stops |
| src/dzbrain/dzbrain/route/planner.py:1497 | def | _barrier_turn_stops |
| src/dzbrain/dzbrain/route/planner.py:1519 | def | mark_pivots |
| src/dzbrain/dzbrain/route/planner.py:1544 | def | _refresh_headings |
| src/dzbrain/dzbrain/route/planner.py:1577 | def | _final_heading |
| src/dzbrain/dzbrain/route/planner.py:1582 | def | _cell |
| src/dzbrain/dzbrain/route/planner.py:1587 | def | _to_route |
| src/dzbrain/dzbrain/route/planner.py:1611 | def | route_tokens |
| src/dzbrain/dzbrain/route/planner.py:1632 | def | check_route_structure |
| src/dzbrain/dzbrain/route/planner.py:1772 | def | assert_legal |
| src/dzbrain/dzbrain/route/planner.py:1806 | def | _angle_of |
| src/dzbrain/dzbrain/route/planner.py:1815 | def | pivot_vertices |
| src/dzbrain/dzbrain/route/planner.py:1838 | def | assert_legal_no_reverse |
| src/dzbrain/dzbrain/route/planner.py:1893 | class | PlanResult |
| src/dzbrain/dzbrain/route/planner.py:2040 | class | _Tour |
| src/dzbrain/dzbrain/route/planner.py:2413 | def | _limits |
| src/dzbrain/dzbrain/route/planner.py:2423 | def | _shoot_accept |
| src/dzbrain/dzbrain/route/planner.py:2435 | def | _cross_pools |
| src/dzbrain/dzbrain/route/planner.py:2481 | def | _default_cross |
| src/dzbrain/dzbrain/route/planner.py:2489 | def | _u_shape_order |
| src/dzbrain/dzbrain/route/planner.py:2543 | def | objective_mode |
| src/dzbrain/dzbrain/route/planner.py:2551 | def | _side_split |
| src/dzbrain/dzbrain/route/planner.py:2560 | def | _side_shape_order |
| src/dzbrain/dzbrain/route/planner.py:2602 | def | _shoot_positions_for |
| src/dzbrain/dzbrain/route/planner.py:2633 | def | _insert_stop_ranged |
| src/dzbrain/dzbrain/route/planner.py:2656 | def | _insert_shoot_sided |
| src/dzbrain/dzbrain/route/planner.py:2693 | def | _insert_second_sided |
| src/dzbrain/dzbrain/route/planner.py:2709 | def | _reachable_any |
| src/dzbrain/dzbrain/route/planner.py:2714 | def | _pick_crossing_tasks |
| src/dzbrain/dzbrain/route/planner.py:2765 | def | _best_effort_result |
| src/dzbrain/dzbrain/route/planner.py:2849 | def | plan_route |
| src/dzbrain/dzbrain/route/planner.py:3377 | def | _insert_shoot |
| src/dzbrain/dzbrain/route/planner.py:3428 | def | _axis_pass_ok |
| src/dzbrain/dzbrain/route/planner.py:3453 | def | _dir_turn_cost |
| src/dzbrain/dzbrain/route/planner.py:3469 | def | _mine_aligned_runs |
| src/dzbrain/dzbrain/route/planner.py:3509 | def | straighten_mines |
| src/dzbrain/dzbrain/route/planner.py:3691 | def | _ts_unit |
| src/dzbrain/dzbrain/route/planner.py:3697 | def | _ts_map_yaw |
| src/dzbrain/dzbrain/route/planner.py:3707 | def | _ts_turn_deg |
| src/dzbrain/dzbrain/route/planner.py:3714 | class | _TSGrid |
| src/dzbrain/dzbrain/route/planner.py:4041 | def | _ts_tight_reversals |
| src/dzbrain/dzbrain/route/planner.py:4055 | def | _ts_leg |
| src/dzbrain/dzbrain/route/planner.py:4111 | def | _ts_cut |
| src/dzbrain/dzbrain/route/planner.py:4122 | def | _ts_phase |
| src/dzbrain/dzbrain/route/planner.py:4172 | def | _ts_search_multi |
| src/dzbrain/dzbrain/route/planner.py:4244 | def | _ts_search |
| src/dzbrain/dzbrain/route/planner.py:4277 | def | corner_cut_enabled |
| src/dzbrain/dzbrain/route/planner.py:4287 | def | _seg_point_dist |
| src/dzbrain/dzbrain/route/planner.py:4298 | def | _min_clear |
| src/dzbrain/dzbrain/route/planner.py:4308 | def | _barrier_line_clear |
| src/dzbrain/dzbrain/route/planner.py:4327 | def | corner_cut_cells |
| src/dzbrain/dzbrain/route/planner.py:4439 | def | _inc_start_cell |
| src/dzbrain/dzbrain/route/planner.py:4453 | def | _inc_covered |
| src/dzbrain/dzbrain/route/planner.py:4469 | def | _inc_shape |
| src/dzbrain/dzbrain/route/planner.py:4503 | def | _ts_search_incremental |
| src/dzbrain/dzbrain/route/planner.py:4583 | def | plan_route_tracker_safe |
| src/dzbrain/dzbrain/pursuit/controller.py:41 | class | PursuitCommand |
| src/dzbrain/dzbrain/pursuit/controller.py:54 | def | motion_limit_speed_limit |
| src/dzbrain/dzbrain/pursuit/controller.py:85 | def | speed_profile |
| src/dzbrain/dzbrain/pursuit/controller.py:165 | def | _dist_to_stop |
| src/dzbrain/dzbrain/pursuit/controller.py:184 | def | _corner_dist |
| src/dzbrain/dzbrain/pursuit/controller.py:191 | def | dynamic_gear_speed |
| src/dzbrain/dzbrain/pursuit/controller.py:219 | def | compute_command |
| src/dzbrain/dzbrain/pursuit/controller.py:318 | def | brake_command |
| src/dzbrain/dzbrain/route/parser.py:96 | def | normalize_token |
| src/dzbrain/dzbrain/route/parser.py:161 | def | _normalize_shoot_directive |
| src/dzbrain/dzbrain/route/parser.py:170 | def | _split_block_suffix |
| src/dzbrain/dzbrain/route/parser.py:183 | def | parse_token |
| src/dzbrain/dzbrain/route/parser.py:246 | def | _looks_like_file |
| src/dzbrain/dzbrain/route/parser.py:256 | def | _read_token_file |
| src/dzbrain/dzbrain/route/parser.py:274 | def | _as_token_list |
| src/dzbrain/dzbrain/route/parser.py:289 | def | _call_shoot_planner |
| src/dzbrain/dzbrain/route/parser.py:306 | def | parse_mission |
| src/dzbrain/dzbrain/route/parser.py:383 | def | _dedup_waypoints |
| src/dzbrain/dzbrain/route/parser.py:409 | def | detect_crossing |
| src/dzbrain/dzbrain/route/parser.py:441 | def | parse_reset |
| src/dzbrain/dzbrain/route/parser.py:461 | def | format_route |
| src/dzbrain/dzbrain/io/ros_io.py:36 | def | required_subscription_topics |
| src/dzbrain/dzbrain/io/ros_io.py:46 | def | watched_topic_names |
| src/dzbrain/dzbrain/io/ros_io.py:50 | def | ros_env |
| src/dzbrain/dzbrain/io/ros_io.py:67 | def | env_problems |
| src/dzbrain/dzbrain/io/ros_io.py:97 | def | format_env_line |
| src/dzbrain/dzbrain/io/ros_io.py:114 | def | _signal_handler_options |
| src/dzbrain/dzbrain/io/ros_io.py:134 | def | _qos |
| src/dzbrain/dzbrain/io/ros_io.py:149 | class | RosStats |
| src/dzbrain/dzbrain/io/ros_io.py:170 | class | RosBridge |
| src/dzbrain/dzbrain/pursuit/path.py:50 | class | Projection |
| src/dzbrain/dzbrain/pursuit/path.py:74 | class | Path |
| src/dzbrain/dzbrain/pursuit/lookahead.py:39 | class | LookaheadPoint |
| src/dzbrain/dzbrain/pursuit/lookahead.py:49 | def | find_lookahead_point |
| src/dzbrain/dzbrain/tasks/aim.py:152 | class | FirePolicy |
| src/dzbrain/dzbrain/tasks/aim.py:163 | def | fire_box_px |
| src/dzbrain/dzbrain/tasks/aim.py:196 | class | RailEstimator |
| src/dzbrain/dzbrain/tasks/aim.py:435 | class | _Permit |
| src/dzbrain/dzbrain/tasks/aim.py:487 | class | AimTask |
| src/dzbrain/dzbrain/tasks/aim.py:1358 | def | _ts_of |
| src/dzbrain/dzbrain/tasks/aim.py:1362 | def | _cls_of |
| src/dzbrain/dzbrain/scout/fov.py:46 | class | Pose |
| src/dzbrain/dzbrain/scout/fov.py:70 | def | cell_centers |
| src/dzbrain/dzbrain/scout/fov.py:82 | def | all_blocks |
| src/dzbrain/dzbrain/scout/fov.py:87 | def | block_key |
| src/dzbrain/dzbrain/scout/fov.py:93 | def | sorted_blocks |
| src/dzbrain/dzbrain/scout/fov.py:99 | def | in_bounds |
| src/dzbrain/dzbrain/scout/fov.py:103 | def | cells_in_view |
| src/dzbrain/dzbrain/scout/fov.py:125 | def | footprint_corners |
| src/dzbrain/dzbrain/scout/fov.py:138 | def | polygon_area |
| src/dzbrain/dzbrain/scout/fov.py:150 | def | _inside |
| src/dzbrain/dzbrain/scout/fov.py:154 | def | _intersect |
| src/dzbrain/dzbrain/scout/fov.py:167 | def | clip_convex |
| src/dzbrain/dzbrain/scout/fov.py:191 | def | overlap_fraction |
| src/dzbrain/dzbrain/scout/fov.py:200 | def | cell_overlap_fraction |
| src/dzbrain/dzbrain/scout/fov.py:209 | def | is_node |
| src/dzbrain/dzbrain/core/blackboard.py:91 | class | Entry |
| src/dzbrain/dzbrain/core/blackboard.py:97 | class | Blackboard |
| src/dzbrain/dzbrain/tasks/drone.py:120 | def | default_map_path |
| src/dzbrain/dzbrain/tasks/drone.py:156 | def | _layout_cells |
| src/dzbrain/dzbrain/tasks/drone.py:181 | def | normalize_field_layout |
| src/dzbrain/dzbrain/tasks/drone.py:314 | class | CoverageLedger |
| src/dzbrain/dzbrain/tasks/drone.py:405 | class | FieldVerifyConsumer |
| src/dzbrain/dzbrain/tasks/drone.py:522 | class | DroneTask |
| src/dzbrain/dzbrain/scout/frames.py:43 | class | ScoutFrame |
| src/dzbrain/dzbrain/scout/frames.py:63 | def | _dedupe_poses |
| src/dzbrain/dzbrain/scout/frames.py:74 | def | scan_poses |
| src/dzbrain/dzbrain/scout/frames.py:120 | def | frames_from_poses |
| src/dzbrain/dzbrain/scout/frames.py:130 | def | generate_snake_timeline |
| src/dzbrain/dzbrain/scout/frames.py:143 | def | _yaw_between |
| src/dzbrain/dzbrain/scout/frames.py:148 | def | frames_from_cell_route |
| src/dzbrain/dzbrain/scout/frames.py:190 | def | coverage |
| src/dzbrain/dzbrain/scout/frames.py:198 | def | adjacent_overlaps |
| src/dzbrain/dzbrain/scout/frames.py:204 | def | cell_overlaps |
| src/dzbrain/dzbrain/scout/frames.py:210 | def | timeline_duration_s |
| src/dzbrain/dzbrain/scout/frames.py:217 | def | timeline_stats |
| src/dzbrain/dzbrain/tasks/localization.py:121 | def | _cfg_float |
| src/dzbrain/dzbrain/tasks/localization.py:145 | class | PoseSource |
| src/dzbrain/dzbrain/tasks/localization.py:155 | class | LocalizationTask |
| src/dzbrain/dzbrain/scout/layout.py:42 | def | _as_cells |
| src/dzbrain/dzbrain/scout/layout.py:50 | def | _repo_root |
| src/dzbrain/dzbrain/scout/layout.py:59 | def | default_config |
| src/dzbrain/dzbrain/scout/layout.py:71 | class | TruthMap |
| src/dzbrain/dzbrain/scout/layout.py:143 | def | _neighbours |
| src/dzbrain/dzbrain/scout/layout.py:153 | def | step_ok |
| src/dzbrain/dzbrain/scout/layout.py:181 | def | reachable_cells |
| src/dzbrain/dzbrain/scout/layout.py:204 | def | frontier_cells |
| src/dzbrain/dzbrain/scout/layout.py:220 | class | MissionTarget |
| src/dzbrain/dzbrain/scout/layout.py:233 | def | known_map_cfg |
| src/dzbrain/dzbrain/scout/layout.py:256 | def | _ensure_second |
| src/dzbrain/dzbrain/scout/layout.py:268 | def | plan_mission_route |
| src/dzbrain/dzbrain/scout/layout.py:329 | def | mission_checkpoints |
| src/dzbrain/dzbrain/scout/layout.py:359 | def | plan_mission_target |
| src/dzbrain/dzbrain/scout/layout.py:395 | def | _euclid |
| src/dzbrain/dzbrain/scout/layout.py:401 | def | frontier_target |
| src/dzbrain/dzbrain/scout/layout.py:432 | def | choose_mission_target |
| src/dzbrain/dzbrain/scout/layout.py:452 | def | choose_target_cell |
| src/dzbrain/dzbrain/scout/layout.py:467 | def | build_layout |
| src/dzbrain/dzbrain/scout/layout.py:506 | class | LayoutStream |
| src/dzbrain/dzbrain/scout/layout.py:688 | def | layouts_for_timeline |
| src/dzbrain/dzbrain/scout/heartbeat.py:26 | def | heartbeat_schedule |
| src/dzbrain/dzbrain/scout/heartbeat.py:57 | def | heartbeat_messages |
| src/dzbrain/dzbrain/scout/heartbeat.py:72 | def | to_json |
| src/dzbrain/dzbrain/scout/heartbeat.py:77 | class | LayoutFeed |
| src/dzbrain/dzbrain/scout/__init__.py:46 | class | ScoutResult |
| src/dzbrain/dzbrain/scout/__init__.py:85 | def | simulate |
| src/dzbrain/dzbrain/tasks/fix.py:84 | class | FixTask |
| src/dzbrain/dzbrain/scout/reference.py:51 | def | user_route_timeline |
| src/dzbrain/dzbrain/scout/reference.py:58 | def | compare_routes |
| src/dzbrain/dzbrain/tasks/safety.py:49 | class | SafetyTask |
| src/dzbrain/dzbrain/tasks/telemetry.py:73 | def | render_line |
| src/dzbrain/dzbrain/tasks/telemetry.py:96 | def | build_telemetry |
| src/dzbrain/dzbrain/tasks/telemetry.py:166 | def | _num |
| src/dzbrain/dzbrain/tasks/telemetry.py:173 | def | _target_text |
| src/dzbrain/dzbrain/tasks/telemetry.py:184 | def | _drone_text |
| src/dzbrain/dzbrain/tasks/telemetry.py:206 | class | TelemetryTask |
| src/dzbrain/dzbrain/tasks/tracker.py:55 | class | PivotPhase |
| src/dzbrain/dzbrain/tasks/tracker.py:120 | class | TrackerTask |
| src/dzbrain/dzbrain/core/state.py:17 | class | SafetyState |
| src/dzbrain/dzbrain/core/state.py:33 | class | StopKind |
| src/dzbrain/dzbrain/core/state.py:38 | class | StopPhase |
| src/dzbrain/dzbrain/core/state.py:46 | class | StopPointState |
| src/dzbrain/dzbrain/core/state.py:69 | class | TargetState |
| src/dzbrain/dzbrain/core/state.py:87 | class | AimState |
| src/dzbrain/dzbrain/core/state.py:104 | class | DroneState |
| src/dzbrain/dzbrain/core/state.py:128 | class | FixState |
| src/dzbrain/dzbrain/core/state.py:152 | class | Telemetry |
| src/dzbrain/dzbrain/tasks/mission.py:305 | class | BridgeIntel |
| src/dzbrain/dzbrain/tasks/mission.py:344 | def | retag_block_token |
| src/dzbrain/dzbrain/tasks/mission.py:354 | def | _token_block |
| src/dzbrain/dzbrain/tasks/mission.py:363 | def | rewrite_crossing_tokens |
| src/dzbrain/dzbrain/tasks/mission.py:421 | def | resolve_plan_engine |
| src/dzbrain/dzbrain/tasks/mission.py:439 | def | plan_budget_seconds |
| src/dzbrain/dzbrain/tasks/mission.py:465 | class | GateResult |
| src/dzbrain/dzbrain/tasks/mission.py:502 | def | route_check_script |
| src/dzbrain/dzbrain/tasks/mission.py:515 | def | load_route_check_module |
| src/dzbrain/dzbrain/tasks/mission.py:536 | def | _run_route_check |
| src/dzbrain/dzbrain/tasks/mission.py:570 | def | check_planned_route |
| src/dzbrain/dzbrain/tasks/mission.py:644 | class | FootprintReport |
| src/dzbrain/dzbrain/tasks/mission.py:682 | def | vehicle_geometry |
| src/dzbrain/dzbrain/tasks/mission.py:715 | def | _map_geometry |
| src/dzbrain/dzbrain/tasks/mission.py:721 | def | _merged_target_rect |
| src/dzbrain/dzbrain/tasks/mission.py:737 | def | footprint_diagnostic |
| src/dzbrain/dzbrain/tasks/mission.py:835 | class | AutoPlanBlocked |
| src/dzbrain/dzbrain/tasks/mission.py:840 | class | PlanOutcome |
| src/dzbrain/dzbrain/tasks/mission.py:887 | def | workspace_root |
| src/dzbrain/dzbrain/tasks/mission.py:899 | def | config_dir |
| src/dzbrain/dzbrain/tasks/mission.py:907 | def | resolve_map_path |
| src/dzbrain/dzbrain/tasks/mission.py:929 | def | _read_map_yaml |
| src/dzbrain/dzbrain/tasks/mission.py:941 | def | plan_route_from_config |
| src/dzbrain/dzbrain/tasks/mission.py:1117 | def | _stop_kind_with_spec |
| src/dzbrain/dzbrain/tasks/mission.py:1150 | def | stop_kind_for_cell |
| src/dzbrain/dzbrain/tasks/mission.py:1173 | class | SegmentPlan |
| src/dzbrain/dzbrain/tasks/mission.py:1209 | def | heading_dir_from_yaw |
| src/dzbrain/dzbrain/tasks/mission.py:1218 | def | _seg_turn_deg |
| src/dzbrain/dzbrain/tasks/mission.py:1224 | def | _seg_unit |
| src/dzbrain/dzbrain/tasks/mission.py:1229 | def | plan_segment_route |
| src/dzbrain/dzbrain/tasks/mission.py:1455 | class | _StopContext |
| src/dzbrain/dzbrain/tasks/mission.py:1476 | class | MissionTask |
| src/dzbrain/dzbrain/core/geometry.py:54 | def | block_to_index |
| src/dzbrain/dzbrain/core/geometry.py:70 | def | index_to_block |
| src/dzbrain/dzbrain/core/geometry.py:77 | def | block_to_meters |
| src/dzbrain/dzbrain/core/geometry.py:85 | def | meters_to_block |
| src/dzbrain/dzbrain/core/geometry.py:92 | def | block_to_units |
| src/dzbrain/dzbrain/core/geometry.py:98 | def | units_to_meters |
| src/dzbrain/dzbrain/core/geometry.py:103 | def | meters_to_units |
| src/dzbrain/dzbrain/core/geometry.py:108 | def | map_bounds |
| src/dzbrain/dzbrain/core/geometry.py:114 | def | in_bounds |
| src/dzbrain/dzbrain/core/geometry.py:122 | def | normalize_angle |
| src/dzbrain/dzbrain/core/geometry.py:127 | def | ang_diff |
| src/dzbrain/dzbrain/core/geometry.py:132 | def | deg |
| src/dzbrain/dzbrain/core/geometry.py:136 | def | rad |
| src/dzbrain/dzbrain/core/geometry.py:140 | def | yaw_to_quaternion |
| src/dzbrain/dzbrain/core/geometry.py:146 | def | quaternion_to_yaw |
| src/dzbrain/dzbrain/core/geometry.py:153 | def | heading_token_to_yaw |
| src/dzbrain/dzbrain/core/geometry.py:161 | def | heading_of |
| src/dzbrain/dzbrain/core/geometry.py:172 | class | Pose |
| src/dzbrain/dzbrain/core/task.py:30 | class | TaskState |
| src/dzbrain/dzbrain/core/task.py:39 | class | TaskHealth |
| src/dzbrain/dzbrain/core/task.py:59 | class | TaskThread |
| src/dzbrain/dzbrain/core/task.py:195 | class | TaskGroup |
| src/dzbrain/dzbrain/core/events.py:52 | class | Event |
| src/dzbrain/dzbrain/core/events.py:62 | class | Subscription |
| src/dzbrain/dzbrain/core/events.py:98 | class | EventBus |
| src/dzbrain/dzbrain/core/logging.py:22 | def | banner |
| src/dzbrain/dzbrain/core/logging.py:32 | def | kv_lines |
| src/dzbrain/dzbrain/core/logging.py:42 | class | Throttle |
| src/dzbrain/dzbrain/core/logging.py:63 | class | FaultGuard |

### 3.1 索引统计

- dzbrain 顶层 def/class 总数：429（含 __init__ 等）。
- 按模块分布（由位置前缀统计）：

  - `core`: 40
  - `io`: 32
  - `main.py`: 40
  - `pursuit`: 12
  - `route`: 139
  - `scout`: 56
  - `tasks`: 58
  - `tools`: 52

## §4 数据流与线程模型

### 4.1 8 线程模型

| 线程 | 频率 | 文件 | 职责 |
|---|---|---|---|
| Localization | 20 Hz | tasks/localization.py | 定位/吸附/漂移护栏/航向守卫 |
| Tracker | 50 Hz | tasks/tracker.py | 循迹控制（冻结区 R1） |
| Mission | 10 Hz | tasks/mission.py | 任务状态机/停站/路线推进 |
| Aim | 50 Hz | tasks/aim.py | 云台瞄准/开火判据 |
| Fix | 10 Hz | tasks/fix.py | 里程比例在线估计 |
| Drone | 5 Hz | tasks/drone.py | 无人机链路/布局接收 |
| Safety | 20 Hz | tasks/safety.py | ★唯一强制停车者 |
| Telemetry | 2 Hz | tasks/telemetry.py | 遥测快照 |

> 频率为项目总览/架构文档口径（docs/00_项目总览/系统架构.md）；线程创建在 main.py。本次未逐行核对 main.py 的实配频率，标「未核实」于 §7。

### 4.2 通信契约

- **Blackboard**（core/blackboard.py，162 行）：高频状态，键常量 `P_POSE` / `P_ODOM_AGE` / `P_SAFETY` / `P_ROUTE_S` / `P_AIM_*` 等。多写者（`P_ROUTE_S` 三个写入者，config.py:464-468）。
- **EventBus**（core/events.py，140 行）：低频事件，`EV_TASK_FAULT` / `EV_SAFETY_STOP` / `EV_SAFETY_CLEAR` / `EV_ROUTE_LOADED` / `EV_WAYPOINT_REACHED` / `EV_MISSION_FINISHED`。
- 任务间**禁止直接调用**（core/task.py:14）。

### 4.3 主数据流

```
[下位机串口] → dzactuator.cpp → /odom /raw(IMU) /PowerVoltage
                                              ↓
                              io/ros_io.py（桥，637 行）
                                              ↓
                    Blackboard.P_POSE / P_ODOM_AGE（高频）
                                              ↓
   Localization(20Hz) → 吸附/漂移护栏 → P_POSE / P_ROUTE_S
                                              ↓
              Tracker(50Hz) → pursuit/controller → /cmd_vel
                                              ↓
              Mission(10Hz) → 停站/路线 → EventBus(EV_WAYPOINT_REACHED)
                                              ↓
              Aim(50Hz) → /aim/fire_ready + /nav/scan_request
                                              ↓
              dzrknn_dataprocess.cpp（云台状态机）→ 开火
                                              ↓
              Safety(20Hz) ← Blackboard/EventBus → /cmd_vel 零速（唯一强制停车）
```

### 4.4 竞态与时序风险

- **R-01（high）启动竞态**：Localization 与 Safety 对「有没有定位」的判断差一拍（safety.py:100-111 注释）。已用 `_ever_had_pose` + 启动宽限修复，但宽限期统一 3 s（F-T01）。
- **R-02（medium）P_ROUTE_S 多写者**：Localization / Tracker / Mission 各维护投影链（config.py:464-468）；任一处冻结导致段比值被污染（fix.ds_max_ratio 兜底）。
- **R-03（medium）EV_TASK_FAULT 与 group 检查双通道**：Safety 同时用 `_check_group`（:330-382）与 `_pump_events`（:495-509）判故障，两通道可能重复计数（`fault_events_seen` vs `task_fault` reason）。
- **R-04（low）EventBus poll 容量**：`_pump_events` 用 `poll(32)`（safety.py:498），若事件积压 >32 会丢事件。

### 4.5 看门狗层级

| 层 | 位置 | 判据 |
|---|---|---|
| 底盘指令看门狗 | dzactuator.cpp:1346-1374 | pursuitAngle 与 cmd_vel 都超 CMD_WATCHDOG_S=1.0s → 清零 TargetSpeed |
| 任务心跳 | core/task.py:119-154 | 单拍异常隔离；连续 5 次 → FAULT |
| 安全看门狗 | tasks/safety.py:221-328 | 定位失联/越界/任务停滞/EV_TASK_FAULT |
| 位姿看门狗 | config.py:544-546（step.odom_watch_s） | DRIVING 中连续 N 秒无 odom → ROS_LOST |

---

## §5 配置键汇总

### 5.1 SCHEMA 登记键清单（实测 87 个）

> 由 config.py 的 `SCHEMA: dict[str, Spec]` 块正则提取（只读，未执行模块）。default 列为源码里的首个实参字面量。

| 段 | 键数 |
|---|---|
| `aim` | 25 |
| `drone` | 8 |
| `fix` | 9 |
| `logging` | 1 |
| `mission` | 13 |
| `pursuit` | 26 |
| `route` | 1 |
| `safety` | 3 |
| `telemetry` | 1 |
| **合计** | **87** |

### 5.2 逐键清单（段 / 键 / 默认值）

| 键 | 默认值 |
|---|---|
| `pursuit.lookahead_min_m` | `0.18` |
| `pursuit.lookahead_max_m` | `0.45` |
| `pursuit.lookahead_k_v` | `1.2` |
| `pursuit.min_speed_mps` | `0.08` |
| `pursuit.cruise_speed_mps` | `0.15` |
| `pursuit.pivot_enabled` | `True` |
| `pursuit.pivot_dist_m` | `0.25` |
| `pursuit.pivot_wait_s` | `0.5` |
| `pursuit.pivot_speed_mps` | `0.08` |
| `pursuit.pivot_yaw_tol_deg` | `15.0` |
| `pursuit.pivot_kp` | `1.0` |
| `pursuit.reverse_speed_ratio` | `0.6` |
| `pursuit.arrival_m` | `0.12` |
| `pursuit.approach_m` | `0.60` |
| `pursuit.stop_radius_m` | `0.17` |
| `pursuit.overshoot_tol_m` | `0.30` |
| `pursuit.corner_angle_deg` | `45.0` |
| `pursuit.corner_slow_dist_m` | `0.45` |
| `pursuit.corner_speed_mps` | `0.12` |
| `pursuit.map_safety_margin_m` | `0.60` |
| `pursuit.drift_warn_m` | `0.30` |
| `pursuit.align_straight_before` | `2.0` |
| `pursuit.align_straight_after` | `1.0` |
| `pursuit.align_straight_speed_mps` | `0.10` |
| `pursuit.align_kp` | `12.0` |
| `pursuit.align_steer_limit_deg` | `8.0` |
| `route.tokens` | `None` |
| `mission.auto_start` | `True` |
| `mission.auto_start_delay_s` | `10.0` |
| `mission.reset` | `"N10` |
| `mission.shots_per_stop` | `3` |
| `mission.shoot_timeout_s` | `30.0` |
| `mission.lock_timeout_s` | `18.0` |
| `mission.settle_time_s` | `0.8` |
| `mission.settle_eps_m` | `0.02` |
| `mission.settle_speed_mps` | `0.02` |
| `mission.stable_timeout_s` | `6.0` |
| `mission.stable_offset_px` | `40.0` |
| `mission.candidate_retry` | `3` |
| `mission.finish_voice_id` | `0` |
| `aim.rail_span_m` | `0.70` |
| `aim.target_speed_mps` | `0.04` |
| `aim.max_shots_total` | `10` |
| `aim.max_shots_per_target` | `3` |
| `aim.shot_cooldown_ms` | `500` |
| `aim.fire_box_far_px` | `14.0` |
| `aim.fire_box_near_px` | `26.0` |
| `aim.box_w_far_px` | `30.0` |
| `aim.box_w_near_px` | `120.0` |
| `aim.cls_switch_confirm_frames` | `5` |
| `aim.est_dt_max_s` | `0.50` |
| `aim.est_dt_min_s` | `0.002` |
| `aim.est_gate_base_px` | `3.0` |
| `aim.est_gate_k` | `3.0` |
| `aim.est_gate_vel_scale` | `0.10` |
| `aim.est_jump_floor_px` | `25.0` |
| `aim.est_vel_window_s` | `0.40` |
| `aim.fire_only_blue` | `True` |
| `aim.hit_hold_s` | `0.5` |
| `aim.hold_min_s` | `0.30` |
| `aim.lead_eps_px` | `0.5` |
| `aim.max_lead_factor` | `0.45` |
| `aim.rescan_on_lost` | `False` |
| `aim.reset_confirm_frames` | `5` |
| `aim.vision_nominal_fps` | `30.0` |
| `fix.anchor_at_waypoint` | `True` |
| `fix.scale_est_enabled` | `True` |
| `fix.scale_step_max` | `0.005` |
| `fix.scale_limit` | `0.10` |
| `fix.seg_min_m` | `0.05` |
| `fix.scale_min_samples` | `3` |
| `fix.ratio_jump_max` | `0.15` |
| `fix.ratio_jump_min_prior` | `3` |
| `fix.scale_window` | `5` |
| `drone.enabled` | `True` |
| `drone.topic_in` | `"/dzlink/drone_to_car"` |
| `drone.topic_out` | `"/dzlink/car_to_drone"` |
| `drone.heartbeat_timeout_s` | `3.0` |
| `drone.info_stale_s` | `30.0` |
| `drone.car_state_period_s` | `0.7` |
| `drone.ack_enabled` | `True` |
| `drone.field_diff_max` | `8` |
| `safety.pose_timeout_s` | `1.0` |
| `safety.task_stall_s` | `1.0` |
| `safety.brake_on_fault` | `True` |
| `logging.rate_limit_s` | `1.0` |
| `telemetry.period_s` | `0.5` |

### 5.3 配置漂移表（已核实项）

| # | 键 | SCHEMA 默认 | robot.yaml 现役 | 位置 | 严重级 | 说明 |
|---|---|---|---|---|---|---|
| D-01 | path_executor.auto_start | True | **true** | robot.yaml:336 | high | 与 mission.auto_start=false 矛盾 |
| D-02 | mission.auto_start | True | **false** | robot.yaml:703 | high | 与上条矛盾 |
| D-03 | path_executor.auto_start_delay | — | 45.0 | robot.yaml:337 | high | 注释称与 mission.auto_start_delay_s=45.0 一致，实际该值为 10.0 |
| D-04 | mission.auto_start_delay_s | 10.0 | 10.0 | robot.yaml:704 | medium | 注释自相矛盾（旧值 2.0 / 必须写 45.0） |
| D-05 | aim.lost_frames_to_rescan | 180 | **180**（/dzrknn_dataprocess）/ **60**（/dzbrain.aim） | robot.yaml:236,848 | high | 三处取值分叉；C++ 默认 60 |
| D-06 | aim.scan_pitch_offset_deg | +6.0 | +10.0 | robot.yaml:50,849 | high | SCHEMA 与两段均不一致；注释自相矛盾 |
| D-07 | aim.target_speed_mps | 0.04 | 0.04 | robot.yaml:836 | high | 与 D-031=1 m/s / D-033=0.3 m/s 矛盾 |
| D-08 | aim.lead_time_s | 0.0 | 0.27 | robot.yaml ~:833 | medium | 注释「先归零」后又「回标 0.27」 |
| D-09 | aim.max_shots_per_target | 3 | **10** | robot.yaml ~:839 | medium | 注释 t26 改 10，SCHEMA 未同步 |
| D-10 | route.shoot_distance_m | 0.80 | 0.90 | robot.yaml:687 | medium | planner 兜底常量 0.90（planner.py） |
| D-11 | route.shoot_distance_tol_m | 0.10 | 0.12 | robot.yaml:688 | medium | 同上 |
| D-12 | pursuit.cruise_speed_mps | 0.15 | 0.35 | robot.yaml:625 | medium | 注释 t38 说 0.40 又「暂回 0.15」，值 0.35 |
| D-13 | pursuit.lookahead_min_m | 0.18 | 0.36 | robot.yaml:621 | medium | 注释 A 族加固 |
| D-14 | pursuit.lookahead_max_m | 0.45 | 0.40 | robot.yaml:622 | medium | 注释 t38 |
| D-15 | planner.pivot_penalty | 0.0 | 5.0 | robot.yaml:789 | medium | 注释明示「default 取代码兜底，不是现场值」 |
| D-16 | planner.auto_plan_enabled | False | false | robot.yaml:779 | low | 一致 |
| D-17 | planner.engine | tour | tour | robot.yaml ~:777 | low | 一致 |
| D-18 | det_node.thread_num | —（未登记） | **2**（脏） | robot.yaml:305 | medium | 冻结区在途改动，未进 SCHEMA |
| D-19 | safety.brake_fail_alarm_after | —（未登记） | — | safety.py:71 | high | 读取但 SCHEMA 未登记 |
| D-20 | fix.ratio_bounds | (0.75,1.35) | — | config.py:459 | low | 注释说里程多报 18% 时上界需 ≥1.18 |
| D-21 | /dzactuator.ticksPerMeter | 1000.0（C++） | 3280.0 | dzactuator.cpp:1127 vs robot.yaml:932 | medium | 默认值与标定不一致 |
| D-22 | /dzactuator.ticksPer2PI | 3200.0（C++） | 2680.0 | dzactuator.cpp:1128 vs robot.yaml:941 | medium | 同上 |
| D-23 | use_external_fire_gate | false（C++） | true | dzrknn_dataprocess.cpp:1386 vs robot.yaml ~:244 | medium | 默认与现役相反 |
| D-24 | mission.bridge_wait_at_start | False | — | config.py:327 | low | 未核实 YAML 是否显式写 |

### 5.4 robot.yaml 段结构（实测）

| 段（行） | 展平前缀 | 内容 |
|---|---|---|
| /dzrknn_dataprocess（:31-249） | dzrknn_dataprocess.* | 云台/开火/标定 |
| /usb_cam_node（:250-287） | usb_cam_node.* | 相机 |
| /dzjudgment（:288-296） | dzjudgment.* | 裁判 |
| /det_node（:297-318） | det_node.* | RKNN 检测（thread_num 脏改） |
| path_executor（:319-612） | path_executor.* | 旧核参数（双真源） |
| /dzbrain（:613-906） | （空前缀）pursuit.* / route.* / mission.* / aim.* / fix.* / localization.* / planner.* | 新核参数 |
| /dzactuator（:907-968） | dzactuator.* | 底盘标定/物理几何 |

> 证据：`awk` 提取顶层键（:31,250,288,297,319,613,907）；SECTION_PREFIX 见 config.py:579-593。

## §6 测试与证据

### 6.1 收口门禁实跑（只读）

命令：`cd dzrobot_v2 && PYTHONDONTWRITEBYTECODE=1 python tools/wf/check.py`（2026-10-23）。输出原文：

```
== 工作流 v2 收口门禁 ==
冻结区脏文件: 3
  [OK ] config/robot.yaml
  [OK ] src/dzbrain/dzbrain/route/planner.py
  [OK ] src/dzbrain/dzbrain/tools/sim_cli.py
  [OK ] 文档链接 checked files: 113 broken: 0
  [OK ] 复用门禁 新文件: 0  未登记: 0  存量 tools 未登记(warn): 14
  [WARN] 登记已失效（文件已不脏，可移除）: src/dzbrain/dzbrain/tasks/tracker.py
结果: GREEN
```

- 结论：**GREEN**，但有 1 条 WARN（F-TR01）。
- 冻结区脏文件 3 个，全部在 `frozen.json` 的 in_flight 中登记（config/robot.yaml → AIM-LAT-20261022；planner.py → TRACK-PLAN-B1-20261021；sim_cli.py → TOOLS-SIM-STUDIO-20261023）。**无未登记脏改动**。
- `frozen.json` 的 in_flight 另有 `tasks/tracker.py`（TRACK-R1-20261020-01），但该文件当前**不脏** → 登记失效（WARN）。

### 6.2 测试框架

- **pytest 未安装**：`python -m pytest src/dzbrain/tests -q --collect-only` → `No module named pytest`（Python 3.12，`C:\Users\TestFox\AppData\Local\Programs\Python\Python312\python.exe`）。
- 测试是**纯脚本**（无 pytest/unittest 框架）：每个 `test_*.py` 自带 `main`/`check` 辅助函数，全过 exit 0、任一失败 exit 1（见 `test_mission.py` docstring :1-23）。
- 全量回归入口：`dzrobot_v2/_run_full_regression.py`（遍历 `src/dzbrain/tests/test_*.py`，逐文件跑、落 `_tmp_full_regression/<name>.log`，正则解析「结果：N/M」）。
- 因此「测试数量」= `def test_` 计数，而非 pytest 收集数。

### 6.3 覆盖统计（实测）

- `src/dzbrain/tests/` 下 `test_*.py`：**37 个文件**（`ls | wc -l`）；全仓（含 backup/_scratch）`find -name "test_*.py"` = 179（含历史副本，**勿用**）。
- `def test_` 总数：**108**（`rg -c "def test_"` 汇总）。
- 断言风格：多数文件用自定义 `check(...)` 而非裸 `assert`；`rg -c "assert"` 只有 `test_planner.py`（21）、`test_stepper.py`（1）、`test_calib_backfill.py`（1）等少数文件直接出现 `assert`。**「绿但无断言」风险**：若某文件只打印不校验，会假绿。本次未逐文件确认每个 test 都有断言 → 标「未核实」。

| 测试文件 | def test_ | 备注 |
|---|---|---|
| test_mission.py | 22 | 停站三段/超时/终点锁存 |
| test_tracker.py | 15 | 循迹（R1 相关） |
| test_route.py | 13 | 路线 |
| test_aim.py | 13 | 瞄准 |
| test_pursuit.py | 11 | 循迹控制 |
| test_pivot.py | 10 | PIVOT |
| test_physics.py | 8 | 物理判据 |
| test_link_e2e.py | 6 | 空地链路 e2e |
| test_corner_cut.py | 5 | 切角（known-red 相关） |
| test_bridge_gate.py | 5 | 桥洞门禁 |
| 其余 27 个文件 | 合计 20 | 见 §3/§6.4 |

### 6.4 无测试 / 弱测试模块

| 模块 | test_*.py 数 | 证据 |
|---|---|---|
| src/dzactuator（C++ 底盘） | **0** | `find src/dzactuator -name "test_*.py"` |
| src/dzrknn_dataprocess（C++ 云台） | **0** | 同上 |
| src/dzjudgment（C++ 裁判） | **0** | 同上 |
| src/rknn_pt / det_node（C++ 检测） | **0** | 同上 |
| src/car_localization（旧核） | 1（test_coords.py） | 仅坐标换算 |

- **发现 F-TE01（medium）· 冻结区 C++ 模块零单测**：`dzactuator`、`dzrknn_dataprocess`、`dzjudgment`、`rknn_pt` 均无 `test_*.py`。云台状态机与开火门控（F-RK01/F-RK02）的回归只能靠 `test_gimbal_lock.py`（Python 侧）与 `tools/test_rknn.py` 间接覆盖。
- **发现 F-TE02（low）· 测试断言强度未机器校验**：无 pytest，`_run_full_regression.py` 只解析「结果：N/M」字符串；若某测试全程不调用 check，会打印 0/0 或静默通过。建议门禁增加「每个 test_*.py 至少 1 条 check 调用」的静态检查。

### 6.5 known-red 基线

- TEAM_STATUS.md 记录基线：**37 套 / 13 红**（2026-10-18 21:20 干净树，不含当前工作树改动；裁决 D-023）。
- `test_sim.py` / `test_sim_guard.py` 注释引用 `docs/回归守护_known_reds.md`（test_sim.py:500-501,605；test_sim_guard.py:378,396,454），并标注多条 known-red 已在 2026-10-16「判据过时」修复。
- `test_motion_limit.py`：robot.yaml:869-873 注释称该测试「断言 YAML 值与代码默认逐条相等」。

### 6.6 证据强度

- 强（代码实测）：§2 各模块函数/行号、§3 全表、§5 SCHEMA 与 YAML 值、§6 门禁输出。
- 中（文档口径）：8 线程频率、回归基线 37/13、无人机「从未自主飞行」。
- 弱/未核实：线程实配频率是否与文档一致、每个测试是否有断言、远端仓库当前是否仍含凭据（只验证了 ref 存在，未拉取最新）。

## §7 未核实清单

| # | 事项 | 原因 |
|---|---|---|
| U-01 | 8 个任务线程在 main.py 中的实配频率是否等于文档的 20/50/10/50/10/5/20/2 Hz | 未逐行读 main.py 的线程构造段 |
| U-02 | 每个 test_*.py 是否每条 test 都调用了断言 | 无 pytest，静态扫描未做全 |
| U-03 | 远端 github/gitee 当前 HEAD 是否仍包含凭据脚本 | 仅用 `git cat-file -e <ref>:<path>` 验证 ref 存在，未 fetch |
| U-04 | `safety.startup_grace_s` 等键在 robot.yaml 是否显式写出 | 只核对了 SCHEMA，未逐键比对 YAML |
| U-05 | dzsmartscout 的 2170 文件 / 422 MB 规模 | 沿用历史口径，未重新统计 |
| U-06 | planner.py 在途改动（TRACK-PLAN-B1）的完整测试覆盖 | 未跑 planner 全量（只读审计） |
| U-07 | `test_motion_limit.py` 是否真的断言 YAML↔代码默认一致 | 未读该测试全文 |
| U-08 | 无人机侧 ROS 域号 0 vs 118 的实际配置位置 | 未深入 dzsmartscout 启动脚本 |
| U-09 | `_backups/`、`_scratch_scout/` 下的测试副本是否被门禁误用 | 未排查 include 路径 |
| U-10 | 根仓 `github/main`、`gitee/main` 是否含凭据脚本 | `git cat-file -e` 返回 no/unknown（ref 可能不存在或路径不在） |

## §8 发现清单（分级）与统计表

> 每条：严重级 / 位置 / 问题 / 证据 / 影响 / 建议。ID 前缀：SEC=安全，FRZ=冻结区，BUG=逻辑缺陷，DUP=重复/多真源，TEST=测试，DOC=文档，PLAN=规划。

### 8.1 安全（凭据 / 授权 / 主机校验）

> **脱敏说明**：本节按仓库规则（AGENTS.md §11.3）对凭据**字面值**做了脱敏（写作 @@<已脱敏>@@）；文件路径与行号、以及「明文口令/授权/主机校验」的定性结论**完整保留**，不影响定位与处置。原始字面值已在 git 历史与远端中存在，处置重点是轮换口令与回收机端 key。

#### F-SEC-01 【blocker】明文口令（值已脱敏）硬编码并进入 git 历史、已推远端
- 位置：
  - `dzsmartscout/enable_mdns.py:6`（`HOST, USER, PW = "10.230.128.188", "root", "<已脱敏>"`）
  - `dzsmartscout/fix_drone_avahi.py:6`、`fix_mdns_nsswitch.py:7`、`install_drone_key.py:7`、`probe_drone_dirs.py:7`、`probe_drone_vnc.py:5`、`rename_drone_hostname.py:6`、`run_fix_drone_avahi.py:5`、`crack_vnc_passwd.py:6`（均 `PW = "<已脱敏>"`）
  - `dzsmartscout/setup_drone_access.py:20`（`password="<已脱敏>"`）、`ssh_connect.py:7`（`PASSWORD = "<已脱敏>"`）、`test_drone_users.py:10`、`verify_new_ip.py:36`
  - `dzsmartscout/vnc_handshake.ps1:5`（`$password = '<已脱敏>'`）
  - `dzsmartscout/drone-scripts/start-lidar.sh:16,21`、`start-lidar-on-link.sh:14`（`echo <已脱敏> | sudo -S ...`）
- 证据：`rg -n "<口令字面量>" dzsmartscout --glob "*.py" --glob "*.sh" --glob "*.ps1" -l` 命中 ≥17 个文件；`git -C v1 cat-file -e origin/main:dzsmartscout/setup_drone_access.py` → 命中；`public/main` 同样命中；根仓 `git ls-files` 中 84 个脚本匹配凭据模式。
- 影响：仓库一旦外发（github/gitee 已配远端），无人机与车机的 SSH/VNC/sudo 口令全部暴露；配合 F-SEC-02 可直接免密 root 登录。
- 建议：① 立即轮换该口令；② 回收/轮换机端 authorized_keys；③ 脚本改为从环境变量或密钥管理读取；④ 对全历史做秘密扫描（gitleaks 等）；⑤ 评估是否需清理 git 历史（重写历史需团队授权）。

#### F-SEC-02 【blocker】setup_drone_access.py 把本机公钥写入无人机 root 与 duzhong 的 authorized_keys
- 位置：`dzsmartscout/setup_drone_access.py:50-58`（`for user, home in (("duzhong","/home/duzhong"),("root","/root"))` → `echo PUB >> authorized_keys`）。
- 证据：源码 :51-58；`install_drone_key.py:28-33` 同样追加。
- 影响：任何拿到该仓库的人可获得无人机 root 与 duzhong 的持久免密登录。
- 建议：与 F-SEC-01 一并处置；改用一次性密钥或带审计的部署流程。

#### F-SEC-03 【high】crack_vnc_passwd.py 用 sudo -S 读 VNC 口令并做离线候选爆破
- 位置：`dzsmartscout/crack_vnc_passwd.py:18-51`（`echo PW | sudo -S ... base64 -w0 /home/duzhong/.vnc/passwd`；候选列表 :27-29；DES 校验 :31-50）。
- 影响：脚本本身是凭据读取工具，留在仓库即扩大暴露面；若被滥用可离线破解 VNC 口令。
- 建议：移出仓库或改为人工现场操作；加使用审计。

#### F-SEC-04 【high】sudo -S 明文口令管道遍布脚本
- 位置：`install_drone_key.py:19`、`fix_mdns_nsswitch.py:14`、`fix_drone_avahi.py:15`、`rename_drone_hostname.py:15`、`run_fix_drone_avahi.py:17`、`start-lidar*.sh`（多处）。
- 证据：`rg -n "sudo -S" dzsmartscout` 命中 9 处。
- 影响：sudo 口令进 shell 历史/进程列表/日志的风险；与 F-SEC-01 叠加。
- 建议：改用 `sudo -S` 从受保护文件读，或配置 NOPASSWD 限定命令。

#### F-SEC-05 【high】paramiko AutoAddPolicy 跳过主机密钥校验
- 位置：`setup_drone_access.py:17`、`install_drone_key.py:13`、`fix_mdns_nsswitch.py:10`、`crack_vnc_passwd.py:9`、`probe_drone_vnc.py`、`probe_drone_dirs.py`（均 `set_missing_host_key_policy(paramiko.AutoAddPolicy())`）。
- 影响：中间人可冒充无人机，配合明文口令直接接管。
- 建议：改用 `RejectPolicy` + 预先 pinned host key。

#### F-SEC-06 【medium】deploy_to_car.py 使用 StrictHostKeyChecking=no
- 位置：`dzrobot_v2/tools/deploy_to_car.py:53`（`SSH_OPTS = ["-o","BatchMode=yes","-o","StrictHostKeyChecking=no",...]`）。
- 影响：部署通道可被中间人劫持；部署是唯一入口（AGENTS §3）。
- 建议：pinned host key 或首次交互确认。

#### F-SEC-07 【medium】硬编码 IP 漂移
- 位置：`enable_mdns.py:6`（10.230.128.188）、`fix_mdns_nsswitch.py:7`（10.240.174.104）、`setup_drone_access.py:13`（192.168.43.17）、`probe_drone_vnc.py:5`。
- 影响：IP 变更后脚本静默连错主机（配合 AutoAddPolicy 更危险）。
- 建议：统一从 mDNS/环境变量解析。

#### F-SEC-08 【low】vnc_handshake.ps1 明文口令与成功回显
- 位置：`dzsmartscout/vnc_handshake.ps1:5,65`。
- 建议：口令走参数/环境变量，回显脱敏。

### 8.2 冻结区

#### F-FRZ-01 【high】tracker.py 冻结登记已失效（check.py WARN）
- 位置：`dzrobot_v2/tools/wf/frozen.json` in_flight[0]（`src/dzbrain/dzbrain/tasks/tracker.py`，TRACK-R1-20261020-01）vs `git status --porcelain`（tracker.py 不在脏文件列表）。
- 证据：check.py 输出 `[WARN] 登记已失效（文件已不脏，可移除）: src/dzbrain/dzbrain/tasks/tracker.py`。
- 影响：登记表失真；「过期即红」机制被绕过。
- 建议：Lead 清理该条登记，或把 R1 改动补回工作树并更新登记。

#### F-FRZ-02 【medium】config/robot.yaml 在途改动（thread_num 8→2）未进 SCHEMA
- 位置：`dzrobot_v2/config/robot.yaml:305-307`（脏改）+ `dzrobot_v2/tools/wf/frozen.json` in_flight[3]（AIM-LAT-20261022）。
- 问题：`det_node.thread_num` 未登记进 `core/config.py` SCHEMA，`Config.validate` 的两级判据会对它报「未登记的配置键」warn（`det_node.` 是 node_prefixes 之一 → 实际会**透传不报**，见 config.py:809-816）。
- 影响：该键无类型/范围校验；改错值（如 0）无告警。
- 建议：登记进 SCHEMA 或加启动期范围校验。

#### F-FRZ-03 【info】3 个冻结区脏文件均已登记，无未登记违规
- 证据：check.py GREEN；`git status --porcelain -- <frozen paths>` = `M config/robot.yaml`、`M planner.py`、`M sim_cli.py`，三者均在 frozen.json。

### 8.3 逻辑缺陷 / 潜在 bug

#### F-BUG-01 【high】Config.get 默认值分支缺陷
- 位置：`src/dzbrain/dzbrain/core/config.py:727-739`。
- 问题：缺键 + spec 存在 + 调用方传了非 None default → 返回调用方 default，而非 `spec.default`。
- 影响：SCHEMA 的「兜底值」语义被调用方默认值覆盖，成为第二真源。
- 建议：缺键优先 `spec.default`。

#### F-BUG-02 【medium】_schema_hits 使后续 override 不再校验
- 位置：`config.py:732-734, 747-754`。
- 问题：缺键时 key 进入 `_schema_hits`；此后 `override(key, bad)` 再 `get` 跳过 `spec.check`。
- 建议：`override()` 清除该 key 的 hits 标记。

#### F-BUG-03 【medium】_migrate_legacy 静默跳过已存在的新键
- 位置：`config.py:717-724`。
- 问题：`if new_key in self._flat: continue`；新旧键并存且不等时新键胜出，无告警。
- 建议：不等时记 warn。

#### F-BUG-04 【medium】dzactuator 里程计除零无守护
- 位置：`src/dzactuator/src/dzactuator.cpp:1292-1296`（`Robot_Vel.Left / ticksPerMeter`、`... / ticksPer2PI`）。
- 问题：`ticksPerMeter` 初值 0（:1086），默认参数 1000.0（:1127），配置 3280.0（robot.yaml:932）；若参数未加载或为 0，除零产生 inf/nan。
- 影响：定位瞬间发散 → Safety 判 pose_stale → SAFE_STOP。
- 建议：除法前守护 + 启动校验。

#### F-BUG-05 【medium】dzactuator 参数默认值与标定真源不一致
- 位置：`dzactuator.cpp:1127-1128`（1000.0 / 3200.0）vs `robot.yaml:932,941`（3280.0 / 2680.0）。
- 影响：参数文件缺失时里程比例失真 3.28× / 0.84×，静默。
- 建议：默认值对齐或启动 WARN。

#### F-BUG-06 【medium】ros_io 的 except Exception 吞掉发布失败
- 位置：`src/dzbrain/dzbrain/io/ros_io.py:568-600`（连续多个 except）。
- 影响：除 Safety 的 `_publish_brake` 外，其他发布路径（如 /cmd_vel）失败无法区分。
- 建议：区分可恢复/异常，异常计数上报。

#### F-BUG-07 【medium】任务心跳宽限期统一 3 s，setup 阻塞会误判 stalled
- 位置：`core/task.py:54-56`、`tasks/safety.py:372-377`、`config.py:553`。
- 影响：某任务 setup 阻塞 > 3 s → SAFE_STOP 且不自动恢复。
- 建议：宽限按任务周期或 setup 阶段单独设置。

#### F-BUG-08 【low】teardown 异常被吞
- 位置：`core/task.py:156-159`。
- 建议：teardown 失败上报事件。

#### F-BUG-09 【low】_finite docstring 与实现不符（inf 透传）
- 位置：`tasks/safety.py:514-521`。
- 建议：修正 docstring。

#### F-BUG-10 【medium】index_to_block 夹紧 vs block_to_index 抛错不对称
- 位置：`core/geometry.py:70-74` vs `:54-67`。
- 影响：越界坐标静默映射到边缘格。
- 建议：统一策略或加 debug 断言。

#### F-BUG-11 【medium】物理判据默认值 4 处并行
- 位置：`route/physics.py:171,204,227,286` vs `config.py:65,196,206,210` vs `planner.py:647-653`、`sim.py:335-340`。
- 建议：默认改 None，回退单一真源。

#### F-BUG-12 【low】scout/layout.py 三处 except 静默降级
- 位置：`scout/layout.py:66,264,324,344`。
- 影响：全部引擎失败时返回空布局，Mission 用兜底穿洞格继续。
- 建议：全失败时置 P_LAYOUT_INVALID / 发事件。

#### F-BUG-13 【low】EventBus poll(32) 可能丢事件
- 位置：`tasks/safety.py:498`。
- 建议：溢出计数并告警。

#### F-BUG-14 【medium】底盘看门狗只清速度不清方向
- 位置：`dzactuator.cpp:1367-1373`。
- 影响：惯性滑行；Safety 层另有越界急停兜底。

### 8.4 重复代码 / 平行数组 / 常量多真源

| ID | 严重级 | 项 | 位置 |
|---|---|---|---|
| F-DUP-01 | high | auto_start 双真源矛盾（true vs false） | robot.yaml:336 vs :703 |
| F-DUP-02 | high | auto_start_delay 双真源矛盾（45.0 vs 10.0） | robot.yaml:337 vs :704 |
| F-DUP-03 | high | lost_frames_to_rescan 三源（C++60 / dzrknn180 / dzbrain60） | dzrknn_dataprocess.cpp:1335; robot.yaml:236,848 |
| F-DUP-04 | high | scan_pitch_offset_deg 三源（SCHEMA+6.0 / 两段+10.0） | config.py:383; robot.yaml:50,849 |
| F-DUP-05 | high | target_speed_mps 四方（0.04 / 4cm/s / 1m/s / 0.3m/s） | robot.yaml:836; TEAM_STATUS; 决策提交 |
| F-DUP-06 | medium | shoot_distance_m/tol（SCHEMA0.80/0.10 vs YAML0.90/0.12 vs planner兜底0.90/0.12） | config.py:315-318; robot.yaml:687-688 |
| F-DUP-07 | medium | cruise_speed_mps 注释说 0.40/0.15，值 0.35 | robot.yaml:625 |
| F-DUP-08 | medium | lookahead_min/max（SCHEMA0.18/0.45 vs YAML0.36/0.40） | config.py:73-74; robot.yaml:621-622 |
| F-DUP-09 | medium | pivot_penalty（SCHEMA0.0 vs YAML5.0） | config.py:251; robot.yaml:789 |
| F-DUP-10 | medium | max_shots_per_target（SCHEMA3 vs YAML10） | config.py:377; robot.yaml ~:839 |
| F-DUP-11 | medium | lead_time_s（SCHEMA0.0 vs YAML0.27） | config.py:370; robot.yaml ~:833 |
| F-DUP-12 | medium | use_external_fire_gate（C++ false vs YAML true） | dzrknn_dataprocess.cpp:1386; robot.yaml ~:244 |
| F-DUP-13 | medium | ticksPerMeter/2PI（C++1000/3200 vs YAML3280/2680） | dzactuator.cpp:1127-1128; robot.yaml:932,941 |
| F-DUP-14 | medium | CARL0.15 vs wheelbase_m0.157 | dzactuator.h:68; config.py:65 |
| F-DUP-15 | medium | 物理常量 4 处（0.157/0.035/0.1275/0.02） | physics.py / config.py / planner.py / sim.py |
| F-DUP-16 | low | MIN_SEGMENT_M 与 _SEG_STICKY_M 同值两处 | validate.py:52; sim.py:626 |
| F-DUP-17 | medium | legacy 核与新核双轨（LEGACY_PATH） | robot.yaml:321-332; car_localization/ vs dzbrain/ |
| F-DUP-18 | medium | 三套地图口径（第六/第五/第七张图） | config/map.yaml; config/map_v3.yaml; docs/20_资料索引库/历史/03:36 |

### 8.5 测试覆盖

| ID | 严重级 | 项 | 位置 |
|---|---|---|---|
| F-TEST-01 | medium | dzactuator / dzrknn_dataprocess / dzjudgment / rknn_pt 零单测 | find 结果 |
| F-TEST-02 | low | 断言强度未机器校验（无 pytest，靠脚本自报 N/M） | _run_full_regression.py |
| F-TEST-03 | low | pytest 未安装，无法用标准收集 | python -m pytest → No module named pytest |

### 8.6 文档与代码不一致

| ID | 严重级 | 项 | 位置 |
|---|---|---|---|
| F-DOC-01 | medium | 技术全景_C 记录「内置 MISSION_BLOCKS 20 点 vs 配置 68 token」双轨 | docs/00_项目总览/技术全景_C_车端硬件与配置.md:231 |
| F-DOC-02 | medium | 项目总览/TEAM_STATUS 写 HEAD 0e86568，实际 dzrobot_v2 HEAD 11d9999 | docs/00_项目总览/项目总览.md; dzrobot_v2 git |
| F-DOC-03 | low | 时间线/历史文档混用第五/第六/第七张图口径 | docs/20_资料索引库/历史/03:36; docs/00_项目总览/时间线.md:50 |

### 8.7 规划器在途改动相关

| ID | 严重级 | 项 | 位置 |
|---|---|---|---|
| F-PLAN-01 | medium | allow_no_stop 使「缺停站」从硬 error 降级为可放行 | planner.py:1721-1724（diff） |
| F-PLAN-02 | low | _insert_shoot 起点/终点 continue 后 detour 赋值顺序可读性 | planner.py:3388-3394（diff） |

### 8.8 统计表

| 严重级 | 数量 |
|---|---|
| blocker | 2 |
| high | 10 |
| medium | 28 |
| low | 10 |
| **合计** | **50** |

| 类别 | blocker | high | medium | low | 小计 |
|---|---|---|---|---|---|
| SEC 安全 | 2 | 3 | 2 | 1 | 8 |
| FRZ 冻结区 | 0 | 1 | 1 | 0 | 2 |
| BUG 逻辑缺陷 | 0 | 1 | 9 | 4 | 14 |
| DUP 重复/多真源 | 0 | 5 | 12 | 1 | 18 |
| TEST 测试 | 0 | 0 | 1 | 2 | 3 |
| DOC 文档 | 0 | 0 | 2 | 1 | 3 |
| PLAN 规划 | 0 | 0 | 1 | 1 | 2 |
| **合计** | **2** | **10** | **28** | **10** | **50** |

### 8.9 处置优先级建议

1. **P0（立即）**：F-SEC-01/02/03/04/05 —— 轮换口令、回收机端 key、移除明文、改主机校验策略。
2. **P1（本轮收口前）**：F-FRZ-01（清理失效登记）、F-DUP-01/02（auto_start 双真源二选一）、F-BUG-01（Config.get 默认分支）。
3. **P2（下一迭代）**：F-DUP-03/04/05（三源/四方收敛到单一真源）、F-BUG-04/05（除零与默认值）、F-TEST-01（C++ 单测）。
4. **P3（低优先）**：F-BUG-08~13、F-DOC-03、F-PLAN-02 等可读性/文档项。

---

> 本报告为只读审计产物，未修改任何源码/配置/测试。所有行号均为 2026-10-23 工作树实测；工作树后续变动会使部分行号偏移。
### 2.10 main.py（2264 行）★装配 / 发车 / 看门狗

- 顶层函数（rg 实测）：
  - `install_stop_signals` :85、`ros_context_alive` :114、`default_config_path` :130、`load_and_check` :142、`load_field_layout` :214、`plan_route_from_field_layout` :231、`plan_startup_route` :318、`_step_cells` :362、`normalize_step_layout` :387、`load_step_layout` :455、`_step_mode_enabled` :466、`all_map_cells` :471、`_segment_map_cfg` :477、`classify_segment_target` :497、`plan_segment_from_layout` :519、`_find_task` :579、`StepLoop` class :587、`_run_step_loop` :1238、`_layout_mode_enabled` :1250、`_start_layout_listener` :1263、`_ros_diagnosis` :1294、`_contract_skip_requested` :1365、`_endpoint_counts` :1370、`_subscription_topics_of` :1402、`startup_contract` :1419、`_format_contract_table` :1469、`_log_subscriptions` :1486、`_startup_self_check` :1501、`_duplicate_dzbrain_instances` :1527、`_wait_for_pose_ready` :1560、`_wait_for_field_layout` :1617、`build_tasks` :1667、`start_banner` :1703、`_tele_field` :1745、`finish_report` :1754、`run_bench` :1774、`run_plan_only` :1933、`_live_loop` :1973、`run_live` :2003、`main` :2187。
- 关键常量：`STEP_PHASE_SAFE_STOP = "SAFE_STOP"`（:350）、`STEP_PHASE_ROS_LOST = "ROS_LOST"`（:354）、连续 3 拍落入未知格 ⇒ SAFE_STOP（:357）。
- `StepLoop`（:587-1237）：逐段规划/执行/回报；`_stop_done_replay("ROS_LOST")`（:1128）、`_stop_done_replay("SAFE_STOP")`（:1146）；`_phase` 迁移（:1137,1155）。
- `_wait_for_pose_ready`（:1560）：位姿就绪门，超时 exit 5。
- `_startup_self_check`（:1501）：env + 订阅清单 + 关键话题计数；False ⇒ 不出发（exit 5）。
- `startup_contract`（:1419）：启动契约表（:1325 注释给出 5 种情形映射）。
- **发现 F-MAIN-01（medium）· bench 与 live 对 auto_start 的覆盖方向相反**
  - 位置：`main.py:1799`（`cfg.override("mission.auto_start", True).override("mission.auto_start_delay_s", 0.3)`，bench 立即出发）vs `main.py:2082`（`cfg.override("mission.auto_start", False)`，StepLoop 路径强制关闭）。
  - 影响：同一份配置在 bench/live 下 auto_start 语义相反；结合 F-DUP-01（YAML 双真源），排障时极易误判。
  - 建议：在 YAML 中消除双真源，bench/live 的差异只由代码显式控制。
- **发现 F-MAIN-02（low）· run_bench 覆盖 auto_start 绕过启动横幅核对**
  - 位置：`main.py:1798-1799` 注释「bench 专用覆盖：立即出发（否则要等 auto_start_delay）」。
  - 影响：bench 绿不代表 live 的 auto_start_delay 行为正确。

### 2.11 mission.py（4007 行）★任务状态机

- 顶层：`BridgeIntel` class :305、`retag_block_token` :344、`_token_block` :354、`rewrite_crossing_tokens` :363、`resolve_plan_engine` :421、`plan_budget_seconds` :439、`GateResult` :465、`route_check_script` :502 等。
- `resolve_plan_engine`（:421）：解析 `planner.engine`（tour / tracker-safe）。
- `plan_budget_seconds`（:439）：按引擎取时间预算（tour 2.0 s / tracker-safe 10 s）。
- `rewrite_crossing_tokens`（:363）：迟到情报改道时重写穿洞 token。
- 停站三段式与到站级死线见 §2.3.3。
- **发现 F-MIS-01（medium）· mission.py 的 route_check_script 依赖外部脚本**
  - 位置：`tasks/mission.py:502`（`route_check_script`）。
  - 影响：若车端缺少该脚本，门禁校验静默跳过或报错；未核实具体降级行为。

### 2.12 tracker.py / aim.py / localization.py

- `tracker.py`：`PivotPhase` class :55、`TrackerTask` class :120。R1 目标：发布限速统一钳位（ALIGN/PIVOT 绕过 P_MOTION_LIMIT）。
- `aim.py`（1368 行）：云台瞄准/开火判据；消费 `aim.*` 键。
- `localization.py`：`_cfg_float` :121、`PoseSource` Protocol :145、`LocalizationTask` :155。
- **发现 F-LOC-01（medium）· localization 的 _cfg_float 用 or fallback 兜底**
  - 位置：`tasks/localization.py:121`。
  - 问题：`_cfg_float(cfg, key, fallback)` 与 `Config.get` 的 F-BUG-01 叠加：缺键时用调用方 fallback，可能偏离 SCHEMA 默认。
  - 建议：统一走 `cfg.get(key)` 的 SCHEMA 兜底，或显式登记。

### 2.13 pursuit/ 与 route/parser.py、io/ros_io.py

- `controller.py`：`PursuitCommand` :41、`motion_limit_speed_limit` :54、`speed_profile` :85、`_dist_to_stop` :165、`_corner_dist` :184、`dynamic_gear_speed` :191、`compute_command` :219、`brake_command` :318。
- `path.py`：`Projection` :50、`Path` :74（`point_at` :152、`heading_at` :169、`project` :246、`corner_turn_deg` :228 等）。
- `parser.py`：`normalize_token` :96、`parse_token` :183、`parse_mission` :306、`detect_crossing` :409、`parse_reset` :441、`format_route` :461。
- `ros_io.py`：`required_subscription_topics` :36、`watched_topic_names` :46、`ros_env` :50、`env_problems` :67、`RosStats` :149、`RosBridge` :170。
- **发现 F-CTL-01（medium）· speed_profile 的五层顺序只在注释里**
  - 位置：`pursuit/controller.py:85` 与 robot.yaml:630 注释。
  - 影响：层序变更无测试锚定（除 test_motion_limit 部分）。
  - 建议：把层序写成模块常量或断言。

### 2.14 dzsmartscout 顶层脚本逐个定性

| 脚本 | 行数 | 定性 |
|---|---|---|
| setup_drone_access.py | 68 | 一次性装机：改 nsswitch + 写 root/duzhong authorized_keys（F-SEC-02） |
| install_drone_key.py | 47 | ssh-copy-id 等价；AutoAddPolicy（F-SEC-05） |
| fix_mdns_nsswitch.py | 69 | 修 .local 解析；sudo -S（F-SEC-04） |
| fix_drone_avahi.py | 49 | 修 avahi；sudo -S |
| run_fix_drone_avahi.py | 37 | 远程执行上条 |
| rename_drone_hostname.py | 57 | 改主机名；sudo -S |
| enable_mdns.py | 68 | 开 mDNS；root/duzhong |
| probe_drone_vnc.py | 32 | 探测 VNC/authorized_keys |
| probe_drone_dirs.py | 25 | 探测目录 |
| verify_new_ip.py | 44 | 验证新 IP |
| ssh_connect.py | 75 | 多用户名口令探测 |
| test_drone_users.py | — | 探测可登录用户 |
| find_drone.py | — | mDNS 发现 |
| crack_vnc_passwd.py | 53 | VNC 口令读取/爆破（F-SEC-03） |
| vnc_handshake.ps1 | 71 | VNC RFB 握手（F-SEC-08） |
| drone-scripts/start-lidar*.sh | — | 拉网口；sudo -S（F-SEC-04） |

## §9 综合风险与建议

### 9.1 三大结构性风险

1. **凭据外泄（blocker）**：F-SEC-01~05。根因是「脚本即文档」的开发习惯 + 无秘密扫描 + 远端已推送。
2. **配置多真源（high）**：F-DUP-01~18。根因是 legacy/新核双轨（F-DUP-17）+ SCHEMA 只登记不强制 + `Config.get` 默认分支缺陷（F-BUG-01）。
3. **冻结区在途改动边界模糊（medium）**：F-FRZ-01/02。登记表与 git 状态不同步。

### 9.2 建议（按轮次）

| 轮次 | 动作 | 对应发现 |
|---|---|---|
| 立即 | 轮换该口令；回收机端 authorized_keys；移除明文脚本或改环境变量；加秘密扫描 | F-SEC-01~05 |
| 收口前 | 清理 tracker.py 失效登记；auto_start 双真源二选一；修 Config.get 默认分支 | F-FRZ-01、F-DUP-01/02、F-BUG-01 |
| 下一迭代 | lost_frames/scan_pitch/target_speed 收敛单真源；dzactuator 除零守护；C++ 单测 | F-DUP-03/04/05、F-BUG-04、F-TEST-01 |
| 长期 | legacy 核退役；地图口径统一；断言强度门禁 | F-DUP-17/18、F-TEST-02 |

### 9.3 审计边界声明

- 本次为**只读**审计：未修改任何源码、配置、测试、文档（除本报告）。
- 行号基于 2026-10-23 工作树；后续改动会使行号偏移。
- 未核实项见 §7；标注「〔档〕」者为文档口径。
## §10 附录：取证命令清单（可复跑）

> 全部为只读命令；在 dzrobot_v2 下执行者标注 `[dzrobot_v2]`，在工作区根执行者标注 `[root]`。

| # | 命令 | 用途 |
|---|---|---|
| 1 | `rg -n --no-heading -e "authorized_keys" -e "sudo -S" -e "password" ... dzrobot_v2 dzsmartscout` | 凭据扫描 |
| 2 | `rg -n "<口令字面量>" dzsmartscout --glob "*.py" --glob "*.sh" --glob "*.ps1"` | 明文口令定位 |
| 3 | `git -C v1 ls-files` + 逐文件 rg | 根仓跟踪的凭据脚本 |
| 4 | `git -C v1 cat-file -e origin/main:dzsmartscout/setup_drone_access.py` | 远端是否含凭据脚本 |
| 5 | `git -C v1 remote -v` / `git -C dzrobot_v2 remote -v` | 远端清单 |
| 6 | `[dzrobot_v2] cat tools/wf/frozen.json` | 冻结区登记 |
| 7 | `[dzrobot_v2] git status --porcelain=v1 -- src/dzbrain/dzbrain src/dzrknn_dataprocess config tools/deploy_to_car.py` | 冻结区脏文件 |
| 8 | `[dzrobot_v2] git diff -- config/robot.yaml src/dzbrain/dzbrain/route/planner.py src/dzbrain/dzbrain/tools/sim_cli.py` | 在途改动内容 |
| 9 | `[dzrobot_v2] PYTHONDONTWRITEBYTECODE=1 python tools/wf/check.py` | 收口门禁 |
| 10 | `[dzrobot_v2] python -m pytest src/dzbrain/tests -q --collect-only` | 测试框架探测（未安装） |
| 11 | `[dzrobot_v2] rg -n "^(def|class|async def) " src/dzbrain/dzbrain --glob "*.py" | sort` | §3 全表来源 |
| 12 | `[dzrobot_v2] rg -c "def test_" src/dzbrain/tests/test_*.py` | 测试计数 |
| 13 | `[dzrobot_v2] find src/dzactuator src/dzrknn_dataprocess src/dzjudgment src/rknn_pt -name "test_*.py"` | 无测试模块 |
| 14 | `[dzrobot_v2] python -c "..."` 正则提取 SCHEMA | §5.1/5.2 来源 |
| 15 | `[dzrobot_v2] awk "/^[A-Za-z_/]/" config/robot.yaml` | robot.yaml 段结构 |
| 16 | `[dzrobot_v2] rg -n "target_speed_mps|thread_num|cruise_speed_mps|pivot_penalty|auto_start|lost_frames_to_rescan|scan_pitch_offset_deg|ticksPerMeter|ticksPer2PI" config/robot.yaml` | 配置漂移取证 |

### 10.1 行号口径

- 本报告所有 `路径:行号` 均为 2026-10-23 工作树实测。
- `dzrobot_v2/` 前缀省略时，路径相对 `dzrobot_v2/`；`dzsmartscout/`、`docs/` 前缀表示相对工作区根。

### 10.2 与既有文档的关系

- 本报告为**代码级审计**，与 `docs/00_项目总览/技术全景_A/B/C/D/E` 互补：A=架构，B=？（未读），C=车端硬件与配置，D/E=？。本报告聚焦「发现清单 + 证据」，不重复架构描述。
- 与 `docs/20_资料索引库/赛事与治理/05`（一级风险）口径一致：凭据、规划求解率、回归红为三大风险。

---

**报告结束** · auditor · 2026-10-23
