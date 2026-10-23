---
来源: 本次代码级技术审计（2026-10-23，子智能体）
取证: 实
更新: 2026-10-23
status: 现行
---

# 技术全景 A：车端核心 dzbrain（main / core / io / route / pursuit / tools.sim）

> 面向「全球校园人工智能算法精英大赛·空地协同侦排挑战赛」车端决策核心的代码级技术全景。
> 本文只读代码写成，不修改任何源码或配置；所有结论均附 `路径:行号` 或测试名作为证据。
> 凡未能在本轮代码中直接核实的行为，一律标注「未核实」，绝不凭记忆编造。

---

## §0 导读

### 0.1 审计范围与不变量

本文覆盖 `dzrobot_v2/src/dzbrain/dzbrain/` 下六个包共 21 个源文件（不含 `tasks/` 与 `scout/`）：

| 包 | 文件数 | 总行数 | 职责一句话 |
|---|---|---|---|
| `main.py` | 1 | 2265 | 单进程入口：参数解析、启动门禁、任务装配、步进循环、退出码 |
| `core/` | 7 | 1886 | 线程安全共享状态、配置中心、事件总线、几何真源、日志、跨任务状态类型、任务线程基类 |
| `io/` | 3 | 1299 | ROS 契约（话题/QoS/单位/解码）、裁判帧解析、rclpy 接线 |
| `route/` | 5 | 6984 | 路线数据模型、token 解析、出发前校验、物理几何、A* 规划器（含 tracker-safe 引擎） |
| `pursuit/` | 3 | 884 | 折线投影与弧长、纯追踪舵角与速度剖面、前视点 |
| `tools/` | 2 | 2600 | 离线纯追踪仿真器与命令行入口 |

**全仓唯一真源**（审计中确认，改值必须只改一处）：

1. **坐标真源** = `core/geometry.py`（场地 14 列 A~N × 11 行、格边 0.35 m、米制原点在 N11 角、格单位 = 米/0.35、原始 yaw → 地图 yaw 偏移 +π/2）。`geometry.py:27-48` 定义常量，`geometry.py:54-191` 定义换算。
2. **隔板真源** = `geometry.py:37-43` 的 `BARRIER_ROW_HI=6 / BARRIER_ROW_LO=7 / BARRIER_COL_FROM="D"`，注释明确要求 `route/validate.py`、`route/planner.py`、`tools/*` 一律从此处取值。
3. **物理尺寸真源** = `config/robot.yaml` 的 `vehicle.*` / `field.*` / `planner.obstacle_margin_m`；**公式真源** = `route/physics.py`。`config.py:188-223` 的注释把这段分层写得很清楚：YAML 存数、`physics.py` 存公式、其余模块只调用。
4. **配置键真源** = `core/config.py` 的 `SCHEMA`（本轮实数 **205 个键**，`config.py:63-561`）；未登记键会被告警而不是静默忽略。

### 0.2 阅读路径建议

| 想了解什么 | 直接看 |
|---|---|
| 车怎么跑起来、退出码怎么分 | §2.1 `main.py`、§2.8 线程模型 |
| 坐标与单位会不会搞错 | §2.2 `core/geometry.py`、§4.1 |
| 配置怎么校验、旧键怎么迁移 | §2.3 `core/config.py`、§5 |
| ROS 话题/单位契约 | §2.4 `io/contract.py`、§2.5 `io/ros_io.py` |
| 路线 token 语法 | §2.6 `route/parser.py` |
| 出发前校验规则 R001~R014 | §2.7 `route/validate.py` |
| 规划器状态机、A* 与 tracker-safe 双引擎 | §2.8 `route/planner.py` |
| 纯追踪公式、PIVOT/倒车判定 | §2.9 `pursuit/` |
| 离线仿真怎么跑、指标怎么读 | §2.10 `tools/sim.py`、§2.11 `tools/sim_cli.py` |
| 逐函数索引（穷尽） | §3 |
| 数据流 / 线程 / 锁 | §4 |
| 全部配置键汇总 | §5 |
| 测试证据与当前红项 | §6 |
| 未核实清单与发现的坑 | §7 |

### 0.3 本轮实测要点（先给结论）

1. **代码本身可读、可跑、且核心单元测试大面积通过**：`test_geometry 38/38`、`test_physics 103/103`、`test_core 56/56`、`test_contract 56/56`、`test_referee 81/81`、`test_pursuit 122/122`、`test_pivot 55/55`（运行环境 `Python 3.12.10`，见 §6.1）。
2. **存在多条「测试期望值已过期 / 与当前配置不一致」的红项**，主要集中于 `test_sim.py`、`test_sim_guard.py`、`test_planner.py`、`test_motion_limit.py`、`test_route.py`（§6.2）。这些**不是**源码逻辑崩溃，而是断言里写死的「44 点 / 15.05 m / cruise 0.30 / ld_max 0.35 / ticksPerMeter 3280」与 `config/robot.yaml` 现役值（68 点 / 24.85 m / cruise 0.35 / ld_max 0.40 / ticksPerMeter 3280）不同步。
3. **`planner.py` 里 `_door_turn_margin_m` 被定义了两次**（`planner.py:614` 与 `planner.py:695`），后一次覆盖前一次，`614` 版本是死代码（§7.1）。
4. **多组配置键被代码读取却未登记进 `SCHEMA`**，导致 `validate()` 抓不到拼写错误、也不会做类型/范围校验：`planner.barrier_turn_penalty`、`planner.barrier_turn_margin_m`、`planner.door_turn_penalty`、`planner.edge_mine_turn_penalty`、`planner.obstacle_turn_penalty`、`planner.bridge_approach_align_penalty_m`、`step.layout_ttl_s`、`step.done_replay_s`（§7.2）。
5. **`plan_route_tracker_safe` 在当前配置/地图下会直接 `PlannerError`**：`test_incremental_plan.py:61` 运行时抛出「tracker-safe 引擎无解」（§6.2 / §7.3）。这说明增量续算入口在当前 `config/robot.yaml` + `map.yaml` 组合下不可用。

---

## §1 模块地图

### 1.1 包/模块依赖图

~~~text
                         ┌────────────────────────────┐
                         │       main.py              │  入口 / 装配 / 步进循环
                         └───────┬──────────┬─────────┘
                                 │          │
              ┌──────────────────┼──────────┼──────────────────┐
              ▼                  ▼          ▼                  ▼
     ┌────────────────┐  ┌───────────────┐  ┌─────────┐  ┌────────────────┐
     │ core/config    │  │ core/task     │  │ io/*    │  │ route/planner  │
     │ (SCHEMA 205)   │  │ (TaskThread)  │  │ contract│  │ (A*/tracker)   │
     └───────┬────────┘  └───────┬───────┘  │ referee │  └───────┬────────┘
             │                   │         │ ros_io  │            │
             ▼                   ▼         └─────────┘            ▼
     ┌────────────────┐   ┌───────────────┐           ┌────────────────────┐
     │ core/geometry  │   │ core/blackboard│          │ route/{model,parser,│
     │ (坐标真源)      │   │ core/events   │          │  validate,physics}│
     └───────┬────────┘   └───────┬───────┘           └─────────┬──────────┘
             │                    │                            │
             ▼                    ▼                            ▼
     ┌────────────────┐   ┌───────────────┐           ┌────────────────────┐
     │ core/state     │   │ core/logging  │           │ pursuit/{path,     │
     │ (跨任务状态)    │   │ (横幅/限流)   │           │ controller,lookahead}│
     └────────────────┘   └───────────────┘           └────────┬───────────┘
                                                              │
                                                              ▼
                                                      tools/sim.py  (离线仿真)
                                                      tools/sim_cli.py (CLI)
~~~

**依赖方向铁律（从 docstring 与 import 实测）：**

- `io/contract.py` **只 import `core/geometry`**（`contract.py:19`），不 import ROS —— 纯数据层，可本地单测。
- `io/ros_io.py` 是 `main.py` 之外唯一 import `rclpy` 的地方之一，且 `rclpy` **惰性导入**（`ros_io.py:8-9`、`ros_io.py:173`）。
- `route/*` **只 import `core/geometry`、`core/config`、`route.model/parser/physics/validate`**，不 import ROS（`planner.py:79-91`）。
- `pursuit/*` 只 import `core/config`、`core/blackboard` 与同包模块（`controller.py:29-32`）。
- `tools/sim.py` 不 import ROS、不读文件（`run_sim` 是纯函数，`sim.py:61`），只 import `core/*`、`pursuit/*`、`route.model`、`route.physics`（`sim.py:93-105`）。
- `main.py` 通过**函数内延迟 import** 装配 `tasks/*`（`main.py:1679`），避免装配前加载无人机链（`main.py:210` 的注释）。

### 1.2 每个文件的对外契约（API 边界）

| 文件 | 主要公开 API | 被谁调用 | 是否有副作用 |
|---|---|---|---|
| `core/geometry.py` | block_to_index / index_to_block / block_to_meters / meters_to_block / block_to_units / units_to_meters / meters_to_units / map_bounds / in_bounds / normalize_angle / ang_diff / yaw_to_quaternion / quaternion_to_yaw / heading_token_to_yaw / heading_of + `Pose` | 全仓 | 纯函数（无副作用） |
| `core/config.py` | `Config.load/from_dict/get/require/section/keys/override/validate/summary` + `Spec.check` + `lookahead_distance/curve_speed_limit/pivot_dist_eff` | 全仓 | `load/from_dict` 读文件并迁移旧键；`override` 就地改 |
| `core/blackboard.py` | `Blackboard.set/update/get/entry/age/get_fresh/is_fresh/snapshot/describe` | 任务线程 | 写键 + 计数 `writes` |
| `core/events.py` | `EventBus.subscribe/publish/stats` + `Subscription.poll/drain/unsubscribe` | 任务线程 | 发布非阻塞；队列满丢事件并计数 |
| `core/task.py` | `TaskThread`（`setup/tick/teardown/fault_hook/run/stop`）、`TaskGroup`（`add/start_all/stop_all/health/faulty/describe`） | `main.py`、`build_tasks` | 起/停线程、`_fail` 发布 `EV_TASK_FAULT` |
| `core/state.py` | `SafetyState/StopPointState/TargetState/AimState/DroneState/FixState/Telemetry` | 各任务 | 纯 frozen dataclass（无副作用） |
| `core/logging.py` | `banner/kv_lines/Throttle/FaultGuard` | 各处 | 仅打印/计数 |
| `io/contract.py` | `TOPICS`、`topic()`、`odom_to_pose`、`pose_to_pursuit_twist`、`clamp_steer_for_chassis`、`chassis_speed_counts`、`decode_*` | `ros_io`、任务 | 纯函数 |
| `io/referee.py` | `RefereeTracker.feed`、`RefereeFrame/RefereeDelta` | `ros_io.on_referee_hp` | 写内部计数 |
| `io/ros_io.py` | `RosBridge`（订阅/发布/读缓存/spin/shutdown/topic_counts/subscription_topics/summary） | `main.py` | 建/毁 ROS 节点、发指令 |
| `route/model.py` | `Waypoint/Route/RouteBuilder/make_route` | `parser`、`planner`、`pursuit` | 不可变对象 |
| `route/parser.py` | `normalize_token/parse_token/parse_mission/parse_reset/format_route/detect_crossing` | `main`、`planner`、`validate`、`sim` | 纯函数 |
| `route/validate.py` | `validate_route`、`Issue`、`summary`、`crossing_entries` | `main.load_and_check`、`planner`、`route_check` | 纯函数（只报不改） |
| `route/physics.py` | 几何判据纯函数族（见 §2.7） | `planner`、`sim`、`route_check` | 纯函数 |
| `route/planner.py` | `plan_route`、`plan_route_tracker_safe`、`PlanResult`、`MapSpec`、`Grid`、`check_route_structure`、`assert_legal` | `main`、`tasks/mission` | 纯计算；抛 `PlannerError` |
| `pursuit/path.py` | `Path`（`project/point_at/corner_*`）、`Projection` | `controller`、`sim` | 纯函数 |
| `pursuit/controller.py` | `compute_command/speed_profile/brake_command/motion_limit_speed_limit/dynamic_gear_speed` | `tasks/tracker`、`sim` | 纯函数（读黑板） |
| `pursuit/lookahead.py` | `find_lookahead_point` | `controller` | 纯函数 |
| `tools/sim.py` | `run_sim`、`SimConfig/SimResult/SegmentStat`、`write_csv/write_segments_csv/write_svg/format_metrics/format_segment_table` | `sim_cli`、测试 | `run_sim` 纯；`write_*` 落盘 |
| `tools/sim_cli.py` | `build_parser`、`main` | CLI | 写 `traj.csv/traj.svg/by_segment.csv` |
| `main.py` | `main`、`run_live`、`run_bench`、`run_plan_only`、`load_and_check`、`StepLoop` | CLI | 起停线程、发指令、退出码 |


---

## §2 逐模块详解

### §2.1 `dzbrain/main.py` —— 单进程 8 线程启动 / 参数解析 / 主循环 / shutdown

**定位**：车端唯一入口。一个进程、一个 ROS 节点（`dzbrain`）、一个 `MultiThreadedExecutor`（`main.py:5-7`、`main.py:198-200`）；每个任务一个 `threading.Thread`，固定频率、独立状态机、独立失效兜底（`core/task.py:1-15`）。

#### 2.1.1 模块级常量与信号接管

| 名称 | 行 | 值/说明 |
|---|---|---|
| `LOG = print` | `main.py:76` | 全模块统一日志入口 |
| `_STOP_SIGNALS = ("SIGINT","SIGTERM")` | `main.py:82` | 显式接管信号（见下） |
| `P_FIELD_LAYOUT = "field_layout"` | `main.py:211` | 无人机下发布局的黑盒键，与 `tasks.drone.P_FIELD_LAYOUT` 同值（注释：不 import 任务模块避免装配前加载无人机链） |
| `P_CAR_STATE = "car_state"` | `main.py:341` | 车→机回报键 |
| `STEP_PHASE_*` | `main.py:344-354` | 步进循环 8 个阶段常量（WAIT_LAYOUT / PLANNING / DRIVING / WAIT_INFO / SEGMENT_DONE / FINISH / SAFE_STOP / WAIT_INFO_TIMEOUT / ROS_LOST） |
| `_STEP_UNKNOWN_STOP_TICKS = 3` | `main.py:359` | belief 连续 3 拍（10 Hz ⇒ 0.3 s）落入未知格 ⇒ SAFE_STOP |
| `_CONTRACT_ROWS` | `main.py:1330-1351` | 18 行启动契约：(逻辑键, 方向, 级别, 需 pub≥1, 需 sub≥1, 需本节点建订阅) |
| `_CONTRACT_BLOCKING` | `main.py:1354-1359` | 模式→阻断级别映射：live=required；race-step=required+race；prepare=全量；bench=空 |
| `CONTRACT_SKIP_ENV = "DZ_SKIP_CONTRACT"` | `main.py:1362` | 环境变量跳过契约（bench/离线） |
| `_ROS_DIAG_COMMANDS` | `main.py:1287-1291` | 位姿门/看门狗失败时原样打印的 3 条现场排查命令 |

**`install_stop_signals(stop_flag, log=LOG) -> list[str]`**（`main.py:85-111`）
- 参数：`stop_flag` = dict，须含 `stop` 与 `signal` 键。
- 返回：实际装上的信号名列表（测试断言用）。
- 副作用：仅 `signal.signal(...)` 注册处理器；**处理器内只写字典**（不 print、不拿锁、不碰 ROS）—— 信号上下文里 print 的缓冲锁可能被主线程持有会死锁（`main.py:100-105` 注释，源自 2026-10-11 收栈「假崩溃」事故）。
- 调用方：`run_live`（`main.py:2013`）、`run_bench`（`main.py:1838`）。

**`ros_context_alive() -> bool`**（`main.py:114-127`）
- 无 rclpy 环境（bench/单测）恒判 True；有 rclpy 则返回 `rclpy.ok()`；判活本身出错也返回 True（不误杀正常流程）。主循环每拍调用。

**`default_config_path() -> str`**（`main.py:130-136`）
- 优先级：车上 `~/dzrobot/config/robot.yaml`（存在则用）→ 工作区 `<ws>/config/robot.yaml`（__file__ 上溯 3 层）。

#### 2.1.2 启动门禁 `load_and_check`

**签名**：`load_and_check(config_path, map_path, *, force=False, announce=True, allow_missing_map=False) -> (Config, dict, list[str])`（`main.py:142-204`）

- 参数：`config_path` 配置路径（None→默认）；`map_path` 地图路径；`force` 跳过门禁报错；`announce` 是否在函数内打印错误（`--plan-only` 下避免打印两遍）；`allow_missing_map` 车端布局模式：地图缺失不算错误。
- 返回：`(cfg, map_cfg, errors)`。
- **副作用**：`Config.load()` + `cfg.validate(strict=False)`；若地图存在则 `parse_mission(cfg.get("route.tokens"))` 并 `validate_route(...)`；收集 error 级 `Issue` 到 `errors`。
- 关键语义（`main.py:177-183`）：车端「不留地图文件」—— 地图文件不存在且 `planner.auto_plan_enabled=true` 时**允许启动**，`map_cfg` 保持空，绝不猜默认地图；否则报「地图文件不存在」。
- 被调用方：`main()`（`main.py:2203`）。

#### 2.1.3 布局驱动规划 `plan_route_from_field_layout` / `plan_startup_route`

**`plan_route_from_field_layout(cfg, layout, *, log=None, cross_lower=None, cross_upper=None)`**（`main.py:231-315`）
- 参数：`layout` = 无人机归一化布局 dict；`cross_lower/upper` = 指定穿洞格。
- 返回：`PlanOutcome`（不抛异常；失败 → ok=False + error）。
- 副作用：
  1. `map_cfg = {"map": dict(layout)}`（不读文件，`main.py:251`）；
  2. `resolve_plan_engine(cfg, log)` 选引擎；`plan_budget_seconds` 取预算（失败兜底 2.0）；
  3. **先 `PL.load_map(map_cfg)` 提前校验坏布局**（`main.py:268`）；
  4. `engine == "tracker-safe"` → `PL.plan_route_tracker_safe(...)`；否则 → `PL.plan_route(..., allow_reverse=cfg.get("planner.allow_reverse", False), pivot_penalty=...)`（`main.py:270-277`）；
  5. token 空 ⇒ ValueError；随后 `PL.ensure_second_shoot_stop(...)` 补第二射击停站（`main.py:281-282`）；
  6. tracker-safe 引擎额外跑 `check_planned_route(...)` 双门禁，不过 ⇒ ValueError（`main.py:285-289`）；
  7. `footprint_diagnostic(...)` 做参考点偏移足迹诊断（`main.py:290-291`）；
  8. **失败绝不回退地图文件或旧 `route.tokens`**（`main.py:308-315` 注释）。
- 被调用方：`plan_startup_route`、`run_live`（`main.py:2098`）。

**`plan_startup_route(cfg, bb, *, log=None)`**（`main.py:318-330`）
- 只认内存布局；`load_field_layout(bb)` 为 None ⇒ 返回 None（不规划、不读文件、不用 route.tokens）。

#### 2.1.4 步进布局归一化 `normalize_step_layout`

**签名**：`normalize_step_layout(payload) -> (dict|None, str)`（`main.py:387-452`）
- 在 `tasks.drone.normalize_field_layout()`（v1 字段）之上补 v2 三字段：`known`（已观测格）、`target_cell`（本段目标）、`segment_id`（段序号）。
- 校验规则：
  - `known` 只许合法格名（`_step_cells`，`main.py:362-384`：去重、大写、`geo.block_to_index` 校验）；
  - 特征格（mines/obstacles/targets）**必须 ⊆ known**（`main.py:420-424`）；
  - `target_cell` 非空须为合法格名；`segment_id` 须为整数；
  - 缺 `known` 但有 `target_cell/segment_id` ⇒ 拒收（「半个 v2」，`main.py:412-414`）；
  - 完全缺 `known` 且无 v2 字段 ⇒ 当作 v1 原样返回（保持旧行为逐字节一致，`main.py:409-415`）。
- 另有 `D-022` 透传：`coverage` / `coverage_complete` 原样保留（`main.py:447-451`）。
- 被调用方：`load_step_layout`（`main.py:455-463`）、`_wait_for_field_layout`（`main.py:1633`）。

#### 2.1.5 段规划 `plan_segment_from_layout`

**签名**：`plan_segment_from_layout(cfg, layout, *, from_cell, target_cell, heading_dir=None, log=None)`（`main.py:519-576`）
- 返回：`SegmentPlan`（永不抛异常）。
- 关键逻辑：
  1. `known` 空 ⇒ 失败；`target not in known_set` ⇒ 失败（车绝不进未知格）；
  2. **未知格只对起点破例**：`unknown = [all_map_cells() - known - {src}]`（`main.py:549`）；
  3. 调 `tasks.mission.plan_segment_route`，传 `blocked_extra=unknown`（未知格只影响通行性，不当物理障碍）；
  4. 双保险：规划结果里出现 unknown ⇒ 拒收（`main.py:555-558`）；
  5. **终点优先**：`plan.target_cell == layout.end` ⇒ `stop_kind = SEG_STOP_END`（`main.py:559-563`）；
  6. 日志打印段规划摘要与段末射击停站提示（`main.py:564-575`）。
- 被调用方：`StepLoop._default_planner`（`main.py:1231-1235`）。

#### 2.1.6 `StepLoop` —— T27 步进循环状态机（核心）

**类定义**：`main.py:587-1235`。状态机（`main.py:590-599`）：

~~~text
WAIT_LAYOUT → PLANNING → DRIVING → SEGMENT_DONE →（新布局）PLANNING
                                        ↘（无新布局）WAIT_INFO →（新布局）PLANNING
SEGMENT_DONE（目标 = end）→ FINISH；规划失败/无目标/无位姿 → WAIT_INFO（不退出）
~~~

三条硬语义（`main.py:594-599`）：
① WAIT_INFO **无超时**（step.wait_timeout_s 缺省 0 = 无限）；② **换段只发生在段边界**（DRIVING 期间新布局只记 version）；③ **布局新鲜度**（step.layout_ttl_s 缺省 10 s，0=关）⇒ 过期不规划。

**构造**：`StepLoop(cfg, bb, bridge, mission, *, bus=None, log=LOG, clock=None, layout_getter=None, planner=None)`（`main.py:602-682`）。关键字段：

| 字段 | 行 | 语义 |
|---|---|---|
| `_wait_timeout_s` | `main.py:620` | step.wait_timeout_s |
| `_odom_watch_s` | `main.py:625` | step.odom_watch_s（缺省 6.0） |
| `_odom_fresh_s` | `main.py:629` | max(1.0, safety.pose_timeout_s) |
| `_layout_ttl_s` | `main.py:637` | step.layout_ttl_s（缺省 10.0） |
| `_layout_rx_at` | `main.py:640` | monotonic：收到当前版本布局的时刻 |
| `_layout_complete_noted` | `main.py:644` | D-022 完整布局只说明一次 |
| `_done_replay_s` | `main.py:651` | D1b：段完成单 1 Hz 重发间隔 |
| `_done_seq / _done_latch / _done_replay_stopped / _last_done_replay_at` | `main.py:654-657` | 完成单闩锁状态 |
| `_phase / _layout / _layout_version / _seen_version / _planned_version` | `main.py:659-663` | 状态机核心 |
| `_target_cell / _segment_id / _stop_kind / _known_count` | `main.py:664-669` | 本段目标 / 段序 / 停站类型 / known 格数 |
| `_segments_started / _segments_done / _replans` | `main.py:674-676` | 段计数 |
| `_unknown_hits / _pose_seeded` | `main.py:680-681` | 未知格连续命中 / 是否已种子位姿 |

**公开方法**：
- `phase`（property，`main.py:687-689`）；`status()` → dict（`main.py:691-716`，含 phase/layout_version/seen_version/planned_version/target_cell/segment_id/stop_kind/known_count/reason/segments_started/segments_done/replans/wait_s/layout_ttl_s/layout_age_s/layout_stale/done_seq/done/done_replay_stopped）。
- `start(layout=None)`（`main.py:721-745`）：吃进首份布局，复位 belief（`_seed_pose`），置 PLANNING；无布局则 WAIT_LAYOUT。
- `tick(now=None) -> str`（`main.py:747-760`）：10 Hz 单拍。`_poll_layout` → 按 phase 分派 `_plan/_drive_tick/_after_segment_done/_wait_tick` → `_publish_car_state`。
- `run(*, stop_flag, alive=ros_context_alive, sleep=time.sleep, poll_s=0.1, finish_wait_s=2.0, log=LOG) -> str`（`main.py:762-790`）：阻塞跑，返回 finish/stop/context_dead/timeout/safe_stop/ros_lost。
- `_plan(now)`（`main.py:866-952`）：核心规划。判 `_target_cell` 空 ⇒ WAIT_INFO；`from_cell == target_cell` ⇒ 同格停站（分类 stop_kind、计数段、直接判段完成）；先验布局新鲜度（过期且不完整 ⇒ WAIT_INFO(layout_stale)）；调 `planner`；`mission.request_segment(...)` 下发；未接收 ⇒ planned_version=-1 + WAIT_INFO(retry)。
- `_drive_tick(now)`（`main.py:954-977`）：先跑 `_odom_watch`；读 `mission.segment_status()`；完成单匹配 ⇒ `_mark_segment_done`；`_unknown_violation()` ⇒ SAFE_STOP；任务线程回 IDLE 超 1 s ⇒ WAIT_INFO。
- `_after_segment_done(now)`（`main.py:978-991`）：到 end ⇒ FINISH；新布局已到 ⇒ PLANNING；否则 WAIT_INFO。
- `_wait_tick(now)`（`main.py:993-1012`）：新布局 ⇒ PLANNING；超时 ⇒ WAIT_INFO_TIMEOUT；5 s 节流日志；`_maybe_replay_done`。
- `_mark_segment_done(now, *, block, elapsed, skipped=False, stop_kind="")`（`main.py:1017-1043`）：递增段计数、闩锁完成单（含 seq/replay）、置 SEGMENT_DONE。
- `_maybe_replay_done(now)`（`main.py:1048-1066`）：WAIT_INFO 期间 1 Hz 重发完成单。
- `_stop_done_replay(why)`（`main.py:1068-1073`）：新布局/新段/FINISH/安全停机时停止重发并清闩锁。
- `_to_wait_info(reason, now, *, retry=False)`（`main.py:1075-1086`）：置 WAIT_INFO + 发刹车 + 5 s 节流日志。
- `_to_finish(now)`（`main.py:1088-1097`）。
- `_odom_watch(now) -> bool`（`main.py:1102-1124`）：DRIVING 中连续 step.odom_watch_s 秒无新 /odom/localization ⇒ ROS_LOST（2026-10-18 现场事故防线③）。
- `_to_ros_lost(now, age)`（`main.py:1126-1143`）：停段 + 刹车 + `_ros_diagnosis` + 阶段 ROS_LOST。
- `_to_safe_stop(reason, now)`（`main.py:1145-1158`）。
- `_publish_car_state(now, *, force=False)`（`main.py:1160-1185`）：写 P_CAR_STATE（含 stop_kind、done 闩锁）；内容未变且非 force ⇒ 跳过。
- `_current_block(*, prefer_prev=False)`（`main.py:1190-1198`）：prefer_prev 时若信念离上一段目标 ≤ 半格则沿用上一段目标。
- `_heading_dir()`（`main.py:1200-1205`）：调 `tasks.mission.heading_dir_from_yaw`。
- `_unknown_violation() -> bool`（`main.py:1207-1218`）：连续 _STEP_UNKNOWN_STOP_TICKS 拍命中未知格。
- `_seed_pose(layout)`（`main.py:1220-1229`）：`mission.reset_pose(cfg.get("mission.reset") or "<start>,x")`，失败仅告警不阻塞。
- `_default_planner(layout, from_cell, target_cell, heading_dir)`（`main.py:1231-1235`）：调 `plan_segment_from_layout`。

#### 2.1.7 启动自检与契约

- `_layout_mode_enabled(cfg, map_path) -> bool`（`main.py:1250-1260`）：`--map` 指向真实文件 ⇒ 文件模式；否则看 `planner.auto_plan_enabled`。
- `_start_layout_listener(bridge, bb, bus, cfg, *, log=LOG)`（`main.py:1263-1270`）：先起独立 `DroneTask` 专等 field_layout。
- `_ros_diagnosis(bridge, *, log=LOG)`（`main.py:1294-1313`）：打印 env 行 + 话题计数 + 最近 odom 年龄 + 3 条命令。
- `_contract_skip_requested() -> bool`（`main.py:1365-1367`）。
- `_endpoint_counts(bridge, key, topic) -> (pubs, subs)`（`main.py:1370-1399`）：优先级 = 真 rclpy 节点计数 → 替身 endpoint_counts → topic_counts()[key]；读不到 ⇒ (-1,-1)（未知≠缺失）。
- `_subscription_topics_of(bridge) -> list[str]|None`（`main.py:1402-1416`）。
- **`startup_contract(bridge, mode="live", *, log=None) -> (ok, rows)`**（`main.py:1419-1466`）：逐行对账；ok = all(row["ok"] for row in rows if row["blocking"])；DZ_SKIP_CONTRACT=1 或 mode=bench ⇒ 跳过。
- `_format_contract_table(rows, *, mode="live", ok=True) -> str`（`main.py:1469-1483`）。
- `_log_subscriptions(bridge, *, log=LOG)`（`main.py:1486-1498`）。
- `_startup_self_check(bridge, *, log=LOG) -> bool`（`main.py:1501-1524`）：env + 订阅清单 + 话题计数；env error ⇒ False（exit 5）。
- `_duplicate_dzbrain_instances() -> list[str]`（`main.py:1527-1557`）：pgrep -af dzbrain 找同机重复实例（仅 Linux；查询失败返回空表）。
- `_wait_for_pose_ready(bridge, cfg, *, stop_flag, alive, log, sleep, now) -> bool`（`main.py:1560-1614`）：step.pose_ready_wait_s（缺省 8 s）窗口内等 /odom/localization 首帧；超时 ⇒ 打印诊断 + False（exit 5）。
- `_wait_for_field_layout(bb, cfg, *, stop_flag, alive, log, sleep, now) -> dict|None`（`main.py:1617-1661`）：drone.field_layout_wait_s（缺省 30 s）等待；返回步进归一化布局。

#### 2.1.8 任务装配

**`build_tasks(bridge, bb, bus, cfg, *, log=LOG, skip=None) -> (TaskGroup, list[str])`**（`main.py:1667-1700`）
- 通过 `importlib.import_module(f".tasks.{module}", package=__package__)` 装配（不能用 __import__，注释：函数内没有包上下文会报 '__name__' not in globals）。
- 装配顺序：localization → safety → tracker → mission → aim → fix → drone → telemetry。
- `SafetyTask` 额外传 group=group（安全需要看别的心跳）；其余只传 log。
- 模块导入失败 ⇒ 记入 skipped（只告警，不阻断）。
- 布局模式下 skip={"drone"}（DroneTask 已单独起）。

#### 2.1.9 启动横幅 / 退出报告 / bench / 静态规划

- `start_banner(cfg, group, *, plan=None, layout_mode=False, step_mode=False)`（`main.py:1703-1742`）：按路线来源分四种文案；打印配置/地图/巡航/自动出发/越界余量/开火策略/任务线程。
- `_tele_field(tele, name, default="—")`（`main.py:1745-1751`）：Telemetry 对象或 dict 都能取。
- `finish_report(bb, group, bus, *, elapsed)`（`main.py:1754-1768`）：运行时长/阶段位置/发数命中/安全/事件总数 + 每任务 state/ticks/errors/overruns/max_tick_ms。
- `run_bench(cfg, *, seconds, drive=False, speed_factor=1.0, log=LOG) -> int`（`main.py:1774-1927`）：FakeBridge 联调。drive=True 起虚拟小车线程 `_car_loop`（`main.py:1853-1894`），用自行车模型积分；路径统计必须与 Tracker 一样「跟丢自愈」（path.project(..., s_hint=...) + release_m 重投影，`main.py:1880-1894`）。⚠️ speed_factor>1 只是冒烟（控制器按 50 Hz/0.15 m/s 整定，放大必超调）。
- `run_plan_only(cfg, *, map_path=None, log=LOG) -> int`（`main.py:1933-1967`）：调 `tasks.mission.plan_route_from_config`，打印 token 数/折线/倒车段/A* 代价/门禁/足迹/耗时；失败返回 1。

#### 2.1.10 `run_live` —— 正常上车运行

**签名**：`run_live(cfg, *, log=LOG, alive=ros_context_alive, map_path=None) -> int`（`main.py:2003-2183`）
- 退出码约定（`main.py:50-53`）：0=正常；3=无布局/装配失败；4=布局规划失败；5=ROS 环境/位姿管线未就绪；6=启动契约不通过。
- 流程：
  1. `RosBridge("dzbrain", log=log)`；`install_stop_signals`；
  2. 重复实例检测 ⇒ exit 5；`_startup_self_check` 失败 ⇒ exit 5；
  3. `layout_mode = _layout_mode_enabled(cfg, map_path)`；
  4. `startup_contract(bridge, "live")` 失败 ⇒ exit 6；
  5. 布局模式：`drone.enabled=False` ⇒ exit 3；起 `DroneTask`；`bridge.spin_async(threads=4)`；`_wait_for_field_layout` 超时/None ⇒ exit 3；
  6. `step_mode = _step_mode_enabled(first_layout)`：
     - v2（带 known）：`mission.auto_start=False`；跑 `startup_contract(bridge, "race-step")`（失败 exit 6）；
     - v1：`plan_route_from_field_layout` 失败 ⇒ exit 4；成功则 `cfg.override("route.tokens", ...)` + `planner.auto_plan_enabled=False`；
  7. `build_tasks(..., skip={"drone"} if layout_task else None)`；
  8. step_mode 时取 `MissionTask`（缺失 ⇒ exit 3），构造 `StepLoop`；
  9. `start_banner`；`bridge.spin_async`（若 layout_task is None）；
  10. `_wait_for_pose_ready` 失败 ⇒ exit 5；`group.start_all()`；
  11. `step_loop.start(first_layout)` + `_run_step_loop(...)` ⇒ rc（1 context_dead/safe_stop、5 ros_lost、4 timeout、0 其他）；或 `_live_loop(...)`（context_dead ⇒ 1）；
  12. finally：`group.stop_all` → `layout_task.stop` → `bridge.shutdown(brake=True)` → `finish_report` → 步进循环/布局监听健康汇总。

#### 2.1.11 `main(argv=None) -> int`

**签名**：`main.py:2187-2260`。参数表：

| 参数 | 默认 | 说明 |
|---|---|---|
| `--config` | None | robot.yaml 路径（默认 ~/dzrobot/config/robot.yaml） |
| `--map` | None | map.yaml 路径 |
| `--dry-run` | False | 只做启动门禁 |
| `--plan-only` | False | 静态规划一次，打印 token |
| `--bench` | False | FakeBridge 联调 |
| `--drive` | False | bench 挂虚拟小车 |
| `--speed-factor` | 1.0 | 虚拟车时间快进倍数 |
| `--seconds` | 5.0 | bench 时长 |
| `--force` | False | 门禁不过也启动（应急） |

流程要点：
- `allow_missing_map=True` 恒传（车端不留地图文件，`main.py:2205-2209`）；
- `--map` 透传：`cfg.override("planner.map_path", str(Path(args.map_path).resolve()))`（`main.py:2217-2218`）；
- **`--plan-only` 必须在门禁 return 2 之前分流**（`main.py:2220-2229`）：因为门禁校验的是手写 route.tokens，而 --plan-only 的产物由规划器产出，两者无关；否则坏手写路线会让 --plan-only 恒 exit 2（死代码已修）。
- `--plan-only` 下门禁错误只打印一次并显式标注「只针对 route.tokens，与规划结果无关」（`main.py:2235-2242`）。
- 无 `--plan-only` 且 errors 非空且非 `--force` ⇒ exit 2；
- `--dry-run` ⇒ 打印通过 + 布局模式提示 + cfg.summary()；
- `--bench` ⇒ `run_bench`；否则 ⇒ `run_live`。

---

### §2.2 `dzbrain/core/geometry.py` —— 坐标真源

**定位**：新版唯一坐标真源（`geometry.py:2`「★ 本文件是不可退让的契约：公式与旧代码逐字一致，任何改动都会让车全盘错位」）。

#### 2.2.1 常量（`geometry.py:24-48`）

| 常量 | 值 | 说明 |
|---|---|---|
| `COL_LETTERS = "ABCDEFGHIJKLMN"` | 14 列 | |
| `MAP_COLS = 14` / `MAP_ROWS = 11` | | 场地 |
| `BLOCK_SIZE = 0.35` | m | 格边 |
| `MAP_X_MIN/MAX = 0.175/3.675` | m | N/A 列中心线 |
| `MAP_Y_MIN/MAX = 0.175/4.725` | m | N/A 列中心线 |
| `BARRIER_ROW_HI = 6` / `BARRIER_ROW_LO = 7` | | 隔板上/下侧行号（**全仓唯一真源**，`geometry.py:38-43`） |
| `BARRIER_COL_FROM = "D"` | | 隔板封死起点列 |
| `RAW_YAW_TO_MAP_YAW_OFFSET = π/2` | | 原始车体 yaw → 地图 yaw |
| `HEADING_X = 0.0` / `HEADING_Y = π/2` | | 朝 X+ / 朝 Y+ |

**坐标公式**（`geometry.py:8-12`）：
~~~text
x = (10.5 - row_idx) * 0.35      # row_idx 0-based；row 11 → 0.175, row 1 → 3.675
y = (13.5 - col_idx) * 0.35      # col_idx 0-based；col N → 0.175, col A → 4.725
~~~
即 N11 中心 = (0.175, 0.175)，A1 中心 = (3.675, 4.725)，N10 中心 = (0.525, 0.175)。
**格单位** = 米 / 0.35，/odom/localization 发的就是这个（N10 = 格 (1.5, 0.5)）。

#### 2.2.2 函数索引

| 函数 | 行 | 签名 | 返回 | 副作用 |
|---|---|---|---|---|
| `block_to_index` | 54 | (block: str) -> (row_idx, col_idx) | 0-based 行列；非法格名抛 ValueError | 无 |
| `index_to_block` | 70 | (row_idx, col_idx) -> str | 越界夹紧 | 无 |
| `block_to_meters` | 77 | (block: str) -> (x, y) | 格心米制坐标 | 无 |
| `meters_to_block` | 85 | (x, y) -> str | 最近格名 | 无 |
| `block_to_units` | 92 | (block: str) -> (u, v) | 米/0.35 | 无 |
| `units_to_meters` | 98 | (u, v) -> (x, y) | ×0.35 | 无 |
| `meters_to_units` | 103 | (x, y) -> (u, v) | /0.35 | 无 |
| `map_bounds` | 108 | (margin=0.0) -> (x_min,x_max,y_min,y_max) | 外扩 margin | 无 |
| `in_bounds` | 114 | (x, y, margin=0.0) -> bool | | 无 |
| `normalize_angle` | 122 | (angle) -> [-π, π) | | 无 |
| `ang_diff` | 127 | (a, b) -> [-π, π) | normalize_angle(a-b) | 无 |
| `deg / rad` | 132 / 136 | 弧度↔度 | | 无 |
| `yaw_to_quaternion` | 140 | (yaw) -> (qx,qy,qz,qw) | 仅 z/w 非零 | 无 |
| `quaternion_to_yaw` | 146 | (qx,qy,qz,qw) -> yaw | 与旧 euler_from_quaternion 等价 | 无 |
| `heading_token_to_yaw` | 153 | (token) -> float | 'y'→HEADING_Y，其余→HEADING_X | 无 |
| `heading_of` | 161 | (dx, dy, fallback=0.0) -> float | 零向量返回 fallback | 无 |
| `Pose` | 171 | frozen dataclass x,y,yaw | block()/offset_to()/translated()/rotated() | 无 |

#### 2.2.3 关键换算（逐行）

~~~python
def block_to_index(block):                      # geometry.py:54-67
    if not isinstance(block, str) or len(block) < 2:
        raise ValueError(...)
    letter = block[0].upper()
    if letter not in COL_LETTERS: raise ValueError(...)
    number = int(block[1:])                      # ValueError → 转成 ValueError
    if not 1 <= number <= MAP_ROWS: raise ValueError(...)
    return number - 1, COL_LETTERS.index(letter)

def block_to_meters(block):                     # geometry.py:77-82
    row_idx, col_idx = block_to_index(block)
    x = (10.5 - row_idx) * BLOCK_SIZE
    y = (13.5 - col_idx) * BLOCK_SIZE
    return x, y

def meters_to_block(x, y):                      # geometry.py:85-89
    row_idx = round(10.5 - x / BLOCK_SIZE)
    col_idx = round(13.5 - y / BLOCK_SIZE)
    return index_to_block(row_idx, col_idx)

def normalize_angle(angle):                     # geometry.py:122-124
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
~~~

#### 2.2.4 调用方

全仓（validate.py、planner.py、physics.py、parser.py、contract.py、sim.py、main.py 经 geo.）。**任何新模块要换算坐标必须 from ..core import geometry as geo，不得另写常量**。

---

### §2.3 `dzbrain/core/config.py` —— 配置中心（205 键 SCHEMA）

**定位**：一元化读取 + 显式默认值 + 启动期校验（`config.py:2-13`）。吸取旧教训：旧 _load_config_center() 读失败静默回退内置值；旧键名靠 globals()[k.upper()] 批量注入，未登记键永远读不到。对策：所有键集中 SCHEMA；缺键用显式默认并留 default-used；关键键缺失报错；旧结构（path_executor.ros__parameters.* 等）展平成点号键。

#### 2.3.1 `Spec` 与 `Spec.check`（`config.py:31-52`）

~~~python
@dataclass(frozen=True)
class Spec:
    default: Any
    kind: type | tuple[type, ...]
    unit: str = ""
    lo: float | None = None
    hi: float | None = None
    desc: str = ""
    critical: bool = False

    def check(self, value):                    # config.py:41-52
        if isinstance(value, bool) and self.kind is not bool:
            return f"类型不符：期望 {self.kind}，收到 bool"
        if not isinstance(value, self.kind):
            return f"类型不符：期望 {self.kind}，收到 {type(value).__name__}"
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            if self.lo is not None and value < self.lo: return f"低于下限 {self.lo}{self.unit}"
            if self.hi is not None and value > self.hi: return f"高于上限 {self.hi}{self.unit}"
        return None
~~~

#### 2.3.2 SCHEMA 分组（205 键，`config.py:63-561`）

按段分组计数（本轮实数）：

| 段 | 键数 | 代表键 |
|---|---|---|
| `pursuit.*` | 49 | wheelbase_m(0.157) / max_steer_deg(30) / max_steer_rate_deg_s(120) / curvature_gain(1.0) / lookahead_min_m(0.18) / lookahead_max_m(0.45) / lookahead_k_v(1.2) / min_speed_mps(0.08) / cruise_speed_mps(0.15) / accel_up_mps2(0.0) / accel_down_mps2(0.0) / pivot_*(12 键) / motion_heading_eps_m(0.001) / projection_rewind_m(0.15) / reverse_entry_m(0.06) / vertex_floor_m(0.05) / projection_release_m(0.30) / dynamic_gear_*(4 键) |
| `vehicle.*` | 4 | length_m(0.28) / width_m(0.135) / coverage_radius_m(0.035) / ref_offset_m(0.075) |
| `field.*` | 5 | obstacle_radius_m(0.1275) / mine_pad_m(0.12) / barrier_width_m(0.02) / barrier_cols("D..N") / bridge_entry_redundancy_m(0.0875) |
| `planner.*` | 9 | engine("tour") / footprint_gate("warn") / tracker_max_seconds(10.0) / auto_plan_enabled(False) / map_path("") / fallback_to_tokens(True) / allow_reverse(False) / max_seconds(2.0) / pivot_penalty(0.0) / mine_turn_penalty(2.0) |
| `localization.*` | 24 | anchor_* / drift_* / boundary_guard_* / stuck_* / yaw_guard 系 |
| `route.*` | 7 | tokens / cross_lower_candidates / cross_upper_candidates / cross_lower_default(K10) / cross_upper_default(K5) / shoot_distance_m(0.80) / shoot_distance_tol_m(0.10) |
| `mission.*` | 21 | auto_start(True) / reset("N10,y") / scan_mode / bridge_info_wait_s / bridge_wait_at_start / race_budget_s / shots_per_stop(3) / settle_* / skip_second_stop_after_hit(True) / stall_timeout_s / stall_yaw_deg |
| `aim.*` | 38 | scan_request_on_start / lead_time_s / fire_policy / rail_span_m / max_shots_* / fire_box_* / lost_frames_to_rescan / red_early_leave_s / est_* / gimbal_lock_* |
| `fix.*` | 16 | anchor_at_waypoint / anchor_max_offset_m / anchor_gain / scale_est_enabled / ratio_* / ds_max_ratio / ds_floor_m / ds_reject_max_run |
| `drone.*` | 9 | enabled / topic_in/out / heartbeat_timeout_s / info_stale_s / car_state_period_s / ack_enabled / field_diff_max / field_layout_wait_s / map_path |
| `step.*` | 3 | wait_timeout_s(0.0) / pose_ready_wait_s(8.0) / odom_watch_s(6.0) |
| `safety.*` | 4 | pose_timeout_s(1.0) / startup_grace_s(3.0) / task_stall_s(1.0) / brake_on_fault(True) |
| `logging.*` / `telemetry.*` | 2 | rate_limit_s(1.0) / period_s(0.5) |

**缺省值 = 代码兜底值，不是现场调过的值**（`config.py:231-233` 的 ⚠️）：例如 pivot_penalty 现场 5.0 而 SCHEMA 兜底 0.0；auto_plan_enabled 现场 true 而兜底 False。

#### 2.3.3 `INTENTIONALLY_UNREGISTERED`（`config.py:570-572`）

frozenset({"localization.yaw_trim_deg"}) —— 故意不登记；它的 ±10° 护栏由 tools/calib_backfill.py 提供，tests/test_calib_backfill.py:2.3 把「SCHEMA 里没有它」当设计约束钉住。⚠️ 往这里加键必须写明护栏位置。

#### 2.3.4 旧段名映射 `SECTION_PREFIX`（`config.py:579-593`）

| 旧段 | 前缀 |
|---|---|
| /dzbrain / dzbrain | ""（空前缀） |
| path_executor / /path_executor | path_executor |
| dzactuator / /dzactuator | dzactuator |
| dzrknn_dataprocess / /dzrknn_dataprocess | dzrknn_dataprocess |
| det_node / usb_cam_node / dzjudgment | 同名 |

**为什么 dzbrain 映射为空前缀**（`config.py:575-578`）：其 ros__parameters 下的 pursuit:/route:/… 会直接展平成 pursuit.wheelbase_m；而 YAML 那边仍符合 ROS 的 <节点名>: ros__parameters: 结构 —— 顶层多一个裸键会让 --params-file 解析失败、相机/视觉/云台三节点同时 abort。

#### 2.3.5 `_flatten`（`config.py:607-619`）

~~~python
def _flatten(node, prefix=""):
    out = {}
    if isinstance(node, dict):
        for k, v in node.items():
            key = str(k)
            if key == "ros__parameters":
                out.update(_flatten(v, prefix)); continue
            out.update(_flatten(v, f"{prefix}{key}."))
    else:
        out[prefix.rstrip(".")] = node
    return out
~~~

#### 2.3.6 `Config` 类

| 方法 | 行 | 签名 | 行为 |
|---|---|---|---|
| `__init__` | 629 | (flat, source="<memory>", raw=None) | 存 _flat/source/raw/issues=[]/_schema_hits=set() |
| `load` | 638 | (path=None) -> Config | 读 YAML；按 SECTION_PREFIX 展平；_migrate_legacy() |
| `from_dict` | 658 | (data, source="<dict>") -> Config | 展平 + 迁移 |
| `_migrate_legacy` | 665 | () -> None | 旧键→新键（不删旧键，只补新键） |
| `get` | 727 | (key, default=None) -> Any | 键存在 ⇒ 即时 _check_against_schema；否则查 SCHEMA（critical 抛错；default is None ⇒ 返回 spec.default） |
| `require` | 741 | (key) -> Any | get(key, None)；None ⇒ ConfigError |
| `_check_against_schema` | 747 | (key, value) | 首次遇到该键才校验并记 Issue("error", ...) |
| `section` | 756 | (prefix) -> dict | 去前缀的子字典 |
| `keys` | 760 | () -> Iterable[str] | |
| `override` | 763 | (key, value) -> Config | **只给 bench/测试用**，运行期改配置的唯一入口 |
| `validate` | 773 | (strict=True) -> list[Issue] | 范围/类型/互斥/一致性；错误非空且 strict ⇒ ConfigError |
| `_signatures` | 849 | | |
| `summary` | 852 | () -> str | |

#### 2.3.7 旧键迁移表 `_migrate_legacy`（`config.py:665-724`）—— **重点**

映射方向 = 新键 ← 旧键。摘录（完整 46 条）：

| 新键 | 旧键 | 备注 |
|---|---|---|
| `route.tokens` | `path_executor.mission_blocks` | |
| `mission.auto_start` | `path_executor.auto_start` | |
| `mission.auto_start_delay_s` | `path_executor.auto_start_delay` | |
| `mission.reset` | `path_executor.mission_reset` | |
| `mission.scan_mode` | `path_executor.mission_scan_mode` | |
| `pursuit.cruise_speed_mps` | `path_executor.linear_speed` | |
| `pursuit.reverse_speed_ratio` | `path_executor.reverse_speed_ratio` | |
| `pursuit.reverse_steer_sign` | `path_executor.reverse_steer_sign` | |
| `pursuit.map_safety_margin_m` | `path_executor.map_safety_margin` | |
| `pursuit.drift_warn_m` | `path_executor.drift_warn_m` | |
| `pursuit.bridge_blocks` | `path_executor.bridge_blocks` | |
| `pursuit.align_*` | `path_executor.align_*` | 5 条 |
| `pursuit.stop_radius_m` | `pursuit.shoot_stop_radius_m` | |
| `mission.shots_per_stop` | `path_executor.shoot_count_target` | |
| `mission.shoot_timeout_s` | `path_executor.shoot_timeout` | |
| `mission.lock_timeout_s` | `path_executor.shoot_lock_timeout` | |
| `mission.settle_time_s` | `path_executor.shoot_settle_time` | |
| `mission.settle_eps_m` | `path_executor.shoot_settle_speed` | |
| `mission.stable_timeout_s` | `path_executor.shoot_stable_timeout` | |
| `mission.stable_offset_px` | `path_executor.shoot_stable_offset_px` | |
| `mission.candidate_retry` | `path_executor.shoot_max_retry` | |
| `mission.finish_voice_id` | `path_executor.finish_voice_id` | |
| `aim.max_shots_total` | `dzrknn_dataprocess.max_shots` | |
| `aim.max_shots_per_target` | `dzrknn_dataprocess.max_shots_per_target` | |
| `aim.shot_cooldown_ms` | `dzrknn_dataprocess.shot_cooldown_ms` | |
| `aim.fire_box_far_px` | `dzrknn_dataprocess.fire_dz_far_px` | |
| `aim.fire_box_near_px` | `dzrknn_dataprocess.fire_dz_near_px` | |
| `aim.lost_frames_to_rescan` | `dzrknn_dataprocess.lost_frames_to_rescan` | |
| `aim.scan_pitch_offset_deg` | `dzrknn_dataprocess.scan_pitch_offset_deg` | |
| `vehicle.*`（4 条） | `dzactuator.vehicle.*` | **真 bug 修复 2026-10-16**：真源在 /dzactuator 段，而 planner/sim/route_check 读规范名；缺这组映射时规范名恒吃 SCHEMA 默认 ⇒ 现场改尺寸被静默忽略（P0） |
| `field.*`（4 条） | `dzactuator.field.*` | 同上 |

**迁移实现**（`config.py:717-724`）：
~~~python
for new_key, olds in legacy_map.items():
    if new_key in self._flat: continue
    for old in olds:
        if old in self._flat:
            self._flat[new_key] = self._flat[old]
            self.issues.append(Issue("info", new_key, f"由旧键 {old} 迁移"))
            break
~~~
⚠️ 迁移**不删旧键**（双向兼容）；迁移后 issues 里多一条 info。

#### 2.3.8 `validate()` 两级未知键判据（`config.py:797-822`）—— **重点**

~~~text
① 键在 SCHEMA 里              ⇒ 合法（登记即生效）
② 键的段在 SCHEMA 里但键本身没登记 ⇒ 报"未登记的配置键"（拼错仍能抓到）
③ 段也不认识（ROS 节点段）     ⇒ 透传不报
~~~
node_prefixes = ("path_executor.", "dzactuator.", "dzrknn_dataprocess.", "det_node.", "usb_cam_node.", "dzjudgment.", "ackermann_sim.")（`config.py:809-810`）。
⚠️ 不能只做 ①+③：那样 planner.pivot_penalty 这类段内拼错会静默通过（注释引项目踩过的坑）。

#### 2.3.9 一致性断言（`config.py:824-842`）

- `pursuit.lookahead_min_m > lookahead_max_m` ⇒ error；
- `route.tokens` 非字符串 token ⇒ error；
- `route.cross_lower_default ∉ cross_lower_candidates` ⇒ warn（上/下各一条）；
- `pursuit.align_straight_speed_mps > cruise_speed_mps` ⇒ warn。

#### 2.3.10 模块级工具函数

~~~python
def lookahead_distance(speed, cfg):        # config.py:861-866
    ld = cfg.get("pursuit.lookahead_k_v") * abs(speed)
    lo = cfg.get("pursuit.lookahead_min_m")
    hi = cfg.get("pursuit.lookahead_max_m")
    return max(lo, min(hi, ld))            # = clamp(k_v·|v|, [min, max])

def curve_speed_limit(curvature, cfg):    # config.py:869-875
    a = cfg.get("pursuit.lat_acc_max_mps2")
    k = abs(curvature)
    if k < 1e-6: return float("inf")
    return math.sqrt(a / k)               # v = sqrt(a_lat/|κ|)

def pivot_dist_eff(cfg, speed_mps):       # config.py:878-889
    base = cfg.get("pursuit.pivot_dist_m")
    lead = cfg.get("pursuit.pivot_speed_lead_s")
    ref = cfg.get("pursuit.pivot_speed_ref_mps")
    cap = cfg.get("pursuit.pivot_dist_max_m")
    eff = base + lead * max(0.0, abs(speed_mps) - ref)
    return max(base, min(cap, eff))       # clamp(dist + lead×max(0,v−v_ref), [dist, dist_max])
~~~
调用方：pursuit/controller.py（前视距离）、pursuit/controller.py:129（曲率限速）、tasks/tracker.py（PIVOT 触发距离）。

---

### §2.4 `dzbrain/core/` —— 共享状态 / 事件 / 任务线程 / 日志 / 跨任务状态

#### 2.4.1 `core/blackboard.py` —— 高频只读共享通道

**设计要点**（`blackboard.py:4-8`）：值必须不可变；每项带时间戳与来源；一把 RLock；临界区只有 dict 存取；**禁止**在持锁期间做 I/O 或外部回调；高频数据（位姿、视觉帧）走这里，低频事件走 EventBus。

**标准键名**（`blackboard.py:21-87`）—— 这些是**跨任务契约**，注释里写了读写方：

| 常量 | 值 | 语义（读侧契约） |
|---|---|---|
| `P_POSE` | "pose" | 信念位姿 Pose |
| `P_POSE_RAW` | "pose_raw" | 未修正原始融合位姿 |
| `P_ODOM_AGE` | "odom_age" | 最近一帧里程计距今秒数 |
| `P_ROUTE` | "route" | 当前路线 |
| `P_ROUTE_INDEX` / `P_ROUTE_S` | | 航段索引 / 沿程弧长 |
| `P_MISSION_PHASE` | "mission_phase" | IDLE/DRIVE/APPROACH/STOP/WAIT/FINISH/SAFE |
| `P_STOP_POINT` | "stop_point" | 当前停站点上下文 |
| `P_TARGET` | "target" | 移动靶估计状态 |
| `P_AIM` | "aim" | 云台与开火状态 |
| `P_AIM_RED` | "aim_red_face" | **AimTask 写、MissionTask 读**（唯一消费者） |
| `P_DRONE` | "drone" | 无人机情报 |
| `P_FIELD_LAYOUT` | "field_layout" | 无人机布局（DroneTask 写 / main 读） |
| `P_FIELD_VERIFY` | "field_verify" | 核对差异汇总 |
| `P_FIX` / `P_FIX_EST` | | 修正量 / 估计量 |
| `P_SAFETY` | "safety" | 安全状态 |
| `P_TELEMETRY` | "telemetry" | 遥测快照 |
| `P_PURSUIT_GEAR` | "pursuit_gear" | Tracker 写、遥测/诊断读（观测面） |
| `P_MOTION_LIMIT` | "motion_limit" | **Localization 写、pursuit 读**（限速请求） |

**P_AIM_RED 读写契约**（`blackboard.py:32-47`）：键不存在/None ⇒ no-op；{"confirmed":False,...} ⇒ 仅观测；{"confirmed":True,...} 且连续累计 ≥ aim.red_early_leave_s ⇒ Mission 在射击停站内提前离站。写侧纪律：离开停站或类别变红 ⇒ 立即归零 run_s、confirmed=False。

**P_MOTION_LIMIT 读写契约**（`blackboard.py:70-86`）：键不存在/None ⇒ no-op；{"speed_mps":...} ⇒ 把线速度**钳到该值以内（只降不升）**；倒车按绝对值钳（保号）；speed_mps=0.0 是请求停车，**不受 min_speed_mps 兜底**。**不取代 Safety**：P_SAFETY 仍是唯一强制停车者。

**类方法**（`blackboard.py:97-161`）：

| 方法 | 行 | 签名 | 返回 | 副作用 |
|---|---|---|---|---|
| `set` | 106 | (key, value, source="", ts=None) | None | 存 Entry(value,source,ts)；writes += 1 |
| `update` | 112 | (source="", **items) | None | 批量写，同一 ts |
| `get` | 120 | (key, default=None) | 值/默认 | 无 |
| `entry` | 125 | (key) | Entry|None | 无 |
| `age` | 129 | (key) | None 或距今秒 | 无 |
| `get_fresh` | 135 | (key, max_age_s, default=None) | 新鲜则值，否则 default | 无 |
| `is_fresh` | 143 | (key, max_age_s) -> bool | | 无 |
| `snapshot` | 147 | (keys=None) -> dict | 单次加锁取全部 | 无 |
| `describe` | 154 | () -> dict[str, dict] | 值摘要+age+source | 无 |

~~~python
def get_fresh(self, key, max_age_s, default=None):     # blackboard.py:135-141
    with self._lock:
        entry = self._data.get(key)
    if entry is None or (time.time() - entry.ts) > max_age_s:
        return default
    return entry.value
~~~
调用方：几乎全仓（controller.py:68 读 P_MOTION_LIMIT、main.py 读 P_FIELD_LAYOUT/P_CAR_STATE、tasks/*）。

#### 2.4.2 `core/events.py` —— 低频事件总线

**设计要点**（`events.py:4-8`）：发布非阻塞（消费者慢⇒丢事件并计数，不拖慢控制回路）；每消费者独立队列；事件不可变；**禁止用事件传高频数据**。

**事件类型常量**（`events.py:24-48`）：EV_MISSION_STARTED/FINISHED、EV_ROUTE_LOADED、EV_WAYPOINT_REACHED、EV_ARRIVED_STOP_POINT、EV_STOP_POINT_DONE、EV_TARGET_FIRST_SEEN/LOST、EV_FIRE_ALLOWED/BLOCKED、EV_HIT_CONFIRMED、EV_AMMO_CHANGED、EV_POSE_ANCHORED、EV_DRIFT_WARNING、EV_DRONE_ONLINE/OFFLINE、EV_BRIDGE_INFO、EV_FIELD_VERIFY、EV_SAFETY_STOP/CLEAR、EV_TASK_FAULT。

**类**：

| 类/方法 | 行 | 签名 | 返回 | 副作用 |
|---|---|---|---|---|
| `Event` | 51 | frozen type, source, data, ts, seq | | 无 |
| `Subscription.__init__` | 65 | (bus, name, maxsize=256) | | 建 queue.Queue；dropped=0 |
| `Subscription.poll` | 71 | (max_items=32) -> list[Event] | 最多 N 条（非阻塞） | 无 |
| `Subscription.drain` | 81 | (handler, max_items=32) -> int | 处理条数 | 回调 handler |
| `Subscription._offer` | 88 | (ev) | None | 满则 dropped += 1 |
| `Subscription.unsubscribe` | 94 | () | None | bus._remove(self) |
| `EventBus.__init__` | 101 | () | | _subs = dict[str, dict[str, Subscription]]；_seq = itertools.count(1) |
| `EventBus.subscribe` | 107 | (name, types=None, maxsize=256) -> Subscription | 订阅句柄 | types=None/[] ⇒ 订阅全部（"*"） |
| `EventBus.publish` | 115 | (ev_type, source="", **data) -> Event | 事件 | published += 1；非阻塞 |
| `EventBus._remove` | 128 | (sub) | None | 从所有桶移除 |
| `EventBus.stats` | 133 | () -> dict | published/subscribers/dropped | 无 |

~~~python
def publish(self, ev_type, source="", **data):   # events.py:115-126
    ev = Event(type=ev_type, source=source, data=dict(data), seq=next(self._seq))
    with self._lock:
        targets = list(self._subs.get(ev_type, {}).values())
        if "*" in self._subs:
            targets.extend(s for s in self._subs["*"].values()
                           if s.name not in {t.name for t in targets})
    self.published += 1
    for sub in targets:
        sub._offer(ev)
    return ev
~~~
调用方：core/task.py:175（EV_TASK_FAULT）、tasks/*、main.py（EventBus 传给 build_tasks）。

#### 2.4.3 `core/task.py` —— 任务线程基类

**设计要点**（`task.py:4-14`）：固定频率（绝对时基排下一拍，补偿执行耗时）；单拍异常隔离（连续 ≥ max_consecutive_errors=5 ⇒ FAULT）；心跳可查；优雅停止。约束：tick() 必须非阻塞、无长 I/O、不持锁等待别的任务；任务之间**只能**通过 Blackboard/EventBus 通信。

| 类/方法 | 行 | 签名 | 返回 | 副作用 |
|---|---|---|---|---|
| `TaskState` | 30 | Enum：INIT/RUNNING/DEGRADED/FAULT/STOPPED | | |
| `TaskHealth` | 38 | name/rate_hz/state/ticks/errors/consecutive_errors/overruns/max_tick_ms/last_tick_at/last_error/started_at；age(now=None) | | |
| `TaskThread.__init__` | 65 | (name, rate_hz, *, bus=None, blackboard=None, logger=None) | | daemon=True；rate_hz<=0 抛 ValueError |
| `setup/tick/teardown/fault_hook` | 90/93/97/100 | 子类接口 | | tick 默认 NotImplementedError |
| `TaskThread.run` | 106 | () | None | setup 失败 ⇒ _fail("setup")；循环 tick；异常计数；超时/落后重置时基 |
| `TaskThread._fail` | 165 | (where, exc) | None | 置 FAULT、记 last_error、调 fault_hook、发布 EV_TASK_FAULT、_stop_event.set() |
| `TaskThread.stop` | 179 | (timeout=2.0) -> bool | 是否已退出 | _stop_event.set() + join |
| `TaskThread.stopping` | 186 | property | _stop_event.is_set() | |
| `TaskThread.health` | 190 | () -> TaskHealth | 拷贝 | 加锁 |
| `TaskGroup.__init__` | 198 | (bus, blackboard, logger=None) | | |
| `TaskGroup.add` | 205 | (task) -> task | | |
| `TaskGroup.start_all` | 209 | () | | 逐个 start() |
| `TaskGroup.stop_all` | 215 | (timeout=2.0) -> list[str] | 未优雅退出的任务名 | **逆序**停止 |
| `TaskGroup.health/faulty/describe` | 223/226/229 | | dict / list[str] / list[dict] | |

**run() 主循环**（`task.py:117-154`）：
~~~python
next_deadline = time.perf_counter()
while not self._stop_event.is_set():
    t0 = time.perf_counter()
    try:
        self.tick()
        h.ticks += 1; h.consecutive_errors = 0; h.last_tick_at = time.time()
    except Exception as exc:
        h.errors += 1; h.consecutive_errors += 1; h.state = DEGRADED
        if consec >= self.max_consecutive_errors:
            self._fail("tick", exc); break
    dt_ms = (time.perf_counter() - t0) * 1000.0
    if dt_ms > self.period * 1000.0: h.overruns += 1
    next_deadline += self.period
    sleep_s = next_deadline - time.perf_counter()
    if sleep_s > 0: self._stop_event.wait(sleep_s)
    else: next_deadline = time.perf_counter(); h.overruns += 1
~~~
调用方：main.py:build_tasks / start_banner / finish_report；被调用方：tasks/* 子类。

#### 2.4.4 `core/state.py` —— 跨任务共享状态类型（全 frozen）

**定位**（`state.py:2-6`）：集中定义，避免字段名与语义漂移（旧代码 90 个状态字段平铺在 __init__）。

| 类 | 行 | 字段要点 |
|---|---|---|
| `SafetyState` | 16 | ok/reasons/since/stops；with_reason(reason, ts, *, ok=False) 去重 reasons、按 ok 计 stops |
| `StopKind` | 33 | SHOOT / SCAN |
| `StopPhase` | 38 | SETTLE / ACQUIRE / FIRE / DONE |
| `StopPointState` | 45 | index/block/kind/phase/group/started_at/phase_started_at/shots_at_arrival/shots_needed/detail；active property（index≥0 且 phase≠DONE） |
| `TargetState` | 68 | seen/dx/dy/cls/box_w/seen_at/lost_since/s_rail/v_rail/samples/notes |
| `AimState` | 86 | status/fire_gate/lead_px/shots_fired/hits/ammo/cooled_until/last_reason |
| `DroneState` | 103 | online/last_heartbeat/bridge_half/bridge_block/bridge_confidence/bridge_ts/field_diff/messages/last_type；bridge_of(half, *, now, stale_s) |
| `FixState` | 127 | dx/dy/dyaw/scale/anchor_count/last_anchor/drift_m/max_drift_m；corrected(x,y,yaw) |
| `Telemetry` | 151 | 一页纸：phase/block/x/y/yaw_deg/speed/steer_deg/seg_index/s/route_len/aim/target/shots/hits/ammo/safety_ok/drone/extra |

#### 2.4.5 `core/logging.py` —— 横幅 / 限流 / 一次性守卫

| 名称 | 行 | 签名 | 行为 |
|---|---|---|---|
| `WIDTH = 78` | 19 | | |
| `banner(lines, char="=", log=print)` | 22 | | 打 78 宽框；str 自动转 list |
| `kv_lines(data, indent=2)` | 32 | | dict→对齐多行 |
| `Throttle` | 41 | interval=1.0；_last: dict | allow(key, now=None) 首次必放行；reset(key=None) |
| `FaultGuard` | 63 | _fired: set | first(key) 首次 True；clear(key=None) |

调用方：全仓（main.py:16、start_banner、finish_report、tasks/*）。


---

### §2.5 `io/contract.py` / `io/referee.py` / `io/ros_io.py` —— ROS 契约与接线

#### 2.5.1 `io/contract.py` —— 纯数据契约层

**定位**：ROS 契约集中层（**纯数据，无 rclpy 依赖，可本地单测**，`contract.py:2`）。旧代码最容易错的不是算法，而是**单位与坐标系约定散落各处**：/odom/localization 的 position 是【格】不是米；pursuitAngle.angular.z 是【度】舵角而 cmd_vel.angular.z 是【弧度/秒】；四元数 yaw 是原始车体角，要 +π/2。**任务层只允许通过本层访问话题，禁止再手写话题名/单位换算**（`contract.py:9`）。

**QoS 契约**（`contract.py:24-41`）：

| 常量 | 值 | 说明 |
|---|---|---|
| `QosSpec` | frozen depth/reliability/durability | describe() 返回 KEEP_LAST(n)/RELIABLE/VOLATILE |
| `QOS_PURSUIT_ANGLE` | depth=1, BEST_EFFORT | ★ 底盘对 pursuitAngle 订阅是 depth 1 / BEST_EFFORT（survey §2.6、L776-782）；换 RELIABLE 会让底盘收到慢/旧指令，表现为「车一顿一顿」 |
| `QOS_ODOM` / `QOS_SENSOR_FRAME` | depth=10, BEST_EFFORT | |
| `QOS_EVENT` | depth=10, RELIABLE | |
| `QOS_LINK` | depth=10, BEST_EFFORT | 无人机链路允许丢旧帧 |

**话题表 `TOPICS`**（`contract.py:48-110`，键→Topic）：

| 键 | 话题名 | 方向 | QoS | 单位/说明 |
|---|---|---|---|---|
| `odom` | /odom/localization | sub | QOS_ODOM | 格（×0.35=米）；yaw=原始车体角(+π/2=地图航向)；发布者是 serial_reader |
| `imu` | /imu_data | sub | QOS_SENSOR_FRAME | rad/s, m/s² |
| `offset_center` | /offset_center | sub | QOS_EVENT | px / 类别 / 框宽高；字段 [0]dx [1]dy [2]cls+1 [3]框宽 [4]框高 [5..7]关键点 |
| `aim_status` | /aim/status | sub | QOS_EVENT | 0=IDLE 1=SCAN 2=TRACK |
| `shoot_count` | /shoot_count | sub | QOS_EVENT | 累计发数 |
| `manual_override` | /nav/manual_override | sub | QOS_EVENT | 1=遥控接管底盘 |
| `manual_gimbal` | nav/manual_gimbal | sub | QOS_EVENT | 1=遥控接管云台（注意：历史话题名没有前导斜杠） |
| `waypoint_cmd` | /nav/waypoint | sub | QOS_EVENT | 外部命令：路线/复位/单点 |
| `pursuit_angle` | pursuitAngle | pub | QOS_PURSUIT_ANGLE | linear.x=m/s；★angular.z=舵角【度】(正=左转)；零 Twist=刹车；50Hz 保活 |
| `scan_request` | /nav/scan_request | pub | QOS_EVENT | 0=停扫 1=单趟 2=持续 |
| `aim_fire_gate` | /aim/fire_gate | pub | QOS_EVENT | 1=允许开火 |
| `aim_fire_ready` | /aim/fire_ready | pub | QOS_EVENT | 1=瞄准合格（预测落点在开火框内）—— 与 fire_gate 构成**两键开火** |
| `stop_signal` | /move_base/stop_signal | pub | QOS_EVENT | 1=允许云台扫描+射击（历史话题，云台侧原先无人订阅） |
| `motion_state` | /nav/motion_state | pub | QOS_EVENT | 0 前进 / 1 倒车（仅变化时发） |
| `voice_switch` | /voice_switch | pub | QOS_EVENT | 语音 ID |
| `drone_in` | /dzlink/drone_to_car | sub | QOS_LINK | JSON 情报：桥洞/靶区/雷障核对/心跳 |
| `drone_out` | /dzlink/car_to_drone | pub | QOS_LINK | JSON：车状态/ack |
| `referee_hp` | /HpAndHitmsg | sub | QOS_EVENT | 裁判回传帧（★★命中闭环真源） |
| `referee_hit` | /referee/hit | sub | QOS_EVENT | 命中确认（车端**无生产者**；只在从未收到合法帧时才认） |
| `referee_ammo` | /referee/ammo | sub | QOS_EVENT | 剩余弹药（兜底源，referee_hp 的 [4] 才是真源） |
| `aim_lead` | /aim/lead_x | pub | QOS_EVENT | 提前量（px，云台补丁订阅） |

**函数**：

| 函数 | 行 | 签名 | 返回 | 副作用 |
|---|---|---|---|---|
| `topic(key)` | 113 | (key) -> Topic | 话题定义 | 拼错抛 KeyError（提示可选键） |
| `odom_to_pose` | 123 | (pos_x_units, pos_y_units, quat) -> geo.Pose | 米制地图 Pose | 无 |
| `pose_to_pursuit_twist` | 136 | (linear_x_mps, steer_deg) -> dict | {linear_x, angular_z_deg} | 无 |
| `clamp_steer_for_chassis` | 144 | (steer_deg, cfg) -> float | 夹到 ±min(cfg.max_steer_deg, 35.0) | 无 |
| `chassis_speed_counts` | 150 | (linear_x_mps) -> int | int(abs(v)*23.0/0.33)（1 计数≈0.0144 m/s；旧代码 0.15→10，少 4.3%） | 无 |
| `is_brake_command` | 165 | (cmd) -> bool | | 无 |
| `decode_offset_center` | 233 | (data, ts=0.0) -> OffsetObservation|None | 长度 <5 一律 None（绝不猜字段，防 C++ 越界读） | 无 |
| `decode_aim_status` | 252 | (value) -> str | | 无 |
| `_norm_coverage_cells` | 341 | (items) -> (cells, dropped) | 大写去空格、去重、丢越界/非字符串 | 无 |
| `decode_slice_info` | 372 | (value) -> SliceInfo|None | 形状非法返回 None | 无 |
| `decode_coverage` | 390 | (value, *, scan_id, slice_info) -> Coverage|None | 容错优先级见下 | 无 |
| `ContractSummary.render` | 450 | () -> list[str] | | 无 |

**枚举**（`contract.py:172-207`）：
- `AimStatus`：IDLE=0 / SCAN=1 / TRACK=2；NAMES；name(value)。
- `MissionPhase`：IDLE/DRIVE/APPROACH/STOP/SHOOT/WAIT_SCAN/HOLD/FINISH/SAFE_STOP；ALL。HOLD 注释：不在 tracker.ACTIVE_PHASES 里 ⇒ TrackerTask 每拍走自己的 _hold() 分支（publish_brake），不需要第二个驱动指令写者。
- `ShotClass`：NONE=0 / BLUE=1 / RED=2（/offset_center[2] 编码）。

**数据结构**：`OffsetObservation`（dx/dy/cls/box_w/box_h/ts；has_target/bearing_px）、`SliceInfo`（id/index/of/route；valid/to_dict）、`Coverage`（cells/complete/mode/scan_id/slice/dropped/problems；key/to_dict）。

**decode_coverage 容错优先级**（`contract.py:396-401`）：
1. cells 必须是 list/tuple，否则整条 coverage 非法（None）；
2. 缺/非 bool 的 complete ⇒ False（UNKNOWN）+ problems；
3. mode 非法 ⇒ 回退 strip + problems；
4. scan_id 超长（>64）⇒ 丢弃去重键 + problems（不丢数据）；
5. slice 形状非法 ⇒ slice=None + problems（不丢数据）。
**向后兼容**：旧机侧不发 coverage ⇒ 本函数不被调用或返回 None，车侧行为与加特性前逐位一致（test_drone ⑬ A/B 证据）。

**关键换算（逐行）**（`contract.py:123-155`）：
~~~python
def odom_to_pose(pos_x_units, pos_y_units, quat):     # contract.py:123-133
    x, y = geo.units_to_meters(pos_x_units, pos_y_units)   # ① 格 → 米
    raw_yaw = geo.quaternion_to_yaw(*quat)
    return geo.Pose(x, y, geo.normalize_angle(raw_yaw + geo.RAW_YAW_TO_MAP_YAW_OFFSET))  # ② +π/2

def clamp_steer_for_chassis(steer_deg, cfg):          # contract.py:144-147
    limit = min(cfg.get("pursuit.max_steer_deg"), 35.0)
    return max(-limit, min(limit, float(steer_deg)))

def chassis_speed_counts(linear_x_mps):               # contract.py:150-155
    return int(abs(linear_x_mps) * 23.0 / 0.33)
~~~
调用方：ros_io.on_odom（229-239）、tasks/*、tests/test_contract.py。

#### 2.5.2 `io/referee.py` —— 裁判帧解析（命中/挨打/弹药）

**为什么有这个模块**（`referee.py:5-18`）：原先 io/contract.py 声明订阅 /referee/hit、/referee/ammo，但**车端从未实现这两个话题**（全仓检索：除 dzbrain 自己外没有任何生产者）⇒ AimTask 的「命中即停火」与「弹药对账」在真车上恒空转（take_hits() 永远返回 0，看代码却一切正常）。处置：直接消费车端已经在发的真实话题 /HpAndHitmsg（judgment.cpp:193-203 发布；case 0x07 收帧即发 + 1 Hz 心跳），在纯 Python 侧解析。

**帧格式**（`referee.py:23-29`，真源 judgment.cpp:195-201 + judgment.h:87-91）：

| 下标 | 字段 | 语义 | 初值 |
|---|---|---|---|
| [0] | targethp | **对方血量** —— 我们打中靶的唯一真信号 | 0 |
| [1] | self_hp | 己方血量 —— 我们挨打 | 0 |
| [2] | Bing_hit | 被击打方位 0x01左 0x02前 0x03右 0x04后 0x10未被 | 0x10 |
| [3] | Shooting_Count | 硬件侧累计射击次数 | 0 |
| [4] | ammo | 剩余弹药 | 10 |

**两处必须记牢的语义纠正**（`referee.py:31-37`）：
1. **命中判据是 targethp 下降，不是 Bing_hit**（Bing_hit 是「我方被谁打了」的方位）；拿它当命中确认会把「我们挨打」记成「我们打中」，于是每挨一发就停火一次。
2. **心跳与实帧的值相同** ⇒ 计数**必须基于状态变化**（HP 下降量），绝不能按「收到帧」计数 —— 否则 1 Hz 心跳会把命中数刷成天文数字。

**类与方法**：

| 名称 | 行 | 说明 |
|---|---|---|
| `FRAME_LEN = 5` | 55 | |
| `IDX_TARGET_HP/SELF_HP/BING_HIT/SHOTS_HW/AMMO` | 58-62 | |
| `BING_NOT_HIT = 0x10` | 65 | |
| `BING_NAMES` | 68 | 方位码→中文 |
| `RefereeFrame` | 73 | frozen targethp/self_hp/bing_hit/shots_hw/ammo/at；bing_name/being_hit |
| `RefereeDelta` | 94 | hits/self_hits/reset/frame；__bool__ |
| `RefereeTracker` | 107 | frames/malformed/resets/hits_total/self_hits_total/_last/last_seen_at |
| `RefereeTracker.state` | 129 | 最近一帧 |
| `RefereeTracker.valid_seen` | 133 | 是否至少收到过一帧合法帧（ros_io 据此决定是否退回兜底话题） |
| `RefereeTracker.ammo` | 142 | None = 未知，绝不假设成某个数 |
| `RefereeTracker.age(now)` | 146 | inf = 从未收到 |
| `RefereeTracker.feed(data, now)` | 153 | 喂一帧；返回 RefereeDelta |
| `RefereeTracker.describe()` | 196 | 人类可读一行 |

**feed 关键实现**（`referee.py:159-191`）：
~~~python
if data is None or len(data) < FRAME_LEN:
    self.malformed += 1; return RefereeDelta()          # 长度不足 ⇒ 丢弃
try:
    vals = [int(v) & 0xFF for v in list(data)[:FRAME_LEN]]
except (TypeError, ValueError):
    self.malformed += 1; return RefereeDelta()
frame = RefereeFrame(...)
prev = self._last; self._last = frame; self.frames += 1
if prev is None: return RefereeDelta(frame=frame)       # 首帧只建基线
hits = max(0, prev.targethp - frame.targethp)           # 下降 = 我们打中靶
self_hits = max(0, prev.self_hp - frame.self_hp)        # 下降 = 我们挨打
reset = frame.targethp > prev.targethp                  # 上升 = 新一局/复位
~~~
调用方：`ros_io.on_referee_hp`（`ros_io.py:284-301`）。被调用方：tests/test_referee.py。

#### 2.5.3 `io/ros_io.py` —— rclpy 接线

**定位**：唯一允许 import rclpy 的地方之一（`ros_io.py:2`）。订阅回调**只做三件事**：解码（走 contract 纯函数）、打时间戳、写缓存；绝不在回调里做决策/计算。发布走命名方法，话题名/QoS/单位一律取自 contract。rclpy **惰性导入**，无 ROS 机器也能 import（只是不能实例化）。

**模块级常量/函数**：

| 名称 | 行 | 说明 |
|---|---|---|
| `REQUIRED_SUBSCRIPTIONS` | 30 | ("odom","imu","offset_center","aim_status","shoot_count","manual_override","manual_gimbal","drone_in","waypoint_cmd") —— 决策层必须订阅的会话句柄（2026-10-18 现场事故教训：新构建起栈后「图里有没有 /odom/localization 订阅」没人当场知道） |
| `required_subscription_topics()` | 36 | 逻辑名→实际话题名（纯函数） |
| `WATCHED_TOPICS` | 43 | ("odom","drone_in","pursuit_angle") —— 启动自检打点数的关键话题 |
| `watched_topic_names()` | 46 | |
| `ros_env()` | 50 | ROS_DOMAIN_ID/ROS_LOCALHOST_ONLY/RMW_IMPLEMENTATION/DZ_ROS_DOMAIN（纯函数） |
| `env_problems()` | 67 | (errors, warnings)：域号与 DZ_ROS_DOMAIN 不一致 ⇒ error（不出发）；DZ_ROS_DOMAIN 未设置 / ROS_LOCALHOST_ONLY=1 ⇒ warn |
| `format_env_line()` | 97 | 启动横幅一行 |
| `_signal_handler_options()` | 114 | rclpy SignalHandlerOptions.NO；取不到返回 None（= 不传该参数） |
| `_qos(spec)` | 134 | QosSpec → rclpy QoSProfile |
| `RosStats` | 148 | 计数（odom_msgs/imu_msgs/vision_msgs/.../commands_sent/zero_commands/...） |

**`RosBridge` 方法**（`ros_io.py:170-637`）：

| 方法 | 行 | 签名 | 行为 |
|---|---|---|---|
| `__init__` | 173 | (node_name="dzbrain", *, log=print) | 惰性 rclpy.init（关默认信号处理）；建 _lock/_pose/_imu/_vision/_referee/_drone_inbox 等缓存；_create_subs()+_create_pubs() |
| `_create_subs` | 221 | | 12 条订阅，回调只解码+写缓存+计数 |
| `on_odom` | 229 | | ct.odom_to_pose(...)；写 _pose/_pose_ts；odom_msgs++ |
| `on_vision` | 249 | | decode_offset_center；obs is None ⇒ return（绝不猜字段） |
| `on_referee_hp` | 284 | | RefereeTracker.feed；delta.hits → _hit_pending；self_hits → stats |
| `on_hit / on_ammo` | 267/276 | | **兜底源**：一旦 _referee.valid_seen ⇒ 彻底忽略（否则会与 hp 解析双计命中） |
| `_create_pubs` | 351 | | 9 条发布者 |
| `pose()/pose_age()` | 378/382 | | 加锁读 |
| `vision(max_age_s=0.5)` | 394 | | 过期观测（>0.5 s）返回 None |
| `take_hits()` | 421 | | 取走并清零 _hit_pending |
| `take_drone_messages(max_items=16)` | 449 | | 取走并删除 |
| `publish_command(linear_x, steer_deg)` | 464 | | pursuitAngle Twist；commands_sent++ |
| `publish_brake()` | 474 | | **零 Twist = 刹车**；上下文已死时**必须抛异常**（修复点 C：过去 _safe_call 把 RCLError 吞成 None、照样打「已发零速指令」⇒ 唯一真刹手段静默失效） |
| `publish_fire_gate(allow)` | 495 | | 新话题 + 历史 stop_signal 同步发一份 |
| `publish_fire_ready(ready)` | 504 | | **两键开火第二把钥匙**：fire_gate←MissionTask（任务级）、fire_ready←AimTask（瞄准级）；云台补丁只在两者都为 1 时点亮激光 |
| `publish_lead(lead_px)` | 535 | | 提前量（px） |
| `spin_async(threads=4)` | 545 | | MultiThreadedExecutor + 独立 spin 线程（daemon） |
| `shutdown(*, brake=True) -> int` | 556 | | 默认先发零指令；上下文已死则 rc=1 且跳过（不再刷 RCLError）；executor/node/rclpy 逐级 try/except |
| `topic_counts()` | 588 | | watched 话题的 publishers/subscribers；异常安全（-1） |
| `subscription_topics()` | 604 | | 当前真实创建的订阅话题名 |
| `summary()` | 617 | | 一次性读锁内取值（注释：持锁内不得再调 pose_age() 等也要锁的访问器 ⇒ 死锁 bug 曾潜伏） |

调用方：`main.run_live`（`main.py:2010`）；被调用方：tests/test_contract.py、tests/fakes.py（FakeBridge 替身）。


---

### §2.6 `route/model.py` / `route/parser.py` —— 路线数据模型与 token 语法

#### 2.6.1 `route/model.py`

**定位**：用**不可变对象**替代旧代码的 5 个平行数组（path_points/shoot_flags/scan_flags/reverse_flags/shoot_groups + 每点四元数，共 6 个数组靠手工同步，插入一个接近段要在 5 处 insert，漏一个就静默错位）。本模块把「一个航点」收敛成一个 frozen dataclass，路线本身不可变；拼接/改写用 `RouteBuilder` 生成新的 Route（写时复制，线程安全）。

**`Waypoint`**（`model.py:20-60`）字段：

| 字段 | 行 | 类型 | 语义 |
|---|---|---|---|
| `block` | 24 | str | 格名，如 "K9" |
| `x, y` | 25-26 | float | 米制坐标（格心） |
| `yaw` | 27 | float | **到达该点时应有的朝向**（离开段方向 / 末点终朝向） |
| `reverse` | 28 | bool | R: 前缀 —— 该段倒车到位 |
| `shoot` | 29 | bool | * 后缀 —— 射击停站 |
| `scan` | 30 | bool | :SCAN_STOP —— 纯扫描停站 |
| `speed_scale` | 31 | float | 该段限速系数（1.0=巡航） |
| `dwell_s` | 32 | float | 到位后停留秒数 |
| `group` | 33 | str|None | SHOOT@ 候选链组号 |
| `branch` | 34 | str|None | 分叉标签 |
| `token` | 35 | str | 原始 token |
| `pivot` | 40 | bool | :PIVOT 后缀 —— **折返点声明**。★ 纯声明，不是指令：pursuit 与 tasks/tracker.py **不读这个字段**，PIVOT 仍由 pursuit.path.corner_turn_deg 与 pursuit.pivot_angle_deg 比较自动触发；对 is_stop_point 无影响 |

方法：`at_block(cls, block, *, yaw=None, **kw)`（44）、`is_stop_point`（50）、`distance_to(x,y)`（53）、`with_yaw(yaw)`（56）、`heading_to(other)`（59）。

**`Route`**（`model.py:63-136`）：`waypoints: tuple` / `name` / `crossing: dict`（如 {"lower":"K9","upper":"K4"}）。
方法：`__len__/__iter__/__getitem__/__bool__`（73-83）、`blocks()`（86）、`tokens()`（89）、`start/end`（93/97）、`stop_points()`（100）、`shoot_points()`（103）、`reverse_segments()`（106，返回倒车段**起点索引**集合）、`pivot_points()`（111）、`pivot_blocks()`（115）、`path_length()`（119）、`describe(limit=40)`（125）、`as_list()`（132）、`slice(start,end=None)`（135）。

**`RouteBuilder`**（`model.py:139-195`）：`__init__(name="")`（142）、`add(wp)`（147）、`add_block(block, **kw)`（151）、`extend(wps)`（154）、`add_route(route)`（159）、`crossing(half, block)`（162）、`__len__`（166）、`build(*, fix_final_yaw=None, drop_consecutive_duplicates=True)`（169）。

**build 收尾逻辑**（`model.py:169-195`）：
1. 去重相邻同格点（保留信息更丰富的那个；shoot/scan 取并集；pivot 取并集；reverse 不同绝不合并）；
2. 填中间点朝向 = 离开段方向；
3. 末点朝向 = fix_final_yaw（否则 w.yaw）；
4. 返回新 Route。

**`make_route(waypoints, name="", crossing=None)`**（`model.py:198-205`）：便捷构造，朝向按「离开段方向」补齐。

调用方：`parser`（RouteBuilder/make_route）、`planner._to_route`、`pursuit.path.Path`。被调用方：tests/test_route.py。


#### 2.6.2 `route/parser.py` —— token 语法

**语法总表**（`parser.py:9-19`，兼容旧协议）：

| 写法 | 含义 | 旧协议行号 |
|---|---|---|
| `K9` | 普通航点 | 1192, 1204 |
| `D3*` | 射击停站（到达后停车、找靶、稳定后放行开火） | 1182-1183, 1206 |
| `D3:SHOOT_STOP / D3:SHOOT` | 等价于 D3*（内部归一成 *） | 1174-1178 |
| `F2:SCAN_STOP / F2:SCAN` | 纯扫描停站（不射击） | 1179-1180 |
| `F8:PIVOT` | ★ 折返点**声明**（该顶点车头需转过 ≥135°，循迹层自动 PIVOT）—— 不是停车指令 | 新增 |
| `R:B5` | 倒车到位（该段线速度为负，车尾朝目标摆） | 1167-1169, 1208 |
| `R:B5*` | 倒车 + 射击（唯一合法顺序：R: 在前、* 在后） | 注释 1163-1166 |
| `SHOOT@G11` | **指令而非坐标**：调 shoot_planner 算候选链并整链插入，共用一个组号 | 1136-1161 |
| `N5,y / N5,x` | 终点朝向后缀，**只有最后一个 token 允许带** | 1108-1117；校验 375-380 |

**常量**（`parser.py:56-77`）：
- `REVERSE_PREFIX = "R:"`、`SHOOT_SUFFIX = "*"`、`SHOOT_AT_PREFIX = "SHOOT@"`
- `SHOOT_SUFFIXES = frozenset({":SHOOT_STOP",":SHOOT"})`
- `SCAN_SUFFIXES = frozenset({":SCAN_STOP",":SCAN"})`、`SCAN_CANON = ":SCAN"`
- `PIVOT_SUFFIX = ":PIVOT"`（回写顺序：R: → 格名 → * → :SCAN → :PIVOT → ,x|,y）
- `HEADING_SUFFIXES = frozenset({",X",",Y"})`、`_HEADING_ALT = (":Y",":X")`
- `_HEAD_BLOCK_RE = re.compile(r"^([A-N])([0-9]{1,3})")`、`_BARE_GAP_RE = re.compile(r"[,\u005cs;]+")`
- `_CANON_TAIL_RE`（83-90）：一次正则匹配全部后缀，**只接受规范序** —— 乱序写法（如 F2:SCAN*）仍然报错，不会被「先剥谁」的顺序悄悄修正成合法写法。

**函数**：

| 函数 | 行 | 签名 | 返回 | 副作用 |
|---|---|---|---|---|
| `normalize_token` | 96 | (token) -> str | 规范化 token | 非法抛 ValueError；不判越界 |
| `_normalize_shoot_directive` | 161 | (token) -> str | SHOOT@<TARGET> | |
| `_split_block_suffix` | 170 | (block, raw) -> (block, heading|None) | | |
| `parse_token` | 183 | (token) -> dict | {block,reverse,shoot,scan,heading,group,raw,pivot,shoot_at,target,token} | 非法抛 ValueError |
| `_looks_like_file` | 246 | (spec) -> bool | | |
| `_read_token_file` | 256 | (path) -> list[str] | 整行一个 token；# 注释与空行跳过 | |
| `_as_token_list` | 274 | (tokens) -> list[str] | 支持 str（含文件路径）/list/tuple/Iterable | |
| `_call_shoot_planner` | 289 | (planner, target) -> list[str] | 返回值规整成 list[str] | |
| `parse_mission` | 306 | (tokens, *, shoot_planner=None, name="mission") -> Route | Route | 非法 token/越界/中间点带朝向 ⇒ ValueError |
| `_dedup_waypoints` | 383 | (route) -> Route | 相邻同格且 reverse 相同 ⇒ 合并（shoot/scan/pivot 取并集；reverse 不同绝不合并） | |
| `detect_crossing` | 409 | (route) -> dict | {"lower":"K9","upper":"K4"}（K 列、行号分半场） | |
| `parse_reset` | 441 | (spec) -> (block, yaw) | "N10,y"→("N10",π/2)；"N10"/"N10,x"→("N10",0.0) | 非法抛 ValueError |
| `format_route` | 461 | (route) -> list[str] | token 列表（必须能往返） | |

**normalize_token 关键逻辑**（`parser.py:96-158`）：
1. 非字符串/空 ⇒ ValueError；
2. 剥 R: 前缀（重复前缀 / 剥后无格名 ⇒ ValueError）；
3. 一次 _CANON_TAIL_RE 匹配全部后缀（shoot/scan/pivot/heading）；
4. head 去空格；必须以列字母开头（否则拦 *B5 这类写法）；
5. 折掉组分隔符 G:10；
6. 非法残留 ⇒ ValueError；
7. 按规范序重建：head.upper() + (R:) + (*|:SCAN) + (:PIVOT) + (,x|,y)。

**parse_mission 关键逻辑**（`parser.py:306-380`）：
- `shoot_planner=None` ⇒ 默认 lambda target: [target]（靶标自身一格）；
- 遇 SHOOT@ ⇒ 调 planner 拿候选链，**整链插入**（先算整数索引区间再切片拼接，天然不会错位），共享同一 group；
- 普通点遇任何普通点 ⇒ 封口 group/insert_at；
- 朝向后缀只允许最后一个 token（否则 ValueError）；
- **调用 build(drop_consecutive_duplicates=False)**：因为 RouteBuilder 内置去重是「二选一」，会把 B5* + B5:SCAN_STOP 合并成只剩 shoot 或只剩 scan —— 违反「标记取并集」约定；
- 最后 _dedup_waypoints + detect_crossing。

**detect_crossing**（`parser.py:409-435`）：路线里落在 K 列（COL_LETTERS.index("K")=10）的格，按行号分半场（row_idx>=6 ⇒ lower；否则 upper）；返回 {half: 第一个命中格}。这样 validate.py 的 R008/R009 不需要另外喂配置就能定位穿洞点。

调用方：main.load_and_check（`main.py:190-192`）、sim_cli._resolve_route、tests/test_route.py。被调用方：tests/*、tasks/*。


---

### §2.7 `route/validate.py` / `route/physics.py` —— 出发前校验与物理几何

#### 2.7.1 `route/validate.py` —— R001~R014 出发前路线校验

**定位**：纯函数、无 ROS 依赖、**只报问题不改写路线**（`validate.py:2`）。血统：旧 path_executor.validate_mission_blocks（path_executor.py:342-478，只报错不改写）+ 用户侧 tools/route_check.py。每个检查一个**稳定编号** R001~R014，被测试与文档引用，**不得改号**；每条 msg 都带证据数值。

**常量**（`validate.py:35-61`）：

| 常量 | 值 | 说明 |
|---|---|---|
| `CROSS_COLUMN = "K"` | 35 | 穿洞列 |
| `CROSS_LOWER_ROWS = (7, 11)` / `CROSS_UPPER_ROWS = (1, 6)` | 38/40 | 上/下半场穿洞行 |
| `U_TURN_COLUMNS = ("A","B","C")` | 45 | U 形绕行必须经过的列 |
| `LOWER_HALF_ROWS = (7, 11)` / `UPPER_HALF_ROWS = (1, 6)` | 47/48 | 半场行号 |
| `MAX_SEGMENT_BLOCKS = 2.5` / `MIN_SEGMENT_M = 0.02` | 51/52 | R011 段长上/下限 |
| `MAX_CROSS_ANGLE_DEG = 10.0` | 54 | R009 入洞段夹角上限 |
| `MAX_REVERSE_RATIO = 0.30` | 56 | R014 倒车段占比上限 |
| `DEFAULT_SHOOT_DISTANCE_MIN_M = 0.20` / `DEFAULT_SHOOT_DISTANCE_MAX_M = 0.45` | 58/59 | R006/R013 射击窗口缺省 |
| `EPS = 1e-9` | 61 | 数值比较容差 |

**Config 键名**（`validate.py:66-70`）：_KEY_SHOOT_MIN=path_executor.shoot_distance_min、_KEY_SHOOT_MAX=path_executor.shoot_distance_max、_KEY_CROSS_LOWER=route.cross_lower_candidates、_KEY_CROSS_UPPER=route.cross_upper_candidates、_KEY_BRIDGE_BLOCKS=pursuit.bridge_blocks。

**模块级函数**：

| 函数 | 行 | 签名 | 说明 |
|---|---|---|---|
| `_fallback_cfg` | 75 | () -> Config | 无外部 Config 时用 SCHEMA 显式默认值兜底（绝不硬编码第二份默认值）；模块级单例缓存 _FALLBACK_CFG |
| `_cfg_get` | 83 | (cfg, key, fallback=None) | 安全取配置：缺失/类型不符/缺 PyYAML 都退回 fallback，绝不因少一个键崩掉预检 |
| `_window` | 93 | (cfg) -> (lo, hi) | 射击窗口；lo>hi 则交换 |
| `_as_list` | 101 | (value) -> list[str] | str 按 ; , 切；list/tuple/set/frozenset 转 str |
| `_cross_rows` | 111 | (cfg, cross_candidates=None) -> (lower, upper) | 穿洞候选行：优先从候选格名集合反推行号（与现场 YAML 永远同步）；显式覆盖时两边都要覆盖 |
| `_map_section` | 148 | (map_cfg) -> dict | 取 map: 段；接受原始 YAML / 展平 dict / Config |
| `_cells` | 171 | (merged, key) -> list[str] | 大写去空格 |
| `_rc` | 178 | (block) -> (行号 1-based, 列索引 0-based) | |
| `_sign` | 184 | (value) -> int | |
| `_blocks` | 188 | (route) -> list[str] | 兼容 Route / 航点序列 / 字符串序列 |
| `_barrier_crossing` | 201 | (a, b) -> (违规?, 交点列索引, 交点行坐标) | 判据只有一条：**两个端点的行号必须分居第 6 行两侧**（(6-r1)*(6-r2) < 0）；只在第 6/7 行内横向移动（B6→D6→C6）**不算跨越** |
| `_crossing_row_of_cell` | 227 | (cell) -> float|None | 由穿洞格直接给行号 |
| `_is_cross` | 238 | (a, b) -> bool | 线段 a→b 是否朝该列推进并抵达/越过它 |
| `_crossing_row` | 275 | (a, b) -> float|None | 线段在 K 列那一刻的行号 |
| `_crossing_cell` | 297 | (a, b) -> str|None | 进入 K 列时落在该列的那一格 |
| `_crossing_at_column` | 312 | (a, b) -> (row, angle_in, angle_out, col_diff) | 只对真正的穿越返回非 None |
| `crossing_entries` | 345 | (blocks) -> [(a, b, (row, in, out, col_diff))] | **一次过洞 = 一条记录**（只收起点不在 K 列上的那一段） |
| `validate_route` | 391 | (route, *, map_cfg=None, cfg=None, cross_candidates=None) -> list[Issue] | 主入口，返回 list[Issue]（空 = 可发车） |
| `_euc` | 731 | (wp, block) -> float | 航点到格心欧氏距离 |
| `_stop_ok` | 736 | (wp, target, lo, hi) -> bool | 距离落在 [lo, hi] |
| `_arrives_reverse` | 742 | (route, idx) -> bool | 第 idx 航点是否倒车到位 |
| `_total_length` | 748 | (blocks) -> float | |
| `summary` | 760 | (issues) -> str | 人类可读汇总（error/warn 计数） |

**`Issue` 类**（`validate.py:378-388`）：frozen level("error"|"warn") / code(R001~R014) / msg；__str__ 带 ❌/⚠️ 前缀。

**_is_cross 判据（2026-10-11 修正，重点）**（`validate.py:238-272`）：
~~~text
① K 列夹在两端之间（含恰好落在端点上）
② 只排除「起点就在 K 列上、而且往东/沿列走」两种情形：
   - K9→L9 假（只是从洞里出来）
   - K9→K10 假（在洞里横着走，由 R009 单独报）
③ 任何斜切 ⇒ 真，但角度 >10° 会被 R009 拦下
~~~
旧判据 min(c1,c2) < k_col <= max(c1,c2) 在一端恰好压在 K 列上时恒假 ⇒ **东侧进洞（L→K）整类漏判**（连 docstring 自举的第一个例子 L9→K9 实测都是 False），且方向不对称。修正后两端点对称。

**validate_route 主流程（按 R 编号）**：
- **R001**（405-409）：点数 <2 ⇒ error，直接返回。
- **R002**（419-453）：格名非法 / token 非法 / token 与格名不一致 ⇒ error；起点与 map.start 不一致 ⇒ warn（里程计开机硬锚在 map.start）。
- **R010**（455-460）：相邻重复航点 ⇒ error（parser 应已合并）。
- **R003**（462-474）：跨越第 6/7 行隔板且交点落在 D~N ⇒ error（只有 A/B/C 列能上下穿行）。
- **R004**（476-483）：航点踩障碍格 ⇒ error。
- **R005**（485-494）：雷覆盖不足（route_set ∩ mines）⇒ error，报文列出漏压雷。
- **R007/R008/R009**（496-647）：把连续相接的穿越段并成一次「过洞」（run）；R009 逐次过洞查夹角（>10° ⇒ error）；R007 每半场恰好 1 次且方向正确（下半场向西 col_diff<0、上半场向东 col_diff>0）；R008 穿洞行不在本半场行段内 ⇒ error。⚠️ R008 的**硬门是行段**（下 7~11 / 上 1~6），候选集合不再当硬门（无人机新鲜情报可纠正过期地图）。
- **R012**（649-655）：路线完全没有经过 A/B/C 列 ⇒ error（没绕过隔板）。
- **R006/R013**（657-687）：靶覆盖与射击窗口。R006 = 存在射击停站且每个靶都被覆盖；R013 = 每个射击停站点对靶的距离都在窗口内（倒车到位时用 hi=inf 判）。
- **R011**（689-711）：段长 < MIN_SEGMENT_M ⇒ error；>2.5 格 ⇒ **汇总成一条 warn**（P0接口约定 §2 要求「R011 至多 1 条 warn」）。
- **R014**（713-726）：倒车段占比 >30% ⇒ warn。

调用方：main.load_and_check（`main.py:193`）、planner.check_route_structure（复用 _crossing_at_column/_barrier_crossing/_euc/_stop_ok）、tools/route_check.py（同口径副本）、tests/test_route.py。


#### 2.7.2 `route/physics.py` —— 物理几何唯一真源

**定位**：纯函数：不读文件、不 import ROS、不产生 Issue（`physics.py:2`）。数值全部来自 config/robot.yaml / core/config.py SCHEMA；本模块只提供**公式**与默认形参，绝不把现场数值烘焙进调用方。

**现场几何表**（`physics.py:6-12`）：

| 物件 | 现场几何 | 判据 |
|---|---|---|
| **车体** | 长 0.28 m × 宽 0.135 m 矩形；另有「车心假设圆」半径 0.035 m | 压雷用 3.5 cm 圆；转弯用扫掠圆（半对角线） |
| **障碍** | 半径 0.1275 m 圆柱，柱心在障碍格格心 | 车心距柱心 ≥ r + 车半宽 + margin（拐角处车半宽取扫掠半径） |
| **雷** | 0.12 × 0.12 m 垫片，中心在雷格格心 | 车心 3.5 cm 圆与垫片**产生过重叠** ⇔ 车心到垫片最近距离 ≤ 0.035 m |
| **挡板** | 第 6/7 行之间、D..N 列，长 11×0.35 = 3.85 m、厚 0.02 m 细条（中心线 x=1.75） | 车心到中心线 ≥ 0.01 + 0.0675 = 0.0775 m |

**三个容易搞错的地方**（`physics.py:14-23`）：
1. 压雷不能用「车心到雷心 ≤3.5 cm」（垫片是方块，贴边压过时车心离雷心可到 0.095 m）⇒ 必须用**点到矩形的最近距离**；
2. 转弯不能用「车半宽」当扫掠半径（车身外包圆半径 = 半对角线 0.1554 m）；
3. 挡板是 2 cm 细条而不是一整格（格带口径会高估占位）。

**函数索引**（含 `__all__`，`physics.py:42-80`）：

| 函数 | 行 | 签名 | 公式/判据 |
|---|---|---|---|
| `point_segment_distance_m` | 89 | (px,py,ax,ay,bx,by) | 投影钳位到 [0,1]；退化线段按点处理 |
| `point_rect_distance_m` | 101 | (px,py,cx,cy,half) | **压雷判据**要用的距离；在矩形内 = 0 |
| `segment_rect_distance_m` | 114 | (ax,ay,bx,by,cx,cy,half) | ① 相交（Liang–Barsky）⇒ 0；② 否则 min(4 角到线段, 两端点到矩形) = 精确解 |
| `_segment_rect_intersects` | 133 | 同上 | Liang–Barsky 平板裁剪 |
| `vehicle_half_diagonal_m` | 162 | (length_m=0.28, width_m=0.135) | hypot(L/2, W/2) = 0.15542 m |
| `sweep_radius` | 170 | (*, length_m, width_m, wheelbase_m=0.157, max_steer_deg=30.0) -> (扫掠半径, 最小转弯半径) | R = wheelbase/tan(max_steer) = 0.2719 m |
| `mine_clearance_m` | 190 | (px,py,mine_x,mine_y, *, mine_pad_m=0.12) | point_rect_distance_m(..., mine_pad/2) |
| `mine_clearance_segment_m` | 196 | 同上（段版） | segment_rect_distance_m |
| `mine_pressed` | 203 | (px,py,mine_x,mine_y, *, coverage_radius_m=0.035, mine_pad_m=0.12) -> bool | clearance <= coverage_radius + 1e-12 |
| `obstacle_clearance_point_m` | 216 | (px,py,ox,oy) | hypot |
| `obstacle_clearance_segment_m` | 221 | (ax,ay,bx,by,ox,oy) | point_segment_distance_m(ox,oy,...) |
| `required_obstacle_center_distance_m` | 227 | (*, obstacle_radius_m=0.1275, body_radius_m=0.0675, margin_m=0.02) | r + body_r + margin（直段 0.215 / 拐角 0.3029） |
| `parse_col_span` | 241 | (spec) -> (起列, 止列) | 支持 "D..N" / "D-N" / "D" |
| `barrier_centerline` | 257 | (*, col_from="D", col_to="N", row_hi=6, row_lo=7) -> (x, y0, y1) | x=1.75；y 覆盖 0.0~3.85 |
| `barrier_distance_point_m` | 273 | (px,py, *, x=1.75, y0=0.0, y1=3.85) | point_segment_distance_m |
| `barrier_distance_segment_m` | 279 | 段版 | _seg_seg_distance |
| `required_barrier_distance_m` | 286 | (*, barrier_width_m=0.02, car_width_m=0.135) | barrier_width/2 + car_width/2 = 0.0775 m（2026-10-11 澄清：挡板在 K 列处也连续，无通口，**不需要桥洞豁免**） |
| `k_cells_sealed_along_column` | 302 | () -> bool | True（解释既有判据的物理根据） |
| `board_half_m` | 319 | () -> float | CELL_M/2 = 0.175 m（靶板 35×35 cm，格心对齐 ⇒ 格级挡块恰好物理精确） |
| `required_target_body_clearance_m` | 329 | (*, length_m, width_m, at_corner=False) | 直段 = 车半宽 0.0675；拐角 = 半对角线 0.1554 |
| `target_body_overlap` | 341 | (clearance, *, ...) -> bool | clearance < required - 1e-12 |
| `body_overlaps_rect_centered` | 348 | (px,py,cx,cy,*, half, ...) -> bool | 保守粗判（车体当圆） |
| `_seg_seg_distance` | 359 | (ax,ay,bx,by,cx,cy,dx,dy) | 相交 = 0；否则 4 端点对另一线段取最小 |
| `_segments_intersect` | 371 | 同上 | cross 积判跨 |
| `segment_outside_band` | 382 | (ax,ay,bx,by,*, band_y, band_half) | 抠掉 |y−band_y| ≤ band_half 的带内部分（桥洞豁免；**不能两端点有一个在带内就整段豁免**） |
| `body_center` | 435 | (px,py,yaw,*, offset_m=0.0) | 沿车头方向前移 offset_m |
| `body_corners` | 441 | (px,py,yaw,*, length_m=0.28, width_m=0.135, offset_m=0.0) | 车体矩形 4 角（逆时针） |
| `corner_poly` | 457 | (cx,cy,half_x,half_y) | 轴对齐矩形 4 角 |
| `polygon_overlap_depth` | 483 | (a, b) -> float | SAT/MTV；>0 = 真重叠 |
| `polygon_distance_m` | 500 | (a, b) -> float | 重叠 = 0；顶点-边对取最小 |
| `body_obstacle_clearance_m` | 530 | (px,py,yaw,ox,oy,*, ...) | 车体矩形表面 → 圆柱表面的间隙（**严格**，实测比老口径多抓 253 拍真重叠） |
| `body_barrier_clearance_m` | 549 | (px,py,yaw,*, ...) | 车体矩形表面 → 挡板细条表面（判据是「两个矩形最近距离」，不是「车心到中心线」） |
| `body_rect_clearance_m` | 567 | (px,py,yaw,*, rect_cx, rect_cy, rect_half_x, rect_half_y, ...) | 车体矩形表面 → 任意轴对齐矩形实体 |
| `bridge_entry_walls` | 606 | (block, redundancy_m) -> (北壁, 南壁) | 每段 = (x0,y0,x1,y1)；**虚拟墙，现实不存在**（设计冗余） |
| `body_bridge_wall_clearance_m` | 637 | (px,py,yaw,walls,*, ...) | 含**包含检测**（墙整段在车体内 ⇒ 0），判据必须写成「间隙 > 0」 |

**关键实现逐行**：
~~~python
def point_rect_distance_m(px, py, cx, cy, half):   # physics.py:101-111
    ox = abs(px - cx) - half
    oy = abs(py - cy) - half
    if ox <= 0.0 and oy <= 0.0:
        return 0.0
    return math.hypot(max(ox, 0.0), max(oy, 0.0))

def vehicle_half_diagonal_m(length_m=0.28, width_m=0.135):   # physics.py:162-167
    return math.hypot(length_m / 2.0, width_m / 2.0)

def sweep_radius(*, length_m=0.28, width_m=0.135, wheelbase_m=0.157, max_steer_deg=30.0):
    # physics.py:170-184
    if not (0.0 < max_steer_deg < 90.0): raise ValueError(...)
    return vehicle_half_diagonal_m(length_m, width_m), wheelbase_m / math.tan(math.radians(max_steer_deg))

def required_obstacle_center_distance_m(*, obstacle_radius_m=0.1275, body_radius_m=0.0675, margin_m=0.02):
    # physics.py:227-235
    return obstacle_radius_m + body_radius_m + margin_m
~~~

**参考点偏移感知的车体足迹**（`physics.py:413-432`）：背景是现场卷尺实测（后轴中心→车尾 6.5 cm、→车头 21.5 cm、总长 28 cm）⇒ 车体几何中心在参考点（= 后轴中心 = 里程计/仿真积分点）**前方 7.5 cm**。真源 = config/robot.yaml: vehicle.ref_offset_m。⚠️ **兼容性铁律**：offset_m 默认 DEFAULT_REF_OFFSET_M = 0.0 ⇒ 不读配置的老调用方（planner/sim/route_check/既有测试）行为**逐字节不变**；只有显式传值才进入「真实几何」口径。

**桥洞进洞侧冗余墙**（`physics.py:580-678`）：
- 用户设计规格（2026-10-16）：在桥洞的**进洞侧**加一段 1/4 格 = 0.0875 m 的虚拟墙，逼规划器让车「进洞前摆正、不碰桥室侧壁」。
- **现实不存在**（桥室外侧没有墙），是设计冗余；规划器必须当真实墙对待，但**不要**加进 tools/route_check.py 的用户侧判据。
- 进洞方向真源 = config/robot.yaml 的 bridge_order（两处取值必须在同一次改动内一致）：下半场 row 7~11 → L→K→J（向西穿，y 小侧进洞）；上半场 row 1~6 → J→K→L（向东穿，y 大侧进洞）。
- `BRIDGE_ENTRY_LOWER_ROW_IDX = 6`（row_idx >= 6 ⇒ 下半场）。
- **★ 包含检测是实测补的真 bug，不要删**：_seg_seg_distance 的合同只在「两段边相交」时给 0；墙段完全落在车体矩形**内部**时不与任何一条边相交 ⇒ 退化成「端点↔边」距离 ⇒ **错报正间隙**（实测 K9 进洞线 y=1.05、车心 x = cx+0.13 时错报 0.0225 m，真值 0）。判据：墙段任一端点或中点落在车身多边形内 ⇒ 直接返回 0。
- **⚠️ 判据必须写成「间隙 > 0」**：本模块距离口径是「重叠 = 0」，返回值恒 ≥ 0 ⇒ 写成 ≥ 0 会**恒真**、判据形同虚设（落地时实测确认）。

调用方：planner.py（Grid/_TSGrid/planner 判据）、tools/sim.py（FieldObs.observe）、tools/route_check.py（等价副本，由 test_physics.py 对拍钉住两份实现不许分叉）、tests/test_physics.py。


---

### §2.8 `route/planner.py` —— A* 规划器与 tracker-safe 引擎（4849 行，最大模块）

**定位**：换一张 map.yaml 就能自动生成路线（`planner.py:2-7`）。只读 map.yaml 的六个字段（start/end/mines/obstacles/targets/bridges）+ Config 里的几个约束值，**不复制任何一份地图要素**。把 U 形结构**显式建模**，不指望 A* 自己摸出来。

#### 2.8.1 常量区（`planner.py:149-283`）

| 常量 | 行 | 值 | 说明 |
|---|---|---|---|
| `DIRS` | 155 | ((-1,0),(0,-1),(1,0),(0,1)) | 朝向索引 0=北 1=西 2=南 3=东；与 tools/global_planner.py 逐字一致 |
| `TURN_LEFT / TURN_RIGHT` | 156 | -1 / +1 | |
| `HEAD_NAMES` | 157 | ("北(行号减小)","西(列号减小)","南(行号增大)","东(列号增大)") | |
| `START_HEADING` | 170 | 1 | **设计常量**：起点朝向 = 西 = 列号减小 = 地图 +y（图左 / K 列方向）。不再由 mission.reset 反推 |
| `START_HEADING_YAW` | 171 | geo.HEADING_Y | 同一朝向的地图 yaw |
| `ACT_START/FORWARD/BACKWARD/LEFT/RIGHT` | 174 | 9/0/1/2/3 | |
| `COST_FORWARD / BACKWARD / TURN` | 177-179 | 1.0 / 2.2 / 1.4 | ⚠️ 与 tools/global_planner.py:88-89（1.6/1.9）存在数值分叉，属历史口径 |
| `OBSTACLE_DANGER_RADIUS / W1 / W2` | 188-190 | 2 / 0.9 / 0.4 | 障碍安全势场（切比雪夫 1/2 两档） |
| `BARRIER_NEAR_COST` | 192 | 0.15 | 隔板相邻行额外罚 |
| `UTURN_REVERSE_DEG` | 195 | 150.0 | 编译阶段倒车改写阈值 |
| `PIVOT_MARK_DEG` | 201 | 135.0 | 折返点（:PIVOT）判定阈值（= pursuit.pivot_angle_deg） |
| `PIVOT_PENALTY` | 207 | 5.0 | TSP/顺序搜索折返惩罚（等效代价单位） |
| `MINE_PRESS_TOL_M` | 215 | 0.03 | 雷心必须被车心压过（米） |
| `MINE_TURN_PENALTY` | 220 | 2.0 | 雷格不得作拐点的惩罚（90° 记 1 份、180° 记 8 份） |
| `BARRIER_TURN_PENALTY` | 230 | 8.0 | 隔板相邻行死胡同式 90° 转向惩罚；**缺省 0.0 = 关闭 = 与历史逐 token 一致** |
| `DOOR_TURN_PENALTY` | 241 | 6.0 | 过门转弯点惩罚（几何推导取代固定 0.5 m） |
| `EDGE_MINE_TURN_MIN_CELLS / PENALTY` | 246/248 | 2 / 6.0 | 约束②：最外行/列雷转弯距离 |
| `DEFAULT_MAX_SECONDS` | 258 | 2.0 | 单次规划时间预算（0.8→2.0 修正，门禁负载下 flake） |
| `MAX_TOUR_ROUNDS / MAX_2OPT_PASSES` | 260/262 | 3 / 6 | |
| `MAX_HOP_BLOCKS` | 265 | 2 | 相邻航点最大间距（格）；>2.5 格 R011 告警 |
| `ROUTE_CHECK_TOL_LO / TOL_HI` | 272/273 | 0.03 / 0.06 | tools/route_check.py 的实际接受窗口（0.17~0.51 m） |
| `OBJECTIVE_LEGACY / TASKS / OBJECTIVES` | 281-283 | "legacy" / "tasks" | 目标函数口径；**默认 legacy = 历史行为逐字节不变** |

**起始航向设计常量说明**（`planner.py:159-171`）：现场约定每次发车车都摆在起点格（现场图 = N10）、车头水平朝图左（列号减小 = +y）。摆位由现场用户保证，判据里不需要、也不允许有「摆错就拒绝发车」的校验。规划器把它当**设计输入**：不按 mission.reset 反推、不在「朝西不可行」时把起点朝向改成朝 X+。

#### 2.8.2 `MapSpec` 与 `load_map`

`MapSpec`（`planner.py:293-307`）：frozen name/start/end/mines/obstacles/targets/bridges/barrier_col_from/barrier_row_hi/barrier_row_lo/source。

**`load_map(map_cfg, *, source="") -> MapSpec`**（`planner.py:329-364`）：缺失/非法一律抛 PlannerError，**绝不猜一个默认起点**。barrier_col_from 非法 ⇒ PlannerError；barrier_row_hi/lo 用 int 转换失败则兜底 geo 常量。

#### 2.8.3 `Grid` —— 通行栅格（`planner.py:370-523`）

**构造**：`Grid(spec, bridge_rows=(), *, allow_reverse=True, barrier_turn_margin_m=0.0)`。
- self.blocked = 障碍格 + **靶板格**（用户 2026-10-11 裁定：所有靶子都是占用两格的跨格靶板，是**物理实体**，压过去就是撞板；向后兼容：现图拉黑靶格输出逐 token 相同）。
- self.target_cells = 靶板格集合（供日志/诊断区分）。
- self.hazard_cells = **只含障碍**（势场源）。★ 为什么必须分开：靶板要贴着打，给靶板加「离远点」势场是反着用力（实测会把射击停站从 G10 挤到 F10 并让现图路线越界 196 拍）。
- self.barrier_col / barrier_hi / barrier_lo / bridge_rows。
- self.barrier_turn_margin_m = max(0.0, ...)：**负值不再「关闭判据」**（旧文档的暗门已封，构造口钳位为 0.0）。

**方法**：

| 方法 | 行 | 签名 | 行为 |
|---|---|---|---|
| `in_bounds` | 436 | (row, col) -> bool | 1..MAP_ROWS, 0..MAP_COLS-1 |
| `passable` | 439 | (row, col) -> bool | in_bounds and not blocked |
| `barrier_cross_ok` | 442 | (row_from, row_to, col) -> bool | 只有列索引 < barrier_col_from 能过隔板 |
| `zone_of` | 448 | (row) -> int | 0=上半场, 1=下半场 |
| `move_ok` | 452 | (row, col, nrow, ncol) -> bool | 四条判据：可通行 / 不非法跨隔板 / K 列内不上下挪格 / 只在桥洞行进出 K 列。★ 前置断言：**每一步必须是正交相邻一格**（跳格会静默放过中间格上的桥墩/隔板） |
| `danger` | 489 | (row, col) -> float | |
| `_build_danger` | 492 | () -> list[list[float]] | 障碍势场（只由 hazard_cells 生成）；隔板相邻行 +0.15 |

#### 2.8.4 `_barrier_turn_cell` —— 隔板相邻行死胡同式 90° 转向（`planner.py:536-611`）

**判据**：命中充要形态 = 车从「另一侧那一行」（第 8 行）开进隔板紧邻行（第 7 行），在行内做侧向 90° 转向。
逐条：① 不在 K 列；② row == barrier_lo（第 7 行）；③ barrier_col > 0 且 col >= barrier_col（**A/B/C 是过板缺口，那里行内转向物理上不可能蹭板**）；④ 来向与去向垂直；⑤ 定向来向 = 从 barrier_lo±1 那一行进来；⑥ 半格 0.175 m 装不下侧向 90° 转向的摆动（0.5 * BLOCK_SIZE >= margin）⇒ 一律拦下；显式 margin_m < 0 才关闭，但构造口已钳到 0 ⇒ **实际不可关闭**。

**实测归因**：base_v015（现场手工路线 @0.15）实测 353 拍触线 = 2 个事件，全部落在该形态（N7 178 拍、H7 175 拍）。同行的另两类拐角（J7 西→北、E7 西→北）实测都不触线 —— 所以判据必须区分「来向」。

#### 2.8.5 门/障碍/雷的几何判据（`planner.py:614-783`）

| 函数 | 行 | 说明 |
|---|---|---|
| `_door_turn_margin_m`（旧，死代码） | 614 | 早期版本（返回可用余量）；**被 695 行同名函数覆盖，是死代码** |
| `_phys_params` | 632 | 从 Config 读物理几何参数（length/width/coverage_radius/obstacle_radius/mine_pad/barrier_width/barrier_cols/obstacle_margin/wheelbase/max_steer/bridge_entry_redundancy_m/bridge_approach_align_penalty_m）；None ⇒ physics.py 默认形参 |
| `_door_turn_required_margin_m` | 664 | 过门转弯所需最小余量 = max(w/2, R·sinθ + w/2·cosθ)；现场 90° ⇒ 0.2719 m（**几何化后 C 列其实合规**） |
| `_turn_deg_from_dirs` | 686 | 行进方位间转角（0/90/180/90） |
| `_door_turn_margin_m`（现用） | 695 | 该格作过门转弯点的**可用余量** = (barrier_col − col) × 0.35 |
| `_obstacle_turn_clearance_m` | 712 | 转弯点到最近障碍柱心距离（判据 ≥ 所需中心距） |
| `_required_obstacle_clearance_m` | 734 | 直段车身半径 / 拐角扫掠半径两档 |
| `_mine_pressed_at` | 745 | 车心在 cell 时是否压到 mine_pad |
| `_edge_mine_turn_gap` | 760 | 转弯点到最外行/列雷的最小曼哈顿格距 |
| `_edge_mine_cells` | 775 | 最外行/列的雷格集合 |

#### 2.8.6 `_moves` / `flood` / `_reconstruct` / `_flood_value`（`planner.py:786-937`）

**`_moves(grid, row, col, head, prev_head=-1, prev_valid=False, barrier_turn_penalty=0.0)`**（`planner.py:786-817`）
- 产出 `((row,col,head,prev_head,prev_valid), cost, action)` 的阿克曼可执行动作集。
- allow_reverse=False 时**不产出倒车动作**：180° 折返 = 正向开进死格 + 原地转向（2×COST_TURN）后正向开出（顶点处由 TrackerTask PIVOT）。
- 前进 ⇒ COST_FORWARD + grid.danger；倒车 ⇒ COST_BACKWARD + danger；转向 ⇒ COST_TURN + barrier_turn_penalty（命中 _barrier_turn_cell 时）。

**`flood(grid, source, barrier_turn_penalty=0.0)`**（`planner.py:820-871`）
- 从 (row,col,heading) 出发的**一次 Dijkstra 洪泛**（不是逐对 A*：代价矩阵要「一个点到其它所有点 × 四个到达朝向」，逐对 A* 是 2700+ 次搜索，直接吃光预算；状态空间 11×14×4=616，一次洪泛即可且仍最优）。
- 返回 (dist, came, actions, settled, rank)。rank = 各状态的**收紧次序**（第一次拿到有限距离时的序号）—— **确定性 tie-break**。
- ★ 为什么要 rank：状态多带 prev_head 后，同格同朝向会出现多个同代价状态；纯字典迭代序非确定性 ⇒ 会选出「代价一样但几何不同」的腿（本轮实测踩过：默认参数下路线突然在下半场穿 K 列两次）。

**`_reconstruct(dist, came, actions, goal, rank=None)`**（`planner.py:874-902`）
- 在 dist 里找「位置+朝向匹配 goal」的最优状态，按 (代价, rank) 全序比较（**容差比较**：多状态投影下同一 (格,朝向) 会出现相差 1e-15 的等价代价，严格 < 会让「谁更小」变成噪声判决）。
- 回传路径元组仍是 (row,col,head,action)（调用方不需要知道新增维度）。

**`_flood_value`**（`planner.py:905-919`）：与 _reconstruct 用同一条取法（含同一条 tie-break）—— 否则 _Tour.cost()（读 dist）与 _Tour.path()（回溯）会挑到不同的腿。

#### 2.8.7 判据复用（`planner.py:943-1000`）

| 函数 | 行 | 说明 |
|---|---|---|
| `stop_distance_m` | 943 | 两格中心欧氏距离；**复用 validate._euc**（与 tools/shoot_planner.dist_m、route_check 同算法） |
| `turn_angle_deg` | 952 | 折返角（度）：进入段与离开段夹角 |
| `plate_center_m` | 964 | 靶板中心 = 各靶格中心的算术中点（旧口径取「到靶格距离」，实际打的是板心） |
| `SHOOT_DISTANCE_M_DEFAULT / TOL_M_DEFAULT` | 984/985 | 0.90 / 0.12（缺键兜底；必须与 robot.yaml 同值，test_planner 会钉） |
| `shoot_distance_window` | 988 | (mid − tol, mid + tol) |
| `plate_bisector_axis_m` | 1001 | 轨道垂直平分线法向（排序依据） |
| `plate_face_normal_m` | 1019 | 靶板板面法向（= 两靶格连线的垂线）；2026-10-18 修：旧实现错把轨道轴当法向 |


---

### §2.9 `pursuit/` —— 纯追踪（路径投影 / 弧长 / 舵角速度剖面 / 前视点）

#### 2.9.1 `pursuit/path.py` —— 折线与弧长（`path.py:1-456`）

**定位**：把 `Route` 编译成可投影的折线；提供弧长查询、分段属性、投影（核心）。

**`Path` 类**（`path.py:74-456`）：

| 成员 | 行 | 签名 | 说明 |
|---|---|---|---|
| `__init__` | 80 | (route: Route) | 预建段表 `_segments`（x0,y0,x1,y1,length,heading）、`_s_start`（每段起点弧长）、`_total` |
| `total_length` | 113 | property | |
| `segments` | 118 | property | |
| `segment_count` | 123 | property | |
| `route / waypoints / stop_indices` | 127-140 | | |
| `_seg_index_at(s)` | 143 | (s) -> int | bisect；边界归**后**一段；越界夹紧 |
| `point_at(s)` | 152 | (s) -> (x, y, seg_index) | 线性插值；越界夹到 [0,total] |
| `heading_at(s)` | 169 | (s) -> float | 所在段航向（弧度） |
| `s_at_waypoint(index)` | 173 | (index) -> float | index==航点数-1 ⇒ total |
| `remaining(s)` | 181 | (s) -> float | 负值夹 0 |
| `segment_is_reverse(seg_index)` | 189 | (seg_index) -> bool | R: 标记挂在**终点航点**上（与旧 _is_reverse_segment 同语义）；越界返回 False（防御，不外抛） |
| `segment_speed_scale(seg_index)` | 195 | -> float | 两端点 speed_scale 取**较小值**（限速挂哪端都生效，失败偏安全） |
| `corner_angle_deg(seg_index)` | 203 | -> float | 本段与下一段的**行进方向**夹角（0=共线，180=原路折返）；末段无下一段 ⇒ 0。⚠️ 这是行进方向夹角，不是车头要转的角度 |
| `car_heading_on(seg_index)` | 215 | -> float | 该段行驶时**车头**该朝的方向；倒车段车头与行进方向相反 |
| `corner_turn_deg(seg_index)` | 228 | -> float | **车头在顶点处实际需要转过的角度**（0~180°）。★★ 本项目真 bug 的来源：旧代码在 PIVOT 判据里用 corner_angle_deg（行进方向夹角）；折返压线时下一段若是倒车段，行进方向差 180° 而**车头一度都不用转** ⇒ PIVOT 被误触发，还把车头转反 180° |
| `project(...)` | 246 | (x, y, *, s_hint=None, window_m=0.60, allow_backward=True, motion_heading=None, back_m=None, s_min=None) -> Projection | **核心投影**（见下） |

**`Projection`**（`path.py:49-69`）：frozen seg_index, t, x, y, s, dist, seg_length, seg_heading, passed_end；property s_vertex / dist_to_vertex。

**`project` 关键语义**（`path.py:246-456`）：
- 有 s_hint 时只在 `[s_hint - window_m, s_hint + window_m]` 内搜索；allow_backward=False 时下界收到 s_hint。
- allow_backward=False 时索引只前进（防倒退误锁）；**倒车段（R:）必须用 True**。
- back_m：**单拍内投影点允许向后跳的最大距离**（默认 None = 用 window_m）。为什么需要：折返压线时去程段与回程段**完全共线**，同一物理点到两条段等距，而两条候选的 s 相差整整一段（0.0 vs 0.70）⇒ 若允许向后搜 0.6 m，s 就在 0.0↔0.70 之间跳，hint 也跟着跳，两个吸引子互锁（实测 601 次翻转、位置毫米级抖动、12 s 后触发「拐角绕圈兜底」）。正常每拍只走 2.4 mm，15 cm 余量绰绰有余。
- 末点之后：最近段 = 末段且 t > 1 ⇒ `t=1, s=total, passed_end=True`。
- s_min：**弧长地板**（单调推进的硬下界）—— 已经走过的顶点不允许再退回去（与仿真 _project_with_floor 同语义），断的是「倒车段↔正向段交界处」的自指环。
- 顶点前瞻重投影：最优点正好贴在当前段末端顶点（t ≥ 1）且还有下一段时，内部 hint 推到 s_at_waypoint(i+1) + 1e-3 再投影一次（最多一次），采用「不比第一遍更差」的结果 —— 纯追踪在顶点上必须用下一段算前视，返回顶点本身会让前视点塌缩到车身上（舵角恒 0、车直行漂出走廊）。
- motion_heading：**实际行进方向**（地图系弧度；倒车时是车头反向）。路线折返压在同一条线上时（现场图 B6→A6→R:B6），同一物理点落在两条航段上等距，位置本身无法裁决 —— 唯一判据是「车正在往哪边走」：往前一趟 ⇒ 命中前一条段，倒回来 ⇒ 命中后一条段。不传时退化为「等距取更接近 s_hint 的候选」。

**`_search_window` / `_nearest`**（`path.py:290-453`）：
- `_search_window`：返回 (i_first, i_last, t_floor)；窗口下界必须在**弧长**上生效（不能只按段生效）；并对首段加 t 下限（否则首段会被整段搜索 ⇒ back_m 形同虚设，实测 s 在 0.202↔0.499 之间倒带）。
- `_nearest`：窗口内用**平方距离**比较找最近点；折返压线的等距裁决依次用 motion_heading → s_hint 连续性 → 段序。


#### 2.9.2 `pursuit/controller.py` —— 纯追踪输出层（`controller.py:1-325`）

**不变量**（`controller.py:15-22`）：
1. steer_deg 单位是**度**，正 = 左转；
2. |steer_deg| ≤ max_steer_deg，且相邻两拍变化 ≤ max_steer_rate_deg_s × dt；
3. linear_x < 0 ⟺ 倒车；倒车速度 = 前进速度 × reverse_speed_ratio，下界 min_speed_mps；
4. curvature = 2·sinα / l_d（**不含 gain** 的目标几何曲率，正 = 左弯）—— 限速用「路线要求转多少」，与转向增益解耦。

**`PursuitCommand`**（`controller.py:40-48`）：frozen linear_x(m/s) / steer_deg(°) / curvature(1/m) / speed_reason / reverse。

**`motion_limit_speed_limit(bb) -> float|None`**（`controller.py:54-79`）：读黑板键 P_MOTION_LIMIT → 线速度上限。**任何情况都不抛异常**（50 Hz 控制环里被调用）；键不存在/None/非 dict/缺 speed_mps/非数字/NaN/±inf/负数 ⇒ 全部当「无信号」（读侧行为与没有这个键时逐位一致）。合法值 ⇒ 钳到该值以内（只降不升，倒车按绝对值保号）；0.0 = 请求停车，必须真的给 0（不受 min_speed_mps 兜底）。

**`speed_profile(...) -> (v, reason)`**（`controller.py:85-162`）—— 速度剖面 8 层（顺序即优先级）：

| 层 | 判据 | reason | 行 |
|---|---|---|---|
| 1 巡航 | v = cruise × max(0, speed_scale) | cruise | 123-127 |
| 2 曲率限速 | v = min(v, sqrt(a_lat/|κ|)) | curve | 129-131 |
| 3 停站减速 | v = min(v, max(min_speed, stop_approach_speed_gain × dist_to_stop)) | stop_approach | 133-136 |
| 4 转角减速 | 距尖角顶点 < corner_slow_dist_m ⇒ v = min(v, corner_speed_mps) | corner | 138-141 |
| 5 量化补偿 | v ×= speed_quant_comp | （不变） | 143 |
| 6 加减速斜坡 | v_hi = last + accel_up·dt；v_lo = max(min_speed, last − accel_down·dt) | ramp_up / ramp_down | 146-156 |
| 7 下限 | v = max(v, min_speed) | min_speed | 158-159 |
| 8 限速请求 | v = min(v, motion_limit_mps) | motion_limit | 160-161 |

**★ 加减速斜坡**（`controller.py:99-115`）：两个键 **缺省 0.0 = 整层完全不参与**（与历史逐位一致）。last_speed_mps = 上一拍控制器输出的速度（命令量）；dt = 控制周期。**两者都给了、且至少一个限值 > 0 时本层才参与**。**首拍/无历史（last_speed_mps is None）或没给 dt ⇒ 本层不参与** —— 为什么不是「当成 0」：调用方若不传历史，每一拍都被当成首拍 ⇒ 速度被永久压在 accel_up·dt（0.7×0.02=0.014 m/s）⇒ 车根本不会动。施加位置在 min_speed 下限**之前**，因此「运动中速度 ≥ min_speed」的既有不变量不变；而 motion_limit（安全层）仍在**最后**，所以「请求停车 = 0」不会被斜坡或下限顶回去。
**★ cruise_mps 注入点**：动态档的巡航速度覆盖；None = 用 pursuit.cruise_speed_mps（与历史逐位一致）；给值就只换「巡航那一层」的基准速度，其余层一字不改。

**`_dist_to_stop(pose, path, proj, cfg)`**（`controller.py:165-181`）：前方最近停站/终点距离；只遍历 path.stop_indices（射击点/扫描点/末点），不做全表扫描；已过（i ≤ proj.seg_index）的点不再减速。

**`_corner_dist(path, proj, cfg)`**（`controller.py:184-188`）：本段末端是尖角（corner_angle_deg > corner_angle_deg）⇒ 返回 dist_to_vertex，否则 None。

**`dynamic_gear_speed(cfg, dist_to_cross_m) -> float|None`**（`controller.py:191-213`）：动态档。总闸 pursuit.dynamic_gear_enabled 默认 **False** ⇒ 返回 None（调用方不给 compute_command 传覆盖值 ⇒ 与历史逐位一致）。键：near_dist_m 默认 1.05 m（3 格）、near_mps 0.35、far_mps 0.40。dist_to_cross_m is None（前方没有桥洞）= 远档。

调用方：tasks/tracker.py（动态档）、tests/test_dynamic_gear.py、tests/test_motion_limit.py。


**`compute_command(...) -> PursuitCommand`**（`controller.py:219-315`）—— **纯追踪公式（逐行）**：

~~~python
limit = motion_limit_mps if motion_limit_mps is not None else motion_limit_speed_limit(bb)
wheelbase = cfg.get("pursuit.wheelbase_m")
gain = cfg.get("pursuit.curvature_gain")
ld = lookahead_distance(cfg.get("pursuit.cruise_speed_mps"), cfg)   # clamp(k_v·|v|,[min,max])
denom = max(ld, cfg.get("pursuit.lookahead_floor_m"))               # 分母下限防增益爆炸
target = find_lookahead_point(path, proj, ld, cfg)

# 车体系分解（与旧实现同一旋转写法）
dx = target.x - pose.x
dy = target.y - pose.y
cos_neg = math.cos(-pose.yaw)
sin_neg = math.sin(-pose.yaw)
rel_x = dx * cos_neg - dy * sin_neg
rel_y = dx * sin_neg + dy * cos_neg
chord = math.hypot(rel_x, rel_y)
alpha = 0.0 if chord < _CHORD_EPS else math.atan2(rel_y, rel_x)   # 目标贴脸 ⇒ alpha=0
sin_a = math.sin(alpha)

steer_deg = math.degrees(math.atan(gain * 2.0 * wheelbase * sin_a / denom))   # δ = atan(gain·2L·sinα/l_d)
max_steer = cfg.get("pursuit.max_steer_deg")
steer_deg = max(-max_steer, min(max_steer, steer_deg))

rev = bool(reverse) or path.segment_is_reverse(proj.seg_index)   # 倒车判定 = 显式参数 OR 该段带 R:
if rev: steer_deg *= cfg.get("pursuit.reverse_steer_sign")

rate = cfg.get("pursuit.max_steer_rate_deg_s")                   # 速率限幅作用于最终输出（含倒车符号翻转）
if last_steer_deg is not None and dt > 0.0:
    step = rate * dt
    delta = steer_deg - last_steer_deg
    if delta > step: delta = step
    elif delta < -step: delta = -step
    steer_deg = last_steer_deg + delta
    steer_deg = max(-max_steer, min(max_steer, steer_deg))

curvature = 2.0 * sin_a / denom                                  # 不含 gain
speed, reason = speed_profile(curvature=curvature, dist_to_stop=_dist_to_stop(pose, path, proj, cfg),
                              corner_dist=_corner_dist(path, proj, cfg),
                              speed_scale=path.segment_speed_scale(proj.seg_index),
                              cfg=cfg, motion_limit_mps=limit, last_speed_mps=last_speed_mps,
                              dt=dt, cruise_mps=cruise_mps)
if rev:
    min_speed = cfg.get("pursuit.min_speed_mps")
    speed = -max(min_speed, speed * cfg.get("pursuit.reverse_speed_ratio"))
    # 限速请求对倒车同样有效：符号翻转后再按绝对值钳（保号）；
    # 必须放在翻转之后，否则 speed=0 会被 max(min_speed,…) 顶成蠕动速度
    if limit is not None and abs(speed) > limit:
        speed = 0.0 if limit == 0.0 else math.copysign(float(limit), speed)
        reason = "motion_limit"

return PursuitCommand(linear_x=speed, steer_deg=steer_deg, curvature=curvature,
                      speed_reason=reason, reverse=rev)
~~~

**★ 公式说明**：旧实现（path_executor.pure_pursuit_control 1736-1809）舵角公式**缺教科书因子 2**（δ = atan(L·sinα/d)，等效增益只有一半）；现在 δ = atan(gain · 2L·sinα / l_d)，**gain = 0.5 逐位等于旧行为**（0.5 × 2 = 1 在 IEEE754 里精确），所以现场要不要吃那半个增益是一个 YAML 键的事。

**★★ 倒车判定与 PIVOT/倒车的关系**：
- 倒车判定 = 显式 reverse 参数 **OR** 该段带 R: 标记（取或，两者都不会漏）；
- 倒车只做两件事：速度取负 × reverse_speed_ratio、舵角 × reverse_steer_sign；
- **PIVOT**（原地转向）不在本模块：它是「接近顶点时停车 + 低速蠕动转向，直到航向对上再放行」，状态机在 tasks/tracker.py，阈值 pursuit.pivot_angle_deg（135°）/ pivot_dist_m / pivot_timeout_s 等。本模块的 corner_turn_deg 是 PIVOT 判据的几何输入。

**★ 前视距离由巡航速度决定（不随瞬时速度变化）**：否则「减速 → 前视变短 → 转向变猛」会形成正反馈，低速贴站时反而摆头。
**★ 前视/速度两处耦合点**：ld 用 cfg 的 cruise_speed_mps（与 cruise_mps 覆盖无关），几何不变。

**`brake_command(cfg) -> PursuitCommand`**（`controller.py:318-325`）：零速刹车。**steer_deg 也必须是 0**：下位机 servo = angular.z + 60，发非零舵角会让轮子保持打死姿态停车（下次起步带偏）。

调用方：tasks/tracker.py（每拍调 compute_command）、tools/sim.py:run_sim（`sim.py:1209-1213`）；被调用方：tests/test_pursuit.py、tests/test_sim.py、tests/test_motion_limit.py、tests/test_dynamic_gear.py。


#### 2.9.3 `pursuit/lookahead.py` —— 前视点（`lookahead.py:1-100`）

**设计**：从投影点沿路径按弧长 ld 前进，**可跨段**；路径不足前视距离时直接瞄准终点；转角顶点钳制；接近末点时钳到最后一点绝不越界。

**`LookaheadPoint`**（`lookahead.py:38-46`）：frozen x, y, seg_index, s, clamped_to_vertex。

**★★ 退化保护**（`lookahead.py:10-13, 65-83`）：车正好站在顶点上（dist_to_vertex == 0，Localization 把位姿吸附到格心时必然出现）时**不做顶点钳制**，而是把 s_target 抬到顶点之后（`_VERTEX_STEP_M = 1e-3`）—— 否则前视点 = 车自己 ⇒ 弦长 0 ⇒ 舵角恒 0、车直行漂出走廊（现场 F10 实测：2.5 s 内投影 s 冻在 3.500 不动、dist 长到 0.300 m、舵角恒 +0.0°、v 恒 +0.120，12 s 后触发「拐角绕圈兜底」强制跳点）。

**★★ 前视点只按弧长推进（不是直线插值）**：若按直线取点，过弯时会把目标点切到路线内侧，等于主动抄近道。

**`find_lookahead_point(path, proj, ld, cfg) -> LookaheadPoint`**（`lookahead.py:49-100`）逻辑顺序：
1. **退化保护**（优先）：dist_to_vertex ≤ _DEGENERATE_M（1e-6）且还有下一段且 ld > 0 ⇒ s_jump = s_at_waypoint(seg_index+1) + 1e-3；s_target = max(proj.s + ld, s_jump)，再夹到 total；返回 seg_index = 顶点之后那一段、clamped_to_vertex=False；
2. **转角顶点钳制**：clamp_m > 0 且 corner_angle_deg(seg_index) > lookahead_corner_angle_deg 且 dist_to_vertex < clamp_m ⇒ 返回顶点本身（seg_index = 正在行驶的段，s = s_at_waypoint(seg_index+1)，clamped_to_vertex=True）；
3. **沿弧长前进**：s_target = proj.s + (ld if ld > 0 else 0)；s_target ≥ total ⇒ 返回 point_at(total)；s_target < 0 ⇒ 0；返回 point_at(s_target)。
⚠️ ld ≤ 0 只表示「不前视」，必须退化成投影点本身，**不能**跳到终点。

调用方：`pursuit/controller.py:249`；被调用方：tests/test_pursuit.py。

---

### §2.10 `tools/sim.py` —— 离线纯追踪仿真器（`sim.py:1-2309`）

**定位**：离线纯追踪仿真器（C 线，docs/P0接口约定.md §5）——**不碰车就能调参与回归**。把「车 + 里程计误差 + 到下位机的链路延迟」压成一个确定性模型，同一组参数永远得到同一条曲线。

**模型（★★ 2026-10-07 重写：单一执行量驱动两条轨迹）**：
- **执行链（唯一的物理真相）**：控制器每拍输出一条 PursuitCommand → cmd_latency_s 延迟队列（**队列空时保持上一条已生效指令**，绝不回零）→ 执行侧舵角斜坡（受 max_steer_rate_deg_s 限制）→ 得到本拍**实际执行量** (v_exec, δ_exec)。
- **真值（物理世界）与信念（里程计）用同一套离散积分器、同一个执行量**：真值 step_bicycle(x,y,ψ,v_exec,δ_exec)；信念 step_bicycle(x,y,ψ, v_exec·(1+ε), δ_exec) + yaw_drift + 高斯噪声。
- ★ **零误差不变量**：ε=0、零漂=0、噪声=0 时两条轨迹**逐位相同**（差 = 0，不是「很小」）。唯一的例外是**到位吸附**（外部把信念拽过去）。

**误差源 → SimConfig 字段**：里程比例误差 speed_scale_err；航向零漂 yaw_drift_deg_per_min；位置白噪声 pos_noise_m；到位吸附 anchor_on_arrival + anchor_radius_m。

**★ 本仿真器复现的是「信念驱动」闭环**（实车真实形态）：控制器拿信念位姿投影 + 决策，指令作用在真值车体上。后果必须写清楚：
1. 单纯 speed_scale_err **几乎不产生横向误差** —— 信念投影沿路线「多跑」18%，但真值仍在同一条折线上，只是沿弧长滞后；真正放大的是 belief_error_rms_m 与顶点超调。
2. 「锚定 vs 不锚定」的对照要看**信念误差**（锚定把它夹在 anchor_radius_m 内，不锚定则按 ε × 已走弧长无界累积）。
3. 位置白噪声会被 50 Hz 控制环 + 舵机速率限幅低通掉大半。
4. ⚠️ **倒车段的舵角符号翻转**（reverse_steer_sign，60° 量级）是切换瞬间的物理事实，不是「速率超限」。它单独统计（steer_sign_flip_steps / steer_cmd_rate_at_flip_deg_s），不污染速率分位数与峰值判定。

**★ 不变量**：run_sim() 是**纯函数**（不读文件、不写文件、不 import rclpy）；所有模型参数来自 Config 与 SimConfig（不硬编码魔法数字）；真值轨迹**不参与控制**；metrics 键名是契约（docs/P0接口约定.md §5），只增不改；指标**按段类型分组**（normal/reverse/corner）；起拍位姿 = 现场摆位约定（DESIGN_START_BLOCK="N10"、DESIGN_START_YAW=geo.HEADING_Y），不从路线反推。

**刻意不做的事（复核定稿，别再当 bug 报）**（`sim.py:107-112`）：本仿真**不建模 PIVOT**（状态机在 tasks/tracker.py，sim 不 import tasks/）。后果：① config 里所有 pivot_* 键在本仿真**恒为 no-op**；② 由 180° 折返点主导的指标（A6/A1 大弧、横向 MAX 等）**不可迁移实车**；③ 千万别用本仿真验证/调 PIVOT 行为。

**`FieldSpec`**（`sim.py:137-154`）：frozen mines/obstacles/targets/bridges/barrier_col_from/barrier_row_hi/barrier_row_lo/name/source；describe()。
**`parse_map_yaml(text, *, source="") -> FieldSpec`**（`sim.py:184-242`）：**纯函数**，不用 PyYAML（车端最小镜像/CI 也能跑）；只解析 map: 段下真正要用的键（标量 + 列表，行内 [..] 或块状 - X 两种写法）。
**`load_field_spec(map_path=None, *, cfg_source=None) -> FieldSpec|None`**（`sim.py:259-292`）：**本模块唯一的读文件入口**（run_sim 自己不调它）。查找顺序：显式 map_path > 环境变量 ROBOT_MAP > cfg_source 同目录 map.yaml > 仓库 config/map.yaml。**显式给路径时不再回退**（悄悄改用另一张地图比报错危险）。
**`body_corners / body_point_clearance_m`**（`sim.py:295-319`）：车体矩形 4 角 / 点到车体矩形最近距离（换坐标系后套 physics.point_rect_distance_m，不另写一套几何）。


**`FieldObs`**（`sim.py:322-553`）—— 场地物理观测累加器：每拍喂一次**真值**位姿，最后取指标。**只用真值**（地图上的物理事实不能跟着里程计漂移走）；**只观测不改动力学**（stop_on_violation=False 默认时对轨迹零影响，有硬断言）。

构造形参：spec / length_m(0.28) / width_m(0.135) / coverage_radius_m(0.035) / obstacle_radius_m(0.1275) / mine_pad_m(0.12) / barrier_width_m(0.02) / barrier_cols("D..N") / obstacle_margin_m(0.02) / stop_on_violation(False) / bridge_entry_redundancy_m(0.0875) / ref_offset_m(0.0)。

**`__post_init__`**（`sim.py:353-405`）预建：mine_xy / obs_xy / tgt_xy / open_rows / k_col / bar_x,bar_y0,bar_y1 / bar_need / board_half；累加量 press_steps / mine_best / obs_hit_steps / obs_min_center / obs_breach_normal / obs_breach_corner / bar_min / bar_breach_steps / board_overlap_steps / board_breach_steps / board_min / k_col_steps / k_seal_steps / k_solid_steps / bridge_modeled / bridge_entry_walls / bridge_steps / bridge_min / steps / first。
⚠️ bridge_modeled 的判据是：bridge_modeled = 1 if (spec.bridges and redundancy > 0.0) else 0。为什么关闭必须走「不建墙 + modeled=0」而不是「把墙退化成零长度线段」：零长墙刚好落在洞口边线上，车正摆着穿洞时车体横向边缘离该线仅 0.175 − 0.0675 = 0.1075 m ⇒ 横向偏移 ≥ 0.1075 m 就报 0；实测横向 dx ≥ 0.1075 时零长墙**全部**返回 0.000000（假阳性），而正摆穿洞的轨迹横向偏移本就在 ±0.05 m 内 ⇒ 全轨迹恰好 0 拍而「看着没事」，这个陷阱是**潜伏**的。

**`observe(t, x, y, yaw, *, kind="normal") -> bool`**（`sim.py:411-519`）—— 六类判据（全部用**真值**位姿）：

| # | 判据 | 统计量 | 行 |
|---|---|---|---|
| ① 盖雷 | 车心 3.5 cm 圆 ∩ 12×12 垫片（mine_clearance_m ≤ coverage_radius + 1e-12） | press_steps / mine_best | 416-421 |
| ② 障碍 | 物理碰撞（车体 ∩ 圆柱 = body_point_clearance_m ≤ 0）+ 净空门槛（直段 0.215 / 拐角 0.3029） | obs_hit_steps / obs_breach_normal / obs_breach_corner / obs_min_center | 422-442 |
| ③ 挡板 | 车心到中心线 < 半厚 + 车半宽 = 0.0775 | bar_breach_steps / bar_min | 443-450 |
| ④ 靶板 | **两级**：board_overlap_steps（物理重叠，精确）+ board_breach_steps（设计判据不足：间隙 < 车半宽 0.0675 / 拐角 0.1554） | board_overlap_steps / board_breach_steps / board_min | 451-477 |
| ⑤ K 列桥室 | 非通格 = 实体；纵向相邻格之间封死；从非通格横穿出去 | k_col_steps / k_solid_cell_steps / k_seal_violation_steps | 478-498 |
| ⑥ 桥洞进洞侧冗余墙 | 车体旋转矩形 ∩ 虚拟墙（body_bridge_wall_clearance_m ≤ 0） | bridge_steps / bridge_min | 499-518 |

⚠️ 不要把 physics.target_body_overlap() 套在「已有精确车体距离」上：那个函数的输入是折线/顶点的间隙（把扫掠包络折算成间隙门槛），套在精确车体距离上会把扫掠算两遍、虚报违规。

**`metrics() -> dict[str, float]`**（`sim.py:521-553`）：键名只增不改。返回 field_modeled / mine_total / mine_pressed_count / mine_press_steps / mine_worst_clearance_m / obstacle_* / barrier_* / board_* / k_column_steps / k_seal_violation_steps / k_solid_cell_steps / bridge_redundancy_modeled / bridge_redundancy_steps / bridge_redundancy_min_clearance_m / bridge_entry_redundancy_m。⚠️ modeled=0（关判据 / 本图无桥洞）时下面三条**照样存在**但语义是「没测」：bridge_redundancy_steps 恒 0、bridge_..._min_clearance_m = inf —— 报告里必须与 modeled 一起读。

**`SegmentStat`**（`sim.py:556-573`）：frozen index / from_block / to_block / length_m / kind("normal"|"reverse") / peak_m / peak_t / peak_s / corner_deg。**为什么要它**：整条路线的 cross_track_rms_m 是一个标量，0.20 m 到底是「全程均匀偏 0.2」还是「29 个直段完美 + 3 个拐角各偏 1.0 m」？两者修法完全不同。

**`SimConfig`**（`sim.py:692-746`）—— 仿真注入参数（默认 = 理想车：无误差、无延迟、有锚定）：

| 字段 | 行 | 默认 | 说明 |
|---|---|---|---|
| dt | 699 | 0.02 | 控制周期（50 Hz） |
| cmd_latency_s | 700 | 0.06 | 指令到执行延迟 |
| speed_scale_err | 701 | 0.0 | 里程比例误差（+0.18 = 里程计比真实多报 18%） |
| yaw_drift_deg_per_min | 702 | 0.0 | 航向零漂 |
| pos_noise_m | 703 | 0.0 | 位置白噪声 |
| anchor_on_arrival | 704 | True | 到位吸附 |
| anchor_radius_m | 705 | 0.12 | 吸附触发半径 |
| max_steps | 706 | 0 | 0 = 自动推算 |
| seed | 707 | 20261007 | 白噪声种子（固定种子 = 可复现） |
| trace_every | 708 | 1 | 每 N 拍记一条 |
| stop_on_completion | 709 | True | 到终点即停 |
| bounds_margin_m | 710 | 0.0 | 越界判据余量；0 = 用 pursuit.map_safety_margin_m |
| init_yaw | 718 | "start" | 起拍航向来源："start"（现场摆位约定，默认）/ "route"（历史行为）/ "waypoint" |
| actuator_rate_limit | 719 | True | 执行侧舵角斜坡跟随 |
| command_rate_limit | 720 | True | 控制器侧速率限幅 |
| trace_hook | 721 | None | 每拍回调（排障用） |
| stop_on_field_violation | 725 | False | 本拍出现物理违规即结束（对照组用） |

**`SimResult`**（`sim.py:749-798`）：逐拍数组 t/x/y/yaw/v/steer（**信念**）、truth_x/truth_y/truth_yaw（**真值**）、seg_index/s、truth_seg_index/truth_s、steer_cmd（控制器**输出**舵角）、anchored/near_vertex/reverse；metrics dict；segments list[SegmentStat]；field_report dict；init_yaw_src str。
⚠️ steer 是**信念侧看到的舵角**（= 执行舵角），steer_cmd 才是控制器**输出**的舵角 —— 两者差一个执行侧斜坡。混淆它们正是老实现「指令速率 680.9 °/s」假指标的来源。
方法：steps property / metric(key, default) / series(name)。


**算法常数（不是可调参数）**（`sim.py:591-689`）：

| 常量 | 行 | 值 | 说明 |
|---|---|---|---|
| `_PROJ_FWD_M` / `_PROJ_BACK_M` | 600/601 | 0.80 / 0.20 | 投影窗口**前宽后窄**：前视约 2.3 格（过弯时车先于投影点向前走）；后视 0.20 m（路线自交处后视一大就会锁到另一条边） |
| `_PROJ_BELIEF_M` | 609 | 1.50 | 信念投影窗口（以物理投影弧长为锚 ±1.5 m）；太小 ⇒ 投影锁在窗口边界不动；太大 ⇒ 自交处最近点落到别的航段 |
| `_MH_MIN_STEP_M` | 615 | 1.6e-5 | 行进方向估计的最低位移（米/拍）；低于它就认为「没在走」保留上次方向 |
| `_SEG_STICKY_M` | 626 | 0.02 | 段选择切换滞回：新候选要比上一拍命中段近这么多才允许换段（现场 B6 四条段交汇，等距抖动实测 2000 拍） |
| `_ADVANCE_FACING_TOL_DEG` | 632 | 90.0 | 目标推进的朝向门容差；90° 拐角卡在边界上（<= 放行），180° 掉头一定被挡 |
| `_DIVERGE_STEPS` | 635 | 25 | 投影弧长连续这么多拍不前进且已远离路线 ⇒ 跑飞 |
| `_ANCHOR_CORNER_TOL_DEG` | 638 | 5.0 | 拐角顶点不做到位吸附 |
| `_GOAL_STOP_M` | 643 | 0.03 | 终点停车阈值（比 pursuit.arrival_m 更紧） |
| `_POS_NOISE_LIMIT_M` | 646 | 0.50 | 位置噪声上限 |
| `_END_EPS_M` | 649 | 1e-6 | 停止推进的信念误差阈值 |
| `_CORNER_ZONE_M` | 654 | 0.35 | 分组用的顶点邻域宽度（= 一格） |
| `_DELIVERED_JUMP_DEG` | 659 | 1e-9 | 执行侧舵角跳变告警阈值（正常必须为 0，是模型自检指标） |
| `_SVG_W/H/PAD` | 662-664 | 1000/800/56 | SVG 画布 |
| `DESIGN_START_BLOCK` / `DESIGN_START_YAW` | 685/689 | "N10" / geo.HEADING_Y | 起拍位姿 = 现场摆位约定 |

**`run_sim(route, cfg, sim=None, field=None) -> SimResult`**（`sim.py:892-1531`）：跑一次闭环仿真。sim=None ⇒ 理想车；field=None ⇒ **不建模任何地图要素**（metrics["field_modeled"]=0.0）。

**主循环七步**（`sim.py:1045-1360`）：
1. **真值 + 信念：同一执行量、同一积分器、同一拍**（`sim.py:1046-1061`）。★ 顺序是模型正确性的核心：本拍先把上一拍**已生效**的执行量 (v_exec, steer_exec) 同时积分到真值与信念，然后才做决策。两个后果：零注入时信念与真值逐位相同；控制器用的 (bx,by,byaw) 与真值 (tx,ty,tyaw) 是同一时刻状态。⚠️ 老顺序把信念积分放在拍首、用上上拍的执行量，真值积分放在拍尾、用当拍量 —— 错开一个控制周期，零误差下信念恒定落后真值 v·dt = 3 mm，所有注入实验都失去基准。
2. **行进方向估计**（`sim.py:1063-1080`）：取**信念位姿的实际位移**（本拍 − 上拍），不是车头朝向（过弯时车头几乎垂直于路线，拿车头当方向会把投影裁决到垂直于车的那条段上）。只在位移 ≥ _MH_MIN_STEP_M 时更新。
3. **到位吸附**（`sim.py:1082-1113`）：四条约束缺一不可：① 触发用**真值**；② 每个航点只锚一次、锚完立即推进；③ 只锚还没走过的点（_advance_anchor）；④ **拐角顶点不锚**（_anchor_allowed）。修正量 = 钉到**真值此刻的位置**（bx,by = tx,ty）—— 三个候选都实测过：钉格心会把 0.44 m 偏差灌进估计；沿路线法向归零抹不掉纵向里程误差；钉真值位置 ⇒ 零注入时修正量恒为 0（不变量成立）。⚠️ **只在修正幅度不超过 anchor_radius_m 时才真的修**（一次跳 1~2 m 会把信念投影甩到别的航段上）。
4. **任务推进**（`sim.py:1115-1173`）：两条规则缺一不可：① 到位（真值进入当前航点的 arrival_m 球内）⇒ 推进；② **已过点即放行**（投影弧长已越过该顶点）⇒ 推进。⚠️ 末点用更紧的 _GOAL_STOP_M（0.03），否则车会在离终点 0.12 m 处停下（final_error_m 长期钉在 0.118 的原因）。★★ **朝向门**：进到位球还不够，车头必须已经**朝着下一段的走向**（容差 90°）—— 否则 F11→E11 之后紧接 R:F11 时车被推离路线且再也回不来（实测 s 在 4.200 上钉死 1100 拍）。
5. **物理投影**（真值，`sim.py:1175-1193`）：_project_sticky + _clamp_forward；s_floor = path.s_at_waypoint(target_idx-1)。终点判据用**真值**（吸附会把信念瞬移到格心，用信念判会提前 arrival_m 就刹车）。
6. **信念投影 → 纯追踪算一拍指令**（`sim.py:1195-1248`）：_project_sticky(path, bx, by, s_hint=proj_t.s, window_m=_PROJ_BELIEF_M) + _clamp_forward；reached_end ⇒ brake_command(cfg)；否则 compute_command(Pose(bx,by,byaw), path, proj, cfg, last_steer_deg=..., dt=dt, last_speed_mps=last_cmd_speed)。指令侧速率 = 相邻两拍**控制器输出**之差；命令速度历史用于加减速斜坡。延迟队列 + 舵机斜坡（brake_hold 时舵机保持原位）。
7. **采样与分组统计 + 场地物理观测**（`sim.py:1250-1333`）：分组基于**真值**（否则误差注入下「车在哪一组」会跟着估计漂移）；逐段峰值记录；场地观测用真值位姿，**只读不改轨迹**。

**跑飞看门狗**（`sim.py:1347-1358`）：投影弧长连续 _DIVERGE_STEPS 拍不前进（abs(proj_t.s − last_s) < 1e-4）且已远离路线（proj_t.dist > 0.90）⇒ diverged=True。

**指标计算**（`sim.py:1364-1531`）：契约键 completed/time_s/cross_track_rms_m/cross_track_max_m/final_error_m/overshoot_max_m/max_steer_deg/max_steer_rate_deg_s/path_len_ratio/anchors/steps；附加键 belief_cross_track_rms_m/belief_error_rms_m/belief_error_max_m/belief_error_max_free_m（扣除锚定拍的残差 —— 零注入时必须 ≈0，是「模型自洽」的可测定义）；out_of_bounds_steps；分组键 cross_track_rms_normal_m/reverse_m/corner_m、vertex_overshoot_*、steer_rate_p95_deg_s、steer_sign_flip_steps、steer_delivered_jump_steps 等；场地键 field_modeled/mine_* 等；speed_ramp_steps。
⚠️ completed / anchor_on_arrival 是 bool，其余指标是 float —— 混类型是刻意的（CSV/SVG/控制台都不依赖类型统一，而测试要读布尔真值）。


**投影/推进的四个关键修正（互补，缺一不可）**（`sim.py:71-88`）：这类「跑不完」的根因**不是**几何（把 max_steer_deg 放到 60°、最小转弯半径 0.09 m 也跑不完），而是**投影在折返交汇处选错了段**：
1. **投影候选比距离**（`_candidates` / `_pick_candidate`）：原 `Path.project` 的「先到先得」会让**被夹在段端点的候选**赢过真正最近的点 —— 车头已伸过顶点，投影却停在顶点上（dist_to_vertex=0 ⇒ 前视点被转角钳制塌缩 ⇒ 舵角退化）。正确判据是「哪个候选离车**最近**」，而不是「哪个候选先被看到」。
2. **任务段下界 s_min**：投影不许落到 Mission 已经走过的段上（任务给段、定位给弧长）。
3. **行进方向裁决 motion_heading**：折返压线时同一物理点到两条段**等距**，位置裁决不了，只能用「车实际往哪边走」（取信念位姿的**实测位移方向**，不是车头朝向）。
4. **段选择滞回 `_SEG_STICKY_M`**：投影是**跟踪**（上一拍那段是强先验），不是每拍从零重算，否则在四段交汇的 B6 处逐拍翻转。

**投影辅助函数**（`sim.py:1534-1881`）：

| 函数 | 行 | 签名 | 说明 |
|---|---|---|---|
| `_prev_steer` | 1534 | (history) -> float | 上一拍执行舵角 |
| `_record_final` | 1538 | (result, t, bx, by, byaw, v_eff, steer_eff, tx, ty, tyaw, s) | 补记最后一拍；只补位置列，诊断列用上一拍值兜底（少一列会让 CSV 与分组统计错位） |
| `_project_with_seg` | 1567 | (path, seg, x, y) | 把点投影到指定段（退化段退化为端点距） |
| `_candidates` | 1580 | (path, x, y, i_lo, i_hi) | 窗口内每段的投影候选 |
| `_pick_candidate` | 1588 | (cands, *, x, y, hint, motion_heading, allow_beyond) | **先比距离**，等距时依次用行进方向 → 连续性 → 段序裁决；allow_beyond=False 时不许选任务段之后的候选 |
| `_project_sticky` | 1635 | (path, x, y, *, s_hint, window_m, allow_backward, motion_heading, prev_seg, s_min=0.0) | **带任务下界 + 段选择滞回**的投影（Path.project 的加强版）。三层修法：① s_min = 当前任务段起点弧长；② 窗口内比距离；③ 上一拍那段是强先验（别的候选至少要近 _SEG_STICKY_M 才允许换段，换段后立即生效，滞回不对称） |
| `_window_segments` | 1688 | (path, s_hint, window_m, allow_backward) -> (i_first, i_last) | 与 Path.project 同口径的段区间 |
| `_seg_index_of_s` | 1709 | (path, s) -> int | 与 Path._seg_index_at 同义 |
| `_projection_at` | 1719 | (path, seg, s, x, y) | 构造「段 seg 内、弧长 s 处」的投影（dist 如实取到车的位置） |
| `_clamp_forward` | 1740 | (path, proj, s_floor, x, y, release_dist) | 投影弧长单调不退回：抬到 s_floor（= 当前航段起点弧长）。⚠️ **但「钉住」必须能解开**：当车在段边界外绕圈/倒车（到被钉住那点的距离 > release_dist）时必须让投影落回真实最近点（哪怕退到上一段）。★★ **真正的死锁机制**：s_floor 来自 Mission 的目标，而 Mission 会在「车进入顶点 arrival_m 球内」时**提前**推进 —— 此时车还差一点没到顶点，s_floor 却已跳到下一段起点；两个「释放」判据都要求 dist ≤ arrival_m ⇒ 都不成立 ⇒ 投影被钉在顶点朝南，控制器认为「我已到 F11、该倒车了」，车被推离路线（实测 0.38 m，xt_rms 0.107 → 0.424）。⇒ 只要「车其实还在上一段上」（proj.seg_index < s_floor 所在段），钉住就必然是假的：取「原始投影」与「本段起点弧长」的**较大者** |
| `_anchor_allowed` | 1795 | (path, idx) -> bool | 拐角顶点不允许吸附（锚到顶点会让前视点塌缩成投影点本身，纯追踪算 0 舵角直行漂出） |
| `_advance_anchor` | 1815 | (path, anchor_idx, s_belief) -> int | 推进到第一个「信念还没领先于」的航点（判据 s_at_waypoint(i) >= s_belief） |
| `_schedule` | 1829 | (queue, at, v, steer_deg) | 插进延迟队列（bisect 保序） |
| `_take` | 1837 | (queue, now, v_hold, steer_hold) -> (v, steer) | 取出到点生效的指令；**队列里没有新指令时保持上一条**。⚠️ 老实现队列空时返回 (0,0)：dt(0.02) < cmd_latency_s(0.06) 时队列每拍都是空的 ⇒ 舵角在 ±30° 与 0 之间逐拍跳（实测执行侧 1500 °/s） |
| `_Rng` | 1853 | (seed) | xorshift32 + Box–Muller 高斯（不引第三方，种子固定则结果固定） |

**输出函数**（`sim.py:1884-2309`）：

| 函数 | 行 | 签名 | 说明 |
|---|---|---|---|
| `METRIC_KEYS` / `GROUPED_METRIC_KEYS` | 1888/1895 | tuple | 契约要求的指标键（存在性由测试断言；缺一个都算接口违约） |
| `_METRIC_TABLE` / `_GROUP_TABLE` / `_FIELD_TABLE` | 1906/1929/1950 | tuple | 控制台三张表的 (键, 中文标题, 单位, 小数位)。⚠️ **场地表只在接了地图时打印**（field_modeled > 0）—— 否则「没建模」与「零违规」在控制台上长得一模一样，那是在说谎 |
| `format_metrics` | 1984 | (result, sim=None, title="") -> str | 主表 + 分组表 + 场地表 + 跑飞/步数上限告警 |
| `rate_desc` | 2031 | (sim, m) -> str | 执行侧限速一句话 |
| `format_segment_table` | 2038 | (result, top=0, title="") -> str | 逐段横向误差峰值表（top>0 只打峰值最大的 top 段） |
| `write_csv` | 2062 | (result, path) -> str | 逐拍轨迹 CSV；列 = docs/P0接口约定.md §5 十列（顺序不变）+ 追加诊断列 |
| `kind_of` | 2095 | (result, index) -> str | 采样点分组标签（与分组指标同口径：倒车优先，其次顶点邻域） |
| `write_segments_csv` | 2104 | (result, path) -> str | 逐段峰值表 CSV |
| `_traj_color` | 2125 | (metrics) -> str | 按信念误差取色（绿→蓝→橙→红） |
| `write_svg` | 2134 | (result, path, route, title="", field=None) -> str | 手写 SVG：路线折线 + 航点 + 真值轨迹 + 信念轨迹；给了 field 就按物理形状画场地（雷/障碍/挡板/K 列桥室/靶板/冗余墙），形状尺寸取自 result.field_report["shape"]（与碰撞判据**同源**） |
| `_xml_escape` | 2307 | (text) -> str | |

调用方：`tools/sim_cli.py`；被调用方：tests/test_sim.py、tests/test_sim_guard.py、tests/test_physics.py、tests/test_route.py 等。


---

### §2.11 `tools/sim_cli.py` —— 仿真器命令行入口（`sim_cli.py:1-289`）

**定位**：离线仿真命令行入口（docs/P0接口约定.md §5）。**全程无异常，退出码 0**；参数/文件问题 → 退出码 2（与 route_check.py 同约定）。`--json` 时 stdout 末尾追加一行 `SIM_RESULT_JSON {...}`（机器可读完整指标，契约版本 JSON_SCHEMA = "dzrobot-sim-result/1"）；**该一行永远是 stdout 最后一行**，文字表排版保持不变。
**刻意不做的事**：不 import rclpy（必须在没有 ROS 的笔记本上跑）；不改任何配置/路线文件。

**`build_parser`**（`sim_cli.py:50-100`）参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `--config` | <ws>/config/robot.yaml | 只读 pursuit.* 与 route.tokens |
| `--route` | None | 路线文件路径；缺省用 --config 里的 route.tokens |
| `--out-dir` | <ws>/artifacts/sim | 输出目录（traj.csv / traj.svg / by_segment.csv） |
| `--dt` / `--latency` | 0.02 / 0.06 | 控制周期 / 指令→执行延迟 |
| `--speed-scale-err` | 0.0 | 里程比例误差 |
| `--yaw-drift` | 0.0 | 航向零漂 °/min |
| `--pos-noise` | 0.0 | 位置白噪声 m/拍 |
| `--anchor / `--no-anchor` | True | 到位吸附开关 |
| `--anchor-radius` | 0.12 | 吸附触发半径 m |
| `--max-steps` | 0 | 步数上限（0 = 自动推算） |
| `--set KEY=VALUE` | [] | 覆盖任意点号配置键（可重复），如 --set pursuit.arrival_m=0.20；只影响本次运行，不写磁盘 |
| `--seed` | 20261007 | 噪声种子 |
| `--no-stop` | True | 到终点不停，一直跑到步数上限 |
| `--pretty` | False | 打印完整配置摘要与逐步统计 |
| `--segments-top` | 10 | 打印横向误差峰值最大的 N 段（0 = 全部，-1 = 不打印） |
| `--quiet` | False | 只打印指标表 |
| `--json` | False | 末尾追加 SIM_RESULT_JSON 行 |

**`_resolve_route`**（`sim_cli.py:103-113`）：优先 `--route` 文件，否则用配置里的 `route.tokens`；都缺 ⇒ ValueError。
**`_parse_set_overrides`**（`sim_cli.py:116-140`）：值按 YAML 解析（0.20→float、true→bool、[a,b]→list），失败则当字符串。★ 为什么需要它：仿真器的用途就是「**不碰车**做参数对照实验」。
**`_apply_overrides`**（`sim_cli.py:143-151`）：覆盖配置键（不改磁盘），并重新跑一次旧键迁移。
**`_json_sanitize`**（`sim_cli.py:154-164`）：非有限浮点（NaN/±Inf）换成 None，保证输出是严格 JSON。
**`_emit_json_error`**（`sim_cli.py:167-171`）：`--json` 模式下参数/输入类失败也产出一行机器可读结果（ok=false，退出码不变）。
**`main(argv=None) -> int`**（`sim_cli.py:174-285`）：读配置 → 应用 --set → 读路线 → 组装 SimConfig → run_sim → 写 CSV/SVG/分段 CSV → 打印指标表 + 逐段峰值表 → `--pretty` 打印配置摘要 → `--json` 追加 SIM_RESULT_JSON。

调用方：CLI；被调用方：tests/test_sim.py、tests/test_sim_guard.py。

---

### §2.12 模块间调用关系速查

| 调用方 | 被调用方 | 用途 |
|---|---|---|
| `main.py:load_and_check` | `Config.load / validate_route / parse_mission` | 启动门禁（配置 + 路线） |
| `main.py:plan_route_from_field_layout` | `planner.load_map / plan_route / plan_route_tracker_safe / ensure_second_shoot_stop` | 布局驱动规划 |
| `main.py:plan_segment_from_layout` | `tasks.mission.plan_segment_route` | 段规划 |
| `main.py:build_tasks` | `tasks.{localization,safety,tracker,mission,aim,fix,drone,telemetry}` | 装配 8 个任务线程 |
| `io/ros_io.py:on_odom` | `contract.odom_to_pose` | 格单位 + 四元数 → 米制地图 Pose |
| `io/ros_io.py:on_referee_hp` | `referee.RefereeTracker.feed` | 命中/挨打/弹药记账 |
| `route/planner.py` | `route/validate.py / route/physics.py / route/parser.py` | 复用校验与几何判据（**不复制规则**） |
| `pursuit/controller.py` | `pursuit/lookahead.py / pursuit/path.py / core/config.py` | 纯追踪输出 |
| `tools/sim.py:run_sim` | `pursuit/controller.py:compute_command / brake_command、pursuit/path.py` | 闭环仿真 |
| `tools/sim.py:FieldObs.observe` | `route/physics.py` | 场地物理判据 |
| `main.py:StepLoop._plan` | `plan_segment_from_layout` | 段规划 |
| `core/task.py:TaskThread.run` | `bus.publish(EV_TASK_FAULT)` | 故障上报 |


---

## §3 函数级索引大表（重点 · 尽量穷尽）

> 本表由脚本从源码逐行提取（`def`/`class` 行号 + 缩进判定模块级/类方法/嵌套），签名取自源码原文；**行号与签名均为实测**。
> 「类型」列：M=模块级函数，C=类方法（含 property），N=嵌套函数/闭包。

### 3.1 `main.py`

**类**：`StepLoop`@L587

| 行 | 名称 | 类型 | 签名（源码原文，≤160 字符） |
|---|---|---|---|
| 85 | `install_stop_signals` | M | def install_stop_signals(stop_flag: dict, log: Callable[[str], None] = LOG) -> list[str]: |
| 100 | `_handler` | N | def _handler(_signum, _frame, _name=name): |
| 114 | `ros_context_alive` | M | def ros_context_alive() -> bool: |
| 130 | `default_config_path` | M | def default_config_path() -> str: |
| 142 | `load_and_check` | M | def load_and_check(config_path: str \| None, map_path: str \| None, *, force: bool = False, announce: bool = True, allow_missing_map: bool = False) -> tuple[Confi |
| 214 | `load_field_layout` | M | def load_field_layout(bb: Blackboard \| None) -> dict[str, Any] \| None: |
| 231 | `plan_route_from_field_layout` | M | def plan_route_from_field_layout(cfg: Config, layout: dict[str, Any], *, log: Callable[[str], None] \| None = None, cross_lower: str \| None = None, cross_upper:  |
| 318 | `plan_startup_route` | M | def plan_startup_route(cfg: Config, bb: Blackboard \| None, *, log: Callable[[str], None] \| None = None) -> Any: |
| 362 | `_step_cells` | M | def _step_cells(value: Any, key: str, *, allow_empty: bool = True ) -> tuple[list[str] \| None, str]: |
| 387 | `normalize_step_layout` | M | def normalize_step_layout(payload: Any) -> tuple[dict[str, Any] \| None, str]: |
| 455 | `load_step_layout` | M | def load_step_layout(bb: Blackboard \| None) -> dict[str, Any] \| None: |
| 466 | `_step_mode_enabled` | M | def _step_mode_enabled(layout: Any) -> bool: |
| 471 | `all_map_cells` | M | def all_map_cells() -> list[str]: |
| 477 | `_segment_map_cfg` | M | def _segment_map_cfg(layout: dict[str, Any], from_cell: str, target_cell: str) -> dict[str, Any]: |
| 497 | `classify_segment_target` | M | def classify_segment_target(cfg: Config, layout: dict[str, Any], target_cell: str) -> str: |
| 519 | `plan_segment_from_layout` | M | def plan_segment_from_layout(cfg: Config, layout: dict[str, Any], *, from_cell: str, target_cell: str, heading_dir: tuple[int, int] \| None = None, log: Callable |
| 579 | `_find_task` | M | def _find_task(group: Any, name: str) -> Any: |
| 602 | `__init__` | N | def __init__(self, cfg: Config, bb: Blackboard, bridge: Any, mission: Any, *, bus: Any = None, log: Callable[[str], None] = LOG, clock: Callable[[], float] \| No |
| 688 | `phase` | N | def phase(self) -> str: |
| 691 | `status` | N | def status(self) -> dict[str, Any]: |
| 721 | `start` | N | def start(self, layout: dict[str, Any] \| None = None) -> None: |
| 747 | `tick` | N | def tick(self, now: float \| None = None) -> str: |
| 762 | `run` | N | def run(self, *, stop_flag: dict, alive: Callable[[], bool] = ros_context_alive, sleep: Callable[[float], None] = time.sleep, poll_s: float = 0.1, finish_wait_s |
| 796 | `_layout_ts_of` | N | def _layout_ts_of(layout: dict[str, Any]) -> float: |
| 803 | `_layout_is_stale` | N | def _layout_is_stale(self, now: float) -> bool: |
| 808 | `_layout_is_complete` | N | def _layout_is_complete(self, layout: dict[str, Any]) -> bool: |
| 826 | `_layout_stale_reason` | N | def _layout_stale_reason(self, now: float) -> str: |
| 836 | `_poll_layout` | N | def _poll_layout(self, now: float) -> None: |
| 866 | `_plan` | N | def _plan(self, now: float) -> None: |
| 954 | `_drive_tick` | N | def _drive_tick(self, now: float) -> None: |
| 978 | `_after_segment_done` | N | def _after_segment_done(self, now: float) -> None: |
| 993 | `_wait_tick` | N | def _wait_tick(self, now: float) -> None: |
| 1017 | `_mark_segment_done` | N | def _mark_segment_done(self, now: float, *, block: str, elapsed: float, skipped: bool = False, stop_kind: str = "") -> None: |
| 1048 | `_maybe_replay_done` | N | def _maybe_replay_done(self, now: float) -> None: |
| 1068 | `_stop_done_replay` | N | def _stop_done_replay(self, why: str) -> None: |
| 1075 | `_to_wait_info` | N | def _to_wait_info(self, reason: str, now: float, *, retry: bool = False) -> None: |
| 1088 | `_to_finish` | N | def _to_finish(self, now: float) -> None: |
| 1102 | `_odom_watch` | N | def _odom_watch(self, now: float) -> bool: |
| 1126 | `_to_ros_lost` | N | def _to_ros_lost(self, now: float, age: float) -> None: |
| 1145 | `_to_safe_stop` | N | def _to_safe_stop(self, reason: str, now: float) -> None: |
| 1160 | `_publish_car_state` | N | def _publish_car_state(self, now: float, *, force: bool = False) -> None: |
| 1190 | `_current_block` | N | def _current_block(self, *, prefer_prev: bool = False) -> str: |
| 1200 | `_heading_dir` | N | def _heading_dir(self) -> tuple[int, int] \| None: |
| 1207 | `_unknown_violation` | N | def _unknown_violation(self) -> bool: |
| 1220 | `_seed_pose` | N | def _seed_pose(self, layout: dict[str, Any]) -> None: |
| 1231 | `_default_planner` | N | def _default_planner(self, layout: dict[str, Any], from_cell: str, target_cell: str, heading_dir: tuple[int, int] \| None) -> Any: |
| 1238 | `_run_step_loop` | M | def _run_step_loop(loop: StepLoop, *, stop_flag: dict, alive: Callable[[], bool] = ros_context_alive, log: Callable[[str], None] = LOG) -> str: |
| 1250 | `_layout_mode_enabled` | M | def _layout_mode_enabled(cfg: Config, map_path: str \| None) -> bool: |
| 1263 | `_start_layout_listener` | M | def _start_layout_listener(bridge: Any, bb: Blackboard, bus: EventBus, cfg: Config, *, log: Callable[[str], None] = LOG) -> Any: |
| 1294 | `_ros_diagnosis` | M | def _ros_diagnosis(bridge: Any, *, log: Callable[[str], None] = LOG) -> None: |
| 1365 | `_contract_skip_requested` | M | def _contract_skip_requested() -> bool: |
| 1370 | `_endpoint_counts` | M | def _endpoint_counts(bridge: Any, key: str, topic: str) -> tuple[int, int]: |
| 1402 | `_subscription_topics_of` | M | def _subscription_topics_of(bridge: Any) -> list[str] \| None: |
| 1419 | `startup_contract` | M | def startup_contract(bridge: Any, mode: str = "live", *, log: Callable[[str], None] \| None = None) -> tuple[bool, list[dict]]: |
| 1469 | `_format_contract_table` | M | def _format_contract_table(rows: list[dict], *, mode: str = "live", ok: bool = True) -> str: |
| 1486 | `_log_subscriptions` | M | def _log_subscriptions(bridge: Any, *, log: Callable[[str], None] = LOG) -> None: |
| 1501 | `_startup_self_check` | M | def _startup_self_check(bridge: Any, *, log: Callable[[str], None] = LOG) -> bool: |
| 1527 | `_duplicate_dzbrain_instances` | M | def _duplicate_dzbrain_instances() -> list[str]: |
| 1560 | `_wait_for_pose_ready` | M | def _wait_for_pose_ready(bridge: Any, cfg: Config, *, stop_flag: dict, alive: Callable[[], bool] = ros_context_alive, log: Callable[[str], None] = LOG, sleep: C |
| 1617 | `_wait_for_field_layout` | M | def _wait_for_field_layout(bb: Blackboard, cfg: Config, *, stop_flag: dict, alive: Callable[[], bool] = ros_context_alive, log: Callable[[str], None] = LOG, sle |
| 1667 | `build_tasks` | M | def build_tasks(bridge: Any, bb: Blackboard, bus: EventBus, cfg: Config, *, log: Callable[[str], None] = LOG, skip: Iterable[str] \| None = None) -> tuple[TaskGr |
| 1674 | `add` | N | def add(module: str, cls_name: str, label: str) -> None: |
| 1703 | `start_banner` | M | def start_banner(cfg: Config, group: TaskGroup, *, plan: Any = None, layout_mode: bool = False, step_mode: bool = False) -> None: |
| 1745 | `_tele_field` | M | def _tele_field(tele: Any, name: str, default: Any = "—") -> Any: |
| 1754 | `finish_report` | M | def finish_report(bb: Blackboard, group: TaskGroup, bus: EventBus, *, elapsed: float) -> None: |
| 1774 | `run_bench` | M | def run_bench(cfg: Config, *, seconds: float, drive: bool = False, speed_factor: float = 1.0, log: Callable[[str], None] = LOG) -> int: |
| 1853 | `_car_loop` | N | def _car_loop() -> None: |
| 1933 | `run_plan_only` | M | def run_plan_only(cfg: Config, *, map_path: str \| None = None, log: Callable[[str], None] = LOG) -> int: |
| 1973 | `_live_loop` | M | def _live_loop(stop_flag: dict, *, bb: Blackboard, alive: Callable[[], bool] = ros_context_alive, log: Callable[[str], None] = LOG, poll_s: float = 0.2, finish_ |
| 2003 | `run_live` | M | def run_live(cfg: Config, *, log: Callable[[str], None] = LOG, alive: Callable[[], bool] = ros_context_alive, map_path: str \| None = None) -> int: |
| 2187 | `main` | M | def main(argv: list[str] \| None = None) -> int: |

### 3.2 `core/blackboard.py`

**类**：`Entry`@L91、`Blackboard`@L97

| 行 | 名称 | 类型 | 签名（源码原文，≤160 字符） |
|---|---|---|---|
| 100 | `__init__` | N | def __init__(self) -> None: |
| 106 | `set` | N | def set(self, key: str, value: Any, source: str = "", ts: float \| None = None) -> None: |
| 112 | `update` | N | def update(self, source: str = "", **items: Any) -> None: |
| 120 | `get` | N | def get(self, key: str, default: Any = None) -> Any: |
| 125 | `entry` | N | def entry(self, key: str) -> Entry \| None: |
| 129 | `age` | N | def age(self, key: str) -> float \| None: |
| 135 | `get_fresh` | N | def get_fresh(self, key: str, max_age_s: float, default: Any = None) -> Any: |
| 143 | `is_fresh` | N | def is_fresh(self, key: str, max_age_s: float) -> bool: |
| 147 | `snapshot` | N | def snapshot(self, keys: Iterable[str] \| None = None) -> dict[str, Any]: |
| 154 | `describe` | N | def describe(self) -> dict[str, dict]: |

### 3.3 `core/config.py`

**类**：`Spec`@L32、`Issue`@L597、`ConfigError`@L622、`Config`@L626

| 行 | 名称 | 类型 | 签名（源码原文，≤160 字符） |
|---|---|---|---|
| 41 | `check` | N | def check(self, value: Any) -> str \| None: |
| 602 | `__str__` | N | def __str__(self) -> str: |
| 607 | `_flatten` | M | def _flatten(node: Any, prefix: str = "") -> dict[str, Any]: |
| 629 | `__init__` | N | def __init__(self, flat: dict[str, Any], source: str = "<memory>", raw: dict \| None = None) -> None: |
| 639 | `load` | N | def load(cls, path: str \| None = None) -> "Config": |
| 659 | `from_dict` | N | def from_dict(cls, data: dict[str, Any], source: str = "<dict>") -> "Config": |
| 665 | `_migrate_legacy` | N | def _migrate_legacy(self) -> None: |
| 727 | `get` | N | def get(self, key: str, default: Any = None) -> Any: |
| 741 | `require` | N | def require(self, key: str) -> Any: |
| 747 | `_check_against_schema` | N | def _check_against_schema(self, key: str, value: Any) -> None: |
| 756 | `section` | N | def section(self, prefix: str) -> dict[str, Any]: |
| 760 | `keys` | N | def keys(self) -> Iterable[str]: |
| 763 | `override` | N | def override(self, key: str, value: Any) -> "Config": |
| 773 | `validate` | N | def validate(self, strict: bool = True) -> list[Issue]: |
| 775 | `note` | N | def note(level: str, key: str, msg: str) -> None: |
| 849 | `_signatures` | N | def _signatures(self) -> list[tuple[str, str, str]]: |
| 852 | `summary` | N | def summary(self) -> str: |
| 861 | `lookahead_distance` | M | def lookahead_distance(speed: float, cfg: "Config") -> float: |
| 869 | `curve_speed_limit` | M | def curve_speed_limit(curvature: float, cfg: "Config") -> float: |
| 878 | `pivot_dist_eff` | M | def pivot_dist_eff(cfg: "Config", speed_mps: float) -> float: |

### 3.4 `core/events.py`

**类**：`Event`@L52、`Subscription`@L62、`EventBus`@L98

| 行 | 名称 | 类型 | 签名（源码原文，≤160 字符） |
|---|---|---|---|
| 65 | `__init__` | N | def __init__(self, bus: "EventBus", name: str, maxsize: int = 256) -> None: |
| 71 | `poll` | N | def poll(self, max_items: int = 32) -> list[Event]: |
| 81 | `drain` | N | def drain(self, handler: Callable[[Event], None], max_items: int = 32) -> int: |
| 88 | `_offer` | N | def _offer(self, ev: Event) -> None: |
| 94 | `unsubscribe` | N | def unsubscribe(self) -> None: |
| 101 | `__init__` | N | def __init__(self) -> None: |
| 107 | `subscribe` | N | def subscribe(self, name: str, types: list[str] \| None = None, maxsize: int = 256) -> Subscription: |
| 115 | `publish` | N | def publish(self, ev_type: str, source: str = "", **data: Any) -> Event: |
| 128 | `_remove` | N | def _remove(self, sub: Subscription) -> None: |
| 133 | `stats` | N | def stats(self) -> dict: |

### 3.5 `core/geometry.py`

**类**：`Pose`@L172

| 行 | 名称 | 类型 | 签名（源码原文，≤160 字符） |
|---|---|---|---|
| 54 | `block_to_index` | M | def block_to_index(block: str) -> tuple[int, int]: |
| 70 | `index_to_block` | M | def index_to_block(row_idx: int, col_idx: int) -> str: |
| 77 | `block_to_meters` | M | def block_to_meters(block: str) -> tuple[float, float]: |
| 85 | `meters_to_block` | M | def meters_to_block(x: float, y: float) -> str: |
| 92 | `block_to_units` | M | def block_to_units(block: str) -> tuple[float, float]: |
| 98 | `units_to_meters` | M | def units_to_meters(u: float, v: float) -> tuple[float, float]: |
| 103 | `meters_to_units` | M | def meters_to_units(x: float, y: float) -> tuple[float, float]: |
| 108 | `map_bounds` | M | def map_bounds(margin: float = 0.0) -> tuple[float, float, float, float]: |
| 114 | `in_bounds` | M | def in_bounds(x: float, y: float, margin: float = 0.0) -> bool: |
| 122 | `normalize_angle` | M | def normalize_angle(angle: float) -> float: |
| 127 | `ang_diff` | M | def ang_diff(a: float, b: float) -> float: |
| 132 | `deg` | M | def deg(rad: float) -> float: |
| 136 | `rad` | M | def rad(degree: float) -> float: |
| 140 | `yaw_to_quaternion` | M | def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]: |
| 146 | `quaternion_to_yaw` | M | def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float: |
| 153 | `heading_token_to_yaw` | M | def heading_token_to_yaw(token: str) -> float: |
| 161 | `heading_of` | M | def heading_of(dx: float, dy: float, fallback: float = 0.0) -> float: |
| 179 | `block` | N | def block(self) -> str: |
| 182 | `offset_to` | N | def offset_to(self, x: float, y: float) -> tuple[float, float, float]: |
| 187 | `translated` | N | def translated(self, dx: float, dy: float) -> "Pose": |
| 190 | `rotated` | N | def rotated(self, dyaw: float) -> "Pose": |

### 3.6 `core/logging.py`

**类**：`Throttle`@L42、`FaultGuard`@L63

| 行 | 名称 | 类型 | 签名（源码原文，≤160 字符） |
|---|---|---|---|
| 22 | `banner` | M | def banner(lines: list[str] \| str, char: str = "=", log: Callable[[str], None] = print) -> None: |
| 32 | `kv_lines` | M | def kv_lines(data: dict[str, Any], indent: int = 2) -> list[str]: |
| 48 | `allow` | N | def allow(self, key: str, now: float \| None = None) -> bool: |
| 56 | `reset` | N | def reset(self, key: str \| None = None) -> None: |
| 66 | `__init__` | N | def __init__(self) -> None: |
| 69 | `first` | N | def first(self, key: str) -> bool: |
| 76 | `clear` | N | def clear(self, key: str \| None = None) -> None: |

### 3.7 `core/state.py`

**类**：`SafetyState`@L17、`StopKind`@L33、`StopPhase`@L38、`StopPointState`@L46、`TargetState`@L69、`AimState`@L87、`DroneState`@L104、`FixState`@L128、`Telemetry`@L152

| 行 | 名称 | 类型 | 签名（源码原文，≤160 字符） |
|---|---|---|---|
| 25 | `with_reason` | N | def with_reason(self, reason: str, ts: float, *, ok: bool = False) -> "SafetyState": |
| 61 | `active` | N | def active(self) -> bool: |
| 115 | `bridge_of` | N | def bridge_of(self, half: str, *, now: float, stale_s: float) -> str \| None: |
| 143 | `corrected` | N | def corrected(self, x: float, y: float, yaw: float) -> tuple[float, float, float]: |

### 3.8 `core/task.py`

**类**：`TaskState`@L30、`TaskHealth`@L39、`TaskThread`@L59、`TaskGroup`@L195

| 行 | 名称 | 类型 | 签名（源码原文，≤160 字符） |
|---|---|---|---|
| 54 | `age` | N | def age(self, now: float \| None = None) -> float: |
| 65 | `__init__` | N | def __init__( self, name: str, rate_hz: float, *, bus: EventBus \| None = None, blackboard: Blackboard \| None = None, logger: Callable[[str], None] \| None = None |
| 90 | `setup` | N | def setup(self) -> None: |
| 93 | `tick` | N | def tick(self) -> None: |
| 97 | `teardown` | N | def teardown(self) -> None: |
| 100 | `fault_hook` | N | def fault_hook(self, exc: BaseException) -> None: |
| 106 | `run` | N | def run(self) -> None: # noqa: D401 |
| 165 | `_fail` | N | def _fail(self, where: str, exc: BaseException) -> None: |
| 179 | `stop` | N | def stop(self, timeout: float \| None = 2.0) -> bool: |
| 187 | `stopping` | N | def stopping(self) -> bool: |
| 190 | `health` | N | def health(self) -> TaskHealth: |
| 198 | `__init__` | N | def __init__(self, bus: EventBus, blackboard: Blackboard, logger: Callable[[str], None] \| None = None) -> None: |
| 205 | `add` | N | def add(self, task: TaskThread) -> TaskThread: |
| 209 | `start_all` | N | def start_all(self) -> None: |
| 215 | `stop_all` | N | def stop_all(self, timeout: float = 2.0) -> list[str]: |
| 223 | `health` | N | def health(self) -> dict[str, TaskHealth]: |
| 226 | `faulty` | N | def faulty(self) -> list[str]: |
| 229 | `describe` | N | def describe(self) -> list[dict]: |

### 3.9 `io/contract.py`

**类**：`QosSpec`@L25、`Topic`@L48、`AimStatus`@L172、`MissionPhase`@L184、`ShotClass`@L202、`OffsetObservation`@L214、`SliceInfo`@L272、`Coverage`@L297、`ContractSummary`@L445

| 行 | 名称 | 类型 | 签名（源码原文，≤160 字符） |
|---|---|---|---|
| 30 | `describe` | N | def describe(self) -> str: |
| 113 | `topic` | M | def topic(key: str) -> Topic: |
| 123 | `odom_to_pose` | M | def odom_to_pose(pos_x_units: float, pos_y_units: float, quat: tuple[float, float, float, float]) -> geo.Pose: |
| 136 | `pose_to_pursuit_twist` | M | def pose_to_pursuit_twist(linear_x_mps: float, steer_deg: float) -> dict[str, float]: |
| 144 | `clamp_steer_for_chassis` | M | def clamp_steer_for_chassis(steer_deg: float, cfg) -> float: |
| 150 | `chassis_speed_counts` | M | def chassis_speed_counts(linear_x_mps: float) -> int: |
| 165 | `is_brake_command` | M | def is_brake_command(cmd: dict[str, Any]) -> bool: |
| 180 | `name` | N | def name(cls, value: int) -> str: |
| 225 | `has_target` | N | def has_target(self) -> bool: |
| 229 | `bearing_px` | N | def bearing_px(self) -> int: |
| 233 | `decode_offset_center` | M | def decode_offset_center(data, ts: float = 0.0) -> OffsetObservation \| None: |
| 252 | `decode_aim_status` | M | def decode_aim_status(value) -> str: |
| 286 | `valid` | N | def valid(self) -> bool: |
| 289 | `to_dict` | N | def to_dict(self) -> dict: |
| 321 | `key` | N | def key(self) -> tuple[str, str] \| None: |
| 327 | `to_dict` | N | def to_dict(self) -> dict: |
| 341 | `_norm_coverage_cells` | M | def _norm_coverage_cells(items) -> tuple[tuple[str, ...], int]: |
| 372 | `decode_slice_info` | M | def decode_slice_info(value) -> SliceInfo \| None: |
| 390 | `decode_coverage` | M | def decode_coverage(value, *, scan_id=None, slice_info=None) -> Coverage \| None: |
| 450 | `render` | N | def render(self) -> list[str]: |

### 3.10 `io/referee.py`

**类**：`RefereeFrame`@L74、`RefereeDelta`@L95、`RefereeTracker`@L107

| 行 | 名称 | 类型 | 签名（源码原文，≤160 字符） |
|---|---|---|---|
| 85 | `bing_name` | N | def bing_name(self) -> str: |
| 89 | `being_hit` | N | def being_hit(self) -> bool: |
| 103 | `__bool__` | N | def __bool__(self) -> bool: |
| 116 | `__init__` | N | def __init__(self) -> None: |
| 129 | `state` | N | def state(self) -> RefereeFrame \| None: |
| 133 | `valid_seen` | N | def valid_seen(self) -> bool: |
| 142 | `ammo` | N | def ammo(self) -> int \| None: |
| 146 | `age` | N | def age(self, now: float) -> float: |
| 153 | `feed` | N | def feed(self, data: Sequence[int], now: float) -> RefereeDelta: |
| 196 | `describe` | N | def describe(self) -> str: |

### 3.11 `io/ros_io.py`

**类**：`RosStats`@L149、`RosBridge`@L170

| 行 | 名称 | 类型 | 签名（源码原文，≤160 字符） |
|---|---|---|---|
| 36 | `required_subscription_topics` | M | def required_subscription_topics() -> dict[str, str]: |
| 46 | `watched_topic_names` | M | def watched_topic_names() -> dict[str, str]: |
| 50 | `ros_env` | M | def ros_env() -> dict[str, str]: |
| 67 | `env_problems` | M | def env_problems(env: dict[str, str] \| None = None) -> tuple[list[str], list[str]]: |
| 97 | `format_env_line` | M | def format_env_line(env: dict[str, str] \| None = None) -> str: |
| 114 | `_signal_handler_options` | M | def _signal_handler_options(): |
| 134 | `_qos` | M | def _qos(spec: ct.QosSpec): |
| 173 | `__init__` | N | def __init__(self, node_name: str = "dzbrain", *, log: Callable[[str], None] = print) -> None: |
| 221 | `_create_subs` | N | def _create_subs(self) -> None: |
| 229 | `on_odom` | N | def on_odom(msg: Odometry) -> None: |
| 241 | `on_imu` | N | def on_imu(msg: Imu) -> None: |
| 249 | `on_vision` | N | def on_vision(msg: Int32MultiArray) -> None: |
| 258 | `on_aim_status` | N | def on_aim_status(msg: Int32) -> None: |
| 263 | `on_shoot_count` | N | def on_shoot_count(msg: Int32) -> None: |
| 267 | `on_hit` | N | def on_hit(msg: Int32) -> None: |
| 276 | `on_ammo` | N | def on_ammo(msg: Int32) -> None: |
| 284 | `on_referee_hp` | N | def on_referee_hp(msg: UInt8MultiArray) -> None: |
| 303 | `on_manual_override` | N | def on_manual_override(msg: UInt8) -> None: |
| 307 | `on_manual_gimbal` | N | def on_manual_gimbal(msg: UInt8) -> None: |
| 311 | `on_drone` | N | def on_drone(msg: String) -> None: |
| 318 | `on_waypoint` | N | def on_waypoint(msg: String) -> None: |
| 351 | `_create_pubs` | N | def _create_pubs(self) -> None: |
| 378 | `pose` | N | def pose(self) -> geo.Pose \| None: |
| 382 | `pose_age` | N | def pose_age(self) -> float: |
| 386 | `imu` | N | def imu(self) -> tuple[float, float, float, float, float, float] \| None: |
| 390 | `imu_age` | N | def imu_age(self) -> float: |
| 394 | `vision` | N | def vision(self, max_age_s: float = 0.5) -> ct.OffsetObservation \| None: |
| 402 | `vision_raw` | N | def vision_raw(self) -> ct.OffsetObservation \| None: |
| 406 | `aim_status` | N | def aim_status(self) -> int: |
| 410 | `aim_status_name` | N | def aim_status_name(self) -> str: |
| 413 | `shoot_count` | N | def shoot_count(self) -> int: |
| 417 | `ammo` | N | def ammo(self) -> int \| None: |
| 421 | `take_hits` | N | def take_hits(self) -> int: |
| 428 | `referee` | N | def referee(self) -> RefereeTracker: |
| 432 | `referee_age` | N | def referee_age(self) -> float: |
| 436 | `referee_shots_hw` | N | def referee_shots_hw(self) -> int: |
| 441 | `manual_override` | N | def manual_override(self) -> bool: |
| 445 | `manual_gimbal` | N | def manual_gimbal(self) -> bool: |
| 449 | `take_drone_messages` | N | def take_drone_messages(self, max_items: int = 16) -> list[str]: |
| 455 | `take_waypoint_commands` | N | def take_waypoint_commands(self, max_items: int = 8) -> list[str]: |
| 464 | `publish_command` | N | def publish_command(self, linear_x: float, steer_deg: float) -> None: |
| 474 | `publish_brake` | N | def publish_brake(self) -> None: |
| 488 | `publish_scan_request` | N | def publish_scan_request(self, mode: int) -> None: |
| 495 | `publish_fire_gate` | N | def publish_fire_gate(self, allow: bool) -> None: |
| 504 | `publish_fire_ready` | N | def publish_fire_ready(self, ready: bool) -> None: |
| 517 | `publish_motion_state` | N | def publish_motion_state(self, reverse: bool) -> None: |
| 523 | `publish_voice` | N | def publish_voice(self, voice_id: int) -> None: |
| 529 | `publish_drone` | N | def publish_drone(self, payload: str) -> None: |
| 535 | `publish_lead` | N | def publish_lead(self, lead_px: float) -> None: |
| 545 | `spin_async` | N | def spin_async(self, threads: int = 4): |
| 556 | `shutdown` | N | def shutdown(self, *, brake: bool = True) -> None: |
| 588 | `topic_counts` | N | def topic_counts(self) -> dict[str, dict[str, int]]: |
| 604 | `subscription_topics` | N | def subscription_topics(self) -> list[str]: |
| 617 | `summary` | N | def summary(self) -> dict[str, Any]: |

### 3.12 `route/model.py`

**类**：`Waypoint`@L21、`Route`@L64、`RouteBuilder`@L139

| 行 | 名称 | 类型 | 签名（源码原文，≤160 字符） |
|---|---|---|---|
| 44 | `at_block` | N | def at_block(cls, block: str, *, yaw: float \| None = None, **kw) -> "Waypoint": |
| 50 | `is_stop_point` | N | def is_stop_point(self) -> bool: |
| 53 | `distance_to` | N | def distance_to(self, x: float, y: float) -> float: |
| 56 | `with_yaw` | N | def with_yaw(self, yaw: float) -> "Waypoint": |
| 59 | `heading_to` | N | def heading_to(self, other: "Waypoint") -> float: |
| 73 | `__len__` | N | def __len__(self) -> int: |
| 76 | `__iter__` | N | def __iter__(self) -> Iterator[Waypoint]: |
| 79 | `__getitem__` | N | def __getitem__(self, idx): |
| 82 | `__bool__` | N | def __bool__(self) -> bool: |
| 86 | `blocks` | N | def blocks(self) -> list[str]: |
| 89 | `tokens` | N | def tokens(self) -> list[str]: |
| 93 | `start` | N | def start(self) -> Waypoint \| None: |
| 97 | `end` | N | def end(self) -> Waypoint \| None: |
| 100 | `stop_points` | N | def stop_points(self) -> list[tuple[int, Waypoint]]: |
| 103 | `shoot_points` | N | def shoot_points(self) -> list[tuple[int, Waypoint]]: |
| 106 | `reverse_segments` | N | def reverse_segments(self) -> list[int]: |
| 111 | `pivot_points` | N | def pivot_points(self) -> list[tuple[int, "Waypoint"]]: |
| 115 | `pivot_blocks` | N | def pivot_blocks(self) -> list[str]: |
| 119 | `path_length` | N | def path_length(self) -> float: |
| 125 | `describe` | N | def describe(self, limit: int = 40) -> str: |
| 132 | `as_list` | N | def as_list(self) -> list[Waypoint]: |
| 135 | `slice` | N | def slice(self, start: int, end: int \| None = None) -> "Route": |
| 142 | `__init__` | N | def __init__(self, name: str = "") -> None: |
| 147 | `add` | N | def add(self, waypoint: Waypoint) -> "RouteBuilder": |
| 151 | `add_block` | N | def add_block(self, block: str, **kw) -> "RouteBuilder": |
| 154 | `extend` | N | def extend(self, waypoints: Iterable[Waypoint]) -> "RouteBuilder": |
| 159 | `add_route` | N | def add_route(self, route: Route) -> "RouteBuilder": |
| 162 | `crossing` | N | def crossing(self, half: str, block: str) -> "RouteBuilder": |
| 166 | `__len__` | N | def __len__(self) -> int: |
| 169 | `build` | N | def build(self, *, fix_final_yaw: float \| None = None, drop_consecutive_duplicates: bool = True) -> Route: |
| 198 | `make_route` | M | def make_route(waypoints: Sequence[Waypoint], name: str = "", crossing: dict \| None = None) -> Route: |

### 3.13 `route/parser.py`

**类**：（无）

| 行 | 名称 | 类型 | 签名（源码原文，≤160 字符） |
|---|---|---|---|
| 96 | `normalize_token` | M | def normalize_token(token: str) -> str: |
| 161 | `_normalize_shoot_directive` | M | def _normalize_shoot_directive(token: str) -> str: |
| 170 | `_split_block_suffix` | M | def _split_block_suffix(block: str, raw: str) -> tuple[str, str \| None]: |
| 183 | `parse_token` | M | def parse_token(token: str) -> dict: |
| 246 | `_looks_like_file` | M | def _looks_like_file(spec: str) -> bool: |
| 256 | `_read_token_file` | M | def _read_token_file(path: str) -> list[str]: |
| 274 | `_as_token_list` | M | def _as_token_list(tokens: Any) -> list[str]: |
| 289 | `_call_shoot_planner` | M | def _call_shoot_planner(planner: Callable[[str], Any], target: str) -> list[str]: |
| 306 | `parse_mission` | M | def parse_mission(tokens, *, shoot_planner: Callable[[str], Any] \| None = None, name: str = "mission") -> Route: |
| 383 | `_dedup_waypoints` | M | def _dedup_waypoints(route: Route) -> Route: |
| 409 | `detect_crossing` | M | def detect_crossing(route: Route) -> dict: |
| 441 | `parse_reset` | M | def parse_reset(spec: str) -> tuple[str, float]: |
| 461 | `format_route` | M | def format_route(route: Route) -> list[str]: |

### 3.14 `route/validate.py`

**类**：`Issue`@L379

| 行 | 名称 | 类型 | 签名（源码原文，≤160 字符） |
|---|---|---|---|
| 75 | `_fallback_cfg` | M | def _fallback_cfg() -> Config: |
| 83 | `_cfg_get` | M | def _cfg_get(cfg: Config \| None, key: str, fallback: Any = None) -> Any: |
| 93 | `_window` | M | def _window(cfg: Config \| None) -> tuple[float, float]: |
| 101 | `_as_list` | M | def _as_list(value: Any) -> list[str]: |
| 111 | `_cross_rows` | M | def _cross_rows(cfg: Config \| None, cross_candidates: Iterable[str] \| None = None) -> tuple[list[int], list[int]]: |
| 148 | `_map_section` | M | def _map_section(map_cfg: Any) -> dict: |
| 171 | `_cells` | M | def _cells(merged: dict, key: str) -> list[str]: |
| 178 | `_rc` | M | def _rc(block: str) -> tuple[int, int]: |
| 184 | `_sign` | M | def _sign(value: int) -> int: |
| 188 | `_blocks` | M | def _blocks(route: Any) -> list[str]: |
| 201 | `_barrier_crossing` | M | def _barrier_crossing(a: str, b: str) -> tuple[bool, float, float] \| None: |
| 227 | `_crossing_row_of_cell` | M | def _crossing_row_of_cell(cell: str \| None) -> float \| None: |
| 238 | `_is_cross` | M | def _is_cross(a: str, b: str) -> bool: |
| 275 | `_crossing_row` | M | def _crossing_row(a: str, b: str) -> float \| None: |
| 297 | `_crossing_cell` | M | def _crossing_cell(a: str, b: str) -> str \| None: |
| 312 | `_crossing_at_column` | M | def _crossing_at_column(a: str, b: str) -> tuple[float, float, float, float] \| None: |
| 345 | `crossing_entries` | M | def crossing_entries(blocks: Sequence[str] ) -> list[tuple[str, str, tuple[float, float, float, float]]]: |
| 386 | `__str__` | N | def __str__(self) -> str: |
| 391 | `validate_route` | M | def validate_route(route: Any, *, map_cfg: Any = None, cfg: Config \| None = None, cross_candidates: Iterable[str] \| None = None) -> list[Issue]: |
| 595 | `_fmt` | N | def _fmt(run_list: list[dict]) -> str: |
| 731 | `_euc` | M | def _euc(wp: Any, block: str) -> float: |
| 736 | `_stop_ok` | M | def _stop_ok(wp: Any, target: str, lo: float, hi: float) -> bool: |
| 742 | `_arrives_reverse` | M | def _arrives_reverse(route: Any, idx: int) -> bool: |
| 748 | `_total_length` | M | def _total_length(blocks: Sequence[str]) -> float: |
| 760 | `summary` | M | def summary(issues: Iterable[Issue]) -> str: |

### 3.15 `route/physics.py`

**类**：（无）

| 行 | 名称 | 类型 | 签名（源码原文，≤160 字符） |
|---|---|---|---|
| 89 | `point_segment_distance_m` | M | def point_segment_distance_m(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float: |
| 101 | `point_rect_distance_m` | M | def point_rect_distance_m(px: float, py: float, cx: float, cy: float, half: float) -> float: |
| 114 | `segment_rect_distance_m` | M | def segment_rect_distance_m(ax: float, ay: float, bx: float, by: float, cx: float, cy: float, half: float) -> float: |
| 133 | `_segment_rect_intersects` | M | def _segment_rect_intersects(ax: float, ay: float, bx: float, by: float, cx: float, cy: float, half: float) -> bool: |
| 162 | `vehicle_half_diagonal_m` | M | def vehicle_half_diagonal_m(length_m: float = 0.28, width_m: float = 0.135) -> float: |
| 170 | `sweep_radius` | M | def sweep_radius(*, length_m: float = 0.28, width_m: float = 0.135, wheelbase_m: float = 0.157, max_steer_deg: float = 30.0 ) -> tuple[float, float]: |
| 190 | `mine_clearance_m` | M | def mine_clearance_m(px: float, py: float, mine_x: float, mine_y: float, *, mine_pad_m: float = 0.12) -> float: |
| 196 | `mine_clearance_segment_m` | M | def mine_clearance_segment_m(ax: float, ay: float, bx: float, by: float, mine_x: float, mine_y: float, *, mine_pad_m: float = 0.12) -> float: |
| 203 | `mine_pressed` | M | def mine_pressed(px: float, py: float, mine_x: float, mine_y: float, *, coverage_radius_m: float = 0.035, mine_pad_m: float = 0.12) -> bool: |
| 216 | `obstacle_clearance_point_m` | M | def obstacle_clearance_point_m(px: float, py: float, ox: float, oy: float) -> float: |
| 221 | `obstacle_clearance_segment_m` | M | def obstacle_clearance_segment_m(ax: float, ay: float, bx: float, by: float, ox: float, oy: float) -> float: |
| 227 | `required_obstacle_center_distance_m` | M | def required_obstacle_center_distance_m(*, obstacle_radius_m: float = 0.1275, body_radius_m: float = 0.0675, margin_m: float = 0.02) -> float: |
| 241 | `parse_col_span` | M | def parse_col_span(spec: str) -> tuple[str, str]: |
| 257 | `barrier_centerline` | M | def barrier_centerline(*, col_from: str = "D", col_to: str = "N", row_hi: int = 6, row_lo: int = 7) -> tuple[float, float, float]: |
| 273 | `barrier_distance_point_m` | M | def barrier_distance_point_m(px: float, py: float, *, x: float = 1.75, y0: float = 0.0, y1: float = 3.85) -> float: |
| 279 | `barrier_distance_segment_m` | M | def barrier_distance_segment_m(ax: float, ay: float, bx: float, by: float, *, x: float = 1.75, y0: float = 0.0, y1: float = 3.85) -> float: |
| 286 | `required_barrier_distance_m` | M | def required_barrier_distance_m(*, barrier_width_m: float = 0.02, car_width_m: float = 0.135) -> float: |
| 302 | `k_cells_sealed_along_column` | M | def k_cells_sealed_along_column() -> bool: |
| 319 | `board_half_m` | M | def board_half_m() -> float: |
| 329 | `required_target_body_clearance_m` | M | def required_target_body_clearance_m(*, length_m: float = 0.28, width_m: float = 0.135, at_corner: bool = False) -> float: |
| 341 | `target_body_overlap` | M | def target_body_overlap(clearance_to_board_m: float, *, length_m: float = 0.28, width_m: float = 0.135, at_corner: bool = False) -> bool: |
| 348 | `body_overlaps_rect_centered` | M | def body_overlaps_rect_centered(px: float, py: float, cx: float, cy: float, *, half: float, length_m: float = 0.28, width_m: float = 0.135) -> bool: |
| 359 | `_seg_seg_distance` | M | def _seg_seg_distance(ax: float, ay: float, bx: float, by: float, cx: float, cy: float, dx: float, dy: float) -> float: |
| 371 | `_segments_intersect` | M | def _segments_intersect(ax: float, ay: float, bx: float, by: float, cx: float, cy: float, dx: float, dy: float) -> bool: |
| 373 | `cross` | N | def cross(ox, oy, px, py, qx, qy) -> float: |
| 382 | `segment_outside_band` | M | def segment_outside_band(ax: float, ay: float, bx: float, by: float, *, band_y: float, band_half: float ) -> list[tuple[float, float, float, float]]: |
| 435 | `body_center` | M | def body_center(px: float, py: float, yaw: float, *, offset_m: float = DEFAULT_REF_OFFSET_M) -> tuple[float, float]: |
| 441 | `body_corners` | M | def body_corners(px: float, py: float, yaw: float, *, length_m: float = 0.28, width_m: float = 0.135, offset_m: float = DEFAULT_REF_OFFSET_M ) -> tuple[tuple[fl |
| 457 | `corner_poly` | M | def corner_poly(cx: float, cy: float, half_x: float, half_y: float ) -> tuple[tuple[float, float], ...]: |
| 464 | `_poly_axes` | M | def _poly_axes(poly) -> list[tuple[float, float]]: |
| 478 | `_poly_proj` | M | def _poly_proj(poly, ax: tuple[float, float]) -> tuple[float, float]: |
| 483 | `polygon_overlap_depth` | M | def polygon_overlap_depth(a, b) -> float: |
| 500 | `polygon_distance_m` | M | def polygon_distance_m(a, b) -> float: |
| 517 | `_point_in_poly` | M | def _point_in_poly(px: float, py: float, poly) -> bool: |
| 530 | `body_obstacle_clearance_m` | M | def body_obstacle_clearance_m(px: float, py: float, yaw: float, ox: float, oy: float, *, obstacle_radius_m: float = 0.1275, length_m: float = 0.28, width_m: flo |
| 549 | `body_barrier_clearance_m` | M | def body_barrier_clearance_m(px: float, py: float, yaw: float, *, barrier_width_m: float = 0.02, barrier_x: float = 1.75, barrier_y0: float = 0.0, barrier_y1: f |
| 567 | `body_rect_clearance_m` | M | def body_rect_clearance_m(px: float, py: float, yaw: float, *, rect_cx: float, rect_cy: float, rect_half_x: float, rect_half_y: float, length_m: float = 0.28, w |
| 606 | `bridge_entry_walls` | M | def bridge_entry_walls(block: str, redundancy_m: float ) -> tuple[tuple[float, float, float, float], |
| 637 | `body_bridge_wall_clearance_m` | M | def body_bridge_wall_clearance_m(px: float, py: float, yaw: float, walls, *, length_m: float = 0.28, width_m: float = 0.135, offset_m: float = DEFAULT_REF_OFFSE |

### 3.16 `route/planner.py`

**类**：`PlannerError`@L286、`MapSpec`@L294、`Grid`@L370、`_Stop`@L1338、`PlanResult`@L1893、`_Tour`@L2040、`_TSGrid`@L3714

| 行 | 名称 | 类型 | 签名（源码原文，≤160 字符） |
|---|---|---|---|
| 310 | `map_section` | M | def map_section(map_cfg: Any) -> dict: |
| 315 | `_as_cells` | M | def _as_cells(value: Any) -> tuple[str, ...]: |
| 320 | `_checked` | M | def _checked(cells: Iterable[str], group: str) -> tuple[str, ...]: |
| 329 | `load_map` | M | def load_map(map_cfg: Any, *, source: str = "") -> MapSpec: |
| 348 | `_int` | N | def _int(key: str, fallback: int) -> int: |
| 377 | `__init__` | N | def __init__(self, spec: MapSpec, bridge_rows: Iterable[int] = (), *, allow_reverse: bool = True, barrier_turn_margin_m: float = 0.0) -> None: |
| 436 | `in_bounds` | N | def in_bounds(self, row: int, col: int) -> bool: |
| 439 | `passable` | N | def passable(self, row: int, col: int) -> bool: |
| 442 | `barrier_cross_ok` | N | def barrier_cross_ok(self, row_from: int, row_to: int, col: int) -> bool: |
| 448 | `zone_of` | N | def zone_of(self, row: int) -> int: |
| 452 | `move_ok` | N | def move_ok(self, row: int, col: int, nrow: int, ncol: int) -> bool: |
| 489 | `danger` | N | def danger(self, row: int, col: int) -> float: |
| 492 | `_build_danger` | N | def _build_danger(self) -> list[list[float]]: |
| 536 | `_barrier_turn_cell` | M | def _barrier_turn_cell(grid: Grid, row: int, col: int, prev_head: int, head: int, margin_m: float \| None = None) -> bool: |
| 614 | `_door_turn_margin_m` | M | def _door_turn_margin_m(grid: Grid, row: int, col: int) -> float: |
| 632 | `_phys_params` | M | def _phys_params(cfg: "Config \| None") -> dict: |
| 640 | `g` | N | def g(key: str, fallback: float \| str) -> Any: |
| 664 | `_door_turn_required_margin_m` | M | def _door_turn_required_margin_m(turn_deg: float, phys: dict) -> float: |
| 686 | `_turn_deg_from_dirs` | M | def _turn_deg_from_dirs(entry: int, leave: int) -> float: |
| 695 | `_door_turn_margin_m` | M | def _door_turn_margin_m(grid: Grid, row: int, col: int, *, turn_deg: float = 90.0, phys: "dict \| None" = None) -> float: |
| 712 | `_obstacle_turn_clearance_m` | M | def _obstacle_turn_clearance_m(grid: Grid, cell: tuple[int, int], obstacles: Iterable[tuple[int, int]], *, at_corner: bool = True, phys: "dict \| None" = None) - |
| 734 | `_required_obstacle_clearance_m` | M | def _required_obstacle_clearance_m(phys: dict, *, at_corner: bool) -> float: |
| 745 | `_mine_pressed_at` | M | def _mine_pressed_at(cell: tuple[int, int], mine_cell: tuple[int, int], phys: dict) -> bool: |
| 760 | `_edge_mine_turn_gap` | M | def _edge_mine_turn_gap(seq_cell: tuple[int, int], edge_mines: Iterable[tuple[int, int]]) -> int: |
| 775 | `_edge_mine_cells` | M | def _edge_mine_cells(grid: Grid, spec: MapSpec) -> set[tuple[int, int]]: |
| 786 | `_moves` | M | def _moves(grid: Grid, row: int, col: int, head: int, prev_head: int = _STATE_START, prev_valid: bool = False, barrier_turn_penalty: float = 0.0): |
| 820 | `flood` | M | def flood(grid: Grid, source: tuple[int, int, int], barrier_turn_penalty: float = 0.0): |
| 874 | `_reconstruct` | M | def _reconstruct(dist: dict, came: dict, actions: dict, goal: tuple[int, int, int], rank: dict \| None = None) -> tuple[list[tuple[int, int, int, int]], float]: |
| 905 | `_flood_value` | M | def _flood_value(dist: dict, goal: tuple[int, int, int], rank: dict \| None = None) -> float: |
| 922 | `_heading_of` | M | def _heading_of(drow: int, dcol: int, fallback: int) -> int: |
| 935 | `_path_reverse` | M | def _path_reverse(path: Sequence[tuple[int, int, int, int]]) -> list[bool]: |
| 943 | `stop_distance_m` | M | def stop_distance_m(block_a: str, block_b: str) -> float: |
| 952 | `turn_angle_deg` | M | def turn_angle_deg(start: tuple[int, int], mid: tuple[int, int], end: tuple[int, int]) -> float: |
| 964 | `plate_center_m` | M | def plate_center_m(spec: MapSpec) -> tuple[float, float]: |
| 988 | `shoot_distance_window` | M | def shoot_distance_window(accept: tuple[float, float] \| None = None) -> tuple[float, float]: |
| 1001 | `plate_bisector_axis_m` | M | def plate_bisector_axis_m(spec: MapSpec) -> tuple[float, float]: |
| 1019 | `plate_face_normal_m` | M | def plate_face_normal_m(spec: MapSpec) -> tuple[float, float]: |
| 1035 | `barrier_sight_blocked` | M | def barrier_sight_blocked(spec: MapSpec, p: tuple[float, float], q: tuple[float, float]) -> bool: |
| 1088 | `shoot_candidates` | M | def shoot_candidates(spec: MapSpec, grid: Grid, accept: tuple[float, float] \| None = None, *, pool: Sequence[str] \| None = None) -> list[str]: |
| 1158 | `accept_window` | M | def accept_window(limits: tuple[float, float]) -> tuple[float, float]: |
| 1168 | `covering_stops` | M | def covering_stops(spec: MapSpec, pool: Sequence[str], accept: tuple[float, float] \| None = None) -> dict[str, tuple[str, ...]]: |
| 1189 | `plate_side_of` | M | def plate_side_of(spec: MapSpec, block: str) -> int: |
| 1216 | `second_shoot_stop` | M | def second_shoot_stop(spec: MapSpec, grid: Grid, primary: str, accept: tuple[float, float] \| None = None) -> str: |
| 1251 | `shoot_stop_distance` | M | def shoot_stop_distance(block: str, map_cfg: Any, *, source: str = "") -> float: |
| 1263 | `ensure_second_shoot_stop` | M | def ensure_second_shoot_stop(tokens: Sequence[str], map_cfg: Any, cfg: "Config \| None" = None, *, source: str = "") -> tuple[list[str], str, str]: |
| 1360 | `_collapse` | M | def _collapse(path: Sequence[tuple[int, int, int, int]], keep: set[tuple[int, int]]) -> list[tuple[int, int, int, bool]]: |
| 1413 | `_split_long_hops` | M | def _split_long_hops(stops: list[_Stop], grid: Grid, max_blocks: int = MAX_HOP_BLOCKS) -> list[_Stop]: |
| 1446 | `_fix_uturn_reverse` | M | def _fix_uturn_reverse(stops: list[_Stop]) -> int: |
| 1475 | `_merge_duplicate_stops` | M | def _merge_duplicate_stops(stops: list[_Stop]) -> list[_Stop]: |
| 1497 | `_barrier_turn_stops` | M | def _barrier_turn_stops(grid: Grid, stops: Sequence["_Stop"]) -> list[str]: |
| 1519 | `mark_pivots` | M | def mark_pivots(stops: list[_Stop], *, min_deg: float = PIVOT_MARK_DEG) -> list[tuple[int, str, float]]: |
| 1544 | `_refresh_headings` | M | def _refresh_headings(stops: list[_Stop]) -> None: |
| 1577 | `_final_heading` | M | def _final_heading(stops: Sequence[_Stop]) -> int \| None: |
| 1582 | `_cell` | M | def _cell(block: str) -> tuple[int, int]: |
| 1587 | `_to_route` | M | def _to_route(stops: Sequence[_Stop], *, name: str, crossing: dict, final_heading: int \| None = None) -> Route: |
| 1611 | `route_tokens` | M | def route_tokens(route: Route, *, final_heading: bool = True) -> list[str]: |
| 1632 | `check_route_structure` | M | def check_route_structure(route: Route, spec: MapSpec, grid: Grid, limits: tuple[float, float], acc: tuple[float, float] \| None = None, *, expected_start: str \| |
| 1772 | `assert_legal` | M | def assert_legal(route: Route) -> None: |
| 1806 | `_angle_of` | M | def _angle_of(a: Waypoint, b: Waypoint, c: Waypoint) -> float: |
| 1815 | `pivot_vertices` | M | def pivot_vertices(route: Route, *, min_deg: float = PIVOT_MARK_DEG ) -> list[tuple[int, str, float]]: |
| 1838 | `assert_legal_no_reverse` | M | def assert_legal_no_reverse(route: Route) -> None: |
| 1955 | `pivot_count` | N | def pivot_count(self) -> int: |
| 1958 | `shoot_blocks` | N | def shoot_blocks(self) -> list[str]: |
| 1966 | `as_dict` | N | def as_dict(self) -> dict: |
| 2012 | `summary` | N | def summary(self) -> str: |
| 2051 | `__init__` | N | def __init__(self, grid: Grid, deadline: float, pivot_penalty: float = 0.0, record_dirs: bool = True, mine_cells: Iterable[tuple[int, int]] = (), mine_turn_pena |
| 2103 | `_flood_of` | N | def _flood_of(self, cell: tuple[int, int]) -> tuple[dict, dict, dict, int, dict]: |
| 2114 | `cost` | N | def cost(self, a: tuple[int, int], b: tuple[int, int], ha: int, hb: int) -> float: |
| 2121 | `path` | N | def path(self, a: tuple[int, int], b: tuple[int, int], ha: int, hb: int ) -> tuple[list[tuple[int, int, int, int]], float]: |
| 2130 | `build` | N | def build(self, cells: Sequence[tuple[int, int]]) -> None: |
| 2152 | `_record_leg_dirs` | N | def _record_leg_dirs(self, a: tuple[int, int], b: tuple[int, int]) -> None: |
| 2183 | `leg_cost` | N | def leg_cost(self, a: tuple[int, int], b: tuple[int, int]) -> float: |
| 2202 | `reversals` | N | def reversals(self, seq: Sequence[tuple[int, int]]) -> int: |
| 2214 | `penalty_of` | N | def penalty_of(self, seq: Sequence[tuple[int, int]]) -> float: |
| 2280 | `order_cost_total` | N | def order_cost_total(self, seq: Sequence[tuple[int, int]]) -> float: |
| 2293 | `seq_cost` | N | def seq_cost(self, seq: Sequence[tuple[int, int]]) -> float: |
| 2302 | `nearest` | N | def nearest(self, first: tuple[int, int], nodes: Sequence[tuple[int, int]]) -> list[tuple[int, int]]: |
| 2317 | `two_opt` | N | def two_opt(self, seq: list[tuple[int, int]], *, keep_ends: bool = True) -> list[tuple[int, int]]: |
| 2346 | `pair_cost` | N | def pair_cost(self, a: tuple[int, int], b: tuple[int, int], ha: int, hb: int) -> float: |
| 2354 | `route_cost` | N | def route_cost(self, order: Sequence[tuple[int, int]], heads: Sequence[int]) -> float: |
| 2359 | `best_headings` | N | def best_headings(self, order: Sequence[tuple[int, int]], seed: Sequence[int] \| None = None, fixed: dict[tuple[int, int], int] \| None = None) -> list[int]: |
| 2394 | `polish` | N | def polish(self, order: list[tuple[int, int]], heads: Sequence[int], rounds: int = 2) -> tuple[list[tuple[int, int]], list[int], float]: |
| 2413 | `_limits` | M | def _limits(cfg: Config \| None) -> tuple[float, float]: |
| 2423 | `_shoot_accept` | M | def _shoot_accept(cfg: Config \| None) -> tuple[float, float]: |
| 2435 | `_cross_pools` | M | def _cross_pools(cfg: Config \| None, spec: MapSpec) -> tuple[list[str], list[str]]: |
| 2445 | `split` | N | def split(items: Iterable[str]) -> tuple[list[str], list[str]]: |
| 2481 | `_default_cross` | M | def _default_cross(cfg: Config \| None, pool: Sequence[str], half: str) -> str: |
| 2489 | `_u_shape_order` | M | def _u_shape_order(tour: "_Tour", start: tuple[int, int], end: tuple[int, int], mines_lower: Sequence[tuple[int, int]], mines_upper: Sequence[tuple[int, int]],  |
| 2543 | `objective_mode` | M | def objective_mode(value: object) -> str: |
| 2551 | `_side_split` | M | def _side_split(cells: Sequence[tuple[int, int]]) -> tuple[list[tuple[int, int]], |
| 2560 | `_side_shape_order` | M | def _side_shape_order(tour: "_Tour", start: tuple[int, int], end: tuple[int, int], mines_lower: Sequence[tuple[int, int]], mines_upper: Sequence[tuple[int, int] |
| 2571 | `walk` | N | def walk(anchor: tuple[int, int], nodes: Sequence[tuple[int, int]] ) -> tuple[list[tuple[int, int]], tuple[int, int]]: |
| 2602 | `_shoot_positions_for` | M | def _shoot_positions_for(order: Sequence[tuple[int, int]], cell: tuple[int, int], cross_lower: tuple[int, int] \| None, cross_upper: tuple[int, int] \| None) -> l |
| 2633 | `_insert_stop_ranged` | M | def _insert_stop_ranged(tour: "_Tour", order: list[tuple[int, int]], cell: tuple[int, int], positions: Sequence[int] ) -> list[tuple[int, int]]: |
| 2656 | `_insert_shoot_sided` | M | def _insert_shoot_sided(tour: "_Tour", order: list[tuple[int, int]], pool: Sequence[str], cross_lower: tuple[int, int], cross_upper: tuple[int, int] ) -> tuple[ |
| 2693 | `_insert_second_sided` | M | def _insert_second_sided(tour: "_Tour", order: list[tuple[int, int]], block: str, cross_lower: tuple[int, int], cross_upper: tuple[int, int]) -> list[tuple[int, |
| 2709 | `_reachable_any` | M | def _reachable_any(dist: dict, rank: dict, cell: tuple[int, int]) -> bool: |
| 2714 | `_pick_crossing_tasks` | M | def _pick_crossing_tasks(spec: MapSpec, cfg: "Config \| None", lower_pool: Sequence[str], upper_pool: Sequence[str], *, allow_reverse: bool, barrier_turn_margin_ |
| 2726 | `preferred` | N | def preferred(pool: Sequence[str], half: str) -> str: |
| 2765 | `_best_effort_result` | M | def _best_effort_result(spec: MapSpec, cfg: "Config \| None", grid: Grid, *, name: str, source: str, t0: float, allow_reverse: bool, reason: str, warnings: Seque |
| 2783 | `end_dist` | N | def end_dist(cell: tuple[int, int]) -> float: |
| 2849 | `plan_route` | M | def plan_route(map_cfg: Any, cfg: Config \| None = None, *, cross_lower: str \| None = None, cross_upper: str \| None = None, max_seconds: float = DEFAULT_MAX_SECO |
| 3377 | `_insert_shoot` | M | def _insert_shoot(tour: _Tour, order: list[tuple[int, int]], heads: Sequence[int], pool: Sequence[str]) -> tuple[list[tuple[int, int]], list[int], str]: |
| 3428 | `_axis_pass_ok` | M | def _axis_pass_ok(grid: Grid, cell: tuple[int, int], d: tuple[int, int] ) -> tuple[bool, tuple[int, int], tuple[int, int]]: |
| 3453 | `_dir_turn_cost` | M | def _dir_turn_cost(from_dir: int, to_dir: int) -> float: |
| 3469 | `_mine_aligned_runs` | M | def _mine_aligned_runs(stops: Sequence["_Stop"], spec: MapSpec) -> list[tuple[str, int]]: |
| 3509 | `straighten_mines` | M | def straighten_mines(order: list[tuple[int, int]], spec: MapSpec, grid: Grid, tour: "_Tour" ) -> tuple[list[tuple[int, int]], list[tuple[str, int, bool]]]: |
| 3691 | `_ts_unit` | M | def _ts_unit(v: tuple[int, int]) -> tuple[int, int]: |
| 3697 | `_ts_map_yaw` | M | def _ts_map_yaw(d: tuple[int, int]) -> float: |
| 3707 | `_ts_turn_deg` | M | def _ts_turn_deg(a: tuple[int, int], b: tuple[int, int]) -> float: |
| 3731 | `__post_init__` | N | def __post_init__(self) -> None: |
| 3788 | `_entry_walls_of` | N | def _entry_walls_of(self, cell: tuple[int, int]): |
| 3792 | `_entry_clear_ok` | N | def _entry_clear_ok(self, px: float, py: float, yaw: float, walls) -> bool: |
| 3806 | `_entry_sweep_ok` | N | def _entry_sweep_ok(self, x0: float, y0: float, x1: float, y1: float, yaw: float, walls) -> bool: |
| 3825 | `entry_gate_segment` | N | def entry_gate_segment(self, a: tuple[int, int], b: tuple[int, int]) -> bool: |
| 3847 | `entry_gate_turn` | N | def entry_gate_turn(self, cell: tuple[int, int], d_out: tuple[int, int]) -> bool: |
| 3865 | `bridge_approach_penalty` | N | def bridge_approach_penalty(self, cell: tuple[int, int], d_out: tuple[int, int]) -> float: |
| 3889 | `entry_cell_crossed_sideways` | N | def entry_cell_crossed_sideways(self, cell: tuple[int, int], d_in: tuple[int, int], d_out: tuple[int, int]) -> bool: |
| 3916 | `move_ok` | N | def move_ok(self, a: tuple[int, int], b: tuple[int, int]) -> bool: |
| 3925 | `_move_ok_raw` | N | def _move_ok_raw(self, a: tuple[int, int], b: tuple[int, int]) -> bool: |
| 3964 | `turn_gate` | N | def turn_gate(self, cell: tuple[int, int], d_prev: tuple[int, int] \| None, d_in: tuple[int, int], d_out: tuple[int, int] ) -> tuple[bool, float, float]: |
| 4041 | `_ts_tight_reversals` | M | def _ts_tight_reversals(cells: Sequence[tuple[int, int]]) -> list[int]: |
| 4055 | `_ts_leg` | M | def _ts_leg(ts: _TSGrid, a: tuple[int, int], b: tuple[int, int], h_in: tuple[int, int], cache: dict, first_dir: tuple[int, int] \| None = None) -> dict: |
| 4111 | `_ts_cut` | M | def _ts_cut(out: dict, keep: int \| None) -> dict: |
| 4122 | `_ts_phase` | M | def _ts_phase(ts: _TSGrid, cache: dict, order: Sequence[tuple[int, int]], cur: tuple[int, int], hpair: tuple, last: tuple[int, int], deadline: float, *, keep: i |
| 4143 | `go` | N | def go(mask: int, node: tuple[int, int], hp: tuple) -> dict: |
| 4172 | `_ts_search_multi` | M | def _ts_search_multi(ts: _TSGrid, spec: MapSpec, *, cross_lower_cell: tuple[int, int], cross_upper_cell: tuple[int, int], shoot_cells: Sequence[tuple[int, int]] |
| 4244 | `_ts_search` | M | def _ts_search(ts: _TSGrid, spec: MapSpec, *, cross_lower_cell: tuple[int, int], cross_upper_cell: tuple[int, int], shoot_cell: tuple[int, int], phase_cell: tup |
| 4277 | `corner_cut_enabled` | M | def corner_cut_enabled(cfg: "Config \| None") -> bool: |
| 4287 | `_seg_point_dist` | M | def _seg_point_dist(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float: |
| 4298 | `_min_clear` | M | def _min_clear(centers: Sequence[tuple[str, tuple[float, float]]], segs: Sequence[tuple[float, float, float, float]]) -> float: |
| 4308 | `_barrier_line_clear` | M | def _barrier_line_clear(segs: Sequence[tuple[float, float, float, float]], row_hi: float, col_from: str) -> float: |
| 4327 | `corner_cut_cells` | M | def corner_cut_cells(cells: Sequence[tuple[int, int]], spec: MapSpec, *, barrier_row_hi: float, barrier_col_from: str, mine_clear_m: float = CORNER_CUT_MINE_CLE |
| 4439 | `_inc_start_cell` | M | def _inc_start_cell(grid: "Grid", from_cell: str) -> tuple[int, int]: |
| 4453 | `_inc_covered` | M | def _inc_covered(already_covered: Sequence[str] \| None) -> frozenset[str]: |
| 4469 | `_inc_shape` | M | def _inc_shape(spec: MapSpec, start_cell: tuple[int, int], covered: frozenset[str]) -> dict: |
| 4476 | `_name` | N | def _name(c: tuple[int, int]) -> str: |
| 4503 | `_ts_search_incremental` | M | def _ts_search_incremental(ts: "_TSGrid", spec: MapSpec, *, cross_lower_cell: tuple[int, int], cross_upper_cell: tuple[int, int], shoot_cells: Sequence[tuple[in |
| 4583 | `plan_route_tracker_safe` | M | def plan_route_tracker_safe(map_cfg: Any, cfg: "Config \| None" = None, *, cross_lower: str \| None = None, cross_upper: str \| None = None, phase_cell: str = TS_P |
| 4722 | `_compile` | N | def _compile(_cells: list) -> tuple[list, Any, list]: |

### 3.17 `pursuit/path.py`

**类**：`Projection`@L50、`Path`@L74

| 行 | 名称 | 类型 | 签名（源码原文，≤160 字符） |
|---|---|---|---|
| 64 | `s_vertex` | N | def s_vertex(self) -> float: |
| 69 | `dist_to_vertex` | N | def dist_to_vertex(self) -> float: |
| 80 | `__init__` | N | def __init__(self, route: Route) -> None: |
| 114 | `total_length` | N | def total_length(self) -> float: |
| 119 | `segments` | N | def segments(self) -> tuple[tuple[float, float, float, float, float, float], ...]: |
| 124 | `segment_count` | N | def segment_count(self) -> int: |
| 128 | `route` | N | def route(self) -> Route: |
| 132 | `waypoints` | N | def waypoints(self) -> tuple[Waypoint, ...]: |
| 136 | `stop_indices` | N | def stop_indices(self) -> tuple[int, ...]: |
| 143 | `_seg_index_at` | N | def _seg_index_at(self, s: float) -> int: |
| 152 | `point_at` | N | def point_at(self, s: float) -> tuple[float, float, int]: |
| 169 | `heading_at` | N | def heading_at(self, s: float) -> float: |
| 173 | `s_at_waypoint` | N | def s_at_waypoint(self, index: int) -> float: |
| 181 | `remaining` | N | def remaining(self, s: float) -> float: |
| 189 | `segment_is_reverse` | N | def segment_is_reverse(self, seg_index: int) -> bool: |
| 195 | `segment_speed_scale` | N | def segment_speed_scale(self, seg_index: int) -> float: |
| 203 | `corner_angle_deg` | N | def corner_angle_deg(self, seg_index: int) -> float: |
| 215 | `car_heading_on` | N | def car_heading_on(self, seg_index: int) -> float: |
| 228 | `corner_turn_deg` | N | def corner_turn_deg(self, seg_index: int) -> float: |
| 246 | `project` | N | def project(self, x: float, y: float, *, s_hint: float \| None = None, window_m: float = 0.60, allow_backward: bool = True, motion_heading: float \| None = None,  |
| 290 | `_search_window` | N | def _search_window() -> tuple[int, int, float]: |
| 341 | `_nearest` | N | def _nearest() -> tuple[int, float, float, float, float]: |
| 454 | `__repr__` | N | def __repr__(self) -> str: # pragma: no cover - 仅日志用 |

### 3.18 `pursuit/controller.py`

**类**：`PursuitCommand`@L41

| 行 | 名称 | 类型 | 签名（源码原文，≤160 字符） |
|---|---|---|---|
| 54 | `motion_limit_speed_limit` | M | def motion_limit_speed_limit(bb) -> float \| None: |
| 85 | `speed_profile` | M | def speed_profile(*, curvature: float, dist_to_stop: float \| None, corner_dist: float \| None, speed_scale: float, cfg: Config, motion_limit_mps: float \| None =  |
| 165 | `_dist_to_stop` | M | def _dist_to_stop(pose, path: Path, proj: Projection, cfg: Config) -> float \| None: |
| 184 | `_corner_dist` | M | def _corner_dist(path: Path, proj: Projection, cfg: Config) -> float \| None: |
| 191 | `dynamic_gear_speed` | M | def dynamic_gear_speed(cfg: Config, dist_to_cross_m: float \| None) -> float \| None: |
| 219 | `compute_command` | M | def compute_command(pose, path: Path, proj: Projection, cfg: Config, *, reverse: bool = False, last_steer_deg: float \| None = None, dt: float = 0.02, bb=None, m |
| 318 | `brake_command` | M | def brake_command(cfg: Config) -> PursuitCommand: |

### 3.19 `pursuit/lookahead.py`

**类**：`LookaheadPoint`@L39

| 行 | 名称 | 类型 | 签名（源码原文，≤160 字符） |
|---|---|---|---|
| 49 | `find_lookahead_point` | M | def find_lookahead_point(path: Path, proj: Projection, ld: float, cfg: Config) -> LookaheadPoint: |

### 3.20 `tools/sim.py`

**类**：`FieldSpec`@L138、`FieldObs`@L323、`SegmentStat`@L557、`SimConfig`@L693、`SimResult`@L750、`_Rng`@L1853

| 行 | 名称 | 类型 | 签名（源码原文，≤160 字符） |
|---|---|---|---|
| 151 | `describe` | N | def describe(self) -> str: |
| 157 | `_strip_comment` | M | def _strip_comment(line: str) -> str: |
| 169 | `_parse_inline_list` | M | def _parse_inline_list(value: str) -> tuple[str, ...]: |
| 184 | `parse_map_yaml` | M | def parse_map_yaml(text: str, *, source: str = "") -> FieldSpec: |
| 245 | `_repo_root` | M | def _repo_root() -> str: |
| 259 | `load_field_spec` | M | def load_field_spec(map_path: str \| os.PathLike \| None = None, *, cfg_source: str \| None = None) -> FieldSpec \| None: |
| 295 | `body_corners` | M | def body_corners(x: float, y: float, yaw: float, *, length_m: float = 0.28, width_m: float = 0.135) -> tuple[tuple[float, float], ...]: |
| 304 | `body_point_clearance_m` | M | def body_point_clearance_m(x: float, y: float, yaw: float, px: float, py: float, *, length_m: float = 0.28, width_m: float = 0.135) -> float: |
| 353 | `__post_init__` | N | def __post_init__(self) -> None: |
| 408 | `_first` | N | def _first(self, key: str, t: float, block: str, value: float) -> None: |
| 411 | `observe` | N | def observe(self, t: float, x: float, y: float, yaw: float, *, kind: str = "normal") -> bool: |
| 521 | `metrics` | N | def metrics(self) -> dict[str, float]: |
| 727 | `validate` | N | def validate(self) -> None: |
| 787 | `steps` | N | def steps(self) -> int: |
| 790 | `metric` | N | def metric(self, key: str, default: float = float("nan")) -> float: |
| 794 | `series` | N | def series(self, name: str) -> list: |
| 804 | `step_bicycle` | M | def step_bicycle(x: float, y: float, yaw: float, v: float, steer_deg: float, dt: float, wheelbase_m: float) -> tuple[float, float, float]: |
| 821 | `_dist_point_seg` | M | def _dist_point_seg(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float: |
| 833 | `_percentile` | M | def _percentile(values: list[float], q: float) -> float: |
| 851 | `_ramp_towards` | M | def _ramp_towards(current: float, target: float, max_delta: float) -> float: |
| 861 | `_cross_track_m` | M | def _cross_track_m(x: float, y: float, segments: tuple[tuple[float, float, float, float, float, float], ...]) -> float: |
| 872 | `_rms` | M | def _rms(values: list[float]) -> float: |
| 878 | `auto_max_steps` | M | def auto_max_steps(path: Path, cfg: Config, sim: SimConfig) -> int: |
| 892 | `run_sim` | M | def run_sim(route: Route, cfg: Config, sim: SimConfig \| None = None, field: FieldSpec \| None = None) -> SimResult: |
| 1534 | `_prev_steer` | M | def _prev_steer(history: list[float]) -> float: |
| 1538 | `_record_final` | M | def _record_final(result: SimResult, t: float, bx: float, by: float, byaw: float, v_eff: float, steer_eff: float, tx: float, ty: float, tyaw: float, s: float) - |
| 1567 | `_project_with_seg` | M | def _project_with_seg(path: Path, seg: int, x: float, y: float): |
| 1580 | `_candidates` | M | def _candidates(path: Path, x: float, y: float, i_lo: int, i_hi: int): |
| 1588 | `_pick_candidate` | M | def _pick_candidate(cands, *, x: float, y: float, hint: float \| None, motion_heading: float \| None, allow_beyond: bool): |
| 1635 | `_project_sticky` | M | def _project_sticky(path: Path, x: float, y: float, *, s_hint: float \| None, window_m: float, allow_backward: bool, motion_heading: float \| None, prev_seg: int, |
| 1688 | `_window_segments` | M | def _window_segments(path: Path, s_hint: float \| None, window_m: float, allow_backward: bool) -> tuple[int, int]: |
| 1709 | `_seg_index_of_s` | M | def _seg_index_of_s(path: Path, s: float) -> int: |
| 1719 | `_projection_at` | M | def _projection_at(path: Path, seg: int, s: float, x: float, y: float): |
| 1740 | `_clamp_forward` | M | def _clamp_forward(path: Path, proj, s_floor: float, x: float, y: float, release_dist: float): |
| 1795 | `_anchor_allowed` | M | def _anchor_allowed(path: Path, idx: int) -> bool: |
| 1815 | `_advance_anchor` | M | def _advance_anchor(path: Path, anchor_idx: int, s_belief: float) -> int: |
| 1829 | `_schedule` | M | def _schedule(queue: deque[tuple[float, float, float]], at: float, v: float, steer_deg: float) -> None: |
| 1837 | `_take` | M | def _take(queue: deque[tuple[float, float, float]], now: float, v_hold: float, steer_hold: float) -> tuple[float, float]: |
| 1858 | `__init__` | N | def __init__(self, seed: int) -> None: |
| 1862 | `_uniform` | N | def _uniform(self) -> float: |
| 1871 | `gauss` | N | def gauss(self) -> float: |
| 1973 | `_fmt_row` | M | def _fmt_row(key: str, label: str, unit: str, nd: int, m: dict) -> str: |
| 1984 | `format_metrics` | M | def format_metrics(result: SimResult, sim: SimConfig \| None = None, title: str = "") -> str: |
| 2031 | `rate_desc` | M | def rate_desc(sim: SimConfig, m: dict) -> str: |
| 2038 | `format_segment_table` | M | def format_segment_table(result: SimResult, top: int = 0, title: str = "") -> str: |
| 2062 | `write_csv` | M | def write_csv(result: SimResult, path: str) -> str: |
| 2095 | `kind_of` | M | def kind_of(result: SimResult, index: int) -> str: |
| 2104 | `write_segments_csv` | M | def write_segments_csv(result: SimResult, path: str) -> str: |
| 2125 | `_traj_color` | M | def _traj_color(metrics: dict) -> str: |
| 2134 | `write_svg` | M | def write_svg(result: SimResult, path: str, route: Route, title: str = "", field: FieldSpec \| None = None) -> str: |
| 2173 | `sx` | N | def sx(x: float) -> float: |
| 2176 | `sy` | N | def sy(y: float) -> float: |
| 2180 | `polyline` | N | def polyline(points: list[tuple[float, float]], color: str, width: float, dash: str = "", opacity: float = 1.0) -> str: |
| 2307 | `_xml_escape` | M | def _xml_escape(text: str) -> str: |

### 3.21 `tools/sim_cli.py`

**类**：（无）

| 行 | 名称 | 类型 | 签名（源码原文，≤160 字符） |
|---|---|---|---|
| 50 | `build_parser` | M | def build_parser() -> argparse.ArgumentParser: |
| 103 | `_resolve_route` | M | def _resolve_route(args: argparse.Namespace, cfg: Config) -> tuple[Route, str]: |
| 116 | `_parse_set_overrides` | M | def _parse_set_overrides(items: list[str]) -> dict[str, object]: |
| 143 | `_apply_overrides` | M | def _apply_overrides(cfg: Config, overrides: dict[str, object]) -> Config: |
| 154 | `_json_sanitize` | M | def _json_sanitize(value: object) -> object: |
| 167 | `_emit_json_error` | M | def _emit_json_error(stage: str, exc: Exception) -> None: |
| 174 | `main` | M | def main(argv: list[str] \| None = None) -> int: |

**合计**：21 个文件，**585 个 def**，**61 个 class**。

---

## §4 数据流与线程模型

### 4.1 线程模型（单进程 8~9 线程）

**线程清单**（`main.py:build_tasks`，`main.py:1690-1700`）：

| # | 任务线程 | 频率 | 职责 | 关键黑板键 / 事件 |
|---|---|---|---|---|
| 1 | `LocalizationTask` | 20 Hz（据 history） | 里程计/IMU → 融合位姿 → 投影自愈 → 漂移门禁 → 限速请求 | 写 P_POSE / P_POSE_RAW / P_ODOM_AGE / P_ROUTE_S / P_ROUTE_INDEX / P_ROUTE_S；写 P_MOTION_LIMIT |
| 2 | `SafetyTask` | 10 Hz | 心跳看门狗 + 位姿失联 + 越界 → 强制停车 | 写 P_SAFETY；发 EV_SAFETY_STOP/CLEAR |
| 3 | `TrackerTask` | 50 Hz | 纯追踪控制 → 下发 pursuitAngle 指令 | 读 P_POSE / P_ROUTE / P_STOP_POINT / P_MOTION_LIMIT；写 P_PURSUIT_GEAR；发 EV_WAYPOINT_REACHED 等 |
| 4 | `MissionTask` | 10 Hz | 任务状态机（出发/到站/停站/开火放行） | 写 P_MISSION_PHASE / P_STOP_POINT；读 P_AIM_RED；发 EV_MISSION_* |
| 5 | `AimTask` | 视觉帧率 | 云台打击：找靶/预测/开火门 | 写 P_TARGET / P_AIM；发 EV_FIRE_ALLOWED/BLOCKED / EV_HIT_CONFIRMED |
| 6 | `FixTask` | 10 Hz | 误差修正量估计（比例/漂移） | 写 P_FIX_EST |
| 7 | `DroneTask` | 心跳 | 无人机情报解析（桥洞/布局） | 写 P_DRONE / P_FIELD_LAYOUT / P_FIELD_VERIFY；发 EV_DRONE_* / EV_BRIDGE_INFO / EV_FIELD_VERIFY |
| 8 | `TelemetryTask` | 2 Hz | 遥测快照 | 写 P_TELEMETRY |
| + | `ros-executor` | 4 线程 | rclpy MultiThreadedExecutor（ROS 回调线程池） | 回调只解码+写缓存（`ros_io.py:5-7`） |
| + | `virtual-car` | bench 专用 | run_bench --drive 的虚拟小车 | 读 bridge.commands，写 bridge.set_pose |

**线程不变量**（`core/task.py:12-14`）：`tick()` 必须非阻塞、无长 I/O、不持锁等待别的任务；重活放到独立进程（RKNN/底盘/云台）。任务之间**只能**通过 Blackboard（高频状态）与 EventBus（低频事件）通信，禁止直接互相调用。

**锁与共享**：
- `Blackboard._lock` = threading.RLock（`blackboard.py:102`）；临界区只有 dict 存取，**禁止**在持锁期间做 I/O 或外部回调。
- `EventBus._lock` = threading.RLock（`events.py:103`）；publish 时只在锁内取订阅列表，**投递本身在锁外**（非阻塞）。
- `RosBridge._lock` = threading.Lock（**非重入**！`ros_io.py:196`）—— `summary()` 曾因持锁内再调 `pose_age()` 而死锁（`ros_io.py:618-620` 注释：该 bug 因 summary 几乎没被调用而潜伏）。
- `TaskThread._health_lock` = threading.Lock（`task.py:85`）。
- `TaskHealth` 的 `age()` 用 time.time()；`Blackboard.age/get_fresh` 用 time.time()（墙钟），而 `StepLoop` 用 `clock = time.monotonic`（`main.py:613-615`）—— **两套时钟并存**，StepLoop 的 `_layout_rx_at` 是 monotonic，日志里 `layout_age_s` 也是 monotonic；而黑板 age 是 wall clock。

### 4.2 数据流：一条指令的完整生命周期

~~~text
[ROS 订阅回调线程（ros-executor 4 线程）]
  /odom/localization ─► on_odom ─► contract.odom_to_pose（格→米；+π/2）─► RosBridge._pose（加锁）
  /HpAndHitmsg      ─► on_referee_hp ─► RefereeTracker.feed ─► _hit_pending / _ammo
  /offset_center    ─► on_vision ─► contract.decode_offset_center（长度<5 ⇒ None）─► RosBridge._vision
                                                                │
[LocalizationTask 20 Hz] ─► Blackboard.set(P_POSE / P_ODOM_AGE / P_ROUTE_S / P_ROUTE_INDEX)
                           └─► Blackboard.set(P_MOTION_LIMIT)（漂移门禁 → 限速请求）
                                                                │
[TrackerTask 50 Hz] ─► Blackboard.get_fresh(P_POSE) ─► Path.project（弧长投影）
                           └─► find_lookahead_point（前视点）
                           └─► controller.compute_command（纯追踪 δ + 速度剖面 8 层）
                           └─► RosBridge.publish_command(linear_x, steer_deg)
                                                                │
[dzactuator 下位机] ─► pursuitAngle（50 Hz 保活；零 Twist = 刹车；1 s 看门狗）
                                                                │
[MissionTask 10 Hz] ─► Blackboard 读 P_STOP_POINT / P_AIM_RED ─► publish_fire_gate
[AimTask]           ─► Blackboard 写 P_TARGET / P_AIM ─► publish_fire_ready / publish_lead
                                                                │
[云台补丁] ─► 只在 fire_gate == 1 且 fire_ready == 1 时点亮激光
~~~

**开火两键模型**（`ros_io.py:504-515`）：`fire_gate` ← MissionTask（任务级：到站、停稳、进入 FIRE 阶段）；`fire_ready` ← AimTask（瞄准级：目标确认、预测落点合格、冷却已过）。云台补丁只在**两者都为 1** 时点亮激光 —— 任一任务误判都不会误开火。

### 4.3 数据流：一次步进循环（T27）

~~~text
[无人机 /dzlink/drone_to_car] ─► DroneTask（解析 field_layout）─► Blackboard.set(P_FIELD_LAYOUT)
                                                                          │
[StepLoop 10 Hz] ─► _poll_layout（version 大者胜）
                    ├─ PLANNING ─► plan_segment_from_layout ─► tasks.mission.plan_segment_route
                    │              └─ planner.load_map + plan_route / plan_route_tracker_safe
                    │              └─ blocked_extra = 未知格（只影响通行性）
                    │              └─ mission.request_segment(tokens, segment_id, target_cell, known_count, stop_kind)
                    │
                    ├─ DRIVING ─► _odom_watch（位姿看门狗）
                    │            └─ mission.segment_status() ─► 完成单（segment_id 匹配）
                    │            └─ _unknown_violation（belief 连续 3 拍落未知格 ⇒ SAFE_STOP）
                    │
                    ├─ SEGMENT_DONE ─► 到 end ⇒ FINISH；新布局 ⇒ PLANNING；否则 WAIT_INFO
                    ├─ WAIT_INFO ─► 停车保持位姿；无超时（默认）；1 Hz 重发完成单
                    └─ _publish_car_state ─► Blackboard.set(P_CAR_STATE)
                                                                        │
[无人机 /dzlink/car_to_drone] ◄─ RosBridge.publish_drone（车状态 / ack）
~~~

**StepLoop 的 P_CAR_STATE 字段**（`main.py:1161-1181`）：phase / segment_id / known_count / target_cell / **stop_kind**（T32：'shoot'|'mine'|'path'|'end'）/ block / version / reason / ts / segments_started / segments_done / replans / wait_s / **done_seq** / **done**（完成单闩锁）。

### 4.4 关键数据流的「真源」与「副本」

| 量 | 真源 | 副本/消费者 | 是否同步风险 |
|---|---|---|---|
| 坐标换算 | `core/geometry.py` | 全仓（validate/planner/physics/parser/contract/sim/main） | **单一真源**（文件头注释明说不可退让） |
| 隔板行/列 | `geometry.py:41-43` | validate.py（`geo.BARRIER_*`）、planner.py、tools/* | **单一真源** |
| 物理尺寸 | `config/robot.yaml`（vehicle.*/field.*/planner.obstacle_margin_m） | planner._phys_params、sim.FieldObs、tools/route_check.py | ⚠️ **两份真源风险**：`config/map.yaml`（规划器/校验器用）与 `robot.yaml 的 obstacle_blocks`（path_executor/射击点规划用）目前同步，但**换图必须两处同时改** |
| 障碍表 | `map.yaml 的 map.obstacles` | validate.py（只认 map_cfg）、planner.Grid | validate.py 的注释明确「**只认 map_cfg**，不再接受第二份障碍表」（K13 教训：obstacle_blocks 曾与 map.obstacles 不一致 ⇒ 射击点判据全错） |
| 射击窗口 | `route.shoot_distance_m / _tol_m` | planner._shoot_accept、check_route_structure | planner 缺键兜底 `SHOOT_DISTANCE_*_DEFAULT`（0.90/0.12） |
| 裁判命中/弹药 | `/HpAndHitmsg`（referee.py 解析） | ros_io.on_referee_hp → take_hits/_ammo | `/referee/hit、/referee/ammo` 车端**无生产者**，只在从未收到合法帧时兜底 |
| 配置键 | `core/config.py SCHEMA` | 全仓 cfg.get(...) | 未登记键告警；旧键经 _migrate_legacy 迁移 |

### 4.5 启动序列（`main.run_live`）

~~~text
1. RosBridge("dzbrain")  ─► rclpy.init（关默认信号处理）
2. install_stop_signals ─► 只置 stop_flag（信号上下文里不 print/不拿锁）
3. _duplicate_dzbrain_instances ─► 重复 ⇒ exit 5
4. _startup_self_check ─► env/订阅清单/话题计数 ⇒ 失败 exit 5
5. _layout_mode_enabled ─► 是否布局模式
6. startup_contract(bridge, "live") ─► 必需端点对账 ⇒ 失败 exit 6
7. [布局模式] _start_layout_listener ─► 独立 DroneTask；bridge.spin_async(threads=4)
8. _wait_for_field_layout ─► 30 s；超时/None ⇒ exit 3
9. _step_mode_enabled(first_layout)
   ├─ v2（带 known）: cfg.override(mission.auto_start=False)；startup_contract("race-step") ⇒ 失败 exit 6
   └─ v1: plan_route_from_field_layout ⇒ 失败 exit 4；成功 ⇒ cfg.override(route.tokens) + auto_plan_enabled=False
10. build_tasks(skip={"drone"})
11. [step_mode] 取 MissionTask ⇒ StepLoop
12. start_banner；bridge.spin_async（若未起）
13. _wait_for_pose_ready（8 s）⇒ 失败 exit 5
14. group.start_all()
15. [step_mode] step_loop.start + _run_step_loop ⇒ rc（1 context_dead/safe_stop、5 ros_lost、4 timeout、0 finish/stop）
    [否则] _live_loop ⇒ context_dead ⇒ 1
16. finally: group.stop_all → layout_task.stop → bridge.shutdown(brake=True) → finish_report
~~~

**退出码总表**：

| 码 | 含义 | 触发位置 |
|---|---|---|
| 0 | 正常（finish/stop） | `main.py:2153-2155` / 2161` |
| 1 | 任务故障 / context_dead / safe_stop | `main.py:1925`（bench）、2153`、2162` |
| 2 | 启动门禁未通过（--force 除外） | `main.py:2245-2246` |
| 3 | 无布局 / 装配失败 / 布局监听起不来 | `main.py:2057-2067`、2074-2076`、2125` |
| 4 | 布局规划失败 / 步进循环 timeout | `main.py:2104`、2155` |
| 5 | ROS 环境 / 位姿管线未就绪（域号冲突、重复实例、odom 无首帧、运行期断流） | `main.py:2028/2031/2141`、2155` |
| 6 | 启动契约不通过（必需 sub/pub 端点缺失） | `main.py:2043`、2096` |


---

## §5 配置键汇总（SCHEMA 实数 205 键，逐一摘录）

> 由脚本从 `core/config.py` 的 `SCHEMA` 逐条解析（键 / 缺省值 / 类型 / 单位 / 下限 / 上限 / critical）。
> ⚠️ 缺省值是**代码兜底值**（键缺失时规划器的真实行为），不是现场调过的值（`config.py:231-233` 的 ⚠️）。
> `INTENTIONALLY_UNREGISTERED` = {`localization.yaw_trim_deg`}（故意不登记，护栏在 `tools/calib_backfill.py`）。

### 5.1 `pursuit.*`（55 键）

| 键 | 缺省 | 类型 | 单位 | 范围 | critical |
|---|---|---|---|---|---|
| `pursuit.wheelbase_m` | 0.157 | _F | "m" | 0.05 ~ 0.60 | 否 |
| `pursuit.max_steer_deg` | 30.0 | _F | "°" | 5.0 ~ 45.0 | 否 |
| `pursuit.max_steer_rate_deg_s` | 120.0 | _F | "°/s" | 10.0 ~ 720.0 | 否 |
| `pursuit.curvature_gain` | 1.0 | _F | "" | 0.2 ~ 2.0 | 否 |
| `pursuit.lookahead_min_m` | 0.18 | _F | "m" | 0.05 ~ 1.0 | 否 |
| `pursuit.lookahead_max_m` | 0.45 | _F | "m" | 0.10 ~ 2.0 | 否 |
| `pursuit.lookahead_k_v` | 1.2 | _F | "s" | 0.0 ~ 5.0 | 否 |
| `pursuit.lat_acc_max_mps2` | 0.35 | _F | "m/s²" | 0.05 ~ 3.0 | 否 |
| `pursuit.min_speed_mps` | 0.08 | _F | "m/s" | 0.01 ~ 0.30 | 否 |
| `pursuit.cruise_speed_mps` | 0.15 | _F | "m/s" | 0.02 ~ 0.60 | 否 |
| `pursuit.accel_up_mps2` | 0.0 | _F | "m/s²" | 0.0 ~ 3.0 | 否 |
| `pursuit.accel_down_mps2` | 0.0 | _F | "m/s²" | 0.0 ~ 3.0 | 否 |
| `pursuit.pivot_enabled` | True | _B | "" | — | 否 |
| `pursuit.pivot_angle_deg` | 135.0 | _F | "°" | 60.0 ~ 180.0 | 否 |
| `pursuit.pivot_dist_m` | 0.25 | _F | "m" | 0.05 ~ 1.0 | 否 |
| `pursuit.pivot_speed_lead_s` | 1.0 | _F | "s" | 0.3 ~ 3.0 | 否 |
| `pursuit.pivot_speed_ref_mps` | 0.15 | _F | "m/s" | 0.05 ~ 0.40 | 否 |
| `pursuit.pivot_dist_max_m` | 0.32 | _F | "m" | 0.25 ~ 0.60 | 否 |
| `pursuit.pivot_wait_s` | 0.5 | _F | "s" | 0.0 ~ 5.0 | 否 |
| `pursuit.pivot_speed_mps` | 0.08 | _F | "m/s" | 0.02 ~ 0.30 | 否 |
| `pursuit.pivot_yaw_tol_deg` | 15.0 | _F | "°" | 3.0 ~ 45.0 | 否 |
| `pursuit.pivot_timeout_s` | 14.0 | _F | "s" | 1.0 ~ 30.0 | 否 |
| `pursuit.pivot_kp` | 1.0 | _F | "°/°" | 0.2 ~ 5.0 | 否 |
| `pursuit.motion_heading_eps_m` | 0.001 | _F | "m" | 0.0002 ~ 0.05 | 否 |
| `pursuit.projection_rewind_m` | 0.15 | _F | "m" | 0.0 ~ 0.60 | 否 |
| `pursuit.speed_quant_comp` | 1.0 | _F | "×" | 0.8 ~ 1.3 | 否 |
| `pursuit.reverse_speed_ratio` | 0.6 | _F | "×" | 0.1 ~ 1.0 | 否 |
| `pursuit.reverse_entry_m` | 0.06 | _F | "m" | 0.0 ~ 0.30 | 否 |
| `pursuit.vertex_floor_m` | 0.05 | _F | "m" | 0.0 ~ 0.30 | 否 |
| `pursuit.projection_release_m` | 0.30 | _F | "m" | 0.10 ~ 1.00 | 否 |
| `pursuit.reverse_steer_sign` | 1.0 | _F | "" | -1.0 ~ 1.0 | 否 |
| `pursuit.arrival_m` | 0.12 | _F | "m" | 0.03 ~ 0.35 | 否 |
| `pursuit.approach_m` | 0.60 | _F | "m" | 0.10 ~ 2.0 | 否 |
| `pursuit.stop_radius_m` | 0.17 | _F | "m" | 0.05 ~ 0.35 | 否 |
| `pursuit.overshoot_tol_m` | 0.30 | _F | "m" | 0.05 ~ 0.60 | 否 |
| `pursuit.vertex_clamp_m` | 0.04 | _F | "m" | 0.0 ~ 0.20 | 否 |
| `pursuit.lookahead_floor_m` | 0.10 | _F | "m" | 0.02 ~ 0.50 | 否 |
| `pursuit.corner_angle_deg` | 45.0 | _F | "°" | 10.0 ~ 120.0 | 否 |
| `pursuit.corner_slow_dist_m` | 0.45 | _F | "m" | 0.05 ~ 1.5 | 否 |
| `pursuit.corner_speed_mps` | 0.12 | _F | "m/s" | 0.02 ~ 0.40 | 否 |
| `pursuit.lookahead_corner_angle_deg` | 20.0 | _F | "°" | 5.0 ~ 90.0 | 否 |
| `pursuit.stop_approach_speed_gain` | 0.30 | _F | "1/s" | 0.05 ~ 2.0 | 否 |
| `pursuit.map_safety_margin_m` | 0.60 | _F | "m" | 0.0 ~ 1.5 | 否 |
| `pursuit.drift_warn_m` | 0.30 | _F | "m" | 0.05 ~ 1.0 | 否 |
| `pursuit.drift_seg_gate_m` | 0.18 | _F | "m" | 0.02 ~ 1.0 | 否 |
| `pursuit.bridge_blocks` | None | list | "" | — | 否 |
| `pursuit.align_straight_before` | 2.0 | _F | "格" | 0.5 ~ 5.0 | 否 |
| `pursuit.align_straight_after` | 1.0 | _F | "格" | 0.5 ~ 5.0 | 否 |
| `pursuit.align_straight_speed_mps` | 0.10 | _F | "m/s" | 0.02 ~ 0.30 | 否 |
| `pursuit.align_kp` | 12.0 | _F | "" | 1.0 ~ 40.0 | 否 |
| `pursuit.align_steer_limit_deg` | 8.0 | _F | "°" | 1.0 ~ 20.0 | 否 |
| `pursuit.dynamic_gear_enabled` | False | _B | "" | — | 否 |
| `pursuit.dynamic_gear_near_dist_m` | 1.05 | _F | "m" | 0.0 ~ 20.0 | 否 |
| `pursuit.dynamic_gear_near_mps` | 0.35 | _F | "m/s" | 0.02 ~ 0.60 | 否 |
| `pursuit.dynamic_gear_far_mps` | 0.40 | _F | "m/s" | 0.02 ~ 0.60 | 否 |

### 5.2 `vehicle.*`（4 键）

| 键 | 缺省 | 类型 | 单位 | 范围 | critical |
|---|---|---|---|---|---|
| `vehicle.length_m` | 0.28 | _F | "m" | 0.10 ~ 1.00 | 否 |
| `vehicle.width_m` | 0.135 | _F | "m" | 0.05 ~ 0.60 | 否 |
| `vehicle.coverage_radius_m` | 0.035 | _F | "m" | 0.005 ~ 0.20 | 否 |
| `vehicle.ref_offset_m` | 0.075 | _F | "m" | -0.30 ~ 0.30 | 否 |

### 5.3 `field.*`（5 键）

| 键 | 缺省 | 类型 | 单位 | 范围 | critical |
|---|---|---|---|---|---|
| `field.obstacle_radius_m` | 0.1275 | _F | "m" | 0.01 ~ 0.50 | 否 |
| `field.mine_pad_m` | 0.12 | _F | "m" | 0.02 ~ 0.35 | 否 |
| `field.barrier_width_m` | 0.02 | _F | "m" | 0.002 ~ 0.20 | 否 |
| `field.barrier_cols` | "D..N" | _S | "" | — | 否 |
| `field.bridge_entry_redundancy_m` | 0.0875 | _F | "m" | 0.0 ~ 0.35 | 否 |

### 5.4 `planner.*`（11 键）

| 键 | 缺省 | 类型 | 单位 | 范围 | critical |
|---|---|---|---|---|---|
| `planner.obstacle_margin_m` | 0.02 | _F | "m" | 0.0 ~ 0.30 | 否 |
| `planner.engine` | "tour" | _S | "" | — | 否 |
| `planner.footprint_gate` | "warn" | _S | "" | — | 否 |
| `planner.tracker_max_seconds` | 10.0 | _F | "s" | 1.0 ~ 120.0 | 否 |
| `planner.auto_plan_enabled` | False | _B | "" | — | 否 |
| `planner.map_path` | "" | _S | "" | — | 否 |
| `planner.fallback_to_tokens` | True | _B | "" | — | 否 |
| `planner.allow_reverse` | False | _B | "" | — | 否 |
| `planner.max_seconds` | 2.0 | _F | "s" | 0.1 ~ 60.0 | 否 |
| `planner.pivot_penalty` | 0.0 | _F | "" | 0.0 ~ 100.0 | 否 |
| `planner.mine_turn_penalty` | 2.0 | _F | "" | 0.0 ~ 100.0 | 否 |

### 5.5 `localization.*`（25 键）

| 键 | 缺省 | 类型 | 单位 | 范围 | critical |
|---|---|---|---|---|---|
| `localization.anchor_max_pull_m` | 0.12 | _F | "m" | 0.0 ~ 0.50 | 否 |
| `localization.anchor_reject_eps_m` | 0.005 | _F | "m" | 0.0 ~ 0.10 | 否 |
| `localization.anchor_scale_max` | 0.08 | _F | "" | 0.0 ~ 0.50 | 否 |
| `localization.implied_scale_window` | 4 | _I | "拍" | 1 ~ 100 | 否 |
| `localization.reanchor_pull_max_m` | 0.35 | _F | "m" | 0.0 ~ 1.00 | 否 |
| `localization.drift_slow_after` | 3 | _I | "段" | 1 ~ 10000 | 否 |
| `localization.drift_slow_speed_mps` | 0.08 | _F | "m/s" | 0.0 ~ 0.40 | 否 |
| `localization.drift_stop_after` | 8 | _I | "段" | 1 ~ 10000 | 否 |
| `localization.drift_clear_ticks` | 5 | _I | "拍" | 1 ~ 1000 | 否 |
| `localization.boundary_guard_m` | 0.10 | _F | "m" | 0.0 ~ 1.00 | 否 |
| `localization.boundary_guard_speed_mps` | 0.08 | _F | "m/s" | 0.0 ~ 0.40 | 否 |
| `localization.boundary_stop_m` | 0.03 | _F | "m" | 0.0 ~ 1.00 | 否 |
| `localization.stuck_ticks` | 5 | _I | "拍" | 2 ~ 50 | 否 |
| `localization.stuck_eps_m` | 0.01 | _F | "m" | 0.001 ~ 0.10 | 否 |
| `localization.stuck_dist_m` | 0.20 | _F | "m" | 0.05 ~ 1.0 | 否 |
| `localization.yaw_guard` | True | _B | "" | — | 否 |
| `localization.yaw_kin_margin` | 1.25 | _F | "" | 1.0 ~ 3.0 | 否 |
| `localization.yaw_rate_floor_dps` | 8.0 | _F | "°/s" | 0.0 ~ 60.0 | 否 |
| `localization.yaw_abs_floor_dps` | 20.0 | _F | "°/s" | 0.0 ~ 90.0 | 否 |
| `localization.yaw_static_dps` | 45.0 | _F | "°/s" | 5.0 ~ 180.0 | 否 |
| `localization.yaw_min_speed_mps` | 0.03 | _F | "m/s" | 0.0 ~ 0.10 | 否 |
| `localization.yaw_bad_ticks` | 3 | _I | "拍" | 1 ~ 20 | 否 |
| `localization.yaw_plausible_speed_mps` | 0.70 | _F | "m/s" | 0.20 ~ 2.0 | 否 |
| `localization.yaw_max_dt_s` | 0.60 | _F | "s" | 0.10 ~ 2.0 | 否 |
| `localization.yaw_imu_mismatch_ratio` | 0.35 | _F | "" | 0.0 ~ 1.0 | 否 |

### 5.6 `route.*`（7 键）

| 键 | 缺省 | 类型 | 单位 | 范围 | critical |
|---|---|---|---|---|---|
| `route.tokens` | None | list | "" | — | 否 |
| `route.cross_lower_candidates` | [f"K{r}" for r in range(7, 12)] | list | "" | — | 否 |
| `route.cross_upper_candidates` | [f"K{r}" for r in range(1, 7)] | list | "" | — | 否 |
| `route.cross_lower_default` | "K10" | _S | "" | — | 否 |
| `route.cross_upper_default` | "K5" | _S | "" | — | 否 |
| `route.shoot_distance_m` | 0.80 | _F | "m" | 0.0 ~ 5.0 | 否 |
| `route.shoot_distance_tol_m` | 0.10 | _F | "m" | 0.0 ~ 2.0 | 否 |

### 5.7 `mission.*`（23 键）

| 键 | 缺省 | 类型 | 单位 | 范围 | critical |
|---|---|---|---|---|---|
| `mission.auto_start` | True | _B | "" | — | 否 |
| `mission.auto_start_delay_s` | 10.0 | _F | "s" | 0.0 ~ 120.0 | 否 |
| `mission.reset` | "N10,y" | _S | "" | — | 否 |
| `mission.scan_mode` | "continuous" | _S | "" | — | 否 |
| `mission.bridge_info_wait_s` | 20.0 | _F | "s" | 0.0 ~ 120.0 | 否 |
| `mission.bridge_wait_at_start` | False | _B | "" | — | 否 |
| `mission.bridge_wait_max_s` | 12.0 | _F | "s" | 0.0 ~ 120.0 | 否 |
| `mission.race_budget_s` | 180.0 | _F | "s" | 10.0 ~ 600.0 | 否 |
| `mission.bridge_reroute_max` | 1 | _I | "次" | 0 ~ 5 | 否 |
| `mission.shots_per_stop` | 3 | _I | "发" | 1 ~ 10 | 否 |
| `mission.shoot_timeout_s` | 30.0 | _F | "s" | 1.0 ~ 120.0 | 否 |
| `mission.lock_timeout_s` | 18.0 | _F | "s" | 1.0 ~ 120.0 | 否 |
| `mission.settle_time_s` | 0.8 | _F | "s" | 0.0 ~ 10.0 | 否 |
| `mission.settle_eps_m` | 0.02 | _F | "m" | 0.001 ~ 0.20 | 否 |
| `mission.settle_speed_mps` | 0.02 | _F | "m/s" | 0.001 ~ 0.20 | 否 |
| `mission.scan_stop_timeout_s` | 8.0 | _F | "s" | 0.5 ~ 60.0 | 否 |
| `mission.stable_timeout_s` | 6.0 | _F | "s" | 0.5 ~ 30.0 | 否 |
| `mission.stable_offset_px` | 40.0 | _F | "px" | 5.0 ~ 200.0 | 否 |
| `mission.candidate_retry` | 3 | _I | "个" | 1 ~ 10 | 否 |
| `mission.finish_voice_id` | 0 | _I | "" | 0 ~ 255 | 否 |
| `mission.stall_timeout_s` | 12.0 | _F | "s" | 2.0 ~ 120.0 | 否 |
| `mission.stall_yaw_deg` | 360.0 | _F | "°" | 90.0 ~ 1440.0 | 否 |
| `mission.skip_second_stop_after_hit` | True | _B | "" | — | 否 |

### 5.8 `aim.*`（40 键）

| 键 | 缺省 | 类型 | 单位 | 范围 | critical |
|---|---|---|---|---|---|
| `aim.scan_request_on_start` | 2 | _I | "" | 0 ~ 2 | 否 |
| `aim.lead_time_s` | 0.0 | _F | "s" | 0.0 ~ 2.0 | 否 |
| `aim.fire_policy` | "moving_lead" | _S | "" | — | 否 |
| `aim.rail_span_m` | 0.70 | _F | "m" | 0.05 ~ 3.0 | 否 |
| `aim.target_speed_mps` | 0.04 | _F | "m/s" | 0.001 ~ 1.0 | 否 |
| `aim.max_shots_total` | 10 | _I | "发" | 1 ~ 10 | 否 |
| `aim.max_shots_per_target` | 3 | _I | "发" | 1 ~ 10 | 否 |
| `aim.shot_cooldown_ms` | 500 | _I | "ms" | 100 ~ 5000 | 否 |
| `aim.fire_box_far_px` | 14.0 | _F | "px" | 2.0 ~ 100.0 | 否 |
| `aim.fire_box_near_px` | 26.0 | _F | "px" | 2.0 ~ 200.0 | 否 |
| `aim.lost_frames_to_rescan` | 180 | _I | "帧" | 10 ~ 600 | 否 |
| `aim.scan_pitch_offset_deg` | 6.0 | _F | "°" | -60.0 ~ 30.0 | 否 |
| `aim.box_w_far_px` | 30.0 | _F | "px" | 5.0 ~ 200.0 | 否 |
| `aim.box_w_near_px` | 120.0 | _F | "px" | 10.0 ~ 400.0 | 否 |
| `aim.cls_switch_confirm_frames` | 5 | _I | "帧" | 1 ~ 60 | 否 |
| `aim.red_early_leave_s` | 3.0 | _F | "s" | 1.0 ~ 10.0 | 否 |
| `aim.est_dt_max_s` | 0.50 | _F | "s" | 0.01 ~ 2.0 | 否 |
| `aim.est_dt_min_s` | 0.002 | _F | "s" | 0.0 ~ 0.1 | 否 |
| `aim.est_flip_min_speed_px_s` | 2.0 | _F | "px/s" | 0.5 ~ 20.0 | 否 |
| `aim.est_gate_base_px` | 3.0 | _F | "px" | 1.0 ~ 20.0 | 否 |
| `aim.est_gate_k` | 3.0 | _F | "s" | 0.0 ~ 20.0 | 否 |
| `aim.est_gate_vel_scale` | 0.10 | _F | "×" | 0.0 ~ 2.0 | 否 |
| `aim.est_jump_floor_px` | 25.0 | _F | "px" | 5.0 ~ 100.0 | 否 |
| `aim.est_max_speed_px_s` | 300.0 | _F | "px/s" | 50.0 ~ 3000.0 | 否 |
| `aim.est_vel_window_s` | 0.40 | _F | "s" | 0.05 ~ 2.0 | 否 |
| `aim.est_xi` | 3.5 | _F | "" | 0.5 ~ 10.0 | 否 |
| `aim.fire_only_blue` | True | _B | "" | — | 否 |
| `aim.hit_hold_s` | 0.5 | _F | "s" | 0.0 ~ 5.0 | 否 |
| `aim.hold_min_s` | 0.30 | _F | "s" | 0.0 ~ 2.0 | 否 |
| `aim.lead_eps_px` | 0.5 | _F | "px" | 0.0 ~ 20.0 | 否 |
| `aim.max_lead_factor` | 0.45 | _F | "×" | 0.0 ~ 2.0 | 否 |
| `aim.px_per_m_rail` | 140.0 | _F | "px/m" | 10.0 ~ 1000.0 | 否 |
| `aim.rearm_after_lost_s` | 1.0 | _F | "s" | 0.0 ~ 10.0 | 否 |
| `aim.rescan_on_lost` | False | _B | "" | — | 否 |
| `aim.reset_confirm_frames` | 5 | _I | "帧" | 1 ~ 60 | 否 |
| `aim.vision_nominal_fps` | 30.0 | _F | "fps" | 1.0 ~ 120.0 | 否 |
| `aim.gimbal_lock_enabled` | True | _B | "" | — | 否 |
| `aim.gimbal_lock_scan_request` | 0 | _I | "" | 0 ~ 2 | 否 |
| `aim.gimbal_seek_scan_request` | 2 | _I | "" | 0 ~ 2 | 否 |
| `aim.gimbal_seek_on_shoot_stop_only` | True | _B | "" | — | 否 |

### 5.9 `fix.*`（16 键）

| 键 | 缺省 | 类型 | 单位 | 范围 | critical |
|---|---|---|---|---|---|
| `fix.anchor_at_waypoint` | True | _B | "" | — | 否 |
| `fix.anchor_max_offset_m` | 0.08 | _F | "m" | 0.01 ~ 0.30 | 否 |
| `fix.anchor_gain` | 1.0 | _F | "×" | 0.0 ~ 1.0 | 否 |
| `fix.scale_est_enabled` | True | _B | "" | — | 否 |
| `fix.scale_step_max` | 0.005 | _F | "×/段" | 0.0 ~ 0.05 | 否 |
| `fix.scale_limit` | 0.10 | _F | "×" | 0.0 ~ 0.5 | 否 |
| `fix.drift_cull_m` | 0.30 | _F | "m" | 0.0 ~ 1.0 | 否 |
| `fix.seg_min_m` | 0.05 | _F | "m" | 0.0 ~ 0.5 | 否 |
| `fix.scale_min_samples` | 3 | _I | "段" | 1 ~ 20 | 否 |
| `fix.ratio_bounds` | (0.75, 1.35) | tuple | "×" | — | 否 |
| `fix.ratio_jump_max` | 0.15 | _F | "×" | 0.0 ~ 1.0 | 否 |
| `fix.ratio_jump_min_prior` | 3 | _I | "段" | 1 ~ 20 | 否 |
| `fix.scale_window` | 5 | _I | "段" | 1 ~ 50 | 否 |
| `fix.ds_max_ratio` | 2.0 | _F | "×" | 1.0 ~ 10.0 | 否 |
| `fix.ds_floor_m` | 0.05 | _F | "m" | 0.0 ~ 0.30 | 否 |
| `fix.ds_reject_max_run` | 3 | _I | "拍" | 1 ~ 50 | 否 |

### 5.10 `drone.*`（10 键）

| 键 | 缺省 | 类型 | 单位 | 范围 | critical |
|---|---|---|---|---|---|
| `drone.enabled` | True | _B | "" | — | 否 |
| `drone.topic_in` | "/dzlink/drone_to_car" | _S | "" | — | 否 |
| `drone.topic_out` | "/dzlink/car_to_drone" | _S | "" | — | 否 |
| `drone.heartbeat_timeout_s` | 3.0 | _F | "s" | 0.5 ~ 30.0 | 否 |
| `drone.info_stale_s` | 30.0 | _F | "s" | 1.0 ~ 300.0 | 否 |
| `drone.car_state_period_s` | 0.7 | _F | "s" | 0.1 ~ 10.0 | 否 |
| `drone.ack_enabled` | True | _B | "" | — | 否 |
| `drone.field_diff_max` | 8 | _I | "条" | 0 ~ 100 | 否 |
| `drone.field_layout_wait_s` | 30.0 | _F | "s" | 1.0 ~ 120.0 | 否 |
| `drone.map_path` | "" | _S | "" | — | 否 |

### 5.11 `step.*`（3 键）

| 键 | 缺省 | 类型 | 单位 | 范围 | critical |
|---|---|---|---|---|---|
| `step.wait_timeout_s` | 0.0 | _F | "s" | 0.0 ~ 3600.0 | 否 |
| `step.pose_ready_wait_s` | 8.0 | _F | "s" | 0.0 ~ 120.0 | 否 |
| `step.odom_watch_s` | 6.0 | _F | "s" | 0.0 ~ 120.0 | 否 |

### 5.12 `safety.*`（4 键）

| 键 | 缺省 | 类型 | 单位 | 范围 | critical |
|---|---|---|---|---|---|
| `safety.pose_timeout_s` | 1.0 | _F | "s" | 0.1 ~ 10.0 | 否 |
| `safety.startup_grace_s` | 3.0 | _F | "s" | 0.0 ~ 30.0 | 否 |
| `safety.task_stall_s` | 1.0 | _F | "s" | 0.1 ~ 10.0 | 否 |
| `safety.brake_on_fault` | True | _B | "" | — | 否 |

### 5.13 `logging.*`（1 键）

| 键 | 缺省 | 类型 | 单位 | 范围 | critical |
|---|---|---|---|---|---|
| `logging.rate_limit_s` | 1.0 | _F | "s" | 0.0 ~ 60.0 | 否 |

### 5.14 `telemetry.*`（1 键）

| 键 | 缺省 | 类型 | 单位 | 范围 | critical |
|---|---|---|---|---|---|
| `telemetry.period_s` | 0.5 | _F | "s" | 0.1 ~ 10.0 | 否 |

### 5.末 旧键迁移映射（`config.py:665-724`，46 条）

完整映射表见 §2.3.7。要点：

- 方向 = **新键 ← 旧键**；迁移**不删旧键**（双向兼容），迁移后 `issues` 多一条 info。
- `/dzbrain` 段映射为**空前缀**（避免顶层裸键让 `--params-file` 解析失败、相机/视觉/云台三节点同时 abort）。
- `vehicle.*` / `field.*` 从 `dzactuator.*` 段迁移（2026-10-16 真 bug 修复：缺这组映射时 planner/sim/route_check 恒吃 SCHEMA 默认 ⇒ 现场改尺寸被静默忽略）。

### 5.附 未登记但被读取的键（⚠️ 本轮发现，见 §7.2）

| 键 | 读取位置 | 后果 |
|---|---|---|
| `planner.barrier_turn_penalty` | `planner.py:2917` | 拼错不报错、无范围校验 |
| `planner.barrier_turn_margin_m` | `planner.py:2921` | 同上 |
| `planner.door_turn_penalty` | `planner.py:2926` | 同上 |
| `planner.edge_mine_turn_penalty` | `planner.py:2930` | 同上 |
| `planner.obstacle_turn_penalty` | `planner.py:2936` | 同上 |
| `planner.bridge_approach_align_penalty_m` | `planner.py:659-660` | 同上 |
| `step.layout_ttl_s` | `main.py:637` | 同上 |
| `step.done_replay_s` | `main.py:651` | 同上 |

---

## §6 测试与证据

### 6.1 测试清单与本轮实测结果

测试目录：`dzrobot_v2/src/dzbrain/tests`（27 个测试文件 + `harness.py` + `fakes.py`）。运行方式：`python <test>.py`（**本机无 pytest**，项目惯例是纯脚本测试 + `harness.py` 的 `check/check_close/check_raises/finish`；`harness.py:1-9`）。
运行环境：Windows 11 / Python 3.12.10（`python --version` 实测）；`pytest` **未安装**（No module named pytest）。

| 测试文件 | 覆盖 | 本轮实测 |
|---|---|---|
| `test_geometry.py` | 坐标锚点（N11/A1/N10 格心、格单位、角度、四元数） | **38/38 通过** |
| `test_physics.py` | 物理几何（压雷/障碍/挡板/靶板/扫掠半径/冗余墙） | **103/103 通过** |
| `test_core.py` | Config / Blackboard / EventBus / TaskThread | **56/56 通过** |
| `test_contract.py` | ROS 契约（话题/QoS/单位换算/解码） | **56/56 通过** |
| `test_referee.py` | 裁判帧解析（命中/挨打/弹药/复位） | **81/81 通过** |
| `test_pursuit.py` | 纯追踪（投影/前视/舵角/速度剖面/倒车） | **122/122 通过** |
| `test_pivot.py` | PIVOT 原地转向（物理转弯时间现算卡住） | **55/55 通过** |
| `test_autoplan.py` | 自动规划（--plan-only 门禁豁免/横幅台账） | **171/171 通过** |
| `test_route.py` | 路线校验 R001~R014（_is_cross 对称性等） | 156/157（1 失败） |
| `test_motion_limit.py` | P_MOTION_LIMIT 限速请求 + 12 键登记 | 15/17（2 失败） |
| `test_planner.py` | 规划器（结构自检/折返点/route_check 对拍/起始航向） | 10 失败 |
| `test_sim.py` | 仿真器（误差注入/延迟/契约指标/场地物理） | 多处失败 |
| `test_sim_guard.py` | 仿真护栏（现场图结构/场地物理洁净度/锚定） | 多处失败 |
| `test_incremental_plan.py` | tracker-safe 增量续算 | **直接抛异常退出** |
| `test_bridge_gate.py` | 桥洞门禁（车心 vs 车身口径） | 44/47（3 失败） |
| `test_corner_cut.py` | 拐角切角判据 | exit 1 |
| `其余 20 个测试文件` | test_deploy / test_mission / test_aim / test_tracker / test_drone / test_localization / test_drift / test_fix / test_safety / test_gimbal_lock / test_telemetry / test_aim_probe / test_car_state_report / test_stepper / test_goal_stop / test_path_overlap / test_link_e2e / test_scout / test_calib_backfill / test_dynamic_gear | 本轮未逐个运行（见 §7.1） |

### 6.2 本轮红项明细（全部为「断言期望值已过期 / 与当前配置不一致」，非源码逻辑崩溃）

**`test_route.py`（1 失败）**：`test_validate_field_route` / `test_validate_negatives` 失败；报文内容为 `route.tokens 里的 44 个 token 里官方图 tracker-safe 均无规划结果；t3-A 缺 0.90 m 停站 [68 格]` —— 期望 44 点，实际 68 点。

**`test_motion_limit.py`（2 失败）**：
- `5.2 逐条等于代码默认值`：期望 `localization.drift_slow_after=3、drift_stop_after=8、boundary_guard_m=0.1、boundary_stop_m=0.03`，实际 `config/robot.yaml` 里被现场改成 999（解除降速/停车门禁）、boundary_guard_m=0.25、boundary_stop_m=0.1（`config.py:264-265` 的注释正是为此把上限放宽到 10000）。
- `5.3 落盘值`：期望 cruise 0.30 / ld_max 0.35 / 44 点 / ticksPerMeter 3280，实际 cruise 0.35 / ld_max 0.4 / 68 点 / ticksPerMeter 3280（期望值未随 `config/robot.yaml` 更新）。

**`test_planner.py`（10 失败）**：
- `雷落在障碍格上（地图自相矛盾）-> 抛错`：没有抛异常，返回 PlanResult（`planner.py:3033-3038` 在 legacy 口径下会抛，但该用例走的是 tasks 口径 ⇒ 走降级）。
- `无倒车 40 token / 里程 15.05 m（0 段 R:，0 处 :PIVOT）`：实际 68 token / 24.85 m。
- `折返点 现场无倒车路线实测 0 处 180° 折返`：实际 3 处 [(37,I4,180),(39,G4,180),(52,A1,180)]。
- `折返点 token 里没有任何 :PIVOT`：实际 3 处 [I4:PIVOT, G4:PIVOT, A1:PIVOT]。
- `折返点 PlanResult 清单与是否无倒车模式一致`：count=3 allow_reverse=False penalty=5.0。
- `惩罚 折返点已是唯一通路的最小值，惩罚前后都是 0 处`：惩罚前 3 → 惩罚后 3（pivot_penalty 在当前图上没能消除任何折返）。
- `惩罚 单元 A1->A2->B1 判为 1 处折返`：reversals=0。
- `惩罚 单元 同一序列的 seq_cost 差 = 5.0`：12.7000 - 12.7000 = 0（惩罚没生效）。

**`test_sim.py`（多处失败）**：
- `6.7 ε=+0.18 时锚定无实质作用`：保持记录而非洗绿（同窗口 2500 拍有锚定 0.6374 vs 无锚定 0.6247 m；吸附只 2 次，因为 |信念-真值| 常超过 anchor_radius_m=0.12 m 这个观测可用窗口）。
- `7.4 0.2 s 延迟在新路线下跑完且不发散`：实际 t=80.9 s len=0.71 completed=False diverged=True。
- `7.5 延迟 0.8 s 时新路线仍跑完`：实际 completed=False len=0.93 横向MAX=0.55 m。
- `8.13 现场路线几何 44 航点 / 15.050 m / 0 倒车段 / 0 个 ≥135° PIVOT 折返`：实际 68 航点 / 24.850 m / PIVOT 折返 [A1, G4, I4]。
- `10.7b 在通格 K9 横向贯通 ⇒ 封死/非通格违规都是 0`：实际 K 列 168 拍，违规 0（该项实际通过但报文期望值不同）。
- `10.8b 远程路线不碰靶板 ⇒ 重叠 = 0 且净距 > 0`：**现役路线最近 0.0000 m**（真问题：现役路线会压到靶板）。

**`test_sim_guard.py`（多处失败）**：
- `2.5b 第二射击停站 = G3`：实际 block=C4 shoot=False。
- `2.8 / 2.14 真值逐拍都在地图内`：实际越界 74 拍。
- `3.6 锚定压低信念误差`：实际锚定=0.73462 vs 不锚定=0.70789（压低 -3.8%，锚定 2 次）—— 锚定方向反了。
- `10.2 现役路线零注入：物理硬违规全为 0`：实际挡板触线拍=113、车体压靶板拍=78、越界拍数=74。
- `10.3 现役路线零注入：8/8 盖雷且硬违规仍全 0`：实际盖雷 8/8，但最远雷心距 0.0016 m 且明确写「接近腿改造需用户现场裁决，不得为凑 8/8 引入碰撞/硬违规」。

**`test_bridge_gate.py`（3 失败）**：
- `A8 类车心口径侵入 = 2 拍`：实际车心 179 拍 / 车身 416 拍。
- `4 门禁放行的路线：车身口径 K 桥室侵入 = 0 拍`：实际 6 拍。
- `5 车心口径会漏：A8 类斜切进洞 车身 155 拍 / 车心 77 拍`：实际车身 416 拍 / 车心 179 拍。

**`test_incremental_plan.py`（直接异常退出）**：
~~~text
File ".../tests/test_incremental_plan.py", line 61, in plan
    return PL.plan_route_tracker_safe(spec, Config.load(str(CFG_PATH)), ...)
File ".../dzbrain/route/planner.py", line 4697, in plan_route_tracker_safe
    raise PlannerError("tracker-safe 引擎无解（可能被『禁 180°/禁紧凑 U/雷穿心』筛空）")
dzbrain.route.planner.PlannerError: tracker-safe 引擎无解（可能被『禁 180°/禁紧凑 U/雷穿心』筛空）
~~~
`plan_route_tracker_safe` 在当前 `config/robot.yaml` + `map.yaml` 组合下直接无解（`test_incremental_plan.py:61`、`planner.py:4697`）。

### 6.3 测试夹具与替身

| 文件 | 作用 |
|---|---|
| `tests/harness.py` | `check / check_close / check_raises / finish`；finish 打印汇总并以 0/1 退出；sys.stdout.reconfigure(errors="replace") 防 Windows GBK 控制台 UnicodeEncodeError 把整个脚本炸掉（`harness.py:17-24`） |
| `tests/fakes.py` | `FakeBridge`（bench 用）：记录全部指令、可设位姿 |
| `tests/_run_loop_case.py` | 跑循环用例夹具 |
| `tests/test_core.py / test_contract.py / test_referee.py` | 纯逻辑单测（无 ROS） |

### 6.4 证据链（本轮可复跑命令）

~~~bash
cd D:/User/Desktop/比赛存档/v1/dzrobot_v2/src/dzbrain/tests
python test_geometry.py         # 38/38 通过
python test_physics.py          # 103/103 通过
python test_core.py             # 56/56 通过
python test_contract.py         # 56/56 通过
python test_referee.py          # 81/81 通过
python test_pursuit.py          # 122/122 通过
python test_pivot.py            # 55/55 通过
python test_autoplan.py         # 171/171 通过
python test_route.py            # 156/157
python test_motion_limit.py     # 15/17
python test_planner.py          # 10 失败
python test_sim.py              # 多处失败
python test_sim_guard.py        # 多处失败
python test_incremental_plan.py # 异常退出
python test_bridge_gate.py      # 44/47
python test_corner_cut.py       # exit 1
~~~


---

## §7 未核实清单与发现的坑

### 7.1 未核实项（本轮未做，不作结论）

| # | 未核实项 | 原因 | 若要核实 |
|---|---|---|---|
| 1 | `tasks/*`（localization/safety/tracker/mission/aim/fix/drone/telemetry）的**实现细节** | 任务要求范围是 `main/core/io/route/pursuit/tools`，tasks 不在本次必覆盖清单 | 单独开卡 |
| 2 | `config/robot.yaml` 与 `config/map.yaml` 的**逐键一致性** | 本轮只读了 `robot.yaml` 的 3 处（settle_*、corner_cut） | 跑 `python tools/wf/check.py` + config_selfcheck |
| 3 | 除 §6.1 已跑测试外的**其余测试文件**（test_deploy / test_mission / test_aim / test_tracker / test_drone / test_localization / test_drift / test_fix / test_safety / test_gimbal_lock / test_telemetry / test_aim_probe / test_car_state_report / test_stepper / test_goal_stop / test_path_overlap / test_link_e2e / test_scout / test_calib_backfill / test_dynamic_gear） | 本轮只跑了 §6.1 列出的 16 个 | 逐个跑 |
| 4 | `plan_route_tracker_safe` 无解的**根因**（是 map.yaml 变了还是 turned_mines 缺项） | 本轮只看到抛出的异常文本 | 在 `_tmp_session/` 里二分定位 |
| 5 | `tools/sim.py` 场地判据与 `tools/route_check.py` 的**逐条对拍** | 本轮只读了 sim.py，未读 route_check.py | 读 route_check.py 并对拍 |
| 6 | 真车侧 `compute_command` 的 `last_speed_mps` / `dt` 是否**真的接上了**（加减速斜坡的调用方责任） | `controller.py:110-112` 明确说「调用方有责任播种历史」，但本轮未读 `tasks/tracker.py` | 读 `tasks/tracker.py` |
| 7 | `planner.py:614` 的 `_door_turn_margin_m` 旧定义是否**曾被调用** | 695 行同名定义覆盖后旧版不可达 | 全文 grep `_door_turn_margin_m` |
| 8 | `config.py:688-690` 旧键迁移映射的**语义正确性**（settle_eps_m ← shoot_settle_speed） | 单位不同（m vs m/s），可疑但未核实 | 读 `tasks/mission.py:3500-3520` |

### 7.2 发现的坑 / bug（带 file:line）

| # | 位置 | 现象 | 影响 |
|---|---|---|---|
| 1 | `planner.py:614` vs `planner.py:695` | **同一文件里 `_door_turn_margin_m` 被定义两次**：614 行是旧版（返回可用余量、无 turn_deg/phys 参数），695 行是现用版（带 `*, turn_deg, phys`）。Python 后者覆盖前者 | 614 行是**死代码**；若有人改了 614 行以为生效，实际不生效。建议删除 614 行（本卡只读，未改）。 |
| 2 | `planner.py:2917/2921/2926/2930/2936`；`planner.py:659-660` | `planner.barrier_turn_penalty / barrier_turn_margin_m / door_turn_penalty / edge_mine_turn_penalty / obstacle_turn_penalty / bridge_approach_align_penalty_m` **被读取却未登记进 `SCHEMA`** | `validate()` 抓不到拼写错误、不做类型/范围校验 ⇒ 现场改错键名或填越界值**静默失效**。与 `config.py:228-233` 的注释（「这 10 键一直在 robot.yaml 里生效却从没进过 SCHEMA」）是同一类问题，但这批 planner.* 键是**新加的**，仍未补登记。 |
| 3 | `main.py:637`；`main.py:651` | `step.layout_ttl_s` / `step.done_replay_s` **被读取却未登记进 `SCHEMA`** | 同上（StepLoop 的两个新键）。 |
| 4 | `planner.py:4697` | `plan_route_tracker_safe` 在当前配置/地图下抛 `PlannerError: tracker-safe 引擎无解` | **增量续算入口当前不可用**（`test_incremental_plan.py:61` 直接异常退出）。这是本轮最严重的功能性发现。 |
| 5 | `test_sim.py:10.8b` | `现役路线最近 0.0000 m`（靶板重叠） | **现役路线（68 点）会压到靶板**（物理重叠 0 m）。`test_sim_guard.py:10.2` 也报车体压靶板拍=78。⇒ 用户 2026-10-18 的「不许压靶子」约束**在当前路线上不满足**。 |
| 6 | `test_sim.py:10.7b` | `K 列 168 拍，违规 0` | 该项**通过**，但期望值写的是旧口径。 |
| 7 | `test_sim_guard.py:3.6` | `锚定=0.73462 vs 不锚定=0.70789`（锚定**反而更大**） | `anchor_radius_m=0.12` 太小 ⇒ 锚定几乎不触发（只 2 次），信念误差按 ε×弧长无界累积。期望值应为「锚定 < 不锚定」。 |
| 8 | `test_sim_guard.py:2.8 / 2.14`；`test_sim.py:7.4 / 7.5` | `越界 74 拍`；`0.2 s 延迟 diverged=True`；`0.8 s 延迟 completed=False len=0.93` | 当前 68 点路线（含 3 处 :PIVOT 折返）在仿真里**跑不完且会越界**；而 :PIVOT 折返点正是仿真**不建模**的（`sim.py:107-112`）⇒ 这些失败**不能直接迁移实车**，但说明仿真基线已与代码脱节。 |
| 9 | `test_planner.py:惩罚单元` | `reversals=0` / `seq_cost 差 = 0` | `_Tour.reversals` / `penalty_of` 在当前图上**惩罚未生效**（折返点 3 处惩罚前后都是 3）。`_record_leg_dirs` 在 `allow_reverse=False` 下未被触发，或 `mine_cells` 传入为空（`planner.py:3080`：`mine_cells = set() if allow_reverse else {...}`，在无倒车模式下**非空**，所以应生效）—— **根因未核实**（§7.1 #4）。 |
| 10 | `config.py:688-690` | `mission.settle_time_s` ← `path_executor.shoot_settle_time`；`mission.settle_eps_m` ← `path_executor.shoot_settle_speed` | 旧键映射里 `settle_eps_m` ← `shoot_settle_speed`（注释写的是「底盘速度小于此值视为停住」）。语义上 `settle_eps_m` 是**位置**容差、`shoot_settle_speed` 是**速度**容差 —— 两者单位不同（m vs m/s）。`mission.settle_speed_mps` 没有对应的旧键映射。⚠️ 这是一处**可疑的迁移映射**（本卡只读，未改）。 |
| 11 | `validate.py:201-224` | `_barrier_crossing` 判据只有一条（两端点行号分居第 6 行两侧） | 只在第 6/7 行内横向移动（`B6→D6→C6`）**不算跨越** —— 这是**有意的**（注释说明是第一版踩过的坑），非 bug。 |
| 12 | `physics.py:273-296` | `required_barrier_distance_m` 的注释说「不需要任何桥洞豁免」 | 但 `barrier_distance_segment_m` 的 docstring 仍说「桥洞 K 处要先用 `segment_outside_band()` 抠掉豁免段再调用本函数」—— **两处注释口径不一致**（函数本身已按 2026-10-11 澄清不再调用豁免）。 |
| 13 | `controller.py:292` | `curvature = 2.0 * sin_a / denom` 的 `denom` = `max(ld, lookahead_floor_m)` | `ld` 由**巡航速度**决定（不随瞬时速度变化），这是**有意设计**（防「减速→前视变短→转向变猛」正反馈）—— 但意味着**减速时曲率限速不降**，低速贴站时横向加速度可能超 `lat_acc_max_mps2`。⚠️ 设计取舍，非 bug（本卡只读，未改）。 |
| 14 | `config.py:690` | `mission.settle_eps_m` ← `path_executor.shoot_settle_speed` | 见 #10 的单位不一致问题。 |

### 7.3 本轮审计方法说明（可复现）

1. 用 glob 摸 `dzrobot_v2/src/dzbrain/**/*.py` 的文件清单与行数（21 个目标文件，共 15921 行）。
2. 用 grep 抽取 `def | class` 的全量定义与行号（§3 索引的输入）。
3. 逐文件 read 精读：blackboard / events / logging / geometry / config / task / state / contract / referee / ros_io / model / parser / validate / physics / pursuit/path / pursuit/controller / pursuit/lookahead / tools/sim / tools/sim_cli / main / planner（分段）。
4. §3 的 585 个 def / 61 个 class 由脚本**从源码逐行提取**（签名取自源码原文），非人工誊抄。
5. §5 的 205 个键由脚本**从 `SCHEMA` 逐条解析**。
6. 测试结果由 `python <test>.py` 在 `dzrobot_v2/src/dzbrain/tests` 下实跑得到（Python 3.12.10，无 pytest）。

---

## 附：本文覆盖边界

- **覆盖**：`main.py`、`core/{blackboard,config,events,geometry,logging,state,task}.py`、`io/{contract,referee,ros_io}.py`、`route/{model,parser,validate,physics,planner}.py`、`pursuit/{path,controller,lookahead}.py`、`tools/{sim,sim_cli}.py`。
- **不覆盖**（超出本次必覆盖清单）：`tasks/*`、`scout/*`、`tests/*`（仅 §6 引用）、`setup.py`、`__init__.py`、`config/*.yaml`（仅摘录 3 处）、`tools/route_check.py`、`tools/deploy_to_car.py`。
- **只读**：本轮未修改任何源码或配置；唯一写入目标是本文。
