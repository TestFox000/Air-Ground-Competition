---
来源: 本次代码级技术审计（2026-10-23，子智能体 link-doc）
取证: 实
更新: 2026-10-23
status: 现行
---

# 技术全景 E · 空地链路与工程设施（代码级审计）

> 任务卡：task-E（空地链路与工程设施）。审计人：link-doc（子智能体）。
> 工作区根：D:/User/Desktop/比赛存档/v1。
> 纪律：**只读代码**，未修改任何源码/配置/测试；唯一写入目标为本文档。
> 覆盖面：① 空地链路端到端（/nav/waypoint RELIABLE、/dzlink/* BEST_EFFORT、StepLoop 状态机、布局版本与过期、心跳/情报超时、桥洞窗口、D1/D1b/D2/D3）；② 工程设施（wf 门禁、部署唯一入口、aim_autotune、dzmem、dsh-plugin-*、.zhiyuan、V2.0、git_save、CI/门禁命令）。
> 既有文章（docs/20_资料索引库/空地链路/00~03）只引用、不重复展开 scout 内部细节；与代码冲突处以代码为准并并列写 A/B 版。

---

## §0 导读

### 0.1 一句话结论

空地链路是**两条 QoS 语义完全相反的话题通道 + 一个车端 10 Hz 状态机**的组合：机侧 `dzlink_supervisor`/`dz_send_waypoint` 把感知结果变成心跳、桥洞情报、靶/雷/障核对与布局 v2，经 BEST_EFFORT 下发；车侧 `DroneTask` 收下并落黑盒，`StepLoop` 在拿到 `field_layout` 后逐段规划（`/nav/waypoint` RELIABLE 通道）并回报段完成。链路**整体真机跑过 v1 情报门控**，v2 步进循环仍是"离线 + 一次段链路"，2026-10-18 审计判 **NO-GO**，根因是三块地基缺口（known 无空域来源 / 段规划不过门禁 / 布局无新鲜度门）。工程设施是"约定 > 代码"的治理层：wf 门禁、deploy_to_car 唯一部署入口、dzmem 本地记忆、dsh-plugin 生命周期管理器。

### 0.2 最关键的五个事实（读代码得出）

1. **两条链路是"语义相反"的通道**：`/nav/waypoint` 用 RELIABLE（丢包会重传），`/dzlink/*` 用 BEST_EFFORT（丢包即丢）。
   - 车侧真值：`src/dzbrain/dzbrain/io/contract.py:73-74`（waypoint_cmd）、`:94-97`（drone_in/drone_out）。
   - 机侧真值：`dzsmartscout/tools/link/dzlink_supervisor.py:100-106,1237-1242`；`dz_send_waypoint.py:73-76,80-88`。
2. **BEST_EFFORT 发 → RELIABLE 收 = 静默 0 条**，两端端点计数仍各为 1；**ack 是唯一硬证据**（`dzlink_supervisor.py:1158-1159` 注释；`docs/20_资料索引库/空地链路/01_协议v2与契约.md:35`）。
3. **布局 `known` 只增不减**，车端 `unknown = 不可通行`；`known=154`（14×11）才算完整布局（`main.py:808-824`；`docs/01:15`）。
4. **StepLoop 的 SEGMENT_DONE 只活 1~2 拍**，机侧 1.4 Hz 采样 → 漏收率约 86%（`main.py:646-657` 注释；`docs/02:34`）。这是 D1b。
5. **域号是车 118 / 机 0 两套环境**：非交互 SSH 默认 0，症状是"ping/SSH 全通、ROS 零报错、互相看不见"（`dzlink.env:7-15`；`dzros.env:16-17`）。

### 0.3 审计方法

- 工具：`read` / `grep` / `glob` / `ops_bash`（`wc -l`、`head`、`sed`、`find`），全程只读。
- 行号口径：本文所有 `路径:行` 指向**当前工作树（2026-10-23）**。`config/robot.yaml` 行号会随配置增长漂移，引用以键名为准（`docs/00_索引.md:56`）。
- 证据等级标注：**〔实〕**=本次代码级审计实读；**〔档〕**=引用既有文档/历史记录，未重新核验。
- 未核实一律写「未核实」，不凭记忆编造。

### 0.4 阅读地图

| 你想查 | 去哪 |
|---|---|
| 两条话题的 QoS/域号/收发契约 | §2.1、§5.2 |
| StepLoop 状态机与各阶段的判据 | §2.3、§4.2 |
| 布局 v2 字段与版本/过期语义 | §2.2、§2.3.5 |
| 心跳/情报超时、桥洞窗口 | §2.1.2、§2.3.4、§5.2 |
| D1/D1b/D2/D3 四个缺陷的机制与证据 | §2.5 |
| 机侧 dzlink_supervisor / dz_send_waypoint 逐函数 | §2.4 |
| scout 侦察模型（只引用，不展开） | §2.6、§3 |
| 工程设施逐项 | §2.7 |
| 全部函数的索引大表 | §3 |
| 线程/数据流 | §4 |
| 配置键汇总 | §5 |
| 测试与证据 | §6 |
| 未核实清单 | §7 |

---

## §1 模块地图

### 1.1 顶层目录地图（本文档覆盖范围）

| 目录/文件 | 一句话 | 规模 | 审计覆盖 |
|---|---|---|---|
| `dzrobot_v2/src/dzbrain/dzbrain/scout/` | 机端侦察仿真模型（纯函数，无 ROS） | 6 文件 1513 行 | §2.6（引用，不展开） |
| `dzrobot_v2/src/dzbrain/dzbrain/tasks/drone.py` | 车端无人机链路任务（5 Hz） | 1196 行 | §2.2 全精读 |
| `dzrobot_v2/src/dzbrain/dzbrain/main.py` | 车端入口 + StepLoop 步进循环 | StepLoop 段 334~1248 | §2.3 精读 |
| `dzrobot_v2/src/dzbrain/dzbrain/io/contract.py` | 车侧话题表 + QoS 单一真源 | 456 行 | §2.1 |
| `dzrobot_v2/src/dzbrain/dzbrain/tasks/mission.py` | 车端任务编排 + 段执行（D2 根因） | 4007 行 | §2.5.3 |
| `dzsmartscout/tools/link/dzlink_supervisor.py` | 机侧常驻指挥节点 | 2014 行 | §2.4 全精读 |
| `dzsmartscout/tools/link/dz_send_waypoint.py` | 机侧一次性发送器 | 452 行 | §2.4.2 |
| `dzsmartscout/tools/link/patches/patch_drone_sender.py` | 机侧域号补丁（方案 A） | 224 行 | §2.4.3 |
| `dzsmartscout/tools/drone/dzlink_*.py` | 机侧雷达/点云/序号信标辅助 | 约 600 行 | §2.4.4 |
| `dzsmartscout/tools/sim_airground/` | 空地联合仿真（18 臂 + 360 场） | 约 2500 行 | §2.4.5 |
| `dzrobot_v2/tools/` | 车端工具（含 wf 门禁、deploy_to_car） | 见下 | §2.7.1~§2.7.3 |
| `dzrobot_v2/tools/wf/` | 收口门禁（check/check_reuse/check_links/repo_index） | 565 行 | §2.7.1 |
| `tools/`（根级） | 根级工具（aim_autotune、pack_handoff） | 799 行 | §2.7.4 |
| `memory/` | dzmem 本地长期记忆（SQLite） | 262+60+73 行 | §2.7.5 |
| `dsh_plugin/`（D:/User/Desktop/ZhiYuan/dsh_plugin） | DSH 插件唯一存放处 + admin.mjs | admin 1397 行 | §2.7.6 |
| `.zhiyuan/` | 纸鸢心跳守护 v2 + 双保险 | 99+60 行 | §2.7.7 |
| `V2.0/` | 独立 git 仓（仅 .git + .gitattributes） | 1 文件 | §2.7.8 |
| `git_save/` | V1 大项目版本存档（20 个文件夹） | README 47 行 | §2.7.9 |
| `dzrobot_v2/config/robot.yaml` | 车端统一配置（唯一改参数处） | 968 行 | §5 |
| `dzsmartscout/tools/link/tests/` | dzlink 单测 | 81+24 项 | §6 |

> 说明：本任务卡把 `ml` 与 `aim_autotune` 归到 `dzrobot_v2/tools/`，但**实测 `dzrobot_v2/tools/` 下没有 `ml/` 也没有 `aim_autotune/`**（`ls` 返回 no such file）；`aim_autotune` 实际在根 `tools/aim_autotune/`，`verify_contract.py` 与 `air_ground_models/` 在 v1 中不存在。详见 §7 未核实清单第 1、2 条。

### 1.2 空地链路模块地图（数据流向）

    [机端感知] 相机/雷达 → dzuav → JSONL/ROS/UDP
                       │
                       ▼  decide()  认不出就丢，绝不猜
   ┌──────────────── dzlink_supervisor ─────────────────┐
   │ ① 1 Hz heartbeat     → /dzlink/drone_to_car (BE)  │
   │ ② intel(bridge/target/field) → /dzlink/drone_to_car │
   │ ③ field_layout v2   → /dzlink/drone_to_car (BE)  │
   │ ④ 订阅 car_to_drone ← ack / car_state              │
   │ ⑤ /nav/waypoint 下发路线（RELIABLE）                │
   └──────────────────────┬─────────────────────────────┘
                          │ DDS 域 118
                          ▼
   ┌──────────────── 车端 dzbrain（单进程多线程）─────────┐
   │ DroneTask(5Hz) 收 intel → P_DRONE / P_FIELD_LAYOUT   │
   │ StepLoop(10Hz) 读 P_FIELD_LAYOUT → 段规划 → 发段     │
   │ MissionTask(10Hz) 执行段 → 到站回报 SEGMENT_DONE     │
   │ car_state 回传（1~2 Hz）→ /dzlink/car_to_drone       │
   └──────────────────────────────────────────────────────┘

证据：`dzlink_supervisor.py:1-79`（模块 docstring 职责表）、`main.py:587-600`（StepLoop 状态机）、`tasks/drone.py:1-49`（收发表）。

### 1.3 工程设施模块地图

| 设施 | 入口 | 一句话 | 证据 |
|---|---|---|---|
| 收口门禁 | `cd dzrobot_v2 && python tools/wf/check.py` | 冻结区/复用/断链/过期四道红 | `wf/check.py:80-197` |
| 复用门禁 | `wf/check_reuse.py` | 新增 .py 未登记 = 红 | `wf/check_reuse.py:69-84` |
| 断链门禁 | `wf/check_links.py` | docs/** 相对链接必须存在 | `wf/check_links.py:25-56` |
| 目录索引 | `wf/repo_index.py --write` | 目录索引自动生成 | `wf/repo_index.py`（未逐行精读，见 §7） |
| 唯一部署入口 | `tools/deploy_to_car.py` | probe/backup/push/build/verify/all，sha256 对账 | `deploy_to_car.py:1-89,306-328` |
| 云台 PID 整定 | `tools/aim_autotune/autotune_pid.py` | id/sweep/verify/report 三步 | `autotune_pid.py:1-33` |
| 本地记忆 | `memory/dzmem.py` | 单文件 SQLite，12 子命令 | `dzmem.py:1-258` |
| DSH 插件管理 | `D:/User/Desktop/ZhiYuan/dsh_plugin/admin.mjs` | 唯一生命周期管理器（1397 行） | `dsh_plugin/README.md:8-18` |
| 纸鸢守护 | `.zhiyuan/watchdog.ps1` + `poll.ps1` | 5min/30min 双层看门狗 | `watchdog.ps1:1-99`、`poll.ps1:1-60` |
| 版本存档 | `git_save/<v>/` | 20 个版本归档 | `git_save/README.md:1-47` |
| 交接打包 | `tools/pack_handoff.py` | L0/L1/L2 三级交接包 | `pack_handoff.py:1-260` |

---

---

## §2 逐模块详解

### 2.1 话题与域号契约（车侧真源 `contract.py`）

#### 2.1.1 话题表与 QoS

车侧话题表是唯一真源，定义在 `dzrobot_v2/src/dzbrain/dzbrain/io/contract.py:58-110`。与空地链路直接相关的三条：

| 键 | 话题 | 类型 | QoS | 方向 | 证据 |
|---|---|---|---|---|---|
| `waypoint_cmd` | `/nav/waypoint` | `std_msgs/String` | RELIABLE / depth10 / VOLATILE（`QOS_EVENT`） | 机→车订阅 | `contract.py:73-74` |
| `drone_in` | `/dzlink/drone_to_car` | `std_msgs/String`（JSON） | BEST_EFFORT / depth10（`QOS_LINK`） | 机→车订阅 | `contract.py:94-95` |
| `drone_out` | `/dzlink/car_to_drone` | `std_msgs/String`（JSON） | BEST_EFFORT / depth10 | 车发布 | `contract.py:96-97` |

QoS 常量定义：`QOS_EVENT = QosSpec(depth=10, reliability="reliable")`、`QOS_LINK = QosSpec(depth=10, reliability="best_effort")`（`contract.py:37-41`）。

★ 关键坑：**BEST_EFFORT 发 → RELIABLE 收 = 静默 0 条**。两端端点计数（publisher/subscriber）都还是 1，但数据一条不传，且不报错（`dzlink_supervisor.py:21-22` 注释）。只有 `ack` 是"车确实收到了"的硬证据（`contract.py:96-97` 的 `drone_out` 方向 pub；`docs/01:35`）。

#### 2.1.2 域号：两端唯一真相源

| 项 | 值 | 证据 |
|---|---|---|
| 车侧 | `ROS_DOMAIN_ID=118`、`ROS_LOCALHOST_ONLY=0` | `dzrobot_v2/dzros.env:16-17` |
| 机侧 | `export ROS_DOMAIN_ID=118` | `dzsmartscout/tools/link/dzlink.env:15` |
| 机侧默认回退 | `DEFAULT_DOMAIN_ID = 118` | `dzlink_supervisor.py:126` |
| 生效时机 | 必须在 `rclpy.init()` **之前**灌进 `os.environ` | `dzlink_supervisor.py:22-23`、`patch_drone_sender.py:7-8` |

`bootstrap_domain`（`dzlink_supervisor.py:155`）在**模块导入期**读 `~/dzlink_tools/dzlink.env` 并 `os.environ.setdefault`（`:215+`），原因是 `rclpy.init()` 时才读 `ROS_DOMAIN_ID`，晚了就无效（`patch_drone_sender.py:7-8`）。非交互 SSH 默认域 0，症状是"ping/SSH 全通、ROS 零报错、互相看不见"（`dzlink.env:7-15`；`docs/02:16`）。

★ 另一个坑：**无人机飞行栈默认域 0**（Humble/FastDDS 默认），与链路域 118 是两套 ROS 图，别一概而论（`docs/01:99-107`）。

#### 2.1.3 心跳 / 情报超时 / 桥洞窗口（时序契约）

机侧 `SupervisorCore` 的默认周期与车侧真值对齐在 `CAR_CONTRACT`（`dzlink_supervisor.py:116-124`）：

| 键 | 车侧值 | 机侧默认 | 证据（车） | 证据（机） |
|---|---|---|---|---|
| 心跳超时 | 3.0 s | `heartbeat_period_s=1.0` | `tasks/drone.py:545`、`config/robot.yaml:890` | `dzlink_supervisor.py:948` |
| 情报过期 | 30.0 s | `info_ttl_s=30.0` | `tasks/drone.py:546`、`config/robot.yaml:891` | `dzlink_supervisor.py:952` |
| 桥洞等待窗口 | 2.0 s | `bridge_period_s=2.0` | `config/robot.yaml:725` | `dzlink_supervisor.py:949` |
| 车状态上报周期 | 0.7 s（≈1.4 Hz） | — | `tasks/drone.py:547` | `dzlink_supervisor.py:120` |
| 自动出发延迟 | 45.0 s | — | `config/robot.yaml:337` | `dzlink_supervisor.py:119` |
| 兜底穿洞格 | lower K10 / upper K5 | — | `route.cross_lower_default / cross_upper_default` | `dzlink_supervisor.py:122-123` |

★ **桥洞窗口只覆盖"两半场相隔 ≤2 s 到达"**：先到的半场入库，超窗口才到的半场只进遥测，路线已冻结（车侧不换洞）（`dzlink_supervisor.py:1167-1172`；`docs/20_资料索引库/空地链路/README 相关章节` 引用 `docs/01:66`）。机侧 `tick()` 因此把两半场**同轮成对发出**：`bridge_round_due`（`dzlink_supervisor.py:1173-1177`）。

#### 2.1.4 路线 token 语法（`/nav/waypoint` 裸文本）

车侧解析真源：`route/parser.py` + `route/validate.py`（引用，不展开）。机侧在下发前用 `validate_route` 做**形状校验**（不校验可达性），见 §2.4.1。语法表见 `dzlink_supervisor.py:559-627` 与 `docs/01:109-123`：

| 写法 | 含义 |
|---|---|
| `N10` | 航点（列 A–N × 行 1–11） |
| `D3*` | 该点停车打靶 |
| `N5,y / N5,x` | 朝向后缀，只允许挂最后一个 token |
| `R:F11` | 该段倒车（真车 `allow_reverse=false` ⇒ 默认不发） |
| `K9:SCAN_STOP` | 扫描后缀 |
| `K9:PIVOT` | 180° 折返声明 |
| `reset:N10,y` | 复位信念位姿 + 清路线 → IDLE |
| `load:文件名` | 读车侧文件并出发 |
| `SHOOT@G11` | 指令非坐标：车侧算候选链整链插入 |

★ 第二写者警告：`/nav/waypoint` 还有车上 `start_path_executor.sh:35`、`send_waypoint.py:20` 两处；与机侧 supervisor 同跑会让路线在两个来源间跳（`docs/01:123`）。

---

### 2.2 车端 `DroneTask`（`tasks/drone.py`，5 Hz）

`DroneTask` 是车侧空地链路的**唯一出入口**（`docs/00_项目总览/系统架构.md:43`）。职责：收 5 种 v1 报文 + v2 布局，发 `car_state` 与 `ack`。

#### 2.2.1 收发表（模块 docstring 逐行）

| 方向 | 载荷 | 动作 | 代码证据 |
|---|---|---|---|
| 收 | `heartbeat` | 刷新心跳；超时/恢复各发一次 `EV_DRONE_OFFLINE`/`EV_DRONE_ONLINE` | `drone.py:779-788`、`:1061-1073` |
| 收 | `bridge_passable` | 写 `P_DRONE`，发 `EV_BRIDGE_INFO` | `drone.py:808-824` |
| 收 | `target_zone` | 与本地 `config/map.yaml` 的 `targets` 比对 → `field_diff` | `drone.py:827-839` |
| 收 | `field_verify` | 同上；带 `coverage` 时按片内作用域判定 + 记覆盖账本 | `drone.py:892-954` |
| 收 | `field_layout` v2 | 校验后写 `P_FIELD_LAYOUT`（规划真源） | `drone.py:842-889` |
| 收 | 未知 `type` / 非法 JSON / 缺字段 | 记日志不崩 | `drone.py:737-776` |
| 发 | `car_state` | 1~2 Hz 周期上报 | `drone.py:1107-1144` |
| 发 | `ack` | 对带 `seq` 的消息回执 | `drone.py:790-805` |

#### 2.2.2 心跳与离线

- `_on_heartbeat`（`drone.py:779-788`）**先算 silent_s 再刷新 `last_heartbeat`**——反过来会让恢复事件丢掉唯一证据（代码注释明确写了顺序要紧）。
- `_check_heartbeat`（`drone.py:1061-1073`）只在 `online` 状态变化时发一次 `EV_DRONE_OFFLINE`，阈值 `drone.heartbeat_timeout_s`（默认 3.0 s）。
- 失效兜底：**心跳丢失不阻塞任务、不改变默认分支/默认地图**（"宁可漏打，不可卡死"，`drone.py:38-41`）。
- `drone.enabled=False` → 整任务空转：`tick()` 直接 `return`，一个 bridge 调用都不发（`drone.py:716-717`）。

#### 2.2.3 桥洞情报

`_on_bridge_passable`（`drone.py:808-824`）**刻意不校验格名是否在地图内**：穿洞格每局随机，无人机才是权威；本地没有这一格不代表情报坏，兜底靠 `MissionTask` 候选集合校验。归一化：`half` 转小写、`block` 转大写；`confidence` 用 `_as_float` 容错（`drone.py:817`）。

#### 2.2.4 覆盖语义（G5-2 / G5-3）

`CoverageLedger`（`drone.py:314-402`）是片级覆盖账本：

- 幂等键 = `(scan_id, slice.id)`（BEST_EFFORT 重发是常态，重复片整条忽略，`drone.py:922-954`）。
- 只有 `complete=True` 的片才把 `cells` 记入 `cells_done`（唯一"已探明"来源），分母 `14×11=154`（`drone.py:330,362-363`）。
- `complete=False` ⇒ 片内其余仍是 UNKNOWN，**不得**产生"本地要素消失"的差异（`drone.py:991`）。
- `_diff`（`drone.py:965-1026`）两条口径：旧报文无 coverage 走全图差集；带 coverage 走**片内作用域**（`only_local = local_scoped - got if coverage.complete else []`）。

`FieldVerifyConsumer`（`drone.py:405-519`）订阅 `EV_FIELD_VERIFY`，把雷/障/靶差异折算成汇总，由本任务写黑盒 `P_FIELD_VERIFY` / `P_DRONE_COVERAGE`。处置策略 `drone.field_verify_policy ∈ {off|record|alert|hold}`，缺省 `record`=只记录不阻断（`drone.py:311`）；**本任务自己绝不停车/改路线**（`drone.py:688-705` `field_verify_gate` 只给建议）。

★ 两键 `drone.coverage_enabled` / `drone.field_verify_policy` **未登记进 `core/config.py` 的 SCHEMA**（写域不含它），一律 `cfg.get(key, default)`（`drone.py:35-36,83-84`）。

#### 2.2.5 布局 `field_layout` v2 校验与版本语义

`normalize_field_layout`（`drone.py:181-294`）纯内存、无 I/O、不依赖本地地图；未知字段一律忽略。字段校验：

| # | 条件 | 结果 | 证据 |
|---|---|---|---|
| 1 | 必填/格式非法 | 整份拒收，保留上一份 | `drone.py:189-199,206-264` |
| 2 | 特征 ⊄ known（mines/obstacles/targets 含未观测格） | 拒收 | `drone.py:231-239` |
| 3 | `bridges` 为空 | 拒收 | `drone.py:241-244` |
| 4 | `target_cell` ∉ known | 拒收 | `drone.py:273-279` |
| 5 | `version < 当前` | 拒收 + `layout_stale+1` | `drone.py:854-860` |
| 6 | `version == 当前` 且内容（除 ts）不同 | 拒收 + `layout_rejects+1` | `drone.py:861-873` |
| 7 | `version == 当前` 且内容相同 | 心跳重发，不重复计数 | `drone.py:861-873` |

★ v1 报文（无 `known` 键）归一化后**不补 known 键**：车端 `_step_mode_enabled()` 用"有没有 known"区分 v2 步进循环 / v1 旧"规划一次"行为（`drone.py:217-229`；`main.py:466-468`）。补键会把旧报文误判成 v2。

`version` 与 `segment_id` **必须是两个号**：known 长格会让 version 涨，混成一个号则"SEGMENT_DONE 完成的是哪一段"不可判定（`docs/01:65`）。

---

### 2.3 车端 `StepLoop` 步进循环（`main.py:587-1248`）

#### 2.3.1 状态机

    状态机（phase 即 P_CAR_STATE["phase"]）：
        WAIT_LAYOUT → PLANNING → DRIVING → SEGMENT_DONE →（新布局）PLANNING
                                                ↘（无新布局）WAIT_INFO →（新布局）PLANNING
        SEGMENT_DONE（目标 = end）→ FINISH；规划失败/无目标/无位姿 → WAIT_INFO（不退出）

证据：`main.py:590-593`。阶段常量：

| 常量 | 值 | 证据 |
|---|---|---|
| `STEP_PHASE_WAIT_LAYOUT` | `WAIT_LAYOUT` | `main.py:344` |
| `STEP_PHASE_PLANNING` | `PLANNING` | `main.py:345` |
| `STEP_PHASE_DRIVING` | `DRIVING` | `main.py:346` |
| `STEP_PHASE_WAIT_INFO` | `WAIT_INFO` | `main.py:347` |
| `STEP_PHASE_SEGMENT_DONE` | `SEGMENT_DONE` | `main.py:348` |
| `STEP_PHASE_FINISH` | `FINISH` | `main.py:349` |
| `STEP_PHASE_SAFE_STOP` | `SAFE_STOP` | `main.py:350` |
| `STEP_PHASE_WAIT_TIMEOUT` | `WAIT_INFO_TIMEOUT` | `main.py:351` |
| `STEP_PHASE_ROS_LOST` | `ROS_LOST`（2026-10-18 现场事故新增） | `main.py:352-354` |

三条硬语义（`main.py:594-599`）：

1. `WAIT_INFO` **无超时**（`step.wait_timeout_s` 缺省 0 = 无限；>0 才超时退出，退出码 4）。
2. **换段只发生在段边界**：DRIVING 期间到达的新布局只记 version，绝不重规划。
3. **布局新鲜度**：规划前 `now - 收到时刻 > step.layout_ttl_s`（缺省 10 s，0=关）⇒ `WAIT_INFO(reason=layout_stale)`。

#### 2.3.2 单拍 `tick()`

`tick`（`main.py:747-760`）10 Hz 推进：`_poll_layout` → 按 phase 分发 → `_publish_car_state`。运行入口 `run`（`main.py:762-790`）在 `FINISH` 等后返回 `finish`；`WAIT_INFO_TIMEOUT`→`timeout`；`SAFE_STOP`→`safe_stop`；`ROS_LOST`→`ros_lost`；ROS 上下文失效→`context_dead`。

#### 2.3.3 布局轮询 `_poll_layout`

`_poll_layout`（`main.py:836-864`）：读黑盒 → `version <= self._seen_version` 直接忽略（旧版只每 5 s 记一次日志）→ 新布局更新 `_layout_rx_at = now`（**新鲜度计时清零**）→ 停止上一条完成单重发（D1b）→ WAIT_LAYOUT 状态自愈转 PLANNING。

#### 2.3.4 规划 `_plan` 与新鲜度门

`_plan`（`main.py:866-952`）的判据顺序：

- 无 `target_cell` → `WAIT_INFO`（车只等不做）。
- `from_cell == target_cell` → **同格停站**（recon 段 6 形态 F4(mine)→F4(stop)，0 token）也计一段并发 SEGMENT_DONE（`main.py:888-899`）。
- 规划前先验新鲜度（`main.py:900-915`）：过期且**非完整布局** → `WAIT_INFO(layout_stale)`；过期但**完整布局**（known=154 或 coverage complete）→ 按 D-022 视为静态场地信息允许规划，只说明一次（`main.py:906-913`）。
- 规划器依次尝试：tracker-safe(from_cell) → tracker-safe → tour（`scout/layout.py:298-316` 的 attempts 列表，同一口径在 `plan_mission_route`）。
- 段路线自检：`bad = [c for c in plan.cells if c in set(unknown)]` → 拒收（`main.py:555-558`）。
- 终点优先：目标 = `layout.end` 时 `plan.stop_kind = SEG_STOP_END`，**绝不在终点停站打靶**（`main.py:559-563`）。

#### 2.3.5 布局过期与完整性的判据

| 函数 | 逻辑 | 证据 |
|---|---|---|
| `_layout_is_stale` | `_layout_ttl_s > 0 且 _layout_rx_at > 0 且 now - _layout_rx_at > ttl` | `main.py:803-806` |
| `_layout_is_complete` | known 去重数 ≥ `MAP_ROWS*MAP_COLS`，或 `coverage.complete`，或 `coverage_complete` | `main.py:808-824` |
| `_layout_stale_reason` | reason 以 `layout_stale` 开头（测试/机端可判） | `main.py:826-833` |

★ sender ts **只作诊断**，不参与门控（机/车时钟不同步，拿它判过期必然误杀）（`main.py:796-801`）。

#### 2.3.6 段完成回报（D1b 的修复核心）

`_mark_segment_done`（`main.py:1017-1043`）闩锁最后一单完成回报：

    self._done_seq += 1
    self._done_latch = {segment_id, block, stop_kind, skipped, elapsed_s, seq, replay:0}
    self._done_replay_stopped = False

`_maybe_replay_done`（`main.py:1048-1066`）在 WAIT_INFO 期间按 `step.done_replay_s`（缺省 1.0 s）重发同一份闩锁，只把 `replay` +1，phase 仍为 WAIT_INFO。停止条件：`_stop_done_replay`（`main.py:1068-1073`）在新布局到达 / 开新段 / FINISH / SAFE_STOP / ROS_LOST 时调用。

#### 2.3.7 运行期位姿看门狗（2026-10-18 现场事故防线③）

`_odom_watch`（`main.py:1102-1124`）：DRIVING 中连续 `step.odom_watch_s`（缺省 6.0 s）没有新 `/odom/localization` ⇒ 判 `ROS_LOST`：停段 + 刹车 + 诊断 + 退出码 5，**不自动恢复**（`main.py:1126-1143`）。阈值语义：任何一帧新 odom 都把计时清零，间歇断流不误杀。

`_unknown_violation`（`main.py:1207-1218`）：belief 连续 `_STEP_UNKNOWN_STOP_TICKS=3` 拍不在 known 里 ⇒ `SAFE_STOP`（0.3 s，10 Hz）。

#### 2.3.8 `car_state` 上报（D1 的修复核心）

`StepLoop._publish_car_state`（`main.py:1160-1185`）写 `P_CAR_STATE`，字段：`phase / segment_id / known_count / target_cell / stop_kind / block / version / reason / ts / segments_started / segments_done / replans / wait_s / done_seq / done`。`car_state` 不是心跳，心跳靠 DroneTask 周期发（`docs/02:97`）。



---

### 2.4 机侧 `dzlink` 代码

#### 2.4.1 `dzlink_supervisor.py`（2014 行，常驻指挥节点）

**架构：纯逻辑层与 rclpy 层分离。** `SupervisorCore` 只吃「时间戳 + 输入样本」，吐「待发事件」，**完全不 import rclpy**；所以 Windows 本机可 `--dry-run` / 跑单测，不需要 ROS（`dzlink_supervisor.py:15-16`）。ROS 层是唯一 import rclpy 的地方（`make_qos` 在 `:1237`，`SupervisorNode` 在 `:1248`）。单测有硬约束：`test_no_rclpy_import_at_module_level` 断言模块导入期不得出现 `rclpy`（`tests/test_dzlink_supervisor.py:773-775`）。

**职责（docstring，`:3-11`）**：① 1 Hz 心跳 → `/dzlink/drone_to_car`；② 感知结果转发（bridge_passable / target_zone / field_verify）；③ 订阅 `/dzlink/car_to_drone` 解析 car_state / ack；④ 路线 / 复位下发 → `/nav/waypoint` RELIABLE。

**决策纯函数 `decide`（`:327-392`）**：一份感知样本 → [(event_type, canonical_payload)]。认不出来返回空列表（绝不猜）。支持外层包裹 `payload/data/intel/result/msg`（`:338-343`）。type 归一化：`t.replace("-","_").replace(" ","_")`（`:346`）。无 type 时按字段猜：half/direction → bridge_passable；mines/obstacles → field_verify；blocks → target_zone（`:348-357`）。

**桥洞情报 `_bridge_payload`（`:438-470`）**：half 归一化（lower/upper / low/bottom / up/top / 中文）；方向词 left→lower、right→upper 是**场地约定假设，不是源码事实**——默认打印映射表（`:453-462`、`_direction_map` `:473-476`）。`confidence` 夹到 [0,1]，默认 0.9。

**槽位语义 `slot_key`（`:535-545`）**：同一槽位的新内容替换旧内容。★ 这是修掉的一个真 bug：第一版把 lower/upper 存同一个 `bridge_passable` 记录，后到的 upper **直接覆盖** lower，表现是"只看到一个半场的情报"且不报错。现在 `bridge_passable:lower` / `bridge_passable:upper` 是两个独立槽（`:543-545`）。

**报文组装 `canonical_text`（`:479-509`）**：字段与车侧契约严格一致；带 `seq` → 车回 `{"type":"ack","of":N}`（硬证据）。

**内容指纹 `intel_key`（`:512-522`）** 与 **下发顺序 `EMIT_ORDER = ("bridge_passable","target_zone","field_verify")`**（`:132`，固定顺序不靠字典序）。

**路线形状校验 `validate_route`（`:563-600`）**：只校验形状不校验可达性。两个正则坑写在注释里（`:551-557`）：`1[01]|[1-9]` 必须先长后短，否则 K10/K11 全判非法；开头 `(?!,)` 必需，否则 `N5,y` 莫名变"形状不认识"。回归测试 `test_grid_columns_10_and_11_are_accepted`（`tests/test_dzlink_supervisor.py`；`README.md:277-278`）。

**`SupervisorCore` 核心循环 `tick`（`:1131-1194`）**：

1. ⑤ 过期情报只记 `intel_stale`，**停止下发**（车侧回落默认值）。
2. ④ 路线优先（路线一变，后面的情报就没意义）。
3. ① 心跳（`_next_hb` 到期即发；心跳也入 ack 台账，ack 率就是链路活度最灵敏指标）。
4. ② 情报按槽位遍历，**两半场同轮成对**（`bridge_round_due`）。

**输入三选一可叠加**：文件尾随 `FileTailer/GlobTailer`（`:665-745`）、ROS 话题、UDP JSON `UdpJsonSource`（`:757-797`）。命令文件 `CommandFile`（`:1365-1438`，JSONL 追加即生效，坏命令只跳过不崩）。日志 `JsonlLogger`（`:889-943`，64 MB 轮转，每次写 flush）。

**退出码**：0 正常 / 2 无 ROS 环境 / 3 发得出去但没有任何订阅者 / 1 参数错（`:48`）。

#### 2.4.2 `dz_send_waypoint.py`（452 行，一次性发送器）

职责：直接下发路线 / 桥洞情报 / 靶区 / 雷障 / 心跳，或监听车回传。**默认发 3 遍**（两入口都是 VOLATILE，订阅端还没被发现时一条就永久丢）（`:22-24`）。

- `send`（`:189-236`）：等 DDS 匹配（`pub.get_subscription_count()`），再给发现过程 1.5 s 让计数收敛；订阅者匹配数 = 0 时打印两种可能（车没起栈 / 两端 DDS 没互相发现）但**仍然按 repeat 发出**；返回 `0 if matched else 3`。
- `listen`（`:237-270`）：订阅 `/dzlink/car_to_drone`，收到 0 条 → 退出码 3。
- t12 修复 t4 真机缺陷 D4（`:52-68`）：`target` 新增 `--blocks` 别名；格名按 `[空格 , ; | 、]` 拆开，避免 `--mines A3,B4` 生成畸形格名 `"A3,B4"`（车侧 `_norm_blocks` 不拆逗号，会当一个格名进 field_diff 且全程不报错）。

#### 2.4.3 `patch_drone_sender.py`（224 行，机侧域号补丁）

方案 A 的机侧补丁，做两件事：① 写 `~/dzlink_tools/dzlink.env`（域号 118）；② 给 `dz_send_waypoint.py` 加"自动读 env + 打印实际域号"。必须在 import 阶段读 env（`:7-8`）。三处插入全部基于唯一锚点、幂等、先备份（`:10-13`）；NOSUB_BLOCK 缩进必须是 12 空格（`:105-106`）。最后 `py_compile` 检查，失败即回滚（`:206-211`）。

#### 2.4.4 机侧辅助节点 `dzsmartscout/tools/drone/dzlink_*.py`

| 文件 | 职责 | 关键证据 |
|---|---|---|
| `dzlink_seq_beacon.py` | 50 Hz 单调序号信标，跨机丢包精确对账（`span = max-min+1, 丢包率 = 1 - recv/span`） | `:1-20` |
| `dzlink_points_check.py` | 点云逐点数量/时序检查 | `:22-64` |
| `dzlink_livox_bridge.py` | Livox 点云 → ROS 桥接 | `:52-159` |
| `dzlink_mid360_pc2_launch.py` | MID-360 PC2 launch（`xfer_format 1→0`，回退 CustomMsg） | `:8-21` |
| `dzlink_seq_check.py` | 序号连续性检查 | （见 §3） |

#### 2.4.5 `dzsmartscout/tools/sim_airground/`（空地联合仿真）

18 臂 + 360 场仿真，给出 §2.5.5 的硬数字。关键模块：

- `link.py`（56 行）：`Link.apply(messages, takeoff_delay_s)` 做 0.06 s 量级延迟 + 抖动 + Bernoulli 丢包，**单发布者不建模乱序**，投递按接收时刻排序（`:1-21`）。
- `car_gate.py`（427 行）：`CarIntelGate.feed`（`:135`）、`intel_snapshot`（`:154`）、`MissionFlow.commit_decision`（`:234`）复刻车侧"采纳门"（候选 ∩ 地图桥洞）。
- `drone_model.py`（276 行）：`build_timeline`（`:129`）、`emit_messages`（`:181`）、`bridge_half`（`:264`）。
- `runner.py`（359 行）：`run_arm`（`:74`）、`blind_failure_analysis`（`:275`）、`radius_sweep/speed_sweep/sector_sweep`（`:295,313,327`）。
- `run_airground.py`（359 行）：`build_arm_matrix`（`:60`）、`run_all`（`:147`）。

---

### 2.5 D1 / D1b / D2 / D3 缺陷

> 命名提醒：`D1` 在**车侧**指"car_state 必须回报 StepLoop 段状态"；在**机侧 `dzlink`** 指 `_run_ros` 的 `finally` 引用未绑定局部变量（UnboundLocalError）。本文并列写 A/B 版，避免混淆。

#### 2.5.1 D1（车侧）：car_state 必须回报 StepLoop 段状态

| 项 | 内容 |
|---|---|
| 现象 | 2026-10-18 实车首跑：车到段末，机端永远等不到"这段完成"，检查点不推进 |
| 根因 | `_car_state_fields` 只读 `P_MISSION_PHASE`（DRIVE/FINISH），而段级真相在 `P_CAR_STATE`（StepLoop 写 SEGMENT_DONE/WAIT_INFO/ROS_LOST…） |
| 证据 | `tasks/drone.py:1085-1105`（★D1 注释）；`main.py:1160-1185`（`P_CAR_STATE` 字段） |
| 修法 | `_car_state_fields` 在场时以 `step["phase"]` 为准，并把 `segment_id/known_count/target_cell/stop_kind/version/reason/done/done_seq` 一并上报；变化判据 sig 含 `segment_id/target_cell/stop_kind`（同 phase 换段也要发） |
| 证据 | `tasks/drone.py:1107-1119`；回归 `tests/test_car_state_report.py`（4 组断言） |

★ 键名刻意写字面量 `"car_state"` 而不 import main 的 `P_CAR_STATE`：main → tasks 是单向依赖，反向 import 会成环（`tasks/drone.py:1093-1094`）。

#### 2.5.2 D1b：SEGMENT_DONE 只活 1~2 拍（漏收率 ≈86%）

| 项 | 内容 |
|---|---|
| 现象 | 段完成事件窗口只有 1 拍（`main.py:1017-1043` → `:978-991`），机侧 1.4 Hz 采样（`drone.py:547`）⇒ 漏收率约 86%，段末永久 WAIT_INFO |
| 加剧 | 发射器"催版"默认关；车端 `step.wait_timeout_s=0` 无限等（`robot.yaml:732`） |
| 修法 | 完成单闩锁 + WAIT_INFO 期间 1 Hz 重发 + 单调 `done_seq` |
| 证据 | `main.py:646-657`（D1b 注释）、`:1011`、`:1048-1066`、`:1177-1180`；`docs/02:30-37` |
| 状态 | 代码已提交（238f6ce，10-19），**未实车复验**（`docs/02:37`） |

#### 2.5.3 D2：到站上报差半格/一格

| 项 | 内容 |
|---|---|
| 现象 | 到站 `block` 与 target 差半格/一格（例：目标 E5 报 E4），机侧 `block==target` 判据不成立 ⇒ 永久 WAIT_INFO |
| 机制 | `MissionTask._install_route`（`mission.py:2857-2897`）**不复位 `_stall_last_s`** ⇒ 新段进度判据 `s - _stall_last_s >= _arrival_m` 永假 → 12 s 后"拐角绕圈兜底"强跳末点（`mission.py:3199-3228`），`done.block=目标 / pose_block=上一格` |
| 修法 | 按 `segment_id/target_cell` 关联，不用 `block` 字面相等；同时 `car_state` 带 `segment_id/target_cell/stop_kind/version` |
| 状态 | **未修**（`docs/02:46`；`EMITTER_CROSSCHECK.md` E4 同签名） |

#### 2.5.4 D3：射击停站 0 发 / 0 中

| 项 | 内容 |
|---|---|
| 现象 | 10-18 第七图实跑：F9 停站 30.10 s 硬死线 **0 发**；全轮 2 发（均在离站后的行进段）、0 中 |
| 根因 | **不在链路**：开火 7 条 AND + 提前量只进判据不进控制环 + 类别锁被闪烁清零 |
| 证据 | `docs/02:48-51`；见 `docs/20_资料索引库/视觉与云台/01_云台瞄准与开火链路.md` §6/§8（本篇不展开） |

#### 2.5.5 仿真给出的三个硬数字（30 组合 × 4 朝向）

| 策略 | 完赛率 | 预算通过率 | 蓝面命中率 | 均等待 |
|---|---|---|---|---|
| 无无人机 | 3.3% | 100%（n=1） | 0.8% | 0 |
| 理想揭示（权威门+桥前等待） | 100% | 75.0% | 75.0% | 2.14 s |
| 受限（起飞晚 8 s + 朝向误判） | 100% | 57.5% | 0% | 9.44 s |

- 无情报直冲失败概率 `29/30 = 96.7%`；叠加靶朝向，"完赛且蓝面得分" ≈ `1/120 = 0.83%`（`docs/02:72-82`；`docs/空地联合仿真设计.md` §9a）。
- 预算不过的根因**不是等待、是路线长度**：最长 K7/K1 24.85 m，0.15 m/s 巡航 172.62 s，加驻留即超 180 s ⇒ 建议巡航 ≥0.25 m/s。
- 关键结论：现车采纳门（候选 ∩ 地图桥洞）使无人机**无法纠正过期地图**——B2 臂理想情报 K9 被判 rejected，车按默认 K10 直冲封死桥室 5.92 s 中断。

#### 2.5.6 v2 的三块地基缺口（NO-GO 理由）

| # | 缺口 | 一句话证据 |
|---|---|---|
| 1 | **known 没有"空域"来源** | 机载 13 类里 `CELL_LABEL`/`BUCKET_*` 被显式丢弃，`known` 只收"有检出物"的格 ⇒ 可通行格≈特征格，段规划恒无解 |
| 2 | **段规划完全不过门禁** | `main.py:519-576` `plan_segment_from_layout` 直接规划，无 `check_planned_route`、无物理净空 |
| 3 | **布局没有新鲜度门** | `layout_ts` 不参与判据；`drone.info_stale_s` 只管桥洞情报（缺口 E 已补 `step.layout_ttl_s`） |
| 4 | 双写风险 | `/nav/waypoint` 命令通道仍在（`mission.py:3900-3955` `_handle_command`），与 `field_layout` 并列成为路线写者 |
| 5 | 完赛判定降级 | StepLoop 只在"段完成格 == layout.end"时 FINISH，不再校验雷覆盖/射击/靶位 |

证据：`docs/02:53-62`；缺口 E 的修复见 `main.py:633-644,803-833,900-915`。

#### 2.5.7 机侧四个必修坑（对端交叉验证得出）

| ID | 坑 | 后果/修法 |
|---|---|---|
| E1 | `known` 漏了**车当前格** | 车第一拍就 `SAFE_STOP：belief 连续落在未知格`（`main.py:1034` 同源）⇒ 每份布局 known 必须含车位姿所在格 |
| E2 | 特征类别与现役真值图冲突（B7 真值障碍，recon 报 target_blue） | `_stop_kind_with_spec()` 把 obstacles/targets 都当禁站格 |
| E3 | dry_full 建在第六图，车/机部署是第七图 | 按第六图发的布局会漏压雷、撞桶、停错靶位；机端真值钉 `~/dzlink_tools/map7.yaml` |
| E4 | `belief` 段完成差一格（B10→B11） | 按 `segment_id` 消费，不要用 `block == target` 字符串判据 |

---

### 2.6 `scout` 侦察模型（只引用，不展开内部细节）

`dzrobot_v2/src/dzbrain/dzbrain/scout/` 是**机端侦察仿真模型**（纯内存、无 ROS、无 DDS），对应步进循环协议 v2 §4。任务卡要求"引用即可，不重复展开 scout 内部细节"，故此处只给定位与入口，细节见 §3 索引大表：

| 模块 | 职责 | 规模 |
|---|---|---|
| `fov.py` | 4×2 格视场模型 + 面积重叠（`cells_in_view`、`overlap_fraction`） | 225 行 |
| `frames.py` | 蛇形条带帧位 + 任意格路线转帧位（`scan_poses`、`generate_snake_timeline`） | 250 行 |
| `layout.py` | 真值图 / known 可达 / 侦察前沿 / 目标格决策 / 布局 v2（`build_layout`、`LayoutStream`） | 704 行 |
| `heartbeat.py` | 变化即发 + 每秒重发（`heartbeat_schedule`、`LayoutFeed`） | 114 行 |
| `reference.py` | 用户手画路线对照（`USER_ROUTE_BLOCKS`） | 90 行 |
| `__init__.py` | `simulate()` 一站式入口 | 130 行 |

**入口**：`from dzbrain.scout import simulate, generate_snake_timeline, TruthMap`（`scout/__init__.py:7`）。`simulate` 的流程（`:85-110`）：`TruthMap.load → generate_snake_timeline → LayoutStream.layouts_for → heartbeat_messages → compare_routes`。

**与车端 StepLoop 的契约衔接**：`layout.build_layout`（`scout/layout.py:467-503`）产出的 dict 字段顺序即协议顺序，直接可下发；`version` 单调递增、`known` 只增不减。`LayoutStream` 的目标格决策是 **T30 修正**：任务目标优先（剩余雷 + 射击停站 + 终点），规划失败/目标未进 known 时回退"朝任务目标方向的最远可达侦察前沿格"（`:432-449` `choose_mission_target`）。

★ 与既有文章的关系：`docs/20_资料索引库/空地链路/00~03` 已覆盖协议与缺口，本文不重复展开；本文新增的是**函数级索引**（§3）与**配置键汇总**（§5）。



---

### 2.7 工程设施

#### 2.7.1 `wf` 门禁（收口前必跑）

四个脚本都在 `dzrobot_v2/tools/wf/`（565 行）。

**`check.py`（201 行）**：回答三个问题——冻结区有没有未登记脏文件、登记的 in_flight 有没有过期未收口、三本真相文件在不在（`:5-8`）。REQUIRED_DOCS = 决策台账/复用清单/工作流v2（`:31-35`）。逻辑：

- `git_status` 用 `git status --porcelain`，**显式 utf-8 + errors=replace**（Windows GBK 解码中文文件名会 UnicodeDecodeError）（`:56-77`）。
- 冻结区脏文件 glob 匹配 `cfg["frozen"]`，未在 `cfg["in_flight"]` 登记 = 红（`:133-145`）。
- in_flight 过期 / `owner` 为空 = 红（`:147-163`，"无 owner = 没人收口的悬空改动"）。
- 内联跑 `check_links.py` 与 `check_reuse.py`，非 0 即并入 fail（`:94-126`）。

**`check_reuse.py`（88 行）**：新增 .py（git status ??/A/AM）位于 `tools/` 或 `src/dzbrain/dzbrain/` 必须在 `docs/复用清单.md` 或 `docs/能力索引.md` 出现文件名，否则红；既有 tools/*.py 未登记只警告（`:3-11`）。排除 `__pycache__` / `tests` / _tmp_session / `backup` / `legacy` 五类目录（`:24`）。

**`check_links.py`（72 行）**：扫描 `docs/**/*.md` 与 `src/history/INDEX.md`、`AGENTS.md`（含 v1/docs/**），检查 markdown 相对链接是否存在；跳过 `_archive/history_cache/_work/_工具`（`:25-35`）。

**`repo_index.py`（204 行）**：目录索引自动生成。本次未逐行精读（见 §7）。

#### 2.7.2 部署唯一入口 `tools/deploy_to_car.py`（470 行）

**唯一部署入口**（AGENTS.md §3 禁止手工 scp 散文件）。六个子命令：`probe / backup / push / build / verify / all`（`:424-434`）。

关键常量（`:53-89`）：

| 常量 | 值 | 说明 |
|---|---|---|
| `WS` | `tools/../ = dzrobot_v2 根` | `deploy_to_car.py:52` |
| `CAR_HOST/USER/WS` | `duzhong.local / duzhong / /home/duzhong/dzrobot` | `:53-55` |
| `PUSH_FILES` | 7 项：src/dzbrain、dzrknn 头/源、config/robot.yaml、config_selfcheck.py、dzbrain.py、healthcheck.sh | `:56-74` |
| `BACKUP_PATHS` | robot.yaml、start_all.sh、src/dzbrain、src/dzrknn_dataprocess | `:77-83` |
| `BACKUP_ROOT` | `/home/duzhong/_backups`（**工作区之外**，否则 colcon 报重名） | `:85-87` |
| `BUILD_PACKAGES` | `["dzbrain","dzrknn_dataprocess"]` | `:89` |

★ **刻意不推 `config/map.yaml`**：车机不留地图文件，规划依据由无人机 `field_layout` 下发（`:62-65`）。

★ **目录推送必须用 tar 流就地替换**，绝不能用 `scp -r`：目标目录已存在时 scp 会把源目录整个塞进去变成 `dst/<basename>/...`，"推送成功"但车端一个文件都没更新（2026-10-08 真机踩过）（`:243-254` `push_dir`、`dir_swap_cmd`）。

★ **推送后必须做 sha256 对账**：`verify_pushed`（`:306-328`）逐项比对本地/车端 `local_manifest` ↔ `remote_manifest`（远端内嵌 Python 走文件系统，`:257-277`），缺/不一致即非 0 退出并中止后续编译（`main:455-459`）。

- `cmd_backup`（`:344-354`）：备份到 `_backups/dzbrain_deploy_<ts>`，并给工作区内历史备份目录加 `COLCON_IGNORE`。
- `cmd_push`（`:357-386`）：默认**不碰**车端 `auto_start`；只有 `--force-no-autostart` 才强制 false。
- `cmd_build`（`:389-402`）：dzbrain 构建类型改 `ros.ament_python` 后必须 `rm -rf build/dzbrain install/dzbrain` 清残留。
- `cmd_verify`（`:405-420`）：保留两个瞬时检查——dzbrain 包/入口存在性 + 云台补丁符号 `use_external_fire_gate`（证明补丁已编入）。自检已从 `all` 移除（2026-10-11 定版）。

#### 2.7.3 车端其它工具（`dzrobot_v2/tools/`）

| 工具 | 行数 | 一句话 |
|---|---|---|
| `plan_route.py` | 23928 B | 路线规划 + `--emit tokens --check` 门禁 |
| `route_check.py` | 68835 B | 路线检查（雷/靶/碰撞） |
| `shoot_planner.py` | 27459 B | 射击计划 |
| `global_planner.py` | 40246 B | 全局规划 |
| `calib_backfill.py` | 65563 B | 标定回填 |
| `config_selfcheck.py` | 3986 B | 配置自检 |
| `deploy_car.py` | 12436 B | **旧部署工具**（deploy_to_car 的前身，勿用） |
| `healthcheck.sh` | 22044 B | 出车体检（双钥匙开火门控/裁判回传/保活速率） |
| `scout_sim.py` | 8129 B | scout 仿真入口 |
| `sim_map_render.py / sim_param_sweep.py / sim_pursuit.py` | — | 仿真辅助 |
| `e2e_monitor.py / p0_verify.py / shoot_probe.py` | — | 端到端监控/验证 |

★ `deploy_car.py` 是旧入口，唯一现役是 `deploy_to_car.py`（AGENTS.md §3）。

#### 2.7.4 根级 `tools/`：aim_autotune + pack_handoff

**`tools/aim_autotune/autotune_pid.py`（335 行）**：云台自瞄 PID 自动整定，三步走：

- `id`（`:150-184`）：开环辨识 a(px/tick)、伺服时间常数、视觉端到端延迟。
- `sweep`（`:147-160`，plant_model.py）：用 P1 结果跑离线模型扫参，缩小候选集（不动云台）。
- `verify`（`:260-275`）：逐组上闭环，静止靶阶跃，按指标打分。

安全设计（`:27-33`）：整定期间把 `gimbal.max_shots` 强制改 0（打不出一发）；任何异常走 `finally` 恢复 yaml + 重启节点；每次试验 ≤12 s，总时长 ≤15 min。

`plant_model.py`（204 行）：`simulate`（`:35`）、`metrics`（`:91`）、`score`（`:125`）、`kp_stability_bound`（`:142`）、`sweep`（`:147`）。

**`tools/pack_handoff.py`（260 行）**：打包 agent 交接包 L0/L1/L2。L0=一页纸；L1=总览+规则+现状+台账+状态板；L2=车/机自研源码（排除 .git/模型/录像/_tmp_session）。文本自动脱敏 `sshpass / sudo -S / password= / user:pass@@`（`REDACT_RULES` :82-88`）；L2 提示"外发前必须人工复核"（`:186,222-226`）。输出 `docs/agent_handoff/packs/*.zip` + `MANIFEST.md`。

#### 2.7.5 `memory/`：dzmem

`memory/dzmem.py`（262 行）是"大肥鱼的本地长期记忆"，**单文件 SQLite，零依赖**，四张表 `facts / events / preferences / people`（`SCHEMA` :19-49`）。12 个子命令：`add / event / diary / pref / person / search / recent / today / digest / stats / export`（`:163-258`）。无 HTTP/MCP。数据文件 `memory/dzmem.db`（42 行 SQLite）。

`memory/seed.py`（60 行）把 10-18 晚之前的事实/时间线写进 dzmem（幂等：先清空再写）。

★ 设计边界：dzmem 是**单机本地记忆**，不与车/机共享；它是治理辅助而非链路组件。

#### 2.7.6 DSH 插件（`dsh-plugin-*`）

**唯一存放处**：`D:/User/Desktop/ZhiYuan/dsh_plugin`（README 标题"DSH 插件的唯一存放处"，`:1-4`）。插件实体全在这里，profile 里只放 junction；**一个目录一个插件，删除即卸载**。

现役 6 个本地插件（`plugins.json:6-61`）：

| 插件 | files | bundle | 来源 |
|---|---|---|---|
| `dsh-deepseek-peak-whale` | 72 | true | `C:/Users/TestFox/.dsh/local-packages/...` |
| `dsh-our-free-model` | 80 | true | `.../vendor/dsh-our-free-model-2.5.6.tgz` |
| `dsh-studymate-seat` | 5 | false | `D:/User/Text/...` |
| `dsh-dzairground-ssh` | 112 | true | `D:/User/Desktop/比赛存档/v1/tools/plugins/...` |
| `dsh-dzdrone-ssh` | 98 | false | 同上 |
| `dzrobot-ssh` | 122 | false | 同上 |

★ **protocol 全部 `link`**（`plugins.json` 每个条目）：这是 2026-10-23 21:20 的迁移结论——`file:` 会被 pnpm 打包导入，把 D 盘目录搬空（21:08 实测砸了 3 个插件）；`link:` 只建符号链接（`README.md:20-40`）。

★ `plugins.json` 里 dzairground/dzdrone/dzrobot 三个插件的 `source` 指向 `D:/User/Desktop/比赛存档/v1/tools/plugins/...`，但**该目录在当前工作树不存在**（`ls tools/plugins` → No such file）。实体已迁到 `D:/User/Desktop/ZhiYuan/dsh_plugin/<name>`（§7 未核实第 3 条）。

工具集（`dsh_plugin/tools/`）：`selftest.mjs`（478 行，72 项端到端自测）、`dsh-file-dep-watchdog.ps1`（39 行，每 5 分钟 `admin repair`）、`finalize.md`。

`admin.mjs`（1397 行）：唯一生命周期管理器，命令 `list / verify / enable / disable / remove / install / repair`（`README.md:10-17`）。`verify` 是门禁（6 通过 0 失败才继续）。生效条件：改插件文件后 **Host 半身要重启 DSH** 才换（`README.md:60`）。

#### 2.7.7 `.zhiyuan` 纸鸢守护

**`watchdog.ps1`（99 行）**：主守护，每 5 分钟执行。职责：检测 DSH 会话假死/崩溃，假死时重装目标并发送"继续"唤醒；进程死了重启应用。三步：

1. GUI 活着吗：探测端口 19387 Listen；死了就 `Start-Process` 重启（`:40-49`）。
2. 心跳新鲜度：**双源**——会话日志 `session.v4.jsonl.zstd` 的 mtime 优先，`heartbeat.txt` 兜底；取较小值（`:51-63`）。v1 只看 heartbeat.txt，长工具调用（>20 min）会被误判假死。
3. 冷却 10 min 后：WScript.Shell 激活窗口，**无条件**先发 `/goal resume` 再发 `继续`（`:69-92`）。

★ v2 补的窟窿：`dsh-goal` 的 session-start 处理函数每次强制 `activation=disarmed`，唯一重新武装路径是 `ctx.goals.resume()`（`/goal resume`）；不 resume 就只剩一轮，自动续跑永久失效（`:7-16`）。

★ 编码坑：必须存 UTF-8 **带 BOM**，否则 PS 5.1 按 GBK 解码，`继续` 变乱码敲进 GUI（`:18-19`）。

**`poll.ps1`（60 行）**：第二层守护，每 30 分钟。唯一理由：主守护 2026-10-16 被静默 Disabled，第 2 层整层失效却无提示。它检查主任务存在/Enabled/电池策略，异常自动修回；修好就从 issues 摘掉只留 FIXED 记账，退出码保持 0（避免 LastTaskResult 长期非 0 被误读）（`:1-60`）。退出码语义：`267009/0x41301`=正在运行、`267011/0x41303`=从未运行，都不是故障（`:45-48`）。

#### 2.7.8 `V2.0`

`V2.0/` 是一个**独立 git 仓**，当前只含 `.git`（含 .gitattributes、.git/config、.git/HEAD=refs/heads/main）与一个 `.gitattributes`，**没有工作树文件**（`find V2.0 -not -path "V2.0/.git/*"` 只返回 2 项）。与 `git_save/` 的 v4.0.1/v4.0.2（V1 根仓）不是一个东西。

#### 2.7.9 `git_save`

`git_save/` 是 V1 大项目版本存档，**20 个文件夹 = 18 个 dzrobot_v2 + 2 个 V1 根仓**（`git_save/README.md:1-22`）。生成方式 `git archive <tag> | tar -x`，内容与源标签逐一比对通过。版本映射：

- v1.1.0 ~ v4.0.0 = dzrobot_v2 仓的标签（v1.1.0 d2e4da1 … v4.0.0 193567a）。
- V1 根仓的 v4.0.0（2cce6c7）→ 改记 **v4.0.1**；v4.0.1（5ec96a1）→ 改记 **v4.0.2**（避免与 dzrobot_v2 冲突）。
- 2026-10-23 16:32 物理改号落地，旧标签对象备查 `3dd6e4f`（原 v4.0.0）、`b4cfb18`（原 v4.0.1）；GitHub origin 暂不通待核。

#### 2.7.10 CI / 门禁命令

- **本仓库无项目级 CI**：`find . -name "*.yml" -path "*workflow*"` 只命中第三方源码 `dzsmartscout/src/mavros/.github/workflows`、`px4_msgs/.github/workflows`、`px4_ros_com/.github/workflows`（coverage/main/semgrep/stale/build），以及 `git_save/v4.0.1/.ssh2-tmp/node_modules` 里的第三方 CI。**没有 `.github/workflows` 项目配置**（§7）。
- 门禁命令（AGENTS.md §4/§9.6）：

    cd D:/User/Desktop/比赛存档/v1
    python docs/_工具/gen_index.py --ns <ns>        # 只重扫本 ns
    python docs/_工具/check_doc_policy.py           # R1/R2/R3 红
    cd dzrobot_v2 && python tools/wf/check.py       # 冻结区/复用/断链

- 文档守卫 `docs/_工具/check_doc_policy.py` 三条红线：R1 库外新增 .md、R2 docs 下开新库、R3 库内写了没登记（AGENTS.md §8.6）。白名单真源 `docs/_工具/doc_policy.json` + `doc_policy_baseline.json`。



---

## §3 函数级索引大表（重点）

> 口径：函数名 / 位置（路径:行）/ 一句话职责。优先穷尽与空地链路、工程设施直接相关的模块。标记 `★` = 本次逐行精读；`○` = 由 `grep` 行号索引得出，未逐行精读（见 §7）。

### 3.1 `scout` 侦察模型（`dzrobot_v2/src/dzbrain/dzbrain/scout/`）

| 函数/类 | 位置 | 一句话 |
|---|---|---|
| `simulate` | __init__.py:85 | 一站式入口：真值图→帧位→布局→心跳→对照 |
| `ScoutResult` | __init__.py:45 | 产物 dataclass（truth/frames/layouts/heartbeat/stats/comparison） |
| `cell_centers` | fov.py:69 | 全部 154 格的 (格名, 格心 x, y) |
| `all_blocks` | fov.py:81 | 全部格名（行优先） |
| `block_key` | fov.py:87 | 行优先排序键 |
| `sorted_blocks` | fov.py:93 | 去重 + 行优先排序（报文确定性顺序） |
| `cells_in_view` | fov.py:103 | 该姿态视场覆盖格名（格心严格在内） |
| `view_blocks` | fov.py:122 | `cells_in_view` 兼容别名 |
| `footprint_corners` | fov.py:125 | 视场矩形 4 角 |
| `polygon_area` | fov.py:138 | 鞋带公式面积 |
| `clip_convex` | fov.py:167 | Sutherland-Hodgman 凸多边形求交 |
| `overlap_fraction` | fov.py:191 | 相邻帧物理面积重叠率 |
| `cell_overlap_fraction` | fov.py:200 | 相邻帧格集合 Jaccard（诊断） |
| `is_node` | fov.py:209 | 姿态是否落在格点（0.35 m 整数倍） |
| `ScoutFrame` | frames.py:42 | 帧 dataclass（index/t/pose/cells） |
| `scan_poses` | frames.py:74 | 推荐蛇形路线帧位序列（含转角双帧） |
| `frames_from_poses` | frames.py:120 | 姿态序列 + 时间戳 + 视场格集 |
| `generate_snake_timeline` | frames.py:130 | 推荐蛇形路线 timeline |
| `frames_from_cell_route` | frames.py:148 | 格心路线转帧位（手绘路线对照） |
| `coverage` | frames.py:190 | 全部帧覆盖格（并集） |
| `adjacent_overlaps` | frames.py:198 | 相邻帧物理足迹重叠率 |
| `cell_overlaps` | frames.py:204 | 相邻帧格集合 Jaccard |
| `timeline_duration_s` | frames.py:210 | 时间线时长 |
| `timeline_stats` | frames.py:217 | 路线级指标（覆盖率/帧数/时长/重叠） |
| `TruthMap` | layout.py:70 | 本局真值图 |
| `TruthMap.from_map` | layout.py:86 | 从 YAML dict 构造 |
| `TruthMap.load` | layout.py:116 | 读 `config/map.yaml` |
| `blocked_for_driving` | layout.py:135 | 障碍/靶板不可站 |
| `step_ok` | layout.py:153 | 车从 cur 走到 nxt 是否合法（四类硬约束） |
| `reachable_cells` | layout.py:181 | known 图 BFS 可达集 |
| `frontier_cells` | layout.py:204 | 侦察前沿点 |
| `MissionTarget` | layout.py:219 | 目标格决策结果 |
| `known_map_cfg` | layout.py:233 | known 图 → planner 输入 |
| `plan_mission_route` | layout.py:268 | known 图上规划"剩余雷+停站+终点" |
| `mission_checkpoints` | layout.py:329 | planner token 串切检查点 |
| `plan_mission_target` | layout.py:359 | 任务目标优先 |
| `frontier_target` | layout.py:401 | 回退：最远可达侦察前沿 |
| `choose_mission_target` | layout.py:432 | 目标格决策总入口 |
| `choose_target_cell` | layout.py:452 | 兼容旧接口（只返回格名） |
| `build_layout` | layout.py:467 | 生成 field_layout v2 dict |
| `LayoutStream` | layout.py:506 | 按帧增量生成布局 |
| `LayoutStream.feed` | layout.py:622 | 喂一帧，有新格才产生布局 |
| `LayoutStream.retarget` | layout.py:633 | 车移动后重算目标格 |
| `LayoutStream.note_segment_done` | layout.py:643 | 车到达 target_cell：记已压雷/停站并推进 |
| `LayoutStream.observe_car_block` | layout.py:658 | 车位置回报 |
| `LayoutStream.mark_covered` | layout.py:672 | 显式标记已压雷 |
| `layouts_for_timeline` | layout.py:688 | 整条时间线 → 布局序列 |
| `heartbeat_schedule` | heartbeat.py:26 | 变化时刻 → [(t, version_index)] |
| `heartbeat_messages` | heartbeat.py:57 | 布局序列 → [(t, dict)] |
| `to_json` | heartbeat.py:72 | 规范 JSON |
| `LayoutFeed` | heartbeat.py:77 | "变化即发+每秒重发"无 I/O 状态机 |
| `LayoutFeed.push` | heartbeat.py:92 | 只接受更新版本 |
| `LayoutFeed.poll` | heartbeat.py:100 | 返回要发的布局 dict |
| `USER_ROUTE_BLOCKS` | reference.py:31 | 用户手画路线格序列 |
| `user_route_timeline` | reference.py:51 | 用户路线帧位时间线 |
| `compare_routes` | reference.py:58 | 推荐 vs 手绘路线对照 |

### 3.2 `tasks/drone.py`（车端 DroneTask，★）

| 函数/类 | 位置 | 一句话 |
|---|---|---|
| `default_map_path` | drone.py:120 | 仓库默认地图路径（上溯找 config/map.yaml） |
| `normalize_field_layout` | drone.py:181 | 校验并归一化 field_layout（★） |
| `_layout_cells` | drone.py:156 | 格名列表归一化 |
| `CoverageLedger` | drone.py:314 | 片级覆盖账本 |
| `CoverageLedger.update` | drone.py:341 | 记一条 coverage（new/duplicate） |
| `CoverageLedger._done` | drone.py:368 | 某次扫描 1..of 片齐全且全 complete |
| `CoverageLedger.summary` | drone.py:381 | 覆盖率快照 |
| `FieldVerifyConsumer` | drone.py:405 | EV_FIELD_VERIFY 生产订阅方 |
| `FieldVerifyConsumer.drain` | drone.py:440 | 取完当前事件队列 |
| `FieldVerifyConsumer._consume` | drone.py:447 | 逐条汇总（按 tag 分类） |
| `FieldVerifyConsumer._kind_of` | drone.py:483 | tag → target/hazard/other |
| `FieldVerifyConsumer.summary` | drone.py:492 | 汇总快照（含 severity/hold/alert） |
| `DroneTask` | drone.py:522 | 无人机情报收发任务（5 Hz） |
| `DroneTask.setup` | drone.py:610 | 生命周期：载地图、起消费方 |
| `DroneTask.teardown` | drone.py:633 | 解订阅 + 汇总日志 |
| `DroneTask.state` | drone.py:654 | 当前 P_DRONE 快照 |
| `DroneTask.bridge_of` | drone.py:664 | 便捷直读（转发 DroneState.bridge_of） |
| `DroneTask.field_verify_summary` | drone.py:669 | 核对消费汇总（+覆盖账本信号） |
| `DroneTask.field_verify_gate` | drone.py:688 | 给上层读的处置建议（不自己执行） |
| `DroneTask.coverage_summary` | drone.py:707 | 覆盖进度汇总 |
| `DroneTask.tick` | drone.py:714 | 单拍（空转 if disabled） |
| `DroneTask._drain_inbox` | drone.py:727 | 取无人机消息（16 条） |
| `DroneTask._handle_raw` | drone.py:737 | 单条报文解析入口（坏消息只记日志） |
| `DroneTask._on_heartbeat` | drone.py:779 | 心跳（先算 silent_s 再刷新） |
| `DroneTask._maybe_ack` | drone.py:790 | 对带 seq 的消息回 ack |
| `DroneTask._on_bridge_passable` | drone.py:808 | 桥洞情报 |
| `DroneTask._on_target_zone` | drone.py:827 | 靶区核对 |
| `DroneTask._on_field_layout` | drone.py:842 | 布局校验 + 版本语义（★） |
| `DroneTask._on_field_verify` | drone.py:892 | 雷/障/靶核对（+片级覆盖） |
| `DroneTask._take_coverage` | drone.py:922 | 解析并登记 coverage |
| `DroneTask._drain_field_verify` | drone.py:956 | 消费 EV_FIELD_VERIFY 成汇总 |
| `DroneTask._diff` | drone.py:965 | 登记差异并发 EV_FIELD_VERIFY（两条口径） |
| `DroneTask._record_diff` | drone.py:1028 | field_diffs 截断（diff_max=8） |
| `DroneTask._norm_blocks` | drone.py:1033 | 格名归一化（不校验地图内） |
| `DroneTask._warn_missing` | drone.py:1056 | 缺字段节流告警 |
| `DroneTask._check_heartbeat` | drone.py:1061 | 心跳超时（状态变化才发） |
| `DroneTask._silent_s` | drone.py:1075 | 静默时长（无心跳=inf） |
| `DroneTask._car_state_fields` | drone.py:1085 | **★D1**：取本拍段状态（StepLoop 优先） |
| `DroneTask._maybe_send_car_state` | drone.py:1107 | 周期/变化判据（sig 含 segment_id/target/stop_kind） |
| `DroneTask._send_car_state` | drone.py:1121 | 组装并发布 car_state |
| `DroneTask._publish` | drone.py:1149 | 写 P_DRONE |
| `DroneTask._load_local_map` | drone.py:1152 | 本地地图三来源 |
| `DroneTask._read_yaml` | drone.py:1174 | 读 YAML（坏地图不杀任务） |
| `DroneTask._block_set` | drone.py:1185 | 格名 → frozenset |
| `DroneTask._as_float` | drone.py:1191 | 容错转 float |

### 3.3 `main.py` 步进循环相关（★）

| 函数/类 | 位置 | 一句话 |
|---|---|---|
| `P_CAR_STATE` | main.py:341 | 车→机回报键（dict） |
| `STEP_PHASE_*` | main.py:344-354 | 步进阶段常量 |
| `_STEP_UNKNOWN_STOP_TICKS` | main.py:359 | belief 连续落在未知格的确认拍数=3 |
| `_step_cells` | main.py:362 | 布局格名列表校验 |
| `normalize_step_layout` | main.py:387 | 布局→步进归一化（兼容 v1，★） |
| `load_step_layout` | main.py:455 | 读黑盒 P_FIELD_LAYOUT |
| `_step_mode_enabled` | main.py:466 | 是否走运行期步进循环（v2=有 known） |
| `all_map_cells` | main.py:471 | 整图格名 |
| `_segment_map_cfg` | main.py:477 | 步进布局→段规划 map_cfg |
| `classify_segment_target` | main.py:497 | 段目标格停站类型 |
| `plan_segment_from_layout` | main.py:519 | 用步进布局规划一段（未知格不可通行，★） |
| `_find_task` | main.py:579 | 按线程名找任务 |
| `StepLoop` | main.py:587 | 车端步进循环（★） |
| `StepLoop.__init` | main.py:602 | 读 step.* / safety.* 配置 |
| `StepLoop.start` | main.py:721 | 吃进首份布局 + 复位 belief |
| `StepLoop.tick` | main.py:747 | 一拍（10 Hz） |
| `StepLoop.run` | main.py:762 | 阻塞跑步进循环 |
| `StepLoop._layout_ts_of` | main.py:796 | 布局机侧 ts（仅诊断） |
| `StepLoop._layout_is_stale` | main.py:803 | 布局是否过期 |
| `StepLoop._layout_is_complete` | main.py:808 | D-022 完整性判据 |
| `StepLoop._layout_stale_reason` | main.py:826 | layout_stale 原因串 |
| `StepLoop._poll_layout` | main.py:836 | 布局轮询（版本去重+新鲜度清零） |
| `StepLoop._plan` | main.py:866 | 规划（新鲜度门+同格停站+终点优先） |
| `StepLoop._drive_tick` | main.py:954 | DRIVING 单拍（位姿看门狗+完成+未知护栏） |
| `StepLoop._after_segment_done` | main.py:978 | SEGMENT_DONE 后续（终点→FINISH/新布局→PLANNING） |
| `StepLoop._wait_tick` | main.py:993 | WAIT_INFO 单拍（超时+重发完成单） |
| `StepLoop._mark_segment_done` | main.py:1017 | **★D1b**：闩锁完成回报 |
| `StepLoop._maybe_replay_done` | main.py:1048 | WAIT_INFO 期间 1 Hz 重发完成单 |
| `StepLoop._stop_done_replay` | main.py:1068 | 停止重发并清闩锁 |
| `StepLoop._to_wait_info` | main.py:1075 | 转 WAIT_INFO + 刹车 |
| `StepLoop._to_finish` | main.py:1088 | 转 FINISH |
| `StepLoop._odom_watch` | main.py:1102 | **位姿看门狗**（ROS_LOST） |
| `StepLoop._to_ros_lost` | main.py:1126 | 位姿断流→停段+刹车+诊断 |
| `StepLoop._to_safe_stop` | main.py:1145 | 转 SAFE_STOP |
| `StepLoop._publish_car_state` | main.py:1160 | 写 P_CAR_STATE（★D1） |
| `StepLoop._current_block` | main.py:1190 | 当前格（prefer_prev 可选） |
| `StepLoop._heading_dir` | main.py:1200 | 航向 → 网格方向 |
| `StepLoop._unknown_violation` | main.py:1207 | 连续 3 拍未知 ⇒ SAFE_STOP |
| `StepLoop._seed_pose` | main.py:1220 | 复位 belief |
| `StepLoop._default_planner` | main.py:1231 | 默认规划器 |
| `_run_step_loop` | main.py:1238 | run_live 的步进循环入口 |
| `_layout_mode_enabled` | main.py:1250 | 车端"布局模式"判定 |
| `_start_layout_listener` | main.py:1263 | 先起独立 DroneTask 等 field_layout |
| `_ros_diagnosis` | main.py:1294 | 可落盘的 ROS 现场诊断 |

### 3.4 `dzlink_supervisor.py`（机侧常驻，★）

| 函数/类 | 位置 | 一句话 |
|---|---|---|
| `_parse_env_file` | dzlink_supervisor.py:138 | 解析 env 文件（只认 export KEY=VALUE） |
| `bootstrap_domain` | dzlink_supervisor.py:155 | 导入期灌域号（早于 rclpy.init） |
| `default_log_path` | dzlink_supervisor.py:215 | 默认日志路径 |
| `expand` | dzlink_supervisor.py:221 | 展开 ~ |
| `_force_utf8` | dzlink_supervisor.py:225 | 强制 stdout/stderr UTF-8 |
| `diag` | dzlink_supervisor.py:244 | 诊断打印（带时间戳） |
| `IntelRecord` | dzlink_supervisor.py:252 | 情报记录 dataclass |
| `IntelRecord.age` | dzlink_supervisor.py:263 | 距上次更新的秒数 |
| `OutEvent` | dzlink_supervisor.py:267 | 待发事件 dataclass |
| `CarMsg` | dzlink_supervisor.py:281 | 车回传消息 dataclass |
| `CoreStats` | dzlink_supervisor.py:289 | 统计计数器 |
| `CoreStats.snapshot` | dzlink_supervisor.py:305 | 快照 |
| `decide` | dzlink_supervisor.py:327 | **感知样本→事件**（纯函数，★） |
| `_norm_block` | dzlink_supervisor.py:395 | k9/K9,y/SHOOT@K9 → K9 |
| `_block_list` | dzlink_supervisor.py:410 | 从候选键取格名列表 |
| `_bridge_payload` | dzlink_supervisor.py:438 | 桥洞情报→{half,block,confidence} |
| `_direction_map` | dzlink_supervisor.py:473 | 方向词→half（约定假设） |
| `canonical_text` | dzlink_supervisor.py:479 | 组装车侧契约 JSON |
| `intel_key` | dzlink_supervisor.py:512 | 内容指纹 |
| `slot_sort_key` | dzlink_supervisor.py:525 | 槽位排序（EMIT_ORDER） |
| `slot_key` | dzlink_supervisor.py:535 | **桥洞分半场槽位** |
| `validate_route` | dzlink_supervisor.py:563 | 路线形状校验 |
| `_route_token_problems` | dzlink_supervisor.py:603 | 单 token 形状检查 |
| `route_blocks` | dzlink_supervisor.py:630 | 抽格名序列 |
| `reset_command` | dzlink_supervisor.py:644 | 组装 reset: 命令 |
| `route_payload` | dzlink_supervisor.py:654 | 路线命令→裸文本 |
| `FileTailer` | dzlink_supervisor.py:665 | 文件尾随 |
| `GlobTailer` | dzlink_supervisor.py:725 | glob 尾随 |
| `sample_from_file_line` | dzlink_supervisor.py:746 | JSONL 行→样本 |
| `UdpJsonSource` | dzlink_supervisor.py:757 | UDP JSON 源 |
| `parse_car_message` | dzlink_supervisor.py:804 | 车回传字符串→CarMsg |
| `AckLedger` | dzlink_supervisor.py:819 | ack 台账（sent/ack/pending） |
| `AckLedger.note_sent` | dzlink_supervisor.py:842 | 记已发 seq |
| `AckLedger.note_ack` | dzlink_supervisor.py:851 | 记已收 ack |
| `AckLedger.pending` | dzlink_supervisor.py:866 | 未 ack 的 seq |
| `AckLedger.summary` | dzlink_supervisor.py:869 | 台账快照 |
| `JsonlLogger` | dzlink_supervisor.py:889 | JSONL 日志（64MB 轮转） |
| `JsonlLogger.log` | dzlink_supervisor.py:904 | 写一条 |
| `JsonlLogger._rotate` | dzlink_supervisor.py:923 | 轮转 |
| `CoreConfig` | dzlink_supervisor.py:945 | 核心配置 dataclass |
| `CoreConfig.normalise` | dzlink_supervisor.py:958 | 归一化周期/阈值 |
| `CoreConfig.period_for` | dzlink_supervisor.py:969 | 某类情报重发周期 |
| `SupervisorCore` | dzlink_supervisor.py:976 | 无 rclpy 决策核心 |
| `SupervisorCore.next_seq` | dzlink_supervisor.py:996 | 单调序号 |
| `SupervisorCore.submit_route` | dzlink_supervisor.py:1000 | 路线入队（契约校验） |
| `SupervisorCore.submit_reset` | dzlink_supervisor.py:1016 | 复位入队 |
| `SupervisorCore.submit_heartbeat` | dzlink_supervisor.py:1027 | 安排立即心跳 |
| `SupervisorCore.request_stop` | dzlink_supervisor.py:1032 | 请求停止 |
| `SupervisorCore.ingest_sample` | dzlink_supervisor.py:1040 | 吃感知样本落台账 |
| `SupervisorCore.ingest_car_raw` | dzlink_supervisor.py:1082 | 吃车回传字符串 |
| `SupervisorCore._stale_slots` | dzlink_supervisor.py:1112 | 过期槽位 |
| `SupervisorCore._make_intel_event` | dzlink_supervisor.py:1116 | 组装情报事件 |
| `SupervisorCore.tick` | dzlink_supervisor.py:1131 | **本拍该发什么**（★） |
| `SupervisorCore.status` | dzlink_supervisor.py:1197 | 状态快照 |
| `SupervisorCore.log_status` | dzlink_supervisor.py:1222 | 写状态日志 |
| `_short` | dzlink_supervisor.py:1226 | 短 JSON 串 |
| `make_qos` | dzlink_supervisor.py:1237 | 唯一 import rclpy 的地方 |
| `SupervisorNode` | dzlink_supervisor.py:1248 | ROS 层节点 |
| `SupervisorNode.dispatch` | dzlink_supervisor.py:1300 | 把事件搬上 DDS（返回未匹配数） |
| `SupervisorNode.spin_step` | dzlink_supervisor.py:1339 | 单步 spin |
| `SupervisorNode.destroy` | dzlink_supervisor.py:1355 | 销毁 |
| `CommandFile` | dzlink_supervisor.py:1365 | 命令文件（JSONL 追加） |
| `CommandFile.poll` | dzlink_supervisor.py:1372 | 增量读命令 |
| `CommandFile.execute` | dzlink_supervisor.py:1390 | 执行单条命令 |
| `build_argparser` | dzlink_supervisor.py:1440 | 命令行 |
| `_core_from_args` | dzlink_supervisor.py:1549 | 从 args 建 core |
| `_add_samples_from_args` | dzlink_supervisor.py:1568 | **★D2**：把 --bridge/--target/... 注入 core |
| `cmd_contract` | dzlink_supervisor.py:1589 | contract 子命令 |
| `cmd_selftest` | dzlink_supervisor.py:1626 | 纯逻辑自检 |
| `cmd_dry_run` | dzlink_supervisor.py:1725 | dry-run（内部已注入一次） |
| `_run_ros` | dzlink_supervisor.py:1745 | **常驻 ROS 主循环（★D1/D2 所在）** |
| `main` | dzlink_supervisor.py:1881 | 入口分发 |
| `_run_ros_once` | dzlink_supervisor.py:1947 | 一次性发送 |
| `_install_signal_handlers` | dzlink_supervisor.py:1993 | SIGINT/SIGTERM 处理 |

### 3.5 `dz_send_waypoint.py` / `patch_drone_sender.py` / 机侧辅助（★）

| 函数/类 | 位置 | 一句话 |
|---|---|---|
| `make_qos` | dz_send_waypoint.py:80 | QoS（RELIABLE/BEST_EFFORT） |
| `now_s` | dz_send_waypoint.py:90 | 当前秒 |
| `split_blocks` | dz_send_waypoint.py:101 | 按 [空格 , ; | 、] 拆格名 |
| `had_separator` | dz_send_waypoint.py:120 | 是否多格 |
| `note_split` | dz_send_waypoint.py:125 | 拆分告警 |
| `route_payload` | dz_send_waypoint.py:135 | 路线→裸文本 |
| `bridge_payload` | dz_send_waypoint.py:143 | 桥洞情报 JSON |
| `heartbeat_payload` | dz_send_waypoint.py:158 | 心跳 JSON |
| `blocks_payload` | dz_send_waypoint.py:165 | 靶区 JSON |
| `field_payload` | dz_send_waypoint.py:174 | 雷障 JSON |
| `send` | dz_send_waypoint.py:189 | 发 repeat 遍（等匹配+1.5s 收敛） |
| `listen` | dz_send_waypoint.py:237 | 监听车回传 |
| `parse_kv` | dz_send_waypoint.py:272 | key=value 解析 |
| `main` | dz_send_waypoint.py:282 | 入口 |
| `_read_link_env` | patch_drone_sender.py:59 | 读 dzlink.env |
| `domain_report` | patch_drone_sender.py:86 | 一行域号摘要 |
| `die` | patch_drone_sender.py:111 | 报错退出 |
| `write_env` | patch_drone_sender.py:116 | 写 dzlink.env |
| `patch_sender` | patch_drone_sender.py:149 | 三处锚点插入 + py_compile |
| `main` | patch_drone_sender.py:214 | 入口 |
| `SeqBeacon` | dzlink_seq_beacon.py:22 | 50 Hz 序号信标 |
| `SeqBeacon.tick` | dzlink_seq_beacon.py:44 | 递增发布 + 5s 统计 |
| `Check` | dzlink_points_check.py:22 | 点云检查节点 |
| `Check.cb_pc` | dzlink_points_check.py:38 | 点云回调 |
| `Check.cb_seq` | dzlink_points_check.py:57 | 序号回调 |
| `LivoxBridge` | dzlink_livox_bridge.py:52 | Livox→ROS 桥 |
| `LivoxBridge.on_seq` | dzlink_livox_bridge.py:92 | 序号对齐 |
| `LivoxBridge.on_cloud` | dzlink_livox_bridge.py:99 | 点云转换 |
| `generate_launch_description` | dzlink_mid360_pc2_launch.py:40 | MID360 PC2 launch |

### 3.6 `sim_airground`（联合仿真）

| 函数/类 | 位置 | 一句话 |
|---|---|---|
| `Link` | sim_airground/link.py:34 | 确定性链路（延迟+抖动+丢包） |
| `Link.apply` | sim_airground/link.py:41 | 应用链路 → 投递表 |
| `BridgeRecord/IntelVerdict/GateEvent/RouteVersion/MissionFlowResult` | car_gate.py:56-117 | 数据载体 |
| `CarIntelGate` | car_gate.py:119 | 车侧采纳门 |
| `CarIntelGate.feed` | car_gate.py:135 | 喂情报 |
| `CarIntelGate.intel_snapshot` | car_gate.py:154 | 情报快照 |
| `CarIntelGate._cell_ok` | car_gate.py:197 | 格是否可穿 |
| `MissionFlow` | car_gate.py:219 | 任务流 |
| `MissionFlow.commit_decision` | car_gate.py:234 | 提交决策 |
| `MissionFlow.load_route` | car_gate.py:260 | 装载路线 |
| `MissionFlow._plan` | car_gate.py:274 | 规划 |
| `MissionFlow._manual_rewrite` | car_gate.py:317 | 手工改道 |
| `MissionFlow._detect_cross` | car_gate.py:334 | 检测穿洞 |
| `MissionFlow.bridge_wait_decision` | car_gate.py:351 | 桥前等待决策 |
| `_rewrite_triple` | car_gate.py:391 | 穿洞三连改写 |
| `run_car` | carsim.py:46 | 跑一臂车 |
| `gap_metrics` | carsim.py:91 | 间隙指标 |
| `lead_table` | carsim.py:120 | 提前量表 |
| `DroneTimeline` | drone_model.py:82 | 无人机时间线 |
| `DroneTimeline.position` | drone_model.py:95 | t → 位置 |
| `build_timeline` | drone_model.py:129 | 建时间线 |
| `emit_messages` | drone_model.py:181 | 发消息 |
| `bridge_half` | drone_model.py:264 | 格 → 桥洞半场 |
| `run_arm` | runner.py:74 | 跑一臂 |
| `_planned_stop/_route_cells` | runner.py:52,63 | 停站/格集 |
| `_arm_to_dict` | runner.py:267 | 臂结果转 dict |
| `blind_failure_analysis` | runner.py:275 | 盲冲失败分析 |
| `radius_sweep/speed_sweep/sector_sweep` | runner.py:295,313,327 | 参数扫描 |
| `build_arm_matrix/run_all` | run_airground.py:60,147 | 臂矩阵/全跑 |
| `geo.*` | sim_airground/geo.py:21-53 | 格↔米↔索引 |

### 3.7 工程设施函数索引

**`dzrobot_v2/tools/wf/`**

| 函数/类 | 位置 | 一句话 |
|---|---|---|
| `glob_to_re` | wf/check.py:38 | glob→正则 |
| `git_status` | wf/check.py:56 | git status --porcelain（utf-8） |
| `main` | wf/check.py:80 | 收口门禁主流程 |
| `registered_text` | wf/check_reuse.py:27 | 复用清单/能力索引文本 |
| `new_code_files` | wf/check_reuse.py:35 | 新增 .py 列表 |
| `existing_tools` | wf/check_reuse.py:58 | 存量 tools 未登记 |
| `main` | wf/check_reuse.py:69 | 复用门禁 |
| `check` | wf/check_links.py:25 | 断链检查 |
| `main` | wf/check_links.py:59 | 断链门禁 |

**`dzrobot_v2/tools/deploy_to_car.py`（★）**

| 函数/类 | 位置 | 一句话 |
|---|---|---|
| `log/sh` | deploy_to_car.py:92,96 | 日志/子进程 |
| `resolve_ip` | deploy_to_car.py:109 | 解析车端 IP（mDNS 抖，最多 5 次） |
| `ssh_run` | deploy_to_car.py:126 | 远端命令 |
| `sh_bytes` | deploy_to_car.py:132 | 带 stdin 的远端命令 |
| `iter_payload` | deploy_to_car.py:153 | tar 载荷迭代 |
| `sha256_file/sha256_bytes` | deploy_to_car.py:165,173 | 哈希 |
| `normalize_auto_start` | deploy_to_car.py:177 | auto_start 归一化 |
| `expected_remote_bytes` | deploy_to_car.py:191 | 期望远端字节 |
| `local_manifest` | deploy_to_car.py:211 | 本地 sha256 清单 |
| `make_tarball` | deploy_to_car.py:224 | 打包 tar 流 |
| `dir_swap_cmd` | deploy_to_car.py:234 | tar 就地替换命令 |
| `push_dir` | deploy_to_car.py:243 | 目录推送（★ 绝不 scp -r） |
| `push_file` | deploy_to_car.py:250 | 单文件推送 |
| `remote_manifest` | deploy_to_car.py:280 | 远端哈希清单 |
| `diff_manifest` | deploy_to_car.py:298 | 清单差异（缺/多/不一致） |
| `verify_pushed` | deploy_to_car.py:306 | **★ sha256 对账闸门** |
| `cmd_probe/cmd_backup/cmd_push/cmd_build/cmd_verify` | deploy_to_car.py:333,344,357,389,405 | 六个子命令 |
| `main` | deploy_to_car.py:424 | 入口 |

**`tools/aim_autotune/` + `tools/pack_handoff.py` + `memory/dzmem.py` + `.zhiyuan`**

| 函数/类 | 位置 | 一句话 |
|---|---|---|
| `read_yaml/patch_yaml/restore_yaml` | autotune_pid.py:43,48,64 | YAML 读写/回滚 |
| `kill_node/start_node/node_alive` | autotune_pid.py:70,75,85 | 节点生命周期 |
| `Recorder` | autotune_pid.py:90 | 订阅 off/st/fb |
| `Recorder.spin/send_scan/cmd/close` | autotune_pid.py:124,129,136,141 | 采集 |
| `identify` | autotune_pid.py:150 | P1 开环辨识 |
| `compute_metrics` | autotune_pid.py:185 | 指标 |
| `one_trial` | autotune_pid.py:215 | 单次试验 |
| `verify/load_report/save_report/report/main` | autotune_pid.py:260-301 | 闭环验证/报告 |
| `Plant` | plant_model.py:24 | 被控对象模型 |
| `simulate/metrics/score/kp_stability_bound/sweep/main` | plant_model.py:35-161 | 离线模型 |
| `is_excluded` | pack_handoff.py:92 | 排除规则 |
| `sanitize` | pack_handoff.py:103 | 脱敏 |
| `sha/estimate_tokens` | pack_handoff.py:114,118 | 哈希/token 估算 |
| `collect_level/build/manifest/write_zip/main` | pack_handoff.py:122-255 | 打包流程 |
| `connect/add_fact/add_event/set_pref/upsert_person/search/recent/fmt_ts/stats/call_digest/export_json/main` | dzmem.py:52-258 | SQLite 记忆 API |
| `Log` | watchdog.ps1:32 | 日志 |
| `(watchdog.ps1 无命名函数，主流程 34-99)` | watchdog.ps1:34-99 | GUI 存活/心跳/冷却/唤醒 |
| `(poll.ps1 无命名函数，主流程 12-59)` | poll.ps1:12-59 | 主守护巡检+自愈 |


---

### 3.8 `tasks/mission.py` 段执行接口（★，D2 根因所在）

| 函数/类 | 位置 | 一句话 |
|---|---|---|
| `BridgeIntel` | mission.py:305 | 单半场桥洞情报（from_drone/describe/as_dict） |
| `retag_block_token` | mission.py:344 | 单 token 换格 |
| `rewrite_crossing_tokens` | mission.py:363 | 穿洞三连换行改写 |
| `resolve_plan_engine` | mission.py:421 | 从 config 读引擎（未知回退 tour） |
| `plan_budget_seconds` | mission.py:439 | 规划时间预算 |
| `GateResult` | mission.py:465 | 门禁结果 |
| `route_check_script` / `load_route_check_module` | mission.py:502,515 | 载入 route_check 模块 |
| `_run_route_check` | mission.py:536 | 跑 route_check |
| `check_planned_route` | mission.py:570 | **双门禁（仅 tracker-safe 会跑）** |
| `FootprintReport` | mission.py:644 | 足迹诊断报告 |
| `vehicle_geometry` | mission.py:682 | 车体几何参数 |
| `footprint_diagnostic` | mission.py:737 | 车体扫掠诊断 |
| `AutoPlanBlocked` / `PlanOutcome` | mission.py:835,840 | 自动规划异常/结果 |
| `resolve_map_path` | mission.py:907 | 解析地图路径 |
| `plan_route_from_config` | mission.py:941 | 从 config 规划 |
| `_stop_kind_with_spec` | mission.py:1117 | 单停站分类（shoot/mine/path/end） |
| `stop_kind_for_cell` | mission.py:1150 | **T21 判据唯一实现** |
| `SegmentPlan` | mission.py:1173 | 段规划结果 |
| `heading_dir_from_yaw` | mission.py:1209 | yaw → 网格方向 |
| `plan_segment_route` | mission.py:1229 | 段规划（支持 blocked_extra） |
| `_StopContext` | mission.py:1455 | 停站上下文 |
| `MissionTask` | mission.py:1476 | 任务编排（★） |
| `MissionTask.request_segment` | mission.py:1789 | StepLoop 投递段 |
| `MissionTask.stop_segment` | mission.py:1820 | 停段 |
| `MissionTask.segment_status` | mission.py:1827 | 段执行状态（done/phase/error） |
| `MissionTask.reset_pose` | mission.py:1840 | 复位 belief |
| `MissionTask._process_segment_requests` | mission.py:1844 | 处理段请求 |
| `MissionTask.bridge_state` | mission.py:1937 | 桥洞状态 |
| `MissionTask._half_candidates` | mission.py:2009 | 半场候选桥洞 |
| `MissionTask._bridge_pool` | mission.py:2024 | 桥洞池 |
| `MissionTask._intel_cell_ok` | mission.py:2044 | 情报格是否可穿 |
| `MissionTask._bridge_subscription` | mission.py:2078 | 订阅桥洞情报 |
| `MissionTask._record_intel` | mission.py:2109 | 记情报 |
| `MissionTask._drain_bridge_events` | mission.py:2125 | 取桥洞事件 |
| `MissionTask._intel_of` | mission.py:2156 | 单半场情报 |
| `MissionTask._refresh_intel` | mission.py:2195 | 刷新情报 |
| `MissionTask._intel_complete` | mission.py:2221 | 两半场都齐 |
| `MissionTask._has_fresh_half` | mission.py:2226 | 有新鲜半场 |
| `MissionTask._should_wait_for_intel` | mission.py:2234 | 是否继续等情报 |
| `MissionTask._bridge_wait_cap` | mission.py:2267 | 桥前等待上限 |
| `MissionTask._decision_point` | mission.py:2276 | 决策点（改道点） |
| `MissionTask._maybe_enter_bridge_hold` | mission.py:2307 | 进桥前等待 |
| `MissionTask._run_bridge_hold` | mission.py:2340 | 执行桥前等待 |
| `MissionTask._release_bridge_hold` | mission.py:2357 | 释放等待 |
| `MissionTask._reroute_at_decision` | mission.py:2373 | 决策点改道 |
| `MissionTask._apply_intel_to_tokens` | mission.py:2416 | 把情报应用到 token |
| `MissionTask._swapped_tokens_ok` | mission.py:2474 | 换洞后路线是否可用 |
| `MissionTask.setup/teardown/tick` | mission.py:2509,2527,2546 | 生命周期 |
| `MissionTask._maybe_auto_start` | mission.py:2587 | 自动出发 |
| `MissionTask._load_startup_route` | mission.py:2640 | 启动路线装载 |
| `MissionTask._start_mission` | mission.py:2722 | 出发 |
| `MissionTask._reset_belief` | mission.py:2798 | 复位信念 |
| `MissionTask._load_tokens` | mission.py:2814 | 装载 token |
| `MissionTask._load_single_point` | mission.py:2827 | 单点路线 |
| `MissionTask._install_route` | mission.py:2857 | **装路线（唯一真源）** |
| `MissionTask._classify_stops` | mission.py:2925 | 停站分类 |
| `MissionTask._shoot_planner` | mission.py:3000 | 射击计划 |
| `MissionTask._current_s` | mission.py:3031 | 当前弧长 |
| `MissionTask._advance_waypoints` | mission.py:3062 | 航点推进 |
| `MissionTask._detect_stall_and_skip` | mission.py:3163 | **停滞跳点（D2 机制）** |
| `MissionTask._pivot_in_progress` | mission.py:3250 | PIVOT 进行中 |
| `MissionTask._update_still_window` | mission.py:3299 | 车实际静止窗口 |
| `MissionTask._reset_stall` | mission.py:3329 | 复位停滞窗口 |
| `MissionTask._dist_to_stop` | mission.py:3369 | 到停站距离 |
| `MissionTask._should_trigger_stop` | mission.py:3397 | 是否触发停站 |
| `MissionTask._enter_stop_point` | mission.py:3418 | 进停站 |
| `MissionTask._phase_settle/_acquire/_fire` | mission.py:3497,3526,3542 | 停站三段式 |
| `MissionTask._complete_stop` | mission.py:3584 | 完成停站 |
| `MissionTask._skip_following_shoot_stop` | mission.py:3595 | 打中后跳过后续停站 |
| `MissionTask._set_fire_gate` | mission.py:3658 | 开火闸门 |
| `MissionTask._request_scan` | mission.py:3666 | 请求扫描 |
| `MissionTask._check_finish` | mission.py:3721 | 完赛判定 |
| `MissionTask._handle_command` | mission.py:3900 | **/nav/waypoint 命令处理（第二写者）** |

### 3.9 `dzsmartscout/tools/perception/`（机端感知，仅引用）

| 函数/类 | 位置 | 一句话 |
|---|---|---|
| `detect_arrows` / `infer_bridge_passable` | bridge_arrow.py:399,438 | 桥顶箭头检测 → 穿洞格 |
| `direction_from_points` | bridge_arrow.py:281 | 由点云方向推断箭头角度 |
| `detect_car` / `best_car` | car_pose.py:120,203 | 车位姿检测 |
| `cli.py 各 cmd_*` | cli.py:209-350 | detect/coverage/calibrate/synth/arrow/topics |
| `Palette` / `ColorRule` | colors.py:173,116 | 调色板/颜色规则（provisional 开关） |

### 3.10 `dzsmartscout/tools/sim_airground/`（补充）

| 函数/类 | 位置 | 一句话 |
|---|---|---|
| `DroneParams/LinkParams/MissionParams/Scenario` | scenarios.py:41-... | 参数与场景载体 |
| `run_random_maps` | random_maps.py:91 | 随机地图生成 |
| `draw_map` | plot.py:55 | 画图 |
| `_planned_stop / _route_cells` | runner.py:52,63 | 停站/格集 |


## §4 数据流与线程模型

### 4.1 车端线程模型（`TaskThread`）

车端 = **单进程多线程**，8 个任务线程 + 4 个 ROS executor 线程（`docs/00_项目总览/系统架构.md:13,35-46`）。任务基类 `TaskThread`（`core/task.py:59-160`）保证四件事：

1. **固定频率**：按绝对时基 `next_deadline += period` 排下一拍，补偿执行耗时（`:118-154`）；落后太多重置时基，避免追赶式连拍。
2. **异常隔离**：单拍异常不杀线程；连续 ≥ `max_consecutive_errors=5` → FAULT（`:127-138`）。
3. **健康度**：`TaskHealth`（last_tick_at/ticks/errors/overruns）供 SafetyTask 消费（`:38-56`）。
4. **优雅停止**：`stop()` 置位 + join，`teardown()` 保证调用（`:156-159`）。

任务线程频率（`docs/00_项目总览/系统架构.md:35-45`）：Localization 20 Hz、Safety 20 Hz、Tracker 50 Hz、Mission 10 Hz、Aim 50 Hz、Fix 10 Hz、Drone 5 Hz、Telemetry 2 Hz。**布局模式（StepLoop）下 DroneTask 会先单独起一个再起任务组（实际 8+1 线程），并强制 `mission.auto_start=false`**（`main.py:2063-2082`）。

### 4.2 线程通信契约

任务之间**只能**通过 Blackboard（高频状态）与 EventBus（低频事件）通信，禁止直接互相调用（`core/task.py:14`）。

| 通道 | 用途 | 例子 |
|---|---|---|
| `Blackboard` | 高频状态 | `P_POSE`、`P_DRONE`、`P_FIELD_LAYOUT`、`P_CAR_STATE`、`P_ROUTE` |
| `EventBus` | 低频事件 | `EV_DRONE_ONLINE/OFFLINE`、`EV_BRIDGE_INFO`、`EV_FIELD_VERIFY`、`EV_TASK_FAULT` |

布局模式的单向依赖规则：`main → tasks` 是单向依赖，所以 `StepLoop` 用字面量 `"car_state"` 而不是 import main 的 `P_CAR_STATE`（`tasks/drone.py:1093-1094`）。

### 4.3 布局模式启动顺序（功能前提）

`main.py:2045-2130` 的顺序**是功能前提**：MissionTask 的路线在构造/启动时就要定下来，所以布局规划必须发生在 build_tasks 之前。

    ① 布局模式判定（--map 不存在且 auto_plan_enabled=true）
    ② drone.enabled=False → exit 3（永远收不到布局）
    ③ 起独立 DroneTask 等 field_layout（bridge.spin_async(threads=4)）
    ④ _wait_for_field_layout（超时 exit 3，绝不回退 map.yaml/route.tokens）
    ⑤ step_mode = "known" in layout
       ├─ v2：cfg.override(mission.auto_start=False) → 步进赛时契约 → 任务组立即起 → StepLoop.start(first_layout)
       └─ v1：plan_route_from_field_layout → 写回 route.tokens + auto_plan_enabled=False
    ⑥ _wait_for_pose_ready（位姿就绪门，超时 exit 5）
    ⑦ group.start_all() → 步进循环 run / 旧 live loop

证据：`main.py:2045-2172`。★ 启动契约（`startup_contract`）在步进赛时检查 `drone_in` 本节点订阅 + 对端发布方；缺则 exit 6（`main.py:2086-2096`）。

### 4.4 机端线程模型

- `dzlink_supervisor` 是**单进程单线程**常驻节点：主循环 `_run_ros` 里轮询尾随/UDP/命令文件 → `core.tick()` → `node.dispatch()` → `node.spin_step()`（`:1809-1863`）。rclpy 回调（`_on_car`/`_on_perception`）由 `spin_step` 驱动，**不额外起线程**。
- `stdin` 读取用一条 daemon 线程（`_reader`，`:1790-1798`）。
- 纯逻辑层 `SupervisorCore` 无锁、单线程，输入在**主循环**里同步喂入；因此"同一槽位被两次 ingest"只在主循环内发生，无竞态。
- `dz_send_waypoint` 的 `send()`/`listen()` 各自 `rclpy.init/shutdown`，一次性进程。

### 4.5 端到端数据流（时序）

    T0      机: dzlink_supervisor run 起（心跳 1Hz + 尾随感知 + 命令文件）
           车: start_all.sh → 启动契约 → 位姿就绪门
    T1      机: intel(bridge/target/field) → /dzlink/drone_to_car (BE, 每条 repeat=3)
           车: DroneTask 收 → P_DRONE / P_FIELD_VERIFY → MissionTask 桥洞决策
    T2      机: field_layout v2（known 增量）→ /dzlink/drone_to_car
           车: DroneTask → P_FIELD_LAYOUT → StepLoop 转 PLANNING
    T3      车: StepLoop 规划一段 → MissionTask.request_segment(tokens, segment_id, target_cell, stop_kind)
           车: MissionTask 执行 → 到站回报 → StepLoop._mark_segment_done(SEGMENT_DONE + done 闩锁)
    T4      车: DroneTask 把 P_CAR_STATE 组装成 car_state → /dzlink/car_to_drone (BE)
           机: SupervisorCore.ingest_car_raw → ack 台账
    T5      机: 收到 SEGMENT_DONE/car_state 的 done 字段 → 决定下一段 → field_layout 新版本

★ 竞态点：DRIVING 期间到达的新布局只记 version，**绝不重规划**（换段只发生在段边界，`main.py:596-598`）。完成单闩锁在 WAIT_INFO 期间按 1 Hz 重发，直到新布局到达/新段开始/FINISH 才停（`main.py:1048-1073`）。

### 4.6 域号/时钟/时间基准

| 项 | 事实 | 证据 |
|---|---|---|
| 域号 | 车 118 / 机 118（链路）/ 无人机飞行栈默认 0 | `dzros.env:16-17`、`dzlink.env:15`、`docs/01:99-107` |
| 时钟 | 车 `monotonic`（`main.py:615`）；机 `time.time()`；布局 `ts` 只作诊断，不参与门控 | `main.py:796-801`、`dzlink_supervisor.py:506` |
| 统计周期 | 车 5 Hz（DroneTask）/ 10 Hz（StepLoop）；机 `loop_hz=20` 默认 | `drone.py:534`、`main.py:747`、`dzlink_supervisor.py:1806` |

---

## §5 配置键汇总

> 口径：`config/robot.yaml` 行号会漂移，引用以键名为准（`docs/00_索引.md:56`）。标注〔实〕=本次实读代码取值；〔档〕=引用既有文档。

### 5.1 车端无人机链路 `drone.*`

| 键 | 当前值 | 默认 | 语义 | 证据 |
|---|---|---|---|---|
| `drone.enabled` | true | — | false ⇒ 整任务空转 | `robot.yaml:889`、`drone.py:544` |
| `drone.heartbeat_timeout_s` | 3.0 | 3.0 | 心跳超时 → online=False + 一次告警 | `robot.yaml:890`、`drone.py:545` |
| `drone.info_stale_s` | 30.0 | 30.0 | 情报过期 → bridge_of() 返回 None | `robot.yaml:891`、`drone.py:546` |
| `drone.car_state_period_s` | 0.7（未在 yaml 写，走默认） | 0.7 | car_state 上报周期 | `drone.py:547` |
| `drone.ack_enabled` | true（默认） | true | 是否对带 seq 的消息回 ack | `drone.py:548` |
| `drone.field_diff_max` | 8（默认） | 8 | field_diffs 保留条数 | `drone.py:549` |
| `drone.coverage_enabled` | true（默认） | true | **未登记 SCHEMA**；false ⇒ 忽略 coverage | `drone.py:554` |
| `drone.field_verify_policy` | record（默认） | record | **未登记 SCHEMA**；off/record/alert/hold | `drone.py:555-557` |
| `drone.map_path` | — | config/map.yaml | 本地地图核对真源 | `drone.py:1155-1156` |

### 5.2 步进循环 `step.*` / 安全 `safety.*`

| 键 | 当前值 | 默认 | 语义 | 证据 |
|---|---|---|---|---|
| `step.wait_timeout_s` | 0.0 | 0.0 | WAIT_INFO 超时（0=无限等） | `robot.yaml:732`、`main.py:620` |
| `step.layout_ttl_s` | （未写，走默认） | 10.0 | 布局新鲜度门（0=关） | `main.py:637` |
| `step.done_replay_s` | （未写，走默认） | 1.0 | D1b 完成单重发周期（0=关） | `main.py:651` |
| `step.odom_watch_s` | （未写，走默认） | 6.0 | DRIVING 位姿看门狗阈值 | `main.py:625` |
| `safety.pose_timeout_s` | 1.5 | 1.5 | `_odom_fresh_s` | `robot.yaml:894`、`main.py:629` |
| `mission.auto_start` | false（步进模式被 override） | — | 步进赛时强制 false | `main.py:2082` |
| `mission.bridge_info_wait_s` | 2.0 | — | 等桥洞情报窗口 | `robot.yaml:725` |
| `mission.auto_start_delay_s` | 45.0 | — | 起栈后自动出发延迟 | `robot.yaml:337` |
| `mission.skip_second_stop_after_hit` | （t13 新增） | — | 第一停站打中则跳过第二停站 | `robot.yaml:810-818` |

### 5.3 规划 `planner.*` / 路线 `route.*`

| 键 | 当前值 | 默认 | 语义 | 证据 |
|---|---|---|---|---|
| `planner.engine` | tour | tour | 规划引擎（三引擎审计裁决） | `robot.yaml:749` |
| `planner.auto_plan_enabled` | false | false | 车上自动规划开关 | `robot.yaml:779` |
| `planner.tracker_max_seconds` | 10.0 | 10.0 | tracker-safe 时间预算（车端实测 2.40× 提速） | `robot.yaml:778` |
| `planner.allow_reverse` | false | false | 真车不支持倒车 | `robot.yaml:787` |
| `planner.max_seconds` | 2.0 | 2.0 | 规划时间预算 | `robot.yaml:788` |
| `planner.pivot_penalty` | 5.0 | 5.0 | 折返惩罚/处 | `robot.yaml:789` |
| `planner.mine_turn_penalty` | 2.0 | 2.0 | 雷格不得作拐点 | `robot.yaml:790` |
| `planner.obstacle_margin_m` | 0.02 | 0.02 | 障碍净空余量 | `robot.yaml:792` |
| `planner.footprint_gate` | warn | warn | 足迹诊断严重级（warn 不拦） | `robot.yaml:764` |
| `planner.fallback_to_tokens` | true | true | 规划失败回退 route.tokens | `robot.yaml:786` |
| `route.cross_lower_default` | K10 | — | 车侧兜底下洞格 | `CAR_CONTRACT dzlink_supervisor.py:122` |
| `route.cross_upper_default` | K5 | — | 车侧兜底上洞格 | `CAR_CONTRACT dzlink_supervisor.py:123` |
| `route.tokens` | 现役 68 token（仓库） | — | 手动路线（历史档） | `robot.yaml:530-598`、`docs/01:130` |

### 5.4 机侧 dzlink 配置

| 项 | 值 | 证据 |
|---|---|---|
| `CoreConfig.heartbeat_period_s` | 1.0 | `dzlink_supervisor.py:948` |
| `CoreConfig.bridge_period_s` | 2.0 | `dzlink_supervisor.py:949` |
| `CoreConfig.target_period_s` / `field_period_s` | 0.0（变了才发） | `dzlink_supervisor.py:950-951` |
| `CoreConfig.info_ttl_s` | 30.0（=车侧 drone.info_stale_s） | `dzlink_supervisor.py:952` |
| `CoreConfig.intel_repeat / route_repeat` | 3 / 3 | `dzlink_supervisor.py:953-954` |
| `CAR_CONTRACT` | info_stale 30 / wait 2 / auto_start 45 / hb_offline 3 / K10 / K5 | `dzlink_supervisor.py:116-124` |
| 域号 | `ROS_DOMAIN_ID=118` | `dzlink.env:15` |

---

## §6 测试与证据

### 6.1 机侧 dzlink 测试

| 测试文件 | 项数 | 覆盖 |
|---|---|---|
| `dzsmartscout/tools/link/tests/test_dzlink_supervisor.py` | **81** | 心跳节律/情报翻译与时效/报文契约/路线形状/ack 台账/文件尾随/命令文件/UDP/端到端/D1/D2/t16 成对 |
| `dzsmartscout/tools/link/tests/test_dz_send_waypoint.py` | **24** | 发送器报文与拆分 |

运行：`python -m unittest discover -s dzsmartscout/tools/link/tests -p "test_*.py"`（README 称 70 项，实际 grep `def test_` 得 81+24；差异见 §7）。

关键回归：

- **D1（机侧）**：`test_d1_udp_src_initialised_before_try` + `test_d1_finally_only_uses_names_bound_before_try`（通用 AST 守卫：finally 引用的局部名必须在 try 前绑定）— `tests/test_dzlink_supervisor.py:810-851`。
- **D2（机侧）**：`test_d2_run_bridge_injects_both_halves` / `test_d2_run_target_and_field_inject` / `test_d2_run_without_intel_args_stays_empty` / `test_d2_run_dry_run_does_not_double_inject` — `:866-912`。
- **t16 成对**：`test_halves_emitted_as_adjacent_pair_in_fixed_order` / `test_late_second_half_pulls_the_first_one_along` / `test_pairing_does_not_spam_within_period` / `test_expired_half_is_not_resurrected_by_pairing` — `:926-968`。
- **rclpy 隔离**：`test_no_rclpy_import_at_module_level` — `:773-775`。

### 6.2 车端测试（`dzrobot_v2/src/dzbrain/tests/`）

共 34 个 `test_*.py`。与本篇直接相关的：

| 测试 | 项数 | 覆盖 |
|---|---|---|
| `test_car_state_report.py` | 4 组断言（无 P_CAR_STATE / 有 / 同 phase 换段 / 非法回退） | **D1（车侧）** 回归 |
| `test_link_e2e.py` | 6 | 链路端到端 |
| `test_drone.py` | （grep 0，断言为脚本式） | v1 报文 + 覆盖语义 + 布局版本 |
| `test_scout.py` | （脚本式） | 侦察模型 |
| `test_bridge_gate.py` | 5 | 桥洞采纳门 |
| `test_mission.py` | 22 | 任务编排 |
| `test_route.py` / `test_tracker.py` / `test_pursuit.py` | 13/15/11 | 路线/循迹/追踪 |

> grep `def test_` 对脚本式断言（`check()/finish()`）计 0，实际断言数更多（见 §7）。

### 6.3 门禁与部署对账

| 命令 | 判据 | 证据 |
|---|---|---|
| `cd dzrobot_v2 && python tools/wf/check.py` | 退出码 0；冻结区脏未登记/过期/无 owner=红；断链=红；复用未登记=红 | `wf/check.py:80-197` |
| `python tools/wf/check_reuse.py` | 新增 .py 未登记=红 | `wf/check_reuse.py:69-84` |
| `python tools/wf/check_links.py` | docs/** 断链=红 | `wf/check_links.py:25-56` |
| `python tools/deploy_to_car.py all` | 推送后 sha256 对账（缺/不一致）非 0 则中止编译 | `deploy_to_car.py:306-328,455-459` |
| `python tools/deploy_to_car.py verify` | dzbrain 包/入口存在 + 云台补丁符号 `use_external_fire_gate` | `deploy_to_car.py:405-420` |
| `node admin.mjs verify`（dsh_plugin） | 6 通过 0 失败 | `dsh_plugin/README.md:12` |
| `python tools/pack_handoff.py --level all` | L0/L1/L2 交接包 + MANIFEST | `pack_handoff.py:242-257` |
| `python memory/dzmem.py stats` | facts/events/prefs/people 计数 | `dzmem.py:115-120` |

### 6.4 证据等级总表

| 组件 | 等级 | 证据 |
|---|---|---|
| v1 情报链路（双向） | **真机** | t4 域 118 双向；t12 修 D1/D2/D4；t16/t18 收口（ack 硬证据） |
| 段链路 field_layout v2 | **真机 1 次** | 实车首跑：known 15 格、target L8 全通过 |
| 段规划 | **真机 1 次** | N10→L8，5 格 / 1.40 m / 9.3 ms |
| StepLoop 段执行 | **实车 0 米 + 10-18 晚 6 段** | 14:50 车一步没动；晚场连续 6 段（含 1 停站） |
| 感知→known（实时） | **仅代码 + fixture** | 无实机分钟级运行 |
| 发射器→车端闭环 | **仅 dry-run** | t3/emitter/dry_*.jsonl |
| 布局→碰撞安全 | **未验证** | 无"错布局下是否撞"用例 |

证据来源：`docs/20_资料索引库/空地链路/02_链路联调与已知缺口.md:18-28`。

---

### 5.5 非链路但关键的配置键（引用，不展开）

| 键 | 当前值 | 语义 | 证据 |
|---|---|---|---|
| `pursuit.max_steer_deg` | 35（底盘 clamp 到 servo∈[25,95]） | 舵角上限 | `contract.py:144-147` |
| `localization.anchor_max_pull_m` | 0.12 | 单次吸附位移上限 | `robot.yaml:866` |
| `localization.drift_slow_after / drift_stop_after` | 3 / 8 | 漂移降速/停车阈值 | `robot.yaml:871-873` |
| `localization.boundary_guard_m / boundary_stop_m` | 0.25 / 0.10 | 边界护栏 | `robot.yaml:875-877` |
| `localization.yaw_trim_deg` | -0.6 | 横向漂移补偿零点 | `robot.yaml:882` |
| `localization.yaw_rate_floor_dps` | 40.0 | 低速航向守卫噪声地板 | `robot.yaml:886` |
| `aim.red_early_leave_s` | 3.0 | 锁定红面持续秒数 → 提前离站 | `robot.yaml:819` |
| `aim.lead_time_s` | 0.27 | 提前量（2026-10-18 实车回标） | `robot.yaml:833` |
| `aim.fire_policy` | moving_lead | 开火策略 | `robot.yaml:834` |
| `aim.px_per_m_rail` | 140.0 | 轨道 px/m（标定值） | `robot.yaml:842` |
| `aim.max_shots_total / per_target` | 10 / 10 | 弹药硬上限 | `robot.yaml:843-846` |
| `aim.shot_cooldown_ms` | 500 | 射击冷却 | `robot.yaml:847` |
| `aim.lost_frames_to_rescan` | 60 | 丢靶重扫帧数 | `robot.yaml:848` |
| `aim.scan_pitch_offset_deg` | +10.0 | 扫描俯仰（t52 现场实测） | `robot.yaml:849-851` |
| `mission.auto_start` | false | 自动出发（步进赛时被 override） | `robot.yaml:337`、`main.py:2082` |
| `fix.anchor_at_waypoint / scale_est_enabled` | true / true | 吸附/尺度估计 | `robot.yaml:854-855` |
| `safety.task_stall_s / startup_grace_s / brake_on_fault` | 1.5 / 8.0 / true | 安全看门狗 | `robot.yaml:895-897` |

---

### 6.5 测试命令清单（可复跑）

| 目的 | 命令 | 判据 |
|---|---|---|
| 机侧 dzlink 单测 | `python -m unittest discover -s dzsmartscout/tools/link/tests -p "test_*.py"` | 全绿（81+24 项） |
| 机侧纯逻辑自检 | `python dzsmartscout/tools/link/dzlink_supervisor.py selftest` | 无 FAIL |
| 机侧契约一页纸 | `python dzsmartscout/tools/link/dzlink_supervisor.py contract` | 含当前域号 118 |
| 机侧 dry-run | `python dzsmartscout/tools/link/dzlink_supervisor.py dry-run --bridge lower=K9 --target G10,H10 --mines A3 --obstacles K9` | 报文字段正确、不发 |
| 车端 DroneTask 回归 | `python src/dzbrain/tests/test_car_state_report.py` | 4 组断言全过（D1） |
| 车端链路端到端 | `python src/dzbrain/tests/test_link_e2e.py` | 6 项全过 |
| 车端 scout 对账 | `python src/dzbrain/tests/test_scout.py` | 检查点序列覆盖 8/8 雷 + 2/2 停站 + 终点 |
| 收口门禁 | `cd dzrobot_v2 && python tools/wf/check.py` | 退出码 0 |
| 文档守卫 | `python docs/_工具/check_doc_policy.py` | 退出码 0（R1/R2/R3 红=0） |
| 断链门禁 | `cd dzrobot_v2 && python tools/wf/check_links.py` | 退出码 0 |
| 复用门禁 | `cd dzrobot_v2 && python tools/wf/check_reuse.py` | 退出码 0 |
| 部署对账 | `cd dzrobot_v2 && python tools/deploy_to_car.py all` | 推送后 sha256 对账 0 不符 |
| 插件自检 | `node D:/User/Desktop/ZhiYuan/dsh_plugin/admin.mjs verify` | 6 通过 0 失败 |
| 插件端到端自测 | `node D:/User/Desktop/ZhiYuan/dsh_plugin/tools/selftest.mjs` | 72 项全过 |
| dzmem 健康 | `cd memory && python dzmem.py stats` | 四表计数 |
| 交接包 | `python tools/pack_handoff.py --level all` | L0/L1/L2 zip + MANIFEST |

> 说明：本文**未实际运行**以上命令（只读审计）。运行结果以现场为准（§7 未核实）。


## §7 未核实清单

| # | 项 | 为什么未核实 | 需要谁/什么才能定 |
|---|---|---|---|
| 1 | `dzrobot_v2/tools/ml/verify_contract.py` 是否存在 | 任务卡点名 `tools/ml`，但 `ls dzrobot_v2/tools/ml` → No such file；全树 `find -name verify_contract.py` 无命中 | 确认 ml 工具是否在别的仓/被删；引用 `docs/00_项目总览/系统架构.md:157` 的命令会直接失败 |
| 2 | `air_ground_models/` 是否存在 | `ls air_ground_models` → No such file；`docs/00_项目总览/系统架构.md:148,157` 引用了它 | 确认模型目录是否迁移/删除 |
| 3 | `plugins.json` 里三个插件的 `source`（`D:/User/Desktop/比赛存档/v1/tools/plugins/...`） | 该目录不存在；实体在 `D:/User/Desktop/ZhiYuan/dsh_plugin/` | 确认 `source` 字段是否为迁移前的历史路径、admin.mjs 是否仍以此为基准 |
| 4 | `docs/20_资料索引库/空地链路/README.md` | `ls docs/20_资料索引库/空地链路/` 只有 00_索引/01/02/03，无 `README.md` | 本文 §2.4.1 引用了它，疑为笔误（应为 `00_索引.md`） |
| 5 | `dzlink_supervisor` 真机 `run` 模式 | 模块 docstring 自述"未在无人机真机上跑过本轮无真机窗口"（`:67-70`）；只有 `smoke_ros.sh` 回环验证 | 需要无人机真机窗口 |
| 6 | D1b / D2 修复的实车复验 | `docs/02:37` 记"代码已提交未实车复验"；`docs/02:46` 记 D2"未修" | 需要实车段链路复跑 |
| 7 | `wf/repo_index.py` 逐行 | 只读到 204 行规模与用途，未逐行精读 | 若需审计目录索引生成逻辑需补读 |
| 8 | `admin.mjs` 逐行 | 只确认 1397 行与命令集，未逐行精读 | 若需审计插件生命周期需补读 |
| 9 | `docs/20_资料索引库/车端/05_配置与部署.md` | grep 无 `dzlink/car_state/SEGMENT_DONE` 命中，未读全文 | 与部署/配置主题交叉核对 |
| 10 | 机侧 dzlink 测试"70 项"vs 实测 81+24 | README 写 70 项，`grep -c "def test_"` 得 81（supervisor）+24（sender） | 统计口径不同（含/不含参数化），需确认 |
| 11 | `config/robot.yaml` 中 `step.layout_ttl_s` / `step.done_replay_s` / `step.odom_watch_s` 是否显式写出 | 只看到 `step.wait_timeout_s`（`robot.yaml:732`）；其余走代码默认 | 现场是否已显式配置需查车端 robot.yaml |
| 12 | `git_save/` 各版本是否含 `.zhiyuan/` 与 `dsh-plugin` | 只抽查了 v1.1.0/v2.1.0 的 `dsh-plugin-dzrobot-ssh` 存在 | 全版本盘点 |
| 13 | `dzsmartscout/tools/link/README.md:12` 的"70 个单测" | 同 #10 | 同 #10 |
| 14 | `docs/20_资料索引库/空地链路/02` 引用的 `_tmp_session/t3/...` 原始文件 | 未逐一核对其存在性（本次只读代码，未深入 `_tmp_session`） | 若要复核 NO-GO 原文需补 |

---

## §8 发现清单（分级）与统计表

> 严重级定义沿用 `docs/00_项目总览/技术全景_F_代码审计报告.md:13-17`。

### 8.1 blocker / high

| ID | 级别 | 发现 | 证据 | 影响 |
|---|---|---|---|---|
| E-B1 | high | **D1b：SEGMENT_DONE 只活 1~2 拍，漏收率 ≈86%**，段末永久 WAIT_INFO（代码已加闩锁+1Hz 重发，未实车复验） | `main.py:646-657,1048-1066`；`docs/02:30-37` | 段链路可能永久卡住 |
| E-B2 | high | **D2：到站 block 差半格/一格，机侧 `block==target` 判据永久卡住**（未修） | `mission.py:2857-2897,3199-3228`；`docs/02:38-46` | 同上 |
| E-B3 | high | **known 无"空域"来源** ⇒ 可通行格≈特征格，段规划恒无解 | `docs/02:53-57` | v2 步进循环 NO-GO 根因 |
| E-B4 | high | **段规划完全不过门禁**（无 check_planned_route、无物理净空） | `main.py:519-576` | 错布局下安全性未验证 |
| E-B5 | high | **布局无新鲜度门**（缺口 E 已补 `step.layout_ttl_s`，需确认车端已显式配置） | `main.py:633-644,803-833` | 30 s 前的布局与刚采的不可区分 |
| E-H1 | high | 车侧 `/nav/waypoint` 有**第二写者**（车上 start_path_executor + send_waypoint），与机侧 supervisor 同跑会跳路线 | `docs/01:123` | 路线来源竞态 |
| E-H2 | high | 车侧决策核↔底盘的显式停车/使能信号**全部悬空**（StopCmd/logicontrol/laser_switch/vehicle_state/vehicle_Drivestate/cmd_vel 无生产者），唯一兜底是底盘 1.0 s 看门狗 | `docs/03:52-67` | 决策核无法主动急停/激光使能握手 |
| E-H3 | high | 机侧 `_run_ros` 的 `listen` 分支在 try 内 return，`finally` 引用未绑定局部名（D1） | `dzlink_supervisor.py:1753-1756` | 已修 + AST 回归，但新增同类写法会复发（有通用守卫） |

### 8.2 medium / low

| ID | 级别 | 发现 | 证据 |
|---|---|---|---|
| E-M1 | medium | `run` 子命令声明了 --bridge/--target/--mines/--obstacles 但原先不生效（D2） | `dzlink_supervisor.py:1936-1940` | 已修 + 回归 |
| E-M2 | medium | 桥洞两半场分槽前的真 bug：后到的 upper 覆盖 lower | `dzlink_supervisor.py:535-545` | 已修 + 回归 |
| E-M3 | medium | 插件 `source` 指向不存在的 `v1/tools/plugins` | `plugins.json:37,46,55` | 迁移后字段陈旧 |
| E-M4 | medium | 文档引用了不存在的 `air_ground_models/` 与 `tools/ml/verify_contract.py` | `docs/00_项目总览/系统架构.md:148,157` | 命令会失败 |
| E-L1 | low | `deploy_car.py` 旧部署入口与 `deploy_to_car.py`并存 | `dzrobot_v2/tools/` | 易误用 |
| E-L2 | low | `dzsmartscout/tools/link/README.md` 写"70 个单测"，实测 81+24 | `README.md:12` | 统计口径 |
| E-L3 | low | `docs/20_资料索引库/空地链路/README.md` 被本文引用但不存在 | §7 #4 | 疑为笔误 |
| E-L4 | low | `V2.0/` 仅 .git，无工作树 | §2.7.8 | 需确认用途 |

### 8.3 统计

| 项 | 数 |
|---|---|
| 本次精读文件 | 22 |
| 函数/类索引条目 | 约 260 |
| 配置键 | 约 30 |
| blocker/high | 8 |
| medium/low | 8 |
| 未核实 | 14 |

---

## 相关文件

- `docs/20_资料索引库/空地链路/00_索引.md`、`01_协议v2与契约.md`、`02_链路联调与已知缺口.md`、`03_车侧话题对账_20261018.md`
- `docs/00_项目总览/系统架构.md`、`现状快照.md`、`技术全景_F_代码审计报告.md`
- `docs/20_资料索引库/车端/05_配置与部署.md`、`06_回归与测试基线.md`
- `docs/20_资料索引库/无人机/04_路线下发与轨迹回放.md`、`02_飞控与uXRCE域号.md`
- `dzrobot_v2/docs/step_loop.md`、`dzrobot_v2/docs/决策台账.md`

---


---

## §2 附录 A：前置真源（地图 / 域号 / 环境）

### A.1 地图真源 `config/map.yaml`（41 行）

地图要素与 robot.yaml **分开存放**：ROS params 文件顶层只允许 `<节点名>: ros__parameters:` 结构，多一个顶层键会让 `--params-file` 解析失败，相机/视觉/云台三节点同时 abort（`map.yaml:1-5`）。

文件字段（`map.yaml:24-41`）：

| 字段 | 当前值（第六图） | 语义 |
|---|---|---|
| `map.name` | `第六张地图 · 2026-10-17 用户现场图（逐格识读）` | 图名 |
| `map.start / end` | `N10 / N5` | 车起终点 |
| `barrier_col_from` | `D` | 隔板起始列 |
| `barrier_row_hi / lo` | `6 / 7` | 隔板两侧行号 |
| `mines` | `A2 F4 H4 G5 E8 B10 D11 I11` | 雷 8 |
| `obstacles` | `B1 D1 F3 I6 H7 A8 C9 E9` | 障 8 |
| `targets` | `D5 D6` | 靶（跨格靶板） |
| `bridges` | `K5 K7` | 桥洞 2 |

★ 与第七张图的关系：第七图（雷 H8 G9 A2 B1 C3 E3 F5 I2；障 D5 I6 B7 F8 C9 E10 G10 A11；靶 D7+E7；桥 K10/K4；起 N10 终 N5）是 10-21 离线新线，**未落盘/未部署**（`docs/01:131`；`memory/seed.py:24`）。机端真值钉 `~/dzlink_tools/map7.yaml`（`docs/02:69`）。

★ 赛道模型：整个 K 列是一座桥，每格是横向贯通的桥洞、格间有侧壁；挡板在 6/7 行之间 D~N 列封死 ⇒ 小车只能走 U 形（`map.yaml:28-32`）。

### A.2 环境真源

**车侧 `dzrobot_v2/dzros.env`**（19 行）：`DZ_ROS_DOMAIN="${DZ_ROS_DOMAIN:-118}"`、`export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$DZ_ROS_DOMAIN}"`、`export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"`（`:16-18`）。注释明确：交互式终端起栈会读 ~/.bashrc → 118；systemd/cron/ssh 一行命令不读 → 默认 0（`:8-11`）。外部显式 export 优先，启动横幅报警（`:14`）。

**机侧 `dzsmartscout/tools/link/dzlink.env`**（16 行）：`export ROS_DOMAIN_ID=118`、`export ROS_LOCALHOST_ONLY=0`（`:15-16`）。2026-10-07 联调事故记在注释里：车在域 118、机默认 0 → ping/SSH/网页台全通但 ROS 互相看不见且不报错（`:7-11`）。

**部署脚本 `dzsmartscout/tools/link/deploy_supervisor.sh`** 与 `smoke_ros.sh`、`verify_bidir.sh`、`recon.sh`、`beacon.sh`、`car_probe_v1.sh`、`relaunch_dzlink.sh`（机侧脚本族）。

---

## §2 附录 B：工程设施补充

### B.1 车端工具细节

**`dzrobot_v2/tools/deploy_car.py`（281 行）**：旧部署工具。与 `deploy_to_car.py` 的区别：`deploy_to_car.py` 是 2026-10 后定版的唯一入口（带 sha256 对账、tar 流就地替换、不推 map.yaml、verify 可选）。**两者并存是 E-L1 低危发现**，误用 `deploy_car.py` 会绕过对账闸门。

**`dzrobot_v2/tools/healthcheck.sh`（22044 B）**：出车体检。包含新核心 dzbrain 章节（双钥匙开火门控 / 裁判回传 / 保活速率）。★ 部署清单里**必须推**它（否则车上一直跑旧版，章节号仍是 N/6，出车前"假全绿"，`deploy_to_car.py:68-73`）。推之前要确认本地版已并回车端适配（源码撤离后跳过 declare/get 配对、模型路径优先 install、ROS 域号取自 dzros.env）。

**`dzrobot_v2/tools/config_selfcheck.py`（3986 B）**：配置自检。原属 `verify` 的"配置+路线门禁 dry-run"已随自检简化移除（`deploy_to_car.py:405-408`）。

**`dzrobot_v2/tools/plan_route.py` + `route_check.py`**：路线规划与检查。门禁口径：`python tools/plan_route.py --emit tokens --engine <config 里那个> --check`，**0 error 才放行**（`docs/01:122`）。

### B.2 `docs/_工具/`（文档治理工具）

AGENTS.md §8 规定的文档纪律工具，本文 §2.7.10 已列命令，这里补文件职责：

| 文件 | 职责 |
|---|---|
| `docs/_工具/gen_index.py` | 重新生成 `docs/目录索引.md`（读 front matter + TL;DR）；支持 `--ns <ns>` 增量只扫本 ns |
| `docs/_工具/check_doc_policy.py` | 文档守卫：R1 库外新增 .md / R2 docs 下开新库 / R3 库内写了没登记 |
| `docs/_工具/doc_policy.json` + `doc_policy_baseline.json` | 白名单/基线（唯一真源） |
| `docs/_工具/snapshot_docs.py` | 文档快照（zip 落 `_archive/docs_snapshots/`，保留最近 12 个） |

★ **本子智能体的写域**：唯一写入目标是本文；索引更新由 Lead 收口时执行（见文末"文档变更"）。

### B.3 DSH 插件细节

**`admin.mjs`（1397 行）** 命令集（`dsh_plugin/README.md:8-18`）：

    node admin.mjs list                          # 看有谁、健康否
    node admin.mjs verify                        # 门禁：6 通过 0 失败
    node admin.mjs enable|disable <包名>          # 启停（只动 bundles）
    node admin.mjs remove <包名> --dry-run        # 先看要做什么
    node admin.mjs remove <包名>                  # 真卸载：目录进 _archive
    node admin.mjs install <目录|tgz>             # 装：拷到本目录 + 建 junction + 登记
    node admin.mjs repair                         # 断链/错向时重建 junction

配套：`tools/selftest.mjs`（478 行，72 项端到端自测）、`tools/dsh-file-dep-watchdog.ps1`（39 行，每 5 分钟 `admin repair`，健康时不写日志）、`tools/finalize.md`（何时才需要"归一化"）、`INVENTORY.md`（227 行，迁移前依赖/加载路径盘点）、`MIGRATION.md`（143 行，迁移记录）。

**迁移结论（2026-10-23 21:20）**：6 条本地依赖协议从 `file:` 换成 `link:`。`file:` 会被 pnpm 打包导入，把 D 盘目录搬进 node_modules，**D 盘目录被搬空**（21:08 实测砸了 3 个插件）；`link:` 只建符号链接（`README.md:20-40`）。两轮实测：`pnpm install` 21.1 s 退出码 0、6 目录文件数与 index.js 哈希零变化；`pnpm add dshmarket` 1.6 s 退出码 0；6/6 manifest 逐文件 sha256 PASS。

备份：`_backup/P0-20261023122519/`（6 份 manifest + 6 包完整副本 + profile 配置）、`C:/Users/TestFox/.dsh/_archive-live-pre-migration-20261023/`（迁移前 node_modules 原件）。

### B.4 `git_save/` 版本映射细节

18 个 dzrobot_v2 标签 + 2 个 V1 根仓标签，共 20 个文件夹。版本号有三次重整（`git_save/README.md:8-22`）：

- v3.1.0 = 原 v2.5（空地指挥链路闭环）；v3.2.0 = 原 v2.6（空地协同车侧三卡落地）；v4.0.0 = air-ground 空地协同架构基线。
- V1 根仓 v4.0.0（2cce6c7）→ 改记 **v4.0.1**；V1 根仓 v4.0.1（5ec96a1）→ 改记 **v4.0.2**。
- 2026-10-23 16:32 物理改号落地，旧标签对象备查 `3dd6e4f`、`b4cfb18`。

完整性：18 个文件夹 `tar -d` 全量比对，仅 Windows uid/gid/mode 元数据噪声，**内容 0 差异**。

### B.5 `.zhiyuan` 状态

- `state.json`：`last_recovery=2026-10-16T16:25:05`、`recoveries=1`（即 watchdog 至今只恢复过一次）。
- `heartbeat.txt`：`2026-10-16 18:49:06`（带 BOM）。
- `poll.log` / `watchdog.log` / `工作日志.md.bak-gbk`（历史 GBK 备份）。

★ 注意：主守护 2026-10-16 被静默 Disabled 过一次（poll.ps1:3-5 记录），这是第二层守护存在的唯一理由。



---

## §4 附录 C：StepLoop 状态机迁移表（逐条判据）

| 当前 phase | 触发条件 | 下一 phase | 证据 |
|---|---|---|---|
| `WAIT_LAYOUT` | 首份布局到达（或启动门给了） | `PLANNING` | `main.py:859-860,721-745` |
| `PLANNING` | `_plan` 成功下发段 | `DRIVING` | `main.py:940-952` |
| `PLANNING` | 无 target_cell / 无位姿 / 规划失败 / 布局过期 | `WAIT_INFO` | `main.py:869-923` |
| `PLANNING` | 同格停站 / single_cell | `SEGMENT_DONE` | `main.py:888-899,927-929` |
| `DRIVING` | MissionTask 回报 done 且 segment_id 匹配 | `SEGMENT_DONE` | `main.py:963-968` |
| `DRIVING` | 段执行失败（err） | `WAIT_INFO` | `main.py:960-961` |
| `DRIVING` | 连续 3 拍未知格 | `SAFE_STOP` | `main.py:969-970,1207-1218` |
| `DRIVING` | 任务线程回 IDLE 且 >1 s | `WAIT_INFO` | `main.py:972-976` |
| `DRIVING` | 位姿断流超 `step.odom_watch_s` | `ROS_LOST` | `main.py:956-957,1102-1143` |
| `SEGMENT_DONE` | 完成格 == layout.end | `FINISH` | `main.py:978-983` |
| `SEGMENT_DONE` | 段边界新布局到达（seen > planned） | `PLANNING` | `main.py:984-990` |
| `SEGMENT_DONE` | 无新布局 | `WAIT_INFO` | `main.py:991` |
| `WAIT_INFO` | 新布局到达（seen > planned） | `PLANNING` | `main.py:994-997` |
| `WAIT_INFO` | waited ≥ `step.wait_timeout_s`（>0） | `WAIT_INFO_TIMEOUT` | `main.py:998-1004` |
| `FINISH` | 终止 | — | `main.py:1088-1097` |

### 4.7.1 完成单（done latch）生命周期

    _mark_segment_done:  _done_seq += 1; _done_latch = {segment_id, block, stop_kind, skipped, elapsed_s, seq, replay:0}; _done_replay_stopped=False
    _maybe_replay_done:  WAIT_INFO 期间每 done_replay_s（默认 1s）重发一次，latch["replay"] += 1
    _stop_done_replay:   新布局到达 / 开新段 / FINISH / SAFE_STOP / ROS_LOST 时置 True 并清 _done_latch

证据：`main.py:1017-1073`。**关键**：phase 照旧推进，完成单作为独立字段组随帧携带（`main.py:1177-1180`），机端漏收 SEGMENT_DONE 也能从后续任意一帧拿到 segment_id/block/stop_kind/done_seq。

## §4 附录 D：关键函数伪码（逐行读代码）

### 4.8.1 `DroneTask._on_field_layout`（`drone.py:842-889`）

    def _on_field_layout(self, payload, now):
        layout, reason = normalize_field_layout(payload)      # 纯内存校验，非法 → (None, reason)
        if layout is None:
            self.layout_rejects += 1; log(...); return        # 整份忽略，保留上一份
        new_version = int(layout.get("version") or 0)
        if self.field_layout is not None:
            cur_version = int(self.field_layout.get("version") or 0)
            if new_version < cur_version:                     # 迟到旧报
                self.layout_stale += 1; log(...); return
            if new_version == cur_version:                     # 同版必须同内容（除 ts）
                same = ({k:v for k,v in layout.items() if k != "ts"} ==
                        {k:v for k,v in self.field_layout.items() if k != "ts"})
                if not same: self.layout_rejects += 1; return
                log("心跳重发（同版同内容，忽略）"); return
        if new_version > FIELD_LAYOUT_VERSION: log("高于支持，只按已知字段解析")
        self.field_layout = layout; self.layout_updates += 1; self.layout_ts = now
        self.bb.set(P_FIELD_LAYOUT, layout, source=self.name, ts=now)

### 4.8.2 `DroneTask._diff` 两条口径（`drone.py:965-1026`）

    # 口径 1：无 coverage（旧机侧）
    if got == local: log("核对一致"); return
    only_remote = sorted(got - local); only_local = sorted(local - got)
    publish(EV_FIELD_VERIFY, remote=got, local=local, only_remote, only_local)

    # 口径 2：有 coverage（t5a 片级报文）
    scope = frozenset(coverage.cells)
    local_scoped = local & scope
    only_remote = sorted(got - local)
    only_local  = sorted(local_scoped - got) if coverage.complete else []   # ★ complete=False 不得报消失
    out_of_scope = len(local - local_scoped)
    publish(EV_FIELD_VERIFY, ..., scoped=True, complete=coverage.complete, out_of_scope_local=out_of_scope)

### 4.8.3 `StepLoop._plan` 新鲜度门（`main.py:866-952`）

    if not self._target_cell: self._to_wait_info("布局未给 target_cell"); return
    from_cell = self._current_block(prefer_prev=True)
    if not from_cell: self._to_wait_info("没有位姿"); return
    if from_cell == self._target_cell:
        self._stop_kind = classify_segment_target(...)          # 空段也要如实报 stop_kind
        self._segments_started += 1; self._mark_segment_done(..., skipped=True); return
    if self._layout_is_stale(now):                              # 缺口 E
        if self._layout_is_complete(layout):                   # D-022 例外：完整布局=静态信息
            log("过期但完整，按 D-022 允许规划（只说明一次）")
        else: self._to_wait_info(self._layout_stale_reason(now)); return
    self._replans += 1
    plan = self._planner(layout, from_cell, self._target_cell, self._heading_dir())
    if not plan.ok: self._to_wait_info(f"段规划失败：{plan.error}"); return
    self._stop_kind = str(getattr(plan, "stop_kind", "") or "")
    if plan.single_cell: self._mark_segment_done(...); return
    res = self.mission.request_segment(plan.tokens, segment_id=..., target_cell=..., stop_kind=...)
    if not res.get("accepted"): self._planned_version = -1; self._to_wait_info(..., retry=True); return
    self._segments_started += 1; self._phase = STEP_PHASE_DRIVING

### 4.8.4 `SupervisorCore.tick`（`dzlink_supervisor.py:1131-1194`）

    for slot in self._stale_slots(now): log("intel_stale")          # ⑤ 过期撤回
    if self._routes: ev = self._routes.popleft(); out.append(ev)   # ④ 路线优先
    if self._next_hb == 0.0 or now >= self._next_hb:                # ① 心跳
        seq = self.next_seq(); text = canonical_text("heartbeat", {}, seq, ts=now)
        self._next_hb = now + self.cfg.heartbeat_period_s
        self.ledger.note_sent(seq, {"type":"heartbeat"}, now)
        out.append(OutEvent(topic=TOPIC_TO_CAR, qos=QOS_LINK, text=text, ...))
    fresh_bridge = [s for s,r in self.records.items() if r.etype=="bridge_passable" and r.age(now) <= ttl]
    bridge_round_due = any(now >= self._next_intel.get(s,0.0) for s in fresh_bridge)
    for slot in sorted(self.records, key=slot_sort_key):             # ② 情报（按槽位）
        due = self._next_intel.get(slot, 0.0)
        if now < due and not (bridge_round_due and rec.etype=="bridge_passable"): continue
        ev = self._make_intel_event(slot, now); ...
        period = self.cfg.period_for(rec.etype)
        self._next_intel[slot] = (now + period) if period > 0 else float("inf")

### 4.8.5 `normalize_field_layout` 校验顺序（`drone.py:181-294`）

    ① payload 是 dict            否则拒
    ② version 是 int 且 >= 0     否则拒
    ③ ts 坏值不拒（置 0.0）
    ④ start/end 是合法格名        否则拒
    ⑤ "known" in payload → 校验（v2 至少 1 格）；否则 known_set = 全图（v1 语义，不写回 known 键）
    ⑥ mines/obstacles/targets 各自校验 + 必须 ⊆ known
    ⑦ bridges 非空
    ⑧ barrier_col_from 合法列；barrier_row_hi/lo 1..11 且 hi < lo
    ⑨ target_cell 空或合法且 ∈ known
    ⑩ segment_id 非负整数



## 文档变更（本卡）

- **新增**：本篇 `docs/00_项目总览/技术全景_E_空地链路与工程设施.md`（唯一写入目标）。
- **未修改**任何源码/配置/测试；未新建其它 .md（遵守文档守卫 R1）。
- **索引**：按 §8.4 要求需在 `docs/00_项目总览/` 的册级索引加一行并重跑 `gen_index.py`——**本子智能体只读代码、唯一写域为本文档，索引更新由 Lead 收口时执行**（本卡写域不含 `docs/00_项目总览/` 索引与 `docs/目录索引.*`）。

<!-- AUDITOR-SKEL -->