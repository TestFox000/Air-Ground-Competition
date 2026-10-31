---
来源: dzrobot_v2/src/dzbrain/dzbrain/**（io/contract.py、io/ros_io.py、core/blackboard.py、core/task.py、core/events.py、tasks/aim.py、tasks/safety.py、tasks/tracker.py、tasks/mission.py、tasks/localization.py、tasks/telemetry.py、main.py）；dzrobot_v2/start_all.sh；dzrobot_v2/tools/{wf/check.py, config_selfcheck.py, car_reconcile.py, route_check.py, healthcheck.sh, deploy_to_car.py}
取证: 实
status: 现行
更新: 2026-10-30
取代: 无（新建；07_发车门禁口径_G1-G4 讲"发车前四门怎么验"，09_明早全流程SOP 讲"这一趟按什么顺序做"，本页讲"断了之后从现象往回定位"）
摘要: dzbrain 链路诊断手册：5 类现场症状各自连到候选原因 + 可复跑命令 + 预期输出 + 判据；遥测字段判读；离线可跑命令已本机实跑标〔实〕，现场命令标〔码〕。
---

# dzbrain 链路诊断手册

> **适用前提**：dzbrain 是决策脑，**不直接驱动电机**。它算完"下一步干什么"发到 `pursuitAngle`，底盘节点去执行（`src/dzbrain/dzbrain/io/contract.py`）。所以"车不动"可能发生在 dzbrain 这侧（没算出来）或底盘那侧（没执行），**先分清哪一侧**再查。
> **一条铁律**：GREEN ≠ 回归绿 ≠ 实车通过（`dzrobot_v2/AGENTS.md` §5）。本页所有判据绿了只证明"这一层没查出事"，不证明车会跑对。

---

## 0. 一页纸速查（现场先跑这 6 条）

| 你看到 | 先跑（现场） | 看什么 |
|---|---|---|
| 车完全不动 | `ros2 topic echo /dzlink/car_to_drone --once` | `phase` 是不是停在 `WAIT_INFO`？`safety_ok` 是不是 `false`？〔码〕 |
| 车在跑但跑偏 | `ros2 topic hz /pursuitAngle` | 有没有发布？`steer_deg` 是不是异常？〔码〕 |
| 停不下来 | `ros2 topic echo /aim/fire_gate --once` | `SAFE_STOP` 是否已触发（safety_reasons）？〔码〕 |
| 打不出去 | `ros2 topic echo /aim/fire_ready --once` | 两键（fire_gate + fire_ready）是否**同时**为 1？〔码〕 |
| 两边互看不见 | `ros2 topic list`（车）/（机） | 话题集是否对称？`ROS_DOMAIN_ID` 是否一致（`~/dzrobot/dzros.env` 真源）？〔码〕 |
| 起栈就崩 | `bash ~/dzrobot/start_all.sh` | 退出码：`3`=无布局 / `4`=布局规划失败 / `5`=ROS 或位姿未就绪 / `6`=启动契约不通过 / `0`=正常（`main.py` 退出码表）〔码〕 |

> 现场命令未实跑（车/机不在线），预期输出按代码打印格式推，标〔码〕。**第一次现场用请用真实输出替换〔码〕**。

---

## 1. 诊断金字塔：现象 → 原因 → 命令 → 判据

现场看到症状后，**按这个顺序一层层往下问**，每层一条命令判死一个原因。

### 1.1 第一层：先分"哪一侧"

| 命令 | 判什么 | 判据 |
|---|---|---|
| `ros2 topic hz /pursuitAngle` | dzbrain 是否在发驱动 | `hz≈50` = dzbrain 在跑；`hz=0` = **决策侧问题**，跳 1.2 |
| `ros2 topic echo /pursuitAngle --once` | 发的值是否合理 | 速度/舵角都在正常范围（`pursuit.cruise_speed_mps` 量级）；速度=0 且非停站 = **no-op 契约问题**，见 §2.3〔码〕 |
| 底盘日志 / `hz /cmd_vel` | 底盘是否收到并执行 | 收到 ≠ 执行（有延迟/被看门狗清零，`dzrobot_v2/AGENTS.md` §6 部署≠生效）〔码〕 |

**决策**：
- dzbrain 没发（`hz=0`）→ 查 §1.2「决策侧不产驱动」
- dzbrain 发了但车不动 → 查底盘侧（本页不覆盖，见 `07_发车门禁口径_G1-G4.md` G3）

### 1.2 第二层：决策侧为什么不产驱动

按 `telemetry.py:96-163` 的 `build_telemetry` 字段顺序判读，一屏遥测就能定位：

| 遥测字段 | 正常 | 异常指向 |
|---|---|---|
| `phase` | `EXECUTING` | `WAIT_INFO`=缺情报（机不在线或过期）；`PLANNING`=规划器在算；`IDLE`=等布局/未发车 |
| `safety_ok` | `true` | `false` = SAFE_STOP 或安全层拦停（看 `extra.safety_reasons`） |
| `drone` | `online` 或 `offline` | `online:lower=K9`=有桥洞情报；`online!过期`=情报超 30 s（`_drone_text` `telemetry.py:184-200`） |
| `seg_index` / `s` / `route_len` | 递进 | `s` 卡住不动 = 段完成判据问题（`P2/T1` 已挂账，`08_回归守护_known_reds.md`）；`route_len=0` = 无路线 |
| `aim` | `IDLE` | 其它值 = 自瞄状态（`AimStatus.name`） |
| `shots` / `hits` | 随打靶增长 | 不变 = 没打出去或没判中 |

**判据**：`phase=EXECUTING ∧ safety_ok=true ∧ drone.online ∧ s 在递进` = 决策侧在正常产驱动。

---

## 2. 六个结构性隐患的现场表现与判定

> 六隐患来自 `archrev-1028/ARCH-RESEARCH-20261028.md`（AR-1），带源码定位。本页把每个隐患变成一条**现场可判**的条目。

### 2.1 【单点】Tracker 是唯一驱动写者，静默不 tick 无人发现

**现场表现**：车不动，但 dzbrain 进程还在、日志还在打。

**代码定位**：`pursuitAngle` 的发布唯一入口 `ros_io.py:499` `publish_command()`；`blackboard.py:68` 声明 Tracker 是驱动唯一写者。

**判定命令**：
```bash
ros2 topic hz /pursuitAngle              # 〔码〕预期 hz≈50
ros2 topic echo /dzlink/car_to_drone --once   # 〔码〕看 steer_deg 是否随指令变
```

**判据**：`hz=0` 或 `steer_deg` 恒 0 而 `Mission.phase=EXECUTING` ⇒ **高度怀疑 Tracker 静默不 tick**（非 SAFE_STOP，因为 safety_ok=true）。

**现场处置**：停栈重启（`bash start_all.sh stop && start_all.sh`）〔码〕。**根修**见 P1（Tracker 心跳探针），需冻结区登记。

### 2.2 【跨进程】两键开火、车端无法自证

**现场表现**：打了没中 / 打不出去 / 误开火。

**代码定位**：`aim.py:11` 区分 `fire_gate`（Mission 授权）与 `fire_ready`（Aim 视觉就绪）；**最终汇合点在云台 C++ 补丁**，dzbrain 看不到（`ARCH-RESEARCH` §外部一手）。

**判定命令**：
```bash
ros2 topic echo /aim/fire_gate --once    # 〔码〕预期 bool
ros2 topic echo /aim/fire_ready --once   # 〔码〕预期 bool
```

**判据**：**两键必须同时为 1 才开火**。只一键=1 ⇒ 打不出去是设计行为不是 bug。

**现场处置**：若两键都=1 但没打出去，是**车端无法自证的盲区**（P0-2 契约测试正在补，见 P0-1 子任务）。

### 2.3 【注释契约】no-op 契约只写在注释里

**现场表现**：车莫名不跑，速度=0，但不是停站/SAFE_STOP。

**代码定位**：`blackboard.py:88-89` 注释 "0.0 必须真给 0"；"有数据才生效，没数据就当没这回事"（红面早退键范本）。

**判定命令**：
```bash
ros2 topic echo /dzlink/car_to_drone --once   # 〔码〕看 speed / safety_ok / phase
```

**判据**：`speed=0 ∧ safety_ok=true ∧ phase=EXECUTING` ⇒ **no-op 契约被误触发**（某键被给了 None 或 0 而语义上"没数据"应早退）。

**现场处置**：查黑板日志（`-D` 或 verbose 模式打关键键值）。**根修**见 P1（黑板运行时断言，默认关）。

### 2.4 【单文件】Mission 单文件 4268 行/146 函数

**现场表现**：改一处段执行/停站/规划，容易波及另一处，回归发现晚。

**代码定位**：`tasks/mission.py`（146 函数，`main.py:2531-2540` 装配）。

**判定命令**（**离线**）：
```bash
python src/dzbrain/tests/test_mission.py   # 〔实〕预期：N/N 全部通过
python _run_full_regression.py             # 〔实〕全量套件汇总
```

**判据**：改 mission 相关后，`test_mission.py` 必须全绿且 `_run_full_regression.py` 无新增红。**单元绿不算过**（`dzrobot_v2/AGENTS.md` §6 判据 1），还要跑 `sim_cli` 仿真闭环。

### 2.5 【双向陈旧】config 与测试的真源不一致

**现场表现**：`config_selfcheck` 报 error 但代码看着对；测试钉在旧夹具。

**代码定位**：`08_回归守护_known_reds.md` K1~K9（2026-10-29 重建）；K10/K11 已挂账。

**判定命令**：
```bash
python tools/config_selfcheck.py --config config/robot.yaml --map config/map.yaml
# 〔实〕（不带 --map）预期：结论：error 0 / warn 0 ✅ 通过
# 〔实〕带图：走 G1 完整空间校验（见 07 号 G1 节）
```

**判据**：`error 0 ∧ warn 0`（**不是只看退出码**，`dzrobot_v2/AGENTS.md` §6 判据 2）。

**纪律**：K1~K11 是**已知挂账**，不重开、不当作新问题重查；处置读 `08_回归守护_known_reds.md`。

### 2.6 【增量叠加】no-op 补丁越来越多，行为难预测

**现场表现**：某特性关了以后行为反而变了；A/B 对拍找不出差异源。

**代码定位**：多个任务各带独立 no-op 契约（`blackboard.py` 多键）。

**判定命令**（**离线**，A/B 对拍）：
```bash
python -m dzbrain.tools.sim_cli --pretty --set <key>=<default>  > base.txt
python -m dzbrain.tools.sim_cli --pretty --set <key>=<new>      > cand.txt
diff base.txt cand.txt
# 〔实〕默认路线能跑通，exit 0，产 traj.csv / traj.svg / by_segment.csv
```

**判据**：`diff` 为空 = 该键开关不影响行为（好，说明 no-op 早退生效）；`diff` 非空但语义相同 = 预期；`diff` 出现异常（速度/舵角突变）= 契约被破坏。

---

## 3. 命令索引

### 3.1 【离线可跑】本机就能判，不连车不连机

| 命令（在 `dzrobot_v2` 目录下） | 测什么 | 2026-10-30 实测 |
|---|---|---|
| `python tools/wf/check.py` | 工作流门禁（冻结区/断链/复用） | 〔实〕`结果: GREEN` exit 0 |
| `python tools/wf/check_links.py` | 文档断链 | 〔实〕broken 0 |
| `python src/dzbrain/tests/test_contract.py` | ROS 契约表 | 〔实〕`56/56 全部通过` |
| `python src/dzbrain/tests/test_tracker.py` | 循迹/驱动 | 〔实〕`139/139` |
| `python src/dzbrain/tests/test_localization.py` | 定位 | 〔实〕`175/175` |
| `python _run_full_regression.py` | 全量回归汇总 | 〔未跑〕按 `_run_full_regression.py` 落 `_tmp_full_regression/*.log` |
| `python tools/config_selfcheck.py --config config/robot.yaml --map config/map.yaml` | 配置+空间校验 | 〔实〕（不带 --map）`结论：error 0 / warn 0 ✅ 通过` |
| `python tools/car_reconcile.py --offline` | 三方对账（车离线） | 〔实〕输出 111 项「待同步，缺首次 `--record`」；**exit 0 不代表失败** |
| `python tools/route_check.py --config config/robot.yaml --map config/map.yaml` | 路线 vs 图校验 | 〔实〕`--help` 可用 |
| `python -m dzbrain.tools.sim_cli --pretty --max-steps 3000`（需 `PYTHONPATH=src/dzbrain`） | 仿真闭环 | 〔实〕exit 0，产 3 文件 |

### 3.2 【现场必跑】连车（`duzhong.local` / 用户报告的 IP）

| 命令 | 测什么 |
|---|---|
| `ssh duzhong@<ip> 'cat ~/dzrobot/dzros.env'` | 域号真源（非交互 SSH 不 source ~/.bashrc ⇒ 域 0） |
| `ssh duzhong@<ip> 'bash ~/dzrobot/start_all.sh'` | 七步起栈 |
| `ssh duzhong@<ip> 'bash ~/dzrobot/tools/healthcheck.sh --pre'` | 车上静态体检（0=过 /1=❌ /2=⚠️） |
| `ros2 topic list` | 话题图是否对称 |
| `ros2 topic hz /pursuitAngle` | dzbrain 驱动频率 |
| `ros2 topic echo /dzlink/car_to_drone --once` | 遥测一屏 |
| `ros2 topic echo /aim/fire_gate --once` / `/aim/fire_ready --once` | 两键开火 |

### 3.3 【现场必跑】连机（`dzsmart.local`）

| 命令 | 测什么 |
|---|---|
| `ssh dzsmart@<ip> 'bash ~/dzsmartscout/start.sh'` | 机端起栈 |
| `ros2 topic list` | 双方话题集 |
| 侦察布局发车：`bridge`/`field`/`heartbeat` 通道（见 `fc-audit/fixes/05-drone-to-car-waypoint/APPLY.md` 已交付的发送器脚本） | 空域链路 |

> 若机端离线（mDNS 解析失败），**空域通道 = 第一优先级**（用户裁决：不拉回仓库地图，每局现场侦察作唯一真源）。空域不通 = 车只能 `WAIT_INFO` 或走默认兜底。

---

## 4. 判读表

### 4.1 遥测一屏字段（`telemetry.py:96-163` `build_telemetry`）

| 字段 | 含义 | 关键判读 |
|---|---|---|
| `phase` | Mission 阶段 | `EXECUTING`=正常；`WAIT_INFO`=缺情报；`PLANNING`=在规划 |
| `block` / `x,y` / `yaw_deg` | 当前格 / 坐标 / 航向 | 单位：格 / m / 度（`contract.py` 单位契约） |
| `speed` / `steer_deg` | Tracker 当前下发的速度 / 舵角 | 唯一驱动写者 Tracker 的快照 |
| `seg_index` / `s` / `route_len` | 当前段号 / 沿路里程 / 路线长度 | `s` 卡住 = 段完成判据问题 |
| `aim` / `target` / `shots` / `hits` / `ammo` | 自瞄状态 / 靶摘要 / 已打 / 命中 / 弹药 | `target` 格式：`none` / `seen` / `s<rail>v<vel>` |
| `safety_ok` | 安全层是否放行 | `false` = 拦停，看 `extra.safety_reasons` |
| `drone` | 机端状态 | `offline` / `online` / `online:lower=K9` / `!过期` / `!差异N` |
| `extra.stop_point` | 停站目标 | `index/block/kind/phase` |
| `extra.fix_est` | 纠偏估计 | `scale/samples/max_drift_m` |
| `extra.safety_reasons` | 安全拦停原因 | 列表，逐条可判 |

### 4.2 退出码（`main.py` 启动契约）

| 退出码 | 含义 | 处置 |
|---|---|---|
| `0` | 正常（FINISH） | 正常收工 |
| `3` | 无布局 或 装配失败 | 布局门（最常见）：机端发布局 / 本地发射器补发 |
| `4` | 布局规划失败 | 看规划器日志（`route/planner.py`） |
| `5` | ROS 或位姿管线未就绪 | 等 `/odom/localization` 发布（见 G4） |
| `6` | 启动契约不通过 | 逐项看契约日志（required 项） |
| `1` | SAFE_STOP 触发 | 人工解除后重跑 |

### 4.3 已知挂账（不重开）

`K1~K9` 见 `08_回归守护_known_reds.md`（2026-10-29 重建）；`K10/K11`（config 与测试真源）已挂账，处置见 P0-1 契约表。

---

## 5. 边界：哪些不能离线判、必须现场

| 症状 | 为什么不能离线判 | 必须现场看什么 |
|---|---|---|
| 底盘真执行没执行 | dzbrain 只发 `pursuitAngle`，执行在底盘 | 底盘日志 / 外部相机 / 里程回读 |
| 两键开火最终是否汇合 | 汇合点在云台 C++ 补丁，dzbrain 看不到 | 云台补丁日志 / 裁判打分 |
| 真开火了没 | 命中判据在 `aim.py`，但最终裁决在裁判 | 裁判反馈 / `shots/hits` 变化 |
| 车端实际跑的版本 | 本地代码 ≠ 车端部署（`deploy_to_car.py` 才同步） | `car_reconcile.py` 对账 |
| 域号/QoS 是否匹配 | 静默失败（跨机最经典） | 双方 `ros2 topic list` 对称性 |

**本页能离线判的**：门禁 GREEN、单测、配置自检、路线校验、仿真闭环、车端离线对账。**本页不能离线判的**：实车闭环、开火汇合、底盘真执行。

---

## 附：与相邻文档的关系

| 文档 | 讲什么 | 何时用 |
|---|---|---|
| `07_发车门禁口径_G1-G4.md` | 发车前四门怎么验 | **发车前**必跑 |
| `09_明早全流程SOP_20261029.md` | 这一趟按什么顺序做 | **当天**流程 |
| `10_日志阅读课_新手版.md` | 怎么读日志 | **看不懂日志**时 |
| `15_dzbrain零基础源码导读.md` | dzbrain 结构科普 | **想理解代码**时 |
| `13_模块协调与链路展开.md` | 模块怎么协作 | **想改某模块**时 |
| `01_症状索引_52条.md` | 症状 → 原因速查 | **有具体现象**时 |

## 核实边界

- **〔实〕**：§0-§3.1 所有离线命令与遥测/退出码字段，2026-10-30 读源码 + 本机实跑。
- **〔码〕**：§0、§1、§2、§3.2/§3.3 现场命令的"预期输出"，按代码打印格式推，**未连车/机实跑**，第一次现场用请替换成真实输出。
- **未核实**：车/机当前在线 IP、云台补丁的两键汇合最终逻辑、底盘真执行日志格式。