---
卡号: RECT-LINK
ns: agc-1026
agent: link-analyst
owner: link-analyst
reviewer: lead
树节点: R-2
写域: "_tmp_session/AGC-RECTIFY-20261026/link/"
status: in_progress
创建: 2026-10-26
---

# RECT-LINK · 布局和段完成回执

## 目标
用户本轮目标：优先完赛，在安全和时间余量内尽量压雷、两有效最佳射击点并命中。不默认为禁火仅跑路线。

共享任务按派发为 **task-2**（卡内旧 task-13 为历史号，运行时重启后 board 重建；本会话工具面未暴露 team_task_list/get/claim，无法在 board 侧领卡——已在首报中如实告知 Lead，agent/owner 填真实子会话 ID，不冒用 lead）。

本卡实际执行范围：独立复核 C01（StepLoop 段完成单 → P_CAR_STATE done/done_seq）与 C02（DroneTask 优先 car_state、PMISSION 回退）真实行为；用真实 Tracker/Mission 验证 WAIT_INFO 是否只是"一拍刹车"；隔离副本复跑现行套件；不重造协议、不改生产。

## 读过的台账条目
执行者本轮实际读取并遵循：
- **D-008 冻结区**：`src/dzbrain/dzbrain/**`、`config/**` 等冻住；本卡生产代码只读，候选只落隔离副本，未改主树一行。
- **D-012 规划引擎**：`planner.engine=tour` 唯一规划入口；本卡不新建规划器、不改引擎。
- **D-020 首段可靠 GO**：不要求 154 格先齐；本卡只验证布局 v2 协议与段边界，不把"全图齐"当发车条件。
- **D-023 每局真图不纳管为默认**：主树 `config/map.yaml`、`map_v3.yaml`〔已删〕 工作区已删除（`git status: D`）；隔离副本从 `git HEAD` 显式注入历史图仅作**测试 fixture**，不恢复主树、不代表现场新真图。D-023 同时记录 `test_link_e2e`/`test_mission` 属已知 13 红基线（C 类）。
- **D-033 靶速 0.3 m/s**：本卡未改任何火控/靶速口径；旧 0.04/5.6px/s 注释未用于任何判定。
- **D-044 不自动 commit/push**：全程无 add/commit/push/reset/clean/restore。

## 复用检索结果
1. 资料库检索（真实命令 `python docs/_工具/doc_search.py "段完成回执 WAIT_INFO done_seq" -k 3`，rc=0，0.218 s / 411 篇）命中：
   - `docs/00_项目总览/技术全景_E_空地链路与工程设施.md` L318/L427/L429（D1b：段完成单闩锁 + 1 Hz 重发 + 单调 done_seq；旧 1 拍窗口漏收率 ~86%）
   - `docs/10_agent提示词/04_场景/空地协同闭环推进.md` L216（完成回执以段 ID/序号去重，不能靠帧年龄代替推进）
   - 复用结论：本卡不再造协议，直接测现行 main/drone/mission 的实现。
2. 代码复用（`dzrobot_v2/docs/复用清单.md`）：
   - 任务状态机 = `mission.py`/`tracker.py`（唯一权威实现，禁止另写状态机）
   - 规划唯一入口 = `route/planner.py`（本卡只调用，未改）
   - 布局生成 = `dzbrain/scout/layout.py`（LayoutStream/TruthMap，机侧唯一实现）
   - 测试用假件 = `tests/fakes.py` FakeBridge/FakeClock（鸭子类型，和 RosBridge 同签名）
3. 未新建任何永久工具/规划器/仿真器；探针均为一次性脚本，落本卡 `_tmp_session/.../link/probes/`。

## 新增轮子及理由
**无永久新增轮子。** 新增件全部是隔离副本内的一次性证据脚本（不登记、不进生产）：
- `probes/setup_iso.py`：建隔离副本 + 注入 HEAD 历史地图 fixture + 记 preimage sha256；
- `probes/run_suites.py` / `rerun_tools.py`：现行套件复跑与真实 rc/日志落盘；
- `probes/probe_batches.py`：10 批布局 targets∩known / target∈known / batch7 是否有靶；
- `probes/probe_c01_c02.py`：生产者-消费者最小闭环（真实 LayoutStream→DroneTask→StepLoop→MissionTask→DroneTask→LayoutStream）；
- `probes/probe_wait_info.py`：真实 Tracker+Mission 下 WAIT_INFO 停段持久性 + 白盒反证；
- `_tmp_session/AGC-RECTIFY-20261026/link/probes/make_candidate.py` + `_tmp_session/AGC-RECTIFY-20261026/link/candidates/wait_info_stop_guard.diff`：最小候选（未应用主树）。
理由：现行 test_stepper 的 WAIT_INFO 只断言 `brakes >= 1`（`test_stepper.py:290` 3.4），没有真 Tracker，测不出"单拍刹车是否被覆盖"；这一缺口必须另立探针，但不值得新增永久测试工具（等 Lead 决定是否收编）。

## 写域
`_tmp_session/AGC-RECTIFY-20261026/link/` 及本卡；其余生产代码只读。主树 `git status` 未出现本会话新增/修改的生产文件（仅既有在途改动）。

## 验收命令与清单
统一 env：`PYTHONUTF8=1 PYTHONIOENCODING=utf-8 PYTHONDONTWRITEBYTECODE=1`，解释器 `python -B -X utf8`；cwd 与真实 returncode 见各条。PASS/FAIL/UNKNOWN 分开。

- [x] 生产者消费者最小闭环复现C01/C02，非注释/同词判定
  - 命令：`python -B -X utf8 _tmp_session/AGC-RECTIFY-20261026/link/probes/probe_c01_c02.py`（cwd=v1，rc=0）
  - 结果：8/8 PASS。真实闭环跑出 4 段（done_seq 1→4，block=F11/D7/E7/F7）；`P_CAR_STATE` done.seq 与 done_seq 严格同值 +1；机端 car_state 报文带齐 4 个 done.seq（相位列表 SEGMENT_DONE×4 + WAIT_INFO×3，证明不是只活一拍）；消费者按 done.seq 每段只推进一次；重复帧被 `seq <= last_done_seq` 门挡住；C02-A 在 `P_MISSION_PHASE=IDLE` 时仍报 `phase=SEGMENT_DONE`（证明 car_state 优先，不是 grep 同词），C02-B 无 car_state 时回退 `P_MISSION_PHASE=DRIVE` 且不带 done/done_seq。
- [x] 靶格未知不泄露、已知且确认的靶可进入后续任务
  - 命令：`python -B -X utf8 _tmp_session/AGC-RECTIFY-20261026/link/probes/probe_batches.py`（rc=0）
  - 结果：10 批全部 PASS 协议不变式——每份布局 targets/mines/obstacles ⊆ known；target_cell ∈ known；车端 `normalize_step_layout` 全部接受；version 1→10 单调。**靶首次出现 = 第 7 批（index 6，known=100，targets=[D5,D6]）**；第 1~6 批 targets=[] 不泄露未知格。边界：在 HEAD 第六图 + 当前在途 `robot.yaml` 组合下，全 known 时 `plan_mission_route` 返回 None（规划器结构自检报"双射击停站应为 2 个，实际 1 个：B4"），target 因此回退 end/N5；这是**配置-地图对不匹配**，不是 build_layout 灌了未确认真值，真实 map7 组合仍 UNKNOWN（见"未验证"）。
- [x] 当前段完成回执可被消费且不重放旧段
  - 同上 C01 闭环：消费者接收 seq=[1,2,3,4] 无重复/无跳号；重复注入同 done.seq 不再产 layout；`LayoutStream` 产出的 next layout 版本严格递增且 target⊆known（done 3/4 时目标未变 ⇒ 按设计只发心跳不换版，emit_step_v3 同口径）。
- [x] 布局失效WAIT_INFO确实停段，旧单测不增红
  - 命令：`python -B -X utf8 _tmp_session/AGC-RECTIFY-20261026/link/probes/probe_wait_info.py`（rc=0）
  - 结果：P1 真实闭环 65 个 WAIT_INFO 拍，mission.segment_status().active 全为 False、mission.phase 全为 IDLE（不在 Tracker.ACTIVE_PHASES）；P2 真 Tracker 连续 40 拍 cmd_vel **全零**、刹车 +40、40 拍后仍 WAIT_INFO（不假完赛/不超时）。
  - "旧单测不增红"：isolated 复跑 `test_stepper.py` 仍 116/117；`test_car_state_report.py` 15/15；主树未改任何测试。
  - ★二次复核修正（19:36~19:56 · link-analyst，§8）：以 **git HEAD=945bbc14** 为基线，**链路层无新增红**；但全仓相对 HEAD 有 **3 处新增红**——test_stepper 12.11（在途 mission.py 开火门时序）与 test_drone ⑮ 2 条（在途 planner.py 双停站硬校验）。原"与派发前一致"只对 round2 基线成立，**相对 HEAD 12.11 是新红**；详见 §8.2 / §8.6。
  - 补充证据（非生产可达，白盒）：P3 在 mission 仍 DRIVING 时强制 StepLoop 进 WAIT_INFO，单拍刹车 1 拍内被 Tracker 覆盖（5/5 拍非零），说明"停住"实际依赖 mission 离开 ACTIVE_PHASES，而非那一次 publish_brake。
- [ ] Lead独立复跑后认可（作者不得自行勾此项）

## 执行记录
时间：2026-10-26 19:00~19:36（Asia/Shanghai）。执行者：5f4aec44-2a63-40dc-9736-fd54a812f0e5。

### 1. 基线与隔离副本（不碰主树）
- 主树事实：`dzrobot_v2 git status` 显示 `D config/map.yaml`、`D config/map_v3.yaml`；scout/drone/link_e2e 因默认图缺失中断。
- 隔离副本：`_tmp_session/AGC-RECTIFY-20261026/link/iso_20261026_1916/dzrobot_v2/`，只复制 `src/dzbrain`（6.6M）+ `config/*.yaml`；地图从 `git HEAD` 注入：
  - `map.yaml` sha256=`385e07182aa84ebc26aab6e1c970838d912fc4f0ddea4abcacbf1b2dc63ed20d`（2370 B）
  - `map_v3.yaml`〔已删〕 sha256=`59ae71ec30c11972c86066c15a36ca1ba40f643b16f7ed66afc04660798688e9`
- 生产 preimage（本卡相关）：`main.py`=`50251579813e9ccbb90e8fd8fbe6f9060dfc1d2d168391edb53140c6930bc57b`；`mission.py`=`2a671642d3520b51…`；`drone.py`=`46cef961ad1bc9eb…`；`tracker.py`=`64837c927e137200…`；其余见 `link/baseline_link.json`（含 root/nested HEAD、完整 status、逐文件 sha256）。
- 主树地图未恢复（遵守"不能恢复主树 runtime 图"）；历史 fixture 仅存在于隔离副本。

### 2. 隔离副本现行套件复跑（真实 rc，日志全落盘）
命令模板：`python -B -X utf8 <ISO>/src/dzbrain/tests/<suite>`，cwd=`ISO/dzrobot_v2`；结果 `_tmp_session/AGC-RECTIFY-20261026/link/runs_20261026_1916/result.json`。

| 套件 | rc | 结果 | 归因 |
|---|---|---|---|
| test_scout.py | 0 | 79/79 PASS | 首次 77/79 是隔离副本缺 `tools/scout_sim.py`；补必要件后全绿（非生产缺陷） |
| test_drone.py | 1 | 175/177 | 2 红均在 ⑮：HEAD 第六图 + 当前在途 robot.yaml 规划自检"双射击停站应为 2 个，实际 1 个：B4"（配置-地图对不匹配；非本卡链路） |
| test_stepper.py | 1 | 116/117 | 唯一红 12.11（基线一致）；见 §3 独立归因 |
| test_link_e2e.py | 1 | 41/44 | 1.7 桥洞路线期望随 robot.yaml/地图对漂移；6.5/6.6 属 D-023 已登记 C 类 baseline（test_link_e2e 在 13 红名单） |
| test_car_state_report.py | 0 | 15/15 PASS | C02 行为基线（无 car_state 回退 / 有 car_state 段级字段 / 同 phase 换段 / 非法回退） |
| test_mission.py | 1 | 229/246 | 17 红集中在桥洞默认格 K7 vs 测试期望 K10/K9 与停站台账 D3/G3 vs 当前 F4/B4：当前在途 robot.yaml 与 HEAD 图不一致；属 D-023 已登记 baseline，非本卡引入 |

### 3. stepper 12.11 的独立归因（不篡改测试期望）
- 现象：`fire_gate` 首次/末态均为 []，但 12.9（SETTLE→ACQUIRE→FIRE→DONE）、12.12（SEGMENT_DONE stop_kind=shoot）均 PASS。
- 机制：测试在 `mission.phase == STOP` 的第一拍就 `add_shots(3)`（`test_stepper.py` 12 节循环内），而 `shots_needed=3`；FIRE 首拍 delta 已 ≥3 ⇒ `_phase_fire` 直接 `_complete_stop`，永远到不了 `_set_fire_gate(True)`（`mission.py:3590-3605`）。
- 独立反证：把"加 3 发"时点改到"fire_gate 首次为 True 之后"（只在隔离副本的 `_diag_stepper_1211.py`，不动原测试）→ **117/117 全绿**，12.11 首次 [True,False]。结论：12.11 是**测试序列与仿真发数时序**的缺陷，不是生产开火门失效；是否修测试由 stepper owner/Lead 定。
- 证据：`link/runs_20261026_1916/diag_stepper_1211.stdout.txt`（rc=0；117/117）。

### 4. C01/C02 最小闭环（按"生产者-消费者"复现，不靠注释/同词）
组件全部真实：`LayoutStream`（机侧）→ field_layout JSON → `DroneTask`（车侧解析，写 P_FIELD_LAYOUT）→ `StepLoop` → `MissionTask`（段执行）→ `P_CAR_STATE`（done/done_seq）→ `DroneTask`（car_state 发布）→ 消费者（emit_step_v3 的 `done.seq > last` 口径）→ `LayoutStream.observe_car_block(done=True)` → 下一份 layout。
- 实跑 188 拍 / 4 段完成；轨迹 block=F11→D7→E7→F7，known 从 45 逐批增长；
- done 帧相位分布：SEGMENT_DONE×4 + WAIT_INFO×3（同一 done_seq 跨相位重发可见）；
- 消费者 seq=[1,2,3,4] 各一次；重复注入被 seq 门挡住；next layout v6/v8（done 3/4 目标未变 ⇒ None，符合 emit_step_v3"只发心跳"）；
- C02-A：强行令 `P_MISSION_PHASE=IDLE`，报文仍 `phase=SEGMENT_DONE`（car_state 优先）；C02-B：删掉 car_state，报文回 `phase=DRIVE` 且无 done 字段（PMISSION 回退）。
- 结论：**Lead 先前的 C02 RED 是旧 checker 无条件 grep 造成的误判，实测 PASS；C01 闩锁/1 Hz 重发在真实 DroneTask 1.4 Hz 采样下可被消费**。
- 证据：`link/probe_c01_c02_result.json`（pass=8，fail=0）。

### 5. WAIT_INFO 是否"仅一拍刹车"
- P1 不变式（真实闭环）：65 个 WAIT_INFO 拍全部 mission.phase=IDLE 且 segment_status().active=False；无一拍仍 active。
- P2 真 Tracker 40 拍：非零 cmd_vel 0 条，brakes +40，仍 WAIT_INFO。
- P3 白盒反证（**非生产可达**）：mission 仍 DRIVE 时强制 `loop._to_wait_info(...)`，随后 5/5 拍 Tracker 发非零命令（如 (0.12,−30.0)）⇒ 一旦出现该组合，单拍刹车会被覆盖。
- 结论：**在现行状态机全部可达路径上，WAIT_INFO 是真停段**（停住由 Tracker 的 phase 白名单 + Mission 离开 ACTIVE_PHASES 保证，不是那一次 publish_brake）；"仅一拍刹车"的担心在可达路径上**不成立**。但代码没有显式不变量守卫，P3 证明该保护是隐式的。
- 证据：`_tmp_session/AGC-RECTIFY-20261026/link/probe_wait_info_result.json`（pass=10 / fail=0 / unknown=1，unknown=P3.3 生产可达性仅静态核对）。

### 6. 最小候选（未应用主树）
- 文件：`link/candidates/wait_info_stop_guard.diff`（+16 行，仅 `main.py:_to_wait_info`）；副本 `link/candidates/iso_20261026_1955/dzrobot_v2/`。
- 语义：`_to_wait_info` 非 retry 且 `mission.segment_status().active` 时，请求 `mission.stop_segment("WAIT_INFO 停车：…")`，让 Mission 回 IDLE ⇒ Tracker 每拍刹；不触发时零行为变化。
- preimage=`50251579…3bc57b`（与生产 main.py 一致，脚本断言 true），candidate=`647929bf…0f569f`；证据 `link/candidates/wait_info_stop_guard.json`。
- 候选复跑：`RECT_LINK_ISO=<候选副本> RECT_EXPECT_GUARD=1 python -B -X utf8 .../probe_wait_info.py` → rc=0，P3 强制场景命令 0/5（补丁生效）；`test_car_state_report` 15/15、`test_stepper` 116/117（与基线一致，无新增红）。
- **裁决建议**：该候选是 defense-in-depth，不是已证可达缺陷的修复；若 Lead 认为"不加隐式保护"可接受，退回候选、只收 P1/P2 证据与回归测试即可。候选未应用生产，主树 `main.py` 字节未变。

### 7. 10 批布局与靶出现（回答派发问题）
- 第 1~6 批 targets=[]；**第 7 批（CDE rows1-6，known=100）首次出现 targets=[D5,D6]**；第 10 批 known=154、target=end N5。
- 全部布局 features⊆known、target∈known、normalize 接受、version 单调；证明 build_layout 未把未确认靶灌进布局。
- 风险提示（UNKNOWN，非本卡协议缺陷）：在 HEAD 第六图 + 当前在途 robot.yaml 组合下，全 known 时 mission 路线规划失败（自检"2 停站 vs 1 个 B4"），LayoutStream 因此只能给 frontier/end，不会把车导向射击停站；真实 `map7` + 对应 robot.yaml 的组合未在本卡复跑，**不能据此断言实车目标选择正确**。建议由 Lead 用当前 map7 对重跑 `probe_batches.py` 同口径。


### Lead 独立复核（2026-10-26 20:0x）

已亲自核验（非采信摘要）：`_tmp_session/AGC-RECTIFY-20261026/link/EVIDENCE_INDEX.json` 在盘、五个 run 的 rc/PASS 记录齐全；候选 preimage_sha256=50251579813e9ccbb90e8fd8fbe6f9060dfc1d2d168391edb53140c6930bc57b，与当前生产 `dzrobot_v2/src/dzbrain/dzbrain/main.py` 实测 revision **逐字节一致**；生产字节未动。

**采纳**：C01 段完成回执真实闭环、C02 car_state 优先于 P_MISSION_PHASE（旧 checker 的 C02 RED 确为误判）、WAIT_INFO 真停靠来自 Tracker 的 ACTIVE_PHASES 白名单而非那一次 publish_brake（单拍刹车 1 拍内被覆盖，5/5 非零，白盒反证成立）。

**纠正摘要一处**：摘要写“全 known 时 plan_mission_route 返回 None”，实际 `diag_layout_target.json` 中 plan_mission_route 抛的是 `TypeError: missing from_block`（探针调用参数不全），`plan_mission_target` 才是 null，`checkpoints` 另有 NameError。**真正有效的证据是 `plan_route_from_config` 的 PlannerError**，不要把探针自身的调用错误当成生产缺陷。

**转规划线处理**：`plan_route_from_config` 的失败为“规划结果未通过结构自检：双射击停站应为 2 个，实际 1 个：B4”，与 RECT-PLAN 单站结论同源，按 RECT-PLAN 处理，不在本卡修。

**+16 行候选（WAIT_INFO 时 stop_segment）不采纳进生产**：生产者路径本就 IDLE/active=False 已能真停；该候选只在“mission 仍 DRIVE 时强进 WAIT_INFO”这种非生产可达的白盒状态下生效，属 defense-in-depth。无当前缺陷证据即改冻结区，不符合“一次只改一个变量”。保留在隔离副本备查。

**第 5 项不勾**：整卡仍 in_progress。作者侧 4 项与本次采纳项成立，但“两有效射击站”结论受 RECT-PLAN 阻塞，卡不能单独收口。

### 8. 二次复核（19:36~19:56 · link-analyst）：三条红四选一定性 + HEAD 基线

**目的**：按 Lead 派发，把 1.7 / 6.5 / 6.6 各做四选一定性并给出 token 级原因与 A3 来源；同时用 **git HEAD 基线**回答"旧单测不增红"。本节数字全部本轮独立复跑，不采信前文摘要。

**作者侧状态**：§8 完成（四项卡面判据 + 三条存量红定性 + 两处新红归因）；`status` 仍 `in_progress`——第 5 项（Lead 独立复跑认可）与 RECT-PLAN 双站阻塞未变，作者不勾第 5 项。

#### 8.1 基线方法（为什么必须跑 HEAD）
- 嵌套仓在本轮记录期被其它线并发提交又回退（reflog：441376da → reset → 945bbc14；19:47~19:50 间 HEAD 多次变动）。**基线钉死 945bbc14**：`iso_head` 由该 commit 的 `git archive HEAD config src/dzbrain tools/scout_sim.py` 建出，另复制 `dzsmartscout/tools/perception` 供 test_link_e2e。
- 现役对照 `iso` = 工作区 `src/dzbrain`（含在途改动与未跟踪新模块）+ 工作区 `config/robot.yaml` + **同一份 HEAD 历史图 fixture**（主树 map 已删，按纪律不恢复）。
- 两副本同解释器/同 env：`PYTHONUTF8=1 PYTHONIOENCODING=utf-8 PYTHONDONTWRITEBYTECODE=1`，`python -B -X utf8`；cwd=各自 `dzrobot_v2`。
- 首轮 `iso` 缺 `tools/scout_sim.py` 造成 test_scout 77/79（两条 FAIL，rc=2 找不到文件）；补该必要件后 79/79 全绿 ⇒ **这两条 FAIL 归因"证据构造问题"，不是产品缺陷**（与 Lead 判断一致）。

#### 8.2 两基线套件对照（真实 rc）
| 套件 | HEAD 945bbc14 | 现役（含在途改动） | 差异 |
|---|---|---|---|
| test_scout | 79/79 rc=0 | 79/79 rc=0（补 tool 后） | 无 |
| test_drone | 177/177 rc=0 | 175/177 rc=1 | **+2 新红（planner）** |
| test_stepper | 117/117 rc=0 | 116/117 rc=1 | **+1 新红（12.11）** |
| test_link_e2e | 41/44 rc=1 | 41/44 rc=1（同三条） | 无（存量） |
| test_car_state_report | 15/15 rc=0 | 15/15 rc=0 | 无 |

代码/配置隔离实验（**现役代码 + HEAD robot.yaml**）：test_drone 仍 175/177、test_stepper 仍 116/117 ⇒ 两处新红由**在途代码**（planner.py / mission.py）引起，与 robot.yaml 无关。证据 `round3/cfg_isolation/*.headcfg.stdout.txt`。

#### 8.3 三条存量红：四选一定性（HEAD 已红 ⇒ 非本轮在途改动引入）
| 断言 | 定性 | 一句话原因 | HEAD 已红 |
|---|---|---|---|
| 1.7 穿洞三连 | **证据构造问题**（测试夹具漂移） | 夹具替换字面量 `cross_lower_candidates: ["K10"]` 在当前与 HEAD robot.yaml 均命中 0 次（两者都是 `["K7"]`），期望默认 K9 亦过期；产品行为正确：token 级只有一处三元替换 | 是 |
| 6.5 重发幂等 | **断言过严（测试期望错）** | A3 在 HEAD 图里本就不是本地雷 ⇒ 首片记录真差异符合设计语义；重发本身幂等（dup=1，事件数/差异条数均不增），FAIL 来自累计 `field_diff` 非空 | 是 |
| 6.6 hazard_new | **断言过严（测试期望错，6.5 的推论）** | A3 来自 mines 的 `only_remote`，与 `out_of_scope_local` 无关；反事实把 A3 加入本地雷表后期望逐字成立 | 是 |

#### 8.4 1.7 具体 token 对比（Lead 要的"改多了还是夹具不匹配"）
- base=route.tokens 68 格，注入 K11 情报后仍 68 格；`difflib` 全序列**只有一处 replace**：
  `base[5:8] = [L7, K7, J7]` → `new[5:8] = [L11, K11, J11]`
- set 差集显示 `−[J7,K7,L7] +[K11,L11]` 的原因：**J11 在 base 第 11 位已存在**（`J11_base_indices=[11]`；new 里 `[7,11]`）——set 比较隐藏了"同格位移"，于是出现"3 删 2 加、点数相等"的错觉。
- 期望侧：测试把 `["K10"]` 替换成放宽候选，并期望默认 K9 的三连（10-17 版图/配置）；现役/HEAD 默认已是 K7。K11 仍被采纳（t31 行段放宽），替换的正是当前默认 K7 三连。
- 结论：**穿洞重写逻辑没有"改多了"**；是夹具与期望随地图/配置换代失同步（证据构造问题）。修期望归测试 owner，不在本卡写域。

#### 8.5 6.5 / 6.6 证据与 A3 来源
- `drone.py::default_map_path()` 解析到 HEAD 图：mines=`A2,F4,H4,G5,E8,B10,D11,I11`，**A3 不在其中**（targets=D5,D6）。
- 当前图下复现：首片 S2（mines=[A3]，coverage cells=[A3]，complete=True）→ `dup=0 / field_diff=1 条 / 消费者 events=1 / hazard=[A3]`；同 (scan_id, slice.id) 重发 → `dup=1`，`field_diff` 仍 1 条、`events` 仍 1、`hazard` 仍 [A3] ⇒ **重发未重复记账，账本幂等成立**，6.5 失败只因断言要求"差异为空"。
- 6.6 场景（新片 N3/N4）→ `hazard_new=[A3,N3,N4]`；A3 是 mines 的 only_remote，**不是** `out_of_scope_local=8`（该数只在 last 事件里作诊断）。分类逻辑与 `FieldVerifyConsumer` docstring "无人机看到、本地图没有的雷/障" 一致。
- **反事实**：只把 A3 加入本地雷表（其余照抄 HEAD 图；临时 fixture `round3/tmp/map_with_A3.yaml`）→ 同一流程得到 `field_diff=()`（重发前后均空）、`hazard_new=[N3,N4]`，6.5/6.6 期望**逐字成立**。⇒ 期望是按"A3 是本地雷"的那版图写的：`700d61cb:config/map.yaml` mines 含 A3（C1,A3,E3,G4,I6,B7,D9,H10），当前 HEAD 图不含 ⇒ **真值图换代后的测试期望漂移**。
- 边界未漏判：diff 文本里"片外 8 未判读"与 only_remote 分列，A3 不计入 only_local。

#### 8.6 两处新红（不在本卡写域，转对口卡）
- `test_stepper 12.11`：HEAD 绿 → 现役红。12.9（SETTLE→ACQUIRE→FIRE→DONE）与 12.12（stop_kind=shoot）仍 PASS ⇒ FIRE 段确实进入；在途 `mission.py` 改为"先判 `delta >= shots_needed`/超时，再 `_set_fire_gate(True)`"，而测试在 `MissionPhase.STOP` **首拍**（SETTLE）就 `add_shots(3)` ⇒ FIRE 首拍已达标、直接 `_complete_stop`，`fire_gate` 永远没有 True。属**开火门时序契约变更 + 测试仿真时序未同步**；定性建议"测试模拟过严/未同步"，不是开火门失效；由 RECT-AIM/stepper owner 裁决。
- `test_drone ⑮ 2 条`：HEAD 绿 → 现役红。断面 `PlannerError: 规划结果未通过结构自检：双射击停站应为 2 个，实际 1 个：B4`，来自在途 `planner.py::check_shoot_stops` 硬校验；与 RECT-PLAN 同源（Lead 已转），本卡不修。
- ⇒ "旧单测不增红"判定：**链路层**（StepLoop / DroneTask / car_state / LayoutStream / test_car_state_report / test_link_e2e）**无新增红**；全仓套件层面 3 处新增红均已隔离定位到非链路代码。

#### 8.7 本轮独立复跑卡面四项（独立实现，非复用前一执行脚本）
- `round3/rect_link_round3.py` rc=0，**28/28 PASS**：
  - A1-A5：真实 StepLoop 同格停站 → `P_CAR_STATE` done/seq=1 → 真实 `DroneTask` 在 `P_MISSION_PHASE=DRIVE` 下仍发 `phase=SEGMENT_DONE`（C01）；删 car_state 后回退 `WAIT_INFO` 且不带 done（C02）；car_state 在场绝不回落 DRIVE（反证旧 grep 判红为误判）。
  - B1-B9：10 批**逐版** `targets == 真值靶 ∩ known`；target_cell 恒 ⊂ known；第 1~6 批 targets=[]，第 7 批（known=100）首批 [D5,D6] 且第 10 批不多不少；`covering_stops` 得 B4/B7 覆盖 D5/D6，规划到 B4 末 token `B4*`+`stop_kind=shoot`；反向 target_cell=D5 而 known 不含 D5 ⇒ 整份拒收。
  - C1-C5：真实 StepLoop 在 WAIT_INFO 1 Hz 重发同一 done（replay 0→3），全部帧灌进**真实 `s2run/emit_step_v3.py` 的 `on_car``（桩 rclpy 加载原文件，非重写）⇒ 只触发 1 次 `observe_car_block(done=True)`；新布局后 seq=2 只处理一次；seq=1 再灌仍被忽略。
  - D1-D9（真 Tracker）：DRIVING→段完成→WAIT_INFO 后 20 拍**零运动指令**、刹车 +20；partial 布局过期（153/154，D-022 完整布局例外不生效）⇒ WAIT_INFO(layout_stale)、replans=0、20 拍零运动、mission 非 DRIVE；全部真实迁移点 mission.phase ⊆ {FINISH, IDLE}；白盒反证：**只有**在 mission 仍 DRIVE 时强进 WAIT_INFO，单拍刹车才被下一拍命令覆盖（1 拍内 +1 条）。⇒ "WAIT_INFO 是真停段"成立，停住由 Tracker `ACTIVE_PHASES` 白名单 + mission 离开 ACTIVE_PHASES 保证，**不是**那一次 publish_brake；本卡不重开已被 Lead 否决的 +16 行候选。

#### 8.8 命令与证据（本轮真实 rc）
| # | 命令（cwd） | rc | 证据 |
|---|---|---|---|
| 1 | `python -B -X utf8 ../../round3/rect_link_round3.py`（cwd=`iso/dzrobot_v2`） | 0 | `round3/round3_result.json`（28/28） |
| 2 | `python -B -X utf8 ../../round3/link17_tokens.py`（同上） | 0 | `round3/link17_tokens.json`（token 级 diff） |
| 3 | `python -B -X utf8 ../../round3/link65_evidence.py`（同上） | 0 | `round3/link65_evidence.json`（当前图 vs 反事实） |
| 4 | 两基线套件复跑（见 §8.2；cwd=`iso/dzrobot_v2`、`iso_head/dzrobot_v2`） | 见表 | `round3/baseline_iso/*`、`round3/baseline_head/*` |
| 5 | 代码/配置隔离（临时换 HEAD robot.yaml 复跑，cwd=`iso/dzrobot_v2`） | 见表 | `round3/cfg_isolation/*.headcfg.stdout.txt` |
| 6 | `python -B -X utf8 _tmp_session/.../round3/make_evidence_index.py`（cwd=v1） | 0 | `_tmp_session/AGC-RECTIFY-20261026/link/round3/EVIDENCE_INDEX.json`（11 文件 sha256、基线 HEAD、各套件 rc 摘要） |

#### 8.9 与前一执行记录的差异（据实修正，不删前文）
- §2 表把 test_drone 2 红归为"HEAD 图 + 在途 robot.yaml 的配置-地图对不匹配"；本轮隔离实验证明**换成 HEAD robot.yaml 仍红**，真正原因是**在途 `planner.py` 双停站硬校验**（HEAD 代码绿）。
- §3 写 12.11"与派发前一致（基线）"；该判断只对 round2 基线成立，相对 **git HEAD 945bbc14 它是新红**（HEAD 117/117）。根因结论不变，但"是否新增红"必须按 HEAD 口径记载。
- 其余采纳项（C01/C02/WAIT_INFO、10 批靶出现、候选不采纳）本轮独立复跑一致，无异议。

#### 8.10 建议（只提不改）
- 1.7/6.5/6.6 的修法应由测试 owner 决定：① 1.7 夹具改为"从当前 robot.yaml 读默认格再构造期望"，并把 set 比较改为**有序**三元比较；② 6.5/6.6 显式注入与断言匹配的 `local_map`，或把断言改为"重发不新增事件/差异"而非 `field_diff == ()`。本卡不改测试期望（禁例）。
- 卡仍受 RECT-PLAN（双站）阻塞，第 5 项不勾。

## 未验证
- 无 ROS、无车、无无人机：全部为 FakeBridge/FakeClock/真实任务类离线闭环；**不证明实车 GO**，不发布/起栈/发车/出光/飞行。
- 历史地图 fixture（HEAD 第六图）≠ 现场新真图；map7 组合未验证（见 §7）。
- 候选补丁未应用生产、未跑全量回归；P3.3 生产可达性仅静态核对 `main.py:2111-2117`（步进模式强制 auto_start=false），未动态复现启动竞态。
- 12.11 归因未改原测试；是否重钉由 stepper owner 决定。
- 共享 task board：前一执行会话无 team_task_* 工具，未执行 list/get/claim/complete。**本轮（link-analyst）已 claim task-2，完成即置 completed**；卡第 5 项（Lead 认可）仍由 Lead 勾，作者不勾。
- 二次复核（§8）未做逐 commit bisect：新红归因依据是「HEAD 945bbc14 绿 / 现役红」+「现役代码 + HEAD robot.yaml 仍红」两点，未逐提交二分。
- 未复跑全部 33/39 套；test_mission 及其它会话报告的红，本卡未逐条定性（只对 test_scout/drone/stepper/link_e2e/car_state_report 五套做了 HEAD 对照）。
- 反事实 `round3/tmp/map_with_A3.yaml` 仅用于把 6.5/6.6 期望归因到旧真值图；**不代表建议改图/改测试**，也不恢复主树 runtime 图。

## 回滚点
- 生产零改动：主树 `main.py/mission.py/drone.py/tracker.py/tests/**` 字节未变（preimage 见 `link/baseline_link.json`；候选 preimage 与生产一致）。
- 本卡全部新增件只在 `_tmp_session/AGC-RECTIFY-20261026/link/` 与 `link/candidates/`；如需回退，删除该目录下 `iso_20261026_1916/`、`runs_20261026_1916/`、`probes/`、`candidates/`、`baseline_link.json` 即可（本卡正文保留）。禁止对主树 git reset/clean。
- 若 Lead 采用候选：以 `candidates/wait_info_stop_guard.diff` 对生产 `main.py` 打补丁前，先核对 preimage sha256=`50251579…3bc57b`；不符不得应用。该候选本轮**不重开、未应用**（Lead 已裁决不采纳）。
- 二次复核新增件全部在 `_tmp_session/AGC-RECTIFY-20261026/link/round3/`（`iso/`、`iso_head/`、`baseline_iso/`、`baseline_head/`、`cfg_isolation/`、三个一次脚本 + JSON 证据 + `_tmp_session/AGC-RECTIFY-20261026/link/round3/EVIDENCE_INDEX.json`）。回退 = 删除 `round3/` 整目录与 `iso*`，本卡正文保留；禁止对主树 git reset/clean/restore。
- 基线口径：`iso_head` 固定为 **945bbc14**；嵌套仓在本轮记录期被其它线并发提交又回退（HEAD 曾 441376da / caaee3b1），故不拿"当前 HEAD"做判据（详见 `_tmp_session/AGC-RECTIFY-20261026/link/round3/EVIDENCE_INDEX.json` 的 `repo_head` 说明）。

## 文档变更
仅本卡（执行记录 §8 二次复核、清单补充、经验总结补充）；树 `00_树.md`、目录索引、状态板、台账由 Lead 单写，本卡未动。

## 经验总结
- **现象**：旧 checker 无条件 grep `PMISSION` 判 C02 RED；实际行为是 `car_state` 优先、`PMISSION` 回退。**处置**：用真实 DroneTask + 双状态对照复现（C02-A/B），以报文实测替代同词判定；落点 `docs/20_资料索引库/空地链路/`（待 Lead 提炼）。
- **现象**：WAIT_INFO 的 `publish_brake` 只发一次，却看起来"车停住"。**机制**：真停由 Tracker 的 `ACTIVE_PHASES` 白名单保证（Mission 段完成→FINISH/IDLE），StepLoop 单拍刹车只是当前拍的保险；一旦 mission 仍 active，1 拍内被覆盖（P3 实测）。**处置**：可达路径实测全绿；候选 `main.py:_to_wait_info` 停段守卫（+16 行）供 Lead 裁决；落点 `docs/20_资料索引库/车端/` 或坑库（待 Lead 定）。
- **现象**：`test_stepper` 12.11 长期红但 12.9/12.12 绿。**机制**：测试在 STOP 首拍预加 3 发，FIRE 首拍即达标，永不发 `fire_gate=True`。**处置**：隔离副本把加发时点后移 → 117/117；不改原测试期望，交 stepper owner 决定。
- **现象**：主树删图导致 3 套测试中断，容易误判成"回归红"。**处置**：隔离副本 + 显式 HEAD fixture + preimage 哈希，区分"缺 fixture"与"真回归"；落点 本卡 §1/§2。
- **现象**：测试红容易直接算到"在途代码"头上，或反过来算成"存量 baseline"。**处置**：先跑 **git HEAD 基线**（同解释器/同 fixture），再跑现役；同一套测试 HEAD 绿、现役红才是新增红。**机制**：本轮 test_drone 2 红被前文误记为"配置-地图对不匹配"，隔离掉 robot.yaml 后仍红，才锁定是 planner.py。落点 本卡 §8.1/§8.9。
- **现象**：`removed/added` 用 set 差集比较，出现"3 删 2 加却点数相等"的假象。**机制**：J11 在旧路线另一个位置已存在，set 看不见"同格位移"。**处置**：路线类断言必须用**有序对比**（difflib / 逐位元组），set 只能当汇总。落点 本卡 §8.4。
- **现象**：6.5/6.6 断言"重发后差异为空"，但实现是累计留痕。**机制**：断言与"真值图版本"绑定——A3 在 10-17 图里是本地雷，现 HEAD 图不是。**处置**：① 逐条把断言绑到显式 `local_map` fixture；② 幂等性断言应写成"事件/差异**计数不增**"，不是"差异为空"。落点 本卡 §8.5。
