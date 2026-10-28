---
卡号: S1-CORRIDOR
ns: agc-1026
agent: 39437d35-62ab-4680-acef-62fc4a6232b8
owner: 39437d35-62ab-4680-acef-62fc4a6232b8
reviewer: lead
树节点: R-17
写域: "src/dzbrain/dzbrain/tasks/drone.py(最小改) + 新测试 test_drone_corridor.py + _tmp_session/S1-CORRIDOR/ + 本卡"
status: in_progress
创建: 2026-10-27
---

# S1 · 起步走廊 35 s 确认（机端 + 车端）

## 已查出的必然冲突
drone.field_layout_wait_s 默认 30.0（等布局超时 exit 3）< 用户 35 s 走廊确认 ⇒ 按新节奏必然 exit 3（2026-10-27 18:50 实跑即死在此）。p1-geom 同批已把默认值改 60.0。

## 任务
- 车端：审计「首布局到达 → 规划 → 起步」；出可复跑离线重放/注入脚本；量化 first_layout_wait_s 与 corridor_confirm_s（注入须标注）。
- 机端：定位「谁发 field_layout」；若不在本仓，给文件级证据 + 最小改动点与验收判据（起飞后先扫 N10 左 5~8 格、≤35 s 发首布局、日志 corridor_confirm_s）。
- 开赛信号 gate：准备阶段禁升空/禁采集，做设计 + 车端可判部分。
- 视场按约 6 格宽估算覆盖速度。

## 验收
- [x] V1 corridor_confirm_s 出数（判据 ≤35 s；注入 12.00/15.00 s，真实机端未验）
- [x] V2 40 s 后到布局仍继续；0 条布局 60 s 才退出（负例；已随 S3 main.py 落地，实测 60.1 s）
- [x] V3 布局非法 → 拒收 +1 且不崩（4 负例）
- [x] V4 机端不在本仓时给证据 + 现场最小验证命令（证据可用；**判因退回**，见 Lead 复核）

## 五项必填
- 读过的台账条目：D-044（提交/推送等用户令）、D-046（候选验收纠偏）、D-047（离线边界）；本卡关联下条 Lead 裁决。
- 复用检索：复用现有 route/physics、route/planner、core/config、main.py StepLoop、drone.field_layout 协议，不新建第二套同类工具。
- 新增轮子及理由：无（仅在本卡写域内改现有实现）；如需新模块，理由写在本节。
- 验收命令：见「验收」逐条，全部机器可判。
- 回滚点：见「回滚」。

## 执行报告
（coord-s1 追加）
### coord-s1 · 2026-10-27 23:5x

**改动（仅写域内两文件）**
- `dzrobot_v2/src/dzbrain/dzbrain/tasks/drone.py`：纯增量观测 —— `corridor_cells_of()` +
  `CORRIDOR_MIN/MAX_CELLS`；`DroneTask` 新增 `first_layout_s`/`corridor_confirm_s`/
  `corridor_since_gate_s`/`layouts_pre_gate`；`_note_layout_arrival()` 在"新版本被接受"时计时；
  `corridor_stats()`；teardown 追加一行摘要。**不改变**接收/拒收/版本语义（A/B 逐位一致），
  不发事件、不新增黑盒键。
- 新增 `dzrobot_v2/src/dzbrain/tests/test_drone_corridor.py`（48 断言）。**刻意不依赖
  `config/map.yaml`**（该文件当前工作树不存在，`test_drone.py` 因此本来就跑不起来，非本卡造成）。
- 未动 `main.py` / `config/**` / `core/config.py` / 索引 / 状态板 / 台账 / frozen.json。

**验收（复跑命令 + 实测输出）**
```
cd dzrobot_v2 && python src/dzbrain/tests/test_drone_corridor.py                → 48/48 ✅ rc=0
cd dzrobot_v2 && python src/dzbrain/tests/test_drone_corridor.py --require-production-60 → 48/49 rc=1（仅 F 红）
cd .. && python _tmp_session/S1-CORRIDOR/replay_corridor_injection.py           → 5/6 rc=1（仅 F 红）
```
日志：`_tmp_session/S1-CORRIDOR/logs/{test_drone_corridor,test_drone_corridor_gated,replay}.log` +
`replay_result.json`；说明见 `_tmp_session/S1-CORRIDOR/REPORT.md`。

- **V1 corridor_confirm_s 出数（≤35 s）→ 通过（注入）**：一帧到位 12.00 s；分两帧 5.00→**15.00 s**；
  永不覆盖 → **None**（不填 0）；v1 整图 → 首帧即确认；起点贴 A 列 → 走廊为空不出数、不死等。
  ★ 全部为构造帧 + 假钟，**非无人机实测**（机端不可达）。
- **V2 等待门 → 逻辑通过 / 生产真值红**：60 s 门 + 40 s 到布局 = **40.10 s 继续**；
  0 条布局 = **60.10 s** 才退出；对照 30 s 门 = **30.00 s** 退出；55 s 迟到仍继续；61 s 不回补。
- **V3 布局非法 → 通过（4 负例）**：缺 `mines`(1) / 越界格 `Z9`(2) / 好布局后坏布局保留同一对象(3) /
  同版异内容(4)；全部 `layout_rejects` 递增且不崩、不清空。
- **V4 机端 → 完成取证：本仓（车端仓）无发射器**，机端代码在 v1 树 `dzsmartscout/`（HEAD 已提交态）：
  `perception/protocol.py:89` `PAYLOAD_TYPES` 无 `field_layout`；`build_field_layout`/`FIELD_LAYOUT_VERSION`
  **实测 hasattr False**；而 `perception/tests/test_field_layout_v2.py:19` 导入它们、`run_tests.py:35`
  又把它列入套件 ⇒ 实测 `ImportError: cannot import name 'FIELD_LAYOUT_VERSION' from 'perception.protocol'`。
  ⇒ 18:50「等 30 s 收 0 条」= **发射端缺实现**，不是链路/域号问题。最小改动点与现场只读验证命令见 REPORT §4。

**★ 跨卡阻塞（不在本卡写域，需 Lead 派工）**
`core/config.py:774-786`：`Config.get(key, 字面量)` 在 key 不在 yaml 时返回**字面量**、丢掉 SCHEMA 默认值。
`main.py:1659` 正是 `cfg.get("drone.field_layout_wait_s", 30.0) or 30.0` ⇒ **p1-geom 已把 SCHEMA 改 60.0
（实测），生产链路仍实测等 30.0 s**（< 35 s 走廊节奏 ⇒ 仍会在走廊确认前 exit 3）。修法二选一：
① `config/robot.yaml` 的 `drone:` 段显式写 `field_layout_wait_s: 60.0`（p1-geom 写域，足够）；
② `main.py:1659` 去掉字面量（rt-gate 写域）。本卡只量化，未改。

**开赛信号 gate（§3 设计）**：车端**结构上没有起飞/采集通道**（`drone.py` 对外只发 `car_state`:1322 与
`ack`:906），故车端可判部分 = 预 gate 帧计数 + `corridor_since_gate_s`（实测 −100.0 / −40.0 即赛前采集）；
**机端禁升空/禁采集仍无机制**，只有约定。

**风险 / 未验**：① 注入 ≠ 实测，真值待机端发射器落地后跑一趟；② 走廊秒数从 `DroneTask.setup()` 起算，
准备阶段先起 stack 会虚高，配 `start_gate_ts` 才是"开赛后多少秒"；③ 用户口径"视场约 6 格宽"与
`coverage.corridor_m=0.525`（=1.5 格几何反推）差 4 倍，以谁为准需现场实测视场；
④ `test_stepper.py` 实跑 117/118，1 红在 `12.11` 火控/离站断言（mission/aim 侧，非 drone），
且该文件正被并行卡改写，不计入本卡结论；⑤ 本卡 `树节点: R-17` 在 `00_树.md` 里尚无 R-17 行（树只到 R-11），
树是 Lead 单写点，未代写。

**回滚点**：删除 `_note_layout_arrival` 调用与其定义、`corridor_cells_of` 与两常量、`__init__` 新增字段、
setup 两段读取、teardown 追加行；新测试文件整体删除。改动前 `drone.py` 与 HEAD 一致（当时 `git status` 未列出它）。

**§4 门禁（23:5x 实跑，含他卡并行改动）**：`check_doc_policy` **GREEN**（0 error/0 warn）；
`check_anchors` 新增红 6 —— **6/6 全在他人卡**（`agc-1024/AGC-T1-PROGRESS-20261024.md`、
`lead-1023/{ACK-PURGE,DOC-LIB-TIDY}-20261027.md`），本卡 0 红；`check_numbers` 新增漂移 8 ——
**1/8 是本卡**（`thread_rate_drone` 锚点 `drone.py:535` ⇒ 现应 `:584`，本卡 +49 行所致），
另 7 条来自 `config/robot.yaml`/`physics.py` 的他卡改动；`tools/wf/check.py` RED(1) = 他人新增
未登记 `core/run_archive.py`。**待他人落的一行修法**：`docs/20_资料索引库/数字速查.yaml` 的
`thread_rate_drone` 代码锚点 `:535`→`:584`（本卡写域不含该库文件，未代写）。
详见 `_tmp_session/S1-CORRIDOR/REPORT.md` §8。

## Lead 复核（2026-10-28 00:3x · §11 独立复跑）

- 复跑：`test_drone_corridor.py` **48/48 rc=0**；`--require-production-60` **49/49 rc=0**；`replay_corridor_injection.py` **6/6 rc=0**。→ 报告里的「48/49（仅 F 红）」「5/6（仅 F 红）」**已过期**：`main.py:_wait_for_field_layout` 已被 rt-gate 在途改动改为 `_cfg_float(..., _spec_default(...))`，生产实测等待 **60.1 s**。
- ⇒ 跨卡派工**取消**：不必让 p1-geom 在 yaml 补 `field_layout_wait_s: 60.0`；`Config.get(key, 字面量)` 的机制缺陷仍在（⑱ 探针保留），但生产调用点不再传字面量。**耦合记账**：该修复寄生于 S3/G1 未提交改动，若回退则 30 s 回归，收口时两卡同批验。
- V4 **判因退回**：18:50 的 `sub drone_in pubs=0` 与机端 SSH 超时同现；发射端真源是机端 `~/dzlink_tools/layout_batches.py`（T12 卡 + DRONE-SYNC 归档：`sys.path.insert(.../dzbrain_pkg)` → `from dzbrain.scout.layout import ...` → 发 `/dzlink/drone_to_car`），不在仓且未证运行。`perception/protocol.py` 无 field_layout 是 D-039 **已删重复构造器**的遗留测试残影（`test_field_layout_v2.py:19` 仍 import 已删符号，故 ImportError），不是 live 链路缺失 ⇒ **「补 build_field_layout」违反 D-039，禁止**。正确动作：修遗留测试（或标 deprecated）；机端在线后复核 layout_batches 实发。
- 树节点：R-17 **已存在**（23:37 建，见 `00_树.md:38`），报告中的「树只到 R-11」是旧快照误读。
- `thread_rate_drone` 锚点 :535→:584 由 +175 行所致，属收口数值漂移，收口时统一 `--fix-sources`/改 `数字速查.yaml`，不在本卡处理。
- 结论：观测与门逻辑**接受**（V1–V3）；V4 证据可用、**判因退回**。卡面「生产仍等 30 s」一句以本段为准。


