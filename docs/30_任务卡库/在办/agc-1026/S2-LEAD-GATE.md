---
卡号: S2-LEAD-GATE
ns: agc-1026
agent: 6719d8a0-37ff-45a1-af0b-f24157aa831b
owner: 6719d8a0-37ff-45a1-af0b-f24157aa831b
reviewer: lead
树节点: R-18
写域: "src/dzbrain/dzbrain/main.py + 唯一命名新测试 + _tmp_session/rt-gate/ + 本卡"
status: in_progress
创建: 2026-10-27
---

# S2 · 安全信息量门（领先量判据）

## 判据（本期只记不拦）
lead_cells = 已确认可通行格数 − 车前方需走格数；lead_cells < 2 ⇒ 日志 info_lead_low，可选降速。
禁止：用车速追无人机（会冲出情报边界）；只允许降速或原地等。

## 验收
- [x] `info_lead_low` 计数与最短 `lead_cells` 出数（自检套件覆盖；真实链路待机端）
- [x] 本判据本期只记不拦；sim 越界 74 拍与 lead_cells 无因果关系，已按「判据错则回退重定」挂账
- [x] 负例 `lead_cells=-1` → `info_lead_low` 且不拦停

## 五项必填
- 读过的台账条目：D-044（提交/推送等用户令）、D-046（候选验收纠偏）、D-047（离线边界）；本卡关联下条 Lead 裁决。
- 复用检索：复用现有 route/physics、route/planner、core/config、main.py StepLoop、drone.field_layout 协议，不新建第二套同类工具。
- 新增轮子及理由：无（仅在本卡写域内改现有实现）；如需新模块，理由写在本节。
- 验收命令：见「验收」逐条，全部机器可判。
- 回滚点：见「回滚」。

## 执行报告

**结论：lead_cells 计算 + `info_lead_low` 只记不拦落地（含负领先量注入），本卡 5 项判据全绿。**

### 改了什么
- `StepLoop._lead_update(plan, from_cell)`：`lead_cells = 已确认可通行格数(len(known)) − 车前方需走格数`
  （`ahead = len(plan.cells) − 1`，不含当前格）；`lead_cells < 2` ⇒
  `ℹ️ INFO key=info_lead_low lead_cells=… known_cells=… ahead_cells=… from_cell=… target_cell=… segment=…`。
- **只记不拦**：`info()` 不进 `warn_counts`、不改相位、不刹停、不降速（本轮按卡面"只记不拦"，
  未实现"可选降速"）；`lead_cells` 同时进 `status()` 与 `P_CAR_STATE`（机端/复盘可读）。
- 每段规划后算一次 + `trace.jsonl` 记 `event=lead`。

### 验收
```
S2.1a lead_cells = known(3) − ahead(2) = 1（<2 ⇒ 记 info）
S2.1b ℹ️ INFO key=info_lead_low lead_cells=1 known_cells=3 ahead_cells=2 …
S2.2  负例-1 lead<2 不拦停（相位不在 SAFE_STOP / WAIT_INFO_TIMEOUT）
S2.3  负例-2 info_lead_low 不计入 warn_counts（只记不拦，不是 warn）
S2.3b 负例-3 注入 lead=-2（4 格路线 / known=1）⇒ 仍出 info_lead_low，相位不变
S2.4  lead 充足（known≈152 / ahead=2）⇒ 不出 info_lead_low
S2.5  P_CAR_STATE 带 lead_cells 字段
```
`python src/dzbrain/tests/test_agc_rectify_rt_gate.py` → **69/69**（本卡 S2.1~S2.5）。

### 风险 / 下一步
- 卡面要求"全跑日志 info_lead_low 次数与最短领先量出数"需**实车一趟**才有数；本轮只有注入样本
  （lead=1 / lead=−2），**不得**当作现场判据。
- "若 lead_cells 从不为负却仍撞/越界 ⇒ 判据错"：本轮无法判定（无实车数据）；当前定义里
  `known` 是无人机已确认格、`ahead` 是本段规划格数，二者口径不同源（不是同一张图的剩余量），
  实车后若发现高估应改口径而不是调阈值。
- 未做"可选降速"（卡面写"可选"）；若 Lead 要降速，需新卡并明确与 A 区"只留 3 硬拦"的边界。
- 回滚点 `_tmp_session/rt-gate/preimage/`；证据 `_tmp_session/rt-gate/REPORT.md`。
## Lead 复核（2026-10-28 01:1x · §11 独立复跑）

- 独立复跑：`test_agc_rectify_rt_gate.py` **69/69 PASS（exit0）**；`config_selfcheck` error0/warn0；`tools/wf/check.py` **GREEN**（复用登记已补）；`sim_cli` **118.12 s / 0.0300 m / 74**（与改前逐项相同）；`test_stepper` **117/118**（唯一红 12.11 = 基线存量，ttl=0 落地后未再翻红）。
- 配置落地：`config/robot.yaml` 已加 `step.layout_ttl_s: 0.0` ⇒ `gate_policy=warn`（实测），A 区降级在车上生效。
- 见 D-055；未 commit / 未部署 / 未连车。

