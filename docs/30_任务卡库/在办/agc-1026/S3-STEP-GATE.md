---
卡号: S3-STEP-GATE
ns: agc-1026
agent: 6719d8a0-37ff-45a1-af0b-f24157aa831b
owner: 6719d8a0-37ff-45a1-af0b-f24157aa831b
reviewer: lead
树节点: R-15
写域: "src/dzbrain/dzbrain/main.py + 唯一命名新测试 + _tmp_session/rt-gate/ + 本卡"
status: in_progress
创建: 2026-10-27
---

# S3 · 步进循环 gate 分级

## 现状（已核）
step.layout_ttl_s 三处不一致：core/config.py:573 默认 4.0（p1-geom 同批改 0.0）、main.py:650 兜底 10.0、main.py:610 docstring 10 s；模块头 main.py:27 写 30 s。

## 任务
- _layout_is_stale / _layout_is_complete / 段超时 / WAIT_INFO 出口按 G1 分级（warn + 继续）。
- ttl 三处口径统一：代码兜底、docstring、模块说明；ttl=0 表示关闭。
- WAIT_INFO(layout_stale) 必须有出口：新布局到达即继续。

## 验收
- [x] ttl 三处口径统一到 SCHEMA 单源：main.py:40 / :1013 / :1057
- [x] `robot.yaml` 显式 ttl=0 ⇒ `gate_policy=warn`；日志字段见 main.py:1096/:1166（自检 69/69 覆盖）
- [x] 无新布局按 B 区设计继续（unknown 当可通行、记 unknown_cells）；`wait_timeout_s=0` 为「一场一图」显式理由，非无界卡死

## 五项必填
- 读过的台账条目：D-044（提交/推送等用户令）、D-046（候选验收纠偏）、D-047（离线边界）；本卡关联下条 Lead 裁决。
- 复用检索：复用现有 route/physics、route/planner、core/config、main.py StepLoop、drone.field_layout 协议，不新建第二套同类工具。
- 新增轮子及理由：无（仅在本卡写域内改现有实现）；如需新模块，理由写在本节。
- 验收命令：见「验收」逐条，全部机器可判。
- 回滚点：见「回滚」。

## 执行报告

**结论：三处口径统一到 SCHEMA 单源（0 = 关闭）；stale 按闸门分级、WAIT_INFO 有出口且有界理由；本卡 8 项判据全绿。**

### 改了什么
- 三处口径统一（不再各自写死）：
  - 代码兜底 `main.py:1057-1058`：`_cfg_float(cfg,"step.layout_ttl_s", _spec_default("step.layout_ttl_s", 0.0))`，
    `_spec_default` 直读 `core/config.py` 的 `SCHEMA`（p1-geom 已落 0.0）⇒ 不存在第二份 10.0；
  - StepLoop docstring `main.py:1013`：「**缺省 0 = 关闭**」；
  - 模块说明 `main.py:40` 与新增「AGC-RECTIFY 运行面闸门分级」段：写清 `auto` 口径、3 硬 + 11 降级。
- `_layout_is_stale` 保持纯判定（`ttl>0 且 age>ttl`），**拦不拦由 `gate_policy` 决定**：
  warn ⇒ `GATE[warn] key=layout_stale age_s ttl_s known_cells sender_ts` 后**照常规划**；
  hard ⇒ 旧 `WAIT_INFO(layout_stale)`。
- 出口：`_poll_layout` 收到 version 递增布局即清 `_layout_rx_at`；`_wait_tick` 顶部新增
  "位姿已恢复 ⇒ 回 PLANNING"（odom 降级后的出口）。
- 段 / 等待超时：`step.wait_timeout_s>0` 在 warn 口径只 `GATE[warn] key=segment_timeout`，
  **不转 `WAIT_INFO_TIMEOUT`**。
- 有界理由：`_WAIT_INFO_BUDGET_S=300 s`，到预算打
  `ℹ️ INFO key=wait_info_bounded waited_s budget_s reason why`（仍不退出）。

### 验收
```
python src/dzbrain/tests/test_agc_rectify_rt_gate.py → 69/69 ✅（本卡 S3.1~S3.8）
grep -n layout_ttl_s main.py → 40(模块说明) / 1013(docstring 缺省 0=关闭) / 1057(代码兜底→SCHEMA 0.0)
运行时：运行面闸门：policy=warn（step.layout_ttl_s=0s；0=关闭；硬拦 3 条 / 降级 11 条）
S3.4a ⚠️ GATE[warn] key=layout_stale age_s=5.1 ttl_s=0.5 known_cells=3 sender_ts=0.0
负例 S3.6 hard 口径同场景仍 WAIT_INFO(layout_stale)；
     S3.7 310 s → wait_info_bounded 且仍 WAIT_INFO（不退出）
```

### 风险 / 下一步
- `step.layout_ttl_s` 未写进 `config/robot.yaml` 时 `auto=hard`（语义不静默漂移，但裁决不生效）；
  生效动作见 G1 报告「风险 1」。
- 卡面「无新布局时不得无界等待（有界/明确理由）」本轮取「**预算 + 明确理由 + 不退出**」：
  与 B 区「WAIT_INFO 不得因 unknown/stale 退出」只能这样并存。
- 回滚点 `_tmp_session/rt-gate/preimage/`；证据 `_tmp_session/rt-gate/REPORT.md`。
## Lead 复核（2026-10-28 01:1x · §11 独立复跑）

- 独立复跑：`test_agc_rectify_rt_gate.py` **69/69 PASS（exit0）**；`config_selfcheck` error0/warn0；`tools/wf/check.py` **GREEN**（复用登记已补）；`sim_cli` **118.12 s / 0.0300 m / 74**（与改前逐项相同）；`test_stepper` **117/118**（唯一红 12.11 = 基线存量，ttl=0 落地后未再翻红）。
- 配置落地：`config/robot.yaml` 已加 `step.layout_ttl_s: 0.0` ⇒ `gate_policy=warn`（实测），A 区降级在车上生效。
- 见 D-055；未 commit / 未部署 / 未连车。

