---
卡号: G1-DEGRADE
ns: agc-1026
agent: 6719d8a0-37ff-45a1-af0b-f24157aa831b
owner: 6719d8a0-37ff-45a1-af0b-f24157aa831b
reviewer: lead
树节点: R-14
写域: "src/dzbrain/dzbrain/main.py + 唯一命名新测试 + _tmp_session/rt-gate/ + 本卡"
status: in_progress
创建: 2026-10-27
---

# G1 · 自检/契约降级（运行面）

> 用户 2026-10-27 四区共识：运行面拦停型互锁降级为 warn+日志；离线交付面不降级。

## 边界
- 运行面（发车前 / 步进循环）：降级为 warn + 日志。
- 离线交付面（§4 门禁 / check.py / 单测）：**不降级**。

## 只保留 3 条硬拦
1. 硬件急停 / 遥控接管
2. 激光 / 开火联锁（含红蓝面未定）
3. 软件边界保护（边界 = 地面线）

## 降级清单（逐条 key + 日志字段）
布局不完整 / 布局新鲜度 / 足迹侵入 / 重复访问 / 停滞 drift_stop / 位姿就绪 / odom 看门狗 / 段超时 / 完成单缺失 / 靶未锁 / 云台状态。

## 验收
- [x] V1 注入 5 违约 → 各自 GATE[warn]、40 拍后相位 [DRIVING, PLANNING]、gated_continues=10（不停死）
- [x] V2 硬拦调用点 3 处：main.py:738 fire_interlock / :1892 hw_estop / :1909 field_boundary
- [x] V3 离线门禁全绿（doc_policy GREEN / check.py GREEN / links 0 / numbers 0；anchors 新增红 6 全在他人卡）

## 回滚
main.py 改前 preimage 存 _tmp_session/rt-gate/preimage/；禁 git checkout。

## 五项必填
- 读过的台账条目：D-044（提交/推送等用户令）、D-046（候选验收纠偏）、D-047（离线边界）；本卡关联下条 Lead 裁决。
- 复用检索：复用现有 route/physics、route/planner、core/config、main.py StepLoop、drone.field_layout 协议，不新建第二套同类工具。
- 新增轮子及理由：无（仅在本卡写域内改现有实现）；如需新模块，理由写在本节。
- 验收命令：见「验收」逐条，全部机器可判。
- 回滚点：见「回滚」。

## 执行报告

**结论：G1 机制落地并自证 69 项（含本卡 14 项）；生产是否生效取决于配置一行（见「风险」）。未 commit / 未部署 / 未连车。**

### 改了什么（写域内）
- `main.py` 新增模块级闸门表 `GATE_SPECS`：**3 硬拦**（`hw_estop` / `fire_interlock` / `field_boundary`）
  + **11 降级**（`layout_incomplete` / `layout_stale` / `footprint` / `revisit` / `drift_stop` /
  `pose_ready` / `odom_watchdog` / `segment_timeout` / `done_missing` / `target_unlocked` /
  `gimbal_status`），每条带固定日志字段与信号来源（一处可查、可 grep）。
- `GateLedger`：`GATE[warn] key=… 字段=…` 结构化日志 + 按 key 计数（进归档 `warn_counts`）+ `trace.jsonl`；
  `info()` 承载"只记不拦"（unknown_traversed / info_lead_low / wait_info_bounded）。
- 降级落地：布局不完整（`plan_segment_from_layout` 三处判据降 warn 且照常规划）、足迹（配置层
  `planner.footprint_gate: error → warn` + 段级 `footprint_diagnostic` 只记）、drift/重复访问
  （`classify_gate_reason` 文本映射 → warn + 停段后**立即重规划**）、位姿就绪门（超时不再 exit 5）、
  odom 看门狗（不再 exit 5，改停段 + 等位姿恢复即继续）、段超时（不转 WAIT_INFO_TIMEOUT）、
  完成单缺失（重规划）、靶未锁 / 云台状态（`P_AIM` 只读观测）。
- 硬拦只剩三个 `hard_stop()` 调用点：`_hard_estop`（`bridge.manual_override()` / `P_SAFETY` 接管原因）、
  `_hard_boundary`（信念越地面线）、`assert_fire_interlock`（发车前查 `aim.fire_only_blue` /
  `aim.max_shots`，不在位 ⇒ 退出码 7，不进任务组）。

### 验收（机器判据）
```
python src/dzbrain/tests/test_agc_rectify_rt_gate.py   → 69/69 全部通过 ✅（exit 0）
V1 五违约：layout_incomplete / footprint / drift_stop / revisit / target_unlocked 五条 warn 齐；
          40 拍注入后相位集合 = ['DRIVING','PLANNING']，gated_continues=10
V2 grep：hard_stop( 调用点 = 3（main.py:738 / :1887 / :1904）=
          fire_interlock / hw_estop / field_boundary
负例：G1.10 hard 口径过期仍 WAIT_INFO(layout_stale)；G1.11 unknown 3 拍仍 SAFE_STOP；
      G1.12 遥控接管→SAFE_STOP+hard warn；G1.13 越界→SAFE_STOP+hard warn；
      G1.14 关 fire_only_blue→拒绝发车；G1.2c 空 known / target 未知仍被协议层拒收
```
离线门禁：`_run_full_regression.py` 与基线逐文件比对，**本卡 0 新增红**（唯一非本卡差异见
`_tmp_session/rt-gate/REPORT.md` §2 V7）；`tools/wf/check.py` **RED(1)** = `core/run_archive.py`
未登记（登记落 Lead 的 `docs/复用清单.md`，我不写）。

### 风险 / 下一步
1. **生产当前仍是 hard**：`config/robot.yaml` 无 `step.layout_ttl_s` 行 ⇒ `auto` 判 hard（实测
   `Config.load('config/robot.yaml')` → `policy=hard explicit_ttl=False`）。生效动作 = 配置 owner 在
   `step:` 段加 `layout_ttl_s: 0.0`（或 `gate_policy: warn`）；未表态时启动会打一行
   "⚠️ 运行面闸门仍是 hard … 目标态为 warn"，不会静默。
2. 真车体足迹侵入样本仍无（只证机制与配置联动）；靶未锁 / 云台状态是只读观测，不参与决策。
3. 回滚点：`_tmp_session/rt-gate/preimage/src/dzbrain/dzbrain/main.py`（sha256 `50251579…c57b`
   逐字节等于改前，禁 `git checkout`）。证据全文：`_tmp_session/rt-gate/REPORT.md`。
## Lead 复核（2026-10-28 01:1x · §11 独立复跑）

- 独立复跑：`test_agc_rectify_rt_gate.py` **69/69 PASS（exit0）**；`config_selfcheck` error0/warn0；`tools/wf/check.py` **GREEN**（复用登记已补）；`sim_cli` **118.12 s / 0.0300 m / 74**（与改前逐项相同）；`test_stepper` **117/118**（唯一红 12.11 = 基线存量，ttl=0 落地后未再翻红）。
- 配置落地：`config/robot.yaml` 已加 `step.layout_ttl_s: 0.0` ⇒ `gate_policy=warn`（实测），A 区降级在车上生效。
- 见 D-055；未 commit / 未部署 / 未连车。

