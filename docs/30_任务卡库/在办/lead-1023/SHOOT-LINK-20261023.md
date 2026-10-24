---
来源: 2026-10-23 用户指令「结合循迹+射击」；方案 v2 _tmp_session/lead-1023/SHOOT-PLAN-v2-20261023.md
取证: 实/机检（mission.py:3445-3610、aim.py:1283-1315、config.py:390-401、planner.py:4717-4830；22:1x 实读）
更新: 2026-10-23
取代: 无
status: in_progress
ns: lead-1023
agent: lead
树节点: L-4
写域: src/dzbrain/dzbrain/tasks/mission.py；src/dzbrain/dzbrain/tasks/aim.py；src/dzbrain/dzbrain/core/config.py；config/robot.yaml；src/dzbrain/tests/；_tmp_session/lead-1023/
---

# SHOOT-LINK-20261023 · 循迹×射击合流（账本/跳站判据/几何先验）

- owner: **lead**
- created: 2026-10-23 22:20
- 非目标: 不改 `dzrknn_dataprocess` C++（三核推理由另一会话负责）；不动 det_node；不改路线几何本身（只补第二停站位置）

## 0 现状（实读，非回忆）

| 事实 | 证据 |
|---|---|
| 命中即封顶：打中 1 发 → `hit_latched` → `fire_ready`=False | `aim.py:853-860` |
| 完成判据：`shoot_count` 增量 ≥ `shots_per_stop`(10) 才算"打完" | `mission.py:3548-3553`、`config/robot.yaml:713` |
| 云台单靶上限 3 发 | `config.py:377` `aim.max_shots_per_target: 3` |
| ⇒ 到不了 10 ⇒ 干等到站 30 s，且 `_hit_this_stop` 恒 False | `mission.py:3456-3463`、`3588` |
| 双停站早退/跳过逻辑**已存在** | `aim.red_early_leave_s`、`mission.skip_second_stop_after_hit` |
| 但跳过判据是"打出去了"，不是"看到蓝" | `mission.py:3588` |
| 本地路线只有 1 个射击停站 `B4*` | `config/robot.yaml:581`（68 token） |
| 车端连通性：22:1x `ssh 10.15.127.188` 超时 | 需现场复核车端路线版本 |
| **命中计数是"取走清零"**：`take_hits()` 读一次就归零，`aim.py:854` 与 `mission.py:3749` 两个消费者抢同一个数 | `ros_io.py:421-426` |
| ⇒ Mission 在停站中**拿不到命中增量**，FINISH 统计里的 hits 也不可信（被 aim 先取走） | 同上 |

## 1 方案

**A1 完成判据三选一**（`mission.py` + `aim.py`）
- 前置：AimTask 新增黑板键 `P_AIM_HIT = {"total": 累计命中, "at": t}`（唯一消费者是 aim，写累计值；
  Mission 只读快照，不再调 `take_hits()`）——否则两处抢清零、谁都算不准。
- 条件：① 命中增量 ≥1（读 `P_AIM_HIT`）；② 发数增量 ≥ min(`shots_per_stop`, `aim.max_shots_per_target`)；③ 到站 30 s 死线。
- 任一满足 ⇒ `_complete_stop`，并置"本站已打出去"（用于跳第二站）。
- 无裁判（bench）时退化为 ②③。

**A2 跳过第二站判据**（新增键 `mission.skip_second_stop_on_blue: true`）
- 锁定蓝（`aim` 状态 cls==1）持续 ≥1.5 s ⇒ 离站后跳过下一射击停站（不管是否打出去）。
- 保留原"已出弹/已命中 ⇒ 跳"作为第二触发；红面早退仍强制去下一站。
- 早退/跳过都写日志 + 黑板，便于复盘。

**A3 第二停站落进路线**
- 手动路线补 token（或打开规划器补标），两侧各一个停站；须过 `route_check` 0 error + 对侧视线不挡。
- 与 A2 配套：单停站路线时 A2 自动 no-op（现有实现即如此）。

**B1 几何先验（Python）**
- 停站点位姿 + `map.yaml` 靶格 ⇒ 靶板中心相对车体方位/俯仰角，写黑板；云台优先朝该方向扫。
- 不改 C++，不引入绝对预置。

### 1.1 用户 2026-10-24 最终口径（**已落地**，覆盖上文"命中即走"假设）

- 射击站逻辑：**本侧看到蓝面 ⇒ 打满 10 发**；**看不到蓝（红面/无靶）⇒ 去第二射击停站打满 10 发**。
  不做"打中一发就走"：命中**不再**提前停火，停火只由预算门（10 发）与到站 30 s 死线决定。
- 落地三处：
  1. 新键 `aim.stop_on_hit`（SCHEMA `_B` 默认 True）；`config/robot.yaml` 设 **false**
     ⇒ `hit_latched` 只记账/发事件，**不再闭锁** `fire_ready`（`aim.py` 硬门 5 加条件）。
  2. 第二射击停站 **F4***（`config/robot.yaml:577`）：与 B4* 分居靶板两侧
     （B4 在西、F4 在东；到靶格 D5/D6 均 0.783 / 0.990 m，落在 `[0.78,1.02]` 窗口内）。
     F4 **本就在现役 68 点路线上** ⇒ 只加星、零路径改动。
  3. 红面早退（`aim.red_early_leave_s=3.0` → `P_AIM_RED`）与"发数达标跳第二站"
     （`mission.skip_second_stop_after_hit`）沿用现有实现，本轮未改。
- 现场约束（用户口径）：**每一次真出光测试前必须手动重置车侧打击系统**
  （`aim.max_shots_total`/云台 `max_shots` 都是进程内累计 10 发；不重置第二趟就是 0 发）。
- 已知边界：蓝面已确认但**不足 10 发**（靶出框/弹药耗尽）时，离站后**不会**跳第二站
  （跳过判据 = 发数增量达标）。要不要补"确认蓝 ⇒ 跳"，等现场首跑后再定。

## 2 验收（2026-10-24 早实测）

| 命令 | 结果 |
|---|---|
| `python src/dzbrain/tests/test_aim.py` | **136/136 通过**（新增 5b 组 5 条：命中不闭锁 / 锁存仍记账 / 继续发许可 / 对照旧行为） |
| `python tools/config_selfcheck.py` | **exit 0，0 error / 0 warn**；路线 **68 点**、射击停站 `['F4*','B4*']` |
| `python tools/route_check.py` | **exit 0 / 0 error**（1 warn = 急转弯 23 处，存量） |
| `python tools/wf/check.py` | **GREEN**（4 个冻结区脏文件全部已登记，本卡新增 3 条 in_flight） |
| 全量 `test_*.py` | **13 套红**，抽查到的失败项全部是**地图漂移基线**（测试仍钉第五张图 44 点 / `D3*`+`G3*` / K9）；**本轮未重做基线 A/B**，如实记录 |
| 台架 / 实车 | **未做**：车 22:1x 起离线；出光测试需用户现场重置打击系统 |

## 3 回滚

- 全部改动在 `mission.py/aim.py/config.py` 三个文件 + 新配置键（默认值与现行为一致处逐位不变）；
- 回滚：`git checkout -- <file>`；配置键删除即回默认。

## 4 文档变更

- 新增：本卡；`_tmp_session/lead-1023/SHOOT-PLAN-v2-20261023.md`（过程稿，不进库）
- 修改：无
- 归档：无
