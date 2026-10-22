---
来源: _tmp_session/TRACK-*/CARD*.md, _tmp_session/TRACK-*/REPORT*.md, dzrobot_v2/_tmp_session/*/REPORT.md, _tmp_session/overnight/*.md, _tmp_session/paper50/三大优化方案.md, _tmp_session/PROJECT_STATUS_20261021.md, dzrobot_v2/docs/TEAM_STATUS.md
取证: 档
更新: 2026-10-21
取代: 无
---

# 时间线 · doc-cards（任务卡库编纂）

## TL;DR（给 agent 看的）

- 这张表回答"项目是怎么走到今天的"：24 条关键事件，从 10-18 的测试红分类到 10-21 的 B1 交报告。
- 读法：先看"对现在还有什么用"这一列，它直接告诉你哪条结论今天仍被引用。
- 最大坑：表里 21:00 之后的三个数字都是卡内自报〔未〕，别当独立复核结果。

> 一行一条，覆盖 2026-10-18 → 2026-10-21。证据列是 `路径:行`，可直接回原文核对。

| 日期 | 事件（≤40字） | 证据 | 对现在还有什么用 |
|---|---|---|---|
| 2026-10-18 | v4.0-closeout 提交 f883c52：位姿就绪门 + ROS_LOST 看门狗 | `dzrobot_v2/_tmp_session/v4.0-closeout/VERIFY.md:33-44` | 全库唯一**真提交**的卡；exit 5/6 语义沿用至今 |
| 2026-10-18 | t35 把 46 行 FAIL 拆成 23 条唯一失败 | `dzrobot_v2/_tmp_session/t35/REPORT.md:8-13` | 其中 6 条真缺陷 = 今天 B1 卡的唯一目标 |
| 2026-10-18 | t34 复核 11/11 stop_kind 一致但全场 0 发 0 中 | `dzrobot_v2/_tmp_session/t34/REPORT.md:11-18` | "语义对齐≠打中"，T4 打靶归因的起点 |
| 2026-10-18 | p3_self_evolve 留出集驳回 hybrid ⇒ 不晋级 | `dzrobot_v2/_tmp_session/p3_self_evolve/REPORT.md:11-17` | D-016 不切引擎的可复跑证据 |
| 2026-10-18 22:45 | 夜间作业开始（9 篇 md，无 CARD 结构） | `_tmp_session/overnight/PLAN.md:1-4` | 非卡产物，只在归档页登记 |
| 2026-10-18 23:0x | 审计发现"仓库=第六图 / 车端=第七图" | `_tmp_session/overnight/NIGHT_AUDIT.md:33-39` | 后来 Top3 失败原因 #3 的源头 |
| 2026-10-19 | task-44 改差速包络 + 闩锁可恢复（未部署） | `dzrobot_v2/_tmp_session/task-44/REPORT.md:9-11` | 525 增益的循环论证问题今天仍未被推翻 |
| 2026-10-19 | t_ae_contract 四层 odom 诊断 + 布局 TTL 门 | `dzrobot_v2/_tmp_session/t_ae_contract/REPORT.md:12-21` | "与旧裁决冲突时留开关回退"的范式 |
| 2026-10-19 | t_d1b 加 done_seq 闩锁，但 dzlink 白名单未转发 | `dzrobot_v2/_tmp_session/t_d1b/REPORT.md:41` | 空地协同 P0-12 仍开放 |
| 2026-10-20 18:20 | R0 开卡：只复现 PIVOT/ALIGN 绕过限速 | `_tmp_session/TRACK-R0-20261020-01/CARD.md:4-6` | "先复现再修补"的卡序范式 |
| 2026-10-20 18:38 | R0 结论 9 PASS / 4 REPRODUCED / 0 INVALID | `dzrobot_v2/docs/TEAM_STATUS.md:13` | ALIGN 不可达、PIVOT 可达的口径来源 |
| 2026-10-20 19:19 | R1 开卡：发布点单点钳位 14 行 | `_tmp_session/TRACK-R1-20261020-01/CARD.md:8` | tracker 候选 hash e9bff6f0 的来源 |
| 2026-10-20 19:46 | R2 上实车：F1b 50 Hz / F2 0.6961 m / F3 共线 | `_tmp_session/TRACK-FIELD-R2-20261020-01/review.md:33-40` | 全库唯一一组真实静止+直线数据 |
| 2026-10-20 20:18 | R2 被判 needs_revision（3 条阻断） | `_tmp_session/TRACK-FIELD-R2-20261020-01/review.md:6,17-19` | Top3 失败原因 #1 的定义性案例 |
| 2026-10-20 21:13 | R3 收口：实车 0 趟，证据不足 | `_tmp_session/TRACK-FIELD-R3-20261020-01/REPORT.md:4-6` | "现场门不齐就收口"的纪律来源 |
| 2026-10-20 21:31 | 部署准备卡完成（候选≠已发布） | `_tmp_session/TRACK-DEPLOY-PREP-20261020-01/CARD.md:9` | R1 至今未部署的账在这里 |
| 2026-10-21 18:50 | 《三大优化方案》出具（242 行） | `_tmp_session/paper50/三大优化方案.md:3` | B1/B5/T-A0/T-C0 全部任务卡的母本 |
| 2026-10-21 19:52-20:45 | R3B 5 趟有效后车端整机掉电 | `_tmp_session/PROJECT_STATUS_20261021.md:45-62` | C0 余量 0.8 mm、C1 只有 1/3 趟的口径 |
| 2026-10-21 20:57 | D-028 把三大方案定为执行基线 | `_tmp_session/PROJECT_STATUS_20261021.md:84` | 解释 21:00 后为什么突然开 4 张离线卡 |
| 2026-10-21 21:05 | 现状快照：求解率 43.75%、13 套红 | `_tmp_session/PROJECT_STATUS_20261021.md:12,74-80` | 任何"项目已完成"的说法都要先过这一页 |
| 2026-10-21 21:17-21:27 | T0 基线冻结：24 绿 / 13 红，与 D-023 逐套一致 | `_tmp_session/TRACK-BASE-20261021/CARD.md:8,20-26` | 回归基线的唯一现行数字 |
| 2026-10-21 21:22 | B5 复核通过 + 10-19 原始日志从车端找回 | `dzrobot_v2/docs/TEAM_STATUS.md:93` | L8 复算不必等现场了 |
| 2026-10-21 21:3x | B1 交报告：47/64 + test_autoplan 171/171 | `dzrobot_v2/_tmp_session/TRACK-PLAN-B1-20261021/REPORT.md:8,29-36` | 求解率问题首次有可验收的解 |

## 边界

- 本表只记"对卡库有长期价值"的事件；同一天的会议、消息、试跑细节不入表。
- 21:00 之后的三个数字（47/64、171/171、24绿13红）都是**卡内自报**，编纂者未复跑〔未〕。
