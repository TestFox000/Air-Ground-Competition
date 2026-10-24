---
来源: _tmp_session/TRACK-FIELD-B5-20261021/REPORT.md, _tmp_session/TRACK-FIELD-B5-20261021/guard_replay.md, _tmp_session/TRACK-FIELD-B5-20261021/slip_diagnosis.md, _tmp_session/TRACK-FIELD-B5-20261021/MEASURE_SOP.md, dzrobot_v2/docs/TEAM_STATUS.md
取证: 档
更新: 2026-10-21
取代: 无
status: in_progress
ns: track-1021
agent: motion-auditor（身份待核实）
树节点: T21-1
写域: docs/30_任务卡库/在办/track-1021/
---

# TRACK-FIELD-B5-20261021 · 阿克曼量测 + 航向守卫复算 + X05 电流分诊（task-8）

## TL;DR（给 agent 看的）

- 目标：把 T-B5 现场项收敛成 5 项量测（最小转弯半径 / PIVOT 形态 / 实测轮距 / ticksPerMeter 2.10 m 推车 / ticksPer2PI 二分），并离线复算现行航向守卫。
- 当前状态：**本地离线交付完成，待 Lead 验收；未上车、未采数、未改生产代码/配置**（`_tmp_session/TRACK-FIELD-B5-20261021/REPORT.md:4`）。
- 最大坑：A2 的原始 5 Hz 帧与 10-19 L8 原始日志**都缺**，所以"守卫是否误触发"目前**判不了**，只能写到"证据不足 + 物理不可能/可能"的边界。

## 1 卡面

| 项 | 值 |
|---|---|
| 目标 | ① 5 项地面量测作业包（MEASURE_SOP）② 现行守卫离线复算 ③ X05 电流可行性 ④ C1_1 2.6 cm 分诊 |
| owner | motion-auditor（角色名；具体 agent 身份未核实〔未〕） |
| 状态 | 离线完成，待验收 |
| 写域 | `_tmp_session/TRACK-FIELD-B5-20261021/**`（**注意**：task-8 原文写 `dzrobot_v2/_tmp_session/…`，实际落在根目录，REPORT:99 已登记差异） |
| 交付物 | MEASURE_SOP.md、measure_record_template.csv、car_capture.py、replay_guard.py、guard_replay.md、slip_diagnosis.md、replay_out/、MANIFEST.json（17 文件） |

## 2 四条关键结论（每条都能回到源码/日志）

| # | 结论 | 证据 |
|---|---|---|
| 1 | 现行守卫原样复算：`limit=max(60,525×v)`，v<0.03 用 45；单帧 \|Δyaw\|>30°；连续 3 帧；20 可信帧自动复位 | `REPORT.md:28-29`；代码 `tasks/localization.py:97-118,664-844` |
| 2 | **A2（10-18，79.8°/s @ v=0.19）现行不触发、旧阿克曼口径触发；但 79.8°/s 需要 δ≈47.7~49°（手册 30±2°）⇒ 物理上不是正常阿克曼转弯** | `REPORT.md:30-34`；增益 525=420×1.25，420=79.8/0.19 ⇒ **用待分类事件做标定再证明它不触发 = 循环论证** |
| 3 | L8（10-19，111.1°/s @ v=0.06）**证据缺失**：全树只有文档转述，无 10-19 原始日志 | `REPORT.md:36-38`；10-20 的对照日志在 `_tmp_session/TRACK-FIELD-R2-20261020-01/evidence/car_logs/3b_localization.log:1281-1340` |
| 4 | **X05 电流当前做不了**：上行帧 34 B 只有轮速/电压/IMU/云台负载，无驱动电流/PWM；`current_vel_state` 是命令回显 | `REPORT.md:44`；`slip_diagnosis.md` |

另：C1_1 的 2.6 cm **不能单判 ticksPerMeter 偏大**——0.0257 = 0.0174（到点后滑行）+ 0.0083（停短），纯尺度假设（3.8% 高报，`ticksPerMeter≈3405`）也能拟合（`REPORT.md:45-46`）。

## 3 卡在哪

| # | 阻塞 | 需要什么 |
|---|---|---|
| 1 | PIVOT 形态未实测（代码 `tracker.py:869` 带 0.08 m/s = 前进弧线，而 `localization.py:100` 当差速可原地转） | 现场 S3.0：架空 + 地面 v=0/v=0.08，判"真自转 vs 阿克曼弧" |
| 2 | A2 原始 5 Hz 帧缺失 | 车端 `3_localization.log` 或原始 bag；或复跑 A2 同段 |
| 3 | 10-19 L8 原始日志缺失 | 车端 `7_path/3_localization/bag`（21:22 已在车端找回并拉回，见下） |
| 4 | 实测轮距未量（等价轮距 0.130 是编码器口径，不是物理轮距） | MEASURE_SOP §4.2/4.3 |

> 21:22 更新：车端 `robot_logs` 已找到 10-19 P1 原始日志并拉回 `_tmp_session/TRACK-FIELD-B5-20261021/car_pull_1019/`（`3_loc_p1.log` 16122 帧 5 Hz 带时间戳、`7_path_p1.log` 含 `yaw 111.1°/s > 60（v=0.06）` 原文）；**A2 79.8 原始帧本地/车端均缺**（`dzrobot_v2/docs/TEAM_STATUS.md:93`）。

## 4 下一步（建议 Lead 派卡顺序）

1. **S3.0 量测包一次执行**（PIVOT 形态 + 2.10 m 推车 + 定圆）——需现场窗口/人工/授权。
2. 用拉回的 10-19 日志补 L8 复算（不必等现场）。
3. X05 走外挂电流钳（不改固件），或固件加字段（冻结区另立卡）。
4. R4A/R4B1 的守卫包络与 PIVOT done/abandon 语义，**等 S3.0 数据后再定**。

## 5 依赖

- 上游：D-027 阿克曼硬件证据（`docs/agent_handoff/36_R2复核_R3补测与收尾计划.md:287`）；R4A/R4B1 是建议卡、**未开工**（`dzrobot_v2/docs/TEAM_STATUS.md:29`）。
- 下游：S3.0 数据 → 守卫包络 → 才允许谈提速档（`_tmp_session/paper50/三大优化方案.md:218`）。

## 相关文件

- `_tmp_session/TRACK-FIELD-B5-20261021/REPORT.md`（主报告 128 行）
- `_tmp_session/TRACK-FIELD-B5-20261021/MEASURE_SOP.md`（5 项作业包）
- `_tmp_session/TRACK-FIELD-B5-20261021/guard_replay.md` + `replay_out/`（复算产物）
- `_tmp_session/TRACK-FIELD-B5-20261021/slip_diagnosis.md`（X05 审计）
- `_tmp_session/TRACK-YAW-20261021/REPORT.md`（同一守卫的另一种改法：阿克曼包络 k=1.6）
