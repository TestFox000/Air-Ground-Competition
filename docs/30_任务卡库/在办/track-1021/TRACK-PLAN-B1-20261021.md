---
来源: dzrobot_v2/_tmp_session/TRACK-PLAN-B1-20261021/REPORT.md, _tmp_session/PROJECT_STATUS_20261021.md, dzrobot_v2/docs/TEAM_STATUS.md, _tmp_session/paper50/三大优化方案.md
取证: 档
更新: 2026-10-21
取代: 无
status: in_progress
ns: track-1021
agent: track-lead（身份待核实）
树节点: T21-3
写域: dzrobot_v2/src/dzbrain/dzbrain/route/, dzrobot_v2/src/dzbrain/tests/
---

# TRACK-PLAN-B1-20261021 · T-B1 规划器目标函数换口径（task-7）

## TL;DR（给 agent 看的）

- 目标：把 tour 的"最短巡回+硬约束"换成"先最大化成功任务数、再最小化时间"，并让**降级显式化**；第七图必须出 C9*+C5* 双停站。
- 当前状态：**离线实现与验收完成，未 commit / 未部署 / 未上车**（`dzrobot_v2/_tmp_session/TRACK-PLAN-B1-20261021/REPORT.md:4`），等 Lead 验收。
- 最大坑：**默认仍是 legacy**，新口径必须显式 `--objective tasks`；只看"64 图 47/64"会误以为车端已变好。

## 1 卡面

| 项 | 值 |
|---|---|
| 目标 | tour 新口径 `objective=tasks`；第七图双停站；64 图 ≥70%；6 条真缺陷**改代码**（禁重钉） |
| owner | implementer（唯一生产写者；本编纂者未核实具体 agent 身份，仅有角色名） |
| reviewer | 待 Lead/独立复核（REPORT 未附 reviewer 段落）〔未〕 |
| 状态 | **验收通过（离线，未部署）**（2026-10-22 Lead 复跑 A/B/C） |
| 写域 | `src/dzbrain/dzbrain/route/planner.py`、`tools/plan_route.py`、`src/dzbrain/tests/test_autoplan.py`、卡目录 |
| 冻结登记 | `planner.py` in_flight，expires **2026-11-04**（`dzrobot_v2/docs/TEAM_STATUS.md:22`） |

## 2 已交证据（卡内自报〔档〕，未复跑〔未〕）

| 指标 | 改前 | 改后 |
|---|---:|---:|
| 64 图全任务覆盖 | 28/64 = 43.75% | **47/64 = 73.44%**（≥45 图达标） |
| 出 token（含降级，无解=0） | 29/64 | **64/64** |
| plan-failed | 35（Lead 基线记 36，口径差 1 图） | 0 |
| test_autoplan | 148/171（23 红） | **171/171 exit 0** |
| 单次规划 | avg 519.4 ms | avg 386.7 / max 760.5 ms |

出处：`dzrobot_v2/_tmp_session/TRACK-PLAN-B1-20261021/REPORT.md:29-36`（压测）、`:56-60`（test_autoplan）、`:64-66`（耗时）。第七图 `map7_t24`：66 token / 23.45 m / `shoot_stops=["C9","C5"]` / route_check exit 0（同文件 `:47-49`）。

改动量与改后哈希（`:14-16`）：`planner.py` +501/−20 `c7f237b4…`；`plan_route.py` +22/−3 `df856f69…`；`test_autoplan.py` +83/−38 `3448fcd5…`。

## 3 卡在哪（三个真问题，不是没做完）

| # | 问题 | 说明 | 出处 |
|---|---|---|---|
| 1 | **15 张随机图物理不可达终点** | 靶板/障碍封死 K 列桥洞，任意 5×6 桥洞组合都不可达；只能出降级串并如实让 route_check 红 | REPORT:146 |
| 2 | **2 张图无合法停站** | index34 无单停站能覆盖 A4/A5；index54 窗口内唯一候选是终点 N5 | REPORT:147 |
| 3 | **degraded 未进遥测** | 字段落在 `route/planner.PlanResult`；`mission.PlanOutcome` 属写域外，车端遥测仍只有 `shoot_stops/second_stop_added` | REPORT:148 |

口径提醒：**默认 legacy 的 64 图求解集合与改前逐图一致（28/64）**（REPORT:18,137）——本卡没有偷偷改默认行为。

## 4 下一步

1. Lead 验收：核 3 份改前原件哈希（REPORT:139-141）与 `stress_after.json` 逐图字段。
2. 决定"degraded 进遥测"是否单开一张 `mission.py` 卡（同 planner/mission 写域，必须串行，勿与 B 包合入撞车）。
3. 若采用新口径：写进台账并明确"默认是否切换"，否则车端永远跑 legacy。

## 5 依赖与风险

- 依赖：无现场条件；纯离线（不占现场窗口）。
- 风险：本卡只证"规划层出串"，**不含执行层**；sim 不建模 PIVOT/限速/Safety（`docs/agent_handoff/36_R2复核_R3补测与收尾计划.md:50`）。
- 与三包关系：B 包（外部副本）目标是同一处 tour 单停站缺陷，**合入时必然与此卡同文件**，需三方合并，不能整目录覆盖。

## 6 Lead 验收结论（2026-10-22）

| 项 | 预期 | 复跑结果 | 退出码 | 证据（归档包内路径） |
|---|---|---|---|---|
| A · 64 图压测 `--objective tasks` | 47/64，plan_failed=0 | **47/64 = 73.44%**，plan_failed 0，plan_ms avg 486.9 / max 981.4 | 0 | `dzrobot_v2_tmp_session_20261022.tar.gz::_tmp_session/TRACK-PLAN-B1-20261021/lead_accept/lead_A_stress64_stdout.txt` |
| B · 第七图 `map7_t24` 双停站 | 66 token / 23.45 m / C9+C5 / route_check exit 0 | 一致；token 全合法（普通 66 / 倒车 0 / 射击 2 / 折返 4） | 0 | 同包 `lead_accept/lead_B_map7_stdout.txt` |
| C · `test_autoplan.py` | 171/171 exit 0 | **171/171** | 0 | 同包 `lead_accept/lead_C_test_autoplan_stdout.txt` |

**判定：验收通过（离线，未部署，未上车）。** §4 三项待裁决不回退验收入账：① `degraded` 进遥测（`mission.py`，需串行单卡）；② 默认是否切 `--objective tasks`（车端仍跑 legacy）；③ 与外部 B 包三方合并（同文件，禁止整目录覆盖）。

口径更正：§2/§4 引用的 `dzrobot_v2/_tmp_session/...` 路径已于 10-22 资料区重组中归档，现只能从 `docs/history_cache/03_过程稿快照/dzrobot_v2_tmp_session_20261022.tar.gz` 内按上述路径解出。

## 相关文件

- `dzrobot_v2/_tmp_session/TRACK-PLAN-B1-20261021/REPORT.md`（主报告 171 行）
- `dzrobot_v2/_tmp_session/TRACK-PLAN-B1-20261021/stress_before.json` / `stress_after.json` / `stress_legacy_after2.json`
- `dzrobot_v2/_tmp_session/TRACK-PLAN-B1-20261021/logs/`（A/B/C/E 原始 stdout）
- `dzrobot_v2/_tmp_session/t35/REPORT.md`（6 条真缺陷来源）
- `_tmp_session/paper50/三大优化方案.md:81`（T-B1 原始验收判据）
