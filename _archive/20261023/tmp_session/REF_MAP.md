---
status: 过程
更新: 2026-10-23
---

# REF_MAP —— 文档里 `_tmp_session` 引用的存活对照表（2026-10-23 21:45 生成）

> 生成方式：扫 `docs/**/*.md`（不含 `history_cache/`），抽取所有 `_tmp_session/...` 路径，逐条判存在性。
> **用途**：读者看到文档里引用 `_tmp_session/xxx` 时先查本表——标 `失效` 的不要去点，那是 10-21 过程目录清空后的历史指针。
> 本表由脚本按当前磁盘状态生成，属过程稿，不保证长期有效；重扫请跑 `python docs/_工具/check_tmprefs.py`（若已建卡）。

## 0 汇总

- 唯一条目 **243**：存活 **13** / 失效 **230**（占位符 `_tmp_session/<卡号>/`、`_tmp_session/**` 已排除）
- 失效条目的处置口径：① 原件在 `_archive/20261023/tmp_session/pack_20261023.zip` 里的，改指包内路径；② 不在包里的（10-21 前的过程稿），正文标「〔档〕已清空」，保留原路径供追责，不再指望能打开。

## 1 存活（可以点）

| 路径 | 引用文件数 |
|---|---:|
| `_tmp_session/aim-p0/car_src/` | 1 |
| `_tmp_session/abs/` | 1 |
| `_tmp_session/AG-TEAM-20261023/P/PLAN.md` | 1 |
| `_tmp_session/aim-p0/` | 1 |
| `_tmp_session/AG-TEAM-20261023/P/run_baseline_45.sh` | 1 |
| `_tmp_session/AG-TEAM-20261023/T/` | 1 |
| `_tmp_session/TOOLS-PLUGIN-REWORK-20261022/repoint-deps.cjs` | 1 |
| `_tmp_session/LEAD-FIELD-20261022/lead_yaw_probe_1956.csv` | 1 |
| `_tmp_session/tools-organize-20261022/manifest.json` | 1 |
| `_tmp_session/tools-organize-20261022/organize-plugins2.ps1` | 1 |
| `_tmp_session/tools-organize-20261022/acl-repair/` | 1 |
| `_tmp_session/AG-TEAM-PLAN-20261023` | 1 |
| `_tmp_session/AG-TEAM-PLAN-20261023/gates.json` | 1 |

## 2 失效（不要点）

| 路径 | 引用数 | 引用它的文件（前 3） |
|---|---:|---|
| `_tmp_session/PROJECT_STATUS_20261021.md` | 10 | `docs/00_项目总览/时间线.md`<br>`docs/00_项目总览/现状快照.md`<br>`docs/00_项目总览/项目总览.md` |
| `_tmp_session/paper50/三大优化方案.md` | 9 | `docs/00_项目总览/时间线.md`<br>`docs/00_项目总览/现状快照.md`<br>`docs/00_项目总览/项目总览.md` |
| `_tmp_session/agent_overview_20261020/drone.md` | 6 | `docs/10_agent提示词/04_场景/无人机恢复.md`<br>`docs/20_资料索引库/无人机/00_索引.md`<br>`docs/20_资料索引库/无人机/01_架构总览.md` |
| `_tmp_session/TRACK-R0-20261020-01/CARD.md:4-6` | 5 | `docs/00_项目总览/时间线.md`<br>`docs/30_任务卡库/README.md`<br>`docs/30_任务卡库/归档/00_索引.md` |
| `_tmp_session/TRACK-R1-20261020-01/CARD.md` | 5 | `docs/10_agent提示词/03_角色/implementer.md`<br>`docs/10_agent提示词/03_角色/lead.md`<br>`docs/10_agent提示词/03_角色/verifier.md` |
| `_tmp_session/v4.0-closeout/VERIFY.md:33-44` | 4 | `docs/00_项目总览/时间线.md`<br>`docs/30_任务卡库/归档/00_索引.md`<br>`docs/30_任务卡库/归档/2026-10-18.md` |
| `_tmp_session/TRACK-BASE-20261021/CARD.md:8` | 4 | `docs/00_项目总览/时间线.md`<br>`docs/30_任务卡库/归档/00_索引.md`<br>`docs/30_任务卡库/归档/2026-10-21.md` |
| `_tmp_session/TRACK-R1-20261020-01/REPORT.md` | 4 | `docs/10_agent提示词/03_角色/implementer.md`<br>`docs/10_agent提示词/03_角色/reviewer.md`<br>`docs/10_agent提示词/04_场景/离线回归攻坚.md` |
| `_tmp_session/TRACK-PLAN-B1-20261021/REPORT.md` | 4 | `docs/20_资料索引库/车端/03_规划器与自动规划.md`<br>`docs/30_任务卡库/在办/track-1021/TRACK-PLAN-B1-20261021.md`<br>`docs/30_任务卡库/模板/回报模板.md` |
| `_tmp_session/TRACK-FIELD-R2-20261020-01/review.md:6` | 4 | `docs/30_任务卡库/README.md`<br>`docs/30_任务卡库/归档/00_索引.md`<br>`docs/30_任务卡库/归档/2026-10-20.md` |
| `_tmp_session/report_20261018/7_path.log:625-632` | 3 | `docs/00_项目总览/时间线.md`<br>`docs/20_资料索引库/车端/03_规划器与自动规划.md`<br>`docs/20_资料索引库/车端/05_配置与部署.md` |
| `_tmp_session/PC-01-提示词压缩/账本.md` | 3 | `docs/10_agent提示词/00_索引.md`<br>`docs/10_agent提示词/02_发送指南.md`<br>`docs/10_agent提示词/README.md` |
| `_tmp_session/agent_overview_20261020/drone.md（10-20` | 3 | `docs/20_资料索引库/无人机/00_索引.md`<br>`docs/20_资料索引库/无人机/01_架构总览.md`<br>`docs/20_资料索引库/无人机/06_已取证与未验证.md` |
| `_tmp_session/t3/audit/FLOW_COMPARISON.md` | 3 | `docs/20_资料索引库/空地链路/00_索引.md`<br>`docs/20_资料索引库/空地链路/01_协议v2与契约.md`<br>`docs/20_资料索引库/空地链路/02_链路联调与已知缺口.md` |
| `_tmp_session/TRACK-PLAN-B1-20261021/REPORT.md:8` | 3 | `docs/20_资料索引库/车端/03_规划器与自动规划.md`<br>`docs/_work/timeline_doc-car.md`<br>`docs/_work/timeline_doc-cards.md` |
| `_tmp_session/TRACK-FIELD-R3-20261020-01/REPORT.md:4-6` | 3 | `docs/30_任务卡库/README.md`<br>`docs/30_任务卡库/归档/00_索引.md`<br>`docs/_work/timeline_doc-cards.md` |
| `_tmp_session/overnight/NIGHT_AUDIT.md:33-39` | 3 | `docs/30_任务卡库/README.md`<br>`docs/30_任务卡库/归档/2026-10-18.md`<br>`docs/_work/timeline_doc-cards.md` |
| `_tmp_session/TRACK-FIELD-B5-20261021/REPORT.md` | 3 | `docs/30_任务卡库/在办/track-1021/TRACK-FIELD-B5-20261021.md`<br>`docs/30_任务卡库/归档/2026-10-19.md`<br>`docs/30_任务卡库/模板/回报模板.md` |
| `_tmp_session/p3_self_evolve/REPORT.md:11-17` | 3 | `docs/30_任务卡库/归档/00_索引.md`<br>`docs/30_任务卡库/归档/2026-10-18.md`<br>`docs/_work/timeline_doc-cards.md` |
| `_tmp_session/TRACK-R1-20261020-01/CARD.md:8` | 3 | `docs/30_任务卡库/归档/00_索引.md`<br>`docs/30_任务卡库/归档/2026-10-20.md`<br>`docs/_work/timeline_doc-cards.md` |
| `_tmp_session/TRACK-DEPLOY-PREP-20261020-01/CARD.md:9` | 3 | `docs/30_任务卡库/归档/00_索引.md`<br>`docs/30_任务卡库/归档/2026-10-20.md`<br>`docs/_work/timeline_doc-cards.md` |
| `_tmp_session/yb_tree/docs/archive/进度报告-20260927.md` | 2 | `docs/00_项目总览/时间线.md`<br>`docs/20_资料索引库/历史/01_沿革_20260927_20261012.md` |
| `_tmp_session/yb_tree/docs/会话交接-20261002.md` | 2 | `docs/00_项目总览/时间线.md`<br>`docs/20_资料索引库/历史/01_沿革_20260927_20261012.md` |
| `_tmp_session/paper50/三大优化方案.md:165` | 2 | `docs/00_项目总览/时间线.md`<br>`docs/30_任务卡库/在办/drone-1022/T12-无人机上线执行单.md` |
| `_tmp_session/overnight/PLAN.md:1-4` | 2 | `docs/00_项目总览/时间线.md`<br>`docs/_work/timeline_doc-cards.md` |
| `_tmp_session/task-44/REPORT.md:7-11` | 2 | `docs/00_项目总览/时间线.md`<br>`docs/30_任务卡库/归档/00_索引.md` |
| `_tmp_session/paper50/三大优化方案.md:3` | 2 | `docs/00_项目总览/时间线.md`<br>`docs/_work/timeline_doc-cards.md` |
| `_tmp_session/TRACK-FIELD-R3-20261020-01/review_20261021.md` | 2 | `docs/00_项目总览/时间线.md`<br>`docs/30_任务卡库/在办/track-1021/TRACK-FIELD-R3B-20261021.md` |
| `_tmp_session/PROJECT_STATUS_20261021.md:12` | 2 | `docs/00_项目总览/时间线.md`<br>`docs/_work/timeline_doc-cards.md` |
| `_tmp_session/TRACK-PLAN-B1-20261021/REPORT.md:29-36` | 2 | `docs/00_项目总览/时间线.md`<br>`docs/30_任务卡库/在办/track-1021/TRACK-PLAN-B1-20261021.md` |
| `_tmp_session/p1_hybrid/stress_results.json` | 2 | `docs/00_项目总览/现状快照.md`<br>`docs/20_资料索引库/车端/03_规划器与自动规划.md` |
| `_tmp_session/aim_step3/rec4.csv` | 2 | `docs/10_agent提示词/00_索引.md`<br>`docs/10_agent提示词/04_场景/云台瞄准调参.md` |
| `_tmp_session/agent_overview_20261020/car.md` | 2 | `docs/10_agent提示词/03_角色/auditor.md`<br>`docs/_work/timeline_doc-prompts.md` |
| `_tmp_session/yb_tree/docs/archive/（16` | 2 | `docs/20_资料索引库/历史/00_索引.md`<br>`docs/20_资料索引库/历史/01_沿革_20260927_20261012.md` |
| `_tmp_session/v4.0-closeout/VERIFY.md` | 2 | `docs/20_资料索引库/历史/01_沿革_20260927_20261012.md`<br>`docs/30_任务卡库/归档/2026-10-18.md` |
| `_tmp_session/agent_overview_20261020/drone.md:22` | 2 | `docs/20_资料索引库/无人机/00_索引.md`<br>`docs/20_资料索引库/无人机/01_架构总览.md` |
| `_tmp_session/agent_overview_20261020/drone.md:161` | 2 | `docs/20_资料索引库/无人机/00_索引.md`<br>`docs/20_资料索引库/无人机/01_架构总览.md` |
| `_tmp_session/t3/perception/README.md` | 2 | `docs/20_资料索引库/视觉与云台/00_索引.md`<br>`docs/20_资料索引库/视觉与云台/02_视觉模型与检测.md` |
| `_tmp_session/aim_audit/体检报告_20261018_1741.md` | 2 | `docs/20_资料索引库/视觉与云台/01_云台瞄准与开火链路.md`<br>`docs/_work/cleanup_summary.md` |
| `_tmp_session/yb_tree/docs/raw_txt/02_15空地协同侦排挑战赛.txt（赛规` | 2 | `docs/20_资料索引库/赛事与治理/01_赛题规则与场地.md`<br>`docs/20_资料索引库/赛事与治理/02_计分与时间预算.md` |
| `_tmp_session/t_d1b/REPORT.md:7-11` | 2 | `docs/20_资料索引库/车端/04_定位与安全.md`<br>`docs/_work/timeline_doc-car.md` |
| `_tmp_session/t_d1b/REPORT.md` | 2 | `docs/20_资料索引库/车端/04_定位与安全.md`<br>`docs/30_任务卡库/归档/2026-10-19.md` |
| `_tmp_session/t_ae_contract/REPORT.md` | 2 | `docs/20_资料索引库/车端/04_定位与安全.md`<br>`docs/30_任务卡库/归档/2026-10-19.md` |
| `_tmp_session/task-44/REPORT.md` | 2 | `docs/20_资料索引库/车端/04_定位与安全.md`<br>`docs/30_任务卡库/归档/2026-10-19.md` |
| `_tmp_session/TRACK-R2-REVIEW-20261020/REPORT.md:11-15` | 2 | `docs/30_任务卡库/README.md`<br>`docs/30_任务卡库/归档/2026-10-20.md` |
| `_tmp_session/PROJECT_STATUS_20261021.md:45-62` | 2 | `docs/30_任务卡库/README.md`<br>`docs/_work/timeline_doc-cards.md` |
| `_tmp_session/PROJECT_STATUS_20261021.md:84` | 2 | `docs/30_任务卡库/README.md`<br>`docs/_work/timeline_doc-cards.md` |
| `_tmp_session/TRACK-FIELD-R3-20261020-01/RUN_CARD_20261021.md` | 2 | `docs/30_任务卡库/在办/track-1021/TRACK-FIELD-R3B-20261021.md`<br>`docs/30_任务卡库/在办/track-1022/TRACK-FIELD-D11-20261022.md` |
| `_tmp_session/TRACK-PLAN-B1-20261021/lead_accept/lead_A_stress64_stdout.txt` | 2 | `docs/30_任务卡库/在办/track-1021/TRACK-PLAN-B1-20261021.md`<br>`docs/30_任务卡库/在办/track-1022/TRACK-DAY-20261022.md` |
| `_tmp_session/t35/REPORT.md` | 2 | `docs/30_任务卡库/在办/track-1021/TRACK-PLAN-B1-20261021.md`<br>`docs/30_任务卡库/归档/2026-10-18.md` |
| `_tmp_session/TRACK-HANDOFF-20261020/CARD.md:22` | 2 | `docs/30_任务卡库/归档/00_索引.md`<br>`docs/30_任务卡库/归档/2026-10-20.md` |
| `_tmp_session/TRACK-R0-REVIEW-20261020/CARD.md:19` | 2 | `docs/30_任务卡库/归档/00_索引.md`<br>`docs/30_任务卡库/归档/2026-10-20.md` |
| `_tmp_session/TRACK-FIELD-PLAN-20261020/CARD.md:18` | 2 | `docs/30_任务卡库/归档/00_索引.md`<br>`docs/30_任务卡库/归档/2026-10-20.md` |
| `_tmp_session/TRACK-R1-REVIEW-20261020/REPORT.md:16-21` | 2 | `docs/30_任务卡库/归档/00_索引.md`<br>`docs/30_任务卡库/归档/2026-10-20.md` |
| `_tmp_session/TRACK-DISPATCH-20261020/CARD.md:11` | 2 | `docs/30_任务卡库/归档/00_索引.md`<br>`docs/30_任务卡库/归档/2026-10-20.md` |
| `_tmp_session/TRACK-DISPATCH-COPY-20261020/CARD.md:11` | 2 | `docs/30_任务卡库/归档/00_索引.md`<br>`docs/30_任务卡库/归档/2026-10-20.md` |
| `_tmp_session/TRACK-PAPER-MERGE-20261021/review.md:5-13` | 2 | `docs/30_任务卡库/归档/00_索引.md`<br>`docs/30_任务卡库/归档/2026-10-21.md` |
| `_tmp_session/TRACK-VIS-20261021/CARD.md:31-37` | 2 | `docs/30_任务卡库/归档/00_索引.md`<br>`docs/30_任务卡库/归档/2026-10-21.md` |
| `_tmp_session/TRACK-YAW-20261021/CARD.md:12-22` | 2 | `docs/30_任务卡库/归档/00_索引.md`<br>`docs/30_任务卡库/归档/2026-10-21.md` |
| `_tmp_session/t35/REPORT.md:8-13` | 2 | `docs/30_任务卡库/归档/2026-10-18.md`<br>`docs/_work/timeline_doc-cards.md` |
| `_tmp_session/task-44/REPORT.md:9-11` | 2 | `docs/30_任务卡库/归档/2026-10-19.md`<br>`docs/_work/timeline_doc-cards.md` |
| `_tmp_session/t_ae_contract/REPORT.md:12-21` | 2 | `docs/30_任务卡库/归档/2026-10-19.md`<br>`docs/_work/timeline_doc-cards.md` |
| `_tmp_session/TRACK-VIS-20261021/CARD.md` | 2 | `docs/30_任务卡库/归档/2026-10-21.md`<br>`docs/30_任务卡库/模板/任务卡模板.md` |
| `_tmp_session/TRACK-YAW-20261021/CARD.md` | 2 | `docs/30_任务卡库/归档/2026-10-21.md`<br>`docs/30_任务卡库/模板/任务卡模板.md` |
| `_tmp_session/aimtune、aim-lat` | 1 | `docs/00_项目总览/交接_20261023.md` |
| `_tmp_session/aimtune/{grab.py` | 1 | `docs/00_项目总览/交接_20261023.md` |
| `_tmp_session/{t3` | 1 | `docs/00_项目总览/团队与分工.md` |
| `_tmp_session/PC-01-提示词压缩` | 1 | `docs/00_项目总览/团队与分工.md` |
| `_tmp_session/yb_tree/docs/archive/官方环境修复记录-20260928.md` | 1 | `docs/00_项目总览/时间线.md` |
| `_tmp_session/yb_tree/docs/archive/官方资料对齐-20260929.md` | 1 | `docs/00_项目总览/时间线.md` |
| `_tmp_session/yb_tree/docs/archive/画圆根因分析-20260930.md:10-13` | 1 | `docs/00_项目总览/时间线.md` |
| `_tmp_session/yb_tree/docs/archive/朝向偏差根因-20261001.md` | 1 | `docs/00_项目总览/时间线.md` |
| `_tmp_session/yb_tree/docs/官方框架重构方案-20261001.md` | 1 | `docs/00_项目总览/时间线.md` |
| `_tmp_session/carsnap_tree/src/history/book/项目技术文档2026-10-03.md` | 1 | `docs/00_项目总览/时间线.md` |
| `_tmp_session/carsnap_tree/src/history/log/log_20261011_0819_auto_planner.md` | 1 | `docs/00_项目总览/时间线.md` |
| `_tmp_session/carsnap_tree/docs/agent_archive/v2-全量整理-2026-10-12.md` | 1 | `docs/00_项目总览/时间线.md` |
| `_tmp_session/carsnap_tree/docs/agent_archive/v3-空地协同闭环-2026-10-14.md` | 1 | `docs/00_项目总览/时间线.md` |
| `_tmp_session/yb_tree/docs/分区规划与感知约束-20261015.md` | 1 | `docs/00_项目总览/时间线.md` |
| `_tmp_session/yb_tree/docs/SIL发现-PIVOT过冲-20261016.md` | 1 | `docs/00_项目总览/时间线.md` |
| `_tmp_session/yb_tree/docs/实测方案-车端三项-20261016.md` | 1 | `docs/00_项目总览/时间线.md` |
| `_tmp_session/yb_tree/docs/悬停稳定性取证-20261017.md` | 1 | `docs/00_项目总览/时间线.md` |
| `_tmp_session/PROJECT_STATUS_20261021.md:78` | 1 | `docs/00_项目总览/时间线.md` |
| `_tmp_session/TRACK-BASE-20261021/snapshot/docs/三引擎审计_20261018.md:44-58` | 1 | `docs/00_项目总览/时间线.md` |
| `_tmp_session/PC-01-提示词压缩/账本.md:7` | 1 | `docs/00_项目总览/时间线.md` |
| `_tmp_session/PROJECT_STATUS_20261021.md:74-80` | 1 | `docs/00_项目总览/时间线.md` |
| `_tmp_session/TRACK-FIELD-R3-20261020-01/` | 1 | `docs/00_项目总览/时间线.md` |
| `_tmp_session/yb_tree/docs/raw_txt/02_15空地协同侦排挑战赛.txt:280-304` | 1 | `docs/00_项目总览/时间线.md` |
| `_tmp_session/TRACK-R0-20261020-01/` | 1 | `docs/00_项目总览/现状快照.md` |
| `_tmp_session/TRACK-R1-20261020-01/` | 1 | `docs/00_项目总览/现状快照.md` |
| `_tmp_session/agent_overview_20261020/history.md` | 1 | `docs/10_agent提示词/02_发送指南.md` |
| `_tmp_session/agent_overview_20261020/` | 1 | `docs/10_agent提示词/03_角色/auditor.md` |
| `_tmp_session/TRACK-R1-20261020-01/verification.md` | 1 | `docs/10_agent提示词/03_角色/verifier.md` |
| `_tmp_session/aim_step3/rec4.csv（本次实算` | 1 | `docs/10_agent提示词/04_场景/云台瞄准调参.md` |
| `_tmp_session/agent_overview_20261020/link.md` | 1 | `docs/10_agent提示词/04_场景/无人机恢复.md` |
| `_tmp_session/d13_verify/` | 1 | `docs/20_资料索引库/历史/00_索引.md` |
| `_tmp_session/yb_tree/docs/战略评估-省一冲刺-20261014.md` | 1 | `docs/20_资料索引库/历史/01_沿革_20260927_20261012.md` |
| `_tmp_session/yb_tree/docs/接手说明.md` | 1 | `docs/20_资料索引库/历史/01_沿革_20260927_20261012.md` |
| `_tmp_session/yb_tree/docs/` | 1 | `docs/20_资料索引库/历史/01_沿革_20260927_20261012.md` |
| `_tmp_session/d13_verify/切换记录_20261018.md` | 1 | `docs/20_资料索引库/历史/02_并入与改名_20261018.md` |
| `_tmp_session/agent_overview_20261020/drone.md:11-14` | 1 | `docs/20_资料索引库/无人机/01_架构总览.md` |
| `_tmp_session/agent_overview_20261020/drone.md:275、:289` | 1 | `docs/20_资料索引库/无人机/01_架构总览.md` |
| `_tmp_session/agent_overview_20261020/drone.md:71` | 1 | `docs/20_资料索引库/无人机/03_感知与视觉侦察.md` |
| `_tmp_session/agent_overview_20261020/drone.md:249` | 1 | `docs/20_资料索引库/无人机/03_感知与视觉侦察.md` |
| `_tmp_session/agent_overview_20261020/drone.md:154、:314（截图` | 1 | `docs/20_资料索引库/无人机/03_感知与视觉侦察.md` |
| `_tmp_session/drone-review/REVIEW.md` | 1 | `docs/20_资料索引库/无人机/03_感知与视觉侦察.md` |
| `_tmp_session/agent_overview_20261020/drone.md:312` | 1 | `docs/20_资料索引库/无人机/04_路线下发与轨迹回放.md` |
| `_tmp_session/agent_overview_20261020/drone.md:272` | 1 | `docs/20_资料索引库/无人机/05_现场操作与脚本.md` |
| `_tmp_session/agent_overview_20261020/drone.md:241-254` | 1 | `docs/20_资料索引库/无人机/05_现场操作与脚本.md` |
| `_tmp_session/agent_overview_20261020/drone.md:154、:314（截图哈希` | 1 | `docs/20_资料索引库/无人机/06_已取证与未验证.md` |
| `_tmp_session/drone_ev/EV验证_20261017.md` | 1 | `docs/20_资料索引库/无人机/06_已取证与未验证.md` |
| `_tmp_session/drone/、drone_arch/、drone_l3/、drone_probe/、drone_recon/、drone_src/` | 1 | `docs/20_资料索引库/无人机/06_已取证与未验证.md` |
| `_tmp_session/t3/emitter/README.md` | 1 | `docs/20_资料索引库/空地链路/01_协议v2与契约.md` |
| `_tmp_session/drone_gap/ab_legacy.py` | 1 | `docs/20_资料索引库/空地链路/01_协议v2与契约.md` |
| `_tmp_session/t3/emitter/README.md、docs/20_资料索引库/车端/（规划器` | 1 | `docs/20_资料索引库/空地链路/01_协议v2与契约.md` |
| `_tmp_session/t3/carcheck/EMITTER_CROSSCHECK.md` | 1 | `docs/20_资料索引库/空地链路/02_链路联调与已知缺口.md` |
| `_tmp_session/t3/carcheck/runbook.md` | 1 | `docs/20_资料索引库/空地链路/02_链路联调与已知缺口.md` |
| `_tmp_session/t3_batch/README.md` | 1 | `docs/20_资料索引库/空地链路/02_链路联调与已知缺口.md` |
| `_tmp_session/t3/perception/perception_cells.py:96-97` | 1 | `docs/20_资料索引库/空地链路/02_链路联调与已知缺口.md` |
| `_tmp_session/t3/audit/FLOW_COMPARISON.md（NO-GO` | 1 | `docs/20_资料索引库/空地链路/02_链路联调与已知缺口.md` |
| `_tmp_session/t3/carcheck/EMITTER_CROSSCHECK.md（E1–E4` | 1 | `docs/20_资料索引库/空地链路/02_链路联调与已知缺口.md` |
| `_tmp_session/t3/carcheck/runbook.md（起栈判据` | 1 | `docs/20_资料索引库/空地链路/02_链路联调与已知缺口.md` |
| `_tmp_session/t3_batch/README.md（recon` | 1 | `docs/20_资料索引库/空地链路/02_链路联调与已知缺口.md` |
| `_tmp_session/aim_audit/体检报告_20261018_1741.md:37` | 1 | `docs/20_资料索引库/视觉与云台/00_索引.md` |
| `_tmp_session/paper50/三大优化方案.md:17` | 1 | `docs/20_资料索引库/视觉与云台/00_索引.md` |
| `_tmp_session/paper50/三大优化方案.md:113` | 1 | `docs/20_资料索引库/视觉与云台/00_索引.md` |
| `_tmp_session/gimbal-patch/REPORT.md` | 1 | `docs/20_资料索引库/视觉与云台/01_云台瞄准与开火链路.md` |
| `_tmp_session/t3/perception/README.md（UDP` | 1 | `docs/20_资料索引库/视觉与云台/02_视觉模型与检测.md` |
| `_tmp_session/_pdf_text/场地布置说明书V2-空地协同侦排挑战赛.txt（12` | 1 | `docs/20_资料索引库/赛事与治理/01_赛题规则与场地.md` |
| `_tmp_session/yb_tree/docs/raw_txt/02_15空地协同侦排挑战赛.txt〔档` | 1 | `docs/20_资料索引库/赛事与治理/01_赛题规则与场地.md` |
| `_tmp_session/_pdf_text/场地布置说明书V2-空地协同侦排挑战赛.txt〔档〕` | 1 | `docs/20_资料索引库/赛事与治理/01_赛题规则与场地.md` |
| `_tmp_session/vision_map_report.md` | 1 | `docs/20_资料索引库/赛事与治理/01_赛题规则与场地.md` |
| `_tmp_session/_pdf_text/场地布置说明书V2-空地协同侦排挑战赛.txt` | 1 | `docs/20_资料索引库/赛事与治理/02_计分与时间预算.md` |
| `_tmp_session/archive/` | 1 | `docs/20_资料索引库/赛事与治理/06_版本存档规则.md` |
| `_tmp_session/TRACK-R0/TRACK-R1` | 1 | `docs/20_资料索引库/车端/02_循迹与控制.md` |
| `_tmp_session/p1_hybrid/` | 1 | `docs/20_资料索引库/车端/03_规划器与自动规划.md` |
| `_tmp_session/p1_hybrid/STRESS_REPORT.md` | 1 | `docs/20_资料索引库/车端/03_规划器与自动规划.md` |
| `_tmp_session/t35/` | 1 | `docs/20_资料索引库/车端/03_规划器与自动规划.md` |
| `_tmp_session/t_d1b/` | 1 | `docs/20_资料索引库/车端/04_定位与安全.md` |
| `_tmp_session/task-44/REPORT.md:30-37` | 1 | `docs/20_资料索引库/车端/04_定位与安全.md` |
| `_tmp_session/step_loop/map7.yaml` | 1 | `docs/20_资料索引库/车端/05_配置与部署.md` |
| `_tmp_session/sim_map_20261018/map.yaml` | 1 | `docs/20_资料索引库/车端/05_配置与部署.md` |
| `_tmp_session/step_loop/map7.yaml:15-18` | 1 | `docs/20_资料索引库/车端/05_配置与部署.md` |
| `_tmp_session/sim_map_20261018/map.yaml:8-11` | 1 | `docs/20_资料索引库/车端/05_配置与部署.md` |
| `_tmp_session/TRACK-R0/` | 1 | `docs/20_资料索引库/车端/06_回归与测试基线.md` |
| `_tmp_session/t35/REPORT.md:64-68` | 1 | `docs/20_资料索引库/车端/06_回归与测试基线.md` |
| `_tmp_session/forensics/项目体检_20261019.md` | 1 | `docs/20_资料索引库/车端/06_回归与测试基线.md` |
| `_tmp_session/TRACK-R0-20261020-01/motion.md` | 1 | `docs/20_资料索引库/车端/08_车体几何与执行约束.md` |
| `_tmp_session/gitee_main_tree/（2537` | 1 | `docs/20_资料索引库/车端/09_外部实现借鉴_air_ground_20261018.md` |
| `_tmp_session/gitee_main_tree/（本地工作副本` | 1 | `docs/20_资料索引库/车端/09_外部实现借鉴_air_ground_20261018.md` |
| `_tmp_session/t37_run2/` | 1 | `docs/30_任务卡库/README.md` |
| `_tmp_session/TRACK-FIELD-R2-20261020-01/review.md:17-19` | 1 | `docs/30_任务卡库/README.md` |
| `_tmp_session/TRACK-FIELD-R3-20261020-01/review_20261021.md:866` | 1 | `docs/30_任务卡库/README.md` |
| `_tmp_session/t35/REPORT.md:20-21` | 1 | `docs/30_任务卡库/README.md` |
| `_tmp_session/TRACK-R1-REVIEW-20261020/REPORT.md:43` | 1 | `docs/30_任务卡库/README.md` |
| `_tmp_session/aimtune/{aim_rec.py` | 1 | `docs/30_任务卡库/在办/aim-1023/AIM-BASELINE-20261023.md` |
| `_tmp_session/AG-TEAM-20261023/P/data/{P_static.jsonl` | 1 | `docs/30_任务卡库/在办/aim-1023/AIM-BASELINE-20261023.md` |
| `_tmp_session/AG-TEAM-20261023/T/PLAN.md、决策台账` | 1 | `docs/30_任务卡库/在办/aim-1023/TRACK-C0EXT-20261023.md` |
| `_tmp_session/AG-TEAM-20261023/T/C0EXT/{run1..3}/` | 1 | `docs/30_任务卡库/在办/aim-1023/TRACK-C0EXT-20261023.md` |
| `_tmp_session/PROJECT_STATUS_20261021.md:109` | 1 | `docs/30_任务卡库/在办/drone-1022/T12-无人机上线执行单.md` |
| `_tmp_session/paper50/三大优化方案.md:172` | 1 | `docs/30_任务卡库/在办/drone-1022/T12-无人机上线执行单.md` |
| `_tmp_session/paper50/三大优化方案.md:220` | 1 | `docs/30_任务卡库/在办/drone-1022/T12-无人机上线执行单.md` |
| `_tmp_session/20261023_root_tmp/` | 1 | `docs/30_任务卡库/在办/lead-1023/DOC-FIX-20261023.md` |
| `_tmp_session/TRACK-FIELD-B5-20261021/guard_replay.md` | 1 | `docs/30_任务卡库/在办/track-1021/TRACK-FIELD-B5-20261021.md` |
| `_tmp_session/TRACK-FIELD-B5-20261021/slip_diagnosis.md` | 1 | `docs/30_任务卡库/在办/track-1021/TRACK-FIELD-B5-20261021.md` |
| `_tmp_session/TRACK-FIELD-B5-20261021/MEASURE_SOP.md` | 1 | `docs/30_任务卡库/在办/track-1021/TRACK-FIELD-B5-20261021.md` |
| `_tmp_session/TRACK-FIELD-B5-20261021/REPORT.md:4` | 1 | `docs/30_任务卡库/在办/track-1021/TRACK-FIELD-B5-20261021.md` |
| `_tmp_session/…` | 1 | `docs/30_任务卡库/在办/track-1021/TRACK-FIELD-B5-20261021.md` |
| `_tmp_session/TRACK-FIELD-R2-20261020-01/evidence/car_logs/3b_localization.log:1281-1340` | 1 | `docs/30_任务卡库/在办/track-1021/TRACK-FIELD-B5-20261021.md` |
| `_tmp_session/TRACK-FIELD-B5-20261021/car_pull_1019/` | 1 | `docs/30_任务卡库/在办/track-1021/TRACK-FIELD-B5-20261021.md` |
| `_tmp_session/paper50/三大优化方案.md:218` | 1 | `docs/30_任务卡库/在办/track-1021/TRACK-FIELD-B5-20261021.md` |
| `_tmp_session/TRACK-YAW-20261021/REPORT.md` | 1 | `docs/30_任务卡库/在办/track-1021/TRACK-FIELD-B5-20261021.md` |
| `_tmp_session/TRACK-FIELD-R3-20261020-01/CARD_20261021.md` | 1 | `docs/30_任务卡库/在办/track-1021/TRACK-FIELD-R3B-20261021.md` |
| `_tmp_session/PROJECT_STATUS_20261021.md:57-62` | 1 | `docs/30_任务卡库/在办/track-1021/TRACK-FIELD-R3B-20261021.md` |
| `_tmp_session/PROJECT_STATUS_20261021.md:68` | 1 | `docs/30_任务卡库/在办/track-1021/TRACK-FIELD-R3B-20261021.md` |
| `_tmp_session/TRACK-FIELD-R3-20261020-01/REPORT.md` | 1 | `docs/30_任务卡库/在办/track-1021/TRACK-FIELD-R3B-20261021.md` |
| `_tmp_session/TRACK-PLAN-B1-20261021/REPORT.md:4` | 1 | `docs/30_任务卡库/在办/track-1021/TRACK-PLAN-B1-20261021.md` |
| `_tmp_session/TRACK-PLAN-B1-20261021/stress_before.json` | 1 | `docs/30_任务卡库/在办/track-1021/TRACK-PLAN-B1-20261021.md` |
| `_tmp_session/TRACK-PLAN-B1-20261021/logs/` | 1 | `docs/30_任务卡库/在办/track-1021/TRACK-PLAN-B1-20261021.md` |
| `_tmp_session/paper50/三大优化方案.md:81` | 1 | `docs/30_任务卡库/在办/track-1021/TRACK-PLAN-B1-20261021.md` |
| `_tmp_session/TRACK-PLAN-B1-20261021/` | 1 | `docs/30_任务卡库/在办/track-1022/TRACK-DAY-20261022.md` |
| `_tmp_session/t34/REPORT.md:11-13` | 1 | `docs/30_任务卡库/归档/00_索引.md` |
| `_tmp_session/t35/REPORT.md:62-68` | 1 | `docs/30_任务卡库/归档/00_索引.md` |
| `_tmp_session/t_ae_contract/REPORT.md:62-71` | 1 | `docs/30_任务卡库/归档/00_索引.md` |
| `_tmp_session/t_d1b/REPORT.md:25-30` | 1 | `docs/30_任务卡库/归档/00_索引.md` |
| `_tmp_session/TRACK-R2-REVIEW-20261020/REPORT.md:7-15` | 1 | `docs/30_任务卡库/归档/00_索引.md` |
| `_tmp_session/TRACK-ACK-PLAN-20261021/review.md:5-6` | 1 | `docs/30_任务卡库/归档/00_索引.md` |
| `_tmp_session/t34/REPORT.md` | 1 | `docs/30_任务卡库/归档/2026-10-18.md` |
| `_tmp_session/p3_self_evolve/REPORT.md` | 1 | `docs/30_任务卡库/归档/2026-10-18.md` |
| `_tmp_session/overnight/NIGHT_AUDIT.md` | 1 | `docs/30_任务卡库/归档/2026-10-18.md` |
| `_tmp_session/t34/REPORT.md:11-17` | 1 | `docs/30_任务卡库/归档/2026-10-18.md` |
| `_tmp_session/overnight/` | 1 | `docs/30_任务卡库/归档/2026-10-18.md` |
| `_tmp_session/overnight/radius_audit.md:20-36` | 1 | `docs/30_任务卡库/归档/2026-10-18.md` |
| `_tmp_session/t_d1b/REPORT.md:7-21` | 1 | `docs/30_任务卡库/归档/2026-10-19.md` |
| `_tmp_session/TRACK-FIELD-B5-20261021/REPORT.md:30-40` | 1 | `docs/30_任务卡库/归档/2026-10-19.md` |
| `_tmp_session/TRACK-R0-20261020-01/review.md:11-21` | 1 | `docs/30_任务卡库/归档/2026-10-20.md` |
| `_tmp_session/TRACK-R1-20261020-01/review.md:39` | 1 | `docs/30_任务卡库/归档/2026-10-20.md` |
| `_tmp_session/TRACK-R2-REVIEW-20261020/REPORT.md:7` | 1 | `docs/30_任务卡库/归档/2026-10-20.md` |
| `_tmp_session/TRACK-FIELD-R3-20261020-01/REPORT.md:4-19` | 1 | `docs/30_任务卡库/归档/2026-10-20.md` |
| `_tmp_session/TRACK-FIELD-R3-20261020-01/review.md:9-11` | 1 | `docs/30_任务卡库/归档/2026-10-20.md` |
| `_tmp_session/TRACK-R0-20261020-01/CARD.md` | 1 | `docs/30_任务卡库/归档/2026-10-20.md` |
| `_tmp_session/TRACK-BASE-20261021/CARD.md` | 1 | `docs/30_任务卡库/归档/2026-10-21.md` |
| `_tmp_session/TRACK-BASE-20261021/REPORT.md` | 1 | `docs/30_任务卡库/归档/2026-10-21.md` |
| `_tmp_session/TRACK-ACK-PLAN-20261021/CARD.md` | 1 | `docs/30_任务卡库/归档/2026-10-21.md` |
| `_tmp_session/TRACK-PAPER-MERGE-20261021/CARD.md` | 1 | `docs/30_任务卡库/归档/2026-10-21.md` |
| `_tmp_session/TRACK-VIS-20261021/attribution_matrix.md` | 1 | `docs/30_任务卡库/归档/2026-10-21.md` |
| `_tmp_session/TRACK-YAW-20261021/diff_draft.patch` | 1 | `docs/30_任务卡库/归档/2026-10-21.md` |
| `_tmp_session/TRACK-ACK-PLAN-20261021/CARD.md:12` | 1 | `docs/30_任务卡库/归档/2026-10-21.md` |
| `_tmp_session/paper50/lead_notes.md` | 1 | `docs/30_任务卡库/归档/2026-10-21.md` |
| `_tmp_session/AG-TEAM-PLAN-20261023/backups与baseline.json` | 1 | `docs/30_任务卡库/归档/2026-10-23_AG-TEAM-PLAN-20261023.md` |
| `_tmp_session/AG-TEAM-PLAN-20261023/backups/的priority_前缀文件` | 1 | `docs/30_任务卡库/归档/2026-10-23_AG-TEAM-PLAN-20261023.md` |
| `_tmp_session/t35/REPORT.md:86-90` | 1 | `docs/30_任务卡库/模板/任务卡模板.md` |
| `_tmp_session/TRACK-PLAN-B1-20261021/REPORT.md:171` | 1 | `docs/30_任务卡库/模板/回报模板.md` |
| `_tmp_session/TRACK-FIELD-R3-20261020-01/review.md:19-31` | 1 | `docs/30_任务卡库/模板/回报模板.md` |
| `_tmp_session/TRACK-FIELD-B5-20261021/REPORT.md:89-99` | 1 | `docs/30_任务卡库/模板/回报模板.md` |
| `_tmp_session/TRACK-BASE-20261021` | 1 | `docs/_work/plan_库外清理.md` |
| `_tmp_session/aim_audit/体检报告_20261018_1741.md:37-41` | 1 | `docs/_work/timeline_doc-airlink.md` |
| `_tmp_session/gimbal-patch/REPORT.md:13-17` | 1 | `docs/_work/timeline_doc-airlink.md` |
| `_tmp_session/t3/audit/FLOW_COMPARISON.md:13-21` | 1 | `docs/_work/timeline_doc-airlink.md` |
| `_tmp_session/t34/REPORT.md:11-18` | 1 | `docs/_work/timeline_doc-cards.md` |
| `_tmp_session/t_d1b/REPORT.md:41` | 1 | `docs/_work/timeline_doc-cards.md` |
| `_tmp_session/TRACK-FIELD-R2-20261020-01/review.md:33-40` | 1 | `docs/_work/timeline_doc-cards.md` |
| `_tmp_session/agent_overview_20261020/drone.md:288` | 1 | `docs/_work/timeline_doc-drone.md` |
| `_tmp_session/agent_overview_20261020/drone.md:154、:314` | 1 | `docs/_work/timeline_doc-drone.md` |
| `_tmp_session/agent_overview_20261020/drone.md:3-5` | 1 | `docs/_work/timeline_doc-drone.md` |
| `_tmp_session/yb_tree/docs/archive/进度报告-20260927.md:1` | 1 | `docs/_work/timeline_doc-ops.md` |
| `_tmp_session/yb_tree/docs/会话交接-20261002.md:1` | 1 | `docs/_work/timeline_doc-ops.md` |
| `_tmp_session/yb_tree/docs/（10-12` | 1 | `docs/_work/timeline_doc-ops.md` |
| `_tmp_session/yb_tree/docs/战略评估-省一冲刺-20261014.md:1` | 1 | `docs/_work/timeline_doc-ops.md` |
| `_tmp_session/TRACK-BASE-20261021/` | 1 | `docs/_work/timeline_doc-ops.md` |
| `_tmp_session/PC-01-提示词压缩/账本.md:10` | 1 | `docs/_work/timeline_doc-prompts.md` |
