# dzrobot_v2 · 车端工作区

> 继承根 `AGENTS.md`（v3）全部铁律。本文件只放车端专属判断，不重复全局规则。

## 关键路径（实测）

| 你要干什么 | 去哪 |
|---|---|
| 改车端决策层 | `src/dzbrain/`（Python 8 线程：Localization/Tracker/Mission/Aim/Fix/Drone/Safety/Telemetry） |
| 改门禁工具 | `tools/wf/`（check.py / check_links.py / check_reuse.py） |
| 改配置 | `config/robot.yaml`（**冻结区**） |
| 跑仿真 | `python -m dzbrain.tools.sim_cli`（需 `PYTHONPATH=src/dzbrain`） |
| 跑全量回归 | `python _run_full_regression.py` |
| 发车判据 | G1~G4 见 `../docs/20_资料索引库/运维与排障/07_发车门禁口径_G1-G4.md`（2026-10-29 改指） |

## 车端冻结区（不许直接改默认值）

`src/dzbrain/dzbrain/**`、`src/dzrknn_dataprocess/**`、`config/**`、`tools/deploy_to_car.py`
实验一律进 `_tmp_session/` 或 feature flag。

## 车端专属判据

1. **改车端代码必须跑仿真闭环**：`sim_cli` 退出码 + 指标表。单元测试绿不算过。
2. **改 config 必须跑 G1**：`python tools/config_selfcheck.py --config config/robot.yaml --map config/map.yaml`，判据是 error 0 ∧ warn 0，**不是只看退出码**。
3. **部署唯一入口 `tools/deploy_to_car.py`**（D-019），禁止手工 scp 散文件。
4. **标定真源**：ticksPerMeter=3280.0（冻结）、ticksPer2PI=2670.0（待仲裁；2026-10-29 由 2680 下调后回调至 2670）、yaw_trim=-0.6。改标定 = 必须实车复标。
5. **门禁 GREEN ≠ 回归绿 ≠ 实车通过**：三件事三套判据。
6. **已知挂账红**：现行台账 `../docs/20_资料索引库/运维与排障/08_回归守护_known_reds.md`（K1~K9，2026-10-29 重建）；裁决另见 `docs/决策台账.md` 与 `docs/TEAM_STATUS.md`（旧原件在 `../docs/history_cache/01_旧文档归档/dzrobot_v2/docs/`，不再维护），不许当成新问题重开。