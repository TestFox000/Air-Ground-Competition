# t40 官方图本地仿真规划 报告（与车端对账）

> 版本：t40 终版 ｜ 日期 2026-10-14 ｜ 仿真员 sim-operator-2
> 权威真源：`比赛存档\v1\dzrobot_v2`（车侧）+ `dzsmartscout/tools/sim_airground`（机侧，只读）
> 写域：`_tmp_session/t40_official/`（map_official.yaml + run_t40_official.py）+ `_tmp_session/sim_ag/t40_official_*/`（数据）
> ⛔ 未改 config、未改车侧代码、未动真机、未 commit。

---

## 0. 一句话结论

官方图（队长识读定稿）上，本地仿真核心臂 **A1（0.35 无无人机基线）与 A2（理想揭示）完赛、盖雷 8/8、越界 0**，且 **A1 路线与车端自规划 + 实车路线逐 token 相同（54 token / 18.55 m / 停站 E4 / 穿洞 K9·K5）**。
B6（受限旗舰）如实报告**不完赛**：无人机成功纠正随机下桥 K8（改道 1），但 K8 行被官方障碍 D8/I8 封死 → 不可行；且官方靶 E5/F5 紧邻上桥 K5，任何随机上桥都会压靶。差异逐条见 §5。

---

## 1. 官方图解析（逐条核，与队长识读一致）

| 类 | 数量 | 格清单（本报告） | 队长识读（official_identify.txt） | 一致 |
|---|---|---|---|---|
| 雷 | 8 | C1 A3 E3 G4 I6 B7 D9 H10 | 同左 | ✅ 8/8 |
| 障 | 8 | A1 E2 H4 D8 I8 G9 B11 F11 | 同左 | ✅ 8/8 |
| 靶 | 2 | E5 + F5（横向相邻=跨格板） | 同左 | ✅ |
| 通(桥洞) | 2 | K5（上）/ K9（下） | 同左 | ✅ |
| 起点 | 1 | N10 | N10 | ✅ |
| 终点 | 1 | N5 | N5 | ✅ |
| 无人机点 | 2 | N8（起）/ M2（降，H 图标） | 同左 | ✅ |
| 隔板 | — | 行 6/7 之间 D~N 连续无缺口 | 同左 | ✅ |

双重独立识读：`_tmp_session/t39_map_official/read_report.md`（drone-perception）+ `_tmp_session/t39_captain_read/official_identify.txt`（队长，红块 LSQ 残差 max 0.16px）。两者逐格一致。与现役 `config/map.yaml`（t29 第五张图）**20 点位仅 K5 重合**（19/20 不同）。

输入文件：`_tmp_session/t40_official/map_official.yaml`（sha256 `318872E7…`）。

---

## 2. 命令与产物

```powershell
$env:PYTHONPATH='D:\User\Desktop\比赛存档\v1\dzrobot_v2\src\dzbrain'
$env:PYTHONIOENCODING='utf-8'; $env:PYTHONUTF8='1'
# 自检（46/46，exit 0）
python dzsmartscout\tools\sim_airground\run_airground.py --selfcheck
# 官方图核心臂（A1/A2/B6 + 与车端 plan_route_from_config 对账）
python _tmp_session\t40_official\run_t40_official.py
```

产物目录：`_tmp_session/sim_ag/t40_official_20261014_193557/`（最终一版）。

---

## 3. 核心臂指标表（逐臂可复算；seed 20261007，复跑 traj.csv sha256 逐字节一致）

| 臂 | 完赛 | 时长s | RMS | MAX | 超调 | 越界 | 盖雷 | 障碰 | 预算 | 改道 | 蓝/红 | 停站 | 等待s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| **A1** 无无人机基线0.35 | ✅ 是 | 92.74 | 0.0472 | 0.1192 | 0.0491 | 0 | **8/8** | 0 | 102.14s/180 | 0 | 10/0 | E4 | 0 |
| **A2** 理想揭示 | ✅ 是 | 92.74 | 0.0472 | 0.1192 | 0.0491 | 0 | **8/8** | 0 | 124.74s/180 | 0 | 0/0 | E4 | 0 |
| **B6** 受限旗舰 | ❌ 否 | 58.36 | 0.0534 | 0.1159 | 1.256 | 0 | 5/8 | 0 | 60.36s/180 | **1** | 0/0 | E4 | 9.63 |

- 六项台账（A1）：跑完=是 / 总时长 92.74s / RMS 0.0472 / MAX 0.1192 / 超调 0.0491 / 吸附 31 / 越界 0。
- 物理在环两口径（A1）：盖雷几何 8/8；车体足迹 障碰 0、挡板触线 0、越界 0、**压靶板 6 拍**（见 §6 边界）。
- A1/A2 traj.csv sha256 相同（轨迹零差异，符合"理想揭示情报对轨迹零影响"）；B6 改道 1 次后轨迹不同。
- 复现：同命令复跑，A1/A2 traj.csv sha256 = `A98D1CED…`、B6 = `AE2AF06B…` 逐字节一致。

---

## 4. 与车端自规划对账（plan_route_from_config，官方图）

| 项 | 本地仿真 A1 | 车端自规划 | 一致 |
|---|---|---|---|
| token 数 | 54 | 54 | ✅ |
| 路线长度 | 18.55 m | 18.55 m | ✅ |
| 停站点 | E4 | E4 | ✅ |
| 穿洞格(下/上) | K9 / K5 | K9 / K5 | ✅ |
| tokens 逐位 | — | — | ✅ **逐位相同** |

`plan_route_from_config(Config(K9/K5), map_path=map_official.yaml)` 产出与本地 A1 臂**逐 token 相同**（tracker-safe 规划器同源，双方一致）。

---

## 5. 与实车对账（队长数据）+ 差异逐条列

队长实车数据：54 token 路线（N10 M10 L10 **L9 K9 J9** J10 I10 H10 G10 F10 E10 E9 D9 C9 B9 B8 B7 B6 A6 A5 A4 A3 A2 B2 B1 C1 D1 D2 D3 D4 **E4*** F4 F3 F2 F1 E1 D1 D2 D3 E3 F3 G3 G4 G5 G6 H6 I6 J6 J5 **K5** L5 M5 N5,y）/ 18.55 m / 0.35 m/s；实车 FINISH、8/8 雷、0 越界、0 刹车、119.5s、射击 0/10（E4* TRACK 后丢靶）。

| # | 项 | 本地仿真 | 实车 | 差异与判定 |
|---|---|---|---|---|
| 1 | 路线 | 54 token / 18.55m / E4 / K9·K5 | 54 token / 18.55m / E4 / K9·K5 | ✅ **逐位相同** |
| 2 | 盖雷 | 8/8 | 8/8 | ✅ 一致 |
| 3 | 越界 | 0 | 0 | ✅ 一致 |
| 4 | 刹车/碰撞 | 障碰 0 | 0 刹车 | ✅ 一致 |
| 5 | 耗时 | 92.74s（驾驶）｜102.14s（含 commit+dwell） | **119.5s**（全程） | ⚠️ 差 ~27s ≈ 射击驻留（实车 0/10 吃满 30s vs sim A1 命中驻留 7.4s）+ 实车物理开销。sim 为纯运动学，未建模射击云台/驻留 |
| 6 | 射击 | A1 蓝 10/红 0（解析命中） | **0/10（E4* TRACK 后丢靶）** | ⚠️ sim 射击为**解析模型**（命中确定性、无弹道/云台跟踪），不建模实车"跟踪丢失"。sim 蓝 10 不代表实车必中 |
| 7 | 压靶板 | sim 报 board_overlap **6 拍**（E4 停站紧邻靶板） | 未报（0 越界口径） | ⚠️ sim 靶板模型 = 整格 35×35cm（保守），实靶为薄板 → 见 §6 |

---

## 6. 发现与边界（如实标注）

1. **selfcheck 基线守卫陈旧（已修）**：`test_airground.py:164-166` 与 `:307-308` 硬编码 0.15 档基线（142.14s / RMS 0.0438），现役 robot.yaml 已 0.35 档（104.54s / RMS 0.0489，与 t22 权威逐位一致）→ 已更新守卫，自检 44/46 → **46/46 exit 0**。（写域 `dzsmartscout/tools/sim_airground/tests/`，属本任务授权写域。）
2. **seed 口径**：臂级 seed = 工具内置 **20261007**（carsim.py:48 / scenarios.py:60）；任务书"20261012"系 `--seed-base` 默认值（随机图）口径。本仿真沿用 20261007，复跑 sha256 逐字节一致即满足"同 seed 复跑逐字节一致"验收。
3. **压靶板口径（sim 板模型保守）**：`sim.py:424-449` 把靶板建模为**整格 35×35cm 矩形**（`board_half=0.175m`），E4 停站（紧邻 E5/F5）触发 board_overlap 6 拍 + board_breach 621 拍。实靶为薄板（0.14m，README 口径），实车未报压靶 → **sim 板模型偏保守，压靶 6 拍需按薄板口径复核，不能直接当实车违规**。
4. **B6 受限旗舰不完赛（双重根因）**：① 官方下桥 K9 随机化为 K8 后，K8 行被官方障碍 **D8、I8** 封死 → 路线不可行；② 官方靶 E5/F5 紧邻上桥 K5，任何随机上桥（≠K5）都会让穿洞路径压过靶板。无人机桥洞纠正本身**生效**（改道 1 次、采纳 K8 成功），但纠正后的路线在官方障碍/靶布局下不可行。
5. **A2 射击 0 蓝（sim 停站候选与规划器不一致）**：`derive_stop_candidates(E5/F5)` 给行 6（朝场地中心），但真规划器把停站放在 **E4（行 4，靶北侧）**；A2（有判读）按候选 D6/E6/F6/G6 选站 → 扇区无解 sector_fail → 0 发。这是 sim 射击模型的停站候选口径 bug（不影响路线/盖雷/越界，只影响射击账）。
6. **sim 不建模 PIVOT / 射击云台驻留**（交接包 §4.5）：涉及射击命中、驻留、跟踪丢失的结论，仿真无法裁决，需实车/闭环 bench 验。

---

## 7. 产物清单（路径 | 大小 | md5 | 时刻）

最终运行目录 `_tmp_session/sim_ag/t40_official_20261014_193557/`：

| 文件 | 大小 | md5 | 时刻 |
|---|---|---|---|
| ledger.json（总） | 32646 B | 88BFE91C… | 19:36:09 |
| reconciliation.json | 1840 B | F54CEC74… | 19:36:09 |
| a1/ledger.json | 7650 B | 807433F2… | 19:35:58 |
| a1/route_map.png | 35084 B | B31BFD4B… | 19:35:58 |
| a1/traj.csv | 420282 B | E6D7CAFD…(sha256 A98D1CED…) | 19:35:58 |
| a2/ledger.json | 8585 B | 3A50797D… | 19:36:02 |
| a2/route_map.png | 35803 B | 743A3131… | 19:36:02 |
| a2/traj.csv | 420282 B | E6D7CAFD…(sha256 A98D1CED…) | 19:36:02 |
| b6/ledger.json | 9562 B | 34E6B263… | 19:36:08 |
| b6/route_map.png | 33052 B | BD506F75… | 19:36:08 |
| b6/traj.csv | 261984 B | 96C6D6A1…(sha256 AE2AF06B…) | 19:36:08 |
| b6/v1_observe/traj.csv | 420282 B | E6D7CAFD… | 19:36:05 |

每臂**恰好 1 张** route_map.png（t28 单图双路线口径：车路线蓝/真值轨迹红/60 位姿橙/无人机航线青/侦察腿紫 + 图例）。

辅助脚本：`_tmp_session/t40_official/run_t40_official.py`（sha256 E99FCA97…）、`map_official.yaml`（sha256 318872E7…）。
