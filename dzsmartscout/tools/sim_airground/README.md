# sim_airground — 空地联合仿真（t11 + t28 四项改进）

无人机天眼模型 × 车侧循迹仿真（`dzbrain.tools.sim.run_sim` **只读 import**）的联合仿真器：
把无人机建模为**移动情报源**——N8 起飞 → 与小车同向的 U 形航线（含 K 列直飞侦察腿）→
M2 降落；每拍揭示以自身为中心半径 2.5 格（0.875 m）的圆形区域；情报按 t2 感知管线协议
JSON（`bridge_passable`/`target_zone`/`field_verify`，dzlink 契约）经 0.06 s 量级延迟与
可配丢包注入小车；小车侧采纳门与 t6 `BridgeIntel` 对齐（缺/过期/离线 → 回落
`route.cross_lower_default=K10 / cross_upper_default=K5`）。

## t28 四项改进（用户验收口径）

1. **单图双路线**：每次运行**恰好 1 张** `route_map.png`，同一坐标系同时画小车路线
   （蓝·参考 token 折线）+ 小车真值轨迹（红）+ 无人机航线（青）+ K 列侦察腿（紫），
   图例区分（自检按精确色值数像素钉住）。
2. **车位姿标注**：真值轨迹上均匀抽**定死 60 个位姿点**（短轨迹按 steps 数自动减少），
   画车体矩形（0.28×0.135 m）+ 车头朝向箭头；**位姿 = 真值口径**
   （`truth_x/truth_y/truth_yaw`，图例明示"车位姿(真值)"），不与信念位姿混用
   （t21 教训：信念↔真值 RMS 误当横向 RMS）。
3. **靶坐标/方向设定**：`--targets "H11 I11"` + `--target-orient DEG`（或
   `--targets-file JSON`）只改**内存**情景（truth 靶格 + 射击模型 + 规划器地图副本），
   ⛔ 绝不写 `dzrobot_v2/config/map.yaml`；冲突校验（雷/障/桥洞格、不相邻、越界）抛
   ValueError → exit 2。
4. **一键随机地图批量调试**：`--random-maps N --seed-base X` 按 t11 规则随机
   （入口 K7~K11 × 出口 K1~K6 × 靶朝向 4 种），每张跑最佳路线（真规划器）+ 全仿真 +
   1 张路线图；输出明细行 + 分量级统计（平均计算时间 = 规划墙钟全样本均值；
   平均完赛时间 = 完赛样本 commit+drive+wait+dwell 均值，**失败/超时样本单列**）。

## 运行（Windows/PowerShell，工作区根 `D:\User\Desktop\比赛存档\v1`）

```powershell
$env:PYTHONPATH='D:\User\Desktop\比赛存档\v1\dzrobot_v2\src\dzbrain'
$env:PYTHONIOENCODING='utf-8'; $env:PYTHONUTF8='1'
python dzsmartscout\tools\sim_airground\run_airground.py --selfcheck   # 确定性自检（exit 0）
python dzsmartscout\tools\sim_airground\run_airground.py               # 全臂矩阵（18 臂）
python dzsmartscout\tools\sim_airground\run_airground.py --quick       # 四臂快检 A1/A2/B1/B3
python dzsmartscout\tools\sim_airground\run_airground.py --arms B3,C1  # 指定臂
# t28③ 靶坐标/方向（只机侧参数化）:
python dzsmartscout\tools\sim_airground\run_airground.py --arms b3 --targets "H11 I11" --target-orient 0 --no-stats
# t28④ 随机地图批量（先 10 张试点再 50 张）:
python dzsmartscout\tools\sim_airground\run_airground.py --random-maps 10 --seed-base 20261012
python dzsmartscout\tools\sim_airground\run_airground.py --random-maps 50 --seed-base 20261012
```

- 固定种子（`SimConfig.seed=20261007`、链路种子 `LinkParams.seed`、随机图
  `seed_base + i`）→ **全确定性**：同参复跑 `traj.csv` 的 sha256 逐字节相同（自检钉住）。
- 输出：每臂 `_tmp_session/sim_ag/<runid>/<arm>/` 下 `ledger.json` + `traj.csv` +
  **恰好 1 张** `route_map.png`；总台账 `ledger.json` + `console.txt`；
  随机批量在 `<out-dir>/random_maps_n<N>/`（逐图 PNG + `random_maps_summary.json`）。
- 随机批量遇到"规划器判该桥洞组合无解"**不再崩溃**：如实记一行 `plan_failed_rows`
  （`plan_ok=false` + 错误摘要，仍渲染 1 张无车路线图），批量继续跑完；画图/统计口径不变。
  批量内**靶模型（track_cells/stop_candidates）跟随真值靶格**（2026-10-17：地图靶= E5/F5，
  旧实现只改朝向不改靶格，会让射击模型按 B11/C11 判扇区 → 假 `sector_fail`）。
- exit 0 = 全部正常；exit 2 = 参数问题（与 `route_check.py` 同约定）。
- `--no-stats`：跳过 360 场解析统计段（约省 6 分钟），臂与台账照常。

## 2026-10 最新参数口径（队长两轮澄清，取代任务书旧假设）

| 项 | 口径 | 出处 |
|---|---|---|
| 无人机 | 高 1.5 m、感知半径 2.5 格、速度 **0.5 m/s**、U 形航线、无雷障桥约束 | 队长消息 ②③ |
| 总预算 | **3 分钟** = 车全程 + 机全程（伴飞并发）+ 打靶 ≤30 s，超时判负 | ④ |
| 桥洞 | **每场随机**：入口 K7~K11、出口 K1~K6（默认 K10/K5 只是范围合法兜底） | ③ |
| 靶 | 一块**可动双面板**（蓝/红各一面，很薄 ≈0.4 格宽）+ 水平轨道两格，0.1 m/s 来回，朝向每场随机；打蓝面得分、打红面**扣分**；有效性 = **命中时**车→靶连线落在蓝面法线扇区（±45°，敏感度 ±30/±45/±60）；蓝面朝向需无人机判读（YOLO 口径） | ④/② |
| 桥顶箭头 | 画了箭头的垫片（与雷同物理形态），YOLO 可解；判据 = **H1：箭头贴哪半场 = 该半场通行格** | ⑤ |

## 臂矩阵（World A/B/C）

- **A**（桥洞=默认 K10/K5；靶蓝面北）：`A1` 无无人机基线（等价现基线）、`A2` 理想揭示、
  `A3` 链路受限（丢包 0.5）。
- **B**（随机桥洞 K9/K4；靶蓝面东）：`B1` 无无人机直冲中断、`B2` 理想无人机+**现车代码**
  （情报被采纳门拒）、`B3` 改进变体（权威门+L10 桥前等待+重规划）、`B4` 起飞晚 8 s+等待
  上限 3 s、`B5` 靶朝向误判（蓝红翻转 180°）、`B6` 受限旗舰（随机桥洞+迟到+误判）、
  `B7` 迟到 8 s+现车代码（无桥前等待）。
- **C**（最坏组合 K11/K1；靶蓝面南=贴墙不可打）：`C1` 3 分钟预算上限压力测试。
- **MC1~4**：无情报直冲失败概率的蒙特卡洛抽样验证（解析 29/30）。
- **MV1~3**：drive_est 标定（6.883 s/m）的全仿真验证场。

## 指标定义（台账契约）

- 车完赛六项（跑完/时长/RMS/MAX/超调/越界）+ 吸附 + 物理在环两口径（盖雷几何
  `mine_pressed_count` vs 车体足迹 `obstacle_hit_steps`/`board_overlap_steps`）
  —— 全部取自 `run_sim` 契约指标（`docs/P0接口约定.md` §5）。
- 信息领先契约：`lead_rows`（每关键格揭示时刻 vs 车到达时刻）、`gap_key_ticks/gap_key_cells`
  （关键格缺口：车已在格内而该格未揭示的拍数/格数；全口径 `gap_*` 另报）。
- 改道次数：相对无情报基线的**执行路线版本更换次数**；`adopted_changes` 为采纳门级变化。
- 侦察账：`drone.recon_done_s`（侦察耗时，④新指标）、`drone.recon_deadlines`（每半场
  桥洞情报送达 vs 车到桥前决策点时刻的布尔值+裕量秒）。
- 预算账：`budget.total_s`（commit+drive+wait+dwell）≤ 180 与否 + `margin_s`（④总时间预算余量）。
- 射击账：`shoot.blue_hits/red_hits/score`（蓝中得分、红中扣分、净分）逐发按命中时刻扇区判。
- 多场次统计（`match_stats`）：30 桥洞组合 × 4 靶朝向 × 3 策略 → 完赛率/预算通过率/
  蓝面命中率/红面扣分率/均分/均裕量（解析统计 + MV 全仿真验证标定）。

## 模型边界（如实标注，交接包纪律）

1. **sim 不建模 PIVOT**（`sim.py:109-113` 刻意不做）：凡涉及折返/PIVOT 的结论标
   "仿真无法裁决"，需实车/闭环 bench 验。
2. **sim 不建模射击停站驻留与云台**：射击/驻留/扣分账由本包 `shoot.py` 解析模型给出
   （命中确定性、无弹道散布；面板相位锚定标注 ±2 s 不确定）。
3. 车侧任务流为**镜像**（`car_gate.py`，逐条 file:line 出处），不 import
   `dzbrain.tasks.mission`（其运行时状态机不在本仿真范围）。
4. 桥前等待/权威门是**改进变体**（现车代码只在发车窗口取一次情报，执行中不改 token）；
   所有含这两个开关的臂在台账里显式标注。
5. 轨迹时间轴把桥前等待**折叠**掉（run_sim 连续行驶），总时长 = commit+drive+wait+dwell
   相加；改道 splice 的前提 = 新旧路线前缀逐位相同（自检钉住现役 `N10 M10 M9`；2026-10-17 t6 换线后口径）。
6. 飞行高度 1.5 m 在 2D 投影下无几何效应（仅记录口径）；感知连续采样（帧率远高于 dt）。
7. 链路：单发布者 FIFO（BEST_EFFORT 真实语义）→ 不建模乱序，只建模延迟 0.06 s+抖动+丢包。

## 目录

```
sim_airground/
  geo.py            格名↔米制（车侧 geometry 公式逐字沿用）
  scenarios.py      情景：真值 vs 车侧静态信念 vs 机/链/任务/靶参数（只读 config，不落盘）
  drone_model.py    无人机航线（U 形+侦察腿）/揭示/箭头 H1/协议报文
  link.py           dzlink 延迟/抖动/丢包注入（种子固定）
  car_gate.py       车侧采纳门 + 任务流镜像（t6 口径，file:line 出处）
  carsim.py         run_sim 包装：指标/逐格到达/缺口/领先表 + 真值位姿序列（t28②）
  shoot.py          双面移动靶：扇区选点/逐发命中/扣分账/驻留（t28③ 任意相邻两格轨道）
  match_stats.py    ① 多场次随机抽取统计（30×4×3）+ plan_for_bridges（t28③ 靶格覆盖）
  random_maps.py    t28④ 一键 N 张随机地图批量（明细行 + 分量级统计，失败/超时单列）
  runner.py         臂编排 + 台账落盘 + 半径/速度/扇区敏感度 + 出图（t28①②）
  plot.py           route_map.png（Pillow；t28①② 双路线同图 + 60 个真值位姿）
  run_airground.py  主入口（--selfcheck/--quick/--arms/--out-dir/--targets/--random-maps）
  tests/            46 项确定性自检
```

## 复现与验证

```powershell
# 1) 自检
python dzsmartscout\tools\sim_airground\run_airground.py --selfcheck
# 2) 全矩阵（同一命令复跑 → 同 sha256）
python dzsmartscout\tools\sim_airground\run_airground.py --out-dir _tmp_session\sim_ag\rerun1
# 3) 逐字节可复现性：比较两次运行的 <arm>/traj.csv sha256 与 ledger.json
```

报告：`docs/空地联合仿真设计.md`。
