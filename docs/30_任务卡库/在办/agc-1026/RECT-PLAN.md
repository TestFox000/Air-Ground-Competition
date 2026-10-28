---
卡号: RECT-PLAN
ns: agc-1026
agent: plan-analyst
owner: plan-analyst
reviewer: lead
树节点: R-4
写域: "_tmp_session/AGC-RECTIFY-20261026/plan/"
status: in_progress
创建: 2026-10-26
---

# RECT-PLAN · 比赛目标规划与运动阻塞复核

## 目标

用户本轮目标：**优先完赛**，在安全与时间余量内尽量压雷、取两个有效最佳射击点并命中；不降为仅循迹。共享任务 `task-3`（claim 时 revision=1→2，owner=plan-analyst）。生产规划器在途只读：不另造规划器、不部署、不发车、不出光、不飞行、不 commit/push。

## 答复 Lead 三问（结论先行）

### Q1 · 第六图「只找到 1 个射击停站」是物理最优，不是搜索不足

**结论：受几何 + 隔板视线所限，不是 tour 搜索漏找。** 判据是对 `shoot_candidates()` 的**全量枚举**（门禁同源判据），再加对**全图所有可通行格**的暴力复核。

窗 = 与 `route_check` 同口径 `[0.78, 1.02] m`（板心距离；`config/robot.yaml:688-689` `route.shoot_distance_m=0.90` / `tol_m=0.12`）。第六图靶 `D5+D6` 沿 x 跨两格，板心 `(2.10, 3.675)`，板面法向 `(0,-1)`：

| 候选格 | 板面侧 | 到 D5 | 到 D6 | 到板心 | 两靶都在窗内 | 是否入池 |
|---|---|---|---|---|---|---|
| `B4` | −1 | 0.7826 | 0.9899 | 0.8750 | 是 | 是 |
| `B7` | −1 | 0.9899 | 0.7826 | 0.8750 | 是 | 是 |
| `E3` | **+1** | 0.7826 | **1.1068** | 0.9424 | 否（D6 超窗 0.0868） | 是（仅覆盖 D5） |
| `C3` / `C8` | −1 | 0.7826 / 1.1068 | 1.1068 / 0.7826 | 0.9424 | 否 | 是（各覆盖 1 靶） |
| `D3` / `D8` | 0 | 0.7000 | 1.0500 | 0.8750 | 否 | 是（覆盖 0 靶，与板面共面） |
| `F7` | **+1** | 0.9899 | 0.7826 | — | 是 | **否：视线被隔板挡（两靶均挡）** |
| `F6` | +1 | — | — | 0.7215 | — | **否：0.7215 < 0.78，太近** |

两条硬结论（都可复跑）：

1. **能同时覆盖两靶的合法点只有 `B4`/`B7`，两者在板面同一侧（−1）**；`second_shoot_stop()` 要求「异侧 + 覆盖全靶」，因此返回空串 —— 不是没搜到，是**池里没有**。
2. 唯一在另一侧、且两靶都在窗内的格子是 `F7`；它对 `D5`/`D6` 的视线**都被第 6/7 行之间那道 D~N 连续隔板挡住**（`barrier_sight_blocked()` 对两靶均返回真）。也就是说这张图上「另一面」根本看不见靶，坐上去也打不了。

复跑：`python -B -X utf8 _tmp_session/AGC-RECTIFY-20261026/plan/scripts/sixth_side_proof.py`（cwd=`dzrobot_v2`，rc=0）、`.../scripts/f7_los_probe.py`（rc=0）；证据 `plan/analysis/shoot_candidates.json`。

### Q2 · 硬门「双射击停站应为 2 个」确实把第六图卡成「无路可出」

**现状（实测）：**

| 口径 | 结果 | 后果 |
|---|---|---|
| `--objective legacy`（**生产默认**，见 D-012/D-016/D-023） | `plan_route` **rc=1**，stderr：`规划结果未通过结构自检：双射击停站应为 2 个，实际 1 个：B4` | **一条路线都出不来** ⇒ 完赛直接归零 |
| `--objective tasks` | rc=0，但 50 token、**射击停站 0 个**、`degraded` | 能跑完，但**靶覆盖 0/2**（`route_check` rc=1），就是用户明确否掉的「禁火整趟」 |

两条都不是可接受的竞赛形态。因此在第六图（以及任何「对侧无合法点」的图）必须二选一。

**推荐 A（窄口径放宽），且必须靠枚举证明，不能靠感觉：**

- 放宽条件**不是**「允许随时单站」，而是：**先对全候选池证明「不存在对侧且覆盖全靶的合法点」，才允许单站出车，并打一条显式 warning（写明池大小与被选站）。**
- 隔离候选实测（`plan/isolated/`，仅 32 行 diff，未进生产）：

| 校验 | 结果 |
|---|---|
| `plan_route --objective legacy`（第六图） | **rc=0**，68 token，`B4*`，3 处 PIVOT，24.85 m |
| `route_check`（生产工具，`ROBOT_MAP`=第六图） | **rc=0**：雷覆盖 **8/8**（最差 0.0000 m）、靶覆盖 **2/2**（`D5`←0.78 m、`D6`←0.99 m 同一停站）、障碍净空 0.350 ≥ 0.215/0.303、航点不压靶板、折返声明一致 3 |
| `sim_cli` 闭环 | rc=0，`ok=true`，**跑完全程 = 是**，108.24 s，末点误差 **0.0285 m**，路径长度比 0.9477，`diverged=false` |
| 对现役图回归 | 隔离副本 `carloop --objective tasks` 与生产产物 **63 token 逐 token 相同** |

- **会不会因此少压雷：不会，反而从 0 变 8/8。** 这条门只判「停站个数」，雷覆盖另有判据（`route_check` 任务层）。现状 legacy 直接不出路 ⇒ 0 个雷；放宽后 8/8 全压。不改雷/靶权重，不改射击距离窗，不动 `check_route_structure`。
- **代价与边界（如实说）**：单站在「蓝面朝对面」那一局**打不了**（板一面蓝一面红、蓝面每局随机但恒定，`planner.py:1198-1199`）。所以 A 必须配一条运行时闸：**看到红面就不出光**（现役自瞄有蓝/红靶类别，属 RECT-AIM 线）。否则可能出现「打了红面」的负分。
- **为什么不选 B（保持硬门 + 改搜索）**：B 在这张图上**没有可搜的解** —— 池里另一侧唯一的几何命中点 `F7` 视线被挡，`E3` 到 `D6` 超窗 0.0868 m。要让 B 成立只能改射击窗（`0.90±0.12`）或放宽视线判据，那是**改赛规口径 + 改门禁 R006/R013**，不是「改善搜索」，代价远大于收益。B 只在「池里确实有解、但 tour 没落站」时才有意义，本图不属于此类。
- **影响面**：仅 `planner.py:3305-3307` 调用处（+ 约 20 行）。不动 `config/**`（无需新冻结登记）、不动 `robot.yaml`、不动 Mission/StepLoop 既有的 `require_pair=False` 分段语义。

### Q3 · 「雷心压过风险」量化：是**车心**偏差，且风险点 1:1 落在 180° 折返顶点旁

- **口径先钉死**：压雷判据是**车心** 0.035 m 圆 ∩ 0.12 m 方垫片（`tools/route_check.py:931-959`；`MINE_PAD_M=0.12`、`COVERAGE_RADIUS_M=0.035`）。**与激光无关** —— 激光偏多少不参与压雷判定。warning 里「车心到雷心 3~8 cm」说的是**车体轨迹**偏差。
- **被点名的是哪几个雷、为什么**：carloop legacy 的 5 个告警雷 `H8 G9 A2 E3 F5` 逐个都是「雷格前共线只有 1 格」，而且**每个都在一处 PIVOT 顶点 1~3 个 token 之内**（`H8`/`G9` ← `G8:PIVOT`；`F5` ← `F6:PIVOT`；`A2` ← `A3:PIVOT`；`E3` ← `E4:PIVOT`）。即**风险不是随机分布，而是折返点的副产品**；`F9*` 这一站正好落在 `G8` 折返后的回程上，一个折返同时压住 `H8` + `G9` 两个雷（最大簇）。
- **3~8 cm 这个数字偏乐观**：规划器注释里的实测值是 90° 拐角 **2.7~3.1 cm**（`planner.py:213`），而 180° 折返点 **12.7 cm**（`planner.py:3560`，A9 实测）。本次 5 个风险雷全部挨着 180° 折返 ⇒ 应按 **12.7 cm** 量级看（≈3.6 倍于 0.035 m 判据），不是 3~8 cm。
- **闭环仿真实测（同一 0.035 m 判据，折线 vs 仿真真值轨迹）**：

| 路线 | 折线判据 | 仿真真值判据 | 差在哪 |
|---|---|---|---|
| carloop legacy | 8/8 | **8/8**（最差 0.0017 m） | 全部贴住 |
| carloop tasks | 8/8 | **7/8**：`B1` 离垫片 **0.2907 m**（超判据 25.6 cm） | 同一处 `A3` 折返，纯追踪甩出 29 cm |
| field legacy | 8/8 | 7/8：`C4` **0.0436 m**（超 0.86 cm，贴线过） | `C3:PIVOT` 旁 |
| 第六图单站（隔离候选） | 8/8 | **8/8** | 全部贴住 |

- **是否需要改射击站选择顺序**：**本轮不改**，理由可测而非审美：① 站点选择不是主因，折返顶点才是；② 换顺序确实会改变结果（同图同两站，`tasks` 顺序把 `B1` 从 0 压变成漏 29 cm），说明顺序**敏感**，但在**仿真不建模 PIVOT**（`sim.py:107-112`）前提下，这个差不能当「真车会漏」的证据；③ 真要动顺序，判据必须先有：**下一趟逐雷车心—垫片最小距离**。
- **第三种口径：连续轨迹审计（我独立复跑，rc=0 / `ok=true`）** —— `tools/route_check.py --route "<62 token>" --map src/dzbrain/tests/fixtures/map_carloop_20261024.yaml --trajectory --json`（28.0 s，`static_geometry_ok=true`，六类违例均 0）。它把折线编译成**连续轨迹**（生产 Path 语义）后逐采样判距离：

    | 量 | 值 |
    |---|---|
    | 轨迹长度 / 采样 / cusp | 18.80 m / 11764 / 11 |
    | 轨迹到原折线最大偏离 | 0.4313 m |
    | 航点最大误差 | **0.6598 m**（切角换来的「能过」） |
    | `strict_mine_covered`（车心→**雷心** ≤0.035 m） | **6/8**，漏的正是 `C3`（7.50 cm）与 `E3`（6.90 cm） |
    | `pad_mine_covered`（车心 0.035 m 圆 ∩ 0.12 m **垫片**） | **8/8** |
    | 射击窗 | 两站均 0.875 m ∈ [0.78, 1.02] |

  ⇒ **压雷到底按哪个口径，会直接决定 6/8 还是 8/8**：文档化的生产判据是「车心圆 ∩ 垫片」（`tools/route_check.py:931-959`）＝ 8/8；「车心→雷心」是更严的保守口径 ＝ 6/8。两个口径必须并列写，不许只报对自己有利的那个。
- **注意：规划器那条 warning 是启发式代理，与几何审计结果不一致。** warning 点的是 `H8 G9 A2 E3 F5`（按「进点前共线格数」），几何审计真正偏离的是 `C3` 与 `E3`（按连续轨迹采样）。两者只有 `E3` 重合 ⇒ **不能拿 warning 当漏压清单**；下一趟要按几何判据量。
- **下一趟测量判据（可直接进发车单）**：从 bag/日志取真值车心轨迹，对 8 个雷格算两个数并同时上报 —— ① 到 0.12 m 垫片的最近距离（生产判据，`≤0.035 m` 记压到）；② 到雷心的最近距离（保守判据）。重点看几何审计已经偏的 `C3`、`E3`，再看 warning 名单 `H8 G9 A2 F5`。若 ② 超过 0.05 m 且 ① 仍过，属「擦边压到」，要写进下一趟风险项，不要当成 8/8 全压。
## 与前序执行者证据包的关系（必须声明）

本卡写域内已有**更早一次 RECT-PLAN 执行**留下的证据包：`_tmp_session/AGC-RECTIFY-20261026/plan/20261026_191000/`（其 README 自记执行者 `rect-plan-analyst`、子会话 `session-198acfc4-...`，产出时间 19:10~19:36）。处理口径：

- **核心数字我独立复跑并一致**：carloop 62 token / 22.75 m / `F9* C5*` / route_check rc0 / 雷 8/8 / 靶 2/2；第六图 legacy rc=1 且失败文案相同；第六图 tasks 50 token / 0 站 / 靶 0/2；sim 112.50 s / 104.64 s 与末点误差 0.0652 / 0.0640 —— 逐项相同。
- **它比我多做的三项，我按「前序自报 + 我已复跑的部分」分开记**：① `test_dual_shoot.py` 35/35（**我未复跑**）；② 连续轨迹审计（**我已独立复跑，见 Q3 第三口径，与它一致**）；③ C11/C12/yaw 三条（**我未复跑**，结论方向与我卡内 UNKNOWN 判定一致：C11 是单位不是尺度缺陷 ⇒ 不接 scale 修正；C12 可构造但当前 launch 集不可达；yaw 缺同步信号不称假毛刺）。
- **不把它当独立验收**：同卡同写域的前序产物属作者自报，不能替代 Lead 的独立复跑（验收纪律 §11.2）。我卡内所有「PASS」只对我亲自跑过的命令成立。
- 该包另有两条我未复跑但有决策价值的读数：`config/robot.yaml:780` `auto_plan_enabled: true`（2026-10-25 起零地图+布局模式）、`:715` `shoot_timeout_s: 30.0`；据此「112.5 s 行驶 + 2 站 × 30 s = 172.5 s / 180 s」只有 **7.5 s（4%）** 最坏余量（我按同两键独立算过，算式成立；sim 不建模 FIRE 停留与 PIVOT，112.5 s 是下界）。

## 读过的台账条目

`dzrobot_v2/docs/决策台账.md`：

- **D-008**（冻结区）：`src/dzbrain/dzbrain/**`、`config/**` 冻结 ⇒ 本轮全部只读，候选补丁只进 `_tmp_session` 隔离副本。
- **D-012**（唯一引擎）：`planner.engine=tour`；`global_planner.py` 不得转正 ⇒ 全程只用 `tools/plan_route.py`（tour）+ `tools/route_check.py` + `sim_cli`，未新建规划器。
- **D-014 / D-017 / D-020**（Go 门）：Go 只要「首段可靠」，全场 layout 是运行中里程碑；等全场反而 188.6 s 超时 ⇒ 本轮不要求 154 格齐，按首段口径评估。
- **D-022**（布局过期）：步进模式过期只拦新规划 → `WAIT_INFO(layout_stale)`；完整布局例外防「无人机掉线后车停半路」。
- **D-023**（回归红线与 C5）：地图纳管**不做**（每局重侦察），第六图只作回归夹具；L2/L3 禁重钉。
- **D-027**（阿克曼 / PIVOT 物理前提）：PIVOT 不是原地转，是同格前进弧；`localization.py` 的「差速可原地转」前提与阿克曼结论冲突 ⇒ 本卡对 PIVOT 相关结论一律标 UNKNOWN，不拿仿真当实车证据。
- **D-028**（执行基线）：不改火控/规划参数。
- **D-033**（靶速 0.3 m/s）：本卡未重开口径；旧 0.04 / 5.6 px/s 注释不参与任何计算。
- **D-044**（提交/推送）：未 commit、未 push、未部署。
- 资料库：[车端/03 规划器与自动规划](../../../20_资料索引库/车端/03_规划器与自动规划.md)（§7 双最佳射击点；末条「默认地图冲突必须暴露」）。

## 复用检索结果

- `docs/README.md` → `doc_search` 命中：[车端/03 规划器与自动规划](../../../20_资料索引库/车端/03_规划器与自动规划.md)（双站规则/窗口/默认地图冲突）、[赛事与治理/09 工具复用与能力登记](../../../20_资料索引库/赛事与治理/09_工具复用与能力登记.md)（`test_dual_shoot.py` + `map_carloop_20261024.yaml` 夹具）、[交接_20261024_空地协同接手](../../../00_项目总览/交接_20261024_空地协同接手.md)（两趟实跑与未验收项）。
- `dzrobot_v2/docs/复用清单.md` → `能力索引.md` → `目录索引.md`：规划唯一入口 `src/dzbrain/dzbrain/route/planner.py`；CLI `tools/plan_route.py`；校验 `tools/route_check.py`；仿真 `python -m dzbrain.tools.sim_cli`；双站专项 `src/dzbrain/tests/test_dual_shoot.py`。
- **结论：零个新永久轮子**；三个一次性诊断脚本 + 一个隔离候选补丁（见下），全部落在本卡写域。

## 新增轮子及理由

**无永久工具、无新规划器。** 现有工具不足处，用本卡 `_tmp` 内一次性脚本补齐（不进 `tools/`、不登记复用清单、不参与门禁）：

1. `plan/scripts/analyze_stops.py` —— 现有 CLI 不输出「全候选池 + 逐靶距离 + 板面侧 + 视线」；`plan_route` 失败时只给一句「实际 1 个：B4」，无法区分「物理无解」与「搜索没找到」。本脚本只调用 `planner` 既有函数（`shoot_candidates`/`covering_stops`/`plate_side_of`/`barrier_sight_blocked`），不重算任何规划。
2. `plan/scripts/mine_press_quant.py` —— `route_check` 只判**折线**；要回答「仿真真值轨迹是否也压到」，把同一 0.035 m / 0.12 m 判据套到仿真轨迹 CSV 的真值列（`truth_x` / `truth_y`；复用 `route_check._phys_seg_rect_dist`，未另写几何）。
3. `plan/scripts/rc_v2.py` / `rc_sim_rerun.py` / `isolated_probe.py` / `f7_los_probe.py` / `sixth_side_proof.py` —— 批量取证与隔离副本驱动（仅 subprocess 调度，无算法）。
4. `plan/isolated/` —— 第六图候选补丁的**隔离副本**（`cp -r src/dzbrain`，4.9 MB）；生产 `planner.py` 未改（见回滚点）。

## 写域

`_tmp_session/AGC-RECTIFY-20261026/plan/` + 本卡。**生产代码零写入**：现存工具就地只读运行（`python -B` 不落 `__pycache__`），仿真产物全部 `--out-dir` 指到本卡 `_tmp`，隔离副本也在 `_tmp` 内。未碰共享树/台账/索引/`frozen.json`/`robot.yaml`/`config/**`。

## 验收命令与清单

环境统一：`cwd=D:/User/Desktop/比赛存档/v1/dzrobot_v2`，`PYTHONPATH=src/dzbrain`，`PYTHONIOENCODING=utf-8`，`python -B -X utf8`；返回码为**进程本身** rc（非管道尾部）。

- [x] **V1 现役图双站成立**：`tools/plan_route.py --map src/dzbrain/tests/fixtures/map_carloop_20261024.yaml --engine tour --objective legacy --emit json --quiet` → **rc=0**，62 token，停站 `F9* C5*`（`plan/logs/carloop_legacy.plan_route.log`）。
- [x] **V2 第六图 legacy 硬失败**：同上换第六图 → **rc=1**，stderr 含 `双射击停站应为 2 个，实际 1 个：B4`（`plan/logs/sixth_legacy.plan_route.log`）。
- [x] **V3 第六图 tasks 退化为禁火**：`--objective tasks` → rc=0 但 50 token / 0 停站；`route_check` **rc=1**（`靶覆盖 0/2`）（`plan/logs/sixth_tasks.route_check_v2.log`）。
- [x] **V4 候选池穷举**：`scripts/analyze_stops.py` → 第六图池 7 个、覆盖全靶合法点 2 个（`B4`/`B7`，同侧 −1），另一侧 0 个（`plan/analysis/shoot_candidates.json`）。
- [x] **V5 全图暴力复核 + 视线**：`scripts/sixth_side_proof.py`（rc=0）、`scripts/f7_los_probe.py`（rc=0）→ `F7` 两靶视线均被挡；`F6` 到板心 0.7215 m 落在窗内不足侧。
- [x] **V6 隔离候选可出完整路线**：`cd plan/isolated && PYTHONPATH=$PWD/src/dzbrain python -B -X utf8 tools/plan_route.py --config config/robot.yaml --map fixtures/map_sixth_20261017.yaml --engine tour --objective legacy --emit report --quiet` → **rc=0**，68 token / 24.85 m / `B4*` / 3 处 PIVOT（`plan/isolated/candidate_patch.diff`，32 行）。
- [x] **V7 隔离产物过生产校验器**：`ROBOT_MAP=src/dzbrain/tests/fixtures/map_sixth_20261017.yaml python -B -X utf8 tools/route_check.py --config config/robot.yaml --route "<68 token 串>"` → **rc=0**：雷 8/8、靶 2/2、障碍净空 0.350（`plan/logs/sixth_legacy_isolated.route_check.log`）。
- [x] **V8 补丁对现役图零影响**：隔离副本 `carloop --objective tasks` 与生产产物 **63 token 逐 token 相同**（`plan/analysis/isolated_probe.json` 的 `determinism.identical=true`）。
- [x] **V9 闭环仿真对照**：`python -m dzbrain.tools.sim_cli --config config/robot.yaml --route <tokens> --out-dir plan/sim/<case> --json --quiet` → 六例 rc=0，指标见 E6。
- [x] **V10 压雷量化**：`scripts/mine_press_quant.py` → 折线 8/8；仿真真值 carloop legacy 8/8、carloop tasks 7/8（`B1` 0.2907 m）、field legacy 7/8（`C4` 0.0436 m）（`plan/analysis/mine_press_quant.json`）。
- [x] **V11 工作流门禁现状**：`python -B -X utf8 tools/wf/check.py` → **rc=0 / 结果 GREEN**（5 条「登记已失效」WARN；`_tmp_session/AGC-RECTIFY-20261026/plan/logs/wf_check.log`）。
- [x] **V13 连续轨迹审计**：`python -B -X utf8 tools/route_check.py --config config/robot.yaml --route "<62 token>" --map src/dzbrain/tests/fixtures/map_carloop_20261024.yaml --trajectory --json` → **rc=0**，`ok=true`、`static_geometry_ok=true`、六类违例 0；`strict_mine_covered=6/8`（漏 `C3` 7.50 cm、`E3` 6.90 cm）、`pad_mine_covered=8/8`、航点最大误差 0.6598 m（`plan/logs/trajectory_audit.out`）。
- [ ] **V12 Lead 独立复跑并认可**（作者不得自勾；另需 Lead 裁决 Q2 选 A 还是 B、以及 Q3 是否进下一趟发车单）。

## 回滚点

- **生产文件零改动**（开工与收口 `git status` 一致；本卡只在 `_tmp_session` 写）。前置哈希 `plan/preimage/tool_preimage_sha256.txt`：

| 文件 | sha256（前缀） |
|---|---|
| `tools/plan_route.py` | df856f69 |
| `tools/route_check.py` | c30d71be |
| `src/dzbrain/dzbrain/route/planner.py` | c14dadd4 |
| `config/robot.yaml` | d61effc3 |
| `fixtures/map_carloop_20261024.yaml` | e487d0e9 |
| `fixtures/map_sixth_20261017.yaml` | 445975bc |
| `fixtures/map_field.yaml` | f4e5c14e |

- 隔离副本补丁前后哈希：`c14dadd4` → `a09717f6`（`_tmp_session/AGC-RECTIFY-20261026/plan/isolated/preimage_planner_sha256.txt` / `_tmp_session/AGC-RECTIFY-20261026/plan/isolated/postimage_planner_sha256.txt`），diff = `_tmp_session/AGC-RECTIFY-20261026/plan/isolated/candidate_patch.diff`（32 行）。**要回滚只需删 `plan/isolated/`**；生产无需回滚。
- 基线：车仓开工 `945bbc14`、收口 `caaee3b1`（分支 `agents/dzhong/dzrobot-v2-20261018`）；根仓开工 `ecb9bdce`、收口 `db1b063f`。两处 HEAD 移动均为**他人提交**（`caaee3b1` = `wf(frozen)` 补登两张删图 + `localization.py`），不在本卡写域；本卡全程未跑任何 git 写命令。

## 执行记录

### E1 基线与前置哈希

`sha256sum <7 文件>` + 两仓 `git status --short` + `git rev-parse HEAD` 落 `plan/preimage/`。车仓在途改动 **33 项**（含 `route/planner.py`、`tasks/localization.py`、`config/robot.yaml`、`tools/route_check.py`、`tools/wf/frozen.json`），故全程只读运行。

### E2 三图 × 两口径矩阵（生产工具，只读）

`tools/plan_route.py --map <fixture> --engine tour --objective <legacy|tasks> --emit json --quiet`

| 图 | 口径 | plan rc | token | 停站 | `route_check` rc | 雷覆盖 | 靶覆盖 | 备注 |
|---|---|---|---|---|---|---|---|---|
| carloop（现役） | legacy | 0 | 62 | `F9* C5*` | **0** | 8/8 | 2/2 | 22.75 m，4 PIVOT |
| carloop | tasks | 0 | 63 | `F9* C5*` | **0** | 8/8 | 2/2 | 23.45 m，4 PIVOT |
| sixth | legacy | **1** | — | — | — | — | — | 双射击停站应为 2 个，实际 1 个：B4 |
| sixth | tasks | 0 | 50 | **无** | **1** | 8/8 | **0/2** | `degraded`，等于禁火整趟 |
| field | legacy | 0 | 54 | `E10*` | **0** | 8/8 | 1/1 | 单靶图不强制双站（`len(targets)<2`） |
| field | tasks | 0 | 53 | `I9*` | **0** | 8/8 | 1/1 | — |

证据：`plan/matrix_results.json`、`plan/analysis/route_check_v2.json`、`plan/routes/*.tokens.txt`。

### E3 工具坑 1：`route_check --route` 要「路线串」，不是文件路径

第一次传文件路径 → 被当成 1 个 token、解析失败（rc=1）。`--route` 语义是路线串；文件要自己读成一行。已在 `rc_v2.py` 修正并全部重跑。

### E4 工具坑 2：当前工作树没有 `config/map.yaml`，legacy 校验会「静默丢图」

两张地图 YAML（现役 map 与 v3 版）在工作树里是**已删除**状态（基线）。`tools/route_check.py:1252-1269` 的 `_find_map_file()` 顺序 = `ROBOT_MAP` → `config/map.yaml` → 都没有就**什么地图都不加载**。实测（不带 `ROBOT_MAP` 跑现役图路线）：

- `雷覆盖 未提供，跳过` ← **压雷判据整个被跳过**；
- 改用 `robot.yaml` 里的历史 `obstacle_blocks` ⇒ 报 `障碍净空不足 直段 0.000 < 0.215 m（航段#4 距 障I9）`、`拐角 0.000（航点#12 距 障E9）` —— 这两个都不是本图障碍，**假红**，rc=1。

**⇒ 发车前任何人跑 `route_check` 都必须带 `ROBOT_MAP=<本局 map>`，否则会拿到「雷覆盖没判 + 障碍假红」的结论。** 本卡不改工具（`tools/route_check.py` 在途有他人改动），仅报告根因：删图 + `frozen.json` 把删图登记为预期 ⇒ 门变绿，但运行时后果仍在。

### E5 工具坑 3：`plan_route --emit tokens` 输出不能直接喂 `sim_cli --route`

路线末 token 带终点朝向（`N5,x`），而 `route/parser.py` 的 `_BARE_GAP_RE = [,\s;]+`（`parser.py:77`）按逗号切分 ⇒ 被切成 `N5` + `x`，报 `无法解析的 token: x`。适配方式：喂仿真前剥掉**末 token 的 `,x|,y` 朝向声明**（只影响终点朝向声明，不改折线几何），六例各剥 1 个。这属工具间契约缺口，是否开卡由 Lead 定（不在本卡写域）。

### E6 闭环仿真矩阵（`sim_cli`，dt=0.02 / 延迟 0.06 s / 无噪声 / 吸附开）

| 路线 | rc | 跑完全程 | 总时长 | 末点误差 | 横向 RMS / MAX | 顶点超调 | 越界拍 | 路径比 | `diverged` |
|---|---|---|---|---|---|---|---|---|---|
| carloop legacy | 0 | **是** | 112.50 s | 0.0652 m | 0.1331 / 0.7789 | 0.1609 | 0 | 0.9383 | false |
| carloop tasks | 0 | **否** | 81.60 s | **5.4635 m** | 0.1459 / 0.9003 | **1.5020** | **60** | 0.6688 | **true**（第 4080 拍） |
| sixth tasks（禁火） | 0 | 是 | 74.26 s | 0.0299 m | 0.1379 / 0.7298 | 0.2641 | 0 | 0.9119 | false |
| field legacy | 0 | 是 | 104.64 s | 0.0640 m | 0.1243 / 0.3712 | 0.2038 | 0 | 0.9302 | false |
| field tasks | 0 | **否** | 49.20 s | **5.2717 m** | 0.1927 / 0.9056 | **2.3746** | 0 | 0.5255 | **true**（第 2460 拍） |
| 第六图单站（隔离） | 0 | **是** | 108.24 s | **0.0285 m** | 0.1304 / 0.6818 | 0.1672 | 74 | 0.9477 | false |

读法（**重要，别过度解读**）：`diverged=true` 的判据是「投影弧长连续多拍不前进 **且** 距路线 > 0.90 m」（`sim.py:1416-1427`）。两条 `tasks` 路线都停在**180° 折返顶点附近**（carloop 停在 `A3:PIVOT` 之后的 `A3→A2` 段，顶点超调 1.50 m；field 停在折返段）。
**但仿真不建模 PIVOT**（`sim.py:107-112`）⇒ 这些顶点在真车上应由「停-转」兜住。所以本表能证明的是「**若 PIVOT 没兑现，`tasks` 路线会跑飞；`legacy` 路线不会**」，**不能**证明「真车跑 `tasks` 会跑飞」。由此给结论：**在拿到 PIVOT 实车证据之前，不要把默认口径从 legacy 切到 tasks**（直接服务「完赛优先」）。

### E7 隔离候选补丁（32 行，未进生产）

- 内容：`planner.py:3305-3307` 的 `check_shoot_stops(..., require_pair=True)` ⇒ 先枚举全池，**只有在「不存在对侧且覆盖全靶的合法点」时**才 `require_pair=False` 并追加 warning（写明池大小与被选站）。
- 结果见 Q2 表：第六图 **rc=0 / 68 token / `B4*` / 24.85 m / route_check rc=0（雷 8/8、靶 2/2）/ sim 跑完全程 108.24 s、末点 0.0285 m**；carloop 双站产物**逐 token 相同**。
- 位置：`plan/isolated/`（`candidate_patch.diff`、前后 sha256、`src/dzbrain` 副本 4.9 MB）。生产 `planner.py` 哈希未变。

### E8 压雷量化（`mine_press_quant.py`）

见 Q3 表。补充：`carloop tasks` 的 `B1` 仿真离垫片 0.2907 m、但折线判据 0.0000 m —— 「折线绿」不代表「跑起来压到」，两者必须分开报（`plan/analysis/mine_press_quant.json`）。第六图 8 个雷在折线与仿真下都贴住（0.0000 m）。

### E9 门禁现状（基线已变，如实记）

`cd dzrobot_v2 && python -B -X utf8 tools/wf/check.py` → **rc=0 / 结果 GREEN**：冻结区脏文件 16 项全部 `[OK]`（含两张删图、`localization.py`、两个新增 `pursuit/*.py`）；登记来源是**车仓提交 `caaee3b1`**（`wf(frozen): 补登 3 条既有在途改动（map.yaml / map_v3.yaml / localization.py）`），不是本卡改的；另有 5 条「登记已失效（文件已不脏，可移除）」WARN。
⇒ Lead 交办时说的「wf 基线 RED3（两张删图 + localization 未登记）」**在本轮实测中已不成立**（被登记掉了）。这不是我把红跑绿 —— 本人未改 `frozen.json`（`git status` 可证）。同时见 E4：登记让门变绿，但 `route_check` 的地图仍缺失，**绿门不等于能发车**。

## 未验证（PASS / FAIL / UNKNOWN 分开）

| 项 | 判定 | 依据 / 边界 |
|---|---|---|
| 第六图「对侧无合法射击点」 | **PASS（已证）** | 池穷举 7 个 + 全图暴力复核 + `F7` 视线双挡 + `F6` 超近窗 |
| 第六图现状「有路线可出」 | **FAIL（已证）** | legacy rc=1 无路线；tasks rc=0 但靶覆盖 0/2 |
| 放宽后第六图「能跑完 + 压满雷 + 打到两靶」 | **PASS（仅离线）** | route_check rc=0 + sim 跑完全程 0.0285 m；**不等于实车** |
| 放宽对现役图无副作用 | **PASS（限定）** | 仅证 `carloop tasks` 63 token 逐 token 相同；未跑全量回归 37 套 |
| 「tasks 路线真车会跑飞」 | **UNKNOWN** | 仿真不建模 PIVOT（`sim.py:107-112`），只能实车/bench 证 |
| 5 个风险雷真车漏压 | **UNKNOWN** | 折线/仿真都不是实车；需下一趟逐雷车心—垫片测量 |
| 「3~8 cm」是否成立 | **FAIL（已证偏差，方向说反了对象）** | 48 处里 5 处告警是启发式代理；连续轨迹审计实际偏离的是 `C3` 7.50 cm / `E3` 6.90 cm（车心→雷心），其余 6 个雷 ≤0.34 cm。180° 折返旁实测口径 12.7 cm（`planner.py:3560`）仍属未验的实车值 |
| C11 观测 scale 必须接修正 | **UNKNOWN（不接）** | `robot.yaml` 有 `scale_est_enabled` / `anchor_scale_max`，但「观测到尺度误差」≠「必须把观测灌进定位」；本轮**未接、未改、未证其必要** |
| C12 `current_mode_=0` + `modeTimer` 双活跃 y<0.5 是「车不动」原因 | **UNKNOWN** | 代码事实：`dzactuator.h:308` `current_mode_ = 0`（0=未定）；`dzactuator.cpp:1422-1444` 按 `pursuit_y >= 0.5` 与 3 s 窗口仲裁。要定因果必须用**构造器 + 事件先后**复现（谁先发、y 何时跨 0.5、`last_pursuit_msg_` 是否为空）；本轮无该复现，**不称其为所有车不动的原因**（`cam_viewer` 20 Hz 全零覆盖是已证的另一条，见 [运维与排障/01 症状索引](../../../20_资料索引库/运维与排障/01_症状索引_52条.md) 症-01） |
| yaw 异常 = 假毛刺 | **UNKNOWN（不认定）** | `tasks/localization.py:677-696` 明确「持续 N 帧才判故障，单帧毛刺不刹」；缺同步原始信号时**不放宽阈值、不取消闩锁**（本轮未改任何阈值） |
| 全量回归 37 套 | **未跑** | 本卡只跑清单内专项；不宣称回归绿 |

## 文档变更

仅新增/更新本卡（`docs/30_任务卡库/在办/agc-1026/RECT-PLAN.md`）。树 `00_树.md`、全库索引、台账、`frozen.json` 由 Lead 单写，本卡未动。

## 经验总结

- **现象**：`plan_route` 失败信息只有一句「双射击停站应为 2 个，实际 1 个：B4」，读者无法判断是「这张图物理上没有第二个站」还是「搜索没找到」，容易被写成「双站物理不可能」这类过度断言。
  **处置**：把**候选池穷举**当成判据的一部分 —— 先列全池（逐靶距离/板面侧/视线），再对全图可通行格暴力复核；「池里没有」才算物理无解，且必须指出最近的那个候选**为什么不行**（`F7` 视线挡、`F6` 0.7215 m 超近窗）。可复跑：`plan/scripts/sixth_side_proof.py`、`f7_los_probe.py`。
- **现象**：删掉 `config/map.yaml` 后，`route_check` 不报错，而是「雷覆盖 未提供，跳过」，同时拿 `robot.yaml` 的历史障碍列表给出**假红**。
  **处置**：发车前一律 `ROBOT_MAP=<本局 map>` 显式指定；门禁 GREEN 不能替代这条运行时检查。建议把「`ROBOT_MAP` 缺省时拒绝运行」作为下一次工具整改候选（不在本卡写域）。
- **现象**：折线判据 8/8 与闭环判据 8/8 不是同一件事；`tasks` 路线折线全绿、仿真漏 `B1` 29 cm。
  **处置**：压雷结论必须写清是「折线口径」还是「闭环/实车口径」，禁止用折线绿代替压到。
- **现象**：同一条硬门在两套口径下后果差别巨大 —— legacy 直接不出路（0 分），tasks 出路线但禁火（靶 0/2）且不报错。
  **处置**：改进度门时，除了「过/不过」还要看**失败形态是否可跑**；「degraded 但能跑完」会让门禁看起来绿而实战禁火。
