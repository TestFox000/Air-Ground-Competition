# tools/planner —— 无人机侧路径规划器（**单源码同步产物**，禁止手改）

> 目标（t60）：把小车侧的路径规划器以"单源码同步"方式落到无人机侧，打通
> **感知地图 → 无人机端算路线 → `/nav/waypoint` 下发小车** 的契约。
> 契约文档：[`../../docs/空地路线下发契约_v2.md`](../../docs/空地路线下发契约_v2.md)

## 0. 铁律（先读这三条）

1. **真源永远是 `dzrobot_v2/`**（`src/dzbrain/{core,route}` + `tools/{plan_route,route_check,shoot_planner}.py` + `config/{map,robot}.yaml`）。
   本目录 `vendor/**` 是**生成物** —— 手工改它 = fork，一定会在 `--verify-only` 上爆。
2. `dzrobot_v2/**` 对本目录**只读**：同步脚本只读它、只往本目录写。
3. 跑的是**同一个入口**：`vendor/dzbrain_ws/tools/plan_route.py`（保持 `src/ + tools/ + config/` 结构），
   机上路径 `~/dzplanner/dzbrain_ws/`。

## 1. 目录

| 路径 | 说明 |
|---|---|
| `sync_planner_from_source.py` | **同步脚本**：逐字节复制真源 → `vendor/dzbrain_ws/`，写 `HASH_LEDGER.json`（文件哈希 **＋ 路线内容指纹**）+ `EVIDENCE/sync_log.txt`；`--verify-only` 检测文件/路线双层漂移（写 `EVIDENCE/verify_log.txt`，不覆盖同步日志）；`--push-drone [--run-drone]` 推机上并跑 |
| `run_plan_and_compare.py` | 双侧对拍：`vendor`(无人机侧) 与 `dzrobot_v2`(小车侧) 同一输入各跑 2 次 + `--check`；逐 token 比较 |
| `dryrun_full_route.py` | 整线下发**干跑**：planner → `dzlink_supervisor.py route --dry-run` / `dz_send_waypoint.py route --dry-run` |
| `dzlink_mission_ctl.py` | 编排：`plan → route_check 门禁 → 下发 /nav/waypoint`，带 15 s 计时 |
| `route_crop.py` | **L0 过渡线（t5e）**：按车 `car_state.block` 裁掉整线前缀 → 自检（不回头/不压已占格）→ 经 A 通道下发（见 §6） |
| `bench_route_crop.py` | L0 的 H1/H2 bench（离线近似模型）→ `EVIDENCE/route_crop_bench.txt` |
| `link_status.py` | 非侵入链路现状复验（ICMP / TCP:22 / 本网段 / 只读配置域号）→ `EVIDENCE/link_status.md` |
| `HASH_LEDGER.json` | **哈希账本**：每文件 sha256 + size + 源 mtime + 生成时间 + 闭包自检结论 ＋ **route 块**（`mission_blocks` 逐 token、count、`" ".join` sha256：车侧/产物各一份、`same`） |
| `vendor/dzbrain_ws/` | 与真源**逐字节一致**的树（`src/ tools/ config/` + `MANIFEST.txt`） |
| `EVIDENCE/` | 每次运行的证据（同步日志、对拍结果、route_check 全文、干跑输出、链路现状、L0 bench） |
| `tests/` | 单测：账本一致性 + vendor≠fork + 规划确定性与双侧一致 + route_check 0 error + **L0 裁剪** |

## 2. 一次性 & 日常命令

```powershell
cd D:\User\Desktop\比赛存档\v1

# ① 同步（生成 vendor + 账本 + 闭包自检）
python dzsmartscout\tools\planner\sync_planner_from_source.py

# ② 只要有真源改动，先验漂移（exit 1 = 真源变了/产物被手改）
python dzsmartscout\tools\planner\sync_planner_from_source.py --verify-only

# ③ 回归：双侧路线逐 token 对拍 + route_check 0 error
python dzsmartscout\tools\planner\run_plan_and_compare.py

# ④ 整线下发干跑（不发车、不连 ROS）
python dzsmartscout\tools\planner\dryrun_full_route.py

# ⑤ DLL 链路现状（非侵入）
python dzsmartscout\tools\planner\link_status.py

# ⑥ 推无人机并跑（需无人机在线，默认 duzhong@10.29.122.104:~/dzplanner）
python dzsmartscout\tools\planner\sync_planner_from_source.py --push-drone --run-drone

# ⑦ 单测
python -m unittest discover -s dzsmartscout/tools/planner/tests -p "test_*.py" -v

# ⑧ L0 裁剪：本地干跑（不连 ROS）→ 真发（A 通道 /nav/waypoint）
python dzsmartscout\tools\planner\route_crop.py crop --from-plan --car-block E10 --car-yaw 90
python dzsmartscout\tools\planner\route_crop.py crop --from-plan --car-state .\car_state.jsonl --publish

# ⑨ L0 裁剪的 H1/H2 bench（离线近似模型）
python dzsmartscout\tools\planner\bench_route_crop.py
```

## 3. 本轮（t60）实测结果

| 项 | 结果 |
|---|---|
| 同步 | **16 个文件 sha256 全部一致**，闭包自检通过（含 n1 常量收敛新增的 `tools/_field_const.py`；`EVIDENCE/sync_log.txt`、`HASH_LEDGER.json`） |
| ★ 哈希门禁实战 | 执行期间队友改了一次 `config/robot.yaml`（纯注释 5 行）⇒ 门禁**立刻报漂移**，按"存明细→重同步→重跑全部验证"处置，token 未变（[`EVIDENCE/drift_event.md`](EVIDENCE/drift_event.md)） |
| ★★ 引擎口径（收口轮） | **`plan_route.py --engine` 默认是 `tour`，实车走 `planner.engine`**（第五张图 = `tracker-safe`，`robot.yaml:586`）。本目录所有脚本改为**从 config 读引擎**（`planner_engine.py`，用车上同一份 `Config`）；不显式传就是"机算 40 / 车跑 44"的静默错位 |
| 规划（官方第五张图，`tracker-safe`） | `N10 M10 M9 L9 K9 J9 J10 I10 H10 G10 F10 E10 E9 D9 C9 B9 B8 B7 B6 A6 A5 A4 A3 A2 B2 B1 C1 D1 D2 D3* E3 F3 G3* G4 G5 G6 H6 I6 I5 J5 K5 L5 M5 N5,y`（**44 token**，2026-10-17 现役线） |
| 确定性 | 两侧各跑 2 次，token 串完全相同（每次都一样） |
| 双侧一致 | **无人机侧(产物树) == 小车侧(真源树)，逐 token 相同** ✅（`EVIDENCE/plan_compare.json`，`same_tokens=true`） |
| ★ 与板载回退路线一致 | 机算 44 token == `robot.yaml` 的 `mission_blocks` **逐 token 相同** ✅（`mission_blocks_same=true`；路线指纹 sha256 `df24e944ae4ccf8b…`） |
| 出车校验 | `route_check` **0 error / 1 warn**（warn = 急转弯提示级、`:PIVOT` 声明 0；提示级） |
| 整线干跑 | supervisor（`/nav/waypoint qos=reliable x3`）与发送器都输出**完整 44 token** |
| 单测 | `Ran 11 tests / OK`（账本一致性 + 防 fork + 确定性 + 双侧一致 + route_check 0 error + **引擎口径** + **mission_blocks 一致**） |
| 推机 | ❌ **未完成**：无人机 `10.29.122.104` 不可达、不在 `192.168.137.0/24`（见 §4） |

## 4. 链路现状（本轮）

* 无人机 `10.29.122.104`：**不在线**（ICMP 100% 丢包 / TCP:22 不通）⇒ **`--push-drone` 与端到端 DDS 可见性未验证**，不得当成已部署。
* 小车 `192.168.137.241`：在线（ICMP 8ms / TCP:22 开），与本机**同网段**；按 t60 边界**未登车、未起 ROS**。
* 详见 [`EVIDENCE/link_status.md`](EVIDENCE/link_status.md)（含域号真值取证：两端 **118**，非 0）。

## 5. 边界

* 本目录**不碰** `dzrobot_v2/**`（只读同步）；不碰车端栈（保持冷态）。
* 改 `dzsmartscout/tools/link/**` 只能**加新文件**，不改 `dzlink_supervisor.py` 现有行为。

## 6. L0 过渡线：`route_crop.py`（卡 t5e）

> 依据：决策层方案 §7.1（L0）+ R1 裁决「**L0 只作 bench 对照，L2 为主线**」。
> L0 拿不到「已压雷/已过区域」语义 ⇒ 它是**过渡**，不是终局。

链路（全部在机侧/无人机侧完成，**零车侧改动**）：

```
机侧整线（vendor planner，引擎从 config 读）
   → 读车回传 car_state{block,yaw_deg,phase}（/dzlink/car_to_drone，1~2 Hz）
   → 定位车当前格在整线里的下标 → 裁掉前缀（首点 = 车当前格）
   → 自检：不回头 / 不压已占格 / 交接航向 / phase 闸门
   → A 通道 /nav/waypoint（RELIABLE ×3，复用 dz_send_waypoint.py）
```

### 6.1 三条铁律（改它之前先读）

1. **只裁前缀，绝不重排、绝不改 token 原文**：输出恒为 `tokens[anchor:]`，
   `*`（停站打靶）、`,y`（朝向）、`:SCAN_STOP`、`R:` 全部原样保留。
2. **拒绝优先于猜测**：锚点定不下来（车格不在线里 / 同格多次出现且航向也分不出）、
   `phase` 不是 `DRIVE`/`APPROACH`、后缀里出现已压/已占格、交接夹角 > 135° —— 一律
   `ok=False` + 原因，**不发**。一条错线的表现是"车收到但行为不明"，比不发更危险。
3. **UNKNOWN ≠ ABSENT**：`coverage.complete=false` 或旧报文无 `coverage` ⇒ 一律当没扫过，
   不减少未知格计数。语义不在这里重实现，直接消费 `perception.protocol.field_verify_absent`（t10）。

### 6.2 为什么"首点 = 车当前格"就不会掉头（源码依据）

车侧 `_install_route` 每次装线都置 `_next_wp = 1`（`mission.py:2299`）、
`P_ROUTE_INDEX = 0 / P_ROUTE_S = 0`（`:2325`）⇒ 纯追踪从**第 2 个 token** 开始追。
所以裁剪后第一个真正被追的目标就是「车当前格的下一格」，不存在"倒车回首点"。

### 6.3 bench（`bench_route_crop.py`）测什么

| 用例 | 断言 | 为什么重要 |
|---|---|---|
| H1a | 直段交接（G10）：首拍航向误差 ≤ 10°、0 拍需倒车、20 拍偏离路线 ≤ 0.10 m、实际前进 ≥ 0.60 m | 对应方案 §7.1 H1「首点=当前格不倒车」 |
| H1c/H1d | 车心偏 0.10 m（横向 / 纵向）：首拍 ≤ 30°、首个目标 ≤ 2 格 | belief 不在格心时也不倒车 |
| H1b | 90° 拐点交接：交接瞬间 ≤ 60°、verdict ≠ reversal | 拐点是最不利的交接位置 |
| **H1e（对照）** | **不裁剪**（整线原样）⇒ 首个目标 2.8 m、首拍 180°、20/20 拍需倒车 | 反证「裁剪不是可选项」 |
| H2a | 已压雷落在**被裁掉的前缀**里 ⇒ 0 回头、0 重压、后缀不含前缀格 | 对应 §7.1 H2 |
| H2b（对照） | 锚点=起点 ⇒ 一个 token 都没裁（危险态） | 锚点错 = 整段重走 |
| H2c | 已压格落在**后缀**里 ⇒ 报 `repress` 并拒绝 | L0 修不了的情形，必须拒发（交 L2） |
| H2d | `STOP/FINISH/SAFE/IDLE` 拒绝改线，`DRIVE/APPROACH` 放行 | 打靶中重装路线 = 进度归零 |
| H2e | `complete=false` / 旧报文 ⇒ 不产生 ABSENT；`complete=true` 全场 ⇒ 未知格 = 0 | t10 负证据语义被正确继承 |
| H2f/H2g/H2h | 重复格消歧、车格不在线里（默认拒/显式就近）、航向与锚点矛盾 | 三类"车状态不可信"的兜底 |

证据：[`EVIDENCE/route_crop_bench.txt`](EVIDENCE/route_crop_bench.txt)（**同时跑 vendor 与真源两条整线**，各 13 用例，全过）。

### 6.4 如实交代（未做/未验）

* bench 是**近似模型**（转向律按 `config.py:72` 注释的教科书式 `δ=atan(2L·sinα/ld)`，
  `path_executor` 本体不在 vendor 子集内）⇒ **不是**逐行复现；真车那 20 拍 `pursuitAngle` 仍需现场录。
* 拐点交接会出现 `|α|>90` 的拍（切角 + 0.12 m 到位判定），**每个拐点都这样**、属路线固有；
  H1 的结论只针对**交接瞬间**。
* `--publish` 只在 Windows 本机验到"形状契约通过 + 干跑报文"，**未在车上真发**（本卡无车）。
* 锚点带 `*` 时车侧停站行为未验证（`_next_wp=1` 与"首点停站"的交互），代码里已打告警。

### 6.5 vendor 漂移 → 已重同步（2026-10-17 n4 收口）

> **历史（t5e 当时）**：`--verify-only` 曾报 5 条漂移（`core/config.py`、`route/physics.py`、
> `route/planner.py`、`tools/route_check.py`、`config/robot.yaml`，账本停在 2026-10-16 19:28），
> 且车侧路线在未提交状态下换线（`L10→M9`、`J6→I5`、双停站 `D3*`+`G3*`）。
> 证据存档：[`EVIDENCE/drift_check_2026-10-17.txt`](EVIDENCE/drift_check_2026-10-17.txt)、
> [`EVIDENCE/drift_event.md`](EVIDENCE/drift_event.md)（历史，不再代表现状）。

**现状（n4 重同步后，2026-10-17 16:46）**：

* 16 个文件逐字节一致、闭包自检通过；`--verify-only` **漂移 0 条 / 缺失 0 条**（`EVIDENCE/verify_log.txt`）。
  （n1 卡把 `tools/{route_check,shoot_planner,global_planner}.py` 的场地常量收敛到
  `tools/_field_const.py` —— 它是 script 侧唯一真源入口，已加入 `FILE_SET` 一并 vendor；
  漏了它产物树跑 `route_check` 会 `ModuleNotFoundError`。）
* 路线内容级对账：车侧现役 `mission_blocks` 44 token sha256 `df24e944ae4ccf8b…` == 产物侧，**逐 token 一致**；
  与规划器产物也一致（`EVIDENCE/plan_tokens.txt`、`plan_compare.json`、`route_check_{vendor,source}.txt`、
  `dryrun_full_route.txt` 全部按现役线重生成）。
* 账本新增 `route` 块后，**"vendor 自洽但已与车侧分叉"这种假绿会在 `--verify-only` 直接报出**（v4 审计 F-17
  / 结构性洞①的落地）。
* 另修：`--verify-only` 不再覆盖 `EVIDENCE/sync_log.txt`（改写 `verify_log.txt`）；Windows GBK 控制台
  打印 ✅/❌ 不再 `UnicodeEncodeError`（同步成功被 print 崩成 exit 1 的坑）。
* ⚠️ 真源仍可能被并发卡改写：本目录的纪律不变 —— **每次用之前先 `--verify-only`**；若有漂移，重跑同步即可。
