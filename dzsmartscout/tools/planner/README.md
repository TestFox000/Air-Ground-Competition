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
| `sync_planner_from_source.py` | **同步脚本**：逐字节复制真源 → `vendor/dzbrain_ws/`，写 `HASH_LEDGER.json` + `EVIDENCE/sync_log.txt`；`--verify-only` 检测漂移；`--push-drone [--run-drone]` 推机上并跑 |
| `run_plan_and_compare.py` | 双侧对拍：`vendor`(无人机侧) 与 `dzrobot_v2`(小车侧) 同一输入各跑 2 次 + `--check`；逐 token 比较 |
| `dryrun_full_route.py` | 整线下发**干跑**：planner → `dzlink_supervisor.py route --dry-run` / `dz_send_waypoint.py route --dry-run` |
| `link_status.py` | 非侵入链路现状复验（ICMP / TCP:22 / 本网段 / 只读配置域号）→ `EVIDENCE/link_status.md` |
| `HASH_LEDGER.json` | **哈希账本**：每文件 sha256 + size + 源 mtime + 生成时间 + 闭包自检结论 |
| `vendor/dzbrain_ws/` | 与真源**逐字节一致**的树（`src/ tools/ config/` + `MANIFEST.txt`） |
| `EVIDENCE/` | 每次运行的证据（同步日志、对拍结果、route_check 全文、干跑输出、链路现状） |
| `tests/` | 单测：账本一致性 + vendor≠fork + 规划确定性与双侧一致 + route_check 0 error |

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
```

## 3. 本轮（t60）实测结果

| 项 | 结果 |
|---|---|
| 同步 | **15 个文件 sha256 全部一致**，闭包自检通过（`EVIDENCE/sync_log.txt`、`HASH_LEDGER.json`） |
| ★ 哈希门禁实战 | 执行期间队友改了一次 `config/robot.yaml`（纯注释 5 行）⇒ 门禁**立刻报漂移**，按"存明细→重同步→重跑全部验证"处置，token 未变（[`EVIDENCE/drift_event.md`](EVIDENCE/drift_event.md)） |
| ★★ 引擎口径（收口轮） | **`plan_route.py --engine` 默认是 `tour`，实车走 `planner.engine`**（第五张图 = `tracker-safe`，`robot.yaml:586`）。本目录所有脚本改为**从 config 读引擎**（`planner_engine.py`，用车上同一份 `Config`）；不显式传就是"机算 40 / 车跑 44"的静默错位 |
| 规划（官方第五张图，`tracker-safe`） | `N10 M10 L10 L9 K9 J9 J10 I10 H10 G10 F10 E10 E9 D9 C9 B9 B8 B7 B6 A6 A5 A4 A3 A2 B2 B1 C1 D1 D2 D3 E3 F3* G3 G4 G5 G6 H6 I6 J6 J5 K5 L5 M5 N5,y`（**44 token**） |
| 确定性 | 两侧各跑 2 次，token 串完全相同（每次都一样） |
| 双侧一致 | **无人机侧(产物树) == 小车侧(真源树)，逐 token 相同** ✅（`EVIDENCE/plan_compare.json`） |
| ★ 与板载回退路线一致 | 机算 44 token == `robot.yaml` 的 `mission_blocks` **逐 token 相同** ✅ |
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
