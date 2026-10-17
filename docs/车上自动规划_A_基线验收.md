# 车上自动规划 · 任务卡 A 基线验收报告

> ⚠️ **历史报告（t7 复审标注 2026-10-17）**：本文是 2026-10-10 的验收快照（当时路线 43 token / 17.15 m），
> 正文数字**不代表现役**。现役 = **44 token / 15.05 m / 双停站 `D3*`+`G3*`**（2026-10-17 t13；板心均 0.875 m），
> 真源见 `docs/发车门禁口径.md` §6。保留原文为存档验收过程，**排障勿按本文数字**。

- 验证人：`verifier`（独立验证者，只读；不改任何源码）
- 任务：`t1` ／ attempt_id `17839b80-3cb8-40d4-ab40-49329b8ed57f`
- 工作区：`D:\User\Desktop\dzrobot_v2`
- 报告时间：2026-10-10（数据采集 19:22–19:35）
- 允许写入的路径：`backup/cardA-baseline/`、`docs/车上自动规划_A_基线验收.md`（本报告）
- 统一环境变量（下称 `$ENV`）：

```powershell
$env:PYTHONPATH='D:\User\Desktop\dzrobot_v2\src\dzbrain'
$env:PYTHONIOENCODING='utf-8'
$env:PYTHONUTF8='1'
```

> ⚠️ 本机 `pwsh` 实测为 **Windows PowerShell 5.1.26100.9444**。PS 5.1 用 ANSI(CP936) 解码 UTF-8 中文文件会**吞掉部分换行**，导致 `Get-Content`/`Measure-Object -Line` 的行数与真实值不符（见 §6 与 finding A5）。本报告所有行号来自 `read`/`grep`（ripgrep，UTF-8 正确），行数来自 Python。

---

## 0. 结论速览

| 验收项 | 结论 | 证据位置 |
|---|---|---|
| 从 `config/map.yaml` 生成合法路线 | ✅ 通过（43 token，`route_check` **0 error**） | §2、§3 |
| CLI 默认路线的 `R:` 段数 | ⚠️ **6 段**（不是 0） | §3 |
| `:PIVOT` 声明 token | ❌ **不存在**（实现/语法/输出三处都没有） | §4、finding A2 |
| `plan_route` 支持 `allow_reverse=False` | ✅ 支持（Python API 实测可出 **0 段 `R:`** 路线） | §4、§5 |
| 该能力从 CLI / 车上可达 | ❌ **不可达**（CLI 无 `--no-reverse`，无调用点传 False） | §4、§5、finding A1 |
| `assert_legal_no_reverse` 是否被调用 | ✅ 已调用，但只在 `allow_reverse=False` 分支 | §4 |
| 基线备份冻结 | ✅ 6 文件 + `git-state.txt` + sha256 清单 | §6 |
| 20 套件回归 | ✅ **20/20 exit 0，1601 断言**（独立实跑） | §7 |

---

## 1. 基线与 git 状态（原始输出）

命令：

```powershell
cd D:\User\Desktop\dzrobot_v2
git status --porcelain
git log --oneline -3
```

原始输出：

```
 M src/dzbrain/dzbrain/route/planner.py
?? .agent-teams/

fab01b7 docs: 上下文快照（防失忆；flash 子智能体连挂 3 次后由主智能体手写）
fcf09b9 feat(tracker/mission): PIVOT 完成通知直接判到达 —— 修掉真车实测的 5 点被兜底跳过
0c93a63 fix(test): test_path_overlap 的 PIVOT 点期望修正 off-by-one
```

命令与输出（未提交改动规模）：

```
> git diff --numstat -- src/dzbrain/dzbrain/route/planner.py
57      5       src/dzbrain/dzbrain/route/planner.py
> git diff --stat
 src/dzbrain/dzbrain/route/planner.py | 62 +++++++++++++++++++++++++++++++++---
 1 file changed, 57 insertions(+), 5 deletions(-)
```

结论：全仓**只有 `planner.py` 一个文件有未提交改动**（+57/−5），`tools/plan_route.py`、`config/robot.yaml`、`parser.py`、`mission.py`、`main.py` 均与 HEAD 一致。因此本次冻结的备份就是干净的「卡 B / 卡 C 改动前」基线。

---

## 2. 验收项 1：`--emit tokens --quiet`

命令（`$ENV` 之后）：

```powershell
python tools\plan_route.py --emit tokens --quiet
```

原始 stdout（**逐字节，单行 + 结尾 LF；UTF-8 共 152 字节**）：

```
N10 N9 M9 L9 K9 I9 I10 H10* R:H11 G11 R:E11 R:F11 F10 F9 F8 E8 D8 C8 C6 R:D6 R:C6 B6 A6 A5 C5 C4 D4 E4 F4 G4 R:G3 G2 G1 H1 J1 J2 J3 J4 K4 L4 M4 N4 N5,x
```

- 退出码：**0**
- 实测统计（`python -c` 解析同一 stdout）：`token_count=43`、`R_count=6`、`R_segments=['R:H11','R:E11','R:F11','R:D6','R:C6','R:G3']`、`star=['H10*']`、`comma=['N5,x']`

---

## 3. 验收项 2：`--check`

命令：

```powershell
python tools\plan_route.py --check
```

原始输出（逐字，含 ANSI 色码位置）：

```
==============================================================================
  路径规划器 · 换地图即出路线
==============================================================================
  配置中心 : D:\User\Desktop\dzrobot_v2\config\robot.yaml
  地图文件 : D:\User\Desktop\dzrobot_v2\config\map.yaml
  地图名称 : 现场布置图 2026-10-07 (识图标定)

路线[K4] 43 token / 16.45 m | 穿洞 下=K9 上=K4 | 射击停站 H10 到靶 0.350 m（H11）
  A* 代价 108.70 / 展开 7672 次 / 耗时 17.1 ms | 倒车段 6（折返改写 2）
  N10 N9 M9 L9 K9 I9 I10 H10* R:H11 G11 R:E11 R:F11 F10 F9 F8 E8 D8 C8 C6 R:D6 R:C6 B6 A6 A5 C5 C4 D4 E4 F4 G4 R:G3 G2 G1 H1 J1 J2 J3 J4 K4 L4 M4 N4 N5,x
  route_check: ✅ 0 error
    ⚠️ 无法导入 path_executor（ModuleNotFoundError），改用内置副本校验
    ✅ 全部 token 合法                    普通 43 | 倒车 6 | 射击 1
    ✅ 解析出 43 个航点
    ✅ 地图边界                           全部 43 点在 [0.17,3.67]x[0.17,4.72] 内
    ✅ 挡板穿越 1 次                       均在 C 列，最小余量 0.350 m
    ✅ 障碍最近距离                         0.350 m（航段#12 距 障G9）> 0.30 m
    ⚠️ 急转弯 13 处                       最大 90° @航点#1（将触发 PIVOT 原地转向）
    ✅ 桥洞水平直穿 4 次                     K 列  段#3(行9) 段#4(行9) 段#37(行4) 段#38(行4)
    ✅ 雷覆盖 8/8
    ✅ 靶覆盖 2/2（射击点可达）                 G11<-射击点#7(0.49m) H11<-射击点#7(0.35m)
    ✅ 射击点距离                          1 个射击点都能服务于某个靶（0.20~0.45 m 内）
    ⚠️ 倒车+转向 2 组                      最长 1.05 m / 2 段 —— 盲区里打舵风险高，建议缩短或拆开
    ✅ 桥洞保护点覆盖                        K4 K9
    ⚠️  全部通过，但有 2 项提醒

✅ 规划 + 出车前校验全部通过（route_check 0 error）
```

**逐项计数（原样报出）：**

| 项 | 数 | 出处（上述原始输出） |
|---|---|---|
| `route_check` **error** | **0** | `route_check: ✅ 0 error`、末行 `route_check 0 error` |
| `route_check` **warn** | **2** | `⚠️ 全部通过，但有 2 项提醒` → ① `急转弯 13 处` ② `倒车+转向 2 组` |
| 该路线 `R:` **段数** | **6** | `倒车段 6（折返改写 2）`；token 级实测同样 `R_count=6` |
| 额外告警（非 route_check warn） | 1 | `⚠️ 无法导入 path_executor（ModuleNotFoundError），改用内置副本校验` —— 见 finding A4 |
| 退出码 | **0** | 命令实测 |

---

## 4. 验收项 3：`allow_reverse` 三问（行号证据）

`grep allow_reverse` 全仓库（`*.py`）共 10 处命中，**全部在 `planner.py` 内部**：

| 问 | 答 | 行号证据 |
|---|---|---|
| ① 是否已支持 `allow_reverse=False`？ | **支持**。签名有该关键字参数；`Grid.__init__` 亦有；三条语义分支都已接上 | `planner.py:1206` `allow_reverse: bool = True,`（`plan_route` 签名，函数定义在 `:1203`）；`planner.py:252` `*, allow_reverse: bool = True) -> None:`（`Grid.__init__`）；`:257` `self.allow_reverse = bool(allow_reverse)`；`:347` `if not grid.allow_reverse:`（`_moves` 去掉倒车动作）；`:1249` `allow_reverse=allow_reverse`（传给 Grid）；`:1394` `uturn_fixes = _fix_uturn_reverse(stops) if allow_reverse else 0`（跳过折返改写）；`:1409-1412` 三分支尾部断言 |
| ② 默认值是什么？ | **`True`**（= 旧行为，允许倒车段） | `planner.py:1206`；`Grid` 侧同为 `True`：`planner.py:252`；文档字符串 `:1220` 明写「True = 旧行为……默认不变」 |
| ③ `assert_legal_no_reverse` 是否已被调用？ | **已被调用，但只在 `allow_reverse=False` 分支** | 定义 `planner.py:881`；导出 `:99`（`__all__`）；调用点 `planner.py:1409-1412`：`if allow_reverse:` → `assert_legal(route)` `else:` → `assert_legal_no_reverse(route)` |

**结论补充（关键）**：能力存在，但**在 CLI 与车上路径上不可达**——

- `tools/plan_route.py:151-154` 调用 `PL.plan_route(...)` 时**只传了** `cross_lower / cross_upper / name / source / max_seconds`，**没有传 `allow_reverse`** → 落到默认 `True`。
- `tools/plan_route.py` 的 CLI 参数（`:171-187`）里**没有** `--no-reverse`。
- 全仓库 `plan_route(` 调用点共 8 处：定义 1（`planner.py:1203`）、测试 6（`test_planner.py:266/329/361/381/427/438`）、CLI 1（`tools/plan_route.py:151`）—— **无一处传 `allow_reverse`**。

---

## 5. 那 57 行未提交改动：实现了什么 / 没实现什么

### 5.1 diff 事实（`git diff -U0` 摘录，全文另存 `backup/cardA-baseline/planner-57lines.diff`）

```
@@ -98,0 +99 @@ __all__ = [
+    "assert_legal_no_reverse",
@@ -250 +251,2 @@ class Grid:
+                 *, allow_reverse: bool = True) -> None:
@@ -251,0 +254,4 @@ class Grid:
+        self.allow_reverse = bool(allow_reverse)
@@ -331 +337,6 @@ def _moves(grid: Grid, row: int, col: int, head: int):
+    `grid.allow_reverse=False`（无倒车模式）时**不产出倒车动作**：真车底盘不支持倒车
@@ -335,0 +347,4 @@ def _moves(grid: Grid, row: int, col: int, head: int):
+    if not grid.allow_reverse:
@@ -865,0 +881,24 @@ def _angle_of(a: Waypoint, b: Waypoint, c: Waypoint) -> float:
+def assert_legal_no_reverse(route: Route) -> None:
@@ -1166,0 +1206 @@ def plan_route(map_cfg: Any, cfg: Config | None = None, *,
+               allow_reverse: bool = True,
@@ -1202 +1248,2 @@ def plan_route(...):
+                allow_reverse=allow_reverse)
@@ -1347 +1394 @@ def plan_route(...):
+    uturn_fixes = _fix_uturn_reverse(stops) if allow_reverse else 0
@@ -1360 +1409,4 @@ def plan_route(...):
+    if allow_reverse:
+        assert_legal_no_reverse(route)
```

### 5.2 逐条判定（对照队长判断）

| 队长判断 | 实测判定 | 依据 |
|---|---|---|
| 已有 `allow_reverse` 雏形 | ✅ **对** | `planner.py:252/257/1206/1249/1394/1409` |
| 已有 `assert_legal_no_reverse` 雏形 | ✅ **对**（不只雏形：完整断言 + 已接入分支 + 已导出） | `planner.py:99/881/1412` |
| 没有 `:PIVOT` 输出 | ✅ **对**，且比"没输出"更彻底：**全仓没有 `:PIVOT` 这个语法** | 见 §4 finding A2 |
| `tools/plan_route.py` 没有 `--no-reverse` | ✅ **对** | `tools/plan_route.py:171-187` 无该参数；`:151-154` 也未传 `allow_reverse` |

### 5.3 额外实测（只读探针，队长未要求，但决定卡 B 的可行性）

探针脚本：`backup/cardA-baseline/probe_no_reverse.py`（**只调用规划器 API，不改任何源码**）。
命令：`python backup\cardA-baseline\probe_no_reverse.py` → 退出码 0，原始输出：

```
allow_reverse=True: OK tokens=43 R_seg=6 PIVOT_token=False elapsed=17.4ms waypoints=43
   tokens: N10 N9 M9 L9 K9 I9 I10 H10* R:H11 G11 R:E11 R:F11 F10 F9 F8 E8 D8 C8 C6 R:D6 R:C6 B6 A6 A5 C5 C4 D4 E4 F4 G4 R:G3 G2 G1 H1 J1 J2 J3 J4 K4 L4 M4 N4 N5,x
allow_reverse=False: OK tokens=43 R_seg=0 PIVOT_token=False elapsed=12.4ms waypoints=43
   tokens: N10 N9 M9 L9 K9 I9 I10 H10* F10 F8 F9 F10 F11 E11 E10 C10 C9 C8 C7 C6 D6 C6 B6 A6 A5 C5 C4 C2 D2 E2 F2 G2 G1 H1 J1 J2 J3 J4 K4 L4 M4 N4 N5,x
```

**无倒车模式在当前地图上真的能解出来**（0 段 `R:`，12.4 ms，结构自检 + `assert_legal_no_reverse` 全过）。
再把这条无倒车路线喂给 `tools/route_check.py`（子进程调用原文件，未复制规则）：

```powershell
python tools\route_check.py --route "N10 N9 M9 L9 K9 I9 I10 H10* F10 F8 F9 F10 F11 E11 E10 C10 C9 C8 C7 C6 D6 C6 B6 A6 A5 C5 C4 C2 D2 E2 F2 G2 G1 H1 J1 J2 J3 J4 K4 L4 M4 N4 N5,x" --config config\robot.yaml
```

原始输出要点（全文另存 `backup/cardA-baseline/logs/route_check_no_reverse.txt`）：

```
✅ 全部 token 合法                    普通 43 | 倒车 0 | 射击 1
✅ 解析出 43 个航点
✅ 地图边界 / ✅ 挡板穿越 1 次 / ✅ 障碍最近距离 0.350 m
⚠️ 急转弯 20 处                       最大 180° @航点#9（将触发 PIVOT 原地转向）
✅ 桥洞水平直穿 4 次 / ✅ 雷覆盖 8/8 / ✅ 靶覆盖 2/2 / ✅ 射击点距离
     总里程                            17.15 m（42 航段）
✅ 桥洞保护点覆盖                        K4 K9
⚠️  全部通过，但有 1 项提醒  · 急转弯 20 处
ROUTE_CHECK_EXIT=0
```

⇒ **无倒车路线同样 0 error**，符合「0 段 `R:`＋折返由 TrackerTask 自动 PIVOT」的目标语义。缺的只是**把开关接到 CLI / 车上**（卡 B）。

---

## 6. 验收项 4：基线备份冻结

位置：`backup/cardA-baseline/`（保留相对路径）。清单同时写入 `backup/cardA-baseline/sha256.txt`。

| 文件 | 字节数 | sha256（小写） | 与源文件逐字节一致 |
|---|---|---|---|
| `src/dzbrain/dzbrain/route/planner.py` | 73254 | `1b7d38071fde255b4e0dfdbe616aee9f1142ccfbbf9d21c58e0c8429bf132f64` | ✅ SRC==BAK |
| `src/dzbrain/dzbrain/route/parser.py` | 19759 | `ecc95bbc609a6aa509de7c993713feafbbe0b76c9bfcf71f895efaf5e8266e80` | ✅ SRC==BAK |
| `src/dzbrain/dzbrain/tasks/mission.py` | 56126 | `519394c8d48fbd1148bc7d2aa85ef81995f9dcd507d3b1f5f1046b96eb8257c5` | ✅ SRC==BAK |
| `src/dzbrain/dzbrain/main.py` | 17879 | `26bfceda24c1be4addebf6d4dd0c7825dd52a19dc1f1f7cf98d9ba56f00b2486` | ✅ SRC==BAK |
| `config/robot.yaml` | 33357 | `952584b55737037451d23725cb26b69b4e91a45cbf92e6afb9f547bb7c39036e` | ✅ SRC==BAK |
| `tools/plan_route.py` | 14508 | `b6c36c6a217ec83e23aad1cf17327c3ac00b14d40507d3b4860e25e977e4cf78` | ✅ SRC==BAK |
| `git-state.txt`（附加） | 528 | `d1a3c2a12cadc62fc9a36b7767e92a4e023c63f89d34d10bfaa8b405c428d27b` | — |

命令：`Get-FileHash <file> -Algorithm SHA256`（源/备份两侧各算一次，取 `match=True` 才算过）。

附加取证文件（同一目录，便于卡 B/C 直接复现）：

| 附加文件 | 字节 | sha256 |
|---|---|---|
| `planner-57lines.diff`（那 57 行的完整 diff） | 8019 | `b075057b3632277bc2d6ade6274d196386c9945c71fe42a52ad9e7b4f9b4c7aa` |
| `logs/`（20 套件 + 探针 + route_check 的完整原始输出） | 20 个 `.txt` + `summary.json` | 见各文件 |
| `probe_no_reverse.py`（只读探针脚本） | — | §5.3 |

### ★ 回滚语义必须说清（给卡 B/C）

- 本备份冻结的是**当前工作树**，即**含任务卡 A 那 57 行未提交改动**的 `planner.py`（73254 B）。
- `HEAD` 版本的 `planner.py` 是 **69975 B / 1424 行**（`git show HEAD:src/dzbrain/dzbrain/route/planner.py`），**不含**那 57 行。
- ⇒ 「回滚到本备份」= 回到「A 卡改动已在、B/C 改动未做」的状态；若要回到 `HEAD`，请显式用 `git checkout -- src/dzbrain/dzbrain/route/planner.py`（**会丢掉 57 行**）。

---

## 7. 验收项 5：20 套件独立实跑（本次由我实跑，不引用队长数字）

命令（逐个套件直接跑，不做 pytest）：

```powershell
$env:PYTHONPATH='D:\User\Desktop\dzrobot_v2\src\dzbrain'; $env:PYTHONIOENCODING='utf-8'; $env:PYTHONUTF8='1'
Get-ChildItem src\dzbrain\tests\test_*.py | Sort-Object Name | ForEach-Object { python $_.FullName; ... }
```

| # | 套件 | exit code | 结果行（原始） | 断言数 |
|---|---|---|---|---|
| 1 | `test_aim.py` | 0 | `结果：122/122 全部通过 ✅` | 122 |
| 2 | `test_contract.py` | 0 | `结果：56/56 全部通过 ✅` | 56 |
| 3 | `test_core.py` | 0 | `结果：50/50 全部通过 ✅` | 50 |
| 4 | `test_deploy.py` | 0 | `结果：76/76 全部通过 ✅` | 76 |
| 5 | `test_drone.py` | 0 | `结果：66/66 全部通过 ✅` | 66 |
| 6 | `test_fix.py` | 0 | `结果：74/74 全部通过 ✅` | 74 |
| 7 | `test_geometry.py` | 0 | `结果：38/38 全部通过 ✅` | 38 |
| 8 | `test_localization.py` | 0 | `结果：89/89 全部通过 ✅` | 89 |
| 9 | `test_mission.py` | 0 | `结果：97/97 全部通过 ✅` | 97 |
| 10 | `test_path_overlap.py` | 0 | `结果：32/32 全部通过 ✅` | 32 |
| 11 | `test_pivot.py` | 0 | `结果：41/41 全部通过 ✅` | 41 |
| 12 | `test_planner.py` | 0 | `🎉 全部通过：53 项` | 53 |
| 13 | `test_pursuit.py` | 0 | `结果：全部通过 (ALL PASS)`；其内是 `断言：PASS 100 / FAIL 0（共 100 条，耗时 0.07 s）` | 100 |
| 14 | `test_referee.py` | 0 | `结果：81/81 全部通过 ✅` | 81 |
| 15 | `test_route.py` | 0 | `结果：125/125 通过，0 失败` | 125 |
| 16 | `test_safety.py` | 0 | `结果：162/162 全部通过 ✅` | 162 |
| 17 | `test_sim.py` | 0 | `结果：82/82 全部通过 ✅` | 82 |
| 18 | `test_sim_guard.py` | 0 | `结果：104/104 全部通过 ✅` | 104 |
| 19 | `test_telemetry.py` | 0 | `结果：79/79 全部通过 ✅` | 79 |
| 20 | `test_tracker.py` | 0 | `结果：74/74 全部通过 ✅` | 74 |

- **套件数 20 / 失败 0**，20 个 exit code 全为 **0**。
- **断言合计 1601**（= 上述 19 套件之和 1501 + `test_pursuit` 100）。
- 原始输出全文另存：`backup/cardA-baseline/logs/<套件名>.txt`，汇总 `logs/summary.json`。
- ⚠️ 与队长的会话级记录（「约 1486 断言」）**不一致**：我实跑得到 1601。差异来自 `test_route`（125，队长记账时可能按 120 计）与 `test_pursuit`（100，其"结果行"不含数字，容易漏计）。**以本表为准**，并可随时用 `logs/*.txt` 复核。

---

## 8. 给 B/C 卡的既有事实

1. **CLI 真源就是 `tools/plan_route.py`**（324 行 / 14508 B，与 HEAD 一致）；`python -m dzbrain` **当前跑不了**（`src/dzbrain/dzbrain/__main__.py` 缺失），卡 B 才补。卡 A 期间不要依赖 `-m dzbrain`。
2. **规划器默认仍是有倒车模式**：`planner.py:1206` `allow_reverse: bool = True`。CLI 实测输出 **6 段 `R:`**（`R:H11 R:E11 R:F11 R:D6 R:C6 R:G3`），其中 2 段来自折返改写（`_fix_uturn_reverse`，`planner.py:1394`）。
3. **无倒车模式已能解出合法路线**（实测 `R_seg=0`，`route_check` 0 error，1 个 warn「急转弯 20 处 / 最大 180°」），但**只能从 Python API 触发**：`tools/plan_route.py:151-154` 未传 `allow_reverse`，CLI 也无 `--no-reverse`（`:171-187`）。
4. **`:PIVOT` 在语法层就不存在**：`parser.py:61` `SCAN_SUFFIXES = frozenset({":SCAN_STOP", ":SCAN"})`，无 `:PIVOT`；`parser.py` 回写后缀序为 `R:`→格名→`*`→`:SCAN_STOP`→`,x|,y`。planner 里所有 `PIVOT` 字样都在**注释**（`planner.py:256/341/885/886/1219/1399`）。卡 C 若要加 `:PIVOT`，需同时动 `parser.py`（新后缀）+ `planner.py`（声明点）**并证明不改变 TrackerTask 行为**（PIVOT 现在由 `pursuit.pivot_angle_deg=135°`、`corner_turn_deg` 自动触发，见 `core/config.py:85-101`、`tasks/tracker.py:501-593`、`tasks/mission.py:173-178/584-600`）。
5. **`assert_legal_no_reverse` 已是完整实现**（24 行，`planner.py:881-902`），卡 B 直接复用即可，不必重写；它要求 0 段 `R:` 并禁止相邻重复点。
6. **测试锚点**：`test_planner.py` 有 6 处 `plan_route` 调用（`:266/329/361/381/427/438`），**没有一处传 `allow_reverse`**；卡 B 加 `--no-reverse` 时这两种模式都需要新断言，否则 1601 条里对该分支仍为 0 覆盖。
7. **`--check` 的语法层是降级校验**：`⚠️ 无法导入 path_executor（ModuleNotFoundError），改用内置副本校验` —— 车端真源 `path_executor` 未参与，token 合法性由内置等价副本判定。卡 B/C 若改动 token 语法，请注意这条降级路径会掩盖真源差异。
8. **`route_check` 是子进程调用原文件**（`tools/plan_route.py:108-132`，子进程强制 UTF-8 编码），不是复制规则；复现无倒车路线校验可直接 `python tools\route_check.py --route "<tokens>" --config config\robot.yaml`。
9. **备份与 HEAD 的差异已在 §6 末尾说明**：备份含那 57 行；`HEAD` 版本 69975 B / 1424 行。
10. **本机 shell 是 Windows PowerShell 5.1**：不要用 `Get-Content`/`Measure-Object -Line` 数 UTF-8 中文文件的行（会少算，见 finding A5），行号一律以 `read`/`grep` 为准；跑 Python 记得带 `$env:PYTHONUTF8='1'`、`$env:PYTHONIOENCODING='utf-8'`。

---

## 9. Findings（只报告，未修任何代码）

### A1 · high · 无倒车模式在 CLI 与车上路径上不可达
- 位置：`tools/plan_route.py:151-154`（调用未传 `allow_reverse`）、`tools/plan_route.py:171-187`（无 `--no-reverse`）、`src/dzbrain/dzbrain/route/planner.py:1206`（默认 `True`）
- 复现：
  ```powershell
  $env:PYTHONPATH='D:\User\Desktop\dzrobot_v2\src\dzbrain'; python tools\plan_route.py --emit tokens --quiet
  ```
- 期望：车上默认应给出 **0 段 `R:`** 的路线（目标原文「0 段 R:」）。
- 实际：输出含 **6 段 `R:`**；CLI 无法表达 `allow_reverse=False`。
- 影响：卡 B 的核心交付（把开关接出来 + 默认值定为车上口径）。

### A2 · high · `:PIVOT` 声明 token 完全不存在
- 位置：`src/dzbrain/dzbrain/route/parser.py:61`（`SCAN_SUFFIXES` 无 `:PIVOT`）；`src/dzbrain/dzbrain/route/planner.py:256/341/885/886/1219/1399`（仅注释）
- 复现：`grep ':PIVOT'`（正则 `:\s*PIVOT`）在 `src/ tools/ config/` 中 0 命中；`python tools\plan_route.py --emit tokens --quiet` 输出中 0 命中
- 期望：折返点应带 `:PIVOT` 声明（目标原文「折返点 :PIVOT」）。
- 实际：无该后缀；折返点目前只表现为「正常前进航点」，PIVOT 由 `pursuit` 侧按角度自动触发。
- 影响：卡 C 的交付；且必须保持「声明/断言不改变 TrackerTask 行为」。

### A3 · medium · 无倒车分支零测试覆盖
- 位置：`src/dzbrain/tests/test_planner.py:266/329/361/381/427/438`
- 复现：`grep allow_reverse` 全仓 `*.py` → 10 处命中全在 `planner.py`；`plan_route(` 调用点 8 处均未传该参数
- 期望：新增分支至少有正/负向断言（0 段 `R:` 通过、出现 `R:` 则抛错）。
- 实际：1601 条断言中对该分支覆盖为 **0**。

### A4 · medium · `--check` 语法层降级为内置副本
- 位置：`tools/plan_route.py:108-132`（子进程 `route_check.py`）；输出行 `⚠️ 无法导入 path_executor（ModuleNotFoundError），改用内置副本校验`
- 复现：`python tools\plan_route.py --check`（§3 原始输出）
- 期望：token 合法性由车端真源 `path_executor` 判定（或明确失败）。
- 实际：静默降级为「内置等价副本」，仍报 ✅；差异可能被掩盖。
- 备注：这可能**不是**缺陷而是刻意的开发机兼容（车端才有 path_executor）；但卡 B/C 改 token 语法时必须知道校验源不是真源。

### A5 · low · 任务背景里的行数口径是错的（会给 B/C 卡误导）
- 复现（Python 权威计数）：
  ```powershell
  python -c "b=open('src/dzbrain/dzbrain/route/planner.py','rb').read();print(len(b), b.count(b'\n'), b.count(b'\r'))"
  ```
  输出：`73254 1476 0`
- 期望/实际：任务背景称 `planner.py` 1152 行、`tools/plan_route.py` 260 行；**实际为 1476 行 / 324 行**（Python 计数，无 CR、无 BOM）。
- 根因：本机 `pwsh` 实为 **Windows PowerShell 5.1**，按 ANSI(CP936) 解码含中文的 UTF-8 文件会吞掉部分换行；`Measure-Object -Line` 又跳过空行 → 得出 1152 / 260。交叉验证：`Get-Content` 项数 1305、`-Raw` 按 LF 切分 1306，与 Python 的 1477 段都不符，可复现该口径失真。
- 影响：把「行号引用」当验收依据时必须以 `read`/`grep` 的行号为准（本报告 §4/§5 的行号已用 `read` 复核）。

---

## 10. 未执行 / 未覆盖（如实标注）

| 条目 | 状态 |
|---|---|
| 在真车 / 车端 `install/` 空间跑规划器 | **未执行**（本卡只做工作区只读验收；网络不可达与地面栈未运行不在本卡范围） |
| `python -m dzbrain ...` 的端到端启动 | **未执行**（`__main__.py` 缺失，命令必然失败，属卡 B 范围） |
| `mission._maybe_auto_start()` 真正消费规划结果的链路 | **未执行**（属卡 D 全链路验证） |
| `:PIVOT` 端到端行为（Parser→Tracker→真车） | **未执行**（该 token 目前不存在） |
| 随机地图 / `site_*.yaml` 变体 | **未执行**（本卡只用唯一地图 `config/map.yaml`） |
| `allow_reverse=False` 的几何最优性（是否比有倒车更长/更差） | **部分执行**：只记录了总里程 17.15 m vs 16.45 m，未做代价比较 |
| 修改任何源码 / `git commit` / `git push` / 部署 | **未执行**（纪律要求；`git status` 结束时仍只有 ` M planner.py` + `?? .agent-teams/`，另新增 `backup/`、`docs/车上自动规划_A_基线验收.md` 两个本卡交付物） |

---

## 附：本卡新增/写入的文件清单

| 路径 | 说明 |
|---|---|
| `backup/cardA-baseline/`（6 文件副本，保留相对路径） | 卡 B/C 改动前基线（含 A 卡 57 行改动，见 §6） |
| `backup/cardA-baseline/git-state.txt` | `git status --porcelain` + `git log --oneline -3` 原始输出（528 B） |
| `backup/cardA-baseline/sha256.txt` | 上述 6 文件 + git-state + diff 的 sha256 与字节数清单 |
| `backup/cardA-baseline/planner-57lines.diff` | 那 57 行未提交改动的完整 diff（8019 B） |
| `backup/cardA-baseline/logs/` | 20 套件 + 探针 + route_check 的完整原始输出（20 txt + summary.json） |
| `backup/cardA-baseline/probe_no_reverse.py` | 只读对比探针（True/False 两种模式） |
| `docs/车上自动规划_A_基线验收.md` | 本报告 |
