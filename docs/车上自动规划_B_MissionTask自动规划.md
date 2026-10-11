# 任务卡 B —— MissionTask 自动规划（读 map.yaml 自算路线）+ planner 配置段 + `--plan-only`

- 工作区：`D:\User\Desktop\dzrobot_v2`（Windows，**Windows PowerShell 5.1**，`python` = 3.12）
- 执行者：`mission-autoplan-v5`（attempt 12，接手 attempt 11 的落盘产物并补完/返工）
- 一次性验证批次：**2026-10-11 07:58:03 → 07:59:13**，批次内 8 个受关注文件的头尾 sha256 **全部一致**
  （即这批证据属于同一个版本；并行队友中途改文件会让头尾哈希不等，已内置该检查）

## 0. 结论（先说能不能用）

| 验收项 | 结果 | 证据位置 |
|---|---|---|
| 1. `planner:` 段（同级同缩进、恰好 5 键） | ✅ | §2（YAML 行号 + `git diff` 原文） |
| 2. `auto_plan_enabled=true` → 自动规划启动日志 | ✅ | §3（横幅 + 完整 token 列表 + 耗时 ms） |
| 3. `auto_plan_enabled=false` → 手动配置 + 基线等价 | ✅ | §4 |
| 4. 地图路径解析顺序（绝对→配置目录→工作区） | ✅ | §5（三候选都存在的对照实验） |
| 5. 回退分支（坏图 + fallback=true → 仍 DRIVE） | ✅ | §6 |
| 6. 不出发分支（坏图 + fallback=false → 停 IDLE） | ✅ | §7 |
| 7. `python -m dzbrain --plan-only`（新建 `__main__.py`） | ✅ | §8（exit 0 + 门禁豁免） |
| 8. 新增测试 + 全套 21 个套件 exit 0 | ✅ | §9（含 `test_autoplan.py` 88/88） |
| 9. 改动前备份 `backup/cardB-before/` | ⚠️ **返工后达成** | §10（attempt 11 备份是"改后"拍的，已重建） |
| 10. 本报告 | ✅ | 本文件 |
| 附加：**未修改 `tools/plan_route.py`** | ⚠️ **被并行队友改了** | §11（有作者归属证据，非本卡所为） |

三条需要看的人注意的**诚实结论**：

1. **"缺 `mines` 的坏图"不是一个失败用例** —— 它能正常解出路线（实测 32 token）。回退分支的坏图
   必须用别的方式构造（本卡用 `mines` 含非法格名 `Z9` → `PlannerError`）。详见 §6.1。
2. **`backup/cardB-before/` 第一版是"改后"拍的**（副本里已经含 `auto_plan`/`PlanOutcome`），
   不满足"改动前备份"。已用"逐行 diff 审计 + `git show HEAD:`"重建，并用**任务卡 A 的基线哈希**
   交叉验证。详见 §10。
3. **`tools/plan_route.py` 的 `M` 不是本卡所为**：内容全是 `--allow-reverse`/`--pivot-penalty`/`:PIVOT`
   （卡 C 范围），mtime 落在队友工作窗口内。详见 §11。

---

## 1. 交付物与版本钉

| 文件 | 行数（`\n` 口径） | 改动 | sha256（批次时） |
|---|---|---|---|
| `config/robot.yaml` | 529 | +10 | `8081DF9C…1CE06` |
| `src/dzbrain/dzbrain/tasks/mission.py` | 1323 | +238 / −3 | `A5978B92…D4767` |
| `src/dzbrain/dzbrain/main.py` | 467 | +77 / −2 | `00FA445D…CABB3` |
| `src/dzbrain/dzbrain/__main__.py` | 23 | **新建** | `4A9C3347…AB08E` |
| `src/dzbrain/tests/test_autoplan.py` | 512 | **新建** | `715FE81F…46D6` |
| `_scratch_auto_plan/mission/`（夹具/探针/日志） | —— | **新建** | （见 §12 清单） |
| `backup/cardB-before/` | —— | **新建/返工** | （见 §10） |
| `docs/车上自动规划_B_MissionTask自动规划.md` | 本文件 | **新建** | —— |

> ⚠️ **行数口径警告**（任务卡 A 已经踩过）：本机是 PowerShell 5.1，`Get-Content`/`Measure-Object -Line`
> 对**无 BOM 的 UTF-8 中文文件**会给出偏小的行数（实测 `mission.py`：`Get-Content` 默认 ANSI = **1193** 行、
> `Measure-Object -Line` = **1083** 行、`Get-Content -Encoding UTF8` = **1323** 行 = 真值）。
> 原因：ANSI(GBK) 解码时，`0x81~0xFE` 的前导字节会把紧跟的 `\n` 当作自己的第二字节**吃掉**。
> 复核脚本：`_scratch_auto_plan/mission/probe_line_counts.py`（同时核对 BOM/裸 CR）。

## 2. `planner:` 配置段（验收项 1）

`config/robot.yaml` **第 441–449 行**（`planner:` 在第 444 行，与 `route:`(423) / `mission:`(430)
同为 4 空格缩进 ⇒ 点号键 `planner.*`）：

```yaml
441:    # ---- ★车上自动规划（任务卡 B）：启动时自己读地图算路线 ----
442:    #   与 route:/mission: 同级同缩进（4 空格）→ 点号键 planner.*
443:    #   对应实现：tasks/mission.py 的 `_load_startup_route` / `plan_route_from_config`
444:    planner:
445:      auto_plan_enabled: true         # true=启动即规划并执行规划结果；false=完全走旧逻辑读 route.tokens
446:      map_path: "config/map.yaml"     # 解析顺序：绝对路径 → 相对配置文件目录 → 相对工作区
447:      fallback_to_tokens: true        # 规划失败（PlannerError/超时/坏图）→ warn 后回退 route.tokens，仍出发
448:      allow_reverse: false            # ★真车底盘不支持倒车 → 规划器走无倒车模式（0 段 R:，折返交 PIVOT）
449:      max_seconds: 2.0                # 规划时间预算；超出抛 PlannerError → 走上面的回退分支
```

`git diff config/robot.yaml` 原始输出（**纯插入 10 行、0 删除**）：

```diff
diff --git a/config/robot.yaml b/config/robot.yaml
index e5cea86..8403c61 100644
--- a/config/robot.yaml
+++ b/config/robot.yaml
@@ -438,6 +438,16 @@ path_executor:
       settle_time_s: 0.8
       bridge_info_wait_s: 20.0        # 等无人机桥洞情报的最长时间
 
+    # ---- ★车上自动规划（任务卡 B）：启动时自己读地图算路线 ----
+    #   与 route:/mission: 同级同缩进（4 空格）→ 点号键 planner.*
+    #   对应实现：tasks/mission.py 的 `_load_startup_route` / `plan_route_from_config`
+    planner:
+      auto_plan_enabled: true         # true=启动即规划并执行规划结果；false=完全走旧逻辑读 route.tokens
+      map_path: "config/map.yaml"     # 解析顺序：绝对路径 → 相对配置文件目录 → 相对工作区
+      fallback_to_tokens: true        # 规划失败（PlannerError/超时/坏图）→ warn 后回退 route.tokens，仍出发
+      allow_reverse: false            # ★真车底盘不支持倒车 → 规划器走无倒车模式（0 段 R:，折返交 PIVOT）
+      max_seconds: 2.0                # 规划时间预算；超出抛 PlannerError → 走上面的回退分支
+
     aim:
       # ★ 移动轨道靶（G11+H11 轨道，行程 0.70m，4cm/s，无停顿来回）
       lead_time_s: 0.35               # 提前量补偿时间（旧版只预判 0.5 帧≈16ms，链路延迟却有 0.5~0.9s）
```

被测试钉住：`test_autoplan.py` **8.1**（`planner.*` 是真实点号键）、**8.2**（4 空格同级）、
**8.3**（恰好这 5 个键值）、**8.4**（5 行都在 `planner:` 段内部）。

## 3. 模式 A：`auto_plan_enabled=true` 启动日志（验收项 2）

命令与完整日志：`python -m dzbrain --bench --seconds 15`
→ `_scratch_auto_plan/mission/logs/v4_auto.txt`（149 行，exit 0）

下面是**启动段逐行原文**（1–12 行进程横幅；89–100 行复位→规划→载入→出发→任务横幅）。
中间 13–88 行是其它 6 个任务在"等 10 s 延时"窗口里的既有节拍日志（与本卡无关），
为篇幅略去并**在此显式声明**，全文见上面的文件。

```
  1| bench：自动规划路线 43 token / 13.5 ms （已同步给虚拟小车，保证统计口径一致）
  2| ==============================================================================
  3| dzbrain 启动 · 多线程任务架构
  4|   配置文件 : D:\User\Desktop\dzrobot_v2\config\robot.yaml
  5|   路线来源 : 自动规划
  6|   路线   : 自动规划（map_path=config/map.yaml，无倒车，预算 2.0s；实际 token 见任务开始横幅）
  7|   巡航速度 : 0.15 m/s
  8|   自动出发 : True（延时 10.0s）
  9|   越界余量 : 0.6 m
 10|   开火策略 : moving_lead（提前量 0.35s）
 11|   任务线程 : Localization@20Hz Safety@20Hz TrackerTask@50Hz MissionTask@10Hz AimTask@50Hz FixTask@10Hz DroneTask@5Hz TelemetryTask@2Hz
 12| ==============================================================================
...
 89| 复位 belief：N10 (0.525, 0.175) yaw=90.0°
 90| 自动规划成功：map=D:\User\Desktop\dzrobot_v2\config\map.yaml 无倒车 耗时 13.3 ms / 43 token / 17.15 m / 倒车段 0 / A* 代价 114.10
 91| 自动规划 token 列表（43）：N10 N9 M9 L9 K9 I9 I10 H10* F10 F8:PIVOT F9 F10 F11 E11 E10 C10 C9 C8 C7 C6 D6:PIVOT C6 B6 A6 A5 C5 C4 C2 D2 E2 F2 G2 G1 H1 J1 J2 J3 J4 K4 L4 M4 N4 N5,x
 92| 路线载入[自动规划(D:\User\Desktop\dzrobot_v2\config\map.yaml)]：路线[...] 43 点 / 17.15 m / 穿洞 {'lower': 'K9', 'upper': 'K4'}
 93|   N10, N9, M9, L9, K9, I9, I10, H10, F10, F8, F9, F10, F11, E11, E10, C10, C9, C8, C7, C6, D6, C6, B6, A6, A5, C5, C4, C2, D2, E2, F2, G2, G1, H1, J1, J2, J3, J4, K4, L4 …(+3)
 94| 自动出发：延时 10s 到，复位 'N10,y'
 95| 阶段 IDLE → DRIVE
 96| ==============================================================================
 97| 【任务开始】路线 43 点 / 17.15 m / 穿洞 {'lower': 'K9', 'upper': 'K4'}
 98| 停站点 2 个：7:2.8 42:17.14999999999999
 99| 扫描模式 continuous（continuous → scan_request=2）
100| 路线来源：自动规划（map=D:\User\Desktop\dzrobot_v2\config\map.yaml 耗时 13.3 ms 无倒车）
```

逐项对照验收要求：

| 要求 | 命中行 | 值 |
|---|---|---|
| 横幅「路线来源：自动规划」 | 5（启动横幅）、100（任务横幅） | `自动规划` |
| **完整** token 列表 | 91 | 43 个，*无省略号* |
| **规划耗时 (ms)** | 90、100 | `13.3 ms` |
| 执行路线 = `PlanResult.tokens` | 91 vs 92/93 | 43 = 43，逐 token 相同 |
| 无倒车（`allow_reverse=false`） | 90、100 | `无倒车`、`倒车段 0`、token 里 `R:` 段 = 0 |

> 注意第 93 行那个 `…(+3)` 是 `Route.describe(limit=40)` 的既有截断（**不是**本卡引入的），
> 验收要的"完整 token 列表"看的是第 91 行。测试 `3.6` 专门断言"无省略号且与独立规划同长"。

被测试钉住：`3.1`～`3.15`（阶段 DRIVE、两处横幅、`EV_MISSION_STARTED`/`EV_ROUTE_LOADED` 各一次、
token 列表 = 执行路线块名序列、长度一致、0 段 `R:`、耗时 > 0、遥测同源、与独立规划确定性一致、`无倒车` 字样、`路线载入[自动规划(` 前缀）。

## 4. 模式 B：`auto_plan_enabled=false` 启动日志（验收项 3）

命令与完整日志（配置 = 真 `robot.yaml` 副本，只把 `auto_plan_enabled` 改成 `false`）：
`python -m dzbrain --bench --seconds 12 --config _scratch_auto_plan/mission/manual_mode.yaml`
→ `_scratch_auto_plan/mission/logs/v5_manual.txt`（134 行，exit 0）

```
  1| ==============================================================================
  2| dzbrain 启动 · 多线程任务架构
  3|   配置文件 : _scratch_auto_plan/mission/manual_mode.yaml
  4|   路线来源 : 手动配置
  5|   路线   : 34 点（穿洞 K9 / K4）
  6|   巡航速度 : 0.15 m/s
  7|   自动出发 : True（延时 10.0s）
  8|   越界余量 : 0.6 m
  9|   开火策略 : moving_lead（提前量 0.35s）
 10|   任务线程 : Localization@20Hz Safety@20Hz TrackerTask@50Hz MissionTask@10Hz AimTask@50Hz FixTask@10Hz DroneTask@5Hz TelemetryTask@2Hz
 11| ==============================================================================
...
 87| 复位 belief：N10 (0.525, 0.175) yaw=90.0°
 88| 路线载入[route.tokens]：路线[route.tokens] 34 点 / 16.45 m / 穿洞 {'lower': 'K9', 'upper': 'K4'}
 89|   N10, M10, L10, L9, K9, J9, J10, I10, H10, G10, F10, F11, E11, F11, F10, F8, B8, B6, A6, B6, D6, C6, C4, C2, G2, H2, H1, I1, J1, J4, K4, L4, L5, N5
 90| 自动出发：延时 10s 到，复位 'N10,y'
 91| 阶段 IDLE → DRIVE
 92| ==============================================================================
 93| 【任务开始】路线 34 点 / 16.45 m / 穿洞 {'lower': 'K9', 'upper': 'K4'}
 94| 停站点 2 个：9:3.1500000000000004 33:16.45
 95| 扫描模式 continuous（continuous → scan_request=2）
 96| 路线来源：手动配置
```

**"与卡 A 基线逐字节等价"的判据**（不是"看着像"）：

1. 日志里**没有**「自动规划成功」「自动规划 token 列表」任何一行（=规划器压根没被调用）；
2. 载入 `source=route.tokens`（第 88 行），34 点 / 16.45 m，与 `config/robot.yaml` 里
   `path_executor.mission_blocks`（经 `_migrate_legacy` 映射成 `route.tokens`）逐点相同；
3. 遥测 `mission_state.plan == {}`（`test_autoplan.py` **4.6**）；
4. 极端对照：把 `planner.map_path` 指向**不存在的文件**，手动模式下依旧照常出发、路线不变
   （**4.8**）—— 证明手动模式真的不碰规划器，而不是"碰了但没出错"。

## 5. 地图路径解析顺序（验收项 4）

实现：`tasks/mission.py` 的 `resolve_map_path()`（:142-161）—— **绝对路径 → 相对配置文件目录
→ 相对工作区**，三个候选按序 `is_file()`；都不存在时返回工作区候选（错误信息里能看到实际尝试路径）。
`workspace_root()`（:122-131）用 `parents[4]`（`tasks/` 比 `main.py` 深一层，取 `[3]` 会得到
`<ws>/src/config/map.yaml` —— 这个 off-by-one 在开发时真的踩到过）。

测试（`test_autoplan.py` §2）构造了**三个候选都存在**的对照实验，这是唯一能证明"优先级"的方式：

| 用例 | 布置 | 断言 |
|---|---|---|
| 2.1 | 绝对路径 | 原样返回，不拼接 |
| **2.2** | `<cfg目录>/_scratch_auto_plan/mission/order_probe.yaml` 与 `<ws>/_scratch_auto_plan/mission/order_probe.yaml` **同时存在** | 取**配置目录**那份（marker=`CONFIGDIR`，不是 `WORKSPACE`） |
| 2.3 | 同上，读内容 | `CONFIGDIR` ✅ |
| 2.4 | 删掉配置目录那份 | 落到**工作区**那份 |
| 2.5 | 两个都不存在 | 返回工作区候选且 `is_file()` 为假 |
| 2.6 | `Config.from_dict`（无真实文件） | 只走工作区候选，不炸 |

真机链路也验证过：`--plan-only` 打印 `地图: D:\User\Desktop\dzrobot_v2\config\map.yaml`（cwd 无关，
测试 **7.9** 把 cwd 换到上级目录后再断言）。

## 6. 回退分支：坏图 + `fallback_to_tokens=true`（验收项 5）

命令与完整日志：`python -m dzbrain --bench --seconds 3 --config _scratch_auto_plan/mission/fallback_true.yaml`
（该副本的 `map_path` 是**相对配置文件目录**的 `"broken_map.yaml"`，顺带验证了解析规则 2）
→ `_scratch_auto_plan/mission/logs/v6_fallback_on.txt`（99 行，exit 0）

```
      路线来源 : 自动规划                       ← 进程横幅 = "配置口径"
      路线   : 自动规划（map_path=broken_map.yaml，无倒车，预算 2.0s；实际 token 见任务开始横幅）
      自动出发 : True（延时 0.5s）
      规划失败（将按 fallback_to_tokens 处理）：map=_scratch_auto_plan\mission\broken_map.yaml 预算 2s PlannerError: 地图 mines 里的格名非法: 非法列字母: 'Z9'（应为 A~N）
    ⚠️ 自动规划失败：map=_scratch_auto_plan\mission\broken_map.yaml 无倒车 预算 2s → PlannerError: 地图 mines 里的格名非法: 非法列字母: 'Z9'（应为 A~N）；fallback_to_tokens=true → 回退 route.tokens 并继续出发
    路线载入[route.tokens(自动规划失败回退)]：路线[...] 34 点 / 16.45 m / 穿洞 {'lower': 'K9', 'upper': 'K4'}
    自动出发：延时 0.5s 到，复位 'N10,y'
    阶段 IDLE → DRIVE
    【任务开始】路线 34 点 / 16.45 m / 穿洞 {'lower': 'K9', 'upper': 'K4'}
    路线来源：手动配置                        ← 任务横幅 = "实际口径"（确实用的是手写路线）
```

✅ warn 含**异常类型**（`PlannerError`）+ **消息**（`地图 mines 里的格名非法: 非法列字母: 'Z9'`）；
✅ 回退到 `route.tokens`（34 点，与 §4 基线一致）；✅ **仍进入 DRIVE**（`阶段 IDLE → DRIVE`）。

### 6.1 为什么坏图夹具不是"缺 mines 的副本"（重要，改了一个反直觉的坑）

验收原文写的是"`broken_map.yaml`，**缺 `mines` 的副本** 或 不存在的路径"。第一版夹具就是"整行删掉
`mines`"，**实测它规划成功了**：

```
_scratch_auto_plan/mission/broken_map_missing_mines_probe.yaml
    load_map OK mines=0 -> plan_route OK 32 token / R段=0 / 5.6 ms
_scratch_auto_plan/mission/broken_map.yaml
    load_map 失败 -> PlannerError: 地图 mines 里的格名非法: 非法列字母: 'Z9'（应为 A~N）
config/map.yaml
    load_map OK mines=8 -> plan_route OK 43 token / R段=0 / 13.3 ms
```

原因：`load_map` 对缺失的 `mines` 取空元组（`_as_cells(None) → ()`），而"必须压过所有 mines"
在**空集合上恒真** ⇒ 规划器照样给出 32 token 的合法路线。**用"缺 mines"测回退分支等于测了个假分支。**
故夹具改为 `mines: ["H1", "Z9"]`（非法列字母）—— 失败发生在 `load_map` 阶段，抛 `PlannerError`。
探针：`_scratch_auto_plan/mission/probe_broken_map.py`（日志 `logs/probe_broken_map.txt`）。
另备 `bad_yaml_map.yaml`（YAML 语法非法 → `ParserError`，**不是** PlannerError），
用来测"宽捕获 `except Exception`"这条路径（测试 **1.10**）。

## 7. 反向分支：坏图 + `fallback_to_tokens=false`（验收项 6）

命令：`python -m dzbrain --bench --seconds 3 --config _scratch_auto_plan/mission/fallback_false.yaml`
→ `_scratch_auto_plan/mission/logs/v7_fallback_off.txt`（89 行，exit 0）

```
      规划失败（将按 fallback_to_tokens 处理）：map=...\broken_map.yaml 预算 2s PlannerError: 地图 mines 里的格名非法: 非法列字母: 'Z9'（应为 A~N）
    ❌ 自动规划失败：map=_scratch_auto_plan\mission\broken_map.yaml 无倒车 预算 2s → PlannerError: 地图 mines 里的格名非法: 非法列字母: 'Z9'（应为 A~N）；fallback_to_tokens=false → 停在 IDLE，不出发
  运行时长  : 3.0 s
  阶段/位置 : — @ —            ← 从未进入 DRIVE（没有横幅、没有路线载入）
  事件总数  : 0                ← ★ 整场 0 个事件 ⇒ 没有 EV_MISSION_STARTED / EV_ROUTE_LOADED
  [STOPPED ] MissionTask    ticks=31      errors=0 overruns=0 max_tick=2.19ms
```

✅ 停 IDLE、✅ error（`❌`）日志含异常类型+消息、✅ **不 publish `EV_MISSION_STARTED`**（事件总数 0）、
✅ 不出发。测试另加了三条"有牙"的断言：**6.7** 定格（error 只打一次，不是每拍刷屏）、
**6.11** 遥测里能看到失败证据、**6.10** 定格不污染后续任务。

## 8. `python -m dzbrain --plan-only`（验收项 7）

新建 `src/dzbrain/dzbrain/__main__.py`（23 行，只做转发）：

```python
from .main import main
if __name__ == "__main__":
    raise SystemExit(main())
```

**为什么必须有它**：文档/`tools/`/验收命令用的都是 `python -m dzbrain ...`，而缺 `__main__.py` 时
该命令直接失败（`No module named dzbrain.__main__`）⇒ 这一层根本不可达。

原始输出（`_scratch_auto_plan/mission/logs/v1_plan_only.txt`，**exit 0**）：

```
==============================================================================
静态规划结果（--plan-only：未起线程、未发指令、未连 ROS）
地图: D:\User\Desktop\dzrobot_v2\config\map.yaml
模式: 无倒车（planner.allow_reverse=False）
token 数: 43 / 折线 17.15 m / 倒车段 0
A* 代价: 114.10
==============================================================================
token 列表（43）：N10 N9 M9 L9 K9 I9 I10 H10* F10 F8:PIVOT F9 F10 F11 E11 E10 C10 C9 C8 C7 C6 D6:PIVOT C6 B6 A6 A5 C5 C4 C2 D2 E2 F2 G2 G1 H1 J1 J2 J3 J4 K4 L4 M4 N4 N5,x
规划耗时：13.7 ms
```

| 要求 | 证据 |
|---|---|
| 只规划、打印 token + 耗时 | 上面输出（`token 列表（43）`、`规划耗时：13.7 ms`） |
| 退出码 0 | `exit=0`（批次表） |
| **不起线程** | 测试 **7.7**：调用前后 `threading.active_count()` 不变（1 → 1） |
| **不发指令** | `run_plan_only()` 里不建 Bridge、不 import `io/ros_io.py`（`grep` 可查） |
| **无 ROS** | 测试 **7.8**：调用前后 `rclpy` 均不在 `sys.modules` |
| 不传 `--config` 也能解析 | 测试 **7.2** 断言 `default_config_path()` == `<ws>/config/robot.yaml`（本机 `~/dzrobot/config/robot.yaml` 不存在） |

### 8.1 顺手修掉一个死代码（`--plan-only` 原来被门禁挡死）

`main()` 原来的顺序是 `load_and_check()` → `if errors: return 2` → `if args.dry_run` → `if args.plan_only`
⇒ 第 4 步那句 `if errors:` 是**永不执行的死代码**，而更早的 `return 2` 会把 `--plan-only` 直接掐死。
本卡把它挪到门禁之前（**这是功能前提**：`auto_plan_enabled=true` 时车上执行的是规划结果，
而门禁校验的恰恰是 `route.tokens` —— 手写路线一过时就"什么都算不出来"，而它正是唯一的离线入口）。

实测（`_scratch_auto_plan/mission/gate_fail.yaml` = 真配置 + `route.tokens: ["N10","N5"]`，
门禁报 7 条 error：R003/R005/R007×3/R012/R006）：

```
python -m dzbrain --plan-only --config ...\gate_fail.yaml   → exit 0（打印门禁错误后照样规划）
python -m dzbrain --dry-run   --config ...\gate_fail.yaml   → exit 2（门禁语义未变）
python -m dzbrain --bench 0.5 --config ...\gate_fail.yaml   → exit 2（门禁语义未变）
```

测试 **7.14/7.15/7.16/7.17** 把这三条钉住（`--plan-only` 仍继续 + 门禁错误如实打印 + dry-run/bench 不放行）。

## 9. 测试证据（验收项 8）

新建 `src/dzbrain/tests/test_autoplan.py`（512 行，纯脚本无 pytest，`from harness import check, section, finish`）。

**全套 21 个套件 exit 0**（`_scratch_auto_plan/mission/logs/BATCH.txt`）：

```
test_aim.py              exit=0  结果：122/122 全部通过 ✅
test_autoplan.py         exit=0  结果：88/88 全部通过 ✅      ← 本卡新增
test_contract.py         exit=0  结果：56/56 全部通过 ✅
test_core.py             exit=0  结果：50/50 全部通过 ✅
test_deploy.py           exit=0  结果：76/76 全部通过 ✅
test_drone.py            exit=0  结果：66/66 全部通过 ✅
test_fix.py              exit=0  结果：74/74 全部通过 ✅
test_geometry.py         exit=0  结果：38/38 全部通过 ✅
test_localization.py     exit=0  结果：89/89 全部通过 ✅
test_mission.py          exit=0  结果：97/97 全部通过 ✅
test_path_overlap.py     exit=0  结果：32/32 全部通过 ✅
test_pivot.py            exit=0  结果：41/41 全部通过 ✅
test_planner.py          exit=0  🎉 全部通过：53 项
test_pursuit.py          exit=0  结果：全部通过 (ALL PASS)
test_referee.py          exit=0  结果：81/81 全部通过 ✅
test_route.py            exit=0  结果：125/125 通过，0 失败
test_safety.py           exit=0  结果：162/162 全部通过 ✅
test_sim.py              exit=0  结果：82/82 全部通过 ✅
test_sim_guard.py        exit=0  结果：104/104 全部通过 ✅
test_telemetry.py        exit=0  结果：79/79 全部通过 ✅
test_tracker.py          exit=0  结果：74/74 全部通过 ✅
套件数=21  通过=21  失败=[]
```

断言合计（本批次）≈ **1689** = 基线 20 套件 1601 + 本卡 88。

> ⚠️ 断言总数会随并行队友的改动变化：重跑时 `test_planner` 53→83、`test_route` 125→154，
> 合计约 **1748**。**判据是"每套 exit 0"，不是断言总数**（本报告只保证本批次那个版本的 1689）。

`test_autoplan.py` 的 88 项按验收项映射：

| 章节 | 覆盖 | 关键项 |
|---|---|---|
| §1（14） | 规划入口纯函数 | 真地图成功/0 段 `R:`；坏图 `PlannerError`（含类型+消息）；YAML 语法错走宽捕获；文件不存在；**"缺 mines 不是失败用例"** |
| §2（6） | 路径解析顺序 | 三候选同时存在的优先级对照 |
| §3（15） | 模式 A | 横幅/完整 token 列表/耗时/`执行路线==PlanResult.tokens`/0 段 `R:`/确定性 |
| §4（8） | 模式 B | 手动配置横幅 + 基线等价 + 完全不调规划器（含"map_path 指向不存在文件也照常出发"） |
| §5（9） | 回退分支 | warn 内容/回退路线/仍 DRIVE/遥测证据/文件不存在同样回退 |
| §6（11） | 不出发分支 | 60 拍仍 IDLE/无事件/`P_ROUTE` 未写/error 只打一次/定格不污染 |
| §7（17） | `--plan-only` | exit 0/token+耗时/不起线程/无 rclpy/cwd 无关/坏图 exit 1/**门禁豁免 + dry-run·bench 语义未变** |
| §8（8） | 边界 | 5 键与缩进/缺 `planner` 段向后兼容/非法 `max_seconds`/顶层非映射 |

## 10. 改动前备份（验收项 9）—— ⚠️ 返工项

**发现的问题**：attempt 11 建的 `backup/cardB-before/` 里那些"原样副本"**已经含有卡 B 的代码**
（`auto_plan` / `plan_route_from_config` / `PlanOutcome`），而且是改动**之后**才落盘的 ——
判据是 `config/robot.yaml` 的副本 sha256 = `8081df9c…`，与**当前工作区**文件哈希完全相同
（真"改前"副本不可能等于改后的文件）。也就是说"开工第一步备份"这件事**实际上没做到**。

**返工做法**（不猜测、不手改内容）：

1. **逐行 diff 审计**证明"未提交的改动全部属于卡 B"，从而"改动前的工作区 == HEAD 版"：
   `robot.yaml` +10/−0；`main.py` +77/−2（删的两行是「路线」横幅原文）；`mission.py` +238/−3
   （删的三行是 `from dataclasses import dataclass`、旧 `__all__`、旧的
   `self._load_tokens(route.tokens)`）；`test_mission.py` 0/0（本卡没改）。**没有任何与卡 B 无关的改动。**
2. 用 `git show HEAD:<path>` **按原字节**重写成真·改前副本；误存的"改后"副本**改名保留**为
   `POSTCHANGE__*`（不删，留证）。
3. **第三方交叉验证**：重建后的哈希与**任务卡 A 冻结的基线**逐字节一致 ——
   `robot.yaml 952584b5…/33357`、`main.py 26bfceda…/17879`、`mission.py 519394c8…/56126`（三项全中）。
4. 顺带修掉第二处坏证据：attempt 11 的 `HEAD__*` 是 PowerShell 写的，**换行全被吃掉**
   （`0x0a` 计数 = 0，整文件一行）且多带 UTF-8 BOM ⇒ 既不是 git blob 也无法对照。已按 blob 原字节重写：

```
HEAD__config__robot.yaml                旧 32841 字节/换行 0  →  新 33357 字节/519 行
HEAD__src__...__main.py                 旧 17490 字节/换行 0  →  新 17879 字节/392 行
HEAD__src__...__tasks__mission.py       旧 55041 字节/换行 0  →  新 56126 字节/1088 行
HEAD__src__...__tests__test_mission.py  旧 47568 字节/换行 0  →  新 48485 字节/920 行
```

**现状**（`probe_backup.py`，exit 0）：

```
文件                                           改前行数     工作区     含卡B符号?   =HEAD副本    =卡A基线
config/robot.yaml                             519     529      False      True     True
src/dzbrain/dzbrain/main.py                   392     467      False      True     True
src/dzbrain/dzbrain/tasks/mission.py         1088    1323      False      True     True
src/dzbrain/tests/test_mission.py             920     920      False      True     True
结论：✅ 改前快照可信（不含卡 B 代码，且与卡 A 基线哈希一致）
```

目录内容：4 份"改前"副本（保留相对路径）+ `HEAD__*`（= 改前，逐字节相同）+ `POSTCHANGE__*`（误存的改后副本）
+ `SHA256.txt`（三方哈希对照）+ `README.md`（审计过程与回滚命令）+ `git-state.txt`。
审计/重建脚本：`_scratch_auto_plan/mission/fix_backup.py`（`--apply` 才落盘）。

> 教训（已写进 README）：本机是 **Windows PowerShell 5.1**，`>` / `Out-File` / `Set-Content`
> **都不是"原样二进制"**（会加 BOM、吃换行、或写 UTF-16）——存快照必须用
> `[System.IO.File]::WriteAllBytes` 或 Python `write_bytes`。本报告的批次驱动器
> `verify_batch.py` 因此**改用 Python**（第一版 `.ps1` 的日志被写成 UTF-16/BOM，不可读，已删）。

## 11. `tools/plan_route.py` 的 `M` 不是本卡所为（验收项 4 的例外）

`git status --porcelain` **当前**（07:59 之后）是：

```
 M config/robot.yaml                 ← 卡 B
 M src/dzbrain/dzbrain/main.py       ← 卡 B
 M src/dzbrain/dzbrain/route/model.py       ← 并行队友（卡 C）
 M src/dzbrain/dzbrain/route/parser.py      ← 并行队友（卡 C）
 M src/dzbrain/dzbrain/route/planner.py     ← 并行队友（卡 C，含卡 A 的 57 行）
 M src/dzbrain/dzbrain/tasks/mission.py     ← 卡 B
 M src/dzbrain/tests/test_planner.py        ← 并行队友（卡 C）
 M src/dzbrain/tests/test_route.py          ← 并行队友（卡 C）
 M tools/plan_route.py                      ← 并行队友（卡 C）
 M tools/route_check.py                     ← 并行队友（卡 C）
?? .agent-teams/
?? docs/车上自动规划_A_基线验收.md
?? src/dzbrain/dzbrain/__main__.py          ← 卡 B
?? src/dzbrain/tests/test_autoplan.py       ← 卡 B
```

`tools/plan_route.py` 的改动内容**全是卡 C 的范围**（`--allow-reverse` / `--no-reverse` /
`--pivot-penalty` / `:PIVOT` 的文档与参数），diff 里没有一行与卡 B 相关：

```diff
+    # ★ 倒车开关：默认**无倒车**（真车底盘不支持倒车，180° 折返由循迹层 PIVOT 完成）
+    python tools/plan_route.py --emit tokens          # 默认：0 段 R: + 折返点 :PIVOT
+    python tools/plan_route.py --allow-reverse        # 旧行为：允许 R: 倒车段（卡 A 基线）
...
+        allow_reverse=bool(args.allow_reverse), pivot_penalty=args.pivot_penalty,
```

旁证：任务书自己写明"`planner-pivot` 正在并行改 `route/planner.py` / `route/parser.py` /
**`tools/plan_route.py`**（都在你的 outOfScope）"；文件 **mtime = 2026-10-11 07:58:25，正好落在
本卡验证批次窗口（07:58:03–07:59:13）内**，而本卡自己的文件 mtime 都在 07:56 前后；
本卡对 `tools/` 的写入次数为 **0**（本卡所有写入都在 `config/robot.yaml`、`tasks/mission.py`、
`main.py`、`__main__.py`、`test_autoplan.py`、`_scratch_auto_plan/`、`backup/`、`docs/`）。

> 补充说明（避免误读批次结论）：本批次"头尾哈希一致"的检查只覆盖 **8 个受关注文件**
> （见 §1 表 + `route/planner.py`、`route/parser.py`、`route/model.py`），**不含 `tools/`**。
> `src/dzbrain` 不 import `tools/` 下任何模块（`grep 'route_check|tools\.' src/dzbrain` 只匹配到
> 名为 `path_executor` 的**配置键**，不是 import），所以 `tools/plan_route.py` 在第 22 秒被改写
> **不影响**本批次的 `--plan-only` / `--dry-run` / `--bench` / 21 个套件结论。

**结论**：无法给出"`tools/plan_route.py` 未修改"的 porcelain 证据（它确实被**别人**改了），
但可以给出"**本卡没有修改它**"的归属证据。若收口时要求工作区干净，应由卡 C 的负责人决定提交顺序。

## 12. 复现方式（一次跑完）

```powershell
cd D:\User\Desktop\dzrobot_v2
$env:PYTHONPATH='D:\User\Desktop\dzrobot_v2\src\dzbrain'
$env:PYTHONIOENCODING='utf-8'; $env:PYTHONUTF8='1'
python -m dzbrain --plan-only                 # 契约 verify 1，exit 0
python -m dzbrain --dry-run                   # 契约 verify 2，exit 0
python -m dzbrain --bench --seconds 5         # 契约 verify 3，exit 0
Get-ChildItem src\dzbrain\tests\test_*.py | ForEach-Object { python $_.FullName; if ($LASTEXITCODE -ne 0) { throw "SUITE FAIL: $($_.Name)" } }
python _scratch_auto_plan\mission\verify_batch.py     # 上面全部 + 6 个模式日志 + 头尾哈希一致性检查
python _scratch_auto_plan\mission\probe_broken_map.py # "缺 mines 能解 / 非法格名不能解"的探针
python _scratch_auto_plan\mission\probe_backup.py     # 备份体检（exit 0 = 改前快照可信）
python _scratch_auto_plan\mission\probe_line_counts.py# 行数口径（PS 5.1 陷阱）
```

`_scratch_auto_plan/mission/` 目录：

| 文件 | 用途 |
|---|---|
| `broken_map.yaml` | 坏图夹具：`mines` 含非法格名 `Z9` → `PlannerError` |
| `bad_yaml_map.yaml` | 坏图夹具：YAML 语法非法 → `ParserError`（测宽捕获） |
| `broken_map_missing_mines_probe.yaml` | 反例探针：**缺 mines 反而能解** |
| `not_a_mapping_map.yaml` | 顶层非映射 → `ValueError` |
| `manual_mode.yaml` / `fallback_true.yaml` / `fallback_false.yaml` | 三种模式的真配置副本 |
| `gate_fail.yaml` | 门禁必红（`route.tokens: ["N10","N5"]`） |
| `verify_batch.py` / `probe_*.py` / `fix_backup.py` / `make_gate_fail_cfg.py` | 验证与审计脚本 |
| `logs/` | 全部原始输出（`BATCH.txt`、`v1..v7_*.txt`、`suite_*.txt`） |

## 13. 未做 / 不确定 / 待裁决

1. **`planner.*` 未登记进 `core/config.py` 的 SCHEMA 与 `known_prefixes`**（不在本卡允许改动范围内）：
   `Config.validate()` 会给这 5 个键各记一条 "未知配置段（拼写错误？）" **warn**（不是 error，门禁照过）；
   副作用是**键名拼错不会被 SCHEMA 兜住**。建议收口时把 `planner.` 加进 `known_prefixes` 并补 5 个 Spec。
2. **`P_MISSION_PHASE` 在开局 IDLE 时不写黑盒**：`_write_phase()` 首行 `if phase == self._phase: return`，
   而构造时 `_phase` 已是 IDLE ⇒ 起步阶段黑盒里该键是 `None`（bench 里表现为
   `刹车：阶段 未设置 不允许循迹`）。本卡**没有改它**（改的是既有 PIVOT 卡的行为，超范围），
   只在测试 6.6 改成断言遥测里的 `mission_state.phase`，并在此如实报告。
3. **两处"路线来源"是两个口径**：进程横幅（`main.py:start_banner`）按**配置**说"自动规划"，
   任务横幅（`MissionTask._banner_start`）按**实际**说"手动配置"。回退场景下二者不一致是**有意的**
   （配置说打算怎么跑、任务说实际怎么跑），但初次看会疑惑 —— 若嫌绕，可让进程横幅标
   "自动规划（失败则回退）"。
4. **未在真车/install 空间运行**，未 `git commit`、未 `git push`、未部署（遵守本卡纪律）。
   跨机对账（车 `10.226.211.188`、无人机）**全部不可达**，如实标 BLOCKED。
5. **并行改动风险**：`route/*`、`tools/*` 由 `planner-pivot` 并行演进（本批次期间
   `planner.py` 的哈希变过两次：`1B7D3807` → `EE3ED996` → `F266ADDA` → **`B0DA4D66`（本批次钉住）**）。
   本卡的调用方式只依赖冻结的 API（`plan_route(map_cfg, cfg, *, allow_reverse, max_seconds, source)`
   → `PlanResult.tokens/elapsed_ms`），新增 kwarg（如 `pivot_penalty`）不影响本卡。
   若队友再次改动导致 `--plan-only` 失败，请以**批次表里的哈希**为准重跑 `verify_batch.py`。
6. **`mission.auto_start_delay_s = 10.0`**：真车启动后 10 s 才出发（本卡没改这个值），
   所以 `--bench --seconds 5` 看不到"任务开始横幅"——要看启动横幅请用 `--seconds 15`（或 temp 配置）。

## 14. 收尾终验（交付前最后一跑）

`planner.py` sha256 = `B0DA4D66E60FE4DACA1A80C6424DCF1FD43FA5FCFAC015DC52E96B77090C0E72`
（**与 §9 批次钉住的哈希相同** ⇒ 批次证据仍然有效）。

```
##### 1) python -m dzbrain --plan-only
token 列表（43）：N10 N9 M9 L9 K9 I9 I10 H10* F10 F8:PIVOT F9 F10 F11 E11 E10 C10 C9 C8 C7 C6 D6:PIVOT C6 B6 A6 A5 C5 C4 C2 D2 E2 F2 G2 G1 H1 J1 J2 J3 J4 K4 L4 M4 N4 N5,x
规划耗时：13.0 ms                                        EXIT=0
##### 2) python -m dzbrain --dry-run
启动门禁通过：配置 D:\User\Desktop\dzrobot_v2\config\robot.yaml   EXIT=0
##### 3) python -m dzbrain --bench --seconds 5
bench：自动规划路线 43 token / 13.4 ms （已同步给虚拟小车，保证统计口径一致）
  路线来源 : 自动规划                                     EXIT=0
##### 4) 全部测试套件
套件数=21  失败=[]                                        （21/21 exit 0，含 test_autoplan 88/88）
```

