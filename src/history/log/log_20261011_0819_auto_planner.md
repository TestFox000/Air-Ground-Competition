# 车上自动规划 — 任务卡 D：自动规划全链路验证日志

- **执行者**：verifier-v5（独立验证者，任务卡 D；attempt 2）
- **执行时间**：2026-10-11 08:05:54 – 08:19（本机时钟）
- **工作区**：`D:\User\Desktop\dzrobot_v2`（HEAD = `fab01b7`，未 commit / 未 push / 未部署）
- **任务范围**：只读验证任务卡 A/B/C 的全部改动；本任务**未修改任何源码**，只写本报告与 `backup/cardD/`
- **总判定**：**failed** —— 第 1/3/4 步通过；**第 2 步「到点率 ≥ 95%」不成立**（实测 73.81% 严格 / 83.33% 含兜底），PIVOT 次数 = 折返点数成立。缺陷写成 finding D1–D4，未改代码。

## 0. 版本钉（证据同版本）

本批开始（08:05:54）与结束（08:19）两次采样，以下 10 个文件 sha256 **完全相同** ⇒ 本报告所有数字出自同一份代码：

| 文件 | sha256（前 16） | 字节 |
|---|---|---|
| `config/robot.yaml` | `8081df9c170435cd` | 34254 |
| `config/map.yaml` | `5560641dc749bd1a` | 2511 |
| `src/dzbrain/dzbrain/main.py` | `00fa445d4440e9ba` | 22925 |
| `src/dzbrain/dzbrain/__main__.py` | `4a9c3347eeaed486` | 956 |
| `src/dzbrain/dzbrain/route/planner.py` | `b0da4d66e60fe4da` | 84841 |
| `src/dzbrain/dzbrain/route/parser.py` | `f9fa4c6f8cbe6ea5` | 21356 |
| `src/dzbrain/dzbrain/route/model.py` | `678a8d1fab9c6e76` | 8565 |
| `src/dzbrain/dzbrain/tasks/mission.py` | `a5978b928cb19864` | 69428 |
| `tools/plan_route.py` | `e5bf63953d3898ed` | 16266 |
| `tools/route_check.py` | `d2c4068f8521b1be` | 39362 |

> `planner.py` 的 `b0da4d66…` 与任务卡 B 终验钉住的哈希一致 ⇒ A/B/C/D 证据同版本。
> 命令与环境（除非特别说明都一样）：
> `$env:PYTHONPATH='D:\User\Desktop\dzrobot_v2\src\dzbrain'; $env:PYTHONIOENCODING='utf-8'; $env:PYTHONUTF8='1'`
> 原始输出全部原样落盘在 `backup/cardD/verify2/logs/`（本报告引用的每段都能在那里逐字节核对）。

**规划器实际产出（本报告所有后续步骤都基于这一串）**

```
N10 N9 M9 L9 K9 I9 I10 H10* F10 F8:PIVOT F9 F10 F11 E11 E10 C10 C9 C8 C7 C6 D6:PIVOT C6 B6 A6 A5 C5 C4 C2 D2 E2 F2 G2 G1 H1 J1 J2 J3 J4 K4 L4 M4 N4 N5,x
43 token / 17.15 m / 倒车段 R: = 0 / :PIVOT = 2 / sha256(串) = 9aedac965e89d42775abfc7b5d3a5b9a388b2daa9478290a9fa77f8d04a7623e
```

---

## 1. 第 1 步：车端静态规划 `python -m dzbrain --plan-only`

**命令**

```powershell
$env:PYTHONPATH='D:\User\Desktop\dzrobot_v2\src\dzbrain'; $env:PYTHONIOENCODING='utf-8'; $env:PYTHONUTF8='1'
python -m dzbrain --plan-only
```

**结果：exit 0 × 3 次**（wall 0.194 / 0.194 / 0.195 s，stdout 610 B）

**原始输出（run1，`backup/cardD/verify2/logs/step1_plan_only_run1.txt`）**

```
==============================================================================
静态规划结果（--plan-only：未起线程、未发指令、未连 ROS）
地图: D:\User\Desktop\dzrobot_v2\config\map.yaml
模式: 无倒车（planner.allow_reverse=False）
token 数: 43 / 折线 17.15 m / 倒车段 0
A* 代价: 114.10
==============================================================================
token 列表（43）：N10 N9 M9 L9 K9 I9 I10 H10* F10 F8:PIVOT F9 F10 F11 E11 E10 C10 C9 C8 C7 C6 D6:PIVOT C6 B6 A6 A5 C5 C4 C2 D2 E2 F2 G2 G1 H1 J1 J2 J3 J4 K4 L4 M4 N4 N5,x
规划耗时：13.6 ms
```

三次 `规划耗时`：**13.6 / 14.0 / 13.8 ms**（另一批独立测量 15.0 ms；`tools/plan_route.py` 内部 `elapsed_ms` 13.39–14.02 ms）。

- **判定：通过。** 退出码 0；打印的地图路径就是 `config/map.yaml`；43 个 token 全部打印；**实测规划耗时 13.6–15.0 ms < 100 ms**（余量约 6.7 倍）。
- 附加证据（规划结果**确实随地图变化**，不是写死 token）：把 `mines` 的 `E11` 换成 `B5` 得到变体图，直接调用 `plan_route_from_config(cfg, map_path=变体)` 得到**另一条 38 token、无 `:PIVOT`** 的路线（`logs/map_override_probe.json`）。该探针同时暴露 finding D2。

---

## 2. 第 2 步：台架 `python -m dzbrain --bench`（自动规划路线）—— **判据不成立**

**命令（跑完整路线必须加 `--drive`，见 `--help`）**

```powershell
python -m dzbrain --bench --drive --seconds 240   # run1 → exit 0，203.3 s
python -m dzbrain --bench --drive --seconds 300   # run2（复现）→ exit 0，199.5 s
python -m dzbrain --bench --seconds 5             # 契约 verify 原命令 → exit 0，5.2 s
```

**原始统计行（run1，`logs/step2_bench_drive240.txt`；run2 见 `logs/step2_bench_drive300_run2.txt`）**

```
bench：自动规划路线 43 token / 15.1 ms （已同步给虚拟小车，保证统计口径一致）
自动规划成功：map=D:\User\Desktop\dzrobot_v2\config\map.yaml 无倒车 耗时 12.9 ms / 43 token / 17.15 m / 倒车段 0 / A* 代价 114.10
停站点 2 个：7:2.8 42:17.14999999999999
★ 进入 PIVOT（原地转向）：顶点 s=4.20 距 0.25m / 换向 180° > 阈值 135°
✓ PIVOT 完成：航向误差 -14.9° ≤ 15°（用时 10.7s）
★ 进入 PIVOT（原地转向）：顶点 s=8.40 距 0.25m / 换向 180° > 阈值 135°
✓ PIVOT 完成：航向误差 +14.5° ≤ 15°（用时 10.2s）
到点 42/42 N5（s=17.15 m）
阶段 DRIVE → FINISH
虚拟小车：走到第 43/43 点，位置 (2.17, 0.22)，最大横向偏差 0.875 m，积分 9966 拍，统计跟丢自愈 7 次
```

### 2.1 PIVOT 触发次数 = 折返点数 → **成立**

| 量 | 值 | 出处 |
|---|---|---|
| 规划器折返点数 | **2**（`pivot_count:2`；`F8` index 9 / `D6` index 20，均 180°） | `plan_route.py --emit json` → `plans[0]`；route_check 第 6 层 `折返点索引 [9, 20]` |
| bench 进入 PIVOT | **2**（顶点 s=4.20 = `s_at_waypoint(9)`=F8；s=8.40 = `s_at_waypoint(20)`=D6） | `★ 进入 PIVOT` × 2 |
| bench PIVOT 完成 / 超时 | **2 / 0** | `✓ PIVOT 完成` × 2；`PIVOT 超时` 0 条 |

两处顶点与 route_check 的 `折返点索引 [9,20]`、`--emit json` 的 `pivots[{F8,180°},{D6,180°}]` **三点独立互证**，PIVOT 不是多也不是少。

### 2.2 到点率 → **不成立（< 95%）**

任务卡 D 的命令口径是「`--bench` → 到点率」。bench 里唯一逐航点记录"到点"的是 MissionTask 的 `到点 i/N` 日志与兜底 `拐角绕圈兜底：航点 X（#i）` 日志；`虚拟小车：走到第 43/43 点` 和收尾的 `到点 42/42` 都是**最大索引**（累计量），不是逐点到达率。逐航点台账如下（分母 = `len(route)-1` = 42，起点不计）：

**（a）"到点"事件 31 条（`^到点 \d+/42`）：**

```
到点 1/42 N9 / 2 M9 / 3 L9 / 4 K9 / 5 I9 / 6 I10 / 7 H10 / 8 F10 / 11 F10 / 9 F8 / 10 F9 / 12 F11
13 E11 / 14 E10 / 15 C10 / 16 C9 / 17 C8 / 18 C7 / 19 C6 / 21 C6 / 20 D6 / 27 C2 / 28 D2 / 29 E2
30 F2 / 31 G2 / 34 J1 / 35 J2 / 36 J3 / 37 J4 / 42 N5
```

**（b）兜底强制跳过 4 条（`宁可漏点，不可卡死`）：**

```
⚠️ 拐角绕圈兜底：航点 B6（#22）停滞 12.0s / 累计转  94° 仍不可达，强制跳过
⚠️ 拐角绕圈兜底：航点 A6（#23）停滞 12.0s / 累计转 225° 仍不可达，强制跳过
⚠️ 拐角绕圈兜底：航点 A5（#24）停滞 12.0s / 累计转  63° 仍不可达，强制跳过
⚠️ 拐角绕圈兜底：航点 C5（#25）停滞 12.0s / 累计转  51° 仍不可达，强制跳过
```

**（c）既无"到点"、也无 forced_skip 记录的航点 7 个（静默丢点）：**

`#26 C4`、`#32 G1`、`#33 H1`、`#38 K4`、`#39 L4`、`#40 M4`、`#41 N4`

对应原始跳变：`到点 31/42 G2（s=12.95 m）` → `到点 34/42 J1（s=14.35 m）`（一步跨过 G1/H1）；`到点 37/42 J4（s=15.40 m）` → `到点 42/42 N5（s=17.15 m）`（一步跨过 K4/L4/M4/N4）——单步 s 增量 1.40 m / 1.75 m ≫ `arrival_m`(0.12 m)。

| 度量 | 计算 | 值 | 是否 ≥ 95% |
|---|---|---|---|
| 严格到点率 | 31 / 42 | **73.81%** | ✗ |
| 到点+兜底（把强制跳过当"处理过"） | 35 / 42 | **83.33%** | ✗ |
| 虚拟小车进度索引 | 43 / 43 | 100%（**不是**逐点到达率：11 个目标航点从未登记） | 口径不适用 |
| Mission 收尾计数 | 到点 42/42 | 100%（同上：索引最大值） | 口径不适用 |

**（d）与任务目标直接挂钩的账**：本图 8 颗雷中 `A6`（兜底跳过）、`C4`（静默丢）、`H1`（静默丢）**未被登记到达** ⇒ **雷到达 5/8 = 62.5%**（`I10/F8/D6/G2/E11` 登记到达）。

**（e）两次独立 bench 完全一致**（可复现，不是抖动）：

| 运行 | 到点 | 兜底 | 静默丢 | PIVOT | 最大横偏 | FINISH |
|---|---|---|---|---|---|---|
| run1 `--seconds 240` | 31 | 4 | 7 | 2 | 0.875 m | 是（202.9 s） |
| run2 `--seconds 300` | 31 | 4 | 7 | 2 | 0.874 m | 是（199.1 s） |

**（f）归因对照实验（关掉自动规划，走 `route.tokens` 回退路线；临时配置写在 `backup/cardD/verify2/robot_noauto.yaml`，`config/` 未动）**

```powershell
python -m dzbrain --config backup\cardD\verify2\robot_noauto.yaml --bench --drive --seconds 300   # exit 0，170.3 s
```

```
到达日志条数=22；兜底跳过=8；分母=33      → 严格 22/33 = 66.67%；含兜底 30/33 = 90.91%
兜底: G10(#9) 19.7s / F8(#15) 12.0s / B8(#16) / D6(#20) / C6(#21) / H1(#26) / I1(#27) / J4(#29)
PIVOT 进入=2 完成=2（几何折返点 3 处）
虚拟小车：走到第 33/34 点 … 最大横向偏差 0.485 m
```

⇒ **回退路线更差**（还跳过了打靶停站点 `G10*` 与 3 颗雷 F8/D6/H1）。结论：**「≥95% 到点率」在本工作区当前状态下对两条路线都不成立**，根因在到达判定 + bench 投影链（见 finding D1），**不是**规划器产出的路线几何（route_check 六层全过，见第 3 步）。

**（g）噪声计数（同一跑动）**：`段位移不可信` 84 条、`投影跟丢/卡死自愈` 10 次、`漂移告警` 24 条、`虚拟小车统计跟丢自愈` 7 次；典型行：

```
⚠️ 段位移不可信：本窗口 ds=8.750 m 远超位姿位移 0.009 m（上限 0.050 m = 位移×2 与 0.050 m 取大）…
⚠️ 投影卡死自愈：s 冻在 8.260（第 19 段，离线 0.34 m），车已在动 → 全路线重投影得 10.500（第 24 段，离线 0.20 m，+2.240 m，累计自愈 7 次）
```

- **判定：不通过。** PIVOT 判据成立（2 = 2），**到点率判据不成立**（73.81% / 83.33% < 95%，且有 3/8 颗雷未登记到达）。按任务卡 D 的「不要改判据、不要放宽」，本任务判 **failed**。

---

## 3. 第 3 步：路线校验 `tools/route_check.py`

**★ 命令纠正（记录，不照抄笔误）**：需求原文的 `python tools/route_check.py --config config/map.yaml` 是笔误 —— `--config` 期望的是 **robot.yaml**（含 `mission_blocks`/路线与 pursuit 参数），且本工具还需要 `--route`。照抄原文实测：`exit 2 / ❌ 未指定路线：用 --route / --from-planner…`（`--from-planner` 另需 `--start/--end`，走的是旧 `global_planner`）。故采用：

```powershell
python tools\route_check.py --config config\robot.yaml --route "<第 1 步产出的 43 token>"
```

**结果：exit 0**（wall 0.118 s，stdout 2539 B）

```
地图: 现场布置图 2026-10-07 (识图标定)   (D:\User\Desktop\dzrobot_v2\config\map.yaml)
共 43 个 token
【1/6 语法层】 ⚠️ 无法导入 path_executor（ModuleNotFoundError），改用内置副本校验
              ✅ 全部 token 合法      普通 43 | 倒车 0 | 射击 1 | 折返 2
【2/6 解析层】 ✅ 解析出 43 个航点      射击点索引 [7] / 倒车段索引 [] / 折返点索引 [9, 20]
【3/6 几何层】 ✅ 地图边界 全部 43 点在 [0.17,3.67]x[0.17,4.72] 内
              ✅ 挡板穿越 1 次 均在 C 列，最小余量 0.350 m
              ✅ 障碍最近距离 0.350 m（航段#8 距 障G9）> 0.30 m
              ⚠️ 急转弯 20 处 最大 180° @航点#9（其中 2 处已声明 :PIVOT）
【4/6 任务层】 ✅ 桥洞水平直穿 4 次      K 列 段#3(行9) 段#4(行9) 段#37(行4) 段#38(行4)
              ✅ 雷覆盖 8/8
              ✅ 靶覆盖 2/2（射击点可达）  G11<-射击点#7(0.49m) H11<-射击点#7(0.35m)
              ✅ 射击点距离     1 个射击点都能服务于某个靶（0.20~0.45 m 内）
【5/6 安全层】  总里程 17.15 m（42 航段）/ 越界保护 MAP_SAFETY_MARGIN = 0.60 m
              ✅ 桥洞保护点覆盖      K4 K9
【6/6 折返层】 ✅ 折返点声明一致 2 处    #9 180° @F8 #20 180° @D6
⚠️  全部通过，但有 1 项提醒  · 急转弯 20 处
```

**另跑 `python tools\plan_route.py --check`（内部子进程调 route_check）→ exit 0**

```
路线[K4] 43 token / 17.15 m | 穿洞 下=K9 上=K4 | 射击停站 H10 到靶 0.350 m（H11）
  A* 代价 114.10 / 展开 7672 次 / 耗时 13.5 ms | 倒车段 0（折返改写 0） | 无倒车模式 是
  折返点 2 处（:PIVOT 声明）：F8 180° D6 180° | 顺序代价 110.40（折返惩罚 0.0/处）
  route_check: ✅ 0 error
✅ 规划 + 出车前校验全部通过（route_check 0 error）
```

**再跑 `python tools\plan_route.py --emit json --quiet`** → `plans[0]`：`token_count 43`、`reverse_segments 0`、`allow_reverse false`、`pivot_count 2`、`pivots [{9,F8,180°},{20,D6,180°}]`、`polylength_m 17.15`、`elapsed_ms 14.02`。

- **判定：通过。** 六层全过（【1/6】…【6/6】无 ❌）；**error = 0**；warn = **1 项**（`急转弯 20 处`；另有 1 条 `⚠️ 无法导入 path_executor → 内置等价副本校验` 的降级提示，非路线缺陷）；**`R:` 段 = 0**（语法层「倒车 0」、解析层「倒车段索引 []」、规划器「倒车段 0」三处一致）。

---

## 4. 第 4 步：一致性对账

### 4.1 本机 5 个独立进程 × 2 条入口 → **逐字节一致**

驱动脚本 `backup/cardD/verify2/run_steps.py`（每次 `subprocess` 新进程，捕获 stdout **字节**后算 sha256）：

| PYTHONHASHSEED | `tools/plan_route.py --emit tokens --quiet` exit / 字节 / sha256 | `python -m dzbrain --plan-only`（token 行）exit / 字节 / sha256 |
|---|---|---|
| 0 | 0 / 154 B / `e244ed8ca10e8744ff1743f6…` | 0 / 153 B / `d9fef04a313e04e26f3582e1…` |
| 1 | 0 / 154 B / `e244ed8ca10e8744ff1743f6…` | 0 / 153 B / `d9fef04a313e04e26f3582e1…` |
| 2 | 0 / 154 B / `e244ed8ca10e8744ff1743f6…` | 0 / 153 B / `d9fef04a313e04e26f3582e1…` |
| 42 | 0 / 154 B / `e244ed8ca10e8744ff1743f6…` | 0 / 153 B / `d9fef04a313e04e26f3582e1…` |
| 未设置 | 0 / 154 B / `e244ed8ca10e8744ff1743f6…` | 0 / 153 B / `d9fef04a313e04e26f3582e1…` |

```
cli 去重哈希集合大小=1        → 逐字节一致 = True
plan-only token 行集合大小=1  → 逐字节一致 = True
```

**交叉对拍（4 条来源同一串，`logs/token_crosscheck.json`）**：`--plan-only` 打印的 token 行 == `plan_route.py --emit tokens` 的 stdout（`json_joined_equals_cli=True`）== bench run1 的 `自动规划 token 列表` == bench run2 的同名行；四者 sha256 均为 `9aedac96…`，43 token。
（`--plan-only` 整份 stdout 的 sha256 三次不同，仅因打印了不同的 `规划耗时：x ms`；token 行本身一致。）

> 注：sha256 是全串（含 `,x` 后缀与无换行 153 B）的哈希；`plan_route.py --emit tokens` 多一个 LF（154 B）。

### 4.2 车 / 无人机对账 → **BLOCKED（网络不可达，照实标注）**

原始取证：`backup/cardD/verify2/logs/net_probe.txt`

```
本机 WLAN IPv4 = 10.187.31.14 / 255.255.255.0（网关 10.187.31.227）
   ★ 与任务背景里的 10.100.40.14/24 不同 —— 本机地址已被 DHCP 换过，跨机对账前提不成立。

ping 10.226.211.188（车）     : Request timed out. ×2 → Sent=2 Received=0 Lost=2 (100% loss)，ping_exit=1
ping 10.240.174.104（无人机） : Request timed out. ×2 → Sent=2 Received=0 Lost=2 (100% loss)，ping_exit=1

ssh -o ConnectTimeout=6 duzhong@10.226.211.188 "echo ok"
  → ssh: connect to host 10.226.211.188 port 22: Connection timed out   ssh_exit=255 elapsed_s=6.1
ssh -o ConnectTimeout=6 root@10.240.174.104 "echo ok"
  → ssh: connect to host 10.240.174.104 port 22: Connection timed out   ssh_exit=255 elapsed_s=6.0
```

**未执行**：车端/无人机端临时目录运行、车端对账 —— 均因不可达 **BLOCKED**；**没有**以任何形式伪造车端结果，也未部署/改车（本来就够不着）。

- **判定：本机跨进程部分通过（≥3 个独立进程、含 seed 0/1/默认，逐字节一致）；跨机部分 BLOCKED 并附原始超时输出**（契约允许的处理方式）。

---

## 5. 交付物、只读纪律、回归

- **报告**：本文件 `src/history/log/log_20261011_0819_auto_planner.md`（命名匹配 `log_YYYYMMDD_HHMM_auto_planner.md`）。
- **原始证据目录**：`backup/cardD/verify2/`（`run_steps.py`、`analyze_bench.py`、`crosscheck_tokens.py`、`probe_map_override.py`、`robot_noauto.yaml`、`map_variant.yaml`、`logs/`）。
- **回归（契约 verify 第 4 条，实跑）**：

```powershell
Get-ChildItem src\dzbrain\tests\test_*.py | ForEach-Object { python $_.FullName; if ($LASTEXITCODE -ne 0) { throw "SUITE FAIL: $($_.Name)" } }
```

```
21/21 套件 exit 0（无一家失败），断言合计 1748
test_aim 122 | test_autoplan 88 | test_contract 56 | test_core 50 | test_deploy 76 | test_drone 66 | test_fix 74
test_geometry 38 | test_localization 89 | test_mission 97 | test_path_overlap 32 | test_pivot 41 | test_planner 83
test_pursuit 100 | test_referee 81 | test_route 154 | test_safety 162 | test_sim 82 | test_sim_guard 104
test_telemetry 79 | test_tracker 74
```

- **本报告本身不在 `git status --porcelain` 里**：`.gitignore:4` 的 `log/` 规则命中 `src/history/log/log_…_auto_planner.md`（`git check-ignore -v` 实测 `.gitignore:4:log/`），`backup/` 亦被 `.gitignore:8` 忽略 ⇒ 本任务新增文件对 git 全部不可见（证据是文件本身：`src/history/log/log_20261011_0819_auto_planner.md` 与 `backup/cardD/verify2/`；本文件自身的字节数/行数/sha256 请用 `Get-Item` / `Get-FileHash` 现查——写入本行会导致自指哈希失效，故不内嵌）。
- **只读纪律（`git status --porcelain` + 哈希双重证据）**：`src/`、`tools/`、`config/` 下**没有任何文件在本批窗口（08:05:54 之后）被改动**（`Get-ChildItem src,tools,config -Recurse -File | Where-Object LastWriteTime -gt 08:05:54` 结果为空），10 个受关注文件头尾 sha256 全一致。工作区里的 ` M` 条目（`config/robot.yaml`、`main.py`、`model.py`、`parser.py`、`planner.py`、`mission.py`、`test_planner.py`、`test_route.py`、`tools/plan_route.py`、`tools/route_check.py`）与 `?? src/dzbrain/dzbrain/__main__.py`、`?? src/dzbrain/tests/test_autoplan.py` 全部是**任务卡 A/B/C 的既有未提交改动**，与本次验证无关（哈希可证）。未 `git commit` / 未 `push` / 未部署。
- **每条数字都对应一条已执行命令**；未执行项只出现在下方「BLOCKED / 未执行」。

### BLOCKED / 未执行清单（禁止当作通过）

1. **车端运行**：`python -m dzbrain --plan-only` 在 Orange Pi 上跑（第 1 步只在 PC 上跑）—— 车不可达 → BLOCKED。
2. **车/无人机跨机 token 对账** —— 网络不可达，见 4.2 → BLOCKED。
3. **规划失败 → `route.tokens` 回退链路的端到端断言**（契约目标里提到，但验收 4 步未列）—— 只做了对照跑（2f），**未**用坏地图触发回退并断言 warn+出发 → 未执行。
4. **`planner.max_seconds` 超时 → 回退** 的实测 → 未执行。
5. **真车物理结论**（压雷/打靶/是否真能在 A6/C4/H1 停住）—— 未执行（无车）。

---

## 6. Findings（只报不修；每条含 文件:行 + 复现 + 期望/实际 + 建议）

### D1 — blocker（直接导致第 2 步判据不成立）：42 个目标航点里 11 个未被登记到达，其中 3 颗雷

- **文件:行**
  - `src/dzbrain/dzbrain/tasks/mission.py:712-728`：到达判据 `reached = (s_reached and near) or dist ≤ arrival_m`，其中 `near = (lat ≤ 0.30) or dist ≤ 2×arrival_m`；**s 一步跨过航点时（`over` ≫ `arrival_m`）该点被静默丢弃**，既不置位也不记账（代码注释 700 行自称"这既不诚实（没有 forced_skip 记录）"，实测正是如此）。
  - `src/dzbrain/dzbrain/tasks/mission.py:746-813`：兜底看门狗在 12 s 停滞/绕圈后把"下一个未到达航点"强制跳过（本次 4 次，含 1 颗雷 A6）。
- **复现**：`python -m dzbrain --bench --drive --seconds 240`（exit 0）→ `logs/step2_bench_drive240.txt`
- **期望**：42 个目标航点中 ≥ 40（95%）登记到达，8 颗雷全部登记到达。
- **实际**：到点事件 **31**；兜底强制跳过 **4**（`B6#22`、`A6#23`、`A5#24`、`C5#25`，皆 `停滞 12.0s`）；**静默丢 7**（`C4#26`、`G1#32`、`H1#33`、`K4#38`、`L4#39`、`M4#40`、`N4#41`）。到点率 **31/42 = 73.81%**（含兜底 83.33%）< 95%；**雷到达 5/8**（`A6`/`C4`/`H1` 未登记）。两次独立跑数值完全一致（run2 亦 31/4/7）。
- **建议修法**（择一/组合）
  1. 对被 s 跨过而**未能**满足 `near` 的航点，补一条显式记录（`forced_skip` 或新增 `EV_WAYPOINT_SKIPPED`）+ 计数器，别让丢点无声（这同时修掉"47 个兜底阈值/计数不可审计"的问题）。
  2. 到达判据在"刚越过窗口"内补一次**几何复核**（`dist ≤ max(格半宽 0.175, arrival_m)` 或投影横向偏差 `lat ≤ pursuit.arrival_m`），使"正常切角被 s 跳变跨过"不再等于丢点。
  3. 先修 s 的来源：本次 `段位移不可信 84 条 / 投影自愈 10 次 / 最大横偏 0.875 m`，s 在路线**自折叠处**（F8、D6 折返；F10、C6 重访）出现 1.4–8.75 m 的级跳（`P_ROUTE_S` 有 Localization/Tracker/Mission 三个写入者，见 `fix.py:349-366` 的自述），这是静默丢点的同一根因；不动这条，95% 判据在 bench 里无法成立。
  4. 回退路线同样不达标（66.67% / 8 次兜底 / 打靶停站点 `G10*` 被跳过）⇒ 该判据当前**不可达**，建议队长把 D 卡的第 2 步判据改为可分辨的口径（例如"雷登记到达 ≥ 8/8 且 兜底 ≤ 0"，或先修投影链再谈 95%）。

### D2 — medium：CLI `--map` 对 `--plan-only` 的规划地图**无效**（静默忽略）

- **文件:行**：`src/dzbrain/dzbrain/main.py:446-450`（`return run_plan_only(cfg)` 未转发 `args.map_path`）；`src/dzbrain/dzbrain/tasks/mission.py:176-194`（`plan_route_from_config(cfg, *, map_path=None)` 已支持覆盖，只是没人传）。
- **复现**：`python -m dzbrain --plan-only --map backup\cardD\verify2\map_variant.yaml`
- **期望**：规划按 `--map` 指定的地图；或至少打印"`--map` 只影响门禁，不影响 `planner.map_path`"。
- **实际**：打印 `地图: D:\User\Desktop\dzrobot_v2\config\map.yaml`（仍是配置里的图），token 与默认图逐字节相同；**而门禁却按变体图报错** `❌ [R005] 雷覆盖 7/8，漏 B5`（同一命令里两套地图口径并存，容易误导"换地图即出路线"的使用者）。对照：直接 `plan_route_from_config(cfg, map_path=变体)` 得到**不同**的 38 token 无 `:PIVOT` 路线（`logs/map_override_probe.json`，`same_tokens=false`）。
- **建议修法**：`run_plan_only(cfg, map_path=args.map_path)`（一行），或在不一致时显式 warn。

### D3 — low：`--plan-only` 的门禁提示把"手写路线"的错误与"规划结果"混在一起打印

- **文件:行**：`src/dzbrain/dzbrain/main.py:446-450`。
- **复现**：同上（变体图）→ 输出先 `!! 启动门禁未通过 ❌ [R005] 雷覆盖 7/8，漏 B5`，紧接着仍打印 `静态规划结果…token 数 43`，**两次重复打印同一段门禁错误**，且读者难以判断该 error 是针对 `route.tokens` 还是规划结果（实际是 `route.tokens`）。
- **期望**：错误来源标注清楚、且不重复两遍。
- **实际**：报文重复 2 次、无来源标注。
- **建议修法**：在 banner 里写明"（针对 route.tokens，不影响下面的规划结果）"，并把重复打印合并为一次。

### D4 — info：回退路线（`auto_plan_enabled=false`）的 3 处 180° 折返只触发 2 次 PIVOT

- **文件:行**：`config/robot.yaml` 的 `route.tokens`（手写回退路线，34 点）；配合 `src/dzbrain/dzbrain/tasks/tracker.py:517-540` 的 PIVOT 触发条件（`angle ≥ pursuit.pivot_angle_deg=135°` 且 `dist_to_vertex ≤ pivot_dist_m`）。
- **复现**：`python -m dzbrain --config backup\cardD\verify2\robot_noauto.yaml --bench --drive --seconds 300`
- **期望**：若回退路线确有 3 处前进-前进 180° 折返（任务卡 C 报告 §11 指出 `E11/A6/D6`），PIVOT 应为 3 次（或该点被显式声明/记录）。
- **实际**：`★ 进入 PIVOT` 2 次（s=4.20、s=8.05）、`✓ PIVOT 完成` 2 次；同时 `D6(#20)` 与 `C6(#21)` 被兜底跳过，`G10*` 打靶停站点也被兜底跳过（`停滞 19.7s`）。
- **影响**：现场默认 `planner.auto_plan_enabled=true`（本次验证的主路径），风险只在"规划失败回退到手写路线"时出现。
- **建议修法**：给回退路线补 `:PIVOT` 声明（与 planner 同规则：`corner_turn_deg ≥ 135` 处），并让回退分支在启动横幅里打印"回退路线未经 PIVOT 校验"。

---

## 7. 一句话结论

**第 1、3、4 步通过**（`--plan-only` 13.6–15.0 ms ≪ 100 ms 且随地图变化；route_check 六层全过 / 0 error / 0 段 `R:`；本机 5 进程 token 逐字节一致 = `9aedac96…`；跨机 BLOCKED 有 ping/ssh 原始超时）。**第 2 步不通过**：PIVOT 触发 2 次 = 折返点 2 处（成立），但**到点率 31/42 = 73.81%（含兜底 83.33%）< 95%**，11 个目标航点未登记到达（4 个兜底跳过 + 7 个静默丢弃），其中 3 颗雷（`A6/C4/H1`）⇒ 按"不放宽判据"的要求，本任务判 **failed**，缺陷见 finding **D1**（另附 D2/D3/D4）。
