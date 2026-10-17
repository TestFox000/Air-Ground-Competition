# L4 · 无人机侧与空地链路只读审计报告（复核轮 · HEAD 0eb3570 之后）

- 审计员：`aud-drone`（团队 air-ground-v4）
- 任务：t4 [L4] — 无人机侧与空地链路只读审计（Attempt 1 / `10cca402-7e54-438c-8d1e-f551815b58d6`）
- 模式：**全程只读**。未连车、未连无人机、未重启服务、未占串口、无 git 写操作、无代码/配置改动。唯一写入物：本文件。
- 审计根：`D:\User\Desktop\比赛存档\v1\`
- 上一轮（air-ground-v3 同卡）报告即本文位置，500 行/49 KB。本卡为**复核轮**：验证其结论在车侧 HEAD 0eb3570 → 现 e40b9a1 之后是否仍成立，并登记其间新增漂移。
- 复核范围：`dzsmartscout`（结构/脚本残留/44 token 契约）、车侧 `dzrobot_v2` 工作区未提交改动对契约的影响（只读）。

---

## 0. 结论卡（先说答案）

| # | 结论 | 判定 |
|---|---|---|
| 1 | `dzsmartscout` 完全不在版本控制里 | **确认（未变）** |
| 2 | 体积清点：**2 091 文件 / 564 目录 / 424.0 MB**（较 v3 审计 +11.4 KB，全部来自 `__pycache__`/嵌套 `.git/index` 的 mtime 刷新，无新增内容文件） | **确认（未变）** |
| 3 | 根目录一次性取证脚本：**29 个散文件全部仍在**（crack_vnc_passwd.py / fix_drone_avahi.py 等），零清理 | **确认（未变，应清理）** |
| 4 | 无人机侧对 **44 token 路线** 的对账：vendor 镜像 + 4 处引用 + EVIDENCE 全部停在**旧路线**（sha `ae82370e…`） | **确认（未变）** |
| 5 | 但车侧真源已**静默换线**：工作区未提交改动（卡 t6，2026-10-16）把 `mission_blocks`/`route.tokens` 改成 44 token 新路线（`L10→M9`、`J6→I5`，sha `886d0738…`） | **⚠️ 新漂移（本次复核核心新增）** |
| 6 | vendor 镜像漂移从 **3 文件 → 4 文件**（新增 `route/planner.py`） | **确认（恶化）** |
| 7 | 坐标契约/报文/QoS/域号 118 / 语法缺口（F-02）/ K10 兜底（F-04）/ 死键（F-06）/ docs 时效 **全部与 v3 一致，未变** | **确认（未变）** |
| 8 | 新发现：车侧 `test_corner_cut.py` 在当前工作区 **30/34 失败**（t6 改线后 4 条断言未同步） | **⚠️ 新发现（车侧，阻断级待修）** |
| 9 | 新发现：无人机侧 `sim_airground` 测试套件对当前车侧真源**硬崩溃**（TypeError @ L173）——v3 审计 F-05 的"no-op"定性过轻，实为 exit 1 崩溃 | **⚠️ 新确认（严重度修正）** |
| 10 | 明文口令/root 直连/`AutoAddPolicy` 面 **未变**（P0 安全项仍在） | **确认（未变）** |

**一句话**：v3 审计的 17 项发现里 15 项原样成立；**本轮新增的实质变化只有一件——车侧真源路线在未提交状态下被卡 t6 换掉（44 token 不变、token 变了），而无人机侧镜像/EVIDENCE/契约文档全部停在旧线上，且车侧测试与机侧仿真测试都已对不上新路线**。

---

## 1. 必查项① · `dzsmartscout` 结构清点（复核）

### 1.1 体量（全量递归实测）

| 指标 | v3 审计 | 本轮实测 | 变化 |
|---|---|---|---|
| 文件总数 | 2 091 | **2 091** | 无 |
| 目录总数 | 564 | **564** | 无 |
| 总体积 | 444 606 278 B | 444 617 727 B | +11 449 B（0.003%） |

**+11.4 KB 去向**：全部来自 `tools/planner/vendor/**/__pycache__/*.pyc` 与 7 个嵌套 `.git/index` 的 mtime/体积刷新（编译产物 + git 索引），**无任何新内容文件**。内容层面的 `dzsmartscout` 与 v3 审计时逐字节等价（以本轮对账为准，见 §3）。

### 1.2 版本控制（复核）

```
$ git -C dzsmartscout rev-parse --show-toplevel
fatal: not a git repository            ← 未变
$ git -C dzrobot_v2 log -1 --format="%h %ad %s" --date=iso
e40b9a1 2026-10-17 08:28:07 +0800 v3.8.1_台账补登…   ← 车侧 HEAD（v3 审计时为 0eb3570）
```

- `dzsmartscout` 仍不在任何 git 仓库内；8 个嵌套第三方仓库（mavros/px4_msgs/px4_ros_com/livox_ros_driver2/FASTLIO2_ROS2/FAST_LIO_LOCALIZATION_ROS2 + fc-audit/src-mirror 两个）的本地改动仍为裸文件（`.git/index` mtime 被刷新但内容未变）。
- v3 审计的全部风险结论（单副本存活、误删不可恢复、回滚无锚点、336 MB 嵌套 `.git` 制造"在管"错觉）**原样成立**。

---

## 2. 必查项② · 根目录一次性取证脚本清点（复核）

根目录散文件 **29 个，全部在，零清理**（v3 审计记 27，差 2 为当时统计口径：`icon_preview.{html,png}` 计入与不计入的差别）。全部 mtime ≤ 2026-10-07，与 v3 审计清单逐项一致：

| 类别 | 文件 | 性质 |
|---|---|---|
| 接入期一次性脚本 | `ssh_connect.py`、`probe_drone_vnc.py`、`probe_drone_dirs.py`、`enable_mdns.py`、`crack_vnc_passwd.py`、`install_drone_key.py`、`fix_drone_avahi.py`、`fix_mdns_nsswitch.py`、`rename_drone_hostname.py`、`run_fix_drone_avahi.py`、`remote_fix_drone_avahi.sh`、`setup_drone_access.py`、`verify_new_ip.py`、`test_drone_users.py`、`find_drone.py`、`sweep_current.py`、`ascii_icon_check.py`、`adapt_drone_plugin.mjs`、`asar_list.js` | 含明文口令/root 直连/改机上系统配置（v3 §4 逐行登记） |
| 中间产物 | `ar5_p7_8.txt`、`qsg_p4_6.txt`、`lidar_manual_extract.txt` | 一次性取证/问答记录 |
| 0 字节垃圾 | `err.txt`、`out.txt`、`_pdf_log.txt` | 无价值 |
| 工程文档 | `WORKFLOW.md`（10-07，最旧） | 含被证伪声明（v3 §6.2） |
| 图片 | `icon_preview.{html,png}` | 一次性探针产物 |

**应清理判定（只登记，本卡不改任何文件）**：
- **必须清理（安全 P0）**：`crack_vnc_passwd.py`（内置口令字典 + `echo PW|sudo -S`）、`fix_drone_avahi.py`/`fix_mdns_nsswitch.py`/`rename_drone_hostname.py`/`run_fix_drone_avahi.py`（sudo 改机上 `/etc/*` 并 restart，违反 `WORKFLOW.md:9`「SSH 只读」纪律）、`install_drone_key.py`、`setup_drone_access.py`、`verify_new_ip.py`、`test_drone_users.py`——这批脚本含明文口令 + root 直连 + `AutoAddPolicy`，v3 已记 F-12/F-15，**至今未处置**。
- **可清理（卫生）**：0 字节三件、中间 txt 三件、`icon_preview.*`。
- **保留**：`WORKFLOW.md`（纪律层仍有价值，但需按 v3 §6.2 修订其过期声明）。
- **结论**：v3 的"应清理"判定**成立且更紧迫**——`dzsmartscout` 无版本控制，这些脚本一旦随目录被分享/提交即刻外泄凭据（v3 F-12 的 P0 理由不变）。

---

## 3. 必查项③ · 44 token 路线契约依赖（t60 对账契约）复核

### 3.1 车侧真源发生了一次**未提交的换线**（v3 之后新增漂移）

车侧 `config/robot.yaml` 工作区（未提交）在 2026-10-16 由卡 t6 改线：

| | v3 审计时（与 HEAD 一致） | 本轮实测（工作区未提交状态） |
|---|---|---|
| `mission_blocks` | 44 项 / sha `ae82370e…` | 44 项 / sha **`886d07382758af12`** |
| token 变化 | `N10 M10 **L10** L9 K9 … H6 I6 **J6** J5 K5 …` | `N10 M10 **M9** L9 K9 … H6 I6 **I5** J5 K5 …` |
| 其余 | 44 token / 0 折返 / `F3*` 停站 / K9+K5 直穿 / 雷 8/8 不变 | **同**（t6 注释明写"同长度同转角，只是拐弯顶点前移 1 格"） |

关键事实：
- `route.tokens` 是 `mission_blocks` 的**别名**（`core/config.py:228` Spec + `:543` `["path_executor.mission_blocks"]`）⇒ 两者始终相等，本次实测工作区 `route.tokens == mission_blocks == 44 token`（sha `886d0738`）✅ **车侧自身内部一致**。
- HEAD（提交态 e40b9a1）仍是旧线（`L10`/`J6`，`ae82370e`）——**换线只在工作区，未提交**。
- 换线动因（robot.yaml:434-444 注释）：桥洞进洞前对正，两处"洞口前 1 格 90° 拐弯"（L10、J6）前移 1 格（M9、I5），消除进洞瞬间车身斜置（t5 实测摆正误差 38~52°、最大穿透 56.7 mm）。
- **无人机侧对此零感知**：`sync_planner_from_source.py` 的哈希账本钉的是**文件哈希**，不比对 `mission_blocks` 内容（v3 §2.4 结构性洞①原样成立）；而 `--verify-only` 自 2026-10-16 19:28 之后**从未再跑**（`EVIDENCE/sync_log.txt` 最后写入 19:28:11，账本 `HASH_LEDGER.json` 生成于 19:28:06）。

### 3.2 无人机侧 4 处引用 + vendor 镜像现状（全部停在旧线）

实测对拍（Python，逐 token + join 后 SHA256）：

| 位置 | 内容 | 与车侧 HEAD（旧线） | 与车侧工作区（新线 886d0738） |
|---|---|---|---|
| `docs/空地路线下发契约_v2.md:82` | 44 token 整线 | ✅ 一致 | ❌ 过期 |
| `tools/planner/EVIDENCE/plan_tokens.txt:7/10` | 双侧对拍两行 | ✅ 一致 | ❌ 过期（且自述 robot.yaml sha `58298dc2…` 的快照也已过期） |
| `tools/planner/EVIDENCE/{plan_compare.json,route_check_source.txt,route_check_vendor.txt,dryrun_full_route.txt}` | 同一串 | ✅ 一致 | ❌ 过期 |
| `tools/planner/README.md:62/65` | 同一串 + 「逐 token 相同 ✅」 | ✅ 一致 | ❌ 过期（**该 ✅ 现已不成立**） |
| `tools/planner/vendor/dzbrain_ws/config/robot.yaml` | `mission_blocks` | ✅ 一致 | ❌ 过期 |
| `tools/planner/vendor/dzbrain_ws/**`（planner 等 12 个文件） | 同步副本 | ✅ 一致 | ❌ 过期（见 §3.3） |

**vendor 自洽性复核**（本轮实跑 vendor planner）：

```
VENDOR planner output   n=44 sha=ae82370e51ee9e1f   ← 旧线
VENDOR mission_blocks   n=44 sha=ae82370e51ee9e1f   ← 旧线
self-consistent: True
```

⇒ t60 证据里"机算 44 token == mission_blocks 逐 token 相同"在**本机自洽口径**下仍然成立（vendor 自己的 planner 产出 == vendor 自己的 mission_blocks）；但它**不等于车侧现役真源**了。这正是 v3 §2.4 结构性洞①的实例化：**"对账"是文件哈希账本，不是路线内容对账**——今天两侧路线已分叉，账本仍标 `all_same: true`（15 条），若直接拿它当"两侧一致"的证据就是假绿。

### 3.3 vendor 漂移：3 文件 → 4 文件（恶化）

账本（19:28）vs vendor vs live 三方 SHA256 复算（15 文件）：

| 文件 | 账本=vendor | live 现值 | 判定 |
|---|---|---|---|
| `core/config.py` | 一致 | **已变**（t53：`aim.lead_time_s` 0.35→0.0、`aim.fire_box_far_px/near_px` 3.5→14.0/26.0） | **漂移** |
| `route/physics.py` | 一致 | **已变**（v3.8 提交 0eb3570：桥洞进洞侧冗余墙包含检测，188 行） | **漂移** |
| `route/planner.py` | 一致 | **已变（新增漂移）**（0eb3570 的 49 行 + 工作区 t6 桥洞对正惩罚 `BRIDGE_APPROACH_ALIGN_PENALTY_M` + t13 负值钳位，vendor→live +148/−6） | **漂移** |
| `config/robot.yaml` | 一致 | **已变**（t6 换线 + t53 aim 键 + confirm 帧数 4→2/5→2） | **漂移** |
| 其余 11 个文件 | 一致 | 一致 | ✅ |

⇒ **drift 数 = 4（v3 时为 3）**。vendor 侧 15 个文件 hash 与账本**完全一致**（无任何部分同步），即漂移纯粹是"live 往前走了、vendor 没跟"，纪律（发车前 `--verify-only`）自 19:28 起未执行。

### 3.4 换线对机侧测试/仿真的连带冲击（本轮新实证）

| 测试 | 依赖 | 当前结果 | 性质 |
|---|---|---|---|
| 车侧 `src/dzbrain/tests/test_corner_cut.py` | 冻结 `mission_blocks` + 旧切点集合 `L10/J10/E10/B9/G6` + 旧 sim 峰值 | **30/34，4 失败**（3.1 切点变 `M10`、3.3 删点集合、5.1/5.2 sim 拐角超调**反而上升** 0.044→0.064 m、0.116→0.140 m） | 车侧新回归（t6 换线未同步测试，未提交） |
| 机侧 `tools/sim_airground/tests/test_airground.py` | `_rewrite_triple(baseline,"K10","K9")`（依赖基线含 K10） | **exit 1 硬崩溃**：TypeError @ L173（内层 `_rewrite_triple` 返回 None → 外层 `enumerate(None)`） | 机侧仿真套件不可用 |

**F-05 严重度修正**：v3 审计把 `test_airground.py:115` 定性为"no-op（注入 K9 而 cfg 已 K9）"，过轻。实测是：基线（`route.tokens`，无论新旧 44 token 线均不含 K10）里找不到 K10 ⇒ `_rewrite_triple` 返回 None ⇒ 第 5 节断言失败 + **第 10 节 L173 无条件崩溃**。该崩溃在 v3 审计时即已存在（当时基线同样无 K10），本轮复核确认并如实升级为"测试套件不可用"。**根因仍是 v3 登记的同一设计冲突**（夹具默认=注入值 ⇒ 换洞成空操作），但表现形式应记为"exit 1 崩溃"。

**新增风险**：车侧 `test_corner_cut.py` 30/34 失败意味着**车侧工作区当前是一个"路线已换、测试未跟"的中间态**——若有人现在按"全量回归绿"验收工作区会拿到假绿（该文件不在本轮回归口径，但 v3 后任何车侧回归都会暴露）。

---

## 4. v3 结论逐条复核表

| v3 条目 | v3 判定 | 本轮复核 | 结论 |
|---|---|---|---|
| F-01 vendor 漂移 | 阻断（3 文件） | 4 文件（+planner.py）；账本/vendor/live 三方复算 | **成立且恶化** |
| F-02 token 语法不等价（机侧拒 `:PIVOT`/`:SCAN`/`R:`） | 高 | `supervisor.py:559` RE_TAIL 正则未变；本轮正则实测 `K9:PIVOT→False`、`F2:SCAN→False`、`F2:SCAN_STOP→True`、`G10*→True`、`R:B5*→False`（车侧 parser 均 OK） | **成立（未变）** |
| F-03 感知话题通道死代码 | 高 | `supervisor.py:1280-1286` AST 复核：`ingest_sample` 仍在 `if obj is None: return` 之后，不可达 | **成立（未变）** |
| F-04 兜底格 K10 vs 车侧 K9 | P0 口径 | `supervisor.py:112/122` 仍硬编码 `bridge_default_lower="K10"`（注释仍引 `robot.yaml:520`，实际 `:530`）；`:1614` 横幅仍用 K10 | **成立（未变）** |
| F-05 sim 世界语义被换图反转 | 中 | `scenarios.py:248/260`（世界 A=K10/K5）、`:268`（B 世界 entrance≠K10）、`:295`（C 世界 K11/K1）**逐字未变**；且测试崩溃（见 §3.4） | **成立（未变）+ 严重度修正** |
| F-06 死键 `drone.heartbeat_offline_s` | 中 | `supervisor.py:121`、`link/README.md:50`、`probe/t4_contract.sh:41` 仍引用；车侧真名仍是 `drone.heartbeat_timeout_s`（`config.py:428`） | **成立（未变）** |
| F-08 车侧 drone.py 过期注释 | 低 | `tasks/drone.py:34-35` 仍写"SCHEMA 缺键：map_path/field_diff_max/ack_enabled"，而 `config.py:431-433` 三键俱在 | **成立（未变）** |
| F-07 裁判话题名前缀 | 待实机 | 无新证据 | 维持待实机 |
| F-09/F-10 协议 backlog / 双写者 | 提示 | 无变化 | 维持登记 |
| 结构性 ① 只有快照没有内容对账 | 洞 | `sync_planner_from_source.py` 仍只比文件哈希；本轮实证其"all_same:true"账本已掩盖路线分叉 | **成立（本次实例化）** |
| 结构性 ② 62 点口径两侧都错 | 洞 | `契约_v2.md:20/120/142` 仍写"手工 62 点"；车侧 `robot.yaml:596/612` 注释仍写"62 点"（v3 后未修） | **成立（未变）** |
| 结构性 ③ 10 处旧 34 点示例 | 洞 | 无任何 dzsmartscout 内容文件变化 ⇒ 10 处位置原样 | **成立（未变）** |
| 凭据/明文口令/root/AutoAddPolicy | P0 | 脚本零清理（§2），口令面逐项复存 | **成立（未变）** |
| docs 时效分档 | 见 v3 §6 | mtime 全部未变：契约_v2 10-15 20:56、perception README 10-15 20:52、planner README 10-15 20:54、总览/技术文档/link README 10-12、WORKFLOW 10-07 | **成立（未变）** |
| 坐标/报文/QoS/域号 118 两侧一致 | ✅ | grid.py:9-10、sim_airground/geo.py:7-8、车侧 geometry.py:9-10 三处公式 `(10.5−row)·0.35 / (13.5−col)·0.35` 逐字一致；`run_field_map_check.py:43` 仍读 `dzrobot_v2/config/map.yaml` 当真值（map.yaml sha `d5d7903d…` 三方仍一致） | **成立（未变）** |
| vendor 漂移含物理判据修复 | 阻断 | physics.py 漂移仍在（0eb3570 已提交进 live，vendor 未跟） | **成立（未变）** |

### v3 一处表述修正

- v3 §7-3 称 `EVIDENCE/sync_log.txt:23`「曾试图推到 10.35.165.104 但**未记录结果**」——实测 sync_log.txt:23-27 **有完整结果**（scp 完成 / 远端退出码 0 / 远端 route 输出 41 token 旧线 `N10 N9 M9 L9 K9 I9 I10 H10 G10 E10 E9 D9 C9 C7 B7 A7 A6 A5 A4 A3 A2 B2 B1 C1 D1 D3 E3 F3* G3 G4 G5 H5 H6 I6 J6 J5 K5 L5 M5 N5,y`）。**有结果，且远端产物是另一条（tour 引擎口径的）路线**——与本地 44 token 契约线不同，佐证"远端当时跑的是默认 tour 引擎"的坑（契约_v2 §3 已有记录）。

---

## 5. 本轮新增发现台账（v3 之后）

| 编号 | 严重度 | 位置 | 问题 | 建议（只登记） |
|---|---|---|---|---|
| **F-17** | **高** | 车侧工作区（未提交）+ 机侧 vendor/EVIDENCE/契约文档 | 卡 t6 换线（L10→M9、J6→I5，44 token 不变）后，无人机侧 5 处引用 + vendor 镜像全部过期；无内容级对账，`--verify-only` 自 10-16 19:28 未再跑 | 发车/换图前先重跑 `sync_planner_from_source.py` + 重建 EVIDENCE；把"mission_blocks 内容比对"补进同步账本（结构性洞①） |
| **F-18** | **高** | 车侧 `src/dzbrain/tests/test_corner_cut.py` | t6 换线后 30/34 失败（切点集合、删点集合、sim 拐角峰值未同步） | 修测试期望或冻结路线回退；提交 t6 前必须全绿 |
| **F-19** | **中高** | 机侧 `tools/sim_airground/tests/test_airground.py:113-173` | 对当前车侧真源硬崩溃（TypeError @ L173）；v3 F-05 定性过轻 | 重钉夹具（基线语义改为"从 config 读 K 列洞"或改用可注入基线） |
| **F-20** | 中 | 车侧 `robot.yaml:434-444`（t6 注释） | 新路线注释引用 `test_corner_cut.py:145` 作为"规划器产物==本表"的锚，但该测试当前 4 条失败 ⇒ 注释锚点失真 | 修测试后复核注释 |
| **F-21** | 低 | `EVIDENCE/sync_log.txt:23-27` | 远端推包产物（41 token tour 线）与本地契约线不同，无任何注释说明 | 加一行说明"远端默认引擎=tour，勿当作契约线" |

**新增正面确认**：vendor 自洽性（vendor planner 产出 == vendor mission_blocks == `ae82370e…`）实测成立；车侧工作区 route 三源（route.tokens / mission_blocks / planner 产物）自洽于新线 `886d0738…`，route_check 0 error（t6 新线过六层校验，仅急转弯提醒级 warn）。

---

## 6. 无法验证清单（更新，不猜测）

与 v3 基本一致（无人机不可达、无 ROS 运行时），本轮新增/修正：

| # | 项 | 缺什么 |
|---|---|---|
| 1-10 | 同 v3 §7-1~10（F-07 话题名前缀、DDS 实配、机侧真机规划器、yaw 基准、supervisor 真机行为、机上现状、实车闭环、AP 组播、口令有效性、sim 台账复现） | 无人机在线 + 只读 SSH；本卡禁止 |
| 11 | **t6 新路线的真车进洞行为**（M9/I5 前移是否真消除斜插） | 实车 + 现场；只有 route_check 几何层 0 error |
| 12 | **vendor 漂移在真机上的影响**（physics.py 包含检测缺失 ⇒ 机侧桥洞净空结论与车侧不等价） | 需重跑 `--verify-only` + 重建 EVIDENCE 后再谈 |

---

## 7. 发现台账（汇总 · 含 v3 原条目）

| 编号 | 严重度 | 位置 | 问题 | 阻断换图/发车？ | 状态 |
|---|---|---|---|---|---|
| F-01 | 阻断 | vendor 4 文件漂移（+planner.py） | 机侧路线合法性证明过期；token 序列不受影响 | 是（证明过期） | **未修（恶化）** |
| F-02 | 高 | supervisor RE_TAIL | `:PIVOT`/`:SCAN` 机侧拒发 | 换洞产生折返时 | 未修 |
| F-03 | 高 | supervisor:1280-1286 | 感知 topic 通道死代码，收样本不决策零告警 | 用 topic 通道时 | 未修 |
| F-04 | P0 口径 | supervisor:112/122 + 6 处文档 | 兜底格 K10 vs 真值 K9 | 误导现场决策 | 未修 |
| F-05 | 中 | scenarios.py + 测试 | 世界 A/B/C 语义反转 + 夹具崩溃 | 否 | 未修（严重度修正） |
| F-06 | 中 | supervisor:121/README:50 | 死键 heartbeat_offline_s | 否 | 未修 |
| F-07 | 待实机 | judgment.cpp:62 | 相对话题名风险 | 需实测 | 维持 |
| F-08 | 低 | drone.py:34-37 | 过期注释 | 否 | 未修 |
| F-11 | 中高 | dzsmartscout 全目录 | 无版本控制 | 阻断"能改/能回滚" | 未修 |
| F-12 | P0 安全 | §2/§4 全部 | 明文口令扩散、root 直连、AutoAddPolicy | 不阻断，安全 P0 | **未处置（零清理）** |
| F-13 | 中 | 契约_v2.md | 行号引用漂移 | 否 | 未修 |
| F-14 | 低 | WORKFLOW.md:43 | src-mirror 声明不成立 | 否 | 未修 |
| F-15 | 低 | WORKFLOW.md:9 vs 根脚本 | "SSH 只读"被违反 | 否 | 未修 |
| F-16 | 低 | 根目录垃圾文件 | 仓库卫生 | 否 | 未修 |
| F-17 | 高 | 车侧工作区 ↔ 机侧镜像 | **t6 换线后两侧路线分叉**（新增） | **是（契约假绿）** | 新增 |
| F-18 | 高 | test_corner_cut.py | t6 换线后 30/34 失败（新增） | 提交前必须修 | 新增 |
| F-19 | 中高 | test_airground.py | 对当前真源硬崩溃（严重度修正） | 机侧仿真不可用 | 确认升级 |
| F-20 | 中 | robot.yaml t6 注释 | 锚点测试失败（新增） | 否 | 新增 |
| F-21 | 低 | sync_log.txt:23-27 | 远端产物口径不明（修正 v3 表述） | 否 | 新增 |

**正面确认（供队长据此放心）**：
- vendor 镜像**自洽**（planner 产出 == mission_blocks == `ae82370e…`，本轮实跑）；车侧工作区 route 三源**自洽**于新线 `886d0738…`，route_check 0 error。
- 坐标公式/报文字段/QoS/域号 118/格名归一化行为两侧一致（本轮复核 3 处公式副本 + map.yaml 三方一致）。
- vendor 15 文件与账本**逐字节一致**（账本本身过期，但镜像本身没有被部分同步破坏）。

---

## 8. 复核命令附录（全部只读，本轮实际执行）

```powershell
cd D:\User\Desktop\比赛存档\v1

# ① 体量
(gi dzsmartscout -rec -file -fo).Count                        # 2091（不变）
(gi dzsmartscout -rec -dir -fo).Count                         # 564（不变）
(gi dzsmartscout -rec -file -fo | measure Length -Sum).Sum    # 444617727（+11449）

# ② git 边界
git -C dzsmartscout rev-parse --show-toplevel                 # fatal: not a git repository
git -C dzrobot_v2 log -1 --format="%h %ad %s" --date=iso     # e40b9a1 v3.8.1（0eb3570 → e40b9a1）

# ③ 44 token 对账
#    car live mission_blocks   n=44 sha=886d07382758af12（新线 M9/I5）
#    car HEAD  mission_blocks  = 旧线 L10/J6（ae82370e…）
#    vendor mission_blocks     n=44 sha=ae82370e51ee9e1f（旧线）
#    vendor planner 实跑产出   = mission_blocks（自洽）
#    route_check 新线          0 error / 1 warn（急转弯提醒）

# ④ vendor 漂移三方复算（15 文件）
#    账本=vendor 全同；live 漂移 4 文件：core/config.py、route/physics.py、
#    route/planner.py（新增）、config/robot.yaml

# ⑤ 语法对拍（不碰 ROS）
#    车侧 parser：K9:PIVOT OK / F2:SCAN OK
#    机侧 RE_TAIL：K9:PIVOT False / F2:SCAN False / F2:SCAN_STOP True / R:B5* False

# ⑥ 车侧回归（只读）
python dzrobot_v2\src\dzbrain\tests\test_corner_cut.py        # 30/34（4 fail：3.1/3.3/5.1/5.2）

# ⑦ 机侧仿真（只读）
python dzsmartscout\tools\sim_airground\tests\test_airground.py  # exit 1 TypeError @ L173

# ⑧ 凭据面 / 根目录（只列清单，未复现口令明文）
#    root 散文件 29 个全部在，mtime ≤ 10-07；0 字节 3 件；脚本零清理
```

**只读纪律自检**：
- 未修改任何被审计文件；未执行 git 写操作；未连车/连机；未起 ROS；未占串口。
- 本轮仅产生本文件；复核过程在 PowerShell 内联 Python 完成，无临时脚本落盘。
- 报告内未复现任何口令明文（沿用 v3 的 `<CRED>` 代称规则）。
