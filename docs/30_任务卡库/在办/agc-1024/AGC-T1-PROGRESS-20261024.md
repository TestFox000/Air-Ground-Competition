---
来源: 交接_20261024_空地协同接手 §3；CAR-PREFLIGHT §9.2；用户 2026-10-24 接手指令
取证: 档（run2-mission.log / run2-localization.log）+ 只读源码 + 本地最小复现
更新: 2026-10-24
status: in_progress
ns: agc-1024
agent: t-car
树节点: AGC-1
写域: ~~_tmp_session/agc-1024/t1/~~；本卡；~~dzrobot_v2/src/dzbrain/tests/~~ 下新增测试文件（不覆盖他人测试）
摘要: 查清 Tracker/Localization/Mission 三处写 P_ROUTE_INDEX/P_ROUTE_S 的生产-消费契约，给出唯一可判定假设与最小复现；生产文件改动须 Lead 加锁。
---

# AGC-T1-PROGRESS-20261024 · 进度生产者/消费者契约

owner: t-car；reviewer: lead（必要时 e-integ 只读复核）。

## 1 依据与复用（五必填）

- 读过的台账：D-036（preflight 边界）、D-039（情报契约三态 source，本卡只引用口径）、D-001（不整包替换）；现行推进计划 §3 T1。
- 复用检索：`Path.project` 投影、`tests/_fixtures` 假车/Rig、`test_projection_relock`、`test_overlap_no_flipflop`、定位测试；**不新建第二套投影或进度框架**。
- 新增轮子：仅允许一个最小复现脚本（驱动既有 tracker/localization/mission 状态机、FakeClock，不复制生产决策逻辑）。
- 验收命令：见 §3。
- 回滚点：本卡只读生产码；新增测试/脚本均在 `_tmp_session/agc-1024/t1/`，删除即回滚。

## 2 当前问题（已证事实，不得重开）

1. Tracker、Localization、Mission 三处都写 `P_ROUTE_INDEX`/`P_ROUTE_S`，各有自己的投影状态（已证）。
2. run2：F9 STOP 显示段11/s4.40，DRIVE 恢复即显示段6/s2.48；定位投影自愈 6→10→8；行驶中长期显示段6/s2.2~2.8（已证日志事实）。
3. 同趟车实际持续推进到 A8 附近，`drift_stop` 限停 0 拍（已证）。**因此不得把遥测旧段写成“车辆卡死”**。
4. 未证：三处竞争对控制器实际用值的影响；也与“重复航点提前记账”（T2）不是同一个缺陷。

## 3 唯一假设与验收

- 假设：三个投影状态竞争共享进度键，至少一层（显示 / Mission 任务判定 / 控制器用值）采纳了非权威值。
- 必答项（缺一不算完成）：① 谁写、什么时候写、写什么；② 控制器实际读哪个键/哪个对象；③ Mission 到点与段推进采纳谁；④ 每个数值变化能否用“线程顺序/授权 reset/换路/正常推进”解释完，无剩余。
- 离线验收（命令写进卡）：
  - 最小复现 H8 折返与 F9 STOP→DRIVE 两个场景，逐拍打印 `(writer, index, s, 用值来源)`。
  - 调整线程执行顺序后仍能解释每次进度变化（不是靠固定顺序才不抖）。
  - 授权 reset / 换路事件与正常推进在日志中可分离。
- **禁止**：直接删写入点、直接关方向守卫、为让显示好看而改常量。

## 4 交付

1. 生产者-消费者映射表（文件:函数:行 + 键 + 写时机）。
2. 影响层判定（显示/判定/控制三者选实，附证据）。
3. 最小修补候选（若确定需要改生产文件，先在卡里写“请求 Lead 加锁 mission.py/tracker.py/localization.py”，批准后同一 owner 串行实施）。
4. 复跑命令 + 输出摘要 + 回滚点。

## 4A 执行记录（2026-10-24 · t-car · 已按 §4 四条交付）

### 4A.1 唯一结论（一句话）

P_ROUTE_INDEX / P_ROUTE_S 是**一张单槽键：没有权威、没有新鲜度、没有单调性契约**——三个生产者各持一份自己的投影状态，最后写入者赢得槽位；**控制层不读它**（Tracker 每拍自算投影），真正的"段号回退"由 **MissionTask._current_s 用旧 hint 回写**造成，而且这个值会反过来拖累 Mission 自己的到点判定（E9 因此被 12 s 兜底强制跳过）。旧段遥测 = 键值换手，**不是车辆卡死**（车继续走到 A8、drift_stop 限停 0 拍，与此一致）。

### 4A.2 生产者 → 消费者映射表（真源：dzrobot_v2/src/dzbrain）

| # | 写者（文件:函数:行） | 键 | 何时写 | 写什么（自己的投影口径） |
|---|---|---|---|---|
| W1 | tasks/localization.py:LocalizationTask.tick:395-399 | 两键 | 每拍（20 Hz） | _project(cx,cy,cyaw)：allow_backward=False + window_m=arrival_m(0.12)，hint=自己上一拍；_heal_stuck 兜吸收态 |
| W2 | tasks/localization.py:LocalizationTask.tick:437-444 | 两键 | 每拍**吸附成功后第二次** | 吸附后的信念位姿再投影一次（同一拍内键被写两遍） |
| W3 | tasks/tracker.py:TrackerTask.tick:675-677 | 两键 | 每拍（50 Hz，仅在未拦停时） | path.project(pose, s_hint=_seg_s, window_m=0.60, back_m=0.15, s_min=_s_floor, motion_heading=实测位移方向) |
| W4 | tasks/mission.py:MissionTask._current_s:3051-3055 | 两键 | 每拍最多 3 次（10 Hz × 调用点 3069 / 3391 / 3748） | path.project(pose, s_hint=_last_s)（默认窗口 0.60、允许后退）；**只有当 \|缓存−自己\| > arrival_m(0.12) 才回写** |
| R1 | tasks/mission.py:_reset_belief:2806 | 两键 | reset:BLOCK,HEAD | 段-1 / s0.00（授权复位） |
| R2 | tasks/mission.py:_install_route:2911-2922 | 两键 | 路线装载 / 决策点改道 | 先 段0 / s0.00；keep_progress=True 时按快照回填（2404 记 index_before） |
| — | tasks/telemetry.py:build_telemetry:109-112 | 两键（**只读**） | 2 Hz | 直读黑板原值，**不查 source/age** → 现场打印的"段X/sY"就是最后写入者 |
| — | tasks/fix.py:_accumulate:321,352（第三方消费者） | P_ROUTE_S（只读） | 10 Hz | 把键当估计弧长做单拍 ds 守卫（core/config.py:469-485）；**同一静止窗口里键被换手时它就报"段位移不可信"**（日志 366 行：ds=1.906 m / 位姿位移 0.007 m） |

- 黑板本身：core/blackboard.py:106-117 的 set/update 是纯覆盖（带 source/ts，但**没有任何消费者用 get_fresh/entry 校验来源或年龄**，全仓 P_ROUTE_S 读点均走 get）。
- 换路/复位与正常推进**在日志上可分**：R1/R2 立刻把键改成 -1/0.0 或 0/0.0 且 source=MissionTask；其余变化都是 W1~W4 之间的换手。

### 4A.3 影响层判定（显示 / 判定 / 控制）

| 层 | 是否受该键影响 | 证据 |
|---|---|---|
| **显示**（Telemetry 2 Hz） | **受影响**：槽里是谁的就显示谁 | 日志 367 / 372-394 停站段 11/s4.40（Localization；Tracker 被 STOP 拦停不写）；400 / 407 / 411 离站立刻变 段6/s2.48；449 / 452 / 459 打印 **段6 s2.18 → 段6 s2.13 → 段5 s2.10**（段界怪现象，本地 Mission 同姿态逐值复现） |
| **任务判定**（Mission 到点/推进） | **受影响**：_current_s 自己就是写者与读者 | 3051 只比"缓存 vs 自己"；3069 用它判"刚越过"窗口；410 行 E9 因此 12 s 超时被强制跳过 |
| **控制**（Tracker → pursuitAngle） | **不受影响**：控制不读该键 | tracker.py:379-382 仅当 _seg_s is None 才把它当 s_hint；412-416 每拍用位姿自算 proj；343-355 被拦停时只写遥测不写键；探针 §⑤ 把键写成 s=99/段42 后 Tracker 的投影与控制逐位不变 |

### 4A.4 每次数值变化都解释得完（无剩余）

| 现象 | 解释 |
|---|---|
| 停站 段11/s4.40 → 离站 段6/s2.48 | Tracker 在 STOP 被拦停（不写键）→ 槽里是 W1/W2 的 4.40；恢复 DRIVE 后 W3/W4 继续写自己的投影，其中 **Mission 的旧 hint 驻点在 段6/2.48**，2 Hz 采样取到它 |
| 行驶中长期 段6/s2.2~2.8、且 s 随 x 变化（≈2.10+(x−0.525)） | Mission _current_s 的窗口驻点：hint 停在 2.4x 时窗口 [hint−0.6, hint+0.6] 只能看到 H10→H8 那条线，与车的 y 差 0.2~0.55 m 也照选 |
| 定位投影自愈 6→10→8 | W1 自己的 _heal_stuck（日志 311 / 321，全程 2 次、拒绝 0 次）——是**修好**而不是新增故障 |
| 同拍出现 1.9 m 的 ds 跳变（日志 366） | W1(4.40) 与 W4(2.483) 在 0.1 s 内交替写槽，FixTask 的 10 Hz 采样跨过换手 |
| 授权复位 / 换路 | R1/R2，且 keep_progress 有快照回填 |
| 倒车入口 s 冻在 2.617 | 实车运行版 tracker 的推车期冻结 _seg_s（与 HEAD 差异见 4A.6），属**预期冻结**；推车完成后抬地板 2.80 并切段（日志 296 / 302） |

**反向证据（本卡最硬的一条）**：全程日志里 **0 条** "⚠️ 投影跟丢自愈"（那是 Tracker 独有文案，tracker.py:444-448，1 s 限流）。说明 Tracker 每拍 proj.dist 从未超过 pursuit.projection_release_m=0.30——它一直贴着车，**不可能写出 2.4x**（车在 F9 时离 H10→H8 那条线 0.55 m）。探针 7 也证实：把 Tracker 强行按到 _seg_s=2.60 / _s_floor=2.10，它在 F9 位姿上第一拍就自愈到 段11/4.41，输出的 (v,st) 与实跑日志 400~441 行逐拍吻合（v0.08~0.10 / st+1.7→+28.5）。**所以控制用的是 4.4x，不是 2.4x。**

### 4A.5 最小修补候选（**Lead 2026-10-24 已批准并放行写域 —— 实施记录见 §4A.10**）

> **请求 Lead 加锁 dzrobot_v2/src/dzbrain/dzbrain/tasks/mission.py**，理由：_current_s 是唯一"读-算-回写"闭环，既影响任务判定又把旧值固化进共享键，改一处即可（D-036 边界内）。
> 获批后按此改法实施（`mission.py:1540-1545` / `3069-3075` / `3080`，增量补丁 `mission_t1_patch.diff` +29/−4）：采纳判据加一条**几何可信门**——复用同一函数已算出的 proj.dist（3050 里的 _last_lat_m）：当 proj.dist > pursuit.drift_warn_m(0.30) 时，先做一次**无 hint 全路线重投影**（与 localization._heal_stuck、tracker 的 relock 同语义，复用 Path.project），再与缓存比对；并且**读侧不回写**（只有 Localization 写键），或回写时带 max_stale_m 约束。
> 明确不做：不删写入点、不关 motion_heading/方向守卫、不为显示好看改任何常量、不动 config/**。

### 4A.6 版本差异（只读发现，不在本卡修）

车上第二趟运行的是窄覆盖层 _tmp_session/carloop-1024/reverse-fix/src/dzbrain/dzbrain/tasks/tracker.py（sha256 6365fc02b90a133b…，881 行），与本仓 HEAD（e9bff6f07fefeb0f…，892 行）在**倒车入口推车**上不同：运行版推车期不抬地板、以**原始里程反向 ≥2 cm 且切段**为完成、完成后地板抬到 s_at_waypoint(_rev_push_seg+1)；HEAD 版在推车**进入**时就抬地板、用 P_POSE 位移判反向，并多一处发布前 P_MOTION_LIMIT 钳制。两份都**没有**改 relock/floors 语义，所以 4A.4 的结论对两版都成立。

### 4A.7 复跑命令与输出摘要

| 命令（v1 根目录） | 作用 | 结果 |
|---|---|---|
| python -B -X utf8 dzrobot_v2/src/dzbrain/tests/test_agc_t1_progress_contract.py | 新增契约测试（7 组 28 条，不覆盖既有测试） | **28/28 通过，exit 0** |
| python -B -X utf8 _tmp_session/agc-1024/t1/repro_progress_contract.py | **验收要求的逐拍打印**：H8 折返 + F9 STOP→DRIVE，三写者各自的 段/s 与"键(最后写)/来源"、实跑显示对照 | 见下摘要 |
| python -B -X utf8 _tmp_session/agc-1024/t1/probe7_command_backsolve.py | 用控制器输出 (v,st) 反推控制用的是 4.4x 而非 2.4x | 10 拍逐拍吻合 |
| python -B -X utf8 _tmp_session/agc-1024/t1/probe3_duo.py | 运行版 vs HEAD 版 tracker 回放对照 + _s_floor | 见 4A.6 |

repro_progress_contract.py 摘要（H8 窗口，摘 4 拍）：

    时间   位姿(实跑)        阶段     | 真值(全路线) | Tracker(50Hz) | Localization(20Hz) | Mission(10Hz) | 键(最后写)                  | 实跑显示
    30.0s (1.035,2.353) DRIVE    | 段 7 s 2.99 | 段   7 s 2.990 | 段   6 s 2.597 | 段   6 s 2.551 | 段  7 s 2.990 TrackerTask  | 段6 s2.55
    31.0s (0.889,2.283) DRIVE    | 段 7 s 3.14 | 段   8 s 3.150 | 段   6 s 2.617 | 段   6 s 2.617 | 段  8 s 3.150 TrackerTask  | 段6 s2.59
    33.5s (0.648,2.327) DRIVE    | 段 9 s 3.62 | 段   8 s 3.377 | 段   8 s 3.371 | 段   6 s 2.250 | 段  8 s 3.377 TrackerTask  | 段8 s3.35
    35.0s (0.786,2.416) DRIVE    | 段10 s 3.99 | 段  10 s 3.991 | 段  10 s 3.982 | 段   6 s 2.298 | 段 10 s 3.991 TrackerTask  | 段6 s2.30

F9 窗口（STOP→DRIVE 换手最清楚）：

    37.0s (0.908,2.664) APPROACH | 段11 s 4.24 | 段  11 s 4.239 | 段  11 s 4.222 | 段   6 s 2.486 | 段 11 s 4.239 TrackerTask  | 段6 s2.47
    39.0s (0.908,2.826) STOP     | 段11 s 4.40 | 段  11 s 4.398 | 段  11 s 4.398 | 段   6 s 2.484 | 段 11 s 4.398 Localization | 段11 s4.39
    43.5s (0.903,2.896) DRIVE    | 段11 s 4.47 | 段  11 s 4.471 | 段  12 s 4.550 | 段   6 s 2.483 | 段 11 s 4.471 TrackerTask  | 段6 s2.48
    51.5s (0.493,3.991) DRIVE    | 段15 s 5.92 | 段  16 s 5.950 | 段  16 s 5.950 | 段   5 s 2.100 | 段 16 s 5.950 TrackerTask  | 段5 s2.10

> 读法：现场 2 Hz 打印的"段6/s2.4x"与 **Mission 列**同形同量级；两个健康写入者同拍是 4.4~5.9。
> ★ 精确指纹（测试 §⑦）：用日志 449/452/459 三行的**原样位姿**喂 Mission，本地逐值复现 (段6,2.18) (段6,2.13) (段5,2.10)——连 s=2.100 落到段界时段号变 5 这个怪现象都一致；同拍 Tracker/Localization 是 5.71/5.77/5.84。

### 4A.8 未闭合项（诚实登记）

1. **逐拍归属**仍不能从现场日志 100% 判定"那 20 ms 最后一笔是谁"——因为**记录器不写 source**。这是 E3 取数缺口，建议一行改动：TelemetryTask 打印 bb.entry(P_ROUTE_S).source。
2. W3 的 50 Hz / W4 的 ~30 Hz 在 DDS 阻塞下的真实占空比未知（不影响 4A.3/4A.4 的层次判定）。
3. 本卡不修 T2（重复格提前记账）与 T3（停滞时钟），但 4A.3 已给出 E9 跳点的**进度侧**成因；T2/T3 按原排程串行。

### 4A.9 回滚点

- **2026-10-24 前（只读阶段）**：只新增文件，零生产改动；删两个 `test_agc_t1_*.py` 与 `_tmp_session/agc-1024/t1/` 即完全回滚。
- **2026-10-24 后（§4A.10 补丁阶段）**：`mission.py` 已被本卡修改 **+29/−4**；回滚 = 一个 commit 的 revert（补丁 + 两个测试文件同一个 commit）。
  未提交前若要撤销：`git -C dzrobot_v2 apply --reverse _tmp_session/agc-1024/t1/patch/mission_t1_patch.diff`（只撤本卡增量，**保留**他人 +18/−4）。
  `tracker.py` / `localization.py` / `config/**` 全程未触碰。

### 4A.10 生产补丁（Lead 2026-10-24 批准 · **只交补丁，不提交** · **B 已落地 · 已转 e-integ 评审**）

> 前置事实（Lead 已复核并更正）：改前 `mission.py` 工作树里有 **+17/−3** 的**他人**未提交 hunk
> （CAR-GATE-20261024，owner `/root`，status in_progress：FIRE 放行门加固）。基线原样另存，
> 我的增量单独成补丁；两者用 `git apply --check --reverse` 互证 ⇒ 没吃掉别人的改动。
> （早期卡面写的 +18/−4 是 `grep -c '^+'` 把 `+++`/`---` 文件头也数进去的假数，真实改动为 **17 增 3 删**。）

#### 归属与拼合证明（给 e-integ 逐字节复核用）

| 断言 | 方法 | 结果 |
|---|---|---|
| 他人 FIRE hunk **逐字节未变** | 把 `mission_baseline.diff` 的 17 条 `+` 行逐条在当前文件里查、3 条 `−` 行确认已消失 | **17/17 存在、3/3 已消失、0 残留** |
| 当前文件 = 改前快照 + 本卡增量 | 在临时目录放 `mission_pre_t1.py` → `git apply` 本卡增量 → 比 sha256（同行尾归一后） | `a202dcb3…1b58ef` == 当前 `mission.py` **完全一致** |
| 本卡增量不含他人行 | 增量 diff 的 `+` 行 **33/33** 全部落在当前文件、且全部来自本卡两处区域 | 见 `patch/mission_t1_patch.diff` |
| 拼合计数 | 17+33=50 vs `git diff --stat` 的 52 / 3+4=7 vs 9 | **差 2/2 是 diff 对齐差异**（A→B 与 B→C 的 diff 不可加），不是漏改 |

#### 交付物（全部在 `_tmp_session/agc-1024/t1/patch/`）

| 文件 | 内容 | 校验 |
|---|---|---|
| `mission_baseline.diff` | 改前工作树 diff（**他人** hunk） | 63 行 / **+17 −3** / ``git diff`` 原样 |
| `mission_pre_t1.py` | 改前全文快照 | sha256 `8a47e341…0c0abf` |
| `mission_t1_patch.diff` | **本卡增量 A+B 定稿**（**+33 −4**；含 B 的 2 行 `_last_seg`） | `git apply --check --reverse` **通过** ⇒ 与工作树逐字一致 |
| `test_mission_t1_patch.diff` | 本卡对 `test_mission.py` 的 **2 行**（15.10/15.11 改读 `_last_seg`） | `git apply --check --reverse` 通过 |
| `test_mission_pre_t1.py` | `test_mission.py` 改前快照（生成增量用） | 与 `git show HEAD:` 一致 |
| `mission_full_after.diff` | 改后完整 diff（他人 + 本卡） | 供 e-integ 逐行审 |
| `history/` | 落地前的 A-only 增量与两份 optionB 候选（沿革留档） | 已 apply，仅备查 |

#### 改动（严格按批：两半、可分开回退）

| 半 | 位置 | 内容 |
|---|---|---|
| (a) | `mission.py:1540-1545` | 新增 `_drift_gate_m = float(cfg.get("pursuit.drift_warn_m", 0.30) or 0.30)`；**不新增配置键**（现场要临时关这层门就设 inf） |
| (a) | `mission.py:3069-3075` | 门：`proj.dist > _drift_gate_m` ⇒ `path.project(x, y)`（无 hint = 全路线，与 `localization._heal_stuck` / tracker relock 同一 `Path.project` 语义），只有更贴线才采用 |
| (b) | `mission.py:3080` | 删掉 `bb.update(P_ROUTE_INDEX=…, P_ROUTE_S=…)`：读侧只算不写 |
| (B) | `mission.py:1706` + `3080` | **Lead 裁定的扩展 B**：段号留在**本地** `_last_seg = int(proj.seg_index)`，**仍不写黑板**——给遥测/测试一个"Mission 认为自己在第几段"的见证，替代被删掉的共享键回写 |
| (B) | `test_mission.py:1844` / `1855`（共 2 行） | 15.10/15.11 的进度见证由 `bb.get(P_ROUTE_INDEX)` 改 `task._last_seg`；该文件原本干净，**只动这 2 行** |

**未动**：`Path.project`、tracker.py、localization.py、FIRE 段、`config/**`、任何常量；
授权写路径（`_reset_belief` → 段-1/s0.0、`_install_route` → 段0/s0.0 / keep_progress 快照）**原样保留**（G6 专门断言）。

#### 测试（28 + 22）

| 命令（v1 根） | 结果 |
|---|---|
| `python -B -X utf8 dzrobot_v2/src/dzbrain/tests/test_agc_t1_progress_contract.py` | **28/28** |
| `python -B -X utf8 dzrobot_v2/src/dzbrain/tests/test_agc_t1_mission_gate_fix.py` | **22/22**（G1 F9 自愈 / G2 449-452-459 指纹 / G3 零写 / G4-G5 采纳 / G6 授权写 / G7 幂等） |
| 40 套全量 A/B（同深度镜像树，同一套测试，只差补丁） | 未打补丁 **17 红** / 打补丁 **16 红**；**patched-only 红 = ∅**；唯一差异是本卡 contract 文件由红转绿 |
| `test_mission.py`（A+B 落地后） | **229/246，17 失败**；与基线逐条 diff **完全相同**（`diff fails_unpatched.txt fails_now_AplusB.txt` 空输出 ⇒ 红集合一字不差） |

复跑命令（v1 根，全部可直接跑）：

    python -B -X utf8 dzrobot_v2/src/dzbrain/tests/test_agc_t1_progress_contract.py     # 28/28
    python -B -X utf8 dzrobot_v2/src/dzbrain/tests/test_agc_t1_mission_gate_fix.py      # 22/22
    python -B -X utf8 _tmp_session/agc-1024/t1/replay_full_ab.py --json .../replay_ab.json
    python -B -X utf8 _tmp_session/agc-1024/t1/run_suite.py dzrobot_v2/src/dzbrain/tests out.txt
    git -C dzrobot_v2 apply --check --reverse _tmp_session/agc-1024/t1/patch/mission_t1_patch.diff
    cd dzrobot_v2 && python tools/wf/check.py        # 结果: GREEN（mission.py 已按本卡登记 in_flight）

★ 28 条里有 6 条（1.2 / 1.4 / 2.5 / 3.2 / 3.3 / 并 7.1 的取值口径）原本断言的是"Mission 用旧 hint 把共享键改错"这一**缺陷行为**；
补丁把回写删掉后它们必须改口径：现在断言「Mission 自己算出的值（**数值指纹一字未改**：2.483 / 2.18 / 2.13 / 2.10）+ 键不再被它覆盖」。
修复后的语义由 G1/G2/G3 断言；"修复前"用 `Rig(legacy=True)`（`_drift_gate_m = inf`）仍可逐值复现。

★ **既有的 `test_mission.py` 冲突：已由 Lead 裁决"要 B"并落地（以下留作归因证据）**：
基线（未打补丁）`test_mission.py` = **229/246，17 红**（其中 15.2 / 15.11 / 15.14 是他人路线在途改动导致的**存量红**）；
打本卡补丁 = **228/246，18 红**（**只多 15.10 这一条**）；补丁 + 可选扩展 B = **229/246，17 红，红集合与基线逐条相同**。
实测变体 A（测试改读 `task._next_wp`）= 15.10 绿但 15.11/15.12 转红（`_next_wp` 与"段号"语义不等价）⇒ **不可用**。

具体到那一条（**已按 B 落地**）：
`test_mission.py:1847` 的 **15.10「只到一半 → 仍在 HOLD」** 断言 `rig5.bb.get(P_ROUTE_INDEX) > 0`——
它拿**共享键**当"进度没被 HOLD 归零"的代理指标，而那个键在该 Rig 里**只有 Mission 会写**（`test_mission.Rig` 不含 Tracker/Localization）。
补丁按批准口径删掉回写 ⇒ 键恒为 0 ⇒ 该断言会失效（**不是回归**：HOLD 期间进度确实没归零）。
落地改法：`idx5 = rig5.task._last_seg`（`test_mission.py:1844`）+ 15.11 的 `rig5.task._last_seg == idx5`（:1855），共 2 行，其余未碰。
落地后 `test_mission.py` = **229/246，红集合与未打补丁基线逐条相同**（`diff` 空输出）。
车端三任务同构时 Localization 20 Hz 会持续写键，**现场不会出现键恒 0**（G5.3 已断言"下一拍就修回"）。

#### 整段 7_path 回放对比（`_tmp_session/agc-1024/t1/replay_full_ab.py`）

源：`_tmp_session/carloop-1024/reverse-fix/run2-mission.log`（即车上 `7_path` 节点日志，565 行 / 125 个 2 Hz 采样，N10/IDLE → A8/DRIVE）。
驱动频率与车端一致（Tracker 50 Hz / Localization 20 Hz / Mission 10 Hz），**同一份代码只差 `_drift_gate_m`**。

| 模式 | 到达 | 置位 | 键段跳变/回退/最大回退/大跳 | Mission s 回退(>5cm) | s 误差 均值/最大/超门限(0.30)拍数 |
|---|---|---|---|---|---|
| before（inf，修复前） | 17 | 18 | 19 / 0 / 0 / 1 | **5**（最深 −0.167） | **1.541 / 4.721 / 73 拍** |
| after（0.30，补丁后） | **20** | 21 | 19 / 0 / 0 / 1 | **3**（最深 −0.167） | **0.256 / 1.611 / 21 拍** |

- **零新跳变**：键段跳变序列**逐条相同**（19 次，事件列表完全一致）；停滞兜底跳点 before/after **都是 0**；越窗待定两边都是 `[]`。
- **新增 3 个"判到达"**：`#7 H8`、`#15 D10`、`#18 B9`——车实际压过这三个格心，旧代码因 s 驻留在 2.4x 只能靠 0.12 m 的"距格心"碰运气，碰不到就**静默漏点**（这正是后段一连串"距格心"的来源）。
- **8 条到达的判据变干净**：#11~#20 的到达**时刻完全相同**，只是 `why` 从"距格心"（几何兜底）变成"s+近路径"（主判据自己成立）。
- **逐拍差异 52/125**，典型（t, before_s, after_s, 真值_s）：`(1016.0, 2.483, 4.402, 4.402)`、`(1018.4, 2.455, 4.759, 4.718)`、`(1022.6, 2.359, 6.7555, 6.527)`——旧值整段冻在 H10→H8 那条线上，新值与真值差 0~2 cm。
- **after 残留的 21 拍超门限全部落在 t=1010.6~1014.6（H8 折返/倒车段）**，且 before 在这段的值与 after **逐位相同**（`diff_rows` 首条是 t=1014.8，此前 21 拍 |Δ|≤0.01 m 全部未入 diff）⇒ 门根本没参与：belief 位姿同时压在去程段与回程段上，**两段 dist 都 ≈0**，门判不出来。

**T1b（登记，非本补丁引入）· 证据行号**
| # | 证据 | 位置 |
|---|---|---|
| 1 | Mission 这层投影**不传 `motion_heading`**，共线折返只能靠 `s_hint` 裁决 | `mission.py:3072` `path.project(pose.x, pose.y, s_hint=self._last_s)` |
| 2 | Tracker 传了 —— 它用"实测位移方向"断同一物理点落在去程还是回程 | `tracker.py:405-416`（`travel_heading` → `motion_heading=travel_heading`） |
| 3 | 等距容差只有 1e-6 m²（=1 mm），H8 处两段真等距 ⇒ 只能由 hint/方向裁决 | `path.py:40,45`（`_TIE_D2_M2`）、`path.py:351`（`tie_tol = _TIE_D2_M2`） |
| 4 | 现象：`proj.dist≈0` 两段都是 0 ⇒ 本卡几何可信门（`dist>0.30` 才重投影）**结构上不可能触发** | `mission.py:3069-3075` 门；`replay_ab.json` 的 `over_gate_after` 21 行 t 全在 1010.6~1014.6 |

修法方向（不在本卡）：`_current_s` 复用 Tracker 的实测位移方向（`P_POSE` 相邻拍差分）作为 `motion_heading`，或改用 `s_min` 地板；两者都要单独验证倒车段`R:`与 PIVOT，故另开卡。

#### 回滚点

一个 commit 的 revert：**`mission.py` 补丁 + `test_mission.py` 2 行 + 两个新增测试 = 同一个 commit**。
只 revert 补丁（留下新测试）会让 contract 文件 5 条红、gate_fix 文件自动 SUSPEND —— 前者是"契约被破坏"的正确信号，后者已做 `hasattr` 守卫。
未提交前要单独撤：`git -C dzrobot_v2 apply --reverse <patch>/mission_t1_patch.diff` + `.../test_mission_t1_patch.diff`（只撤本卡，**保留**他人 +17/−3）。

#### 未闭合（交 Lead 裁）

1. ~~`test_mission.py:1847`（15.10）的代理指标要换~~ → **已由 Lead 裁决"要 B"并落地**（2 行，红集合与基线逐条相同）。
2. **提交边界**：他人 +17/−3 hunk 不能进本卡 commit（`git commit --only` 仍会带整文件；需按 hunk 拆分，或等 CAR-GATE 先提交）。**暂存区实测为空（未 add），维持不动 —— Lead 裁决 ③。**
3. 按 Lead 要求，本卡归档时把 §4A.2 的**写者-读者-复位点全表**提炼进 `docs/20_资料索引库/车端/`（可迁移的坑）。
4. **T1b：挂起（Lead 裁决：现在别做）**。四条行号已收下，结论定为"H8 等距裁决的隐患在**数据层**（路线/几何），不在本卡的门里"；等 **M1 实跑**回来再定，届时 `_current_s` 补 `motion_heading` 或改 `s_min` 由那条线负责。
5. **诊断串混源（挂起）**：`test_mission.py:1858` 的 15.11 **失败提示串**仍打印 `bb.get(P_ROUTE_INDEX)`（补丁后恒 0），会出现 `idx=0 vs 4` 的混源打印。**只影响失败时可读性，不影响判定**。Lead 裁决 ①：**不改**（保持 2 行最小 diff），等 CAR-GATE 收口后一起清。
6. **K7 / K10 / K9 三处不一致**（车端现役默认格 K7、仓库测试期待 K10、另有一条期待 K9）：**不在本卡写域**，已由 Lead 转入 e-integ 的路线回归。

## 10 文档变更

新增本卡；修改本 ns 任务树节点状态。新增 §4A 执行记录（本卡内）；新增契约测试 dzrobot_v2/src/dzbrain/tests/test_agc_t1_progress_contract.py（代码仓 tests，不计文档）；过程脚本在 _tmp_session/agc-1024/t1/（豁免区）。归档：无。

**2026-10-24 补丁轮追加（含 Lead 裁决"要 B"后的二次落地、三条收口裁决）**：卡内新增 §4A.10（补丁交付/归属拼合证明/测试/回放/回滚/T1b+诊断串+三处不一致的挂起项）；新增回归测试 `dzrobot_v2/src/dzbrain/tests/test_agc_t1_mission_gate_fix.py`（代码仓 tests，不计文档）；新增过程件 `_tmp_session/agc-1024/t1/patch/`（4 份 diff/快照）、`replay_full_ab.py`、`run_suite.py`、`ab/`（同深度镜像树）。**资料库提炼（`20_资料索引库/车端/` 写者-读者-复位全表）按 Lead 要求留到归档时做**，本轮未写库。
