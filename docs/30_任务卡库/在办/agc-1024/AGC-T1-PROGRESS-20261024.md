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

### 4A.5 最小修补候选（★需要 Lead 加锁，未动生产文件）

> **请求 Lead 加锁 dzrobot_v2/src/dzbrain/dzbrain/tasks/mission.py**，理由：_current_s 是唯一"读-算-回写"闭环，既影响任务判定又把旧值固化进共享键，改一处即可（D-036 边界内）。
> 建议改法（待批，未实施）：采纳判据加一条**几何可信门**——复用同一函数已算出的 proj.dist（3050 里的 _last_lat_m）：当 proj.dist > pursuit.drift_warn_m(0.30) 时，先做一次**无 hint 全路线重投影**（与 localization._heal_stuck、tracker 的 relock 同语义，复用 Path.project），再与缓存比对；并且**读侧不回写**（只有 Localization 写键），或回写时带 max_stale_m 约束。
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

只新增文件，零生产改动：删 dzrobot_v2/src/dzbrain/tests/test_agc_t1_progress_contract.py 与 _tmp_session/agc-1024/t1/（6 个探针/复跑脚本）即完全回滚；生产 mission.py / tracker.py / localization.py / config/** 未触碰（git status 可证）。

## 10 文档变更

新增本卡；修改本 ns 任务树节点状态。新增 §4A 执行记录（本卡内）；新增契约测试 dzrobot_v2/src/dzbrain/tests/test_agc_t1_progress_contract.py（代码仓 tests，不计文档）；过程脚本在 _tmp_session/agc-1024/t1/（豁免区）。归档：无。
