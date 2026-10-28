---
卡号: RECT-AIM-VERIFY
ns: agc-1026
agent: 4f02de25-d9ad-4285-9299-15d185f5954f
owner: 4f02de25-d9ad-4285-9299-15d185f5954f
reviewer: lead
树节点: R-7
写域: "_tmp_session/AGC-RECTIFY-20261026/verify-aim/ 及本卡"
status: in_progress
创建: 2026-10-26
---

# RECT-AIM-VERIFY · 自瞄 v5+v6 候选的独立验证

## 目标
独立判定 patches/aim_v5.patch 与 aim_v6_additional.patch 是否可采纳为候选，以及 test_aim 的 3.2/3.3 期望变更是否成立。作者为 aim-analyst，本卡为独立 reviewer，不作为作者自审。

## 读过的台账条目
D-008 冻结区；D-020 首段可靠可 GO（不要求 154 格齐）；D-023 每局真图不纳管为默认；D-033 现场靶速 0.3 m/s；D-044 不自动提交/推送。本卡新增待入账：自瞄折返判据改「窗口确认」的语义变更需独立成条。

## 复用检索结果
复用作者交付物作为**被审对象**（_tmp_session/AGC-RECTIFY-20261026/aim/ 下的 patches、candidates、out、evidence），复用既有 test_aim.py / test_stepper.py / test_drone.py 与 fakes.py、harness.py。不新建第二套仿真器或估计器；不重写被审实现。

## 新增轮子及理由
只允许独立审查探针（自己的隔离副本与自己的计数逻辑）。不新建长期工具；不把探针放进生产目录。

## 写域
_tmp_session/AGC-RECTIFY-20261026/verify-aim/ 及本卡。生产源码与冻结区一律只读；不得修改被审的 aim.py/planner.py/既有测试文件；不得覆盖作者已有产物（sandbox/、sandbox_v4/、sandbox_v5/、logs/base_test_aim.* 等）。

## 验收清单
- [x] 逐行审两个 patch，把每处改动分类为必要/可疑/越界，确认未碰 warmup/box/lead/冷却/预算/config/SCHEMA → 见执行记录 §1
- [x] 独立搭隔离副本复现 F1 死锁（±2px 交替相位下现役许可为 0），不照抄作者日志 → 见 §2（现役 ±2px 交替：90 flips / 0 许可 / ready 0）
- [x] 独立复现 F2（30fps 输入 + 50Hz tick 下 confirm_frames=2 被同一帧打满）→ 见 §3（15fps+50Hz：现役每帧改锁 45 次/3s，v6 为 0）
- [x] v5+v6 下安全语义不退化：确认后 fire_ready=0、闭锁≥5拍、stale 立即 0、撤销当拍闭锁、冷却≥500ms、budget=3、Aim 0 次写 fire_gate → 见 §4（8/8 通过；**但新增 0.14s 方向错误许可窗口，见 §5 与 Lead 必验项回答**）
- [x] 独立判定「3.2/3.3 必须改期望」是否成立；若存在保留当帧语义且不重新引入死锁的方案，必须写出 → 见 §6：**不成立（普遍命题）**；三种替代实现实测，n5 与 hybrid 均 136/136
- [x] 核验包线声明未夸大（±3px 仍出假折返、≤20px/s 检不出），不得写成已解决 → 见 §7：如实；补 n5 漏检一列
- [x] 给出接受/退回/有条件接受三选一，逐条附证据文件与命令 → **有条件接受**，见 §0/§8/§9
- [ ] Lead 独立复核后勾选（作者与本卡执行者不得自行勾此项）

## 回滚点
本卡只在 _tmp_session 内写探针与副本，无生产写入；回滚 = 丢弃本卡目录，无需 git 操作。若误触生产文件，立即报告并由 Lead 用 `git diff -- <file>` 核对后单独处置，禁止自行 restore。


## Lead 追加的必验项（20:4x，基于代码审查而非作者日志）

Lead 已自行审阅 `_tmp_session/AGC-RECTIFY-20261026/aim/patches/aim_v5.patch` 的 hunk 结构：改动集中在 RailEstimator 的折返判据（新增 `quant_step_px/flip_noise_k/flip_window_s`、`v_ref` 锁存、`_base_span/_flip_floor/_flip_slope` 辅助方法），**未见触及 warmup 帧数、box/lead、类别、冷却、预算、config/SCHEMA**。这一点与作者声明一致，但仍需逐行核。

**必须额外回答的一个问题（作者摘要未覆盖）**：
`aim.py:285` 的 `armed` = `init and not stale and warmup <= 0`。旧代码在折返**当帧**调 `_block_warmup` 使 warmup>0 从而闭锁；v5 改为**窗口确认后才**重置。因此在折返发生到被确认的这段时间（0.3 m/s 下约 0.14–0.16 s）里，warmup 仍为 0，**理论上有开火窗口**，而此期间 `_slope()` 仍在 0.40 s 窗口内带旧方向样本，lead 方向可能仍是旧的。

请独立回答：
1. 这段窗口内 `fire_ready` 实际会不会为 1？用你自己的场景构造（真折返 0.3 m/s、30fps、无噪声）逐拍打印 `armed/fire_ready/lead/v/warmup`，给出**是否存在 enabled 且方向错误的拍**。
2. 若存在，量化最坏偏差：以 0.14 s × 42 px/s 计，提前量方向取反后的落点误差是多少 px、折合板面百分比（用仓库内已有的 px/m 与靶板尺寸口径，注明出处）。
3. 这个风险与「现役 0 发」相比，是否仍是净改善？给结论，不要只给数据。

**判定口径**：若存在方向错误开火窗口，**不阻断采纳**（因现役为 0 发死锁），但必须写进"已知代价"，并给出下一趟现场可用的一拍级观测方式（从现有 telemetry 字段即可读出，不新增埋点）。


### Lead 自查（代码级，20:5x）— 供 reviewer 核对，不代替其结论

按 `aim.py:898-988` 的 `_evaluate` 判定链读：硬门 8 是 `if not est.armed`（`armed = init and not stale and warmup<=0`，见 `:285`）。旧代码折返**当帧**调 `_block_warmup` → warmup>0 → 当拍闭锁；v5 改成**窗口确认后**才重置，故折返发生后到确认前，warmup 仍为 0，**硬门 8 不拦**。

该窗口内是否真能开火，取决于硬门 7 的几何判据 `abs(pred) > box`，其中 `pred = dx + lead`、`lead` 由仍带旧方向的 `est.v` 算出（滑动窗口 `vel_window_s=0.40`，0.16 s 内换不掉旧样本）。**因此存在"方向错误却仍在框内"的构造可能**，需逐拍实测确认，不能靠读码下结论。

粗算量级（待 reviewer 用仓库口径复核，Lead 不以此为准）：0.3 m/s=42 px/s（D-033），0.14 s 迟滞 ≈5.9 px；按 D-033 的提前量 @0.875 m ≈11 px，方向取反后落点误差 ≈2×lead ≈22 px；若沿用 140 px/m@0.875 m 口径 ≈0.157 m，对 35 cm 宽靶（半宽 17.5 cm）已接近边缘。**该量级说明这不是可忽略项，必须现场用真实靶验证，不能只用离线断言"安全语义未退化"。**

**Lead 的初判（不作为验收结论）**：现役在 ±2 px 交替相位为 0 发，v6 为 10 发，即便存在 0.14 s 方向窗口，仍是净改善；但"能打"与"打得中"是两件事，前者已证、后者未证。故本卡结论最多为"有条件接受为候选"，不得写成"已解决打靶"。

## 执行记录（reviewer 执行，2026-10-26 19:44–20:05；所有判据由我自己的探针重算）

环境：cwd = D:/User/Desktop/比赛存档/v1；python 3.12.10（python -B -X utf8）；PYTHONPATH 两段用分号；
沙箱 = 我自己的 6 棵隔离树 _tmp_session/AGC-RECTIFY-20261026/verify-aim/sandbox/t_{prod,v5,v6,n5,single,hybrid}
（t_prod 与生产 aim.py 逐字节一致：sha256 7d59fd59…）。作者目录只读，未覆盖 sandbox_v4/v5 与 base_test_aim.*。

### 0. 一句话结论

**有条件接受**（v5 折返判据 + v6 类别计数），**但退回「3.2/3.3 必须改期望」的表述**，并**不得**按
evidence/EXPECTATION_DELTA.md 的方案 A 直接改主回归——作者自己的 n5 证据（本卡同目录 RECT-AIM.md:248-254）
与我本卡 §6 的独立实测都已推翻"必须改"的前提。另：v5 相对现役**新增** 0.14 s 的"方向错误仍可发许可"窗口
（见 §5），必须写进台账的已知代价，不能只用"安全语义未退化"收口。

### 1. P1 · 逐行审两个 patch

先验：两个补丁都能 **逐字节复现**作者候选（v5 用 patch -p1 --fuzz=0 打生产镜像：5 hunk、+80/−4；
v6 补丁头是候选对候选 a/aim_v5_candidate.py → b/aim_v6_candidate.py，把 v5 候选命名后同样无 fuzz 施加：
3 hunk、+9/−4）。证据 out/patch_repro.txt。

| # | 改动 | 分类 | 判断 |
|---|---|---|---|
| v5-1 | RailEstimator 新增 quant_step_px=2.0 / flip_noise_k=1.25 / flip_window_s=0.16（数据类默认值） | 必要 | 未新增 cfg 键、未改 SCHEMA/robot.yaml/_OPTIONAL_KEYS |
| v5-2 | 新增 v_ref 状态 | 必要 | 折返需"锁存的可信方向" |
| v5-3 | _seed 里 v_ref=0 | 必要 | 否则重建后沿用旧方向 |
| v5-4 | 新增 _base_span / _flip_floor / _flip_slope | 必要 | 纯新增；_slope/_trim/_seed/_block_warmup 未改 |
| v5-5 | update() 折返判据（生产 aim.py:400 那一行替换） | 必要 | 唯一语义替换；flips 只进遥测 est_flips（aim.py:1203），无外部消费者 |
| v6-1 | tick() 透传 _ingest 的"是否新帧" | 必要 | 唯一能区分帧与拍的信息 |
| v6-2 | _update_class 追加 new_frame: bool = True | **可疑** | 默认 True = 老语义，未来调用点漏传就静默回到 F2；建议必填 |
| v6-3 | _update_class 主体加 if has_target and not new_frame: return（在 NONE 早退之后） | 必要 | 位置正确：目标丢失仍清零 _cls_run |
| — | 是否碰 warmup/box/lead/冷却/预算/cfg/SCHEMA？ | **无越界** | 改动只落在 RailEstimator 与 tick/_update_class；_evaluate/_set_fire_ready/_lead_px/_permit/fire_gate/_OPTIONAL_KEYS/core/config.py 均未触及（复核命令见 §9） |

记一笔（不算越界但影响现场可调性）：三个新常量写死在冻结源码里，不读 cfg、未进 SCHEMA；
若实测抖动 ≥3 px，只能改冻结源码才能调，建议另行登记 aim.est_quant_step_px（Lead 决定）。

### 2. P2 · F1 独立复现（50Hz tick / 30fps 帧 / 靶在画面中心 / 6 s）

| 抖动 | 现役 flips / 许可 / ready | v5(+v6) flips / 许可 / ready |
|---|---|---|
| 无 | 0 / 10 / 0.482 | 0 / 10 / 0.482 |
| ±1px 交替 | **90 / 0 / 0.000** | 0 / 10 / 0.482 |
| ±2px 交替 | **90 / 0 / 0.000** | 0 / 10 / 0.482 |
| ±3px 交替 | 60 / 0 / 0.000 | **29 / 10 / 0.063** |
| ±1px 白噪声 | 60 / 2 / 0.013 | 0 / 10 / 0.482 |
| ±2px 白噪声 | 68 / 1 / 0.010 | 0 / 10 / 0.482 |
| ±3px 白噪声 | 70 / 0 / 0.000 | **6 / 10 / 0.389** |

F1 成立：现役每 2 帧判一次折返并重播种，±2px 交替下 6 s 内 fire_ready **0 拍为 1**。
修复成立：v5 在 ±1/±2px 下 flips=0、许可 10 次，与无抖动完全相同。
证据 out/{prod,v5,v6}__static_*.json、out/probe_tables.md 表 1。

### 3. P3 · F2 独立复现（3 s；数 _locked_cls 跳变，不用作者断言）

| 场景 | 拍/帧 | 现役 | v5 | v6 |
|---|---|---|---|---|
| 15fps + 每帧换类 | 3.283 | **45 次改锁（15/s = 每帧必改）** | 45 次 | **0 次** |
| 30fps + 每帧换类 | 1.659 | 30 次 | 30 次 | **0 次** |
| 30fps + 每 2 帧换类（合法序列） | 1.659 | 46 次 | 46 次 | **45 次** |
| 真·持续改类（对照） | 1.656 | 2 次 | 2 次 | 2 次 |

F2 成立：cls_switch_confirm_frames=2（robot.yaml:194）在 50Hz tick/15fps 下被**单帧**打满（第 2 拍就凑够 2 次）。
v6 修复成立且未过度抑制：每帧交替时永不确认（cls_unstable 151/151 拍），每 2 帧交替仍 45 次改锁。
证据 out/{prod,v5,v6}__cls_*.json、out/probe_tables.md 表 4。

### 4. P4 · 安全语义 8 条（v5+v6，逐条实测）

| # | 要求 | 结果 |
|---|---|---|
| 1 | 折返确认后 fire_ready=0 | 通过：30/20/15fps、有无 ±2px 噪声共 8 组，检出拍一律 False |
| 2 | 闭锁 ≥5 拍 | 通过：30fps 8 拍、20fps 12 拍、15fps 16 拍（reset_confirm_frames=5 ≈ 8.3 拍@30fps） |
| 3 | 闭锁期逐拍发 0 | 通过：区间取值集合全 False |
| 4 | 断流 stale 立即 ready=0 | 通过：场景 clear 后首拍 False，其后 30 拍全 False |
| 5 | hold 尾巴期撤销 → 当拍闭锁 | 通过：allow 后第 1 拍（hold 窗内）撤安全 → 当拍 ready=False，reason=safety_not_ok；撤销前那拍 True（迟滞正常） |
| 6 | 冷却 ≥500ms | 通过：min_gap=0.52 s，全部运行一致 |
| 7 | budget=3 恰好 3 发 | 通过：3 次许可后 reason=budget_per_target |
| 8 | Aim 0 次写 fire_gate | 通过：107 份结果 JSON 的 fire_gate_writes 全为 0 |

### 5. 对 Lead「必验项」三问的回答（用我自己的场景 rev_cross）

场景：0.3m/s（42px/s，D-033）、30fps、50Hz tick、**折返瞬间 dx 过 0**（= 几何门不掩盖估计器）、
开火框 16 px（box_w=45 ⇒ fire_box_px=16）、冷却 500ms、单靶上限 10。

**Q1 是否存在 armed 且方向错误的 ready=1 拍？——存在，且 v5 比现役更长。**

| 树 | 检出延迟 | 盲区内 ready=1 拍数 | 其中 lead 仍为旧方向（>0） | lead 范围 | 确认后 |
|---|---|---|---|---|---|
| 现役 | 0.04 s | 2 拍 | 2 拍 | +11.47（不变） | 立即 warmup=5、ready=0 |
| v5 / v6 | **0.14 s** | **7 拍**（1.00–1.12s） | **7 拍** | +11.47 → +5.25（窗口在摊薄） | 1.14 s 确认后 warmup=5，随后 25 拍 ready=0 |

逐拍证据（现役 t=1.00/1.02：ready=True reason=ok lead=+11.47 warmup=0；v5 t=1.00→1.12：
ready 全 True、lead +11.47→+5.25、warmup 仍 0、flips 仍 0，1.14 s 才 flips=1/warmup=5）。
文件：out/prod__rev_cross.json、out/v5__rev_cross.json、out/v6__rev_cross.json（trace 字段含
ready/reason/lead/v/warmup/flips）。

**Q2 最坏偏差量化**：窗口首拍 emitted lead=+11.47 px 而正确方向应为 −11.47 px ⇒ 落点误差
2×11.47 ≈ **22.9 px**；按 140 px/m（robot.yaml:852）≈ **0.164 m**；靶板 35×35 cm
（board_half_m = CELL_M/2 = 0.175 m，技术全景_A_车端核心:1221）⇒ 16.4 cm ≈ 半宽 17.5 cm 的 **94%**，已贴板边。
窗口末拍 lead=+5.25 ⇒ 误差 10.5 px ≈ 0.075 m ≈ 半宽 43%。另：窗口内靶自身又走了 42px/s×0.14s ≈ 5.9 px，
误差按同一方向叠加，不会互相抵消。

**Q3 是否仍是净改善？——是净改善，但只能写成"把永久失效换成每次折返 0.14 s 的可发窗口"。**
现役在 ±2px 交替/白噪声下 6 s 只有 0/1 次许可（几乎永久失效）、±1px 下 2 次；v5 在同噪声下 10 次许可、
0 假折返。因此净改善成立（现役是"打不出去"，v5 是"打得出去但折返前后 0.14 s 里有 1~2 发可能偏 10~23 px"）。
不成立的部分：**不得**据此写"安全语义未退化"或"打得中已解决"——命中未证。
可行的下一轮（非本卡阻断项）：折返确认前的拍把 lead 输出强制为 0（只压提前量、不动 ready），
可把窗口内错误提前量变成"无提前量瞄准"；或让"最近一帧 dz 与 v 符号相反"当拍 lead=0。二者都要另行验证噪声下的抖动。

### 6. P5 · 「3.2/3.3 必须改期望」是否成立

我自己算：判据在 aim.py:400（abs(v)≥2.0 and v*dz<0，dz 是单帧）。D-033 口径 42 px/s ⇒ 单帧 1.4 px@30fps、
2.1 px@20fps；±2 px 抖动下 dz=1.4+Δnoise（Δ∈[−4,4]），**P(dz<0)≈24%**，交替相位每 2 帧必出一次 dz<0
⇒ **单帧确实不可分**（这点作者对）。但"不可分 ⇒ 必须改期望"不成立：干净序列（σ=0）上单帧可分，
而 test_03（tests/test_aim.py:420，断言在 :451/:455）用的正是零噪声 2px/帧理想序列。

| 方案 | test_aim | 静靶±2px | 42px/s 真实反向 | 42px/s+±2px 反向 | 判定 |
|---|---|---|---|---|---|
| v5（作者） | 134/136 | 0/0 flip，10 许可 | 0.14 s | 0.18 s，1 flip | 噪声鲁棒，弃当帧 |
| n5（作者自己的 σ 自适应） | **136/136** | 0/2 flip，10 许可 | **0.04 s** | **未检出（0 flip）** | 保住当帧，但噪声下**丢折返闭锁** |
| single（我写：单帧 dz + 可信 v_ref） | **136/136** | 0/0，10 许可 | 0.04 s | **12 flips，仅 2 许可**（±3px 交替：60 flips/120 resets/**0 许可**） | **重新引入死锁**，应否定 |
| hybrid（我写：v5 窗口 + n5 快路径） | **136/136** | 0/0，10 许可 | **0.04 s** | 0.18 s，2 flip（多 1 次假折返） | 两者兼得，需再调 |

结论：(1) **"必须改"作为普遍命题不成立**（n5/single/hybrid 都 136/136）；(2) **作为 v5 的后果成立**
（固定 Q=2 的窗口判据不可能在喂进反向样本那一拍给 resets+1）；(3) EXPECTATION_DELTA §2 里
"不是实现妥协、是可证的不可能"应改为"在 ±2px 量化噪声 + 单帧判据下不可能；换噪声自适应判据可保留当帧，
但代价是噪声下漏检折返"；(4) 作者卡 RECT-AIM.md:248-254 已自我更正（n5 不用改），
**交付物之间不一致**，这是我要求阻断"按方案 A 改主回归"的直接理由。

**建议改法（比方案 A 好，也不放松安全）**：保留 3.2/3.3，另加两条抖动子用例——
3.2b：±2px 交替 6 s 内 flips==0 且许可≥2（否则死锁）；3.2c：±3px 时允许假折返但不得出现"许可=0"
（把包线边界写进测试）。若 Lead 坚持先合 v5，则方案 A 可用，但必须同时写明"放弃干净序列当帧确认"这一代价。

### 7. P6 · 包线声明核查（未夸大）

- ±3px 起仍出假折返：作者 envelope_v5.log A/B 表标"退化为不可用"，卡里写"7~29 次、仍能发许可、不再死锁"；
  我实测 ±3px 交替 29 flips/10 许可/ready 0.063，±3px 白噪声 6 flips/10 许可/0.389 → **如实**。
- ≤20px/s 检不出：作者写"漏检"；我实测 10/20 px/s 均 0 flips（30fps）→ **如实**。
- 补充：**n5 在 ±2px 噪声下把 42px/s 的真实反向也漏检**（0 flips；其 table_4col.md 只报"假折返=0"）
  → 选型必须加"真折返检出率"一列，否则 n5 的优势是单边证据。
- 我一度按 robot.yaml:846 的 target_speed_mps=0.04（5.6px/s）怀疑"真靶不可检出"，核 D-033 后**撤回**：
  该键是注释残留，不得据此重开（本卡口径一律 42 px/s）。

### 8. 发现清单

| 级别 | 项 | 证据 | 要求 |
|---|---|---|---|
| S1 表述（阻塞合并口径） | EXPECTATION_DELTA 的"可证的不可能"与作者卡 §0 自相矛盾；照此删断言会永久丢掉干净序列的当帧确认 | evidence/EXPECTATION_DELTA.md:18-30；RECT-AIM.md:248-254；本卡 §6 | Lead 先裁决：改子用例（推荐）或明确接受代价 |
| S2 交付形态 | v6 补丁头是候选对候选；_update_class(new_frame=True) 默认值留后门 | out/patch_repro.txt §3 | 合并前改生产路径；new_frame 改必填 |
| S3 可维护性 | 三个新常量写死、不读 cfg、未进 SCHEMA | aim_v6_candidate.py:254-258；core/config.py:421 | Lead 决定是否登记 aim.est_quant_step_px |
| S4 证据可复现 | out/matrix_v5.json 对应的 sandbox_v5 已不存在；sandbox_v4 现 sha 563f85ee… ≠ aim_v4_q12 的 913f079d… ⇒ 旧 matrix 无法按现状复跑 | aim/out/matrix_v5.json、aim/make_sandboxes.py:44 | 以我的独立矩阵（§2/§3/§4/§5）作验收证据 |
| S5 安全残余 | 检出盲区 0.14~0.22 s，盲区内可发许可（本卡 §5：7 拍、lead 仍旧方向、最坏 22.9 px） | out/v*__rev_cross.json | 写进台账"折返判据=窗口确认"条目，作为已知代价 |
| S6 未验证 | 车端 det_node 真实抖动幅度/形状未知，Q=2px 是假设；实测 ≥3px 时按包线会退化 | 作者卡同结论；本卡 §2 | 上车前抓真实 dx 序列估 quant_step_px |

### 9. 复跑命令（cwd = D:/User/Desktop/比赛存档/v1；真实 returncode）

    V=_tmp_session/AGC-RECTIFY-20261026/verify-aim

    # 构造 6 棵隔离树 + 补丁复现对账
    python -B -X utf8 $V/probes/mk_evidence.py      # → out/patch_repro.txt、out/alt_tradeoff.md
    python -B -X utf8 $V/probes/build_variants.py   # n5、single（n5 只 import 作者 make_sandboxes.V5 常量）
    python -B -X utf8 $V/probes/build_hybrid.py     # hybrid

    # 75 次独立探针（24 场景 × prod/v5/v6），rc 全 0
    bash $V/probes/run_matrix.sh                    # → out/rc_probe.tsv、out/*.json、logs/*.log
    python -B -X utf8 $V/probes/agg.py              # → out/probe_tables.md

    # 本卡 §5 的 Lead 必验项场景（现役/v5/v6）
    for tag in prod v5 v6; do
      PYTHONPATH="$V/sandbox/t_$tag;$V/sandbox/t_$tag/tests" PYTHONUTF8=1         python -B -X utf8 $V/probes/probe.py --tree sandbox/t_$tag --scenario rev_cross         --trace > $V/out/${tag}__rev_cross.json 2>/dev/null
    done

    # 既有回归（各自独立进程，取真实 returncode）
    for tag in prod v5 v6 n5 single hybrid; do
      PYTHONPATH="$V/sandbox/t_$tag;$V/sandbox/t_$tag/tests" PYTHONUTF8=1         python -B -X utf8 $V/sandbox/t_$tag/tests/test_aim.py > $V/logs/test_aim_$tag.log 2>&1
      echo "$tag rc=$?"
    done
    # prod 136/136 rc=0；v5、v6 134/136 rc=1（仅 3.2/3.3）；n5、single、hybrid 136/136 rc=0
    # 邻接套件（prod vs v6 同结果）：test_gimbal_lock 58/58、test_safety 190/190、
    # test_contract 56/56（需把 config/robot.yaml 拷进沙箱根）、test_dual_shoot 32/35
    # （3 条红是沙箱缺 tools/ 的路径问题，两棵树完全一致，非候选引入）

    # 越界复核
    grep -n 'quant_step|flip_noise|flip_window' dzrobot_v2/src/dzbrain/dzbrain/core/config.py   # 无输出 = 未改 SCHEMA

### 10. 影响面与回滚

- 生产零改动：aim.py 前像 sha256 7d59fd5970c494031998d8d71abca2cd2fce91c204cde892a3af6dcf67d170d2
  （与 candidates/aim_prod.py 逐字节一致）；既有 tests/test_aim.py 未被我触碰。
- 全部产出在 _tmp_session/AGC-RECTIFY-20261026/verify-aim/（probes/ sandbox/ out/ logs/ patchwork/ v6apply/），删目录即回滚。
- 未 commit/push/reset/clean/restore；未部署、未接车/机、未发 ROS/激光、未起栈。

### 11. 未验证（UNKNOWN，不许当已解决）

1) 未上车、未接设备：全部结论来自 FakeBridge 隔离仿真。2) ±2px 抖动是威胁模型假设，不是实测。
3) v5/v6 未合并、未部署。4) 不声称"已验收"——本卡只给"有条件接受"。5) n5/single/hybrid 是我造的评审对照树，
不是可合并候选；single 应被明确否定（会重新引入死锁）。

## 未验证
本卡不接设备、不起栈、不发车、不出光、不飞行、不部署。离线回放不等于实车行为。

## 文档变更
新增本卡；树与索引由 Lead 单写。（R-7 节点 Lead 已于 19:46 落树：`00_树.md:21` 已指向本卡，`check_doc_policy` 的 R8 因此为绿。）

**本卡门禁（reviewer 跑，2026-10-26 20:04）**：
- `python -B -X utf8 docs/_工具/check_doc_policy.py` → GREEN，受管 2221，**红线 0 / 提醒 0**（rc=0）
- `python -B -X utf8 dzrobot_v2/tools/wf/check_links.py` → checked 241 / **broken 0**（rc=0）
- `python -B -X utf8 docs/_工具/check_anchors.py` → **本卡 0 红 0 越界**；全局"新增红 2"两条均在 `RECT-PLAN.md`（plan 线：`_tmp_session/AGC-RECTIFY-20261026/plan/isolated/preimage_planner_sha256.txt` / `_tmp_session/AGC-RECTIFY-20261026/plan/isolated/postimage_planner_sha256.txt`），1 条行号越界在 `技术全景_D_无人机.md`，**都不是本卡引入**（原始输出 `_tmp_session/AGC-RECTIFY-20261026/verify-aim/out/anchors.log`）。

## 经验总结
待复核后提炼，来源/结论回指项目资料库。
