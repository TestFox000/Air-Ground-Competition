---
卡号: RECT-AIM
ns: agc-1026
agent: aim-analyst
owner: aim-analyst
reviewer: lead
树节点: R-1
写域: "_tmp_session/AGC-RECTIFY-20261026/aim/"
共享任务: task-1
执行者会话: 571b4e57-8944-49c9-ac0b-4d9aa8614f0a
status: in_progress
创建: 2026-10-26
---

# RECT-AIM · 自瞄估计与真实两钥匙

## 目标
用户本轮目标：优先完赛，在安全和时间余量内尽量压雷、两有效最佳射击点并命中。不默认为禁火仅跑路线。

共享任务 **task-1**（卡面旧 task-12 是运行时重启前的历史编号，按 Lead 派发包原位补新 ID）。
本会话**没有** team_task_list/get/claim 这组工具（探测 tools.team_task_list 不存在），无法走
list/get/claim；按 Lead 派发包执行，agent/owner 已写真实执行者 aim-analyst（DSH_SESSION_ID
571b4e57-8944-49c9-ac0b-4d9aa8614f0a，父会话 session-c9533fa6），不冒用 lead。

## 读过的台账条目
Lead已读D-008冻结登记、D-012唯一tour入口、D-023每局真图不纳管为默认、D-033现场靶速0.3m/s、D-044不自动commit/push。执行者须补充本卡相关条目。旧注释0.04/5.6px/s只作历史，不得重开口径。

执行者补读（dzrobot_v2/docs/决策台账.md）：
- **D-033**（台账 :136-144）：靶速 = 0.3 m/s（现场实测），D-031 的 1.0 m/s 为设计约定值已被取代；判据一律按 0.3 m/s ⇒ 42 px/s @140 px/m。**本卡所有场景的"真实靶速"用它，不用 0.04/5.6px/s。**
- **D-033 §2**（台账 :141）：config/** 仍冻结，aim.target_speed_mps=0.04 继续按"注释残留"处理。**实测它的唯一运行时作用是估计器速度夹紧兜底 max(300, 5×5.6)=300 px/s，对 42 px/s 靶零影响**，故本卡不改配置、也不把它当缺陷重开。
- **D-020**（台账 :20）：Go 只要求"首段可靠"，不要求 154 格齐 ⇒ 本卡不把"整场跑完才能校准估计器"当先决条件。
- **D-044**（台账 :260）：提交/推送一律等用户指令 ⇒ 本卡**未提交、未推送、未做任何 git 写操作**。
- D-008 冻结区：src/dzbrain/dzbrain/** 属冻结区 ⇒ 本卡生产只改**隔离副本**，补丁以 diff 交回。

## 复用检索结果
先读docs/README.md，用doc_search检索本主题；再读dzrobot_v2/docs/复用清单.md→能力索引.md→目录索引.md。复用night1025的工具与证据但不信其自报验收；此段由执行者补命中项。

执行者检索结果（命中项 + 结论）：
- doc_search "折返 量化抖动 提前量" --in 视觉与云台 → 4 篇命中：视觉与云台/03（L15 0.3 m/s 下提前量 11 px < 25 px 框、真瓶颈是观测连续性）、00_索引（L15 口径定案 D-033；L30 端到端 0.5~0.9 s 待定）、01（L19 三条结构性错位、靶速 1 m/s 结论已作废）、02（L105 FIRE 段 lead 非零仅 2.8%）。**结论：本卡不需要新口径，只需把"估计器折返判据"这条实现缺口补上。**
- 复用清单（docs/复用清单.md:20-21,53）与能力索引（:23-24）：与瞄准相关的既有能力是 aim_autotune（PID 整定）与 aim_lock（检测锁定门）——**都不是估计器/折返**，本轮不新建工具、不改它们；
- 目录索引/共用实现：RailEstimator + AimTask 在 src/dzbrain/dzbrain/tasks/aim.py，唯一实现；测试脚手架在 tests/fakes.py + tests/harness.py + tests/test_aim.py（本卡 16 条候选用例直接复用它们，不复制）。

## 新增轮子及理由
不新建规划器/仿真器/部署器；只补缺口测试，若新增永久工具必须先说明既有不足并请Lead登记。

执行者结论：**不新增任何永久工具**。本轮所有脚本都在 _tmp_session 内（scripts/matrix.py 场景矩阵、
scripts/envelope.py 包线扫描、scripts/make_v5.py / make_v6.py 生成补丁），只作证据复跑入口；
唯一要进仓库的是 **生产单测候选** candidates/test_aim_rectify.py（合并进 tests/test_aim.py 后即为既有测试的一部分，
不新增工具、无需复用清单登记）。为什么必须新写它：现役 test_aim 第 1/3 组用的是**无量化噪声**的理想序列，
抓不到 ±1/±2px 整型抖动导致的假折返与死锁（现役 35 场景里 21 条红），这不是"现有测试不够用"的判断题，是实测结论。

## 写域
_tmp_session/AGC-RECTIFY-20261026/aim/及本卡；其余生产代码只读，现有未提交改动不得覆盖。补丁先在隔离副本；只有RECT-GATE可直接改所列非冻结文件。不碰共享登记/索引/状态板。

## 验收命令与清单

环境：**python -B -X utf8**；cwd = D:/User/Desktop/比赛存档/v1/_tmp_session/AGC-RECTIFY-20261026/aim；
沙箱 = sandbox/src_dzbrain（dzbrain 6.6 M 拷贝，生产目录只读）；PYTHONPATH 两段用**分号**（Windows）。
判定一律取**真实 returncode**（不是 tail 退出码），日志 UTF-8 落 logs/，PASS/FAIL/UNKNOWN 分开。

    # 1) 35 场景矩阵（现役 / v4 / v5 / v6 各跑一遍，输出 out/matrix_<cand>.json）
    python -B -X utf8 scripts/matrix.py --cand prod --out out/matrix_prod.json
    python -B -X utf8 scripts/matrix.py --cand v4   --out out/matrix_v4.json
    python -B -X utf8 scripts/matrix.py --cand v5   --out out/matrix_v5.json
    python -B -X utf8 scripts/matrix.py --cand v6   --out out/matrix_v6.json

    # 2) 生产单测候选（16 条；现役 8/16、v5 15/16、v6 16/16）
    cp candidates/aim_v6_candidate.py sandbox/src_dzbrain/dzbrain/tasks/aim.py
    PYTHONPATH="sandbox/src_dzbrain;sandbox/src_dzbrain/tests" PYTHONUTF8=1         python -B -X utf8 candidates/test_aim_rectify.py

    # 3) 生产既有套件（同一沙箱，切补丁前后各跑一次）
    PYTHONPATH="sandbox/src_dzbrain;sandbox/src_dzbrain/tests" PYTHONUTF8=1         python -B -X utf8 sandbox/src_dzbrain/tests/test_aim.py

    # 4) 包线扫描（抖动幅度 / 靶速 / 帧率）
    python -B -X utf8 scripts/envelope.py --cand v5

- [x] 当前生产基线可复现与修后对照，且非仅字符串测试 —— 35 场景矩阵：现役 14/35 通过、v4 14/35、v5 34/35、v6 35/35；逐场景数字 out/matrix_*.json
- [x] 已知Aim专项与慢折返、量化抖动、实际靶速0.3m/s场景运行 —— 0.3 m/s（42 px/s，D-033）× 15/20/30/60 fps × 无噪声/±1px/±2px 白噪声/±2px 交替相位；真实折返（三角波端点）检出与闭锁
- [x] 许可双钥匙**Aim 侧**、撤销与冷却不退化 —— 35/35 场景 bridge.fire_gate 写入 **0 次**（Aim 不碰任务钥匙）；hold 尾巴期间撤销安全当拍闭锁；冷却间隔 ≥500 ms；budget=3 恰好 3 发；断流 ready=0/v=0
- [ ] C++ 两钥匙**实车链路**（不属本卡离线写域；Lead 已用 test_aim 136/136 与源码复核过，本轮未复跑）
- [x] 最小补丁/前置hash/原始输出与回滚齐备 —— patches/aim_v5.patch（+80/−4；核心判定只改 update() 里一处 if，另加 3 个字段 + 2 个辅助方法）、patches/aim_v6_additional.patch（+9/−4）；前像 sha256 见回滚点；原始日志 logs/
- [ ] Lead 独立复跑后认可（作者不得自行勾此项）

## 回滚点
修改前记录sha256和逐文件副本到本卡_tmp目录；只逆转本卡差异，禁止git reset/clean/整树覆盖/提交/推送。

**本轮实际状态：生产零改动**，所以"回滚"= 丢弃候选。前像（与 Lead baseline/rehash_1855 一致，aim.py same:true）：

| 文件 | sha256（前像） |
|---|---|
| dzrobot_v2/src/dzbrain/dzbrain/tasks/aim.py | 7d59fd5970c494031998d8d71abca2cd2fce91c204cde892a3af6dcf67d170d2 |
| dzrobot_v2/src/dzbrain/tests/test_aim.py | 7ae42342b42985714aadb453e0596f67237f59b7c90c35a3c72a6a57a1587f7b |
| dzrobot_v2/src/dzbrain/tests/fakes.py | cad37ae8fbf372624e105c8ae9f362f0c082f513c385a1175989d8ace83f1844 |
| dzrobot_v2/src/dzbrain/tests/harness.py | 1ef6f86bc0ef9e19a76928090e57d64f300797e5b9e4a869330ba658be8dbccb |
| dzrobot_v2/src/dzbrain/dzbrain/core/config.py | 0cd14fc29be7164a841d8e089328e838a99e3e81eb92a1a0a168e547564add23 |
| dzrobot_v2/config/robot.yaml | d61effc361b02284fd70399e7cb2d19e3309c8ecf1ac4f2a06767ae83d44d313 |

沙箱内 aim.py 每轮切补丁后都已还原到 7d59fd59…（跑完实测 restored sha 一致）。
若要落地：apply patches/aim_v5.patch（+ v6 可选）→ 由 owner 跑 test_aim 与合并候选单测；
**回退 = git checkout 该文件**（本卡没有提交，D-044）。

## 执行记录
执行输出落本卡_tmp目录，结论在本卡写，不新建第二份长期REPORT。

### 执行者报告（aim-analyst / task-1，2026-10-26 19:09–19:40）

#### 0. 一句话结论
**night1025 的 v4 候选不足以采纳**：它只在"静靶 ±1px 白噪声"这一格部分有效（3→10 发），
在 ±2px、交替相位、15/20/60fps 与移动靶场景上仍与现役同红 —— 35 场景里**现役 21 红、v4 21 红**，
且静靶 ±2px 交替相位两个版本都是 **0 发（死锁）**。根因不是阈值高低，而是**判据本身仍用单帧 dz**。
本轮隔离复现后交两个**互相独立**的最小补丁（v5 折返判据 / v6 类别按帧计数）+ 16 条生产单测候选 + 前后证据；
**生产零改动、未提交、未推送**。

#### 1. 复现环境（可独立复跑）
- 沙箱：sandbox/src_dzbrain（dzbrain 6.6 M 拷贝）+ sandbox/config/robot.yaml；**生产目录只读**，aim.py 前后 sha256 = 7d59fd59…（与 Lead baseline same:true）。
- 场景矩阵 scripts/matrix.py：50Hz tick、视觉 15/20/30/60 fps、**35 个场景**；AimTask/RailEstimator 用生产代码原样（候选以 dzbrain.tasks.<name> 载入），替身用生产 tests/fakes.py 的 FakeBridge/FakeClock。
- 判据与实现无关（先声明的，不是跑完再凑）：**假折返** = 距真实折返 >0.6 s 的 flips 增量；**真折返** = 三角波半周期端点（第一版三角波公式在端点有 48 px 台阶，已修并记在脚本注释里）；**许可** = telemetry.permits_total 增量；另判 hold 尾巴上限、闭锁期零许可、fire_gate 零写入。

#### 2. 已证缺陷 F1：折返判据用单帧 dz（P0，v5 修）
机制（代码级）：aim.py:399-403 旧判据 = abs(v) >= flip_min(2.0) and v*dz < 0，dz 是**单帧**位移；
而 _seed() 把 base 重建成**单样本**（aim.py:297），重播种后第一拍 dt≈33 ms ⇒ 1 px 抖动 = **30 px/s** ⇒ 越过 2.0 ⇒ 判折返 ⇒ 再 _seed。**抖动→判折返→重播种→再抖动**，永不收敛。

实测（6 s，30 fps，静靶，同一条假目标只换噪声）：

| 版本 | ±1px 白噪声 | ±2px 白噪声 | ±2px 交替相位 |
|---|---|---|---|
| 现役 | 56 次判折返 / **3 发** | 63 次 / **1 发** | 90 次 / **0 发** |
| night1025 v4 | 15 次 / 10 发 | 43 次 / 7 发 | 90 次 / **0 发** |
| v5（本轮） | **0 次 / 10 发** | **0 次 / 10 发** | **0 次 / 10 发** |

v4 为什么不够（可复算）：它把下限改成 Q/dt（Q=1.2）——只对"算 v 的那段基线"有效；
满窗 0.40 s 时 1.2/0.4 = **3.0 px/s**，而 ±2px 抖动的满窗端点差分噪声上限是 2×2/0.4 = **5.0 px/s**，
仍然越线；且它**没改判据**，dz 还是单帧的，交替相位下每帧反号 ⇒ 依然死锁。

#### 3. 已证缺陷 F2：类别确认数的是 tick，不是帧（P1，v6 修，独立变量）
aim.py:1256-1281 的 _update_class 每拍（50 Hz）计一次 cls_run，但视觉是 30 Hz：**同一帧会被读到 2 次**，
于是 cls_switch_confirm_frames=2（robot.yaml:194，10-24 由 5 改 2）被**一帧**打满 —— 单帧误判即可改锁。
实测（30 fps 视觉 / 50 Hz tick、帧类别交替 1 s）：锁定类别**改锁 11 次**，cls_run 在同一帧的两拍里就 1→2。
修法：把 _ingest 的"本拍是否吃到新帧"传给 _update_class，非新帧不再计数（v6，+3 逻辑行）。
修后同一测试：锁定类别 0 次切换，且 fire_only_blue 下红帧仍全部被拦。

#### 4. 前后对照（35 场景，判定口径同上）

| 版本 | 通过 | 红 | 关键项 |
|---|---|---|---|
| 现役 | 14 | 21 | 静靶 ±2px 交替 0 发；0.3 m/s 移动靶 ±2px 白噪声 17 次假折返 / 6 发 |
| night1025 v4 | 14 | 21 | 同上（仅静靶 ±1px 改善） |
| **v5** | **34** | 1 | 唯一红是 F2（类别），与 F1 无关 |
| **v6** | **35** | 0 | F1+F2 全绿 |

0.3 m/s（D-033 = 42 px/s）+ ±2px 交替相位（60 fps）：现役 150 次判折返 / 0 发；v5 5 次 / 10 发。
0.3 m/s 三角波真实折返：现役 41 次判折返（19 次是噪声假折返）/ 4 发；v5 **5 次判出 5 次真折返、0 假折返 / 10 发**。

#### 5. 修法（最小补丁）
- **v5**（patches/aim_v5.patch，+80/−4；新增里约一半是写进源码的不变量注释，**判定逻辑本身是一处 if + 两个辅助方法**）：
  1) 折返改为**两道闸**：参考方向 v_ref（只在 |v| 越过噪声底时刷新、折返时锁存）+ 最近 flip_window_s=0.16 s 的**端点差分** v_fast 反号；2) 下限 = max(配置 2.0, k·2Q/span)，Q=2px、k=1.25 ⇒ 0.16 s 基线 ≈31 px/s、满窗 0.4 s ≈12.5 px/s、刚重播种 ≈120 px/s（单帧 30 px/s 的假折返够不到）；3) **完全不碰** config/SCHEMA、warmup 帧数、box/lead、类别、冷却、预算门。
  为什么不加"当帧快判"：20 fps 下真实反向的单帧位移 ≈2.1 px，±2 px 噪声的单帧位移也可以是 2 px —— 单帧上与噪声**不可分**（见 evidence/EXPECTATION_DELTA.md §2），加了就等于把假折返放回去。
- **v6**（patches/aim_v6_additional.patch，+9/−4，逻辑就 3 处：函数签名加参数、tick 里把 new_frame 传进去、非新帧早退）：_ingest 的 new_frame 传入 _update_class，非新帧不计数。单独变量、单独测量，不混在 v5 里。

#### 6. 验收结果（真实 rc + 原始日志位置）
| 验收项 | 现役 | v5 | v6 | 证据 |
|---|---|---|---|---|
| 35 场景矩阵 | 14/35 | 34/35 | **35/35** | out/matrix_*.json、logs/ |
| 生产单测候选（16 条） | 8/16 rc=1 | 15/16 rc=1 | **16/16 rc=0** | logs/test_rectify_*.log |
| test_aim.py（既有 136 条） | 136/136 rc=0 | 134/136 | 134/136 | logs/test_aim_*.log；2 条见 EXPECTATION_DELTA |
| test_gimbal_lock / test_safety / test_agc_t1 / test_contract | 58/58·190/190·22/22·56/56 | 同 | 同 | logs/*.log（prod 与 v6 **逐字节相同**） |
| test_dual_shoot / test_bridge_gate | 32/35·44/47（环境缺图，既有红） | 同 | 同 | 同上，日志逐字节相同 |
| test_mission / test_deploy | rc=1（缺 config/map*.yaml，工作区已删） | 同 | 同 | 同上，日志逐字节相同 |
| fire_gate 写入（双钥匙纪律） | 0 | 0 | 0 | 35 场景矩阵每场景检查 |

包线（scripts/envelope.py，如实写边界）：
- 静靶白噪声：±0/±1/±2 px ⇒ 0 假折返、10 发；**±3 px ⇒ 7 次假折返（仍 10 发）；±4/±6 px ⇒ 16 次（仍 10 发）**——即"能打但会白丢闭锁窗口"，不再死锁。
- 交替相位：±1/±2 px ⇒ 0 假折返；±3 px ⇒ 29 次。
- 折返检出时延（无噪声 30 fps）：61.6 px/s 0.140 s、42 px/s 0.140 s、30 px/s 0.180 s；**20 px/s 与 10 px/s 检不出**（0.143/0.071 m/s，低于噪声可辨下限）。
- 帧率：15/20/30/60 fps 在 0.3 m/s + ±2px 下都是 5 次真折返、9~10 发、0 假折返。

#### 7. 需要 Lead 决定的三件事（我不越权）
1. **采纳范围**：v5 必修（F1，21 红→1 红）；v6 是独立变量（F2），可同批也可另开卡。
2. **test_03 的 3.2/3.3 期望**：与"窗口确认"互斥，改法见 evidence/EXPECTATION_DELTA.md（推荐方案 A：改成 ≤0.25 s 内确认，安全语义不放松；本卡**没有**改测试文件）。
3. **quant_step_px 是否升级为 SCHEMA 键**（现场可调）：需要改 core/config.py 的 Spec（非本卡写域）＋ _OPTIONAL_KEYS 自检；不升级则它是 RailEstimator 的类默认值（Q=2.0），现场改要改代码。

#### 8. 需要 Lead 知道的一处并发情况（不是我的产出）
本卡写域 _tmp_session/AGC-RECTIFY-20261026/aim/ 里出现了**不是我写的**一组文件：
sandbox/dzbrain、sandbox_v4/、sandbox_v5/、logs/base_test_aim.stdout/stderr.txt、
evidence/base.json（tag=base，19:07–19:38 时间戳），用的是"假目标 + 逐门 drop"的另一套脚手架。
我按"不覆盖他人产出"**一个都没动**，我的产物全部另行命名（sandbox/src_dzbrain、sandbox_repo、
candidates/aim_v5_candidate.py、patches/、scripts/matrix.py、evidence/summary.json）。
若那是同一 task-1 的旧进程仍在跑，请 Lead 决定谁是权威，避免两边同时改本卡。

#### 9. 纪律与回滚
- 生产源码**零改动**；沙箱 aim.py 每轮切补丁后都还原并核对 sha256；**未 commit / 未 push / 未 reset / 未 clean / 未改测试期望 / 未部署 / 未接设备**。
- 回滚 = 丢弃候选（或 git checkout aim.py），无残留；补丁与证据全在 _tmp_session/AGC-RECTIFY-20261026/aim/（candidates/、patches/、scripts/、logs/、out/、evidence/summary.json）。
- 树/索引/台账/状态板由 Lead 单写，本卡只在末尾写了"请求 Lead 落地"的两行。

### Lead前置复核（2026-10-26）

昨夜C03“同名即复制钥匙”不足以定缺陷：当前本地Mission经publish_fire_gate发任务钥匙（mission.py，任务放行函数），Aim只经publish_fire_ready发瞄准钥匙；io/ros_io.py两publisher独立，C++externalFireAllowed要求两项与各自新鲜度。P_AIM.fire_gate是状态字段名，不能等同/aim/fire_gate话题。既有test_aim第9组专门断言Aim不发任务钥匙，Lead本轮复跑136/136、rc0，日志见[原始输出](../../../../_tmp_session/AGC-RECTIFY-20261026/gate/round1_results/20261026_181003_814771/test_aim.stdout.txt)。

因此暂撤“C03已证链断”前提，**不改许可链来修命名**。这不证明车上二进制/参数相同或已实射；现场use_external_fire_gate、两topic身份及失效撤销仍须核。估计器v4本身仍待指定执行者补现行0.3m/s、帧率、抖动和hold尾巴边界证据，不关闭本卡。

### Lead 裁决（2026-10-26 20:1x）

**采纳范围：v5 + v6 同批**（仅隔离候选，未合入生产）。F2 是独立变量（tick 计数 vs 帧计数），只上 v5 则 30fps 下类别仍会被单帧打满确认；二者互不覆盖，分批反而多跑一轮。

**方案 A 有条件同意**：3.2/3.3 改为“≤0.25 s 内确认”，但必须同时满足——
1. 不许删除或弱化 3.4（折返瞬间 fire_ready=0）/3.5（闭锁≥5 拍）/3.6–3.8（方向、lead 符号、闭锁期逐拍发 0）；这四条才是安全语义，本轮回放中仍全过；
2. 新语义要写清“为什么不可当帧”：20 fps、0.3 m/s 下单帧反向位移 ≈2.1 px，±2 px 噪声也能造出 2 px 单帧位移，二者在单帧上不可分（evidence/EXPECTATION_DELTA.md §2）。这是可证的不可能，不是实现妥协；
3. 语义变更必须入决策台账（新增 D-xxx），不许只留在测试注释里。

**quant_step_px 暂不升级为 SCHEMA 键**：v6 用类默认 Q=2.0 已覆盖 ±2 px；改 core/config.py 属冻结区且无现场需求证据，等实测抖动确实 ≥3 px 再单开卡。避免一轮改多个变量。

**并发归属**：以 571b4e57-8944-49c9-ac0b-4d9aa8614f0a 为权威。sandbox_v4/、sandbox_v5/、logs/base_test_aim.*、evidence/base.json 属另一套脚手架，本卡产物已全部另名互不覆盖；清理另开卡，不顺手删。

**不勾第 5 项、不定为已验收**：以上全为离线回放，云台延迟/激光同轴/dy 偏置/真实 det 节奏未进模型，sim_cli 不含 AimTask。生产 aim.py 零改动（sha256 与基线一致），未部署。

**对“完赛并打中”的影响（必须让决策者看到）**：现役在 ±2 px 交替相位下是 **0 次许可**，即车到了站、两把钥匙都给了，也打不出去。这不是命中率问题，是“根本不开火”。v6 把它变成 35/35 可发。**但可发 ≠ 打中**：命中仍取决于蓝面朝向、云台延迟、激光同轴与补偿表，这些一个都没在离线里证。

## 未验证
本轮不部署、不起栈、不发车、不出光、不飞行。不为求绿放宽安全阈值/取消闩锁/伪造靶。

执行者补充（UNKNOWN，不许当成"已解决"）：
1. **车端真实抖动形状与幅度未知**：补丁按 ±2px 整型抖动设计（Q=2.0）。包线实测：±2px 内 0 假折返；**±3px 起会再出假折返（6 s 7~29 次），但仍能发许可，不再死锁**。若车端实测 ≥3px，需要把 quant_step_px 调到对应值（会进一步抬高检出下限）。
2. **折返检出下限 ≈30 px/s（0.214 m/s）**：低于它的慢折返检不出（±2px 噪声下与"没动"不可分）。D-033 的 0.3 m/s 有 1.4 倍余量；旧 0.04 m/s（5.6 px/s）口径下**必然检不出**——这正是 v4 那套 5.6 px/s 场景不该作为验收主线的原因（只能当沿革）。
3. **全部离线回放**：云台延迟、激光同轴、dy 偏置、真实 det 节奏未进模型；sim_cli 闭环**不含 AimTask**（实测 grep -c "AimTask|aim." = 0），所以 sim_cli 只能证明"没碰坏循迹"。
4. **C++ 两钥匙链路未复跑**（Lead 已证）；本卡只断言 Aim 侧 0 次写 fire_gate。
5. **test_03 的 3.2/3.3 与窗口确认语义互斥**（见 evidence/EXPECTATION_DELTA.md），改法要 owner/Lead 定，本卡没改测试文件。
6. **类别计数修正（v6）属另一个变量**：它改变"真类别切换时的锁定时延"（从 1 帧变回 N 帧），实车观感需现场确认；本卡只证明"单帧误判不再改锁"。

## 文档变更
新增本卡；树和索引由Lead单写。

执行者本轮文档改动：**只有本卡**（front matter 的 agent/owner/status/共享任务、目标里的 task ID、
台账条目、复用检索、新增轮子、验收清单、回滚点、未验证、执行记录）。未改树、未改索引、未改状态板、
未改任何 docs/ 其它篇；证据全在 _tmp_session/AGC-RECTIFY-20261026/aim/（不新建第二份长期 REPORT）。

请求 Lead 落地（我不写树/索引）：树 R-1 追加一句"v4 否决 / v5+v6 待审 + 证据路径"；
若采纳，另需在 20_资料索引库/视觉与云台/03 或 00 索引补一条"折返判据 = 窗口确认 + 噪声底"的结论。

## 经验总结
待复核后提炼，来源/结论回指项目资料库。

### 执行者补充报告 #2：独立脚手架复现 + 不触碰生产测试期望的补丁 n5（aim-analyst / task-1 · 2026-10-26 19:07–19:55）

> **本节与上面 §1–§9 是两套独立脚手架、同一张卡。** 上面那套（sandbox/src_dzbrain、candidates/、patches/aim_v5.patch、scripts/、out/）**不是我写的，我一行没改**；
> 我的产物全部另名 **n5**（sandbox_n5、evidence/patch_n5.diff、evidence/n5.json、logs/*_n5.txt）。
> 代号说明：**n5 不是上面的 v5**。两者目标相同（把假折返压到 0），判据不同，后果也不同 —— 见 §5。

#### 0. 一句话结论

独立复现了同一个缺陷，并给出一版**不需要改任何生产测试期望**的最小补丁（n5）：
现役 `test_aim` **136/136 rc=0** → n5 **仍 136/136 rc=0**；而上面 §2–§9 的 v5 是 134/136，需要按"方案 A"改 test_03 的 3.2/3.3。
也就是说，Lead 20:1x 裁决里"窗口确认 ⇒ 必须放宽 3.2/3.3"这个前提**在 n5 上不成立**：n5 的折返门限随在线估计的位置噪声 σ 缩放，
**无噪声数据上 σ=0 ⇒ 门限=0 ⇒ 保持当帧确认（老行为逐字不变）**；只在有整数噪声时自动退化为窗口确认。
若采纳 n5，3.2/3.3 不用改，也就不需要为它新增 D-xxx 台账条目。

#### 1. 复现环境（真实命令 / cwd / env / returncode）

- cwd = `D:/User/Desktop/比赛存档/v1/_tmp_session/AGC-RECTIFY-20261026/aim`；解释器 `python -B -X utf8`（Windows，UTF-8 原始输出落 logs/）。
- 沙箱：`sandbox/dzbrain`（现役逐字节副本 6.6 M；**生产目录只读**）+ `sandbox/config/robot.yaml`；`sandbox_v4/`、`sandbox_n5/` 由 [make_sandboxes.py](../../../../_tmp_session/AGC-RECTIFY-20261026/aim/make_sandboxes.py) 从同一前像构造，**每个替换断言"恰好命中一次"**，diff 见 evidence/patch_v4.diff（40 行）、evidence/patch_n5.diff（119 行）。
- 前像 sha256（生产 aim.py，与卡面"回滚点"表一致）：`7d59fd5970c494031998d8d71abca2cd2fce91c204cde892a3af6dcf67d170d2`；n5 后像 `5d0bba81d566ad932ae584a32ce9332de5719611c6b888ea228e30d1fb067872`；v4 后像 `563f85eeb2b96e1aa481f5e746a46ab8970e9229d21cd22d18cf84ea3003f3be`。
- **我实际跑过的命令与真实 returncode**（不是 tail 的退出码）：

| 命令 | rc | 证据 |
|---|---|---|
| `python -B -X utf8 sandbox/dzbrain/tests/test_aim.py` | **0** | 136/136，logs/base_test_aim.stdout.txt |
| `python -B -X utf8 sandbox_v4/dzbrain/tests/test_aim.py` | **0** | 136/136，logs/test_aim_v4.txt |
| `python -B -X utf8 sandbox_n5/dzbrain/tests/test_aim.py` | **0** | 136/136，logs/test_aim_n5.txt |
| `python -B -X utf8 probe.py --sandbox sandbox --tag base --out evidence/base.json` | **0** | 52 场景矩阵 + 7 门撤销 + 停帧，logs/base_probe.txt |
| `python -B -X utf8 probe.py --sandbox sandbox_v4 --tag v4 --out evidence/v4.json` | **0** | logs/v4_probe.txt |
| `python -B -X utf8 probe.py --sandbox sandbox_n5 --tag n5 --out evidence/n5.json` | **0** | logs/n5_probe.txt |
| `python -B -X utf8 compare.py base v4 n5` | **0** | logs/compare.txt（全表） |
| `python -B -X utf8 table.py` | **0** | evidence/table_4col.md（四列对照全表） |
| `python -B -X utf8 sandbox[&#124;_n5]/dzbrain/tests/test_aim_probe.py` | **0** | **0/0 条断言** —— 它只打印诊断，**不能当门禁**，我不用它作证据 |

探针与上面矩阵的独立性（这是"独立复现"的实质）：我自己写 [probe.py](../../../../_tmp_session/AGC-RECTIFY-20261026/aim/probe.py)，
驱动模型是 **50 Hz 任务拍 + F Hz 视觉帧**（帧 ts 落在 1/F 的精确栅格上、不吸附到 20 ms 拍，因此同一帧会被读到 1–2 次、真实走 `_ingest` 的 dedup 路径），
真值模型 = 恒速 / 三角波端点折返 / 轨道中部折返 / 静止；噪声 = **确定性整数哈希噪声**（均匀落在 [−A, +A]，A=0/1/2/3 px）；
判据只用四个可机器判的量（许可数、首个许可时刻、折返后首次 `est.armed` 时刻、`lead_px` 与真值 `v_true × lead_time` 的偏差），**不读日志措辞、不用 hold 尾巴当通过条件**。

#### 2. 已证缺陷（独立数字，与 §2 的 F1 同一条根因）

现役代码 `aim.py:399-403`（`abs(v) >= flip_min(2.0) and v*dz < 0`，dz 是**单帧**位移）+ `_seed()` 把基线重建为单样本（`aim.py:297`）⇒ 重播种后 1 px/33 ms = **30 px/s** 越门 ⇒ 判折返 ⇒ 再 `_seed`。"抖动→判折返→重播种→再抖动"的死循环。

| 场景（0.3 m/s = 42 px/s，D-033） | 现役 | 现象 |
|---|---|---|
| 静止靶 @30 fps ±1 px | **3 发 / 47 次判折返 / armed 0.060** | 车到站也基本打不出去 |
| 静止靶 @30 fps ±2 px | **2 发 / 54 次 / armed 0.016** | 同上 |
| 静止靶 @30 fps ±3 px | **0 发 / 55 次 / armed 0.000** | **完全死锁**：两把钥匙都给也打不出去 |
| 静止靶 @15 fps ±3 px | **0 发 / 26 次 / armed 0.000** | 同上（不是 30 fps 的偶发） |
| 0.3 m/s 恒速 @30 fps ±2 px | 2 发 / 17 次 / armed 0.370 | 假折返打断跟踪 |
| 0.3 m/s 端点折返 @30 fps ±2 px | 6 发（干净 9 发）/ 64 次 / armed 0.206 / 折返后重武装 **1.147 s** | 真折返与假折返混在一起 |
| 0.3 m/s 中部折返 @30 fps ±2 px | **1 发** / 25 次 / armed 0.220 | 折返点附近几乎不出许可 |

"0 发"的含义必须写清：**这不是命中率问题，是"根本不开火"** —— `fire_ready` 一直 0，C++ 那第二把钥匙给不给都一样。

#### 3. v4（只抬折返门）为什么不够 —— 用数字回答"为什么不选它"

v4 = `flip_min_eff = max(flip_min, quant_step(1.2) / max(_v_dt, dt_min))`，只在**短基线**（刚 `_seed`）时把门限抬到 36 px/s。

| 场景 | 现役 | v4 | 判读 |
|---|---|---|---|
| 静止靶 ±1 px @30 fps | 3 发 / 47 次 | 9 发 / 14 次 | 有效 |
| 静止靶 ±2 px @30 fps | 2 发 / 54 次 | **5 发 / 36 次** | 仍破 |
| 静止靶 ±3 px @30 fps | **0 发** / 55 次 | **2 发 / 46 次** | **仍近乎死锁** |
| 0.3 m/s 端点折返 ±2 px | 6 发 / 64 次 | **6 发 / 62 次** | **等于没修** |
| 0.3 m/s 中部折返 ±2 px | 1 发 / 25 次 | **1 发 / 24 次** | **等于没修** |
| 0.3 m/s 恒速 ±2 px | 2 发 / 17 次 / armed .37 | **17 次 / armed .37** | 一字不差 |

原因可复算：v4 的门限随 `_v_dt`（算 v 用的基线时长）**变大而变小**，满窗 0.40 s 时降到 1.2/0.4 = **3.0 px/s**；而假折返恰恰发生在"|v|=42 px/s、长基线已建立"的时刻 —— 42 ≫ 3.0，门限形同虚设。它只覆盖了"刚重播种那一格"，**移动靶场景完全没被覆盖**。

#### 4. n5 的最小补丁（只动 RailEstimator，不碰许可链/配置/测试）

判定逻辑本身 = 1 处 if + 3 个辅助方法；其余是写进源码的不变量注释。动了 6 处（aim.py:247 / :276 / :311 / :413 / :441 / :464 附近），**不碰** `AimTask`、`_evaluate`、`_set_fire_ready`、`config/SCHEMA`、`robot.yaml`、`tests/`。完整 diff 见 [evidence/patch_n5.diff](../../../../_tmp_session/AGC-RECTIFY-20261026/aim/evidence/patch_n5.diff)（119 行）。

1. **位置噪声 σ 在线估计（二阶差分）**：`σ ← EWMA(|z_k − 2z_{k−1} + z_{k−2}| / 1.96, α=0.25)`。
   为什么是二阶差分而不是一阶残差：**对匀速信号它是纯噪声**（与 v 无关），折返/加速不会把它污染成"高噪声"；一阶残差 `z − (x + v·dt)` 做不到（v 一有误差就整段算成噪声）。
2. **位移显著性门** `sig = 3·√2·σ`（端点差分是两个独立样本之差 ⇒ 噪声 ×√2）。样本不足 3 个 ⇒ `sig=+inf` ⇒ 不判折返、不发布速度（失败偏安全）。
3. **折返 = 反向位移超过 sig**：快路径 `v·dz<0 且 |dz|>sig`；慢路径 `v·dw<0 且 |dw|>sig`，`dw` = 最近 0.12 s 的端点位移。
   为什么窗口取 0.12 s 而不是 0.25 s（我第一版取 0.25，实测失败并改正）：窗口只需**比长基线 0.4 s 先翻号**；0.25 s 窗口在 ±2 px 噪声下要 τ≈0.19 s 才过门，而长基线恰在 τ≈0.2 s 翻号 ⇒ 两者撞在一起，**折返永远判不出来**（midrev ±1 px 实测 flips=0、v 平滑穿过 0 且无闭锁）。
4. **速度显著性门**：基线位移 `|z_last − z_first| <= sig` ⇒ `v = 0`（速度不可分辨）。宁可提前量 0（打当前位置），也不把 ±1 px/33 ms = ±30 px/s 的基线噪声外推成 10 px 提前量。

#### 5. n5 与上面 v5 的差别（决定要不要改测试）

| | 上面 §2–§9 的 v5 | 本节 n5 |
|---|---|---|
| 折返判据 | v_ref 锁存 + 0.16 s 端点差分反号（**取消**单帧当帧判据） | 单帧当帧判据**保留**，但要求"反向位移 > 3√2σ"；σ 在线估计 |
| 无噪声数据 | 当帧确认消失 ⇒ test_03 3.2/3.3 需按方案 A 放宽 | σ=0 ⇒ 门限=0 ⇒ **当帧确认逐字不变** |
| `test_aim` | 134/136（2 红） | **136/136 rc=0** |
| 生产测试/台账改动 | 需改 3.2/3.3 + 新增 D-xxx | **不需要** |
| 慢折返检出下限 | 门限 k·2Q/span ≈ 31 px/s（0.16 s 基线） | 由 σ 决定：±2 px 时约 **20~25 px/s**（见 §6 数字），±0 px 时无下限 |

结论：两者都把假折返压到 0，但 **n5 的语义变更面更小**（不去掉"当帧确认"这条既有安全语义）。若 Lead 更看重"单帧不可分"的严格论证，n5 也不冲突 —— 它在**有噪声时**同样不做当帧确认，只是把"什么时候算有噪声"交给在线估计，而不是写死。

#### 6. 四列前后对照（Lead 指定的四列）

`许可数 / 首个许可 / 折返后重武装 / lead MAE(px)`，全 52 场景见 [evidence/table_4col.md](../../../../_tmp_session/AGC-RECTIFY-20261026/aim/evidence/table_4col.md)，下面摘关键行（base / v4 / n5）：

| 场景 | 许可数 | 首个许可(s) | 折返后重武装(s) | lead MAE(px) |
|---|---|---|---|---|
| 静止靶 @30 fps ±1 px | 3 / 9 / **10** | 0.50 / 0.30 / 0.18 | — | 4.57 / 0.85 / **0.03** |
| 静止靶 @30 fps ±2 px | 2 / 5 / **9** | 0.48 / 0.44 / 0.18 | — | 4.05 / 1.70 / **0.01** |
| 静止靶 @30 fps ±3 px | **0** / 2 / **10** | — / 1.00 / 0.18 | — | — / 2.18 / **0.02** |
| 静止靶 @15 fps ±3 px | **0** / 2 / **10** | — / 2.00 / 0.34 | — | — / 1.50 / **0.11** |
| 静止靶 @10 fps ±2 px | 2 / 5 / **7** | 1.40 / 1.30 / **0.50** | — | 1.50 / 0.89 / **0.05** |
| 0.3 m/s 恒速 @30 fps ±2 px | 2 / 2 / 2 | 0.38 / 0.38 / **0.24** | — | 2.33 / 2.33 / **1.27** |
| 0.3 m/s 端点折返 @30 fps ±2 px | 6 / 6 / **9** | 0.38 / 0.38 / 0.54 | 1.15 / 1.15 / **0.007** | 3.29 / 3.36 / **2.79** |
| 0.3 m/s 端点折返 @20 fps ±2 px | 6 / 6 / **9** | 0.46 / 0.46 / 0.56 | **0.007** | 2.13 / 2.12 / 2.89 |
| 0.3 m/s 端点折返 @15 fps ±2 px | 7 / 7 / **9** | 0.48 / 0.48 / 0.48 | **0.007** | 2.20 / 2.20 / 2.52 |
| 0.3 m/s 端点折返 @10 fps ±2 px | 9 / 9 / 9 | 0.50 / 0.50 / 0.60 | **0.007** | 2.22 / 2.22 / 3.49 |
| 0.3 m/s 中部折返 @30 fps ±2 px | **1** / 1 / **3** | 0.38 / 0.38 / **0.20** | 0.0 | 2.39 / 2.39 / **1.59** |
| 0.3 m/s 端点折返 @30 fps ±0 px | 9 / 9 / 9 | 0.54 | 0.007 | 0.632（三者逐位相同） |

如实写的两处**倒退**（不是亮点，但必须写）：
1. **0.3 m/s 恒速 @10 fps ±0 px：许可 2 → 1**。原因是 σ 冷启动要攒 3 个二阶差分样本（10 fps 下 = 0.3 s），这段时间 `v=0`、提前量 0，靶穿过开火框的窗口从 0.76 s 缩到约 0.46 s，少打一发。30/20/15 fps 与 ±2 px 各档不受影响。
2. **真折返的"反向提前量时间"变长**（`wrong_dir_ticks`：tri ±2 px @30 fps 2 → 26 拍 ≈ 0.52 s/10 s）。幅度不变（旧提前量 ≤11.3 px），只是多维持约 0.2 s。见 §7 的上界论证。

#### 7. 反例：为什么"失败方向偏安全"（含一条真实反例）

- **反例（唯一方向性差异）**：静止靶 ±3 px 现役 **0 许可** → n5 **10 许可**。方向是"多发许可"，不是"冲过射击格或提前开火"：许可 ≠ 开火，`_evaluate` 里安全 → 遥控 → 目标新鲜 → 类别 → 命中锁存 → 弹药/预算 → **几何框** → 估计器 warmup → 冷却 共 9 道门都在 permit **之前**，任何一道不过都发 0；C++ 侧还要 AND 任务钥匙 + 各自租期（Lead 已复核）。
- **n5 会不会让车"提前开火"**：不会新增任何"pred 出框也放行"的路径。几何门 `|dx + lead| <= box` 未改；n5 只能把 lead 变小（显著性门把不可分辨的速度置 0）或维持，**从不放大**。
- **反号许可的实测计数**：52 场景里"许可时刻提前量方向与真值速度相反"（`wrong_dir_permits`）—— base **0**、n5 **1**。唯一一条是 `midrev @20 fps ±1 px`（**轨道中部瞬时不折返**，真实 0.70 m 轨道不产生这种运动）。
- **物理折返的安全余量（可复算）**：开火框 `fire_box_px(45)=16 px` = 0.114 m；轨道半宽 0.35 m ⇒ 折返点距框边界 **0.236 m**，0.3 m/s 需 **0.79 s** 才回到框内。n5 折返检出延迟隔离实测 **0.12–0.21 s** + warmup 5 帧 = 0.167 s ⇒ 合计 **≤0.38 s < 0.79 s**。所以物理折返场景既不会误开火、也不会错过回框后的第一发（tri 场景 n5 9 发 = 干净数据同值）。
- **总上界**：即便在"框内被人工折返"这种无轨道支撑的场景，n5 最坏也只是用旧方向提前量多瞄 ≤0.2 s，量级上限 = 提前量本身 ≤11.3 px（0.3 m/s × 0.27 s），且**打飞一发而非持续错误**（下一拍折返检出即 `_block_warmup`）。

#### 8. 直接回答 Lead 的三问

1. **probe 跑了吗**：跑了，真实 rc=0，命令/cwd/env/前后 sha256 见 §1；52 场景 × 3 版本 = 156 次运行，日志在 logs/。
2. **四列表 + 反例**：见 §6、§7。
3. **是否需要碰生产 aim.py**：**需要 —— 与 Lead 记的"不需要，只加 fail-safe 门"不一致，必须点出来。** 缺陷本体在 `aim.py:399-403`（`RailEstimator.update` 的折返判据），任何修法都落在这个函数里；n5 的 diff 只动 `RailEstimator`（6 处，119 行含注释），**不碰** `AimTask` 的许可链、不碰 config/SCHEMA、不碰 tests。生产文件我**没有**改（sha256 与基线一致），落地由 Lead 决定。

#### 9. 与另一套脚手架的交叉验证

- **一致**：都定位到同一根因（单帧 dz 判折返 + `_seed` 后短基线放大噪声），都同意"只抬折返门不够"，都覆盖 0.3 m/s / 多帧率 / ±1~2 px / 真折返 / 许可撤销。
- **我不采信也不否定的**：夜班 F2（`_update_class` 按 tick 而非按帧计数）。我在探针里确实看到"50 Hz tick 读 30 Hz 视觉 ⇒ 同一帧被喂两次"的事实（`_ingest` 只按 dt 去重），但 v6 的对照实验不在本轮 Lead 指派给我的三件事里，**标 UNKNOWN**，不勾相关验收项。
- **口径漂移（按 Lead 裁决记录，不改冻结区）**：`config/robot.yaml:846 target_speed_mps: 0.04`（评估真值 = 0.3 m/s，D-033）。影响面仅 `_v_expected = 0.04 × 140 = 5.6 px/s` 与 `max_speed_px_s = max(300, 5×5.6) = 300 px/s` 兜底；`moving_lead` 策略**不读** `_v_expected`，对 42 px/s 靶零影响。**不据此改任何生产值。**

#### 10. 未验证 / 不成立（UNKNOWN，不许当"已解决"）

1. **全部离线模型**：云台延迟、激光同轴、dy 偏置、真实检测节奏、C++ 两钥匙链路**一个都没进模型**；`sim_cli` 闭环不含 AimTask ⇒ 只能证明"没碰坏循迹"。
2. **停帧（frozen vision）是设计行为，未改**：视觉发布端停更后 Aim 仍 `armed` **0.5 s**（`VISION_MAX_AGE_S`），且停帧后 **0.2 s 内还能发 1 次许可**（三版本一致，实测 `permit_after_freeze_s=0.2`、`armed_off_s=0.5`）。0.3 m/s 下 0.2 s = 8.4 px，已到框半宽（16 px）的一半 ⇒ **建议另开卡**评估是否收紧 `vision_max_age_s`（改阈值要用户点头，本轮不动）。
3. **噪声边界**：±3 px 静止靶 n5 **0 假折返 / 10 发**（@30/15 fps），但 **@10 fps ±2 px 仍有 5 次假折返（armed 0.56、7 发）**；±4 px 以上未扫。
4. **慢折返下限**：±2 px 时约 20~25 px/s 可检出，D-033 的 42 px/s 有 1.7~2 倍余量；旧 0.04 m/s（5.6 px/s）口径下检不出（与夜班结论一致）。
5. **`test_aim_probe.py` 0/0 断言**：不是门禁，不能计入验收。
6. **验收清单第 5 项（Lead 独立复跑认可）我保持 false**（§11.2 作者不得自评）；因此**本卡 status 我保持 `in_progress`**，没有按"完成时 closed"自行关闭 —— 卡里还有未勾项（C++ 实车链路、Lead 认可），且 20:1x 裁决写明"不定为已验收"。请 Lead 复核后自行闭合。

#### 11. 纪律与回滚

生产源码**零改动**（aim.py sha256 与基线逐位一致）；沙箱内补丁每次重建都从同一前像生成并断言"恰好命中一次"；**未 commit / 未 push / 未 reset / 未 clean / 未改测试期望 / 未部署 / 未接设备 / 未起栈 / 未发车 / 未出光**。
回滚 = 丢弃 `sandbox_v4/`、`sandbox_n5/` 与 evidence/patch_*.diff；生产侧无需回滚（没动）。
本轮我只**追加**了本节，未改本卡上面任何一字，未改树/索引/台账/状态板/测试/生产代码。

#### 12. 请 Lead 决断

1. **采纳范围**：n5 独立于夜班 v5（判据不同、语义面不同）。可选 **A** = 夜班 v5+v6（要改 3.2/3.3 + 台账 D-xxx）；**B** = n5（+ 夜班 v6 的类别按帧，若 F2 复现），**不需要改任何生产测试**；**C** = 两者都先不合，等实车抖动实测。
2. **要不要我补 n5 专属回归**：本轮按"候选用例放 _tmp、不动 tests/"执行，**没有**写生产候选单测（夜班已写 candidates/test_aim_rectify.py，我不重复造）；若 Lead 要 n5 的专属回归（±1/±2/±3 px × 10/15/20/30 fps 的假折返=0 与 0.3 m/s 许可数），下一轮补 `_tmp` 候选交你审。

