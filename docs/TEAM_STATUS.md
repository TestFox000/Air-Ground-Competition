# 团队状态板（TEAM_STATUS）

> **维护**：Lead 每轮跑车 / 关键裁决 / 收工前更新；只写"此刻为真"的状态，不复述历史（历史看 `src/history/大事记.md`）。
> **2026-10-23优先级纠正（用户确认）**：PID与完整循迹尚未完成；[空地闭环计划](../../docs/10_agent提示词/04_场景/空地协同闭环推进.md)现按Lead/T循迹/P云台/E集成/R评审推进F-T、F-P双主线，空地链路只做不抢现场窗口的准备。C1三趟/曾到N5不等于完整循迹验收；R3B的C0外测/C2b仍缺，D11有效FIRE样本仍0发。B1候选47/64、171/171为离线结果，靶速以D-031=1m/s为准。F-T通过后安排运动协同，含射击整赛还须F-P与飞控/链路验收。本轮仅修订方案，未SSH/部署/改生产；以下旧摘要不当今日实时事实。
> 更新于：2026-10-22 夜 · **TOOLS-ORGANIZE 整理半归档（332 文件双哈希核），重做拆出 TOOLS-PLUGIN-REWORK-20261022（缺口 1/2/3/5/6/7 已修，deploy --dry-run 成为事实，honest-offline 503 落地；sim 家族 5 项待用户机复跑；profile 两个 file: 依赖已改指 tools/plugins/（部署债清），挂载进 bundles 等窗口）**。以下为旧注：**用户确认B/D/E三包已交回，目录D:\User\Desktop\交回包，待本地DeepSeek验包/三方合入/主副本复测，见37文§4。** 36§7阿克曼路线继续有效；R4A/R4B1为待领取建议，未实施。R3实车0趟、证据不足收口；R1候选 `e9bff6f0…8eab9b` 无新发布证据，136/136等仍为原测试。最新已回收车端身份仅截至10-20 20:55。本轮论文复核不等于三包验收，其余旧任务未重新验收。
> **10-21 20:50 补**：R3B 实车补测已完成 5 趟有效（C0×3 / C1-short / C1_1），c1_2/c1_3 因 20:45 车端整机掉电关机未跑；证据 [RUN_CARD_20261021](../../docs/history_cache/03_过程稿快照/process_TRACK-FIELD-R3_20261022.tar.gz)，未部署、未改生产。
> 回归基线：**37 套 / 13 红**（2026-10-18 21:20 干净树，不含当前工作树改动；见 [37 套审查台账](../../docs/history_cache/01_旧文档归档/dzrobot_v2/docs/全量回归37套_严格审查台账_20261018.md)，裁决 [D-023](决策台账.md)）
> **2026-10-23 18:30 团队重组（用户指令）**：常驻 5 角色 **Lead ＋ T循迹 ＋ P云台PID/打靶 ＋ E部署集成 ＋ R只读评审**；按《空地协同闭环推进计划》§2 续接既有 owner。本轮不切引擎、不重构底盘；冻结域登记后才写；R 只评审。
>
> **当前车端实况（18:28 SSH 只读）**：up 8h22m；**仅 usb_cam_node（16:06 手动 launch）在跑**，无 dzbrain/chassis/aim/det；~/dzrobot 与 ~/dzrobot2 并存（后者归属他人，不碰）；robot.yaml PID = **L 档 2.5/0.35/0（:167-169）**、thread_num: 2（:316），config/** 仍未 frozen 登记。门禁四连实跑：gen_index=0 / check_doc_policy=0（GREEN）/ check.py=0（GREEN，1 WARN：tracker.py 登记已失效）/ check_links=0。
>
> **本轮文件锁（Lead 预分配；未过卡不转写权）**：T=_tmp_session/TRACK-FIELD-*/ ＋循迹卡面；P=_tmp_session/TRACK-FIELD-D11*/、_tmp_session/aimtune/ ＋D11/AIM 卡面；E=_tmp_session/E-*/ ＋部署候选取证；R=只读，结论交 Lead 入卡；Lead=docs/TEAM_STATUS.md、docs/00_项目总览/*、docs/README.md、dzrobot_v2/docs/决策台账.md、config/**、tools/wf/frozen.json。**生产代码（tracker/mission/aim/云台 cpp）本轮一律不写**，单卡验收通过后才申请锁。


## 1. 当前在跑

| 卡 | owner | 内容 | 状态 |
|---|---|---|---|
| TRACK-HANDOFF-20261020 | Codex Lead | [循迹团队执行方案](../../docs/history_cache/01_旧文档归档/docs/agent_handoff/30_循迹团队执行方案.md)、R0 提示词、回报模板 | 文档交付完成；R0 后续结果与复核见下行；未部署/动车 |
| TRACK-R0-20261020-01 | lead（control-verifier / motion-auditor / reviewer） | R0：基线核对 + 控制限速契约离线复现 + 定位/标定采样方案 | ✅ 完成（附条件）：[REPORT](../../docs/history_cache/03_过程稿快照/process_drafts_20261022.tar.gz) · [review](../../docs/history_cache/03_过程稿快照/process_drafts_20261022.tar.gz)；13 例 PASS=9/REPRODUCED=4/INVALID=0；ALIGN/PIVOT TURN 绕过 P_MOTION_LIMIT（ALIGN 现役 bridge_blocks=[] 不可达，PIVOT 可达）；未部署/未动车 |
| TRACK-R0-REVIEW-20261020 | Codex Lead | [R0 复核与 R1 任务](../../docs/history_cache/01_旧文档归档/docs/agent_handoff/33_循迹R0复核与R1任务.md) | 已交付并由R1接续完成；R0 控制缺陷证据保留，标定草案退回重审；不重复派工 |
| TRACK-FIELD-PLAN-20261020 | Codex Lead | [实车基线细则](../../docs/history_cache/01_旧文档归档/docs/agent_handoff/34_实车基线执行任务.md) | 已交付；现场结果由R2承接，当前按36补测；34只作通用接口参考 |
| TRACK-R1-20261020-01 | lead（implementer / control-verifier / reviewer） | R1：Tracker 最终发布限速统一钳位 + 离线回归 | ✅ 本地完成：[REPORT](../../docs/history_cache/03_过程稿快照/process_drafts_20261022.tar.gz)；独立探针13/13、tracker136/136、pivot55/55；7套权限中断已补证，75条可比失败与10-18同ID同值，planner仍历史异常中断（详见D-025）；冻结登记保留至2026-10-27；车端安装待现场核验 |
| TRACK-R1-REVIEW-20261020 | Codex Lead | [R1独立复核](../../docs/history_cache/03_过程稿快照/process_drafts_20261022.tar.gz)、[R2派工](../../docs/history_cache/01_旧文档归档/docs/agent_handoff/35_R1验收与R2实车派工.md) | 本地验证及派工已交付；原MANIFEST316/316；未改生产、未SSH、未部署/动车 |
| TRACK-FIELD-R2-20261020-01 | lead（data-auditor / tracking-auditor / reviewer） | R2原始现场证据 | ◐ 报告收口、场测部分完成：[REPORT](../../docs/history_cache/03_过程稿快照/process_drafts_20261022.tar.gz)；F1b双路约50Hz、漂移约0.29°/59.58s，loc/pursuit空；F2一趟raw-loc净0.6961m、内部横偏5.24mm，非外测；F3共线角点起步、内部轨迹进K10，不能算普通单弯；外测/停车/重复待补 |
| TRACK-R2-REVIEW-20261020 | Codex Lead | [R2复核](../../docs/history_cache/03_过程稿快照/process_drafts_20261022.tar.gz)、[六模块派工/进度与工期](../../docs/history_cache/01_旧文档归档/docs/agent_handoff/36_R2复核_R3补测与收尾计划.md) | 只读复核与派工；纠正K10/K带、raw吸附、FINISH/初始化及旧参数；8–10人并行、明确四引擎/仿真覆盖边界；未连车/部署/动车 |
| TRACK-FIELD-R3-20261020-01 | DP Lead | 静态身份/配置/脚本核验、R2原件回收 | 已收口，实车0趟、证据不足；见原REPORT，不记实车通过，不再续§6.6旧调度 |
| TRACK-FIELD-R3B-20261021 | lead | R3B 实车基线补测（C0 / C1-short / C1） | ◐ 5 趟有效：C0 接管停距 22.0/24.2/23.3 mm（判据 34.2<35 mm）；C1-short 到点滑行 17.4 mm、停于 M10 前 3.9 mm；C1_1 全程 0.6917 m（−8.3 mm）、横偏 13.7 mm、滑行 17.4 mm。c1_2/c1_3 未跑（20:45 车端关机）。未部署、未改生产 |
| TRACK-PLAN-B1-20261021 | lead（implementer） | T-B1 目标函数换口径（任务优先+降级标记）+ 第七图双停站；64 图 ≥70%、无解=0、6 条真缺陷改代码 | ◐ 已派工（本地离线）；基线 28/64；planner.py 冻结登记至 2026-11-04 |
| TRACK-FIELD-B5-20261021 | lead（motion-auditor） | T-B5 前置：阿克曼量测作业包 + 航向守卫离线复算（10-18/10-19）+ X05 电流分诊 | ◐ 已派工（离线准备，待车实测） |
| TRACK-DEPLOY-R3-20261020-01 | 第3位成员/E组（具体owner以交回卡为准） | 唯一部署入口窄推、备份、失败传播及回滚 | 用户确认已交回；待验包/合入/复测，尚未宣称工具通过或发布完成 |
| TRACK-DEPLOY-PREP-20261020-01 | DP Lead | R1发布候选/保护项/门禁清单 | 准备完成；现可接E包做主副本复测与失败注入，最终由人执行；不是已发布 |
| B/D原模块交回 | 第1/2位成员（以包内卡为准） | tour单停站、sim_cli地图接入 | 用户确认已交回；主副本按37文核实际范围和基线，不整目录覆盖 |
| TRACK-PAPER-MERGE-20261021 | Codex Lead | 论文方案适用性复核、三包集成提示词 | 文档交付；包根目录已填，未逐包核验/合入，未改生产/SSH/部署 |
| TRACK-ACK-PLAN-20261021 | Codex Lead | 手册与代码复核、36§7下一步派工、状态纠正 | 文档交付，验收见本卡REPORT；未改生产/未SSH/未部署/动车 |
| R4A / R4B1（建议卡） | 待领取，不视为已开工 | 阿克曼航向守卫；abandon不得作为到点依据 | 可在主副本按不同文件并行；领取/登记/修前复现后实施，详见36§7.2 |
| TRACK-DISPATCH-20261020 | Codex Lead | 36第6节：当前DeepSeek续A/C/F，其他组员领B/D/E；接单与异机交付契约 | 派工消息已编制，尚未代用户发送或收到接单回执；R3/R3-D领取状态仍以上述实际记录为准 |
| TRACK-DISPATCH-COPY-20261020 | Codex Lead | 36第6节改为独立副本开发/改动包/主副本合并；R3首报只读复核 | 历史交付，独立副本契约仍有效；R3续接已结束，当前派工见36§7；未改生产/未SSH/部署 |
| T12 | drone-arch | 无人机上线执行单（飞控断电恢复→看门狗→118 切换） | in_progress（离线） |
| T34 | drone-integrator | 机端 target vs 车端 stop_kind 逐段复核 | ✅ 完成：**11/11 一致**；射击/实车两层缺证据 |
| TOOLS-PLUGIN-REWORK-20261022 | lead | 统一空地插件重做（缺口销账 + 测试基建 + honest-offline） | in_progress：核心 8 自检绿（含无车 honest-offline 3 新断言）；sim 5 项沙箱 EPERM 待用户机复跑；profile file: 依赖已改指 tools/plugins/，挂载进 bundles 需用户窗口 |
| T35 | verifier | test_autoplan 红分类 | ✅ 完成：**23 条唯一失败**（46 行=每条印两次）→ **17 条旧图/契约过期（纯测试重钉）+ 6 条真缺陷（tour 单停站，必须改代码，禁止重钉）**；6 条见 `_tmp_session/t35/REPORT.md` §2.3/§4；处置 D-023 |

## 2. 阻塞 / 等用户

| 项 | 等什么 | 影响 |
|---|---|---|
| 下次实车采样 | 今天的操作者/窗口待确认，10-20的1–2小时不顺延；接管停车、净空、外测与初态齐备 | 按36§7.4另起人工停车/直线/左右短弧量测组；无需等B/D全部改完，不自动启动旧PIVOT |
| R1窄部署 | 已交回E包待验；真实CLI、6类失败验证/恢复及现场执行人 | R3已经结束；E通过后R1可独立发布，不等B/D，不用旧整目录push绕过 |
| 阿克曼一致性 | 守卫假设、PIVOT放弃/到点、连续可执行连接与全车扫掠 | 本地两张小卡先并行；普通PP/sim复用，几何与有效外测齐备后再排自动单弯 |
| 赛时 GO Gate | **已更正**（D-020）：Go 条件=首段可靠，全场 layout 运行中完成（约 37.1 s）；待实测相机/航速/车速复算 + 段边界等待政策（D-015 ③） | race_manifest 字段与段长定稿 |
| hybrid P2 | D-016：执行层全绿后再开 | 未阻塞任何当前工作 |
| **随机图求解率 ≈44%** | D-018：结构约束修复（穿洞 2/3 次、无射击停站） | 赛图若随机/临时改布设 → 一半概率无解 |
| **13 套红未收口** | Q-101 已裁决（D-023）：D 类 3 套改 tour / C 类按第六图重钉 / A 类 3 项开卡 | 全量回归维持 **37 套 / 13 红**；任何「全绿」声明先核对基线树 |

## 3. 冻结区占用

**2026-10-20 R1复核：登记1条，冻结区脏文件1个。** `src/dzbrain/dzbrain/tasks/tracker.py` 属 `TRACK-R1-20261020-01`，owner=`implementer`，expires=`2026-10-27`；离线完成不自动清除此登记。门禁输出见复核报告；GREEN只说明工作流符合要求，不是实车通过。

## 4. 最近提交（10 条内）

```
f50947e ledger: D-017 双门时间轴(系统自检10s + 全场layout后Go) + D-014修订
783cce2 docs(agents): 并入团队协作/防重复规则并精简 AGENTS.md
8cc1589 ledger: D-016 hybrid P2 暂缓 + 下次先 A 级离线对比
41ab3a3 wf: 复用门禁(新增未登记=红,变异验证) + 能力索引(能力→唯一实现)
9381a52 wf: 链接检查并入收口门禁(断链=红,变异验证) + 现状报告入库
12ebc54 docs: 整理技术文档与历史记录——39 篇历史报告归档 docs/archive/20261018
78196e8 steploop: D1 实车验证通过(6段自动推进) + D1b/D2/D3 新发现 + 本轮日志/脚本
f51edc6 docs: 归档 20261018 旧文档到 docs/archive/20261018/（0 内容改动）
fbc03b2 docs: 车端更新补记——src/ 根误建包已备份移除，dzbrain+dzrknn 重编成功
723e899 docs(D-013): hybrid 引擎实验选项裁决（默认 tour 不变）
```

## 5. 待办（按优先级，Lead 视角）

**本轮循迹优先项（2026-10-21）**：按36§7先做R4A/R4B1离线候选、地面量测与E包验收；B/D原成果分包交回后接连续轨迹/扫掠。以下D1b/D2/D3及T35旧问题继续保留，不因新的硬件证据被记为已解决。

1. **D1b 复验**：当前代码已有完成单闩锁、单调 `done_seq` 和重发；复验消费端与实车段接续，不重做实现；
2. **D2**：`belief 差一格` 属定位/投影（T34 同类），先修投影，不要用放宽判据掩盖；
3. **D3**：射击停站 发0/中0 —— 单独立卡，先抓 30 s "靶在框内但 fire_ready=0" 数据；
4. **T35 后续**：17 条旧图/契约过期按第六图重钉；**6 条真缺陷（tour 单停站）必须改代码，禁止重钉**（D-023）；
5. **结构约束修复**（D-018）：随机图 44% 求解率的主因是"穿洞 2/3 次 + 无射击停站"，属规划器结构层，不是 DP 顺序；修完重测 64 图；
6. 赛规两问（GO Gate）确认后出 `race_manifest`。

### 无人机侧待修（审阅发现，docs/审查_无人机飞控执行侧_20261018.md）

- **P1-2**（一行修）：`path_follow_waypoint_node.cpp` 按键 4 时若 INIT 先 reload —— **T12 验收路径会踩**；
- **P1-1**（安全）：`on_goto` 加 z/半径/单帧位移边界 —— **任何飞行前必做**；
- **P1-3**：过点判据加投影越过 + 超时；
- **P1-4**：相机断流 busy-loop —— **若用无人机相机侦察，先修**。

### 低优先级（赛前不动，done 后再说）

7. 工具 legacy 化：`global_planner.py`/`e2e_monitor.py`/`p0_verify.py`/`test_rknn.py` 无引用 → `tools/legacy/`（`deploy_car.py` 按 D-019 保留为只读诊断）；
8. 测试夹具 `tests/_fixtures/`（29 套测试迁移需单独卡）；
9. 月度指标脚本 `tools/wf/metrics.py`。

## 6. 现场状态快照

- **2026-10-21 21:22 T-B5 复核 + 10-19 日志找回**：task-8 完成并复核（MANIFEST 17 文件全哈希 0 mismatch、replay_guard 复跑 exit 0、check.py GREEN）；车端 `robot_logs` 找到 10-19 P1 原始日志（`3_loc_p1.log` 16122 帧 5 Hz 带时间戳、`7_path_p1.log` 含 `yaw 111.1°/s > 60（v=0.06）` 原文），已拉回 `_tmp_session/TRACK-FIELD-B5-20261021/car_pull_1019/`；A2 79.8 原始帧本地/车端均缺。T-B1/T0/T3/T4 在跑。
- **2026-10-21 21:35 靶形态澄清**：用户确认 1:1 圆面在 35×70 板上、不转不翻、轨道 0.70 m 水平横移；与 `robot.yaml:830-840` 一致。按配置 4 cm/s，提前量仅 ≈1.5 px，D3 重点转向开火门/几何；靶速需现场用“跑完 0.70 m 几秒”复核（10-17 有“提速”记录，二者需对齐）。

- **2026-10-21 20:40 R3B 复采**：tracker src/install=`9e9766a1…`、robot=`edff2921…`、map=`b5629d05…`、start_all=`1da921ba…`、dzros=`7802e6ce…`，与 10-20 一致；实验配置 `robot_r3.yaml`=`e928ce0f…`（仅 cruise 0.10）。预检 4 次 exit 0。**20:45 整机掉电关机**（sshd 「System is going down」），c1_2/c1_3 未跑；车恢复后先复采身份再续。另：`/odom` 与 `/odom/localization` 位移比 2.7–2.84×（后者起点恒 (1.5,0.5)），本卡测距只用 `/odom`。
- **最新已回收身份为2026-10-20 20:55**：tracker src/install=9e9766a1…、robot=edff2921…、map=b5629d05…、start_all=1da921ba…、dzros=7802e6ce…；R3最终REPORT实车0趟。10-21仅本地手册/代码复核，未SSH；当前车位/进程/版本需新采样，不能把旧快照说成此刻状态。
- **2026-10-20 19:57 R2报告快照（附条件）**：报告称车端tracker src+install为R1前`9e9766a1…1abb2`、现役配置/map未改，但安装副本身份/差异/脚本/bag原件未完整落本地，下一卡补证。实际取得故障态与干净静止、F2一趟直线、F3共线角点起步越界观察，普通单弯及重复未完成。报告称19:57:31全栈停、车在L8附近；不当作下一卡当前实物位置。
- **2026-10-20 18:03 只读采样**：`ssh duzhong@duzhong.local` 可访问；六个核心源码与本地哈希一致。按名称筛选未匹配到循迹/底盘等进程，仅代表一次采样；物理位置与当前场地未观察。
- 车端配置文件：`planner.engine=tracker-safe`、`auto_plan_enabled=true`、`fallback_to_tokens=true`、`footprint_gate=warn`；`mission.auto_start=false`。与 D-012 的 tour 口径差异待核实验归属；本轮未切换或修改，文件值不等于进程实际加载值。
- 最新 `7_path.log:71` 有静止 yaw 异常；用户说明架空测试过或不确定，不能据此归因为传感器坏。本轮未起栈、未发布运动、未实测轨迹。
- 无人机本轮未连接，运行状态未验证。
- 证据：[只读快照](../artifacts/tracking_readonly_20261020.json)；后续：[循迹团队执行方案](../../docs/history_cache/01_旧文档归档/docs/agent_handoff/30_循迹团队执行方案.md)。旧 E4/E5 停位和 auto_plan=false 仅是上次快照，不能继续当当前事实。

## 7. 里程碑消息模板（发给 Lead / 全员）

- **① 上线 + 预检**：车 IP/连通性（ping+ssh）、main.py md5、auto_plan、route_check RC、残留进程；异常项原文贴回。
- **② 发车 + 首段**：路线 token 数/穿洞格、首段位姿与 v、订阅者（/odom/localization）、日志横幅；危险动作先 stop 再报。
- **③ 完赛 / 失败 + 摘要**：到达点/雷覆盖/停站/射击/用时/异常原因；日志与数据落盘路径；未证实项单列。

## 8. 文档库指标（2026-10-23 17:53 复核 · 实跑口径）

> 来源：`python docs/_工具/check_doc_policy.py`、`python tools/wf/check_links.py`、`docs/目录索引.json` 实跑结果，非估数。

| 指标 | 值 | 备注 |
|---|---:|---|
| 库内篇数（机器计） | **86** | 含 docs/README.md 与 目录索引.md；20 库 41 篇 / 任务卡库 16 / 总览·提示词若干 |
| 库外受管 .md | **65** | 基线只减不增；本轮移走 `src/todo/` 3 篇 + 重跑 `--write-baseline` 清 4 条失效项 |
| 红线 | **0** | R1/R2/R3 全清 |
| 提醒 | **0** | 已 `--write-baseline` 清掉 4 条 Y4 失效项 |
| 过期候选（现行且 >180 天） | **0** | 最旧现行 10-21 |
| 断链 | **0** | 94 文件 |
| 索引未登记 | 0 | 仅 README/目录索引不登记（设计如此） |
| 幽灵索引条目 | 0 | 索引指向的文件全部存在 |
| 册级索引覆盖 | 100% | 20 库 6 册全列 |
| 只读 / 豁免 / 第三方 | 119 / 1496 / 101 | 豁免含 history_cache、_tmp_session、git_save 等 |
| 10-23 新增 | **8 篇** | 历史/04、历史/05、历史/06、车端/08、车端/09、空地链路/03、赛事与治理/06、赛事与治理/07 |

