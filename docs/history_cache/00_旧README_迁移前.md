# 旧文归档（_archive）

> **一句话**：这里只放**迁移前的旧长期文档原文**，按「仓库相对原路径」原样保留、**只读、不再维护**；现行内容一律以 `docs/00_项目总览`、`docs/10_agent提示词`、`docs/20_资料索引库`、`docs/30_任务卡库` 为准。

## 1. 统计

| 项 | 值 |
|---|---|
| 归档篇数 | **118** |
| 总字节 | **5,184,331**（≈ 4.94 MiB） |
| 文件类型 | .md 113 · .zip 3 · .txt 1 · .json.bak 1 |
| 哈希核验 | manifest **118 行「一致」全 TRUE**，0 条不符 |
| 归档时间 | 2026-10-22 |
| 卡号 | task-1（owner: archivist） |
| 记账清单 | [archive_manifest.csv](../_work/archive_manifest.csv)（逐文件原/归档 sha256） |

### 5 个来源各自篇数

| # | 来源 | 原路径 | 篇数 | 归档位置 |
|---|---|---|---|---|
| 1 | 根 docs/ 顶层旧文 | `docs/*.md`、`docs/*.txt` | 8 | `docs/_archive/docs/` |
| 2 | agent 交接包 | `docs/agent_handoff/**` | 18 | `docs/_archive/docs/agent_handoff/**` |
| 3 | 插件整合施工记录 | `docs/merge/**` | 11 | `docs/_archive/docs/merge/**` |
| 4 | 车端旧文档 | `dzrobot_v2/docs/**`（81 篇 − 8 个门禁活文件） | 73 | `docs/_archive/dzrobot_v2/docs/**` |
| 5 | 无人机旧文档 | `dzsmartscout/docs/**` | 8 | `docs/_archive/dzsmartscout/docs/**` |
| | **合计** | | **118** | |

子目录分布（供点验）：根 docs 8 · agent_handoff 12 + packs 6 · merge 11 · dzrobot_v2/docs 根 24 + library 5 + agent_archive 3 + archive 1 + archive/20261018 40 · dzsmartscout 3 + drone 5。

### 与任务书数字的关系（必读，别被「正好 118」骗了）

任务书要求归档 **118** 篇，本表也是 **118** —— 数字相同，但**构成不同，属巧合抵消**：

| 项 | 任务书口径 | 实测口径 | 差 |
|---|---|---|---|
| `dzrobot_v2/docs/**` 总篇数 | 79 | **81** | +2 |
| 必须留在原位的例外 | 6 个 | **8 个** | +2 |
| 该归档篇数 | 79 − 6 = 73 | **81 − 8 = 73** | **0** |
| 合计 | 8+18+11+73+8 = 118 | 8+18+11+73+8 = **118** | 0 |

- 总篇数差的 2 篇全在 `dzrobot_v2/docs` **根目录**（任务书按 79 反推根=30，实测 32）；
- 例外从 6 变 8 的依据是 2026-10-22 更新后的 `AGENTS.md §8.1`：新增 `工作流v2.md`（`tools/wf/check.py:31-35` REQUIRED_DOCS）与 `索引标注.yaml`（`tools/wf/repo_index.py:13` 注解源）。
- 明细见 [archive_unresolved.md](../_work/archive_unresolved.md) 的 U-01 / U-02。

## 2. 映射表（118 行）

| 原路径 | 归档路径 | 取代者 | 一句话内容 |
|---|---|---|---|
| `docs/云台PID自动整定方案.md` | `docs/_archive/docs/云台PID自动整定方案.md` | docs/20_资料索引库/视觉与云台/00_索引.md（册级声明；场景版见 docs/10_agent提示词/04_场景/云台瞄准调参.md） | 云台自瞄 PID 三段式整定：先辨识 a=px/tick 再算 Kp 上界，最后闭环验收；脚本就绪但未实车。 |
| `docs/交接_无人机飞行分析_agent技术文档.md` | `docs/_archive/docs/交接_无人机飞行分析_agent技术文档.md` | docs/20_资料索引库/无人机/00_索引.md（同主题分册；无点名取代） | 无人机飞行数据分析的 agent 交接：手动飞 + 录 rosbag/ULog，做「摇杆→指令→电机」闭环，产飞行指纹卡。 |
| `docs/交接_无人机飞行分析_人类操作手册.txt` | `docs/_archive/docs/交接_无人机飞行分析_人类操作手册.txt` | docs/20_资料索引库/无人机/00_索引.md（同主题分册；无点名取代） | 同一件事的人类侧手册：你负责飞和拧旋钮，电脑负责看数据出诊断，含分工表与飞行科目。 |
| `docs/感知可行性_摄像头评估.md` | `docs/_archive/docs/感知可行性_摄像头评估.md` | docs/20_资料索引库/视觉与云台/02_视觉模型与检测.md | 10-12 对 Jetson 只读体检，评估无人机摄像头现状与感知链可行性（含 trtexec 旁证）。 |
| `docs/瞄准代码对比报告_gitee_vs_本地_20261018.md` | `docs/_archive/docs/瞄准代码对比报告_gitee_vs_本地_20261018.md` | docs/20_资料索引库/视觉与云台/00_索引.md（册级声明） | gitee 最新码 vs 本地云台/开火代码：差异只有 1 个文件 16 行，本地多一条「类别锁跨短暂丢失保留」修复。 |
| `docs/空地联合仿真设计.md` | `docs/_archive/docs/空地联合仿真设计.md` | docs/20_资料索引库/空地链路/02_链路联调与已知缺口.md（§8）；计分口径并见 docs/20_资料索引库/赛事与治理/02_计分与时间预算.md | 空地联合仿真（t11）：18 臂 × 360 场统计、参数口径、时间预算 §7~§9，交付 dzsmartscout/tools/sim_airground。 |
| `docs/第三趟实车验证方案_射击链.md` | `docs/_archive/docs/第三趟实车验证方案_射击链.md` | docs/20_资料索引库/视觉与云台/03_移动靶缺口与改进方向.md | 第三趟实车射击链验证方案（10-15）：扫描速度降到 0.30，逐条给「期望日志串→通过/否→现场动作」判据。 |
| `docs/项目总览_AGENT_20261020.md` | `docs/_archive/docs/项目总览_AGENT_20261020.md` | docs/00_项目总览/项目总览.md（取代）；§2~§7 并见 docs/00_项目总览/系统架构.md | 10-20 17:40 的 agent 交接总览（477 行）：30 秒读懂、场地硬约束、各线现状，全部带〔实〕〔档〕〔未〕标记。 |
| `docs/agent_handoff/00_一页纸.md` | `docs/_archive/docs/agent_handoff/00_一页纸.md` | docs/10_agent提示词/00_一页纸.md | L0 30 秒项目卡：场地 4×5 m / 14×11 / 35 cm、3 分钟、压雷8避障8打靶穿2洞到 N5，可直接贴进提示词。 |
| `docs/agent_handoff/10_提示词模板.md` | `docs/_archive/docs/agent_handoff/10_提示词模板.md` | docs/10_agent提示词/01_提示词模板.md | 6 套可直接复制的提示词（速览/接管/单卡/审查/子智能体/小模型）+ 五段骨架 + 验收 5 问。 |
| `docs/agent_handoff/20_发送指南.md` | `docs/_archive/docs/agent_handoff/20_发送指南.md` | docs/10_agent提示词/02_发送指南.md | 发什么、走哪条通道：能同机读盘就别发文件；L1 七个文件≈3.3 万 tokens，L2 仅同机/内网，含脱敏红线。 |
| `docs/agent_handoff/30_循迹团队执行方案.md` | `docs/_archive/docs/agent_handoff/30_循迹团队执行方案.md` | docs/10_agent提示词/04_场景/离线回归攻坚.md（§4/§5 来源） | 循迹团队 R0~R3 路线图与写域划分（R1 已离线通过），含证据等级要求与「不重新派发整轮」声明。 |
| `docs/agent_handoff/31_循迹首轮启动提示词.md` | `docs/_archive/docs/agent_handoff/31_循迹首轮启动提示词.md` | docs/10_agent提示词/README.md（§历史，勿用） | R0 首轮启动提示词：只执行 R0、交报告即停，不得自动进 R1，附必读顺序与工作区绝对路径。 |
| `docs/agent_handoff/32_循迹回报模板.md` | `docs/_archive/docs/agent_handoff/32_循迹回报模板.md` | docs/10_agent提示词/05_回报模板.md | 每轮 Lead 一份 REPORT.md + Reviewer 一份 review.md 的回报/续轮模板，可转交摘要约 700 字。 |
| `docs/agent_handoff/33_循迹R0复核与R1任务.md` | `docs/_archive/docs/agent_handoff/33_循迹R0复核与R1任务.md` | docs/10_agent提示词/README.md（§历史，勿用）；来源并见 docs/10_agent提示词/03_角色/implementer.md | 10-20 Codex 复核 R0：最终发布限速缺陷成立、可派 R1；标定草案尚不可执行。 |
| `docs/agent_handoff/34_实车基线执行任务.md` | `docs/_archive/docs/agent_handoff/34_实车基线执行任务.md` | docs/10_agent提示词/04_场景/现场实车执行.md | 实车基线现场操作与采样细则：先取直线/单弯/停车数据，第一次现场不动算法、不改标定。 |
| `docs/agent_handoff/35_R1验收与R2实车派工.md` | `docs/_archive/docs/agent_handoff/35_R1验收与R2实车派工.md` | docs/10_agent提示词/README.md（§历史，勿用） | R1 验收结论（316 文件哈希全对、13 PASS、136/136、55/55）与 R2 派工边界、版本不混算要求。 |
| `docs/agent_handoff/36_R2复核_R3补测与收尾计划.md` | `docs/_archive/docs/agent_handoff/36_R2复核_R3补测与收尾计划.md` | docs/10_agent提示词/04_场景/离线回归攻坚.md（§0.2 来源） | R2 复核 / R3 补测与收尾：六个可独立交付模块、交回契约、阿克曼纠偏入口（§7）。 |
| `docs/agent_handoff/37_论文方案取舍与三包集成.md` | `docs/_archive/docs/agent_handoff/37_论文方案取舍与三包集成.md` | docs/10_agent提示词/04_场景/云台瞄准调参.md（§1/§2 来源） | 50 篇论文取舍：采方案一精简工程版为循迹主线，先验三包再接阿克曼纠偏，不动 D-012/016/019/023/027。 |
| `docs/agent_handoff/packs/L0_MANIFEST.md` | `docs/_archive/docs/agent_handoff/packs/L0_MANIFEST.md` | docs/10_agent提示词/02_发送指南.md（数字来源） | L0 速览包清单：1 文件、2,951 字符≈1,736 tokens、脱敏 0 处，逐文件 sha256。 |
| `docs/agent_handoff/packs/L0_速览包.zip` | `docs/_archive/docs/agent_handoff/packs/L0_速览包.zip` | docs/10_agent提示词/02_发送指南.md（包本体） | L0 速览包压缩体（4,463 B），内含 00_先读我.md。 |
| `docs/agent_handoff/packs/L1_MANIFEST.md` | `docs/_archive/docs/agent_handoff/packs/L1_MANIFEST.md` | docs/10_agent提示词/02_发送指南.md（数字来源） | L1 接管包清单：7 文件、56,331 字符≈33,138 tokens、脱敏 0 处。 |
| `docs/agent_handoff/packs/L1_接管包.zip` | `docs/_archive/docs/agent_handoff/packs/L1_接管包.zip` | docs/10_agent提示词/02_发送指南.md（包本体） | L1 接管包压缩体（50,049 B，7 文件）。 |
| `docs/agent_handoff/packs/L2_MANIFEST.md` | `docs/_archive/docs/agent_handoff/packs/L2_MANIFEST.md` | docs/10_agent提示词/02_发送指南.md（数字来源） | L2 代码包清单：682 文件、7.72 MB≈222 万 tokens、脱敏 7 处，明确警告勿整包入上下文。 |
| `docs/agent_handoff/packs/L2_代码包.zip` | `docs/_archive/docs/agent_handoff/packs/L2_代码包.zip` | docs/10_agent提示词/02_发送指南.md（包本体） | L2 代码包压缩体（3,028,642 B，682 文件）。 |
| `docs/agent_handoff/README.md` | `docs/_archive/docs/agent_handoff/README.md` | docs/10_agent提示词/README.md | agent 交接目录入口：四档材料用途表，先读 20_发送指南 §1 决策树再决定发什么。 |
| `docs/merge/capability-matrix.md` | `docs/_archive/docs/merge/capability-matrix.md` | docs/20_资料索引库/历史/02_并入与改名_20261018.md（同批提炼） | t1 双侧 Host 能力盘点与合并裁决矩阵：车侧插件 32 路由 vs 机侧 20 路由。 |
| `docs/merge/coordination-design.md` | `docs/_archive/docs/merge/coordination-design.md` | docs/20_资料索引库/历史/02_并入与改名_20261018.md（同批提炼） | t4 统一数据模型与面板设计：应该是「三条链路各一条状态轴」，不是两节点各一张卡。 |
| `docs/merge/drift-audit.md` | `docs/_archive/docs/merge/drift-audit.md` | docs/20_资料索引库/历史/02_并入与改名_20261018.md（同批提炼） | t2 机侧插件漂移审计：车侧有 git（HEAD 0264159）、机侧裸目录无版本控制。 |
| `docs/merge/regression-matrix.md` | `docs/_archive/docs/merge/regression-matrix.md` | docs/20_资料索引库/历史/02_并入与改名_20261018.md（同批提炼） | t3 回归测试矩阵：只有 2 条测试会因改名结构性碎（test-host/test-client），其余不碰标识符。 |
| `docs/merge/S0-冻结记录.md` | `docs/_archive/docs/merge/S0-冻结记录.md` | docs/20_资料索引库/历史/02_并入与改名_20261018.md（S0~S2 来源） | S0 冻结：三个回滚点就位并逐字节校验（missing=0/diff=0），用户决议 ⑦=B-1。 |
| `docs/merge/S1-施工记录.md` | `docs/_archive/docs/merge/S1-施工记录.md` | docs/20_资料索引库/历史/02_并入与改名_20261018.md（S0~S2 来源） | S1 骨架施工：新包 dsh-dzairground-ssh 107 文件，89 文件与车侧基线逐字节相同。 |
| `docs/merge/S2-施工记录.md` | `docs/_archive/docs/merge/S2-施工记录.md` | docs/20_资料索引库/历史/02_并入与改名_20261018.md（S0~S2 来源） | S2/S3 施工：无人机节点接入 + 面板重构，12 文件改 + 4 文件新增。 |
| `docs/merge/S5-profile-package.json.bak` | `docs/_archive/docs/merge/S5-profile-package.json.bak` | docs/20_资料索引库/历史/02_并入与改名_20261018.md（S5 回退证据） | S5 切换前的 desktop profile package.json 备份，是「S5 失败回退」的原始回滚证据。 |
| `docs/merge/S5-切换记录.md` | `docs/_archive/docs/merge/S5-切换记录.md` | docs/20_资料索引库/历史/02_并入与改名_20261018.md（S5 来源） | S5 切换上线：增量安装新包、旧两包并存，唯一改动点是 profile 的 dependencies。 |
| `docs/merge/S6-改名记录.md` | `docs/_archive/docs/merge/S6-改名记录.md` | docs/20_资料索引库/历史/02_并入与改名_20261018.md（S6 来源） | S6 命名空间隔离：dzr_→dzag_ 共 412 处，版本 0.6.0-s2→0.7.0-s6。 |
| `docs/merge/空地协同插件整合方案.md` | `docs/_archive/docs/merge/空地协同插件整合方案.md` | docs/20_资料索引库/历史/02_并入与改名_20261018.md（原文 48 KB，此处提炼） | 讨论稿 v1：两个插件合并成一包/一侧栏/一套路由/一份 noVNC，车与机降为同包两个节点。 |
| `dzrobot_v2/docs/agent_archive.md` | `docs/_archive/dzrobot_v2/docs/agent_archive.md` | docs/20_资料索引库/历史/01_沿革_20260927_20261012.md（同主题提炼） | agent 视角项目档案索引：vN-主题-日期 版本号规范，指向 agent_archive/ 下各版旧档。 |
| `dzrobot_v2/docs/README.md` | `docs/_archive/dzrobot_v2/docs/README.md` | docs/20_资料索引库/车端/00_索引.md（入口职能） | 车端文档总览（10-18 整理）：五分钟上手顺序、该读哪几篇、哪篇是当前真源。 |
| `dzrobot_v2/docs/step_loop.md` | `docs/_archive/dzrobot_v2/docs/step_loop.md` | docs/20_资料索引库/空地链路/01_协议v2与契约.md（§2 来源） | 步进循环协议 v2 设计稿：无人机给 known 布局与目标格，小车逐段走并回报，安全由车兜。 |
| `dzrobot_v2/docs/StepLoop_实车第二轮_20261018.md` | `docs/_archive/dzrobot_v2/docs/StepLoop_实车第二轮_20261018.md` | docs/20_资料索引库/空地链路/02_链路联调与已知缺口.md（同主题） | StepLoop 二跑（10-18 20:21）：D1 修好连跑 6 段，暴露 D1b 窗口太短漏收、D2 差半格、D3 发0/中0。 |
| `dzrobot_v2/docs/StepLoop_实车首跑_20261018.md` | `docs/_archive/dzrobot_v2/docs/StepLoop_实车首跑_20261018.md` | docs/20_资料索引库/空地链路/02_链路联调与已知缺口.md（同主题） | StepLoop 首跑（10-18 20:00）：4 段真实行驶、安全层全程 OK，段末交接与停站开火两处真缺陷。 |
| `dzrobot_v2/docs/版本号台账.md` | `docs/_archive/dzrobot_v2/docs/版本号台账.md` | 仅存档（无直接取代者） | git tag↔提交↔主题 的唯一对照表，只增不改，含「下次版本号」口径。 |
| `dzrobot_v2/docs/发车门禁口径.md` | `docs/_archive/dzrobot_v2/docs/发车门禁口径.md` | docs/10_agent提示词/04_场景/现场实车执行.md（§4/§5 来源） | 发车门禁口径 v4（10-17）：结论只对 HEAD 0264159 + 指定 robot.yaml md5 成立，44 token/双停站判据。 |
| `dzrobot_v2/docs/仿真台_插件交付.md` | `docs/_archive/dzrobot_v2/docs/仿真台_插件交付.md` | 仅存档（无直接取代者） | 插件内仿真台交付（t21/t25）：一键跑仿真、横置场地视图、可改点位，不连车不写 config。 |
| `dzrobot_v2/docs/黑图探索与动态规划_决策层方案.md` | `docs/_archive/dzrobot_v2/docs/黑图探索与动态规划_决策层方案.md` | docs/20_资料索引库/赛事与治理/02_计分与时间预算.md（来源） | t5 黑图探索与动态规划决策层方案：要素清单、报文口径、覆盖策略 S1/S2/S3，只出方案不改码。 |
| `dzrobot_v2/docs/回归守护_known_reds.md` | `docs/_archive/dzrobot_v2/docs/回归守护_known_reds.md` | docs/20_资料索引库/赛事与治理/05_已知红与回归口径.md（来源） | known-red 唯一挂账真源：台账内同名同数算基线红，台账外即新增红，不许口头解释。 |
| `dzrobot_v2/docs/空地指挥链路_协议与运行手册.md` | `docs/_archive/dzrobot_v2/docs/空地指挥链路_协议与运行手册.md` | docs/20_资料索引库/空地链路/01_协议v2与契约.md（§2 来源） | 空地指挥链路协议与运行手册：车侧 mission.py / 机侧 link 的哈希绑定、报文与运行判据。 |
| `dzrobot_v2/docs/瞄准代码对比报告_gitee_vs_本地_20261018.md` | `docs/_archive/dzrobot_v2/docs/瞄准代码对比报告_gitee_vs_本地_20261018.md` | docs/20_资料索引库/视觉与云台/00_索引.md（册级声明） | 与根 docs/ 同名件内容一致的重复件：gitee vs 本地瞄准代码差异 1 文件 16 行。 |
| `dzrobot_v2/docs/清理记录_20261018.md` | `docs/_archive/dzrobot_v2/docs/清理记录_20261018.md` | docs/20_资料索引库/历史/02_并入与改名_20261018.md（来源） | 10-18 本地删减归档的执行与复核：工作区 6.41→3.81 GB，附删减清单与复核方式。 |
| `dzrobot_v2/docs/全量回归37套_严格审查台账_20261018.md` | `docs/_archive/dzrobot_v2/docs/全量回归37套_严格审查台账_20261018.md` | docs/20_资料索引库/车端/06_回归与测试基线.md（保留原文为证据） | 37 套严格审查（10-18 21:20 实测）：证伪「33 套全绿」，基线 37 套/13 红并逐套给复跑命令。 |
| `dzrobot_v2/docs/三引擎审计_20261018.md` | `docs/_archive/dzrobot_v2/docs/三引擎审计_20261018.md` | docs/20_资料索引库/车端/03_规划器与自动规划.md（同主题） | tour/tracker-safe/global_planner 三引擎同 seed 三图审计，判据是退出 0 且 route_check 0 error。 |
| `dzrobot_v2/docs/审查_路径规划器v3.7对比.md` | `docs/_archive/dzrobot_v2/docs/审查_路径规划器v3.7对比.md` | docs/20_资料索引库/车端/03_规划器与自动规划.md | v3.7 vs 当前规划器：同图同配置输出逐 token 相同，不是代码变弱而是换到第六张图。 |
| `dzrobot_v2/docs/审查_无人机飞控执行侧_20261018.md` | `docs/_archive/dzrobot_v2/docs/审查_无人机飞控执行侧_20261018.md` | 仅存档（无直接取代者） | 无人机飞控执行侧 P1 清单：3 条关键缺陷全在执行侧，不阻塞 T12（T12 卡硬件断电）。 |
| `dzrobot_v2/docs/使用手册.md` | `docs/_archive/dzrobot_v2/docs/使用手册.md` | docs/20_资料索引库/车端/05_配置与部署.md | 小车使用手册：启动/监控/排错/改车命令，§8 路线已同步 44 点/双停站。 |
| `dzrobot_v2/docs/无人机拟真与动态规划_方案.md` | `docs/_archive/dzrobot_v2/docs/无人机拟真与动态规划_方案.md` | 仅存档（无直接取代者） | 无人机固定轨迹 + 动态规划-循迹方案（v2 已拍板）：底层已存在约 80%，余下按卡推进。 |
| `dzrobot_v2/docs/无人机全要素感知_现状与方案.md` | `docs/_archive/dzrobot_v2/docs/无人机全要素感知_现状与方案.md` | docs/20_资料索引库/无人机/03_感知与视觉侦察.md（同主题） | t4 无人机全要素感知只读盘点：要素清单与补齐方案，每个数字带命令或 文件:行。 |
| `dzrobot_v2/docs/项目现状_20261018.md` | `docs/_archive/dzrobot_v2/docs/项目现状_20261018.md` | docs/00_项目总览/项目总览.md（来源）；架构段并见 docs/00_项目总览/系统架构.md | 10-18 17:10 车端现状报告（HEAD 193567a / tag v4.0.0），〔实〕〔会〕〔未〕三级标注。 |
| `dzrobot_v2/docs/新版架构设计.md` | `docs/_archive/dzrobot_v2/docs/新版架构设计.md` | docs/20_资料索引库/车端/01_架构与线程.md | 10-07 新版架构草案 v1：八线程划分 + 四任务点，依据三份只读调研（含行号证据）。 |
| `dzrobot_v2/docs/循迹算法说明.md` | `docs/_archive/dzrobot_v2/docs/循迹算法说明.md` | docs/20_资料索引库/车端/02_循迹与控制.md | 纯追踪循迹链路说明：弧长投影/前视/PIVOT 推导，附 10-17 路线口径注记（34 点→44 token）。 |
| `dzrobot_v2/docs/压测_规划器64图_20261018.md` | `docs/_archive/dzrobot_v2/docs/压测_规划器64图_20261018.md` | docs/20_资料索引库/车端/03_规划器与自动规划.md（28/64 数据来源） | hybrid 同源 64 图压测（seed=20261018）：scratch 性质，未改 planner/config/默认引擎。 |
| `dzrobot_v2/docs/library/README.md` | `docs/_archive/dzrobot_v2/docs/library/README.md` | 仅存档（无直接取代者） | 团队图书馆馆藏索引：可以查可以改，但改数字必须给出处，无出处一律标 🟡。 |
| `dzrobot_v2/docs/library/车体与标定.md` | `docs/_archive/dzrobot_v2/docs/library/车体与标定.md` | docs/20_资料索引库/车端/01_架构与线程.md（同主题；阿克曼约束段） | 车/机身份、几何、底盘标定、运动与安全限制、硬件清单，标注为配置级已核实。 |
| `dzrobot_v2/docs/library/流程与架构.md` | `docs/_archive/dzrobot_v2/docs/library/流程与架构.md` | docs/20_资料索引库/车端/01_架构与线程.md（同主题） | 一页数据流 + 8 线程 + 5 任务点，真源 main.py / 新版架构设计。 |
| `dzrobot_v2/docs/library/现场联调与部署.md` | `docs/_archive/dzrobot_v2/docs/library/现场联调与部署.md` | docs/20_资料索引库/车端/05_配置与部署.md（同主题） | 现场拓扑、部署流程与实跑坑，依据 StepLoop 两跑记录与 deploy_to_car.py。 |
| `dzrobot_v2/docs/library/引擎能力现状.md` | `docs/_archive/dzrobot_v2/docs/library/引擎能力现状.md` | docs/20_资料索引库/车端/04_定位与安全.md（定位/安全段） | 引擎阶段表：tour 唯一现役，hybrid 只到 P1 原型，64 图未证明净收益，P2 未启动。 |
| `dzrobot_v2/docs/agent_archive/v1_循迹收官.md` | `docs/_archive/dzrobot_v2/docs/agent_archive/v1_循迹收官.md` | docs/20_资料索引库/历史/01_沿革_20260927_20261012.md（同主题提炼） | v1 快照（10-11）：循迹收官，实车 4 趟的到点率/scale/漂移告警总账，核心标定冻结。 |
| `dzrobot_v2/docs/agent_archive/v2-全量整理-2026-10-12.md` | `docs/_archive/dzrobot_v2/docs/agent_archive/v2-全量整理-2026-10-12.md` | docs/20_资料索引库/历史/01_沿革_20260927_20261012.md（同主题提炼） | v2 全量整理（10-12）：7 个只读子智能体分头勘察的代码地图/配置真源/疑点/空间治理。 |
| `dzrobot_v2/docs/agent_archive/v3-空地协同闭环-2026-10-14.md` | `docs/_archive/dzrobot_v2/docs/agent_archive/v3-空地协同闭环-2026-10-14.md` | docs/20_资料索引库/历史/01_沿革_20260927_20261012.md（同主题提炼） | v3 增量档案（10-14）：只补 2e1c0c7→2523845 三段提交带来的空地协同闭环与仿真台变化。 |
| `dzrobot_v2/docs/archive/README.md` | `docs/_archive/dzrobot_v2/docs/archive/README.md` | docs/20_资料索引库/历史/00_索引.md（同主题） | 车端文档归档说明：按整理日期分批，每批一个 README 登记「原文件/日期/一句话/取代者」，只读不再更新。 |
| `dzrobot_v2/docs/archive/20261018/README.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/README.md` | docs/20_资料索引库/历史/00_索引.md（同主题） | 20261018 批 39 篇的登记表：每篇一行「原文件/日期/一句话/取代者」。 |
| `dzrobot_v2/docs/archive/20261018/C线_仿真器报告.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/C线_仿真器报告.md` | 仅存档（无直接取代者） | C 线离线纯追踪仿真器：模型修正 + 回归测试 + 调参结论（10-07）。 |
| `dzrobot_v2/docs/archive/20261018/n6_规划器隔板避让能力.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/n6_规划器隔板避让能力.md` | 仅存档（无直接取代者） | n6 规划器卡：隔板相邻行侧向 90° 转向的避让能力评估（planner-v6）。 |
| `dzrobot_v2/docs/archive/20261018/P0接口约定.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/P0接口约定.md` | 仅存档（无直接取代者） | P0 地基接口约定：core/route/pursuit/io 的 R 规则与签名，附 10-17 数字已变注记。 |
| `dzrobot_v2/docs/archive/20261018/P1接口约定.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/P1接口约定.md` | 仅存档（无直接取代者） | P1 循迹闭环四线程接口：统一构造签名、只经 Blackboard/EventBus 通信、tick 非阻塞。 |
| `dzrobot_v2/docs/archive/20261018/sim_v2_规格增补.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/sim_v2_规格增补.md` | 仅存档（无直接取代者） | sim_v2 规格增补：硬约束/棋子槽/冗余同源/无人机入链（用户 10-17 四条需求），原规格正文不动。 |
| `dzrobot_v2/docs/archive/20261018/sim_v2_设计规格.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/sim_v2_设计规格.md` | 仅存档（无直接取代者） | sim_v2 设计规格：拖拽棋子地图 / 50 图压测 / 无人机协同仿真，纯规格零代码改动。 |
| `dzrobot_v2/docs/archive/20261018/sim-bench-free_交接清单_20261015.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/sim-bench-free_交接清单_20261015.md` | 仅存档（无直接取代者） | sim-bench-free 队交接（10-15 晚）：作废 1014 版 §4、勘误其 §1/§3 数字。 |
| `dzrobot_v2/docs/archive/20261018/sim-bench-improve_交接清单_20261014.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/sim-bench-improve_交接清单_20261014.md` | 仅存档（无直接取代者） | sim-bench-improve 队交接（10-15 修订）：实测证伪旧版「SyntaxError 致测试跑不起来」。 |
| `dzrobot_v2/docs/archive/20261018/survey_chassis_vision.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/survey_chassis_vision.md` | 仅存档（无直接取代者） | 底盘驱动 / 里程计 / 辅助接口只读调研：逐行通读 dzactuator 等，全部结论带 文件:行。 |
| `dzrobot_v2/docs/archive/20261018/survey_gimbal_fire.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/survey_gimbal_fire.md` | docs/20_资料索引库/视觉与云台/01_云台瞄准与开火链路.md（提炼版） | 云台瞄准与射击链路只读调研：文件规模与三状态机链路的原始取证。 |
| `dzrobot_v2/docs/archive/20261018/survey_path_executor.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/survey_path_executor.md` | 仅存档（无直接取代者） | 循迹核心代码结构化分析：path_executor 2622 行 + car_localization 包，纯只读。 |
| `dzrobot_v2/docs/archive/20261018/备份清理记录.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/备份清理记录.md` | 仅存档（无直接取代者） | t21 备份清理执行记录：按精简审计清单删低风险项，清单外零删除。 |
| `dzrobot_v2/docs/archive/20261018/标定需求卡.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/标定需求卡.md` | 仅存档（无直接取代者） | 标定需求卡：3 张任务需求卡 + 1 张关键数字表，逐项标[实测]/[推定]/[待测]。 |
| `dzrobot_v2/docs/archive/20261018/部署清单_2026-10-11.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/部署清单_2026-10-11.md` | 仅存档（无直接取代者） | 10-11 第四张图（62 token）的上车部署清单，历史判据与哈希快照。 |
| `dzrobot_v2/docs/archive/20261018/车上自动规划_A_基线验收.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/车上自动规划_A_基线验收.md` | 仅存档（无直接取代者） | 车上自动规划任务卡 A 基线验收：43 token / 17.15 m 的当时快照。 |
| `dzrobot_v2/docs/archive/20261018/车上自动规划_B_MissionTask自动规划.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/车上自动规划_B_MissionTask自动规划.md` | 仅存档（无直接取代者） | 任务卡 B：MissionTask 读 map.yaml 自算路线 + planner 配置段 + --plan-only。 |
| `dzrobot_v2/docs/archive/20261018/车上自动规划_C_规划器无倒车.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/车上自动规划_C_规划器无倒车.md` | 仅存档（无直接取代者） | 任务卡 C：无倒车路线 + 折返点 :PIVOT 标记 + 合法性自检。 |
| `dzrobot_v2/docs/archive/20261018/仿真复验_第五张图44token基线_t2.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/仿真复验_第五张图44token基线_t2.md` | 仅存档（无直接取代者） | t2 第五张图 tracker-safe 44 token 基线复验（绑定 yaml sha256 F8BDEC15）。 |
| `dzrobot_v2/docs/archive/20261018/仿真复验_桥洞冗余墙_t5.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/仿真复验_桥洞冗余墙_t5.md` | 仅存档（无直接取代者） | t5 桥洞进洞侧冗余墙仿真复验：只改 sim.py 与产物，未动 config/route。 |
| `dzrobot_v2/docs/archive/20261018/仿真基线_三档对照.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/仿真基线_三档对照.md` | 仅存档（无直接取代者） | t22 三档（0.15/0.35/0.40 m/s）串行权威仿真对照，--set 只影响内存不改配置。 |
| `dzrobot_v2/docs/archive/20261018/仿真联合复验_t6.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/仿真联合复验_t6.md` | 仅存档（无直接取代者） | t6 三档复跑 + 路线证据（D3 档 44 token 单停站），文中 50 点/17.15 m 是历史档。 |
| `dzrobot_v2/docs/archive/20261018/仿真台对接说明_仿真员.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/仿真台对接说明_仿真员.md` | 仅存档（无直接取代者） | 给仿真员的六节对齐文档：正确用仿真台、解读画面差异、正确跑测试。 |
| `dzrobot_v2/docs/archive/20261018/仿真员工作交接包.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/仿真员工作交接包.md` | 仅存档（无直接取代者） | 仿真员岗位完整交接包：工作流/方法/判据，命令均本机实测过。 |
| `dzrobot_v2/docs/archive/20261018/机侧产物哈希清单.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/机侧产物哈希清单.md` | 仅存档（无直接取代者） | 机侧 dzsmartscout 无版本控制，v2.6 起以本 sha256 清单作为交付存档依据。 |
| `dzrobot_v2/docs/archive/20261018/阶段交接_复核.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/阶段交接_复核.md` | 仅存档（无直接取代者） | 阶段交接独立复核（HEAD e906e6c，10-09 19:2x）：只读 + 真跑命令，结论附 文件:行。 |
| `dzrobot_v2/docs/archive/20261018/阶段总结_出车就绪.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/阶段总结_出车就绪.md` | 仅存档（无直接取代者） | 出车就绪阶段总结：7 个提交独立核对 + 归档，「报告自述 vs 实物」不符处以实物为准。 |
| `dzrobot_v2/docs/archive/20261018/进度总结.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/进度总结.md` | 仅存档（无直接取代者） | 10-09 进度总结：dzbrain 已是可跑可测可解释的 8 线程系统，19 套 1385 断言全绿。 |
| `dzrobot_v2/docs/archive/20261018/精简审计清单.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/精简审计清单.md` | 仅存档（无直接取代者） | t15 精简审计清单：扫 74 个 .py 给出可删/低风险条目，供后续清理卡消费。 |
| `dzrobot_v2/docs/archive/20261018/联合复审_t7.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/联合复审_t7.md` | 仅存档（无直接取代者） | t7 联合复审 t1/t2/t3 交付与 t6 仿真证据：不采信卡片自报，全部本机重跑。 |
| `dzrobot_v2/docs/archive/20261018/路径规划报告.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/路径规划报告.md` | 仅存档（无直接取代者） | t22 规划器现状盘点 + 映射机制修复 + 优化路线图，数据全部来自实际命令输出。 |
| `dzrobot_v2/docs/archive/20261018/路线加固盘点与候选方案.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/路线加固盘点与候选方案.md` | 仅存档（无直接取代者） | t1 路线加固现状盘点与候选方案（planner-v6）。 |
| `dzrobot_v2/docs/archive/20261018/上下文快照.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/上下文快照.md` | 仅存档（无直接取代者） | 10-12 的 v2 档案版上下文快照，附 10-17 「数字不代表现役」提示。 |
| `dzrobot_v2/docs/archive/20261018/算法归纳规则.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/算法归纳规则.md` | 仅存档（无直接取代者） | 常驻铁律：路线/参数修正必须归纳进车上的 plan_route，不许只改验证用路线文件。 |
| `dzrobot_v2/docs/archive/20261018/图传UI改造_t4_验收.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/图传UI改造_t4_验收.md` | 仅存档（无直接取代者） | t4 图传页 UI 改造验收：删轨迹地图/常用参数/速度控制并美化布局。 |
| `dzrobot_v2/docs/archive/20261018/项目总览_架构与工作流报告.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/项目总览_架构与工作流报告.md` | 仅存档（无直接取代者） | 10-11 项目全景报告：背景/架构/演进/证据/问题，回答「哪里最脆弱最值得动」。 |
| `dzrobot_v2/docs/archive/20261018/新版小车报告.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/新版小车报告.md` | 仅存档（无直接取代者） | 新版本小车（dzbrain）交付报告：8 线程四任务点全通，未推送车端。 |
| `dzrobot_v2/docs/archive/20261018/语音模块调研.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/语音模块调研.md` | 仅存档（无直接取代者） | 车端语音发声链路与自定义可行性调研，车端全程只读。 |
| `dzrobot_v2/docs/archive/20261018/云台补丁说明.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/云台补丁说明.md` | 仅存档（无直接取代者） | 云台 C++ 补丁（两键开火 + 延迟补偿提前量）：默认关闭、可回退，已改本地未编译未上车。 |
| `dzrobot_v2/docs/archive/20261018/诊断_投影与顶点.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/诊断_投影与顶点.md` | 仅存档（无直接取代者） | bench 跳点与遥测离群根因诊断：投影链/顶点/前视塌缩，两个独立只读复现。 |
| `dzsmartscout/docs/drone/uXRCE恢复手册.md` | `docs/_archive/dzsmartscout/docs/drone/uXRCE恢复手册.md` | docs/20_资料索引库/无人机/02_飞控与uXRCE域号.md（两篇合并） | uXRCE-DDS 会话恢复手册：10-17 起 agent 每 ~61 s 被看门狗重启、/fmu/* 发布者为 0 的排查与恢复。 |
| `dzsmartscout/docs/drone/感知链体检_20261017.md` | `docs/_archive/dzsmartscout/docs/drone/感知链体检_20261017.md` | docs/20_资料索引库/无人机/03_感知与视觉侦察.md | 10-17 感知链只读体检：链路没坏是没起——雷达/FAST-LIO/点云桥/相机全部未启动。 |
| `dzsmartscout/docs/drone/无人机架构总览.md` | `docs/_archive/dzsmartscout/docs/drone/无人机架构总览.md` | docs/20_资料索引库/无人机/01_架构总览.md（两篇总览合并） | 无人机架构总览：dzpx4 飞控四包 + 感知链，回答进程/话题 QoS/域号/排障四问。 |
| `dzsmartscout/docs/drone/无人机项目详细报告_20261019.md` | `docs/_archive/dzsmartscout/docs/drone/无人机项目详细报告_20261019.md` | docs/20_资料索引库/无人机/01_架构总览.md（两篇总览合并）；状态段并见 06_已取证与未验证.md | 10-19 无人机全量现状报告：机端未连上，以本地仓库 + 10-17/18 机端实测为据。 |
| `dzsmartscout/docs/drone/域号切换单_118.md` | `docs/_archive/dzsmartscout/docs/drone/域号切换单_118.md` | docs/20_资料索引库/无人机/02_飞控与uXRCE域号.md（两篇合并） | 飞控链域号 0→118 统一切换单：PX4 UXRCE_DDS_DOM_ID 与 Orin 全部 ROS_DOMAIN_ID，回滚=两侧回 0。 |
| `dzsmartscout/docs/空地路线下发契约_v2.md` | `docs/_archive/dzsmartscout/docs/空地路线下发契约_v2.md` | docs/20_资料索引库/无人机/04_路线下发与轨迹回放.md（提炼） | 无人机侧视角的路线下发契约 v2：/nav/waypoint 与 /dzlink/* 字段、token 语法与双侧对拍证据。 |
| `dzsmartscout/docs/无人机项目总览.md` | `docs/_archive/dzsmartscout/docs/无人机项目总览.md` | docs/20_资料索引库/无人机/00_索引.md（入口职能） | 无人机新接手者 10 分钟总览：是什么、能干什么、怎么连怎么用、坑在哪。 |
| `dzsmartscout/docs/一次性脚本清单_20261017.md` | `docs/_archive/dzsmartscout/docs/一次性脚本清单_20261017.md` | docs/20_资料索引库/无人机/05_现场操作与脚本.md（三篇合并） | 根目录 29 个一次性脚本/残留文件清单与处置建议，凭据统一用 <CRED> 代称。 |

> 「取代者」填法：优先取 `docs/20_资料索引库/**`、`docs/00_项目总览/**`、`docs/10_agent提示词/**` front matter 里 `取代:` 的点名声明；无点名则取同主题新文档；两者都无写「仅存档（无直接取代者）」（本表共 43 篇属此类，多为过程报告/验收快照/施工记录/接口约定）。
> 「一句话内容」均由归档时**逐篇读过开头 20 行**后撰写，非按标题猜测。

## 3. 旧路径怎么取文件

规则只有一条：**在原路径前面加上 `docs/_archive/`**。正文里出现 `dzrobot_v2/docs/xxx`，就到 `docs/_archive/dzrobot_v2/docs/xxx` 取。

| 正文里看到的旧路径 | 实际到哪取 |
|---|---|
| `dzrobot_v2/docs/循迹算法说明.md` | `docs/_archive/dzrobot_v2/docs/循迹算法说明.md` |
| `dzrobot_v2/docs/archive/20261018/survey_gimbal_fire.md` | `docs/_archive/dzrobot_v2/docs/archive/20261018/survey_gimbal_fire.md` |
| `dzrobot_v2/docs/library/引擎能力现状.md` | `docs/_archive/dzrobot_v2/docs/library/引擎能力现状.md` |
| `docs/merge/S5-切换记录.md` | `docs/_archive/docs/merge/S5-切换记录.md` |
| `docs/agent_handoff/packs/L1_MANIFEST.md` | `docs/_archive/docs/agent_handoff/packs/L1_MANIFEST.md` |
| `docs/云台PID自动整定方案.md` | `docs/_archive/docs/云台PID自动整定方案.md` |
| `dzsmartscout/docs/drone/域号切换单_118.md` | `docs/_archive/dzsmartscout/docs/drone/域号切换单_118.md` |

**反向例外（8 个，没进 `_archive`，仍在 `dzrobot_v2/docs/` 原位）**：`决策台账.md`、`复用清单.md`、`工作流v2.md`、`目录索引.md`、`目录索引.json`、`索引标注.yaml`、`能力索引.md`、`TEAM_STATUS.md`。查到这 8 个名字请回**原路径**找。

## 4. 怎么核对（可复跑）

```powershell
cd "D:\User\Desktop\比赛存档\v1"

# ① 归档篇数应为 118（此命令会数到 119，因为本 README 自身也算一个文件）
(Get-ChildItem docs\_archive -File -Recurse).Count          # 期望 119 = 118 + 本 README

# ② 总字节应为 5184331
(Get-ChildItem docs\_archive -File -Recurse | Measure-Object Length -Sum).Sum

# ③ manifest 应 118 行数据 + 1 行表头，且「一致」列无 FALSE
(Get-Content docs\_work\archive_manifest.csv).Count        # 期望 119
Select-String -Path docs\_work\archive_manifest.csv -Pattern ",FALSE$"   # 期望无输出

# ④ 任取一篇逐字节复核
Get-FileHash docs\_archive\dzrobot_v2\docs\循迹算法说明.md -Algorithm SHA256
Get-FileHash dzrobot_v2\docs\循迹算法说明.md -Algorithm SHA256
```

全量复制脚本留档：[docs/_work/_do_archive.ps1](../_work/_do_archive.ps1)（幂等，重跑只覆盖 `_archive` 内副本，不碰原文件）。

## 5. 明确没归档的（不许动）

| 不归档对象 | 原因 |
|---|---|
| `docs/README.md` | 新体系总入口 |
| `docs/目录索引.md` / `.json` | 由 `docs/_工具/gen_index.py` 自动生成，**不许手改** |
| `docs/00_项目总览`、`docs/10_agent提示词`、`docs/20_资料索引库`、`docs/30_任务卡库` | 新库现行内容 |
| `docs/_工具`、`docs/_work`、`docs/_archive` 本 README | 工具、记账与归档说明 |
| `_tmp_session/`、`dzrobot_v2/_tmp_session/` | 证据仓，由 Lead 另行处置（本卡不碰） |
| `dzrobot_v2/docs/` 上述 **8 个**活文件 | 门禁工具机器读取，必须留在原路径（`AGENTS.md §8.1`） |

## 6. 已知风险 / 删除原件前必读

1. **删除范围**：只删第 2 节映射表里的 118 行对应的**原路径**；第 3 节的 8 个例外**不许删**。
2. **删后必跑门禁**：`cd dzrobot_v2 && python tools/wf/check.py`。本卡**没有**跑过门禁（不在验收范围），删除后必须由 Lead 跑一次，确认断链 = 0（`check_links.py` 覆盖 `docs/**`）。
3. **同名重复件**：`docs/瞄准代码对比报告_gitee_vs_本地_20261018.md` 与 `dzrobot_v2/docs/` 下同名件是两份独立归档（内容重复），删原件时两份都要删。
4. **本卡只复制不删除**：所有原件仍在原位；`_archive` 里任何文件丢失都不影响现行体系。
