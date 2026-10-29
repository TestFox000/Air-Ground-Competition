# 团队状态板（TEAM_STATUS）

> **2026-10-29 18:3x onboarding t4 验收通过 + Lead 修复复用门禁绕过漏洞（lead · t6 终验中）**：
**t4 调参实验室**（sim-lab）判 **接受**：tools/sim_lab/ 三件 1084 行 + 车端/14 文档 257 行。Lead 独立复跑：--selftest 9/9、--all 15/15 exit 0；**确定性复跑两遍全量产物 diff=2722B，逐条核对差异全部是 report.txt 的墙钟秒数，traj/by_segment 等观测产物逐字节一致**（成员口头报的「逐字符相同」不成立，但其文档并未写这句话——文档无虚标，口径以本条为准：观测数据逐字节可复现，报告耗时字段除外）。三条教学结论产物坐实：lookahead_min 零差异（max 钳住机理写入）、safety_margin 74→212 拍轨迹不动（只观测不改控）、latency RMS-41.5% 假象带「⚠ 未跑完·不可横比」旗（4476 拍发散）——诚实标注是本实验室最值钱的设计。**Lead 修复（工具写域）**：check_reuse.py:37 加 --untracked-files=all——t4 报的绕过漏洞属实（未跟踪新目录折叠成 ?? dir/ 使 .py 判定失效），探针实证修复后嵌套新文件判红、撤探针 GREEN；K9 warn 计数经核仍 13（修复未误伤存量口径）。**外部干扰第三例**：dzrobot_v2/_archive_20261029/（256M，17:33 外部进程建，非本队）出现在 git 未跟踪——与 clear/、根目录两份草稿一并留给用户定夺，本队不动。t1~t5 全接受，t6 终验进行中。

> **2026-10-29 18:2x onboarding t5 验收通过 + README 入口落笔（lead）**：
**t5 导览+题库**（quiz-teacher）判 **接受**：15 页 279 行/6 单元/30 题；Lead 独立复核——quiz spec 结构解析（5 radio 各带 group+answer、submit 在位）+ validate_dsh_ui 现场重验 valid/rendered=6；答案表出处可回溯（scan_pitch 实验命令在文）。**README.md 入口行已落**（30 秒行后新增「完全新手/想接手」行，链 15 页+三篇配套），更新日期 10-26→10-29，gen_index --only 两册，links 275/0 断。**R1 三条红线全部为外部并发进程**（clear/goals/ + clear/ontology/ 17:45 后持续增文件、根目录 dzbrain_零基础深挖报告.md 17:59）——非本队成员所写，不动，收口报告用户。车端/14（sim-lab 18:17 落盘中）README 计数已被其 owner 同步至 15，无 R6 风险。t4 在途。


> **2026-10-29 18:0x 工作区归档整理（lead · 未 commit/未推送/未连设备）**：
按用户口令「能删的按日期归档；不能删的先验证是否现用，纯备份也一起归档」执行。

**已归档**：① v1 `backup/v1-full-20261024.bundle`（2.13 GiB 纯备份，10-24 全仓快照，落后 HEAD 5 天，文档 0 引用）→ `_archive/20261024/`，附 MANIFEST（sha256 `0afae0ee…4741` + 三种恢复命令），源目录已删。② dzrobot_v2 `_scratch_aim`（3 .py）→ `_archive_20261029/20261012/`（git mv 保历史，内容哈希不变）。

**沿用既有归档区不另造**：`dzrobot_v2/_archive_20261029/`（并行会话 17:32 建，248 MB、六日期桶）内已含截图里疑似「消失」的 `_scratch_auto_plan`/`_scratch_displayfix`（20261012）、`_scratch_scout`（20261018）、`artifacts_sim`（20261018/23/24/25）；`.agent-teams`、`_tmp_pre` 已不存在（后者原为空目录）。

**核实后不动（附证据）**：
- `v1/git_save/`（3.0 G，v1.1.0~v4.0.2 共 20 版）保留原位：D-032 版本存档线实体，有 `06_版本存档规则.md`/`08_版本存档线收尾_20261023.md` 与 5 处真链接，且已在 doc_policy 豁免区。**但落后当前 v4.3.2 五版** ⇒ 挂账。
- `dzrobot_v2/_backups/`（84 K / 3 跟踪文件）**不可移动**：`docs/_工具/check_anchors.py:71` 硬编码其为 baseline 目录；`tools/calib_backfill.py:_HISTORICAL_SNAPSHOTS` 登记为「历史部署快照」豁免项。动它同时弄坏两个现行工具。
- `dzrobot_v2/backup/`（69 M）**本就是官方归档区**（`tools/wf/repo_index.py:183` 写明「归档 | dzrobot_v2/backup/」），3 个 v3.9 bundle + 9 个改前快照已在位。
- `artifacts/sim`（2.0 M）、`artifacts/runs`（914 K/93 场）**现用非备份**：sim_cli 默认输出目录 / `run_archive.py` 运行期写入。

**门禁**：`wf/check.py` GREEN；`check_links` 273 文件 0 断；`calib_backfill --selfcheck` error 0/warn 4（标的仲裁，非本轮）；索引经 `repo_index.py --write` 自愈（`_scratch_*` 四条过期行消失、`_archive_20261029` 已登记、`_backups` 加「不可移动」注解）。

**挂账（不挡明早）**：`git_save` 缺 v4.3.0/4.3.1/4.3.2 三版存档；`dzrobot_v2/.gitignore` 未列 `_archive_20261029/`（现靠 doc_policy 豁免区兜住，非 git 忽略）。

> **2026-10-29 18:0x onboarding t2 验收通过 + anchors 欠账清偿 45/50（lead）**：
**t2 模块协调**（chain-guide）判 **接受**：420 行/176 锚点。Lead 抽查：行数属实、代码锚点密度实测、§5 P_POSE 写者断言抽中（localization.py:459/:465 实测在——「3 写者」全表与 contract.py:48 字段序完整复核移交 t6）；本篇 anchors 零贡献，numbers/links/wf 全绿。**索引口径核对**：README 车端 14/全库 77 与实扫一致。gen_index 全量误触发已两例——收口前 Lead 给工具补 --help no-op。**anchors 欠账清偿**：50→5，方式=自 _archive/trim tar 选择性还原 ~26 个成员（distill/、rec/、emit/、detbench/、runs/、digest 等），_tmp_session 仍 1.1G；剩 5 条全归他卡在途（agc-1024/1026 ×3、suite_table 裸名误判 ×2——实际文件在 TRAJ-BASE/suites/ 与 traj-1025/frame-migration/），移交 t6/收口不扩修复面。policy 唯一红 = 外部 clear/（非本队，报用户）。t4/t5 在途。

> **2026-10-29 17:5x onboarding t3 验收通过（lead）**：
**t3 日志课堂**（log-medic）判 **接受**：549 行，§1~§4+§6 四段齐全。Lead 独立抽查：版本差异表逐行可指（1_chassis.log 第25行 ticksPer2PI 2680 vs robot.yaml:972=2670——「先对版本再判故障」纪律成立）；manual_gimbal 假红指到修复点（ros_io.py:48-57/main.py:2262）；**Y9 口径成文**（过程区引用禁写 path:line，改「路径+第 N 行」）已吸收进收口规范。README 篇数口径核对一致（车端 14 文件/运维 11、全库 77，R6 无红）。本篇 policy 零贡献。**遗留**：main.py 头注释缺退出码 7 的补行——代码注释属冻结区，Lead 收口时一并裁定。**外部干扰登记**：clear/ontology/verification-loop.md 系非本队并发进程 17:45 重建（17:19 草稿的副本在 ONBOARD-SCRATCH，已双份保全），R1 单条红为外因——不再反复搬移，收口时报用户定夺。

> **2026-10-29 17:4x onboarding t1 验收通过 + Lead 欠账登记（lead · 未 commit）**：
**t1 调参地图**（param-mapper）判 **接受**：694 行/256 键全表+117 默认档+26 场景反查。Lead 独立抽查：死键断言逐行属实（mission.py:1564 读 shot_timeout_s，yaml:724 写 shoot_timeout_s），锚点格式实样合格，其文档在 policy/anchors 零贡献（policy GREEN / numbers 0 / wf GREEN）。11 条「改了不生效」键与 18 键迁移表价值高，t4/t5 受益。**Lead 欠账**：anchors 新增红 50——归属核对 ~40 条系昨晚 _tmp 裁剪余震（历史/09、车端/06、vtest 卡指向 archive-20261025/vtest-1025 未选择性还原的文件；当时只按 Y9 清单还原 17 个，anchors 扫描面更广），余为他卡存量。处方：补还原小文件 或 标〔失效〕/tmprefs 基线——收口时 Lead 清偿，不挡在途成员。杂物清理：robot.yaml_配置解读报告.md + clear/ 两份草稿（17:19/17:29 并发产物）收进 _tmp_session/ONBOARD-SCRATCH-20261029/，policy 回 GREEN。

> **2026-10-29 04:0x P4 全项清零 · 证书挪位更正（lead · commit ×3 / 未推送 / 未连车）**：
① **agc-1026 R8 修复**：补建 LIVE-RUN-20261028.md 文件入口（D-057「不另建卡文件」口径与 R8 的冲突裁决：树节点必须有文件，正文仍统一在 PLAN.md）+ 树节点挂链接 + gen_index 重扫 → **check_doc_policy 首次全 GREEN（0 红）**（v1 commit 1780b5e2）。② **可跑证书挪位**：dzrobot_v2/_tmp_session 内的 .md 触发 R1（库外新增）→ 移至 **v1/_tmp_session/CERT-20261029/**（含 sim_full.log；TEAM_STATUS 上一条所指路径以此为准）。③ **CRLF 归一**：dzrobot_v2 .gitattributes（eol=lf 全套策略）早已在位，renormalize 后仅 TEAM_STATUS.md 索引转 LF（commit 47ad60e7）；v1 索引本已全 LF。④ **_tmp_session 5.7G→1019M**：archive-20261025 + vtest-1025 压成 1.49G tar.gz 存 _archive/trim_20261029/（内容可恢复），原目录删除。P4 六项全清。

> **2026-10-29 04:0x P4/P5 收口完成 · 可跑证书签发（lead · 已本地 commit ×2 / 未推送 / 未连车）**：
**里程碑提交**：dzrobot_v2 **8e30e509**（68 文件 +11407/−1043，P1+P2+P3+Lead 修复+各卡 frozen 在途）；v1 **8c5a1a75**（205 文件文档收口 + .gitignore 隔离 _archive/artifacts/git_save/backup/cache）。两仓工作树现已干净。**P5 彩排**：sim_cli RC=0 / 118.12 s / RMS 0.1187 m（68 点全路程），route_check 六层全过（雷 8/8 靶 2/2 桥 K5/K7）。**可跑证书** _tmp_session/CERT-20261029/RUN-CERTIFICATE.md（验证矩阵 + K10-K14 红账 + 明早前置 + 回滚锚）。**清理战果**：p2_verify 19G 回收、根目录 _tmp 垃圾清零、frozen 失效登记清除、K-list 扩至 K14。明早入口 = 09_SOP（20 库运维与排障）。

> **2026-10-29 04:0x V2 裁定修复 + V3 小修全落地（lead · 未 commit/未连车）**：
**P2 修复（V-04）**：Lead 第一手复核坐实 V2 归因（备份-pas补丁-还原：mission.py sha 119d7f23 前后一致，passthrough→118/118）。裁定走 ①号方案：sim_layout 加 start 形参（默认 N10 不变），12.18/15.13/15.14 三处同格/空段用例 start 设为 C9=复位格 + 口径注释。**test_stepper 118/118、test_p2_t2 30/30 双绿**，生产语义（布局 start 唯一权威）原样保留。P2 正式判**接受（修复后）**。V2 另两条挂账：test_mission 17 红=既有基线（换 HEAD config 同红，入 K10/K11）；drift 1.2b=localization F5 在途（入 K13）；2s 逃生门=设计取舍（入 K14）。**V3 小修 4 件全做**：07 页 G1 命令①改有图/无图双分支、07/09 页 start_all.sh 锚点改 228/265/284（带日期注）、AGENTS.md:28 旧档路径补层、**删 dzrobot_v2/_tmp_session/p2_verify（19G 整仓副本，doc_policy 188 条 R1 源）探针留 p2_verify_probes/**。frozen.json 删 map.yaml/map_v3.yaml 两条失效登记（29 条，JSON 有效）。门禁终态：policy 仅 R8 / anchors 新增 3（他卡） / numbers 0 / wf GREEN 零 WARN。

> **2026-10-29 03:5x map.yaml 恢复裁决 + K10 挂账（lead · 未 commit/未连车）**：
**裁决：config/map.yaml + map_v3.yaml 从 HEAD 恢复**（刀口仅这两文件，不动他人在途）。理由：门禁机器判据 test_deploy ③c「本地仍保留」+ G1 自检 --map 依赖 + test_mission/test_drone 模块级依赖三方同指；「冷启动无图」线的正确落点是车端不部署（PUSH_FILES 本就不含 map.yaml），不是删仓内开发/仿真入口。恢复后实测：**test_deploy 94/94 全绿**、**G1 error 0 / warn 0 通过**、test_drone 177/177、test_fix 74/74。新发现预存红：test_mission §18 两条（期望 44 点 D3*/G3* vs 现役 68 点 F4*/B4*）——route.tokens HEAD==工作区无在途改动，测试与已提交配置**双向陈旧**（且 F4* 落在第六图雷区，路线本身不自洽），与今晚任何卡无关、不在彩排路径（fallback_to_tokens=false）。已挂 **K10**（08 页，到期 11-05，处置=路线卡 owner 更新 config+测试）。8 套件全量回归后台扫描中（bash-97869b09dcbb4de8）。

> **2026-10-29 03:40 V1 独立验收通过（lead · 未 commit/未连车）**：
V1（e50a9aef，误挂 Kimi 路由，Lead 已复核 verdict 语义）对 P1 LINK-HARDEN 12/12 全过：canonical 话题口径（斜杠全量不缺失/删一条报缺/None 不谎报）、--map 守卫（无图不传/残留图警告+带参）、T3/T3b 域号真话（一致 0错0警/分叉1错）、auto_start=false + 横幅、test_deploy 93/94（③c 唯一红=map.yaml 预存）、sim_cli 冒烟 exit 0、frozen.json 4 条登记齐。P1 **接受，可进 P4**。

> **2026-10-29 03:3x P3 交付 + Lead 指针修复 + numbers 自愈（lead · 未 commit/未连车）**：
P3 交付三篇现行文档（07 发车门禁 G1-G4 267 行 / 08 known_reds K1-K9 215 行 / 09 明早 SOP 395 行）+ car_reconcile.py 三方对账工具（254 行，复用 deploy_to_car 清单逻辑）。六处「文档≠代码」全部按代码真相修正（布局门 60s、exit 7、SAFE_STOP 三分、stall 10 拍、planner SCHEMA warn 已销、无现行表节）。Lead 补 5 处指针/陈旧值（AGENTS.md G1-G4→07 页、known_reds→08 页、复用清单→08 页、00_索引 与 02_runbook 30s→60s〔实〕）。**numbers --fix-sources 自愈 23 条 yaml 行号漂移 → exit 0**（fire_hold_frames 异文为 D-052 声明并存项，非失败）。门禁终态：gen_index 0 / doc_policy 1(仅 R8 他卡) / anchors 1(新增 3 全他卡) / numbers 0 / wf GREEN。V3 已派（raccoon/on）。

> **2026-10-29 03:35 P2 交付 + 两条 Lead 裁决（lead · 未 commit/未连车）**：
P2 交付 T1/T2/T3：劫持闸（步进段限定 + 反证）、起点口径（布局 start 强制 + 只写一次 belief）、AIM 链路核查（只验证）。三套新测试 26/30/19 自报全绿，check.py GREEN，sim_cli 冒烟 exit 0。**裁决一：mission.reset 不置空**——P2 的 _authoritative_reset_spec 已在有布局时强制 layout.start（配置值仅剩无布局回退用途）；且 :2730 bridge.set_pose_at_block 的兜底硬编码 N10,y 与配置无关，置空治不了 odom 基线，还要动冻结区 ⇒ 一变量原则，不动。**裁决二：AIM lead_apply_gain 维持 0.5**——等效提前 0.135 s vs 链路延迟 0.5-0.9 s 的杠杆问题记入实跑检查单（明天脱靶时第一杠杆=lead_apply_gain 0.5→1.0，须实靶验证后动），今晚不动冻结配置。**V2 已派（raccoon/v4.1-flash/on，ae4441b7）**，头号靶子=stepper 3 红「118/118 中途 vs 3 红终报」矛盾 + test_mission/test_drone map.yaml 模块级依赖真假。**路由口径**：全部子智能体一律 raccoon/sn-deepseek-v4-1-flash-1017 + effort on（V1 误挂 Kimi 为唯一例外，Lead 逐条复核其 verdict）。

> **2026-10-29 03:2x P2 中途诊断 + T7/T8 生产重写授权（lead · 未 commit/未连车）**：
P2 交付中途报告：T5 诊断三红各自根因（①belief 未重锚到布局 start → 首段偏一格；②GATE_MIN_PROGRESS 0.02 m 形同虚设；③token 重锚跳 s）；T6 已改 _seg_locked 信念锚定（:1026-1150，含负例：未应用补丁旧闸放 0.02 s 段 + 应用后正确拒绝）；T7 验收清单成文（→即 V2 判据）；T8 步进器测试重写 **115→118/118 全绿**（12.18/15.13/15.14 修复，同格 0 token 停站算有效段）。**Lead 授权**（subagent 不可中途寻址，落盘供核对）：T7+T8 语义落生产 _report_segment_done 重写（:1419-1554 区）方向批准，约束五条——①frozen.json in_flight 登记先行；②负例探针保留（去 30% 闸必红）；③本地全回归（test_mission 103/103、test_stepper 118/118、test_drone、test_safety）+ sim_cli --help 冒烟全绿；④T7 清单逐条对应报告；⑤判据⑤ SAFE_STOP 超时恢复同纪律（先诊断+负例，全绿才动，登记 in_flight）。与 A 审查问题 4（段完成劫持）方向收敛。

> **2026-10-29 03:15 P1 交付 + T3b 根因收口（lead · 未 commit/未部署/未连车）**：
① P1-LINK-HARDEN 交付 T1-T4：T1 纠正了我 A 审查里「manual_gimbal 订阅缺失」的判断——订阅早就在（f883c520），真问题是名字口径（合同无前导斜杠，rclpy 回读带斜杠），已用 canonical 统一，不建双订阅；T2 start_all MAP_ARGS 守卫；T3 env 文案；T4 auto_start=false + 横幅。② **T3b 根因（Lead 补）**：dzros.env:16 / start_all.sh:124 DZ_ROS_DOMAIN 只 set 未 export ⇒ 子进程核不了域一致性；已补 export 两行，bash -n OK，env_problems 四态实测（一致 0/0、分叉 1 错、未导出 1 警、全空 1 警）正确，A/B 打法仍可用（两个域一起设为同值即通过）。③ test_deploy 93/94（③c 预存）、test_contract 待 V1、check.py GREEN。④ test_stepper 3 红（12.18/15.13/15.14）归因=P2 在途改动（mission.py 03:09 mtime + _tmp_p2_*.log），不归 P1，V2 硬判据含「stepper 全绿」。⑤ **路由失误**：V1（e50a9aef）误继承 Kimi 路由（用户要求 raccoon/v4.1-flash/on），其产出的 12 条 verdict 将由 Lead 逐条复核；后续所有子智能体一律 raccoon/sn-deepseek-v4-1-flash-1017/on。

> **2026-10-29 03:1x test_deploy ③b 幂等语义再钉（lead · 未 commit/未部署/未连车）**：
P1/T4 把 config/robot.yaml 的 path_executor.auto_start 归位 false 后，test_deploy.py:180-198 两条断言因硬假设「真文件 auto_start 为 true」转红（真实文件 diff=0 行；180 行只靠 CRLF→LF 副作用偶然存活）。Lead 裁定**幂等空操作语义**（t31 形态）：180 行改条件断言「残留 true ⇒ 内容必变；false 稳态 ⇒ 内容不变」（与取值/行尾无关），194 行去掉 len>=1 硬前提只留「改动行 ⊆ auto_start 配置行」（空 diff 合法）；「true⇒必变」方向由合成样本(:171-179)钉住，语义不降。**复跑 test_deploy 93/94 rc0**，唯一红=③c「本地 config/map.yaml 仍保留」（预存，map.yaml 被「冷启动无图」线在途删除，与门禁 ③c 不变式及 G1 --map 依赖直接冲突）⇒ **map.yaml 恢复裁决挂 P4**。写域备注：test_deploy.py 名义属 P2 域，本改动为阻断项+两行级，P2 未触该文件，无冲突；V1 验收须复跑 test_deploy。

> **2026-10-29 02:5x 夜间收口开工（lead · 用户全权授权「睡觉期间自行执行」· 未 commit/未部署/未连车/未连机）**：三实现者并行——P1 链路止血（manual_gimbal 订阅 / start_all.sh --map 死参数 / 域号横幅假警报 / 旧核 auto_start 防护，子 eaa71f4d）、P2 判据口径（段完成劫持闸 / 起点三口径收敛 / AIM 提前量链路核验，子 ef4d2891）、P3 发车文档+对账 SOP（子 ea283d62）。已知转向修正待验收时落实：发车门禁口径应建 dzrobot_v2/docs/（dzrobot_v2/AGENTS.md 引用点）、回归守护_known_reds.md 不得重建（02:45 已改指决策台账+TEAM_STATUS，原件在 history_cache）——本运行时 subagent 不可中途寻址，V3 验收时修。回滚锚 _tmp_session/BASELINE-20261029/（哈希+diff，**回滚禁 git checkout**）。每阶段独立验收员复跑后才进 P4 清理与 P5 仿真彩排。车/机均离线，今晚全部本机验证。

> **2026-10-29 02:45 档案库卫生小轮（lead · 未 commit/未部署/未连车）**：删根目录再生垃圾 nul；交接_20261024 红锚补全路径（新增红 4→3）；dzrobot_v2/AGENTS.md 悬空指针「回归守护_known_reds」改指 决策台账+TEAM_STATUS（原件在 history_cache）；anchors 基线纯收缩 28 行（Thin Ledger 修好的 4 条出基线）；20库 8 分册篇数复核全绿；gen_index 257 篇。门禁：numbers/wf/links exit 0；policy 1=agc-1026 树 R8（他 ns）；anchors 1=3 条红全在 agc-1024/agc-1026 在办卡（他 ns，处方：各 1 行标〔失效〕或补路径）。

> **2026-10-29 01:45 Thin Ledger v1 收口（lead · 未 commit/未部署/未连车）**：决策台账三线分流全链完成并独立验收通过（v2 7/8 + v3 4/4）。台账 420→500 行：首屏速查区 19 条（40 行压线）、主表 D-001~D-023 严格递增、D+ 现行指针 26 处、D-058/D-059 入附录 A；数字速查.yaml 52→72 条（§7 台账分流迁入 20 条，65 auto 全 PASS / 靶速 0.3 manual 带曾判沿革）；查账四处统一（AGENTS/台账/03 细则），全库现行区「查账三步」0 残留；5 处台账行号锚点 + PIVOT 锚点 :108→:125 修复。门禁：gen_index/numbers/wf/check/links exit 0；policy 1（agc-1026 存量 R8）与 anchors 1（10 条他域在途红）核实非本任务。**回滚禁 git checkout**（HEAD 与工作区差 199 行他人未提交）。验收报告 _tmp_session/doc-thin-ledger/verdict_v2.md + verdict_v3.md。

> **2026-10-29 01:5x test_stepper 12.11 红转绿（lead · 未 commit/未部署/未连车）**：`test_stepper` **117/118 → 118/118 rc0**。根因是测试自己的激励与断言自相矛盾：旧写法在刚进 STOP（SETTLE）那拍就 `add_shots(3)`，FIRE 首拍 `delta=3 ≥ shots_needed` ⇒ 走 `_complete_stop` 返回，`_set_fire_gate(True)` **一次都不执行**，却又断言「开火门开过」。修法：注入条件绑定开门拍（`fire_gate[-1] is True` 时逐发注入 1 发），断言改为「开门拍数 ≥ 注入发数 = 3 且末态 False」；实测 **开门 3 拍 / 注入 3 发**。**负例已证**：把 `_set_fire_gate` 打成 no-op ⇒ 12.11 必红（开门 0 / 注入 0 / exit1），脚本 `_tmp_session/stepper_1211_negative.py`。**产品代码 0 行改动**（仅 `src/dzbrain/tests/test_stepper.py`，+30/−13）⇒ sim_cli 无路径变化、不适用；`tools/wf/check.py` **GREEN**。附带：`dzrobot_v2/AGENTS.md` 指向的 `docs/回归守护_known_reds.md` 在现树**不存在**（只在 `docs/history_cache/` 与 `git_save/` 有副本），挂账红实际记在决策台账/TEAM_STATUS。

> **2026-10-29 逐模块架构研究续接（/root）**：已只读评估车端任务/算法、公共状态与配置、机端飞控/感知及云台跨语言边界；建议保留总体分层，优先收紧状态写权、观测身份和任务职责，再按证据拆分。记录续入 [ARCH-RESEARCH](../../docs/30_任务卡库/在办/archrev-1028/ARCH-RESEARCH-20261028.md)。未实施代码改造、未连接设备，不把架构建议当实车验收。

> **2026-10-28 架构研究（/root）**：用户优先改善现场故障定位与复现。只读核查现有任务分层、sim/bench、运行归档与工作流，建议优先落实既有 E3 证据链，再做小范围解耦；未改生产/配置、未连接设备、未提交推送。研究与待讨论建议见 [ARCH-RESEARCH](../../docs/30_任务卡库/在办/archrev-1028/ARCH-RESEARCH-20261028.md)。工作流门禁与断链过；全库文档守卫仍有其他 ns 红，未报全局收口。

> **2026-10-28 缓存直连已实际执行，首段仍拒收（lead）**：仅给既有Lead入口补--saved-stream及构造成功后进入原laptop.main的接缝；本次启用live参数实际运行，0.91秒返回exit2/SOURCE_REJECTED，新推理0帧。消费既有2347条记录，64次原布局构造均因首段格子未同时覆盖拒绝；后续源累积类别/坐标冲突，含M10类别冲突，不能拿最终诊断known替代可用observation。controller_entered=false，下发0/运动0，未重新连接SSH或起车栈；规划/循迹/真实done未发生。实时成功分支尚未实际走到，不报通过。未补known/清冲突/改生产默认，未追加审计或测试套件，未提交推送。[本次运行原件](../../_tmp_session/laptop_step_control/lead/program_run_1793200530310965400/run.json)；[实际命令](../../_tmp_session/laptop_step_control/lead/resume_4_cached_1793200500.sh)；[整合卡](../../docs/30_任务卡库/在办/laptopstep-1028/LS-LIVE.md)。

> **2026-10-28 真实车端直跑，未进入循迹（lead）**：用户要求直接加入车端规划/循迹，已实际起自有监护、原底盘节点、定位与dzbrain，不是bench/仿真。已恢复原先仅留TF的缺失底盘驱动；真实串口打开，定位N10，odom与转向所需端点均接通，原live契约通过。原程序随后因60秒收到0条field_layout自行停车退出exit3，未进入规划/循迹，未发车/出光/飞行。4号素材已全片执行但布局未构造/下发，当前断点仍是视频观测→布局输入。自有unit全部MainPID0、监护最后停至inactive；保留旧TF/他人进程。未改生产源码/默认配置/未提交推送。完整原件：[本次真实结束及回收](../../_tmp_session/laptop_step_control/lead/direct_car_finished_4.json)；[整合卡](../../docs/30_任务卡库/在办/laptopstep-1028/LS-LIVE.md)。后续复用已落盘素材，不再重放整片或增加审计。

> **2026-10-28 22:21 · 改用4号素材直接执行（lead）**：用户最新指令“不验证，直接放入制作好的程序链路里执行，要看完成的替代链路能否正常运行”。已停止额外审计/回归，成员已回收且无运行作业。Lead直接将[指定素材](D:/User/Desktop/无人机视频/测试素材/4.mp4)接现有来源→观测→布局入口，run=LSPROGRAM-1793197278785180900，2347帧/约78s已全部实际处理，1861帧定位、64帧来源候选；进程最终exit2/LAYOUT_REJECTED，未能构造首段。最后N10/M10均在诊断known中但最终observation=null，不能概括成全片起点不可见。未用旧实飞3结果代替、未改冻结候选/生产默认；未下发、未发车，后续规划/循迹/真实完成回传未执行。完整结果见[本次直接执行原件](../../_tmp_session/laptop_step_control/lead/program_run_1793197278785180900/run.json)。替代全流程仍未跑通，目标/卡保持在办，不继续额外验收。下文22:0x为前一素材阶段。

> **2026-10-28 22:0x · 笔记本替代本轮阶段交付，未发车（lead）**：本轮[laptopstep-1028](../../docs/30_任务卡库/在办/laptopstep-1028/00_树.md)实验桥接、租约锁存、逐段人工放行及视频来源适配已实现；stop-review独立复跑整套110/110（源53）与来源反例10/10，候选前后哈希稳定，只有实验功能窄范围接受。此前经唯一部署器窄推4份实验文件并做真实ROS锁停订阅/SSH挑战租约静态验证；自有服务已停，未启动脑/底盘/定位，未发车/出光/飞行。最终候选3e090903的新实飞3短窗8处理/8定位/0drop，完整H/mask/raw已保留，诊断known13格仍缺N10/M10，当前校核拒绝F11/K11，observation=null，首段NO_GO。不能将reason名称当作物理矛盾已证；格心边界与物理格边/全局锚定需定向诊断，不补known、不借旧桥、不调阈值。冷启动静态已有现役listener与fresh-start同步复用路径，不再报缺第三实现；真实冷启动、GID、到点停稳/断链停距均未验。生产冻结源码/默认配置未由本轮改动，未commit/push。整库门禁他ns仍有红，本ns不归档、不报可提交。证据及复跑见[本轮整合卡](../../docs/30_任务卡库/在办/laptopstep-1028/LS-LIVE.md)。

> **2026-10-28 本轮停在S1，未跑通（Lead）**：指定机端192.168.137.62 SSH首次与原地址一次重试均exit255超时（最后结果17:25:29）；车.164 SSH可达，仅只读。用户已确认N10朝M10/清场/人在旁可接管/本趟发射器物理禁用，并重申机IP应为.62。未启动dzuav、未写发射器、未部署/重启车栈/发车、未飞行/射击。P0八条已签，A1–A6未验收。A7本轮生产代码+0行/文件0/测试0/配置0/依赖0/仿真0；**子线扩展备用IP与网段探测违反N3，已记账且不再探测**。备份裁决不重开，未补做备份。事实、时序、证据见[PLAN](../../_tmp_session/今晚实跑/PLAN.md)。等待现场确认机端当前网络，不写修复方案。下方17:1x为开工快照，不是现状。

> **2026-10-28 17:1x 实机直跑开工 · S0 落账（lead · 未连车/未连机/未跑门禁/未改生产）**：用户 17:12 裁决已落 **D-057**（**覆写行动口径**，不改 D-056 记录、不修旧报告）：Lead 启动 dzuav；车端 `~/dzrobot` **覆盖部署**，**唯一命令** `python tools/deploy_to_car.py all`；**不旁路、不离线 truth map、不仿真（0）**、不预验证（部署前仅 `py_compile main.py` ≤5 s）；不改冻结区/他人写域/测试、不修问题、不新配置/依赖；**只跑一趟抓日志，不起飞不射击**。**P0 八条 8/8 已签**（2026-10-28 17:12），另登记 **1 条**问题：§一 第 2 条公式缺 0-based 限定（`row=行号−1`、`col=A=0`），字面代入 N11 得 −0.175 与第 3 条冲突——**只记不改源码**。新卡 **R-22**（树 `agc-1026/00_树.md`）；卡面/现场快照/A1–A7 清单/A7 增量基线统一在 [_tmp_session/今晚实跑/PLAN.md](../../_tmp_session/今晚实跑/PLAN.md)，不另建卡文件。**硬停**：S1 15 min 无帧 / S2 60 min（硬上限 90 min）或代码 >150 行 / S3 部署失败即停不换方法。**A7 开工基线**：根仓 104 files +2335/−839（status 133 行）、车仓 39 files +4900/−1065（status 58 行），收口按增量算。**待确认**：v4.3.1 **备份目录路径与备份时间两文件未记载 ⇒ 未知**（可核回滚锚点 = tag v4.3.1 / `0c846010` @ 2026-10-27 21:33:03 +08）；**场地清空尚无用户确认**（S4 前必须确认）；车 .164 / 机 .62 / 两端已连 / 车复位 N10 朝 M10 均为**用户报告、未实测**。**A1–A7 全部未测，不得预填完成；本次仅文档签字，不代表实机验收**。

> **2026-10-28 16:5x 首次实车冷启动（lead · 只测不修 · 历史记录，行动口径已由 D-057 覆写）**：车/机在线，车复位 N10 朝 M10。`start_all.sh` 起 7/7 节点，dzbrain 布局模式；**`/dzlink/drone_to_car publishers=0`**、等 30 s 收到 **0 条 field_layout** → 不规划不出发（设计如此），**无移动无开火**。根因：机端**没有 field_layout 发射器**（`dzbrain_bridge.py` 只发 intel 四类；`~/dzlink_tools` 不存在；全盘无 layout*.py）；车端 build 旧（main.py 2297 行 vs 本地 3162、等待 30 s vs 60 s、无降级/归档）。日志已取回 `_tmp_session/live/car_logs/`（7 份）。按用户令**先不修**；待拍板：机端补发射器 vs 车端本地发射器验步进。见 D-056。


> **2026-10-27 23:3x 三线整备开工（lead · 用户拍板）**：用户裁决三件——**P1 一次性授权**（车宽 0.135→0.17、桥洞开口 0.33/桥墙 0.02、清 0.0875 虚拟冗余墙；同批 wait 30→60、ttl 4→0）、**A1 开火路线选 A**（保持 C++ 内部判据只喂 lead）、**运行归档落点自定**（定为 `dzrobot_v2/artifacts/runs/<UTC ts>/`，只写不读）。Lead 建卡 **R-12..R-20**（P0/P1/G1/S3/M1/S1/S2/M2/A1）并登记 frozen.json（physics.py/main.py/drone.py/config 批）。派出 4 个子智能体，路由 **raccoon/sn-deepseek-v4-1-flash-1017**，该模型仅 on/off、**max 不可用**，取最高档 **on**：p1-geom(9aff0fda)、rt-gate(6719d8a0)、coord-s1(39437d35)、aim-a1(d3eaabdd)。运行面按 D-051 降级，只留 3 硬拦（急停/激光联锁/地面线边界）；离线 §4 门禁不降级。界内未 commit/push/部署，未连车/未飞/未出光。

> **2026-10-28 00:0x A1 判据链回放裁决（lead · 未连车/未出光/未打靶）**：Lead 独立复跑 `_tmp_session/A1-AIM-STAGE1/` 三脚本（a1_numbers exit0、`--do-assert` PASS 7/7、gate_replay exit0），数字与作者报告逐位一致：匀速段 `|dx+lead|` 41.0→67.0（+T·v，−63.4%）/ 14.9（−T·v，+63.6%）；`dx≈0.245·v`（CI 0.127–0.360）；判据链五种 lead 配置开火时刻全同 51.724/61.212 s，仅 `hold=5` 复现实车 max|dt|=0.05 s（默认 1 出 12 发）⇒ **车端加载 5、仓内 robot.yaml:187=1**，第三例仓内≠车端。**裁决**：符号锁 **+v·T**、`−T·v` 假绿禁用；路线 A 匀速段结构无效（lead 只进 cpp:300 门、伺服 cpp:113-122 不读），预期改为「门更严」；**不花弹药验 A**、A2 暂停；伺服 1 行 + 稳定门语义待用户批。见 D-052 与 R-20 卡。

> **2026-10-28 00:3x R-17 S1 复核（lead · 未连机/未出光）**：Lead 独立复跑 `test_drone_corridor` **48/48 rc0**、`--require-production-60` **49/49 rc0**、`replay_corridor_injection` **6/6 rc0** —— 原报「48/49、5/6 仅 F 红」已过期：rt-gate 在途 `main.py:_wait_for_field_layout` 改用 `_spec_default`，生产等待 **60.1 s**（yaml 无键亦成立）⇒ **取消** p1-geom 补 yaml 的派工。V4 判因**退回**：18:50 `drone_in pubs=0` 与机端 SSH 超时同现；真发射端是机端 `~/dzlink_tools/layout_batches.py`（不在仓），`perception.protocol` 缺 field_layout 属 D-039 已删构造器的遗留测试残影，**禁止补回**。树 R-17 已存在（23:37）。drone 当前 SSH 超时 ⇒ 空地实测仍缺机端。见 D-053。

> **2026-10-28 01:0x P1 复核 + R-21 收尾（lead · 未 commit/未连车）**：P1 几何真值**接受**（V2/V5/V6/V8 绿；V7.2 仍 FAIL：洞口前 1 格起转 **+5.9 mm 放行**，记已知缺口，不为变绿改判据）。Lead 补两处写域外高危：`sim.py` bridge_modeled 改「有桥洞即建墙」（探针 0.09 m→**21 拍违规**）、`route_check.py` 车宽 0.17 + vehicle 段改读 `/dzactuator` + 桥墙随配置（carloop trajectory rc0 PASS；sixth legacy **rc0 / 雷 8-8 / 靶 2-2**）。sim 无图 ⇒ `field_modeled=0`，桥洞指标待接图。见 D-054 与 R-21 卡。

> **2026-10-28 01:1x rt-gate 五卡复核（lead · 未 commit/未部署/未连车）**：Lead 独立复跑 **69/69**；`config_selfcheck` 0/0；`check.py` **GREEN**（`run_archive.py` 已登记复用清单）；`sim_cli` **118.12 s / 0.0300 m / 74**（与改前同）；`test_stepper` **117/118**（仅 12.11 存量）。**Lead 落配置**：`robot.yaml step.layout_ttl_s: 0.0` ⇒ `gate_policy=warn` 实测，A 区降级在车上生效；numbers `--fix-sources` 自愈 **8** 条 → exit0。G1/S3/M1/S2 **接受**；M2 **部分**（六键/只写不读/同名 RUNX-1 已证，**SIGTERM 未证**：Windows 无 POSIX 语义，真机待实跑）。非本线：`test_corner_cut` 90/91（5.9 预算，pursuit/trajectory 并行候选）。见 D-055。

> **2026-10-27 22:2x R-8 决策 A 落生产工作树（lead · 未提交/未部署/未连车）**：生产 `src/dzbrain/dzbrain/route/planner.py` `c14dadd4`→**`405d8e8d`**（+137 行），并按决策 A 改写 4 条在途断言（`test_dual_shoot.py`→`cc77bf4a`、`test_autoplan.py`→`4da2a542`）；三者与作者隔离副本 `iso/cand` 逐字节相同。**A/B 隔离**（两棵当前生产树仅差该 planner）：第六图 legacy/tasks **由 rc=2 整趟判死 → rc=0 / 68 token / 单站 `B4*` / degraded=True**，且与 HEAD `lead_head.json` **逐 token 一致**；carloop 62/63、field 54/53 **逐 token 零变化**。`route_check` rc=0（雷 8/8、靶 2/2、射击点#48）、`sim_cli` rc=0（108.24 s、末点 0.0285 m，**但 `out_of_bounds=74`/`field_modeled=0` 存量未解，不得当空间安全**）。测试 `test_dual_shoot` 37/37、`test_autoplan` 182/182；规划相关 10 套 pre/post 无新增红。§4 门禁：gen_index rc0 / doc_policy GREEN / numbers rc0 / check.py GREEN / check_links 0 / **anchors 新增红 2，全部落在他人新卡 `lead-1023/DOC-LIB-TIDY-20261027.md`（裸文件名），与本改动无关**。**待用户拍板**是否本地 commit（D-044）；回滚点 `_tmp_session/AGC-RECTIFY-20261026/lead-land/preimage/`，禁 `git checkout`。见 [REPORT](../../_tmp_session/AGC-RECTIFY-20261026/lead-land/REPORT.md) 与 [R-8 卡](../../docs/30_任务卡库/在办/agc-1026/RECT-PLAN-FIX.md)。

> **2026-10-27 网络固化（lead · 已实测，现场请勿再用旧地址）**：热点 **LapFox000** / 0d000721。**小车 = 192.168.137.164/24**（LapFox000 静态 manual，autoconnect 优先级 100，默认路由 192.168.137.1，ssh 别名 `dzrobot` 已直连 IP，不再依赖 mDNS）。**无人机 = 192.168.137.62/24**（静态 manual，优先级 100，免密 SSH，别名 `dzsmart`；USB 备用 `dzsmart-usb` = 192.168.55.1）。Windows 防火墙拦入站 ICMP（ping 不通本机属正常）；热点面板「已连接设备」只列 DHCP 租约，静态设备看不到不代表没连上；换网后须重启 `dzlink_supervisor`，否则车端显示无人机 offline。**沿革**：下文较早记录的 192.168.0.4 / 192.168.0.5 / 137.79 均为历史地址，仅作参考，不可用于现场连接。

> **2026-10-27 19:02 实跑续轮1（lead）**：未重发无地图任务，未点动/回归。无人机旧地址与当前名称解析地址SSH均超时；热点仅见车端有效邻居，不能据此证明无人机断电。待恢复机端网络或获得当前IP，才能续取真实全片结果；既有桥洞语义缺口未解除。未追加运动/开火/飞行命令。见[当轮连接证据](../../_tmp_session/AGC-RECTIFY-20261026/video-step-20261027/lead/goal_round1_connectivity.json)。

> **2026-10-27 18:50 全程正式入口实际执行（lead）· 未发车/未完赛**：按用户“不要测试、直接跑全程”运行车端既有 `/usr/bin/python3 -u -m dzbrain.main --config /home/duzhong/dzrobot/config/robot.yaml`，未使用 bench/dry-run/force、未恢复固定图、未改生产。真实 live 契约通过，定位有发布方；无人机输入发布方0，等布局30秒、实际收到0条，随后brake/shutdown并以exit 3退出。程序没有进入任务组、没有发出运动/开火任务。无人机别名及设备本名均解析192.168.137.62且SSH超时，完整原生结果未能续取；已有partial布局的桥洞语义仍未解决，不把恢复连接当作全链完成。首次带timeout包装被旧重复实例检测误判，包装已去除；该次包装退出码采集也有错误，保留原记录，不计为业务结果。当前正式结果：[启动](../../_tmp_session/AGC-RECTIFY-20261026/video-step-20261027/lead/full_run_direct_result.json) / [结束](../../_tmp_session/AGC-RECTIFY-20261026/video-step-20261027/lead/full_run_direct_completed.json)。原四个基础节点未清场，未重启他人进程。

> **2026-10-27 本轮实体起栈（lead）**：用户已明确现场值守/可急停并要求视频驱动完整实跑，含有条件比赛打靶；不授权飞行。task-27已真实起底盘与定位，PGID 60127/60169，定位约50Hz，自报N10/+y；采样时cmd_vel无发布者，未发运动指令，未起云台/裁判。相机真实帧已回取见[现场画面](../../_tmp_session/AGC-RECTIFY-20261026/video-step-20261027/entity-car/frame_n10_181012.jpg)。机端清晰片段60帧均有地标和定位，已取回[原生观测](../../_tmp_session/AGC-RECTIFY-20261026/video-step-20261027/lead/clear_clip_trace.jsonl)，这不代表物体类别或整图可靠。整片原生识别正在机端隔离网络运行，task-28接真实布局，尚未向实车发布布局或任务。当前目标实际闭环，不再增加离线审计；没有把起栈记作发车、打靶或完赛。下列较早NO-GO记录按各自时点理解，生产默认配置未改，不提交推送。

> **2026-10-27 A对短测后 · 仍NO-GO**：A整对结构匹配，隔离短测381条逐帧记录、零格号地标/零定位，rc124为超时非EOF；模型类别语义未证，不定为生产配对。旧观测候选漏world且存在NaN/越界格/时间校验缺口，独立评审及Lead复跑后拒绝作为交付。已只读补齐部署网格源码，纠正“缺map文件所以不能落格”的伪阻塞；当前只做task-24完成单错配及task-25观测接口定向修复。未起车栈、未发路线/动车/出光/飞行，生产冻结。最新裁决见[集成卡](../../docs/30_任务卡库/在办/agc-1026/RECT-INTEGRATION.md)。

> **2026-10-27 16:22 视频步进准备（lead协调）**：用户确认N10朝M10、录像同场与现场值守/硬件急停等条件；task-17/18/19分担视频链、停车链与设备只读预检，Lead不重复执行。指定视频hash复核一致。task-17确认原生无field_layout接口，固定图emit_step_v3禁用；task-20正补隔离观测导出/保守布局候选，task-21只准机端30秒空netns推理探针，不能放行车。保留打靶目标，但先禁光，实际出光另确认；技术放行仍NO-GO，未起车栈或动车。此后新子调用使用Raccoon DeepSeek-V4.1-Flash-1017/on，先前在跑任务不重开。详见[本轮集成卡](../../docs/30_任务卡库/在办/agc-1026/RECT-INTEGRATION.md)。

> **2026-10-27 16时连接窗口（lead）· 仅只读/NO-GO**：用户提供当前连接入口，两端免交互SSH已成功，硬件确认Jetson Orin Nano与OPi 5 Pro。未见关键词匹配业务进程，不代表物理禁动；车端两处auto_start文件值均false，选定7文件有5项与本地不同。机端原生入口已找到，两目录同名引擎SHA不同；文件视频源码仍有UDP广播路径，不能直接当无副作用离线模式。未运行推理/起ROS/部署/改参数/运动/出光/飞行；下一步须固定模型身份并证实禁发隔离，待现场禁动禁火与机端临时推理许可。保活未重启。见[本次证据与边界](../../docs/30_任务卡库/在办/agc-1026/RECT-INTEGRATION.md)；以下较早状态按记录时点理解。

> **2026-10-27 离线检修续办交接（lead）· NO-GO**：task-10至14已交回并独立裁决。已复现跨趟旧布局/回执混用、节点失败却启动成功、空图失败闭合缺口及零有效帧仍rc0；尚未生产修复。回归退出传播一行候选、临时保活快照容错仅限域接受。视频已用既有ONNX评测入口跑完全片，但无可靠侦察地图；独立复核推翻“负z即地下相机/两符号同负”推论，不移植为NED高度闸。保活已于昨夜23:01正常释放，[释放记录](../../_tmp_session/AGC-RECTIFY-20261026/offline-night/heartbeat-release.json)，今天未续期；无运行中的后台job。未改冻结生产、未连车机、未部署/提交/推送。整体集成及现场完赛/双有效点/命中仍未验收；本轮收口与待办见[现行集成卡](../../docs/30_任务卡库/在办/agc-1026/RECT-INTEGRATION.md)。

> **2026-10-26 较早续办记录 · agc-1026（lead）· NO-GO**：按deepseek-account/deepseek-flash/max派出独立前沿评审与复跑入口整改。**前沿候选拒绝合入**：独立反例及Lead重跑确认运动缩格可从WAIT_INFO重回DRIVING、新版不覆盖当前格也续跑，且无新版或版号抖动时等待无界。保留生产锁存停车。**DDS等待原候选needs_revision**；隔离v2现经独立28例窄范围接受（原候选同探针8反例），仅限等待与日志；未合入/部署，真实DDS与严格总时限未证。sim_cli两版本输出相同但均越界74拍、field_modeled=0，不能宣称空间安全。上轮13PASS混有旧日志与SKIP已撤回；作者运行器交回后Lead真跑19EXP/3BIND/0SKIP及7个注入场景。R7原136/136、隔离141/141实跑；R8单站降级与两fixture非回归实跑，但非运动/打靶验收。Lead把ACCEPTED改为REPRODUCED并显式NOT_ASSESSED/launch_go=false，此口径修订已独立PASS_LIMITED，Lead复核检查器rc0；elapsed强转、仓内TMPDIR自测分类及R11临时out侧写局限均留账，不当作全工具安全保证。定位17m不能直接除3认定实丢5.7m；缺图仍空间UNKNOWN，不固定夹具或换obstacle_blocks制造GO。详见[本轮证据与复跑](../../docs/30_任务卡库/在办/agc-1026/RECT-INTEGRATION.md)。本轮未改生产冻结源码/配置，未部署/发车/出光/飞行/commit/push。用户随后确认N10朝M10及人工急停，Lead只读连车：基础栈与step_emitter_v3在线、未见dzbrain及空地布局话题；零末帧速度不等于定位对齐。待确认控制归属，不擅自接管。文档全链最终仍有数字守卫RED（数值重算通过、文档漂移/异文未清），不报整卡收口或可提交。

> **2026-10-24 17:2x 正式容器 `agc-1024` 上岗（lead）**：已用 `agent_teams_create(approval=automatic)` 建正式容器（此前 17:0x 的 spawn 层四卡已并入），成员 **t-car / p-aim / dl-drone / e-integ**，首轮四任务并行：T1 进度契约、P1B det_node 3→8 字段修复候选、DL1 二进制→JSON 薄适配、E1 四方对账。本轮**只做离线与只读**：不部署、不重启、不发车、不出光、不飞。
> **Lead 只读取证（2026-10-24 17:2x，SSH 192.168.0.4，零写入）——F9 target=none 根因升级为「选错检测进程」**：① `dzrobot2/src/rknn_pt/src/det_node.cc:28` 建 publisher，`:104-106` **只填 3 字段** `[dx,dy,1]`（无 cls 映射/无 box_w/box_h/conf）；② 同一台车的 `dzrobot/src/rknn_pt/src/det_node.cc:273-281` **填满 8 字段** `[dx,dy,cls+1(1蓝2红),box.w,box.h,kpt_dx,kpt_dy,kpt_conf*100]`，与上午 mvlog 8 字段逐位对应（w p50=133 / conf p50=99）；③ 两个 install 二进制与模型都不同（dzrobot 二进制 10-24 09:06 构建、模型 8.1MB@10-23 19:12；dzrobot2 二进制 10-20 21:51、模型 9.9MB@10-20 20:15）；④ **两仓同名包 `det_node`，实测 source dzrobot/install/setup.bash 覆盖在 dzrobot2 之后 `ros2 pkg prefix det_node` 仍解析到 dzrobot2/install** ⇒ 15:43 那次栈起的是 3 字段版。结论：**这是进程/环境选择问题，不是必须重编译的代码缺陷**；最小修复 = 换回 dzrobot 的 8 字段 det_node 进程（只换一个进程，不动 lead/EKF/停止锁存）。待用户决定：下一次现场窗口是否批准该窄启动变更（含 `ros2 topic echo /offset_center --once` 看到 8 个数的验收判据）。
> **设备可达性（17:2x）**：车 192.168.0.4 ping 11-19ms、SSH 免密可用；无人机 192.168.0.5 ping 29-44ms。当前**无** det_node/视觉/云台/dzbrain 进程在跑（上趟栈已停），车位须重新确认，不能沿用上趟末位姿。
> **2026-10-24 17:1x 增设攻坚席 `h-ops`（用户指令）**：模型 `codex/gpt-6-astra`（用户口径 GPT6_Astra），思考强度 **`max`**（该路由只提供 low/medium/high/xhigh/max，**无 "Ultra" 档**；max = Maximum reasoning depth for the hardest problems，按最高档落地，已向用户说明此名称映射）。**只由 Lead 召唤**：任何成员直连一律拒收并回「请把问题与证据交回 Lead」。**只接硬阻塞**（现有证据互相矛盾 / 两条独立路径都失败 / 根因不明且卡住 M1–M6 之一），不接日常问题、进度催促、重复审计、写卡收口。默认只读、**无写域**；产出=根因排序+判别实验+修复方案（落 `_tmp_session/agc-1024/h-ops/`），由 Lead 派给对应 owner 落地。无待命任务，不被召唤不产生调用。树节点 AGC-15。

> **2026-10-24 17:0x 团队 agc-1024 上岗（lead）**：DeepSeek 五角色团队已建，ns=`agc-1024`（登记表已加行，树 `docs/30_任务卡库/在办/agc-1024/00_树.md`）。成员与写域：**t-car**=tasks/{tracker,localization,mission}.py + dzbrain/tests 新增测试；**p-aim**=dzrknn_dataprocess/**（云台C++）+ aim.py；**dl-drone**=dzsmartscout/tools/link/** + 机端链路；**e-integ**=tools/deploy_to_car.py（其改动由 lead 独立评审）；**lead**=树/卡库索引/TEAM_STATUS/冻结登记。首轮四卡并行：T1 进度生产者-消费者契约、P1 F9 target=none 归因、DL1 二进制:50000→JSON 薄适配、E1 四方对账。卡面写域即文件锁，mission.py 单写者为 t-car。本轮**只做离线与只读**：不部署、不重启节点、不发车、不出光、不飞。实测网络：车 192.168.0.4、无人机 192.168.0.5 均可 ping。
> **2026-10-24 DeepSeek接手交付**：事实入口改为[空地协同接手](../../docs/00_项目总览/交接_20261024_空地协同接手.md)，详细任务原位更新[推进计划](../../docs/10_agent提示词/04_场景/空地协同闭环推进.md)。当前保留第二趟已证倒车后前进及F9停站离站；进度三处写入、重复访问提前记账、停滞时钟/H8覆盖分别交T线验证，P线先解释F9无目标，D/L接续机端真实数据/协议与飞行。66文件证据ZIP已校验。此轮只编制交接，不操作设备；程序PID与车位须按最后确认时点理解。

> **2026-10-24 第二趟实跑结果（/root，当前）**：用户归位并批准后，定位里程符号+Tracker倒车确认两补丁已窄部署，两包构建2.85s/exit0，备份100文件，运行安装哈希一致。第二趟真实倒车53拍完成，随后恢复前进，F9停站搜索6.10s未锁靶后离站、云台SCAN→IDLE，禁火。运行100.7s后主agent因路线进度旧段与E9强制跳点执行停车；不是drift_stop，八任务已退出。未到C5/N5、未第三次发车。定位626901保留，新Mission626921已退出。回滚点carloop_reverse_20261024_153942；证据与具体判据见 [CAR-PREFLIGHT](../../docs/30_任务卡库/在办/carloop-1024/CAR-PREFLIGHT-20261024.md)。本地正式源码尚未同步这两份车端候选；未提交/推送。

> **2026-10-24 首趟实跑结果（/root）**：用户要求先跑再采数，已低速0.10m/s、54点F9/C5禁火发车；在首个H8倒车区失败停住，未到射击站/终点。直接停因drift_stop限速0；负速度期间定位仍正向增长。已回取实际serial_reader，发现hypot距离恒正且未乘倒车方向；另有Tracker仅1拍即误判倒车入口完成。已manual_override接管并结束本趟Mission，激光隔离保留，未二次发车。详见 [CAR-PREFLIGHT](../../docs/30_任务卡库/在办/carloop-1024/CAR-PREFLIGHT-20261024.md)。后续聚焦这处真实失败，不扩展检查矩阵。

> **2026-10-24 部署及静止恢复完成（/root）**：用户批准的四文件已窄部署，源/安装哈希吻合；两包ROS编译45.1s、exit0，96文件备份已校验。随后单独获准恢复camera/chassis/loc/gimbal，DDS118恢复；云台外门禁true、LaserShot输出隔离到/carloop/inhibited，裁判和Mission未启动，车未发。用户确认N10朝M10；云台最终IDLE、目标/反馈残差2/5 tick。54点替代路线虽静态门禁绿，动态仿真横偏最大0.432m且场地约束失败，未放行。用户要求不再检查，已停止追加测试。详情见 [CAR-PREFLIGHT](../../docs/30_任务卡库/在办/carloop-1024/CAR-PREFLIGHT-20261024.md)；未提交/推送，文档门禁旧阻塞未解除。

> **2026-10-24 14:31 跑车准备（/root）**：只读直连192.168.0.4；相机/底盘/云台DDS仍绑定旧137.79；无dzbrain/定位/裁判进程，云台argv外门禁false/max_shots3。真实车端源码窄合并候选已备，原调优保留；Python专项32/32、自动规划177/177、云台58/58、C++30/30、R3部署器187/187。四文件部署预演退出0，尚待用户批准。62点F9/C5路线有靶板40mm/挡板20mm足迹告警，两轮未消；tour入口footprint_gate=error仍不拦，候选配置拒用，不发车。见 [CAR-PREFLIGHT](../../docs/30_任务卡库/在办/carloop-1024/CAR-PREFLIGHT-20261024.md)。未部署/车端构建/节点重启/出光；以下较早离线结论不代表实车就绪。

> **2026-10-24 carloop-1024 更新（/root）**：双站规划、离站回正、权限500ms失联撤销已完成本地修改。当前图 F9/C5、62 token/22.75 m，几何未改；双站35/35、自动规划177/177、Python云台58/58、Aim136/136、真实C++函数编译26/26均过。Mission修前后229/246，17旧失败无增减。已登记本ns与冻结写域；共享索引外部行保持不变。全库收口仍被既有R2文档位置与SSH-DIAG两条索引断链阻塞，未提交/推送。卡见 [carloop-1024](../../docs/30_任务卡库/在办/carloop-1024/00_树.md)。未完整ROS构建，且本地源码与车端版本不同；未部署/发车/出光，后续须用户确认。

> **2026-10-24 · 治理形态裁决（用户 A 案，lead）**：当前四线跑在 spawn 层（`team_task_*` 共享板 + durable 成员），**未建 `agent_teams` 容器**（无 team plan / 无质量合同 / 无 attempt 接管）。A 案：不迁移在跑的 T1/P1/DL1/E1，**在 M2（双站打击验收）前用 `agent_teams_create(approval="required")` 建正式容器**，把 M2/M5/M6 的门禁做成带 objective+acceptance+verify 的 review/repair 循环；T/P/DL 成果经卡片交接，不重跑。
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
- 证据：只读快照（原 artifacts/tracking_readonly_20261020.json，2026-10-29 起归档于 [scatter_archive_20261029.zip](../artifacts/scatter_archive_20261029.zip)）；后续：[循迹团队执行方案](../../docs/history_cache/01_旧文档归档/docs/agent_handoff/30_循迹团队执行方案.md)。旧 E4/E5 停位和 auto_plan=false 仅是上次快照，不能继续当当前事实。

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

## 9. 文档库指标（2026-10-24 P3 收口复核 · t7）

> 来源：`python docs/_工具/check_doc_policy.py`、`python docs/_工具/check_anchors.py`、`python tools/wf/check_links.py` 实跑；非估数。口径见 AGENTS §8.9。

| 指标 | 值 | 备注 |
|---|---:|---|
| 库内篇数 | **184** | check_doc_policy「库内」桶（docs/ 白名单内 .md） |
| 库外受管 .md | **63** | 基线只减不增 |
| 红线 | **0** | R1/R2/R3/R6/R7/R8/R9 全清（R6 由 lead 更新 README 计数后消掉） |
| 过期候选（现行且 >180 天） | **0** | 提醒仅 Y9（_tmp_session 失效引用 321，基线 250） |
| 断链 | **0** | check_links：checked 191 / broken 0 |
| 锚点新增红 | **0** | check_anchors：存量红 306（基线 240），黄 195 |
| 20_资料索引库实扫 | **63** | README「应为 63 篇」一致 |
| 文档异文 | **6** | check_numbers 默认只报不判：前视区间 / 视觉 thread_num / 流水线延迟 / 巡航 / 轴距 / 丢靶回扫 |
## 17:4x 更新（Lead 亲自复核）
- **AGC-3/t3 DL1 完成并经 Lead 复跑**：link 105 OK（pipefail）、dl1 51 OK、verify_dl1 exit 0、坏帧 5 类全拒收、dzsmartscout 生产树零改动、快照 sha256 9ff259a3… 与教员原件一致。候选件全在 _tmp_session/agc-1024/dl1/。
- **DL1 四条裁决已下**：①解析核入仓一份=钉死快照（sha256 PIN + provenance 单测，禁 .md）②合入窗口开（仅三个 link 文件）③单写者=替代不并存（双写即红）④空类别整键不发（缺失≠空，合成 confidence 不参与阈值）。
- **新增卡**：t5=AGC-17 合入（dl-drone）；t6=AGC-16 transform_fusion tf2_ros.transformations 导入失败修复（dl-drone，排队）。
- **在跑**：t1 t-car、t2 p-aim、t4 e-integ。
- **物理阻塞（待用户窗口）**：D1 雷达 eno1 carrier=0（插线）；D3 无 /dev/ttyACM*、PX4 板未插；D2 机上感知未启动取不到真样本；车端 UDP:50000 无监听（需窄部署）。

- **AGC-2/t2 P1B 完成（Lead 复跑 verify PASSED）**：3 字段 vs 8 字段源级定案；车上有两份 det_node（776K@10-24 09:06 / 3.86M@10-20）与两份模型（8.1M / 9.9M），零代码换进程即修。**等用户现场授权 GO**（只起 camera+det_node，V1-V3）。
- 新增卡：t7=AGC-19 上车验证（p-aim，等 GO）；t8=AGC-18 fire_dz_far_px 漂移取证（只取证不改值）。
- 现场实测：车 192.168.0.4 可达；当前无 det_node/云台/dzbrain 进程（V1 前须确认无残留，防双发布）。

- **DL1 已提交并双推**：commit a5a662df（5 个文件，link 195 OK）；github/gitee main 均 a5a662df，ls-remote 已核对。
- **门禁修复（Lead）**：check_anchors.py 空 max() 崩溃守卫；回报模板两处相对路径；20_资料索引库 README 篇数 48→49；gen_index 登记 2 篇。现 doc_policy GREEN / wf/check GREEN；anchors 尚余 13 条新增红全在他人会话文档（纪要 6 + 日志线 7）。
- **mission.py** 已登记 frozen in_flight（owner t-car，到期 10-31），T1 最小补丁进行中。
- **AGC-19 阶段一**：p-aim 交付脚本+日志分流+dry-run+只读预检 13 PASS/0 FAIL，车上零进程；等用户 GO。V6 证据加强：per-run 5_gimbal.log 1425 条「字段不足(3<5)」覆盖 14:52–16:51（含 15:43 趟）。

- **DL1 闭环（18:3x）**：e-integ 独立复评 **PASS**（RESULT ADVERSARIAL CLEAN；B0 绿、M1–M4 四变异全红、A1–A9 全绿；A4/A8/A9 三个原 bug 均判红）。提交链 a5a662df → 9c722bfc（快照 MANIFEST）→ a066e7c8 → e58aa4a9（守卫修复）→ 74af8812（README 放行），两端 main 一致。
- **AGC-16**：tf_compat 32 单测绿；FAST_LIO 是未初始化子模块 ⇒ 改走 fc-audit/fixes/02-tfcompat 补丁包（含 CMakeLists install 行）。
- **门禁**：doc_policy / wf/check / check_links 全 GREEN；anchors 新增红 27（18 在我们两张卡，p-aim 标注中；9 在他人文档）。
- **现场**：车 192.168.0.4 通；无人机 192.168.0.5 **掉线**（ARP 无效，需重新上电/联网）。

## 21:4x doc-1024 收口推进（Lead · 2026-10-24）

- **t5 热词缺口清单已完成**（att 929ce48f）：242 待提炼 → 173 关键词 → 84 覆盖 / **89 缺口**；交付 `docs/20_资料索引库/赛事与治理/热词缺口清单_20261024.md` + `artifacts/wordcloud/热词_2026W43.csv`；门禁 gen_index/check_links/check_numbers 全绿，check_doc_policy 仅既有 R6（README 60 vs 实扫 62，未动）。
- **R6 漂移待消**：README「应为」计数落后 2 篇，已交 t7 处置，lead 不擅自改 README。
- **t7 已派 doc-archivist-ops**：6 条门禁 + R6 处置 + 主题矩阵（89 缺口三档分类）。
- **t10 已建并派 doc-mcpsmith**：dzdoc 8 个工具端到端冒烟（t6 只证明握手，本卡证明真跑得通）。
- **MCP 已交付并挂载**：server `docs/_工具/mcp_dzdoc_server.py`（sha256 d15131f5ab30…，19121 B，477 行，8 只读工具）；注册块 `dsh-dzairground-ssh/cordis.patch.yml`（sha256 5a3bb92fb5ee…，1082 B，L10 起，stdio / serverName dzdoc）；本会话 list_mcp_resources 返回 OK。

## 2026-10-25 traj-1025 本轮登记

/root 负责共享登记与独立复核；原三个子agent已交回本轮写域。三层循迹候选仍离线、未部署。用户明确AGC-T1未交还tracker/localization/mission，运行链及全仓+X西/+Y南坐标迁移只在`_tmp_session/traj-1025/frame-migration`副本评审，不覆盖共享原文件；PIVOT/ALIGN尚未删除。开场基线41套/18红；pytest临时补装后因原harness导入SystemExit退出3，原生套件另记。用户追加64新图压测已完整跑完：49规划失败、8编译失败、7闭环完赛、全部已实现验收0/64；7张均车心8/8、采样区域通过，但赛时206.42–265.46秒。固定输入、同版哈希与逐图结果见本ns INTEG卡和random64-results。坐标迁移最终原生全量复验41套/18非零退出，与开场红套件集合一致；corner_cut仍89/92，固定第七图障碍净空、动力学与约束冲突、180秒三项未通过。外测资料暂无，外测仍未通过；新图首动方向接口、jerk硬cap冲突、跨端场地帧接线未完成。冻结门禁仍有3项任务外未登记改动，锚点守卫12条任务外新增红；不能代填owner，不满足提交/双推条件。
