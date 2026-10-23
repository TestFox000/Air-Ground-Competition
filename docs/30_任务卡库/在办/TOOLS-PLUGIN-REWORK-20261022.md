---
来源: TOOLS-ORGANIZE-20261022（缺口清单交接）、本会话执行记录（2026-10-22 22:14–22:50）
取证: 实/机检
更新: 2026-10-23
取代: 无
status: in_progress
---

# TOOLS-PLUGIN-REWORK-20261022 · 统一空地插件重做

- owner: lead
- created: 2026-10-22（自 TOOLS-ORGANIZE-20261022 拆分，整理半已归档）
- 写域: tools/plugins/dsh-dzairground-ssh/**
- 非目标: 不碰 dzrobot_v2/src 冻结区；不碰另两插件树（除非 divergence 检查要求）；不在本卡挂载 profile bundle（另需用户窗口）

## 台账 / 复用检索（5 必填）

- 台账检索: docs/20_资料索引库/历史/02_并入与改名_20261018.md（S0~S6 全史，「已建成、未挂载」）；TOOLS-ORGANIZE-20261022 缺口清单五条为开工底
- 复用检索: 全部修复在既有文件内改（client.js/index.js/src/files.js/scripts/deploy.mjs + 测试），无新脚本、无新轮子
- 验收命令: 见「全套自检结果」表；复跑 = node scripts/<name>.mjs 逐个 exit 0
- 回滚点: git 提交前 v1 树 = HEAD；逐文件哈希可由 git diff 复核

## 缺口销账

| # | 缺口 | 处置 | 证据 |
|---|---|---|---|
| 1 | deploy.mjs 无 --dry-run（README 声称有） | **已修**：--dry-run 逐文件报 would-copy、零写入；dry-run 不再检查安装目录（预览不该断言安装）；顺带修 walk(installed) 在空安装目录 ENOENT 崩溃 | node scripts/deploy.mjs --dry-run → 112 would-copy、exit 0、目录零变化；实跑 exit 1 报 FATAL（安装目录不存在时如实失败） |
| 2 | bridgeConnectorConfig 默认角色语义与测试矛盾 | **已修**：role 改为显式传参；无 role → 遗留 S1 面 api/dzairground-ssh/vnc（host 仍注册该路径为 car 通道，测试契约）；car/drone 显式路径不变 | test-client 桥接段 PASS |
| 3 | connCarBridge/connCarBridgeHint/connCarBridgeDone 缺 zh+en | **已补齐**（文案对齐既有 connNodeBridge 系列口径） | test-client 字典段 PASS |
| 4 | README 安装位置过时 | 已在前卡（整理）销账 | TOOLS-ORGANIZE 执行记录 |
| 5 | /files SSH 失败返回 400，违 honest-offline | **已修**：files.js 以 classifyFailure 区分 transport（ssh 起不来/网络/认证）与语义（路径不存在/目录/超大）；index.js statusFor：transport→503、语义→400；client 视图 api() 抛错文案原样透传，无需改 | test-host 无车分支 3 条新断言全 OK（503+transport 标志、文案指传输不指路径）；有车分支保留原断言 |
| 6 | **新发现**（本卡执行中）：openInTerminal 签名漏 role，rename 行恒抛 ReferenceError 被 try 吞——终端标签从不改名 | **已修**：role 入签名并透传；缺陷注释落盘 | test-client: renames=["小车 duzhong@stub.local"]，CLIENT TEST PASSED |
| 7 | **新发现**（整理卡连带）：check-css-namespace 对照路径搬坏（拼出 tools/plugins/tools/plugins/...）ENOENT 崩 | **已修**：同包兄弟目录 = here/../../dsh-dzrobot-ssh | 复跑 hazard 0、exit 0 |

## 测试基建修复（无车环境下的确定性）

- test-client：连接流程原样真跑宿主路由，但 /car/probe、/car/status 用本机替身（同 test-host 的 TCP VNC 替身思路，SSH 不可 spawn 则流程测不了）；/drone/* 不拦——dzsmart.local 真不可达是 S7 断言的前提。**不改任何断言**。
- test-host：files 段按 probe.state 分支——有车走原全断言；无车断言 honest-offline（503+transport），SKIP 需真车项。

## 全套自检结果（本沙箱，node v24.21.0）

| 脚本 | exit | 备注 |
|---|---|---|
| check-css-namespace | 0 | hazard 0 / keyframes 0 |
| test-vnc-guard / wrapper / connect | 0 ×3 | |
| test-package-files | 0 | SKIP：profile 未装本包（如实） |
| test-client | 0 | CLIENT TEST PASSED |
| test-host | 0 | HOST TEST PASSED（无车分支断言 503） |
| test-s2 | 0 | |
| test-sim-overlay-ab | 0 | |
| test-sim-parity/incremental/map-render/drone-render/overlay-fix | 1~2 | **全部沙箱 EPERM**：spawn python 被拦 + D:/…/dzrobot_v2/artifacts mkdir 被拦（探针复现同错）；SIM_ROOT 经 sim-root.json 正确解析至 dzrobot_v2，非搬家后遗症。待用户机复跑收口 |

## 未闭合（如实挂账）

1. sim 家族 5 项需在无 EPERM 沙箱的用户机复跑确认（命令上表）。
2. /files 需真车段验证（check-files.mjs + test-host 有车分支）。
3. 挂载：本包仍未进 desktop profile bundles——挂载动作 = profile 改动（沙箱外），与旧插件 file: 依赖改路径一并等用户窗口。**核对补记**：pnpm-lock 里两旧插件仍有 10 条旧路径条目（下次 pnpm install 自动按 package.json 新路径刷新，非阻塞）；node_modules 内旧安装副本与 tools/plugins/ 源码存在差异（车 90 行 / 机 78 行 diff 条目：安装副本 = 10-22 18:42 快照，源码此后含 README 更正与其他会话演进；源码侧另有 _reg_*/_t1_* 草稿 txt 属他人文件不触碰）。**现役跑的是旧安装副本，源码是真源**——下次 install/deploy 覆盖前别把安装副本当基准改。
4. ~~部署债：desktop profile 两个 file: 依赖指旧路径~~ —— **已清除**（2026-10-22 23:00，经用户授权）：两行改指 `tools/plugins/`，回读校验均解析到同名包（脚本 `_tmp_session/TOOLS-PLUGIN-REWORK-20261022/repoint-deps.cjs`）。**未跑 pnpm install**——安装动作归挂载事项，仍需窗口。

## 执行记录

**2026-10-22 22:14–22:50 · Lead**
- 基线复跑：7 项核心里 css-ns 崩（缺口#7）、client 缺 6、host files 崩；确认卡面缺口清单与实际代码一致后逐项修（缺口#1/#2/#3/#5 + #6/#7 新发现）。
- 编辑均以 ops_edit_file CAS 落盘（client.js/index.js/files.js/deploy.mjs/test-client.mjs/test-host.mjs 六文件，哈希链见 git diff）。
- deploy 实跑一次触发 FATAL——确认是安装目录本就不存在（0 文件），无副作用；dry-run 路径零写入复验。
- sim 红全部 EPERM 归因（两个独立探针复现），未放宽任何断言。
- README 的 --dry-run 声称自本卡起成为事实（原为不实宣传）。

**2026-10-22 22:50–23:10 · Lead（收口段）**
- 提交：v1 `2ab6c76`（13 文件，精确 pathspec）+ `5b980db`（卡面落账）；dzrobot_v2 `290e9c7`（TEAM_STATUS）。
- **如实记录（混提）**：`290e9c7` 除本人两处行改动外，同文件携带了另一会话未提交的链接改写（agent_handoff/_tmp_session → history_cache 指向，16 增 12 删的大头）。系整文件提交所致、非故意；内容无损、方向与其文档迁移一致；撤销会抹掉他人改动，故不回滚只落账。共享文件单写者（§4）后续要靠领取先看 git 脏态执行。
- **事故记录（次日补）**：23:02–23:10 另一会话把 v1 仓切到新根分支 `chunk-push`（chunk base/1/2 三笔，快照式重排全部历史）。我的两笔提交脱离主线（仍在 `github-main`：2ab6c76/5b980db），且本卡工作树被回退到初版——72 行、丢两段落账。代码修复（statusFor/classifyFailure/deploy --dry-run/字典/role 透传）**都在 HEAD 里完好**；profile file: 依赖、TEAM_STATUS、归档件、归档索引也都在。已重贴落账并补记本段。教训落盘：**并行会话会重排 git 历史，卡片类文件收口后必须 git 落库并以提交号为准，不能只靠工作树**。

## 2026-10-23 · 事故 #2（main 重建丢线）与恢复 + sim 家族复跑

**事故 #2**：v1 历史第二次被快照式重排——另一会话以 TOOLS-ORGANIZE 三笔为顶建了新 `main`（883cbe4 为尖），本卡全线（7 代码文件 + 卡面三笔落账）与 ORGANIZE 归档（归档卡 + 索引行 + 在办删旧）**全部被丢出线外**，只在 github-main/chunk-push 上幸存。恢复 = v1 `9387901`（逐文件取回，七锚点复核：statusFor/classifyFailure/dryRun/CAR_STUB/css-namespace/role 透传/字典三键均在；css-ns/client/host/s2/deploy-dry-run 复跑全绿）。教训升级：**重排已发生两次且第二次开始丢代码——任何线收口必须当场 git 落库 + 报提交号，工作树一个字节都不算数。**

**sim 测试三修** = v1 `d333925`：map-render body CRLF 归一（sim.html 是 CRLF，跨行变异体锚点假失效）；靶朝向两钉从写死 E5+F5→h 改由 field 真源两格几何现推；overlay-fix E5 按 D-012 重钉 tour、F 段 argv 改读现役 planner.engine；incremental G 段探针包 try/except（无解改 SKIP 带 planner 原文，不再崩段）。

**全家族复跑（全权限环境，原沙箱 EPERM 障碍已消）**：

| 脚本 | 结果 | 备注 |
|---|---|---|
| test-sim-map-render | **61/61 绿** | 现役图 D5+D6 竖向重钉通过 |
| test-sim-overlay-fix | **30/30 绿** | E5 读到 tour；F1 真跑 plan_route 出 68 token |
| test-sim-drone-render | **31/31 绿** | 未动，原生通过 |
| test-sim-parity | 22/25 | 余红 3.1/7.1/8.2 → 挂账① |
| test-sim-incremental | 9/13 + 2 SKIP | 余红 B3/E1/F2/F3 → 挂账①② |
| 有车时 | check-files + test-host 有车分支全断言 | 仍待真车窗口 |

**余红挂账（两笔均上游债，在本卡写域外，不粉饰、不改断言迎合）**：

- **挂账① 车端 route.tokens 数据债（`config/robot.yaml`，冻结区）**：tokens 含非法项 `N5,y`（尾缀 `,y` 疑为粘贴残留）；68 token 去重仅 57（重复 11 项）；tour 规划产物与 tokens 逐 token 不等 → parity 3.1/7.1/8.2、incremental B3 红。影响面不止测试：兜底路线合法性是启动门禁依据，非法 token 可能直接判红门禁。**建议修法（单变量）**：以现役 tour 引擎产物重生成 route.tokens 一次落盘；需冻结区登记 + owner，待规划线/Lead 领。
- **挂账② 增量续算 × D-012 冲突**：现役 CLI 已声明 --from-cell/--already-covered，运行时却「只支持 --engine tracker-safe（增量续算 G4）」，而 tracker-safe 在现役折返图 0/5 无解（D-012）→ incremental E1/F2/F3 红。**增量续算在现役配置下整体不可用**。去向二选一：规划线修 tracker-safe 对折返图的支持；或用户裁决增量功能取舍。

## §10 文档变更

- 修改：本卡（新增）、TOOLS-ORGANIZE-20261022（拆分注记+归档件）、docs/30_任务卡库/归档/00_索引.md、docs/目录索引.md/.json（gen_index 重跑）、dzrobot_v2/docs/TEAM_STATUS.md（在跑表+更新头）
- profile 侧（库外、非文档）：C:/Users/TestFox/.dsh/profiles/desktop/package.json 两行 file: 依赖路径
- docs 快照：`_archive/docs_snapshots/docs-20261022-2253.zip`（187 文件）
- 无新增长期文档于库外。
- 2026-10-23 补：v1 `9387901`（事故#2 恢复：7 代码 + 本卡 + 归档卡/索引 + 在办删旧卡）、`d333925`（sim 三修）；归档/00_索引.md 增 10-23 导航与 AG-TEAM 行、REWORK 在办行刷新；docs/目录索引.md/.json 随 gen_index 重跑。
