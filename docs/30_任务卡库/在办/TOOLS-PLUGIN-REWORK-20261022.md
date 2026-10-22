---
来源: TOOLS-ORGANIZE-20261022（缺口清单交接）、本会话执行记录（2026-10-22 22:14–22:50）
取证: 实/机检
更新: 2026-10-22
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
3. 挂载：本包仍未进 desktop profile bundles——挂载动作 = profile 改动（沙箱外），与旧插件 file: 依赖改路径一并等用户窗口。
4. 部署债转挂本卡：desktop profile 两个 file: 依赖仍指旧路径（dzsmartscout/、dzrobot_v2/ 下旧目录，已空），下次 pnpm install 前必须改到 tools/plugins/，否则断链。改法已给出（一行 sed/手改均可），沙箱不可代跑。

## 执行记录

**2026-10-22 22:14–22:50 · Lead**
- 基线复跑：7 项核心里 css-ns 崩（缺口#7）、client 缺 6、host files 崩；确认卡面缺口清单与实际代码一致后逐项修（缺口#1/#2/#3/#5 + #6/#7 新发现）。
- 编辑均以 ops_edit_file CAS 落盘（client.js/index.js/files.js/deploy.mjs/test-client.mjs/test-host.mjs 六文件，哈希链见 git diff）。
- deploy 实跑一次触发 FATAL——确认是安装目录本就不存在（0 文件），无副作用；dry-run 路径零写入复验。
- sim 红全部 EPERM 归因（两个独立探针复现），未放宽任何断言。
- README 的 --dry-run 声称自本卡起成为事实（原为不实宣传）。

## §10 文档变更

- 修改：本卡（新增）、TOOLS-ORGANIZE-20261022（拆分注记+归档件）、docs/30_任务卡库/归档/00_索引.md、docs/目录索引（gen_index 重跑）
- 无新增长期文档于库外。
