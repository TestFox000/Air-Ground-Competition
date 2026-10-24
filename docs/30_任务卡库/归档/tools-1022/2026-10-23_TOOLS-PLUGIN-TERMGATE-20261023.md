---
卡号: TOOLS-PLUGIN-TERMGATE-20261023
title: 终端按钮探测门禁改为"先开门后说理"——车/机离线也能打开 SSH 终端
owner: lead
status: 已归档
ns: tools-1022
创建: 2026-10-23
---

# TOOLS-PLUGIN-TERMGATE-20261023 · 终端探测门禁放行

## 1. 需求（用户原话）
「先不推送上云，现修这个插件的本地功能——让我能在插件里打开 VNC 和终端先。」

## 2. 读过的台账/卡面（铁律1）
- `dzrobot_v2/docs/决策台账.md`：检索 terminal/终端/VNC/探测 —— 无相关已裁决事项。
- `docs/30_任务卡库/在办/tools-1022/TOOLS-PLUGIN-REWORK-20261022.md`：bug#1 确立「缓存探测显示灰色、进 panel 自动重探」——本卡**只动点击时的开门闸门，不动探测缓存语义**；S7 确立终端/VNC 按节点（车/机）双通道。
- test-client.mjs:473 的旧断言「drone connect 必须在 probing 停住」是**当时设计**的记录，本卡经用户明确需求推翻该设计，断言按新语义重写（非为过测而改）。

## 3. 复用检索（铁律2）
- `dzrobot_v2/docs/复用清单.md`、`docs/能力索引.md` grep `终端|terminal|xterm|VNC|vnc`：**零命中**。
- 结论：不新增任何脚本/模块；只改 `tools/plugins/dsh-dzairground-ssh/client.js` 既有 connect() 一处 + 面板行提示 + 两个语言包条目。

## 4. 病灶（实测证据）
- 2026-10-23 19:4x 活探测：car `unreachable 连接超时(4505ms)`、drone `unreachable 网络不可达`——**两台都离线**。
- `client.js` connect() 旧逻辑：`POST /{role}/probe` 非 ready 直接 `return {ok:false}`——终端标签根本不创建。车不在时「打开 SSH 终端」必死。
- VNC 工具（dzag-vnc-car/drone）走 openToolTab，无探测闸门，标签可开；隧道连不上由 noVNC 画面如实报断开——**本卡不动 VNC**（一次一个变量，铁律6）。

## 5. 修改（单变量）
connect() 探测未过不再 return：继续 ensureConversation → 开终端 → 写入 ssh 命令（ssh 在真 PTY 里自己报超时/要密码），探测结论以 `result.warn` 随行，面板行显示「探测未通过（…），终端已照开——以终端输出为准」。

## 6. 新增轮子及理由
无。

## 7. 验收命令
```
cd tools/plugins/dsh-dzairground-ssh
node scripts/test-client.mjs     # 含重写后的 S7 无人机通道断言
node scripts/test-host.mjs
node scripts/test-vnc-guard.mjs && node scripts/test-vnc-wrapper.mjs && node scripts/test-vnc-connect.mjs
node scripts/test-package-files.mjs
```
全部 exit 0；另 curl 复测 /api/dzairground-ssh/car/status 200。

## 8. 回滚点
- 源码：`git show a7df165:tools/plugins/dsh-dzairground-ssh/client.js`（及 scripts/test-client.mjs）覆盖回。
- 部署副本：`C:/Users/TestFox/.dsh/profiles/desktop/node_modules/dsh-dzairground-ssh/client.js` 同法覆盖回，刷新页面即还原。

## 9. 影响面
仅插件浏览器半身的终端打开路径；探测本身、缓存语义、VNC/文件/仿真/浮窗均不变。无人机通道同享此修复（role 参数化）。

## 10. 文档变更
- 新增：本卡（在办 → 归档）。
- 修改：`docs/30_任务卡库/归档/00_索引.md`（归档时加行）。
- 重跑 `python docs/_工具/gen_index.py`。

## 11. 提交
按 §11 本地提交；**推送按用户 2026-10-23 指令暂停**（「先不推送上云」），仅本地 commit。

## 执行记录（2026-10-23 20:0x 收口）

**改动**（3 文件）：
- `tools/plugins/dsh-dzairground-ssh/client.js`：connect() 探测非 ready 不再 return，warn 随行；面板行新增 warn 提示；中英语言包各 +1 条（terminalOpenedAnyway）。
- `scripts/test-client.mjs`：S7 段按新语义重写（探测未过→终端照开+写入无人机 ssh 行+warn 必带）；车通道加「ready 不得带 warn」反向护栏。
- `scripts/test-vnc-connect.mjs`：vnc-connect.html 纯 CRLF（397/397）导致 import 剥离正则失效——读取后归一化 \r\n（与 test-sim-map-render 同款修法）。**此为存量红，非本卡引入**（organize 搬迁前就在，REWORK 核心五连测未覆盖此套件）。

**测试**（全绿）：
- test-client PASS（exit 0；S7 新断言：drone 探测 unreachable → ok:true + warn.state=unreachable + 写入 `ssh … duzhong@dzsmart.local`）
- test-host PASS / test-vnc-guard PASS / test-vnc-wrapper PASS / test-vnc-connect PASS / test-package-files PASS
- 注：test-host 首跑 exit 1 为端口占用 flake，单独重跑 PASS。

**追记（2026-10-23 20:5x 部署后核验）**：插件已由 d1e5c6e 迁出本仓至 `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/`，§7 验收命令应在新仓目录下执行。已核验：新仓 client.js 与本仓 17076fc 提交内容逐字节一致（CRLF 归一后 diff 0 行、sha256 均 75aa040b）；desktop profile 部署副本同哈希；新仓下 test-client.mjs / test-vnc-connect.mjs 重跑 PASS。VNC 侧确认无探测闸门（openToolTab 直开），用户「VNC+终端都要能开」的需求中 VNC 本就畅通，本卡只需修终端。

**部署**：client.js → `~/.dsh/profiles/desktop/node_modules/dsh-dzairground-ssh/client.js`，部署哈希 == 源码哈希 `75aa040b91567394`（test-package-files 绿即证）。用户刷新 GUI 页面生效。

**验收命令复跑**：`curl /api/dzairground-ssh/car/status` → 200。

**结论判定：通过。** 车/机离线时点「打开 SSH 终端」不再被闸门挡死：终端照开、ssh 命令照打、探测结论以行内提示如实告知。VNC 本次未动（一次一个变量）。
