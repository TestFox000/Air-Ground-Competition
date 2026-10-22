# TOOLS-ORGANIZE-20261022 — tools 区整理 + 统一空地插件重做

- owner: lead
- status: in_progress
- created: 2026-10-22
- 写域: v1/tools/**（含新增 plugins/ 与 _legacy/）；统一包整体迁移；两旧插件树迁移入 _legacy；v1/docs 索引类；dzrobot_v2/tools/wf/repo_index.py 与 dzrobot_v2/docs/索引标注.yaml（内容改、位置不动）
- 非目标: 不碰 desktop profile 部署（用户 2026-10-22 裁「先不重新挂」）；不碰 deploy_to_car.py 与 wf 门禁位置；不碰另一会话在途文件（docs/_work、docs/_工具/cleanup_*、docs/history_cache/01_旧文档归档/snapshots）

## 台账 / 复用检索（5 必填）

- 台账检索: docs/20_资料索引库/历史/02_并入与改名_20261018.md——统一包 S0~S6 全史，现役状态「已建成、未挂载」；无翻案，本卡只推进「前置已备、未挂载」态
- 复用检索: tools/ 现存 aim_autotune/、pack_handoff.py 保持原样；统一包 12 条自检脚本作为重做验收基座复用；无新造轮子

## 基线（2026-10-22 实测）

| 检查 | 结果 |
|---|---|
| check-css-namespace | PASS（与车侧无命名冲突） |
| test-vnc-guard / wrapper / connect | PASS |
| test-package-files | PASS（SKIP：desktop 未装本包） |
| test-host | 全部逻辑断言过；/files 段被沙箱 spawn EPERM 挡住（需真车） |
| test-client | 渲染/座位断言过；connect 流程被 spawn EPERM 挡；2 处真缺口见下 |
| deploy.mjs --dry-run | exit 1（包未安装）+ 代码无 dry-run 分支（缺口#1） |

## 缺口清单（证据化）

1. deploy.mjs 无 --dry-run 分支：README 声称「只看不改」，代码实际执行同步
2. bridgeConnectorConfig 默认角色语义：测试期望无 role → 遗留 /vnc，代码返回 /car/vnc（test-client.mjs:640）
3. 字典键 connCarBridge/connCarBridgeHint/connCarBridgeDone 缺 zh+en（test-client.mjs:670）
4. README「安装位置」过时：声称已挂载、旧目录名 dsh-plugin-dzairground-ssh
5. /files 在 SSH 失败时返回 400，对照 probe/metrics 的 honest-offline 约定需评审（沙箱内无法实测）

## 验收命令

- node scripts/{check-css-namespace,test-host,test-client,test-vnc-guard,test-vnc-wrapper,test-vnc-connect,test-package-files}.mjs 全 exit 0（除需真车的 /files 段，记录在卡）
- python dzrobot_v2/tools/wf/check.py（确认门禁绿）
- docs/目录索引.md + .json 经 gen_index.py 重跑
- git commit（v1 仓）

## 回滚点

- 移动前哈希清单: _tmp_session/tools-organize-20261022/manifest.json
- 两旧插件树迁移后进入 v1 git 跟踪（原为 gitignore），可从 tools/_legacy/ 原样移回
- 统一包同上；v1 仓未提交前全部可逆

## 执行记录

（本节随执行追加）
