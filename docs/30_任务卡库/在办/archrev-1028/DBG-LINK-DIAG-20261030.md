---
卡号: DBG-LINK-DIAG-20261030
ns: archrev-1028
agent: /root
树节点: AR-4
owner: /root
reviewer: /root/test_research
kind: requirements
status: in_progress
创建: 2026-10-30
更新: 2026-10-30
摘要: P0-3 落地——把 ARCH-RESEARCH-20261028 挖出的 6 个 dzbrain 结构性隐患与 ARCH-RESEARCH §「建议先做的可验收切片」变成一份机器可判的《链路诊断手册》。零代码改动，纯文档。门禁已跑：本卡引入红归零。
---

# DBG-LINK-DIAG-20261030 · dzbrain 链路诊断手册（P0-3）

## 目标与边界
用户本轮指令：**只能离线修补，先做能完成的地方**。据此选 P0-3（零风险、当天可收口、是 P0-1/P1-1 的自然延伸）。产出《链路诊断手册》，把「车不动 / 车乱跑 / 停不下来 / 打不出去 / 信息不通」五类现场症状，每个都连到一条**可复跑命令 + 预期输出 + 判据**。

**不动**：`src/dzbrain/dzbrain/**`、`config/**`、`tools/deploy_to_car.py`（冻结区）。**不连车 duzhong.local、不连机 dzsmart.local**。**不新建 .py**（避免 check.py 未登记判红）。

## 读过的台账条目
- D-008 冻结区、D-019 唯一部署入口、D-025 本地通过与发布分开、D-044 提交推送由用户决定（`dzrobot_v2/docs/决策台账.md` 薄索引）。
- 已知挂账红 K1~K9 不重开（`docs/20_资料索引库/运维与排障/08_回归守护_known_reds.md`）。
- 标定真源：ticksPerMeter=3280.0（冻结）、ticksPer2PI=2670.0（待仲裁）、yaw_trim=-0.6（`dzrobot_v2/AGENTS.md` §6）。

## 复用检索结果
- 已查 `docs/README.md`（唯一入口）、`07_发车门禁口径_G1-G4.md`、`09_明早全流程SOP_20261029.md`、`15_dzbrain零基础源码导读.md`、`13_模块协调与链路展开.md`、`10_日志阅读课_新手版.md`。
- 已读 `archrev-1028/ARCH-RESEARCH-20261028.md`（AR-1）与其「建议先做的可验收切片」。
- 已读 `dzrobot_v2/AGENTS.md`（G1~G4 指针、标定真源、全量回归命令 `python _run_full_regression.py`）。
- **不新建第二份架构真源**；手册与 G1-G4 / SOP / 源码导读互补，不重复。

## 新增轮子及理由
唯一新文件：`docs/20_资料索引库/运维与排障/11_dzbrain链路诊断手册.md`。**理由**：G1-G4 讲「发车前四门怎么验」，SOP 讲「这一趟按什么顺序做」，二者都没有「断了之后从现象往回定位」的故障树；这正是现场最痛的一层（dzrobot_v2/AGENTS.md §6「门禁 GREEN ≠ 回归绿 ≠ 实车通过」）。手册不新增代码、不改配置、不动任何现有判据。

## 写域
- 本 ns 卡（新建）+ `archrev-1028/00_树.md`（追加 AR-4 行）。
- `docs/20_资料索引库/运维与排障/11_dzbrain链路诊断手册.md`（新建）。
- 仅 Lead 追加 ns 登记、生成索引、状态板注明本轮。其他代码与他人卡只读。

## 验收清单
- [x] 手册每个【离线可跑】命令**本机实跑过**，输出为真，非按印象写。
- [x] 手册每个【现场必跑】命令明确标〔码〕（按代码打印格式推，未实跑）。
- [x] 5 类现场症状各自连到候选原因 + 命令 + 预期 + 判据。
- [x] 遥测字段判读取自 `telemetry.py:96-163` 的 `build_telemetry` 真源。
- [x] 明确「哪些不能离线判 / 必须现场」的边界。
- [x] 冻结区零改动；新增文件仅 .md，无新 .py。
- [x] 独立 reviewer 复跑文档门禁（gen_index / check_doc_policy / check_anchors / check_numbers + wf/check / check_links），**本卡引入红归零**；存量他域红如实列出不挡收口。

## 收口门禁实跑结果（2026-10-30）

| 门禁 | 结果 | 说明 |
|---|---|---|
| `gen_index.py` | exit 0 | 全量重建 |
| `check_doc_policy.py` | 剩 1 条 R1 | `clear/ontology/verification-loop.md`（ClearAI 系统写域，不归本卡）；本卡 R3/R6/R10 全归零 |
| `check_anchors.py` | 13 条新增红 | **全为他人文档**（`agc-1024`/`agc-1026`/`lead-1023`/`traj-1025`/`tools-1022` 引用 `_tmp_session/` 或已清理的 [mvlog.csv](../../../../artifacts/aim-mv-20261024/mvlog.csv)/[mvrec.py](../../../../artifacts/aim-mv-20261024/mvrec.py) 等）；本卡与 11 号手册、16 号契约表**零红** |
| `check_numbers.py` | exit 0 | 无新增漂移/未声明异文；2 条存量已声明（wheelbase、fire_hold_frames） |
| `wf/check.py` | `结果: GREEN` | exit 0 |
| `wf/check_links.py` | `broken: 0` | 279 文件 |

## 落点指针（§9 收口三选一）
- **坑库症状索引**：`docs/20_资料索引库/运维与排障/01_症状索引_52条.md` —— 本手册是它「症状→机制→位置」之上再加「命令→预期→判据」的一层；读者拿到具体症状先查 01，要下钻到可执行命令再查 11。
- 反向指针已在 `运维与排障/00_索引.md` §1（11 号行）与 `20_资料索引库/README.md` 分册表（运维与排障 12 篇、「断了之后从现象往回定位看 11」）登记。

**验收命令**（固定顺序，在工作区根跑前四条）：
```powershell
cd D:\User\Desktop\比赛存档\v1
python docs\_工具\gen_index.py --only 20_资料索引库/运维与排障/
python docs\_工具\check_doc_policy.py
python docs\_工具\check_anchors.py
python docs\_工具\check_numbers.py
cd dzrobot_v2
python tools\wf\check.py
python tools\wf\check_links.py
```
预期：四条文档守卫 exit 0 / 新增红 0；wf/check 结论 GREEN；check_links broken 0。既有他域红不改基线、不扩大写域。

## 回滚点
只撤销本卡新增文件、`00_树.md` 追加的 AR-4 行、生成索引中本 ns 新增行与状态板段落。不覆盖他人文件，不 reset/clean。生产代码本轮零变更。

## 执行记录与结论
开场已读唯一入口与 archrev-1028 现有树；因 P0-3 是 AR-1 的自然延伸（而非新议题），在 archrev-1028 追加 AR-4 不另开 ns。以下命令均已本机实跑（2026-10-30）：

| 命令 | 实测结果 | 标 |
|---|---|---|
| `python tools/wf/check.py` | `结果: GREEN`，exit 0（13 条失效登记 WARN 不挡收口） | 〔实〕 |
| `python src/dzbrain/tests/test_contract.py` | `56/56 全部通过` | 〔实〕 |
| `python src/dzbrain/tests/test_tracker.py` | `139/139 全部通过` | 〔实〕 |
| `python src/dzbrain/tests/test_localization.py` | `175/175 全部通过` | 〔实〕 |
| `python tools/config_selfcheck.py`（不带 --map） | 68 项，`结论：error 0 / warn 0 ✅ 通过`，exit 0 | 〔实〕 |
| `python tools/car_reconcile.py --offline` | 输出 111 项「待同步，缺首次 --record」；**exit 0 不代表失败**（缺 record 时的信息输出） | 〔实〕 |
| `python -m dzbrain.tools.sim_cli --pretty --max-steps 3000` | exit 0；分段表（段/起/终/长/状态/舵角/速度/时长）+ `traj.csv` / `traj.svg` / `by_segment.csv` 三产物 | 〔实〕 |
| `python tools/route_check.py --help` | 用法打印正常（`--config/--route/--map/--baseline/--simulate/--from-planner` 等） | 〔实〕 |

## 文档变更
新建本卡 + `archrev-1028/00_树.md` 追加 AR-4 + 新建 `docs/20_资料索引库/运维与排障/11_dzbrain链路诊断手册.md` + 自动生成索引与状态板段落。生产代码零变更。

## 经验总结
**现象**：用户「只能离线修补、先做能完成的地方」——把上一轮的大方案拆到最小可收口切片时，P0-3 纯文档最快落地。
**处置**：所有【离线可跑】命令必须本机实跑出真实输出再写进文档，标〔实〕；现场命令标〔码〕并注明未实跑。这避免了「手册里引用了跑不通的命令」这类文档病。
**落点**：坑库症状索引（`docs/20_资料索引库/运维与排障/01_症状索引_52条.md`）——补充"离线能验的判据才是文档能用的判据"。
## 2026-10-30 文档治理补注

仅把在办卡元数据改回合法 in_progress，保留原业务完成陈述，未代其业务归档。Clear 原位已获授权无损归档，旧R1记录仍是当时事实；本次唯一迁移回指：
[clear/ontology/verification-loop.md](../../../history_cache/01_旧文档归档/clear/ontology/verification-loop.md)
