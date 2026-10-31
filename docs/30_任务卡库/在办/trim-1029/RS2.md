---
卡号: RS2
ns: trim-1029
agent: trim-car
树节点: TRIM-19
owner: trim-car
kind: review
status: in_progress
更新: 2026-10-30 13:05
摘要: 独立评审 t15/S2 机端三块注释扩展（实现者 trim-doc）：源码 before/after 与 R0 S2 精确 SHA 相等、完整 AST 含 docstring 不变、四门原始退出 0/1/1/1 原样保留，未闭合实质缺陷 0 ⇒ verdict=pass。
---

# RS2 · 扩展机端独立评审（round 1，审 t15/S2）

## 0 身份与独立性

| 项 | 值 |
|---|---|
| 被审 task | **`t15`** · S2 · 实现者 **trim-doc**（我核过正式字段 `reviewedTaskId=t15`，与 captain 说法一致） |
| 本评审 | `t19` · attempt `4df9fb93-7574-4caa-9a74-ba54506de28d` · reviewer **trim-car** |
| 独立性 | S1/t7 的原实现是 **trim-scout**；S2/t15 的三块源码是 **trim-doc 在新授权下实际写**的。我**未参与** S2 实现 ⇒ 评审成立，非自评。我自己写的是车端 C2，与本卡无关。 |
| 写域 | 仅 `docs/30_任务卡库/在办/trim-1029/RS2.md` + `_tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/` |
| 未写 | 源码、`docs/20_资料索引库/**`、他人卡、共享索引、守卫/baseline。**未跑 `gen_index`**。 |

> **过程目录纪律**：`_tmp_session/TRIM-EXEC-20261029/RS2/` 里已有 **S1 round2 阶段的误阶段日志**（一批 `rs2_*` 探针输出与 `g_*` 守卫日志，另有若干 unittest/verify 原始日志，10-30 09:5x–10:2x；具体文件名见该目录本身，本文不逐一列举）。本轮**一律不碰**，所有新证据只写 `_tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/`，旧数据原样保留。

## 1 判定

**verdict = pass**（2 条 nit 只登记，不阻断）。

| 合同 acceptance | 结论 | 我的实测 |
|---|---|---|
| ① sender 真实 before/after == R0 S2 精确 SHA，仅三授权注释块；完整 AST 含 docstring、非注释 token/常量、同 prog/argv 原始 CLI 字节不变；未动活跃逻辑或扩白名单 | **通过** | 前像 `before_h1_raw.bin` 21293 B == R0 `S2.before`(=`S1.after`)；现场 21076 B / CRLF 442 / lone-LF 0 / sha `fde0886d…` == R0 `S2.after`；**B0 副本 + S1-DOC + 三块 S2 逐块唯一替换 = 现场逐字节重建成功**；三块 `new_text` 逐行**纯 `#` 注释**；完整 AST（**含全部 docstring**）sha `5220ca32…` 前后**同值**；非注释 token **3069 = 3069 全等**；5 常量全等；同 argv[0] 下 `--help` 原始 **757 B / 18 CRLF / sha `3cba51a7…`** 两侧逐字节相同 |
| ② 按最新 t15 非回归门核四入口原始日志/退出/计数/失败身份/skip；原 raw 1 保 FAIL；仅同输入无新差异可 scope pass；未解析/ImportError 未收集不算通过；不改原测试/fixture/旧红 | **通过** | 四门原始退出 **link 0 / planner 1 / perception 1 / simair 1**（我独立复跑复得同值）；link 203 ran / 202 pass / skip 1，前后同；planner **17 条失败断言名前后完全相同**，pass 27/44；perception 通过 99/失败 1、`FIELD_LAYOUT_VERSION` ImportError 仍**计未收集不计通过**；simair 自检 49/51 FAIL 2（t28 与基线完成指标）；`new_fail_ids_all=[]`；未改任何测试/夹具/期望 |
| ③ S2 前像与回滚点真实、S1 历史不覆盖、保护资产与官方 baseline 不变；采样角色/时间范围自洽，不用 mtime 代历史完整性 | **通过** | 前像逐字节 21293 B（== R0 S2.before）、回滚配方见 §5；S1 历史段在 **B0/S1 前像**里完整在场（`S2.before` 已是 S1 之后状态，属正常演进，**未被二次改写**）；保护资产 5/5 == B0；官方 baseline **9/9 == B0**；采样时序 mtime 实测 base(11:34) < apply(11:45) < post(11:46) **有序**，但**我只把它当角色自洽的证据，不当历史完整性证明** |
| ④ 只把未闭合实质契约问题判 needs_revision；风格 nit、含尾空行窗口不单独阻断 | **遵守** | 本轮**零** needs_revision 级问题；2 条 nit 见 §6 |

## 2 五必填

### 2.1 台账条目

| 编号 | 约束 | 本评审遵守 |
|---|---|---|
| D-008 | 冻结区四路径 | 目标文件在 `dzsmartscout/tools/link`，不在四路径内；我**零写入**该树 |
| D-019/D-007 | 部署唯一入口、人工替换 | 未部署、未连车 |
| D-044 | commit/push 等用户指令 | 未 commit / 未 push |
| 树「当前唯一写域」 | 成员不跑 gen_index、不写共享索引 | 未跑、未写；索引只读 |
| AGENTS §1.7 | 验收机器可判、不许自评 | 全部判据由我自己复跑/复算（§3、§4） |

### 2.2 复用（不重造基线）

| 需要 | 复用对象 |
|---|---|
| 白名单唯一真源 | `_tmp_session/TRIM-EXEC-20261029/R0/scope_manifest.json`（S2 的 before/after 与三块 old/new） |
| 前像 | `_tmp_session/TRIM-EXEC-20261029/S2/before_h1_raw.bin`（+ `before_h1.json`） |
| 非回归证据 | `_tmp_session/TRIM-EXEC-20261029/S2/` 的 `base_*` / `post_*` 原始日志与 `verify_nonregression_report.json` |
| 官方 baseline | `docs/_工具/` 下 9 文件，对 B0 副本比对 |
| 不采用 | t15 卡自报的计数与结论；一律自己重算 |

### 2.3 新增轮子

仅 `_tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/rs2p_*.py`（一次性只读探针，过程区，不登记、不进 `tools/`）。**无长期工具新增。**

### 2.4 验收命令（我自己跑的）

```bash
# 源码与哈希
python -B -X utf8 _tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/rs2p_src.py
python -B -X utf8 _tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/rs2p_ast.py        # 完整 AST 含 docstring
python -B -X utf8 _tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/rs2p_tokens_cli.py  # token/常量/CLI 原始字节
# 四门（用 B0 09_run_scout.py 的原入口，非自造路径）
python -B -X utf8 _tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/rs2p_run_gates2.py
python -B -X utf8 _tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/rs2p_planner.py
# 保护资产 / baseline / 门禁
python -B -X utf8 _tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/rs2p_assets.py
python -B -X utf8 _tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/rs2p_baselines.py
python -B -X utf8 docs/_工具/check_doc_policy.py; python -B -X utf8 docs/_工具/check_anchors.py
```

### 2.5 回滚点核对

| 项 | 实测 |
|---|---|
| S2 前像 | `_tmp_session/TRIM-EXEC-20261029/S2/before_h1_raw.bin` **21293 B / CRLF 444 / sha `fffffde7…`** == R0 `S2.before` |
| 可逆性 | 反向把三块 `new→old` 还原，逐字节回到前像（sha 相等） |
| 第二来源 | B0 副本（R0.before 原始 22089 B）可作更早一档回滚 |
| 影响面 | 1 文件 / 3 块 / −217 B（442 CRLF） |

## 3 源码：字节级重建（最强判据）

| 环节 | 实测 |
|---|---|
| 前像 | 21293 B / CRLF 444 / lone-LF 0 / sha `fffffde76dc96da5…` == R0 `S2.before` |
| 三块唯一性（在**前像**上） | S2-BLOCKHDR **1** 次、S2-TARGET **1** 次、S2-BRIDGE **1** 次 |
| 前像 + 三块（按 manifest 顺序）重建 | **与现场逐字节相同**，sha `fde0886d5da3961bc8e0d7ba07aeafa48efe0653cc37f1192479374df2eaf354` == R0 `S2.after` |
| 现场 | 21076 B / CRLF **442** / lone-LF **0** / 同 sha |
| 三块 `new_text` 性质 | 逐行**全部纯 `#` 注释**（无活跃语句） |
| 字节账 | BLOCKHDR **+9**、TARGET **−128**、BRIDGE **−98** ⇒ 合计 **−217 B**，CRLF 444→442（三处都含换行，净删 2 行） |

## 4 语义等价（完整 AST / token / 常量 / CLI）

| 核验面 | 我实测 |
|---|---|
| **完整 AST 含全部 docstring**（合同要求，非 S1 的剔 docstring 例外） | `ast.dump` sha **`5220ca320b065398…` 前后同值** ⇒ **完全相同** |
| 非注释 token 流 | **3069 = 3069，完全相同** |
| 模块常量 | `BANNER` / `BLOCK_SEP` / `TOPIC_ROUTE` / `TOPIC_TO_CAR` / `TOPIC_FROM_CAR` **5/5 相等** |
| CLI（同 prog 名、同 argv） | 两侧各在临时目录内以 `dz_send_waypoint.py` 为名运行：exit **0/0**、stderr **0/0 B**、stdout **757 B / 18 CRLF / sha `3cba51a7…`**、**逐字节相同** |
| 与 t15 存的 `help_before.bin` / `help_after.bin` | 我复跑结果与两者**逐字节相同**（757 B / 同 sha） |
| 白名单是否扩大 | **否**：只动 S2 三块；`perception`/`planner`/`sim_airground` 本轮 R0 块数=0，未碰；未碰任何 C++ |

## 5 非回归门（我自己用 B0 原入口复跑）

**入口来源**：`_tmp_session/TRIM-EXEC-20261029/B0/scripts/09_run_scout.py` 第 72–79 行的原命令（不是我自造的路径）。我第一次误用 `tools/planner/run_tests.py`（不存在，exit 2），改用 `-m unittest discover -s tools/planner/tests` 后正常。

| 套 | 原始退出（我复跑） | 计数（我复算） | 与 t15 post 对照 |
|---|---|---|---|
| link | **0** | ran **203** / pass **202** / skip **1** / fail 0 | **完全相同** |
| planner | **1** | ran **44** / pass **27** / 失败断言名 **17** | 断言名集合**完全相同**；pass 27 相同 |
| perception | **1** | 通过 **99** / 失败 **1**；`FIELD_LAYOUT_VERSION` **ImportError 计未收集、不算通过** | 相同 |
| simair | **1** | 自检 **49/51**；FAIL 2（`t28 基线完成指标（现役`、`t28`） | 相同 |

**`new_fail_ids_all = []`（我独立确认）**：四套失败断言名集合与 t15 的 base/post 完全一致，**无新增失败身份、无 skip 增加、无覆盖减少**。

**scope 与 global 分明**：`scope_verdict=pass`（同输入无新差异）/ `global_verdict=fail`（planner/perception/simair 的 raw 1 **原样保留**，**未把旧红改成 exit 0**）。

### 5.1 我核到的一处**采样细节差异**，并已定位为环境非确定性（不阻断）

| 运行 | planner `FAILED (...)` 拆分 |
|---|---|
| t15 base（11:34） | `failures=15, errors=2` |
| t15 post（11:46） | `failures=15, errors=2` |
| 我复跑 3 次 | `failures=16, errors=1`（三次稳定） |
| **B0 隔离副本复跑 2 次**（该树 sender 是 **R0.before 原始 22089 B，未做 S1/S2**） | `failures=16, errors=1`（两次稳定） |

**关键证据**：B0 副本里**根本没有 S1/S2 改动**，却同样得到 `16/1` ⇒ **该拆分差异与我审的 t15/S2 无关**，是 `test_emit_json_and_exit1_on_repress` 本身的环境敏感（一侧抛 `JSONDecodeError` 记 ERROR，一侧断言 `'revisit' != 'repress'` 记 FAIL）。

**断言名集合 4 次运行完全相同（17 条）**，只是 FAIL/ERROR 归类翻转 ⇒ 按合同「仅同输入无新差异可 scope pass」，**不构成新差异，不阻断**。登记为 nit（§6 N-1）。

## 6 nit（仅登记，不阻断）

| ID | 内容 | 为何不阻断 |
|---|---|---|
| N-1 | planner 的 `failures/errors` 拆分在本机可复现为 `16/1`，而 t15 记录为 `15/2`；我用 **B0 未改动副本**复现出 `16/1`，证明与环境敏感有关、与 S2 无关 | 断言名集合 17/17 完全相同，失败身份无新增；raw exit 同为 1 |
| N-2 | t15 卡正文写「逐块 delta +9 / −128 / −98 = −217 B」，我复算一致；但卡里未写明「这三处都含行尾换行、净删 2 行」，只看 B 数会以为 CRLF 该减 0 | 数值正确，只是说明粒度；`apply_record.json` 与我的实测均一致 |

## 7 保护资产 / baseline / 门禁

| 项 | 实测 |
|---|---|
| 保护资产 5 项 | `robot.yaml` / `map.yaml` / `deploy_to_car.py` / `dzrknn_dataprocess.cpp` / `dzrknn_dataprocess.h` —— **5/5 == B0 副本** |
| 官方 baseline **9 文件**（3 守卫 + `doc_policy.json` + 5 个 `*baseline*.json`） | **9/9 == B0 副本**（字节未动） |
| `check_doc_policy.py` | 真退出 **1**，红线 **3** 条全在他人写域；**trim-1029 归因 0** |
| `check_anchors.py` | 真退出 **1**，新增红 **15 occ / 14 唯一 key**；`--json` 逐条归属，**`trim-1029` 命中 0** |
| `check_numbers.py` | 真退出 **0** |
| `tools/wf/check.py` | 真退出 **0**，`结果: GREEN` |
| `tools/wf/check_links.py` | 真退出 **0**，`309 files / broken 0` |

> `GREEN ≠ 回归绿 ≠ 实车通过`；全局 policy/anchors 仍 exit 1 属外域既有红，**原样保留不重标为绿**。

## 8 变更归属（不采信任何 changedPaths）

按 B0 `input_manifest.json` 的 **822 个 in-copy 输入**逐字节重算：**810 同 / 12 变 / 0 缺失**。12 个逐一带主：

| 文件 | 归属 |
|---|---|
| `dzsmartscout/tools/link/dz_send_waypoint.py`（22089→21076） | **S1(t7) + S2(t15)** ← 被判对象 |
| `rkpt.hpp`(906) / `common.h`(719) / `rknnPool.hpp`(3048) / `dzjudgment.cpp`(206) | C1(t6) + C2(t14，我实施) |
| `技术全景_C/D/E` | D1(t8) + D2(t16) |
| 两索引 + `TEAM_STATUS` + `决策台账` | Lead 治理/索引发布 |

⇒ **t15 只落地 1 个源码文件**，无越界；S2 三块之外的 sender 内容未动。

## 9 证据索引

| 证据 | 路径 |
|---|---|
| 源码/哈希/块唯一性 | `_tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/rs2p_src.log`、`_tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/rs2p_blocks.log` |
| 完整 AST 含 docstring | `_tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/rs2p_ast.log` |
| token/常量/CLI 原始字节 | `_tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/rs2p_tokens_cli.log` |
| 前像/回滚 | `_tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/rs2p_preimage.log` |
| 四门复跑（B0 原入口） | `_tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/rs2p_run_gates2.log`、`_tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/rs2p_planner.log` |
| 四门计数/失败身份对比 | `_tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/rs2p_cmp_gates.log`、`_tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/rs2p_final_gate.log`、`_tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/rs2p_splitall.log` |
| 环境敏感定位（B0 副本复现 16/1） | `_tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/rs2p_b0planner.log` |
| 保护资产 / baseline | `_tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/rs2p_assets.log`、`_tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/rs2p_baselines.log` |
| 变更归属对账 | `_tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/rs2p_reconcile.log` |
| 门禁原始日志 | `_tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/check_doc_policy.log`、`_tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/check_anchors.log`、`_tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/check_anchors_json.log`、`_tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/check_numbers.log`、`_tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/wf_check.log`、`_tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/wf_links.log` |
| 四门我的原始日志 | `_tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/mine2_link.log`、`_tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/mine2_perception.log`、`_tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/mine2_simair.log`、`_tmp_session/TRIM-EXEC-20261029/RS2/s2_phase/mine3_planner.log` |

## 10 未做 / 边界

- **未跑 59 套全量、未重建基线**（合同要求「按 t15 四门核」，不要求另建基线）。
- 未改任何源码/测试/期望/夹具/skip/baseline；未 `gen_index`。
- 未验证真实 ROS 下 `send()/listen()`；未编译 RKNN；未实车/连车/部署/commit/push。
- `_tmp_session/TRIM-EXEC-20261029/RS2/` 里 S1 round2 的旧日志**未读入结论、未改动、未删除**；本轮新证据全在 `s2_phase/`。
