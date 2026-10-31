---
卡号: P2-CAR
ns: trim-1029
agent: trim-car
树节点: TRIM-P2CAR
owner: trim-car
reviewer: trim-verify
kind: work
status: in_progress
更新: 2026-10-30 17:05
摘要: 第一阶段行政门未闭时的只读预研，不开启生产改造、不宣称第一阶段完成。
---

# P2-CAR · 车端全域只读覆盖预研

## 1 目标与边界
dzrobot_v2 内自研生产源码、启动入口、构建和测试工具；文档按需查依据，不修改。第三方/派生/生成物逐项标明来源，不能因在 vendor/history 下就自动判可删。

只写本卡及本任务独占过程区，不改生产源码、测试、期望、配置、基线、其他卡或索引；不部署、不连接设备、不提交推送。原 G4/I0 全局零红硬门仍未满足。

## 2 五必填
- 读过的台账：以 [决策台账](../../../../dzrobot_v2/docs/决策台账.md) 当前薄索引与 D-060 为先，冲突旧稿不作现行授权；实际读取项由 owner 记录。
- 复用检索：先查 [资料入口](../../../README.md) 与既有 A2/R0/B0 清单；它们不是全部源码覆盖的证据。
- 新增轮子及理由：不新建长期工具；临时最小清单辅助如确需，须先说明既有为何不够，且仅写自己的过程区。
- 验收命令：两仓 git status --short 记录前后状态；源文件哈希清单前后相等及覆盖计数用结构化结果独立核对；不能以无命中/零检查冒充完成。
- 回滚点：本任务只读源文件，无生产回滚动作；既有脏改动不覆盖。新增过程证据独立保留，不 reset/clean/stash。

## 3 验收清单
- [ ] 源文件清单完整性、扫描预算/截断/未读缺口明确。
- [ ] 每条有真实路径、来源分类、证据等级与保留/精简/等价重构/缺证判断。
- [ ] 文件名/AST inventory 不冒充语义读完，grep 零命中不证明死代码。
- [ ] 源代码/配置/测试/基线无写入，前后哈希与原有脏状态有机器证据。
- [ ] 全部候选仍须后续 requirements → implementation → verification → review → integration，当前不执行。

## 4 文档变更
本卡是预研任务状态与真实产物入口；过程文件存在后再补完整真实链接，不预写尚不存在的文件名。

## 5 经验总结
现象：第一阶段实现技术通过但外域行政门未闭。
处置：保持边界不伪绿，利用空闲资源做不影响实现的只读覆盖准备，不将预研当正式完工。

## 6 本次执行记录（t50 / attempt 8fddb67c-567d-4a25-8adf-d9ef1f8e11ad）

> 本节为追加记录，**不改动上面 §1–§5 的原有正文**。

### 6.1 五必填之实际执行

| 项 | 实际 |
|---|---|
| 读过的台账 | 本卡 §1 边界、`AGENTS.md` §3 冻结区、`docs/README.md`（7466 B 首屏分流表）；`D-060` 仅作边界参考，本卡**不涉及**该机制改动 |
| 复用检索 | `docs/README.md`、`00_树.md`（本卡节点 `TRIM-P2CAR`）；A2/R0/B0 **仅按需交叉参考**，B0 为**有限采样**不冒充全部覆盖 |
| 新增轮子 | **无长期工具**。全部为 `_tmp_session/TRIM-EXEC-20261029/AUDIT-CAR/` 下一次性只读探针（11 份 JSON），不建 baseline、不建第二套能力 |
| 验收命令 | 见 §6.4，全部 exit **0** |
| 回滚点 | 只读，无生产回滚动作；既有脏改动不覆盖；不 `reset`/`clean`/`stash` |

### 6.2 覆盖计数（读与未读分开，不承诺一次读完）

| 指标 | 数值 |
|---|---|
| `dzrobot_v2` 全枚举（排除 `.git`/`__pycache__`/缓存） | **5999** 文件，逐 path+bytes+sha256 |
| 本域（自研源码/入口/构建/测试工具） | **221** |
| 本域 Python AST 解析 | **143 / 143 成功，0 语法错** |
| 本域 C/C++ 文本清单 | **31**（行数/注释行/预处理行/`__LINE__`/`has_main`） |
| 入口形态识别（静态） | **66**（**按文件去重后的 union**；形态计数 `__main__` 55 / `def main()` 47 / ROS2 launch 8 / setup entry_points 4 / `python -m` 1 **可重叠、不可相加**，合计 115 ≠ 66） |
| **语义阅读（逐行）** | **仅 6 个**（dzbrain core，全部 `self_dev_source`；**无一个 SDK**），标 `semantic_read_partial` |
| **未做语义阅读** | **215 / 221**（inventory 或 AST 级；**不含 SDK**，SDK 已移出本域） |
| 知识落点（00+20 库 109 篇静态文本命中） | 有落点 **183** / 零落点 **38** |

### 6.3 分类与决策（不含「可精简」结论）

| 分类 | 数量 | 处置 |
|---|---|---|
| `self_dev_source` | **174** | 保留；待后续语义审读 |
| `self_dev_tool` | 36 | 保留（门禁/回归依赖） |
| `self_dev_frozen` | 8 | 冻结区，只读，不可改不可删 |
| `self_dev_entry` | 3 | 保留（入口断链路） |
| `third_party_sdk` | **10** | 内容证据：文件头含 Rockchip/Copyright/All rights reserved |
| `third_party_model` | 2 | 模型二进制，不清理 |
| `config_frozen` | 7 | 冻结区，只读 |
| `doc_live` | 9 | 活文档，只读 |
| `generated_or_data` | 778 | 运行产物/数据集，排除出本域 |
| `historical_or_process` | 4965 | 归档/备份/过程区，排除出本域 |

**全部 221 条决策均为「保留/待核」，`needs_further_verification: true`；本卡不给任何「可精简」结论。**

### 6.4 真实退出（全部 0）

```bash
python -B -X utf8 _tmp_session/TRIM-EXEC-20261029/AUDIT-CAR/p2_inventory.py   # 5999 枚举+hash+分类
python -B -X utf8 _tmp_session/TRIM-EXEC-20261029/AUDIT-CAR/p2_ast.py         # 143 py AST
python -B -X utf8 _tmp_session/TRIM-EXEC-20261029/AUDIT-CAR/p2_cpp.py         # 31 C/C++ 文本
python -B -X utf8 _tmp_session/TRIM-EXEC-20261029/AUDIT-CAR/p2_proof.py       # 只读证明
```

### 6.5 只读证明（机器证据）

| 项 | 实测 |
|---|---|
| 两仓 `git status --short` 前后 | `.` **57 行 identical** / `dzrobot_v2` **8 行 identical**（**注意：status 相同≠内容全不变**，见下「以逐 hash 为主证据」） |
| 5999 文件 hash 前后 | **5998 未变 / 1 变 / 0 缺失** |
| 唯一变化 | `dzrobot_v2/docs/TEAM_STATUS.md`（83202 B，mtime 16:52:29，晚于我 16:50 采集）⇒ **Lead 单写状态板，非本任务写入**，已单列 `concurrent_external_write`，**不笼统宣称「零写入」** |

### 6.6 我的三处自我更正（如实记案）

| # | 起初怎么判 | 实测后怎么改 |
|---|---|---|
| 1 | `rga/` 与 `rknn_api/` 按**目录名**归入自研（self_dev_source 174） | **按文件内容**核出 10 个文件头含 Rockchip/Copyright ⇒ 重分类 `third_party_sdk`（自研仍为 **174**，SDK 已移出本域） |
| 2 | 只扫 `dzrobot_v2/docs`（6 篇）得「零命中 159」 | 扩到 `v1/docs` 00+20 库（**109 篇**）后 **183 有 / 38 零** —— 原数是**扫描面太窄**的假象 |
| 3 | 首版只读证明写「零写入」 | 复核发现确有 **1 个文件变了** ⇒ 查明为 Lead 并发写，改为**单列登记** |

### 6.7 明确缺口（不宣称已覆盖）

- **语义阅读 6 / 221（2.7%）**；其余为 inventory/AST 级。**本卡不宣称「已覆盖」全域。**
- 逐包渐进计划见 `p2_groups.json`；`src/dzbrain` 最大（105 文件），仅读 6 个核心。
- 域外 4965 `historical_or_process` 与 778 `generated_or_data` **按设计排除**，未逐文件语义核。
- 冻结区（`config/**`、`src/dzbrain/dzbrain/**`、`src/dzrknn_dataprocess/**`、`tools/deploy_to_car.py`）**只读**，未给精简结论。

### 6.8 过程产物（`_tmp_session/TRIM-EXEC-20261029/AUDIT-CAR/`）

`p2_inventory.json`（2.55 MB 全枚举）· `p2_structure.json` · `p2_groups.json` · `p2_ast_inventory.json` · `p2_cpp_inventory.json` · `p2_entries.json` · `p2_knowledge.json` · `p2_semantic_sample.json` · `p2_decisions.json` · `p2_readonly_proof.json` · `p2_baseline.json` · `p2_artifacts_manifest.json`（11 份 JSON，含各自 bytes/sha256）

### 6.9 未做 / 边界

- 未改生产源码/测试/期望/配置/基线/他人卡/索引；未跑 `gen_index`；未跑 59 套回归；未安装依赖；未连设备/部署/提交/推送。
- 原 **3 policy + 12 anchor** 外债**保留原样**，本卡**不尝试跨 ns 消红**。
- **本卡不是第一阶段完成声明**；`G4/I0` 全局零红硬门仍未满足。
- 全部候选仍须后续 requirements → implementation → verification → review → integration，**当前不执行**。

### 6.10 审计数据口径修正（t50 补充，只改数据口径：未重扫大树、未改源码、未重建 Baseline）

> 依据 `_tmp_session/TRIM-EXEC-20261029/AUDIT-CAR/p2_audit_clarification.json`，**全部用已有 JSON 重算**。

#### (1) 分类互斥/穷尽 + 我的算术笔误

| 项 | 值 |
|---|---|
| 分类规则 | **11 个 class 互斥且穷尽**：每文件恰 1 个，`SUM == total_files` |
| 实测 SUM | `4965+778+174+36+10+9+8+7+7+3+2` = **5999** = `total_files` ✔ |
| 本域定义 | = `self_dev_source`+`self_dev_tool`+`self_dev_frozen`+`self_dev_entry` = `174+36+8+3` = **221** |
| **我的笔误** | §6.3 曾写 `self_dev_source 164` ⇒ 使四项相加成 **211**，与 221 不符 |
| 错因 | 174 已是**扣掉 10 个 SDK 后**的数；我又多减一次 10 得 164。**221 本身是对的** |
| 是否「10 个 SDK 仍留 221 域」 | **否**。10 个 SDK 的 `class` 已改为 `third_party_sdk`，**不在本域四类内** |

#### (2) 语义 6 / 215 是否含 SDK

| 项 | 值 |
|---|---|
| 语义阅读 6 个的 class | **全部 `self_dev_source`**（dzbrain core：main/config/blackboard/events/geometry/logging） |
| 6 个中含 SDK | **0** |
| 215 未读中含 SDK | **0**（SDK 10 个已移出本域统计，既不在这 6 也不在这 215） |

#### (3) 入口 66 是**文件去重 union**，各形态不可相加

| 项 | 值 |
|---|---|
| `entries.count` | **66** |
| `len(items)` / 唯一文件数 | 66 / **66** ⇒ **is_file_dedup_union = true** |
| 形态计数 | `__main__` 55 · `def main()` 47 · ROS2 launch 8 · setup entry_points 4 · `python -m` 1 |
| 形态合计 | **115 ≠ 66** —— **可重叠、不可相加**（同一文件可同时是 `__main__` guard 与 `def main()`） |

#### (4) 只读证明的正确解读（不让 before/after git 相同冒充内容全不变）

| 证据 | 值 | 说明 |
|---|---|---|
| `git status --short` 前后 | v1 **57 行** / dzrobot_v2 **8 行** identical | **status 相同 ≠ 内容全不变**（已跟踪文件只反映内容/索引态；未跟踪目录内部改动不展开到文件级） |
| **主证据：逐文件 hash** | **5998 未变 / 1 变 / 0 缺失**（5999 文件） | 唯一变化 `dzrobot_v2/docs/TEAM_STATUS.md`（mtime 16:52:29 晚于采集）＝**Lead 单列并发写** |
| 结论 | **以逐 hash 为主证据**，不因 status 相同而宣称内容全不变 |

#### (5) 决策语义澄清

`保留/待核` **不是**「已论证可精简」的反向，也**不是**「永久保留」的结论；仅表示**本卡不做删减判定**，须后续 requirements→implementation→verification→review→integration。

### 8.1 实际读取范围（区间并集 + 两来源分层）

| 文件 | 行数 | 我本人读段并集 | 父补读段并集 | **合并并集** | 覆盖率 | 未读 |
|---|---|---|---|---|---|---|
| `dzrobot_v2/src/dzbrain/dzbrain/core/config.py` | 945 | 466（1-90, 570-945） | 674（1-569, 760-864） | **945** | **100%** | 无 |
| `dzrobot_v2/src/dzbrain/dzbrain/core/blackboard.py` | 170 | 170 | — | **170** | **100%** | 无 |
| `dzrobot_v2/src/dzbrain/dzbrain/core/events.py` | 140 | 140 | — | **140** | **100%** | 无 |
| `dzrobot_v2/src/dzbrain/dzbrain/core/geometry.py` | 191 | 191 | — | **191** | **100%** | 无 |
| `dzrobot_v2/src/dzbrain/dzbrain/core/logging.py` | 80 | 80 | — | **80** | **100%** | 无 |

**合计 1526 行，合并并集 1526 行（100%），未读 0。**

> **★ 证据来源分层（不得混称）**：`config.py` 的 **91-569 段（SCHEMA 中段）由父补读**，原始记录在 `_tmp_session/TRIM-EXEC-20261029/lead/config_schema_parent_read.json`（含 `readRanges: [[91,569],[1,90],[760,864]]` 与 sha256 `fa12ca42…`）。**我本人只读 1-90 与 570-945（466 行）并对其语义摘要负责**；合并并集 100% 是**两来源合计**，**不得写成「trim-doc 独自读完整 945 行」**。

**口径（按 Lead 指出更正两次）**：
1. 覆盖按**区间并集**计算，不累加 raw chunk 长度；`dedup_removed` 单列。
2. **两来源分层**：`my_segments` / `parent_complement_segments` / `merged_segments` 分别记录，未读段单列。

### 8.4 实质发现（**DC1-1 已撤回**；仅余 1 条 QA 记录）

**★ DC1-1（原 low）已 RETRACTED —— 我统计方法有错，前提不成立**

| 项 | 内容 |
|---|---|
| 原声明 | 「`critical=True` 计数为 0，与 docstring 表述有偏差；219 键全部可缺省」 |
| **错因** | **我只用 AST keyword 分支统计，漏了位置参数**。`Spec` 字段顺序为 `default, kind, unit, lo, hi, desc, critical`（L31-39），**critical 是第 7 个位置参数**（index 6） |
| **实证** | L316 `"route.tokens": Spec(None, list, "", None, None, "mission_blocks（格名 token 列表）", True)` —— 第 7 位传 `True` |
| **重算** | 按 keyword：**0**；按位置参数第 7 位：**1（`route.tokens`）**；合计 **1** |
| **正确结论** | `critical=True` 实际有 **1 个**；`get()` L791-792 缺该键时 `raise ConfigError("关键配置缺失: …")` ⇒ **机制与 docstring 表述一致，无偏差** |
| docstring 复核 | 第 11 行「关键键（critical=True）缺失 → 报错」是**条件契约**，不因当前键数多少而失真。我原把它误读为「应有若干键设 True」 |
| **教训** | **AST 统计必须同时覆盖 positional args 与 keywords**；只查 keyword 会漏掉「用位置参传第 7 字段」的全部实例。这与我在 t55 把 `_pos in fence_spans`（元组成员测试）误读为区间包含是同一类「只看一层表象」的错 |

**DC1-2（qa，非代码缺陷）**：`blackboard.py` 读段 `[1,90]`+`[90,170]` 相邻重叠 1 行，raw 累加 171 → 并集 170。**这是审计计数口径问题，不是代码缺陷，也不是可精简项**，已按区间并集纠正。

> 本批**无代码级 finding**。

### 8.5 无可精简项（不预设必须有可删项；**不宣 219 键全部活跃**）

**本批五个文件均无可精简/可删项**：

| 文件 | 不可删理由 |
|---|---|
| `dzrobot_v2/src/dzbrain/dzbrain/core/config.py` | SCHEMA 共 **219** 键，均为**已登记配置契约**（类型/单位/范围/说明）；其中 `route.tokens` 为 `critical=True`（缺则 `ConfigError`）。`_migrate_legacy` 的 47 条旧键映射是双向兼容层，删了旧 YAML 就读不到。**注：本轮未逐个核验每键的全部消费者，故不宣「219 键全部活跃」**，只宣「均为已登记契约、无据本轮证据可判为可删」 |
| `dzrobot_v2/src/dzbrain/dzbrain/core/blackboard.py` | 共享键常量 + 读写契约注释；「`None` → 完全 no-op、逐位一致」是防回归承诺 |
| `dzrobot_v2/src/dzbrain/dzbrain/core/events.py` | 事件常量与 `EventBus` 实现全活跃；「禁止用事件传高频数据」是架构约束说明 |
| `dzrobot_v2/src/dzbrain/dzbrain/core/geometry.py` | docstring 明写「不可退让的契约：公式与旧代码逐字一致，任何改动都会让车全盘错位」 |
| `dzrobot_v2/src/dzbrain/dzbrain/core/logging.py` | `banner`/`Throttle`/`FaultGuard` 是「绝不静默报成功」判据的支撑件 |

**纪律**：grep 零命中、无静态调用、文件名重复**均不构成死代码证据**；本批未做任何删除建议。

### 8.3 默认值契约（**现值与历史注释因果分等级**）

| 键 | 当前默认 | 证据等级 |
|---|---|---|
| `drone.field_layout_wait_s` | **60.0**（注释记原 30.0） | **现值我本人核**（570-945 段）；**变更理由属历史决策说明**，弧我只证「注释文本确实这么说」，不证现场已验证 |
| `step.layout_ttl_s` | **0.0**（注释记原 4.0，语义为关闭新鲜度门） | 同上；**0 = 关闭，不是立即过期** |
| `step.pose_ready_wait_s` / `step.odom_watch_s` | 8.0 / 6.0 | 同上（注释称 2026-10-18 现场 pose_stale 事故后新增） |
| `safety.startup_grace_s` | 3.0 | 同上 |
| `safety.pose_timeout_s` / `safety.task_stall_s` | 1.0 / 1.0 | 现值我本人核 |

> **证据等级声明**：上表「当前默认」是我读段内实测的**现值**；括注的「原 30.0 / 原 4.0 / 事故后新增」是**源码注释所述的历史因果**，我**未独立验证**其现场效果（那需实跑证据）。二者不得混为一谈。
>
> 这些键属冻结区契约，**本卡只读记录不改**；改配置须按 AGENTS.md 跑 G1（**error 0 ∧ warn 0**，不是只看退出码）。

### 8.6 零写入证明（含冻结区确认）

| 项 | 结果 |
|---|---|
| 5 目标文件前后 SHA | **5/5 SAME**（`fa12ca42` / `9ba4bc81` / `01922f38` / `d932fc8b` / `d80f314e`） |
| 变更文件数 | **0** ⇒ ZERO_WRITE |
| git v1 / dzrobot_v2 | 23 → 23 / 8 → 8，dzrobot_v2 **状态逐行 identical** |
| 冻结区归属 | 5/5 全部位于 `dzrobot_v2/src/dzbrain/dzbrain/core/`，**属冻结区，只读未改** |

### 8.7 机器可读产物

全部在 `_tmp_session/TRIM-EXEC-20261029/AUDIT-CAR/deep-r1/`：

| 文件 | 内容 |
|---|---|
| `_tmp_session/TRIM-EXEC-20261029/AUDIT-CAR/deep-r1/body_read_manifest.json` | 逐文件 `my_segments` / `parent_complement_segments` / `merged_segments` / `my_union_lines` / `parent_union_lines` / `union_lines_read` / `unread_segments` / `coverage_pct` / `evidence_grade` / sha256；含 summary 与 coverage_algorithm |
| `_tmp_session/TRIM-EXEC-20261029/AUDIT-CAR/deep-r1/findings.json` | **DC1-1 已撤回**（`retracted` 段含原声明/错因/实证/重算/正确结论/教训）+ DC1-2（qa）+ `no_deletion_candidates` |
| `_tmp_session/TRIM-EXEC-20261029/AUDIT-CAR/deep-r1/semantic_summary.json` | 5 文件十项语义字段（含 config 的 default_value_contracts、blackboard 的 shared_key_contracts） |
| `_tmp_session/TRIM-EXEC-20261029/AUDIT-CAR/deep-r1/no_write_proof.json` | 前后 SHA / git / 冻结区确认 |
| `_tmp_session/TRIM-EXEC-20261029/AUDIT-CAR/deep-r1/pre_state.json` | 开工前基线 |

### 8.8 边界与未做

- **未改**任何源码/配置/测试/基线/索引/他人卡；**未 import 生产模块、未执行 Spec 表达式**（仅 `ast.parse` 辅助定位与读正文）。
- 未跑 59 套、未做新全仓扫、未建巨副本、未新建长期工具或第二套能力。
- 跨文件查证仅按实际调用点必要少量（见各文件 `callers` 字段），**未扩展成已覆盖整个包**。
- **`main.py` 不在本批**（原部分读不当完成），保留为后续缺口。
- **不宣「219 键全部活跃」**：消费者未逐个核验。
- **不宣「trim-doc 独自读完整 config.py」**：91-569 段证据来自父补读。
- 原 3 policy + 12 anchor 外债保留原样，不跨 ns 消红。
