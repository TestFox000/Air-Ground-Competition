---
卡号: P2-ENG
ns: trim-1029
agent: trim-verify
树节点: TRIM-P2ENG
owner: trim-verify
reviewer: lead
kind: work
status: pending
更新: 2026-10-30
摘要: 第一阶段行政门未闭时的只读预研，不开启生产改造、不宣称第一阶段完成。
---

# P2-ENG · 工程根与工具链只读覆盖预研

## 1 目标与边界
两生产树以外的自研源码、根入口、文档工具和构建测试链；先确认实际目录，不假定某路径存在。两生产树由另外两卡覆盖，不能把 B0 有限采样当全项目。

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

## P2-ENG 根与工具链只读覆盖预研（owner 追加，2026-10-30 · attempt cca8eb80）

> **这是 G4 行政前置未闭期间的只读预研，不代表第一阶段完成，不解锁任何改源。**
> 全程未改任何源/配置/测试/基线/他人卡/索引/历史实体；未跑实机、未部署/提交/推送、未重跑 59、未安装依赖。

## 1 范围与实际枚举（不假定目录）

| 区域 | 文件数 | 性质 |
|---|---|---|
| `tools/` | **3** | 自研工程工具（根级） |
| `docs/_工具/` | **53** | 文档/守卫工具链 |
| `memory/` | **6** | 记忆服务独立包 |
| `artifacts/` | **24** | 数据产物 |
| **合计** | **86** | — |

**未纳入**：两生产树 `dzrobot_v2/`、`dzsmartscout/`（清单由另两卡提供，**不重复覆盖**）。
根目录除 `AGENTS.md`/`README.md` 外**无**构建脚本、无 CI 配置（`.github` 不存在）。

## 2 证据分级

| 等级 | 数量 | 含义 |
|---|---|---|
| `semantic_read` | **49** | 已静态读源码判断入口/角色，**未执行** |
| `inventory_only` | **37** | 只登记存在/大小/SHA，未做语义读取 |

**合同入口（保留）8 个**：`gen_index.py`（§4-1，当前发布者=Lead）、`check_doc_policy.py`（§4-2）、`check_anchors.py`（§4-3，G1 治理对象）、`check_numbers.py`（§4-4）、`snapshot_docs.py`（§4-7）、`doc_search.py`（MCP 后端）、`mcp_dzdoc_server.py`（MCP 入口）、`test_check_anchors.py`（G1 新增，41 例）。

**待核 32 个**：命名似一次性探针（`cleanup_*`/`salvage_*`/`verify_tars*` 等）。

> **纪律**：`grep-0`、无静态调用、文件名重复**均不证明**死代码。这 32 个只标「待核」，**没有任何一个被判废除**，也未列入精简候选。

## 3 跨两域交界脚本（不漏不重）

以下脚本**位于远侧但会读取生产树**，移交两生产树卡时须按交界标注，避免漏或重：

```
tools/pack_handoff.py, docs/_工具/check_anchors.py, check_doc_policy.py,
cleanup_check.py, cleanup_detail.py, cleanup_exec_p1p2.py, cleanup_fix2.py, cleanup_gap.py …
```

它们**归本卡（远侧）所有**，但其输入触及生产树；生产树卡不应重复登记同名文件。

## 4 工程测试 runner：真实退出码聚合缺陷（只静态取证）

| 项 | 事实 |
|---|---|
| 文件 | `dzrobot_v2/_run_full_regression.py`（**生产树，非本卡清单项**，仅静态只读） |
| 逐套 exit | **有**：落盘一份逐套 CSV 结果表（含 exit 列），逐套记 `proc.returncode`；本文不写该表裸文件名以免撞同名 |
| 聚合刹车 | **无**：全文件 `sys.exit` 出现 **0** 次 |
| 后果 | 有红时只 `print` 汇总，**总退出码恒 0** ⇒ 调用方无法据此判红 |
| 交叉印证 | B0 卡 §2 已独立记录同一事实（「原 runner 总 exit 恒 0、且会覆盖旧报告」） |

**这是静态取证，我不改实现、也不重开已裁决项**；仅作为「集成判据不可依赖该 runner 退出码」的证据。

**零检查**：`memory/tests/test_dzmem.py` 用标准 `unittest.main()`（有真退出码）；远侧**不存在**其它聚合 runner，`grep sys.exit(1)` 在远侧 0 命中。

## 5 G4 覆盖边界（按任务卡要求附注）

| 事实 | 核实结果 |
|---|---|
| G4 的 **14** 个恢复对象全部落在 B0 采样 822 输入内 | **True**（`all_within_b0_sample=True`） |
| 新建 `test_check_anchors.py` | **不在** B0 采样、**也不在** G4 的 14 个恢复对象内 |
| 因此 G4 恢复演练**未覆盖** | 该新建 test、采样外 `02`/`06` 知识库、以及其它未采文件 |

**G4 JSON 性质区别（不可互相替代）**：

| 文件 | 性质 |
|---|---|
| `G4/g4_rollback.json` | 我做的 14 文件**隔离恢复演练**（含故意损坏对照）→ 验证**可恢复性** |
| `lead/G4-index-preservation.json` | Lead 的索引发布前后对照（298→298 / 外部 262→262）→ 验证**索引不丢失** |

**当前 G1 前像 4 份在位**：`preimage_r3/check_anchors.py` 39212 B、`preimage_r3/test_check_anchors.py` 27682 B（另两份 docs 前像在 `G1/preimage`）。**不能声称覆盖采样外全部文件。**

## 6 owner 归属更正（不照搬旧 owner）

| 卡 | 当前归属 |
|---|---|
| S1 | 历史 owner = **trim-scout**；当前证据一致性由 **trim-doc** 接手（t42） |
| S2 | 当前作者 = **trim-doc** |

## 7 零写入证明

| 项 | 结果 |
|---|---|
| 纳入清单文件数 | **86** |
| 前后 SHA 变化 | **0** |
| 缺失 | **0** |
| 完全一致 | **true** |

`inv_before.sha256`（86 行）与事后逐文件重算**逐条相同**，证明本卡**未写任何实体**。只写过程产物 `_tmp_session/TRIM-EXEC-20261029/AUDIT-ENGINEERING/`。

## 8 结论

| 项 | 判定 |
|---|---|
| 实际目录已枚举（不假定） | **成立**（tools 3 / _工具 53 / memory 6 / artifacts 24） |
| 每文件记录 path/来源/证据等级/分类/entry/caller/验证需求 | **成立**（清单主文件 86 行，见 §12） |
| 未凭 grep-0 或命名判死代码 | **成立**（32 个仅标待核，0 判废除） |
| 交界脚本标明不漏不重 | **成立**（专列一节） |
| 零写入证明（86 文件 SHA 前后一致） | **成立** |
| 附 G4 覆盖边界（14 为子集、未覆盖采样外） | **成立** |

### 未覆盖 / 未证（如实）

1. **32 个待核脚本未做语义读**，只到 inventory_only 之上的入口探测，**不判任何性质**。
2. **两生产树不在本卡范围**，其清单由另两卡提供，本卡不声称覆盖。
3. **G4 恢复演练只覆盖 14 个 B0 采样内文件**，未覆盖新建 test 与采样外知识库。
4. **未执行任何脚本**（只静态读），故「真实退出码」类结论只到**静态可证**层面。
5. **原 3 policy + 12 anchors 外债保留未动**，本卡不跨 ns 消红。

## 9 文档变更

| 动作 | 文档 |
|---|---|
| 更新 | `docs/30_任务卡库/在办/trim-1029/P2-ENG.md`（本卡） |
| 新增过程产物 | `_tmp_session/TRIM-EXEC-20261029/AUDIT-ENGINEERING/`（清单主文件 + 8 脚本 + 快照；见 §12） |
| 无 | 其余（未写源/配置/测试/基线/他人卡/索引） |

## 10 自查补正：两个字段曾留空，现已补齐（append-only，不改 §1–§9 原结论）

我提交前自查发现清单主文件有两处**不符合卡面要求**，已补齐并如实记录：

| 缺口 | 原状 | 现在 |
|---|---|---|
| `knowledge_landing` | **86/86 全为 null** | **0 空白**；无落点者显式记为「无（未见现行文档登记，缺证）」 |
| 四分类（保留/可精简/等价重构/缺证） | 只有自由文本 `classification` | **`classification_4way` 逐行已标**（原文本保留在 `classification_detail`） |

### 10.1 四分类实测分布

| 分类 | 数量 | 说明 |
|---|---|---|
| **保留** | **8** | 合同入口（AGENTS §4 步骤 + MCP 入口 + G1 单测） |
| **缺证** | **78** | 其中 32 个命名似探针、9 个角色未定、37 个非 py/生成物 —— **均未做语义读，不得据此删改** |
| 可精简 | **0** | **本卡不列任何一项**（未证成重复/取代） |
| 等价重构 | **0** | **本卡不列任何一项**（未做等价证明） |

**「可精简 0 / 等价重构 0」不是漏填，是结论** —— 没有任何一项达到证据门槛。

### 10.2 48 个文件没有任何文档落点（这是缺证的直接实证）

| 情况 | 数量 |
|---|---|
| 自有知识落点（AGENTS 步骤/台账/卡/MCP） | 38 |
| **查不到任何现行文档登记** | **48** |

这 48 个（含 32 个 `cleanup_*`/`salvage_*` 探针与部分 `tools/` 脚本）**既无合同位置、也无文档落点**，属典型的「过程稿沉淀」。

> **但按纪律：无文档落点 ≠ 死代码。** 它们在 `docs/_工具/` 下、可能被历史卡引用过，**本卡一律留「缺证」，不列精简候选**。若后续要处置，须先做语义读 + 调用面取证。

## 11 更正：runner 退出码结论收窄（原误述保留，不改实现）

我原在 §4 写「**总退出码恒 0**」，这是**过宽推论**：`grep sys.exit 计数`不是控制流证据，且未捕获异常仍可非零退出。现按**实际控制流与行号**收窄。

### 11.1 逐行控制流证据（全文 47 行）

| 行 | 事实 | 意义 |
|---|---|---|
| **L23–L25** | `subprocess.run([...], cwd=WS, env=env, stdout=fh, stderr=STDOUT, timeout=1200)` | **未传 `check=True`** ⇒ 子套件非零不抛 `CalledProcessError`，主流程继续 |
| **L34–L35** | `rows.append({... 'exit': proc.returncode ...})` | 子套件退出码**被记录**（进 CSV 与逐套打印），但**不参与主流程控制** |
| **L44–L46** | `red = [r for r in rows if r['exit'] != 0]`；`print(套数/非零退出)`；`for r in red: print(RED...)` | 红**只被 print**，**无 return/raise/sys.exit** |
| **L47（EOF）** | 模块正常走到文件末尾 | 正常结束 ⇒ 进程退出码 0，**不依据任何子套件失败置非零** |
| **L18–L20** | `files = sorted(TESTS.glob('test_*.py'))`；`for name in files:` | glob 空 ⇒ `rows=[]` ⇒ `red=[]` ⇒ 只打印「套数=0」⇒ exit 0，**空套件不被拒绝** |

### 11.2 收窄后的准确结论

**可以证**：

| # | 结论 |
|---|---|
| 1 | 正常跑完（无未捕获异常）时，**不依据子套件失败置非零** —— 有红也只是 print，最终 exit 0 |
| 2 | **空套件（glob 为空）不被拒绝** —— 同样 exit 0 |
| 3 | 子套件退出码**有记录**（CSV exit 列 + 逐套打印），信息未丢失，**只是未形成刹车** |

**不能证 / 不许再说**：

| # | 禁止声称 |
|---|---|
| 1 | **「所有执行恒 0」** —— `timeout=1200` 触发的 `TimeoutExpired`、日志写入/读取的 IO 异常等**未捕获异常仍以非零退出** |
| 2 | 具体崩溃路径的退出值（本卡**不执行**该脚本） |

> 原误述「总退出码恒 0」**保留在 §4 供对照**，此处为更正记录。**未改该实现、未重跑 59、未执行该 runner**（只静态读）。

### 11.3 跨域归属

该文件在 **`dzrobot_v2/` 生产树（Car 域）**，**不属本卡清单项**。合并时按**跨域关联**标注，**不与 Car 域卡双算覆盖**。

## 12 actual-path 映射（供 t53 读真实产物，不造副本）

交付预期名与实际落盘名不同，以**实际路径**为准：

| 交付预期 | 实际路径 | 内容 |
|---|---|---|
| inventory | `AUDIT-ENGINEERING/inv_snapshot.json` | 86 文件 path/bytes/sha256/area/ext |
| inventory（分级） | `AUDIT-ENGINEERING/inv_classified.json` | 逐文件入口/角色/待核标记 |
| findings / 清单 | 本卡过程区 `_tmp_session/TRIM-EXEC-20261029/AUDIT-ENGINEERING/` 下的清单主文件（含 86 行 provenance/evidence_level/classification_4way/knowledge_landing/verification_need）；本文不写其裸文件名以免撞全库同名 |
| 零写入证明 | `AUDIT-ENGINEERING/inv_before.sha256` + `write_proof_and_boundary.json` | 86 行 SHA 快照 + 前后一致证明 |
| G4 边界 | `AUDIT-ENGINEERING/g4_boundary_corrected.json` | 14 恢复集 ⊆ B0 采样、新建 test 未覆盖 |
| runner 收窄 | `AUDIT-ENGINEERING/runner_exit_narrowed.json` | 逐行控制流证据 + 可证/不可证边界 |

**实际目录全部文件**：01–08 共 8 个带序号的过程脚本，加上清单主文件、分类快照、SHA 快照、零写入证明、G4 边界与 runner 收窄共 6 份 JSON/文本产物。**本文不逐个写裸文件名**（全库多份同名会撞锚点守卫），需要时按 §12 上表的前缀路径到本卡过程区取。

## 13 口径声明：本卡结论只按本域

| 数字 | 范围 |
|---|---|
| **86** 文件 | **仅远侧四区**（tools 3 / docs/_工具 53 / memory 6 / artifacts 24） |
| **49** semantic_read / **37** inventory_only | 同上，**不含两生产树** |
| **32** 待核 | 同上，**全部未做语义读** |
| 四分类：保留 **8** / 缺证 **78** / 可精简 **0** / 等价重构 **0** | 同上 |
| **48** 无文档落点 | 同上 |

**这不是「全工程审完」**：两生产树 `dzrobot_v2/`、`dzsmartscout/` 的清单由另两卡提供，本卡**一行未覆盖**其内容。

## deep-r1：八工具正文只读审计（owner 追加，2026-10-30 · attempt affe313f）

> **只读审计**，G4/I0 未闭，**不解锁任何新生产改动**。未执行、未 import、未运行设备/网络/部署；未改源/配置/测试/基线/索引/他人卡。

## 1 先补 t53 的两个数据边界

### 1.1 141 个未分配缺口：按真实文件集合归因（不是猜）

| 项 | 值 |
|---|---|
| 缺口总量 | 778 |
| 波次已分配 | 637 |
| **未分配** | **141** |
| 算术核对 | 778 − 637 = **141**，与实际未分配 **true** |

**实测归因（captain 的猜测被证实）**：

| 域 | 未分配 | 构成 |
|---|---|---|
| 车端 | **23** | `config_frozen` 7 + `doc_live` 9 + `root_misc` 7 |
| 机端 | **118** | `fc_audit` 79 + `root_misc` 39 |

**车端 23 + 机端 118 = 141**，与 778−637 完全吻合。原因是波次只按「自研类」划域，未覆盖同域其它 class。

### 1.2 原 49 个 `body_semantic_read` 的依据核验 → **全部降级**

| 检查项 | 结果 |
|---|---|
| 是否有行段记录 | **无** |
| 是否有函数体读取范围证据 | **无** |
| 原字段 | P2-ENG 的 `semantic_read`，判据只是「静态读了源码判入口/角色」 |

**按本卡要求处置**：不能因为全文被 Python 读去算 hash 就叫模型正文审过；**没有行段证据就不能宣称「正文全读」**。

⇒ **49 个全部从 `body_semantic_read` 降级为 `claimed_static_read（未经行段独立核）`**，
**真实 `body_semantic_read` 计数 = 0**（与 SCOUT r3 的 `body_lines_fully_read=0` 一致）。

> 我**不对全部 49 个做巨大重扫**；本批只把本卡 8 个目标真正逐行读完并记录行段。

## 2 八目标正文审计（全部逐行读完，无未读段）

| 文件 | 行数 | 实读行段 | 未读 | 副作用类别 | 外部进程 |
|---|---|---|---|---|---|
| cleanup_alive.py | 17 | 1–17 | 无 | 只读 | 否 |
| cleanup_audit_ab.py | 34 | 1–34 | 无 | 只读 | 否 |
| cleanup_check.py | 22 | 1–22 | 无 | 只读 | 否 |
| cleanup_detail.py | 31 | 1–31 | 无 | 只读 | 是 |
| cleanup_exec_p1p2.py | 78 | 1–78 | 无 | **破坏** | 是 |
| cleanup_fix2.py | 44 | 1–44 | 无 | 写入 | 是 |
| cleanup_gap.py | 35 | 1–35 | 无 | 写入 | 否 |
| cleanup_gitprobe.py | 23 | 1–23 | 无 | 写入 | 是 |

**8/8 完整读完，共 284 行，0 未读段。**

### 2.1 副作用分类（按代码行证据，非关键词）

| 类别 | 文件 |
|---|---|
| **破坏性（会删文件）** | `cleanup_exec_p1p2.py` |
| 写入（会创建/覆盖文件） | `cleanup_fix2.py`、`cleanup_gap.py`、`cleanup_gitprobe.py` |
| 只读 | `cleanup_alive.py`、`cleanup_audit_ab.py`、`cleanup_check.py`、`cleanup_detail.py` |
| 调用外部进程 | `cleanup_detail.py`（git ls-files）、`cleanup_gitprobe.py`（git --git-dir）、`cleanup_fix2.py`（git show）、`cleanup_exec_p1p2.py`（git ls-files） |

**一处自我更正**：先前用 `'写' in io_side_effects` 子串判断，把 **8 个全判成写脚本**（连「不写任何文件」也命中）。现改为逐文件按实际代码行证据判定。

## 3 Findings（5 项，均 OPEN，本卡未改任何代码）

| ID | 级别 | 文件 | 问题 |
|---|---|---|---|
| F1 | high | `docs/_工具/cleanup_exec_p1p2.py` | 破坏性删除脚本仍在现行目录，且其 git 保护闸只覆盖 dzrobot_v2 前缀 |
| F2 | medium | `docs/_工具/cleanup_fix2.py` | 「恢复」动作会从兄弟插件目录或 git HEAD 直接写目标文件，无哈希前置校验 |
| F3 | medium | `docs/_工具/cleanup_gap.py` | 全仓同哈希索引脚本吞掉 OSError，且会遍历整个 v1（含大目录） |
| F4 | low | `docs/_工具/cleanup_gitprobe.py` | 依赖硬编码的外部 git 仓路径 public_v1.git |
| F5 | info | `（本批 8 个文件）` | 8 个工具均无文档知识落点，属典型过程稿沉淀 |

### 3.1 F1 详情（最高优先级）

**`cleanup_exec_p1p2.py` 会 os.remove 删文件**，但其安全闸①「git 跟踪跳过」在 L19–L24 中：

```python
def git_tracked(rel):
    if not rel.startswith("dzrobot_v2/"): return False   # ← 非 dzrobot_v2 直接放行
```

⇒ **`dzsmartscout/` 与 v1 根下被 git 跟踪的重复文件不会被拦截**。这不是「有保护闸」就等于安全。

**处置建议**（本卡不实施）：扩展 git_tracked 覆盖面，或明确标注「仅限一次性历史任务、禁止重跑」并移入 legacy/。

## 4 零写入证明

| 检查 | 结果 |
|---|---|
| 8 个目标 sha256（读时 vs 事后） | **8/8 完全未变** |
| 原 ENG 86 文件快照 | **86 检查 / 0 变化** |
| git status 行数 | 23（**只作辅助，不作内容未变的证明**） |

**只写**：`P2-ENG.md` 与本卡过程子目录 `AUDIT-ENGINEERING/deep-r1/`。**8 个脚本一个都没跑、未 import、未联网。**

## 5 结论

| 项 | 判定 |
|---|---|
| t53 的 141 未分配缺口已按真实文件集合归因（23+118，算术吻合） | **成立** |
| 49 个 body_semantic_read 无行段证据 → 全部降级，真实正文读 = 0 | **成立** |
| 8 个目标逐行完整读取（284 行 / 0 未读段），含全部函数与顶层流程 | **成立** |
| 每文件记录 SHA / 实读行段 / 职责 / API / 状态并发 / I-O 副作用 / 异常退出 / 常量单位 / 动态入口 / 知识落点 | **成立** |
| 5 项 findings（1 high / 2 medium / 1 low / 1 info） | **成立** |
| 零写入（8 目标 + 86 快照全未变） | **成立** |
| 不以名称/grep0 判死代码 | **成立**（8 个全部标「缺证」，0 个判废除/可精简） |

### 未证 / 边界（如实）

1. **8 个仍未判废除**：只到「正文读完 + 缺证」，是否可精简需后续 requirements 阶段；
2. **跨文件调用关系只做了必要少量查证**（git/subprocess 调用点），**未扩展成整包覆盖**；
3. **48 个仍无文档落点**这类缺口不在本批；
4. 未跑 59、未联网、未执行任何脚本；**不解锁改源、不绕 G4 外域阻断**。

## deep-r2：下一组八个遗留工具正文只读审计（owner 追加，2026-10-30 · attempt 1439c523）

> **只读正文审计，不解锁新生产改动。** 原 G4/I0 仍受 3 policy + 12 anchor 外域前置阻断。
> 未执行、未 import、未运行删除/部署；未联网/设备；未改源码/配置/测试/期望/基线/索引/他人卡；未跑 59、未新全仓扫。

## 1 读取范围（实际路径 + SHA + 区间并集）

| 文件 | 行数 | 实读行段 | 未读段 | 结论 |
|---|---|---|---|---|
| cleanup_inventory.py | 99 | 1–99 | 无 | 读 |
| cleanup_left.py | 17 | 1–17 | 无 | 读 |
| cleanup_lost.py | 29 | 1–29 | 无 | 读 |
| cleanup_lost2.py | 52 | 1–52 | 无 | 读 |
| cleanup_personal.py | 21 | 1–21 | 无 | 读 |
| cleanup_probe.py | 32 | 1–32 | 无 | 读 |
| cleanup_probe2.py | 26 | 1–26 | 无 | 读 |
| cleanup_restore.py | 51 | 1–51 | 无 | 读 |

**8/8 逐行完整读完，共 327 行，0 未读段。**

> 机器读取字节算 SHA **不等于**语义审过；本批是逐行读代码，每文件给职责/API、状态并发、I/O、异常退出、调用约束、知识落点、未证点与候选理由。

## 2 副作用分类（按实际代码行，不按含「写」字或 grep0）

| 类别 | 文件 | 依据 |
|---|---|---|
| **破坏性删除** | `cleanup_personal.py` | L20 `os.remove(p)` 删 5 个个人域文件 |
| 写入 | `cleanup_inventory.py`（写 inventory.csv/摘要 JSON）、`cleanup_lost.py`（写 lost JSON）、`cleanup_lost2.py`（写 ds_recover.json）、`cleanup_restore.py`（写目标文件） | 各自 open(...'w')/json.dump/open(dst,'wb') |
| 只读 | `cleanup_left.py`、`cleanup_probe.py`、`cleanup_probe2.py` | 仅 walk + print |
| 调外部进程 | `cleanup_restore.py`（git --git-dir，跨 public_v1.git） | L22/L37 subprocess |

## 3 删除/覆盖闸的实际形态（本批重点）

| 脚本 | 闸 | 实测评价 |
|---|---|---|
| `cleanup_personal.py` | 先写 ledger 再删 | **无前像**；ledger **sha256 列写空串** ⇒ 事后无法验证被删内容；os.remove 无 try/except |
| `cleanup_restore.py` | **先比 sha 再落盘**（L23/L30） | **本批最严格**：只有旧版本时**不落盘仅记录**，避免以错版本覆盖 |
| `cleanup_inventory.py` | 保留份按 (len(rel), rel) 最小 | **无内容/时间依据**，路径字典序决定留哪份 |
| `cleanup_lost.py` | 同哈希找现存副本 | L17 libdir**声明后未使用** ⇒ 不区分库内/库外 |

## 4 幂等与异常

| 脚本 | 幂等性 | 异常风险 |
|---|---|---|
| `cleanup_personal.py` | **不幂等**：重复运行会重复写 ledger（同一 rel 多行） | os.remove 无保护，中途失败留半完成态 |
| `cleanup_inventory.py` | 幂等（覆盖写同一 CSV） | OSError 被吞（L22/L43），失败文件静默跳过 |
| `cleanup_restore.py` | 幂等（同哈希则重写同内容） | `inv[rel]` **KeyError 未保护**（L18），rel 不在清单即中断 |
| `cleanup_lost/lost2` | 幂等（覆盖写 JSON） | 依赖 inventory.csv 存在，缺失即崩 |

## 5 Findings（5 项，全 OPEN，未改任何代码）

| ID | 级别 | 文件 | 问题 |
|---|---|---|---|
| G1 | high | `cleanup_personal.py` | 不可逆删除脚本：无前像、ledger 的 sha256 字段留空，删除后无法验证被删内容 |
| G2 | medium | `cleanup_restore.py` | 恢复脚本依赖两个硬编码跨双仓绝对路径，且 inv[rel] 未做存在性保护 |
| G3 | medium | `cleanup_inventory.py` | 保留份由路径字典序决定，同哈希多份的「保留哪一份」无内容/时间依据 |
| G4 | low | `cleanup_lost.py` | 声明了 libdir 变量但从未使用（L17）；同哈希复原未区分库内/库外来源 |
| G5 | info | `（本批 8 个文件）` | 8 个工具均无 00/20 库文档落点，且依赖 docs/_work 中间产物链 |

### 5.1 G1 详情（最高优先级）

**`cleanup_personal.py` 是不可逆删除**：

```python
w.writerow(["P6", rel, "", os.path.getsize(p), "E", ...])   # ← sha256 位置是空串
os.remove(p)                                                # ← 无前像、无 try/except
```

**「前像真实性」不可证** —— ledger 里没有哈希，删完就无从验证删掉的是什么。

## 6 「无候选」如实

| 项 | 结论 |
|---|---|
| 可精简候选 | **0 个** |
| 等价重构候选 | **0 个**（无一项达到「确证收益」门槛） |
| 判废除 | **0 个** |

**不预设必须可删；无证一律保留待核。** 上表 5 项 finding 都是**缺陷/风险**，不是删改授权。

## 7 零写入证明

| 检查 | 结果 |
|---|---|
| 8 目标 sha（读时 vs 事后） | **8/8 完全未变** |
| 未执行 | 8 个脚本**一个都没跑**；未 import；未联网 |

**只写** `P2-ENG.md` 与 `AUDIT-ENGINEERING/deep-r2/`。旧 epoch（deep-r1 与前 8 个）**未覆写**。

## 8 结论

| 项 | 判定 |
|---|---|
| 8 目标逐行完整读取（327 行 / 0 未读段），按实际路径+SHA+区间并集 | **成立** |
| 每文件给职责/API、状态并发、I/O、异常退出、调用约束、知识落点、未证点 | **成立** |
| 删除/覆盖闸按实际代码行判定（不按「写」字/grep0） | **成立** |
| 5 项 findings，0 个可删/重构候选 | **成立** |
| 零写入（8 目标全未变，未执行） | **成立** |

### 未证 / 边界（如实）

1. **8 个仍未判废除**：只到「正文读完 + 缺证」；
2. **docs/_work 中间产物链未验证存在性**（inventory.csv 等由前序脚本生成，本卡未跑）；
3. **G1 的不可逆删除未做任何补救建议验证**，属风险登记；
4. **不重复前批 F1~F5**（cleanup_alive/exec_p1p2 等 8 个已在 deep-r1 记录）；
5. **G4/I0 原始 0 门仍失败**，不借本批证据绕收口。