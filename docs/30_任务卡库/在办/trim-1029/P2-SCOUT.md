---
卡号: P2-SCOUT
ns: trim-1029
agent: trim-doc
树节点: TRIM-P2SCOUT
owner: trim-doc
reviewer: trim-verify
kind: work
status: pending
更新: 2026-10-30
摘要: 第一阶段行政门未闭时的只读预研，不开启生产改造、不宣称第一阶段完成。
---

# P2-SCOUT · 机端全域只读覆盖预研

## 1 目标与边界
dzsmartscout 内自研生产源码、入口和全部工具边界；planner/vendor 的已知分叉只记录，不能同步。fc-audit/drone-scripts/历史实体只读分型，证据不足保留。

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

## 6 预研结论（append-only；以上 Lead 预置正文未改）

### 6.1 覆盖范围与证据等级（诚实分层，不冒充全覆盖）

**总盘点：`dzsmartscout/` 1727 个文件**（全量枚举 + SHA-256 逐个实测），按分区：

| 分区 | 文件数 | 处置 |
|---|---|---|
| `tools/`（自研生产/入口/工具） | **355** | 本次预研主体 |
| `src/`（第三方/派生，B0 已排除 162MB） | 1230 | 仅枚举，按来源保护 |
| `fc-audit/` | 79 | 仅枚举，按来源保护 |
| `drone-scripts/` | 7 | 仅枚举，按来源保护 |
| `tools/planner/vendor/`（已知分叉） | 17 | 仅枚举，**只审证据不同步** |
| 根目录杂项 | 39 | 混合（历史脚本/临时件），部分待核 |

**证据等级分布（不把枚举当读完）**：

| 等级 | 数量 | 含义 |
|---|---|---|
| `semantic_read` | **85** | 已读 AST 结构 / docstring（79 个自研 .py 全部解析成功 + 6 个核心大文件） |
| `artifact_identified` | 152 | 已识别为生成物/数据产物，**未读内容语义** |
| `identified_not_read` | 46 | 已按路径/类型识别，**未逐篇读完** |
| `待核` | **111** | **未读**（其中自研区 89，含 80 个 `.sh` 脚本） |
| `inventory_only` | 1333 | 仅枚举（来源保护分区） |

> **明确缺口**：**111 个文件未读**（自研区 89 个以 `.sh` 运维脚本为主 + 2 个第三方 ref + 根目录杂项 22）。**本轮不做全量语义通读，不宣称覆盖全部生产逻辑。**

### 6.2 入口与副作用（保留不明副作用入口）

- **可执行入口候选 66 个**（含 `__main__` 或 `argparse`），全部单独登记于 `entries.json`。
- **含副作用/外部交互 63 / 110 个 .py**：network 38、write_fs 28、subprocess 18、deploy_install 13、ssh_remote 13、device_io 10。
- **21 个判为「保留（副作用入口，须实物核）」** —— 凡触 SSH/部署/设备的，一律**不列入精简候选**。

### 6.3 一条实质发现：`dzsmartscout/tools/link/dzudp_core.py` 的「钉死快照」声明**成立且已量化**

文件头部自述「教员 udp_bridge 脚本的钉死快照（只同步不改）」，并给出 来源路径 / 取证镜像 / SHA-256。我做了三项独立核验：

1. **自报 SHA 不是本文件自身的 SHA**（本文件 `049145be…`，自报 `9ff259a3…`）—— 说明它记的是**原件**的 sha。
2. **实测取证镜像 `_tmp_session/sqc-20261024/tools/udp_bridge.py`（取证镜像） 的 sha 恰为 `9ff259a3…`** ⇒ 自报值属实、可验。
3. **逐行 diff**：教员镜像 304 行，当前 351 行，**304 行全部逐行相同**，差异仅为**前部插入 47 行本地头注**（含来源与 sha 声明）。

⇒ 「只同步不改」**成立**；该文件属**第三方派生（钉死快照）**，**不列入精简候选**，且**不得与上游再同步**（会破坏快照语义）。教员原件（`D` 盘桌面「教员交接」目录下的 udp_bridge 脚本）**当前不存在**，故上游不可再取。

### 6.4 候选精简（仅候选，未执行）

**118 个 `tools/reports/` 产物**（1.42 MB，`.txt/.json/.jsonl/.md/.log/.png`）判为「**候选精简（生成物，需确认可重生成）**」。
**判定条件未满足前不动**：须先证明每份产物可由现存脚本重生成、且有对应报告卡引用；否则保留。**本卡不执行任何删除。**

### 6.5 安全项（只脱敏报告）

识别 **5 个敏感路径**，**全部只记路径与大小、未读内容、未复制、未外传**：
`dzsmartscout/crack_vnc_passwd.py`、`dzsmartscout/tools/drone/known_hosts_dz`、`dzsmartscout/tools/link/dzlink.env`、`dzsmartscout/tools/link/dzros.env`、`dzsmartscout/tools/planner/EVIDENCE/plan_tokens.txt`。

### 6.6 零写入证明（机器证据）

| 项 | 前 | 后 | 判定 |
|---|---|---|---|
| `dzsmartscout` 文件数 | 1727 | 1727 | 同 |
| 新增 / 删除 / 内容变化 | — | **0 / 0 / 0** | **PASS（零写入）** |
| v1 `git status --short` 行数 | 23 | 23 | 未变 |
| `dzrobot_v2` `git status --short` 行数 | 8 | 8 | 未变 |

**既有脏状态归属**（未清理、未 reset/clean/stash）：`dzsmartscout/tools/link/dz_send_waypoint.py` 为 **S2（t15）既有改动**，`mtime 11:45:45` 远早于本轮；实测 sha `fde0886d…` == R0 `S2.after` ⇒ **非本卡引入**。

### 6.7 未做与边界

- 未改任何源/测试/期望/配置/基线/他人卡/索引；未部署、未连设备、未提交推送。
- 未重跑 59 套、未安装依赖、未跨 ns 消红。**原 3 policy + 12 anchor 外债原样保留**（未尝试处置）。
- 未做全仓语义通读；**111 个文件明确未读**（见 §6.1）。
- 复用 A2/R0/B0 资料，但**B0 为有限采样，不据此声称全覆盖**。

### 6.8 机器可读产物（全部在 `_tmp_session/TRIM-EXEC-20261029/AUDIT-SCOUT/`）

| 文件 | 内容 |
|---|---|
| `inventory_all.json` | 1727 全量清单（path / bytes / sha256 / zone） |
| `catalog.json` | **主清单**：每文件 path / bytes / sha256 / zone / purpose / evidence / disposition / effects / is_entry |
| `entries.json` | 66 个入口候选 |
| `side_effects.json` | 63 个副作用文件分类 |
| `semantic_samples.json` / `semantic_batch.json` | 语义读取结果（6 核心 + 79 自研 .py） |
| `_tmp_session/TRIM-EXEC-20261029/AUDIT-SCOUT/deep-r1/no_write_proof.json` | 前后哈希零写入证明 |
| `dirty_attribution.json` | 既有脏状态归属（S2） |
| `sensitive_paths_only.json` | 5 个敏感路径（仅路径与大小） |
| `pre_git_status.txt` / `post_git_status.txt` | 两仓前后 git 状态原始记录 |

### 6.9 经验总结（补充）

**现象**：`dzsmartscout/tools/link/dzudp_core.py` 头部自报一个 sha，但**不等于该文件自身 sha**——若直接采信「自报=自身」会得出错误结论。
**处置**：必须同时核「自报值是否等于某个可验证实体（此处是取证镜像）」+ 逐行 diff 量化差异，才能判定「钉死快照」是否真成立。本卡据此确认 304 行零改动、仅前插 47 行头注。

## 7 口径更正（r3 epoch · append-only，§6 历史表述原样保留不回写）

> **触发**：Lead 指出两处口径过宽。**不删、不伪改 §6 原文**；本节为新 epoch 的补充口径，后续 t53 请按本节读。

### 7.1 更正一：`semantic_read = 85` 标错了，实为「静态结构分析」

**问题**：任务原文明确「AST inventory 不等于语义读完」。我原把 85 个标成 `semantic_read`，**实际没有任何一个文件被逐行读完正文**。

**重叠关系（已核清）**：

| 集合 | 数量 | 说明 |
|---|---|---|
| 6 个核心大文件（读了结构+docstring+类/函数名） | 6 | `dzlink_supervisor.py` / `dzudp_adapter.py` / `dzudp_core.py` / `dz_send_waypoint.py` / `route_crop.py` / `perception/protocol.py` |
| 79 个自研 .py（AST 解析成功 + docstring 首行 + 类/函数名） | 79 | 见 `semantic_batch.json` |
| **交集** | **0** | **无重叠** |
| **并集** | **85** | **85 = 去重并集，不是重复计数** |

**按 actual-files 重分级（`r3/evidence_levels_r3.json`）**：

| 真实等级 | 数量 | 含义 |
|---|---|---|
| `static_ast_analysis` | **76** | docstring 首行 + 顶层类名/函数名（**不含正文逐行**） |
| `ast_parsed` | **9** | AST 可解析，但无可读语义摘要（如无 docstring 或无顶层符号） |
| `body_semantic_read` | **0** | **正文逐行语义读完（一个都没有）** |

**实测支撑（`r3/per_file_read_depth.json`）**：85 个文件共 **20164 行**；`docstring_head_read` 78/85；`body_lines_fully_read` **0/85**。

**结论口径（请以此为准）**：这 85 个只达到「**静态结构分析**」，**不是语义读完**。整体 1727 文件中仍有 **111 个明确未读**，**依旧不宣称语义全覆盖**。

### 7.2 更正二：敏感项「未读内容」说得过宽，字节其实已被读取算 SHA

**问题**：`inventory_gen.py` 对全部 1727 文件（含这 5 个敏感项）执行 `open(...,"rb").read()` **计算 sha256** ⇒ **字节已在本地被读取**。原写「未读内容」不准确。

**准确表述**：

| 维度 | 事实 |
|---|---|
| 字节读取 | **已发生**（为计算 sha256） |
| 原文进入模型上下文 | **否** |
| 原文复制到报告/过程区 | **否** |
| 原文或其可用摘要外传 | **否** |
| 额外读取这些文件内容 | **本轮未做** |

**输出边界**：公开报告只输出 **path / bytes / zone**（`sensitive_paths_only.json`）；`catalog.json` 另含全量 sha256 值。**不输出任何敏感原文、配置值、口令、密钥或主机指纹原文**。

### 7.3 不受影响的历史结论

- **零写入**：1727 文件前后 SHA 全同（新增/删除/变化 0/0/0）。
- **钉死快照结论**（`dzsmartscout/tools/link/dzudp_core.py`：教员镜像 304 行全同 + 前插 47 行头注）不受本次口径更正影响。
- 未改源码、未跑 59 套、未跨 ns 消红；原 3 policy + 12 anchor 外债原样保留。

### 7.4 更正后文件（新 epoch，原产物保留）

| 文件 | 性质 |
|---|---|
| `r3/CORRECTION_r3.json` | 结构化更正声明（含原表述、为何过宽、重叠核清、新分级、敏感口径） |
| `r3/evidence_levels_r3.json` | 85 条逐文件真实等级 + 读取操作清单 |
| `r3/per_file_read_depth.json` | 逐文件读取深度原始实测 |

> 原 `catalog.json` 与 `p2_section.txt` **原样保留为历史产物，未覆写**。

## 8 deep-r1 章 · 四个链路模块正文只读审计（t55，append-only；§6/§7 原样保留）

> **性质**：G4/I0 未闭期间的**只读正文审计**，不解锁任何新生产改动。静态阅读，未执行/未导入模块/未连网/未跑设备/未改源码。

### 8.1 实际读取范围（不把 AST/哈希当正文读完）

| 文件 | 行数 | 已读 | 覆盖率 | 未读缺口 |
|---|---|---|---|---|
| `dzsmartscout/tools/link/dz_send_waypoint.py` | 442 | 442 | **100%** | 无 |
| `dzsmartscout/tools/link/dzudp_adapter.py` | 1010 | 854 | **84.6%** | 191-333（Cell/落格细节）、583-675（FrameResult/AdapterConfig） |
| `dzsmartscout/tools/link/dzudp_core.py` | 351 | 351 | **100%** | 无 |
| `dzsmartscout/tools/link/dzlink_supervisor.py` | 2017 | 946 | **46.9%** | CommandFile、main/CLI、validate_route 等 6 段 |

**合计 3820 行，已读 2593 行（67.9%）**。未读区间逐段登记于 `_tmp_session/TRIM-EXEC-20261029/AUDIT-SCOUT/deep-r1/body_read_manifest.json`，**不宣称全文读完**。

### 8.2 ★ 实质发现：`--perception-topic` 路径实际失效（DR1-1，high）

**位置**：`dzsmartscout/tools/link/dzlink_supervisor.py` L1283-1289（`SupervisorNode._on_perception`）。

**证据（三重独立，非 grep 判死）**：

| 手段 | 结果 |
|---|---|
| AST | 函数体仅 2 条语句：`Assign` L1284 + `If` L1285-1289（**无 orelse**）；`If` 块体 4 条，`ingest_sample` 位于第 4 条 |
| 缩进 | L1289 缩进 **12**，在 `if obj is None:` 块内、且在 L1288 `return` **之后** |
| **字节码** | 反汇编 35 条指令，**L1289 未生成任何指令**；L1288 `RETURN_CONST` 后直接 `RETURN_CONST` 结束 |

**实际行为**：

| 输入 | 走向 |
|---|---|
| `obj is None`（坏 JSON） | diag + `samples_rejected += 1` + return（**正确**） |
| `obj` 有效（正常 JSON） | 跳过 if 块 ⇒ 函数直接结束 ⇒ **样本被静默丢弃，从不进入 `core.ingest_sample`** |

**影响面**：CLI 暴露 `--perception-topic`（L1485），传了即注册该订阅（L1273-1275）；但该路径下正常感知 JSON 全部丢弃。`--perception-glob` / `--perception-udp` 走别的分支，不受影响。

**测试零覆盖**：`_on_perception` / `perception_topic` / `perception-topic` 在 `dzsmartscout/tools/link/tests/*.py` **0 命中**；195 个用例未触及该函数——这解释了为何长期未被发现。

**归属**：**既有缺陷、非本轮引入**。B0 前像（97224 B / `277539bca28e70c2`）中该函数形态**逐行相同**，且现行文件 sha **与 B0 前像完全相同** ⇒ 自 B0 采集时即如此。

**处置**：**仅登记，本卡不改源码**。建议后续独立卡把 L1289 左移至 8 缩进（与 L1284 同级）并补一条覆盖该函数的单测。

### 8.3 钉死快照：前 47 行**含 6 个可执行常量**，不是纯头注（按任务要求落实）

`dzudp_core.py` 前 47 行结构：模块 docstring（L2-29）+ `#` 注释 + **6 个可执行常量赋值**（L32-45）。

| 常量 | 用途 | 外部引用（实测） |
|---|---|---|
| `PINNED_SOURCE` | 教员原件绝对路径（仅留痕，运行时不依赖） | 无外部引用 |
| `PINNED_SHA256` | 原件 SHA-256 | **`tests/test_dzudp_core_provenance.py` L78 断言** |
| `PINNED_BYTES` | 原件字节数 | **L79 断言** |
| `PINNED_DATE` | 抓取日期 | **L80 断言** |
| `PINNED_MIRROR` | 取证镜像路径 | 无外部引用 |
| `PINNED_MARKER` | 分界行（**故意拆两段隐式拼接**，避免源码中 marker 字节出现两次） | **L81 断言** |

⇒ **这 6 个常量绝不可当注释候选删除**：其中 4 个被 provenance 单测直接断言，删了单测必红。

**已证 / 未证（严格划界）**：

| 已证 | 未证 |
|---|---|
| 本地两实体（当前文件第 48-351 行 vs 取证镜像 304 行）**有序逐行全同** | **不证**「教师身份」——镜像只在本仓内 |
| 自报 `PINNED_SHA256` = 实测镜像 sha（`9ff259a3…`） | **不证**上游当前内容（未联网、未访问任何外部仓） |
| 本文件自身 sha 为 `049145be…`（≠ 自报值，因自报记的是镜像） | **不证**未来可同步（上游状态未知） |

### 8.4 R0/S1/S2 注释差分与真实代码职责分离

| 阶段 | 块 | 性质 | 与可执行逻辑的关系 |
|---|---|---|---|
| S1 | `S1-DOC` L47-56 | 模块 docstring 历史叙述 → 2 行指向 | **AST 剔除模块 docstring 后完全相同**（`b5af445a3d908974` 前后同值） |
| S2 | `S2-BLOCKHDR` / `S2-TARGET` / `S2-BRIDGE` | 冗余旧注释 | **S2 前像 vs 现行 AST 完全相同** ⇒ 纯注释，零可执行变化 |

⇒ **注释治理的差分与代码职责完全隔离**，本卡语义摘要只描述真实职责，不把注释过程当行为。

### 8.5 零写入证明

| 项 | 结果 |
|---|---|
| 4 目标文件前后 SHA | **4/4 SAME**（`fde0886d` / `bc523550` / `049145be` / `277539bc`） |
| 变更文件数 | **0** ⇒ ZERO_WRITE |
| git v1 / dzrobot_v2 | 23 → 23 / 8 → 8，**状态逐行 identical** |

### 8.6 机器可读产物（`_tmp_session/TRIM-EXEC-20261029/AUDIT-SCOUT/deep-r1/`）

| 文件 | 内容 |
|---|---|
| `_tmp_session/TRIM-EXEC-20261029/AUDIT-SCOUT/deep-r1/body_read_manifest.json` | 逐文件：sha256/bytes/lines/read_segments/unread/lines_read/lines_unread/read_coverage_pct |
| `_tmp_session/TRIM-EXEC-20261029/AUDIT-SCOUT/deep-r1/findings.json` | 结构化发现（DR1-1 含证据链、归属、处置建议） |
| `_tmp_session/TRIM-EXEC-20261029/AUDIT-SCOUT/deep-r1/semantic_summary.json` | 4 文件语义摘要：职责/公共 API/状态并发/IO 副作用/异常退出/常量单位/调用关系/知识落点/等价简化候选/未读缺口 |
| `_tmp_session/TRIM-EXEC-20261029/AUDIT-SCOUT/deep-r1/no_write_proof.json` | 前后 SHA 与 git 状态证明 |
| `_tmp_session/TRIM-EXEC-20261029/AUDIT-SCOUT/deep-r1/pre_state.json` | 开工前基线 |

### 8.7 边界与未做

- **未改**任何源码/配置/测试/基线/索引/他人卡；**未执行或导入任何模块**、未连网/设备/部署、未提交推送。
- 未跑 59 套、未做新全仓扫、未建巨副本、未新建长期工具或第二套能力。
- 跨文件查证仅按实际调用点做必要少量（provenance 单测常量引用、解析核借用点），**未扩展成「已覆盖整个包」**。
- **不宣称语义全覆盖**：两文件有明确未读段（见 8.1）。
- 原 3 policy + 12 anchor 外债保留原样，未跨 ns 消红。
