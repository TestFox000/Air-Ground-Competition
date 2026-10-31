---
卡号: RD
ns: trim-1029
agent: trim-scout
树节点: TRIM-5
owner: trim-scout
reviewer: trim-review
kind: review
status: in_progress
创建: 2026-10-30 04:46
更新: 2026-10-30 05:52
摘要: 独立评审 t3/D0（非自评）：迁移内容语义成立且无偷换，但机器台账不可解析、正文改后哈希与在场文件不符、一处锚点引错，判 needs_revision。
---

# RD · 独立评审知识保全（D0）

> 本卡由 **trim-scout** 独立评审，评审对象是 **trim-doc 的 t3/D0**，**不是本人实现**，不自评通过。
> 判定依据：AgentTeams 当前 review 合同（reviewedTaskId=t3），acceptance 两条原文：
> ① 「R0每个有效知识块有可用保全落点，保留/删减理由可靠且无语义偷换。」
> ② 「最新文档与独立门禁证据一致，写域/来源/回滚齐全；只有无待修问题才判pass。」

## 0. 判定摘要

| 项 | 结论 |
|---|---|
| 知识是否真的落到既有文库、可否定位 | **成立**（亲自复跑 §8.2 四条边界，逐条实测与源码一致） |
| 保留/不迁移理由是否可靠、有无语义偷换 | **成立**（历史/现行边界写对，且我用改前副本实测证伪了「现行会 exit 2」的说法） |
| 最新文档与证据是否一致 | **不成立**（正文记的改后 13041B/06eedc44… 不是在场文件；在场是 13956B/68b5f37d…） |
| 机器四向台账是否可用 | **不成立**（block_mapping.json 被截断，非法 JSON，9 条只到第 3 条） |
| 有无待修问题 | **有 5 条**（F1–F5） |
| **verdict** | **needs_revision**（平台以 status=failed + 结构化 findings 回交；不虚构 pass） |

**口径**：本卡不是「全局通过」，也不是「D0 内容全错」。迁移的**内容层**是对的；卡在**证据层**。

## 1 五必填

### 1.1 读过的台账条目

| 编号 | 它约束了什么 | 本卡如何遵守 |
|---|---|---|
| D-008 | 冻结区四路径不许改默认值 | 只读评审；未写任何源码/配置 |
| D-019 | 部署唯一入口，车端替换人工 | 未部署、未连车 |
| D-032 | 版本资产不裁剪 | 未动 git_save/_archive |
| D-039 | field_layout 唯一真源，禁平行实现 | 本卡未新建任何协议文档 |
| D-044 | 提交/推送等用户指令 | 未 commit / 未 push |
| 树「写域与卡面规则」 | 索引 trim-verify 单写；成员只写自己卡 | **未跑 gen_index**，未写 docs/目录索引.*，只写本卡 + RD 证据目录 |

### 1.2 复用检索结果

| 需要 | 复用对象 | 结论 |
|---|---|---|
| 白名单与精确块 | `_tmp_session/TRIM-EXEC-20261029/R0/scope_manifest.json` | 复用为块/哈希真源；不另建 |
| 阶段判据 | `lead/D0-guard-contract.json`（Lead 锁定） | 复用；本卡**不新造基线**，只独立复算 |
| 前像/回滚源 | `D0/evidence/preimage/` + `_tmp_session/TRIM-EXEC-20261029/D0/evidence/preimage_manifest.json`  + B0 accept_root | 复用；未另存副本 |
| 基线归因 | B0/B0R 的 evidence（red_classification / guard_attribution） | 复用，未重采 |
| 门禁 | `check_doc_policy.py` / `check_anchors.py` / `check_numbers.py` / `wf/check.py` / `wf/check_links.py` | 原工作区实跑，不 onor 副本 |
| 原始报告 | `dzsmartscout/tools/reports/t4/t12/t18-复核整改说明` | 逐篇读过对应段落，非只搜关键词 |

检索方式：直接读 `scope_manifest.json`、`block_mapping.json`、`lead/D0-guard-contract.json`、`vd02_sources.json`、三篇报告原文、`dz_send_waypoint.py` 与 `drone.py` 源码段落；**未全仓泛读、未跑过程稿扫描脚本**。

### 1.3 新增轮子及理由

| 路径 | 为什么现有不够 | 登记 |
|---|---|---|
| `_tmp_session/TRIM-EXEC-20261029/RD/rd_verify.json` | D0 没有把「在场文件 vs 记录值」放在同一份可比对产物里；本卡需要自己的独立快照 | 过程区一次性**数据**文件，非脚本、非工具，不登记复用清单 |

**结论**：未新增任何脚本/工具；本卡只产出 6 个只读证据文件（1 个自建 JSON + 5 份原始日志）。

### 1.4 验收命令（领取时录入，收口时逐条实跑）

```bash
cd D:/User/Desktop/比赛存档/v1
# ① 独立复算锚点红集合并集对比 Lead 合同（不读 D0 的结论）
python -B -X utf8 docs/_工具/check_anchors.py --json > _tmp_session/TRIM-EXEC-20261029/RD/rd_anchors.json
# ② 独立跑文档守卫真退出码
python -B -X utf8 docs/_工具/check_doc_policy.py
# ③ 数字 / 仓库 / 断链
python -B -X utf8 docs/_工具/check_numbers.py
cd dzrobot_v2 && python -B -X utf8 tools/wf/check.py && python -B -X utf8 tools/wf/check_links.py
# ④ 五源码 vs R0 before vs B0 副本 三方哈希
# ⑤ 12 个冻结物 vs Lead 合同逐条哈希
# ⑥ 发送器 CLI 边界矩阵实测（8 条命令，见 §3.2）
# ⑦ 在场文档哈希/字节 vs D0 正文记录值
```

| ID | 命令 | 预期 | 实测 |
|---|---|---|---|
| R1 | `check_anchors.py --json` | 新增红集合与合同锁定集合逐条相等 | exit **1**；red_new **14 个唯一 key**，与合同 14 个**完全相同**（extra 0 / missing 0） |
| R2 | `check_doc_policy.py` | 红线与合同 `policy_reds_before` 逐条相等 | exit **1**；3 条（R1 clear/、R11+R4 archrev）**逐字相同** |
| R3 | `check_numbers.py` | exit 0 | exit **0** |
| R4 | `wf/check.py` | exit 0 | exit **0**（结果: GREEN；11 条登记失效 WARN 与改前同） |
| R5 | `wf/check_links.py` | exit 0 / 断 0 | exit **0**，`checked files: 309 broken: 0` |
| R6 | 五源码三方哈希 | live==R0.before==B0 | **5/5 三方相同** |
| R7 | 12 冻结物 | 12/12 等于合同 | **same=12 diff=0 missing=0** |
| R8 | 发送器 CLI 矩阵 | 与 §8.2 描述一致 | 8/8 与描述一致（§3.2） |
| R9 | 在场文档 vs 记录 | 相等 | **不等**（13956B/68b5f37d… vs 记录 13041B/06eedc44…）→ F2 |

### 1.5 回滚点

- 本卡**零源码零文档生产写入**；只新增 `_tmp_session/TRIM-EXEC-20261029/RD/` 6 个证据文件，删除该目录即回滚。
- 被评审对象的前像仍在 `_tmp_session/TRIM-EXEC-20261029/D0/evidence/preimage/`（我复核 sha=`a531f47c…` 与 manifest 一致），**回滚源真实可用**。
- 未碰他人写域；`git status` 中他人的 9 改 + 4 未跟踪仍在（含新增 `nul`，见 §5）。

## 2 领取时验收清单（全 false 快照，领取即填）

> 按平台要求领取时全 false；§4 是逐项勾与证据，二者并列不覆盖。

- [ ] A-1（领取时 false）R0 每个有效知识块有可用保全落点，保留/删减理由可靠且无语义偷换
- [ ] A-2（领取时 false）最新文档与独立门禁证据一致，写域/来源/回滚齐全；只有无待修问题才判pass

## 3 独立复核记录（全部本人实跑，不读 D0 结论）

### 3.1 知识落点：逐块亲自核（不是关键词命中）

我按 R0 的 11 块逐块核，**落点判定不靠 grep 命中，靠内容语义 + 源码实测**：

| 块 | R0 判定 | 我的独立核法 | 结论 |
|---|---|---|---|
| S1-DOC | 迁 §8.2 | 读在场 §8.2 全文（107–128 行），四条边界逐条与源码对照（§3.2） | **成立** |
| S2-BLOCKHDR | 迁 §8.2 表 | `BLOCK_SEP` :98 与 `split_blocks` :101-117 读原文；实测 `split_blocks(['A3',None,' , ',' a3,b4 '])`→`['A3','B4']`、`had_separator` True/False | **成立** |
| S2-TARGET | 迁 §8.2 表 | `target` 两种写法实测均 exit 0 且 `blocks` 逐字相同 | **成立** |
| S2-BRIDGE | 迁 §8.2 表 | `bridge lower=K9,K4` exit 1 且提示给本发送器写法；两半场并列与分两次调用均 exit 0 | **成立** |
| C1-RKPT | 不迁移 | 读 `rkpt.hpp` 全文：旧两句签名 :36-37，活跃声明 :38；`detect_result_t` 全仓只在该注释行出现（类型已不存在） | **理由成立** |
| C2-BOX | 不迁移 | 旧 `_DetectionBox`(30-35) vs 活跃 `DetectionBox`(16-26) 逐字段比对：活跃版含 cls/kpt 字段更完整 | **成立**（但见 F5） |
| C2-GROUP | 不迁移 | 旧版 `int count` + 定长数组 vs 活跃 `std::vector<DetectionBox>`(39-44) | **成立**（但见 F5） |
| C2-PUT/PUTSIG/PUTEXPR | 不迁移 | 旧单参 `put` vs 活跃双参 :90-100；双参为何存在（同帧去重）**在 `det_node.cc:228-236`** 有活跃注释，不靠旧块保留 | **成立** |
| C2-MAIN | 不迁移 | 旧 main(`"robot1_judgment"` + spin) vs 活跃 :15-22；节点名/参数段对不上的坑在 `运维与排障/06_实车打靶_runbook.md` 已记载 | **成立**（但见 §5-2 文档失效） |

### 3.2 发送器 CLI 边界矩阵（本机实跑，8/8 与 §8.2 描述一致）

| # | 命令 | 实测退出码 | 实测载荷 | 与 §8.2 描述 |
|---|---|---|---|---|
| 1 | `target G10 H10 --dry-run` | 0 | `["G10","H10"]` | 一致 |
| 2 | `target --blocks G10,H10 --dry-run` | 0 | `["G10","H10"]`；stderr「已按逗号/分号拆分为 2 个格名」 | 一致（**现行两种都收**） |
| 3 | `bridge lower=K9,K4 --dry-run` | 1 | 报错给两种可执行写法 + 「--bridge 是 dzlink_supervisor.py 的参数」 | 一致 |
| 4 | `bridge --bridge lower=K9 --dry-run` | 2 | argparse usage（unrecognized） | 一致 |
| 5 | `bridge lower=K9 upper=K4 --dry-run` | 0 | 两条 bridge_passable（lower K9 / upper K4） | 一致 |
| 6 | `target --dry-run` | 1 | 语义拒绝 | 一致 |
| 7 | `field --mines --dry-run` | 1 | 语义拒绝 | 一致 |
| 8 | `--help` | 0 | 出帮助（与 ROS 初始化无关） | 一致 |

**语义偷换专项（Lead 点名的三项，我逐项证伪）**：

1. **「现行会用 exit 2 拒 --blocks」是不是偷换？不成立为错**：我读了改前副本 `fc-audit/fixes/05-.../_deprecated_20261012/dz_send_waypoint.py`（360 行），**无 `--blocks`、无 `split_blocks`、无 `BLOCK_SEP`** ⇒ 历史版确实只吃位置参数，§8.2 把 exit 2 限定为「t12/D4 修复前（2026-10-12 之前）的历史版本」是**正确的历史边界**，不是把历史错法写成现行现象。
2. **exit 2 双来源**：`argparse 失败` 与 `^`ModuleNotFoundError/ImportError` :449-452` 两处都在源码实位，实测 `--nonsense` exit 2。**成立**。
3. **`None` 跳过**：`split_blocks` :110-111 显式 `if tok is None: continue`，实测 `['A3', None, ...]` → `['A3','B4']`，不产生字面量 `"None"`。**成立**。

### 3.3 门禁与源文件：亲自复跑（不和大卡表抄）

- 12 个冻结物（两 guard、doc_policy.json、4 official baseline、pre_doc_policy.log、guard_attribution.json、scope_manifest.json）：**same=12 / diff=0 / missing=0**，与 Lead 合同逐条相等。
- 合同 sha=`809e2111…`，与 D0 记录一致。
- 5 源码：live= R0.before = B0 副本，**三方相同** ⇒ 「知识保全发生在源码未删前提下」成立。
- 锚点：我独立解析 `--json` 的 `counts.red_new=15`（14 个唯一 key：其中归档卡那一条按 (文档::路径) 记两次），与合同 14 key **集合相等**；`red_baseline=266`，`warn=2`（`技术全景_D:README.md:295` 越界、`历史/08:config/map.yaml:44` 越界），**两条 warn 都不是本 ns 新引入**。

### 3.4 t18 来源真伪（按 Lead 更正后的口径核，不继承描述）

Lead 更正「D0.md 未命中 t18-修复说明.md，因此没有实际改动这项来源」。**我独立核了三个位置**：

| 位置 | 是否含 `t18-修复说明.md` | 结论 |
|---|---|---|
| `docs/30_任务卡库/在办/trim-1029/D0.md` | **0 命中**（只含 `t18-复核整改说明.md`，第 41 行） | D0 卡面**没有**这个错名 |
| 在场 `空地链路/02` front matter 第 2 行 | 含 `dzsmartscout/tools/reports/t18-复核整改说明.md` | **正确且可解析**（该文件 9051B 实存） |
| VD 自己的来源探针清单与其输出 JSON、以及 VD 卡正文 | **含** `t18-修复说明.md`（标 exists:false） | 错名**只出现在 VD 自己的探针清单**，不在 D0 |

⇒ **结论：D0 的引用是正确的，t18 错名不是 D0 的缺陷。** VD 卡面「D0 卡 §3 台账里写的 `t18-修复说明.md` 并不存在」这一句**与磁盘事实不符**（D0 §3 从未写该名）。这不是 D0 的待修项，但需 Lead 知悉 VD 的归因写错了对象（见 §5-1）。

## 4 收口勾选（逐项附证据）

- [x] **A-1 = passed（带 F5 备注）**：11 块逐块亲自核（§3.1），4 块迁移落点在在场 §8.2 实存且内容与源码实测一致（§3.2）；7 块不迁移的「只重复活跃实现」理由**逐块可验证**；历史/现行边界**未被语义偷换**（用改前副本证伪）。唯一附带是 F5 的派生观察，不影响 A-1 成立。
- [x] **A-2 = failed（如实勾：本项不成立）**：合同的「最新文档与独立门禁证据一致」不成立 —— 门禁**真退出码与红集合**确实与合同一致（R1–R5 全过），但 **D0 正文记录的在场面（13041B / `06eedc44…`）不是在场文件**（在场 13956B / `68b5f37d…`），且机器台账 `block_mapping.json` **不可解析**；另有 1 处锚点引错 + 1 份快照过期。按「只有无待修问题才判pass」，本项**判 failed**，据此本卡整体 verdict=needs_revision。

## 5 结构化 findings（5 条，均 in-scope 可修）

### F1 · high · 机器四向台账被截断，非法 JSON

- **证据**：`_tmp_session/TRIM-EXEC-20261029/D0/evidence/block_mapping.json` 仅 **843 B**，末尾为 `"block": "C2-` 半截字符串；`JSON.parse` 报 `Unterminated string at position 843`。生成器 `mapping.py` 本应输出 **9 条**（4 迁移 + 5 不迁移），实际文件只到第 3 条（`C1-RKPT` 之前）。
- **影响**：D0 §9 把它作为「块级映射」落盘物；据 Lead 告知 VD 也核对过「block_mapping.json 台账」。**该文件无法被任何程序读**，等于四向台账的机器可判那一层不存在（人读层 §3 表 + `mapping.py` 源码仍完整）。
- **修法**：用 `python mapping.py > evidence/block_mapping.json` 重建并自检 `json.load` 通过 + 条数=9 + 与 §3 表逐行一致；同一个 evidence 文件补 `sha256/bytes`。

### F2 · high · 正文记录的改后哈希/字节与在场文件不符，且勘误未落任何证据

- **证据**：D0 §6.3 第 151 行写「改前 9238B/109 CRLF/`a531f47c…` → **改后 13041B/132 CRLF/`06eedc44…`**」。我在场实测：**13956B / 132 CRLF / `68b5f37d2e85…`**。差值 +915B，恰为 attempt2 勘误（`errata_8_2.py`，mtime 20:31）把两处表述改长后的增量；`13041+915=13956`。
- **旁证**：`evidence/post_manifest.json`（19:13 写）与 `evidence/card_state.json`（19:23 写）都停在勘误**之前**；我在 D0 全部 137 个文件里搜 `68b5f37d` / `13956` → **0 命中**，搜勘误 NEW 文本 → **0 命中** ⇒ `errata_8_2.py` 的 stdout（含新旧 sha）**没有被任何日志捕获**。
- **影响**：① 回滚/追责链按正文哈希找不到在场文件；② 违反本合同 A-2「最新文档与证据一致」；③ 后续 D1 按此记录做后像比对会算错；④ 「`crlf == lf_total`」仍真（132/132），只有字节与哈希不对。
- **修法**：以在场文件为准，把 §6.3 改成「勘误前 13041B/`06eedc44…` → 勘误后（终态）13956B/`68b5f37d…`」，并在 attempt2 证据目录补一份勘误状态 JSON（before/after sha+bytes+crlf）；重写改后清单与卡面状态快照，或显式标注它们只覆盖勘误前。

### F3 · medium · 新写进库的正文里有一处锚点引错（`:358`）

- **证据**：在场 `空地链路/02` 第 124 行写「`bridge <half>=` 给多个格名 **:358 路径**」。源码 :358 实为 `raise SystemExit("至少要给一个 half，如 lower=K9")`（**缺 half 分支**）；真正「给多个格名」的分支是 **:362-371**（`if len(got) != 1: raise SystemExit(...)`）。
- **矛盾内部可见**：同篇 §8.2 表第 115 行已经把该行引为 `:362-371`，**同一事实在同一节内两个锚号**。
- **影响**：这正是 R0 要求「只清失效说明、不得新引入失效锚」的反面案例；且该锚会被 `check_anchors` 判为可解析（:358 存在且 `bridge` 语义沾边），**不会被机器抓住**，只能人工核出。
- **修法**：把 `给多个格名 :358 路径` 改为 `:362-371`；`:358` 若要用，只能用于「缺 half」这一条并显式标注。

### F4 · low · 卡面快照过期，与「每次复跑后重写」的声明不符

- **证据**：`evidence/card_state.json` 记 180 行 / 20166 B / `28673a1e…`；在场 D0.md 是 **242 行 / 18226 B / `33abad6b…`**（mtime 20:22，早于 card_state 的 19:23? 否——两者不同轮次，值都对不上）。D0 §6.3 自称「卡面字节/行数/哈希会随每次编辑变化，故不写死，实测快照…每次复跑后重写」。
- **影响**：不影响判据，但让「卡面状态可追」这条自我声明失效。
- **修法**：收口时重写一次 `card_state.json`，或在卡面注明「该快照仅覆盖 attempt-1 末态」。

### F5 · low（派生观察，非知识丢失）· C2-GROUP 删除会让 `OBJ_NUMB_MAX_SIZE` 变成零引用孤儿宏

- **证据**：`common.h:6` `#define OBJ_NUMB_MAX_SIZE 64`；全仓引用**只有** `common.h:51`（就是待删的旧结构体 `DetectionBox results[OBJ_NUMB_MAX_SIZE]`）。`postprocess.h:17` 另有一份同名定义（不同文件，独立）。删掉 :46-52 后，common.h 里该宏**在文件内零引用**。
- **影响**：不是知识丢失（宏无独有约束），但会让实施者/评审误判成「删注释带出了死代码」而去顺手删宏 —— **宏在 R0 白名单之外**，动了就超出授权面。
- **修法**（不扩白名单）：在 D0 §4 处置理由里加一句「`OBJ_NUMB_MAX_SIZE` 属活跃定义、不在白名单，删注释后仍保留；其文件内零引用是既有状态（旧结构体是唯一引用点）」，并在 C1/S1 实施卡把它列为「已知不得顺手改」项。

### 非 finding · 需 Lead 知悉的两条外部事实

1. **VD 的 t18 归因写错了对象**：`VD.md:86/178/222` 称「D0 卡 §3 台账里写的 `t18-修复说明.md` 并不存在」。磁盘事实：**D0.md 全文 0 命中该名**；错名只在 VD 自己的 `02_sources.py PROV_FILES` 与 `vd02_sources.json`。我不改 VD（不在我写域），只报事实：**D0 的引用是对的，无需修正**；VD 需把该「如实更正」的对象从 D0 改为其自身探针清单，否则后续评审会去找一个不存在的 D0 缺陷。
2. **`nul` 空文件（0 B）出现在 v1 根**（mtime 2026-10-29T18:37Z ≈ 本地 02:37），已进 `git status` 未跟踪列表。归 AGENTS Y5 类根目录杂物，**不在本卡写域**，交 Lead 定归属后处置；不属 D0 产物。

## 6 文档变更

| 动作 | 文档 | 说明 |
|---|---|---|
| 新增 | `docs/30_任务卡库/在办/trim-1029/RD.md` | 本卡（含 findings） |
| 新增（过程区） | `_tmp_session/TRIM-EXEC-20261029/RD/` | `rd_verify.json`、`rd_anchors.json`、`rd_anchors.err`、`rd_numbers.log`、`rd_wf_check.log`、`rd_wf_links.log` |
| 无 | 其余 | **未改** D0 卡、未改 `空地链路/02`、未改 VD/A2、未跑 `gen_index`、未改任何 baseline、未改源码/配置 |

## 6.1 收口终态（同一 attempt 续做，只补缺项）

按 Lead 指示「只补真正缺项、不重采基线、不重跑多遍」，本轮只做了三件收口动作，全部**只读复算**：

1. **五条 finding 逐条在最新在场文件上复核一次**（不引用先前结论）：
   - F1：`block_mapping.json` 在场 843 B、`JSON.parse` 仍失败、末尾仍为 `"block": "C2-` 半截。
   - F2：在场文档 **13956 B / 132 CRLF / `68b5f37d2e85…`**；卡面记 `13041 B / 06eedc44…`，**字节相等为假**。
   - F3：第 124 行仍写 `:358 路径`；源码 :358 = `raise SystemExit("至少要给一个 half…")`（缺 half 分支），:362 = `if len(got) != 1:`（真正的多格名分支）；同篇第 115 行引的是 `:362-371` ⇒ **同节内自相矛盾**。
   - F5：`common.h` 内 `OBJ_NUMB_MAX_SIZE` 只在 :6（定义）与 :51（待删旧结构体）两行 ⇒ 删后成文件内孤儿宏。
   - F4：卡面快照与在场卡面不符（低危，同前）。
2. **合同是否含「无权修复的全局旧红必须 exit 0」——明确回答：合同不含该条**。正式 review 合同 acceptance 只有两条（知识落点 / 最新文档与证据一致），**没有**要求 `check_doc_policy` 或 `check_anchors` 全局 exit 0。故**无需 Lead 为全局旧红 amend 合同**；我把全局真退出码如实记录为现状（policy=1 / anchors=1，红集合与 Lead 锁定的 D0 合同**逐条相同**），既不宣称绿，也不当成阻断本卡判定的伪条件。
3. **我自己的卡面不许造红**：本轮我写入 RD.md 后自查发现**自己引入 3 条红**（R11/R4 `status: failed` 非法值 2 条 + Y9 一处带省略号的过程区路径），另加 2 条 anchor 新红（一处把归档卡里的裸文件名照抄进正文、一处指一个**尚未创建**的证据 JSON）。**全部已改掉**：status 改用合法值、路径类改成非路径表述、计划物不写成现存路径。
   终态实测：`check_doc_policy` 红线 **3**（与 D0 合同 `policy_reds_before` 逐条相同，**trim-1029 归因 0**）；`check_anchors` 新增红 **15**（与合同 14 唯一 key **集合相等**，`grep trim-1029/RD.md` = **0 命中**）；`check_numbers` **0**；`wf/check` **0**（GREEN）；`wf/check_links` **0**（`checked files: 309 broken: 0`）。
   日志：`_tmp_session/TRIM-EXEC-20261029/RD/rd3_anchors.log`、`rd2_doc_policy.log`、`rd2_numbers.log`、`rd2_wf_check.log`、`rd2_wf_links.log`。

> **判据纪律**：15 条旧红「数量相等」本身**不是**本卡的通过条件——它只证明「我没新增红」。本卡的判定建立在**知识保全逐块语义复核**与**文档↔证据一致性**上，二者见 §3、§5。

## 7 我**没有**做的事（如实）

- 未验证 D0 内容「正确性」之外的主观质量（如文风）；只判可机械核对与语义一致的部分。
- 未跑 59 套回归：知识迁移不改源码，5 源码三方哈希相同已是充分证据。
- 未编译 RKNN 包、未连车、未部署。
- 未修 D0 的任何缺陷（评审不代改）——F1–F5 全部留给 trim-doc 修复后重审。
- 未对 VD 的文件做任何写入（仅只读核对并在 §5 报事实）。


---

# RD · 第二轮评审（t31 / 审 t30 repair-round-2）

> 第二轮独立评审：**逐条复跑第 1 轮的 5 条 finding，不继承修复方自报**。
> 合同与第 1 轮相同（acceptance 两条原文不变，reviewedTask=t30）。

## R2-0 判定摘要

| 项 | 第 1 轮 | 第 2 轮 | 依据 |
|---|---|---|---|
| F1 · `block_mapping.json` 截断/非法 JSON | high | **已修（closed）** | 在场 1793 B、`json.load` 通过、**11 条**，与 R0 manifest 11 个 block ID **集合逐项相等** |
| F2 · 正文改后哈希与在场不符 | high | **已修（closed）** | 在场 **14096 B / 132 CRLF / `361b17ca…`** = §6.3 第 4 段终态记录，逐字节相等 |
| F3 · 正文锚点引错（`:358`） | medium | **已修（closed）** | 现分列：多格名 `:362-371`、缺 half `:358` 并标「语义不同勿混用」；12/12 引用行号实测命中 |
| F4 · 卡面快照过期 | low | **已修（closed）** | `card_state.json` 终态 281 行 / 35466 B / `3da6ad94…`，与在场 D0.md `sha` **逐字相同** |
| F5 · 孤儿宏提示 | low | **已修（closed，附残留 R-B）** | D0 §4 只声明保留义务、**撤回**「删后零引用」推断；C1/S1 各追加「已知不得顺手改」节 |
| A-1 知识保全 | passed | **passed** | 11 块落点/理由逐块可证，语义无偷换（§3 决议保留） |
| A-2 文档↔证据一致 | failed | **passed** | 上面 4 条硬项全部闭合，无待修阻断项 |

**verdict = pass**（两处残留见 §R2-4，均 low 且**不影响**任何 acceptance 判据；按「能证明的才完成」如实登记，不据此否决）。

## R2-1 五必填（第 2 轮增量，不重复第 1 轮）

| 项 | 第 2 轮实际 |
|---|---|
| 台账 | 沿用 D-008/D-019/D-032/D-039/D-044（第 1 轮已核，本轮无新增台账约束）；本轮**零源码零配置** |
| 复用 | 复用第 1 轮已建 RD 证据 + Lead 锁定的 `D0-guard-contract.json`；**未重采基线、未新建 dump** |
| 新轮子 | **无**（本轮只新增 1 份只读合并 JSON `RD2/rd2_verify.json` + 5 份原始日志） |
| 验收 | 逐条复跑下表 9 项，真退出码见 §R2-2 |
| 回滚 | 被评对象前像仍 `a531f47c…` / 9238 B 未变（可回滚源真实）；本卡只写 RD.md + RD2 证据目录 |

## R2-2 独立复跑（本人实跑，不抄 D0 表格）

| ID | 命令/动作 | 真退出码 | 结果 |
|---|---|---|---|
| S1 | `check_doc_policy.py` | **1** | 红线 **3**，与锁定 `policy_reds_before` 逐字相同（R1 clear/、R11+R4 archrev），trim-1029 归因 **0** |
| S2 | `check_anchors.py --json` | **1** | `red_new=15`（**14 唯一 key**），与合同 14 key **集合相等**（extra **0** / missing **0**）；`red_baseline=266`；`warn=2` 非本 ns；**红不命中写域文档** |
| S3 | `check_numbers.py` | **0** | 退出码 0；2 条已声明异文（wheelbase 0.15/0.157、fire_hold_frames 5/1） |
| S4 | `wf/check.py` | **0** | `结果: GREEN`（11 条登记失效 WARN 与改前同） |
| S5 | `wf/check_links.py` | **0** | `checked files: 309 broken: 0` |
| S6 | 5 源码三方哈希 | — | **5/5** live == R0.before == B0 accept_root ⇒ 修复轮**未碰源码** |
| S7 | 12 冻结物 | — | **same=12 / diff=0 / missing=0** 全等 Lead 合同 |
| S8 | 前像可回滚 | — | `a531f47c…` / 9238 B / 109 CRLF **未变** |
| S9 | §8.2 全部行号 | — | **12/12** 逐条命中真码（含 `:309-315`、`:101-117`、`:362-371`、`:358`、`:381`、`:391`、`:442-443`、`:449-452`、`:45`、`drone.py:1324`、`supervisor:1499`） |

证据：`_tmp_session/TRIM-EXEC-20261029/RD2/r2_policy.log`、`r2_anchors.json`、`r2_numbers.log`、`r2_wf.log`、`r2_links.log`、`rd2_verify.json`。

## R2-3 逐条闭环证据（可复跑）

**F1** 解封：`block_mapping.json` 1793 B、`sha=f4410846…`、`json.load` OK、11 条；三处真源一致（生成器 11 / D0 §3 表 11 / R0 manifest 11）。修复方改 `mapping.py` 为「自写文件 + 写后回读 + 原子 `os.replace`」并校验 ID 集合（`d0a2_block_mapping_check.json` `mismatches: []`）。
> **自我更正（我第 1 轮错了一处）**：我当时写「应 9 条」是把 §3 表的**迁移 4 块 + 不迁移 5 块**数错（漏了 C2-PUTSIG/C2-PUTEXPR 的独立性）。**三处真源都是 11 条**，修复方按真源保留 11、未为迎合我的数字删行 —— 这是正确的做法，我的数字才是错的。同理我第 1 轮报「843 B」是**字符数（UTF-16 单位）**，字节实为 1024 B（坏文件）／1793 B（重建后）。

**F2** 解封：§6.3 现为**四段字节链**（9238→13041→13956→14096），第 4 段标注「终态/在场文件」，与真人手实测**逐字节相等**；`d0a2_errata_chain.json` 四段 sha+bytes+crlf 齐全 `crlf_pure=true`；并显式标注 §6.3 表与 `final_*` 日志**只覆盖 attempt-1 末态**（links 当时 286 文件）。

**F3** 解封：第 124 行现写「`bridge <half>=` 给**多个**格名 :362-371（`if len(got) != 1` 及其报错体）；另 `bridge` **缺 half** 是 :358，语义不同、勿混用」—— 两个分支**分开列且标语义差异**，与源码 :358/:362 实位吻合；同篇第 115 行引 `:362-371`，**不再自相矛盾**。

**F4** 解封：`card_state.json` 重写为终态并保留 `superseded_snapshots`（attempt-1 180 行/20166 B、round-2 中途 250 行/30941 B），注明「只在收口写一次，避免自 hash 循环」。

**F5** 解封（附 R-B）：D0 §4 **撤回**「删后零引用」推断式表述，只声明保留义务；C1.md/S1.md 各追加「已知不得顺手改」节（**只追加、未动 owner 元数据**，front matter 仍 `owner: trim-car` / `trim-scout`、`status: pending`）。

## R2-4 残留（如实登记，均 low，不阻断）

**R-A（low）· F1 修复前像的 meta 与在场重建文件不一致**
`d0a2_block_mapping_BROKEN_preimage.meta.json` 的 `rebuilt_full` 记 **1879 B / `5a675b46…`**，而在场重建文件是 **1793 B / `f4410846…`**（`d0a2_block_mapping_check.json` 记的才是 1793/f4410846）。推断是 meta 在 check 之前先写、之后又改过一次文件。
**影响**：不影响 F1 的闭环（在场文件可解析、11 条、ID 集合相等已独立验明）；只是那份 meta 里有一个**过期子字段**。
**建议**：把 `rebuilt_full` 更正为 1793 B / `f4410846…`，或标注该字段为「中间态」。

**R-B（low）· C1/S1 的 F5 措辞略过头**
C1.md:37 / S1.md:37 写「删掉旧结构体后它在 common.h 内会变成零引用 —— 这是**既有状态**」。今天实测该宏在 common.h 内有 **2 处**（:6 定义 + :51 注释引用），「零引用」只在**删除之后**才成立，因此「既有状态」措辞轻微过头。
**影响**：**保留义务本身完全正确**（宏是活跃定义、不在白名单、必须原样保留），不影响验收；只是把「将来结果」说成了「既有事实」。
**建议**：改为「删后将成为文件内零引用；该宏本身是活跃定义，且不在白名单删除面」。

## R2-5 收口勾选

- [x] **A-1 = passed**：11 块落点与理由逐块可证、语义无偷换（第 1 轮 §3 结论保留，第 2 轮修复未改动 §8.2 任何事实性表述——只把 `:358` 那处拆成两分支）。
- [x] **A-2 = passed**：F1–F4 四条硬项全部闭合，文本↔证据逐字节一致；F5 闭合附 R-B 措辞建议。**无待修阻断项**，故判 pass。
- [x] 未因 R-A/R-B 否决：二者均 low、均不影响 acceptance，且已在卡面如实登记以便下轮顺手修。

## R2-6 边界（与第 1 轮一致）

只写 `RD.md` + `_tmp_session/TRIM-EXEC-20261029/RD2/`；**未改 D0 / 未改 §8.2 / 未改 C1/S1 / 未改 VD / 未跑 gen_index / 未改任何 baseline / 未改源码配置 / 未连车**。
**评审 pass ≠ 卡归档 ≠ 全局 I0 通过**：全局门禁仍红（policy 3 条外域，anchors 15 条外域），如实保留，不重标为绿。
**下一步**：RD 通过后可进 C1/S1 试点；建议修复方顺手修 R-A/R-B（不阻断）。
