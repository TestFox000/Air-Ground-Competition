---
卡号: RC1
ns: trim-1029
agent: trim-scout
树节点: TRIM-10
owner: trim-scout
reviewer: trim-review
kind: review
status: in_progress
创建: 2026-10-30 08:23
更新: 2026-10-30 08:55
摘要: 独立评审 trim-car 的 C1（车端 rkpt.hpp 注释试点）；按文件系统证据重推实际变更集并逐项复跑，无待修问题，判 pass。
---

# RC1 · 独立评审试点车端最新实现（审查 t6 / C1）

> 评审对象是 **trim-car 的 C1**，**不是本人实现**，不自评通过。
> 合同 acceptance 两条原文：① 最新实现逐项满足R0与实施合同，无语义变化、知识遗失、越界修改或假绿。② 独立验证和恢复证据齐全且对应最新哈希，未修发现为零才verdict=pass。

## 0 判定摘要

| 项 | 结论 |
|---|---|
| 变更面是否只含 R0 白名单块 | **成立**（rknn_pt 树 27 文件仅 rkpt.hpp 变，diff 恰为删除那 2 行注释） |
| 是否语义变化 | **不成立**（预处理 token 流前后同 hash；纯声明头文件，无代码行变动） |
| 是否有知识遗失 | **不成立**（2 行是已不存在类型 `detect_result_t` 的旧签名；活跃声明 `:38` 原样保留） |
| 是否越界修改 | **无越界**（B0 822 输入中仅 8 个变更，全有主、0 未归因） |
| 是否假绿 | **不成立**（未跑项明确声明，未拿 GREEN/未跑冒充通过） |
| 待修问题 | **0** |
| **verdict** | **pass** |

## 1 五必填

### 1.1 读过的台账条目

| 编号 | 约束 | 本卡遵守 |
|---|---|---|
| D-008 | 冻结区四路径（含 `src/dzrknn_dataprocess/**`、`config/**`） | 只读评审；两者实测未变 |
| D-019 | 部署人工执行 | 未部署、未连车 |
| D-032 | 版本资产不裁剪 | 未动 git_save/_archive |
| D-039 | field_layout 唯一真源 | 与本卡无关，未涉协议 |
| D-044 | 提交/推送等用户指令 | 未 commit / 未 push |
| 树「写域与卡面规则」 | 索引 Lead 单写；成员只写自己卡 | 未跑 gen_index，只写本卡+RC1 证据 |

### 1.2 复用检索结果

| 需要 | 复用对象 | 结论 |
|---|---|---|
| 白名单与精确块 | `_tmp_session/TRIM-EXEC-20261029/R0/scope_manifest.json` | 复用为唯一真源 |
| 基线真源 | `B0/evidence/accept_copy_manifest.json`（822 输入 / 103 运行时产物） | **复用于重推变更集**，不另建基线 |
| 冻结证据 | `B0R/evidence/b0_frozen_manifest.json`（138 文件） | 复用复核 B0 未被污染 |
| C1 的验证器 | `C1/c1_apply_verify.py` | **只读复跑**，不继承其结论、不新造校验器 |
| 门禁 | `check_doc_policy` / `check_anchors` / `wf check` | 原工作区实跑 |

### 1.3 新增轮子及理由

**无。** 全部结论由「复用 C1 已有校验器 + 独立 diff/哈希/token 复算」得到，无须新工具。

### 1.4 验收命令（本人实跑）

| ID | 命令 | 真退出码 | 结果 |
|---|---|---|---|
| R1 | `check_doc_policy.py` | **1** | 红线 **3**（全外域 R1/R11+R4），**C1 归因 0** |
| R2 | `check_anchors.py --json` | **1** | red_new **18 / 16 唯一 key**；基线 15 条全保留；C1 归因 **0**；+2 属 B0/B0R 卡自身（早于本批） |
| R3 | `dzrobot_v2 && tools/wf/check.py` | **0** | `结果: GREEN`（GREEN ≠ 回归绿） |
| R4 | `test_control_contract.py --suite all` | **0** | **26 passed / 0 failed**（WSL g++ 15.2.0 真编译真回调） |
| R5 | `C1/c1_apply_verify.py` | **0** | **11/11 checks true**，`all_checks_pass=true` |
| R6 | 独立 diff（B0 前像 → 在场） | — | **恰 2 处删除**，无任何 `+` 行 |
| R7 | 预处理 token 流 md5 对比 | — | 前后 **`b1dbc35be86fd3dbf0e2f547d15bdb61` 完全相同** |
| R8 | B0 822 输入 vs 在场逐字节 | — | **8 变更 / 0 缺失**，8 条全有主 |

证据：`_tmp_session/TRIM-EXEC-20261029/RC1/`。

### 1.5 回滚点

- 被评对象前像：`B0/accept_root/…/rkpt.hpp`（`c4a1e1bf…` / 1007 B）**== R0 before** ⇒ 回滚源真实。
- 本卡零源码写入；只新增 RC1 证据目录，删除即回滚。

## 2 领取时验收清单（全 false 快照）

- [ ] AC-1（领取时 false）最新实现逐项满足R0与实施合同，无语义变化、知识遗失、越界修改或假绿。
- [ ] AC-2（领取时 false）独立验证和恢复证据齐全且对应最新哈希，未修发现为零才verdict=pass。

## 3 独立复核记录（全部本人实跑，不继承 C1 自报）

### 3.1 白名单块与哈希（三方核对）

| 项 | 实测 |
|---|---|
| 在场 sha / bytes | `ecc01722c67799ba…` / **906 B** / 0 CRLF |
| R0 `stages[0].after_sha256` | `ecc01722c67799ba…` → **逐字节相等** |
| R0 `before_sha256` | `c4a1e1bfbab9ab9f…` |
| B0 副本 sha | `c4a1e1bfbab9ab9f…` → **== R0 before** |
| 结论 | 在场 = R0 预测后像；B0 副本 = R0 前像 ⇒ 后像判据与回滚都成立 |

### 3.2 变更面：独立 diff 而非采信卡面

用 **B0 前像 → 在场** 逐行 LCS diff（不看 `git diff --numstat` 自报）：

```
changed lines: 2
  - pre:36 | // cv::Mat infer(cv::Mat &ori_img);
  - pre:37 | // std::vector<detect_result_t> infer(cv::Mat &ori_img);
```

**只有 2 行删除、0 新增**；字节 1007→906（−101，与 R0 `old_text` 实际长度一致）。

### 3.3 语义等价（我自己的判据，不用 C1 现成结论）

| 面 | 方法 | 结果 |
|---|---|---|
| 预处理+去注释 token 流 | `g++ -fpreprocessed -dD -E` → 去行号 → `tr -s` 归一 → md5 | **前后同 hash `b1dbc35b…`** |
| `__LINE__`/`#line` 敏感性 | 全 `src/rknn_pt` 树搜索 | 删行是整行 `//`，无续行、不落宏体、不参与 `#if` ⇒ 无敏感项 |
| 知识是否遗失 | `detect_result_t` 全仓命中 | **仅在被删那一行**（该类型已不存在）⇒ 非现行约束 |
| 局部必要说明保留 | 看活跃声明 | `DetectResultsGroup infer(cv::Mat &, int cur_frame_id);`（:38）**原样在位** |
| Python AST | 本批无 Python 改动 | 该项 **N/A**，未虚称通过 |

### 3.4 越界：从文件系统重推，不采信 changedPaths

按 B0 `copied_inputs`（**822 输入**）逐字节比对，**不**把 `runtime_outputs`（103 个测试产物）算入：

| 结论 | 值 |
|---|---|
| B0 输入 vs 在场 | **CHANGED 8 / MISSING 0** |

8 条全部有主：

| # | 文件 | owner |
|---|---|---|
| 1 | `dzrobot_v2/src/rknn_pt/include/det/rkpt.hpp` | **C1（t6）R0 授权** |
| 2 | `dzsmartscout/tools/link/dz_send_waypoint.py` | S1（t7）R0 授权 |
| 3–4 | `docs/00_项目总览/技术全景_D/E` | D1（t8）行号补偿 |
| 5–6 | `docs/目录索引.json` / `.md` | Lead 治理 |
| 7 | `dzrobot_v2/docs/TEAM_STATUS.md` | Lead 治理 |
| 8 | `dzrobot_v2/docs/决策台账.md` | Lead 治理 |

**我另查出 5 个「差异」是 B0 的运行产物，不是输入**（`artifacts/sim/` 下 traj.csv / by_segment.csv / traj.svg，以及 `test_autoplan_fixtures` 两件）—— 它们在 B0 `runtime_outputs` 里登记，是测试在副本内跑出来的。**若不区分输入/产物会误报 13 处变更；按输入口径才是 8**。这与 V1 的 8 条独立吻合。

### 3.5 保护资产零改动

| 资产 | vs B0 |
|---|---|
| `config/robot.yaml` | **SAME** |
| `config/map.yaml` | **SAME** |
| `tools/deploy_to_car.py` | **SAME** |
| `dzrknn_dataprocess.cpp` / `.h`（冻结区） | **SAME** |
| `rknn_pt/include/common.h`、`rknnPool.hpp`（C2 目标，本批未动） | **SAME** |
| `rknn_pt/include/det/rkpt.hpp` | **DIFF**（唯一，符合预期） |

`rknn_pt` 树 27 文件：**25 SAME / 1 DIFF（rkpt.hpp）/ 1 不在副本**（`model/light_det.rknn`，B0 按设计排除，只登记）。

### 3.6 B0 写域污染已闭环（C1 自曝，我复核）

C1 §6.5.2 承认曾误在 B0 写域建 `B0/C1/sim_copyroot/` 并 `rm -rf`。我独立复核：

| 复核项 | 实测 |
|---|---|
| `B0/` 顶层 | `accept_root / evidence / logs / restore_drill / scripts` —— **与开工一致，无 `C1/` 残留** |
| `B0R` 冻结清单的 **138** 个证据文件 | **same=138 / diff=0 / missing=0** ⇒ B0 证据链未被污染 |

### 3.7 E1/E2 证据保全不足是否诚实

C1 §6.5.1 末段写明「E1/E2 中间日志（16 条 / 18 条两版）在清理脚本目录时被一并删掉，现存只有终态日志，这两个数**不再有原始日志可复核**」。我核 `C1/logs/` 现存 7 个文件，**确实没有中间版** ⇒ **属如实披露**，不冒充「全程可复跑」。终态日志可复现：`final_check_anchors.log` 首行「新增红 15」、`final_check_doc_policy.log`「红线 3」，且两文件 **`C1.md` 命中 0 次**，与「本卡归因 0」自洽。

> 该条属**过程证据保全不足**（low），不影响本 acceptance：判据是「最新哈希 + 当前状态可复核」，两者均成立。

## 4 收口勾选

- [x] **AC-1 = passed**：独立 diff 证明变更面**只有 1 块 2 行删除**（R0 `C1-RKPT` 精确 `old_text`）；预处理 token 流前后同 hash ⇒ **无语义变化**；2 行是已不存在类型 `detect_result_t` 的旧签名 ⇒ **非知识遗失**；822 输入仅 8 变更、全有主 ⇒ **无越界**；未跑项明确声明、未拿 GREEN 冒充 ⇒ **无假绿**。
- [x] **AC-2 = passed**：在场 sha 与 R0 后像**逐字节相等**、B0 副本与前像**逐字节相等** ⇒ 恢复证据对应最新哈希且可回滚；独立重跑 11/11、C++ 26/0、wf GREEN、两守卫真退码均与 C1 自报一致；**未修发现 = 0**。

## 5 文档变更

| 动作 | 文档 | 说明 |
|---|---|---|
| 新增 | `docs/30_任务卡库/在办/trim-1029/RC1.md` | 本卡 |
| 新增（过程区） | `_tmp_session/TRIM-EXEC-20261029/RC1/` | `rc1_anchors.json`、`rc1_policy.log` |
| 无 | 其余 | **未改** C1 卡、未改源码/配置/文档、未跑 gen_index、未改 baseline |

## 6 两条 low 观察（不影响 verdict，不算待修）

1. **过程证据保全不足**（§3.7）：16/18 两版中间日志已删，只剩终态；作者已自曝。建议后续卡先落日志再清目录。
2. **命名耦合教训**（C1 自曝 E2）：在自己写域内取与他人卡面同基名的文件，会把**别人的卡**判红。建议 Lead 把「新文件取名先查全库同基名占用」写成规范；**不是 C1 的待修缺陷**（已改名并回落 15）。

> 我不把这两条当待修项：不改变 acceptance 任一判据，且第 1 条已如实披露、第 2 条已闭环。
