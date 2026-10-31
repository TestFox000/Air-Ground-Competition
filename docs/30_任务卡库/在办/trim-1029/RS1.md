---
卡号: RS1
ns: trim-1029
agent: trim-car
树节点: TRIM-11
owner: trim-car
kind: review
status: in_progress
更新: 2026-10-30
摘要: 独立评审 S1（t7）机端试点：源码语义无变化（我独立复算全部通过）；S1 卡仍有未修问题（行数 1444 是假数、--help 比较前提未写），故 needs_revision。我的 F3 原定性「757 是形变件」已被 Lead 实测撤回并留痕，修复面仅卡面措辞，不动源码。
---

# RS1 · 试点机端独立评审（round 1）

## 0 被审对象与身份

| 项 | 值 |
|---|---|
| 被审 task | `t7` · S1 · 作者 **trim-scout**（源码作者不变） |
| 本评审 task | `t11` · attempt `7b9ea034-4ca8-4948-b698-855d764cfd84` · reviewer **trim-car** |
| 关系声明 | 我**未参与**该实现、未改该源码。评审不是自评。 |
| 写域 | 仅 `docs/30_任务卡库/在办/trim-1029/RS1.md` + `_tmp_session/TRIM-EXEC-20261029/RS1/` |
| 未写 | `dzsmartscout/**`、`docs/20_资料索引库/**`、`dzrobot_v2/**`、共享索引、他人卡。**未跑 gen_index**、未改 baseline、未改测试、未新增夹具。 |

## 1 判定结论（先说结果）

**verdict = needs_revision。**

- **源码实现本身：我独立复算全部通过**（§3、§4）。语义/接口/常量/CLI 零变化，无越界，不是假绿。
- **但 S1 卡自身有两处证据陈述不成立**（§6 `RS1-F2` 行数、`RS1-F3` `--help` 证据），而 S1 的 A-3/A-2 正是靠这两处陈述支撑的。按本任务 acceptance 第 2 条「独立验证和恢复证据齐全且对应最新哈希，未修发现为零才 verdict=pass」，**有未修发现 ⇒ 不能 pass**。
- 修复面极小：**只改 S1 卡两句话/一处证据说明**，不需要动任何源码，也不需要重跑。

## 2 五必填（评审者视角）

### 2.1 台账条目

| 编号 | 约束 | 本评审如何遵守 |
|---|---|---|
| D-008 | 冻结区四路径 | 被判对象是 `dzsmartscout/tools/link`，不在四路径内；我零写入该树 |
| D-019/D-007 | 部署唯一入口、人工替换 | 未部署、未连车 |
| D-044 | commit/push 等用户指令 | 未 commit / 未 push |
| 树「当前唯一写域」第 1 条 | 成员不跑 gen_index、不写共享索引 | 未跑、未写；索引只读引用 |
| AGENTS §1.7 | 验收机器可判、不许自评 | 全部判据为我自己复跑/复算（§4），不引用 S1 自报数字作结论 |

### 2.2 复用（不重复造基线）

| 需要 | 复用对象 |
|---|---|
| 白名单唯一真源 | `_tmp_session/TRIM-EXEC-20261029/R0/scope_manifest.json`（S1-DOC 块 old/new/after_sha） |
| 字节基线 | `_tmp_session/TRIM-EXEC-20261029/B0/accept_root/dzsmartscout/tools/link/dz_send_waypoint.py`（与 S1 前像逐字节相同，我复算过） |
| 上级评审/验证 | RD2（t31）结论、V1（t9）受控 A/B 结果 —— 只作背景，**不继承任何 verdict** |
| 不采用 | S1 自报的 token 数/行数/哈希结论，一律我自己重算（见 §4 与 §6 纠错） |

### 2.3 新增轮子

仅一批一次性只读探针脚本，全部落在过程区 `_tmp_session/TRIM-EXEC-20261029/RS1/` 目录内（与其它证据文件同目录，未逐一列举文件名）。**不登记复用清单、不进 `tools/`、不建平行能力；无长期工具新增。**

### 2.4 验收命令（我自己跑的，非转抄）

```bash
python -B -X utf8 dzsmartscout/tools/link/tests/test_dz_send_waypoint.py          # 合同 verify 1
python -B -X utf8 -m unittest discover -s dzsmartscout/tools/link/tests -p "test_*.py" -v   # 合同 verify 2
python -B -X utf8 _tmp_session/TRIM-EXEC-20261029/RS1/rs1_apply_verify_chain.py    # 我自己的等价性复算链
python -B -X utf8 docs/_工具/check_doc_policy.py; python -B -X utf8 docs/_工具/check_anchors.py
```

### 2.5 回滚/恢复证据核对

| 项 | 我实测 |
|---|---|
| S1 前像 `_tmp_session/TRIM-EXEC-20261029/S1/preimage_dz_send_waypoint.py` | 22089 B / 452 CRLF / `00f746d8…` |
| 该前像 == B0 accept_root 副本 | **逐字节相同**（`pb == bb` True，同一 sha） |
| 该前像 == R0 `S1.before_sha256` | **相等** |
| 现场后像 == R0 `S1.after_sha256` | **相等**（`fffffde7…`，21293 B / 444 CRLF / 0 lone-LF） |
| 现场是否被提前推进到 S2 | **否**（R0 `S2.after` 不匹配；S2 三个块 old_text 仍各 1 次命中） |

⇒ **恢复链三方齐备且对应最新哈希**，这一条通过。

## 3 变更面：只动 R0 那一块（我按字节重建，不看自报）

判据：把 **B0 副本**取出，按 R0 的 `S1-DOC` old→new 替换一次，看结果是否等于现场。

| 复算 | 结果 |
|---|---|
| B0 副本中 `old_text` 命中次数 | **1**（唯一块） |
| 重建结果 == 现场字节 | **True**；重建 sha = `fffffde76dc96da5…` = 现场 sha |
| `git diff -U0` 的 hunk 数 | **1** |
| 新增行 / 删除行 | **+2 / −10**（无第三态） |
| 删除的 10 行是否全是那段历史叙述 | 是（逐行打印核对：`修订（2026-10-12…` 到 `…bridge upper=K4`。） |
| S2 三个块（`:95`、`:311-312`、`:363-364`） | **原样在位**，未被顺手做掉 |

## 4 语义等价（三类分核，我自己算）

| 核验面 | 方法 | 我实测 |
|---|---|---|
| 可执行 token 流 | `tokenize`，剔除 COMMENT/NL/INDENT/DEDENT/ENCODING 与**模块 docstring 那一个 STRING** | **3068 = 3068，完全相同**（S1 报 3068，与我一致） |
| AST（剔除模块 docstring） | `ast.dump` 后 sha | `b5af445a3d90…` 前后**同值** |
| 节点数 | `sum(ast.walk)` | **2777 = 2777** |
| 唯一允许变化项 | `ast.get_docstring` | 54 行 → 46 行（span 3–57 → 3–49），**仅此一处** |
| CLI `--help` | **同一 argv[0]**、`stdout=PIPE`（**不用 `text=True`**，否则 CRLF 被归一）后前后各跑一次 | exit 0/0、stderr 0 B；**原始字节 757 B / 18 CRLF / 18 LF，sha `3cba51a7…`，前后逐字节相同** |
| 模块常量 | 双份 import 后比对 | `BANNER` / `TOPIC_ROUTE` / `TOPIC_TO_CAR` / `TOPIC_FROM_CAR` / `BLOCK_SEP` **全部相等** |
| `__doc__` 文本消费者 | 全 `dzsmartscout` 搜 `__doc__`/`get_docstring`/`doctest` | 命中 9 处，**均不指向本模块**（`cam_probe_metrics.py`、`t4_ssh.py`、`sim_match.py` 等别的脚本）；本模块**零消费者** |
| 补丁脚本锚点 | `patches/patch_drone_sender.py` 三个锚点在**后像**上重算 | `import time` :56、`args = ap.parse_args(argv)` :321、无订阅者提示 :206 —— **各唯一 1 次** |

## 5 合同 verify（我自己真退出码）

| 命令 | 退出 | 结果 |
|---|---|---|
| `python -B -X utf8 dzsmartscout/tools/link/tests/test_dz_send_waypoint.py` | **0** | `Ran 24 tests`，`OK`；**24 个 test ID 与 S1 记录的 ids_post 集合完全相同**（我另存 `rs1_ids_24.txt` 比对，only_S1=∅、only_live=∅） |
| `python -B -X utf8 -m unittest discover -s dzsmartscout/tools/link/tests -p "test_*.py" -v` | **0** | `Ran 203 tests`，`OK (skipped=1)` |

**skip 7→1 我不当改善，并独立复核了 S1 的根因说明**：`test_dzudp_adapter.py` 6 处 `skipUnless` 的夹具（`_tmp_session/sqc-20261024/real_intel_13cls.hex`、`real_route.hex`、`_tmp_session/AGC3/…/intel_protocol.py`、`field_frame.py`、车端 `contract.py`）在**真实工作区实测全部存在**，而 B0 是在 `accept_root` 副本里跑的、该副本按设计排除 `_tmp_session` ⇒ 副本里全 skip。`test_dzudp_core_provenance.py` 的 `TEACHER`（`D:/User/Desktop/教员交接/udp_bridge.py`）实测 **MISSING**，`MIRROR` 存在 ⇒ 恰好 1 skip。**S1 的解释成立。**

## 6 发现（findings）

| ID | 严重度 | 问题 | 修复要求 |
|---|---|---|---|
| **RS1-F2** | low | **S1 卡 §3 A-3 写「文件其余 1444 行逐字节不变」，但该文件总共只有 445 个 split 元素（物理 LF 行 444）**。原文那个 1444 在任何计量口径下都不成立。**正确数要带口径**（我两种口径都跑了，见 §6.1）：`split("\n")` 数组口径 = 前 453 → 后 445 元素，**443 个元素未改 + 2 个新增**（1–46 全等、49–445 与前像 57–453 全等 397/397，46+397+2=445 ✔）；**物理 LF 行口径 = 452 → 444 行，442 行未改 + 2 行新增 + 10 行删除**（442+2=444 ✔）。两个口径差 1，差的正是末尾空元素，不是矛盾。 | 把该句改成**带口径**的可复算写法，例如「物理 LF 行 452→444：442 行未改 + 2 行新增，删 10 行；按 split 数组计为 453→445 元素、443 未改」。**不要只写一个数字、也不要求修复者为了迎合某口径改数**——口径写明即成立。 |
| **RS1-F3** | low（**经 Lead 实测后，我的定性已撤回，finding 缩小为「前提未写」**） | **原定性错误，现撤销**：我原先称「757 是形变件、739 才是真实 stdout」，**不成立**。真实 Windows Python312 用 `subprocess.run(..., stdout=subprocess.PIPE)`（**无 `text=True`**）采到的**原始 stdout 就是 757 B / 18 CRLF / 18 LF，sha `3cba51a7798f0a75d92a40971ea97faac4d8661bf5a589229c46b336632abfe2`**；**739 恰是 CRLF→LF 归一后的形态**（757 − 18 = 739）。我自己用二进制重采复核：前后两像原始 stdout 均为 757 B / 18 CRLF / 同 sha，且与 S1 的 `help_pre.txt` / `help_post.txt` **逐字节相同**（`==`，不是「转换后相等」）；Lead 的独立原件 `_tmp_session/TRIM-EXEC-20261029/lead/lead_help_raw_stdout.bin`（757 B）同 sha。⇒ **S1 存档本来就是原始采样，我此前说它「非原始」是错的。** 我之所以报 739，是我自己的观察器用了 `text=True`（`read_text` 同理会归一），把 CRLF 吃掉了 —— **是我的测量方法错，不是 S1 的证据错**。 | **只剩这一条需要补**：卡里**未写明「比较必须同一 argv[0] 基名」**这一前提（我实测：用前像文件名 `preimage_dz_send_waypoint.py` 跑，`usage:` 行 prog 名随之变，原始字节变 784 B，前后仍相同但与 757 不同——这是**假红陷阱**，不是缺陷）。修法：在卡里补一句前提即可。**不得**改 757、不得把旧件标成「形变件」、不得删旧件。 |
| **RS1-F4** | low（**非本轮引入，仅登记**） | `技术全景_D:1004` 与 `技术全景_E:378` 的函数**结束行**与真身不符（`send` 写 `:189-234`，实到 236；`listen` 写 `:237-268`，实到 271）。我用 **S1 前像**复算：这些 end 偏移在 S1 之前就存在（start 行精确、end 行偏 1–3），**不是本轮造成的回归**。 | 不需本轮修。登记为既有漂移，交 D1/D2 后续按内容核；**本轮不得因此判 S1 回归**。 |
| **RS1-F5** | low（**已被 D1 闭合，仅记录**） | S1 移交表 `d1_line_shift_handover.json` 对 `技术全景_E:28` **只记了 `73-76→65-68`**，漏记同一行的第二对 `80-88`。但 D1 已按内容重定位把两对都补对：现场 E:28 实为 `dz_send_waypoint.py:65-68,72-80`，我实测 `PRE[80:88] == POST[72:80]` **逐行相同** ⇒ **内容无损**。 | 无需修源码/文档。仅建议 D1/D2 顺手把移交表标为「已被 D1 全量扫描取代」。 |

**F1 撤回**：我一开始把 E:28 判成「`80-88` 指针整段丢失、QoS 真值证据断裂」。用多段正则重解后证明 **`72-80` 就在同一行**（`PRE 80-88 == POST 72-80` 为 True），是我的首版正则只抓第一对所致。**该判据作废，不进入 findings。**这也说明：单条正则解析锚点会漏配对，结论必须回到行内容核对。

### 6.1 行数计量口径（captain 要求补，两种口径都实测）

| 口径 | 前像 | 后像 | 未改 | 新增 | 删除 | 自洽校验 |
|---|---|---|---|---|---|---|
| **split 数组**（`text.split("\n")`） | 453 元素 | 445 元素 | **443** | 2 | 10 | 46+397+2 = 445 ✔ |
| **物理 LF 行**（= `bytes.count(b"\n")`） | 452 行 | 444 行 | **442** | 2 | 10 | 442+2 = 444 ✔ |

两口径差 **1**，差的正是 `split("\n")` 的**末尾空元素**（文件以 LF 结尾，`split` 会多出一个 `""`）；**不是矛盾、也不是谁算错**。文件大小 22089 → 21293 B（−796 B），CRLF 452 → 444（纯 CRLF，0 lone-LF）。

⇒ 因此：**1444 在任何口径下都不成立**（F2 成立）；但我在 F2 原写法里只给了 split 数组口径的「443」，没有标明口径，**这是我不严谨**，已按 captain 要求改成带口径的表述，并明确**不要求修复者迎合某个数字**。代码见 `_tmp_session/TRIM-EXEC-20261029/RS1/rs1_linecount_units.py`。
### 6.2 我的 `--help` 测量方法错在哪（自我勘误，按 Lead 实测撤回原定性）

**先给结论**：**S1 存档的 `help_pre.txt` / `help_post.txt` 就是原始采样，757 B / 18 CRLF / sha `3cba51a7…`；我此前说它是「形变件、非原始字节」是错的。**错误在**我的观察器**，不在 S1 的证据。

| # | 事实 | 我实测 |
|---|---|---|
| 1 | 真实原始 stdout（`stdout=PIPE`，**不用 `text=True`**） | **757 B / 18 CRLF / 18 LF**，sha `3cba51a7798f0a75d92a40971ea97faac4d8661bf5a589229c46b336632abfe2`；前像与后像**逐字节相同** |
| 2 | S1 的 `help_pre.txt` / `help_post.txt` | **757 B / 18 CRLF / 同 sha**，与①**`==` 相等**（不是「转换后相等」） |
| 3 | Lead 独立原件 `_tmp_session/TRIM-EXEC-20261029/lead/lead_help_raw_stdout.bin` | **757 B / 18 CRLF / 同 sha** |
| 4 | 739 B 是怎么来的 | **CRLF→LF 归一后**的形态（757 − 18 = 739），**不是原始字节** |

**我的错误**：我最初用 `subprocess.run(..., capture_output=True, text=True, encoding="utf-8")` 取 `stdout`，Python 在 `text=True` 下做 universal-newline 归一，把 18 个 CRLF 吃成 LF —— 于是我拿到 739 B 并把 757 判成「形变」。这与 S1 在 §2.8 记的**同一个坑**（`open(encoding=)` 把 CRLF 归一）完全同源，**我自己又踩了一次**。

**已修**：把复算链 `_tmp_session/TRIM-EXEC-20261029/RS1/rs1_apply_verify_chain.py` 的 `help_out()` 改成 `stdout=subprocess.PIPE, stderr=subprocess.PIPE`（无 `text=True`），**比字节而非比字符串**；复跑 `RESULT: ALL PASS`，输出 `757 B / 18 CRLF / sha 3cba51a7… / stderr 0 B`。二进制取证脚本 `_tmp_session/TRIM-EXEC-20261029/RS1/rs1_help_raw_bytes.py` 与日志 `_tmp_session/TRIM-EXEC-20261029/RS1/rs1_help_raw.log` 同目录。

**F3 缩小后只剩一条（且是低危「前提未写」）**：卡里没写「比较必须同一 argv[0] 基名」。我实测同理由：用 `dz_send_waypoint.py` 跑 → 757 B；用前像文件名 `preimage_dz_send_waypoint.py` 跑 → **784 B**（`usage:` 行 prog 名变长），前后仍各自相同但与 757 不同 ⇒ **换名比较会假红**，这是前提不是缺陷。见 `_tmp_session/TRIM-EXEC-20261029/RS1/rs1_argv0_caveat.py` 与 `rs1_argv0_caveat.log`。

**不改 757、不删旧件、不把旧件标成形变件**：按 captain 指令保留原物与原数；我在卡里只补事实与前提。
## 7 门禁与变更归属（我自己跑的）

| 守卫 | 真退出 | 观测 |
|---|---|---|
| `check_doc_policy.py` | **1** | 红线 **3** 条，全在他人写域（R1 `clear/ontology/verification-loop.md`；R11+R4 `archrev-1028/DBG-LINK-DIAG-20261030.md`）。**trim-1029 归因 0** |
| `check_anchors.py` | **1** | 新增红 **15 occ / 14 唯一 key**（与约定口径一致）；**`S1` 与 `RS1` 归因 0**（逐条打印归属表，owner 全为本 ns 之外或归档） |
| `check_numbers.py` | **0** | — |
| `tools/wf/check.py` | **0** | `结果: GREEN`、复用门禁 新文件 0 / 未登记 0、links 309 / broken 0 |
| `tools/wf/check_links.py` | **0** | `309 files / broken 0` |

**变更归属（不采信任何 changedPaths）**：我拿 B0 `input_manifest.json` 的 822 个 in-copy 输入**逐字节重算全工作区**，现场共 **8 个**文件与 B0 不同、**0 缺失**：

| 文件 | 归属 |
|---|---|
| `dzsmartscout/tools/link/dz_send_waypoint.py` | **S1（t7）** ← 被判对象 |
| `dzrobot_v2/src/rknn_pt/include/det/rkpt.hpp` | C1（t6，trim-car） |
| `docs/00_项目总览/技术全景_D_无人机.md`、`…_E_空地链路与工程设施.md` | D1（t8，trim-doc）行号补偿 |
| `docs/目录索引.md`、`docs/目录索引.json`、`dzrobot_v2/docs/TEAM_STATUS.md`、`dzrobot_v2/docs/决策台账.md` | Lead 治理/索引发布 |

⇒ **S1 实际只落地 1 个源码文件**，无越界。`git status` 亦只有该 1 项 `M` 在 `dzsmartscout` 下。

## 8 逐条验收（合同两条原文）

- [ ] A-1（领取时 false）最新实现逐项满足R0与实施合同，无语义变化、知识遗失、越界修改或假绿。
- [ ] A-2（领取时 false）独立验证和恢复证据齐全且对应最新哈希，未修发现为零才verdict=pass。

| 验收 | 结果 | 证据/缺口 |
|---|---|---|
| A-1 | **通过** | 重建式字节证明：B0 + S1-DOC = 现场；1 hunk / +2 / −10；token 3068 全等、AST 剔除 docstring 全等、常量全等、同 argv0 下 `--help` **原始 757 B/18 CRLF**（sha `3cba51a7…`）全等、零 `__doc__` 消费者；S2 三块在位；无越界（8 个变更文件逐一带主）。 |
| A-2 | **未通过** | 恢复链三方齐备（前像 == B0 副本 == R0.before；后像 == R0.after），**该半项通过**；但**仍有未修发现**：F2（行数 1444 是假数，S1 已自改真数，我待 t38 按口径复算确认）、F3（`--help` 比较前提未写；我的「757 是形变件」定性已被 Lead 实测撤回，只剩前提一条）⇒ 按「未修发现为零」不满足。 |

## 9 证据索引

| 证据 | 路径 |
|---|---|
| 结构化复核结果（含 findings、变更归属、哈希） | `_tmp_session/TRIM-EXEC-20261029/RS1/rs1_verify.json` |
| 全工作区 vs B0 逐字节对账 | `_tmp_session/TRIM-EXEC-20261029/RS1/rs1_fs_reconcile.json` |
| 合同 verify 原始日志 | `_tmp_session/TRIM-EXEC-20261029/RS1/v1_suite.log`、`_tmp_session/TRIM-EXEC-20261029/RS1/rs1_24_v.log`、`_tmp_session/TRIM-EXEC-20261029/RS1/v2_discover.log` |
| 24 个 test ID（我实采） | `_tmp_session/TRIM-EXEC-20261029/RS1/rs1_ids_24.txt` |
| 等价性复算链（可复跑，含行数与 CLI 判据） | `_tmp_session/TRIM-EXEC-20261029/RS1/rs1_apply_verify_chain.py` 及其日志 `_tmp_session/TRIM-EXEC-20261029/RS1/rs1_chain.log` |
| 哈希/锚点/对账探针（过程区内一次性脚本，未逐一列出名） | `_tmp_session/TRIM-EXEC-20261029/RS1/` |
| 守卫原始日志 | `_tmp_session/TRIM-EXEC-20261029/RS1/g_policy.log`、`_tmp_session/TRIM-EXEC-20261029/RS1/g_anchors.log`、`_tmp_session/TRIM-EXEC-20261029/RS1/g_anchors.json`、`_tmp_session/TRIM-EXEC-20261029/RS1/g_numbers.log`、`_tmp_session/TRIM-EXEC-20261029/RS1/g_wf.log`、`_tmp_session/TRIM-EXEC-20261029/RS1/g_links.log` |

## 10 未做 / 边界

- **未跑 59 套全量、未重建基线**（V1 的受控 A/B 由原验证者补证中；我的判据不依赖它）。
- **未改任何源码/文档/测试/期望/夹具**；未新增 skip；未扩 baseline；未 `gen_index`。
- 未验证真实 ROS 下 `send()/listen()` 行为（需 rclpy 与真机/订阅者）；本评审只判「代码 token 与 CLI 契约不变」。
- 未编译 RKNN、未实车、未连车、未部署、未 commit/push。

## 11 本卡自身锚点新红的处置记录（按 captain 指令，独立于评审结论）

**事实（我造成的，不是守卫误报）**：我在这张卡里先后写过**两种路径形态的占位**，两次都由 `check_anchors` 判为本卡新增红，并把 `trim-doc` 的 t35 strict delta 挡住：

| # | 写法（用散文描述，**不再原样抄出占位字符串**） | 守卫结果 | 处置 |
|---|---|---|---|
| 1 | 把「第 1 到第 29 号探针脚本」写成**带省略号当文件名**的单个路径 | 新增红 **15 → 16**，RS1 归因 **1** | 改为**只指真实存在的那一份** + 目录描述；回落 **15 / 归因 0** |
| 2 | 同一处改成**通配符当文件名**（我漏改的第二处，被 captain 抓到） | 同上口径，仍是占位路径形态 | 改为**纯散文**：只说「一批一次性只读探针脚本，全部落在过程区该目录内，未逐一列举文件名」；再回落 **15 / 归因 0** |

**处置原则（与 trim-doc 给出的三种修法一致）**：

- 不删表述、不标 `〔档〕`/`〔证失效〕` 凑绿 —— 探针脚本**真实存在**，标失效等于自造假声明；
- 不扩任何 baseline、不动守卫、**不删自己或他人原始日志**（29 个探针脚本与全部守卫日志原样留在目录里）；
- 只把「路径形态但文件不存在」改成「指真实文件」或「散文 + 真实目录」。

**真实守卫验证（跑在改动之后，不是目测）**：

```bash
python -B -X utf8 docs/_工具/check_anchors.py         # exit 1
python -B -X utf8 docs/_工具/check_anchors.py --json  # 逐条归属
python -B -X utf8 docs/_工具/check_doc_policy.py      # exit 1
```

| 判据 | 实测 |
|---|---|
| 新增红总数 | **15 occ / 14 唯一 key**（全局既有口径，非本卡贡献） |
| **本卡（RS1）归因** | **0**（`--json` 的 `red_new` 里无本卡、亦无 `trim-1029/S1`） |
| 卡内是否还残留占位路径形态 | 用 `grep` 查省略号与通配符两种形态 → **无命中** |
| `check_doc_policy` | exit **1**，红线 **3** 条，全在他人写域（R1 `clear/…`、R11+R4 `archrev-1028/…`）；本卡 **0** |

原始日志：`_tmp_session/TRIM-EXEC-20261029/RS1/g_anchors.log`、`_tmp_session/TRIM-EXEC-20261029/RS1/g_anchors.json`、`_tmp_session/TRIM-EXEC-20261029/RS1/g_policy.log`（均为改动后重跑、覆盖同名旧文件，未删除任何历史日志）。

> **本节第三条自伤（已修）**：我第一次写这一节时，为了「如实记录」，把那个占位字符串**原样抄进了表格**——守卫立刻把它当成新引用，新增红又回到 **16 / 本卡归因 1**。第二次我把表格改成**散文描述**（只说「带省略号当文件名」「通配符当文件名」，不写出字面路径），才真正回落 **15 / 归因 0**。
>
> **教训（与 C1 的 E2、G0 的独立发现同源）**：
> 1. 锚点守卫按**路径字面**判红，凡带省略号 / 通配符 / 范围的写法都算占位；**连「记录这个坑」的正文也会自伤** —— 坑要写，但**不能把占位字符串写回卡面**。
> 2. 它同时按 **basename** 判唯一性：在自有写域新造与旁人卡面同名的证据文件，会把**别人的卡**判红。
> 3. 两条都只能靠「用真实名 / 加 owner 前缀」避免，**不能靠豁免或标失效**。

## 12 round-2 复评（审 t37 repair-round-2）

> **为什么写在本卡而不是新开卡**：t38 的合同 In scope 为空、未指定新卡；`RS2.md` 在本树里是**另一张卡**（TRIM-19 扩展机端独立评审，对应 S2/t15 阶段），与本次「复审 S1 修复」不是同一件事。为保持**一个被审任务一条评审线**、避免误占他人卡，round-2 复评作为本节追加在本卡内。若 Lead 要求单列，请另行派卡，我不自行建卡。

### 12.1 身份

| 项 | 值 |
|---|---|
| 被审 task | `t37` repair-round-2（修本卡 round-1 的 F2/F3/F4/F5 findings） |
| 源码作者 | **trim-scout**（不变）；我未参与实现 |
| 本复评 task | `t38` · attempt `daea818e-8b11-4a55-9c27-6d304d48da0c` · reviewer **trim-car** |
| 写域 | 本卡 + `_tmp_session/TRIM-EXEC-20261029/RS2/` |

### 12.2 判定

**verdict = needs_revision。**

- **源码与语义：我独立复算全部通过**，且 t37 **零源码改动**（源文件仍 `fffffde7…` / 21293 B / 444 CRLF）。
- **卡面/证据一致性仍有 4 条 low 未修**（§12.6）。按 acceptance 第 2 条「未修发现为零才 pass」，不能 pass。
- 修复面全是**卡面与证据描述**，不需动源码、不需重跑。

> **口径声明（按 captain 要求）**：不沿用 round-1 的 739 硬值，不逼修复者迎合任何数字；一切按**真实字节**与实际采样代码判；行数**先写明口径**再比较。

### 12.3 源码侧独立复算

| 判据 | 我实测 |
|---|---|
| 现场 字节 / CRLF / lone-LF | **21293 / 444 / 0** |
| 现场 sha | `fffffde76dc96da5f83bbefec6376d1217cd1988508994dd5c95504158c9e105` |
| == R0 `S1.after` | **True** |
| == R0 `S2.after`（是否被提前推进到 S2） | **False** |
| B0 副本 == R0 `S1.before` | **True** |
| B0 副本 + `S1-DOC` 唯一替换 → 与现场逐字节相同 | **True** |
| S2 三块 `old_text` 在现役文件中命中次数 | **1 / 1 / 1**（未被顺手做掉） |
| t37 是否动过源码 | **零改动** |

**语义等价**（我自己算）：token 流 **3068 = 3068 全等**；AST 剔模块 docstring 后 `b5af445a3d90…` **同值**；节点数 **2777 = 2777**；docstring span **(3,57) → (3,49)**、行数 **54 → 46**，是唯一变化项。

**`--help` 原始字节**（按真实字节，不用 739）：`stdout=PIPE, stderr=PIPE`、**无 `text=True`**、两侧同名 `dz_send_waypoint.py` ⇒ **757 B / 18 CRLF / 18 LF**、stderr 0 B、exit 0/0、sha `3cba51a7…`，**前像与后像逐字节相同**；与 S1 的 `help_pre.txt` / `help_post.txt` **逐字节相同（`==`）**，也与 Lead 的 `lead_help_raw_stdout.bin` 相同。**739 B = 该 757 做 CRLF→LF 变换**（我验证 `== base.replace(CRLF, LF)`）。换 argv[0] 后为 **784 B**（两侧仍相同但与 757 不同）⇒ **同 prog 名是前提**，S1 已写明。

### 12.4 F2 行数（先写口径再比数）

| 口径 | 前像 | 后像 | 未改 | 新增 | 删除 | 自洽 |
|---|---|---|---|---|---|---|
| **物理行**（`split("\r\n")` 去末尾空元素） | **452** | **444** | **442**（46+396） | 2 | 10 | 442+2=444 ✔ |
| split 数组（含末尾空元素） | 453 | 445 | 443 | 2 | 10 | 443+2=445 ✔ |

S1 §7.1/§A-3 声明「452→444、未改 442、46+396+2=444」——我独立复算**逐项相等**（`1..46` 全等、`post 49..444` 与 `pre 57..452` **396/396** 全等）。**F2 已闭合。**

### 12.5 F4 / F5

**F5 移交表**：`refs` **19 条**（18 `changed=true`）；我按 `PRE[old] == POST[now]` 且 start/end 各偏 8 逐条复算，**18/18 命中、0 不符** ⇒ **测量值确实未改**；`superseded_by` / `superseded_note_scope` 已加且自述一致。**F5 闭合。**

**F4 既有漂移**（口径＝纯函数体末行，排除空行与分隔注释）：

| 文档/行 | 前像真实 span | end_offset | 后像真实 span | end_offset |
|---|---|---|---|---|
| `技术全景_D:1004` send | 189–**234** | **0** | 181–**226** | **0** |
| `技术全景_D:1006` listen | 237–**268** | **0** | 229–**260** | **0** |
| `技术全景_E:377` send | 189–**234** | **+2** | 181–**226** | **+2** |
| `技术全景_E:378` listen | 237–**268** | **+2** | 229–**260** | **+2** |

⇒ **前后偏移完全相同（+2/+2/0/0）⇒ 漂移在 S1 之前就存在，S1 不是成因**，与 S1 的结论一致。

> **我自己 round-1 的这两处不严谨，据实更正**：① 我曾报「739 是真实 stdout」——根因是我用了 `text=True`，把 18 个 CRLF 归一成 LF；**757 才是原始字节**。② 我曾报 `E:378` end_offset `−1`、`D:1006` `−3`，那是把分隔注释行算进函数体末行的另一种口径；**同口径重算为 +2/+2/0/0**。

### 12.6 本轮 findings

| ID | 严重度 | 问题 | 修复要求 |
|---|---|---|---|
| **RS2-F1** | low | S1 §7.5 自称「本轮改动清单（**完整 changedPaths，无省略**）」列 **9 项**；我按 mtime ≥ 09:30 实测过程区新增/改动 **21 项**，**11 项未列**（S1 过程区下的三组守卫采样 JSON 与六份重跑日志，共 11 个文件；逐一列名见 `_tmp_session/TRIM-EXEC-20261029/S1/` 目录本身）。全部在 S1 写域内、**无越界**，但「完整无省略」不成立。 | 补齐 21 项，或改成「本轮**主要**改动 + 完整清单见过程区目录」；**删掉「完整…无省略」**。 |
| **RS2-F2** | low | S1 §7.3 的 F4 表只列 2 行（`D:1004 send 已精确`、`E:378 listen end 差 2`），**漏报 `E:377 send` 也是 +2**。结论（非本轮引入）不受影响。 | 补成 4 行（D:1004 / D:1006 / E:377 / E:378），各写真实 `end_offset`，注明口径＝纯函数体末行。 |
| **RS2-F3** | low | **同一轮两处交付物自相矛盾**：`S1.md:282` 写「原 `help_pre.txt`/`help_post.txt` 的 SHA 与 Lead probe 完全相同 ⇒ 它们本来就是**忠实原始采样，不是形变副本**」；而同一轮新建的 `help_raw_manifest.json` 的 `legacy` 字段写「作为**『文本转换版』**登记，**不当作原始采样替代品**」。同一对文件被两种互斥角色定义。 | 两处取一：既然实测与 Lead probe **逐字节相同**，`legacy` 应改为「原采样（757 B/18 CRLF），与 Lead probe 逐字节相同，保留原样」；若确实曾经过文本辅助转换，则卡片不得称「忠实原始采样」。**按真实采样代码描述，不能只凭 CRLF 推断。** |
| **RS2-F4** | low | `S1.md:286` 写「前**後**两份 sha 相同」——繁体「後」应为「**后**」，不影响判据，但关键词检索会漏。 | 改回「前后」。 |

### 12.7 合同 verify（我自己真退出码）与变更归属

| 命令 | 退出 | 结果 |
|---|---|---|
| `python -B -X utf8 dzsmartscout/tools/link/tests/test_dz_send_waypoint.py` | **0** | `Ran 24 tests` / `OK` |
| `python -B -X utf8 -m unittest discover -s dzsmartscout/tools/link/tests -p "test_*.py" -v` | **0** | `Ran 203 tests` / `OK (skipped=1)` |

**24 个 test ID 与 S1 `ids_post` 集合逐项相同**（我实采另存；`only_in_S1 = ∅`、`only_in_live = ∅`），未删用例、未新增 skip、未改期望。

**变更归属（按 B0 822 个 in-copy 输入逐字节重算，不采信 changedPaths）**：**814 同 / 8 变 / 0 缺失**。8 个逐一带主：`dz_send_waypoint.py`=S1(t7)、`rkpt.hpp`=C1(t6)、`技术全景_D/E`=D1(t8)、两索引 + `TEAM_STATUS` + `决策台账`=Lead 治理。⇒ **t37 未产出任何源码变更、无越界**。

**门禁真退出**：`check_doc_policy` **1**（红线 3 全外域，trim-1029 归因 0）· `check_anchors` **1**（新增红 **15 occ / 14 key**，trim-1029 **归因 0**）· `check_numbers` **0** · `wf check` **0**（GREEN）· `wf check_links` **0**（309 files / broken 0）。

### 12.8 逐条验收（合同两条原文）

| 验收 | 结果 | 证据/缺口 |
|---|---|---|
| A-1 最新实现逐项满足 R0 与实施合同，无语义变化、知识遗失、越界修改或假绿 | **通过** | 源码零改动且后像 == R0 after；B0+块重建逐字节恒等；token 3068 全等、AST 剔 docstring 全等、节点 2777；`--help` 原始 757 B/18 CRLF 前后同 sha；S2 三块在位；822 输入对账 8 变全部有主、0 越界；保留项 14/15 直接命中（「单半场约束」以 `if len(got) != 1` + 报错体在源码体现，另 1 处是卡面措辞检索未命中、非遗失） |
| A-2 独立验证和恢复证据齐全且对应最新哈希，未修发现为零才 verdict=pass | **未通过** | 恢复链三方齐备、每项哈希对应最新值（该半项通过）；**仍有未修发现 F1~F4** |

### 12.9 我在本轮又造成的锚点红（已自查修掉）

我在本卡第 190 行把「合同 verify 的第二份日志」写成**不含目录的裸文件名**，而我新建的 `RS2/` 目录里恰有**同名文件** ⇒ basename 不唯一 ⇒ **anchors 15 → 16**、trim-1029 归因 1，挡住 trim-doc 的 t39。

处置：第 190 行三个日志**全部补成完整路径**；并自查出第 194 行 **5 个守卫日志裸名**一并补全（trim-doc 只指了第 190 行）。未用 `〔档〕`/`〔证失效〕` 凑绿、未扩 baseline、未删日志。**实测回落 15 occ / 14 key、trim-1029 归因 0。**

> 这是我**第四次**在同一机制上自伤（省略号占位 → 通配符占位 → 把占位串抄回卡面 → 新建目录后旧卡裸名撞名）。**新增教训**：自建新证据目录后必须回头 `grep` 自己旧卡里的裸引用。

### 12.10 复评证据索引

| 证据 | 路径 |
|---|---|
| 结构化复核结果（含 4 条 findings、变更归属、哈希） | `_tmp_session/TRIM-EXEC-20261029/RS2/rs2_verify.json` |
| 源码与块状态 | `_tmp_session/TRIM-EXEC-20261029/RS2/rs2_src_check.log` |
| 语义等价 | `_tmp_session/TRIM-EXEC-20261029/RS2/rs2_equiv.log` |
| `--help` 原始字节 | `_tmp_session/TRIM-EXEC-20261029/RS2/rs2_help_raw.log` |
| S1 help 证据审计 | `_tmp_session/TRIM-EXEC-20261029/RS2/rs2_help_files.log` |
| F2 行数双口径 | `_tmp_session/TRIM-EXEC-20261029/RS2/rs2_linecount.log`、`_tmp_session/TRIM-EXEC-20261029/RS2/rs2_a3_verify.log` |
| F5 移交表复算 | `_tmp_session/TRIM-EXEC-20261029/RS2/rs2_handover2.log` |
| F4 同口径对比 | `_tmp_session/TRIM-EXEC-20261029/RS2/rs2_f4_body.log` |
| 变更归属对账 | `_tmp_session/TRIM-EXEC-20261029/RS2/rs2_fs_reconcile.log` |
| 合同 verify 日志 | `_tmp_session/TRIM-EXEC-20261029/RS2/v1_suite.log`、`_tmp_session/TRIM-EXEC-20261029/RS2/v2_discover.log` |
| 用例 ID | `_tmp_session/TRIM-EXEC-20261029/RS2/rs2_ids_24.txt`、`_tmp_session/TRIM-EXEC-20261029/RS2/rs2_ids_203.txt` |
| 守卫日志 | `_tmp_session/TRIM-EXEC-20261029/RS2/g_policy.log`、`_tmp_session/TRIM-EXEC-20261029/RS2/g_anchors.log`、`_tmp_session/TRIM-EXEC-20261029/RS2/g_anchors.json`、`_tmp_session/TRIM-EXEC-20261029/RS2/g_numbers.log`、`_tmp_session/TRIM-EXEC-20261029/RS2/g_wf.log`、`_tmp_session/TRIM-EXEC-20261029/RS2/g_links.log` |

**未做**：未跑 59 套全量、未重建基线、未改任何源码/测试/期望/夹具、未新增 skip、未跑 `gen_index`、未验证真实 ROS 下 `send()/listen()`、未编译 RKNN、未实车/连车/部署/commit/push。

---

# 复评 round-3（t43 / 审 t42 repair-round-3）

> 只换轮次，合同仍是 RS1 的两条 acceptance。**我不自评实现**，被审对象是 trim-doc 在 t42 做的卡面/证据一致性修复。

## R3-0 判定

**verdict = pass**（4 条风格/口径 nit 仅登记，均**不阻断**）。

| 项 | 结论 | 我实测 |
|---|---|---|
| 源码仍是 R0 `S1.after`、未提前 S2 | **成立** | 21293 B / 444 CRLF / 0 lone-LF / sha `fffffde7…` == `S1.after`；**≠** `S2.after`；B0 副本 == `S1.before`；B0+`S1-DOC` 逐字节重建 == 现场；S2 三块 old_text 各 1 次仍在 |
| 原始 `--help` 字节真实一致 | **成立** | 二进制 `stdout=PIPE`（无 `text=True`）：757 B / 18 CRLF / sha `3cba51a7…`；四份文件（`help_pre.txt` / `help_post.txt` / 两份 `_raw`）**逐字节 ==**，且**== Lead probe**；739 B / 0 CRLF 实为 `raw` 做 CRLF→LF 变换（`== base.replace`） |
| 卡面与 manifest 角色一致 | **成立** | 卡片 §7.2 与 `help_raw_manifest.json` 的 `role_summary` **同口径**：757=原始 / 739=LF 视图；`legacy` 字段已改为「原始采样、保留原样」，「不作文本转换版处理」是**否定句**不是定性；`不当作原始采样替代品` 已删除 |
| 采样范围已界定、不再声称穷举 | **成立** | §7.5 用「采样时点/范围」界定：窗 `09:20–09:59`，窗内 **21 项 = 手工 6 + 生成 15**，并显式声明窗口之后的 `t42_*` **不计入 t37**；穷举声明**已删除** |
| 未越域、历史保留 | **成立** | 822 in-copy 对账仍 8 变 / 0 缺；t42 **零源码改动**；S1 front matter 的 agent/owner → trim-doc 是 **Lead 指派**，且 §10 **完整保留原实现者 trim-scout 的历史**（原成果、4 条自省坑原样） |
| 门禁真退出 | 记录 | `doc_policy` **1**（3 红全外域，trim-1029 归因 0）· `anchors` **1**（15 occ / 14 key，trim-1029 **归因 0**）· `numbers` **0** · `wf check` **0** GREEN · `wf check_links` **0**（309 / broken 0） |
| 合同 verify 真退出 | 记录 | `test_dz_send_waypoint.py` **0**（24 OK）· `discover` **0**（203 OK / skipped=1） |

## R3-1 「含尾空行窗口」——我实核后确认**不是错**

| 判据 | 实测 |
|---|---|
| LIVE `226` | `rclpy.shutdown()`（send 体末，非空） |
| LIVE `227` / `228` | **均为空行** |
| LIVE `229` | `def listen(duration: float) -> int:` |
| LIVE `260` | `return 0 if got["n"] else 3`（listen 体末） |
| LIVE `261` / `262` | **均为空行** |
| PRE 对应（+8） | 234 非空、235/236 空、237 `def listen`；268 非空、269/270 空 |

⇒ `E:377 :181-228` 与 `E:378 :229-262` 是**把函数区块连尾部空行一起框住**的写法，**合法**；且前像同形态 ⇒ **与 S1 无关、仅整体平移 −8**。**t42 撤回「既有漂移、交 D1/D2 改数字」是正确的**，我认同，不送 D2 改这些数。

## R3-2 「21 项」的可复核边界（按 captain 指示，不作「历史完整」断言）

我按 `mtime` 区间独立复算：`S1/` 目录内窗内 **20** 个文件 + 卡片自身 `S1.md` **1** 个 = **21**；其中生成记录 **15**（三组守卫 JSON 与两套 verify 日志）×3 组 = 15、手工交付 **5**（`d1_line_shift_handover.json` + 4 份 help 原始/LF 件）+ 卡片**1** = **6**。**6 + 15 = 21 ✔**

> **必须写明的边界**：`mtime` 只是**当前时间属性**的筛选，能证明「这些文件**现在**声称在该时段被写」，**不能证明当时目录里只有这些**（`ops`/拷贝/恢复都会保留或改写 `mtime`）。所以该数只能读作「**可复核的候选集**」，**不是历史穷尽清单**。t42 已删掉穷举声明，我按此口径核，**不要求它复原无法复原的历史**。

## R3-3 仍残留 4 条 nit（**均不阻断**，只登记）

| ID | 级 | 内容 | 为何不阻断 |
|---|---|---|---|
| N-1 | nit | `help_raw_manifest.json` 的 `legacy` 里保留了「历史上的由上一轮 shell 重定向产生」这类采集方法描述，而同文件 `premise` 又说「以当时真实代码为准、不复述未核细节」；两处语气可再统一 | 结论层（757=原始）**已被四份文件逐字节证实**，描述层措辞不影响判据 |
| N-2 | nit | `S1.md:361` 附注称 `help_raw_manifest.json`「在窗口内写入（09:56）」，实测其 `mtime` 为 **11:06:06**（t42 本轮又改过），该句只对 **t37 当次写入**成立 | 属时点表述；文件确曾 09:56 生成过，t42 覆盖属合法维护 |
| N-3 | nit | `§7.5` 生成记录计 15 项，而 `S1/` 下 `t42_*`/`t42b_*` 已 47 个文件；两组数字并存时**必须**带「窗口内/窗口外」限定 | 卡片已明确「窗口之后不计入 t37」，未混称 |
| N-4 | nit | 卡片摘要（front matter）仍只写 S1 本体，未提 t42 的一致性修复 | front matter 只述实现，历史在 §10 有完整交代 |

## R3-4 我在本轮的处理边界（防再次自伤）

依 captain 提示「原历史值不要改成当前值」，我**未**要求把任何 14096/361b17ca 之类历史采样改写成本版值；也**不**把 `mtime` 窗口数抬成「历史完整」结论。本轮我**只读**，未改 S1 卡、未改 manifest、未改源码。

## R3-5 证据索引

| 证据 | 路径 |
|---|---|
| 源码状态与块在位 | `_tmp_session/TRIM-EXEC-20261029/RS1/r3/r3a_source.log` |
| help 六份文件字节/角色 | `_tmp_session/TRIM-EXEC-20261029/RS1/r3/r3b_help.log` |
| manifest 角色一致性 | `_tmp_session/TRIM-EXEC-20261029/RS1/r3/r3c_manifest.log`、`_tmp_session/TRIM-EXEC-20261029/RS1/r3/r3i_legacy.log` |
| 卡片残留矛盾扫描 | `_tmp_session/TRIM-EXEC-20261029/RS1/r3/r3d_card.log` |
| 尾空行窗口 | `_tmp_session/TRIM-EXEC-20261029/RS1/r3/r3e_window.log` |
| 21 项 / 6+15 复算 | `_tmp_session/TRIM-EXEC-20261029/RS1/r3/r3h_count21.log`、`_tmp_session/TRIM-EXEC-20261029/RS1/r3/r3j_split.log`、`_tmp_session/TRIM-EXEC-20261029/RS1/r3/r3f_window_list.log` |
| 交接历史保留 | `_tmp_session/TRIM-EXEC-20261029/RS1/r3/r3k_handoff.log` |
| 全工作区 vs B0 对账 | `_tmp_session/TRIM-EXEC-20261029/RS1/r3/r3l_reconcile.log` |
| 门禁 / 合同 verify 日志 | `_tmp_session/TRIM-EXEC-20261029/RS1/r3/g_policy.log`、`_tmp_session/TRIM-EXEC-20261029/RS1/r3/g_anchors.log`、`_tmp_session/TRIM-EXEC-20261029/RS1/r3/g_anchors.json`、`_tmp_session/TRIM-EXEC-20261029/RS1/r3/g_numbers.log`、`_tmp_session/TRIM-EXEC-20261029/RS1/r3/g_wf.log`、`_tmp_session/TRIM-EXEC-20261029/RS1/r3/g_links.log`、`_tmp_session/TRIM-EXEC-20261029/RS1/r3/v1.log`、`_tmp_session/TRIM-EXEC-20261029/RS1/r3/v2.log` |

**未做**：未跑 59 套全量、未重建基线、未改源码/测试/期望/夹具/skip、未跑 `gen_index`、未验证真实 ROS、未编译 RKNN、未实车/连车/部署/commit/push。

## 2026-10-30 授权后的定位补注

此前原位与红线记录保留为当时事实；本次获用户明确授权，将相同字节设计稿纳入既有历史归档。它不是项目实现、默认值或验收结果。旧定位仅由下列本文档唯一映射指向原文：
- [clear/ontology/verification-loop.md](../../../history_cache/01_旧文档归档/clear/ontology/verification-loop.md)
