---
卡号: B0R
ns: trim-1029
agent: trim-verify
树节点: TRIM-23
owner: trim-verify
reviewer: trim-review
kind: verification
status: in_progress
创建: 2026-10-30 00:02
更新: 2026-10-30 01:05
摘要: 复核 B0 基线计数与失败集合、纠正汇总口径歧义、查清全局门禁红归属；不改源码/测试/守卫/基线，不重跑全量 59 套。
---

# B0R · 复核基线计数、逐失败对照与门禁归属

## 一句话目标

B0 采到了真实基线，但它的**汇总数字对不上自己的明细**。本卡不改 B0 的结论、不宣称既有红转绿，只做三件事：

1. 把 19 套非零退出**逐套**映射到「在册 / 首次观测 / 未解释」，让汇总等于明细；
2. 把**混合单位的 3949** 拆成五种互不可加的计数，并把 145/83/93/3866 的口径差算清；
3. 把全局门禁红**逐文件**归到 owner / ns / 并发状态，给出不改他人写域的最小修复建议。

给 R0 一份可直接独立核对的输入，且新口径**不扩宽失败集合**。

## 1 读过的台账条目

| 编号 | 它约束了什么 | 本卡如何遵守 |
|---|---|---|
| D-008 | 冻结区四路径不许直接改默认值 | 本卡**零源码零配置改动**；只读日志与文档 |
| D-023 / D-012 | 回归基线口径（37 套/13 红）与 `planner.engine=tour` | 作为「在册」判据来源之一，不改它 |
| D-025 | 本地通过与发布分开记录 | 本卡结论仍只是**本地离线**口径 |
| D-032/D-039 | 决策薄索引口径 | 走薄索引，不新开 K 页 |
| D-044 | 本地 commit 与推送等用户指令 | 不 commit / 不 push / 不部署 / 不连车 |
| K10/K11/K12/K13/L5/C类/D类CRASH | 已挂账红不重开 | 逐套比对；未在册者只标「首次观测」，不新增 K 编号 |
| 树 §“B0 后续裁决”第 2/3/4/5 条 | B0R 须逐行纠正 + 追加勘误，R0 独立核对后才放行；在册用本 ns 发现编号不新增 K 编号；门禁红须列逐文件归属但不越域处置 | 本卡全部照此执行 |

## 2 复用检索结果

| 需要 | 复用对象 | 结论 |
|---|---|---|
| 逐套原始日志 | B0 已落盘的 `logs/car__*.log`、`logs/scout_*.log`、`logs/cpp__*.log` | 直接解析，**不重跑**（除 4 个 <1.5 s 抽样复核） |
| 逐套 CSV | `_tmp_session/TRIM-EXEC-20261029/B0/logs/` 下的 suite_table.csv / suite_table_scout.csv（runner 原生输出）、suite_table_v2.csv（12_parse_logs 重解析） | 三者并存；**exit 只认前两者**，v2 的负数是解析失败不是结果 |
| harness 计数语义 | `dzrobot_v2/src/dzbrain/tests/harness.py` | `check()` 一次 = 一个断言；`finish()` **把每条失败再打印一遍** ⇒ 日志行数 ≈ 2×失败数 |
| unittest 计数语义 | Python `unittest` 默认输出 | `Ran N` **含 skipped**；pass = Ran − failures − errors − skipped |
| 失败明细 | `B0/evidence/fail_cases.json`（19_fail_cases.py 产出） | **行级去重**，不是断言级；且漏掉 perception 的 import 失败 |
| 已知红台账 | `docs/20_资料索引库/运维与排障/08_回归守护_known_reds.md`（K1–K14）+ `车端/06_回归与测试基线.md`（13 套归因）+ `赛事与治理/05_已知红与回归口径.md` | 逐套比对在册/未在册 |
| 守卫 | `check_doc_policy.py` / `check_anchors.py` / `check_numbers.py` / `wf/check*.py` | 只在**原工作区**跑；不改守卫、不改 baseline |

## 3 新增轮子及理由

| 路径 | 为什么现有不够 | 是否登记 |
|---|---|---|
| `_tmp_session/TRIM-EXEC-20261029/B0R/scripts/01..07_*.py`（7 个） | B0 的 `12_parse_logs.py` / `19_fail_cases.py` / `20_summary.py` 会把五种单位混加、把 `Ran` 当 pass、把行当断言——本卡要在**不改 B0 原脚本、不覆盖原输出**的前提下重算，只能另起只读脚本。全部落过程区，不属 `tools/`，不登记复用清单 | 过程区，非长期轮子 |

**结论**：不新增任何长期工具；7 个脚本均为一次性只读重算，输出全部写在 `B0R/evidence/`，B0 目录零写入。

> 与 B0 的 47 个脚本相比刻意收敛：本卡**只补 B0 缺的那一层重算**，不重复采集。

## 4 写域（一文件一 owner）

| 文件/目录 | owner | 说明 |
|---|---|---|
| `docs/30_任务卡库/在办/trim-1029/B0R.md` | trim-verify | 本卡 |
| `docs/30_任务卡库/在办/trim-1029/B0.md` | trim-verify | **仅追加勘误段**（§12），不改任何原有结论/勾选 |
| `_tmp_session/TRIM-EXEC-20261029/B0R/**` | trim-verify | 新证据、复核日志、重算脚本 |
| `docs/目录索引.md` / `docs/目录索引.json` | trim-verify（唯一索引写者） | 增量生成 `--ns trim-1029`，保留外部在途行 |

**禁止写**：`dzrobot_v2/**`、`dzsmartscout/**`、`docs/20_资料索引库/**`、`docs/history_cache/`、`clear/`、`docs/30_任务卡库/在办/archrev-1028/`、`docs/30_任务卡库/归档/`、`docs/_工具/**`、他人 `_tmp_session/` 目录。**不 commit / 不 push / 不部署 / 不连车 / 不安包 / 不改守卫或 baseline**。

## 5 验收命令（可复跑）

> 重算脚本在**原工作区**跑（只读 B0 已落盘证据）；守卫在**原工作区**跑；抽样复跑在**唯一隔离副本** `B0/accept_root` 内跑。

```powershell
cd D:\User\Desktop\比赛存档\v1
$env:PYTHONUTF8='1'; $env:PYTHONIOENCODING='utf-8'; $env:PYTHONDONTWRITEBYTECODE='1'
python -B -X utf8 _tmp_session/TRIM-EXEC-20261029/B0R/scripts/01_reconcile.py    # 冻结 B0 + 首轮重算
python -B -X utf8 _tmp_session/TRIM-EXEC-20261029/B0R/scripts/02_ledger.py       # 单位分离后的权威套件台账
python -B -X utf8 _tmp_session/TRIM-EXEC-20261029/B0R/scripts/03_inputs.py       # 十棵输入树指纹 + 副本/现场漂移
python -B -X utf8 _tmp_session/TRIM-EXEC-20261029/B0R/scripts/04_guards.py       # 门禁红逐文件归属
python -B -X utf8 _tmp_session/TRIM-EXEC-20261029/B0R/scripts/05_classify.py     # 19 套在册/首次观测分类 + 非回归规范
python -B -X utf8 _tmp_session/TRIM-EXEC-20261029/B0R/scripts/06_rerun_check.py  # 4 套抽样复跑（副本内）
python -B -X utf8 _tmp_session/TRIM-EXEC-20261029/B0R/scripts/07_guards_run.py   # 六道守卫真退码
```

| ID | 命令 | 预期 | 实测 |
|---|---|---|---|
| R1 | `01_reconcile.py` | B0 138 个证据文件 SHA256 冻结清单落盘 | exit 0；frozen=138 |
| R2 | `02_ledger.py` | 59 套逐套单位归一；五类单位分开；145/83/94 对账闭合 | exit 0；**3949 = 3859 + 83 + 7** 恒等成立 |
| R3 | `03_inputs.py` | 10 棵输入树指纹 + 822 输入对现场漂移检查 | exit 0；**same 819 / differs 3 / missing 0** |
| R4 | `04_guards.py` | doc_policy 逐条 + anchors 新增红逐条归属 | exit 0；doc_policy 3R+7Y9、anchors 15 |
| R5 | `05_classify.py` | 19 套零遗漏分类，汇总 == 明细 | exit 0；**registered 12 + 集合在册 1 + 首次观测 6 = 19** |
| R6 | `06_rerun_check.py` | 4 套抽样复跑与 B0 计数一致 | exit 0；**4/4 reproduced_b0_counts=true** |
| R7 | `07_guards_run.py` | 六道守卫真退出码 | doc_policy **1**、anchors **1**、numbers **0**、wf_check **0**、wf_links **0**、wf_reuse **0** |

## 6 回滚点

- **本卡不改任何业务源码 / 配置 / 测试 / 守卫 / baseline**，正常情形无需回滚。
- 本卡唯一的治理写入是 `docs/目录索引.md|json` 的增量重生成（树指定的唯一索引写者职责）；如需撤销按 `git checkout -- docs/目录索引.md docs/目录索引.json`（**仅限这两个文件**）。
- B0 证据**零写入**：已用 `evidence/b0_frozen_manifest.json` 冻结 138 个文件，复核后 `same=138 / diff=0 / missing=0`。
- 抽样复跑全部写在 `B0R/logs/`，未碰 `B0/logs/`。

## 7 执行记录

> 全部时间 2026-10-30。证据根 `_tmp_session/TRIM-EXEC-20261029/B0R/`。

### 7.1 冻结 B0 原始证据（先做，后算）

`01_reconcile.py` 先对 B0 的 `evidence/` 与 `logs/` 下 138 个 `.json/.csv/.log/.patch/.txt/.md` 逐一记 `sha256 + bytes + mtime`，落 `evidence/b0_frozen_manifest.json`。

收口前复核：**same=138 / diff=0 / missing=0** ⇒ B0 原始日志与汇总**一字未动**，本卡所有更正另存新文件。

### 7.2 五类计数单位必须分开（B0 的 3949 不能当判据）

| 单位 | 套数 | passed | failed | total | skipped | 语义 |
|---|---:|---:|---:|---:|---:|---|
| `harness_assertion` | 46 | 3441 | 63 | 3504 | 0 | `harness.check()` 一次 = 一个断言；`finish()` 会把失败**再打印一遍** |
| `unittest_case` | 8 | 244 | 17 | 268 | 7 | `Ran N` 含 skipped；pass = Ran − fail − error − skipped |
| `cpp_check` | 1 | 26 | 0 | 26 | 0 | WSL g++ 真编译真回调的检查项 |
| `perception_testfunc` | 1 | 99 | 1 | 100 | 0 | 逐 `test_*` 函数；1 项失败是**模块 import**不是用例 |
| `selfcheck_check` | 1 | 49 | 2 | 51 | 0 | `[OK]/[FAIL]`；另有 `[LIM]` 已知局限行（本套 0 条） |
| `CRASH` | 2 | 0 | 11 | 0 | 0 | 负计数，**不算通过也不算 0** |
| **合计** | **59** | **3859** | **83** | **3949** | **7** | 只作量级参考 |

**恒等式（本卡闭合）**：`3949 = 3859 + 83 + 7` ✅（skipped 已在 Ran 内，故三项互斥）

### 7.3 三个数字的口径差（这是 B0 汇总自相矛盾的根）

| 数 | 真身 | 与其它数差在哪 |
|---|---|---|
| **145** | `fail_cases.json` 条目数 | **行级去重**：harness 双打印 ⇒ 每失败 2 行；`planner`/`mission_ctl` 的单打印按 1 行算；perception 的 import 失败**根本没被抓**（19_fail_cases.py 的 perception 正则只匹配 `[FAIL] name`） |
| **83** | CSV `fails` 列合计 | **不含** `test_planner` 的 11 条（该套 CSV 记 −1 因为终态是 CRASH） |
| **94** | 本卡枚举的**去重失败行** | = 83 + 11（planner 崩溃前已打印的 11 条）；`incremental_plan` 贡献 0（在第一次 `plan()` 就崩，一条 FAIL 都没打） |
| **3866** | B0 卡面的“通过” | **多算了 7**：`09_run_scout.py` 与 `12_parse_logs.py` 算 unittest pass 用 `Ran − FAIL/ERROR`，**没减 skipped（5+2）**。正确值 **3859** |

**145 = 94 + 51**：51 条是 harness `finish()` 的重复汇总行，逐套差额见 `count_reconciliation.json`：

```
bridge_gate 6=3+3   corner_cut 2=1+1   drift 2=1+1   link_e2e 6=3+3   mission 34=17+17
path_overlap 6=3+3  physics 8=4+4     sim 20=10+10   sim_guard 20=10+10
planner 11=11+0(崩溃前打印，无双汇总)   mission_ctl 7=7+0   planner_sync 7=7+0
route_crop 10=10+0  airground 2=2+0    calibration_backfill 1=1+0  route 1=1+0
motion_limit 2=2+0   perception 0 (import 失败从未入库)
```

### 7.4 19 套非零退出逐套分类（汇总 == 明细）

**B0 说“未在册 9 套”却只列 2 车 + 5 机 = 7 套，差的 2 套是哪来的？**

答：**把“登记族”当“套件”数了**。B0 交付写“10 套车端红可在册（K10/K11/K12/K13/D-023 C 类/D 类 CRASH/L5）”——那是 **7 个登记族**，落到**套件**上是 **12 套**。再按“19 − 10 = 9”作差，分母就已经错了。逐套记账如下（**这是唯一权威表**）：

| # | 套件 | exit | 分类 | 依据（可独立核对） |
|---:|---|---|---|---|
| 1 | `car__test_bridge_gate` | 1 | **在册** | `06_回归与测试基线.md:14,32`（L5，**禁止重钉**） |
| 2 | `car__test_corner_cut` | 1 | **在册** | `08_回归守护_known_reds.md:222 §K12` |
| 3 | `car__test_drift` | 1 | **在册** | `08_回归守护_known_reds.md:230 §K13` |
| 4 | `car__test_mission` | 1 | **在册** | `08_回归守护_known_reds.md:202 §K10 / :214 §K11` |
| 5 | `car__test_sim` | 1 | **在册** | `06_回归与测试基线.md:22–25`（C 夹具 + A2/A3 真缺陷） |
| 6 | `car__test_sim_guard` | 1 | **在册** | `06_回归与测试基线.md:21,26`（C 夹具 + A1/A3） |
| 7 | `car__test_path_overlap` | 1 | **在册** | `06_回归与测试基线.md:31` |
| 8 | `car__test_route` | 1 | **在册** | `06_回归与测试基线.md:28` |
| 9 | `car__test_motion_limit` | 1 | **在册** | `06_回归与测试基线.md:29` |
| 10 | `car__test_link_e2e` | 1 | **在册** | `06_回归与测试基线.md:30` |
| 11 | `car__test_planner` | 1 | **在册** | `06_回归与测试基线.md:33`（D 类 废引擎 CRASH） |
| 12 | `car__test_incremental_plan` | 1 | **在册** | `06_回归与测试基线.md:33` 同族（B0 标 D 类） |
| 13 | `car__test_calib_backfill` | 1 | **集合在册 / 逐套红未在册** | `06_回归与测试基线.md:35` **明写**“其红数/断言在 37 套台账 §2 未单列 — **未核实**”；`05_已知红与回归口径.md:39` 只把它列进 C 类重钉**集合** |
| 14 | `car__test_physics` | 1 | **首次观测** | 全 `docs/` grep `test_physics` **无任何 known_reds 条目**；全景只记旧值 103/103 |
| 15 | `scout_planner__test_planner_sync` | 1 | **首次观测** | `08_回归守护_known_reds.md` 全文 0 命中；机端从未纳入回归口径 |
| 16 | `scout_planner__test_route_crop` | 1 | **首次观测** | 同上 |
| 17 | `scout_planner__test_mission_ctl` | 1 | **首次观测** | 同上 |
| 18 | `scout_perception__run_tests` | 1 | **首次观测** | 同上；且为 HEAD 树既有 ImportError |
| 19 | `scout_sim_airground__selfcheck` | 1 | **首次观测** | 同上；49/51 的 2 项是 `[FAIL]` 不是 `[LIM]` |

**汇总 == 明细**：在册 **12** + 集合在册逐套未在册 **1** + 首次观测 **6** = **19** ✅

> **口径修正**（相对 B0 交付文字）：未在册是 **7 套**（1 集合在册 + 6 首次观测），不是 9 套；B0 的 9 来自用“族数 10”作差。**这不改变任何失败事实，只是把分母说清。**

### 7.5 perception 的 ImportError 按什么计

```
[import] perception.tests.test_field_layout_v2
ImportError: cannot import name 'FIELD_LAYOUT_VERSION' from 'perception.protocol'
```

| 问题 | 事实 |
|---|---|
| 崩溃发生在哪 | **模块导入阶段**，`run_tests.py` 的 `except` 把它记成 `failed += 1` |
| 所以“失败 1”是什么 | 是**一个模块**，**不是**一个测试函数失败；该模块内的用例**一个都没执行** |
| 未执行多少 | `test_field_layout_v2.py` 有 **11 个 `def test_`**，全部 0 执行（`unrun=11`） |
| 影响覆盖吗 | **是**：99/100 里那个 100 的分母不含这 11 个 ⇒ 真实覆盖是 **99 / 111** |
| 是副本造成的吗 | **不是**：`git grep FIELD_LAYOUT_VERSION` 在 **HEAD 树**与工作树**完全相同**（仅 4 处命中，全部在 test 文件里引用一个已不存在的符号）；`protocol.py` 从未导出该常量 |
| fail_cases.json 抓到它了吗 | **没有**：`19_fail_cases.py` 的 perception 正则只匹配 `[FAIL] name`，不匹配 `[import] module` ⇒ 145 条里**缺这一条崩溃** |

**结论**：perception 必须记作 **「模块导入崩溃 + 11 用例未执行 + 覆盖 99/111」**，不能记成 “99/100 通过 1 失败”。

### 7.6 `_tmp_session/TRIM-EXEC-20261029/B0/logs/suite_table_scout.csv`  与 `suite_table_v2.csv` 为什么不一样

| 文件 | 谁写的 | 性质 |
|---|---|---|
| `_tmp_session/TRIM-EXEC-20261029/B0/logs/suite_table.csv` （49 行） | `08_run_car.py` **runner 原生输出** | **exit 权威来源** |
| `_tmp_session/TRIM-EXEC-20261029/B0/logs/suite_table_scout.csv` （10 行） | `09_run_scout.py` **runner 原生输出** | **exit 权威来源**；`test_mission_ctl` 因无匹配正则记为 `pass=-1/total=-1/parse=UNPARSED` |
| `suite_table_v2.csv`（59 行） | `12_parse_logs.py` **只用日志重解析**，再用前两个 CSV 回填 exit | **派生视图**；同时把 car 与 scout 合并成 59 行 |

三处差异的成因（逐条已核对）：

1. **mission_ctl 在 scout CSV 是 −1/−1/UNPARSED，在 v2 是 2/9/harness**：v2 的正则认 `结果：2/9 通过`（无“失败”尾巴），scout 的 `parse()` 只认 `结果：N/M 全部通过|通过，F 失败`，没覆盖纯 `通过` 结尾 ⇒ **v2 更准**。
2. **pursuit 在 v2 是 167/167 assert-line，在 suite_table.csv 是 −1**：`car__test_pursuit` 用 `断言：PASS 167 / FAIL 0` 的格式，`08_run_car.py` 的 `classify()` 只认 `结果：N/M` ⇒ runner 记 −1，v2 认出来了。**这是 B0「3866」虚高的另一半来源被掩盖的地方**（B0 用 v2 的 3866，但又用 suite_table.csv 的 3592 口径解释）。
3. **cpp 在 v2 是 26/26 cpp-fixture，在 suite_table.csv 是 −1**：`08_run_car.py` 没实现 `control contract: N passed, M failed` 分支。

> **裁定**：**exit 只认两个 runner 原生 CSV**（它们是真实进程退出码）；**计数以本卡 `suite_ledger.json` 为准**（它同时认全部五种格式，并区分双打印）。v2 CSV 的负数是**解析器缺口**，不是“0 个用例”。

### 7.7 抽样复跑（4 套，非全量）

只挑 B0 运行时长 <1.5 s 的 4 套，在**唯一隔离副本** `accept_root` 内复跑，验证「同输入 ⇒ 同计数」：

| 套件 | B0 | 复跑 | 一致 | 备注 |
|---|---|---|---|---|
| `car__test_drift` | 23/24 | 23/24 | **✅** | 日志 SHA256 **逐字节相同** |
| `car__test_route` | 156/157 | 156/157 | **✅** | 日志 SHA256 逐字节相同 |
| `car__test_calib_backfill` | 100/101 | 100/101 | **✅** | 计数相同；日志差 3 行（**临时目录名 + 时间戳**，非结果） |
| `scout_planner__test_planner_sync` | 4/11 | 4/11 | **✅** | 计数相同；日志只差 `Ran 11 tests in 4.816s` → `3.198s`（**耗时**） |

**4/4 计数一致**。差异行全部是**非确定性元数据**（tmp 目录名、时间戳、耗时），**没有一条是断言结果**。这给 R0 一个可核对的样本。

### 7.8 输入指纹与漂移

十棵输入树（只算真正被测试消费的目录）落 `evidence/input_hashes.json`：

| 树 | 文件数 | 聚合 SHA256（前 24 位） |
|---|---:|---|
| `car_dzbrain_src` | 98 | `25b28d1e4d1395793e49db89` |
| `car_dzbrain_tests` | 51 | `01981a0c0adba4d74b1bf15b` |
| `car_config` | 3 | `f194540dc3932fcbae4f2d28` |
| `car_rknn_contract` | 5 | `0f85f15feb95beb166364d38` |
| `car_tools` | 27 | `e738f20602caf2b1871a6bcd` |
| `scout_link` | 31 | `d7af62e9db96f499c1e14b45` |
| `scout_planner_tests` | 2 | `fab0b829024e87dfb40351cb` |
| `scout_planner_core` | 10 | `17d1b6fd7ad0399c474ac8df` |
| `scout_perception` | 28 | `043cb5ac07c3a724e275bcaf` |
| `scout_airground` | 15 | `f70f9f799e73f3898f961e02` |

另按套落 59 个测试脚本的精确 `sha256`（`per_suite_test_file`）。

**副本 vs 现场漂移**：822 输入 → **same 819 / differs 3 / missing 0**。3 个差异：

| 路径 | 原因 | 谁改动 |
|---|---|---|
| `docs/目录索引.json` | B0 自己的索引写者职责（B0 卡 A-19 已登记） | trim-verify（B0） |
| `docs/目录索引.md` | 同上 | trim-verify（B0） |
| `dzrobot_v2/docs/TEAM_STATUS.md` | **B0 未登记的新漂移**：该文件在 dzrobot_v2 是 `M`（已跟踪已改），B0 建副本时取的是 20:56 版，现为 00:01 版（80628→81593 B） | **他人**（状态板由 Lead 每轮更新） |

> **更正 B0 的「820/822 逐字节同开工」**：实际是 **819/822**。B0 漏记了 `TEAM_STATUS.md`——它在 B0 期间被 Lead 更新过，B0 既没把它列进恢复材料，也没标为在途保护。**这是 B0 的第 4 个口径缺陷**，本卡补记（该文件不参与任何测试，故不影响基线计数；但它是**在途保护遗漏**）。

### 7.9 全局门禁红逐文件归属

**doc_policy：exit 1，3 条 R + 7 条 Y9（B0 收口时是 3R + 7Y9；开工时是 4R）**

| 规则 | 目标文件 | 存在 | 字节 | SHA256(前 12) | ns / zone | 并发状态 | owner | 拟议最小修复 |
|---|---|---|---:|---|---|---|---|---|
| R1 | `clear/ontology/verification-loop.md` | ✅ | 7123 | `220dba316478` | 过程区外层 `clear/` | **未跟踪**（`?? clear/`） | 外部会话（非 trim-1029） | ① 移入 `docs/` 并登记；② 或 Lead 在守卫配置 `--accept` 并写理由。**本团队越域，不处置** |
| R11 | `docs/30_任务卡库/在办/archrev-1028/DBG-LINK-DIAG-20261030.md` | ✅ | 7697 | `ec457c5701ed` | archrev-1028 / 在办 | **未跟踪**（`??`），他人活跃在途 | archrev-1028 | 把 front matter `status: 完成` 改成 `pending`/`in_progress`，说明移正文。**本团队越域，不处置** |
| R4 | 同上 | ✅ | 7697 | `ec457c5701ed` | 同上 | 同上 | 同上 | 同上（R11 与 R4 是**同一文件同一个根因**的两条规则） |
| Y9 ×7 | `docs/30_任务卡库/在办/trim-1029/B0.md` | ✅ | — | — | trim-1029 / 在办 | 本卡独占 | **trim-verify** | **已修**：路径后缺空格导致正则把 `：1684`/`(16)`/`：26` 一起吞进路径。改成“路径 + 空格 + 括号说明”后**7 条 Y9 全部消失** |

**修复前后实测**：`final__doc_policy.log`（B0 收口，红线 10）→ 本卡 `_tmp_session/TRIM-EXEC-20261029/B0R/logs/guard__doc_policy.log` （红线 **3**）。**delta = −7，全部是我自己的 Y9；R1/R11/R4 原样未动。**

**anchors：exit 1，新增红 15（B0 收口时也是 15；本卡归因 trim-1029 = 0）**

| # | 文档 | 裸引用 | 文档 zone / ns | 引用可解析 | 在 K2 在册集 | owner |
|---:|---|---|---|---|---|---|
| 1–2 | `docs/00_项目总览/交接_20261024_打靶闭环.md` | mvlog.csv, mvrec.py（裸写，无目录前缀） | 项目总览 | ❌ | ❌ | 该会话 owner |
| 3 | `docs/20_资料索引库/车端/06_回归与测试基线.md` | suite_table.csv（裸写） | 资料库 | ❌ | ❌ | 车端基线的卡 owner |
| 4 | `agc-1024/AGC-18-FIRE-DZ-DRIFT-20261024.md` | mvrec.py（裸写） | agc-1024 / 在办 | ❌ | ❌ | agc-1024 |
| 5–6 | `agc-1024/AGC-P1-TARGET-20261024.md` | mvrec.py, mvlog.csv（裸写） | agc-1024 / 在办 | ❌ | ❌ | agc-1024 |
| 7 | `agc-1024/AGC-T1-PROGRESS-20261024.md` | run_suite.py（裸写） | agc-1024 / 在办 | ❌ | **✅ K2** | agc-1024 |
| 8 | `agc-1026/00_树.md` | tag.txt（裸写） | agc-1026 / 在办 | ❌ | **✅ K2** | agc-1026（**树是单写点，只有 ns owner 能改**） |
| 9 | `agc-1026/RECT-LINK.md` | probes/run_suites.py（裸写） | agc-1026 / 在办 | ❌ | **✅ K2** | agc-1026 |
| 10–11 | `archrev-1028/DBG-LINK-DIAG-20261030.md` | mvlog.csv, mvrec.py（裸写） | archrev-1028 / 在办 | ❌ | ❌ | archrev-1028 |
| 12 | `lead-1023/SIM-STUDIO-OPT-20261024.md` | dzrobot_v2/artifacts/tracking_readonly_20261020.json（裸写，原物已不存在） | lead-1023 / 在办 | ❌ | ❌ | lead-1023 |
| 13 | `traj-1025/TRAJ-BASE-20261025.md` | suite_table.csv（裸写） | traj-1025 / 在办 | ❌ | ❌ | traj-1025 |
| 14–15 | `归档/tools-1022/2026-10-23_TMP-GC-20261023.md` | pack_20261023.zip ×2（裸写） | tools-1022 / **归档** | ❌ | ❌ | tools-1022（**归档不许改，只能标〔档〕/〔失效〕**） |

**归因结论**：K2 在册 **3** 条（#7/#8/#9，与 `08_回归守护_known_reds.md` §K2 逐条一致）；**新观测 12** 条；**trim-1029 归因 0**（`grep trim-1029` = 0 命中）。全部落在**他人写域**。

**拟议最小修复（按风险排序，均不由本卡执行）**：
1. **#14/#15 归档**：只能在该行尾标 `〔档〕`——**不许改归档正文**（改动归档 = 伪造历史）。
2. **#7/#8/#9 K2 在册**：直接改裸文件名为 `_tmp_session/AGC-RECTIFY-20261026/...` 完整相对路径（同 ns 上一轮就是这么修的，新增红 6→0）。
3. **#1/#2/#3/#4/#5/#6/#10/#11/#12/#13**：把 mvlog.csv / mvrec.py / run_suite.py / suite_table.csv / tracking_readonly_20261020.json 换成可解析路径，或在该行尾标 〔档〕。
4. **禁止**：为消红跑 `--write-baseline` 扩容基线（守卫自己就写了“重建默认纯收缩，写宽了就是洗白”）。

**其他四道守卫**（本卡实跑真退出码）：`numbers 0`（2 条已声明异文 K4）、`wf_check 0`（GREEN，**不代表回归绿**）、`wf_links 0`、`wf_reuse 0`。

### 7.10 本次复核**没有**做的事

| 项 | 状态 |
|---|---|
| 重跑全部 59 套 | **没做**（只抽样 4 套），避免重复 B0 的采集与时间成本 |
| 改 B0 的任何原结论 | **没做**；只在 B0.md 追加 §12 勘误段，原勾选与结论一字未动 |
| 改守卫 / baseline | **没做** |
| 改他人卡 / `clear/` / `归档` | **没做** |
| 修源码 / 测试 / 加依赖 | **没做** |
| 新增正式 K 编号 | **没做**；只用 `TRIM-B0R` 临时编号 |

## 8 第一阶段逐套非回归比较规范（R0 用的判据）

> 由本卡定义，**trim-review 独立验明后才生效**。完整机器可读版见 `evidence/red_classification.json` 的 `non_regression_spec`。

### 8.1 固定输入（全部必须逐字节相等）

1. **解释器**：CPython 3.12.10（绝对路径见 `input_hashes.json.interpreter.exe`）
2. **环境**：`PYTHONUTF8=1`、`PYTHONIOENCODING=utf-8`、`PYTHONDONTWRITEBYTECODE=1`；车端另加 `PYTHONPATH=<root>/dzrobot_v2/src/dzbrain`
3. **十棵输入树**：`input_hashes.json.trees.*.aggregate_sha256` 全部相等；任一不等 = **输入漂移 = 阻断**
4. **59 个测试脚本**：`per_suite_test_file[*].sha256` 全部相等；任一不等 = **测试被改 = 阻断**

### 8.2 六条不变量（逐套判定）

| # | 判据 | 违反后果 |
|---|---|---|
| **N1** | 套件**名单集合**相等（59 套），不许少 | 阻断（减少覆盖） |
| **N2** | 单套 `total`（同类单位）**不减少** | 阻断（删断言/删用例） |
| **N3** | 基线里**通过的**断言/用例 ID 不得出现在新失败集合 | 阻断（原通过转失败） |
| **N4** | 新失败 ID 集合 **⊆** 基线失败 ID 集合（允许缩小，**禁止新增**） | 阻断（失败集合扩大） |
| **N5** | 新 `skipped` ≤ 基线（车端 0，机端 7） | 阻断（新增 skip） |
| **N6** | **CRASH / UNPARSED / 零检查不得算通过**：基线 2 套 CRASH + `test_aim_probe` 0 断言，即使“这次没崩”也**不得记 pass** | 阻断（假绿） |

### 8.3 受既有失败遮蔽的变更

- **判定**：若改动只落在既有红套件覆盖的路径上，该套件的红**无法区分**“旧红”与“新引入红”。
- **要求**：必须补一条**能独立证明该改动无副作用**的证据（同输入的定点复算 / 逐用例对拍），**否则该文件不进白名单**。
- **禁止**：不得靠“反正这套早就红”把新增失败吞掉；不得改测试期望、加 `skip`、删用例来消红。

### 8.4 单位分离（比较时不许跨单位）

`harness_assertion` / `unittest_case` / `cpp_check` / `perception_testfunc` / `selfcheck_check` **五者不可相加**。`3949` 是混合量级，**不可作判据**；判据必须逐套、逐单位、逐 ID 比较。

## 9 结论

| 项 | 判定 | 证据 |
|---|---|---|
| B0 原始证据冻结、未覆盖 | **成立** | `evidence/b0_frozen_manifest.json`（138 文件）；复核 same=138/diff=0/missing=0 |
| 19 套非零退出逐套分类、汇总==明细 | **成立** | `evidence/red_classification.json`；12 + 1 + 6 = 19 |
| 五类计数单位分开、skip/CRASH/未跑/零检查正确区分 | **成立** | `evidence/suite_ledger.json`；`3949 = 3859 + 83 + 7` |
| 145 与 83 的口径差可追踪 | **成立** | `evidence/count_reconciliation.json`；145 = 94 + 51 |
| 逐失败稳定 ID + 命令 + 解释器 + 输入哈希够 R0 核对 | **成立** | `suite_ledger.json.fail_ids` + `input_hashes.json` + `rerun_check.json` 4/4 一致 |
| 非回归规范写完 | **成立** | §8 + `red_classification.json.non_regression_spec` |
| 门禁红逐文件归属 + 最小修复建议 | **成立** | `evidence/guard_attribution.json`；doc_policy 3R+7Y9、anchors 15 |
| 未越域修改 / 未扩基线 / 未改断言掩盖红 | **成立** | 唯一治理写入是本 ns 卡 + 索引；B0 证据零写入 |
| 结果交 trim-review | **成立** | 已发消息；R0 集合复核前不放行 |

### 未通过 / 未验证（必须写明）

1. **全局仍红**：doc_policy exit 1（3 条，全在他人写域）+ anchors exit 1（15 条新增红，全在他人写域，trim-1029 归因 0）。**本卡无权处置**，交 Lead。
2. **19 套非零退出仍是失败**，其中 **6 套首次观测**（physics + 机端 5）在现有台账**无法在册**，属台账缺口，需 Lead 裁决。
3. **实车链路完全未验证**（本卡禁止连车）。
4. **`test_incremental_plan` 的 11 条失败无法枚举**：它在第一次 `plan()` 就崩溃，一条 `FAIL` 都没打印 ⇒ 该套件**只知道崩了，不知道崩在哪个断言**。这是 B0 与本卡都**无法补救的**证据缺口。
5. **`dzsmartscout/src`（162 MB 第三方）不在副本内**，真实 ROS 节点功能未被覆盖。
6. **抽样只 4 套**：其余 55 套的“同输入⇒同计数”是**由 B0 单次运行推断**，未经本卡二次确认。

## 10 文档变更

| 动作 | 文档 | 说明 |
|---|---|---|
| 新增 | `docs/30_任务卡库/在办/trim-1029/B0R.md` | 本卡 |
| 追加 | `docs/30_任务卡库/在办/trim-1029/B0.md` §12 勘误段 | 只追加，不改原结论/勾选 |
| 修正 | `docs/30_任务卡库/在办/trim-1029/B0.md` 6 处 Y9 触发行 | 路径后加空格，清掉 7 条 Y9 红 |
| 生成 | `docs/目录索引.md` / `docs/目录索引.json` | `gen_index.py --ns trim-1029` 增量；保留外部在途行 |
| 无 | 其余 | 未写源码、未改守卫、未改他人卡、未动归档/clear |

## 11 验收勾选清单（领卡时全 false，收口时逐项勾）

| # | 验收项 | 状态 |
|---|---|---|
| A-1 | **成立** — `_tmp_session/TRIM-EXEC-20261029/B0R/evidence/red_classification.json` ：19 套逐套 12 在册 + 1 集合在册逐套未在册 + 6 首次观测 = 19；见 §7.4 表 |
| A-2 | **成立** — 同上；12 + 1 + 6 = 19 与 §7.4 明细表逐行相等，无「9」这类笼统数 |
| A-3 | **成立** — `_tmp_session/TRIM-EXEC-20261029/B0R/evidence/suite_ledger.json` 的 unit_breakdown：harness_assertion 46 套 / unittest_case 8 / cpp_check 1 / perception_testfunc 1 / selfcheck_check 1 / CRASH 2 |
| A-4 | **成立** — skipped 7（全在 link）、CRASH 2（负计数不计通过）、perception 11 用例未执行、aim_probe 0 断言；见 §7.2 与 §7.5 |
| A-5 | **成立** — B0 原日志 138 文件 SHA256 冻结（`_tmp_session/TRIM-EXEC-20261029/B0R/evidence/b0_frozen_manifest.json`），复核 same=138/diff=0/missing=0；更正全写 §12 新段，未覆盖任何原日志 |
| A-6 | **成立** — `_tmp_session/TRIM-EXEC-20261029/B0R/evidence/input_hashes.json`（10 棵输入树聚合哈希 + 59 个测试脚本 sha256 + 解释器/env）+ `_tmp_session/TRIM-EXEC-20261029/B0R/evidence/rerun_check.json`（4/4 计数一致） |
| A-7 | **成立** — `_tmp_session/TRIM-EXEC-20261029/B0R/evidence/count_reconciliation.json` 逐套 excess 列：145 = 94 真实 + 51 重复；harness finish() 双打印逐套可算 |
| A-8 | **成立** — §9「未通过/未验证」6 条：全局仍红、6 套首次观测未在册、实车未验证、incremental_plan 失败无法枚举、dzsmartscout/src 未覆盖、只抽样 4 套 |
| A-9 | **成立** — §8 N1–N6 六条不变量 + 固定输入四条 + 单位分离；机器可读版在 `_tmp_session/TRIM-EXEC-20261029/B0R/evidence/red_classification.json` 的 non_regression_spec |
| A-10 | **成立** — §8.3：受遮蔽变更须补独立证据或排除，禁止靠旧红吞新红；同时禁止改期望/加 skip/删用例 |
| A-11 | **成立** — `_tmp_session/TRIM-EXEC-20261029/B0R/evidence/guard_attribution.json` —— doc_policy 3R + 7Y9（含 owner/ns/并发/sha256/最小修复）；anchors 15 条逐条含 doc zone/可解析性/K2 在册/owner |
| A-12 | **成立** — 唯一治理写入是 trim-1029 卡 + 目录索引；B0 证据零写入；守卫与 baseline 未动；未装依赖；未改任何断言/期望 |
| A-13 | **成立** — 已向 trim-review 发送复核结论与本卡证据路径；R0 集合复核前本卡不放行任何白名单 |
| A-14 | **成立** — `_tmp_session/TRIM-EXEC-20261029/B0R/evidence/index_check.json` —— 9 条被重排索引行 9/9 全部重现、lost=0；B0/B0R/树/外部在途 6 项全部在册 |
| A-15 | **成立** — B0 原 §1–§11 与 A-1~A-20 全部保留原文（逐项已核）；本卡只追加 §12 勘误段并修正 6 处 Y9 触发写法的空格 |

## 2026-10-30 授权后的定位补注

此前原位与红线记录保留为当时事实；本次获用户明确授权，将相同字节设计稿纳入既有历史归档。它不是项目实现、默认值或验收结果。旧定位仅由下列本文档唯一映射指向原文：
- [clear/ontology/verification-loop.md](../../../history_cache/01_旧文档归档/clear/ontology/verification-loop.md)
