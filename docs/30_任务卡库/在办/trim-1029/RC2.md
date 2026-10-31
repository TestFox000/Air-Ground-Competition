---
卡号: RC2
ns: trim-1029
agent: trim-doc
树节点: TRIM-18
owner: trim-doc
reviewed_task: t14（C2 车端扩展，实现者 trim-car）
kind: review
status: in_progress
更新: 2026-10-30 12:45
摘要: 独立评审 C2（t14）—— 三文件后像哈希、六块纯注释删除可精确重建、token/预处理/__LINE__ 等价、保留项在位、dzactuator 零改动、前像==B0。verdict=pass，未修发现 0；另登记 2 条不阻断的表述 nit。
---

# RC2 · 独立评审扩展车端最新实现（审 t14 / C2）

## verdict = **pass**（未修发现 0；2 条表述 nit 已登记，不阻断）

**独立性声明**：C2（t14）实现者为 **trim-car**，我是 **trim-doc**，非本实现作者，可独立评审。全部判据为我**本人独立复算**，不继承 C2 卡自报数字。

## 1 独立复算（逐项）

### 1.1 三文件后像哈希 vs R0 声明

| 文件 | live 字节 | live sha16 | R0 C2.after | 相符 |
|---|---|---|---|---|
| `common.h` | 719 | `c27f327ad0f72c75` | `c27f327ad0f72c75` | ✔ |
| `rknnPool.hpp` | 3048 | `1c3f855547e6dd2c` | `1c3f855547e6dd2c` | ✔ |
| `dzjudgment.cpp` | 206 | `f5b779700f5512b4` | `f5b779700f5512b4` | ✔ |

三文件 live 均 **≠ R0.before**（`e073425f…` / `23a3807d…` / `5ff7aff9…`）⇒ 确实已改，且改到 R0 预定后像。**3/3 相符**。

### 1.2 六块严格删除可精确重建（最强判据）

用 **B0 副本前像 + 逐块删除 old_text** 独立重建：

| 文件 | 前像 sha16 | ==R0.before | 块 | old_text 命中 | rebuild==R0.after | rebuild==live |
|---|---|---|---|---|---|---|
| `common.h` | `e073425f` | ✔ | C2-BOX(125B) / C2-GROUP(183B) | 各 **1** | ✔ | ✔ |
| `rknnPool.hpp` | `23a3807d` | ✔ | C2-PUT(318B) / C2-PUTSIG(76B) / C2-PUTEXPR(90B) | 各 **1** | ✔ | ✔ |
| `dzjudgment.cpp` | `5ff7aff9` | ✔ | C2-MAIN(307B) | **1** | ✔ | ✔ |

⇒ 重建字节流与**真实工作区逐字节相同**，且与 R0 声明后像相同。**六块各唯一命中、new_text 全为空串** ⇒ 是**纯删除**，非替换改写。

### 1.3 删除行性质：全部纯注释，零活跃代码

| 块 | 行数 | 纯注释 |
|---|---|---|
| C2-BOX | 6 | ✔ |
| C2-GROUP | 7 | ✔ |
| C2-PUT | 7 | ✔ |
| C2-PUTSIG | 1 | ✔ |
| C2-PUTEXPR | 1 | ✔ |
| C2-MAIN | 10 | ✔ |

**逐行核**：每行 strip 后均以 `//` 开头，无一行含活跃语句。合计 32 行，与 C2 声明一致。

### 1.4 语义等价（三层分核）

| 层 | 判据 | common.h | rknnPool.hpp | dzjudgment.cpp |
|---|---|---|---|---|
| **token 流**（剥注释后序列） | 完全相同 | 113 = 113 ✔ | 696 = 696 ✔ | 68 = 68 ✔ |
| **预处理指令序列** | 逐条相同 | 5 = 5 ✔ | 9 = 9 ✔ | 2 = 2 ✔ |
| **__LINE__ / #line** | 前像是否存在 | 无 ✔ | 无 ✔ | 无 ✔ |

> 我的 token 计数（113/696/68）与 C2 自报（166/102/643/55 等）不同，原因是 **tokenizer 粒度不同**。**不影响等价结论** —— 关键是**同口径下前后完全相同**。

### 1.5 保留项在位（按内容核，不看 C2 自报）

| 保留项 | 判定 |
|---|---|
| `common.h` 的 `#define OBJ_NUMB_MAX_SIZE 64` | **在位** ✔（在白名单块之外，未被误删） |
| `rknnPool.hpp` 继承式 `// int put(inputType inputData);` | **在位** ✔（未列入删除面） |
| `dzjudgment.cpp` 两处重复 `#include` | **未去重** ✔（仍在第 1、3 行） |

### 1.6 删除量与行数

| 文件 | 前像行 | 后像行 | 删 | C2 声明 |
|---|---|---|---|---|
| `common.h` | 54 | 41 | **13** | 13 ✔ |
| `rknnPool.hpp` | 122 | 113 | **9** | 9 ✔ |
| `dzjudgment.cpp` | 21 | 11 | **10** | 10 ✔ |

合计 32 行，**零新增行**（后像行数 = 前像 − 删除数），无格式化痕迹。

### 1.7 越界核查（以 B0 822 输入为准）

逐文件比对 B0 `accept_root`：`same=835` / `diff=17` / `missing=72`（合计 924）。**逐一归因**：

| 变更文件 | 归属 |
|---|---|
| `dzjudgment.cpp` / `common.h` / `rknnPool.hpp` | **C2（t14, trim-car）** ← 只有这 3 个属 C2 |
| `rkpt.hpp` | C1（t6, trim-car） |
| `dz_send_waypoint.py` | S1(t7) + S2(t15, trim-doc) |
| `技术全景_C_车端硬件与配置.md` | D2（t16, trim-doc） |
| `技术全景_D_无人机.md` / `技术全景_E_空地链路与工程设施.md` | D1/D2（trim-doc） |
| `docs/目录索引.json` / `docs/目录索引.md` | **Lead**（索引唯一发布者） |
| `dzrobot_v2/docs/TEAM_STATUS.md` / `决策日志` | **Lead** |
| `dzrobot_v2/artifacts/sim/traj.csv` / `by_segment.csv` / `traj.svg` | V2（t17, trim-verify）仿真产物 |
| `_tmp_session/test_autoplan_fixtures/*.yaml` | 过程区夹具（非源码） |

⇒ **C2 只改 3 个白名单文件，其余 14 个变更各有其主，无越界**。

**missing 72 个**：全部在 `dzrobot_v2/artifacts/runs/`（运行产物），**不在 B0 822 输入范围**（B0 按设计排除），不构成越界或删源码。

### 1.8 dzactuator 零改动

- R0 白名单 5 文件里**没有 dzactuator** ⇒ C2「不凭叙述扩白名单」**正确**。
- 实测 `dzactuator/include/dzactuator.h`（13067 B）与 `src/dzactuator.cpp`（54907 B）**与 B0 逐字节相同** ⇒ 零改动。

### 1.9 恢复源真实性

C2 的 `preimage/` 三份前像与 **B0 副本逐字节相同**（1027 / 3532 / 513 B）⇒ 回滚源真实可用，非事后拼造。

### 1.10 保护资产

| 资产 | 与 B0 |
|---|---|
| `config/robot.yaml` (82269 B) | **same=True** ✔ |
| `config/map.yaml` (2370 B) | **same=True** ✔ |
| `tools/deploy_to_car.py` (23648 B) | **same=True** ✔ |
| `dzrknn_dataprocess.cpp` (85809 B) | **same=True** ✔ |

## 2 独立跑的门禁（原始退出码）

| 守卫 | 退出码 | 结果 |
|---|---|---|
| `check_doc_policy.py` | **1** | 红线 **3** 条，全外域既有：R1 `clear/verification-loop.md`、R11+R4 `archrev-1028/DBG-LINK-DIAG-20261030.md`；**trim-1029 归因 0** |
| `check_anchors.py` | **1** | `red_new` **15 occ / 14 唯一 key** = 锁定集合、`baseline=266` 未动；**trim-1029 命中 0** |
| `check_numbers.py` | **0** | 无新增漂移 |
| `tools/wf/check.py` | **0** | GREEN（仅既有 WARN） |
| `tools/wf/check_links.py` | **0** | 309 files / broken 0 |

> 全局仍 exit 1 属**外域既有红**，原样保留、**不重标为绿**。

## 3 两条表述 nit（已登记，不阻断）

### N-1 「仿真产物与 B0 逐字节相同」的比较对象须写明

C2 卡称 sim 的 `traj.csv` / `by_segment.csv` 与 B0 同名产物逐字节相同。我实测发现：**工作区 `dzrobot_v2/artifacts/sim/` 与 B0 并不相同**（live 369567 B vs B0 733650 B，mtime 22:15:16 早于 C2）。改用 **V2 隔离副本 `V2/accept_root_h2/dzrobot_v2/artifacts/sim/`** 比对，则三份**全部逐字节相同**（733650 / 3856 / 172303 B）。

⇒ C2 的**结论成立**，但卡面未写明比较对象是隔离副本，读者按字面去比工作区会产生误解。建议补一句「在隔离副本内跑，产物与 B0 逐字节相同」。**不影响实质通过**。

### N-2 token 计数与 C2 自报口径不同

我按「标识符整体」计数得 113/696/68，C2 报 166/102/643/55 等。**粒度不同**，但同口径下前后完全相等，等价结论不受影响。建议 C2 卡注明 tokenizer 口径。**不阻断**。

## 4 合同 acceptance 判定

| 条目 | 判定 | 依据 |
|---|---|---|
| 最新实现逐项满足 R0 与实施合同，无语义变化、知识遗失、越界修改或假绿 | **通过** | 后像 3/3 相符；六块重建成 live 逐字节同；token/预处理/__LINE__ 全等价；保留项 3/3 在位；越界 0（17 个 diff 逐一归因，C2 只占 3）；保护资产 4/4 同 B0；dzactuator 零改动；无假绿（全局红原样保留、未重标） |
| 独立验证和恢复证据齐全且对应最新哈希，未修发现为零才 verdict=pass | **通过** | 全部判据为本人独立复算；C2 preimage 3/3 == B0；门禁 5 条原始退出码已存；**未修发现 = 0**（2 条为表述 nit，不构成实质缺陷） |

## 5 边界

- 只写 `RC2.md` 与本卡过程区 `_tmp_session/TRIM-EXEC-20261029/RC2/`；**未改任何源码/测试/基线/他人卡**。
- 未跑新基线、未重跑 48 套 Python / 机端 10 套（C2 变更纯注释，token 级已证等价）；未编译 rknn_pt / dzjudgment ROS 包（无 SDK/ROS 环境，如实记录）。
- 索引仍 **Lead 单写**，我未生成。
- 未部署、未连车、未 commit/push。

## 6 证据

`_tmp_session/TRIM-EXEC-20261029/RC2/`：`v1_hashes.py`（后像哈希）、`v2_rebuild.py`（六块重建）、`v3_tokens.py`（token/预处理/__LINE__）、`v4_retain.py`（删除行性质 + 保留项）、`v5_scope.py`+`v6_attrib.py`（越界归因）、`v7_protect.py`（保护资产）、`v8_sim.py`+`v9_sim2.py`（仿真产物比对）、`v10_pre.py`+`v11_pre_eq.py`（前像真实性）、`v12_actuator.py`+`v13_act2.py`（dzactuator），以及 `guard_policy.json` / `guard_anchors.json` / `guard_numbers.log` / `guard_wf.log` / `guard_links.log` 五份原始门禁输出。
## 2026-10-30 授权后的定位补注

此前原位与红线记录保留为当时事实；本次获用户明确授权，将相同字节设计稿纳入既有历史归档。它不是项目实现、默认值或验收结果。旧定位仅由下列本文档唯一映射指向原文：
- [clear/verification-loop.md](../../../history_cache/01_旧文档归档/clear/ontology/verification-loop.md)
