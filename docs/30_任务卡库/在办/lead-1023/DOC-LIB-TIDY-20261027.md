---
来源: 本轮实测（check_anchors --json / check_numbers / wf/check 真实退出码）；AGENTS §4 判红清单
取证: 实/机检
更新: 2026-10-27
取代: 无
status: in_progress
ns: lead-1023
agent: lead
树节点: L-15
写域: docs/00_项目总览/**；docs/10_agent提示词/**；docs/20_资料索引库/**；dzrobot_v2/tools/wf/frozen.json
---

# DOC-LIB-TIDY-20261027 · 资料库债务整理

> 用户 2026-10-27 指令「整理一下资料库」。**不重排分册、不合并文档、不改守卫判据**——只做四类可机器复跑的债务清除，并把剩余债务分类成下一次可机械执行的清单。

## 0 五必填

| 项 | 内容 |
|---|---|
| 读过的台账条目 | D-029（轴距按固化值，不再重量）／D-033（靶速 0.3）／§4 判红清单（R1~R12、R6、R10） |
| 复用检索结果 | 复用现成守卫 `check_numbers.py --fix-*`、`check_anchors.py --write-baseline`、`frozen.json` 机制；复用 `docs/_work/rebuild_8_lost.md` 作为丢失过程稿的替代出处 |
| 新增轮子及理由 | **零新增脚本**。全部门禁改动走既有工具 |
| 验收命令 | `python docs/_工具/check_numbers.py` → exit 0；`python docs/_工具/check_anchors.py` → exit 0 且新增红 0；`cd dzrobot_v2 && python tools/wf/check.py` → GREEN 且「登记已失效」WARN 0 |
| 回滚点 | `git checkout -- docs dzrobot_v2/tools/wf/frozen.json`（本轮全部为文本编辑，无新增/删除文件） |

## 1 本轮做了什么（五类，全部机器可验）

| # | 债务 | 处置 | 前后 |
|---|---|---|---|
| 1 | `check_numbers` **红**：2 条新增行号漂移（`thread_rate_localization` / `thread_rate_mission`） | 代码锚点从 `:162 Localization` / `:1485 MissionTask` 改到真类定义行 `:163 LocalizationTask` / `:1507 MissionTask`（与 `tracker.py:139 TrackerTask` 同惯例） | exit 1 → **exit 0** |
| 2 | `frozen.json` 5 条 **失效 in_flight 登记**（文件早已不再脏） | 逐条核实「改动已进 HEAD（`945bbc14` / `5195da06`）且工作区干净」后删除：tracker / aim / drone / scout-layout / blackboard | 5 WARN → **0 WARN**；23→18 条 |
| 3 | 现行库 **29 行**指向已清理的 `_tmp_session` 过程稿（死引用） | 按守卫既有机制标 `〔失效：会话过程目录已清理〕`（同行走标记，不动路径本身，保留「当时看过什么」的信息） | 存量红 304 → **277**；已标失效 94 → **121** |
| 4 | 2 处**占位符被当成真路径**解析（裸 `xxx` / `YYYY` 形） | 改写为带 `<>` 的占位形式：`dzrobot_v2/docs/<文件名>.md`、`log_<YYYY-MM-DD>_<HHMM>.md`（守护只认带 `<>` 的占位） | 存量红 277 → **275** |
| 5 | `DOC-FIX-20261023` §P2-2 里 3 处「已规划但从未落地」的工具引用（`check_linerefs.py`） | 标〔失效：未落地〕并加一句核实说明——该能力实际由 `check_anchors.py` 实现（路径:行号抽取 + 歧义记黄不判红），避免后人去找一个不存在的脚本 | 存量红 275 → **274** |

## 2 门禁实测（真实退出码，未接管道）

| 守卫 | exit | 备注 |
|---|---|---|
| `gen_index.py` | 0 | 234 篇 |
| `check_doc_policy.py` | 0 | GREEN，红线 0 |
| `check_anchors.py` | 0 | 新增红 0／存量红 274／黄 464／已标失效 129 |
| `check_numbers.py` | 0 | 仅剩 `wheelbase_m` 一条**已声明异文**（D-029） |
| `wf/check.py` | 0 | GREEN，失效登记 WARN 已清零 |
| `wf/check_links.py` | 0 | 243 文件 0 断 |

## 3 剩余债务分类台账（274 红 / 464 黄）

> 分类由 `check_anchors.py --json` 全量导出后按目标路径形态归并。**判据：一条红只有「目标形态」决定修法，不能一刀切。**
> 基线同步收缩：`anchor_baseline.json` key 数 **238 → 217**（纯收缩，无洗白）。

| 类 | 条数 | 目标形态 | 正确修法 | 本轮为何不做 |
|---|---:|---|---|---|
| **D 裸文件名** | 166 | 无目录（`watchdog.ps1`〔失效：示例名，`.zhiyuan` 已清理〕、`poll.ps1`〔失效：示例名，`.zhiyuan` 已清理〕、`index.js`、`plugins.json`、`dzmem.py` 等） | 目标分散在 `.zhiyuan/`〔2026-10-27 已清理〕、`D:/User/Desktop/ZhiYuan/dsh_plugin/`、`git_save/`、DSH 安装目录；守卫对裸名不做全盘兜底 ⇒ **写法问题，不是目录可见性问题** | 改写法（补全路径）或标〔失效〕。原「给点目录开白名单」议题随 `.zhiyuan` 清理**自动消解** |
| **A `_tmp_session` 死引用** | 54 | 会话过程目录已清理 | 同 §1-3 标〔失效〕 | **现行库已清零（26 条已做 + 3 条 DOC-FIX）**；剩 54 条全在 `30_任务卡库` 各卡内，属他人写域 |
| **C 其它带目录** | 37 | 有目录但目标不存在 | 逐条查新位置或标〔失效〕 | 需逐条查证，含跨仓/外部路径，不可批处理 |
| **B 仓库内路径** | 17 | `dzrobot_v2/`、`docs/`、`tools/` 下 | 查新位置改指，或标〔失效〕 | 含 2 条已在 §1-4 处置；余 15 条同上 |
| 黄（引用搬到非现行区） | 464 | 目标只在 `_tmp_session`(443)/`history_cache`(7)/`_archive`(7)/`_work`(5)/`git_save`(2) | **多数不是缺陷**：任务卡引用自己 `_tmp_session` 证据是 §5 规定的正当行为 | 只处理「现行库正式文档依赖临时过程稿」这一子集（86 条 / 17 篇，含历史篇 43 条不动） |

**结论**：债务的主体是 D 类（166/274 = 61%）。

> ⚠️ **2026-10-27 21:5x 更正（实测证伪初稿归因）**：本文初稿把 D 类归因为「守卫跳过 `.zhiyuan/` 点目录」。用户当日决定清理该目录后实测：**存量红仅 274 → 272（少 2 条）**，D 类绝大多数指向 `ZhiYuan/dsh_plugin/`、`git_save/`、DSH 安装目录等**同样不在扫描白名单里**的真实文件。根因是「裸名不做全盘兜底」的**写法问题**，与点目录无关。原待裁决项「`.zhiyuan/` 是否纳入锚点扫描」**自动消解**。

## 4 未做与理由

1. **不重排分册、不合并文档**：库里 R6/R10 计数全绿、编年卷一~卷五齐全、无同主题重复现行篇 ⇒ 结构没病，重排只会制造断链。
2. **不动 `30_任务卡库` 各卡的 54 条 A 类 + 卡内其它红**：§9「读任意 ns，只写自己的 ns」。已在卡内注明，等各 ns owner 自行处置。
3. **不改 `check_anchors.py`**：D 类的正解是改文档写法（补全路径），若要靠「扩白名单」解则是**放行判据**，须单独裁决并留反向自检，不能在「整理」名义下悄悄做。
4. **`docs/_工具/` 的 18 个一次性脚本保持原位**：查证后确认它们是清理工具链（`cleanup_*.py` 被 8 个脚本硬编码读取，见 `DOC-FIX-20261023` §188）、且已在 `赛事与治理/10` 登记为「不暴露的危险脚本」。**搬走=破坏工具链**，属反整理。

## 5 复跑与回滚

```bash
cd D:/User/Desktop/比赛存档/v1
python docs/_工具/check_numbers.py          # 期望 exit 0
python docs/_工具/check_anchors.py --json   # 期望 exit 0、red_new=0
cd dzrobot_v2 && python tools/wf/check.py   # 期望 GREEN、0 条「登记已失效」
```

回滚：`git checkout -- docs dzrobot_v2/tools/wf/frozen.json`。本轮无 commit、无 push。

## 6 追加：`.zhiyuan/` 清理收尾（2026-10-27 21:4x~21:5x）

用户裁决「`.zhiyuan` 可以清理掉了」。该目录是 2026-10-16 的值守工具残留（8 文件 / 34 KB），个人域原文已于 10-22 按 D06 删除；剩余运维件执行如下：

| 动作 | 结果 |
|---|---|
| 归档（先留证据） | `artifacts/zhiyuan_retired_20261027.tar.gz`（9 条目 / 8.0 KB） |
| 删目录 | `.zhiyuan/` 已移除（`.gitignore:24` 规则保留，防再生成污染） |
| 注销任务计划 | `纸鸢心跳守护`、`纸鸢守护双保险` 已 `Unregister`（此前均 `Disabled`，末次 10-25 16:00 / Result 0）；`DshFileDepWatchdog` 属插件工具**未动** |
| `06_本机值守与守护器.md` | 加「当前状态：未部署」声明；「脚本仍在运行」改为清理事实；两任务标注已注销；自检段加「仅重建后适用」；更新日期 10-22→10-27 |
| `技术全景_E` | §2.7.7 与速查表 4 处标〔已清理 2026-10-27〕；两处带行号锚点（`watchdog.ps1:1-99`〔失效：源已清理，现指归档包〕、`poll.ps1:1-60`〔失效：源已清理，现指归档包〕）改为归档包指向 |
| `团队与分工.md` | doc-ops 目录范围去掉 `.zhiyuan/` |

**门禁影响实测**：删目录后 `check_anchors` 存量红 274→272（`--write-baseline` 已收缩），六项守卫均 exit 0。**同日 21:54:05 并行会话新建 `_tmp_session/DIFF-PURGE-20261027/run_suite.py`〔失效：10-27 22:0x 已改名 `ack_purge_runner.py`〕（第二份同名），使 `agc-1024/AGC-T1-PROGRESS` 的裸名 `run_suite.py`〔失效：同上改名〕由「黄」改判「新增红」——非本卡造成，属他人写域；**2026-10-29 ACK-PURGE 自查已改名，本条红消解**。**
