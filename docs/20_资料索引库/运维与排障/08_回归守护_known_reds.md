---
来源: docs/_工具/check_doc_policy.py 与 check_anchors.py 与 check_numbers.py 的实跑输出、dzrobot_v2/tools/wf/check.py 实跑输出、D:/User/Desktop/ZhiYuan/dsh_plugin 的 admin.mjs verify 与 check-divergence.mjs 实跑输出
取证: 实
更新: 2026-10-29
取代: docs/history_cache/01_旧文档归档/dzrobot_v2/docs/回归守护_known_reds.md（旧文已归档并不再维护；本页是 2026-10-29 重新核实过的现行挂账页，不是旧文搬运）
status: 现行
摘要: 已知红/黄的挂账台账：每条给现象、复跑命令、owner、处置计划、到期。分必须处置与存量不挡收口两类，别把存量当新问题重开。
---

# 回归守护 · known reds（挂账台账）

> **用法**：门禁出现红，先在本页搜现象。**在册 = 已知，不许当新问题重开**；不在册 = 真的新问题。
> **两条纪律**：① 存量在 baseline 内的红/黄**不挡收口**，但必须有人、有到期；② 本页只记"核实过的现状"，不记"据说"——每条都带 2026-10-29 的实跑命令与原文片段。

## 0. 挂账总表（先看这张）

| # | 守卫 | 现象一句话 | 性质 | owner | 到期 |
|---|---|---|---|---|---|
| K1 | check_doc_policy R8 | agc-1026 树里 [卡] LIVE-RUN-20261028 没有对应卡文件 | **必须处置** | agc-1026 / Lead | 2026-10-30 |
| K2 | check_anchors | 新增红 3 条（三张卡里裸写文件名） | **必须处置** | 对应卡 owner（agc-1024 / agc-1026） | 2026-10-30 |
| K3 | check_anchors | 存量红 268 / 黄 503 | 存量·不挡收口 | doc-line | 只减不增 |
| K4 | check_numbers | 已声明异文 2 条（轴距 / 开火保持帧） | 已声明·不挡收口 | 标定 owner | 待复标 |
| K5 | check_numbers | 需人核 7 条（算不了） | 存量·不挡收口 | 各自 owner | 逐条 |
| K6 | 插件仓 admin.mjs verify | dsh-dzairground-ssh 文件数 274≠116 | **必须处置（不洗白）** | 插件 owner | 2026-10-30 |
| K7 | check-divergence.mjs | copy#4（SIM-STUDIO-MERGE）漂移 | 记录·不判失败 | 插件 owner | 2026-11-05 |
| K8 | wf/check.py | robot.yaml planner 段 SCHEMA warn 2 条：**已不存在** | 已消失（见 K8） | — | — |
| K9 | wf/check.py | 存量 tools 未登记 warn 13 | 存量·不挡收口 | tools owner | 只减不增 |

---

## K1 · check_doc_policy R8：树指着一张不存在的卡

**现象**（2026-10-29 实跑原文）：

```
[红线] 必须处置：
  R8  docs/30_任务卡库/在办/agc-1026/00_树.md
       树节点指向的卡不存在：LIVE-RUN-20261028
结论：RED —— 1 条红线。
```

**根因**（读代码 + 读卡面核实，不是猜）：R8 认的是"树里出现的 [卡] XXX 形态"，然后要求 ns 目录下存在 XXX.md（docs/_工具/check_doc_policy.py 的 re.findall 那条正则）。而 00_树.md 的 R-22 节点写的是：

```
- R-22 [卡] LIVE-RUN-20261028 · 实机直跑：… （不另建卡文件）
```

卡面正文按设计**统一写在 _tmp_session/今晚实跑/PLAN.md**，仓内有意不建 LIVE-RUN-20261028.md。所以这是"树语法与设计意图冲突"，**不是文件丢失**。目录里确实没有该文件（ls 该目录共 19 个 .md）。

**复跑**：

```powershell
cd D:\User\Desktop\比赛存档\v1
python docs\_工具\check_doc_policy.py
grep -n 'LIVE-RUN-20261028' docs/30_任务卡库/在办/agc-1026/00_树.md
ls docs/30_任务卡库/在办/agc-1026/
```

**处置计划（二选一，都不许改守卫去迁就文档）**：

1. 建 LIVE-RUN-20261028.md（哪怕只有 front matter + 一句"正文在 PLAN.md"的指针）—— 最省事，且卡面从此可被 ns/agent/树节点 三项检查覆盖；
2. 把树节点改成非卡引用（不再用 [卡] 标记），正文仍在 _tmp_session。

> 二者都要 Lead 动手：docs/30_任务卡库/在办/agc-1026/00_树.md 是**单写点**（ns owner 才许改树），本页只挂账不改树。

## K2 · check_anchors：新增红 3 条（必须处置）

**现象**（2026-10-29 实跑原文）：

```
锚点守卫  扫描 257 篇 / 引用 6735 条（带行号锚点 4094）｜新增红 3（存量红 268）｜黄 503｜行号越界 1｜同名待判 149｜行号无主 34｜外部 104｜已标失效 137

[新增红] 路径全盘不存在，必须处置（改路径 / 标〔失效〕 / 删引用）：
   RED  docs/30_任务卡库/在办/agc-1024/AGC-T1-PROGRESS-20261024.md
        run_suite.py
   RED  docs/30_任务卡库/在办/agc-1026/00_树.md
        tag.txt
   RED  docs/30_任务卡库/在办/agc-1026/RECT-LINK.md
        probes/run_suites.py
```

**根因**：三处都是在卡面里写成了**裸文件名**（没有目录前缀），锚点守卫按"路径全盘不存在"判红。同 ns 上一轮就是这么修的（树里自己记过：改为 _tmp_session/AGC-RECTIFY-20261026/ 下完整相对路径后新增红 6→0）。

**复跑**：

```powershell
cd D:\User\Desktop\比赛存档\v1
python docs\_工具\check_anchors.py
python docs\_工具\check_anchors.py --all
```

**处置**：改卡面路径（推荐）或在该行尾标 〔失效〕。**注意：不是所有守卫红都要修** —— 存量红 268 条已在 baseline 内，只减不增；本次只有这 3 条新增红挡收口。

---

## K3 · check_anchors：存量红 268 / 黄 503（不挡收口）

**现象**：同上首行。存量红 268 条按文档分布（Top）：DOC-FIX-20261023.md 24、技术全景_E 22、2026-10-20.md（按日摘要）16、TRACK-FIELD-D11-20261022.md 15、06_本机值守与守护器.md 13…
黄 503 条按分区：_tmp_session 481 / history_cache 7 / _archive 7 / _work 5 / git_save 3。

**口径**："**存量在 baseline 内，不挡收口**"（AGENTS.md §4 判红清单：锚点守卫**只对新增红判失败**）。黄 = 引用的原物已搬进非现行区，本来就不判失败。

**复跑**：python docs/_工具/check_anchors.py（默认只报统计 + 新增红）、--all 看全集。
**处置**：修一条少一条；**不要**为凑数重建 baseline（重建默认纯收缩，写宽了就是洗白）。

---

## K4 · check_numbers：2 条已声明异文（有意义，不是待修）

**现象**（2026-10-29 实跑原文，退出码 **0**）：

```
文档侧漂移 2 条（分类见下；新增/未声明异文判红，存量进基线）：
  base wheelbase_m: 异文仍存 —— 文档 docs/20_资料索引库/车端/08_车体几何与执行约束.md:26 记 0.15，代码现值 0.157
  base fire_hold_frames: 异文仍存 —— 文档 docs/20_资料索引库/视觉与云台/01_云台瞄准与开火链路.md:77 记 5，代码现值 1
```

**为什么是"已声明"**：wheelbase_m 是用户 10-03 尺量 0.15 与代码固化 0.157 的**有意并列**（D-029：按固化值算、不再重量）；fire_hold_frames 是**仓内 1 / 车端 5** 的第三例"仓内不等于车端"（D-052；10-24 用户令 5→1 未上车）。两条都在 docs/20_资料索引库/数字速查.yaml 里写了异文说明，所以不判红。

**现场含义（必须记住的一条）**：fire_hold_frames 现场必须 ros2 param get 核实，**不许二选一** —— 仓内 1 意味着"首帧满足即开火"，车端 5 才是保真档。

**复跑**：python docs/_工具/check_numbers.py（预期 exit 0；可重算 65 条全 PASS）。
**处置**：fire_hold_frames 待一次部署把 1 推上车后由本页销账；wheelbase_m 待实车复标。

---

## K5 · check_numbers：需人核 7 条（算不了，别当已核）

**现象**：需人核 7 条（算不了，别当已核）：rear_track_outer_m、drone_heartbeat_hz、lost_grace_ms、total_latency_ms、det_node_draw、planner_engine、target_speed_field_mps。
**每条的"为什么算不了"都印在 check_numbers.py 的输出里**（有的全仓 0 命中、有的是布尔/枚举不进数值、有的在冻结区且被在办卡占用）。**不要为了让它们可重算去造锚点常量** —— 那会把"没人验证"伪装成"机器验证"。
**复跑**：同上，看最后一段。
**处置**：逐条落实现场实测出处，或明确写"未核实"。

---

## K6 · 插件仓 admin.mjs verify：1 FAIL（274≠116，明确不洗白）

**现象**（node admin.mjs verify，退出码 **1**）：

```
[PASS] dsh-deepseek-peak-whale  v0.5.5  文件 72/72  依赖 link:（仅符号链接，安全）
[PASS] dsh-our-free-model  v2.5.6  文件 80/80  依赖 link:（仅符号链接，安全）
[FAIL] dsh-dzairground-ssh  v0.7.0-s6  文件 274/116  依赖 link:（仅符号链接，安全）
        - 文件数 274 ≠ 基线 116（若改动合法：verify --update-baseline）
协议：link: 3 个；file: 0 个
结果：2 通过，1 失败
```

**怎么读**：三条依赖全部 link:（协议正确，不是 2026-10-23 那次 file: 事故）；FAIL 的语义是**文件数比基线翻倍**（116→274）。link: 协议 + divergence 判"install copy 与源树 identical" ⇒ 更像**基线过期**（合法新增文件没登记），但仍属"未知差异"，**不许在没看清 158 个新增文件是什么之前 pnpm 一把梭**。

**复跑**：

```powershell
cd D:\User\Desktop\ZhiYuan\dsh_plugin
node admin.mjs verify; echo "RC=$?"
cd dsh-dzairground-ssh; node scripts/check-divergence.mjs; echo "RC=$?"
```

**处置计划**：① 先列 158 个新增文件（git status / 源树对 install copy）；② 全是合法新增（如 novnc 补丁、fixtures）⇒ node admin.mjs verify --update-baseline，并把理由写进插件仓 README；③ 有不明文件 ⇒ 不更新基线，先查来源。**未做 ①② 之前不许跑 --update-baseline**（那就是洗白）。
**影响面**：不影响车端任何门禁；只影响插件管理器"健康"读数。

---

## K7 · 插件仓 check-divergence.mjs：copy#4 漂移（记录，不判失败）

**现象**（node scripts/check-divergence.mjs，退出码 **0**，结论 DIVERGENCE CHECK PASSED）：

```
=== drift summary ===
  in sync  install copy (DSH actually runs this)
  DRIFTED  package copy #4 (SIM-STUDIO-MERGE-20261024)  (plugin-only 106 / copy-only 24 / differing 13)
```

**怎么读**：**正在跑的那份（install copy）与源树 identical**，这是唯一影响 DSH 行为的副本 ⇒ 不判失败。漂移的那份是 _tmp_session 里的 sim_studio 参考包：该包**不是超集**（routeExportPayload / /sim/step-loop 只在插件里），所以**绝不能拿它覆盖插件**（脚本自己会把"overwrite 会删掉的 plugin-only 文件"列出来）。
**处置**：任何"把 sim_studio 包并入插件"的动作，先按 plugin-only 清单反向补齐，再谈覆盖。到期 2026-11-05（或该实验线关闭时）。

---

## K8 · wf/check.py：robot.yaml planner 段 SCHEMA warn 2 条 —— **已不存在**

**核实结论（与预期不符，如实记）**：2026-10-29 实跑 python tools/wf/check.py **结论 GREEN，输出里没有任何 SCHEMA warn**。grep 只在 tools/calib_backfill.py 的说明文字与 docs/wf/frozen.json 的登记理由里命中 SCHEMA，没有"planner 段 SCHEMA warn"这条判据的代码。

**复跑**：

```powershell
cd D:\User\Desktop\比赛存档\v1\dzrobot_v2
python tools\wf\check.py; echo "RC=$?"
```

**处置**：本页**销账**（不存在的东西不挂账）。若明早有人在别的机器上看到这条 warn，说明那是**另一个版本的 check.py**，先比 git status 与文件 sha256，再决定信哪边。

---

## K9 · wf/check.py：存量 tools 未登记 warn 13（不挡收口）

**现象**：[OK ] 复用门禁  新文件: N  未登记: 0  存量 tools 未登记(warn): 13。
**口径**：新增 .py 未登记 = **红**（check_reuse.py 规则 1）；存量 tools 未登记 = **只警告**（规则 2，历史存量）。
**处置**：谁清理谁收口，逐条补进 dzrobot_v2/docs/复用清单.md。**不要**为了消 warn 把存量脚本改名/删除。

---

## K10 · test_mission §18 停站台账红：测试期望 44 点 D3*/G3* vs 现役 config 68 点 F4*/B4*（双向陈旧）

**现象**：`cd dzrobot_v2 && PYTHONPATH=src/dzbrain python -B -X utf8 src/dzbrain/tests/test_mission.py` → §18 两条红（18.0 前置 / 18.2 逐点 kind）。
**证据**（2026-10-29 实测）：`route.tokens` 加载结果 HEAD == 工作区（68 token，SHOOT=F4*/B4*，首点 N10）；测试写死期望 44 点、D3*/G3*、末点 N5。route.tokens 无在途改动（git diff 该键为空）。
**归因**：测试与已提交配置**双向陈旧**——测试按"第七图路线"写（t7 卡时代），config 里仍是更老的 68 点路线；且 68 点里的 F4* 落在第六图雷区（雷含 F4）上，路线本身也不自洽。与 map.yaml 在途删除、与今晚任何卡无关。
**影响**：不在明早彩排路径上（步进链 auto_plan_enabled=true、fallback_to_tokens=false，不走 route.tokens legacy 回退）。
**处置**：P4 或路线卡 owner 二选一——①更新 config/route.tokens 为现行图并同步测试；②测试改为跳过/按现役 config 自适应断言（推荐①，legacy 回退虽关但台账数字会引用）。
**复跑**：`python -B -X utf8 src/dzbrain/tests/test_mission.py 2>&1 | grep -E "^\s*FAIL\s+18"`
**到期**：2026-11-05

---

## K11 · test_mission §14/15 桥洞情报红（K10 同族：config 与测试双向陈旧）

**现象**：`test_mission.py` §14/§15 共 12+3 条红（V2 实测整表 229/246、17 红；换 HEAD config/robot.yaml 复跑**同样 17 红** ⇒ 既有基线）。
**机制**：config/robot.yaml 的 `route.cross_lower_candidates=[K7]`（HEAD==工作区）与测试期望的 K10/K9 兜底口径失配；§15 的改道断言写死 K9→K11。
**处置**：路线/桥洞卡 owner 收口时统一——要么 config 回 K10 系，要么测试改读 config 自适应。到期 2026-11-05。与 K10 合并处置。

---

## K12 · test_corner_cut 5.9 预算红（90/91）

**现象**：`python -B -X utf8 src/dzbrain/tests/test_corner_cut.py` → 5.9「含射击停站总预算 180 s 内」红：sim 141.38 s + 2 停站 ×30 s = 201.38 s > 180 s。
**口径**：10-28 台账已记（pursuit/trajectory 并行候选期遗留）；非任何在卡引入。
**处置**：pursuit/trajectory owner 按新轨迹重算预算或改判据；到期 2026-11-05。

---

## K13 · test_drift 1.2b 横向重锚红（23/24）

**现象**：1.2b reanchored=0 红。把 localization.py 换回 HEAD 即 24/24 ⇒ localization.py 的 F5 在途改动引起，与 MISSION-GATE 无关（V2 置换实验证明）。
**处置**：localization F5 卡 owner 收口；到期见其卡。

---

## K14 · 段完成闸 2 s 逃生门（设计取舍备忘，非缺陷）

**现象**：步进段遭投影持续劫持 >2 s 时，「用时 ≥2.0 s」判据会放行完成（travel 仍 0）。
**口径**：闸的设计目标是消灭 0.02 s 假完成；2 s 门是为合法原地掉头/停顿段留的逃生门（V2 V-13 探针实测 2.1 s 放行）。彻底治投影劫持靠 AGC-T1 几何可信门（_drift_gate_m），不归本闸。
**处置**：不改。若现场出现「holds 后 2 s 假完成」日志链，先查几何可信门，勿再调闸阈值。

---

## 附 · 本页的复跑总命令

```powershell
cd D:\User\Desktop\比赛存档\v1
python docs\_工具\check_doc_policy.py; echo "RC=$?"
python docs\_工具\check_anchors.py; echo "RC=$?"
python docs\_工具\check_numbers.py; echo "RC=$?"
cd dzrobot_v2; python tools\wf\check.py; echo "RC=$?"
cd D:\User\Desktop\ZhiYuan\dsh_plugin; node admin.mjs verify; echo "RC=$?"
cd dsh-dzairground-ssh; node scripts/check-divergence.mjs; echo "RC=$?"
```

> 期望（2026-10-29 状态）：doc_policy RC=1（仅 K1）、anchors RC=1（仅 K2 的三条新增红）、numbers RC=0、check.py RC=0（GREEN）、verify RC=1（仅 K6）、divergence RC=0。
> **除 K1/K2/K6 外任何新红都是新问题**，不要往本页里塞。
