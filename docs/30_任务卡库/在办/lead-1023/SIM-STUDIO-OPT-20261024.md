---
卡号: SIM-STUDIO-OPT-20261024
ns: lead-1023
agent: 待指派（建议 sim-integrator；Lead 不开工，本卡即交接件）
树节点: L-13
status: pending
创建: 2026-10-24
owner: 待指派
写域: D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/**（工作区外·当前不在任何 git 仓内，见 P0-1）；_tmp_session/SIM-STUDIO-OPT-20261024/**（过程稿）；docs/20_资料索引库/空地链路/**（仅 P2-3 新篇 + 00_索引.md）；docs/能力索引.md
只读依赖: dzrobot_v2/tools/plan_route.py、dzrobot_v2/src/dzbrain/dzbrain/tools/sim_cli.py、dzrobot_v2/config/robot.yaml（**只读，禁改**）、dzrobot_v2/src/dzbrain/dzbrain/route/planner.py（**冻结区，禁改**）、dzrobot_v2/artifacts/sim/**（产物区，按 P1-3 规则裁剪）
---

# SIM-STUDIO-OPT-20261024 · 仿真台（sim_studio 移交包）优化方案 · 交接件

> **一句话**：桌面 `sim_studio_优化版_20261024.zip` 是仿真台「界面 + 编排层」的独立可跑版（37 文件），
> 内核仍留 `dzrobot_v2` 命令行。实测结论：**值得融，但不能整包覆盖**；
> 真正的收益不在界面，而在 **P0 三条地基**（版本控制、单一根解析、JSON 契约通道）。
> 用户指令：先不落地。本卡把每一项写到「拿来就能跑、跑就能判红绿」的粒度，供他人执行。
>
> **执行前必读 · 包位置**：本卡所有 `…/_tmp_session/SIM-STUDIO-MERGE-20261024/incoming/sim_studio/…` 引用指向我的**只读探测解压件**，位于过程稿区（`_tmp_session` 属月清对象，可能已被清走）。开卡第一步先确认该目录存在；
> 若不存在，从桌面原件重新解压到本卡写域 `_tmp_session/SIM-STUDIO-OPT-20261024/incoming/sim_studio/`：`D:\User\Desktop\sim_studio_优化版_20261024.zip`（原件实测存在，0.40 MB）。锚点守卫对这类引用判**黄不判红**（Y9），属有意保留。

---

## 0 先读：本轮纠正的三条错误认知

执行者若沿用上一轮的口头结论会踩空，三条必须改写：

1. **插件早已不在 `tools/plugins/`**。提交 `449512d9`「插件迁出仓库：dz 三件套移至 D:/User/Desktop/ZhiYuan/dsh_plugin（由 admin.mjs 统一管理）」；
   实测 `git ls-tree HEAD tools/` 只剩 `aim_autotune/` 与 `pack_handoff.py`，`tools/plugins` 在索引里 **0 文件**，`git check-ignore` 无命中（**不是被忽略，是不存在**）。
   → 任何写 `tools/plugins/dsh-dzairground-ssh/...` 的路径引用**全部失效**（库内现存 8 处，见 P2-2）。
   → 更要紧：`D:/User/Desktop/ZhiYuan/dsh_plugin/` 下**没有 `.git`**，插件源码目前**零版本控制**（这就是 P0-1）。

2. **「移交包不读 `DZROBOT_SIM_ROOT`」是上一轮的错判**。包 `D:/User/Desktop/比赛存档/v1/_tmp_session/SIM-STUDIO-MERGE-20261024/incoming/sim_studio/src/simbench.js` 第 61 行 `process.env.DZROBOT_SIM_ROOT ?? ''` 就是第一优先级。
   E4/F0 判红的**真因**是另外两条：
   - 包内 `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/sim-root.json` 烧死了打包机的路径 `D:/v1/v1/dzrobot_v2`，本机实测**该路径不存在**；
   - `scripts/test-sim-*.mjs` 各自**另抄了一份** `findWorkspaceRoot`（包版：先读指针再向上走 4 级；插件版：先向上走 4 级再读指针），**而这份内联副本不读环境变量**——所以设了 env 仍然 E4 FAIL。
   → 同一仓库里两个同名解析器、语义还不一样，这就是 P0-2 要收的口。

3. **「换夹具图就能过规划闸门」也是错判**。夹具图只在**配 tracker-safe** 时可解；配现役引擎 tour 仍失败（§1 矩阵）。

---

## 1 实测事实表（本卡作者亲跑，逐条可复跑）

| # | 事实 | 复跑命令 / 证据 |
|---|---|---|
| F-1 | 引擎×地图矩阵：**只有 5th×tracker-safe 出路线** | `cd dzrobot_v2 && set PYTHONIOENCODING=utf-8 && python tools/plan_route.py --map <夹具> --engine tracker-safe --emit tokens` → exit 0，`N10 M10 M9 L9 K9 … N5`；6th×tour → 「双射击停站应为 2 个，实际 1 个：B4」；5th×tour → 「…实际 1 个：D3」；6th×tracker-safe → 「tracker-safe 引擎无解」（与 `dzrobot_v2/config/robot.yaml:779` 注释的 K7 legs0=0 一致） |
| F-2 | 现役引擎实跑判红（**插件副本也红**） | `cd D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh && node scripts/test-sim-overlay-fix.mjs` → 末行 `结果：28/30 存在失败 ❌`，红项 `FAIL F1 现役引擎（tour）实跑 exit 0 —— … 双射击停站应为 2 个，实际 1 个：B4`（同文件 `:234` 是该断言） |
| F-3 | 闸门实现是**默认参数**，不是配置项 | `dzrobot_v2/src/dzbrain/dzbrain/route/planner.py:1266` `require_pair: bool = True`；`:1274` 判 `len(names) != 2` 即报「双射击停站应为 2 个」；`:1285` 还要求两站分居板面两侧；`:105` 注明「双射击停站（**对侧互备**）」。全仓 `require_pair` 仅 5 处，**唯一 False 用法在测试里**：`dzrobot_v2/src/dzbrain/tests/test_dual_shoot.py:107` |
| F-4 | 内核有 JSON 契约、**消费端一行没用** | `dzrobot_v2/src/dzbrain/dzbrain/tools/sim_cli.py`：`:97` `add_argument("--json", dest="json_line")`、`:169`/`:284` `print("SIM_RESULT_JSON " + json.dumps(...))`、`:45` 契约版本常量。插件侧 `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/src/simbench.js`/`D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/src/stressbench.js`/`D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/src/dronesim.js`/`D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/index.js` 四文件 `SIM_RESULT_JSON` 与 `metricsFull` 命中**全为 0**，仍在 `src/simbench.js:708 METRIC_LABELS` + `:715 parseMetrics` 逐行啃 stdout（`:976 record.metrics = parseMetrics(stdout)`） |
| F-5 | 旧正则**抓错过指标**（有记录的真事故） | `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/src/simbench.js:701-706` 注释原文：旧正则因 `(真值)` 挡在中间匹配不到真值那行，抓到 `信念↔真值RMS` 的 6 位小数 ——「**错的数比没有数更糟**」 |
| F-6 | 包侧已有**正确姿势**可抄 | 包 `D:/User/Desktop/比赛存档/v1/_tmp_session/SIM-STUDIO-MERGE-20261024/incoming/sim_studio/src/simbench.js:1047` `simJsonCache = /add_argument\(\s*["']--json["']/.test(fs.readFileSync(SIM_CLI, 'utf8'))`（从 argparse 真源探测；`:1039` 注明「旧内核硬传会被 argparse exit 2，**仿真根本不跑**」）→ `:1072` `if (simJsonSupported()) argv.push('--json')` → `:1120-1122` 取末行解析 62 键 → `:1466` 台账记是否走契约通道 |
| F-7 | 产物**无保留策略** | `dzrobot_v2/artifacts/sim` 实测 **234 MB**：`stress/` **194 MB / 72 项**、`drone/` **15 MB / 176 项**、顶层另有 30 个 `1792*_smoke/field7/sixth/merge-probe/mutex_a` run 目录；上述四文件 `prune\|retention\|cleanup` **0 命中** |
| F-8 | 情报**写侧从未被真实跑证明过** | `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/src/dronesim.js:375` 写 `SIM_ROOT/artifacts/sim/drone/<id>/intel.json`、`latestIntel`（`:404` 注释「数据源 = 每次跑落的 sidecar」，`:415` 读它）；实测 176 个 drone run 目录**含 intel.json = 0 个**，整个 `D:/User/Desktop/比赛存档/v1/dzrobot_v2/artifacts/sim`（深 3）**intel.json = 0 个**。`D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/scripts/test-intel-route.mjs` 16/16 全绿只覆盖**读侧与三态契约** |
| F-9 | 包**不是超集**，整包覆盖即瘫 | 包缺 `routeExportPayload`（插件 `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/src/simbench.js` 1 处 + `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/index.js` 2 处）、缺 `latestIntel`（`D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/src/dronesim.js:1` + `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/index.js` 2 处：`index.js:477 GET /intel/latest`、`:481 GET /sim/route-export`）；插件**无 `/sim/scout`、`/sim/step-loop`** 两条路由（页面要）；包 `D:/User/Desktop/比赛存档/v1/_tmp_session/SIM-STUDIO-MERGE-20261024/incoming/sim_studio/sim.html:511` 仍是 `var API = '/api/dzrobot-ssh'`（`:510` 注释也写着旧前缀），现役挂载是 `/api/dzairground-ssh`（插件 `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/client.js`/`D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/sim.html` 已正确） |
| F-10 | 分叉**已是事实**，不是风险 | 6 个同名测试逐文件哈希全不同；包独有 14 测试（`test-sim-scout / stress-plan / stress-viz / plan-cache / metrics-table / segment-table / report-card / compare / history-409 / readtail / server-routes / page-e2e / field7 / sixth-map`）+ `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/scripts/fixtures/`（`map_fifth_solvable.yaml`、`solvable-map.mjs`、`traj_sample.csv`、`by_segment_sample.csv`） |
| F-11 | 包测试**在本机可跑**，非全红 | 我实跑：13/20 通过；`D:/User/Desktop/比赛存档/v1/_tmp_session/SIM-STUDIO-MERGE-20261024/incoming/sim_studio/scripts/test-sim-stress-plan.mjs` = 24/27（唯一红 B5「至少 1 张规划成功」= 0，成因即 F-1/F-3）；`D:/User/Desktop/比赛存档/v1/_tmp_session/SIM-STUDIO-MERGE-20261024/incoming/sim_studio/scripts/test-sim-stress-viz.mjs` **19/19 ✅**；E4/F0 两条红的成因见 §0-2（**与内核无关**：插件副本 E4 是 PASS） |
| F-12 | 行尾噪声源头 | v1 根 `.gitattributes` 全文只有注释 + `* text=auto`；插件 `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/sim.html` 实测 140,984 B / 2,772 换行中 **2,767 为 CRLF** → 历史「4587 行假 diff」的成因（真实改动 `--ignore-all-space` 下 +9/-82） |
| F-13 | 装载与一致性**当前是好的** | 上一轮 `node admin.mjs verify` → 3 通过 / 0 失败，`dsh-dzairground-ssh v0.7.0-s6 · 116/116 文件 · link:`；`D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/scripts/test-package-files.mjs` 通过（安装副本 == 源树）。**开卡第一步请复跑作基线快照。** |
| F-14 | 真源指针现状 | 插件 `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/sim-root.json` 指向 `D:\User\Desktop\比赛存档\v1\dzrobot_v2`（**有效**，note 记「t24 后裁决：比赛存档\v1\dzrobot_v2 为权威真源」）；包内同一文件指向死路径（§0-2）。`D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/src/simbench.js:54-72` 是解析顺序与 `SIM_ROOT_SOURCE` 诊断键的出处 |
| F-15 | 包页面能力**实跑成功过** | 包结构 + 本机内核起独立宿主真跑：`status=done · 0.897s · finished=true time_s=118.12 rms=0.1187 oob=74 len_ratio=0.9332 · configMd5=ed976a24 mapMd5=2d2f658d`；`#compare=` / `#view=` 深链可用，62 键指标弹窗可出（产物在 `_tmp_session/SIM-STUDIO-MERGE-20261024/`） |

---

## 2 读过的台账条目（§2 必填）

| 条目 | 与本卡的关系 |
|---|---|
| **D-018**（求解率与失败结构：tour 28/64=43.75%，失败主因**结构约束**不是 DP） | F-1/F-3 就是它说的「结构约束」在 10-24 的具体形态；**不得靠调 DP 参数绕开** |
| **D-023**（全量回归 37 套 / 13 红处置） | 现状以「基线即有红」为准；F-2 不许算成本卡新引入的退化 |
| **D-030**（靶不翻面、圆面；红蓝门控 = 保险闸） | 决定「双站」语义：两站是**对侧互备**（`planner.py:105`），不是两块独立靶 |
| **D-031 → D-033**（靶速改判 0.3 m/s；0.70 m 行程 ⇒ 相邻两格都需可打停站） | `require_pair` 的业务根因；也是「第二站由规划器搜出来还是由现场路线给」的裁决依据 |
| **D-035 / D-036 / D-037**（sim_cli `--json` / preflight / AGC2 三态契约） | P0-3 直接沿用 D-035 已裁的契约形态，不新造字段 |
| `docs/20_资料索引库/车端/03_规划器与自动规划.md` | 已写「求解率卡在结构约束不是 DP；`auto_plan_enabled: false`（D-011 执行层 pilot 未全绿）」——**这就是车上看不见此缺陷的原因** |
| `docs/20_资料索引库/车端/06_回归与测试基线.md` | 既有 3 类红（D3 / C9 / A3）；F-2 属同族，登记时归 D3 族 |
| `docs/20_资料索引库/空地链路/00_索引.md` | v2 步进 10-18 审计 **NO-GO**；新增 `/sim/step-loop`（P3-1）不得被描述成「链路已通」 |
| 卡 `TOOLS-SIM-STUDIO-20261023`（已归档，`归档/lead-1023/2026-10-23_TOOLS-SIM-STUDIO-20261023.md`） | 母卡：L-3.1 已裁「sim_cli `--json` 62 键 + exit2 契约」；本卡是**它的消费端落地**，不是翻案 |
| 卡 `PLUGIN-AGC2-20261024`（已归档，L-8.1/L-8.3） | `latestIntel`（B1 UI 先行）与 `/sim/route-export`（C1 半步版）的出处；F-9 的「包不是超集」以它为准 |

**翻案三问自检**：新证据 = F-1 矩阵 + F-4 零引用 + F-8 零 sidecar（均本轮实测）；影响面 = 只影响**离线评估能力**，不影响比赛现场（现场跑手动路线、`auto_plan_enabled: false`）；不重开的代价 = 仿真台所有重规划路径持续不可用。**→ 属新工作，不属翻案。**

---

## 3 复用检索结果（§2 必填）

- `dzrobot_v2/docs/复用清单.md` + `docs/能力索引.md` 检索「仿真 / sim / 仿真台」：能力索引**没有「仿真台（界面+编排层）」这一条能力** → 这正是第 4 份副本（本 zip）长出来的原因，见 P2-2 的登记动作。
- **优先复用，别新写**：
  - `plan_route.py` 已有 `--map / --engine / --json / --check / --emit tokens / --cross-lower / --cross-upper / --from-cell / --already-covered`（头 80 行文档即契约）→ **夹具图 + 显式引擎这条今天就通**，不需要任何新工具。
  - `dzrobot_v2/tools/scout_sim.py` 已存在 → `/sim/scout` 的内核侧不缺。
  - 包 `D:/User/Desktop/比赛存档/v1/_tmp_session/SIM-STUDIO-MERGE-20261024/incoming/sim_studio/src/simbench.js` 的 `simJsonSupported()` / 末行 `SIM_RESULT_JSON` 解析 / 台账「契约通道」三件套 → P0-3 **照搬**。
  - 包 `D:/User/Desktop/比赛存档/v1/_tmp_session/SIM-STUDIO-MERGE-20261024/incoming/sim_studio/scripts/fixtures/map_fifth_solvable.yaml` + `D:/User/Desktop/比赛存档/v1/_tmp_session/SIM-STUDIO-MERGE-20261024/incoming/sim_studio/scripts/fixtures/solvable-map.mjs`（自带「非现场真源、运行时零引用」声明，几何 = 历史第五张图 44 token / route_check 0 error）→ P1-1 夹具直接用它，**别再造第三份图**。
  - 插件 `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/scripts/check-divergence.mjs` 已存在 → P2-2 只扩扫描路径，不写新比对脚本。
  - 插件 `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/sim-root.json` 指针机制已存在 → P0-2 只统一实现，不新增机制。
- **新增轮子及理由**（本卡全部工作只新增 1 个）：`D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/src/workspace-root.js`（单一根解析器）。理由：同一语义现有 3 份实现（插件 `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/src/simbench.js:54-72`、包 `D:/User/Desktop/比赛存档/v1/_tmp_session/SIM-STUDIO-MERGE-20261024/incoming/sim_studio/src/simbench.js`、`D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/scripts/` 与 `D:/User/Desktop/比赛存档/v1/_tmp_session/SIM-STUDIO-MERGE-20261024/incoming/sim_studio/scripts/` 里测试各自的内联副本），且**语义互相不一致**（是否读 env、指针与向上走的先后相反）→ 摆放位置一变就给出不同答案，这是「测试红得不讲道理」的根。收成一个导出函数是唯一收敛方向。

---

## 4 工作分解（P0 → P3；每项可独立收口）

### P0-1 把插件源码纳入版本控制 · **最优先，约 10 分钟**

- **做什么**：让 `D:/User/Desktop/ZhiYuan/dsh_plugin/`（至少 `dsh-dzairground-ssh/`）进 git，先写 `.gitignore`（排除 `node_modules`、`_backup`、`_archive`、`_tmp`、`_verify`）；入库前留 `admin.mjs verify` 输出 + 逐文件 sha256 清单作基线。远端与分支按 §9 第 2 问裁决（**不许自行假定推 main**）。
- **为什么**：插件是**现役交付物**（desktop profile 靠 `link:` 指向它）却**零提交历史** —— 没有 diff、没有回滚点，AGENTS.md §11「完成一项任务 → 提交 → 双推」对插件工作**直接落空**。F-9 那种「包缺两个函数」级别的事故全靠 diff 才抓得到；本次迁移（`449512d9`）把源码移出仓库后，这条缺口是新出现的。
- **验收**：`git -C <插件目录> log --oneline -1` 有首个提交；`git status --porcelain` 干净；改一个字节能出 `git diff`；`admin.mjs verify` 仍 3/0。
- **边界**：**不许**动 `D:/User/Desktop/ZhiYuan/dsh_plugin/admin.mjs` 的 `link:` 结构；**不许**把 `node_modules` 塞进仓；**不许**提交任何凭据（`D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/scripts/check-ssh.mjs` 附近注意私钥与 `~/.ssh` 内容）。

### P0-2 单一工作区根解析器 · 约 30 分钟

- **做什么**：新建 `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/src/workspace-root.js`，导出 `resolveSimRoot({ env, pluginDir, pointerFile })`，固定优先级 `DZROBOT_SIM_ROOT` → `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/sim-root.json` 指针 → 向上走 4 级；判据统一为「`src/dzbrain` 且 `config/robot.yaml` 都在」。`D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/src/simbench.js`、`D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/src/stressbench.js`、`D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/src/dronesim.js` 与**全部** `scripts/test-sim-*.mjs` 改为 import 它，删内联副本。解析失败时**打印指针内容与尝试过的候选路径**；把包 `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/sim-root.json` 的死路径 `D:/v1/v1/dzrobot_v2` 校正为本机真源。
- **为什么**：§0-2 + F-11。E4/F0 与内核无关（插件副本 E4 PASS），纯粹是两份解析器语义漂移 + 包里烧了别人机器的绝对路径。不收这个口，下次换目录/换机器还会红一整批，每次都要人重查根因。
- **验收**：
  ```
  node scripts/test-sim-overlay-fix.mjs                     # E4 与 F0 必须 PASS（F1 允许仍红 → P1-1）
  node scripts/test-sim-parity.mjs                          # 不回归
  DZROBOT_SIM_ROOT="" node scripts/test-sim-overlay-fix.mjs # 无 env 时靠指针，仍 PASS
  node scripts/test-package-files.mjs                       # 安装副本已同步
  ```
- **边界**：解析器**只读**，任何路径都不许创建目录；不许把内核路径写回 `config/`。

### P0-3 消费端切 JSON 契约通道 · 约 40 分钟（**本卡收益最大**）

- **做什么**：把包三件套移植进插件 `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/src/simbench.js` —— ① 读 `sim_cli.py` 源码探测是否声明 `--json`（缓存结果）；② 声明了才 `argv.push('--json')`；③ 取 stdout 末行 `SIM_RESULT_JSON` 解析 62 键。探测失败或旧内核**回落现有 `METRIC_LABELS`/`parseMetrics` 五键**，并在 `run_meta.jsonl` 记 `simJson: true/false`。`D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/stressbench.js`/`D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/dronesim.js` 同一套。前端 62 键表随 P3-1 进来。
- **为什么**：F-4/F-5/F-6。现在每个数都靠正则从人类排版里抠，内核改一次措辞就**静默给错数**，而 F-5 记录的是**已经发生过一次**（横向 RMS 抓成信念↔真值 RMS）。`--json` 早在 D-035 就裁了、代码在 `sim_cli.py:97/169/284`，**窟窿在消费端**，包作者已把正确姿势写完。
- **验收**：真跑一次比较两条通道同键同值 ——
  ```
  node scripts/test-sim-map-render.mjs && node scripts/test-sim-parity.mjs
  # 台账须出现 simJson:true，且 rms_m 与 62 键里的真值横向 RMS 逐位一致
  ```
  再把 `SIM_ROOT` 指到一份**无 `--json`** 的旧内核副本（可用 `_archive` 里的旧版），验证 `simJson:false` 回落且**不崩**。
- **边界**：**不改** `sim_cli.py`（冻结区）；不加 `--json` 之外的新内核参数；探测不确定时**不许硬传**（旧内核 argparse exit 2，包 `:1039` 已注明这条实测教训）。

### P1-1 离线重规划闸门：先给绕行口径，改判走台账（**不许动内核**）

- **现状（F-1/F-2/F-3）**：`require_pair` 默认 True，而 **tour 对第六图与第五图夹具都只找到一个停站** ⇒ 结构自检否决整条路线 ⇒ 页面 `planRoute`、增量续算透传、`truthPlanEvidence` 同引擎对账、压测 plan 模式、规划缓存**全废**；现场不受影响（手动 68 token 路线 `route_check` 全通过，`auto_plan_enabled: false`）。
- **今天可用的绕行口径（零冻结区改动）**：离线/what-if 一律**显式** `--engine tracker-safe --map <夹具>`（实测 exit 0 出 44 token），**不读** `robot.yaml:749` 的 `engine: tour`。据此把 F1 断言拆两条：「**夹具×可解引擎必须 exit 0**」（能力守卫，现在就该绿）+「**现役引擎×现网图**」（现状记录项，允许红，卡内写明原因与台账指向）。
- **为什么这么切**：F1 现在把「现场不自动规划」这个**有意决策**（D-011）报成仿真台缺陷，每次都要人重新解释一遍；拆开后能力回归与现状红不再混在一起。
- **三条待裁决路线（见 §9 第 1 问，执行者不要替用户选）**：(a) `plan_route.py` 新增显式 `--require-pair=false` 旗标（不动 planner 默认，属 `tools/` 非冻结区）；(b) 给 tour 补第二停站搜索（**冻结区 `planner.py`**，须走解冻登记）；(c) 维持现状，只把「夹具×tracker-safe」定为离线口径。
- **验收**：`D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/scripts/test-sim-overlay-fix.mjs` 两条 F1 语义分明（一绿 + 一标注现状红）；`D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/scripts/test-sim-stress-plan.mjs` 的 `B5` 转绿；卡内留 `plan_route.py --engine tracker-safe --map <夹具>` 全输出。
- **边界**：**严禁**为绿灯改 `planner.py:1266` 默认值；**严禁**改 `config/robot.yaml` 的 `engine:` / `auto_plan_enabled:`（§3 冻结区）；`route_check.py` 规则不复制。

### P1-2 情报写侧取证 · 约 20 分钟

- **做什么**：真跑一次 `runDroneSim`，确认 `artifacts/sim/drone/<id>/intel.json` **落盘**；加断言：sidecar 三类报文（bridge/target/field）全量存在，且条数与 `run_meta.jsonl` 摘要行 `intel:[{half,block,decision,checkpoint}]` 一致；`latestIntel()` 读回后 `source==='sim'` 且 `ageS` 合理。
- **为什么**：F-8 是硬事实——**176 个 run 目录 0 个 intel.json、全 `D:/User/Desktop/比赛存档/v1/dzrobot_v2/artifacts/sim` 0 个**，而 `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/scripts/test-intel-route.mjs` 16/16 全绿。写侧分支从来没被任何一次真实运行执行过。「读侧绿、写侧从没跑过」正是**假绿**的教科书形状，而假绿是整合方案 D1 点名要防的病。
- **验收**：`ls artifacts/sim/drone/<新id>/intel.json` 存在；`D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/scripts/test-intel-route.mjs` 从 16 条增至 ≥19 条且含**写侧**断言；若确实不可达 → 在卡里写「写侧不可达 + 原因」，并把 `latestIntel` 的契约文档改为「目前恒 `none`，链路侧未验证」。**不许留一条没人验证的假通道。**
- **边界**：不许伪造 `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/intel.json` 当测试数据（那是把假绿搬进产物区）；夹具数据只进 `_tmp_session`。

### P1-3 产物保留策略 · 约 30 分钟

- **做什么**：给 `D:/User/Desktop/比赛存档/v1/dzrobot_v2/artifacts/sim/` 定保留规则（例：`stress/` 保最近 N 批 + 被 label/pinned 标记的；`drone/` 保最近 M 个；顶层裸 `*_smoke` 目录按日期清），实现为**先 dry-run 出清单**（大小 + mtime + 是否被 `run_meta.jsonl` 引用），确认后再执行；`run_meta.jsonl` 与产物**同步裁**，不留指向已删目录的行。
- **为什么**：F-7 —— 234 MB 且四文件零 prune/retention/cleanup，只增不减；压测一轮几十 MB，赛前几周会把磁盘吃穿，还会让「找那张图」变成翻垃圾。
- **验收**：dry-run 清单进 `_tmp_session/SIM-STUDIO-OPT-20261024/`；执行后 `D:/User/Desktop/比赛存档/v1/dzrobot_v2/artifacts/sim` ≤ 目标值（建议 ≤80 MB）；`run_meta.jsonl` 无悬空引用（写条断言）；被保留的 run 仍能打开产物页。
- **边界**：**不许**删有裁决价值的证据 run（F-15 那次真跑、双站/禁火相关）；`dzrobot_v2/artifacts` 里非 sim 的 `t1_/t3_` 截图与 `D:/User/Desktop/比赛存档/v1/dzrobot_v2/artifacts/tracking_readonly_20261020.json` **不属本卡**。

### P2-1 行尾与 diff 噪声治理 · 约 15 分钟

- **做什么**：P0-1 入库后给插件仓写 `.gitattributes`，按扩展名钉死（`*.js`/`*.mjs`/`*.html`/`*.json`/`*.sh` 统一 `text eol=lf`），一次性归一并**单独一个「纯行尾」提交**；v1 根 `.gitattributes`（F-12：现在只有 `* text=auto`）同步补规则；README/贡献说明加一句：**看插件 diff 一律 `git diff --ignore-all-space`**。
- **为什么**：F-12 —— 历史上「4587 行假 diff」真实发生并浪费一次人工核对；另有一次用 PowerShell `Set-Content` 改 JS 把多行压成一行（注释吞掉代码，靠 `node --check` 才抓住）。两者都**机器可防**。
- **验收**：`git check-attr text eol -- src/simbench.js` 显示规则生效；归一提交后任意 JS 改动的 `git diff --stat` 只显示真实行；全部测试仍绿。
- **边界**：行尾归一**必须独立成提交**；**不许**用 PowerShell `Set-Content`/`Out-File` 写 JS/HTML（一律 `edit`/`write` 或 node 脚本）。

### P2-2 副本治理与能力登记 · 约 30 分钟

- **做什么**：
  1. `docs/能力索引.md` 新增两行：**「仿真台（界面 + 编排层）→ 唯一权威实现 = `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/sim.html` + `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/src/simbench.js`/`D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/stressbench.js`/`D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/dronesim.js`，owner = lead-1023」**；「**仿真内核 → `dzrobot_v2/src/dzbrain/dzbrain/tools/sim_cli.py` + `dzrobot_v2/tools/plan_route.py`，owner = 车端**」。
  2. `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/scripts/check-divergence.mjs` 扫描范围加上 `_tmp_session/SIM-STUDIO-MERGE-20261024/incoming/sim_studio/`（第 4 副本）与桌面 zip 的登记路径，输出「哪几份漂了」。
  3. 库内 8 处 `tools/plugins/dsh-dz*` 死引用（F-1）：`归档/tools-1022/2026-10-23_TOOLS-PLUGIN-TERMGATE-20261023`（:22/:37/:46/:63 共 4 处）、`在办/tools-1022/TOOLS-PLUGIN-REWORK-20261022`（:17 写域）、`docs/_工具/anchor_baseline.json`(:246)、`docs/_工具/doc_policy_baseline.json`(:71/:76)。归档卡加「路径已迁，现址 D:/...」一行**不改正文结论**；在办卡的写域**由该 ns owner 改**。
- **为什么**：AGENTS.md §3「同一能力不建第二份」；F-10 的六文件哈希全不同、F-1 的 8 处死引用，都是**没有登记入口**的直接后果。不登记，下一份副本还会来。
- **验收**：`D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/check-divergence.mjs` 能列出第 4 副本与权威实现的差异；能力索引两行存在；`python docs/_工具/check_doc_policy.py` 不因本动作判红。
- **边界**：**§9.1 铁律 —— 只写自己的 ns**。`在办/tools-1022/` 卡面写域属 tools-1022 owner，本卡只在此**提出请求**（由 Lead 转达/落树），不许跨 ns 改别人的卡。

### P2-3 包内长期文档提炼入库 · 约 40 分钟（动文档库，需 Lead 排期）

- **做什么**：包里三份长期文档现在躺在 `_tmp_session`（豁免区，§8.1 违规风险；桌面另有 zip 原件）。按 §8.2 三步处理：
  - `CONTRACT.md`（62 键清单、`--json` 契约、HTTP 语义、`_pinnedCommit` 版本钉扎）+ `OPTIMIZATION.md` + `CHANGELOG-20261024.md` → **提炼为一篇** `docs/20_资料索引库/空地链路/05_仿真台契约与版本钉扎_20261024.md`（`status: 现行`，front matter 写「来源: sim_studio 移交包三篇 + 本卡实测 F-1~F-15」）；
  - 在 `空地链路/00_索引.md` §1 加一行（标题 / 一句话结论 / 关键数字 / 来源），TL;DR 补「仿真台 = 界面+编排层在插件、内核在 `dzrobot_v2`；两代 API 前缀并存」；
  - 跑 `python docs/_工具/gen_index.py --ns lead-1023`（**只用 `--ns`**，§9.8：全量重扫会把别人未入库的文档卷进索引）；`20_资料索引库/README.md` 的「应为 N 篇」同步（R6）。
  - 原文留 `_tmp_session/.../sim_studio/`（过程稿），库内写「原文位置」。
- **为什么**：§8.1「长期文档只能存 `docs/` 一处」+ §8.10「在 `_tmp_session` 写报告当交付」；而 `docs/` 全文检索 sim_studio 只有 4 处提及、**无一篇是契约** —— F-10 的漂移正因为没有一份被索引的契约。
- **验收**：`check_doc_policy.py` 红线 0；该篇出现在 `目录索引.md`；`check_links.py` 0 断。
- **边界**：**不许**把三篇原文整篇搬进库（必须先提炼 + 写结论，§8.3）；**不许**手改 `目录索引.md/json`。

### P3-1 合并落地（真要做时按 6 步，顺序不可换）

> 前置：P0-1/P0-2/P0-3 完成，§9 第 1、2 问有结论。**用户当前指令是先不落地**，本节只作可执行脚本留存。

1. 以**插件版为基底**（保留 `routeExportPayload` / `latestIntel` / `/sim/route-export` / `/intel/latest` —— F-9）；
2. 套包的 `D:/User/Desktop/比赛存档/v1/_tmp_session/SIM-STUDIO-MERGE-20261024/incoming/sim_studio/sim.html` + `D:/User/Desktop/比赛存档/v1/_tmp_session/SIM-STUDIO-MERGE-20261024/incoming/sim_studio/src/simbench.js`（含 62 键与台账尾部 64 KB 倒读）+ `D:/User/Desktop/比赛存档/v1/_tmp_session/SIM-STUDIO-MERGE-20261024/incoming/sim_studio/src/stressbench.js` + `D:/User/Desktop/比赛存档/v1/_tmp_session/SIM-STUDIO-MERGE-20261024/incoming/sim_studio/src/dronesim.js`；
3. 回填两个缺失函数：`routeExportPayload`（33 行）+ `latestIntel`（24 行）到合并后的文件（**照插件实现搬，不重写**）；
4. 加 `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/src/scoutsim.js` + `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/index.js` 两条路由 `/sim/scout`、`/sim/step-loop`（内核侧 `dzrobot_v2/tools/scout_sim.py` 已在）；
5. 改前缀：包 `D:/User/Desktop/比赛存档/v1/_tmp_session/SIM-STUDIO-MERGE-20261024/incoming/sim_studio/sim.html:511` `/api/dzrobot-ssh` → `/api/dzairground-ssh`（连同 `:510` 注释一起改）；
6. 测试收口：包独有 14 测试（源在 `D:/User/Desktop/比赛存档/v1/_tmp_session/SIM-STUDIO-MERGE-20261024/incoming/sim_studio/scripts/`）进 `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/scripts/` 并在 `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/scripts/test-package-files.mjs` 清单登记 —— 此时它们才有宿主可跑（F-10 的「现在搬必全红」届时解除）。

**全程对账**：`git diff --ignore-all-space --stat` 记真实改动量（这包全 CRLF，直接 diff 会虚高，F-12）；逐文件 sha256 前后清单；`admin.mjs verify` 收尾。
**不许**：整包覆盖（F-9 四硬阻塞）；在 P0/P1 完成前把 14 测试搬进插件。

### P3-2 前端可见性节流 · 约 15 分钟（**先量化再改**）

- **做什么**：页面与侧栏定时器加 `document.hidden` 守卫 —— 包 `D:/User/Desktop/比赛存档/v1/_tmp_session/SIM-STUDIO-MERGE-20261024/incoming/sim_studio/sim.html` 实测 `visibilitychange` **0 命中**、`stressPoll` 4 处（压测态 1500 ms 轮询后台照打）；插件 `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/client.js` 有 `visibilitychange` 2 处 + `setInterval` 3 处（1000 ms 指标 / 5000 ms），先确认是否已在守卫内。合并后统一为「隐藏时暂停轮询，回前台立即刷一次」。
- **诚实口径**：这条**我没做成本量化** —— 只有代码计数，没测 CPU/流量。动手前先用一次 `stress` 真跑量一轮耗时；收益不明显就只登记不改（别为改而改）。

---

## 5 明确不做（防开倒车，AGENTS.md §6）

- 不改 `planner.py:1266` 的 `require_pair` 默认、不改 `check_shoot_stops` 判据（**冻结区**，要改走解冻登记 + §9 裁决）。
- 不改 `config/robot.yaml` 的 `engine:` / `auto_plan_enabled:`（**冻结区**；`:749`/`:779` 都有裁决注释，动它属翻案）。
- 不整包覆盖 `D:/User/Desktop/比赛存档/v1/_tmp_session/SIM-STUDIO-MERGE-20261024/incoming/sim_studio/sim.html`（F-9）；不在 P0/P1 前搬包独有测试（F-10）。
- 不为「绿」删掉或 `skip` F1 断言（那是把已知缺陷改成不可见）。
- 不新建 `sim2/`、`sim_studio/` 之类第 5 副本；不在代码仓里写 README/报告（§8.10）。
- 不碰小车、不碰无人机（本卡全离线）；`/sim/step-loop` 不得写成「链路已通」（v2 10-18 审计 NO-GO）。

---

## 6 验收命令（卡关闭前逐条跑，输出贴进本卡）

```
# 0) 基线快照（开工第一件事）
cd D:/User/Desktop/ZhiYuan/dsh_plugin && node admin.mjs verify

# 1) 插件离线测试全集合
cd D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh
for t in test-client test-host test-package-files test-preflight test-intel-route test-sim-parity test-sim-map-render test-sim-overlay-fix test-sim-overlay-ab test-sim-incremental test-sim-drone-render; do node scripts/$t.mjs > /dev/null 2>&1; echo "$t exit=$?"; done

# 2) P0-3 契约通道证据（须见 simJson:true + 62 键）
node scripts/test-sim-map-render.mjs

# 3) P1-1 离线能力（须 exit 0 且出 token）
cd D:/User/Desktop/比赛存档/v1/dzrobot_v2 && set PYTHONIOENCODING=utf-8 && python tools/plan_route.py --map "<…>/sim_studio/scripts/fixtures/map_fifth_solvable.yaml" --engine tracker-safe --emit tokens

# 4) 收口门禁（§4 / §8.6 顺序）
cd D:/User/Desktop/比赛存档/v1
python docs/_工具/gen_index.py --ns lead-1023
python docs/_工具/check_doc_policy.py
python docs/_工具/check_anchors.py --ns lead-1023
python docs/_工具/check_numbers.py
cd dzrobot_v2 && python tools/wf/check.py
```

**判定口径**：既有红（D-023 那 13 套族、F-2 的 F1 现状项）允许保留但须在卡里写明「既有/现状」；**新引入红 = 0** 才算过。

---

## 7 回滚点

- P0-1 之前：`admin.mjs verify` 输出 + 逐文件 sha256 清单（`_tmp_session/SIM-STUDIO-OPT-20261024/baseline/`）。
- 每个 P 一个独立提交（P0-1 入库后天然可 `git revert`）；行尾归一单独提交（P2-1）。
- 产物裁剪前：dry-run 清单留档 + zip 到 `_archive/sim_pruned_20261024/`（F-15 那次真跑 run 目录**列入保留白名单**）。
- 测试脚本改造前留 `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/scripts/test-sim-overlay-fix.mjs.orig`。
- 文档入库前跑 `python docs/_工具/snapshot_docs.py`（§8.9 第 5 件）。

---

## 8 影响面

- **用户可见**：指标 5 键 → 62 键 + 逐段峰值；规划类操作从「必失败」→「夹具×tracker-safe 可跑」；产物不再无限膨胀；插件改动可回滚（P0-1）。
- **不受影响**：车端运行、`config/`、`D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/src/dzbrain/`、机端、部署链路（`link:` 结构不变）。
- **依赖关系**：P3-1 依赖 P0-1/2/3 + §9 裁决；P2-2 的 ③ 需 tools-1022 owner 配合；P1-1 若走 (b) 属冻结区解冻，按 §4 登记 `tools/wf/frozen.json` 的 in_flight + 解冻日期。
- **门禁**：只碰 `_tmp_session/`、`artifacts/`、`docs/`（P2-3）三处 + 插件目录本身；`check.py` 冻结区项应保持不变。

---

## 9 待裁决 / 待确认（执行者**不要自行选**，回报 Lead 转用户）

1. **离线规划闸门走 (a)(b)(c) 哪条**？（P1-1）我倾向「绕行口径 + F1 拆两条」先做，(a)(b) 作为独立裁决 —— 这样**不碰冻结区就把离线评估能力恢复**。
2. **插件是否入 git、入哪个仓、推哪个分支**？（P0-1 涉及 §11 双远端；插件在 v1 工作区之外，远端与分支约定需用户确认。）
3. **`latestIntel` 契约口径**：若 P1-2 证明写侧不可达，是否接受在 README/契约文档把它降级为「目前恒 `none`，链路侧未验证」？
4. **第 4 份副本（桌面 zip）处置**：只读留档进 `_archive/`，还是允许执行者当参考实现直接改？
5. `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/check-divergence.mjs` 是否把桌面 zip 与 `_tmp_session` 解压件**纳入常规守卫**（每次 verify 顺带跑）？

---

## 10 文档变更（§8.8 预告 · 收口时按实填）

- **新增**：`docs/20_资料索引库/空地链路/05_仿真台契约与版本钉扎_20261024.md`（P2-3）。
- **修改**：`docs/能力索引.md`（+2 条能力行，P2-2）；`docs/20_资料索引库/空地链路/00_索引.md`（§1 加行 + TL;DR 补一句）；`docs/20_资料索引库/README.md`（R6 篇数）；`归档/tools-1022/2026-10-23_TOOLS-PLUGIN-TERMGATE-20261023.md`（加「路径已迁」一行，需该 ns owner 同意）；插件 `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/README.md`（367 行版，补「diff 一律 `--ignore-all-space`」与两代 API 前缀说明）。
- **重跑**：`gen_index.py --ns lead-1023` → `目录索引.md/json`（自动生成，勿手改）。
- **不新建**：任何 `docs2/`、`sim_studio/` 平行目录；不把包三篇原文整篇入库。

---

## 附：本卡证据位置

- 解压包：`_tmp_session/SIM-STUDIO-MERGE-20261024/incoming/sim_studio/`（含 `CONTRACT.md` / `OPTIMIZATION.md` / `CHANGELOG-20261024.md` / `D:/User/Desktop/比赛存档/v1/_tmp_session/SIM-STUDIO-MERGE-20261024/incoming/sim_studio/scripts/fixtures/`）
- 独立宿主对照：`_tmp_session/SIM-STUDIO-MERGE-20261024/baseline_host/`
- 真跑产物与截图：`D:/User/Desktop/比赛存档/v1/_tmp_session/SIM-STUDIO-MERGE-20261024/…/resp.json`、`D:/User/Desktop/比赛存档/v1/_tmp_session/SIM-STUDIO-MERGE-20261024/shot_top.png`、`D:/User/Desktop/比赛存档/v1/_tmp_session/SIM-STUDIO-MERGE-20261024/shot_modal_tall.png`、`D:/User/Desktop/比赛存档/v1/_tmp_session/SIM-STUDIO-MERGE-20261024/shot_compare_tall.png`
- 包测试逐项日志（20 套）：`D:/User/Desktop/比赛存档/v1/_tmp_session/SIM-STUDIO-MERGE-20261024/…/tests.log`
- 本卡新增实测输出：`_tmp_session/m_6th_tour.txt`、`D:/User/Desktop/比赛存档/v1/_tmp_session/SIM-STUDIO-MERGE-20261024/m_6th_tracker-safe.txt`、`D:/User/Desktop/比赛存档/v1/_tmp_session/SIM-STUDIO-MERGE-20261024/m_5th_tour.txt`、`D:/User/Desktop/比赛存档/v1/_tmp_session/SIM-STUDIO-MERGE-20261024/m_5th_tracker-safe.txt`、`D:/User/Desktop/比赛存档/v1/_tmp_session/SIM-STUDIO-MERGE-20261024/plan_tour.txt`、`D:/User/Desktop/比赛存档/v1/_tmp_session/SIM-STUDIO-MERGE-20261024/plan_tracker-safe.txt`
