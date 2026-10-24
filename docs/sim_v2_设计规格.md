# sim_v2 设计规格 — 拖拽棋子地图 · 50图压测 · 无人机协同仿真

> 性质：**纯规格**（requirements 卡 t5，零代码改动）。四大必答 = A 棋子模型 / B 压测 / C 无人机 / D 模块契约。
> 每条设计决策后附「理由：」一句。事实依据见 §5，实现卡文件归属见附录。
> 面向后续实现卡；本队（sim-bench-free）本轮仍不触碰 sim.html / src/simbench.js / index.js。

## 0. 范围

- 做：①四类棋子拖拽替代点位文本框，运行即重规划；②一键 50 张随机图并行仿真与统计；③无人机按最新地图规则入仿（起飞→扫描→边扫边算，小车并行开跑，**不建模打靶**）。
- 不做：改 `config/**`、`src/dzbrain/**`、`tools/**`（全程只读）；实车链路；打靶动作。
- 复用底盘：`GET /sim/map`（fieldSpec 只读真源）、`POST /sim/run`（runSim：field→运行目录 override→plan_route→sim_cli，见 v3 档案 §4.2 的「改点位三步」）。

## A. 拖拽棋子模型（数据模型与落格规则）

1. **四类棋子**：雷 `mines`、障 `obstacles`、靶 `targets`（双格板）、通 `bridges`。棋盘状态即唯一 UI 真源，每类一组格名数组。理由：与 `runSim body.field` 及 map.yaml 键名一一对应，零转换层。
2. **落格规则**（拖拽时前端即时校验、拒绝非法落格）：
   - 格名合法：`/^[A-N](?:[1-9]|1[01])$/`（simbench.js `CELL_RE`，14×11=154 格）；
   - 一格至多一类棋子（四类互斥），不占禁落格 `start=N10`、`end=N5`、无人机点 `N8/M2`（map.yaml:18-19 注释）；
   - 通：只能落 K 列，上半场（行 1–6）恰 1、下半场（行 7–11）恰 1 —— 理由：K 列整列是桥、隔板行 6/7 封死 D~N，U 形路线每半场恰需一洞（map.yaml:33-35、:59-60）；
   - 靶：恰 2 格横/竖相邻（`targetYawOfCells` 判 'h'/'v'），两格须在隔板同侧（不得一行 ≤6 另一行 ≥7）—— 理由：实体是 35×70cm 跨格一块板，不能骑缝在隔板上（map.yaml:57-58）；
   - 数量软口径：雷 8/障 8/通 2/靶 2 为默认（map.yaml 现值）；拖多拖少前端只警告不拦截，**最终判据 = plan_route --check exit=0**（不过则走现成 422 plan-failed 通路）。理由：权威规则在 tools/route_check.py 六层校验，前端复刻必与之分叉。
3. **靶=相邻两格合并**：渲染为整块矩形（沿用 targetBoardRect），拖任一格另一格跟随保相邻，顺序无关；提交值 `body.field.targets = [格A, 格B]`。理由：与 simbench `targetBoard()` 横竖判定（t28）口径一致。
4. **与 body.field 通道映射**：现 `FIELD_KEYS=['mines','obstacles','bridges']`（simbench.js:91），**缺 targets** → 实现卡1 把 `'targets'` 追加进该数组即可：`applyFieldToMapText`/`sanitizeParams`/`normalizeCells` 均按 FIELD_KEYS 循环，map.yaml 本就有 `targets: [...]` 行，改动一行、通路全通（override 副本、零写 config/，v3 档案 §4.3 铁律）。理由：复用现成通道，不新开协议。
5. **清空语义**：某类拖到 0 枚 = 不生效（沿用 `cells.length===0 → continue`），UI 提示「至少留 1 枚」。理由：现语义把空数组当"未改"，改它要动既有守卫断言，且空雷图无赛事意义。
6. **恢复默认**：保留每类「恢复默认」+新增「整图恢复」，一律重新 `GET /sim/map` 回填（现 sim.html:920-922 机制）。理由：真源在 map.yaml，本地缓存恢复会漂。
7. **文本框去留**：三个 textarea（sim.html:194/198/202）**下线**，`fieldFromForm()` 改 `fieldFromBoard()`，返回结构不变 `{mines,obstacles,bridges[,targets]}`；`fieldInputErrors` 拦截前移至落格时。理由：用户明确"替代"；文本框与棋盘双源必分叉。

## B. 压测语义（50 图）

1. **随机生成**：入口参数 `{count=50, seed, concurrency=4}`。第 i 图用自带 PRNG `mulberry32(seed+i)`（**禁用 Math.random**）：先排禁落格，再抽雷 8、障 8（类间互斥），随机取相邻对作靶（横/竖各半概率），K 列上/下半场各抽 1 通。理由：seed 是用户点名验收项，平台 Math.random 不可复现。
2. **合法性两级判据**：①A.2 静态规则（生成器自检，不过重掷至多 3 次）；②`plan_route.py --map override图 --check` exit=0。两级都过才跑 sim_cli；失败记 `status=plan_failed/invalid` 照实进台账、进分母。理由：规划器是唯一权威，静默剔除失败图会虚抬完赛率。
3. **并发池=4**：压测持有自己的池闸（≤4 个 python 子进程并发）；有 field 的子跑额外 +1 个 plan_route 进程。理由：sim_cli 单线程 python，4 路是 wall-clock 收益与内存/句柄/台账追加争用的折中，>4 收益递减且同毫秒撞 run id 概率上升；50 图顺序跑无法满足"同时跑"。
4. **单跑 running 锁不得破坏**：simbench 模块级 `running` 布尔与手动 `/sim/run` 的 409 语义**原样保留**；压测期间手动单跑仍 409（文案含「压测进行中」）。压测子跑改走新内部通道 `runSimJob(body,{lock:false})`（`runSim` = 锁检查 + `runSimJob`，对外行为逐字不变），池内互斥由 stressbench 自己管。理由：409 是对外既有行为（守卫断言覆盖区），锁拆分责任归压测方。
5. **run id 防撞**：压测子跑 id = `${Date.now()}_${batchId}_${seq}_${label}`（现 `Date.now()_label`（simbench.js:466）并发同毫秒会同目录互踩）。理由：outDir 同名 = 产物互相覆盖。
6. **统计口径公式**（分母与缺失口径全部显式；缺失指标的 run 单列 `missing`，不参与均值）：
   - 完赛率 = `count(finished=true) / 50`；
   - 平均完赛时间(s) = `Σ time_s(finished) / count(finished)`（sim_cli「总时长」，METRIC_LABELS 已有正则）；
   - 平均规划耗时(ms) = `Σ plan.ms / count(有 plan)` —— **新增字段**：runSim 用 Date.now 包 `planRoute` 落 `record.plan.ms`（现 record.plan 只有 exitCode/tokenCount，simbench.js:503-512）。理由：用户点名「路径规划计算时间」，现台账没这个数。
   - 平均仿真墙钟(s) = `Σ record.seconds / count(有 seconds)`（现字段 simbench.js:560）；
   - 平均长度比 = `Σ length_ratio / count(有值)`（sim_cli「路径长度比」）；
   - 另报 status 分布（done/plan_failed/timeout/exit*）与各均值的 count。理由：只给均值不给分母，50 张里挂 20 张也看不出来。
7. **seed 可复现**：仿真 `--seed = (seed*1000+i) mod 2^31`，与地图生成同源；`{seed, count}` 相同 ⇒ 同一批 50 图、同批指标；整批 `{seed,count,concurrency,stats}` 落 `artifacts/sim/stress/<batchId>/summary.json`。理由：复审者要能一键重放出同批数据（三档基线同口径：一律带 --seed）。
8. **写入边界**：产物只落 `artifacts/sim/stress/<batchId>/`；地图/配置仍走运行目录 override 副本，`config/**`、`src/dzbrain/**` 零写入（沿 t25 铁律）。理由：与既有一致，免得压测成为绕过台账的后门。

## C. 无人机协同时序

1. **起飞点取 map 注释的 N8**：`fieldSpec().drone` 已从 map.yaml「无人机点」注释抽两格（source='map-comment'，兜底 N8/M2 内建，simbench.js:93、:204-231；map.yaml:19；v3 档案 §2.0 四黄框 N10/N5/N8/M2）。终点 M2。理由：读取通道现成，不另造配置项。
2. **扫描覆盖模型**：无人机沿 U 形俯瞰航线（手册 §0：N8→M2）按**步长 1 格**取扫描点序列；每点覆盖以该点为中心的 3×3 邻域（半径 1 格）——步长 1 + 窗口半径 1 ⇒ 走廊无缝覆盖。理由：0.35m/格 ×3 = 1.05m 视场，与感知管线按格判定的口径一致（map.yaml:10 格宽）。
3. **情报报文同构**：产出 `bridge_passable / target_zone / field_verify` 三种 type，字段与车侧契约一致，且照搬采纳校验（非 K 列/半场行段不符/∉候选集 → `rejected` 走默认，手册 §2.1、§7 降级矩阵）。理由：仿真与真车报文同构，结论才可迁移。
4. **边扫边算的重规划节奏**：t=0 双方同时出发（车按先验默认 `route.cross_lower/upper_default=K9/K5`，robot.yaml:527-528、手册 L130；无人机自 N8 起飞）。无人机每完成一个半场的通口扫描结算一次 bridge 情报：情报**通过采纳校验且与车当前路线假设不同** ⇒ 触发重规划（车原地暂停 1 拍 → plan_route(真值图) → 从当前格换新 token 续跑）；情报相同 ⇒ 记 `confirmed` 不重跑规划（plan_route 秒级，同值重跑纯浪费）。上限 2 次（每半场 1 次）。真值由参数 `truthBridges` 构造，默认取 map.yaml bridges（与先验同值 ⇒ 演示 0 次重规划 + confirmed）。**有意差异标注**：车侧现实是出发前才换洞、途中情报只更新遥测（手册 L153）——本仿真是按用户「边扫边算、小车边跑」要求把重规划放宽到途中，属仿真增强，非车侧行为复刻。理由：差异必须写明，否则仿真结论会被误读为实车语义。
5. **小车并行开跑**：车不等无人机情报、t=0 即按先验默认出发；等不到/被拒情报走默认完赛，对应降级矩阵「最坏也能完赛」（手册 §7）。理由：用户明确并行；降级矩阵已证明无情报可完赛。
6. **明确不建模打靶**：`target_zone` 只更新 field_diff 显示，**不产生射击动作、耗时、得分**；靶格作为静态不可压要素沿 sim 现行处理。理由：用户明确"仿真内暂不考虑打靶"；且车侧 target_zone 本就只报差异不改路线（手册 §2.1）。

## D. 模块契约（三张实现卡零沟通对接）

**`src/stressbench.js`（新，卡2）**
```js
export function generateMap({ seed, index })   // → {ok, field}；纯函数，静态合法性自检
export function stressStats(records)           // → {n, statusCounts, finishedRate, avgFinishS, avgPlanMs, avgWallS, avgLengthRatio, missing}
export async function runStress({ count=50, seed, concurrency=4, label='stress' })
                                               // → {ok, batchId, records, stats}；批次忙 → {ok:false, code:409}
export function stressStatus()                 // → {running, batchId, done, total}
```
**`src/dronesim.js`（新，卡3）**
```js
export function scanPlan({ field, drone })     // → {takeoff, land, points:[{cell, covers:[cell…]}]}（纯函数）
export function intelAt({ plan, truthBridges })// → 与车侧同构的报文数组（含 adopted/rejected 判定）
export async function runDroneSim({ field, seed, truthBridges, carDefaultBridges })
                                               // → {ok, timeline:[{t, actor, event}], reroutes, confirmed, record}
```
**`index.js` 新端点（均在 `/api/dzrobot-ssh` 前缀下，卡2/卡3 追加）**

| 方法 | 路径 | 请求 → 响应 |
|---|---|---|
| POST | `/sim/stress` | `{count,seed,concurrency,label}` → 202 `{batchId}`；忙 409 |
| GET | `/sim/stress/status` | → `{running,batchId,done,total}` |
| GET | `/sim/stress/result?id=` | → `{batch,stats,records}` |
| POST | `/sim/drone/run` | `{field?,seed?,truthBridges?}` → 200 `{ok,timeline,reroutes,record}`（同步单跑） |
| GET | `/sim/drone/history` | → `{runs}`（仿 listHistory，台账 jsonl） |

simbench.js 内部配合项（不新增端点）：`runSimJob(body,{lock})` 拆分、`record.plan.ms`、压测 run id 加 batchId/seq、FIELD_KEYS 追加 `targets`。

## 5. 事实依据（引用的现有事实）

- `config/map.yaml`（唯一真源）：格网 14×11×0.35m（:10）；start N10/end N5、无人机点 N8/M2（:18-19）；隔板行 6/7 D~N、U 形路线（:20、:33-35）；现值雷8/障8/靶 E5+F5 横向/通 K5+K9（:54-60）。
- `docs/空地指挥链路_协议与运行手册.md`：U 形俯瞰航线 N8→M2（:24）；双向报文契约 bridge_passable/target_zone/field_verify（§2.1）；cross 候选/默认 K9/K5（§3 :128-130）；出发前换洞、途中情报只更新遥测（:153）；降级矩阵最坏完赛（§7）。
- `docs/agent_archive/v3-空地协同闭环-2026-10-14.md`：情报→换洞→重规划 bench 32 项（§0）；仿真台接口前缀 `/api/dzrobot-ssh` 与改点位三步 override 法（§4.2-4.3）；第六张图四黄框 N10/N5/N8/M2（§2.0）。

## 实现卡对接附录 — 文件归属矩阵

| 卡 | 新增文件 | 修改文件（**同文件必须单写者串行**） | 只读依据 | 禁区 |
|---|---|---|---|---|
| 卡1 拖拽棋盘 | — | `sim.html`（棋盘/文本框下线）、`src/simbench.js`（FIELD_KEYS +targets，一行） | config/map.yaml | config/**、src/dzbrain/**、tools/** |
| 卡2 50图压测 | `src/stressbench.js` | `index.js`（3 端点）、`src/simbench.js`（runSimJob/plan.ms/run id；**排卡1 之后**） | 同上 + 手册 §4 | 同卡1 |
| 卡3 无人机协同 | `src/dronesim.js` | `index.js`（2 端点，排卡2 之后）、`sim.html`（时间线面板，排卡1 之后） | 同上 + v3 档案 §4 | 同卡1 |

冲突规则：`simbench.js` 卡1→卡2 串行、`sim.html` 卡1→卡3 串行、`index.js` 卡2→卡3 串行；三卡其余文件互不相交。理由：本工作区有并发写入者前科（v2 档案 §5），同文件并行写必产竞态。

## E. t12 增补 — 四条新需求的可验收规格（唯一判据）

> 本章为 t12 纯增补（原章节一字未改）：**与 §A~§D 冲突处以本章为准**，显式取代仅限 §A.2 第 5 条「数量软口径」一行。每条给：验收判据 / 真源 / 边界·非目标 / verify 命令建议。

### E1. 需求1 — 硬约束优先级最高，不得放宽（障碍/隔板/桥洞含冗余）

- **验收判据**：
  1. 三类硬约束（障碍、隔板、桥洞含冗余墙）在**一切路径**上优先级高于通过性/效率/长度：拖拽图、压测 50 图、无人机开关联动均不得为「跑通/更短/更快」放宽任何一条，复审发现放宽实现 = reject；
  2. **规划器侧审计清单（现状 文件:行，只读取证）**——障碍：`src/dzbrain/dzbrain/route/planner.py:347`（Grid 通行栅格含障碍占格）、`:429`（move_ok① 目标格可通行）、`:1403`（check_route_structure 踩障碍格报错）、`:2945`（tracker-safe 物理净空）+ `tools/route_check.py:840-859`（障碍净空，error 级 t22）；隔板：`planner.py:416`（仅列索引 < barrier_col_from 可跨越）、`:430`（move_ok②）、`:1433`（非法跨越第 hi/lo 行报错）、`:2511-2520`（隔板旁行侧向 90° 硬校验）+ `route_check.py:304/711`（barrier_crossings / barrier_touch_min）；桥洞含冗余：`planner.py:431-433`（move_ok③④，R007/R009）、`:443-447`（正交相邻前置断言）、`:624-625`（冗余取值）、`:2984-2995` 与 `:3000/:3039/:3061`（_TSGrid 段级+顶点级冗余墙判据）+ `route/physics.py:606 bridge_entry_walls()`；
  3. **回归验收口径**：改动前后 `plan_route --check` 与 `route_check.py` 全量基线 exit 不得变差；三类各留 1 条**变异必须变红**的探针（例：放开 `:416` 跨隔板、删 `:443` 断言、冗余置 0），红→恢复绿双向实测记档；现有守卫断言（G1/T8-1/T8-2）判据零变化。
- **真源**：`config/robot.yaml:549-557`（冗余设计注记与「route_check 刻意不含」）、`src/dzbrain/dzbrain/core/config.py:214-223`（Spec 0.0875，明文不得进 route_check 用户侧判据）。
- **边界·非目标**：不改 planner / route_check / config 任何行；不为压测生成图开豁免层；审计时不得把冗余墙「顺手补进」`route_check.py`（现实不存在）。
- **verify 建议**：`python tools/route_check.py --check <样例>` 基线 exit=0；`Select-String -Path src/dzbrain/dzbrain/route/planner.py -Pattern 'barrier|redundancy|障碍' | Measure-Object | Select-Object -Expand Count`（审计锚点在位）；三条变异探针红/绿双向实测。

### E2. 需求2 — 棋子槽定容与靶板交互（升级 §A.2 软警告为硬拦）

- **验收判据**：
  1. **棋子槽硬顶 8/8/2/1**：雷 8、障 8、通 2、靶板 1（整对恰 2 格）为**硬顶**——槽满后第 N+1 枚落格**直接拒绝**并提示「槽已满」，原 §A.2「只警告不拦截」升级为硬拦（提示语保留，拦截为准）；
  2. **只拖拽放置**：palette 点击仅选中棋子类型，**palette 点击不落子**，落子唯一途径 = 拖拽；移除 = 拖回 palette 或拖出棋盘；
  3. **已放置靶板点击 = 顺时针旋转** 90°（横↔竖换向），**不等于移除**——棋子不消失、恒 2 格；旋转后仍须过「相邻 + 隔板同侧」校验，非法旋转拒绝并回弹原状；
  4. **靶板蓝红两色分半渲染**：沿长边对半、蓝红各半且色值固定不随机；**整图仿真象棋观感**：四类棋子统一「棋子形（圆/方子 + 投影）+ 格位对齐」渲染，不再是裸色块；
- **真源**：数量现值 `config/map.yaml:53-60`（雷 8/障 8/靶 E5+F5 横向两格/通 K5+K9），「恢复默认」一律回读该真源（§A.6 原样）；硬顶数值本身是赛场规则、写在本规格，**不落 config/**。
- **边界·非目标**：不改 map.yaml、route_check 六层；压测生成器（§B.1）同受 8/8/2/1 **硬顶**约束（生成即合法）；旋转/落子不改变 `body.field` 提交结构（§A.3/A.4 原样）。
- **verify 建议**：`Select-String -Path docs/sim_v2_设计规格.md -Pattern '棋子槽|硬顶|顺时针' | Measure-Object | Select-Object -Expand Count`；UI 手测四连击：槽满拖入被拒 / palette 点击不落子 / 靶板点击顺时针换向不消失 / 蓝红分半截图存证。

### E3. 需求3 — 冗余墙渲染值 == 规划器取值（逐字相等）

- **验收判据**：
  1. 仿真器渲染桥洞进洞侧冗余墙，取值唯一真源 = `config/robot.yaml:559 field.bridge_entry_redundancy_m: 0.0875`（= 1/4 格 = 0.35×0.25）；
  2. **一致性链**：规划器 `planner.py:624-625`（`g("field.bridge_entry_redundancy_m", …)`）与 `:2987-2988` → 仿真 `tools/sim.py:934-935`（`cfg.get("field.bridge_entry_redundancy_m", 0.0875)`）→ `sim.py:1516-1519`（`shape.bridge_redundancy_m` 与判据同源）→ `sim.py:551-552`（`metrics.bridge_entry_redundancy_m`）；
  3. **断言口径**：渲染层只读 run 结果同一 JSON 字段，断言 `shape.bridge_redundancy_m === metrics.bridge_entry_redundancy_m === 0.0875`（严格 `===`，禁四舍五入/字符串化往返）；渲染侧**禁止出现 `0.0875` 字面量**（注释豁免），改真源值时渲染必须自动跟随；
- **真源**：`config/robot.yaml:559`；兜底常量 `planner.py:2900-2903 BRIDGE_ENTRY_REDUNDANCY_M` 仅在缺 config 时兜底、与 YAML 同值。
- **边界·非目标**：`field.bridge_entry_redundancy_m = 0.0` ⇒ 判据整体关闭（`sim.py:347/:390 bridge_modeled=0`），此时**不渲染**该墙；冗余墙不进 `route_check.py`（现实不存在）；不新增配置键。
- **verify 建议**：`Select-String -Path <渲染文件> -Pattern '0\.0875' | Measure-Object | Select-Object -Expand Count`（字面量应为 0 处）；跑一次 `/sim/run` 后对响应 JSON 断言两字段 `=== 0.0875`；`Select-String -Path config/robot.yaml -Pattern 'bridge_entry_redundancy_m'` 核真源。

### E4. 需求5 — 无人机选项（开关联动 + 约束清单 + 与 t8 共存）

- **验收判据**：
  1. **开关默认态 = 关闭**。理由：`/sim/run` 纯车语义是守卫断言与 §B 统计口径的覆盖区，默认开会在无预告下改变全部基线；开启须是显式选择（若用户点名默认开，只翻转本行，联动语义不变）；
  2. **开关联动**：无人机选项开启时，本次 `/sim/run` 除车路线外**同时生成无人机航线并纳入本次规划与约束判定**，响应 `record` 追加 `drone` 段与 `constraints[]`（每条 `{id, ok, detail}`）；任一 `ok:false` ⇒ 记 `constraints_failed`，不得静默继续；
  3. **约束清单（四条，规格列明）**：`bounds` 越界——车+机全部航线点 ∈ 14×11 格网与黄框边界内；`takeoff_land` 起降点——takeoff/land == map 注释 N8/M2（`fieldSpec().drone`，兜底 `dronesim.js:21`）且属禁落格（sim.html:1027-1031）；`bridge_gate` 通口——扫描走廊行由 `field.bridges` 推导（`dronesim.js:69-70` 六段正交），须与两半场通口一致；`intel_link` 情报衔接——报文与 §C.3 同构，adopted/rejected 判定走 `intelAt`（`dronesim.js:108`），rejected ⇒ 按默认路线完赛、不触发重规划；
  4. **与 t8 时间线共存**：t8「跑空地协同」按钮与 `renderDroneTimeline`（sim.html:321-324、:1633-1694）**原样保留**，端点 `/sim/drone/run` 契约不变；选项开的普通 run **不替换**该时间线面板，结果只附在自身 record（label 加后缀区分，沿 §B.5 思路），两入口各跑各的、互不覆盖；
- **真源**：起飞点 `config/map.yaml:19` 注释 + `simbench.js:93/:204-231`；t8 已验收实现 = `src/dronesim.js`（scanPlan/intelAt/runDroneSim，§D 签名原样）；手册 §0/§2.1/§7、v3 档案 §4。
- **边界·非目标**：不建模打靶（§C.6 原样）；不改 `dronesim.js` 已验收导出签名与 t8 端点；**关闭态与今日基线逐字节一致**（守卫断言零变化）。
- **verify 建议**：关态跑 `/sim/run` 断言 record 无 `drone` 段（基线不变）；开态断言 `constraints` 四 id 齐全且 `ok:true`、`drone` 段含 takeoff/land/points；`Select-String -Path sim.html -Pattern 'droneRun|renderDroneTimeline' | Measure-Object | Select-Object -Expand Count`（t8 入口在位）。

### E5. 增补文件归属（并入附录矩阵；单写者串行规则不变）

| 需求 | 落点 | 改动文件（同文件串行） | 只读依据 |
|---|---|---|---|
| E1 硬约束审计 | 审计/复验卡 | 无（纯取证） | planner.py、route_check.py、robot.yaml、config.py |
| E2 棋子槽硬顶+旋转 | 卡1 拖拽棋盘 | `sim.html`、`src/simbench.js`（排卡2 前） | map.yaml:53-60、本章 E2 |
| E3 冗余墙渲染一致 | 卡1（渲染侧） | `sim.html`/`src/simbench.js` 同文件串行 | robot.yaml:559、sim.py:551/:1516 |
| E4 无人机选项 | 卡3（+卡1 开关 UI） | `sim.html`（排卡1 后）、`index.js`（若增端点排卡2 后） | dronesim.js、手册、v3 档案 |
