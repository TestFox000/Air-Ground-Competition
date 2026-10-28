---
卡号: RECT-PLAN-FIX
ns: agc-1026
agent: d55ee7b6-d7e5-42b3-bb62-bb9ae10a6483
owner: d55ee7b6-d7e5-42b3-bb62-bb9ae10a6483
reviewer: lead
树节点: R-8
写域: "_tmp_session/AGC-RECTIFY-20261026/fix-plan/ 及本卡"
status: in_progress
创建: 2026-10-26
---

# RECT-PLAN-FIX · 规划器单站判死的隔离候选

> **本卡不部署、不 commit、不改生产代码。** 生产 `route/planner.py` 全程只读；唯一写入是 `_tmp_session/AGC-RECTIFY-20261026/fix-plan/` 与本卡。候选补丁只存在隔离副本。
> **请求 Lead 落树**：`00_树.md` 现只有 R-0…R-6，**没有 R-7/R-8 行**；R-8 由 ns owner（Lead）写入后，R8 判据（「树里没有指向本卡的节点」）才会转绿。

## 目标
独立证实或推翻：`planner.py:3305` 的 `len(spec.targets)>=2` 把「靶格数」当「靶板数/需求站数」，与同文件 `:1754` 的「1~2 个」口径冲突，导致第六图单站时 tokens=0、整趟不发车。若证实，在隔离副本给最小候选。

## 结论（先给判断，再给证据）

| # | 待证事实（派单方给的前提） | 独立复核 |
|---|---|---|
| 1 | `:1749-1758` 已放宽为「射击停站 1~2 个」，注释写「不再因此把整条路线判死」 | **成立**：生产 `planner.py:1754` 为 `if not 1 <= len(stops) <= 2`；0 个只在 tasks + shoot_missing 时放行（`:1757`） |
| 2 | `:3305-3307` 在 `len(spec.targets) >= 2` 时又调 `check_shoot_stops(...)`，后者要求恰好 2 个 | **成立**：`:3305` 调用；`:1274-1275` 判 `len(names) != 2`；`:1285-1288` 另要求两站分居板面两侧 |
| 3 | 第六图 `targets: [D5, D6]` 是**一块纵向跨格靶板**，不是两块板 | **成立**：夹具 `map_sixth_20261017.yaml:14` 原文「纵向相邻两格 = 一块 35×70cm 跨格靶板」；数据模型上 `targets` 是靶格列表，`Grid` 把这两格当实体拉黑（见 §语义判定） |
| 4 | `diag_layout_target.json`：engine=tour、tokens_len=0、报「双射击停站应为 2 个，实际 1 个：B4」 | **成立**：隔离副本独立复现，探针 rc=2，文案逐字相同（见 §复现） |

**但第 4 条附带的「第六图唯一合法站 B4」不成立**：第六图的对侧合法站是 `F4`（雷格，现役手写路线里就带 `F4*`），它进不了池只是因为 `shoot_candidates()` 把雷格一起排除了。**「第六图物理上不可双面停站」是候选池策略的产物，不是几何结论**（见 §反证）。

**时间线事实（必须先摆清）**：这条「整趟判死」是**在途回归**，不是历史设计。HEAD `caaee3b1` 的 `planner.py`（sha256 `c7f237b4…`）在同一张图上 **rc=0 / 68 token / B4\***。`:3305` 的严格调用、`check_shoot_stops()`、以及钉住该行为的 `test_dual_shoot.py`（未跟踪新文件）与 `test_autoplan.py §13`（未提交修改）全部来自**同一批未提交改动**。

### 需要 Lead 裁决的三件事

1. **采用候选 A 必须同时改 4 条既有断言**（`test_dual_shoot.py:156`、`test_autoplan.py:1325/1331/1340`）。它们是同批在途改动自带的期望，**我没有改测试**（守红线）。改不改、怎么改由 Lead 定，并须连同「翻案三问」记台账。
2. **候选 A 只解除「整趟无解」**：第六图出单站降级路线（B4\*）。要拿回与现役手写路线**同样的两站**（F4\*+B4\*），需要动「射击候选池是否排除雷格」——那是另一张卡（§实验 B 已给爆炸半径实测）。
3. **在办状态提醒**：`planner.fallback_to_tokens = false` 且 `planner.auto_plan_enabled = true` ⇒ 今天第六图不是「回退手写路线」，而是**规划失败即不出车（IDLE）**。所以 A 的收益是「0 条路线 → 1 条单站降级路线」，不是「手写路线 → 降级路线」。

## 复现（独立，非采信）

隔离副本 `iso/prod/`：仓库形状 `src/dzbrain/{dzbrain,tests}` + `config/robot.yaml` + `tools/` + `maps/{sixth,carloop,field}.yaml`。

| 项 | 值 |
|---|---|
| 生产 planner（preimage） | `c14dadd4bd08fee704305d93c57f9d1ec4fb29bd516db90e133b9c3265d5d0b7` |
| HEAD 的 planner | `c7f237b4c49d22fcc9504b2a15ae2b1075004bd82909f18a939247ed95021adb`（HEAD `caaee3b1`） |
| `config/robot.yaml` | `d61effc361b02284fd70399e7cb2d19e3309c8ecf1ac4f2a06767ae83d44d313` |
| 第六图 / carloop / field 夹具 | `445975bc…` / `e487d0e9…` / `f4e5c14e…` |
| 关键生效配置 | `route.shoot_distance_m=0.90`、`route.shoot_distance_tol_m=0.12` ⇒ 窗 **[0.78, 1.02]**；`planner.engine=tour`；`auto_plan_enabled=true`；`fallback_to_tokens=false`；`mission.shoot_timeout_s=30.0` |

```bash
cd "D:/User/Desktop/比赛存档/v1/_tmp_session/AGC-RECTIFY-20261026/fix-plan"
python -B -X utf8 probes/probe_plan.py --src iso/prod/src/dzbrain \
  --robot iso/prod/config/robot.yaml --map iso/prod/maps/sixth.yaml \
  --out runs/prod_sixth_legacy.json        # 实测 rc=2
```

```
plan_route   : ok=False
exception    : PlannerError: 规划结果未通过结构自检：
                 - 双射击停站应为 2 个，实际 1 个：B4
from_config  : ok=False  tokens_len=0  engine=tour  shoot_stops=[]
```

同图换 `--objective tasks`：`plan_route` 不抛，但返回 **50 token / 0 个射击停站 / degraded「结构（无可行顺序）」**（`_best_effort_result` 兜底，靶覆盖 0/2）；消费入口 `plan_route_from_config` 仍 rc=2（该入口固定走 legacy）。两条都不是可竞赛形态，与 RECT-PLAN Q2 一致（我独立复现，未采信其数字）。

## 语义判定：`targets` 是「靶格」，不是「靶板」

1. `MapSpec`（`planner.py:294`）只有 `targets: tuple[str, ...]`，**没有「板」实体/编号/朝向**；`load_map()` 把 `map.targets` 归一化为格名元组。
2. `Grid.__init__`（`planner.py:416-418`）逐格把 `spec.targets` 加入 `self.blocked`，并另存 `target_cells` —— **一块跨格靶板在数据上就是两个被拉黑的格**。
3. `plate_center_m()`（`planner.py:965`）= 各靶格中心算术中点；`plate_bisector_axis_m()` / `plate_face_normal_m()` 由「两格连线」推板面法向。三者只吃 `spec.targets`，**无法区分「一块两格板」与「两块一格板」**。
4. ⇒ `len(spec.targets) >= 2` 的真实语义 = **「这块板跨了两格」**（在「所有靶板都是跨格板」裁定下，对**单块板**的图恒成立），不是「两块板/两处靶区」。拿它当板数用，就会硬求「一块板两个停站」。

第六图实测：板心 `(2.10, 3.675)` m、板面法向 `(0, −1)`、两靶间距 0.35 m；窗内候选 7 格：`B4/B7` 侧 −1 覆盖两靶；`E3` 侧 +1 只覆盖 D5（到 D6 = 1.1068 m 超窗）；`C3/C8` 各覆盖 1 靶；`D3/D8` 与板心同面（覆盖 0 靶）。`second_shoot_stop(B4) = ""`。

## 反证：第六图存在合法对侧两站 —— `F4*` + `B4*`

**这条推翻了「单站是这张图的物理必然」**，所以不能拿「物理上只能单站」给判死背书。

```
F4：到板心 0.875 m（窗内）· 覆盖 D5=0.7826 / D6=0.9899（两靶全中）
    板面侧 +1（与 B4 的 −1 异侧）· 本来就是 tour 路线上的普通航点
    是雷格 ⇒ 被 shot_candidates()/shoot_candidates() 的
    blocked = set(spec.obstacles) | set(spec.mines)（planner.py:1124）排除
```

两条硬证据：

1. `config/robot.yaml` 的现役手写路线 `path_executor.mission_blocks`（68 token、与第六图 tour 产物同几何、同 3 处 `:PIVOT`）**就是 `F4* + B4*`**。
2. 用生产工具 + 第六图夹具跑这条手写路线：`tools/route_check.py --config … --route <68 token>` → **rc=0**：`射击点索引 [44, 48]`、`雷覆盖 8/8`、`靶覆盖 2/2`（D5←#44 0.78 m、D6←#44 0.99 m）、`射击点距离：2 个射击点都能服务于某个靶`。

⇒ **第六图的「对侧无合法站」只是「雷格不入池」这条策略的后果；两站解存在，且是现场正在用那条。**

## 候选 A（隔离副本，最小口径；**未进生产**）

设计：`:3305` 与 `check_route_structure()` 的「1~2 个」**共用同一口径**；2 个仍全量校验（异侧/去重/全靶覆盖）；**单站不判死，但必须显式降级**（返回值 `degraded` / `degraded_reason` + `warnings`），文案**分三种**（物理无解 / 本路线未兑现 / 落在板心平面），不许把「没兑现」说成「物理无解」。
范围刻意与旧硬门一致（多格靶板 `len(targets) >= 2`）：单格靶图（`map_field` 的 G8）行为完全不变。

| 项 | 值 |
|---|---|
| 副本 | `iso/cand/…/route/planner.py` |
| preimage sha256 | `c14dadd4bd08fee704305d93c57f9d1ec4fb29bd516db90e133b9c3265d5d0b7` |
| postimage sha256 | `c4a89e5a01f72ed12f7119666b12a8b52d51f66c4e878e81bf66bdd7fabd3132` |
| 规模 | **+78 / −3 行**（`git diff --no-index --numstat` 实测；1 个私有函数 `_single_stop_degradation()` + 4 处调用点改动，其余为注释） |
| 改动面 | **仅 `route/planner.py`**；不动 `config/**`、不动 `ensure_second_shoot_stop()` 的双站校验、不动搜索/代价/路径 |

候选 diff 摘要（完整版 `evidence/candidate_patch.diff`；行号为隔离副本）：

```diff
@@ -1289,6 +1289,50 @@
     return problems
 
 
+def _single_stop_degradation(spec: MapSpec, grid: Grid, block: str,
+                             accept: tuple[float, float] | None = None
+                             ) -> tuple[str, list[str]]:
+    """单停站**显式降级**的一句话原因（+ 对侧候选清单）。**只报不判死。**"""
+    if not block or not spec.targets:
+        return "", []
+    side = plate_side_of(spec, block)
+    if side == 0:
+        return (f"单停站 {block}（落在板心平面：…）", [])
+    pool = shoot_candidates(spec, grid, accept)
+    cover = covering_stops(spec, pool, accept)
+    opposite = [c for c in pool
+                if c != block and plate_side_of(spec, c) == -side
+                and len(cover.get(c, ())) == len(spec.targets)]
+    if opposite:
+        return (f"单停站 {block}（对侧有 {len(opposite)} 个…但本路线未兑现…）", opposite)
+    return (f"单停站 {block}（全候选池 {len(pool)} 格里没有…物理上不可双面停站…）", [])
+
+
@@ -3302,9 +3350,27 @@
-        if len(spec.targets) >= 2:
-            problems.extend(check_shoot_stops(
-                [wp.block for _i, wp in _route.shoot_points()], spec, _shoot_accept(cfg)))
+        _stop_blocks = [wp.block for _i, wp in _route.shoot_points()]
+        if spec.targets:
+            problems.extend(check_shoot_stops(_stop_blocks, spec, _shoot_accept(cfg),
+                                              require_pair=(len(_stop_blocks) == 2)))
+            if len(_stop_blocks) == 1 and len(spec.targets) >= 2:
+                single_stop_note, _opp = _single_stop_degradation(
+                    spec, grid, _stop_blocks[0], _shoot_accept(cfg))
+            else:
+                single_stop_note = ""
@@ -3332,6 +3398,10 @@
         route = _route
         break
+    if route is not None and single_stop_note:
+        warnings.append(single_stop_note)
     if route is None:
@@ -3393,6 +3463,10 @@
     if spec.targets and not tasks_stops:
         _reasons.append("缺停站")
+    if single_stop_note:
+        _reasons.append(single_stop_note)
     degraded = bool(_reasons)
```

**它不改变的东西**（证明「只动一个变量」）：候选池 `shoot_candidates()` 未改；`covering_stops()` / `plate_side_of()` / `second_shoot_stop()` / `ensure_second_shoot_stop()` 未改；搜索、代价、朝向、穿洞、PIVOT 全未动；`config/**` 未动。

## 验证矩阵（隔离副本实测，全部落 JSON）

| 图 | objective | prod（`c14dadd4`） | HEAD（`c7f237b4`） | 候选 A（`c4a89e5a`） |
|---|---|---|---|---|
| 第六图 | legacy | **rc=2**、ok=False、0 token、`…实际 1 个：B4` | rc=0、68 token、B4\*、degraded=False | **rc=0、68 token、B4\*、degraded=True** |
| 第六图 | tasks | 兜底 50 token / **0 停站** / degraded「结构（无可行顺序）」；入口 rc=2 | rc=0、68 token、B4\*、degraded=False | rc=0、68 token、B4\*、degraded=True |
| carloop（双站） | legacy | rc=0、62 token、**F9\*+C5\***、sides [−1,+1]、degraded=False | rc=0、62 token，但**引擎只标 F9\***，C5 由 Mission 层补星（`second_added=C5`） | **与 prod 逐 token 相同**（F9\*+C5\* 引擎自带，`second_added=""`）、degraded=False |
| carloop | tasks | rc=0、63 token、F9\*+C5\* | rc=0、63 token | **与 prod 逐 token 相同** |
| field（单格靶 G8） | legacy/tasks | rc=0、54 token、E10\*、degraded=False | rc=0、54 token、E10\* | **与 prod 逐 token 相同**、degraded=False（未标降级：单格靶图不在缺陷范围） |

第六图降级理由实测（`runs/cand_sixth_legacy.json`）：

```
单停站 B4（全候选池 7 格里没有『异侧且覆盖全靶』的合法点 ⇒ 本图物理上不可双面停站；蓝面朝对侧那局打不到）
```

三分支直测（`probes/probe_degradation_branches.py`，rc=0）：第六图 B4 →「池里没有」；carloop F9 →「对侧有 1 个合法候选 C5，但本路线未兑现」；field E10 →「落在板心平面」。**三种情形文案不同，不会把「没兑现」伪装成「物理无解」。**

其余门禁（候选产物，第六图 68 token）：

| 门禁 | 命令 | 结果 |
|---|---|---|
| 路线校验 | `tools/route_check.py --route <68 token>`（`ROBOT_MAP`=第六图） | **rc=0**：雷 8/8（最差 0.0000 m）、靶 2/2（#48 一站覆盖 D5 0.78 / D6 0.99）、不压靶板、折返 3 处一致、24.85 m；1 项提醒「急转弯 22 处」（该图固有） |
| 仿真闭环 | `python -m dzbrain.tools.sim_cli --route <文件>` | **rc=0**：`ok=true`、`completed=true`、108.24 s、末点误差 **0.0285 m**、`path_len_ratio=0.9477`、`diverged=false` |

（`sim_cli --route` 只吃**文件**，且 `parse_mission` 的切分符含逗号，会把末 token `N5,y` 拆成 `N5`+`y`；改用等价 `:y` 写法回读，见 `runs/cand_sixth_legacy.simroute.txt`。既有解析链的坑，与本候选无关。）

## 测试影响（**未改任何测试**；这是采用前提下的阻塞项）

| 测试 | prod | 候选 A | 差异 |
|---|---|---|---|
| `test_dual_shoot` | 35/35 ✅ | **34/35** | `missing_second :: legacy 完整规划拒绝单站假成功`（`test_dual_shoot.py:156`，期望抛 `PlannerError`，现在返回 degraded） |
| `test_autoplan` | 177/177 ✅ | **174/177** | §13.2（:1325）、13.3（:1331）、13.4（:1340）「第六图必须明确失败/回退/IDLE」 |
| `test_route` | 156/157 | 156/157 | 相同；唯一失败是 `route.tokens 是 44 个 token`（现配置 68）——**存量**，与本卡无关 |
| `test_planner` | 崩溃 | 崩溃 | 相同：`config/map.yaml` 已从本仓删除（`git status` 显示 D），测试仍按旧路径读 ⇒ **环境存量** |

**性质判定（重要）**：这 4 条不是历史裁决，而是**同批在途改动自带的期望**（`test_dual_shoot.py` 未跟踪、`test_autoplan.py` §13 未提交；`git show HEAD:…` 里都没有）。「单站必须失败」目前只是**在途作者口径**，未进决策台账（全文检索 `决策台账.md` 无「双站/单站/CAR-DUAL」条目）。

若 Lead 采用 A，最小改法是把这 4 条从「必须失败」改为「必须 degraded 且理由非空」；**只是建议文本，我没有改**。

## 实验 B（不推荐本轮采纳，只给爆炸半径实测）

只把 `shoot_candidates()` 的排除集从 `obstacles|mines` 改成 `obstacles`（允许雷格当射击停站，与现役手写路线一致）：

| 图 | 结果 | 与现产对比 |
|---|---|---|
| 第六图 | 68 token、**F4+B4**（sides +1/−1）、degraded=False | 与现役手写路线**同站点**，两站兑现，判死不出现 |
| carloop | **57 token**（原 62）、第二站 **C5→F5**（F5 是雷） | **路线变了**：已调好的双站图被改写 |
| field | 54 token、E10\*、不变 | 无变化 |

⇒ B 能让第六图一次拿回两站，但会改动其它已调好的图，**必须单独立卡**：全量回归 + `route_check`/`sim` + `test_dual_shoot` 几何钉值（62 token / 22.75 m）重裁。**不与本候选同批落地**（一次只改一个变量）。

## 读过的台账条目

- **D-008**（冻结区）：`src/dzbrain/dzbrain/**` 冻结 ⇒ 全程只读，候选只进 `_tmp_session`。
- **D-012**（唯一引擎 `tour`）：未改引擎、未转正 `global_planner`。
- **D-023**（L3 处置）：原文「L3 tour 单停站（`test_autoplan` 6 条，**必须改代码**）」——「引擎单站」历史上被定过真缺陷；本卡不冲突（有对侧解时仍必须两站，只在无解/未兑现时显式降级）。
- **D-025 / D-028**：不改火控/规划参数；本候选零参数改动。
- **D-044**：未 commit、未 push。
- 待入账建议（供 Lead）：①「单站 = 显式 degraded 而非整趟失败」；②「雷格是否可作射击停站」（实验 B）。两条都属于口径变更，需单独条目。

## 复用检索结果

- `doc_search "双射击停站"` → RECT-PLAN Q2、技术全景_B:525、lead-1023/SIM-STUDIO-OPT（`require_pair` 全仓用法）、`交接_20261024_打靶闭环`。
- `doc_search "两侧各留一个射击停站"` → 同批（含现役路线 `F4*+B4*` 的记录）。
- `dzrobot_v2/docs/复用清单.md` / `能力索引.md`：规划唯一入口 `route/planner.py`；校验 `tools/route_check.py`；仿真 `dzbrain.tools.sim_cli`；双站专项 `src/dzbrain/tests/test_dual_shoot.py`。
- **结论：零个新永久轮子**；一次性脚本全部落在本卡写域 `probes/`，不进 `tools/`、不登记复用清单。

## 新增轮子及理由

一次性诊断脚本（不参与门禁、不部署）：

1. `_tmp_session/AGC-RECTIFY-20261026/fix-plan/probes/probe_plan.py` —— CLI 失败时只回一句话，看不到候选池/降级原因；本脚本同时打「直调 `plan_route`」与「消费入口 `plan_route_from_config`」，并把配置/地图/planner 的 sha256 落进 JSON。
2. `_tmp_session/AGC-RECTIFY-20261026/fix-plan/probes/compare_runs.py` —— 两份 JSON 的 tokens/停站/降级字段逐项比对（证明「逐 token 相同」）。
3. `probes/probe_f4.py` —— 把现役手写路线里的星号逐个过判据（这条发现了 F4）。
4. `probes/probe_degradation_branches.py` —— 直测降级文案三分支。
5. `_tmp_session/AGC-RECTIFY-20261026/fix-plan/probes/` 下另有四个纯搬运脚本（结果汇总、token 导出、手写路线导出、实验 B 单行替换），无算法、无判据。

## 写域

`_tmp_session/AGC-RECTIFY-20261026/fix-plan/` 及本卡。生产源码与冻结区一律只读；未修改任何既有测试文件；未覆盖作者已有产物（`sandbox/`、`sandbox_v4/`、`sandbox_v5/`、`logs/base_test_aim.*` 等）。

## 验收清单

- [x] 用第六图 fixture + 当前 config 独立复现 tokens=0 与该错误，给出确切命令和 rc（rc=2；见 §复现）
- [x] 查 MapSpec/load_map/plate_center_m，判定 targets 是格列表还是板实体，确认是否把格数当板数（**是格列表；确实把格数当板数用**；见 §语义判定）
- [x] 隔离副本最小候选：与 `:1754` 共用 1~2 口径；单站显式 degraded 可见，不静默（见 §候选 A）
- [x] 历史双站夹具证明候选不破坏双站路径（carloop 62/63 token 逐 token 相同，F9\*+C5\*）；第六图证明候选下 tokens 非空（68）
- [x] 反证检查：未发现「单站本来就该判死」的赛规依据；反而发现对侧两站解（F4\*+B4\*）与生产配置 `fire_only_blue=true`（打红不开火，单站代价是白停而非违规）。**但发现 4 条既有断言把「单站必须失败」钉住**，已写明需 Lead 裁决，未自行改测试
- [x] 给 preimage sha256、候选 diff、rollback 命令；未改生产 `planner.py`
- [x] Lead 独立复核后勾选（作者与本卡执行者不得自行勾此项）—— 2026-10-27 22:2x，见下方「Lead 复核：决策 A 落生产工作树」

## 验收命令（机器可判；reviewer 独立复跑用）

```bash
cd "D:/User/Desktop/比赛存档/v1/_tmp_session/AGC-RECTIFY-20261026/fix-plan"

# 1) 复现基线：必须 rc=2 且 error 含「实际 1 个：B4」
python -B -X utf8 probes/probe_plan.py --src iso/prod/src/dzbrain \
  --robot iso/prod/config/robot.yaml --map iso/prod/maps/sixth.yaml \
  --out runs/prod_sixth_legacy.json ; echo rc=$?

# 2) 候选：必须 rc=0，且 JSON 里 plan_route.ok=true / n_tokens=68 /
#    shoot_blocks=["B4"] / degraded=true / degraded_reason 非空
python -B -X utf8 probes/probe_plan.py --src iso/cand/src/dzbrain \
  --robot iso/cand/config/robot.yaml --map iso/cand/maps/sixth.yaml \
  --out runs/cand_sixth_legacy.json ; echo rc=$?

# 3) 双站图不退化：carloop 的 tokens/stops/sides/degraded 必须与 prod 完全一致
python -B -X utf8 probes/probe_plan.py --src iso/cand/src/dzbrain \
  --robot iso/cand/config/robot.yaml --map iso/cand/maps/carloop.yaml \
  --out runs/cand_carloop_legacy.json
python -B -X utf8 probes/compare_runs.py runs/prod_carloop_legacy.json runs/cand_carloop_legacy.json

# 4) 单格靶图不变：field 的 54 token / E10* / degraded=false 与 prod 一致
python -B -X utf8 probes/compare_runs.py runs/prod_field_legacy.json runs/cand_field_legacy.json

# 5) 测试影响面：候选应只红 4 条（34/35 与 174/177），其余与 prod 相同
cd iso/cand && python -B -X utf8 src/dzbrain/tests/test_dual_shoot.py ; echo rc=$?
python -B -X utf8 src/dzbrain/tests/test_autoplan.py ; echo rc=$?
```

通过判据（硬）：步骤 1 的 rc=2 且文案含 `双射击停站应为 2 个，实际 1 个：B4`；步骤 2 的 rc=0 且 4 个字段全中；步骤 3/4 的「不同字段」只剩 `planner_sha256`；步骤 5 的失败条目**恰好**是上表 4 条。

## 回滚点

- **生产代码（若有人把候选合进去）**：`route/planner.py` 是**未提交**改动，**不能**用 `git checkout` 回滚（会连在途 CAR-DUAL 改动一起抹掉，HEAD 里没有它们）。正确回滚 = 用 preimage 覆盖：
  ```bash
  cp _tmp_session/AGC-RECTIFY-20261026/fix-plan/evidence/preimage_planner.py \
     dzrobot_v2/src/dzbrain/dzbrain/route/planner.py
  sha256sum dzrobot_v2/src/dzbrain/dzbrain/route/planner.py   # 必须回到 c14dadd4…
  ```
- **隔离副本**：`rm -rf _tmp_session/AGC-RECTIFY-20261026/fix-plan/iso/{prod,head,cand,pool}`（生产零影响）。
- **测试**：本卡未改任何测试文件，无需回滚。
- **配置**：本卡未改 `config/**`，无需回滚、无需冻结登记。

## 执行记录

环境：Windows / Python 3.12.10 / PyYAML 6.0.3 / `python -B -X utf8`（全程禁用字节码写入，生产树零写入）/ 子会话 `d55ee7b6-d7e5-42b3-bb62-bb9ae10a6483`。

| # | 动作 | cwd | rc | 证据 |
|---|---|---|---|---|
| 1 | 记录 preimage（planner / mission / 三张夹具 / robot.yaml / HEAD） | `dzrobot_v2` | 0 | `evidence/preimage_sha256.txt` |
| 2 | 建隔离副本 prod/head/cand（head = `git show HEAD:…planner.py`） | `fix-plan` | 0 | 三份 planner sha：`c14dadd4` / `c7f237b4` / `c4a89e5a` |
| 3 | 复现 prod 第六图 legacy | `fix-plan` | **2** | `runs/prod_sixth_legacy.json`（文案逐字） |
| 4 | 复现 prod 第六图 tasks | `fix-plan` | **2**（入口）/ plan_route 兜底 50 token | `runs/prod_sixth_tasks.json` |
| 5 | HEAD 同图 legacy/tasks | `fix-plan` | 0 / 0 | `runs/head_sixth_{legacy,tasks}.json`（68 token、B4\*） |
| 6 | 候选 A 三图 × 两 objective | `fix-plan` | 0 / 0 / 0 / 0 / 0 / 0 | `runs/cand_*.json` |
| 7 | 逐 token 比对（carloop / field） | `fix-plan` | 0 | `_tmp_session/AGC-RECTIFY-20261026/fix-plan/probes/compare_runs.py` 输出：不同字段只剩 `planner_sha256` |
| 8 | 候选 A 三分支文案直测 | `fix-plan` | 0 | `runs/degradation_branches.json` |
| 9 | `route_check` 跑候选第六图产物 | `iso/cand` | **0** | `runs/cand_sixth_route_check.log` |
| 10 | `sim_cli` 闭环跑候选第六图产物 | `iso/cand` | **0** | `runs/cand_sixth_sim.log`（108.24 s / 0.0285 m） |
| 11 | `route_check` 跑现役手写路线（F4\*+B4\*） | `iso/prod` | **0** | `runs/prod_mission_blocks_route_check.log` |
| 12 | 实验 B（允许雷格入池）三图 | `fix-plan` | 0 | `runs/pool_*.json`（carloop 57 token） |
| 13 | 测试面：prod vs cand × 4 套 | 各副本根 | prod 0/1/0/1；cand 1/1/1/1 | `runs/{prod,cand}_test_*.log` |
| 14 | 收口门禁三件（doc_policy / links / anchors） | `v1` / `dzrobot_v2` | 0 / 0 / 0（本卡 0 新增红） | [锚点原始报告](../../../../_tmp_session/AGC-RECTIFY-20261026/fix-plan/evidence/anchors.json) + 上方「未验证」末条 |

结论：**待证事实 1–4 全部成立；但「第六图单站是物理必然」被 F4 反证推翻**。候选 A 在隔离副本中解除整趟无解、单站显式降级，双站图与单格靶图零改动；采用它需要 Lead 先裁决那 4 条在途断言。

## Lead 复核：决策 A 落生产工作树（2026-10-27 22:2x · Lead 亲自执行，未 commit）

结论：**决策 A 已在生产工作树落地并通过生产代码复跑**。作者隔离副本的结论（解除整趟判死、单站显式降级、双站图零改动）**独立复现成立**；本卡「待 Lead 裁决的 4 条在途断言」按「保留意图、只换机制」已改写并跑绿。

| 项 | 落地身份 |
|---|---|
| 生产 `route/planner.py` | `c14dadd4…` → **`405d8e8d…`**（+137 行；`git apply -p1 patch/candidate_plan_fix.patch`） |
| 生产断言 4 条 | `test_dual_shoot.py` → `cc77bf4a…`；`test_autoplan.py` → `4da2a542…`（`patch/candidate_tests_fix.patch`） |
| 与作者副本一致性 | 三个文件 sha256 **与 `iso/cand` 逐字节相同** |
| 回滚点 | `_tmp_session/AGC-RECTIFY-20261026/lead-land/preimage/`（planner=**c14dadd4**、两条测试原始版）；**禁 `git checkout`**（未提交在途改动会被连带抹掉） |

**A/B 隔离复跑（两棵当前生产树仅差 planner.py，故差异只能归因于该文件）**

| 图/objective | pre（c14dadd4） | post（405d8e8d） |
|---|---|---|
| 第六图 legacy | **rc=2**、0 token、`双射击停站应为 2 个，实际 1 个：B4` | **rc=0、68 token、`B4*`、degraded=True** |
| 第六图 tasks | 兜底 50 token / 0 停站；入口 rc=2 | rc=0、68 token、`B4*`、degraded=True |
| carloop legacy / tasks | 62 / 63 token、F9*+C5* | **逐 token 相同** |
| field legacy / tasks | 54 E10* / 53 I9* | **逐 token 相同** |

**与 HEAD 对照（用户要求的 `lead_head.json` 口径）**：post 第六图 68 token 与 HEAD `c7f237b4` 的 `lead_head.json` **逐个 token 相同**（含 3 处 `:PIVOT`、`B4*`、`N5,y`）；差别只在语义——HEAD 是 `degraded=False`＋泛泛的「单侧射击停站」，post 是 `degraded=True`＋点名「只有 F4（雷格）、F7（视线被挡）被候选策略排除，要打另一面只能人工加星号 `F4*`」。

**门禁与回归**：`route_check` rc=0（雷 8/8、靶 2/2、射击点#48、24.85 m）、`sim_cli` rc=0（108.24 s、末点 0.0285 m、`diverged=false`，但**同报 `out_of_bounds=74`/`field_modeled=0` 存量项，不得当空间安全**）；`test_dual_shoot` **37/37**、`test_autoplan` **182/182**；规划相关 10 套 pre/post **无一条新增红**（corner_cut 89/91、stepper 117/118、bridge_gate 44/47、route 156/157 与 pre 逐条相同；`test_planner`/`test_mission`/`test_drone`/`test_incremental_plan` 仍因 `config/map.yaml` 已删而 FileNotFoundError）。§4 六门禁：gen_index rc0 / doc_policy GREEN / numbers rc0 / check.py GREEN / check_links broken 0 / **anchors rc=1（新增红 2）**——两条均在他人今日新卡 `lead-1023/DOC-LIB-TIDY-20261027.md`（正文裸写了两个 PowerShell 脚本名，无目录），**与本改动无关，Lead 未改他人卡**。

**留账风险**：① planner 行号 +137 ⇒ `docs/` 中约 84 条 `planner.py:<行号>` 锚点准确度下降（不进红，需单独授权批量重指）；② `mission.py` 的 `PlanOutcome` 无结构化 `degraded` 字段，可见性靠 CLI `summary()` + mission 日志（13.2a 已钉）；③ 未接车/未出光/未实车，`tracker-safe` 未验；④ A ≠ 第六图能双面打靶（要两站仍须人工在 `F4` 加星，或另立卡谈「雷格能否作停站」= 本卡实验 B）。

证据：[lead-land 复核报告](../../../../_tmp_session/AGC-RECTIFY-20261026/lead-land/REPORT.md)（其中列出逐条复跑命令；原始 JSON、route、sim 曲线与每道门禁日志同在其下的两个子目录里，按报告第 10 节索引取）。

## 未验证

- 只验 `engine=tour`（现役缺省，D-012）；`tracker-safe` 未验（现役图含 `:PIVOT`，D-012 记录其 0/5）。
- 只离线规划 + `route_check` + `sim_cli`；**未接车 / 未发 ROS / 未出光 / 未实车**；单站降级路线的实车时序（SETTLE→ACQUIRE→FIRE 30 s 死线）未测。
- 时间预算只做算术：sim 108.24 s；单站 +30 s ≈ 138 s；两站（F4+B4）+60 s ≈ 168 s / 180 s（余量 12 s）。B 的时间余量更紧。
- 未验证「雷格停站 30 s 对压雷判定/裁判口径」的现场影响（实验 B 的前提之一）。
- 收口门禁（20:1x 复核）：`check_doc_policy.py` **GREEN（红线 0 / 提醒 0）**、`tools/wf/check_links.py` **broken 0 / rc=0**、`check_anchors.py` **本卡 0 新增红 / 16 黄（全部指向 `_tmp_session` 证据路径）**。注：19:49 首次运行时该守卫因他人的未提交修改报 `NameError: name 'base_dead' is not defined`，20:1x 已恢复，故以恢复后的 GREEN 为准。

## 文档变更

新增本卡（R-8）。树与索引由 Lead 单写（本卡未改 `00_树.md`、未跑 `gen_index.py`）。
2026-10-27 22:2x Lead 追加「Lead 复核：决策 A 落生产工作树」段并勾选验收最后一项；同步更新 `00_树.md`（R-8 行＋日志一条）、重跑 `gen_index.py`（索引重建）、记 `TEAM_STATUS.md` 一条。生产 3 文件已改（未提交），证据在 `_tmp_session/AGC-RECTIFY-20261026/lead-land/`。

## 经验总结

- **现象**：第六图（一块纵向跨格板 D5+D6）在 `engine=tour` + 现役 robot.yaml 下整趟规划失败，报「双射击停站应为 2 个，实际 1 个：B4」，`fallback_to_tokens=false` ⇒ 不出车。
- **处置**：先分清「格数/板数」——`targets` 是靶格列表、无板实体，`len(targets)>=2` 对单块板恒成立；再把「停站个数」判据收敛到一处口径（1~2 个，2 个必须异侧），单站改为显式 degraded（返回值 + warnings + 三分支文案），不静默放过。
- **坑**：① 不能只看「判据太严」，要回头查**候选池**是否把合法解滤掉了（本例：雷格 F4 是现场正在用的对侧站）；② 在途改动的测试期望会伪装成「既有契约」，判断前先看 `git cat-file -e HEAD:…` 与 `git status`；③ 生产文件是未提交改动时，回滚只能按 sha256 覆盖，不能 `git checkout`。
