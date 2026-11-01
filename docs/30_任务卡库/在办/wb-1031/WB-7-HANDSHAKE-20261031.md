---
卡号: WB-7-HANDSHAKE-20261031
ns: wb-1031
agent: wb-doc
树节点: wb-7
kind: implementation
status: pending
创建: 2026-10-31 21:2x
owner: 待领
reviewer: wb-reviewer
objective: 【2026-10-31 22:3x 范围重划】握手项（`config/robot.yaml` 加 `step.layout_ttl_s: 0.0`）已随 HEAD 落地并实测 `gate_policy=warn`、`test_stepper 118/118` ⇒ 本卡只做剩下的三块：① tier-0 守护套件（test_sim 106/113、test_sim_guard 106/110）的陈旧钉按现 68 点路线/现出厂常量重钉，逐条分「陈旧钉 / 真缺陷 / 如实钉住」三类；② 本线文档真源收口（能力索引 tier-0/tier-1 并列关系、复用清单登记、06 册置灰口径核对、`sim_map_render.py` 处置）；③ **禁止二次修改 `config/robot.yaml`**（该行已提交，再改即返工）。
acceptance:
  - `python tools/config_selfcheck.py --config config/robot.yaml` 退出码 0（0 error 0 warn）
  - `git status --short config/` 为空（证明本卡未二次修改配置）+ 实测 `gate_policy(cfg)==warn`
  - `grep -n layout_ttl_s` 三处口径一致（SCHEMA / 配置 / 运行时日志）
  - test_stepper 保持 118/118 零红（本卡不因握手再动它；若因文档改动翻红须改回）
  - tier-0：`test_sim.py`、`test_sim_guard.py` 失败数**只减不增**；每条被处理的红必须归入「陈旧钉（重钉）/ 真缺陷（登记另立卡）/ 如实钉住（保留并写理由）」三类之一，禁止删断言或放宽判据换绿
  - `tools/sim_map_render.py` 的抽样口径给出处置（修 or 明确标 legacy），并同步能力索引:13 的「唯一渲染器」登记与现状一致
  - 复用登记：本线新增件（`tools/sim_world.py`〔未实现〕/`sim_rig.py`〔未实现〕/`map_synth.py`〔未实现〕/`track_sweep.py`〔未实现〕）全部进 `复用清单.md` + `能力索引.md`（tier-0/tier-1 并列关系写清，避免后来人再造第四套）
  - `python tools/wf/check.py` GREEN 且 `check_reuse` 无新增未登记
verify:
  - cd D:/User/Desktop/比赛存档/v1/dzrobot_v2 && python tools/config_selfcheck.py --config config/robot.yaml; echo rc=$?
  - cd D:/User/Desktop/比赛存档/v1/dzrobot_v2 && git status --short config/ ; echo "(空 = 未二次改配置)"
  - cd D:/User/Desktop/比赛存档/v1/dzrobot_v2 && PYTHONPATH=src/dzbrain PYTHONIOENCODING=utf-8 python -c "import sys;sys.path.insert(0,'src/dzbrain');from dzbrain.core.config import Config;from dzbrain.main import gate_policy;print(gate_policy(Config.load('config/robot.yaml')))"
  - cd D:/User/Desktop/比赛存档/v1/dzrobot_v2 && PYTHONPATH=src/dzbrain PYTHONIOENCODING=utf-8 python src/dzbrain/tests/test_stepper.py | tail -2
  - cd D:/User/Desktop/比赛存档/v1/dzrobot_v2 && PYTHONPATH=src/dzbrain PYTHONIOENCODING=utf-8 python src/dzbrain/tests/test_sim.py | grep -E "结果：|FAIL" 
  - cd D:/User/Desktop/比赛存档/v1/dzrobot_v2 && PYTHONPATH=src/dzbrain PYTHONIOENCODING=utf-8 python src/dzbrain/tests/test_sim_guard.py | grep -E "结果：|FAIL"
  - cd D:/User/Desktop/比赛存档/v1/dzrobot_v2 && python tools/wf/check.py | tail -3; echo rc=$?
inScope: dzrobot_v2/src/dzbrain/tests/test_sim.py；dzrobot_v2/src/dzbrain/tests/test_sim_guard.py；dzrobot_v2/docs/{复用清单,能力索引}.md；docs/20_资料索引库/空地链路/**；docs/20_资料索引库/车端/**（仅索引行）；`_tmp_session/wb-1031/wb7/`
outOfScope: **dzrobot_v2/config/robot.yaml（`layout_ttl_s: 0.0` 已随 HEAD 落地，本卡禁止写入，再改即返工）**；dzrobot_v2/src/dzbrain/tests/test_stepper.py（握手后实测已 118/118，不属本卡红）；dzrobot_v2/src/dzbrain/dzbrain/**（含 main.py 的 gate_policy 实现）；插件目录；`tools/sim*.py` 的实现逻辑（处置结论写文档，改动另立卡）
---

# WB-7 · tier-0 重钉 + 文档真源收口（握手项已落地，范围重划）

> **2026-10-31 22:3x captain 重划**：本轮实测推翻卡面原判断——`config/robot.yaml:218` 的 `layout_ttl_s: 0.0` **已在 HEAD 里**（`git show HEAD:config/robot.yaml` 同一行、`git status --short config/` 为空），`gate_policy(Config.load(...))` 实测 = **warn**，`test_stepper` 实测 **118/118 全绿**（rt-gate 预告的 10.x/14.x 翻红**没有发生**）。⇒ 原任务①「加一行握手」与「改判 stepper 断言」两项**已完成或不需要**，本卡收窄为 tier-0 重钉 + 文档收口，且**禁止再写 config/**。

## 读过的台账条目

D-026（不把仿真绿当实车绿）；D-055 / B 区裁决（一场一图、取消保质期 `step.layout_ttl_s→0`）；D-063（本线依据）；AGENTS §1 铁律⑤「一次一变量」——故握手与重钉分两步跑，不混在同一次改动里判因。

## 为什么这一行必须你来点头（已在册事实，非猜测）

rt-gate 交付报告风险①实测：`python -m dzbrain.main` 起动打印 `policy=hard explicit_ttl=False`。我本轮复核并**更正 rt-gate 的一句归因**：不是「`Config.get` 对缺失键不回落 SCHEMA」——`core/config.py` 的 `Config.get` 对缺失键**会**回落 SCHEMA 默认（`:790` 区）。真实机制是 `gate_policy()` 用 `_cfg_explicit()`：**要求配置里显式写了这一行才算表态**，没写就判 hard，且 `main.py:533` 注明这是刻意的（「配置没表态 ⇒ 语义不漂移」）。

⇒ 所以这不是 bug，是**一次需要人授权的握手**：A 区（互锁降级）与 B 区（一场一图）是同一次裁决的两半，握手点就是配置里那一行。`config/**` 属冻结区，我不擅自加。

## 副作用清单（必须预告，不许静默）

加这一行会让依赖旧拦停语义的断言翻红：`test_stepper` 10.x / 14.x（rt-gate 已预告）。处置原则 = 「**保留意图、只换机制**」（与 R-8 断言改写同一口径），**禁止删断言换绿**。

## tier-0 重钉为什么排在这里

本轮测试网审计实测：`test_sim` 102/113→106/113、`test_sim_guard` 100/110→106/110 在审计窗口内被并发会话持续移动（18:04/18:08/18:12/18:32 分别改过 bridge_gate/physics/link_e2e/route）。**在被改写的基线上钉数值 = 把钉子钉进沙**。等握手落地、口径固定后再钉，本卡就是那个"口径固定"的时点。

## 文档收口三处（现存与现状矛盾，须人裁定后写）

| 项 | 在册说法 | 现状实测 | 处置 |
|---|---|---|---|
| 渲染器唯一权威 | `能力索引.md:13`：`tools/sim_map_render.py` 唯一渲染器 | 实为两套；python 侧默认 `--map` 指已删图（实测 rc=1）、零测试引用、抽样 bug 仍在（`:272 i%4` 套 `:277 i%125` ⇒ 实际 500 拍=10 s，注释写 2.5 s）；插件 SVG 已自行修正 | 本卡给出处置并同步登记 |
| 仿真台唯一权威 | `能力索引.md:12` 插件四文件 | 现行确为 `dsh-dzairground-ssh`，但没挂进 bundles（verify FAIL） | 随 wb-6 enable 后补登记 |
| tier-0/tier-1 关系 | 无此概念 | 本轮结论：`sim_cli` 保留为回归基线、rig 为闭环台架 | 新增登记行，写明「不得互为替代品」 |

## 回滚点

`config/robot.yaml` 与三个测试文件改前存 `_tmp_session/wb-1031/wb7/preimage/` + sha256；逆转 = 删那一行（回到 `policy=hard`，行为与今日一致）+ 还原测试文件。禁止 `git checkout`（该文件另有他人 in_flight 登记，条目见 `frozen.json`，不动其内容）。

## 执行报告（wb-doc · 2026-11-01 01:2x · attempt f2a11c27）

### 0. 前置：9 脏文件归属判定（只读 diff，未 checkout/未提交/未格式化）

| 文件 | 归属 | 判据 |
|---|---|---|
| `src/dzbrain/tests/test_sim.py`<br>`src/dzbrain/tests/test_sim_guard.py` | **本卡（wb-7）** | diff 内出现 `wb-7` 12 处；mtime 00:48/00:50（同一会话在途） |
| `src/dzbrain/dzbrain/route/parser.py` (+28/−4) | wb-1（冻结区点名豁免 `_read_token_file`） | mtime 21:44；`frozen.json` 已登记 in_flight |
| `test_incremental_plan / test_link_e2e / test_motion_limit / test_path_overlap / test_planner / test_route` | 并行会话（非本线） | mtime 18:12–18:38，早于本线开工（21:2x） |
| `tools/calib_backfill.py` (+1/−1) | 并行会话 | mtime 18:31 |
| `docs/复用清单.md` / `docs/能力索引.md` | **本卡写域**（树 §写域分配已列） | — |

⇒ 结论：**本卡实际只写 5 个文件**（两张测试 + 两篇索引 + 06 册），其余 7 个脏文件一个字节没动。

### 1. tier-0 三分类（A 块）

基线（本卡自己复跑，**不拿 HEAD 当基线**）：
- `test_sim.py` → **108/114 通过，6 失败**（卡面原写 106/113 是 00:48 改写前的旧数）
- `test_sim_guard.py` → **109/111 通过，2 失败**（卡面原写 106/110 同上）
- 与在途 `wb7/out/{ts,sg}_final.txt` 逐项 diff：**FAIL 集合完全相同**（结论独立复现）

| # | 断言 | 分类 | 判读理由（一句话） |
|---|---|---|---|
| 1 | `4.2 无越界拍` → 拆 **4.2a/4.2b** | **陈旧钉（重钉）** | 68 点夹具带 3 处 180° 折返（I4/G4/A1），sim 未建模 PIVOT ⇒ 纯追踪必画 2R_min=0.544 m 弧；A1 是角落格 ⇒ 必顶出 0.60 m 窗。重钉为**结构判据**（越界拍必须落在折返弧窗内 + 单拍出界 ≤1/3 格），实测 48 拍全在窗内、最大 0.0359 m |
| 2 | `5.4b 无 ≥135° 折返` | **陈旧钉（重钉）** | 同一事实。判据换成**真正要守的不变量**：任何 ≥135° 顶点必须在路线里声明 `:PIVOT`（漏声明=规划漏标）。实测 pivots=[A1,G4,I4] 且全部已声明 |
| 3 | `8.13 44 航点/15.050 m/0 折返` | **陈旧钉（重钉）** | 夹具换 68 点/24.850 m/3 折返（D-062 夹具化）。重钉为实测真值，非放宽 |
| 4 | `10.7a/b`（K9 通格写死） | **陈旧钉（重钉）** | 本图 `bridges=[K5,K7]`，写死 K9 是"测非通格横穿"。改为**从 `_FIELD.bridges` 派生**（t20 教训：写死旧图格名一换图就集体假红） |
| 5 | guard `2.1/2.2/2.3b/2.5/2.5b` | **陈旧钉（重钉）** | 同 #3：44→68 点、15.050→24.850 m、停站 (29,32,43)→(44,48,67)、D3*→F4*、G3→B4 |
| 6 | guard `2.8` → `2.8a/2.8b`、`2.14` | **陈旧钉（重钉）** | 与 #1 同机理同形态，但**不做跨文件 import**（免得一处改口径悄悄改掉另一处）。实测 74 拍全在弧窗内、最大 0.0818 m（1.43× 余量） |
| 7 | guard `10.0 config/map.yaml 可加载` | **陈旧钉（重钉）** | 该文件已随 D-062 删除 ⇒ 改指夹具 `map_field7_t24.yaml` |
| 8 | `6.7 ε=+0.18 锚定无实质作用` | **真缺陷（登记不修，另立卡）** | 实测有锚定 0.6099 vs 无锚定 0.4595 m（**1.33×，锚定把误差搞坏 33%**）；probe10 拆因：可修窗口只有 **2/2500 拍**（`anchor_radius_m=0.12` 门限）。**不是数字陈旧**——t22 在 0.35 巡航下已是 1.26×，方向早已翻转，换回 44 点也回不去 |
| 9 | `6.11 噪声 0.005 跑完但打折` | **真缺陷（登记不修）** | probe7 逐拍拆因：末 28 拍 `s_truth` 恒 18.90 m（投影弧长不前进）、真值从 C1 滑到 D1，s=18.90 正落在 **A1 折返段**。⇒ 与 #10/#11 同族 |
| 10 | `6.12 噪声 0.01 拐角绕圈` | **真缺陷（登记不修）** | s_truth 恒 17.85 m、落在 **C7→C6→C5 的 90° 锯齿段**（最近航点 #30 C6 0.228 m）⇒ **不同子族**：90° 锯齿连续拐角够不到，属"够不到就重规划"兜底缺失 |
| 11 | `7.4 / 7.5` 延迟 0.2/0.8 s | **真缺陷（登记不修）** | 两档**同一失锁点**（s_truth 恒 19.25 m、块恒 A1）⇒ 失锁与延迟量级无关、与折返几何有关。原注记"44 点已消除该折返区"在 68 点上失效，但**不能把判据翻成"发散也算过"** |
| 12 | `10.8b 远程路线不碰靶板` | **真缺陷（登记不修）** | 压靶 95 拍、净距 0.0000 m；probe4 落点簇 D7 86/E7 5/E6 18 拍。机理：`D7` 与靶格 `D6` 同列差一行 ⇒ 车体 x 向名义间隙仅 0.035 m |
| 13 | guard `10.2 硬违规全为 0` | **真缺陷（登记不修）** | 挡板触线 176 拍（最近净距 **0.0012 m**）/ 压靶 95 拍 / 越界 74 拍。前三个（碰撞/K封死/K非通格）**仍是有效判据且已绿** |
| 14 | guard `10.3 盖雷8/8 且硬违规0` | **如实钉住（保留复合形态）** | **盖雷半边是绿的（8/8，最远雷心距 0.0016 m）**；红的只是硬违规半边。不拆成两条：拆出来的那条与 10.2 **逐字同判据**，会把红数从 2 虚增到 3。已在 detail 与注记里写明"红≠雷没盖住" |

**禁止删断言/放宽判据的机器证明**：断言计数 `test_sim` **106→107**、`test_sim_guard` **106→107**（只增未减）。
diff 里 13 行"消失"的 check 标题全部是**改写/拆分**（4.2→4.2a+4.2b、2.8→2.8a+2.8b 等），无一条净删除。

**为什么 6 条真缺陷保持红而不是洗绿**（统一判据，D-026 子约束）：这 6 条的红**正是 D-063 判据①（生产线程在环）要建台架的理由**——sim 未建模 PIVOT、任务层无"够不到就重规划"兜底，是**模型边界的事实**。把"跑不完也算过"钉成预期 = 让这条反常失去机器信号。

### 2. 文档收口（B 块）

| 项 | 处置 |
|---|---|
| 能力索引 tier-0/tier-1 并列 | `:11` 改标 **tier-0 · 回归基线**（冻结、台账 1150+ 跑次可比性载体、无生产状态机）；`:12` **新增** tier-1 闭环台架行（wb-core/WB-2），写明"**并列，不是替代品**" + 四条判据 + 禁塞回 `SIM_RESULT_JSON` |
| 复用清单登记 | 新增 §「2026-10-31 wb-1031」表：`sim_world`/`sim_rig` **已落地**；`map_synth`/`track_sweep` 按 wb-4 占位（owner wb-sweep、状态〔未实现〕）；`wb_events`（wb-3）；**补登 wb-core 请求的 `wb_rig/` 7 文件**（`__init__`/`scenarios`/`arms`/`actuator`/`sched`/`counters`/`metrics`），每个带职责+禁止事项 |
| `sim_map_render.py` 处置 | **明确标 legacy**（不修，修/删另立卡）。三条实测：① `--map` 默认写死 `WS/config/map.yaml`（`:341`），文件已删 ⇒ 给 `--out` 也在 **`:165`** 抛 `FileNotFoundError`（裸跑先被 argparse 挡成 rc=2，别误判成"地图缺失"）；② 零测试引用（`src/`+`tools/` 除自身与 `wf/repo_index.py` 登记行外无命中）；③ 相位 bug lcm(4,125)=**500** ⇒ 车身每 500 行=**10 s** 画一个，注释写 2.5 s（差 4×）。**能力索引**:13「唯一渲染器」已同步为 legacy + 现行权威改为插件侧 SVG 渲染（全路径登记见能力索引该行） |
| 06 册置灰口径 | **实测校正了一处与任务书字面不符的地方**（见下） |

**★ 06 册口径校正（这是本卡的一个实质发现，与任务书字面"越界类必须 null"不同）**：任务书写「`field_modeled=0` 时越界类必须 null 而非 0」，但 probe12 实测（可复跑）：

```
键数: 未建模 62 / 已建模 82     键集差 = 20（全是 mine_/obstacle_/barrier_/board_/k_/bridge_）
out_of_bounds_steps        未建模=74.0  已建模=74.0  同值
belief_out_of_bounds_steps 未建模=74.0  已建模=74.0  同值
bounds_margin_m            未建模=0.6   已建模=0.6   同值
field_stop_t               未建模=-1.0  已建模=-1.0  同值
```

源码归因：`sim.py:1561` 只在 `obs is not None` 时 `metrics.update(obs.metrics())`，未建模只补 `field_modeled=0.0`（`:1594`）；而越界走 `geo.in_bounds(x,y,margin)`（`:1336-1337`/`:1514-1516`），**与场地建模无关**（与 Safety 第②道检查同源）。
⇒ **"置 null" 的只有那 20 个场地键**（真·不存在）；**越界类必须保留真值**——把它抹成 null 是"把事实藏起来"，`field_stop_t=-1` 也是有效值。已在 06 册 §2 与复用清单里双向写清。

### 3. 验收（真实输出与 rc）

| 命令 | 结果 | rc |
|---|---|---|
| `test_sim.py` | **108/114 通过，6 失败**（FAIL 集合与在途一致） | **1**（6 条按三分类**故意保持红**） |
| `test_sim_guard.py` | **109/111 通过，2 失败** | **1**（同上，登记不修） |
| `test_stepper.py` | **118/118 全部通过 ✅** | 0 |
| `config_selfcheck.py --config config/robot.yaml` | **error 0 / warn 0**，`status=LAYOUT-MODE` | **3**（`AGENTS.md` §判据：exit 3 = 布局模式，配置可用但空间未校验；**HEAD 配置同样 rc=3**，已对拍证明） |
| `tools/wf/check.py` | **GREEN** | 0 |
| `tools/wf/check_reuse.py` | 新文件 9 / **未登记 0** | 0 |
| `gen_index` / `check_doc_policy` / `check_anchors` / `check_numbers` / `check_links` | 全 GREEN（0 断链 0 新增红） | 各 0 |
| `git status --short config/` | **空**（零写入） | — |
| `gate_policy(cfg)` | **warn**；`step.layout_ttl_s` = **0.0** | — |

**★ check.py 由 RED(1)→GREEN 的直接原因**：wb-core 请求登记的 7 个 `tools/wb_rig/*.py`——`check_reuse.py` 的判据是"文件名出现一次即算登记"，登记在 wb-7 写域（我改），wb-core 自己改不了。

### 4. 边界与未证

- **零写入**：`config/**` 空、`src/dzbrain/dzbrain/**` 未碰（parser.py 是 wb-1 的在途，我没动）、插件目录未碰、未 commit/未 push。
- **未证**：6 条真缺陷的**根治**不在本卡（只登记 + 给出拆因与处置归属：wb-2 台架复跑 / wb-4 扫掠足迹 / wb-5 判是否属 16 条案例之一）。
- 改后 5 文件的 sha256 见下节（供 reviewer 复核）。

### 5. 改后 sha256（reviewer 复核用）

```
627deac9…4532d  dzrobot_v2/docs/能力索引.md
5720683f…13350e dzrobot_v2/docs/复用清单.md
6bc78390…35c88  docs/20_资料索引库/空地链路/06_仿真台契约与版本钉扎_20261024.md
358c777e…51bd   dzrobot_v2/src/dzbrain/tests/test_sim.py        （本卡在途件，非我原创）
05e8ee5a…33eba  dzrobot_v2/src/dzbrain/tests/test_sim_guard.py （同上）
```


## 开工前置（强制，captain 2026-10-31 21:4x 追加）

动手前先读 `00_树.md` §开工前置：`dzrobot_v2` 有 9 个上一会话留下的**未提交脏文件**（最后写入 18:12–18:38，v1 侧干净），其中若干正压在本卡写域上。**三步**：① 跑测试前先记录当前通过数当基线（别拿 git HEAD 当基线，否则会把别人的改动算成自己引入的红）；② `git diff` 只读判明归属并写进本卡「## 执行报告」；③ 禁止 `checkout`/`stash`/`restore` 清理、禁止顺手提交、禁止格式化；CRLF 文件保持原行尾。
