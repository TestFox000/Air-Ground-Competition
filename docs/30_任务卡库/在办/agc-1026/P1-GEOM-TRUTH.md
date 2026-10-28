---
卡号: P1-GEOM-TRUTH
ns: agc-1026
agent: 9aff0fda-1af0-4272-b40b-52a154e23a12
owner: 9aff0fda-1af0-4272-b40b-52a154e23a12
reviewer: lead
树节点: R-13
写域: "config/robot.yaml + src/dzbrain/dzbrain/core/config.py + route/physics.py + route/planner.py(默认值/注释) + 直接相关测试 + _tmp_session/P1-GEOM-TRUTH/ + 本卡"
status: in_progress
创建: 2026-10-27
---

# P1 · 几何真值修正（冻结区，用户一次性授权）

> 用户 2026-10-27 23:3x：「P1 一次性授权」。执行者 p1-geom；Lead 只做 frozen.json 登记与独立复核。

## 目标
把车宽占位 0.135 改为实测 0.17、桥室由整格无墙改为「开口 0.33 + 桥墙 0.02」、删除现实不存在的 0.0875 虚拟冗余墙；并同批完成 M1/S1 前置配置：drone.field_layout_wait_s 30→60、step.layout_ttl_s 4→0。

## 改动清单
- config/robot.yaml:959 width_m 0.135→0.17；:701 bridge_entry_redundancy_m→0.0；field 段加 bridge_opening_m:0.33、bridge_wall_m:0.02
- core/config.py:211,239 SCHEMA 同步；:566 wait 30→60；:573 ttl 4→0
- route/physics.py:625 桥室由 opening/wall 推导；多处 width_m 默认值 0.135→0.17（只改默认值）
- route/planner.py 默认值/注释算式（行号已漂移）
- 相关测试断言/夹具（列清单）

## 验收（全部机器可判）
- [x] V1 grep -rn "0.135" config src → 残留 3 处（2 处历史对照注释 + `mission.py:710` 兜底；sim.py 4 处已由 R-21 清零；运行期读 config=0.17，inert）
- [x] V2 config_selfcheck：exit0 / error 0 / warn 0（LAYOUT-MODE；Lead 复跑）
- [x] V3 40 套交集 PRE red=20 / FINAL red=20（零新增；本机多数套件被 `config/map.yaml` 有意删除阻塞，收口以 fixture 口径重跑）
- [x] V4 route_check **fixture 口径**：`--trajectory --map carloop` rc0 PASS；`ROBOT_MAP=map_sixth` legacy **rc0 / 雷 8-8 / 靶 2-2 / 桥洞 K5 K7**（默认无图不可跑属设计形态，禁恢复 map.yaml）
- [x] V5 sim_cli **118.12 s / 0.0300 m / 74**；隔离 PRE 树同三数（轨迹驱动，几何不可见）；旧基线 108.24 早于他人在途改动
- [x] V6 走廊位 **+80.0 mm**、45° 切内 **−20.0 mm**（PRE/POST 同值；Lead 复跑 PASS）
- [ ] V7 **未达标**：POST 余量 +5.9 mm（PRE +28.3）**仍放行**；5.9 mm < 追踪误差 50 mm ⇒ 工程上不可行但判据未翻；不为变绿改判据，记已知缺口（D-054）
- [x] V8 0.079→1 mm ok / **0.081→0 判蹭壁** / 0.080 边界；真值余量 0.080（Lead 复跑 PASS）

## 回滚
preimage 存 _tmp_session/P1-GEOM-TRUTH/preimage/；禁 git checkout；未 commit/未部署。

## 五项必填
- 读过的台账条目：D-044（提交/推送等用户令）、D-046（候选验收纠偏）、D-047（离线边界）；本卡关联下条 Lead 裁决。
- 复用检索：复用现有 route/physics、route/planner、core/config、main.py StepLoop、drone.field_layout 协议，不新建第二套同类工具。
- 新增轮子及理由：无（仅在本卡写域内改现有实现）；如需新模块，理由写在本节。
- 验收命令：见「验收」逐条，全部机器可判。
- 回滚点：见「回滚」。

## 执行报告
（p1-geom · 2026-10-27 23:35–2026-10-28 00:0x；未 commit / 未部署 / 未连车）

### 一、改了哪 4 个生产文件（只改默认值与几何推导，不改算法）

| 文件 | 改动 | preimage sha256（前 10） |
|---|---|---|
| `config/robot.yaml` | `:965` width_m 0.135→**0.17**；`:702` 新增 `bridge_opening_m: 0.33`/`bridge_wall_m: 0.02`；`:704` `bridge_entry_redundancy_m` 0.0875→**0.0** | `b26ab9f991` |
| `core/config.py` | `:211` width Spec→0.17；`:239` redundant Spec→0.0；新增两键 Spec；`:566` wait 30→**60**；`:573` ttl 4→**0** | `87b6bd1479` |
| `route/physics.py` | 新增 `DEFAULT_BRIDGE_OPENING_M=0.33`/`DEFAULT_BRIDGE_WALL_M=0.02` 与 3 个几何助手；`bridge_entry_walls()` 由 **opening/wall 推导**（碰撞面=内缘 ±0.165）；11 处 `width_m` 默认 0.135→0.17；推导数字 0.15542→0.16378、0.3029→0.3113、0.0775→0.095 | `6b1ec3f8ba` |
| `route/planner.py` | `_phys_params` width 默认→0.17 + 新增 opening/wall 两参；`BRIDGE_ENTRY_REDUNDANCY_M`→0.0；`_TSGrid` 真墙**无条件建**（不再挂 `red>0`） | `405d8e8d` |

测试（直接相关，5 个文件）：`test_physics`（0.16378/0.2325/0.31128/0.095/7.9cm边界）、`test_planner`（width 0.17/0.2325/0.3113/0.095）、`test_autoplan`（0.17）、`test_core`（wait 60.0）。

### 二、几何自洽（本卡的真值算式，三处同改才算改对）
```
格宽 0.35；墙厚 0.02 居中压在格边界线 ⇒ 墙心 = cx ± (0.33/2 + 0.02/2) = ±0.175 = 格线 ✓
⇒ 净开口 = 0.35 − 0.02 = 0.33；碰撞面 = 内缘 cx ± 0.165
⇒ 车宽 0.17 穿洞横向真值余量 = (0.33 − 0.17)/2 = 0.080 m   ← V8 的判据边界
旧实现：整格无墙(±0.175) + 0.0875 虚拟冗余 ⇒ 多给 2.75 cm，负余量也"看起来可行"
```
**一处偏离卡面文字（已按物理与 V8 判定，请 Lead 裁）**：卡面写"桥壁 x=cx±(opening/2+wall/2)"=±0.175，
但那是**墙心**；若把碰撞线段放在墙心，第 8 项负例（偏 0.081）会被判成**安全**（车缘 0.166 < 0.175），
与"真值余量 0.080"直接矛盾。故碰撞面取**内缘 ±0.165**，墙心 ±0.175 保留为 `bridge_wall_center_offset_m()`
并作自洽断言用。

### 三、验收逐条（命令 + 关键输出）

**V1 `grep -rn "0.135" config src` → 7 处，非 0（清单如下）**
| 位置 | 性质 |
|---|---|
| `planner.py:4095,4099` | 我写的**历史对照**文字（"车宽 0.135→0.17"），故意保留 ⇒ 注释例外 |
| `tasks/mission.py:710` | **写域外**：`_f((...), 0.135)` 兜底默认值 |
| `tools/sim.py:297,306,335,966` | **写域外**：4 处 0.135 默认值 |

⇒ V1 **未达 0**。mission.py/sim.py 的值只在"配置缺键"时生效（现场配置有键 ⇒ 运行时已用 0.17），
但按判据仍是红，需要 Lead 扩写域或另开卡。

**V2 `python tools/config_selfcheck.py` → exit 0，error 0 / warn 0**
```
[布局模式] 地图文件不存在：...\config\map.yaml
结论：配置 error 0 / warn 0 —— 仅配置可用；空间证据未校验（布局模式）→ 不是发车放行
RESULT_JSON={"ok": true, "launch_go": false, "status": "LAYOUT-MODE", "warn": 0, "layout_mode": true}
```
新增两键已进 SCHEMA（否则会按"未登记键"记 warn）。**注**：`config/map.yaml` 是**他人在途删除态**（frozen.json 已登记），我没有恢复它。

**V3 全量回归（本仓 test_*.py 皆**脚本式、无 pytest**；用 `_tmp_session/P1-GEOM-TRUTH/run_suite.py` 逐套跑）**
```
对照口径：改动前后**交集 40 套**
PRE   red = 20
FINAL red = 20
NEW RED  : 无            ← 关键判据
FIXED RED: 无
```
过程中我改 `drone.field_layout_wait_s` 30→60 曾打破 `test_core` 的写死断言（`默认 30.0 s`）
⇒ 已把该断言改成 60.0 并保留"范围 1..120 / 越界必拦"的意图，`test_core` **56/56 全绿**。
补齐末尾 3 套（聚合器被我提前 kill 未跑到）：`test_telemetry` 80/80 ✅、`test_tracker` 139/139 ✅、
`test_stepper` **117/118**（唯一红 `12.11`，与 PRE **同一条**）。
⇒ **零新增红**（存量红 20 套的根因绝大多数是 `config/map.yaml` 被删：
`FileNotFoundError: ...config\map.yaml`，详见 §五 并发）。
修完三处**陈旧断言**后 `test_physics` 只剩 1 条红（见下 R2），其余套件红项与 PRE 逐条相同
（`test_bridge_gate/corner_cut/drift/route/sim/sim_guard/deploy/motion_limit/path_overlap/calib_backfill`
等 18 套存量红，**绝大多数根因是 `config/map.yaml` 被删**：`FileNotFoundError: ...config\map.yaml`）。

**V4 `python tools/route_check.py` → exit 1，雷/靶/桥洞全部「未提供，跳过」**
```
✅ 全部 token 合法  普通 68 | 倒车 0 | 射击 2 | 折返 3
✅ 挡板不触线 0.1750 m ≥ 0.0775 m          ← 门槛仍是 0.135 口径（route_check 自带常量）
❌ 障碍净空不足  直段 0.000 < 0.215 m（航段#22 距 障C8）
   桥洞覆盖/雷覆盖/射击点距离 → 未提供，跳过（config/map.yaml 删除态，legacy 模式无 --map 入口）
```
⇒ **V4 不达标**。两条独立原因都**不是我引入**：① `航段#22 距 障C8 0.000 < 0.215` 是本轮之前
就记在 agc-1026 树上的存量红；② 缺图导致 8 雷/2 靶无法统计。且 0.215 这个门槛说明 route_check
用的是它**自己的** `VEHICLE_WIDTH_M=0.135`（`tools/route_check.py:105`），写域外，未改。

**V5 `python -m dzbrain.tools.sim_cli` → exit 0**
| 指标 | 卡面旧基线 | 本卡 PRE（隔离 preimage 树实跑） | POST |
|---|---|---|---|
| 总时长 | 108.24 s | **118.12 s** | **118.12 s** |
| 末点误差(真值) | 0.0285 m | **0.0300 m** | **0.0300 m** |
| 越界拍数 | 74 | **74** | **74** |
**PRE 与 POST 三数完全相同**，且 PRE 也不是卡面的 108.24 —— 说明：① 卡面旧基线早于他人后续改动；
② 这三数是**轨迹驱动**（pure-pursuit 输出），车宽/桥墙只进碰撞计数器，不改变轨迹 ⇒ 几何真值
在这三个数上**本来就不可见**。已用隔离树实跑对照（`_tmp_session/P1-GEOM-TRUTH/pre_tree/`，
`half_diag=0.15542` 证明确实载入的是 preimage 代码），不是"看着像"。

**V6 拐角 J8→J7→I7（physics.py 重算，非估计）**
```
J7=(1.5750,1.5750)  挡板中心线 x=1.7500 [0,3.8500]
yaw=90°（走廊位，车沿 y 走）  车体↔挡板 = +80.0 mm   ← = 真值余量 0.080
yaw=45°（切内，历史口径）      车体↔挡板 = -20.0 mm   ← 侵入，且 SAT 饱和(=挡板全厚 20 mm)
```
负号结论：**在格心路径上按 45° 切内，车体已经把 2 cm 挡板整条盖住**，PRE/POST **同值**
（yaw=45° 时横向占位由**车长**项主导，与车宽无关）⇒ 该姿态对任何车宽都不成立，
不是本次改动造成的，也不能靠改车宽"修好"。

**V7 负例：洞口前 1 格（0.35 m）起转 → 卡面要求"判不可行"**
```
R_min = 0.157/tan(30°) = 0.2719 m
yaw=45° 横向余量：PRE +28.3 mm → POST +5.9 mm（收窄 4.8 倍）
最坏姿态：PRE +19.6 mm@64° → POST +1.2 mm@59°
结论：仍 +5.9 mm > 0 ⇒ **仍放行**
```
⇒ **V7 未达标（如实报，不为了变绿改判据）**。真值几何把余量压到 5.9 mm < 纯追踪横向误差 RMS 50 mm，
工程上等价于不可行，但**判据层面没有翻成"不可行"**。要真正翻，只有两条路，都属**规划策略**而非默认值，
超出本卡写域，请 Lead 裁：
① 把 `planner.bridge_approach_align_penalty_m` 抬到足以让该形态永不被选（仍是惩罚，不保证无解）；
② 把 `entry_gate_turn` 从"顶点摆正扫掠"扩成"整个转弯弧扫掠"（能判死，但要重算性能与既有 62 点路线）。

**V8 负例：桥心横向偏移 → `_tmp_session/P1-GEOM-TRUTH/accept_v678.py`（exit 判据）**
```
偏移 0.000 → clearance 0.080000  ok
偏移 0.079 → clearance 0.001000  ok        ← 负例对照，防判据恒真
偏移 0.080 → clearance 0.000000  蹭壁      ← 真值余量边界
偏移 0.081 → clearance 0.000000  蹭壁      ← 卡面要求
PASS V8.1 margin==0.080 | V8.2 0.081 蹭壁 | V8.3 0.079 不蹭壁 | V8.4 0.080 边界蹭壁
```
（PRE 几何下 0.079/0.080/0.081 **全部 ok** —— 这正是要修掉的假余量。）

### 四、必须由 Lead 处置的 3 条风险（都不在本卡写域）

**R1（高）`tools/sim.py` 的桥洞判据会因冗余归零而整体**消失**。**
`:390` `self.bridge_modeled = 1 if (self.spec.bridges and self.bridge_entry_redundancy_m > 0.0) else 0`。
现场 `bridge_entry_redundancy_m: 0.0` ⇒ `modeled=0` ⇒ 不建墙、不统计 `bridge_steps`。
旧代码这么做是因为"零长墙会假阳性"（`:383-389` 注释），而**本卡已把墙段改成非退化的真实段**，
那个旁路不再需要。建议一行修：`:390` → `= 1 if self.spec.bridges else 0`，并把 `:349` 默认
`0.0875→0.0`、`:976` `cfg.get(..., 0.0875)` 同步。**未修之前，sim 对桥洞是"没测"，不是"测了没事"。**

**R2（中）`tools/route_check.py` 与 physics 常量已分叉，且由测试机器判出。**
`test_physics` 第 6 节对拍断言现在**红**：
`FAIL route_check 读到的物理常量与本模块一致（8 项） [0.28/0.135/0.035/...]`。
根因 `tools/route_check.py:105` `VEHICLE_WIDTH_M = 0.135`（+ 依赖它的 `vehicle_half_diagonal_m`
默认与门槛文案 0.215/0.0775）。该断言**就是为抓这种分叉而存在的**，我没去改弱它。
另：`route_check.py:1405` 调 `bridge_entry_walls(b, 0.0)` **不传** opening/wall，走 physics 默认
（当前与 config 同值 0.33/0.02，暂时不出错，但只要现场改 config 就会静默分叉）。

**R3（低）`tasks/mission.py:710`` 兜底默认仍 0.135**（写域外）。运行时读得到 config，只在缺键时偏。

### 五、并发（不是我改的，但影响"零新增红"的读数，如实记）
- `src/dzbrain/dzbrain/main.py` 在我 POST 跑测期间（**mtime 23:56:43**，hash `7a0f2cb3…`）被他人改成
  `:1406` 调 `self._footprint_check(plan)` 而**该方法当时不存在** ⇒ `test_stepper` 由
  PRE 的 `117/118(12.11)` 变成 `AttributeError` 崩溃。我**没有**碰 main.py。
- 同一窗口他人新增 `test_drone_corridor.py`（48/48 绿），故 POST 套数 42→43。
- `config/map.yaml`/`config/map_v3.yaml` 是**他人在途删除态**〔已删〕，是 18 套存量红的主要根因；未恢复、未还原。

### 六、回滚
`_tmp_session/P1-GEOM-TRUTH/preimage/`（含 `SHA256SUMS.txt`，9 个文件：4 生产 + 5 测试）；
隔离 PRE 树 `_tmp_session/P1-GEOM-TRUTH/pre_tree/` 可复跑对照。未 commit / 未推送 / 未部署 / 未连车。

### 七、复跑命令一览
```bash
cd W/dzrobot_v2
grep -rn "0.135" config src --include=*.py --include=*.yaml | grep -v __pycache__   # V1
python tools/config_selfcheck.py                                                   # V2
python tools/route_check.py                                                        # V4
PYTHONPATH=src/dzbrain python -m dzbrain.tools.sim_cli                             # V5
python ../_tmp_session/P1-GEOM-TRUTH/accept_v678.py                                # V6/V7/V8
python ../_tmp_session/P1-GEOM-TRUTH/run_suite.py <outdir> <tag>                   # V3
```

---

## Lead 复核（2026-10-28 01:0x · §11 独立复跑 + R-21 收尾）

- 复跑：`config_selfcheck` **exit0 / error0 / warn0**（LAYOUT-MODE）；`accept_v678.py` V6/V8 全 PASS、**V7.2 FAIL**；`sim_cli` **118.12 s / 0.0300 m / 74**（与 pre 逐项相同；`field_modeled=0`，桥洞指标不在台上）；`route_check --trajectory --map <carloop fixture>` **rc0 / STATIC GEOMETRY PASS**。
- R-21 补掉两处写域外高危：① `sim.py` bridge_modeled 不再挂 redundancy（探针：偏移 0.09 m → **bridge_steps=21**，0.08 m 内 0 拍）；② `route_check.py` 车宽 0.135→0.17、vehicle 段改读 `/dzactuator`、桥墙随配置传 opening/wall ⇒ legacy + sixth fixture **rc0 / 雷 8-8 / 靶 2-2**。
- 裁决：**P1 几何真值接受**（V2/V5/V6/V8 绿；V1 残留 3 处 inert；V3 采信交集 20/20；V4 以 fixture 口径通过）。**V7 未达标、不勾**：+5.9 mm 仍放行是已知缺口，禁止为变绿改判据；后续路线若出现「洞口前 1 格起转」按不可行处理，翻案走抬惩罚或整弧扫掠（另卡）。
- 未提交/未部署/未连车；回滚 `_tmp_session/P1-GEOM-TRUTH/preimage/`（P1）与 `_tmp_session/R-21/preimage/r21.diff`（R-21），禁 `git checkout`。

