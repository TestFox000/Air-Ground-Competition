---
来源: 用户 2026-10-25 循迹替换交接说明
取证: 实/算/未
更新: 2026-10-25
取代: 无
status: in_progress
ns: traj-1025
agent: /root/baseline
owner: /root/baseline
reviewer: /root
树节点: TR-5
写域: dzrobot_v2/tools/route_check.py, dzrobot_v2/src/dzbrain/tests/test_physics.py; 本卡
---

# TRAJ-VERIFY-20261025 · 连续轨迹物理验收CLI

## 1 目标与台账
连续轨迹物理验收CLI。已读 D-008/D-012/D-023/D-027/D-028/D-034/D-041：冻结登记、保留 planner、旧红分类、阿克曼前提、卡树隔离、双远端分叉不得擅自强推。用户本轮授权替换循迹，但不授权部署。离线候选通过前不切现役默认。

## 2 复用检索
已查 doc_search「连续轨迹 阿克曼 循迹」、复用清单、能力索引、目录索引；源码搜索 trajectory/clothoid/dubins/quintic/jerk 无现成实现。复用 Path/Route/geometry/physics/route_check/现有 sim 与现有 tests 树。仿真不能证明深度相机实测 RMS。

## 3 新增轮子及理由
无新文件、无第二套物理实现。扩展既有 `dzrobot_v2/tools/route_check.py` 的显式 `--trajectory` 模式，复用已有 planner CLI、Trajectory、route.physics；新增验收留在既有 `dzrobot_v2/src/dzbrain/tests/test_physics.py`。

## 4 写域
dzrobot_v2/tools/route_check.py, dzrobot_v2/src/dzbrain/tests/test_physics.py 与本卡。只写本卡列举文件；不覆盖既有在途文件，不碰 planner/云台/感知/config 默认值，不联网操作车。

## 5 验收命令
根目录：python docs/_工具/gen_index.py --ns traj-1025；python docs/_工具/check_doc_policy.py；python docs/_工具/check_anchors.py；python docs/_工具/check_numbers.py。
dzrobot_v2：python tools/wf/check.py；python tools/wf/check_links.py；python -m pytest src/dzbrain/tests -q；现有 test_pursuit.py / test_corner_cut.py / test_speed_ramp.py 逐脚本验收。无 pytest 不能算通过，另记原生 harness 实测。

## 6 回滚点
根仓 HEAD aa9b37341187f311c707d30288ff433c7851eea8；本轮原件在 `_tmp_session/traj-1025/before/`。只逆转本卡差异，禁止 reset/clean 或覆盖其他团队改动；两个嵌套仓分开核对。

## 7 执行记录
root 已交 route_check.py（含 TrajectoryFieldGate）给 /root/baseline 单写，扩既有CLI，不另建校验器。用户澄清：桥洞开口0.35m，直线进洞；障碍硬验收为body surface >= planner.obstacle_margin_m(.02)，.30中心距为warning。

### 7.1 验证入口与判据

- 新CLI支持重复 `--map`；省略 `--route` 时调用既有 tour planner 与第二停站收口；显式路线支持文本文件/JSON，保留末点 `,x` 朝向。编译失败、输入失败、规划失败均有 JSON 和非零退出码，不把有限搜索失败说成地图无解。
- 误差分别记录原顶点到曲线距离、映射弧长处必经点实际误差、轨迹到原折线偏离；原顶点值及 0.113 m 只作差异记录，遵守用户最新验收，不作为成败阈值。
- 严格雷心半径 0.035 m 与原垫片覆盖分列；射击停站复用配置中的板心窗口，原默认不改。
- 净空复用完整旋转矩形，包含后轴参考点偏置。障碍硬门槛 0.02 m；0.30 m 中心距只告警。桥墙止于格边界，不沿用旧虚拟进洞外延；桥内横偏不超过 0.0875 m，同时要求直线对正。
- AABB 只用于排除远物体，近物体仍调用原 physics 几何；500 个随机姿态与完整几何门禁对拍，加近障/偏置/桥边界负向例。
- 连续区间证书用 `ds/2 * (1 + (hypot(L/2,W/2)+abs(offset))*kmax)` 扣采样净空，并检查解析片段两侧的位置/车身朝向/实际转向曲率连续及 cusp 两侧曲率为零。下界不足标“未证实”，与实际采样碰撞分开。

### 7.2 原树测试与历史红

`python dzrobot_v2/src/dzbrain/tests/test_physics.py --trajectory-only`：exit 0，**19/19**，日志 `_tmp_session/TRAJ-VERIFY-20261025/physics-trajectory.log`。该选项仅定位，不能代替完整套件。

后续按用户澄清补上全车进入桥洞期间的轴向约束、后轴与几何车心区分、可选闭环双臂、CSV/SVG 复用与套件快照合表，定向原树最终 **25/25**（同日志覆盖为最后运行；不等价于完整套件绿）。

完整原文件执行仍 exit 1，旧 `test_cross_check_route_check` 因改前已删的配置地图读取失败；日志 `_tmp_session/TRAJ-VERIFY-20261025/physics-full.log`。未恢复默认地图、未修改旧断言/阈值。所有原函数保留。

### 7.3 多图实际结果与不可隐去的失败

- 初版第七图曾通过，但后续轨迹算法和桥物理模型改变后必须重验；`_tmp_session/TRAJ-VERIFY-20261025/map7-audit.log` 是历史过程，不能代替最终证据。
- 第六图在既有规划器双停站结构自检处失败（只得到 B4 一个停站），尚未进入轨迹编译。证据 `_tmp_session/TRAJ-VERIFY-20261025/map6-audit.log`，exit 2，未改 planner。
- `map_field` 初版编译在第 12→14 个原航点耗尽 18000 节点预算，证据 `_tmp_session/TRAJ-VERIFY-20261025/map-field-audit.log`；后续 geometry 通用修正解除了这个失败，旧记录保留。
- `map_carloop_20261024` 与第七图地图内容布局完全相同，仅名称/注释不同。虽通过，不能当另一布局成功演示；证据 `_tmp_session/TRAJ-VERIFY-20261025/map-carloop-audit.log`。
- 固定 geometry SHA256 `034a5a4f173d0396bf8cb69c7336eeba56c1c49dfdf9e0068eee15eeab94f425` 的真实双布局 2 mm 审计：第七图长度 18.1703555 m / 7 cusp，必经误差约 2e-15 m、严格雷心 8/8、障碍连续净空下界 0.0320509 m，但桥内朝向偏 0.442991 rad、曲率 3.67739，**直线对正失败**。`map_field` 长度 18.1201906 m / 12 cusp，必经误差约 2e-15 m、雷 8/8、桥直线，但障碍净空下界 0.0194891 m，**2 mm 精度未能证明 0.02 m 门槛**，不是已发现实际碰撞。两图解析片段连续性检查均通过。证据 `_tmp_session/TRAJ-VERIFY-20261025/two-layouts-final.log`，exit 1。
- root 已授权同一轨迹统一细化到 1 mm 审计，以缩小证明误差，物理门槛不变；等待桥直线结构修正后再重验。新静态结果也不能覆盖尚未通过的动态 RMS / 完整任务验收。

统一复跑命令（在仓库根；最终实际执行与结果待下节追加）：

```powershell
python dzrobot_v2/tools/route_check.py --trajectory --map _tmp_session/vtest-1025/truth/map7.yaml --map dzrobot_v2/src/dzbrain/tests/fixtures/map_field.yaml --sample-step 0.001 --json --baseline _tmp_session/TRAJ-BASE-20261025/map7-sim-field/baseline.json
```

### 7.4 真实车心与最终独立静态结果

坐标复核：Pose / 积分点为后轴；真实车体几何中心在车头方向前移配置 `vehicle.ref_offset_m`（本场 0.075 m）。只报后轴 8/8 会漏掉 cusp 处车心未压雷。`bf6e48...` 旧候选独立复核第七图车心 5/8、另一布局 6/8，证据 `_tmp_session/TRAJ-VERIFY-20261025/two-layouts-bridge-body-final.log`，未隐藏。

新 geometry SHA256 `0fc8c6346cd3d7820b654e9aefa0b19c8ecc8aadc69fd40142afd27af4cca7ab` 将 retained 雷任务事件定位为车心到雷心，其它必经事件保留后轴契约。两图同配置、同 18000 节点预算、同 1 mm 精度的独立审计 **exit 0**：

| 指标 | 第七图 | 另一布局 map_field |
|---|---:|---:|
| 轨迹长度 / cusp | 18.80360684 m / 11 | 17.50945688 m / 9 |
| 真车心严格覆盖 / 后轴覆盖 | 8/8 / 6/8 | 8/8 / 6/8 |
| 必经事件最大误差 | 1.7e-15 m | 1.7e-15 m |
| 障碍表面采样最小 / 连续下界 | 0.034601 / 0.033677 m | 0.031648 / 0.030725 m |
| 桥内曲率 / 轴向误差 / 横偏 | 0 / 约 6e-17 rad / 约 2e-15 m | 同左 |
| 原顶点最大距离（仅差异） | 0.659771 m | 0.704006 m |

所有解析片段两侧 C0 / body yaw / 实际转向曲率连续、cusp 两侧曲率为零。证据 `_tmp_session/TRAJ-VERIFY-20261025/two-layouts-center-final.log`；两图结构化报告在 `_tmp_session/TRAJ-VERIFY-20261025/center-final-static/`，含最小净空对应姿态和精确车体四角，可复核矩形口径。该结果只证明静态轨迹子验收，不证明运行链、深度相机 RMS 或任务时间达标。

### 7.5 同一 CLI 的闭环与快照对比入口

`--simulate` 复用既有 run_sim，旧 Path / 候选 Trajectory 各一臂；相同 Route / cfg / FieldSpec / SimConfig，均关闭格心锚定，默认共同上限 20000 拍。候选显式独立 TrajectoryControlState。RMS 分别相对各自参考；只有两臂完成才可比较。旧臂物理失败不抹掉它的 RMS 测量，但候选验收单独要求真实车心雷覆盖、精确区域、RMS 改善、override=0、正反向加减速/jerk/最终舵角速率约束。

动态区域基于真实 pose 再过当前门禁并分列精确矩形违规；原 FieldObs 数保留为历史口径，其 board_breach 含额外半宽/半对角线，且参考点偏置不同，不把它直接当新物理碰撞。门禁严格设计轴向拒绝与实际物理区域违规分列。时间预算使用实际 sim time 加每射击停站 `max(30 - explicit dwell, 0)`，避免重复计显式 dwell；未完赛不宣称 180 s 达标。物理相机回放 RMS 始终 null。

可选 `--baseline-suites` / `--candidate-suites` 只读已有原生 runner 的 CSV，显示非零退出套件数、新红/修复红及文件 SHA256，不自动跑新 runner。`--output-dir` 复用原 CSV/逐段 CSV/SVG 保存器，候选 SVG 显示采样新参考；保存的精确矩形证据在每图审计 JSON。已有产物拒绝覆盖。

首轮完整闭环命令已执行，exit 1，原始结果保留在 `_tmp_session/TRAJ-VERIFY-20261025/final-two-map.log`：

```powershell
python dzrobot_v2/tools/route_check.py --trajectory --map _tmp_session/vtest-1025/truth/map7.yaml --map dzrobot_v2/src/dzbrain/tests/fixtures/map_field.yaml --sample-step 0.001 --simulate --sim-max-steps 20000 --json --baseline _tmp_session/TRAJ-BASE-20261025/map7-sim-field/baseline.json --baseline-suites _tmp_session/TRAJ-BASE-20261025/suites/suite_table.csv --output-dir _tmp_session/TRAJ-VERIFY-20261025/final-two-map
```

### 7.6 首轮完整闭环失败证据（不当最终通过）

固定 geometry `0fc8...` / controller `66e33...` / velocity `a8dad...` / sim `4da6...` 的两图闭环完成，运行中源文件未变；全哈希见每图 audit JSON 与 `_tmp_session/TRAJ-VERIFY-20261025/final-two-map/final-summary.json`。该轮 CLI 源哈希为 `eabe3a2fa0ca534ab190dc814be6904c654a70f0124912d0c41063923e42145b`。

| 观测 | 第七图旧 → 新 | 另一布局旧 → 新 |
|---|---:|---:|
| 完赛 | 是 → 是 | 是 → 是 |
| 各自参考横向 RMS | 0.133073 → 0.003771 m | 0.124313 → 0.018451 m |
| 真车心严格雷覆盖 | 3/8 → 8/8 | 3/8 → 8/8 |
| 候选精确区域违规采样 | 0 | 0 |
| 候选真实最小障碍表面净空 | 0.034162 m | 0.029386 m |
| 候选真实桥横偏 | 0.004034 m | 0.008271 m |
| 候选 constraint override | 18 拍 | 4 拍 |
| 候选命令上升 / 下降加速度 | 0.37656 / 0.37496 m/s² | 0.29816 / **3.35775** m/s² |
| 候选 jerk | **6.26492** m/s³ | **166.88768** m/s³ |
| 候选最终命令舵速 | 120°/s | 120°/s |
| 候选执行横向加速度（CSV 复算） | **0.355658** m/s²（28 拍超限） | **0.353443** m/s²（1 拍超限） |
| 实际运动 + 射击预算 | 147.16 + 60 = **207.16 s** | 163.02 + 30 = **193.02 s** |

另一布局是既有单靶夹具，planner 实际输出一个射击停站；若按完整任务一律两个停站，则其预算为 223.02 s，同样超 180 s。不得把单停站数字当双停站达标。横向加速度取已保存执行值 `v²|tan(steer_deg)|/L`，不是指令角猜测；首轮日志尚无该新增字段，但 CSV 独立复算已存汇总。之后 CLI 增补执行横向加速度与 1 mm 剖面 `v(s)²|κ(s)|` 检查，定向 25/25 复验；没有修改原闭环轨迹数据或阈值。

后续发现另一布局轨迹初始切向取首段 +X，而 run_sim 现场设计起姿固定 +Y。root 要求修正真实输入，不能把 sim 改为“对齐路线”来洗绿。geometry 提供显式 start_pose / start_yaw，sim owner 将提取唯一初始朝向解析器供原 run_sim 与 CLI 共用；本卡等待统一源后再最终审计。上述结果是已执行的失败对照，不作新初姿版本的最终验收。

### 7.7 初姿接线与坐标范围等待裁决

共用解析器已落地：CLI 调用 sim 的 `resolve_initial_yaw(route, SimConfig())`，将返回值显式传给 Trajectory；候选初始位置/车身角与该真源不符即红。原测试树增加“首段 +X、设计 N10 起姿仍 +Y”的回归，定向 **27/27**。未改仿真为 `init_yaw=route`。

用户随后提出“首段 +X、车头朝 M10”，root 正在区分局部生成坐标与全仓场地换轴；最终整图重跑暂缓，不把未定坐标契约当最终通过。只读审计结果：

| 位置 | 与轴有关的现行契约 |
|---|---|
| `dzrobot_v2/src/dzbrain/dzbrain/core/geometry.py` | block_to_meters 的 x 由行号、y 由列字母决定；N10→M10 是 world +Y。地图中心边界 x=[0.175,3.675]、y=[0.175,4.725]；物理区域再扩半格。 |
| `dzrobot_v2/src/dzbrain/dzbrain/route/physics.py` 的 barrier_centerline / body_barrier_clearance_m | 挡板固定 x=1.75，延伸沿 y，矩形厚度给 x 半边；直接喂旋转坐标不等价。 |
| 同文件 bridge_entry_walls | K 列中心 y=1.225，壁位于 x=cx±0.175；按 row≥7 选 y 方向入口。 |
| 同文件 segment_outside_band | 仅裁剪 y 轴带状区域；旧注释存在桥洞豁免话术，当前轨迹门禁没有使用该豁免。 |
| 同文件 body_center / body_corners / body_obstacle_clearance_m / polygon_distance_m | 旋转车体与圆障的几何公式可随一致刚体变换工作；须同步变换场地要素与 yaw，不是只换车辆坐标。 |
| `dzrobot_v2/tools/route_check.py` 的 TrajectoryFieldGate | 按 world +Y 约束桥轴、用 abs(x-cx) 算横偏，挡板/K 矩形/边界继承上述 world 契约。 |

若只需局部首向 +X，正确方向保持的变换可为 `x_local=y_world-y0`、`y_local=-(x_world-x0)`、`yaw_local=yaw_world-π/2`；物理门禁将姿态反变换回 world 即可复用真源。单纯交换 x/y 是反射（行列式 -1），会翻曲率和左右转符号，不能当等价旋转。本卡未改任何 physics/geometry 模块或现场坐标契约。

## 8 结论
进行中，不宣称整体通过。CLI 明确标为 STATIC GEOMETRY 子验收，不包括实际跟踪 RMS 与完整套件；只有逐区间连续运动上界和解析接点检查都满足时才给连续净空证书。

## 9 经验总结
现象：旧sim可完成但实际物理观察有违规。处置：复用physics全车footprint，任务覆盖和净空分列。

现象：名义桥前中后三锚点直线仍可能被其它连接段提前侵入桥洞；后轴压雷也可能让实际车心偏 7.5 cm。处置：桥约束施加到任何连接段的全车矩形入/出洞全过程；雷事件明确车心坐标框架，原路线顶点偏离与任务覆盖分别统计。独立审计没有调低阈值来迎合候选。

## 10 文档变更
新增本卡；修改本ns任务树；归档无。
