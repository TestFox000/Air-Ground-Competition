---
来源: 用户2026-10-25全仓坐标统一指令
更新: 2026-10-25
status: in_progress
ns: traj-1025
agent: /root/trajectory
owner: /root/trajectory
reviewer: /root
树节点: TR-7
写域: _tmp_session/traj-1025/frame-migration/dzrobot_v2/ 下 隔离副本的 core/geometry.py、route/planner.py、route/parser.py、route/model.py、pursuit/trajectory.py；tests/test_core.py、test_route.py、test_pursuit.py、test_corner_cut.py、test_scout.py、test_dual_shoot.py、test_dynamic_gear.py、test_geometry.py、test_goal_stop.py；scout/**（仅坐标迁移与本卡回归）
---

# TRAJ-FRAME-CORE-20261025 · 坐标真源与规划/轨迹迁移

## 1 台账与目标
已读D-004/D-023/D-027/D-041。用户明确翻案D-004的坐标命名：+X=N10→M10（西），+Y=N10→N11（南），N9为北即−Y；起步yaw=0°、正舵左转不变。新证据为用户本轮明确指定；影响面为坐标转换/初态/物理区域/消费者；不迁移会使相同摆位出现90°错相。其余比赛与规划规则不重开。

## 2 复用与新增理由
复用唯一core.geometry和现有physics/测试树，不另建坐标真源。正交变换为 x_new=y_old，y_new=11×0.35−x_old，yaw_new=yaw_old−π/2，det=+1；不可只交换xy。隔离副本源哈希在frame-migration/source-manifest.json。

## 3 写域与保护
隔离副本的 core/geometry.py、route/planner.py、route/parser.py、route/model.py、pursuit/trajectory.py；tests/test_core.py、test_route.py、test_pursuit.py、test_corner_cut.py、test_scout.py、test_dual_shoot.py、test_dynamic_gear.py、test_geometry.py、test_goal_stop.py；scout/**（仅坐标迁移与本卡回归）。所有源码修改仅在隔离副本；原共享工作区未交还的运行链不写、不部署。与其他成员按文件分工，禁止整份覆盖他人新改。

## 4 验收
现有对应测试脚本、154格坐标往返、N10→M10 yaw0、N10→N9 yaw−π/2、左转符号、两图物理审计及原生全量回归；物理阈值不改。离线补丁审查后才处理共享工作区接入。

## 5 回滚点
隔离副本改前source-manifest.json及原共享源码；禁止reset/clean。原运行链无本卡改动。

## 6 执行与结论
进行中；原坐标的旧验收不能代替新坐标结果。

## 9 经验总结
现象：轴名和物理格名方向不一致使起步差90°。处置：按用户格名方向重建右手系，并让所有消费者同时迁移；不改一个yaw常量假装完成。

## 10 文档变更
新增本卡、更新本ns树；结论提炼待验证。归档无。

### 2026-10-25 坐标核心与侦察帧位执行补记

- 唯一真源 `core.geometry` 增加 `MAP_FRAME_ID=map-west-x-south-y-v1`、场地两轴边界、显式历史坐标/航向转换、格方向与 yaw 双向转换；起步 N10=(0.175,3.325)、车头 M10 即 yaw0。154格往返和 det=+1 均通过。正舵左转未翻转。
- `route.parser` 支持末token有符号 x/y/-x/-y；中间token朝向仍拒绝。planner仅旋转实际几何/挡板接口，不改格序搜索。旧末朝向折叠缺陷（旧±X折为+X、旧±Y折为+Y）保留：两图末行驶切线朝南，新任务末朝向仍朝北。`_barrier_line_clear` 历史按列轴的误命名启发式也仅刚体旋转，未顺手修算法。
- `planner-frame-equivalence.json`：map7同格序/标记22.75m，map_field同格序/标记19.95m；旧末token N5,x 显式迁为新 N5,-y，物理任务朝向保持。解析几何 primitive（quintic/clothoid）的长度/位姿/有符号曲率/曲率导数刚体等价。搜索量化格因3.85m平移改变相位，离散选路允许不同，必须重新审计64图，不能复用旧绿。
- Lead追加的侦察写域仅修改副本 `scout/frames.py`、`scout/fov.py` 与 `tests/test_scout.py`、`tests/test_dual_shoot.py`。扫描线从 SCAN_LINE_YS 改为 SCAN_LINE_XS；扫描行位置转为 LOWER_SCAN_YS/UPPER_SCAN_YS。手绘格心偏移点从旧(+半格,+半格)转为新(+半格,-半格)，保持同一物理西北角。FOV通用算法无改动，只改轴说明与既有测试预期。
- `scout-before-frame.json` 为改前生产源码生成的只读oracle；`scout-frame-equivalence.json` 记录68帧逐帧刚体误差最大6.7321e-7（旧yaw六位小数舍入），全部逐帧覆盖格相同；154/154、33.5s、最小足迹重叠0.5均保持。`scout-dual-frame-diagnostic.json` 记录手绘路线84帧/146格和35帧/126格不变；dual legacy/tasks 的历史SHA保持原值，校验前显式把新有符号末朝向转换回历史frame，没有改历史摘要。

副本目录下已跑命令与结果：

| 命令 | 结果 | 证据（frame-migration/ 下） |
|---|---|---|
| `python -X utf8 src/dzbrain/tests/test_core.py` | 62/62，exit0 | test-core-frame.log |
| `python -X utf8 src/dzbrain/tests/test_pursuit.py` | 167/167，exit0 | test-pursuit-frame.log |
| `python -X utf8 src/dzbrain/tests/test_route.py` | 164/165，exit1；既有默认路线44/68 token断言失效 | test-route-frame.log |
| `python -X utf8 src/dzbrain/tests/test_corner_cut.py --geometry-only` | 72/72，exit0；未代替全闭环验收 | test-corner-geometry-frame.log |
| `python -X utf8 src/dzbrain/tests/test_scout.py` | 26个坐标/覆盖断言PASS后缺失config/map.yaml退出1；未恢复或替换默认输入 | test-scout-frame.log |

首段只读审计：`first-leg-readonly-audit.json` 的内部合成反例为非R路线 (1,1)→(0.5,1)、末yaw0、start_yaw0。现生成器产生 bodyyaw0 但 reverse=True、travel yawπ、首1cm Δx=-0.01。因此 start_yaw 只约束车头，尚不保证任意输入第一运动段为+X；该无field反例不代表64实图任一失败。按Lead要求记录后冻结源码，不在64批次中途改方向策略。

冻结哈希：副本 core.geometry=e8e941a004c4d1ebbd42880e190340e7f7310fab386713ce2f0896c09752d860，planner=d0bc2011d63672fc737d2b7f767826bb33f57303e995bf5e88fd4d3c10a783bb，parser=1f369173bfd981258950707ab294564d86317dcfffdf624423978bde5f097c10，trajectory=596e2e07a7d45e35c975cdb6b8ddcf9304d466761d5299093dfb20863aaeda3e。生产trajectory同hash未改；时间cost实验未混入。此卡未收口：等待64图/统一原生回归及Lead审查，无提交、无部署。

### 原生回归新增红：三套坐标夹具迁移

Lead 在全量41套/27红（基线41套/18红）后分配本卡仅修改副本 `test_dynamic_gear.py`、`test_geometry.py`、`test_goal_stop.py`，新增写域已同步卡头。原因均为测试仍沿旧轴投喂/期待，并非压力版本控制器或geometry遗漏：

- dynamic_gear：同一条N10→M10→K10的朝向由旧+y转新+x；远点从旧y−0.70转新x−0.70。两处历史数值位姿连同body yaw使用显式legacy转换；旧隐式末北向显式写新−y。0.15/0.35/0.40m/s断言不变。
- geometry：N11/A1/N10锚点、单位坐标、边界、Pose物理偏移与K列单调方向迁新frame；154格双向、非法输入、四元数与原1e-9/1e-12阈值保持。
- goal_stop：任务reset从旧N10,y转新N10,x；末旧默认北向显式L10,-y；沿程未到距离从旧y负偏移转新x负偏移。Localization历史body yaw0显式转新−π/2，不能顺手把原先与路线垂直的车头改成沿路姿态。末点0.03m/中间点0.12m、0.11不达与0.02到达、锁存事件数与刹车数全部原样。

在隔离副本逐个跑 `python -X utf8 src/dzbrain/tests/<name>.py`：dynamic_gear由24/26到26/26，geometry由28/38到38/38，goal_stop由13/26到26/26，三条命令均exit0。日志位于 `frame-migration/test_dynamic_gear-fixture-migrated.log`、`test_geometry-fixture-migrated.log`、`test_goal_stop-fixture-migrated.log`。旧native-after全量日志保留，不覆盖历史结果。无源码算法修改、无阈值放宽、无后台进程。
