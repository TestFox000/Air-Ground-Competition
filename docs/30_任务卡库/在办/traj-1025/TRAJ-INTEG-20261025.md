---
来源: 用户 2026-10-25 循迹替换交接说明
取证: 实/算/未
更新: 2026-10-25
取代: 无
status: in_progress
ns: traj-1025
agent: lead
owner: lead
reviewer: /root/baseline
树节点: TR-4
写域: dzrobot_v2/src/dzbrain/dzbrain/pursuit/controller.py, dzrobot_v2/src/dzbrain/dzbrain/pursuit/lookahead.py, dzrobot_v2/src/dzbrain/dzbrain/core/config.py, dzrobot_v2/src/dzbrain/dzbrain/tasks/tracker.py, dzrobot_v2/src/dzbrain/tests/test_pursuit.py, dzrobot_v2/tools/route_check.py, dzrobot_v2/src/dzbrain/dzbrain/tools/sim.py, 本 ns 卡树及单写治理登记
---

# TRAJ-INTEG-20261025 · 前馈反馈接线与现有验收对比

## 1 目标与台账
前馈反馈接线与现有验收对比。已读 D-008/D-012/D-023/D-027/D-028/D-034/D-041：冻结登记、保留 planner、旧红分类、阿克曼前提、卡树隔离、双远端分叉不得擅自强推。用户本轮授权替换循迹，但不授权部署。离线候选通过前不切现役默认。

## 2 复用检索
已查 doc_search「连续轨迹 阿克曼 循迹」、复用清单、能力索引、目录索引；源码搜索 trajectory/clothoid/dubins/quintic/jerk 无现成实现。复用 Path/Route/geometry/physics/route_check/现有 sim 与现有 tests 树。仿真不能证明深度相机实测 RMS。

## 3 新增轮子及理由
无重复仿真器；扩展现有 route_check 与控制入口。

## 4 写域
dzrobot_v2/src/dzbrain/dzbrain/pursuit/controller.py, dzrobot_v2/src/dzbrain/dzbrain/pursuit/lookahead.py, dzrobot_v2/src/dzbrain/dzbrain/core/config.py, dzrobot_v2/src/dzbrain/dzbrain/tasks/tracker.py, dzrobot_v2/src/dzbrain/tests/test_pursuit.py, dzrobot_v2/tools/route_check.py, dzrobot_v2/src/dzbrain/dzbrain/tools/sim.py, 本 ns 卡树及单写治理登记。只写本卡列举文件；不覆盖既有在途文件，不碰 planner/云台/感知/config 默认值，不联网操作车。

## 5 验收命令
根目录：python docs/_工具/gen_index.py --ns traj-1025；python docs/_工具/check_doc_policy.py；python docs/_工具/check_anchors.py；python docs/_工具/check_numbers.py。
dzrobot_v2：python tools/wf/check.py；python tools/wf/check_links.py；python -m pytest src/dzbrain/tests -q；现有 test_pursuit.py / test_corner_cut.py 逐脚本验收。加速度斜坡测试在 test_pursuit.py 内，无独立 speed_ramp 测试文件。无 pytest 不能算通过，另记原生 harness 实测。

## 6 回滚点
根仓 HEAD aa9b37341187f311c707d30288ff433c7851eea8；本轮原件在 `_tmp_session/traj-1025/before/`。只逆转本卡差异，禁止 reset/clean 或覆盖其他团队改动；两个嵌套仓分开核对。

## 7 执行记录
2026-10-25 已读取规则并登记；基线与实现进行中。检查和独立复核未完成，不提交、不归档。

## 8 结论
未验收。外部深度相机真值、全部布局可行性与 180 秒实车预算未证实。

## 9 经验总结
现象：根仓 status 不能覆盖 dzrobot_v2 嵌套仓；后者有既有改动及暂存项。处置：分别读取 git status/diff，保存改前副本，只归属本卡差异。

## 10 文档变更
新增：本卡与 traj-1025 任务树。修改：ns登记、复用/能力登记、冻结登记、D-042、团队状态中的本轮段落、车端02循迹与控制、06回归与测试基线及册索引。归档：无；验收失败保持在办。

### 写域移交与用户澄清
2026-10-25：test_pursuit.py 暂交 /root/speed 单写，交回后 root 接控制测试；route_check.py 从当前快照交 /root/baseline 做审计CLI。桥洞实际开口0.35m（正摆单边0.1075m），轨迹需直线进洞。障碍硬判据=车体表面到柱表面至少planner.obstacle_margin_m(0.02m)；0.30m中心距只是警告线。

### 2026-10-25 控制实现移交
controller.py / lookahead.py / tools/sim.py / test_pursuit.py 的本轮实现唯一写者改为 /root/speed；root 只读评审与配置Schema登记，不并发编辑。旧 Path 路径默认保持，候选显式 Trajectory 才启用。Tracker/Localization/Mission 共享弧长与吸附迁移须待几何全图验收，不允许只替换一端。

### 工具踩坑（实）
现象：试读 gen_index.py --help 时脚本没有 help 分支，实际执行了全量索引生成（208篇）。机制：parse_scope忽略未知参数，非--ns/--only都回落全量。处置：此后先读脚本参数解析，固定使用 --ns traj-1025；共享索引本轮不整份提交、不覆盖他人文档。该坑不是门禁通过证据。

### 方法依据与证据边界
Stanley反馈参考原论文 https://ai.stanford.edu/~gabeh/papers/thrun.stanley05.pdf ；二阶时间参数化与三阶jerk约束不是同一最优问题，原研究 https://arxiv.org/abs/1609.05307 。当前 velocity 输出二阶可达扫描及有界jerk可行剖面，不能声称严格三阶全局时间最优。root 独立有限差分与积分验证见 _tmp_session/traj-1025/velocity-independent.json（两个剖面，积分误差<1e-12，jerk界实测成立）。

### 用户验收修正（2026-10-25）
0.113m取消为验收线，仅旧几何论证。验收三条：必经点误差、区域硬约束（精确矩形表面净空≥0.02m；桥横偏≤0.0875m且格边墙不碰）、实际轨迹相对新参考的横向RMS。旧顶点偏离只作diff。已同步geometry/control/verify三个owner。
正式知识提炼已写入车端/02、车端/06并加来源卡号；册索引同步，尚未归档。

### 外测与环境补充（2026-10-25）
用户明确回复「暂无，先保留外测验收未通过」：无额外深度相机整场回放和标定，外测RMS验收保持未通过，不用仿真冒充。为真正执行指定pytest命令，已在获准联网后将pytest 9.1.1及依赖只装入本任务临时目录 `_tmp_session/traj-1025/python-deps/`，未改系统Python或项目依赖。原始缺包退出1仍作为开场环境基线保留；装包后的执行结果单列。

### 本轮范围确认与审查记录
用户明确回复「尚未交还，先完成离线候选」：AGC-T1仍占tracker/localization/mission写域，故本轮只完成三层候选及离线验收，运行链迁移、PIVOT/ALIGN删除留待写权交还。已撤销本轮预登记但未使用的tracker冻结项，原owner登记保留；不得把保留旧FSM说成已删除。

指定pytest命令装包后实跑退出3：test_autoplan在导入期运行原harness并调用finish→SystemExit，pytest显示no tests ran；日志 `_tmp_session/traj-1025/pytest-installed.log`。其中规划的2秒预算也在pytest导入重写/并发负载下超时，不能把pytest收集失败换名为原生套件通过或新增算法回归。收口仍需逐脚本原生全量对照。

独立复核先后否决了三项中间“通过”口径：①格边实墙替换虚拟墙后允许斜入桥，需另保留全车进洞直线约束；②长距离曲率前馈与当前Frenet反馈不在同一相位，零误差并非闭环解；③积分点是后轴而不是几何车心，offset=+0.075m，旧8/8后轴覆盖不能代替真实车心覆盖（中间版本仅5/8）。均要求修改通用模型/约束，并保留旧hash证据，未降低物理门限。

速度核独立有限差分/积分复核见 `_tmp_session/traj-1025/velocity-independent-final.json`（文件含源hash）。该证据只对应其中记录的速度核版本；后续按停点局部参数化改进须再跑原测试树，不以旧探针覆盖新版本。

### 64张新随机图压力测试（2026-10-25，完成试验、验收失败）

用户明确新增要求“按照地图规则随机做64个地图然后压力测试”；这是本次离线64图扩量的直接授权，旧D-005/D-011的扩量限制不阻断本次请求；不改变tour引擎、不授权部署。

输入复用active stressbench.generateMap，seed固定20261025..20261088，64张布局唯一、四类特征都随机，未按规划/轨迹是否成功筛选。manifest SHA256 `bad675d2f3fbecef94cb0b96f0c97d8755697bb6d18d22090db67ed0f6ca0bff`。满足目前可核实的静态落格规则，不声称所有图都有物理解；5张（零基编号17/29/46/55/59）M10有障碍，固定N10/yaw0车体初始表面净空仅0.0075m，已单列且保留在64分母。查阅原场地说明书与规则未找到M10邻格禁放条款，因此未事后加规则筛掉它们。

固定参数：同一Config、同一sim模型，两臂关闭格心吸附；曲线采样步长0.001m、连接搜索预算18000、仿真上限20000拍。每图audit/CSV/SVG原样保留；CLI退出2（包含规划失败），没有内部异常丢图。

| 阶段 | 结果（总分母64） |
|---|---|
| 规划 | 15成功 / 49失败（含结构校验及时间预算失败，不一概叫物理无解） |
| 连续轨迹编译 | 7成功 / 8失败 / 49未进入 |
| 静态约束与候选闭环完成 | 7 / 64 |
| 全部已实现验收项通过 | **0 / 64** |

7张候选完赛图全部真实车心8/8覆盖、采样区域零违规，RMS为2.41–4.17mm；其中仅4张旧臂也完赛，4张RMS均改善，另外3张不可比。7张含2×30秒射击的总用时为206.42–265.46秒，全部超过180秒；5张jerk超限、2张减速度超限、1张剖面网格横向加速度超限、6张存在约束override。实际执行横向加速度及最终转向/速率这7张均在限内。未进入候选仿真的57张对应指标是未测，不是57张动力学违规。

桥洞实际最大轴向角误差0.44–1.89度、最大车身角速度0.23–1.72度/秒，仅报告观测值，不自造动态容差。7张初始车头均0度、首个CSV可分辨位移均向+X，但接口层仍存在允许首段倒车的通用反例，不能以7张通过代替接口保证。深度相机外测、真实mission接线、PIVOT/ALIGN删除仍未完成。

产物：`_tmp_session/traj-1025/frame-migration/random64/manifest.json`、`_tmp_session/traj-1025/frame-migration/random64-results/summary.json`、`_tmp_session/traj-1025/frame-migration/random64-results/summary.csv`、`_tmp_session/traj-1025/frame-migration/random64-results/batch.log`及64份独立audit。前后源锁无漂移。初始PowerShell锁脚本误写tools/sim.py，告警保留；完整核对使用批前`_tmp_session/traj-1025/frame-migration/physics-frame-ready-source-hashes.json`中的真实`src/dzbrain/dzbrain/tools/sim.py`，没有补造批前证据。

同图同参复跑（v1根PowerShell；新输出目录必须尚不存在）：

```powershell
$trajArgs = @('-B','-u','-X','utf8','_tmp_session/traj-1025/frame-migration/dzrobot_v2/tools/route_check.py','--trajectory','--sample-step','0.001','--max-nodes','18000','--simulate','--sim-max-steps','20000','--output-dir','_tmp_session/traj-1025/frame-migration/random64-rerun','--json')
Get-Content '_tmp_session/traj-1025/frame-migration/random64/map-paths.txt' | ForEach-Object { $trajArgs += @('--map',$_) }
python @trajArgs
```

下一实验依据：random18的最大jerk出现在连续倒车中段，不是起步/停车/cusp；硬cap裁剪是直接机制，离散速度包络是否漏掉解析限速低谷尚需补观测证明。禁止调松约束掩盖、禁止把有限搜索失败当地图无解、禁止把这批0/64写成候选完成。全量测试第一轮41/27，新增坐标兼容红正在逐项复核，原日志保留。

### 64图之后的兼容复验与提交状态

第二轮原生全量为41/18，与开场非零套件集合完全一致，无新增红套件；所有原始日志保留在 `_tmp_session/traj-1025/frame-migration/native-final/`，比较表见同目录comparison.json。第一轮9个新红套件来自坐标旧夹具、mission足迹诊断漏迁挡板API及隔离副本缺C++只读取证源，按原物理断言修正；未改控制核或64图结果。旧红套件里的route/drift坐标额外失败随后定向消除，恢复164/165（原44/68配置期望）和23/24（原1.2b）。test_deploy补齐原只读依赖后93/94，保留缺map原失败。

test_corner_cut完整为89/92，5.5/5.8/5.9（障碍净空/动力学与约束冲突/180秒）仍红，不能宣布替换完成。固定第七图记录67个障碍表面净空违规采样，含射击总时198.4秒；这与随机批中7张完成图的采样区域通过是不同输入的结果，不得混用。pytest原命令再次实跑exit3，harness导入SystemExit、no tests ran；日志 `_tmp_session/traj-1025/frame-migration/pytest-after.log`。门禁本轮doc-policy/numbers/links退出0；workflow退出1的3个未登记项仍为原map/map_v3删除和localization改动，不代填其他owner。锚点守卫仍有12条任务外新增红，本任务裸证据路径已补全。

分支agents/dzhong/dzrobot-v2-20261018；未建commit、未推GitHub/Gitee。理由是验收未过、冻结/锚点门禁未全绿，且运行链写权未交还。卡保持in_progress，不归档，不部署。

### 下一 agent 交接快照（2026-10-25）

本节是最新接续入口；前文按轮次保存的旧hash、两图绿色结果和等待状态属于过程，不能覆盖64图新帧证据。三层离线候选已实现，64图试验完成，但替换目标及验收未完成。

**工作位置与保护**

- 沿用traj-1025已有卡树。先读本卡、TR-7/8/9/10、车端02/06与D-042。
- 后续源码工作位置为 _tmp_session/traj-1025/frame-migration/dzrobot_v2/；无人机草案在同级dzsmartscout。共享dzrobot_v2也有早期候选，其坐标上下文不同，不能只复制三个文件覆盖集成。
- 当前嵌套仓分支agents/dzhong/dzrobot-v2-20261018、HEAD 945bbc14605c18cd24c526ae5813ecd03c05d8aa。这是接续观察值，不是本任务新提交。两层仓库均有他人改动/暂存项，不可add全部、reset、clean、恢复他人已删地图。
- 用户确认t-car尚未交还tracker/localization/mission。原运行链仍无写权；PIVOT/ALIGN未删除；无部署、无本任务commit/双推。输入、audit与原始日志保留，重跑使用新输出目录并记录新hash。

**已有实现**

| 模块 | 能力与边界 |
|---|---|
| pursuit/trajectory.py | clothoid/quintic连续曲率连接、κ界、普通点可偏离、真实车心压雷、全车进出桥直线、cusp、停点与s_max投影；sample.yaw为行进切向，body_yaw为车头，x/y为后轴 |
| pursuit/controller.py、lookahead.py | 延迟一致曲率前馈、Frenet反馈、时间进度/停车握手、倒车系数、最终舵角幅度/速率/横向加速度可行域；普通Path保留旧分支 |
| pursuit/velocity.py | 两遍扫描、停点分区、可行jerk剖面、MotionState、SpeedEnvelope未来cap过滤；不是严格三阶全局时间最优，闭环仍有hardcap冲突 |
| tools/route_check.py、src/dzbrain/dzbrain/tools/sim.py | 复用原模型/延迟队列，静态动态同源审计、两臂比较、真实车心/矩形净空、CSV/SVG/hash；不替代相机实测 |
| core.geometry及消费者 | 新帧+X西/+Y南，N10=(.175,3.325)、yaw0，154格往返/侦察覆盖等价；跨端契约尚未闭合 |

不可回退：旧到新为(x,y,yaw)→(y,3.85−x,wrap(yaw−π/2))；正舵左转、度制、最终rate限幅、首拍无历史不限rate。真实车心在后轴前0.075m，压雷半径0.035m；矩形至柱表面至少0.02m；桥开口0.35m、横偏最多0.0875m且服从格边墙。0.113m只作旧折线论证。motion_limit最终层能压到0，缺信号不额外干预。历史fixture显式转帧，不放宽阈值。

**问题与下一步方案（尚未实施）**

| 问题 | 已有证据 | 建议方案 |
|---|---|---|
| 首段+X接口缺口 | first-leg-readonly-audit.json：非R路线(1,1)→(.5,1)，yaw0仍倒车起步；不是64图实际反例 | 首次_connect增加initial_direction=+1，覆盖direct/搜索根/首扩展；之后保留cusp；首段显式R冲突拒绝。若整段到M10都须直行，另编码直线约束，不能只钉初始切向 |
| 整图测试门禁不完整 | test_corner_cut只检查compare_simulation部分动态条件；CLI已合取静态与动态 | 在现有test_corner_cut复用check_trajectory完整结果，补静态合取与横向加速度项，不另建测试体系、不删旧断言 |
| jerk/减速与hardcap冲突 | random18在55.96s持续倒车中段jerk49.51749>1、减速度.990585>.7；不在停车/cusp | 第一轮只加观测：v_cmd、解析cap、cell最小cap、v/a、acceleration_ceiling、future_feasible、override、投影/停点、队列实际速度。区分采样包络遗漏解析低谷和延迟行程低估；证实后统一保守区间cap，不移除hardcap/放宽jerk |
| 第七图障碍净空 | 新帧固定图67个违规采样、含射击198.4s | 对照同一s的参考与实际车体净空，分清几何问题或跟踪误差吃掉余量；后者按统一误差/延迟界预留轨迹裕量，保留20mm硬线 |
| 49规划失败 | 29结构自检、10分段搜索、6射击绕路代价全无穷、2无共同射击停站、2超原2秒预算 | 先复现01，检查plan_once/tour/第二停站/结构自检及crossing_entries顺序，再按族定位并交planner owner；不重写引擎、不放宽自检、不用控制特例补规划 |
| 8轨迹编译失败 | 4连接搜索耗尽、4端点无候选切向；有限搜索不完备 | 区分输入/起姿违规/候选全拒/桥连接限制/节点预算/队列耗尽/曲率认证未决，记录锚点/拒绝计数/搜索框，再修通用连接；失败不等于物理无解 |
| 全部超180秒 | 7完成图206.42–265.46s | 分解直行/倒车/制动/停车/射击；处理局部参数化保守性与连续连接倒车代价，保留tour任务顺序。若做严格jerk时间最优，作为独立求解层修复验证，不能把当前构造改名最优 |
| 跨端坐标与运行链 | perception/grid.py、car_pose.py、route_crop.py、supervisor未闭合；C++视觉是第三套偏置帧；vendor同步未执行 | 按TR-9具名帧/边界转换审计，验证格锚点、四轴后缀、帧标识；不盲旋相机内参/机体系。待写权交还再接tracker/mission并删特例 |
| 回归/外测缺口 | pytest exit3；41/18仅说明红套件集合；无外部标定数据 | 逐套断言对照及下述补漏；pytest另做harness兼容。无相机数据继续标外测未通过，不用仿真RMS替代 |

规划/编译分类来自64份已有audit，只读复核；5张起姿障碍冲突仍在64分母，不重抽。桥内偏角/角速度只报告观测，未规定动态容差，不能宣称零摆动。

**当前测试证据边界**

- 开场 _tmp_session/TRAJ-BASE-20261025/suites/suite_table.csv 为41/18；完整候选 _tmp_session/traj-1025/frame-migration/native-final/suite_table.csv 也为41/18，红套件集合相同。runner自身exit0不代表全绿。
- 随后route164/165、drift23/24、deploy93/94，新增坐标/依赖问题已定向消除，只留原红。
- test_sim_guard的1.8/5.8仍钉旧N10，应迁移复验；calib_backfill头文件依赖已补，缺随后复跑证据，行号漂移待核对。因此不能宣称“无新增断言回归”。
- test_pursuit167/167；test_corner_cut89/92。后者原基线缺map未执行完整断言，不能把当前3红叫原有旧红；新帧第七图也不能借旧帧两图净空通过。
- 64图以random64-results/summary.json/audit为准：49规划失败、8编译失败、7完成、0全项通过；7张中5 jerk、2减速、1网格横向加速度、6 override违规。其余57张动态项为未测。

**接手操作顺序**

1. 核对副本/hash/写域，沿用已有卡并更新owner，保留64基线。
2. 首段契约与整图门禁分别一轮修复，不与控制改动混成一个实验。
3. 只加观测跑random18尖峰复现，定位后每轮只改一个机制；另诊断第七图净空及各失败族代表图。
4. 短复现和现有test_pursuit/test_corner_cut改善后，用相同64输入/同参数串行复验；不并发重CPU任务污染2秒预算。
5. 逐套回归、外测、运行链接入、特例删除分别验收，达到原任务条件且治理门禁通过后才提交双推。

在隔离dzrobot_v2内执行（输出目录须尚不存在）：

~~~powershell
python -B -X utf8 src/dzbrain/tests/test_pursuit.py
python -B -X utf8 src/dzbrain/tests/test_corner_cut.py
python -B -X utf8 tools/route_check.py --trajectory --map ../random64/random-01-seed-20261026.yaml --config config/robot.yaml --sample-step 0.001 --max-nodes 18000 --output-dir ../handoff-static-01 --json
python -B -X utf8 tools/route_check.py --trajectory --map ../random64/random-18-seed-20261043.yaml --config config/robot.yaml --sample-step 0.001 --max-nodes 18000 --simulate --sim-max-steps 20000 --output-dir ../handoff-dynamic-18 --json
~~~

64图复跑见上节。全量复用_run_full_regression.py，先在进程内将OUT重定向至新目录；检查suite_table逐套exit。最后一次治理为doc-policy/numbers/links通过、workflow三项他人未登记改动、anchors十二条任务外新增红；接手时重新核对，不代填owner、不改基线隐藏问题。

本次交接仅只读复核并追加本卡，没有再次修改算法、重跑长压测或改变完成判定。

---

## P0 派发记录（2026-10-25 · Lead 接卡）

- 接卡：owner/agent 由 /root 变更为 lead；沿用本卡、TR-4 树节点与 64 图基线；写域不变（P0 零代码改动，产出只落 `_tmp_session/traj-1025/p0/`）。
- 用户五项裁决（2026-10-25）：①目标=赛前上车、循迹更通用（P2~P5 全速推进）；②49 张规划失败先攒着——P0 只做分族清单，不开卡移交规划 owner；③t-car 的 tracker/localization/mission 写权到 P6 用时再查；④第七图优先级提前至 P0；⑤深度相机外测数据已提供：`D:\User\Desktop\无人机视频\链轮测试\1.mp4`（45,618,870 B，2026-10-25 18:50，单文件）——能否充当外测真源在 P5 验货，验货前外测仍标未通过。
- P0（只读侦察）已派发子智能体：③第七图 67 违规「参考轨迹 vs 实际车体」对照诊断（A 几何/B 跟踪/C 政策分类）〔未实现〕；①老帧共享仓同批 64 图对照定 49 张罪责（含新旧 route_check 检查集差异与 planner-frame-equivalence.json 复核）；②49 张分族普查 + random-01 钻取（自留账不移交）；④2 张规划超时串行复跑。产出 `p0/P0_REPORT.md`。

## P0 执行结果（2026-10-25 20:40 · 子智能体三路失败后 Lead 自办）

- **派发失效记录**：fbaac6bf / b790f392 / 389bc701 三路均 failed-before-finished；改 Lead 直接执行，仍守 P0 只读、产出只落 `_tmp_session/traj-1025/p0/`。汇总见 `p0/P0-SUMMARY.md`。
- **① 老帧对照**（`p0/oldframe64/REPORT.md`）：老帧共享仓（分支 `agents/dzhong/dzrobot-v2-20261018`，HEAD `945bbc14`）跑同一 64 图 → 规划失败 **49 张，与新区集合完全相同**（交集 49，双方独有 0）→ **49 张非新帧回归**；新规划器把 13 张超时收敛为 2 张（更快的同判决）；编译失败 8 对 8（7/8 同图）。
- **② 失败族**（`p0/families/REPORT.md`）：结构自检 29 + 连通无解 10 + 射击停站不可达 6 + 靶覆盖无停站 2 + 超时 2 = 49；编译 8 = 连接预算耗尽 4（7/16/41/53）+ 端点无候选切向 4（10/28/51/55）；完成 7 = 3/8/18/22/24/61/63，**0 全项通过**（时间 7/7 超、jerk 5/7、override **6/7**、减速 2/7、网格横向 1/7；4 张可比 RMS 全部改善，3 张基线未完赛不可比）。override 口径经 A 复核更正：`constraint_override_status` 实为 violated 6 / within 1（仅 idx61 通过），原叙述 5/7 少算 1 张。
- **③ 第七图**（`p0/section7/REPORT.md`）：静态 **PASS 0 违规**（obstacle 2.29cm vs 硬线 2cm）；物理回放 **67 违规全部在 G10**（t=7.12~8.44 s，最小 1.16cm，深 0.84cm，未触本体）；旧折线臂回放同样 G10 32 + I6 9，且 boundary/barrier/bridge 大违规 → G10 是地图级窄点；**根因=连接器只留 2.9mm 余量，非几何无解**。
- **④ 超时复跑**：random-06/19 串行 **1.16/0.70 s，不再超时**，但结构自检失败（穿洞次数）→ **超时族清零并入结构**；失败确定、模式随预算抖动。C 路反证（把 `planner.max_seconds` 2.0→30.0，副本在 `C/c2/robot_30s.yaml`）：两图仍同理由结构失败、elapsed 0.796/0.803 s ⇒ **结构失败与预算无关，超时只是同一条不合格路线恰在 2.00 s 线附近返回**，该结论已从断言升级为可证事实。
- **责任表**：49 张 → 规划器/地图（不归本层；用户已定"先攒着"）；8 编译 + 7 动态超限 + G10 余量 → 本层 P1~P4 输入。
- **外测视频验货（P5 输入）**：`D:\User\Desktop\无人机视频\链轮测试\1.mp4` 头解析 = 45,618,870 B / isom / 229.167 s / 640×480 / 30 fps / mp4v；C 路已用 cv2 完成定量验货（`C/c3/`），**结论：不能当外测真源**——仅 81~203 s（122 s，53%）是场地+白线可用段；10~70 s 画面近乎冻结（帧差 0.81、位移 0.01 px/s）；尺度 34.5 px/格 ⇒ 1 px≈1.01 cm 但无标定；140 s 后整幅换视角（37~42% 像素差）；场地窗内实心运动块 0.11 个/帧、90% 帧为 0、运动像素 0.42%，而检测下限约 0.07 m/s（比赛车速 22~59 px/s 高 4~12 倍）⇒ 有车行驶必被检出，但没有。可作定性佐证/人工标注底片（分米级），外测仍标未通过。
- **零写入声明**：老帧对照在共享仓执行但未改任何文件（输出全落 p0/），隔离副本只被只读调用。
- 路线（已与用户对齐）：P0 侦察 → P1 首段前进契约 + test_corner_cut 门禁补全 → P2 random18 jerk 观测 → P3 第七图净空修复 → P4 同 64 图复测 → P5 全量回归 + 外测 → P6 运行链接入（受写权门控）。本轮未提交、未部署、零代码改动。
- 19:16 重派记录：P0 首派四合一子智能体失败、零产出（p0/ 目录为空、无新文件）。已拆三路重派：③第七图诊断→p0/section7/、①老帧64图对照→p0/oldframe64/、②分族普查+random-01钻取→p0/families/（纯读不占CPU）；④两张超时图的串行复跑改由 Lead 亲自在 CPU 空闲后执行，避免并发污染计时。三路均要求边跑边落盘。

## P0 复核与重派（2026-10-25 20:5x · 用户指令「重建子智能体重新跑，先复核再定下一步」）

- Lead 复核既有 P0 产物：
  - `_tmp_session/traj-1025/frame-migration/random64-results/summary.csv` 64 行；`candidate_completed=True` 7 张，sim_time 205.46/146.42/196.34/198.20/195.30/147.78/184.14 s ✓；含射击总时长 265.46/206.42/256.34/258.20/255.30/207.78/244.14 s（206.42~265.46）✓ —— 与 P0-SUMMARY 及卡面一致。
  - jerk/override/减速 不在 summary.csv 而在逐图 audit JSON：本卡 "jerk 5/7、override 5/7、减速 2/7、网格横向 1/7" 须从 audit 重算，已交 C 路复核。
  - **口径矛盾（已确认）**：卡面记 "本机无 ffmpeg、抽帧留 P5"，但 p0/video/ 实有 cv2 抽的 7 帧 + contact_sheet + motion_strip + zoom 图 + verify.json —— 本机 cv2 可用、抽帧已做但**结论未落盘**；且当前模型无图像输入，只能数值判定。该条改判 "视频内容定量验货未完成"，交 C 路 C3 补。
- 重派（上一轮 fbaac6bf/b790f392/389bc701 三路全灭后重建）：
  - A 新帧 64 图独立复跑 → `p0/recheck/A/`（核 summary 49/8/7、7 图时间、jerk/override）
  - B 老帧 64 图独立复跑 → `p0/recheck/B/`（核 49/49 同集、8 对 8、7/8 同图）
  - C 第七图静态+物理、random-06/19 串行复跑、视频定量验货 → `p0/recheck/C/`
  - 三路均只读源码、输出只落各自目录、PYTHONDONTWRITEBYTECODE=1、禁 git 操作；结果回填本卡后再定 P3/P4 顺序。
## P0 复核收口（2026-10-25 21:45 · A/B/C 三路独立复跑完成）

三路均为独立子智能体（不采信 Lead 中间结论、各自重建分类口径从零跑），产出只落 `p0/recheck/<路>/`，源码零改动、零 git 写、零连车。

- **A（新帧 64 图独立复跑，`recheck/A/`）**：64/64 张 `stage` 与基线逐图相同（planning 49 / audit 7 / compile 8），7 张完成图 `sim_time` 7/7 逐位一致，0 差异 ⇒ 新帧基线完全可复现。额外发现两处基线口径错误：①override 实为 6/7（基线叙述 5/7）；②"jerk 最大≈49.5（random-18）"不成立——7 张中 jerk 最大是 **random-24 = 96.8019**，49.5174 只是 random-18 的值，若把 49.5 当上限引用会低估 2 倍。
- **B（老帧 64 图独立复跑，`recheck/B/`）**：与基线老帧 64/64 张桶级完全相同（planning 49 / compile 8 / audit 6 / audit_fail 1）；与新帧交集 49、双方独有 0 ⇒ "49 张同集、非新帧回归"复现成立。编译失败 8 对 8（老独有 8、新独有 7）、完成 6 对 7（交集 5，老独有 7、新独有 8/63）。全量 64 张 946.4 s。唯一偏差：失败族分布 18/13 → 20/11，8 张图在"超时 vs 结构自检"间翻转，根因 `planner.py:2942` 墙钟 `deadline=t0+max_seconds`（2 s），边界图随负载翻转（8 张单独复跑 8/8 全变结构自检）；不影响集合判定，但基线"超时 13→2、结构自检 18→29"迁移表不宜当强信号（噪声 ±2）。
- **C（第七图 + 超时图 + 视频，`recheck/C/`）**：C1 第七图逐数字复现——静态 PASS、`errors=[]`、obstacle 名义 2.286 cm（硬线 0.02）、trajectory 回放 67 违规全在 G10、t=7.1200 s 起、最小净空 1.1613 cm（越线 0.84 cm）、`obstacle_hit_steps=0` 未触本体；旧折线臂 41 = G10 32 + I6 9。C2 两图 4 次复跑判定文本逐字相同（确定性失败），并加做反证：`max_seconds` 2.0→30.0 仍同理由结构失败 ⇒ 超时族并入结构族成立。C3 视频定量验货结论见上（不能当真源）。

**本卡口径修正汇总（正文已改）**：override 5/7→6/7；"jerk 最大 random-18 49.5"→最大为 random-24 96.8019（random-18 49.5174 不是最大）；"严格压雷 8/8"→车心 8/8 / 后轴 5/8；外测视频→定量验货完成，结论"不能当真源"。

**下一步决定**：P3（第七图 G10 加 tracking reserve）依据成立，优先开工；P2 观测目标按 A 的更正改为 random-24（jerk 96.8），不是 random-18；C2 的"2.0 s 预算对随机图偏紧"单列一条交规划 owner（不归本层）。

**A 席补记（2026-10-25 21:5x · 影响后续裁决的两条）**：①**random-22 的网格横向加速度 = 0.350028 m/s² vs 上限 0.35，只超 2.8e-5** —— 边界抖动即翻面，不得当作"新帧引入横向加速度问题"的证据；②random-06/19 实际规划耗时 **1.13–1.24 s（2.00 s 预算尚余 0.76–0.87 s）**，4/4 次复跑全为结构失败 ⇒ 原"超时"应改写为"**边缘图**（结构不合格、恰在墙钟线附近返回）"，与 C 的 30 s 反证互证。A 还自证：64/64 map_sha256 与 config_sha256 未变、5 个源文件 sha256 逐位相同、7 张完成图 6 类数值 Δ=0.000e+00（容差 1e-12）。

## P3 执行记录（2026-10-25 22:0x–22:3x · 第七图 G10 加 tracking reserve）

- **改动**（仅 `tools/route_check.py`，默认 0 = 现行为不变）：`TrajectoryFieldGate` 新增 `obstacle_tracking_reserve_m`，且**两阈值拆开**——`obstacle_surface_m` 仍是验收硬线 2cm（审计沿用），`obstacle_search_m` = 2cm+reserve 只用于连接搜索的 pose_valid；CLI 新增 `--obstacle-tracking-reserve` 并透传。
- **扫描结果**（同图同参，只变 reserve，详见 `p3/REPORT.md`）：

| reserve | traj 时间 | G10 违规 | 回放最小净空 | jerk | 总时(含射击) |
|---|---|---|---|---|---|
| 0.0 / 0.002 | 138.40 s | **67** | 0.0116 m | 10.286 | 198.40 s |
| **0.003** | 137.80 s | **0** | 0.0342 m | 1.000 | 197.80 s |
| **0.004** | **130.74 s** | **0** | 0.0342 m | 1.000 | **190.74 s** |
| 0.005 / 0.006 | 158.54 / 137.80 s | 0 | 0.0341/2 m | 4.945/1.000 | 218.54/197.80 s |
| 0.008 / 0.01 | 128.34 / 167.68 s | 0 | 0.0341/0 m | 10.296/70.535 | 188.34/227.68 s |

- **结论**：①修法有效且余量极小——**≥3 mm 即把 G10 的 67 违规清到 0**，净空 1.16cm→3.42cm、未触本体；②**4 mm 为当前最优平衡点**（时间比基线还短 5.7s，jerk 恰在上限内，总时 190.74s）；③**响应非单调**（不同档落到不同连接链），不能当连续可调参数，必须离散验证；④**时间问题独立于净空**：8 档最好 188.34s 仍超 180s → P4 必要；⑤jerk=1.000 恰等于上限是 hard-cap 边界值，不能当"稳定达标"，交 P2 观测。
- **回归**：`test_corner_cut` 仍 3 红（5.5/5.8/5.9，与基线 89/92 一致，无新增红）；`test_physics` 报 `config/map.yaml` 缺失——**既有夹具缺失**（两仓均无 map.yaml，D-019 刻意不推），与本次改动无关。
- **建议（未执行，等裁决）**：config 增 `planner.obstacle_tracking_reserve_m: 0.004`（现场可调 3–5 mm），但 jerk 边界值先交 P2；运行链接入时 reserve 必须与现场实测跟踪误差匹配，不能照抄仿真 4 mm。
- **写域声明**：只改 `tools/route_check.py`；未改 config、未改源逻辑、未提交、未部署。

