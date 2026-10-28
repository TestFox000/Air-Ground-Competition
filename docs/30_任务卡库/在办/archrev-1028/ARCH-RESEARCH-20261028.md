---
卡号: ARCH-RESEARCH-20261028
ns: archrev-1028
agent: /root
树节点: AR-1
owner: /root
reviewer: /root/test_research
kind: requirements
status: in_progress
创建: 2026-10-28
更新: 2026-10-29
摘要: 研究当前架构、逐模块职责及适合嵌入式机器人的开发测试模式，区分事实、推断与待讨论建议。
---

# ARCH-RESEARCH-20261028 · 架构与开发测试模式研究

## 目标与边界
用户要求探讨架构优化、agent 工作流及相关方面。只读代码，不部署、不连接设备、不修改配置或控制逻辑。建议不代表推翻既有裁决。

## 读过的台账条目
D-007 人工部署、D-008 冻结区、D-012 唯一规划入口、D-019 唯一部署入口、D-023 失败分类、D-025 本地通过与发布分开、D-044 提交推送由用户决定。只研究，不改变以上结论。

## 复用检索结果
已查资料库检索、系统架构、工作流细则，以及 dzrobot_v2 的复用清单、能力索引、目录索引、工作流v2。仿真沿用 sim_cli；本轮不新增工具。

## 新增轮子及理由
无。现有系统架构速览没有本轮代码核实和改造讨论，因此研究记录留在本卡，不建立第二份架构真源。

## 写域
本 ns 卡与树；Lead 仅追加 ns 登记、生成索引、在状态板注明本轮研究。其他代码和他人卡只读。

## 验收清单
- [x] 当前代码事实带可定位证据，旧资料不作当前实车状态。
- [x] 测试与流程由独立研究者核查；test_research 已读主卡，核实范围内未发现事实错误。
- [ ] 独立 reviewer 复跑文档门禁；新增红归零，存量红如实列出。

验收命令：根目录依次运行 gen_index.py、check_doc_policy.py、check_anchors.py、check_numbers.py（均在 docs/_工具）；dzrobot_v2 内依次运行 tools/wf/check.py、tools/wf/check_links.py。预期退出码零；既有失败不标本卡通过。

## 回滚点
只撤销本卡新增的 ns 登记行、新文件、生成索引中本 ns 行和本轮状态板段落；不覆盖他人文件，不 reset/clean。生产代码本轮无变更。

## 执行记录与结论
开场已读唯一入口、注册表与 lead-1023 现有树；因新议题不抢既有卡，另登记 archrev-1028 并读取本树后领卡。后续证据追加于本节。

## 文档变更
新增本 ns 三卡与树；修改 ns 登记与自动生成索引。研究建议待讨论，卡保持在办，不伪报已实施。

## 用户优先级
本轮用户明确选择「现场调试：故障更容易定位和复现」。优化目标据此改为缩短故障定位与离线复现时间；不是追求更多模块或更多测试数量。

## 当前代码核实（2026-10-28，只读）
| 事实 | 定位 | 对现场调试的含义 |
|---|---|---|
| 任务分层、ROS 接口解码和任务时钟注入已存在 | dzrobot_v2/src/dzbrain/dzbrain/main.py:2498；dzrobot_v2/src/dzbrain/dzbrain/io/ros_io.py；dzrobot_v2/src/dzbrain/dzbrain/tasks/mission.py | 应扩展这些边界，不另写决策内核或仿真器 |
| StepLoop、门禁策略、装配和 live/bench 入口同在 main | dzrobot_v2/src/dzbrain/dzbrain/main.py:1002；dzrobot_v2/src/dzbrain/dzbrain/main.py:2498；dzrobot_v2/src/dzbrain/dzbrain/main.py:3085 | 高变化职责集中；拆分应按职责和对比用例逐步进行，行数本身不证明缺陷 |
| 运行模式取决于地图文件是否存在 | dzrobot_v2/src/dzbrain/dzbrain/main.py:2067 | 建议以后明确模式并打印有效配置来源；本轮不改变既有每局动态地图裁决 |
| 黑板有锁，但值为 Any，读取返回原对象，snapshot 为浅快照 | dzrobot_v2/src/dzbrain/dzbrain/core/blackboard.py:114；dzrobot_v2/src/dzbrain/dzbrain/core/blackboard.py:155 | 锁保护容器访问不等于字段契约或深层对象不可变；建议逐步引入有类型的每拍输入快照，不能据此直接断言已有竞态事故 |
| 任务调度、黑板年龄与业务逻辑使用的时间接口尚未统一 | dzrobot_v2/src/dzbrain/dzbrain/core/task.py:118；dzrobot_v2/src/dzbrain/dzbrain/core/blackboard.py:137；dzrobot_v2/src/dzbrain/dzbrain/main.py:1033 | 完整可重复回放还需要统一注入时钟、事件到达顺序与调度语义；仅录消息不能保证重现线程交错 |
| EventBus 有限队列满时丢弃并计数 | dzrobot_v2/src/dzbrain/dzbrain/core/events.py:88 | 关键事件需要可追踪序号与丢失可见性；不是本轮已证实丢事件事故 |
| 已有最终速度钳制、停车原因及发布失败计数 | dzrobot_v2/src/dzbrain/dzbrain/tasks/tracker.py:681；dzrobot_v2/src/dzbrain/dzbrain/tasks/safety.py:403；dzrobot_v2/src/dzbrain/dzbrain/tasks/telemetry.py | 不应声称项目完全没有原因遥测；应把分散信息按同一运行、段、指令关联 |
| 归档固定六文件，明确在线只写不读旧场次 | dzrobot_v2/src/dzbrain/dzbrain/core/run_archive.py | 离线分析回放必须独立于生产在线输入，禁止自动加载上场地图补当前信息 |

开场 python tools/wf/check.py（cwd：dzrobot_v2）exit 0 / GREEN；这只验证工作流合规，未运行全量功能回归、未连接设备。

## 架构建议（提案，未实施、未裁决）
1. 优先贯通现有归档、遥测与插件诊断。用同一 run_id/segment_id/command_seq 关联输入来源、数据年龄、决策/限制原因、最终发出命令和可获得的底盘反馈；发布成功不能当成执行成功，没有反馈就明确未知。
2. 给每场保存有效配置及哈希、两仓代码身份与脏差异摘要、模型/标定/消息契约版本、启动参数和输入来源。根仓 HEAD 不能单独代表车仓，更不能代表未提交工作树或已安装机器。
3. 复用 sim_cli/bench，增加显式离线回放适配入口与统一时钟，而非第二套仿真。sim_cli 复用控制函数但不跑完整 Mission/射击/PIVOT；bench 运行真实任务与 FakeBridge，但当前依赖真实线程/睡眠，也未装配 StepLoop/RunArchive。回放应优先复用 FakeBridge/FakeClock 与任务 tick；先覆盖一例历史「无运动」故障，再扩大输入与调度覆盖。
4. 保留当前模块化单进程基础。逐步分离任务装配、段级状态机和场景入口；按任务超时、CPU/温度、时延和故障隔离证据决定是否另拆进程。高频控制路径避免同步刷盘。
5. 不新建控制权来源。以后若统一指令仲裁，必须保持现有硬停/降级裁决，证明刹车优先级和命令过期语义；本轮不修改任何安全策略。

## 建议 agent 流程（待讨论）
现场由一名操作者控制部署与设备动作；Lead 接症状并钉运行基线，调查者只读取证，实施者修最小复现，独立 reviewer 复跑。按文件分工，现场故障不同时修改算法与配置。每轮交付可复现输入、唯一假设、证据、回滚点；已有索引和状态板优先由结构化卡生成，减少同一状态多处手填。

## 建议先做的可验收切片
选一例「车不动」：从现有归档还原失败阶段，区分输入被拒、布局/规划未完成、控制主动给零、发布失败、硬件未响应；用同一命令离线得到相同阶段与原因。缺输入必须报不可复现，不补历史真值、不伪造反馈。该切片通过后再决定后续拆分范围。

## 外部一手资料（设计参考）
- ROS 2 Managed nodes：https://design.ros2.org/articles/node_lifecycle.html 。配置与激活分开，有明确状态与失败处理；不要求本项目立即迁移所有节点。
- ROS 2 launch_testing：https://index.ros.org/p/launch_testing/ 。验证实际进程输出与退出状态，可作为现有纯 Python/仿真测试以外的 Linux 集成层。

## 独立研究补充
### 测试研究者 test_research
- 已有归档缺少完整原始输入与命令关联，不把六文件齐全解释成可回放；定位在 dzrobot_v2/src/dzbrain/dzbrain/core/run_archive.py:137 与 dzrobot_v2/src/dzbrain/dzbrain/main.py:1321。
- 已复现一处真实接线错误：dzrobot_v2/src/dzbrain/dzbrain/main.py:1282 和 dzrobot_v2/src/dzbrain/dzbrain/main.py:1314 传入 trace 的 event 关键词，与 dzrobot_v2/src/dzbrain/dzbrain/core/run_archive.py:193 的首参数同名，异常又被外层捕获。影响 attach/map_version 轨迹，未证明所有归档事件失效，未修改代码。
- 可复跑只读命令（cwd 为 dzrobot_v2）：
```powershell
$env:PYTHONPATH='src/dzbrain'
python -B -X utf8 -c "import inspect; from dzbrain.core.run_archive import RunArchive; exec('try:\n inspect.signature(RunArchive.trace).bind(object(), \'brain\', event=\'map_version\')\nexcept TypeError as e:\n print(\'REPRODUCED:\', e)\nelse:\n raise SystemExit(1)')"
```
预期 exit 0，输出 `REPRODUCED: multiple values for argument 'event'`。这是故障复现通过，不是修复通过。
- 当前单测包括直接调用归档器与替身 body 收尾，不能代替真实 StepLoop 到归档器的接线测试；定位在 dzrobot_v2/src/dzbrain/tests/test_agc_rectify_rt_gate.py:574。

### 流程研究者 flow_research
- 现行推进计划已有 E3 RUN-EVIDENCE：docs/10_agent提示词/04_场景/空地协同闭环推进.md:121 。本卡第1/2建议属于落实和补全既有计划，不是另建平台。
- 状态板最近几次记录区分 SOURCE_REJECTED、0 layout 和 LAYOUT_REJECTED，并明确未进入循迹。此为状态板记载，本轮未连接设备复核；应先解释来源→观测→布局的首个失败边界，不能据此去调下游控制参数。
- 现有本机插件已有 source/live/cached、新鲜度和原因文案；仅核源码，未确认 UI 当前挂载。不建议另建仪表盘。
- 父仓忽略独立车仓，且两仓均有在途变更：基线应含两仓身份、脏差异、实际进程路径和安装产物哈希。禁止为了做基线擅自提交。
- 现场一故障一卡容纳多趟，先采机器字段与关键观测，跑后统一提炼。新按风险分级门禁与自动生成状态板仅为后续提案。

## 本轮守卫结果（主 agent 执行）
| 顺序 | 命令 | 退出码与关键结果 |
|---|---|---|
| 1 | python docs/_工具/gen_index.py --ns archrev-1028 | 0；增量登记本 ns，保留外部行 |
| 2 | python docs/_工具/check_doc_policy.py | 1；他 ns 的 agc-1026 树指向不存在的 LIVE-RUN-20261028 卡 |
| 3 | python docs/_工具/check_anchors.py | 1；新增红9条，均在本 ns 以外；存量红268 |
| 4 | python docs/_工具/check_numbers.py | 0；可重算48/48，4项需人工核实 |
| 5 | python tools/wf/check.py（cwd：dzrobot_v2） | 0 / GREEN；冻结脏项19条均有登记 |
| 6 | python tools/wf/check_links.py（cwd：dzrobot_v2） | 0；checked files 264 / broken 0 |

未扩大或重建基线，未修他人写域。本卡未达到全库收口条件，不报可提交；研究结论已交付讨论，建议尚未实施。快照仅大收口/每月要求，本轮不属于该类。

## 独立复核结果
test_research 在最终研究记录落盘后按相同顺序独立复跑五项只读守卫：退出码依次为 1、1、0、0、0，与上表一致；本 ns 无新增红。主卡复现命令原样执行 exit 0，返回 event 重复实参的 REPRODUCED 信号；核实范围内未发现主卡新增事实错误。全库红线仍在其他 ns，正式归档保持等待，不宣称全局通过。

## 2026-10-29 · 逐模块架构评估续研
用户继续询问各模块架构是否合适。本轮复用原卡与独立研究者，不新增管理体系，不实施生产改造。下面是基于源码职责的架构建议，不是性能、闭环或实车验收结论。

### 车端与公共设施
| 模块 | 判断 | 演进方向 |
|---|---|---|
| 入口、StepLoop 与 Mission | 层次概念合理，装配/段协同/任务执行的实现职责集中 | 保留一个明确的段请求与结果协议；先抽离装配和任务适配，再拆停站、情报与命令处理 |
| 规划器 | 单一入口、地图输入和规划结果边界适合现状 | 保留 tour 与唯一入口；内部按搜索、任务约束、路线编译整理，不另写规划器 |
| pursuit 算法 | 几何、速度、每拍控制的分层值得保留 | 逐步用显式输入替代算法对黑板的读取；连续轨迹仍按候选验收，不能因为类存在就认为默认启用 |
| Tracker | 执行适配层位置合理，运动模式职责较多 | 模式可拆成内部状态对象，统一返回命令结果；保持正常运动最终限速和发布出口，安全制动优先语义不变 |
| Localization 与 Fix | 定位发布与误差估计分离合理 | 保持 Fix 只估计；匹配、修正和可信度判断抽为可回放计算，公共路线进度明确写权 |
| Safety | 监测、原因汇总、锁存和显式恢复职责内聚 | 保留独立模块；明确许可、限制、制动请求、发布结果与物理停车证据，沿用现有硬停/降级裁决 |
| Blackboard、状态类型与 EventBus | frozen 状态类型和非阻塞总线是可保留基础，跨模块写权与时钟仍需收紧 | 有类型的输入快照、版本和来源；关键完成协议明确丢失/重发语义；不能把容器有锁当成整轮状态一致 |
| 配置中心 | 集中 SCHEMA 和校验合适 | 逻辑上区分标定参数、算法设置和本场运行计划；固化最终生效值与来源，不直接改变现有默认值 |
| ROS 与硬件适配 | contract 与 RosBridge 分离适合无 ROS 测试 | 历史协议在适配边界终结，业务数据明确单位与坐标；后续类型迁移须兼容现役设备 |
| 遥测、归档与测试 | 独立观察职责正确，当前三者尚未贯通现场因果链 | 优先补运行身份、输入/决策/命令/反馈关联；复用 sim/bench，不增加第二套内核 |

### 当前代码证据与适用边界
- dzrobot_v2/src/dzbrain/dzbrain/tasks/mission.py:515 动态加载工具校验器；dzrobot_v2/src/dzbrain/dzbrain/tasks/mission.py:950 规划适配；dzrobot_v2/src/dzbrain/dzbrain/tasks/mission.py:1831 段请求；dzrobot_v2/src/dzbrain/dzbrain/tasks/mission.py:3509 停站执行。建议分解的是职责，不能仅以文件长作缺陷结论。
- 公共路线进度多处写入：dzrobot_v2/src/dzbrain/dzbrain/tasks/tracker.py:695、dzrobot_v2/src/dzbrain/dzbrain/tasks/localization.py:417、dzrobot_v2/src/dzbrain/dzbrain/tasks/mission.py:2954。建议增加路线代次、位姿序号和来源的一致快照；控制器仍可保留私有高频投影。本轮未证明新的竞态事故。
- dzrobot_v2/src/dzbrain/dzbrain/core/state.py:16 已有 frozen 状态；dzrobot_v2/src/dzbrain/dzbrain/core/blackboard.py:155 仅浅快照；dzrobot_v2/src/dzbrain/dzbrain/core/events.py:88 队列满时计丢失。应完善现有接口，不把现有系统概括为完全无类型或无丢失监测。
- dzrobot_v2/src/dzbrain/dzbrain/core/config.py:819 将 override 描述为 bench/测试入口，但 dzrobot_v2/src/dzbrain/dzbrain/main.py:2958 和 dzrobot_v2/src/dzbrain/dzbrain/main.py:2983 在 live 装配期间会改 auto_start/route.tokens。这是静态设置与运行计划边界需要澄清的证据，不代表本轮已证明线程间配置竞态。
- dzrobot_v2/src/dzbrain/dzbrain/io/contract.py:60 声明里程计为格单位；dzrobot_v2/src/dzbrain/dzbrain/io/contract.py:77 声明 Twist.angular.z 为舵角度。现役契约必须兼容；业务层宜用显式类型和单位，避免标准工具按不同语义解释。官方 Twist 定义为线/角速度：https://raw.githubusercontent.com/ros2/common_interfaces/humble/geometry_msgs/msg/Twist.msg ；ROS 单位与坐标约定：https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0103.rst 。

test_research 已独立检查车端算法/任务职责，建议先统一运行/路线/输入身份与进度契约，再拆 Mission，随后整理 Tracker 运动模式；规划器整理与候选轨迹晋级分别单卡推进。开场 wf/check.py exit 0 / GREEN；本轮不以此宣称功能或实车通过。

### 机端、感知和云台（flow_research 独立核查）
| 模块 | 判断 | 演进方向 |
|---|---|---|
| px4_path → px4_control → PX4 | 路径请求与飞控执行权分离合理，保留 | 围绕超时、失效定位、取消和停止路径补确定性测试 |
| FAST-LIO 与定位转换桥 | 定位算法和飞控转换桥分层合理，保留 | 将坐标转换、跳变和 reset 守卫抽为可回放纯计算 |
| 机端感知流水线 | Detector、Classifier、Localizer、Broadcaster 职责已分开，保留 | 统一帧身份、采集时间、模型/标定版本与拒绝原因；仓内多条实现不代表当前都在机上运行 |
| 机车感知/布局链路 | 多种输入入口可以存在，情报与可规划布局的契约需优先补全 | 每条边分别报告输入、接受、拒绝及原因；不能把收到情报解释为布局已接受 |
| 车端 det_node | 独立推理节点合适 | 先做兼容解码和有名字段的内部观测，再评估外部消息迁移；带上源图时间、帧号和模型身份 |
| Python AimTask 与 C++ 云台 | 任务级策略与终端执行分离合理，跨层观测/许可关系需优先明确 | Python 管搜索策略/运动估计与许可，C++ 保留伺服、扫描执行与最终两键联锁及租期；不能概括成两个模块都直接争抢云台 |
| dzjudgment | 独立裁判串口与激光适配层合适 | 关联命令、串口发送、反馈；分清发布成功、发送成功、实际出光与命中 |

- dzsmartscout/src/px4_control/src/px4_control_node.cpp:58 接收指令、同文件67行发布飞控设定值、581行有超时；dzsmartscout/src/px4_path/src/path_follow_waypoint_node.cpp:31 消费许可。此为源码边界，不是本轮飞行验证。
- dzsmartscout/src/uav_camera_code/onboard/dzuav/src/main.cpp:70 的帧处理组合四个职责对象。多输入链路需要关联到实际进程/版本，不能仅凭文件存在判当前部署路线。
- dzrobot_v2/src/dzbrain/dzbrain/tasks/aim.py:11 区分 fire_gate 与 fire_ready；C++ 最终联锁保留两键各自租期。当前外部 lead 在开火误差判据中使用，未直接进入该段伺服误差：dzrobot_v2/src/dzrknn_dataprocess/src/dzrknn_dataprocess.cpp:299 与 dzrobot_v2/src/dzrknn_dataprocess/src/dzrknn_dataprocess.cpp:113 。这是职责事实，本轮不裁决算法更改。
- dzrobot_v2/src/dzbrain/dzbrain/io/ros_io.py:249 按接收时间解码观测；现有数组长度守卫有价值，但不足以描述源图时序和身份。低频 String/JSON 无须一律替换，须明确 schema、场次/段号/序号和拒绝回执。

test_research 已只读复核本节之前的车端表格，核实范围内事实成立；其建议的 Mission 进度锚点已由2953修正到2954。无源码变更、无新模块、无参数或安全策略变更。

2026-10-29 最终文档独立复跑：doc_policy=1、anchors=1、numbers=0、wf/check=0、check_links=0。本 ns 无新增红；全库仍为其他 ns 的缺卡及9条新增锚点红，不改基线、不扩大写域，正式收口继续等待。架构判断是研究建议，未运行功能回归或实车验证。
