---
卡号: LS-LIVE
ns: laptopstep-1028
agent: lead
owner: lead
reviewer: stop-review
树节点: LS-4
写域: _tmp_session/laptop_step_control/lead/；本ns树；共享治理登记；TEAM_STATUS增量段
kind: implementation
status: in_progress
创建: 2026-10-28
更新: 2026-10-28
来源: 本轮用户请求；dzrobot_v2/docs/决策台账.md
取证: 实
摘要: 整合、设备静态及单段实车；本地实验，不将离线结果冒充实车通过。
---

# LS-LIVE · 整合、设备静态及单段实车

## 读过的台账条目
D-039布局构造器唯一真源；D-051运行面硬接管优先；D-055 ttl=0不是失联停车；D-057是历史另一趟，不作本轮授权。

## 复用检索结果
docs入口及空地链路检索；复用清单→能力索引→目录索引，复用dzbrain.scout.layout.build_layout、io.contract.TOPICS、StepLoop及Tracker；不重建规划器或控制器。现有supervisor不是透明双向桥，旧发射器会自动推进，不满足人工逐段及锁存停车。

## 新增轮子及理由
本轮仅实验目录中的最小JSONL适配、纯状态机与视频来源薄适配；独立驻车端服务负责本地单调租约；无新依赖、无生产配置键。写入前已在本卡及共享任务登记；实际文件与哈希见下文独立证据。

## 写域
_tmp_session/laptop_step_control/lead/；本ns树；共享治理登记；TEAM_STATUS增量段。禁止改冻结生产源码、默认配置及他人文件；禁止commit/push。除Lead外禁止设备连接、部署及运动发布。

## 当前直接执行入口（用户最新裁决）

用户已改指[4.mp4](D:/User/Desktop/无人机视频/测试素材/4.mp4)，并明确“不验证，直接放入制作好的程序链路里执行”。停止本轮额外审计/回归/独立重放；原候选保持冻结，源输入拒收仍按程序真实错误记录，不旁路填图或伪造回执。

~~~bash
bash _tmp_session/laptop_step_control/lead/resume_4_cached_1793200500.sh
~~~

[原有入口](../../../../_tmp_session/laptop_step_control/lead/capture_final_video_evidence.py)已参数化接片，复用既有run_video→ObservationAdapter→build_from_observation，不另写识别/布局/规划。历史全片实际run_id=LSPROGRAM-1793197278785180900，已回收后台job=bash-d23db4f3c05b4f59；后续仅用上述缓存入口，不再运行whole-clip。视频2347帧、30fps、720×1280，逐帧处理而非抽帧替代；实际阶段结果写入每块chunk与stream，完成后写run.json。尚无下发/运动，不将程序执行称为全链路完成。

## 历史验收命令（本次不追加执行）
工作目录=v1；前两项预期exit0，仅说明实验功能/边界；真实重放脚本预期捕获NO_GO且不做任何设备操作，每次会创建新run和完整JSON，不冒充旧原件。仿真闭环的实际命令/输出见独立仿真报告；field_modeled=0不作现场安全证明。

~~~bash
python -B -X utf8 -m unittest discover -s _tmp_session/laptop_step_control/tests -v
python -B -X utf8 -m unittest discover -s _tmp_session/laptop_step_control/review -p test_independent_video.py -v
python -B -X utf8 _tmp_session/laptop_step_control/lead/capture_final_video_evidence.py
~~~

## 验收清单（领卡初始全false）
- [ ] 单段正确性及真实回执匹配（离线/仿真已验，本趟实车未验）
- [ ] 重复、旧回执、超时、断链与重连不续跑（离线已验，本趟物理停车未验）
- [x] 独立reviewer复跑并记录退出码
- [x] 冻结区增量0及门禁结果明确
- [x] 实车未证项与残余风险明确

## 回滚点
原生产零改动；停止本轮新进程，保留证据，仅移除本卡新增实验文件。共享登记按本轮增量外科逆转，不reset、不覆盖他人。设备写入另需部署清单/哈希和回滚记录。

## 执行记录
2026-10-28 Lead建卡。基础check.py GREEN/exit0；根仓和车仓均有大量既有改动，全部保留。用户已确认本轮N10朝M10、清场及有人保障；软件验收未完成前不发车。

## 本轮阶段性证据（不等于发车通过）

- 用户本轮现场确认：N10朝M10、已清场且有人保障；视频选“沿用，现场布局与视频一致”，指定实飞3。
- [本轮只读车端库存](../../../../_tmp_session/laptop_step_control/lead/car_inventory.json)与[被动DDS图](../../../../_tmp_session/laptop_step_control/lead/car_passive.json)：车端可SSH，实际无dzbrain、无odom、无pursuitAngle消费者；没有用历史ready状态替代本轮现状。
- [实验部署与哈希对账](../../../../_tmp_session/laptop_step_control/lead/verified_runtime_manifest.json)：只经现有deploy_to_car模块push_file/verify_pushed，向先证不存在的新实验目录窄推protocol/guard/transport/car_bridge。四文件local/remote SHA逐字一致；未覆盖生产配置/包/地图，未build/起脑。
- [真ROS锁停订阅](../../../../_tmp_session/laptop_step_control/lead/static_service_observation.json)：独立systemd用户服务发manual1，被另次SSH中的真实ROS订阅收到；override发布方独占，QoS RELIABLE。只证明这个消息到订阅者，不证明底盘实际停稳。
- [笔记本SSH静态握手](../../../../_tmp_session/laptop_step_control/lead/static_probe_result.json)：5次challenge/lease互通，最终LATCHED，car_instance=null，sync/permit/layout全部0。无车端脑时拒运动正确，未伪造car_state/GID。
- [自有服务回收](../../../../_tmp_session/laptop_step_control/lead/static_service_stopped.json)：核对具体PID/cmdline后只停本轮unit，stop_rc=0，MainPID=0，ActiveState=inactive。未动其它现场栈。
- [第一段视频新识别](../../../../_tmp_session/laptop_step_control/lead/video_window/execution.json)：120帧真实处理、0drop、地标/定位0，仅开头4.4秒，不推断全片无定位。
- [全片按录制节奏回放](../../../../_tmp_session/laptop_step_control/lead/video_replay_full/execution.json)：源4321帧时间轴完整覆盖，CPU实际处理210/显式丢4111，10帧有定位；不是全4321帧推理，不是零定位。[真实定位行摘要](../../../../_tmp_session/laptop_step_control/lead/video_replay_full/analysis.json)保留10帧raw语义/启发FOV用于定位缺口，不充当可用地图。
- [独立仿真报告](../../../../_tmp_session/laptop_step_control/review/1793193849571897400_sim_report.json)：N10→M10 completed=true、越界0；field_modeled=0，不作现场障碍/桥洞/边界安全证明。
- 当前无运动发布、无射击/飞行、未发车。原固定NO_GO诊断器的功能缺口已由step-contract接现役H_world、同域去畸变有效mask、整格投影与语义适配修复；不是另造规划器。
- [最终独立窄范围复核](../../../../_tmp_session/laptop_step_control/review/1793195966758132300_suites.json)：作者整套110/110（含视频53）及独立来源10/10，均由stop-review实际执行exit0、前后hash稳定；两条先红反例（零质量H仍出图、像素/world/cell未互核）已在新候选收敛，未改独立断言。仅说明实验功能和验证边界，不说明本片全局锚定或实车可用。
- [最终完整新视频原件](../../../../_tmp_session/laptop_step_control/lead/final_video_1793196044323982300/full_result.json)：run_id=LSVIDEO-FINAL-1793196044323982300，源候选3e090903，8帧实际处理/8定位/0drop；完整H、mask、raw对象和每帧结论先落JSON再摘要，不是旧d5c窗的丢失stdout重建。13个诊断known仍不含N10/M10；新校核以object_world_pixel_cell_conflict:F11/K11拒绝，observation=null，layout未构造，hardware_go=false。[独立静态分类](../../../../_tmp_session/laptop_step_control/review/1793196395732969600_window_gate.json)已证：这两个点的逆H半量化区间与原world相交，拒绝来自格心域margin0与原pnp边缘域margin0.10不一致，不是已证像素/world数值或物理矛盾；原片全局锚定仍未证。旧窗仅保留[精确派生工具摘录](../../../../_tmp_session/laptop_step_control/lead/retained_replay_event1231.json)，不冒称完整原始JSON。
- 冷启动纠正：现役main先起原DroneTask listener再等布局，该listener可先发真实GID的空phase/无version car_state；guard已有显式fresh-start=N10/base0同步。静态有复用路径，不再报告“必缺bootstrap实现”；真实空phase/GID冷启动、实际到点停稳/断链停车仍未测。
- 本阶段结论：实验来源适配有正例、实际首段NO_GO。后续只定向修实片全局锚定/边缘域，不能以P0起点或旧桥补视频覆盖，不能以离线绿替代现场停稳。

## 最终原件独立复核

stop-review只读最终新run原件核字节哈希与候选一致，再将原件raw喂回适配器复现同NO_GO/同known/同reasons。证据见[独立原件复核](../../../../_tmp_session/laptop_step_control/review/1793196201403147700_window_gate.json)。由于源先拒，未调用首段layout builder、未尝试permit，不能把它写成“已构造首段并跑车”。

## 4号素材直接执行结果（非新增验收）

上述后台job已collect：进程exit2，整段2347帧（0–2346，PTS0–78200ms）全部处理，1861帧有原识别器定位，64帧曾产来源observation；这些候选送既有build_from_observation时未能构造N10→M10首段。最终N10和M10都在诊断known中，但最终observation=null且程序已累计格号/类别一致性拒收原因，因此不能再把本素材问题简化为“全片起点不可见”。[直接运行原件](../../../../_tmp_session/laptop_step_control/lead/program_run_1793197278785180900/run.json)记录decision=LAYOUT_REJECTED、layout_runtime_error=“current and target cells must both have video observations”、input_exhausted=true、layout_constructed=false、layouts_sent=0、motion_commands=0。last_stage字段仍保留入口初始值，不能据此否认已调用构造器；以decision/exception及实际代码路径理解。

本次仅实际输入主链执行，没有追加测试、模拟done、补旧图、发布布局或起车。替代全流程仍未跑通；卡及目标保持在办，不自报完成。未继续安排成员验证。

## 本次真实车端启动结果（按用户要求直接执行）

用户随后要求“太慢，可以直接开始实际运行，加入车端规划和循迹”。Lead没有追加离线测试，实际启动自有租约监护、定位及原dzbrain入口；原启动shell虽然存在，但真实dzactuator_node已死，仅有TF子进程，首次原程序因odom发布方0/转向消费者0以exit6拒启。只启动缺失的原dzactuator_node（保留旧TF/他人进程），加原定位后第二次真实启动成功：串口打开，定位回报N10/+90度附近，原live输入契约通过、真实odom发布方1/转向消费者1/布局发布方1。射击与扫描授权在此进程内重映射到inhibited命名空间；未启动云台/裁判/无人机、未改生产文件或默认配置。

随后原程序实际等待field_layout 60.18秒，收到0条布局，以exit3自行brake/shutdown；归档里程0.0m。不能把启动原程序/端点接通称作已进入规划或循迹，Tracker/真实done未发生。当前断点仍为本素材观测→布局构造→下发，未旁路填known或造布局。

[第二次真实启动原件](../../../../_tmp_session/laptop_step_control/lead/direct_car_native_result_4.json)；[实际结束和自有进程回收](../../../../_tmp_session/laptop_step_control/lead/direct_car_finished_4.json)。所有本轮自有unit最终MainPID=0；监护最后stop_rc0/inactive；底盘与定位自有unit在180秒限定后退出。未停旧TF/他人进程，未发permit/运动/射击/飞行命令。全目标仍在办，后续应复用本次已落盘素材，不再从头整片推理或追加审计。

## 缓存直连续作（本轮开始）

只修改本卡现有Lead入口，增加已落盘观测到既有laptop.main的实际接缝；无新识别器/地图/规划器。沿用D-039/D-051/D-055，不改已裁决安全口径。复用检索=既有capture入口、video_source.read_jsonl、laptop.build_from_observation和laptop.main；原入口只存layout、未调用实时控制器，故需补该接缝。回滚=撤回本轮入口增量，生产与车端保持原样。按用户最新指令不追加审计、回归或独立预验证。

- [x] 已保存观测实际送入原构造器，成功或拒收可机器判定（本次真实exit2拒收）
- [ ] 布局成功后进入既有人工逐段控制器；真实下发/规划/循迹/done分开记录
- [x] 无全片新推理、无补known/消冲突/绕过锁停

本轮已按原输入程序实际执行缓存直连（启用live入口，但只在构造出合法布局后进入原人工控制器），耗时约0.91秒，exit2/SOURCE_REJECTED；消费既有2347条帧记录，原构造器实际收到64次来源observation，均因N10/M10未同时覆盖拒绝。首个源冲突为frame254的H7类别冲突，后续包括M10类别冲突；最终诊断known含N10/M10不代表observation可用。新推理0帧，controller_entered=false、下发0、运动0。本轮未连接SSH/重启车栈，未消冲突或补known，规划/循迹/真实done仍未发生；实时成功分支尚未实际走到，不能勾完成。结果见[缓存直接执行原件](../../../../_tmp_session/laptop_step_control/lead/program_run_1793200530310965400/run.json)，复跑入口见[本次实际命令](../../../../_tmp_session/laptop_step_control/lead/resume_4_cached_1793200500.sh)。撤回本轮入口增量可恢复[原入口字节备份](../../../../_tmp_session/laptop_step_control/lead/capture_final_video_evidence.py.before_cache_1793200500.bak)，不触及生产或他人进程。

## 用户指定航点实车（2026-10-28 深夜，非视频链路）

用户停止视频工作，改为直接用原车端规划/循迹跑指定航点。全部为原 `run_live`，未改生产默认、未飞行未射击、未起云台/裁判/无人机：

- 第1趟 [run_1793202242631252700](../../../../_tmp_session/user_waypoint_run/run_1793202242631252700/car.log)：N10→A9→E8→K10，120.53 s / 8.22 m / 3 段全接受 / 退出原因 finish。
- 第2趟：N10→A9→E9→H8→K10，125.86 s / 8.33 m / 4 个航点全登记 / finish。
- 失败趟：`复位串为空` 程序自行拒启，未动车；作为"程序自身拒错"证据原样保留，未伪造出发。

**现场纠正（重要）**：第1趟程序判定到位 A9 时，车实际停在 A9/A8 格线上，横向残差≈0.171 m 偏车右侧；同日志 belief 终点距 A9 格心仅 0.004 m。即「到位吸附」会把真实横向残差抹成 0，程序全程自认精确。详见[横向偏航观测](../../../../_tmp_session/user_waypoint_run/20261029_lateral_drift_observation.md)。未据此改标定或控制器。

## 门禁与影响面

本轮基线及阶段wf/check.py GREEN；数字守卫与断链通过。[最近有序门禁原件](../../../../_tmp_session/laptop_step_control/lead/workflow_1793196337741849800.json)保留每项命令/stdout/stderr/exit；总结果all_pass=false。2026-10-28阶段全库门禁RED：本ns 10条链接片段假死链已改整文件链接消除；仍有他ns的R8缺卡与9条新增锚点红，不越权改他人卡/扩大baseline，未宣称可收口/可提交。生产冻结目录未由本轮实现修改，既有脏工作树保留；不commit、不push。

## 文档变更
本卡及本ns树；正式收口由Lead统一索引。

## 经验总结
现象：无人机不可达不必阻断笔记本模拟输入。处置：保留真实控制闭环，只在实验链路适配；done不等于停稳、心跳TTL不等于停车。
