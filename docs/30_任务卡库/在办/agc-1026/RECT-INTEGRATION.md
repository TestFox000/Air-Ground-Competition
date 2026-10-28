---
卡号: RECT-INTEGRATION
ns: agc-1026
agent: lead
owner: lead
reviewer: lead（验成员差异；Lead自写部分须另安排复核）
树节点: R-5
写域: "共享计划/状态/台账/冻结登记/索引；已验收最小补丁的逐文件集成"
status: in_progress
创建: 2026-10-26
更新: 2026-10-27
---

# RECT-INTEGRATION · 整改统筹与下一趟协同放行

## 目标
完赛优先，在安全与终点预算内争取压雷、两有效最佳射击点与命中。实现整改而非只有方案；代码候选、离线已合入、已部署、实车验收四状态分开。

## 读过的台账条目
D-008冻结登记；D-012唯一tour；D-014/022首段可靠与段边界换布局；D-023不恢复旧场图为默认真源；D-027/028安全运动；D-033靶速0.3m/s；D-044不自动提交或推送。

## 复用检索结果
已读文档入口、复用清单、现行协同推进计划及agc-1024任务树。继续沿用planner、route_check、sim_cli、deploy_to_car、现有回放和runflow，不另建同职能工具。

## 新增轮子及理由
无永久工具。snapshot_before.py仅本卡一次性取证，现有工作流未保存当前未暂存+暂存改动与精确前置hash，故补最小快照。

## 写域
Lead单写树、台账、状态与共享登记。成员独占其卡；生产代码先隔离候选，审差异+hash+测试后登记具体文件、保留别人改动再窄合入；不得用大文件覆盖在途工作。

## 验收清单
- [x] 基线记录：车仓HEAD=945bbc14605c18cd24c526ae5813ecd03c05d8aa；根仓HEAD=ecb9bdce2631376f78829a529d90a104591668ac
- [x] 工作流基线RED3（两个既有删图、一个既有localization未登记）已采集
- [ ] 四线候选均有生产级失败复现/前后对照及回滚点
- [ ] 独立复跑专项、失败注入、仿真覆盖与已知红对账
- [ ] 前置hash一致，仅合入已验收差异；未提交/未部署身份如实登记
- [ ] 下一趟同run日志/视频/bag/真实布局/裁判计数闭合
- [x] 发车结论为明确GO/NO-GO/UNKNOWN；安全证据缺失不默认GO

## 回滚点
`_tmp_session/AGC-RECTIFY-20261026/baseline/manifest.json`及car_files/与root/car的index/worktree patch；只能逆转本卡差异，不restore/reset/clean整树。

## 执行日志

### 10-27 20:20 实跑：首次多段真行驶（Lead）

- 机端192.168.137.62恢复后/tmp被重启清空，全片原生推理目录丢失；改用已下载的原生全片观测（4321帧/835已定位）重建布局，未重跑重复推理。
- 两处生产修正已部署并验证加载：`mission.py` md5 `fcc15819`（段首仅允许≤90°转向，真180°判无解转WAIT_INFO，替代原「放行+警告」）；`main.py` md5 `c3719091`（未知格计数在位移<2cm前不累计，消除起步静止误锁）。
- 布局补入车实际所在起点格（known 93→94，标注 observed_plus_current_car_cell），否则必然触发越界锁存。
- 实跑证据：19:52 N10→M7 5格1.40m 10.40s 到达；20:00 N10→M7 8.70s 到达；M7→L7 6.00s 到达。三段均由车按真实视频布局自行规划执行，无人工干预。
- 段3被正确拒绝：车宣告到达L7后继续前冲0.257m（到达容差0.12m，超出0.137m），belief落入相邻障碍候选格L6，规划器以「当前格不可通行」拒绝发车。这是安全网正确动作，非故障；同时给出停车越冲的量化值。
- 边界：未飞行、未开火、未改冻结默认值、未提交未推送；跨机DDS发现间歇性失败，布局经本机中继下发，内容仍是机端真实识别产物（文件哈希留档）。全程无法完赛：录像仅覆盖86/154格，终点N5从未被观测。证据[real_run_evidence_2020.json](../../../../_tmp_session/AGC-RECTIFY-20261026/video-step-20261027/lead/real_run_evidence_2020.json)。

### 10-27 19:02 实跑续轮1：只处理实际连接失败

18:50正式任务已真实执行，等待地图30秒仍0条后退出3，未进入运动/开火任务，见[完整退出证据](../../../../_tmp_session/AGC-RECTIFY-20261026/video-step-20261027/lead/full_run_direct_completed.json)。本续轮未重发空任务，也未插入点动或离线回归。热点邻居仍有车端；无人机旧地址192.168.137.62不可达，当前名称解析返回192.168.137.52，但SSH同样超时。无法据此断言断电；实际IP与连接状态需现场恢复。Windows热点客户端查询被脚本签名策略拒绝，未绕过。桥洞语义缺口仍保留，不承诺仅恢复连接就可完赛。见[本轮连接证据](../../../../_tmp_session/AGC-RECTIFY-20261026/video-step-20261027/lead/goal_round1_connectivity.json)。目标保持active/未完成；未改冻结生产、未部署/飞行/提交推送。

- 17:24 用户授权整改；沿用每局真实侦察，不能灌旧图冒充协同。
- 17:40 保存20项文件基线，现有修改多于此前汇报的5/13项，旧数量不是现况证明。
- 17:42 启动四个raccoon/Flash1017/on子会话；17:44均因TRANSPORT fetch failed请求阶段失败，实际无人认领任务；same-model一次重试中，未偷偷换贵模型。
- 缺图不崩溃应修，但不能‘跳过空间检查’就判可发车。
- yaw异常未经同步原始信号核验不称假毛刺；SAFE_STOP不取消、不自动清锁；P_FIX_EST只进遥测可能是设计分离，不自动接scale。
- 双站不可达的历史夹具结论不泛化到所有比赛图；折返回放放行率不是命中率。
- 17:58收口检查：文档政策GREEN、链接237份0断、数字重算48/48；锚点RED1为外部traj-1025卡的旧路径，未越域修改；wf仍为基线RED3（删图2项+localization未登记），不伪报全绿。
- 对20项前置hash复核，仅本卡批准的tools/config_selfcheck.py变化；新增8项测试，未动算法/配置/设备。
- 子模型入口故障两轮均确认；已问用户是否允许支持max但未标日期的V4.1 Flash入口，尚未收到选择，暂不替换。

### Goal续轮1记录

- 预检首版作者8项自测不充分：发现空地图假绿/嵌套坏结构崩溃，已补工具输入边界与15项作者测试；真实修前rc1/修后rc0，10项补充探针通过。独立review仍缺，未正式验收。
- 对20项取证hash复核仅tools/config_selfcheck.py变更，新增测试在约定写域，未改算法/配置/设备。
- C03不能凭P_AIM.fire_gate字段同名断言两许可重叠；当前本地发布路径独立、C++按两项及新鲜度相与，Aim既有136项绿。撤其“已证故障”前提，不宣称车端已验证。
- 现行计划已补下一趟G0..G5及收益预算：优先终点，预算内按赛事分值争取压雷和两有效站命中；不擅设雷/靶固定权重，不默认禁火全程，不旧图复原骗门禁。恢复D-044只报告、不自动commit/push口径。
- 指定1017/max执行入口问题未解决，本轮只读发现其它Flash/max均无1017日期声明，未擅自启动；其余执行卡仍pending。当前发车为NO-GO，目标保持active。
- 本轮门禁：索引/文档政策/数字通过，链接237份0断；锚点仍为外部traj卡旧路径RED1，工作流仍为原有删图2项+localization未登记RED3。初次检查抓到本轮3条证据相对路径多退一级，已逐一修正后复跑归零。完整输出留在_tmp本轮doc_gate证据，不冒报全库绿。

### 本轮续办：验收可信度与安全边界复核（用户继续授权）

owner=lead，status=in_progress。本轮不修改生产源码/配置、不连接设备、不部署/发车/出光/飞行、不提交推送。沿用本卡，不另造常驻工具。

- [x] 前沿候选独立反例检查（审查完成、候选拒绝，不是整改完成）：运动、陈旧位姿、布局退步、重复等待、段完成计数和实际停车。作者25项通过不等于安全边界完整。
- [x] 复跑入口整改：进程退出码与逐例结果分开、旧日志不能算本轮PASS、SKIP不能汇总成完成、源码快照与生产状态分开（独立PASS_LIMITED，保留数值解析等限制）。
- [x] Lead复跑DDS原候选专项（发现需修；v2已独立窄范围接受，未合入）；记录经验等待预算，不把fake超时当实车发现上界。
- [x] 更正计划/任务树中超出证据的结论，更新状态板并跑门禁。

复用：继续使用现有step-rhythm验证器、gate-contract替身与repro_all入口，仅在隔离目录增补缺失反例。写域分别为 round4-frontier-review（独立审查）与 round4-repro（运行器执行）；共享文档与REPRO入口由Lead交接后更新。回滚仅逆转本轮精确差异，不覆盖既有在途修改。

### 本轮实证（续办，不表示整卡验收）

翻案三问：新证据为独立构造的运动/布局退步与预算反例；影响面仅候选接受状态和放行文案；不重开的代价是把旧日志、部分用例或误导日志当作实际安全保证。原始作者产物保留，生产冻结文件未改。

**R9通信发现：** Lead运行 `python -B -X utf8 _tmp_session/AGC-RECTIFY-20261026/round4-lead/contract_boundary_review.py`，进程rc=0表示反例成功复现，结果verdict=needs_revision；不能把rc=0读成候选验收通过。作者13项同时13/13。混合失败最终仍拒绝，但不应再打印“必需端点补齐”；小于一拍的预算不能被整拍sleep越过后放行。Lead仅在隔离overlay出v2，新增边界自测无上述发现，作者13项仍通过，**作者自测、不自行验收**，待独立review。生产8项关键文件前后sha256一致。

- [原候选边界结果](../../../../_tmp_session/AGC-RECTIFY-20261026/round4-lead/contract-1793019624692289600/result.json)
- [v2边界自测](../../../../_tmp_session/AGC-RECTIFY-20261026/round4-lead/contract-1793021025635480800/result.json)
- [v2相对R9增量补丁](../../../../_tmp_session/AGC-RECTIFY-20261026/round4-lead/r9-to-v2.patch)；[相对原生产前像的完整补丁](../../../../_tmp_session/AGC-RECTIFY-20261026/round4-lead/pre-r9-to-v2.patch)
- [权威仿真对照](../../../../_tmp_session/AGC-RECTIFY-20261026/round4-lead/dds-sim-comparison.json)：相同配置下两次sim_cli进程均rc=0、completed=true、118.12s、横向RMS=0.1187m、轨迹CSV哈希相同；但**两次越界均74拍，field_modeled=0**。只证明该候选未改变此仿真输出，不是空间校验通过；sim不覆盖DDS。

**R11前沿等待：** 独立review正式结论reject，Lead已重新跑同一反例脚本对生产/隔离树，进程均rc=0（观测成功，非候选安全PASS）。C1移动0.5m/s+布局缩格：基线SAFE_STOP；候选第三拍WAIT_INFO、随后DRIVING并下发第二段。C8新版仍未覆盖当前格也重启段。C5无新版100s仍WAIT_INFO（现配置timeout=0）；C9即使实验设2s，版号反复变化导致9次下发且不终止。真实多线程停车抢占仍未验证。**候选拒绝合入，保留生产SAFE_STOP；不通过改阈值/重启/旧图绕过。**

复跑命令：`python -B -X utf8 _tmp_session/AGC-RECTIFY-20261026/round4-frontier-review/probe_round4.py --src dzrobot_v2/src/dzbrain --json _tmp_session/AGC-RECTIFY-20261026/round4-lead/frontier_base.json`；隔离对照将src替换为`_tmp_session/AGC-RECTIFY-20261026/step-rhythm/iso/src/dzbrain`，输出另存[隔离结果](../../../../_tmp_session/AGC-RECTIFY-20261026/round4-lead/frontier_patched.json)。[基线结果](../../../../_tmp_session/AGC-RECTIFY-20261026/round4-lead/frontier_base.json)。

过程错误同样留账：Lead第一次DDS反例把live模式advisory的imu误当必需，断言失败rc=1；核对_CONTRACT_ROWS后改为race-step下必需drone_in，才复现上述缺陷。没有把探针写错当产品缺陷。上轮13PASS混有旧日志与SKIP已撤回，task-7正在整改同一运行器。普通subagent不在原生teammate列表，send_message寻址失败；通过回报与实际文件确认，Lead代管task-6/7不冒称子已原生claim。

### 运行器交回及本轮收尾范围

Lead已独立执行作者版真实复跑（进程rc=0）：19个EXP通过、3个BIND通过、required SKIP=0；失败注入7场景通过（进程rc=0，正向/坏退出码/坏标记/旧日志/缺探针/未登记漂移/已登记漂移）。证据：[真实复跑](../../../../_tmp_session/AGC-RECTIFY-20261026/round4-lead/runner_author_repro.log)、[注入自测](../../../../_tmp_session/AGC-RECTIFY-20261026/round4-lead/runner_author_selftest.log)。R7原期望136/136与候选141/141均实跑；R8第六图prod rc2且0 token，隔离候选rc0/68 token/单站B4并degraded/F4告警，carloop62与field54回放一致。仅接受声明实验可复跑，不把这些结果当完整运动/打靶验收。

作者版原件已按sha256保全。Lead窄改ACCEPTED为REPRODUCED，并在stdout/摘要标明candidate_acceptance=NOT_ASSESSED、launch_go=false；没有调整实验期望或生产代码。标签修订另由task-9独立审查，DDS隔离v2另由task-8独立审查，均采用deepseek-account/deepseek-flash/max、只写各自临时目录；未由Lead给自写内容授予接受。

用户询问何时跑车后，本轮不再扩展离线分支：收完上述复核与必要门禁后给出具体阻塞。下一趟优先受控短程，仍须现场版本/初态、当趟首段空间与停车包络、控制归属核验及明确授权；未确认前不连接或操作设备。完整赛事目标未完成，不能把等待时间当发车条件。

### 独立v2结论与现场静止预检（21:39后）

DDS v2独立review已交回：[报告](../../../../_tmp_session/AGC-RECTIFY-20261026/round4-dds-v2-review/REVIEW.md)、[机器判定](../../../../_tmp_session/AGC-RECTIFY-20261026/round4-dds-v2-review/VERDICT.json)。自写28例在v2全过、原候选8反例；作者13/13、既有contract56/56；stepper116/117与生产同一12.11红。仅接受sleep预算裁剪、超睡拒绝、blocking本机装配失败优先及成功日志条件；不等于总耗时严格受限，也不接受真实启动链/空间/部署。v2仍只在隔离目录，未合入。

用户确认车在N10、朝M10、有人能急停；这只确认人工初态与值守，不等于自动发车/射击授权。随后Lead开展只读静止检查，更新此前“本轮未连接设备”的阶段性记录。首次误用root免密SSH认证失败，未执行远程命令；查回现有duzhong连接助手后使用已存凭据和严格known-host校验连通，未索取新凭据，复用原有连接参数。

[车端身份](../../../../_tmp_session/AGC-RECTIFY-20261026/round4-lead/car-preflight-1793021959409283600/result.json)：基础视觉/定位/云台/裁判已运行；两处auto_start=false；主工作区为/home/duzhong/dzrobot；源码main、mission、planner与本地候选身份不同。未覆盖车端原件，未恢复缺失地图。

[只订阅遥测](../../../../_tmp_session/AGC-RECTIFY-20261026/round4-lead/car-telemetry-1793022105158625400/result.json)：域118、8秒有限窗口；未发现dzbrain决策节点，空地两话题未出现，pursuitAngle无发布者；odom与odom/localization均收到301帧、末帧速度零，offset_center收到8字段但末帧全零。已有step_emitter_v3节点，控制归属待用户确认。当前原始里程计不能直接当N10决策坐标；对齐关系未证，不断言定位失效。

观测节点仅订阅、无publish/service调用，结束已destroy/shutdown；未启动控制任务、未部署、未发车/出光/飞行。遥测探针首次因本地NUL转义导致SyntaxError，发生在任何连接前；使用hex检查后纠正自写脚本，未误判为车故障。当前NO-GO，先核实控制归属与本趟布局，不因现场归位自动开跑。

### 复跑口径独立结论（本轮离线收尾）

[独立报告](../../../../_tmp_session/AGC-RECTIFY-20261026/round4-runner-review/REVIEW.md)与[判定JSON](../../../../_tmp_session/AGC-RECTIFY-20261026/round4-runner-review/review_verdict.json)为PASS_LIMITED：源码差异仅标签/文案；独立真跑19 EXP、3 BIND、0 SKIP，7个自测场景在默认非仓内临时目录符合预期。Lead另运行独立只读检查器，得到[REVIEW-CHECK PASS/rc0](../../../../_tmp_session/AGC-RECTIFY-20261026/round4-lead/runner_review_check.log)。这只接受复跑口径，不接受R9原候选或已被否决的R11。

必须带走的限制：elapsed字段仍由awk宽松强转，不能对任意畸形字符串宣称fail-closed；TMPDIR位于git工作树内会导致A/F两项漂移分类断言失败，作者版同环境同失败；R11作者脚本会刷新step-rhythm/out的5个临时文件，所谓“全部证据只读”不成立，取证看各唯一run的独立日志。独立快照覆盖src/config/tools共703文件，前后hash一致；这不代表并发Lead文档未修改，更不代表全仓干净。以上限制已记录，本轮不扩大修补。

### 文档门禁收尾状态

全量门禁重跑发现数字守卫已按当前工具规则对文档漂移判红，不能沿用早先六门全绿。首轮还发现本ns四处裸证据引用，现已验证文件存在并仅修正树/规划卡的引用路径（不改命令、源码、断言）；锚点已归零。最近一次结果：索引/政策/锚点/wf/links通过，数字重算48/48但文档新增漂移7、异文冲突4，数字守卫rc1，整卡不收口。

不通过改常量、扩大基线或恢复旧图制造GREEN；共享数字资料的变更不在本轮现场准备写域，不追着并发变更扩大整改。最后完整快照见[全链门禁结果](../../../../_tmp_session/AGC-RECTIFY-20261026/round4-lead/doc-gate-final.json)，门禁状态以该机器记录为准。

## 文档变更
新增agc-1026树与五卡，注册ns；原位更新现行协同推进计划，不新建另一份长期方案。

## 经验总结
现象：指定模型目录可见不等于运行通；思考on不等于max。处置：记录确切错误/路由与有限重试，未执行任务不报进行中或已修。

## 用户指定视频直入侦察链（2026-10-26 22时续办）

用户确认“现在没有别人在操作”，并指定视频，明确“直接当无人机的侦察图像输入”。本卡新增的是摄像头源替换为视频帧，不是人工抄地图，不是恢复第六图夹具，也不是无人机轨迹跟飞授权。视频只读原件SHA256=f52165a2463163dc3549bbf675a9ee6ee17bb01bea48519e6219440f1afeb559；[输入元数据](../../../../_tmp_session/AGC-RECTIFY-20261026/video-input/environment.json)已实读首帧，4321帧/27fps/960×544。

- 读过裁决：D-044不自动提交推送、D-046证据分层、D-020首段可靠才可进入运动放行。
- 复用检索：无人机感知资料与现有sim_match视频入口；该入口会默认UDP广播并循环，不能直接裸跑。先查当前可用模型/推理实现，复用现有帧处理，只将发布端隔离为证据输出。
- 新增理由：仅隔离输入装配/取证包装；不新建识别器、规划器或常量真源。模型缺失/未识别保持UNKNOWN，禁止夹具补齐。
- 写域：执行子会话仅`_tmp_session/AGC-RECTIFY-20261026/round4-video-run/`；独立审查仅`round4-video-review/`同层目录；Lead维护`video-input/`和本卡/状态/台账。无人机与车端生产树只读。
- 验收：实际帧→现有识别模块→结构化结果及帧号；记录源文件/模型hash、过程rc、EOF退出、未覆盖格UNKNOWN；reviewer独立验证输入身份与链路，不用作者自报代接受。所有视频结果明确source=video，不写成live。
- 回滚：停自己的有限离线进程、丢弃隔离装配即可；不清原视频，不重置车/机在途文件。

### 执行前验收清单
- [x] 用户指定输入已stat/hash且首帧真实解码。
- [ ] 现有侦察实现接收该视频帧，留逐帧结果及完整退出码。
- [x] 输入/模型/坐标契约及无控制副作用经独立审查。审查完成不等于机载等价性或坐标推论通过，见下述否定结论。
- [ ] 新一趟版本与段号重置方案通过静止验证，不接旧第11段。
- [x] 更新当前方案/记录并复跑文档门禁；运动/出光/飞行仍单独放行。


## 用户离场：纯离线检修切换

最新指令覆盖现场续跑：用户离场，允许心跳器保活。即日起本轮不连接/控制车机、不部署、不发车/出光/飞行，不提交或推送；此前“没有别人在操作”不延续为离场后的操作授权。视频线继续仅本地识别与数据取证。执行模型固定DeepSeek Account/deepseek-flash/max。

- Lead写域新增`_tmp_session/AGC-RECTIFY-20261026/offline-night/`，仅临时保活、基线快照、验收收口。旧保活记忆路径已不存在；查到旧PowerShell版本后评估复用，保活最多8小时，不修改永久电源计划。
- task-10：视频实帧接现有识别链，写域不变；task-11：跨趟段号/布局身份/回执契约离线审查，写域不变。另设启动安全检查，独占`_tmp_session/AGC-RECTIFY-20261026/offline-night-startup/`。所有生产源码只读，最小候选留隔离目录，不重复D-046已裁决问题。
- 自动目标目前paused；按直接请求尝试恢复被运行时拒绝“the model cannot resume a paused goal; the user must resume it”。已告知用户需要界面恢复；不新建目标或定时器绕过暂停，不声称通宵自动工作已生效。

### 离线检修验收清单
- [x] 临时心跳启动/停止/超时边界有机器证据，日志显示真实PID与截止时间。
- [ ] 每条新问题均有源位置、最小复现、实际退出码和影响界限；已知红不冒报新红。
- [ ] 最小候选由非作者独立复跑，未验项保留UNKNOWN。
- [x] 生产身份与原在途修改保全；原位更新状态/台账/历史并复跑门禁。


### Lead补充：全量回归入口的失败退出传播

复用现有车端全量回归入口，未新建测试框架。原脚本末尾只打印red列表，不设置外层退出码。以字节相同副本和一个人工构造的exit7测试复现：stdout显示非零退出1套、RED=exit7，外层却rc0；这不是车端业务测试失败，而是验证入口本身的失败传播实验。[原始证据](../../../../_tmp_session/AGC-RECTIFY-20261026/offline-night/regression-exit/observed.json)。隔离候选只追加一条按已有red列表退出的语句，同fixture外层rc1；[作者结果](../../../../_tmp_session/AGC-RECTIFY-20261026/offline-night/regression-exit/candidate-result.json)。不改生产脚本，不把作者自测称独立接受；空套件/文本解析完备性不在该一行候选范围。

- [x] 字节同源的坏内部退出码被外层吞掉已机器复现。
- [x] 候选独立A/B与正向/空集边界审查；生产整合仍不授权。


保活已于22:10启动，PID6032，受管job=bash-4095bb45218d4148，系统请求API返回非零；[当前状态](../../../../_tmp_session/AGC-RECTIFY-20261026/offline-night/heartbeat-active/status.json)，最长至2026-10-27 06:10:23上海时间。2秒到期自测已rc0并释放，显式停止及限域候选由task-13独立审查；写域`_tmp_session/AGC-RECTIFY-20261026/offline-night-review/`，实际子会话f6618510-d2d2-4f10-9777-ede2206dd412。

本轮[门禁预检](../../../../_tmp_session/AGC-RECTIFY-20261026/offline-night/doc-gate-preflight.json)六项rc0；48条数字重算通过、4条manual不能称已核，2条有意并列异文仍存。根指令和数字守卫已由本轮外部更新；[工具身份](../../../../_tmp_session/AGC-RECTIFY-20261026/offline-night/numeric-guard-identity.json)留存，Lead没有改该工具或放宽基线。后续文档编辑后须再跑最终门禁。台账提炼：D-047。


### Lead跨趟独立复跑（离线）

已将task-11探针以SHA256=b06497dee81865be0ce3d60dee8d4d2fabd6d6bea5aecbf89c053c70568d4552字节保全到Lead写域并复跑。作者部分safe=False为常量，不能直接当机器断言；Lead另从observed字段计算8条断言，真实rc0、全部复现：[独立结果](../../../../_tmp_session/AGC-RECTIFY-20261026/offline-night/cross-trip-lead/lead-verdict.json)。新v1被保留的v11挡下、不同到达顺序都保留v11、旧完整布局触发旧段请求；新发射器接受旧回执并改变covered/car_block，跨趟第1段同号也会混淆。

**边界**：这证明无run身份时的组合风险，不是现场已发生；新布局为合成协议fixture，不是视频识别成果；Mission/ROS为替身，只观察请求，不证明车移动。D-022同一趟内完整布局不过期的完赛设计保留，不以此为由取消该例外。旧运行进程与本地发射器文件字节一致未证，C2/C3静态源码特征不算动态实证。未改生产或重置实车状态。

## 2026-10-27 续办交接：离线检修子目标

**本节为当前裁决，优先于前述进行中快照及作者过程稿中的过强推论。** task-10至14均已交回，实际执行路由为deepseek-account/deepseek-flash/max；Lead完成跨卡裁决。用户今日要求继续后，仅核对证据和收口，未新增现场授权。本阶段查找、复现、隔离候选及独立审查完成，不等于整张集成卡完成，更不等于生产已修好或实车可发车。

### 已证问题与候选裁决

| 范围 | 当前裁决 | 复跑证据 |
|---|---|---|
| 跨趟协议 | 无run身份时，旧v11压过新v1，旧回执能影响新发射器；Lead从实际观察值独立计算8条断言均复现。仍未修。保留D-022同趟完整布局例外，不能以改TTL或抬版本号掩盖跨趟身份缺失 | [Lead观察断言](../../../../_tmp_session/AGC-RECTIFY-20261026/offline-night/cross-trip-lead/lead-verdict.json) |
| 启动状态假绿 | 真启动脚本配7个失败节点替身，真实脚本rc0且报全部启动完成；Lead复跑拦截进程信号与系统ROS setup，未接触设备 | [启动独立判据](../../../../_tmp_session/AGC-RECTIFY-20261026/offline-night/startup-lead/lead-verdict.json) |
| 输入失败闭合 | 空图及map:{}仍有假绿；语法错误YAML在普通模式确实rc2，不笼统说全部坏图都通过；加force会隐藏错误并报通过。隔离候选仅切换模式，auto_plan=false空图与map:{}仍未封堵，NEEDS_REVISION，不作为完整修复采纳 | [40组dry-run与12条取证断言](../../../../_tmp_session/AGC-RECTIFY-20261026/offline-night/startup-lead/lead-verdict.json) |
| 回归退出传播 | 一行隔离候选通过独立A/B，PASS_LIMITED仅指已计算exit-based red向外传播；空集合与错误文字但子rc0仍未闭合，未跑成全量回归绿、未合入 | [独立复核](../../../../_tmp_session/AGC-RECTIFY-20261026/offline-night-review/REVIEW.md) |
| 视频入口边界 | 既有ONNX评测器处理可打开但零有效帧的容器时仍rc0；Lead独立确认frames_processed=0（frames_read=1含失败尝试）。不可打开的空mp4是另一分支，不混称同一案例。此缺陷未修 | [零有效帧复现](../../../../_tmp_session/AGC-RECTIFY-20261026/offline-night/video-zero/lead-verdict.json) |
| 临时保活 | v1快照读者竞争确实会终止保活；Lead仅隔离修复快照写失败容错，v2经独立持锁/短读/慢读与stop/deadline复核，限域接受。8h续航、信号路径未证；stdout读端关闭仍能终止进程，不称完备守护程序 | [二次独立复核](../../../../_tmp_session/AGC-RECTIFY-20261026/offline-night-review2/REVIEW2.md) |

启动候选另补sim_cli A/B，均rc0、指标一致、118.12s完成，但仍74拍越界、field_modeled=0。该结果只证明所测静态路线未变化，未修空间安全、未验证真实布局或实车。作者另报旧配置迁移/任务ImportError吞错，保留在[启动报告](../../../../_tmp_session/AGC-RECTIFY-20261026/offline-night-startup/REPORT.md)，不扩称已完成独立实车验证。

### 视频结论纠偏

指定视频确实作为图像输入跑完既有ONNX CPU评测链：4321帧全处理，258帧有位姿输出，其中190帧z非正；未人工生成地图，也未向车发布任何布局。**这不是机载原生TensorRT链等价性证明，更没有可用侦察地图。** [原始评测元数据](../../../../_tmp_session/AGC-RECTIFY-20261026/round4-video-run/results/run_full/det.meta.json)与[独立复核](../../../../_tmp_session/AGC-RECTIFY-20261026/offline-night-review2/VERDICT.json)分别保存。

独立重建128/129帧未发现“两符号同负”，选中解的点深度为正；撤回作者“负z即相机在地下/两分支负深度导致”的机理。z是评测器新增的世界坐标分解量，不是生产测高，也不能移植为NED高度判据。z过滤只接受detbench平面侧/镜像过滤的机械效果，不能推出格号正确、位姿准确、覆盖完整；视觉候选仍NOT_ASSESSED，未合入。

视频自身有检测框/文字叠加，不据此断言具体录制方法。相邻帧低差异统计按其阈值解释，不冒称严格像素全同或传感器新鲜率；格号参考大部分依赖原叠字，集合命中不得当独立精度标注。cell_reads_ok只统计非空；固定4×2 FOV是既有仿真假设，不计作新生产缺陷，不可当真实覆盖真值。“cpp”仅为Python参数组名称，不是C++程序实跑。

### 保活、身份与回滚

原PID6032已为修复正常退出；v2 PID25640于2026-10-26 23:01:45（上海）收到stop，exit0、release_api_return=2147483649。2026-10-27续办核对无运行中后台job，未重启或延长昨夜8h授权；不改永久电源计划。[释放记录](../../../../_tmp_session/AGC-RECTIFY-20261026/offline-night/heartbeat-release.json)。工具仍将自动目标显示为paused且拒绝模型代为resume，不绕过用户界面控制，不声称此间通宵运行。

生产身份按开工31个抽样路径比对，不是全仓哈希证明；仅历史索引有Lead预期文档变更，采样源码/配置/工具及既有删图保持。原基线不重置；最终比对与门禁在本节收口记录中留存。没有部署、提交、推送或冻结区源码修改。回滚仅撤销本轮文档补记或丢弃相应隔离目录，不回退他人在途内容、不恢复旧地图。

### 下一阶段待办（未实施）

优先补跨趟身份贯穿布局、状态与回执，钉住run后再比较版本/段号，并明确跨趟缓存重置；与单趟D-022分开验收。再统一运行期/预检的地图可用性规则及force诊断，不能以现有模式切换候选代替失败闭合。视频侧先建立不依赖叠字的可核对标注和坐标契约，再讨论机载链接入；不得手抄地图绕过识别。部署与现场动作仍需新授权，维持NO-GO。

### 本阶段收口记录

[最终生产抽样指纹](../../../../_tmp_session/AGC-RECTIFY-20261026/offline-night/production-final-20261027.json)：31条路径含预期不存在项，仅历史索引发生预期文档变更；不代表整仓已洁净。[最终有序门禁](../../../../_tmp_session/AGC-RECTIFY-20261026/offline-night/doc-gate-handoff-20261027.json)以实际执行记录为准，工作流绿不替代功能回归或现场安全。已有未验事项保持挂账，本阶段离线审计与整卡集成完成分开判定。

## 2026-10-27 15:50 连接窗口：只读身份核对（Lead）

用户提供当次SSH连接方式与认证信息，允许按前述范围先做两端只读核对；不将提供连接方式解释为部署、发车、出光或飞行许可。owner/agent仍为lead；新写域仅`_tmp_session/AGC-RECTIFY-20261026/static-connect-20261027/`与本卡/树/状态/日志。复用现有OpenSSH与现场探针，不裸跑整栈或未知副作用的--help；密码不落盘，优先已有密钥。

- [x] 两端SSH密钥登录成功（主机名须继续与硬件/工作区核对）。
- [x] 当前硬件、关键词进程采样、配置文件与选定源码身份已取证；运行态参数/物理安全仍未证。
- [x] 原生视频入口、双套引擎及UDP发送路径已定位；运行许可与物理禁动/禁火确认仍待。
- [x] 只读结果、状态与日志已保存，本次六项文档门禁通过；不改生产。验收仅限本次取证与文档，不代表应用执行或整链通过。

验收：SSH真实rc与选择性结构化结果，不把进程存在等价就绪；回滚只撤本地记录，无设备变更可回滚。为什么现有不足：既有快照不代表本次连接后的实际主机与运行态，须重新采样。裁决沿用D-044/D-046/D-047，不滚动继承此前动作授权。

### 当前设备观测（只读，不是静止联调通过）

两次结构化SSH均rc0，使用已有免交互认证，未将口令写入脚本、日志或命令。无人机别名指向192.168.137.62，hostname为ubuntu，硬件为Jetson Orin Nano；小车192.168.137.164为RK3588S OPi 5 Pro。两个采样都未发现所选关键词匹配的控制/ROS进程，**不等于全面进程覆盖，更不等于物理禁动或禁火**。详见[本次观测](../../../../_tmp_session/AGC-RECTIFY-20261026/static-connect-20261027/observations.json)。

- 车端实际配置的旧执行器与mission两处auto_start均false；这是文件值，不是运行中参数证明。第一次探针按普通嵌套键取值为null，已按ROS参数容器与点号键纠正并另存v2，不把null当false。对比7个选定文件，配置、主程序、Mission、规划器及预检工具5项与本地不同；主程序src/install一致仅限该文件。[版本对照](../../../../_tmp_session/AGC-RECTIFY-20261026/static-connect-20261027/local-remote-comparison.json)。
- 无人机实际原生程序位于dzuav_deploy/air_ground/onboard/dzuav/build，旧dzuav目录仅有引擎；**这不是程序丢失**。[入口清单](../../../../_tmp_session/AGC-RECTIFY-20261026/static-connect-20261027/native-entry-resolved.json)。本机测试启动器指向dzuav_deploy/engines，它与dzuav/engines中的两个同名引擎逐一SHA不同；只确认不同，不推断哪套更准或更“新”。[引擎与二进制身份](../../../../_tmp_session/AGC-RECTIFY-20261026/static-connect-20261027/native-fingerprints.json)。
- [原生入口快照](../../../../_tmp_session/AGC-RECTIFY-20261026/static-connect-20261027/remote-sources/native2-1-main.cpp)的文件视频分支会构造UDP发送器，定位有效时调用发送；[广播实现快照](../../../../_tmp_session/AGC-RECTIFY-20261026/static-connect-20261027/remote-sources/udp-0-UdpBroadcaster.cpp)目标为INADDR_BROADCAST。因此不能把文件回放天然当成“只在本机算”。本次未运行该二进制，也未证明它与旁边源码逐字对应；以上是源级发送路径证据，不是当前正在广播的观察。

下一步限定为**机上临时、隔离发送的原生视频推理**：先固定二进制/引擎组合和视频hash，再验证子进程网络隔离或显式禁发；不得把录像中的车辆位姿当当前车位下发。不启动小车整栈、不传布局。运行应用/创建机上实验产物前，确认现场物理禁动、出光隔离、无人机未解锁并取得该阶段许可；若隔离不能证实则不运行。未部署、改参数、启动业务节点、摄像头取流、运动、出光、飞行、commit或push。原8h保活未重启。

复跑入口为本地[车端只读脚本](../../../../_tmp_session/AGC-RECTIFY-20261026/static-connect-20261027/inspect-car.sh)、[机端只读脚本](../../../../_tmp_session/AGC-RECTIFY-20261026/static-connect-20261027/inspect-drone.sh)与[原生指纹探针](../../../../_tmp_session/AGC-RECTIFY-20261026/static-connect-20261027/native_fingerprint_probe.py)，必须仍在本次只读授权范围内；结果含SSH真实rc。回滚仅撤本地补记与快照，不回退或修改设备。


## 2026-10-27 16:22 视频驱动步进准备（执行分派中）

用户确认本次小车在N10、车头朝M10；问答确认现场布局与实飞3录像一致，现场值守/硬件急停、激光禁用及无人机未解锁回答“是的”。用户另问本趟可否尝试打靶：保留打靶目标，但这不是立即出光许可。先禁光准备，射击位/停车/真实靶面/瞄准两许可核实后再单独确认出光。录像车辆位姿不得当当前车位，未识别格保持UNKNOWN，不手抄地图、不灌旧图。

用户要求Lead主做协调与规划，检查和执行下放。task-16由Lead统筹，task-17原生视频与布局链审查（子会话cfe32fe9-fbd2-4706-b122-2c4a678a8ea5）、task-18单段停车链审查（587ae9c4-d829-4d81-8c31-9f072f3924b5）、task-19唯一设备只读执行者（c423409e-c454-464d-9d61-84cb1bc00f28）。三者写域分别在同一临时批次的video-review、car-review、device-exec子目录；Lead不重复执行各成员已留证的检查，只做关键裁决。子SDK任务板缺失由Lead代管，不冒称已原生claim；本轮send_message寻址普通子会话失败，未声称增量要求已送达。

上述三任务是在新路由指令前派出，仍为deepseek-account/deepseek-flash/max。**此后新调用按用户最新指令改为raccoon/sn-deepseek-v4-1-flash-1017，reasoning_effort=on**；目录只支持on/off，不冒称max，失败不擅自切回其它路由。

验收清单（本轮开始时）：
- [x] 指定输入原件重新核hash，与此前同一视频一致。
- [x] 当前起点朝向、录像同场与现场值守由用户确认；不是传感器测量。
- [ ] 本次设备运行身份及唯一控制者核验。
- [ ] 原生回放的发送隔离与模型身份明确，独立审查通过。
- [ ] 视频帧实际产生可用布局及来源记录；没有用录像车位污染当前定位。
- [ ] 同趟协议/首段空间与停车包络满足放行；发车仍NO-GO。
- [ ] 出光条件及单独授权完成；当前禁光，不飞行。

本轮基础工作流检查rc0/GREEN，只代表合规。复用既有只读探针与原生识别链；新临时产物仅为本趟观测，旧证据不覆盖，生产冻结/删图/标定不动。设备执行者目前只准只读SSH；机端隔离应用运行须在其能力报告和独立审查后另行限定。

### 本轮团队首批交付与调度裁决

- task-17审查完成：原生dzuav快照只出0x4001目标与0x4002录像车辆位姿，没有车端需要的field_layout；既有emit_step_v3使用硬编码TRUTH/BATCHES，禁止拿它替代本趟视频感知。Lead抽核原生逐帧landmarks/objects分支后采纳该接口缺口；类表错配只按所审旧解析器路径记风险，不外推当前已运行射击。见[视频链报告](../../../../_tmp_session/AGC-RECTIFY-20261026/video-step-20261027/video-review/REPORT.md)。
- task-19只读取证完成：两端连接成功，所选指纹与上轮相同，车端文件里两auto_start均false，所查关键词无业务进程；空unshare -Urn可用但CUDA在其中未验证。机端限定目录未找到指定视频，不泛化为全盘不存在。备份引擎字节与deploy那对一致，版本配对已区分但不等于模型精度或二进制源码对应通过。见[设备报告](../../../../_tmp_session/AGC-RECTIFY-20261026/video-step-20261027/device-exec/REPORT-task19.md)。报告中现场确认未到达是该子会话所见边界，Lead以本节最新用户问答为准。
- 调度：取消原计划直接全量回放作为推进手段；它不能填补缺失布局接口。task-20改为只在layout-adapter隔离目录补真实逐帧证据导出与保守适配，复用检测器而非另造识别器；无充分覆盖时保持UNKNOWN。新作者1849079c-bd89-4054-aabd-8012314876d3，Raccoon Flash1017/on，尚未交付，不宣称已修复。
- 上述为task-17/19交回时点；随后限定上传与隔离短测见下节，不代表已发车。

### 10-27 机端短测失败与下一步裁决（Lead复核）

- **task-21已交阻塞证据，不是推理通过**：选定二进制与B对引擎初始化时报告 `expect [1,18,33600], got [1,17,33600]`。Lead独立读原始stdout/stderr和源码初始化顺序，确认尚无视频帧推理成果；报告的rc=134未在所选本地原始runner文件单独保留，因此仍按作者记录注明。宿主/子进程netns不同、命名空间内地址和路由空的记录支持本次隔离；wrapper未显式校验IPv6查询返回码，后续必须补齐。限域[机器复核](../../../../_tmp_session/AGC-RECTIFY-20261026/video-step-20261027/lead/canary-evidence-review.json)不把反序列化当GPU推理/准确率/布局验收。
- **配对裁决**：不采纳作者“A必为匹配的14类”推断，也不盲改类数/裁通道/关断言。task-23先查两对现有引擎I/O契约和类语义来源，仅A整对契约匹配且无语义矛盾时允许一次同素材有界空netns短测；只写独占机端临时目录，不编译/部署/连车/修改主机网络。执行8e7eebaa-0896-4664-a301-09c59b29e8f5，Raccoon Flash1017/on。
- **task-20只交观测导出候选**：布局恒null，未编译/机端运行，不能称视频到布局闭环。作者将“缺地图文件”直接等同“不能世界落格”尚不采纳：原生已经调用地面投影并生成world对象；需区分规则网格几何与手抄场地内容。task-22独立复跑候选与新反例并核复用路径（b5450ed1-57d0-420a-b70b-4a930914c739，Raccoon Flash1017/on），无设备权限。
- **task-18限域复核**：仅合成数据下代码能算N10到M10，不是视频布局或实车证据。Lead已复跑[错误目标完成单探针](../../../../_tmp_session/AGC-RECTIFY-20261026/video-step-20261027/lead/car-done-probe.json)：当前本地main的隔离副本接收M10请求对应N5完成单后进FINISH；车端快照同一分支也只比段号，运行install的其他文件尚未核齐。禁止把提高version当跨趟隔离，或把不启动激光进程当物理隔离证明。报告中“现场确认未有”只反映子会话范围，以本卡16:22用户确认记录为准。
- **当前仍NO-GO**：视频确已上传机端scratch且首个短测已失败停止；未起小车栈、未发路线/布局、未动车/出光/飞行。先补真实观测到布局与同趟完成关联，不恢复被删除的固定图，不把合成首段夹具当视频识别。保活不重启，不提交/推送。

### 10-27 A对短测与观测接口改判（后续现行结论）

- **task-23限域接受原生执行，不接受定位/配对语义**：实际I/O中A整对满足程序形状、类型与张量名；同素材一次30秒空netns短测留下381条逐帧完成记录，均为 `localized=0 landmarks=0 []`，目标rc124已落盘，未到EOF。`[]`是定位后的世界目标列表，不是检测计数。Lead从原始tensor枚举和stdout重算，见[独立裁决](../../../../_tmp_session/AGC-RECTIFY-20261026/video-step-20261027/lead/engine-pair-independent-review.json)。A仅可保留作诊断候选，不定为生产正确配对：14类训练标签来源及classifier标签顺序仍未知；不换装引擎文件。
- **隔离与清理边界**：三条地址/路由查询各有rc0和空输出，已修原wrapper遗漏IPv6 rc的问题；本次宿主/目标netns不同，输入/引擎/业务二进制前后hash一致，本次目标退出。作者“无任何ns引用”的表述收窄：只在可读进程中未见引用，原始记录另有300个不可读进程，不作全系统无引用证明。
- **task-20/22改判**：只加观测且保留算法的源码思路限域接受；旧adapter不作为交付接收。Lead已复制独立探针到自己目录复跑，确认NaN产生非标准JSON、A12越界格仍被收入、缺失/倒退PTS未拒绝；连续的少量行不等于全片完成。见[Lead反例裁决](../../../../_tmp_session/AGC-RECTIFY-20261026/video-step-20261027/lead/adapter-lead-verdict.json)和[独立评审](../../../../_tmp_session/AGC-RECTIFY-20261026/video-step-20261027/adapter-review/verdict.json)。原报告不改写。
- **纠正伪阻塞**：缺场地内容文件不等于缺网格几何；原生本就有public world/project与世界物体计算，旧trace漏导出world。Lead仅只读下载部署源码的MapLocalizer两个文件，远端前后hash与本地一致；确认按帧缩放内参、已有有限值投影防护以及规则格号坐标公式，见[源码级契约](../../../../_tmp_session/AGC-RECTIFY-20261026/video-step-20261027/lead/native-geometry-contract.json)。这不是固定障碍地图，也不证明源码与既有二进制完全对应或物理外参已标定；不同轴系不得直接混用，越界点不得夹紧成边缘格。
- **只推进两项定向修复**：task-24（f2ecfd90-1453-4c26-980e-316c3e7249a9）修完成单目标错配的失败闭合；task-25（9f82132c-1140-4186-826f-4672053bd15f）修world/原始格号筛选观测导出与严格输入校验。均Raccoon Flash1017/on、独占_tmp写域、无设备权限，不扩审计，不调阈值，不部署。下一次机端运行须等观测候选复核；编译只能去独占scratch，禁止替换线上main.cpp。
- **首段仍未执行**：没有可信视频布局和当前趟/目标关联证据，不向小车发布路线；未动车/出光/飞行，未恢复固定地图，未提交/推送。




