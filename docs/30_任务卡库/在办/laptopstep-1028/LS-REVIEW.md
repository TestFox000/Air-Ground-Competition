---
卡号: LS-REVIEW
ns: laptopstep-1028
agent: stop-review
owner: stop-review
reviewer: lead
树节点: LS-3
写域: _tmp_session/laptop_step_control/review/；本卡
kind: review
status: in_progress
创建: 2026-10-28
更新: 2026-10-28
来源: 本轮用户请求；dzrobot_v2/docs/决策台账.md
取证: 实
摘要: 独立安全评审及故障注入；本地实验，不将离线结果冒充实车通过。
---

# LS-REVIEW · 独立安全评审及故障注入

## 读过的台账条目
D-039布局构造器唯一真源；D-051运行面硬接管优先；D-055 ttl=0不是失联停车；D-057是历史另一趟，不作本轮授权。

## 复用检索结果
docs入口及空地链路检索；复用清单→能力索引→目录索引，复用dzbrain.scout.layout.build_layout、io.contract.TOPICS、StepLoop及Tracker；不重建规划器或控制器。现有supervisor不是透明双向桥，旧发射器会自动推进，不满足人工逐段及锁存停车。

## 新增轮子及理由
本卡新增仅review目录内独立反例、loopback/fake ROS探针和证据包装器；不新增规划器/识别器/坐标常量，不改作者实现与测试期望。复用作者公开API及现役build_layout/normalize。无新依赖、无生产配置键。

## 写域
_tmp_session/laptop_step_control/review/；本卡。禁止改冻结生产源码、默认配置及他人文件；禁止commit/push。除Lead外禁止设备连接、部署及运动发布。

## 验收命令
工作目录=v1；已独立复跑并留原始退出码、输出与快照；局部通过不等于实车通过。

~~~bash
python -B -X utf8 -m unittest discover -s _tmp_session/laptop_step_control/tests -v
~~~

## 验收清单（领卡初始全false）
- [ ] 单段正确性及真实回执匹配（真实ROS完成回执/物理停稳尚未验证；本地原字段合成帧验证已通过）
- [x] 重复、旧回执、超时、断链与重连不续跑（限定本地纯类/fake ROS/loopback子进程，不冒充车端失联制动）
- [x] 独立reviewer复跑并记录退出码（最终作者110项；独立通信/停车28项与独立视频10项分组exit0，原输出与哈希见下）
- [ ] 冻结区增量0及门禁结果明确（本卡前后冻结区diff摘要相同；全局门禁由Lead统一运行，尚不宣称收口绿）
- [x] 实车未证项与残余风险明确（来源功能窄验收通过；最终真实源NO_GO且观测为空，未发车）

## 回滚点
原生产零改动；停止本轮新进程，保留证据，仅移除本卡新增实验文件。共享登记按本轮增量外科逆转，不reset、不覆盖他人。设备写入另需部署清单/哈希和回滚记录。

## 执行记录
2026-10-28 Lead建卡。基础check.py GREEN/exit0；根仓和车仓均有大量既有改动，全部保留。用户已确认本轮N10朝M10、清场及有人保障；软件验收未完成前不发车。

stop-review 已按 list/get → revision=1 claim task-7（现revision=2）。恢复D-039/D-051/D-055及接管、TTL源码证据；运行前独立固定反例见[acceptance_cases.json](../../../../_tmp_session/laptop_step_control/review/acceptance_cases.json)。该文件为本卡新增实验验收清单，不是规划器/仿真器或生产构造器；后续review侧黑盒探针仅调用作者公开边界与fake IO，不修改作者实现/测试期望。所有运行与现场验收初始仍false，未SSH、未发布真实ROS。

## 独立复跑阶段交付（不等于运动放行）

- 通信/停车冻结阶段命令：`python -B -X utf8 _tmp_session/laptop_step_control/review/run_suites.py --author`。当时作者97项exit0（protocol 11 / guard 22 / transport 24 / video诊断40），独立28项exit0（纯guard 16 / serve离线进程9 / fake ROS发布失败3）。[总报告与完整源/断言哈希](../../../../_tmp_session/laptop_step_control/review/1793193933003062400_suites.json)、[作者原输出](../../../../_tmp_session/laptop_step_control/review/1793193933003062400_author.log)、[独立原输出](../../../../_tmp_session/laptop_step_control/review/1793193933003062400_independent.log)。候选与测试前后身份稳定；冻结目录原有git diff摘要前后均为391efae0ee34ae1f59708586adac345cc830b04b82613eb90d2043d1d862b94b，不能据此把既有脏树称为干净。
- 独立探针：[纯类](../../../../_tmp_session/laptop_step_control/review/test_independent_guard.py)、[loopback独立serve/attach进程](../../../../_tmp_session/laptop_step_control/review/test_independent_process.py)、[发布/反压故障注入](../../../../_tmp_session/laptop_step_control/review/test_independent_effects.py)。含无EOF黑洞、半行JSON、EOF、被杀attach后serve存活、stdout真实反压、第二socket不接管、override竞争/查询失败的fake IO注入、发布异常、旧GID完全重新握手仍拒恢复。子进程全部归fixture回收；未SSH/真实ROS。
- 用户指定视频由既有Python312环境独立新跑8帧，原CLI返回2=NO_GO；frame0至7与原PTS0至259.259ms对应，视频SHA f52165a2463163dc3549bbf675a9ee6ee17bb01bea48519e6219440f1afeb559与Lead W120记录一致。[短窗原结果及哈希](../../../../_tmp_session/laptop_step_control/review/1793193777769497400_video_report.json)。包装器exit0只表示诊断契约复现；`video_to_layout_feature_pass=false`，旧video_source固定NO_GO不能算功能完成。此后LS-VIDEO已转step-contract续作，新候选待独立正负例；[预置视频反例](../../../../_tmp_session/laptop_step_control/review/video_acceptance_cases.json)，不要求虚构安全证书。
- 独立短段仿真及机器断言exit0：completed=true、3.94s、truth/belief越界拍0；field_modeled=0且数值bounds margin=0.6，**不是场地/桥洞净空或物理制动通过**。[机器报告](../../../../_tmp_session/laptop_step_control/review/1793193849571897400_sim_report.json)。最初DSH bundled Python缺PyYAML而exit2，后改用主机已存在Python312/PyYAML6.0.3，未安装依赖。
- 只读复核Lead设备回执：[四文件对账](../../../../_tmp_session/laptop_step_control/lead/verified_runtime_manifest.json)与本地protocol33e0dba6/guarde582b1f4/transportea9a7075/car_bridge4e10a5fa一致；[静态双向探针](../../../../_tmp_session/laptop_step_control/lead/static_probe_result.json)为5次challenge/lease、最终LATCHED、car_instance=null、sync/permit/layout均未发；[停自有服务回执](../../../../_tmp_session/laptop_step_control/lead/static_service_stopped.json)stop_rc0、MainPID0、inactive。真实manual1被动收到8条、manual发布方独占只证明静态软件话题链；无brain GID与Tracker/底盘消费证据。review本人未连设备。
- 阶段裁决：**仅窄范围接受候选本地停车/通道边界及锁停静态联调证据；运动NO_GO**。完成单是软件完成而非物理停稳。待证：视频→可用观测正例/本轮有效布局，首段真实brain GID/跨话题release与layout到达顺序，故障接管被真实StepLoop/Tracker消费，硬急停链、滑行距离及停稳录像；正常多段逐次人工放行。监护自身/ROS/底盘失效仍为残余风险。

首轮纯类红由review自造夹具缺生产必填bridges造成，补入明确synthetic K2后重跑，不修改作者实现/测试期望，不把K2当现场地图。[原失败报告](../../../../_tmp_session/laptop_step_control/review/1793192959024342400_suites.json)保留。

## 本轮最终独立评审交卡（停止于NO_GO）

- [最终结构化报告](../../../../_tmp_session/laptop_step_control/review/final_review.json)。task-7评审交付完成不等于实现/运动/整场验收通过；卡仍按在办规范保留in_progress，正式归档及全局红由Lead处理，不修改他ns或baseline。
- 最终source哈希3e090903efccee1e0238623ae31543ca0193481bd0abe9d18f5126929f1cff05，测试cb494b45e10824035fb4669ae82f4f787c428c1d77669e5ab944ac88f09ee03a。[独立最终复跑](../../../../_tmp_session/laptop_step_control/review/1793195966758132300_suites.json)：作者110/110（源53）exit0，review原10视频断言10/10 exit0，前后快照稳定。正例真调laptop→唯一build_layout→normalize；无H、黑mask内部像素、半格、错域/帧/PTS、奇异/非有限H、类别冲突、缺桥、零原solver地标/投票、同帧H数值字段不一致均核对。旧d5c候选两条红保留在[原失败证据](../../../../_tmp_session/laptop_step_control/review/1793195402970872200_suites.json)，不改review断言换绿。
- [最终完整实窗原件](../../../../_tmp_session/laptop_step_control/lead/final_video_1793196044323982300/full_result.json)由Lead用新run LSVIDEO-FINAL-1793196044323982300采集，review只读字节SHA d7f3dc0256852ebd3c9d186206578d9a782b350aa4c8a8895cc9b6986a7e02a7匹配，candidate与本地3e090903匹配。不是恢复/冒充旧d5c窗。frame1345–1352，8 localized/8 H；known为F10,F9,G10,G9,H10,H11,H9,I9,J10,J11,J9,K10,K9，**N10/M10均不在known**。当前source已NO_GO且observation=null，accepted_bridges=null；raw K10/K11箭头投影不能当有效桥。
- 以完整原件的raw-derived数据重新喂现适配器（无新识别），得到同NO_GO/同known/同reasons；[最终静态核对](../../../../_tmp_session/laptop_step_control/review/1793196395732969600_window_gate.json)exit0只表示拒绝复现。因为源先拒，**未调用laptop builder、未构造layout、未尝试permit**，不得写成已实调空观测后的首段拒绝。
- 原件reason名虽为object_world_pixel_cell_conflict:F11/K11，但静态审计不得把名字冒充已证物理矛盾：[geometry](../../../../dzrobot_v2/src/dzbrain/dzbrain/core/geometry.py)以0.175格心域为下界，[原pnp](../../../../_tmp_session/vtest-1025/detbench/pnp.py)meters_to_cell默认margin0.10，[适配器](../../../../_tmp_session/laptop_step_control/video_source.py)object_matches_plane用margin0。F11 x=.0962/K11 x=.1698均与原逆H半量化区间相交、margin0拒而margin0.10接纳；已证的是**当前适配器拒绝及边缘域口径差异**，不是已证像素↔world数值不符；全局锚定仍未验证。遵Lead要求不再改代码/放宽边界/推理。N10/M10双缺独立维持NO_GO。
- 冷启动不再称源码缺第三bootstrap：现役独立DroneTask listener可回空phase/无version/空block car_state，guard已有真实GID+fresh_start人工sync路径，纯类正负例已验；**真实冷启动现场链尚未执行**，不是要求新实现。
- 所有实车未测项继续false：真实brain GID/完成回执/跨话题顺序、接管被StepLoop/Tracker消费、底盘零速与停稳/滑行量、独立硬急停链、后段人工permit执行、视频全局锚定。done不等于停稳、TTL/租约不等于硬急停；监护自身/ROS/底盘故障残余保留。本轮不再盲试，不部署新源、不发车。

## 文档变更
本卡及本ns树（树由Lead单写）；正式收口由Lead统一索引。

## 经验总结
现象：无人机不可达不必阻断笔记本模拟输入。处置：保留真实控制闭环，只在实验链路适配；done不等于停稳、心跳TTL不等于停车。
