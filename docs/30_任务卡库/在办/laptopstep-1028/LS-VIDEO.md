---
卡号: LS-VIDEO
ns: laptopstep-1028
agent: step-contract
owner: step-contract
reviewer: stop-review
树节点: LS-5
写域: _tmp_session/laptop_step_control/video_source.py；_tmp_session/laptop_step_control/tests/test_video_source.py
kind: implementation
status: in_progress
创建: 2026-10-28
更新: 2026-10-28
来源: 本轮用户请求；dzrobot_v2/docs/决策台账.md
取证: 实
摘要: 复用既有视频识别链生成有来源的局部观测；缺失落格不造地图。
---

# LS-VIDEO · 视频识别来源适配

## 读过的台账条目
D-039唯一build_layout；D-051硬接管优先；D-055布局TTL不等于停车；D-057旧趟不作本轮行动口径。

## 复用检索结果
docs入口、空地链路/复用清单/能力索引/目录索引；现有ONNX视频评测入口与观测导出，复用build_layout和geometry，不造识别器/坐标公式/规划器。读完现役入口后在本卡追加。

## 新增轮子及理由
唯一视频来源适配与对应边界/功能测试已续作实现；仅捕获原_lattice_solve返回H并新增完整格覆盖生产者，不复制识别器、定位算法、布局构造器或坐标公式。旧回放记录不作本轮新识别；CLI无ROS/网络，无新增依赖。候选观测可用不等于真实映射可靠，更不等于可发车。

## 当前用户确认（2026-10-28本轮）
用户已选择“沿用，现场布局与视频一致”。指定输入=D:/User/Desktop/无人机视频/测试素材/实飞3.mp4。glob已核文件存在；允许本轮真实读取与识别，不使用旧det.jsonl充作本轮识别。模型候选=_tmp_session/vtest-1025/detbench/models/hbb_d13.onnx 与 cell_v4d.onnx（先核存在/依赖）。旧det.meta中4321帧曾耗1539秒；先做有时间盒短窗，不再无意义重跑整片30分钟。

## 与laptop-link的冻结入口
laptop.py提供build_from_observation(observation,*,version,segment_id,car_block,target_cell,ts)。observation={map:{start,end,mines,obstacles,targets,bridges,barrier_col_from,barrier_row_hi,barrier_row_lo},known:[实观测格],source:{video,sha256,...}}。内部使用TruthMap.from_map+权威build_layout，显式target不自动全图known。视频yield该对象，CLI支持每次permit前重读--observation。子会话无法直接send_message给团队，Lead将读你的消息并转达；不要因为不能私信停工。

## 写域
仅[video_source.py](../../../../_tmp_session/laptop_step_control/video_source.py)、[test_video_source.py](../../../../_tmp_session/laptop_step_control/tests/test_video_source.py)及本卡正文；树仅Lead。冻结区及他人测试禁止写，不SSH/设备/ROS/部署/commit/push；guard/protocol和接线四候选哈希保持原冻结。

## 验收命令
工作目录v1：python -B -X utf8 -m unittest discover -s _tmp_session/laptop_step_control/tests -p test_video_source.py -v。预期exit0，stop-review独立复跑。真实短窗另留source/哈希/帧数/落格有效数与NO_GO原因，不许只以rc0当图可用。

## Lead复核纠偏（功能完成≠诊断完成）

2026-10-28 Lead读候选发现iter_observations/summarize均无条件NO_GO、usable_layout_frames固定0。这只能交付诊断入口，不能标“视频→布局”已实现。用户已明确录像与当前布局一致，recorded_video_not_live是来源说明，不应独立作拒发布理由。known应基于可信完整可视覆盖，不要求不存在于旧协议的certified证书。现役run_onnx保留H_world供对象落格、cls_sem在纸目标投影后按K列arrow/其它mine；请区分无法定位的实际窗NO_GO，与任何输入都不能生成布局的功能缺口。先给具体最短可实施缺口，勿宣称全流程完成。



### step-contract 续作候选（2026-10-28，本轮唯一8帧新窗）

- [唯一源实现](../../../../_tmp_session/laptop_step_control/video_source.py)不再无条件NO_GO。原_lattice_solve只运行一次，保留其H及原质量字段，源adapter捕获同帧元数据，不改原算法/模型阈值。H坐标域明确为undistorted_frame_pixels；CapturedLocalizer先调用原undistort，再复用其同一_map_xy对全255 mask以同INTER_LINEAR和CONSTANT0 remap，仅值255像素有效，不把场上真实黑色内容判成坏像素。
- 几何覆盖复用geo.block_to_meters和BLOCK_SIZE取四角；奇异/非有限H、horizon分母换号/近零、退化投影排除。必须四角均在图内，且整个投影格多边形的raster均在真实有效像素mask中，才形成known。单OCR点、启发FOV格心、P0固定起点均不能直接填known。
- paper按原cls_sem（K列arrow/其它mine）及原H世界落格累积；未知/未投影物体触及格保守排除。已有正类别不会因后帧漏检而删除；同格正类别冲突锁存NO_GO。桥洞仍须真实可落格观测，缺桥只保留known诊断、observation=None；不填K5/K7旧图。显式fixed_start=N10/fixed_end=N5仅引用[P0固定层](../agc-1026/P0-FIXED-LAYER.md)已签事实，不冒充新识别起终。
- [作者测试](../../../../_tmp_session/laptop_step_control/tests/test_video_source.py)保留原40项，新增11项，实际运行51 tests OK/exit0。功能正例以明确synthetic H/有效mask/纸桥生成非空observation，并调用laptop.build_from_observation→权威build_layout成功；这些不是实片结果。新增负例含半格、黑边mask、奇异H、horizon、错坐标域/帧/PTS/尺寸、缺mask、类别冲突及未投影物体。独立review另验，不自打勾。
- 冻结SHA256：video_source.py = d5c7fe6629ad4104d23f9aeb6317563ce4edd0c8a7f036bf1d7236fbcfecf3c3；test_video_source.py = a56c88314473814674475daf8e13e85e7c7bbd1f6591af70ebec6aa0f5663370。guard仍e582b1f4…、protocol仍33e0dba6…，未改冻结候选。

#### 唯一真实新窗口结果：首段NO_GO

本轮命令（内存stdout，不写第三份产物）：

~~~bash
python -B -X utf8 _tmp_session/laptop_step_control/tests/test_video_source.py --real-window 1345 8
~~~

- job bash-bb78d0c644cf4f96已完成并collect，进程exit0仅说明执行成功。run_id=LSVIDEO-REAL-1345-1793195111559300900；视频SHA256=f52165a2463163dc3549bbf675a9ee6ee17bb01bea48519e6219440f1afeb559（9553824 bytes）。原frame 1345–1352；原PTS 49814.81481481481–50074.074074074066 ms，27fps源；8读/8处理/0丢，8 localized/8 H捕获。默认conf0.25、nms0.45、cell_conf0.45、undistort=true、pose_mode=lattice均不改。
- 原H假说几何覆盖F至K列9–11行，排除物体遮挡/投影不明区域后的known精确13格：F10、F9、G10、G9、H10、H11、H9、I9、J10、J11、J9、K10、K9。**N10不在known，M10不在known**，不得补起点覆盖。
- 源功能层可产OBSERVATION_READY schema候选：map.start=N10/end=N5来自P0；H候选bridges=[K10]、mines=[J10]，obstacles/targets空；但paper另落F11、K11或场外null，和Lead亲看K至N列画面存在明显全局锚定疑点，不能把候选K10桥当现场已证桥。无跨帧正类冲突不等于锚定正确。
- **首段行动结论NO_GO**：missing_current_N10 / missing_target_M10 / world_anchor_semantics_unverified。laptop既有current/target双known门保持；非空summary.observation不是发车放行，不能把本组候选送车或用P0起点替覆盖。未部署、未连设备、未发ROS或运动命令，不扩阈值、不另开窗口、不全片重算。
- 本轮实片证据完整stdout由工具结果留存，Lead负责在lead写域捕获/索引；本卡不新建第二视频源或第三份数据文件。整库门禁由Lead统一，实际停车与实车未证。

### 独立review两项边界反例后的窄修候选

原d5c7候选独立源10例8绿2红，证据由reviewer保存；红因不是阈值调优：H存在但plane.n_landmarks/votes为0仍能READY；对象world/cell与同帧H下u/v不一致未拒。已仅在适配边界修正，未改review断言、原模型/算法/阈值，未追加任何视频推理。

- plane自身原solver接受元数据须满足keep对应n_landmarks>=6、votes>=3、有限非负reproj且median<=max；不要求row.n_landmarks等于plane.n_landmarks（二者本就不同口径）。
- 同帧H逆投影对象原u/v必须与world/cell/cell_proj自洽。原导出u/v保留0.1 px、world保留0.0001 m，因此用两者半量化区间而不是新调px阈值；horizon/跨格歧义/矛盾均禁observation。
- 新冻结SHA256：video_source.py = 3e090903efccee1e0238623ae31543ca0193481bd0abe9d18f5126929f1cff05；test_video_source.py = cb494b45e10824035fb4669ae82f4f787c428c1d77669e5ab944ac88f09ee03a。作者原40项保留，新增13项，共53 tests OK/exit0，待reviewer独立复跑。
- **证据留存限制如实更正**：真实唯一8帧执行的是旧d5c7候选；job首次流式stdout被读取后，仅console.inspect的summary+逐帧摘录被返回会话，嵌套[Object]折叠，未保留完整H/mask/raw字段。没有工具返回的spill path；二次job_output为空。现有证据为会话ed142c2b-6bb5-41f4-8d2f-76483a88ea46事件1231，只是派生摘录，不是完整原始JSON。已向Lead补发明确标注derived_machine_excerpt_not_original_stdout的机器JSON，未保留字段显式说明；不得声称新hash已经对该窗raw重放或有完整原始日志。不为补证而重开推理。
- **冷启动接口有现役静态复用路径，实际未验**：原run_live的_start_layout_listener/_wait_for_field_layout会在建立完整StepLoop前启动现有DroneTask接收布局，收到首layout后再建立任务组；不是“冷启动缺实现”，也不需第三套listener。fresh_start_confirmed只解决操作者确认当前车位/基线，不生成视频free格。本片缺N10/M10覆盖且有锚定疑点、最终新run另触发对象自洽冲突，故没有可用于该现役路径的合法首段数据；不能借P0起点/空ready绕过双known门。真实listener/首permit/运动闭环尚未现场验证，本任务未部署或发车；全局门禁/现场动作由Lead统一。

### 最终独立窄验收与交接

stop-review对最终3e090903/cb494b45复跑同10项源反例均exit0，两条原红的断言未改；作者整套110项（其中源53项）亦由其独立复跑exit0。已亲读[结构化原始汇总](../../../../_tmp_session/laptop_step_control/review/1793195966758132300_suites.json)：local_pass=true、snapshot_stable=true、hardware_go=false；原命令/exit与源码前后哈希俱在。仅接受本地源功能、同域畸变mask与输入自洽边界，不接受真实世界全局锚定、桥位或实车。

Lead另授权由其对同1345起8帧新run一次取证，先完整JSON落lead写域再打印摘要，review只读该原件并验证首段拒绝；这是新run，不是旧丢失stdout重建、不称算法改进。本作者未抢写或再次推理。LS-VIDEO窄实现交付完成，候选继续冻结；全局门禁/真实图取证/是否发车由Lead收口。初始清单不在本作者手中冒充实片通过。

### Lead最终新run原件（不是旧d5c摘录）

已只读核验[完整原件](../../../../_tmp_session/laptop_step_control/lead/final_video_1793196044323982300/full_result.json)，SHA256=d7f3dc0256852ebd3c9d186206578d9a782b350aa4c8a8895cc9b6986a7e02a7；run_id=LSVIDEO-FINAL-1793196044323982300，候选3e090903冻结且hardware_go=false。这是Lead新执行并先落完整JSON的单独run，不是旧stdout重建；本作者未重跑。原frame1345–1352/原PTS49814.81481481481–50074.074074074066ms，8/8 localized；13格known仍缺N10/M10。最终源decision=NO_GO、observation=null，明确原因object_world_pixel_cell_conflict:F11和object_world_pixel_cell_conflict:K11；不能再拿旧d5c的OBSERVATION_READY当最终源结果。适配功能已窄完成、独立10反例已过，但本轮真实首段数据不可用，未发车。

## 初始验收清单
- [ ] 复用现役识别入口与无网络边界
- [ ] 本轮真实读取视频并产出来源证据
- [ ] 严格world/地标/时戳/有限数/已知格验证
- [ ] 无法可靠落格则明确NO_GO而非补图
- [ ] 独立review复跑

## 回滚点
仅停本轮视频进程、保留证据，移除两份实验新增文件；原生产不动，禁reset和覆盖他人。

## 文档变更
本卡与本ns树；Lead统一登记索引。

## 执行记录

Lead已启动本轮真实ONNX 120帧短窗（run_id LS-VIDEO-20261028-W120），调用既有run_onnx.py、原算法阈值、threads=2，结果将落 _tmp_session/laptop_step_control/lead/video_window/det.jsonl 与 det.meta.json（计划产物〔未实现〕）。这是本轮新识别，不是旧det；视频子线无需重复推理同120帧，可在产出后用它联调parser。CLI仍应可独立接受视频。输出脚本 _tmp_session/laptop_step_control/lead/run_video_window.py 已落。
Lead建卡；本轮视频实际短窗识别已授权，无需再等待确认。两文件独占范围允许CLI向stdout输出JSONL；Lead负责捕获到lead写域留证，你不写第三份产物。保持原识别算法与阈值，不用旧det作为新识别。known的语义是可信视频覆盖/已观测，不是绝对安全证明；有有效地标标定及完整格可视的覆盖可以形成局部观测，不应要求不存在于旧协议的“安全证书”。单个OCR点不等同整格可视，不能只由点生成known；如确无完整格覆盖/地图特征语义，必须明确NO_GO并提供最短具体缺口，勿为图绿调阈值。请现在实施并自测，Lead统一读取stdout真实短窗执行。
