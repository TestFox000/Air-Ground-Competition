---
来源: 用户 2026-10-25 循迹替换交接说明
取证: 实/算/未
更新: 2026-10-25
取代: 无
status: in_progress
ns: traj-1025
agent: /root/speed
owner: /root/speed
reviewer: /root
树节点: TR-6
写域: dzrobot_v2/src/dzbrain/dzbrain/pursuit/controller.py, dzrobot_v2/src/dzbrain/dzbrain/pursuit/lookahead.py, dzrobot_v2/src/dzbrain/dzbrain/tools/sim.py, dzrobot_v2/src/dzbrain/tests/test_pursuit.py
---

# TRAJ-CONTROL-20261025 · 前馈反馈接线与现有验收对比

## 1 目标与台账
前馈反馈接线与现有验收对比。已读 D-008/D-012/D-023/D-027/D-028/D-034/D-041：冻结登记、保留 planner、旧红分类、阿克曼前提、卡树隔离、双远端分叉不得擅自强推。用户本轮授权替换循迹，但不授权部署。离线候选通过前不切现役默认。

## 2 复用检索
已查 doc_search「连续轨迹 阿克曼 循迹」、复用清单、能力索引、目录索引；源码搜索 trajectory/clothoid/dubins/quintic/jerk 无现成实现。复用 Path/Route/geometry/physics/route_check/现有 sim 与现有 tests 树。仿真不能证明深度相机实测 RMS。

## 3 新增轮子及理由
无重复仿真器；扩展现有 route_check 与控制入口。

## 4 写域
dzrobot_v2/src/dzbrain/dzbrain/pursuit/controller.py, dzrobot_v2/src/dzbrain/dzbrain/pursuit/lookahead.py, dzrobot_v2/src/dzbrain/dzbrain/tools/sim.py, dzrobot_v2/src/dzbrain/tests/test_pursuit.py；不得碰其余源码。

## 5 验收命令
根目录：python docs/_工具/gen_index.py --ns traj-1025；python docs/_工具/check_doc_policy.py；python docs/_工具/check_anchors.py；python docs/_工具/check_numbers.py。
dzrobot_v2：python tools/wf/check.py；python tools/wf/check_links.py；python -m pytest src/dzbrain/tests -q；现有 test_pursuit.py / test_corner_cut.py 逐脚本验收。无 pytest 不能算通过，另记原生 harness 实测。

## 6 回滚点
根仓 HEAD aa9b37341187f311c707d30288ff433c7851eea8；本轮原件在 `_tmp_session/traj-1025/before/`。只逆转本卡差异，禁止 reset/clean 或覆盖其他团队改动；两个嵌套仓分开核对。

## 7 执行记录
2026-10-25 唯一写域收窄为 controller.py / lookahead.py / tools/sim.py / test_pursuit.py。root已交写权，core/config.py仍由root单写。实现显式Trajectory候选控制与既有sim调用；不碰Tracker/mission/localization与实际默认值。

## 8 结论
待实现和独立复核。三层候选须验证地图后再接现役Mission进度轴，当前不宣称替换完成。

## 9 经验总结
现象：v(s=0)=0若直接按投影取速度会永远不出发。处置：时间参数化执行，并在停站等待任务许可后推进。

## 10 文档变更
新增本卡；修改ns任务树；归档无。

## 7.1 控制候选接线与首轮实证

- 只有显式 Trajectory 使用新分支；普通 Path 原公式和调用顺序保留，未动 Tracker/PIVOT/ALIGN/mission/default config。
- `compute_command(..., trajectory_state=TrajectoryControlState(path,cfg))` 使用曲率前馈、Frenet横偏/航向反馈；状态包含时间剖面、纵向进度反馈、加速度前馈、jerk限制和停点握手。reverse 的速度比例和舵角符号均实际参与，转向最后限幅。后续审查修正硬急停：速度立即给零，舵角仍经最终rate逐拍回正，最终形成零 Twist；仅不连续的纵向急停记为 override。
- `run_sim(..., trajectory=path)` 复用唯一现有自行车模型、命令延迟队列、舵机速率模型与场地观察器。新候选横向 RMS 相对连续参考轨迹；不是外部相机 RMS。提供已建状态时该对象被本趟消费。
- 原树 test_pursuit 当前 154 PASS / 0 FAIL，其中控制卡 §7e 11 条：首拍/无信号等价、无状态拒绝、横偏符号、最终rate、倒车两个系数、急停/重启、真实单直线、60 ms 延迟的1 m前进+cusp+1 m倒车。后者 11.56 s、终点 7.67 mm，速度范围 [-0.21,0.35]，命令 jerk 最大 1.0，硬cap覆盖0拍。
- 普通宽90°诊断仍失败：统一纵向控制修复将末点误差从 74 mm 降到 40 mm，但仍超过原 30 mm 停车门；没有放宽门限。第七图首轮仅释放1个停点后在第二停点停住；200/320s的停止驻留不能拿来冲淡RMS或宣称完赛。
- 首轮证据 `_tmp_session/traj-1025/control-map7-round1.json`；后续同一 `control_diag.py` 调用已有 run_sim，记录停点前 e_s/e_y/eψ 与四个源文件SHA256；该脚本仅过程诊断，不是第二仿真器。

## 9.1 本轮发现及处置
现象：参考时间抵达停点且投影 s 到位，但实车二维位置仍有横向偏差，停止握手会永久等待。机制：开环时间积分不等于实际曲线弧长；且长距离提前曲率与当前 Frenet 反馈不在同一相位。处置：已用通用纵向反馈补弧长；下一轮只改变前馈预览为“巡航速度×实际执行延迟”，不调增益/放宽停车门。reference_completed 仅表示参考时间结束，物理 completed 必须位置和速度同时达标。

## 7.2 延迟一致前馈与完整行程诊断

- 单变量将前馈距离改为 `min(曲率自适应前视, 配置巡航速度×已建模命令延迟)`。无延迟、零Frenet误差时输出当前参考曲率，新增真实曲线契约检验；普通 Path 未改。
- 固定旧几何哈希 `034a5a4f173d0396bf8cb69c7336eeba56c1c49dfdf9e0068eee15eeab94f425` 的同参数200s对照：RMS从0.070265m降为0.002651m，第二停点横偏从−0.077059m降为−0.006211m，已释放停点1→5；两次均未完赛，不把未完成RMS当全图通过。过程证据 `_tmp_session/traj-1025/control-before-delay-preview.json` 和 `_tmp_session/traj-1025/control-after-delay-preview.json`。
- 桥结构修正版几何哈希 `bf6e48c27125c679f3d89df67e8fcb7b4f9fc1a4d0bc91f83901f07c55a16e72` 的400s完整诊断仍失败：参考停点s=12.052735，投影误选重叠的出站支路s=12.342185，纵向反馈把正向速度压到0，无法进入holding；e_y约1mm。机制是停点尚未许可时投影搜索没有上界，下一项修复为真实搜索区间上界，不事后夹出虚假到站。
- 旧FieldObs的board_breach=627是旧设计净空（车半宽/转角外接圆）计数，board_overlap=0；不能替代用户新车身表面20mm指标。且旧FieldObs与新gate参考点偏置定义不同，真实车心雷覆盖必须由统一gate复算，旧7/8不是最终验收。
- 当前 `test_pursuit.py` 160 PASS / 0 FAIL；包括首末拍、倒车、停点、真实动态仿真、突变限速下最终舵速、真实黑板缺键/无效载荷无操作、候选动态巡航覆盖明确拒绝。候选速度时间剖面在构建时固定；传非空cruise_mps会抛清晰错误，要求重建状态，不静默忽略。
- 底盘读证：pursuitAngle回调独立计算TargetSpeed与TargetAngle，零速命令不依赖舵角；其速度平滑仍可能带来实际停车延迟，本卡没有在线车端验证。

## 7.3 授权运动区间与最终离线接线

- 候选真值/信念投影均传 `s_max=next_stop_s`，几何模块用真正截断的索引排除未授权出站支路；不以事后夹弧长伪造到站。停点释放后才打开下一段。
- 当前挡位从已授权区间的入站sample取得；参考时钟先到cusp而命令尚未实停时仍保持入站挡位。原树7e.16钉住该负例，避免非零速度提前反号。
- 实际终点判据统一用连续轨迹末端sample的参考位置；原格心仅作源路线信息，防雷终点车心/后轴偏置导致假失败。
- sim候选新指标 `trajectory_profile_direction_time_s` 已按实际反向比例折算；a_up/down是命令速度绝对值的增减，jerk仍用有符号实际命令加速度差分，最终steer速率统计排除首拍无历史。实际time_s包含route显式dwell_s，但shoot标记本身不自动加30s。
- `test_pursuit.py` 163 PASS / 0 FAIL。原 `test_sim.py` 的默认临时目录在本沙箱不可写；仅把tempfile.tempdir放本卡过程目录后原脚本能继续，仍因旧44航点常数及缺失config/map.yaml为红，与改前已有红对应；没有改其断言。
- 最终候选源冻结供root串行验收：controller SHA256 `66e33ff70fc952969221795749edb3ca345ef44010ed5fbcc1c87a8956f139e7`；lookahead `acb691c032d18b4ec29b8a8bce5649a7bf9922a39193ba3a1c6a5a7d0983bb5e`；sim `4da6bd1904aa031ad972a0c45cf8c091ca5bc568e68e812984bf9996270e37ee`。
- 根据用户最新边界，本轮只交离线候选；Tracker/localization/mission仍由t-car占用，PIVOT/ALIGN与现役调用链未集成、未删除。最终两图命令与物理验收由baseline唯一执行，避免并发导致搜索预算假红。

## 7.4 统一最终两图结果：几何闭环成立，动力学与时间预算失败

baseline以同参数、同1mm审计、同20000拍预算串行执行现有route_check的两图仿真；全过程源哈希未变。权威产物：`_tmp_session/TRAJ-VERIFY-20261025/final-two-map/final-summary.json`，每图audit.json与既有sim生成CSV/SVG同目录保存。

| 图 | completed | 相对自身参考RMS | 真车心雷覆盖 | 精确区域违规 | profile时间 | 反向比例后参考时间 | 实际行驶时间 | 含射击预算 | 失败项 |
|---|---|---|---|---|---|---|---|---|---|
| map7 | true | .003771m | 8/8 | 0 | 96.098s | 134.716s | 147.160s | 207.160s | override18拍、命令jerk6.264924>1、实际横向加速度.355658>.35、预算>180s |
| map_field | true | .018451m | 8/8 | 0 | 109.154s | 151.472s | 163.020s | 193.020s | override4拍、命令减速度3.357754>.7、jerk166.887683>1、实际横向加速度.353443>.35、预算>180s |

- 只读定位：在线MotionState只对当前cap做卸载，纵向进度反馈/前馈可把命令状态推到未来cap不可达集；最终硬cap仍必须保留，但其覆盖会破坏a/j。离线profile自身满足a/j不能推导实际闭环命令满足a/j。
- 第七图尖峰集中在s约9.89、10.25、倒车12.58附近进入.308506m/s曲率上限（倒车乘.6）。另一图最大尖峰在t2.48s，执行速度.16322→.09606，对应s=.48m处解析转向速率cap从.35降到.092298m/s。没有放宽cap或移除override计数。
- map_field还存在实际起始朝向与参考轨迹不一致：sim默认起始+y、轨迹首切线+x，相差90°；时间轴先走而实际投影初段停留，进度反馈加速放大了不一致。不能改仿真初始朝向来洗绿，下一轮应将真实起始姿态作为轨迹输入约束。
- 统一后续方向（尚未实现）：未来cap的jerk制动可达过滤；最终转向rate域与基于实际命令速度的横向加速度可行域求交，并显式处理空交。还须在原延迟/执行模型上检验actual约束，不能只看命令侧。
- 当前判定为**未通过离线总验收**，无部署、无集成、无删现役FSM。两图均已完成不等于全部验收成立。

## 7.5 第二轮约束修正（先短场景，后统一两图）

- 独立步骤一：最终舵角取物理幅度域、`[上一拍舵角±rate×dt]` 与 `±atan(L*a_lat/v_cmd²)` 的交集，求交后不再追加一个可能顶破lat的rate裁剪。空交保留物理/rate域内最接近零的输出，并显式 `steering_constraint_infeasible` / `constraint_override`；不硬裁速度伪造jerk合格。原树7e.17/18可行与不可行负例通过，阶段证据 `control-lateral-step.json`。
- 独立步骤二：SpeedEnvelope按已采样几何cap和当前v/a，预测下一拍加速度加上随后最大合法jerk制动的可达速度；每个原空间cell检查cap，求可行下一拍加速度上界，给MotionState使用。保留当前hardcap最后校验；已有状态来不及制动则future_envelope_infeasible并入override计数，不能冒充可行。
- 原树7d.8/9以相同.35→.10m/s局部cap、60ms延迟预算对照：无过滤a=12.414/j=620.891/override1；过滤a=.4792/j=1.00000000000003/override0、cap超限0。已不可达初始状态明确返回false。全过程不修改物理上限、增益、停点或地图专用参数。
- 统一初始姿态规则提取为现有sim中的 `resolve_initial_yaw(route_or_path,sim,trajectory=None)`，CLI编译与run_sim共同调用；旧分支语义保留，轨迹编译接真实起始yaw，不改模拟摆位。
- 当前原树167/167；第二轮源hash及短场景数据在 `_tmp_session/traj-1025/velocity-reachability-step.json`。此时仍等待统一两图动态验收，短场景通过不等于整体通过。
