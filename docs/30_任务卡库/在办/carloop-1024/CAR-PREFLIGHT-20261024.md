---
来源: 用户 2026-10-24 指令「现在准备跑车」；CAR-DUAL/CAR-HOME/CAR-GATE
取证: 现场只读与本地候选
更新: 2026-10-24
status: in_progress
ns: carloop-1024
agent: /root
树节点: CL-4
写域: docs/30_任务卡库/在办/carloop-1024/；_tmp_session/carloop-1024/；dzrobot_v2/docs/TEAM_STATUS.md；docs/目录索引.json；docs/目录索引.md
摘要: 准备跑车，先车端只读版本/节点/路线/权限核对；本地形成差异与回滚方案，部署/发车/出光须另行确认。
---

# CAR-PREFLIGHT-20261024 · 跑车准备

owner: /root；reviewer: /root/gimbal_audit；历史/工具复核: /root/history_audit。

## 1 依据与复用

- 台账 D-001（不整包替换自瞄）、D-007/D-019（人工部署、唯一 deploy_to_car 入口）、D-012（tour）、D-036（preflight 边界）；已读 AGENTS、复用清单、能力索引、目录索引及部署资料篇。
- 复用 `deploy_to_car.py probe`、`deploy_car.py` 只读拉取/对账、现有 ROS 查询/采集与规划器；不创建第二部署器，不调用会连接无人机的空地联合 preflight。
- 新增轮子：无。过程区只容纳远端只读快照、候选差异与验收输出；不直接修改车端源文件或默认配置。
- 先查历史：离线35/177/58/136/26测试已通过，不重复从零做；需补的是车端身份/实效、完整ROS构建与硬件行为，旧图单段或两发闭环不等于本图全流程。

## 2 验收命令与边界

- 首次只读：代码仓 `python -B tools/deploy_to_car.py probe --ip 192.168.0.4`。
- 版本核对：通过既有只读 pull 获取真实 src/install/config/start_all 后本地 SHA256 与 diff；读取运行进程/ROS域/节点/话题发布者与运行参数，不发任何控制话题。
- 本地候选：以车端真实代码为基线只移植本轮改动，保留车端额外补丁；复用既有测试，产出精确清单和回滚方式后再请求部署确认。
- 不启动/重启整栈、不连接无人机、不自动发车、不出光。若运行中需停火/停机，先向操作者报告并确认。

## 3 回滚点

只读阶段车端零写入。候选只在本 ns 过程目录；不覆盖现有源码/他人文件。真实部署备份与回滚点在审批前补齐。

## 4 执行记录

- 14:14 probe exit0：主机 duzhong、地址192.168.0.4、运行1天4小时、Python3.10.12、剩余磁盘13G。src/dzbrain存在，生产robot.yaml最后修改10-23 21:29。probe 的colcon --version无输出不能当工具链已验证。
- 本机 duzhong.local 的 hosts 旧绑定、dzrobot SSH别名强制密码已有他人卡诊断；复用当前有效直连地址，不修改本机配置。记录根 `_tmp_session/carloop-1024/` 的 prep-probe.log。
- 新发现代码仓决策台账已被其他会话修改，本卡不碰。

## 5 现场核对（10-24 14:17～14:29，只读）

- 实际图与用户图片一致：N10→N5、D7/E7靶、K10/K4桥；F5是雷格。车端配置仍是 tracker-safe 自动规划、允许 fallback，手写50点含 F9*/F5*；两处 auto_start 均 false。不能把旧路线直接拿来发车。
- 无 dzbrain、定位或裁判进程；现存云台 PID 545330 的启动参数明确覆盖 `use_external_fire_gate=false`、`max_shots=3`、`shot_cooldown_ms=2000`。配置文件中的 true/10 不等于运行实效；尚未读到裁判板重置后的当前计数。
- ROS域118、LOCALHOST_ONLY=0下，延长发现到5秒仍只见 det_node 和静态TF；关键权限/控制/裁判话题均不可见。`ss`进一步证实相机276472、底盘507373、云台545330的UDP地址仍是旧192.168.137.79，当前wlan0为192.168.0.4；这解释了进程存在却DDS发现失败，不可当作节点可用。
- PID记录并不完整：底盘PID文件空但串口ttyACM0由507373占用；相机PID记录103255过期，真实父子进程276470/276472；云台PID/PGID和记录均545330。不能直接用“PID文件缺失则启动”的脚本，以免产生第二写者。裁判串口ttyACM1当前无打开者。
- 车端planner/mission/aim的src与install SHA256逐项一致，但不同于本地工作树。Mission等于本地HEAD；Planner尚未包含本地B1任务目标改版；Aim仍无stop_on_hit开关；C++保留10-19的lead进控制环、平滑跟踪及停止锁存补丁。因此不做整包覆盖。
- 证据位于本卡过程目录：prep-pull.json、prep-runtime.log、prep-ros-discovery.log、prep-processes.log、prep-node-ownership.log。首次ROS查询的CRLF导致source失败已单独识别并改用LF字节输入重做，不把那次失败当现场状态。

## 6 本地候选与复用验证

- 以本轮车端只读快照为基线，候选位于 `_tmp_session/carloop-1024/prep-candidate/` 。C++三方合并只在停扫分支有一处冲突，保留车端 scan_homing=false 和 stop_latched=true，再将停止分支提到TRACK短路之前；完整保留lead/EKF控制律。增加双钥匙各500ms租期与过期撤销。
- 复用生产函数编译夹具：原26项全部通过，补ROS Time.nanoseconds()模拟接口并增加4项车端停止锁存兼容回归，30/30退出0。没有复制生产决策算法到测试；尚未完整ROS构建。
- 复用AG-TEAM-20261023/E的R3部署候选，原187项离线测试本轮实跑187/187退出0；审计hook禁止真实网络/子进程。主工具和E包原件不变。候选四文件窄范围预演为4份源码加2棵install备份，不含生产robot.yaml/map.yaml；正式执行必须带本地/车端双向SHA256前置条件。
- 现役deploy_to_car仍无窄范围、单独push不备份；不能用它的固定全量清单覆盖车端。R3候选属于已复验的过程区副本，未合入主入口，更未部署。
- 禁火不能用 max_shots=0：车端构造函数会改回10；Python预算0也超出有效范围。laser_switch只控制常亮，且裁判节点另有referee_cmd串口入口。话题隔离未找到历史实车验证，因此不把未经实测的隔离方案当作已证明禁火。
- Python窄移植保留车端旧接口，不顺带升级B1任务目标体系。本图legacy专项32/32；自动规划车端基线169/177→候选177/177；云台任务55/58→58/58。车端Aim未修改，现行本地套件134/136，两个失败是车端没有stop_on_hit=false行为，不能承诺打满10发。
- 完整新双站套件在旧接口候选上为16/20，4项因objective参数不存在提前失败，不记全绿；复用原harness的verify_car_candidate.py过程验收适配到真实legacy接口，补证本图32项。该脚本仅在本卡过程区，不新增部署/规划能力；现有新接口测试不能直接覆盖旧接口是其必要理由。所有diff、日志及复跑入口在prep-python-test目录。
- 当前图候选62点F9*/C5*双站合法，但折线足迹诊断有靶板侵入40mm@E8、挡板20mm@F6。两轮只改一个变量的现有接口试验均未消除：tracker-safe无解；tour提高barrier_turn_penalty至5仍同一路线。因此停止调参，不能发车。旧50点路线足迹无侵入告警，但F5*射击站非法，不能回退它。
- 将本地候选 footprint_gate 改error后，真实车端main --dry-run仍exit0：该入口只validate_route；Mission的tour分支仍把footprint当顾问，不能靠这个开关阻断。候选配置明确拒用且不入部署范围；没有改车端默认值或扩写新门禁。
- 最终四文件部署清单、双端SHA256与回滚argv保存在过程目录prep-deployment.json；真实合并候选的dry-run exit0（prep-deploy-dry-run.log），只预演备份→四文件替换→两包构建，没有真实执行。预定备份点为 /home/duzhong/_backups/carloop_20261024_143105；恢复范围含原源码与两个install包，不含配置或节点启动。

## 7 当前放行边界

用户随后明确回复「确认，尽快跑车」，批准上一轮明确申请的四文件备份、部署与两包编译；本次不启节点、不发车、不出光，拒用的路线配置不部署。执行前重新检查四文件候选SHA、备份目录尚不存在、无dzbrain/colcon进程，保存配置指纹用于部署后对照。代码候选通过离线测试不等于车可发；完整跑车还需恢复DDS链路、检查真实权限单写者、验证当前图路径与禁火闭环。

## 8 文档检查与收口状态

本轮索引只重扫本ns，外部125行逐项不变；gen_index/anchors/numbers退出0，48项可重算数字一致。doc_policy退出1：既有R2位置问题，加上并行lead会话两张DRONE卡尚未登记；wf/links退出1：现剩一条既有SSH-DIAG归档索引断链。上述其他会话文件均未接管。两仓diff --check均0；本卡不关闭，不提交/推送。日志为本卡过程目录prep-gate-*与prep-gates.json。

## 9 已批准部署与后续准备

- 14:39～14:40按已批准manifest真实执行：双向SHA256核验8/8；备份96文件回读通过；四份源文件上传对账4/4；两包colcon构建退出0，dzbrain 2.92s、云台44.7s、总45.1s。仅包命名和Boost弃用提示，无编译错误。R3工具在既有工作区内备份目录补COLCON_IGNORE，防colcon重名。
- 复核安装态：Planner/Mission的install与候选哈希一致；Aim仍原车750542b2…；新云台ELF为84c0c200…并含externalFireAllowed符号。robot.yaml/map.yaml部署前后逐字节一致。备份点仍为 /home/duzhong/_backups/carloop_20261024_143105；尚未执行恢复演练。
- 当前PID545330仍运行旧ELF 544fa4cb…，参数仍外门禁false/max_shots3。没有启动/停止/重启任何节点，不能把“安装完成”说成“运行已生效”。证据：deploy-execution.log、deploy-result.json、deploy-verify.log、deploy-verified-hashes.json。
- 路线新证据：旧50点不含C5，任一旧线段也不经过C5，最近0.70m；全部合法候选中旧线只经过F9，单纯改星号不能满足双站。上一轮tour参数试验继续停止。
- 新的独立假设只复用旧路线现成的直进倒出模式，不调planner参数：移除F5星，在旧C1→C3→D3中增加C4→C5*→R:C4→R:C3，其余几何不变。由integration_review仅本地检验一次；须双站、独立路线门禁、足迹error及前后向转换全部证实，未经通过不部署/发车。输出仅进本卡过程目录route-reuse。
- 54点支线候选：静态双站/route_check/validate/严格足迹均过，但相同车端配置快照下的本地仿真不通过。原50→新54：C3→D3最大横偏0.086715→0.345317m，全局0.228285→0.432409m；压雷均4/8、障碍净空不足160→198拍，K列封死均2拍。碰撞/挡板/靶板/越界均0不代表完整路线通过。新候选拒绝发车，未部署；证据original50-vs-candidate54.json。本轮不继续扫参。
- 用户确认车在N10朝M10，随后单独批准恢复相机/底盘/定位/云台并做禁火静止检查，不启动循迹、不发车，云台可转动。执行前核对旧PID/PGID与命令；精确TERM旧gimbal545330、chassis507371组、camera276470组，确认退出后按既有启动命令恢复。新组PID分别：chassis620777、camera620788、loc620792、gimbal620811。复用原命令及PID管理，不调用会因失效PID产生第二写者的旧重启脚本。
- 实际运行云台已显式use_external_fire_gate=true，并将LaserShot_Command重映射到/carloop/inhibited/LaserShot_Command；运行参数true/10读回成功，隔离话题1发布者0订阅者。裁判、Mission均未启动。DDS118已恢复camera/det/chassis/localization/gimbal及反馈链；det原有实例未动。
- 定位原始(1.5,0.5)是格单位，按既有入口×0.35即N10=(0.525,0.175)m；相对yaw入口加π/2，符合用户N10朝M10。无证据需要reset，未发reset。单帧速度为0，不据此宣称已完成位姿稳定性测试。
- 复用fire_rec临时内存适配增加云台命令/扫描/隔离激光话题，未上传新脚本。静止采样最终IDLE，目标(pitch,yaw)=(2262,2047)，反馈(2260,2042)，残差2/5 tick；未出现射击命令1。单次scan=1后SCAN自行结束，stop=0时已在IDLE，故本轮未覆盖真实TRACK→0，不冒称该硬件回归通过。记录static-stop0与static-scan-home的JSONL/summary；原控制算法未改。
- 用户随后要求「能直接发车吗，不检查了」：停止追加检查，当前采集已正常结束且finally发送scan=0。保持云台IDLE、激光输出隔离、无Mission/裁判，无发车。本轮后续不运行新增测试；完整路线仍有上述已知阻塞。
- **后续用户明确改为「跑一遍验证成果然后再采集数据」**：按此授权执行一趟54点F9/C5全路线现场试跑，巡航0.10m/s、禁火、保留运行急停，不再做离线筛选。承认该路线已有仿真失败，试跑不等于先验通过。默认robot.yaml/map.yaml未改。
- 新临时配置文件不在R3发布清单，push被范围校验拒绝，车端无新增配置文件；未改工具白名单或手工scp。改用本次进程私有memfd传给原main的--config，保持原启动入口及内置门禁。首启动在DDS尚未发现端点时exit6；随后在相同ROS上下文先等待4秒发现，再运行原main，必需odom/pursuit双向端点均1、启动契约通过。没有使用DZ_SKIP_CONTRACT或--force。
- 实际Mission PID622358，54点/19.25m、延时3秒自动出发；真实运行已从N10向M10、经K10向H8区。激光始终隔离，裁判未启动，无外部数据录制器；按用户顺序在失败停住后取回原生日志。
- **本趟失败停在首个倒车区**：段7、s=2.800，终末belief约(1.604,2.380)m，日志格H7；车头约8°，DRIVE速度0。直接停因是P_MOTION_LIMIT drift_stop=0，不是Safety闩锁，安全状态仍OK。倒车v=-0.080期间定位x由1.026增加至1.559；方向守卫据此拒绝推进，随后偏差扩大至约0.39m触发停车。
- **实际车端serial_reader源码已取回**：odom_callback以hypot(dx,dy)得到恒正step，再沿map_yaw累加dr_x/dr_y，未使用倒车符号。它与实际定位在倒车时仍正向增加的日志一致；不能通过关闭方向守卫解决。另有Tracker漏洞：固定同一pose两拍，入口仅发1拍负速度就因投影地板推进宣布“已切倒车段”，不能证明已反向移动2cm。这是本趟新增具体问题，非“倒车后物理上不能前进”。
- 已下发manual_override=1并在日志确认遥控接管刹车，然后对本趟Mission精确SIGTERM收尾；未重启整栈、未再次发车。运行证据位于run-observed目录（mission.log、实际tracker.py/localization.py/serial_reader.py及serial-reader.log），停车收尾见run-stop.log。未修改这些生产模块。

### 9.1 首趟故障的最小本地候选

- 用户追问「倒车了就不能再前进了吗」；已解释直接停因和两个软件缺陷，先修本地候选，未再次发车。写域继续限于本卡过程目录，不修改车端或冻结生产模块。
- 只读回取实际底盘 `dzactuator.cpp`：先更新编码器yaw，再用同一yaw和有符号轮程积分位置，发布同帧四元数。故原始位移点乘同帧原始车头方向可以恢复符号，地图方向仍由原IMU链计算。没有用控制命令推断实际前后运动。
- 历史复核未找到已有符号修补；现有 `test_localization` 倒车用例直接输入正确递减的FakePose，坐标脚本也不调用serial_reader，不能覆盖本次上游bug。复用既有FakeClock/harness，过程区验证脚本直接执行实际serial_reader回调；新增理由仅为补这一缺口，不新增定位算法或通用测试框架。
- `reverse-fix/src/car_localization/car_localization/serial_reader.py` 以车端实际安装快照为基线，保留R3重置位置/航向补丁，先用非负距离过抖动门限，再取原始odom前后符号。版本标记改为v4-dr-reset-signed。针对前进→倒车→再前进、原始坐标系与IMU不同、静止及reset的17条断言，原车6/17、候选17/17（exit0）。复跑：`python -B -X utf8 _tmp_session/carloop-1024/reverse-fix/verify_serial_reader.py _tmp_session/carloop-1024/reverse-fix/src/car_localization/car_localization/serial_reader.py`；反例将最后路径换成`run-observed/serial_reader.py`。日志和diff同在reverse-fix。
- 回滚：尚未上车，生产源码零变更；保留run-observed原始快照。部署前仍需确认，且当前R3发布白名单未包含serial_reader，不得伪装路径或手工scp绕过。
- Tracker候选移除倒车入口提前抬地板，以bridge.pose原始里程反向≥2cm且切入倒车段确认完成，保留3s超时、2s冷却、两次失败停车。聚焦原车3/14→候选14/14；既有Tracker132/136→131/136，其中4条ALIGN限速旧失败不变，新增1条是旧断言要求静止五拍即成功，与修复目标相反，未篡改旧测试凑绿；路径重叠30/33→30/33，3条旧图预期失败不变。复跑入口reverse-fix/tracker-regression.py与tracker-run-existing.py，精确diff/日志/哈希同目录。
- 用户「继续」后准备最小两文件部署包，仍未上车。复用R3工具过程副本，仅在显式--only时允许精确serial_reader路径，默认全量清单不变、目录扩大仍拒绝；未改冻结的正式部署器。原187项加6项发布范围/备份验证，193/193退出0，全程审计hook禁真实网络与子进程；两文件backup→push→两包build预演exit0。部署清单与预留回滚点见reverse-fix/deployment.json。
- 发布对账发现定位src与install不同：src 1575cac4…未带R3重置补丁，运行install d02821b7…带补丁。候选以运行install为准完整保留R3，部署前置哈希针对实际src，备份同时含源码和install，不把两者错误当成同版。

### 9.2 第二趟禁火实跑（用户确认归位并批准部署重跑）

- 用户明确回复「车已归位，允许部署并重跑」。通过上述R3窄发布执行两份源码及两包构建：备份100文件回读通过，源文件哈希2/2一致，car_localization 2.16s、dzbrain 2.40s，总2.85s，exit0。回滚点 `/home/duzhong/_backups/carloop_reverse_20261024_153942`，备份同时保留src/install原有差异。安装态两文件与候选哈希一致。
- 仅重启定位（旧组620792→新组626901），其他节点保持；新Mission PID626921。仍复用同一进程私有配置、54点/19.25m、0.10m/s、3秒延迟、禁火和DDS预发现4秒；启动契约必需端点通过。新定位日志已出现v4-dr-reset-signed。
- 实车日志证实负速度时位姿x从1.042降至1.014，倒车入口53拍后确认真实反向并切段，继续倒退至H10附近后恢复正速度0.10。第一趟的恒正里程及1拍假完成已在本次运行中消除。
- F9完成APPROACH→STOP，按配置停稳0.40s后搜索；6.10s未锁到目标，按原策略放弃并STOP→DRIVE，发scan_request=0，遥测SCAN→IDLE。禁火，无真实射击。不把IDLE读数等同于本趟云台位置反馈到中心已验收。
- 本趟仍未完成全程：重复H10/H9先被后续索引提前判到，定位投影自愈6→10→8，行驶时共享进度长期显示段6/s约2.2～2.8；F9静止时又恢复段11/s4.40。日志有E9停滞12.0s强制跳过。说明有进度/到点判据问题，不是“倒车后不能前进”。完整日志确认F9实际停站，已纠正只看尾部日志时“跳过射击站”的不准确表述。
- 主agent发现进度异常后对精确Mission PID626921发送SIGTERM，运行100.7s结束；收尾报告刹车已尝试、八任务全部STOPPED。最后任务遥测A8附近，未到C5/N5。该次终止由agent执行，不是drift_stop：本趟漂移限停0拍、Safety锁存False。未发第三趟车。
- 证据：reverse-fix/deployment-execution.log、deployment-result.json、run2-launch.log、run2-mission.log、run2-localization.log、run2-stop.log。后续只读区分控制投影与遥测/Localization双写问题，尚未改第三处生产模块。
- 只读复核结论：不能把遥测旧段解释成车辆卡死，车实际持续推进到A8附近；Tracker和Localization同写共享进度且数值不一致已证，控制影响仍待证。H10/H9未来重复点提前登记已证；日志另有H8被停滞兜底跳过，故也不能认定整条路线正确。后续应核对Mission到点资格和两层投影状态契约，不能仅关显示或关方向守卫。

## 10 文档变更

新增本卡；修改本ns任务树、TEAM_STATUS及本ns索引；后续预检结果继续追加同卡。归档：无。
