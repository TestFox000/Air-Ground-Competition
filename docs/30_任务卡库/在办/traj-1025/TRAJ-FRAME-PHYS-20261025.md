---
来源: 用户2026-10-25全仓坐标统一指令
更新: 2026-10-25
status: in_progress
ns: traj-1025
agent: /root/baseline
owner: /root/baseline
reviewer: /root
树节点: TR-8
写域: _tmp_session/traj-1025/frame-migration/dzrobot_v2/ 下 隔离副本的 route/physics.py、tools/route_check.py、tools/_field_const.py、tests/test_physics.py；root追加授权 tests/test_safety.py、test_telemetry.py、test_referee.py（后者未改）
---

# TRAJ-FRAME-PHYS-20261025 · 物理区域和验收工具迁移

## 1 台账与目标
已读D-004/D-023/D-027/D-041。用户明确翻案D-004的坐标命名：+X=N10→M10（西），+Y=N10→N11（南），N9为北即−Y；起步yaw=0°、正舵左转不变。新证据为用户本轮明确指定；影响面为坐标转换/初态/物理区域/消费者；不迁移会使相同摆位出现90°错相。其余比赛与规划规则不重开。

## 2 复用与新增理由
复用唯一core.geometry和现有physics/测试树，不另建坐标真源。正交变换为 x_new=y_old，y_new=11×0.35−x_old，yaw_new=yaw_old−π/2，det=+1；不可只交换xy。隔离副本源哈希在frame-migration/source-manifest.json。

## 3 写域与保护
隔离副本的 route/physics.py、tools/route_check.py、tools/_field_const.py、tests/test_physics.py；仅这些文件。所有源码修改仅在隔离副本；原共享工作区未交还的运行链不写、不部署。与其他成员按文件分工，禁止整份覆盖他人新改。

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

## 6.1 隔离坐标物理迁移实测（2026-10-25）

仅修改隔离副本的四个授权文件。`barrier_segment()` 为唯一挡板几何真源，返回当前帧 `(x0,y0,x1,y1)`；默认端点约 `(0,2.1,3.85,2.1)`。点/段/车体净空均消费 `segment=`；旧三元组入口和旧轴关键词明确抛 `LEGACY_AXIS_BARRIER_API`，不静默猜帧。桥轴迁为 X，横偏为 Y；墙端点、场地范围、障碍和旋转矩形保持刚体等价，物理阈值未改。

CLI 记录 core 的 `MAP_FRAME_ID`、geometry/planner/physics/trajectory/CLI/config/map 哈希；旧帧或未声明帧的 baseline 不合表。规划失败和未知实现异常均保留逐图 JSON 状态与独立 audit 文件，异常不会丢掉批次分母；已有文件拒绝覆盖。成功/失败均在以地图文件名及SHA前8位命名的独立子目录保存审计JSON。

验证（均在隔离 dzrobot_v2 根；PowerShell 设 PYTHONUTF8=1、PYTHONIOENCODING=utf-8，TEMP/TMP 指向同级 test-temp）：

- `python src/dzbrain/tests/test_physics.py --trajectory-only --random-manifest ../random64/manifest.json`：**exit 0，42/42**。证据 `_tmp_session/traj-1025/frame-migration/physics-frame-final-focused.log`。包括 200 姿态刚体不变（四角/挡板/障碍误差≤1e-12）、500 姿态宽相位与完整几何对拍、154 格工具与 core 一致、原短仿真 CSV/SVG、64 个故障注入均落文件且保分母、重复输出不覆盖。
- 原完整 `python src/dzbrain/tests/test_physics.py`：**exit 1**，在原 `config/map.yaml`〔已删〕缺失处停止；此前断言全 PASS。未恢复地图、未松门限。证据 `_tmp_session/traj-1025/frame-migration/physics-frame-full.log`。
- 为覆盖被原缺失夹具挡住的后续物理断言，直接加载同一文件调用 `test_ref_offset_footprint()`、`finish()`：**exit 0，19/19**，证据 `_tmp_session/traj-1025/frame-migration/physics-frame-footprint.log`。原记录刚体转换后仍为挡板余量 0.0544m、侵入 0.000243m、F4 到靶板间隙 0.035m/侵入 0.04m，未重钉数值。
- 首次系统 TEMP 不可写导致 PermissionError；只改运行环境 TEMP/TMP 指向本卡隔离过程目录后重跑，不修改测试跳过。中间原件保留在 `_tmp_session/traj-1025/frame-migration/physics-frame-focused.log`。

源码冻结清单 `_tmp_session/traj-1025/frame-migration/physics-frame-ready-source-hashes.json`：physics=e43cb54cf984c79c0ab31e8f58e0514d05fd4628e4d50fd790ce8c9d04b8765d；route_check=1266661f1c52a41be2947c2316b45ed201c94cacba9c5e28e9e781e72e53494d；_field_const=aac78301da3d73ad1f068a48e189f90f22a4bef5dd03d4d0cfadecd0d153117d；test_physics=9a77cc8c5c36544eb926b3f9ce05b3156a9825f41fdd3379bfb15f5b3454dcbd。

只读审计留下的旧缺陷：桥洞斜切告警旧公式沿行轴取角，本轮仅按刚体变换保持其数值，没有顺带改变该历史角判据；连续轨迹 hard 桥轴对正仍为 X 轴、整车进入开口期间 `sin(body_yaw)=0`。旧折线近似与新精确矩形口径继续分别报告。

## 6.2 用户新增64张随机图要求与生成准入

用户本轮明确要求按地图规则随机 **64 张新图**并压力测试，取代本轮原来的两图演示范围；不是拿历史 64 图结果冒充本次。先查既有入口：`dzsmartscout/tools/sim_airground/random_maps.py` 只变桥洞与靶朝向，不满足四类变化；`dzrobot_v2/_scratch_auto_plan/maplab/gen_maps.py` 含 BFS/射击停点可解性预筛并重选 seed，不适合本轮无算法筛选口径。原 64 图来源是既有宿主 `stressbench.generateMap`；按原函数复用，未重写 Python 生成器。

实际复用的现役文件为 `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/src/stressbench.js`，sha256=f1b511ce20ebe70691a331ee952f463d53975b9fed53444a37f8685218cbe96f；依赖同目录 simbench.js，sha256=70443e63051e4bc869aa1eb5f93db536938f5aa255d9955b3d2d5297bb52224f。Node 直接 import 并调用 `generateMap({seed:20261025,index:i})`，i=0..63。原函数缺省读取已删 map；一次性进程输入适配将**仅该 readFileSync 路径**转到已存在 `_tmp_session/vtest-1025/truth/map7.yaml`，获得固定起终点 N10/N5 与原宿主无人机 N8/M2 fallback；不恢复配置文件、不改宿主源码、不调用 planner。适配脚本是本次复跑胶水，非第二生成器：`_tmp_session/traj-1025/frame-migration/generate64-adapter.mjs`。

输出清单 `_tmp_session/traj-1025/frame-migration/random64/manifest.json`（sha256=bad675d2f3fbecef94cb0b96f0c97d8755697bb6d18d22090db67ed0f6ca0bff），路径清单 `_tmp_session/traj-1025/frame-migration/random64/map-paths.txt`。实际 seed **20261025..20261088**，每图原生成器 attempt=1，无重掷，**64/64 唯一布局**；二次调用结果一致，生成前后宿主源码哈希不变。四类独立布局数：雷64、障64、靶56、桥洞28。

准入只看：154格合法名称；雷8/障8/靶2/洞2；四类与 N10/N5/N8/M2 互斥；非洞元素不落K列；桥洞K列上1..6、下7..11各1；靶两格横/竖相邻且不跨6/7隔板；固定挡板D..N、起终点不变；文件与manifest哈希一致。**没有 BFS、规划成功、轨迹可编译、完赛或安全结果筛选**，不可解/不可跟随图必须照实进入64分母。42项既有树验收包含独立读取全部64文件的以上准入。

生成复跑（输出目录必须未存在，原产物不覆盖）：

```powershell
& 'C:/Users/TestFox/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/bin/node.exe' '_tmp_session/traj-1025/frame-migration/generate64-adapter.mjs' 'D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/src/stressbench.js' '_tmp_session/traj-1025/frame-migration/dzrobot_v2' '_tmp_session/vtest-1025/truth/map7.yaml' '_tmp_session/traj-1025/frame-migration/random64-replay'
```

既有 CLI 正式批次命令（由 root 唯一发起；此处仅复核命令，不提前启动）：

```powershell
cd D:/User/Desktop/比赛存档/v1/_tmp_session/traj-1025/frame-migration/dzrobot_v2
$env:PYTHONUTF8='1'
$env:PYTHONIOENCODING='utf-8'
$trajBatchArgs=@('tools/route_check.py','--trajectory','--config','config/robot.yaml','--sample-step','0.001','--max-nodes','18000','--simulate','--sim-max-steps','20000','--json','--output-dir','../random64-results')
foreach($mapPath in Get-Content -LiteralPath '../random64/map-paths.txt') { $trajBatchArgs += @('--map',$mapPath) }
python @trajBatchArgs *> '../random64-cli.log'
```

截至本卡本段：只证生成与规则准入，不代表64图规划/静态/动态通过；完整压测由 root 收齐。旧坐标两图报告不能作为新帧通过证据。门禁/全套件/真实外测仍由根任务收口；本卡未归档、未提交。

## 6.3 批次期间只读验收审计

本段不改任何运行源码，不重启压测。CLI 顶层 `ok` 是静态与动态联合；`STATIC GEOMETRY PASS` 只是静态标签。聚合不得把未生成候选时的13个 false 解释为13项真实超限：阶段未到则指标未测；两臂任一未完赛，RMS只作观测且比较为不可评估；候选已完赛但时间超180s需保留真实完赛和超时两项。

已发现后续需明确的口径覆盖：当前记录/硬验初始 body yaw=0，未单设首个非零运动方向验收；已完成候选的 CSV 初始可辨位移均为 +X，但这不是“整个首段直到M10保持直线”的证明。参考轨迹整车进洞期间沿轴直线已静态硬验；实际洞内轴向偏差的 gate rejection 单列、没有自定测量容差，不冒充精确车体碰撞。

**输入落格合法 ≠ 固定初始姿态满足全部物理约束。** 独立只读遍历64份已固定输入，用现有 physics 对 N10 后轴、yaw=0、ref_offset=+0.075 的精确矩形复算：index **17、29、46、55、59**（seed 20261042、20261054、20261071、20261080、20261084）在 M10 放障碍，起步时柱面净空仅 **0.0075m < 0.02m**。解析式：M10柱心x=0.525 − 柱半径0.1275 − (后轴x0.175+偏置0.075+半车长0.14)=0.0075m；此结论无需搜索或仿真。未发现起始靶板重叠。

这5张不得从64分母剔除或换seed，也不能归因成控制器跟踪失败；当前生成器规则未保护M10，准入规则本来只验证落格/互斥等。未来若要求发车空间保护，应显式变更规则与另立图集，不能修改本批已冻结输入。

## 6.4 批次结束独立复核与官方规则边界

实际64份逐图 audit 均存在：49份 input_or_planning、8份 compile、7份 audit，整体通过 **0/64**。7份动态为 index 03/08/18/22/24/61/63，均实际完赛、车心雷8/8、采样精确矩形物理区域通过；包含每处射击补足30秒后的任务时间依次 **265.46/206.42/256.34/258.20/255.30/207.78/244.14秒**，均超过180秒。命令包络 override 6/7、jerk超限5/7、减速度超限2/7、细网格剖面横加速度超限1/7；不得把前57份未运行控制器算成这些项目超限。仅08/22/24/63两臂均完成，可比较RMS，4/4改善；其余3份旧臂未完赛，比较不可评估，不能描述为RMS变差。

为排除生成器遗漏官方 M10 禁放条款，完整读取原始场地说明的相关页，并视觉核对起终点图。真源 `dzrobot_v2/src/history/附加资源/场地布置说明书V2-空地协同侦排挑战赛.pdf`（内文版本V1.2，2026-07-14）第4–8页规定随机8雷/8个水桶/相连2靶格/两处桥洞，第10页只说明4个起终点；未写起点邻格或M10禁放。视觉核对产物 `_tmp_session/traj-1025/frame-migration/official-field-start-page10.png`。另直接读 `docs/history_cache/02_仓库快照/ext_yb_tree_20261022.tar.gz` 内归档赛事规则文字第4–6、9页，亦未找到邻格禁放；准备阶段是先放机器人、再布随机场地。归档原文路径为归档内 `yb_tree/docs/raw_txt/02_15空地协同侦排挑战赛.txt`〔档/未落盘〕。

由此仅能称本批满足**现有可核实的静态落格规则**，不能声称复现了官方裁判生成器的全部隐藏限制；现有证据不足以把上述5张宣布为违规地图并剔除。障碍外形以较新的场地说明水桶口径为准；较早赛事文本中的35cm方障碍描述未用于覆盖当前用户物理参数。

## 6.5 全量后两处测试夹具迁移补漏

root追加授权后只修改隔离副本既有两套测试：`test_safety.py` 四边界夹具按同一刚体变换旋转，保留每点距旧边界和原0.6m余量；`test_telemetry.py` N10期望改为新坐标(0.175,3.325)、首朝向0，原误差阈值不变。逐套原命令复跑：`python src/dzbrain/tests/test_safety.py` **190/190、exit0**；`python src/dzbrain/tests/test_telemetry.py` **80/80、exit0**。日志 `_tmp_session/traj-1025/frame-migration/test-safety-frame-repaired.log`、`_tmp_session/traj-1025/frame-migration/test-telemetry-frame-repaired.log`。SHA256分别 dd9e8ebc1a4730ceb49e7c6d886b88d5d1ebedcc8ef23fc1e380b6ab2691bf1d、3686c78e69be3da26d881733285a52c155b9948ed965c33d51c16e1b64e6f26c。

`test_referee.py` 崩溃原因为隔离副本缺车端C++取证文件，与坐标无关；不改测试、不加跳过或路径回退。root已逐字节补入真源并报告原套件81/81，证据 `_tmp_session/traj-1025/frame-migration/referee-source-fixture-manifest.json`。本轮运行/物理/控制源码与64图输入保持冻结，交回写域给root统一全量复跑。原生产目录不动。
