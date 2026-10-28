---
来源: 用户2026-10-25全仓坐标统一指令
更新: 2026-10-25
status: in_progress
ns: traj-1025
agent: /root/speed
owner: /root/speed
reviewer: /root
树节点: TR-9
写域: _tmp_session/traj-1025/frame-migration/dzrobot_v2/ 下 隔离副本的 tools/sim.py、tasks/tracker.py、tasks/localization.py、tasks/mission.py、io/**、main.py；对应test_sim.py、test_tracker.py、test_localization.py、test_mission.py、test_pivot.py、test_contract.py、test_bridge_gate.py；tests/fakes.py 及既有共享夹具；隔离副本 src/car_localization/** 生产者/旧消费者；仅坐标迁移
---

# TRAJ-FRAME-RUNTIME-20261025 · 起始姿态与运行链迁移草案

## 1 台账与目标
已读D-004/D-023/D-027/D-041。用户明确翻案D-004的坐标命名：+X=N10→M10（西），+Y=N10→N11（南），N9为北即−Y；起步yaw=0°、正舵左转不变。新证据为用户本轮明确指定；影响面为坐标转换/初态/物理区域/消费者；不迁移会使相同摆位出现90°错相。其余比赛与规划规则不重开。

## 2 复用与新增理由
复用唯一core.geometry和现有physics/测试树，不另建坐标真源。正交变换为 x_new=y_old，y_new=11×0.35−x_old，yaw_new=yaw_old−π/2，det=+1；不可只交换xy。隔离副本源哈希在frame-migration/source-manifest.json。

## 3 写域与保护
隔离副本的 tools/sim.py、tasks/tracker.py、tasks/localization.py、tasks/mission.py、io/**、main.py；对应test_sim.py、test_tracker.py、test_localization.py、test_mission.py、test_pivot.py、test_contract.py、test_bridge_gate.py；仅坐标迁移。所有源码修改仅在隔离副本；原共享工作区未交还的运行链不写、不部署。与其他成员按文件分工，禁止整份覆盖他人新改。

## 4 验收
现有对应测试脚本、154格坐标往返、N10→M10 yaw0、N10→N9 yaw−π/2、左转符号、两图物理审计及原生全量回归；物理阈值不改。离线补丁审查后才处理共享工作区接入。

## 5 回滚点
隔离副本改前source-manifest.json及原共享源码；禁止reset/clean。原运行链无本卡改动。

## 6 执行与结论

- 仅隔离副本：sim 起拍规则取 core 常量，场地/挡板改四端点 physics API，SVG 格网沿新轴；CSV 保持原列顺序并追加 map_frame_id。mission 的格方向量化改用 core，main 默认 reset 指向设计起点 +X。
- 定位生产者 serial_reader 的 START=(.5,9.5)、DR yaw offset=0；与 path_executor/generate_path 一起复用 core 的格/米/航向转换。保留 ROS 入口，car_localization 声明 dzbrain 运行依赖；未部署。published quaternion 仍按现有 raw 协议，reset 的 snap offset 不在本次改成新协议。
- 历史 fixture 数值保留为 LEGACY 序列，再通过 core 显式旋转；FakeBridge 不做隐式兼容变换。沿程/侧向偏移、初始车头一起旋转，距离容差、停车语义和物理硬断言不改。
- 官方命令（在隔离 dzrobot_v2 内）：`python src/dzbrain/tests/test_tracker.py` → 139/139、`python src/dzbrain/tests/test_localization.py` → 171/171、`python src/dzbrain/tests/test_contract.py` → 62/62、`python src/dzbrain/tests/test_pivot.py` → 55/55，均 exit 0。`python src/car_localization/test_coords.py` → 154 格往返通过、N10/M10/N9/N11 方向通过；compileall 全部通过。
- `python src/dzbrain/tests/test_bridge_gate.py` → 44/47、exit 1；三红与原基线逐条相同，车身/车心计数 6/416/179 一致，未重钉数字。
- 官方 mission 原命令仍因已删除 config 地图入口而在导入期报错；按 root 要求不夹带依赖修补。仅内存将缺失入口替换为既有第六图 fixture 的诊断跑出 229/246（17 红为旧路线/桥洞/44 点断言），不是正式 suite 通过。其后新增 signed 单点/裸命令契约，裸命令归一化缺少负轴 token 的源修复待批次冻结解除。
- sim 的新帧起拍断言已改，尚未重跑完整长套件以免与随机 64 图抢 CPU。baseline 独立短验收含 5 拍双臂 CSV/SVG，全组 39/39；不能代替两图/64 图闭环验收。
- 证据：`_tmp_session/traj-1025/frame-migration/runtime-tests/`；源码哈希 source-hashes.json，审阅差异 runtime-frame.patch，正式及诊断日志各自命名。原共享运行源码未写；候选仍离线，PIVOT/ALIGN 运行链不删除。
- 当前状态：运行源码冻结供统一 64 图同版执行；测试可继续。未满足全部验收，不提交、不双推、不部署。
- 64 图批次结束后，仅修改隔离 mission 的命令归一化四轴白名单，使裸 `F10,-y` 和 `N10 M10,-y` 保留负号；没有修改控制器、速度核、sim 或压力结果。原树四项 signed 检查全部通过；内存诊断为 233/250、17 红，失败编号与原诊断基线相同，新增两红已清零。缺失地图入口保持原状，诊断不替代正式 suite。证据为 runtime-tests/mission-signed-fixed-diagnostic.log 与 mission-signed-fixed-summary.json；mission SHA256=bf8bff55c61203071ed21de44d5c2df3e334e6a70c1b58fc15684c65d23aed83。交回 root 串行全量，后续不再写源码。
- 全量后复核坐标新增红：两套 AGC-T1 历史回放仍直接喂旧帧米制/度姿态，改为保留 *_LEGACY 原值并显式调用 core 刚体变换；测试仍保持原门限、弧长和段号断言。两套分别恢复 22/22、28/28，日志 runtime-tests/agc-mission-gate-migrated.log 与 agc-progress-migrated.log；原全量红日志保留。
- 全量还定位 mission 足迹诊断漏迁：废止 barrier_centerline 抛 TypeError，被旧异常回退吞掉后继续画 x=1.75 的旧竖挡板。仅隔离任务链改用 physics.barrier_segment 的端点及默认真源，复用 body_corners 形成原厚度矩形；目标/障碍和物理阈值不改。原测试 test_autoplan 的 10.35 恢复通过，全套 177/177、exit 0；尾段证据 runtime-tests/autoplan-barrier-migrated-tail.log。该诊断不在 64 图运行链，压力结果与控制核未变。

### 跨端场地坐标只读审计（64 图批次期间）

本轮 64 图只验证车端隔离副本的离线候选，不能证明无人机与车的部署版本一致、ROS/UDP 两端坐标契约一致或真实双端闭环完成。本节只读核查本地源文件，未连接设备、未发布消息、未运行重测试；感知、相机、飞控机体系与 UDP 算法均未修改。

| 边界 | 当前源码证据 | 尚需适配或确认 |
| --- | --- | --- |
| Python 感知场地米制 | `dzsmartscout/tools/perception/grid.py` 的 block_to_meters/反解/场地边界/半场/角点仍为旧车帧；`car_pose.py` 的 to_json 直接输出旧帧 x_m/y_m/yaw_deg，未带帧标识 | 将场地 GridSpec 与对应输出、缓存标定元数据统一迁移，或在明确边界转换一次；不可只改消费者，更不可旋转相机内参、机体系向量或相对安装角 |
| 车状态到路线裁剪 | `dzsmartscout/tools/planner/route_crop.py` 的 cell_xy/_course_through 使用上述旧 GridSpec，locate_anchor/handover_check 直接减 car_state.yaw_deg；末后缀正则仅接受 X/Y | 新车 yaw0 朝 M10 时，旧场地计算会使重复格消歧及交接航向差 90°；上游 GridSpec 与该消费者须同帧迁移，signed 后缀契约也未闭合 |
| /nav/waypoint 发令 | `dzsmartscout/tools/link/dzlink_supervisor.py` 的 reset_command 默认 y，reset/路线正则只接受 x/y；`dz_send_waypoint.py` 透传文本 | 设计起步应为 N10,x；新合法的负轴末后缀会被 supervisor 拒绝。历史命令与新命令须明确帧语义，不能对透传文本无版本地猜测旋转 |
| 视觉二进制车位姿 | `dzsmartscout/src/uav_camera_code/onboard/dzuav/src/MapLocalizer.cpp` 的 world 使用 x=.065+col×.35、y=.295+(11-row)×.35；UdpBroadcaster 的 0x4002 发 x/y 米和 yaw 弧度，未带帧标识 | 这是带偏移的另一套视觉场地帧，不能套旧车帧到新车帧的 90°公式。按现有公式推导到新帧应为 x_new=4.790−x_vision、y_new=3.970−y_vision、yaw_new=wrap(yaw_vision+π)；该推导尚未做部署验证，应通过具名帧适配层与格名锚点证明 |
| UDP 适配与回传 | `dzsmartscout/tools/link/dzudp_adapter.py` 保持视觉 world_to_cell，输出格名；0x4002 仅记录、不发布 car_pose。`dzudp_core.py` 的 PUBLISH_ENABLED=False。车端 `tasks/drone.py` 的 _send_car_state 直接发 pose.yaw 的度值，未带帧标识 | 视觉二进制到格名的现有落点无需随车坐标改公式；若交叉比较视觉 yaw 与 car_state yaw，必须显式转帧。实际部署是否另跑远端 ag_integrate 生产者/转发器，本地源码不足以确认 |
| field_layout | 车端 `tasks/drone.py` 接受 field_layout；`scout/layout.py` 构造的是格名和行列屏障数据，无米制/yaw 字段。无人机 `tools/perception/protocol.py` 的 PAYLOAD_TYPES 当前无 field_layout，亦未定义其构造器，而既有 `test_field_layout_v2.py` 正在导入这些符号 | 格名本身不旋转。当前可确认本地源接口版本缺口，不能据此报告无人机真实布局发送链已具备；需核对实际部署生产者和同版契约测试。本节未运行该测试，不将静态发现记成实跑结果 |

部署验证缺口：尚无两端安装源哈希及唯一发布者清单；尚无新帧 N10(.175,3.325)/M10/N9/N11 锚点与朝向回放；无视觉米制→格名→新车帧的闭环证据；无正负四轴路线/reset 通过实际接收者的同版证据；无新帧 car_state 对消费方的帧标识或明确拒收旧帧验证。以上均保留未完成，不借车端离线 64 图结果替代。

## 9 经验总结
现象：轴名和物理格名方向不一致使起步差90°。处置：按用户格名方向重建右手系，并让所有消费者同时迁移；不改一个yaw常量假装完成。

## 10 文档变更
新增本卡（root 建立）、本次追加执行记录；代码内契约注释同步。长期结论提炼待验证，归档无。
