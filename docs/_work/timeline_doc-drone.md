# 无人机线时间线 · doc-drone

> 一行一条；证据列为相对路径:行。覆盖 2026-10-06 雷达首通 → 2026-10-21 本册建立。

| 日期 | 事件 | 证据 | 对现在还有什么用 |
|---|---|---|---|
| 2026-10-06 | 雷达首通：10 Hz、19968 点/帧、Init lds lidar success | dzsmartscout/docs/drone/感知链体检_20261017.md:13-16 | 感知层全部基线数字的来源 |
| 2026-10-07 | 相机 AR0234 接入，/image_raw 29.245 Hz 出图 | dzsmartscout/fc-audit/fixes/04-camera-bringup/APPLY.md:134-140 | 相机 bring-up 仍照此执行 |
| 2026-10-07 | 机→车 DDS 首次闭环（车侧订阅数=2） | dzsmartscout/docs/无人机项目总览.md:125 | 跨机链路的起点，t4 之前的基础 |
| 2026-10-08 | 技术文档与总览落盘，登记 R-01~R-25 | dzsmartscout/fc-audit/无人机技术文档.md:663-673 | R 编号至今被 10-19 报告沿用 |
| 2026-10-12 | t4 双向全验：域 118、8 格 QoS、4 类报文、ack matched | dzsmartscout/tools/reports/t4-链路验证报告.md:16-27 | QoS 兼容铁律（反向静默 0 条）仍有效 |
| 2026-10-17 | 雷达+FAST-LIO+三 python 节点+VIO 42.4 Hz 全链拉通 | dzsmartscout/docs/drone/感知链体检_20261017.md:92、:97-113 | 定位链唯一一次完整验收 |
| 2026-10-17 | EV 被 EKF 接受：eph 0.032 / epv 0.029、RTT≈3 ms | docs/交接_无人机飞行分析_agent技术文档.md:39 | 定位可用性的唯一硬证据 |
| 2026-10-17 19:03 | Orin 重启后 uXRCE 会话死，Publisher 恒 0、每 61 s 重启 | dzsmartscout/docs/drone/uXRCE恢复手册.md:88-94、:169 | P0-1 的起点，恢复三步卡的依据 |
| 2026-10-17 20:47 | 机载相机录制 3 个产物（mp4/mkv，共约 29 MB） | _tmp_session/agent_overview_20261020/drone.md:288 | 全项目唯一机载视频素材 |
| 2026-10-18 晨 | agent 以 ttyACM1 恢复，8 路 /fmu/out 有数据（102/133/142 Hz） | docs/交接_无人机飞行分析_agent技术文档.md:43-50、:64 | 串口 A/B 冲突的 B 版来源；链路当前态 |
| 2026-10-18 18:38 | live 窗口 1280x720、21.0 fps、det=0、画面糊 | _tmp_session/agent_overview_20261020/drone.md:154、:314 | 视觉侦察未闭环的铁证（截图哈希 0ff7a610…） |
| 2026-10-19 | 全量只读审查出报告，R-01~R-25 状态逐条更新 | dzsmartscout/docs/drone/无人机项目详细报告_20261019.md:50、:331-332 | P0/P1 清单的现行版 |
| 2026-10-20 | 只读调研报告定四链路与证据分级（实/档/未） | _tmp_session/agent_overview_20261020/drone.md:3-5 | 本册全部证据口径的来源 |
| 2026-10-21 | 无人机资料索引库建立（本册 7 篇 + 本时间线） | docs/20_资料索引库/无人机/00_索引.md:1 | 此后查无人机资料只看这一处 |
