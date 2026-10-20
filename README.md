# air_ground · 空地协同侦排挑战赛

面向官方技术总框架的车端集成工程。**飞的、看的、跑的都用官方栈，本仓库只做官方没给的集成胶水。**

> 📌 **开工前必读**：[开工指引](docs/开工指引.md)（在哪干活、开工三步、收工两步）· [多副本协作公约](docs/多副本协作公约.md)（铁律与事故档案）

## 技术总框架

```
[机载 Orin Nano]
  dzpx4（官方）：Livox MID360 → FASTLIO2 建图/定位 → PX4 飞控
  dzuav（官方/先行者）：AR0234 下视相机 → HbbDetector(13 类) → CellClassifier(格号)
                          → MapLocalizer(格号 PnP 自定位) → 二进制 UDP 广播 :50000
                                                                      ↓ 同机回环
  机载自主规划（本仓库）：orin_plan_sidecar 收 :50000 → 世界模型 → 落格矩阵
                          → 任务路由 → 落 route_plan.json / cell_matrix.json / planned_path.txt
                          → 广播 0x4003 车命令
                                                                      ↓ UDP :50000
[车端香橙派]
  官方 dzrobot：path_executor 负责开车，det_node 负责瞄准开火
  ag_integrate（本仓库，薄接收）：收 0x4003 → 转 /nav/waypoint（规划不在车端）
```

要点：
- **规划跑在机上**：官方控制要求「无人机的定位、建图、识别、规划与控制等实时任务须由机载平台自主完成」。
  机载 `tools/orin_plan_sidecar.py` 直接复用 `src/ag_integrate` 的纯 Python 规划核心（无 ROS），
  车端 `integrate_node` 只需 `car_command_source=onboard` 变成薄接收（默认 `local` 保持旧行为）。
- 定位靠识别**地面格号做 PnP**，无人工单应矩阵标定。
- 空地通信为**二进制** UDP :50000（魔数 `0xefefefef`、CRC16）：`0x4001` 情报 / `0x4002` 车位姿 / `0x4003` 车命令。
- 只有**蓝色敌方靶**才标 `*`（停靠开火）；红色友方绝不打（误击 −5/计数）。

机上源码只读归档见 `reference/onboard/`；**可部署的 dzuav 补全版见 `onboard/dzuav/`**。

> ⚠️ **引擎与二进制的唯一口径**（2026-10-20 复查）：一键启动固定使用
> `<仓库>/onboard/dzuav/build/dzuav` + `hbb_detector_d13_fp16` + `cell_classifier_v4d_fp16`。
> 历史坑：`orin_run_dzuav.sh` 曾默认 `~/ag_test/dzuav` + `hbb_detector_fp16`，于是
> 「一键启动」拉起的是 9 月的 **7 类**老二进制与 7 类引擎，类别索引与 13 类协议错位
> （障碍被当蓝靶）而界面无任何异常。`tools/orin_accept_plan.sh` 会用二进制里的类别名
> （`uav_end`/`paper`）与 canonical 引擎名做前置断言，错版本当场报错。

## 仓库结构

| 路径 | 作用 |
|---|---|
| `onboard/dzuav/` | 机载侦察程序（要拷到 Orin 编译）：4 个 cpp 沿用官方先行者代码，`include/dzuav/` 补全了缺失头文件；`main.cpp` 在归档版（`reference/onboard/`，已含图片/视频/`argus:`/`uvc:` 全部输入源）基础上做健壮性加固：UDP 端口校验、相机断流重开、单帧异常容错、贴边半标签过滤等（diff 约百行，无协议变更） |
| `onboard/dzpx4_patch/` | 机载韧性定位补丁（px4_odom_convert：auto-align + jump guard + armed 门控 + z 换源），用 `tools/patch_deploy.py` 部署/回滚到 Orin 的 dzpx4 工作空间 |
| `src/ag_integrate/` | 规划与集成核心：协议解析(`intel_protocol` 含 0x4001/0x4002/0x4003)、场地坐标(`field_frame`)、世界模型(`world_model`)、落格矩阵(`field_matrix`)、任务路由(`mission_router`)、车命令(`car_commands`)、禁火保险(`fire_guard`)、`integrate_node` + launch。**规划当前跑在机上**（`tools/orin_plan_sidecar.py` 复用本包纯 Python 核心，无 ROS），车端 `integrate_node` 以 `car_command_source=onboard` 退化为薄接收 |
| `src/ag_core/` | 与 ROS 无关的复盘工具：黑匣子 `blackbox` + `log_diagnosis` |
| `ag_orbit_target/` | 支线数据采集工具（绕靶录像采集，不在现役主链路；**勿与官方栈同跑**，详见其 README） |
| `tools/remote_run.py` | SSH 上传/执行/取回（paramiko） |
| `tools/orin_deploy.py` | Orin 一键部署：dzuav 源码 + current 指针模型 + orin 工具链（含全部 `orin_*.py` 看门狗/黑匣子/采集/**规划旁挂 `orin_plan_sidecar.py`**）+ **机载规划包 `src/ag_integrate`**（覆盖前对机上原副本 `cp -a` 备份）推到 `~/dzuav_deploy/air_ground`；`--verify` 远程 L1 编译 + L2 自检（2026-10-16 实机跑通） |
| `tools/orin_accept_plan.sh` | 机载规划一键验收（机上跑，只写 `/tmp`）：① 规划进程起得来并绑住 UDP:50000 ② 同机广播投递成立 ③ 落格矩阵→门控→路线→三份产物 ④ 可选 `--send` 验 0x4003 真广播。随 `tools/orin_deploy.py` 自动上机 |
| `tools/patch_deploy.py` | `onboard/dzpx4_patch/` 韧性补丁部署/回滚：部署时写回滚索引 + md5 指纹，回滚先校验指纹再重编重启 |
| `tools/preflight.py` | 赛前自检：本地段（模型契约/指针/md5/配置，CI 同跑）+ 车端/机载 SSH 段 |
| `tools/flight_console.py` | 飞行黑匣子地面台：实时面板 + 应急指挥（降落/停GOTO/KILL 双确认）+ 黑匣子取回；自动把 `orin_flight_blackbox.py` 送上机上 tmux(agup:bb)。密码走 `AG_ORIN_PASS` |
| `tools/orin_watchdog.py` | 机载自治守卫（`orin_bringup.sh` 的 flight_wd 窗口）：断流/掉 OFFBOARD/位置跳变 → 机上本地自动 LAND |
| `tools/uav_link.py` | 无人机地面站：run 只读遥测+黑匣子（/fmu/out/* 自描述采样，8 条告警）、ctl 调控（arm/kill 等，--yes 闸门）、logs 取回机载日志、report 离线复盘分级。密码走 `AG_ORIN_PASS` |
| `tools/monitor_link.py` | 空地链路看板：监听 UDP:50000 解析 0x4001/0x4002，终端 ASCII 场地格图 + 帧率/CRC/延迟统计（Windows 可直接跑，配合 fake_udp_sender 彩排） |
| `tools/smoke_test_chain.py` | 车端全链路冒烟：UDP 帧→解析→态势→路径→命令→禁火，无 ROS/硬件依赖的快速自检入口 |
| `tools/orin_capture_dataset.py` | 机上数据采集（uvc/argus/视频源 → 标定/训练素材），随 orin_deploy 上机 |
| `tools/log_report.py` | 黑匣子复盘报告 |
| `tools/track_diag.py` | 循迹"信念 vs 真实"对照诊断：SSH 抓车上信念位姿/命令 + 相机 ArUco 解算场上真值，实时对照 + 规则报警（画圈/定位停更/执行器未动等）。现场用：双击根目录 `循迹诊断.bat`，手册 `docs/track_diag现场手册.md`，打印标记在 `docs/track_diag/markers/` |
| `docs/` | 规则原文(`raw_txt/`)、现场证据(`probe/`)、当前架构说明；重构前的旧文档统一在 `docs/archive/`（带过时标注） |

## 常用命令

```bash
python run.py            # 不带子命令 = 跑全部单元测试
python run.py check      # 环境体检
python run.py log        # 复盘报告
```

实车（香橙派，需 ROS2，默认 dry_run 只算不发）：

```bash
# 两阶段启动（循迹对齐）：先底盘+定位+integrate_node 做朝向对齐，
# 对齐后再起 path_executor——它一启动就以 20Hz 保活 pursuitAngle，
# 会把 dzactuator 钉死在模式2，/cmd_vel 转向命令被静默丢弃（源码实测）。
tools/car_match_bringup.sh            # 车上执行；AG_DRY_RUN=0 实发
```

## 测试

```bash
python -m pytest tests -q
```

纯算法全部可在 Windows 直接跑，无需 ROS/板卡；`integrate_node` 运行在车端 Linux（依赖 rclpy）。
CI 已修复为 compileall + pytest + preflight 本地段（`.github/workflows/ci.yml`，全程不依赖 ROS/硬件）。
循迹链路有软件在环回归（`tests/test_sim_tracking.py`）：加载官方 path_executor
真码 + dzactuator 契约仿真，验证"68.6° 摆位偏差 → align 对齐 → 发车到点"全流程，
对照组证明不 align 必画圈。

## 下一步（待硬件）

1. ~~机载：把 `onboard/dzuav/` 拷到 Orin 编译~~ **已完成（2026-10-16）**：`tools/orin_deploy.py`
   一键部署（源码+模型+工具链 → `~/dzuav_deploy/air_ground`），`--verify` 实机编译跑通，
   当日两场实飞（日志取证见 `docs/悬停稳定性取证-20261017.md`）。换机/刷机后机上重跑
   `tools/orin_convert_engines.sh -o ~/dzuav/engines` 即可。剩下待验证：相机实测
   （格号识别率/PnP 稳定性/帧率）与 dzpx4 单飞航线执行。
2. 车端：上传 `src/ag_integrate` 到香橙派 colcon build 并切薄接收：
   ```bash
   tools/car_deploy.py --host <车IP> --user duzhong --command-source onboard
   ```
   （`--command-source onboard` 会把 `car_command_source` 写成薄接收，原配置自动备份为
   `<工作空间>/match_params.yaml.deploy_orig`——放在包目录**外**，后续每次部署都不会连带删掉；
   回滚：`--command-source local`。不传该参数则完全不改，车端仍按 `local` 自己规划。）
   然后用 `tools/car_match_bringup.sh` 两阶段启动，先 dry_run 验证，再实发。
   ⚠️ 机载 `orin_plan_sidecar` 未跑时车端不会收到任何路线（在等 0x4003）。
3. 现场核实中场挡板/桥洞拓扑后，给节点传 `cross_gates:=['<可通行格>']` 启用
   “只能从可通行格穿越半场”约束（代码已就绪，默认关闭）。
   ⚠️ 注意：显式传入 cross_gates 会一并关掉 A/B/C 列的自由穿越
   （默认自动推导含 A/B/C + 桥洞缺口），见 `docs/interface_contract.md` §7。
