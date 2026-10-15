# air_ground · 空地协同侦排挑战赛

面向官方技术总框架的车端集成工程。**飞的、看的、跑的都用官方栈，本仓库只做官方没给的集成胶水。**

## 技术总框架

```
[机载 Orin Nano]
  dzpx4（官方）：Livox MID360 → FASTLIO2 建图/定位 → PX4 飞控
  dzuav（官方/先行者）：AR0234 下视相机 → HbbDetector(7 类) → CellClassifier(格号)
                          → MapLocalizer(格号 PnP 自定位) → 二进制 UDP 广播 :50000
                                                                      ↓
[车端香橙派]
  官方 dzrobot：path_executor 负责开车，det_node 负责瞄准开火
  ag_integrate（本仓库）：收 :50000 → 世界模型 → 任务路由 → 下发 /nav/waypoint
```

要点：
- 定位靠识别**地面格号做 PnP**，无人工单应矩阵标定。
- 空地通信为**二进制** UDP :50000（魔数 `0xefefefef`、CRC16）。
- 只有**蓝色敌方靶**才标 `*`（停靠开火）；红色友方绝不打（误击 −5/计数）。

机上源码只读归档见 `reference/onboard/`；**可部署的 dzuav 补全版见 `onboard/dzuav/`**。

## 仓库结构

| 路径 | 作用 |
|---|---|
| `onboard/dzuav/` | 机载侦察程序（要拷到 Orin 编译）：4 个 cpp 沿用官方先行者代码，`include/dzuav/` 补全了缺失头文件；`main.cpp` 在归档版（`reference/onboard/`，已含图片/视频/`argus:`/`uvc:` 全部输入源）基础上做健壮性加固：UDP 端口校验、相机断流重开、单帧异常容错、贴边半标签过滤等（diff 约百行，无协议变更） |
| `src/ag_integrate/` | 车端薄集成包：协议解析(`intel_protocol`)、场地坐标(`field_frame`)、世界模型(`world_model`)、任务路由(`mission_router`)、车命令(`car_commands`)、禁火保险(`fire_guard`)、`integrate_node` + launch |
| `src/ag_core/` | 与 ROS 无关的复盘工具：黑匣子 `blackbox` + `log_diagnosis` |
| `ag_orbit_target/` | 支线数据采集工具（绕靶录像采集，不在现役主链路；**勿与官方栈同跑**，详见其 README） |
| `tools/remote_run.py` | SSH 上传/执行/取回（paramiko） |
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

1. 机载：把 `onboard/dzuav/` 拷到 Orin（与机上 `~/dzuav` 并列或覆盖其源码目录），
   `cmake -S . -B build && cmake --build build`；先用现成图片验证离线帧，再跑
   `./build/dzuav uvc:/dev/video0 ~/dzuav/engines/hbb_detector_fp16.engine
   ~/dzuav/engines/cell_classifier_fp16.engine`（若实测走 CSI/ISP 则换 `argus:0`）。
2. 车端：上传 `src/ag_integrate` 到香橙派 colcon build（`tools/car_deploy.py`），
   用 `tools/car_match_bringup.sh` 两阶段启动，先 dry_run 验证，再实发。
3. 现场核实中场挡板/桥洞拓扑后，给节点传 `cross_gates:=['<可通行格>']` 启用
   “只能从可通行格穿越半场”约束（代码已就绪，默认关闭）。
   ⚠️ 注意：显式传入 cross_gates 会一并关掉 A/B/C 列的自由穿越
   （默认自动推导含 A/B/C + 桥洞缺口），见 `docs/interface_contract.md` §7。
