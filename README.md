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

机上源码只读归档见 `reference/onboard/`。

## 仓库结构

| 路径 | 作用 |
|---|---|
| `src/ag_integrate/` | 车端薄集成包：协议解析(`intel_protocol`)、场地坐标(`field_frame`)、世界模型(`world_model`)、任务路由(`mission_router`)、车命令(`car_commands`)、禁火保险(`fire_guard`)、`integrate_node` + launch |
| `src/ag_core/` | 与 ROS 无关的复盘工具：黑匣子 `blackbox` + `log_diagnosis` |
| `tools/remote_run.py` | SSH 上传/执行/取回（paramiko） |
| `tools/log_report.py` | 黑匣子复盘报告 |
| `docs/` | 规则、场地、实机接口与复盘；重构方案见 `docs/官方框架重构方案-20261001.md` |

## 常用命令

```bash
python run.py            # 不带子命令 = 跑全部单元测试
python run.py check      # 环境体检
python run.py log        # 复盘报告
```

实车（香橙派，需 ROS2，默认 dry_run 只算不发）：

```bash
ros2 launch ag_integrate integrate.launch.py
# 验证后实发：在 launch/节点参数里将 dry_run 置为 false
```

## 测试

```bash
python -m pytest tests -q
```

纯算法全部可在 Windows 直接跑，无需 ROS/板卡；`integrate_node` 运行在车端 Linux（依赖 rclpy）。

## 下一步（待硬件）

1. 机载：补全 `dzuav` 缺失头文件 → 编译 → main 接 AR0234 实时帧（详见 `reference/onboard/README.md`）。
2. 车端：上传 `ag_integrate` 到香橙派，先 dry_run 验证，再实发。
3. 现场核实中场挡板/桥洞拓扑，必要时在 `mission_router` 加“只能从可通行格穿越”约束。
