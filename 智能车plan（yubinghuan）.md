# air_ground —— 空地协同侦排挑战赛

无人机侦察 + 地面车执行的完整软件栈（ROS2 Humble / Python）。
算法核心与 ROS 解耦：**不出机房就能跑通全链路**，上车前先在笔记本验证闭环。

## 1. 架构

```
              ┌──────────── 机载端（无人机 / 机载电脑）────────────┐
 相机 ─▶ vision_node ─▶ /perception/detections ─┐
                                                ├─▶ intel_fusion_node ─▶ air_link_node
 相机 ─▶ vision_node ─▶ /perception/mines ──────┘   （像素→格号→情报）
              └──────────────────────────┬──────────────────────────┘
                                  UDP 广播（局域网，禁公网）
              ┌──────────────────────────┴──────────────────────────┐
              │           车端（地面车 / 香橙派 RK3588S）            │
 ground_link_node ─▶ /link/field_intel ─▶ mission_node ─▶ /mission/waypoints ─▶ 底盘
                                              ▲                                   │
 vision_node ─▶ /perception/detections ─▶ target_node                             │
                                              │                                   ▼
                                     /gimbal/command ─▶ referee_node     /mission/task_done
                                              ▲                │
                                              └── /referee/state┘
```

| 包 | 作用 | 依赖 ROS |
|---|---|---|
| `ag_core` | 纯算法：栅格地图、任务规划、后处理、敌我策略、裁判/链路协议、推理后端、单应标定 | 否（可单测） |
| `ag_interfaces` | 自定义消息 | 是 |
| `ag_perception` | `vision_node`（图像→检测）、`intel_fusion_node`（检测→格号情报） | 是 |
| `ag_mission` | `mission_node`（情报→规划→路点）、`target_node`（检测→云台/开火）、`field_fusion`（车端多帧投票） | 是 |
| `ag_referee` | `referee_node`（串口↔ROS）、`serial_io`（可注入假串口） | 是 |
| `ag_bridge` | `air_link_node` / `ground_link_node` / `fake_intel_node`、`udp_link` | 是 |
| `ag_bringup` | launch 文件 + 统一参数 | 是 |

两级情报确认是刻意设计：机载端 `IntelAccumulator` 抗**视觉误检**，车端 `IntelVoter` 抗**链路误码与口径不一致**。

## 2. 快速开始

```bash
pip install -r requirements.txt        # 核心：numpy + pytest
python -m pytest tests -q              # 162 项
python tools/sim_selfcheck.py          # 离线闭环自检（无需 ROS）
```

可选依赖（PC 调试 / 板端推理 / 串口）见 `requirements-extra.txt`。

### 2.1 上车前全链路仿真（无无人机、无串口、无模型）

```bash
ros2 launch ag_bringup sim.launch.py
```

`fake_intel_node` 逐帧揭示情报（带噪声），可观察 `/mission/plan`、`/mission/waypoints`、`/mission/status`。
推进任务只需发一次完成信号：

```bash
ros2 topic pub --once /mission/task_done std_msgs/msg/String "{data: C3}"
```

### 2.2 实机

```bash
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash

ros2 launch ag_bringup air.launch.py model_path:=/models/best_new.rknn   # 机载端
ros2 launch ag_bringup ground.launch.py                                  # 车端
```

调试阶段推荐（只瞄准不开火、不开串口）：

```bash
ros2 launch ag_bringup ground.launch.py enable_fire:=false simulate_referee:=true
```

## 3. 车端与底盘的接口

底盘只需订阅 `/mission/waypoints`（JSON）：

```json
{"done": false, "index": 2, "type": "mine", "cell": "C3",
 "path": ["B2","B3","C3"], "score": 5, "eta_s": 4.7}
```

走完一段后向 `/mission/task_done` 发布格号（`"C3"`）或任务类型（`"MINE"`），
`mission_node` 自动推进；收到新情报时只对**剩余**任务重排，已执行段不动。

## 4. 评分策略（180 s 单限）

| 项目 | 分值 | 策略 |
|---|---|---|
| 驶离起点 / 回到终点 | 5 / 5 | 始终预留 `goal_margin_s` 回终点 |
| 排雷 | 5 × 8 | 按「分值 / 耗时」贪心，时间不够时优先放弃远雷 |
| 桥洞 | 10 × 2 | 分值高、优先排 |
| 打击敌方 | 5 × 3 | 需连续 `consensus_frames` 帧类别一致才开火 |
| 误击友方 | −5 × 3 | 红面一律 `AVOID`，节点层还有一道硬拦截 |

满分 85（5+5+8×5+2×10+15）。`tools/sim_selfcheck.py` 会给出当前参数下的预估得分。

## 5. 现场必须先校准的三件事

1. **单应矩阵**（`tools/calib_homography.py`）— 没有它，像素落不到格号
   ```bash
   python tools/calib_homography.py --corners "120,80 980,95 965,700 105,690" --check "500,400"
   ```
   四角顺序固定为 A1 → K1 → K14 → A14。残差 > 5 cm 要重标。
   输出的 9 个数填进 `config/params.yaml` 的 `homography`。

2. **模型解码参数**（`tools/verify_decode.py`）— `cls_activation` / `kpt_unit`
   Ultralytics 导出到 RKNN 时行为不一致，猜错表现为「框准但瞄准点乱飘」。
   ```bash
   python tools/verify_decode.py --model models/best_new.rknn --image samples/frame.jpg --nc 2 --nkpt 1
   ```

3. **裁判系统帧格式**（`src/ag_core/referee_protocol.py`）
   官方未公开帧格式，当前是自洽的参考实现（AA 55 + cmd + len + payload + crc8）。
   拿到实机抓包后只改 `RefereeState.from_bytes` / `to_bytes` 的字段偏移。

## 6. 工具

| 工具 | 用途 |
|---|---|
| `tools/verify_decode.py` | 模型输出体检，校准 `cls_activation` / `kpt_unit` |
| `tools/calib_homography.py` | 四角像素 → 单应矩阵，输出可直接粘贴的 yaml |
| `tools/replay_intel.py` | 回放情报帧（JSONL），`--sweep` 扫描 `min_votes` / `min_ratio` / `ttl_s` |
| `tools/sim_selfcheck.py` | 离线闭环自检：情报→规划→执行→打击，输出预估得分与耗时 |

```bash
python tools/replay_intel.py --input tools/samples/intel_sample.jsonl --sweep
python tools/sim_selfcheck.py --speed 0.45 --noise-ratio 0.4 --min-votes 3
```

## 7. 测试与 CI

- `tests/` 共 162 项，覆盖：算法核心、协议编解码、情报投票、串口帧、UDP 链路、单应标定、图像解码
- `tests/ros_stub.py` 提供极简 ROS2 替身，使**节点层**（参数、话题、回调状态机、端到端链路）也能在没有 ROS 的机器上测试
- `tests/test_params_consistency.py` 校验 `params.yaml` 的参数名与节点声明一致，防止拼写错误静默失效
- `.github/workflows/ci.yml`：Python 3.10/3.12 跑单测 + 语法检查 + 工具冒烟；另有一个 ROS2 Humble 容器做 `colcon build` 与可执行文件检查

## 8. 链路可靠性

- 禁止公网；自带路由器组局域网，UDP 广播，端口见 `params.yaml`
- 每帧带 `seq` + `crc32`：车端丢弃乱序/重复帧，损坏帧计数发在 `/link/link_stats`
- 无新情报时发心跳，便于判断链路存活

## 9. 目录

```
air_ground/
├── src/
│   ├── ag_core/          纯算法核心（无 ROS）
│   ├── ag_interfaces/    自定义 msg
│   ├── ag_perception/    vision_node / intel_fusion_node / intel_accumulator
│   ├── ag_mission/       mission_node / target_node / field_fusion
│   ├── ag_referee/       referee_node / serial_io
│   ├── ag_bridge/        air_link / ground_link / fake_intel / udp_link
│   └── ag_bringup/       launch + config
├── tests/                pytest（Windows 可直接跑）
├── tools/                校准 / 回放 / 自检
└── .github/workflows/    CI
```

上场前的逐项确认见 `CHECKLIST.md`。
