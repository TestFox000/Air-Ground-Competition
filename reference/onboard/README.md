# 机载电脑（Orin Nano）源码归档 · 只读参考

> 2026-10-01 通过 Type-C（NCM，`192.168.55.1`，用户 `duzhong`）从无人机机载电脑取回。
> 本目录**只读参考，不是构建目标**。用于理解官方技术路径、后续补齐 `dzuav`。

## 硬件

- 机载电脑：NVIDIA Jetson Orin Nano Developer Kit（tegra234，内核 5.15.185-tegra，15W 模式）
- 激光雷达：Livox MID360（顶部蓝穹顶）
- **下视侦察相机：ON Semiconductor AR0234**（USB `32e4:0234`，`/dev/video0/1`，
  约 230 万像素全局快门，走 `nvargus-daemon` / ISP，非普通 V4L2）
- 无线数传：CP2102 串口 LoRa 地面端（只连 PX4 飞控，走 MAVLink，**摸不到机载电脑**）

## 目录

| 路径 | 内容 | 状态 |
|---|---|---|
| `dzpx4/` | 飞行栈 4 个定制包：`px4_control`（Offboard 状态机+键盘）、`px4_path`（录路径/航点/样条跟随）、`px4_odom_convert`（SLAM→NED）、`px4_imu_bridge`（飞控 IMU→ROS IMU） | **完整、机上已编译** |
| `dzpx4_commands.txt` | 机上 `~/dzpx4/1.txt`：标准启动命令顺序 | — |
| `dzuav/` | 侦察栈：`CMakeLists.txt` + 5 个 cpp（HbbDetector / CellClassifier / MapLocalizer / UdpBroadcaster / main）+ `src/cell_classifier`（双头分类器完整 Python 训练管线） | **半成品，见下** |

dzpx4 里其余上游栈（livox_ros_driver2 / FASTLIO2 / FAST_LIO_LOCALIZATION /
mavros / ego-planner / px4_msgs / px4_ros_com）未归档，按需要时上机器看。

## dzuav 技术路径

```
HbbDetector  TensorRT YOLOv8 1280，7 类：
             mine / car / arrow(桥顶箭头) / target_red / cell_label / target_blue / obstacle
  → CellClassifier  96×96 MobileNetV3-Small 双头（14 字母 A~N + 11 行号 1~11）
  → MapLocalizer    ≥4 个格号 solvePnPRansac 自定位，目标投影到世界坐标（相机内参已硬编码标定）
  → UdpBroadcaster  二进制 UDP 广播端口 50000，魔数 0xefefefef，CRC16
```

## 关键缺口（接手时必须知道）

1. **dzuav 缺全部头文件，编译不过**：5 个 cpp 引用的 `.hpp`
   （TensorRtEngine 基类、CellId、Detection、WorldObject）在整机 `find /` 均不存在，
   机上也没有 build 产物。需按 cpp 反推补齐数据结构。
2. **main.cpp 只吃图片/视频文件，未接实时相机**（AR0234）。
3. UDP 50000 为**二进制**协议，与本仓库自研的 JSON UDP（link_protocol）不兼容；
   车端需另写接收解析。
4. 官方路径用「格号 PnP」自定位，**不需要单应矩阵 H 人工标定**。

## 模型/引擎（在机上，未入库）

`~/dzuav/models/`：`hbb_detector.pt`、`hbb_detector.onnx`、`cell_classifier.onnx`
`~/dzuav/engines/`：`hbb_detector_fp16.engine`、`cell_classifier_fp16.engine`（2026-07-28 构建）
