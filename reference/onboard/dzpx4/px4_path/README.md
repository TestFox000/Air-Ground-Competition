# px4_path

`px4_path` 负责 FAST-LIO 路径录制、文件加载、坐标转换、路径平滑和目标生成。
它使用 `px4_control/msg/GotoTarget`，但不负责 Offboard、起降、锁桨或最终 PX4
设定值。

## 启动

`record_mode` 和 `follow_mode` 都只在 `config/params.yaml` 中配置，不再提供
同名 launch 参数，也不支持飞行中切换：

```yaml
px4_path_launch:
  ros__parameters:
    record_mode: true

path_follow_node:
  ros__parameters:
    follow_mode: "trajectory"
```

配置并重新构建后，两种模式使用同一条启动命令：

```bash
ros2 launch px4_path px4_path.launch.py
```

`record_mode=true` 只启动 `path_record_node`；`false` 只启动统一的
`path_follow_node`。`record_mode=false` 时再由 `follow_mode` 选择 `waypoint`
或 `trajectory`。控制节点和键盘节点需要通过 `px4_control` 分别启动。

## 录制

`path_record_node` 不订阅任何键盘或路径命令。启动后自动清空会话缓存并订阅
`/localization`，正常退出时补充末点并原子写入 TXT。RViz 路径发布在
`/path_record/visualization`。

录制代码只按“节点/算法”职责拆分：

- `src/path_record_node.cpp`：程序入口、基础采样、关键点判定、可视化和文件保存；
- `src/path_record_process.cpp`：处理参数、在线均值滤波、保存前平滑和近点压缩。

处理开关只在 `config/params.yaml` 中设置：

```yaml
enable_filter_smoothing: true
```

- `true`：启用在线滑动均值、保存前中心平滑和近重复点压缩。
- `false`：直接使用通过基础安全检查的原始 SLAM 位姿抽取关键点。

无论开关状态如何，无效数值/四元数、原始定位跳变、关键点距离与时间阈值以及
最终相邻距离检查都不会关闭。

文件每行格式：

```text
x y z qx qy qz qw
```

文件保存 FAST-LIO SLAM 原始坐标。加载时由 `path_loader` 统一转换为 PX4 NED。
当前录制与读取目录统一固定为：

```text
/home/lyl/code/dzpx4/px4_path/src/path
```

## 跟随方案与暂停恢复

新的 `src/path_follow.cpp` 只读取 `follow_mode`，然后实例化 waypoint 或
trajectory 接口；算法仍分别位于：

- `src/path_follow_waypoint_node.cpp`
- `src/path_follow_trajectory_node.cpp`

`waypoint` 按 `switch_tol` 推进游标，并通过 `max_step_dist` 限制单次位置目标
距离。它发布完整 `GotoTarget`，速度、加速度和 yawspeed 前馈均为 0。

`trajectory` 构建曲率感知、动力学受限的连续轨迹，支持急转弯分段、跟踪误差
减速/暂停和超限中止，并发布连续位置、速度、加速度、yaw 与 yawspeed。

两个方案都订阅锁存话题 `/px4_control/goto_enabled`：

- `false`：进入 `PAUSED`，停止发布并冻结游标或轨迹时间。
- `true`：首次启动或从 `PAUSED` 恢复原进度。
- trajectory 恢复时只重置 timer 基准，不重建轨迹或累计暂停时长。
- 到达终点后进入 `HOLD`；暂停再恢复时继续发布末端保持目标。

路径内部发生不可恢复错误时，节点发布 `GotoTarget(active=false)`，由控制节点
关闭许可并悬停。

## 参数

录制、waypoint 和 trajectory 的全部调试参数位于同一个文件：

```text
px4_path/config/params.yaml
```

`path_follow_node` 参数段包含两套跟随方案的参数并集；启动时仅选中方案读取其
所需项。路径目录和文件名只在 YAML 中配置，不再由 launch 覆盖。请保证：

```text
path_follow_node.path_dir  == path_record_node.path_dir
path_follow_node.path_file == path_record_node.output_file
```

这样录制和跟随才会指向同一个明确文件。
