# 高频 Odometry 与 Localization 修改说明

## 目标

- LiDAR 约 10 Hz 时，两帧雷达之间使用全部 IMU 传播状态。
- IMU 约 200 Hz 时，按真实观测时间选择约 50 Hz 新状态发布 `/Odometry`。
- 不重复发布缓存帧，不按墙钟补发历史帧。
- `/localization` 跟随新的 `/Odometry` 发布。
- 全局 ICP 的频率和 `map -> odom` 修正逻辑保持不变。

## `/Odometry` 数据流

```text
200 Hz IMU
        -> 相邻 IMU 角速度、加速度取平均
        -> 使用 FAST-LIO 的加速度尺度与过程噪声
        -> prop_kf.predict()，全部 IMU 均参与传播
        -> IMU 观测时间跨过 20 ms 时发布一个当前新状态
        -> 约 50 Hz /Odometry

10 Hz LiDAR
        -> 主 kf 完成 IMU 传播和 scan-to-map 更新
        -> 获得 lidar_end_time 的校正状态
        -> 在传播锁内复制到 prop_kf
        -> 重放所有晚于 lidar_end_time 的 IMU
        -> 后续新 IMU 继续从校正后的状态传播
```

发布由 IMU 事件驱动，不再由雷达处理定时器驱动。`prop_last_published_time`
记录最后一次实际发布状态对应的 IMU 观测时间。没有新传播状态时不发布；如果
发生较长的数据间隔，恢复后只发布当前最新状态，不追补中间历史帧。

## 并发与队列

- 使用三线程 `MultiThreadedExecutor`。
- IMU、LiDAR 输入和 FAST-LIO 映射处理分别使用独立的互斥回调组。
- IMU 订阅深度为 500。
- `prop_kf`、传播时间和 IMU 重放历史由独立互斥锁保护。
- 主 EKF 的雷达计算不会长期占用传播锁；只在复制校正状态和重放时短暂加锁。

## `/localization` 数据流

`transform_fusion.py` 每收到一条新 `/Odometry` 就计算：

```text
T_map_to_body = T_map_to_odom * T_odom_to_body
```

随后发布一条具有相同 `header.stamp` 的 `/localization`。没有新的
`/Odometry` 就不发布。全局 ICP 只更新 `T_map_to_odom`，在下一条
`/Odometry` 到达时生效。

## 时间戳与复位

- `/Odometry.header.stamp` 是发布状态对应的 IMU 观测时间。
- `/localization.header.stamp` 原样继承 `/Odometry.header.stamp`。
- IMU 时间戳回退时，清空主 IMU 缓冲、传播历史和发布去重状态。
- 雷达校正后先把 `prop_kf` 重放到最新 IMU 时刻，再继续对新 IMU传播。

## 运行诊断

节点每秒打印一次：

```text
Propagation rates: imu=... live_predict=... replay=... odom=... invalid_gap=...
```

正常情况下，`imu` 和 `live_predict` 应接近 IMU 输入频率，`odom` 应接近
50 Hz，`invalid_gap` 应保持为 0。
