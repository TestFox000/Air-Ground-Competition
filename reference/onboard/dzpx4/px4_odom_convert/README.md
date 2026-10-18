# px4_odom_convert

将定位数据转换为 PX4 所需的 `VehicleOdometry` 格式，并发布至 `/fmu/in/vehicle_visual_odometry`。节点只在收到新观测时发布，不缓存重发或补帧。

支持三种定位数据来源，通过 launch 参数 `mode` 切换：

| 模式 | 数据来源 | 话题/接口 | 频率 |
|------|---------|-----------|------|
| `odom` | FASTLIO2 原始里程计 | `/Odometry` | 跟随输入话题 |
| `localization` | FAST_LIO_LOCALIZATION | `/localization` | 50 Hz |
| `tf` | FASTLIO2 localizer TF 树 | `map → lidar` | 跟随 TF 新观测 |

> `odom`、`localization` 模式收到一条新消息就转换并发布一条；`tf` 模式以 50 Hz 查询 TF，但只有 TF 时间戳变化时才发布。

---

## 定位方案

### 方案一：FASTLIO2 建图 + localizer 定位（`tf` 模式）

#### 1. 建图

```bash
# 启动 LIO 建图
ros2 launch fastlio2 lio_launch.py

# 启动位姿图优化
ros2 launch pgo pgo_launch.py

# 保存地图
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: 'your_save_dir', save_patches: true}"
```

#### 2. 地图优化（可选，HBA）

```bash
# 启动 HBA
ros2 launch hba hba_launch.py

# 执行地图精化
ros2 service call /hba/refine_map interface/srv/RefineMap \
  '{"maps_path": "your_maps_directory"}'
```

#### 3. 定位

```bash
# 启动 LIO
ros2 launch fastlio2 lio_launch.py

# 启动 localizer
ros2 launch localizer localizer_launch.py

# 发送重定位请求（填入初始位姿估计）
ros2 service call /localizer/relocalize interface/srv/Relocalize \
  "{pcd_path: '/home/duzhong/dzpx4/src/maps/map.pcd',
    x: -0.0495,
    y: -1.32,
    z: 0.0,
    yaw: 0.0,
    pitch: 0.0,
    roll: 0.0}"

# 检查重定位是否成功（code: 0 表示查询当前状态）
ros2 service call /localizer/relocalize_check interface/srv/IsValid \
  '{"code": 0}'
```

定位输出：`/tf`，TF 树为 `map → lidar → body`，频率约 **10 Hz**。

#### 4. 启动 px4_odom_convert（tf 模式）

```bash
ros2 launch px4_odom_convert px4_odom_convert.launch.py \
  mode:=tf \
  tf_map_frame:=map \
  tf_lidar_frame:=lidar \
  z_offset_m:=0.0
```

---

### 方案二：FAST_LIO_LOCALIZATION 定位（`localization` 模式）

#### 1. 定位

```bash
ros2 launch fast_lio_localization velodyne_localization.launch.py
```

定位输出：`/localization`（`nav_msgs/Odometry`），频率 **50 Hz**。

#### 2. 启动 px4_odom_convert（localization 模式）

```bash
ros2 launch px4_odom_convert px4_odom_convert.launch.py \
  mode:=localization \
  z_offset_m:=0.0
```

---

### 方案三：FASTLIO2 原始里程计（`odom` 模式）

```bash
ros2 launch fastlio2 lio_launch.py

ros2 launch px4_odom_convert px4_odom_convert.launch.py \
  mode:=odom \
  z_offset_m:=0.0
```

---

## Launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `mode` | `localization` | 数据来源：`odom` / `localization` / `tf` |
| `odom_topic` | `/Odometry` | `odom` 模式订阅的话题 |
| `localization_topic` | `/localization` | `localization` 模式订阅的话题 |
| `tf_map_frame` | `map` | `tf` 模式的父坐标系 |
| `tf_lidar_frame` | `lidar` | `tf` 模式的子坐标系 |
| `z_offset_m` | `0.0` | LiDAR 安装高度补偿（LiDAR 相对无人机重心的 Z 轴偏移，单位 m），三种模式均生效 |

---

## 坐标系转换

| 输入 | 输出 |
|------|------|
| 位置：ENU | 位置：NED |
| 姿态：ROS 四元数 | 姿态：PX4 四元数 |
| 角速度：FLU | 角速度：FRD |
