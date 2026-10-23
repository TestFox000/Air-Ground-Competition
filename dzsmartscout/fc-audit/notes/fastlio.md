# FAST_LIO_LOCALIZATION_ROS2 — 只读代码审计笔记

- 审计对象：`fc-audit/src-mirror/FAST_LIO_LOCALIZATION_ROS2/`（不含 `doc/`）
- 对照源：`src/FAST_LIO_LOCALIZATION_ROS2/`、`src/livox_ros_driver2/`、`fc-audit/src-mirror/px4_odom_convert/`
- 审计方式：纯静态阅读（未 SSH、未运行节点、未改动被审代码）
- 文中路径若无前缀，均相对 `FAST_LIO_LOCALIZATION_ROS2/`
- 行号基于上述镜像副本；镜像与 `src/` 副本的哈希对比结果见 §1.1

---

## 0. 结论速览（TL;DR）

| 问题 | 结论 |
|---|---|
| 实际生效的 config | **`config/mid360_test.yaml`**（launch 默认值，运行手册未覆盖） |
| 是否存在 `config/` 下 7 个 yaml | **实际 7 个**：`avia / horizon / mid360 / mid360_test / ouster64 / velodyne / velodyne_test` |
| MID-360 该用哪个 | **基础 LIO 参数上两者都对 MID-360 有效**，但两者用途不同：`mid360_test.yaml` 是"定位全链路"配置（本地图路径 + 4 个节点参数），`mid360.yaml` 是"纯建图/纯 FAST-LIO"配置且带一个**不存在的绝对路径**。运行手册选 `mid360_test.yaml` **正确**，但 `mid360.yaml` 里的地图路径会让副作用出现（见 §2.4、§7-R4） |
| 最终对外里程计 | **`/localization`**，`nav_msgs/msg/Odometry`，`frame_id="map"`，`child_frame_id="body"`，约 50 Hz，**不 latched（VOLATILE）** |
| 输出坐标系 | `map` 帧 = 静态 PCD 地图帧（原点/朝向由建图时决定）；`camera_init` = FAST-LIO 自身原点。**是"Z 朝上"的右手系，但不是 ENU**：初始化只对齐重力、**不对齐偏航**（见 §4） |
| 重定位怎么触发 | **必须人工发一条 `/initialpose`（`geometry_msgs/PoseWithCovarianceStamped`，VOLATILE，一次性）**；此后每 2 s 自动 ICP 修正。**首发竞态不可自愈**，失败就永久卡死（见 §5、§7-R1/R2） |

---

## 1. 包的整体结构

### 1.1 二进制与源码

- `CMakeLists.txt:95` — 唯一可执行文件：`fastlio_mapping`（`src/laserMapping.cpp` + `include/ikd-Tree/ikd_Tree.cpp` + `src/preprocess.cpp`）。C++ 代码**没有独立于 laserMapping.cpp 的节点**；`fastlio_mapping` 就是整个 C++ 侧。
- `CMakeLists.txt:129-135` — 4 个 python 脚本以 `install(PROGRAMS ...)` 装到 `lib/fast_lio_localization/`，**不打 `executable` 位以外的包装**；launch 里直接以 `executable='global_localization.py'` 等名字调用（`velodyne_localization.launch.py:58/68/78`）。
- `CMakeLists.txt:124-127` — `config launch rviz rviz_cfg` 原样装到 `share/fast_lio_localization/`，所以 launch 里 `get_package_share_directory` 取到的 config 就是源码里那一份（无生成、无替换）。
- 镜像 vs `src/` 逐文件 SHA256：`src/laserMapping.cpp`、`launch/*.py`、`scripts/*.py`、`config/mid360_test.yaml`、`config/velodyne_test.yaml`、`MODIFICATIONS.md` **全部 SAME**；**唯一差异**是 `config/mid360.yaml:10`（镜像 `"/home/duzhong/dzpx4/src/maps/buliding_a_floor1.pcd"`，上游 `"./test.pcd"`）。→ 说明镜像 = 无人机当前代码，且 `mid360.yaml` 的地图路径是**本机（本项目）后来手改的**。

### 1.2 五个执行体的职责

| 执行体 | 文件 | 职责 |
|---|---|---|
| `fastlio_mapping` | `src/laserMapping.cpp` | FAST-LIO 核心：LiDAR+IMU 紧耦合 IEKF，输出 `camera_init→body` 位姿。另含 §6 记录的高频传播改造 |
| `global_map_publisher.py` | `scripts/global_map_publisher.py` | 启动时用 Open3D 读一个 PCD（`global_map_publisher.py:23`），每 `interval` 秒把整张地图作为 `PointCloud2` 发到 `/global_map`（`:31`、`:54`）。**不订阅任何话题**，是链路最上游 |
| `global_localization.py` | `scripts/global_localization.py` | 收到 `/global_map` 建体素地图；收到 `/initialpose` 做一次全局 ICP；成功后起 2 Hz 定时器持续 ICP，把 `map→odom` 修正发布为 `/map_to_odom`（`:52-65`、`:84-100`、`:115-142`） |
| `transform_fusion.py` | `scripts/transform_fusion.py` | 每来一条 `/Odometry` 就用最新 `T_map_to_odom` 合成一次 `/localization`，并广播 TF `map→camera_init`（`:36-39`、`:52-87`） |
| `publish_initial_pose.py` | `scripts/publish_initial_pose.py` | 一次性命令行工具：把 6 个数值（x y z yaw pitch roll）发成一条 `/initialpose`，然后退出（`:14-23`、`:40-42`）。**launch 不启动它** |

### 1.3 依赖方向与启动顺序

```
[livox_ros_driver2 msg_MID360_launch.py]   ← 必须另外、先启动（本 launch 不含）
        │  /livox/lidar (CustomMsg 10Hz) + /livox/imu (Imu 200Hz)
        ▼
  global_map_publisher ──(/global_map, 5s 周期)──┐
        │                                        │
  fastlio_mapping ──(/Odometry, ~50Hz)───────────┼──► transform_fusion ──► /localization  ★对外唯一出口
        │  (/cloud_registered, /cloud_registered_body, /path, /Laser_map)   └──► TF map→camera_init
        │  (TF camera_init→body)
        ▼                                        │
  global_localization ◄──(/initialpose, 人工/一次性)┘
        └──(/map_to_odom, 2Hz)──► transform_fusion
```

- 硬性先后关系：`global_map_publisher` → `global_localization`（`global_localization.py:85-87` 地图为空时拒绝初始化）；`global_localization` 成功 → `transform_fusion` 才有非单位阵修正（`transform_fusion.py:54`，`map2odom is None` 时退化为 `eye(4)`）。
- **launch 一次性并行拉起 4 个节点 + rviz，不编排顺序**（`velodyne_localization.launch.py:100-104`）。依赖关系靠"全局地图 5 s 一周期 + 人工 initialpose"兜住，见 §7-R2。

---

## 2. launch 与实际生效的 config

### 2.1 两个 launch 的默认参数

| 项目 | `velodyne_localization.launch.py` | `mapping.launch.py` |
|---|---|---|
| 默认 `config_file` | **`mid360_test.yaml`**（`:33`） | **`mid360_test.yaml`**（`:34`） |
| 默认 `config_path` | `share/fast_lio_localization/config`（`:15`） | 同左（`:15`） |
| 默认 rviz | `true`（`:37`） | `true`（`:38`） |
| 默认 rviz 配置 | `rviz/fastlio_localiztion.rviz`（`:16`） | `rviz/fastlio.rviz`（`:17`） |
| 启动节点 | `fastlio_mapping`(name=`fast_lio_mapping`) + `global_localization.py` + `transform_fusion.py` + `global_map_publisher.py` + rviz2（`:46-104`） | 只有 `fastlio_mapping`（name=`fast_lio_mapping`）+ rviz2（`:46-68`） |

**两个 launch 的 `config_file` 默认值相同**，都是 `mid360_test.yaml` → `mapping.launch.py` 会把 `/global_localization`、`/transform_fusion`、`/global_map_publisher` 三个参数块一起塞进参数文件，虽然只有 `fastlio_mapping` 在跑，那三段参数**静默失效**（无害但会误导排查）。

### 2.2 参数是怎么落到节点上的

`velodyne_localization.launch.py:50-51`：

```python
parameters=[PathJoinSubstitution([config_path, config_file]),
            {'use_sim_time': use_sim_time}]
```

→ yaml 走 `__node:=<name>` / `__ns` 规则；命令行 dict 后合并、优先级更高。因此：

- `/**` 通配块（`mid360_test.yaml:19`）的 `common.*`、`mapping.*`、`preprocess.*`、`publish.*`、`pcd_save.*` 落到 `fast_lio_mapping`（`laserMapping.cpp:953-1015` 逐个 `get_parameter_or` 读取，键名逐一对得上）。
- `/global_localization:`（`:1`）落到 `name='global_localization'` → `global_localization.py:30-42` 读取，键名对得上。
- `/global_map_publisher:`（`:14`）落到 `name='global_map_publisher'` → `global_map_publisher.py:15-18` 读取。
- `/transform_fusion:`（`:10`）落到 `name='transform_fusion'` → **但这个节点只声明了 0 个参数**（`transform_fusion.py` 全文无 `declare_parameter`）→ `freq_pub_localization: 50.0` 是**死参数**，被静默忽略。函数上它靠"每条 `/Odometry` 触发一次"达到 50 Hz，结果偶然正确（README `README.md:85` 却把它写成"发布频率"，与实际机制不符）。
- **`/global_map_publisher/map_file_path` 被配了两遍**：`mid360_test.yaml:16`（精确节点名）和 `:29`（`/**`）。ROS 2 参数文件对各条目的优先级没有可依赖的规范（本地无法在无 ROS 环境验证），**这是一个应当消除的隐患**，见 §7-R5。

### 2.3 7 个 yaml 的区别

| 文件 | 结构 | 雷达相关关键值 | 用途 |
|---|---|---|---|
| `mid360.yaml` | 只有 `/**`（`:1`） | `lidar_type:1`、`scan_line:4`、`blind:0.5`、`timestamp_unit:3`、`det_range:100`、`extrinsic_est_en:true`、`extrinsic_T:[-0.011,-0.02329,0.04412]`、`filter_size_surf/map:0.5`、`point_filter_num:3`、`pcd_save_en:true` | **纯 FAST-LIO**。无任何 `/global_*` 节点参数。**`:10` 写死 `"/home/duzhong/dzpx4/src/maps/buliding_a_floor1.pcd"`**（该文件在 `src/maps/` 下不存在，实际只有 `map.pcd` / `map_compile_4cm.pcd` / `output_voxel_0.05.pcd`；且 "buliding" 是原作者拼写错误） |
| `mid360_test.yaml` | 4 块：`/global_localization`、`/transform_fusion`、`/global_map_publisher`、`/**` | `lidar_type:1`、`scan_line:4`、`blind:0.8`、`timestamp_unit:3`、`det_range:40`、`extrinsic_est_en:false`、`extrinsic_T` 同左、`filter_size_surf/map:0.18`、`point_filter_num:2`、`max_iteration:5`、`acc_cov/gyr_cov:0.8`、`pcd_save_en:false` | **定位全链路**（当前实际生效） |
| `velodyne.yaml` | 只有 `/**` | `lidar_type:2`、`scan_line:32`、`/velodyne_points` + `/imu/data`、`extrinsic_T:[0,0,0.28]` | Velodyne HDL-32E 建图。**与 MID-360 不兼容** |
| `velodyne_test.yaml` | 4 块（同 mid360_test 结构） | 同上 + `/global_map_publisher.map_file_path="/home/duzhong/dzpx4/src/maps/map.pcd"` | Velodyne 定位全链路 |
| `avia.yaml` | 只有 `/**` | `scan_line:6`、`blind:4.0`、`fov_degree:90`、`det_range:450`、`extrinsic_T:[0.04165,0.02326,-0.0284]` | Livox Avia。extrinsic 与 MID-360 的 `[-0.011,-0.02329,0.04412]` 明显不同 |
| `horizon.yaml` | 只有 `/**` | Livox Horizon | 非本机雷达 |
| `ouster64.yaml` | 只有 `/**` | Ouster OS1-64 | 非本机雷达 |

### 2.4 MID-360 应该用哪个 / 手册选择是否正确 —— **明确结论**

**结论：运行手册选 `mid360_test.yaml` 是正确的、也是必须的；但 `mid360.yaml` 也不是"错的 MID-360 配置"，而是"不完整"——它缺定位链路，且它的地图路径指向不存在的文件。**

证据链：

1. **型号正确性（两者都对）**：`mid360.yaml:20` 与 `mid360_test.yaml:38` 都是 `lidar_type: 1`。`laserMapping.cpp:1085` 判断 `if (p_pre->lidar_type == AVIA)` 才订阅 `livox_ros_driver2::msg::CustomMsg`，否则订阅 `sensor_msgs::msg::PointCloud2`（`:1092`）。而 `src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py`（`publish_freq=10.0`、`frame_id='livox_frame'`）发布的 `/livox/lidar` 正是 `CustomMsg`。`lidar_type:1` = `AVIA` 枚举 → **MID-360 走对分支**。（若误用 `velodyne*.yaml` 的 `lidar_type:2`，会去订阅 `sensor_msgs/PointCloud2` 类型的 `/velodyne_points`，而无人机上根本没有这个话题 → 直接无输入。）
2. **话题正确性**：`mid360*.yaml` 的 `common.lid_topic=/livox/lidar`、`imu_topic=/livox/imu`；`velodyne*.yaml` 是 `/velodyne_points` + `/imu/data` → 后者在本机上**必然无输入**。
3. **extrinsic 与型号匹配**：`[-0.011, -0.02329, 0.04412]`（`mid360.yaml:34`、`mid360_test.yaml:52`）≈ Livox 官方 MID-360 内置 IMU 相对 LiDAR 的出厂标定（mm 级偏移、单位旋转）。`avia.yaml:32` 的 `[0.04165,0.02326,-0.0284]` 是 Avia 的，**不可混用**。镜像里这份 `mid360.yaml` 是该值的唯一来源，说明作者是针对 MID-360 改的。
4. **只有 `mid360_test.yaml` 含定位链路参数**：`/global_localization`（`:1-8`）、`/transform_fusion`（`:10-12`）、`/global_map_publisher`（`:14-17`）。缺了这三块，`global_localization` 会用**代码内置默认值**（`global_localization.py:30-35`：`map_voxel_size=0.1`、`scan_voxel_size=0.1`、`freq_localization=0.5`、`localization_th=0.9`、`fov_far=100.0`），`global_map_publisher` 会用**上游作者的绝对路径**（`global_map_publisher.py:15` → `/home/myungw00/.../highway2.pcd`，本机不存在）→ **`/global_map` 永不发布 → 定位链路整体静默死亡**。
5. **手册命令本身**：`ros2 launch fast_lio_localization velodyne_localization.launch.py` **不带 `config_file:=`** → 生效值就是 `:33` 的 `mid360_test.yaml`。**所以手册选择正确。** 唯一需要注意：launch 文件名带 `velodyne` 只是历史遗留，与雷达型号无关。

**但 `mid360_test.yaml` 用于"定位"时有 4 个数值是偏保守/偏激进的，值得复核**：

| 参数 | 值 | 位置 | 关注点 |
|---|---|---|---|
| `localization_th` | `0.85` | `mid360_test.yaml:6` | ICP fitness 判据 0.85 很高。`global_localization.py:128` 要求 `fitness > 0.85` 才接受。地图是 5 cm 体素降采样（84387 点）、水平 25×16 m，室内稀疏时很容易低于此值 → 见 §7-R2 |
| `scan_voxel_size` / `map_voxel_size` | `0.18` | `:3-4` | 与 `filter_size_surf/map: 0.18`（`:24-25`）一致，内部自洽；但**地图本身是 `output_voxel_0.05.pcd`（0.05 m 体素）**，配准前再降采样到 0.18，会进一步丢掉特征 |
| `acc_cov` / `gyr_cov` | `0.8` | `:45-46` | 比标准值（0.1/0.1）大 8 倍，是"更信任观测、更不信任 IMU 预测"的调法。若姿态噪声大，会削弱飞控对姿态的信任；**上机前建议先用默认量级对比一次** |
| `pcd_save_en: false` | `:66` | | 与 `mid360.yaml:48` 的 `true` 相反。定位模式下不保存点云是对的（省 IO），但这也意味着**不能靠这次运行产出新地图** |

---

## 3. 接口清单

### 3.1 `fastlio_mapping`（`src/laserMapping.cpp`）

**订阅**

| 话题 | 类型 | QoS | 依据 |
|---|---|---|---|
| `/livox/lidar`（`common.lid_topic`） | `livox_ros_driver2/msg/CustomMsg` | `rclcpp::QoS(20)`（默认 reliable、volatile） | `:1087-1088`（`lidar_type==AVIA` 分支） |
| `/livox/imu`（`common.imu_topic`） | `sensor_msgs/msg/Imu` | `SensorDataQoS()` + `keep_last(500)` | `:1096-1104`，深度常量 `:78` |

**发布**

| 话题 | 类型 | 频率/条件 | frame_id | 依据 |
|---|---|---|---|---|
| `/Odometry` | `nav_msgs/msg/Odometry` | **~50 Hz**（IMU 事件驱动，20 ms 周期） | `camera_init`，child `body` | 发布点 `:1109`、`:1148`；内容 `:780-791`；周期 `:76` `PROP_ODOM_PUBLISH_PERIOD_S=0.02` |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | 每帧（`scan_publish_en`） | **`camera_init`** | `:1105`、`:653`、`:652` |
| `/cloud_registered_body` | `sensor_msgs/msg/PointCloud2` | `scan_bodyframe_pub_en` | **`body`** | `:1106`、`:704` |
| `/cloud_effected` | `sensor_msgs/msg/PointCloud2` | `effect_map_en`（默认 false） | `camera_init` | `:1107`、`:721` |
| `/Laser_map` | `sensor_msgs/msg/PointCloud2` | **1 Hz 定时器**，`map_en` | `camera_init` | 定时器 `:1120-1124`，回调 `:1323-1326`，内容 `:725-743`。注意 `:1290` 同一次调用被注释掉、`:1325` 保留 → 只由定时器发 |
| `/path` | `nav_msgs/msg/Path` | 每 10 帧一次（`jjj%10`） | `camera_init` | `:1110`、`:807-819` |
| TF `camera_init → body` | `tf2` | 同 `/Odometry` | | `:793-804`（`prop_state`，即传播后的状态） |
| 服务 `map_save` | `std_srvs/srv/Trigger` | 按需 | 落盘到 `map_file_path` | `:1126-1131`、`:1328-1342` |

> ⚠️ **`/Laser_map` 与 `/global_map` 不是一张图**：前者是 FAST-LIO 运行中不断增长的增量 ikd-Tree 地图，frame=`camera_init`；后者是静态 PCD，frame=`map`。**不要混用，也不要用 `/Laser_map` 去判断"地图加载成功"**。

> ⚠️ **`map_file_path` 在 C++ 节点里只用于"保存"，不用于"加载"**：`:989` 读入变量，唯一消费点是 `:1330`（保存日志）和 `:755-756`（`save_to_pcd()` 里的 `pcd_writer.writeBinary(map_file_path, ...)`）。**FAST-LIO 本身从不读 PCD 做重定位**——它一开机就从零建局部图（`:1216-1228` 的 `ikdtree.Build`）。全局地图只由 `global_map_publisher.py` 提供。**这是本项目里最容易被误解的一处。**

**参数**：`:946-1015` 共 30 项逐一 `declare_parameter` + `get_parameter_or`。yaml 未给的走默认值。值得注意的默认值偏差：`publish.map_en` 默认 `false`（`:948`）但 yaml 里是 `true`；`publish.path_en` 默认 `true`；`mapping.fov_degree` 默认 `180.`；`preprocess.lidar_type` 默认 `AVIA=1`。

### 3.2 `global_map_publisher.py`（节点名 `global_map_publisher`）

- 订阅：**无**
- 发布：`/global_map` — `sensor_msgs/msg/PointCloud2`，frame_id=`map`（`:44`），queue depth 1（`:30`），**VOLATILE（非 latched / 非 transient_local）**，周期由参数 `interval` 决定（`mid360_test.yaml:17` = 5 s）
- 参数：`map_file_path`（`:15`，默认是上游作者的 `/home/myungw00/...highway2.pcd`）、`interval`（`:16`，默认 5）
- 失败行为：PCD 读不到时只 `error` 一行日志 + 每 5 s `warn` 一次（`:26`、`:37`），**节点保持存活、`/global_map` 永不出现**，无任何"致命"信号

### 3.3 `global_localization.py`（节点名 `global_localization`）

- 订阅：

| 话题 | 类型 | 深度 / QoS | 依据 |
|---|---|---|---|
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | 1 / VOLATILE | `:57` |
| `/Odometry` | `nav_msgs/msg/Odometry` | 1 / VOLATILE | `:58` |
| `/global_map` | `sensor_msgs/msg/PointCloud2` | 1 / VOLATILE，**收到一条就 `destroy_subscription`** | `:59`、`:82` |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 1 / VOLATILE | `:60-65` |

- 发布：

| 话题 | 类型 | 频率 | frame_id | 依据 |
|---|---|---|---|---|
| `/map_to_odom` | `nav_msgs/msg/Odometry` | 2 Hz（`freq_localization`） | `map`（`:137`） | `:54`、`:138` |
| `/cur_scan_in_map` | `sensor_msgs/msg/PointCloud2` | 每帧 `/cloud_registered` | **`camera_init`**（`:106`） | `:52`、`:105-108` |
| `/submap` | `sensor_msgs/msg/PointCloud2` | 每次 ICP | `map`（`:164`） | `:53`、`:162-166` |

> ⚠️ `/cur_scan_in_map` **名不副实**：名字暗示"地图坐标系下的当前扫描"，实际是把 `/cloud_registered` 原样改 frame_id 后转发（`:106` 强制改为 `camera_init`），**没有任何 `map` 变换**。RViz 里把它和 `/global_map` 叠在一起看会得到"看起来对不上"的错觉。

> ⚠️ `/map_to_odom` 的 `header.stamp` 取的是 `self.cur_odom.header.stamp`（`:136`），若此刻还没收到过 `/Odometry`，`self.cur_odom` 为 `None` → `AttributeError` 使回调抛异常。实践中 `/Odometry` 早于 `/initialpose`，但**顺序假设是隐含的**。

- 参数：`map_voxel_size`(0.1) `scan_voxel_size`(0.1) `freq_localization`(0.5) `localization_th`(0.9) `fov`(2π) `fov_far`(100.0)，全部 `declare_parameter` 带默认（`:30-42`），可被 yaml 覆盖
- **QoS 硬编码**：所有 pub/sub 都用整型 depth `1`（如 `:52-65`），无 `rclpy.qos.QoSProfile` 定制 → 全部 RELIABLE + VOLATILE。**在代码/launch 里都看不到 QoS 可配项**。

### 3.4 `transform_fusion.py`（节点名 `transform_fusion`）

- 订阅：`/Odometry`（depth 1，`:27`）、`/map_to_odom`（depth 1，`:28`）
- 发布：`/localization` — `nav_msgs/msg/Odometry`（depth 1，`:31`），**`frame_id="map"`、`child_frame_id="body"`**（`:80-81`），`header.stamp` 原样继承 `/Odometry`（`:79`），`twist` 原样复制（`:86`），**`pose.covariance` / `twist.covariance` 全零（未填）**
- TF：`map → camera_init`（`:60-70`），**时间戳用的是 `self.get_clock().now()`（`:60`）而不是 `/Odometry` 的时间戳** → 与 `fastlio_mapping` 广播的 `camera_init→body`（用 `sample.timestamp`，`:783`）**时间基准不一致**
- 参数：**无**（`freq_pub_localization` 是死参数）
- ⚠️ 首帧即发布：`map2odom is None` 时用 `eye(4)`（`:54`）→ **在重定位成功前，`/localization` 就已经在发了，且内容是"map 坐标系下未修正的 LIO 位姿"**。下游（`px4_odom_convert`）没有"是否已重定位"的判据可看。这是 §7-R3 的一部分。

### 3.5 `publish_initial_pose.py`

- 发布：`/initialpose`（depth 10，`:11`），`frame_id="map"`（`:21`）
- 参数：无（命令行 6 个位置参数 `x y z yaw pitch roll`，`:29-34`）
- 行为：`publish_pose()` 在构造函数里调用 → 立即发一条 → `spin_once(timeout_sec=1.0)` → 退出（`:37-42`）。**VOLATILE，非 latched，发完即走。**

### 3.6 星型汇总：谁需要谁

| 消费方 | 需要的话题 | 帧 | 类型 |
|---|---|---|---|
| **`px4_odom_convert`（同事负责，此处只确认其输入）** | **`/localization`** | **`map` → `body`** | **`nav_msgs/msg/Odometry`** |
| `px4_path`（`slam_topic`） | `/localization` | — | `nav_msgs/msg/Odometry`（`fc-audit/src-mirror/px4_path/px4_path_README_DELIVERY.md:125`） |
| RViz（`rviz/fastlio_localization.rviz`） | `/global_map` `/cur_scan_in_map` `/localization` `/submap`，Fixed Frame=`map`（`:194`） | | |

`px4_odom_convert` 侧的证据：`fc-audit/src-mirror/px4_odom_convert/src/px4_odom_convert.cpp:14` 默认 `localization_topic="/localization"`；`:42` 用 `nav_msgs::msg::Odometry` 订阅；`README.md:92` 明示"定位输出：`/localization`（`nav_msgs/Odometry`），频率 50 Hz"；`:89` 的命令就是本手册那条 launch。**输入契约明确：只管 `/localization` 一个话题，不查 TF、不查任何健康标志。**

---

## 4. 坐标系约定

### 4.1 三个 frame 的定义

| frame | 定义 | 依据 |
|---|---|---|
| `body` | **IMU body**。FAST-LIO 的 `state.pos/rot` 是 IMU 的位置/姿态，`/Odometry` 的 position 直接取 `prop_state.pos`（`:784-786`） | `:781-782`、`:784-790` |
| `camera_init` | **FAST-LIO 启动瞬间的 `body` 帧**（等价于"t0 时刻的 IMU 姿态"），**不是** `livox_frame` | 见 4.2 |
| `map` | 静态 PCD 地图帧（`global_map_publisher.py:44`）；`global_localization` 求的 `T_map_to_odom` 把 `camera_init` 对齐到它 | `:44`、`:137`、`transform_fusion.py:61-70` |

`/Odometry.header.frame_id="camera_init"`、`child_frame_id="body"`（`:781-782`）；`/localization` 是 `map`→`body`（`transform_fusion.py:80-81`）。TF 链：**`map` --(`transform_fusion`)--> `camera_init` --(`fastlio_mapping`)--> `body`**（`transform_fusion.py:61-62`、`laserMapping.cpp:794-796`）。

### 4.2 是否有重力对齐？—— **有重力估计，但初始姿态是单位阵，即"不对齐偏航"**

`src/IMU_Processing.hpp:197-204`：

```cpp
state_ikfom init_state = kf_state.get_x();
init_state.grav = S2(- mean_acc / mean_acc.norm() * G_m_s2);   // :198  ← 重力在 body 系下的方向
//state_inout.rot = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));  // :200  ← 对齐那行被注释掉了
init_state.bg  = mean_gyr;
...
kf_state.change_x(init_state);                                  // :204
```

- 重力向量按平均加速度估计（`:198`），**所以世界系有"上"的概念**；
- 但 `init_state.rot` **从未被设置**（`:200` 的对齐被注释掉），保持 `kf_state.get_x()` 的初值 = 单位阵 → **`camera_init` 的 X/Y 轴就是 t0 时刻 IMU 的 X/Y 轴，偏航完全未定义/未对齐任何地理方向**。
- 这不是本仓库的改动，是上游 FAST-LIO 原样行为。

**→ 输出既不是 ENU 也不是 NED。** 它是"局部、Z 向上、偏航任意"的世界系。要让 PX4 EKF2 用，必须由下游显式给一个偏航旋转。而 `px4_odom_convert` 的做法是**写死一个 180° 偏航**：

`fc-audit/src-mirror/px4_odom_convert/src/px4_odom_convert.cpp:20-28`：

```cpp
R_enu_slam_ << -1.0, 0.0, 0.0,
                0.0,-1.0, 0.0,
                0.0, 0.0, 1.0;      // = Rz(180°)
t_enu_slam_ << 0.0, 0.0, 0.0;
```

即它认为"`/localization` 是 slam 帧，把它视作 ENU 前先转 180° 偏航"（`:127-129` 得 `position_enu`，`:142` 得 `R_enu_body`），再 `enu_to_ned`（`:151-153`）。**这个 `-1,-1,1` 意味着：`map` 帧的 +X 必须恰好与真实东向相差 180°。** 如果建图时机器人的起始朝向不是"朝西"，或 `map` 帧由别的工具重定义过，**外部视觉给 EKF2 的偏航就是错的**，而 EKF2 会把它当作绝对航向融合 → 起飞后自主飞行直接朝错方向。见 §7-R3。

### 4.3 `livox_frame` 的关系 —— **无 TF 连接**

- 驱动侧：`src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py` 设 `frame_id = 'livox_frame'`（`:13`、`:28`），`publish_freq = 10.0`（`:11`）。所以 `/livox/lidar`、`/livox/imu` 的 `header.frame_id` 都是 `livox_frame`。
- 本包侧：`fastlio_mapping` **完全不读输入消息的 frame_id**（订阅回调只取 `points`/`time`/IMU 数值），并统一按 `camera_init`/`body` 命名输出。
- → **TF 树里没有任何 `body ↔ livox_frame` 或 `camera_init ↔ livox_frame` 变换**。任何以 `livox_frame` 为父去查 TF 的下游都会失败。若 RViz 的 Fixed Frame 被设成 `livox_frame`，会一直报 "No transform"。
- 语义上的对应关系是：`body` ≡ MID-360 内置 IMU 帧，`livox_frame` ≡ LiDAR 光学帧，二者差一个 `extrinsic_T/R`（`mid360_test.yaml:52-55`，≈ mm 级平移 + 单位旋转）。**这个差值没有被发布成 TF**——即 LiDAR 与"定位输出所代表的机体位置"之间的小偏移在 TF 层不可见（在 `/localization` 的数值里是被算进去的，因为 FAST-LIO 的状态就是 IMU 状态）。

### 4.4 原点在哪

- `map` 帧原点是**建图那次运行 `camera_init` 的原点**（`global_map_publisher.py` 只是把 PCD 原样发出，不做任何变换，`:39-54`）。
- 实测 `src/maps/output_voxel_0.05.pcd`（84387 点，`FIELDS x y z`，`binary`，12 字节/点）：`x ∈ [-11.49, 13.79]`、`y ∈ [-11.75, 4.19]`、`z ∈ [-0.23, 2.57]`；**`z ∈ [2.17, 2.37]` 一个 bin 就占了 43702 点（51.8%），`z > 2` 占 59.8%，`z < 1` 仅 6.7%**。`map.pcd`（314814 点）范围几乎相同。
- 解读：**Z 轴确实是竖直向上**（否则不会出现这么集中的水平面），该水平面 z≈2.27 极可能是**天花板**，z≈0 附近是地面。→ 原点在**地面高度附近**，与"无人机放地面开机"一致。
- **未确认**：`map` 帧的偏航朝向（无法从 PCD 数值判断），以及 `output_voxel_0.05.pcd` 是否为当前使用的地图（`mid360_test.yaml:16/29` 指向它，且文件存在）。**这两点必须上机用 `/initialpose` + ICP fitness 实测确认。**
- **地图的水平覆盖只有约 25 m × 16 m 且严重不对称**（X 直方图 78% 的点在 x>7，Y 在 -11.7…4.2）。`mid360_test.yaml:8` 的 `fov_far=25.0` 相对这个尺度是合理的，但**一旦飞出这块区域，ICP 必然失败**（且失败是静默的，见 §7-R2）。

---

## 5. 地图加载与重定位链路

### 5.1 PCD 从哪加载

1. `global_map_publisher.py:15-28` — `map_file_path` 指向的 PCD 由 **Open3D** 读入（`:23` `o3d.io.read_point_cloud`）。
   - 生效值：`mid360_test.yaml:16` = `/home/duzhong/dzpx4/src/maps/output_voxel_0.05.pcd`。
   - 该路径在无人机上是否真实存在：**本次未确认**（未 SSH）。本地镜像 `src/maps/` 下确有同名文件（1012816 B / 84387 点），但镜像目录 ≠ 无人机目录，**必须上机核对一次**。
   - 注意 `mid360_test.yaml:29` 在 `/**` 块里又写了一遍同一路径 → 双份配置（§2.2）。
2. `global_map_publisher.py:30-31` — 每 5 s 发一次 `/global_map`。
3. `global_localization.py:76-82` — 收到第一条即用 `voxel_down_sample(map_voxel_size=0.18)` 降采样存为 `self.global_map`，并 `destroy_subscription`（不再接收）。
4. `fastlio_mapping` 自身**不加载任何 PCD**（`map_file_path` 仅用于保存，见 §3.1）。

### 5.2 全局配准用什么 —— **Open3D Point-to-Point ICP + 人工先验位姿，两级尺度**

- 算法：`global_localization.py:170-179` — `o3d.pipelines.registration.registration_icp(..., TransformationEstimationPointToPoint(), ICPConvergenceCriteria(max_iteration=20))`。**Point-to-Point**（不是 Point-to-Plane），对 0.18 m 稀疏体素不算理想。
- 两级尺度：`global_localization.py:124-125` — 先 `scale=5`（体素 ×5，`max_correspondence_distance=5.0`）粗配准，再 `scale=1`（距离 1.0 m）精配准。
- 先验位姿：**必须人工给**，来自 `/initialpose`（`:94` `initial = self.pose_to_mat(msg)`）。
- FOV 裁剪：`:144-168` — 用当前位姿把整张地图变换到扫描系，取 `x < fov_far`（全周时）的子图参与配准，并顺便把子图发到 `/submap`。
- 接受判据：`:128` `if fitness > self.localization_th`（生效 0.85）。**不通过就只打一行 warn（`:141`），什么都不更新。**

### 5.3 `transform_fusion.py` 如何把局部 LIO 与全局地图对齐

`transform_fusion.py:52-87`，每次收到 `/Odometry`：

```
T_map_to_odom = pose_to_mat(latest /map_to_odom)     # :54，无则单位阵
T_odom_to_base = pose_to_mat(当前 /Odometry)          # :73
T_map_to_base  = T_map_to_odom @ T_odom_to_base      # :74
→ /localization (map→body)                            # :78-87
→ TF map→camera_init = T_map_to_odom                  # :59-70
```

其中 `/map_to_odom` 就是 ICP 直接算出来的 `T`（`global_localization.py:129` `self.T_map_to_odom = T`，`:138` 发布）。

⚠️ **命名与语义的混淆**：`/Odometry` 的 `frame_id` 是 `camera_init`（不是 `odom`），但这一环被叫做 `T_map_to_odom`。数学上自洽（`camera_init` 就是"局部里程计世界系"），**但配置/文档里出现 "odom" 字样时不要去找一个叫 `odom` 的真实 frame——TF 树里没有 `odom`**。

⚠️ **全局修正只在 ICP 成功后生效，且是"跳变式"的**：`transform_fusion.py:41-43` 与 `:36-39` 之间无插值、无平滑，`/map_to_odom` 一更新，下一条 `/localization` 就整体跳一个 `T`。对 EKF2 而言这是一次位置/姿态阶跃。

### 5.4 `publish_initial_pose.py` 的用途

- 唯一用途：**给 `global_localization` 提供 ICP 先验位姿**，是整条重定位链路的"人工扳机"。
- 用法（`publish_initial_pose.py:29-34` 的 6 个位置参数按 `x y z yaw pitch roll`）：
  ```bash
  ros2 run fast_lio_localization publish_initial_pose.py <x> <y> <z> <yaw> <pitch> <roll>
  ```
  （四元数由 `tf_transformations.quaternion_from_euler(roll, pitch, yaw)` 生成，`:16`）
- 也可由 RViz 的 `SetInitialPose` 工具代替：`rviz/fastlio_localization.rviz:205-214` 就配了该工具，`Topic: /initialpose`。**这也是无人机上更现实的做法**——但需要 VNC 里有人手动点。

---

## 6. `MODIFICATIONS.md` 逐条转述与接口影响判断

原文 `MODIFICATIONS.md`（69 行，标题"高频 Odometry 与 Localization 修改说明"），共 7 组内容。逐条：

| # | 原文要点 | 我方核验（代码位置） | 是否影响对外接口 |
|---|---|---|---|
| 1 | 目标：LiDAR 10 Hz 时两帧之间用**全部 IMU** 传播状态 | `laserMapping.cpp:499-504` 每个 IMU 回调都 `predict_prop_kf_with_imu_pair_locked` | 否（内部机制） |
| 2 | IMU 200 Hz 时按真实观测时间选择**约 50 Hz** 发布 `/Odometry` | `:76` `PROP_ODOM_PUBLISH_PERIOD_S=0.02`；判据 `:508-517`（按 `prop_last_time` 差，非墙钟） | **是**：`/Odometry` 从"每帧雷达一次（10 Hz）"变成"~50 Hz IMU 事件驱动"。频率变了，但**话题名/类型/frame 未变** |
| 3 | **不重复发布缓存帧，不按墙钟补发历史帧** | `:512-517` 只在 `first_sample \|\| publish_period_elapsed` 时置 `sample_ready`；`:517` 后无重发路径；长间隔恢复后只发最新（`MODIFICATIONS.md:29-31` 亦如此声明） | **是（行为契约）**：下游不能假设"没消息=没在动"，也不能期待固定节拍 |
| 4 | `/localization` 跟随新的 `/Odometry` 发布 | `transform_fusion.py:36-39`（每收到 `/Odometry` 就发一条），无独立定时器 | **是**：`/localization` 的频率与 `/Odometry` 绑定 |
| 5 | 全局 ICP 的频率和 `map→odom` 修正逻辑**保持不变** | `global_localization.py` 未被本次改造触及（仍 2 Hz 定时器 + `fitness>th` 才更新） | 否 |
| 6 | 并发：**三线程 `MultiThreadedExecutor`**；IMU / LiDAR 输入 / 映射处理各自独立互斥回调组；**IMU 订阅深度 500**；`prop_kf`、传播时间、IMU 重放历史由独立互斥锁保护；主 EKF 不长期占用传播锁 | `laserMapping.cpp:1383-1384`（`MultiThreadedExecutor(ExecutorOptions(), 3)`）；`:1075-1084` 三个 `MutuallyExclusive` 回调组；`:78` `IMU_SUBSCRIPTION_DEPTH=500`、`:1097` `imu_qos.keep_last(500)`；`:458-479` / `:483-518` 两把锁 | 否（但**提高了 CPU 占用**，Jetson 上值得留意） |
| 7 | 时间戳与复位：`/Odometry.header.stamp` = 发布状态对应的 **IMU 观测时间**；`/localization.header.stamp` **原样继承**；IMU 时间戳回退时清空主 IMU 缓冲/传播历史/发布去重；雷达校正后先把 `prop_kf` 重放到最新 IMU 时刻 | `/Odometry` stamp：`:783` `get_ros_time(sample.timestamp)`；`/localization` stamp：`transform_fusion.py:79`；回退清空：`:470-475`、`:486-492`；重放：`:1269-1270` `reset_and_replay_prop_kf(lidar_end_time)` | **是（关键）**：`/Odometry` 与 `/localization` 的时间戳**不再是雷达帧时间，而是 IMU 观测时间**。任何"用雷达时间戳对齐"的外部逻辑都会错位最多一个 IMU 周期（5 ms）；更要注意 `/localization` 的 stamp 是 IMU 时间而非"信号发布时刻" |

**附**：`MODIFICATIONS.md:60-69` 要求每秒打印 `Propagation rates: imu=... live_predict=... replay=... odom=... invalid_gap=...`。已核验存在于 `laserMapping.cpp:1162-1169`，字段顺序与文档一致（`imu / live_predict / replay / odom / invalid_gap`）。**这是当前唯一可用的 LIO 侧健康指示**，但只在终端打印，不发话题。

---

## 7. 风险清单

> 格式：**现象 → 触发条件 → 后果 → 建议修法**。`文件:行号` 相对本镜像副本。

### R1 —【严重】`/initialpose` 是 VOLATILE 且只发一次，首发竞态不可自愈

- **现象**：`publish_initial_pose.py` 构造即发、发完退出（`scripts/publish_initial_pose.py:12`、`:40-42`）；发布端 QoS 深度 10、**VOLATILE（非 latched / 非 transient_local）**（`:11`）；订阅端同样 VOLATILE 深度 1（`global_localization.py:60-65`）。而 `cb_init_pose` 开头就 `if self.initialized: return`（`:88-89`），**且该回调里没有任何"重试"路径**。
- **触发条件**：(a) 在 `global_localization` 起来之前执行 `publish_initial_pose.py`；或 (b) 第一条 `/initialpose` 到达时 `/global_map` 还没到（`:85-87` `global_map is None` 直接 return）或首帧 `/cloud_registered` 还没到（`:90-92` 直接 return）。
- **后果**：那条消息被**永久丢弃**。此后无论再发多少次 `/initialpose`……**其实是可以再触发的**（因为 `initialized` 仍是 `False`，回调会继续处理）——但**没人知道需要重发**，因为日志里只有一条 warn，节点看起来"活着"。运维会以为"已经初始化过了"，而实际上 `global_localization` 从未成功、`/map_to_odom` 从未发布、`transform_fusion` 一直用 `eye(4)` 合成 `/localization`（`transform_fusion.py:54`），**`/localization` 照常以 50 Hz 输出"看起来正常"的数据，只是没有全局修正**。飞控拿到的是纯 LIO 里程计。
- **建议修法**：(1) 把 `/initialpose` 的 QoS 改成 `transient_local`（发布端 depth 1 + TRANSIENT_LOCAL，订阅端同），或在 `global_localization` 里改为"缓存最后一条 `/initialpose`，等地图+扫描就绪后自动重试"；(2) `cb_init_pose` 增加重试与限流日志（例如每次拒绝都 warn 且打上计数）；(3) 关键：**发布一个明确的"重定位已就绪"标志**（见 R3）。

### R2 —【严重】ICP 失败是静默的：无标志、无降级、无超时，失败即无限沿用陈旧修正

- **现象**：`global_localization.py:128-142` — `if fitness > localization_th` 才更新 `T_map_to_odom`；否则只 `warn('Global localization failed (fitness below threshold).')` 并 `return False`。**没有任何计数器、没有阈值告警升级、没有"多久没成功就报错"的看门狗。** 生效阈值 `mid360_test.yaml:6` = `0.85`（偏高）。同时 `transform_fusion.py` **不检查任何新鲜度**，无条件用当前 `cur_map_to_odom` 合成（`:38`、`:54`）。
- **触发条件**：任一 —— 初始先验位姿给偏（`:94` 完全信任人工输入）；机器移出地图覆盖（实测地图水平仅约 25 m × 16 m）；场景几何退化（长走廊/空旷）；`fov_far=25.0`（`:8`）截出的子图点太少；`scan_voxel_size/map_voxel_size=0.18`（`:3-4`）降采样过度。
- **后果**：**ICP 失败后系统静默退化为"纯 LIO + 陈旧全局修正"**。若失败发生在起飞前，`/localization` 与真实地图位置存在固定偏移 → EKF2 把错误位置当绝对观测融合 → **定位漂移被"钉死"在错误位置，飞控按错误位置飞行**。若在飞行中失败，LIO 漂移无界增长且无任何提示。**这是"飞控拿到错误定位会直接飞丢"的直接成因。**
- **建议修法**：(1) 发布健康标志：`/localization_valid`（`std_msgs/Bool`，或把 `/localization.pose.covariance` 对角线按 fitness 写成"很大/很小"）——`/localization` 当前 covariance **全零**（`transform_fusion.py` 未填任何 covariance，`:78-87`），下游无从判断可信度；(2) `transform_fusion` 增加超时：`now - last_icp_success > T` 则停止发布或把 covariance 置为"不可信"；(3) 降低 `localization_th` 到实测可达区间，并用真实数据先跑一次 fitness 分布；(4) `global_localization` 每次拒绝都累加计数并周期打印。

### R3 —【严重】输出帧不是 ENU；偏航修正被下游写死为 180°，无任何校验手段

- **现象**：`IMU_Processing.hpp:197-204` 中重力对齐那行被注释（`:200`），`init_state.rot` 保持单位阵 → `camera_init` 的偏航未对齐任何地理方向（§4.2）。`/localization` 直接继承该朝向（`transform_fusion.py:74-85`）。下游 `px4_odom_convert` 用**写死的** `R_enu_slam_ = diag(-1,-1,1)`（`fc-audit/src-mirror/px4_odom_convert/src/px4_odom_convert.cpp:20-23`）把它当作 SLAM→ENU 的变换，再转 NED（`:151-153`）。
- **触发条件**：`map` 帧的真实偏航 ≠ 180°。即：建图时的起始朝向不是"朝西"，或地图被外部工具（CloudCompare / PGO / HBA）重定义过朝向，或 `map` 的 +X 与地理东向无固定关系。
- **后果**：外部视觉给 PX4 EKF2 的**绝对航向系统性错误**。EKF2 会融合这个错误航向 → 无人机按错误机头方向理解"前/后"，**起飞后按目标点飞行时朝错方向，且不会自愈**（因为融合的是"绝对"观测）。同时 `/localization` 的 covariance 全零，地面站也看不出异常。此外 `extrinsic_T`（LiDAR↔IMU 的 mm 级偏移，`mid360_test.yaml:52`）与 LiDAR 安装高度（`px4_odom_convert` 的 `z_offset_m`，`README.md:125`）是**两个独立参数**，容易只调一个。
- **建议修法**：(1) 建图时用一致、可复现的初始朝向，并把该朝向**记录在案**（例如"开机机头朝正西，故需 180° 修正"）；(2) 把 `R_enu_slam_` 从 `px4_odom_convert.cpp:20-23` 的硬编码改成 ROS 参数（这样不改代码就能纠正）；(3) 加一次**地面静态校验**：把无人机放在地图里已知点、已知朝向上，比较 `/localization` 与真值的 yaw 偏差，确认 `diag(-1,-1,1)` 是否成立；(4) 在校验通过前禁止解锁。

### R4 —【高】硬编码绝对路径，且 `mid360.yaml` 指向不存在的文件

- **现象**：
  - `config/mid360.yaml:10` = `"/home/duzhong/dzpx4/src/maps/buliding_a_floor1.pcd"`（镜像版；上游是 `"./test.pcd"`）。该文件在 `src/maps/` 下**不存在**（实际只有 `map.pcd` / `map_compile_4cm.pcd` / `output_voxel_0.05.pcd`）。
  - `config/mid360_test.yaml:16` 与 `:29` = `"/home/duzhong/dzpx4/src/maps/output_voxel_0.05.pcd"`（同一路径写两遍，且是绝对路径 + 特定用户名）。
  - `config/velodyne_test.yaml:16` = `"/home/duzhong/dzpx4/src/maps/map.pcd"`。
  - `scripts/global_map_publisher.py:15` 默认值 = `'/home/myungw00/ROS2/localization_ws/src/FAST_LIO_LOCALIZATION_ROS2/PCD/highway2.pcd'`（**上游作者主机的路径**，本机绝不存在）。
  - `src/laserMapping.cpp:96` `root_dir = ROOT_DIR`，而 `CMakeLists.txt:12` 把 `ROOT_DIR` 定义为 `${CMAKE_CURRENT_SOURCE_DIR}/`（**源码目录**）→ 落盘位置是源码树里的 `PCD/`、`Log/`（`:679`、`:1060`、`:1397`），与运行时的 `install/` 空间分离。这也解释了 `map.pcd` / `output_voxel_0.05.pcd` 为何出现在 `src/maps/`。
- **触发条件**：改用 `mid360.yaml`（例如按 README `README.md:237` 的建议"换 config 就换 radar"）；或换机、换用户名、换工作空间路径；或 yaml 未覆盖 `map_file_path` 而落到 `global_map_publisher.py:15` 的默认值。
- **后果**：
  - 用 `mid360.yaml` 启动时，`/global_map_publisher` 那段参数**不存在** → `map_file_path` 落到 `global_map_publisher.py:15` 的 `/home/myungw00/...highway2.pcd` → Open3D 报错（`global_map_publisher.py:26`）→ **`/global_map` 永不发布** → `global_localization` 永远 `global_map is None`（`:85-87`）→ **整条重定位链路静默死亡**，而 `/localization` 仍在 50 Hz 输出未修正位姿（同 R1 的恶化路径）。
  - 反之，若有人"修好"了 `mid360.yaml:10` 的路径，`fastlio_mapping` 的 `map_file_path` 会在 `map_save` 服务被调用时（`:1330-1333`）**覆盖该文件**——把一个只用于"加载"的地图路径同时用作"保存"目标是危险的（虽然需要主动调服务才触发）。
  - `CMakeLists.txt:12` 的 `ROOT_DIR` 指向源码目录，安装后若源码目录被删或只读，落盘会失败。
- **建议修法**：(1) 把地图路径统一成**单一来源**：只保留 `/global_map_publisher.map_file_path`，删掉 `/**` 里那份，并改用启动参数（`ros2 launch ... map_file_path:=...`）或环境变量，而不是写死在 yaml；(2) 给 `global_map_publisher.py` 加"文件不存在即 FATAL 退出/持续 error 级告警"的行为，而不是 warn 后静默存活（`:26`、`:37`）；(3) `mid360.yaml:10` 改成 `""` 或删除该行，避免指向不存在文件；(4) `CMakeLists.txt:12` 的 `ROOT_DIR` 改为可运行时配置的输出目录参数。

### R5 —【高】参数双份配置 + 死参数，配置面不可信

- **现象**：`config/mid360_test.yaml:16`（`/global_map_publisher.map_file_path`）与 `:29`（`/**` 的 `map_file_path`）值相同但语义重叠；`:10-12` 的 `/transform_fusion.freq_pub_localization` 在该节点**根本没有被声明**（`scripts/transform_fusion.py` 全文无 `declare_parameter`）→ 静默忽略；`README.md:85` 却把它描述成"Fused pose publish rate"。
- **触发条件**：任何人试图"通过改 yaml 调频率/换地图"。改 `freq_pub_localization` 无任何效果；改两份 `map_file_path` 中的一份可能与另一份冲突（ROS 2 对"精确节点名 vs `/**` 通配"在同一参数文件内的优先级**没有可依赖的规范**，行为依赖版本）。
- **后果**：运维按文档改参数却看不到变化 → 误判为"节点没读到配置" → 进一步乱改，甚至误以为地图换了其实没换。**配置面失去可信度，是排查 R1/R2/R4 时最大的干扰源。**
- **建议修法**：(1) 删掉 `mid360_test.yaml:29` 那行（保留精确节点名那份）；(2) 要么在 `transform_fusion.py` 里真的声明并实现 `freq_pub_localization`，要么从 yaml 和 README 里删掉它；(3) 启动时把**实际生效**的关键参数打印一遍（`map_file_path`、`localization_th`、`freq_localization`、`lidar_type`、两个 topic 名），让日志成为唯一真相。

### R6 —【高】重依赖：Open3D / `tf_transformations` / PyTorch 类栈，且 `package.xml` 未声明

- **现象**：`scripts/global_localization.py:7` `import open3d as o3d`、`:11` `import tf_transformations`、`:14` `from sensor_msgs_py import point_cloud2`；`scripts/global_map_publisher.py:4` `import open3d`；`scripts/publish_initial_pose.py:5` `import tf_transformations`。但 `package.xml` 里**只有 C++ 依赖**（`package.xml:21-30`：`geometry_msgs nav_msgs rclcpp std_msgs sensor_msgs common_interfaces tf2 pcl_ros pcl_conversions livox_ros_driver2`），**没有任何 `exec_depend` 指向 `python3-open3d` / `python3-numpy` / `tf_transformations` / `sensor_msgs_py`**。`CMakeLists.txt:47` 还 `find_package(PythonLibs REQUIRED)`、`:100-102` 把 Python 库链进 `fastlio_mapping`，`:48` 找 `matplotlibcpp.h`。
- **触发条件**：干净环境 `rosdep install` + `colcon build`（README `README.md:60`、`:63` 教的就是这套）；或在 Jetson（aarch64）上装 Open3D——**Open3D 官方 pip wheel 长期不提供 aarch64，Jetson 上通常需要源码编译或第三方轮子**。
- **后果**：3 个 python 节点在 `import` 阶段就 `ModuleNotFoundError` 崩溃，`/global_map`、`/initialpose` 处理、`/map_to_odom` 全部不存在 → **重定位链路整个不工作，但 `fastlio_mapping` 照常跑、`/localization` 照常输出**（因为 `transform_fusion.py` 只依赖 numpy + tf_transformations，会以 `eye(4)` 模式正常运行）。**故障表现与"正常"几乎一样。** 另外 `scripts/__pycache__/` 里有 `global_localization.cpython-311.pyc` 与 `transform_fusion.cpython-311.pyc` → 说明这两个脚本**曾在 Python 3.11 环境被成功 import 过**（Humble 默认 3.10，3.11 值得注意），可作为"曾经装通过"的证据。
- **建议修法**：(1) `package.xml` 补齐 `exec_depend`：`python3-numpy`、`python3-open3d`（或明确记录非 rosdep 的安装方式）、`tf_transformations`、`sensor_msgs_py`（Humble 里在 `ros-humble-sensor-msgs-py`）；(2) 在 README 里给出 **Jetson/aarch64 的 Open3D 安装步骤**（这是本项目最容易卡住的一步）；(3) 在 3 个脚本的 `import` 外做 try/except 并给出明确的中文错误提示，而不是裸 traceback；(4) 确认飞控机上 python 版本（3.10 还是 3.11）与 `__pycache__` 一致。

### R7 —【中】`config_file` 默认值硬编码成 `mid360_test.yaml`，切雷达需要改代码

- **现象**：`velodyne_localization.launch.py:33` 与 `mapping.launch.py:34` 都把 `config_file` 默认值写成 `mid360_test.yaml`；`README.md:26` 声称示例配置是 `velodyne_test.yaml`，`README.md:95-97` 的示例命令又显式传 `config_file:=velodyne_test.yaml`。
- **触发条件**：换雷达型号（Avia/Velodyne/Ouster）时忘了传 `config_file:=`。
- **后果**：用 MID-360 的 extrinsic（`extrinsic_T`/`extrinsic_R`、`det_range`、`blind`）去跑别的雷达，或反之 → LIO 精度崩坏但不报错。当前无人机是 MID-360，所以**这个默认值是对的**；风险在于"默认值悄悄变成了唯一真相"，而 `README.md` 里的描述与它有冲突。
- **建议修法**：README 按实际默认值更正；`config_file` 与雷达型号绑定关系写进文档；或引入 `lidar_model:=mid360` 之类的上层参数做间接映射。

### R8 —【中】RViz 默认配置路径拼写错误，`-d` 指向不存在的文件

- **现象**：`launch/velodyne_localization.launch.py:16` = `os.path.join(package_path, 'rviz', 'fastlio_localiztion.rviz')` —— **`localiztion` 少了一个 `a`**。实测 `rviz/` 下只有 `fastlio.rviz` 和 `fastlio_localization.rviz`，**`fastlio_localiztion.rviz` 不存在**（`Test-Path` 为 False）。`mapping.launch.py:17` 用的 `fastlio.rviz` 则是存在的。
- **触发条件**：直接跑手册命令（默认 `rviz:=true`），且没有传 `rviz_cfg:=`。
- **后果**：`rviz2 -d <不存在的路径>` → RViz 报错并**要么直接退出、要么用空配置启动**（取决于版本）。因为 `rviz: false` 可以规避，且不影响定位链路本身，故定级为中；但它会让"按手册操作 → RViz 打不开 → 以为整个包坏了"成为一个常见的第一印象。
- **建议修法**：把 `:16` 改成 `fastlio_localization.rviz`。

### R9 —【中】`cb_init_pose` 的一次性门闩与 `cur_odom` 隐式非空假设

- **现象**：`global_localization.py:84-100` — `cb_init_pose` 内 `if self.initialized: return`（`:88-89`），所以**成功后不再接受任何新的先验位姿**；而 `global_localization()` 的失败分支（`:141-142`）不会重置任何状态，`initialized` 保持 `False`、定时器也不会创建（`:99`）。另一处：`:136` 用 `self.cur_odom.header.stamp`，但 `cb_save_cur_odom`（`:102-103`）之前 `cur_odom` 为 `None`；`:90-92` 只检查了 `cur_scan`，**没检查 `cur_odom`**。
- **触发条件**：(a) 想"重新初始化一次"（例如无人机被搬动了）——没有接口，只能重启整个 launch；(b) ICP 第一次成功后 `T_map_to_odom` 被固定为定时器输入（`:116`），若之后漂移出去就只剩 2 Hz 的局部 ICP 自我维持；(c) 极端时序下 `/Odometry` 第一条晚于 `/initialpose` 触发成功路径，`:136` 抛 `AttributeError`。
- **后果**：现场无法重新对齐；`AttributeError` 抛在 `rclpy` 回调里会打印 traceback，**不一定会终止进程**（`rclpy.spin` 默认对回调异常的处理是记录并继续，行为随版本变化）→ 可能表现为"节点活着但 `T_map_to_odom` 永不更新"。
- **建议修法**：(1) 提供"重新初始化"服务或话题（例如收到新的 `/initialpose` 就无条件重跑一次 ICP）；(2) `cb_init_pose` 里补 `if self.cur_odom is None: warn + return`；(3) 明确失败后的重试节拍。

### R10 —【中】大量 Python 侧热点：全图逐点循环、深度 1 队列、无 OpenMP 上限

- **现象**：
  - `global_localization.py:69-74` `pc2_to_array` 用 **Python 逐点 `for` 循环** 把 `PointCloud2` 转 numpy；`:57` 每帧 `/cloud_registered` 都调一次（10 Hz）。`/cloud_registered` 在 `dense_publish_en:false`（`mid360_test.yaml:62`）下是降采样后的 `feats_down_body`（`laserMapping.cpp:637`），但仍是**每帧数千到上万点**。
  - `global_localization.py:144-158` `crop_global_map_in_FOV` 对 **84387 点的整张地图**做 4×4 齐次矩阵乘法（每 2 s 一次），加上 `:165` 的 `subpts[::10].tolist()` 再构造消息。
  - 订阅深度普遍为 1（`global_localization.py:57-65`、`transform_fusion.py:27-28`、`:31`）→ 一旦某次回调慢于消息间隔，**消息被静默丢弃**（`/Odometry` 丢帧会让 `transform_fusion` 少发 `/localization` 对应帧）。
  - `laserMapping.cpp:1383-1384` 起 3 线程执行器、`:1075-1078` 三个回调组，加上 `laserMapping.cpp:29-30` 的 OpenMP 并行（`CMakeLists.txt:43-45`），**在 Jetson Orin Nano（6 核）上 CPU 争用明显**。
  - `laserMapping.cpp:1064-1066` `fout_pre` / `fout_out` **无条件打开**（受 `runtime_pos_log_enable` 影响的是内部逻辑，但文件打开在启动阶段），持续写盘到源码树的 `Log/`。
- **触发条件**：地图点数增大；点云频率/密度提高；同时跑 `ego-planner`、`mavros`、`px4_*` 等（工作区里都有）。
- **后果**：ICP 周期抖动甚至超过 2 s，`/localization` 与 `/Odometry` 时间戳错配，EKF2 观测延迟增大（**EKF2 对延迟敏感**，会表现为"定位看起来在动但融合效果差"）；`/Odometry` 丢帧导致 50 Hz 承诺不成立。
- **建议修法**：(1) `pc2_to_array` 改用 `numpy.frombuffer(msg.data).reshape(-1, ...)` 的向量化写法（按 `point_step` 步长取 x/y/z），或在 `fastlio_mapping` 侧**直接新增一个 `sensor_msgs/PointCloud2` 直通话题**供 ICP 用，避免 python 逐点；(2) 提高 `/Odometry`、`/cloud_registered` 的订阅深度（如 5–10）并统计丢帧；(3) 给 `global_localization` 与 `laserMapping` 设 CPU 亲和性（`taskset`）或 cgroup 限额，避免与规划器抢核；(4) 关闭不需要的 `path_en`（`mid360_test.yaml:58` 为 `true`，`laserMapping.cpp:807-819` 每 10 帧 push 一次 Pose 且 `path.poses` 无上限 → **`/path` 长期运行会持续增长内存**）。

### R11 —【低】`/cur_scan_in_map` 语义错误 + 就地改 header（潜在 UB）

- **现象**：`global_localization.py:105-108` — 回调参数是 `const SharedPtr`（非 const 消息），却直接 `msg.header.frame_id = 'camera_init'` 和 `msg.header.stamp = now()`，**就地修改了输入消息**，然后转发。话题名叫 `_in_map` 但内容是 `camera_init` 帧。
- **触发条件**：任何以 `map` 为 Fixed Frame 的 RViz 视图同时显示 `/cur_scan_in_map` 与 `/global_map`。
- **后果**：视觉上"当前扫描和地图对不上"，浪费排查时间；就地改输入消息在共享订阅场景下可能影响其他订阅者（当前 `/cloud_registered` 只被 `global_localization` 订阅，故实际风险低）。
- **建议修法**：改名或真的做 `map` 变换；改副本而不是原消息（`msg = copy.deepcopy(msg)` 再改）。

### R12 —【低】`MODIFICATIONS.md` 声称与实现细节的两处偏差

- **现象**：(a) `MODIFICATIONS.md:31` 说"恢复后只发布当前最新状态" —— 实现上 `:512` 的 `first_sample || publish_period_elapsed` 加上 `:517` 的单一 `sample_ready` 确实只产出一条，符合描述，**已核实一致**；(b) 但 `MODIFICATIONS.md:44` 的公式写作 `T_map_to_body = T_map_to_odom * T_odom_to_body`，而代码里 `T_map_to_odom` 实际是把 `camera_init` 对齐到 `map` 的变换（`transform_fusion.py:52-74`），"odom" 一词在 TF 树里无对应 frame（§5.3）。
- **触发条件**：新人按文档去 TF 树里找 `odom` 或按文档理解公式。
- **后果**：概念混淆，易在排查时走错方向（例如去查一个不存在的 `odom` frame）。
- **建议修法**：文档改称 `T_map_to_camera_init`，或在文中显式说明 `odom ≡ camera_init`。

### R13 —【低】`api`/构建层面的瑕疵

- **现象**：`CMakeLists.txt:8-18` 反复设置标准：`ADD_COMPILE_OPTIONS(-std=c++17)` 写了两遍（`:8`、`:9`），`set(CMAKE_CXX_FLAGS "-std=c++17 -O3")`（`:10`）又在 `:18` 追加 `-std=c++17 -pthread -std=c++0x -std=c++17`，同时 `:15` 设 `CMAKE_CXX_STANDARD 14`、`:17` `CMAKE_CXX_EXTENSIONS OFF` → **互相矛盾**。`README.md:32` 声称可执行文件在 `scripts/fastlio_mapping`（实际装到 `lib/fast_lio_localization/`，`CMakeLists.txt:119-121`），`README.md:36` 说地图放 `PCD/`。`package.xml:13` maintainer 是 `dev@livoxtech.com` / `:17` author 是 LOAM 作者，`package.xml:6-11` 的 description 也是 LOAM 的。
- **触发条件**：换编译器版本；或按 README 去 `scripts/` 找二进制。
- **后果**：低。主要是误导。
- **建议修法**：清理 CMake 重复设置；README 的目录树按实际更新。

---

## 8. 未确认项（不得当作结论使用）

1. **无人机上 `/home/duzhong/dzpx4/src/maps/output_voxel_0.05.pcd` 是否真实存在**（本次未 SSH，仅本地镜像有同名文件）。
2. **`map` 帧的偏航朝向**，以及 `px4_odom_convert.cpp:20-23` 的 `diag(-1,-1,1)` 是否与实际一致（需上机静态校验，见 R3）。
3. **无人机上 Open3D / `tf_transformations` / `sensor_msgs_py` 是否已装、python 版本**（本地 Windows 上 `open3d`、`tf_transformations`、`sensor_msgs_py` 均不存在，不能代表无人机；`__pycache__` 的 `cpython-311` 暗示曾装通）。
4. **无人机上 `install/` 空间里的 config 是否与 `src/` 一致**（本镜像与 `src/FAST_LIO_LOCALIZATION_ROS2/` 逐文件哈希一致，但**未与无人机上的 `install/share/` 对比**；若有 `colcon build` 后未重装的情况，实际生效的可能是旧 yaml）。
5. **`/initialpose` 在无人机运行时是否已有人发过、`global_localization` 是否真的初始化成功过**（需看运行日志）。
6. **`freq_pub_localization` 是否真的被忽略**（本次从代码结构判断——`transform_fusion.py` 无 `declare_parameter`；未在真机 `ros2 param get /transform_fusion freq_pub_localization` 验证）。
7. **`mid360_test.yaml` 与 `mid360.yaml` 哪一份对应 PCD 里那张地图的建图参数**（`output_voxel_0.05.pcd` 的 0.05 m 体素暗示建图时用的是更细的地图过滤，与 `mid360_test.yaml` 的 0.18 不同 → **地图与运行参数可能不同源**，这会直接降低 ICP fitness，见 R2）。
