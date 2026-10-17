# 补丁 01 —— 定位来源从 `/localization` 切到 `/Odometry`

| 项 | 值 |
| --- | --- |
| 无人机目标路径 | `/home/duzhong/dzpx4/src/px4_odom_convert/launch/px4_odom_convert.launch.py` |
| 本地文件 | `files/px4_odom_convert.launch.py` |
| 改动行数 | 1 行实体改动（`default_value='localization'` → `'odom'`）+ 注释 |
| 是否要编译 | **要**（见下）；但有一个**零编译的运行时替代写法** |

---

## 一、为什么改

`/localization` 从来没有被发布过。链条：

```
transform_fusion.py:13   import tf_transformations   -> ModuleNotFoundError
transform_fusion.py:15   from tf2_ros import transformations  -> 也不存在
=> transform_fusion 进程启动即死
=> 它是 /localization 的唯一发布者（transform_fusion.py:31）
=> px4_odom_convert 在 mode=localization 下订阅一个不存在的话题
=> PX4 从来拿不到外部视觉输入
```

而 `px4_odom_convert.cpp:12` 里代码自带的默认就是 `"odom"`，是 **launch 第 35 行把它覆盖成了 `localization`**。

## 二、为什么切到 `odom` 是安全的

`transform_fusion.py:54`：

```python
T_map_to_odom = self.pose_to_mat(map2odom) if map2odom is not None else np.eye(4)
```

`/map_to_odom` 由 `global_localization.py:54` 发布，而它需要 `open3d`（该文件第 7 行），同样缺失。
所以 `map2odom` 永远是 `None` ⇒ `T_map_to_odom = I` ⇒ `/localization == /Odometry`。

**结论：在重定位链路修好之前，`mode=odom` 与 `mode=localization` 的数值输入逐位相同，只是去掉了两个已死的节点。**

`/Odometry` 由 `FAST_LIO_LOCALIZATION_ROS2/src/laserMapping.cpp:1109` 发布，是活的。

## 三、应用方法（两种，任选）

### 方式 A（推荐先做这个）：零编译，改启动命令

`mode` 本来就是 launch 参数，运行时就能传：

```bash
ros2 launch px4_odom_convert px4_odom_convert.launch.py mode:=odom
```

**不用改任何文件、不用编译**。建议先用这个把链路跑通。

### 方式 B：改默认值（本补丁），让手册里的命令直接正确

```bash
# 1) 备份
cp ~/dzpx4/src/px4_odom_convert/launch/px4_odom_convert.launch.py \
   ~/dzpx4/src/px4_odom_convert/launch/px4_odom_convert.launch.py.bak

# 2) 用本目录 files/ 里的文件覆盖（scp / VNC 拖拽 / 手工编辑均可）
#    目标：~/dzpx4/src/px4_odom_convert/launch/px4_odom_convert.launch.py

# 3) 先判断是不是 symlink-install（决定要不要编译）
ls -l ~/dzpx4/install/px4_odom_convert/share/px4_odom_convert/launch/

# 4) 若是普通文件（拷贝安装）——需要重编译：
cd ~/dzpx4
colcon build --packages-select px4_odom_convert
source install/setup.bash
```

**为什么需要编译**：`px4_odom_convert/CMakeLists.txt:68` 是
`install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)`，
即把 launch 目录**拷贝**进 `install/share/`；`ros2 launch` 通过 ament 索引从 install 空间读取，不读 src。
如果第 3 步显示是符号链接（`--symlink-install`），则跳过后面的编译。

## 四、验证（助手只读执行，也可用户自查）

```bash
source /opt/ros/humble/setup.bash && source ~/dzpx4/install/setup.bash

# 1) 节点日志应打印 Mode: odom  topic: /Odometry
ros2 launch px4_odom_convert px4_odom_convert.launch.py

# 2) 另开终端：确认上游输入活着（约 10 Hz）
ros2 topic hz /Odometry

# 3) 确认 PX4 侧真的在收（这是最终判据）
ros2 topic hz /fmu/in/vehicle_visual_odometry
```

期望：第 3 条能测到稳定频率（与 `/Odometry` 同频，节点会丢弃前 20 帧做初始化，
日志出现 `Odometry initialized, start publishing PX4 VIO`）。

## 五、回退

```bash
cp ~/dzpx4/src/px4_odom_convert/launch/px4_odom_convert.launch.py.bak \
   ~/dzpx4/src/px4_odom_convert/launch/px4_odom_convert.launch.py
cd ~/dzpx4 && colcon build --packages-select px4_odom_convert && source install/setup.bash
```

---

## 六、顺带记录（本补丁**未**改动）

`launch` 第 18–19 行传了 `max_position_jump_m: 0.5` 和 `max_attitude_jump_rad: 0.5`，
但**节点从未声明这两个参数**（`px4_odom_convert.cpp:12-18` 只声明了 7 个），
`hpp` 里也没有任何跳变限制成员。

ROS 2 会静默忽略未声明的参数覆盖，所以不影响运行 —— 但这意味着
**"位置/姿态跳变保护"这个功能在代码里根本不存在**，配置文件却在暗示它存在。
本补丁保持原样（最小改动原则），仅记录为此处的一个待办。
