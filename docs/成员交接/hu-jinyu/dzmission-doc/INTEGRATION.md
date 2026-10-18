# dzmission 集成说明

本文件说明 **需要你在现有工程里做的改动**，以及每条需求分别由哪段代码承担。
dzmission 包本身不修改 dzactuator / dztrack / dzjudgment / dzrknn_dataprocess /
rknn_pt 的行为，只通过话题与动作接入。

---

## 1. 需求 → 实现映射

| 需求 | 实现位置 | 关键话题 |
|---|---|---|
| 接收指令 | `CmdLink`（串口/UDP/loopback）+ `MissionManager::onMissionCommand` | `/mission_cmd` |
| 路径规划 | `MissionManager::handleNavTo` → nav2 `NavigateToPose`；dztrack 预录路径仍可并行工作 | `navigate_to_pose` 动作、`/dztrack_raw_cmd` |
| 运动控制 | `MotionArbiter`（唯一速度出口，带优先级/TTL/加减速限幅） | `/cmd_vel`、`/pursuitAngle` |
| 自身定位补偿 | `LocalizationCompensator` + `Eskf`（IMU 主导航向 + 轮速观测量测 + 绝对位姿重置 + map→odom 重锚） | `/odometry/filtered`、`/localization/healthy` |
| 近距离避碰 | `SafetyMonitor`（雷达扇区三级门限）+ dztrack 自带 `obstacleDetect` | `/safety_brake_level`、`/safety_markers` |
| 云台识别标靶 | `TargetAimer`（扫描/锁定/视觉伺服）；识别由 `rknn_pt/det_node` 提供 | `/offset_center`、`/CameMo_Conl` |
| 激光发射与打击 | `TargetAimer` 开火状态机 + 联锁；下发到裁判板 | `/LaserShot_Command` |
| 车载裁判板 / 裁判系统 | `RefereeBoard`（订阅 dzjudgment 结果，或直连串口） | `/HpAndHitmsg`、`/referee/*` |

### 架构

```
                    ┌──────────────── 无人机 / 上位机 ────────────────┐
                    │  任务指令帧 (0xAA55…)        状态回传帧          │
                    └───────────────┬──────────────────▲─────────────┘
                          serial/UDP│                  │serial/UDP
┌───────────────────────────────────▼──────────────────┴──────────────────────┐
│                              dzmission_node                                 │
│                                                                             │
│  CmdLink ──/mission_cmd──► MissionManager ──► nav2 NavigateToPose           │
│                                │   ▲                                        │
│                    ┌───────────┘   └────────────┐                           │
│                    ▼                            ▼                           │
│            MotionArbiter                    TargetAimer                     │
│        （唯一速度出口，优先级仲裁）      （扫描/锁定/伺服/开火联锁）          │
│                    ▲                            ▲                           │
│            SafetyMonitor                 RefereeBoard                       │
│          （雷达三级门限）              （血量/弹量/被击/物资）                 │
│                    ▲                            ▲                           │
│      LocalizationCompensator                    │                           │
│        （ESKF 定位补偿）                        │                           │
└────┬──────────────┬──────────────┬──────────────┬──────────────┬────────────┘
     │/cmd_vel      │/odometry     │/scan         │/offset_center│/HpAndHitmsg
     │/pursuitAngle │  /filtered   │              │              │
     ▼              ▲              ▲              ▲              ▲
 dzactuator      dzactuator    rplidar_ros    rknn_pt        dzjudgment ──► 裁判板
 (串口下位机)    (odom,imu)                                  (串口 /dev/dzjudgment)
```

---

## 2. 必须做的改动

### 2.1 让 dzmission 成为速度唯一出口

现状冲突：

| 节点 | 现在发布 | 冲突点 |
|---|---|---|
| `dztrack` | `cmd_vel_dztrack`、`pursuitAngle` | 直接驱动底盘，绕过仲裁 |
| nav2 `velocity_smoother` | `cmd_vel` | 与 dzmission 争 `cmd_vel` |
| `dzactuator` | 订阅 `cmd_vel`、`pursuitAngle`、`logicontrol` | 需要唯一来源 |

**做法：用 launch remapping，不改源码。** 仲裁器已经订阅了下面两个"改道后"的话题：

| 仲裁器订阅 | 优先级 | 需要把谁的输出重映射过来 |
|---|---|---|
| `dztrack_raw_cmd` | 高 | `dztrack` 的 `pursuitAngle` |
| `cmd_vel_nav_raw` | 低 | nav2 `velocity_smoother` 的 `cmd_vel_smoothed` |

启动 dztrack 时加：

```python
Node(
    package='dztrack', executable='dztrack', name='dztrack',
    remappings=[
        ('pursuitAngle', 'dztrack_raw_cmd'),   # 速度输出改道
        ('cmd_vel_dztrack', 'dztrack_raw_cmd'),
    ],
)
```

启动 nav2 时加（`navigation_launch.py` 的 `velocity_smoother` 节点，或直接用
`dzmission.launch.py` 里包含的 nav2）：

```python
remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel_nav_raw')]
```

> 若你暂时不想动 `dztrack` 的 launch（`dzactuator/launch/dztrack.launch.py`），
> 也可以先只跑 nav2 一条链路：把 `mission.use_dztrack_when_active` 设为
> `false`，dztrack 的速度就不会被采信 —— **但 dztrack 仍然会直接发布
> `pursuitAngle` 驱动底盘，这是硬冲突，必须用 remapping 解决，无法用参数绕过。**

### 2.2 保证只有一个节点驱动云台

`dzrknn_dataprocess` 与 `TargetAimer` 功能重叠。二者**不能同时**发布
`CameMo_Conl` / `LaserShot_Command`。

选择一（推荐，改动最小）：**云台仍归 dzrknn_dataprocess**

```yaml
aim:
  own_gimbal: false      # dzmission 只监视 + 联锁判断，不发云台/激光
```

选择二：**云台归 dzmission**（联锁更完整：弹药/被击/电量/射程全部生效）

```yaml
aim:
  own_gimbal: true
```

同时需要在 `dzrknn_dataprocess` 里屏蔽它的云台与激光发布。
最小侵入做法是给它加一个参数并在发布前判断，共 3 处：

```cpp
// 构造函数中新增
this->declare_parameter("publish_commands", true);
this->get_parameter("publish_commands", publish_commands_);

// Control() 中
if (publish_commands_) {
  pub_LaserShot_Command->publish(shot);
  pub_CameMo_Conl->publish(camemo_cmd);
}
```

或者更彻底：只运行 `rknn_pt/det_node`（它只发布 `/offset_center`），
不运行 `dzrknn_dataprocess`。此时改 `dzactuator/launch/det_node.launch.py`：

```python
Node(package='rknn_pt', executable='det_node', name='det_node', output='screen')
```

> `det_node.launch.py` 现在写的是 `package='det_node'`，而实际包名是
> `rknn_pt`（见 `rknn_pt/CMakeLists.txt` / `package.xml`），两处都要改成
> `rknn_pt`。同时 `det_node.cc` 里模型路径硬编码为
> `/home/duzhong/dzrobot/src/rknn_pt/model/light_det.rknn`，建议改成
> 用 `ament_index_cpp::get_package_share_directory("rknn_pt")` 拼接。

### 2.3 TF 树 —— 有一个必须先确认的冲突

现有 `dzactuator/launch/bringup.launch.py` 已发布 `base_link→imu_link` 与
`base_link→laser` 静态变换。**注意该文件里 EKF 那段是被注释掉的**，所以
`odom→base_link` 实际上由 `LocalizationCompensator` 负责（这正是它存在的意义）。

但如果你后来把 `bringup.launch.py` 里的 `ekf_node` 取消注释，就会变成
**两个发布者争同一条 TF 边**（`odom→base_link`）。此时请三选一：

| 方案 | 做法 |
|---|---|
| 推荐 | 保持 `ekf_node` 注释，TF 由 `LocalizationCompensator` 全权负责 |
| 保留 EKF | 设 `loc.publish_tf: false`，只让 EKF 发布 `odom→base_link` |
| 只要 AMCL 的 map→odom | 设 `loc.publish_map_to_odom: false` |

`LocalizationCompensator` 会发布：

- `odom → base_link`（滤波后位姿；`loc.publish_tf` 控制）
- `map → odom`（动态重锚，仅在收到绝对位姿后生效；`loc.publish_map_to_odom` 控制）

另外：**不要**再启动别的 `map→odom` 静态发布（例如
`dzactuator/launch/fake_amcl.launch.py` 里的 `map_to_odom_tf`），否则 TF 冲突。

`dzmission.launch.py` 只启动 `dzmission_node`，而 `dzmission_node` 内部**同时**
实例化了 `LocalizationCompensator` 与 `MissionManager`（多线程执行器，2 线程），
所以不需要额外 launch 条目。可用下列命令确认两条边都只有一个发布者：

```bash
ros2 run tf2_tools view_frames
ros2 topic echo /tf_static --once
```

### 2.4 定位数据源

`LocalizationCompensator` 需要三路输入：

| 话题 | 来源 | 现状 |
|---|---|---|
| `imu_data` | `dzactuator` | 已发布（`pub_diff_filter`），滤波器带零偏校正 |
| `odom` | `dzactuator` | 已发布，`twist.linear.x` 已乘 50（见下） |
| `amcl_pose` | `nav2_amcl` 或 `fake_amcl_node` | 二选一 |

> **注意 `dzactuator` 的速度标度**：`Publish_Odom()` 里
> `odom.twist.twist.linear.x = linear_Speed * 50`，
> `angular.z = ThetaSpeed * 50`。这是因为下位机每 20 ms 上报一次速度
> （`Control()` 循环 100 Hz，但串口数据帧是 50 Hz 节奏），
> 乘以 50 的含义是"把单帧位移当成速度"。
> **如果下位机固件或轮径标定变化，这个 50 必须同步修改**，
> 否则 ESKF 的速度观测会整体偏大 50 倍，表现为定位飞速漂移。
> 验证方法：推车 1 m，看 `/odometry/filtered` 的 x 变化是否为 1 m。

`fake_amcl_node` 只是把 `/odometry/filtered` 原样转成 `/amcl_pose`，
用它做绝对位姿观测等于自反馈，没有修正作用。**建议**：

- 有真实定位（nav2_amcl + 地图）时用它；
- 没有时把 `loc.enable_absolute_fix` 设为 `false`，纯靠 ESKF + ZUPT 航位推算，
  并定期人工重置起点。

### 2.5 裁判板

当前 `dzjudgment` 独占 `/dev/dzjudgment` 并解析出以下话题：

| 话题 | 结构 | 含义 |
|---|---|---|
| `HpAndHitmsg` | `[targethp, self_hp, Bing_hit, Shooting_Count, ammo]` | 血量/被击/射击数/弹量 |
| `all_Material_Number` | 物资点编号列表 | 全场物资 |
| `self_Material_Number` | 己方物资 | |
| `enemy_Material_Number` | 敌方物资 | |

`RefereeBoard` **默认只订阅这些话题**（`referee.direct_serial: false`），
所以请保持 `dzjudgment` 运行：

```bash
ros2 launch dzactuator judgment.launch.py
```

若你不想跑 `dzjudgment`，把 `referee.direct_serial` 设为 `true`，
`RefereeBoard` 会自己按同样的 0xB5 0x5B 协议读写串口。**但此时必须停掉
`dzjudgment`**，否则两个进程争抢同一个设备节点。

> 需确认的对接细节（我按现有代码推断，若与固件不符请在
> `referee_board.cpp::loadHpAndHit` / `processData` 中调整）：
> 1. `0x07` 帧的 `Bing_hit` 字段取值：代码按"`0x01..0x04` 表示被击方向，
>    `0x10` 表示未被击"处理（与 `judgment.h` 中注释一致）。
> 2. 弹量是 `0x07` 帧第 5 字节（`ammo`）。若裁判板把弹量放在
>    `all_Material_Number`（部分赛制把物资数与弹量混用），需要改这里。
> 3. `dzjudgment::recvESP32Info()` 用的是 `while(true)` 阻塞读，且
>    `charArray[dataLength + 3]` 是变长数组，`dataLength` 理论上可被
>    对端控制到 255，存在栈越界风险。建议顺手上限校验
>    （`if (dataLength > 64) return;`）。

### 2.6 相机

`rknn_pt/det_node.cc` 订阅 `/image_raw`，`dzactuator/launch/camera.launch.py`
用 `usb_cam` 发布的默认话题是 `/image_raw`，一致，无需改动。
但 `camera.launch.py` 里中文注释被转义破坏（`鎽勫儚澶磋澶囪矾寰?`），
`video_device` 等键名可能已损坏，建议直接重写该文件：

```python
Node(
    package='usb_cam', executable='usb_cam_node_exe', name='usb_cam_node',
    output='screen',
    parameters=[{
        'video_device': '/dev/video0',
        'image_width': 640,
        'image_height': 480,
        'framerate': 30.0,
        'pixel_format': 'yuyv',
        'io_method': 'mmap',
    }],
)
```

### 2.7 射程联锁（可选，需要改一行视觉代码）

`aim.max_engage_range_m` 的判定依赖视觉给出目标距离。当前
`rknn_pt/src/det_node.cc` 的 `/offset_center` 只发 `[cx, cy, class_id]`
（`msg.data = {0, 0, 0}`，第 4 位固定为 0），所以射程联锁**默认自动放行**
（`last_target_range_m_ <= 0` 视为"无距离信息，不限制"）。

要让射程联锁真正生效，把 `det_node.cc::processTimer()` 里的

```cpp
msg.data = {0, 0, 0};
...
msg.data[2] = 1;
```

改为填入距离（单位 0.01 m）：

```cpp
msg.data = {0, 0, 0, 0};        // 扩到 4 位
...
msg.data[2] = 1;                 // class_id（1 = 敌方靶标）
msg.data[3] = static_cast<int>(estimated_range_m * 100.0);
```

距离可由靶标成像宽度反算（需要靶标物理尺寸标定），或由无人机在
`ENGAGE` / `TARGET_REPORT` 帧里给出的 `range` 转发。若两条链路都没有距离
来源，就保持现状：射程不限制，由云台视场与 `fire_tolerance_px` 控制命中。

### 2.8 雷达安装角已按 `rpy=(pi,0,0)` 修正

`safety_monitor.cpp` 里把雷达方位角映射到 `base_link` 前向角的方式是

```
a_base = -a_laser
```

因为 `base_link→laser` 的 `rpy=(3.14159,0,0)` 是绕 x 轴翻转 180°，
`R = Rx(pi) = diag(1,-1,-1)`，所以 `a_base = π - a_laser ≡ -a_laser`（mod π 意义下
的方位角取反）。

**如果你发现避障转向方向仍然相反**（障碍在左却向左转），只可能是硬件实际安装
与 launch 里的 `rpy` 不一致。此时把两处 `normalizeAngle(-a)` 改为
`normalizeAngle(a)`（`safety_monitor.cpp` 的扫描循环与侧向循环各一处）即可，
不要改成 `a + M_PI`（那会把左右镜像而距离判断仍然正确，是最坏的错法）。

---

## 3. 启动顺序

```bash
# 0. 编译
cd ~/dzrobot
colcon build --symlink-install --packages-select dzmission dzactuator dztrack dzjudgment dzrknn_dataprocess rknn_pt fake_amcl_node
source install/setup.bash

# 1. 底盘 + IMU + 里程计
ros2 launch dzactuator bringup.launch.py

# 2. 雷达
ros2 launch dzactuator rplidar_s3e_launch.py

# 3. 相机
ros2 launch dzactuator camera.launch.py

# 4. 定位（二选一）
ros2 launch dzactuator amcl.launch.py            # 有地图，真实 AMCL
# 或
ros2 launch dzactuator fake_amcl.launch.py       # 无地图，纯航位推算

# 5. 标靶识别（只识别，不带云台控制）
ros2 run rknn_pt det_node

# 6. 裁判板
ros2 launch dzactuator judgment.launch.py

# 7. 任务编排层（含 nav2）
ros2 launch dzmission dzmission.launch.py

# 8. 预录路径跟踪（可选；必须带 remapping，否则与仲裁器抢底盘）
ros2 launch dztrack dztrack.launch.py
```

离线自测（无无人机、无导航）：

```bash
ros2 launch dzmission dzmission_selftest.launch.py
ros2 topic echo /mission_state
ros2 topic echo /safety_brake_level
ros2 topic echo /odometry/filtered
```

---

## 4. 优先级与安全行为

`MotionArbiter` 的优先级（高 → 低）：

| 优先级 | 触发条件 | 输出 |
|---|---|---|
| 1 | 雷达 `brake_level>=2` 或定位数据 500 ms 未更新 | 零速 + `linear.y=1`（刹车请求） |
| 2 | 遥控接管（`logicontrol` 500 ms 内有消息） | 零速（遥控通道由 dzactuator 自己走） |
| 3 | 云台锁车（交战 + 雷达急停 + `gimbal_lock_chassis`） | 零速 + 刹车请求 |
| 4 | 任务停止（`STOP` / `HOLD` / 到达 / IDLE） | 零速 |
| 5 | dztrack 速度（`dztrack_active_timeout_s` 内） | 限幅后的 dztrack 速度 |
| 6 | nav2 速度（`source_ttl` 内） | 限幅后的 nav2 速度 |
| 7 | 无源 | 零速 |

每个速度源都有 TTL（`arbiter.source_ttl`，默认 0.4 s），超时自动失效。
这一条专门用来堵住 `control.cpp::doplanner()` 在 `makePlan` 失败时
`continue` 而不发布零速的漏洞 —— 即使 dztrack 卡住不发速度，
底盘也会在 0.4 s 后自动停下。

### 4.1 仲裁结果如何落到 dzactuator（重要）

`dzactuator::modeTimerCallback()` 用"最近 0.5 s 内哪个话题有消息"来选模式：

| 最近活跃话题 | `current_mode_` | 回调 | 角度解读 |
|---|---|---|---|
| `pursuitAngle` | 2（Pursuit） | `callback_movebase_angle` | **直接使用 `angular.z`** |
| `cmd_vel` | 1（DWA） | `callback_cmd_vel_angle` | 用 `atan(CARL*w/v)*57.3` 换算 |

因此 `MotionArbiter` 的发布策略是：

| 情形 | 发布到 | `linear.y` |
|---|---|---|
| 源 = dztrack | 只发 `pursuitAngle` | 0 |
| 源 = nav2 / mission | 只发 `cmd_vel` | 0 |
| 刹车（避障急停 / 硬急停） | **两个话题都发**，`v=0` | 1 |
| 刹车刚解除的那一帧 | 两个话题都补发一帧 | 0 |

最后一行是为了把 `dzactuator` 的 `raw[3]` 从 Pursuit 拉回 DWA
（`raw[3]` 只在收到 `linear.y >= 0.5` 的 `pursuitAngle` 后 100 ms 内才等于 DWA），
否则解除刹车后自动驾驶的角度指令会被当成纯跟踪的 `w` 来解读。

> 如果这一层错位，症状是：**自动驾驶时转向明显偏大或反向**，
> 而手动遥控（走 `logicontrol`）却完全正常。

---

## 5. 已知风险与建议后续改动

| 风险 | 位置 | 建议 |
|---|---|---|
| 纯旋转时航向不更新 | `dzactuator.cpp` `Control()` 里 `if (linear_Speed > 0.00001)` 才更新 `Robot_Pos.Z` | dzmission 已用 IMU 绕开；若要用原生 odom 做定位，建议把积分条件改成"速度或角速度任一非零" |
| 速度标度 `*50` 语义不明 | `dzactuator.cpp::Publish_Odom()` | 与固件/轮径一起标定，写进注释 |
| 里程计速度单位与注释不符 | `Odom_Trans()` 直接把 mm/s 的 16 位当 float 返回 | 确认下位机上报单位，必要时 `*0.001` |
| `dzjudgment` 变长数组越界 | `judgment.cpp::recvESP32Info()` 用 `uint8_t charArray[dataLength + 3]`，`dataLength` 可被对端控制到 255 | 加 `if (dataLength > 64) return;` 上限校验（`dzmission` 的 `RefereeBoard` 直连实现已做此校验） |
| `doplanner` 失败不归零 | `control.cpp::doplanner()` 在 `makePlan` 失败时 `continue` 而不发零速 | 已在仲裁层用 0.4s TTL 兜底；根治需在 `continue` 前把 `cmd_vel` 归零并发布 |
| 模型路径硬编码 | `rknn_pt/src/det_node.cc`、`dzrknn_dataprocess.cpp` | 改用 `get_package_share_directory` |
| `det_node.launch.py` 包名错误 | `package='det_node'`，实际包名是 `rknn_pt` | 改为 `package='rknn_pt'`，否则 `ros2 launch` 直接失败 |
| `offcenter_table.txt` 全是 0 | `dzrknn_dataprocess` | 该表用于 `compute_comp_pixel_0`，当前未被调用；若要启用需重新标定 |
| 雷达安装角与 launch 不一致 | `bringup.launch.py` 的 `rpy=(pi,0,0)` | `SafetyMonitor` 已按此修正为 `a_base = -a_laser`；若实测相反，改成 `normalizeAngle(a)` 而**不是** `a + M_PI`（见 2.8） |
| 视觉不给距离 | `rknn_pt/det_node.cc` 的 `/offset_center` 第 4 位固定 0 | 射程联锁默认放行；要启用见 2.7 |

---

## 6. 自检脚本

`dzmission/test/` 下的脚本不依赖 ROS，可在任意机器上运行：

| 脚本 | 作用 |
|---|---|
| `protocol_check.py` | 协议 v1 的 Python 参考实现 + 自检（CRC 标准向量、帧结构、编解码往返、损坏帧拒绝）。也可用 `--decode/--nav-to/--engage` 生成或解析帧 |
| `params_check.py` | 核对 `params/dzmission.yaml` 的键名与 C++ 里 `declare_parameter` 的键名是否完全一致（防止参数名拼错导致静默退回默认值） |
| `member_check.py` | 核对 `.cpp` 用到的成员变量是否都在头文件里声明（防止编译期 `undeclared identifier`） |
| `decl_def_check.py` | 核对 `.cpp` 里定义的每个成员函数是否都在头文件里有声明 |
| `cpp_consistency_check.py` | CMakeLists 源文件覆盖与安装目录检查 |

```bash
cd src/dzmission/test
python3 protocol_check.py
python3 params_check.py ../params/dzmission.yaml
python3 member_check.py
python3 decl_def_check.py
```

编译后建议按顺序验证：

```bash
# 1. 协议自检（上位机侧也用同一份算法对接）
python3 src/dzmission/test/protocol_check.py

# 2. 参数是否全部生效（不该出现 "parameter not declared" 警告）
ros2 run dzmission dzmission_node --ros-args \
  --params-file install/dzmission/share/dzmission/params/dzmission.yaml

# 3. 关键话题是否都在
ros2 topic list | grep -E 'mission|odometry/filtered|localization|safety|referee|aim'

# 4. 定位补偿是否收敛：推车 1m，看 x 变化是否约 1m
ros2 topic echo /odometry/filtered --field pose.pose.position

# 5. 雷达方向是否正确：在车左侧 0.5m 放障碍，看 safety_markers 的 y 是否为正
ros2 topic echo /safety_brake_level
```

---

## 7. 借鉴自学员工程的关键结论（★ 实测数据，改动依据）

来源：`C:\Users\Ran\Desktop\无人车\发学员\src(走斜线)\` 与 `src(走直角)\`。
两者 `serial_reader.py` / `path_planner.py` **完全相同**，唯一运动差异在
`path_executor.py`（圆弧版 vs 对角直线版）。以下结论均带实测记录。

### 7.1 ★ `pursuitAngle.angular.z` 是舵机角度，不是角速度

```python
# 学员代码 docstring 原文
"Publishes Twist on 'pursuitAngle' where angular.z is a SERVO ANGLE in
 degrees (0 = straight, valid range [-35, +35]), matching dzactuator."
"dzactuator treats angular.z as a SERVO ANGLE in DEGREES, not angular
 velocity: servo_angle = angular.z + 60, clamped to [25, 95]"
```

| 话题 | `angular.z` 含义 | 单位 | 范围 |
|---|---|---|---|
| `pursuitAngle` | **舵机角度** | 度 | [-35, +35] |
| `cmd_vel` | 角速度 | rad/s | — |

**两个通道绝不能共用同一个限幅参数**。`MotionArbiter` 为此拆成
`max_steer_deg`（度）与 `max_w`（rad/s），并加 `max_acc_servo`（度/s）
保护舵机。切换通道时另一路的积分状态会归零，避免跳变。

> 如果这一条搞错，症状是：**车按角速度的量级转舵机，直接打满乱转**。

### 7.2 ★ 不需要显式发 `logicontrol` 切模式

学员圆弧版**只发 `pursuitAngle`**，注释写：

```
"Note: modeTimerCallback will auto-switch to Pursuit mode
 when pursuitAngle is the only active topic"
```

这与我们对 `dzactuator` 的分析一致 —— 所以仲裁器只需保证**同一时刻只有一个
话题活跃**，靠 `modeTimerCallback` 自动切模式，不必自己发 `logicontrol`。

### 7.3 ★★ 底盘积分的位姿不能用，但行驶距离可以用

`serial_reader.py` 的实测记录（原文）：

> `dzactuator's integrated pose lives in an encoder-yaw frame whose rotation
> does not match the physical world (verified 2026-09-06: after the same 90deg
> right turn, the raw track drifted to opposite sides across two runs and
> spirals outward monotonically), so raw pose deltas are unusable as world
> coordinates. The sample-to-sample travel DISTANCE is rotation-invariant and
> accurate, so it is integrated along the IMU map yaw instead.`

因此 `LocalizationCompensator` 已改为：

| 量 | 来源 | 理由 |
|---|---|---|
| 位置 | `step = |Δpos_raw|`，沿 **IMU 地图偏航**积分 | 距离是旋转不变量且实测准确 |
| 偏航 | IMU 相对角 + 标定偏移 | 底盘偏航系是坏的 |
| ~~位姿~~ | ~~`odom.pose`~~ | **不再使用**（会污染滤波器状态） |

标定方式：`/nav/anchor` 话题，格式 `anchor:COL,ROW,HEADING_DEG`
（例如车放在 M7 朝 +x → `anchor:M,7,0`）。
语义 = 车**此刻**在该格心、朝该方向。作用是把位置吸附到格心 + 重标偏航偏移。

同时提供 `snapToCell(col, row)`，供任务层在到达格心时调用 —— 这是学员
"每个转角做位姿快照"的等价物，能把漂移限制在一段之内。

### 7.4 ★ 舵机左右行程严重不对称 → 转向必须用偏航闭环

```
TURN_ARC_SCALE: 1.0 -> 110deg overshoot. 0.80 gives ~90deg right, ~111deg left.
2026-09-04 clean run (X-mirror fixed): right arc rotated only ~50deg over
0.38m (R_phys~0.44m) while left arc rotated ~140deg over 0.37m
(R_phys~0.15m) at the SAME full-lock command. The servo travel is strongly
asymmetric, so a DISTANCE-based arc can never give correct angles on both
sides. Arc stop is therefore YAW-based now.
```

⇒ **基于距离的圆弧永远不可能左右都对**。所以：
- 原地转向的主判据 = 偏航角闭环（`PivotController` 的 phase 2）
- 行程只作**安全上界**（`creep_max_cm` = 名义圆弧 × `safety_scale`）
- 再加总超时兜底

另外实测有一个**恒定的机械左偏**，学员用 `STEER_TRIM = -1.0` 度（负=右）抵消。
本节点在 `FieldGrid::Spec::steer_trim_deg` 与 `pivot.steer_trim_deg` 里保留该补偿。

### 7.5 ★ 窄条内必须用原地转向，不能用单圆弧

| 转向方式 | 扫掠半径 | 依据 |
|---|---|---|
| 单圆弧 90° | `R + 车体对角半径` = 26 + 18.6 = **44.6 cm** | `test/geometry_check.py` |
| **原地转向** | 车体对角半径 = **18.6 cm** | `test/pivot_turn_check.py` |

后者只有前者的 **1/2.4**，因为原地转向绕后轴中心**纯转动**、没有平动扫掠。
普查（`test/pivot_turn_check.py`）：

```
可通行格 145 个；绕自身 360° 旋转不侵入真实阻挡的 85 个
失败的 60 个：压场边 44（第1/11行、最外列）、压水平挡板 4、邻格被挡 12
```

**L/M/N（窄条）全部可行** —— 而单圆弧在窄条内**一个可转点都没有**。

> ⚠️ 判据陷阱：「格与格之间的分界线**不是障碍物**」（场地没有四周围挡）。
> 只有场边、水平隔离线挡板、桥洞隔板、水桶是真实阻挡。
> 我第一版脚本把"跨过格边界"当失败，得出"只有 88/154 可行"，是错的。

实现：`FieldGrid::checkPivot(col,row)` / `pathPivotsFeasible()`，
`PivotController` 三段式（STOP → WAIT_SERVO → CREEP → DONE）。

### 7.6 车体常数与速度（与官方手册的差异需现场核实）

| 项 | 学员代码 | 官方手册 | 备注 |
|---|---|---|---|
| 轴距 | **0.17 m** | 150 mm | 差 13%，**必须现场量** |
| 最大舵角 | 30° | 30 ± 2° | 一致 |
| 直线速度 | 0.32 m/s | 最高 1.0 m/s | 学员取保守值 |
| 转弯速度 | 0.14 m/s | — | |
| 原地蠕行 | 0.08 m/s | — | |
| 到点阈值 | 0.20 m | — | |
| 观测前瞻 | 0.35 m | — | |

纯跟踪律用的是 `atan(WHEELBASE*sin(α)/ld)`，**没有标准公式里的 2.0 系数** ——
说明 `dzactuator`/`dztrack` 内部可能已含该系数，接的时候要注意别重复。

### 7.7 其他可直接复用的工程做法

| 做法 | 价值 |
|---|---|
| **节点版本戳**：`NODE_VERSION="v4-dr-reset"` 打进每行日志 | launch 加载的是 colcon **安装副本**，源码改了忘 build 就静默跑旧代码 |
| **无数据看门狗**：定时器检查 `/odom/localization` 是否真的在流 | 原文："publisher count alone does not prove messages flow" |
| **QoS 用 BEST_EFFORT**：`QoSProfile(depth=1, reliability=BEST_EFFORT)` | 原文："a QoS mismatch here used to silently deliver zero messages, leaving the car standing" |
| **删掉 final-approach 预插点** | "with 0.35 m legs it created backtrack segments that forced real 180 deg turns" —— 我在排雷规划里正好踩过这个 |
| **到达但朝向反了要重试**：偏航错 >60° 就重新接近（限 2 次） | 防"到了但朝反" |
| **射击握手**：到点发 `stop_signal=1` 触发云台扫描+射击，订阅 `/shoot_count` 数到 N 发再走 | 可与我们的 `TargetAimer` 对接 |
| **坐标系自检脚本** + `FLIP_X/FLIP_Y` 开关 | 应对"前进方向反了"这类低级但致命的错 |

### 7.8 `/offset_center` 消息格式（★ 已修正，原实现链路是断的）

**实测发现的三个 bug（已修）**

| # | 问题 | 症状 |
|---|---|---|
| 1 | `det_node` 只发 **3** 个元素，而 `dzrknn_dataprocess::callback_offset_center` 要求 `size() >= 4` | **每一条视觉消息都被下游 `return` 丢弃**，云台视觉伺服与开火判定永远收不到 |
| 2 | `SafetyMonitor` 按 `Float32MultiArray` 订阅，而 `det_node` 发的是 `Int32MultiArray` | **类型不匹配，回调永不触发**（视觉近场判据实际从未生效） |
| 3 | `SafetyMonitor` 读 `data[4]/data[5]` 当 y1/y2，但当时的消息只有 3 个元素 | 索引越界逻辑，从未生效 |

**修正后的统一格式（`std_msgs/Int32MultiArray`，8 个元素）**

| 索引 | 字段 | 含义 | 消费者 |
|---|---|---|---|
| 0 | `offset_x` | 目标中心 − 画面中心（像素） | `dzrknn_dataprocess`、`TargetAimer` |
| 1 | `offset_y` | 同上（y） | 同上 |
| 2 | `class_id` | `0` = 无目标，`1` = 检测到目标 | `dzrknn_dataprocess` 判 `== 1` |
| 3 | `dist_cm` | 距离（cm）。**0 = 未知**（模型不输出距离） | `TargetAimer` 射程联锁（读不到则放行） |
| 4–7 | `bbox_x/y/w/h` | 检测框（像素） | `SafetyMonitor` 近场占用 |

**★ 无目标时也发一帧（全 0）** —— 这是故意的：
否则下游无法区分"没看到目标"和"视觉节点已死"。

**★ 本模型是单类的**（`postprocess.h` 里 `OBJ_CLASS_NUM = 1`，
`DetectionBox` 结构里**没有 `class_id` 字段**）。
所以"红蓝分类头"不是改几行就能用 —— 必须：
1. 重新训练/导出 `OBJ_CLASS_NUM = 2` 的模型
2. 改 `post_process()` 把类别带出来
3. 给 `DetectionBox` 加 `class_id` 字段
4. `det_node` 把 `data[2]` 填成真实类别

在此之前，**红蓝只能靠车端其它手段或按规则默认处理**。

### 7.9 算力优化（已实施）

| 项 | 原状态 | 现状态 | 依据 |
|---|---|---|---|
| 检测帧率 | 硬编码 33ms（30fps） | `detect_rate_hz` 参数，**默认 15** | 瞄准是静态任务，15fps 足够，省一半视觉算力 |
| 推理线程 | 固定 4 | `infer_threads` 参数，**默认 3** | 与 NPU 核数(3)一致，避免核 0 被两线程争抢 |
| 可视化 | 硬编码开 | `show_results` 参数，**默认关** | 车载无显示器，imshow 白烧 CPU 甚至因无 X 卡住 |
| 每帧拷贝次数 | **5 次**全画幅 | 降到 **2 次** | 详见下表 |
| 模型路径 | 硬编码 `/home/duzhong/...` | 参数化 + 多路径回退 | 换机器必挂 |
| NPU 核绑定 | `core_mask` 未初始化（UB） | `core_num_to_mask()` 带 default | 原 switch 无 default 分支 |
| `package.xml` 安装 | 未安装 | 已加 `install(FILES package.xml)` | 否则 `get_package_share_directory` 找不到包 |
| CPU 亲和性 | 无 | `scripts/set_affinity.sh` | 保护 100Hz 实时环 |

**全画幅拷贝的优化明细（640×480 BGR = 0.92 MB/帧）**

| 位置 | 原 | 现 |
|---|---|---|
| `imageCallback` `toCvShare` + `clone` | 2 次 | 1 次（必须，msg 生命周期不受控） |
| `processTimer` `ros_frame_.clone()` | 1 次 | **0 次**（改用 `std::move` 交换） |
| `processTimer` `put(frame,...)` | 1 次 | 1 次（Mat 浅拷贝，仅引用计数） |
| `rkpt.cc` `cvtColor(BGR2RGB)` | 1 次 | 1 次（必需） |
| `rkpt.cc` `cur_img = orig_img.clone()` | 1 次 | **0 次**（仅 `show_results` 时拷贝） |
| **合计** | **5 次 ≈ 138 MB/s** | **2 次 ≈ 55 MB/s** |

---

