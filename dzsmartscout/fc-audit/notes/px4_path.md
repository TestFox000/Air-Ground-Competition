# px4_path 只读代码审计笔记

- 审计对象：`D:\User\Desktop\dzsmartscout\fc-audit\src-mirror\px4_path\`（全部 22 个文件，逐个读完）
- 审计方式：纯静态阅读，未 SSH、未执行任何机上命令、未改动被审代码
- 镜像可信度核对：镜像与开发机副本 `D:\User\Desktop\dzsmartscout\src\px4_path\` 逐文件 SHA256 比对，**21 个文件全部一致**；镜像多出的唯一文件是 `px4_path_README_DELIVERY.md`（开发机 `src/` 里没有）。结论：镜像是忠实的源码副本，不存在版本混淆。
- 行号约定：本文所有 `文件:行号` 均相对于镜像仓库根 `fc-audit/src-mirror/px4_path/`（同包路径相对引用，例如 `src/path_loader.cpp:34`）。
- 未确认项集中在 §12，请勿把"未确认"当结论使用。

---

## 1. 包职责与依赖关系

**一句话职责**：`px4_path` 把 FAST-LIO 的 SLAM 轨迹录成文本、读回来转成 PX4 NED，并把它变成一串**外部位置参考**（`px4_control/msg/GotoTarget`）通过 `/px4_control/goto` 发给 `px4_control`；它自己**不碰 Offboard / 解锁 / 起降 / `/fmu/in/*`**。

依赖方向（只有一条，单向）：

```text
FAST-LIO(/localization, nav_msgs/Odometry)
        │  （录制时）
        ▼
  path_record_node ──► TXT (x y z qx qy qz qw, SLAM 原始坐标)

  path_follow_node ──► /px4_control/goto (px4_control/msg/GotoTarget, NED)
        ▲
        └── /px4_control/goto_enabled (std_msgs/Bool, 锁存) ◄── px4_control
        └── /fmu/out/vehicle_local_position (px4_msgs/VehicleLocalPosition) ◄── PX4
```

- 上游给它的：`/fmu/out/vehicle_local_position`（自身状态）、`/px4_control/goto_enabled`（执行许可）、`/localization`（仅录制）。
- 它给下游的：`/px4_control/goto`（唯一输出）、`/path_record/visualization`（仅 RViz 可视化，无控制作用）。
- 编译期依赖：`px4_control`（只要 `msg/GotoTarget`）、`px4_msgs`、`geometry_msgs`、`nav_msgs`、`rclcpp`、`std_msgs`（`CMakeLists.txt:14-20`、`package.xml:13-18`）。
- 与 `px4_control/README.md:4-5` 声明的"依赖方向只有 `px4_path -> px4_control`"一致 ✅。

---

## 2. 编译真相、可执行文件与死代码（任务重点问题）

### 2.1 CMakeLists 到底编译谁（`CMakeLists.txt:22-54`）

| 产物 | 类型 | 编译的源文件 | 结论 |
|---|---|---|---|
| `px4_path_core` | 静态库 | `src/motion_trajectory.cpp`、`src/path_loader.cpp` | 被 `path_follow_node` 链接（`:54`） |
| `path_record_node` | 可执行 | `src/path_record_node.cpp`、`src/path_record_process.cpp` | 与 core **不链接**，录制不使用 loader |
| `path_follow_node` | 可执行 | `src/path_follow.cpp`、`src/path_follow_waypoint_node.cpp`、`src/path_follow_trajectory_node.cpp` | 链接 core |

**7 个 .cpp 全部被编译进 2 个可执行文件，没有一个是"未参与编译"的死源码。** `path_follow.cpp` 是模式选择入口，两个 `path_follow_*_node.cpp`/`motion_trajectory.cpp`/`path_loader.cpp` 都是活代码。

机上 `install/` 只有 `path_record_node` 与 `path_follow_node` 两个可执行文件 —— 与上表**完全吻合**，说明机上装的就是这份源码编出来的（`include/` 与 `launch/`、`config/` 也按 `:56-71` 安装）。

### 2.2 真正的死代码 / 永不生效的代码

| 位置 | 现象 | 说明 |
|---|---|---|
| `CMakeLists.txt:73-77` | `if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/path") install(DIRECTORY path ...)` | **永不生效**：真实路径文件目录是 `src/path/`，包根下没有 `path/`。所以 `recorded_path*.txt` 永远不会被装进 `share/px4_path/`。 |
| `src/path_follow_trajectory_node.cpp:8`、`src/path_follow_waypoint_node.cpp:8` | `using namespace std::chrono_literals;` | 未使用（代码全程写 `std::chrono::milliseconds(...)`），纯噪声。 |
| `src/path_record_node.cpp:59` | 注释"空字符串表示使用 share/px4_path/path" | 与代码矛盾：`:136-138` 明确把空 `path_dir` 判为非法并拒绝录制。注释是过期残留。 |
| `include/px4_path/path_loader.hpp:27-30` | `z_min_enu` / `z_max_enu` / `quat_norm_min` / `quat_norm_max` | 两个调用者只覆盖 `max_neighbor_dist`（`src/path_follow_trajectory_node.cpp:175-176`、`src/path_follow_waypoint_node.cpp:154-155`），这四个值事实上是硬编码常量，YAML 改不动。 |

**没有发现 ego-planner 或任何第三方轨迹库残留**：对 `px4_path/` 全量正则搜索 `ego|bspline|b_spline|polynomial|uniform_bspline|traj_utils|MINCO|minimum_snap`，唯一命中是 `src/path_follow_waypoint_node.cpp:278` 的一句注释里提到的 PX4 参数名 `MPC_XY_VEL_MAX`。`MotionTrajectory` 是手写的自然三次样条（见 §5），**不是** ego-planner 的 `UniformBspline`/`polynomial_traj` 那套（后者用 B 样条/多项式闭式解，本包用 Thomas 算法解三对角 + 均匀弧长重采样），两者在数据结构上无可复用痕迹。详见 §9。

### 2.3 代码重复（不是死代码，但同源复制痕迹明显）

waypoint 与 trajectory 两个节点各自实现了一遍**几乎逐字相同**的模板代码，约 150 行：

| 功能 | waypoint | trajectory |
|---|---|---|
| `state_name` | `src/path_follow_waypoint_node.cpp:71-85` | `src/path_follow_trajectory_node.cpp:96-111` |
| `set_state` | `:87-95` | `:113-121` |
| `distance` | `:97-102` | `:123-129` |
| `prepare_path_directory` | `:119-141` | `:141-163` |
| `reload_path` | `:143-176` | `:165-195` |
| `publish_stop` | `:195-199` | `:387-391` |
| `on_local_position` | `:227-239` | `:409-428` |
| `on_goto_enabled` | `:241-258` | `:430-448` |

两个 `enum class`（`include/px4_path/path_follow_waypoint_node.hpp:18-24` 与 `path_follow_trajectory_node.hpp:19-25`）成员完全相同。**建议**：抽 `PathFollowBase`（持有 `state_/paused_from_/points_/goto_enabled_/pub_goto_` 与上述方法），两套算法只实现 `activate/on_tick`。

---

## 3. 可执行节点明细

### 3.1 `path_record_node`（源码：`src/path_record_node.cpp` + `src/path_record_process.cpp`，`main` 在 `:488-495`）

职责：订阅 SLAM 里程计 → 基础位姿校验 → 可选在线滤波 → 关键点抽取 → 缓存 → 退出时补末点 + 平滑 + 压缩 → 原子写文件；同时把整条缓存发到 RViz。

| 方向 | 话题 | 类型 | QoS（源码位置） |
|---|---|---|---|
| 订阅 | `slam_topic`（默认 `/localization`） | `nav_msgs/msg/Odometry` | `rclcpp::SensorDataQoS()`（best_effort, depth 5）`:93-95` |
| 发布 | `visualization_topic`（默认 `/path_record/visualization`） | `nav_msgs/msg/Path` | `QoS(1).reliable().transient_local()` `:97-98` |

- **无定时器**。所有工作由里程计回调驱动（事件驱动，`:387-484`）。
- 该节点**不订阅任何命令话题**、不响应键盘、不读路径文件（`README.md:34-36` 与代码一致 ✅）。
- 参数：见 §6 表（`slam_topic`、`expected_frame_id`、`visualization_topic`、`path_dir`、`output_file`、`overwrite_existing`、`enable_filter_smoothing`、`min_xy_m`、`min_z_m`、`max_neighbor_dist_m`、`min_record_interval_s` + 处理器参数 `filter_window`、`smoothing_window`、`min_smoothed_point_dist_m`、`min_points`）。
- 启动即录制（`:106-107`），**正常退出**（Ctrl-C → `spin` 返回 → `recorder.reset()` → 析构 `:114-118` → `stop_recording("node shutdown")`）时才落盘。

### 3.2 `path_follow_node`（源码：`src/path_follow.cpp` + 二选一的算法文件，`main`/模式选择在 `src/path_follow.cpp:18-42`）

- 进程启动先建一个**临时节点**（名字硬编码 `path_follow_node`）读 `follow_mode`（`src/path_follow.cpp:11-14`），随后析构它，再建真正的算法节点（同为 `path_follow_node` 名字，`:25/:27`）。参数覆盖走的是进程级 `--ros-args --params-file`（由 `launch/px4_path.launch.py:31-37` 传入），所以两个同名节点都能拿到 YAML 段。
- `follow_mode` 非 `waypoint`/`trajectory` 时打印 FATAL、`shutdown()`、返回 2（`:28-34`）。
- 两套算法共享的 ROS 接口：

| 方向 | 话题 | 类型 | QoS |
|---|---|---|---|
| 订阅 | `/fmu/out/vehicle_local_position` | `px4_msgs/msg/VehicleLocalPosition` | `KeepLast(5).best_effort()`（`path_follow_trajectory_node.cpp:45-50`、`path_follow_waypoint_node.cpp:24-29`） |
| 订阅 | `/px4_control/goto_enabled` | `std_msgs/msg/Bool` | `QoS(1).reliable().transient_local()`（`:52-55` / `:31-34`）—— **锁存** |
| 发布 | `/px4_control/goto` | `px4_control/msg/GotoTarget` | `KeepLast(10)`，默认 reliable/volatile（`:57-58` / `:36-37`） |

- QoS 与 `px4_control` 侧严格配对：`px4_control/src/px4_control_node.cpp:62-64`（goto 订阅，depth 10 默认）、`:73-74`（goto_enabled 发布，reliable+transient_local）✅。
- 定时器：`create_wall_timer(milliseconds(int(1000/max(1.0,pursuit_rate))), on_timer)`，trajectory `:60-63`、waypoint `:39-42`；YAML 里 `pursuit_rate: 20.0` → **50 ms / 20 Hz**。
- 参数见 §6。

### 3.3 waypoint 算法节点（`include/px4_path/path_follow_waypoint_node.hpp` + `src/path_follow_waypoint_node.cpp`）

- 状态：`INIT→LOADED→PURSUING→(PAUSED)↔(HOLD)`（`path_follow_waypoint_node.hpp:18-24`）。
- 每 tick（`src/path_follow_waypoint_node.cpp:260-293`）：`while (cursor_+1 < size && dist(current, points_[cursor_]) < switch_tol) ++cursor_;`（`:268-273`）→ `clamp_step` 限制单步 ≤ `max_step_dist`（`:104-117`，`d<1e-6` 时直接返回目标，**无除零**）→ `publish_goto`（`:178-193`，`vx=vy=vz=ax=ay=az=yawspeed=0`，`yaw = wp.yaw`）→ 末端判定（`:284-287`）。`HOLD` 时每 tick 重发终点（`:288-292`）。
- `current_pos_` 只存 x/y/z，**yaw 域不写**（`path_follow_waypoint_node.hpp:76` 注释与 `:229-232` 一致）。

### 3.4 trajectory 算法节点（`include/px4_path/path_follow_trajectory_node.hpp` + `src/path_follow_trajectory_node.cpp`）

- 状态：`INIT→LOADED→PURSUING→(PAUSED)↔(HOLD)`（`path_follow_trajectory_node.hpp:19-25`）。
- 每 tick（`src/path_follow_trajectory_node.cpp:450-588`）：算跟踪误差 → 定参考时钟缩放 → 加速度可行域夹紧 → 推进 `trajectory_elapsed_s_` → 采样 → 超限中止判定 → 发布 → 分段切换/终点判定。
- 发布内容：位置 + 速度 + 加速度 + yaw + yawspeed（`publish_reference`，`:336-385`）。

---

## 4. 状态机与完整工作流（人做了什么 → 飞机怎么动）

### 4.1 录制链路

```text
① 人：改 params.yaml record_mode: true（params.yaml:8），ros2 launch px4_path px4_path.launch.py
   → launch 读 YAML 里 px4_path_launch.record_mode（launch/px4_path.launch.py:11-27）
   → 只启动 path_record_node
② 节点构造即 recording_=true（path_record_node.cpp:106-107），并先发一条空 Path 清 RViz（:210）
③ 每帧 /localization：
   frame_id 检查（:392-399，expected_frame_id 非空且不符 → 丢弃；录制中 frame 变化 → 立即停录并写盘 :403-408）
   → NaN/Inf + 四元数归一（:421-425；归一失败丢帧）
   → 原始跳变检查 raw_step > max_neighbor_dist_m → 丢帧（:429-438，注释解释必须先于滤波，否则大跳变会被均值拆成小位移绕过检查 —— 这个顺序是对的）
   → 可选在线滑动均值 filter()（:442-443）
   → 关键点判定：空则首点；否则 elapsed < min_record_interval_s 丢弃；否则
     距离 > max_neighbor 丢弃；否则 (水平 ≥ min_xy_m 或 竖直 ≥ min_z_m) 才记录（:446-483）
④ 人：Ctrl-C → 析构 → stop_recording（:253-293）
   → append_final_pose 补末点（:224-251，末点若跳变 > max_neighbor 则警告并丢弃）
   → 点数 < min_points 不写盘（:260-267）
   → finalize = 中心滑动平均 smooth_path + 近点压缩 remove_close_points（path_record_process.cpp:74-79）
   → 压缩后又 < min_points 不写盘（:278-285）
   → write_path_file：先做相邻距离终检（:304-313）→ 写 .tmp → flush/good → close → fs::rename 原子替换（:315-360）
```

文件格式（`path_record_node.cpp:324-341`）：8 行 `#` 头 + 每行 `x y z qx qy qz qw`，`fixed` 9 位小数，**保存 SLAM 原始坐标**（不做 ENU/NED）。实样（`src/path/recorded_path.txt` 前 10 行）确认头里带 `source_frame: map`、`sampling: min_xy=0.12m ... max_neighbor=5m`。

### 4.2 加载链路（只在**节点构造时**发生一次）

```text
prepare_path_directory（trajectory :141-163；waypoint :119-141）
  path_dir 空 → 报错 return false，节点永久留在 INIT，不发布任何东西（:65-70）—— 失效安全 ✅
  path_dir 不存在 → create_directories
  resolved_path_file_ = path_dir + "/" + path_file_
reload_path（trajectory :165-195；waypoint :143-176）
  max_neighbor_dist_m 非法 → 清空 points_ 并拒绝（:166-173）—— 防止"参数写错却继续飞"
  load_path（path_loader.cpp:47-158）：逐行 trim → 跳空行/`#`；必须正好 7 个浮点，多一个 token 即整条失败（:76-82）
    → SLAM→ENU→NED；z_enu 越界只 warning（:89-94）；|q| 越界 warning 但仍归一化（:96-115）
    → 点数 < min_points 整条失败（:132-138）；相邻点 > max_neighbor 整条失败（:142-154）
```

**关键事实：`reload_path()` 只在构造函数里被调用一次**（`src/path_follow_trajectory_node.cpp:72`、`src/path_follow_waypoint_node.cpp:51`），此后**没有任何重读入口**（见风险 R-09）。

### 4.3 跟随链路（含人的动作）

```text
① 人：终端 1 ros2 launch px4_control px4_control.launch.py；终端 2 ros2 launch px4_control px4_teleop.launch.py
② 人：终端 3 ros2 launch px4_path px4_path.launch.py（record_mode: false）
   （px4_control 构造时已 publish_goto_enabled(false)，锁存为 false；跟随节点锁定 INIT/LOADED 不发布）
③ 人按 '0' → CMD_OFFBOARD（px4_teleop_node.cpp:106-109）→ px4_control 先攒够 heartbeat_before_offboard=20 再发 DO_SET_MODE(Offboard)
④ 人按 '1' → CMD_ARM → px4_control 要求 on_ground + is_offboard + local_pos_valid 才 arm，
   并把 target_.z 设为 -takeoff_height（NED 负=上），状态 ARMING→TAKEOFF→(到高)HOVER
   ★ 跟随节点此期间已经在跑：一旦 /fmu/out/vehicle_local_position 有效且 points_ 非空 → INIT→LOADED
     （trajectory :423-426；waypoint :234-237），但 goto_enabled_ 仍 false，所以一帧都不发 ✅
⑤ 人按 '4' → CMD_GOTO_ENABLE → px4_control.handle_goto_enable（px4_control_node.cpp:468-484）
   门槛：state==HOVER 且 local_pos_valid；通过则 goto_enabled_=true、pursuit_active_=false、publish_goto_enabled(true)
⑥ 跟随节点 on_goto_enabled(true)（trajectory :430-448）
   状态==PAUSED → 只重置 timer 基准并回到 paused_from_，**不重建轨迹、不累计暂停时长**
   否则 activate_if_ready（:393-403）：要求 goto_enabled_ && local_pos_valid_ && state==LOADED
   → start_trajectory（:249-334）：
       参数合法性总检（:253-274，含 tracking_pause > tracking_slowdown、tracking_pause < max_tracking_error）
       yaw_mode 必须是 tangent/recorded/fixed（:275-280）
       找急弯分割点 find_path_turn_splits（:288-294）
       make_execution_segments（:197-247）：第 0 段前面插入"当前位置→路径首点"的连接点（步长 max_step_dist，
         位置线性插值、yaw 按 wrap_pi 最短弧插值），再按急弯切段（切点两侧共享，故 `begin_index = end_index`）
       每段 MotionTrajectory::build（:301-322），任一段失败 → 清空所有段并 return false
       trajectory_start_time_/last_timer_time_/trajectory_elapsed_s_=0/scale=1/command_yaw_=当前 heading（:324-328）
   → 状态 LOADED→PURSUING
⑦ 定时器 20Hz：见 §5.4 的逐步逻辑；输出到 /px4_control/goto
⑧ px4_control.on_goto（px4_control_node.cpp:285-333）：active=false → 立即关许可并悬停；
   goto_enabled_ 为 false → 直接忽略；state!=HOVER 或位置无效 → 忽略；
   任一字段 NaN/Inf → 报错 + 关许可；否则接受为 target_ + 前馈，pursuit_active_=true，记录 last_goto_stamp_
⑨ 人按 '5' → CMD_GOTO_DISABLE → px4_control.disable_goto_and_hold（:486-488 → :152-171）：
   清 goto_enabled_/pursuit_active_/前馈，HOVER 下把 target_ 钉在当前位姿，publish_goto_enabled(false)
   → 跟随节点 on_goto_enabled(false)：PURSUING 或 HOLD → paused_from_=当前态，进入 PAUSED，**停止发布、冻结进度**
⑩ 人再按 '4' → 许可重新为 true → 从冻结处继续（游标 / 轨迹时间都不回退）
⑪ 终止：trajectory 末段到点且误差 < terminal_tol → HOLD，持续重发终点（:581-587）；
   waypoint 末点且 < terminal_tol → HOLD（:284-287 / :288-292）
   中途不可恢复错误 → publish_stop() 发 GotoTarget(active=false) → px4_control 关许可并悬停 → 跟随节点退回 LOADED
   （这条路径是**唯一**能让跟随节点"重新从当前位置开始"的入口：再按 '4' 会重新 start_trajectory）
```

**因果链的"人按键"证据**：`px4_teleop_node.cpp:106-160`（'0' Offboard、'1' Arm、'2' Disarm、'3' Land、'4' GOTO_ENABLE、'5' GOTO_DISABLE、WASDQE 平移、L/R 偏航），常量定义在 `px4_control/msg/ControlCommand.msg:7-15`。

**许可与抢占语义**：`/px4_control/goto_enabled` 是**唯一**的启动闸门，跟随节点无法自行开闸；反向地，`GotoTarget(active=false)` 是跟随节点**唯一**能"夺走"许可的手段（`px4_control_node.cpp:288-291` 对 active=false **不检查 goto_enabled_**，无条件关许可）。因此不存在"Goto 许可被跟随节点抢占后仍在发目标"的情况；但存在状态机不同步风险（R-04）。

---

## 5. 算法：`motion_trajectory.cpp`（527 行）

### 5.1 轨迹生成方法（结论：**插值型自然三次样条 + 弧长参数化 + 曲率限速 + 前向/后向加速度约束 + 夹紧三次时间样条 + 全局时间拉伸**）

不是多项式最优控制（无 QP），不是 B 样条（无控制点/阶数概念），不是采样式（非 A*/RRT 后处理）。

1. **预处理去重**（`:245-268`）：相邻距离 < `kMinPointDistanceM = 1e-2`（`:11`）的点被合并（后者的 yaw 覆盖前者的 yaw），保证样条节点严格递增。非有限值直接失败。
2. **空间样条**（`:270-307`）：以**累积弦长** `knots[i] = knots[i-1] + |P_i - P_{i-1}|`（`:290-293`）为参数，对 x(s)、y(s)、z(s) 各建一条 `NaturalCubicSpline::build`（自然边界，M''=0，Thomas 算法解三对角，`:67-119`）。yaw 单独建一条**解缠（unwrap）后**的样条（`:284-289`），避免 ±π 跳变。因此位置是 **C² 连续**（段内三次多项式，节点处一阶/二阶连续）。
3. **曲率限速**（`:312-350`）：在 `profile_count = max(2, 2N, ceil(len/profile_resolution_m)+1)` 个**均匀弧长**采样点上算 `κ = |P'×P''| / |P'|³`（`:334-335`），令 `v ≤ sqrt(max_centripetal_accel_mps2 / κ)`（`:336-339`）。采样点若 `|P'| < 1e-6` → 直接判定样条退化并失败（`:328-332`）。
4. **前向/后向加速度约束**（`:352-363`）：首末速度强制 0（`:352-353`），
   前向 `v_i = min(v_i, sqrt(v_{i-1}² + 2a·Δs_i))`（`:354-358`），后向同式反向（`:359-363`），Δs 用弦长（≤真实弧长 → 约束偏保守，安全方向）。
5. **时间参数化**（`:365-379`）：梯形速度假设 `Δt_i = 2Δs_i / (v_i + v_{i+1})`；当 `v_i+v_{i+1}` 过小时退化为 `sqrt(2Δs/a)`（`:369-372`），并把每段时长下限夹到 1 ms（`:378`）。得到节点对 `(t_i, s_i)`。
6. **时间样条 s(t)**（`:473-480`）：`build_clamped(knots=t_i, values=s_i, 首斜率=0, 末斜率=0)` —— 夹紧三次样条，保证首末速度为 0；同时 `duration_s_ = t_last`。时间节点必须严格递增，否则 `build_clamped` 返回 false → 整段失败（`motion_trajectory.cpp:124-133`）。
7. **全量限值测量**（`:482-518`）：在 `max(2000, 40·节点数)` 个时间采样点上，用链式法则算
   `v = P'(s)·ṡ`（`:443`）、`a = P''(s)·ṡ² + P'(s)·s̈`（`:444-446`）、`j = P'''(s)·ṡ³ + 3P''(s)·ṡ·s̈ + P'(s)·s⃛`（`:447-450`，**jerk 只用于校验，不发布**），并对每个点重算向心加速度 `κ·|v|²`（`:507-516`）。若采样到负速度（时间分配非单调）→ 四个限值全部置 `+inf` 并提前返回（`:492-499`），由调用方判非有限而失败（`:386-392`）。
8. **全局时间拉伸**（`:393-409`）：`scale = max(1, v_ratio, sqrt(a_ratio), cbrt(j_ratio), sqrt(acent_ratio))`，再乘安全系数 `kLimitSafetyScale = 1.001`（`:13`），把所有时间节点乘 `scale` 后重建样条并重测。数学上正确：时间乘 k ⇒ 速度 ÷k、加速度 ÷k²、jerk ÷k³、向心 ÷k²，所以取四个比例的最大值即可同时满足四项。

### 5.2 时间参数化与约束汇总

| 约束 | 参数 | 强制点 |
|---|---|---|
| 速度上限 | `max_speed_mps` | 初始化 `speed[]`（`:320`）+ 拉伸校验 |
| 加速度上限 | `max_accel_mps2` | 前向/后向两遍（`:354-363`）+ 拉伸校验 |
| jerk 上限 | `max_jerk_mps3` | 只测量（`:505-506`）→ 参与拉伸（`:396`）→ **不发给 PX4** |
| 向心加速度上限 | `max_centripetal_accel_mps2` | 曲率限速（`:337-338`）+ 拉伸校验 |
| 首末速度 0 | 硬编码 | `:352-353`（停到停，每段都是） |
| 空间分辨率 | `profile_resolution_m` | `profile_count`（`:312-316`） |

**注意**：拉伸后**没有再断言**测量值是否真的落在限值内（`:408` 调了 `measure_limits()` 但结果只被写入成员、无人校验）。按 §5.1.8 的数学推导不会越界，但代码层面这是一个"验证缺口"。

### 5.3 急弯分割 `find_path_turn_splits`（`:28-65`）

对每个内部点算入/出方向夹角的补角 `angle = acos(clamp(cos))`（`:54-59`），`≥ threshold_deg` 记一个切点（`:60-62`）。`threshold_deg ≤ 0` 或 `>180` 或点数 <3 → 返回空（`:32-35`，即"关闭自动拆段"）。**该函数在一次 `start_trajectory` 里被调用两次**（`:204` 在 `make_execution_segments` 内、`:289` 在 `start_trajectory` 里为了打日志），纯重复计算。

### 5.4 跟随是"位置环 + 前馈 + 参考时钟重定时"，**没有本地位姿闭环**

`on_timer`（`src/path_follow_trajectory_node.cpp:450-588`）逐步：

1. `!local_pos_valid_` → 直接 return，**不发任何消息**（`:451-453`）。
2. 状态必须是 PURSUING，且 `active_segment_index_ < trajectories_.size()`，否则报错退回 LOADED 并发 `active=false`（`:455-462`）。
3. `timer_dt = now - last_timer_time_`，若 `≤0` 或 `>0.5 s` 则用标称周期（`:464-469`）。
4. 用**当前参考点**算三维误差 `current_error`（`:471-481`）。
5. 参考时钟缩放（`:483-490`）：误差 ≥ `tracking_pause_error_m` → 目标 `scale=0`（**冻结参考**）；介于 slowdown 与 pause 之间 → 线性降到 0；否则 1。变化率受 `max_time_scale_rate_per_s` 限制（`:491-494`）。
6. **重定时下的加速度可行域夹紧**（`:496-523`）：链式法则 `a_retimed = a·k² + v·k̇`，把 `k̇` 限制在使 `|a_retimed| ≤ max_accel_mps2` 的区间（解一个关于 `k̇` 的二次不等式；判别式 <0 时把 `k̇` 置 0，保守冻结）。这是本文件里技术含量最高、也是最正确的一段。
7. 推进 `trajectory_elapsed_s_ += 0.5(k_prev+k_new)·dt`，并夹在 `duration()` 之内（`:524-530`）。
8. 采样并算真实跟踪误差；`max_tracking_error_m > 0 且超限` → 报错、退回 LOADED、发 `active=false`（`:532-545`）。
9. 发布（`:547-554`）：段末（`elapsed ≥ duration`）用 `scale=0, k̇=0` 发布（即速度/加速度前馈全 0 的终点保持）；否则带 `scale, k̇`。
10. 段末且误差 < `terminal_tol` → 若还有下一段，要求 `current_speed_mps_ ≤ segment_switch_speed_mps` 才切段并重置时间/scale（`:556-576`，不满足则按 1 Hz 打节流日志等待）；否则进入 HOLD（`:577-580`）。

**结论**：本包**没有任何反馈控制器**。位置环完全由 PX4 承担；本包只提供 (p, v, a, yaw, yawspeed) 参考，并用一个**标量**参考时钟缩放做"沿轨迹方向的追赶调制"。它**不能**做横向纠偏（没有任何横向误差项进入输出），横向偏差只能靠 PX4 的位置环。这一点在 §10 R-02 会变成一个安全问题。

---

## 6. 坐标系、单位与转换点

### 6.1 转换点只有一个：`path_loader.cpp`

```text
SLAM → ENU   (x,y,z) → (-x, -y, z)                path_loader.cpp:30-32
ENU  → NED   (x,y,z) → (y, x, -z)                 path_loader.cpp:34
yaw:  yaw_slam = atan2(2(qw qz + qx qy), 1 - 2(qy²+qz²))   :119-120
      yaw_enu  = wrap_pi(yaw_slam + π)                      :121
      yaw_ned  = wrap_pi(π/2 - yaw_enu)                     :122
```

录制侧**完全不做**坐标转换（`path_record_node.cpp:48` 注释 + `path_record_process.hpp:12` 注释 + 代码事实），可视化也保持 SLAM frame（`:366-368` 用 `source_frame_id_`）。

### 6.2 与 `px4_odom_convert` 的一致性（**已核对，一致** ✅）

| 项 | px4_odom_convert | px4_path loader | 判定 |
|---|---|---|---|
| SLAM→ENU 位置 | `R_enu_slam_ = diag(-1,-1,1)`，`t=0`（`px4_odom_convert.cpp:20-28, 127-129`） | `(-x,-y,z)` | **完全一致** |
| ENU→NED 位置 | `ft::enu_to_ned_local_frame`（`px4_odom_convert.cpp:151-154`）= 静态阵 `ENU_TO_NED` 左乘 | `(y, x, -z)` | 静态阵是 `[[0,1,0],[1,0,0],[0,0,-1]]`，**一致** |
| yaw 提取式 | `atan2(2(q0q3+q1q2), 1-2(q2²+q3²))`（`px4_ros_com/src/lib/frame_transforms.cpp:105`） | 同一式子（`path_loader.cpp:120`） | **逐字一致** |
| ENU→NED yaw | `transform_orientation(ENU_TO_NED): out = NED_ENU_Q * q`（`frame_transforms.cpp:157-166`，**左乘**参考系变换） | `π/2 − yaw_enu` | 左乘参考系变换对纯偏航即 `yaw_ned = π/2 − yaw_enu`（East→+90°，North→0），**一致** |
| FLU↔FRD | `out = q * AIRCRAFT_BASELINK_Q`（`:167-170`，**右乘** 180° 滚转） | 不涉及（只取 yaw） | 右乘 180° 滚转是纯 roll，**不改变 yaw 值** → 一致 |

另外 `/fmu/out/vehicle_local_position.heading` 的官方语义（`src/px4_msgs/msg/VehicleLocalPosition.msg:41`）是"相对 NED 的欧拉偏航，**-PI..+PI**"，与 loader 产出的 `[-π,π]` 同域同向 ✅。

**两点补充（都已核算，供交叉引用）**：
1. 同一目录下已有的数值验证脚本 `fc-audit/verify_frames.py` 独立确认了"位置与姿态共用同一个 `R_total = R_NED_ENU · R_ENU_SLAM`"，且 `NED_ENU_Q`（`frame_transforms.h:173`，`quaternion_from_euler(π,0,π/2)`）的旋转矩阵恰为 `[[0,1,0],[1,0,0],[0,0,-1]]`（det=+1，是正当旋转而非反射），与本笔记 §6.2 的判定一致、无冲突。
2. loader 的 yaw 转换对**任意姿态**都精确，不是小角度近似：设 `C = Rz(ψ)Ry(θ)Rx(φ)`，左乘 `Rz(π)` 只把 yaw 变 `ψ+π`；`M = [[0,1,0],[1,0,0],[0,0,-1]]` 左乘后 `yaw(M·C) = atan2(C11, C21) = atan2(cosψcosθ, sinψcosθ) = π/2 − ψ`（`|θ|<90°` 下成立）；右乘 `Rx(π)`（FLU↔FRD）只把 roll 变 `φ+π`，yaw 不变。因此 `path_loader.cpp:121-122` 的 `yaw_ned = wrap(π/2 − wrap(yaw_slam + π))` 与 `px4_odom_convert` 姿态链的 yaw 分量逐值相等。数值抽查：`ψ=0`（体系 x 沿 SLAM +x）→ 链式 `atan2(-1,0) = −π/2`，loader 亦得 `−π/2`；`ψ=π/2` → 两者同为 `π`。

**唯一耦合条件**：该一致性成立的前提是 `px4_odom_convert` 的 `z_offset_m` 保持 `0.0`（`launch/px4_odom_convert.launch.py:54-56` 默认 `0.0`，默认下满足）。**loader 里没有对应的 z 偏移项**，一旦有人给 `z_offset_m` 填非零值，路径高度就会整体差该偏移量（见 R-06）。

### 6.3 单位与帧约定总表

| 量 | 单位/帧 | 位置 |
|---|---|---|
| 路径点位置 | m，PX4 **NED**（z 正=下），加载后 | `path_loader.hpp:11-16` |
| 文件里的位置 | m，FAST-LIO **SLAM** 原始帧 | `path_record_node.cpp:324-341` |
| `yaw` | rad，NED 航向（0=北，顺时针为正），`[-π,π]` | `path_loader.hpp:15`、`path_loader.cpp:122` |
| `yawspeed` | rad/s | `GotoTarget.msg:18` |
| 速度/加速度/jerk | m/s、m/s²、m/s³（三维范数约束） | `params.yaml:47-55` |
| `segment_split_turn_deg` | **度**（与其它角度参数不同单位，易错） | `params.yaml:64-66`、`motion_trajectory.hpp:32` |
| `profile_resolution_m`、各种 `*_m`/`*_dist` | m | `params.yaml:60-84` |
| 时间 | s | 全部 |
| `GotoTarget.x/y/z` | m，NED | `px4_control/msg/GotoTarget.msg:1-11` |

**yaw 双域风险点**：trajectory 节点用 PX4 `heading`（`[-π,π]`）初始化 `command_yaw_`（`:328`），再用 loader 的 NED yaw 做目标（`recorded` 模式 `:349-350`）、用 `atan2(dy,dx)` 做 `tangent` 模式目标（`:351-359`，也是 NED 习惯：atan2(北,东)… 注意这里是 `atan2(dy, dx)`，在 NED 下 dx=北增量、dy=东增量，故结果正是"从北顺时针"的 NED 航向 ✅）。三者同域，**不会出现 ±π 混用错误**。

---

## 7. `config/params.yaml` 逐条解释

> "代码默认"列取自 `declare_parameter` 的默认值；**代码默认 ≠ YAML 值**的项已用 ⚠ 标出（只影响不带 params_file 直接 `ros2 run` 的场景，此时 `path_dir` 为空 → 节点拒绝工作，属失效安全）。

### 7.1 `px4_path_launch`（`params.yaml:5-8`）

| 参数 | YAML | 代码默认 | 含义 / 建议 |
|---|---|---|---|
| `record_mode` | `false` | 无（launch 直接读 YAML） | `true` 只起 `path_record_node`，`false` 只起 `path_follow_node`。**必须显式写成 YAML 布尔**，否则 launch 抛 `RuntimeError`（`launch/px4_path.launch.py:21-24`）。不支持运行中切换。 |

### 7.2 `path_follow_node`（`params.yaml:10-96`）

| 参数 | YAML | 代码默认（traj / wp） | 含义、单位、取值建议 |
|---|---|---|---|
| `follow_mode` | `waypoint` | `trajectory`（`path_follow.cpp:13`） | 二选一。`waypoint` 简单但走停式；`trajectory` 有速度/加速度前馈、更顺。**文档默认值与 YAML 相反，注意别照文档理解现场行为。** |
| `path_dir` ⚠ | `/home/duzhong/dzpx4/src/px4_path/src/path` | `""` | 绝对路径，**必须与录制段一致**。硬编码到具体用户名/工作区，换机必改。 |
| `path_file` ⚠ | `recorded_path_2.txt` | `recorded_path.txt` | 只写文件名。**必须与录制段 `output_file` 一致 —— 当前 YAML 里并不一致**（见 R-01）。 |
| `pursuit_rate` ⚠ | `20.0` | `50.0` / `10.0` | GotoTarget 发布频率 Hz。必须明显高于 `px4_control.pursuit_goto_timeout`(1.0 s) 的倒数；20 Hz ≫ 1 Hz ✅。建议 10~50；**>1000 会让定时器周期取整成 0 ms**（R-12）。 |
| `switch_tol` ⚠ | `0.10` | `0.30` | **仅 waypoint**：三维距离小于它就推进游标，m。太小 → PX4 稳态误差可能永远进不去，路径卡死；太大 → 拐角被切。建议 0.10~0.30，需实测稳态误差。 |
| `terminal_tol` ⚠ | `0.15` | `0.20` / `0.20` | 到终点小于它就进 HOLD，m。太小难收敛、太大提前结束。 |
| `max_step_dist` | `1.5` | `1.5` | waypoint：单次位置目标最大距离；trajectory：**连接段**（当前位置→路径首点）的插值步长，m。**注意它并不限制连接段总长**（R-03）。 |
| `max_neighbor_dist_m` | `5.0` | `5.0` | 加载时相邻点硬门限，m；超限**整条路径拒绝**。应与录制段同名参数一致。同时被 `find_path_turn_splits` 之外的跳变语义复用。 |
| `max_speed_mps` ⚠ | `0.8` | `0.5` | trajectory 参考速度上限 m/s。**只约束参考，不约束飞机**（R-02）。 |
| `max_accel_mps2` ⚠ | `1.0` | `0.5` | 同上，加速度上限 m/s²；同时用于前向/后向速度曲线与重定时夹紧。 |
| `max_jerk_mps3` ⚠ | `2.5` | `1.0` | 只用于测量+时间拉伸，不发布。 |
| `max_centripetal_accel_mps2` | `0.35` | `0.35` | 弯道向心加速度上限 m/s²，越小弯道降速越明显。 |
| `profile_resolution_m` ⚠ | `0.12` | `0.10` | 曲率/速度曲线空间采样间隔 m。越小越能抓到短急弯，构建越贵（复杂度见 R-14）。 |
| `segment_split_turn_deg` ⚠ | `140.0` | `150.0` | 转角 ≥ 该值就拆成"停到停"段，**度**。0 关闭拆段。 |
| `segment_switch_speed_mps` ⚠ | `0.10` | `0.15` | 切段前要求实际速度 ≤ 该值 m/s（防高速切弯）。 |
| `tracking_slowdown_error_m` ⚠ | `1.00` | `0.30` | 误差超它开始降参考时钟速度，m。 |
| `tracking_pause_error_m` ⚠ | `1.50` | `0.70` | 误差达它**冻结参考时钟**等飞机，m。必须 > slowdown，且（当 max_tracking_error>0 时）< max_tracking_error，否则启动被拒（`:264-269`）。 |
| `max_time_scale_rate_per_s` ⚠ | `1.2` | `1.0` | 参考时钟缩放比例最大变化率 1/s，防前馈突变。 |
| `max_tracking_error_m` ⚠ | `5.0` | `1.0` | 误差超它发 `active=false` 中止，m；≤0 关闭该保护。**注意误差 ≥ pause 时参考已冻结、飞机不再远离，所以 5 m 这个中止门几乎不会触发**（R-02 相关）。 |
| `yaw_mode` | `fixed` | `fixed` | `fixed`=保持开始跟随时的航向；`recorded`=用文件记录的航向；`tangent`=沿轨迹切线。非法值启动被拒（`:275-280`）。 |
| `yaw_lookahead_s` | `0.5` | `0.5` | 仅 `tangent` 的前视时间 s。 |
| `max_yaw_rate_rps` | `0.5` | `0.5` | 发布 yaw 的最大变化率 rad/s（同时决定 `yawspeed` 前馈）。 |

### 7.3 `path_record_node`（`params.yaml:98-159`）

| 参数 | YAML | 代码默认 | 含义 / 建议 |
|---|---|---|---|
| `slam_topic` | `/localization` | 同 | 必须是 `nav_msgs/msg/Odometry`；类型不符则回调根本不会触发（静默）。 |
| `expected_frame_id` | `""` | 同 | 空=采用首帧；录制中 frame 变化会**立即停录并落盘**（`:403-408`）。 |
| `visualization_topic` | `/path_record/visualization` | 同 | `nav_msgs/Path`，保持 SLAM frame，需 RViz 有对应 TF。 |
| `path_dir` ⚠ | `/home/duzhong/dzpx4/src/px4_path/src/path` | `""` | 必须与跟随段一致；空字符串 = 拒绝录制（不是"用 share 目录"，与 `:59` 注释矛盾）。 |
| `output_file` ⚠ | `recorded_path.txt` | 同 | 只允许文件名（含目录会被判非法，`:142-146`）。 |
| `overwrite_existing` ⚠ | `false` | `true` | `false` 时若同名文件存在，自动找 `_1`、`_2`… 第一个不存在的名字（`:179-189`）。**这是 R-01 的直接诱因**：现场已有 `recorded_path.txt`、`recorded_path_1.txt`、`recorded_path_2.txt`，所以下一次录制会写成 `recorded_path_3.txt`（未确认现场当前是否已存在，见 §12）。 |
| `enable_filter_smoothing` | `true` | 同 | `false` 时 `filter_window`/`smoothing_window`/`min_smoothed_point_dist_m` 不会被声明（参数被静默忽略）。 |
| `min_xy_m` ⚠ | `0.12` | `0.10` | 水平关键点阈值 m。变小更贴原始轨迹。 |
| `min_z_m` | `0.10` | 同 | 竖直阈值 m；水平**或**竖直任一达标即记录（`:475`）。 |
| `max_neighbor_dist_m` | `5.0` | 同 | 原始帧跳变门限 + 最终航点相邻距离门限；应与跟随段一致。 |
| `min_record_interval_s` ⚠ | `0.05` | `0.10` | 关键点最短时间间隔 s；0 关闭限频（`:456` 的 `elapsed >= 0.0 &&` 保证 0 时不禁用）。 |
| `filter_window` | `5` | 同 | 在线滑动均值窗口，1~101；1=不过滤。 |
| `smoothing_window` | `3` | 同 | 保存前中心平滑窗口，1~101（偶数会被 +1 变奇，`path_record_process.cpp:154-156`）；1=不平滑。 |
| `min_smoothed_point_dist_m` | `0.05` | 同 | 平滑后近点压缩阈值 m；0 关闭。 |
| `min_points` | `2` | 同 | 最少点数，必须 ≥2（`:159-161`）。 |

---

## 8. 与 `px4_control` 的接口契约

### 8.1 话题与 QoS（已逐条比对，**全部兼容** ✅）

| 话题 | 方向 | 类型 | px4_path 侧 QoS | px4_control 侧 QoS | 兼容 |
|---|---|---|---|---|---|
| `/px4_control/goto` | path→control | `px4_control/msg/GotoTarget` | pub `KeepLast(10)` 默认 reliable/volatile（`path_follow_trajectory_node.cpp:57-58`） | sub `10` 默认（`px4_control_node.cpp:62-64`） | ✅ |
| `/px4_control/goto_enabled` | control→path | `std_msgs/msg/Bool` | sub `QoS(1).reliable().transient_local()`（`:52-55`） | pub 同（`px4_control_node.cpp:73-74`） | ✅ 锁存 |
| `/fmu/out/vehicle_local_position` | PX4→both | `px4_msgs/msg/VehicleLocalPosition` | sub `KeepLast(5).best_effort()` | sub 同（`px4_control_node.cpp:46-52`） | ✅ |

### 8.2 `GotoTarget` 字段约定（`px4_control/msg/GotoTarget.msg:1-18` + `px4_control_node.cpp:285-333`）

| 字段 | 发送侧约定 | 接收侧行为 |
|---|---|---|
| `active` | `true`=所有字段有效且有限；`false`=立即停止追踪并保持当前位置（其余字段忽略） | `:288-291`：`active=false` → `disable_goto_and_hold()`（**不看当前许可状态，无条件关**）；`active=true` 且许可关闭 → 忽略（`:293-295`） |
| `x/y/z` | m，**PX4 NED**（z 正=下） | `:313-315` 直接作为 `target_`，写入 `TrajectorySetpoint.position`（`:512`） |
| `vx/vy/vz` | m/s，NED，**前馈速度**（不是反馈） | `:317-319` 存前馈；`m.velocity` = 前馈值（`:527-528`），非追踪时填 NaN（`:530`） |
| `ax/ay/az` | m/s²，NED 前馈 | `:320-322`；写入 `m.acceleration`（`:534-535`），非追踪时 NaN（`:537`） |
| `yaw` | rad，NED 航向 `[-π,π]` | `:316` → `target_.yaw` → **总是**写入 `TrajectorySetpoint.yaw`（`:540`，非追踪时也生效） |
| `yawspeed` | rad/s 前馈 | `:323`；写入 `m.yawspeed`（`:541-543`），非追踪时 NaN |

- **有限性要求**：接收侧对 11 个字段逐一 `std::isfinite`（`px4_control_node.cpp:300-311`），任一 NaN/Inf → 报错 + **关许可并悬停**。发送侧 trajectory 保证数值有限（`sample()` 输出经 `measure_limits` 校验过），waypoint 显式填 0 ✅。
- **超时与失效**：`pursuit_goto_timeout = 1.0 s`（`px4_control_node.cpp:43-44`、`config/params.yaml:17`），在定时器里检查 `now - last_goto_stamp_`（`:579-586`）→ 超时则 `disable_goto_and_hold("Goto timeout")` 并悬停。20 Hz 下每 50 ms 一条，余量 20×。
- **失效方向**：跟随节点进程崩了 → 1 s 后 px4_control 关许可悬停 ✅；跟随节点只"停发"不主动发 `active=false` 也能被兜住 ✅。反向（px4_control 挂了）**没有兜底**：跟随节点会一直以为许可有效（锁存值不会变），继续发目标，但没人接收（R-04）。
- **状态门槛**：接收侧要求 `state_ == HOVER && local_pos_valid_`（`:297-299`），而 `goto_enabled_` 只在 HOVER 下才可能被置真（`:468-474`），所以"许可为真"蕴含"曾经处于 HOVER"。
- **键盘互锁**：`goto_enabled_` 为真时 WASDQE 与 L/R 被拒绝（`handle_move` `:437-440`、`handle_yaw` `:454-457`），避免手动指令与路径目标打架 ✅ —— 这就是两个跟随节点在 HOLD 里仍持续重发终点的原因（`src/path_follow_waypoint_node.cpp:289-290` 注释、`src/path_follow_trajectory_node.cpp:582` 注释）。
- **单话题保序**：目标与停止消息走同一话题，保证顺序（`GotoTarget.msg:5`）✅。

---

## 9. 未使用/遗留代码汇总

1. **`CMakeLists.txt:73-77`**：`install(DIRECTORY path)` 条件永假（真实目录 `src/path`）→ 路径样例/录制结果永不安装。若把包部署到别的机器，`share/px4_path/path` 下不会有任何文件，而 `params.yaml` 又指向源码目录 → 路径读取必然失败（失效安全，不会误飞，但功能不可用）。
2. **注释残留**：`src/path_record_node.cpp:59`（"空字符串表示使用 share/px4_path/path"）与实际拒绝逻辑矛盾。
3. **`using namespace std::chrono_literals;`**：`src/path_follow_trajectory_node.cpp:8`、`src/path_follow_waypoint_node.cpp:8`，未使用。
4. **参数化外壳无实参**：`LoadOptions` 的 `z_min_enu/z_max_enu/quat_norm_min/quat_norm_max`（`include/px4_path/path_loader.hpp:27-30`）没有任何调用者覆盖 → 名义可配、实际恒定。
5. **重复计算**：`find_path_turn_splits` 双调用（`src/path_follow_trajectory_node.cpp:204, 289`）；`measure_limits()` 双调用（`:385, 408`，每次都在 `max(2000, 40N)` 个点上做 4 次样条求值）。
6. **双节点模板重复**（见 §2.3）。
7. **第三方残留：无**。全包搜索 `ego|bspline|polynomial|uniform_bspline|traj_utils|MINCO|minimum_snap` 无命中；`MotionTrajectory` 与 ego-planner 的轨迹类在算法与数据结构上均无同源迹象（详见 §5.1）。
8. 目录里的三个样例路径文件 `src/path/recorded_path{,_1,_2}.txt` 是**运行产物**（头里带 `source_frame: map`、采样参数），被一起纳入源码树/镜像，属"脏仓库"而非死代码；`src/path/recorded_path_2.txt` 正是 YAML 当前指向的文件。

---

## 10. 风险与缺陷清单（现象 → 触发条件 → 后果 → 建议修法）

### R-01【高危·功能/安全】录制与跟随的文件名契约已经断裂，且路径只在启动时读一次

- 现象：`params.yaml:25` `path_file: "recorded_path_2.txt"`，`params.yaml:119` `output_file: "recorded_path.txt"` —— 违反 `README.md:103-106` 与 `px4_path_README_DELIVERY.md:91-96` 自己声明的"两者必须一致"。且 `overwrite_existing: false`（`params.yaml:123`）会让每次录制自动改名 `_1/_2/_3…`（`src/path_record_node.cpp:179-189`）。
- 触发：按文档流程先录一条新路径（`record_mode: true` 跑一遍、Ctrl-C），再切回跟随。
- 后果：**跟随节点读到的仍是旧的 `recorded_path_2.txt`，人在屏幕上看到"路径已加载"，飞的是上一次的航线**。加载只在构造时发生（`src/path_follow_trajectory_node.cpp:72`、`src/path_follow_waypoint_node.cpp:51`），日志里"press '4' after creating a valid path"（`:76-77`、`:55-56`）**是错的 —— 按 '4' 不会重新读文件**，只会走 `activate_if_ready → start_trajectory` 并在 `points_` 为空时立刻发 `active=false`（`:398-402`）。
- 建议修法：(a) 在 `start_trajectory`/录制启动时**断言 `path_follow_node.path_file == path_record_node.output_file` 且 `path_dir` 相同**，不一致直接拒飞；(b) 记录并打印文件 mtime + 点数 + 首点坐标，让操作者肉眼可辨新旧；(c) 提供显式"重载路径"入口（键或服务），并把日志改成"按 5 后按 4 可重载"之类**真实可用**的指引；(d) 建议 `overwrite_existing: true` + 录制完成后打印最终文件名。

### R-02【高危·安全】跟踪误差靠 PX4 位置环纠正，实际速度不受 `max_speed_mps` 约束

- 现象：参考时钟被冻结时（`tracking_pause_error_m` 判据，`src/path_follow_trajectory_node.cpp:483-490`），发布的是"冻结的参考点 + 速度前馈 0 + 加速度前馈 0"（`publish_reference` `:341-346` 在 `time_scale=0` 时把 v、a 全乘成 0，`:550`/`:585-586` 直接用 scale=0）。px4_control 拿到的是一个距离飞机 1.5 m 以上、但前馈为 0 的位置目标（`px4_control_node.cpp:313-324, 523-538`）。
- 触发：任何让误差超过 `tracking_pause_error_m`（当前 YAML 1.5 m）的情形 —— 起点 join 段、被手动 WASDQE 顶开（不可能，互锁了）、EKF 抖动、`recorded`/`tangent` 模式下航向变化引起的耦合、或 R-03 的自动起飞后。
- 后果：飞机以 **PX4 自己的 `MPC_XY_VEL_MAX` / `MPC_Z_VEL_MAX`**（不是 `max_speed_mps: 0.8`）飞向目标点。也就是说"动力学受限轨迹"的限速**在误差纠正阶段完全失效**，实际速度可能远大于 0.8 m/s。`max_tracking_error_m: 5.0` 的中止门也救不了：参考冻结后飞机只会越来越近，误差不会再涨（所以这个门几乎永不触发）。
- 建议修法：(a) 把误差纠正也纳入限速：把发布位置**夹在参考点前 `k·error` 处**（lead 限制），或按误差生成一个受限的爬升速度前馈（而不是 0）；(b) 在机上把 `MPC_XY_VEL_MAX` 降到与 `max_speed_mps` 同量级（0.5~1.0 m/s）作为**硬兜底**，这是最省事也最可靠的一条；(c) 把 `max_tracking_error_m` 降到与 `tracking_pause_error_m` 同量级（如 1.6~2.0 m）并让它真正可达。

### R-03【高危·安全】没有任何"显式开始"动作；锁存的许可 + 无上限的连接段 = 一启动就自动飞

- 现象：`/px4_control/goto_enabled` 是 `transient_local` 锁存订阅（`src/path_follow_trajectory_node.cpp:52-55`）。跟随节点一旦同时具备"许可为真 + 位置有效 + LOADED"，就在回调里**自动** `start_trajectory()` 并开始发布（`on_goto_enabled` `:447` → `activate_if_ready` `:393-403`；`on_local_position` `:427` 也会触发）。连接段长度**没有任何上限**：只按 `max_step_dist` 打点（`make_execution_segments` `:210-229`），起点离路径首点 200 m 就老老实实插 134 个点、要求飞 200 m。
- 触发：飞机已在 HOVER 且操作者已按过 '4'（许可 true），此时再启动/重启 `px4_path`（例如为了重载路径），或 `px4_path` 崩溃后重启。
- 后果：**操作者以为只是"起了个跟随节点"，飞机立刻开始自主飞行**，且第一段可能是一条任意长的直线（沿途无安全检查、无地理围栏）。
- 建议修法：(a) 增加显式"开始跟随"输入（键/服务/参数 `autostart: false` 默认），要求 `goto_enabled` 为真**且**操作者确认才启动；(b) 给连接段总长设硬上限（如 `max_join_length_m` 默认 ≤ 3 m），超限则拒绝启动并提示"先把飞机放到路径起点附近"；(c) 启动前把首点与当前位姿的三维距离打日志并要求 `max_step_dist` 量级以内。

### R-04【中高危·安全】许可状态机不同步：跟随节点 PUrsuing 而 control 已关许可，恢复时不会重建轨迹

- 现象：跟随节点只在 `state_ ∈ {PURSUING, HOLD}` 时响应 `goto_enabled=false`（`:433-438`）；`goto_enabled=true` 只对 `PAUSED` 或 `LOADED` 有语义（`:442-447` + `activate_if_ready` 要求 `state_==LOADED`）。若在 PURSUING 期间 px4_control 因**自身原因**关掉许可（超时 `px4_control_node.cpp:582`、丢失 Offboard `:242`、位置失效 `:190`），跟随节点的 `goto_enabled_` 会变 false 并**正常进入 PAUSED**（这条路径 OK）；但若跟随节点发出的 `active=false` 让它自己退回 LOADED、随后操作者补按 '4'，`start_trajectory()` 会**从当前位置重建**（正确）；**真正的缺口是没有"control 已死/许可永远不会再变"的看门狗**。
- 触发：`px4_control` 进程崩溃或被杀（锁存值停在最后的 true/false，不会再有消息）；或 build/重建耗时超过 1 s 导致 `Goto timeout`。
- 后果：(a) 若锁存停在 `true`，跟随节点继续发目标而无人接收，屏幕上一切正常；(b) 若某次重建耗时 >1 s 触发 `Goto timeout`，px4_control 关许可悬停，而跟随节点已完成重建处于 PURSUING —— 此时再按 '4'，跟随节点不会重建轨迹，而是**从已经推进过的时间点继续**（`activate_if_ready` 对非 LOADED 直接 no-op），从冻结/滞后状态恢复的瞬间可能出现一次位置跳跃。
- 建议修法：(a) 跟随节点订阅一个 control 心跳（或在 `goto_enabled` 上做"超过 N 秒没消息"的看门狗），control 消失时主动退回 LOADED 并停发；(b) 把 `start_trajectory()` 的重活（样条+限值测量）放到构造期或独立线程，避免在许可回调里阻塞超过 `pursuit_goto_timeout`；(c) 恢复时若误差 > `tracking_pause_error_m`，强制重建轨迹而不是续跑。

### R-05【中·功能】录制只在进程正常退出时落盘

- 现象：`stop_recording` 只由析构函数（`src/path_record_node.cpp:114-118`）和"frame 变化"（`:403-408`）触发，没有任何"停止/保存"命令，也没有周期性落盘。
- 触发：SIGKILL、段错误、断电、`kill -9`、容器被杀。
- 后果：**整段录制全部丢失**（缓存只存在于 `recorded_points_` 内存中）。飞手往往在飞完后才发现白飞一趟。
- 建议修法：加"按需保存"入口（信号/std_srvs/键盘）；或每 N 个关键点把当前缓存原子写一次 `.partial` 文件；至少在启动时日志里明确写"保存只发生在 Ctrl-C 时"。

### R-06【中·一致性】路径与 EKF 原点同一性、以及 `z_offset_m` 的隐式耦合没有任何运行时校验

- 现象：loader 把文件里的绝对 SLAM 坐标硬转换成绝对 NED 目标（`path_loader.cpp:30-34, 84-85`），并**假定录制时的 SLAM 原点 = 当前 EKF 原点**；同时 `px4_odom_convert` 有 `z_offset_m`（`src/px4_odom_convert/src/px4_odom_convert.cpp:17, 124-125`），loader 里**没有**对应项。
- 触发：录制后重启 FAST-LIO（原点漂移/换场地）；或有人把 `z_offset_m` 改成非 0。
- 后果：整条路径被平移（z_offset 情形是"整体高度错一个常值"；原点情形是"整体平移到错误位置"），飞机朝**错误的绝对位置**飞。唯一兜底是 join 段限速与 `max_tracking_error_m`，但 4 m 的错位不会触发 5 m 的中止门。
- 建议修法：(a) 加载后打印首点/末点的 NED 坐标与当前位姿的距离并给出**显式警告阈值**（例如 >3 m 就红色警告）；(b) 约定 loader 读取 `z_offset_m`（同一 YAML 或 ROS 参数）以保持与 odom_convert 同步；(c) 长期方案：把路径存成**相对首点**的增量，跟随前平移到当前位姿（会牺牲"绝对复现"能力，需产品决策）。

### R-07【中·性能】RViz 可视化每次新增关键点都整条重发，且是 `transient_local`

- 现象：`publish_visualization(recorded_points_)` 在每个新关键点上都把**整个** `recorded_points_` 序列化成一条 `nav_msgs/Path` 重发（`src/path_record_node.cpp:449, 478`），发布者 QoS 为 `QoS(1).reliable().transient_local()`（`:97-98`，会保留最后一条样本）。
- 触发：长时间录制（点数上千）。
- 后果：CPU/带宽 O(n²) 增长，dds 里常驻一份大样本；录制节点与跟随节点共用 DDS 时可能挤占 `/px4_control/goto` 的实时性。
- 建议修法：可视化限频（如 1 Hz）、或只发增量（`nav_msgs/Path` 拼接语义上不安全，可改用 Marker/Points 增量话题）、或把 publisher 改成 volatile。

### R-08【中·一致性】waypoint 模式的"零前馈"与 `px4_control` 自己声明的契约相互矛盾

- 现象：waypoint 节点在追踪时显式发 `vx=vy=vz=0`、`ax=ay=az=0`、`yawspeed=0`（`src/path_follow_waypoint_node.cpp:184-191`）；而 `px4_control/src/px4_control_node.cpp:505-508` 的注释明确写"零值会被当作前馈、与位置环打架，造成起飞超调/跳动"，并在非追踪时改用 NaN（`:509, 530, 537`）。
- 触发：使用 `follow_mode: waypoint`（当前 YAML 就是它）。
- 后果：**未确认**。若 PX4 把显式 0 与 NaN 同等处理（把 NaN 填 0），两者等价、无影响；若 PX4 对"提供速度前馈"和"未提供"走不同分支（例如不同增益表/一致性检查），则 waypoint 模式的位置保持品质会差于 trajectory 模式。**这是一个必须实测的项**（悬停 60 s 看稳态误差与振荡，两种模式对比）。
- 建议修法：把 waypoint 的 v/a/yawspeed 改成 NaN（与 `px4_control` 的非追踪分支一致），并让 `px4_control` 对 NaN 走"无前馈"分支；实测确认后固化结论。

### R-09【中·可运维】"按 4 重新加载"的日志是错的（见 R-01 修法 c）

- 现象：`src/path_follow_trajectory_node.cpp:75-78`、`src/path_follow_waypoint_node.cpp:53-57`。
- 后果：操作者按提示操作，节点什么也没变（更糟：`points_` 为空时按 '4' 会立刻发 `active=false`，把刚打开的许可又关掉，表现为"按 4 没反应"）。
- 建议：改文案，并真正实现重载（见 R-01）。

### R-10【中·部署】路径目录硬编码 + CMake 安装规则失效

- 现象：`params.yaml:21, 114` 写死 `/home/duzhong/dzpx4/src/px4_path/src/path`；`CMakeLists.txt:73-77` 的安装条件永不成立（目录其实是 `src/path`）；`config/params.yaml` 注释也写"固定从源码目录读取，不使用 install 目录"。
- 触发：换机器/改目录名/纯 `install` 空间部署（`ros2 launch` 只 source install）。
- 后果：节点启动即报 `path load failed: cannot open path file`（失效安全，不会误飞），但功能不可用且报错信息不指向"路径目录配置错误"。
- 建议：把路径目录改为**运行参数**（launch 可覆盖）或 `share/px4_path/path`；修 CMake 为 `install(DIRECTORY src/path DESTINATION share/${PROJECT_NAME})`；在错误信息里回显最终解析出的绝对路径与"目录里现有的 .txt 文件列表"。

### R-11【低·健壮性】空容器上的 `back()`/`[]` 无防御

- 现象：`src/path_follow_trajectory_node.cpp:584` `trajectories_.back()`（HOLD 分支）、`:352` `trajectories_[active_segment_index_]`（tangent 模式）；`src/path_follow_waypoint_node.cpp:291` `points_.back()`（HOLD 分支）。
- 触发：当前代码路径**不可达**（HOLD 只能从非空 trajectories_/非空 points_ 进入，二者又都不会在运行中被清空）。但这是"靠不变式活着"，任何后续改动（新增重载路径、清空 points_ 的逻辑）都会把 UB 引进来。
- 后果：越界读 → 采样出垃圾位置 → `publish_reference(active=true)` 把飞机指向垃圾坐标。
- 建议：`if (trajectories_.empty() || points_.empty()) { publish_stop(); set_state(LOADED,"empty"); return; }` 前置守卫；`MotionTrajectory::sample()` 在 `!valid_` 时不要返回全 0 样本（见 R-13）。

### R-12【低·健壮性】参数边界缺失：`pursuit_rate` 无上下限、`>1000` 时定时器周期被截断为 0

- 现象：`std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(1.0, pursuit_rate_)))`（`:60-61`、`:39-40`）—— `pursuit_rate=2000` → `0 ms`；`pursuit_rate=NaN` → `std::max` 返回 1.0（因为与 NaN 比较全 false）→ 1 Hz，静默降频。
- 触发：YAML 手误。
- 后果：周期 0 时行为取决于 rclcpp 版本：**或** 抛 `std::invalid_argument` 导致节点启动失败，**或** 定时器满速空转吃满一个核并刷爆 `/px4_control/goto`（**未确认**，见 §12）。1 Hz 情形则是"跟随频率低于 control 的 1 s 超时"，会周期性触发 `Goto timeout`。
- 建议：加 `1.0 ≤ pursuit_rate ≤ 200.0` 与 `isfinite` 校验，非法即拒启动；并在日志里回显实际周期。

### R-13【低·失败语义】`MotionTrajectory::sample()` 在轨迹无效时返回全 0 样本（= 地面原点）

- 现象：`src/motion_trajectory.cpp:520-525`：`if (!valid_) return {};` —— `TrajectorySample` 默认构造为位置 (0,0,0)、速度 0（`include/px4_path/motion_trajectory.hpp:21-28`），而 (0,0,0) 是**合法的 EKF 原点坐标**，无法与"有效目标"区分。
- 触发：任何让 `valid_==false` 的轨迹被采样（当前不可达：`publish_reference` 只接收 build 成功的段）。
- 后果：一旦被引入，飞机收到的目标是"回到 EKF 原点/起飞点"，且 `active=true`（不会触发任何保护）。
- 建议：`sample()` 改为返回带 `bool valid` 的结构，或让调用方在发布前用 `trajectory.valid()` 断言；更稳妥的做法是在 `publish_reference` 里对位置做"距当前位姿 ≤ max_tracking_error_m·2"的合理性检查。

### R-14【低·性能】大路径会在回调里花掉秒级 CPU，可能撞上 control 的 1 s 超时

- 现象：`profile_count = max(2, 2N, len/profile_resolution_m+1)`（`:312-316`）→ `measure_limits` 采样 `max(2000, 40·profile_count)` 次，每次 4 次样条求值（`:482-518`），且被调用两遍（`:385, 408`）；全部发生在 `on_goto_enabled` 回调线程里。连接段点数 ∝ 起点距离（无上限，R-03）。
- 触发：起点距路径首点很远（连接点上千个）或路径很长（点数多、`profile_resolution_m` 很小）。
- 后果：`start_trajectory` 阻塞单线程执行器 → 期间不发布 → 若 >1 s，`px4_control` 判 `Goto timeout` 关许可悬停（与 R-04 叠加成"按了 4 却不飞/飞一半停"）。
- 建议：把 `measure_limits` 的采样数改为按时间（如固定 2000 点）+ 限制 `N`；移到非回调上下文；把耗时打日志（当前只打长度/时长，没打构建耗时）。

### R-15【低·健壮性】录制节点的阻塞式 IO 与异常

- 现象：`write_path_file`（`:295-361`）在**析构**或**里程计回调**（frame 变化路径 `:407`）中同步做全量写盘 + rename；`PathRecordProcessor::average_poses` 会 `throw std::invalid_argument`（`src/path_record_process.cpp:111-113`），而 rclcpp 回调里没有 catch，异常穿透执行器会直接 `std::terminate`。
- 触发：磁盘慢/满；或将来有人改动 `smooth_path` 的边界导致 begin≥end。
- 后果：卡顿（丢帧）或进程崩溃（丢失整段录制，与 R-05 叠加）。
- 建议：保存逻辑移到独立线程或用 `rclcpp::AsyncParametersClient` 式的"延迟任务"；`average_poses` 的越界改为返回参考值 + 错误日志而非 throw；写盘前检查剩余空间。

### R-16【低·可用性】路径文件解析过严：一个多余 token 就整条拒绝

- 现象：`src/path_loader.cpp:76-82`（行尾多一个字段 → 硬失败）。这条是**有意设计**（注释说明是防手误），但副作用是：手工在数据行尾加注释、或从别的工具导出带额外列的文件，会直接被拒。
- 建议：保留硬失败（安全优先），但在错误信息里给出**行号 + 该行内容**（已有）以及"是否包含注释符"的提示；或在录制文件头写清"数据行不得追加任何内容"。

### R-17【低·可维护】两个跟随节点约 150 行模板代码重复（§2.3），且线程安全依赖"单线程执行器"

- 现象：跟随节点没有任何互斥锁，`on_local_position`/`on_goto_enabled`/`on_timer` 直接读写共享成员（`path_follow_trajectory_node.hpp:87-103`）。`rclcpp::spin`（`src/path_follow.cpp:38`）是单线程执行器 → 当前安全。
- 触发：任何人把它换成 `MultiThreadedExecutor`（在 px4_control 里已用 `std::mutex`，见 `px4_control_node.cpp:179, 202, 254, 286, 574`，可见作者知道这个模式）。
- 后果：数据竞争（`points_` 与 `trajectories_` 的读写在回调/定时器间交错）。
- 建议：抽基类 + 明确注释"本节点要求单线程执行器"，或直接加锁。

### R-18【低·可观测】除了 topic echo，操作者无法看到"参考 vs 实际"

- 现象：跟随节点**只**发布 `/px4_control/goto`，没有状态话题、没有参考轨迹的可视化（录制节点有 `/path_record/visualization`，跟随节点没有对应物）。
- 后果：R-02/R-04 这类"参考与飞机不一致"的问题在现场极难判断。
- 建议：新增只读诊断话题（`nav_msgs/Path` 参考轨迹 + `Float32MultiArray` 当前的误差/时间缩放/段号），或至少按 1 Hz 打一条含"段号/参考时间/误差/scale"的日志。

---

## 11. 文档与代码/YAML 不一致清单（逐条可核查）

| 位置 | 文档写的 | 代码/YAML 实际 | 影响 |
|---|---|---|---|
| `README.md:65` | 固定目录 `/home/lyl/code/dzpx4/px4_path/src/path` | `params.yaml:21,114` = `/home/duzhong/dzpx4/src/px4_path/src/path` | 照 README 找文件会找不到（开发机遗留路径） |
| `px4_path_README_DELIVERY.md:296` | `terminal_tol` 默认 `0.20` | YAML `0.15`（`params.yaml:35`） | 参数理解偏差 |
| `px4_path_README_DELIVERY.md:403` | `profile_resolution_m` 默认 `0.15` | YAML `0.12`（`params.yaml:62`） | 同上 |
| `px4_path_README_DELIVERY.md:405` | `segment_switch_speed_mps` 默认 `0.20` | YAML `0.10`（`params.yaml:70`） | 同上 |
| `px4_path_README_DELIVERY.md:292` | `follow_mode` 默认 `trajectory` | YAML `waypoint`（`params.yaml:15`） | **现场实际跑的是 waypoint，而文档默认写 trajectory**，容易误判行为 |
| `px4_path_README_DELIVERY.md:165` | `overwrite_existing` 默认 `false` | YAML `false` ✅，但代码默认 `true`（`path_record_node.cpp:66`） | 只在"不带 params_file 启动"时不同 |
| `px4_path_README_DELIVERY.md:163,293` | `path_dir` 默认"工程内 `src/path` 目录" | 代码默认 `""` 且空值直接拒绝（`path_record_node.cpp:136-138`、`path_follow_trajectory_node.cpp:144-148`） | 文档暗示有默认值，实际没有 |
| `src/path_record_node.cpp:59` | 注释"空字符串表示使用 share/px4_path/path" | 同上，实际拒绝 | 误导后续维护 |
| `README.md:28-29` / `px4_path_README_DELIVERY.md:73` | "不支持飞行中切换"（与实现一致 ✅） | `follow_mode`/`record_mode` 确实只在启动读 ✅ | 无需修 |
| 各文档 | "路径目录与文件名在 YAML 中统一" ✅ | 但 YAML 自身两处就不一致（R-01） | **最需要修的一条** |

---

## 12. 未确认事项（明确列出，不要当结论用）

1. **PX4 对"显式 0 速度/加速度前馈"的处理**（R-08）：本机没有 PX4 固件源码在审计范围内，无法判定 `TrajectorySetpoint.velocity=0` 与 `=NaN` 是否等价。`px4_control_node.cpp:505-508` 的注释只代表作者当时的结论。→ 必须实测（两种模式各悬停 60 s，看稳态误差/振荡）。
2. **rclcpp 对 `0 ms` 周期定时器的行为**（R-12）：抛异常还是满速空转，取决于 rclcpp 版本；本次审计范围内没有 rclcpp 源码。
3. **`pursuit_goto_timeout` 与构建耗时的真实余量**（R-04/R-14）：没有机上实测的 `start_trajectory` 耗时数据。日志里只有长度/时长（`src/path_follow_trajectory_node.cpp:313-320`），没有构建耗时。
4. **`src/path/` 下三个 txt 的实际用途与新旧**（R-01）：文件头相同、无法仅凭文件判断哪个是"最终想飞的那条"；`recorded_path_2.txt` 是 YAML 当前指向的文件，但它是"最新"还是"上一版"未确认。机上 `install/` 里也没有这些文件（与 CMake 结论一致）。
5. **机上是否还存在 `recorded_path_3.txt` 等后续版本**：本次只读的是本地镜像，未 SSH，未确认。
6. **飞控实际参数 `MPC_XY_VEL_MAX` / `MPC_Z_VEL_MAX` / `MPC_JERK_MAX` 的取值**（R-02 的后果量级）：审计范围内没有 PX4 参数文件。
7. **`max_tracking_error_m` 中止门是否曾经触发过**：需要机上日志（本次不 SSH，未取）。
8. **`enable_filter_smoothing: true` 时 `smoothing_window` 对急弯的实际削弱程度**：属定量问题，需用现场路径数据仿真（`remove_close_points` 的"首末点恒保留"行为见 `path_record_process.cpp:175-189`，逻辑本身已核对无误）。

---

## 13. 复核方法（想自己验一遍时怎么做）

```bash
# 1. 编译产物只有两个可执行 —— 与 CMakeLists.txt:22-63 一致
ls -l install/px4_path/lib/px4_path/            # 期望：path_record_node  path_follow_node

# 2. 死代码：install(DIRECTORY path) 条件永假
ls -d src/px4_path/path 2>/dev/null || echo "包根没有 path/ 目录 —— CMakeLists.txt:73 永不成立"
ls src/px4_path/src/path/                        # 真实目录在这里

# 3. R-01：录制/跟随文件名契约
grep -n "path_file\|output_file" src/px4_path/config/params.yaml

# 4. 只在启动读一次路径
grep -n "reload_path" src/px4_path/src/*.cpp     # 期望：只在构造函数里出现一次

# 5. QoS 兼容性（两侧都看）
grep -n "create_publisher<px4_control::msg::GotoTarget>\|create_subscription<msg::GotoTarget>" -r src/px4_path src/px4_control

# 6. 坐标系一致性（loader vs odom_convert）
grep -n "slam_to_enu\|enu_to_ned\|R_enu_slam_\|enu_to_ned_local_frame" -r src/px4_path src/px4_odom_convert

# 7. 无第三方轨迹库残留
grep -rniE "ego|bspline|polynomial|traj_utils|MINCO" src/px4_path/   # 期望：只有一句 MPC_XY_VEL_MAX 注释
```

---

*审计结束。本文件是本次任务唯一产出的文件；未修改、未创建其它任何文件，未连接无人机。*
