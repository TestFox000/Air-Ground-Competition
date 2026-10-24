# 循迹核心代码结构化分析报告（path_executor 及 car_localization 包）

- 分析对象：`src/car_localization/`（ROS2 Humble / Python 3.10，阿克曼小车）
- 分析方式：**纯只读**（未修改任何被测文件，仅新增本报告）
- 工作副本：`D:\User\Desktop\dzrobot_v2`（车端对应 `~/dzrobot`）
- 行号基准：本副本实际行数 —— `path_executor.py` **2622 行**、`serial_reader.py` 271 行、
  `generate_path.py` 141 行、`send_waypoint.py` 53 行、`localization.launch.py` 20 行、`setup.py` 33 行

> 说明：本报告中的"未使用/死参数/注释与代码不符"均为逐处 grep 核实后的结论，不是推断。

---

## 1. 文件 / 类 / 方法地图

### 1.1 包内文件与角色

| 文件 | 行数 | 角色 |
|---|---|---|
| `car_localization/path_executor.py` | 2622 | 主循迹节点：路线解析 → 纯追踪转向 → 停站/打靶/桥洞保护 全套状态机 |
| `car_localization/serial_reader.py` | 271 | 定位节点：IMU 航向 + 里程差分死推算 → `/odom/localization`（**block 单位**） |
| `car_localization/generate_path.py` | 141 | 离线工具：格名 → 直线插值路径文件（每行 `x,y,z,qx,qy,qz,qw`），仅测试用 |
| `car_localization/send_waypoint.py` | 53 | 离线工具：一次性向 `/nav/waypoint` 发 `BLOCK,HEAD` |
| `launch/localization.launch.py` | 20 | 只启 `serial_reader_node`；注释明确禁止第二个 `/odom/localization` 发布者 |
| `setup.py` | 33 | 注册两个入口：`serial_reader_node`、`path_executor_node`（第 30 行，2026-10-03 补） |

启动方式（`start_all.sh`，非本包）：第 160 行 `ros2 launch car_localization localization.launch.py`（走 **install 副本**）；
第 183 行 `python3 "$WS/src/car_localization/car_localization/path_executor.py"`（走 **源码**）。
→ 改 `serial_reader.py` 需 `colcon build` 才生效；改 `path_executor.py` 直接重启即可。

### 1.2 模块级常量（按行号）

| 行 | 名称 | 值/含义 |
|---|---|---|
| 103 | `CMD_QOS` | `depth=1, BEST_EFFORT`，匹配 dzactuator 订阅端 |
| 106–109 | `COL_LETTERS / MAP_COLS / MAP_ROWS / BLOCK_SIZE` | 14 列 × 11 行，格边 0.35 m |
| 113–117 | `MAP_X_MIN/MAX`, `MAP_Y_MIN/MAX`, `MAP_SAFETY_MARGIN` | 0.175/3.675、0.175/4.725 m；余量 0.60 m |
| 123 | `RAW_YAW_TO_MAP_YAW_OFFSET` | `+π/2`：原始车体系 X+ → 地图 Y+ |
| 141–163 | `MISSION_BLOCKS` | **内置**路线 18 点（K7 K9 J9 D10 B9 E7 B7 B5 C5 B3 B1 A1 D2 E4 F4 J5 K1 N5） |
| 168–172 | `MISSION_RESET / AUTO_START / AUTO_START_DELAY` | 默认 `M10,y` / True / 8.0 s |
| 178 | `DRIFT_WARN_M` | 0.30 m 单段漂移告警阈值 |
| 191–195 | `REVERSE_SPEED_RATIO / REVERSE_STEER_SIGN` | 0.6 / 1.0（倒车速度比与舵向系数） |
| 200–204 | `SHOOT_DISTANCE_TARGET/MIN/MAX`, `SHOOT_MAX_RETRY`, `SHOOT_TIMEOUT` | 0.30 / 0.20 / 0.45 m、3 次、20 s（**YAML 覆盖为 0.35 / 30 s**） |
| 207–210 | `SHOOT_SETTLE_SPEED / STABLE_FRAMES / STABLE_OFFSET_PX / STABLE_TIMEOUT` | 0.02 m/s、8 帧、25 px、6 s |
| 223,228,241,243 | `SHOOT_SETTLE_TIME / SHOOT_SCAN_ROUNDS / SHOOT_OVERSHOOT_TOL / SHOOT_LOCK_TIMEOUT` | 0.8 s、2 轮、0.30 m、18 s |
| 249–253 | `ALIGN_STRAIGHT_BEFORE/AFTER/SPEED`, `ALIGN_KP`, `ALIGN_STEER_LIMIT` | 2.0 格 / 1.0 格 / 0.10 m/s / 12.0 / 8.0° |
| 256,259,263 | `MISSION_SCAN_MODE / BRIDGE_BLOCKS / OBSTACLE_BLOCKS` | `continuous` / `['K7','K1']` / `[]` |
| 271,275 | `LINEAR_SPEED / SHOOT_COUNT_TARGET` | 0.15 m/s / 3 发 |
| 287–289 | `CONFIG_FILE` | `$ROBOT_CONFIG` 或 `~/dzrobot/config/robot.yaml` |
| 422–472 | `_CFG` 覆盖块 | 见 §5.3；读取失败**静默回退内置值** |

### 1.3 函数 / 方法目录（按行号）

| 行 | 名称 | 职责一句话 |
|---|---|---|
| 292 | `_load_config_center()` | 读 robot.yaml 的 `path_executor` 段，兼容 `/path_executor` 与 `ros__parameters` 两种写法 |
| 329 | `normalize_mission_token()` | token 大小写/空格规范化，保留 `R:` 与 `*`，`,` 后缀转小写 |
| 342 | `validate_mission_blocks()` | **只报错不改写**的路线校验：返回 (合法?, 问题清单, 统计) |
| 479 | `euler_from_quaternion()` | 四元数 → yaw |
| 490 | `normalize_angle()` | 角度折到 [-π, π] |
| 501 | `block_to_meters()` | 格名 → 米制中心：`x=(10.5-row)*0.35, y=(13.5-col)*0.35` |
| 529 | `heading_to_quaternion()` | `'x'→0, 'y'→π/2` |
| 554 | `class _GatedCmdPub` | 速度指令闸门：手动接管期间吞掉自动指令（15 处 publish 的统一入口） |
| 581 | `PathExecutorNode.__init__` | 全部状态变量 + 订阅/发布 + 50 Hz 定时器 + 3 s 健康检查 + 自动出发定时器 |
| 880 | `check_odom_publishers()` | 检查 `/odom/localization` 发布者数量（>1 报错、=0 警告） |
| 908 | `load_path()` | 读路径文件；文件路径 **无射击停站、无扫描停站**（935–937） |
| 954 | `waypoint_callback()` | 命令总入口：`load:` / `reset:` / 单点 `M10,y` / 多点多派发 |
| 1098 | `follow_route_command()` | 路线解析核心：`SHOOT@`、`R:`、`*`、`:SHOOT_STOP`、去重、终点接近段、四元数回填 |
| 1302 | `shoot_count_cb()` | 缓存累计弹数 |
| 1312 | `_update_chassis_speed()` | 里程差分 + 一阶低通（0.7/0.3）估底盘速度 |
| 1325 | `offset_center_cb()` | 缓存视觉偏移 dx/dy/cls/box_w（开火判据唯一数据源） |
| 1339 | `aim_status_cb()` | 缓存云台状态 0=IDLE 1=SCAN 2=TRACK |
| 1343 | `manual_override_cb()` | 手动接管开关，仅变化时打日志 |
| 1356 | `enter_scan_stop()` | 停车 + 发 `scan_request=1` + 标记一次性 |
| 1372 | `_banner()` | 带框日志 |
| 1377 | `_print_start_banner()` | 任务开始横幅（航点数/扫描模式/巡航速度/桥洞保护） |
| 1387 | `_print_finish_banner()` | 结束横幅（用时、累计弹数、漂移汇总）+ 完成提示音 |
| 1427 | `_publish_motion()` | `/nav/motion_state`：0 前进 / 1 倒车，仅变化时发 |
| 1437 | `_start_mission_scan()` | 记任务起始时刻、打开始横幅、按模式设置扫描 |
| 1460 | `_auto_start_mission()` | 延时到点：先 `reset:` 再 `follow_route_command(路线)`，一次性 |
| 1516 | `enter_shoot_stop()` | 停站 + 位姿锚回格心 + 三段式状态机复位 + **收回**开火许可 |
| 1591 | `odom_callback()` | block→m 换算、raw yaw + π/2 → 地图系、叠加 snap 偏移 |
| 1617 | `find_nearest_segment()` | **只向前**搜索最近航段 + 投影点（返回 idx/t/px/py/dist） |
| 1657 | `find_lookahead_point()` | 前视点：跨段累加；20° 转角 + 最后 4 cm 时钳到顶点 |
| 1736 | `pure_pursuit_control()` | 舵角公式 + 速度剖面 + 倒车分支（**唯一转向输出**） |
| 1812 | `_load_shoot_planner()` | 懒加载 `tools/shoot_planner.py`，失败降级为靶标自身 |
| 1841 | `compute_shoot_chain()` | 靶标 → 候选射击点链（评分排序，最多 `SHOOT_MAX_RETRY` 个） |
| 1882 | `_passed_waypoint()` | "开过头"判据：车→航点向量与当前段方向点积 < 0 |
| 1907 | `_is_reverse_segment()` | 段是否倒车（属性记在端点 `reverse_flags[idx+1]`），防御式返回 False |
| 1926 | `_rebuild_align_segments()` | 预计算需强制直行的**航段索引集合** |
| 1956 | `_align_straight_update()` | 判定当前段是否在保护集合内，返回锁定航向 = 本段走向 |
| 2005 | `control_loop()` | 50 Hz 优先级状态机（**588 行，全文件最长**） |
| 2599 | `main()` | 命令行可选路径文件 → init → spin → Ctrl-C 发零 Twist |

---

## 2. 纯追踪（Pure Pursuit）实现细节

### 2.1 关键几何参数（注意：**大部分不能用 robot.yaml 改**）

| 项 | 值 | 行号 | 可否配置 |
|---|---|---|---|
| 前视距离 `lookahead_dist` | **0.15 m** | 621 | ❌ 硬编码 |
| 轴距 `wheelbase` | 0.157 m（编码器标定值，PDF 名义 0.150） | 629 | ❌ 硬编码 |
| 到达判定 `arrival_threshold` | `0.10 + π·0.073/4 ≈ 0.157 m` | 637 | ❌ 硬编码 |
| 终点接近段 `approach_dist` | 0.6 m | 638 | ❌ 硬编码 |
| 舵角硬限幅 | ±30°（dzactuator 再限到 ±35°） | 1783 | ❌ 硬编码 |
| 射击停站半径 `SHOOT_STOP_RADIUS` | 0.17 m | 669 | ❌ 硬编码 |
| 扫描停站半径 `SCAN_STOP_RADIUS` | 0.20 m | 699 | ❌ 硬编码 |
| `max_angular_speed` | 1.5 | 625 | ❌ **死变量，全文件无引用** |

> ⚠️ **注释与代码不一致（实测）**：模块 docstring 第 52 行、`find_lookahead_point` docstring 第 1658 行、
> 第 1680 行行内注释都写 "0.2 m"，实际代码是 `0.15`（第 621 行）。重写时以 0.15 为准。

### 2.2 前视点计算 `find_lookahead_point`（1657–1732）

1. 起点：车在最近航段上的投影 `seg['px'], seg['py']`（由 `find_nearest_segment` 给出）。
2. 预算 `remaining = lookahead_dist`；先扣掉本段剩余长度 `left = (1-t)·seg_len`（1686）。
3. **转角保护**：若下一段与当前段夹角 > `CORNER_ANGLE_RAD = 20°`（1677）且距顶点 < 0.04 m（1702），
   直接把前视点钳到顶点 `p2`，防最后一刻切角。
4. 若本段能容纳整个预算 → 沿本段单位向量前进 `remaining`（1708–1714）。
5. 否则逐段扣减、跨段继续（1720–1730）；路径总长不足前视距离时直接瞄准终点（1732）。
6. 返回的合成点用内嵌 `_pt()` 造（1670–1673），四元数全填默认值 —— **纯追踪不使用四元数**。

### 2.3 曲率 / 转角公式 `pure_pursuit_control`（1736–1809）

- 车体系分解（1759–1762）：`rel_x = dx·cos(-yaw) - dy·sin(-yaw)`，`rel_y = dx·sin(-yaw) + dy·cos(-yaw)`
  （前向 / 左向分量）。
- 目标方位角 `alpha = atan2(rel_y, rel_x)`（1778）。
- 舵角（**度**，不是角速度）：
  `steer_deg = degrees(atan(wheelbase · sin(alpha) / max(dist, 0.10)))`（1779–1781）。
  - `dist < 0.01 m` 时直接返回 `(0,0)`（1767）。
  - 分母下限 0.10 防"目标贴脸时增益爆炸"。
  - 硬限幅 ±30°（1783）。
- 🔍 **观察**：教科书写法为 `δ = atan(2·L·sinα / l_d)`。本实现的 `dist` 就是前视弦长，
  但**没有那个因子 2**，等效转向增益约为教科书的一半。若重写后出现"过弯偏软"，这是第一嫌疑点；
  反之若换成教科书式，需重新整定（可能会诱发洞口摆头，与桥洞保护的设计前提冲突）。
- 倒车分支（1799–1804）：**舵角公式完全相同**，只把速度取负（`linear_vel < 0` → dzactuator 置倒车方向位），
  并乘方向系数 `REVERSE_STEER_SIGN`（默认 1.0，无实车依据，注释明说"偏了改 -1.0"）。

### 2.4 速度剖面（三层串联，取最小值）

| 层 | 公式 | 行号 |
|---|---|---|
| ① 转向降速（基座） | `v = cruise·(1 - 0.5·(|steer|/30))`，下限 0.1 m/s | 1788, 1806–1807 |
| ①' 倒车 | `v = -max(0.08, cruise·0.6·(1-0.5·turn_factor))` | 1802–1803 |
| ② 终点减速 | `dist_to_final < 0.5` → `v = min(v, max(0.1, 0.35·d))` | 2532–2533 |
| ③ 射击点减速 | 距任一未完成射击点 `< 0.60 m` → `v = min(v, max(0.08, 0.30·d))` | 2536–2544 |
| ④ 转角减速 | 尖角（>45°）且距顶点 `< 0.45 m` → `v = min(v, 0.12)` | 2550–2568 |
| ⑤ 桥洞直行 | 固定 `ALIGN_STRAIGHT_SPEED = 0.10`（倒车段取负） | 2484–2486 |
| ⑥ PIVOT 蠕动 | 固定 0.08 m/s | 2455 |

### 2.5 阿克曼约束与限幅

- **有**：轴距模型（自行车模型）转向、舵角 ±30° 硬限幅（1783）、桥洞内 ±8° 小舵角限幅（2480–2481）。
- **无**：转向**速率**限制（舵角可一帧内跳 60°）、横向加速度/曲率上限、速度爬升率限制。
- 输出语义（2570–2574）：`Twist.linear.x` = m/s；`Twist.angular.z` = **舵角（度）**，
  dzactuator 侧 `servo = angular.z + 60` 并 clamp 到 [25, 95]。
- 转向**无积分项** —— 纯比例（`atan` 天然限幅），稳态横向误差靠"每站位姿锚回格心"消化。

### 2.6 控制频率与实时性

- 主控制定时器 **0.02 s = 50 Hz**（838）→ `control_loop` 每拍最多发 1 条 `Twist`。
- 健康检查定时器 `create_timer(3.0, check_odom_publishers)`（843）：注释写 "once 3 s after startup"，
  但 rclpy 的 `create_timer` 是**周期定时器 → 实际每 3 s 重跑一次**（无害，但注释错）。
- 自动出发定时器（850）在回调内 `cancel()`（1472），确为一次性。
- 上游速率：dzactuator 主循环 `rclcpp::Rate rate(100)`（`dzactuator.cpp:1270`）→ `/odom`、`/imu_data` 名义 ≈100 Hz；
  `serial_reader` 每帧 `/odom` 触发一次 → `/odom/localization` ≈100 Hz。50 Hz 控制环有约 2 倍余量。
- 单位/坐标一致性（已核对）：`serial_reader` 起点 `1.5, 0.5` block = `0.525, 0.175 m` = N10 中心；
  `path_executor` 第 1602–1603 行乘 0.35 还原 —— **两端约定一致**。

---

## 3. 航点 token 语义（`mission_blocks`）

解析入口 `follow_route_command`（1098），校验入口 `validate_mission_blocks`（342）。

| 写法 | 含义 | 行号 |
|---|---|---|
| `N10` | 普通航点，直线连到下一个点（允许斜线） | 1192, 1204 |
| `G10*` | **射击停站**：到达后停车、找靶、稳定后放行开火 | 1182–1183, 1206 |
| `R:B5` | **倒车到位**：`R:` 前缀，该段线速度为负（车尾朝目标摆） | 1167–1169, 1208 |
| `R:B5*` | 倒车 + 射击（**唯一合法顺序：R: 在前、* 在后**） | 注释 1163–1166 |
| `F2:SHOOT_STOP` / `F2:SHOOT` | 等价于 `F2*`（内部归一成 `*`），可读性更好 | 1174–1178 |
| `F2:SCAN_STOP` / `F2:SCAN` | 只剥后缀，**不射击**（纯扫描停站） | 1179–1180 |
| `SHOOT@<靶标>` | **指令而非坐标**：现场调 `shoot_planner` 算出候选链并**整链插入**，共用一个组号 | 1136–1161 |
| `...,x` / `...,y` | 终点朝向后缀，**只有最后一个 token 允许带** | 1108–1117；校验 375–380 |

**明确回答："ALIGN:"/"PIVOT:" 之类前缀 —— 无。**
- 桥洞强制直行（ALIGN_STRAIGHT）不是 token，而是由 `bridge_blocks` + 路线几何**自动推导**（1926–1954）。
- PIVOT（原地转向）不是 token，而是由相邻航段夹角 > 45° **自动触发**（2372, 2400–2411）。
- 解析器里也没有任何 `ALIGN:`/`PIVOT:` 关键字分支（已逐行确认）。

其它解析规则：
- **去重**：相邻同坐标且 `R:` 标记相同 → 合并（重复写 `*` 只把前一个点标成射击点，1200–1203）；
  若 `R:` 标记不同（`B5 R:B5`）**不合并**，否则倒车段会丢失（注释 1195–1196）。
- **终点接近段**：仅当"强制终点朝向"且末段与目标轴夹角 > ~2.6°（`|cos| < 0.999`）时，
  在末点前插一个退 0.6 m 的接近点（1234–1253），该点 `shoot/scan/reverse` 全 False。
- 每个点的四元数 = "到达该点时应有的朝向"（中间点 = 离开段方向，末点 = 终朝向，1259–1266）；
  行驶中不用，仅到达锚定与日志读它。
- 单点命令 `M10,y`（1022–1088）也会自动插接近段（1043–1060），**语法里没有 R: 的位置 → 一律正向**（1069）。
- `reset:BLOCK,HEAD`（972–1006）：belief = 目标格心 + 目标朝向；清空路线与所有一次性状态。
- `load:<文件>`（961–964）：文件路径点密，**不触发任何停站扫描**（935–937）。

当前工作副本 `config/robot.yaml` 的路线（第 329–358 行）实测：**29 个 token**
（`N10 N9 N8 K8 I8 I10 G10* E10 E9 D9 D7 E7 R:D7 R:B7 B6 B5 B3 A3 A4 C4 G4 G6 R:G2 G1 H1 K1 K2 K5 N5`），
其中射击点 1 个（`G10*`）、倒车点 3 个（`R:D7 / R:B7 / R:G2`）、普通 25 个。
⚠️ 注意：**内置 `MISSION_BLOCKS`（141–163）是另一张地图的旧路线**，只有在配置缺失时才会生效。

---

## 4. 状态机与流程

### 4.1 启动 → 终点

| # | 阶段 | 触发 | 关键行 |
|---|---|---|---|
| 0 | 等定位 | `has_odom == False` 时整拍直接 return（不发任何指令） | 2010–2011 |
| 1 | 自动出发 | `AUTO_START=True` 且延时到 → `reset:N10,y` → `follow_route_command(路线)` | 849–851, 1460–1511 |
| 2 | 打开始横幅 | `_start_mission_scan()`，按模式设置扫描 | 1437–1458 |
| 3 | 循迹主循环 | 见 4.2 优先级树 | 2005–2592 |
| 4 | 射击停站 | 距未完成射击点 ≤0.17 m，或"已越过且 ≤0.30 m" | 2265–2283 |
| 5 | 扫描停站 | `mission_scan_mode='waypoint'` 时距待扫描点 ≤0.20 m（已 TRACK 则不停） | 2245–2260 |
| 6 | 尖角 PIVOT | 夹角 >45° 且距顶点 <0.25 m → 停/等 0.5 s/0.08 m/s 蠕动至误差 <15° | 2372–2468 |
| 7 | 桥洞直行 | 当前航段 ∈ 预计算保护集合 → 关纯追踪，锁航向 ±8°、0.10 m/s | 2475–2504 |
| 8 | 正常纯追踪 | 上述都不成立 | 2506–2592 |
| 9 | 终点到达 | 距末点 <0.157 m → **`arrived` 永久锁存**、锚位姿、发 `stop_signal=1` + `scan_request=1`、打结束横幅 | 2294–2363 |

### 4.2 `control_loop` 优先级树（每拍 50 Hz，自上而下短路）

```
1 无定位                → return
2 shoot_wait            → 零 Twist 保活 + 四阶段狙击状态机（2034–2193）
3 scan_wait             → 零 Twist 停车等扫描（2200–2217）
4 无路径                → 零 Twist 保活（维持 dzactuator Pursuit 模式）（2222–2226）
5 求最近段 + 前视点      → 更新 current_target_idx（2230–2239）
6 扫描停站检测（遍历全部点）
7 射击停站检测（遍历全部点，含"开过头"补触发）→ 命中即 return
8 终点到达判定（锁存）
9 PIVOT 进入判定 + PIVOT 状态机
10 ALIGN_STRAIGHT 接管（含越界急停 → return）
11 正常纯追踪 + 越界急停 + 三层减速 + 发布
```

### 4.3 打靶四阶段状态机（2034–2193，"宁可漏打，不可卡死"）

| 阶段 | 名称 | 进入条件 | 放行/退出条件 | 超时兜底 |
|---|---|---|---|---|
| 0 | SETTLE | `enter_shoot_stop` | 速度 <0.02 m/s 且位置连续 0.8 s 不变 | `cand_t > 0.4×SHOOT_TIMEOUT` → 降级进阶段 1（2067–2074） |
| 1 | ACQUIRE | 停稳 | 视觉 `det_cls≠0` 且 `det_age<1 s` → 阶段 2；不主动发扫描请求（云台自主持续扫描） | 完成 2 轮扫描仍无靶 → 放行下一候选；`SHOOT_LOCK_TIMEOUT=18 s`（2089–2105） |
| 2 | STABLE | 看到靶 | `|dx|,|dy| ≤ 40 px`（YAML）且静止，连续 5 帧 → **发 `stop_signal=1`** 进阶段 3 | `SHOOT_STABLE_TIMEOUT=6 s` → 降级放行开火（2135–2148） |
| 3 | FIRE | 已放行 | 累计弹数 − 到站快照 ≥ `shoot_count_target`(3) → 成功，**取消同组剩余候选点** | `cand_t > SHOOT_TIMEOUT`(30 s) → 换下一候选点（2174–2177） |

- 退出时统一发 `stop_signal=0`（2181–2183），路线继续，绝不阻塞。
- `SHOOT@` 插入的候选链共享 group id，打中一个即跳过同组其余点（2157–2169）。
- 死代码：`self.SHOOT_WAIT_TIMEOUT = 150.0`（668）**全文件无引用** → 模块 docstring 第 49 行
  "150 s timeout" 是过期描述，实际单点上限 = `shoot_timeout`。
- 死状态：`shoot_lock_seen`（663/1560/2079）只写不读；`shoot_chain`（685）只初始化不使用。

### 4.4 全局超时兜底汇总

| 变量 | 值（代码 / YAML） | 作用 |
|---|---|---|
| `SCAN_WAIT_TIMEOUT` | 8.0 s | 点位扫描停站硬超时（698, 2213） |
| `SHOOT_TIMEOUT` | 20.0 / **30.0** | 单候选射击点总硬超时（204, 286） |
| `SHOOT_SETTLE_TIME` | 0.8 s | 停稳判定 |
| `SHOOT_STABLE_TIMEOUT` | 6.0 s | 等瞄稳超时 → 降级开火 |
| `SHOOT_LOCK_TIMEOUT` | 18.0 s | 找靶超时 |
| `SCAN_ROUNDS` | 2 轮 | 扫完两轮无靶即放弃 |
| PIVOT | 无超时（2443 仅靠 `|yaw_err|<15°`） | ⚠️ 见 §7 风险 R2 |

---

## 5. ROS 接口清单

### 5.1 订阅

| 话题 | 类型 | 回调 | 队列 | 速率 | 行号 |
|---|---|---|---|---|---|
| `/odom/localization` | `nav_msgs/Odometry` | `odom_callback` | 10 | ≈100 Hz | 747–752 |
| `/nav/waypoint` | `std_msgs/String` | `waypoint_callback` | 10 | 事件 | 757–762 |
| `/shoot_count` | `std_msgs/Int32` | `shoot_count_cb` | 10 | 事件 | 793–794 |
| `/nav/manual_override` | `std_msgs/UInt8` | `manual_override_cb` | 10 | 事件 | 804–805 |
| `/aim/status` | `std_msgs/Int32` | `aim_status_cb` | 10 | ≈100 Hz（云台上报） | 807–808 |
| `/offset_center` | `std_msgs/Int32MultiArray` | `offset_center_cb` | 10 | ≈30 Hz（视觉帧） | 824–825 |

### 5.2 发布

| 话题 | 类型 | QoS | 语义 | 行号 |
|---|---|---|---|---|
| `pursuitAngle` | `geometry_msgs/Twist` | **depth 1 / BEST_EFFORT** | `linear.x`=m/s，`angular.z`=舵角(°)，50 Hz | 776–782 |
| `/move_base/stop_signal` | `std_msgs/UInt8` | depth 10 | 1=允许云台扫描+射击，0=收回 | 786–790 |
| `/nav/scan_request` | `std_msgs/UInt8` | depth 10 | 0=停 / 1=单次全向 / 2=持续扫描 | 811 |
| `/nav/motion_state` | `std_msgs/UInt8` | depth 10 | 0 前进 / 1 倒车，仅变化时发 | 815, 1427–1435 |
| `/voice_switch` | `std_msgs/UInt8` | depth 10 | 完成提示音（默认 ID=0 不播） | 829, 1419–1425 |

- `pursuitAngle` 外面套 `_GatedCmdPub`（782）：手动接管期间**全部**自动指令被吞（574–577）。
- 全文件 `cmd_pub.publish` 实测 **15 处**调用点（与第 564 行注释一致，已逐处核对）。

### 5.3 读取的 robot.yaml 键（`path_executor.ros__parameters`，422–472）

| 键 | 代码默认 → 工作副本 YAML | 类型 |
|---|---|---|
| `mission_blocks` | 内置 18 点 → 29 点 | list[str] |
| `mission_reset` | `M10,y` → `N10,y` | str |
| `auto_start` / `auto_start_delay` | True / 8.0 → true / 10.0 | bool / float |
| `linear_speed` | 0.15 → 0.15 | float |
| `shoot_count_target` | 3 → 3 | int |
| `mission_scan_mode` | continuous → continuous | str |
| `map_safety_margin` | 0.60 → 0.60 | float |
| `drift_warn_m` | 0.30 → 0.30 | float |
| `finish_voice_id` | 0 → 0 | int |
| 浮点批量键（453–462） | `shoot_distance_target/min/max`、`shoot_settle_speed`、`shoot_stable_offset_px`、`shoot_stable_timeout`、`shoot_overshoot_tol`、`shoot_slowdown_m`、`shoot_score_w_gimbal/detour`、`shoot_timeout`、`shoot_settle_time`、`shoot_lock_timeout`、`reverse_speed_ratio`、`reverse_steer_sign`、`align_straight_before/after/speed`、`align_yaw_tol_deg`、`align_kp`、`align_steer_limit` | float |
| 整数批量键（470–472） | `shoot_max_retry`、`shoot_stable_frames`、`shoot_candidates_keep` | int |
| 列表键（466–469） | `bridge_blocks`（YAML = **整列 K1…K11**）；`obstacle_blocks`（YAML = 8 个障碍） | list[str] |

**不可通过 YAML 调整**（硬编码）与**读了但无引用**（死键）：
- 硬编码：`lookahead_dist`、`wheelbase`、`arrival_threshold`、`approach_dist`、`SHOOT_STOP_RADIUS`、
  `SCAN_STOP_RADIUS`、舵角 ±30°、PIVOT 全部阈值（45°/0.25 m/15°/0.45 m）、`CORNER_ANGLE_RAD=20°`。
- 死键：`align_yaw_tol_deg` → `ALIGN_YAW_TOL_DEG`、`shoot_candidates_keep` → `SHOOT_CANDIDATES_KEEP`（全文无引用）。
- 类型隐患：批量分支用 `globals()[k.upper()] = float(...)`，**未在键表登记的新键不会被读取**；
  且 YAML 里若把某个 int 段参数写成浮点会静默改变类型。

### 5.4 辅助入口

- `send_waypoint.py` → `/nav/waypoint`（一次性发 `BLOCK,HEAD`）。
- `start_path_executor.sh`（42 行）：source install → 生成测试路径 → 后台直跑源码 → `ros2 topic pub --once`。
- `setup.py:30` 新增 `path_executor_node` 入口，但注释明说**不要**把 `start_all.sh` 改成 `ros2 run`。

---

## 6. 桥洞保护（K 列）与越界急停

### 6.1 桥洞保护 = 配置驱动，代码里**没有 K 列特判**

- 保护点来源：`BRIDGE_BLOCKS`（代码默认 `['K7','K1']`，259）← YAML `bridge_blocks` **整列 K1…K11**（robot.yaml:252）。
  → "K 列"这一事实只存在于配置注释里；换地图只改 YAML，代码无需动。
- 段级预计算 `_rebuild_align_segments`（1926–1954）：
  容差 `tol = 0.35 × 0.4 = 0.14 m`（1938）；凡是坐标落在任一桥洞点 0.14 m 内的**航点**索引记为 `hit`；
  对每个 hit 展开为航段 `[h-2, h+1]`（`align_straight_before=2`、`after=1`），钳到 `[0, 末段]` → `self._align_segs`。
- 每拍判定 `_align_straight_update`（1956–1994）：取 `find_nearest_segment()` 的 `seg['idx']`，
  命中集合则激活；**锁定航向 = 该航段自身走向**（1984），进出各打一条 `★` 日志。
- 控制接管（2475–2504）：关掉纯追踪，`steer = clamp(yaw_err_deg × ALIGN_KP/10, ±8°)`（2480–2481，
  即 1.2 °舵角/°误差），速度 0.10 m/s（倒车段取负），并**复用越界急停检查**。
- 防误伤设计（注释 1929–1933）：早期版本按"离最近保护点 <2 格"的几何判据，导致 K 列附近 ±2 格
  任何位置都被锁航向+限速（表现为"循迹僵直"）；改成**航段索引**后只有真正穿洞的段受保护。
- 与 PIVOT 的互斥：`align_on` 为真时禁止进入 PIVOT（2384 条件里显式排除），桥洞内绝不做原地转向。
- 路线级约束 `bridge_order`（robot.yaml:258–260）**只存在于 YAML 注释**，代码不校验；
  实际方向由 `mission_blocks` 顺序决定（route_check.py 负责校验，非本节点）。

### 6.2 越界急停（MAP_SAFETY_MARGIN）

- 边界：`x ∈ [0.175−0.60, 3.675+0.60] = [−0.425, 4.275]`，`y ∈ [−0.425, 5.325]`（113–117、YAML `map_safety_margin`）。
- **两处**实施（判据完全相同）：
  - 桥洞直行分支内（2489–2492）→ 发零 Twist 后 return；
  - 正常纯追踪分支内（2512–2528）→ 发零 Twist、**只报一次** error + 横幅，然后 return。
- 设计理由（注释 111–112）：纯轮式死推算会漂移，实测"跑出地图 12 m 仍未停"，必须有硬边界。
- ⚠️ **覆盖缺口（低危但真实）**：越界检查位于正常分支的**发布前**，因此以下路径**不做**越界判定：
  PIVOT 阶段 0/1/2（2422–2468，阶段 2 以 0.08 m/s 蠕动且仅靠 15° 航向误差退出，
  **无超时**）、射击停站/扫描停站分支（发零 Twist，安全）、终点锁存分支（已停车，安全）。
  即：若在尖角蠕动期间里程计已越界，车会继续蠕动直到航向对齐才停。重写时应把边界检查提到状态机最顶层。

---

## 7. 显式 TODO / FIXME / 已标注的"已知坑"摘要

代码中**没有** `TODO` / `FIXME` / `XXX` / `HACK` 标记；所有坑都以 `★`/`⚠️` 中文注释形式记录。摘要如下：

| # | 内容 | 行号 | 状态 |
|---|---|---|---|
| K1 | 倒车舵向 `REVERSE_STEER_SIGN` 仅理论推导，**无实车依据**，偏了改 −1.0（不用编译） | 192–195, 1797–1798 | 待实测 |
| K2 | 完成提示音各 ID 含义**无实机依据**，需先跑 `tools/test_voice.sh` 试出；底盘有"同 ID 至少隔 2 s"闸门 | 180–186, 1418–1425 | 默认关闭 |
| K3 | 云台实际扫描范围是 **−160°…+10°**，与早期 readme 的 "±90°" 不符；注释仍沿用 ±90° 表述 | 230–243, 2272 | 注释过期 |
| K4 | 旧打靶逻辑"到点即放行开火"浪费弹药（全场仅 10 发）→ 已改四阶段 | 215–222, 2051–2148 | 已修 |
| K5 | 停站时**不再**发 `scan_request=1`：会把云台的持续扫描降级成单次 —— "循迹限制了云台找目标"的根源 | 2059–2063 | 已修 |
| K6 | "开过头"漏打：原来只认 0.17 m 半径，冲过就永不触发 → 增加 `SHOOT_OVERSHOOT_TOL=0.30 m` 补触发 | 2272–2281 | 已修 |
| K7 | 桥洞保护曾按几何距离判定 → K 列附近全域被锁航向（"循迹僵直"）→ 改航段索引 | 1929–1933 | 已修 |
| K8 | 越界急停：实测漂移出图 12 m 仍未停 | 111–112 | 已加 |
| K9 | 定位死推算：dzactuator 原始位姿坐标系与物理世界旋转不符（2026-09-06 实测，两跑漂向相反且螺旋外扩）→ 只积分"旋转不变的步长"，靠 IMU 航向积分 | `serial_reader.py` 13–26 | 已改 |
| K10 | `serial_reader` 的 launch 走 **install 副本**，过期 colcon install 会静默跑旧代码 → 用 `NODE_VERSION` 日志标签暴露 | `serial_reader.py` 55–59 | 已加 |
| K11 | `ticksPer2PI=3236` 与轮距反推 ≈2274 **差 42%**，二者必有一错 → 会导致"命令转 90° 实际约 38°"，过桥洞撞壁 | robot.yaml 386–392 | ⚠️ 待实测 |
| K12 | 扫描"没找到"的判定延迟必须 ≥ 一个完整扫描周期（≈10 s），否则刚起步就误判放弃 —— "这个坑踩过" | robot.yaml 306–310 | 已记录（对应键已被 `shoot_lock_timeout` 取代） |
| K13 | 障碍表真源唯一性：`obstacle_blocks` 曾与 `map.yaml` 不一致 → 射击点判据全错 + 误判路线踩障 | robot.yaml 292–299 | 已修（route_check 校验） |

**本次分析新发现的注释/代码不一致**（原文件未标记）：
1. `lookahead_dist` 注释写 0.2 m，实为 **0.15**（52 / 1658 / 1680 vs 621）。
2. 健康检查定时器注释写 "once"，实为**每 3 s 重复**（843 vs rclpy 语义）。
3. `SHOOT_WAIT_TIMEOUT = 150 s` 从未使用；模块 docstring 第 49 行的 "150s timeout" 已过期（668）。
4. `self.max_angular_speed`（625）、`shoot_lock_seen`（663）、`shoot_chain`（685）、
   `ALIGN_YAW_TOL_DEG`、`SHOOT_CANDIDATES_KEEP`：**写了不用**。
5. `_update_chassis_speed()` 在 `shoot_wait` 分支内被**同一拍调用两次**（2015、2037），第二次 `dt≈0` 被 `dt>1e-3` 挡掉（无害冗余）。
6. 教科书纯追踪公式的因子 2 缺失（详见 §2.3）。

---

## 8. 复杂度热点与拆分建议

### 8.1 热点排序（按"最长 × 最难改"）

| 排名 | 函数 | 行号 | 行数 | 为什么难改 |
|---|---|---|---|---|
| 1 | `control_loop` | 2005–2592 | **588** | 11 个分支挤在一个函数体：4 阶段狙击状态机（160 行）、扫描等待、PIVOT 三段式（95 行）、桥洞接管、三层减速、越界急停、日志节流。任一处改动都可能影响优先级顺序 |
| 2 | `__init__` | 581–874 | **294** | 约 90 个状态字段平铺，无分组、无 dataclass；`_GatedCmdPub` 要求 `manual_override` **先**初始化（772–774）——隐式顺序依赖 |
| 3 | `follow_route_command` | 1098–1295 | **198** | 5 种 token 语法 + 去重 + 接近段插入 + 四元数回填 全在一个函数，且有 5 个平行数组（points/shoot/scan/reverse/groups）+ 每点四元数手工同步 |
| 4 | `waypoint_callback` | 954–1091 | **138** | 命令分发 + 单点路径构建 + reset 的全部状态清理（15 个字段）混在一起 |
| 5 | `pure_pursuit_control` | 1736–1809 | 74 | 前向/倒车混写，速度与转向耦合 |
| 6 | `find_lookahead_point` | 1657–1732 | 76 | 跨段预算 + 转角钳位 + 退化分支 |
| 7 | `enter_shoot_stop` | 1516–1589 | 74 | 位姿锚定 + 状态复位 + 信号发布三合一 |

**性能热点（当前规模无感，重写放大后要注意）**：`control_loop` 每拍（50 Hz）遍历**全部航点三次**
（扫描检测 2245、射击检测 2265、射击减速 2536），加 `find_nearest_segment` 的 O(N) 前向扫描（1635）
和 `_align_straight_update` 内再调一次 `find_nearest_segment`（1965）→ **每拍 2 次最近段搜索 + 3 次全表遍历**。
29 个航点无所谓；若换成"文件路径"模式（`generate_path.py` 按 0.05 m 插值 → 16 m 路径约 320 点，量级 ×10）
或更密，需要改为**索引窗口 + 缓存**。

### 8.2 一致性隐患（重写时必须建模的正确性约束）

1. **`current_target_idx` 只前进不后退**（1635）：一旦误判段索引，永远无法回锁旧段；
   倒车段（`R:`）存在时，"车实际在往后走而索引往前走"这一情形没有专门处理。
2. **5 个平行数组手工同步**：`path_points / shoot_flags / scan_flags / reverse_flags / shoot_groups`
   + 每点四元数；插入接近段时要在 5 处 `insert`（1246–1253）——漏一个就静默错位。
   建议改为单点对象 `Waypoint(x, y, yaw, shoot_group, reverse, scan)`。
3. **停站检测是"距离全表扫描"而非"索引邻近"**：若路线自交或折返经过某个未来射击点附近，
   会**提前误触发**停站（2265–2283 无"索引窗口"限制）。
4. **越界检查不在最顶层**（见 §6.2），PIVOT 蠕动是唯一无边界保护的驱动分支。
5. **两处几何判据重复实现**：转角判断在 `find_lookahead_point`（1690–1700）与 `control_loop`（2385–2399, 2550–2568）
   各写一遍，阈值却不同（20° / 45° / 45°）。建议抽 `corner_geometry(seg_idx) -> (angle, dist, turn_dir)`。
6. **两处位姿锚定重复**：`enter_shoot_stop`（1529–1541）与终点到达（2309–2326）逻辑近乎逐字重复。
   建议抽 `snap_belief_to(x, y, yaw)`。

### 8.3 拆分建议（面向重写）

```
path_executor/
  config.py      # 常量 + _load_config_center + validate/normalize（现 1–472 行）
  geometry.py    # euler/normalize/block_to_meters/heading_to_quaternion + 转角几何（现 479–546）
  route.py       # token 解析 → List[Waypoint]（现 1098–1295 抽成纯函数，无 ROS 依赖，可单测）
  pursuit.py     # find_nearest_segment / find_lookahead_point / pure_pursuit_control（现 1617–1809）
  safety.py      # in_bounds() / MAP_SAFETY_MARGIN（把 §6.2 两处重复合成一处，提到最顶层调用）
  bridge.py      # _rebuild_align_segments / _align_straight_update（现 1926–1994）
  shoot.py       # 四阶段狙击状态机（现 2034–2193 抽成独立类，输入视觉/云台状态，输出 (allow_fire, resume)）
  pivot.py       # PIVOT 三段式（现 2365–2468 抽成小状态机，并**补上超时**）
  node.py        # ROS 接线：订阅/发布/定时器 + 顶层 dispatch（现 581–958, 2005–2078 的骨架）
```

优先级建议：**先抽 `route.py` 与 `pursuit.py`（纯函数、可离线单测，风险最低）**，
再抽 `shoot.py`（行为最复杂但接口清晰），`node.py` 的接线最后动。
重写时保留的"不可退让"契约：`pursuitAngle` 的 QoS（depth 1 / BEST_EFFORT）、
`angular.z` 是**度**、零 Twist = 刹车、50 Hz 保活（否则 dzactuator 2 s 看门狗可能被其它模式抢占）。

---

## 9. 与本次任务相关的核对结论（供决策）

| 问题 | 结论 | 依据 |
|---|---|---|
| `mission_blocks` 里的 `*` 是射击停站？ | 是，且 `:SHOOT_STOP` 是等价写法 | 1182, 1177 |
| 有 `ALIGN:` / `PIVOT:` 前缀吗？ | **无**（两者均自动推导） | 1926–1954, 2372–2411 |
| 桥洞保护是硬编码 K 列吗？ | 不是，配置驱动（YAML 给 K1…K11） | 259 vs robot.yaml:252 |
| 巡航速度 0.15 m/s 能改吗？ | 能（YAML `linear_speed`） | 441–442 |
| 前视距离/轴距能改吗？ | **不能**，硬编码 0.15 / 0.157 | 621, 629 |
| 越界急停在所有分支都生效吗？ | **否**，PIVOT 蠕动分支缺口 | 2489, 2512 vs 2443–2468 |
| 打靶会卡死路线吗？ | 不会，四阶段各带超时 + 总硬超时 30 s | 2034–2193, 2174 |
| 文件路径模式会触发停站吗？ | 不会 | 935–937 |
