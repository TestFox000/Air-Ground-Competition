# 补丁 03 —— 跟随器安全加固（显式开始 + 接驳段上限 + HOLD 防御）

| 项 | 值 |
| --- | --- |
| 无人机目标路径 | `/home/duzhong/dzpx4/src/px4_path/src/path_follow_waypoint_node.cpp`<br>`/home/duzhong/dzpx4/src/px4_path/include/px4_path/path_follow_waypoint_node.hpp`<br>`/home/duzhong/dzpx4/src/px4_path/src/path_follow_trajectory_node.cpp`<br>`/home/duzhong/dzpx4/src/px4_path/include/px4_path/path_follow_trajectory_node.hpp` |
| 本地文件 | `files/` 下同名 4 个文件 |
| 是否要编译 | **要**（`colcon build --packages-select px4_path`） |
| 风险 | 中 —— 改了状态机入口条件，逻辑可预期但需实测 |

改动规模：`path_follow_waypoint_node.cpp` +45/-3，`path_follow_trajectory_node.cpp` +63/-3，
两个 hpp 各 +5/+7 行。**没有删除任何原有功能，全部是新增的闸门与日志。**

---

## 一、为什么两个跟随器都要改

`px4_path` 有两个跟随器，由 `params.yaml` 的 `follow_mode` 选择：

- `waypoint` → `path_follow_waypoint_node.cpp`
- `trajectory` → `path_follow_trajectory_node.cpp`

**当前 `params.yaml:15` 是 `waypoint`。** 两个文件里这段逻辑是**逐字重复**的：

```cpp
// path_follow_waypoint_node.cpp:241-258  和  path_follow_trajectory_node.cpp:430-448
void ...::on_goto_enabled(const std_msgs::msg::Bool::SharedPtr m) {
  goto_enabled_ = m->data;
  ...
  activate_if_ready();      // 只要许可为真 + 状态 LOADED 就开飞
}
```

⇒ 只改一个，切模式就失效。所以两个都改。

## 二、改了什么（3 项）

### 2.1 显式开始闸门（核心）

`/px4_control/goto_enabled` 是 **transient_local 锁存话题**（两侧 QoS 都是
`rclcpp::QoS(1).reliable().transient_local()`）。锁存的含义是：
**新的订阅者一建立连接，就会立刻收到上一次发布的值，而不是"等下一次变化"。**

于是原来的行为是：

```
飞机已悬停 + 操作员之前按过 '4'（许可已置真并锁存）
  -> 此刻启动/重启 path_follow_node
  -> 立刻收到锁存的 true
  -> INIT --(首个 local_position)--> LOADED --(activate_if_ready)--> PURSUING
  -> 飞机马上沿路径飞走
```

**操作员来不及看日志、来不及检查加载的是哪条路径。**

改法（两个跟随器一致）：

- 新增成员 `goto_permission_observed_` / `enable_transition_seen_`。
- **第一帧只当基线**，不视为命令：
  - 若第一帧就是 `true` → 打 WARN `GOTO permission is latched ON at startup; NOT starting.`，**不开飞**。
  - 只有在之后观测到一次真正的 `false -> true` 跳变，才置 `enable_transition_seen_ = true`。
- `activate_if_ready()` 增加闸门：`require_fresh_enable_ && !enable_transition_seen_` 时拒绝进入
  `PURSUING`，并每 5 s 打一条提示。

新增参数 `require_fresh_enable`（**默认 `true`**，设 `false` 即恢复旧行为）：

```yaml
path_follow_node:
  ros__parameters:
    require_fresh_enable: true
```

**操作手势：在 `px4_control` 里先按 `'5'` 再按 `'4'`。**

为什么这样就行得通 —— 已核实按键实现：

| 键 | 代码 | 效果 |
| --- | --- | --- |
| `'5'` | `px4_control_node.cpp:487` → `disable_goto_and_hold`（`:152-162`） | 置 `goto_enabled_=false` 并 **publish** |
| `'4'` | `handle_goto_enable()`（`:468-484`） | `state_==HOVER && local_pos_valid_` 才放行；**已在使能态时直接 return，不重发**；否则置 `true` 并 publish |

所以 `5 → 4` 一定产生一次干净的 `false → true` 边沿。**没有引入新话题、没有引入新按键。**

覆盖到的场景：

| 场景 | 行为 |
| --- | --- |
| 全新启动（许可锁存为 false） | 照旧：按 `'4'`（悬停后）即开始 |
| 许可已锁存为 true 时启动节点 | **不开飞**，按 `5` 再 `4` 才开始 |
| 飞行中按 `'5'` → `PAUSED`，再按 `'4'` | 边沿成立，原地续跑（不变） |
| 内部错误发 `active=false` 中止后 | control 置假并发布；操作员再按 `'4'` 会重新开始（这是**有意**的：那是明确的按键动作） |

### 2.2 接驳段长度上限（trajectory 专用）

`make_execution_segments()` 生成"从当前位置连到路径首点"的接驳段时，
只用 `max_step_dist` 控制**打点密度**：

```cpp
// path_follow_trajectory_node.cpp:213-214
const std::size_t segment_count =
    static_cast<std::size_t>(std::ceil(start_distance / max_join_step));
```

`start_distance` **本身没有任何上限**。若加载到的路径起点在 50 m 外
（读错文件、地图坐标不对、起飞点跟录制点不同），飞机就会先朝那个方向飞 50 m。

`waypoint` 模式有 `clamp_step()`（`path_follow_waypoint_node.cpp:104-117`）兜底，
**`trajectory` 模式没有**。所以新增参数：

```yaml
max_join_length_m: 0.0    # <=0 关闭；建议首次试飞前设 5.0 或 10.0
```

超过即 `start_trajectory()` 返回 `false` → 上层 `publish_stop()` → control 关许可悬停，
并打 `trajectory start refused: distance from current position to the recorded path start is XX.XXm, ...`。
**默认 0 = 不改变现有行为**，需要你显式开启。

### 2.3 两处防御性修复

- **HOLD 空容器**：两个跟随器的 `HOLD` 分支原本直接取 `trajectories_.back()` /
  `points_.back()`，没有空检查。虽然当前状态机下不可达（`HOLD` 只能从
  `PURSUING` 进入，而 `PURSUING` 已校验非空），但空 `std::vector::back()` 是 UB，且
  后续改动很容易把它变成可达。已加检查 + 报错 + 退回 `LOADED`。
- **日志文案纠错**：两个跟随器的

  ```cpp
  "startup path load failed; press '4' after creating a valid path"
  ```

  **是错的**。`'4'` 只打开 GOTO 许可，节点**不会**重新读取路径文件 ——
  路径只在构造函数里读一次，没有重载入口。已改为
  `"... this node reads the path file only once at startup, so fix the file and restart the node"`。

## 三、应用步骤

```bash
# 1) 备份（4 个文件）
cd ~/dzpx4/src/px4_path
mkdir -p ~/px4_path-bak-$(date +%Y%m%d)
cp src/path_follow_waypoint_node.cpp                    ~/px4_path-bak-$(date +%Y%m%d)/
cp src/path_follow_trajectory_node.cpp                  ~/px4_path-bak-$(date +%Y%m%d)/
cp include/px4_path/path_follow_waypoint_node.hpp       ~/px4_path-bak-$(date +%Y%m%d)/
cp include/px4_path/path_follow_trajectory_node.hpp     ~/px4_path-bak-$(date +%Y%m%d)/

# 2) 用本目录 files/ 下 4 个文件覆盖到上面表格里的 4 个路径

# 3) 编译（只编这一个包，很快）
cd ~/dzpx4
colcon build --packages-select px4_path
source install/setup.bash
```

若编译报错，把报错原文发我，我改本地文件后再给你一版。

## 四、验证

### 4.1 不需要接飞控的台架验证（推荐先做这个）

**前提：不要启动 `px4_control`**（否则你会在真实的许可话题上注水）。
只启动一个可控的发布者 + 跟随节点：

```bash
source /opt/ros/humble/setup.bash && source ~/dzpx4/install/setup.bash

# 终端 A：以锁存 QoS 持续发布 true（模拟"上次留在使能态"）
ros2 topic pub -r 1 --qos-durability transient_local \
    /px4_control/goto_enabled std_msgs/msg/Bool "{data: true}"

# 终端 B：启动跟随节点
ros2 launch px4_path px4_path.launch.py
```

期望在终端 B 看到：

```
[WARN] GOTO permission is latched ON at startup; NOT starting. Press '5' (disable) then '4' (enable) to start explicitly.
```

（**不应该**出现 `pursuit: INIT -> PURSUING` 或 `LOADED -> PURSUING`。）

然后在终端 A 改成 `{data: false}` 发一次，再改回 `{data: true}`，期望看到：

```
[INFO] explicit GOTO enable edge observed; start permitted
```

若飞机没接、`local_position` 无效，会停在 `LOADED` 并周期性打出
`start blocked: waiting for an explicit OFF->ON edge ...` —— 这条也说明闸门在工作。

### 4.2 参数生效确认

```bash
grep -E 'require_fresh_enable|max_join_length_m' \
     ~/dzpx4/install/px4_path/share/px4_path/config/params.yaml
```

节点启动日志里也应出现 `require_fresh_enable=true max_join_length=0.0`。

### 4.3 整机验证（助手只读执行）

```bash
ros2 topic echo --once /px4_control/goto_enabled      # 看锁存值
ros2 topic hz /px4_control/goto                        # PURSUING 时应有 ~20 Hz
```

## 五、回退

```bash
cd ~/dzpx4/src/px4_path
cp ~/px4_path-bak-*/path_follow_waypoint_node.cpp              src/
cp ~/px4_path-bak-*/path_follow_trajectory_node.cpp            src/
cp ~/px4_path-bak-*/path_follow_waypoint_node.hpp              include/px4_path/
cp ~/px4_path-bak-*/path_follow_trajectory_node.hpp            include/px4_path/
cd ~/dzpx4 && colcon build --packages-select px4_path && source install/setup.bash
```

或者：只把 `params.yaml` 的 `require_fresh_enable` 改回 `false` 重新编译，
即可在不回退代码的情况下恢复旧的自动开始行为。

---

## 六、本补丁**没有**修的东西（重要，别当成修好了）

### R-02「追赶速度不受 `max_speed_mps` 约束」

`on_timer()` 里当跟踪误差超过 `tracking_pause_error_m` 时，参考时钟被冻结
（`time_scale = 0`）：

```cpp
if (current_error >= tracking_pause_error_m_) {
  desired_time_scale = 0.0;                    // path_follow_trajectory_node.cpp:484-485
}
...
publish_reference(sample, timer_dt, reference_time_scale_, time_scale_rate);
```

`publish_reference()` 里 `velocity = input.velocity * time_scale` ⇒ 前馈速度变成 0，
而位置目标停在冻结时的参考点上。PX4 的位置环会用自己的
`MPC_XY_VEL_MAX`（默认远大于 0.8）飞向那个点。

**`max_speed_mps: 0.8` 只约束轨迹自身的速度曲线，不约束追赶阶段。**

顺带解释了为什么 `max_tracking_error_m: 5.0` 几乎永不触发：参考冻结后误差不再增长，
飞机在追一个静止的点，误差单调下降。

**为什么本补丁不动它**：真正的修法要么改发布语义（发"限速的胡萝卜点"而不是远端位置点，
需要选定 lookahead 并实测 `MPC_XY_P` 的实际响应），要么改 Offboard 控制模式
（位置 → 速度），**两者都属于控制参数整定，必须上台架/系留实测**，不能凭静态分析发货。
`px4_path` 的 `waypoint` 模式其实已经有一个可借鉴的先例（`clamp_step()` +
`max_step_dist`），可以照它给 trajectory 加同类限幅 —— 但那会改变轨迹跟踪的动态，
需要你决定是否要在这个阶段引入。

**现在的实际暴露度**：只要 `follow_mode` 保持 `waypoint`，R-02 不生效
（waypoint 有 `clamp_step` 兜底）。切到 `trajectory` 且路径起点离当前位置较远时才会暴露 ——
`max_join_length_m` 能挡住最坏的那种（起点很远），但挡不住"路径中段跟不上"的情况。

### 其他

`fc-audit/飞控代码整理.md` 里另有 17 条风险（除上面这两项外），本轮未动。
