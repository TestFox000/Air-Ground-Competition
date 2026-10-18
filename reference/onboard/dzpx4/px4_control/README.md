# px4_control

`px4_control` 只负责键盘命令、外部 Goto 准入和 PX4 Offboard 控制。路径文件、
录制、优化与进度均由 `px4_path` 管理，依赖方向只有
`px4_path -> px4_control`。

## 节点与接口

- `px4_control_node`：Offboard 心跳、解锁、起飞、悬停、降落、Goto 门控和
  `TrajectorySetpoint` 下发。
- `px4_teleop_node`：读取终端键盘，只发布 `/px4_control/command`。

控制节点接收：

- `/px4_control/command` (`px4_control/msg/ControlCommand`)
- `/px4_control/goto` (`px4_control/msg/GotoTarget`)

控制节点以 reliable + transient-local QoS 发布：

- `/px4_control/goto_enabled` (`std_msgs/msg/Bool`)

`goto_enabled` 默认为 `false`。只有飞机处于 `HOVER` 且本地位置有效时，按键
`4` 才能把它设为 `true`。按键 `5` 会关闭许可、清除前馈，并把按键瞬间的
当前位置固化为悬停目标。许可关闭后，持续到来的 `active=true` 目标不会覆盖
该悬停点。

`GotoTarget(active=true)` 的位置、速度、加速度、yaw 和 yawspeed 必须全部为
有限数；`active=false` 会关闭 Goto 并悬停。Goto 超时、无效目标、降落、锁桨
请求或退出 Offboard 也会关闭许可。

自主下降只负责持续提供下降目标。触地判断和自动锁桨由 PX4 完成，控制节点只在
`VehicleStatus.arming_state == STANDBY` 后进入 `LANDED`。

## 启动

```bash
# 终端 1：控制节点
ros2 launch px4_control px4_control.launch.py

# 终端 2：键盘节点必须运行在真实 TTY 中
ros2 run px4_control px4_teleop_node --ros-args \
  --params-file install/px4_control/share/px4_control/config/params.yaml
```

## 键位

| 键 | 操作 |
|---|---|
| `0` | 请求 Offboard |
| `1` | 解锁并起飞 |
| `2` | 请求锁桨 |
| `3` | 请求降落 |
| `4` | 允许执行外部 Goto |
| `5` | 禁止 Goto，并固化当前位置悬停 |
| `W/S/A/D/Q/E` | 调整悬停目标位置 |
| `L/R` | 调整目标 yaw |

Goto 许可打开期间，WASDQE 和 L/R 被屏蔽，即使第一帧 Goto 尚未到达也不会
修改目标。
