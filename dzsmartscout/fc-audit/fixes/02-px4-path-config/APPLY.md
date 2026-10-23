# 补丁 02 —— `px4_path` 路径文件契约修复

| 项 | 值 |
| --- | --- |
| 无人机目标路径 | `/home/duzhong/dzpx4/src/px4_path/config/params.yaml` |
| 本地文件 | `files/params.yaml` |
| 是否要编译 | **要**（config 被拷贝安装到 install/share） |
| 前置动作 | **先备份 `src/path/` 下三个 txt** |

---

## 一、修的是什么

`px4_path/README.md:103-106` 明确要求：

```text
path_follow_node.path_dir  == path_record_node.path_dir
path_follow_node.path_file == path_record_node.output_file
```

而原文件里：

| 行 | 参数 | 原值 |
| --- | --- | --- |
| `params.yaml:25` | `path_follow_node.path_file` | `recorded_path_2.txt` |
| `params.yaml:119` | `path_record_node.output_file` | `recorded_path.txt` |
| `params.yaml:123` | `path_record_node.overwrite_existing` | `false` |

**两个契约同时断裂。** 叠加起来的实际行为：

1. `path_record_node.cpp:178-189`：`overwrite_existing=false` 且目标文件已存在时，
   依次尝试 `recorded_path_1.txt`、`recorded_path_2.txt`、`recorded_path_3.txt`…
   取第一个不存在的。
2. 机上 `src/path/` 下 `recorded_path.txt`、`recorded_path_1.txt`、`recorded_path_2.txt`
   **三个都已存在** ⇒ 下次录制写出的是 **`recorded_path_3.txt`**。
3. 跟随节点读的是 **`recorded_path_2.txt`**。

⇒ **录完一条新路径，飞机飞的仍然是上一次的旧路径，而且没有任何报错。**

（顺带：`path_record_node.cpp:66` 里代码自带默认值是 `true`，是这行 YAML 把它改成了 `false`。
另外 `path_follow_trajectory_node.cpp:77` / `path_follow_waypoint_node.cpp:56` 打印的
"press '4' after creating a valid path" 也是错的 —— `'4'` 只是使能 Offboard 跟随，
不会重新读文件；路径只在节点构造时读一次。）

## 二、改成了什么

| 参数 | 新值 |
| --- | --- |
| `path_follow_node.path_file` | `recorded_path.txt` |
| `path_record_node.output_file` | `recorded_path.txt`（原本就是这个） |
| `path_record_node.overwrite_existing` | **`true`** |

为什么 `overwrite_existing` 必须改成 `true`：只把 `path_file` 对齐到 `recorded_path.txt`
是不够的 —— 下次录制仍会写出 `_3`，而 `recorded_path.txt` 还是 9 月 24 日那份旧文件。
必须让"录制写哪个文件"变成确定的、单一的名字。

为什么这样是安全的：保存走 `path_record_node.cpp:316`（写 `xxx.tmp`）→ `:353`
（`fs::rename` 到目标）。同目录内 `rename` 是原子操作，**录制中途失败或掉电不会破坏旧文件**。
代价是每次录制覆盖上一次的 `recorded_path.txt`；需要留档时手动改名。

## 三、本文件同时包含补丁 03 需要的两个新参数

- `path_follow_node.require_fresh_enable: true`
- `path_follow_node.max_join_length_m: 0.0`

**先应用本补丁、暂不应用补丁 03 也不会出问题**：这两个键在节点里没声明，
ROS 2 会静默忽略未声明的参数覆盖，节点用编译期默认值运行。
（反之，先应用 03 而不应用 02 也不会崩：节点用代码里的默认值，只是不能从 YAML 调。）

## 四、本补丁**没有**改的一件事（需要你决策）

`params.yaml:15` 当前是：

```yaml
follow_mode: "waypoint"
```

也就是**当前跑的是航点跟随器，不是轨迹跟随器**。
上一轮审计里 R-02（接驳段无长度上限、追赶不受 0.8 m/s 约束）主要影响 trajectory 模式。
本补丁**不动这个值**，因为切模式会改变飞行行为，属于你的决策。

判断依据：`px4_path/README.md:76-80` —— waypoint 用 `max_step_dist` 限制单次目标距离，
trajectory 构建曲率感知的连续轨迹并发布速度/加速度前馈。两者取舍不同。

## 五、应用步骤

```bash
# 1) 备份三个路径文件（重要：overwrite_existing 改 true 后会被覆盖）
cp -a ~/dzpx4/src/px4_path/src/path ~/dzpx4/src/px4_path/src/path.bak-$(date +%Y%m%d)

# 2) 备份配置
cp ~/dzpx4/src/px4_path/config/params.yaml ~/dzpx4/src/px4_path/config/params.yaml.bak

# 3) 用本目录 files/params.yaml 覆盖到
#    ~/dzpx4/src/px4_path/config/params.yaml

# 4) 判断是否需要编译
ls -l ~/dzpx4/install/px4_path/share/px4_path/config/params.yaml
#    是普通文件 -> 需要编译；是符号链接 -> 跳过第 5 步

# 5) 编译
cd ~/dzpx4
colcon build --packages-select px4_path
source install/setup.bash
```

## 六、验证（助手只读执行）

```bash
source /opt/ros/humble/setup.bash && source ~/dzpx4/install/setup.bash

# 确认 install 里的配置真的变成了新值（这是生效判据，不是看 src）
grep -E 'path_file|output_file|overwrite_existing|require_fresh_enable|max_join_length_m' \
     ~/dzpx4/install/px4_path/share/px4_path/config/params.yaml

# 跟随节点启动日志里应打印 file=/home/duzhong/dzpx4/src/px4_path/src/path/recorded_path.txt
ros2 launch px4_path px4_path.launch.py
```

期望：日志 `trajectory follower up. file=.../recorded_path.txt`（或 `waypoint follower up. file=.../recorded_path.txt`）。

## 七、回退

```bash
cp ~/dzpx4/src/px4_path/config/params.yaml.bak ~/dzpx4/src/px4_path/config/params.yaml
cd ~/dzpx4 && colcon build --packages-select px4_path && source install/setup.bash
```
