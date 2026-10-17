# 飞控补丁暂存区

**这里是本地唯一的改动源头。无人机的文件由用户手动替换、由用户手动编译。**

工作流约定见仓库根目录 [WORKFLOW.md](../../WORKFLOW.md)。

## 怎么用

每个补丁目录里有：

- `files/` —— **完整文件**，直接覆盖到无人机对应路径即可（不是 diff 片段，不需要打补丁工具）。
- `APPLY.md` —— 替换目标路径、是否要 `colcon build`、验证命令、回退命令。

## 补丁索引

| # | 目录 | 改什么 | 要不要编译 | 风险 | 状态 |
| --- | --- | --- | --- | --- | --- |
| 01 | [01-odom-source](01-odom-source/) | `px4_odom_convert` 的定位来源从死掉的 `/localization` 换成活着的 `/Odometry` | 要（launch 装在 install/share）——**但有零编译的运行时替代写法** | 极低 | 已产出，待应用 |
| 02 | [02-px4-path-config](02-px4-path-config/) | `px4_path` 的路径文件契约修复（录制写哪个文件 == 跟随读哪个文件）+ 补丁 03 需要的新参数 | 要（config 装在 install/share） | 低（会覆盖一个旧路径文件，需先备份） | 已产出，待应用 |
| 03 | [03-follow-safety](03-follow-safety/) | 两个跟随器的"显式开始"闸门 + trajectory 连接段长度上限 + HOLD 空轨迹防御 | **要**（C++） | 中（改了状态机入口，行为可预期但需实测） | 已产出，待应用 |
| 04 | [04-camera-bringup](04-camera-bringup/) | USB/UVC 相机点亮 + 出图验证（**不是"覆盖某个文件"，是新增工具包**） | **不要**（纯脚本） | 极低（只读探测，抓帧只写 `/tmp`） | **已在真机验证通过** |
| 05 | [05-drone-to-car-waypoint](05-drone-to-car-waypoint/) | 无人机 → 小车 航点/情报发送器。**小车侧零改动**：用小车现成的 `/nav/waypoint`(String) 与 `/dzlink/drone_to_car`(JSON) | **不要**（纯 Python） | 极低（只发不收，不动两端源码/配置） | **发送器已真机验证：车侧发 2 收 2**；新增链路体检工具 |
| 06 | [06-car-wait-for-drone](06-car-wait-for-drone/) | **车侧**：起栈后**原地待命**，只等无人机指令再动（`auto_start:false` + `DZ_AUTO_START` 环境变量覆盖，保住比赛一键出发）。含完整 [RUNBOOK](06-car-wait-for-drone/RUNBOOK.md) | **不要**，但**必须同步 install 副本**（`ros2 run` 跑的是它，不是 `src/`） | 低（关掉一个自动行为 + 加一层覆盖；有备份/自检/一键回滚） | **已应用到真车并实测通过**（起栈 35 s 未出发、发 2 收 2、回滚逐字节还原） |

> 01–03 是**无人机侧**"覆盖机上某个文件"的补丁；**04** 是拷到 `~/camtools/` 的独立脚本包；
> **05** 是无人机侧发送/体检工具；**06 是第一个车侧补丁**（改的是小车 `~/dzrobot`）。
> 06 的应用方式与前几个不同：由助手直接 SSH 应用（带备份 + 自检 + 回滚脚本），
> 不是"用户在机上手动替换文件"。细节见 [06-car-wait-for-drone/APPLY.md](06-car-wait-for-drone/APPLY.md)。

## 建议的应用顺序

1. **01** —— 它让整条链路第一次真正有数据。不应用 01，后面所有验证都无从谈起。
2. **02 + 03 一起** —— 02 里的 `require_fresh_enable` / `max_join_length_m` 是 03 的配套参数。
   只应用 02 不应用 03 也不会报错（多出来的 YAML 键被 ROS 2 忽略，节点用编译期默认值）。
3. 应用后由助手做**只读**复验。

## 应用前的统一备份

在无人机上（用户手动执行）：

```bash
cd ~/dzpx4/src
tar czf ~/dzpx4-backup-$(date +%Y%m%d-%H%M%S).tgz \
    px4_odom_convert/launch/px4_odom_convert.launch.py \
    px4_path/config/params.yaml \
    px4_path/src/path_follow_waypoint_node.cpp \
    px4_path/include/px4_path/path_follow_waypoint_node.hpp \
    px4_path/src/path_follow_trajectory_node.cpp \
    px4_path/include/px4_path/path_follow_trajectory_node.hpp \
    px4_path/src/path
ls -l ~/dzpx4-backup-*.tgz
```

## 一个必须知道的前置问题（不是代码问题）

`/localization` 从未被发布过，根因是**两个 Python 依赖缺失**，不是代码 bug：

| 缺失模块 | 导致死亡的节点 | 后果 |
| --- | --- | --- |
| `open3d` | `global_localization.py:7`、`global_map_publisher.py:4` | `/map_to_odom`、`/global_map` 永不发布 |
| `tf_transformations` | `transform_fusion.py:13`、`global_localization.py:11`、`publish_initial_pose.py:5` | **`/localization` 永不发布** |

补丁 01 是**绕过**它（改用 `/Odometry`）；要让基于先验地图的**重定位**真正工作，还需要在无人机上补依赖：

```bash
sudo apt install -y ros-humble-tf-transformations
pip3 install open3d
```

这两条属于**装软件包**，按约定由用户手动执行。装完后再把 `mode` 切回 `localization` 即可启用重定位
（届时 `/localization` 不再等于 `/Odometry`，而是带地图修正的结果）。
