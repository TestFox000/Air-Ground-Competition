# 工作约定（必须遵守）

> 用户明确要求：**先在本地改完代码，之后由用户手动替换/编译到无人机上。**
> 助手可以 SSH 连上无人机，但**只能只读**。

## 规则

1. **本地优先（唯一写入路径）**：所有代码修改 —— 本仓库插件、脚本、以及要同步到无人机的源码/配置/补丁 —— 一律先在本地完成。
2. **SSH 只读**：助手对无人机（`dzsmart.local`，用户 `duzhong`）的 SSH 仅用于**只读检查**——`ls` / `cat` / `pgrep` / 读日志 / `ros2 topic list` / `ros2 topic hz` / `ros2 topic info`。**不得**在无人机上创建、修改、删除任何文件，不得装软件包，不得改配置。
3. **不代跑部署**：补丁由助手在本地产出；**替换文件、`colcon build`、`source install/setup.bash` 全部由用户在无人机上手动执行**。助手不代执行任何写入类命令。
4. **可追溯**：每个补丁在 `fc-audit/fixes/<NN-名称>/` 下独立成目录，必须包含
   - `files/` —— 可直接覆盖的**完整文件**（不是 diff 片段）；
   - `APPLY.md` —— 替换到哪个路径、**要不要重新编译**、怎么验证、怎么回退。

## 补丁应用通用流程

1. 助手在本地 `fc-audit/fixes/` 产出补丁。
2. 用户在无人机上**先备份**原文件（`APPLY.md` 里给了具体命令）。
3. 用户把 `files/` 里的文件覆盖到 `APPLY.md` 指定的路径。
4. 若 `APPLY.md` 标注"需要编译"：用户在 `~/dzpx4` 执行
   `colcon build --packages-select <包名>`，然后 `source install/setup.bash`。
5. 助手用**只读** SSH 复验（话题、进程、日志）。

## 需要注意的编译/安装真相（已从 CMakeLists 核实）

| 改动对象 | 是否要 `colcon build` | 原因 |
| --- | --- | --- |
| `*/src/*.cpp`、`*/include/*.hpp` | **要** | 编译产物在 `install/lib/` |
| `*/launch/*.py` | **要** | `CMakeLists` 用 `install(DIRECTORY launch ...)` 拷贝到 `install/share/`；`ros2 launch` 从 install 读 |
| `px4_path/config/params.yaml` | **要** | 同上，`px4_path.launch.py` 用 `FindPackageShare` 定位 install 里的副本 |
| `px4_path/src/path/*.txt` | **不要** | `params.yaml` 里用的是绝对路径 `/home/duzhong/dzpx4/src/px4_path/src/path`，直读源码目录。（`CMakeLists.txt:73` 的 `if(EXISTS .../path)` 条件永假，本来也没安装。） |

例外：若工作空间是用 `colcon build --symlink-install` 建的，launch/config 是符号链接，则改完即刻生效。用 `ls -l ~/dzpx4/install/px4_path/share/px4_path/config/params.yaml` 判断——是软链就不必重编译。

## 当前状态

- 无人机：`dzsmart.local`，用户 `duzhong`，Jetson Orin Nano / Ubuntu 22.04 / ROS 2 Humble。
  换网后地址从 `10.240.174.x` 变为 `10.158.138.x`（旧网段已失效）。
- 源码工作空间：`~/dzpx4`，`src/` 下 7 个包（`px4_msgs`、`px4_ros_com`、`px4_odom_convert`、`px4_imu_bridge`、`px4_control`、`px4_path`、`FAST_LIO_LOCALIZATION_ROS2`）。
- VNC：`:1`（端口 5901，密码 `duzhong`），`tigervncserver@:1.service` 开机自启。
- 助手 SSH：需 `-4` 强制 IPv4（`dzsmart.local` 会先解析出 IPv6 导致挂起）；
  `known_hosts` 用仓库内 `tools/drone/known_hosts_dz`，不动用户的 `~/.ssh/known_hosts`。
- 本地镜像：`fc-audit/src-mirror/` 已与机上 `~/dzpx4/src` 逐文件 SHA256 比对一致。

## 历史脚本清单（换网/接入阶段的旧工具，仍可参考）

| 脚本 | 用途 |
| --- | --- |
| `setup_drone_access.py` | 统一接入：探测用户/网络 |
| `install_drone_key.py` | 安装本地 SSH 公钥实现免密登录 |
| `enable_mdns.py` / `fix_drone_avahi.py` / `remote_fix_drone_avahi.sh` | 无人机 Avahi/mDNS（`dzsmart.local`）配置 |
| `rename_drone_hostname.py` | 主机名改为 `dzsmart` |
| `probe_drone_dirs.py` / `probe_drone_vnc.py` / `crack_vnc_passwd.py` | 无人机环境探测（目录 / VNC） |
| `test_drone_users.py` / `ssh_connect.py` | SSH 连通性验证 |
| `find_drone.py` / `sweep_current.py` / `verify_new_ip.py` | 局域网内定位无人机 IP |

> 注：以上脚本属于"网络接入"阶段产物。**飞控代码类改动一律走 `fc-audit/fixes/` 流程，不再用这些脚本推送。**
