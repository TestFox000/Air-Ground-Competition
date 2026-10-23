# 补丁与验证脚本（无人机 ↔ 小车 链路）

本目录是**方案 A（域号收敛）**的全部可复现材料。所有脚本都**先备份、幂等、自带断言**，
并且都配一个只读验证脚本 —— 不接受"改完就算完"。

## 一、为什么会有这些脚本

2026-10-07 联调事故：小车在 ROS **域 118**（来自 `~/.bashrc:126`），无人机侧**一处都没设过**（默认域 **0**）。
两边域号不同时：`ping` 通、SSH 通、网页台 `:8080` 通，但 **ROS 死活看不见对方，而且一个错都不报**。
排查因此白白花了一整轮（我在域 0 的 `7400` 端口嗅探，自然嗅不到域 118 的 `239.255.0.1:36900`）。

根治办法不是"记住要设域号"，而是**把它钉进脚本**：任何起法都必须得到同一个域号。

## 二、两份"唯一真相源"

| 文件 | 装到哪 | 装完的 md5 |
| --- | --- | --- |
| `dzros.env` | 小车 `~/dzrobot/dzros.env` | `43e35e36ab6b5cd58e9d3cb574e4f33a` |
| `dzlink.env` | 无人机 `~/dzlink_tools/dzlink.env` | `a2cae2c0c01707a80fd562d52e47f76b` |

两边域号必须一致（当前 **118**）。改域号 = 只改这两个文件。

车侧 5 个 ROS 入口全部 source `dzros.env`：
`start_all.sh` / `monitor_aim.sh` / `tools/healthcheck.sh` / `tools/restart_node.sh` / `tools/test_voice.sh`。

## 三、脚本清单

| 脚本 | 作用 | 幂等标记 |
| --- | --- | --- |
| `patch_car_domain.py` | 车侧 v1：只改 `start_all.sh` | `DZ_ROS_DOMAIN` |
| `patch_car_domain_v2.py` | 车侧 v2：新建 `dzros.env` + 改 5 个入口（**v1 之后才跑**） | `全车唯一真相源` |
| `patch_drone_sender.py` | 无人机侧：装 `dzlink.env` + 发送器自动读 env/打印域号 | `domain_report` |
| `verify_car_domain.sh` | 车侧只读验证（6 个用例，**不起任何节点**） | — |
| `verify_drone_link.sh` | 无人机侧只读验证（含真实 rclpy 发送一次） | — |

用法（脚本先 scp 到对端 `/tmp/`，模板文件放同目录）：

```bash
# 车侧（模板 dzros.env 必须与脚本同目录）
scp patch_car_domain.py patch_car_domain_v2.py verify_car_domain.sh dzros.env <车>:/tmp/
ssh <车> 'python3 /tmp/patch_car_domain_v2.py --dry-run'   # 先干跑看锚点
ssh <车> 'python3 /tmp/patch_car_domain_v2.py'             # 再实打（自动备份 + bash -n）
ssh <车> 'bash /tmp/verify_car_domain.sh'

# 无人机侧
scp patch_drone_sender.py dzlink.env <机>:/tmp/
ssh <机> 'python3 /tmp/patch_drone_sender.py --dry-run'
ssh <机> 'python3 /tmp/patch_drone_sender.py'
ssh <机> 'bash /tmp/verify_drone_link.sh'
```

## 四、回退（全部改动都有备份）

```bash
# 车侧
cp ~/dzrobot/_backups/start_all.sh.bak_<时间戳>      ~/dzrobot/start_all.sh
cp ~/dzrobot/_backups/monitor_aim.sh.bak_<时间戳>    ~/dzrobot/monitor_aim.sh
cp ~/dzrobot/_backups/healthcheck.sh.bak_<时间戳>    ~/dzrobot/tools/healthcheck.sh
cp ~/dzrobot/_backups/restart_node.sh.bak_<时间戳>   ~/dzrobot/tools/restart_node.sh
cp ~/dzrobot/_backups/test_voice.sh.bak_<时间戳>     ~/dzrobot/tools/test_voice.sh
rm -f ~/dzrobot/dzros.env

# 无人机侧
cp ~/dzlink_tools/_backups/dz_send_waypoint.py.bak_<时间戳> ~/dzlink_tools/dz_send_waypoint.py
rm -f ~/dzlink_tools/dzlink.env
```

## 五、两个"看起来能省事、实际会踩"的坑

1. **`source /opt/ros/humble/setup.bash` 不能在有 `set -u` 的脚本里裸调** ——
   它第 8 行会展开未定义变量 `AMENT_TRACE_SETUP_FILES`，脚本会**静默终止**。
   必须在 `set +u` / `set -u` 之间 source。本轮验证脚本自己就复现了这个坑（已修）。
2. **PowerShell 5.1 给 `ssh.exe` 传参时会吃掉双引号** ——
   `ssh host 'echo "a b"'` 到远端会变成 `echo a b`，症状是"命令行为莫名不对"或 grep 静默无输出。
   可靠做法：**把远端要跑的东西写成脚本 scp 过去再执行**，别拼长命令行。
