#!/bin/bash
# t4_rosenv.sh —— 统一入口：source ROS + 域号真相源，然后 exec 传入的命令
# 用法: bash t4_rosenv.sh python3 /tmp/t4_qos_probe.py sub --qos best_effort ...
#
# 为什么必须包一层：域号有两个来源（无人机 ~/dzlink_tools/dzlink.env、小车 ~/dzrobot/dzros.env），
# 而 `ssh host 'cmd'` 这种非交互 shell 不读 ~/.bashrc —— 直接跑 ros2 会掉回域 0。
source /opt/ros/humble/setup.bash 2>/dev/null || true
[ -r "$HOME/dzrobot/install/setup.bash" ] && source "$HOME/dzrobot/install/setup.bash"
[ -r "$HOME/dzpx4/install/setup.bash" ] && source "$HOME/dzpx4/install/setup.bash"
[ -r "$HOME/dzrobot/dzros.env" ] && . "$HOME/dzrobot/dzros.env"      # 车侧唯一真相源
[ -r "$HOME/dzlink_tools/dzlink.env" ] && . "$HOME/dzlink_tools/dzlink.env"  # 机侧唯一真相源
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-118}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
echo "[t4_rosenv] host=$(hostname) ROS_DISTRO=${ROS_DISTRO:-?} ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW=${RMW_IMPLEMENTATION:-default}" >&2
exec "$@"
