#!/usr/bin/env bash
# =============================================================================
# 查询这些依赖在 ROS 2 Humble 里**到底存不存在**
# =============================================================================
#     bash "/mnt/c/Users/Ran/Desktop/无人车/src-car/check_deps.sh"
#
# ★ 为什么需要这一步：
#   修改 package.xml 前必须确认"要删的依赖是真的不存在"。
#   例如 `tf` 是个短名字 —— 它可能是 ROS 1 的 tf（ROS 2 里没有），
#   也可能是某个 ROS 2 端口。**猜错会导致删掉真正需要的依赖**。
#   所以这里逐个查 apt 源与 /opt/ros/humble。
# =============================================================================

set +e
BOLD=$'\033[1m'; RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; CYN=$'\033[36m'; RST=$'\033[0m'
ok()   { printf "  ${GRN}✓${RST} %-28s %s\n" "$1" "$2"; }
bad()  { printf "  ${RED}✗${RST} %-28s %s\n" "$1" "$2"; }
warn() { printf "  ${YEL}!${RST} %-28s %s\n" "$1" "$2"; }
info() { printf "  ${CYN}·${RST} %-28s %s\n" "$1" "$2"; }
sec()  { printf "\n${BOLD}=== %s ===${RST}\n" "$1"; }

source /opt/ros/humble/setup.bash 2>/dev/null
printf "${BOLD}依赖可用性查询（ROS 2 Humble）${RST}\n"
printf "  ROS_DISTRO = %s\n" "${ROS_DISTRO:-未设置}"

# 待查的依赖名（来自 package.xml）
DEPS=(
  tf tf2 tf2_msgs tf2_ros tf2_geometry_msgs
  actionlib actionlib_msgs
  message_generation message_runtime
  rclcib rclcpp rclpy
  nav_msgs sensor_msgs geometry_msgs std_msgs std_srvs
  visualization_msgs lifecycle_msgs builtin_interfaces
  cv_bridge image_transport
  pcl_conversions pcl_ros laser_geometry
  eigen3_cmake_module
  nav2_msgs nav2_util nav2_common nav2_controller nav2_lifecycle_manager
  joint_state_publisher joint_state_publisher_gui robot_state_publisher rviz
)

sec "逐个检查（ros2 pkg prefix + /opt/ros/humble 目录 + apt 候选）"

printf "  %-28s %-10s %-10s %s\n" "依赖名" "ros2 pkg" "目录" "apt 候选版本"
printf "  %s\n" "----------------------------------------------------------------------------"

AVAILABLE=()
MISSING=()

for d in "${DEPS[@]}"; do
  # 1) ros2 pkg prefix
  if ros2 pkg prefix "$d" >/dev/null 2>&1; then
    PK="yes"; AVAILABLE+=("$d")
  else
    PK="no"
  fi
  # 2) /opt/ros/humble 目录
  if [ -d "/opt/ros/humble/share/$d" ] || [ -d "/opt/ros/humble/include/$d" ]; then
    DIR="yes"; [ "$PK" = "no" ] && AVAILABLE+=("$d")
  else
    DIR="no"
  fi
  # 3) apt 候选
  APT=$(apt-cache policy "ros-humble-$d" 2>/dev/null | awk '/Candidate:/{print $2}')
  [ -z "$APT" ] && APT="(无)"

  if [ "$PK" = "yes" ] || [ "$DIR" = "yes" ]; then
    printf "  ${GRN}%-28s %-10s %-10s %s${RST}\n" "$d" "$PK" "$DIR" "$APT"
  elif [ "$APT" != "(无)" ] && [ "$APT" != "(none)" ]; then
    printf "  ${YEL}%-28s %-10s %-10s %s${RST}  (可 apt 装)\n" "$d" "$PK" "$DIR" "$APT"
  else
    printf "  ${RED}%-28s %-10s %-10s %s${RST}  ★ 不存在\n" "$d" "$PK" "$DIR" "$APT"
    MISSING+=("$d")
  fi
done

sec "汇总：不存在的依赖（应从 package.xml 删除或改正）"

if [ ${#MISSING[@]} -eq 0 ]; then
  printf "  ${GRN}没有不存在的依赖${RST}\n"
else
  for m in "${MISSING[@]}"; do
    case "$m" in
      rclcib)              printf "  ${RED}%-20s${RST} ★ 拼写错误，应为 rclcpp\n" "$m" ;;
      message_generation)  printf "  ${RED}%-20s${RST} ROS 1 专用（ROS 2 用 rosidl_default_generators）\n" "$m" ;;
      message_runtime)     printf "  ${RED}%-20s${RST} ROS 1 专用（ROS 2 用 rosidl_default_runtime）\n" "$m" ;;
      tf)                  printf "  ${RED}%-20s${RST} ROS 1 专用（ROS 2 用 tf2）\n" "$m" ;;
      actionlib)           printf "  ${RED}%-20s${RST} ROS 1 专用（ROS 2 用 rclcpp_action）\n" "$m" ;;
      *)                   printf "  ${RED}%-20s${RST} 不存在\n" "$m" ;;
    esac
  done
fi

sec "补充：这些名字很像但不等于 tf（供参考）"
printf "  搜索 apt 里所有含 'tf' 的 ROS 包：\n"
apt-cache search '^ros-humble-tf' 2>/dev/null | head -10 | sed 's/^/      /'
printf "\n  搜索 apt 里所有含 'actionlib' 的 ROS 包：\n"
apt-cache search 'ros-humble-actionlib' 2>/dev/null | head -10 | sed 's/^/      /'
printf "\n  搜索 apt 里所有含 'message' 的 ROS 包（前 10）：\n"
apt-cache search '^ros-humble-message' 2>/dev/null | head -10 | sed 's/^/      /'

printf "\n  ${BOLD}把本脚本完整输出贴回给助手。${RST}\n\n"
exit 0
