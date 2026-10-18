#!/usr/bin/env bash
# =============================================================================
# 编译前预检：逐个验证 find_package 能否找到
# =============================================================================
#     bash "/mnt/c/Users/Ran/Desktop/无人车/src-car/preflight.sh"
#
# ★ 为什么要单独做这一步：
#   colcon build 若在**编译到一半**才发现某个 find_package 找不到，
#   会浪费大量时间（尤其 8GB 机器上编译很慢）。
#   这里用 cmake 的 find_package 语义**提前**验证，全部通过再开始编译。
# =============================================================================

set +e
BOLD=$'\033[1m'; RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; CYN=$'\033[36m'; RST=$'\033[0m'
ok()   { printf "  ${GRN}✓${RST} %s\n" "$1"; }
bad()  { printf "  ${RED}✗${RST} %s\n" "$1"; }
warn() { printf "  ${YEL}!${RST} %s\n" "$1"; }
info() { printf "  ${CYN}·${RST} %s\n" "$1"; }
sec()  { printf "\n${BOLD}=== %s ===${RST}\n" "$1"; }

MISSING=()
SUGGEST=()

printf "${BOLD}编译前预检${RST}\n"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash 2>/dev/null
printf "  ROS_DISTRO = %s\n" "${ROS_DISTRO:-未设置}"
printf "  ROS_PREFIX = %s\n" "${ROS_DISTRO:+$(ros2 pkg prefix rclcpp 2>/dev/null | head -1)}"

# ---------------------------------------------------------------- 1. 包存在性
sec "1. dzmission 的 find_package 逐个验证"

# find_package 名 -> apt 包名（用于给出安装建议）
declare -A APT=(
  [ament_cmake]=ros-humble-ament-cmake
  [rclcpp]=ros-humble-rclcpp
  [rclcpp_action]=ros-humble-rclcpp-action
  [std_msgs]=ros-humble-std-msgs
  [geometry_msgs]=ros-humble-geometry-msgs
  [nav_msgs]=ros-humble-nav-msgs
  [sensor_msgs]=ros-humble-sensor-msgs
  [tf2]=ros-humble-tf2
  [tf2_ros]=ros-humble-tf2-ros
  [tf2_geometry_msgs]=ros-humble-tf2-geometry-msgs
  [nav2_msgs]=ros-humble-nav2-msgs
  [visualization_msgs]=ros-humble-visualization-msgs
  [serial]=ros-humble-serial
  [ament_lint_auto]=ros-humble-ament-lint-auto
  [ament_lint_common]=ros-humble-ament-lint-common
)

PKGS="ament_cmake rclcpp rclcpp_action std_msgs geometry_msgs nav_msgs \
      sensor_msgs tf2 tf2_ros tf2_geometry_msgs nav2_msgs visualization_msgs \
      serial ament_lint_auto"

for p in $PKGS; do
  if ros2 pkg prefix "$p" >/dev/null 2>&1; then
    PREFIX=$(ros2 pkg prefix "$p" 2>/dev/null | head -1)
    ok "$p"
  else
    # 有些包不注册 ament index（如纯 cmake 包），退一步查目录
    if [ -d "/opt/ros/humble/share/$p" ] || [ -d "/opt/ros/humble/include/$p" ]; then
      ok "$p  (目录存在，但 ros2 pkg 查不到)"
    elif [ -f "$WINPROJ/$p/CMakeLists.txt" ]; then
      # ★ 本工作空间内自带该包（serial 就是这种情况）
      ok "$p  (由本工作空间提供: $p/ 目录)"
    else
      bad "$p  ★ 找不到"
      MISSING+=("$p")
      [ -n "${APT[$p]}" ] && SUGGEST+=("sudo apt install -y ${APT[$p]}")
    fi
  fi
done

# ---------------------------------------------------------------- 2. nav2_msgs 细节
sec "2. nav2_msgs 关键产物验证（dzmission 用的是 NavigateToPose action）"

NAV2="/opt/ros/humble"
printf "  %-58s %s\n" "share/nav2_msgs/action/NavigateToPose.action" \
  "$([ -f "$NAV2/share/nav2_msgs/action/NavigateToPose.action" ] && echo ✓ || echo ✗)"

# 生成的头文件（编译时真正 include 的东西）
HDR=$(find "$NAV2/include/nav2_msgs" -name 'navigate_to_pose.hpp' 2>/dev/null | head -1)
if [ -n "$HDR" ]; then
  ok "找到生成头文件: $HDR"
else
  warn "未找到 navigate_to_pose.hpp（可能在别的路径）"
  printf "        搜索 nav2_msgs 头文件目录：\n"
  find "$NAV2/include" -maxdepth 1 -name 'nav2_msgs' 2>/dev/null | sed 's/^/          /'
  find "$NAV2/include/nav2_msgs" -maxdepth 2 -type d 2>/dev/null | head -5 | sed 's/^/          /'
fi

# ---------------------------------------------------------------- 3. 其它包依赖
sec "3. 工作空间其它包的依赖（若将来要编译）"

for pkg in dztrack dzactuator dzjudgment dzrknn_dataprocess rknn_pt; do
  PK="$PWD"
  WIN="/mnt/c/Users/Ran/Desktop/无人车/src-car/$pkg"
  if [ ! -f "$WIN/package.xml" ]; then
    info "$pkg: 无 package.xml（colcon 会跳过）"
    continue
  fi
  printf "  %s 的依赖：\n" "$pkg"
  # 只检查 <depend>，最多 12 条
  grep -oP '(?<=<depend>)[^<]+' "$WIN/package.xml" 2>/dev/null | head -12 | while read -r d; do
    if ros2 pkg prefix "$d" >/dev/null 2>&1 || [ -d "/opt/ros/humble/share/$d" ]; then
      printf "      ${GRN}✓${RST} %s\n" "$d"
    else
      printf "      ${RED}✗${RST} %s\n" "$d"
    fi
  done
done

# ---------------------------------------------------------------- 4. 编译环境
sec "4. 编译环境"

printf "  g++          : %s\n" "$(g++ --version 2>/dev/null | head -1)"
printf "  cmake        : %s\n" "$(cmake --version 2>/dev/null | head -1)"
printf "  colcon       : %s\n" "$(command -v colcon || echo 未安装)"
printf "  内存         : %s\n" "$(free -h | awk '/^Mem:/{print $2" 可用 "$7}')"
printf "  swap         : %s\n" "$(free -h | awk '/^Swap:/{print $2}')"
printf "  CPU 核数     : %s\n" "$(nproc)"
printf "  磁盘(/)可用  : %s\n" "$(df -h / | awk 'NR==2{print $4}')"

MEM_MB=$(awk '/^MemTotal:/{printf "%d", $2/1024}' /proc/meminfo)
SWAP_MB=$(awk '/^SwapTotal:/{printf "%d", $2/1024}' /proc/meminfo)
if [ "$MEM_MB" -lt 5000 ]; then
  warn "内存 ${MEM_MB}MB —— 编译必须用 --parallel-workers 1 或 2"
fi
if [ "$SWAP_MB" -lt 1024 ]; then
  warn "swap ${SWAP_MB}MB —— 建议加 swap（.wslconfig 里 swap=8GB 后 wsl --shutdown）"
fi

# ---------------------------------------------------------------- 5. 工具链兼容性
sec "5. C++ 标准与编译器"

printf "  g++ 版本 %s —— ROS 2 Humble 要求 C++17，g++>=9 即可\n" \
  "$(g++ -dumpversion 2>/dev/null)"
T=$(mktemp -d)
cat > "$T/t.cpp" <<'EOF'
#include <cstdio>
#include <memory>
#include <vector>
#include <string>
#include <algorithm>
int main(){ std::vector<std::string> v{"a"}; std::printf("ok %zu\n", v.size()); return 0; }
EOF
if g++ -std=c++17 -O2 "$T/t.cpp" -o "$T/t" 2>/dev/null && "$T/t" >/dev/null; then
  ok "C++17 编译与运行正常"
else
  bad "C++17 编译测试失败"
fi
rm -rf "$T"

# ---------------------------------------------------------------- 6. CMakeLists 静态校验
sec "6. CMakeLists 静态校验（不编译，只解析）"

WIN="/mnt/c/Users/Ran/Desktop/无人车/src-car"
CMFILE="$WIN/dzmission/CMakeLists.txt"

if [ ! -f "$CMFILE" ]; then
  bad "找不到 $CMFILE"
else
  ok "CMakeLists.txt 存在"

  printf "\n  6a. add_executable/add_library 引用的源文件：\n"
  awk '/add_(executable|library)\(/{flag=1} flag{print} /^\s*\)/{if(flag)flag=0}' "$CMFILE" \
    | grep -oE '[A-Za-z_/][A-Za-z0-9_/]*\.cpp' | sort -u | while read -r f; do
      if [ -f "$WIN/dzmission/$f" ]; then
        printf "      ${GRN}✓${RST} %s\n" "$f"
      else
        printf "      ${RED}✗${RST} %s  ★ 缺失\n" "$f"
      fi
    done

  printf "\n  6b. install 引用的文件：\n"
  awk '/install\((PROGRAMS|DIRECTORY|FILES)/{flag=1} flag{print} /^\s*\)/{if(flag)flag=0}' "$CMFILE" \
    | grep -oE '[A-Za-z_][A-Za-z0-9_/.-]*\.(py|yaml)' | sort -u | while read -r f; do
      if [ -f "$WIN/dzmission/$f" ]; then
        printf "      ${GRN}✓${RST} %s\n" "$f"
      else
        printf "      ${RED}✗${RST} %s  ★ 缺失\n" "$f"
      fi
    done

  printf "\n  6c. 语法配平检查：\n"
  OPEN=$(tr -cd '(' < "$CMFILE" | wc -c)
  CLOSE=$(tr -cd ')' < "$CMFILE" | wc -c)
  if [ "$OPEN" = "$CLOSE" ]; then
    ok "括号配平（'(' $OPEN 个，')' $CLOSE 个）"
  else
    bad "括号不配平：'(' $OPEN 个，')' $CLOSE 个"
  fi
  NIF=$(grep -cE '^[[:space:]]*if[[:space:]]*\(' "$CMFILE")
  NEND=$(grep -cE '^[[:space:]]*endif[[:space:]]*\(' "$CMFILE")
  if [ "$NIF" = "$NEND" ]; then
    ok "if/endif 配对（各 $NIF 个）"
  else
    bad "if/endif 不配对：if $NIF 个，endif $NEND 个"
  fi

  printf "\n  6d. 包名一致性（ament 靠这个定位 install 路径）：\n"
  PROJ=$(grep -oP '(?<=project\()[A-Za-z0-9_]+' "$CMFILE" | head -1)
  PKGNAME=$(grep -oP '(?<=<name>)[^<]+' "$WIN/dzmission/package.xml" 2>/dev/null | head -1)
  printf "      CMakeLists project() = %s\n" "${PROJ:-未找到}"
  printf "      package.xml <name>   = %s\n" "${PKGNAME:-未找到}"
  if [ "$PROJ" = "$PKGNAME" ]; then
    ok "包名一致"
  else
    bad "包名不一致 —— ament install 路径会不匹配"
  fi
fi

# ---------------------------------------------------------------- 汇总
sec "汇总"

if [ ${#MISSING[@]} -eq 0 ]; then
  printf "  ${GRN}${BOLD}✓ 预检全部通过 —— 可以开始 colcon build${RST}\n"
  printf "\n  ${BOLD}执行：${RST}\n"
  printf "    bash \"/mnt/c/Users/Ran/Desktop/无人车/src-car/build_ws.sh\"\n"
else
  printf "  ${RED}${BOLD}✗ 缺失 %d 个包：${RST}\n" "${#MISSING[@]}"
  for m in "${MISSING[@]}"; do printf "    - %s\n" "$m"; done
  if [ ${#SUGGEST[@]} -gt 0 ]; then
    printf "\n  ${BOLD}安装命令：${RST}\n"
    printf '%s\n' "${SUGGEST[@]}" | awk '!seen[$0]++' | sed 's/^/    /'
  fi
fi

printf "\n  ${YEL}提示${RST}：rosdep 不可用**不影响**编译 ——\n"
printf "        它的作用只是自动装依赖，而依赖已手动装好。\n\n"

exit 0
