#!/usr/bin/env bash
# 在车端编译本项目。
#
# ⚠️ 这台车是**公用**的（别人在 ~/dzrobot 等目录有数据）。因此：
#   * 工作空间固定放在 **/tmp/ag_ws**，绝不写进家目录；
#   * /tmp 重启即清，工作结束后由 tools/car_cleanup.sh 彻底删除；
#   * 全程不读不写 ~/dzrobot（只在不涉及编译时才只读探测）。
#
# 用法：
#   python tools/remote_run.py --host <车IP> --user duzhong --put src --dest /tmp/ag_ws/src
#   python tools/remote_run.py --host <车IP> --user duzhong --script tools/car_build.sh --timeout 900

set -o pipefail
WS="${AG_WS:-/tmp/ag_ws}"
LOG="/tmp/ag_build.log"

echo "==================== 车端编译 ===================="
date
echo

echo "## 1. 环境"
source /opt/ros/humble/setup.bash 2>/dev/null || { echo "!! 找不到 /opt/ros/humble/setup.bash"; exit 1; }
echo "ROS_DISTRO=$ROS_DISTRO"
echo "colcon    = $(command -v colcon || echo '缺失')"
echo "python3   = $(python3 --version 2>&1)"
echo "numpy     = $(python3 -c 'import numpy;print(numpy.__version__)' 2>/dev/null || echo '缺失')"
echo

echo "## 2. 源码"
ls "$WS/src" 2>/dev/null || { echo "!! $WS/src 不存在，先上传源码"; exit 1; }
echo "包数: $(find "$WS/src" -maxdepth 2 -name package.xml 2>/dev/null | wc -l)"
echo "-- 每个包是否都有 package.xml 同级的构建文件 --"
for d in "$WS/src"/*/; do
  p=$(basename "$d")
  [ "$p" = "COLCON_IGNORE" ] && continue
  printf "  %-16s package.xml:%s setup.py:%s CMakeLists.txt:%s\n" "$p" \
    "$([ -f "$d/package.xml" ] && echo 有 || echo 无)" \
    "$([ -f "$d/setup.py" ] && echo 有 || echo 无)" \
    "$([ -f "$d/CMakeLists.txt" ] && echo 有 || echo 无)"
done
echo

echo "## 3. colcon build"
cd "$WS" || exit 1
rm -rf build install log
colcon build --symlink-install > "$LOG" 2>&1
RC=$?
echo "colcon 退出码 = $RC"
echo
echo "---- 摘要 ----"
grep -E "^(Starting|Finished|Failed|Aborted|Summary)" "$LOG" | tail -30
echo
echo "---- 错误/警告 ----"
grep -nE "error|Error|ERROR|failed|Failed|Traceback|ModuleNotFound" "$LOG" | head -40
echo
echo "---- 日志尾部 ----"
tail -25 "$LOG"
echo

echo "## 4. 安装结果"
if [ -d "$WS/install" ]; then
  ls "$WS/install" | head
  echo "-- 可执行文件 --"
  ls "$WS/install/ag_car/lib/ag_car" "$WS/install/ag_mission/lib/ag_mission" \
     "$WS/install/ag_perception/lib/ag_perception" "$WS/install/ag_bridge/lib/ag_bridge" \
     "$WS/install/ag_referee/lib/ag_referee" "$WS/install/ag_chassis/lib/ag_chassis" 2>/dev/null
  echo "-- msg 生成 --"
  find "$WS/install" -name "*.py" -path "*ag_interfaces*" 2>/dev/null | head -8
else
  echo "!! 没有 install/，编译失败"
fi
echo
echo "完整日志：$LOG"
echo "==================== 结束 ===================="
