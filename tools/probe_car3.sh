#!/usr/bin/env bash
# 车端第三轮：把 ~/dzrobot/src 里「既有包」的接口摸清（只读）。
# 目标：回答 裁判怎么接、底盘/云台怎么控、视觉模型输入输出、有没有 Nav2。
# 跳过第三方大包（navigation2 / robot_localization / rplidar / slam_gmapping / rviz）。

SRC="$HOME/dzrobot/src"
cd "$SRC" || exit 1

hr() { echo "----------------------------------------------------"; }
SKIP="navigation2 robot_localization-humble-devel rplidar_ros-ros2 slam_gmapping-eloquent-devel rviz"

echo "==================== 车端既有包接口清单 ===================="
echo "工作空间: $SRC"
date
echo

hr; echo "## 1. 包清单与用途"
for d in */; do
  p="${d%/}"
  case " $SKIP " in *" $p "*) continue;; esac
  echo "===== $p ====="
  grep -E "<description>|<name>" "$p/package.xml" 2>/dev/null | sed 's/^[[:space:]]*//'
  echo "  文件:"
  find "$p" -maxdepth 2 \( -name "*.py" -o -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.launch*" \) 2>/dev/null | head -15 | sed 's/^/    /'
  echo
done

hr; echo "## 2. 全部话题/服务名（从源码字符串里抽）"
for d in */; do
  p="${d%/}"
  case " $SKIP " in *" $p "*) continue;; esac
  echo "===== $p ====="
  grep -rhoE 'create_(publisher|subscription|service|client)<[^>]*>\(\s*"[^"]+"|create_(publisher|subscription)<[^>]*>\(\s*"[^"]+"' "$p" 2>/dev/null | head -40
  grep -rhoE '"[a-zA-Z_]+(/[a-zA-Z0-9_]+)+"' "$p" 2>/dev/null | sort -u | head -40
done

hr; echo "## 3. 消息类型（接口是否自定义）"
for d in */; do
  p="${d%/}"
  case " $SKIP " in *" $p "*) continue;; esac
  find "$p" -maxdepth 3 \( -name "*.msg" -o -name "*.srv" -o -name "*.action" \) 2>/dev/null | sed 's/^/  /'
done
echo "-- 依赖的 ROS 消息包 --"
grep -rhoE "<(depend|exec_depend|build_depend)>[a-z_]+_msgs</" */package.xml 2>/dev/null | sed -E 's/<[^>]+>//' | sort | uniq -c | sort -rn | head -20

hr; echo "## 4. 关键代码片段：dzactuator（执行机构/开火）"
find ~/dzrobot/src/dzactuator -name "*.py" -o -name "*.cpp" -o -name "*.hpp" 2>/dev/null | head -8 | while read -r f; do
  echo "----- $f -----"; head -60 "$f"
done

hr; echo "## 5. 关键代码片段：dzjudgment（裁判）"
find ~/dzrobot/src/dzjudgment -name "*.py" -o -name "*.cpp" -o -name "*.hpp" 2>/dev/null | head -8 | while read -r f; do
  echo "----- $f -----"; head -60 "$f"
done

hr; echo "## 6. 关键代码片段：naviPointPath（导航目标点接口）"
find ~/dzrobot/src/naviPointPath -name "*.py" -o -name "*.cpp" -o -name "*.hpp" 2>/dev/null | head -8 | while read -r f; do
  echo "----- $f -----"; head -80 "$f"
done

hr; echo "## 7. launch 文件全清单"
find ~/dzrobot/src -maxdepth 3 -name "*.launch*" -not -path "*navigation2*" -not -path "*rplidar*" 2>/dev/null | head -30

hr; echo "## 8. 历史命令里的官方启动方式（只看 ros2/launch 相关行）"
grep -aE "ros2 (launch|run|topic pub)" ~/.bash_history 2>/dev/null | tail -40

echo "==================== 结束 ===================="
