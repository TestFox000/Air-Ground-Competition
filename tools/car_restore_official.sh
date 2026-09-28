#!/usr/bin/env bash
# 把官方栈补全到"能发车、能跑视觉"的状态（在官方工作区 `~/dzrobot` 里编译缺失的包）。
#
# 2026-09-28 实测车上的问题与根因：
#   症状：`car_localization`/`dzjudgment` 能用，但 `det_node` 只装了个空壳、
#         `dzactuator` 根本没装 → 相机起不来、底盘也起不来。
#   根因：`~/dzrobot` 下有 **src / src_1 / src_4 三份源码**，包名重复。
#         colcon 遇到重复包名会**直接拒绝构建**：
#           "Duplicate package names not supported: det_node: src/rknn_pt, src_4/rknn_pt ..."
#         所以谁在这里 `colcon build` 都会失败，install 一直是残缺的。
#   修法：用 `--base-paths src` **只扫正主那份**（src_4 是它的整份副本，src_1 已被
#         COLCON_IGNORE 标记）。零侵入 —— 不改、不删官方任何源码。
#
# 附：`det_node` 不是独立源码包，而是 `~/dzrobot/src/rknn_pt`
#     （其 CMakeLists 是 `project(det_node)`、package.xml 是 `<name>det_node</name>`），
#     所以构建目录叫 build/det_node、包名也叫 det_node。
#
# 用法：python tools/remote_run.py --host <IP> --user duzhong --script tools/car_restore_official.sh

# ⚠️ 不要用 set -u：ROS 的 setup.bash 会引用未定义变量，非交互 shell 会直接退出
set -o pipefail
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
source "$HOME/install/setup.bash" 2>/dev/null

#: 发车 + 视觉都需要的官方包
NEEDED="car_localization dzactuator dzjudgment det_node dzrknn_dataprocess"

hr() { echo "----------------------------------------------------"; }
echo "==================== 补全官方栈（在 ~/dzrobot 里编译） ===================="
date
echo

hr; echo "## 1. 现状：哪些包可见"
MISSING=""
for p in $NEEDED; do
  if ros2 pkg list 2>/dev/null | grep -qx "$p"; then
    echo "  [OK] $p"
  else
    echo "  [缺] $p"
    MISSING="$MISSING $p"
  fi
done
echo

if [ -z "$MISSING" ]; then
  hr; echo "## 2. 没有缺失，跳过编译"
  echo "  如需强制重建：cd ~/dzrobot && colcon build --base-paths src --packages-select $NEEDED"
  echo "==================== 结束 ===================="
  exit 0
fi

hr; echo "## 2. 编译缺失的包：$MISSING"
cd "$HOME/dzrobot" || { echo "  [FAIL] 进不去 ~/dzrobot"; exit 1; }
SELECT=""
for p in $MISSING; do SELECT="$SELECT $p"; done
# 清掉这几个包的 build/install：强制重新安装。
# 原因：如果上一次是用 `--symlink-install` 装的，可执行文件会是指向 build/ 的**软链接**，
# 一旦有人清理 build 目录，装好的东西就全失效了 —— 赛场上这种脆弱点不能留。
for p in $MISSING; do rm -rf "install/$p" "build/$p"; done
echo "  工作区: $(pwd)"
echo "  命令:   colcon build --base-paths src --packages-select$SELECT   （普通安装，落真实文件）"
echo "  开始:   $(date '+%T')"
# shellcheck disable=SC2086
colcon build --base-paths src --packages-select $SELECT > /tmp/ag_official_build.log 2>&1
RC=$?
echo "  结束:   $(date '+%T')   退出码=$RC"
grep -E "^Summary|^Finished|^Failed" /tmp/ag_official_build.log | tail -8 | sed 's/^/    /'
if [ "$RC" -ne 0 ]; then
  echo "  [FAIL] 编译失败，错误前 15 条："
  grep -nE "error:|CMake Error|fatal error|Duplicate package" /tmp/ag_official_build.log \
    | head -15 | sed 's/^/    /'
  echo "  完整日志在车端 /tmp/ag_official_build.log"
  echo "==================== 结束 ===================="
  exit 1
fi
echo "  -- 复核可执行文件是真实文件（不是软链）--"
for p in $MISSING; do
  PREFIX="$(ros2 pkg prefix "$p" 2>/dev/null)"
  for exe in $(ls "$PREFIX/lib/$p" 2>/dev/null | head -2); do
    f="$PREFIX/lib/$p/$exe"
    [ -L "$f" ] && echo "    [软链!] $p/$exe -> $(readlink "$f")" \
                || echo "    [真实]  $p/$exe ($(stat -c%s "$f" 2>/dev/null) 字节)"
  done
done
echo

hr; echo "## 3. 复核"
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
OK=1
for p in $NEEDED; do
  n=$(ros2 pkg list 2>/dev/null | grep -cx "$p")
  echo "  $([ "$n" -ge 1 ] && echo '[OK]' || echo '[缺]') $p"
  [ "$n" -lt 1 ] && OK=0
done
echo
echo "  -- 可执行文件 --"
for p in det_node dzactuator dzrknn_dataprocess; do
  printf "    %-22s %s\n" "$p" "$(ros2 pkg executables "$p" 2>/dev/null | tr '\n' ' ')"
done
echo
if [ "$OK" -eq 1 ]; then
  echo "  官方栈已就绪：可以跑视觉链（tools/car_camera_check.sh）与发车（tools/car_move_dryrun.sh）"
else
  echo "  仍有缺失 —— 看上面的 compile 日志/错误；可能需要厂家补源码"
fi
echo "==================== 结束 ===================="
exit "$([ "$OK" -eq 1 ] && echo 0 || echo 1)"
