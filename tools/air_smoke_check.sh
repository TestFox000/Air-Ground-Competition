#!/usr/bin/env bash
# 机载端（无人机）开机自检 —— **只读、零动作**，不启动任何节点、不动任何硬件。
#
# 为什么要有这个脚本：
# 机载端是"零实测"的一块（2026-09-28 状态）。第一次通电时，"没有情报"这一个现象
# 背后可能有五六个原因：ROS2 没起 / 话题名不对 / 相机没出图 / 模型没加载 /
# 解码参数错 / 单应没标。车端就吃过这个亏 —— usb_cam 崩了之后整条链是**静默的**，
# 现场表现永远只是"什么都检测不到"，真因在上游三跳之外。
# 所以：**动手联调之前，先让这个脚本把"环境层"的问题一次性排掉。**
#
# 用法（在机载电脑上本地跑）：
#   bash tools/air_smoke_check.sh
#   INPUT_TOPIC=/camera/image_raw bash tools/air_smoke_check.sh    # 相机话题名不同就覆盖
#   MODEL=/path/to/light_det.rknn   bash tools/air_smoke_check.sh  # 顺便验模型文件
#   AG_WS=/home/xxx/ag_ws           bash tools/air_smoke_check.sh  # 工作空间不是默认值
#
# 退出码：0 = 环境层健康；1 = 有问题（看输出定位）
#
# 通过之后干什么：见 docs/机载端联调.md 步骤 2（DummyBackend 验链路）

set -o pipefail

INPUT_TOPIC="${INPUT_TOPIC:-/image_raw}"
MODEL="${MODEL:-}"
AG_WS="${AG_WS:-/tmp/ag_ws}"
WAIT_S="${WAIT_S:-5}"

hr() { echo "----------------------------------------------------"; }
OK=1

echo "==================== 机载端开机自检（只读，零动作） ===================="
date
echo "  期望相机话题: $INPUT_TOPIC"
[ -n "$MODEL" ] && echo "  附带检查模型: $MODEL"
echo

# ---------------------------------------------------------------- 1. ROS2 环境
hr; echo "## 1. ROS2 环境"
if [ -f /opt/ros/humble/setup.bash ]; then
  echo "  [OK] /opt/ros/humble 存在"
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "  [FAIL] 找不到 /opt/ros/humble —— ROS2 Humble 没装，或装在别处"
  echo "         -> 先确认：ls /opt/ros/"
  OK=0
fi

if [ -f "$AG_WS/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$AG_WS/install/setup.bash" 2>/dev/null
  echo "  [OK] 已 source $AG_WS"
else
  echo "  [注意] $AG_WS/install/setup.bash 不存在 —— 我们的包可能还没编译/没传过来"
fi

if command -v ros2 >/dev/null 2>&1; then
  echo "  [OK] ros2 命令可用"
else
  echo "  [FAIL] ros2 命令不可用（环境没 source 全？）"
  OK=0
fi

if python3 -c "import rclpy" 2>/dev/null; then
  echo "  [OK] rclpy 可导入"
else
  echo "  [FAIL] rclpy 导不进来 —— 后面的检查都没有意义了"
  OK=0
fi
echo

# ---------------------------------------------------------------- 2. 项目包
hr; echo "## 2. 项目包能否导入"
for m in ag_core ag_perception ag_bridge; do
  if python3 -c "import $m" 2>/dev/null; then
    echo "  [OK] $m"
  else
    echo "  [FAIL] $m 导不进来 —— 源码没传全 / 没编译 / 没 source $AG_WS"
    OK=0
  fi
done
echo

# ---------------------------------------------------------------- 3. 当前 ROS 图
hr; echo "## 3. 当前在跑的节点 / 话题"
timeout 12 ros2 node list 2>/dev/null | sed 's/^/  /' || echo "  （取不到 —— ros2 本身有问题？）"
echo "  -- 含 image 的话题 --"
timeout 12 ros2 topic list 2>/dev/null | grep -i image | sed 's/^/  /' || echo "  （一个都没有）"
echo

# ---------------------------------------------------------------- 4. 相机出图
hr; echo "## 4. 相机是否出图（数 ${WAIT_S} 秒）"
if timeout $((WAIT_S + 5)) ros2 topic list 2>/dev/null | grep -qx "$INPUT_TOPIC"; then
  HZ=$(timeout $((WAIT_S + 10)) ros2 topic hz "$INPUT_TOPIC" 2>/dev/null | grep -m1 "average rate")
  if [ -n "$HZ" ]; then
    echo "  [OK] $HZ"
  else
    echo "  [FAIL] 话题在，但 ${WAIT_S} 秒内一帧都没有 —— 相机没出图 / 崩了"
    echo "         -> 车端踩过的坑：usb_cam 会以 SIGABRT 静默崩掉，整条链没有任何报错"
    OK=0
  fi
else
  echo "  [注意] 当前 ROS 图里没有 $INPUT_TOPIC 这个话题"
  echo "         -> 要么相机没起，要么话题名不是它（看上面 §3 里含 image 的话题）"
  echo "            确认后：INPUT_TOPIC=/实际话题名 bash tools/air_smoke_check.sh"
  OK=0
fi
echo

# ---------------------------------------------------------------- 5. NPU 运行时
hr; echo "## 5. NPU 运行时（RKNN）"
if python3 -c "from rknnlite.api import RKNNLite" 2>/dev/null; then
  echo "  [OK] rknn-toolkit-lite2 可用"
else
  echo "  [注意] rknnlite 导不进来"
  echo "         若机载电脑是 RK 板且要用 .rknn → 需要装 rknn-toolkit-lite2（官方 SDK 提供）"
  echo "         若不是 RK 平台、或改用 onnx 后端 → 可忽略这一条"
fi
echo

# ---------------------------------------------------------------- 6. 模型文件（可选）
hr; echo "## 6. 模型文件"
if [ -n "$MODEL" ]; then
  if [ -f "$MODEL" ]; then
    SZ=$(stat -c%s "$MODEL" 2>/dev/null || echo "?")
    echo "  [OK] 存在，${SZ} 字节"
    echo "       基线 light_det.rknn 应为 8487541 字节"
    echo "       md5 应为 49F3700590A907994E7EE20E4D30CC13"
    if command -v md5sum >/dev/null 2>&1; then
      echo "       实测 md5: $(md5sum "$MODEL" | awk '{print $1}')"
    fi
  else
    echo "  [FAIL] 文件不存在：$MODEL"
    OK=0
  fi
else
  echo "  （没传 MODEL=，跳过。要验：MODEL=/path/to/x.rknn bash tools/air_smoke_check.sh）"
fi
echo

# ---------------------------------------------------------------- 结论
hr; echo "## 7. 结论"
if [ "$OK" -eq 1 ]; then
  echo "  [OK] 环境层健康 → 下一步进入《docs/机载端联调.md》步骤 2（DummyBackend 验链路）"
else
  echo "  [FAIL] 上面有 FAIL 项 —— 先把环境层修掉，别急着跑联调"
  echo "         联调顺序与诊断思路见 docs/机载端联调.md §2 / §3"
fi
echo "==================== 结束（本脚本没有启动任何节点） ===================="
exit "$([ "$OK" -eq 1 ] && echo 0 || echo 1)"
