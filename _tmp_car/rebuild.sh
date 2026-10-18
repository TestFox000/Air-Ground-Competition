#!/bin/bash
# 干净重建 dzrknn_dataprocess（源码已改名为 src/dzrknn_dataprocess，旧 install 符号链接指向已删除的 _save 目录）
set +u
cd /home/duzhong/dzrobot || exit 1
source /opt/ros/humble/setup.bash
echo "== 清旧构建产物 =="
rm -rf build/dzrknn_dataprocess install/dzrknn_dataprocess
echo "== colcon build =="
colcon build --packages-select dzrknn_dataprocess \
  --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -8
echo "== install 校验 =="
ls -la install/dzrknn_dataprocess/share/dzrknn_dataprocess/launch/ 2>/dev/null | head -4
ls -la install/dzrknn_dataprocess/lib/dzrknn_dataprocess/ 2>/dev/null | head -3
