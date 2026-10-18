#!/bin/bash
# 检查 install 目录完整性（launch/share 是否被清掉）
set +u
echo "=== install/dzrknn_dataprocess 树 ==="
ls -la /home/duzhong/dzrobot/install/dzrknn_dataprocess 2>/dev/null
echo "=== share ==="
ls -la /home/duzhong/dzrobot/install/dzrknn_dataprocess/share/dzrknn_dataprocess 2>/dev/null
echo "=== share/launch ==="
ls -la /home/duzhong/dzrobot/install/dzrknn_dataprocess/share/dzrknn_dataprocess/launch 2>/dev/null
echo "=== lib ==="
ls -la /home/duzhong/dzrobot/install/dzrknn_dataprocess/lib/dzrknn_dataprocess 2>/dev/null
echo "=== build 时间戳 ==="
ls -la /home/duzhong/dzrobot/build/dzrknn_dataprocess 2>/dev/null | head -5
echo "=== 源码 launch 是否在 ==="
ls -la /home/duzhong/dzrobot/src/dzrknn_dataprocess_save/launch 2>/dev/null
echo "=== 是否有别的 colcon 在跑 ==="
ps -ef | grep -c '[c]olcon'
