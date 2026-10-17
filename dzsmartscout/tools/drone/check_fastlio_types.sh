#!/bin/bash
# 判断无人机自身 SLAM 栈（FAST-LIO 系）期望 /livox/lidar 是什么消息类型
echo "=== 1) 相关 yaml 配置文件 ==="
find /home/duzhong/dzpx4/src -maxdepth 5 -name "*.yaml" 2>/dev/null | grep -iE "fast|lio|mid360" | head -20

echo "=== 2) 所有含 lid_topic / lidar_topic / lidar_type 的配置行 ==="
grep -rn "lid_topic\|lidar_topic\|lidar_type" /home/duzhong/dzpx4/src --include=*.yaml 2>/dev/null | head -30

echo "=== 3) 源码中引用了 livox 消息包的文件 ==="
grep -rln "livox_ros_driver2" /home/duzhong/dzpx4/src 2>/dev/null | head -20

echo "=== 4) 类型分支细节（CustomMsg vs PointCloud2） ==="
grep -rn "CustomMsg\|PointCloud2::ConstSharedPtr\|sensor_msgs::msg::PointCloud2" \
  /home/duzhong/dzpx4/src/FASTLIO2_ROS2 /home/duzhong/dzpx4/src/FAST_LIO_LOCALIZATION_ROS2 2>/dev/null | head -25

echo "=== 5) 构建依赖是否声明 livox 消息包 ==="
grep -rn "livox" /home/duzhong/dzpx4/src/FASTLIO2_ROS2/fastlio2/CMakeLists.txt \
                /home/duzhong/dzpx4/src/FASTLIO2_ROS2/fastlio2/package.xml 2>/dev/null | head -10
grep -rn "livox" /home/duzhong/dzpx4/src/FAST_LIO_LOCALIZATION_ROS2/CMakeLists.txt \
                /home/duzhong/dzpx4/src/FAST_LIO_LOCALIZATION_ROS2/package.xml 2>/dev/null | head -10

echo "=== 6) 当前是否有 SLAM 节点在运行 ==="
pgrep -af "fastlio|fast_lio|laserMapping|lio_node" || echo "(无 SLAM 节点在跑)"
echo "=== DONE ==="
