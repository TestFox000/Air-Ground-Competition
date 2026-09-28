#!/usr/bin/env bash
# 车端第四轮（只读）：把「模型类别顺序 / 数据集标签 / path_executor 关键常量」钉死。
# 这轮要回答的是决定胜负的几个数：
#   1) light_det*.rknn 里到底谁蓝谁红（搞反 = 误击友方 -15）
#   2) path_executor 打击停靠点的射满次数、到达容差
#   3) 现场标定表 offcenter_table.txt 是否存在

hr() { echo "----------------------------------------------------"; }
say() { echo "[$1] $2"; }

echo "==================== 车端第四轮：模型与执行器常量 ===================="
date
echo

hr; echo "## 1. 模型文件（大小 + md5，确认哪几个其实是同一个）"
MODEL_DIR="$HOME/dzrobot/src/rknn_pt/model"
ls -la "$MODEL_DIR" 2>/dev/null
echo "-- md5 --"
for f in "$MODEL_DIR"/*.rknn; do
  [ -e "$f" ] || continue
  printf "%s  %s\n" "$(md5sum "$f" | cut -c1-8)" "$(basename "$f")"
done
echo

hr; echo "## 2. 模型内嵌的字符串（找类别名 / 训练工程名）"
for f in "$MODEL_DIR"/*.rknn; do
  [ -e "$f" ] || continue
  echo "===== $(basename "$f") ====="
  strings -n 4 "$f" 2>/dev/null \
    | grep -iE "blue|red|target|mine|glow|enemy|friend|class|label|names|yolo|pt$|best" \
    | sort -u | head -25
  echo
done

hr; echo "## 3. 数据集与标签（找类别名定义）"
find "$HOME" -maxdepth 5 \( -name "*.yaml" -o -name "*.yml" \) \
  -not -path "*/.cache/*" -not -path "*/build/*" -not -path "*/install/*" \
  -not -path "*/navigation2/*" -not -path "*/robot_localization*" 2>/dev/null | head -20
echo "-- 疑似 data.yaml 内容 --"
for y in $(find "$HOME" -maxdepth 5 -name "*.yaml" -path "*data*" 2>/dev/null | head -5); do
  echo "--- $y ---"; cat "$y" 2>/dev/null | head -30
done
echo "-- camera_dataset 目录 --"
ls "$HOME/dzrobot/camera_dataset" 2>/dev/null | head -10
du -sh "$HOME/dzrobot/camera_dataset" 2>/dev/null
echo

hr; echo "## 4. path_executor 关键常量"
PE="$HOME/dzrobot/src/car_localization/car_localization/car_localization/path_executor.py"
grep -nE "^[A-Z_]+ *=|SHOOT_COUNT|ARRIVAL|TOLERANCE|SPEED|approach|MAX_" "$PE" 2>/dev/null | head -40
echo

hr; echo "## 5. 现场标定表 / 参数文件"
ls -la "$HOME/dzrobot/src/dzrknn_dataprocess/" 2>/dev/null
echo "-- offcenter_table.txt 头几行 --"
head -5 "$HOME/dzrobot/src/dzrknn_dataprocess/offcenter_table.txt" 2>/dev/null || echo "  不存在"
echo

hr; echo "## 6. 官方启动方式（launch 里带了什么参数）"
for L in bringup_all dzrknn_dataprocess det_node judgment camera; do
  f=$(find "$HOME/dzrobot/src/dzactuator/launch" -name "$L.launch.py" 2>/dev/null | head -1)
  [ -n "$f" ] || continue
  echo "===== $L ====="
  grep -nE "package=|executable=|name=|[a-z_]+ *=|delay|TimerAction" "$f" 2>/dev/null | head -25
  echo
done

hr; echo "## 7. 车端是否已有本项目（避免重复部署）"
ls -d "$HOME/ros2_ws" "$HOME/ag_ws" 2>/dev/null || echo "  无独立工作空间"
ls "$HOME/dzrobot/src" 2>/dev/null

hr; echo "## 8. 当前 ROS 图（若官方栈已在跑，这里能看到真实话题与类型）"
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/dzrobot/install/setup.bash" 2>/dev/null
echo "-- nodes --";  timeout 15 ros2 node list 2>/dev/null || true
echo "-- topics --"; timeout 15 ros2 topic list 2>/dev/null || true
echo "-- 关键话题类型 --"
for t in /nav/waypoint /HpAndHitmsg /LaserShot_Command /shoot_count /offset_center /image_raw /odom/localization; do
  printf "%-24s " "$t"; timeout 8 ros2 topic type "$t" 2>/dev/null || echo "(未发布/无节点)"
done

echo "==================== 结束 ===================="
