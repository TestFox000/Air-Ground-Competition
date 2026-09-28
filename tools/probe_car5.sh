#!/usr/bin/env bash
# 车端第五轮（只读）：修正上一轮路径错误 + 摸清数据集家底。
# 目标：
#   1) path_executor 的打击停靠/到达/速度常量（决定路线怎么排）
#   2) camera_dataset 里到底有多少可用素材、有没有标注
#   3) 训练/导出脚本在不在车上（判断"模型是谁训的、怎么导的"）

hr() { echo "----------------------------------------------------"; }

echo "==================== 车端第五轮：执行器常量 + 数据集 ===================="
date
echo

hr; echo "## 1. path_executor 常量（修正路径：src/car_localization/car_localization/）"
PE="$HOME/dzrobot/src/car_localization/car_localization/path_executor.py"
if [ -f "$PE" ]; then
  echo "文件: $PE ($(wc -l < "$PE") 行)"
  echo "-- 模块级常量 --"
  grep -nE "^[A-Z][A-Z0-9_]* *=" "$PE" | head -40
  echo "-- '*'(停靠打击) 相关 --"
  grep -nE "SHOOT|shoot_count|shoot_wait|SHOOT_COUNT_TARGET|shots" "$PE" | head -25
else
  echo "未找到，列出实际结构："
  find "$HOME/dzrobot/src/car_localization" -name "*.py" 2>/dev/null
fi
echo

hr; echo "## 2. camera_dataset 家底"
CD="$HOME/dzrobot/camera_dataset"
echo "总大小: $(du -sh "$CD" 2>/dev/null)"
echo "文件数: $(find "$CD" -type f 2>/dev/null | wc -l)"
echo "-- 按扩展名统计 --"
find "$CD" -type f 2>/dev/null | sed 's/.*\.//' | sort | uniq -c | sort -rn | head
echo "-- 目录结构（2 层）--"
find "$CD" -maxdepth 2 -type d 2>/dev/null | head -20
echo "-- 有没有标注（labels / txt / json / xml）--"
find "$CD" -maxdepth 3 \( -name "*.txt" -o -name "*.json" -o -name "*.xml" \) 2>/dev/null | head -10
echo "-- 图片分辨率抽样 --"
for f in $(find "$CD" -name "*.jpg" 2>/dev/null | head -3); do
  python3 -c "
from PIL import Image
try:
    im = Image.open('$f'); print('$f', im.size)
except Exception as e:
    print('$f', 'PIL 不可用:', e)
" 2>/dev/null || echo "  $f (PIL 不可用)"
done
echo

hr; echo "## 3. 全盘找训练素材/标签/脚本（限制深度，排除 build/install）"
find "$HOME" -maxdepth 6 \( -name "*.pt" -o -name "*.onnx" -o -name "*.yaml" \) \
  -not -path "*/.cache/*" -not -path "*/build/*" -not -path "*/install/*" \
  -not -path "*/navigation2/*" -not -path "*/robot_localization*" \
  -not -path "*/rviz/*" -not -path "*/slam_gmapping*" -not -path "*/rplidar*" 2>/dev/null | head -25
echo "-- 可疑的数据集目录 --"
find "$HOME" -maxdepth 4 -type d \( -iname "*data*" -o -iname "*label*" -o -iname "*dataset*" -o -iname "*train*" \) \
  -not -path "*/.cache/*" -not -path "*/build/*" -not -path "*/install/*" 2>/dev/null | head -20
echo

hr; echo "## 4. rkpt.cc 解码逻辑（确认类别数与关键点个数）"
RC="$HOME/dzrobot/src/rknn_pt/rknn_pt/src/det/rkpt.cc"
POST="$HOME/dzrobot/src/rknn_pt/rknn_pt/src/det/postprocess.cc"
for f in "$RC" "$POST"; do
  [ -f "$f" ] || continue
  echo "===== $(basename "$f") 关键行 ====="
  grep -nE "NUM_CLASS|num_class|CLASS|KPT|kpt|8400|dfl|DFL|conf_thres|score|scale|stride|anchor" "$f" | head -30
  echo
done
echo "-- 头文件里的宏 --"
grep -rnE "#define +[A-Z_]+" "$HOME/dzrobot/src/rknn_pt/rknn_pt/include/det/" 2>/dev/null | head -20
echo

hr; echo "## 5. 车上有没有训练/导出脚本（判断 light_det 是谁生成的）"
find "$HOME" -maxdepth 5 \( -name "export*.py" -o -name "*rknn*.py" -o -name "train*.py" -o -name "convert*.py" \) \
  -not -path "*/.cache/*" -not -path "*/build/*" -not -path "*/install/*" 2>/dev/null | head -15
echo "-- test_rknn.py 头部（可能就是导出/验证脚本）--"
head -30 "$HOME/dzrobot/test_rknn.py" 2>/dev/null
echo

hr; echo "## 6. offcenter_table.txt 是否已标定"
OT="$HOME/dzrobot/src/dzrknn_dataprocess/offcenter_table.txt"
echo "行数: $(wc -l < "$OT" 2>/dev/null), 去重值: $(sort -u "$OT" 2>/dev/null | tr '\n' ' ')"
echo

echo "==================== 结束 ===================="
