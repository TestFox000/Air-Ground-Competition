#!/bin/bash
echo ---DEPLOY; find ~/dzuav_deploy -maxdepth 4 -type d 2>/dev/null | head -25
echo ---DEPLOY_ONNX; find ~/dzuav_deploy -maxdepth 6 -type f -name "*.onnx" 2>/dev/null | head -10
echo ---HBB_H; sed -n '1,50p' ~/dzuav/src/HbbDetector.cpp
echo ---HBB_KEYS; grep -nE "engines?/|input|width|height|imgsz|classes|num_class|kNum|Engine|load" ~/dzuav/src/HbbDetector.cpp | head -40
echo ---CELL_KEYS; grep -nE "engine|input|width|height|class|out" ~/dzuav/src/CellClassifier.cpp | head -20
echo ---MAIN; grep -nE "argv|argc|engine|model" ~/dzuav/src/main.cpp | head -30
echo ---IMAGES; find /home/duzhong -maxdepth 4 -type f \( -iname "*.jpg" -o -iname "*.jpeg" -o -iname "*.png" \) 2>/dev/null | grep -vE "ros_logs|Pictures|Videos|Desktop|Music|\.cache|\.local" | head -20
