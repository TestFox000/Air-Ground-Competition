#!/bin/bash
echo ---V4D_TAIL; tail -25 ~/v4d_engine.log 2>/dev/null
echo ---MAIN_IMAGE; grep -n "runImage" ~/dzuav/src/main.cpp | head -5
sed -n '/runImage/,/^}/p' ~/dzuav/src/main.cpp | head -60
echo ---TOOLS; ls ~/dzuav_deploy/air_ground/tools 2>/dev/null | head -20
echo ---PERCEPTION; ls ~/dzlink_tools/perception 2>/dev/null | head -20
echo ---WIRING; grep -rnE "hbb_detector|cell_classifier|\.engine" ~/dzuav_deploy/air_ground/tools ~/dzlink_tools 2>/dev/null | head -20
