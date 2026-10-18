#!/bin/bash
echo ---STRINGS_ONNX
strings ~/dzuav/models/hbb_detector_d13_20261016.onnx | grep -aiE "^(mine|car|arrow|target_red|cell_label|target_blue|obstacle)$" | sort -u | head
strings ~/dzuav_deploy/air_ground/models/release/hbb_detector_7cls_distill_20261011.onnx | grep -aiE "^(mine|car|arrow|target_red|cell_label|target_blue|obstacle)$" | sort -u | head
echo ---FIND_YAML
find /home/duzhong -maxdepth 6 -type f \( -name "*.yaml" -o -name "*.yml" -o -name "*.txt" -o -name "*.json" \) 2>/dev/null | xargs -r grep -l "cell_label" 2>/dev/null | head -10
echo ---PY_VER; python3 -V; python3 -c 'import numpy; print(numpy.__version__)' 2>&1 | tail -1
