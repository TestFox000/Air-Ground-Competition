#!/bin/bash
BIN=~/dzuav/build/dzuav
IMG=~/dzuav_deploy/air_ground/l3_field_88cells.jpg
mkdir -p /tmp/v88_old /tmp/v88_new
echo "=== OLD on 88cells ==="
t0=$(date +%s%N)
"$BIN" "$IMG" ~/dzuav/engines/hbb_detector_fp16.engine ~/dzuav/engines/cell_classifier_fp16.engine 50557 /tmp/v88_old 2>&1 | tail -12
t1=$(date +%s%N); echo "elapsed_ms=$(( (t1-t0)/1000000 ))"
echo "=== NEW on 88cells ==="
t0=$(date +%s%N)
"$BIN" "$IMG" ~/dzuav/engines/hbb_detector_d13_fp16.engine ~/dzuav/engines/cell_classifier_v4d_fp16.engine 50558 /tmp/v88_new 2>&1 | tail -12
t1=$(date +%s%N); echo "elapsed_ms=$(( (t1-t0)/1000000 ))"
echo "=== PY LIBS ==="
python3 -c 'import cv2, numpy; print("cv2", cv2.__version__, "numpy", numpy.__version__)' 2>&1 | tail -1
python3 -c 'import pycuda; print("pycuda ok")' 2>&1 | tail -1
python3 -c 'import tensorrt as trt; print("trt", trt.__version__)' 2>&1 | tail -1
