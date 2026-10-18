#!/bin/bash
BIN=~/dzuav/build/dzuav
IMG=~/dzuav_deploy/air_ground/l3_test.jpg
mkdir -p /tmp/verify_old /tmp/verify_new
echo "=== OLD engines ==="
t0=$(date +%s%N)
"$BIN" "$IMG" ~/dzuav/engines/hbb_detector_fp16.engine ~/dzuav/engines/cell_classifier_fp16.engine 50555 /tmp/verify_old 2>&1 | tail -25
rc=$?
t1=$(date +%s%N); echo "exit=$rc elapsed_ms=$(( (t1-t0)/1000000 ))"
echo "=== NEW engines (d13 + v4d) ==="
t0=$(date +%s%N)
"$BIN" "$IMG" ~/dzuav/engines/hbb_detector_d13_fp16.engine ~/dzuav/engines/cell_classifier_v4d_fp16.engine 50556 /tmp/verify_new 2>&1 | tail -25
rc=$?
t1=$(date +%s%N); echo "exit=$rc elapsed_ms=$(( (t1-t0)/1000000 ))"
echo "=== OUTPUTS ==="; ls -la /tmp/verify_old /tmp/verify_new
