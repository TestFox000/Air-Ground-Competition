#!/bin/bash
echo "===CPP_PITCH==="
grep -n scan_pitch_offset_deg /home/duzhong/dzrobot/src/dzrknn_dataprocess/src/dzrknn_dataprocess.cpp
ls -l --time-style=+%m-%d_%H:%M /home/duzhong/dzrobot/src/dzrknn_dataprocess/src/dzrknn_dataprocess.cpp /home/duzhong/dzrobot/src/dzrknn_dataprocess/include/dzrknn_dataprocess.h
echo "===START_ALL_STEPS==="
grep -n -E 'ros2 (run|launch)|dzbrain|path_executor' /home/duzhong/dzrobot/start_all.sh | head -40
echo "===RUNNING_NODES==="
ps -ef | grep -E 'ros2|node' | grep -v grep | awk '{print $2, $8, $9, $10, $11}' | head -25
