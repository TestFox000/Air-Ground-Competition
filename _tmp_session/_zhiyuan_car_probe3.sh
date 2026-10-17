#!/bin/bash
echo "===WHO==="
who -a 2>/dev/null | head -10
echo "===W==="
w 2>/dev/null | head -10
echo "===CPP_HEAD==="
head -25 /home/duzhong/dzrobot/src/dzrknn_dataprocess/src/dzrknn_dataprocess.cpp
echo "===CPP_LINES==="
wc -l /home/duzhong/dzrobot/src/dzrknn_dataprocess/src/dzrknn_dataprocess.cpp
echo "===H_PITCH==="
grep -n -E 'pitch|scan_pitch' /home/duzhong/dzrobot/src/dzrknn_dataprocess/include/dzrknn_dataprocess.h | head -10
echo "===BIN_SYMBOL==="
strings /home/duzhong/dzrobot/install/dzrknn_dataprocess/lib/dzrknn_dataprocess/dzactuator_node 2>/dev/null | grep -c scan_pitch_offset_deg
ls -l --time-style=+%m-%d_%H:%M /home/duzhong/dzrobot/install/dzrknn_dataprocess/lib/dzrknn_dataprocess/dzactuator_node
echo "===PKG_FILES==="
ls -l --time-style=+%m-%d_%H:%M /home/duzhong/dzrobot/src/dzrknn_dataprocess/src/ /home/duzhong/dzrobot/src/dzrknn_dataprocess/include/
