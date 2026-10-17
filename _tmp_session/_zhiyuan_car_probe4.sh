#!/bin/bash
echo "===BACKUPS_TODAY==="
ls -lt --time-style=+%m-%d_%H:%M /home/duzhong/_backups/ 2>/dev/null | head -12
echo "===PKG_META==="
ls -l --time-style=+%m-%d_%H:%M /home/duzhong/dzrobot/src/dzrknn_dataprocess/ 2>/dev/null
echo "===LOCAL_VS_CAR_H==="
md5sum /home/duzhong/dzrobot/src/dzrknn_dataprocess/include/dzrknn_dataprocess.h
echo "===H_KEY_LINES==="
grep -n -E 'declare_parameter|scan_pitch_levels' /home/duzhong/dzrobot/src/dzrknn_dataprocess/src/dzrknn_dataprocess.cpp | head -20
echo "===BUILD_DIRS==="
ls -lt --time-style=+%m-%d_%H:%M /home/duzhong/dzrobot/log/ 2>/dev/null | head -6
echo "===DATE==="
date
