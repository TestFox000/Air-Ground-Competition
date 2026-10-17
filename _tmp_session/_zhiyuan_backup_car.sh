#!/bin/bash
set -e
BK=/home/duzhong/_backups/zhiyuan_20261016_restore
mkdir -p $BK
cp /home/duzhong/dzrobot/src/dzrknn_dataprocess/src/dzrknn_dataprocess.cpp $BK/cpp.foreign_backup
cp /home/duzhong/dzrobot/src/dzrknn_dataprocess/include/dzrknn_dataprocess.h $BK/h.before_restore
echo "===BACKUP_DONE==="
ls -l --time-style=+%m-%d_%H:%M $BK
