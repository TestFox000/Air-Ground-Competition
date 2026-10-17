#!/bin/bash
B=/home/duzhong/_backups/dzbrain_deploy_20261015_212940
echo "===BACKUP_TREE==="
find $B -name 'dzrknn_dataprocess.cpp' -o -name 'dzrknn_dataprocess.h' 2>/dev/null | head
echo "===SIZES==="
find $B -name 'dzrknn_dataprocess.*' -exec ls -l --time-style=+%m-%d_%H:%M {} \; 2>/dev/null
echo "===MD5_BACKUP==="
find $B -name 'dzrknn_dataprocess.*' -exec md5sum {} \; 2>/dev/null
echo "===PITCH_IN_BACKUP_CPP==="
find $B -name 'dzrknn_dataprocess.cpp' -exec grep -c scan_pitch_offset_deg {} \; 2>/dev/null
echo "===t52_BACKUPS==="
ls -l --time-style=+%m-%d_%H:%M /home/duzhong/_backups/t52_20261015_202313/ /home/duzhong/_backups/t52_20261015_202159/ 2>/dev/null
