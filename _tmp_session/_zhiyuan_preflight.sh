#!/bin/bash
cd /home/duzhong/dzrobot
echo "=== BEFORE ==="
ps -ef | grep -E 'path_executor|dzbrain|dzactuator_node|det_node|usb_cam|serial_reader|dzjudgment' | grep -v grep | awk '{print $2, $10, $11, $12}'
echo "=== KILL STRAY OLD-CORE path_executor ==="
pkill -f path_executor_node 2>/dev/null
sleep 2
if pgrep -f path_executor_node >/dev/null; then echo STILL_ALIVE; else echo STRAY_GONE; fi
echo "=== MANIFEST EXISTS? ==="
ls -l .launch_manifest.sha256 2>&1
echo "=== MANIFEST CHECK ==="
sha256sum -c .launch_manifest.sha256 2>&1 | tail -10
