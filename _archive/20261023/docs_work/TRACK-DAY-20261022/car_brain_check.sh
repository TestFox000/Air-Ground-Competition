#!/bin/bash
LOG=/home/duzhong/r3_field_20261021/logs
echo "=== PROC ==="
pgrep -af 'dzbrain.main|serial_reader_node' | head
echo "=== BRAIN LOG SIZE ==="
ls -l "$LOG/7_path_aim.log" 2>/dev/null
echo "=== TAIL ==="
tail -30 "$LOG/7_path_aim.log" 2>/dev/null
echo "=== HEAD ==="
head -8 "$LOG/7_path_aim.log" 2>/dev/null
