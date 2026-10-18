#!/bin/bash
export PATH=/usr/local/bin:$PATH
echo "== dzuav help =="
dzuav --help 2>&1 | head -40
echo "== dzuav image help =="
dzuav image --help 2>&1 | head -30
echo "== procs =="
pgrep -af dzuav | head
echo "== video0 =="
ls -l /dev/video0
echo "== display =="
ls /tmp/.X11-unix/ 2>/dev/null
echo "== models dir =="
ls -l ~/dzuav/models/ | head -20
echo "== engines =="
ls -l ~/dzuav/engines/ 2>/dev/null | head -20
