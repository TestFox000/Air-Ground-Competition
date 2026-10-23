#!/bin/bash
echo "=== dzros.env ==="
cat -A /home/duzhong/dzrobot/dzros.env 2>/dev/null | head -20
echo "=== plain ==="
cat /home/duzhong/dzrobot/dzros.env 2>/dev/null
echo "=== how main.py checks ==="
grep -n 'DZ_ROS_DOMAIN' /home/duzhong/dzrobot/src/dzbrain/dzbrain/main.py | head -10
