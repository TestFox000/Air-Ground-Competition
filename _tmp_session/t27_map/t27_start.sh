#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/duzhong/dzlink_tools/dzlink.env
cd /home/duzhong/dzlink_tools
mkdir -p logs
pkill -f dzlink_supervisor.py 2>/dev/null
sleep 1
setsid nohup python3 dzlink_supervisor.py run \
  --bridge lower=K10 --bridge upper=K4 \
  --target D7,E7 \
  --mines B1,A2,I2,C3,E3,F5,H8,G9 \
  --obstacles D5,I6,B7,F8,C9,E10,G10,A11 \
  --log-file /home/duzhong/dzlink_tools/logs/t27_20261018.jsonl \
  --status-period 60 \
  > /home/duzhong/dzlink_tools/logs/t27.stdout 2>&1 < /dev/null &
sleep 5
echo "--- procs ---"
pgrep -af dzlink_supervisor.py | head -2
echo "--- stdout ---"
tail -n 8 /home/duzhong/dzlink_tools/logs/t27.stdout
