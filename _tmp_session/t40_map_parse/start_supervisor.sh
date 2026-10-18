#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/duzhong/dzlink_tools/dzlink.env
cd /home/duzhong/dzlink_tools
mkdir -p logs
setsid nohup python3 dzlink_supervisor.py run \
  --bridge lower=K7 --bridge upper=K5 \
  --target D5,D6 \
  --mines A2,F4,H4,G5,E8,B10,D11,I11 \
  --obstacles B1,D1,F3,I6,H7,A8,C9,E9 \
  --log-file /home/duzhong/dzlink_tools/logs/t40_20261017.jsonl \
  --status-period 60 \
  > /home/duzhong/dzlink_tools/logs/t40.stdout 2>&1 < /dev/null &
sleep 5
echo "--- procs ---"
pgrep -af "python3 dzlink_supervisor.py" || true
echo "--- stdout ---"
tail -n 30 /home/duzhong/dzlink_tools/logs/t40.stdout 2>/dev/null || true
