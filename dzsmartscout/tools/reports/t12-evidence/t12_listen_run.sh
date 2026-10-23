#!/bin/bash
bash /tmp/t4_rosenv.sh bash -c 'cd /home/duzhong/dzlink_tools && python3 dzlink_supervisor.py listen --duration 14 --log-file /tmp/t12_listen.jsonl' > /tmp/t12_listen.out 2>&1
echo "LISTEN_EXIT=$?" >> /tmp/t12_listen.out
