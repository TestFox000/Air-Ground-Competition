#!/bin/bash
# t4 远端探针语法自检（只读）
for f in /tmp/t4_car_consumer.py /tmp/t4_car_sender.py /tmp/t4_qos_info.py \
         /tmp/t4_qos_probe.py /tmp/t4_mcast_count.py; do
  if python3 -m py_compile "$f" 2>/tmp/pyc.err; then
    echo "OK   $f"
  else
    echo "FAIL $f"; cat /tmp/pyc.err
  fi
done
echo "--- 关键探针 md5 ---"
md5sum /tmp/t4_*.py /tmp/t4_*.sh 2>/dev/null
