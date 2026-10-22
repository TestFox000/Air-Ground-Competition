#!/bin/bash
# 抓 ttyUSB0 原始数据 5 秒，判断协议类型（只读，不改任何配置）
echo '=== [1] 串口设备与成员 ==='
ls -l /dev/ttyUSB0
echo "  duzhong in dialout: $(id -nG | grep -o dialout)"

echo
echo '=== [2] 尝试以多种波特率轮询抓 5 秒原始字节 ==='
for baud in 57600 115200 9600; do
  echo "--- 波特率 $baud ---"
  timeout 3 cat /dev/ttyUSB0 2>/dev/null > /tmp/raw_$baud.bin || true
  # 用 stty 尝试设置波特率
  if command -v stty >/dev/null 2>&1; then
    stty -F /dev/ttyUSB0 $baud raw -echo 2>/dev/null
    sz=$(timeout 3 cat /dev/ttyUSB0 2>/dev/null | wc -c)
    echo "  收到 $sz 字节"
  fi
done

echo
echo '=== [3] 用 115200 抓 5 秒并 hexdump 前 64 字节 ==='
if command -v xxd >/dev/null 2>&1; then
  stty -F /dev/ttyUSB0 115200 raw -echo 2>/dev/null
  timeout 5 cat /dev/ttyUSB0 2>/dev/null | head -c 64 | xxd | head -8
  echo '(空=无数据)'
else
  echo '  无 xxd，尝试 od'
  stty -F /dev/ttyUSB0 115200 raw -echo 2>/dev/null
  timeout 5 cat /dev/ttyUSB0 2>/dev/null | head -c 64 | od -Ax -tx1z | head -8
  echo '(空=无数据)'
fi