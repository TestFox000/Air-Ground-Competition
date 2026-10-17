#!/bin/bash
# 车端 ADB/USB 取证脚本（只读，不做任何限制/禁用）
echo "==================== 1. ADB 授权密钥（谁连过 adb） ===================="
for f in /data/misc/adb/adb_keys /root/.android/adbkey.pub /home/duzhong/.android/adbkey.pub /data/misc/adb/adb_keys.tmp; do
  if [ -f "$f" ]; then
    echo "--- $f ---"
    cat "$f"
  fi
done
if ! ls /data/misc/adb/ /root/.android/ /home/duzhong/.android/ >/dev/null 2>&1; then
  echo "(无 adb 授权目录)"
fi

echo
echo "==================== 2. adbd 状态 / 属性 ===================="
ps -ef | grep -i adbd | grep -v grep
getprop ro.adb.secure 2>/dev/null
getprop service.adb.tcp.port 2>/dev/null
getprop persist.adb.tcp.port 2>/dev/null

echo
echo "==================== 3. adbd 日志痕迹 ===================="
ls -la /data/misc/adb/ 2>/dev/null
dmesg 2>/dev/null | grep -i adb | tail -30
journalctl -u adbd --no-pager 2>/dev/null | tail -30
logcat -d 2>/dev/null | grep -i adb | tail -30

echo
echo "==================== 4. USB 当前功能 / 历史 ===================="
cat /tmp/.usbdevice 2>/dev/null
echo "--- usbdevice.log ---"
tail -60 /tmp/usbdevice.log 2>/dev/null

echo
echo "==================== 5. shell 历史（谁操作过） ===================="
echo "--- duzhong bash_history 尾部 60 行 ---"
tail -60 /home/duzhong/.bash_history 2>/dev/null
echo "--- root bash_history 尾部 40 行 ---"
tail -40 /root/.bash_history 2>/dev/null

echo
echo "==================== 6. 登录 / 会话记录 ===================="
last -25 2>/dev/null | head -30
echo "--- wtmp/btmp 存在性 ---"
ls -la /var/log/wtmp /var/log/btmp 2>/dev/null

echo
echo "==================== 7. 项目目录最近被读/改的文件（近48h） ===================="
echo "--- /home/duzhong/dzrobot 近48h mtime 变化 top 40 ---"
find /home/duzhong/dzrobot /home/duzhong/dzrobot_v2 -type f -mtime -2 2>/dev/null \
  -printf '%TY-%Tm-%Td %TH:%TM:%TS  %s  %p\n' | sort -r | head -40

echo
echo "==================== 8. 是否有整包拷贝痕迹（tar/zip/cp -r/scp/adb pull） ===================="
grep -nE "adb pull|adb push|scp |tar -c|tar czf|zip -r|cp -r|rsync" \
  /home/duzhong/.bash_history /root/.bash_history 2>/dev/null | tail -40

echo
echo "==================== 9. 临时目录可疑文件 ===================="
ls -la /tmp/ 2>/dev/null | grep -iE "dz|robot|backup|tar|zip|\.sh" | head -30
ls -la /home/duzhong/ 2>/dev/null | grep -iE "backup|tar|zip|\.zip|\.tar" | head -20

echo
echo "==================== DONE ===================="
