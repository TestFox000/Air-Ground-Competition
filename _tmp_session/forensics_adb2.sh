#!/bin/bash
# 车端 ADB/USB 取证补充（只读）
echo "==================== A. 车上现存打包文件（谁打包过、什么时候） ===================="
find /home/duzhong /tmp /media /mnt /run/media -maxdepth 3 \( \
  -name "*.zip" -o -name "*.tar" -o -name "*.tar.gz" -o -name "*.tgz" -o -name "*.7z" -o -name "*.rar" \) \
  -type f 2>/dev/null -printf '%TY-%Tm-%Td %TH:%TM  %10s  %p\n' | sort -r | head -40

echo
echo "==================== B. 是否有 U盘/移动存储挂载痕迹 ===================="
ls -la /media /mnt /run/media 2>/dev/null
grep -nE "sd[a-z]|usb|media|/mnt" /etc/fstab 2>/dev/null | grep -v '^#'
echo "--- dmesg 里的 usb-storage / 挂载事件 ---"
dmesg 2>/dev/null | grep -iE "usb-storage|scsi|sd[a-z]:|new SuperSpeed|new high-speed" | tail -30

echo
echo "==================== C. ADB 完整授权密钥（决定性：谁 adb 连过） ===================="
echo "--- /data/misc/adb/adb_keys ---"
cat /data/misc/adb/adb_keys 2>/dev/null || echo "(不存在)"
echo "--- /root/.android ---"
ls -la /root/.android 2>/dev/null && cat /root/.android/adbkey.pub 2>/dev/null
echo "--- /home/duzhong/.android ---"
ls -la /home/duzhong/.android 2>/dev/null && cat /home/duzhong/.android/adbkey.pub 2>/dev/null

echo
echo "==================== D. adbd 网络端口历史（有没有开过 5555 tcp） ===================="
getprop 2>/dev/null | grep -iE "adb|5555|tcp.port" 
echo "--- 当前 5555 监听 ---"
ss -tlnp 2>/dev/null | grep -E ':5555|:5037'
echo "--- 谁起了 5555 ---"
ps -ef | grep -iE "adb|5555" | grep -v grep

echo
echo "==================== E. bash_history 里 adb 相关（谁在车上用过 adb） ===================="
grep -nE "adb |adb shell|adb pull|adb push|adb connect|adb tcpip|adbkey" /home/duzhong/.bash_history /root/.bash_history 2>/dev/null | tail -30
echo "(若上面为空 = 无人用过 adb 命令)"

echo
echo "==================== F. 项目 src 是否被整包读走的间接证据 ===================="
echo "--- src 目录 atime 异常（被集中读取过） top 20 ---"
find /home/duzhong/dzrobot/src /home/duzhong/dzrobot_v2/src -type f -anewer /home/duzhong/.bash_history 2>/dev/null -printf '%AY-%Am-%Ad %AH:%AM  %p\n' | sort | head -20
echo "(注：atime 可能被 noatime/relatime 影响，仅供参考)"

echo
echo "==================== G. 是否有外部创建的账号/后门 ===================="
cat /etc/passwd | grep -vE "nologin|false" | tail -20
echo "--- 最近添加的用户文件 ---"
ls -la /home/ 2>/dev/null
echo
echo "==================== DONE ===================="
