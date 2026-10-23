#!/usr/bin/env bash
# 飞控代码「源码 vs 已安装产物」一致性核验 + 遗留物排查（只读）
export LC_ALL=C
WS="$HOME/dzpx4"
SRC="$WS/src"
INS="$WS/install"

echo "########## [1] install 与 src 的 launch/config 是否一致 ##########"
for p in px4_control px4_path px4_odom_convert px4_imu_bridge; do
  echo "--- $p ---"
  for kind in launch config msg; do
    s="$SRC/$p/$kind"; i="$INS/$p/share/$p/$kind"
    [ -d "$s" ] || continue
    for f in "$s"/*; do
      [ -f "$f" ] || continue
      b=$(basename "$f")
      if [ -f "$i/$b" ]; then
        a=$(md5sum "$f" | cut -c1-12); c=$(md5sum "$i/$b" | cut -c1-12)
        if [ "$a" = "$c" ]; then echo "   SAME  $kind/$b  $a"
        else echo "   DIFF  $kind/$b  src=$a install=$c"; fi
      else
        echo "   MISSING-IN-INSTALL  $kind/$b"
      fi
    done
  done
done

echo
echo "########## [2] 已编译二进制是否包含源码里的关键字符串 ##########"
BIN="$INS/px4_odom_convert/lib/px4_odom_convert/px4_odom_convert"
echo "   binary: $BIN  ($(stat -c '%y' "$BIN" | cut -d. -f1))"
for s in source_mode localization_topic max_position_jump_m max_attitude_jump_rad z_offset_m "\"/localization\""; do
  n=$(strings "$BIN" 2>/dev/null | grep -Fc "$s")
  echo "   contains $s : $n"
done
echo "   本节点声明的参数（从二进制里看到的 declare 名）:"
strings "$BIN" 2>/dev/null | grep -E '^(source_mode|odom_topic|localization_topic|tf_map_frame|tf_lidar_frame|z_offset_m|loop_rate_hz|max_position_jump_m)$' | sort -u | sed 's/^/     /'

echo
echo "########## [3] 全源码树里是否有人真正声明/使用那两个 jump 参数 ##########"
grep -rn "max_position_jump\|max_attitude_jump" "$SRC" 2>/dev/null | sed 's/^/   /' || echo "   (源码里除 launch 外没有任何引用)"

echo
echo "########## [4] px4_odom_convert 已安装 launch 的参数清单 ##########"
sed -n '1,60p' "$INS/px4_odom_convert/share/px4_odom_convert/launch/px4_odom_convert.launch.py" 2>/dev/null | grep -nE "LaunchConfiguration|default_value|'" | sed 's/^/   /'

echo
echo "########## [5] 谁真正在用哪些包（启动脚本/README 里出现的包名） ##########"
for p in mavros ego_planner fastlio2 fast_lio_localization localizer pgo hba livox_ros_driver2 px4_control px4_path px4_odom_convert px4_imu_bridge px4_ros_com; do
  n=$(grep -rIl --exclude-dir=build --exclude-dir=install --exclude-dir=.git "$p" "$SRC" "$HOME/dzpx4/1.txt" 2>/dev/null | wc -l)
  used=$(grep -rIn --exclude-dir=build --exclude-dir=install "$p" "$HOME/dzpx4/1.txt" 2>/dev/null | head -1)
  printf "   %-22s 被引用的文件数=%-4s 运行手册里=%s\n" "$p" "$n" "${used:-无}"
done

echo
echo "########## [6] PX4 飞控侧参数（EKF2 / 外部视觉）有没有留在机载电脑上 ##########"
grep -rIl --exclude-dir=build --exclude-dir=install -E "EKF2_EV|EKF2_GPS|COM_OBL|EKF2_HGT_MODE|MAV_1_CONFIG" "$HOME" 2>/dev/null | head -20 | sed 's/^/   /' || echo "   (没有找到任何 PX4 参数文件)"
echo "   --- .ros / rosbag 里有没有录过 PX4 参数 ---"
ls -la "$HOME/.ros" 2>/dev/null | head -10 | sed 's/^/   /'

echo
echo "########## [7] uXRCE-DDS 代理与串口环境 ##########"
ls -l /usr/local/bin/MicroXRCEAgent 2>&1 | sed 's/^/   /'
/usr/local/bin/MicroXRCEAgent --version 2>&1 | head -3 | sed 's/^/   /'
echo "   --- 当前没有任何 ttyACM/USB 串口设备 ---"
ls /dev/ttyACM* /dev/ttyUSB* 2>&1 | sed 's/^/   /'

echo
echo "########## [8] gst_camera / test_py / 其它 ##########"
echo "   --- gst_camera 内容 ---"
find "$SRC/gst_camera" -maxdepth 3 2>/dev/null | sed 's/^/   /'
echo "   --- install 里有但 src 里没有的包（历史遗留） ---"
for d in "$INS"/*/; do
  n=$(basename "$d")
  [ -d "$SRC/$n" ] || echo "   $n"
done

echo
echo "核验结束"
