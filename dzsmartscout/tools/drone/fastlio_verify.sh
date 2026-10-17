#!/usr/bin/env bash
# FAST-LIO 定位链路的关键事实核验（只读）
export LC_ALL=C
SRC="$HOME/dzpx4/src/FAST_LIO_LOCALIZATION_ROS2"
INS="$HOME/dzpx4/install/fast_lio_localization"

echo "########## [1] install 与 src 的 config/launch/scripts 是否一致 ##########"
for kind in config launch scripts rviz; do
  s="$SRC/$kind"; i="$INS/share/fast_lio_localization/$kind"
  [ -d "$s" ] || continue
  echo "--- $kind ---"
  ls -1 "$i" 2>/dev/null | while read -r b; do
    if [ -f "$s/$b" ]; then
      a=$(md5sum "$s/$b" | cut -c1-12); c=$(md5sum "$i/$b" | cut -c1-12)
      [ "$a" = "$c" ] && echo "   SAME $b" || echo "   DIFF $b src=$a ins=$c"
    else
      echo "   ONLY-IN-INSTALL $b"
    fi
  done
  ls -1 "$s" | while read -r b; do
    [ -f "$i/$b" ] || echo "   ONLY-IN-SRC $b"
  done
done

echo
echo "########## [2] launch 里引用的 rviz 配置是否存在 ##########"
grep -n "rviz" "$SRC/launch/velodyne_localization.launch.py" | sed 's/^/   /'
echo "   实际存在的 rviz 配置："
ls -1 "$SRC/rviz" "$SRC/rviz_cfg" 2>/dev/null | sed 's/^/     /'

echo
echo "########## [3] python 依赖是否真的装上了（决定重定位到底能不能跑） ##########"
for m in open3d tf_transformations numpy scipy sklearn; do
  r=$(/usr/bin/python3 -c "import $m,sys;print(getattr($m,'__version__','?'))" 2>&1 | tail -1)
  printf "   %-18s %s\n" "$m" "$r"
done
echo "   --- pip 里有没有 open3d ---"
/usr/bin/python3 -m pip list 2>/dev/null | grep -iE "open3d|transformations|scikit" | sed 's/^/     /' || echo "     (pip 查询无结果)"
echo "   --- 包内 4 个 python 脚本 ---"
ls -l "$SRC/scripts" | sed 's/^/   /'

echo
echo "########## [4] 地图 output_voxel_0.05.pcd 的几何（验证 map 的 z 是否竖直向上） ##########"
/usr/bin/python3 - <<'PY'
import struct, os
for name in ("output_voxel_0.05.pcd", "map_compile_4cm.pcd", "map.pcd"):
    p = os.path.expanduser("~/dzpx4/src/maps/" + name)
    if not os.path.exists(p):
        print(f"   {name}: 不存在"); continue
    with open(p, "rb") as f:
        hdr = {}
        while True:
            line = f.readline().decode("ascii", "ignore").strip()
            if not line or line.startswith("#"):
                continue
            k, _, v = line.partition(" ")
            hdr[k] = v
            if k == "DATA":
                break
        fields = hdr.get("FIELDS", "").split()
        sizes = [int(x) for x in hdr.get("SIZE", "").split()]
        types = hdr.get("TYPE", "").split()
        counts = [int(x) for x in hdr.get("COUNT", "1" * len(fields)).split()]
        n = int(hdr.get("POINTS", 0))
        stride = sum(s * c for s, c in zip(sizes, counts))
        fmt = {("F", 4): "f", ("F", 8): "d", ("U", 1): "B", ("U", 2): "H", ("U", 4): "I"}[
            (types[fields.index("x")], sizes[fields.index("x")])]
        raw = f.read()
    xs, ys, zs = [], [], []
    off = 0
    for _ in range(n):
        base = off
        vals = []
        for i, fl in enumerate(fields):
            sz, ct, ty = sizes[i], counts[i], types[i]
            for c in range(ct):
                chunk = raw[base:base + sz]
                if ty == "F":
                    vals.append(struct.unpack("<f" if sz == 4 else "<d", chunk)[0])
                base += sz
            if fl in ("x", "y", "z"):
                pass
        ix, iy, iz = fields.index("x"), fields.index("y"), fields.index("z")
        xs.append(vals[ix]); ys.append(vals[iy]); zs.append(vals[iz])
        off += stride
    print(f"   {name}: POINTS={n} stride={stride}B  DATA={hdr.get('DATA')}")
    print(f"      x ∈ [{min(xs):.2f}, {max(xs):.2f}]  ({max(xs)-min(xs):.1f} m)")
    print(f"      y ∈ [{min(ys):.2f}, {max(ys):.2f}]  ({max(ys)-min(ys):.1f} m)")
    print(f"      z ∈ [{min(zs):.2f}, {max(zs):.2f}]  ({max(zs)-min(zs):.1f} m)")
    # z 直方图（0.1 m 一格）
    import collections
    h = collections.Counter(round(z, 1) for z in zs)
    top = h.most_common(3)
    for zval, cnt in top:
        print(f"      z≈{zval:+.1f} m : {cnt} 点  ({100.0*cnt/n:.1f}%)")
PY

echo
echo "########## [5] ROS 历史日志里这些节点到底跑过没有、有没有报错 ##########"
LOGROOT="$HOME/.ros/log"
echo "   日志目录数: $(ls -1 "$LOGROOT" 2>/dev/null | wc -l)"
for pat in global_localization transform_fusion publish_initial_pose fastlio_mapping px4_control_node path_follow_node path_record_node px4_odom_convert sensor_combined_to_imu; do
  d=$(grep -rl "$pat" "$LOGROOT" 2>/dev/null | head -1)
  n=$(grep -rl "$pat" "$LOGROOT" 2>/dev/null | wc -l)
  printf "   %-24s 出现在 %-4s 个日志文件中   最近一次: %s\n" "$pat" "$n" "${d:-无}"
done
echo "   --- 最近 5 个 run 的目录与主要日志文件 ---"
for d in $(ls -dt "$LOGROOT"/*/ 2>/dev/null | head -5); do
  echo "   == $d"
  ls -lt "$d" | head -6 | sed 's/^/      /'
done
