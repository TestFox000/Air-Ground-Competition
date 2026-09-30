#!/usr/bin/env bash
# 车端解压源码包（配合"打包成单个 zip 上传"用）
#
# 为什么不用 `car.py deploy` 逐文件上传：车端 Wi-Fi 很抖，传几百个小文件时
# SSH 连接会**反复被重置**（实测连传 4 轮全断）。改成一个 zip 只传一次，
# 单文件传输成功率高一截（2026-09-30 踩过后定下的做法）。
set -o pipefail
ZIP="${AG_ZIP:-/tmp/ag_ws/src.zip}"
DEST="${AG_DEST:-/tmp/ag_ws}"

echo "==================== 解压源码包 ===================="
date
if [ ! -f "$ZIP" ]; then
  echo "  [FAIL] 找不到 $ZIP"; exit 1
fi
echo "  包: $ZIP  ($(du -h "$ZIP" 2>/dev/null | awk '{print $1}'))"

# ⚠️ **不要整目录删除**（用户 2026-09-30 明确要求：车里的码别删，传一次很费劲）。
#    解压是**覆盖式**的：同名文件被替换，车端已有的其它文件/编译产物一律保留。
#    这样即使上传中途断了，车上的东西也不会被清掉。
mkdir -p "$DEST/src"
python3 - "$ZIP" "$DEST" <<'PY'
import sys, zipfile
z, d = sys.argv[1], sys.argv[2]
with zipfile.ZipFile(z) as f:
    f.extractall(d)
print("  解压完成")
PY

echo "  -- 解压后的包 --"
ls -1 "$DEST/src" 2>/dev/null | sed 's/^/      /'
echo "  -- 校验关键文件 --"
for f in "$DEST/src/ag_interfaces/CMakeLists.txt" "$DEST/src/ag_core/ag_core/grid_map.py" \
         "$DEST/src/ag_car/ag_car/car_node.py"; do
  [ -f "$f" ] && echo "      [OK] ${f#$DEST/src/}" || echo "      [缺] ${f#$DEST/src/}"
done
echo "==================== 结束 ===================="
