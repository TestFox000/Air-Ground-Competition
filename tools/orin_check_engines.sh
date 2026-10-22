#!/usr/bin/env bash
# 机上引擎指纹核查（2026-10-22）：md5 / inode / 链接数 + 抽一张 L3 测试帧。
#
# 用途：① 确认 ~/dzuav/engines 下各名字对应的**内容**是否同一份（旧 7 类名常是
#       人工别名，md5 相同即内容一致）；② 为 tools/orin_build_verify.sh --run 备测试图。
# 结论口径：名字不等于内容，对账看 md5；引擎与二进制都是逐字节上机，md5 不符先查行尾。
set -u
cd "$HOME/dzuav/engines" || exit 9
echo "=== 引擎 md5 / inode / 链接数 ==="
for f in *.engine; do
    printf '%s  %s\n' "$(md5sum "$f" | cut -c1-32)" "$f"
done
ls -li *.engine | awk '{print "inode " $1 "  links " $3 "  " $NF}'

echo "=== 抽 L3 测试帧 ==="
python3 - <<'PY'
import cv2
cap = cv2.VideoCapture('/home/duzhong/Videos/recordings/cam_20261018_214302.mp4')
cap.set(cv2.CAP_PROP_POS_FRAMES, 900)
ok, frame = cap.read()
if ok:
    cv2.imwrite('/tmp/l3_frame.jpg', frame)
    print(f"[抽帧] ok shape={frame.shape} -> /tmp/l3_frame.jpg")
else:
    print("[抽帧] 失败")
PY