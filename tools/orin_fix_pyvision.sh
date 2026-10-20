#!/usr/bin/env bash
# =============================================================================
# Orin 机上 Python 视觉环境一键修复（video_recon.py 等离线工具依赖）
#
# 两个实测故障（2026-10-20，Orin dzsmart / Ubuntu 22.04 / Python 3.10）：
#   1. numpy 冲突：~/.local 曾装 numpy 2.2.6，而系统 python3-opencv 4.5.4 按
#      numpy 1.x ABI 编译 → import cv2 报 "_ARRAY_API not found"，
#      video_recon.py 等一切用到 cv2 的 Python 工具全部瘫痪。
#      修复：numpy 钉 1.26.4（Python 3.10 上最后的 1.x；机上 open3d/scipy/
#      matplotlib/pandas 实测全部兼容，仅 scipy 打一条无害的版本提示）。
#   2. HardSwish：v4d 格号模型含 HardSwish 算子，OpenCV DNN 4.6 起才支持，
#      系统 4.5.4 加载即报 Can't create layer "HardSwish"。
#      修复：--user 装 opencv-python-headless 4.10.0.84（官方 aarch64 轮子，
#      装进 ~/.local 遮蔽系统 4.5.4）。headless 足够：video_recon 只用
#      readNetFromONNX/imread/imwrite，无 imshow 等 GUI 调用）。
#
# 安全性：
#   - 全部 pip --user 安装（~/.local），不碰系统包、不需要 sudo；
#   - C++ dzuav / 机载规划 sidecar 不依赖 Python cv2，零影响；
#   - 回滚：pip3 uninstall opencv-python-headless numpy
#           && pip3 install --user numpy==2.2.6
#
# 用法（Orin 上，仓库任意位置）：
#   bash tools/orin_fix_pyvision.sh              # 修复 + 导入自检
#   bash tools/orin_fix_pyvision.sh --smoke      # 追加 video_recon 端到端冒烟
#
# 冒烟测试自动找料（可用环境变量覆盖）：
#   D13_ONNX   默认 ~/dzuav/models/hbb*d13*.onnx（取最新）
#   V4D_ONNX   默认 ~/dzuav/models/cell*v4d*.onnx（取最新）
#   VIDEO      默认 ~/Videos/recordings/*.mp4（取最新）
# =============================================================================
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
SMOKE=0
[[ "${1:-}" == "--smoke" ]] && SMOKE=1

say()  { printf '\e[1;36m==>\e[0m %s\n' "$*"; }
ok()   { printf '\e[1;32m[通过]\e[0m %s\n' "$*"; }
warn() { printf '\e[1;33m[警告]\e[0m %s\n' "$*"; }
fail() { printf '\e[1;31m[失败]\e[0m %s\n' "$*" >&2; exit 1; }

# ---------------------------------------------------------------- 安装
say "安装 numpy 1.26.4 + opencv-python-headless 4.10.0.84（--user，不动系统包）"
pip3 install --user 'numpy==1.26.4' 'opencv-python-headless==4.10.0.84'

# route_render.png 需要 Pillow（render_route 懒导入，缺了会静默跳过渲染）
if ! python3 -c 'import PIL' 2>/dev/null; then
    say "补装 Pillow（route_render.png 渲染依赖）"
    pip3 install --user pillow
fi

# ---------------------------------------------------------------- 自检
say "导入自检"
python3 - <<'PY'
import cv2
import numpy

assert numpy.__version__.startswith("1.26"), \
    f"numpy 应为 1.26.x，实际 {numpy.__version__}"
assert tuple(int(v) for v in cv2.__version__.split(".")[:2]) >= (4, 6), \
    f"cv2 应 >= 4.6（HardSwish 支持），实际 {cv2.__version__}"
assert ".local" in cv2.__file__, (
    f"cv2 仍解析到系统包 {cv2.__file__}——"
    "user-site 未遮蔽 4.5.4，请检查 ~/.local 权限或 PYTHONPATH")
print(f"[自检] numpy {numpy.__version__} | cv2 {cv2.__version__} "
      f"({cv2.__file__})")
PY
ok "numpy / cv2 / PIL 就绪（HardSwish 已可用）"

# ---------------------------------------------------------------- 冒烟
if [[ $SMOKE -eq 0 ]]; then
    echo
    ok "完成。需要端到端验证 video_recon 时加 --smoke"
    exit 0
fi

say "定位模型与测试视频"
D13_ONNX="${D13_ONNX:-$(ls -t "$HOME"/dzuav/models/hbb*d13*.onnx 2>/dev/null | head -1 || true)}"
V4D_ONNX="${V4D_ONNX:-$(ls -t "$HOME"/dzuav/models/cell*v4d*.onnx 2>/dev/null | head -1 || true)}"
VIDEO="${VIDEO:-$(ls -t "$HOME"/Videos/recordings/*.mp4 2>/dev/null | head -1 || true)}"
[[ -n "$D13_ONNX" ]] || fail "未找到 D13 模型（~/dzuav/models/hbb*d13*.onnx），用 D13_ONNX=... 指定"
[[ -n "$V4D_ONNX" ]] || fail "未找到 v4d 模型（~/dzuav/models/cell*v4d*.onnx），用 V4D_ONNX=... 指定"
[[ -n "$VIDEO"   ]] || fail "未找到测试视频（~/Videos/recordings/*.mp4），用 VIDEO=... 指定"
echo "  d13  = $D13_ONNX"
echo "  v4d  = $V4D_ONNX"
echo "  video= $VIDEO"

# 模型能否解析（原来 4.5.4 就死在这一步：Can't create layer "HardSwish"）
python3 - "$D13_ONNX" "$V4D_ONNX" <<'PY'
import sys
import cv2
for path in sys.argv[1:]:   # 依次 d13、v4d
    cv2.dnn.readNetFromONNX(path)
    print(f"[自检] ONNX 解析 OK: {path}")
PY
ok "两个 ONNX 模型均可在新 cv2 下解析"

say "video_recon 端到端冒烟（12 采样帧，CPU 推理约 1 分钟内）"
OUT=/tmp/ag_pyvision_smoke
rm -rf "$OUT"
cd "$REPO"
python3 src/ag_integrate/ag_integrate/video_recon.py "$VIDEO" --out "$OUT" \
    --max-frames 12 --stride 30 --d13 "$D13_ONNX" --v4d "$V4D_ONNX"

for f in field_state.json route_plan.json route_render.png; do
    [[ -s "$OUT/$f" ]] || fail "产物缺失: $OUT/$f"
done
python3 - "$OUT" <<'PY'
import json
import sys
from pathlib import Path

out = Path(sys.argv[1])
state = json.loads((out / "field_state.json").read_text(encoding="utf-8"))
plan = json.loads((out / "route_plan.json").read_text(encoding="utf-8"))
print(f"[产物] 雷 {state['mines_video']} | 障碍 {state['obstacles_video']}"
      f" | 蓝靶 {state['targets_video']['blue']}")
cmd = plan["car_cmds"]["route"]
print(f"[产物] 路线 {plan['route_len_cells']} 格 | 转向 {plan['turns']} 次"
      f" | 车命令 {cmd[:52]}{'…' if len(cmd) > 52 else ''}")
PY
echo
ok "video_recon 冒烟通过：field_state / route_plan / route_render 三产物齐全"
echo "现场复用命令："
echo "  cd $REPO && python3 src/ag_integrate/ag_integrate/video_recon.py <视频> \\"
echo "      --d13 $D13_ONNX --v4d $V4D_ONNX"
