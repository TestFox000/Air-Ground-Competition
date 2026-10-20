#!/usr/bin/env bash
# =============================================================================
# Orin（Jetson）上一键编译 + 分层验证 dzuav（机载侦察栈）
#
# 为什么必须在 Orin 上跑：UdpBroadcaster.cpp 用 POSIX socket 头、CMakeLists
# 硬编码 aarch64-linux-gnu 的 TensorRT 路径、argus: 相机依赖 Jetson 专有的
# nvarguscamerasrc —— Windows 上配不出这套环境，TRT/CUDA 也只随 JetPack 刷机
# 提供（不在 apt 仓库）。
#
# 分层验证（对应 docs/接手说明.md §2.4 六级测试顺序的前几级）：
#   L0  环境自检：arm64 / TensorRT / CUDA（JetPack 刷机产物，缺失即硬失败）
#       + apt 安装 cmake/OpenCV 等编译依赖（默认换清华 ubuntu-ports 源）
#   L1  编译：cmake Release 并行构建 -> onboard/dzuav/build/dzuav
#   L2  二进制自检：无参数运行应打印 usage（退出码 2 属预期）
#   L3  真实推理 + 协议闭环（--run 启用）：用 TRT 引擎跑一张测试图；
#       后台 Python 监听 UDP:50000，用**车端同一个解析器**
#       ag_integrate.intel_protocol.parse 做 magic/CRC16/后缀全量校验
#
# 用法：
#   tools/orin_build_verify.sh                                   # L0~L2
#   tools/orin_build_verify.sh --run ~/engines ~/test/field.jpg  # 追加 L3
#   tools/orin_build_verify.sh --no-mirror                       # 不动 apt 源
#
# apt 源说明：默认改写为清华 ubuntu-ports 镜像（Jetson arm64 专用；只换域名、
# 保留发行代号 focal/jammy，原文件备份 .bak；NVIDIA L4T 源清华不镜像、保持
# 官方不动）。
# =============================================================================
set -euo pipefail

UDP_PORT=50000   # 与 dzuav UdpBroadcaster / 车端 integrate_node 约定一致

REPO=$(cd "$(dirname "$0")/.." && pwd)
SRC_DIR=$REPO/onboard/dzuav
BUILD_DIR=$SRC_DIR/build
BIN=$BUILD_DIR/dzuav

NO_MIRROR=0
RUN_ENGINES=""
RUN_IMAGE=""

# ---------------------------------------------------------------- 输出辅助
if [[ -t 1 ]]; then
    C_STEP=$'\e[1;36m'; C_OK=$'\e[1;32m'; C_WARN=$'\e[1;33m'
    C_ERR=$'\e[1;31m'; C_OFF=$'\e[0m'
else
    C_STEP=''; C_OK=''; C_WARN=''; C_ERR=''; C_OFF=''
fi
step() { printf '%s\n' "${C_STEP}==>${C_OFF} $*"; }
ok()   { printf '%s\n' "${C_OK}[通过]${C_OFF} $*"; }
warn() { printf '%s\n' "${C_WARN}[警告]${C_OFF} $*"; }
fail() { printf '%s\n' "${C_ERR}[失败]${C_OFF} $*" >&2; exit 1; }

print_help() {
    cat <<'EOF'
用法: tools/orin_build_verify.sh [--no-mirror] [--run <engines目录> <测试图>]

分层验证:
  L0    环境自检（arm64/TensorRT/CUDA）+ apt 安装编译依赖（清华源）
  L1    编译 onboard/dzuav -> build/dzuav
  L2    二进制自检（无参数运行应打印 usage）
  L3    真实推理 + UDP 协议闭环（需 --run，目录里放 *hbb*/*cell* 引擎文件）

选项:
  --no-mirror            不改写 apt 源（默认换清华 ubuntu-ports 镜像）
  --run <dir> <image>    追加 L3：用引擎跑一张测试图，并校验广播出的 UDP 帧
  -h, --help             显示本帮助

示例:
  tools/orin_build_verify.sh
  tools/orin_build_verify.sh --run ~/engines ~/test/field.jpg
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-mirror) NO_MIRROR=1; shift ;;
        --run)
            [[ $# -ge 3 ]] || { printf '%s\n' "--run 需要两个参数: <engines目录> <测试图>" >&2; exit 1; }
            RUN_ENGINES=$2; RUN_IMAGE=$3; shift 3 ;;
        -h|--help) print_help; exit 0 ;;
        *) printf '%s\n' "未知参数: $1（-h 查看用法）" >&2; exit 1 ;;
    esac
done

TMP=$(mktemp -d)
LISTENER_PID=""
cleanup() {
    if [[ -n "$LISTENER_PID" ]]; then
        kill "$LISTENER_PID" 2>/dev/null || true
    fi
    rm -rf "$TMP"
}
trap cleanup EXIT

SUDO=""
if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
    SUDO="sudo"
fi

# ---------------------------------------------------------------- L0 环境
check_jetson() {
    step "L0 环境自检：架构 / TensorRT / CUDA"
    local arch
    arch=$(dpkg --print-architecture 2>/dev/null || uname -m)
    if [[ "$arch" != "arm64" && "$arch" != "aarch64" ]]; then
        fail "架构是 $arch（应为 arm64）——本脚本面向 Jetson Orin，请在机载电脑上运行"
    fi
    # 注意不能用 grep -q：本脚本 set -o pipefail，grep -q 匹配后提前退出会让
    # ldconfig 收 SIGPIPE(141)，整条管道被误判失败（2026-10-16 Orin 实测踩坑，
    # 概率性"未找到 TensorRT"误报）。改为读完全部输入再丢弃输出，管道状态即 grep 状态。
    if ! ldconfig -p | grep 'libnvinfer\.so' >/dev/null; then
        fail "未找到 TensorRT (libnvinfer.so)：TRT/CUDA 不在 apt 仓库、只随 JetPack 刷机提供，请先用 SDK Manager 或 sdcard 镜像刷机后再跑本脚本"
    fi
    [[ -d /usr/local/cuda ]] || fail "未找到 /usr/local/cuda（CUDA 同样只随 JetPack 提供）"
    ok "arm64 / TensorRT / CUDA 就绪"
}

configure_mirror() {
    if [[ $NO_MIRROR -eq 1 ]]; then
        warn "--no-mirror：跳过 apt 源配置"
        return 0
    fi
    step "配置 apt 清华镜像源（ubuntu-ports）"
    if grep -rq 'mirrors\.tuna\.tsinghua\.edu\.cn' /etc/apt/sources.list /etc/apt/sources.list.d/ 2>/dev/null; then
        ok "已是清华源，跳过（幂等）"
        return 0
    fi
    local changed=0 f
    for f in /etc/apt/sources.list /etc/apt/sources.list.d/*.list /etc/apt/sources.list.d/*.sources; do
        [[ -f $f ]] || continue
        grep -q 'ports\.ubuntu\.com/ubuntu-ports' "$f" || continue
        $SUDO cp -n "$f" "$f.bak"
        $SUDO sed -i 's|https\?://ports\.ubuntu\.com/ubuntu-ports|https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports|g' "$f"
        printf '  改写 %s（备份: %s.bak）\n' "$f" "$f"
        changed=1
    done
    if [[ $changed -eq 0 ]]; then
        warn "没有 ports.ubuntu.com 条目可改写——若已是第三方镜像可忽略"
    fi
    # NVIDIA L4T 源保持官方（清华不镜像）；update 个别源报错只警告不强退
    if ! $SUDO apt-get update; then
        warn "apt-get update 有报错（常见于 NVIDIA 官方源）——继续尝试安装依赖"
    fi
}

install_deps() {
    step "L0 安装编译依赖（build-essential cmake libopencv-dev gstreamer1.0-tools）"
    if dpkg -s build-essential cmake libopencv-dev gstreamer1.0-tools >/dev/null 2>&1; then
        ok "依赖均已安装"
    else
        $SUDO apt-get install -y build-essential cmake libopencv-dev gstreamer1.0-tools
    fi
    # 相机链路软性自检：nvarguscamerasrc 是 Jetson 专有 GStreamer 插件
    if command -v gst-inspect-1.0 >/dev/null 2>&1 && gst-inspect-1.0 nvarguscamerasrc >/dev/null 2>&1; then
        ok "GStreamer nvarguscamerasrc 可用（argus: MIPI 相机就绪）"
    else
        warn "nvarguscamerasrc 不可用——argus: 模式无法用（uvc: USB 相机不受影响）"
    fi
}

# ---------------------------------------------------------------- L1 / L2
build() {
    step "L1 编译 dzuav（Release，-j$(nproc)）"
    cmake -S "$SRC_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
    cmake --build "$BUILD_DIR" --parallel "$(nproc)"
    [[ -x $BIN ]] || fail "构建完成但未找到可执行文件 $BIN"
    ok "L1 通过：$BIN"
}

verify_binary() {
    step "L2 二进制自检（无参数运行应打印 usage，退出码 2 属预期）"
    local out rc
    set +e
    out=$("$BIN" 2>&1)
    rc=$?
    set -e
    if [[ $rc -eq 2 ]] && grep -q 'usage:' <<<"$out"; then
        ok "L2 通过：usage 输出正常"
    else
        fail "L2 失败：退出码 $rc，输出：$out"
    fi
}

# ---------------------------------------------------------------- L3 实测
run_inference() {
    step "L3 真实推理 + 协议闭环"
    [[ -d $RUN_ENGINES ]] || fail "引擎目录不存在: $RUN_ENGINES"
    [[ -f $RUN_IMAGE ]] || fail "测试图不存在: $RUN_IMAGE"

    local hbb_engine cell_engine
    # 优先 canonical 固定名（13 类部署线：d13 检测 + v4d 格号），找不到才回退模糊匹配。
    # 不能直接用 `find|sort|head -1`：字典序下 cell_classifier_fp16（7 类旧引擎）
    # 排在 cell_classifier_v4d_fp16 前面，会把 L3 验成 7 类，与实跑口径不一致
    # （2026-10-20 复查发现）。
    hbb_engine="$RUN_ENGINES/hbb_detector_d13_fp16.engine"
    cell_engine="$RUN_ENGINES/cell_classifier_v4d_fp16.engine"
    if [[ ! -f $hbb_engine ]]; then
        hbb_engine=$(find "$RUN_ENGINES" -type f \( -name '*.engine' -o -name '*.trt' \) -iname '*hbb*' 2>/dev/null | sort | head -n 1)
    fi
    if [[ ! -f $cell_engine ]]; then
        cell_engine=$(find "$RUN_ENGINES" -type f \( -name '*.engine' -o -name '*.trt' \) -iname '*cell*' 2>/dev/null | sort | head -n 1)
    fi
    [[ -n $hbb_engine ]] || fail "在 $RUN_ENGINES 未找到 hbb 引擎（匹配 *hbb*.engine|.trt）"
    [[ -n $cell_engine ]] || fail "在 $RUN_ENGINES 未找到 cell 引擎（匹配 *cell*.engine|.trt）"
    printf '  hbb  引擎: %s\n  cell 引擎: %s\n  测试图:   %s\n' "$hbb_engine" "$cell_engine" "$RUN_IMAGE"

    # 监听器先起：dzuav 定位成功时会向 UDP:50000 广播一帧，这里用**车端同一个
    # 解析器**做 magic/version/CRC16/后缀全量校验——机上编码与车端解析的闭环。
    step "L3b 启动 UDP:$UDP_PORT 监听器（intel_protocol.parse，超时 30s）"
    REPO_DIR="$REPO" UDP_PORT="$UDP_PORT" python3 - >"$TMP/listener.out" 2>&1 <<'PYEOF' &
import os
import socket
import sys

sys.path.insert(0, os.path.join(os.environ["REPO_DIR"], "src", "ag_integrate"))
from ag_integrate.intel_protocol import parse  # 车端解析器：与车机共用同一实现

names = {0: "paper", 1: "cell_label", 2: "car", 3: "target_red",
         4: "target_blue", 5: "obstacle", 6: "bucket_red",
         7: "bucket_green", 8: "bucket_cyan", 9: "car_start",
         10: "car_end", 11: "uav_start", 12: "uav_end"}

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("", int(os.environ["UDP_PORT"])))
sock.settimeout(30)
try:
    data, _addr = sock.recvfrom(4096)
    frame = parse(data)  # 任何字段/CRC 不一致都会抛 ValueError
except socket.timeout:
    print("TIMEOUT: 30 秒内未收到 UDP 帧")
    sys.exit(1)
except ValueError as exc:
    print(f"BAD_FRAME: {exc}")
    sys.exit(2)
summary = ", ".join(
    f"{names[obj.type]}({obj.x:.2f},{obj.y:.2f})" for obj in frame.objects
) or "无目标"
print(f"UDP_OK: {len(data)} 字节, {len(frame.objects)} 个目标, ts={frame.timestamp_ms}")
print(f"  目标: {summary}")
PYEOF
    LISTENER_PID=$!

    step "L3a 运行推理（首次含引擎反序列化，可能较慢）"
    local out rc
    set +e
    out=$("$BIN" "$RUN_IMAGE" "$hbb_engine" "$cell_engine" "$UDP_PORT")
    rc=$?
    set -e
    printf '%s\n' "$out"
    [[ $rc -eq 0 ]] || fail "推理退出码 $rc"
    grep -q 'localized=' <<<"$out" || fail "stdout 未出现 localized=（推理管线未走完）"

    if grep -q 'localized=1' <<<"$out"; then
        step "L3b 等待监听结果"
        local lrc
        set +e
        wait "$LISTENER_PID"
        lrc=$?
        set -e
        LISTENER_PID=""
        cat "$TMP/listener.out"
        [[ $lrc -eq 0 ]] || fail "L3b 失败：UDP 帧未通过车端解析器校验"
        ok "L3 全链路通过：图像 -> TRT 推理 -> PnP 定位 -> UDP 广播 -> 车端 parse"
    else
        kill "$LISTENER_PID" 2>/dev/null || true
        wait "$LISTENER_PID" 2>/dev/null || true
        LISTENER_PID=""
        warn "本图定位失败（localized=0，未发 UDP 属预期行为）"
        warn "L3a 通过（推理管线完整运行）；换一张含 4 个以上清晰格号的图可补验 L3b"
    fi
}

# ---------------------------------------------------------------- 主流程
check_jetson
configure_mirror
install_deps
build
verify_binary
if [[ -n "$RUN_ENGINES" ]]; then
    run_inference
fi

echo
echo "==================== 完成 ===================="
echo "二进制 : $BIN"
echo "已通过 : L0 环境自检 / L1 编译 / L2 运行自检"
if [[ -n "$RUN_ENGINES" ]]; then
    echo "        L3 真实推理 + 协议闭环（结果见上方）"
else
    echo "未执行 : L3 —— $0 --run <engines目录> <测试图>"
fi
echo "下一步 : 相机实测 dzuav uvc:/dev/video0 <hbb> <cell> $UDP_PORT 1920x1080@30"
echo "=============================================="
