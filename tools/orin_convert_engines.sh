#!/usr/bin/env bash
# 从仓库 models/release/*.onnx 重新生成 FP16 TensorRT 引擎（Orin 上执行）。
#
# 用途：换机 / 重刷 JetPack / 升级 TensorRT 后，旧 .engine 会因 TRT 版本或
# GPU 架构不匹配而加载失败 —— .engine 是「TRT 版本 + GPU 型号」绑定产物，
# 而 ONNX 是通用交换格式（已入库 models/release/），届时跑本脚本重新生成即可。
#
# 用法：
#   tools/orin_convert_engines.sh                  # 引擎输出到 ~/dzuav/engines
#   tools/orin_convert_engines.sh -o ~/engines     # 指定输出目录
#
# 前置：JetPack 已刷（trtexec 在 /usr/src/tensorrt/bin），仓库已 git pull。
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OUT_DIR="$HOME/dzuav/engines"

while [[ $# -gt 0 ]]; do
    case "$1" in
        -o|--output) OUT_DIR="$2"; shift 2 ;;
        -h|--help)
            # 只打文件头那段连续注释块（同 orin_bringup.sh 的理由）
            awk 'NR>1 && /^#/ {sub(/^# ?/, ""); print; next} NR>1 {exit}' "$0"
            exit 0 ;;
        *) echo "未知参数: $1" >&2; exit 2 ;;
    esac
done

TRTEXEC=""
for candidate in /usr/src/tensorrt/bin/trtexec /usr/local/bin/trtexec; do
    # 2026-10-16 实机踩坑：`break fi` 之间缺分号是 bash 语法错误，机上首跑即暴露
    if [[ -x "$candidate" ]]; then TRTEXEC="$candidate"; break; fi
done
if [[ -z "$TRTEXEC" ]]; then
    echo "找不到 trtexec：请确认已刷 JetPack（TensorRT 组件），或 locate trtexec" >&2
    exit 3
fi

mkdir -p "$OUT_DIR"

# resolve <指针名> <无版本回退名> -> 实际 onnx 文件名
# 优先读 release_models.py 维护的 current 指针（带版本名入库，指针指最新），
# 没有指针则回退无版本名（历史遗留），最后按修改时间取最新带版本文件。
resolve() {
    local pointer="$1" fallback="$2"
    local name
    name=$(cat "$REPO/models/release/$pointer" 2>/dev/null || true)
    if [[ -n "$name" && -f "$REPO/models/release/$name" ]]; then
        echo "$name"; return 0
    fi
    if [[ -f "$REPO/models/release/$fallback" ]]; then
        echo "$fallback"; return 0
    fi
    name=$(ls -1t "$REPO/models/release/" 2>/dev/null | grep -E "^${fallback%.*}_" | head -n 1 || true)
    [[ -n "$name" ]] && { echo "$name"; return 0; }
    return 1
}

# convert <指针名> <无版本回退名> <输出引擎名>
convert() {
    local onnx_name
    onnx_name=$(resolve "$1" "$2") || { echo "找不到模型（指针 $1 / 回退 $2 均无）: 先 git pull" >&2; exit 4; }
    local onnx="$REPO/models/release/$onnx_name"
    local engine="$OUT_DIR/$3"
    local tmp="$engine.tmp.$$"

    # 转换前先过契约门禁：引擎名与形状是两回事，名字对不代表通道数对。
    # 7 类 ONNX 转出来的引擎一上机就在 HbbDetector 构造期 throw，
    # 而那时旧的可用引擎已经被覆盖掉了——所以这一步必须在写任何文件之前。
    if ! python3 "$REPO/tools/ml/verify_contract.py" --model "$onnx"; then
        echo "[失败] $onnx_name 不符合当前契约，拒绝转换（不覆盖既有引擎）" >&2
        exit 5
    fi

    echo "==> 转换 $onnx_name -> $engine（FP16）"
    # 输入形状均为静态契约（hbb 1280/cell 96），无需 --shapes；--fp16 允许 FP16 内核。
    #
    # 先写 .tmp 再 mv：trtexec --saveEngine 一开写就截断目标文件，中途 OOM /
    # 掉线 / Ctrl-C 会把机上唯一可用的引擎毁掉，而 .engine 是「TRT 版本 + GPU
    # 型号」绑定产物，**在 PC 上无法重建**。mv 同目录内是原子的。
    rm -f "$tmp"
    if ! "$TRTEXEC" --onnx="$onnx" --saveEngine="$tmp" --fp16; then
        rm -f "$tmp"
        echo "[失败] trtexec 转换失败，既有引擎未被触碰: $engine" >&2
        exit 6
    fi

    # 目标已存在则先留备份（带时间戳，不覆盖上一次备份）
    if [[ -f "$engine" ]]; then
        local bak="$engine.bak_$(date +%Y%m%d_%H%M%S)"
        mv -f "$engine" "$bak"
        echo "    旧引擎已备份: $(basename "$bak")"
    fi
    mv -f "$tmp" "$engine"
    echo "    完成: $(basename "$engine") ($(du -h "$engine" | cut -f1))"
}

# 输出引擎名必须与 tools/orin_run_dzuav.sh 的 HBB_ENGINE / CELL_ENGINE 默认值、
# tools/orin_build_verify.sh 的 CANON_*_ENGINE 逐字一致，否则「换机后重转引擎」
# 得到的产物启动脚本根本找不到、L3 也会拿另一个引擎验出假绿。
# tests/test_tools_smoke.py::test_convert_and_run_agree_on_engine_names 锁这三处，
# 改一处必须改三处。
HBB_ENGINE_NAME="${HBB_ENGINE_NAME:-hbb_detector_d13_fp16.engine}"
CELL_ENGINE_NAME="${CELL_ENGINE_NAME:-cell_classifier_v4d_fp16.engine}"

convert "current_hbb_detector"    "hbb_detector.onnx"    "$HBB_ENGINE_NAME"
convert "current_cell_classifier"  "cell_classifier.onnx" "$CELL_ENGINE_NAME"

echo "=================================================="
echo "引擎已生成于: $OUT_DIR"
ls -lh "$OUT_DIR"
echo "可配合 tools/orin_run_dzuav.sh 启动 dzuav。"
echo "=================================================="
