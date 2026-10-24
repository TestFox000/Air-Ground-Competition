#!/usr/bin/env bash
# AGC-16 补丁 07-tfcompat：transform_fusion 的 tf2_ros.transformations 兜底 -> 本地 tf_compat.py
#
# 做什么：
#   1) 把 files/dzpx4/src/FAST_LIO_LOCALIZATION_ROS2/ 下的 3 个文件拷到目标机同名路径
#      - scripts/tf_compat.py        （新增，numpy-only 的 4 个变换函数）
#      - scripts/transform_fusion.py （只改 import 兜底；其余逻辑零改动）
#      - CMakeLists.txt              （install(PROGRAMS) 多列一行 scripts/tf_compat.py）
#   2) 先备份被覆盖的原文件到 <dest>/.tfcompat-backup-<时间戳>/
#   3) 就地做四项校验（import 成功 / 兜底串在位 / 旧兜底已清除 / CMake 已列 tf_compat）
#
# 用法：
#   bash apply.sh                      # 目标默认 ~/dzpx4
#   bash apply.sh -d /path/to/dzpx4    # 指定其它目标根
#   bash apply.sh --rollback           # 从最近一次备份还原（并删掉新增的 tf_compat.py）
#   bash apply.sh -h
#
# 注意：本脚本只改源文件，**不**执行 colcon build，也不重启任何节点。
set -euo pipefail

SELF_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MIRROR_REL="files/dzpx4/src/FAST_LIO_LOCALIZATION_ROS2"
PKG_NAME="FAST_LIO_LOCALIZATION_ROS2"

usage() {
  cat <<EOF
AGC-16 补丁 07-tfcompat

  bash apply.sh [-d DEST]        应用补丁（DEST 默认 $HOME/dzpx4）
  bash apply.sh --rollback [-d DEST]
                                 从最近一次 .tfcompat-backup-* 还原
  bash apply.sh -h               本帮助

DEST 下应存在 src/FAST_LIO_LOCALIZATION_ROS2/scripts/。
EOF
}

MODE="apply"
DEST="${HOME}/dzpx4"
while [ $# -gt 0 ]; do
  case "$1" in
    --rollback) MODE="rollback"; shift ;;
    -d|--dest)  DEST="${2:-}"; shift 2 ;;
    -h|--help)  usage; exit 0 ;;
    *) echo "未知参数：$1" >&2; usage >&2; exit 2 ;;
  esac
done

[ -n "${DEST}" ] || { echo "错误：DEST 为空" >&2; exit 2; }
TARGET="${DEST}/src/${PKG_NAME}"

# 找一个**真能用**的 python（Windows 上 python3 常是 Microsoft Store 的占位 stub：
# 能"跑"但什么都干不了，所以这里必须真的 import numpy 试一次）。
pick_python() {
  for cand in python3 python; do
    if command -v "$cand" >/dev/null 2>&1 && "$cand" -c "import numpy" >/dev/null 2>&1; then
      echo "$cand"; return 0
    fi
  done
  echo ""
}

if [ "${MODE}" = "rollback" ]; then
  [ -d "${DEST}" ] || { echo "错误：DEST 不存在：${DEST}" >&2; exit 2; }
  BK="$(ls -1d "${DEST}"/.tfcompat-backup-* 2>/dev/null | sort | tail -n 1 || true)"
  [ -n "${BK}" ] || { echo "错误：${DEST} 下没有 .tfcompat-backup-* 可还原" >&2; exit 2; }
  echo "== 回滚：从 ${BK} =="
  for f in scripts/transform_fusion.py CMakeLists.txt; do
    if [ -f "${BK}/${f}" ]; then
      cp -p "${BK}/${f}" "${TARGET}/${f}"
      echo "  还原 ${f}"
    else
      echo "  跳过 ${f}（备份里没有）" >&2
    fi
  done
  if [ -f "${TARGET}/scripts/tf_compat.py" ] && [ ! -f "${BK}/scripts/tf_compat.py" ]; then
    rm -f "${TARGET}/scripts/tf_compat.py"
    echo "  删除新增的 scripts/tf_compat.py"
  fi
  echo "回滚完成。若要生效：cd ${DEST} && colcon build --packages-select fast_lio_localization"
  exit 0
fi

# ---------------- apply ----------------
[ -d "${SELF_DIR}/${MIRROR_REL}/scripts" ] || { echo "错误：补丁包不完整，缺 ${MIRROR_REL}" >&2; exit 2; }
if [ ! -d "${TARGET}/scripts" ]; then
  echo "错误：目标不存在：${TARGET}/scripts" >&2
  echo "      请确认 -d 指向 dzpx4（其下有 src/${PKG_NAME}），机上示例：-d ~/dzpx4" >&2
  exit 2
fi

TS="$(date +%Y%m%d-%H%M%S)"
BK="${DEST}/.tfcompat-backup-${TS}"
mkdir -p "${BK}/scripts"
echo "== 备份原文件 -> ${BK} =="
for f in scripts/transform_fusion.py CMakeLists.txt; do
  if [ -f "${TARGET}/${f}" ]; then
    cp -p "${TARGET}/${f}" "${BK}/${f}"
    echo "  ${f}"
  fi
done

echo "== 拷贝补丁文件 =="
cp -p "${SELF_DIR}/${MIRROR_REL}/scripts/tf_compat.py"        "${TARGET}/scripts/tf_compat.py"
cp -p "${SELF_DIR}/${MIRROR_REL}/scripts/transform_fusion.py" "${TARGET}/scripts/transform_fusion.py"
cp -p "${SELF_DIR}/${MIRROR_REL}/CMakeLists.txt"              "${TARGET}/CMakeLists.txt"
echo "  scripts/tf_compat.py（新增）"
echo "  scripts/transform_fusion.py（import 兜底）"
echo "  CMakeLists.txt（install 多一行）"

echo "== 校验 =="
PY="$(pick_python)"
if [ -n "${PY}" ]; then
  "${PY}" -c "import sys; sys.path.insert(0, '${TARGET}/scripts'); import tf_compat; print('  import tf_compat OK:', len(tf_compat.__all__), 'functions')"
else
  echo "  跳过 import 校验（本机无 python）" >&2
fi
if grep -q "import tf_compat as tf_transformations" "${TARGET}/scripts/transform_fusion.py"; then
  echo "  import 兜底在位"
else
  echo "  兜底串缺失" >&2; exit 1
fi
if grep -q "tf2_ros import transformations" "${TARGET}/scripts/transform_fusion.py"; then
  echo "  旧兜底仍在（from tf2_ros import transformations）" >&2; exit 1
else
  echo "  旧兜底已清除"
fi
if grep -q "scripts/tf_compat.py" "${TARGET}/CMakeLists.txt"; then
  echo "  CMakeLists 已列 tf_compat.py"
else
  echo "  CMakeLists 未列 tf_compat.py" >&2; exit 1
fi

cat <<EOF

打完了。要生效还需要（本脚本不代替你执行）：
  cd ${DEST} && colcon build --packages-select fast_lio_localization && source install/setup.bash
  # 然后照常用 launch 起定位链，另开终端验：
  ros2 topic hz /localization        # 期望有稳定频率（修之前：话题不存在）

回滚：
  bash ${SELF_DIR}/apply.sh --rollback -d ${DEST}
EOF
