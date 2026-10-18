#!/usr/bin/env bash
# =============================================================================
# set_affinity.sh —— RK3588S CPU 亲和性绑定 + 实时优先级
# -----------------------------------------------------------------------------
# 为什么需要这个脚本：
#   dzactuator 是 100Hz(cpp 里 rclcpp::Rate(100)) 的硬实时主循环，
#   负责读下位机串口、积分里程、下发速度。抖动 >10ms 就会掉帧，
#   表现为"指令下发延迟/车辆一顿一顿"。
#
#   RK3588S 是 **4×A76(大核, cpu4-7) + 4×A55(小核, cpu0-3)** 异构。
#   Linux 默认调度**不会**保护这个实时环：
#     · det_node 有 3~4 个推理线程 + CPU 密集的预处理/后处理
#     · 它们会被调度到任意核，包括大核，与实时环抢 CPU
#   ⇒ 必须手工绑定：把实时环隔离到小核独享，把推理放大核。
#
# 用法：
#   ./set_affinity.sh              # 应用（需要 sudo）
#   ./set_affinity.sh --status     # 只看当前绑定
#   ./set_affinity.sh --reset      # 清除所有自定义绑定
#
# ★ 现场必须实测验证（见文末"验证方法"）
# =============================================================================
set -u

A55_MASK="0-3"     # 小核：留给实时/控制类节点
A76_MASK="4-7"     # 大核：留给视觉/推理类节点

# 节点名 -> 目标核掩码
#   原则：时间确定性要求高的放小核（A55），吞吐要求高的放大核（A76）
declare -A PIN=(
  # ---- 硬实时：100Hz 主循环，绝不能被抢占 ----
  ["dzactuator"]="A55"
  # ---- 100Hz IMU 航位推算，与主控同频 ----
  ["localization_compensator"]="A55"
  # ---- 50Hz 控制/仲裁，时间敏感 ----
  ["dzmission_node"]="A55"
  ["dzjudgment"]="A55"
  # ---- 图像处理与推理：吞吐型，放小核会拖慢帧率 ----
  ["usb_cam_node"]="A76"
  ["det_node"]="A76"
  ["dzrknn_dataprocess"]="A76"
  # ---- 预录路径跟踪（可选）----
  ["dztrack"]="A55"
)

c_red()  { printf '\033[31m%s\033[0m\n' "$*"; }
c_grn()  { printf '\033[32m%s\033[0m\n' "$*"; }
c_ylw()  { printf '\033[33m%s\033[0m\n' "$*"; }

mask_of() {
  if [ "$1" = "A55" ]; then echo "$A55_MASK"; else echo "$A76_MASK"; fi
}

need_root() {
  if [ "$(id -u)" -ne 0 ]; then
    c_red "需要 root（taskset/chrt 修改其它进程需要权限）。请用 sudo 运行。"
    exit 1
  fi
}

find_pids() {
  # 按可执行名匹配（pgrep -x 精确匹配，避免误伤同名子串）
  pgrep -x "$1" 2>/dev/null || true
}

show_status() {
  echo "=== CPU 拓扑 ==="
  if [ -d /sys/devices/system/cpu/cpu0/cpufreq ]; then
    for c in 0 4; do
      f="/sys/devices/system/cpu/cpu${c}/cpufreq/cpuinfo_max_freq"
      [ -r "$f" ] && echo "  cpu${c} 最高频率: $(cat "$f") kHz"
    done
  fi
  echo "  预期: cpu0-3 = A55(小核), cpu4-7 = A76(大核)"
  echo
  echo "=== 当前绑定 ==="
  printf '  %-28s %-8s %s\n' "进程" "PID" "允许的 CPU"
  for name in "${!PIN[@]}"; do
    pids=$(find_pids "$name")
    if [ -z "$pids" ]; then
      printf '  %-28s %-8s %s\n' "$name" "-" "(未运行)"
      continue
    fi
    for pid in $pids; do
      allowed=$(taskset -pc "$pid" 2>/dev/null | sed 's/.*: //')
      printf '  %-28s %-8s %s\n' "$name" "$pid" "${allowed:-?}"
    done
  done
}

apply() {
  need_root
  echo "=== 应用 CPU 亲和性绑定 ==="
  local applied=0 missing=0
  for name in "${!PIN[@]}"; do
    target=$(mask_of "${PIN[$name]}")
    pids=$(find_pids "$name")
    if [ -z "$pids" ]; then
      printf '  %-28s %s\n' "$name" "跳过（未运行）"
      missing=$((missing+1))
      continue
    fi
    for pid in $pids; do
      if taskset -pc "$target" "$pid" >/dev/null 2>&1; then
        # 实时环进一步提优先级（SCHED_FIFO 需要 root）
        if [ "${PIN[$name]}" = "A55" ]; then
          chrt -f -p 20 "$pid" >/dev/null 2>&1 \
            && extra=" +SCHED_FIFO(20)" || extra=" (chrt 失败，忽略)"
        else
          extra=""
        fi
        c_grn "  $name (pid $pid) -> CPU $target$extra"
        applied=$((applied+1))
      else
        c_red "  $name (pid $pid) 绑定失败"
      fi
    done
  done
  echo
  echo "已绑定 $applied 个进程，$missing 个未运行。"
  echo
  c_ylw "★ 注意：新启动的节点需要重新运行本脚本（或用 systemd/launch 内嵌 taskset）"
}

reset_all() {
  need_root
  echo "=== 清除自定义绑定（恢复全核）==="
  for name in "${!PIN[@]}"; do
    pids=$(find_pids "$name")
    for pid in $pids; do
      taskset -pc "0-7" "$pid" >/dev/null 2>&1 && echo "  $name (pid $pid) -> 0-7"
      chrt -o -p 0 "$pid" >/dev/null 2>&1 || true
    done
  done
}

case "${1:---apply}" in
  --status) show_status ;;
  --reset)  reset_all ;;
  --apply|"") apply ;;
  *) echo "用法: $0 [--apply|--status|--reset]"; exit 1 ;;
esac

cat <<'EOF'

=============================================================================
验证方法（★ 必须现场实测，绑定只是手段，效果要量）
=============================================================================
1) 看实际检测帧率是否达到设定值
     ros2 topic hz /offset_center
   若明显低于 detect_rate_hz，说明 NPU 或 CPU 侧仍有瓶颈。

2) 看主控是否掉帧（最关键）
     ros2 topic hz /odom
   dzactuator 是 100Hz，若明显低于 100Hz 或有长间隔 -> 实时环被抢占。

3) 看温度是否导致降频
     watch -n1 'cat /sys/class/thermal/thermal_zone*/temp'
   持续 >80°C 时会降频，此时算法再好也没用，需要改善散热。

4) 看单核是否有饱和热点
     top -H -p $(pgrep -d, -x det_node)
   若有线程长期 100%，说明预处理/后处理是瓶颈（考虑启用 RGA / 降帧率）。

判读：
  · /offset_center 低但 /odom 正常 -> NPU/视觉侧瓶颈
      -> 降 detect_rate_hz 到 10，或换更小模型，或降分辨率到 480×480
  · /odom 掉帧         -> CPU 抢占
      -> 确认本脚本已生效，并检查是否有其他进程占满 A55
  · 两者都正常但车行为异常 -> 不是算力问题，回到功能/逻辑排查
=============================================================================
EOF
