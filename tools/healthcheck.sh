#!/usr/bin/env bash
# =============================================================================
#  智能车 · 出车体检脚本   healthcheck.sh
# -----------------------------------------------------------------------------
#  出车前跑一次，30 秒内出一份"全绿/标红"报告，把今天踩过的坑全部前置拦截：
#
#    · 设备在不在          (相机 / 底盘串口 / 云台串口)
#    · 配置中心读不读得到  + 三条硬性参数不变量
#    · ★ 运行时参数是否真的设上了
#        —— 这是最有价值的一项：以前用 LaunchConfiguration 传参是字符串，
#           与节点声明的 double 类型不匹配会【静默失败】，参数根本没设上，
#           而日志里看代码默认值一切"正常"。本脚本逐个参数与配置中心比对。
#    · 节点在不在 / 话题出不出数据
#    · 端口有没有被残留进程占住
#
#  用法:
#      ./healthcheck.sh              # 完整体检（节点在跑时最有用）
#      ./healthcheck.sh --pre        # 仅静态检查（出车前，节点还没起）
#
#  退出码: 0=全过  1=有 ❌ 失败项  2=只有 ⚠️ 警告
# =============================================================================
set -uo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG="${ROBOT_CONFIG:-$WS/config/robot.yaml}"
MODE="${1:-full}"

FAIL=0
WARN=0
ok()   { printf "  \033[32m✅\033[0m %-34s %s\n" "$1" "${2:-}"; }
bad()  { printf "  \033[31m❌\033[0m %-34s %s\n" "$1" "${2:-}"; FAIL=$((FAIL+1)); }
warn() { printf "  \033[33m⚠️ \033[0m %-34s %s\n" "$1" "${2:-}"; WARN=$((WARN+1)); }
sec()  { printf "\n\033[1m【%s】\033[0m\n" "$1"; }

printf "\n\033[1m==============================================================\033[0m\n"
printf "\033[1m  智能车 · 出车体检报告\033[0m\n"
printf "  时间: %s   配置: %s\n" "$(date '+%Y-%m-%d %H:%M:%S')" "$CONFIG"
printf "\033[1m==============================================================\033[0m\n"

# -----------------------------------------------------------------------------
# 1) 设备
# -----------------------------------------------------------------------------
sec "1/7 设备"
for d in /dev/video0; do
  [ -e "$d" ] && ok "$d" "存在" || bad "$d" "缺失 —— 检查相机 USB"
done
acm=$(ls /dev/ttyACM* 2>/dev/null | tr '\n' ' ')
usb=$(ls /dev/ttyUSB* 2>/dev/null | tr '\n' ' ')
if [ -n "$acm$usb" ]; then ok "串口设备" "$acm$usb"
else bad "串口设备" "未发现 ttyACM*/ttyUSB* —— 检查底盘/云台接线"; fi

# -----------------------------------------------------------------------------
# 2) 配置中心 + 硬性不变量
# -----------------------------------------------------------------------------
sec "2/7 配置中心与参数不变量"
if [ ! -f "$CONFIG" ]; then
  bad "config/robot.yaml" "不存在 —— 所有节点会回退到代码内置默认值"
else
  ok "config/robot.yaml" "存在"
  # Python 侧读 YAML 做不变量校验
  INV=$(python3 - "$CONFIG" <<'PY'
import sys
try:
    import yaml
    d = yaml.safe_load(open(sys.argv[1], encoding='utf-8')) or {}
except Exception as e:
    print("PARSE_FAIL|%s" % e); raise SystemExit(0)
p = (d.get('/dzrknn_dataprocess') or {}).get('ros__parameters', {}) or {}
qz  = p.get('track_deadzone_px')
ff  = p.get('fire_dz_far_px')
lf  = p.get('lost_frames_to_rescan')
pm  = p.get('patrol_speed_ratio')
po  = p.get('scan_pitch_offset_deg')
fb  = p.get('fire_only_blue')
mt  = p.get('max_shots_per_target')
print("OK|%s|%s|%s|%s|%s|%s|%s" % (qz, ff, lf, pm, po, fb, mt))
PY
)
  case "$INV" in
    PARSE_FAIL*) bad "robot.yaml 解析" "${INV#PARSE_FAIL|}";;
    OK\|*)
      IFS='|' read -r _ QZ FF LF PM PO FB MT <<< "$INV"
      # ① 死区 < 最小开火框
      if [ -n "$QZ" ] && [ -n "$FF" ] && awk "BEGIN{exit !($QZ < $FF)}"; then
        ok "不变量①  死区 < 最小开火框" "${QZ}px < ${FF}px"
      else
        bad "不变量①  死区 < 最小开火框" "${QZ}px 不小于 ${FF}px → 目标会停在死区里，一发打不出"
      fi
      # ② 开火框 > 机械精度参考(12px)
      if [ -n "$FF" ] && awk "BEGIN{exit !($FF <= 12)}"; then
        warn "不变量②  开火框 > 机械精度" "${FF}px 偏小(云台实际能压到约 12px)，可能时打时不打"
      else
        ok "不变量②  开火框 > 机械精度" "${FF}px > 12px 参考值"
      fi
      # ③ 丢帧容忍 >= 60
      if [ -n "$LF" ] && [ "$LF" -ge 60 ] 2>/dev/null; then
        ok "不变量③  丢帧容忍 >= 60" "${LF} 帧(约 $((LF/30))s)"
      else
        warn "不变量③  丢帧容忍 >= 60" "${LF} 帧偏小，识别间歇时易被判'脱离视野'"
      fi
      printf "     ─── 关键参数 ───\n"
      printf "     %-24s %s\n" "巡逻速度比例"   "${PM:-<未设>}"
      printf "     %-24s %s\n" "俯仰偏移(度)"   "${PO:-<未设>}"
      printf "     %-24s %s\n" "仅打蓝靶"       "${FB:-<未设>}"
      printf "     %-24s %s\n" "单目标连发上限" "${MT:-<未设>}"
      ;;
  esac

  # ★★ ROS params 文件结构校验（致命项，必须放在最前）★★
  #   踩过的坑：往 robot.yaml 里加了一个顶层 `map:` 段（普通键值而非 ros__parameters），
  #   结果 --params-file 解析失败，usb_cam / det_node / dzrknn_dataprocess
  #   【三个节点同时 abort(exit -6)】—— 视觉与云台全线崩，而车还在按路线行驶。
  #   而 `yaml.safe_load` 是能正常解析它的（语法没错，是 ROS 的结构约束），
  #   所以普通的 YAML 可解析性检查【根本拦不住】。这里补上结构校验。
  STRUCT=$(python3 - "$CONFIG" <<'PY'
import sys
try:
    import yaml
    d = yaml.safe_load(open(sys.argv[1], encoding='utf-8')) or {}
except Exception as e:
    print("PARSE_FAIL|%s" % e); raise SystemExit(0)
bad = []
for k, v in d.items():
    if not isinstance(v, dict):
        bad.append("顶层键 '%s' 不是字典" % k)
    elif 'ros__parameters' not in v:
        bad.append("段 '%s' 缺少 ros__parameters" % k)
print("OK|%d" % len(d) if not bad else "BAD|" + "；".join(bad))
PY
)
  case "$STRUCT" in
    PARSE_FAIL*) bad "params 文件解析" "${STRUCT#PARSE_FAIL|}";;
    BAD*)        bad "params 文件结构非法" "${STRUCT#BAD|}"
                 printf "        \033[31mROS 会拒绝加载它 → 用到 --params-file 的节点(相机/视觉/云台)会全部崩溃\033[0m\n"
                 printf "        \033[31m顶层只允许 <节点名>: ros__parameters: 结构；地图等数据请放 config/map.yaml\033[0m\n";;
    OK*)         ok "params 文件结构" "${STRUCT#OK|} 个节点段，符合 ROS 约束";;
  esac

  # ★★ 静态防呆：declare_parameter 与 get_parameter 必须成对 ★★
  #   踩过的坑：scan_pitch_offset_deg 声明了却从未 get -> 配置文件改了只更新参数服务器，
  #   ros2 param get 显示新值、本脚本的运行时比对也"一致"，但代码用的是头文件默认值。
  #   这类"参数设上了、代码却不读"的静默失败，运行时比对【根本查不出来】，只能查源码。
  #   代价：俯仰参数改了三次都毫无效果，排查了很久。
  MISPAIR=$(python3 - "$WS" <<'PY'
import re, os, sys
ws = sys.argv[1]
hits = []
for pkg in ('dzrknn_dataprocess', 'det_node'):
    srcs = []
    for root, _d, fs in os.walk(os.path.join(ws, 'src')):
        if pkg not in root:
            continue
        for f in fs:
            if f.endswith(('.cpp', '.cc')):
                srcs.append(os.path.join(root, f))
    for f in srcs:
        try:
            txt = open(f, encoding='utf-8', errors='ignore').read()
        except Exception:
            continue
        decl = set(re.findall(r'declare_parameter\(\s*"([^"]+)"', txt))
        getp = set(re.findall(r'get_parameter\(\s*"([^"]+)"', txt))
        for k in sorted(decl - getp):
            hits.append('%s:%s' % (os.path.basename(f), k))
print('\n'.join(hits))
PY
)
  if [ -n "$MISPAIR" ]; then
    bad "参数声明未配对" "$(echo "$MISPAIR" | wc -l) 个 declare 后没有 get"
    echo "$MISPAIR" | head -6 | sed 's/^/        ❌ /'
    printf "     \033[31m这类参数改了不生效，且运行时比对查不出来（只更新参数服务器）\033[0m\n"
  else
    ok "参数声明配对" "declare / get 全部成对"
  fi

  # ★★ 狙击链路：激光到底能不能亮 ★★
  #   踩过的巨坑：dzjudgment（射击驱动）以前根本没被启动过，导致
  #     ros2 topic info /LaserShot_Command -> Publisher 3 / Subscription 0
  #   云台算得好好的开火指令全发进了空话题，激光永远不亮且毫无报错。
  #   所以这里必须把"链路两端"都查一遍：设备在不在 + 订阅者在不在。
  if [ -e /dev/dzjudgment ]; then
    ok "激光串口" "/dev/dzjudgment -> $(readlink /dev/dzjudgment 2>/dev/null)"
  else
    bad "激光串口" "/dev/dzjudgment 不存在 —— 激光模块连线或 udev 规则有问题"
  fi

  if timeout 8 ros2 node list 2>/dev/null | grep -q "dzjudgment"; then
    SC=$(timeout 8 ros2 topic info /LaserShot_Command 2>/dev/null | grep -i "Subscription count" | grep -oE '[0-9]+')
    if [ "${SC:-0}" -ge 1 ]; then
      ok "狙击链路" "/LaserShot_Command 订阅者 $SC 个（云台指令可达激光模块）"
    else
      bad "狙击链路" "dzjudgment 在跑但没订阅上 —— 检查节点日志"
    fi
  else
    warn "射击驱动未运行" "开火不会有反应；启: ./start_all.sh start 或 tools/restart_node.sh judgment"
  fi

  # 模型文件：det_node 只会加载 share/det_node/model/light_det.rknn
  MODEL="$WS/src/rknn_pt/model/light_det.rknn"
  if [ -f "$MODEL" ]; then
    ok "识别模型" "$(du -h "$MODEL" | cut -f1)  $(stat -c%y "$MODEL" | cut -d. -f1)"
  else
    bad "识别模型" "$MODEL 不存在 —— det_node 会启动失败"
  fi

  # 完成提示音：填了 ID 但很可能没试过，提醒一句
  VID=$(python3 -c "
import yaml
d = yaml.safe_load(open('$CONFIG', encoding='utf-8')) or {}
print((d.get('path_executor') or {}).get('ros__parameters', {}).get('finish_voice_id', 0))
" 2>/dev/null || echo 0)
  if [ "${VID:-0}" -gt 0 ] 2>/dev/null; then
    warn "完成提示音 ID=$VID" "确认已在车上用 tools/test_voice.sh 试听过这句"
  else
    printf "     %-24s %s\n" "完成提示音" "未启用 (finish_voice_id=0)"
  fi
fi

# -----------------------------------------------------------------------------
# 3) 运行时参数核对  ★ 本脚本最有价值的一项
# -----------------------------------------------------------------------------
sec "3/7 运行时参数是否真的设上（防静默失败）"
if [ "$MODE" = "--pre" ]; then
  printf "  (--pre 模式跳过：节点尚未启动)\n"
else
  # ★ 必须在 source 期间临时关闭 nounset：
  #   ROS 的 setup.bash 会展开未定义变量 $AMENT_TRACE_SETUP_FILES，
  #   与本脚本开头的 `set -u` 冲突，会让脚本在此处【静默终止】——
  #   表现为"体检报告只打印到第 3 段就没了"。(start_all.sh 当初踩过同一个坑)
  set +u
  source /opt/ros/humble/setup.bash >/dev/null 2>&1 || true
  source "$WS/install/setup.bash" >/dev/null 2>&1 || true
  set -u
  if ! timeout 8 ros2 node list 2>/dev/null | grep -q "dzrknn_dataprocess"; then
    warn "云台节点未运行" "无法核对运行时参数"
  elif [ ! -f "$CONFIG" ]; then
    warn "无配置中心文件" "跳过参数比对"
  else
    # 一次性 dump 全部参数再比对（逐个 ros2 param get 要起几十个进程，太慢）
    # ★ 同时覆盖云台与视觉两个节点 —— 视觉节点的参数也是新增的，
    #   同样存在"YAML 里写了、节点没声明"的静默失败风险。
    MISM=$(python3 - "$CONFIG" <<'PY'
import subprocess, sys, yaml

NODES = ['/dzrknn_dataprocess', '/det_node']

try:
    d = yaml.safe_load(open(sys.argv[1], encoding='utf-8')) or {}
except Exception as e:
    print('DUMPFAIL|配置读取失败 %s' % e); raise SystemExit(0)

miss, diff, checked, live_nodes = [], [], 0, []
for node in NODES:
    sec = d.get(node)
    if not isinstance(sec, dict):
        continue
    want = sec.get('ros__parameters', {}) or {}
    if not want:
        continue
    try:
        out = subprocess.run(['ros2', 'param', 'dump', node],
                             capture_output=True, text=True, timeout=25).stdout
        have = ((yaml.safe_load(out) or {}).get(node) or {}) \
               .get('ros__parameters', {}) or {}
    except Exception:
        have = {}
    if not have:
        # 节点没在跑：不算失败，但要如实说出来，免得"空绿"
        miss.append('%s  <节点未运行，%d 个参数无从核对>' % (node, len(want)))
        continue
    live_nodes.append(node)
    for k, v in want.items():
        if k not in have:
            miss.append('%s.%s  <节点未声明 → 该参数改了也不会生效>' % (node, k))
            continue
        got = have[k]
        checked += 1
        if isinstance(v, bool):
            if bool(got) != v:
                diff.append('%s.%s: 配置=%s 实际=%s' % (node, k, v, got))
        elif isinstance(v, (int, float)):
            try:
                if abs(float(got) - float(v)) > 1e-6:
                    diff.append('%s.%s: 配置=%s 实际=%s' % (node, k, v, got))
            except (TypeError, ValueError):
                diff.append('%s.%s: 配置=%s 实际=%s' % (node, k, v, got))
        elif isinstance(v, str):
            if str(got) != v:
                diff.append('%s.%s: 配置=%s 实际=%s' % (node, k, v, got))

for m in miss:
    print('MISS|%s' % m)
for x in diff:
    print('DIFF|%s' % x)
print('CHECKED|%d|%s' % (checked, ','.join(live_nodes) or '无'))
PY
)
    if grep -q '^DUMPFAIL|' <<<"$MISM"; then
      bad "参数导出失败" "$(grep '^DUMPFAIL|' <<<"$MISM" | head -1 | cut -d'|' -f2)"
    else
      n_miss=$(grep -c '^MISS|' <<<"$MISM" || true)
      n_diff=$(grep -c '^DIFF|' <<<"$MISM" || true)
      n_chk=$(grep '^CHECKED|' <<<"$MISM" | cut -d'|' -f2)
      if [ "${n_diff:-0}" -eq 0 ] && [ "${n_miss:-0}" -eq 0 ]; then
        ok "全部 ${n_chk:-?} 个参数与配置一致" "$(grep '^CHECKED|' <<<"$MISM" | cut -d'|' -f3)"
      else
        [ "${n_miss:-0}" -gt 0 ] && bad "${n_miss} 个参数未被节点声明" "该参数改了也不会生效"
        [ "${n_diff:-0}" -gt 0 ] && bad "${n_diff} 个参数与配置不符" "见下"
        grep -E '^(MISS|DIFF)\|' <<<"$MISM" | head -14 | sed 's/^/        /'
      fi
    fi
  fi
fi

# -----------------------------------------------------------------------------
# 4) 节点与话题
# -----------------------------------------------------------------------------
sec "4/7 节点与话题"
if [ "$MODE" = "--pre" ]; then
  printf "  (--pre 模式跳过)\n"
else
  NODES=$(timeout 8 ros2 node list 2>/dev/null)
  for n in usb_cam_node det_node dzrknn_dataprocess path_executor_node serial_reader; do
    if grep -q "$n" <<<"$NODES"; then ok "$n" "在线"; else warn "$n" "未在线"; fi
  done
  printf "     ─── 话题速率 ───\n"
  for t in /image_raw /offset_center /odom/localization /pursuitAngle; do
    r=$(timeout 5 ros2 topic hz "$t" 2>/dev/null | grep -oP 'average rate: \K[0-9.]+' | head -1)
    if [ -n "$r" ]; then printf "     \033[32m✅\033[0m %-24s %s Hz\n" "$t" "$r"
    else printf "     \033[33m⚠️ \033[0m %-24s 无数据\n" "$t"; WARN=$((WARN+1)); fi
  done
fi

# -----------------------------------------------------------------------------
# 4b) 新核心 dzbrain（走旧核 / 未安装时自动降级为提示，绝不误报失败）
# -----------------------------------------------------------------------------
#  为什么必须补这一节：本脚本原本**只**体检旧架构（usb_cam/det_node/dzrknn/
#  path_executor/serial_reader + pursuitAngle）。新版决策层 dzbrain（8 个任务线程）
#  上线后，出车前跑一遍旧脚本会拿到"全绿"，而新核心的契约一条都没被检查过 ——
#  最典型的是**双钥匙开火门控**（`/aim/fire_gate` + `/aim/fire_ready` 缺一个就永远
#  打不出去）与**裁判回传真源** `/HpAndHitmsg`（命中即停火 / 弹药对账的空转来源）。
sec "4b/7 新核心 dzbrain（循迹/瞄准/修正/无人机决策层）"
if [ "$MODE" = "--pre" ]; then
  printf "  (--pre 模式跳过)\n"
else
  NODES=${NODES:-$(timeout 8 ros2 node list 2>/dev/null)}
  TOPICS=$(timeout 8 ros2 topic list 2>/dev/null)
  if grep -q dzbrain <<<"$NODES"; then
    ok "dzbrain 节点" "在线（新核心）"
    # ★ pursuitAngle 是底盘的**保活指令**（零 Twist = 刹车，底盘有 1s 看门狗）
    r=$(timeout 5 ros2 topic hz pursuitAngle 2>/dev/null | grep -oP 'average rate: \K[0-9.]+' | head -1)
    if [ -z "$r" ]; then
      bad "pursuitAngle 保活" "无数据 —— 底盘将在 1s 看门狗后停车"
    elif awk -v r="$r" 'BEGIN{exit !(r>=40 && r<=60)}' </dev/null; then
      ok "pursuitAngle 保活" "${r} Hz（保活正常）"
    else
      warn "pursuitAngle 保活" "${r} Hz（期望 ~50Hz —— 过高/过低都会让底盘顿挫）"
    fi
    # ★ 双钥匙开火门控：两个键必须都在，缺一个云台永远不会开火
    for t in /aim/fire_gate /aim/fire_ready; do
      if grep -q "^${t}$" <<<"$TOPICS"; then ok "$t" "已发布"
      else bad "$t" "不存在 —— 双钥匙门控不完整，云台不会开火"; fi
    done
    # ★ 裁判回传真源（命中即停火 / 弹药对账的数据来源，1Hz 心跳）
    r=$(timeout 6 ros2 topic hz /HpAndHitmsg 2>/dev/null | grep -oP 'average rate: \K[0-9.]+' | head -1)
    if [ -n "$r" ]; then
      ok "/HpAndHitmsg 裁判回传" "${r} Hz（命中/弹药闭环真源）"
    else
      warn "/HpAndHitmsg 裁判回传" "无数据 —— 裁判未开启时命中闭环为空转（不上车也有此现象）"
    fi
    # 输入侧：缺哪个都会让对应线程降级
    for t in /odom/localization /offset_center /aim/status /shoot_count; do
      if grep -q "^${t}$" <<<"$TOPICS"; then ok "$t" "存在"
      else warn "$t" "不存在"; fi
    done
  else
    if [ -n "${LEGACY_PATH:-}" ]; then
      warn "dzbrain" "LEGACY_PATH=1 → 本次走旧循迹核（新核心检查跳过，符合预期）"
    elif [ -d "$WS/install/dzbrain" ] || (cd "$WS" 2>/dev/null && python3 -c "import dzbrain" 2>/dev/null); then
      bad "dzbrain" "已安装但节点不在线 —— 新核心没起来（查 start_all.sh 日志）"
    else
      warn "dzbrain" "未安装 —— 出车将走旧循迹核（回退路径）"
    fi
  fi
fi

# -----------------------------------------------------------------------------
# 5) 端口占用
# -----------------------------------------------------------------------------
sec "5/7 端口与残留进程"
if ss -lntp 2>/dev/null | grep -q ':8080'; then
  ok "8080 (网页弹窗)" "已被占用 = 服务在跑"
else
  warn "8080 (网页弹窗)" "空闲 —— cam_viewer 未启动，浏览器将打不开"
fi
TF=$(timeout 6 ros2 node list 2>/dev/null | grep -c static_transform)
if [ "${TF:-0}" -gt 6 ]; then
  warn "static_transform_publisher" "${TF} 个(正常<=6) —— 可能有多套底盘栈残留"
elif [ "${TF:-0}" -gt 0 ]; then
  ok "static_transform_publisher" "${TF} 个"
fi

# -----------------------------------------------------------------------------
# 7) 结论
# -----------------------------------------------------------------------------
printf "\n\033[1m==============================================================\033[0m\n"
if [ "$FAIL" -eq 0 ] && [ "$WARN" -eq 0 ]; then
  printf "  \033[32m\033[1m🎉 全部通过，可以出车\033[0m\n"
  RC=0
elif [ "$FAIL" -eq 0 ]; then
  printf "  \033[33m\033[1m⚠️  ${WARN} 项提醒（非致命），建议确认后再出车\033[0m\n"
  RC=2
else
  printf "  \033[31m\033[1m❌ ${FAIL} 项失败 / ${WARN} 项提醒 —— 请先处理再出车\033[0m\n"
  RC=1
fi
printf "\033[1m==============================================================\033[0m\n\n"
exit $RC
