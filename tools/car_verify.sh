#!/usr/bin/env bash
# 车端只读自检：编译完之后，确认「安装正确、可执行文件在、launch 能解析、导入没问题」。
#
# ⚠️ 全程**不启动任何节点**（不用 ros2 run、不用 ros2 launch 真跑），
#    只做列举与解析，绝不会让车动、让云台转、让串口被占用。
#
# 用法：python tools/remote_run.py --host <车IP> --user duzhong --script tools/car_verify.sh

WS="${AG_WS:-/tmp/ag_ws}"
FAIL=0

hr() { echo "----------------------------------------------------"; }
ok()   { echo "  [OK]   $1"; }
bad()  { echo "  [FAIL] $1"; FAIL=$((FAIL+1)); }

echo "==================== 车端只读自检 ===================="
date
echo

source /opt/ros/humble/setup.bash 2>/dev/null
source "$WS/install/setup.bash" 2>/dev/null || { echo "!! 先编译：tools/car_build.sh"; exit 1; }
echo "工作空间: $WS"
echo

hr; echo "## 1. 包是否注册"
for p in ag_core ag_interfaces ag_perception ag_car ag_mission ag_chassis ag_referee ag_bridge ag_bringup; do
  if ros2 pkg prefix "$p" >/dev/null 2>&1; then ok "$p"; else bad "$p 未注册"; fi
done
echo

hr; echo "## 2. 可执行文件（ros2 pkg executables）"
for p in ag_perception ag_car ag_mission ag_chassis ag_referee ag_bridge; do
  list=$(timeout 30 ros2 pkg executables "$p" 2>&1 | awk -v pkg="$p" '{ for (i=1;i<=NF;i++) if ($i==pkg) { print $(i+1); break } }' | tr '\n' ' ')
  printf "  %-16s %s\n" "$p" "${list:-（空）}"
done
echo "-- 期望的 9 个入口 --"
for e in vision_node intel_fusion_node car_node mission_node target_node \
         chassis_node referee_node air_link_node ground_link_node; do
  if find "$WS/install" -name "$e" -type f 2>/dev/null | grep -q .; then ok "$e"; else bad "$e 缺失"; fi
done
echo

hr; echo "## 3. launch 文件能否解析（--show-args，不会真的启动）"
for l in car air ground sim; do
  f="$WS/install/ag_bringup/share/ag_bringup/launch/$l.launch.py"
  if [ -f "$f" ]; then
    out=$(timeout 60 ros2 launch ag_bringup "$l.launch.py" --show-args 2>&1)
    if echo "$out" | grep -qiE "error|Traceback|not found"; then
      bad "$l.launch.py 解析异常"; echo "$out" | tail -5 | sed 's/^/         /'
    else
      ok "$l.launch.py -> $(echo "$out" | grep -c "'") 个参数"
    fi
  else
    bad "$l.launch.py 未安装到 share"
  fi
done
echo

hr; echo "## 4. Python 导入（纯算法模块 + 消息）"
python3 - <<'PY' || FAIL=$((FAIL+1))
import sys
mods = [
    "ag_core.grid_map", "ag_core.mission_planner", "ag_core.homography",
    "ag_core.dz_referee", "ag_core.referee_protocol", "ag_core.link_protocol",
    "ag_interfaces.msg",
    "ag_car.route_builder", "ag_car.car_node",
    "ag_mission.field_fusion", "ag_perception.intel_accumulator",
]
bad = []
for m in mods:
    try:
        __import__(m)
    except Exception as e:
        bad.append(f"{m}: {type(e).__name__}: {e}")
if bad:
    print("  [FAIL] 导入失败：")
    for b in bad:
        print("        ", b)
    sys.exit(1)
print(f"  [OK]   {len(mods)} 个模块导入成功")
from ag_interfaces.msg import FieldIntelMsg  # noqa
print("  [OK]   自定义消息可用:", FieldIntelMsg().__class__.__name__)
PY
echo

hr; echo "## 5. 参数文件可解析且与车端常量一致"
python3 - <<'PY' || FAIL=$((FAIL+1))
import sys, yaml
p = "/tmp/ag_ws/install/ag_bringup/share/ag_bringup/config/params.yaml"
try:
    cfg = yaml.safe_load(open(p, encoding="utf-8"))
except Exception as e:
    print("  [FAIL] params.yaml:", e); sys.exit(1)
print("  [OK]   params.yaml 解析成功，节点段:", ", ".join(sorted(cfg)))
import re
src = open("/tmp/ag_ws/src/ag_car/ag_car/route_builder.py", encoding="utf-8").read()
car_cols = int(re.search(r"MAP_COLS = (\d+)", src).group(1))
car_rows = int(re.search(r"MAP_ROWS = (\d+)", src).group(1))
for sec in ("mission_node", "car_node", "intel_fusion_node"):
    node_cfg = cfg[sec].get("ros__parameters", cfg[sec])   # ROS2 参数在 ros__parameters 下
    c, r = node_cfg["cols"], node_cfg["rows"]
    if (c, r) != (car_cols, car_rows):
        print(f"  [FAIL] {sec}: {c}x{r} != 车端常量 {car_cols}x{car_rows}"); sys.exit(1)
print(f"  [OK]   场地 {car_cols} 列 x {car_rows} 行，各节点参数一致")
PY
echo

hr; echo "## 6. 结论"
if [ "$FAIL" -eq 0 ]; then
  echo "  全部通过：可以进入上车联调（仍建议先 dry_run:=true）"
else
  echo "  有 $FAIL 项失败，先修再上车"
fi
echo "==================== 结束 ===================="
exit "$FAIL"
