#!/usr/bin/env bash
# 契约交叉校验：把「我们代码里的假设」与「车端源码里的常量」逐项比对。
#
# 为什么需要：场地尺寸、弹药、停靠等待、类别数这些常量散落在车端源码里，
# 车端一旦被人改动（公用小车，别人也会改），我们的规划就会静默失准。
# 这个脚本只读车端源码 + 读我们自己的源码，比对不一致就报 FAIL。
#
# 用法：python tools/remote_run.py --host <IP> --user duzhong --script tools/car_xcheck_constants.sh

WS="${AG_WS:-/tmp/ag_ws}"
VENDOR="$HOME/dzrobot/src"

echo "==================== 车端常量契约校验 ===================="
echo "车端源码: $VENDOR"
echo "我们的代码: $WS/src"
echo

python3 - "$VENDOR" "$WS/src" <<'PY'
import os
import re
import sys
import yaml

vendor, ours = sys.argv[1], sys.argv[2]
fails = []

SKIP_DIRS = {"build", "install", "log", ".git", "navigation2",
             "robot_localization-humble-devel", "rplidar_ros-ros2",
             "slam_gmapping-eloquent-devel", "rviz"}


def locate(roots, filename):
    """在车端源码里按文件名查找（不硬编码路径，车端目录结构变过）。"""
    for root in roots:
        for dirpath, dirnames, filenames in os.walk(root):
            dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
            if filename in filenames:
                return os.path.join(dirpath, filename)
    return None


def read(path):
    if path is None:
        return "<<not found>>"
    try:
        return open(path, encoding="utf-8", errors="replace").read()
    except OSError as e:
        return f"<<missing: {e}>>"


def grab(text, pattern, cast=str, default=None):
    m = re.search(pattern, text)
    if not m:
        return default
    try:
        return cast(m.group(1))
    except (TypeError, ValueError):
        return default


PE_PATH = locate([f"{vendor}/car_localization", vendor], "path_executor.py")
POST_PATH = locate([f"{vendor}/rknn_pt"], "postprocess.h")
DZRKNN_PATH = locate([f"{vendor}/dzrknn_dataprocess"], "dzrknn_dataprocess.cpp")
HDR_PATH = locate([f"{vendor}/dzrknn_dataprocess"], "dzrknn_dataprocess.h")

for label, p in (("path_executor.py", PE_PATH), ("postprocess.h", POST_PATH),
                 ("dzrknn_dataprocess.cpp", DZRKNN_PATH),
                 ("dzrknn_dataprocess.h", HDR_PATH)):
    print(f"  [定位] {label}: {p or '未找到'}")
print()

PE = read(PE_PATH)
POST = read(POST_PATH)
DZRKNN = read(DZRKNN_PATH) + "\n" + read(HDR_PATH)
RB = read(f"{ours}/ag_car/ag_car/route_builder.py")
PARAM_PATH = f"{ours}/ag_bringup/config/params.yaml"
try:
    params = yaml.safe_load(open(PARAM_PATH, encoding="utf-8"))
except Exception as e:                                    # noqa: BLE001
    print(f"  [FAIL] 读不到 {PARAM_PATH}: {e}")
    sys.exit(1)

cfg = {k: v.get("ros__parameters", v) for k, v in params.items()}

checks = []   # (说明, 车端值, 我们的值, 比较方式 eq|le)

# ---- 场地几何 ----
checks.append(("场地列数 MAP_COLS",
               grab(PE, r"MAP_COLS\s*=\s*(\d+)", int),
               grab(RB, r"MAP_COLS\s*=\s*(\d+)", int)))
checks.append(("场地行数 MAP_ROWS",
               grab(PE, r"MAP_ROWS\s*=\s*(\d+)", int),
               grab(RB, r"MAP_ROWS\s*=\s*(\d+)", int)))
checks.append(("单元边长 BLOCK_SIZE",
               grab(PE, r"BLOCK_SIZE\s*=\s*([\d.]+)", float),
               grab(RB, r"BLOCK_SIZE\s*=\s*([\d.]+)", float)))
checks.append(("列字母表 COL_LETTERS",
               grab(PE, r"COL_LETTERS\s*=\s*'([A-Z]+)'", str),
               grab(RB, r'COL_LETTERS\s*=\s*"([A-Z]+)"', str)))
for sec in ("mission_node", "car_node", "intel_fusion_node"):
    c = cfg.get(sec, {})
    checks.append((f"{sec}.cols", grab(PE, r"MAP_COLS\s*=\s*(\d+)", int), c.get("cols")))
    checks.append((f"{sec}.rows", grab(PE, r"MAP_ROWS\s*=\s*(\d+)", int), c.get("rows")))
    checks.append((f"{sec}.cell_size", grab(PE, r"BLOCK_SIZE\s*=\s*([\d.]+)", float),
                   c.get("cell_size")))

# ---- 打击停靠 ----
car_wait = cfg.get("car_node", {}).get("car_shoot_wait_timeout_s")
checks.append(("打击停靠超时 SHOOT_WAIT_TIMEOUT",
               grab(PE, r"SHOOT_WAIT_TIMEOUT\s*=\s*([\d.]+)", float), float(car_wait), "eq"))

# 弹药与停靠点不是同一个单位，比的是「能撑几个停靠点」：
#   每个 '*' 停靠点会打满 SHOOT_COUNT_TARGET 发，全程弹药只有 AIM_MAX_SHOTS 发。
#   我们的 max_shoot_stops 必须 ≤ 这个上限（保守是允许的）。
shots = grab(DZRKNN, r"AIM_MAX_SHOTS\s*=\s*(\d+)", int)
per_stop = grab(PE, r"SHOOT_COUNT_TARGET\s*=\s*(\d+)", int)
allowed = (shots // per_stop) if (shots and per_stop) else None
checks.append((f"停靠点数上限(弹药{shots}/每次{per_stop})",
               allowed, cfg.get("car_node", {}).get("max_shoot_stops"), "le"))
# 车端超时是上限，我们的等待预算是我们自愿等的时间 → 必须 ≤ 上限
checks.append(("等待预算 ≤ 车端超时",
               car_wait, cfg.get("car_node", {}).get("shoot_wait_budget_s"), "le"))

# ---- 视觉契约 ----
checks.append(("类别数 OBJ_CLASS_NUM",
               grab(POST, r"#define\s+OBJ_CLASS_NUM\s+(\d+)", int),
               cfg.get("vision_node", {}).get("nc")))
checks.append(("关键点数 KPT_NUM",
               grab(POST, r"#define\s+KPT_NUM\s+(\d+)", int),
               cfg.get("vision_node", {}).get("nkpt")))

print(f"{'项目':<34}{'车端/上限':<12}{'我们':<12}结果")
print("-" * 74)
for item in checks:
    name, v, o = item[0], item[1], item[2]
    mode = item[3] if len(item) > 3 else "eq"
    if v is None or o is None:
        print(f"{name:<34}{str(v):<12}{str(o):<12}?? 有一边读不到")
        continue
    good = (v == o) if mode == "eq" else (o <= v)
    label = "一致" if (mode == "eq" and good) else ("满足" if good else "不满足 <<<")
    print(f"{name:<34}{str(v):<12}{str(o):<12}{label}")
    if not good:
        fails.append(f"{name}: 车端上限={v} 我们={o}（要求{'相等' if mode == 'eq' else '≤'}）")

print()
if fails:
    print(f"[FAIL] {len(fails)} 项不一致，上车前必须先对齐：")
    for f in fails:
        print("   -", f)
    sys.exit(1)
print("[OK] 所有常量一致 —— 我们的假设与车端源码完全对齐")
PY
echo "==================== 结束 ===================="
