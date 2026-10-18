#!/usr/bin/env python3
"""出厂代码症状诊断：自稳可用 / 高度失控 / 无法解锁 三症状定位。

两个候选根因（修复动作完全不同，先区分）：
  假设A：飞控 EV 融合参数仍是开启状态（之前调好的），但出厂 Orin 代码
         在喂坏 VIO（SLAM 未跑好，z 漂移实测 5~8.9m）→ EKF z 崩 →
         高度模式失控 + pre-flight 拒绝解锁。自稳不读位置所以勉强能用。
  假设B：飞控参数也被重置回出厂（无 RC 解锁被 COM_RCL_EXCEPT/NAV_RCL_CTRL
         默认值拒绝；高度问题另查气压计）。

诊断项（SSH 到 Orin，全部只读，不改任何东西）：
  1) vehicle_status：pre_flight_checks_pass / failure_flags / 解锁拒绝原因
  2) vehicle_local_position：z_valid / xy_valid / z 数值 / reset 风暴
  3) /fmu/in/vehicle_visual_odometry：发布者数 + 采样（是否 NaN/乱跳）
     —— 判定"出厂代码是否正在喂坏 VIO"
  4) vehicle_air_data 气压计高度采样（若 uXRCE 配置暴露该话题）
  5) 板上进程清单（出厂栈/我们的栈/teleop 谁在跑）

用法: set AG_ORIN_PASS=****** 后
    python tools/factory_flight_diag.py --host <IP> [--user duzhong]
"""
from __future__ import annotations

import argparse
import os
import re
import sys
import time
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import remote_run  # noqa: E402

REMOTE_ENV = ("source /opt/ros/humble/setup.bash; "
              "source ~/dzpx4/install/setup.bash 2>/dev/null; ")


def read_password(args) -> str:
    pw = args.password or os.environ.get("AG_ORIN_PASS") or os.environ.get("AG_CAR_PASS")
    if pw:
        return pw
    import getpass
    if not sys.stdin.isatty():
        raise SystemExit("缺少密码：set AG_ORIN_PASS=******（或 --password）")
    return getpass.getpass(f"{args.user}@{args.host} 的密码: ")


def run(host, user, pw, port, cmd, timeout=90, allow_nonzero=False):
    """远程执行一条命令。

    两种"失败"必须分开：
      - SSH 连接异常（连不上/中途断）：重试 3 次后 SystemExit——这才叫"无响应"；
      - 命令非零退出码：诊断命令的非零退出往往正是要诊断的症状
        （`ros2 topic echo --once` 超时=124、`grep` 无匹配=1），调用方传
        allow_nonzero=True 时本函数返回 (code, out) 由各检查项自判 FAIL——
        否则"话题无输出"分支永远不可达（自杀式诊断）。
    """
    for attempt in range(1, 4):
        try:
            code, out, err = remote_run.run(host, user, pw, cmd, timeout=timeout, port=port)
            if code == 0 or allow_nonzero:
                return (code, out) if allow_nonzero else out
        except Exception as e:  # noqa: BLE001
            print(f"[..] 连接重试（{attempt}/3）: {type(e).__name__}")
            time.sleep(5)
    raise SystemExit(f"[失败] 远程命令无响应")


def field(out, name):
    m = re.search(rf"{name}:\s*(\S+)", out)
    return m.group(1) if m else "?"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="出厂代码三症状一键诊断（只读）")
    ap.add_argument("--host", required=True)
    ap.add_argument("--user", default="duzhong")
    ap.add_argument("--password", default=None)
    ap.add_argument("--port", type=int, default=22)
    args = ap.parse_args(argv)
    pw = read_password(args)
    host, user, port = args.host, args.user, args.port

    print("=" * 64)
    print("[1/5] 飞控状态（解锁拒绝原因 + pre-flight）")
    print("=" * 64)
    # topic echo 超时（退出码 124）即"无输出"症状，放行给下方 if 自判 FAIL
    _code, st = run(host, user, pw, port,
                    f"{REMOTE_ENV} timeout 8 ros2 topic echo --once /fmu/out/vehicle_status 2>/dev/null",
                    allow_nonzero=True)
    if "arming_state" in st:
        print(f"  arming_state          = {field(st, 'arming_state')}  (1=disarmed 2=armed)")
        print(f"  nav_state             = {field(st, 'nav_state')}  (14=position 15=altitude 17=stabilized...)")
        pfc = field(st, "pre_flight_checks_pass")
        print(f"  pre_flight_checks_pass = {pfc}  (false=解锁必拒)")
        print(f"  failure_flags         = {field(st, 'failure_flags')}")
        print(f"  latest_disarming_reason = {field(st, 'latest_disarming_reason')}")
        if pfc == "false":
            print("  -> pre-flight 未通过：解锁被拒的直接原因，看 [2][3] 哪项估计坏了")
    else:
        print("  [FAIL] vehicle_status 无输出——uXRCE 链路断（Agent 死了？重启 MicroXRCEAgent）")

    print()
    print("=" * 64)
    print("[2/5] EKF 估计有效性（3s 采样两次）")
    print("=" * 64)
    _code, lp = run(host, user, pw, port,
                    f"{REMOTE_ENV} timeout 8 ros2 topic echo --once /fmu/out/vehicle_local_position 2>/dev/null",
                    allow_nonzero=True)
    if "z_valid" in lp:
        print(f"  第1帧: z={field(lp, 'z')}  z_valid={field(lp, 'z_valid')}  "
              f"xy_valid={field(lp, 'xy_valid')}  xy_reset={field(lp, 'xy_reset_counter')}  "
              f"z_reset={field(lp, 'z_reset_counter')}")
        time.sleep(1)
        lp2_code, lp2 = run(host, user, pw, port,
                            f"{REMOTE_ENV} timeout 8 ros2 topic echo --once /fmu/out/vehicle_local_position 2>/dev/null",
                            allow_nonzero=True)
        print(f"  第2帧: z={field(lp2, 'z')}  xy_reset={field(lp2, 'xy_reset_counter')}")
        z1, z2 = field(lp, "z"), field(lp2, "z")
        try:
            drift = abs(float(z2) - float(z1))
            print(f"  z 漂移(两帧): {drift:.3f} m" + ("  <- 静止时 >0.3m 即高度估计崩" if drift > 0.3 else ""))
        except ValueError:
            pass
        if field(lp, "z_valid") == "false":
            print("  -> z_valid=false：高度模式必然失控，arm 必拒（COM_ARM_EKF_HGT）")
    else:
        print("  [FAIL] local_position 无输出")

    print()
    print("=" * 64)
    print("[3/5] 关键：出厂代码是否在喂 VIO？内容是否正常？")
    print("=" * 64)
    _code, info = run(host, user, pw, port,
                      f"{REMOTE_ENV} ros2 topic info /fmu/in/vehicle_visual_odometry 2>/dev/null | grep 'Publisher count'",
                      allow_nonzero=True)
    m = re.search(r"Publisher count:\s*(\d+)", info)
    n_pub = int(m.group(1)) if m else -1
    print(f"  VIO 发布者数: {n_pub}")
    if n_pub == 0:
        print("  -> 无人发 VIO。若飞控 EV 融合开着：EKF 等视觉超时后回退，属'断粮'（相对安全）")
    elif n_pub >= 1:
        _code, vio = run(host, user, pw, port,
                         f"{REMOTE_ENV} timeout 8 ros2 topic echo --once /fmu/in/vehicle_visual_odometry 2>/dev/null",
                         allow_nonzero=True)
        pos = re.search(r"position:\s*\[([^\]]+)\]", vio)
        q = re.search(r"q:\s*\[([^\]]+)\]", vio)
        rc = field(vio, "reset_counter")
        if pos:
            vals = [v.strip() for v in pos.group(1).split(",")]
            bad = any(v in ("nan", "inf", "-inf") for v in vals)
            print(f"  VIO position = [{', '.join(vals)}]  reset_counter={rc}"
                  + ("  <- NaN/Inf！坏数据正在进 EKF" if bad else ""))
            print(f"  VIO q = [{q.group(1) if q else '?'}]")
        if n_pub > 1:
            print("  -> 双发布者（双实例打架），先跑 check_stack_health.py --kill-dup")

    print()
    print("=" * 64)
    print("[4/5] 气压计原始高度（若话题已暴露）")
    print("=" * 64)
    _code, air = run(host, user, pw, port,
                     f"{REMOTE_ENV} timeout 8 ros2 topic echo --once /fmu/out/vehicle_air_data 2>/dev/null",
                     allow_nonzero=True)
    if "baro_altitude" in air or "pressure" in air:
        print(f"  baro_altitude = {field(air, 'baro_altitude')}  "
              f"temperature = {field(air, 'temperature')}")
        print("  （与 [2] 的 EKF z 对比：若 baro 正常但 EKF z 乱跳 -> EV/视觉污染高度）")
    else:
        print("  话题未暴露（uXRCE yaml 未配置 vehicle_air_data），跳过")

    print()
    print("=" * 64)
    print("[5/5] 板上进程（谁在跑）")
    print("=" * 64)
    ps = run(host, user, pw, port,
             "for p in fastlio_mapping px4_odom_convert livox_ros_driver2 MicroXRCEAgent "
             "px4_teleop px4_control_node; do printf '%-22s: ' $p; pgrep -xc $p 2>/dev/null || echo 0; done")
    print("  " + ps.strip().replace("\n", "\n  "))

    # ---- 判读 ----
    print()
    print("=" * 64)
    print("[判读]")
    print("=" * 64)
    z_valid = field(lp, "z_valid") if "z_valid" in lp else "?"
    if n_pub >= 1 and z_valid == "false":
        print("  符合假设A：VIO 在发布且 EKF z 无效 -> 出厂代码喂坏 VIO 污染高度。")
        print("  修复（二选一）：")
        print("   快速恢复可用（纯气压计，无定位）：QGC 关 EV 融合 EKF2_EV_CTRL=0")
        print("   彻底恢复定位链路：部署 guard_v2_deploy_20261017（出厂栈先停掉）")
    elif n_pub == 0 and field(st, "pre_flight_checks_pass") == "false":
        print("  VIO 无发布但 pre-flight 不过：看 failure_flags 与 QGC Vehicle Messages；")
        print("  高度问题指向气压计/EKF 参数 -> 假设B 方向，查 EKF2_BARO_CTRL / 传感器标定")
    elif field(st, "pre_flight_checks_pass") in ("false", "?"):
        print("  假设B 方向：无 RC 解锁被拒（键盘控制场景）。")
        print("  QGC 检查/设置：COM_RCL_EXCEPT=2（offboard 例外）或 NAV_RCL_CTRL=0（调试用）；")
        print("  COM_ARM_WO_GPS=1（无 GPS 解锁）。改参数前确认场地安全。")
    else:
        print("  症状组合不典型：把上面 [1]-[5] 输出发我，结合 QGC 的解锁报错信息进一步定位。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
