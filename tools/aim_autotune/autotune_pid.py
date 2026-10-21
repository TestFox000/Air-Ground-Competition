#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""云台自瞄 PID 自动整定 —— 在车上（Orange Pi + ROS2 Humble）运行。

三步走（与 docs/云台PID自动整定方案.md 对应）：
  P1  id      : 开环辨识，量出 a(px/tick)、伺服时间常数、视觉端到端延迟
  P2  sweep   : 用 P1 结果跑离线模型扫参，缩小候选集（不需要动云台）
  P3  verify  : 逐组上闭环，静止靶阶跃，按指标打分，留下最好的一组

用法（必须先 source 环境）：
  source /opt/ros/humble/setup.bash
  source ~/dzrobot/install/setup.bash
  [ -f ~/dzrobot/dzros.env ] && source ~/dzrobot/dzros.env

  python3 autotune_pid.py id
  python3 autotune_pid.py sweep --a 0.35 --tau 0.08 --delay 0.26
  python3 autotune_pid.py verify --sets "7,0.5,1.0;10,1.0,1.8" --secs 6
  python3 autotune_pid.py report

安全设计：
  * 整定期间把 gimbal.max_shots 强制改 0 —— 无论判据怎么算都打不出去一发
  * 任何异常（超时/视觉断流/|dx|>200/进程死）都走 finally：恢复 yaml + 重启节点
  * 每次试验 <= 12 s，总时长 <= 15 min；结果写 JSON，原始配置另存备份
"""
import argparse
import json
import math
import os
import re
import subprocess
import sys
import time

YAML = os.path.expanduser("~/dzrobot/config/robot.yaml")
LAUNCH = "dzrknn_dataprocess dzrknn_dataprocess.launch.py"
NODE_PAT = "dzrknn_dataprocess"
REPORT = os.path.expanduser("~/robot_logs/pid_autotune.json")

TUNED_KEYS = ("pid_kp", "pid_ki", "pid_kd")


# ---------------------------------------------------------------- yaml 读写
def read_yaml():
    with open(YAML, "r", encoding="utf-8") as f:
        return f.read()


def patch_yaml(**kv):
    """只改第一处 key: value，返回 {key: (old, new)} 用于回滚。"""
    s = read_yaml()
    old = {}
    for k, v in kv.items():
        pat = re.compile(r"^(    " + re.escape(k) + r": )([^\s#]+)", re.M)
        m = pat.search(s)
        if not m:
            raise RuntimeError("yaml 里找不到 %s" % k)
        old[k] = m.group(2)
        s = pat.sub(lambda mm: mm.group(1) + str(v), s, count=1)
    with open(YAML, "w", encoding="utf-8") as f:
        f.write(s)
    return old


def restore_yaml(old):
    if old:
        patch_yaml(**old)


# ---------------------------------------------------------------- 进程控制
def kill_node():
    subprocess.run(["pkill", "-f", NODE_PAT], check=False)
    time.sleep(2.0)


def start_node(wait=8.0):
    subprocess.Popen(
        ["bash", "-lc",
         "cd ~/dzrobot && source /opt/ros/humble/setup.bash && source install/setup.bash "
         "&& ([ -f ./dzros.env ] && source ./dzros.env || true) "
         "&& nohup ros2 launch " + LAUNCH + " > ~/robot_logs/kb11_5_gimbal.log 2>&1 &"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(wait)


def node_alive():
    r = subprocess.run(["pgrep", "-f", NODE_PAT], capture_output=True, text=True)
    return bool(r.stdout.strip())

# ---------------------------------------------------------------- ROS 采样
class Recorder:
    """订阅 offset_center / aim_status / camemo_feedback，按时间戳存曲线。"""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Int32MultiArray, Int32, UInt8
        self.rclpy = rclpy
        self.UInt8 = UInt8
        self.Multi = Int32MultiArray
        rclpy.init()
        self.n = Node("pid_autotune")
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.t0 = time.time()
        self.dx, self.dy, self.status, self.pos = [], [], [], []
        self.n.create_subscription(Int32MultiArray, "offset_center", self._on_off, q)
        self.n.create_subscription(Int32, "/aim/status", self._on_st, 10)
        self.n.create_subscription(Int32MultiArray, "camemo_feedback", self._on_fb, q)
        self.scan_pub = self.n.create_publisher(UInt8, "/nav/scan_request", 10)
        self.cmd_pub = self.n.create_publisher(Int32MultiArray, "CameMo_Conl", 10)

    def _on_off(self, m):
        if len(m.data) >= 3:
            self.dx.append((time.time() - self.t0, int(m.data[0]), int(m.data[2])))
            self.dy.append((time.time() - self.t0, int(m.data[1])))

    def _on_st(self, m):
        self.status.append((time.time() - self.t0, int(m.data)))

    def _on_fb(self, m):
        if len(m.data) >= 4:
            self.pos.append((time.time() - self.t0, int(m.data[0]), int(m.data[1])))

    def spin(self, secs):
        t_end = time.time() + secs
        while time.time() < t_end:
            self.rclpy.spin_once(self.n, timeout_sec=0.02)

    def send_scan(self, n=5):
        self.spin(0.2)
        for _ in range(n):
            self.scan_pub.publish(self.UInt8(data=2))
            self.spin(0.12)
        self.spin(0.8)

    def cmd(self, yaw, pitch, sy, sp):
        m = self.Multi()
        m.data = [int(yaw), int(pitch), int(sy), int(sp)]
        self.cmd_pub.publish(m)

    def close(self):
        try:
            self.n.destroy_node()
            self.rclpy.shutdown()
        except Exception:
            pass


# ---------------------------------------------------------------- P1 辨识
def identify(args):
    rec = Recorder()
    try:
        rec.spin(0.5)
        if not rec.pos:
            raise RuntimeError("没有 camemo_feedback，云台下位机不在线？")
        yaw0, pitch0 = rec.pos[-1][1], rec.pos[-1][2]
        print("[ID] 初始刻度 yaw=%d pitch=%d" % (yaw0, pitch0))
        kill_node()
        for d in args.steps:
            for _ in range(40):
                rec.cmd(yaw0, pitch0, 400, 400)
                rec.spin(0.05)
            rec.spin(0.6)
            mark = len(rec.dx)
            for _ in range(60):
                rec.cmd(yaw0 + d, pitch0, 400, 400)
                rec.spin(0.05)
            rec.spin(1.2)
            seg = rec.dx[mark:]
            if len(seg) > 10:
                before = seg[0][1]
                after = sum(v for _, v in seg[-8:]) / 8.0
                print("[ID] step %+d tick: dx %.1f -> %.1f (delta=%.1f px, a=%.3f px/tick)"
                      % (d, before, after, after - before, (after - before) / d))
            for _ in range(40):
                rec.cmd(yaw0, pitch0, 400, 400)
                rec.spin(0.05)
        return 0
    finally:
        start_node()
        rec.close()
        print("[ID] 自瞄节点已恢复")

# ---------------------------------------------------------------- P3 闭环验证
def compute_metrics(t, x):
    tol, hold = 5.0, 0.5
    gaps = sorted(t[i + 1] - t[i] for i in range(len(t) - 1))
    dt = gaps[len(gaps) // 2] if gaps else 0.0333
    win = max(1, int(hold / dt))
    settle = None
    for i in range(len(x)):
        if i + win <= len(x) and all(abs(v) <= tol for v in x[i:i + win]):
            settle = t[i]
            break
    x0 = x[0]
    s0 = 1.0 if x0 >= 0 else -1.0
    over, crossed = 0.0, False
    for v in x:
        if v * s0 < 0:
            crossed = True
        if crossed:
            over = max(over, -v * s0)
    over_pct = 100.0 * over / max(abs(x0), 1e-6)
    iae = sum(abs(v) for v in x) * dt
    zc, prev = 0, x[0]
    for tt, v in zip(t, x):
        if tt > 0.3 and prev != 0 and v * prev < 0:
            zc += 1
        prev = v
    tail = x[-int(2.0 / dt):]
    rms = math.sqrt(sum(v * v for v in tail) / max(len(tail), 1))
    return dict(settle=settle, overshoot_pct=over_pct, iae=iae, zero_cross=zc, rms_tail=rms)


def one_trial(kp, ki, kd, secs):
    old = patch_yaml(pid_kp=kp, pid_ki=ki, pid_kd=kd, max_shots=0)
    rec = None
    try:
        kill_node()
        start_node()
        if not node_alive():
            raise RuntimeError("节点没起来")
        rec = Recorder()
        rec.spin(0.5)
        locked = False
        for _attempt in range(3):
            rec.send_scan()
            t_end = time.time() + 6.0
            while time.time() < t_end:
                rec.spin(0.1)
                if rec.status and rec.status[-1][1] == 2:
                    locked = True
                    break
                if not rec.status and sum(1 for _, v, _c in rec.dx if v != 0) > 10:
                    locked = True  # status 通道异常时兜底
                    break
            if locked:
                break
        if not locked:
            return dict(ok=False, why="未进入 TRACK（靶不在视野/扫描没触发）")
        rec.spin(0.3)
        mark = len(rec.dx)
        rec.spin(secs)
        raw = rec.dx[mark:]
        valid = [(tt, v) for (tt, v, c) in raw if c > 0]
        if len(valid) < 20:
            return dict(ok=False, why="有效视觉帧太少(%d/%d)" % (len(valid), len(raw)))
        t = [x[0] - valid[0][0] for x in valid]
        seg = [x[1] for x in valid]
        m = compute_metrics(t, seg)
        m.update(ok=True, n=len(seg), kp=kp, ki=ki, kd=kd,
                 det_ratio=round(len(valid) / float(max(len(raw), 1)), 2))
        return m
    finally:
        if rec:
            rec.close()
        restore_yaml(old)


def verify(args):
    sets = []
    for grp in args.sets.split(";"):
        parts = [float(v) for v in grp.split(",")]
        if len(parts) == 3:
            sets.append(parts)
    rows = load_report()
    for kp, ki, kd in sets:
        print("[VERIFY] kp=%.2f ki=%.2f kd=%.2f" % (kp, ki, kd))
        m = one_trial(kp, ki, kd, args.secs)
        print("   ", json.dumps(m, ensure_ascii=False))
        rows.append(m)
        save_report(rows)
    return 0


def load_report():
    if os.path.exists(REPORT):
        with open(REPORT, "r", encoding="utf-8") as f:
            return json.load(f)
    return []


def save_report(rows):
    os.makedirs(os.path.dirname(REPORT), exist_ok=True)
    with open(REPORT, "w", encoding="utf-8") as f:
        json.dump(rows, f, ensure_ascii=False, indent=2)


def report(_args):
    rows = [r for r in load_report() if r.get("ok")]
    rows.sort(key=lambda r: (r["settle"] if r["settle"] is not None else 99))
    print("%-18s %7s %7s %7s %6s %6s" % ("kp,ki,kd", "settle", "over%", "IAE", "ZC", "rms"))
    for r in rows:
        print("%-18s %7s %7.1f %7.2f %6d %6.2f" % (
            "%.2f,%.2f,%.2f" % (r["kp"], r["ki"], r["kd"]),
            ("%.2f" % r["settle"]) if r["settle"] is not None else "none",
            r["overshoot_pct"], r["iae"], r["zero_cross"], r["rms_tail"]))
    return 0


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    p1 = sub.add_parser("id")
    p1.add_argument("--steps", type=int, nargs="+", default=[60, 120, -60, -120])
    p2 = sub.add_parser("sweep")
    p2.add_argument("--a", type=float, default=0.35)
    p2.add_argument("--tau", type=float, default=0.08)
    p2.add_argument("--delay", type=float, default=0.26)
    p2.add_argument("--dx0", type=float, default=60.0)
    p3 = sub.add_parser("verify")
    p3.add_argument("--sets", type=str, required=True)
    p3.add_argument("--secs", type=float, default=6.0)
    sub.add_parser("report")
    a = ap.parse_args()

    if a.cmd == "id":
        return identify(a)
    if a.cmd == "sweep":
        from plant_model import Plant, sweep, kp_stability_bound
        p = Plant(a=a.a, tau_servo=a.tau, delay=a.delay)
        print("Kp 稳定上界 %.2f" % kp_stability_bound(p))
        for sc, kp, ki, kd, m in sweep(p, a.dx0)[:10]:
            st = ("%.2f" % m["settle"]) if m["settle"] is not None else ">5"
            print("%-16s settle=%-5s over=%5.1f iae=%6.2f zc=%d score=%.2f" % (
                "%.2f,%.2f,%.2f" % (kp, ki, kd), st, m["overshoot_pct"],
                m["iae"], m["zero_cross"], sc))
        return 0
    if a.cmd == "verify":
        return verify(a)
    return report(a)


if __name__ == "__main__":
    sys.exit(main())
