# -*- coding: utf-8 -*-
"""
小车端演示程序(纯标准库)
=========================
连接无人机规划服务, 接收航点 JSON 并模拟行驶:
  - 按 CELL_TIME_S 一格一格前进;
  - 收到 stop_face 航点 -> 停靠蓝色面对面, 车头对准蓝面;
  - 收到 hit 航点 -> 击打蓝色面 3 次;
  - 收到 return 航点 -> 任务完成;
  - 无人机中途重规划会推送新航点, 小车从当前位置自动接上。

用法(先启动无人机端):
  python mission_planner.py --link tcp
  python car_client_demo.py [--host 127.0.0.1] [--port 9600] [--speed 1]
"""

import argparse
import json
import socket
import time

from mission_planner import (Field, Planner, name2rc, rc2name,
                             CELL_TIME_S, FACING_PAUSE_S, FALL_TIMES, FALL_INTERVAL,
                             BUCKET)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=9600)
    ap.add_argument("--speed", type=float, default=1.0, help="演示倍速")
    args = ap.parse_args()

    s = socket.create_connection((args.host, args.port), timeout=5)
    s.settimeout(0.3)
    print(f"[car] 已连接无人机 {args.host}:{args.port}, 等待航点...")

    route, roles, idx = [], {}, 0
    stepped = set()
    buf = b""

    def adopt(plan):
        """采纳新航点: 从当前格接到新目标序列(避开未来目标格, 防止角色乱序)"""
        nonlocal route, roles, idx
        blocked = [name2rc(n) for n in plan.get("blocked", [])]
        f = Field()
        for rc in blocked:
            f.set(rc, BUCKET)          # 绿线/障碍物一律视为禁入
        p = Planner(f)
        path = [name2rc(n) for n in plan["path_cells"]]
        goals = []
        seen_goals = set()
        for pt in plan["points"]:
            rc = name2rc(pt["cell"])
            role = pt.get("role")
            if role in ("stop_face", "hit", "return", "explosive", "bridge") and rc not in seen_goals:
                seen_goals.add(rc)
                goals.append(rc)
        roles = {name2rc(pt["cell"]): pt.get("role") for pt in plan["points"]
                 if pt.get("role")}
        cur = route[idx] if route and idx < len(route) else (path[0] if path else None)
        if cur is None:
            route, idx = path, 0
        else:
            new = [cur]
            ok = True
            for gi, g in enumerate(goals):
                if new[-1] == g:
                    continue
                avoid = {x for x in goals[gi + 1:]}   # 不提前穿过未来目标
                leg = p.a_star(new[-1], g, extra_avoid=avoid)
                if leg is None:
                    leg = p.a_star(new[-1], g)
                if leg is None:
                    ok = False
                    break
                new.extend(leg[1:])
            if not ok or len(new) < 2:
                route, idx = path, 0
            else:
                route, idx = new, 0
        pts = plan["points"]
        names = " -> ".join(q["cell"] + (f"({q['role']})" if q["role"] else "")
                            for q in pts)
        print(f"[car] 收到航点 #{plan['seq']} ({len(pts)}个): {names}")

    def send_status(cell, event=""):
        try:
            s.sendall((json.dumps({"type": "status", "cell": cell, "event": event})
                       + "\n").encode("utf-8"))
        except OSError:
            pass

    while True:
        try:
            data = s.recv(4096)
            if not data:
                print("[car] 无人机离线, 退出")
                break
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                plan = json.loads(line.decode("utf-8"))
                if plan.get("type") == "plan":
                    adopt(plan)
                elif plan.get("type") == "done":
                    print(f"[car] 无人机:{plan.get('message', '任务完成')}, 退出")
                    s.close()
                    return
        except socket.timeout:
            pass
        if route and idx < len(route) - 1:
            time.sleep(CELL_TIME_S / args.speed)
            idx += 1
            cell = route[idx]
            role = roles.get(cell)
            name = rc2name(cell)
            send_status(name, role or "move")
            print(f"[car] 到达 {name}" + (f"  [{role}]" if role else ""))
            if role == "explosive" and cell not in stepped:
                stepped.add(cell)
                print(f"[car] 踩爆炸物 {name}")
            if role == "stop_face":
                time.sleep(FACING_PAUSE_S / args.speed)
                print(f"[car] 停靠蓝色面对面 {name}, 车头对准蓝面")
                # 标靶所占格子无法通行: 停靠位原地对蓝面打击(不驶入标靶)
                for i in range(FALL_TIMES):
                    time.sleep(FALL_INTERVAL / args.speed)
                    print(f"[car] 原地击打蓝色面 {i + 1}/{FALL_TIMES} (不驶入标靶)")
            elif role == "return":
                print("[car] 任务完成, 已到达返程点")
                route, idx = [], 0
        else:
            time.sleep(0.1)
    s.close()


if __name__ == "__main__":
    main()
