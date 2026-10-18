# -*- coding: utf-8 -*-
"""实时场地矩阵态势工具（教员建议的矩阵形式输出）。

监听机载 UDP :50000 态势帧（0x4001），把爆炸物/障碍物/靶等识别结果
落到 11×14 场地格矩阵并打印精确世界坐标。不依赖 ROS，方便联调演示。

用法：
    python tools/matrix_report.py                # 实时清屏刷新
    python tools/matrix_report.py --once         # 收一帧打印后退出
    python tools/matrix_report.py --port 50000   # 指定端口
    python tools/matrix_report.py --json out.json  # 末次结果另存 JSON
"""
from __future__ import annotations

import argparse
import json
import socket
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

try:  # src/ 外层布局（直跑 python tools/matrix_report.py，本文件上方注入的路径）
    from ag_integrate.ag_integrate.field_matrix import (  # noqa: E402
        KIND_NAMES, render_report, spots_from_frame,
    )
    from ag_integrate.ag_integrate.intel_protocol import (  # noqa: E402
        detect_msg_id, parse,
    )
except ImportError:  # src/ag_integrate 内层布局（conftest 注入 / ROS 安装形态）
    from ag_integrate.field_matrix import (  # noqa: E402
        KIND_NAMES, render_report, spots_from_frame,
    )
    from ag_integrate.intel_protocol import (  # noqa: E402
        detect_msg_id, parse,
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="实时场地矩阵态势")
    ap.add_argument("--port", type=int, default=50000)
    ap.add_argument("--once", action="store_true", help="收一帧即退出")
    ap.add_argument("--json", type=str, default="",
                    help="退出时把末次结果写入该 JSON 文件")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", args.port))
    sock.settimeout(2.0)
    print(f"监听 UDP:{args.port}，等待机载态势帧……（Ctrl+C 退出）")

    last_spots: list = []
    while True:
        try:
            data, _ = sock.recvfrom(4096)
        except socket.timeout:
            continue
        except KeyboardInterrupt:
            break
        if detect_msg_id(data) != 0x4001:
            continue  # 0x4002 车位姿帧与本工具无关
        try:
            frame = parse(data)
        except ValueError as exc:
            print(f"丢弃坏帧: {exc}")
            continue
        spots = spots_from_frame(frame)
        last_spots = spots
        text = render_report(frame)
        if args.once:
            print(text)
            break
        sys.stdout.write("\x1b[2J\x1b[H" + text + "\n")  # 清屏重绘
        sys.stdout.flush()

    sock.close()
    if args.json and last_spots:
        payload = {
            "timestamp_ms": frame.timestamp_ms,
            "spots": [
                {"kind": KIND_NAMES[s.kind], "cell": s.cell,
                 "x_m": round(s.x, 3), "y_m": round(s.y, 3)}
                for s in last_spots
            ],
        }
        Path(args.json).write_text(
            json.dumps(payload, ensure_ascii=False, indent=2),
            encoding="utf-8")
        print(f"已写入 {args.json}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(0)
