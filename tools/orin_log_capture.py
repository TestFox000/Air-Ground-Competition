#!/usr/bin/env python3
"""机载端日志采集器：把 dzuav 的 stdout 转成与车端黑匣子对齐的 JSONL。

为什么需要这个：
  * 车端 integrate_node 用 ag_core.blackbox 写结构化 JSONL，复盘工具 log_report.py 能直接读；
  * 机载端 dzuav 是 C++，只往 stdout 打一行文本（localized=... landmarks=... [...]），
    复盘时无法与车端时间轴对齐，也没法用现成的诊断规则。

本脚本在 **Orin 本地**运行（Python 3，无第三方依赖），包装 dzuav 子进程：
  1. 逐行解析 dzuav stdout，提取 localized / landmarks / objects / car_pose；
  2. 用与车端相同的事件名（vision.detect / intel.recv / link.tx）写 JSONL；
  3. 每条带 ts / mono / seq / node=dzuav，log_report.py 可直接读。

用法（Orin 上）：
    python3 orin_log_capture.py -- dzuav uvc:/dev/video0 hbb.engine cell.engine 50000
    python3 orin_log_capture.py --log-dir /tmp/ag_logs -- dzuav ...
    # 回放已有日志：
    python3 orin_log_capture.py --replay dzuav_stdout.log
"""
from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Optional

# 与车端 ag_core.blackbox.EV 对齐的事件名
EV_VISION = "vision.detect"
EV_INTEL = "intel.recv"
EV_LINK = "link.tx"

# D13 十三类（与 onboard/dzuav main.cpp className 逐字一致，2026-10-16 起）。
# 旧 7 类映射（mine=0/car=1/arrow=2...）在 D13 下全错——car 实为 2 而非 1，
# 且 mine/arrow 两名已删除（都是 paper，落格分流在车端 world_model 侧）。
CLASS_MAP = {
    "paper": 0, "cell_label": 1, "car": 2, "target_red": 3,
    "target_blue": 4, "obstacle": 5, "bucket_red": 6, "bucket_green": 7,
    "bucket_cyan": 8, "car_start": 9, "car_end": 10,
    "uav_start": 11, "uav_end": 12,
}


def _find_matching_bracket(s: str, start: int, open_ch: str, close_ch: str) -> int:
    """从 s[start] 开始找与 open_ch 匹配的 close_ch 下标（支持嵌套）。"""
    depth = 0
    for i in range(start, len(s)):
        if s[i] == open_ch:
            depth += 1
        elif s[i] == close_ch:
            depth -= 1
            if depth == 0:
                return i
    return -1


def parse_line(line: str) -> Optional[dict]:
    """解析 dzuav 的一行 stdout，返回结构化事件；解析失败返回 None。

    行格式（见 onboard/dzuav/src/main.cpp）：
        <output> localized=<0|1> landmarks=<N> [{...},...] [car_pose=(x,y,yaw=z)]
    objects 数组里嵌套了 [x,y]，所以用括号深度计数而不是简单正则。
    """
    m = re.search(r"localized=(\d+)\s+landmarks=(\d+)", line)
    if not m:
        return None
    localized = int(m.group(1)) == 1
    landmarks = int(m.group(2))

    # objects 数组：从 landmarks=N 之后的第一个 [ 开始
    bracket_start = line.find("[", m.end())
    objects = []
    if bracket_start >= 0:
        bracket_end = _find_matching_bracket(line, bracket_start, "[", "]")
        if bracket_end > bracket_start:
            arr_text = line[bracket_start:bracket_end + 1]
            try:
                arr = json.loads(arr_text)
                for o in arr:
                    cls_name = o.get("class", "")
                    center = o.get("center", [0, 0])
                    objects.append({
                        "cls": CLASS_MAP.get(cls_name, -1),
                        "cls_name": cls_name,
                        "x": float(center[0]), "y": float(center[1]),
                        "length": float(o.get("length", 0)),
                        "width": float(o.get("width", 0)),
                    })
            except (json.JSONDecodeError, ValueError, IndexError, TypeError):
                pass

    # car_pose=(x,y,yaw=z)
    car_pose = None
    cp = re.search(r"car_pose=\(([^,]+),([^,]+),yaw=([^)]+)\)", line)
    if cp:
        try:
            car_pose = {"x": float(cp.group(1)), "y": float(cp.group(2)),
                        "yaw": float(cp.group(3))}
        except ValueError:
            pass

    # 按类别计数（D13：雷/箭头同为 paper，落格分流在车端 world_model 侧；
    # 本文件的 mines 字段口径 = paper 总数，含桥洞箭头，机载无格位置信息无法细分）
    counts = {}
    for o in objects:
        counts[o["cls_name"]] = counts.get(o["cls_name"], 0) + 1

    return {
        "localized": localized,
        "landmarks": landmarks,
        "objects": objects,
        "counts": counts,
        "car_pose": car_pose,
    }


def write_event(fh, seq: int, ts: float, ev: str, node: str, session: str, **fields) -> None:
    rec = {
        "ts": round(ts, 3),
        "mono": round(time.monotonic(), 3),
        "seq": seq,
        "node": node,
        "ev": ev,
        "lvl": "info",
        "session": session,
    }
    rec.update(fields)
    fh.write(json.dumps(rec, ensure_ascii=False) + "\n")
    fh.flush()


def run_capture(dzuav_cmd: list[str], log_dir: Path, session: str) -> int:
    log_dir.mkdir(parents=True, exist_ok=True)
    log_file = log_dir / session / f"dzuav-{os.getpid()}.jsonl"
    log_file.parent.mkdir(parents=True, exist_ok=True)

    print(f"[orin_log_capture] 启动 dzuav: {' '.join(dzuav_cmd)}")
    print(f"[orin_log_capture] 日志写入: {log_file}")

    proc = subprocess.Popen(
        dzuav_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        bufsize=1, text=True, encoding="utf-8", errors="replace",
    )

    seq = 0
    with open(log_file, "a", encoding="utf-8", buffering=1) as fh:
        write_event(fh, seq, time.time(), "node.ready", "dzuav", session,
                    pid=os.getpid(), cmd=" ".join(dzuav_cmd), log_file=str(log_file))
        seq += 1
        try:
            for line in proc.stdout:
                line = line.rstrip("\n")
                if not line:
                    continue
                ts = time.time()
                parsed = parse_line(line)
                if parsed is None:
                    # 非结构化行（启动信息、报错等）作为 ros.log 镜像
                    write_event(fh, seq, ts, "ros.log", "dzuav", session,
                                lvl="info", msg=line[:500])
                else:
                    # 一帧写两个事件：vision.detect（检测本身）+ intel.recv（态势广播）
                    write_event(fh, seq, ts, EV_VISION, "dzuav", session,
                                localized=parsed["localized"],
                                landmarks=parsed["landmarks"],
                                counts=parsed["counts"],
                                n_objects=len(parsed["objects"]))
                    seq += 1
                    if parsed["localized"]:
                        write_event(fh, seq, ts, EV_INTEL, "dzuav", session,
                                    ok=True, mines=parsed["counts"].get("paper", 0),
                                    obstacles=parsed["counts"].get("obstacle", 0),
                                    targets=parsed["counts"].get("target_blue", 0)
                                            + parsed["counts"].get("target_red", 0),
                                    car_pose=parsed["car_pose"])
                        seq += 1
                        write_event(fh, seq, ts, EV_LINK, "dzuav", session,
                                    bytes=0, n=len(parsed["objects"]))
                seq += 1
        except KeyboardInterrupt:
            pass
        finally:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()

    print(f"[orin_log_capture] 结束，日志: {log_file}")
    return 0


def replay_log(input_file: Path, log_dir: Path, session: str) -> int:
    """回放已有的 dzuav stdout 日志文件，转成 JSONL（离线复盘用）。"""
    log_dir.mkdir(parents=True, exist_ok=True)
    log_file = log_dir / session / f"dzuav-replay-{os.getpid()}.jsonl"
    log_file.parent.mkdir(parents=True, exist_ok=True)

    print(f"[orin_log_capture] 回放: {input_file} -> {log_file}")
    seq = 0
    base_ts = time.time()
    with open(input_file, "r", encoding="utf-8", errors="replace") as fin, \
         open(log_file, "a", encoding="utf-8", buffering=1) as fh:
        write_event(fh, seq, base_ts, "node.ready", "dzuav", session,
                    pid=os.getpid(), replay=str(input_file), log_file=str(log_file))
        seq += 1
        for i, line in enumerate(fin):
            line = line.rstrip("\n")
            if not line:
                continue
            ts = base_ts + i * 0.033  # 假设 30fps
            parsed = parse_line(line)
            if parsed is None:
                write_event(fh, seq, ts, "ros.log", "dzuav", session,
                            lvl="info", msg=line[:500])
            else:
                write_event(fh, seq, ts, EV_VISION, "dzuav", session,
                            localized=parsed["localized"],
                            landmarks=parsed["landmarks"],
                            counts=parsed["counts"],
                            n_objects=len(parsed["objects"]))
                seq += 1
                if parsed["localized"]:
                    write_event(fh, seq, ts, EV_INTEL, "dzuav", session,
                                ok=True, mines=parsed["counts"].get("paper", 0),
                                obstacles=parsed["counts"].get("obstacle", 0),
                                targets=parsed["counts"].get("target_blue", 0)
                                        + parsed["counts"].get("target_red", 0),
                                car_pose=parsed["car_pose"])
            seq += 1
    print(f"[orin_log_capture] 回放完成，共 {seq} 条事件")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="机载端 dzuav 日志采集器（stdout -> 黑匣子 JSONL）")
    ap.add_argument("--log-dir", type=Path,
                    default=Path(os.environ.get("AG_LOG_DIR", tempfile.gettempdir())) / "ag_logs",
                    help="日志根目录（默认 $AG_LOG_DIR 或 /tmp/ag_logs）")
    ap.add_argument("--session", default=time.strftime("%Y%m%d"),
                    help="会话名（默认当天日期）")
    ap.add_argument("--replay", type=Path, default=None,
                    help="回放已有 dzuav stdout 日志文件")
    ap.add_argument("dzuav_cmd", nargs=argparse.REMAINDER,
                    help="dzuav 命令行（用 -- 分隔，如 -- dzuav uvc:/dev/video0 ...）")
    args = ap.parse_args()

    if args.replay:
        return replay_log(args.replay, args.log_dir, args.session)

    cmd = args.dzuav_cmd
    if cmd and cmd[0] == "--":
        cmd = cmd[1:]
    if not cmd:
        ap.error("需要指定 dzuav 命令（用 -- 分隔），或用 --replay 回放")

    return run_capture(cmd, args.log_dir, args.session)


if __name__ == "__main__":
    raise SystemExit(main())
