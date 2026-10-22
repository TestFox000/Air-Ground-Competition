# -*- coding: utf-8 -*-
"""机载自主规划 · 一键跑道与评测  air_plan_bench.py

背景（依据官方文档，勿改口径）
--------------------------------
`docs/raw_txt/02_15空地协同侦排挑战赛.txt` 第 2 页「控制要求」原文：
    "比赛过程中，无人机的定位、建图、识别、规划与控制等实时任务
      须由机载平台自主完成"
`docs/无人机工作流程.md` 第 0 节职责边界：
    飞的部分 → 官方 dzpx4（px4_control / px4_path 执行航线，我们不碰）
    看的部分 → 我们的 dzuav（相机→检测→PnP 定位→UDP/规划）
    车的部分 → 官方 dzrobot，我们只做薄集成

当前无人机不能起飞，因此以「视频输入 → 机载自主跑完整条
检测→定位→落格→规划→落盘」作为等价验收手段。

本脚本一条命令做两件事
--------------------------------
  ① 一键跑道：把指定视频喂给机上侦察栈，让它自主处理并产出规划结果
     （route_plan.json / planned_path.txt / cell_matrix.json / 运行日志）
  ② 一键评测：按「时间」与「稳定性」两套口径采集指标并生成报告

机上两套候选栈（实测后按结果定用哪套，本脚本两套都支持）
--------------------------------
  planner7 : ~/dzuav/build/dzuav
             7 类血缘，自带 CellGrid + RoutePlanner，能直接出规划结果
             CLI: <image|video> <hbb_engine> <cell_engine> [udp_port]
                  [output_dir] [matrix_json] [route_json] [path_file]
  mjpg13   : ~/dzuav_deploy/air_ground/onboard/dzuav_mjpg/build/dzuav
             13 类 canonical 血缘，只有 检测+PnP+UDP，没有落格/规划层
             CLI: <image|video> <hbb_engine> <cell_engine> [udp_port] [output_dir]

用法（在 Windows 本机执行；密码走 AG_ORIN_PASS 环境变量或 --password）
--------------------------------
  python air_plan_bench.py --stack both --runs 3        # 两套栈各跑 3 轮
  python air_plan_bench.py --stack planner7 --runs 1    # 只跑带规划那套
  python air_plan_bench.py --list-videos                # 只列机上可用视频
  python air_plan_bench.py --dry-run                    # 不通硬件，只打印将要执行的命令

指标口径（本脚本定稿）
--------------------------------
  时间  : t_wall 端到端墙钟秒 / fps_eff 等效帧率 / t_first_plan 首份规划产出时延秒
          / per_frame_ms 单帧均摊毫秒 / n_plan_updates 规划触发次数
          / frames_processed 处理帧数 / localized_ratio 定位成功帧占比
  稳定性: exit_all_zero 全轮退出码为 0 / route_consistent 跨轮路线文本一致
          / peak_rss_kb 峰值常驻内存 / peak_gpu_mem_mb GPU 峰值显存（tegrastats 可用时）
          / crash_keywords 崩溃与异常关键字命中 / runs_failed 失败轮数

不污染原机约定
--------------------------------
  * 所有机上产物只落 /tmp/ag_bench_<时间戳>/，不改动任何源文件、引擎、配置；
  * 跑完可加 --clean 一键清理远端临时目录；
  * 默认带 --ensure-link：无人机 USB-RNDIS 链路 DHCP 租约丢失时自动续租（本机侧）。

依赖: pip install paramiko
"""
from __future__ import annotations

import argparse
import datetime as _dt
import json
import os
import re
import subprocess
import time
from pathlib import Path

try:
    import paramiko
except ImportError:  # pragma: no cover
    raise SystemExit("缺少依赖：pip install paramiko")

# ---------------------------------------------------------------- 常量

# 电脑端侦察用的那批视频（机上 ~/Videos/recordings/ 内），实测同一批才可比
DEFAULT_VIDEOS = [
    "cam_20261017_205245.mp4",
    "cam_20261017_205411.mp4",
    "cam_20261017_205906.mp4",
    "cam_20261018_091746.mp4",
]
VIDEO_DIR = "/home/duzhong/Videos/recordings"   # 绝对路径：~ 在引号内不会展开

STACKS = {
    "planner7": {
        "bin": "~/dzuav/build/dzuav",
        "hbb": "~/dzuav/engines/hbb_detector_fp16.engine",
        "cell": "~/dzuav/engines/cell_classifier_fp16.engine",
        # 视频模式: <video> <hbb> <cell> <port> <output_dir(空)> <matrix> <route> <path>
        "args": "{video} {hbb} {cell} 50000 \"\" {matrix} {route} {path}",
    },
    "mjpg13": {
        "bin": "~/dzuav_deploy/air_ground/onboard/dzuav_mjpg/build/dzuav",
        "hbb": "~/dzuav/engines/hbb_detector_d13_fp16.engine",
        "cell": "~/dzuav/engines/cell_classifier_v4d_fp16.engine",
        # 2026-10-22 起已接入规划层：<video> <hbb> <cell> <port> [out] [matrix] [route] [path]
        "args": "{video} {hbb} {cell} 50000 \"\" {matrix} {route} {path}",
    },
}

# 运行时脚本（推到机上，负责：计时、首份规划时延、退出码）
RUNNER_SH = r"""#!/usr/bin/env bash
# 机载跑道计时器（由 air_plan_bench.py 推送；只读不改任何源文件）
set -u
START=$(date +%s.%N)
LOG="$1"; shift
FIRST_FLAG="$1"; shift
ROUTE_JSON="$1"; shift
RC_FILE="$1"; shift
"$@" > "$LOG" 2>&1 &
PID=$!
while kill -0 "$PID" 2>/dev/null; do
  if [ ! -f "$FIRST_FLAG" ] && [ -f "$ROUTE_JSON" ]; then
    date +%s.%N > "$FIRST_FLAG"
  fi
  sleep 0.2
done
wait "$PID"; echo $? > "$RC_FILE"
END=$(date +%s.%N)
echo "$START $END" > "${RC_FILE}.time"
"""

CRASH_PAT = re.compile(
    r"规划失败|cannot|Cannot|error|Error|ERROR|terminate|Segmentation|abort|"
    r"core dumped|not found|失败")


# ---------------------------------------------------------------- 本机链路

def _ps(cmd: str) -> str:
    try:
        return subprocess.run(["powershell", "-NoProfile", "-Command", cmd],
                              capture_output=True, text=True, timeout=90).stdout
    except Exception:  # noqa: BLE001
        return ""


def ensure_link(verbose: bool = True) -> bool:
    """确保本机在 192.168.55.0/24 内有地址（无人机 USB-RNDIS 链路）。

    链路 DHCP 租约会间歇性丢失（实测），丢失时 SSH 直接超时。
    做法：找不到地址时，对候选 RNDIS/NCM 网卡逐个 release+renew。
    """
    for attempt in range(1, 4):
        addr = _ps("(Get-NetIPAddress -AddressFamily IPv4 | "
                   "Where-Object {$_.IPAddress -like '192.168.55.*'} | "
                   "Select-Object -First 1 -ExpandProperty IPAddress)").strip()
        if addr.startswith("192.168.55."):
            if verbose:
                print(f"[link] 本机地址 {addr}")
            return True
        if verbose:
            print(f"[link] 无 192.168.55.x 地址 → 续租（第 {attempt} 次）")
        names = _ps("(Get-NetAdapter | Where-Object {$_.Status -eq 'Up' -and "
                    "($_.InterfaceDescription -like '*NDIS*' -or "
                    "$_.InterfaceDescription -like '*Ncm*')} | "
                    "Select-Object -ExpandProperty Name) -join '|'").strip()
        for name in [n for n in names.split("|") if n.strip()]:
            _ps(f"ipconfig /release '{name}' | Out-Null")
            time.sleep(1)
            _ps(f"ipconfig /renew '{name}' | Out-Null")
            time.sleep(3)
            addr = _ps("(Get-NetIPAddress -AddressFamily IPv4 | "
                       "Where-Object {$_.IPAddress -like '192.168.55.*'} | "
                       "Select-Object -First 1 -ExpandProperty IPAddress)").strip()
            if addr.startswith("192.168.55."):
                if verbose:
                    print(f"[link] 续租成功 {addr}（网卡 {name}）")
                return True
    print("[link] 链路不可用：请确认无人机 USB 已连接 / 供电正常")
    return False


# ---------------------------------------------------------------- SSH

class Remote:
    def __init__(self, host, user, pw, port=22, retries=3):
        self.host, self.user, self.pw, self.port, self.retries = host, user, pw, port, retries
        self.cl = None

    def connect(self):
        last = None
        for i in range(1, self.retries + 1):
            try:
                cl = paramiko.SSHClient()
                cl.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                cl.connect(self.host, username=self.user, password=self.pw,
                           timeout=12, banner_timeout=20, auth_timeout=20)
                self.cl = cl
                return self
            except Exception as ex:  # noqa: BLE001
                last = type(ex).__name__
                print(f"[ssh] {self.host} 连接失败（{i}/{self.retries}）：{last}")
                time.sleep(4)
        raise SystemExit(f"[失败] 无法连接 {self.host}：{last}")

    def run(self, cmd, timeout=60):
        """返回 (rc, stdout+stderr 合并文本)。"""
        assert self.cl, "未连接"
        _i, o, e = self.cl.exec_command(cmd, timeout=timeout)
        out = o.read().decode(errors="replace") + e.read().decode(errors="replace")
        rc = o.channel.recv_exit_status()
        return rc, out.strip()

    def put_text(self, text, remote_path):
        sftp = self.cl.open_sftp()
        try:
            with sftp.file(remote_path, "w") as f:
                f.write(text)
        finally:
            sftp.close()

    def get(self, remote_path, local_path):
        sftp = self.cl.open_sftp()
        try:
            local_path.parent.mkdir(parents=True, exist_ok=True)
            sftp.get(remote_path, str(local_path))
            return True
        except Exception:  # noqa: BLE001
            return False
        finally:
            sftp.close()

    def close(self):
        try:
            self.cl.close()
        except Exception:  # noqa: BLE001
            pass


def read_password(args) -> str:
    pw = args.password or os.environ.get("AG_ORIN_PASS") or os.environ.get("AG_CAR_PASS")
    if pw:
        return pw
    raise SystemExit("缺少密码：set AG_ORIN_PASS=<密码>（或 --password）")


# ---------------------------------------------------------------- 机上探查

def list_videos(r: Remote):
    rc, out = r.run(f"ls -1 {VIDEO_DIR}/*.mp4 2>/dev/null | head -40")
    vids = [os.path.basename(p) for p in out.splitlines() if p.strip()]
    print("[videos] 机上可用视频：")
    for v in vids:
        mark = "  ← 电脑端同批" if v in DEFAULT_VIDEOS else ""
        print(f"   - {v}{mark}")
    return vids


def probe(r: Remote, stack: str) -> dict:
    """跑道前置体检：二进制、引擎、视频、tegrastats 是否齐备。"""
    s = STACKS[stack]
    chk = {
        "bin": s["bin"], "hbb": s["hbb"], "cell": s["cell"],
        "video_dir": VIDEO_DIR,
    }
    cmd = "; ".join(
        [f"test -x {s['bin']} && echo BIN_OK || echo BIN_MISSING",
         f"test -f {s['hbb']} && echo HBB_OK || echo HBB_MISSING",
         f"test -f {s['cell']} && echo CELL_OK || echo CELL_MISSING",
         "command -v tegrastats >/dev/null && echo TEGRA_OK || echo TEGRA_MISSING",
         "stat -c '%y' " + s["bin"]])
    _rc, out = r.run(cmd)
    lines = out.splitlines()
    for i, k in enumerate(["bin", "hbb", "cell", "tegrastats"]):
        chk[k + "_state"] = lines[i] if i < len(lines) else "?"
    chk["bin_mtime"] = lines[4] if len(lines) > 4 else "?"
    return chk


# ---------------------------------------------------------------- 单轮跑道

def run_once(r: Remote, stack: str, video: str, work: str, local_out: Path,
             timeout: int, use_tegrastats: bool) -> dict:
    """机上跑一轮视频，取回产物与计时，返回本轮指标。"""
    s = STACKS[stack]
    video_path = video if video.startswith("/") else f"{VIDEO_DIR}/{video}"
    log = f"{work}/run.log"
    matrix, route, pathf = f"{work}/cell_matrix.json", f"{work}/route_plan.json", f"{work}/planned_path.txt"
    first_flag, rc_file = f"{work}/first_plan.flag", f"{work}/rc"
    runner = f"{work}/runner.sh"
    r.put_text(RUNNER_SH, runner)
    r.run(f"chmod +x {runner}")

    args = s["args"].format(video=f"'{video_path}'", hbb=s["hbb"], cell=s["cell"],
                            matrix=matrix, route=route, path=pathf)
    # 计时 + 首份规划时延 + 退出码，由机上 runner 负责。
    # 注：机上无 GNU /usr/bin/time（实测 NO_TIME），峰值内存由 tegrastats 提供。
    inner = f"LOG={log} ROUTE={route} bash {runner} {log} {first_flag} {route} {rc_file} {s['bin']} {args}"
    teg_start = teg_stop = ""
    if use_tegrastats:
        teg = f"{work}/tegrastats.log"
        teg_start = f"(nohup tegrastats --interval 1000 > {teg} 2>&1 & echo $! > {work}/teg.pid); "
        teg_stop = f"; kill $(cat {work}/teg.pid) 2>/dev/null"
    t0 = time.time()
    rc, out = r.run(teg_start + inner + teg_stop, timeout=timeout)
    wall_wrapper = time.time() - t0

    # 取回产物
    got = {}
    for name, rem in [("run.log", log), ("route_plan.json", route),
                      ("planned_path.txt", pathf), ("cell_matrix.json", matrix),
                      ("rc", rc_file), ("rc.time", rc_file + ".time"),
                      ("first_plan.flag", first_flag)]:
        got[name] = r.get(rem, local_out / stack / video.replace(".mp4", "") / name)
    if use_tegrastats:
        r.get(f"{work}/tegrastats.log", local_out / stack / video.replace(".mp4", "") / "tegrastats.log")

    # 解析
    def _local(p):
        f = local_out / stack / video.replace(".mp4", "") / p
        return f.read_text(errors="replace") if f.exists() else ""

    rlog = _local("run.log")
    rc_txt = _local("rc").strip()
    ts = _local("rc.time").split()
    first = _local("first_plan.flag").strip()

    exit_code = int(rc_txt) if rc_txt.isdigit() else -1
    t_start, t_end = (float(ts[0]), float(ts[1])) if len(ts) >= 2 else (0.0, 0.0)
    t_wall = (t_end - t_start) if t_end > t_start else wall_wrapper
    t_first_plan = (float(first) - t_start) if first and t_start else None

    frames = len(re.findall(r"localized=", rlog))
    localized = len(re.findall(r"localized=1", rlog))
    plan_updates = len(re.findall(r"=== 路线规划 ===", rlog))
    rss = re.search(r"Maximum resident set size \(kbytes\):\s*(\d+)", rlog)
    # 异常行扫描：排除 TensorRT 的已知良性告警（跨设备使用 engine 的提示）。
    crash_hits = []
    for line in rlog.splitlines():
        if "models of devices is not recommended" in line:
            continue
        if CRASH_PAT.search(line):
            crash_hits.append(line.strip()[:120])
    crash_hits = crash_hits[:6]

    route_txt = ""
    rp = local_out / stack / video.replace(".mp4", "") / "route_plan.json"
    if rp.exists():
        try:
            route_txt = json.dumps(json.loads(rp.read_text()) , sort_keys=True, ensure_ascii=False)
        except Exception:  # noqa: BLE001
            route_txt = rp.read_text(errors="replace")

    return {
        "stack": stack, "video": video, "exit_code": exit_code,
        "t_wall_s": round(t_wall, 3), "t_first_plan_s": (round(t_first_plan, 3) if t_first_plan else None),
        "frames_processed": frames, "localized_frames": localized,
        "localized_ratio": round(localized / frames, 4) if frames else None,
        "fps_eff": round(frames / t_wall, 2) if t_wall > 0 and frames else None,
        "per_frame_ms": round(t_wall * 1000 / frames, 3) if frames else None,
        "n_plan_updates": plan_updates,
        "peak_rss_kb": int(rss.group(1)) if rss else None,
        "crash_keywords": crash_hits[:6],
        "artifacts_ok": all(got.get(k) for k in ("run.log", "route_plan.json")),
        "route_json_present": rp.exists(),
        "_route_text": route_txt,
    }


# ---------------------------------------------------------------- 评测汇总

def summarize(all_runs: list[dict], local_out: Path) -> dict:
    by_stack: dict[str, list[dict]] = {}
    for x in all_runs:
        by_stack.setdefault(x["stack"], []).append(x)

    report = {}
    for stack, runs in by_stack.items():
        # 路线一致性只在**同一段视频的重复轮次**之间比较（不同视频路线本就不同）。
        per_video: dict[str, int | None] = {}
        for video in sorted({x["video"] for x in runs}):
            routes = [x["_route_text"] for x in runs
                      if x["video"] == video and x["_route_text"]]
            per_video[video] = len(set(routes)) if routes else None
        judged = [v for v in per_video.values() if v is not None]
        consistent = (all(v == 1 for v in judged)) if judged else None
        walls = [x["t_wall_s"] for x in runs if x["t_wall_s"]]
        firsts = [x["t_first_plan_s"] for x in runs if x["t_first_plan_s"] is not None]
        rss = [x["peak_rss_kb"] for x in runs if x["peak_rss_kb"]]
        report[stack] = {
            "runs": len(runs),
            "runs_failed": sum(1 for x in runs if x["exit_code"] != 0),
            "exit_all_zero": all(x["exit_code"] == 0 for x in runs),
            "route_consistent": consistent,
            "route_variants": max(judged) if judged else None,
            "per_video_variants": per_video,
            "t_wall_s": {"min": min(walls) if walls else None,
                         "max": max(walls) if walls else None,
                         "avg": round(sum(walls) / len(walls), 3) if walls else None},
            "t_first_plan_s": {"min": min(firsts) if firsts else None,
                               "max": max(firsts) if firsts else None,
                               "avg": round(sum(firsts) / len(firsts), 3) if firsts else None},
            "peak_rss_kb_max": max(rss) if rss else None,
            "frames_processed_total": sum(x["frames_processed"] for x in runs),
            "n_plan_updates_total": sum(x["n_plan_updates"] for x in runs),
            "crash_any": any(x["crash_keywords"] for x in runs),
            "plan_layer_present": any(x["n_plan_updates"] > 0 for x in runs),
        }
    return report


def write_report(report: dict, all_runs: list[dict], local_out: Path, args) -> None:
    local_out.mkdir(parents=True, exist_ok=True)
    (local_out / "metrics.json").write_text(
        json.dumps({"generated": _dt.datetime.now().isoformat(timespec="seconds"),
                    "host": args.host, "summary": report, "runs": all_runs},
                   ensure_ascii=False, indent=2), encoding="utf-8")

    L = ["# 机载自主规划 · 跑道与评测报告", "",
         f"- 生成：{_dt.datetime.now():%Y-%m-%d %H:%M:%S}",
         f"- 机载：{args.host}（用户 {args.user}）",
         f"- 视频：{', '.join(args.video)}", f"- 轮次：{args.runs}", "",
         "## 指标口径", "",
         "| 维度 | 指标 | 说明 |", "|---|---|---|",
         "| 时间 | t_wall_s | 单段视频端到端墙钟秒（启动→推理→落格→规划→落盘） |",
         "| 时间 | t_first_plan_s | 首份 route_plan.json 产出时延（规划层响应速度） |",
         "| 时间 | fps_eff / per_frame_ms | 等效帧率 / 单帧均摊毫秒 |",
         "| 时间 | n_plan_updates | 规划触发次数 |",
         "| 稳定性 | exit_all_zero / runs_failed | 退出码与失败轮数 |",
         "| 稳定性 | route_consistent | 跨轮路线文本是否完全一致（结果可复现） |",
         "| 稳定性 | peak_rss_kb_max | 峰值常驻内存 |",
         "| 稳定性 | crash_any | 是否命中崩溃/异常关键字 |",
         "| 能力 | plan_layer_present | 该栈是否真的产出规划（接入规划层后应为 ✅） |",
         "", "## 汇总", "",
         "| 栈 | 轮数 | 退出码全0 | 路线一致 | t_wall(avg s) | 首规划(avg s) | 峰值RSS(MB) | 规划层 | 崩溃 |",
         "|---|---|---|---|---|---|---|---|---|"]
    for stack, s in report.items():
        L.append("| {st} | {n} | {ez} | {rc} | {tw} | {fp} | {rss} | {pl} | {cr} |".format(
            st=stack, n=s["runs"], ez="✅" if s["exit_all_zero"] else f"❌({s['runs_failed']})",
            rc=("✅一致" if s["route_consistent"] else ("—无产物" if s["route_consistent"] is None else f"⚠️{s['route_variants']}种")),
            tw=(s["t_wall_s"]["avg"] if s["t_wall_s"]["avg"] is not None else "—"),
            fp=(s["t_first_plan_s"]["avg"] if s["t_first_plan_s"]["avg"] is not None else "—"),
            rss=(round(s["peak_rss_kb_max"] / 1024, 1) if s["peak_rss_kb_max"] else "—"),
            pl=("✅" if s["plan_layer_present"] else "❌无规划层"),
            cr=("⚠️有" if s["crash_any"] else "无")))
    L += ["", "## 逐轮明细", "",
          "| 栈 | 视频 | 退出码 | t_wall(s) | 首规划(s) | 帧数 | 定位率 | 规划次数 | 峰值RSS(kB) | 异常 |",
          "|---|---|---|---|---|---|---|---|---|---|"]
    for x in all_runs:
        L.append("| {s} | {v} | {e} | {w} | {f} | {n} | {lr} | {p} | {r} | {c} |".format(
            s=x["stack"], v=x["video"], e=x["exit_code"], w=x["t_wall_s"],
            f=x["t_first_plan_s"], n=x["frames_processed"],
            lr=(f"{x['localized_ratio']*100:.1f}%" if x["localized_ratio"] is not None else "—"),
            p=x["n_plan_updates"], r=(x["peak_rss_kb"] if x["peak_rss_kb"] else "—"),
            c=("、".join(x["crash_keywords"]) or "无")))
    (local_out / "report.md").write_text("\n".join(L), encoding="utf-8")


# ---------------------------------------------------------------- main

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="air_plan_bench.py",
                                 description="机载自主规划 · 一键跑道与评测（视频喂入 Orin 自主跑完）")
    ap.add_argument("--host", default="192.168.55.1")
    ap.add_argument("--user", default="duzhong")
    ap.add_argument("--password", default=None)
    ap.add_argument("--port", type=int, default=22)
    ap.add_argument("--stack", choices=["planner7", "mjpg13", "both"], default="planner7")
    ap.add_argument("--runs", type=int, default=1, help="每段视频重复轮数")
    ap.add_argument("--video", action="append", default=None,
                    help="机上视频文件名（可多次）；缺省=电脑端同批 4 段")
    ap.add_argument("--out", default="bench_out", help="本机产物目录")
    ap.add_argument("--timeout", type=int, default=1800, help="单轮最长秒数")
    ap.add_argument("--no-link", action="store_true", help="跳过 USB-RNDIS 链路续租")
    ap.add_argument("--no-tegrastats", action="store_true", help="不采 GPU/内存峰值")
    ap.add_argument("--list-videos", action="store_true", help="只列机上视频后退出")
    ap.add_argument("--clean", action="store_true", help="跑完清理机上临时目录")
    ap.add_argument("--dry-run", action="store_true", help="不连硬件，只打印计划")
    args = ap.parse_args(argv)

    args.video = args.video or DEFAULT_VIDEOS
    local_out = Path(args.out).resolve()
    stacks = ["planner7", "mjpg13"] if args.stack == "both" else [args.stack]
    ts = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    remote_root = f"/tmp/ag_bench_{ts}"

    print("=" * 68)
    print("机载自主规划 · 一键跑道与评测")
    print(f"  机载 {args.host}   栈 {', '.join(stacks)}   视频 {len(args.video)} 段 × {args.runs} 轮")
    print(f"  机上临时目录 {remote_root}   本机产物 {local_out}")
    print("=" * 68)

    if args.dry_run:
        for st in stacks:
            s = STACKS[st]
            for v in args.video:
                a = s["args"].format(video=f"{VIDEO_DIR}/{v}", hbb=s["hbb"],
                                     cell=s["cell"], matrix="<work>/cell_matrix.json",
                                     route="<work>/route_plan.json", path="<work>/planned_path.txt")
                print(f"[dry] {s['bin']} {a}")
        return 0

    if (not args.no_link and args.host.startswith("192.168.55.")
            and not ensure_link()):
        return 3

    r = Remote(args.host, args.user, read_password(args), args.port).connect()
    try:
        if args.list_videos:
            list_videos(r)
            return 0
        rc, _ = r.run(f"mkdir -p {remote_root}", timeout=30)
        if rc != 0:
            print(f"[warn] 无法建 {remote_root}，改用 /tmp")
            remote_root = "/tmp/ag_bench"
            r.run(f"mkdir -p {remote_root}")

        all_runs: list[dict] = []
        for st in stacks:
            chk = probe(r, st)
            print(f"\n[体检] {st}: " + ", ".join(f"{k}={v}" for k, v in chk.items()))
            if chk.get("bin_state") != "BIN_OK" or chk.get("hbb_state") != "HBB_OK" \
                    or chk.get("cell_state") != "CELL_OK":
                print(f"[skip] {st} 资产不全，跳过（bin={chk.get('bin_state')} "
                      f"hbb={chk.get('hbb_state')} cell={chk.get('cell_state')}）")
                continue
            runner = f"{remote_root}/{st}/runner.sh"
            r.run(f"mkdir -p {remote_root}/{st}")
            r.put_text(RUNNER_SH, runner)
            r.run(f"chmod +x {runner}")
            for v in args.video:
                for k in range(args.runs):
                    work = f"{remote_root}/{st}/{v.replace('.mp4','')}_r{k}"
                    r.run(f"rm -rf {work}; mkdir -p {work}")
                    print(f"\n[run] {st} · {v} · 第 {k+1}/{args.runs} 轮 …")
                    m = run_once(r, st, v, work, local_out, args.timeout,
                                 not args.no_tegrastats)
                    print(f"      exit={m['exit_code']} wall={m['t_wall_s']}s "
                          f"首规划={m['t_first_plan_s']}s 帧={m['frames_processed']} "
                          f"定位率={m['localized_ratio']} 规划次数={m['n_plan_updates']} "
                          f"峰值RSS={m['peak_rss_kb']}kB")
                    if m["crash_keywords"]:
                        print(f"      ⚠️ 异常关键字：{m['crash_keywords']}")
                    all_runs.append(m)

        if not all_runs:
            print("\n[结论] 没有任何一轮跑成（资产缺失或链路不通）")
            return 1
        report = summarize(all_runs, local_out)
        write_report(report, all_runs, local_out, args)
        print("\n" + "=" * 68)
        for st, s in report.items():
            print(f"[结论] {st}: 退出全0={s['exit_all_zero']} 路线一致={s['route_consistent']} "
                  f"t_wall_avg={s['t_wall_s']['avg']}s 首规划_avg={s['t_first_plan_s']['avg']}s "
                  f"有规划层={s['plan_layer_present']}")
        print(f"[产物] {local_out / 'report.md'}")
        print(f"        {local_out / 'metrics.json'}")
        print("=" * 68)
        if args.clean:
            r.run(f"rm -rf {remote_root}")
            print(f"[clean] 已清理机上 {remote_root}")
        return 0
    finally:
        r.close()


if __name__ == "__main__":
    raise SystemExit(main())
