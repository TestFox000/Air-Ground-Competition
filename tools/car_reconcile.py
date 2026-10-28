#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""三方对账：本地工作树 / 上次部署清单 / 车端同名文件 sha256。

为什么需要（2026-10-28 实车教训）：deploy_to_car.py 的 sha256 对账**只在 push 结束时跑一次**，
结果只打在屏幕上、不落盘。于是"车端现在到底跑的是哪一版"只能靠现场翻日志。
本工具把三方摆在一张表里，并支持把本次车端清单落盘，下次拿它当基线。

复用而非另造（AGENTS.md 铁律 2）：
  · 文件清单  = deploy_to_car.PUSH_FILES（唯一真源，本文件**不另编一份清单**）
  · 清单实现  = deploy_to_car.local_manifest() / remote_manifest()
  · 差集分类  = deploy_to_car.diff_manifest()

退出码：
  0 = 三方一致（或仅有"基线缺失/基线旧"这类非差异）
  1 = 有真实差集（本地改未部署 / 车端被改 / 缺失 / 三方不一致），或车端清单读不出来
  2 = 车不在线 —— **不是失败**，是按设计"今天没车"的形态（只输出本地对账结果）

用法：
  python tools/car_reconcile.py                      # 默认 duzhong.local（DNS 可能指旧 IP，见 README）
  python tools/car_reconcile.py --host 192.168.137.164
  python tools/car_reconcile.py --offline            # 完全不尝试连接（车还没上电时先看本地）
  python tools/car_reconcile.py --record             # 车端清单落盘，作为下次的"上次部署清单"
  python tools/car_reconcile.py --json               # 机器读
"""
from __future__ import annotations

import argparse
import json
import pathlib
import subprocess
import sys
import time

TOOLS = pathlib.Path(__file__).resolve().parent
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

import deploy_to_car as dt  # noqa: E402  （同目录工具，清单真源）

DEFAULT_HOST = dt.CAR_HOST                       # "duzhong.local"
DEFAULT_MANIFEST = dt.WS / "artifacts" / "deploy_manifest_last.json"
PROBE_TIMEOUT_S = 15.0

#: 判成"车端与本地不一致"的状态集合（决定退出码 1 还是 0）
BAD = ("本地改未部署", "车端被改/回滚", "车端缺失", "本地缺失", "三方不一致")
#: 允许保留的"非差异"状态（基线还没建 / 基线比车端旧，都不该挡对账）
OKISH = ("一致", "已同步（基线缺）", "已部署（基线旧）", "本地有·基线无", "一致（基线）")


#: --json 时把人类可读日志改走 stderr，保证 stdout 是纯 JSON
JSON_MODE = False


def log(msg: str) -> None:
    print(msg, file=sys.stderr if JSON_MODE else sys.stdout, flush=True)


def items() -> list[tuple[str, str, bool]]:
    """(本地相对路径, 车端绝对路径, 是否目录) —— 逐项照抄 deploy_to_car.PUSH_FILES。"""
    out: list[tuple[str, str, bool]] = []
    for local, remote in dt.PUSH_FILES:
        src = dt.WS / local
        if not src.exists():
            raise SystemExit("❌ 本地文件不存在：" + local)
        out.append((local, remote, src.is_dir()))
    return out


def local_rows() -> dict[str, str]:
    """本地工作树清单：键 = "本地相对路径"（目录展开成 "目录/子路径"）。"""
    rows: dict[str, str] = {}
    for local, _remote, is_dir in items():
        m = dt.local_manifest(local)
        if is_dir:
            for rel, sha in m.items():
                rows[local + "/" + rel] = sha
        else:
            rows[local] = next(iter(m.values()))
    return rows


def remote_rows(host: str) -> dict[str, str]:
    """车端实时清单。任一项读不到就抛错 —— "读不到"绝不能被当成"清单为空=通过"。"""
    rows: dict[str, str] = {}
    for local, remote, is_dir in items():
        m = dt.remote_manifest(host, remote, is_dir)
        if not m:
            raise RuntimeError("车端清单为空（路径不存在/无权限/ssh 中断）：" + remote)
        if is_dir:
            for rel, sha in m.items():
                rows[local + "/" + rel] = sha
        else:
            rows[local] = next(iter(m.values()))
    return rows


def is_online(host: str) -> bool:
    """一次挥手连接：不看内容，只看 ssh 能不能建立。"""
    argv = ["ssh", *dt.SSH_OPTS, dt.CAR_USER + "@" + host, "true"]
    try:
        proc = subprocess.run(argv, capture_output=True, timeout=PROBE_TIMEOUT_S)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return proc.returncode == 0


def load_manifest(path: pathlib.Path) -> dict[str, str]:
    if not path.exists():
        return {}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    files = data.get("files") if isinstance(data, dict) else None
    return dict(files) if isinstance(files, dict) else {}


def save_manifest(path: pathlib.Path, files: dict[str, str], host: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {
        "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "host": host,
        "source": "deploy_to_car.PUSH_FILES",
        "count": len(files),
        "files": dict(sorted(files.items())),
    }
    path.write_text(json.dumps(data, ensure_ascii=False, indent=1), encoding="utf-8")


def verdict(local: str | None, base: str | None, car: str | None, *, online: bool) -> str:
    """一行的结论。car=None 且 online=False 表示"没车这一列"。"""
    if not online:
        if base is None:
            return "本地有·基线无"
        if local is None:
            return "本地缺失"
        return "一致（基线）" if local == base else "本地改未部署"
    if local is None:
        return "本地缺失"
    if car is None:
        return "车端缺失"
    if base is None:
        return "已同步（基线缺）" if local == car else "本地改未部署"
    if local == base == car:
        return "一致"
    if local != car and base == car:
        return "本地改未部署"
    if local == base != car:
        return "车端被改/回滚"
    if local == car != base:
        return "已部署（基线旧）"
    return "三方不一致"


def short(sha: str | None) -> str:
    return (sha or "")[:12] or "—"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="本地 / 上次部署清单 / 车端 三方 sha256 对账")
    ap.add_argument("--host", default=DEFAULT_HOST,
                    help="车端主机或 IP（默认 " + DEFAULT_HOST + "；DNS 可能指旧 IP，直接给 IP 最稳）")
    ap.add_argument("--manifest", default=str(DEFAULT_MANIFEST),
                    help="上次部署清单 JSON（默认 artifacts/deploy_manifest_last.json）")
    ap.add_argument("--offline", action="store_true", help="不尝试连接（车没上电时用），直接走离线分支")
    ap.add_argument("--record", action="store_true", help="把本次车端清单落盘，供下次当基线")
    ap.add_argument("--json", action="store_true", help="输出 JSON（机器读）")
    args = ap.parse_args(argv)
    global JSON_MODE
    JSON_MODE = bool(args.json)

    # 给了存在的路径就照用（相对当前目录）；否则相对 dzrobot_v2 根解析（默认值就是这种）
    manifest_path = pathlib.Path(args.manifest)
    if not manifest_path.exists() and not manifest_path.is_absolute():
        manifest_path = (dt.WS / manifest_path).resolve()

    local = local_rows()
    base = load_manifest(manifest_path)

    online = False
    if not args.offline:
        log("[探测] " + dt.CAR_USER + "@" + args.host + " …")
        online = is_online(args.host)

    car: dict[str, str] | None = None
    err = ""
    if online:
        try:
            car = remote_rows(args.host)
        except (RuntimeError, OSError) as exc:      # 车在但清单读不出 = 真异常
            err = str(exc)
            log("❌ " + err)

    if car is None:
        log("车不在线，仅输出本地对账结果（" + ("--offline" if args.offline else "ssh " + args.host + " 不通") + "）")
        log("  本地文件数 " + str(len(local)) + "｜基线 " + (str(len(base)) if base else "缺（首次或未 --record）"))
    else:
        log("车端在线：" + str(len(car)) + " 个文件已取回 sha256")

    keys = sorted(set(local) | set(base) | set(car or {}))
    rows = []
    for k in keys:
        st = verdict(local.get(k), base.get(k), car.get(k) if car is not None else None, online=online)
        rows.append({"key": k, "status": st,
                     "local": short(local.get(k)),
                     "base": short(base.get(k)),
                     "car": short(car.get(k)) if car is not None else "—"})

    counts: dict[str, int] = {}
    for r in rows:
        counts[r["status"]] = counts.get(r["status"], 0) + 1

    if args.json:
        print(json.dumps({"host": args.host, "online": online,
                          "manifest": str(manifest_path), "counts": counts, "rows": rows},
                         ensure_ascii=False, indent=1))
    else:
        bad = [r for r in rows if r["status"] in BAD]
        log("")
        if bad:
            log("状态           本地          基线          车端          文件")
            log("-" * 108)
            for r in bad[:40]:
                log("%-14s %-13s %-13s %-13s %s"
                    % (r["status"], r["local"], r["base"], r["car"], r["key"]))
            if len(bad) > 40:
                log("… 另有 " + str(len(bad) - 40) + " 条差异（--json 看全量）")
            log("-" * 108)
        else:
            log("无差异行（逐项一致或仅基线缺失/基线旧）")
        log("统计：" + "｜".join(k + " " + str(v) for k, v in sorted(counts.items())))

    if car is not None and args.record:
        save_manifest(manifest_path, car, args.host)
        log("已落盘基线：" + str(manifest_path) + "（" + str(len(car)) + " 项）")
    elif args.record:
        log("⚠️ 未在线，未落盘基线")

    if err:
        log("结论：❌ 车在线但清单读取失败 —— 不是「没差异」，先修 ssh/路径")
        return 1
    if not online:
        return 2                                   # 设计约定：车不在线 = 2，不是失败
    bad_n = sum(counts.get(s, 0) for s in BAD)
    if bad_n:
        log("结论：❌ 差集 " + str(bad_n) + " 项 —— 先 deploy_to_car.py all，再回来复跑")
        return 1
    log("结论：✅ 三方一致（基线缺失/基线旧不算差异）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
