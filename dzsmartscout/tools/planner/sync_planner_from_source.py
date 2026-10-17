#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""sync_planner_from_source.py —— 把小车侧路径规划器**单源码同步**到无人机侧（不 fork）

为什么要这样：本项目有血泪教训 —— "修了没部署/没对账"导致车跑旧代码。
所以无人机侧**绝不手工维护第二份规划器**，只允许由本脚本从真源生成，
并用逐文件 sha256 账本把"跑的是哪一版"钉死。

真源（**唯一**，本脚本只读它，绝不写）：
    <repo>/dzrobot_v2/
        src/dzbrain/dzbrain/__init__.py
        src/dzbrain/dzbrain/core/{__init__.py, config.py, geometry.py}
        src/dzbrain/dzbrain/route/{__init__.py, model.py, parser.py, physics.py,
                                  planner.py, validate.py}
        tools/{plan_route.py, route_check.py, shoot_planner.py}
        config/{map.yaml, robot.yaml}

产物（全部在 dzsmartscout/tools/planner/ 内）：
    vendor/dzbrain_ws/…            与真源**逐字节一致**的树（保持 src/ + tools/ + config/ 相对结构，
                                   这样 `vendor/dzbrain_ws/tools/plan_route.py` 与车上跑的是**同一个入口**）
    vendor/dzbrain_ws/MANIFEST.txt 本次同步的文件清单
    HASH_LEDGER.json               哈希账本（每文件 sha256 + size + 源 mtime + 生成时间 + 校验结论）
    EVIDENCE/sync_log.txt          最近一次同步/校验的人类可读报告

用法：
    python sync_planner_from_source.py                 # 同步 + 校验 + 落账本
    python sync_planner_from_source.py --verify-only   # 只校验（账本 vs 真源 vs 产物），检测漂移
    python sync_planner_from_source.py --push-drone    # 再 scp 到无人机 ~/dzplanner/（离线则如实报错）
    python sync_planner_from_source.py --drone-host 10.29.122.104 --drone-user duzhong

退出码：0 成功且逐文件 sha256 一致；1 有文件不一致/缺失/漂移；2 参数或路径问题。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import pathlib
import shutil
import subprocess
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent                 # dzsmartscout/tools/planner
WORKSPACE = HERE.parents[2]                                    # 比赛存档/v1
DEFAULT_SOURCE = WORKSPACE / "dzrobot_v2"
VENDOR_REL = "vendor/dzbrain_ws"

#: ★ 显式文件清单（**唯一真源**的闭包；加文件必须在这里加，并说明为什么）
#: 依据：`plan_route.py` / `route_check.py` 的实际 import 闭包
#:   · dzbrain.route.*  -> dzbrain.core.geometry / dzbrain.core.config
#:   · route_check.py   -> 刻意**不** import dzbrain（自带一份物理常量，靠 src/dzbrain/tests 对拍钉住）
FILE_SET = [
    "src/dzbrain/dzbrain/__init__.py",
    "src/dzbrain/dzbrain/core/__init__.py",
    "src/dzbrain/dzbrain/core/config.py",
    "src/dzbrain/dzbrain/core/geometry.py",
    "src/dzbrain/dzbrain/route/__init__.py",
    "src/dzbrain/dzbrain/route/model.py",
    "src/dzbrain/dzbrain/route/parser.py",
    "src/dzbrain/dzbrain/route/physics.py",
    "src/dzbrain/dzbrain/route/planner.py",
    "src/dzbrain/dzbrain/route/validate.py",
    "tools/plan_route.py",
    "tools/route_check.py",
    "tools/shoot_planner.py",
    "config/map.yaml",
    "config/robot.yaml",
]

#: 产物里**不要求**与真源同哈希的东西（脚本自己生成的），校验时跳过
GENERATED = {"MANIFEST.txt"}


def sha256_file(p: pathlib.Path) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


def import_closure_problems(vendor: pathlib.Path) -> list[str]:
    """粗粒度闭包自检：产物里形如 `from ..core import x` / `from . import y` 的相对导入，
    必须能在产物树里找到对应模块文件 —— 防止"复制漏了一个文件"这种静默故障。"""
    problems: list[str] = []
    root = vendor / "src" / "dzbrain"
    for py in sorted(root.rglob("*.py")):
        pkg_rel = py.relative_to(root).with_suffix("")
        parts = list(pkg_rel.parts[:-1])          # 所在包
        for line in py.read_text(encoding="utf-8", errors="replace").splitlines():
            s = line.strip()
            if not (s.startswith("from .") or s.startswith("from ..")):
                continue
            mod = s.split(" import ")[0][4:].strip()       # 去掉 "from "
            dots = len(mod) - len(mod.lstrip("."))
            tail = mod[dots:].replace(".", "/")
            base = parts[: len(parts) - (dots - 1)] if dots > 1 else parts
            cand = root.joinpath(*base, *([tail] if tail else []))
            ok = any((cand.with_suffix(e)).is_file() for e in (".py",)) or \
                (cand / "__init__.py").is_file()
            if not ok:
                problems.append("%s -> %s" % (py.relative_to(vendor), s))
    return problems


def do_sync(src: pathlib.Path, vendor: pathlib.Path, log: list[str]) -> dict:
    entries = []
    vendor.mkdir(parents=True, exist_ok=True)
    for rel in FILE_SET:
        sp = src / rel
        if not sp.is_file():
            raise SystemExit("真源缺文件：%s" % sp)
        dp = vendor / rel
        dp.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(sp, dp)                      # ★ 逐字节复制，不做任何改写
        s_src, s_dst = sha256_file(sp), sha256_file(dp)
        same = s_src == s_dst
        st = sp.stat()
        entries.append({
            "rel": rel.replace("\\", "/"),
            "src": str(sp),
            "size": sp.stat().st_size,
            "sha256": s_src,
            "vendor_sha256": s_dst,
            "same": same,
            "src_mtime": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(st.st_mtime)),
        })
        log.append("  %-52s %9d B  sha256=%s  %s"
                   % (rel, sp.stat().st_size, s_src[:20], "OK" if same else "MISMATCH"))
    (vendor / "MANIFEST.txt").write_text("\n".join(e["rel"] for e in entries) + "\n",
                                         encoding="utf-8")
    return {"files": entries, "count": len(entries),
            "all_same": all(e["same"] for e in entries)}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default=str(DEFAULT_SOURCE), help="真源仓库根（默认 dzrobot_v2）")
    ap.add_argument("--verify-only", action="store_true", help="只校验，不复制")
    ap.add_argument("--push-drone", action="store_true", help="同步后再 scp 到无人机")
    ap.add_argument("--run-drone", action="store_true", help="scp 后在无人机上跑一次 plan_route")
    ap.add_argument("--drone-host", default=os.environ.get("DZ_DRONE_HOST", "10.29.122.104"))
    ap.add_argument("--drone-user", default="duzhong")
    ap.add_argument("--remote-dir", default="~/dzplanner")
    a = ap.parse_args()

    src = pathlib.Path(a.source).resolve()
    vendor = HERE / VENDOR_REL
    ledger_path = HERE / "HASH_LEDGER.json"
    evid = HERE / "EVIDENCE"
    evid.mkdir(parents=True, exist_ok=True)
    log: list[str] = []
    stamp = time.strftime("%Y-%m-%d %H:%M:%S")
    log.append("=== sync_planner_from_source  %s ===" % stamp)
    log.append("源(真源，只读): %s" % src)
    log.append("产物        : %s" % vendor)
    if not src.is_dir():
        print("真源目录不存在：%s" % src)
        return 2

    if a.verify_only:
        old = json.loads(ledger_path.read_text(encoding="utf-8")) if ledger_path.is_file() else None
        if not old:
            print("没有账本可校验（先跑一次不带 --verify-only）")
            return 2
        log.append("模式: verify-only（账本 vs 真源 vs 产物）")
        drift, missing = [], []
        for e in old["files"]:
            sp, dp = src / e["rel"], vendor / e["rel"]
            if not sp.is_file():
                missing.append(e["rel"]); continue
            if not dp.is_file():
                missing.append(e["rel"]); continue
            s_now, v_now = sha256_file(sp), sha256_file(dp)
            if s_now != e["sha256"]:
                drift.append("%s  真源变了 %s -> %s" % (e["rel"], e["sha256"][:12], s_now[:12]))
            elif v_now != s_now:
                drift.append("%s  产物与真源不一致 %s -> %s" % (e["rel"], s_now[:12], v_now[:12]))
        log.append("漂移 %d 条 / 缺失 %d 条" % (len(drift), len(missing)))
        for d in drift + missing:
            log.append("  ! " + d)
        (evid / "sync_log.txt").write_text("\n".join(log) + "\n", encoding="utf-8")
        print("\n".join(log))
        return 1 if (drift or missing) else 0

    res = do_sync(src, vendor, log)
    probs = import_closure_problems(vendor)
    log.append("闭包自检：%s" % ("通过" if not probs else "发现 %d 个可疑相对导入" % len(probs)))
    for p in probs:
        log.append("  ! " + p)

    ledger = {
        "generated_at": stamp,
        "source_root": str(src),
        "vendor_root": str(vendor),
        "file_set_count": res["count"],
        "all_same": res["all_same"] and not probs,
        "closure_problems": probs,
        "files": res["files"],
    }
    ledger_path.write_text(json.dumps(ledger, ensure_ascii=False, indent=2), encoding="utf-8")
    log.append("账本: %s（%d 条）" % (ledger_path, res["count"]))
    log.append("结论：%s" % ("逐文件 sha256 一致 ✅" if ledger["all_same"] else "有不一致 ❌"))

    ok = ledger["all_same"]
    if a.push_drone or a.run_drone:
        log.append("\n=== 推到无人机 %s@%s:%s ===" % (a.drone_user, a.drone_host, a.remote_dir))
        try:
            subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8",
                            "%s@%s" % (a.drone_user, a.drone_host),
                            "mkdir -p %s" % a.remote_dir],
                           check=True, capture_output=True, text=True, timeout=30)
            subprocess.run(["scp", "-q", "-r", str(vendor),
                            "%s@%s:%s/" % (a.drone_user, a.drone_host, a.remote_dir)],
                           check=True, capture_output=True, text=True, timeout=300)
            log.append("  scp 完成：%s/dzbrain_ws" % a.remote_dir)
            if a.run_drone:
                cmd = ("cd %s/dzbrain_ws && python3 tools/plan_route.py --emit tokens "
                       "--config config/robot.yaml --map config/map.yaml" % a.remote_dir)
                p = subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8",
                                    "%s@%s" % (a.drone_user, a.drone_host), cmd],
                                   capture_output=True, text=True, timeout=300)
                log.append("  远端 plan_route 退出码=%d" % p.returncode)
                log.append("  远端输出(尾 5 行)：")
                for line in (p.stdout or "").strip().splitlines()[-5:]:
                    log.append("    " + line)
                if p.returncode != 0:
                    log.append("  远端 stderr(尾 3 行)：")
                    for line in (p.stderr or "").strip().splitlines()[-3:]:
                        log.append("    " + line)
        except Exception as exc:                     # noqa: BLE001  离线是**正常**结果，如实记录
            log.append("  ❌ 无人机不可达/不可推送：%s: %s" % (type(exc).__name__, exc))
            log.append("  ⇒ 网络前提未满足（无人机需上电并接入同一网段）。"
                       "本步骤**未完成**，不代表已部署。")
            ok = ok  # 本地同步仍然有效；远端状态单独看这一行

    (evid / "sync_log.txt").write_text("\n".join(log) + "\n", encoding="utf-8")
    print("\n".join(log))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
