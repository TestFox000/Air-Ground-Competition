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
        tools/{_field_const.py, plan_route.py, route_check.py, shoot_planner.py}
        config/{robot.yaml}

★ 地图**不在**同步清单里（卡 WB-9-VENDOR-20261031）：
  真源 `dzrobot_v2/config/map.yaml` 已按 D-054 裁决4 从工作树移除（车端布局模式零地图），
  WB-1 又把本目录 vendor 下的那份副本（2411 B）移出工作树 ⇒ 再把它列进 `FILE_SET`
  只会要求「一个按裁决不许存在的地图」：`test_every_ledger_file_exists_in_vendor` 必红。
  地图改由 `map_source.py` 解析到**仓库内已有逐字快照**
  （`dzrobot_v2/src/dzbrain/tests/fixtures/map_field7_t24.yaml`，与账本旧 `config/map.yaml`
  条目逐字节同源），账本里只留一条 `map_source` 引用（路径 + sha256），**不复制内容**。

产物（全部在 dzsmartscout/tools/planner/ 内）：
    vendor/dzbrain_ws/…            与真源**逐字节一致**的树（保持 src/ + tools/ + config/ 相对结构，
                                   这样 `vendor/dzbrain_ws/tools/plan_route.py` 与车上跑的是**同一个入口**）
    vendor/dzbrain_ws/MANIFEST.txt 本次同步的文件清单
    HASH_LEDGER.json               哈希账本（每文件 sha256 + size + 源 mtime + 生成时间 + 校验结论
                                   ＋ **路线内容指纹**：mission_blocks 逐 token + join 后 sha256
                                   ＋ **map_source**：本轮地图快照的路径/sha256/选图理由）
    EVIDENCE/sync_log.txt          最近一次**同步**的人类可读报告
    EVIDENCE/verify_log.txt        最近一次 **--verify-only** 的报告（不再覆盖同步日志）

★ 内容级对账（v4 审计 F-17 / 结构性洞①）：
  文件哈希只能证明"vendor 与真源同字节"，**不能**单独回答"两侧路线是不是同一条"。
  本脚本把 `config/robot.yaml:path_executor.ros__parameters.mission_blocks`（`route.tokens`
  的别名 = 车侧现役路线）读出来做逐 token 对账，并把 count/sha256 写进账本与日志 ——
  审计时"vendor 自洽但已与车侧换线分叉、账本仍 all_same=true"那种假绿会在 `--verify-only` 里直接暴露。

用法：
    python sync_planner_from_source.py                 # 同步 + 校验 + 落账本
    python sync_planner_from_source.py --verify-only   # 只校验（账本 vs 真源 vs 产物），检测漂移
                                                      # ★ 卡 WB-10：默认**不写任何文件**；
                                                      #   要留档才加 --write-evidence
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
from dataclasses import dataclass

HERE = pathlib.Path(__file__).resolve().parent                 # dzsmartscout/tools/planner
WORKSPACE = HERE.parents[2]                                    # 比赛存档/v1
DEFAULT_SOURCE = WORKSPACE / "dzrobot_v2"
VENDOR_REL = "vendor/dzbrain_ws"

if str(HERE) not in sys.path:                                  # 同目录 map_source.py
    sys.path.insert(0, str(HERE))
import map_source                                              # noqa: E402

#: ★ 显式文件清单（**唯一真源**的闭包；加文件必须在这里加，并说明为什么）
#: 依据：`plan_route.py` / `route_check.py` / `shoot_planner.py` 的实际 import 闭包
#:   · dzbrain.route.*  -> dzbrain.core.geometry / dzbrain.core.config
#:   · route_check.py / shoot_planner.py -> `tools/_field_const.py`（n1 常量收敛新增：
#:     脚本侧唯一真源入口，按 __file__ 自举 src/dzbrain —— **必须一起 vendor**，
#:     否则产物树跑 route_check 会 ModuleNotFoundError）
#:   · route_check.py   -> 刻意**不** import dzbrain 业务层（自带物理参数，靠 src/dzbrain/tests 对拍钉住）
#:
#: ★ `config/map.yaml` 已从本清单删除（卡 WB-9-VENDOR-20261031）：
#:   ① 真源 `dzrobot_v2/config/map.yaml` 按 D-054 裁决4 已从工作树移除（车端零地图/布局模式），
#:      留着它 = 要求同步一个按裁决不许存在的文件；
#:   ② WB-1 又把 vendor 下的副本移出工作树 ⇒ `test_every_ledger_file_exists_in_vendor` 必红；
#:   ③ 地图改由同目录 `map_source.py` 解析到**仓库内逐字快照**
#:      （`dzrobot_v2/src/dzbrain/tests/fixtures/map_field7_t24.yaml`，与账本旧
#:      `config/map.yaml` 条目 sha256 `385e0718…` 逐字节同源），账本里另记一条 `map_source`。
#:   禁止把地图拷回 vendor：那正是 D-054/D-062 消灭的「第 4/5 份副本」。
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
    "tools/_field_const.py",
    "tools/plan_route.py",
    "tools/route_check.py",
    "tools/shoot_planner.py",
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


#: 现役路线所在的配置（`route.tokens` 是 `mission_blocks` 的 Spec 别名，见 core/config.py）
ROUTE_REL = "config/robot.yaml"


def read_mission_blocks(root: pathlib.Path) -> list[str]:
    """读某棵树 `robot.yaml` 的板载回退路线（`path_executor.ros__parameters.mission_blocks`）。

    解析失败 / 缺 PyYAML → 空列表（调用方按"无内容可比"处理，**绝不猜**）。
    """
    p = root / ROUTE_REL
    if not p.is_file():
        return []
    try:
        import yaml                                    # 与 dzbrain.core.config 同一依赖
        doc = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
    except Exception:                                  # noqa: BLE001  缺依赖/坏 yaml → 无内容可比
        return []
    pe = (doc.get("path_executor") or {}).get("ros__parameters") or {}
    try:
        return [str(t).strip() for t in (pe.get("mission_blocks") or []) if str(t).strip()]
    except Exception:                                  # noqa: BLE001
        return []


def _route_fp(tokens: list[str]) -> dict:
    """路线指纹：token 数 + `" ".join(tokens)` 的 sha256（n2 审计同口径）。"""
    text = " ".join(tokens)
    return {"count": len(tokens),
            "sha256": hashlib.sha256(text.encode("utf-8")).hexdigest(),
            "tokens": text}


def route_record(src: pathlib.Path, vendor: pathlib.Path) -> dict:
    """车侧现役 vs 产物 tree 的**路线内容**对账（逐 token）。"""
    a, b = read_mission_blocks(src), read_mission_blocks(vendor)
    rec = {"source": _route_fp(a), "vendor": _route_fp(b),
           "same": bool(a) and a == b}
    if a and b and a != b:
        idx = next((i for i, (x, y) in enumerate(zip(a, b)) if x != y), min(len(a), len(b)))
        rec["first_diff"] = {"index": idx,
                             "source": a[idx] if idx < len(a) else None,
                             "vendor": b[idx] if idx < len(b) else None}
    return rec


def route_line(rec: dict) -> str:
    """人类可读的一行路线对账（日志/账本共用口径）。"""
    s, v = rec["source"], rec["vendor"]
    if not s["count"] or not v["count"]:
        return "路线内容：无可比数据（缺 robot.yaml 或缺 mission_blocks）"
    line = ("路线内容：车侧 n=%d sha256=%s / 产物 n=%d sha256=%s → %s"
            % (s["count"], s["sha256"][:16], v["count"], v["sha256"][:16],
               "逐 token 一致 ✅" if rec["same"] else "不一致 ❌"))
    if not rec["same"] and rec.get("first_diff"):
        fd = rec["first_diff"]
        line += "（首个差异 @%d: 车侧 %r / 产物 %r）" % (fd["index"], fd["source"], fd["vendor"])
    return line


@dataclass
class _Retired:
    """一个已退役的账本条目（**真源与产物都不再存在**，留着它只会要求一个按裁决不许存在的文件）。"""
    rel: str
    reason: str


#: 已退役文件（卡 WB-9-VENDOR-20261031）——账本 `files` 里**不该再有**这些条目。
#:   `config/map.yaml`：① 真源按 D-054 裁决4 从工作树移除（车端零地图/布局模式）；
#:   ② WB-1 把 vendor 副本移出工作树。两边都不存在 ⇒ 留着它 = `test_every_ledger_file_exists_in_vendor`
#:   永远红（这正是 WB-1 记的"新增 1 error"）。地图改由 `map_source.py` 指仓库内逐字快照
#:   （`map_field7_t24.yaml`），账本里改记 `map_source` 一条引用。
RETIRED = (_Retired("config/map.yaml",
                    "D-054 裁决4（真源不放地图 / 车端布局模式）+ WB-1 移出 vendor 副本；"
                    "地图改由 map_source.py 指 dzrobot_v2/src/dzbrain/tests/fixtures/"
                    "map_field7_t24.yaml（不复制内容）"),)


def prune_retired(ledger: dict, vendor: pathlib.Path) -> tuple[dict, list[str]]:
    """从账本 `files` 里剔除**真源与产物都已不存在**的退役条目，返回 (新账本, 变更日志)。

    只动 `RETIRED` 里点名的 rel，且必须**两侧都不在盘上**才剔除 —— 只要有一侧还在，
    就不动它（宁可留一条红，也不掩盖真实缺失）。同时在账本里留 `retired` 账：
    剔除过什么、为什么、原 sha256 是多少（审计可查，不是我凭空删除）。
    """
    keep: list[dict] = []
    removed: list[dict] = []
    for e in ledger.get("files", []):
        rel = e.get("rel", "")
        hit = next((r for r in RETIRED if r.rel == rel), None)
        if hit is None:
            keep.append(e)
            continue
        # ★ 只有「真源与产物都不在盘上」才允许剔除；任一侧仍在 = 真实缺失，保留并让它继续红
        src_root = ledger.get("source_root")
        if not src_root or not pathlib.Path(src_root).is_dir():
            src_root = str(DEFAULT_SOURCE)
        on_disk = [p for p in (pathlib.Path(src_root) / rel, vendor / rel) if p.is_file()]
        if on_disk:
            keep.append(e)
            continue
        rec = dict(e)
        rec["retired_reason"] = hit.reason
        removed.append(rec)

    changed: list[str] = []
    for rec in removed:
        changed.append("剔除退役条目 %s（原 sha256 %s，%d B）：%s"
                       % (rec["rel"], rec.get("sha256", "")[:20], rec.get("size", 0),
                          rec["retired_reason"]))
    if removed:
        old_ret = list(ledger.get("retired") or [])
        ledger["files"] = keep
        ledger["retired"] = old_ret + removed
        ledger["file_set_count"] = len(keep)
    # ★ MANIFEST.txt 是本脚本自己生成的清单，必须与账本 files **逐条一致**
    #   （`test_manifest_matches_ledger` 就是钉这个）。剔除账本条目时同步剔除清单行，
    #   否则剔一条就换来一条新红 —— 那就成了"修一个红造一个红"。
    #   这里**总是**调一次（幂等）：上一轮剔账本、这一轮补清单也能收敛。
    msg = reconcile_manifest_retired(ledger, vendor)
    if msg.startswith("MANIFEST.txt 剔除"):
        changed.append(msg)
    return ledger, changed


def reconcile_manifest_retired(ledger: dict, vendor: pathlib.Path) -> str:
    """把 MANIFEST.txt 里**退役条目**的行去掉（幂等；没得改就返回说明字符串）。

    只动 `retired` 账里点名的 rel —— 别的一行不碰，防止顺手改坏清单。
    """
    man_path = vendor / "MANIFEST.txt"
    retired = {r.get("rel") for r in (ledger.get("retired") or []) if r.get("rel")}
    if not man_path.is_file() or not retired:
        return "MANIFEST.txt 无需改动（无退役条目或无清单）"
    lines = man_path.read_text(encoding="utf-8").splitlines()
    kept = [l for l in lines if l.strip() and l.strip() not in retired]
    if kept == [l for l in lines if l.strip()]:
        return "MANIFEST.txt 无需改动（已不含退役条目）"
    man_path.write_text("\n".join(kept) + "\n", encoding="utf-8")
    return ("MANIFEST.txt 剔除退役行 %d 行（%s）⇒ 清单 %d 行 == 账本 files %d 条"
            % (len(lines) - len(kept), "、".join(sorted(retired)),
               len(kept), len(ledger.get("files") or [])))


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
    route = route_record(src, vendor)
    log.append("  " + route_line(route))
    return {"files": entries, "count": len(entries),
            "route": route,
            "all_same": all(e["same"] for e in entries) and route["same"]}


def main() -> int:
    # Windows GBK 控制台打不出 ✅/❌：统一 UTF-8 输出，避免"同步其实成功、print 收尾崩成 exit 1"
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except Exception:                              # noqa: BLE001  非 TTY / 老解释器
            pass
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default=str(DEFAULT_SOURCE), help="真源仓库根（默认 dzrobot_v2）")
    ap.add_argument("--verify-only", action="store_true", help="只校验，不复制")
    ap.add_argument("--write-evidence", action="store_true",
                    help="只对 --verify-only 生效：把本次校验报告写进 EVIDENCE/verify_log.txt"
                         "（卡 WB-10 前它每次运行都无条件覆写，等于一个只读模式在替换"
                         "上一份只读结论）。缺省=只读，不写盘")
    ap.add_argument("--prune-ledger", action="store_true",
                    help="只把「真源与产物都不在盘上」的退役条目（RETIRED，卡 WB-9 的 config/map.yaml）"
                         "从账本 files 剔除并记进 retired 账；不复制、不校验、不动别的条目")
    ap.add_argument("--push-drone", action="store_true", help="同步后再 scp 到无人机")
    ap.add_argument("--run-drone", action="store_true", help="scp 后在无人机上跑一次 plan_route")
    ap.add_argument("--drone-host", default=os.environ.get("DZ_DRONE_HOST", "10.29.122.104"))
    ap.add_argument("--drone-user", default="duzhong")
    ap.add_argument("--remote-dir", default="~/dzplanner")
    ap.add_argument("--remote-map", default=None,
                    help="机上自有的地图路径（**只**给 --run-drone 用；本目录不再携带地图，"
                         "不给则跳过机上规划并如实记录，不拷地图到机上）")
    ap.add_argument("--map", default=None,
                    help="覆盖地图 yaml（缺省=map_source 具名缺省 field7：仓库内逐字快照；"
                         "也可用 DZ_PLANNER_MAP / ROBOT_MAP 覆盖）。**只影响本地校验与 --run-drone "
                         "远端命令，不会把地图拷进 vendor/**")
    a = ap.parse_args()

    src = pathlib.Path(a.source).resolve()
    vendor = HERE / VENDOR_REL
    ledger_path = HERE / "HASH_LEDGER.json"
    evid = HERE / "EVIDENCE"
    # ★ 卡 WB-10：目录创建改到**真正要写**的地方（只读模式不产生任何写动作）
    log: list[str] = []
    stamp = time.strftime("%Y-%m-%d %H:%M:%S")
    log.append("=== sync_planner_from_source  %s ===" % stamp)
    log.append("源(真源，只读): %s" % src)
    log.append("产物        : %s" % vendor)
    if not src.is_dir():
        print("真源目录不存在：%s" % src)
        return 2

    # ★ 地图引用（卡 WB-9）：只在账本里登记「该用哪张仓库内快照」，**不复制、不写 vendor**。
    try:
        minfo = map_source.resolve(a.map)
        map_ref = {"path": str(minfo.path), "sha256": minfo.sha256, "size": minfo.size,
                   "origin": minfo.origin, "name": minfo.name,
                   "why": map_source.WHY_FIELD7, "registry": str(map_source.REGISTRY),
                   "note": "内容不复制；vendor/dzbrain_ws/config/ 下不存地图（D-054/D-062）"}
        log.append("地图引用    : " + minfo.describe())
    except map_source.MapSourceError as exc:
        minfo = None
        map_ref = {"error": str(exc), "why": map_source.WHY_FIELD7,
                   "registry": str(map_source.REGISTRY)}
        log.append("地图引用    : ❌ %s" % exc)

    if a.prune_ledger:
        # ★ 只做一件事：把 RETIRED 里点名的、且两侧都已不在盘上的退役条目从账本 files 剔除
        if not ledger_path.is_file():
            print("没有账本可 prune")
            return 2
        led = json.loads(ledger_path.read_text(encoding="utf-8"))
        led, changed = prune_retired(led, vendor)
        if not changed:
            log.append("模式: prune-ledger —— 账本里没有需要剔除的退役条目")
        else:
            log.append("模式: prune-ledger —— 剔除 %d 条退役条目（每条的原地理由写进 retired 账）"
                       % len(changed))
            for c in changed:
                log.append("  " + c)
        # ★ 账本自描述：把「本轮地图引用」写进去（不复制内容，只记路径/sha256/选图理由）。
        #   与 retired 同一次记账，避免出现「账本剔了地图却没说该用哪张」的半截状态。
        if map_ref.get("sha256"):
            old_map = led.get("map_source") or {}
            if old_map.get("sha256") != map_ref["sha256"]:
                led["map_source"] = map_ref
                changed.append("账本写入 map_source（%s %s）"
                               % (pathlib.Path(map_ref["path"]).name, map_ref["sha256"][:20]))
        if changed:
            ledger_path.write_text(json.dumps(led, ensure_ascii=False, indent=2), encoding="utf-8")
            log.append("账本已更新: %s（files %d 条 + retired %d 条 + map_source）"
                       % (ledger_path, len(led["files"]), len(led.get("retired") or [])))
        else:
            log.append("账本无需更新（files %d 条 + retired %d 条，map_source 已一致）"
                       % (len(led["files"]), len(led.get("retired") or [])))
        evid.mkdir(parents=True, exist_ok=True)
        (evid / "prune_log.txt").write_text("\n".join(log) + "\n", encoding="utf-8")
        print("\n".join(log))
        return 0

    if a.verify_only:
        old = json.loads(ledger_path.read_text(encoding="utf-8")) if ledger_path.is_file() else None
        if not old:
            print("没有账本可校验（先跑一次不带 --verify-only）")
            return 2
        log.append("模式: verify-only（账本 vs 真源 vs 产物；含路线内容级对账）")
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
        # ★ 内容级对账：文件哈希之外，再比一次"两侧现役路线是不是同一条"
        route_now = route_record(src, vendor)
        log.append(route_line(route_now))
        route_old = old.get("route") or {}
        if route_old:
            old_sha = (route_old.get("source") or {}).get("sha256", "")
            if old_sha != route_now["source"]["sha256"]:
                drift.append("路线内容  账本 %s -> 车侧现役 %s（mission_blocks 已变）"
                             % (old_sha[:16], route_now["source"]["sha256"][:16]))
            if not route_now["same"]:
                drift.append("路线内容  产物与车侧现役不一致（见上）")
        else:
            log.append("  ! 账本无路线内容指纹（旧版账本）——本次只做了文件哈希对账，"
                       "建议重跑一次同步重建账本")
        # ★ 地图引用校验（卡 WB-9）：账本里记的快照必须仍在盘上且哈希未变。
        #   ★ 旧账本（WB-9 之前）在 `files` 里挂着 `config/map.yaml`（实际文件早已被 WB-1 移出工作树）
        #     ⇒ 那一条只记一条"已退役"的说明，**不**算本仓新增红（上面的 missing 仍如实列它）。
        map_old = old.get("map_source") or {}
        if map_old.get("sha256"):
            p = pathlib.Path(map_old["path"])
            if not p.is_file():
                drift.append("地图引用  账本指的 %s 不存在" % p)
            else:
                now = sha256_file(p)
                if now != map_old["sha256"]:
                    drift.append("地图引用  %s 哈希变了 %s -> %s"
                                 % (p.name, map_old["sha256"][:12], now[:12]))
                else:
                    log.append("地图引用    %s sha256 未变 ✅" % p.name)
        elif map_ref.get("sha256"):
            log.append("  ! 账本无地图引用（WB-9 之前的旧账本）——本次解析到 %s，"
                       "建议重跑一次同步写进账本" % pathlib.Path(map_ref["path"]).name)
        log.append("漂移 %d 条 / 缺失 %d 条" % (len(drift), len(missing)))
        for d in drift + missing:
            log.append("  ! " + d)
        # ★ 与 sync_log.txt 分开写：verify 不覆盖同步日志（v4 审计证据链要求两份都在）
        # ★★ 卡 WB-10-EVIDENCE-20261101：`--verify-only` 的契约是「**只校验，不复制**」，
        #    但它此前每次都无条件覆写 git 跟踪的 `verify_log.txt` —— 一个只读模式在悄悄
        #    替换上一份只读结论（README 还引它作「漂移 0 条/缺失 0 条」的凭证）。
        #    现在默认**不写**；要留档显式加 `--write-evidence`。位置不变、不新增文件。
        if a.write_evidence:
            evid.mkdir(parents=True, exist_ok=True)
            (evid / "verify_log.txt").write_text("\n".join(log) + "\n", encoding="utf-8")
            log.append("证据: %s（--write-evidence 显式要求）" % (evid / "verify_log.txt"))
        else:
            log.append("只读模式：**未写** verify_log.txt（上一份校验结论原样保留；"
                       "要留档加 --write-evidence）")
        print("\n".join(log), flush=True)
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
        "route": res["route"],
        # ★ 地图引用（卡 WB-9）：**不复制内容**，只记「本轮规划该用哪张仓库内快照」。
        #   选 field7 的理由见 map_source.WHY_FIELD7；解析顺序见 map_source 模块头。
        "map_source": map_ref,
        "files": res["files"],
    }
    ledger_path.write_text(json.dumps(ledger, ensure_ascii=False, indent=2), encoding="utf-8")
    log.append("账本: %s（%d 条 + 路线指纹 + 地图引用）" % (ledger_path, res["count"]))
    log.append("结论：%s" % ("逐文件 sha256 一致 + 路线逐 token 一致 ✅"
                            if ledger["all_same"] else "有不一致 ❌（看上面明细）"))
    if map_ref.get("error"):
        log.append("⚠️ 地图引用解析失败（不参与 all_same，但各脚本缺省会因此报错）：%s"
                   % map_ref["error"])

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
                # ★ 地图（卡 WB-9）：不再 scp 地图、不再假设远端有 `config/map.yaml`
                #   —— D-054/D-062 禁止再存第 4/5 份副本，本目录 vendor 已被 WB-1 清空地图。
                #   远端要跑规划必须由人显式给 `--remote-map <机上路径>`（那台机上自有的图），
                #   没给就**如实跳过**，不假装跑过。
                if not a.remote_map:
                    log.append("  ⚠️ 未给 --remote-map：机上规划**未执行**（本目录不再携带地图，"
                               "且禁止为跑一次验证把地图拷到机上）。本地校验结果仍然有效。")
                else:
                    cmd = ("cd %s/dzbrain_ws && python3 tools/plan_route.py --emit tokens "
                           "--config config/robot.yaml --map %s"
                           % (a.remote_dir, a.remote_map))
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

    # ★ 卡 WB-10：目录创建在真正要写的地方（同步是**状态变更操作**，写报告是本职）
    evid.mkdir(parents=True, exist_ok=True)
    (evid / "sync_log.txt").write_text("\n".join(log) + "\n", encoding="utf-8")
    print("\n".join(log))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
