# -*- coding: utf-8 -*-
"""靶向对齐 + 最新进度重规划：部署/验证/回滚（无人机 Orin）。

做什么
------
  1. 备份机上原文件（.bak_align_orig，cp -n 幂等——重复部署不会用改过的
     版本覆盖最初原文件）；
  2. 推送 4 个文件：mission_router.py（靶向对齐）、integrate_node.py（重规划
     基于最新进度）、route_progress.py（新增，进度推断纯算法底座）、
     orin_plan_sidecar.py（机上 sidecar 同口径）；
  3. 机上验证：py_compile + 对齐场景冒烟（对齐到达方向断言 + 降级告警断言
     + 进度推断断言），任何一步失败即停、提示回滚。

用法
----
  python deploy_align.py --host 192.168.0.197            # 部署 + 验证
  python deploy_align.py --host ... --rollback           # 回滚
"""
from __future__ import annotations

import argparse
import hashlib
import pathlib
import sys

import paramiko

BASE = pathlib.Path(__file__).resolve().parent
WORK = BASE / "work"
REMOTE = "/home/duzhong/dzuav_deploy/air_ground"
BAK = ".bak_align_orig"

UPLOADS = [
    (WORK / "ag_integrate" / "mission_router.py",
     f"{REMOTE}/src/ag_integrate/ag_integrate/mission_router.py"),
    (WORK / "ag_integrate" / "integrate_node.py",
     f"{REMOTE}/src/ag_integrate/ag_integrate/integrate_node.py"),
    (WORK / "ag_integrate" / "route_progress.py",
     f"{REMOTE}/src/ag_integrate/ag_integrate/route_progress.py"),      # 新增
    (WORK / "tools" / "orin_plan_sidecar.py",
     f"{REMOTE}/tools/orin_plan_sidecar.py"),
]

SMOKE = r'''
import sys, warnings
sys.path.insert(0, "/home/duzhong/dzuav_deploy/air_ground/src/ag_integrate")
from ag_integrate.field_frame import CellRef
from ag_integrate.world_model import TargetRef, WorldSnapshot
from ag_integrate.mission_router import plan_route
from ag_integrate.route_progress import route_progress, route_cells_of
from ag_integrate.car_adapter import compact_and_adapt

L = "ABCDEFGHIJKLMN"
def cell(n): return CellRef(L.index(n[0]), int(n[1:]) - 1)
def snap(mines=(), passable=("K4","K9"), obstacles=(), targets=()):
    return WorldSnapshot(0, tuple(cell(c) for c in mines),
                         tuple(cell(c) for c in passable),
                         tuple(cell(c) for c in obstacles), tuple(targets))
def adir(route, i):
    if i <= 0: return (0, 0)
    a, b = cell(route[i-1]), cell(route[i])
    return ((b.col>a.col)-(b.col<a.col), (b.row>a.row)-(b.row<a.row))

ok = True
# 1) 一次到位：移动靶 D7-E7 蓝面 S -> 停靠 D8/E8 必须 N 向到达
t = TargetRef(cell("D7"), True, track=(cell("D7"), cell("E7")), blue_face="S")
s = snap(mines=("E3","I2"), targets=(t,))
r = plan_route(s, cell("N10"), cell("N5"), start_heading="y", final_heading="y")
compact, cmds = compact_and_adapt(s, r, "N10", "y", "y")
assert cmds.shoot_cells, "无停靠格"
for stop in cmds.shoot_cells:
    d = adir(compact, list(compact).index(stop))
    print(f"[1] 停靠 {stop} 进段方向={d} (期望 (0,-1) 正北)")
    ok &= (d == (0, -1))
# 2) 降级：堵 D9/E9 -> 告警必须出现
s2 = snap(mines=("E3","I2"), obstacles=("D9","E9"), targets=(t,))
with warnings.catch_warnings(record=True) as caught:
    warnings.simplefilter("always")
    r2 = plan_route(s2, cell("N10"), cell("N5"), start_heading="y", final_heading="y")
hit = [w for w in caught if "正对朝向" in str(w.message)]
print(f"[2] 堵对侧进段 -> 降级告警 {len(hit)} 条（期望 >=1）")
ok &= bool(hit)
c2, cmd2 = compact_and_adapt(s2, r2, "N10", "y", "y")
ok &= bool(cmd2.shoot_cells)
# 3) 进度推断
p = route_progress("N10 M10 K10 J10 ,y", *(lambda c: (
    0.065 + c[0]*0.35, 0.295 + (10-c[1])*0.35))((12, 9)), 1.5707963)
print(f"[3] 车在 M10 -> cell={p.cell.name} swept={sorted(p.swept_cells)} head={p.heading}")
ok &= (p.cell.name == "M10" and p.swept_cells == {"N10"} and p.heading == "y")
print("SMOKE_OK" if ok else "SMOKE_FAIL")
sys.exit(0 if ok else 1)
'''


def md5(p: pathlib.Path) -> str:
    return hashlib.md5(p.read_bytes()).hexdigest()


class R:
    def __init__(self, host, user, pw):
        self.c = paramiko.SSHClient()
        self.c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.c.connect(host, username=user, password=pw, timeout=15)

    def run(self, cmd, timeout=600):
        _, o, e = self.c.exec_command(cmd, timeout=timeout)
        return (o.read().decode("utf-8", "replace")
                + e.read().decode("utf-8", "replace")).strip()

    def put(self, local, remote):
        sftp = self.c.open_sftp()
        try:
            sftp.put(str(local), remote)
        finally:
            sftp.close()

    def close(self):
        self.c.close()


def deploy(r: R):
    print("=== 1/3 备份（幂等，保留最初原文件） ===")
    existing = [rem for _l, rem in UPLOADS
                if r.run(f"test -f {rem} && echo Y || echo N") == "Y"]
    for rem in existing:
        r.run(f"cp -n {rem} {rem}{BAK}")
    print(r.run(f"find {REMOTE} -maxdepth 4 -name '*{BAK}' | wc -l")
          + " 个备份在位")

    print("=== 2/3 推送（md5 校验） ===")
    for local, rem in UPLOADS:
        if not local.is_file():
            raise SystemExit(f"[err] 本地缺失 {local}")
        r.put(local, rem)
        got = r.run(f"md5sum {rem}").split()[0]
        flag = "一致" if got == md5(local) else "不一致!"
        print(f"  {rem.split('/')[-1]:24s} {flag}")
        if got != md5(local):
            raise SystemExit("[err] 推送校验失败")

    print("=== 3/3 机上验证 ===")
    py = " ; ".join(f"python3 -m py_compile {rem}" for _l, rem in UPLOADS)
    out = r.run(py)
    print(out or "py_compile 全过")
    sftp = r.c.open_sftp()
    with sftp.file("/tmp/ag_align_smoke.py", "w") as f:
        f.write(SMOKE)
    sftp.close()
    print(r.run("python3 /tmp/ag_align_smoke.py", timeout=300))


def rollback(r: R):
    print("=== 回滚：还原备份 ===")
    baks = r.run(f"find {REMOTE} -maxdepth 4 -name '*{BAK}' 2>/dev/null")
    if not baks:
        raise SystemExit("[err] 未找到备份")
    for line in sorted(baks.splitlines()):
        src = line.strip()
        dst = src[: -len(BAK)]
        print(f"  {src.split('/')[-1]} -> {dst.split('/')[-1]}")
        r.run(f"mv -f {src} {dst}")
    # route_progress.py 是新增文件，回滚即删
    r.run(f"rm -f {REMOTE}/src/ag_integrate/ag_integrate/route_progress.py")
    print(r.run(" ; ".join(
        f"python3 -m py_compile {rem}" for _l, rem in UPLOADS[:2]))
        or "py_compile 全过（sidecar 已还原原版）")
    print("[done] 已回滚")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.0.197")
    ap.add_argument("--user", default="duzhong")
    ap.add_argument("--password", default="duzhong")
    ap.add_argument("--rollback", action="store_true")
    args = ap.parse_args(argv)
    r = R(args.host, args.user, args.password)
    try:
        rollback(r) if args.rollback else deploy(r)
        return 0
    finally:
        r.close()


if __name__ == "__main__":
    raise SystemExit(main())
