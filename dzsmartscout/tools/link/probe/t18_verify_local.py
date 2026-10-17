#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
t18_verify_local.py —— t18 本机验证 + hub 最终哈希复核（一条命令，输出落证据目录）

用法: python dzsmartscout/tools/link/probe/t18_verify_local.py
"""
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import t4_ssh as S  # noqa: E402

ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
LINK = os.path.join(ROOT, "dzsmartscout", "tools", "link")
OUT = os.path.join(ROOT, "dzsmartscout", "tools", "reports", "t18-evidence")
PY = sys.executable

CASES = [
    ("A1 py_compile", [PY, "-m", "py_compile",
                       os.path.join(LINK, "dzlink_supervisor.py"),
                       os.path.join(LINK, "dz_send_waypoint.py")]),
    ("A2 supervisor 单测", [PY, os.path.join(LINK, "tests", "test_dzlink_supervisor.py")]),
    ("A3 sender 单测", [PY, "-m", "unittest",
                        "dzsmartscout.tools.link.tests.test_dz_send_waypoint"]),
    # 契约 Verify 原文（笔误，保留）：sender 没有 run 子命令
    ("B1 契约原文 dz_send_waypoint.py run --bridge …（照字面）",
     [PY, os.path.join(LINK, "dz_send_waypoint.py"), "run", "--bridge", "lower=K9",
      "--bridge", "upper=K4", "--duration", "3", "--dry-run"]),
    # 等价写法①：位置参数并列
    ("B2 等价写法① bridge lower=K9 upper=K4 --dry-run",
     [PY, os.path.join(LINK, "dz_send_waypoint.py"), "bridge", "lower=K9", "upper=K4", "--dry-run"]),
    # 等价写法②：分两次调用
    ("B3 等价写法②-1 bridge lower=K9 --dry-run",
     [PY, os.path.join(LINK, "dz_send_waypoint.py"), "bridge", "lower=K9", "--dry-run"]),
    ("B4 等价写法②-2 bridge upper=K4 --dry-run",
     [PY, os.path.join(LINK, "dz_send_waypoint.py"), "bridge", "upper=K4", "--dry-run"]),
    # supervisor 侧的语义等价命令（契约真正想测的那条）
    ("B5 dzlink_supervisor.py run --bridge lower=K9 --bridge upper=K4 --duration 3 --dry-run",
     [PY, os.path.join(LINK, "dzlink_supervisor.py"), "run", "--bridge", "lower=K9",
      "--bridge", "upper=K4", "--duration", "3", "--dry-run"]),
    ("C1 target G10 H10 --dry-run",
     [PY, os.path.join(LINK, "dz_send_waypoint.py"), "target", "G10", "H10", "--dry-run"]),
    ("C2 field --mines A3 B4 --dry-run",
     [PY, os.path.join(LINK, "dz_send_waypoint.py"), "field", "--mines", "A3", "B4", "--dry-run"]),
    # 错用法 → 报错文案应指向本发送器的两种写法
    ("D1 bridge lower=K9,K4 --dry-run（错用法，应 exit 1 + 明确写法）",
     [PY, os.path.join(LINK, "dz_send_waypoint.py"), "bridge", "lower=K9,K4", "--dry-run"]),
]


def main():
    os.makedirs(OUT, exist_ok=True)
    log = ["### t18 本机验证 + hub 哈希复核   " + time.strftime("%Y-%m-%d %H:%M:%S")]
    for name, cmd in CASES:
        p = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True,
                           encoding="utf-8", errors="replace")
        log.append("\n--- %s ---" % name)
        log.append("$ " + " ".join(os.path.basename(c) if c == PY else c for c in cmd))
        tail = (p.stdout or "").strip().splitlines()[-6:]
        log += ["  " + t for t in tail]
        for t in (p.stderr or "").strip().splitlines()[-6:]:
            log.append("  [stderr] " + t)
        log.append("  EXIT=%d" % p.returncode)
        print("%-58s EXIT=%d" % (name, p.returncode))

    log.append("\n--- E) hub 最终 md5 / mtime（t18 部署后，复核期间不再写入）---")
    c = S.connect("drone")
    try:
        rc, out = S.run(c, "md5sum /home/duzhong/dzlink_tools/dzlink_supervisor.py "
                           "/home/duzhong/dzlink_tools/dz_send_waypoint.py "
                           "/home/duzhong/dzlink_tools/test_dzlink_supervisor.py "
                           "/home/duzhong/dzlink_tools/test_dz_send_waypoint.py; "
                           "stat -c '%y  %n' /home/duzhong/dzlink_tools/dzlink_*.py "
                           "/home/duzhong/dzlink_tools/test_dz*.py")
        log.append(out)
        print(out)
    finally:
        c.close()
    with open(os.path.join(OUT, "verify_local.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(log))

    # ── t19-G1：证据文件拆分（hub_final_md5.txt 只放 md5/mtime；sender 报错抓取单独一份）──
    with open(os.path.join(OUT, "hub_final_md5.txt"), "w", encoding="utf-8") as f:
        f.write("# hub `~/dzlink_tools/` 最终 md5 / mtime（t18 部署后实测；复核期间不再写入）\n")
        f.write("# 被审版本：dzlink_supervisor.py = 5c7426d6b619da2f4b69bb3149de2d53\n")
        f.write("#           dz_send_waypoint.py  = eb41dbf4b944bc7e8af8e61ec940a228\n")
        f.write("# 生成：python dzsmartscout/tools/link/probe/t18_verify_local.py（E 段）\n")
        f.write("# 注：本文件**只**放 hub 的哈希/时间；sender 错用法的报错抓取见\n")
        f.write("#     sender_badusage_stderr.txt（t19-G1 拆分）\n\n")
        f.write(out.strip() + "\n")

    d1 = next((c_ for c_ in CASES if c_[0].startswith("D1")), None)
    with open(os.path.join(OUT, "sender_badusage_stderr.txt"), "w", encoding="utf-8") as f:
        f.write("# sender 错用法 `bridge lower=K9,K4` 的完整抓取（t18/F2 改后的报错文案）\n")
        f.write("# 期望：exit=1；提示本发送器**可执行**的两种写法，并注明 --bridge 属 supervisor\n")
        if d1:
            p = subprocess.run(d1[1], cwd=ROOT, capture_output=True, text=True,
                               encoding="utf-8", errors="replace")
            f.write("\n$ %s\n" % " ".join(d1[1]))
            f.write("\n[stdout]\n%s\n" % (p.stdout or "").strip())
            f.write("\n[stderr]\n%s\n" % (p.stderr or "").strip())
            f.write("EXIT=%d\n" % p.returncode)
    print("\n### 证据写入 %s（t19-G1 已拆分 hub_final_md5.txt / sender_badusage_stderr.txt）" % OUT)
    return 0


if __name__ == "__main__":
    sys.exit(main())
