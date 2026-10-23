#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
t16_deploy.py —— 把 t16 修订版 supervisor + 单测部署到无人机（备份 → scp → md5 复核）

用法: python dzsmartscout/tools/link/probe/t16_deploy.py
输出每一步的 md5，便于在报告里写「部署前/后」对照；旧文件备份在
`~/dzlink_tools/_backups/t16_<时间戳>/`。
"""
import hashlib
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import t4_ssh as S  # noqa: E402

ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
LINK = os.path.join(ROOT, "dzsmartscout", "tools", "link")
REMOTE = "/home/duzhong/dzlink_tools"

PAIRS = [
    (os.path.join(LINK, "dzlink_supervisor.py"), REMOTE + "/dzlink_supervisor.py"),
    (os.path.join(LINK, "tests", "test_dzlink_supervisor.py"), REMOTE + "/test_dzlink_supervisor.py"),
]


def main():
    ts = time.strftime("%Y%m%d_%H%M%S")
    bak = "%s/_backups/t16_%s" % (REMOTE, ts)
    c = S.connect("drone")
    try:
        rc, out = S.run(c, "mkdir -p %s && cp -p %s/dzlink_supervisor.py "
                           "%s/test_dzlink_supervisor.py %s/ 2>/dev/null; "
                           "echo '--- 备份目录 ---'; ls -l %s; echo '--- 部署前 md5 ---'; "
                           "md5sum %s/dzlink_supervisor.py %s/test_dzlink_supervisor.py"
                        % (bak, REMOTE, REMOTE, bak, bak, REMOTE, REMOTE))
        print(out)
        print("=== 上传 ===")
        local_md5 = {}
        for local, remote in PAIRS:
            n = S.put(c, local, remote)
            h = hashlib.md5(open(local, "rb").read().replace(b"\r\n", b"\n")).hexdigest()
            local_md5[remote] = h
            print("  %-52s %6d bytes  local_md5=%s" % (remote, n, h))
        print("=== 部署后 md5（远端）===")
        rc, out = S.run(c, "md5sum " + " ".join(r for _, r in PAIRS))
        print(out)
        ok = all(local_md5[r] in out for _, r in PAIRS)
        print("=== md5 逐文件一致：%s ===" % ("YES" if ok else "NO"))
        rc, out = S.run(c, "python3 -m py_compile %s/dzlink_supervisor.py && echo PY_COMPILE_OK"
                           % REMOTE)
        print(out.strip())
        rc, out = S.run(c, "cd %s && python3 test_dzlink_supervisor.py 2>&1 | tail -3" % REMOTE,
                        timeout=300)
        print("=== 机上单测 ===")
        print(out)
        return 0 if ok else 1
    finally:
        c.close()


if __name__ == "__main__":
    sys.exit(main())
