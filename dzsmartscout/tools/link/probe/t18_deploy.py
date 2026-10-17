#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
t18_deploy.py —— t18 的「重跑一次部署 + 机上 md5 复核 + 哈希清单」

为什么单独写：t15 复审 F1 指出「复核的版本 ≠ 部署面对的那个版本」。
本脚本把「部署 → 逐文件哈希复核 → 机上自检 → 输出被复核文件哈希清单」做成一次可复跑的动作，
清单格式按 `dzrobot_v2/docs/回归守护_known_reds.md` §4 第 5 条：
每文件一条 = 路径 + mtime + SHA256 前 20 位 + 是否=交付版（这里再加 md5 便于与历史台账对齐）。

用法: python dzsmartscout/tools/link/probe/t18_deploy.py
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
OUTDIR = os.path.join(ROOT, "dzsmartscout", "tools", "reports", "t18-evidence")
REMOTE = "/home/duzhong/dzlink_tools"

PAIRS = [
    (os.path.join(LINK, "dzlink_supervisor.py"), REMOTE + "/dzlink_supervisor.py"),
    (os.path.join(LINK, "dz_send_waypoint.py"), REMOTE + "/dz_send_waypoint.py"),
    (os.path.join(LINK, "tests", "test_dzlink_supervisor.py"), REMOTE + "/test_dzlink_supervisor.py"),
    (os.path.join(LINK, "tests", "test_dz_send_waypoint.py"), REMOTE + "/test_dz_send_waypoint.py"),
]


def md5_of(local):
    data = open(local, "rb").read().replace(b"\r\n", b"\n")
    return hashlib.md5(data).hexdigest(), hashlib.sha256(data).hexdigest(), len(data)


def main():
    os.makedirs(OUTDIR, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    bak = "%s/_backups/t18_%s" % (REMOTE, ts)
    c = S.connect("drone")
    out = []
    try:
        rc, o = S.run(c, "mkdir -p %s && cp -p %s/dzlink_supervisor.py %s/dz_send_waypoint.py "
                         "%s/test_dzlink_supervisor.py %s/test_dz_send_waypoint.py %s/ 2>/dev/null; "
                         "echo '--- 备份目录 %s ---'; md5sum %s/*.py"
                      % (bak, REMOTE, REMOTE, REMOTE, REMOTE, bak, bak, bak))
        out.append("=== 部署前（当前 hub 内容；同时备份到 %s）===" % bak)
        out.append("  [注·t19-G1] 标题里的“部署前”指**内容版本**。备份目录**名**用的是**本机(Windows)时钟**，")
        out.append("             hub 文件 mtime 用的是**无人机时钟**（实测相差 ~1.4 s，无人机偏慢）")
        out.append("             ⇒ 目录名看起来会比部署后的文件 mtime 还晚 ~2 s（“备份目录建于部署后”的观感）。")
        out.append("             实际 `mkdir+cp` 在 scp **之前**执行：目录 mtime 20:09:16.573")
        out.append("             < hub 文件 mtime 20:09:16.721（真机实测，见 docs/probe t19_timestamps.sh）。")
        out.append("  [注·t19-G1] 因此本段 sender 副本 = **部署前版**（`893e7b9f`，sender 本轮才被修）；")
        out.append("             supervisor 副本 = **未改的终版**（`5c7426d6`，本轮未改）")
        out.append("             → 与下面【部署后】段（hub 实测 `eb41dbf4`）**不矛盾**（不同对象）。")
        out.append(o)

        out.append("=== 上传（本地 → hub）===")
        local_md5 = {}
        for local, remote in PAIRS:
            n = S.put(c, local, remote)
            m5, s256, sz = md5_of(local)
            local_md5[remote] = m5
            line = "  %-52s %6d B  md5=%s  sha256[:20]=%s" % (remote, n, m5, s256[:20])
            out.append(line)
            print(line)

        out.append("=== 部署后（hub 实测 md5）===")
        rc, o = S.run(c, "md5sum " + " ".join(r for _, r in PAIRS))
        out.append(o)
        ok = all(local_md5[r] in o for _, r in PAIRS)
        out.append("=== md5 逐文件一致：%s ===" % ("YES" if ok else "NO"))
        print("md5 一致:", ok)

        rc, o = S.run(c, "python3 -m py_compile %s/dzlink_supervisor.py %s/dz_send_waypoint.py "
                         "&& echo PY_COMPILE_OK" % (REMOTE, REMOTE))
        out.append("=== hub py_compile ===")
        out.append(o)

        rc, o = S.run(c, "cd %s && echo '-- supervisor --' && python3 test_dzlink_supervisor.py 2>&1 "
                         "| grep -E '^Ran |^OK$|^FAILED' && echo '-- sender --' && "
                         "python3 test_dz_send_waypoint.py 2>&1 | grep -E '^Ran |^OK$|^FAILED'" % REMOTE,
                      timeout=300)
        out.append("=== hub 单测 ===")
        out.append(o)
        print(o)

        # 被复核文件哈希清单（known_reds §4 第 5 条口径）
        rc, o = S.run(c, "cd %s && for f in dzlink_supervisor.py dz_send_waypoint.py "
                         "test_dzlink_supervisor.py test_dz_send_waypoint.py; do "
                         "printf '%%s  %%s  %%s  %%s\\n' \"$f\" \"$(stat -c %%y $f)\" "
                         "\"$(md5sum $f | cut -d' ' -f1)\" \"$(sha256sum $f | cut -c1-20)\"; done" % REMOTE)
        out.append("=== 被复核文件哈希清单（路径 + mtime + md5 + sha256[:20]）===")
        out.append(o)
        print(o)
    finally:
        c.close()
        with open(os.path.join(OUTDIR, "deploy_and_hashes.txt"), "w", encoding="utf-8",
                  errors="replace") as f:
            f.write("\n".join(out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
