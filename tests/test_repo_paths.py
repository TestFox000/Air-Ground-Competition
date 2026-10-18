# -*- coding: utf-8 -*-
"""网页上传路径污染防线（2026-10-18 两次事故后建制）。

Gitee 网页上传文件夹/zip 会产生两类损坏路径：
  ① 文件名内反斜杠（目录分隔被写进名字）：视频_20261018\\cam_xxx\\file.png
  ② 每个路径组件尾部被塞回车符：docs\\r/成员交接\\r/README.md\\r
Windows 端 merge/checkout 直接报 invalid path，全员无法协作。
本测试扫描全部已跟踪路径，凡含控制字符/Windows 非法字符/组件尾点或空格即判红。

修复方法：python tools/repair_web_upload.py <分支> --push（blob 零改动的纯路径重建）。
"""
import subprocess
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

# 控制字符（0x00-0x09、0x0B-0x1F；0x0A 是 ls-files -z 的分隔符不可能出现在路径里）
# + Windows 文件名非法字符 + 反斜杠
_BAD_CHARS = set(chr(c) for c in list(range(0x00, 0x0A)) + list(range(0x0B, 0x20)))
_BAD_CHARS |= set('<>:"|?*\\')


def _tracked_paths():
    r = subprocess.run(["git", "-C", str(REPO), "ls-files", "-z"],
                       capture_output=True, check=True)
    return [p for p in r.stdout.decode("utf-8").split("\0") if p]


def test_no_polluted_paths():
    """全仓已跟踪路径不得含控制字符/Windows 非法字符/组件尾点或空格。

    此类污染几乎都来自 Gitee 网页上传文件夹/zip（详见 tools/repair_web_upload.py
    头注释与 AGENTS.md 网页上传禁令）。别绕过测试硬推——修复工具一条命令：
    python tools/repair_web_upload.py origin/<你的分支> --push
    """
    polluted = []
    for p in _tracked_paths():
        if any(c in _BAD_CHARS for c in p):
            polluted.append(p)
            continue
        for comp in p.split("/"):
            if comp.endswith(".") or comp.endswith(" ") or comp.endswith("\r"):
                polluted.append(p)
                break
    assert not polluted, (
        "检测到 %d 条网页上传污染路径（含控制字符/非法字符/反斜杠/组件尾点空格），"
        "Windows 端全员无法合并。修复：python tools/repair_web_upload.py origin/<分支> --push"
        "（blob 零改动）。前 10 条：\n  " + "\n  ".join(repr(p) for p in polluted[:10])
    )
