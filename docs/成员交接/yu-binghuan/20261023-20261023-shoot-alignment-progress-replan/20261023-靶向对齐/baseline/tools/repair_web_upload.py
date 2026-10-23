# -*- coding: utf-8 -*-
"""repair_web_upload.py — Gitee 网页上传路径污染 · 检测与自动修复。

背景（2026-10-18 两次事故）：Gitee 网页上传文件夹/zip 会产生两类损坏路径——
  ① 目录分隔被写成文件名内的反斜杠：  视频_20261018\\cam_xxx\\file.png
  ② 每个路径组件尾部被塞入回车符：    docs\\r/成员交接\\r/README.md\\r
Windows 端 git 报 invalid path（控制字符显示为 ?），任何人无法 merge/checkout。

用法：
  python tools/repair_web_upload.py                     # 检查 origin/members/yu-binghuan（默认只读）
  python tools/repair_web_upload.py origin/main         # 检查指定 ref
  python tools/repair_web_upload.py origin/main --push  # 修复并推送（FF，blob 零改动）

原理：Windows 端 read-tree/merge 会拒绝损坏路径，但 ls-tree 读取与 mktree 创建
树对象不做 OS 路径校验——因此可绕开工作区/索引，纯树对象完成路径规范化。
注意：喂 mktree 的 stdin 必须用字节模式（Windows text 模式会把 \\n 翻译成 \\r\\n，
把 CR 重新注入文件名——第一次修复失败的同源教训）。
"""
import argparse
import os
import subprocess
import sys
from collections import namedtuple

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

CmdResult = namedtuple("CmdResult", "returncode stdout stderr")


def g(*args, **kw):
    """字节模式运行 git（stdin 不做换行翻译）。"""
    inp = kw.get("input")
    r = subprocess.run(
        ["git", "-C", REPO, "-c", "core.quotepath=false"] + list(args),
        capture_output=True,
        input=inp.encode("utf-8") if isinstance(inp, str) else inp,
    )
    return CmdResult(r.returncode, r.stdout.decode("utf-8", "replace"),
                     r.stderr.decode("utf-8", "replace"))


def unquote(p):
    """还原 git C 引号：支持 \\\\ \\" \\t \\r \\n \\a \\b \\f \\v \\0 与八进制。"""
    if not (p.startswith('"') and p.endswith('"')):
        return p
    simple = {"\\": "\\", '"': '"', "t": "\t", "r": "\r", "n": "\n",
              "a": "\a", "b": "\b", "f": "\f", "v": "\v", "0": "\0"}
    inner, out, i = p[1:-1], [], 0
    while i < len(inner):
        c = inner[i]
        if c == "\\" and i + 1 < len(inner):
            n = inner[i + 1]
            if n in simple:
                out.append(simple[n]); i += 2; continue
            if n in "1234567":
                digits, j = "", i + 1
                while j < len(inner) and len(digits) < 3 and inner[j] in "01234567":
                    digits += inner[j]; j += 1
                out.append(chr(int(digits, 8))); i = j; continue
        out.append(c); i += 1
    return "".join(out)


INVALID_CTRL = set(chr(c) for c in list(range(0x00, 0x0A)) + list(range(0x0B, 0x20)))
INVALID_WIN = set('<>:"|?*\\')


def is_polluted(path):
    """路径任一组件含控制字符/Windows 非法字符，或组件尾部为点/空格。"""
    for comp in path.split("/"):
        if any(c in INVALID_CTRL or c in INVALID_WIN for c in comp):
            return True
        if comp.endswith(".") or comp.endswith(" "):
            return True
    return False


def clean_path(p):
    """剥离组件尾部控制字符/空白；组件内部反斜杠拆为真实目录。"""
    comps = []
    for comp in p.split("/"):
        comp = comp.rstrip("\r\n\t \x00")
        if not comp:
            return None
        if "\\" in comp:
            comps.extend([c for c in comp.split("\\") if c])
        else:
            comps.append(comp)
    return "/".join(comps)


def tree_entries(rev):
    r = g("ls-tree", "-r", rev)
    if r.returncode != 0:
        print("[失败] 无法读取 %s：%s" % (rev, r.stderr.strip()))
        sys.exit(1)
    ents = {}
    for line in r.stdout.splitlines():
        meta, path = line.split("\t", 1)
        mode, typ, sha = meta.split()
        ents[unquote(path)] = (mode, sha)
    return ents


def mktree(children):
    lines = ["%s %s %s\t%s" % (m, "tree" if m == "040000" else "blob", s, n)
             for n, m, s in children]
    r = g("mktree", input="\n".join(lines) + "\n")
    if r.returncode != 0:
        print("[失败] mktree：%s" % r.stderr.strip())
        sys.exit(1)
    return r.stdout.strip()


def main():
    ap = argparse.ArgumentParser(description="Gitee 网页上传路径污染检测/修复")
    ap.add_argument("ref", nargs="?", default="origin/members/yu-binghuan",
                    help="目标 ref（默认 origin/members/yu-binghuan）")
    ap.add_argument("--push", action="store_true", help="修复并推送（默认只检查）")
    args = ap.parse_args()

    ents = tree_entries(args.ref)
    polluted = {p: v for p, v in ents.items() if is_polluted(p) or clean_path(p) != p}
    print("目标：%s | 条目 %d | 污染 %d" % (args.ref, len(ents), len(polluted)))
    for p in list(polluted)[:10]:
        print("    ", repr(p))
    if len(polluted) > 10:
        print("     ...（其余 %d 条省略）" % (len(polluted) - 10))

    if not polluted:
        print("[干净] 无需修复")
        return 0

    # 清洗 + 冲突检查
    cleaned = {}
    for p, (mode, sha) in ents.items():
        cp = clean_path(p)
        if cp is None:
            print("[失败] 组件清洗后为空，需人工处理：%r" % p)
            return 1
        if cp in cleaned and cleaned[cp][1] != sha:
            print("[失败] 清洗后冲突（同路径不同内容），需人工处理：%r -> %s" % (p, cp))
            return 1
        cleaned[cp] = (mode, sha)
    residual = [p for p in cleaned if is_polluted(p)]
    if residual:
        print("[失败] 清洗后仍有非法路径，需人工处理：%r" % residual[0])
        return 1

    if not args.push:
        print("[待修] %d 条污染路径。确认后加 --push 执行修复（blob 零改动，纯路径重建）" % len(polluted))
        return 2

    # 自底向上重建树
    cache = {}

    def build(D):
        if D in cache:
            return cache[D]
        children, child_dirs = [], set()
        for path, (mode, sha) in cleaned.items():
            d = os.path.dirname(path)
            if d == D:
                children.append((os.path.basename(path), mode, sha))
            else:
                rest = path[len(D) + 1:] if D else path
                if (D == "" or d.startswith(D + "/")) and "/" in rest:
                    child_dirs.add(rest.split("/", 1)[0])
        for sub in child_dirs:
            children.append((sub, "040000", build((D + "/" + sub) if D else sub)))
        cache[D] = mktree(children)
        return cache[D]

    root = build("")
    msg = ("fix(协作): 自动修复网页上传路径污染（%d/%d 条目，blob 零改动）\n\n"
           "由 tools/repair_web_upload.py 以纯树对象操作重建：剥离控制字符/拆分反斜杠"
           "组件，文件内容逐一原样保留。网页上传文件夹/zip 会复现此问题——请用 git 推送。"
           % (len(polluted), len(ents)))
    env = dict(os.environ,
               GIT_AUTHOR_NAME="trae-glm", GIT_AUTHOR_EMAIL="trae-glm@air-ground.team",
               GIT_COMMITTER_NAME="trae-glm", GIT_COMMITTER_EMAIL="trae-glm@air-ground.team")
    c = subprocess.run(["git", "-C", REPO, "commit-tree", root, "-p", args.ref],
                       capture_output=True, input=msg.encode("utf-8"), env=env)
    if c.returncode != 0:
        print("[失败] commit-tree：%s" % c.stderr.decode("utf-8", "replace"))
        return 1
    newsha = c.stdout.decode("utf-8").strip()
    print("修复提交：%s（父：%s）" % (newsha, args.ref))
    p = subprocess.run(["git", "-C", REPO, "push", "origin",
                        newsha + ":" + args.ref.replace("origin/", "refs/heads/", 1)],
                       capture_output=True)
    if p.returncode != 0:
        print("[失败] 推送被拒（远端可能又推进了）：%s\n先 git fetch 后重跑本工具。"
              % p.stderr.decode("utf-8", "replace")[-300:])
        return 1
    print("[完成] 已推送。请相关人员 git fetch 后重新合并；CI 的 test_repo_paths 会持续拦截此类污染。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
