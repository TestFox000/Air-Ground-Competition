# -*- coding: utf-8 -*-
"""一键归档成果物到 Gitee 个人档。

默认落点：仓库 circle-of-joy/air_ground 的 members/yu-binghuan 分支，
目录 docs/成员交接/yu-binghuan/<日期>-<slug>/。

用法：
    python push_deliverable.py --meta <meta.json>

meta.json（UTF-8）字段：
    title    必填，成果标题
    slug     必填，目录短名（英文/数字/连字符）
    files    必填，文件或目录路径列表（目录按 include 通配递归收集）
    include  可选，通配列表，默认 ["*.png","*.json","*.md","*.txt"]
    notes    可选，说明段落（可多行）
    branch   可选，默认 members/yu-binghuan
    dest     可选，默认 docs/成员交接/yu-binghuan
    day      可选，默认今天（YYYYMMDD）

Token 从环境变量 GITEE_TOKEN 读取，绝不写入任何文件。
"""
from __future__ import annotations

import argparse
import base64
import datetime as _dt
import fnmatch
import json
import os
import sys
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

REPO = "circle-of-joy/air_ground"
API = f"https://gitee.com/api/v5/repos/{REPO}/contents/"


def _api(method: str, repo_path: str, token: str, payload: dict | None = None):
    url = API + urllib.parse.quote(repo_path) + "?access_token=" + urllib.parse.quote(token)
    data = urllib.parse.urlencode(payload).encode() if payload else None
    req = urllib.request.Request(url, data=data, method=method,
                                 headers={"User-Agent": "Mozilla/5.0",
                                          "Content-Type": "application/x-www-form-urlencoded"})
    try:
        with urllib.request.urlopen(req, timeout=90) as r:
            return json.load(r)
    except urllib.error.HTTPError as e:
        body = e.read().decode("utf-8", "replace")
        if e.code == 404:
            return None
        raise RuntimeError(f"HTTP {e.code} {repo_path}: {body[:200]}") from None


def _get_remote(repo_path: str, branch: str, token: str) -> tuple[str, bytes | None] | None:
    """查云端已有文件，返回 (sha, 内容字节)；不存在返回 None。

    内容可能因超出阈值拿不到（encoding=none），此时内容位为 None。
    """
    url = (API + urllib.parse.quote(repo_path)
           + "?access_token=" + urllib.parse.quote(token)
           + "&ref=" + urllib.parse.quote(branch))
    req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
    try:
        with urllib.request.urlopen(req, timeout=60) as r:
            data = json.load(r)
            # Gitee 对不存在的深层路径返回空列表（而非 404）——按"不存在"处理
            if not isinstance(data, dict):
                return None
            sha = data.get("sha")
            raw = data.get("content")
            content: bytes | None = None
            if raw and data.get("encoding") == "base64":
                try:
                    content = base64.b64decode(raw)
                except Exception:
                    content = None
            return (sha, content)
    except urllib.error.HTTPError as e:
        if e.code == 404:
            return None
        raise RuntimeError(f"HTTP {e.code} GET {repo_path}") from None


def upload(repo_path: str, local: Path, branch: str, token: str, message: str) -> str | None:
    """新增或更新单文件，返回云端 sha；内容与云端完全相同时跳过并返回 None。"""
    data = local.read_bytes()
    remote = _get_remote(repo_path, branch, token)
    if remote and remote[1] == data:
        return None  # 内容一致——幂等跳过，不产生提交
    b64 = base64.b64encode(data).decode()
    payload = {"content": b64, "message": message, "branch": branch}
    if remote:
        payload["sha"] = remote[0]
        res = _api("PUT", repo_path, token, payload)
    else:
        res = _api("POST", repo_path, token, payload)
    return res["content"]["sha"]


def collect(files: list[str], include: list[str]) -> list[tuple[Path, str]]:
    """返回 [(本地路径, 相对归档路径)]。归档路径统一用正斜杠。"""
    out: list[tuple[Path, str]] = []
    for raw in files:
        p = Path(raw)
        if p.is_dir():
            for f in sorted(p.rglob("*")):
                if f.is_file() and any(fnmatch.fnmatch(f.name, g) for g in include):
                    rel = f.relative_to(p.parent).as_posix()
                    out.append((f, rel.replace("\\", "/")))
        elif p.is_file():
            out.append((p, p.name))
        else:
            raise FileNotFoundError(raw)
    return out


def build_readme(title: str, day: str, slug: str, items: list[tuple[Path, str]],
                 notes: str) -> str:
    lines = [f"# {title}", "",
             f"- 归档日期：{day[:4]}-{day[4:6]}-{day[6:]}",
             f"- 归档目录：`docs/成员交接/yu-binghuan/{day}-{slug}/`",
             f"- 归档工具：`push_deliverable.py`（一键归档，见同目录说明）", ""]
    if notes.strip():
        lines += ["## 说明", "", notes.strip(), ""]
    lines += ["## 文件清单", "", "| 文件 | 大小 |", "|---|---|"]
    for lp, rel in items:
        lines.append(f"| `{rel}` | {lp.stat().st_size:,} B |")
    lines += ["", "## 复现方式", "",
              "```bash",
              "# 视频侦察管线（Windows 可直接跑，无需 ROS）",
              "python video_field_recon.py <video.mp4> --stride 15 --out <输出目录>",
              "python render_waypoint_map.py <输出目录>",
              "```", "",
              "> 本 README 由 push_deliverable.py 自动生成。"]
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--meta", required=True, help="UTF-8 元数据 JSON 路径")
    ap.add_argument("--dry-run", action="store_true",
                    help="只列出将上传的文件，不联网、不产生提交")
    args = ap.parse_args()

    meta = json.loads(Path(args.meta).read_text(encoding="utf-8"))
    title = meta["title"]
    slug = meta["slug"]
    day = str(meta.get("day") or _dt.date.today().strftime("%Y%m%d"))
    branch = meta.get("branch", "members/yu-binghuan")
    dest = meta.get("dest", "docs/成员交接/yu-binghuan")
    include = meta.get("include") or ["*.png", "*.json", "*.md", "*.txt"]
    notes = meta.get("notes", "")
    items = collect(meta["files"], include)
    if not items:
        print("[push] 未匹配到任何文件")
        return 2

    root = f"{dest}/{day}-{slug}"
    rp = f"{root}/README.md"
    readme = build_readme(title, day, slug, items, notes).encode()
    tmp = Path(args.meta).with_name("_readme_tmp.md")
    tmp.write_bytes(readme)

    if args.dry_run:
        print(f"[push] DRY-RUN 目标 {REPO}@{branch}:{root}  共 {len(items) + 1} 个文件（含 README）")
        for _, rel in items:
            print(f"  WOULD {rel}")
        print("  WOULD README.md")
        tmp.unlink(missing_ok=True)
        return 0

    token = os.environ.get("GITEE_TOKEN", "")
    if not token:
        print("[push] 缺少环境变量 GITEE_TOKEN")
        tmp.unlink(missing_ok=True)
        return 2

    msg = f"docs(交接): {title}"
    print(f"[push] 目标 {REPO}@{branch}:{root}  共 {len(items) + 1} 个文件（含 README）")
    changed = skipped = 0
    for lp, rel in items:
        sha = upload(f"{root}/{rel}", lp, branch, token, msg)
        if sha is None:
            skipped += 1
            print(f"  SKIP {rel}（内容未变，不产生提交）")
        else:
            changed += 1
            print(f"  OK {rel}  sha={sha[:10]}")
    sha = upload(rp, tmp, branch, token, msg)
    if sha is None:
        skipped += 1
        print("  SKIP README.md（内容未变，不产生提交）")
    else:
        changed += 1
        print(f"  OK README.md  sha={sha[:10]}")
    tmp.unlink(missing_ok=True)
    if changed == 0:
        print(f"[push] 全部 {skipped} 个文件内容未变，零提交（幂等）")
        return 0
    print(f"[push] 完成：{changed} 提交 / {skipped} 跳过 → "
          f"https://gitee.com/{REPO}/tree/{branch}/{urllib.parse.quote(root)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
