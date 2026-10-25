#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""dzdoc MCP server —— 把现有只读文档工具封装成 stdio MCP server。

只转发、不重写逻辑（AGENTS §2 复用闸）：
  doc_search        -> docs/_工具/doc_search.py          （只读检索）
  doc_read          -> 内置实现，唯一带安全逻辑的一处（docs/ 路径白名单）
  doc_gate          -> 串起 §8.6 顺序（gen_index → policy → anchors → numbers → wf/check → links）
  doc_index_refresh -> docs/_工具/gen_index.py            （可 --ns 增量）
  doc_policy_check  -> docs/_工具/check_doc_policy.py
  doc_anchors_check -> docs/_工具/check_anchors.py
  doc_numbers_check -> docs/_工具/check_numbers.py
  doc_links_check   -> dzrobot_v2/tools/wf/check_links.py

危险脚本一律不注册：cleanup_* / salvage_* / repo_index --write 等 27 个
（salvage_repack.py 能重打包归档 tar，cleanup_restore.py 会回滚）。

协议：MCP stdio（换行分隔的 JSON-RPC 2.0）。无第三方依赖，Python 3.12。

用法：
    PYTHONIOENCODING=utf-8 python docs/_工具/mcp_dzdoc_server.py
注册：见项目插件 bundle 的 cordis.patch.yml（id: dzdoc-mcp，方案 B）。

安全：doc_read 是唯一写额外安全逻辑的地方 —— 解析后必须落在 docs/ 内，
绝对路径 / 家目录 / .. / 以 . 开头的隐藏敏感文件一律拒。
"""
import json
import os
import re
import subprocess
import sys

# ---- 路径真源（按脚本自身位置解析，不依赖 cwd） ----
HERE = os.path.dirname(os.path.abspath(__file__))
DOCS = os.path.dirname(HERE)
ROOT = os.path.dirname(DOCS)
DZROBOT = os.path.join(ROOT, "dzrobot_v2")
TOOLS_DIR = os.path.join(DOCS, "_工具")

SERVER_NAME = "dzdoc"
SERVER_VERSION = "1.0.0"
PROTOCOL_VERSION = "2025-06-18"

# 上限，防止一次调用把模型上下文打爆
MAX_LINES_DEFAULT = 200
MAX_LINES_LIMIT = 2000
OUT_CAP = 20000          # 单个工具输出截断长度
OUT_TAIL = 4000


# --------------------------------------------------------------------------- #
#  工具定义（8 个，全部只读；inputSchema 用 MCP JSON Schema）
# --------------------------------------------------------------------------- #
STR = {"type": "string"}

TOOLS = [
    {
        "name": "doc_search",
        "description": "资料库本地检索：转发 docs/_工具/doc_search.py（中文 2-gram + 英文整词，按相关度排序，返回命中行片段）。",
        "inputSchema": {
            "type": "object",
            "properties": {
                "query": {"type": "string", "description": "查询串，例：激光不亮 / pose_stale / TargetAimer"},
                "limit": {"type": "integer", "description": "返回篇数，默认 5；0 = 全部", "default": 5},
                "in_prefix": {"type": "string", "description": "限定 docs 下子路径（如 20_资料索引库 或 视觉与云台）"},
                "live_only": {"type": "boolean", "description": "排除 history_cache/ 与过程稿，只看现行库", "default": False},
            },
            "required": ["query"],
            "additionalProperties": False,
        },
    },
    {
        "name": "doc_read",
        "description": "只读读取 docs/ 内一篇文档的指定行段。路径白名单：必须解析到 docs/ 内；绝对路径、~/.ssh/id_rsa、../.env 一律拒绝。",
        "inputSchema": {
            "type": "object",
            "properties": {
                "path": {"type": "string", "description": "相对 docs/ 的路径，例：20_资料索引库/车端/01_架构.md（也接受带 docs/ 前缀）"},
                "offset": {"type": "integer", "description": "起始行，1 基，默认 1"},
                "limit": {"type": "integer", "description": "返回行数，默认 200，上限 2000"},
            },
            "required": ["path"],
            "additionalProperties": False,
        },
    },
    {
        "name": "doc_gate",
        "description": "按 AGENTS §8.6 顺序跑完整文档门禁链：gen_index → check_doc_policy → check_anchors → check_numbers → dzrobot_v2 wf/check → check_links。返回 exit code + 每步摘要。",
        "inputSchema": {
            "type": "object",
            "properties": {
                "name": {"type": "string", "description": "可选：卡片/命名空间名，仅作回显，不参与判定"},
                "ns": {"type": "string", "description": "可选：传给 gen_index --ns（增量重扫本 ns，保留外部索引行）"},
                "strict_numbers": {"type": "boolean", "description": "check_numbers 加 --strict（文档侧漂移也判红）", "default": False},
            },
            "additionalProperties": False,
        },
    },
    {
        "name": "doc_index_refresh",
        "description": "刷新目录索引：转发 docs/_工具/gen_index.py。给 ns 则只重扫该 ns（保留外部索引行）。",
        "inputSchema": {
            "type": "object",
            "properties": {
                "ns": {"type": "string", "description": "命名空间，例：doc-1024；留空则全量重生成"},
            },
            "additionalProperties": False,
        },
    },
    {
        "name": "doc_policy_check",
        "description": "文档守卫体检：转发 docs/_工具/check_doc_policy.py。判红 R1 库外新增 / R2 开新库 / R3 未登记 / R4~R9 卡面规范。",
        "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False},
    },
    {
        "name": "doc_anchors_check",
        "description": "锚点守卫：转发 docs/_工具/check_anchors.py。校验正文里的 路径:行号 是否仍指向真代码，只对新增红判失败。",
        "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False},
    },
    {
        "name": "doc_numbers_check",
        "description": "数字速查真源校验：转发 docs/_工具/check_numbers.py。拿 数字速查.yaml 的条目去代码里重算；默认只报不判红。",
        "inputSchema": {
            "type": "object",
            "properties": {"strict": {"type": "boolean", "description": "文档侧漂移也判红", "default": False}},
            "additionalProperties": False,
        },
    },
    {
        "name": "doc_links_check",
        "description": "断链检查：转发 dzrobot_v2/tools/wf/check_links.py。docs/** 与 history/INDEX 的 markdown 相对链接断链 = 红。",
        "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False},
    },
]

TOOL_INDEX = {t["name"]: t for t in TOOLS}


# --------------------------------------------------------------------------- #
#  环境 / 子进程
# --------------------------------------------------------------------------- #
def child_env():
    env = dict(os.environ)
    env["PYTHONIOENCODING"] = "utf-8"      # 本机有过 GBK 塞坏 .gitignore 的事故
    env["PYTHONUTF8"] = "1"
    return env


def run_tool(argv, cwd, timeout=300):
    """转发执行一个既有脚本，返回结构化结果。"""
    try:
        r = subprocess.run(
            argv, cwd=cwd, env=child_env(), capture_output=True,
            text=True, encoding="utf-8", errors="replace", timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        return {"ok": False, "exit_code": 124, "error": "超时（%ds）" % timeout,
                "command": argv, "cwd": cwd}
    except OSError as exc:
        return {"ok": False, "exit_code": 127, "error": "无法启动: %s" % exc,
                "command": argv, "cwd": cwd}
    out = r.stdout or ""
    err = r.stderr or ""
    return {
        "ok": r.returncode == 0,
        "exit_code": r.returncode,
        "command": argv,
        "cwd": cwd,
        "stdout": out[:OUT_CAP],
        "stderr": err[:OUT_CAP],
        "stdout_tail": out[-OUT_TAIL:] if len(out) > OUT_TAIL else None,
        "stdout_head": out[:OUT_TAIL],
        "summary": (out.splitlines()[0] if out.strip() else ""),
    }


# --------------------------------------------------------------------------- #
#  doc_read —— 路径白名单（唯一额外安全逻辑）
# --------------------------------------------------------------------------- #
class PathDenied(Exception):
    pass


def resolve_doc_path(raw):
    """把用户给的 path 解析成 docs/ 内的绝对真实路径；越界一律拒。"""
    if not isinstance(raw, str) or not raw.strip():
        raise PathDenied("path 必须是非空字符串")
    p = raw.strip()

    # 1) 绝对路径 / 盘符 / 家目录：一律拒
    if p.startswith("~"):
        raise PathDenied("拒绝家目录路径（~）：只允许 docs/ 内的相对路径")
    if os.path.isabs(p) or re.match(r"^[A-Za-z]:[\\/]", p):
        raise PathDenied("拒绝绝对路径：只允许 docs/ 内的相对路径")

    # 2) 归一化分隔符，拆开各段
    norm = p.replace("\\", "/")
    parts = [x for x in norm.split("/") if x not in ("", ".")]
    if any(x == ".." for x in parts):
        raise PathDenied("拒绝包含 .. 的路径")
    if not parts:
        raise PathDenied("path 为空或只含分隔符")

    # 3) 隐藏/敏感文件（.env / .gitignore / .ssh 等）一律拒
    if any(x.startswith(".") for x in parts):
        raise PathDenied("拒绝以 . 开头的隐藏/敏感文件")

    # 4) 容忍显式的 docs/ 前缀
    if parts[0].lower() == "docs":
        parts = parts[1:]
    if not parts:
        raise PathDenied("path 为空或只含 docs/")

    # 5) 拼到 docs/ 下，做 realpath（解析符号链接）后强制包含性检查
    target = os.path.join(DOCS, *parts)
    real_docs = os.path.realpath(DOCS)
    real_target = os.path.realpath(target)
    try:
        common = os.path.commonpath([real_docs, real_target])
    except ValueError:
        raise PathDenied("路径跨盘，拒绝")
    if common != real_docs:
        raise PathDenied("路径解析后越出 docs/ 白名单")

    if not os.path.isfile(real_target):
        raise PathDenied("不是普通文件或不存在：%s" % p)
    return real_target


def tool_doc_read(args):
    try:
        path = args.get("path")
        real = resolve_doc_path(path)
    except PathDenied as exc:
        return {"tool": "doc_read", "ok": False, "exit_code": 1, "error": str(exc), "path": path}

    try:
        offset = int(args.get("offset") or 1)
        limit = int(args.get("limit") or MAX_LINES_DEFAULT)
    except (TypeError, ValueError):
        return {"tool": "doc_read", "ok": False, "exit_code": 2, "error": "offset/limit 必须是整数"}
    offset = max(1, offset)
    limit = max(1, min(MAX_LINES_LIMIT, limit))

    try:
        with open(real, "r", encoding="utf-8", errors="replace") as fh:
            lines = fh.read().splitlines()
    except OSError as exc:
        return {"tool": "doc_read", "ok": False, "exit_code": 3, "error": "读取失败: %s" % exc}

    total = len(lines)
    start = offset - 1
    end = min(total, start + limit)
    chunk = lines[start:end]
    truncated = end < total
    rel = os.path.relpath(real, ROOT).replace("\\", "/")
    numbered = ["%6d | %s" % (start + i + 1, ln) for i, ln in enumerate(chunk)]
    head = "# %s（共 %d 行，显示 %d-%d）" % (rel, total, offset, end)
    text = head + "\n" + "\n".join(numbered)
    return {
        "tool": "doc_read", "ok": True, "exit_code": 0,
        "path": rel, "offset": offset, "limit": limit,
        "total_lines": total, "returned": len(chunk), "truncated": truncated,
        "text": text, "lines": chunk,
    }


# --------------------------------------------------------------------------- #
#  转发工具
# --------------------------------------------------------------------------- #
def tool_doc_search(args):
    query = args.get("query")
    if not isinstance(query, str) or not query.strip():
        return {"tool": "doc_search", "ok": False, "exit_code": 2, "error": "query 必填"}
    argv = [sys.executable, os.path.join(TOOLS_DIR, "doc_search.py"), query,
            "-k", str(int(args.get("limit") or 5)), "--json"]
    if args.get("in_prefix"):
        argv += ["--in", str(args["in_prefix"])]
    if args.get("live_only"):
        argv += ["--live-only"]
    r = run_tool(argv, ROOT, timeout=180)
    r["tool"] = "doc_search"
    r["query"] = query
    # 解析 --json 输出，便于模型直接取用
    try:
        payload = json.loads(r.get("stdout") or "")
        if isinstance(payload, dict):
            r["search"] = {
                "total_documents": payload.get("total_documents"),
                "returned": payload.get("returned"),
                "results": payload.get("results", []),
            }
    except (ValueError, TypeError):
        pass
    return r


def tool_doc_index_refresh(args):
    argv = [sys.executable, os.path.join(TOOLS_DIR, "gen_index.py")]
    ns = args.get("ns")
    if isinstance(ns, str) and ns.strip():
        argv += ["--ns", ns.strip()]
    r = run_tool(argv, ROOT, timeout=300)
    r["tool"] = "doc_index_refresh"
    return r


def tool_doc_policy_check(_args):
    r = run_tool([sys.executable, os.path.join(TOOLS_DIR, "check_doc_policy.py")], ROOT, timeout=300)
    r["tool"] = "doc_policy_check"
    return r


def tool_doc_anchors_check(_args):
    r = run_tool([sys.executable, os.path.join(TOOLS_DIR, "check_anchors.py")], ROOT, timeout=600)
    r["tool"] = "doc_anchors_check"
    return r


def tool_doc_numbers_check(args):
    argv = [sys.executable, os.path.join(TOOLS_DIR, "check_numbers.py")]
    if args.get("strict"):
        argv += ["--strict"]
    r = run_tool(argv, ROOT, timeout=600)
    r["tool"] = "doc_numbers_check"
    return r


def tool_doc_links_check(_args):
    r = run_tool([sys.executable, os.path.join(DZROBOT, "tools", "wf", "check_links.py")],
                 DZROBOT, timeout=300)
    r["tool"] = "doc_links_check"
    return r


# §8.6 顺序：gen_index → check_doc_policy → check_anchors → check_numbers → wf/check → check_links
def tool_doc_gate(args):
    ns = args.get("ns")
    strict = bool(args.get("strict_numbers"))
    steps = []
    overall = 0

    def step(label, argv, cwd, timeout=300):
        r = run_tool(argv, cwd, timeout=timeout)
        r["step"] = label
        steps.append(r)
        if not r["ok"]:
            nonlocal overall
            overall = 1

    gen_argv = [sys.executable, os.path.join(TOOLS_DIR, "gen_index.py")]
    if isinstance(ns, str) and ns.strip():
        gen_argv += ["--ns", ns.strip()]
    step("gen_index", gen_argv, ROOT, 300)
    step("check_doc_policy", [sys.executable, os.path.join(TOOLS_DIR, "check_doc_policy.py")], ROOT, 300)
    step("check_anchors", [sys.executable, os.path.join(TOOLS_DIR, "check_anchors.py")], ROOT, 600)
    num_argv = [sys.executable, os.path.join(TOOLS_DIR, "check_numbers.py")]
    if strict:
        num_argv += ["--strict"]
    step("check_numbers", num_argv, ROOT, 600)
    step("wf_check", [sys.executable, os.path.join(DZROBOT, "tools", "wf", "check.py")],
         ROOT, 600)
    step("check_links", [sys.executable, os.path.join(DZROBOT, "tools", "wf", "check_links.py")],
         DZROBOT, 300)

    per_step = [{"step": s.get("step"), "exit_code": s.get("exit_code"), "ok": s.get("ok"),
                 "summary": s.get("summary", "")} for s in steps]
    return {
        "tool": "doc_gate",
        "ok": overall == 0,
        "exit_code": overall,
        "name": args.get("name"),
        "summary": "；".join("%s=%s" % (s["step"], "OK" if s["ok"] else "RED") for s in per_step),
        "steps": per_step,
        "detail": steps,
    }


HANDLERS = {
    "doc_search": tool_doc_search,
    "doc_read": tool_doc_read,
    "doc_gate": tool_doc_gate,
    "doc_index_refresh": tool_doc_index_refresh,
    "doc_policy_check": tool_doc_policy_check,
    "doc_anchors_check": tool_doc_anchors_check,
    "doc_numbers_check": tool_doc_numbers_check,
    "doc_links_check": tool_doc_links_check,
}


# --------------------------------------------------------------------------- #
#  MCP stdio 循环（换行分隔 JSON-RPC 2.0）
# --------------------------------------------------------------------------- #
def send(obj):
    sys.stdout.write(json.dumps(obj, ensure_ascii=False) + "\n")
    sys.stdout.flush()


def result_envelope(payload, is_error=False):
    return {
        "content": [{"type": "text", "text": json.dumps(payload, ensure_ascii=False)}],
        "structuredContent": payload,
        "isError": bool(is_error),
    }


def handle(msg):
    method = msg.get("method")
    mid = msg.get("id")
    params = msg.get("params") or {}

    if method == "initialize":
        return {"result": {
            "protocolVersion": params.get("protocolVersion") or PROTOCOL_VERSION,
            "capabilities": {"tools": {"listChanged": False}},
            "serverInfo": {"name": SERVER_NAME, "version": SERVER_VERSION},
        }}
    if method == "ping":
        return {"result": {}}
    if method == "tools/list":
        return {"result": {"tools": TOOLS}}
    if method == "tools/call":
        name = params.get("name")
        args = params.get("arguments") or {}
        if name not in HANDLERS:
            return {"result": result_envelope(
                {"ok": False, "exit_code": -32601,
                 "error": "未知工具 %r；只暴露 %d 个只读工具" % (name, len(TOOLS))},
                is_error=True)}
        try:
            payload = HANDLERS[name](args if isinstance(args, dict) else {})
        except Exception as exc:  # noqa: BLE001 —— 任何异常都转成工具错误，不让 server 崩
            payload = {"ok": False, "exit_code": -32603,
                       "error": "%s: %s" % (type(exc).__name__, exc), "tool": name}
        return {"result": result_envelope(payload, is_error=not payload.get("ok"))}
    if method in ("notifications/initialized", "notifications/cancelled"):
        return None
    return None


def main():
    # 控制台可能是 GBK（Windows）；显式 UTF-8，避免中文炸/乱码
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except Exception:  # noqa: BLE001
            pass

    for raw in sys.stdin:
        line = raw.strip()
        if not line:
            continue
        try:
            msg = json.loads(line)
        except ValueError:
            send({"jsonrpc": "2.0", "id": None,
                  "error": {"code": -32700, "message": "JSON 解析失败"}})
            continue
        if not isinstance(msg, dict) or "method" not in msg:
            continue
        try:
            reply = handle(msg)
        except Exception as exc:  # noqa: BLE001
            reply = {"jsonrpc": "2.0", "id": msg.get("id"),
                     "error": {"code": -32603, "message": "%s: %s" % (type(exc).__name__, exc)}}
        if reply is None:
            continue
        out = dict(reply)
        out.setdefault("jsonrpc", "2.0")
        if "id" not in out:
            out["id"] = msg.get("id")
        send(out)


if __name__ == "__main__":
    main()
