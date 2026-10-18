# -*- coding: utf-8 -*-
"""tests/yaml_lite：match_params.yaml 极简读取（测试专用）。

契约：load_params(path)["integrate_node"] 直接返回参数字典
（自动解包 ros__parameters 层）。优先用 PyYAML（本机可用），
缺失时退化为缩进解析的最小实现。
"""
from __future__ import annotations

import json
import re
from pathlib import Path


def _with_pyyaml(path: Path) -> dict | None:
    try:
        import yaml  # noqa: PLC0415
    except ImportError:
        return None
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    return {top: (body.get("ros__parameters", body) if isinstance(body, dict) else body)
            for top, body in raw.items()}


def _fallback(path: Path) -> dict:
    """缩进解析：顶层键 -> 二级 ros__parameters 块内的标量/单行列表。"""
    out: dict = {}
    top = None
    in_params = False
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        indent = len(line) - len(line.lstrip())
        s = line.strip()
        if indent == 0 and s.endswith(":"):
            top, in_params = s[:-1], False
            out[top] = {}
            continue
        if top is None:
            continue
        if indent == 2 and s == "ros__parameters:":
            in_params = True
            continue
        if in_params and indent >= 4 and ":" in s:
            k, v = s.split(":", 1)
            v = v.split("#")[0].strip()
            if v.startswith("[") and v.endswith("]"):
                items = [x.strip().strip('"\'') for x in v[1:-1].split(",") if x.strip()]
                out[top][k.strip()] = items
            elif v.startswith('"') and v.endswith('"'):
                out[top][k.strip()] = v[1:-1]
            elif v in ("true", "false"):
                out[top][k.strip()] = v == "true"
            else:
                try:
                    out[top][k.strip()] = json.loads(v)
                except (ValueError, json.JSONDecodeError):
                    out[top][k.strip()] = v
    return out


def load_params(path) -> dict:
    p = Path(path)
    return _with_pyyaml(p) or _fallback(p)
