"""params.yaml 的极简解析器（够用即可，避免为单测引入 PyYAML 依赖）。

只支持本项目配置文件的结构：
    顶层:
      ros__parameters:
        key: value        # 行尾注释
        key2: [1.0, 2.0]
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List


def _strip_comment(line: str) -> str:
    out = []
    in_str = False
    quote = ""
    for ch in line:
        if in_str:
            out.append(ch)
            if ch == quote:
                in_str = False
        elif ch in ("'", '"'):
            in_str = True
            quote = ch
            out.append(ch)
        elif ch == "#":
            break
        else:
            out.append(ch)
    return "".join(out).rstrip()


def _parse_value(raw: str):
    raw = raw.strip()
    if not raw:
        return None
    if raw.startswith("[") and raw.endswith("]"):
        body = raw[1:-1].strip()
        if not body:
            return []
        return [_parse_value(x) for x in body.split(",")]
    if raw.startswith(('"', "'")) and raw.endswith(('"', "'")) and len(raw) >= 2:
        return raw[1:-1]
    low = raw.lower()
    if low in ("true", "false"):
        return low == "true"
    if low in ("null", "~"):
        return None
    try:
        return int(raw)
    except ValueError:
        pass
    try:
        return float(raw)
    except ValueError:
        pass
    return raw


def load_params(path) -> Dict[str, Dict[str, object]]:
    """返回 {节点名: {参数名: 值}}，只收 ros__parameters 段。"""
    text = Path(path).read_text(encoding="utf-8")
    result: Dict[str, Dict[str, object]] = {}
    section: str = ""
    in_params = False
    for raw_line in text.splitlines():
        line = _strip_comment(raw_line)
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip(" "))
        body = line.strip()
        if indent == 0:
            if body.endswith(":") and not body.startswith("#"):
                section = body[:-1].strip()
                in_params = False
                result.setdefault(section, {})
            continue
        if body == "ros__parameters:":
            in_params = True
            continue
        if in_params and ":" in body and section:
            k, _, v = body.partition(":")
            result[section][k.strip()] = _parse_value(v)
    return result


def section_names(path) -> List[str]:
    return list(load_params(path).keys())
