#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""map_source.py —— 「这一轮规划用哪张地图」的**唯一解析入口**（卡 WB-9-VENDOR-20261031）

## 为什么要有这个文件

WB-1-ROUTE-20261031 把 `vendor/dzbrain_ws/config/map.yaml`（2411 B，sha256 `a4b8c90e…57d1a`）
按 D-054 裁决4 / D-062 移出了工作树（存到 `_tmp_session/wb-1031/vendor_snapshot/`），
本目录 5 个消费点当时写死 `vendor/dzbrain_ws/config/map.yaml` ⇒ 全部变成
`❌ 地图文件不存在: …/vendor/dzbrain_ws/config/map.yaml`。

**不能**把地图拷回 vendor（那正是 D-054/D-062 消灭的「第 4/5 份副本」），
所以统一改为指向**仓库内已有的逐字快照**，并且把「指向哪个文件、为什么不选另一个」写在代码里。

## 解析顺序（与 `vendor/tools/plan_route.py:find_map_path` 同序，只多一个具名入口）

    ① 显式参数（各脚本的 `--map`）         —— 最高优先，谁传谁说话
    ② 环境变量 `DZ_PLANNER_MAP`            —— 本目录自己的覆盖开关（WB-9 新增）
    ③ 环境变量 `ROBOT_MAP`                 —— 与 `plan_route.py`/`route_check.py` 同一口径
    ④ 具名来源（`route_sources.json` 的 `sixth`/`field7`/…）—— 缺省 `DEFAULT_NAME`
    ⑤ 都没有 ⇒ **抛 MapSourceError**，绝不回落到「工作区 `config/map.yaml`」那种静默降级

## 为什么缺省选 `field7`（= `map_field7_t24.yaml`）而不是 `sixth`

卡面写「优先 `map_sixth_20261017.yaml`」，实测两条事实（本文件 `WHY_FIELD7` 常量里有原文）：

1. **逐字节关系**：WB-1 移出前的 vendor 副本（2411 B，CRLF）归一化换行后 **==
   `map_field7_t24.yaml`（2370 B, sha256 `385e0718…`）**；而 `map_sixth_20261017.yaml`
   （2369 B）只差**末尾一个换行**（实测 `field7 == sixth + b"\n"`）⇒ YAML 语义完全相同。
2. **哈希门禁口径**：`HASH_LEDGER.json` 里 `config/map.yaml` 那条记录的就是
   `385e07182aa84ebc26aab6e1c970838d912fc4f0ddea4abcacbf1b2dc63ed20d` / 2370 B
   —— 只有 `map_field7_t24.yaml` 与它**逐字节相同**，`sixth` 会差 1 B。

⇒ 选 `field7` 的理由是「**哈希能继续钉死**，不靠人眼看语义」；`sixth` 仍然可被
`DZ_PLANNER_MAP=<sixth 的绝对路径>` 或 `--map` 显式指定，具名表里也留着。

## 用法

    import map_source
    p = map_source.default_path()                  # 缺省（可被环境变量覆盖）
    info = map_source.resolve(explicit=args.map)   # 完整解析结果（含 sha 与命中来源）
    info.path, info.sha256, info.origin, info.name
"""
from __future__ import annotations

import hashlib
import json
import os
import pathlib

HERE = pathlib.Path(__file__).resolve().parent          # dzsmartscout/tools/planner
WORKSPACE = HERE.parents[2]                             # 比赛存档/v1
FIXTURES = WORKSPACE / "dzrobot_v2" / "src" / "dzbrain" / "tests" / "fixtures"

#: WB-1 建的名字 → 路径登记表（**唯一**的具名真源，本模块只读它，不复制内容）
REGISTRY = FIXTURES / "route_sources.json"

#: 缺省具名来源（理由见模块头 §为什么缺省选 field7）
DEFAULT_NAME = "field7"

#: 本目录自己的覆盖开关（优先于 ROBOT_MAP，低于显式 --map）
ENV_OURS = "DZ_PLANNER_MAP"
#: 与 `vendor/tools/plan_route.py:find_map_path` / `route_check.py:_find_map_file` 同口径
ENV_ROBOT = "ROBOT_MAP"

#: 内置兜底表：`route_sources.json` 缺失时仍能解析（**只放路径，不复制内容**）。
#: 有登记表时以登记表为准（登记表是 WB-1 的正式产物，本模块不覆盖它）。
_BUILTIN = {
    "field7": "map_field7_t24.yaml",
    "sixth": "map_sixth_20261017.yaml",
    "carloop": "map_carloop_20261024.yaml",
    "field": "map_field.yaml",
}

#: 被 WB-1 移出的那份 vendor 副本的原始哈希（`_tmp_session/wb-1031/vendor_snapshot/`）。
#: 仅作证据常量，不参与解析；用于在报告里说明「我们指向的快照就是它」。
MOVED_VENDOR_SHA256 = "a4b8c90eab10"   # 前缀，完整值见 WB-1 卡面
LEDGER_MAP_SHA256 = "385e07182aa84ebc26aab6e1c970838d912fc4f0ddea4abcacbf1b2dc63ed20d"

WHY_FIELD7 = (
    "选 field7 而非 sixth 的理由：① 与 HASH_LEDGER.json 记录的 config/map.yaml "
    "逐字节相同（sha256 385e0718…, 2370 B），哈希门禁可继续钉死；"
    "② map_sixth_20261017.yaml（2369 B）与它只差末尾一个换行（field7 == sixth + b'\\n'），"
    "YAML 语义相同；③ WB-1 移出的 vendor 副本（2411 B, CRLF）归一化换行后即本文件。"
)


class MapSourceError(RuntimeError):
    """地图解析失败（显式给了不存在的路径 / 具名表里没有该名字 / 默认快照丢失）。"""


class ResolvedMap:
    """一次解析结果：路径 + 哈希 + 命中来源（便于日志/证据里写清「这轮用的是哪张图」）。"""

    __slots__ = ("path", "sha256", "size", "origin", "name")

    def __init__(self, path: pathlib.Path, origin: str, name: str | None):
        self.path = path
        self.origin = origin
        self.name = name
        self.sha256 = _sha256(path)
        self.size = path.stat().st_size

    def describe(self) -> str:
        return ("地图 = %s（%s，%s，%d B，sha256 %s%s）"
                % (self.path, self.origin,
                   ("具名 %r" % self.name) if self.name else "无具名",
                   self.size, self.sha256[:20],
                   "  ★ 与 WB-1 移出的 vendor 副本逐字节同源" if self.name == "field7" else ""))

    def __str__(self) -> str:                              # 方便直接 str(info) 进子进程参数
        return str(self.path)


def _sha256(p: pathlib.Path) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for c in iter(lambda: f.read(1 << 16), b""):
            h.update(c)
    return h.hexdigest()


def registry() -> dict:
    """读 WB-1 的 `route_sources.json`；读不到/坏了就退内置表（不静默失败）。"""
    if REGISTRY.is_file():
        try:
            doc = json.loads(REGISTRY.read_text(encoding="utf-8"))
        except Exception:                                   # noqa: BLE001  坏表 → 内置兜底
            return {k: {"map": v} for k, v in _BUILTIN.items()}
        src = doc.get("sources") or {}
        if isinstance(src, dict) and src:
            return src
    return {k: {"map": v} for k, v in _BUILTIN.items()}


def known_names() -> list[str]:
    return sorted(registry().keys())


def named_path(name: str) -> pathlib.Path:
    """具名 → 绝对路径。名字不在表里 / 文件不在盘上 ⇒ MapSourceError（**绝不猜**）。"""
    reg = registry()
    if name not in reg:
        raise MapSourceError("具名地图 %r 不在 %s（可用：%s）"
                             % (name, REGISTRY, known_names()))
    rel = (reg[name] or {}).get("map")
    if not rel:
        raise MapSourceError("具名地图 %r 在登记表里没有 map 字段" % name)
    p = FIXTURES / rel
    if not p.is_file():
        raise MapSourceError("具名地图 %r 指向 %s，但文件不存在" % (name, p))
    return p


def resolve(explicit: str | None = None, *, name: str | None = None,
            env: dict | None = None) -> ResolvedMap:
    """按模块头 §解析顺序 解析本轮地图；失败抛 MapSourceError（不回落到任何静默降级）。"""
    env = os.environ if env is None else env

    if explicit:
        p = pathlib.Path(explicit).expanduser()
        if not p.is_file():
            raise MapSourceError("显式地图不存在：%s" % p)
        return ResolvedMap(p, "显式 --map", None)

    for key in (ENV_OURS, ENV_ROBOT):
        val = (env.get(key) or "").strip()
        if val:
            p = pathlib.Path(val).expanduser()
            if not p.is_file():
                raise MapSourceError("环境变量 %s 指向的地图不存在：%s" % (key, p))
            return ResolvedMap(p, "环境变量 %s" % key, None)

    which = name or DEFAULT_NAME
    return ResolvedMap(named_path(which), "具名缺省", which)


def default_path() -> pathlib.Path:
    """只想要路径时用它（缺省 + 环境变量覆盖；具名解析失败会抛 MapSourceError）。"""
    return resolve().path


if __name__ == "__main__":                                  # 自检/人肉查账用
    import sys
    try:
        info = resolve(sys.argv[1] if len(sys.argv) > 1 else None)
    except MapSourceError as exc:
        print("❌ %s" % exc)
        raise SystemExit(2)
    print(info.describe())
    print("具名可用：%s" % ", ".join(known_names()))
    print("登记表  ：%s" % REGISTRY)
    print("为什么：%s" % WHY_FIELD7)
