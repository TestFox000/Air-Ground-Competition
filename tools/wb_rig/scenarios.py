# -*- coding: utf-8 -*-
"""具名场景 → 夹具路径（卡 `WB-2-RIG-20261031`）。

★ 真源是 `src/dzbrain/tests/fixtures/route_sources.json`（WB-1 的产物，
  schema `dzrobot-route-source/1`）：名字 → **本目录内既有快照**的相对路径。

三条负约束（每条都对应一次真实事故，违反即返工）：

1. **取不到名字 → 显式报错**。绝不回落 `config/map.yaml`：
   `config/map.yaml` 已随 D-062 删除，而"悄悄换一张图"比报错危险得多 ——
   现场核查会得出"我以为跑了第七图，其实跑的是另一张图"的假结论（D-054 裁决 4）。
2. **不复制快照内容**。本模块只做名字 → 路径的映射，快照本体留在 `fixtures/`；
   复制一份到 `artifacts/` 就等于把"逐字快照"变成"又一个会漂的副本"（D-062）。
3. **越界路径拒绝**。`sources` 里的值是**相对 fixtures 目录**的路径；解析后必须仍在
   fixtures 目录内 —— 防 `../` 越界把任意仓库文件当夹具加载。

⚠️ 与 `sixth` / `carloop` 这类"有图无路线"的场景：`route` 为 `null` 是**合法状态**，
  不是缺失字段。调用方拿到 `None` 必须自己决定（规划器跑布局 / 只做门禁检查），
  本模块不替它猜一条路线出来。
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass

#: 夹具根目录（相对本包：tools/wb_rig/ → repo/src/dzbrain/tests/fixtures）
_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
FIXTURES_DIR = os.path.join(_REPO, "src", "dzbrain", "tests", "fixtures")
SOURCES_JSON = os.path.join(FIXTURES_DIR, "route_sources.json")

#: 本模块认的 schema 号（WB-1 产物）。别的号一律拒绝 —— 宁可红，不做兼容猜测。
SOURCES_SCHEMA = "dzrobot-route-source/1"


class ScenarioError(ValueError):
    """场景解析失败。**必须显式抛**，调用方不许吞掉换成默认图。"""


@dataclass(frozen=True)
class Scenario:
    """一个具名场景的全部输入路径（已解析成绝对路径）。"""

    name: str
    map_path: str
    route_path: str | None
    origin: str
    ledger: str

    @property
    def has_route(self) -> bool:
        return self.route_path is not None

    def describe(self) -> str:
        return (f"场景[{self.name}] 地图 {os.path.basename(self.map_path)} / "
                f"路线 {os.path.basename(self.route_path) if self.route_path else '（无，由布局规划）'}"
                f" —— {self.origin}")


def _load_sources(path: str = SOURCES_JSON) -> dict:
    """读并校验 `route_sources.json`。坏文件/坏 schema 一律抛 `ScenarioError`。"""
    if not os.path.isfile(path):
        raise ScenarioError(
            f"夹具来源登记表不存在: {path}\n"
            f"  ⇒ 本台架**不回落**默认地图。先跑 WB-1 产出 route_sources.json。")
    try:
        with open(path, encoding="utf-8") as fh:
            doc = json.load(fh)
    except (OSError, ValueError) as exc:
        raise ScenarioError(f"夹具来源登记表读不动: {path}（{type(exc).__name__}: {exc}）") from exc
    schema = str(doc.get("schema") or "")
    if schema != SOURCES_SCHEMA:
        raise ScenarioError(
            f"夹具来源登记表 schema 不符：期望 {SOURCES_SCHEMA!r}，实得 {schema!r}（{path}）")
    sources = doc.get("sources")
    if not isinstance(sources, dict) or not sources:
        raise ScenarioError(f"夹具来源登记表没有 sources 段: {path}")
    return sources


def _resolve_inside(rel: str, *, base: str = FIXTURES_DIR) -> str:
    """把 sources 里的相对值解析成绝对路径，并**保证仍在 base 目录内**。"""
    cand = os.path.abspath(os.path.join(base, str(rel)))
    base_abs = os.path.abspath(base)
    if os.path.commonpath([cand, base_abs]) != base_abs:
        raise ScenarioError(f"夹具路径越界（不在 {base_abs} 内）: {rel!r} → {cand}")
    if not os.path.isfile(cand):
        raise ScenarioError(f"夹具文件不存在: {cand}（sources 里登记的是 {rel!r}）")
    return cand


def available() -> list[str]:
    """登记表里全部场景名（排序后固定，便于 `--list` 与测试断言）。"""
    return sorted(_load_sources())


def resolve(name: str) -> Scenario:
    """名字 → `Scenario`。未知名字抛 `ScenarioError`（**不**回落到任何默认图）。"""
    sources = _load_sources()
    key = str(name or "").strip()
    if key not in sources:
        raise ScenarioError(
            f"未知场景 {name!r}；已登记的有 {sorted(sources)}。"
            f"（禁止回落到 config/map.yaml —— D-054 裁决 4 / D-062）")
    entry = sources[key] or {}
    map_rel = entry.get("map")
    if not map_rel:
        raise ScenarioError(f"场景 {key!r} 没登记地图（sources[{key!r}].map 为空）")
    map_path = _resolve_inside(map_rel)
    route_rel = entry.get("route")
    route_path = _resolve_inside(route_rel) if route_rel else None
    return Scenario(name=key, map_path=map_path, route_path=route_path,
                    origin=str(entry.get("origin") or ""),
                    ledger=str(entry.get("ledger") or ""))
