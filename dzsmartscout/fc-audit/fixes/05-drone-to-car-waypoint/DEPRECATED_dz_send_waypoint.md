# ⛔ 本目录的 `dz_send_waypoint.py` 已废弃（2026-10-12，卡 t5 收口）

## 一句话

**发送器只有一份真源**：[`dzsmartscout/tools/link/dz_send_waypoint.py`](../../../../tools/link/dz_send_waypoint.py)。
本目录 `files/` 下那份是 2026-10-07 的旧副本，**已分叉**，任何人不要再改它、也不要再往岛上拷它。

## 为什么收口（实测差异，不是"看起来旧"）

| 项 | 本目录旧副本（已归档） | 现役 `tools/link/` 真源 |
|---|---|---|
| 文件 mtime | 2026-10-07 20:28 | 2026-10-12 20:08（t18/F2 文案修正后） |
| 大小 / 行数 | 15 657 B / 345 行 | 21 637 B / 417 行 |
| md5 | `B3AA2ED999CD…` | **`EB41DBF4B944…`**（t18 起现役 hub 实值；t12–t18 之间为 `893E7B9F2375…`） |
| 子命令 | `route / bridge / heartbeat / target / field / listen / contract` | 同上 **+ t12 修复**（`listen` 不再崩、`run` 参数不再被静默忽略、用法陷阱提示）**+ t18/F2**（报错提示改为本发送器可执行写法，并注明 `--bridge` 属 supervisor） |
| 默认"等 DDS 匹配" | 有 `wait_match`，冷启动仍易丢首包 | 保留并可 `--wait-match` 调大；**匹配数 0 时不再"假成功"** |
| 读环境（域号） | ❌ 不读 `dzlink.env`（`grep dzlink.env` = 0 命中） | 域号仍由调用方内联（见协议手册 §6.3 的坑） |

> 旧副本 md5 `B3AA2ED999CD…` 与现役 **`EB41DBF4B944…`**（2026-10-12 20:08 起；此前是 `893E7B9F2375…`）
> **不同** ⇒ 这就是"两处同名异体"的实证；
> 2026-10-12 之前凡是照本目录 `APPLY.md` 往机器上拷发送器的人，装的是**没有 t12 修复**的那一份。

## 本次处置（t5，**字节级保真，不改内容**）

```
fc-audit/fixes/05-drone-to-car-waypoint/files/dz_send_waypoint.py
  → fc-audit/fixes/05-drone-to-car-waypoint/files/_deprecated_20261012/dz_send_waypoint.py
```

- 归档是 `Copy-Item` **原字节**搬移（md5 仍为 `B3AA2ED999CD…`，大小 15 657 B **不变**）。
- 为什么是"搬移 + 留指针"而不是删除：`APPLY.md`（26 287 B）是本修复包的**历史证据**，
  按只增不改的留档惯例保留；把可执行副本挪进 `_deprecated_*` 子目录，
  既让 PATH/脚本扫不到它，又留下"当时装的到底是哪一版"的证据。
- `APPLY.md` 正文**未改**（历史文档不静默改写）；本文件是它上方的新增指针。

## 现在该怎么做

1. **要发路线/情报** → 只用 `tools/link/dz_send_waypoint.py`（或常驻节点 `tools/link/dzlink_supervisor.py`）。
2. **要复现 2026-10-07 的现场状态** → 用本目录 `_deprecated_20261012/` 里那份（md5 对齐后再拷）。
3. **要改发送器** → 改 `tools/link/` 那一份，并在 `tools/link/README.md` 里记变更；
   **禁止**再在本目录 `files/` 下放第二个可执行副本（这正是本次分叉的成因）。

## 相关（都在 t5 交付里）

- 协议与运行手册：`dzrobot_v2/docs/空地指挥链路_协议与运行手册.md`（§6.3 域号/匹配数两个坑）
- 端到端 bench 证据：`dzrobot_v2/src/dzbrain/tests/test_link_e2e.py`
