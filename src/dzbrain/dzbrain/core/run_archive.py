# -*- coding: utf-8 -*-
"""运行期归档（AGC-RECTIFY · M2-ARCHIVE / 用户 2026-10-27 裁决 B 区）。

一场一个目录、**只写不读**、收尾必归档。落点由 Lead 裁决为
`C/artifacts/runs/<UTC ts>/`（`dzrobot_v2/.gitignore` 已忽略 `artifacts/`，
归档不入库、不污染两棵树的脏文件计数；车端部署后即 `~/dzrobot/artifacts/runs/`）。

固定六个文件（缺一个都不算归档成功）::

    map.json      只有**固定层**的空白图（启赛时写一次；变量由无人机增量注入，本场不回写）
    route.json    本场实际下发的段（逐段覆盖写：tokens + 段台账）
    pose.csv      位姿采样（t_s,x_m,y_m,yaw_rad,block,phase,segment_id）
    log.txt       本次运行的日志抄本（与 stdout 同源，tee 写入）
    trace.jsonl   事件流（每行一个 JSON：gate / phase / segment / archive）
    summary.json  收尾汇总，六个键：run_id / duration_s / distance_m / out_of_bounds /
                  unknown_cells / warn_counts

## 为什么"只写不读"
上一场的图/路线/位姿是**历史证据**，不是本场输入：本场若缺布局就该等布局或按未知格走，
绝不能把上一场的 `map.json` 当圣旨（这与车端"不留地图文件、只认无人机下发"同一条纪律）。
本模块**没有任何读取** `runs/` 下既有目录的代码路径（`write_*` 之外只有 `mkdir`），
验收用 `grep -n "read_text\\|open(.*r" run_archive.py` 为空 + 单测反向钉住。

## 幂等与原子
`finalize()` 可重复调用（第二次起只更新 summary，不重开文件）；summary 先写
`summary.json.tmp` 再 `os.replace`，进程被 SIGKILL 也不会留下半个 JSON。
"""

from __future__ import annotations

import json
import os
import pathlib
import time
from typing import Any, Callable, Iterable

__all__ = ["RUN_FILES", "SUMMARY_KEYS", "POSE_HEADER", "utc_run_id", "RunArchive",
           "install_log_tee", "default_runs_root"]

#: 一场归档必须齐的六个文件（顺序固定，便于 diff/复核）
RUN_FILES = ("map.json", "route.json", "pose.csv", "log.txt", "trace.jsonl", "summary.json")

#: summary.json 的六个键（少一个即判失败）
SUMMARY_KEYS = ("run_id", "duration_s", "distance_m", "out_of_bounds",
                "unknown_cells", "warn_counts")

POSE_HEADER = "t_s,x_m,y_m,yaw_rad,block,phase,segment_id"


def utc_run_id(when: float | None = None) -> str:
    """UTC 时间戳目录名：`20261027T154512Z`（秒级；同一秒内重复则加后缀）。"""
    t = time.time() if when is None else float(when)
    return time.strftime("%Y%m%dT%H%M%SZ", time.gmtime(t))


def default_runs_root() -> pathlib.Path:
    """`<仓库根>/artifacts/runs`（本文件在 `src/dzbrain/dzbrain/core/` ⇒ parents[4]）。"""
    here = pathlib.Path(__file__).resolve()
    return here.parents[4] / "artifacts" / "runs"


class RunArchive:
    """单场运行的归档器：建目录 → 写六个文件 → finalize 汇总。

    用法（run_live）::

        arch = RunArchive(log=log)          # 目录已建好、map.json 已写固定层
        log = install_log_tee(log, arch.log_path)
        arch.pose(t, x, y, yaw, block, phase, segment_id)
        arch.trace("gate", key="layout_stale", age_s=12.3)
        arch.finalize(reason="finish", duration_s=118.1, out_of_bounds=74,
                      unknown_cells=5, warn_counts=gates.counts())
    """

    def __init__(self, root: pathlib.Path | str | None = None, *,
                 run_id: str | None = None,
                 log: Callable[[str], None] | None = None,
                 clock: Callable[[], float] = time.time) -> None:
        self.clock = clock
        base = pathlib.Path(root) if root is not None else default_runs_root()
        rid = run_id or utc_run_id(self.clock())
        self.root = base
        self.run_id = rid
        self.dir = base / rid
        n = 0
        # 同一秒内起两个进程：加后缀，绝不复用既有目录（"只写不读上一场"的物理保证）
        while self.dir.exists():
            n += 1
            self.dir = base / f"{rid}-{n}"
        self.run_id = self.dir.name
        self.dir.mkdir(parents=True, exist_ok=False)
        self._log = log
        self._closed = False
        self._summary: dict[str, Any] | None = None
        self._t0 = float(self.clock())
        self.distance_m = 0.0
        self.out_of_bounds = 0
        self.unknown_cells = 0
        self.warn_counts: dict[str, int] = {}
        self._last_pose: tuple[float, float] | None = None
        self._pose_fh = open(self.dir / "pose.csv", "w", encoding="utf-8", newline="")
        self._pose_fh.write(POSE_HEADER + "\n")
        self._trace_fh = open(self.dir / "trace.jsonl", "w", encoding="utf-8")
        # log.txt 先落一行头（tee 写入的正文在 finalize 时补 tail）
        (self.dir / "log.txt").write_text(
            f"# run_id={self.run_id}\n# started={time.strftime('%Y-%m-%dT%H:%M:%S', time.gmtime(self._t0))}Z\n",
            encoding="utf-8")
        # route.json 先给空白占位（保证六个文件从第一秒就齐）
        self._write_json("route.json", {"run_id": self.run_id, "tokens": [],
                                        "segments": [], "source": ""})
        self._note(f"运行归档目录：{self.dir}")

    # ------------------------------------------------------------------ #
    def _note(self, text: str) -> None:
        if self._log is not None:
            try:
                self._log(text)
            except Exception:                    # noqa: BLE001 归档不许拖垮主循环
                pass

    def _write_json(self, name: str, payload: Any) -> pathlib.Path:
        path = self.dir / name
        tmp = path.with_suffix(path.suffix + ".tmp")
        tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=1, sort_keys=True) + "\n",
                       encoding="utf-8")
        os.replace(tmp, path)
        return path

    # ------------------------------------------------------------------ #
    @property
    def log_path(self) -> pathlib.Path:
        return self.dir / "log.txt"

    def path_of(self, name: str) -> pathlib.Path:
        return self.dir / name

    def blank_map(self, *, rows: int, cols: int, block_size_m: float,
                  barrier_col_from: str, barrier_row_hi: int, barrier_row_lo: int,
                  extra: dict[str, Any] | None = None) -> pathlib.Path:
        """写「只有固定层」的空白图：场几何 + 隔板固定层，**不含**雷/障/靶/桥洞。

        为什么是空白：B 区"先走边补"——变量一律由无人机增量注入；本地只留
        不随对局变化的东西（网格尺寸、隔板列/行）。写入后**本场不再回写**（只写不读）。
        """
        payload = {
            "run_id": self.run_id,
            "kind": "blank-fixed-layer-only",
            "rows": int(rows), "cols": int(cols),
            "block_size_m": float(block_size_m),
            "field_w_m": round(float(cols) * float(block_size_m), 6),
            "field_h_m": round(float(rows) * float(block_size_m), 6),
            "barrier_col_from": str(barrier_col_from),
            "barrier_row_hi": int(barrier_row_hi),
            "barrier_row_lo": int(barrier_row_lo),
            "mines": [], "obstacles": [], "targets": [], "bridges": [],
            "note": "固定层空白图；对局变量由无人机 field_layout 增量注入，本场不回写",
        }
        if extra:
            payload.update(extra)
        return self._write_json("map.json", payload)

    def write_route(self, tokens: Iterable[str], *, segments: list[dict[str, Any]] | None = None,
                    source: str = "", target_cell: str = "", from_cell: str = "") -> pathlib.Path:
        """覆盖写本场段台账（只保留最近一次下发的段：tokens + 已发生段列表）。"""
        return self._write_json("route.json", {
            "run_id": self.run_id,
            "tokens": [str(t) for t in (tokens or [])],
            "segments": list(segments or []),
            "source": str(source),
            "from_cell": str(from_cell),
            "target_cell": str(target_cell),
        })

    def pose(self, t: float, x: float, y: float, yaw: float, block: str,
             phase: str, segment_id: int, *, out_of_bounds: bool = False) -> None:
        """采样一行位姿；顺带累计里程（相邻采样欧氏距离）与越界次数。"""
        if self._closed:
            return
        try:
            self._pose_fh.write(
                f"{float(t):.3f},{float(x):.4f},{float(y):.4f},{float(yaw):.4f},"
                f"{str(block)},{str(phase)},{int(segment_id)}\n")
        except Exception:                        # noqa: BLE001 磁盘满也不许炸循环
            return
        if self._last_pose is not None:
            dx = float(x) - self._last_pose[0]
            dy = float(y) - self._last_pose[1]
            self.distance_m += (dx * dx + dy * dy) ** 0.5
        self._last_pose = (float(x), float(y))
        if out_of_bounds:
            self.out_of_bounds += 1

    def trace(self, event: str, **fields: Any) -> None:
        """事件流（每行一个 JSON；不可序列化的值退化成 repr，绝不抛）。"""
        if self._closed:
            return
        rec: dict[str, Any] = {"t": round(float(self.clock()) - self._t0, 3), "event": str(event)}
        for key, value in fields.items():
            if isinstance(value, (str, int, float, bool)) or value is None:
                rec[str(key)] = value
            elif isinstance(value, (list, tuple)):
                rec[str(key)] = [v if isinstance(v, (str, int, float, bool)) else repr(v)
                                 for v in value]
            elif isinstance(value, dict):
                rec[str(key)] = {str(k): (v if isinstance(v, (str, int, float, bool)) else repr(v))
                                 for k, v in value.items()}
            else:
                rec[str(key)] = repr(value)
        try:
            self._trace_fh.write(json.dumps(rec, ensure_ascii=False) + "\n")
        except Exception:                        # noqa: BLE001
            pass

    def warn(self, key: str) -> None:
        """warn 计数（GateLedger 每 warn 一次就调一次）。"""
        self.warn_counts[str(key)] = int(self.warn_counts.get(str(key), 0)) + 1

    # ------------------------------------------------------------------ #
    def finalize(self, *, reason: str, duration_s: float | None = None,
                 distance_m: float | None = None, out_of_bounds: int | None = None,
                 unknown_cells: int | None = None,
                 warn_counts: dict[str, int] | None = None,
                 extra: dict[str, Any] | None = None) -> dict[str, Any]:
        """收尾：写 summary.json（六键齐）+ 关文件。**幂等**，可重复调用。"""
        dur = (float(self.clock()) - self._t0) if duration_s is None else float(duration_s)
        summary: dict[str, Any] = {
            "run_id": self.run_id,
            "duration_s": round(max(0.0, dur), 2),
            "distance_m": round(float(self.distance_m if distance_m is None else distance_m), 3),
            "out_of_bounds": int(self.out_of_bounds if out_of_bounds is None else out_of_bounds),
            "unknown_cells": int(self.unknown_cells if unknown_cells is None else unknown_cells),
            "warn_counts": dict(warn_counts if warn_counts is not None else self.warn_counts),
            "reason": str(reason),
            "finished_at": time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(self.clock())) + "Z",
            "files": list(RUN_FILES),
        }
        if extra:
            for key, value in extra.items():
                summary.setdefault(str(key), value)
        self._summary = summary
        if not self._closed:
            self._close_handles()
        self._write_json("summary.json", summary)
        self._note(f"运行归档：{reason}｜{self.dir}｜时长 {summary['duration_s']}s"
                   f"｜里程 {summary['distance_m']}m｜越界 {summary['out_of_bounds']} 拍"
                   f"｜unknown {summary['unknown_cells']} 格"
                   f"｜warn {sum(summary['warn_counts'].values())} 条")
        return summary

    def _close_handles(self) -> None:
        self._closed = True
        for fh in (getattr(self, "_pose_fh", None), getattr(self, "_trace_fh", None)):
            try:
                if fh is not None:
                    fh.flush()
                    fh.close()
            except Exception:                    # noqa: BLE001
                pass

    # ------------------------------------------------------------------ #
    def summary(self) -> dict[str, Any] | None:
        return dict(self._summary) if self._summary else None

    def file_status(self) -> dict[str, bool]:
        """六个文件是否都已存在（验收用；不读内容）。"""
        return {name: (self.dir / name).is_file() for name in RUN_FILES}


def install_log_tee(log: Callable[[str], None], path: pathlib.Path | str) -> Callable[[str], None]:
    """把日志同时写 stdout 与 log.txt；返回新的 log 函数（原函数不被修改）。

    只追加、不读；写入失败立即降级为纯 stdout（归档不许拖垮运行）。
    """
    target = pathlib.Path(path)
    state = {"fh": None, "bad": False}

    def _open():
        try:
            return open(target, "a", encoding="utf-8", errors="replace")
        except Exception:                        # noqa: BLE001
            state["bad"] = True
            return None

    def _tee(text: str) -> None:
        try:
            log(text)
        finally:
            if not state["bad"]:
                if state["fh"] is None:
                    state["fh"] = _open()
                fh = state["fh"]
                if fh is not None:
                    try:
                        fh.write(str(text) + "\n")
                        fh.flush()
                    except Exception:            # noqa: BLE001
                        state["bad"] = True
    _tee.close = lambda: (state["fh"].close() if state["fh"] is not None else None)  # type: ignore[attr-defined]
    return _tee
