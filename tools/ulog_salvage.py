#!/usr/bin/env python3
# coding=utf-8
"""ULog 容错打捞器：pyulog 在定义段报截断时，尽最大努力把还能读的部分抽出来。

为什么需要它：2026-10-15 的 log_1_2026-10-15-17-10-50.ulg（627KB）文件头
magic 完好，但 pyulog 在 "file definitions" 阶段就抛 `unpack requires a
buffer of 3 bytes`——典型原因是文件尾部被下载截断，或中段有撕裂。pyulog
的策略是整条消息读不出来就全文件拒读；本工具改为逐消息遍历，遇到坏帧就
向前重同步（扫描下一个"尺寸合理 + 类型字母合法"的帧头），把定义段
（format/info/parameter）、消息订阅表、logged string 和能解码的数据段
尽量救回来。

用法：
    python tools/ulog_salvage.py <file.ulg>            # 全量打捞报告
    python tools/ulog_salvage.py <file.ulg> --json     # 机器可读摘要
    python tools/ulog_salvage.py <file.ulg> --params   # 只列参数表
"""
from __future__ import annotations

import argparse
import json
import struct
import sys
from dataclasses import dataclass, field
from pathlib import Path

ULOG_MAGIC = b"ULog\x01\x12\x35"
# ULog 帧类型字母（定义段 + 数据段）
KNOWN_TYPES = set(b"FIABPDQMSLRO")
# 定义段帧类型（数据段开始前的合法类型）
DEFN_TYPES = set(b"FIMPQAB")

# ULog 基础类型 -> (struct 代码, 字节数)
_BASE_TYPES = {
    "int8_t": ("b", 1), "uint8_t": ("B", 1),
    "int16_t": ("h", 2), "uint16_t": ("H", 2),
    "int32_t": ("i", 4), "uint32_t": ("I", 4),
    "int64_t": ("q", 8), "uint64_t": ("Q", 8),
    "float": ("f", 4), "double": ("d", 8),
    "bool": ("?", 1), "char": ("c", 1),
}


@dataclass
class SalvageResult:
    """打捞结果：能读多少算多少，坏点位置也记录下来。"""
    path: str
    file_size: int
    header_ok: bool = False
    formats: dict = field(default_factory=dict)        # name -> [(type, name, array)]
    infos: dict = field(default_factory=dict)          # key -> value(str)
    parameters: dict = field(default_factory=dict)     # name -> value
    subscriptions: dict = field(default_factory=dict)  # msg_id -> topic
    logged_strings: list = field(default_factory=list) # (level, ts, text)
    dropouts: list = field(default_factory=list)       # (offset, duration_ms)
    data_counts: dict = field(default_factory=dict)    # msg_id -> 解码成功帧数
    data_first_last_ts: dict = field(default_factory=dict)  # msg_id -> (first,last)us
    corrupt_offsets: list = field(default_factory=list)     # 重同步点
    truncation_offset: int | None = None               # 文件截断点（如确认）
    errors: list = field(default_factory=list)


def _parse_format_fields(fmt_def: str) -> list[tuple[str, str, int]]:
    """'uint64_t timestamp;float[3] x' -> [(类型, 字段名, 数组长度)]"""
    out = []
    for field_def in fmt_def.split(";"):
        field_def = field_def.strip()
        if not field_def or " " not in field_def:
            continue
        ftype, fname = field_def.rsplit(None, 1)
        arr = 1
        if "[" in fname:
            fname, arr_s = fname.rstrip("]").split("[")
            try:
                arr = int(arr_s)
            except ValueError:
                arr = 1
        out.append((ftype, fname, arr))
    return out


def _decode_value(ftype: str, data: bytes):
    """按 ULog 类型字符串解码一个值（用于 info/parameter 帧）。"""
    if ftype.startswith("char[") or ftype.startswith("uint8_t["):
        return data.split(b"\x00")[0].decode("utf-8", errors="replace")
    code_size = _BASE_TYPES.get(ftype)
    if code_size is None:
        return None
    code, size = code_size
    if len(data) < size:
        return None
    return struct.unpack("<" + code, data[:size])[0]


class _FormatDecoder:
    """用已解析的 format 定义把数据帧解成 dict（尽力而为，只解基础类型）。"""

    def __init__(self, name: str, fields: list[tuple[str, str, int]]):
        self.name = name
        self.fields = fields
        self.size = 0
        self.decodable = True
        for ftype, _fname, arr in fields:
            cs = _BASE_TYPES.get(ftype)
            if cs is None:
                self.decodable = False  # 嵌套消息，暂不支持
                return
            self.size += cs[1] * arr

    def decode(self, payload: bytes) -> dict | None:
        if not self.decodable or len(payload) < self.size:
            return None
        out = {}
        off = 0
        for ftype, fname, arr in self.fields:
            code, size = _BASE_TYPES[ftype]
            if arr == 1:
                out[fname] = struct.unpack_from("<" + code, payload, off)[0]
            else:
                out[fname] = list(
                    struct.unpack_from("<" + code * arr, payload, off))
            off += size * arr
        return out


def _looks_like_frame(buf: bytes, off: int, allow_data: bool) -> bool:
    """重同步判定：off 处是否像合法帧头（尺寸能装下 + 类型字母合法）。"""
    if off + 3 > len(buf):
        return False
    size = struct.unpack_from("<H", buf, off)[0]
    mtype = buf[off + 2]
    if mtype not in (KNOWN_TYPES if allow_data else DEFN_TYPES):
        return False
    return off + 3 + size <= len(buf)


def salvage(path: Path, collect_data: bool = True,
            max_resync_scan: int = 1 << 20) -> SalvageResult:
    """逐消息遍历 ULog；坏帧重同步；返回 SalvageResult。"""
    buf = path.read_bytes()
    res = SalvageResult(path=str(path), file_size=len(buf))
    if len(buf) < 16 or not buf.startswith(ULOG_MAGIC):
        res.errors.append("bad magic: 不是 ULog 文件")
        return res
    res.header_ok = True
    # 头：magic(7) + version(1) + timestamp_us(8) = 16 字节
    # （magic 是 "ULog" + \x01\x12\x35 共 7 字节，见 ULog 规范）
    off = 16
    in_definitions = True
    decoders: dict[int, _FormatDecoder] = {}
    sub_names: dict[int, str] = {}

    while off + 3 <= len(buf):
        if not _looks_like_frame(buf, off, allow_data=not in_definitions):
            # 找下一个像帧头的位置重同步
            sync = None
            scan_end = min(len(buf) - 3, off + max_resync_scan)
            for cand in range(off + 1, scan_end + 1):
                if _looks_like_frame(buf, cand, allow_data=True):
                    sync = cand
                    break
            if sync is None:
                res.truncation_offset = off
                break
            res.corrupt_offsets.append((off, sync - off))
            off = sync
            in_definitions = False  # 撕裂后保守起见按数据段处理
            continue

        size, mtype_b = struct.unpack_from("<H", buf, off)[0], buf[off + 2]
        mtype = chr(mtype_b)
        payload = buf[off + 3: off + 3 + size]
        frame_off = off
        off += 3 + size

        try:
            if mtype == "F":
                # format: "name:type;name:type;..."
                text = payload.decode("utf-8", errors="replace")
                if ":" in text:
                    name, defs = text.split(":", 1)
                    fields = _parse_format_fields(defs)
                    res.formats[name] = fields
            elif mtype == "I" or mtype == "M":
                # info: key_len(u8) + "type key" + value
                klen = payload[0]
                key = payload[1:1 + klen].decode("utf-8", errors="replace")
                ftype, kname = key.split(" ", 1)
                vdata = payload[1 + klen:]
                if ftype.startswith(("char[", "uint8_t[")):
                    res.infos[kname] = _decode_value(ftype, vdata)
                else:
                    res.infos[kname] = _decode_value(ftype, vdata)
            elif mtype in ("P", "Q"):
                klen = payload[0]
                key = payload[1:1 + klen].decode("utf-8", errors="replace")
                ftype, kname = key.split(" ", 1)
                # Q 帧多一个 default 值段，这里只取当前值（紧挨 key 后）
                res.parameters[kname] = _decode_value(
                    ftype, payload[1 + klen:])
            elif mtype == "A":
                # subscription: msg_id(u16) [+ multi_id(u1)] + topic 名
                msg_id = struct.unpack_from("<H", payload, 0)[0]
                # 兼容 v1（multi_id 在 msg_id 后一字节）：名字从可打印段找
                name_raw = payload[3:]
                res.subscriptions[msg_id] = \
                    name_raw.decode("utf-8", errors="replace").strip("\x00")
                in_definitions = False
            elif mtype == "L":
                level = payload[0]
                ts = struct.unpack_from("<Q", payload, 1)[0]
                res.logged_strings.append(
                    (level, ts, payload[9:].decode("utf-8", errors="replace")))
                in_definitions = False
            elif mtype == "S":
                in_definitions = False
            elif mtype == "O":
                dur = struct.unpack_from("<H", payload, 0)[0]
                res.dropouts.append((frame_off, dur))
            elif mtype == "D":
                in_definitions = False
                if not collect_data:
                    continue
                msg_id = struct.unpack_from("<H", payload, 0)[0]
                res.data_counts[msg_id] = res.data_counts.get(msg_id, 0) + 1
                dec = decoders.get(msg_id)
                if dec is None and msg_id in sub_names:
                    topic = sub_names[msg_id]
                    if topic in res.formats:
                        dec = _FormatDecoder(topic, res.formats[topic])
                        decoders[msg_id] = dec
                if dec is not None:
                    row = dec.decode(payload[2:])
                    if row is not None and "timestamp" in row:
                        first_last = res.data_first_last_ts.get(msg_id)
                        ts = row["timestamp"]
                        if first_last is None:
                            res.data_first_last_ts[msg_id] = (ts, ts)
                        else:
                            res.data_first_last_ts[msg_id] = \
                                (first_last[0], ts)
            # B/F/I/M/P/Q 在定义段；遇到 A/D/L/S 即进入数据段
        except Exception as exc:  # 单帧解析失败不影响整体打捞
            res.errors.append(f"frame @{frame_off} type={mtype}: {exc}")

        # 订阅帧的名字回填给解码器
        for msg_id, topic in res.subscriptions.items():
            sub_names[msg_id] = topic

    if res.truncation_offset is None and off < len(buf):
        res.truncation_offset = off
    return res


def _fmt_val(v):
    if isinstance(v, float):
        return f"{v:.6g}"
    return str(v)


def render_text(res: SalvageResult) -> str:
    """人类可读的打捞报告。"""
    lines = []
    lines.append(f"文件: {res.path}  ({res.file_size} 字节)")
    lines.append(f"ULog 头: {'完好' if res.header_ok else '损坏'}")
    if res.truncation_offset is not None:
        pct = 100.0 * res.truncation_offset / max(1, res.file_size)
        lines.append(
            f"可读边界: 偏移 {res.truncation_offset} ({pct:.1f}% 处)"
            "——其后内容丢失或撕裂")
    if res.corrupt_offsets:
        lines.append(f"撕裂/重同步点 {len(res.corrupt_offsets)} 处: "
                     + ", ".join(f"@{o}(跳过{n}B)" for o, n
                                 in res.corrupt_offsets[:10]))
    lines.append(f"format 定义: {len(res.formats)} 个")
    lines.append(f"info 条目: {len(res.infos)} 个")
    lines.append(f"参数: {len(res.parameters)} 个")
    lines.append(f"订阅 topic: {len(res.subscriptions)} 个")
    lines.append(f"日志字符串: {len(res.logged_strings)} 条")
    lines.append(f"数据 dropout: {len(res.dropouts)} 段")
    if res.infos:
        lines.append("\n── info ──")
        for k in sorted(res.infos):
            lines.append(f"  {k} = {_fmt_val(res.infos[k])}")
    if res.parameters:
        lines.append("\n── 参数 ──")
        for k in sorted(res.parameters):
            lines.append(f"  {k} = {_fmt_val(res.parameters[k])}")
    if res.subscriptions:
        lines.append("\n── 订阅 topic ──")
        for msg_id in sorted(res.subscriptions):
            cnt = res.data_counts.get(msg_id, 0)
            ts = res.data_first_last_ts.get(msg_id)
            span = ""
            if ts:
                span = f"  时间跨度 {(ts[1]-ts[0])/1e6:.1f}s"
            lines.append(f"  [{msg_id:3d}] {res.subscriptions[msg_id]}"
                         f"  解码帧数={cnt}{span}")
    if res.logged_strings:
        lines.append("\n── 日志字符串（前 40 条）──")
        for level, ts, text in res.logged_strings[:40]:
            lines.append(f"  [{ts/1e6:9.3f}s] L{level} {text}")
    if res.errors:
        lines.append(f"\n── 解析告警 {len(res.errors)} 条（前 10）──")
        for e in res.errors[:10]:
            lines.append(f"  {e}")
    return "\n".join(lines)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="ULog 容错打捞")
    ap.add_argument("ulog", type=Path)
    ap.add_argument("--json", action="store_true", help="输出机器可读摘要")
    ap.add_argument("--params", action="store_true", help="只列参数表")
    args = ap.parse_args(argv)

    res = salvage(args.ulog)
    if args.params:
        for k in sorted(res.parameters):
            print(f"{k} = {_fmt_val(res.parameters[k])}")
        return 0
    if args.json:
        print(json.dumps({
            "file_size": res.file_size,
            "header_ok": res.header_ok,
            "truncation_offset": res.truncation_offset,
            "corrupt_offsets": res.corrupt_offsets,
            "formats": len(res.formats),
            "infos": res.infos,
            "parameters": res.parameters,
            "subscriptions": res.subscriptions,
            "logged_strings": len(res.logged_strings),
            "data_counts": res.data_counts,
            "errors": res.errors[:20],
        }, ensure_ascii=False, indent=2, default=str))
        return 0
    print(render_text(res))
    return 0


if __name__ == "__main__":
    sys.exit(main())
