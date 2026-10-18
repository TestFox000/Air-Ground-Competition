#!/usr/bin/env python3
"""从 PDF 中提取文本（支持 CID 字体的 ToUnicode CMap 解码）

仅用标准库：解析 pdf 的 ToUnicode CMap，把内容流里的 (..) 字符串映射成 Unicode。
"""

import re
import sys
import zlib
import os


def decompress_streams(data: bytes):
    txt = data.decode('latin-1', 'replace')
    raw = re.findall(r'stream\r?\n(.*?)\r?\nendstream', txt, re.S)
    out = []
    for s in raw:
        for attempt in (s, s.lstrip('\r\n')):
            try:
                out.append(zlib.decompress(attempt.encode('latin-1')))
                break
            except Exception:
                continue
    return out


def parse_tounicode(cmap_bytes: bytes):
    """解析 ToUnicode CMap，返回 {code:int -> unicode str}"""
    text = cmap_bytes.decode('latin-1', 'replace')
    mapping = {}

    # bfchar: <src> <dst> ;
    for blk in re.findall(r'beginbfchar(.*?)endbfchar', text, re.S):
        for src, dst in re.findall(r'<([0-9A-Fa-f]+)>\s*<([0-9A-Fa-f]+)>', blk):
            try:
                key = int(src, 16)
                val = bytes.fromhex(dst).decode('utf-16-be', 'replace')
                mapping[key] = val
            except Exception:
                pass

    # bfrange: <lo> <hi> <dstStart> 或 <lo> <hi> [<d1> <d2> ...]
    for blk in re.findall(r'beginbfrange(.*?)endbfrange', text, re.S):
        for m in re.finditer(r'<([0-9A-Fa-f]+)>\s*<([0-9A-Fa-f]+)>\s*'
                             r'(?:<([0-9A-Fa-f]+)>|\[(.*?)\])', blk, re.S):
            lo, hi, dst, arr = m.groups()
            try:
                lo_i, hi_i = int(lo, 16), int(hi, 16)
            except Exception:
                continue
            if arr:
                items = re.findall(r'<([0-9A-Fa-f]+)>', arr)
                for i, it in enumerate(items):
                    if lo_i + i <= hi_i:
                        try:
                            mapping[lo_i + i] = bytes.fromhex(it).decode('utf-16-be', 'replace')
                        except Exception:
                            pass
            elif dst:
                try:
                    base = int(dst, 16)
                except Exception:
                    continue
                for i in range(hi_i - lo_i + 1):
                    try:
                        mapping[lo_i + i] = chr(base + i)
                    except Exception:
                        pass
    return mapping


def collect_cmaps(streams):
    """把所有 ToUnicode CMap 合并成一张大表（本工程 PDF 字体少，合并足够）"""
    merged = {}
    for s in streams:
        head = s[:400].decode('latin-1', 'replace')
        if 'beginbfchar' in head or 'beginbfrange' in head or \
           (b'beginbfchar' in s[:4000] or b'beginbfrange' in s[:4000]):
            merged.update(parse_tounicode(s))
    return merged


def extract_text(streams, cmap):
    """从内容流中提取文本"""
    out_lines = []
    for s in streams:
        # 跳过 CMap 自身
        if b'beginbfchar' in s[:4000] or b'beginbfrange' in s[:4000]:
            continue
        body = s.decode('latin-1', 'replace')
        # 每个 BT..ET 块内按 TD/Td/T* 分行
        for bt in re.findall(r'BT(.*?)ET', body, re.S):
            line = []
            for m in re.finditer(
                    r'<([0-9A-Fa-f\s]+)>\s*Tj|\(((?:[^()\\]|\\.)*)\)\s*Tj|'
                    r'\[(.*?)\]\s*TJ|(T\*|Td|TD|Tm)', bt, re.S):
                hexs, lit, arr, op = m.groups()
                if hexs is not None:
                    h = re.sub(r'\s+', '', hexs)
                    line.append(decode_hex(h, cmap))
                elif lit is not None:
                    line.append(decode_literal(lit, cmap))
                elif arr is not None:
                    for hm in re.finditer(r'<([0-9A-Fa-f\s]+)>|\(((?:[^()\\]|\\.)*)\)', arr, re.S):
                        hh, ll = hm.groups()
                        if hh is not None:
                            line.append(decode_hex(re.sub(r'\s+', '', hh), cmap))
                        elif ll is not None:
                            line.append(decode_literal(ll, cmap))
                elif op in ('T*', 'Td', 'TD', 'Tm'):
                    if line:
                        out_lines.append(''.join(line))
                        line = []
            if line:
                out_lines.append(''.join(line))
    return out_lines


def decode_hex(h: str, cmap) -> str:
    if len(h) % 4 != 0:
        if len(h) % 2 != 0:
            h = h[:-1]
        # 2 字节编码
        out = []
        for i in range(0, len(h), 2):
            code = int(h[i:i + 2], 16)
            out.append(cmap.get(code, ''))
        return ''.join(out)
    out = []
    for i in range(0, len(h), 4):
        code = int(h[i:i + 4], 16)
        out.append(cmap.get(code, ''))
    return ''.join(out)


def decode_literal(lit: str, cmap) -> str:
    # 处理转义
    lit = lit.replace('\\(', '(').replace('\\)', ')').replace('\\\\', '\\')
    if not cmap:
        return lit
    # 单字节码
    out = []
    for ch in lit:
        out.append(cmap.get(ord(ch), ch))
    return ''.join(out)


def main():
    path = sys.argv[1]
    out_path = sys.argv[2] if len(sys.argv) > 2 else None
    with open(path, 'rb') as f:
        data = f.read()

    streams = decompress_streams(data)
    cmap = collect_cmaps(streams)
    print('CMap 条目: %d' % len(cmap))
    lines = extract_text(streams, cmap)
    text = '\n'.join(l for l in lines if l.strip())
    print('提取行数: %d，字符数: %d' % (len(lines), len(text)))
    if out_path:
        with open(out_path, 'w', encoding='utf-8') as f:
            f.write(text)
        print('写入 %s' % out_path)
    else:
        print('--- 前 3000 字 ---')
        print(text[:3000])


if __name__ == '__main__':
    main()
