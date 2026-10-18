#!/usr/bin/env python3
"""从 PDF 提取文本（字节级解析，避免 latin-1 往返破坏 hex 码位）

关键点：解压后的内容流必须**直接在 bytes 上**查找 <...> / (...) 字符串，
        不能先 decode('latin-1') 再 encode 回去 —— 那会丢掉 >0x7F 的字节，
        使 hex 码位与 ToUnicode CMap 对不上（表现为数字变成字母）。
"""

import re
import sys
import zlib


def streams(data: bytes):
    """按 byte 精确切出 stream...endstream 段"""
    out = []
    pos = 0
    while True:
        i = data.find(b'stream', pos)
        if i < 0:
            break
        j = i + len(b'stream')
        # 跳过 CR/LF
        if data[j:j + 2] == b'\r\n':
            j += 2
        elif data[j:j + 1] in (b'\n', b'\r'):
            j += 1
        k = data.find(b'endstream', j)
        if k < 0:
            break
        out.append(data[j:k])
        pos = k + len(b'endstream')
    return out


def inflate(raw: bytes):
    for candidate in (raw, raw.rstrip(b'\r\n'), raw.lstrip(b'\r\n')):
        try:
            return zlib.decompress(candidate)
        except Exception:
            continue
        try:
            d = zlib.decompressobj()
            return d.decompress(candidate)
        except Exception:
            continue
    return None


def parse_cmap(b: bytes):
    """解析 ToUnicode CMap（字节级）"""
    mapping = {}
    for blk in re.findall(rb'beginbfchar(.*?)endbfchar', b, re.S):
        for src, dst in re.findall(rb'<([0-9A-Fa-f]+)>\s*<([0-9A-Fa-f]+)>', blk):
            try:
                mapping[int(src, 16)] = bytes.fromhex(dst.decode()).decode('utf-16-be', 'replace')
            except Exception:
                pass
    for blk in re.findall(rb'beginbfrange(.*?)endbfrange', b, re.S):
        for m in re.finditer(rb'<([0-9A-Fa-f]+)>\s*<([0-9A-Fa-f]+)>\s*'
                             rb'(?:<([0-9A-Fa-f]+)>|\[(.*?)\])', blk, re.S):
            lo, hi, dst, arr = m.groups()
            try:
                lo_i, hi_i = int(lo, 16), int(hi, 16)
            except Exception:
                continue
            if arr:
                for i, it in enumerate(re.findall(rb'<([0-9A-Fa-f]+)>', arr)):
                    if lo_i + i <= hi_i:
                        try:
                            mapping[lo_i + i] = bytes.fromhex(it.decode()).decode('utf-16-be', 'replace')
                        except Exception:
                            pass
            elif dst:
                try:
                    base = int(dst, 16)
                except Exception:
                    continue
                for i in range(min(hi_i - lo_i + 1, 4096)):
                    try:
                        mapping[lo_i + i] = chr(base + i)
                    except Exception:
                        pass
    return mapping


HEX_RE = re.compile(rb'<([0-9A-Fa-f\s]*)>', re.S)
LIT_RE = re.compile(rb'\(((?:[^()\\]|\\.)*)\)', re.S)
TJ_RE = re.compile(rb'\[(.*?)\]\s*TJ', re.S)
TJ_ONE_RE = re.compile(rb'<([0-9A-Fa-f\s]*)>\s*Tj|\(((?:[^()\\]|\\.)*)\)\s*Tj', re.S)


def decode_hex(h: bytes, cmap, two_byte=True):
    s = re.sub(rb'\s+', b'', h).decode('latin-1')
    if len(s) % 2:
        s = s[:-1]
    out = []
    step = 4 if (two_byte and len(s) % 4 == 0) else 2
    for i in range(0, len(s), step):
        try:
            code = int(s[i:i + step], 16)
        except Exception:
            continue
        out.append(cmap.get(code, ''))
    return ''.join(out)


def decode_lit(l: bytes, cmap):
    l = l.replace(b'\\(', b'(').replace(b'\\)', b')').replace(b'\\\\', b'\\')
    if not cmap:
        return l.decode('latin-1', 'replace')
    out = []
    for ch in l:
        out.append(cmap.get(ch, chr(ch) if 32 <= ch < 127 else ''))
    return ''.join(out)


def extract(path, two_byte=True, per_line=True):
    data = open(path, 'rb').read()
    raws = streams(data)
    blobs = []
    for r in raws:
        d = inflate(r)
        if d is not None:
            blobs.append(d)

    # 合并所有 CMap
    cmap = {}
    for b in blobs:
        if b'beginbfchar' in b or b'beginbfrange' in b:
            cmap.update(parse_cmap(b))

    lines = []
    for b in blobs:
        if b'beginbfchar' in b or b'beginbfrange' in b:
            continue
        if b'TJ' not in b and b'Tj' not in b:
            continue
        # 按 TJ / Tj 顺序抓文本，遇 T* / Td / TD / Tm 则换行
        parts = []
        for m in re.finditer(
                rb'\[(.*?)\]\s*TJ|<([0-9A-Fa-f\s]*)>\s*Tj|'
                rb'\(((?:[^()\\]|\\.)*)\)\s*Tj|(T\*|Td|TD|Tm)',
                b, re.S):
            arr, hexs, lit, op = m.groups()
            if arr is not None:
                seg = []
                for hm in re.finditer(rb'<([0-9A-Fa-f\s]*)>|\(((?:[^()\\]|\\.)*)\)', arr, re.S):
                    hh, ll = hm.groups()
                    if hh is not None:
                        seg.append(decode_hex(hh, cmap, two_byte))
                    elif ll is not None:
                        seg.append(decode_lit(ll, cmap))
                parts.append(''.join(seg))
            elif hexs is not None:
                parts.append(decode_hex(hexs, cmap, two_byte))
            elif lit is not None:
                parts.append(decode_lit(lit, cmap))
            elif op is not None:
                if per_line:
                    if parts:
                        lines.append(''.join(parts))
                        parts = []
        if parts:
            lines.append(''.join(parts))
    return cmap, lines


def main():
    path = sys.argv[1]
    out_path = sys.argv[2] if len(sys.argv) > 2 else None
    two_byte = '--one-byte' not in sys.argv
    cmap, lines = extract(path, two_byte)
    text = '\n'.join(l for l in lines if l.strip())
    print('CMap 条目: %d  文本行: %d  字符: %d' % (len(cmap), len(lines), len(text)))
    if out_path:
        with open(out_path, 'w', encoding='utf-8') as f:
            f.write(text)
        print('写入 %s' % out_path)
    else:
        print(text[:4000])


if __name__ == '__main__':
    main()
