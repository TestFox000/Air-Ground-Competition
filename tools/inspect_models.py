# -*- coding: utf-8 -*-
"""模型版本体检：MD5 去重 + ONNX 输入输出结构解析。"""
import hashlib
from datetime import datetime
from pathlib import Path

try:
    import onnx
except Exception:
    onnx = None

DIRS = [
    Path(r"C:\Users\29212\Documents\xwechat_files\wxid_esyua6tojf0x22_b065\msg\file\2026-09"),
    Path(r"C:\Users\29212\Desktop\比赛方案与相关文件\模型\标靶识别"),
    Path(r"C:\Users\29212\Desktop\比赛方案与相关文件\模型\障碍物识别"),
    Path(r"C:\Users\29212\Desktop\比赛方案与相关文件\标靶打击\官方模型文件"),
]

EXTS = {".rknn", ".onnx", ".pt"}


def md5(p: Path, chunk=1 << 20):
    h = hashlib.md5()
    with p.open("rb") as f:
        while True:
            b = f.read(chunk)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


items = []
for d in DIRS:
    if not d.exists():
        print(f"[缺失目录] {d}")
        continue
    for p in sorted(d.iterdir()):
        if p.is_file() and p.suffix.lower() in EXTS:
            items.append(p)

print("=" * 100)
print(f"{'文件':<22}{'大小KB':>9}{'修改时间':>20}  {'MD5':<34}{'所在目录'}")
print("=" * 100)
groups = {}
for p in items:
    m = md5(p)
    size_kb = p.stat().st_size / 1024
    mt = datetime.fromtimestamp(p.stat().st_mtime).strftime("%Y-%m-%d %H:%M")
    groups.setdefault(m, []).append(p.name)
    print(f"{p.name:<22}{size_kb:>9.0f}{mt:>20}  {m:<34}{p.parent.name}")

print("\n【重复文件分组】")
dup = {k: v for k, v in groups.items() if len(v) > 1}
if dup:
    for k, v in dup.items():
        print(f"  {k[:12]}... -> {v}")
else:
    print("  无重复（所有文件 MD5 均不同）")

print("\n" + "=" * 100)
print("【ONNX 结构解析】")
print("=" * 100)
if onnx is None:
    print("未安装 onnx，跳过结构解析")
else:
    def dims(v):
        return [d.dim_param or d.dim_value for d in v.type.tensor_type.shape.dim]

    for p in items:
        if p.suffix.lower() != ".onnx":
            continue
        try:
            m = onnx.load(str(p), load_external_data=False)
            onnx.checker.check_model(m, full_check=False)
        except Exception as e:
            print(f"\n{p.name}: 加载/校验失败 -> {e}")
            continue
        print(f"\n--- {p.name}  ({p.parent.name}) ---")
        print(f"  opset        : {[o.version for o in m.opset_import]}")
        print(f"  ir_version   : {m.ir_version}")
        for i in m.graph.input:
            print(f"  input : {i.name:<16} shape={dims(i)}")
        for o in m.graph.output:
            print(f"  output: {o.name:<16} shape={dims(o)}")
        if m.metadata_props:
            print("  metadata:")
            for kv in m.metadata_props:
                val = kv.value
                if len(val) > 200:
                    val = val[:200] + " ...(截断)"
                print(f"      {kv.key} = {val}")
        n_nodes = len(m.graph.node)
        print(f"  nodes        : {n_nodes}")
