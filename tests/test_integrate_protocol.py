"""ag_integrate intel_protocol 二进制 :50000 协议解析测试。"""

import random
import re
import struct
from pathlib import Path

import pytest

from ag_integrate.intel_protocol import (
    HEADER_SIZE,
    OBJECT_SIZE,
    ObjectType,
    IntelFrame,
    IntelObject,
    build,
    crc16,
    parse,
)

REPO = Path(__file__).resolve().parents[1]
DOCS = REPO / "docs"
TYPES_HPP = REPO / "onboard" / "dzuav" / "include" / "dzuav" / "types.hpp"
MAIN_CPP = REPO / "onboard" / "dzuav" / "src" / "main.cpp"


def make_object(type_index: int = 0) -> IntelObject:
    return IntelObject(ObjectType(type_index), 0.12 * type_index, -0.34, 0.35, 0.35)


def test_intel_frame_golden_bytes():
    """0x4001 黄金帧：逐字节锁定布局，任何一端（C++ UdpBroadcaster 或本模块）
    改了字段顺序/大小/偏移都会在这里炸——机上协议不可静默漂移。

    布局（小端，1 个目标共 52 字节）：
        magic4 + version1 + ts8 + msgid2(0x4001) + len4(=2+25*count) + count2
        + [type1 + x double8 + y double8 + length float4 + width float4] * count
        + crc2 + suffix4
    """
    # length/width 是 float32：选二进制可精确表示的值，避免往返误差干扰断言
    obj = IntelObject(ObjectType.TARGET_BLUE, 1.25, -2.5, 0.5, 0.25)
    frame = IntelFrame(1_700_000_000_123, (obj,))
    raw = build(frame)

    assert len(raw) == HEADER_SIZE + OBJECT_SIZE + 2 + 4 == 52
    assert raw[0:4] == b"\xef\xef\xef\xef"
    assert raw[4] == 0x10
    ts, msg_id, declared, count = struct.unpack_from("<QHIH", raw, 5)
    assert (ts, msg_id, declared, count) == (frame.timestamp_ms, 0x4001, 27, 1)
    type_index, x, y, length, width = struct.unpack_from("<Bddff", raw, HEADER_SIZE)
    assert type_index == int(ObjectType.TARGET_BLUE)
    assert (x, y, length, width) == (1.25, -2.5, 0.5, 0.25)
    body_end = HEADER_SIZE + OBJECT_SIZE
    (received_crc,) = struct.unpack_from("<H", raw, body_end)
    assert received_crc == crc16(raw[4:body_end])
    assert raw[body_end + 2 :] == b"\xfe\xfe\xfe\xfe"

    # 往返一致
    decoded = parse(raw)
    assert decoded.timestamp_ms == frame.timestamp_ms
    got = decoded.objects[0]
    assert got.type is ObjectType.TARGET_BLUE
    assert (got.x, got.y, got.length, got.width) == (1.25, -2.5, 0.5, 0.25)


def test_car_pose_golden_bytes():
    """0x4002 黄金帧：逐字节锁定布局，任何一端（C++ UdpBroadcaster 或本模块）
    改了字段顺序/大小/偏移都会在这里炸——机上机上协议不可静默漂移。

    布局（小端，共 45 字节）：
        magic4 + version1 + ts8 + msgid2(0x4002) + len4(=20)
        + x double8 + y double8 + yaw float4 + crc2 + suffix4
    """
    from ag_integrate.intel_protocol import (
        CAR_POSE_SIZE, HEADER_SIZE_NO_COUNT, CarPose, build_car_pose, parse_car_pose,
    )

    pose = CarPose(1_700_000_000_123, 1.5, -2.25, 0.75)
    raw = build_car_pose(pose)

    assert len(raw) == HEADER_SIZE_NO_COUNT + CAR_POSE_SIZE + 2 + 4 == 45
    assert raw[0:4] == b"\xef\xef\xef\xef"
    assert raw[4] == 0x10
    ts, msg_id, declared = struct.unpack_from("<QHI", raw, 5)
    assert (ts, msg_id, declared) == (pose.timestamp_ms, 0x4002, 20)
    x, y, yaw = struct.unpack_from("<ddf", raw, HEADER_SIZE_NO_COUNT)
    assert (x, y, yaw) == (1.5, -2.25, 0.75)
    body_end = HEADER_SIZE_NO_COUNT + CAR_POSE_SIZE
    (received_crc,) = struct.unpack_from("<H", raw, body_end)
    assert received_crc == crc16(raw[4:body_end])
    assert raw[body_end + 2 :] == b"\xfe\xfe\xfe\xfe"

    # 往返一致
    decoded = parse_car_pose(raw)
    assert decoded.timestamp_ms == pose.timestamp_ms
    assert (decoded.x, decoded.y, decoded.yaw) == (pose.x, pose.y, pose.yaw)


def test_roundtrip_empty_and_many():
    for count in (0, 1, 7):
        frame = IntelFrame(
            1_700_000_000_123,
            tuple(make_object(i % 7) for i in range(count)),
        )
        raw = build(frame)
        assert len(raw) == HEADER_SIZE + OBJECT_SIZE * count + 2 + 4
        decoded = parse(raw)
        assert decoded.timestamp_ms == frame.timestamp_ms
        assert len(decoded.objects) == count
        for got, exp in zip(decoded.objects, frame.objects):
            assert got.type == exp.type
            assert got.x == pytest.approx(exp.x)
            assert got.y == pytest.approx(exp.y)
            assert got.length == pytest.approx(exp.length, abs=1e-6)
            assert got.width == pytest.approx(exp.width, abs=1e-6)


def test_fields_are_preserved():
    obj = IntelObject(ObjectType.TARGET_BLUE, 1.25, 2.75, 0.40, 0.30)
    frame = IntelFrame(999, (obj,))
    decoded = parse(build(frame))
    assert decoded.timestamp_ms == 999
    got = decoded.objects[0]
    assert got.type is ObjectType.TARGET_BLUE
    assert got.x == pytest.approx(1.25)
    assert got.y == pytest.approx(2.75)


def test_crc16_known_value():
    # CRC-16/Modbus check value for "123456789" is 0x4B37
    assert crc16(b"123456789") == 0x4B37


def _crc16_bitwise(data: bytes) -> int:
    """逐位参考实现（与机上 UdpBroadcaster.crc 的写法一一对应）。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def test_crc16_table_matches_bitwise_reference():
    # 查表实现必须与逐位参考在任意输入上逐位等价
    rng = random.Random(20261001)
    for length in (0, 1, 2, 7, 21, 100, 546):
        blob = bytes(rng.randrange(256) for _ in range(length))
        assert crc16(blob) == _crc16_bitwise(blob), f"mismatch at length {length}"


def test_crc16_accepts_memoryview_zero_copy_span():
    raw = build(IntelFrame(1, (make_object(0), make_object(5))))
    mv = memoryview(raw)
    # 与切片拷贝结果一致
    assert crc16(mv[4:52]) == crc16(raw[4:52])
    # 空区间
    assert crc16(mv[4:4]) == 0xFFFF


def _corrupt(raw: bytes, **changes) -> bytes:
    raw = bytearray(raw)
    if "magic" in changes:
        raw[0] = 0x00
    if "version" in changes:
        raw[4] = 0x11
    if "crc" in changes:
        raw[-6] ^= 0xFF
    if "suffix" in changes:
        raw[-1] = 0x00
    if "length" in changes:
        struct.pack_into("<I", raw, 15, changes["length"])
    if "size" in changes:
        del raw[-1]
    return bytes(raw)


@pytest.mark.parametrize(
    "change",
    ["magic", "version", "crc", "suffix", "length", "size"],
)
def test_malformed_frames_rejected(change):
    raw = build(IntelFrame(1, (make_object(0), make_object(2))))
    bad = _corrupt(raw, **{change: True})
    with pytest.raises(ValueError):
        parse(bad)


def test_truncated_and_nan_rejected():
    with pytest.raises(ValueError):
        parse(b"\xef" * 10)

    raw = bytearray(build(IntelFrame(1, (make_object(0),))))
    # overwrite x (float64 at first object) with NaN
    struct.pack_into("<d", raw, HEADER_SIZE + 1, float("nan"))
    with pytest.raises(ValueError):
        parse(bytes(raw))


# ------------------------------------------------- 枚举语义（跨端逐位对齐）


def _object_class_from_types_hpp() -> list[tuple[str, int]]:
    """解析 types.hpp 的 `enum class ObjectClass`，返回 [(名, 值)]（CamelCase 原样）。"""
    text = TYPES_HPP.read_text(encoding="utf-8", errors="replace")
    m = re.search(r"enum\s+class\s+ObjectClass\b[^{]*\{(.*?)\}", text, re.S)
    assert m, "types.hpp 里找不到 enum class ObjectClass"
    members = re.findall(r"^\s*([A-Za-z_]\w*)\s*=\s*(\d+)", m.group(1), re.M)
    assert members, "ObjectClass 成员解析为空"
    return [(name, int(value)) for name, value in members]


def test_object_type_matches_onboard_object_class_bit_for_bit():
    """车端解码用的 ObjectType 必须与机载 ObjectClass 逐位同值同名。

    机载写的是 `uint8_t(x.type)`（UdpBroadcaster.cpp），车端按 ObjectType 反解。
    错一位不报错、不崩、不告警，只是把 5=OBSTACLE 当成"target_blue"开火——
    interface_contract.md §2.4 因此明写"改动任一侧都必须三处同改"。
    但此前只有 types.hpp ↔ video_recon.NAMES ↔ hbb_postprocess.CLASS_NAMES
    那一条链被测试锁住（见 tests/test_hbb_postprocess.py），**真正解析线缆字节
    的这一侧没人守**：把 ObjectType 改回 7 类，全套测试照样全绿。
    """
    cpp = _object_class_from_types_hpp()
    py = [(m.name, m.value) for m in ObjectType]
    # CamelCase → SNAKE_CASE：CellLabel → CELL_LABEL，BucketCyan → BUCKET_CYAN
    cpp_snake = [(re.sub(r"(?<!^)(?=[A-Z])", "_", n).upper(), v) for n, v in cpp]
    assert cpp_snake == py, (
        "车端 ObjectType 与机载 ObjectClass 不一致（顺序、命名或编号）：\n"
        f"  types.hpp        : {cpp_snake}\n"
        f"  intel_protocol   : {py}")


def test_object_type_matches_main_cpp_class_name_table():
    """ObjectType 的 docstring 自称"顺序必须与 dzuav main.cpp 的 className 一致"。

    这句话此前没有任何测试兑现。className 是标注图上打给操作员看的字，
    与线缆上的整数各走一条路——两者分叉时，操作员看到的名字与车端理解的
    语义就不是同一个东西，而 HUD 上完全看不出来。
    """
    text = MAIN_CPP.read_text(encoding="utf-8", errors="replace")
    m = re.search(r"const char\* className\(ObjectClass[^{]*\{(.*?)\n\}", text, re.S)
    assert m, "main.cpp 里找不到 className 函数"
    names = re.findall(r'"([a-z_]+)"', m.group(1))
    cpp = _object_class_from_types_hpp()
    snake = [re.sub(r"(?<!^)(?=[A-Z])", "_", n).lower() for n, _ in cpp]
    assert names == snake, f"main.cpp className 与 types.hpp 不一致:\n  {names}\n  {snake}"
    assert [m.name.lower() for m in ObjectType] == names, (
        f"车端 ObjectType 与 main.cpp className 不一致:\n"
        f"  {[m.name.lower() for m in ObjectType]}\n  {names}")


# ------------------------------------------------- 文档不得携带旧契约


def test_no_doc_carries_the_legacy_seven_class_wire_enum():
    """任何文档都不得再抄一份 7 类时代的线缆枚举。

    `docs/无人机工作流程.md` 曾在 0x4001 的字段表里写着
    `type uint8 (0=mine,1=car,2=arrow,3=target_red,4=cell_label,5=target_blue,
    6=obstacle)`，与真源逐位错位：照着它实现车端解析，会把 5=OBSTACLE 读成
    target_blue 直接开火、把 4=TARGET_BLUE 读成 cell_label 直接丢掉。
    枚举只允许有一处定义（types.hpp），文档只能指向 interface_contract.md §2.4。
    """
    offenders = []
    for doc in sorted(DOCS.rglob("*.md")):
        text = doc.read_text(encoding="utf-8", errors="replace")
        for pat in (r"0\s*=\s*mine", r"1\s*=\s*car\s*,\s*2\s*=\s*arrow"):
            if re.search(pat, text, re.I):
                offenders.append(f"{doc.relative_to(REPO)} 命中 {pat!r}")
    assert not offenders, (
        "这些文档抄了 7 类旧线缆枚举，与 types.hpp 的 13 类真源逐位错位：\n  "
        + "\n  ".join(offenders))


def test_workflow_doc_states_current_class_count_and_measured_fps():
    """现状说明文档不得把 13 类写成 7 类、把实测 5fps 写成 30fps。

    帧率这一条不是措辞问题：扫场速度 ≤0.4-0.5 m/s、每格 ≥2-3 帧冗余这些预算
    全是按 AR0234 USB2 YUY2 实测的 200ms/帧（5fps）推的（见
    docs/执行层深优化-20261016.md「相机帧率」行）。启动脚本请求的是
    1920x1080@30，读到"约 30 fps"的人会把整条时序预算算错 6 倍。
    """
    wf = DOCS / "无人机工作流程.md"
    assert wf.exists(), f"{wf} 不见了（改名了？改名后本测试要跟着走）"
    text = wf.read_text(encoding="utf-8", errors="replace")
    for stale in ("输出 7 类", "7 类检测框", "约 30 fps"):
        assert stale not in text, (
            f"无人机工作流程.md 仍写着 {stale!r}：检测是 13 类、相机实测约 5fps")
    assert re.search(r"\b5\s*fps\b", text), (
        "无人机工作流程.md 没有写出实测帧率；只删掉 30fps 的谎不算修好")
    assert "interface_contract.md" in text, (
        "无人机工作流程.md 没有把枚举真源指向 interface_contract.md——"
        "不指向就迟早有人再抄一份，然后再度错位")
