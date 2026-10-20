# -*- coding: utf-8 -*-
"""部署真源（车端 robot.yaml + dzrknn_dataprocess.cpp）跨参数一致性守卫。

为什么需要这个文件：`docs/成员交接/` 是**逐字保存**的别线工程（见
test_tools_smoke.py 的 _ARCHIVE_PREFIXES 说明），本仓不改它一个字。但里面那份
`deliverable_*/robot.yaml` 同时又是车端的**部署真源**——README 原话"含部署真源
三件套（cpp/.h/robot.yaml）"。于是出现一个缺口：真源里的参数是否自洽、是否和
配套审计工具对得上，**没有任何测试在守**，只能靠人肉在 76KB 的 YAML 里翻。

2026-10-19 深审就是这个缺口漏出来的：`predict_gain: 0.2` 这个值在全部交接文档里
**一个出处都找不到**（文档只给 0.0 或 0.5 两个分支），而 `lead_time_s: 0.27` 的
行内注释记的用户裁决是"先归零"、还自陈"链路真实延迟 0.5~0.9s，旧 0.35 写小一倍"
——0.27 既不等于 0，也比那个已被判为偏小的 0.35 更小。值与注释互相矛盾，且没有任何
机制会报出来。

本文件只**读**归档、不改归档，守两类东西：

  L1 提前量预算（物理不变量，应当恒绿）
     链路延迟期间靶在像面上已经走掉一段；dzbrain 的 lead_time_s 只补一部分，
     残差必须留在开火框里且不许吃掉一半以上。算出来才知道"提前量口径之争"
     到底值不值一个调参周期——见 test_lead_residual_fits_fire_zone 的实测数。

  L2 审计工具保真度（已知欠账，登记在 KNOWN_DEBT）
     `aim_param_audit.py` 自称 CODE_DEFAULT 是"代码里 declare_parameter 的真实
     默认值"。它是现场对表的基准，基准错了整张表就在骗人。用 cpp 逐键复核。

KNOWN_DEBT 与 test_tools_smoke.KNOWN_MISSING_TOOLS 同法双向守：欠账还清了必须
删条目（否则红），新增漂移也会红。不允许"登记完就再也不看"。
"""
from __future__ import annotations

import importlib.util
import re
from pathlib import Path

import pytest

import yaml_lite

REPO = Path(__file__).resolve().parent.parent
HANDOVER = REPO / "docs" / "成员交接"

# robot.yaml:819 行内注释自陈的"链路真实延迟 0.5~0.9s"。这是仓内唯一给出该量级
# 的地方，且出自部署当日的真源注释，比任何外部估计都可信。
LINK_LATENCY_S = (0.5, 0.9)

# L2 已知欠账：(审计表键, 欠账类型, 表内值, cpp 真值)。
#   default_mismatch = 表里写的默认值与 cpp declare_parameter 不符
#   phantom_key      = 表里有这个键，但部署 cpp 根本没 declare_parameter 它
# 证据：2026-10-19 逐键机械复核 92 个 declare_parameter，全表 30 键仅此 5 项不符。
KNOWN_DEBT = {
    # 危害最大的一条：cpp:1385 与 .h:302 的真实默认值是 10（2026-10-11 用户指令 4），
    # 表里写 3。审计工具的"代码默认"列因此把【原封不动躺在默认值上】的键显示成
    # "已被人从 3 调到 10"，正好把该键的分诊优先级判反——而它是全程 10 发弹药
    # 预算的唯一闸口（cpp:382 shots_on_target_ < max_shots_per_target_）。
    ("max_shots_per_target", "default_mismatch", 3, 10):
        "cpp:1385 declare_parameter(\"max_shots_per_target\", 10)；表内 3 是 2026-10-11 用户指令 4 之前的旧值",
    # 下面 4 个键在部署 cpp/.h 里 grep 无任何命中。px_per_tick / vision_latency_ms
    # 的 RECOMMEND 文案自己都写着"车端pid模式无此键"，却仍留在 CODE_DEFAULT 里
    # 冒充 cpp 默认值；momentum_* 两个则连 RECOMMEND 都没有，纯幽灵。
    ("px_per_tick", "phantom_key", 2.0, None):
        "部署 cpp/.h 无此 declare_parameter；RECOMMEND 文案自陈『车端(增量式PID)不存在此键』",
    ("vision_latency_ms", "phantom_key", 0.0, None):
        "部署 cpp/.h 无此 declare_parameter；因永不被 dump 到，审计表恒报 ★不符",
    ("momentum_comp_on", "phantom_key", True, None):
        "部署 cpp/.h 无此 declare_parameter，且 RECOMMEND 无此键 —— 纯幽灵条目",
    ("momentum_sign", "phantom_key", 1.0, None):
        "部署 cpp/.h 无此 declare_parameter，且 RECOMMEND 无此键 —— 与 momentum_comp_on 同为幽灵条目",
}


def _deploy_truth() -> tuple[Path, Path] | None:
    """取最新一份部署真源 (robot.yaml, dzrknn_dataprocess.cpp)。

    按目录名里的日期倒序挑，这样下次归档 20261020-* 时本文件自动跟上新真源，
    不用改测试。找不到就返回 None（归档不入库的浅克隆 ⇒ skip，不是 fail）。
    """
    yamls = sorted(HANDOVER.rglob("robot.yaml"), key=lambda p: str(p), reverse=True)
    for y in yamls:
        cpp = y.parent / "dzrknn_dataprocess.cpp"
        if cpp.exists():
            return y, cpp
    return None


def _cpp_defaults(cpp: Path) -> dict[str, str]:
    src = cpp.read_text(encoding="utf-8", errors="replace")
    return {k: v.strip() for k, v in
            re.findall(r'declare_parameter\(\s*"([A-Za-z0-9_]+)"\s*,\s*([^);]+?)\s*\)\s*;', src)}


def _as_num(v):
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return int(f) if f.is_integer() else f


def _norm(v):
    """归一到可比较标量：true/false -> bool，数字 -> int/float，其余原样字符串。

    必须先判 bool —— Python 里 bool 是 int 的子类，`_as_num(True)` 会走 float()
    拿到 1.0，于是 cpp 的 `true` 和表里的 `True` 比成 "1.0 vs None" 假漂移
    （fire_only_blue / fire_use_image_stable / use_external_fire_gate 三个键
    第一版就是这么被误报的）。
    """
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)):
        return v
    s = str(v).strip()
    if s.lower() in ("true", "false"):
        return s.lower() == "true"
    n = _as_num(s)
    return n if n is not None else s


def _same(claimed, real) -> bool:
    a, b = _norm(claimed), _norm(real)
    if isinstance(a, bool) or isinstance(b, bool):
        return isinstance(a, bool) and isinstance(b, bool) and a == b
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return abs(float(a) - float(b)) < 1e-9
    return str(a).strip().lower() == str(b).strip().lower()


def _aim_section(nodes: dict) -> dict:
    """取 /dzbrain 的 aim 段，兼容 yaml_lite 的两条解析路径。

    PyYAML 在时 `aim` 是嵌套 dict；缺失时 yaml_lite 走 _fallback，它是**扁平**的：
    `    aim:` 这行会被存成空字符串，而 `      lead_time_s: 0.27` 被提到节点层。
    两种形状都要认，否则没装 PyYAML 的 CI 上 L1 会静默 skip 成摆设。
    """
    dz = nodes.get("/dzbrain") or {}
    aim = dz.get("aim")
    if isinstance(aim, dict) and aim:
        return aim
    return {k: v for k, v in dz.items() if k != "aim"}


def _load_audit_table() -> dict | None:
    """按 test_tools_smoke 的加载姿势导入归档里的审计脚本（只读取其常量表）。"""
    cands = sorted(HANDOVER.rglob("aim_param_audit.py"), key=lambda p: str(p), reverse=True)
    if not cands:
        return None
    spec = importlib.util.spec_from_file_location("_archived_aim_param_audit", cands[0])
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.CODE_DEFAULT


@pytest.fixture(scope="module")
def truth():
    found = _deploy_truth()
    if found is None:
        pytest.skip("docs/成员交接 下未找到部署真源 robot.yaml + dzrknn_dataprocess.cpp")
    return found


# ---------------------------------------------------------------------------
# L1 提前量预算：残差必须留在开火框内
# ---------------------------------------------------------------------------

def test_lead_residual_fits_fire_zone(truth):
    """链路延迟残差 ≤ 开火框的一半——否则"打不中"就该先查提前量。

    实测口径（2026-10-19 部署真源）：靶像速 = 0.04 m/s × 140 px/m = 5.60 px/s；
    dzbrain lead_time_s 0.27 只补 1.51 px；按注释自陈的真实延迟 0.5~0.9s，
    残差 1.29~3.53 px，而开火框 far 端 18 px ⇒ 残差只吃掉 7%~20%，余量 5.1×。

    这条**当前是绿的**，绿本身就是结论：`predict_gain` 该填 0.0 还是 0.5、
    `lead_time_s` 该不该归零，对 4cm/s 轨道靶的命中率影响都在框内噪声量级，
    不值得一个调参周期。交接文档自己指认的主项是 laser_offset（发10/中0 的
    隐藏根因，至今为 0 未标）。把它写成断言，是为了在有人把开火框收窄、
    或换更快的靶时，这个"无关紧要"的前提失效时能立刻红。
    """
    y, _ = truth
    nodes = yaml_lite.load_params(y)
    aim = _aim_section(nodes)
    gimbal = nodes.get("/dzrknn_dataprocess") or {}

    speed = aim.get("target_speed_mps")
    px_per_m = aim.get("px_per_m_rail")
    lead_s = aim.get("lead_time_s") or 0.0
    fire_far = gimbal.get("fire_dz_far_px")
    if speed is None or px_per_m is None or fire_far is None:
        pytest.skip(f"{y.name} 缺 aim.target_speed_mps / aim.px_per_m_rail / fire_dz_far_px")

    px_per_s = float(speed) * float(px_per_m)
    # lead_time_s 为负 = 反向补偿（瞄到靶的**身后**去），无论量级多小都是配置错误，
    # 不能被下面的残差钳位吸收掉。第一版就是 max(resid, 0) 把 -0.5 放过去了。
    assert float(lead_s) >= 0.0, (
        f"aim.lead_time_s = {lead_s} 为负 ⇒ 提前量反向补偿（云台瞄向靶身后的位置）；"
        f"符号写反了，不是调参问题"
    )
    worst_resid = max((lat - float(lead_s)) * px_per_s for lat in LINK_LATENCY_S)
    budget = float(fire_far) * 0.5

    assert worst_resid <= budget, (
        f"链路延迟残差 {worst_resid:.2f}px 已吃掉开火框({fire_far}px)的一半以上"
        f"（靶像速 {px_per_s:.2f}px/s，lead_time_s={lead_s} 只补 {float(lead_s)*px_per_s:.2f}px，"
        f"真实延迟 {LINK_LATENCY_S[0]}~{LINK_LATENCY_S[1]}s）——"
        f"此时命中率问题必须先查提前量口径，不能再当框内噪声"
    )


def test_lead_time_s_covers_documented_latency(truth):
    """lead_time_s 与真源注释自陈的真实延迟对表——差太多就必须留痕。

    robot.yaml:819 的注释同时说了三件互相打架的事：用户裁决"先归零"、
    "链路真实延迟 0.5~0.9s"、"旧 0.35 写小一倍致 pred 系统性偏移"；而值是 0.27。
    0.27 既不是 0（裁决），也比已被判偏小的 0.35 更小。这不是本测试能裁决的
    （裁决要有落点分布数据），但**覆盖率**必须显式可见：0.27/0.5~0.9 = 30%~54%。
    """
    y, _ = truth
    aim = _aim_section(yaml_lite.load_params(y))
    lead_s = aim.get("lead_time_s")
    if lead_s is None:
        pytest.skip(f"{y.name} 无 aim.lead_time_s")
    cover = [float(lead_s) / lat for lat in LINK_LATENCY_S]
    # 覆盖率允许低（残差有 L1 兜着），但不允许为负、也不允许超过 1.5×（过冲）
    assert all(-0.01 <= c <= 1.5 for c in cover), (
        f"lead_time_s={lead_s} 对真实延迟 {LINK_LATENCY_S} 的覆盖率 {cover[0]:.0%}~{cover[1]:.0%}"
        f" 越界（<0 = 反向补偿，>150% = 过冲）"
    )


# ---------------------------------------------------------------------------
# L2 审计工具保真度
# ---------------------------------------------------------------------------

def test_audit_code_default_matches_deployed_cpp(truth):
    """aim_param_audit.CODE_DEFAULT 必须与部署 cpp 的 declare_parameter 逐键一致。

    审计工具的用途是"把车上真实参数与建议值对表，直接指出哪些没改"，而现场
    判断"这个键有没有被人动过"靠的就是它的『代码默认』列。该列说谎 ⇒ 分诊判反。
    """
    _, cpp = truth
    table = _load_audit_table()
    if table is None:
        pytest.skip("归档内未找到 aim_param_audit.py")
    real = _cpp_defaults(cpp)
    assert real, f"{cpp.name} 里没解析到任何 declare_parameter，解析口径失效了"

    drift = {}
    for key, claimed in table.items():
        if key not in real:
            drift[(key, "phantom_key", claimed, None)] = None
            continue
        if not _same(claimed, real[key]):
            drift[(key, "default_mismatch", claimed, _norm(real[key]))] = None

    unregistered = set(drift) - set(KNOWN_DEBT)
    cleared = set(KNOWN_DEBT) - set(drift)
    assert not unregistered, (
        "审计表 CODE_DEFAULT 出现新的未登记漂移（键, 类型, 表内值, cpp 真值）：\n  "
        + "\n  ".join(f"{d}  ← {real.get(d[0], '(cpp 无此键)')}" for d in sorted(unregistered, key=str))
    )
    assert not cleared, (
        "以下欠账已经还清，请从 KNOWN_DEBT 删掉对应条目（双向守，不许登记完就不看）：\n  "
        + "\n  ".join(str(c) for c in sorted(cleared, key=str))
    )


def test_known_debt_entries_are_evidenced():
    """KNOWN_DEBT 每条都得带证据串，且格式与 _same 的复核口径一致（防登记表自己腐化）。"""
    for key, why in KNOWN_DEBT.items():
        assert isinstance(key, tuple) and len(key) == 4, f"欠账键必须是 4 元组：{key!r}"
        name, kind, claimed, real = key
        assert kind in ("default_mismatch", "phantom_key"), f"未知欠账类型 {kind!r}（{name}）"
        if kind == "phantom_key":
            assert real is None, f"{name} 是幽灵键就不该有 cpp 真值，实为 {real!r}"
        assert isinstance(why, str) and len(why) >= 20, f"{name} 的证据串太短：{why!r}"
