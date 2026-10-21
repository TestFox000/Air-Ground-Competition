"""tools/orin_deploy_gate.py 的测试：闸门必须既拦得住坏树，也放得过好树。

闸门存在的理由是一次实机事故：仓外那份 fix_live_hud.py --deploy 用它硬编码的
另一个智能体工作副本（kimi 树）里的 main.cpp 覆盖了机上同名文件，界面毫无异常，
复盘时才发现机上跑的是回退后的旧逻辑。那份脚本 2026-10-21 加了「汇流硬门」，
但门只看 jpegdec / DZUAV_VIEW 两个能力标记 + 一个已知坏 md5，**本闸门的 11 条
锚点一条都不在其中**——所以两道门验的不是一回事，谁也不能替谁。

所以这里的测试重点不是"闸门能跑"，而是**每一条锚点都真的在拦**——
一条失效的检查比没有检查更坏：它让人以为树被验过了。

test_gate_flags_every_removed_anchor 就是为此参数化遍历 CHECKS：
逐条把锚点从源码里抠掉，确认闸门点名那条检查失败。哪天有人加了条永远绿的
检查（锚点写错、文件名写错），这条测试会当场红。
"""
from __future__ import annotations

import hashlib
import importlib.util
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent
GATE_PY = REPO / "tools" / "orin_deploy_gate.py"


def _load_gate():
    """按文件路径加载闸门模块。

    必须先登记进 sys.modules 再 exec：闸门用了 `from __future__ import
    annotations` + @dataclass，dataclasses._is_type 会回查
    sys.modules.get(cls.__module__).__dict__，没登记就拿到 None -> AttributeError
    （tests/test_tools_smoke.py 的 _PROBE 里记着同一个坑）。
    """
    spec = importlib.util.spec_from_file_location("_orin_deploy_gate_under_test", GATE_PY)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    try:
        spec.loader.exec_module(mod)
    finally:
        sys.modules.pop(spec.name, None)
    return mod


gate = _load_gate()


@pytest.fixture()
def mini_root(tmp_path: Path) -> Path:
    """搭一棵"只含闸门会读的东西"的最小树：机载源码 + orin shell 工具。

    不整仓拷贝是因为闸门的密码扫描会 rglob 整个根：拿真仓当变异靶子，
    docs/ 里既有的三处明文密码会混进每次断言，分不清是变异生效还是背景噪声。
    """
    shutil.copytree(REPO / "onboard" / "dzuav", tmp_path / "onboard" / "dzuav",
                    ignore=shutil.ignore_patterns("build", "__pycache__"))
    (tmp_path / "tools").mkdir()
    for sh in sorted((REPO / "tools").glob("orin_*.sh")):
        # 逐字节拷（不是 write_text）：*.sh 属 eol=lf，write_text 在 Windows 上
        # 会把 \n 翻成 \r\n，于是"好树"本身就踩了 sh-lf 检查，变异测试全成假绿。
        (tmp_path / "tools" / sh.name).write_bytes(sh.read_bytes())
    return tmp_path


def test_minimal_copy_is_itself_green(mini_root: Path):
    """变异测试的前提：靶子树未变异时必须全绿。

    这条不成立的话，后面每条"闸门拦住了变异"都会因为**别的原因**通过——
    例如拷贝时把 CRLF 带进去，sh-lf 就恒定失败，那 test_..._crlf 永远绿。
    """
    assert gate.main(["--root", str(mini_root)]) == 0


def test_gate_is_green_on_the_real_repo():
    """仓库自身必须过闸门，否则 orin_deploy.py 第 0 步会恒定拒绝部署。"""
    assert gate.main(["--root", str(REPO)]) == 0


# ------------------------------------------------------------------ 锚点逐条
def _first_needle(check) -> str:
    return check.needles[0].text


@pytest.mark.parametrize("cid", [c.cid for c in gate.CHECKS])
def test_gate_flags_every_removed_anchor(mini_root: Path, cid: str):
    """逐条抠掉锚点，闸门必须点名那条检查。

    替换而非删除整行：锚点可能是一行的片段（如 `spec != canon`），删行会连带
    破坏别的东西，失败就说不清是哪条检查起的作用。断言用"cid 在失败集合里"
    而不是"只失败这一条"——同一个文件被多条检查共用，抠掉一处可能同时命中两条，
    那是正确行为。

    **全部**出现都要替换掉，不能只替换第一处：count>1 的锚点在源文件里的实际
    出现次数往往高于阈值（kMaxFrameErrors 实到 4 次、阈值 3），只抠一处还剩 3 次
    仍达标，这条测试就会因为"闸门没报错"而红——首版正是这么失败的。
    """
    check = next(c for c in gate.CHECKS if c.cid == cid)
    p = mini_root / check.relpath
    text = p.read_text(encoding="utf-8")
    needle = _first_needle(check)
    assert needle in text, f"{cid} 的锚点在 {check.relpath} 里找不到，测试已失效"
    p.write_text(text.replace(needle, "/*anchor-removed*/"), encoding="utf-8",
                 newline="")
    assert needle not in p.read_text(encoding="utf-8"), "替换没生效"

    results = gate.run_checks(mini_root)
    failed = {r["id"] for r in results if not r["ok"]}
    assert cid in failed, f"抠掉 {cid} 的锚点后闸门没报它——这条检查是死的"
    # 失败必须带上"后果"文案：现场看到红灯时，那句 why 是唯一能告诉他
    # "为什么这条不能放过"的东西。
    hit = next(r for r in results if r["id"] == cid)
    assert hit["why"].strip(), f"{cid} 失败时没有给出后果说明"
    assert needle in hit["missing"], f"{cid} 的 missing 没点名被抠掉的锚点"


def test_gate_counts_repeated_anchors(mini_root: Path):
    """count>1 的锚点要按次数判：少一次就算缺。

    reopen-release-first 要求 `capture.release();` 至少两处（重开前 + 失败后），
    frame-errors-shared 要求 `frame_errors >= kMaxFrameErrors` 两处（相机分支 +
    离线分支）。写成 count=1 的话，删掉任意一处都还是绿的——那正是这两个 bug
    的形状。

    必须削到**恰好 count-1** 次，不能只削一次：源文件里的实际出现次数往往
    比阈值多，只削一次可能仍达标，于是"闸门会数次数"这件事根本没被验到
    （首版就是这么假绿的）。
    """
    repeated = [c for c in gate.CHECKS if any(n.count > 1 for n in c.needles)]
    assert repeated, "CHECKS 里没有 count>1 的锚点了，本测试失效"
    for check in repeated:
        needle = next(n for n in check.needles if n.count > 1)
        p = mini_root / check.relpath
        original = p.read_bytes()
        text = original.decode("utf-8")
        assert text.count(needle.text) >= needle.count, f"{check.cid} 前提不成立"

        # 逐个替换，直到只剩 needle.count - 1 处
        mutated = text
        while mutated.count(needle.text) >= needle.count:
            mutated = mutated.replace(needle.text, "/*one-less*/", 1)
        assert mutated.count(needle.text) == needle.count - 1
        p.write_text(mutated, encoding="utf-8", newline="")

        results = gate.run_checks(mini_root)
        assert check.cid in {r["id"] for r in results if not r["ok"]}, (
            f"{check.cid}: 锚点削到 {needle.count - 1} 次（阈值 {needle.count}）"
            f"却没被发现——count 没起作用")
        p.write_bytes(original)   # 还原，供下一轮循环从完整状态起


# 阈值本身也是判据。上一条测试是从 needle.count 反推目标的，所以它验的是
# "计数机制在工作"，验不到"阈值定得对"——把 3 改成 1，机制照样运转、那条测试
# 照样绿（变异测试实测过）。这里把阈值钉死。
PINNED_COUNTS = {
    # 重开前一次 + 重开失败后一次；少任何一处都会留下半开的管道。
    "reopen-release-first": ("capture.release();", 2),
    # 相机分支一次 + 离线分支一次。这正是"两处共用同一常量"的判据：
    # 盯常量名本身数不到这件事（源文件里该名字出现 4 次，含声明与日志文案）。
    "frame-errors-shared": ("frame_errors >= kMaxFrameErrors", 2),
}


def test_repeated_anchor_thresholds_are_pinned():
    for cid, (needle, want) in PINNED_COUNTS.items():
        check = next((c for c in gate.CHECKS if c.cid == cid), None)
        assert check, f"CHECKS 里没有 {cid} 了，这条约束被整条删掉了"
        got = next((n for n in check.needles if n.text == needle), None)
        assert got, f"{cid} 不再盯 {needle!r}（锚点换了？请同步 PINNED_COUNTS）"
        assert got.count == want, (
            f"{cid} 的阈值被改成 {got.count}（应为 {want}）——降到 1 就等于放弃"
            f"『多处必须同时存在』，两个分支各写各的字面量也查不出来")
        actual = (REPO / check.relpath).read_text(encoding="utf-8").count(needle)
        assert actual >= want, (
            f"{check.relpath} 里 {needle!r} 实到 {actual} 处，不足阈值 {want}"
            f"——要么源码回退了，要么阈值定高了")


# ------------------------------------------------------------------ 结构性检查
def test_gate_flags_missing_source_file(mini_root: Path):
    """少一个源文件 = 机上 cmake 直接失败，必须在本地就拦下。"""
    victim = mini_root / gate.MAIN
    victim.unlink()
    rc = gate.main(["--root", str(mini_root)])
    assert rc == 1
    present = gate.check_dzuav_sources_present(mini_root)
    assert not present["ok"]
    assert gate.MAIN.as_posix() in present["missing"]


def test_gate_flags_crlf_shell_script(mini_root: Path):
    """CRLF 的 .sh 上机就是 /bin/bash^M: bad interpreter。

    SFTP 是逐字节拷贝，Windows 上被 autocrlf 转过的脚本推上去必炸，
    而炸的是机上**第一条**指令，操作手往往以为是部署本身失败了。
    """
    target = next(iter(sorted((mini_root / "tools").glob("orin_*.sh"))))
    target.write_bytes(target.read_bytes().replace(b"\n", b"\r\n"))
    res = gate.check_shell_line_endings(mini_root)
    assert not res["ok"]
    assert target.name in res["missing"]
    assert gate.main(["--root", str(mini_root)]) == 1


def test_object_class_count_rejects_12_and_14(mini_root: Path):
    """枚举成员数与取值连续性都要管：少一类/多一类，三处名表一起错位。"""
    types_hpp = mini_root / gate.TYPES
    text = types_hpp.read_text(encoding="utf-8")

    # 少一类（删掉 UavEnd = 12）
    types_hpp.write_text(text.replace("UavEnd = 12", "/*gone*/", 1),
                         encoding="utf-8", newline="")
    assert not gate.check_object_class_count(mini_root)["ok"]

    # 多一类（13 类之后再加一个）
    types_hpp.write_text(text.replace("UavEnd = 12", "UavEnd = 12,\n    Extra = 13", 1),
                         encoding="utf-8", newline="")
    res = gate.check_object_class_count(mini_root)
    assert not res["ok"], "加到 14 类还能过 = 只数了上限没数下限"

    # 取值跳号（成员数仍是 13，但 5 号缺失）
    types_hpp.write_text(text.replace("Obstacle = 5", "Obstacle = 6", 1),
                         encoding="utf-8", newline="")
    res = gate.check_object_class_count(mini_root)
    assert not res["ok"], "取值不连续也能过 = 只数了个数没看值"


def test_object_class_count_accepts_the_real_enum():
    assert gate.check_object_class_count(REPO)["ok"]


# ------------------------------------------------------------------ 密码扫描
# 拼装而不是字面写：闸门会扫本文件（tests/ 不在载荷内 -> 只警告），
# 但测试源码里留一条真的赋值样例毫无必要，还会污染仓库自己的闸门输出。
_VAR = "AG_" + "ORIN_PASS"


def test_gate_flags_plaintext_password_inside_payload(mini_root: Path):
    """载荷内的明文密码必须判失败：它会随 sftp 落到无人机上、随 git 进历史。

    这条打的是 tools/orin_*.sh（载荷的另一个类），.cpp 那一类由
    test_cpp_payload_is_actually_scanned 单独钉住。
    """
    victim = next(iter(sorted((mini_root / "tools").glob("orin_*.sh"))))
    victim.write_text(
        victim.read_text(encoding="utf-8") + f"\n# {_VAR}=hunter2secret\n",
        encoding="utf-8", newline="")
    res = gate.check_no_plaintext_password(mini_root)
    assert not res["ok"]
    assert res["missing"], "载荷内命中却没进 missing"
    assert any(f"tools/{victim.name}" in m for m in res["missing"])
    assert gate.main(["--root", str(mini_root)]) == 1


def test_gate_warns_but_passes_for_password_outside_payload(mini_root: Path):
    """载荷外的命中降级成警告、不拦部署。

    这是刻意的：仓库里既有的 docs 明文密码不该让"部署一份修好的机载代码"
    永远红。一道天天红的门，现场第三天就没人看了，比没有门更危险。
    """
    docs = mini_root / "docs"
    docs.mkdir()
    (docs / "handover.md").write_text(f"export {_VAR}=hunter2secret\n", encoding="utf-8")
    res = gate.check_no_plaintext_password(mini_root)
    assert res["ok"], "载荷外的密码不该判失败"
    assert res["warn"], "载荷外的命中必须至少警告一次，静默吞掉等于默许"
    assert gate.main(["--root", str(mini_root)]) == 0


def test_gate_treats_placeholder_as_ok(mini_root: Path):
    """占位符形态是正确用法，不能判成泄漏（首版 14 处误报就是这么来的）。

    这些字面量逐条取自本仓真实出现的写法（tools/*.py 的 docstring、
    docs/无人机工作流程.md、成员交接里的 benchctl.py），不是凭空编的：
    闸门要放过的正是这些，编一套理想形态来自我安慰没有意义。
    """
    docs = mini_root / "docs"
    docs.mkdir()
    placeholders = [
        "******",                       # tools/check_stack_health.py:10
        "******（或 --password）",        # tools/check_stack_health.py:58，全角括号紧跟
        "<车端密码>",                     # tools/car_deploy.py:12
        "'<密码>'",                      # dznav_deploy.py:349 的引号包裹形态
        "'******'",                     # 成员交接 benchctl.py:12
        "...",                          # dznav_deploy.py 的省略号形态
        "",                             # 只有变量名没有值
    ]
    (docs / "usage.md").write_text(
        "".join(f"set {_VAR}={v}\n" for v in placeholders), encoding="utf-8")
    res = gate.check_no_plaintext_password(mini_root)
    assert res["ok"]
    assert not res["warn"], f"这些占位符被误报成泄漏: {res['warn']}"


def test_gate_catches_quoted_powershell_form(mini_root: Path):
    """引号包裹的真值必须抓到。

    本仓唯一一处真泄漏就是这个写法（docs/接手说明.md:19 的 AG_CAR_PASS='...'）；
    把引号当分词边界的话取到的值是空串，当场漏检——所以这条单独钉住。
    """
    docs = mini_root / "docs"
    docs.mkdir()
    (docs / "ps.md").write_text(f"$env:{_VAR} = 'hunter2secret'\n", encoding="utf-8")
    res = gate.check_no_plaintext_password(mini_root)
    assert res["warn"] or not res["ok"], "引号形态的真值被漏检了"


def _looks_like_text(p: Path) -> bool:
    """独立判定"是文本"：不用闸门的白名单，否则这条测试就是自证清白。"""
    try:
        head = p.read_bytes()[:8192]
    except OSError:
        return False
    if b"\x00" in head:
        return False
    try:
        head.decode("utf-8")
    except UnicodeDecodeError:
        return False
    return True


def test_payload_source_suffixes_are_all_scanned():
    """载荷里的**每一个文本文件**后缀都必须在扫描白名单内。

    这是首版真出过的两个洞：
      * 白名单只列了 .py/.sh/.md/.txt，漏了 .cpp/.hpp —— 于是
        onboard/dzuav/src/*.cpp（部署载荷里最要命的那部分）一行都没被扫，
        报告照样打 [通过]；
      * 无后缀文件一律不扫 —— models/release/current_hbb_detector 这类指针
        文件是纯文本、也确实会被 resolve_current_models 推上机。
    "扫了但没扫到"比不扫更坏，因为它给出了假保证。

    文本与否用 _looks_like_text 独立判定（无 NUL 且能 UTF-8 解码），
    这样 .onnx/.engine 自然落在外面，而判定标准不来自被测方。
    """
    scanned = gate._SECRET_SCAN_SUFFIX
    payload_dirs = [REPO / "onboard" / "dzuav", REPO / "src" / "ag_integrate",
                    REPO / "models" / "release"]
    uncovered: dict[str, list[str]] = {}
    seen_text = 0
    for d in payload_dirs:
        assert d.is_dir(), f"载荷目录不存在: {d}"
        for p in d.rglob("*"):
            if not p.is_file() or any(x in p.parts for x in gate._SECRET_SCAN_SKIP):
                continue
            if not _looks_like_text(p):
                continue
            seen_text += 1
            if p.suffix.lower() not in scanned:
                uncovered.setdefault(p.suffix.lower(), []).append(
                    p.relative_to(REPO).as_posix())
    for sh in (REPO / "tools").glob("orin_*.sh"):
        seen_text += 1
        assert sh.suffix.lower() in scanned

    assert seen_text > 10, f"只扫到 {seen_text} 个文本文件，遍历姿势错了"
    assert not uncovered, (
        f"这些载荷文本文件的后缀不在密码扫描白名单里，等于免检: "
        f"{ {s: v[:3] for s, v in uncovered.items()} }")


def test_cpp_payload_is_actually_scanned(mini_root: Path):
    """上一条的端到端版：往 .cpp 里塞口令，闸门必须判失败。"""
    victim = mini_root / gate.MAIN
    victim.write_text(
        victim.read_text(encoding="utf-8") + f"\n// {_VAR}=hunter2secret\n",
        encoding="utf-8", newline="")
    res = gate.check_no_plaintext_password(mini_root)
    assert not res["ok"], ".cpp 里的明文口令没被抓到（后缀白名单又漏了源码？）"
    assert any(gate.MAIN.as_posix() in m for m in res["missing"])


def test_extensionless_pointer_file_is_scanned(mini_root: Path):
    """无后缀的模型指针文件也在载荷里（resolve_current_models 会推它）。

    按后缀白名单一刀切时它落在外面，等于免检。
    """
    pointer = mini_root / "models" / "release" / "current_hbb_detector"
    pointer.parent.mkdir(parents=True)
    pointer.write_text(f"hbb_d13.onnx\n# {_VAR}=hunter2secret\n", encoding="utf-8")
    res = gate.check_no_plaintext_password(mini_root)
    assert not res["ok"], "无后缀的载荷文件没被扫"
    assert any("models/release/current_hbb_detector" in m for m in res["missing"])


def test_secret_value_is_never_printed(mini_root: Path, capsys):
    """报告里只许出现长度，不许出现密码本身。

    闸门输出会被 tee 进现场日志、贴进会话纪要；把真值打出来等于二次泄漏。
    """
    secret = "hunter2secret"
    docs = mini_root / "docs"
    docs.mkdir()
    (docs / "leak.md").write_text(f"{_VAR}={secret}\n", encoding="utf-8")
    gate.main(["--root", str(mini_root)])
    out = capsys.readouterr().out
    assert secret not in out, "闸门把密码原文打进了报告"
    assert f"<字面值 {len(secret)} 字符>" in out, "报告应给出长度而非原文"


# ------------------------------------------------------------------ 输出与自洽
def test_json_mode_is_machine_readable(mini_root: Path, capsys):
    assert gate.main(["--root", str(mini_root), "--json"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["pass"] is True
    ids = {r["id"] for r in payload["results"]}
    assert {"sources-present", "classes-count", "sh-lf",
            "no-plaintext-password"} <= ids
    assert {c.cid for c in gate.CHECKS} <= ids
    assert len(payload["md5"]) == len(gate.DZUAV_SOURCES)


def test_manifest_is_lf_and_covers_every_source(mini_root: Path, tmp_path: Path):
    """清单必须 LF、且覆盖全部源文件。

    *.txt 属 .gitattributes 的 eol=lf；用 write_text 会在 Windows 上写成 CRLF，
    于是每归档一份现场证据，工作区就永久显示"脏"（remote_run.py 踩过两次）。
    """
    out = tmp_path / "manifest.txt"
    assert gate.main(["--root", str(mini_root), "--manifest", str(out)]) == 0
    raw = out.read_bytes()
    assert b"\r\n" not in raw, "清单被写成 CRLF，归档后工作区会一直脏"
    lines = raw.decode("utf-8").strip().split("\n")
    assert len(lines) == len(gate.DZUAV_SOURCES)
    listed = {ln.split("  ", 1)[1] for ln in lines}
    assert listed == {r.as_posix() for r in gate.DZUAV_SOURCES}
    assert all(len(ln.split("  ", 1)[0]) == 32 for ln in lines), "md5 长度不对"


# 部署载荷里**每一个文本后缀**都必须在 .gitattributes 里被 eol=lf 锁住。
# 首版只锁了 *.sh/*.py/*.md/*.txt 等，漏了 *.cpp/*.hpp —— 而 onboard/dzuav/**
# 恰恰是 orin_deploy.py 逐字节 sftp 上机、在 Jetson 上编译的那部分，完全落在
# .gitattributes 自己写的「凡是要上车跑的」这句的范围里。
#
# 后果不是编译失败（g++ 吃得下 CRLF），是**上面那份 md5 清单失去可比性**：
# 清单取本机工作区字节，Windows 检出成 CRLF 后与 Linux 侧同一份源码的 md5 不同
# （实测 main.cpp：CRLF 4e2b679b… / LF fa4e2cca…）。现场看到"md5 不一致"
# 会先怀疑部署坏了，而其实只是行尾——在最不该浪费时间的时刻浪费半小时。
_BINARY_SUFFIX = {".onnx", ".engine", ".trt", ".pt", ".rknn", ".png", ".jpg",
                  ".jpeg", ".zip", ".pdf", ".so", ".a", ".bin", ".mp4", ".avi"}
# orin_deploy.py 实际会推上机的目录（与脚本里的 put(...) 调用一一对应）
_PAYLOAD_DIRS = ("onboard/dzuav", "src/ag_integrate")


def _payload_text_suffixes() -> set[str]:
    out: set[str] = set()
    for d in _PAYLOAD_DIRS:
        base = REPO / d
        assert base.is_dir(), f"{d} 不在仓里——部署载荷口径变了，请同步本测试"
        for p in base.rglob("*"):
            if not p.is_file() or any(s in p.parts for s in ("build", "__pycache__")):
                continue
            s = p.suffix.lower()
            if s and s not in _BINARY_SUFFIX:
                out.add(s)
    for p in (REPO / "tools").glob("orin_*"):
        if p.is_file() and p.suffix.lower() not in _BINARY_SUFFIX:
            out.add(p.suffix.lower())
    return out


def test_every_deployed_text_suffix_is_lf_locked():
    """部署载荷里的文本后缀必须全部 eol=lf，否则 md5 对账会在现场骗人。"""
    attr = (REPO / ".gitattributes").read_text(encoding="utf-8")
    locked = set(re.findall(r"^\*(\.\w+)\s+text\s+eol=lf", attr, re.M))
    assert locked, ".gitattributes 里一条 eol=lf 都没解析到，正则坏了"
    unlocked = sorted(_payload_text_suffixes() - locked)
    assert not unlocked, (
        f"这些后缀在部署载荷里却没被 eol=lf 锁住: {unlocked}\n"
        "  它们会随 core.autocrlf=true 在 Windows 检出成 CRLF，被 orin_deploy.py\n"
        "  逐字节推上机，于是 orin_deploy_gate.py 的 md5 清单与 Linux 侧同一份源码\n"
        "  对不上（同一文件 CRLF/LF 两个 md5）。修法：在 .gitattributes 补一行\n"
        "  `*<后缀>  text eol=lf`——入库那份本来就是 LF，补了是零 blob 改动。")


def test_crlf_and_lf_give_different_md5_so_the_lock_matters():
    """反向确认上面那条测试不是无病呻吟：CRLF 与 LF 的 md5 真的不同。

    少了这条，将来有人会说"行尾而已，md5 工具会归一化吧"然后把它删掉。
    """
    src = REPO / "onboard" / "dzuav" / "src" / "main.cpp"
    assert src.is_file(), "main.cpp 不在仓里，本测试的前提没了"
    blob = src.read_bytes()
    crlf = blob.replace(b"\r\n", b"\n").replace(b"\n", b"\r\n")
    lf = blob.replace(b"\r\n", b"\n")
    assert hashlib.md5(crlf).hexdigest() != hashlib.md5(lf).hexdigest(), (
        "CRLF 与 LF 的 md5 竟然相同——那上面那条 eol=lf 交叉锁确实没必要，可以删")


def test_manifest_records_absent_file_as_placeholder(tmp_path: Path):
    """文件不在时用占位符而不是崩掉：闸门要在残缺树上也能出报告。"""
    manifest = gate.md5_manifest(tmp_path)
    assert len(manifest) == len(gate.DZUAV_SOURCES)
    assert all(h == "-" * 32 for h, _ in manifest)


def test_critical_five_are_all_deployed_sources():
    """CRITICAL_FIVE 必须都在 DZUAV_SOURCES 里，否则 md5 清单上的星标
    永远打不出来，重点盯防等于没盯。"""
    names = {r.name for r in gate.DZUAV_SOURCES}
    for n in gate.CRITICAL_FIVE:
        assert n in names, f"{n} 不在部署源名单里，星号标注失效"
    assert len(gate.CRITICAL_FIVE) == 5


def test_manifest_star_only_claims_overwrite_for_main_cpp(capsys):
    """md5 清单的星标不许说大话：只有 main.cpp 会被 fix_live_hud.py 覆盖。

    事实核对（2026-10-21 读了那份脚本本体）：--deploy 全流程只有一处
    `sftp.put(PATCHED, .../main.cpp)`；SRC_FILES 那五个名字是拿去做**机上齐备性
    点数**（`ls | wc -l` 必须 == 5）的，另四个文件它根本不写。
    首版给五个都打上"会覆盖"，操作手翻一眼脚本就能看出这句是假的——
    一份被抓到说过一次假话的清单，后面每一行都会被打折看。
    """
    rc = gate.main(["--root", str(REPO)])
    assert rc == 0, "本仓自己应该全绿，否则这条测试验的是坏树"
    out = capsys.readouterr().out
    assert "机载源文件 md5" in out, "md5 清单一段都没打出来"
    starred = [l for l in out.splitlines() if "←" in l]
    assert len(starred) == 5, f"应恰好 5 条星标（CRITICAL_FIVE），实到 {len(starred)}"

    overwrite = [l for l in starred if "会覆盖" in l]
    assert len(overwrite) == 1 and "src/main.cpp" in overwrite[0], (
        f"只有 src/main.cpp 该被标成会被覆盖，实到: {overwrite}")
    dense = [l for l in starred if "会覆盖" not in l]
    assert len(dense) == 4, f"另四个该标成'修复密集，对账重点'，实到 {len(dense)}: {dense}"
    assert all("对账重点" in l for l in dense), dense


def test_checks_reference_only_deployed_files():
    """每条检查盯的文件都必须在部署名单里。

    盯一个不上机的文件 = 机上那份永远不会被这条检查覆盖，是纯装饰。
    """
    deployed = {r.as_posix() for r in gate.DZUAV_SOURCES}
    for c in gate.CHECKS:
        assert c.relpath.as_posix() in deployed, (
            f"{c.cid} 盯的 {c.relpath} 不在 DZUAV_SOURCES 里")


def test_gate_is_stdlib_only():
    """闸门必须在 Orin 的系统 python3 上直接跑（机上不装第三方包）。

    orin_deploy.py 的收尾提示让人在机上再跑一次做对账；一旦它 import 了
    paramiko/yaml 之类，那条提示就是假的。
    """
    src = GATE_PY.read_text(encoding="utf-8")
    allowed = {"argparse", "hashlib", "json", "re", "sys", "dataclasses", "pathlib",
               "__future__"}
    imported = {m.group(1).split(".")[0]
                for m in re.finditer(r"^\s*(?:import|from)\s+([A-Za-z_][\w.]*)", src, re.M)}
    extra = imported - allowed
    assert not extra, f"闸门引入了非标准库依赖 {extra}，机上 python3 跑不起来"


def test_gate_runs_as_a_script():
    """按 `python3 tools/orin_deploy_gate.py` 的真实调用方式跑一遍。

    模块内加载通过不代表脚本入口通过（__main__ 守卫、SystemExit 传递都可能坏）。
    """
    proc = subprocess.run(
        [sys.executable, str(GATE_PY), "--root", str(REPO)],
        capture_output=True, text=True, encoding="utf-8", errors="replace")
    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert "可以部署" in proc.stdout


def test_gate_rejects_nonexistent_root(tmp_path: Path):
    proc = subprocess.run(
        [sys.executable, str(GATE_PY), "--root", str(tmp_path / "nope")],
        capture_output=True, text=True, encoding="utf-8", errors="replace")
    assert proc.returncode == 1
    assert "目录不存在" in (proc.stdout + proc.stderr)
