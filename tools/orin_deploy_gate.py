"""上机前的源码完整性闸门：确认即将推上无人机的这份源码**带着全部已知修复**。

为什么需要它（2026-10-20 查出的真实事故形态）：仓外那份 fix_live_hud.py 把 REPO
硬编码到另一个智能体的工作副本（kimi 树），--deploy 会用那份副本的 main.cpp 覆盖
机上同名文件 —— 而那份副本是**修复前**的（无断流重开、无 spec 严格解析、无协商
尺寸对账）。回退后的代码照样能编译、照样能出帧，界面上看不出任何异常，
而 L0~L3 也全都不会红（它们走离线图片分支，从不进 runCamera）。

那份脚本在 2026-10-21 加了「汇流硬门」，但门只看两个能力标记（jpegdec /
DZUAV_VIEW）+ 一个已知坏 md5 —— **本闸门的 11 条锚点一条都不在里面**。
所以「过了它的门」与「带着修复」是两件事，仍需本闸门独立把关。

orin_deploy.py 推的是「脚本所在仓库」，本身没错 —— 错在没有任何一道门确认
「这份源码是不是修复后的」。本脚本就是那道门：部署前跑，任一条锚点缺失即非零退出。

同时打印机载源文件的 md5 清单，让现场日志留得下「到底推上去的是哪一版」。

用法（PC 上，部署前）:
    python tools/orin_deploy_gate.py                     # 校验本仓库
    python tools/orin_deploy_gate.py --root <路径>        # 校验任意一份待部署的树
    python tools/orin_deploy_gate.py --json              # 机器可读
    python tools/orin_deploy_gate.py --manifest out.txt  # md5 清单落盘（随证据归档）

用法（Orin 上，部署后确认落地那份 == 推上去那份）:
    python3 tools/orin_deploy_gate.py --root ~/dzuav_deploy/air_ground

退出码: 0 = 全部通过；1 = 有锚点缺失（不要部署）。
本脚本只读，不改任何文件（--manifest 指定的那一个除外）。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

MAIN = Path("onboard/dzuav/src/main.cpp")
MAPLOC = Path("onboard/dzuav/src/MapLocalizer.cpp")
HBB = Path("onboard/dzuav/src/HbbDetector.cpp")
TYPES = Path("onboard/dzuav/include/dzuav/types.hpp")

# orin_deploy.py 推的就是这 13 个文件（find onboard/dzuav -name '*.cpp' -o ...）；
# 少一个机上编译直接失败，多在部署前就该知道。
DZUAV_SOURCES = (
    Path("onboard/dzuav/CMakeLists.txt"),
    TYPES,
    Path("onboard/dzuav/include/dzuav/CudaCheck.hpp"),
    Path("onboard/dzuav/include/dzuav/TensorRtEngine.hpp"),
    Path("onboard/dzuav/include/dzuav/CellClassifier.hpp"),
    Path("onboard/dzuav/include/dzuav/HbbDetector.hpp"),
    Path("onboard/dzuav/include/dzuav/MapLocalizer.hpp"),
    Path("onboard/dzuav/include/dzuav/UdpBroadcaster.hpp"),
    Path("onboard/dzuav/src/CellClassifier.cpp"),
    HBB,
    MAPLOC,
    Path("onboard/dzuav/src/UdpBroadcaster.cpp"),
    MAIN,
)

# fix_live_hud.py 的 SRC_FILES 名单（它拿这五个做**机上齐备性**点数，`ls | wc -l`
# 必须等于 5）。其中只有 main.cpp 真会被它覆盖（`sftp.put(PATCHED, .../main.cpp)`），
# 另四个只是被点名要求在场。五个都星标的理由不是"都会被覆盖"，而是**修复最密集**：
# 11 条锚点里 8 条落在 main.cpp、MapLocalizer / HbbDetector / types.hpp 各 1 条
# —— 现场对账时先比这五个的 md5，最省事。
CRITICAL_FIVE = ("main.cpp", "CellClassifier.cpp", "MapLocalizer.cpp",
                 "HbbDetector.cpp", "UdpBroadcaster.cpp")


@dataclass(frozen=True)
class Needle:
    """一条必须出现在源码里的锚点。count>1 表示至少出现这么多次。"""
    text: str
    count: int = 1


@dataclass(frozen=True)
class Check:
    cid: str
    title: str
    relpath: Path
    needles: tuple[Needle, ...]
    why: str
    forbid: tuple[str, ...] = field(default=())   # 出现即失败


CHECKS: tuple[Check, ...] = (
    Check("spec-strict", "相机 spec 严格解析（回写比对）", MAIN,
          (Needle("bool parseSpec(const std::string& spec"),
           Needle("spec != canon"),
           Needle("invalid camera spec: '")),
          "spec 静默降档：\"1920x1080@30xyz\" 会被 sscanf 读成 1920x1080@30 并放行，"
          "写错一个字符就以为在跑 30fps、实际按别的 caps 出帧，定位分母全错。"),
    Check("reopen-bounded", "断流有界重开（3 次 + 退避 + 试读判据）", MAIN,
          (Needle("reopen_attempts < 3"),
           Needle("usleep(200000 * reopen_attempts)"),
           Needle("capture.isOpened() && capture.read(probe) && !probe.empty()"),
           Needle("camera stream lost; reopening pipeline (attempt "),
           Needle("return 4;")),
          "一次 USB/CSI 掉线就 return，整局再没有情报、车端只能瞎跑；"
          "而只按 isOpened() 判重开成功，not-negotiated 时会假成功。"),
    Check("reopen-release-first", "重开前先 release 旧管道", MAIN,
          (Needle("capture.release();", 2),),
          "在已打开的 VideoCapture 上直接 open 新 GStreamer pipeline 可能失败，"
          "还会泄漏旧管道；失败后不 release 则下一轮从半开状态重开。"),
    Check("negotiated-size", "协商尺寸 != 请求尺寸必须喊出来", MAIN,
          (Needle("negotiated frame size: "),
           Needle("WARNING: negotiated ")),
          "GStreamer 可以悄悄降档，而内参是按帧分辨率缩放的：尺寸一变定位就系统性"
          "偏移，L1/L2/L3 三级自检都不碰内参，全绿。"),
    Check("output-dir-optional", "离线 output_dir 可省略且不淹 CWD", MAIN,
          (Needle("? std::filesystem::path{}"),),
          "path(\"\") / \"x_0.jpg\" 是非空相对路径：不带 output_dir 跑离线视频会把"
          "每帧标注图写进 CWD，3 分钟素材 ~5400 张 JPEG，跑完之前就把 Orin 盘写满。"),
    Check("frame-errors-shared", "连续异常上限两处共用同一常量", MAIN,
          # 盯的是**比较语句**而不是常量名本身。源文件里 kMaxFrameErrors 实到
          # 4 处（声明 + 相机分支比较 + 离线分支日志 + 离线分支比较），按名字数
          # 到 3 的话，删掉相机分支那一处比较、把 30 写回字面量，仍剩 3 处 -> 照绿。
          # 「两处共用同一常量」的真实判据就是这句比较出现两次。
          (Needle("frame_errors >= kMaxFrameErrors", 2),),
          "相机分支与离线分支各写一个字面量 30 迟早分叉：一处改了另一处没改，"
          "同一份二进制对同类故障的容忍度就不一样了。"),
    Check("offline-interruptible", "离线循环响应 g_running", MAIN,
          (Needle("g_running.load() && capture.read(frame)"),),
          "SIGINT 处理只置标志不终止进程：循环不检查标志就 Ctrl-C 不掉，"
          "现场只能 kill -9，证据文件写到一半。"),
    Check("map-isfinite", "定位输出 NaN 守卫", MAPLOC,
          (Needle("std::isfinite(out.x)"), Needle("std::isfinite(out.y)")),
          "PnP/单应退化时解出 NaN，广播出去车端照单全收，导航目标变成 NaN。"),
    Check("hbb-centered-letterbox", "letterbox 居中（与训练分布一致）", HBB,
          (Needle("(kInputSide - nw) / 2"), Needle("pad_x"), Needle("pad_y")),
          "Ultralytics 的 letterbox 是居中的；贴左上角填充会让整幅画面在模型坐标系里"
          "偏移 pad_y，与所有既有指标的输入分布都不同，召回静默掉档。"),
    Check("classes-13", "13 类协议（真源）", TYPES,
          (Needle("UavEnd = 12"), Needle("Paper = 0")),
          "7 类旧枚举上线 = 类别索引错位：索引 5 文档写 target_blue、实际是 obstacle，"
          "车端会朝障碍物开火，而全场只有 10 发。"),
    Check("classname-table", "main.cpp 类别名表与 13 类对齐", MAIN,
          (Needle("\"uav_end\""), Needle("\"paper\""), Needle("\"obstacle\"")),
          "二进制里的类别名表是线上唯一能查版本的凭据（orin_accept_plan.sh 也靠它"
          "grep uav_end 判 13 类）；缺一个就说明这份源码不是 13 类线。"),
)


def _read(root: Path, rel: Path) -> str:
    return (root / rel).read_text(encoding="utf-8", errors="replace")


def run_checks(root: Path) -> list[dict]:
    out = []
    for c in CHECKS:
        p = root / c.relpath
        if not p.is_file():
            out.append({"id": c.cid, "title": c.title, "ok": False,
                        "missing": [f"<文件不存在: {c.relpath}>"], "why": c.why})
            continue
        text = _read(root, c.relpath)
        missing = [n.text for n in c.needles if text.count(n.text) < n.count]
        hit_forbid = [f for f in c.forbid if f in text]
        ok = not missing and not hit_forbid
        out.append({"id": c.cid, "title": c.title, "ok": ok,
                    "missing": missing + [f"<禁止出现: {f}>" for f in hit_forbid],
                    "why": c.why})
    return out


def check_object_class_count(root: Path) -> dict:
    """types.hpp 的 ObjectClass 必须恰好 13 个成员、值 0..12 连续。

    单独拎出来是因为它不是「锚点在不在」而是「枚举整体形状对不对」：
    少一个成员或多一个成员，className 表与车端 ObjectType 会一起错位。
    """
    text = _read(root, TYPES) if (root / TYPES).is_file() else ""
    m = re.search(r"enum\s+class\s+ObjectClass\b[^{]*\{(.*?)\}", text, re.S)
    if not m:
        return {"id": "classes-count", "title": "ObjectClass 恰好 13 类且 0..12 连续",
                "ok": False, "missing": ["types.hpp 里找不到 enum class ObjectClass"],
                "why": "13 类是协议真源，见 docs/interface_contract.md §2.4。"}
    members = re.findall(r"^\s*([A-Za-z_]\w*)\s*=\s*(\d+)", m.group(1), re.M)
    values = [int(v) for _, v in members]
    ok = len(members) == 13 and values == list(range(13))
    return {"id": "classes-count", "title": "ObjectClass 恰好 13 类且 0..12 连续",
            "ok": ok,
            "missing": [] if ok else [f"实到 {len(members)} 个成员，值 {values}"],
            "why": "13 类是协议真源；成员数或取值一变，三处（types.hpp / "
                   "video_recon.NAMES / intel_protocol.ObjectType）必须同改。"}


def check_shell_line_endings(root: Path) -> dict:
    """tools/orin_*.sh 必须是 LF。

    CRLF 的 shebang 是 "#!/bin/bash\\r"，机上执行直接
    "/bin/bash^M: bad interpreter"——而 SFTP put 是逐字节拷贝，
    Windows 上被 core.autocrlf 转成 CRLF 的脚本推上去就是这个下场。
    """
    bad = []
    for p in sorted((root / "tools").glob("orin_*.sh")) if (root / "tools").is_dir() else ():
        if b"\r\n" in p.read_bytes():
            bad.append(p.name)
    return {"id": "sh-lf", "title": "tools/orin_*.sh 为 LF（CRLF 会 bad interpreter）",
            "ok": not bad, "missing": bad,
            "why": ".gitattributes 规定 *.sh eol=lf；一旦是 CRLF，机上第一条 "
                   "tools/orin_*.sh 指令就报 /bin/bash^M: bad interpreter。"}


def check_dzuav_sources_present(root: Path) -> dict:
    # as_posix() 而不是 str()：Windows 上 str(Path) 出反斜杠，与闸门其它地方
    # （md5 清单、密码命中）以及 orin_deploy.py 打印的远端路径口径不一致，
    # 操作手对不上号就会怀疑是别的文件缺了。
    missing = [r.as_posix() for r in DZUAV_SOURCES if not (root / r).is_file()]
    return {"id": "sources-present", "title": f"dzuav 源文件齐全（{len(DZUAV_SOURCES)} 个）",
            "ok": not missing, "missing": missing,
            "why": "orin_deploy.py 整目录推 onboard/dzuav；少一个机上 cmake 直接失败。"}


# 占位符形态：文档/脚本里"该填密码的位置"是允许的，填了真值才是泄漏。
# 用**前缀**匹配而不是全匹配：报错文案常写成 `set AG_ORIN_PASS=******（或
# --password）`，全角括号不在任何分词边界上，全匹配就会把这条正确用法判成泄漏
# （本脚本首版就栽在这里，14 处全红而其中只有 1 处是真的——一道天天红的门，
# 现场第三天就没人看了）。
_PLACEHOLDER = re.compile(
    r"""(?:^\*+                              # ******
          |^\.{3,}                           # ...
          |^<[^>]*>                          # <密码>
          |^"\$env:\w+|^'\$env:\w+|^\$env:\w+  # PowerShell
          |^\$\{\w+\}?|^\$\w+                # bash ${VAR} / $VAR
          |^%%\w+                            # 批处理 for 变量 %%p
          |^%\w+%                            # cmd set X=%Y%
          )""", re.X)
# 值取到第一个空白 / 半角或全角引号 / 全角括号为止。开头的引号必须先吃掉：
# PowerShell 那种「变量名 = 单引号包裹的真值」写法，若把引号直接当分词边界，
# 取到的值就是空串、当场漏检——而那恰恰是本仓唯一一处真泄漏的写法。
# （本文件自己也在必扫范围内：注释里不写任何赋值样例，否则闸门会红在自己头上。）
_SECRET_ASSIGN = re.compile(r"""\b(AG_(?:ORIN|CAR)_PASS)\b\s*[=:]\s*["']?([^\s"'（）]*)""")
# 判失败的边界 = **部署载荷**（orin_deploy.py 实际会推上机的那些路径）。
# 载荷里带明文密码 = 密码随 sftp 落到无人机上，这才是闸门该拦的。
# 载荷外（docs/ 会话纪要、历史 attic 脚本）的命中降级成警告：那是既有的仓库
# 卫生问题，不该让"部署一份修好的机载代码"因此永远红——一道永远红的门，
# 现场第三天就没人看了，比没有门更危险。
_PAYLOAD_PREFIXES = ("onboard/dzuav/", "src/ag_integrate/", "models/release/")
_PAYLOAD_TOOL_RE = re.compile(r"^tools/orin_.*\.(?:sh|py)$")
# 团队既有约定：车端密码归档在 docs/raw_txt/05_VNC远程桌面使用方法.txt
# （remote_run.py 的 _CAR_DOCS_HINT 直接指它），连警告都不必每次刷。
_SECRET_DOC_DIR = "docs/raw_txt"
# models/ 不在跳过名单里：models/release/README.md 是部署载荷的一部分
# （orin_deploy.resolve_current_models 会推它），跳过就等于漏检。
# .onnx/.engine 由下面的后缀白名单挡掉，不需要按目录跳。
_SECRET_SCAN_SKIP = {".git", "build", "__pycache__", ".pytest_cache",
                     "capture", "runs", "node_modules"}
# 白名单的目的是**排除二进制**（.onnx/.engine 几十 MB，逐行读没意义也慢），
# 不是排除源码。首版漏了 C/C++ 后缀，于是 onboard/dzuav/src/*.cpp —— 恰恰是
# 部署载荷里最要命的那部分 —— 根本没被扫过：在里面硬编码一句口令，闸门照绿。
# 这类"扫了但没扫到"的漏洞比不扫更坏，因为报告上写着 [通过]。
_SECRET_SCAN_SUFFIX = {".py", ".sh", ".bat", ".ps1", ".md", ".txt", ".yaml",
                       ".yml", ".json", ".cfg", ".ini", ".xml",
                       ".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp",
                       # 无后缀也要扫：models/release/current_hbb_detector 这类
                       # 指针文件是纯文本且**确实在部署载荷里**（resolve_current_models
                       # 会推它），按后缀白名单一刀切就把它漏成免检。
                       # 全仓无后缀文件仅 9 个、最大 33 字节，扫它没有代价。
                       ""}


def _in_payload(rel: str) -> bool:
    return rel.startswith(_PAYLOAD_PREFIXES) or bool(_PAYLOAD_TOOL_RE.match(rel))


def check_no_plaintext_password(root: Path) -> dict:
    hits: list[str] = []      # 载荷内 -> 判失败
    warns: list[str] = []     # 载荷外 -> 警告
    known: list[str] = []     # docs/raw_txt 既有约定 -> 不计
    for p in root.rglob("*"):
        if not p.is_file() or p.suffix.lower() not in _SECRET_SCAN_SUFFIX:
            continue
        if any(part in _SECRET_SCAN_SKIP for part in p.parts):
            continue
        rel = p.relative_to(root).as_posix()
        try:
            text = p.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        for i, line in enumerate(text.splitlines(), 1):
            for m in _SECRET_ASSIGN.finditer(line):
                value = m.group(2).strip().strip("'\"")
                if not value or _PLACEHOLDER.match(value):
                    continue
                where = f"{rel}:{i}  {m.group(1)}=<字面值 {len(value)} 字符>"
                if rel.startswith(_SECRET_DOC_DIR):
                    known.append(where)
                elif _in_payload(rel):
                    hits.append(where)
                else:
                    warns.append(where)
    return {"id": "no-plaintext-password",
            "title": "部署载荷内无明文 Orin/车端密码",
            "ok": not hits, "missing": hits, "warn": warns, "known": known,
            "why": "载荷里的明文密码会随 sftp 落到无人机上、随 git 进历史，撤不回来；"
                   "统一走 AG_ORIN_PASS / AG_CAR_PASS 环境变量"
                   "（remote_run.read_password 已收编五处副本）。"}


def md5_manifest(root: Path) -> list[tuple[str, str]]:
    out = []
    for rel in DZUAV_SOURCES:
        p = root / rel
        out.append((hashlib.md5(p.read_bytes()).hexdigest() if p.is_file() else "-" * 32,
                    rel.as_posix()))
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="orin_deploy_gate.py",
        description="上机前源码完整性闸门：确认待部署的树带着全部已知修复")
    ap.add_argument("--root", default=None,
                    help="待校验的仓库根（默认：本脚本所在仓库）")
    ap.add_argument("--json", action="store_true", help="机器可读输出")
    ap.add_argument("--manifest", default=None,
                    help="把 md5 清单写到这个文件（LF），随现场证据归档")
    args = ap.parse_args(argv)

    root = Path(args.root).expanduser().resolve() if args.root \
        else Path(__file__).resolve().parents[1]
    if not root.is_dir():
        print(f"[失败] 目录不存在: {root}", file=sys.stderr)
        return 1

    results = [check_dzuav_sources_present(root), *run_checks(root),
               check_object_class_count(root), check_shell_line_endings(root),
               check_no_plaintext_password(root)]
    manifest = md5_manifest(root)
    failed = [r for r in results if not r["ok"]]
    warns = [w for r in results for w in r.get("warn", [])]
    known = [w for r in results for w in r.get("known", [])]

    if args.json:
        print(json.dumps({"root": str(root), "pass": not failed,
                          "results": results,
                          "md5": {rel: h for h, rel in manifest}},
                         ensure_ascii=False, indent=2))
    else:
        for _s in (sys.stdout, sys.stderr):
            try:
                _s.reconfigure(errors="replace")
            except Exception:  # noqa: BLE001 - 老解释器/被重定向时不强求
                pass
        print(f"==== 上机源码完整性闸门: {root} ====")
        for r in results:
            print(f"  [{'通过' if r['ok'] else '失败'}] {r['id']:<24} {r['title']}")
            for m in r["missing"]:
                print(f"           缺: {m}")
            if not r["ok"]:
                print(f"           后果: {r['why']}")
        for w in warns:
            # 载荷外的明文密码不拦部署，但必须让人看见：它是仓库卫生问题，
            # 静默吞掉就等于默许它继续留在 git 历史里。
            print(f"  [警告] 载荷外发现明文密码（不拦部署，建议清理）: {w}")
        if known:
            print(f"  [信息] docs/raw_txt 内既有密码文档 {len(known)} 处"
                  f"（团队约定位置，见 remote_run._CAR_DOCS_HINT）")
        print("\n---- 机载源文件 md5（现场日志请留档）----")
        # ⚠️ 这份 md5 是**本机工作区字节**的 md5，不是 LF 规范形的。本仓
        # core.autocrlf=true，而 .gitattributes 只锁了 *.sh/*.py/*.md/*.txt 的行尾、
        # **没锁 *.cpp/*.hpp** —— 于是 Windows 检出后 .cpp 全是 CRLF（实测
        # main.cpp：CRLF 版 4e2b679b… / LF 版 fa4e2cca…，两者不同）。
        # sftp_put 是逐字节拷贝，机上那份因此也是 CRLF、md5 能对上；但只要机上
        # 那份被 Linux 侧重新生成或重新 checkout 过，同样的源码就会给出不同 md5。
        # 现场看到"md5 不一致"先想到这里，别先怀疑部署坏了。
        print("     （本机工作区字节的 md5；.cpp/.hpp 未锁行尾，CRLF 与 LF 版 md5 不同。")
        print("       只有机上那份也是 orin_deploy.py 推上去的，两边才可直接比对。）")
        # 两种星标分开写，别混成一句"会被覆盖"：main.cpp 是 fix_live_hud.py --deploy
        # 真正 sftp.put 的那一个；另四个只是它的机上齐备性点名对象，不会被它动。
        # 标错了，操作手去核对 fix_live_hud.py 发现只写一个文件，就会连带不信整份清单。
        star = {f"onboard/dzuav/src/{n}" for n in CRITICAL_FIVE}
        overwritten = f"onboard/dzuav/src/{CRITICAL_FIVE[0]}"
        for h, rel in manifest:
            tag = ""
            if rel == overwritten:
                tag = "   ← 修复密集，且 fix_live_hud.py --deploy 会覆盖它"
            elif rel in star:
                tag = "   ← 修复密集，对账重点"
            print(f"  {h}  {rel}{tag}")

    if args.manifest:
        # newline="\n"：.gitattributes 要求 *.txt 是 LF，write_text 在 Windows 上
        # 会翻成 CRLF，于是每归档一份证据工作区就永久显示"脏"（remote_run.py:341
        # 已踩过两次）。
        with open(args.manifest, "w", encoding="utf-8", newline="\n") as f:
            for h, rel in manifest:
                f.write(f"{h}  {rel}\n")

    if failed:
        if not args.json:
            print(f"\n[失败] {len(failed)} 项锚点缺失 —— **不要部署**。", file=sys.stderr)
            print("       这棵树不是修复后的版本（常见原因：--root 指到了别的智能体的"
                  "工作副本，或部署脚本里 REPO 被硬编码）。", file=sys.stderr)
        return 1
    if not args.json:
        print(f"\n[通过] {len(results)} 项全绿；可以部署。")
        print("       部署后在 Orin 上跑: python3 tools/orin_deploy_gate.py "
              "--root ~/dzuav_deploy/air_ground  # 确认落地那份就是这份")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
