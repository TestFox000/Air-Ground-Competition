# -*- coding: utf-8 -*-
r"""部署通道测试 —— 钉死 2026-10-08 真机踩到的两个"静默失效"：

① `scp -r 本地目录 车端同名目录`：目标已存在时 scp 会把源目录**整个塞进去**，
   变成 `dst/<basename>/...`。结果是"推送成功"但车端一个文件都没更新，
   colcon 仍按旧 package.xml 识别成 `python` 构建类型 → 不生成 ament 前缀钩子
   → `ros2 pkg executables dzbrain` 报 "Package 'dzbrain' not found"。
   现在目录走 tar 流就地替换，且推送后强制 sha256 对账。

② `package.xml` 的 `<maintainer email="duzhong@local">`：catkin_pkg 认为邮箱非法，
   `parse_package` 抛 InvalidPackage → colcon-ros 放弃识别 → 同样退回 `python` 类型。
   邮箱必须像合法邮箱（有域名点号），且 `<build_type>` 必须在 `<export>` 里。

运行：python src\dzbrain\tests\test_deploy.py
"""

from __future__ import annotations

import importlib.util
import io
import pathlib
import re
import sys
import tarfile

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from harness import check, check_raises, finish, section  # noqa: E402

WS = pathlib.Path(__file__).resolve().parents[3]

# 以"文件路径"方式导入 tools/deploy_to_car.py（tools 不是包，且不能污染 sys.path）
_spec = importlib.util.spec_from_file_location("deploy_to_car", WS / "tools" / "deploy_to_car.py")
dep = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(dep)


# --------------------------------------------------------------------------- #
section("① package.xml：邮箱合法性 + build_type 位置（catkin_pkg 的硬要求）")
pkg_xml = (WS / "src" / "dzbrain" / "package.xml").read_text(encoding="utf-8")

# 与 catkin_pkg 同口径的粗略校验：必须有 local@domain.tld
EMAIL_RE = re.compile(r"^[^@\s]+@[^@\s.]+\.[A-Za-z]{2,}$")
mails = re.findall(r'<maintainer email="([^"]*)"', pkg_xml)
check("能解析出 maintainer 邮箱", len(mails) >= 1, f"{mails}")
for m in mails:
    check(f"maintainer 邮箱合法：{m}", bool(EMAIL_RE.match(m)),
          "非法邮箱会让 catkin_pkg 抛 InvalidPackage → colcon 退回 python 构建类型")
check("★ 反例确认：@local 这种写法必须被判非法", not EMAIL_RE.match("duzhong@local"))
check("★ 反例确认：duzhong@example.com 必须合法", bool(EMAIL_RE.match("duzhong@example.com")))
check("maintainer 标签成对闭合", pkg_xml.count("<maintainer") == pkg_xml.count("</maintainer>"))

check("<export> 段存在", "<export>" in pkg_xml and "</export>" in pkg_xml)
export_body = pkg_xml.split("<export>", 1)[-1].split("</export>", 1)[0]
check("★ build_type 在 <export> 内（放外面 colcon-ros 认不出）",
      "<build_type>ament_python</build_type>" in export_body)
check("package format=3", 'format="3"' in pkg_xml)
check("本地 package.xml 无维护者邮箱回归（曾出现 @local）", "duzhong@local" not in pkg_xml)


section("② 目录推送：tar 流就地替换，杜绝 scp 嵌套")
tar_bytes = dep.make_tarball("src/dzbrain")
with tarfile.open(fileobj=io.BytesIO(tar_bytes), mode="r:gz") as tf:
    names = tf.getnames()

check("tarball 非空", len(names) > 40, f"{len(names)} 个成员")
check("★ 归档成员与本地清单 1:1（多一层壳就会破坏相等）",
      set(names) == set(dep.local_manifest("src/dzbrain")), f"{len(names)} vs {len(dep.local_manifest('src/dzbrain'))}")
check("★ 顶层文件落在归档顶层（有壳层时会变成 dzbrain/package.xml）",
      "package.xml" in names and "setup.py" in names)
check("★ 归档里不存在被套一层的形态 dzbrain/package.xml", "dzbrain/package.xml" not in names)
check("核心模块在包内", "dzbrain/main.py" in names)
check("resource 标记在包内（setup.py data_files 依赖它）", "resource/dzbrain" in names)
check("tests 在包内", any(n.startswith("tests/test_") for n in names))
check("★ __pycache__ 不打进包（不污染车端）", not any("__pycache__" in n for n in names))
check(".pyc 不打进包", not any(n.endswith(".pyc") for n in names))

swap = dep.dir_swap_cmd(dep.CAR_WS + "/src/dzbrain")
check("替换脚本用 tar 解流（-C 指定目标）", "tar -xzf - -C" in swap)
check("替换脚本先解到 .dznew 再原子 mv（不留半截目录）", ".dznew" in swap and "mv " in swap, swap[:80])
check("替换脚本 set -e（失败即停，不静默继续）", swap.startswith("set -e;"))

src_text = (WS / "tools" / "deploy_to_car.py").read_text(encoding="utf-8")
check("★ 全工具里不再出现 `scp -r`（嵌套元凶）", '"scp", "-r"' not in src_text and '"-r", *SSH_OPTS' not in src_text)
check("文件推送仍是 scp（单文件覆盖语义正确）", '"scp", *SSH_OPTS' in src_text)
check("推送清单里 dzbrain 指向车的 src/dzbrain", (dep.PUSH_FILES[0][0], dep.PUSH_FILES[0][1]) ==
      ("src/dzbrain", dep.CAR_WS + "/src/dzbrain"))


section("③ 对账闸门：漏推/推错位置必须被抓住（否则「没推上去」会伪装成「已部署」）")
local_m = dep.local_manifest("src/dzbrain")
check("本地清单含 main.py", "dzbrain/main.py" in local_m)
check("本地清单不含 __pycache__", not any("__pycache__" in k for k in local_m))
check("sha256 是 64 位十六进制", all(len(v) == 64 for v in local_m.values()))

missing, extra, mismatch = dep.diff_manifest(local_m, dict(local_m))
check("本地 == 车端 → 三项全空（通过）", not missing and not extra and not mismatch)

# 负向控制：复刻"scp 嵌套"的真实后果 —— 车端清单里文件名前缀多了一层 dzbrain/
nested = {f"dzbrain/{k}": v for k, v in local_m.items()}
missing, extra, mismatch = dep.diff_manifest(local_m, nested)
check("★ 嵌套推送必须被判为「全部缺失」", len(missing) == len(local_m) and not mismatch,
      f"missing={len(missing)}/{len(local_m)}")
check("★ 嵌套推送同时暴露「车端多出」", len(extra) == len(local_m))

# 负向控制：改了本地没推上去（内容不一致）
stale = dict(local_m)
stale["dzbrain/main.py"] = "0" * 64
missing, extra, mismatch = dep.diff_manifest(local_m, stale)
check("★ 内容不一致必须被点名", mismatch == ["dzbrain/main.py"])
check("内容不一致时 missing 为空（区分「没推」与「推错」）", missing == [])

# 负向控制：车端少推了一个文件
partial = {k: v for k, v in local_m.items() if k != "dzbrain/tasks/aim.py"}
missing, extra, mismatch = dep.diff_manifest(local_m, partial)
check("★ 少推一个文件必须被点名", missing == ["dzbrain/tasks/aim.py"])
check("少推文件时 mismatch 为空", mismatch == [])

check("单文件清单用文件名做 key", set(dep.local_manifest("config/robot.yaml")) == {"robot.yaml"})


section("③b 推送后归一化：对账基准必须算上「工具自己改的那一处」，否则假警报")
check("归一化只把 auto_start: true 改成 false",
      dep.normalize_auto_start("auto_start: true") == "auto_start: false")
check("归一化不动 auto_start: false（幂等）",
      dep.normalize_auto_start("auto_start: false") == "auto_start: false")
check("归一化不碰别的键",
      dep.normalize_auto_start("auto_start: true\nmax_speed: 1.5") == "auto_start: false\nmax_speed: 1.5")
check("POST_PUSH_TRANSFORM 只登记被推送且会被车端改写的文件",
      set(dep.POST_PUSH_TRANSFORM) <= {loc for loc, _ in dep.PUSH_FILES})

# 忠实复刻车端 read_text/write_text 语义：改动了才写回，写回即 LF
_crlf_true = b"a: 1\r\nauto_start: true\r\n"
check("★ 有改动 → 期望内容是 LF 且已归一化",
      dep.expected_remote_bytes("config/robot.yaml", _crlf_true) == b"a: 1\nauto_start: false\n")
_crlf_false = b"a: 1\r\nauto_start: false\r\n"
check("★ 无改动 → 期望内容原样保留（车端不写回，CRLF 也得保留）",
      dep.expected_remote_bytes("config/robot.yaml", _crlf_false) == _crlf_false)
_raw = b"whatever\r\n"
check("未登记归一化的文件按原始字节对账",
      dep.expected_remote_bytes("config/map.yaml", _raw) == _raw)

# 真实文件：车端 robot.yaml 的 sha256 应该等于「归一化后」而不是「原始字节」
_rp = WS / "config" / "robot.yaml"
_raw_bytes = _rp.read_bytes()
_expected = dep.expected_remote_bytes("config/robot.yaml", _raw_bytes)
check("★ 本地 robot.yaml 含 auto_start: true（这正是不发车闸门的输入）",
      b"auto_start: true" in _raw_bytes or b"auto_start:true" in _raw_bytes)
check("★ 对账基准用的是归一化内容，不是原始字节",
      dep.sha256_bytes(_expected) != dep.sha256_bytes(_raw_bytes),
      "若相等说明归一化没生效 → 会重现'假警报'或'假通过'")
_raw_text = _raw_bytes.decode("utf-8").replace("\r\n", "\n").replace("\r", "\n")
_exp_text = _expected.decode("utf-8")
check("归一化不增删行（只改行内内容）",
      len(_exp_text.splitlines()) == len(_raw_text.splitlines()),
      f"{len(_exp_text.splitlines())} vs {len(_raw_text.splitlines())}")
_diff_lines = [(a, b) for a, b in zip(_raw_text.splitlines(), _exp_text.splitlines()) if a != b]
check("★ 变化的行全部是 auto_start 行（不误伤别的配置）",
      len(_diff_lines) >= 1 and all("auto_start" in a for a, _ in _diff_lines),
      f"改了 {len(_diff_lines)} 行")
check("★ 每处 auto_start: true 都被改成 false",
      _exp_text.count("auto_start: true") == 0 and _exp_text.count("auto_start: false") == 2)
check("注释与其它键原样保留", "自动出发" in _exp_text and "auto_start_delay: 10.0" in _exp_text)


section("④ 包完整性：整包替换后车端能直接编译（少一个文件就会把车端搞挂）")
for rel in ("package.xml", "setup.py", "setup.cfg", "resource/dzbrain", "dzbrain/__init__.py"):
    check(f"src/dzbrain/{rel} 存在", (WS / "src" / "dzbrain" / rel).exists())
setup_py = (WS / "src" / "dzbrain" / "setup.py").read_text(encoding="utf-8")
check("setup.py 里 maintainer_email 也合法（setuptools 元数据）",
      bool(EMAIL_RE.match(re.search(r'maintainer_email="([^"]*)"', setup_py).group(1))))
check("setup.py 注册了 console_scripts 入口", '"console_scripts"' in setup_py)
check("setup.py 打包 resource 标记", "share/ament_index/resource_index/packages" in setup_py)
check_raises("本地文件缺失时 local_manifest 抛错（不静默返回空）",
             FileNotFoundError, dep.local_manifest, "src/dzbrain/不存在的文件.py")

section("⑤ 出车体检脚本必须覆盖**新核心**契约（旧脚本只体检旧架构 = 假全绿）")
#  为什么钉这条：`tools/healthcheck.sh` 原本只检查 usb_cam / det_node / dzrknn /
#  path_executor / serial_reader 与 pursuitAngle —— 新版决策层 dzbrain 的契约
#  （双钥匙开火门控、裁判回传真源、保活速率）**一条都没查**。出车前跑一遍会拿到"全绿"，
#  而云台可能永远打不出去（缺 `/aim/fire_ready`）或命中闭环空转（`/HpAndHitmsg` 无数据）。
#  这类"假全绿"必须在测试里钉死，否则改脚本时会被人悄悄删掉。
_hc = (WS / "tools" / "healthcheck.sh").read_text(encoding="utf-8")
for _need, _why in (
        ("dzbrain", "必须识别新核心节点（否则新核心没起来也报全绿）"),
        ("/aim/fire_gate", "双钥匙门控第一把钥匙"),
        ("/aim/fire_ready", "双钥匙门控第二把钥匙（缺它就永远不开火）"),
        ("/HpAndHitmsg", "裁判回传真源（命中即停火 / 弹药对账）"),
        ("pursuitAngle", "底盘 1s 看门狗的保活指令"),
        ("LEGACY_PATH", "走旧核时必须降级为提示，而不是误报失败"),
        ("--pre", "--pre 模式（节点未起）不得报错")):
    check(f"healthcheck.sh 覆盖 {_need}（{_why}）", _need in _hc)
check("healthcheck.sh 语法基本自洽：if/then/fi 数量配平",
      _hc.count("\nif ") + _hc.count("\n  if ") + _hc.count("\nelif ") + _hc.count("\n    elif ") > 0
      and _hc.count("fi\n") >= _hc.count("\nif ") - _hc.count("elif "),
      f"if={_hc.count('if ')} fi={_hc.count('fi')}")

finish()
