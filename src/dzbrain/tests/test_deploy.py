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

finish()
