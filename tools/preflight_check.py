#!/usr/bin/env python3
"""上场前体检：把「只能现场确定 / 最容易忘」的项逐条查一遍。

为什么要有这个：
    `docs/实机接口对照.md` §8 列了 6 件"必须现场量/确认"的事（终点格号、摆放朝向、
    桥洞占格、打击朝向、命中换算、激光偏置），`CHECKLIST.md` 里也有一堆勾选项。
    但那些都是**人去勾**的 —— 比赛前 2 分钟没人会一条条看。
    这里把其中**能从配置里查出来的**部分自动查掉，剩下的才需要人确认。

用法：
    python run.py preflight              # 或：python tools/preflight_check.py
    python tools/preflight_check.py --run-tests

退出码：0 = 没有 FAIL；1 = 有 FAIL（WARN 只提示，不拦）。
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import unicodedata
from pathlib import Path
from typing import List

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
# 与其它 tools/*.py 一致：把 src 下**每个**包目录都加进 sys.path
# （真实事故：ag_core 改成两层布局后，少加一层就 ModuleNotFoundError）
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))
for p in sorted(SRC.iterdir()) if SRC.is_dir() else []:
    if p.is_dir() and str(p) not in sys.path:
        sys.path.insert(0, str(p))

for stream in (sys.stdout, sys.stderr):
    try:
        stream.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

try:
    import yaml_lite
except Exception:  # pragma: no cover
    yaml_lite = None  # type: ignore


DEFAULT_PARAMS = ROOT / "src" / "ag_bringup" / "config" / "params.yaml"


def _pad(text: str, width: int) -> str:
    """按**显示宽度**补空格：中文是双宽，用 str.ljust 会对不齐。"""
    s = str(text)
    w = sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in s)
    return s + " " * max(0, width - w)


def _find_git() -> str:
    """找 git：winget 装完后**新进程**未必立刻拿到 PATH（本机实测如此），
    所以 PATH 里找不到时再按常见安装位置兜一次。"""
    g = shutil.which("git")
    if g:
        return g
    for cand in (r"C:\Program Files\Git\cmd\git.exe",
                 r"C:\Program Files\Git\bin\git.exe",
                 r"C:\Program Files (x86)\Git\cmd\git.exe"):
        if Path(cand).is_file():
            return cand
    return ""


class Row:
    __slots__ = ("level", "item", "state", "detail", "fix")

    def __init__(self, level: str, item: str, state: str, detail: str, fix: str = "") -> None:
        self.level, self.item, self.state, self.detail, self.fix = level, item, state, detail, fix


def _flat(cfg: dict, section: str, key: str):
    try:
        return cfg[section][key]
    except Exception:
        return None


def check_params(cfg: dict) -> List[Row]:
    rows: List[Row] = []

    # ① 广播地址：默认值 255.255.255.255 在跨网段/多网卡时常发不出去，
    #    现场必须改成实际网段的广播地址（CHECKLIST §B 有这条）。
    peer = _flat(cfg, "ground_link_node", "peer_ip")
    if peer in (None, "", "255.255.255.255"):
        rows.append(Row("FAIL", "链路广播地址", "未设置", f"peer_ip={peer!r}",
                        "改成现场网段的广播地址（如 192.168.1.255）；两端 peer/bind 端口要交叉对应"))
    else:
        rows.append(Row("OK", "链路广播地址", str(peer),
                        "⚠️ 必须与现场实际网段一致（车是 DHCP，网段会变）",
                        "上车前先 `ip -4 addr show wlan0` 确认网段，再改 peer_ip"))

    # ② 单应：全零 = 未标定 -> 像素落不到格号 -> 机载端发不出情报
    h = _flat(cfg, "intel_fusion_node", "homography") or []
    if not any(float(v) for v in h):
        rows.append(Row("FAIL", "单应矩阵", "未标定（全零）", "机载端产不出格号情报",
                        "tools/calib_homography.py 标定后填入 9 个数；换场地必须重标"))
    else:
        rows.append(Row("OK", "单应矩阵", "已填", "现场仍要确认是当前场地标的", ""))

    # ③ dry_run：上场必须是 false（默认值故意是 true，第一次上车先核对路线）
    dry = _flat(cfg, "car_node", "dry_run")
    if dry is True:
        rows.append(Row("FAIL", "dry_run", "true", "只算不发，车不会动",
                        "确认路线无误后改成 false（params.yaml 或 ros2 param set）"))
    else:
        rows.append(Row("OK", "dry_run", str(dry), "", ""))

    # ④ 模型：空 = DummyBackend，输出恒 0
    model = _flat(cfg, "vision_node", "model_path")
    if not model:
        rows.append(Row("WARN", "模型路径", "空", "会退化成 DummyBackend（检测恒 0）",
                        "车端可用的是 light_det.rknn；注意别指到旧版多输出的模型"))
    else:
        rows.append(Row("OK", "模型路径", str(model), "确认与训练时的类别顺序一致", ""))

    # ⑤ 场地：这些只能现场量，这里只提醒，不自动判
    start = _flat(cfg, "car_node", "start_cell")
    goal = _flat(cfg, "car_node", "goal_cell")
    rows.append(Row("INFO", "起终点格号", f"{start} -> {goal}",
                    "终点是**固定方框**，规则没给格号，必须现场量", "量完写进 params.yaml 的 goal_cell"))
    rows.append(Row("INFO", "桥洞布点", _flat(cfg, "car_node", "bridge_zone_col") or "?",
                    "我们按「K 列 5+6 格」推断（图 7/8 标识在 K4/K9）",
                    "现场看一眼；确认后可把 strict_bridge_zone 改成 true"))
    return rows


def check_env() -> List[Row]:
    rows = []
    git = _find_git()
    if git:
        try:
            dirty = subprocess.run([git, "status", "--short"], cwd=str(ROOT),
                                   capture_output=True, text=True, encoding="utf-8",
                                   errors="replace").stdout.strip()
            head = subprocess.run([git, "log", "--oneline", "-1"], cwd=str(ROOT),
                                  capture_output=True, text=True, encoding="utf-8",
                                  errors="replace").stdout.strip()
            if dirty:
                rows.append(Row("WARN", "代码版本", "有未提交改动",
                                f"{len(dirty.splitlines())} 个文件改动；最新 {head[:40]}",
                                "上场前提交一次，出问题能回滚"))
            else:
                rows.append(Row("OK", "代码版本", "工作区干净", head[:60], ""))
        except Exception as e:  # noqa: BLE001
            rows.append(Row("INFO", "代码版本", "git 查询失败", f"{type(e).__name__}: {e}", ""))
    else:
        rows.append(Row("WARN", "代码版本", "没装 git", "改坏了回不去",
                        "winget install Git.Git"))
    return rows


def check_tests(run: bool) -> List[Row]:
    if not run:
        return [Row("INFO", "单元测试", "未跑", "加 --run-tests 会真的跑一遍", "")]
    r = subprocess.run([sys.executable, "-m", "pytest", "tests", "-q"], cwd=str(ROOT),
                       capture_output=True, text=True, encoding="utf-8", errors="replace")
    tail = (r.stdout.strip().splitlines() or [""])[-1]
    if r.returncode == 0:
        return [Row("OK", "单元测试", "全绿", tail[:60], "")]
    return [Row("FAIL", "单元测试", "有失败", tail[:80], "修完再上场")]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="上场前体检（配置 + 环境）")
    ap.add_argument("--params", default=str(DEFAULT_PARAMS), help="params.yaml 路径")
    ap.add_argument("--run-tests", action="store_true", help="顺便跑一遍单元测试")
    args = ap.parse_args(argv)

    print("=" * 66)
    print("上场前体检")
    print("=" * 66)

    rows: List[Row] = []
    path = Path(args.params)
    if not path.is_file():
        print(f"[FAIL] 找不到参数文件 {path}")
        return 1
    if yaml_lite is None:
        print("[FAIL] 无法导入 tests/yaml_lite.py（跑测试也需要它）")
        return 1

    cfg = yaml_lite.load_params(path)
    rows += check_params(cfg)
    rows += check_env()
    rows += check_tests(args.run_tests)

    print()
    print(_pad("级别", 6) + _pad("项目", 16) + _pad("现状", 20) + "说明")
    print("-" * 66)
    for r in rows:
        print(_pad(r.level, 6) + _pad(r.item, 16) + _pad(r.state, 20) + r.detail)
        if r.fix and r.level in ("FAIL", "WARN"):
            print(f"      -> {r.fix}")

    n_fail = sum(1 for r in rows if r.level == "FAIL")
    n_warn = sum(1 for r in rows if r.level == "WARN")
    print()
    print("-" * 66)
    print(f"FAIL {n_fail} | WARN {n_warn} | 其余为提示")
    if n_fail:
        print("结论：**还不能上场**，先把上面的 FAIL 项处理掉")
        return 1
    print("结论：没有 FAIL 项；WARN 与提示项仍要现场确认（见 CHECKLIST.md）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
