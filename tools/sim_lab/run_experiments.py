# -*- coding: utf-8 -*-
r"""调参实验室 · 入口：把每个"教学参数"跑成一对 A/B 实测。

一句话用法（在 `dzrobot_v2/` 下）
------------------------------------------------------------------------
    python tools/sim_lab/run_experiments.py --list        # 看有哪些实验
    python tools/sim_lab/run_experiments.py --exp cruise_speed_up
    python tools/sim_lab/run_experiments.py --all         # 全部（约 1~2 分钟）
    python tools/sim_lab/run_experiments.py --guide       # 怎么跑 / 看什么 / 判据
    python tools/sim_lab/run_experiments.py --selftest    # 判据自检（毫秒级，不跑仿真）

产物（默认 `artifacts/sim_lab/<实验号>/`，已在 .gitignore 里）
------------------------------------------------------------------------
    A_<出厂>/traj.csv  traj.svg  by_segment.csv     ← sim_cli 真产物，可直接用浏览器看 SVG
    B_<改值>/traj.csv  traj.svg  by_segment.csv
    report.txt                                        ← 指标全表 + 一句话结论 + 机器判据
    （实验室根）index.txt                              ← 全部实验的一句话结论汇总

退出码
------------------------------------------------------------------------
    0  全部实验通过（每臂退出码 0 + 产物 3/3 非空 + expect 判据成立）
    2  用法 / 环境问题（实验号写错、sim_cli 找不到……）
    3  有实验的 **expect 判据不成立**（该变的没变 / 不该变的变了）
    4  有实验 **没跑完**（退出码非 0 或产物缺失）

⚠️ 本工具只读 `config/robot.yaml`，所有改值都走 `sim_cli --set`（只影响本次运行、不写磁盘）。
   要真的改车上参数，走 `docs/20_资料索引库/车端/12_可调参数速查_按场景.md` 的流程，
   不要以为这里跑出好数就等于改好了文件。
"""

from __future__ import annotations

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import catalog                                            # noqa: E402
import runner                                             # noqa: E402
from runner import disp_width, pad, rule                  # noqa: E402

GUIDE = """调参实验室 · 怎么看怎么用
================================================================================
1) 先看目录：python tools/sim_lab/run_experiments.py --list
   每个实验 = 一个**可调的键** + 一对 A/B 两臂（A = 出厂现值，B = 单变量改值）。
   注入场景那两个实验改的不是配置键，而是"现实误差"（里程标定、链路延迟、定位噪声）。

2) 跑一个：python tools/sim_lab/run_experiments.py --exp cruise_speed_up
   跑完在 artifacts/sim_lab/cruise_speed_up/ 里得到：
     A_默认_0.35/ 与 B_改_0.60/ 各一套 traj.csv（逐步真值+信念轨迹）、traj.svg（直接浏览器看）、
     by_segment.csv（逐段横向误差峰值），以及 report.txt（指标全表 + 一句话结论 + 判据）。

3) 看什么（按优先级）：
   ① report.txt 里的「★ 一句话结论」——这是从实测数字算出来的，不是模板话；
   ② 指标表里带 ★ 的行 = 本实验该看的指标；其余行是"顺带发生了什么"；
   ③ traj.svg —— 两个 SVG 并排打开看拐角，文字表永远替代不了图；
   ④ by_segment.csv —— 想知道"哪几段最差"，按峰值列排序，别盯全局 RMS 猜。

4) 判据（机器判，不靠人眼）：
   change    = 该有变化；若全部逐位相同 → 判失败（键名拼错 / 被钳位吃掉 / 实验坏了）
   same      = 该逐位相同（前视下限那条：被 lookahead_max 钳住，改它没用）
   obs_only  = 只该动观测指标（越界拍数），轨迹/时长/误差必须逐位不变

5) 加一个新实验：只改 catalog.py，追加一个 dict（字段约定写在文件头），
   然后 python tools/sim_lab/run_experiments.py --exp <新id>。
   —— 不要另写脚本：所有实验共用同一套跑分、判据、报告格式，才能横向比较。

6) 判据自检：python tools/sim_lab/run_experiments.py --selftest
   —— 9 个合成用例证明「该判失败的真的判失败了」；判据写成永远 True 是这类工具最常见的烂法。

7) 纪律（血泪）：
   · 一次只改一个变量；两臂之间不许顺手改别的（--set 只给被测那个键）。
   · "改了没反应"先查三件事：键名（点号键 vs SCHEMA）、钳位链（clamp/互斥/迁移）、
     消费者（这个键到底有没有代码在读）。查完再下结论。
   · 仿真绿 ≠ 实车绿：本仿真不建模 PIVOT / ALIGN / 最终限速 / Safety（tools/sim.py 头注）。
   · Windows 控制台必须是 UTF-8（本工具会把 PYTHONIOENCODING=utf-8 传给子进程；
     你自己手敲 sim_cli 时也要设，否则 ★ / ↔ 会抛 UnicodeEncodeError）。
================================================================================
"""


def _print_list() -> None:
    """打印实验目录（id / 分组 / 键 / A→B / 问题）。"""
    print("=" * 96)
    print(" 调参实验室 · 实验目录（%d 个）" % len(catalog.EXPERIMENTS))
    print("=" * 96)
    w_id = max(disp_width(e["id"]) for e in catalog.EXPERIMENTS) + 2
    w_grp = 10
    w_key = max(disp_width(e["key"]) for e in catalog.EXPERIMENTS) + 2
    w_ft = max(disp_width("%s→%s" % e["from_to"]) for e in catalog.EXPERIMENTS) + 2
    print("%s%s%s%s%s" % (pad("实验", w_id), pad("分组", w_grp), pad("被测键", w_key),
                          pad("A→B", w_ft), "问题"))
    print(rule(96))
    for exp in catalog.EXPERIMENTS:
        print("%s%s%s%s%s" % (
            pad(exp["id"], w_id), pad(exp["group"], w_grp), pad(exp["key"], w_key),
            pad("%s→%s" % exp["from_to"], w_ft), exp["question"]))
    print(rule(96))
    print("怎么跑：python tools/sim_lab/run_experiments.py --exp <实验号>  /  --all")
    print("详细说明：python tools/sim_lab/run_experiments.py --guide")


def _run_one(exp: dict, out_root: str, quiet: bool) -> dict:
    """跑一个实验：两臂 + 报告 + 判据。返回汇总条目。"""
    exp_dir = os.path.join(out_root, exp["id"])
    arms = []
    for arm in exp["arms"]:
        slug = "%s_%s" % (arm["id"], arm["label"].replace(" ", "_").replace("/", "-"))
        run = runner.run_arm(exp, arm, os.path.join(exp_dir, slug))
        arms.append(run)
        if not quiet:
            print("   臂 %s %-20s rc=%s 产物 %d/%d %s (%.1f s)" % (
                run.arm_id, run.label, run.rc,
                sum(1 for _, s in run.artifacts if s > 0), len(runner.ARTIFACT_NAMES),
                "OK" if run.ok else "FAIL", run.seconds))

    ra, rb = arms[0], arms[1]
    if not ra.ok or not rb.ok:
        reason = "；".join("%s: %s" % (r.arm_id, r.error or "产物不合格")
                           for r in arms if not r.ok)
        item = {"id": exp["id"], "group": exp["group"], "title": exp["title"],
                "key": exp["key"], "ok": False, "code": 4,
                "conclusion": "未跑完 —— " + reason}
        if not quiet:
            print("   [4/未跑完] %s" % item["conclusion"])
        return item

    rows = runner.diff_rows(ra, rb)
    ok, verdict_text = runner.check_expectation(exp, ra, rb)
    report = runner.write_report(exp, ra, rb, rows, ok, verdict_text, exp_dir)
    item = {"id": exp["id"], "group": exp["group"], "title": exp["title"],
            "key": exp["key"], "ok": ok, "code": 0 if ok else 3,
            "conclusion": runner.conclusion_line(exp, ra, rb),
            "verdict": verdict_text, "report": report,
            "arms": {ra.arm_id: ra.metrics, rb.arm_id: rb.metrics},
            "commands": {ra.arm_id: ra.cmdline(), rb.arm_id: rb.cmdline()}}
    if not quiet:
        print("   [%s] %s" % ("3/判据不成立" if not ok else "通过", item["conclusion"]))
        if not ok:
            print("      ⚠ %s" % verdict_text)
    return item


def main(argv: list[str] | None = None) -> int:
    """入口。返回进程退出码（0/2/3/4，见模块头注）。"""
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except Exception:                                # noqa: BLE001 - 老 Python/被重定向
            pass

    ap = argparse.ArgumentParser(
        prog="run_experiments",
        description="调参实验室：把每个教学参数跑成一对 A/B 实测（不碰车、不改磁盘配置）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="退出码：0 全通过 / 2 用法或环境问题 / 3 判据不成立 / 4 有实验没跑完")
    ap.add_argument("--list", dest="do_list", action="store_true", help="只打印实验目录")
    ap.add_argument("--guide", action="store_true", help="打印『怎么跑 / 看什么 / 判据』长说明")
    ap.add_argument("--selftest", action="store_true",
                    help="判据自检：用合成数据验证 change/same/obs_only 三种判据真的会判错（不跑仿真，毫秒级）")
    ap.add_argument("--exp", nargs="*", default=None, metavar="ID",
                    help="只跑指定实验（可多个）；缺省=跑全部")
    ap.add_argument("--all", dest="do_all", action="store_true", help="跑全部实验（等价于不写 --exp）")
    ap.add_argument("--out-dir", default=runner.DEFAULT_OUT, help="产物根目录")
    ap.add_argument("--json", dest="as_json", action="store_true",
                    help="在 stdout 末尾追加一行 LAB_RESULT_JSON（机器可读汇总）")
    ap.add_argument("--quiet", action="store_true", help="只打最终汇总")
    args = ap.parse_args(argv)

    if args.guide:
        print(GUIDE)
        return 0
    if args.selftest:
        ok, lines = runner.selftest()
        print("=" * 96)
        print(" 判据自检（不跑仿真：验证三种 expect 的判错能力）")
        print("=" * 96)
        for line in lines:
            print(line)
        print(rule(96))
        print("结果: %s" % ("全部通过（判据不是空转）" if ok else "有判据失灵 —— 实验室结论不可信"))
        return 0 if ok else 1
    if args.do_list:
        _print_list()
        return 0

    if args.exp is not None and len(args.exp) > 0:
        wanted = []
        for name in args.exp:
            try:
                wanted.append(catalog.by_id(name))
            except KeyError:
                print("❌ 没有这个实验: %s" % name, file=sys.stderr)
                print("   可用: %s" % ", ".join(catalog.ids()), file=sys.stderr)
                return 2
    else:
        wanted = list(catalog.EXPERIMENTS)

    out_root = os.path.abspath(args.out_dir)
    os.makedirs(out_root, exist_ok=True)
    if not args.quiet:
        print("=" * 96)
        print(" 调参实验室 · 跑 %d 个实验（A=出厂现值 / B=单变量改值）" % len(wanted))
        print(" 仓库: %s" % runner.REPO)
        print(" 产物: %s" % out_root)
        print("=" * 96)

    summary = []
    worst = 0
    for exp in wanted:
        if not args.quiet:
            print("[%s] %s" % (exp["id"], exp["title"]))
        item = _run_one(exp, out_root, args.quiet)
        summary.append(item)
        worst = max(worst, item["code"])

    index_path = runner.write_lab_index(summary, out_root)
    ok_count = sum(1 for i in summary if i["ok"])
    print("")
    print("=" * 96)
    print(" 汇总：%d 个实验 / 通过 %d / 失败 %d" % (len(summary), ok_count, len(summary) - ok_count))
    print("=" * 96)
    w_id = max(disp_width(i["id"]) for i in summary) + 2
    w_grp = max(disp_width(i["group"]) for i in summary) + 2
    for item in summary:
        print("%s%s %s %s" % (pad(item["id"], w_id), pad(item["group"], w_grp),
                              pad("通过" if item["ok"] else "失败", 8), item["conclusion"]))
    print(rule(96))
    print("索引: %s" % index_path)
    for item in summary:
        if item.get("report"):
            print("  报告: %s" % item["report"])
            break
    print("   （每个实验目录下各有一份 report.txt）")

    if args.as_json:
        payload = {
            "ok": worst == 0,
            "total": len(summary),
            "passed": ok_count,
            "failed": len(summary) - ok_count,
            "out_dir": out_root,
            "index": index_path,
            "experiments": summary,
        }
        print("LAB_RESULT_JSON " + json.dumps(payload, ensure_ascii=False))
    return worst


if __name__ == "__main__":      # pragma: no cover
    sys.exit(main())
