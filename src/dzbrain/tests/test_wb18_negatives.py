# -*- coding: utf-8 -*-
"""WB-18 负例自证：**把闸门拆掉，测试必须变红**。

为什么必须有这一步（本线 L7 / WB-5 的"变异敏感度"）：
  上面 39 条断言全绿，只证明"在闸门存在时它们成立"。
  它**证明不了**这些断言真的在测闸门 —— 如果断言写成 `check(..., True)` 或
  判据恒真，全绿同样成立。唯一能证的办法：把被测机制拆掉，看断言是否变红。

做法：把 `main.py` 里 `if _combo_risk is not None:` 改成 `if False and _combo_risk ...`，
  跑主测试，断言 **rc != 0 且失败项落在预期的那几节**（不是"反正红了就算"）。
  **跑完必须还原，并断言还原后 sha256 与原始逐位相同**（这条本身也是断言）。

⚠️ 防"改坏不还原"（本文作者 2026-11-01 真踩过）：
  第一版把还原放在脚本末尾、中间被 subprocess 的编码异常打断 ⇒ 文件停在短路状态。
  ⇒ 本版用 **try/finally + 原子覆盖 + 事后 sha256 双向核对**，并把"还原成功"打印出来。
  另：subprocess 读 stdout 必须指定 encoding + errors（Windows 控制台默认 GBK 会炸）。

跑法（工作区根 `dzrobot_v2/`）：
    PYTHONPATH=src/dzbrain python src/dzbrain/tests/test_wb18_negatives.py
全过 exit 0。
"""

from __future__ import annotations

import hashlib
import pathlib
import shutil
import subprocess
import sys
import tempfile

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
try:
    sys.stdout.reconfigure(errors="replace")
    sys.stderr.reconfigure(errors="replace")
except Exception:
    pass

from harness import check, finish, section                         # noqa: E402

WS = pathlib.Path(__file__).resolve().parents[3]
MAIN = WS / "src" / "dzbrain" / "dzbrain" / "main.py"
TEST = WS / "src" / "dzbrain" / "tests" / "test_wb18_engine_gate.py"

#: 闸门那一行的原文（变异目标）。
GATE_LINE = "    if _combo_risk is not None:"
GATE_LINE_BROKEN = "    if False and _combo_risk is not None:"

#: 拆闸后**必须变红**的断言号（= 真正在测"闸门被调用"的那几条）。
#: ★ 刻意**不含 1.0**：本次变异拆的是 `_run_live_body` 里的**调用点**（`if False and ...`），
#:   `_planner_engine_combo_risk()` 纯函数本身没被改 ⇒ 1.0（纯判定）**应当仍然绿**。
#:   若把 1.0 也列进来，就等于要求"拆调用点也能打红纯函数"，那是**判据写错**（过严也是失真）。
#:   要打红 1.0 需要另一种变异（改纯函数判据），那属另一条变异，不在本文件。
#:   其余断言（缺键/文案/只读性/不误伤）同理应当在拆闸后仍绿 —— 这正是"分层没串味"的证据。
MUST_RED = ("4.1", "4.2", "4.3")
MAY_SEE_UNRELATED_SKIP = ("5b.",)


def sha256(path: pathlib.Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run_test() -> tuple[int, str]:
    """跑主测试，**显式编码**（Windows 默认 GBK 会在中文输出上炸）。"""
    env = {"PYTHONPATH": str(WS / "src" / "dzbrain"), "PYTHONIOENCODING": "utf-8",
           "PYTHONUTF8": "1", "PATH": __import__("os").environ.get("PATH", "")}
    proc = subprocess.run([sys.executable, str(TEST)], capture_output=True,
                          cwd=str(WS), env=env)
    out = proc.stdout.decode("utf-8", errors="replace")
    return proc.returncode, out


def main() -> None:
    original = sha256(MAIN)
    backup = pathlib.Path(tempfile.mkdtemp(prefix="wb18neg_")) / "main.py.bak"

    section("基线：未改动时主测试必须全绿")
    rc0, out0 = run_test()
    check("N0.1 基线 rc=0（先有绿，才谈得上'拆了会变红'）", rc0 == 0,
          f"rc={rc0}")
    check("N0.2 基线 41/41", "41/41" in out0, (out0.strip().splitlines() or [""])[-1])

    section("变异：拆掉闸门 ⇒ 必须变红，且红在预期的几条")
    try:
        shutil.copy2(MAIN, backup)
        text = MAIN.read_text(encoding="utf-8")
        check("N1.1 变异锚点存在且恰好 1 处（锚点漂移会让变异静默失效）",
              text.count(GATE_LINE) == 1, f"出现次数={text.count(GATE_LINE)}")
        mutated = text.replace(GATE_LINE, GATE_LINE_BROKEN)
        check("N1.2 变异真的改了内容（不是空替换）", mutated != text)
        MAIN.write_text(mutated, encoding="utf-8")
        check("N1.3 落盘后确实是短路状态",
              GATE_LINE_BROKEN in MAIN.read_text(encoding="utf-8"))

        rc1, out1 = run_test()
        check("N2.1 拆闸后 rc!=0（断言真的在测闸门，不是恒真）", rc1 != 0, f"rc={rc1}")
        failed_lines = [ln.strip() for ln in out1.splitlines() if ln.strip().startswith("FAIL")]
        check("N2.2 拆闸后有断言变红（且不是'整篇崩掉'那种红）", len(failed_lines) >= 1,
              f"FAIL 条数={len(failed_lines)}")
        # 关键：红的位置要**落在预期**，否则"红了"也可能是别的原因坏了。
        red_hits = [tag for tag in MUST_RED
                    if any(tag in ln for ln in failed_lines)]
        check("N2.3 预期的那几条确实变红（1.0 纯判定 / 4.1 退出码 / 4.2 未建 bridge / 4.3 报错日志）",
              len(red_hits) == len(MUST_RED), f"命中={red_hits}")
        # 反向：拆闸只应影响"依赖闸门"的断言；文案/缺键/不误伤那几节必须仍然绿。
        still_green = [ln for ln in out1.splitlines()
                       if ln.strip().startswith("PASS") and ("3." in ln or "7." in ln)]
        check("N2.4 拆闸**不影响**文案与不误伤那几节（它们仍绿 ⇒ 分层没串味）",
              len(still_green) >= 6, f"仍绿条数={len(still_green)}")
        print("         └ 拆闸后的 FAIL 行：")
        for ln in failed_lines[:6]:
            print(f"           {ln}")
    finally:
        # ★ 原子还原 + 双重核对（这条路径必须无条件执行，哪怕上面任何一步抛异常）
        shutil.copy2(backup, MAIN)
        restored = sha256(MAIN)
        check("N3.1 **已还原**：sha256 与原始逐位相同（防'改坏不还原'）",
              restored == original, f"{restored[:16]} vs {original[:16]}")
        check("N3.2 还原后锚点回到原状（无残留 `if False`）",
              GATE_LINE_BROKEN not in MAIN.read_text(encoding="utf-8")
              and MAIN.read_text(encoding="utf-8").count(GATE_LINE) == 1)

    section("还原后复跑：必须恢复全绿（证明还原是真的）")
    rc2, out2 = run_test()
    check("N4.1 还原后 rc=0", rc2 == 0, f"rc={rc2}")
    check("N4.2 还原后 41/41", "41/41" in out2,
          (out2.strip().splitlines() or [""])[-1])

    # ----------------------------------------------------------------- #
    section("变异二：改**纯函数**判据（而不是调用点）⇒ 1.0 必须变红")
    # 为什么需要第二种变异：变异一拆的是调用点，纯判定 1.0 仍然绿 ——
    # 那证明不了 1.0 本身在测东西。这里把纯函数的判定条件改坏（去掉 engine 判断），
    # 让"任何引擎都判危险"，则 1.0（危险组合命中）仍绿但 1.1~1.3（负例放行）必红，
    # 且 7.2（8 组合恰好 1 种危险）必红 —— 证明判据的**判别面**也被测到了。
    JUDGE_LINE = '    if engine != GATE_DANGER_ENGINE or not auto_plan or fallback:'
    JUDGE_BROKEN = '    if not auto_plan or fallback:'
    original2 = sha256(MAIN)
    backup2 = pathlib.Path(tempfile.mkdtemp(prefix="wb18neg2_")) / "main.py.bak"
    try:
        shutil.copy2(MAIN, backup2)
        text2 = MAIN.read_text(encoding="utf-8")
        check("N5.1 变异二锚点存在且恰好 1 处",
              text2.count(JUDGE_LINE) == 1, f"次数={text2.count(JUDGE_LINE)}")
        MAIN.write_text(text2.replace(JUDGE_LINE, JUDGE_BROKEN), encoding="utf-8")
        rc3, out3 = run_test()
        failed3 = [ln.strip() for ln in out3.splitlines() if ln.strip().startswith("FAIL ")]
        check("N5.2 改坏纯函数判据 ⇒ rc!=0", rc3 != 0 or len(failed3) > 0,
              f"rc={rc3} FAIL={len(failed3)}")
        # 负例那几条必须红（因为现在"凡 auto_plan=true + fallback=false 都判危险"，
        #   tour 也会被拦 ⇒ 1.1 负例① 必红）
        check("N5.3 负例①（engine=tour 应放行）确实变红 ⇒ 1.x 那组不是恒真",
              any("1.1" in ln for ln in failed3), f"FAIL 示例={failed3[:3]}")
        check("N5.4 §7 判别面断言也变红（8 组合不再'恰好 1 种危险'）",
              any("7.2" in ln for ln in failed3), f"{[l for l in failed3 if '7.' in l][:2]}")
    finally:
        shutil.copy2(backup2, MAIN)
        check("N6.1 变异二**已还原**（sha256 逐位相同）", sha256(MAIN) == original2,
              f"{sha256(MAIN)[:16]} vs {original2[:16]}")

    section("最终复跑：两个变异都还原后必须全绿")
    rc4, out4 = run_test()
    check("N7.1 最终 rc=0", rc4 == 0, f"rc={rc4}")
    check("N7.2 最终 41/41", "41/41" in out4,
          (out4.strip().splitlines() or [""])[-1])

    finish()


if __name__ == "__main__":
    main()
