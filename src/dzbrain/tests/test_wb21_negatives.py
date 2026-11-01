# -*- coding: utf-8 -*-
"""WB-21 变异自证：**把 bench 空路线闸门拆掉，测试必须变红**（L16）。

## 为什么必须有（L16 · captain 2026-11-01 立，源于我在 WB-18 踩的坑）
`test_wb21_silentempty.py` 26 条全绿，只证明"闸门在时断言成立"，**证明不了断言真在测闸门**。
唯一能证的办法：把被测机制拆掉，看断言是否变红，且**红在对的位置**。

## L16 的四条硬要求（本文件逐条落实）
1. `try/finally` —— 还原无条件执行（哪怕中途抛异常/编码出错）
2. **双重 sha256** —— 植入前记一次、还原后比一次（"还原了"由独立量证明）
3. 子进程**显式 `encoding`/`errors`**（Windows 控制台默认 GBK 会把中文输出读炸，
   我在 WB-18 正是因此把还原语句跳过 ⇒ 文件停在短路状态）
4. 变异**锚点唯一性**先断言（锚点漂移会让变异静默失效 ⇒ 假绿）

跑法（工作区根 `dzrobot_v2/`）：
    PYTHONPATH=src/dzbrain python src/dzbrain/tests/test_wb21_negatives.py
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
TEST = WS / "src" / "dzbrain" / "tests" / "test_wb21_silentempty.py"

#: 变异目标：把 bench 闸门的条件改成恒假（= 拆掉闸门）。
GATE_LINE = "    if len(_route_tokens) < 2:"
GATE_BROKEN = "    if False and len(_route_tokens) < 2:"

#: 拆闸后**必须变红**的断言（= 真正在测闸门的那几条）。
#: 刻意**不含** §2/§6：那些测的是"下游行为/写域边界"，与闸门存不存在无关。
#: ★ 5b.3/5b.5 是 captain 2026-11-01 要求补的「精确边界」断言 —— 实测拆闸后它们也变红
#:   （闸门没了 ⇒ 情形 D 退化成裸 Traceback）。
MUST_RED = ("3.1", "3.2", "4.3", "4.4", "4.5", "5b.3", "5b.5")
#: 拆闸后**必须仍绿**的（正常路径 + 机制事实，与闸门存在与否无关）。
#: 这一组是"分类正确性"的证据：新增的 5b 断言没有把正常路径也牵连进去。
MUST_STAY_GREEN = ("5.1", "5.2", "5.3", "5.4", "5b.1", "5b.2")


def sha256(p: pathlib.Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def run_test() -> tuple[int, str]:
    """跑主测试。★ 显式 encoding（L16 第 3 条）——Windows 默认 GBK 会在中文输出上炸。"""
    env = {"PYTHONPATH": str(WS / "src" / "dzbrain"), "PYTHONIOENCODING": "utf-8",
           "PYTHONUTF8": "1", "PATH": __import__("os").environ.get("PATH", "")}
    proc = subprocess.run([sys.executable, str(TEST)], capture_output=True,
                          cwd=str(WS), env=env)
    return proc.returncode, proc.stdout.decode("utf-8", errors="replace")


def main() -> None:
    original = sha256(MAIN)
    backup = pathlib.Path(tempfile.mkdtemp(prefix="wb21neg_")) / "main.py.bak"

    section("基线：未改动时主测试必须全绿")
    rc0, out0 = run_test()
    check("N0.1 基线 rc=0（先有绿，才谈得上'拆了会变红'）", rc0 == 0, f"rc={rc0}")
    check("N0.2 基线 31/31（含新增 5b 五条）", "31/31" in out0,
          (out0.strip().splitlines() or [""])[-1])

    section("变异：拆掉闸门 ⇒ 必须变红，且红在预期位置")
    try:
        shutil.copy2(MAIN, backup)
        text = MAIN.read_text(encoding="utf-8")
        check("N1.1 变异锚点存在且**恰好 1 处**（锚点漂移会让变异静默失效 ⇒ 假绿）",
              text.count(GATE_LINE) == 1, f"出现次数={text.count(GATE_LINE)}")
        mutated = text.replace(GATE_LINE, GATE_BROKEN)
        check("N1.2 变异真的改了内容（不是空替换）", mutated != text)
        MAIN.write_text(mutated, encoding="utf-8")
        check("N1.3 落盘后确实是拆闸状态",
              GATE_BROKEN in MAIN.read_text(encoding="utf-8"))

        rc1, out1 = run_test()
        failed = [ln.strip() for ln in out1.splitlines() if ln.strip().startswith("FAIL")]
        check("N2.1 拆闸后 rc!=0（断言真的在测闸门，不是恒真）", rc1 != 0, f"rc={rc1}")
        check("N2.2 拆闸后有断言变红（且不是整篇崩掉那种红）", len(failed) >= 1,
              f"FAIL 条数={len(failed)}")
        red_hits = [t for t in MUST_RED if any(t in ln for ln in failed)]
        check("N2.3 预期的那几条确实变红（3.1 rc / 3.2 点名 / 4.3-4.5 三态 / 5b.3+5b.5 边界）",
              len(red_hits) == len(MUST_RED), f"命中={red_hits}")
        # 反向：**正常路径那几条不许红** —— 若它们也红，说明断言互相耦合（一刀切全红也是坏信号）。
        wrongly_red = [t for t in MUST_STAY_GREEN if any(t in ln for ln in failed)]
        check("N2.3b 正常路径/机制事实的那几条**不许被牵连变红**（分类正确性）",
              wrongly_red == [], f"错误变红={wrongly_red}")
        # 反向：拆闸**不该**影响 §2（下游行为）与 §6（写域边界）—— 那些与闸门无关。
        still = [ln for ln in out1.splitlines()
                 if ln.strip().startswith("PASS") and (" 2." in ln or " 6." in ln)]
        check("N2.4 拆闸**不影响** §2/§6（它们仍绿 ⇒ 判据分层没串味）",
              len(still) >= 6, f"仍绿条数={len(still)}")
        print("         └ 拆闸后的 FAIL 行：")
        for ln in failed[:5]:
            print(f"           {ln}")
    finally:
        # L16 第 1/2 条：无条件还原 + 双重 sha256
        shutil.copy2(backup, MAIN)
        restored = sha256(MAIN)
        check("N3.1 **已还原**：sha256 与原始逐位相同（防'改坏不还原'）",
              restored == original, f"{restored[:16]}… vs {original[:16]}…")
        check("N3.2 还原后无残留拆闸标记",
              GATE_BROKEN not in MAIN.read_text(encoding="utf-8")
              and MAIN.read_text(encoding="utf-8").count(GATE_LINE) == 1)

    section("还原后复跑：必须恢复全绿（证明还原是真的）")
    rc2, out2 = run_test()
    check("N4.1 还原后 rc=0", rc2 == 0, f"rc={rc2}")
    check("N4.2 还原后 31/31（含新增 5b 五条）", "31/31" in out2,
          (out2.strip().splitlines() or [""])[-1])

    finish()


if __name__ == "__main__":
    main()
