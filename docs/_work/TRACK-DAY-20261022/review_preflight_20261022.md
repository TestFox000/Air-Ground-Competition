# R3 复核 · 2026-10-22 · 结论判定

> 复核者：r3-reviewer（只读；**未连车、未 SSH、未发令**）  
> 复核对象：`_tmp_session/PREFLIGHT-20261022/preflight.sh`（7505 B，sha256 `b99988cc1dd0b9cb064dd275f1724814e8789ddc60c8d32010e049167b9e42bf`，与 `PREFLIGHT-20261022/REPORT.md:7` 一致）、`_work_tmp_d11_session.sh`（1355 B，10-22 18:33 版）、`docs/30_任务卡库/在办/TRACK-FIELD-D11-20261022.md:78-79` 的两条根因  
> 结论：**四条全部采纳**（2 条根因成立；A/B/C 为风险，需按修法落地）；两点路线语法/语义已独立核过（附 §6）。  
> 合规：`docs/_工具/doc_policy.json` 的 `exempt_prefixes` 含 `docs/_work/`，本文件属过程输出，不入库索引。

## 0. 身份与边界

| 项 | 值 |
|---|---|
| preflight.sh 字节/哈希 | 7505 B / `b99988cc…9e42bf`（本 reviewer 自算，与 REPORT:7 一致） |
| 只读声明 | 脚本只含 `topic list/hz/echo/info`、`node list`、`pgrep`、`grep`、`tail`；**无** `topic pub` / `action send` / `service call`（第 40/53/62/82/90/92/103/109 行全是只读调用） |
| 未做 | 未连车；车端事实以 `PREFLIGHT-20261022/REPORT.md` 的 20:34:58 输出与 `evidence/d11_1900/` 日志为准 |

## 1. 根因①（成立）：单点路线结构上永远没有停站点

- `dzrobot_v2/src/dzbrain/dzbrain/tasks/mission.py:2879`：`self._path = Path(route) if len(route) >= 2 else None`
- `mission.py:2899-2905`：`self._stop_s = {}`；只有 `if self._path is not None:` 才填 `_stop_s`（`route.stop_points()` + 末点），`_stop_blocks` 又由 `_stop_s` 排序而来
- ⇒ 1 航点（`单点 D11*,x`）：`_path=None` ⇒ `_stop_s={}` ⇒ `_classify_stops` 空 ⇒ `*`（shoot）被丢弃，日志打「停站点 0 个（无）」，`fire_gate` 恒 0
- 交叉证据：`mission.py:2853` 自身日志就写「单点不编译 Path；Tracker 侧投影不可用，只用于『停在该点』类调试」
- 判定：D11 卡 `TRACK-FIELD-D11-20261022.md:78` 的根因①**成立**；不是「被判已到终点」

## 2. 根因②（成立）：Safety 锁存不自动恢复，且无外部 clear 入口

- `dzrobot_v2/src/dzbrain/dzbrain/tasks/localization.py:744-760`：航向守卫闩锁后，连续 `yaw_clear_ticks`（默认 20 个新帧）全可信会自动复位（`689-692` 注释：复位只清守卫，Safety 另清）
- `dzrobot_v2/src/dzbrain/dzbrain/tasks/safety.py:20-22`：恢复语义「**刻意不做自动恢复**，必须显式 `clear_safety()`」
- `safety.py:190` 的 `clear_safety()` 只有内部调用；全仓 grep `clear_safety` 只出现在 `tasks/safety.py`、`tasks/localization.py`（注释）与 `tests/`；`main.py` 无 clear 调用 ⇒ **没有 ROS 话题/服务入口**
- ⇒ 车端一旦 SAFE_STOP，只能重启 dzbrain（D11 卡 `:79` 的记录成立）
- 附带事实（风险 B 的根据）：`dzrobot_v2/src/dzbrain/dzbrain/tasks/telemetry.py:90` 的 `[T]` 行只会出现 `安全OK` 或 `安全STOP` **之一**

## 3. 风险 A（需修）：`_work_tmp_d11_session.sh` 命令与日志路径双错

| # | 现状 | 后果 |
|---|---|---|
| A1 命令 | `_work_tmp_d11_session.sh:27` 仍发 `单点 D11*,x` | 按 §1 **必复现**「停站点 0 个」，不可能开火 |
| A2 日志 | `:5` `LOG=$R3/logs`（`/home/duzhong/r3_field_20261021/logs`）、`:33` tail `$LOG/7_dzbrain.log`；而 `dzrobot_v2/start_all.sh:18` 是 `LOGDIR=$HOME/robot_logs`（`~/robot_logs`） | 复用旧 helper 会 tail 到 **10-21 的 R3 旧日志**，排障看错轮次 |

- 判定：**成立**；两点 helper 落地时「命令 + LOG 路径」必须同时改（或显式传日志根目录）。

## 4. 风险 B（需修）：预检的 SAFETY_LINE 会打印历史 OK 行，且不 gate

- `_tmp_session/PREFLIGHT-20261022/preflight.sh:111`：`SAFETY_LINE=$(grep '安全OK' /home/duzhong/robot_logs/7_path.log | tail -1)`
- 当前态只有 `安全OK`/`安全STOP` 两种写法（`telemetry.py:90`）；SAFE_STOP 时最后一条 `[T]` 是 `安全STOP`，`grep 安全OK` 会**回退到历史 OK 行**并打印成「banner 安全OK（最后一条）」
- 实证：`PREFLIGHT-20261022/REPORT.md:35` 打印的 `[T] N10 IDLE … 弹10` 是 **10-21 R3 残留**（10-22 车在 D11、弹9；见 `evidence/d11_1900/7_path_diag.log` 尾行 `[T] D11 … 弹9`）⇒ 证据本身已中招
- 另外：该行只是 `[INFO]`，不进退出码 ⇒ 即使真 SAFE_STOP，预检仍可能 6/6 PASS，白跑一趟 30 s
- 修法：取 `grep '\[T\]' 7_path.log | tail -1` 整行；含 `安全STOP` ⇒ FAIL（并打印该行）；建议第 5 项（`preflight.sh:100-129`）同时断言 `/pursuitAngle` 的 `Publisher count == 1`（脑进程活但 Tracker 挂时，现在仍 PASS）
- 判定：**成立**

## 5. 风险 C（待车端一行确认）：`/image_raw` 的 hz 走默认 RELIABLE 订阅

- `preflight.sh:53`：`ros2 topic hz /image_raw`（`:62` `/offset_center`、`:90` `/HpAndHitmsg` 同）
- `REPORT.md:31` 的 FAIL 只覆盖 **`device missing`** 分支；**相机在线时 hz 从未被验证**
- 若 usb_cam 的 `/image_raw` 是 BEST_EFFORT，可靠订阅收不到 ⇒ `0 Hz` **假 FAIL**（与 R3 的 `/pursuitAngle` 同型坑）
- 修法：先 `ros2 topic info -v /image_raw` 看 Reliability；best_effort 时改用显式 best_effort 订阅（如 `ros2 topic echo --once --qos-reliability best_effort` 计数 / 自写 python），不能直接信 `hz=0`
- 判定：**成立（待确认 QoS）**

## 6. 附：两点路线（E11→D11*）语法/语义已核（供新 helper）

- `dzrobot_v2/src/dzbrain/dzbrain/route/model.py:100-101`：`stop_points()` = `shoot or scan` 的航点
- `mission.py:2936-2951`：末点**带 `*`** 时记 `SHOOT`（T32 例外），不带 `*` 才是 `END`
- `mission.py:3346-3366`：`_next_stop_candidate()` 允许末点 `*` 进三段式停站（打完回 DRIVE，再判 FINISH）
- ⇒ 2 航点路线 + 末点 `*` 可正常停站打靶；命令形态：`reset:E11,x` → `多点 E11 D11*,x`（车须先物理退到 E11 并重置 belief）
- 注意：`*` 只能挂**最后一个** token（`parse_mission` 约束）；1 航点/无 `*` 仍无停站点

## 7. 结论

- 四条**全部采纳**：§1/§2 根因成立；§3/§4/§5 为风险，按各自「修法」落地。
- 两点路线 / 新 helper 落盘后，本 reviewer 先做静态审（核对 A1/A2/B/C 修正是否落地），过了再谈发车；**今晚车在跑自瞄实验，不发车**。
- 未验证项：车端 usb_cam QoS；两点路线实跑；新 helper 尚未落盘。
