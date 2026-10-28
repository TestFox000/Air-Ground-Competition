---
来源: _tmp_session/vtest-1025/CONTRACT.md；docs/00_项目总览/交接_20261024_空地协同接手.md；docs/20_资料索引库/空地链路/05_步进协同与空地通信_实测复盘_20261024.md
取证: 实/档（侦察于 2026-10-25 14:1x，逐条带路径）
更新: 2026-10-25
status: in_progress
ns: vtest-1025
agent: layout-bridge
树节点: VT-4
写域: _tmp_session/vtest-1025/emit/** 本卡
摘要: 检测流→field_layout v2 发射器：把 det.jsonl（或 UDP 实时）聚合成 known/雷/障/靶/桥，复用 build_layout + choose_mission_target，用「版本即心跳」绕开车端 TTL 缺陷、用「目标格排除当前格」绕开同格空转。
---

# VT-4-LAYOUT-BRIDGE-20261025

## 1 五必填

- **读过的台账**：D-015（段边界等 layout ≤10 s）· D-022（布局过期只拦新规划）· D-029（不改 ticksPerMeter，先量）· D-031（靶速封死 1.0 m/s）· D-038/D-039（`field_layout` 唯一构造器 = `build_layout`，发射端必须 import）。
- **复用检索**：见本卡 §2；新增文件一律落 `_tmp_session/vtest-1025/`，**不注册进 复用清单**（过程件），进 `tools/` 才登记。
- **新增轮子及理由**：见 §2 末行"为什么现有不够"。
- **验收命令**：见本卡 §4。
- **回滚点**：本卡只写 `_tmp_session/vtest-1025/`；删目录即回滚，不动仓库。

## 2 事实基线

- `build_layout(truth, known, *, version, ts, car_block, segment_id, target_cell, covered_mines, done_stops, cfg)` → 15 字段 dict（`scout/layout.py:467-500`）；`bridges` 取 `truth.bridges` 全集，不 ∩ known（`:497`）。
- 车端 StepLoop：`version>seen` 才刷新 `_layout_rx_at`（`main.py:844-854`），`step.layout_ttl_s=10`；`target_cell==car_block` 时车记"同格空段"并报 done（`:888-899`）⇒ 机端照做会无限推版本（05 复盘 P2）。
- 现成发射器 `_tmp_session/agc-1024/s2run/emit_step.py`（163 行）是**真值驱动**：known 按 10 批定时灌。本卡要的是**检测驱动**，两者共用 `choose_mission_target`/`build_layout`。
- 线上：`std_msgs/String` JSON，BEST_EFFORT depth 10，domain 118；`car_to_drone` 收 `car_state.done{seq,block,stop_kind}`。

**为什么现有不够**：`emit_step.py` 输入是"内置真值图 + 定时批次"，无法消费真实检测；`LayoutStream.feed()` 只吃 `cells`，不含目标检测类别与箭头。

## 3 交付物

1. `emit/emit_video.py`：
   - 输入二选一：`--det-jsonl`（离线回放，按 `t_video` 节拍）或 `--udp 50000`（实时，复用 `dzudp_core.parse_intel/parse_pose`）；
   - `--known-mode fov|full|none`（fov=按 pnp 位姿 2×4 足迹；full=整场可见时全 154；none=只报检测格并**显式标注**未知）；
   - 聚合：`cell+cls` 多帧投票（默认 conf≥0.35、≥2 帧）；`bridges` 只来自 `arrow` 落在 K 列的格，判不出就整键不发；
   - 版本纪律：内容变**必**换版；无变化时每 4 s 换版（心跳）；`target_cell != car_block`（end 例外）；
   - 收 `car_state.done` → 记 covered_mines/done_stops → 重算目标 → 立即发新版（复用 `choose_mission_target`）；
   - `--dry`：不发 DDS，只把 layout.jsonl 写出来（Windows 可跑，无 rclpy）。
2. `emit/test_emit_video.py`：合成 det.jsonl 的 12+ 断言（含：TTL 对齐、目标不落当前格、箭头→bridges、缺箭头不发键、坏行跳过不崩）。
3. `emit/DRYRUN.md`：用 VT-3 的 det.jsonl 跑一遍，打印 version 序列 + target_cell 序列 + known 增长曲线。

## 4 验收

`python emit_video.py --det-jsonl <VT3产物> --dry --run-id VT1025A` exit 0；`layout.jsonl` 每行可被 `dzbrain.tasks.drone` 的校验器吃下（用仓内 `test_drone` 的 mirror 解析函数做离线校验）；连续 60 s 不产生"同格 target"。

## 5 边界

不改车端冻结代码（`main.py` 不碰）；不重写 `build_layout`；单写者：本发射器与机端 `dzbrain_bridge`/真值发射器**不得同时在线**（起前跑 `dzudp_adapter.py --selfcheck`）。

## 5 禁止项

冻结区、`git add .`、设备写操作、非 ASCII 的机端 .sh 一律禁止；设备当前不可达（车 192.168.0.4 / 机 192.168.0.5 均 100% 丢包，2026-10-25 14:1x 实测）。

## 6 文档变更

无（本卡 + 树 + CONTRACT 均为过程件；结论成熟后再由 Lead 提炼进 `docs/20_资料索引库/空地链路/`）。
