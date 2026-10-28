---
来源: _tmp_session/vtest-1025/CONTRACT.md；docs/00_项目总览/交接_20261024_空地协同接手.md；docs/20_资料索引库/空地链路/05_步进协同与空地通信_实测复盘_20261024.md
取证: 实/档（侦察于 2026-10-25 14:1x，逐条带路径）
更新: 2026-10-25
status: in_progress
ns: vtest-1025
agent: rec-metrics
树节点: VT-5
写域: _tmp_session/vtest-1025/rec/**、runs/*/rec_*.jsonl 本卡
摘要: 双向记录与指标：机端两个话题全量落 JSONL（含单调时钟），车端 robot_logs 拉取，分析器出感知/链路/车端三层指标，附数据字典与报告模板。
---

# VT-5-REC-METRICS-20261025

## 1 五必填

- **读过的台账**：D-015（段边界等 layout ≤10 s）· D-022（布局过期只拦新规划）· D-029（不改 ticksPerMeter，先量）· D-031（靶速封死 1.0 m/s）· D-038/D-039（`field_layout` 唯一构造器 = `build_layout`，发射端必须 import）。
- **复用检索**：见本卡 §2；新增文件一律落 `_tmp_session/vtest-1025/`，**不注册进 复用清单**（过程件），进 `tools/` 才登记。
- **新增轮子及理由**：见 §2 末行"为什么现有不够"。
- **验收命令**：见本卡 §4。
- **回滚点**：本卡只写 `_tmp_session/vtest-1025/`；删目录即回滚，不动仓库。

## 2 事实基线

- 话题：`/dzlink/drone_to_car`、`/dzlink/car_to_drone`（std_msgs/String，BEST_EFFORT）；domain 118。
- 车端日志：`/home/duzhong/robot_logs/{7_path,3_localization,1_chassis,5_gimbal,6_judgment}.log`，**每次起栈覆盖**（05 复盘 §9 有 scp 命令样例）。
- 车端关键事件行（用于分析）：`第 N 份布局`、`layout_stale`、`段 下发/完成/重规划`、`绕圈兜底`、`吸附被拒`、`done_seq`、`SAFE_STOP`、`到点 x/y`。
- 已有证据目录：`_tmp_session/agc-1024/s2run/evidence/`（含 `_tmp_session/agc-1024/s2run/evidence/7_path.log` 598 行、`events.txt`、`loc_an*.py`）——**用它做分析器回归样本**，别拿它当本次结果。

**为什么现有不够**：仓内没有跨话题的统一记录器；`dzlink_supervisor.jsonl` 只记 supervisor 自身；车端 CSV 不含机端视角。

## 3 交付物

1. `rec/recorder.py`（机端跑，rclpy）：订阅两话题 → `runs/<run_id>/rec.jsonl`，每行 `{t_mono,t_wall,topic,seq,raw,parsed:{type,version,target_cell,known_n},bytes}`；坏 JSON 也落（`parse_error`）；1 Hz 打 `topic hz` 与丢包估算。
2. `rec/collect_car.sh`（ASCII）：scp 拉 `robot_logs` 到 `runs/<run_id>/car/`（目标已存在则加时间戳后缀，不覆盖）。
3. `_tmp_session/vtest-1025/rec/analyze.py`：输入 `runs/<run_id>/` → `metrics.json` + `report.md`〔未实现〕（骨架）。四组指标：
   - 感知（有 det.jsonl 时）：每类 P/R、格命中率、known 覆盖、虚警/漏检清单；
   - 情报：版本序列/间隔、target_cell 轨迹、known 增长、布局字段完整性、桥洞来源（箭头 or 缺）；
   - 链路：条数/秒、坏帧、时延（`t_recv - t_emit` 按发射器 ts 对齐）、丢包（seq 缺口）；
   - 车端：布局份数、`layout_stale` 次数与时长、段下发/完成、到点率、强跳、SAFE_STOP、段位移与耗时。
4. `rec/DATA_DICT.md`：每个字段含义、单位、来源（哪一行代码/日志）。
5. `rec/REPORT_TEMPLATE.md`：一页结论式模板（结论/数字表/失败归因/下一步），供 Lead 最终成文。

## 4 验收

用 s2run 的 `_tmp_session/agc-1024/s2run/evidence/7_path.log` 跑 `analyze.py --fake-legacy`，能复现 05 复盘的关键数字（段 4/4、到点 11/12 与 7/8、layout_stale 11.2 s、桥洞情报 0 条），误差在四舍五入内；`recorder.py` 在 Windows 无 rclpy 时给明确报错而不是崩。

## 5 边界

不解析 dzuav 私有 UDP（那是 VT-2/VT-3 的事）；不修改车端日志文件；不把 rec.jsonl 提交进 git。

## 5 禁止项

冻结区、`git add .`、设备写操作、非 ASCII 的机端 .sh 一律禁止；设备当前不可达（车 192.168.0.4 / 机 192.168.0.5 均 100% 丢包，2026-10-25 14:1x 实测）。

## 6 文档变更

无（本卡 + 树 + CONTRACT 均为过程件；结论成熟后再由 Lead 提炼进 `docs/20_资料索引库/空地链路/`）。
