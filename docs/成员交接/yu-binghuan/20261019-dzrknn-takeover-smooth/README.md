# dzrknn 全盘接管：卡顿根因修复与平滑跟踪律上线

- 归档日期：2026-10-19
- 归档目录：`docs/成员交接/yu-binghuan/20261019-dzrknn-takeover-smooth/`
- 归档工具：`push_deliverable.py`（一键归档，见同目录说明）

## 说明

全盘接管 dzrknn_dataprocess：12s 三路探针量化定位卡顿=冻结态（585 条指令 584 条相同，12s 零变化）；根因=18:52 代 enterScanPhase 内'已锁定就 return'守卫导致永钉 TRACK（19:19 代已挪位，接管确认）。接管 4 处外科手术：单写者原则（smooth_track 启用时旧律整段跳过，消除 30Hz 台阶对 100Hz 平滑斜坡的覆盖打架）、TRACK 进入/停止请求双点作废 scan_homing_（堵停止后自动开扫怪象）、smoothTrackStep 接入 gyro_ff 通道（防开平滑后前馈静默丢失）。验证门全过：离线回归 267ms 延迟无发散、指令跳变 4 倍平滑；实车编译部署横幅确认；状态机活性全绿（归位 2 tick 到位、真扫掠、5 帧类别锁、停止不自启）；合成靶台架指令冻结 12000ms→150ms（80 倍）、指令流 93Hz 连续。剩余现场项：真靶闭环复测、gyro_ff 符号标定后开 0.8、laser_offset 标定（工具在车）。

## 文件清单

| 文件 | 大小 |
|---|---|
| `deliverable_takeover_20261019/aim_probe.csv` | 47,531 B |
| `deliverable_takeover_20261019/analyze_synth.py` | 2,903 B |
| `deliverable_takeover_20261019/dzrknn_dataprocess.cpp` | 98,243 B |
| `deliverable_takeover_20261019/README.md` | 4,921 B |
| `deliverable_takeover_20261019/sim_laws.py` | 5,703 B |
| `deliverable_takeover_20261019/synth_bench.csv` | 68,817 B |

## 复现方式

```bash
# 视频侦察管线（Windows 可直接跑，无需 ROS）
python video_field_recon.py <video.mp4> --stride 15 --out <输出目录>
python render_waypoint_map.py <输出目录>
```

> 本 README 由 push_deliverable.py 自动生成。