# 车端部署核验与日志复盘（P0-1 补丁上机确认）

- 归档日期：2026-10-19
- 归档目录：`docs/成员交接/yu-binghuan/20261019-car-deploy-verify/`
- 归档工具：`push_deliverable.py`（一键归档，见同目录说明）

## 说明

车 192.168.0.197 经 MiFi 设备 API 自动发现并接入；P0-1 提前量进控制环补丁经四位一体核验已在车端编译运行（源码 9ea10779 与 _cloud/patch 逐行一致，二进制 18:32:42 产出，进程 18:32:54 启动）；P0-2 参数对齐落地（max_shots 10/两键开/lead_gain 1.0/护栏 300ms·60px·4px）；昨日日志复盘结论：F9 零发主因为旧码类别锁清零+红靶门控（t42 25 次诊断），两轮静止靶 10 发全打出但 0 中 ⇒ laser_offset 系统偏差为命中率主杀手（P1-1 下一优先级）；含部署真源三件套（cpp/.h/robot.yaml）、关键日志与 car.cjs 车端自动化 SSH 工具；P0-3 仓库收口与现场标定项见 README 清单。

## 文件清单

| 文件 | 大小 |
|---|---|
| `deliverable_20261019/5_gimbal.log` | 3,193 B |
| `deliverable_20261019/7_path.log` | 11,579 B |
| `deliverable_20261019/car.cjs` | 5,929 B |
| `deliverable_20261019/dzrknn_dataprocess.cpp` | 88,322 B |
| `deliverable_20261019/dzrknn_dataprocess.h` | 24,784 B |
| `deliverable_20261019/gimtest_5_gimbal.log` | 9,909 B |
| `deliverable_20261019/kb11_5_gimbal.log` | 3,717 B |
| `deliverable_20261019/README.md` | 9,435 B |
| `deliverable_20261019/robot.yaml` | 76,737 B |

## 复现方式

```bash
# 视频侦察管线（Windows 可直接跑，无需 ROS）
python video_field_recon.py <video.mp4> --stride 15 --out <输出目录>
python render_waypoint_map.py <输出目录>
```

> 本 README 由 push_deliverable.py 自动生成。