# ag_orbit_target — 绕靶录像采集（支线工具）

订阅官方 det_node 的 `/offset_center` 检测结果，发 `/cmd_vel` 做 搜索→接近→绕行 控制，
同时对 `/image_raw` 全程录像，用于采集训练数据。**不在现役主链路中**。

## 重要警告

**勿与官方 dzrobot / path_executor 同时运行**：本节点与它们都会向 `/cmd_vel`
发布指令，同跑会争夺底盘控制权，车辆行为不可预期。采集前先确认官方链路已停。

## 用法

```bash
# 安装进工作区后（ament_python 包）
ros2 launch ag_orbit_target orbit.launch.py           # 默认 enable=false，只录像不动车
ros2 launch ag_orbit_target orbit.launch.py enable:=true   # 确认安全后启用底盘控制
```

参数见 `config/orbit_params.yaml`。关键项：

- `enable`（默认 `false`）：安全闸。未启用时**不发布任何 /cmd_vel**，节点只做
  检测订阅、状态机日志与录像，日志周期提示等待启用；与主仓库 dry_run（只算不发）哲学一致。
- `record_dir` / `record_fps`：录像输出目录与帧率，mp4 文件叠加状态信息。
- `det_freshness`：目标新鲜度（秒），超时视为无目标。
- `max_runtime` / `search_timeout` / `lost_timeout`：各类超时保护，超时进 STOP 停车。

录像文件保存在 `record_dir`（默认 `~/orbit_videos`），文件名 `orbit_时间戳.mp4`。
无 cv_bridge/cv2 环境时录像自动关闭，节点其余功能不受影响。
