# 相机可用性核查（SmartScout 无人机）

- 核查时间：2026-10-07 15:2x
- 对象：`dzsmart.local`（`10.158.138.104`）— NVIDIA **Jetson Orin Nano Developer Kit**，`Linux dzsmart 5.15.185-tegra`，L4T **R36.5.0**，ROS 2 **Humble**（`/opt/ros/humble`）
- 方式：**只读** SSH（`-4`，`tools/drone/known_hosts_dz`，`BatchMode`）。机上**未创建/修改/删除任何文件**，未装包，未改配置（遵守 `WORKFLOW.md`）。
- 一句话结论：**机上没有相机硬件；相机软件栈已齐备；`gst_camera` 是空壳。**

---

## 一、硬事实（实测命令 → 输出）

| 检查项 | 命令 | 结果 |
|---|---|---|
| 视频节点 | `ls /dev/video*` | **无**（`NO /dev/video*`） |
| USB 设备 | `lsusb` | 仅 `2109:0822` / `2109:2822` VIA hub ×2、`0bda:c822` Realtek 蓝牙；**无任何 UVC/相机** |
| V4L2 设备名 | `for f in /sys/class/video4linux/*/name` | **空**（无 video4linux 实例） |
| GStreamer 源枚举 | `gst-device-monitor-1.0 Video` | `Probing devices...` 后**无任何设备** |
| 设备树传感器节点 | `ls /proc/device-tree \| grep -iE 'imx\|ov[0-9]\|cam\|sensor'` | 仅标准 `camera-ivc-channels`；**无 imx/ov 传感器节点** |
| CSI I2C 扫描 | `i2cdetect -y -r 9` | `Error: Bus doesn't support detection commands`（Tegra I2C 不支持 quick-write，此路不可判；`/dev/i2c-10` 不存在） |
| 媒体控制器 | `ls -l /dev/media*` | `/dev/media0` **存在**（Tegra VI 媒体图，无相机也建） |
| Argus 服务 | `pgrep -af nvargus` | `nvargus-daemon` **在跑** ⇒ **有服务、无传感器** |
| 内核日志 | `journalctl -b -0 \| grep -iE 'imx\|ov[0-9]\|camera\|camrtc\|vi-output'` | 只有 `nvargus-daemon` 启动行 + systemd 的 `gvfs` 相机监视服务；**无传感器 probe 记录** |
| 用户权限 | `id` | `video`(44)、`i2c`(116)、`render`(104)、`gpio`(999)、`docker`(994) ⇒ **权限不缺** |
| 相机包骨架 | `find ~/dzpx4/src/gst_camera` | **5 个目录、0 个文件**（`config/ launch/ src/ include/gst_camera/`） |

对照片源 `tools/drone/cam_detect.sh`（专为"刚插上相机"设计的检测脚本）——**当前跑它只会输出 `NONE <-- camera not detected`**。

---

## 二、软件底座（全部已装，**不需要 apt、不需要 colcon**）

| 组件 | 状态 |
|---|---|
| `v4l2_camera`（ROS 2 官方 V4L2/UVC 驱动，已在 `ros2 pkg list` 中） | ✅ 已装 |
| `cv_bridge`、`image_transport`、`image_tools`（含 `cam2image`/`showimage`）、`rqt_image_view`、`camera_info_manager`、`camera_calibration_parsers`、`image_geometry`、`depthimage_to_laserscan` | ✅ 已装 |
| GStreamer 1.20.3 + `python3-gi` 绑定（`gi.repository.Gst` 可用） | ✅ |
| 插件：`v4l2src` `jpegenc` `jpegdec` `videoconvert` `nvvidconv` `nvjpegenc` `appsink` `xvimagesink` | ✅ 全部 OK |
| numpy 1.21.5 / OpenCV 4.5.4 | ✅ |
| 资源 | `/` 233G 中 **194G 可用**；内存 **5.4Gi 可用** |

**⇒ 插上一个 USB(UVC) 相机后，一条命令即可出图，零编译：**

```bash
source /opt/ros/humble/setup.bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/video0"
ros2 topic hz /image_raw          # 只读验证：应有稳定帧率
```

---

## 三、手册侧旁证（厂商从未描述相机）

`_pdf_text/SmartScout-AR5_高级功能用户手册202609.txt`、`_pdf_text/SmartScout-AR250_快速使用与安全指南v3(1).txt` 全文中"相机/摄像头/图传/云台/视频/camera"的命中仅三处：

| 位置 | 原文 | 判读 |
|---|---|---|
| AR5 手册 L112 | 「将显示器连接到机载电脑的**视频输出**接口」 | HDMI 出图（接显示器），**不是相机** |
| AR250 指南 L370 / AR5 手册 L705 | 「记录…必要的**照片或视频**，并联系售后」 | 故障报修取证 |

**⇒ 厂商定义的平台是雷达中心**（Livox MID-360 + FAST-LIO + PX4 + 路径录制/跟随），工作流里**没有相机**。

（`~/dzpx4/src` 里唯一的 camera 命中来自第三方 `mavros` 自带的 `optical_flow.cpp`/`px4flow.cpp` —— PX4 光流传感器驱动，**读不了图案，与"读箭头"无关**。）

---

## 四、对比赛规则的影响（规则第 11 条，p8）

> 「地面机器人由于视角限制无法直接识别桥梁顶部设置对应标识，**必须由无人机完成高空侦察**」

- 现状：**无法满足**。没有相机 ⇒ 读不了桥洞顶部绿箭头 ⇒ 缺少"通道指令"这一环（而它是空地协同的分工起点）。
- 需要补的是**硬件（相机）**；软件侧由"已装组件"或"本地新写的节点"补上即可，**不需要动飞控/雷达链路**。

---

## 五、可选路线与代价

| 路线 | 你要做的 | 我要做的 | 代价/风险 |
|---|---|---|---|
| **A. USB(UVC) 相机** | 插到 Jetson 的 USB 口（上电即 `/dev/video0`） | 只读复验 + 出图验证（`v4l2_camera` → `/image_raw` 帧率/分辨率） | **最低**：零安装、零编译。适用性最好 |
| **B. CSI 相机**（IMX219/IMX477 等） | 接 FPC 排线 + `jetson-io` 配设备树 overlay（**改配置，超出助手只读权限**） | 若需 Argus 通路，本地写 `gst_camera`/`gscam` 类节点 | 中：改配置 + 可能需要新节点；`v4l2_camera` 对部分 CSI 驱动不适用 |
| **C. `gst_camera` 补包** | 按 `APPLY.md` 覆盖文件 + `colcon build --packages-select gst_camera` | 本地产出：GStreamer(`nvarguscamerasrc`/`v4l2src`) → `sensor_msgs/Image`，含 QoS/帧率参数、`APPLY.md`、回退说明 | 仅 B/特殊管线才需要；A 路线可完全跳过 |

**建议**：先走 A（最省事、当天可验证）。选相机时优先 **UVC 免驱**、全局快门（拍箭头贴纸不糊）型号；小车那台 `32e4:0234` LRCP AR0234 就是 UVC，插上即认（**注意那是小车的在役相机，不要随意挪用**）。

---

## 六、附带发现（本主题之外，但影响面更大）

无人机 `ros2 topic list` 出现 24 个话题，其中 `/dzactuator`（含 `/CameMo_Conl`、`/camemo_feedback`、`/pursuitAngle`、`/vehicle_state`、`/voice_switch`）是**车式执行机构控制**话题：

- 但机上 **`pgrep -af dzactuator` 为空**，且 **`ls /dev/ttyACM* /dev/ttyUSB*` 无串口设备**（PX4 未接 USB）。
- ⇒ 这些话题**不是无人机本地进程**，而是**通过 DDS 从局域网另一台机器（小车 `dzrobot`/`duzhong.local`）跨机发现**的。

⚠️ **风险**：空地两机**共用同一 ROS 2 域**。`/odom`、`/imu_data`、`/cmd_vel` 这类通用话题名在两机上都可能出现，跨机可见会造成**误订阅/串扰**（例如把小车轮式 `/odom` 当成本机里程计）。
**建议**：两机分别设 `ROS_DOMAIN_ID`（不同值），或改用 `ROS_STATIC_PEERS`/DDS 域隔离；`~/dzlink_ws`（PC2 桥）另议。

---

## 七、复现命令（只读，可随时重跑）

```powershell
# 在 D:\User\Desktop\dzsmartscout 下
Get-Content -Raw tools\drone\cam_detect.sh |
  ssh -4 -o BatchMode=yes -o UserKnownHostsFile=tools\drone\known_hosts_dz `
      duzhong@dzsmart.local bash -s
```

（`cam_recon.sh` 出完整清点，`cam_preview_vnc.sh` 在 VNC `:1` 上放实时画面——**注意 `cam_preview_vnc.sh` 会启动进程，已超出"只读检查"范围，需你确认后再用**。）
