# 补丁 04 —— 相机点亮与出图验证（USB/UVC 路线）

| 项 | 值 |
| --- | --- |
| 目标机路径 | `~/camtools/`（**新建目录**，不覆盖机上任何既有文件） |
| 本地文件 | `files/cam_lib.sh`、`files/cam_env.sh`、`files/cam_shot.sh`、`files/cam_up.sh` |
| 配套工具（已在机上 `/tmp/`） | [tools/drone/cam_ros_snap.py](../../../tools/drone/cam_ros_snap.py)（无头抓帧）、[tools/drone/cam_ros_view.py](../../../tools/drone/cam_ros_view.py)（VNC 里看画面） |
| 是否要编译 | **不要**。纯 shell + python，零 `colcon`、零 `apt` |
| 风险 | 极低。探测是只读的；抓帧只往 `/tmp` 写 |
| 状态 | **已在真机验证通过**（Jetson Orin Nano `dzsmart`，2026-10-07） |

---

## 一、结论：相机现在能用，而且是端到端验证过的

不是"驱动装好了所以应该能用"，是**真的出图了**：

| 验证层 | 命令 | 实测结果 |
| --- | --- | --- |
| ① 设备层 | `ls /dev/video*` | `/dev/video0`（取流）、`/dev/video1`（metadata，取不到流） |
| ② 出图层 | GStreamer 抓帧 | 1920x1200 → **99201 字节**，1280x720 → **41092 字节**，JPEG 头 `ff d8 ff` ✅ |
| ③ ROS 层 | `ros2 topic hz /image_raw` | **29.245 Hz**（640x480 rgb8，`step=1920`） |
| ④ 画面层 | 抓一帧存 PNG 并**人眼看** | 640x480 真实画面，`mean_bgr=[130.6,144.5,148.6]`，`min=0 max=255` |

②③④ 三层都过，才叫"能用"。只做①会出现"节点起来了几秒后又死"这种假成功。

## 二、这台相机是什么

```
lsusb : 32e4:0234 LRCP AR0234
驱动  : uvcvideo（标准 UVC 免驱，无需装任何东西）
节点名: /sys/class/video4linux/video0/name = "LRCP AR0234: LRCP AR0234"
总线  : usb-3610000.usb-2.3，挂在 **USB2** 根集线器下（480 Mbps）
用户组: duzhong 已在 `video` 组内 —— 免 sudo 可取流
```

**能力表（实测枚举，非推测）**

| 格式 | 分辨率 → 最高帧率 |
| --- | --- |
| MJPG | 1920x1200 / 1920x1080 / 896x1200 / 1600x1200 / 1280x960 → **90/60/30**；1280x720 / 960x720 / 800x600 / 640x480 / 320x240 → **120/90/60/30** |
| YUY2（原始） | 1920x1200→5、1280x720→10、1024x768→10、960x720→15、800x600→20、**640x480→30**、320x240→120 |

**关键点：MJPG 帧率远高于 YUYV。**YUYV 是未压缩 4:2:2（2 字节/像素），1920x1200 只能跑 5 fps。

## 三、踩到的坑：`v4l2_camera` **不支持 MJPG**（必读，否则一定重复踩）

第一反应一定是这么起节点：

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -p pixel_format:=MJPG ...   # ❌ 必崩
```

**它会直接 abort：**

```
[WARN] Current pixel format is not supported yet: MJPG 1196444237
[WARN] Image encoding not the same as requested output, performing possibly slow conversion:  => rgb8
terminate called after throwing an instance of 'cv_bridge::Exception'
  what():  Unrecognized image encoding []
[ros2run]: Aborted
```

原因：`v4l2_camera` 0.6.2（Humble）只把**原始格式**交给 `cv_bridge`，MJPG 不在支持集里。
旁证两条，互相独立：
- 从 `/opt/ros/humble/lib/libv4l2_camera*.so` 里提取字符串，`pixel_format` 相关字面量**只有 `YUYV`**；
- [官方 Humble 文档](https://docs.ros.org/en/ros2_packages/humble/api/v4l2_camera/) 里 `pixel_format` 默认值就是 `"YUYV"`。

**所以：ROS 路线只能 YUYV。**想要高分辨率就只能在人眼预览时用 MJPG（GStreamer），两者互斥。

**能用的参数（同样从 `.so` 实测提取）**

| 参数 | 可取值 |
| --- | --- |
| `video_device` | `/dev/video0` |
| `pixel_format` | `YUYV`（**只有它**） |
| `image_size` / `time_per_frame` | 整型数组：`-p image_size:="[640,480]"`、`-p time_per_frame:="[1,30]"` |
| `output_encoding` | `mono8` / `rgb8` / `yuv422` / `yuv422_yuy2` |
| `camera_frame_id` | 任意字符串（建议 `camera_optical_frame`） |

**另一条铁律：同一个 UVC 节点同一时刻只能被一个进程取流。**ROS 节点和 GStreamer 预览不能同时开，第二个会报 `Device or resource busy`。

## 四、怎么用

### 1) 拷贝（用户手动执行）

```bash
mkdir -p ~/camtools
# 把 files/ 下 4 个脚本放到 ~/camtools/
chmod +x ~/camtools/*.sh
# 若传输过程引入了 CRLF（Windows 拷贝常见），先统一换行：
sed -i 's/\r$//' ~/camtools/*.sh
```

### 2) 三层体检 / 点亮

```bash
bash ~/camtools/cam_env.sh          # 只读体检：设备、驱动、权限、工具链、能力枚举
bash ~/camtools/cam_shot.sh         # 连抓 8 帧留最后一帧 -> /tmp/cam_shot.jpg，并判定全黑/全白/正常
bash ~/camtools/cam_up.sh           # 预览：MJPG 1920x1200 开窗到 VNC 桌面
bash ~/camtools/cam_up.sh --ros     # ROS：v4l2_camera YUYV 640x480@30，发布 /image_raw
bash ~/camtools/cam_up.sh --ros 1280x720 10   # ROS 想要更大画面时（YUYV 上限 10fps）
```

### 3) ROS 路线看画面（不抢设备，可与节点并存）

```bash
source /opt/ros/humble/setup.bash
DISPLAY=:1 python3 /tmp/cam_ros_view.py /image_raw 1280 800   # VNC 桌面实时窗口
python3 /tmp/cam_ros_snap.py                                   # 无头抓一帧 -> /tmp/cam_ros_frame.png
```

**为什么不用 `rqt_image_view`**：它的窗口默认只有 **200x105**，而机上 `wmctrl` / `xdotool` / `python-Xlib` **三样全都没有**（实测），无法从外部把窗口撑大，截图出来根本看不清。`cam_ros_view.py` 用 OpenCV 的 `WINDOW_NORMAL + resizeWindow` 自己定尺寸。

## 五、验证（助手只读复验，或用户自查）

```bash
source /opt/ros/humble/setup.bash

# 1) 设备与驱动
ls -l /dev/video*
cat /sys/class/video4linux/video0/name

# 2) 话题与频率（ROS 节点在跑时）
ros2 node list | grep v4l2
ros2 topic hz /image_raw                  # 期望 ~29-30 Hz
ros2 topic echo --once --field width /image_raw      # 期望 640
ros2 topic echo --once --field encoding /image_raw   # 期望 rgb8

# 3) 画面层（唯一能证伪"假成功"的检查）
python3 /tmp/cam_ros_snap.py              # 期望 "写盘=OK" 且 mean 不是 0/255
```

**期望值**（本次实测基线，偏离了就有问题）：

| 指标 | 基线 |
| --- | --- |
| `/image_raw` 频率 | 29.245 Hz（640x480@30，实测 min 0.031s / max 0.069s / std 5.9ms） |
| 帧字段 | `width=640 height=480 encoding=rgb8 step=1920` |
| 像素统计 | `min=0 max=255 mean_bgr≈[131,145,149]` |

## 六、回退

**没有要回退的东西** —— 本包不改机上任何既有文件、不装任何包、不写系统路径。

```bash
rm -rf ~/camtools
pkill -f v4l2_camera_node      # 若 ROS 节点还开着
pkill -f cam_ros_view          # 若预览窗口还开着
```

机上的 `~/dzpx4`、`/opt/ros` 一行都没动过。

## 七、已知边界与未做

| 项 | 说明 |
| --- | --- |
| **`/camera_info` 没有真实内参** | 日志明确：`Unable to open camera calibration file [/home/duzhong/.ros/camera_info/lrcp_ar0234:_lrcp_ar0234.yaml]`。做 VIO/SLAM 前**必须标定**，否则 `/camera_info` 是空壳 |
| **画面糊，而且软件改不动** | 实测**定焦镜头，UVC 对焦控制是空操作**（见第九节：AF 开关、`focus_absolute` 0→255 全扫，拉普拉斯方差恒在 38~47，毫无改善）。只能物理解决：撕镜头保护膜、退到最小对焦距离之外、对着有纹理的亮处 |
| **画面偏暗** | 参数侧：`auto_exposure=3`（光圈优先）、`brightness=0`、`gain=0`。但**先看好消息**：连续取流收敛后 `mean≈140`，亮度其实是够的；单帧抓取偏暗是 AE 未收敛，不是相机问题 |
| **单帧抓取偏暗 ≠ 相机坏** | UVC 开流瞬间自动曝光未收敛。实测单帧 `avg≈38`，连续取流后 `mean≈140`。所以 `cam_shot.sh` 改成连抓 8 帧留最后一帧 |
| CSI / MIPI 相机 | 不在本包范围（走 `nvarguscamerasrc` + 设备树，与 UVC 完全不同的路径） |
| `v4l2-ctl` / `ffmpeg` | 机上**未安装**，所以本包一律不依赖它们；能力枚举用 `gst-device-monitor-1.0` 代替 |
| MJPG 进 ROS | 本包未做。若将来需要（1920x1200@30 进 ROS），要另装 `usb_cam` 或用 GStreamer+a`appsink` 自建节点，属新工作 |

## 八、实测原始数据（可复核）

```
# 设备
crw-rw----+ 1 root video 81, 0 10月  7 16:37 /dev/video0
crw-rw----+ 1 root video 81, 1 10月  7 16:37 /dev/video1
Bus 001 Device 004: ID 32e4:0234 LRCP AR0234 LRCP AR0234
uvcvideo              110592  0
组=...,44(video),...

# 抓帧（GStreamer，MJPG 直通落盘）
1920x1200 -> 99201 字节，头 ff d8 ff
1280x720  -> 41092 字节，头 ff d8 ff
8x8 灰度单帧: min=1 max=122 avg=38.1   <-- AE 未收敛，偏暗

# ROS 节点（YUYV 640x480@30）
/image_raw : average rate 29.245, min 0.031s max 0.069s std 0.00586s
width=640 height=480 encoding=rgb8 step=1920
抓帧 PNG: 640x480 rgb8, mean_bgr=[130.6, 144.5, 148.6], min=0, max=255

# 摄像头控制当前值
auto_exposure=3   exposure_time_absolute=156[inactive]   brightness=0   gain=0
white_balance_automatic=True   sharpness=0   backlight_compensation=60
focus_absolute=0  focus_automatic_continuous=0

# 工具链
cv2 4.5.4 可用；gst-launch-1.0 / gst-device-monitor-1.0 可用
v4l2-ctl MISSING、ffmpeg MISSING、wmctrl MISSING、xdotool MISSING、python-Xlib MISSING
rqt_image_view 窗口默认 200x105（不可用外部调整）
```

## 九、对焦实验：为什么画面糊，以及为什么软件修不了

工具：[tools/drone/cam_sharpness.py](../../../tools/drone/cam_sharpness.py)
指标：拉普拉斯方差（对焦评价函数，越大越清晰；同场景下 <10 极糊、10~100 糊、>100 基本清晰）。

| 设置 | 拉普拉斯方差 mean | 结论 |
| --- | --- | --- |
| 基线（`focus_automatic_continuous=false`, `focus_absolute=0`） | **46.2**（min 45.7 / max 46.8） | 落在"糊"区间 |
| 打开自动对焦 | 46.4 | **噪声级差异，等于没变** |
| `focus_absolute=60` | 46.5 | 无改善 |
| `focus_absolute=120` | 46.3 | 无改善 |
| `focus_absolute=180` | 44.9 | 略降 |
| `focus_absolute=255` | **38.1** | 更差 |

每次 `ros2 param set` 都返回 `Set parameter successful`，**但画面一丝没变**（AF 与手动扫描全都不动）。

**结论：这是一颗定焦镜头，UVC 的对焦控制是空操作（no-op）。**
画面对焦状态**无法通过软件改善**，只能物理解决。现场按这个顺序查：

1. **镜头保护膜/防尘贴是否撕掉** —— 最像当前症状：实测帧有强烈的光晕/泛白（大面积高光溢出），
   这是光线被一层半透明膜散射的典型表现，和纯失焦的观感不同。
2. **镜头是否被拧到/装歪** —— M12 镜头模组如果螺纹没旋到位，整幅就是这种均匀的糊。
3. **退到最小对焦距离之外** —— 画面像是贴在桌面/反光面上拍的。定焦模组的最近对焦距离通常
   在 10~30 cm，贴太近必糊。
4. **对着有纹理、光照适中的目标再测一次** —— 白墙/纯色面拍出来永远是糊的（没有边缘可评价），
   用 `cam_sharpness.py` 复测，方差应显著上升。**换目标后复测是唯一能区分"镜头不行"和"目标不行"的办法。**

亮度侧的好消息：连续取流收敛后 `mean_bgr≈[131,145,149]`，**亮度是够用的**；
之前看到的暗帧是单帧抓取时自动曝光还没收敛，不是相机缺陷。
