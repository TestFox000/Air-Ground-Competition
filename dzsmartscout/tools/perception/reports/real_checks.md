# 真实数据核查报告（自动生成，勿手改）

- 生成脚本：`perception/run_real_checks.py`
- 图纸（A，冻结真值）：`dzrobot_v2/_tmp_session/t28_newmap/map_src.png`
- 车侧真源（供对照）：`dzrobot_v2/config/map.yaml`
- 现役真值端到端对账：`perception/run_field_map_check.py`

## A · 图纸回归（map_src.png ↔ **该图纸自己的冻结点位**）

- ★2026-10-14 官方贴图重掷后 `config/map.yaml` 已描述另一张图（雷 C1/A3/…、靶 E5/F5、通 K5/K9）⇒ 本部分真值改为冻结点位，不再随 map.yaml 变；与现役真值的对账见 `field_map_check.md`。
- 格网（来自 identify_report.txt 原文）：格宽 64.087 px、格高 64.002 px、格心相位 (40.52, 46.42)
- 四角点像素：{"N11": [905.69, 718.44], "N1": [905.69, 14.42], "A1": [8.48, 14.42], "A11": [8.48, 718.44]}
- 注册：{"residual_px": 0.0, "condition": 2831.722, "pairs": 4, "notes": ["角点顺序 N11,N1,A1,A11"], "self_check": true}

| 类 | 真值数 | 检出数 | TP | FP | FN | Precision | Recall | 逐格一致 |
|---|---|---|---|---|---|---|---|---|
| mine | 8 | 8 | 8 | 0 | 0 | 1.0 | 1.0 | ✅ |
| obstacle | 8 | 8 | 8 | 0 | 0 | 1.0 | 1.0 | ✅ |
| target | 2 | 2 | 2 | 0 | 0 | 1.0 | 1.0 | ✅ |
| bridge | 2 | 2 | 2 | 0 | 0 | 1.0 | 1.0 | ✅ |

**结论：逐格完全一致**（叠加图 `A_map_registration.png`）

共剔除 48 个非目标色块（格线/文字碎片居多）；其中值得一提的 8 个：

- mine @ N8：bbox 内深色像素占比 0.0859 > 0.02（判为带文字的标注框，非目标）
- mine @ N5：bbox 内深色像素占比 0.1000 > 0.02（判为带文字的标注框，非目标）
- mine @ N10：bbox 内深色像素占比 0.1047 > 0.02（判为带文字的标注框，非目标）
- mine @ M2：面积比 0.490 不在 [0.55, 1.25]
- mine @ M2：面积比 0.186 不在 [0.55, 1.25]
- bridge @ B4：面积比 0.501 不在 [0.55, 1.25]
- bridge @ G11：面积比 0.474 不在 [0.55, 1.25]
- bridge @ J6：面积比 0.300 不在 [0.55, 1.25]

## B · cam_probe 真实照片可行性

- 相机帧 5 个 / VNC 桌面截图 3 个（分类见 `run_real_checks.py: CAM_PROBE_KIND`，逐个打开看过）
- **结论**：相机帧全部糊到不可用：拉普拉斯方差 1.368、3.076、49.146、45.496、45.586；而且画面里没有场地、没有网格、没有任何一类目标 ⇒ cam_probe 无法用于评估感知精度，只能作为『当前相机出图不可用』的证据。

| 文件 | 类型 | 尺寸 | 拉普拉斯方差 | 平均亮度 | 平均饱和度 | 可用于感知 |
|---|---|---|---|---|---|---|
| cam_1280x720.jpg | camera_frame | 1280x720 | 1.368 | 32.86 | 0.3886 | ❌ |
| cam_1920x1200.jpg | camera_frame | 1920x1200 | 3.076 | 24.92 | 0.4968 | ❌ |
| cam_ros_frame.png | camera_frame | 640x480 | 49.146 | 141.22 | 0.1543 | ❌ |
| camview.png | vnc_desktop | 1600x1000 | 293.394 | 100.29 | 0.9399 | ❌ |
| camview2.png | vnc_desktop | 1600x1000 | 292.524 | 100.29 | 0.9399 | ❌ |
| camview3.png | vnc_desktop | 1600x1000 | 265.056 | 120.35 | 0.5735 | ❌ |
| sharp_00_baseline.png | camera_frame | 640x480 | 45.496 | 109.59 | 0.2197 | ❌ |
| sharp_01_af.png | camera_frame | 640x480 | 45.586 | 109.31 | 0.2196 | ❌ |

合成实验标定的模糊失效点（64 px/m，见 `tests/test_pipeline_e2e.py`）：
| 目标 | 失效的模糊半径 |
|---|---|
| mines（12 cm 圆片） | r=3 全丢（r=2 尚存 7/8） |
| arrows（0.2 m 桥顶箭头） | r=3 全丢（r=2 仍两半场都对） |
| obstacles（30 cm 水桶） | r=4 全丢（r=3 仍 8/8） |
| targets（整格色块/靶区） | r=4 仍在、r=8 已丢 |
| car（0.22×0.10 m 车顶板） | r=3 丢失 |

各类颜色覆盖率（%）与『成块』检出数（≥200 px 且 fill≥0.7）：

| 文件 | 调色板 | 覆盖率最高的三类 | 成块检出 |
|---|---|---|---|
| cam_1280x720.jpg | field+provisional | mine=0.0046% obstacle=0.0046% target=0.0004% | — |
| cam_1280x720.jpg | map_image | bridge=0.0% mine=0.0% obstacle=0.0% | — |
| cam_1920x1200.jpg | field+provisional | mine=0.4901% obstacle=0.4901% target=0.0012% | — |
| cam_1920x1200.jpg | map_image | bridge=0.0482% mine=0.0% obstacle=0.0% | — |
| cam_ros_frame.png | field+provisional | mine=0.2555% obstacle=0.2555% deck=0.1924% | — |
| cam_ros_frame.png | map_image | bridge=6.4277% mine=0.0% obstacle=0.0% | — |
| camview.png | field+provisional | obstacle=93.7966% arrow=0.1479% deck=0.0602% | obstacle:3 |
| camview.png | map_image | bridge=0.2623% obstacle=0.0055% mine=0.0011% | bridge:3 |
| camview2.png | field+provisional | obstacle=93.7967% arrow=0.1479% deck=0.0602% | obstacle:3 |
| camview2.png | map_image | bridge=0.2614% obstacle=0.0055% mine=0.0011% | bridge:3 |
| camview3.png | field+provisional | obstacle=50.9126% arrow=0.5429% mine=0.1899% | arrow:2 obstacle:1 |
| camview3.png | map_image | bridge=2.0306% mine=0.0109% obstacle=0.0% | bridge:4 |
| sharp_00_baseline.png | field+provisional | deck=2.3991% obstacle=1.8734% arrow=1.5951% | — |
| sharp_00_baseline.png | map_image | bridge=11.2357% mine=0.0029% obstacle=0.0% | — |
| sharp_01_af.png | field+provisional | deck=2.266% obstacle=1.8542% arrow=1.6143% | — |
| sharp_01_af.png | map_image | bridge=11.0931% mine=0.001% obstacle=0.0% | — |

判据：拉普拉斯方差（4 邻接，中心 4/邻 −1，定义写死在本脚本）；颜色覆盖率按两套调色板各算一遍；**未做注册**（照片里没有场地，不猜角点）

## C · 实物桥顶箭头正控（p09_1.png）

- 绿掩膜覆盖率：0.9241%
- 连通域：3 个
  - 绿桶 bbox=[1551, 170, 1684, 295] area=11587 dir=0.0° conf=0.0 PCA轴=117.04°
  - 绿桶 bbox=[1315, 4, 1405, 94] area=6304 dir=0.0° conf=0.0 PCA轴=116.7°
  - **箭头** bbox=[634, 534, 714, 591] area=2135 dir=-96.0° conf=1.0 PCA轴=177.48°

- 真值：桥顶箭头尖头指向图像上方 ⇒ 期望 dir_deg ≈ −90°
- **实测方向 -96.0°，误差 -6.00°**

## D · 摄像头硬件现状复测（t59，对无人机只读）

- 探测地址：10.29.122.104, dzsmart.local（逐条结果见 JSON）
- 是否连通：❌ 否
- t1 基线（2026-10-11）：t1（2026-10-11）：整台未接摄像头 —— /dev/video* 空、无 UVC 设备
- **结论：❌ 复测未执行：无人机不在网（10.29.122.104（ssh: connect to host 10.29.122.104 port 22: Connection timed out）、dzsmart.local（ssh: connect to host 10.200.61.104 port 22: Connection timed out）；本网段 192.168.137.0/24 扫 22 端口 发现 2 台主机）⇒ 摄像头有无**无法确认**，沿用 t1 的现状基线（未见摄像头）。感知侧结论：视觉可在真图上跑通（见 part A / field_map 检查）、**实机待摄像头**。**

- 本网段扫描：本机 192.168.137.51，192.168.137.0/24 共 253 台，22 端口命中 2 台：`192.168.137.1`, `192.168.137.241`


- 复跑方法：`在能连上无人机的机器上：cd dzsmartscout/tools && python -m perception.run_real_checks --part D`

