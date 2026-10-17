# 无人机俯瞰场地感知管线（`tools/perception/`）

**一句话**：把一帧俯视照片 → 格名（A1..N11）→ 车侧能吃的 JSON 报文
（`heartbeat` / `bridge_passable` / `target_zone` / `field_verify`）。

纯 Python，只依赖 **numpy + Pillow**（本机 Windows 无 cv2/scipy/pytest/yaml 也能跑全套单测）。
`cv2` 不需要；`rclpy` 只在真发 DDS 时用（仅无人机上有）。

```bash
cd dzsmartscout/tools
python -m perception.cli selftest            # 离线自检（66 项，秒级；不含最慢的 e2e）
python -m perception.cli selftest --with-e2e # 72 项全部
python -m perception.run_real_checks         # 真图核查（图纸回归 + cam_probe + 实物箭头）
python -m perception.cli detect --image 图.png --corners "905.7,718.4;905.7,14.4;8.5,14.4;8.5,718.4"
```

| 交付物 | 说明 |
|---|---|
| [`grid.py`](grid.py) | ① 网格注册：四角点 → 单应 → 任意像素 → 格名（格边 0.35 m） |
| [`colors.py`](colors.py) | ② 颜色分类（HSV/RGB 规则 + 可标定调色板）、形态学、连通域 |
| [`objects.py`](objects.py) | ② 雷 / 障 / 靶区（+ 图纸"通"格）→ 格名 + 拒收理由 + 冲突消解 |
| [`car_pose.py`](car_pose.py) | ③ 车顶标记 → `block` + `yaw`（透视校正） |
| [`bridge_arrow.py`](bridge_arrow.py) | ④ 桥顶箭头 → 方向 → 半场通行格（H1 位置 / H2 方向两种判据） |
| [`protocol.py`](protocol.py) | ⑤ 协议发射器（字段逐条对照 `dzbrain/tasks/drone.py`）+ 车侧解析镜像 |
| [`pipeline.py`](pipeline.py) | 总装：一帧 → 一组报文（含全部中间证据） |
| [`ros_publish.py`](ros_publish.py) | 可选：真发 `/dzlink/drone_to_car`（QoS 与车侧一致的 BEST_EFFORT） |
| [`overlay.py`](overlay.py) · [`cli.py`](cli.py) | 人工复核叠加图 · 命令行入口 |
| [`tests/`](tests) | 72 项确定性单测（合成图 + 真实图纸 + 真实照片） |
| [`run_real_checks.py`](run_real_checks.py) · [`reports/`](reports) | 真图核查脚本与产物（含逐格对比表） |

> 本目录**只读**外部仓库：`dzrobot_v2/config/map.yaml`、`dzrobot_v2/_tmp_session/t28_newmap/`、
> `cam_probe/`、`fc-audit/arena_pdf_img/`，一个字节都没改。

---

## 一、坐标语义（与车侧逐字一致，这是唯一坐标系）

真源：`dzrobot_v2/src/dzbrain/dzbrain/core/geometry.py:6-16,69-81`

```
x = (10.5 - row_idx) * 0.35      row 11 → 0.175, row 1 → 3.675
y = (13.5 - col_idx) * 0.35      col N  → 0.175, col A → 4.725
```

| 概念 | 值 |
|---|---|
| 场地 | 14 列 A~N × 11 行 1~11，格边 0.35 m ⇒ **4.90 m × 3.85 m** |
| 原点 | **N11 格外角**（不是格心） |
| +x | 行号**减小**方向（row 11 → row 1，下半场 → 上半场） |
| +y | 列号**减小**方向（N → A，与 `map.yaml:18`"车头朝 +y = 图左"一致） |
| 半场 | `x > 6×0.35 = 2.10` = **upper**（行 1~6）；否则 **lower**（行 7~11）；分界即 `map.yaml:46-47` 的挡板行 6/7 |
| 四角点标签 | `N11`=(0,0)、`N1`=(3.85,0)、`A1`=(3.85,4.90)、`A11`=(0,4.90)（**该格的外角**） |
| yaw | `atan2(dy, dx)`（度）：0 = 朝 +x；+90 = 朝 +y；±180 = 朝 −x |

**角点顺序默认 `N11, N1, A1, A11`**，也可以标签化传：`--corners "N11=u,v;N1=u,v;A1=u,v;A11=u,v"`。

⚠️ **手性**：场地米制系是右手系（z 朝上），而图像数组 `(u 右, v 下)` 是左手系 ⇒
**正常俯视图的像素有向面积一定是负的**（实测 `map_src.png` = −6.3×10⁵ vs 米制 +18.87）。
两者**同号**只可能是图像被镜像或角点标签张冠李戴，`register()` 默认直接报错
（`expect_flip=False` 才放行）。这条拦下过一次真实的顺序错位。

⚠️ `geometry.py:39` 的注释写 `HEADING_X = 0 → 朝 X+（行号增大方向）`，与 `:72` 的公式
（x 随 `row_idx` **减小**而增大）矛盾。**以公式为准**（`map.yaml:18`、`map_src.png`
视觉核对都站在公式这边）。本工具只记录该矛盾，不改车侧任何文件。

---

## 二、模块速览

### ① 网格注册 `grid.py`

* `register_from_corners(corners_px, spec)` / `register_from_named_corners({...})`：
  DLT 最小二乘（Hartley 归一化，**支持 4 点以上**：把格心也一起给进去可以把标定误差摊薄）。
* `reg.px_to_hit(u, v)` → `CellHit(block, row_idx, col_idx, fx, fy, in_arena, half, offset_from_center_m)`；
  `fx/fy` 是格内归一化坐标（格心 0.5），越界会 >1/<0。
* `reg.cell_polygon_px / cell_center_px / cell_area_px`：格几何（面积随透视变化，
  面积类判据一律用它归一）。
* `reg.self_check()`：154 格逐格"格心 → 像素 → 米 → 格名"闭合自检；`diagnostics()` 给残差与条件数。

### ② 颜色分类 `colors.py` + 目标落格 `objects.py`

**调色板是数据，不是代码**：`Palette` 可 `to_json/from_json`，现场标定后换一行配置即可。
两个内置调色板：

| 调色板 | 用途 | 证据 |
|---|---|---|
| `MAP_IMAGE` | 图纸（`map_src.png`） | RGB 阈值**逐字**抄自 `identify_t28_v2.py:58-63`；语义见 `map.yaml:48-55` |
| `FIELD_DEFAULT` | 实物 | 只有 `deck`（黄桥面/挡板）与 `arrow`（绿箭头贴纸）有实测证据；其余 `provisional=True` **默认关闭** |

⚠️ **实物颜色只有两条有证据，别把图纸口径当实物**：

* 说明书 p4/p8 + 照片 `arena_pdf_img/p09_1.png`：挡板/桥面是**黄色**、桥顶箭头是**白纸上的绿箭头**；
* 说明书 p5「直径 12 cm **彩色**地雷图片」、p7「**彩色**带盖水桶」——没给颜色；
  而且 `p09_1.png` 同框可见**红 / 青 / 绿三种桶** ⇒ **水桶颜色不唯一，单色阈值不可靠**；
* 任务书里"雷红 / 障 / 靶粉"的猜测与唯一真源冲突：`map.yaml` 明写**雷=黄块、障=红块**
  （那是**图纸**口径）。代码因此不写死任何一类颜色，`--provisional` 才启用占位阈值。

颜色分类 → 形态学（`open_/close_`，边界语义与 OpenCV 一致）→ 连通域（run-based union-find，
8 邻接，顺带算二阶矩）→ 几何判据。两套几何口径：

| 目标 | 图纸口径（方块） | 实物口径 |
|---|---|---|
| 雷 | `cell_frac` 0.55~1.25、`fill` 0.78~1.01、**深色像素占比 ≤0.02** | 12 cm 圆片：`cell_frac` 0.03~0.30、`fill` 0.55~0.98 |
| 障 | 同方块的 0.55~1.25 / 0.78~1.01 | 30 cm 桶：`cell_frac` 0.25~0.95、`fill` 0.55~1.01 |
| 靶 | 方块 | 形态未确认 ⇒ 判据放宽（0.03~1.25 / 0.05~1.01） |

* 12 cm 圆片在 35 cm 格里只占 **9.2%** —— 用图纸的"接近满格"去套实物必然漏检（反之亦然）。
* **"深色像素占比"这条判据是实测出来的**：`map_src.png` 上 8 个真雷的 bbox 内深色像素
  （luma<150）占比 ≤0.0014，而 3 个"起终点/无人机点"黄色**标注框**是 0.086~0.105
  ⇒ 阈值 0.02 有 60 倍余量。**没有这条，图纸回归会多报 N10/N5/N8 三个假雷**（这正是
  t28 识图脚本"证据 C"的口径，`identify_t28_v2.py:117-131`）。
* 同格冲突按 `priority`（障 > 雷 > 靶）消解并记录；每条拒收都留 `reason`（面积比/填充率/深色占比）。

### ③ 车位姿 `car_pose.py`

* 输入是"车顶贴一块纯色板"这一**己方假设**（规则全文没有车顶标记这一项）
  ⇒ 颜色必须现场标定，`CarSpec(cls=..., nose_cls=..., yaw_offset_deg=...)`。
* 像素先经单应映到**场地米制系**再算主轴 ⇒ yaw 是**透视校正后**的。
* **单块只能定到 mod 180°**（`yaw_ambiguous=True`，置信度减半）；再贴一个**车头小标记**
  才能定向：用**主板主轴**定方向、车头标记只负责决定正反（实测比"两质心连线"稳得多）。
* 实测精度（合成，64 px/m：板 0.22×0.10 m、车头标记 0.06 m 方块、前移 0.16 m 不重叠）：

  | 配置 | 方向误差 max / mean |
  |---|---|
  | 正俯视 | 4.63° / 1.74° |
  | rot=90°+tilt=0.30（强斜视） | 6.15° / 2.28° |
  | **150 px/m**（≈2.3 mm/px，无人机 2 m 高度量级） | 2.07° / 0.91° |
  | 200 px/m | 1.62° / 0.73° |

  **格名在 50 组配置里 100% 正确**。⚠️ 车头标记若与车顶板**重叠**，会把板的一端遮掉、
  主轴被拉偏（实测误差从 6.2° 涨到 12.6°）⇒ **安装时让两者分开**。

### ④ 桥顶箭头 `bridge_arrow.py`

现场形态：**黄色桥面板**上一张**白纸 + 绿箭头**，箭头贴在**某一块板**上、尖头指向
**穿洞方向**（`p09_1.png` 实测）。赛道模型（`map.yaml:33-35`）：**整个 K 列是一座桥**，
每格一台横向贯通的桥洞，格间有侧壁 ⇒ 桥带长轴 = 行方向(x)，穿洞方向 = 列方向(y)。

两种判据都实现了，**默认 H1（位置）**：

* **H1 位置**：箭头在哪块板上 ⇒ 那个 K 列格就是该半场的通行格
  （证据：箭头确实贴在某一块板上、且尖头朝穿洞方向而非桥带方向）；
* **H2 方向**（`infer_mode="direction"`）：箭头是"沿桥带指向相邻通行格"的指示牌。

**方向估计（可复现）**：不直接用 PCA 主轴 —— 实物箭头**头部宽 80 px、整个箭头才 58 px 高**，
PCA 主轴是**横向**的（实测 PCA 给 −2.5°，真值 −90°，差 87°）。改成**扫描候选轴 +
结构打分**：沿轴看宽度剖面，真箭头轴必满足「一端**等宽低平台**（箭杆）+ 另一端
**单调张开**到头部」；横向轴两端镜像对称、端部是渐变翼尖，拿不到这个结构。

| 轴 | 归一化宽度剖面（`p09_1.png`） | 判据 |
|---|---|---|
| **90°（真值）** | `0.16 0.30 0.47 0.64 0.82 0.92 0.97 1.00` │ `0.66 0.63 0.28 0.28 0.29 0.28` | 杆 4 段 + 张开 8 段 ✓ |
| 0° | `0.27 0.29 0.29 0.28 0.31 0.94 … 0.24 0.24 0.25 0.24` | 两端都像杆 ⇒ 歧义，扣分 |
| 60° | `0.73 0.84 0.92 1.00 0.55 … 0.84 0.15` | 端部渐变，无杆 ⇒ 0 分 |

尖端侧由**结构**决定（箭杆的另一端），再用三线索投票做一致性折扣。

* 合成实心箭头 14 个角度：误差 **≤1.0°**，置信度 1.00；
* **实物照片**（`p09_1.png` 的绿箭头）：估计 **−96°**，真值"指向图像上方"=−90° ⇒ 误差 6°；
  同一张图里的绿桶被正确拒绝（置信度 0.000）。
* 已知失败模式（固化在测试里）：点抖动 ≥ 箭头长的 10% 时方向可能翻 90°+ 而置信度仍 ≥0.9
  ⇒ **位置判据（H1）才是定通行格的那条，方向只作证据**（`dir_kind`/`dir_deg_map` 照样输出）。

### ⑤ 协议发射器 `protocol.py`

消费端：`dzrobot_v2/src/dzbrain/dzbrain/tasks/drone.py`（`DroneTask`，5 Hz）

| 报文 | 车侧要求字段 | 车侧动作 | 出处 |
|---|---|---|---|
| `heartbeat` | `type` | 刷心跳；>3 s 无心跳 → `online=False` + 一次告警 | `drone.py:228,244-253,379-391`；`robot.yaml` `heartbeat_timeout_s: 3.0` |
| `bridge_passable` | `half`∈{lower,upper}、`block` 非空、`confidence` | 写 `P_DRONE`；>30 s 过期返回 None | `drone.py:273-289`；`info_stale_s: 30.0` |
| `target_zone` | `blocks` 非空 list | 与本地 `map.yaml.targets` 差集 → `field_diff` | `drone.py:292-304` |
| `field_verify` | `mines` / `obstacles` list | 同上 | `drone.py:307-326` |
| 任意带 `seq` | — | 回 `ack{type:"ack",of:seq}`（**送达硬证据**） | `drone.py:255-270` |

话题/QoS：`/dzlink/drone_to_car` · `std_msgs/String` · **depth 10 · BEST_EFFORT**
（`contract.py:40-41,94-96`）。**QoS 配错会静默收不到**（机侧 RELIABLE 不匹配车侧
BEST_EFFORT），`ros_publish.py` 因此把 QoS 写死成车侧一致，并提供 `listen_acks()` 收 ack。

四个从源码里读出来、容易踩的点（都写进了 `README` 与自检）：

1. 载荷里的 `ts` **车侧不用**：`_on_bridge_passable` 用**收到时刻**（`drone.py:284`）；
   ⇒ "情报 30 s 过期"是从**车收到**算起，**周期性重发同一结论是有效的续期手段**。
2. `target_zone.blocks` 为空表 = 车侧记一次 `missing_fields`（`drone.py:294`）⇒ 本模块**直接抛错**，别发空表。
3. 格名**不校验是否在地图内**（`drone.py:280-281` 明写"穿洞格每局随机，无人机才是权威"）
   ⇒ 差集核对才有意义。发送端另有一层**严格**校验（`validate_payload(p, spec)`）拦笔误。
4. 未知 `type` 只记日志不崩（`:236-240`）⇒ **加字段安全、加类型不安全**。本模块只产出上述 4 种。

`mirror_car_side_parse()` 逐行**复述**车侧判定（不 import 车侧代码），用来在发送前证明
"这条报文车侧会正常处理"。测试里就是用它 + 镜像 `_diff` 证明：照 `map.yaml` 发 ⇒ 零差异；
改一格 ⇒ 立刻报差异。

### ⑥ 出图 / 场地地图 `field_map.py`（t59）

把①②③④的**感知输出**装配成"规划器可直接消费"的场地地图：

```python
from perception import build_field_map_from_result, write_field_map
fm = build_field_map_from_result(result, name="第五张图", start="N10", end="N5",
                                 barrier_col_from="D")     # result = pipeline 的输出
print(fm.to_json_text())          # {"map": {...map.yaml 同 schema...}, "perception": {...证据...}}
write_field_map(fm, "reports/", stem="field_map_official")  # 出 .json + .yaml
```

要点：

* **`map:` 块与 `config/map.yaml` 同 schema**（`name/start/end/barrier_col_from/barrier_row_hi/
  barrier_row_lo/mines/obstacles/targets/bridges`）⇒ 可直接 diff；**真源仍是 `map.yaml`**。
* **`perception:` 块是证据**：每格 `cell_confidence`、`car`（车位姿）、`blue_face`、`anchors`
  （黄标注框落格）、`rejected`（带理由）、`provenance`（来源图/标定/残差/阈值/时间）。
* **桥洞两种口径**：实拍用 `bridge_calls`（桥顶箭头推得）；图纸用颜色类 `bridge`（绿色通格）。
* **蓝面朝向**：`estimate_blue_face()` —— 无人机俯视**判不了**（竖直面看不见）就如实 `unknown`；
  已内置"外框深蓝 H≈220° 会被误判成蓝面"的守卫（ROI 不外扩 + 色块质心居中）。
* **自动标定**：`grid.register_from_arena_frame(rgb)` 直接从"外框 + 灰格线"图纸算出四角点
  （官方图实测 `sx=45.5857 / sy=45.5073 px`、残差 0.000 px），**不需要手点角点**。
* 阈值可覆盖：`build_field_map(..., accept={"mine": 0.7})`；建议值见 `confidence_thresholds()`。

---

## 三、真图核查结果（`python -m perception.run_real_checks`）

产物：`reports/real_checks.md` · `real_checks.json` · `A_map_registration.png`（叠加图）。
另有 **L1 出图核查** `python -m perception.run_field_map_check` → `reports/field_map_check.md`
与 `field_map_official.{json,yaml}`（**与现役 `config/map.yaml` 逐条对账，0 差异**）；
能力盘点见 [`CAPABILITY_L1.md`](CAPABILITY_L1.md)。

### A · 图纸回归：`map_src.png` ↔ **该图纸自己的冻结点位**

> ★2026-10-14 官方贴图重掷后 `config/map.yaml` 已描述**另一张图**（雷 C1/A3/…、靶 E5/F5、
> 通 K5/K9）⇒ 本部分真值改为 `run_real_checks.DRAWING_TRUTH_FROZEN`（冻结点位，不随真源漂移）；
> "真值 == 现役真源"的对账由 `run_field_map_check.py` 用官方图 `map_official.png` 承担（0 差异）。

标定参数**取自识图报告原文**（`identify_report.txt`：格宽 64.087 px、格高 64.002 px、
格心相位 (40.52, 46.42)），四角点由它算出，不是写死的魔数。

| 类 | 真值 | 检出 | TP | FP | FN | P | R | 逐格一致 |
|---|---|---|---|---|---|---|---|---|
| mine（雷） | 8 | 8 | 8 | 0 | 0 | 1.000 | 1.000 | ✅ |
| obstacle（障） | 8 | 8 | 8 | 0 | 0 | 1.000 | 1.000 | ✅ |
| target（靶） | 2 | 2 | 2 | 0 | 0 | 1.000 | 1.000 | ✅ |
| bridge（通） | 2 | 2 | 2 | 0 | 0 | 1.000 | 1.000 | ✅ |

注册残差 0.00 px、条件数 2831、154 格闭合自检通过。**这是本管线唯一的"真实数据全对"证据**
（用的是与 `config/map.yaml` 同源的图）。被剔除的 4 个黄框都带明确理由
（深色像素占比超限 / 面积比不达标），不是漏检。

### B · `cam_probe/` 真实照片：**不可用**

`cam_probe/` 里 8 个文件，逐个看过，**只有 5 个是相机出图**，另外 3 个（`camview*.png`）
是 Jetson 桌面截图（XFCE + `rqt_image_view` 窗口）——拿它算相机清晰度没有意义，本报告单列。

| 文件 | 类型 | 尺寸 | 拉普拉斯方差 | 平均亮度 | 平均饱和度 |
|---|---|---|---|---|---|
| cam_1280x720.jpg | 相机帧 | 1280x720 | 1.37 | 32.86 | 0.389 |
| cam_1920x1200.jpg | 相机帧 | 1920x1200 | 3.08 | 24.92 | 0.497 |
| cam_ros_frame.png | 相机帧 | 640x480 | 49.15 | 141.22 | 0.154 |
| sharp_00_baseline.png | 相机帧 | 640x480 | 45.50 | 109.59 | 0.220 |
| sharp_01_af.png（AF 全扫） | 相机帧 | 640x480 | 45.59 | 109.31 | 0.220 |
| camview.png / camview2.png | VNC 桌面 | 1600x1000 | 293.4 / 292.5 | 100.29 | 0.940 |

* 拉普拉斯方差定义写死在脚本里（灰度 4 邻接，中心 4 / 邻 −1）。**与 t1 报告独立复算一致**
  （t1 量 640×480 三帧 ≈46.5/46.5/50.2，本脚本 45.50/45.59/49.15）。
* `sharp_01_af` 与 `sharp_00_baseline` 几乎逐位相同（45.59 vs 45.50）⇒ **AF 扫描对清晰度无效**，
  与 t1 的"定焦镜头、`focus_absolute` 是 no-op"结论一致。
* 更关键的是**画面内容**：5 帧全是室内墙面/桌面，**画面里没有场地、没有网格、没有任何一类目标**，
  颜色覆盖率最高的类也只有 0.001~6.4%（且是噪声散点；≥200 px 且 fill≥0.7 的成块检出为 0）。
* ⇒ **`cam_probe` 不能用来评估感知精度**，它只能当"当前相机出图不可用"的证据。
  **本管线的精度证据来自合成图（72 项单测）与真实图纸/真实照片（A/C 两部分）**，不是这几帧。

### C · 实物桥顶箭头正控：`fc-audit/arena_pdf_img/p09_1.png`

* 绿掩膜覆盖 0.924%，连通域 3 个：两个是**绿桶**（area 11587 / 6304，方向估计置信度 0.000 ⇒ 正确拒绝），
  一个是**箭头**（bbox 634,534–714,591，area 2135）。
* 箭头方向估计 **−96°**（真值：指向图像上方 = −90°，误差 **6°**），置信度 1.000。
* 同一张图里 PCA 轴给 177.5°（**横向**）——这就是"不能只用 PCA"的现场证据。

---

## 四、可行性结论：糊相机 / 低分辨率下"哪些能检、哪些不能"

合成图退化特征化（64 px/m，全要素场景；结论固化成
`tests/test_pipeline_e2e.py::test_degradation_characterization_blur_noise_resolution`）：

| 退化 | 8 雷 | 8 障 | 靶区 | 桥顶箭头 | 车 |
|---|---|---|---|---|---|
| 干净 | 8/8 | 8/8 | ✅ | 两半场 ✅ | ✅ |
| 模糊 1 px | 8/8 | 8/8 | ✅ | **两半场仍 ✅** | ✅ |
| 模糊 2 px | 7/8 | 8/8 | ✅ | 两半场 ✅ | ✅ |
| 模糊 3 px | **0/8** | 8/8 | ✅ | **全丢** | **丢** |
| 模糊 4 px | 0/8 | **0/8** | ✅ | 丢 | 丢 |
| 模糊 8 px | 0/8 | 0/8 | **丢** | 丢 | 丢 |
| 噪声 σ=20/255 | 8/8 | 8/8 | ✅ | ✅ | ✅ |
| 噪声 σ=30/255 | 8/8 | 4/8 | ✅ | ✅ | ✅ |
| 噪声 σ=45/255 | 4/8 | 0/8 | ✅ | 只剩上半场 | ✅ |
| 32 px/m | 7/8 | 8/8 | ✅ | ✅ | ✅ |
| 16 px/m | 5/8 | 8/8 | 丢 | 丢 | ✅ |

**能检 / 不能检（如实记）**

* ✅ **能检**：整格大小的色块（靶区）、30 cm 桶、1 px 以内的轻微模糊、σ≤20/255 的噪声、
  ≥32 px/m 的分辨率、0.20 m 箭头在 **150 px/m** 下的方向（≤15°）、车顶板在 150 px/m 下的
  block+yaw（≤3°）。真实图纸（A）逐格 100% 复现。
* ❌ **不能检**：
  * **当前相机出图（B）里什么都检不到** —— 不是算法问题，是画面里没有场地（且糊到 lapVar≈45）；
  * 模糊半径 ≥3 px 时：**12 cm 地雷与桥顶箭头同时失效**（这两路最脆，比 30 cm 桶先坏 1 px）；
  * 分辨率 ≤16 px/m（≈2 m 高度用 640×480 档）时：地雷、靶区、箭头全丢；
  * 点抖动 ≥ 箭头长 10% 时，箭头**方向**可能翻 180° 而置信度仍高 ⇒ 方向只作证据，通行格由位置判据定；
  * 颜色层面：**水桶颜色不唯一**（红/青/绿同框）⇒ 单色阈值不能作为障的唯一判据；
    地雷/靶的实物颜色**没有实测证据**，必须现场用 `perception.cli calibrate` 标定。
* ⚠️ 门槛提醒：规则强制无人机读的正是**桥顶箭头**，而它和 12 cm 地雷一样是"小目标"——
  按 t1 的 GSD 估算（1920 档、2 m 高度 → 2.0 mm/px），0.20 m 箭头 ≈100 px、12 cm 雷片 ≈60 px
  ⇒ **分辨率够、卡在光学清晰度（当前相机糊）**。这条结论与 t1 的《感知可行性_摄像头评估》一致。

---

## 五、现场使用（标定 → 检测 → 发布）

### 1) 标定（每次换机位/换镜头都要做）

```bash
# 取四个场地外角的像素坐标：N11(右下)、N1(右上)、A1(左上)、A11(左下)
python -m perception.cli detect --image shot.png \
    --corners "905.7,718.4;905.7,14.4;8.5,14.4;8.5,718.4" \
    --order N11,N1,A1,A11 --preset map_image --overlay out.png
```

* 想更准：把若干格心也作为对应点一起喂进去（>4 点最小二乘），用 `grid.register(pairs)`。
* 数量/顺序不对、四点共线、镜像顺序都会被 `RegistrationError` 拦下。

```bash
# 颜色标定：在某类目标中心取 3~5 px，打印中位 HSV 并生成调色板
python -m perception.cli calibrate --image shot.png --at 640,360 --radius 4 \
    --cls mine --out field_palette.json
python -m perception.cli detect --image shot.png --corners "..." \
    --palette-json field_palette.json
```

### 2) 检测 + 出报文

```bash
python -m perception.cli detect --image shot.png --corners "..." \
    --messages out.jsonl --json out.json --overlay out.png
```

`out.jsonl` 每行一条报文，例如：

```json
{"type":"heartbeat","ts":1791...}
{"type":"bridge_passable","half":"upper","block":"K5","confidence":0.6,"ts":1791...,"seq":2}
{"type":"target_zone","blocks":["B11","C11"],"ts":...,"seq":3}
{"type":"field_verify","mines":["B1","G2",...],"obstacles":["D1",...],"ts":...,"seq":4}
```

不发空表、不发没把握的结论：检不到就不发那一条（车侧对缺失有兜底，`drone.py:18-21`）。

### 3) 真发 DDS（**在无人机上**，需 ROS 2 Humble）

```bash
python -m perception.cli detect --image shot.png --corners "..." --publish
# 会打印"订阅者匹配数"：0 = 没人订阅 ⇒ 先查 QoS 与 ROS_DOMAIN_ID，别怀疑报文
# 随后 3 s 内打印车侧回传（含 ack{of:<seq>} = 送达硬证据）
```

建议节奏（`protocol.recommended_cadence()`）：心跳 1 Hz；`bridge_passable` 首检即发、
之后每次重检到同结论也重发（续 30 s 窗口）；`target_zone`/`field_verify` 一次检测一条带 `seq`；
**发车前必须发**（Mission 最多等 **2 s**，等不到就用默认 **K10**(下) / **K5**(上)）。
★ 该 2 s 窗口只管「两半场相隔 ≤ 2 s 到达」：**`lower`/`upper` 最好同一条样本一起给出**
（supervisor 已保证任一半场到点时两半场同轮发出，但感知层一次报全能省掉一次往返）。
口径真值：车上 `config/robot.yaml` 的 `mission.bridge_info_wait_s: 546` /
`route.cross_lower_default: 520` / `cross_upper_default: 521`。

---

## 六、测试与自检

```bash
cd dzsmartscout/tools
python -m perception.run_tests                    # 全部 85 项
python -m perception.run_tests grid objects       # 只跑名字含关键字的模块
python -m perception.cli selftest                 # 等价入口（默认跳过最慢的 e2e / 真图用例）
```

85 项全部通过（本机 Windows / Python 3.12 / numpy 2.3.5 / Pillow 12.3.0）。
测试分四层：

1. **合成图单测**（确定性、无随机）：网格/单应/手性、颜色/形态学/连通域、两套几何口径、
   车位姿、箭头方向、协议字段、**出图与对账**（`test_field_map.py`）；
2. **真实图纸回归**：`map_src.png` 逐格复现**冻结点位**（雷/障/靶/通全对）；
3. **官方贴图端到端（t59）**：`map_official.png` **自动标定** → 出图 → 与现役
   `config/map.yaml` **逐条对账 0 差异**（含每格置信度、蓝面守卫回归）；
4. **真实照片正控 + 特征化失败模式**：`p09_1.png` 绿箭头方向（−96° vs 真值 −90°）、
   PCA 会判错、重抖动下方向可能翻、模糊/噪声/分辨率先坏哪一路。

关键设计原则：**拒收必须留理由**（`rejected[].reason`）、**不发没把握的结论**、
**周期性重发是合法续期**、**方向错了也不能影响通行格判定**。

---

## 七、已知限制与待办（不含糊）

1. **现场颜色全部待标定**：只有"黄桥面/绿箭头"有实测证据；雷/障/靶/车顶标记的颜色
   靠 `calibrate` 现场取。`FIELD_DEFAULT` 里的占位规则默认关闭，就是为了不让人误用。
2. **透明靶（"绿色箭头贴纸"以外的桥顶标识形态）未确认**：H1/H2 两种语义都实现了，
   默认 H1（位置）——现场若发现箭头贴在所有格中间、只靠指向表达，就切 `infer_mode="direction"`。
3. **车顶标记需要己方加装**：规则没规定；单板只能给 mod 180° 的 yaw。
4. **未做**：多帧时序滤波/跟踪（本模块是无状态的单帧管线）、雷达-视觉融合、
   相机内参标定（只做平面单应，不需要内参）、真机 DDS 实测（本机无 rclpy；
   `ros_publish.py` 的 QoS/ack 逻辑只做了代码级对照，**未在真机跑过**）。
5. **`cam_probe` 那 5 帧只能证明"当前出图不可用"**：不能用它们评估精度，也不能作为
   阈值调参依据。
6. **管线默认单帧**：帧率、曝光、运动模糊的影响未评估（糊到 lapVar≈45 的帧连 3 px 模糊
   都不如，任何算法都救不回来）。
