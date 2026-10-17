# t65 · dzsmartscout 无人机侧 死代码与重复逻辑审计（纯只读 · 只出清单）

| 元信息 | 值 |
|---|---|
| 执行 | `drone-link`（卡 t65，attempt 1） |
| 时间 | 2026-10-15 21:16（本机时钟） |
| 口径 | 与 t61/t62 同：**AST 精确法**，正则启发式一律弃用；大小写敏感；dunder 不作零引用证据；`from x import *` 不作未用 import 判定 |
| 唯一写入 | 本文件（`_tmp_session/cleanup-audit/t65_dzsmartscout审计.md`）——被审计文件**零改动**（§8 双列 md5 自证） |
| 一句话结论 | **主树真死代码很少**（未用 import 26 条 + 模块级死函数 4 条）；"体积"主要来自**一次性探测脚本**（160 个零调用点）与**镜像/副本**（39 组文件完全相同，78 个文件） |

## 0. 范围、排除域与规模（可复核）

* 扫描根：`D:\User\Desktop\比赛存档\v1\dzsmartscout`
* **排除域**（与卡一致）：`node_modules`、`novnc`、`install/`、`build/`、`log/`、`__pycache__`、`*.bag`、`*.onnx/.engine/.rknn`、`.git`
* 实测规模：排除后 **1869 个文件 / 190.7 MB**；其中文本语料 **875 个 / 4.9 M 字符**；脚本类 **456 个**（主树 364 + `src/**` 92）
* 顶层分布：`src` 1090 · `tools` 355 · `fc-audit` 344 · `dsh-plugin-dzdrone-ssh` 40 · 根 36 · `docs` 2 · `drone-scripts` 2
* 分组口径（后文表格沿用）：**A** 自研主树 ｜ **C** `tools/planner/vendor/**`（t60 单源码同步件）｜ **D** 第三方内置（`.pdfdeps/pypdf`、`fc-audit/src-mirror`、`src/**`）｜ **E** 临时/镜像
* 独立性说明：`dzrobot_v2/**` 仅作"反向 grep 被引用方"，**其代码不在审计范围**（本轮对它的写入 = 0）

---

## 1. ① 死脚本 / 死文件（全库零调用点）

判定法：对每个脚本文件，在**全语料**（排除域外）中按**大小写敏感**串匹配其 `文件名` 与 `茎名`；命中分两类计数——**代码类**（`.py/.sh/.mjs/.js/.json/.yaml/...`）与**文档类**（`.md/.txt/.rst/.html`）。**代码类命中 = 0 ⇒ 零调用点**；文档提及不计为调用点。
扫描命令见 §5-①（Python 只读扫描器，逐文件计数）。

### 1.1 分组计数（合计 **160** 条）

| 分组 | 无任何提及 | 仅文档提及 | 小计 | 代表 file:line | 判读 |
|---|---|---|---|---|---|
| `tools/drone/**` | 33 | 9 | **42** | `tools/drone/cam_probe_psd.py:1`、`tools/drone/check_imu.sh:1` | 机上一/多次性探测脚本，**手动 ssh 调用**（见 §5 注意事项） |
| `tools/link/**` | 12 | 25 | **37** | `tools/link/probe/t4_multicast.sh:1`、`tools/link/probe/t12_verify.py:53` | 链路探针/补丁脚本；`tNN_*` 属历史卡取证 |
| `<root>/*.py·sh·mjs·js·ps1` | 14 | 11 | **25** | `sweep_current.py:1`、`ssh_connect.py:1`、`vnc_handshake.ps1:1` | 无人机接入引导（改域名/换 key/mDNS/VNC 破解）一次性脚本 |
| `dsh-plugin-dzdrone-ssh/**` | 7 | 14 | **21** | `dsh-plugin-dzdrone-ssh/scripts/vnc-log-1617.sh:1`、`.../probe-battery.sh:1` | 插件调试脚本；**`package.json` 的 `scripts` 为空**（实测 `{}`），无 npm 调用点 |
| `tools/_tmp*_*.sh`（tools 根） | 17 | 0 | **17** | `tools/_tmp_car_traj.sh:1`、`tools/_tmp_drone_topics.sh:1` | 命名即"临时"，建议整组归档 |
| `fc-audit/**` | 2 | 7 | **9** | `fc-audit/verify_frames.py:1`、`fc-audit/src-mirror/px4_*/launch/*.launch.py:1` | 镜像 launch（第三方上游）+ 一次性核验脚本 |
| `tools/planner/**` | 1 | 3 | **4** | `tools/planner/tests/test_planner_sync.py:1` | **t60 新交付**：脚本是对外入口，无内部调用点属正常；`test_*` 由 unittest 发现式调用 |
| `tools/_tmp/**` | 3 | 0 | **3** | `tools/_tmp/car_cfg.sh:1`、`tools/_tmp/dzsend.sh:1` | 临时脚本 |
| `.pdfdeps/extract2.py` | 1 | 0 | **1** | `.pdfdeps/extract2.py:1` | PDF 取证脚本（同目录 `pypdf` 为第三方） |
| `drone-scripts/**` | 1 | 0 | **1** | `drone-scripts/start-lidar-on-link.sh:1` | 机上启动脚本，靠人手动跑 |
| **合计** | **91** | **69** | **160** | — | 零调用点 ≠ 可删，见 §5 注意事项 |

### 1.2 无任何提及的 91 条（逐条 file:1，证据 = 全语料 0 命中）

```
.pdfdeps/extract2.py · _pdf_arena.py · _pdf_arena_img.py · _pdf_arena_sheet.py · _pdf_arena_sheet2.py
_pdf_arena_sheet3.py · _pdf_extract.py · _pdf_vnc.py · _probe_thermal.sh · adapt_drone_plugin.mjs
asar_list.js · ascii_icon_check.py · drone-scripts/start-lidar-on-link.sh · fix_mdns_nsswitch.py
run_fix_drone_avahi.py · vnc_handshake.ps1
dsh-plugin-dzdrone-ssh: scripts/check-widget-target.mjs · scripts/probe-battery.sh · scripts/vnc-log-1617.sh
    · scripts/vnc-log-1617b.sh · tmp-fc-noros.sh · tmp-fc-present.sh · tmp-metrics-cmd.sh
fc-audit: fixes/04-camera-bringup/files/make_checkerboard.py · src-mirror/px4_ros_com/launch/sensor_combined_listener.launch.py
tools/_tmp: car_cfg.sh · car_probe_run.sh · dzsend.sh          tools: _tmp_car_chassis_src.sh · _tmp_car_code.sh
    _tmp_car_logs.sh · _tmp_car_logs2.sh · _tmp_car_preflight.sh · _tmp_car_read2.sh · _tmp_car_reset.sh
    _tmp_car_src.sh · _tmp_car_src2.sh · _tmp_car_stop.sh · _tmp_car_traj.sh · _tmp_car_where.sh
    _tmp_drone_gates.sh · _tmp_drone_hist.sh · _tmp_drone_topics.sh · _tmp_drone_values.sh · _tmp_probe_auth.sh
tools/drone: build_and_switch_back.sh · cam_probe_jpegmeta.py · cam_probe_lnv_calib.py · cam_probe_optics.py
    cam_probe_psd.py · check_camera_full.sh · check_fastlio_types.sh · check_imu.sh · diag_xfer.sh
    dump_livox_qos.sh · dzlink_points_check.py · final_state.sh · final_vnc_verify.sh · fix_beacon.sh
    fix_blank_and_capture.sh · lidar_char.py · liveness_check.sh · probe_bridge_shot.sh · probe_dialog_and_rviz.sh
    probe_lidar_cam.sh · probe_net_now.sh · probe_ros_env.sh · probe_vnc_gui.sh · recheck_state.sh
    renew_beacon.sh · report_status.sh · run_probe.sh · start_bridge_bg.sh · start_bridge_cpp.sh
    start_lidar_bg.sh · switch_to_pc2.sh · unlock_vnc1.sh · verify_car_claims.sh
tools/link: car_code_diff.py · car_diff_v1.py · relaunch_dzlink.sh · probe/car_recon2.sh · probe/car_recon3.sh
    · probe/car_recon4.sh · probe/t12_verify.py · probe/t16_drone_final.sh · probe/t18_hub_lineage.sh
    · probe/t4_bridge_state.sh · probe/t4_multicast.sh · probe/t4_writer_recon.sh
tools/planner/tests/test_planner_sync.py
```

抽检复核（`Select-String -SimpleMatch -CaseSensitive`，全树）：`relaunch_dzlink` **0 处** ｜ `check-widget-target` **0 处** ｜ `cam_probe_psd` **0 处** —— 与扫描器一致。

### 1.3 仅文档提及的 69 条（分布）

`tools/link` 25 · `dsh-plugin-dzdrone-ssh` 14 · 根 11 · `tools/drone` 9 · `fc-audit` 7 · `tools/planner` 3。
代表：`tools/link/probe/t18_verify_local.py:1`（我 t18 卡产出，仅 t18 报告引用）、`tools/drone/cam_recon.sh:1`、`enable_mdns.py:1`、`tools/planner/link_status.py:1`。
**判读**：文档提及说明"有人在用/曾用"，但**不构成调用点**；这批主要是**手工调用的探针**，删除前须确认最后一次使用时间（本清单不替其做生杀判断）。

---

## 2. ② 脚本内死函数与未用 import（**AST 精确法**）

### 2.1 方法与"防假阳性"设计（t61 教训）

1. 解析用 `ast.parse()`；**不**用正则扫调用点（t61 的 111 条假阳性全来自多行调用续行）。
2. **未用 import**：只认 AST——`ast.Name` / `ast.Attribute(value=Name)` 视为使用；**排除** `from __future__ import *`（永远无引用）、行内带 `noqa`、`__all__` 中列出的名字、`from x import *`（该文件整体不作判定）。
3. **死函数**：只判**模块级** `def`，条件为**本文件 AST 内零使用 ∧ 跨文件词边界零引用**；排除 dunder、`main`/`setUp*`/`tearDown*`、`test_*`；**仅出现在字符串里**（`getattr`/表驱动）的一律进"不确定项"（§5），不进结论。
4. **修正记录（重要）**：第一版误把"def 行"当成一次使用而多减 1，导致 315 条假"死函数"（如 `tools/perception/cli.py:245 build_parser()` 明明在 `cli.py:307` 被调用）。修正后为 **32 条**；抽检 `build_parser` → `cli.py:245(def)/307(call)` **2 处**，证明修正有效。本报告**只采纳修正版**。

### 2.2 未用 import（主树 26 条；A 类逐条列出）

| file:line | 名字 | 证据（同文件零 AST 使用） |
|---|---|---|
| `crack_vnc_passwd.py:23` | `hashlib` | 全文件无 `hashlib.` |
| `tools/link/car_diff_v1.py:1` | `sys` | 无 `sys.` |
| `tools/link/probe/t4_contract_tests.py:18` | `json` | 无 `json.` |
| `tools/drone/dzlink_livox_bridge.py:22` | `sys` | 无 `sys.` |
| `tools/drone/ref/msg_MID360_launch.py:2,5` | `get_package_share_directory`, `launch` | 上游 livox 参考件（镜像） |
| `tools/drone/ref/rviz_MID360_launch.py:2,5` | 同上 | 同上 |
| `tools/perception/cli.py:48` | `config_for` | 无引用 |
| `tools/perception/field_map.py:44,48,53,53,55,55` | `math`, `asdict`, `Palette`, `blob_masks`, `FIELD_SPECS`, `ObjectClassSpec` | 6 条 |
| `tools/perception/run_field_map_check.py:30` | `np` | 无 `np.` |
| `tools/perception/tests/test_field_map.py:11,16,16` | `re`, `BlueFaceEstimate`, `reconcile_with_map_yaml` | 3 条 |
| `tools/sim_airground/car_gate.py:37,44` | `time`, `parse_map_yaml` | 2 条 |
| `tools/sim_airground/carsim.py:21` | `geo` | 无 `geo.` |
| `tools/sim_airground/link.py:14` | `field` | — |
| `tools/sim_airground/runner.py:16,19` | `RouteVersion`, `meters_to_block` | 2 条 |
| `tools/sim_airground/shoot.py:26` | `field` | — |

计数：**主树 26 条 / 20 个文件**；另 **C-vendor 4 条**（`route/parser.py:36 Sequence`、`route/validate.py:26 Route`、`tools/route_check.py:68 Dict,Sequence`，**属同步件，删改须连同真源与 sync 脚本**）；**D-第三方 9 条**；**E-临时/镜像 40 条**（多为 ROS launch 模板的固定写法，不评）。**总计 79 条 / 39 文件**。

### 2.3 模块级死函数

| 域名 | 条数 | 逐条（file:line 名字） |
|---|---|---|
| **A 自研主树** | **4** | `ascii_icon_check.py:69 sample_ys()` · `tools/perception/grid.py:486 _runs_1d()` · `tools/sim_airground/geo.py:53 block_distance_cells()` · `tools/sim_airground/random_maps.py:70 _fmt_row()` |
| C vendor 同步件 | 15 | 例：`route/planner.py:1097 _collapse()`、`route/parser.py:256 _read_token_file()`、`tools/route_check.py:735 run_check()` —— **同步件，禁单侧删** |
| D 第三方内置 | 2 | `.pdfdeps/pypdf/_page.py:117 _get_rectangle()`、`_page_labels.py:117 number2lowercase_roman_numeral()` |
| E 临时/镜像 | 11 | 例：`src/mavros/mavros/cmd/system.py:152 message_interval()`、`tools/_tmp/car_probe.py:13 on_msg()` |
| **合计** | **32** | — |

四条 A 类逐条 rg 复核（`\b名\b`，大小写敏感，全树）：各 **1 处命中 = 仅 def 行**，与 AST 判定一致。
`tokens`/`_TOKENS` 之类大小写陷阱：本轮全部匹配用 `(?<![A-Za-z0-9_])名(?![A-Za-z0-9_])` + `-CaseSensitive`，未出现假命中。

---

## 3. ③ 重复逻辑与魔法常量多源

| 主题 | 命中 | 代表 file:line | 判读 |
|---|---|---|---|
| **域号 118** | **199 处 / 80 文件** | `docs/无人机项目总览.md:14`、`tools/link/dzlink_supervisor.py:127 DEFAULT_ENV_FILE` | 真相源应为 `dzlink.env`；文档/脚本各写一份，改动需全仓同改（t16 已对齐口径，但**多源**仍在） |
| **话题名 `/dzlink/drone_to_car`** | 122 处 / 36 文件 | `tools/link/dzlink_supervisor.py`、`tools/link/dz_send_waypoint.py`、`fc-audit/无人机技术文档.md:619` | 建议抽 `topics.py` 单源（机制上已有 `CAR_CONTRACT`，但字符串仍多处硬写） |
| **话题 `/nav/waypoint`** | 120 处 / 38 文件 | 同上 + `docs/空地路线下发契约_v2.md:28` | 同上 |
| **话题 `/dzlink/car_to_drone`** | 44 处 / 20 文件 | `fc-audit/fixes/05-*/APPLY.md:32` | 同上 |
| **QoS depth=10 / KEEP_LAST** | 46 处 / 21 文件 | `fc-audit/fixes/05-*/files/_deprecated_*/dz_send_waypoint.py:61`（**已废弃副本也有一份实现**） | 同一 QoS 表在 3 份发送器里各写一遍 |
| **时序常量** `bridge_info_wait`/`auto_start_delay` | 51 处 / 25 文件 | `tools/link/dzlink_supervisor.py:117-121`（CAR_CONTRACT） | 车侧真值在 `dzrobot_v2/config/robot.yaml`；本仓为**镜像口径**，改动需同步 |
| **时序常量** `info_stale`/`heartbeat_offline` | 28 处 / 15 文件 | `tools/link/dzlink_supervisor.py:117,121,952` | 同上 |
| **格子尺度 0.35**（含 `BLOCK_SIZE`） | 63 处 / 21 文件 | `tools/drone/cam_probe_optics.py:26-28`、`tools/perception/bridge_arrow.py:359` | 多份"场地方程"各自实现；`tools/perception` 有 `GridSpec`，其余模块未复用 |
| **网格维度** `MAP_COLS/MAP_ROWS/COL_LETTERS` | 21 处 / 5 文件 | `tools/perception/grid.py:71 COL_LETTERS`、`__init__.py:35` | 相对收敛（仅 perception + 导出） |
| **hash 助手** | 22 处 / 13 文件 | `tools/link/car_code_diff.py:35`、`tools/link/car_diff_v1.py:16`、`fc-audit/.../patch_car_wait_for_drone.py:101 def md5(p)` | **同一职责 13 处实现**（md5/sha256 + 截断 16/20 位口径不一）→ 建议 `tools/lib/hashutil.py` |
| **ssh 连接** | 21 处 / 18 文件 | `install_drone_key.py:12`、`enable_mdns.py:14`、`fix_drone_avahi.py:8`、`fix_mdns_nsswitch.py:9`、`crack_vnc_passwd.py:8`（各写一份 `paramiko.SSHClient()`） | **18 个文件各自造轮子**；`tools/link/probe/t4_ssh.py` 已是较完整实现（exec/run/put/get/bg），未被复用 |
| **"找无人机"逻辑** | 34 处 / 18 文件 | `sweep_current.py:12,20 icmp/tcp22`、`tools/planner/link_status.py:27 ping`、`tools/perception/run_real_checks.py:325,341 _find_drone/_scan_lan_ssh`、`tools/link/probe/t4_ssh.py` | **至少 4 套 LAN 探测/IP 解析实现**（+ 硬编码 `10.29.122/192.168.137/dzsmart.local`）→ 最值得收口的一处 |
| **ping/端口探测** | 3 处 / 2 文件 | `sweep_current.py:12,20`、`link_status.py:27` | 数量不大但语义重复（ICMP + TCP22 探活） |
| **route token 正则** | 14 处 / 5 文件 | `tools/link/dzlink_supervisor.py:403,405,558,559,593,609`、`tools/link/dz_send_waypoint.py`、`tools/perception/protocol.py` | 同一 token 语法（`[A-N](?:1[01]|[1-9])`）在机侧多处重复；**真源是 `dzbrain/route/parser.py`**（vendor 里有，但机侧没复用） |
| **env 文件解析** | 27 处 / 6 文件 | `tools/link/dzlink_supervisor.py:138 _parse_env_file`、`tools/planner/planner_engine.py`（用 `Config`） | 两种实现：手写解析 vs `dzbrain.core.config`；建议统一 |
| **"已废弃副本仍在仓内"** | 1 组 | `fc-audit/fixes/05-drone-to-car-waypoint/files/_deprecated_20261012/dz_send_waypoint.py` | 与现役**同名异体**（t5 已收口并 `DEPRECATED_*.md` 标注）→ 属"留档"，但仍是重复实现来源 |

---

## 4. ④ 近似重复文件

### 4.1 内容**完全相同**（md5 相同）：**39 组 / 78 个文件**

| 组 | 例（file ↔ file） | 判读 |
|---|---|---|
| **镜像对 36 组** | `fc-audit/src-mirror/px4_ros_com/README.md:1` ↔ `src/px4_ros_com/README.md:1`；`fc-audit/src-mirror/FAST_LIO_LOCALIZATION_ROS2/scripts/global_localization.py:1` ↔ `src/...同路径:1` | `fc-audit/src-mirror/**` 与 `dzsmartscout/src/**` **大面积重复**（同一份 ROS 源码两份），建议**择一保留**（src 是被审计本体，src-mirror 是取证副本） |
| 真实代码重复 1 组 | `fc-audit/fixes/06-car-wait-for-drone/files/patch_car_wait_for_drone.py:1` ↔ `tools/link/patches/patch_car_wait_for_drone.py:1` | 同一补丁两处落盘，**改一处必须改另一处** |

### 4.2 近似重复对（difflib ratio ≥ 0.85 且体量相近）：**84 对**（列前 10）

| ratio | 文件 ≤> 文件 | 判读 |
|---|---|---|
| 1.000 | `tools/link/tests/test_dz_send_waypoint.py` ≤> `tools/reports/t12-evidence/test_dz_send_waypoint.py` | 取证目录里**留了一份测试副本**（t12 证据）→ 证据可留，但须标"快照，勿改" |
| 1.000 | `src/livox_ros_driver2/launch_ROS2/rviz_MID360_launch.py` ≤> `tools/drone/ref/rviz_MID360_launch.py` | `tools/drone/ref/**` 是上游 launch 的**参考拷贝** |
| 1.000 | `src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py` ≤> `tools/drone/ref/msg_MID360_launch.py` | 同上 |
| 1.000 | `src/livox_ros_driver2/launch_ROS2/rviz_MID360_launch.py` ≤> `.../rviz_MID360s_launch.py` | 上游内部近重复（第三方，不评） |
| （其余 80 对） | 绝大多数是 §4.1 的镜像对与 `tools/drone/ref/**` 参考件 | 详见 §5 复跑命令 |
| 设计上的"完全一致" | `tools/planner/vendor/dzbrain_ws/**` ↔ `dzrobot_v2/**` | **不许删**：t60 单源码同步件，靠 `HASH_LEDGER.json` + `--verify-only` 对账（本轮复核：漂移 0/缺失 0） |

---

## 5. 不确定项（**不计入结论表**）

1. **"仅字符串引用"5 条**（可能是 `getattr`/表驱动调用，故不作死函数结论）：
   `fc-audit/verify_frames.py:102 enu_to_ned_local_frame` · `tools/drone/cam_probe_metrics.py:47 tenengrad` ·
   `tools/drone/cam_probe_metrics.py:55 edge_density` · `tools/link/probe/t4_contract_tests.py:85 s1b_drone_topicinfo` ·
   `tools/planner/vendor/.../validate.py:216 _blocks`。
2. **① 表全部条目的共同前提**：本轮"调用点"只能看到**落在文件里的**引用。**手工 ssh 执行**（如 `python probe/t4_ssh.py run drone probe/x.sh`）**不产生调用点** ⇒ 零调用点 **≠** 未使用。§1 的每一行都须由人确认"最后一次用是什么时候"。
3. **测试文件不算死文件**：`tools/planner/tests/test_planner_sync.py:1`、`tools/perception/tests/**`、`tools/link/tests/**` 由 unittest 发现式调用（无脚本级调用点）。本轮已从"可删"语义里剔除，仅作 ① 的零调用点事实列出。
4. **3 个文件带 UTF-8 BOM**（`.pdfdeps/extract.py:1`、`.pdfdeps/extract2.py:1`、`tools/planner/vendor/.../route/planner.py:1`）：`ast.parse` 需 `utf-8-sig` 才能解析；**不影响 CPython 运行**，不列为缺陷。
5. **未评域（本轮明确不覆盖）**：`src/**` 与 `fc-audit/src-mirror/**` 的 **C++/msg/srv/xacro**（199 `.cpp` / 475 `.msg` / 51 `.srv`…）、`.pdfdeps/pypdf`（第三方库）、`novnc/node_modules`（第三方）——四类发现**只针对 Python/Shell/JS 脚本与文档**。
6. **`tools/planner/vendor/**` 特注**：一切发现（含 §2.2 的 4 条未用 import、§2.3 的 15 条死函数、§4.2 的"完全一致"）**都是真源 `dzrobot_v2` 的镜像事实**，删改须**连同真源与 `sync_planner_from_source.py` 一起**考虑，**禁止单侧删**。另：审计进行中真源 `route/planner.py` 被队友改动（21:16:01），vendor 已落后一步 —— 见 §8 末注，需 t60 重跑 sync 吸收。

---

## 6. 汇总计数

| 类别 | 自研主树(A) | vendor 同步件(C) | 第三方内置(D) | 临时/镜像(E) | 合计 |
|---|---|---|---|---|---|
| ① 死脚本/死文件（零调用点） | 160（含 69 条仅文档） | 0 | —（未评） | —（未评） | **160** |
| ②A 未用 import | **26**（20 文件） | 4 | 9 | 40 | **79**（39 文件） |
| ②B 模块级死函数 | **4** | 15 | 2 | 11 | **32** |
| ③ 重复逻辑/常量多源 | **16 组主题**（含 4 套"找无人机"、18 文件各写 ssh、13 处 hash） | — | — | — | 16 |
| ④ 完全相同组 / 近似对 | 1 组真实代码重复 + 84 对近似 | 1 组（设计一致，勿删） | 37 组镜像 | — | **39 组 / 78 文件**、**84 对** |

**最值得处理的 5 件事**（按性价比排序，均需人确认后另开卡）：
1. `fc-audit/src-mirror/**` 与 `dzsmartscout/src/**` 择一（36 组文件完全相同，占全部重复体量的大头）。
2. "找无人机/探活"四套实现（`sweep_current.py`、`tools/perception/run_real_checks.py:325,341`、`tools/planner/link_status.py`、`tools/link/probe/t4_ssh.py`）→ 收口成一份 + 一份 IP/域名来源。
3. 18 个文件的 `paramiko.SSHClient()` 各自造轮子 → 复用 `tools/link/probe/t4_ssh.py`。
4. hash 助手 13 处 → 一份（顺带统一 16/20 位截断口径，避免台账对不上）。
5. `tools/_tmp*` / `tools/_tmp/**` 共 20 条 + `_pdf_*.py` 6 条 → 归档到 `_tmp_session/` 或直接删（**需先确认无人再跑**）。

---

## 7. 边界声明

* **未改、未删、未重命名任何被审计文件**；本卡**唯一写入** = 本文件。
* 排除域：`node_modules`、`novnc`、`install/`、`build/`、`log/`、`__pycache__`、`*.bag`、`*.onnx/.engine/.rknn`、`.git`（另把 `.pdfdeps`、`fc-audit/src-mirror`、`src/**` 作为"第三方/镜像"域单列，只做§4.1 的相同性统计，不做代码级评判）。
* **未碰 `dzrobot_v2/**`**（仅作反向 grep 的"被引用方"；本轮对其写入 0）。`tools/planner/vendor/**` 只读。
* 无 git 写操作；未起任何 ROS/车端栈；未登车。
* 复跑命令（逐条已实际执行）：
  * ① 目录清点：`Get-ChildItem dzsmartscout -Recurse -File | Where-Object { $_.FullName -notmatch 'node_modules|novnc|__pycache__|install|build|log' } | Group-Object Extension`
  * ① 零调用点扫描：内联 Python 扫描器（全语料大小写敏感串匹配，分代码/文档两类计数）
  * ②A/②B：内联 AST 扫描器（`ast.parse` + `ast.Name`/`Attribute` 使用集 + 跨文件 `(?<![A-Za-z0-9_])名(?![A-Za-z0-9_])` 计数）
  * ②B 抽检：`Select-String -Pattern "\bsample_ys\b" -CaseSensitive`（同法核对 `_runs_1d`/`block_distance_cells`/`_fmt_row`/`build_parser`）
  * ③：内联正则扫描器（16 个主题，逐主题打印"命中处数/文件数"+ 前 6 处 file:line）
  * ④：`hashlib.md5` 分组 + `difflib.SequenceMatcher().ratio() ≥ 0.85`（体量比 ≥0.6、单文件 ≤120 KB）
  * ④ 基线：`[Security.Cryptography.MD5]::Create().ComputeHash()` 逐文件（见 §8）

---

## 8. md5 基线（审计前 T1 / 审计后 T2）

T1 = 审计开始后的首个只读快照（本次实测于 21:16 前）；T2 = 写完本报告后复测（21:17）。**两列逐条相同 ⇒ 本次审计对所有被审计文件零改动**（mtime 亦逐条未变）。
"md5 (T2)" 列写"（同左）"= 与 T1 逐字节相同（实测值一致，非推断）。

| 文件（dzsmartscout/ 下） | md5 (T1) | md5 (T2) | 一致 |
|---|---|---|---|
| `tools/link/dzlink_supervisor.py` | `5c7426d6b619da2f4b69bb3149de2d53` | （同左） | ✅ |
| `tools/link/dz_send_waypoint.py` | `eb41dbf4b944bc7e8af8e61ec940a228` | （同左） | ✅ |
| `tools/planner/sync_planner_from_source.py` | `77b379e9952734d3f7baa4c2b845edc9` | （同左） | ✅ |
| `tools/planner/run_plan_and_compare.py` | `e7492f2fccbc6e0373c21ea32766de82` | （同左） | ✅ |
| `tools/planner/planner_engine.py` | `e039d4dc4e9e87af385604e9512f4e21` | （同左） | ✅ |
| `tools/planner/HASH_LEDGER.json` | `a10e68a26f5067df78d15847e9922726` | （同左） | ✅ |
| `tools/perception/pipeline.py` | `f4e0a09724a647a583e4c1ac0050dbf3` | （同左） | ✅ |
| `tools/perception/protocol.py` | `e9578d1e3421d71be87deae5bfef17f3` | （同左） | ✅ |
| `tools/perception/run_tests.py` | `acea19f4880057fb1f131bd25125143f` | （同左） | ✅ |
| `tools/drone/dzlink_livox_bridge.py` | `98efafe8cc83791305bdf221ea93d781` | （同左） | ✅ |
| `docs/空地路线下发契约_v2.md` | `474517298a1fb2061ec6756be145d480` | （同左） | ✅ |
| `docs/无人机项目总览.md` | `e0f4f980949266f3dea2e0fd6622e3ff` | （同左） | ✅ |
| `dsh-plugin-dzdrone-ssh/index.js` | `092ebf09deb75054ce173a9a53ee754a` | （同左） | ✅ |
| `dsh-plugin-dzdrone-ssh/package.json` | `95bda1b011532fae62f71c46390efadc` | （同左） | ✅ |
| `tools/planner/vendor/.../route/planner.py` | `fb8e420b5b311f815570f83ba0adf293` | （同左） | ✅ |
| `tools/planner/vendor/.../config/robot.yaml` | `b5137ebf879211c536445652c931eeb5` | （同左） | ✅ |

> **⚠️ 审计期间发现的"在途漂移"（属 t60 维护动作，本卡只报不改）**：
> 用 t60 的独立对账门禁复跑 `sync_planner_from_source.py --verify-only`（21:17:14）得到 **漂移 1 条 / 缺失 0 条**：
> `src/dzbrain/dzbrain/route/planner.py` 真源哈希 `78451e47a2e2…` → `1e35477d75b3…`，
> 真源 mtime **21:16:01**（= 本审计进行中，**由队友在 `dzrobot_v2` 侧改动**；本卡对 `dzrobot_v2/**` 写入为 0）。
> ⇒ `tools/planner/vendor/dzbrain_ws/**` 当前已落后真源一步，需由 **t60 的 sync 脚本重跑**吸收（并连带更新 `HASH_LEDGER.json`）；
> **不得**在本卡里顺手重跑（会改 vendor 树，违反"唯一写入"）。这也再次说明：**并发写入者存在时，用之前必须先 `--verify-only`**。
