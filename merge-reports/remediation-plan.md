# air_ground 分级整改计划

**配套**：`architecture-review.md`（证据）、`risk-map.dot`（失效链）
**排期原则**：按"外场演示前的时间闸门"分层，不按严重度分层。每项含**验收标准**——没有验收标准的整改等于没做。

> 总工时估算：**阶段 0 ≈ 2h，阶段 1 ≈ 1.5 人日，阶段 2 = 现场清单（0 开发），阶段 3 ≈ 3-5 人日（赛后）**。
> 如果只有 4 小时，做阶段 0 全部 + 阶段 1 的 #5 #6 #7。

---

## 阶段 0 — 今晚就能做完，不动任何算法（≈2h）

这 6 项都是"改几行 / 跑几条命令"，但每一项都堵一个 P0。

### 0-1 把权威源定下来，重打两个上机包 `P0-1`
**动作**：以 `路径规划交接_20261017/src/ag_integrate/`（1585 行版）为唯一权威源，覆盖到 `无人机上机包_20261017/03_机载代码与工具/air_ground/src/ag_integrate/` 与 `车端上机包_20261017/01_车端大脑ag_integrate/src/ag_integrate/`，并把 `tests/` 一并随包。
**验收**：三处 `md5sum ag_integrate/mission_router.py` 完全一致；部署包内 `pytest tests/ -q` 全绿（尤其 `test_planning_hardening.py` 能 import 成功）。
**风险**：1585 行版含 `_verify_zone_covered`，它会在规划不覆盖必经格时抛异常。**先跑一遍测试确认异常路径有兜底**，否则会把"安静漏打一个区"换成"直接规划失败"——后者更好，但要配合 0-3。

### 0-2 `git init` 一次全量快照 `P0-2`
**动作**：在 `Trae-CN/default/` 下 `git init`，先写 `.gitignore`（排除 `.venv_dml/ __pycache__/ *.mp4 *.tar *.zip .pytest_cache _tmp_*`），然后一次提交。
**验收**：`git log --stat` 能看到全量文件；`git status` 干净；仓库体积 < 20MB（视频/zip 已忽略）。
**为什么是 P0**：这是唯一能让"哪份是真的"这个问题从此有答案的动作，成本 15 分钟。

### 0-3 决策节点加异常兜底 + respawn `P0-5`
**动作**：
- `integrate_node.py:581` 的 `except (ValueError, IndexError, KeyError)` → 外层再包一层 `except BaseException`，记黑匣子 + `get_logger().error(exc_info=True)`，**不重抛**。
- `mission_router.py:623` 的生产 `assert best_route is not None` → 改成显式 `if best_route is None: raise PlanningError(...)`（assert 在 `python -O` 下会被剥掉）。
- `launch/integrate.launch.py:29-44` 加 `respawn=True, respawn_delay=2.0`。
**验收**：往 `_plan_core` 注入一个 `raise RuntimeError` 的单测/手动 patch，节点**不退出**、日志有完整 traceback、路线发布暂停但心跳继续。

### 0-4 NaN 双检 + ESTOP `P0-3`
**动作**：`executor_node.py:464-468` `_publish` 入口加：
```python
if not (math.isfinite(v) and math.isfinite(steer_deg)):
    self.get_logger().error(f"NON-FINITE cmd v={v} steer={steer_deg} -> ESTOP")
    self._enter_estop("non_finite_command")
    v, steer_deg = 0.0, 0.0
```
**验收**：单测注入 `float('nan')` 与 `float('inf')`，断言 `/cmd_vel` 输出 `(0,0)` 且状态机进入 ESTOP 闩锁（需显式 reset 才恢复）。

### 0-5 SIGTERM 归零 `P0-4`
**动作**：`executor_node.py` 的 `main()` 注册信号处理：
```python
import signal
def _on_term(signum, frame):
    node.pub_cmd.publish(Twist())      # 归零
    node.destroy_node(); rclpy.shutdown()
    raise SystemExit(0)
signal.signal(signal.SIGTERM, _on_term)
```
**验收**：车跑动中 `kill -TERM <pid>`，`ros2 topic echo /cmd_vel` 观察到一条全零消息后进程退出。**这一条必须实车验证，不能只看代码。**

### 0-6 删密钥、改口令 `P0-7`
**动作**：
1. 删除 `_tmp_pw.txt`；
2. `无人机上机包_20261017/README.md:26` 的 `set AG_ORIN_PASS=duzhong` → `set AG_ORIN_PASS=<见口令本>`；`:3` 同样处理；
3. 约 10 个 paramiko 硬编码 `password="duzhong"` 改 `os.environ["AG_ORIN_PASS"]`；
4. 删除 `dk_train_pack_20261016/car_probe.py:13` 的密码字典逻辑；
5. 改掉 Orin 的登录口令（用户名==密码是当前最大问题），改用 SSH key；
6. 重打所有已分发的 zip。
**验收**：`grep -rn "duzhong" .`（排除 `.venv_dml`）在**密码语境**下 0 命中；用户名可保留（README 里写 `--user duzhong` 不算泄密），密码不可。

---

## 阶段 1 — 外场前必须（≈1.5 人日）

### 1-1 禁火 TTL 改用本地单调时钟 `P1-1`（**最高优先，改动最小，防的是最坏后果**）
**动作**：`world_model.py:98-109` 的红靶粘性否决 TTL，**不要用 `frame.timestamp_ms`**，改记本地 `time.monotonic()` 收包时刻：
```python
# 收包线程里
self._recv_mono = time.monotonic()
# world_model 里判过期
if self._recv_mono - veto_mono > self.red_veto_ttl_s: 清除
```
配套：`main.cpp:183-185` 的 `system_clock` → `steady_clock` + 启动时一次性对齐 epoch（协议字段语义不变，只是变成单调时基）。
**验收**：构造帧序列，时间戳在中间**前跳 10s**，断言红靶否决**仍然生效**；再构造**回跳**帧，断言不会让否决永不过期。
**为什么排第一**：这是唯一一个"失效方向是开火打友方"的问题，且触发条件（无 NTP + 重启时钟跳变）项目**已经实测到过**。

### 1-2 未定位也发心跳 `P1-2`
**动作**：`main.cpp:186-187` 去掉 `if (localized || getenv(...))` 条件，**始终发 0x4001**（目标列表可为空），协议里加一个 `localized` 状态位（或用现有 reserved 字段）。车端按"N 秒无帧"进 failsafe。
**验收**：遮住所有格号让 `localized=0`，地面 `monitor_link` 仍持续收到帧且状态位显示未定位；拔网线 2s 后车端进 failsafe。
**注意**：改协议字段要同步 `intel_protocol.py` 的 `build()`/`parse()`，两侧版本必须一起换（这正是 P0-2 无 VCS 的代价）。

### 1-3 车端断链看门狗 + `/estop` `P0-4`
**动作**：
- 订阅一个无人机心跳话题（或直接复用 0x4001 的到达），**超时 ≤1s 即停车并闩锁**；
- 新增 `/estop` 订阅，QoS `TRANSIENT_LOCAL`（latched），收到即停车闩锁；
- 现有的 180s `nav_timeout`（executor_node.py:66,321-324）**保留但降级为兜底**，不能作为主看门狗——3 分钟太长了。
**验收**：车跑动中拔掉无人机侧网络，**≤1s 内车停**；`ros2 topic pub --once /estop std_msgs/msg/UInt8 "data: 1"` 能立刻停车且需显式 reset。

### 1-4 Held-Karp 规模上限 + 降级 `P0-6`
**动作**：`mission_router.py:547` 前加：
```python
if n > self.max_tsp_n:            # 建议 12
    self._log.warn(f"n={n} > {self.max_tsp_n}, 降级为最近邻贪心")
    return self._greedy_order(...)
```
并把规划从 10Hz timer 回调里移出（独立线程 + 结果队列，或 `MultiThreadedExecutor` + `ReentrantCallbackGroup`）。
**验收**：注入 22 个误检目标，规划在 **<100ms** 内返回（贪心解），且 `/cmd_vel` 在规划期间**不中断**。

### 1-5 `monitor_link.py` 修枚举 `P1-7`（5 分钟，但比赛日会崩）
**动作**：`monitor_link.py:47-53, 57-59` 的 `ObjectType.MINE` / `ObjectType.ARROW` 按现行 13 类枚举重写：`PAPER` 按落格列分流（K 列=箭头，其余=雷，语义在 `world_model.py:84-87`），并补上 `BUCKET_RED/GREEN/CYAN`、`CAR_START/END`、`UAV_START`。顺手把 `:191` 的 `__import__('math')` 改成正常 import。
**验收**：`python -c "import monitor_link"` 不抛异常；喂一段录制的 UDP 流，态势图能正确渲染雷/箭头/桶。
**加一条 CI 式检查**：写一个 5 行的脚本，遍历 `tools/*.py` 做 `import` 冒烟测试。**这次是 monitor_link，下次是别的**——工具脚本没有任何导入检查，是同一类 bug 的温床。

### 1-6 `DZ_NAV_CONFIG` 接上 `P1-5`
**动作**：`dznav_deploy.py` 的 `_start_dznav`（:143-147）启动命令前加环境变量：
```
DZ_NAV_CONFIG=~/dznav/config/dznav_default.yaml DZ_AUTO_START=0 ...
```
并确认 yaml 确实被推到 `~/dznav/config/`。
**验收**：改 yaml 里一个明显参数（如 `v_max: 0.10`），重启后车**明显变慢**。变慢 = 配置生效。

### 1-7 QoS 逐对核对 `P1-14`（10 分钟，收益极高）
**动作**：上机跑
```bash
for t in /odom /imu_data /cmd_vel /nav/waypoint /odom/localization /dzlink/drone_to_car; do
  ros2 topic info -v $t; done
```
把每对 pub/sub 的 Reliability / Durability / History depth 记下来。**传感器流（/odom、/imu_data）订阅端应改 `BEST_EFFORT` + `depth 1`；`/cmd_vel` 发布端应 `depth 1`**（depth 10 会积压历史转向命令）。
**验收**：`ros2 topic hz /odom` 在 executor_node 运行时能看到数据；一张表列出全部话题的 QoS 并归档进 README。
**为什么必须现场做**：QoS 不兼容的症状是"订阅者存在但一帧都收不到，且无任何报错"，仓内静态审查无法闭环。

### 1-8 preflight 变成硬门禁 `P1-10`
**动作**：
1. 修 `preflight.py:200` 的路径口径 → `~/dzuav_deploy/air_ground`（与 `orin_deploy.py:44` 对齐）；
2. `orin_bringup.sh` 起飞链路**第一步调用 preflight，退出码非 0 就 `fail()` 中止**（现在是纯打印，无人消费退出码）；
3. 补最关键的 5 项缺口（按性价比排序）：**TensorRT 版本 ↔ 引擎匹配**（`trtexec --version` + 记录进黑匣子首条事件）、**ROS_DOMAIN_ID 冲突**、**磁盘余量**、**钟差**（`timedatectl`，见 1-9）、**话题级存活**（`ros2 topic hz /fmu/out/*` 而非只查端口监听）。
**验收**：故意把一个引擎文件换成旧版本，preflight **拦下并阻止起飞**；故意把 `ROS_DOMAIN_ID` 设成与车端不同，preflight 报错。

### 1-9 手动对时 + 钟差入黑匣子 `P1-12`
**动作**：起飞前跑一次（写进 README 起飞清单）：
```bash
sudo timedatectl set-ntp false
sudo date -s "$(ssh car 'date -u +%Y-%m-%dT%H:%M:%S')"   # 或以开发机为准
```
并把 `date` 输出与钟差写进 `orin_flight_blackbox.py` 的**首条事件**。
**验收**：坠机复盘时，能用首条事件的钟差把两机日志换算到同一时间轴。
**成本**：约 5 分钟。**这是全计划里性价比最高的一项**——它同时救了 P1-1（禁火 TTL）、P1-12（复盘）、`monitor_link.py:107` 的延迟读数（现在是噪声）。

### 1-10 修黑匣子 `P1-13`
**动作**：进度纪要待办#3 明确写着"flight_bb 只记 2~3 行（待在线修，源码在机）"——**这是当前唯一的事后取证手段，而它是坏的**。上机第一件事就修它。同时：
- 加大小上限 + 轮转（10Hz JSONL 长时间飞可写满 eMMC）；
- `orin_log_capture.py:127-128` 只 flush 从不 fsync → 关键路径补 fsync（`orin_flight_blackbox.py:331` 的分级 fsync 策略本身是对的，照抄）；
- `:210` 回放"假设 30fps"造假时间戳 → 用真实帧时间戳（注意实测只有 6fps，假设 30fps 会把时间轴压缩 5 倍）。
**验收**：飞一次，黑匣子行数 ≈ 飞行秒数 × 记录频率（不是 2~3 行）；`kill -9` 后已写内容不丢。

### 1-11 `field_frame.py:53` 死守卫 `P1-8`
**动作**：`max_residual` 默认 `0.5` → `0.35`（`round()` 后残差 ≤0.5 恒成立，必须 <0.5 才有效）。
**验收**：单测——输入偏离格心 0.45 格的点，**断言抛 ValueError**（现在不抛）。

### 1-12 PnP 门槛与 `isfinite` `P1-9`
**动作**：
- `MapLocalizer.cpp:99` PnP 的 `max(4, 30%)` → 建议 `max(5, 30%)`，与你们在单应路径（:153）已经采用的标准一致，理由你们自己已经写清楚了；
- 加 leave-one-out 一致性检查（去掉任一地标重解，位姿变化超阈即拒绝）；
- 同一 `CellId` 多观测去重（取置信度最高，或冲突即报警）；
- `MapLocalizer.cpp:164-166` `project()` 输出加 `std::isfinite` 检查，非法点丢弃该检测（`main.cpp:172-175` 组帧前也无检查）。
**验收**：把 `test_localized_gate.py` 的 6 场景扩展两个：4 地标含 1 误读 → 断言 `localized=0`；退化单应产生 inf → 断言不进 UDP 帧。

### 1-13 相机看门狗 `P1-3`
**动作**：`main.cpp:328` 的 `capture.read()` 加超时保护——独立看门狗线程监测"距上次成功 read 的时长"，超过 N×帧间隔（建议 N=5）即告警并触发重开；重开前显式 `capture.release()`（:343 缺）；`:350-354` 的"只重开一次，二次失败 return 4"改成指数退避多次重试，且退出前发一条"离线"告警帧；用 systemd/supervisor 拉起。SIGINT 打不断阻塞 read（:387 只置 flag）——改带超时的 appsink pull 或 `waitAny`。
**验收**：`kill -STOP` argus 进程模拟 ISP 冻死，dzuav 在 N×帧间隔内**告警并重开**，而不是永久静默。

### 1-14 收包线程与就绪检查的 fail-open `P1-4`
**动作**：
- `integrate_node.py:298-299` `except OSError: break` → break 前记 `fatal` + 黑匣子事件，并让节点进入"情报丢失"显式状态；
- `:641-642` `except Exception: return True` → **改 `return False`** 并记 warning（fail-open 是错的：任何异常都被解释成"对端已上线"，路线照发进真空）；
- `:316` `_car_true_yaw` 加收包时刻，超 1s 视为无效，无效时 align 不启动并告警；
- 加全局"N 秒无帧"周期告警（现在 `_log_hold_once` 同一 reason 只打一次，链路断了看不出来）。
**验收**：拔掉 UDP 源，节点在 N 秒内**周期性**告警（不是只一次）；`_waypoint_consumer_ready` 抛异常时路线**不发**。

---

## 阶段 2 — 上机日现场清单（0 开发，纯执行）

按顺序，每步都有判据。**这份清单本身就是交付物**，建议打印出来。

| # | 动作 | 判据（不满足就停，不要往下走） |
|---|---|---|
| 1 | 确认板上 dzuav 二进制的编译源 | `python fix_localized_gate.py --probe` 用 strings 特征串确认家族与编译源。**进度纪要说"live 程序可能是 main.cpp 的带 HUD 变体"——这条不确认，后面所有"我改过了"都不可信** `M13` |
| 2 | 对时 | `timedatectl` 两机钟差 <1s，写进黑匣子首条事件 `1-9` |
| 3 | 飞控 uXRCE 链路 | **这是项目自己记录的唯一断点**。单独断飞控供电 5s 再上电（整机重启不断它——独立供电），然后重启 Agent，验证 `LINK_OK` |
| 4 | ROS_DOMAIN_ID | 两机域值一致，`ros2 node list` 能看到对端节点 |
| 5 | QoS 核对 | 按 1-7 的表逐对确认，`ros2 topic hz /odom` 有数据 |
| 6 | 跑 preflight 且**退出码为 0** | 不是"打印看起来没问题"，是 `echo $?` == 0 `1-8` |
| 7 | 防双实例 | `ros2 topic info /localization` **Publisher count == 1**（项目已实锤过双 FAST-LIO 同发 406 秒的事故；注意 `pgrep` 进程名 15 字符截断会误报，**必须以 Publisher count 为准**） |
| 8 | `DZ_AUTO_START=0` | 调试期必须为 0，否则默认 45s 自动出发跑 43 航点 |
| 9 | wheelbase 实测 | 卡尺量轴距，与 `servo.py:26` / `executor_node.py:60` / `yaml:7` / `contract.py:32` 的 0.157 比对（`contract.py:32` 自注"发学员版为 0.17"）`Q2` |
| 10 | 相机分辨率口径统一 | `orin_bringup.sh:28` = `1920x1200@30` 而 `orin_run_dzuav.sh:4,39` = `1920x1080@30`，C++ 内参基准写死 1920×1080（`MapLocalizer.cpp:18-19`）。**统一成 1920×1080**，否则内参失配 |
| 11 | 急停演练 | 车跑动中：`kill -TERM` → 观察 `/cmd_vel` 归零；`ros2 topic pub /estop` → 车停；拔网线 → ≤1s 停车 `0-5 1-3` |
| 12 | NaN 演练 | 单测/手动注入 NaN，确认车**停**而不是满舵 `0-4` |
| 13 | 悬停验证 | 项目自己的判据：0.5m / 10s / 位移 <0.3m / 零突跳 |
| 14 | 黑匣子行数 | 飞完立刻检查行数 ≈ 时长×频率，**不是 2~3 行** `1-10` |

---

## 阶段 3 — 赛后结构性整改（≈3-5 人日）

这些不会导致比赛失败，但决定了这个项目能不能继续演进。

### 3-1 消灭参数多源 `P1-6`（**最高价值**）
**问题**：`contract.py:32-36,85-90` 定义了 `WHEELBASE / STEER_LIMIT_DEG / CMD_KEEPALIVE_S / deg / rad`，但 **grep 证实全包零引用**——契约模块里的常量没人用，各模块各自硬编码。这就是 wheelbase 出现 4 次、`v_max` 0.32 vs 0.30 的根因。
**动作**：让 `contract.py` 成为**唯一真源**，所有模块从它导入；yaml 只覆盖、不重复定义。`_MID_BOUNDARY` 同理（现在在 `mission_router.py:50`、`match_params.yaml:27`、`dzbrain_bridge.py:269` 三处，挡板行错位 = 路线撞墙）。
**验收**：`grep -rn "0.157\|0\.17" src/` 只剩 1 处；`test_field_config_consistency.py`（已存在于交接包）随包并常绿——**这个测试的意图完全正确，只是从来没被部署**。
**附带修**：`executor_node.py:86-90` `_deep_update` 只覆盖已存在键，**yaml 拼错的键静默丢弃** → 加未知键告警。`route_time.py:25` 的 `SOFT_CAP_S=165` 改用可配置的 `time_soft_cap_s`（现在改参数后 `over_soft_cap` 标志失真）。

### 3-2 拆 god 函数 `P3`
实测行数：`IntegrateNode.__init__` **219 行**（:62-280）、`plan_route` **201 行**（:687-887）、`_plan_core` 147 行；C++ `processFrame` 124 行（main.cpp:96-219）、`MapLocalizer::solve` 110 行、`runCamera` 107 行、`HbbDetector::infer` 99 行；`refpath.build_reference` 111 行、`pursuit._track` 93 行。
`IntegrateNode.__init__` 219 行是最该拆的——它同时做 socket 建立、参数声明、pub/sub 创建、线程启动、模式选择。

### 3-3 清死代码与契约谎言 `P3`
- `evidential_map.py` / `ground_projection.py` / `anchored_registration.py` 三整模块无 import → **要么删掉，要么接线**。注意 `evidential_map.py:107-121` 实为 log-odds 朴素累加而非 DS 融合，有三个真 bug（正证据无饱和上限、同一静止目标逐帧重复计数违反独立性假设、`_last_t` 按格共享导致任一观测重置该格所有类别衰减），**接线前必须先修**。
- `main.cpp:170-171` 注释自认 length/width 字段填像素而非米，"车端忽略所以无实害"→ **要么修协议，要么改字段名**。契约谎言一定会被下一个人踩。
- `integrate_node.py:391` `if self.last_route is None` 永假（:219 初始化为 `""`）。
- `car_adapter.py:26-41` `_target_key` 原样定义两次（第二个覆盖第一个）。
- `executor_node.py:142-143` `_has_odom`、`_last_cmd` 写后永不读。
- `mission_router.py:1001-1026` `compress_route`、`heading_calibration.py:38-85` `estimate_bias`/`corrected_heading`/`ProbeResult` 无调用方。
- `main.cpp:1` 与 `:9` 重复 `#include <algorithm>`；`HbbDetector.cpp:13` 与 `CellClassifier.cpp:12` 各有一份复制粘贴的 `dimsToString` → 提为公共头。
- `HbbDetector.cpp:85-86` 与 `CellClassifier.cpp:85-89` 两处 CHW 转换做法不一致（前者 `blobFromImage` 可能重分配 `blob_` 导致 pinned 内存静默失效、H2D 退化为 pageable 拷贝；后者手写直填 `h_input_`）→ **统一成手写直填**。

### 3-4 性能：把 6fps 提上去 `P2`
实测 fps=6.0~6.6（Orin NX + D13 引擎 90-150ms/帧）。三个方向按性价比：
1. **TTA 合批**：`CellClassifier.cpp:68-97` 每个格号 4 次 TTA 推理全串行（各自 enqueue + streamSync）→ 合成 batch=4 一次 enqueue。这是最大的一块。
2. **隔帧做格号分类 + 结果缓存**：格号是静态的，不需要每帧重算。
3. **热循环日志降频**：`main.cpp:204-216` 每帧 `cout` 手拼 JSON；`:186` 每帧 `getenv`（启动时读一次）。
4. 中期：检测/分类/定位/发送做流水并行（现在整条单线程）。
**验收**：先加 FPS 打点（各阶段耗时分解），**有了数据再优化**——现在无法证伪任何假设。

### 3-5 交付包卫生 `P3`
- 清出约 70MB 垃圾：`recording_*.mp4`(18+21+14MB)、`field_scan_2.mp4`(4.4MB)、`field_pack.tar`(11MB)、14 个散落脚本（`_tmp_diag.py`/`_iter1-3.py`/`_check.py`/`evidence.py`/`_render_*.py`）、14 个 `__pycache__`、2 个 `.pytest_cache`。
- `README.md:61` 与 `L3B_RUNBOOK.md:45,52,90` 指向 `C:\Users\29212\Documents\kimi\tasks\...` —— **包外绝对路径**，换台电脑按 README 走步骤 4/5/6 全部报"找不到文件"。改成包内相对路径。
- `orin_bringup.sh:24` 补 `set -e`；`:61-71` 固定 `sleep 3/2` 改就绪探测；`:66` 硬编码 `/dev/ttyACM0` 改按 by-id 或枚举匹配（进度纪要已实锤"飞控上电晚于 Orin 时 ttyACM0 重新枚举，Agent 旧 fd 失效且 wdog 不会重启活着的 Agent"）；`:48` dzuav 二进制缺失仅 warn 照样起窗口 → 改 fail。
- 权限位统一（`orin_deploy.py`/`remote_run.py`/`preflight.py` 644 vs `orin_flight_blackbox.py`/`monitor_link.py` 755）；zip 内文件名 GBK 编码 → 改 UTF-8（跨平台解包隐患）。
- `FILE_MANIFEST` 头写"共 69 个文件"而 zip 实有 77 条目 → 统一口径。
- 车端 IP 三个矛盾默认值（`preflight.py:221`=192.168.1.188 / `dznav_deploy.py:14`=192.168.0.198 / `deploy_pose_rknn.py:521`=192.168.0.199）→ 统一，且**无 `--host` 时报错而不是猜**。

### 3-6 补依赖钉扎 `P1-11`
`requirements.txt`（或 `environment.yaml`）+ 在 README 记录 JetPack / CUDA / **TensorRT** / ROS2 distro / Python / OpenCV / PyTorch 版本。`orin_convert_engines.sh` 转换时把 `trtexec --version` 写进引擎旁的 `.meta.json`，加载时比对。**TensorRT 引擎与版本+GPU 强绑定**（脚本头注释 :4-6 自己写了），现在全程零校验 = 换机/刷机后静默失败或精度漂移。

### 3-7 控制层细节 `P2`
- 转向斜率限制 + 死区（`pursuit.py:190-191` 速度有斜坡、转向没有；ARC 入口一拍瞬时满舵 :147-149）→ 每拍 Δsteer 上限如 5°/拍。
- `fusion.py:80-88` 里程计步长加离群上限（`v_max·dt·k`），防 raw odom 跳变直接积入信念位置导致假越界 ESTOP。
- `fusion.py:131-135` IMU 首帧晚于 reset 的偏置：首帧到达时令 `yaw_anchor = th_reset - yaw_rel`，ACTIVE 前强制 `imu_seen` 门控（**注释称"等首帧后校正"但代码里没有校正逻辑**）。
- `executor_node.py:287` `_t` 开环累加 → `self.get_clock().now()`；`:203` `_cmd_load` 回调内阻塞文件 IO → 移出回调。
- `pursuit.py:180-188` 射击减速扫描遍历所有后续途经点，回折路线上远端停站会提前误减速 → 与 `:118-130`"只看紧邻点"的触发逻辑统一。
- `pursuit.py:254-255` "对齐弧"是 `arc_v=0.13` 前进满舵弧（R≈0.2-0.44m），**非原地转** → 图缘触发时可能直接越界闩锁 ESTOP，需加边界预判。
- `world_model.py:91-93` 蓝靶不去重（mines/passable/obstacles 都去重了）→ 同帧双检出使射击位组合数×2。
- `dzbrain_bridge.py:271-274,291-295` 桥洞/靶区变空时不发撤销报文 → 加显式 clear 语义。
- `integrate_node.py:109,528-531` align 爬行漂移 ~0.21m **大于半格 0.175m**，漂移后 `reset:N11` 仍断言车在原格心 → align 后用车位姿帧核实仍在起点格，否则告警。
- `integrate_node.py` **无任何 e-stop/中止路径**：路线一次性发出后节点无取消手段（replan 默认关、不订阅急停话题），dry_run 是启动期开关赛中不可切换 → 加 estop 话题 → 发布空路线/零速的最低限度中止链。
- `mission_router.py:515,602,944` 用 `warnings.warn` 报运行期风险 → **不进 ROS 日志且默认同位置只出现一次** → 改 logger。

### 3-8 建立最低限度的回归防线
- 交接包已有 15 个测试文件 / 3168 行（含 `test_router_fuzz.py`、`test_zoned_planning.py`、`test_planning_hardening.py`）——**它们写得不错，问题只是从来没随包、也没在 CI 里跑**。
- 加一个 `run_checks.sh`：`pytest tests/` + `tools/*.py` import 冒烟 + `grep` 密钥扫描 + `md5sum` 与 FILE_MANIFEST 比对。**每次打包前跑一遍**。
- 把进度纪要里的待办转成代码里的 TODO 或 issue——**现在全库 0 个 TODO/FIXME/HACK 标记，而实际待办一大堆**，说明待办只活在会话纪要里，代码本身不携带自己的债务信息。

---

## 附：如果只能做 5 件事

1. **0-1 把权威源定下来重打包**（否则你飞的不是你测的）
2. **1-1 禁火 TTL 改本地单调时钟**（唯一"失效方向是打友方"的问题，且触发条件已实测到过）
3. **0-4 + 0-5 NaN 双检 + SIGTERM 归零**（车带满舵前冲 / 停止时不归零）
4. **1-3 断链看门狗 + `/estop`**（现在链路断了车能自主跑 3 分钟）
5. **1-9 手动对时 + 钟差入黑匣子**（5 分钟，同时救禁火 TTL、复盘、延迟读数三件事）

---

*生成于 2026-10-17，配套 `architecture-review.md` / `risk-map.dot`。*
