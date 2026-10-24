---
来源: 交接_20261024 §2/§3；CAR-PREFLIGHT §9.2；用户 2026-10-24 接手指令
取证: 档（run2-mission.log、云台日志、上午旧栈样本）+ 只读源码 + 证据包
更新: 2026-10-24（17:3x 追加 §4.8 P1B）
status: in_progress
ns: agc-1024
agent: p-aim
树节点: AGC-2
写域: ~~_tmp_session/agc-1024/p1/~~；本卡；只读 aim.py / 云台 cpp+header / 检测源码
摘要: 解释第二趟 F9 搜索 6.10s target=none 的唯一可验证原因（未入画/漏检/陈旧/消费拒绝），只读起步，不调 PID、不换模型。
---

# AGC-P1-TARGET-20261024 · F9 为何 target=none

owner: p-aim；reviewer: lead。

## 1 依据与复用（五必填）

- 读过的台账：D-001（不整包替换自瞄）；现行推进计划 §4 P1；10-24 交接 §2（上午 74.91s 连续目标样本、本趟 target=none）。
- 复用检索：`mvrec.py`、`fire_rec`、既有检测诊断与 JSONL 采样、`docs/20_资料索引库/视觉与云台/01_云台瞄准与开火链路.md`；**不新建感知管线**。
- 新增轮子：预计无；若需一次性解析脚本，仅进过程目录。
- 验收命令：见 §3。
- 回滚点：本轮零生产写入即回滚点。

## 2 当前问题

F9 停稳 0.40s 后进入搜索，6.10s 未锁定目标，按原策略放弃并离站（已证）。缺同期原图/检测/消费/反馈四联证据，**不能先怪 PID 或提前量**。

## 3 唯一假设与验收

四个互斥候选，只允许锁定一个并给证据：
1. **未入画**：云台扫掠位姿/角度与目标方位不符（用命令 pitch/yaw 与靶板几何反算）。
2. **漏检**：图像里有靶但检测无输出（需同期图像；无图像则本项标“证据不足”，不许猜）。
3. **陈旧**：检测有输出但时间戳/年龄超期被消费端拒（offset_center 8 字段 + 内容更新时间/接收年龄）。
4. **消费拒绝**：Aim 收到但因类别未确认/门禁/状态机拒绝（Aim 原因日志 + 状态）。

- 验收：给出四项的判定表（成立/否定/证据不足），并明确“下一项唯一要补的观测”。只有一项成立才写结论。
- **禁止**：调 PID 增益、改补偿参数、换模型、改 target_confirm_frames 凑结果（先用证据定位）。

## 4 执行记录（p-aim · 2026-10-24 17:0x，只读，未改任何生产文件）

**一句话结论**：F9 的 `target=none` 不是"没看见"，而是**看得见的帧进不了消费端**——15:43 那趟在跑的
det_node（`/home/duzhong/dzrobot2/install/det_node/...`，pid 602768，13:00:37 起）发出的
`/offset_center` **只有 3 个 int32 字段**，而两个消费端都硬性要求 ≥5 字段、不足即整帧丢弃
（C++ `cpp:30`、Python `contract.py:246`）。于是 AimTask 永远 `has_target=False`（"无目标观测"）、
C++ 永远进不了 TRACK（`/aim/status` 停在 SCAN），Mission 的 6.00 s ACQUIRE 自然超时。
**候选 ④ 成立，③ 否定，① 否定（非本趟原因），② 证据不足（且被 ④ 完全遮蔽）。**

### 4.1 四项判定表

| # | 候选 | 判定 | 依据（文件:行 / 日志行 / 时间戳） |
|---|---|---|---|
| ① | 未入画（云台位姿与靶板几何不符） | **否定**（不是本趟 `target=none` 的原因） | `target=none` 在**解码层**就已产生（见 ④），与画幅无关；同期云台状态是 `瞄SCAN` 且扫描确在工作（`static-scan-home.jsonl`：yaw 位置 2047→340 = 0→−150°，97.5°/s，1.6 s 到位）。**残留**：本趟无同窗云台位姿/原图，若靶在 ±150° 扇区外则 ① 可独立成立，但会被 ④ 完全遮蔽，本趟无法也无需区分 |
| ② | 漏检（图像有靶、检测无输出） | **证据不足**（按卡面不猜） | 无同期原图。旁证：14:57:11 那一帧 `[9,-199,1]` 证明模型**有**输出 cls=1 的能力（1/679=0.15%）；但该帧同样只有 3 字段、同样被丢 ⇒ 即使漏检成立也不能解释 `target=none` |
| ③ | 陈旧（时间戳/年龄超期） | **否定** | 帧以 **30.4 Hz** 连续到达（439 帧/14.4 s）；3 字段 payload **没有任何内容时间戳**，消费端是"到帧即解码"，丢帧发生在任何年龄判据**之前**；遥测是"靶none"（连观测对象都没有），不是"观测过期" |
| ④ | 消费拒绝 | **成立**（机制修订为：**长度守卫整帧丢弃**，不是类别未确认/门禁/状态机） | 3 < 5 ⇒ C++ 与 Python **双侧丢弃 100% 帧** ⇒ AimTask `has_target=False` → `REASON_NO_TARGET`；C++ 进不了 TRACK → `/aim/status` 停在 SCAN → `reverse-fix/run2-mission.log:395` "锁定超时 6.10s ≥ 6.00s（aim=SCAN 目标=False）" |

### 4.2 判定 ④ 的证据链（七环，全部本地已有）

1. **消费端（C++）**：`_tmp_session/carloop-1024/prep-remote/src/dzrknn_dataprocess/src/dzrknn_dataprocess.cpp:30`
   `if (temp_msg.data.size() < 5)` → `:33` 打印 `offset_center 字段不足(%zu < 5)，已丢弃该帧`；
   **车端原件同**（`prep-base/.../dzrknn_dataprocess.cpp:30`）⇒ 运行的是哪版二进制都丢。
2. **消费端（Python 真源）**：`dzrobot_v2/src/dzbrain/dzbrain/io/contract.py:246` `if len(vals) < 5: return None`；
   调用点 `io/ros_io.py:249-255` —— obs 为 None 时**直接 return，不写 `self._vision`**；
   `tasks/aim.py:728` `has_target = obs is not None and cls in (blue,red)`；`aim.py:132` 把 `no_target` 显示为"无目标观测"。
3. **发送端实际格式（决定性）**：`_tmp_session/carloop-1024/static-scan-home.jsonl`（14:57:03 起 14.4 s）
   **439 帧，字段数全部 = 3**；`static-stop0.jsonl`（14:55:06 起）**240 帧，全部 = 3**。
   记录者 `_tmp_session/VISION-NPU3CORE-20261023/t15_fire/fire_rec.py:13-17` 把 `msg.data` **原样** list 写出，
   不存在截断。⇒ 下午 `/offset_center` 就是 3 字段。
4. **发布者是同一个进程（跨 14:57 与 15:43）**：`prep-node-ownership.log:21` →
   `602768 det_node ... Sat Oct 24 13:00:37 2026`；`prep-processes.log:7` → exe =
   `/home/duzhong/dzrobot2/install/det_node/lib/det_node/det_node`，cwd=`/home/duzhong/dzrobot`，
   `--params-file config/robot.yaml`；交接 §4 记录"本轮未重启"。⇒ 14:55 静态取证与 15:43 跑车是**同一发布者**。
5. **格式分界点**：上午 `artifacts/aim-mv-20261024/mvlog.csv`（10:39:04 起 74.91 s）
   **1897 帧全部 8 字段**、w p50=133 / conf p50=99 / cls 1:1892 + 2:5，且当轮两发 1:1 记账。
   ⇒ 8→3 的分界只能是 13:00:37 那次 det_node 换栈（当天唯一一次更换）。
6. **状态侧反证**：run2 全程 `瞄TRACK = 0`（126 条 2 Hz 遥测：SCAN 9 / IDLE 117；
   `run2-mission.log:367-394` F9 期间 9 条全 `瞄SCAN 靶none`，`:395` 直接超时）；
   第一趟 205 条遥测同样 `瞄TRACK = 0`。⇒ C++ 从未拿到过一帧"有效目标"（否则会进 TRACK 并打类别锁定日志）。
7. **时间线（epoch→CST 已换算）**：10:39:04 mvrec 8 字段 → 13:00:37 det_node 换成 dzrobot2 二进制 →
   14:55:06 / 14:57:03 fire_rec 静态取证 3 字段（其中 14:57:11 唯一一帧 cls=1）→ 15:43 run2 F9 6.10 s target=none。

> 复跑：`python -B -X utf8 _tmp_session/agc-1024/p1/verify_offset_fields.py`
> （输出 `_tmp_session/agc-1024/p1/p1-summary.json`：三份样本的字段数直方图 / 速率 / 类别直方图 / 双侧守卫行号 / 输入 sha256 前 12 位）

### 4.3 未证与边界（不要当结论用）

- **车端安装的 `contract.py` 版本不在证据包内**：本地真源有 `len<5` 守卫；若车端是更早的无守卫版本，
  结果是 `IndexError` 抛异常——**同样拿不到观测**（结论不变），但"静默丢帧 vs 抛异常"需 E1 补车端哈希/日志确认。
- **3 字段语义**按 `[dx, dy, cls]` 推断（唯一非零帧 `[9,-199,1]` 的 1 落在 cls 位），
  未读 dzrobot2 源码；字段序不影响本条结论（无论顺序，长度 3 < 5 必被丢）。
- **8→3 的分界点**由"当天唯一一次 det_node 重启 + 上下午各一份原始逐帧采样"推断，不是逐秒抓包。

### 4.4 同窗必须采集的最小字段集（复用 fire_rec 即可，不新建管线）

1. `/offset_center` **原始 data 长度 + 全部字段**（fire_rec 已原样记录，够用）；
2. `/aim/status`；3. `/camemo_feedback`（云台 yaw/pitch 反馈）；4. `/nav/scan_request`；
5. `/aim/fire_gate` `/aim/fire_ready` `/aim/lead_x`；6. 每秒 1 帧 `/image_raw`（只需能看清靶是否在画幅内）；
7. AimTask 侧一行 decode 日志（长度/字段/cls/是否 None）。
缺 1、3、6 任一项，①/② 永远判不了；本次 1 的"长度"一项就足以定案。

### 4.5 下一项唯一动作（车离线，回线后一条只读命令）

```bash
# 都不重启、不发车、不出光
ros2 topic echo /offset_center --once            # 看 data 里到底几个数
grep -c 'offset_center 字段不足' ~/robot_logs/5_gimbal.log   # C++ fd1 就是这份（prep-processes.log:5） 〔未入库：车端〕
```
两行任一命中即闭环证据链（字段数=3 / 丢弃计数随时间线性增长）。**在拿到这两行之前，不要调 PID、不要换模型、不要改 target_confirm_frames。**

### 4.6 修复建议（本卡不落盘，交 e-integ 合流评审）

- **首选**：把 det_node 统一回**"8 字段 + 与消费端同源"**，而不是给消费端加兼容。
  理由：3 字段连 `box_w/box_h/置信度` 都没有 ⇒ 动态开火框（`fireDeadzonePx(box_w)`）、长宽比过滤
  （`aspectRatioOk` 用 `data[3]/[4]`，cpp:57）、距离代理全部无从计算，加兼容只会造出"能跟踪、打不准"的假通过。
- 若必须消费端兼容：改动点 = `cpp:24-38`（补字段缺省策略）+ `contract.py:233-249`（同步守卫），
  **按"车端基线"补丁片段交 e-integ**，附行号与函数名；**禁止整包推车**（会回退 lead/EKF/平滑跟踪/停止锁存）。
- 本轮**未改** `src/**`、`config/**`、`tools/deploy_to_car.py`；未连车、未部署、未重启、未出光。

### 4.7 回滚点

零生产写入：仅新增 `_tmp_session/agc-1024/p1/verify_offset_fields.py` + `p1-summary.json`，并追加本卡。

### 4.8 P1B 执行记录（det_node 3→8 字段恢复 · t2 · 2026-10-24 17:3x）

**结论：不用改代码、不用重编译。8 字段成品就在同一台车的 dzrobot 栈里，错的是进程选择（`~/.bashrc` 把 `dzrobot2/install` 加进了环境）。**
本轮车端全部只读（`ls/stat/grep/sed/cat/strings/ldd/sha256sum/ps`），未启动进程、未重启、未部署、未发车、未出光。

**① 3 字段产生点（源级定案）**：`/home/duzhong/dzrobot2/src/rknn_pt/src/det_node.cc`（146 行，mtime 10-20 21:51）
`:28` 建 publisher `offset_center`；`:73-75` `dim[0].size=2/stride=2`（与实际元素数不一致）；`:76` `msg.data = {0,0,0}`；
`:104-106` 只写 `data[0]=dx, data[1]=dy, data[2]=1`。
⇒ 字段语义是 **`[dx, dy, 常数1]`**，**不是 `[dx,dy,cls]`**：没有 cls 映射（一切皆"蓝"）、没有 box_w/box_h、没有关键点/置信度。
（本条修正 §4.3 的"按 `[dx,dy,cls]` 推断"。）

**② 8 字段成品（不需要动）**：`/home/duzhong/dzrobot/src/rknn_pt/src/det_node.cc`（334 行，md5 `b6e263b0…`
= 本地 `dzrobot_v2/src/rknn_pt/src/det_node.cc`）`:240-242` `dim=8`、`:245` 初值 8 个 0、`:273-281`
`[0]dx [1]dy [2]cls+1 [3]box.w [4]box.h [5]kx [6]ky [7]kpt_conf*100`；二进制 `/home/duzhong/dzrobot/install/det_node/lib/det_node/det_node`
（776 KB，**10-24 09:06 构建**，`strings` 含 `hot_reload/max_fps/thread_num/model_path` ⇒ 参数化 8 字段版）。
dzrobot2 的同名二进制（3.86 MB，10-20 21:51）四个参数名全无 ⇒ 3 字段版。第三份 `merged_workspace`（10-18）也是 8 字段，**勿误选**。

**③ 选择机制**：`~/.bashrc:124` 与 `:129` 都 `source /home/duzhong/dzrobot2/install/setup.bash`（:129 的注释写"自动加载dzrobot工作空间"，
内容却是 dzrobot2）。实测：dzrobot2+dzrobot 混合 source 后 `ros2 pkg prefix det_node` **仍返回 dzrobot2**；只 source dzrobot 才返回 dzrobot。
⇒ **不能靠 source 顺序**，必须绝对路径启动 + 显式 `model_path`。

**④ 修复候选 A（推荐，零代码）**：
```bash
source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=118; cd /home/duzhong/dzrobot
nohup /home/duzhong/dzrobot/install/det_node/lib/det_node/det_node --ros-args \
  --params-file /home/duzhong/dzrobot/config/robot.yaml \
  -p model_path:=/home/duzhong/dzrobot/install/det_node/share/det_node/model/light_det.rknn \
  -p draw:=false >/tmp/det_8field.log 2>&1 &
```
- 显式钉模型的原因：robot.yaml `:310` `model_path:""` 会走 ament 解析 → 当前环境落到 **dzrobot2 的 9.9 MB 模型**（sha `59aaad60…`）；
  要的是 dzrobot 的 8.1 MB（sha `8b17ec17…`，10-23 19:12）——历史 `robot_logs/4_det.log:4` 证明 8 字段节点一直配这个模型（线程2/窗口关/热加载开）。 〔未入库：车端〕
- 只 source ROS 基础环境可行：`ldd` 在该环境下 `not found` = 0，`librknnrt.so` 由 RUNPATH 解析到 dzrobot 自己的库。
- **禁止** `ros2 run/launch det_node`（走 ament 解析，必落 dzrobot2）；若用必须先证 `ros2 pkg prefix det_node` = dzrobot。

**⑤ 离线已验（exit 0）**：`python -B -X utf8 _tmp_session/agc-1024/p1b/verify_p1b_8fields.py` → `P1B VERIFY PASSED`（20 项断言）：
两份源逐字段布局；`contract.decode_offset_center` 8 字段→`OffsetObservation(box_w=135,box_h=140)`、3 字段→`None`（`contract.py:246`）；
C++ 守卫镜像对真实样本 —— 上午 `mvlog.csv` **1895/1897 通过(99.9%)**、开火死区算得 18..26 px；下午 3 字段 **0/439** 与 **0/240**（全死 `size<5`）。

**⑥ 必须上车验（待授权，只动 det_node 一个进程）**：V1 `topic echo /offset_center --once` 见 **8 个数**；
V2 `topic hz` ≈25 Hz；V3 启动日志模型路径 = dzrobot 8.1 MB/线程2/窗口关；V4 蓝靶入画 `/aim/status` 1→**2 TRACK**（F9 能否打下的判据）；
V5 蓝=1/红=2 类别语义；V6 `字段不足` 计数不再增长（**C++ 日志走 stderr，要 `2>` 落盘**，本轮 `5_gimbal.log` 只接 fd1 故历史计数为 0，不能当反证）。 〔未入库：车端〕

**⑦ 风险与回滚**：模型(8.1 vs 9.9 MB)、cls 语义(恒1 vs 真类别)、帧率/CPU(去重+限帧 vs 裸 33 ms)、
`fire_dz_far_px` 漂移(10:39 记录 25 → 现值 18.0，与本修复无关，需单列 A/B) 四项见 §2 表；
回滚 = 杀掉该 pid 按原命令重起（车上**零文件改动**，不动 lead/EKF/云台/停止锁存/任何 config）。

**⑧ 候选 B（备选，需 Lead 批）**：把 dzrobot2 源补成 8 字段 = `:73-75` 改 8/8、`:76` 初值 8 个 0、`:104-106` 改 `cls+1` 并补 `box.w/box.h`、 〔非路径：字段名〕
再 `colcon build --packages-select det_node`（= 二进制替换 = 部署动作）。只补到 5 字段属"能跟踪"，补 `[3]/[4]` 才"能打准"。

**过程证据**：`_tmp_session/agc-1024/p1b/{P1B_8字段恢复方案.md, car-probe-evidence.md, verify_p1b_8fields.py, p1b-summary.json, dzrobot2_det_node.cc.txt, probe1..11.sh}`。

### 4.9 V6 日志级基线更正（p-aim · 2026-10-24 18:0x，append-only，不改既有结论）

§4.8 与 P1B 方案 §2(c) 里「C++ 丢帧只有源码级+症状级证据、`5_gimbal.log` grep=0」这条注记**需更正**（结论不变，证据更强）：

- 顶层 `/home/duzhong/robot_logs/5_gimbal.log` 确实是 **0 条**（该进程只接了 fd1）；
- 但 **per-run 复制件** `carloop_20261024_static/5_gimbal.log` 有 **1425 条** `[WARN] [dzrknn_dataprocess]: offset_center 字段不足(3 < 5)，已丢弃该帧`，覆盖 **14:52:06 → 16:51:10**（含 15:43 那趟 run2 窗口）；同进程 launch 捕获 `.../ros/dzactuator_node_620811_*.log` 另计 **1398 条**；
- 该 WARN 为 `WARN_THROTTLE 5s`：1425 × 5 s ≈ 1.98 h ≈ 文件跨度 ⇒ 整段**连续丢弃**（条数不能换算帧数）。

⇒ 字段丢弃已有**日志级**铁证；AGC-19 的 V6 判据据此收紧为「新日志 0 条新增」。详见 `AGC-19.md §4.4`。

## 5 交付

1. 候选判定表 + 依据（文件:行、日志行号、时间戳）。
2. 若证据不足：列出同窗必须采集的最小字段集（不开发新框架，复用 fire_rec 临时适配即可）。
3. 下一动作建议（含是否需要检出新源确认：等 E1 的 det_node 来源结论）。

## 10 文档变更

新增本卡；修改本 ns 任务树节点状态。归档：无。

2026-10-24 17:0x（p-aim）：**本卡内新增 §4 执行记录**（判定表 + 七环证据链 + 未证边界 + 最小字段集 + 下一动作 + 修复建议）；
新增过程证据 `_tmp_session/agc-1024/p1/{verify_offset_fields.py,p1-summary.json}`（过程稿，不入库）。
**拟落点（归档时由 Lead 落）**：`docs/20_资料索引库/视觉与云台/02_视觉模型与检测.md` 增一行症状"检测有 30 Hz 心跳但瞄准永远 target=none → 先数 /offset_center 字段数"；
`01_云台瞄准与开火链路.md` §2 补一句"field 数版本差异（3 字段 dzrobot2 det_node 与 8 字段旧栈并存，消费端 ≥5 守卫整帧丢弃）"。
**未新增/未修改任何库内文档**（本轮只读分析）。

2026-10-24 17:3x（p-aim · P1B / t2）：本卡新增 **§4.8 P1B 执行记录**（3 字段源级定案 + 8 字段成品位置 + 进程选择机制 +
候选 A 精确启动方式 + 离线/上车验收 + 风险回滚 + 候选 B）；新增过程证据
`_tmp_session/agc-1024/p1b/{P1B_8字段恢复方案.md, car-probe-evidence.md, verify_p1b_8fields.py, p1b-summary.json, dzrobot2_det_node.cc.txt, probe1..11.sh}`（过程稿，不入库）。
**修正 §4.3 的一条推断**：3 字段语义是 `[dx,dy,常数1]`（`:106` 写死 1），不是 `[dx,dy,cls]`。
**拟落点（归档时由 Lead 落）**：`docs/20_资料索引库/视觉与云台/02_视觉模型与检测.md` 增症状
"F9 target=none → 先 `ros2 pkg prefix det_node`：`~/.bashrc:129` 会把 `ros2 run det_node` 解析到 dzrobot2 的 3 字段栈"；
`01_云台瞄准与开火链路.md` §2 的"field 数版本差异"补一句"两套栈二进制与模型都不同（776 KB/8.1 MB vs 3.86 MB/9.9 MB），恢复时须显式钉 model_path"。
**未新增/未修改任何库内文档**（本轮 P1B 同样只读分析，仅写 `_tmp_session` 过程稿）。
