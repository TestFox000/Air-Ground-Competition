---
来源: AGC-19（树节点）；AGC-P1-TARGET-20261024 §4.8（P1B 源级定案）；Lead 2026-10-24 裁决四条
取证: 实（车端只读 probe12/13 + 本地 dry-run/preflight 真跑）；档（P1B 方案文档 §2）
更新: 2026-10-24（18:0x 阶段一：脚本 + 日志分流 + dry-run + 只读预检）
status: in_progress
ns: agc-1024
agent: p-aim
树节点: AGC-19
写域: _tmp_session/agc-1024/p1b/**；本卡。车端只读（只写 /tmp）；不动 config/lead/EKF/云台/停止锁存
摘要: 把车上 det_node 从 3 字段换回 8 字段做上车验证（V1-V3 本阶段；V4 云台待 Lead 二次确认）。本轮交付 run 脚本 + 日志分流 + dry-run + 只读预检，执行等用户现场授权 GO。
---

# AGC-19 · P1B 8 字段 det_node 上车验证（阶段一：只起 camera + det_node）

owner: p-aim；reviewer: lead。前置：AGC-2 ✔（F9 target=none 根因=3 字段 vs 消费端 ≥5 守卫）。

## 0 一句话（Lead 要的「改什么 / 回滚是什么」）

> **将要改什么**：只把车上**一个进程**从 `dzrobot2` 的 3 字段 det_node 换成 `dzrobot` 的 8 字段 det_node
> （绝对路径 + 显式 8.1 MB 模型 + `draw:=false`），外加临时起一个 camera；
> **不改车上任何文件、不重编译、不部署、不启 mission/云台/发射**。
> **回滚是什么**：`bash p1b_car.sh rollback` 一条命令杀掉这两个进程组并校验「零残留 + /offset_center 无发布者」；
> 要复原原来那个 3 字段进程，加 `--legacy` 用 `.bashrc` 交互环境的原命令起回。

## 1 依据与复用（五必填）

- 读过的台账/卡：AGC-2（P1B 源级定案：3 字段=`dzrobot2/src/rknn_pt/src/det_node.cc:73-75,76,104-106`；8 字段=`dzrobot/src/.../det_node.cc:240-242,245,273-281`）；Lead 四条裁决（换进程方向批准、先交 dry-run、候选 B 不排期、fire_dz_far_px 另立 AGC-18）。
- 复用检索：`tools/healthcheck.sh`（有 usb_cam/det_node 检查，但**不判字段数**，不复用其启动逻辑）；`dzrobot_v2/src/dzrknn_dataprocess`（消费端守卫，只读参照，本卡不改）；`config/robot.yaml` `/det_node` 段（参数单一真源，只读）。**不新建感知管线**。
- 新增轮子及理由：`_tmp_session/agc-1024/p1b/p1b_det8.sh`（车端执行脚本：预检/启动/采样/判定/回滚一条脚本）+ `p1b_car.sh`（本地 ssh 送脚本的薄入口）。理由：现网无「只起 camera+det_node 且按字段数判定」的工具——healthcheck 只查进程存在、不查报文形状；两文件都在过程目录，不进 `docs/`、不登记复用清单（性质=一次性上车验证脚本）。
- 验收命令：见 §3（dry-run / preflight / all 各有退出码）。
- 回滚点：车端**零文件改动**（只写 `/tmp`），回滚=杀进程；脚本 `rollback` 自带零残留校验。

## 2 目标与硬约束

**目标**：在**不启 mission、不发车、不出光**的前提下，证明换进程后 `/offset_center` 是 8 字段、≈25 Hz、模型=dzrobot 8.1 MB。

**硬约束（脚本内置断言，违反即 abort）**

| # | 约束 | 脚本实现 |
|---|---|---|
| 1 | 只起 camera + det_node | 启动后断言 `pgrep dzbrain/path_executor/dzrknn_dataprocess/dzjudgment` 为空 |
| 2 | 绝不 source dzrobot2 | 只 `source /opt/ros/humble/setup.bash` + `source dzrobot/install/setup.bash`；det_node 走**绝对路径**；预检断言 `ros2 pkg prefix det_node` 指向 dzrobot |
| 3 | 模型变量钉死 | 显式 `-p model_path:=<dzrobot 8.1MB>`，预检校验 sha256=`8b17ec17…` |
| 4 | 不双发布 | 起进程前断言 `/offset_center` 无发布者、无残留进程；起后断言发布者=1 |
| 5 | 不改车上文件 | 只写 `/tmp`；`/tmp` 外零写入（预检无写动作） |
| 6 | 授权闸 | `start`/`all` 没有 `--go` 直接拒绝（exit 3）；dry-run/preflight/sample/judge 不受限 |

## 3 交付物（本轮全部已落盘）

| # | 交付 | 路径 |
|---|---|---|
| ① | 一条 run 脚本（启动/采样/判定/一键回滚） | `_tmp_session/agc-1024/p1b/p1b_det8.sh`（543 行） + 本地入口 `p1b_car.sh` |
| ② | 日志分流 | det_node：`/tmp/det_8field.out`（fd1）/ `/tmp/det_8field.err`（fd2）；camera：`/tmp/p1b_camera.out/.err`；采样：`/tmp/p1b_sample.json`、`/tmp/p1b_echo_once.txt`、`/tmp/p1b_hz.txt`；判定：`/tmp/p1b_verdict.txt` |
| ③ | dry-run 输出（只打印不执行） | `_tmp_session/agc-1024/p1b/dryrun-FINAL.txt`（54 行） |
| ④ | 只读预检实测 | `_tmp_session/agc-1024/p1b/preflight-20261024_1802.txt`（13 PASS / 0 FAIL） |
| ⑤ | 车端只读探针 | `probe12.sh`、`probe13.sh`（本目录） |

**运行方式**

    bash _tmp_session/agc-1024/p1b/p1b_car.sh dry-run     # 只打印（本轮已跑）
    bash _tmp_session/agc-1024/p1b/p1b_car.sh preflight   # 只读预检（本轮已跑，13/13 PASS）
    bash _tmp_session/agc-1024/p1b/p1b_car.sh all 12 --go # 【等 GO】起 camera+det_node→采样12s→判定
    bash _tmp_session/agc-1024/p1b/p1b_car.sh rollback    # 收工：杀进程 + 零残留校验

## 4 执行记录（阶段一 · 2026-10-24 17:4x–18:0x，车上零进程启动）

### 4.1 dry-run 关键命令（原文，来自 dryrun-FINAL.txt）

    # P2 camera
    setsid nohup bash -c 'source /opt/ros/humble/setup.bash; source /home/duzhong/dzrobot/install/setup.bash; export ROS_DOMAIN_ID=118; export RCUTILS_LOGGING_BUFFERED_STREAM=0; ros2 launch dzactuator camera.launch.py' > /tmp/p1b_camera.out 2> /tmp/p1b_camera.err < /dev/null & echo $! > /tmp/p1b_cam.pid

    # P3 det_node（8 字段，绝对路径 + 显式模型）
    setsid nohup /home/duzhong/dzrobot/install/det_node/lib/det_node/det_node --ros-args --params-file /home/duzhong/dzrobot/config/robot.yaml -p model_path:=/home/duzhong/dzrobot/install/det_node/share/det_node/model/light_det.rknn -p draw:=false > /tmp/det_8field.out 2> /tmp/det_8field.err < /dev/null & echo $! > /tmp/p1b_det.pid

    # P4 采样（只读订阅，不发布）
    timeout 42 python3 /tmp/p1b_sampler.py --seconds 12 --json /tmp/p1b_sample.json

    # P7 回滚
    kill -TERM -<DET_PGID>; kill -TERM -<CAM_PGID>
    pkill -f '^/home/duzhong/dzrobot/install/det_node/lib/det_node/det_node'; pkill -f 'ros2 launch dzactuator camera.launch.py'; pkill -f 'usb_cam_node_exe'

**设计要点（不这么写就会踩坑）**

| 决定 | 理由（车端证据） |
|---|---|
| `setsid` + 独立进程组 | camera 由 `ros2 launch` 起，父进程是 launch；按组 TERM 才能一次收干净 |
| 绝对路径、不用 `ros2 run/launch` 起 det_node | `bash -ic` 环境下 `ros2 pkg prefix det_node` = **dzrobot2**（probe13-A 实测）；只有 clean env 才是 dzrobot |
| 显式 `-p model_path:=` | robot.yaml `model_path:""` 会经 ament 解析到 **dzrobot2 的 9.9 MB 模型** |
| `RCUTILS_LOGGING_BUFFERED_STREAM=0` | stderr 重定向到文件默认块缓冲，V3/V6 的 grep 会「看不到行」 |
| `.out`/`.err` 分流 | V6 必须看 stderr；V3 启动行在 fd1（ROS_INFO）——两条流必须分开留证 |

### 4.2 只读预检实测（preflight-20261024_1802.txt，13 PASS / 0 FAIL）

| 检查 | 结果 |
|---|---|
| 二进制 size/mtime/md5 | 776 112 B / 2026-10-24 09:06:26 / `a343a1f0…` ✔ |
| 二进制定性（`strings` 含 `hot_reload`+`max_fps` 整行） | ✔ 8 字段参数化版 |
| 模型 size/sha256 | 8 125 813 B / `8b17ec17…` ✔ |
| `robot.yaml` /det_node | `model_path:""`(:310) `draw:false`(:313) `thread_num:2`(:316) `max_fps:30.0`(:317) `hot_reload:true`(:319) |
| 残留进程 | 无 det_node/usb_cam/云台/dzbrain（与 Lead 17:0x 现场一致） |
| `/dev/video0` | 可读、未被占用（duzhong 属 `video` 组） |
| 包解析（clean env） | det_node → dzrobot ✔；dzactuator → dzrobot ✔；usb_cam → `/opt/ros/humble` |
| `/offset_center` | `Unknown topic`（无发布者）⇒ 起后必须=1 |
| `/tmp` | 可写 |

**过程中修掉的两个脚本级坑（真缺陷，不是环境问题）**

1. `strings | grep -qx` 在 `set -o pipefail` 下**假 FAIL**：`grep -q` 命中即退出 → `strings` 吃 SIGPIPE(141) → pipeline 非零。已改为先落 `/tmp/p1b_strings.txt` 再 `grep -qx`（无管道）；修复前的失败输出留证 `preflight-BEFOREFIX_1757_pipefail-bug.txt`（13 项里 1 项 FAIL，其余全绿）。
2. `c=$(grep -c ... || echo 0)` 在无匹配时输出**两行 0**（grep 打印 0 且退出 1，`|| echo 0` 又补一个）。已改为先取值、空则用 bash 参数默认值补 0。

### 4.3 本轮新增的车端事实（probe12/13，只读）

| 事实 | 证据 | 对本案的意义 |
|---|---|---|
| `ROS_DOMAIN_ID=118` **只**在 `~/.bashrc:126` | `bash -lc` 下连 `ros2` 都找不到；`bash -ic` 下 `[118]` | 脚本必须自己 `export ROS_DOMAIN_ID=118`，不能指望环境 |
| `bash -ic` → `ros2 pkg prefix det_node` = **dzrobot2** | probe13-A | 复现 bug 的确切入口；也是 `rollback --legacy` 的复原方式 |
| `usb_cam` 来自 `/opt/ros/humble`（apt）；`dzactuator` 两套工作空间都有 | probe12-E/F/G | 只 source dzrobot 就能起 camera，且解析确定 |
| 无 systemd 单元（`systemctl` 无 dz/ros 服务） | probe12-L | 没有看门狗会自动重拉 3 字段 det_node ⇒ 无双发布隐患 |
| 裸 `python3` 无 `rclpy`；source ROS 后可用（Py 3.10） | probe12-M/H | 采样器必须在 ROS env 里跑（脚本已这么做） |
| `jq/timeout/setsid/nohup` 均在 | probe12-M | 判定与进程组管理可用 |

### 4.4 **V6 基线拿到日志级铁证（更正前轮一处注记）**

- `/home/duzhong/robot_logs/carloop_20261024_static/5_gimbal.log`：**1425 条**
  `[WARN] [dzrknn_dataprocess]: offset_center 字段不足(3 < 5)，已丢弃该帧`，
  时间跨度 `1792824726.85`→`1792831870.05` = **14:52:06 → 16:51:10**（含 15:43 那趟 run2 窗口）；
  同一进程的 launch 捕获 `carloop_20261024_static/ros/dzactuator_node_620811_*.log` 另计 **1398 条**。
- 该 WARN 是 `WARN_THROTTLE 5s`：1425 × 5 s = 7125 s ≈ 1.98 h，与文件跨度 1.98 h 吻合 ⇒ **整段连续丢弃**
  （条数不能换算帧数，只能证明「持续丢弃」）。
- **更正**：AGC-P1 §4.8 与 P1B 方案 §2(c) 写的「`5_gimbal.log` 只接 fd1，`grep -c` = 0，日志级未捕获」
  只对**顶层** `/home/duzhong/robot_logs/5_gimbal.log`（=0）成立；**per-run 复制件里有 stderr**，
  字段丢弃早就有日志级证据。V6 判据据此收紧为「新日志 0 条新增」。

### 4.5 V1-V6 判定口径（脚本已实现；阶段一能判哪些）

| # | 判据（脚本内） | 证据载体 | 阶段一可判？ |
|---|---|---|---|
| V1 | `data_lens` 含 `"8"` 且 `layout.size=8` | `/tmp/p1b_sample.json` + `/tmp/p1b_echo_once.txt` | **可判** |
| V2 | `rate_hz` ∈ [18,32]（期望≈25）+ `ros2 topic hz` 第二路 | 同上 + `/tmp/p1b_hz.txt` | **可判** |
| V3 | fd1 启动行逐字命中：模型=dzrobot 的 8.1 MB `light_det.rknn` + `线程=2` + `窗口=关` + `热加载=开`，并有 `已关闭识别画面窗口` | `/tmp/det_8field.out` | **可判** |
| V5 | `cls` 直方图：1=蓝 2=红 0=无目标 | `/tmp/p1b_sample.json` | **仅当靶入画**；否则 INCONCLUSIVE（不许猜） |
| V6 | 生产者侧 `len_lt5 == 0`；消费端新增 `字段不足` **0 条** | sampler / `/tmp/det_8field.err` | 生产者侧可判；**消费端需 V4 阶段**（本阶段不起 dzrknn_dataprocess） |
| V4 | 蓝靶入画 `/aim/status` 1→2 | `/aim/status` | **本阶段不判**（Lead 二次确认后另起整栈） |

### 4.6 离线可验 vs 必须上车

- **本轮已验完**：`bash -n`（脚本语法）、内嵌采样器 `py_compile`、dry-run 全文、preflight 13 项、车端只读探针 12/13。
- **必须上车（等 GO）**：V1/V2/V3；V5（靶入画，需人摆靶）；V6 消费端（需 V4 阶段 + 消费端 stderr 落盘）。
- **本卡不覆盖**：F9「停站→扫描→锁定→出光」整链实跑（属 M2，需独立出光授权）。

## 5 待 Lead / 用户决定

1. **GO 窗口**：用户现场授权后跑 `all 12 --go`（约 60 s：起进程 ~20 s + 采样 12 s + 判定 5 s + 收尾 3 s）。
2. **V4 二次确认**：V1-V3 绿后，是否另起整栈做 `/aim/status` 1→2。
3. **V5 要不要一次拿全**：采样那 12 s 里请人把**蓝靶**放进取景框，V5 可当场闭环；不摆则留到 V4。二者都不影响 V1/V2/V3。
4. **V6 消费端路由**：整栈启动脚本里 dzrknn_dataprocess 目前只接 fd1；V4 阶段需给它加 stderr 重定向到 `/tmp/gimbal_8field.err`（属跑车脚本，t-car/Lead 决定）。

## 6 风险与回滚

| 风险 | 处置 |
|---|---|
| 解析到 dzrobot2（3 字段） | 绝对路径 + 预检断言 pkg prefix + V3 模型路径反查，三重 |
| 模型被顺带换掉（9.9 MB） | 显式 `-p model_path` + sha256 校验 |
| 双发布 | 起前断言无发布者/无残留；起后断言=1；无 systemd 看门狗 |
| camera 抢 `/dev/video0` 失败（respawn 循环） | camera.launch.py 自带 respawn 2 s；失败时脚本打印 `/tmp/p1b_camera.err` 尾部 |
| 回滚不干净 | 按**进程组** TERM→KILL + 精确路径 pkill + pgrep/topic 双校验 |

## 7 结论（阶段一）

| 项 | 判定 | 证据 |
|---|---|---|
| run 脚本（启动/采样/判定/一键回滚） | 完成 | `p1b_det8.sh`（`bash -n` OK）、`p1b_car.sh` |
| 日志分流 .out/.err | 完成 | 脚本 P3 行；全部证据路径见 §3 |
| dry-run（只打印不执行） | 完成，**车上零动作** | `dryrun-FINAL.txt`（54 行，18:01:29 生成） |
| 只读预检 | 13/13 PASS | `preflight-20261024_1802.txt` |
| V1-V3 上车判定 | **未执行（等 GO）** | — |
| V4/V5/V6 消费端 | 未执行（V4 需二次确认） | — |

## 8 相关文件

- 脚本：`_tmp_session/agc-1024/p1b/p1b_det8.sh`、`p1b_car.sh`
- 证据：`dryrun-FINAL.txt`、`preflight-20261024_1802.txt`、`probe12.sh`、`probe13.sh`
- 前置：`_tmp_session/agc-1024/p1b/P1B_8字段恢复方案.md`、`car-probe-evidence.md`、`AGC-P1-TARGET-20261024.md §4.8`

## 10 文档变更

| 动作 | 文档 | 说明 |
|---|---|---|
| 新增 | `docs/30_任务卡库/在办/agc-1024/AGC-19.md` | 本卡（树节点 AGC-19 由此有实体卡） |
| 修改 | `docs/30_任务卡库/在办/agc-1024/AGC-P1-TARGET-20261024.md` | 追加 §4.9：V6 日志级基线更正（append-only，不改既有结论） |
| 修改 | `_tmp_session/agc-1024/p1b/P1B_8字段恢复方案.md` | 加一行 V6 更正指针（过程稿，不入库） |
