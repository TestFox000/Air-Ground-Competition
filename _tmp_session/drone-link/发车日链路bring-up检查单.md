# 发车日 · 无人机 ↔ 小车 链路 bring-up 检查单

> 产出：`drone-link`（并行线，不在小车关键路径）· 版本 2026-10-12 · 依据 t3/t4/t12/t16/t18/t19 的实测口径
> 用法：发车日按 §1 → §2 → §3 顺序走；出问题直接跳 §4（三步排查）。**每条命令都可复跑**。
> 关联文档：[t4 链路验证报告](../../dzsmartscout/tools/reports/t4-链路验证报告.md)、
> [t16 口径对齐](../../dzsmartscout/tools/reports/t16-口径对齐说明.md)、
> [t18 复核整改](../../dzsmartscout/tools/reports/t18-复核整改说明.md)、
> 协议手册 `dzrobot_v2/docs/空地指挥链路_协议与运行手册.md`（车侧视角）。

---

## 0. 本次复验快照（**无人机当前不在线**）

> 时间口径提醒：本节这些"实测"发生时，**机/本机时钟读数都是 2026-10-12 20:2x**，
> 而**真实时间是 2026-10-15**（已用两个独立 HTTP `Date` 头核对：`baidu.com` 与 `bing.com`
> 都返回 `Thu, 15 Oct 2026 12:31:44 GMT`）⇒ **存在 ~3 天的时钟偏差，发车日前必须对时**（见 §0.2）。

| 项 | 实测结果 |
|---|---|
| 无人机 `10.29.122.104` | ❌ **不可达**：ping 3/3 丢包；SSH `TimeoutError`；`dzsmart.local` 只剩过期缓存（解析到 `192.168.55.1`，Jetson USB-C gadget 常用址，实测也不可达） |
| 当前网段 `192.168.137.0/24` | 扫到**只有两台**：`.1` = `thunderrobot.mshome.net`（网关/热点宿主，22 开）、`.241` = `duzhong.mshome.net`（**车端**，22 开） ⇒ **无人机不在这个网段** |
| 本机网络 | WLAN SSID **`www`**，`192.168.137.51/24`，网关 `192.168.137.1`，802.11ax / ch149 / 信号 62% |
| 车端 | ✅ 在线可达（ICMP 11–15 ms、22 开）。按队长指示**车侧探测暂缓**，本单未登车、未跑车侧 ROS 探针 |
| 仓库交付物哈希（= t18 复核过的在机版） | `dzlink_supervisor.py` **`5c7426d6b619da2f4b69bb3149de2d53`** · `dz_send_waypoint.py` **`eb41dbf4b944bc7e8af8e61ec940a228`** · `test_dzlink_supervisor.py` `558f40a2c4cd77c5a0fc7769e6ddcfa4` · `test_dz_send_waypoint.py` `a95a49ff4b43cc0c8e01161088d69975` |
| 仓库侧口径自检（本机可跑） | `selftest` **全部通过**；单测 **81 OK / 24 OK**；`contract` 横幅实测 = `情报过期 30s` / `Mission 等情报 2s → 默认 K10(下)/K5(上)` / `自动出发延时 45s` / `心跳判离线 3s` / `车状态上报 0.7s` |

⇒ **结论**：链路交付物本身健康（口径/单测/哈希都对），**卡在"无人机没上线"**。
§1 与 §2 的"在机执行"部分需要在无人机接入同一 AP 后按本单粘贴执行；§3 需与队长对时后再动。

### 0.1 本次用到的"找机"手段（IP 变了就复用）
```powershell
ping -n 3 10.29.122.104                                   # 历史地址先试
python _tmp_session\drone-link\find_drone_here.py         # 扫当前 /24 的 TCP:22（并发，只连端口不登录）
netsh wlan show interfaces                                # 确认本机在哪个 SSID/网段
Resolve-DnsName dzsmart.local                             # mDNS（抖动/换网后常只剩过期缓存，别只信它）
```
判读：**本机网段里没有 22 开的第三台机器 ⇒ 无人机不在本网段**（本次就如此）。

### 0.2 ★ 时钟与对时（发车日前必须做）
- 发车日开机后，在**三方**各看一眼时间并写进当日记录：
  - 无人机：`date -Is; timedatectl | grep -E "synchronized|NTP"`
  - 小车：同上（`timedatectl`）
  - 本机：`Get-Date -Format 'yyyy-MM-dd HH:mm:ss zzz'`；权威比对用 HTTP `Date` 头（见上）
- **偏差 >1 s 就要处理**（`timedatectl set-ntp true`；无外网时以本机/手机时间为准手工校时）。
- 为什么重要：
  1. **车侧新鲜度判定用的是车自己的时钟**（`tasks/drone.py` 里 `bridge_ts = now`，不用无人机发来的 `ts`）
     ⇒ 机/车时钟偏差**不会**造成"情报误判过期"；但日志、`ack` 里的 `ts` 会**对不上**，排查时会误导。
  2. 若有人按 `data['ts']`（无人机时钟）判新鲜度，**必须容忍负 age**（t6 口径已写"age<0 视为新鲜"），
     否则 3 天偏差会让每条情报都被当成"过期/未来"。
  3. 哈希台账/证据里记的 mtime 也会跟着偏差（本次就出现"报告写 10-12、实际 10-15"）。

> ⚠️ 换 AP 后 **IP 会变**（历史见过 `10.240.174.104` / `10.226.211.104` / `10.158.138.104` / `10.200.61.104` / `10.29.122.104`）。
> 优先用 `dzsmart.local`，mDNS 抖动时用当时 DHCP 到的 IP，**不要在脚本里写死**。

---

## 1. 无人机侧起栈顺序（在机上执行）

**S0 · 物理/网络前置**
- [ ] 无人机上电；接入**同一个 AP**（当前场地是 SSID `www` ⇒ 加入后应拿到 `192.168.137.x`）
- [ ] 从本机确认可达：`ping <无人机IP>` 或 `ssh -4 duzhong@dzsmart.local "hostname; date -Is; ip -4 addr"`
- [ ] 雷达网口 `eno1` 是否连上不影响指挥链路（那是 FAST-LIO 用的 `192.168.3.x`），可后置

**S1 · 版本与环境核对（只读，先确认"跑的是哪一版"）**
```bash
ssh -4 duzhong@dzsmart.local
md5sum ~/dzlink_tools/dzlink_supervisor.py ~/dzlink_tools/dz_send_waypoint.py
# 期望：5c7426d6b619da2f4b69bb3149de2d53  /  eb41dbf4b944bc7e8af8e61ec940a228
# 不一致 ⇒ 先重部署（见 §5），别急着跑
```
```bash
source /opt/ros/humble/setup.bash
source ~/dzlink_tools/dzlink.env          # ★ 域号唯一真相源：ROS_DOMAIN_ID=118
echo "DOMAIN=$ROS_DOMAIN_ID RMW=${RMW_IMPLEMENTATION:-default}"
# 期望：DOMAIN=118；RMW 缺省=rmw_fastrtps_cpp（与车侧一致）
```

**S2 · 不上 ROS 的自检（10 秒，先排掉代码层问题）**
```bash
python3 ~/dzlink_tools/dzlink_supervisor.py selftest   # 期望「结果：全部通过」
python3 ~/dzlink_tools/dzlink_supervisor.py contract   # 期望：Mission 等情报 2s → 默认 K10(下)/K5(上)；自动出发延时 45s
```

**S3 · 常驻上线（正式跑法）**
```bash
mkdir -p ~/dzlink_tools/logs ~/dzlink_tools/routes
# 情报预置（发车日关键，见 §2）：先写命令文件，再起常驻
cat > ~/dzlink_tools/cmd.jsonl <<'EOF'
{"type":"bridge","half":"lower","block":"K10"}
{"type":"bridge","half":"upper","block":"K5"}
EOF
setsid nohup python3 ~/dzlink_tools/dzlink_supervisor.py run \
    --cmd-file ~/dzlink_tools/cmd.jsonl \
    --log-file ~/dzlink_tools/logs/supervisor.jsonl \
    > ~/dzlink_tools/logs/supervisor.stdout 2>&1 < /dev/null &
sleep 3
pgrep -af dzlink_supervisor            # 期望 1 个常驻进程
tail -n 5 ~/dzlink_tools/logs/supervisor.jsonl   # 期望看到 intel_in(两半场) / intel_emit / heartbeat
```
> 心跳 1 Hz 是"无人机在线"的唯一保活手段（车侧 3 s 无心跳即判离线）；
> bridge 每 2 s 重发一次、且**任一新鲜半场到点会把两半场同轮一起发**（t16 保证，实测 lower→upper ≤0.03 s）。

**S4 · L2 冒烟（不依赖小车，机上自带冒充端）**
```bash
bash ~/dzlink_tools/smoke_ros.sh        # 期望 exit 0：≥3 条心跳 / lower+upper 都在 / target_zone / field_verify / /nav/waypoint 路线 / 日志有 car_ack
```
> 没有 `smoke_ros.sh` 就从仓库重部署（§5）；它用**本机临时订阅者冒充小车**，所以**不碰车**。

**S5 · 存证（当日留档，便于事后回答"跑的是哪一版"）**
- [ ] 记 `md5sum` 输出（两个 .py）
- [ ] 记 `contract` 输出
- [ ] 记 `smoke_ros.sh` 的判据行
- [ ] 记 `date -Is` + 无人机 IP

---

## 2. 情报口径与"什么时候发"（发车日最容易翻车的地方）

车侧真值（读自车上 `config/robot.yaml`，t16 已对齐）：

| 车侧参数 | 真值 | 含义 |
|---|---|---|
| `mission.bridge_info_wait_s` | **2.0 s** | 等无人机桥洞情报的最长时间；且**只管「两半场相隔 ≤2 s 到达」**，超时那半场只进遥测、**路线已冻结** |
| `mission.auto_start_delay_s` | **45.0 s** | `auto_start=true` 时起栈后 45 s 车就出发 |
| `route.cross_lower_default` / `cross_upper_default` | **K10 / K5** | 等不到情报时的兜底穿洞格（本图真值） |
| `drone.info_stale_s` | 30 s | 情报 30 s 过期 ⇒ 车侧当"无情报" |
| `drone.heartbeat_offline_s` | 3 s | 3 s 无心跳判无人机离线 |

**操作口径**：
- [ ] **情报在车起栈之前就写好**（用 §1-S3 的 `--cmd-file` 预置，或一次性 `once`）。不要指望"发车后再抢窗口"——窗口只有 2 s。
- [ ] 两半场一次给全（`--bridge lower=… --bridge upper=…` 或两个命令文件行）；成对已由 supervisor 保证。
- [ ] 若桥顶箭头识别**晚于**车起栈：仍继续发情报（每 2 s 会重发 + 内容一变立即重发），但要知道**可能已经来不及换洞**（路线冻结），当场按"是否已发车"决定要不要重发路线。
- [ ] 一次只留**一个**写者：正式跑用 `dzlink_supervisor`；`dz_send_waypoint.py` 只做单次诊断（两者都写 `/dzlink/drone_to_car`，同时跑会让 ack 台账失真）。

单次诊断命令（不需要车在线）：
```bash
python3 ~/dzlink_tools/dz_send_waypoint.py bridge lower=K10 upper=K5 --dry-run   # 只打印报文
python3 ~/dzlink_tools/dz_send_waypoint.py heartbeat --duration 30               # 真发心跳，看「订阅者匹配数」
python3 ~/dzlink_tools/dz_send_waypoint.py listen --duration 30                  # 听车回传（car_state/ack）
```

---

## 3. 车侧 DDS 探针时机（★ 车在线后与队长对时后再动）

**前置三件（缺一不做）**
- [ ] 车端在线：`ping 192.168.137.241`（或当时 IP）+ 22 端口可达
- [ ] **与队长对时**：确认车栈此刻是"冷态待命"还是"已起栈/已发车"——探针要在**不干扰正式跑**的窗口里跑；顺手**对一下时钟**（§0.2）
- [ ] 两端域号一致：车侧由 `~/dzrobot/start_all.sh:123` source `~/dzrobot/dzros.env`（=118）；机侧由 `~/dzlink_tools/dzlink.env`（=118）

**探针只放 /tmp、只读，绝不写 `~/dzrobot/**`**
```bash
# 1) 把探针拷到车 /tmp（不碰车上的持久配置）
scp dzsmartscout/tools/link/probe/t4_car_consumer.py duzhong@192.168.137.241:/tmp/
# 2) 在车上启动只读消费者（默认不自动回 ack；只有要验"车收到了"时才加 --auto-ack）
ssh duzhong@192.168.137.241 'bash /tmp/t4_rosenv.sh python3 /tmp/t4_car_consumer.py \
     --qos best_effort --duration 40 --out /tmp/t4_car_recv.jsonl'
# 3) 同时机上发一次情报（另一终端）
python3 ~/dzlink_tools/dz_send_waypoint.py bridge lower=K10 upper=K5 --seq 1
```
判据：车上打印 `[rx OK] … half/block/…`（+ `subscribers`/`publishers` 计数），机侧打印「订阅者匹配数 ≥1」。
两侧都通 = 链路活；只有机侧 ≥1 而车上无 `[rx OK]` ⇒ 看 §4-B。

> **网络边界**：本次复验时车端 `192.168.137.241` 与本机同网段（可直连）；
> 若发车日车端在**别的网段**（队长提示过这种可能），则**只报信息、不硬连**——
> 由队长决定是否把无人机/本机接到车的网段，或改用热点统一网段。

---

## 4. 常见故障三步排查（先看证据，再动手）

### A · `subs=0`：发了，但"没有任何订阅者"
1. **域号**：两端都 `echo $ROS_DOMAIN_ID`（机 118 / 车 118）——注意 `ssh host 'cmd'`、systemd、cron **不读 `~/.bashrc`**，会掉回域 0；
   车侧唯一真相源是 `~/dzrobot/dzros.env`（`start_all.sh` 会 source 它）。**域号不一致的症状是 ping/SSH 全通、ROS 互不可见、零报错。**
2. **车是否起栈**：车上 `pgrep -af dzbrain`（`drone.enabled=true` 才会读 `/dzlink/drone_to_car`）。
3. **发现通道**：换 AP 后必测——车上 `ros2 multicast receive` + 机上 `ros2 multicast send`。
   老 AP 实测原始组播丢包 **50%~63%**（DDS 靠重复宣告仍能发现，数据面走单播）；若新 AP 更严，用
   `dzsmartscout/tools/link/fastdds-unicast-wifi.xml` 单播兜底（两端同时设 `FASTRTPS_DEFAULT_PROFILES_FILE`）。

### B · 匹配数 ≥1 但车没反应：**QoS 档位不匹配 = 静默丢**
1. **档位**：`/dzlink/drone_to_car` = **BEST_EFFORT** depth10（车侧 sub）/ `/nav/waypoint` = **RELIABLE** depth10 /
   `/dzlink/car_to_drone` = BEST_EFFORT。**BEST_EFFORT 发 → RELIABLE 收是不兼容**（反之兼容）。
   实测：不兼容时两端"发现层"仍各显示 1 个端点，**只有实收是 0** ⇒ 不能拿 `ros2 topic info` 的计数当"通"。
2. **订阅端 WARN**：`New publisher discovered … offering incompatible QoS … Last incompatible policy: RELIABILITY`
   —— 只有订阅端进程日志里有一行，重定向到文件后基本看不见。
3. **实收对账**：车侧探针（§3）看 `[rx OK]` 与字段；机侧 `dz_send_waypoint.py listen`。

### C · `listen` 收不到车回传（`/dzlink/car_to_drone`）
1. **车在发吗**：车上 `pgrep -af dzbrain` + `drone.enabled=true`（车 1~2 Hz 发 `car_state`）。
2. **域名一致**（同上）。
3. **QoS**：车发 BEST_EFFORT；`ros2 topic echo` 默认 RELIABLE ⇒ **debug 时必须显式 `--qos-reliability best_effort`**，否则误判"车没在发"。
   也可直接看机侧日志 `car_msgs/acks`（`logs/supervisor.jsonl`）。

### D · 组播/发现通道（换 AP / 换场地后的第一步）
1. 车上 `ros2 multicast receive`（先起，等 15 s）→ 机上 `ros2 multicast send`；
2. 收到 `Received from …` ⇒ 组播通，问题在别处；
3. 收不到 ⇒ 走单播兜底 XML（同上），或用有线/热点统一网段。

### E · 现场工具自身坑（别把工具故障当链路故障）
1. **`ros2` CLI daemon 会卡死**：`ros2 topic list/node list` 抛 `xmlrpc … Fault: RuntimeError: !rclpy.ok()` ⇒ `ros2 daemon stop`（下次自动重启）。
   **rclpy 节点不受影响** ⇒ 判断链路以探针**实收**为准，别只看 `ros2 topic list`。
2. **`dzs` 端口/端口号**：DDS 发现端口 = `7400 + 250×域号`（域 118 ⇒ **36900**），在 7400 上嗅探**永远看不到**对端。
3. **VOLATILE 无补发**：首包必丢一（实测 39/40）⇒ 情报必须重复发（supervisor 已做：每条 3 遍 + bridge 每 2 s）。

---

## 5. 停止 / 回退 / 重部署

```bash
# 停常驻（用 -f：脚本名较长，-x 匹配不到）
pkill -f dzlink_supervisor
# 或往命令文件追加一条： echo '{"type":"stop"}' >> ~/dzlink_tools/cmd.jsonl
```
- **回退**：机上的每次部署都在 `~/dzlink_tools/_backups/<卡号>_<时间戳>/` 留了旧文件（`cp -p` 保真），拷回即可。
- **重部署（从本机仓库推，幂等）**：
```powershell
cd D:\User\Desktop\比赛存档\v1
python dzsmartscout\tools\link\probe\t18_deploy.py     # 部署 4 个文件 + 机上 md5 复核 + 哈希清单
python dzsmartscout\tools\link\probe\t16_deploy.py     # 只推 supervisor + 单测
```
- **本机口径自检（不需要 ROS / 不需要无人机）**：
```powershell
python dzsmartscout\tools\link\dzlink_supervisor.py selftest
python dzsmartscout\tools\link\dzlink_supervisor.py contract
python dzsmartscout\tools\link\tests\test_dzlink_supervisor.py            # 81 OK
python -m unittest dzsmartscout.tools.link.tests.test_dz_send_waypoint    # 24 OK
```

---

## 6. 卡在哪 / 找谁

| 现象 | 谁负责 | 依据 |
|---|---|---|
| 无人机不上线 / 不在同一网段 | 现场（用户） | §0 快照：机上端当前不可达，需上电并接入同一 AP |
| 车侧栈起没起 / 何时发车 | 队长（对时） | §3 前置三件 |
| 桥洞半场情报语义（路线冻结后能否换洞） | car-integrator（t6 线） | t4 报告 §8-D3 |
| 情报数值口径（2.0 s / K10,K5 / 45 s） | 本单 §2（已对齐 t16） | t16 口径对齐说明 |

---

## 附：本次复验用到的命令（可复跑）

```powershell
# 无人机在线性 + 当前网段扫描（本机）
ping -n 3 10.29.122.104
python _tmp_session\drone-link\find_drone_here.py         # 扫 192.168.137.0/24 的 TCP:22
netsh wlan show interfaces                                # 看 SSID/信道/信号

# 仓库侧口径自检
python dzsmartscout\tools\link\dzlink_supervisor.py selftest
python dzsmartscout\tools\link\dzlink_supervisor.py contract
python dzsmartscout\tools\link\tests\test_dzlink_supervisor.py
python -m unittest dzsmartscout.tools.link.tests.test_dz_send_waypoint
```

**本次实测摘要**：无人机不可达（不在 192.168.137.0/24，历史地址 10.29.122.104 丢包 100%；`dzsmart.local` 仅剩过期缓存）；
车端 `.241` 可达（按指示未硬连）；`selftest` 全通过；单测 81 OK / 24 OK；
`contract` 口径 = **2.0 s / K10(下)+K5(上) / 45 s / 30 s / 3 s**；
★ **发现 ~3 天时钟偏差**（机/本机当时读数 2026-10-12，真实时间 2026-10-15，两个独立 HTTP `Date` 头核对）⇒ 发车日前先对时（§0.2）。
