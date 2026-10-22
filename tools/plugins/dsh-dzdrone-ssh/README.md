# dsh-dzdrone-ssh

在 DeepSeek Harness 里操作 **dzdrone 无人机**（NVIDIA Jetson Orin Nano / `dzsmart.local`）：侧边栏图标进入
**无人机面板 = 工具列表**，每行一个工具，点"打开"即在**右栏以标签页形式**打开（和内置终端一样的窗口模式）。

| 工具 | 打开为 |
| --- | --- |
| **SSH 终端** | 复用 DSH 内置 `terminal` 类型——回显、Tab 补全、Ctrl-C、缩放都是原生的（Host 拼命令，Client 写入终端视图） |
| **文件目录** | 插件自有标签类型 `dzd-files`：浏览、预览、下载、上传无人机文件（走同一条已信任的 SSH 通道） |
| **VNC 桌面** | 插件自有标签类型 `dzd-vnc`：noVNC + WebSocket 隧道到无人机的 TigerVNC :1 |
| **状态浮窗** | **不进标签页**：挂到 `shell.overlay`（帧级浮层），一张可拖动的小卡片：标题栏显示 **无人机状态 \| 开机时间 m:ss**（无人机 `/proc/uptime`，断连时显示「无人机未连接」），主体是 **温度 / CPU 占用** 两块蓝框卡片，各带动态坐标轴曲线（x 轴当前时间 ±30 秒、5 秒一刻、HH:MM:SS 标注，y 轴当前值 ±5°C / ±10%、虚线十字与红点标出精确值），5 秒一轮自动刷新，不透明度可调 |

入口只有一个：侧栏无人机的图标，点它进工具列表。

**右栏属于会话**（DSH 的结构：主视图切到插件面板时，右栏的 on-screen Session 被清空），
所以点"打开"会先把主视图切到会话、再在会话右侧开标签——列表上方的提示写明了这一点；
切换失败会把面板切回去，错误显示在你点的地方。

**以后加新工具**（这是这套结构的目的）：
1. 在 `client.js` 的 `TOOLS` 数组加一行 `{ id, kind, mode: 'tab', titleKey, descKey }`；
2. 在 `apply()` 里 `ctx.get('sidebarRightTabs').register(...)` 注册标签类型
   （id、kind、`multiple: false` 让重复点击聚焦已有标签），并
   `ctx.slots.inject('sidebar.right.pane.tab', ...)` 以 **类型 id 为 key** 注册标签体
   （`TabSlot` 按 `entryKey: definition.id` 分发，key 写错会渲染"类型不可用"占位）；
3. 字典里加两条文案。
右栏会经 `openContent → expanded → RightbarSeat syncPresentation → openRightbar` 自动展开，
无需自己开栏。VNC 标签设了 `keepMounted: true`——iframe 不随切换卸载，桌面不断线。

**不占标签页的工具**（状态浮窗就是这种）：`mode: 'float'` 的行翻转 store 里的开关，
卡片由 `apply()` 里 `ctx.slots.inject('shell.overlay', ...)` 注册的座位渲染——
浮层在所有栏之上、且在它们的滚动容器之外，所以切面板不会把它带走。
开关本身写进 `localStorage`，刷新页面后还开着。

## 设计要点

**为什么终端不自己实现**：DSH 的终端服务在 Host 侧只允许用「已发现的 shell」启动进程，
所以插件不能把 `ssh` 当成 shell 塞进去。做法是：Host 拼好命令行，Client 打开 DSH 的终端标签页
并把命令写进去——于是你得到的是应用自带的那套终端，而不是一份仿制品。

**VNC 为什么要桥**：浏览器不能开裸 TCP 连到 5901，而无人机上没有 websockify/noVNC。
所以 Host 半身做了两件事——在 `/api/dzdrone-ssh/vnc` 注册一个 WebSocket upgrade 座位，
自己完成 RFC 6455 握手与分帧，然后把它和无人机的 VNC 端口对接：

```
浏览器 noVNC  ──WebSocket──▶  /api/dzdrone-ssh/vnc  ──TCP──▶  无人机 5901 (TigerVNC :1)
```

- `vncMode: direct`（默认）直接连 `<host>:5901`——无人机当前就是 `0.0.0.0:5901`。
- `vncMode: tunnel` 走 `ssh -N -L`，当 VNC 只监听本地时只有这个模式能用，且复用已信任的 SSH。

桥是纯字节泵，不解析 RFB 流：编码方式、扩展、未来协议版本都归 noVNC 管。

**VNC 画面为什么放在 iframe 里**（踩过的坑，值得记住）：DSH 的页面跑在自定义 scheme
`dsh-app://app/` 下，不是 http。在那个页面里：

- `import()` 动态导入会失败：`Failed to fetch dynamically imported module: dsh-app://app/api/...`；
- 用 `location.host` 拼 WebSocket 会得到 `ws://app/...`，而 `app` 不是网络主机。

所以 VNC 画面是一个**独立文档** `/api/dzdrone-ssh/vnc.html`，由 Host 半身用 http 提供，
面板用 iframe 嵌它、只用 `postMessage` 递消息。iframe 内部是普通同源 http，
`import './novnc/core/rfb.js'` 与 WebSocket 都是标准行为。面板所需的地址来自
`/status` 的 `origin` —— 它是**用「收到这个请求的 socket 的 localPort」反推**出来的
（`http://127.0.0.1:<port>`），因为插件没有别的办法知道服务器监听在哪。
另外，回环地址在 Chromium 里算「可信来源」，所以从 `dsh-app://` 页面加载
`http://127.0.0.1` 的 iframe 不会被当作混合内容拦掉。

**文件为什么要用 ssh 而不是 SFTP**：Host 没有 SFTP 客户端库，而 `ssh` 已经装好、已经认证、
已经验证过能连上这台无人机。所以每个操作就是一条 `ssh …命令`，答案用机器可读的格式回传：
目录列表用 `find -printf` 的 NUL 分隔字段再 base64 包裹（`ls` 解析会在机器人项目真实存在的
奇怪文件名上崩掉——名字里有空格、引号也能原样往返，`scripts/check-files.mjs` 就在测这个），
读取用 `base64 -w0`，下载用 `cat` 流式吐，上传用 `cat >` 流式收。

**安全**：所有路由只接受本机回环；装了 `connection` 服务时走它自己的准入判断。
文件名与路径在进入远程 shell 之前，只经过 `src/files.js` 的 `quote()` 一个收口点。

## 安装位置

已经在 `~/.dsh/profiles/desktop` 里注册好了：

- `node_modules/dsh-dzdrone-ssh` 是 pnpm 对本目录（`tools/plugins/dsh-dzdrone-ssh`）`file:` 依赖的**打包副本**（不是 junction）——
  改这里的代码要 `pnpm install` 同步，`scripts/test-package-files.mjs` 会逐字节核对副本没过期；
- `package.json` 的 `dsh.profile.bundles` 里加了 `dsh-dzdrone-ssh`
- 本包的 `cordis.patch.yml` 负责挂载 Host 半身；浏览器半身由 `package.json` 的 `dsh.client` 声明

## 改代码后怎么生效

| 改的是 | 生效方式 |
| --- | --- |
| `client.js`（浏览器半身） | 刷新页面 |
| `index.js` / `src/*`（Host 半身） | `curl -X POST http://127.0.0.1:19387/api/dzdrone-ssh/reload` |

Host 半身是 Node 的 ES 模块，进程按 URL 缓存，**改文件不会自动生效**。所以插件自带
`src/reload.js`：把本包从 loader 的模块缓存里逐出、重新 import、再注册到同一个 fiber 上，
失败会回滚到旧版本。也可以在该插件条目的 config 里写 `watchSources: true`，改文件自动重载。

> 新增 `src/*.js` 或改 `package.json` 的 exports 之后，第一次仍需重启应用一次
> （模块解析本身也被缓存）；之后普通改动都是 `POST /reload` 秒级生效。

## 配置

设置页（Settings → 无人机 SSH）：主机/IP、用户名、端口、`ssh` 别名（填了优先用别名）、
附带的 `ssh` 参数，以及 VNC 主机/端口/通道/密码、文件浏览起始目录。
存在 `~/.dsh/dzdrone-ssh/settings.json`。

默认值取自这台机器上已有的 `~/.ssh/config`：`dzsmart.local` / `duzhong` / 22 / `~/.ssh/id_rsa` 免密；
VNC 默认直连 5901（TigerVNC 在显示 `:1`）。

### 无人机侧的 VNC 现状（本次已确认/已改）

- `vncserver@:1.service` 常驻，`Xtigervnc :1` 监听 **5901**（不是 5900），认证 `VncAuth,TLSVnc`。
- **当前分辨率是 1920x1200**（`-geometry 1920x1200`，2026-10-06 在 `10.240.174.104` 上实测）：
  改分辨率走无人机侧脚本 `scripts/restart-vnc-1080p.sh` 或直接改
  `/etc/systemd/system/vncserver@:1.service` 的 `-geometry` 与 `~/.vnc/config` 的 `geometry=`
  后 `systemctl restart vncserver@:1`（两处都要改，避免互相盖回去）。
  > 注意：重启会把 `:1` 上所有 X 客户端杀掉，需要在无人机侧重新拉起。
- **`-localhost no` 一直就是开的**（`localhost=no`），ufw 里 5901 规则也早已存在；
  ufw 服务本身处于"不活动"（不过滤流量），所以这两项实测都是多余操作。
- **VNC 密码实测就是 `duzhong`**（2026-10-06 用真实 RFB 握手在 `dzsmart.local:5901` 验证通过），
  已作为插件默认值写入设置，面板连接时免输入；改密码后可在设置页更新或改回留空。

- **TigerVNC 黑名单（实测，值得知道）**：`UseBlacklist` 默认为开，**一个连上却没有完成认证就断开的客户端会被拉黑**
  —— 无人机上 `~/.vnc/duzhong:5901.log` 里就是这么记的：
  `Connections: accepted: …` → `Clean disconnection` → `Connections: blacklisted: …`。
  也就是说，**输错密码、或者连上后没登录就关掉，都会吃一次黑名单**；本机实测约 1 分钟就恢复
  （重复违规会翻倍）。因此：
  - 面板里的 VNC **不会自动连接**，必须点一下才连——避免打开面板就吃一次黑名单；
  - 被拉黑时面板会直接说明原因和等待时间，而不是报一个看不懂的错；
  - 如果想彻底避免，可以在无人机的 VNC 单元上加 `-BlacklistThreshold=0` 或 `-UseBlacklist=0`（这是降低安全性，需你决定）。

## 通用 VNC 连接器（即「VNC 桌面」标签页）

**没有独立的侧栏图标**：连接器就是无人机面板工具列表里的「VNC 桌面」那一行（右侧栏标签页）。
一个 VNC 表面同时服务无人机和**任意** TigerVNC + websockify 端点：

```
VNC 桌面标签页 ──iframe──▶ /vnc-connect.html ──noVNC──▶ ws://host:port/websockify ──▶ 目标机 TigerVNC
                              └─ 或切到「无人机通道」：ws://<DSH 地址>/api/dzdrone-ssh/vnc ──TCP──▶ 无人机 5901
```

- **首次打开自动预填「无人机通道」**：插件自己的 WebSocket upgrade（同 `/vnc` 桥，直通无人机
  5901，机上不需要 websockify），密码取 `/status` 里保存的 VNC 密码；只要你改过任一字段，
  你的值就持久化在 localStorage 并优先生效。预填的主机名：**回环源（`127.x.x.x`/`::1`）写
  `localhost`**——桥本来就连本机的 Host 半身，直说 localhost 比显示一个用户从没打过的 IP 强；
  非回环源保留原主机名，那里 localhost 会指错机器。
- **「无人机通道」按钮 = 一键回到一定可用的目标**：通用指引把表单指向一个从未安装的
  websockify、吃一个裸 1006 时，不用重打四个字段，点一下就切回来。
- **输入框**：协议（ws/wss）、VNC 主机、WebSocket 端口、路径（默认 `websockify`）、
  VNC 密码、分辨率（自适应或固定值，固定值走 TigerVNC+RandR 的 SetDesktopSize）。
- **失败解释按目标分叉**：1006 发生在「无人机通道」上→提示查无人机在线/VNC 监听；
  发生在自填目标上→提示 websockify/端口/协议三类原因。
- **独立窗口/新标签页**：同一份文档换个壳，非敏感字段走 URL 参数，密码永不进 URL。
- 两种宿主（Electron/Tauri WebView、浏览器插件）由 `detectRuntime` 识别并在页头打徽标。

### Ubuntu 端：TigerVNC + websockify 完整命令（连无人机以外的机器才需要）

一键脚本（在目标 Ubuntu 上 bash 执行）：`scripts/ubuntu-vnc-websockify.sh`（从小车插件移植，
做的事）：装包 → `vncpasswd` → systemd 单元（`vncserver@:1` + `websockify-5901`）→ 打印两种
暴露方式（局域网放行 6080 / SSH 隧道，推荐后者）。要点：

- **TigerVNC 新版默认只绑回环**，`-localhost no`（或至少让 websockify 能连到它）是必须的；
- `websockify` 是 WebSocket→TCP 代理，**路径参数默认被忽略**，`/websockify` 只是 noVNC 惯例写法；
- 连无人机本身**永远不需要**这些——用「无人机通道」即可。

## HTTP API

前缀 `/api/dzdrone-ssh`，仅本机回环可访问。

| 方法 | 路径 | 作用 |
| --- | --- | --- |
| GET | `/status` | 配置、上次探测、无人机信息、等价命令行、VNC 目标 |
| POST | `/probe[?drone=0]` | 探测可达性与免密登录 |
| POST | `/describe` | 采集无人机信息（用户/主机名/内核/运行时长） |
| GET/POST | `/config` | 读取 / 保存配置 |
| GET | `/files?path=` | 列目录（NUL+base64，名字里带空格/引号也不怕） |
| GET | `/file?path=` | 读文本（二进制会被识别出来而不是渲染成乱码） |
| GET | `/stat?path=` | 取大小/类型/时间 |
| GET | `/download?path=` | 流式下载 |
| POST | `/upload?path=` | 流式上传 |
| GET | `/metrics` | 浮窗读数：温度（多测点取最高）、CPU 占用、开机秒数（无人机 `/proc/uptime`）+ 开机时刻（`/proc/stat` 的 `btime`）、负载、时间戳；**永远 200 + `ok` 标志**，无人机不在线也是"可渲染的读数"而不是 HTTP 错误 |
| GET | `/float.html` | 独立小窗版浮窗（同源、只读，调 `/metrics`） |
| GET | `/vnc-connect.html` | **通用 VNC 连接器**文档（noVNC 直连 ws://host:port/path，带嵌入面板的 postMessage 协议） |
| GET | `/novnc/*` | 本插件自带的 noVNC 静态资源（原生 ES 模块） |
| WS | `/vnc` | VNC 字节隧道（upgrade 座位） |
| GET/POST | `/reload` | 查看上次热重载 / 立即热重载 Host 半身 |
| GET/POST | `/diag` | 诊断日志（浏览器半身上报，落 `diag.jsonl`） |
| GET/POST | `/selftest` | 一次性自检开关 |

## 自检脚本

```bash
# 本机（Node）
node scripts/check-ssh.mjs      # SSH 层：真实免密登录、描述
node scripts/check-files.mjs    # 文件层：列举/读取/二进制识别/奇怪文件名往返/符号链接解引用
node scripts/test-host.mjs      # Host 半身：全部路由 + noVNC 资源 + VNC 隧道（本机替身）
node scripts/test-client.mjs    # Client 半身：五个座位注册与渲染 + 浮窗开关两态 + 连接器 + 完整连接流程
node scripts/test-vnc-guard.mjs # 缓存地址防伪：非 VNC/哑巴对端拒连、横幅校验、失败即清缓存
node scripts/test-vnc-wrapper.mjs # 两份 HTML 的 WebSocket 探针必须过 noVNC 的原型链检查
node scripts/test-vnc-connect.mjs # 连接器文档逻辑无头验证：URL 构造/分辨率/缩放/密码事件/面板协议
node scripts/test-package-files.mjs # files 白名单 + 安装副本逐字节比对（防"装出来的插件缺文件/过期"）
node scripts/test-live-ws.mjs   # 对着「运行中的插件 + 无人机」打一发真实 VNC 横幅
node scripts/debug-ws-handshake.mjs # Chrome 形状的升级请求打印 WS 握手原文 + accept/协议/扩展合法性判定
node scripts/debug-files.mjs    # 目录列不出来时，直接打印远程命令的原始退出码/stderr

# 无人机侧（由你手动 scp 后 bash 执行——见仓库根 WORKFLOW.md：助手对无人机只读）
scripts/probe-drone.sh            # 勘察：桌面/VNC/监听端口/SFTP
scripts/probe-vnc-config.sh     # 勘察：VNC 单元、geometry、:1 上的客户端
scripts/restart-vnc-1080p.sh    # 改 1080p + ufw + systemd 重启（杀 X 客户端，见上）
scripts/check-vnc-suggestion.sh # 核对 localhost/ufw/geometry/占用进程的实况
scripts/vnc-log-families.sh     # 按地址族统计 VNC 黑名单条目
scripts/ubuntu-vnc-websockify.sh # 目标机一键装 TigerVNC + websockify（给连接器连非无人机目标用）
```

`test-host.mjs` 的 VNC 部分是**对着本机的一个替身 TCP 服务**测的，不是对着无人机：
按上面的黑名单机制，对着无人机「连上读个横幅就挂断」本身就是一次安全失败，会把操作者自己的
笔记本拉黑几分钟——测试不该做这种事。替身反而测得更严：70300 字节的往返要求**逐字节相等**，
并且覆盖 WebSocket 三种帧长编码（7 位 / 16 位 / 64 位）。

想真的对着无人机跑一次，就显式打开（跑一次就够，别连跑）：

```bash
DZD_TEST_DRONE_VNC=1 node scripts/test-host.mjs
```

测试都用临时 `DSH_HOME`，**不会**动你真实的 `settings.json`。

## 关于自带的 noVNC

`novnc/` 是从 npm 上 `@novnc/novnc`（1.7.0）**原样**拷来的 `core/` 与 `vendor/`，未做任何修改，
因为浏览器半身是把它当原生 ES 模块直接 `import()` 的（56 个文件的依赖图由浏览器自己解析，
所以不需要打包步骤）。许可证 MPL-2.0，见 `novnc/LICENSE.txt`，升级时整目录替换即可。

## 排错

- **侧栏图标是红点**：无人机不可达。确认它上电、连上同一 WiFi（本机与它同一局域网），
  `ping dzsmart.local`。
- **黄点 / "需要密码"**：能连上但免密不成立，把公钥装到无人机：
  `ssh-copy-id -i ~/.ssh/id_rsa.pub duzhong@dzsmart.local`。
- **VNC 连不上**：先确认无人机上的 `vncserver@:1.service` 在跑（`systemctl status vncserver@:1`），
  端口是 **5901**（不是 5900）。若 VNC 只监听本地，把设置里的通道改成「走 SSH 隧道」。
- **面板点了没反应**：看 `~/.dsh/dzdrone-ssh/diag.jsonl`，每一步都有记录；
  也可以在设置页复制等价命令，直接在终端里粘贴。
