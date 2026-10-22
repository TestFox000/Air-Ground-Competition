# dsh-dzairground-ssh

> **S2/S3 状态（2026-10-17）**：本包已从「换名的小车插件」升级为**双节点空地协同面板**——
> `src/nodes.js` 统一节点注册表让 `/api/dzairground-ssh/car/*` 与 `/drone/*` 共用同一条
> handler 链；机侧 metrics（温度 + PX4 飞控电量 + batteryNote 词汇）移植自冻结机侧插件。
> 状态浮窗为一张卡多格（车电量/CPU + 机温度/CPU/飞控电量），飞控电量三态显示；
> 面板主视图只挂「空地协同」入口，仿真台与 64 图压测收进「高级」折叠区（12 条 `/sim*`
> 路由全保留）。下文的「小车面板/工具列表」描述仍适用于每个节点的工具行，只是侧栏
> 图标现在叫「空地协同」，主视图第一行是协同入口。

在 DeepSeek Harness 里操作 **dzrobot 小车**（RK3588 / `duzhong.local`）与 **dzuav 无人机**
（Jetson / `dzsmart.local`）：侧边栏图标进入
**空地协同面板 = 工具列表**，每行一个工具，点"打开"即在**右栏以标签页形式**打开（和内置终端一样的窗口模式）。

| 工具 | 打开为 |
| --- | --- |
| **SSH 终端** | 复用 DSH 内置 `terminal` 类型——回显、Tab 补全、Ctrl-C、缩放都是原生的（Host 拼命令，Client 写入终端视图） |
| **文件目录** | 插件自有标签类型 `dzag-files`：浏览、预览、下载、上传小车文件（走同一条已信任的 SSH 通道） |
| **VNC 桌面** | 插件自有标签类型 `dzag-vnc`：noVNC + WebSocket 隧道到小车的 TigerVNC :1 |
| **状态浮窗** | 不开标签——`shell.overlay` 座位上浮出一张可拖动的状态卡（**电量 + CPU** 两块并列曲线卡，标题栏带开机时间，5 秒自动刷新，透明度可调并持久化）；独立文档 `/api/dzairground-ssh/float.html` 同源可用 |

入口只有一个：侧栏那辆车的图标，点它进工具列表。

**右栏属于会话**（DSH 的结构：主视图切到插件面板时，右栏的 on-screen Session 被清空），
所以点"打开"会先把主视图切到会话、再在会话右侧开标签——列表上方的提示写明了这一点；
切换失败会把面板切回去，错误显示在你点的地方。

**以后加新工具**（这是这套结构的目的）：
1. 在 `client.js` 的 `TOOLS` 数组加一行 `{ id, kind, mode: 'tab', titleKey, descKey }`
   （`mode: 'float'` 的行不开标签，只翻转 store 里的 `floatOpen`——状态浮窗那行就是这么接的）；
2. 在 `apply()` 里 `ctx.get('sidebarRightTabs').register(...)` 注册标签类型
   （id、kind、`multiple: false` 让重复点击聚焦已有标签），并
   `ctx.slots.inject('sidebar.right.pane.tab', ...)` 以 **类型 id 为 key** 注册标签体
   （`TabSlot` 按 `entryKey: definition.id` 分发，key 写错会渲染"类型不可用"占位）；
3. 字典里加两条文案。
右栏会经 `openContent → expanded → RightbarSeat syncPresentation → openRightbar` 自动展开，
无需自己开栏。VNC 标签设了 `keepMounted: true`——iframe 不随切换卸载，桌面不断线。

## 浏览器控制台已下线（2026-10-09）

插件原先有一个「浏览器控制台」工具（`dzag-console`），内嵌 Eruda 调试面板。
它已**整条链路**移除：标签类型与标签体注册、`TOOLS` 里的一行、
`makeConsoleTabShell` 组件、`/eruda.js` 路由、`vendor/eruda.min.js`（约 500 KB）
与全部中英文文案键。

保留它的成本是每台机器多装 500 KB 第三方调试面板，而实际用途
（在小车自己的 Web 页面里排查 JS 错误）日常几乎为零——出问题时开
DSH 自己的开发者工具更快。`scripts/test-client.mjs` 现在会断言这些部件
**全部不再出现**，避免只删一半、面板里留下一个点开是空白页的行。

## 小车操作台已下线（2026-10-09）

插件原先有一个「小车操作台」标签，直接内嵌车上 `tools/cam_viewer.py`
（`:8080`）的页面。它连同车端工具一起删除了，因为**它不是一个可以随手
重建的网页**：

- `/api/param` 是**全车唯一的调参通道**——参数白名单、类型/范围校验、
  `.bak` + 回读回滚全部在 `param_spec.py` 里（`robot.yaml` 只是被它读写的对象）。
- `/api/rc*` 的可视化遥控器兼着**接管仲裁与四层失效保护**
  （`rc_control.py`：按住才动 / 数值钳制 / 接管看门狗 / 底盘兜底），
  用来和 20Hz 的 `path_executor`、50~100Hz 的云台争抢控制权。
- 它是 `start_all.sh` 第 8 步，也是 `aim_status_shim.py`（Int32→UInt8 类型
  兼容转发）**唯一的存在理由**。

⚠️ **真实缺口**：遥控器**没有替代品**了。底盘自身的 1.0s 无指令强制停车仍在
（最底层兜底），但"人接管"这条路现在只能自己写节点。出车前请确认这点可接受。

调参改走 `config/robot.yaml`（`healthcheck.sh` 第 3 节查配置一致性）；
需要可视化遥控时，自己写一个 ROS 节点订阅 `/nav/manual_override` 即可。

**为什么"先删插件入口、再删车端"是危险的中间态**：插件入口删掉后车端还在跑，
看起来一切正常，直到有人发现 `/api/param` 已经没有任何 UI 能用——所以两边
必须一起下线。

## 设计要点

**为什么终端不自己实现**：DSH 的终端服务在 Host 侧只允许用「已发现的 shell」启动进程，
所以插件不能把 `ssh` 当成 shell 塞进去。做法是：Host 拼好命令行，Client 打开 DSH 的终端标签页
并把命令写进去——于是你得到的是应用自带的那套终端，而不是一份仿制品。

**VNC 为什么要桥**：浏览器不能开裸 TCP 连到 5901，而小车上没有 websockify/noVNC。
所以 Host 半身做了两件事——在 `/api/dzairground-ssh/vnc` 注册一个 WebSocket upgrade 座位，
自己完成 RFC 6455 握手与分帧，然后把它和小车的 VNC 端口对接：

```
浏览器 noVNC  ──WebSocket──▶  /api/dzairground-ssh/vnc  ──TCP──▶  小车 5901 (TigerVNC :1)
```

- `vncMode: direct`（默认）直接连 `<host>:5901`——小车当前就是 `0.0.0.0:5901`。
- `vncMode: tunnel` 走 `ssh -N -L`，当 VNC 只监听本地时只有这个模式能用，且复用已信任的 SSH。

桥是纯字节泵，不解析 RFB 流：编码方式、扩展、未来协议版本都归 noVNC 管。

**VNC 画面为什么放在 iframe 里**（踩过的坑，值得记住）：DSH 的页面跑在自定义 scheme
`dsh-app://app/` 下，不是 http。在那个页面里：

- `import()` 动态导入会失败：`Failed to fetch dynamically imported module: dsh-app://app/api/...`；
- 用 `location.host` 拼 WebSocket 会得到 `ws://app/...`，而 `app` 不是网络主机。

所以 VNC 画面是一个**独立文档** `/api/dzairground-ssh/vnc.html`，由 Host 半身用 http 提供，
面板用 iframe 嵌它、只用 `postMessage` 递消息。iframe 内部是普通同源 http，
`import './novnc/core/rfb.js'` 与 WebSocket 都是标准行为。面板所需的地址来自
`/status` 的 `origin` —— 它是**用「收到这个请求的 socket 的 localPort」反推**出来的
（`http://127.0.0.1:<port>`），因为插件没有别的办法知道服务器监听在哪。
另外，回环地址在 Chromium 里算「可信来源」，所以从 `dsh-app://` 页面加载
`http://127.0.0.1` 的 iframe 不会被当作混合内容拦掉。

**文件为什么要用 ssh 而不是 SFTP**：Host 没有 SFTP 客户端库，而 `ssh` 已经装好、已经认证、
已经验证过能连上这台车。所以每个操作就是一条 `ssh …命令`，答案用机器可读的格式回传：
目录列表用 `find -printf` 的 NUL 分隔字段再 base64 包裹（`ls` 解析会在机器人项目真实存在的
奇怪文件名上崩掉——名字里有空格、引号也能原样往返，`scripts/check-files.mjs` 就在测这个），
读取用 `base64 -w0`，下载用 `cat` 流式吐，上传用 `cat >` 流式收。

**mDNS 会抖，所以地址要缓存**：`duzhong.local` 是 mDNS，实测一次会话里 5 次查询有 3 次失败，
而小车全程在线。`dialOrdered` 因此记住「最近一次真正答上话的地址」（10 分钟 TTL），
解析失败时先试它，把「解析抖一下」降级成「用上次那个」。

**但缓存地址必须先验明正身**——这辆车会在网络之间游走（实测同一天出现过
`192.168.43.17` 和 `10.230.128.188` 两个地址）。车换网之后，旧地址**可能已经被同网段的别的设备占用**，
而桥是原始字节泵。于是转发任何一个字节之前，先把对方的 RFB 版本横幅读回来验一下：

- 横幅对（`RFB 003.00x`）→ 缓存条目有效，放行；
- 横幅不对、或对方根本不是 VNC、或一直不说话（5s 超时）→ **删除缓存条目并断开**，
  一字节都不转发，错误里点名是哪个地址。

注意顺序上的两个坑（都是实测踩出来的）：

1. **校验必须在装转发监听器之前**，否则那几条横幅字节已经泄给浏览器了，校验形同虚设；
2. **横幅被"扣下"之后必须补发**——服务器不会重发横幅，noVNC 在收到之前也不会发自己的版本号，
   补发晚了或丢了，握手就会卡在一个完全健康的连接上。所以 `open()` 在装好转发器之后
   才冲刷 `#pending`，而 Host 侧也必须**先 `onData()` 再 `open()`**（`index.js` 里就是这么写的）。

**为什么报错要说人话**：noVNC 的 `disconnect` 事件**只有 `{clean}`**，不带关闭码和原因——
所以「握手根本没开始」和「服务端 1011 拒绝」在面板上长得一模一样。现在
`vnc-app.html` 包了一层 `WebSocket`，把 URL 和 open/error/close 的 code+reason 都记录下来，
随 `postMessage` 上报；面板优先显示**桥自己说的原因**（比如
「无法连接 duzhong.local:5901（5 个地址均失败）：connect timeout」），
没有原因才退回 `WebSocket code N`。`C:\Users\TestFox\.dsh\dzairground-ssh\diag.jsonl` 里同样有完整线证。

**这个包装器只能用 Proxy，不能写成子类**（踩过的坑，代价很大）。noVNC 的 `Websock.attach()`
用下面这种方式做鸭子类型检查：

```js
const channelProps = [...Object.keys(rawChannel),
                      ...Object.getOwnPropertyNames(Object.getPrototypeOf(rawChannel))];
```

`getOwnPropertyNames` **只看一层原型**。写成 `class extends WebSocket` 的话，`send` / `close` /
`binaryType` / `readyState` 都留在 `WebSocket.prototype`（原型链第二层），于是 noVNC 直接抛
`Raw channel missing property: send`。更麻烦的是这个异常从 `RFB` 构造函数里冒出来，那个 socket
随即失去所有可达引用、被 GC 回收、连接被中止——浏览器侧表现为**只有 `error` + close code 1006、
从不出现 `open`**，跟「小车掉线」长得几乎一样，极难分辨。

所以现在用 `new Proxy(NativeWebSocket, { construct })`，陷阱里返回的是**原生实例**，原型链一层不变。
`scripts/test-vnc-wrapper.mjs` 把 `vnc-app.html` 里那段真实代码（靠 `/* ws-log:start */` 标记提取）
喂给 noVNC **真实的** `Websock.attach()` 跑一遍，并且额外断言「子类确实会被拒绝」——
保证这个测试不会退化成空转。

**安全**：所有路由只接受本机回环；装了 `connection` 服务时走它自己的准入判断。
文件名与路径在进入远程 shell 之前，只经过 `src/files.js` 的 `quote()` 一个收口点。

**浮窗的电量从哪来（以及为什么敢显示"没有"）**：电量是全插件唯一一个**藏在 ROS 后面**的读数——
只有底盘节点 `dzactuator` 通过串口读 VCU、发布 `/Battery_Percentage`（0..1）与 `/PowerVoltage`，
sysfs/systemd 里都没有它。所以 `/metrics` 是一条 SSH 远程命令：两帧 `/proc/stat`（隔 1 秒的增量 =
CPU 占用）+ `/proc/uptime`（`btime` 一并带回做"开机于"）+ 在 `bash -c` 里并行
`ros2 topic echo --once` 拉两个电量话题（`timeout 6`；在 `sh`/dash 里 source ROS 会直接中止
shell，必须 `bash -c`——实测，不是推断）。**栈没跑时话题不存在、回读为空**，此时返回
`battery=null, batteryNote=no_stack`，浮窗显示"—"、原因进 tooltip，**绝不编一个 0 %**；
只有电压没有百分比时按 12 V满 / 10 V空 折算并标注 `derived`。解析全在 `src/metrics.js`，
`DZH_BEGIN/END` 横幅把 MOTD/bashrc 噪音挡在门外；`test-host.mjs` 用合成载荷钉死每条分支。

**浮窗挂在哪个座位**：`shell.overlay`——全界面浮层，不在任何一列的滚动容器里，切视图不会把它
滚走；开/关是 store 里一个 `floatOpen`（面板第 4 行的按钮与卡片上的 × 写同一根线），**不需要
Session**，应用刚启动也能用。透明度是持久化设置（30–100，滑杆防抖 POST `/config`），值走
`--dzag-op` 自定义属性而不是内联 `opacity`——否则调暗的卡片会把自己的控件也一起锁住，
"悬停恢复全透明"就做不到了。曲线的几何（x=now±30 s 每 5 s 一刻度、y=当前值 ±10 点、
越界夹取 0..100）全在纯函数 `chartModel` 里，测试直接断言数值。

## 安装位置

尚未挂载到 `~/.dsh/profiles/desktop`（2026-10-22 实测：profile 的 dependencies 与 bundles 均无本包；此前此处写「已注册」有误，特此更正）：

- 将来挂载时，profile 应以 `file:D:/…/tools/plugins/dsh-dzairground-ssh` 依赖本目录，
  安装时会被 **打包成一份副本**（不是 junction）。**真正在跑的是这份副本。**
- `package.json` 的 `dsh.profile.bundles` 里加了 `dsh-dzairground-ssh`
- 本包的 `cordis.patch.yml` 负责挂载 Host 半身；浏览器半身由 `package.json` 的 `dsh.client` 声明

> ⚠️ 这里曾经坑掉一整轮排查。README 原来写的是「junction，改这里的代码立刻生效」，但实际是
> **打包副本**：源码树改完、`POST /reload` 也回了 `ok`，浏览器加载的却仍是旧副本。而打包遵循
> `package.json#files`，当时 `novnc/` 和 `vnc-app.html` 都没被列进去——安装后**整个 VNC 资源全部 404**。
> 更糟的是所有从源码树启动的测试（`test-host.mjs` 等）都是绿的，因为它们测的是源码树，
> 不是浏览器真正加载的那一份。`scripts/test-package-files.mjs` 就是为堵这个盲区而加的。

## 改代码后怎么生效

先部署，再让它生效：

```bash
node scripts/deploy.mjs            # 把 package.json#files 声明的文件同步进 profile 安装位置
node scripts/deploy.mjs --dry-run  # 只看会改什么，不动文件
```

| 改的是 | 生效方式 |
| --- | --- |
| 任何文件 | 先 `node scripts/deploy.mjs` |
| `client.js` / `vnc-app.html` / `novnc/*`（浏览器半身） | 再刷新页面 |
| `index.js` / `src/*`（Host 半身） | 再 `curl -X POST http://127.0.0.1:19387/api/dzairground-ssh/reload` |

Host 半身是 Node 的 ES 模块，进程按 URL 缓存，**改文件不会自动生效**；而 `POST /reload` 是从
**安装位置**重新 import 的，所以它只会忠实地重载那份副本——不先部署，reload 回 `ok` 也等于没改。
部署完用 `node scripts/test-package-files.mjs` 确认安装位置与源码树逐字节一致。

`src/reload.js` 负责把本包从 loader 的模块缓存里逐出、重新 import、再注册到同一个 fiber 上，
失败会回滚到旧版本。也可以在该插件条目的 config 里写 `watchSources: true`，改文件自动重载。

> 新增 `src/*.js` 或改 `package.json` 的 exports 之后，第一次仍需重启应用一次
> （模块解析本身也被缓存）；之后普通改动都是 `deploy` + `POST /reload` 秒级生效。

## 配置

设置页（Settings → 小车 SSH）：主机/IP、用户名、端口、`ssh` 别名（填了优先用别名）、
附带的 `ssh` 参数，以及 VNC 主机/端口/通道/密码、文件浏览起始目录。
存在 `~/.dsh/dzairground-ssh/settings.json`。

默认值取自这台机器上已有的 `~/.ssh/config`：`duzhong.local` / `duzhong` / 22 / `~/.ssh/id_rsa` 免密；
VNC 默认直连 5901（TigerVNC 在显示 `:1`）。

### 小车侧的 VNC 现状（本次已确认/已改）

- `vncserver@:1.service` 常驻，`Xtigervnc :1` 监听 **5901**（不是 5900），认证 `VncAuth,TLSVnc`。
- **分辨率当前是 1080p**（1920x1080，2026-10-05 应使用者要求从 720p 改回）：
  `/etc/systemd/system/vncserver@:1.service` 的 `-geometry` 与 `~/.vnc/config` 的 `geometry=`
  都是 `1920x1080`，走 systemd 重启生效、重启机器后仍有效。小车侧脚本是
  `scripts/restart-vnc-1080p.sh`（先改两处配置再 `systemctl restart`，避免互相盖回去）。
  > 注意：重启会把 `:1` 上所有 X 客户端杀掉——本次 det_node（含其 `ros2 launch` 父进程）
  > 就是这么没的，需要在小车侧重新拉起。
- **`-localhost no` 一直就是开的**（`localhost=no`），ufw 里 5901 规则也早已存在；
  ufw 服务本身处于"不活动"（不过滤流量），所以这两项实测都是多余操作。
- **VNC 密码是单独的**：`duzhong` 是 Linux 登录（也是 sudo）密码，已实测可用；
  但 **VNC 密码文件里存的不是它**。我没有采用它作为默认值，面板连接时会弹一次密码框。
  想免输入，两种办法：
  1. 在设置页把 VNC 密码填上（此后自动登录）；或
  2. 在小车上 `vncpasswd` 重设一个你记得住的（重启 VNC 后生效）。

- **TigerVNC 黑名单（实测，值得知道）**：`UseBlacklist` 默认为开，**一个连上却没有完成认证就断开的客户端会被拉黑**
  —— 小车上 `~/.vnc/duzhong:5901.log` 里就是这么记的：
  `Connections: accepted: …` → `Clean disconnection` → `Connections: blacklisted: …`。
  也就是说，**输错密码、或者连上后没登录就关掉，都会吃一次黑名单**；本机实测约 1 分钟就恢复
  （重复违规会翻倍）。因此：
  - 面板里的 VNC **不会自动连接**，必须点一下才连——避免打开面板就吃一次黑名单；
  - 被拉黑时面板会直接说明原因和等待时间，而不是报一个看不懂的错；
  - 如果想彻底避免，可以在小车的 VNC 单元上加 `-BlacklistThreshold=0` 或 `-UseBlacklist=0`（这是降低安全性，需你决定）。
  - **新加的横幅闸门本身也是一次"连上就断开"**：校验不通过时它会断开对方，而如果那个地址**就是**小车，
    TigerVNC 同样会记一次失败。所以闸门只在「地址来自缓存」或「配置写的是裸 IP」时才运行，
    并且判定不是 VNC 就**立即删除缓存条目**——下一次会重新走 DNS 解析，不会拿同一个可疑地址反复去撞。
    代价是每次「地址被占」的判定至多吃一次黑名单（约 1 分钟），换来的是零字节泄漏；
    反过来，等 mDNS 恢复、车真的回来了，下一次连接照常成功（这正是期望行为）。

## 通用 VNC 连接器（即「VNC 桌面」）

**没有独立的侧栏图标**：连接器就是小车面板工具列表里的「VNC 桌面」那一行（右侧栏标签页）。
一个 VNC 表面同时服务小车和**任意** TigerVNC + websockify 端点：

```
VNC 桌面标签页 ──iframe──▶ /vnc-connect.html ──noVNC──▶ ws://host:port/websockify ──▶ 目标机 TigerVNC :1
```

- **首次使用自动预填小车网桥**：`ws://<DSH 服务地址>/api/dzairground-ssh/vnc`（插件自己的
  WebSocket 升级，直通小车 5901，无需在小车上装 websockify）；只要改过任一字段，你的值
  就持久化并优先于预填。想连别的机器，直接改主机/端口/路径即可。

- **输入框**：协议（ws/wss）、VNC 主机、WebSocket 端口、路径（默认 `websockify`）、
  VNC 密码、分辨率；配置存在浏览器 localStorage（密码也在其中——与小车面板存密码到
  settings.json 同一量级，但**永不出现在 URL 里**）。
- **自适应缩放**：开关控制 noVNC `scaleViewport`；开=画面始终贴合窗口，关=原始分辨率可滚动。
- **分辨率选择**：「自适应」让远端跟随窗口（`resizeSession`）；选固定值则请求远端改分辨率
  ——实现是利用 noVNC 的 `SetDesktopSize` 扩展（TigerVNC + RandR 支持）：把画面容器临时
  设为 WxH、触发 resize 事件发出请求、再恢复贴合窗口。不支持该扩展的服务器会静默忽略。
- **全屏 / 断开 / 重连**：面板按钮。全屏是把整个 iframe 元素全屏（面板里的直接用户手势，
  iframe 带 `allow="fullscreen"`）；断开是显式动作，不会报成"掉线"。
- **嵌入方式**（面板头部有运行时徽标）：
  - **Electron / Tauri**：面板内嵌 iframe（跑在应用自身的 WebView 里），
    「新窗口」按钮 `window.open` 出一个真正的独立 WebView 窗口；
  - **浏览器插件**：同款 iframe，或「新标签页」打开独立文档。
  刻意**不用** Electron 的 `<webview>` 标签：它要求 webviewTag 开关，且访客→宿主通信走
  ipc 而不是 DOM postMessage，控制回路会一半静默失效。独立文档自带完整表单，
  非敏感字段（host/port/path/resolution/scheme）可由 URL 参数预填，密码在那边再问一次。

### Ubuntu 端：TigerVNC + websockify 完整命令（要求 6）

一键脚本：`scripts/ubuntu-vnc-websockify.sh`（在 Ubuntu 上 bash 执行），做的事：

```bash
sudo apt update && sudo apt install -y tigervnc-standalone-server tigervnc-common websockify
vncpasswd                                              # 设置 VNC 密码（交互式，与 Linux 密码是两回事）
# vncserver@:1.service：Xtigervnc :1 -geometry 1920x1080 -localhost no -SecurityTypes VncAuth
# websockify-5901.service：websockify 6080 127.0.0.1:5901
sudo systemctl enable --now vncserver@:1 websockify-5901
```

然后连接器填：`ws://<server-ip>:6080/websockify`。要点：
- **TigerVNC 新版默认只绑回环**，`-localhost no`（或至少让 websockify 能连到它）是必须的；
- `websockify` 是 WebSocket→TCP 代理，**路径参数默认被忽略**，所以 `/websockify` 只是
  noVNC 的惯例写法；`--web /usr/share/novnc` 只影响"别人用浏览器直接开 websockify 端口"，
  本插件自带 noVNC，可以不加；
- 密码只存 `~/.vnc/passwd`（哈希），插件侧填的就是 `vncpasswd` 设的那个。

### SSH 隧道安全连接（要求 7，推荐做法）

VNC 协议本身明文，密码握手里有质询-响应但也别裸奔。最省事又加密的方案是把
WebSocket 端点通过 SSH 隧道拉回本机——websockify 绑 127.0.0.1、防火墙一个口都不开：

```bash
# 1. Ubuntu 侧（脚本默认就这么配）：websockify 只绑 127.0.0.1，ufw 不放行 6080
# 2. 本机开隧道（Windows 自带 OpenSSH 客户端，PowerShell 同款命令）：
ssh -N -L 6080:127.0.0.1:6080 user@server
# 3. 连接器填：
#    主机 127.0.0.1  端口 6080  路径 websockify  →  ws://127.0.0.1:6080/websockify
```

- 整条链路变成 `浏览器 ──ws(本机)──▶ ssh 加密通道 ──▶ websockify ──▶ TigerVNC`，
  VNC 密码与桌面像素都只在加密通道里走；
- 隧道断了连接器会照常报"连接失败"（WebSocket 关闭码），重开隧道后点重连即可；
- 比 websockify 自带的 `--cert/--key` TLS 省事得多（那要自己管证书、且 `wss://` 证书
  域名必须与主机名匹配）；想用 wss 也支持——连接器协议下拉选 `wss` 即可。

## HTTP API

前缀 `/api/dzairground-ssh`，仅本机回环可访问。

| 方法 | 路径 | 作用 |
| --- | --- | --- |
| GET | `/status` | 配置、上次探测、小车信息、等价命令行、VNC 目标 |
| POST | `/probe[?car=0]` | 探测可达性与免密登录 |
| POST | `/describe` | 采集小车信息（用户/主机名/内核/运行时长） |
| GET/POST | `/config` | 读取 / 保存配置（含浮窗不透明度 `floatOpacity`，30–100 夹取） |
| GET | `/metrics` | 浮窗读数：电量 / CPU / 开机时间（永远 200，车连不上是 `ok:false` + 原因） |
| GET | `/float.html` | 状态浮窗独立文档（同一批数字，自己的刷新按钮） |
| GET | `/files?path=` | 列目录（NUL+base64，名字里带空格/引号也不怕） |
| GET | `/file?path=` | 读文本（二进制会被识别出来而不是渲染成乱码） |
| GET | `/stat?path=` | 取大小/类型/时间 |
| GET | `/download?path=` | 流式下载 |
| POST | `/upload?path=` | 流式上传 |
| GET | `/novnc/*` | 本插件自带的 noVNC 静态资源（原生 ES 模块） |
| GET | `/vnc.html` | 小车 VNC 屏幕文档（iframe 嵌入，postMessage 控制） |
| GET | `/vnc-connect.html` | **通用 VNC 连接器**文档（noVNC 直连 ws://host:port/path） |
| WS | `/vnc` | VNC 字节隧道（upgrade 座位） |
| GET/POST | `/reload` | 查看上次热重载 / 立即热重载 Host 半身 |
| GET/POST | `/diag` | 诊断日志（浏览器半身上报，落 `diag.jsonl`） |
| GET/POST | `/selftest` | 一次性自检开关 |

## 自检脚本

```bash
# 本机（Node）
node scripts/deploy.mjs         # 把改动同步进 profile 安装位置（改完代码第一步；--dry-run 可预览）
node scripts/check-ssh.mjs      # SSH 层：真实免密登录、描述
node scripts/check-files.mjs    # 文件层：列举/读取/二进制识别/奇怪文件名往返/符号链接解引用
node scripts/test-host.mjs      # Host 半身：全部路由 + 电量解析/浮窗端点 + noVNC 资源 + VNC 隧道（本机替身）
node scripts/test-client.mjs    # Client 半身：五个座位注册与渲染 + 浮窗结构契约 + 完整连接流程
node scripts/test-vnc-guard.mjs # 缓存地址闸门：非 VNC 对端被拒、零字节转发、失败即清缓存
node scripts/test-vnc-wrapper.mjs   # 浏览器侧 WebSocket 探针能过 noVNC 自己的通道检查（两份文档都验）
node scripts/test-vnc-connect.mjs   # 连接器文档逻辑无头验证：URL 构造/分辨率/缩放/密码事件/面板协议
node scripts/test-package-files.mjs # 安装位置是否齐全且与源码树一致（堵「测源码树不测产物」的盲区）
node scripts/test-live-ws.mjs   # 对着「运行中的插件 + 小车」打一发真实 VNC 横幅
node scripts/debug-ws-handshake.mjs # 用浏览器同款请求头打一次握手，打印服务端原始响应
node scripts/debug-files.mjs    # 目录列不出来时，直接打印远程命令的原始退出码/stderr

# 任意 Ubuntu 机器（TigerVNC + websockify，供「通用 VNC 连接器」使用）
scripts/ubuntu-vnc-websockify.sh    # 完整安装：包 + vncpasswd + systemd 单元 + 暴露方式（局域网 or SSH 隧道）

# 小车侧（scp 过去再 bash 执行；PowerShell 直接内联会被引号/CR 咬）
scripts/probe-car.sh            # 勘察：桌面/VNC/监听端口/SFTP
scripts/probe-vnc-config.sh     # 勘察：VNC 单元、geometry、:1 上的客户端
scripts/restart-vnc-1080p.sh    # 改 1080p + ufw + systemd 重启（杀 X 客户端，见上）
scripts/check-vnc-suggestion.sh # 核对 localhost/ufw/geometry/占用进程的实况
scripts/vnc-log-families.sh     # 按地址族统计 VNC 黑名单条目
```

`test-host.mjs` 的 VNC 部分是**对着本机的一个替身 TCP 服务**测的，不是对着小车：
按上面的黑名单机制，对着小车「连上读个横幅就挂断」本身就是一次安全失败，会把操作者自己的
笔记本拉黑几分钟——测试不该做这种事。替身反而测得更严：70300 字节的往返要求**逐字节相等**，
并且覆盖 WebSocket 三种帧长编码（7 位 / 16 位 / 64 位）。

想真的对着小车跑一次，就显式打开（跑一次就够，别连跑）：

```bash
DZR_TEST_CAR_VNC=1 node scripts/test-host.mjs
```

测试都用临时 `DSH_HOME`，**不会**动你真实的 `settings.json`。

## 关于自带的 noVNC

`novnc/` 是从 npm 上 `@novnc/novnc`（1.7.0）**原样**拷来的 `core/` 与 `vendor/`，未做任何修改，
因为浏览器半身是把它当原生 ES 模块直接 `import()` 的（56 个文件的依赖图由浏览器自己解析，
所以不需要打包步骤）。许可证 MPL-2.0，见 `novnc/LICENSE.txt`，升级时整目录替换即可。

## 排错

- **侧栏图标是红点**：小车不可达。确认它上电、连上同一 WiFi（本机与它同在 `192.168.43.x`），
  `ping duzhong.local`。
- **黄点 / "需要密码"**：能连上但免密不成立，把公钥装到小车：
  `ssh-copy-id -i ~/.ssh/id_rsa.pub duzhong@duzhong.local`。
- **VNC 连不上**：先确认车上的 `vncserver@:1.service` 在跑（`systemctl status vncserver@:1`），
  端口是 **5901**（不是 5900）。若 VNC 只监听本地，把设置里的通道改成「走 SSH 隧道」。
- **VNC 报「不是这台车的 VNC 服务（缓存已失效）」**：车换网络了，而旧地址已被别的设备占用。
  缓存条目已被自动删除，等 mDNS 解析恢复后重新点一次即可。
- **「连接异常中断」其实是车掉线**：面板显示的是桥自己给的关闭原因。
  `无法连接 …（N 个地址均失败）：connect timeout` = 车不可达（此时 SSH 终端同样连不上）；
  `WebSocket code N` 且没有原因文本 = 看 `diag.jsonl` 里同一条目的 `ws` 数组。
- **面板点了没反应**：看 `~/.dsh/dzairground-ssh/diag.jsonl`，每一步都有记录；
  也可以在设置页复制等价命令，直接在终端里粘贴。
