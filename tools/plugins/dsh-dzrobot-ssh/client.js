/**
 * dzrobot-ssh — browser half.
 *
 * Contributes a small tool launcher and the tools it opens:
 *   1. `sidebar.panellist` — the car icon in the sidebar rail.
 *   2. `main` — the panel it opens: a tool list. Each row opens its tool as a
 *      tab in the right column — terminal (the stock kind), the car's file
 *      tree, and the VNC desktop — so adding a tool later is one row here plus
 *      one tab-type registration below.
 *   3. `sidebar.right.pane.tab` — bodies for the two plugin-owned tab types
 *      (`dsh-dzrobot-ssh/vnc`, `dsh-dzrobot-ssh/files`).
 *   4. `settings.section` — the car's address, the probe result, and the exact
 *      command line, so the integration can be checked without a terminal.
 *   5. `shell.overlay` — the status float: battery and CPU usage as two
 *      blue-boxed cells with live time/value curves, draggable, refreshed every
 *      5 seconds (the battery comes from the chassis node over ROS; see
 *      `src/metrics.js` for why it can honestly say "no stack").
 *
 * The right column belongs to the Conversation (the shell clears its on-screen
 * Session while any other main panel is active), so opening a tool switches the
 * main view there first — `ensureConversation` is that switch, shared by every
 * tool.
 *
 * The interactive session is *the app's own terminal*, not a reimplementation:
 * the terminal tool opens a terminal tab through the sidebar service and types
 * the command line the Host composed into the terminal's view. That way echo,
 * Tab completion, Ctrl-C, resize and scrollback are the shipped ones.
 *
 * Hand-written ModuleLoader bundle: no build step, no dependency beyond the
 * `react` the shell already provides.
 */

window.__ModuleLoader__.load({
  id: 'dsh-dzrobot-ssh',
  factory: require => {
    const module = { exports: {} }
    const exports = module.exports
    const React = require('react')
    const { createElement: h, useState, useEffect, useRef, useCallback } = React

    const API = '/api/dzrobot-ssh'
    const NS = 'dzrobotSsh'
    /** One id in both seats: the panellist icon and the main panel it selects. */
    const PANEL_ID = 'dzrobot-car'

    const inject = ['slots', 'locale', 'sidebarRight', 'webTerminals', 'sidebarRightTabs']

    // ── copy ──────────────────────────────────────────────────────────────────
    const DICT = {
      zh: {
        nav: '小车 SSH',
        title: '小车 SSH',
        subtitle: '一键免密登录 RK3588 小车',
        connecting: '连接中…',
        connectHint: '在终端里登录小车',
        openTerminal: '打开 SSH 终端',
        toolOpen: '打开',
        toolNote: '点“打开”后主视图会切到会话，工具窗口显示在会话右侧——右栏属于会话，这是 DSH 的结构。',
        filesToolDesc: '浏览、预览、下载、上传小车上的文件（走同一条已信任的 SSH 通道）',
        vncToolDesc: '小车的 X 桌面（TigerVNC :1，1920×1080）；需要时会在画面里向你要 VNC 密码',
        simToolDesc: '本地跑 dzbrain 离线仿真：速度档+误差注入一键出指标与路线图，历史对比，不连小车',
        copy: '复制命令',
        copied: '已复制',
        retry: '重试',
        probe: '测试连通性',
        probing: '测试中…',
        save: '保存',
        saved: '已保存',
        saveFailed: '保存失败',
        noSession: '请先打开一个会话，再点侧栏的小车图标',
        terminalFailed: '终端未就绪：{message}',
        notReady: '小车不可达：{detail}',
        needPassword: '能连上，但需要密码或密钥。先在终端里手动 ssh 一次，或把公钥装到小车。',
        sectionAddress: '小车地址',
        sectionState: '连通性',
        sectionCommand: '等价命令',
        sectionAdvanced: '高级',
        fieldHost: '主机 / IP',
        fieldUser: '用户名',
        fieldPort: '端口',
        fieldAlias: 'SSH 别名（优先，可留空）',
        fieldOptions: 'ssh 附加参数',
        fieldVncHost: 'VNC 主机（空=同主机）',
        fieldVncPort: 'VNC 端口',
        fieldVncMode: 'VNC 通道',
        vncModeDirect: '直连 VNC 端口',
        vncModeTunnel: '走 SSH 隧道',
        vncModeNote: '小车当前是 TigerVNC :1 即 5901。若 VNC 只监听本地，选隧道；隧道复用上面的 SSH 地址。',
        carUser: '登录用户',
        carHost: '主机名',
        carKernel: '内核',
        carUp: '已运行',
        stateReady: '可免密登录',
        stateUnreachable: '不可达',
        stateNeedsPassword: '需要密码',
        stateHostKey: '主机密钥不符',
        stateError: '出错',
        latency: '耗时 {ms}ms',
        neverProbed: '尚未测试',
        diag: '记录诊断',
        // panel
        panel: '小车',
        viewVnc: 'VNC 桌面',
        viewTerminal: '终端',
        viewFiles: '文件目录',
        viewSim: '仿真台',
        vncConnect: '连接桌面',
        vncReconnect: '重新连接',
        vncDisconnect: '断开',
        vncConnecting: '正在连接桌面…',
        vncConnected: '桌面已连接',
        vncDisconnected: '桌面已断开',
        vncIdle: '未连接',
        vncIdleTitle: '小车桌面',
        vncIdleHint: '点击“连接桌面”开始',
        vncWaitingPassword: '等待输入密码…',
        vncFailed: '连接失败',
        vncPassword: 'VNC 密码',
        vncPasswordHint: '留空则连接时由 noVNC 弹窗询问',
        vncNeedPassword: '这台车的 VNC 需要密码（与 Linux 登录密码是两回事）',
        vncSubmitPassword: '登录',
        vncBlacklisted: '小车暂时拒绝本机了：连续认证失败会被 TigerVNC 列入黑名单（默认 10 分钟），等一会儿再连。密码没输错的话，先在设置页把正确的 VNC 密码填上。',
        vncNoOrigin: '没能从插件后端取到服务地址，无法加载 VNC 画面',
        simOpenHint: '没能自动打开仿真台（弹窗被拦截，或还没取到服务地址）。手动地址：{url}',
        vncDropped: '连接异常中断——小车可能掉线，或主机名解析抖动（这台车的 mDNS 时好时坏），稍后重试',
        vncSendCad: 'Ctrl+Alt+Del',
        terminalHint: 'SSH 跑在 DSH 自带终端里（回显/补全/缩放都是原生的）。点击后会切到会话视图——终端在会话右侧，这是 DSH 的结构。',
        filesPath: '路径',
        filesEmpty: '（空目录）',
        filesLoading: '正在读取…',
        filesUp: '上一级',
        filesRefresh: '刷新',
        filesDownload: '下载',
        filesUpload: '上传',
        filesPreview: '预览',
        filesClose: '关闭',
        filesTooLarge: '文件过大，请下载查看',
        filesBinary: '二进制文件，不能作为文本预览',
        filesName: '名称',
        filesSize: '大小',
        filesMtime: '修改时间',
        filesRoot: '根目录',
        fieldFilesRoot: '文件浏览起始目录',
        openTerminalFirst: '先打开一个会话，再点这里',
        // status float
        viewFloat: '状态浮窗',
        floatToolDesc: '悬浮显示小车的电量与 CPU 占用——两列蓝框卡片带时间/数值曲线，可拖动，5 秒一轮自动刷新',
        toolClose: '收起',
        floatTitle: '小车状态',
        floatBattery: '电量',
        floatCpu: 'CPU 占用',
        floatUp: '开机时间',
        floatDown: '小车未连接',
        floatLoad: '负载 {load}',
        floatBattVolts: '{v} V · 底盘发布',
        floatBattDerived: '{v} V · 按 12V满/10V空 折算',
        floatBattNone: '底盘未发布电量（ROS 栈未运行）',
        floatSince: '开机于 {time}',
        floatRefresh: '刷新',
        floatClose: '关闭',
        fieldFloatOpacity: '浮窗不透明度',
        opacityHint: '数值越低越透明；鼠标悬停时浮窗恢复完整不透明，方便操作。',
        floatLoading: '正在读取…',
        floatError: '读取失败：{message}',
        floatSync: '{time} 更新 · 5 秒一轮',
        floatDrag: '按住标题栏可拖动',
        floatAxisX: '时间',
        unitDay: '天',
        unitHour: '小时',
        unitMin: '分',
        unitSec: '秒',
        // generic VNC connector
        vncConnectorNav: 'VNC 连接器',
        vncConnectorTitle: 'VNC 连接器',
        vncConnectorSub: 'noVNC 直连任意 ws://host:port/websockify（TigerVNC + websockify）',
        connFieldScheme: '协议',
        connFieldHost: 'VNC 主机',
        connFieldPort: 'WebSocket 端口',
        connFieldPath: '路径',
        connFieldPassword: 'VNC 密码',
        connFieldResolution: '分辨率',
        connResolutionAuto: '自适应（跟随窗口）',
        connConnect: '连接',
        connReconnect: '重新连接',
        connDisconnect: '断开',
        connFullscreen: '全屏',
        connScale: '自适应缩放',
        connNewTab: '新标签页',
        connNewWindow: '新窗口',
        connRuntimeElectron: 'Electron（WebView）',
        connRuntimeTauri: 'Tauri（WebView）',
        connRuntimeBrowser: '浏览器插件',
        connConnecting: '连接中…',
        connConnected: '桌面已连接',
        connDisconnected: '桌面已断开',
        connFailed: '连接失败',
        connIdle: '未连接',
        connWaitingPassword: '等待密码…',
        connIdleTitle: 'VNC 桌面',
        connIdleHint: '填好地址后点“连接”。Ubuntu 侧跑 TigerVNC + websockify，安全起见可用 SSH 隧道（见设置说明）',
        connNeedPassword: '服务器要求 VNC 密码（与 Linux 登录密码是两回事），请在上方填写后重连',
        connBlacklisted: '目标服务器暂时拒绝本机：连续认证失败会被 TigerVNC 列入黑名单（默认 10 分钟），稍后再试',
        connFullscreenFailed: '全屏被浏览器拒绝',
        connNewTabHint: '新窗口/标签页是独立文档，密码不会带过去，会在那里再问一次',
        connScaleHint: '开启时画面始终缩放到窗口大小；关闭时按原始分辨率显示、可滚动',
        connResolutionHint: '自适应让远端分辨率跟随窗口；选固定值则请求远端改分辨率（需 TigerVNC + RandR 支持 SetDesktopSize）',
        connCarBridge: '小车通道',
        connCarBridgeHint: '把目标切回插件自带网桥 ws://<DSH 地址>/api/dzrobot-ssh/vnc：直通小车 5901，车上不需要 websockify',
        connCarBridgeDone: '已切到小车通道，点“连接”即可',
        connHintNoListener: '对端没有响应（WebSocket code 1006）。多数是这三种：对端没跑 websockify、端口/路径写错、协议选错（明文端口要用 ws，选 wss 会握手失败）。目标是小车的话，直接用“小车通道”，不需要 websockify。',
        connHintBridgeFailed: '网桥没连上。请确认小车在线、且 TigerVNC 正在 5901 监听（插件设置里有探测按钮）。',
      },
      en: {
        nav: 'Car SSH',
        title: 'Car SSH',
        subtitle: 'One-click login to the RK3588 car',
        connecting: 'Connecting…',
        connectHint: 'Log in to the car in a terminal',
        openTerminal: 'Open SSH terminal',
        toolOpen: 'Open',
        toolNote: 'Opening a tool switches the main view to the Conversation — the tool window appears on its right. The right column belongs to the Conversation; that is DSH’s structure.',
        filesToolDesc: 'Browse, preview, download and upload the car’s files (over the same trusted SSH channel)',
        vncToolDesc: 'The car’s X desktop (TigerVNC :1, 1920×1080); it may ask for the VNC password in the screen',
        simToolDesc: 'Run the dzbrain offline simulator locally: a speed step plus error injection gives metrics and the route plot in one click, with a history to compare — no car involved',
        copy: 'Copy command',
        copied: 'Copied',
        retry: 'Retry',
        probe: 'Test reachability',
        probing: 'Testing…',
        save: 'Save',
        saved: 'Saved',
        saveFailed: 'Save failed',
        noSession: 'Open a Session first, then click the car icon in the sidebar',
        terminalFailed: 'Terminal not ready: {message}',
        notReady: 'Car unreachable: {detail}',
        needPassword: 'Reachable, but it wants a password or key. Run ssh once by hand, or install your key on the car.',
        sectionAddress: 'Car address',
        sectionState: 'Reachability',
        sectionCommand: 'Equivalent command',
        sectionAdvanced: 'Advanced',
        fieldHost: 'Host / IP',
        fieldUser: 'User',
        fieldPort: 'Port',
        fieldAlias: 'SSH alias (takes priority, optional)',
        fieldOptions: 'Extra ssh options',
        fieldVncHost: 'VNC host (empty = same host)',
        fieldVncPort: 'VNC port',
        fieldVncMode: 'VNC transport',
        vncModeDirect: 'Connect the VNC port directly',
        vncModeTunnel: 'Through an SSH tunnel',
        vncModeNote: 'The car runs TigerVNC on :1, i.e. 5901. Pick the tunnel when the VNC server binds loopback only; it reuses the SSH address above.',
        carUser: 'Login user',
        carHost: 'Hostname',
        carKernel: 'Kernel',
        carUp: 'Uptime',
        stateReady: 'Passwordless login works',
        stateUnreachable: 'Unreachable',
        stateNeedsPassword: 'Password required',
        stateHostKey: 'Host key mismatch',
        stateError: 'Error',
        latency: 'took {ms}ms',
        neverProbed: 'Not tested yet',
        diag: 'Send diagnostics',
        // panel
        panel: 'Car',
        viewVnc: 'VNC desktop',
        viewTerminal: 'Terminal',
        viewFiles: 'Files',
        viewSim: 'Sim bench',
        vncConnect: 'Connect desktop',
        vncReconnect: 'Reconnect',
        vncDisconnect: 'Disconnect',
        vncConnecting: 'Connecting to the desktop…',
        vncConnected: 'Desktop connected',
        vncDisconnected: 'Desktop disconnected',
        vncIdle: 'Not connected',
        vncIdleTitle: 'Car desktop',
        vncIdleHint: 'Press “Connect desktop” to start',
        vncWaitingPassword: 'Waiting for password…',
        vncFailed: 'Connection failed',
        vncPassword: 'VNC password',
        vncPasswordHint: 'Leave empty and the panel asks on connect',
        vncNeedPassword: 'This car’s VNC wants a password (separate from the Linux login password)',
        vncSubmitPassword: 'Sign in',
        vncBlacklisted: 'The car is refusing this machine: repeated failed logins put it on TigerVNC’s blacklist (10 minutes by default). Wait a moment and retry — or set the correct VNC password in Settings first.',
        vncNoOrigin: 'Could not learn the server address from the plugin backend, so the VNC screen cannot load',
        simOpenHint: 'The sim bench could not be opened automatically (popup blocked, or the server address is unknown). Open it manually: {url}',
        vncDropped: 'Connection dropped unexpectedly — the car may be offline, or its mDNS name is flapping (this hotspot is unreliable). Retry shortly.',
        vncSendCad: 'Ctrl+Alt+Del',
        terminalHint: 'SSH runs in the app’s own terminal (native echo, completion, resizing). Clicking switches to the Conversation — the terminal lives in its right column; that is DSH’s structure.',
        filesPath: 'Path',
        filesEmpty: '(empty directory)',
        filesLoading: 'Loading…',
        filesUp: 'Up',
        filesRefresh: 'Refresh',
        filesDownload: 'Download',
        filesUpload: 'Upload',
        filesPreview: 'Preview',
        filesClose: 'Close',
        filesTooLarge: 'Too large to preview — download it instead',
        filesBinary: 'Binary file — not shown as text',
        filesName: 'Name',
        filesSize: 'Size',
        filesMtime: 'Modified',
        filesRoot: 'Root',
        fieldFilesRoot: 'File browser starts at',
        openTerminalFirst: 'Open a Session first, then click here',
        // status float
        viewFloat: 'Status float',
        floatToolDesc: 'A floating gauge for the car’s battery and CPU usage — two blue-boxed charts with time/value axes, draggable, refreshed every 5 seconds',
        toolClose: 'Close',
        floatTitle: 'Car status',
        floatBattery: 'Battery',
        floatCpu: 'CPU usage',
        floatUp: 'Uptime',
        floatDown: 'Car offline',
        floatLoad: 'load {load}',
        floatBattVolts: '{v} V · chassis topic',
        floatBattDerived: '{v} V · from the 12 V-full / 10 V-empty calibration',
        floatBattNone: 'Battery not published (ROS stack is down)',
        floatSince: 'booted {time}',
        floatRefresh: 'Refresh',
        floatClose: 'Close',
        fieldFloatOpacity: 'Float opacity',
        opacityHint: 'Lower is more transparent; hovering the card restores full opacity so its controls stay usable.',
        floatLoading: 'Loading…',
        floatError: 'Reading failed: {message}',
        floatSync: 'updated {time} · every 5s',
        floatDrag: 'Drag the title bar to move',
        floatAxisX: 'Time',
        unitDay: 'd',
        unitHour: 'h',
        unitMin: 'm',
        unitSec: 's',
        // generic VNC connector
        vncConnectorNav: 'VNC Connector',
        vncConnectorTitle: 'VNC Connector',
        vncConnectorSub: 'noVNC dialing any ws://host:port/websockify (TigerVNC + websockify)',
        connFieldScheme: 'Scheme',
        connFieldHost: 'VNC host',
        connFieldPort: 'WebSocket port',
        connFieldPath: 'Path',
        connFieldPassword: 'VNC password',
        connFieldResolution: 'Resolution',
        connResolutionAuto: 'Auto (follow window)',
        connConnect: 'Connect',
        connReconnect: 'Reconnect',
        connDisconnect: 'Disconnect',
        connFullscreen: 'Fullscreen',
        connScale: 'Fit to window',
        connNewTab: 'New tab',
        connNewWindow: 'New window',
        connRuntimeElectron: 'Electron (WebView)',
        connRuntimeTauri: 'Tauri (WebView)',
        connRuntimeBrowser: 'Browser plugin',
        connConnecting: 'Connecting…',
        connConnected: 'Desktop connected',
        connDisconnected: 'Desktop disconnected',
        connFailed: 'Connection failed',
        connIdle: 'Not connected',
        connWaitingPassword: 'Waiting for password…',
        connIdleTitle: 'VNC desktop',
        connIdleHint: 'Fill in the address and press Connect. Ubuntu side runs TigerVNC + websockify; prefer an SSH tunnel for security (see settings notes)',
        connNeedPassword: 'The server wants a VNC password (separate from the Linux login password). Fill it in above and reconnect.',
        connBlacklisted: 'The target server is refusing this machine: repeated failed logins put it on TigerVNC’s blacklist (10 minutes by default). Wait, then retry.',
        connFullscreenFailed: 'The browser refused fullscreen',
        connNewTabHint: 'The new window/tab is a standalone document; the password is not transferred — it asks again there',
        connScaleHint: 'On: the picture always scales to fit the window. Off: shown at native resolution, scrollable',
        connResolutionHint: 'Auto makes the remote follow your window; a fixed value requests a remote resolution change (needs TigerVNC + RandR / SetDesktopSize)',
        connCarBridge: 'Car bridge',
        connCarBridgeHint: 'Point the form back at the plugin’s own bridge ws://<DSH address>/api/dzrobot-ssh/vnc — straight to the car on 5901, no websockify needed',
        connCarBridgeDone: 'Switched to the car bridge — press Connect',
        connHintNoListener: 'The peer did not answer (WebSocket code 1006). Usually one of three: nothing is listening (websockify not running), wrong port/path, or the wrong scheme (plain ports need ws; wss fails the TLS handshake). If the target is the car, use “Car bridge” — it needs no websockify.',
        connHintBridgeFailed: 'The bridge did not connect. Check that the car is online and TigerVNC is listening on 5901 (the plugin settings have a probe button).',
      },
    }

    const STATE_KEY = {
      ready: 'stateReady',
      unreachable: 'stateUnreachable',
      'needs-password': 'stateNeedsPassword',
      'host-key': 'stateHostKey',
      error: 'stateError',
    }

    // ── styles ────────────────────────────────────────────────────────────────
    const CSS = `
.dzr_icon{display:inline-flex;position:relative;flex:none;line-height:0}
.dzr_dot{position:absolute;right:-2px;bottom:-2px;width:7px;height:7px;border-radius:50%;background:var(--dsw-alias-label-tertiary);box-shadow:0 0 0 2px var(--dsw-alias-bg-layer-2,transparent)}
.dzr_dot[data-state="ready"]{background:var(--dsw-alias-state-success-primary)}
.dzr_dot[data-state="unreachable"]{background:var(--dsw-alias-state-error-primary)}
.dzr_dot[data-state="needs-password"],.dzr_dot[data-state="host-key"]{background:var(--dsw-alias-state-warning-primary)}
.dzr_root{display:flex;flex-direction:column;gap:14px;max-width:820px;font-size:13px;line-height:1.6;color:var(--dsw-alias-label-primary)}
.dzr_root *{box-sizing:border-box}
.dzr_card{padding:14px 16px;border-radius:14px;border:1px solid var(--dsw-alias-border-l2);background:var(--dsw-alias-bg-layer-2);display:flex;flex-direction:column;gap:10px}
.dzr_head{display:flex;align-items:center;gap:10px;flex-wrap:wrap}
.dzr_h{font-size:13px;font-weight:650}
.dzr_sub{font-size:11.5px;color:var(--dsw-alias-label-tertiary)}
.dzr_badge{margin-left:auto;font-size:11px;padding:2px 8px;border-radius:999px;border:1px solid var(--dsw-alias-border-l1);color:var(--dsw-alias-label-secondary);white-space:nowrap}
.dzr_badge[data-state="ready"]{color:var(--dsw-alias-state-success-primary);border-color:var(--dsw-alias-state-success-primary)}
.dzr_badge[data-state="unreachable"]{color:var(--dsw-alias-state-error-primary);border-color:var(--dsw-alias-state-error-primary)}
.dzr_badge[data-state="needs-password"],.dzr_badge[data-state="host-key"]{color:var(--dsw-alias-state-warning-primary);border-color:var(--dsw-alias-state-warning-primary)}
.dzr_grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:10px}
.dzr_kv{padding:9px 11px;border-radius:11px;border:1px solid var(--dsw-alias-border-l1);background:var(--dsw-alias-bg-layer-1);display:flex;flex-direction:column;gap:2px;min-width:0}
.dzr_kv b{font-size:12.5px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.dzr_kv span{font-size:10.5px;color:var(--dsw-alias-label-tertiary)}
.dzr_field{display:flex;flex-direction:column;gap:4px;min-width:0}
.dzr_field>span{font-size:11px;color:var(--dsw-alias-label-tertiary)}
.dzr_input{font:inherit;font-size:12px;padding:5px 9px;border-radius:9px;border:1px solid var(--dsw-alias-border-l2);background:var(--dsw-alias-bg-layer-1);color:var(--dsw-alias-label-primary);width:100%;min-width:0}
.dzr_input:focus{outline:none;border-color:var(--dsw-alias-state-business-primary)}
.dzr_row{display:flex;gap:8px;align-items:center;flex-wrap:wrap}
.dzr_actions{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
.dzr_action{font:inherit;font-size:12px;padding:5px 12px;border-radius:9px;border:1px solid var(--dsw-alias-border-l2);background:var(--dsw-alias-bg-layer-3);color:var(--dsw-alias-label-primary);cursor:pointer}
.dzr_action:hover:not(:disabled){border-color:var(--dsw-alias-state-business-primary)}
.dzr_action:disabled{opacity:.5;cursor:default}
.dzr_action.primary{background:var(--dsw-alias-state-business-primary);border-color:var(--dsw-alias-state-business-primary);color:var(--dsw-alias-label-on-accent)}
.dzr_mono{font-family:ui-monospace,SFMono-Regular,Menlo,monospace;font-size:11.5px;padding:8px 10px;border-radius:9px;background:var(--dsw-alias-bg-layer-1);border:1px solid var(--dsw-alias-border-l1);overflow-x:auto;white-space:pre;color:var(--dsw-alias-label-secondary)}
.dzr_note{font-size:11.5px;line-height:1.55;color:var(--dsw-alias-label-tertiary)}
.dzr_err{border-color:var(--dsw-alias-state-error-primary);color:var(--dsw-alias-state-error-primary)}
.dzr_warn{border-color:var(--dsw-alias-state-warning-primary);color:var(--dsw-alias-state-warning-primary)}
/* ── the car panel (main column) ───────────────────────────────────────────── */
.dzr_panel{display:flex;flex-direction:column;height:100%;min-height:0;font-size:13px;color:var(--dsw-alias-label-primary);box-sizing:border-box}
.dzr_panel *{box-sizing:border-box}
.dzr_phead{display:flex;align-items:center;gap:12px;padding:14px 18px;border-bottom:1px solid var(--dsw-alias-border-l1);flex:none;flex-wrap:wrap}
.dzr_ptitle{display:flex;align-items:center;gap:9px;font-size:14px;font-weight:650}
/* Tool list rows in the main panel: glyph, name/description, one action. */
.dzr_toolrow{display:flex;gap:14px;align-items:center;padding:13px 15px;border:1px solid var(--dsw-alias-border-l2);border-radius:13px;background:var(--dsw-alias-bg-layer-3);transition:border-color .15s ease}
.dzr_toolrow:hover{border-color:var(--dsw-alias-state-business-primary)}
.dzr_toolglyph{display:flex;align-items:center;justify-content:center;width:36px;height:36px;border-radius:10px;background:var(--dsw-alias-bg-layer-1);border:1px solid var(--dsw-alias-border-l1);color:var(--dsw-alias-state-business-primary);flex:none}
.dzr_toolinfo{flex:1;min-width:0;display:flex;flex-direction:column;gap:3px}
.dzr_tooltitle{font-size:13.5px;font-weight:650}
.dzr_tooldesc{font-size:11.5px;color:var(--dsw-alias-label-tertiary);overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
/* Tab bodies own no chrome of their own: the right column supplies it, so the
   only job is to fill the tab's height chain (screen flexes inside). */
.dzr_tabroot{display:flex;flex-direction:column;height:100%;min-height:0}
.dzr_pbody{flex:1;min-height:0;display:flex;flex-direction:column;position:relative}
.dzr_pad{padding:16px 18px;display:flex;flex-direction:column;gap:12px;overflow:auto}
.dzr_toolbar{display:flex;align-items:center;gap:8px;padding:8px 14px;border-bottom:1px solid var(--dsw-alias-border-l1);flex:none;flex-wrap:wrap;background:var(--dsw-alias-bg-layer-1)}
.dzr_screen{flex:1;min-height:0;background:#000;position:relative;overflow:hidden}
.dzr_screen canvas{display:block;max-width:100%;max-height:100%;margin:0 auto}
/* Centered overlay over the screen area: idle/failed states and the password
   card. It sits above the iframe only while the desktop itself is not on screen,
   so once connected nothing covers the pointer. */
.dzr_center{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;padding:24px;background:#000}
.dzr_herocard{display:flex;flex-direction:column;align-items:center;gap:10px;max-width:420px;padding:22px 26px;border-radius:14px;border:1px solid var(--dsw-alias-border-l2);background:var(--dsw-alias-bg-layer-2);color:var(--dsw-alias-label-primary);text-align:center;box-shadow:0 8px 32px rgb(0 0 0 / 25%)}
.dzr_herocard h3{margin:0;font-size:14px;font-weight:650}
.dzr_herocard p{margin:0;font-size:12px;color:var(--dsw-alias-label-tertiary);line-height:1.6}
.dzr_herocard input{width:220px;text-align:center}
/* Status pill: one dot + one line, tinted per phase — the whole reason the old
   free-floating status text looked broken across two rows. */
.dzr_pill{display:inline-flex;align-items:center;gap:7px;max-width:340px;padding:4px 11px;border-radius:999px;border:1px solid var(--dsw-alias-border-l1);background:var(--dsw-alias-bg-layer-1);font-size:11.5px;color:var(--dsw-alias-label-secondary);min-width:0}
.dzr_pill span{overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.dzr_pill i{width:7px;height:7px;border-radius:50%;background:var(--dsw-alias-label-tertiary);flex:none}
.dzr_pill[data-phase="connecting"] i,.dzr_pill[data-phase="auth"] i{background:var(--dsw-alias-state-warning-primary)}
.dzr_pill[data-phase="connected"] i{background:var(--dsw-alias-state-success-primary)}
.dzr_pill[data-phase="failed"] i{background:var(--dsw-alias-state-error-primary)}
.dzr_pill[data-phase="connected"]{border-color:var(--dsw-alias-state-success-primary)}
.dzr_pill[data-phase="failed"]{border-color:var(--dsw-alias-state-error-primary);color:var(--dsw-alias-state-error-primary)}
/* Inline activity spinner for the hero card's "connecting" state. */
.dzr_spin2{width:22px;height:22px;border-radius:50%;border:2.5px solid var(--dsw-alias-border-l2);border-top-color:var(--dsw-alias-state-business-primary);animation:dzr-spin2 .8s linear infinite}
@keyframes dzr-spin2{to{transform:rotate(360deg)}}
.dzr_crumbs{display:flex;align-items:center;gap:4px;flex-wrap:wrap;font-size:12px;color:var(--dsw-alias-label-secondary);min-width:0}
.dzr_crumb{border:0;background:transparent;color:var(--dsw-alias-label-secondary);font:inherit;font-size:12px;padding:2px 4px;border-radius:5px;cursor:pointer}
.dzr_crumb:hover{background:var(--dsw-alias-bg-layer-1);color:var(--dsw-alias-label-primary)}
.dzr_crumb.last{color:var(--dsw-alias-label-primary);font-weight:600}
.dzr_chips{display:flex;gap:6px;flex-wrap:wrap}
.dzr_chip{font-size:11px;padding:3px 9px;border-radius:999px;border:1px solid var(--dsw-alias-border-l1);background:var(--dsw-alias-bg-layer-1);color:var(--dsw-alias-label-secondary);cursor:pointer;white-space:nowrap}
.dzr_chip:hover{border-color:var(--dsw-alias-state-business-primary);color:var(--dsw-alias-label-primary)}
.dzr_split{flex:1;min-height:0;display:flex}
.dzr_list{flex:1;min-height:0;overflow:auto;border-right:1px solid var(--dsw-alias-border-l1)}
.dzr_list.narrow{border-right:0}
.dzr_row2{display:grid;grid-template-columns:20px 1fr 90px 150px;align-items:center;gap:8px;padding:5px 14px;border-bottom:1px solid var(--dsw-alias-border-l1);font-size:12.5px;cursor:default;min-width:0}
.dzr_row2.dir{cursor:pointer}
.dzr_row2:hover{background:var(--dsw-alias-bg-layer-1)}
.dzr_row2.sel{background:var(--dsw-alias-bg-layer-3)}
.dzr_row2 b{font-weight:500;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;min-width:0}
.dzr_row2 span{font-size:11px;color:var(--dsw-alias-label-tertiary);font-variant-numeric:tabular-nums;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.dzr_row2.head{position:sticky;top:0;background:var(--dsw-alias-bg-layer-2);color:var(--dsw-alias-label-tertiary);font-size:11px;border-bottom:1px solid var(--dsw-alias-border-l2);z-index:1}
.dzr_glyph{display:flex;align-items:center;justify-content:center;color:var(--dsw-alias-label-tertiary)}
.dzr_glyph.dir{color:var(--dsw-alias-state-business-primary)}
.dzr_preview{flex:1;min-width:0;display:flex;flex-direction:column;min-height:0}
.dzr_prehead{display:flex;align-items:center;gap:8px;padding:8px 14px;border-bottom:1px solid var(--dsw-alias-border-l1);flex:none;flex-wrap:wrap}
.dzr_prebody{flex:1;min-height:0;overflow:auto;margin:0;padding:12px 14px;font-family:ui-monospace,SFMono-Regular,Menlo,monospace;font-size:11.5px;line-height:1.6;white-space:pre-wrap;overflow-wrap:anywhere;color:var(--dsw-alias-label-secondary);background:var(--dsw-alias-bg-layer-1)}
.dzr_tag{font-size:10.5px;padding:2px 7px;border-radius:6px;border:1px solid var(--dsw-alias-border-l1);color:var(--dsw-alias-label-tertiary)}
/* ── the status float (registered on the frame-wide shell.overlay seat) ─────── */
/* Opacity comes from a custom property so a ':hover' rule can win without
   !important: a card dimmed to 30% must still be readable while you are using
   it, and the slider that set it lives inside the card. (No backticks here —
   this comment sits inside the CSS template literal.) */
.dzr_float{--dzr-op:1;position:fixed;width:334px;border-radius:16px;border:1px solid var(--dsw-alias-border-l2);background:var(--dsw-alias-bg-layer-2);color:var(--dsw-alias-label-primary);box-shadow:0 14px 44px rgb(0 0 0 / 32%);font-size:12.5px;line-height:1.5;z-index:2147483000;overflow:hidden;display:flex;flex-direction:column;user-select:none;opacity:var(--dzr-op,1)}
.dzr_float:hover{opacity:1}
.dzr_float *{box-sizing:border-box}
.dzr_fhead{display:flex;align-items:center;gap:8px;padding:9px 12px;border-bottom:1px solid var(--dsw-alias-border-l1);background:var(--dsw-alias-bg-layer-3);cursor:move;touch-action:none}
/* Header line: 名称 | 开机时间：m:ss — one row, no uptime cell in the body. */
.dzr_ftitle{flex:1;min-width:0;display:flex;align-items:baseline;gap:5px;overflow:hidden;white-space:nowrap}
.dzr_ftitle b{font-size:12.5px;font-weight:650;min-width:0;overflow:hidden;text-overflow:ellipsis;flex:none}
.dzr_fsep{font-style:normal;color:var(--dsw-alias-label-tertiary);opacity:.6;flex:none}
.dzr_fup{font-size:11.5px;color:var(--dsw-alias-label-secondary);font-variant-numeric:tabular-nums;flex:none;overflow:hidden;text-overflow:ellipsis}
.dzr_fbtn{font:inherit;font-size:13px;line-height:1;padding:3px 7px;border-radius:8px;border:1px solid var(--dsw-alias-border-l2);background:transparent;color:var(--dsw-alias-label-secondary);cursor:pointer;flex:none}
.dzr_fbtn:hover:not(:disabled){color:var(--dsw-alias-label-primary);border-color:var(--dsw-alias-state-business-primary)}
.dzr_fbtn:disabled{opacity:.45;cursor:default}
/* Two columns (battery, CPU). Each cell is a blue pair of boxes: 名称 +
   数值 on top, the axes-bearing curve plot underneath. The blue is a
   color-mix tint over a declared fallback, so a browser without color-mix
   still gets the neutral box rather than an invisible border. */
.dzr_fgrid{display:grid;grid-template-columns:repeat(2,1fr);gap:7px;padding:10px}
.dzr_fcell{display:flex;flex-direction:column;gap:6px;min-width:0}
.dzr_fbox{min-width:0;display:flex;flex-direction:column;gap:1px;padding:7px 9px;border-radius:12px;border:1px solid var(--dsw-alias-border-l1);border-color:color-mix(in srgb, var(--dsw-alias-state-business-primary) 45%, transparent);background:var(--dsw-alias-bg-layer-1);background:color-mix(in srgb, var(--dsw-alias-state-business-primary) 7%, var(--dsw-alias-bg-layer-1))}
.dzr_fbox>span:first-child{font-size:10.5px;color:var(--dsw-alias-label-tertiary);overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.dzr_fbox b{font-size:17px;font-weight:650;letter-spacing:-.01em;font-variant-numeric:tabular-nums;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.dzr_fbox b small{font-size:10.5px;font-weight:500;color:var(--dsw-alias-label-tertiary);margin-left:1px}
/* Battery thresholds run backwards — low is the bad direction — so the
   level classes below are set by the caller with the inverted scale. */
.dzr_fcell[data-level="warm"] .dzr_fbox b{color:var(--dsw-alias-state-warning-primary)}
.dzr_fcell[data-level="hot"] .dzr_fbox b{color:var(--dsw-alias-state-error-primary)}
.dzr_fplot{padding:5px 6px 3px;gap:0;align-items:stretch}
/* The plot itself: a viewBox whose pixel size is the box's inner width, so the
   tick labels draw at ~1:1 instead of being scaled down with the frame. Axes
   and grid are dashed, the curve keeps the cell's level colour, and the red
   dot plus its dashed cross-hairs mark the latest exact value. */
.dzr_fchart{display:block;width:100%;height:auto;overflow:visible}
.dzr_fchart .dzr_faxis{stroke:var(--dsw-alias-border-l2);stroke-width:1;vector-effect:non-scaling-stroke}
/* Named gridline rather than dzr_fgrid: that class already styles the body
   grid, and a stray display:grid on an SVG line would misrender it. */
.dzr_fchart .dzr_fgridline{stroke:var(--dsw-alias-border-l1);stroke-width:1;stroke-dasharray:2 3;vector-effect:non-scaling-stroke}
.dzr_fchart .dzr_ftick{stroke:var(--dsw-alias-border-l2);stroke-width:1;vector-effect:non-scaling-stroke}
.dzr_fchart .dzr_flab,.dzr_fchart .dzr_faxis-title{fill:var(--dsw-alias-label-tertiary);font-family:inherit;font-size:6.5px;font-variant-numeric:tabular-nums}
.dzr_fchart .dzr_fno{stroke:var(--dsw-alias-border-l2);stroke-width:1.2;stroke-dasharray:3 3;vector-effect:non-scaling-stroke}
.dzr_fchart polyline{fill:none;stroke:var(--dsw-alias-state-business-primary);stroke-width:1.5;stroke-linejoin:round;stroke-linecap:round;vector-effect:non-scaling-stroke}
.dzr_fcell[data-level="warm"] .dzr_fchart polyline{stroke:var(--dsw-alias-state-warning-primary)}
.dzr_fcell[data-level="hot"] .dzr_fchart polyline{stroke:var(--dsw-alias-state-error-primary)}
.dzr_fchart .dzr_fdot{fill:var(--dsw-alias-state-error-primary,#e5484d);stroke:var(--dsw-alias-bg-layer-2);stroke-width:.5}
.dzr_fchart .dzr_fcross{stroke:var(--dsw-alias-state-error-primary,#e5484d);stroke-width:1;stroke-dasharray:3 3;opacity:.75;vector-effect:non-scaling-stroke}
.dzr_ffoot{display:flex;align-items:center;gap:8px;padding:8px 12px;border-top:1px solid var(--dsw-alias-border-l1);flex-wrap:wrap}
.dzr_ffoot .dzr_note{flex:1;min-width:80px}
.dzr_ferr{padding:0 12px 8px}
.dzr_fopacity{width:74px;accent-color:var(--dsw-alias-state-business-primary);cursor:pointer;flex:none}
.dzr_range{width:100%;accent-color:var(--dsw-alias-state-business-primary);cursor:pointer}
`
    function installStyles() {
      const id = 'dsh-dzrobot-ssh/styles'
      if (typeof document === 'undefined') return
      if (document.querySelector(`style[data-plugin-css=${JSON.stringify(id)}]`) !== null) return
      const tag = document.createElement('style')
      tag.dataset.plugin = 'dsh-dzrobot-ssh'
      tag.dataset.pluginCss = id
      tag.textContent = CSS
      document.head.appendChild(tag)
    }

    // ── helpers ───────────────────────────────────────────────────────────────
    async function api(path, options) {
      const response = await fetch(`${API}${path}`, {
        ...options,
        headers: { 'content-type': 'application/json', ...options?.headers },
      })
      const text = await response.text()
      let payload
      try { payload = text === '' ? {} : JSON.parse(text) } catch { payload = { error: text.slice(0, 200) } }
      if (!response.ok) throw new Error(payload?.error ?? `HTTP ${response.status}`)
      return payload
    }

    /** Report one integration step to the Host, where it lands in a file. */
    function diag(step, data) {
      try {
        void fetch(`${API}/diag`, {
          method: 'POST',
          headers: { 'content-type': 'application/json' },
          body: JSON.stringify({ step, data }),
        }).catch(() => {})
      } catch { /* diagnostics never break the feature */ }
    }

    /** What each service this flow needs actually looks like right now. */
    function describeService(value) {
      if (value === undefined || value === null) return { present: false }
      const names = new Set()
      let cursor = value
      let depth = 0
      while (cursor !== null && cursor !== undefined && depth < 4) {
        for (const name of Object.getOwnPropertyNames(cursor)) names.add(name)
        cursor = Object.getPrototypeOf(cursor)
        depth += 1
      }
      return {
        present: true,
        type: typeof value,
        methods: [...names].filter(name => name !== 'constructor').sort(),
      }
    }

    function surveyServices(ctx) {
      const report = {}
      for (const name of ['sidebarRight', 'webTerminals', 'slots', 'locale', 'layout', 'shortcuts', 'sessions', 'commands']) {
        report[name] = describeService(ctx.get?.(name))
      }
      return report
    }

    function localeTag(ctx) {
      try {
        const snapshot = typeof ctx.get('locale')?.getLocale === 'function'
          ? ctx.get('locale').getLocale()
          : ctx.get('locale')?.getSnapshot?.()
        const value = snapshot?.active ?? snapshot?.id
        return typeof value === 'string' ? value : (document.documentElement.lang || 'en')
      } catch { return 'en' }
    }

    function waitFor(predicate, { timeoutMs = 12000, intervalMs = 120 } = {}) {
      return new Promise((resolve, reject) => {
        const started = Date.now()
        const tick = () => {
          let value
          try { value = predicate() } catch (error) { reject(error); return }
          if (value) { resolve(value); return }
          if (Date.now() - started >= timeoutMs) { reject(new Error('timed out')); return }
          setTimeout(tick, intervalMs)
        }
        tick()
      })
    }

    /**
     * Make the Conversation the active main panel and resolve a right-column
     * target there.
     *
     * `commandTarget` reads the right column's on-screen Session, and that
     * Session exists only while the Conversation is the active main panel
     * (sb-right: `show(activePanelId === null ? selected : undefined)`). While
     * THIS plugin's panel is active it is undefined by construction — so opening
     * a right-column tool (terminal, files, VNC) always starts by returning to
     * the Conversation, whose right column is where these tools live.
     *
     * Refuses before navigating when no Session is current at all: switching and
     * then failing would strand the user with no word on why.
     *
     * @returns a target to place tabs against, or `undefined`.
     */
    async function ensureConversation(ctx, element, step) {
      const settings = ctx.get('sidebarRight')
      if (settings === undefined) return undefined
      let target = settings.commandTarget(element)
      diag(`${step}.target`, {
        found: target !== undefined,
        sessionId: target?.sessionId,
        paneId: target?.paneId,
        host: target?.host,
        tabId: target?.tabId,
      })
      if (target !== undefined) return target

      let hasSession = true
      try {
        const uiSession = ctx.get('uiSession')
        const current = uiSession?.adapter?.current
        // Only a *present* service can answer "no session": absent service or
        // unknown shape falls through and lets the poll decide below.
        if (uiSession !== undefined && current !== undefined) {
          const key = current.getSnapshot?.().key
          hasSession = key !== undefined && key !== null
        }
      } catch { /* unknown shape — assume a Session and let the poll decide */ }
      const layout = ctx.get('layout')
      if (!hasSession || layout === undefined || typeof layout.selectPanel !== 'function') {
        diag(`${step}.reveal`, { skipped: true, hasSession, hasLayout: layout !== undefined })
        return undefined
      }
      diag(`${step}.reveal`, { action: 'selectPanel(null)' })
      // Remember where we were: if the target never resolves, returning to the
      // original panel puts the user where the error message can actually be
      // seen (and where they were before clicking).
      const previousPanel = layout.panelInfo?.getSnapshot?.()?.activePanelId ?? null
      try { layout.selectPanel(null) } catch (error) {
        diag(`${step}.reveal.error`, { message: String(error?.message ?? error) })
        return undefined
      }
      // The on-screen Session, its adopted surface and the dock layout settle
      // asynchronously across store subscriptions and React commits — poll
      // rather than guessing one delay that is either wasted or too short.
      for (let attempt = 0; attempt < 20 && target === undefined; attempt += 1) {
        await new Promise(resolve => setTimeout(resolve, 100))
        target = settings.commandTarget()
      }
      diag(`${step}.target.retry`, { found: target !== undefined, sessionId: target?.sessionId })
      if (target === undefined) {
        // Restore the panel so the failure is reported where the user clicked;
        // selectPanel throws if the id is no longer registered — harmless.
        if (previousPanel !== null) { try { layout.selectPanel(previousPanel) } catch { /* gone */ } }
      }
      return target
    }

    /**
     * Open one of this plugin's right-column tools as a tab.
     *
     * The tab type must be registered (apply() does it); `openTab` then claims a
     * fixed contentId when the type is not `multiple`, so a second click focuses
     * the already-open tool instead of stacking copies. The right column reveals
     * itself through the same chain the stock terminal uses
     * (`openContent → expanded → RightbarSeat syncPresentation → openRightbar`).
     */
    async function openToolTab(ctx, kind, element) {
      const settings = ctx.get('sidebarRight')
      diag('tool.start', { kind, sidebarRight: !!settings })
      if (settings === undefined) return { ok: false, reason: 'services-missing' }
      const target = await ensureConversation(ctx, element, 'tool')
      if (target === undefined) return { ok: false, reason: 'no-session' }
      try {
        settings.openTab(kind)
      } catch (error) {
        diag('tool.error', { kind, message: String(error?.message ?? error) })
        return { ok: false, reason: 'open-failed', detail: String(error?.message ?? error) }
      }
      diag('tool.opened', { kind, sessionId: target.sessionId })
      return { ok: true }
    }

    /**
     * Open a terminal tab and run `command` in it.
     *
     * The order is forced by the services: the tab must exist before its body
     * mounts, the body's mount is what attaches the terminal, and input is only
     * accepted once an attachment holds write control. So: open, wait for the tab
     * record, wait for the view to become writable, then write.
     */
    async function connect(ctx, element, onPhase) {
      const settings = ctx.get('sidebarRight')
      const terminals = ctx.get('webTerminals')
      diag('connect.start', { sidebarRight: !!settings, webTerminals: !!terminals })
      if (settings === undefined || terminals === undefined) {
        return { ok: false, reason: 'terminal-services-missing' }
      }

      onPhase('probing')
      const status = await api('/probe', { method: 'POST' })
      diag('connect.probe', { state: status.state, detail: status.detail, latencyMs: status.latencyMs })
      if (status.state !== 'ready') return { ok: false, reason: status.state, detail: status.detail }

      // The probe answers the state; the address and the command line come from
      // the settings, read together so a change mid-click cannot mix two cars.
      const current = await api('/status')
      const command = status.command ?? current.command
      const car = current.settings ?? {}
      const label = car.sshAlias || `${car.user ?? ''}@${car.host ?? ''}`
      diag('connect.command', { command, label })

      const target = await ensureConversation(ctx, element, 'connect')
      if (target === undefined) return { ok: false, reason: 'no-session' }
      return openInTerminal(settings, terminals, target, command, label, onPhase)
    }

    /**
     * Open a terminal tab in the target's session and type the command into it.
     *
     * Split out of {@link connect} because the flow is reached twice: once with
     * the target the caller captured, and once with the target re-resolved after
     * revealing the right column.
     */
    async function openInTerminal(settings, terminals, target, command, label, onPhase) {
      const before = new Set(
        settings.openTabs.getSnapshot().filter(tab => tab.kind === 'terminal').map(tab => tab.tabId ?? tab.id),
      )
      onPhase('opening')
      settings.openTabFromTarget('terminal', target)

      // A new tab is the normal outcome, but the placement rules may reuse an
      // existing terminal instead (the guide's replacement rule, an already
      // focused terminal page). Falling back to the newest terminal of this
      // session keeps the click useful in that case rather than timing out.
      const tab = await waitFor(() => {
        const tabs = settings.openTabs.getSnapshot().filter(candidate =>
          candidate.kind === 'terminal' && candidate.sessionId === target.sessionId)
        return tabs.find(candidate => !before.has(candidate.tabId ?? candidate.id)) ?? tabs[tabs.length - 1]
      })
      const tabId = tab.tabId ?? tab.id
      diag('connect.tab', { tabId, contentId: tab.contentId, sessionId: tab.sessionId, keys: Object.keys(tab) })

      const view = terminals.view(tab.sessionId, tabId, tab.contentId)
      diag('connect.view', {
        hasView: !!view,
        id: view?.id,
        keys: view === undefined ? [] : Object.keys(view),
        phase: view?.state?.getSnapshot?.().phase,
      })

      await waitFor(() => view.state.getSnapshot().writable === true, { timeoutMs: 25000 })
      const snapshot = view.state.getSnapshot()
      diag('connect.writable', { phase: snapshot.phase, title: snapshot.title, info: snapshot.info })

      view.write(`${command}\r`)
      diag('connect.wrote', { command })
      // Cosmetic, and bounded by the Host's own 120-character title rule.
      try { view.rename(`小车 ${label}`.trim()) } catch { /* rename is cosmetic */ }
      onPhase('connected')
      return { ok: true }
    }

    // ── shared state ──────────────────────────────────────────────────────────
    /**
     * One status object both seats read.
     *
     * The two seats are separate registrations with no shared parent, so the
     * store lives in the closure: a probe started from the settings page updates
     * the sidebar dot too.
     */
    function createStore() {
      const listeners = new Set()
      // The float shares this store too: the panel row that opens it and the
      // overlay card that renders it are two registrations apart, and
      // `floatOpen` is the wire between them.
      let value = { phase: 'idle', probe: undefined, car: undefined, message: '', command: '', floatOpen: floatOpenByDefault() }
      return {
        get: () => value,
        set: patch => {
          value = { ...value, ...patch }
          for (const listener of listeners) listener(value)
        },
        subscribe: listener => {
          listeners.add(listener)
          return () => listeners.delete(listener)
        },
      }
    }

    function useStore(store) {
      const [value, setValue] = useState(store.get())
      useEffect(() => store.subscribe(setValue), [store])
      return value
    }

    // ── car glyph ─────────────────────────────────────────────────────────────

    /** The car glyph, used by the panel header, the settings page and the icon. */
    function CarGlyph({ size = 16 }) {
      return h('svg', {
        width: size, height: size, viewBox: '0 0 16 16', fill: 'none', 'aria-hidden': 'true',
      }, [
        h('path', { key: 'body', d: 'M2.5 10V8.4c0-.5.2-.9.6-1.2l1.3-.9c.3-.2.6-.3 1-.3h5.2c.4 0 .7.1 1 .3l1.3.9c.4.3.6.7.6 1.2V10', stroke: 'currentColor', strokeLinecap: 'round' }),
        h('path', { key: 'roof', d: 'M5.2 6l.9-1.4c.2-.3.5-.4.8-.4h2.2c.3 0 .6.1.8.4L10.8 6', stroke: 'currentColor', strokeLinecap: 'round' }),
        h('circle', { key: 'w1', cx: '4.6', cy: '11', r: '1.5', stroke: 'currentColor' }),
        h('circle', { key: 'w2', cx: '11.4', cy: '11', r: '1.5', stroke: 'currentColor' }),
        h('path', { key: 'ground', d: 'M1.5 13.6h13', stroke: 'currentColor', strokeLinecap: 'round' }),
      ])
    }

    // ── settings page ─────────────────────────────────────────────────────────
    function makeSettingsPage(ctx, store, t) {
      return function SettingsPage() {
        const state = useStore(store)
        const [settings, setSettings] = useState(undefined)
        const [busy, setBusy] = useState('')
        const [notice, setNotice] = useState('')
        const [command, setCommand] = useState('')
        const [copied, setCopied] = useState(false)
        const buttonRef = useRef(null)

        const load = useCallback(() => {
          api('/status').then(payload => {
            setSettings(payload.settings)
            setCommand(payload.command)
            store.set({ probe: payload.probe, car: payload.car })
          }).catch(error => setNotice(String(error?.message ?? error)))
        }, [])
        useEffect(load, [load])

        const patch = (key, value) => setSettings(current => ({ ...current, [key]: value }))

        const save = async () => {
          setBusy('save'); setNotice('')
          try {
            const payload = await api('/config', { method: 'POST', body: JSON.stringify(settings) })
            setSettings(payload.settings)
            setCommand(payload.command)
            setNotice(t('saved'))
          } catch (error) {
            setNotice(`${t('saveFailed')}: ${String(error?.message ?? error)}`)
          } finally {
            setBusy('')
          }
        }

        const test = async () => {
          setBusy('probe'); setNotice('')
          try {
            const result = await api('/probe', { method: 'POST' })
            store.set({ probe: result, car: result.car })
            if (result.command !== undefined) setCommand(result.command)
            if (result.state !== 'ready') setNotice(t('notReady', { detail: result.detail }))
          } catch (error) {
            setNotice(String(error?.message ?? error))
          } finally {
            setBusy('')
          }
        }

        const open = async () => {
          setBusy('open'); setNotice('')
          try {
            const result = await connect(ctx, buttonRef.current, () => {})
            if (!result.ok) {
              setNotice(result.reason === 'no-session' ? t('noSession') : t('notReady', { detail: result.detail ?? result.reason }))
            }
          } catch (error) {
            setNotice(t('terminalFailed', { message: String(error?.message ?? error) }))
          } finally {
            setBusy('')
          }
        }

        const copy = () => {
          try {
            void navigator.clipboard?.writeText(command)
            setCopied(true)
            setTimeout(() => setCopied(false), 1600)
          } catch { /* clipboard unavailable */ }
        }

        if (settings === undefined) {
          return h('div', { className: 'dzr_root' }, h('div', { className: 'dzr_note' }, '…'))
        }

        const probe = state.probe
        const car = state.car
        return h('div', { className: 'dzr_root' }, [
          h('div', { key: 'hero', className: 'dzr_card' }, [
            h('div', { key: 'head', className: 'dzr_head' }, [
              h('span', { key: 'glyph', className: 'dzr_icon' }, CarGlyph({ size: 22 })),
              h('span', { key: 'titles' }, [
                h('div', { key: 'h', className: 'dzr_h' }, t('title')),
                h('div', { key: 's', className: 'dzr_sub' }, t('subtitle')),
              ]),
              h('span', {
                key: 'badge',
                className: 'dzr_badge',
                'data-state': probe?.state ?? 'idle',
              }, probe === undefined ? t('neverProbed') : t(STATE_KEY[probe.state] ?? 'stateError')),
            ]),
            h('div', { key: 'actions', className: 'dzr_actions' }, [
              h('button', {
                key: 'open', ref: buttonRef, type: 'button', className: 'dzr_action primary',
                disabled: busy !== '', onClick: open,
              }, busy === 'open' ? t('connecting') : t('openTerminal')),
              h('button', {
                key: 'probe', type: 'button', className: 'dzr_action',
                disabled: busy !== '', onClick: test,
              }, busy === 'probe' ? t('probing') : t('probe')),
            ]),
            probe !== undefined && probe.detail !== undefined
              ? h('div', { key: 'pdetail', className: `dzr_note${probe.state === 'ready' ? '' : ' dzr_warn'}` },
                `${probe.detail}${probe.latencyMs !== undefined ? ` · ${t('latency', { ms: probe.latencyMs })}` : ''} · ${probe.target ?? ''}`)
              : null,
            notice !== '' ? h('div', { key: 'notice', className: `dzr_note${/失败|failed|不可达|unreachable/i.test(notice) ? ' dzr_warn' : ''}` }, notice) : null,
          ]),

          car !== undefined ? h('div', { key: 'car', className: 'dzr_grid' }, [
            kv('carUser', car.user), kv('carHost', car.host), kv('carKernel', car.kernel), kv('carUp', car.up),
          ]) : null,

          h('div', { key: 'addr', className: 'dzr_card' }, [
            h('div', { key: 'h', className: 'dzr_row' }, h('span', { className: 'dzr_h' }, t('sectionAddress'))),
            h('div', { key: 'fields', className: 'dzr_grid' }, [
              field('host', t('fieldHost'), settings.host),
              field('user', t('fieldUser'), settings.user),
              field('port', t('fieldPort'), settings.port),
              field('sshAlias', t('fieldAlias'), settings.sshAlias),
            ]),
            h('div', { key: 'opts', className: 'dzr_field' }, [
              h('span', { key: 'l' }, t('fieldOptions')),
              h('input', {
                key: 'i', className: 'dzr_input', value: settings.sshOptions ?? '',
                onChange: event => patch('sshOptions', event.target.value),
              }),
            ]),
            // The float's opacity lives here as well as on the card itself:
            // settings are the place to find a knob whose name you forgot, and
            // both writers go through the same `/config` route and clamp.
            h('div', { key: 'opacity', className: 'dzr_field' }, [
              h('span', { key: 'l' }, `${t('fieldFloatOpacity')} · ${settings.floatOpacity ?? 100}%`),
              h('input', {
                key: 'i', className: 'dzr_range', type: 'range', min: 30, max: 100, step: 5,
                value: settings.floatOpacity ?? 100,
                onChange: event => patch('floatOpacity', Math.min(100, Math.max(30, Math.round(Number(event.target.value))))),
              }),
            ]),
            h('div', { key: 'opacityNote', className: 'dzr_note' }, t('opacityHint')),
            h('div', { key: 'save', className: 'dzr_actions' }, h('button', {
              type: 'button', className: 'dzr_action', disabled: busy !== '', onClick: save,
            }, busy === 'save' ? '…' : t('save'))),
          ]),

          h('div', { key: 'vnc', className: 'dzr_card' }, [
            h('div', { key: 'h', className: 'dzr_row' }, [
              h('span', { key: 't', className: 'dzr_h' }, t('viewVnc')),
              h('span', { key: 's', className: 'dzr_sub' }, `${settings.vncHost || settings.host}:${settings.vncPort}`),
            ]),
            h('div', { key: 'fields', className: 'dzr_grid' }, [
              field('vncHost', t('fieldVncHost'), settings.vncHost),
              field('vncPort', t('fieldVncPort'), settings.vncPort),
              h('label', { key: 'mode', className: 'dzr_field' }, [
                h('span', { key: 'l' }, t('fieldVncMode')),
                h('select', {
                  key: 'i', className: 'dzr_input', value: settings.vncMode ?? 'direct',
                  onChange: event => patch('vncMode', event.target.value),
                }, [
                  h('option', { key: 'd', value: 'direct' }, t('vncModeDirect')),
                  h('option', { key: 't', value: 'tunnel' }, t('vncModeTunnel')),
                ]),
              ]),
              field('vncPassword', t('vncPassword'), settings.vncPassword, 'password'),
            ]),
            h('div', { key: 'note', className: 'dzr_note' }, t('vncModeNote')),
            h('div', { key: 'files', className: 'dzr_field' }, [
              h('span', { key: 'l' }, t('fieldFilesRoot')),
              h('input', {
                key: 'i', className: 'dzr_input', value: settings.filesRoot ?? '',
                onChange: event => patch('filesRoot', event.target.value),
              }),
            ]),
          ]),

          h('div', { key: 'cmd', className: 'dzr_card' }, [            h('div', { key: 'head', className: 'dzr_head' }, [
              h('span', { key: 'h', className: 'dzr_h' }, t('sectionCommand')),
              h('button', {
                key: 'copy', type: 'button', className: 'dzr_action', onClick: copy,
              }, copied ? t('copied') : t('copy')),
            ]),
            h('div', { key: 'body', className: 'dzr_mono' }, command),
            h('div', { key: 'note', className: 'dzr_note' }, t('connectHint')),
          ]),
        ])

        function kv(keyId, value) {
          return h('div', { key: keyId, className: 'dzr_kv' }, [
            h('b', { key: 'v' }, value === undefined || value === '' ? '—' : String(value)),
            h('span', { key: 'k' }, t(keyId)),
          ])
        }
        function field(key, labelText, value, type) {
          return h('label', { key, className: 'dzr_field' }, [
            h('span', { key: 'l' }, labelText),
            h('input', {
              key: 'i',
              className: 'dzr_input',
              ...type === undefined ? {} : { type },
              value: value ?? '',
              onChange: event => patch(key, event.target.value),
            }),
          ])
        }
      }
    }

    // ── the car panel ─────────────────────────────────────────────────────────
    /** Human-readable byte count. */
    function size(value) {
      const bytes = Number(value) || 0
      if (bytes < 1024) return `${bytes} B`
      if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`
      if (bytes < 1024 * 1024 * 1024) return `${(bytes / 1048576).toFixed(1)} MB`
      return `${(bytes / 1073741824).toFixed(2)} GB`
    }

    function when(seconds) {
      if (!Number.isFinite(seconds) || seconds <= 0) return '—'
      const date = new Date(seconds)
      const pad = value => String(value).padStart(2, '0')
      return `${date.getFullYear()}-${pad(date.getMonth() + 1)}-${pad(date.getDate())} ${pad(date.getHours())}:${pad(date.getMinutes())}`
    }

    /**
     * The VNC desktop, as an embedded document on this plugin's own http origin.
     *
     * LEGACY — superseded by makeVncConnectorPanel, which is what the VNC tab
     * now mounts (one VNC surface total). Kept because vnc.html and its bridge
     * are still served and the regression suite covers this document.
     *
     * Not an in-page noVNC instance, and that is deliberate: the app itself is
     * loaded under a custom `dsh-app://` scheme where ES module loading fails
     * ("Failed to fetch dynamically imported module") and where a WebSocket built
     * from `location.host` targets a host literally named `app`. The iframe is
     * served over http by the host half, so inside it `import './novnc/core/rfb.js'`
     * and the WebSocket are plain same-origin operations — and the panel only has
     * to relay messages, not understand RFB.
     *
     * Dialing stays explicit: a VNC session takes over the car's desktop input,
     * and an abandoned connection counts against TigerVNC's blacklist.
     */
    function makeVncView(ctx, t) {
      return function VncView({ settings, origin: originProp }) {
        // Coerced, not assumed: the panel passes this from an API response, and a
        // missing value must degrade to the "no address" notice rather than throw
        // somewhere inside the render.
        const origin = typeof originProp === 'string' ? originProp : ''
        const frameRef = useRef(null)
        const ready = useRef(false)
        const pending = useRef(null)
        const [phase, setPhase] = useState('idle')
        const [message, setMessage] = useState('')
        const [needPassword, setNeedPassword] = useState(false)
        const [blacklisted, setBlacklisted] = useState(false)
        const [password, setPassword] = useState(settings.vncPassword ?? '')
        const [desktop, setDesktop] = useState('')
        /** Set by an explicit 断开, so its echo is not reported as a failure. */
        const userStopped = useRef(false)

        /**
         * Send a command to the frame, queueing it until the frame says `ready`.
         *
         * postMessage to a frame that has not run its module script yet is simply
         * lost, and the panel is interactive the moment it renders — so a click in
         * the first instants would otherwise do nothing at all.
         */
        const post = useCallback(payload => {
          const packet = { source: 'dzrobot-panel', ...payload }
          if (!ready.current) { pending.current = packet; return }
          try { frameRef.current?.contentWindow?.postMessage(packet, '*') }
          catch { /* frame not ready */ }
        }, [])

        useEffect(() => {
          const onMessage = event => {
            const data = event.data
            if (data === null || typeof data !== 'object' || data.source !== 'dzrobot-vnc') return
            if (data.type === 'ready') {
              // The frame just loaded: it is idle, not connecting. Flushing a
              // command queued before readiness keeps an early click working.
              ready.current = true
              const queued = pending.current
              pending.current = null
              if (queued !== null) {
                try { frameRef.current?.contentWindow?.postMessage(queued, '*') } catch { /* gone */ }
              } else {
                setPhase(current => (current === 'idle' ? current : 'idle'))
              }
            } else if (data.type === 'connecting') { setPhase('connecting'); setMessage(''); setNeedPassword(false); setBlacklisted(false) }
            else if (data.type === 'connected') { setPhase('connected'); setDesktop(String(data.desktop ?? '')); setMessage(''); setNeedPassword(false); setBlacklisted(false) }
            else if (data.type === 'credentials') { setNeedPassword(true); setPhase('auth') }
            else if (data.type === 'credentials-sent') { setNeedPassword(false); setPhase('connecting') }
            else if (data.type === 'disconnected') {
              setNeedPassword(false)
              if (userStopped.current) {
                // An explicit 断开 is not an error — keep the idle card, not a
                // "dropped" warning the user themselves caused.
                userStopped.current = false
                setPhase('idle')
                setMessage('')
              } else {
                setPhase('disconnected')
                if (data.clean !== false) {
                  setMessage('')
                } else if (data.closeReason) {
                  // The bridge explains its own refusal (e.g. a dial failure);
                  // show that instead of a generic "dropped" line.
                  setMessage(String(data.closeReason))
                } else if (data.closeCode !== undefined) {
                  setMessage(`${t('vncDropped')}（WebSocket code ${data.closeCode}）`)
                } else {
                  setMessage(data.reason || t('vncDropped'))
                }
              }
            }
            else if (data.type === 'failed') {
              setPhase('failed')
              setMessage(String(data.message ?? '失败'))
              // The car answers refusals in English ("Too many security
              // failures"), so the frame's own classification is the only
              // reliable blacklist signal — never string-matching here.
              setBlacklisted(data.blacklisted === true)
              // Only a security refusal deserves the password card; a transport
              // error shown with a password box sends the user down the wrong path.
              setNeedPassword(data.kind === 'security' && data.blacklisted !== true)
            }
            // Full wire evidence: noVNC's own events are near-empty, so the
            // frame's socket log and document URL are what make a report
            // diagnosable after the fact.
            diag(`vnc.${String(data.type ?? 'event')}`, {
              message: data.message, clean: data.clean,
              closeCode: data.closeCode, closeReason: data.closeReason,
              href: data.href, ws: data.ws,
            })
          }
          window.addEventListener('message', onMessage)
          return () => window.removeEventListener('message', onMessage)
        }, [])

        const connect = useCallback(() => {
          userStopped.current = false
          setPhase('connecting')
          setMessage('')
          setNeedPassword(false)
          post({ type: 'connect', password })
        }, [post, password])

        const stop = useCallback(() => {
          userStopped.current = true
          post({ type: 'disconnect' })
          setPhase('idle')
          setNeedPassword(false)
        }, [post])

        const submitPassword = useCallback(() => {
          if (password === '') return
          post({ type: 'password', password })
          setNeedPassword(false)
          setPhase('connecting')
        }, [post, password])

        const src = origin === '' ? '' : `${origin}/api/dzrobot-ssh/vnc.html`
        const target = settings.vncHost || settings.host
        const targetLabel = settings.vncMode === 'tunnel' ? `ssh 隧道 → ${target}:${settings.vncPort}` : `${target}:${settings.vncPort}`

        const statusText = {
          idle: t('vncIdle'),
          loading: t('vncIdle'),
          connecting: t('vncConnecting'),
          auth: t('vncWaitingPassword'),
          connected: `${t('vncConnected')}${desktop === '' ? '' : ` · ${desktop}`}`,
          failed: message === '' ? t('vncDisconnected') : message,
          disconnected: message || t('vncDisconnected'),
        }[phase] ?? t('vncIdle')

        // The overlay is only painted while nothing is on screen: once the
        // desktop is up, the iframe must receive the pointer untouched.
        const overlay = phase !== 'connected' && src !== ''
          ? h('div', { key: 'overlay', className: 'dzr_center' }, [
            phase === 'auth' || needPassword
              ? h('div', { key: 'ask', className: 'dzr_herocard' }, [
                h('h3', { key: 't' }, t('vncNeedPassword')),
                h('p', { key: 'h' }, t('vncPasswordHint')),
                h('input', {
                  key: 'i', className: 'dzr_input', type: 'password', autoFocus: true,
                  value: password, placeholder: t('vncPassword'),
                  style: { width: '220px', textAlign: 'center' },
                  onChange: event => setPassword(event.target.value),
                  onKeyDown: event => { if (event.key === 'Enter') submitPassword() },
                }),
                h('button', {
                  key: 'b', type: 'button', className: 'dzr_action primary',
                  disabled: password === '', onClick: submitPassword,
                }, t('vncSubmitPassword')),
              ])
              : h('div', { key: 'idle', className: 'dzr_herocard' }, [
                h('div', { key: 'g', className: 'dzr_icon' }, CarGlyph({ size: 30 })),
                h('h3', { key: 't' },
                  phase === 'failed' ? t('vncFailed')
                    : phase === 'connecting' ? t('vncConnecting')
                      : phase === 'disconnected' ? t('vncDisconnected')
                        : t('vncIdleTitle')),
                h('p', { key: 's' },
                  phase === 'failed'
                    ? (message || t('vncDisconnected'))
                    : phase === 'connecting'
                      ? `${targetLabel} → ${origin.replace(/^https?:\/\//, '')}`
                      : phase === 'disconnected'
                        ? (message || t('vncIdleHint'))
                        : `${targetLabel} · ${t('vncIdleHint')}`),
                (phase === 'failed' || phase === 'disconnected') && blacklisted
                  ? h('p', { key: 'bl' }, t('vncBlacklisted'))
                  : null,
                phase === 'connecting'
                  ? h('div', { key: 'sp', className: 'dzr_spin2', 'aria-hidden': 'true' })
                  : h('button', {
                    key: 'b', type: 'button', className: 'dzr_action primary',
                    disabled: src === '', onClick: connect,
                  }, phase === 'disconnected' || phase === 'failed' ? t('vncReconnect') : t('vncConnect')),
              ]),
          ])
          : null

        return h('div', { className: 'dzr_pbody' }, [
          h('div', { key: 'bar', className: 'dzr_toolbar' }, [
            h('button', {
              key: 'go', type: 'button', className: 'dzr_action primary',
              disabled: src === '' || phase === 'connecting' || phase === 'auth',
              onClick: connect,
            }, phase === 'connected' ? t('vncReconnect') : t('vncConnect')),
            phase === 'connected' || phase === 'connecting' || phase === 'auth'
              ? h('button', { key: 'stop', type: 'button', className: 'dzr_action', onClick: stop }, t('vncDisconnect'))
              : null,
            h('button', {
              key: 'cad', type: 'button', className: 'dzr_action',
              disabled: phase !== 'connected',
              title: 'Ctrl+Alt+Del',
              onClick: () => post({ type: 'cad' }),
            }, t('vncSendCad')),
            h('span', {
              key: 'pill', className: 'dzr_pill', 'data-phase': phase, style: { marginLeft: 'auto' },
              title: statusText,
            }, [h('i', { key: 'd' }), h('span', { key: 't' }, statusText)]),
            h('span', { key: 'target', className: 'dzr_tag', title: origin }, targetLabel),
          ]),
          h('div', { key: 'stage', className: 'dzr_screen' }, [
            src === ''
              ? h('div', { key: 'noorigin', className: 'dzr_center' }, h('div', { className: 'dzr_herocard' }, [
                h('h3', { key: 't' }, t('vncNoOrigin')),
              ]))
              : h('iframe', {
                key: 'frame',
                ref: frameRef,
                src,
                title: t('viewVnc'),
                // Served by this plugin itself, so it keeps its own origin (the
                // module import needs it) and is not sandboxed away from it.
                allow: 'clipboard-read; clipboard-write',
                style: { width: '100%', height: '100%', border: '0', display: 'block', background: '#000' },
              }),
            overlay,
          ]),
        ])
      }
    }

    /**
     * The car's file browser.
     *
     * Every path crossing this component goes to the Host as a query parameter
     * and is quoted for the remote shell there; joining a name onto a path here
     * is the only path arithmetic allowed, and `..` is dropped rather than
     * normalised so a crafted name cannot climb out of the directory shown.
     */
    function makeFilesView(ctx, t) {
      return function FilesView({ settings }) {
        const [path, setPath] = useState(settings.filesRoot ?? '/home/duzhong')
        const [listing, setListing] = useState(undefined)
        const [error, setError] = useState('')
        const [busy, setBusy] = useState(false)
        const [selected, setSelected] = useState(undefined)
        const [preview, setPreview] = useState(undefined)
        const fileInput = useRef(null)

        const load = useCallback(async target => {
          setBusy(true)
          setError('')
          setPreview(undefined)
          try {
            const payload = await api(`/files?path=${encodeURIComponent(target)}`)
            setListing(payload)
            setPath(payload.path)
            diag('files.list', { path: payload.path, entries: payload.entries?.length ?? 0 })
          } catch (problem) {
            // Failures are reported too: without this, "the file view is empty"
            // is indistinguishable from "the listing was refused", and the two
            // need completely different fixes.
            const message = String(problem?.message ?? problem)
            setError(message)
            setListing(undefined)
            diag('files.error', { path: target, message })
          } finally {
            setBusy(false)
          }
        }, [])

        useEffect(() => { void load(settings.filesRoot ?? '/home/duzhong') }, [load, settings.filesRoot])

        const open = useCallback(async entry => {
          if (entry.type === 'directory') {
            const base = path.endsWith('/') ? path : `${path}/`
            void load(`${base}${entry.name}`)
            return
          }
          setSelected(entry)
          setPreview({ loading: true })
          try {
            const payload = await api(`/file?path=${encodeURIComponent(path.endsWith('/') ? path + entry.name : `${path}/${entry.name}`)}`)
            setPreview(payload)
          } catch (problem) {
            setPreview({ ok: false, error: String(problem?.message ?? problem) })
          }
        }, [path, load])

        const upload = useCallback(async file => {
          const base = path.endsWith('/') ? path : `${path}/`
          setBusy(true)
          try {
            const response = await fetch(`${API}/upload?path=${encodeURIComponent(`${base}${file.name}`)}`, {
              method: 'POST',
              body: file,
            })
            if (!response.ok) throw new Error(`HTTP ${response.status}`)
            diag('files.upload', { name: file.name, size: file.size })
            void load(path)
          } catch (problem) {
            setError(String(problem?.message ?? problem))
          } finally {
            setBusy(false)
          }
        }, [path, load])

        const crumbs = () => {
          const parts = path.split('/').filter(Boolean)
          const rows = [{ label: t('filesRoot'), target: '/' }]
          let accumulated = ''
          for (const part of parts) {
            accumulated += `/${part}`
            rows.push({ label: part, target: accumulated })
          }
          return rows
        }

        const fullPath = name => (path.endsWith('/') ? `${path}${name}` : `${path}/${name}`)

        return h('div', { className: 'dzr_pbody' }, [
          h('div', { key: 'bar', className: 'dzr_toolbar' }, [
            h('button', { key: 'up', type: 'button', className: 'dzr_action', disabled: listing?.parent === null || listing === undefined, onClick: () => void load(listing.parent) }, t('filesUp')),
            h('button', { key: 're', type: 'button', className: 'dzr_action', disabled: busy, onClick: () => void load(path) }, t('filesRefresh')),
            h('button', { key: 'ul', type: 'button', className: 'dzr_action', disabled: busy, onClick: () => fileInput.current?.click() }, t('filesUpload')),
            h('input', {
              key: 'file', ref: fileInput, type: 'file', style: { display: 'none' },
              onChange: event => {
                const file = event.target.files?.[0]
                event.target.value = ''
                if (file !== undefined) void upload(file)
              },
            }),
            h('div', { key: 'crumbs', className: 'dzr_crumbs' }, crumbs().map((crumb, index, all) =>
              h('button', {
                key: crumb.target,
                type: 'button',
                className: `dzr_crumb${index === all.length - 1 ? ' last' : ''}`,
                onClick: () => void load(crumb.target),
              }, `${crumb.label}${index === all.length - 1 ? '' : ' /'}`))),
          ]),
          h('div', { key: 'chips', className: 'dzr_toolbar' }, h('div', { className: 'dzr_chips' },
            (settings.filesShortcuts ?? []).map(item => h('button', {
              key: item.path, type: 'button', className: 'dzr_chip', onClick: () => void load(item.path),
            }, item.label)))),

          h('div', { key: 'split', className: 'dzr_split' }, [
            h('div', { key: 'list', className: `dzr_list${preview === undefined ? ' narrow' : ''}` }, [
              error !== '' ? h('div', { key: 'err', className: 'dzr_pad' }, [
                h('div', { key: 'm', className: 'dzr_note dzr_warn' }, error),
                h('div', { key: 'a', className: 'dzr_actions' }, h('button', {
                  key: 'r', type: 'button', className: 'dzr_action', disabled: busy, onClick: () => void load(path),
                }, t('filesRefresh'))),
              ]) : null,
              busy && listing === undefined ? h('div', { key: 'busy', className: 'dzr_pad' }, h('div', { className: 'dzr_note' }, t('filesLoading'))) : null,
              listing === undefined ? null : [
                h('div', { key: 'head', className: 'dzr_row2 head' }, [
                  h('span', { key: 'i' }, ''), h('span', { key: 'n' }, t('filesName')),
                  h('span', { key: 's' }, t('filesSize')), h('span', { key: 'm' }, t('filesMtime')),
                ]),
                ...listing.entries.map(entry => h('div', {
                  key: entry.name,
                  className: `dzr_row2${entry.type === 'directory' ? ' dir' : ''}${selected?.name === entry.name ? ' sel' : ''}`,
                  onClick: () => void open(entry),
                  title: entry.name,
                }, [
                  h('span', { key: 'g', className: `dzr_glyph${entry.type === 'directory' ? ' dir' : ''}` },
                    entry.type === 'directory'
                      ? h('svg', { width: 14, height: 14, viewBox: '0 0 16 16', fill: 'none', 'aria-hidden': 'true' },
                        h('path', { d: 'M1.5 4.5h4l1.2 1.6h7.8v6.4H1.5z', stroke: 'currentColor' }))
                      : h('svg', { width: 14, height: 14, viewBox: '0 0 16 16', fill: 'none', 'aria-hidden': 'true' },
                        h('path', { d: 'M3.5 1.8h5.2L12.5 5.6v8.6h-9z', stroke: 'currentColor' }))),
                  h('b', { key: 'n' }, entry.name),
                  h('span', { key: 's' }, entry.type === 'directory' ? '—' : size(entry.size)),
                  h('span', { key: 'm' }, when(entry.mtime)),
                ])),
                listing.entries.length === 0 ? h('div', { key: 'empty', className: 'dzr_pad' }, h('div', { className: 'dzr_note' }, t('filesEmpty'))) : null,
              ],
            ]),
            preview === undefined ? null : h('div', { key: 'prev', className: 'dzr_preview' }, [
              h('div', { key: 'head', className: 'dzr_prehead' }, [
                h('b', { key: 'n', style: { fontSize: '12.5px', minWidth: 0, overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap' } }, selected?.name ?? ''),
                preview?.size !== undefined ? h('span', { key: 'z', className: 'dzr_tag' }, size(preview.size)) : null,
                h('span', { key: 'sp', style: { marginLeft: 'auto' } }),
                selected === undefined ? null : h('a', {
                  key: 'dl', className: 'dzr_action', style: { textDecoration: 'none' },
                  href: `${API}/download?path=${encodeURIComponent(fullPath(selected.name))}`,
                }, t('filesDownload')),
                h('button', { key: 'x', type: 'button', className: 'dzr_action', onClick: () => { setPreview(undefined); setSelected(undefined) } }, t('filesClose')),
              ]),
              h('pre', { key: 'body', className: 'dzr_prebody' },
                preview?.loading === true ? t('filesLoading')
                  : preview?.ok === false ? (preview.error ?? t('filesBinary'))
                    : preview?.binary === true ? t('filesBinary')
                      : (preview?.text ?? '')),
            ]),
          ]),
        ])
      }
    }

    // ── right-column tab bodies ───────────────────────────────────────────────
    /**
     * Tab bodies for the two plugin-owned tools.
     *
     * Each shell fetches `/status` itself rather than receiving props: the slot
     * calls the component with the framework's own tab context, and keeping the
     * props contract empty means a slot-API change cannot silently break the
     * tool — the shell only ever depends on this plugin's own HTTP surface.
     */
    function makeVncTabShell(ctx, t) {
      // One stable component identity for the shell's whole life. Minting the
      // panel inside the render below would create a new React type on every
      // render, and a changed type means unmount + remount: the iframe reloads,
      // a live RFB session dies mid-handshake, and the form resets — exactly the
      // “连不上” symptom (diag logs hundreds of vnc-connector.ready an hour).
      const ConnectorPanel = makeVncConnectorPanel(ctx, t)
      return function VncTabShell() {
        const [status, setStatus] = useState(undefined)
        const [error, setError] = useState('')
        useEffect(() => {
          let alive = true
          api('/status').then(payload => { if (alive) setStatus(payload) })
            .catch(problem => { if (alive) setError(String(problem?.message ?? problem)) })
          return () => { alive = false }
        }, [])
        if (error !== '') {
          return h('div', { className: 'dzr_tabroot' }, h('div', { className: 'dzr_pad' }, h('div', { className: 'dzr_note dzr_warn' }, error)))
        }
        if (status === undefined) {
          return h('div', { className: 'dzr_tabroot' }, h('div', { className: 'dzr_pad' }, h('div', { className: 'dzr_note' }, t('filesLoading'))))
        }
        // The car's VNC desktop view IS the generic connector now: one VNC
        // surface serves the car (prefilled with the plugin's own bridge) and
        // any other ws://host:port/websockify endpoint. The connector fetches
        // /status itself, so the tab shell just hosts it.
        return h('div', { className: 'dzr_tabroot' }, h(ConnectorPanel))
      }
    }

    function makeFilesTabShell(ctx, t) {
      // Same stable-identity rule as the VNC shell above.
      const FilesView = makeFilesView(ctx, t)
      return function FilesTabShell() {
        const [status, setStatus] = useState(undefined)
        const [error, setError] = useState('')
        useEffect(() => {
          let alive = true
          api('/status').then(payload => { if (alive) setStatus(payload) })
            .catch(problem => { if (alive) setError(String(problem?.message ?? problem)) })
          return () => { alive = false }
        }, [])
        if (error !== '') {
          return h('div', { className: 'dzr_tabroot' }, h('div', { className: 'dzr_pad' }, h('div', { className: 'dzr_note dzr_warn' }, error)))
        }
        if (status === undefined) {
          return h('div', { className: 'dzr_tabroot' }, h('div', { className: 'dzr_pad' }, h('div', { className: 'dzr_note' }, t('filesLoading'))))
        }
        return h('div', { className: 'dzr_tabroot' }, h(FilesView, {
          settings: { ...status.settings, filesShortcuts: status.files?.shortcuts ?? [] },
        }))
      }
    }

    // ── the status float ─────────────────────────────────────────────────────
    /**
     * Remember the float's corner across reloads: it is a window the user
     * positioned once, and having it jump back to a default on every refresh is
     * the kind of small betrayal that makes people stop opening it.
     */
    function readFloatPos() {
      const fallback = () => {
        const width = typeof window !== 'undefined' && Number.isFinite(window.innerWidth) ? window.innerWidth : 1200
        const height = typeof window !== 'undefined' && Number.isFinite(window.innerHeight) ? window.innerHeight : 800
        return { left: Math.max(8, width - 300), top: Math.max(8, Math.round(height * 0.12)) }
      }
      try {
        const raw = typeof localStorage !== 'undefined' ? localStorage.getItem('dsh-dzrobot-ssh.float.pos') : null
        if (raw !== null) {
          const parsed = JSON.parse(raw)
          if (parsed !== null && typeof parsed === 'object' && Number.isFinite(parsed.left) && Number.isFinite(parsed.top)) return parsed
        }
      } catch { /* no storage in this context */ }
      return fallback()
    }

    function saveFloatPos(pos) {
      try { if (typeof localStorage !== 'undefined') localStorage.setItem('dsh-dzrobot-ssh.float.pos', JSON.stringify(pos)) } catch { /* ignore */ }
    }

    /** Whether the float was left open: reopening it on load is the point. */
    function floatOpenByDefault() {
      try { return typeof localStorage !== 'undefined' && localStorage.getItem('dsh-dzrobot-ssh.float.open') === '1' } catch { return false }
    }

    function rememberFloatOpen(open) {
      try { if (typeof localStorage !== 'undefined') localStorage.setItem('dsh-dzrobot-ssh.float.open', open ? '1' : '0') } catch { /* ignore */ }
    }

    /**
     * Human uptime from a reading that is already `age` milliseconds old, so
     * the seconds keep advancing between the 5-second polls.
     */
    function formatDuration(totalSeconds, t) {
      if (!Number.isFinite(totalSeconds) || totalSeconds < 0) return '—'
      const seconds = Math.floor(totalSeconds)
      const days = Math.floor(seconds / 86400)
      const hours = Math.floor((seconds % 86400) / 3600)
      const minutes = Math.floor((seconds % 3600) / 60)
      const rest = seconds % 60
      if (days > 0) return `${days}${t('unitDay')} ${hours}${t('unitHour')}`
      if (hours > 0) return `${hours}${t('unitHour')} ${minutes}${t('unitMin')}`
      if (minutes > 0) return `${minutes}${t('unitMin')} ${rest}${t('unitSec')}`
      return `${rest}${t('unitSec')}`
    }

    /** `hh:mm:ss` for the "updated at" line and the boot time. */
    function clockTime(ms) {
      const date = new Date(ms)
      const pad = value => String(value).padStart(2, '0')
      return `${pad(date.getHours())}:${pad(date.getMinutes())}:${pad(date.getSeconds())}`
    }

    /** CPU thresholds: the RK3588 runs warm past 75 % and saturates at 90. */
    const levelOf = (value, warm, hot) => (value >= hot ? 'hot' : value >= warm ? 'warm' : 'ok')

    /**
     * Battery thresholds run backwards: the pack is the one gauge where *low*
     * is the bad direction — ≤30 % warns, ≤15 % is critical. Returns the same
     * `ok|warm|hot` vocabulary the CSS levels use, so one rule set styles both
     * cells.
     */
    const batteryLevelOf = value => (value <= 15 ? 'hot' : value <= 30 ? 'warm' : 'ok')

    /**
     * Title-bar uptime as `m:ss` — minutes are the *total* elapsed minutes of
     * the car's own /proc/uptime (no hours: it is a stopwatch, not a clock,
     * so a two-day car reads 2880:00). Seconds are padded; `--:--` before
     * the first reading. The full "3天 4小时" text rides as the title's
     * tooltip via formatDuration.
     */
    function formatUptime(totalSeconds) {
      if (!Number.isFinite(totalSeconds) || totalSeconds < 0) return '--:--'
      const seconds = Math.floor(totalSeconds)
      const pad = value => String(value).padStart(2, '0')
      return `${Math.floor(seconds / 60)}:${pad(seconds % 60)}`
    }

    /**
     * How old a reading may be and still be called "the car's uptime".
     *
     * Three 5-second poll intervals plus slack: long enough to ride out one
     * slow SSH round trip, short enough that a stale anchor is never
     * extrapolated into a plausible-looking lie (after the car disconnected,
     * the header kept counting from the last good sample — the reported bug).
     */
    const UPTIME_STALE_MS = 15_000

    /**
     * The header's uptime verdict — pure, so the test pins the contract.
     *
     *   · `live`  — the car answered, the failure flag is clear and the
     *     reading is younger than UPTIME_STALE_MS: the value is the car's
     *     own `/proc/uptime` plus the seconds that have passed since that
     *     sample (`startedAt` prefers the car's `btime` boot instant when
     *     present, falling back to host-clock arithmetic).
     *   · `down`  — the last poll failed, or the reading is too old to
     *     claim: the header must say 小车未连接, not keep ticking.
     *   · `loading` — nothing tried yet: `--:--`.
     *
     * On reconnect the next good reading replaces everything, so all gauges
     * re-anchor on the car's numbers.
     */
    function uptimeView(reading, error, now) {
      const good = reading != null && reading.ok === true && Number.isFinite(reading.uptimeSec)
        && Number.isFinite(reading.at)
      const age = good ? Math.max(0, now - reading.at) : 0
      if (good && error === '' && age <= UPTIME_STALE_MS) {
        const startedAt = Number.isFinite(reading.bootEpoch)
          ? reading.bootEpoch * 1000
          : reading.at - reading.uptimeSec * 1000
        return { state: 'live', uptime: reading.uptimeSec + age / 1000, startedAt, age }
      }
      if (!good && error === '') return { state: 'loading' }
      return { state: 'down' }
    }

    /** Plot geometry in viewBox units; the box's inner width matches it 1:1,
        so tick labels draw at ~1:1 instead of being scaled with the frame. */
    const CHART = { w: 144, h: 78, l: 26, r: 141, t: 6, b: 56 }
    const X_HALF = 30 * 1000    // x axis: now ± 30 seconds
    const X_STEP = 5 * 1000     // a tick every 5 seconds

    /**
     * The plot's geometry, recomputed on every paint so both axes move with the
     * car rather than being pinned to whatever the first reading happened to
     * be:
     *
     *   x — now ± 30 s, a tick every 5 seconds, HH:MM:SS labels at −30 s,
     *       now and +30 s (the red dot lands on the centre line). Seconds
     *       are required in the labels: ±30 s around a whole minute would
     *       otherwise show the same HH:MM at centre and right end.
     *   y — battery and CPU are both percentages: current ±10 points, ticks
     *       every 5 %, clamped into 0..100 (a car sitting at 95 % has no
     *       +110 tick to draw).
     *
     * Returns the ranges, the tick lists with labels pre-formatted, the
     * polyline and the latest-value dot — plain numbers, so the tests assert
     * the scales directly and the renderer only draws what this says. Samples
     * outside the x window are dropped, not clamped: a squeezed line would lie
     * about the rate of change. `current` non-finite (no reading yet) answers
     * null, and the renderer draws an empty frame with a dashed baseline —
     * never a made-up curve.
     */
    function chartModel(kind, samples, now, current) {
      if (!Number.isFinite(current)) return null
      const lo = Math.max(0, current - 10)
      const hi = Math.min(100, current + 10)
      const step = 5
      const xOf = at => CHART.l + ((at - (now - X_HALF)) / (2 * X_HALF)) * (CHART.r - CHART.l)
      const yOf = value => CHART.b - ((value - lo) / (hi - lo)) * (CHART.b - CHART.t)
      const clamp = value => Math.min(hi, Math.max(lo, value))

      // Ticks anchored on the current value, so ±10/±5 land exactly on the
      // range the spec asks for; the clamp drops the one the 0..100 bound
      // pushed outside.
      const yTicks = []
      for (let index = -2; index <= 2; index += 1) {
        const value = current + index * step
        if (value < lo - 1e-9 || value > hi + 1e-9) continue
        yTicks.push({
          v: Math.round(value * 100) / 100,
          y: yOf(value),
          label: String(Math.round(value)),
        })
      }

      const xTicks = []
      const steps = Math.round((2 * X_HALF) / X_STEP)
      for (let index = 0; index <= steps; index += 1) {
        const at = now - X_HALF + index * X_STEP
        // Clock labels only at the ends and the centre (a label on every 5-s
        // tick would collide: ~9 units apart in a 144-unit box), and with
        // second precision so the three labels are distinguishable.
        const major = index === 0 || index === steps || index * X_STEP === X_HALF
        xTicks.push({ x: xOf(at), at, label: major ? clockTime(at) : null })
      }

      const usable = (Array.isArray(samples) ? samples : [])
        .filter(sample => Number.isFinite(sample?.at) && Number.isFinite(sample?.value)
          && sample.at >= now - X_HALF && sample.at <= now + X_HALF)
      const points = usable.length < 2
        ? ''
        : usable.map(sample => `${xOf(sample.at).toFixed(1)},${yOf(clamp(sample.value)).toFixed(1)}`).join(' ')
      const latest = usable.length > 0 ? usable[usable.length - 1] : { at: now, value: current }

      return {
        lo, hi, unit: '%', xTicks, yTicks, points,
        dot: { x: xOf(latest.at), y: yOf(clamp(latest.value)) },
      }
    }

    /**
     * The status float itself: a draggable card of the car's battery level and
     * CPU usage — two parallel cells, each a blue pair of boxes (名称 + 数值
     * over a curve plot with live axes: x = now±30 s ticked every 5 seconds,
     * y = the metric's own ±range). The uptime moved up to the header line
     * (`小车状态 | 开机时间：m:ss`) and no longer draws a curve.
     *
     * Registered on `shell.overlay` — the frame-wide floating layer above every
     * column — rather than as a tab, because the whole point is to keep
     * watching the car's battery while the app shows something else. Opening it
     * is a store flip, not a navigation: no Session is needed, so it works on a
     * fresh start before any conversation exists.
     *
     * Three clocks run while it is open: a 5-second poll (each reading is one
     * ssh round trip to the car, and one history sample — the ring keeps 160
     * ≈ 13 minutes of it; the axis only draws the last ±30 s), a
     * 1-second tick (uptime advances between polls, and the x axis slides with
     * it), and the drag itself. The first two stop when the float closes or
     * the page is hidden.
     *
     * The opacity slider is a persisted setting, not a local tweak: it writes
     * through to the Host's `/config` (debounced — one drag is dozens of change
     * events), and the value rides on the `--dzr-op` custom property so hover
     * can still override it; see the CSS comment for why inline `opacity`
     * would lock a dimmed card's own controls out of reach.
     */
    function makeFloatWindow(ctx, store, t) {
      return function FloatWindow() {
        const state = useStore(store)
        const open = state.floatOpen === true
        const [reading, setReading] = useState(undefined)
        const [error, setError] = useState('')
        const [history, setHistory] = useState([])
        const [pos, setPos] = useState(readFloatPos)
        const [now, setNow] = useState(() => Date.now())
        const drag = useRef(null)
        const opacitySave = useRef(null)

        const pull = useCallback(async () => {
          // A hidden page has no float to keep fresh; skip the round trip.
          if (typeof document !== 'undefined' && document.hidden) return
          try {
            const payload = await api('/metrics')
            if (payload.ok === true) {
              setReading(payload)
              setError('')
              // One sample per successful poll, stamped with the reading's own
              // time so the x axis plots against when it was taken (the host
              // clock, which is this machine's). The ring keeps the last 160
              // (≈13 min; the plot draws the last ±30 s of it); a failure
              // contributes nothing instead of poisoning the curve with 0.
              setHistory(list => [...list, {
                at: Number.isFinite(payload.at) ? payload.at : Date.now(),
                battery: Number.isFinite(payload.battery) ? payload.battery : null,
                cpu: Number.isFinite(payload.cpuPercent) ? payload.cpuPercent : null,
              }].slice(-160))
            } else setError(String(payload.detail ?? payload.state ?? 'error'))
          } catch (problem) {
            setError(String(problem?.message ?? problem))
          }
        }, [])

        useEffect(() => {
          if (!open) return undefined
          void pull()
          const timer = setInterval(() => { void pull() }, 5000)
          return () => clearInterval(timer)
        }, [open, pull])

        // The uptime gauge ticks between polls instead of jumping every 5s.
        useEffect(() => {
          if (!open) return undefined
          const timer = setInterval(() => setNow(Date.now()), 1000)
          return () => clearInterval(timer)
        }, [open])

        // Coming back to the page re-anchors at once: the stale bound would
        // otherwise hold the header on 小车未连接 for a full poll cycle even
        // though the car is right there.
        useEffect(() => {
          if (!open || typeof document === 'undefined' || typeof document.addEventListener !== 'function') return undefined
          const onVisible = () => { if (!document.hidden) void pull() }
          document.addEventListener('visibilitychange', onVisible)
          return () => document.removeEventListener('visibilitychange', onVisible)
        }, [open, pull])

        const startDrag = event => {
          // A press on a control is a click, not a grab. Without this guard the
          // header takes pointer capture for every press on the × button, the
          // browser then retargets the resulting click to the capturing header
          // itself, and the button's onClick never fires — precisely the
          // reported "关不掉". Buttons in the header therefore never start a drag.
          if (typeof event.target?.closest === 'function' && event.target.closest('button') !== null) return
          drag.current = { dx: event.clientX - pos.left, dy: event.clientY - pos.top }
          try { event.currentTarget.setPointerCapture?.(event.pointerId) } catch { /* no pointer capture */ }
        }
        const moveDrag = event => {
          if (drag.current === null) return
          const width = typeof window !== 'undefined' && Number.isFinite(window.innerWidth) ? window.innerWidth : 1024
          const height = typeof window !== 'undefined' && Number.isFinite(window.innerHeight) ? window.innerHeight : 768
          const left = Math.min(Math.max(4, event.clientX - drag.current.dx), Math.max(4, width - 120))
          const top = Math.min(Math.max(4, event.clientY - drag.current.dy), Math.max(4, height - 48))
          setPos({ left, top })
        }
        const endDrag = () => {
          if (drag.current === null) return
          drag.current = null
          saveFloatPos(pos)
        }

        const close = () => { rememberFloatOpen(false); store.set({ floatOpen: false }) }

        // Opacity: instant on screen (store → re-render), durable on disk
        // (debounced POST — a single slider drag fires dozens of change events).
        const opacityRaw = Number(state.floatOpacity)
        const opacityValue = Number.isFinite(opacityRaw) ? Math.min(100, Math.max(30, Math.round(opacityRaw))) : 100
        const setOpacity = event => {
          const value = Math.round(Number(event.target.value))
          if (!Number.isFinite(value)) return
          const clamped = Math.min(100, Math.max(30, value))
          store.set({ floatOpacity: clamped })
          if (opacitySave.current !== null) clearTimeout(opacitySave.current)
          opacitySave.current = setTimeout(() => {
            opacitySave.current = null
            void api('/config', { method: 'POST', body: JSON.stringify({ floatOpacity: clamped }) }).catch(() => {})
          }, 400)
        }

        if (!open) return null

        // One verdict drives every uptime decision: `live` is the car's own
        // /proc/uptime advanced to now; `down` is the disconnect state the
        // header must name — never a stale anchor kept ticking; `loading` is
        // the pre-first-poll placeholder.
        const up = uptimeView(reading, error, now)
        const fresh = up.state === 'live'
        const uptime = fresh ? up.uptime : undefined
        const startedAt = fresh ? up.startedAt : 0
        // Battery/CPU keep their last painted value while the error line below
        // names the outage; the next good poll re-anchors every gauge on the
        // car's own numbers.
        const hasReading = reading !== undefined && reading.ok === true
        const battery = hasReading && Number.isFinite(reading.battery) ? reading.battery : null
        const cpu = hasReading ? reading.cpuPercent : undefined

        /** One metric's samples so far: a failed poll contributes none. */
        const samplesOf = field => history
          .filter(sample => Number.isFinite(sample[field]))
          .map(sample => ({ at: sample.at, value: sample[field] }))

        // The plot: axes and ticks recomputed every paint from chartModel, the
        // curve from the samples, and the latest value as a red dot with dashed
        // cross-hairs to the axes. With no reading model is null and this is
        // an empty frame with a dashed baseline — never an invented curve.
        const chart = kind => {
          const model = chartModel(kind, samplesOf(kind), now, kind === 'battery' ? battery : cpu)
          const kids = []
          const axis = [
            h('line', { key: 'ax', x1: CHART.l, y1: CHART.t, x2: CHART.l, y2: CHART.b, className: 'dzr_faxis' }),
            h('line', { key: 'az', x1: CHART.l, y1: CHART.b, x2: CHART.r, y2: CHART.b, className: 'dzr_faxis' }),
          ]
          if (model === null) {
            kids.push(axis[0], axis[1])
            const mid = (CHART.t + CHART.b) / 2
            kids.push(h('line', { key: 'no', x1: CHART.l, y1: mid, x2: CHART.r, y2: mid, className: 'dzr_fno' }))
          } else {
            kids.push(axis[0], axis[1])
            for (const tick of model.yTicks) {
              kids.push(h('line', { key: `gy${tick.v}`, x1: CHART.l, y1: tick.y, x2: CHART.r, y2: tick.y, className: 'dzr_fgridline' }))
              kids.push(h('text', { key: `ly${tick.v}`, x: CHART.l - 3, y: tick.y + 2.2, className: 'dzr_flab', 'text-anchor': 'end' }, tick.label))
            }
            model.xTicks.forEach((tick, index) => {
              kids.push(h('line', { key: `tx${index}`, x1: tick.x, y1: CHART.b, x2: tick.x, y2: CHART.b + 3, className: 'dzr_ftick' }))
              if (tick.label !== null) {
                // The end labels hug their edge instead of centring, or they
                // would run past the viewBox and touch the box's border.
                const anchor = index === 0 ? 'start' : index === model.xTicks.length - 1 ? 'end' : 'middle'
                kids.push(h('text', { key: `lx${index}`, x: tick.x, y: CHART.b + 11, className: 'dzr_flab', 'text-anchor': anchor }, tick.label))
              }
            })
            if (model.points !== '') kids.push(h('polyline', { key: 'p', points: model.points }))
            kids.push(h('line', { key: 'cx', x1: model.dot.x, y1: model.dot.y, x2: model.dot.x, y2: CHART.b, className: 'dzr_fcross' }))
            kids.push(h('line', { key: 'cy', x1: CHART.l, y1: model.dot.y, x2: model.dot.x, y2: model.dot.y, className: 'dzr_fcross' }))
            kids.push(h('circle', { key: 'd', cx: model.dot.x, cy: model.dot.y, r: 2, className: 'dzr_fdot' }))
            const midY = (CHART.t + CHART.b) / 2
            kids.push(h('text', { key: 'yu', x: 7, y: midY, className: 'dzr_faxis-title', 'text-anchor': 'middle', transform: `rotate(-90 7 ${midY})` }, model.unit))
            kids.push(h('text', { key: 'xu', x: (CHART.l + CHART.r) / 2, y: CHART.h - 2, className: 'dzr_faxis-title', 'text-anchor': 'middle' }, t('floatAxisX')))
          }
          return h('svg', { key: 'c', className: 'dzr_fchart', viewBox: `0 0 ${CHART.w} ${CHART.h}`, 'aria-hidden': 'true' }, kids)
        }

        const number = (text, unit) => (text === undefined
          ? h('b', { key: 'v' }, '—')
          : h('b', { key: 'v' }, [h('span', { key: 'n' }, text), h('small', { key: 'u' }, unit)]))

        // One cell = two blue boxes: 名称 + 数值 on top, its curve plot below.
        // The extra context (the pack's voltage / why there is none, the load
        // average) rides as a tooltip on the cell rather than taking a line of
        // its own.
        const cell = (key, label, value, tip, level) => h('div', {
          key,
          className: 'dzr_fcell',
          'data-kind': key,
          ...level === undefined ? {} : { 'data-level': level },
          ...tip === '' ? {} : { title: tip },
        }, [
          h('div', { key: 'box', className: 'dzr_fbox' }, [h('span', { key: 'l' }, label), value]),
          h('div', { key: 'plot', className: 'dzr_fbox dzr_fplot' }, [chart(key)]),
        ])

        // Where the battery number comes from: the chassis topic (volts as
        // published), the voltage-derived fallback (12 V full / 10 V empty),
        // or nothing at all — in which case the tooltip says the stack is
        // down instead of the gauge inventing a 0 %.
        const battTip = () => {
          if (battery === null) return t('floatBattNone')
          const volts = Number.isFinite(reading.batteryVolt) ? reading.batteryVolt : null
          if (volts === null) return ''
          return reading.batteryNote === 'derived'
            ? t('floatBattDerived', { v: volts.toFixed(2) })
            : t('floatBattVolts', { v: volts.toFixed(2) })
        }

        return h('div', {
          className: 'dzr_float',
          style: { left: `${pos.left}px`, top: `${pos.top}px`, '--dzr-op': `${opacityValue / 100}` },
          'aria-label': t('floatTitle'),
        }, [
          h('div', {
            key: 'head',
            className: 'dzr_fhead',
            title: t('floatDrag'),
            onPointerDown: startDrag,
            onPointerMove: moveDrag,
            onPointerUp: endDrag,
            onPointerCancel: endDrag,
          }, [
            h('span', { key: 'g', className: 'dzr_icon' }, CarGlyph({ size: 14 })),
            // Header line: 名称 | 开机时间：m:ss — the uptime lives here now,
            // so the body can be the two charts. Hover still gives the full
            // detail the compact digits drop: "4分53秒 · 开机于 08:20".
            h('div', { key: 't', className: 'dzr_ftitle' }, [
              h('b', { key: 'n' }, t('floatTitle')),
              h('i', { key: 's', className: 'dzr_fsep' }, '|'),
              h('span', {
                key: 'u',
                className: 'dzr_fup',
                ...(fresh ? { title: `${formatDuration(uptime, t)} · ${t('floatSince', { time: clockTime(startedAt) })}` } : {}),
              }, `${t('floatUp')}：${up.state === 'down' ? t('floatDown') : formatUptime(uptime)}`),
            ]),
            h('button', {
              key: 'c', type: 'button', className: 'dzr_fbtn', title: t('floatClose'),
              // Belt and braces beside the startDrag guard: a press that starts
              // on the button never reaches the draggable header at all, so no
              // pointer capture can retarget its click.
              onPointerDown: event => event.stopPropagation(),
              onClick: close,
            }, '×'),
          ]),
          h('div', { key: 'body', className: 'dzr_fgrid' }, [
            cell('battery', t('floatBattery'),
              number(battery === null ? undefined : battery.toFixed(1), '%'),
              fresh || battery !== null ? battTip() : '',
              battery === null ? undefined : batteryLevelOf(battery)),
            cell('cpu', t('floatCpu'),
              number(cpu === undefined ? undefined : cpu.toFixed(1), '%'),
              fresh && Array.isArray(reading.load)
                ? t('floatLoad', { load: reading.load.map(value => value.toFixed(2)).join(' ') })
                : '',
              cpu === undefined ? undefined : levelOf(cpu, 75, 90)),
          ]),
          error !== '' ? h('div', { key: 'err', className: 'dzr_note dzr_warn dzr_ferr' }, t('floatError', { message: error })) : null,
          h('div', { key: 'foot', className: 'dzr_ffoot' }, [
            h('span', { key: 'sync', className: 'dzr_note' },
              reading === undefined ? t('floatLoading') : t('floatSync', { time: clockTime(reading.at) })),
            h('input', {
              key: 'op', type: 'range', className: 'dzr_fopacity',
              min: 30, max: 100, step: 5, value: opacityValue,
              title: t('fieldFloatOpacity'), 'aria-label': t('fieldFloatOpacity'),
              onChange: setOpacity,
            }),
            h('button', {
              key: 'refresh', type: 'button', className: 'dzr_fbtn', title: t('floatRefresh'),
              onClick: () => { setNow(Date.now()); void pull() },
            }, '↻'),
          ]),
        ])
      }
    }

    // ── the tool list ─────────────────────────────────────────────────────────
    /**
     * The panel's one table of contents.
     *
     * `kind` is the right-column tab type registered in `apply()` — except for
     * `terminal`, which reuses the stock type so the shell keeps owning the PTY,
     * and `float`, which is not a tab at all: it flips the store flag the
     * `shell.overlay` card renders from (no Session needed, so it works before
     * any conversation exists).
     * Adding a tool is: a row here, a tab-type registration there.
     */
    const TOOLS = [
      { id: 'terminal', kind: 'terminal', mode: 'terminal', titleKey: 'viewTerminal', descKey: 'terminalHint' },
      { id: 'files', kind: 'dzr-files', mode: 'tab', titleKey: 'viewFiles', descKey: 'filesToolDesc' },
      { id: 'vnc', kind: 'dzr-vnc', mode: 'tab', titleKey: 'viewVnc', descKey: 'vncToolDesc' },
      { id: 'float', kind: null, mode: 'float', titleKey: 'viewFloat', descKey: 'floatToolDesc' },
      // No tab: the bench is its own document on this plugin's http origin (it runs
      // long child processes and wants a full window), so `mode: 'external'` just
      // opens `${origin}/api/dzrobot-ssh/sim.html` in a new tab.
      { id: 'sim', kind: null, mode: 'external', titleKey: 'viewSim', descKey: 'simToolDesc' },
    ]

    /** Compact glyphs for the list rows. */
    function ToolGlyph({ id }) {
      const paths = {
        // prompt: `>_`
        terminal: [h('path', { key: 'a', d: 'M3 4.5L7 8L3 11.5', stroke: 'currentColor', strokeLinecap: 'round' }), h('path', { key: 'b', d: 'M9 11.5H13', stroke: 'currentColor', strokeLinecap: 'round' })],
        // folder
        files: [h('path', { key: 'a', d: 'M1.8 4.2h4.1l1.2 1.6h7.1v6.4H1.8z', stroke: 'currentColor', strokeLinejoin: 'round' })],
        // desktop screen with stand
        vnc: [
          h('rect', { key: 'a', x: '1.8', y: '3', width: '12.4', height: '7.6', rx: '1.2', stroke: 'currentColor' }),
          h('path', { key: 'b', d: 'M6 13.6h4M8 10.6v3', stroke: 'currentColor', strokeLinecap: 'round' }),
        ],
        // a battery: the cell with its terminal (the float's headline number)
        float: [
          h('rect', { key: 'a', x: '1.8', y: '4.6', width: '10.6', height: '6.8', rx: '1.6', stroke: 'currentColor' }),
          h('path', { key: 'b', d: 'M14.2 6.8v2.4', stroke: 'currentColor', strokeLinecap: 'round' }),
          h('path', { key: 'c', d: 'M4 6.8h4.2v2.4H4z', fill: 'currentColor', stroke: 'none' }),
        ],
        // a lab flask: the bench runs experiments, it does not drive anything
        sim: [
          h('path', { key: 'a', d: 'M6.2 1.8h3.6', stroke: 'currentColor', strokeLinecap: 'round' }),
          h('path', { key: 'b', d: 'M6.9 1.8v3.4L3.6 11.4a1.5 1.5 0 0 0 1.3 2.3h6.2a1.5 1.5 0 0 0 1.3-2.3L9.1 5.2V1.8', stroke: 'currentColor', strokeLinejoin: 'round' }),
          h('path', { key: 'c', d: 'M5.1 10.4h5.8', stroke: 'currentColor', strokeLinecap: 'round' }),
        ],
      }
      return h('svg', { width: 18, height: 18, viewBox: '0 0 16 16', fill: 'none', 'aria-hidden': 'true' }, paths[id] ?? [])
    }

    /**
     * The panel: a launcher list. Every tool opens as a tab in the right column,
     * which belongs to the Conversation — so opening one switches the main view
     * there (the note under the list says so before the first click).
     */
    function makeCarPanel(ctx, store, t) {
      return function CarPanel() {
        const state = useStore(store)
        const [command, setCommand] = useState('')
        const [busy, setBusy] = useState('')
        const [rowMsg, setRowMsg] = useState({})

        useEffect(() => {
          api('/status').then(payload => {
            setCommand(String(payload.command ?? ''))
            store.set({
              probe: payload.probe,
              car: payload.car,
              filesShortcuts: payload.files?.shortcuts ?? [],
              // The overlay card needs the persisted opacity; the panel is one
              // of the two seats that read /status on mount.
              floatOpacity: payload.settings?.floatOpacity,
              // The sim bench is opened as its own http document, and the page
              // that hosts this panel runs under `dsh-app://` — so the new tab's
              // URL has to be built from the origin the request arrived on, not
              // from `location.href`. Kept in the store because the click that
              // needs it happens long after this fetch.
              origin: payload.origin,
            })
            diag('panel.status', { origin: payload.origin, vnc: payload.vnc, root: payload.files?.root })
          }).catch(problem => {
            diag('panel.status.error', { message: String(problem?.message ?? problem) })
          })
        }, [])

        /** One shared explanation for every tool's failure. */
        const explain = result => {
          if (result.reason === 'no-session') return t('openTerminalFirst')
          if (result.reason === 'needs-password') return t('needPassword')
          if (result.reason === 'open-failed') return result.detail ?? String(result.reason)
          if (result.detail) return t('notReady', { detail: result.detail })
          return t('notReady', { detail: result.reason })
        }

        const open = tool => async () => {
          setRowMsg(current => ({ ...current, [tool.id]: '' }))
          if (tool.mode === 'float') {
            // No session, no tab, no navigation: toggling the store flag is the
            // whole protocol — the `shell.overlay` card follows it from there.
            const next = !(store.get().floatOpen === true)
            rememberFloatOpen(next)
            store.set({ floatOpen: next })
            diag('tool.float', { open: next })
            return
          }
          if (tool.mode === 'external') {
            // A standalone document on this plugin's http origin. A popup blocker
            // is the one failure this path can hit, and it must not stay silent:
            // the manual URL goes into the row so the user can still get there.
            const origin = String(store.get().origin ?? '')
            const path = '/api/dzrobot-ssh/sim.html'
            const url = origin === '' ? path : `${origin}${path}`
            let opened = null
            if (origin !== '') {
              try { opened = window.open(url, '_blank') } catch { opened = null }
            }
            const ok = opened !== null && opened !== undefined
            diag('tool.external', { id: tool.id, origin, ok })
            if (!ok) setRowMsg(current => ({ ...current, [tool.id]: t('simOpenHint', { url }) }))
            return
          }
          setBusy(tool.id)
          try {
            const result = tool.mode === 'terminal'
              ? await connect(ctx, document.body, () => {})
              : await openToolTab(ctx, tool.kind, document.body)
            diag('tool.open', { id: tool.id, ok: result.ok, reason: result.reason })
            if (!result.ok) setRowMsg(current => ({ ...current, [tool.id]: explain(result) }))
          } catch (problem) {
            diag('tool.open', { id: tool.id, error: String(problem?.message ?? problem) })
            setRowMsg(current => ({ ...current, [tool.id]: String(problem?.message ?? problem) }))
          } finally {
            setBusy('')
          }
        }

        return h('div', { className: 'dzr_panel' }, [
          h('div', { key: 'head', className: 'dzr_phead' }, [
            h('span', { key: 'title', className: 'dzr_ptitle' }, [
              h('span', { key: 'g', className: 'dzr_icon' }, [CarGlyph({ size: 18 }), h('span', { key: 'd', className: 'dzr_dot', 'data-state': state.probe?.state ?? 'idle' })]),
              t('panel'),
            ]),
            h('span', {
              key: 'probe', className: `dzr_note${state.probe?.state === 'ready' ? '' : ' dzr_warn'}`,
              title: state.probe?.target ?? '',
            }, state.probe === undefined ? t('neverProbed') : t(STATE_KEY[state.probe.state] ?? 'stateError')),
            h('span', { key: 'sp', style: { marginLeft: 'auto' } }),
          ]),
          h('div', { key: 'list', className: 'dzr_pad' }, [
            h('div', { key: 'note', className: 'dzr_note' }, t('toolNote')),
            ...TOOLS.map(tool => h('div', { key: tool.id, className: 'dzr_toolrow' }, [
              h('span', { key: 'g', className: 'dzr_toolglyph' }, ToolGlyph({ id: tool.id })),
              h('div', { key: 'info', className: 'dzr_toolinfo' }, [
                h('div', { key: 't', className: 'dzr_tooltitle' }, t(tool.titleKey)),
                h('div', { key: 'd', className: `dzr_tooldesc${tool.id === 'terminal' && command !== '' ? ' dzr_mono' : ''}` },
                  tool.id === 'terminal' && command !== '' ? command : t(tool.descKey)),
                rowMsg[tool.id] ? h('div', { key: 'm', className: 'dzr_note dzr_warn' }, rowMsg[tool.id]) : null,
              ]),
              h('button', {
                key: 'b', type: 'button',
                className: `dzr_action${tool.mode === 'float' && state.floatOpen === true ? '' : ' primary'}`,
                disabled: busy !== '', onClick: open(tool),
              }, busy === tool.id ? '…' : (tool.mode === 'float' && state.floatOpen === true ? t('toolClose') : t('toolOpen'))),
            ])),
          ]),
        ])
      }
    }

    // ── the generic VNC connector ─────────────────────────────────────────────
    /**
     * Which runtime shell this plugin's client code is running in.
     *
     * Electron and Tauri both embed web content in WebViews, but a plugin's
     * client half runs *inside* the app's renderer, where the only embedding
     * primitive that behaves identically everywhere is the iframe. Electron's
     * `<webview>` tag is deliberately not used: it only exists when webviewTag
     * is enabled, and it wires guest→embedder messaging through ipc instead of
     * DOM postMessage — so it would add a mode whose control loop half-fails
     * silently. The honest mapping, used by the panel:
     *
     *   · Electron/Tauri: the connector document embedded in the app's own web
     *     content (iframe), or in a separate real WebView window (window.open).
     *   · Browser plugin: the same document in an iframe or a new tab.
     *
     * The badge tells the user which of these applies.
     */
    function detectRuntime() {
      if (typeof window === 'undefined') return 'browser'
      try {
        if (window.__TAURI__ !== undefined || window.__TAURI_INTERNALS__ !== undefined) return 'tauri'
      } catch { /* window props can be access-restricted */ }
      try {
        if (/\bElectron\//.test(String(navigator?.userAgent ?? ''))) return 'electron'
        if (typeof window.process === 'object' && window.process !== null && String(window.process.type) === 'renderer') return 'electron'
      } catch { /* same */ }
      return 'browser'
    }

    const CONNECTOR_ID = 'dzrobot-vnc-connector'
    const CONNECTOR_STORE_KEY = 'dsh-dzrobot-ssh/vnc-connector'
    const VNC_RESOLUTIONS = [
      { id: 'auto' },
      { id: '1024x768' }, { id: '1280x720' }, { id: '1366x768' },
      { id: '1600x900' }, { id: '1920x1080' }, { id: '2560x1440' },
    ]

    /**
     * The connector's config, normalized from whatever was persisted.
     *
     * The password is persisted too, on purpose and with eyes open: the car
     * plugin already keeps its VNC password in the Host's settings file, and a
     * connector whose password evaporates on every reload is unusable. It lives
     * in localStorage of the app page only, never in a URL.
     */
    function normalizeConnectorConfig(raw) {
      const value = raw !== null && typeof raw === 'object' ? raw : {}
      const text = key => (typeof value[key] === 'string' ? value[key] : '')
      const host = text('host')
      const port = text('port')
      const path = text('path')
      return {
        scheme: value.scheme === 'wss' ? 'wss' : 'ws',
        host,
        port: port === '' ? '6080' : port,
        path: path === '' ? 'websockify' : path,
        password: text('password'),
        resolution: VNC_RESOLUTIONS.some(item => item.id === value.resolution) ? value.resolution : 'auto',
        scale: value.scale !== false,
      }
    }

    function loadConnectorConfig() {
      let saved = {}
      try {
        const text = typeof localStorage === 'undefined' ? null : localStorage.getItem(CONNECTOR_STORE_KEY)
        if (text !== null && text !== '') saved = JSON.parse(text)
      } catch { /* corrupt or unavailable */ }
      return normalizeConnectorConfig(saved)
    }

    /** True only when the user already saved a config (first run = false). */
    function hasConnectorConfig() {
      try {
        return typeof localStorage === 'undefined' ? true : localStorage.getItem(CONNECTOR_STORE_KEY) !== null
      } catch { return true }
    }

    /**
     * The car's own bridge expressed as a connector config: the plugin's
     * WebSocket upgrade at ws://<served-origin>/api/dzrobot-ssh/vnc, which
     * tunnels straight to the car's 5901 and needs no websockify on the car.
     *
     * One helper, two callers — the first-run prefill and the “car bridge”
     * button — so the two can never drift apart. Throws on an unparseable
     * origin; callers decide what to show.
     */
    function bridgeConnectorConfig(origin, base, carPassword) {
      const url = new URL(origin)
      return {
        ...base,
        scheme: url.protocol === 'https:' ? 'wss' : 'ws',
        host: url.hostname,
        port: url.port === '' ? (url.protocol === 'https:' ? '443' : '80') : url.port,
        path: 'api/dzrobot-ssh/vnc',
        password: base.password === '' ? carPassword : base.password,
      }
    }

    /** Monitor glyph for the connector's header and sidebar icon. */
    function ConnectorGlyph({ size = 16 }) {
      return h('svg', {
        width: size, height: size, viewBox: '0 0 16 16', fill: 'none', 'aria-hidden': 'true',
      }, [
        h('rect', { key: 'm', x: '1.8', y: '3', width: '12.4', height: '7.6', rx: '1.2', stroke: 'currentColor' }),
        h('path', { key: 's', d: 'M6 13.6h4M8 10.6v3', stroke: 'currentColor', strokeLinecap: 'round' }),
        h('circle', { key: 'g', cx: '8', cy: '6.8', r: '1.6', stroke: 'currentColor' }),
      ])
    }

    /**
     * The generic VNC connector panel.
     *
     * Everything RFB lives in the embedded `vnc-connect.html`; this panel owns
     * the form, the status line and the buttons. The same document opens in a
     * real window/tab for the standalone flow, which is why the frame always
     * gets the full config on connect — the two UIs stay interchangeable.
     */
    function makeVncConnectorPanel(ctx, t) {
      return function VncConnectorPanel() {
        const [origin, setOrigin] = useState('')
        const [config, setConfig] = useState(loadConnectorConfig)
        const [phase, setPhase] = useState('idle')
        const [message, setMessage] = useState('')
        const [desktop, setDesktop] = useState('')
        const [notice, setNotice] = useState('')
        const [runtimeKind] = useState(detectRuntime)
        const frameRef = useRef(null)
        const ready = useRef(false)
        const pending = useRef([])
        const userStopped = useRef(false)
        const configRef = useRef(config)
        // The car's saved VNC password, kept aside so the “car bridge” button can
        // refill an emptied field without another round trip to /status.
        const carPassword = useRef('')
        useEffect(() => { configRef.current = config }, [config])

        // Declared before the origin effect below: that effect prefills the
        // car bridge on first run and calls save inside its async callback.
        const save = useCallback(next => {
          setConfig(next)
          try { localStorage.setItem(CONNECTOR_STORE_KEY, JSON.stringify(next)) } catch { /* unavailable */ }
        }, [])

        useEffect(() => {
          let alive = true
          api('/status').then(payload => {
            if (!alive) return
            const servedOrigin = String(payload.origin ?? '')
            setOrigin(servedOrigin)
            carPassword.current = String(payload.settings?.vncPassword ?? '')
            // This panel replaced the car's own VNC view, so on first use the
            // form starts on the car's bridge — the plugin's own WebSocket
            // upgrade at ws://<served-origin>/api/dzrobot-ssh/vnc — instead of an
            // empty form. Once the user edits anything, their values persist and
            // win over this prefill.
            if (servedOrigin !== '' && !hasConnectorConfig()) {
              try { save(bridgeConnectorConfig(servedOrigin, configRef.current, carPassword.current)) }
              catch { /* unparseable origin: leave the form as-is */ }
            }
            diag('vnc-connector.origin', { origin: payload.origin })
          }).catch(problem => {
            diag('vnc-connector.origin.error', { message: String(problem?.message ?? problem) })
          })
          return () => { alive = false }
        }, [save])

        const patch = useCallback((key, value) => save({ ...configRef.current, [key]: value }), [save])

        /**
         * Send one command to the frame, queueing until it reports `ready` —
         * a click before the frame's module script ran would otherwise vanish.
         */
        const post = useCallback(payload => {
          const packet = { source: 'dzrobot-panel', ...payload }
          if (!ready.current) { pending.current.push(packet); return }
          try { frameRef.current?.contentWindow?.postMessage(packet, '*') } catch { /* frame not ready */ }
        }, [])

        const postConnect = useCallback(() => {
          post({ type: 'connect', config: configRef.current })
        }, [post])

        useEffect(() => {
          const onMessage = event => {
            const data = event.data
            if (data === null || typeof data !== 'object' || data.source !== 'dzrobot-vnc-connect') return
            if (data.type === 'ready') {
              // The frame just loaded: flush whatever was queued before it could
              // listen (an early click), otherwise stay idle — dialing is explicit.
              ready.current = true
              const queued = pending.current
              pending.current = []
              for (const packet of queued) {
                try { frameRef.current?.contentWindow?.postMessage(packet, '*') } catch { /* gone */ }
              }
            } else if (data.type === 'connecting') { setPhase('connecting'); setMessage(''); setNotice('') }
            else if (data.type === 'connected') { setPhase('connected'); setDesktop(String(data.desktop ?? '')); setMessage(''); setNotice('') }
            else if (data.type === 'credentials') { setPhase('auth'); setMessage(t('connNeedPassword')) }
            else if (data.type === 'credentials-sent') { setPhase('connecting'); setMessage('') }
            else if (data.type === 'disconnected') {
              if (userStopped.current) {
                // An explicit 断开 is not an error.
                userStopped.current = false
                setPhase('idle')
                setMessage('')
              } else {
                setPhase('disconnected')
                if (data.clean !== false) setMessage('')
                else if (data.closeReason) setMessage(String(data.closeReason))
                else if (data.closeCode !== undefined) setMessage(`${t('connDisconnected')}（WebSocket code ${data.closeCode}）`)
                else setMessage(String(data.reason ?? '') || t('connDisconnected'))
              }
            } else if (data.type === 'failed') {
              setPhase('failed')
              setMessage(String(data.message ?? '失败'))
              if (data.blacklisted === true) setNotice(t('connBlacklisted'))
            }
            // Full wire evidence, same convention as the car VNC: noVNC's own
            // events are near-empty, so the frame's socket log is what makes a
            // report diagnosable after the fact.
            diag(`vnc-connector.${String(data.type ?? 'event')}`, {
              message: data.message, clean: data.clean,
              closeCode: data.closeCode, closeReason: data.closeReason,
              href: data.href, ws: data.ws,
            })
          }
          window.addEventListener('message', onMessage)
          return () => window.removeEventListener('message', onMessage)
        }, [t])

        const connect = useCallback(() => {
          userStopped.current = false
          setPhase('connecting')
          setMessage('')
          setNotice('')
          postConnect()
        }, [postConnect])

        const stop = useCallback(() => {
          userStopped.current = true
          post({ type: 'disconnect' })
          setPhase('idle')
          setMessage('')
        }, [post])

        /**
         * Restore the car's bridge as the target, in one click.
         *
         * This is the escape hatch for the failure mode that actually happens:
         * the user follows the generic instructions, points the form at a
         * websockify that was never installed, gets 1006, and has no way back
         * except retyping four fields from memory. A live session is torn down
         * explicitly rather than left running behind a form that no longer
         * describes it.
         */
        const useBridge = useCallback(() => {
          if (origin === '') { setNotice(t('vncNoOrigin')); return }
          let next
          try { next = bridgeConnectorConfig(origin, configRef.current, carPassword.current) }
          catch { setNotice(t('vncNoOrigin')); return }
          if (phase === 'connected' || phase === 'connecting' || phase === 'auth') {
            userStopped.current = true
            post({ type: 'disconnect' })
          }
          save(next)
          setPhase('idle')
          setMessage('')
          setNotice(t('connCarBridgeDone'))
        }, [origin, phase, post, save, t])

        const toggleScale = useCallback(() => {
          const next = !configRef.current.scale
          save({ ...configRef.current, scale: next })
          post({ type: 'scale', value: next })
        }, [post, save])

        const changeResolution = useCallback(value => {
          save({ ...configRef.current, resolution: value })
          post({ type: 'resolution', value })
        }, [post, save])

        const fullscreen = useCallback(() => {
          setNotice('')
          const element = frameRef.current
          if (element === null) return
          try {
            const request = element.requestFullscreen?.()
            if (request === undefined) throw new Error('no requestFullscreen')
            if (typeof request.catch === 'function') {
              request.catch(() => setNotice(t('connFullscreenFailed')))
            }
          } catch {
            setNotice(t('connFullscreenFailed'))
          }
        }, [t])

        const openStandalone = useCallback(() => {
          if (origin === '') { setNotice(t('vncNoOrigin')); return }
          // Non-secret fields ride the URL; the password stays in this panel.
          // The standalone document asks for it again when the server requires it.
          const params = new URLSearchParams({
            scheme: configRef.current.scheme,
            host: configRef.current.host,
            port: configRef.current.port,
            path: configRef.current.path,
            resolution: configRef.current.resolution,
          })
          try { window.open(`${origin}/api/dzrobot-ssh/vnc-connect.html?${params.toString()}`, '_blank') }
          catch { setNotice(t('vncNoOrigin')) }
        }, [origin, t])

        const src = origin === '' ? '' : `${origin}/api/dzrobot-ssh/vnc-connect.html?embedded=1`
        const targetLabel = `${config.scheme}://${config.host}${config.port === '' ? '' : `:${config.port}`}${config.path === '' ? '' : `/${config.path}`}`
        // A bare 1006 is unreadable, and it is the *normal* outcome of pointing a
        // generic connector at something that was never installed. Say what it
        // means, and which of the two targets the user is actually on.
        const onBridge = config.path === 'api/dzrobot-ssh/vnc'
        const failureHint = phase !== 'failed' && phase !== 'disconnected'
          ? ''
          : onBridge ? t('connHintBridgeFailed') : t('connHintNoListener')
        const runtimeLabel = { electron: t('connRuntimeElectron'), tauri: t('connRuntimeTauri'), browser: t('connRuntimeBrowser') }[runtimeKind]

        const statusText = {
          idle: t('connIdle'),
          loading: t('connIdle'),
          connecting: t('connConnecting'),
          auth: t('connWaitingPassword'),
          connected: `${t('connConnected')}${desktop === '' ? '' : ` · ${desktop}`}`,
          failed: message === '' ? t('connFailed') : message,
          disconnected: message || t('connDisconnected'),
        }[phase] ?? t('connIdle')

        const overlay = phase !== 'connected' && src !== ''
          ? h('div', { key: 'overlay', className: 'dzr_center' }, [
            h('div', { key: 'idle', className: 'dzr_herocard' }, [
              h('div', { key: 'g', className: 'dzr_icon' }, ConnectorGlyph({ size: 30 })),
              h('h3', { key: 't' },
                phase === 'failed' ? t('connFailed')
                  : phase === 'connecting' ? t('connConnecting')
                    : phase === 'auth' ? t('connWaitingPassword')
                      : phase === 'disconnected' ? t('connDisconnected')
                        : t('connIdleTitle')),
              h('p', { key: 's' },
                phase === 'failed' || phase === 'disconnected' || (phase === 'auth' && message !== '')
                  ? (message || t('connIdleHint'))
                  : phase === 'connecting'
                    ? targetLabel
                    : t('connIdleHint')),
              notice !== '' ? h('p', { key: 'n', className: 'dzr_warn' }, notice) : null,
              failureHint !== '' ? h('p', { key: 'h' }, failureHint) : null,
              phase === 'connecting'
                ? h('div', { key: 'sp', className: 'dzr_spin2', 'aria-hidden': 'true' })
                : h('div', { key: 'row', style: { display: 'flex', gap: 8 } }, [
                  h('button', {
                    key: 'b', type: 'button', className: 'dzr_action primary',
                    disabled: src === '', onClick: connect,
                  }, phase === 'disconnected' || phase === 'failed' ? t('connReconnect') : t('connConnect')),
                  // The way out of the dead end: one click back to a target that
                  // needs no setup on the car whatsoever.
                  failureHint !== '' && !onBridge && origin !== ''
                    ? h('button', {
                      key: 'cb', type: 'button', className: 'dzr_action', onClick: useBridge,
                    }, t('connCarBridge'))
                    : null,
                ]),
            ]),
          ])
          : null

        return h('div', { className: 'dzr_panel' }, [
          h('div', { key: 'head', className: 'dzr_phead' }, [
            h('span', { key: 'title', className: 'dzr_ptitle' }, [
              h('span', { key: 'g', className: 'dzr_icon' }, ConnectorGlyph({ size: 18 })),
              t('vncConnectorTitle'),
            ]),
            h('span', { key: 'runtime', className: 'dzr_tag' }, runtimeLabel),
            h('span', { key: 'sp', style: { marginLeft: 'auto' } }),
            h('span', { key: 'pill', className: 'dzr_pill', 'data-phase': phase, title: statusText }, [
              h('i', { key: 'd' }), h('span', { key: 't' }, statusText),
            ]),
          ]),
          h('div', { key: 'body', className: 'dzr_pbody' }, [
            h('div', { key: 'bar', className: 'dzr_toolbar' }, [
              h('label', { key: 'scheme', className: 'dzr_field', style: { width: 64 } }, [
                h('span', { key: 'l' }, t('connFieldScheme')),
                h('select', {
                  key: 'i', className: 'dzr_input', value: config.scheme,
                  onChange: event => patch('scheme', event.target.value),
                }, [
                  h('option', { key: 'ws', value: 'ws' }, 'ws'),
                  h('option', { key: 'wss', value: 'wss' }, 'wss'),
                ]),
              ]),
              h('label', { key: 'host', className: 'dzr_field', style: { flex: '1 1 160px' } }, [
                h('span', { key: 'l' }, t('connFieldHost')),
                h('input', {
                  key: 'i', className: 'dzr_input', value: config.host, placeholder: 'ubuntu-server 或 127.0.0.1',
                  onChange: event => patch('host', event.target.value),
                }),
              ]),
              h('label', { key: 'port', className: 'dzr_field', style: { width: 90 } }, [
                h('span', { key: 'l' }, t('connFieldPort')),
                h('input', {
                  key: 'i', className: 'dzr_input', inputMode: 'numeric', value: config.port,
                  onChange: event => patch('port', event.target.value),
                }),
              ]),
              h('label', { key: 'path', className: 'dzr_field', style: { width: 120 } }, [
                h('span', { key: 'l' }, t('connFieldPath')),
                h('input', {
                  key: 'i', className: 'dzr_input', value: config.path,
                  onChange: event => patch('path', event.target.value),
                }),
              ]),
              h('label', { key: 'pw', className: 'dzr_field', style: { width: 130 } }, [
                h('span', { key: 'l' }, t('connFieldPassword')),
                h('input', {
                  key: 'i', className: 'dzr_input', type: 'password', value: config.password,
                  onChange: event => patch('password', event.target.value),
                }),
              ]),
              h('label', { key: 'res', className: 'dzr_field', style: { width: 150 } }, [
                h('span', { key: 'l' }, t('connFieldResolution')),
                h('select', {
                  key: 'i', className: 'dzr_input', value: config.resolution,
                  onChange: event => changeResolution(event.target.value),
                }, VNC_RESOLUTIONS.map(item => h('option', {
                  key: item.id, value: item.id,
                }, item.id === 'auto' ? t('connResolutionAuto') : item.id))),
              ]),
              h('button', {
                key: 'bridge', type: 'button', className: 'dzr_action',
                title: t('connCarBridgeHint'), disabled: origin === '', onClick: useBridge,
              }, t('connCarBridge')),
              h('button', {
                key: 'go', type: 'button', className: 'dzr_action primary',
                // Deliberately enabled in the 'auth' phase: that is exactly when
                // the user has just corrected the password field and needs to
                // reconnect — disabling it there would trap them with no exit.
                disabled: src === '' || phase === 'connecting',
                onClick: connect,
              }, phase === 'connected' || phase === 'auth' ? t('connReconnect') : t('connConnect')),
              phase === 'connected' || phase === 'connecting' || phase === 'auth'
                ? h('button', { key: 'stop', type: 'button', className: 'dzr_action', onClick: stop }, t('connDisconnect'))
                : null,
              h('button', {
                key: 'fs', type: 'button', className: 'dzr_action', title: t('connFullscreen'),
                disabled: phase !== 'connected', onClick: fullscreen,
              }, t('connFullscreen')),
              h('label', {
                key: 'scale', className: 'dzr_action', title: t('connScaleHint'),
                style: { display: 'inline-flex', alignItems: 'center', gap: 6, cursor: 'pointer' },
              }, [
                h('input', {
                  key: 'c', type: 'checkbox', checked: config.scale,
                  onChange: toggleScale,
                }),
                t('connScale'),
              ]),
              h('button', {
                key: 'tab', type: 'button', className: 'dzr_action', title: t('connNewTabHint'),
                disabled: origin === '', onClick: openStandalone,
              }, runtimeKind === 'browser' ? t('connNewTab') : t('connNewWindow')),
            ]),
            h('div', { key: 'stage', className: 'dzr_screen' }, [
              src === ''
                ? h('div', { key: 'noorigin', className: 'dzr_center' }, h('div', { className: 'dzr_herocard' }, [
                  h('h3', { key: 't' }, t('vncNoOrigin')),
                ]))
                : h('iframe', {
                  key: 'frame',
                  ref: frameRef,
                  src,
                  title: t('vncConnectorTitle'),
                  // Same origin as the served document, and the panel needs the
                  // desktop to be able to go fullscreen from its own button.
                  allow: 'fullscreen; clipboard-read; clipboard-write',
                  style: { width: '100%', height: '100%', border: '0', display: 'block', background: '#000' },
                }),
              overlay,
            ]),
          ]),
        ])
      }
    }

    // ── registration ──────────────────────────────────────────────────────────
    function apply(ctx) {
      installStyles()
      const t = ctx.locale.bind(NS)
      ctx.effect(() => ctx.locale.register(NS, { zh: DICT.zh, en: DICT.en }), 'dzrobot-ssh: dictionaries')
      const store = createStore()
      const locale = localeTag(ctx)

      diag('client.load', { services: surveyServices(ctx), locale })

      ctx.slots.inject('settings.section', () => ctx.slots.register({
        name: 'settings.section',
        id: 'dzrobot-ssh',
        order: 40,
        label: () => t('nav'),
        locale: NS,
      }, makeSettingsPage(ctx, store, t)))

      // The panel itself: a panellist id addresses the main-column seat with the
      // same key, so one id in both places is what makes the sidebar icon select
      // this panel.
      ctx.slots.inject('sidebar.panellist', () => ctx.slots.register({
        name: 'sidebar.panellist',
        id: PANEL_ID,
        order: 30,
        label: () => t('panel'),
        locale: NS,
      }, () => CarGlyph({ size: 18 })))

      ctx.slots.inject('main', () => ctx.slots.register({
        name: 'main',
        key: PANEL_ID,
        label: () => t('panel'),
        locale: NS,
      }, makeCarPanel(ctx, store, t)))

      // The generic VNC connector is not a separate panel anymore: it replaced
      // the car's VNC desktop view (see makeVncTabShell). One VNC surface total.

      // ── the two plugin-owned right-column tab types ─────────────────────────
      // The terminal reuses the stock kind (the shell owns the PTY); these two
      // are ours. `slot` keys equal the type ids — TabSlot dispatches a tab body
      // through `entryKey: definition.id`.
      const tabTypes = ctx.get('sidebarRightTabs')
      // The tab bodies get one stable component identity per client load. The
      // effects below can re-run (slot scope revisions), and re-registering a
      // freshly minted function each time would remount the open tab and kill a
      // live VNC session — so the shells are created once, here.
      const VncTabShell = makeVncTabShell(ctx, t)
      const FilesTabShell = makeFilesTabShell(ctx, t)
      if (tabTypes !== undefined && typeof tabTypes.register === 'function') {
        ctx.effect(() => tabTypes.register({
          id: 'dsh-dzrobot-ssh/vnc',
          kind: 'dzr-vnc',
          // Fixed contentId: a second click focuses the open tool instead of
          // stacking copies (placeTab reuses pageAddress for non-multiple types).
          multiple: false,
          // The RFB connection lives in the frame: unmounting it would drop the
          // desktop on every tab switch, so the body stays mounted.
          keepMounted: true,
          title: () => t('viewVnc'),
        }), 'dzrobot-ssh: tab type vnc')

        ctx.effect(() => tabTypes.register({
          id: 'dsh-dzrobot-ssh/files',
          kind: 'dzr-files',
          multiple: false,
          title: () => t('viewFiles'),
        }), 'dzrobot-ssh: tab type files')

        ctx.effect(() => ctx.slots.inject('sidebar.right.pane.tab', () => ctx.slots.register({
          name: 'sidebar.right.pane.tab',
          key: 'dsh-dzrobot-ssh/vnc',
          locale: NS,
          inject: () => ({}),
        }, VncTabShell)), 'dzrobot-ssh: tab body vnc')

        ctx.effect(() => ctx.slots.inject('sidebar.right.pane.tab', () => ctx.slots.register({
          name: 'sidebar.right.pane.tab',
          key: 'dsh-dzrobot-ssh/files',
          locale: NS,
          inject: () => ({}),
        }, FilesTabShell)), 'dzrobot-ssh: tab body files')
      } else {
        diag('tabs.unavailable', { hasRegistry: tabTypes !== undefined })
      }

      // ── the status float, on the frame-wide overlay ─────────────────────────
      // `shell.overlay` is a floating layer above every column and outside their
      // scroll containers, so the card stays put through panel switches — which
      // is the whole point of watching the car's battery while working
      // somewhere else.
      ctx.effect(() => ctx.slots.inject('shell.overlay', () => ctx.slots.register({
        name: 'shell.overlay',
        id: 'dzrobot-float',
        // Above the built-in overlay items, below any transient layer.
        order: 40,
        label: () => t('floatTitle'),
        locale: NS,
      }, makeFloatWindow(ctx, store, t))), 'dzrobot-ssh: status float overlay')

      // Prime the sidebar dot from the last known probe, then run the one-shot
      // self-test when the Host armed it: the flow is otherwise only reachable by
      // clicking, and a browser plugin cannot be introspected from the Host side.
      ctx.effect(() => {
        let alive = true
        void (async () => {
          try {
            const status = await api('/status')
            if (!alive) return
            store.set({ probe: status.probe, car: status.car, command: status.command, floatOpacity: status.settings?.floatOpacity })
            const selfTest = await api('/selftest')
            diag('client.selftest', selfTest)
            if (selfTest.run !== true) return
            diag('client.selftest.run', {})
            const result = await connect(ctx, document.body, phase => store.set({ phase }))
            diag('client.selftest.result', result)
            store.set({ phase: result.ok ? 'connected' : 'error', probe: result.ok ? { state: 'ready' } : undefined, message: result.ok ? '' : String(result.detail ?? result.reason ?? '') })
          } catch (error) {
            diag('client.selftest.error', { message: String(error?.message ?? error) })
          }
        })()
        return () => { alive = false }
      }, 'dzrobot-ssh: status + self-test')

      // Keep the dot honest without a click: the car appears and disappears as it
      // is powered on, and a stale green dot is the one thing worse than none.
      // `car=0` skips the description round-trip, and a hidden page probes
      // nothing — a background tab has no dot to keep honest.
      ctx.effect(() => {
        const timer = setInterval(() => {
          if (typeof document !== 'undefined' && document.hidden) return
          void api('/probe?car=0', { method: 'POST' })
            .then(result => store.set({ probe: result }))
            .catch(() => {})
        }, 120_000)
        return () => clearInterval(timer)
      }, 'dzrobot-ssh: reachability watch')

      void locale
    }

    exports.apply = apply
    exports.inject = inject
    exports.name = 'dzrobot-ssh'
    // Test seam for scripts/test-client.mjs: the connect flow runs headlessly
    // against a stubbed sidebar and terminal service, so a broken assumption
    // about those services fails here rather than as a dead click in the UI.
    exports.__test = {
      connect, ensureConversation, openToolTab, waitFor, describeService, surveyServices,
      DICT, STATE_KEY, TOOLS, makeVncView, makeFilesView, makeCarPanel,
      makeVncTabShell, makeFilesTabShell,
      // The float's pure parts: uptime formatting, the chart's axis/geometry
      // math and the open/pos persistence are plain logic worth pinning; the
      // card itself is exercised through the panel row and the overlay
      // registration above. uptimeView is the disconnect contract: live only
      // from a fresh car reading, 小车未连接 the moment that fails.
      makeFloatWindow, formatDuration, formatUptime, uptimeView, UPTIME_STALE_MS, levelOf, batteryLevelOf, clockTime, readFloatPos, floatOpenByDefault,
      chartModel, CHART,
      // generic VNC connector
      detectRuntime, normalizeConnectorConfig, makeVncConnectorPanel,
      bridgeConnectorConfig, VNC_RESOLUTIONS, CONNECTOR_ID,
    }
    return module.exports
  },
})