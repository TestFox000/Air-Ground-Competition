## Primary Request and Intent
- Build a noVNC-based VNC connector with 7 requirements: (1) input fields for VNC host, WebSocket port, path, VNC password, **plus resolution selector**; (2) use noVNC RFB to connect to `ws://host:port/websockify`; (3) support adaptive scaling, fullscreen, disconnect, reconnect; (4) if DSH is Electron/Tauri, embed WebView; (5) if DSH is a browser plugin, use iframe or new tab; (6) provide complete Ubuntu-side TigerVNC + websockify commands; (7) provide SSH tunnel secure-connection solution.
- Follow-up correction (verbatim): "VNC连接器不独立存在，而是取代掉"小车\VNC桌面"，火速修改" — the standalone connector must not exist separately; it must **replace** the car panel's "VNC 桌面" tab.
- Latest user messages: pasted a terminal screenshot showing `sudo apt install x11vnc` → "已在此计算机上禁用 Sudo。若要启用它，请转到"设置"应用中的 Developer Settings page." and asks "这是什么意思" (what does this mean); and "vnc又连不上了" (VNC can't connect again).

## Key Technical Concepts
- DSH (DeepSeek Harness) is an **Electron** app (checkout at `C:\Users\TestFox\AppData\Local\Programs\DeepSeek Harness\resources\app.asar\dsh\`), GUI at `http://127.0.0.1:19387`.
- Plugin halves: host half (Node, main process) serves HTTP routes + WebSocket upgrade; client half (renderer) renders panels in the DSH GUI.
- DSH pages render under `dsh-app://` scheme; ES modules cannot load there, so noVNC must be served from the plugin's own `http://127.0.0.1:PORT` origin and embedded via **iframe**.
- Profile installs `file:D:/…` plugin deps as a **packaged copy** governed by `package.json#files` — NOT a junction (an earlier README claim of junction was wrong and has been corrected). Reload re-imports from the installed location, not the source tree.
- `scripts/deploy.mjs` syncs source → installed copy (`--dry-run` available); host supports `POST /reload`.
- noVNC channel validation inspects only **one prototype level** (`Object.getOwnPropertyNames(Object.getPrototypeOf(rawChannel))`), so subclassing `WebSocket` breaks it; a `new Proxy(NativeWebSocket, { construct })` trap returning a native instance is the correct wrapper.
- `scaleViewport` = fit-to-window toggle; resolution selection uses `resizeSession` for "adaptive" and noVNC's `SetDesktopSize` extension (temporarily size the screen container to WxH, then restore).
- Electron/Tauri detection shows a runtime badge; "new window" uses `window.open` (real BrowserWindow) on Electron/Tauri, new tab in browser; `<webview>` tag deliberately avoided (needs webviewTag flag and uses ipc, not DOM postMessage, which would silently break half the control loop).
- TigerVNC new versions bind loopback only; `-localhost no` is required for remote access.

## Files and Code
- `D:\User\Desktop\dzrobot_v10.05\` — workspace root / plugin source tree (`dsh-plugin-dzrobot-ssh`).
- `vnc-connect.html` — the generic connector document served by the plugin host; contains the panel→document postMessage protocol (`dzrobot-panel` direction) and noVNC RFB bootstrap; verified 200 from the running plugin.
- `novnc/` + `vnc-app.html` + `vnc.html` — vendored noVNC assets; `package.json#files` must declare them (missing entries caused 404s).
- `client.js` — legacy `makeVncView` marked LEGACY (still used by `vnc.html` serving and tests) but **no longer mounted**; `makeVncConnectorPanel` is now mounted in the car panel's VNC tab; standalone sidebar icon + `main` panel registration removed.
- `src/vnc.js` — `VncBridge` (car TCP→WebSocket upgrade at `/api/dzrobot-ssh/vnc`, address cache, RFB banner guard).
- `test-host.mjs`, `test-live-ws.mjs` — start from source tree (blind spot: they did not test the installed copy).
- `test-vnc-wrapper.mjs` — feeds real `vnc-app.html` wrapper code into real `Websock.attach()`; asserts subclasses are rejected.
- `test-package-files.mjs` — asserts runtime assets are declared in `files`, exist in the installed copy, and are byte-identical to the source tree.
- `test-vnc-connect.mjs` — executes the document's real module script in Node with an RFB stub; 16 assertions.
- `test-client.mjs` — asserts connector registers no standalone panel/icon, tab mounts connector, must not mount legacy view.
- `scripts/ubuntu-vnc-websockify.sh` — Ubuntu install path: packages → `vncpasswd` → `vncserver@:1` systemd unit with `-localhost no` → `websockify 6080 127.0.0.1:5901` unit → verify.
- `scripts/debug-ws-handshake.mjs` — browser-identical handshake headers, raw response dump.
- `diag.jsonl` — probe log with timestamps for causality.
- README — corrected the junction claim; SSH tunnel section: websockify bound to 127.0.0.1, no firewall ports opened, `ssh -N -L 6080:127.0.0.1:6080 user@server`, connector points at `ws://127.0.0.1:6080/websockify`.

## Errors and Fixes
- `Raw channel missing property: send` → caused by `class extends WebSocket` wrapper (leaves `send`/`close`/`binaryType` one prototype level too deep). Threw from the `RFB` constructor, socket became unreachable, GC'd, connection aborted → browser saw only error + 1016/1006, never `open`. Fixed with a `construct`-trap `Proxy`.
- `/vnc.html`, `/novnc/core/rfb.js` 404 → `package.json#files` omitted `novnc/` and `vnc-app.html`, so the packaged install copy lacked them. Fixed by declaring them; `test-package-files.mjs` reproduces and now guards it.
- "Your VNC is already connected" was a **wrong claim**: the 4 `accepted: 10.240.174.14` log lines were my own test connections, not the user's browser.
- Connector config from panel was ignored by the document (connection used an empty form and failed immediately) → caught by `test-vnc-connect.mjs`.
- Document message listener filtered the wrong direction (`dzrobot-vnc-connect` instead of `dzrobot-panel`) → commands never arrived.
- Auth-state bug: connect button disabled on wrong password, so the user could not retry after correcting the password.
- Scale toggle desynced from the panel after iframe reload.
- TDZ error `Cannot access 'save' before initialization` → `save` was defined after the effect that used it for prefill; moved definition before the effect.
- Car-side `dhd` driver fails to join the hotspot (`WLC_E_SET_SSID`) — hardware/driver level, needs the hardware owner; signal good, power-saving off.
- User feedback corrections to honor: never claim a connection state without evidence attributable to the user's own client; tests must exercise the **installed** artifact, not just the source tree.

## Pending Jobs
- Answer "what does this mean" for the pasted screenshot (Windows/Chinese-locale message: `sudo` is disabled on this machine by policy; enabling requires Settings → Developer Settings page — i.e. Developer Mode / sudo enablement on Windows, so `sudo apt install x11vnc` is not a valid path on this host).
- Diagnose "vnc又连不上了" (VNC cannot connect again) — needs fresh evidence from `diag.jsonl` and the panel/document state after the connector merge.
- Optional, user's call: remove the stale `locale` entry in `package.json#files` (directory does not exist; deploy script warns each run).

## Current Work
- Connector merge into the car panel's VNC tab was completed, deployed (70 files synced), hot-reloaded (`evicted:8`), and all six suites passed (host / client / wrapper / guard / connect / package).
- The user then reported a new connection failure ("vnc又连不上了") alongside an unrelated screenshot about `sudo` being disabled, showing the failure is in the current merged state.

## Next Step
- Explain the screenshot briefly (Windows blocked `sudo`; it is not Linux, so `apt`/TigerVNC/websockify must run on the **Ubuntu** machine, from an Ubuntu terminal or the provided `scripts/ubuntu-vnc-websockify.sh`), then investigate the renewed VNC failure by reading `diag.jsonl` and verifying the served connector/URL state.

## Critical Context
- One VNC surface only: car panel → 「VNC 桌面」 tab → connector form. First run prefills `ws://<DSH host>:<port>/api/dzrobot-ssh/vnc` (plugin's own bridge straight to car 5901; no websockify needed on the car) plus the saved car password; once the user edits and persists values, those win over prefill. Intended: pre-merge behavior for the car, editable to any `ws://host:port/websockify`.
- Passwords are stored in localStorage, never placed in URLs.
- Serving origin matters: `dsh-app://` cannot load ES modules, hence the iframe + plugin HTTP origin design.
- Verified: 4 browser-facing URLs return 200; served page is the fixed (Proxy) version; browser-identical WebSocket handshake correct (accept key, subprotocol, extensions properly rejected); real car RFB banner `RFB 003.003` arrived in 126 ms; six suites green. **Unverified: actual desktop rendering** — requires the user's browser interaction.
- Approval prompts are disabled; do not request sandbox escalation (`sandbox_permissions`). File policy is danger-full-access.
- Current model route: `modlens-our-free-model/z-ai/glm-5.3`.
- Treat the ambiguous "VNC 桌面" and "小车" identifiers as the car panel and its VNC tab; keep panel/document postMessage direction (`dzrobot-panel`) intact when editing.