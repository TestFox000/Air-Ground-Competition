/**
 * dzrobot-ssh — plugin entry (Host half).
 *
 * What this half owns:
 *   · the settings for one car (host, user, port, extra ssh options);
 *   · the reachability and login probe, so the sidebar can say *why* a click
 *     would fail before the user spends the click;
 *   · a small cached description of the car (user, host, kernel, uptime);
 *   · a local JSON API the browser half reads, plus a diagnostic sink.
 *
 * What it deliberately does not own: the interactive SSH session. That runs in
 * a real DSH terminal, so the user gets echo, completion, Ctrl-C and resize from
 * the terminal the app already ships rather than a second-rate imitation here.
 * The browser half opens that terminal and types the command line this API
 * composes. See `src/ssh.js` for why the command is returned as text and the
 * probing path is `BatchMode=yes`.
 *
 * Nothing is a hard dependency: a composition without a web server still loads
 * the plugin and still probes on demand, it just has no page to answer.
 *
 * @module index.js
 */

import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'
import { DATA_DIR_NAME, JsonStore, SETTINGS_INITIAL, resolveDshHome } from './src/store.js'
import { collectMetrics } from './src/metrics.js'
import { commandLine, describe as describeCar, interactiveArgv, localName, probe } from './src/ssh.js'
import { isLoopback, rejectionFor } from './src/trust.js'
import { isReloading, selfReload, watchPackage } from './src/reload.js'
import { VncBridge, vncTarget } from './src/vnc.js'
import { acceptUpgrade, WebSocketChannel } from './src/ws.js'
import {
  MAX_READ_BYTES, MAX_UPLOAD_BYTES, downloadFile, homeShortcuts,
  listDirectory, readTextFile, statPath, uploadFile,
} from './src/files.js'
import { fieldSpec, listHistory, routeTokens, runChecks, runSim, serveArtifact, SIM_ROOT } from './src/simbench.js'
import { runStress, stressResult, stressStatus } from './src/stressbench.js'
import { droneHistory, runDroneSim } from './src/dronesim.js'

export const name = 'dzrobot-ssh'

/** No hard requirement: every host facility below is reached opportunistically. */
export const inject = []

/** The vendored noVNC tree, served to the browser half as native ES modules. */
const NOVNC_DIR = fileURLToPath(new URL('./novnc/', import.meta.url))

/** The VNC screen document, embedded by the panel from a real http origin. */
const VNC_APP_FILE = fileURLToPath(new URL('./vnc-app.html', import.meta.url))

/** The generic connector document: noVNC dialing any ws://host:port/path directly. */
const VNC_CONNECT_FILE = fileURLToPath(new URL('./vnc-connect.html', import.meta.url))

/**
 * The standalone status float: the same three numbers as the in-page overlay
 * (battery, CPU, uptime), served as its own document so it can live in a
 * separate OS window.
 */
const FLOAT_APP_FILE = fileURLToPath(new URL('./float.html', import.meta.url))

/**
 * The simulation bench: runs the local dzbrain simulator without the car, so a
 * repeatable sim run costs one click instead of an agent turn.
 */
const SIM_APP_FILE = fileURLToPath(new URL('./sim.html', import.meta.url))

/** Content types for the vendored tree; anything else is refused, not guessed. */
const NOVNC_TYPES = {
  '.js': 'text/javascript; charset=utf-8',
  '.mjs': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.txt': 'text/plain; charset=utf-8',
  '.md': 'text/plain; charset=utf-8',
}

export function apply(ctx, config = {}) {
  const logger = ctx.logger ?? console
  const home = resolveDshHome()
  const dataDir = path.join(home, DATA_DIR_NAME)
  fs.mkdirSync(dataDir, { recursive: true })

  const settings = new JsonStore(path.join(dataDir, 'settings.json'), SETTINGS_INITIAL, {
    log: message => logger.warn?.(message),
  })
  /** Last probe/description, kept so a reloaded page paints without re-probing. */
  const status = new JsonStore(path.join(dataDir, 'status.json'), {
    version: 1,
    probe: undefined,
    car: undefined,
    at: 0,
  }, { log: message => logger.warn?.(message) })

  const diagFile = path.join(dataDir, 'diag.jsonl')

  // The diagnostic log is a development aid with a hard purpose: a browser
  // plugin cannot be introspected from here, so the browser half reports what it
  // actually found — which services exist, whether the terminal opened, whether
  // the command was written — and that report is the only way to tell a working
  // integration from one that silently did nothing.
  let selfTest = config.selfTest === true
  let selfTestConsumed = false

  // Opt-in source watching: a normal install does not rewrite its own files, but
  // while this plugin is being worked on, a restart per edit is the expensive
  // part. Off unless the entry's config asks for it.
  let watcher
  if (config.watchSources === true) {
    watcher = watchPackage(fileURLToPath(new URL('./', import.meta.url)), {
      logger,
      onChange: () => {
        void selfReload(ctx, { logger }).catch(error => logger.warn?.(`dzrobot-ssh: reload failed (${error?.message ?? error})`))
      },
    })
    logger.info?.('dzrobot-ssh: watching sources, edits reload the plugin')
  }

  /** One probe, with the car description folded in when login works. */
  async function runProbe({ withCar = true } = {}) {
    const current = settings.get()
    const result = await probe(current, { logger })
    let car
    if (result.state === 'ready' && withCar) {
      const described = await describeCar(current, { logger })
      if (described.ok) car = described
    }
    if (result.state === 'ready') settings.update({ lastConnectedAt: Date.now() })
    status.update({ probe: result, car, at: Date.now() })
    status.flush()
    return { ...result, car }
  }

  const optional = service => (typeof ctx.get === 'function' ? ctx.get(service) : undefined)
  const fenceConnection = {
    get admit() {
      const current = optional('connection')
      return current === undefined ? undefined : (request => current.admit(request))
    },
  }

  const handler = async (req, res) => {
    const url = new URL(req.url ?? '/', 'http://localhost')
    const route = url.pathname.replace(/^\/api\/dzrobot-ssh/, '').replace(/\/+$/, '') || '/'
    const method = String(req.method ?? 'GET').toUpperCase()
    const send = (code, payload) => {
      const body = JSON.stringify(payload)
      res.writeHead(code, { 'content-type': 'application/json; charset=utf-8', 'cache-control': 'no-store' })
      res.end(body)
    }
    const rejection = rejectionFor(req, fenceConnection)
    if (rejection !== undefined) return send(rejection, { error: rejection === 401 ? 'unauthorized' : 'forbidden' })

    try {
      if (method === 'GET' && route === '/status') {
        const snapshot = status.get()
        return send(200, {
          settings: settings.get(),
          probe: snapshot.probe,
          car: snapshot.car,
          at: snapshot.at,
          command: commandLine(settings.get()),
          argv: interactiveArgv(settings.get()),
          vnc: vncTarget(settings.get()),
          files: { root: settings.get().filesRoot, shortcuts: homeShortcuts() },
          // The page the panel lives in runs under a custom `dsh-app://` scheme,
          // where ES modules do not load and a WebSocket built from
          // `location.host` points at a host named "app". The port the request
          // actually arrived on is the reliable way to name this server, and it
          // is what the VNC iframe and its WebSocket are built from.
          origin: requestOrigin(req),
          local: { host: localName(), dataDir },
          selfTest,
          reload: { watching: watcher !== undefined, last: globalThis[Symbol.for('dzrobot-ssh.last-reload')] ?? null },
        })
      }

      // ── the car's filesystem ────────────────────────────────────────────────
      if (method === 'GET' && route === '/files') {
        const requested = url.searchParams.get('path') ?? settings.get().filesRoot
        const listing = await listDirectory(settings.get(), requested)
        return send(listing.ok ? 200 : 400, listing)
      }
      if (method === 'GET' && route === '/file') {
        const requested = url.searchParams.get('path') ?? ''
        if (requested === '') return send(400, { ok: false, error: '缺少 path' })
        const file = await readTextFile(settings.get(), requested, { maxBytes: MAX_READ_BYTES })
        return send(file.ok ? 200 : 400, file)
      }
      if (method === 'GET' && route === '/stat') {
        const requested = url.searchParams.get('path') ?? ''
        if (requested === '') return send(400, { ok: false, error: '缺少 path' })
        const info = await statPath(settings.get(), requested)
        return send(info.ok ? 200 : 400, info)
      }
      if (method === 'GET' && route === '/download') {
        const requested = url.searchParams.get('path') ?? ''
        if (requested === '') return send(400, { ok: false, error: '缺少 path' })
        const info = await statPath(settings.get(), requested)
        if (!info.ok) return send(404, info)
        if (info.type === 'directory') return send(400, { ok: false, error: '这是一个目录' })
        res.writeHead(200, {
          'content-type': 'application/octet-stream',
          'content-length': String(info.size),
          'content-disposition': `attachment; filename="${path.basename(info.path).replaceAll('"', '')}"`,
          'cache-control': 'no-store',
        })
        const transferred = await downloadFile(settings.get(), requested, res)
        // A failed transfer has already had headers sent, so the only honest
        // signal left is closing the connection short of its content-length.
        if (!transferred.ok) {
          logger.warn?.(`dzrobot-ssh: download failed (${transferred.stderr})`)
          res.destroy()
        }
        return undefined
      }
      if (method === 'POST' && route === '/upload') {
        const requested = url.searchParams.get('path') ?? ''
        if (requested === '') return send(400, { ok: false, error: '缺少 path' })
        const declared = Number(req.headers['content-length'] ?? 0)
        if (Number.isFinite(declared) && declared > MAX_UPLOAD_BYTES) {
          return send(413, { ok: false, error: '文件过大' })
        }
        const transferred = await uploadFile(settings.get(), requested, req)
        return send(transferred.ok ? 200 : 400, transferred.ok
          ? { ok: true, path: requested }
          : { ok: false, error: transferred.stderr || '上传失败' })
      }
      if (method === 'POST' && route === '/probe') {
        return send(200, await runProbe({ withCar: url.searchParams.get('car') !== '0' }))
      }
      if (method === 'POST' && route === '/describe') {
        const described = await describeCar(settings.get(), { logger })
        if (described.ok) { status.update({ car: described }); status.flush() }
        return send(200, described)
      }
      // The float's reading. Always 200: an unreachable car is an answer the
      // float renders (as an error line), not an HTTP failure to hide behind.
      if (method === 'GET' && route === '/metrics') {
        return send(200, await collectMetrics(settings.get(), { logger }))
      }
      if (route === '/config') {
        if (method === 'GET') return send(200, settings.get())
        if (method === 'POST') {
          const body = await readJson(req)
          if (body === undefined) return send(400, { error: 'body must be a JSON object' })
          const patch = settings.sanitize(body)
          if (typeof patch.host === 'string' && patch.host === '' && (patch.sshAlias ?? settings.get().sshAlias) === '') {
            return send(400, { error: 'host 不能为空（或填写 ssh 别名）' })
          }
          if (typeof patch.user === 'string' && patch.user === '' && patch.sshAlias === '') {
            return send(400, { error: 'user 不能为空（或填写 ssh 别名）' })
          }
          settings.update(patch)
          return send(200, { settings: settings.get(), command: commandLine(settings.get()) })
        }
      }
      if (method === 'POST' && route === '/reload') {
        // The host half is a Node ES module: the process caches it by URL, so a
        // source edit is invisible until the app restarts. This swaps the running
        // plugin for the code on disk instead, which is what makes iterating on
        // the VNC and file paths practical.
        if (isReloading()) return send(409, { ok: false, error: 'a reload is already in progress' })
        const result = await selfReload(ctx, { logger })
        return send(result.ok ? 200 : 500, result)
      }
      if (method === 'GET' && route === '/reload') {
        return send(200, {
          last: globalThis[Symbol.for('dzrobot-ssh.last-reload')] ?? null,
          watching: watcher !== undefined,
        })
      }
      if (route === '/selftest') {        if (method === 'GET') {
          // A one-shot latch: the page that asked for the self-test gets it once.
          const run = selfTest && !selfTestConsumed
          if (run) selfTestConsumed = true
          return send(200, { run, armed: selfTest })
        }
        if (method === 'POST') {
          const body = await readJson(req) ?? {}
          selfTest = body.run === true
          selfTestConsumed = false
          return send(200, { armed: selfTest })
        }
      }
      if (method === 'POST' && route === '/diag') {
        const body = await readJson(req)
        if (body === undefined) return send(400, { error: 'body must be a JSON object' })
        appendDiag(diagFile, body, logger)
        return send(200, { ok: true })
      }
      if (method === 'GET' && route === '/diag') {
        return send(200, { text: readDiag(diagFile) })
      }
      if (method === 'GET' && route === '/graph') {
        // Which client plugins the web boot graph currently knows about. Without
        // this, a browser half that never loads is indistinguishable from one
        // that loads and fails, and the two need opposite fixes.
        const clientModules = optional('clientModules')
        if (clientModules === undefined) return send(200, { available: false })
        const graph = clientModules.graph?.()
        const entries = (graph?.plugins ?? graph?.entries ?? graph?.modules ?? [])
        const list = Array.isArray(entries) ? entries : Object.keys(entries)
        return send(200, {
          available: true,
          keys: Object.keys(graph ?? {}),
          plugins: list.map(entry => (typeof entry === 'string' ? entry : (entry?.id ?? entry?.name ?? JSON.stringify(entry)))).filter(Boolean),
          hasMine: JSON.stringify(graph ?? {}).includes('dzrobot-ssh'),
          clientPath: clientModules.clientPath?.('dsh-dzrobot-ssh'),
        })
      }

      // ── vendored noVNC, over this plugin's own origin ───────────────────────
      if (method === 'GET' && route === '/vnc.html') {
        return serveFile(res, send, VNC_APP_FILE, 'text/html; charset=utf-8')
      }
      // The generic connector: same origin, different document (see the file's
      // header for why it must live on a plain http origin at all).
      if (method === 'GET' && route === '/vnc-connect.html') {
        return serveFile(res, send, VNC_CONNECT_FILE, 'text/html; charset=utf-8')
      }
      if (method === 'GET' && route === '/float.html') {
        return serveFile(res, send, FLOAT_APP_FILE, 'text/html; charset=utf-8')
      }
      // ── the local simulation bench ─────────────────────────────────────────
      if (method === 'GET' && route === '/sim.html') {
        return serveFile(res, send, SIM_APP_FILE, 'text/html; charset=utf-8')
      }
      if (method === 'POST' && route === '/sim/run') {
        const body = await readJson(req) ?? {}
        const result = await runSim(body)
        if (!result.ok) return send(result.code ?? 400, { error: result.error })
        return send(200, result)
      }
      if (method === 'GET' && route === '/sim/history') {
        return send(200, listHistory())
      }
      if (method === 'GET' && route === '/sim/artifact') {
        const outcome = serveArtifact(url.searchParams.get('path') ?? '', res)
        return typeof outcome === 'number' ? send(outcome, { error: `artifact ${outcome}` }) : undefined
      }
      if (method === 'POST' && route === '/sim/check') {
        return send(200, await runChecks())
      }
      if (method === 'GET' && route === '/sim/root') {
        return send(200, { root: SIM_ROOT, ok: fs.existsSync(path.join(SIM_ROOT, 'src', 'dzbrain')) })
      }
      // 场地真值（只读）；页面用它画横置地图 + 预填改点位表单
      if (method === 'GET' && route === '/sim/map') {
        return send(200, { ...fieldSpec(), route: routeTokens() })
      }
      // ── 50 图压测（卡2；规格 §D 表的 3 个端点，跑批在后台、状态/结果轮询）──────
      if (method === 'POST' && route === '/sim/stress') {
        const body = await readJson(req) ?? {}
        // background:true = 同步段（参数校验 + N 图生成 + 状态置位）后立即返回 202 {batchId}，
        // 整批在后台按并发池跑；忙时 runStress 回 {ok:false,code:409} 原样转出。
        const result = await runStress({ ...body, background: true })
        if (!result.ok) return send(result.code ?? 500, { error: result.error })
        return send(202, { batchId: result.batchId, total: result.total })
      }
      if (method === 'GET' && route === '/sim/stress/status') {
        return send(200, stressStatus())
      }
      if (method === 'GET' && route === '/sim/stress/result') {
        const result = stressResult(url.searchParams.get('id') ?? '')
        return result.ok
          ? send(200, { batch: result.batch, stats: result.stats, records: result.records })
          : send(result.code ?? 404, { error: result.error })
      }
      // ── 无人机协同（卡3；规格 §D 表的 2 个端点，排卡2 之后写 index.js）────────────
      if (method === 'POST' && route === '/sim/drone/run') {
        const body = await readJson(req) ?? {}
        // 同步单跑：409（上跑/压测中）与 422（规划不过）原样转出，与 /sim/run 同语义
        const result = await runDroneSim(body)
        if (!result.ok) return send(result.code ?? 500, { error: result.error })
        return send(200, {
          ok: true, timeline: result.timeline, reroutes: result.reroutes,
          confirmed: result.confirmed, record: result.record,
          // ★ 拟真（卡3+）：透传无人机航线 + 情报命中，前端据此在场地地图上画飞行/扫描/采纳。
          scan: result.scan, intel: result.intel,
        })
      }
      if (method === 'GET' && route === '/sim/drone/history') {
        return send(200, droneHistory())
      }
      if (method === 'GET' && route.startsWith('/novnc/')) {
        return serveNovnc(res, route.slice('/novnc/'.length), send)
      }
      return send(404, { error: `no route ${method} ${route}` })
    } catch (error) {
      logger.warn?.(`dzrobot-ssh: ${method} ${route} failed (${error?.message ?? error})`)
      return send(500, { error: String(error?.message ?? error) })
    }
  }

  // The routes need the HTTP carrier, which a headless composition may lack, so
  // they live in their own fiber: its absence must not take the probe down.
  ctx.inject(['webServer'], scoped => {
    scoped.effect(
      () => scoped.webServer.register({ kind: 'prefix', path: '/api/dzrobot-ssh', handler }),
      'dzrobot-ssh: api routes',
    )

    // VNC is a byte tunnel, not a request/response pair: the upgrade seat hands
    // over the raw socket and says the handler owns negotiation and socket use,
    // so this registers the one path and does the handshake itself. A carrier
    // without that seat loses the desktop and keeps everything else.
    if (typeof scoped.webServer.registerUpgrade === 'function') {
      scoped.effect(
        () => scoped.webServer.registerUpgrade({
          path: '/api/dzrobot-ssh/vnc',
          handler: (req, socket, head) => openVncTunnel(req, socket, head, settings.get(), { logger }),
        }),
        'dzrobot-ssh: vnc tunnel',
      )
    } else {
      logger.warn?.('dzrobot-ssh: this web server has no upgrade seat; the VNC view is unavailable')
    }

    logger.info?.('dzrobot-ssh: API at /api/dzrobot-ssh (VNC tunnel at /api/dzrobot-ssh/vnc)')
  })

  ctx.effect(() => () => {
    settings.dispose()
    status.dispose()
    watcher?.()
  }, 'dzrobot-ssh: stores')

  logger.info?.(`dzrobot-ssh: ${commandLine(settings.get())} (data in ${dataDir})`)
}

/**
 * The http origin this server is reachable at, as seen by the request itself.
 *
 * `req.socket.localPort` is the port the request arrived on, which is the only
 * authority here: the browser page may be running under a custom scheme whose
 * `location.host` is not a network host at all, and the server's bind address is
 * not otherwise exposed to a plugin.
 */
function requestOrigin(req) {
  const port = req?.socket?.localPort
  const address = req?.socket?.localAddress
  const host = typeof address === 'string' && address !== '' && address !== '::' && address !== '0.0.0.0'
    ? (address.includes(':') ? `[${address}]` : address)
    : '127.0.0.1'
  return Number.isInteger(port) ? `http://${host}:${port}` : ''
}

/** Serve one plugin file whole, bounded, with the given content type. */
function serveFile(res, send, file, type) {
  let body
  try {
    body = fs.readFileSync(file)
  } catch {
    return send(404, { error: 'not found' })
  }
  res.writeHead(200, {
    'content-type': type,
    'content-length': String(body.length),
    'cache-control': 'no-store',
  })
  res.end(body)
  return undefined
}

/**
 * Serve one file from the vendored noVNC tree.
 *
 * The module graph is fetched by the browser itself (`import()` resolves the
 * relative specifiers inside rfb.js), so this only has to hand back bytes with
 * the right type — and to refuse anything that resolves outside the vendored
 * directory, since the requested path arrives from a URL.
 *
 * `access-control-allow-origin` is sent because the VNC document may be embedded
 * from the app's custom-scheme page, which is a different origin: a cross-origin
 * module import is only allowed when the response says so.
 */
function serveNovnc(res, relative, send) {
  let decoded
  try { decoded = decodeURIComponent(relative) } catch { decoded = '' }
  const target = path.resolve(NOVNC_DIR, decoded)
  const inside = target === NOVNC_DIR.replace(/[\\/]$/, '') || target.startsWith(NOVNC_DIR)
  if (!inside) return send(403, { error: 'forbidden' })
  const type = NOVNC_TYPES[path.extname(target).toLowerCase()]
  if (type === undefined) return send(404, { error: 'not found' })
  let body
  try {
    const stat = fs.statSync(target)
    if (!stat.isFile()) return send(404, { error: 'not found' })
    body = fs.readFileSync(target)
  } catch {
    return send(404, { error: 'not found' })
  }
  res.writeHead(200, {
    'content-type': type,
    'content-length': String(body.length),
    'access-control-allow-origin': '*',
    // The tree is immutable for a given vendored version, and the browser
    // caches ES modules aggressively — which is what makes the import graph
    // cheap on every later page load.
    'cache-control': 'public, max-age=86400',
  })
  res.end(body)
  return undefined
}

/**
 * Adopt one upgraded socket as a VNC tunnel.
 *
 * Order matters: the handshake is answered first so the browser's WebSocket
 * opens, then the car is dialled, and a dial failure is reported as a close
 * rather than a socket that simply never speaks.
 */
function openVncTunnel(req, socket, head, settings, { logger } = {}) {
  const peer = req?.socket?.remoteAddress ?? req?.connection?.remoteAddress
  if (!isLoopback(peer)) { socket.destroy(); return }
  if (acceptUpgrade(req, socket) === null) return

  // The channel is built from the request so the negotiated sub-protocol is
  // known before any payload is decoded: noVNC asks for none by default and for
  // `base64` only when configured to, and those two carry different bytes.
  const protocol = String(req.headers['sec-websocket-protocol'] ?? '')
    .split(',').map(value => value.trim()).includes('base64') ? 'base64' : ''
  const channel = new WebSocketChannel(socket, protocol, head)
  const bridge = new VncBridge(settings, { logger })
  let failed = false

  // Handlers are registered before the dial so nothing can arrive unhandled;
  // VncBridge attaches them to the socket when it connects.
  channel.onClose(() => bridge.close())
  channel.onMessage(payload => { if (!failed) bridge.send(payload) })
  bridge.onData(chunk => channel.send(chunk))
  bridge.onClose(() => channel.close(1001, 'car closed the desktop'))

  void bridge.open()
    .then(() => logger.info?.(`dzrobot-ssh: VNC tunnel open (${bridge.describe()})`))
    .catch(error => {
      failed = true
      logger.warn?.(`dzrobot-ssh: VNC tunnel failed (${error?.message ?? error})`)
      channel.close(1011, String(error?.message ?? error).slice(0, 120))
    })
}

/** Read a JSON request body, bounded; `undefined` for anything that is not an object. */function readJson(req) {
  return new Promise(resolve => {
    let text = ''
    req.on('data', chunk => {
      text += chunk
      if (text.length > 64 * 1024) { text = ''; req.destroy() }
    })
    req.on('end', () => {
      if (text.trim() === '') return resolve({})
      try {
        const parsed = JSON.parse(text)
        resolve(parsed !== null && typeof parsed === 'object' && !Array.isArray(parsed) ? parsed : undefined)
      } catch {
        resolve(undefined)
      }
    })
    req.on('error', () => resolve(undefined))
  })
}

function appendDiag(file, entry, logger) {
  try {
    fs.appendFileSync(file, `${JSON.stringify({ at: Date.now(), ...entry })}\n`, 'utf8')
  } catch (error) {
    logger.warn?.(`dzrobot-ssh: could not write diag (${error?.message ?? error})`)
  }
}

function readDiag(file) {
  try {
    const text = fs.readFileSync(file, 'utf8')
    // Bounded: the point is the most recent integration run, not a growing log.
    return text.split('\n').filter(line => line.trim() !== '').slice(-200).join('\n')
  } catch {
    return ''
  }
}