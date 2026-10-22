/**
 * Host-half integration test with a stub cordis context.
 *
 * Boots `apply()` against a fake ctx, captures the route it registers on the
 * stubbed `webServer`, serves them on loopback, and exercises every endpoint —
 * including a real SSH probe against the drone. Run:
 *   node scripts/test-host.mjs
 */

import http from 'node:http'
import net from 'node:net'
import fs from 'node:fs'
import os from 'node:os'
import path from 'node:path'

// The plugin resolves its data directory from DSH_HOME at apply() time, and this
// test writes settings and diagnostics. Point it at a throwaway directory so a
// test run can never leave the user's real drone address rewritten.
process.env.DSH_HOME = fs.mkdtempSync(path.join(os.tmpdir(), 'dzdrone-ssh-test-'))

const { apply, inject, name } = await import('../index.js')

const routes = []
const upgrades = []
const effects = []
const logs = []

/** Minimal stand-in for the cordis Context members this plugin touches. */
const ctx = {
  logger: {
    info: message => logs.push(`info ${message}`),
    warn: message => logs.push(`warn ${message}`),
    error: message => logs.push(`error ${message}`),
  },
  get: () => undefined,
  effect: (callback, label) => {
    effects.push(label)
    const disposer = callback()
    return typeof disposer === 'function' ? disposer : () => {}
  },
  inject: (deps, callback) => {
    logs.push(`inject ${deps.join(',')}`)
    callback({
      webServer: {
        register: route => {
          routes.push(route)
          return () => {}
        },
        registerUpgrade: route => {
          upgrades.push(route)
          return () => {}
        },
      },
      effect: ctx.effect,
    })
  },
}

console.log('name:', name, '| inject:', JSON.stringify(inject))
apply(ctx, {})

console.log('routes:', routes.map(route => `${route.kind}:${route.path}`).join(', '))
console.log('upgrades:', upgrades.map(route => route.path).join(', '))
console.log('effects:', effects.join(' | '))
if (routes.length === 0) throw new Error('no route registered')

const server = http.createServer((req, res) => {
  const route = routes.find(candidate => req.url.startsWith(candidate.path))
  if (route === undefined) { res.writeHead(404).end('no route'); return }
  void route.handler(req, res)
})
// The upgrade seat is exact-path dispatch onto a raw socket, so the test server
// forwards upgrades the same way the shipped one does.
server.on('upgrade', (req, socket, head) => {
  const route = upgrades.find(candidate => candidate.path === new URL(req.url, 'http://x').pathname)
  if (route === undefined) { socket.destroy(); return }
  void route.handler(req, socket, head)
})

await new Promise(resolve => server.listen(0, '127.0.0.1', resolve))
const port = server.address().port
const base = `http://127.0.0.1:${port}/api/dzdrone-ssh`

let failures = 0
const check = (label, ok, detail) => {
  console.log(`${ok ? 'OK  ' : 'FAIL'}  ${label}${detail === undefined ? '' : `  ${detail}`}`)
  if (!ok) failures += 1
}

async function call(path, options) {
  const response = await fetch(`${base}${path}`, {
    ...options,
    headers: { 'content-type': 'application/json', ...options?.headers },
  })
  const text = await response.text()
  let payload
  try { payload = JSON.parse(text) } catch { payload = text }
  return { status: response.status, payload }
}

const status = await call('/status')
console.log('\nGET /status ->', status.status)
console.log('  settings.host:', status.payload.settings?.host, '| user:', status.payload.settings?.user)
console.log('  command:', status.payload.command)
console.log('  local:', JSON.stringify(status.payload.local))

const probe = await call('/probe', { method: 'POST' })
console.log('\nPOST /probe ->', probe.status, JSON.stringify(probe.payload))

const config = await call('/config', { method: 'POST', body: JSON.stringify({ port: 2222, host: 'dzsmart.local' }) })
console.log('\nPOST /config ->', config.status, '| command now:', config.payload.command)

const bad = await call('/config', { method: 'POST', body: JSON.stringify({ host: '' }) })
console.log('POST /config (empty host, alias also empty) ->', bad.status, JSON.stringify(bad.payload))

await call('/config', { method: 'POST', body: JSON.stringify({ port: 22 }) })

await call('/diag', { method: 'POST', body: JSON.stringify({ step: 'test-host', data: { ok: true } }) })
const diag = await call('/diag')
console.log('\nGET /diag ->', diag.status, '| last line:', String(diag.payload.text).split('\n').filter(Boolean).pop())

const selftest1 = await call('/selftest')
const selftest2 = await call('/selftest')
console.log('\nGET /selftest twice (disarmed) ->', JSON.stringify(selftest1.payload), JSON.stringify(selftest2.payload))

const armed = await call('/selftest', { method: 'POST', body: JSON.stringify({ run: true }) })
const first = await call('/selftest')
const second = await call('/selftest')
console.log('armed ->', JSON.stringify(armed.payload), '| first read:', JSON.stringify(first.payload), '| second read:', JSON.stringify(second.payload))

// The stub context mounts no `clientModules`, so this exercises the absent
// branch — the one a headless composition would hit.
const graph = await call('/graph')
console.log('\nGET /graph (no clientModules mounted) ->', graph.status, JSON.stringify(graph.payload))

// ── the metrics parser (offline, synthetic payloads) ──────────────────────────
// Every gauge number is parsed out of one marker-bounded payload, so a
// mis-parse here paints plausible-looking lies on the card. The cases cover
// the whole uptime contract: /proc/uptime is the source, /proc/stat's btime
// rides along when present (and is dropped when absurd), shell noise outside
// the markers is ignored, and a truncated payload is a failed reading — never
// a plausible zero.
{
  const { parseMetrics } = await import('../src/metrics.js')
  const payload = lines => lines.join('\n')
  const full = parseMetrics(payload([
    'Welcome to Ubuntu 22.04 (banner noise outside the markers)',
    'DZH_BEGIN',
    '100 0 50 900 0 0 0 0 0 0',
    '500 0 100 2400 0 0 0 0 0 0',
    'temp_cpu-thermal=53700',
    'temp_gpu-thermal=51000',
    'uptimeSec=666.97',
    'bootEpoch=1791338271',
    'load1=0.33 load5=0.25 load15=0.20',
    'batteryPercent=65.56',
    'batteryVoltage=11.33',
    'DZH_END',
    'trailing noise',
  ]))
  check('parser: a full payload answers every gauge',
    full !== undefined && Math.abs(full.uptimeSec - 666.97) < 1e-9
      && full.bootEpoch === 1791338271 && full.temp === 53.7 && full.tempAt === 'cpu-thermal'
      && full.zones === 2 && Array.isArray(full.load) && Number.isFinite(full.cpuPercent),
    JSON.stringify(full?.load) + ` up=${full?.uptimeSec} btime=${full?.bootEpoch} cpu=${full?.cpuPercent}`)
  check('parser: a battery payload answers percent and voltage',
    full !== undefined && full.battery !== undefined
      && Math.abs(full.battery.percent - 65.6) < 1e-9 && Math.abs(full.battery.voltage - 11.33) < 1e-9,
    JSON.stringify(full?.battery))
  const noBattery = parseMetrics(payload([
    'DZH_BEGIN', '0 0 0 100 0 0 0 0 0 0', '10 0 0 190 0 0 0 0 0 0',
    'uptimeSec=42', 'DZH_END',
  ]))
  check('parser: no battery lines means no battery field (FC absent), never a fake 0',
    noBattery !== undefined && noBattery.uptimeSec === 42 && !('battery' in noBattery),
    JSON.stringify(noBattery))
  const partialBattery = parseMetrics(payload([
    'DZH_BEGIN', '0 0 0 100 0 0 0 0 0 0', '10 0 0 190 0 0 0 0 0 0',
    'uptimeSec=42', 'batteryPercent=65.5', 'DZH_END',
  ]))
  check('parser: a lone percentage without voltage is not a battery',
    partialBattery !== undefined && !('battery' in partialBattery),
    JSON.stringify(partialBattery))
  // The flight controller is plugged in (batteryFc=present) but its DDS topic
  // stayed silent — a different fact from "no flight controller", so the card
  // must carry a note, not the false 无飞控 of an absent battery field. These
  // three branches were verified on the real drone (FC board in, uXRCE-DDS
  // agent down / ROS absent / topic echo empty) and are the whole point of the
  // batteryNote design: never fabricate 0%, never label a plugged-in board 无飞控.
  const fcSilent = parseMetrics(payload([
    'DZH_BEGIN', '0 0 0 100 0 0 0 0 0 0', '10 0 0 190 0 0 0 0 0 0',
    'uptimeSec=42', 'batteryFc=present', 'batteryNote=dds_silent', 'DZH_END',
  ]))
  check('parser: a plugged-in FC with a silent DDS topic carries a note, not silence',
    fcSilent !== undefined && !('battery' in fcSilent) && fcSilent.batteryNote === 'dds_silent',
    JSON.stringify(fcSilent?.batteryNote))
  const fcNoRos = parseMetrics(payload([
    'DZH_BEGIN', '0 0 0 100 0 0 0 0 0 0', '10 0 0 190 0 0 0 0 0 0',
    'uptimeSec=42', 'batteryFc=present', 'batteryNote=no_ros', 'DZH_END',
  ]))
  check('parser: a plugged-in FC with no ROS 2 carries the no_ros note',
    fcNoRos !== undefined && !('battery' in fcNoRos) && fcNoRos.batteryNote === 'no_ros',
    JSON.stringify(fcNoRos?.batteryNote))
  const fcPresentNoNote = parseMetrics(payload([
    'DZH_BEGIN', '0 0 0 100 0 0 0 0 0 0', '10 0 0 190 0 0 0 0 0 0',
    'uptimeSec=42', 'batteryFc=present', 'DZH_END',
  ]))
  check('parser: a plugged-in FC with no note defaults to dds_silent, not absent',
    fcPresentNoNote !== undefined && !('battery' in fcPresentNoNote)
      && fcPresentNoNote.batteryNote === 'dds_silent',
    JSON.stringify(fcPresentNoNote?.batteryNote))
  const fcAbsent = parseMetrics(payload([
    'DZH_BEGIN', '0 0 0 100 0 0 0 0 0 0', '10 0 0 190 0 0 0 0 0 0',
    'uptimeSec=42', 'batteryFc=absent', 'DZH_END',
  ]))
  check('parser: an explicit absent FC carries neither battery nor note',
    fcAbsent !== undefined && !('battery' in fcAbsent) && !('batteryNote' in fcAbsent),
    JSON.stringify({ b: fcAbsent?.battery, n: fcAbsent?.batteryNote }))
  const noBtime = parseMetrics(payload([
    'DZH_BEGIN', '0 0 0 100 0 0 0 0 0 0', '10 0 0 190 0 0 0 0 0 0',
    'uptimeSec=42', 'DZH_END',
  ]))
  check('parser: an answer without btime still parses, with bootEpoch absent',
    noBtime !== undefined && noBtime.uptimeSec === 42 && !('bootEpoch' in noBtime),
    JSON.stringify(noBtime))
  const absurd = parseMetrics(payload([
    'DZH_BEGIN', '0 0 0 100 0 0 0 0 0 0', '10 0 0 190 0 0 0 0 0 0',
    'uptimeSec=42', 'bootEpoch=123456789', 'DZH_END',
  ]))
  check('parser: a boot instant before 2000 is dropped, not printed',
    absurd !== undefined && !('bootEpoch' in absurd), JSON.stringify(absurd?.bootEpoch))
  const truncated = parseMetrics('DZH_BEGIN\n100 0 50 900 0 0 0 0 0 0\nuptimeSec=99\n')
  check('parser: a payload missing the end marker is a failed reading, not a zero',
    truncated === undefined, String(truncated))
}

// ── vendored noVNC ────────────────────────────────────────────────────────────
console.log('\n--- noVNC assets ---')

// The VNC screen is a separate document because the app's own page runs under a
// custom `dsh-app://` scheme where ES module loading fails. That only works if
// its own relative import resolves, so the exact URL matters.
const vncPage = await fetch(`${base}/vnc.html`)
const vncHtml = await vncPage.text()
check('GET /vnc.html is an html document',
  vncPage.status === 200 && String(vncPage.headers.get('content-type')).startsWith('text/html'),
  `${vncPage.status} ${vncPage.headers.get('content-type')} ${vncHtml.length}B`)
const imported = /import RFB from '\.\/novnc\/core\/rfb\.js'/.test(vncHtml)
check('the screen imports noVNC by a relative path', imported, imported ? './novnc/core/rfb.js' : 'import line not found')
// Resolve that relative specifier the way the browser would, from the page's URL.
const pageBase = new URL(`${base}/vnc.html`)
const resolved = new URL('./novnc/core/rfb.js', pageBase).href
const resolvedResponse = await fetch(resolved)
check('the page-relative module URL resolves', resolvedResponse.status === 200, resolved.replace(port, '<port>'))
check('and it is served for a different origin (cross-origin module load)',
  resolvedResponse.headers.get('access-control-allow-origin') === '*',
  String(resolvedResponse.headers.get('access-control-allow-origin')))
check('the server origin is reported for the panel to build absolute URLs from',
  typeof status.payload.origin === 'string' && status.payload.origin === `http://127.0.0.1:${port}`,
  String(status.payload.origin))

const rfbAsset = await fetch(`${base}/novnc/core/rfb.js`)
const rfbText = await rfbAsset.text()
check('GET /novnc/core/rfb.js', rfbAsset.status === 200 && rfbText.includes('export default class RFB'),
  `${rfbAsset.status} ${rfbAsset.headers.get('content-type')} ${rfbText.length}B`)
check('served as a JS module (not a guessed type)',
  String(rfbAsset.headers.get('content-type')).startsWith('text/javascript'), rfbAsset.headers.get('content-type'))
const nested = await fetch(`${base}/novnc/core/util/int.js`)
check('nested relative import resolves', nested.status === 200, String(nested.status))
const vendored = await fetch(`${base}/novnc/vendor/pako/lib/zlib/inflate.js`)
check('vendored pako resolves', vendored.status === 200, String(vendored.status))

// `fetch` normalises a literal `../..` away before it leaves the process, so the
// guard is exercised with a percent-encoded traversal that survives to the
// handler — which is exactly the shape an attacker would send.
const traversal = await new Promise(resolve => {
  const request = http.request({
    host: '127.0.0.1', port, path: '/api/dzdrone-ssh/novnc/%2e%2e/%2e%2e/package.json', method: 'GET',
  }, response => { response.resume(); resolve(response.statusCode) })
  request.on('error', () => resolve(-1))
  request.end()
})
check('encoded path traversal is refused', traversal === 403 || traversal === 404, String(traversal))

const notJs = await fetch(`${base}/novnc/LICENSE.txt`)
check('a text file is served, not refused', notJs.status === 200, String(notJs.status))

// ── the generic VNC connector document ────────────────────────────────────────
// Same origin as the drone screen, different document: it dials ws://host:port/path
// directly (websockify), with its own form for standalone use and a postMessage
// protocol for the embedded panel.
const connectPage = await fetch(`${base}/vnc-connect.html`)
const connectHtml = await connectPage.text()
check('GET /vnc-connect.html is an html document',
  connectPage.status === 200 && String(connectPage.headers.get('content-type')).startsWith('text/html'),
  `${connectPage.status} ${connectPage.headers.get('content-type')} ${connectHtml.length}B`)
check('the connector imports noVNC by a relative path',
  /import RFB from '\.\/novnc\/core\/rfb\.js'/.test(connectHtml), 'import line not found')
check('the connector speaks the embedded-panel protocol',
  connectHtml.includes("source: 'dzdrone-vnc-connect'"), 'protocol marker not found')
check('the connector builds the ws URL from host/port/path',
  /cfg\.scheme.*cfg\.host.*cfg\.port.*cfg\.path/s.test(connectHtml), 'url builder not found')
check('the connector ships a resolution list with 自适应/auto',
  connectHtml.includes('value="auto"') && connectHtml.includes('2560x1440'), 'resolution options not found')
check('the connector keeps the ws-log probe markers',
  connectHtml.includes('/* ws-log:start') && connectHtml.includes('/* ws-log:end */'),
  'markers present — scripts/test-vnc-wrapper.mjs extracts them')

// ── the drone's filesystem ──────────────────────────────────────────────────────
console.log('\n--- files ---')
const listing = await call('/files?path=/home/duzhong')
check('GET /files lists the home directory',
  listing.status === 200 && listing.payload.ok === true && listing.payload.entries?.length > 5,
  `${listing.status} ${listing.payload.entries?.length} entries`)
// Guarded: when the drone is offline this answers 400 with an `error` and no
// `entries`, and the suite must go on to report every later check instead of
// dying on a TypeError — the offline case is still a FAIL, just a legible one.
const entries = listing.payload.entries
check('listing is sorted with directories first',
  Array.isArray(entries) && (entries.length === 0 || entries[0].type === 'directory'),
  Array.isArray(entries) ? `first=${entries[0]?.type}` : `no entries (${listing.status} ${listing.payload.error ?? ''})`)
const missingDir = await call('/files?path=/nope/nope')
check('a missing directory is a 400 with a message', missingDir.status === 400 && missingDir.payload.error !== undefined,
  `${missingDir.status} ${missingDir.payload.error}`)
const file = await call('/file?path=/etc/hostname')
check('GET /file returns text', file.status === 200 && file.payload.text !== undefined,
  JSON.stringify(String(file.payload.text ?? '').trim()))
const statInfo = await call('/stat?path=/etc/hostname')
check('GET /stat reports a real size', statInfo.status === 200 && statInfo.payload.size > 0,
  `size=${statInfo.payload.size} type=${statInfo.payload.type}`)
// Two probes write files into the drone's /tmp — which WORKFLOW.md rule 2
// forbids for the assistant (SSH is read-only). They are therefore opt-in,
// exactly like the real-VNC banner further down: `DZD_TEST_DRONE_WRITE=1`.
// Everything read-only (listing, reading, stat, download) still runs always.
const mayWrite = process.env.DZD_TEST_DRONE_WRITE === '1'
// A small binary is uploaded rather than borrowed from the system, so the check
// does not depend on any particular file existing on the drone.
if (mayWrite) {
  await fetch(`${base}/upload?path=/tmp/dsh-bin-host-test.bin`, {
    method: 'POST',
    body: new Uint8Array([0, 1, 2, 3, 0, 255, 7, 0, 65, 0, 66]),
  })
  const binary = await call('/file?path=/tmp/dsh-bin-host-test.bin')
  check('a small binary is flagged, never inlined as text',
    binary.payload.binary === true && binary.payload.text === undefined,
    JSON.stringify({ ok: binary.payload.ok, binary: binary.payload.binary, size: binary.payload.size }))
} else {
  console.log('SKIP  binary upload probe (DZD_TEST_DRONE_WRITE not set — WORKFLOW: read-only SSH)')
}

// Regression guard for the symlink bug: undereferenced, this path reports 22
// bytes and the reader then pulls the whole 2.7 MB target through.
const viaLink = await call('/file?path=/usr/bin/Xvnc')
check('a symlink to a huge file is refused by its real size',
  viaLink.status === 400 && viaLink.payload.text === undefined,
  `${viaLink.status} size=${viaLink.payload.size}`)
const download = await fetch(`${base}/download?path=/etc/hostname`)
const downloaded = await download.text()
check('GET /download streams the bytes',
  download.status === 200 && downloaded.trim() === 'dzsmart',
  `${download.status} ${JSON.stringify(downloaded.trim())}`)
if (mayWrite) {
  const upload = await fetch(`${base}/upload?path=/tmp/dsh-upload-probe.txt`, { method: 'POST', body: 'hello from the plugin test' })
  const uploaded = await upload.json().catch(() => ({}))
  check('POST /upload writes to the drone', upload.status === 200 && uploaded.ok === true, JSON.stringify(uploaded))
  const readBack = await call('/file?path=/tmp/dsh-upload-probe.txt')
  check('the uploaded bytes read back identical',
    String(readBack.payload.text ?? '') === 'hello from the plugin test', JSON.stringify(readBack.payload.text))
} else {
  console.log('SKIP  text upload probe (DZD_TEST_DRONE_WRITE not set — WORKFLOW: read-only SSH)')
}

// ── the VNC bridge ────────────────────────────────────────────────────────────
/*
 * Tested against a local TCP stand-in rather than the drone's own VNC, and that is
 * a deliberate correction: TigerVNC counts an RFB connection that never completes
 * authentication as a security failure, so *probing the drone* — even just to read
 * its banner and hang up — puts this machine on the drone's blacklist and locks the
 * operator out of their own desktop for minutes. A test must not do that.
 *
 * The stand-in proves more than the banner regex it replaces: exact bytes in both
 * directions, across all three WebSocket frame-length encodings (7-bit, 16-bit and
 * 64-bit), which is the part of the framing code that is easy to get wrong.
 */
console.log('\n--- vnc bridge (local stand-in) ---')
const echo = net.createServer(socket => {
  socket.write(Buffer.from('RFB 003.008\n'))
  socket.write(Buffer.from([2, 19, 2]))
  socket.on('data', chunk => socket.write(chunk))
})
await new Promise(resolve => echo.listen(0, '127.0.0.1', resolve))
const echoPort = echo.address().port

const pointed = await call('/config', { method: 'POST', body: JSON.stringify({ vncHost: '127.0.0.1', vncPort: echoPort }) })
check('the bridge target is configurable', pointed.status === 200 && pointed.payload.settings.vncPort === echoPort,
  `vncPort=${pointed.payload.settings?.vncPort}`)

/** Sized to hit each frame header: 7-bit, 16-bit, then 64-bit. */
const payloads = [100, 200, 70000].map(size => {
  const buffer = Buffer.alloc(size)
  for (let index = 0; index < size; index += 1) buffer[index] = index % 251
  return buffer
})

const bridged = await new Promise((resolve, reject) => {
  const socket = new WebSocket(`ws://127.0.0.1:${port}/api/dzdrone-ssh/vnc`)
  socket.binaryType = 'arraybuffer'
  // The stand-in speaks first: a 12-byte version banner then 3 bytes of security
  // types. Everything after those 15 bytes must be our payloads, echoed back.
  const PREAMBLE = 15
  const total = PREAMBLE + payloads.reduce((sum, item) => sum + item.length, 0)
  let all = Buffer.alloc(0)
  const timer = setTimeout(() => { try { socket.close() } catch { /* closed */ } ; reject(new Error('bridge timed out')) }, 25000)
  const settle = () => {
    if (all.length < total) return
    clearTimeout(timer)
    try { socket.close() } catch { /* closed */ }
    resolve({ preamble: all.subarray(0, PREAMBLE), echoed: all.subarray(PREAMBLE) })
  }
  socket.onopen = () => { for (const payload of payloads) socket.send(new Uint8Array(payload)) }
  socket.onmessage = event => { all = Buffer.concat([all, Buffer.from(event.data)]); settle() }
  socket.onerror = () => { clearTimeout(timer); reject(new Error('websocket errored')) }
})

check('the tunnel carries the server banner through',
  bridged.preamble.toString('latin1').startsWith('RFB 003.008'), JSON.stringify(bridged.preamble.toString('latin1')))

const expected = Buffer.concat(payloads)
check('large payloads survive the tunnel byte-for-byte (7/16/64-bit frames)',
  bridged.echoed.length === expected.length && bridged.echoed.equals(expected),
  `sent=${expected.length}B got=${bridged.echoed.length}B`)

echo.close()

// The drone's own VNC is probed only when asked for, because of the blacklist
// side effect described above.
if (process.env.DZD_TEST_DRONE_VNC === '1') {
  console.log('\n--- vnc bridge (the real drone, opt-in) ---')
  await call('/config', { method: 'POST', body: JSON.stringify({ vncHost: '', vncPort: 5901 }) })
  const real = await new Promise((resolve, reject) => {
    const socket = new WebSocket(`ws://127.0.0.1:${port}/api/dzdrone-ssh/vnc`)
    socket.binaryType = 'arraybuffer'
    const timer = setTimeout(() => { try { socket.close() } catch { /* closed */ } ; reject(new Error('no banner from the drone')) }, 20000)
    socket.onopen = () => socket.send(new TextEncoder().encode('RFB 003.008\n'))
    socket.onmessage = event => {
      clearTimeout(timer)
      const bytes = Buffer.from(event.data)
      try { socket.close() } catch { /* closed */ }
      resolve(bytes.toString('latin1'))
    }
    socket.onerror = () => { clearTimeout(timer); reject(new Error('websocket errored')) }
  }).catch(error => `ERROR: ${error.message}`)
  check('the drone answers through the bridge (opt-in)', /^RFB \d{3}\.\d{3}\n/.test(real), JSON.stringify(String(real).slice(0, 60)))
}

// ── the status float: its data route and its standalone window ────────────────
// /metrics always answers 200 with an `ok` flag: an unreachable drone is a
// reading the card renders as an error line, not an HTTP failure to hide
// behind — a 503 would make the float's own error path unreachable.
const metrics = await call('/metrics')
console.log('\nGET /metrics ->', metrics.status, JSON.stringify(metrics.payload).slice(0, 160))
check('GET /metrics answers 200 with a boolean ok flag',
  metrics.status === 200 && typeof metrics.payload?.ok === 'boolean',
  `status=${metrics.status} ok=${metrics.payload?.ok}`)
if (metrics.payload?.ok === true) {
  const reading = metrics.payload
  const gauges = ['cpuPercent', 'uptimeSec', 'bootEpoch', 'at', 'load', 'zones'].every(key => reading[key] !== undefined)
  check('a live reading carries the gauges the card paints',
    gauges && (reading.temp === null || Number.isFinite(reading.temp)),
    JSON.stringify({ cpu: reading.cpuPercent, up: reading.uptimeSec, temp: reading.temp, zones: reading.zones }))
  // The battery is optional on a live reading: it is absent (never 0%) while
  // the PX4 flight controller is unplugged.
  if ('battery' in reading) {
    check('a battery reading is well-formed when present',
      Number.isFinite(reading.battery?.percent) && reading.battery.percent >= 0 && reading.battery.percent <= 100
        && Number.isFinite(reading.battery?.voltage) && reading.battery.voltage > 0,
      JSON.stringify(reading.battery))
  } else {
    check('a live reading may omit battery (flight controller absent)',
      reading.ok === true, 'battery field absent — FC unplugged or echo timed out')
  }
  // The card adds `now - at` to the polled uptime so the seconds tick between
  // polls; a stale stamp would make the gauge jump backwards on every refresh.
  check('the reading is stamped fresh enough to tick uptime against',
    Number.isFinite(reading.at) && Math.abs(reading.at - Date.now()) < 60_000,
    `at=${reading.at} skew=${Date.now() - reading.at}ms`)
} else {
  check('an offline drone still yields an explainable reading', typeof metrics.payload?.detail === 'string' || typeof metrics.payload?.state === 'string', JSON.stringify(metrics.payload).slice(0, 120))
}

// The float's opacity is clamped server-side, whatever the client posts. A bad
// value here would be the worst kind of bug: `opacity: NaN` renders a card you
// can click but cannot see, and nothing reports an error anywhere.
const opacityHigh = await call('/config', { method: 'POST', body: JSON.stringify({ floatOpacity: 999 }) })
check('POST /config floatOpacity 999 clamps to 100',
  opacityHigh.status === 200 && opacityHigh.payload?.settings?.floatOpacity === 100,
  JSON.stringify(opacityHigh.payload?.settings?.floatOpacity))
const opacityLow = await call('/config', { method: 'POST', body: JSON.stringify({ floatOpacity: 5 }) })
check('POST /config floatOpacity 5 clamps to the 30 floor',
  opacityLow.status === 200 && opacityLow.payload?.settings?.floatOpacity === 30,
  JSON.stringify(opacityLow.payload?.settings?.floatOpacity))
const opacityGarbage = await call('/config', { method: 'POST', body: JSON.stringify({ floatOpacity: 'opaque' }) })
check('POST /config floatOpacity "opaque" falls back to 100',
  opacityGarbage.status === 200 && opacityGarbage.payload?.settings?.floatOpacity === 100,
  JSON.stringify(opacityGarbage.payload?.settings?.floatOpacity))
// Last write before the round trip must be the one /status is checked against.
const opacityLast = await call('/config', { method: 'POST', body: JSON.stringify({ floatOpacity: 5 }) })
check('the clamped value is what /status hands back to the client',
  opacityLast.status === 200 && opacityLast.payload?.settings?.floatOpacity === 30,
  JSON.stringify(opacityLast.payload?.settings?.floatOpacity))
const opacityRoundTrip = await call('/status')
check('GET /status reports the persisted floatOpacity',
  opacityRoundTrip.status === 200 && opacityRoundTrip.payload?.settings?.floatOpacity === 30,
  JSON.stringify(opacityRoundTrip.payload?.settings?.floatOpacity))
await call('/config', { method: 'POST', body: JSON.stringify({ floatOpacity: 100 }) })

const floatPage = await fetch(`${base}/float.html`)
const floatHtml = await floatPage.text()
check('GET /float.html serves the standalone float window',
  floatPage.status === 200 && /text\/html/.test(floatPage.headers.get('content-type') ?? '') && floatHtml.includes('/metrics'),
  `status=${floatPage.status} type=${floatPage.headers.get('content-type')} bytes=${floatHtml.length}`)

// Leave the drone as it was found — but only when this run put something
// there: the fixtures belong to the opt-in write probes, and WORKFLOW.md
// rule 2 forbids deleting files on the drone when none were created.
if (mayWrite) {
  const { execFileSync } = await import('node:child_process')
  const droneUser = (await call('/status')).payload.settings?.user ?? 'duzhong'
  const droneHost = (await call('/status')).payload.settings?.host ?? 'dzsmart.local'
  try {
    execFileSync('ssh', ['-o', 'BatchMode=yes', `${droneUser}@${droneHost}`,
      'rm -f /tmp/dsh-upload-probe.txt /tmp/dsh-bin-host-test.bin'], { stdio: 'ignore' })
  } catch { /* cleanup is best effort */ }
}

server.close()
console.log('\nlog tail:', logs.slice(-4).join(' | '))
console.log(failures === 0 ? '\nHOST TEST PASSED' : `\n${failures} HOST CHECK(S) FAILED`)
process.exitCode = failures === 0 ? 0 : 1
