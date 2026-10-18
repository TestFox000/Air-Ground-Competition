/**
 * Host-half integration test with a stub cordis context.
 *
 * Boots `apply()` against a fake ctx, captures the route it registers on the
 * stubbed `webServer`, serves them on loopback, and exercises every endpoint —
 * including a real SSH probe against the car. Run:
 *   node scripts/test-host.mjs
 */

import http from 'node:http'
import net from 'node:net'
import fs from 'node:fs'
import os from 'node:os'
import path from 'node:path'

// The plugin resolves its data directory from DSH_HOME at apply() time, and this
// test writes settings and diagnostics. Point it at a throwaway directory so a
// test run can never leave the user's real car address rewritten.
process.env.DSH_HOME = fs.mkdtempSync(path.join(os.tmpdir(), 'dzairground-ssh-test-'))

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
const base = `http://127.0.0.1:${port}/api/dzairground-ssh`

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

const config = await call('/config', { method: 'POST', body: JSON.stringify({ port: 2222, host: 'duzhong.local' }) })
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
// the whole contract: two /proc/stat samples become a delta, /proc/uptime is
// the source, btime rides along when present (and is dropped when absurd), the
// chassis battery is believed at both published scales (0..1 and 0..100),
// volts without percent become a *derived* level with the note that says so,
// and a truncated payload is a failed reading — never a plausible zero.
{
  const { parseMetrics, normalizeBattery, cpuUsage, METRICS_COMMAND } = await import('../src/metrics.js')
  const payload = lines => lines.join('\n')
  const full = parseMetrics(payload([
    'Welcome to Ubuntu 22.04 (banner noise outside the markers)',
    'DZH_BEGIN',
    '100 0 50 900 0 0 0 0 0',
    '500 0 100 2400 0 0 0 0 0',
    'uptimeSec=666.97',
    'bootEpoch=1791338271',
    'load1=0.33 load5=0.25 load15=0.20',
    'batteryPct=0.86',
    'batteryVolt=11.42',
    'DZH_END',
    'trailing noise',
  ]))
  check('parser: a full payload answers every gauge',
    full !== undefined && Math.abs(full.uptimeSec - 666.97) < 1e-9
      && full.bootEpoch === 1791338271 && full.battery === 86 && full.batteryVolt === 11.42
      && full.batteryNote === null && Array.isArray(full.load) && Number.isFinite(full.cpuPercent),
    JSON.stringify(full?.load) + ` batt=${full?.battery} cpu=${full?.cpuPercent}`)
  // The chassis already publishing percent (0..100 scale) must not read as 1 %.
  const percentScale = parseMetrics(payload([
    'DZH_BEGIN', '100 0 50 900 0 0 0 0 0', '500 0 100 2400 0 0 0 0 0',
    'uptimeSec=42', 'batteryPct=77', 'DZH_END',
  ]))
  check('parser: an already-percent battery reads as itself', percentScale?.battery === 77, JSON.stringify(percentScale?.battery))
  // Stack down: voltage topic alone still answers, via the pack calibration,
  // and the note says the number is derived rather than chassis-published.
  const derived = parseMetrics(payload([
    'DZH_BEGIN', '100 0 50 900 0 0 0 0 0', '500 0 100 2400 0 0 0 0 0',
    'uptimeSec=42', 'batteryVolt=11', 'batteryNote=no_stack', 'DZH_END',
  ]))
  check('parser: volts without percent derive 50 % and mark it derived',
    derived?.battery === 50 && derived?.batteryNote === 'derived', JSON.stringify({ battery: derived?.battery, note: derived?.batteryNote }))
  // Stack down and no voltage answer either: null with the reason, never 0 %.
  const noStack = parseMetrics(payload([
    'DZH_BEGIN', '100 0 50 900 0 0 0 0 0', '500 0 100 2400 0 0 0 0 0',
    'uptimeSec=42', 'batteryNote=no_stack', 'DZH_END',
  ]))
  check('parser: no stack yields battery null with the note (not a fake 0)',
    noStack?.battery === null && noStack?.batteryNote === 'no_stack', JSON.stringify({ battery: noStack?.battery, note: noStack?.batteryNote }))
  const noBtime = parseMetrics(payload([
    'DZH_BEGIN', '100 0 50 900 0 0 0 0 0', '500 0 100 2400 0 0 0 0 0',
    'uptimeSec=42', 'DZH_END',
  ]))
  check('parser: an answer without btime still parses, with bootEpoch absent',
    noBtime !== undefined && noBtime.uptimeSec === 42 && !('bootEpoch' in noBtime),
    JSON.stringify(noBtime))
  const absurd = parseMetrics(payload([
    'DZH_BEGIN', '100 0 50 900 0 0 0 0 0', '500 0 100 2400 0 0 0 0 0',
    'uptimeSec=42', 'bootEpoch=123456789', 'DZH_END',
  ]))
  check('parser: a boot instant before 2000 is dropped, not printed',
    absurd !== undefined && !('bootEpoch' in absurd), JSON.stringify(absurd?.bootEpoch))
  const truncated = parseMetrics('DZH_BEGIN\n100 0 50 900 0 0 0 0 0\nuptimeSec=99\n')
  check('parser: a payload missing the end marker is a failed reading, not a zero',
    truncated === undefined, String(truncated))

  // The battery scale itself: both published shapes accepted, junk rejected
  // instead of clamped into the gauge.
  for (const [raw, want] of [[0.86, 86], [0, 0], [1, 100], [77, 77], [100, 100], [101, null], [-1, null], ['abc', null]]) {
    const got = normalizeBattery(raw)
    check(`normalizeBattery(${JSON.stringify(raw)}) -> ${want}`, got === want, `got ${got}`)
  }
  // CPU is a delta: identical samples are 0 %, never a division by zero.
  const before = [100, 0, 50, 900, 0, 0, 0, 0, 0]
  const after = [500, 0, 100, 2400, 0, 0, 0, 0, 0]
  check('cpuUsage between two samples is the idle delta (23.1 %)', cpuUsage(before, after) === 23.1, String(cpuUsage(before, after)))
  check('a clock that did not advance is 0 %, not NaN', cpuUsage(before, before) === 0, String(cpuUsage(before, before)))

  // The remote command itself must carry the battery half — without these
  // markers the float would render "no stack" forever on a perfectly healthy car.
  for (const needle of ['DZH_BEGIN', 'batteryPct', 'Battery_Percentage', 'PowerVoltage', '/proc/stat', '/proc/uptime']) {
    check(`METRICS_COMMAND carries ${needle}`, METRICS_COMMAND.includes(needle), '')
  }

  // ── bug#3: an out-of-range percentage is 数据异常, not "stack down" ──────────
  // The chassis answered, the number was junk: the note must be bad_value so
  // the UI says 数据异常 instead of blaming the ROS stack (or deriving from
  // volts as if the percentage topic had been silent).
  const outOfRange = parseMetrics(payload([
    'DZH_BEGIN', '100 0 50 900 0 0 0 0 0', '500 0 100 2400 0 0 0 0 0',
    'uptimeSec=42', 'batteryPct=150', 'batteryVolt=11.5', 'DZH_END',
  ]))
  check('parser: an out-of-range battery percentage reads as bad_value (bug#3)',
    outOfRange?.battery === null && outOfRange?.batteryNote === 'bad_value',
    JSON.stringify({ battery: outOfRange?.battery, note: outOfRange?.batteryNote }))
  const badOnly = parseMetrics(payload([
    'DZH_BEGIN', '100 0 50 900 0 0 0 0 0', '500 0 100 2400 0 0 0 0 0',
    'uptimeSec=42', 'batteryPct=150', 'DZH_END',
  ]))
  check('parser: bad_value wins over derived even with volts present',
    badOnly?.battery === null && badOnly?.batteryNote === 'bad_value',
    JSON.stringify({ battery: badOnly?.battery, note: badOnly?.batteryNote }))
}

// ── the drone node's metrics parser (offline, synthetic payloads) ──────────────
// Ported from the frozen drone plugin; the cases mirror its own contract plus
// the two words S3 adds: an out-of-range FC reading is bad_value (bug#3), and
// the three-state vocabulary (决议②) is present on the Host side.
{
  const { parseDroneMetrics, METRICS_COMMAND, cpuUsage } = await import('../src/metrics-drone.js')
  const cpu = ['100 0 50 900 0 0 0 0 0', '500 0 100 2400 0 0 0 0 0']
  const payload = lines => lines.join('\n')
  const full = parseDroneMetrics(payload([
    'noise outside the markers',
    'DZH_BEGIN', ...cpu,
    'temp_cpu-thermal=61200', 'temp_gpu-thermal=55000',
    'uptimeSec=666.97', 'bootEpoch=1791338271',
    'load1=0.33 load5=0.25 load15=0.20',
    'batteryFc=present', 'batteryPercent=65', 'batteryVoltage=11.9',
    'DZH_END',
  ]))
  check('drone parser: a full payload answers temp, zones and the FC battery',
    full !== undefined && full.temp === 61.2 && full.tempAt === 'cpu-thermal' && full.zones === 2
      && full.battery?.percent === 65 && full.battery?.voltage === 11.9 && full.batteryNote === undefined
      && Number.isFinite(full.uptimeSec) && Number.isFinite(full.cpuPercent),
    JSON.stringify({ temp: full?.temp, battery: full?.battery, zones: full?.zones }))
  check('drone parser: METRICS_COMMAND gates on the FC device and names the topics',
    METRICS_COMMAND.includes('ttyACM') && METRICS_COMMAND.includes('fmu/out/battery_status')
      && METRICS_COMMAND.includes('dds_silent') && METRICS_COMMAND.includes('no_ros'), '')
  // 决议② on the Host side: a plugged-in board with a silent topic rides as
  // batteryNote=dds_silent — never as a fabricated percentage.
  const silent = parseDroneMetrics(payload([
    'DZH_BEGIN', ...cpu, 'uptimeSec=42', 'batteryFc=present', 'batteryNote=dds_silent', 'DZH_END',
  ]))
  check('drone parser: FC present + DDS silent -> batteryNote=dds_silent (飞控无数据)',
    silent?.battery === undefined && silent?.batteryNote === 'dds_silent',
    JSON.stringify({ battery: silent?.battery, note: silent?.batteryNote }))
  const noRos = parseDroneMetrics(payload([
    'DZH_BEGIN', ...cpu, 'uptimeSec=42', 'batteryFc=present', 'batteryNote=no_ros', 'DZH_END',
  ]))
  check('drone parser: FC present + no ROS 2 -> batteryNote=no_ros',
    noRos?.battery === undefined && noRos?.batteryNote === 'no_ros',
    JSON.stringify({ note: noRos?.batteryNote }))
  const noFc = parseDroneMetrics(payload([
    'DZH_BEGIN', ...cpu, 'uptimeSec=42', 'batteryFc=absent', 'DZH_END',
  ]))
  check('drone parser: no FC board -> no battery, no note, batteryFc=absent rides along (无飞控)',
    noFc?.battery === undefined && noFc?.batteryNote === undefined && noFc?.batteryFc === 'absent', JSON.stringify(noFc))
  // bug#3's drone-side word: a topic that ANSWERED with an out-of-range
  // number is bad_value (数据异常), not dds_silent (the topic was not silent).
  const badValue = parseDroneMetrics(payload([
    'DZH_BEGIN', ...cpu, 'uptimeSec=42', 'batteryFc=present', 'batteryPercent=150', 'batteryVoltage=11.9', 'DZH_END',
  ]))
  check('drone parser: an out-of-range FC reading reads as bad_value (bug#3)',
    badValue?.battery === undefined && badValue?.batteryNote === 'bad_value',
    JSON.stringify({ note: badValue?.batteryNote }))
  check('drone parser: a payload missing the end marker is a failed reading',
    parseDroneMetrics('DZH_BEGIN\n' + cpu.join('\n') + '\nuptimeSec=99\n') === undefined, '')
  check('drone parser: cpuUsage is the same idle-delta contract as the car',
    cpuUsage([100, 0, 50, 900, 0, 0, 0, 0, 0], [500, 0, 100, 2400, 0, 0, 0, 0, 0]) === 23.1, '')
}

// ── the float's endpoints ─────────────────────────────────────────────────────
// /metrics always answers 200 with an `ok` flag: an unreachable car is an
// answer the float renders (as an error line), not an HTTP failure to hide
// behind.
const metrics = await call('/metrics')
console.log('\nGET /metrics ->', metrics.status, JSON.stringify(metrics.payload).slice(0, 160))
check('GET /metrics answers 200 with a boolean ok flag',
  metrics.status === 200 && typeof metrics.payload?.ok === 'boolean',
  `status=${metrics.status} ok=${metrics.payload?.ok}`)
if (metrics.payload?.ok === true) {
  const reading = metrics.payload
  check('a live reading carries battery, uptime and cpu gauges',
    (reading.battery === null || typeof reading.battery === 'number')
      && Number.isFinite(reading.uptimeSec) && Number.isFinite(reading.cpuPercent)
      && [null, 'no_stack', 'derived', 'bad_value'].includes(reading.batteryNote),
    JSON.stringify({ battery: reading.battery, note: reading.batteryNote, up: reading.uptimeSec }))
} else {
  check('an offline car still yields an explainable reading',
    typeof metrics.payload?.detail === 'string' || typeof metrics.payload?.state === 'string',
    JSON.stringify(metrics.payload).slice(0, 120))
}

// The opacity is a persisted, clamped setting: the slider can only send
// 30..100, and anything else must land on a sane value instead of a
// transparent-to-invisible or over-opaque card.
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
// Round trip: /status is where the panel reads the value back after a reload.
const opacityRoundTrip = await call('/status')
check('GET /status reports the persisted floatOpacity',
  opacityRoundTrip.status === 200 && opacityRoundTrip.payload?.settings?.floatOpacity === 100,
  JSON.stringify(opacityRoundTrip.payload?.settings?.floatOpacity))

// The standalone window: a separate document (its own refresh button, no
// overlay needed) that polls the same /metrics the card does.
const floatPage = await fetch(`${base}/float.html`)
const floatHtml = await floatPage.text()
check('GET /float.html serves the standalone float window',
  floatPage.status === 200 && /text\/html/.test(floatPage.headers.get('content-type') ?? '') && floatHtml.includes('/metrics'),
  `${floatPage.status} ${floatPage.headers.get('content-type')} ${floatHtml.length}B`)

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
    host: '127.0.0.1', port, path: '/api/dzairground-ssh/novnc/%2e%2e/%2e%2e/package.json', method: 'GET',
  }, response => { response.resume(); resolve(response.statusCode) })
  request.on('error', () => resolve(-1))
  request.end()
})
check('encoded path traversal is refused', traversal === 403 || traversal === 404, String(traversal))

const notJs = await fetch(`${base}/novnc/LICENSE.txt`)
check('a text file is served, not refused', notJs.status === 200, String(notJs.status))

// ── the generic VNC connector document ────────────────────────────────────────
// Same origin as the car screen, different document: it dials ws://host:port/path
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
  connectHtml.includes("source: 'dzairground-vnc-connect'"), 'protocol marker not found')
check('the connector builds the ws URL from host/port/path',
  /cfg\.scheme.*cfg\.host.*cfg\.port.*cfg\.path/s.test(connectHtml), 'url builder not found')
check('the connector ships a resolution list with 自适应/auto',
  connectHtml.includes('value="auto"') && connectHtml.includes('2560x1440'), 'resolution options not found')
check('the connector keeps the ws-log probe markers',
  connectHtml.includes('/* ws-log:start') && connectHtml.includes('/* ws-log:end */'),
  'markers present — scripts/test-vnc-wrapper.mjs extracts them')

// ── the car's filesystem ──────────────────────────────────────────────────────
console.log('\n--- files ---')
const listing = await call('/files?path=/home/duzhong')
check('GET /files lists the home directory',
  listing.status === 200 && listing.payload.ok === true && listing.payload.entries.length > 5,
  `${listing.status} ${listing.payload.entries?.length} entries`)
check('listing is sorted with directories first',
  listing.payload.entries.length === 0 || listing.payload.entries[0].type === 'directory', '')
const missingDir = await call('/files?path=/nope/nope')
check('a missing directory is a 400 with a message', missingDir.status === 400 && missingDir.payload.error !== undefined,
  `${missingDir.status} ${missingDir.payload.error}`)
const file = await call('/file?path=/etc/hostname')
check('GET /file returns text', file.status === 200 && file.payload.text !== undefined,
  JSON.stringify(String(file.payload.text ?? '').trim()))
const statInfo = await call('/stat?path=/etc/hostname')
check('GET /stat reports a real size', statInfo.status === 200 && statInfo.payload.size > 0,
  `size=${statInfo.payload.size} type=${statInfo.payload.type}`)
// A small binary is uploaded rather than borrowed from the system, so the check
// does not depend on any particular file existing on the car.
await fetch(`${base}/upload?path=/tmp/dsh-bin-host-test.bin`, {
  method: 'POST',
  body: new Uint8Array([0, 1, 2, 3, 0, 255, 7, 0, 65, 0, 66]),
})
const binary = await call('/file?path=/tmp/dsh-bin-host-test.bin')
check('a small binary is flagged, never inlined as text',
  binary.payload.binary === true && binary.payload.text === undefined,
  JSON.stringify({ ok: binary.payload.ok, binary: binary.payload.binary, size: binary.payload.size }))

// Regression guard for the symlink bug: undereferenced, this path reports 22
// bytes and the reader then pulls the whole 2.7 MB target through.
const viaLink = await call('/file?path=/usr/bin/Xvnc')
check('a symlink to a huge file is refused by its real size',
  viaLink.status === 400 && viaLink.payload.text === undefined,
  `${viaLink.status} size=${viaLink.payload.size}`)
const download = await fetch(`${base}/download?path=/etc/hostname`)
const downloaded = await download.text()
check('GET /download streams the bytes',
  download.status === 200 && downloaded.trim() === 'duzhong',
  `${download.status} ${JSON.stringify(downloaded.trim())}`)
const upload = await fetch(`${base}/upload?path=/tmp/dsh-upload-probe.txt`, { method: 'POST', body: 'hello from the plugin test' })
const uploaded = await upload.json().catch(() => ({}))
check('POST /upload writes to the car', upload.status === 200 && uploaded.ok === true, JSON.stringify(uploaded))
const readBack = await call('/file?path=/tmp/dsh-upload-probe.txt')
check('the uploaded bytes read back identical',
  String(readBack.payload.text ?? '') === 'hello from the plugin test', JSON.stringify(readBack.payload.text))

// ── the VNC bridge ────────────────────────────────────────────────────────────
/*
 * Tested against a local TCP stand-in rather than the car's own VNC, and that is
 * a deliberate correction: TigerVNC counts an RFB connection that never completes
 * authentication as a security failure, so *probing the car* — even just to read
 * its banner and hang up — puts this machine on the car's blacklist and locks the
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
  const socket = new WebSocket(`ws://127.0.0.1:${port}/api/dzairground-ssh/vnc`)
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

// The car's own VNC is probed only when asked for, because of the blacklist
// side effect described above.
if (process.env.DZR_TEST_CAR_VNC === '1') {
  console.log('\n--- vnc bridge (the real car, opt-in) ---')
  await call('/config', { method: 'POST', body: JSON.stringify({ vncHost: '', vncPort: 5901 }) })
  const real = await new Promise((resolve, reject) => {
    const socket = new WebSocket(`ws://127.0.0.1:${port}/api/dzairground-ssh/vnc`)
    socket.binaryType = 'arraybuffer'
    const timer = setTimeout(() => { try { socket.close() } catch { /* closed */ } ; reject(new Error('no banner from the car')) }, 20000)
    socket.onopen = () => socket.send(new TextEncoder().encode('RFB 003.008\n'))
    socket.onmessage = event => {
      clearTimeout(timer)
      const bytes = Buffer.from(event.data)
      try { socket.close() } catch { /* closed */ }
      resolve(bytes.toString('latin1'))
    }
    socket.onerror = () => { clearTimeout(timer); reject(new Error('websocket errored')) }
  }).catch(error => `ERROR: ${error.message}`)
  check('the car answers through the bridge (opt-in)', /^RFB \d{3}\.\d{3}\n/.test(real), JSON.stringify(String(real).slice(0, 60)))
}

// Leave the car as it was found: these fixtures live in /tmp on a real robot.
const { execFileSync } = await import('node:child_process')
const carUser = (await call('/status')).payload.settings?.user ?? 'duzhong'
const carHost = (await call('/status')).payload.settings?.host ?? 'duzhong.local'
try {
  execFileSync('ssh', ['-o', 'BatchMode=yes', `${carUser}@${carHost}`,
    'rm -f /tmp/dsh-upload-probe.txt /tmp/dsh-bin-host-test.bin'], { stdio: 'ignore' })
} catch { /* cleanup is best effort */ }

server.close()
console.log('\nlog tail:', logs.slice(-4).join(' | '))
console.log(failures === 0 ? '\nHOST TEST PASSED' : `\n${failures} HOST CHECK(S) FAILED`)
process.exitCode = failures === 0 ? 0 : 1
