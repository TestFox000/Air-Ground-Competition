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
process.env.DSH_HOME = fs.mkdtempSync(path.join(os.tmpdir(), 'dzrobot-ssh-test-'))

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
const base = `http://127.0.0.1:${port}/api/dzrobot-ssh`

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
    host: '127.0.0.1', port, path: '/api/dzrobot-ssh/novnc/%2e%2e/%2e%2e/package.json', method: 'GET',
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
  connectHtml.includes("source: 'dzrobot-vnc-connect'"), 'protocol marker not found')
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
  const socket = new WebSocket(`ws://127.0.0.1:${port}/api/dzrobot-ssh/vnc`)
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
    const socket = new WebSocket(`ws://127.0.0.1:${port}/api/dzrobot-ssh/vnc`)
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
