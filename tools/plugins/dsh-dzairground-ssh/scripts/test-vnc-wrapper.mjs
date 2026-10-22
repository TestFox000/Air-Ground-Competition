/**
 * The WebSocket probes in the shipped HTML documents must survive noVNC's own
 * channel check — in BOTH of them.
 *
 * noVNC's `Websock.attach()` duck-types the channel it is handed with
 *   Object.getOwnPropertyNames(Object.getPrototypeOf(channel))
 * which inspects exactly ONE level of the prototype chain. Wrapping the socket in
 * a subclass therefore hides `send`/`close`/`binaryType`/`readyState` on
 * `WebSocket.prototype` — the second level — and noVNC refuses the socket with
 * "Raw channel missing property: send". Because that throw comes out of the RFB
 * constructor, the socket is left unreachable and gets collected, so the browser
 * reports only `error` + close code 1006 and never `open`: an undiagnosable
 * failure that looks exactly like the target being offline.
 *
 * This runs the *shipped* block (extracted from each HTML between its markers)
 * through the *real* `Websock.attach`, so the mistake cannot come back silently
 * in either document. It also checks that a subclass really does fail, so the
 * test cannot pass by being vacuous.
 *
 * Run: node scripts/test-vnc-wrapper.mjs
 */

import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

// noVNC's logging module reads `window.console` the moment it is imported, so a
// browser-ish global must exist first. It is the only environment dependency in
// the path this test exercises — `Websock.attach` itself is pure inspection.
globalThis.window = { console, Error }

const { default: Websock } = await import('../novnc/core/websock.js')

const here = path.dirname(fileURLToPath(import.meta.url))
const DOCUMENTS = ['vnc-app.html', 'vnc-connect.html']

const failures = []
const check = (label, run) => {
  try {
    run()
    console.log(`OK    ${label}`)
  } catch (error) {
    failures.push(`${label}: ${error?.message ?? error}`)
    console.log(`FAIL  ${label}\n      ${error?.message ?? error}`)
  }
}

// A port nothing listens on: `attach()` only inspects the object, so no traffic
// is needed, and every socket is closed at the end.
const DEAD = 'ws://127.0.0.1:1/'
const opened = []
const make = Ctor => {
  const socket = new Ctor(DEAD)
  opened.push(socket)
  socket.addEventListener('error', () => {})
  return socket
}

/** Attach through noVNC's real code path; returns the thrown error, if any. */
const attach = socket => {
  const sock = new Websock()
  try {
    sock.attach(socket)
    return undefined
  } catch (error) {
    return error
  }
}

// ── 1. a plain native socket passes (proves this environment can run the check) ─
check('a native WebSocket passes noVNC\'s channel check (baseline)', () => {
  const error = attach(make(globalThis.WebSocket))
  if (error !== undefined) {
    throw new Error(`baseline failed, so this environment cannot validate the probes: ${error.message}`)
  }
})

// ── 2. a subclass is rejected — the bug this test exists for ─────────────────
check('a subclassed WebSocket is rejected (the bug being guarded)', () => {
  class NaiveSubclass extends globalThis.WebSocket {}
  const error = attach(make(NaiveSubclass))
  if (error === undefined) {
    throw new Error('a subclass passed, so noVNC\'s check no longer sees one prototype level — revisit the wrappers')
  }
  if (!/missing property/.test(String(error.message))) {
    throw new Error(`a subclass failed for an unexpected reason: ${error.message}`)
  }
})

// ── 3. every shipped document's probe, extracted by its markers ──────────────
for (const document of DOCUMENTS) {
  const html = fs.readFileSync(path.join(here, '..', document), 'utf8')
  const start = html.indexOf('/* ws-log:start')
  const end = html.indexOf('/* ws-log:end */')
  if (start === -1 || end === -1 || end < start) {
    console.error(`FAIL  ${document} no longer contains a ws-log:start/ws-log:end block`)
    console.error('      the probe is extracted by those markers; restore them or update this test')
    process.exit(1)
  }
  const block = html.slice(html.indexOf('\n', start) + 1, end)

  const win = { WebSocket: globalThis.WebSocket }
  let Wrapped
  let wsLog
  try {
    const extracted = new Function('window', `${block}\nreturn { ctor: window.WebSocket, log: wsLog };`)(win)
    Wrapped = extracted.ctor
    wsLog = extracted.log
  } catch (error) {
    console.error(`FAIL  the ws-log block in ${document} does not evaluate: ${error?.message ?? error}`)
    process.exit(1)
  }
  if (typeof Wrapped !== 'function') {
    console.error(`FAIL  the ws-log block in ${document} did not replace window.WebSocket with a constructor`)
    process.exit(1)
  }

  check(`${document}: the probe returns a native instance, not a subclass`, () => {
    const socket = make(Wrapped)
    if (Object.getPrototypeOf(socket) !== globalThis.WebSocket.prototype) {
      throw new Error('the probe inserted a prototype level; noVNC inspects only one')
    }
    if (!(socket instanceof globalThis.WebSocket)) {
      throw new Error('the probe\'s socket is not instanceof WebSocket')
    }
  })

  check(`${document}: a socket from the probe passes noVNC's channel check`, () => {
    const error = attach(make(Wrapped))
    if (error !== undefined) throw error
  })

  // ── the probe still records the lifecycle it exists to record ─────────────
  check(`${document}: the probe records the constructor with URL and protocols`, () => {
    wsLog.length = 0
    const socket = make(Wrapped)
    void socket
    const ctor = wsLog.find(entry => entry.e === 'ctor')
    if (ctor === undefined) throw new Error(`no ctor entry was logged: ${JSON.stringify(wsLog)}`)
    if (ctor.url !== DEAD) throw new Error(`ctor logged the wrong url: ${JSON.stringify(ctor.url)}`)
    if (ctor.at === undefined) throw new Error('ctor entry has no timestamp, so causality cannot be read from it')
  })

  check(`${document}: the probe records an explicit protocols list`, () => {
    const socket = new Wrapped(DEAD, ['binary', 'base64'])
    opened.push(socket)
    const ctor = [...wsLog].reverse().find(entry => entry.e === 'ctor')
    if (JSON.stringify(ctor?.protocols) !== JSON.stringify(['binary', 'base64'])) {
      throw new Error(`protocols were not recorded as an array: ${JSON.stringify(ctor?.protocols)}`)
    }
  })
}

for (const socket of opened) {
  try { socket.close() } catch { /* already gone */ }
}

if (failures.length > 0) {
  console.log('\nFAILURES:')
  for (const failure of failures) console.log(' -', failure)
  process.exit(1)
}
console.log('\nVNC WRAPPER TEST PASSED')
process.exit(0)
