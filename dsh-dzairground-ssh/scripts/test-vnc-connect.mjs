/**
 * The generic connector document's own logic, exercised headlessly.
 *
 * A browser would be the ideal host for this, but the module script of
 * `vnc-connect.html` is self-contained enough to run against stubs: this test
 * extracts it, strips the one `import`, and drives it through the exact
 * postMessage protocol the panel uses. It verifies the facts that decide
 * whether the connector works at all:
 *
 *   · the ws:// URL is built from host/port/path (the whole point of the tool);
 *   · credentials and viewport options reach the RFB constructor;
 *   · a fixed resolution sizes the screen element and flips resizeSession back
 *     off afterwards (the SetDesktopSize trick);
 *   · 'auto' keeps follow-window on;
 *   · scale toggling reaches the live RFB instance;
 *   · credentialsrequired/securityfailure reach the panel as reports.
 *
 * Run: node scripts/test-vnc-connect.mjs
 */

import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'
import assert from 'node:assert/strict'

const here = path.dirname(fileURLToPath(import.meta.url))
const html = fs.readFileSync(path.join(here, '..', 'vnc-connect.html'), 'utf8')

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

const scriptStart = html.indexOf('<script type="module">')
const scriptEnd = html.indexOf('</script>', scriptStart)
if (scriptStart === -1 || scriptEnd === -1) {
  console.error('FAIL  vnc-connect.html has no module script')
  process.exit(1)
}
let body = html.slice(scriptStart + '<script type="module">'.length, scriptEnd)
body = body.replace(/import RFB from '\.\/novnc\/core\/rfb\.js'\n/, '')
if (body.includes("import RFB")) {
  console.error('FAIL  could not strip the RFB import; test out of sync with the document')
  process.exit(1)
}

// ── browser stubs ──────────────────────────────────────────────────────────────
/** One element with just the surface the document touches. */
const element = id => ({
  id,
  value: '',
  checked: true,
  hidden: false,
  textContent: '',
  dataset: {},
  style: {},
  listeners: {},
  addEventListener(name, handler) { this.listeners[name] = handler },
  focus() {},
})

const elements = {}
for (const id of ['scheme', 'host', 'port', 'path', 'pw', 'resolution', 'scale',
  'bar', 'stop', 'fs', 'askbox', 'ask', 'asktext', 'askpw', 'asksend', 'status',
  'screen', 'stage']) {
  elements[id] = element(id)
}
const documentStub = {
  getElementById: id => elements[id] ?? element(id),
  documentElement: {
    requestFullscreen: () => new Promise(resolve => resolve()),
  },
}

// The RFB instances this run creates, so the test can drive their events.
const instances = []
class RfbStub {
  constructor(target, url, options) {
    this.target = target
    this.url = url
    this.options = options
    this.resizeSession = false
    this.scaleViewport = true
    this.handlers = {}
    this.disconnected = false
    this.sentCredentials = []
    instances.push(this)
  }
  addEventListener(name, handler) { this.handlers[name] = handler }
  emit(name, detail = {}) { this.handlers[name]?.({ detail }) }
  disconnect() { this.disconnected = true }
  sendCredentials(credentials) { this.sentCredentials.push(credentials) }
  sendCtrlAltDel() { this.cad = true }
}

// The messages the document posts to its parent (the panel side of the protocol).
const posted = []
const parent = { postMessage: (data, origin) => posted.push({ data, origin }) }
const windowListeners = {}
const win = {
  parent,
  addEventListener: (name, handler) => { windowListeners[name] = handler },
  removeEventListener: () => {},
  dispatchEvent: () => {},
  setTimeout: (...args) => setTimeout(...args),
  WebSocket: globalThis.WebSocket,
}
// The document reads these as globals; Node has neither.
const locationStub = { search: '', href: 'http://127.0.0.1:19387/api/dzairground-ssh/vnc-connect.html?embedded=1', protocol: 'http:' }

// ── evaluate the shipped module body ───────────────────────────────────────────
check('the shipped connector script evaluates against the stubs', () => {
  const factory = new Function('RFB', 'window', 'document', 'location', body)
  factory(RfbStub, win, documentStub, locationStub)
})

/** Hand the document one message, exactly as the panel would. */
const send = data => windowListeners.message?.({ data })

/** What the document reported to the parent, by type, in order. */
const reports = type => posted.map(entry => entry.data).filter(data => data.type === type)

check('the document reports ready with its own protocol name', () => {
  const ready = reports('ready')
  if (ready.length !== 1) throw new Error(`expected one ready report, got ${JSON.stringify(posted)}`)
  if (ready[0].source !== 'dzairground-vnc-connect') throw new Error(`wrong source: ${JSON.stringify(ready[0])}`)
})

check('a panel command whose sender is not the panel is ignored', () => {
  const before = posted.length
  send({ source: 'someone-else', type: 'connect' })
  if (posted.length !== before) throw new Error('an unrelated sender was acted upon')
  if (instances.length !== 0) throw new Error('an unrelated sender created an RFB instance')
})

// ── the connect flow ───────────────────────────────────────────────────────────
send({ source: 'dzairground-panel', type: 'config', config: {
  scheme: 'ws', host: 'ubuntu-box', port: '6080', path: 'websockify', password: 'secret', resolution: 'auto',
} })
send({ source: 'dzairground-panel', type: 'connect', config: {
  scheme: 'ws', host: 'ubuntu-box', port: '6080', path: 'websockify', password: 'secret', resolution: 'auto',
} })

check('connect builds the ws URL from host/port/path', () => {
  const instance = instances.at(-1)
  if (instance === undefined) throw new Error('no RFB instance was created')
  if (instance.url !== 'ws://ubuntu-box:6080/websockify') {
    throw new Error(`url was ${JSON.stringify(instance.url)}`)
  }
})

check('credentials and viewport options reach the RFB constructor', () => {
  const instance = instances.at(-1)
  assert.equal(instance.options.credentials.password, 'secret')
  assert.equal(instance.options.scaleViewport, true, 'scale checkbox default is on')
  assert.equal(instance.options.resizeSession, false, 'resolution is applied after connect, not at construction')
  assert.equal(instance.options.wsProtocols[0], 'binary')
})

check('connecting is reported to the panel', () => {
  const seen = reports('connecting')
  if (seen.length < 1) throw new Error('no connecting report')
})

check('a wrong or missing password still connects without credentials', () => {
  send({ source: 'dzairground-panel', type: 'connect', config: {
    scheme: 'ws', host: 'h', port: '1', path: 'p', password: '', resolution: 'auto',
  } })
  const instance = instances.at(-1)
  assert.equal(instance.options.credentials, undefined)
})

// ── resolution behaviour, driven through real events ───────────────────────────
await new Promise(resolve => setTimeout(resolve, 50))

// 'auto' turns follow-window on once connected.
{
  const instance = instances.at(-1)
  instance.emit('connect')
  await new Promise(resolve => setTimeout(resolve, 900))
  check("resolution 'auto' enables follow-window after connect", () => {
    if (instance.resizeSession !== true) {
      throw new Error(`resizeSession should be on for auto, was ${instance.resizeSession}`)
    }
  })
}

// A fixed resolution sizes the element (so noVNC requests SetDesktopSize), then
// restores fit and turns follow-window back off.
{
  send({ source: 'dzairground-panel', type: 'resolution', value: '1920x1080' })
  const instance = instances.at(-1)
  instance.emit('connect')
  await new Promise(resolve => setTimeout(resolve, 60))
  check('a fixed resolution sizes the screen element to WxH', () => {
    assert.equal(elements.screen.style.width, '1920px')
    assert.equal(elements.screen.style.height, '1080px')
  })
  await new Promise(resolve => setTimeout(resolve, 400))
  check('…then restores fit-to-window and stops following it', () => {
    assert.equal(elements.screen.style.width, '100%')
    assert.equal(instance.resizeSession, false, 'fixed resolution must stop follow-window')
  })
}

// ── scale toggling ─────────────────────────────────────────────────────────────
check('a scale command reaches the live RFB instance', () => {
  send({ source: 'dzairground-panel', type: 'scale', value: false })
  const instance = instances.at(-1)
  assert.equal(instance.scaleViewport, false)
  send({ source: 'dzairground-panel', type: 'scale', value: true })
  assert.equal(instance.scaleViewport, true)
})

// ── password events ────────────────────────────────────────────────────────────
{
  const before = reports('credentials').length
  instances.at(-1).emit('credentialsrequired')
  check('credentialsrequired is reported to the panel', () => {
    if (reports('credentials').length !== before + 1) throw new Error('no credentials report')
  })
  check('the panel can answer with sendCredentials', () => {
    send({ source: 'dzairground-panel', type: 'password', password: 'pw-2' })
    const sent = instances.at(-1).sentCredentials.at(-1)
    assert.equal(sent?.password, 'pw-2')
  })
}

check('a security failure is reported with its blacklist classification', () => {
  const before = reports('failed').length
  instances.at(-1).emit('securityfailure', { reason: 'Too many security failures' })
  const failed = reports('failed')[before]
  if (failed === undefined) throw new Error('no failed report')
  assert.equal(failed.blacklisted, true)
  assert.equal(failed.kind, 'security')
})

// ── disconnect and Ctrl-Alt-Del ────────────────────────────────────────────────
check('cad reaches the live instance', () => {
  const instance = instances.at(-1)
  send({ source: 'dzairground-panel', type: 'cad' })
  if (instance?.cad !== true) throw new Error('Ctrl-Alt-Del was not forwarded')
})

check('disconnect tears the RFB instance down and reports it', () => {
  const before = reports('disconnected').length
  send({ source: 'dzairground-panel', type: 'disconnect' })
  if (instances.at(-1)?.disconnected !== true) throw new Error('instance was not disconnected')
  if (reports('disconnected').length <= before) throw new Error('no disconnected report')
})

if (failures.length > 0) {
  console.log('\nFAILURES:')
  for (const failure of failures) console.log(' -', failure)
  process.exitCode = 1
} else {
  console.log('\nVNC CONNECT DOCUMENT TEST PASSED')
}
