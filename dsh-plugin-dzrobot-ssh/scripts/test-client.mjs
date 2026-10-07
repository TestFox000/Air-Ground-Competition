/**
 * Client-half integration test, headless.
 *
 * Exercises the real connect flow against:
 *   · the real Host half, served on loopback (so a real SSH probe runs), and
 *   · a stub sidebar/terminal service pair that mimics the shipped behaviour
 *     the flow depends on: openTabFromTarget is void and the tab record appears
 *     asynchronously, and a terminal view only becomes writable once attached.
 *
 * Run: node scripts/test-client.mjs
 */

import http from 'node:http'
import fs from 'node:fs'
import os from 'node:os'
import path from 'node:path'

// The Host half this test boots resolves its data directory from DSH_HOME at
// apply() time. A throwaway directory keeps a test run from rewriting the real
// car address in ~/.dsh/dzrobot-ssh/settings.json.
process.env.DSH_HOME = fs.mkdtempSync(path.join(os.tmpdir(), 'dzrobot-ssh-client-test-'))

// ── stubs the browser provides ────────────────────────────────────────────────
const styles = []
const listeners = new Map()
globalThis.document = {
  documentElement: { lang: 'zh' },
  hidden: false,
  head: { appendChild: tag => styles.push(tag) },
  querySelector: () => null,
  createElement: () => ({ dataset: {}, style: {}, setAttribute() {}, appendChild() {} }),
  body: { closest: () => null },
  activeElement: undefined,
}
// Node 24 ships a getter-only `navigator`, so it has to be redefined rather than assigned.
Object.defineProperty(globalThis, 'navigator', {
  value: { clipboard: { writeText: async () => {} } },
  configurable: true,
  writable: true,
})
globalThis.setInterval = () => 0
globalThis.clearInterval = () => {}
globalThis.window = {
  __ModuleLoader__: {
    load: definition => listeners.set(definition.id, definition),
  },
}

let loaded
globalThis.window.__ModuleLoader__.load = definition => { loaded = definition }

// The module registers itself on import, so this must run after the window stub.
await import('../client.js')
if (loaded === undefined) throw new Error('client.js did not register with the module loader')

const client = loaded.factory(name => {
  if (name === 'react') {
    return {
      createElement: () => ({}),
      useState: value => [value, () => {}],
      useEffect: () => {},
      useRef: value => ({ current: value }),
      useCallback: fn => fn,
    }
  }
  throw new Error(`unexpected require(${name})`)
})

console.log('client exports:', Object.keys(client).join(', '), '| inject:', JSON.stringify(client.inject))
if (client.name !== 'dzrobot-ssh') throw new Error('wrong plugin name')

// ── real Host half on loopback ────────────────────────────────────────────────
const { apply: applyHost } = await import('../index.js')
const routes = []
const ctxHost = {
  logger: { info: () => {}, warn: () => {}, error: () => {} },
  get: () => undefined,
  effect: callback => { const dispose = callback(); return typeof dispose === 'function' ? dispose : () => {} },
  inject: (deps, callback) => callback({
    webServer: {
      register: route => { routes.push(route); return () => {} },
      registerUpgrade: route => { return () => {} },
    },
    effect: callback2 => { const dispose = callback2(); return typeof dispose === 'function' ? dispose : () => {} },
  }),
}
applyHost(ctxHost, {})

const server = http.createServer((req, res) => {
  const route = routes.find(candidate => req.url.startsWith(candidate.path))
  if (route === undefined) { res.writeHead(404).end('{}'); return }
  void route.handler(req, res)
})
await new Promise(resolve => server.listen(0, '127.0.0.1', resolve))
const origin = `http://127.0.0.1:${server.address().port}`

const realFetch = globalThis.fetch
globalThis.fetch = (url, options) => realFetch(`${origin}${url}`, options)

// ── stubbed sidebar + terminal services ───────────────────────────────────────
const tabs = []
const writes = []
const renames = []
let viewSnapshot = { phase: 'idle', writable: false, title: 'Terminal', info: undefined }

// The real failure mode: while this plugin's main panel is active the right
// sidebar has no on-screen Session, so commandTarget answers undefined. The stub
// starts in that state and only resolves a target after layout.selectPanel(null)
// — exactly the switch the flow must perform.
let activePanel = 'dzrobot-car'
const panelHistory = []

const sidebarRight = {
  openTabs: { getSnapshot: () => tabs },
  commandTarget: () => (activePanel === null
    ? { sessionId: 'session-1', paneId: 'pane-1', host: 'dock', tabId: undefined }
    : undefined),
  openTabFromTarget: (kind, target) => {
    // Void, exactly like the shipped implementation: the tab record appears later.
    setTimeout(() => {
      tabs.push({ kind, tabId: 'tab-7', id: 'tab-7', contentId: 'content-7', sessionId: target.sessionId })
    }, 40)
  },
  openTab: kind => {
    // Mirrors placeTab: the record lands asynchronously; openToolTab does not
    // wait for it, but recording it proves the right type was opened.
    openedKinds.push(kind)
    setTimeout(() => {
      tabs.push({ kind, tabId: `tab-${kind}`, id: `tab-${kind}`, contentId: `sidebar://${kind}`, sessionId: 'session-1' })
    }, 40)
  },
}
const openedKinds = []

// The registry the right column offers to plugins: record every tab type this
// plugin contributes, so the test can assert both ids and kinds.
const tabTypes = []
const sidebarRightTabsService = {
  register: definition => { tabTypes.push(definition); return () => {} },
}

const layoutService = {
  selectPanel: id => { activePanel = id; panelHistory.push(id) },
  panelInfo: { getSnapshot: () => ({ activePanel: activePanel, activePanelId: activePanel }) },
}
const uiSessionService = {
  adapter: { current: { getSnapshot: () => ({ key: 'session-1' }) } },
}

const webTerminals = {
  view: (sessionId, key, contentId) => {
    console.log(`  view(${sessionId}, ${key}, ${contentId})`)
    setTimeout(() => { viewSnapshot = { ...viewSnapshot, phase: 'connected', writable: true } }, 180)
    return {
      id: 'term-1',
      state: { getSnapshot: () => viewSnapshot },
      write: data => writes.push(data),
      rename: title => renames.push(title),
    }
  },
}

const diagPayloads = []
// One slot name may receive several registrations (the right-column pane slot
// gets one per tab type), so each slot collects a list of {definition, component}.
const registered = {}
const slotsService = {
  inject: (slot, callback) => {
    registered[slot] = [...(registered[slot] ?? []), callback()]
  },
  register: (definition, component) => ({ definition, component }),
}
const localeService = {
  register: () => () => {},
  bind: () => (key, vars) => (vars === undefined ? key : `${key}:${JSON.stringify(vars)}`),
  getLocale: () => ({ active: 'zh' }),
}
const ctxClient = {
  get: name => ({
    sidebarRight, webTerminals, slots: slotsService, locale: localeService,
    layout: layoutService, uiSession: uiSessionService,
    sidebarRightTabs: sidebarRightTabsService,
  }[name]),
  effect: callback => { const dispose = callback(); return typeof dispose === 'function' ? dispose : () => {} },
  slots: slotsService,
  locale: localeService,
}

// Capture the diagnostic reports the flow sends, then fail the test if the flow
// claims success without them — the reports are the only witness to each step.
const fetchWithDiag = globalThis.fetch
globalThis.fetch = (url, options) => {
  if (String(url).includes('/diag')) {
    try { diagPayloads.push(JSON.parse(options.body)) } catch { /* ignore */ }
  }
  return fetchWithDiag(url, options)
}

// ── run ───────────────────────────────────────────────────────────────────────
client.apply(ctxClient)
console.log('registered slots:', Object.keys(registered).join(', '))

// A slot whose registration succeeded but whose first render throws shows
// nothing at all — indistinguishable from a plugin that never loaded. The shell
// calls exactly this function, so calling it here is the real render check.
// One slot name can carry several registrations (the right-column pane slot,
// one per tab type), so every entry is rendered.
const renderFailures = []
for (const [slot, entries] of Object.entries(registered)) {
  entries.forEach((entry, index) => {
    try {
      const tree = entry.component({ wide: true })
      console.log(`  render(${slot}[${index}]) -> ${tree === undefined ? 'undefined' : 'element'}`)
      if (tree === undefined) renderFailures.push(`render(${slot}[${index}]) returned nothing`)
    } catch (error) {
      renderFailures.push(`render(${slot}[${index}]) threw: ${error?.message ?? error}`)
    }
  })
}

// The panel's three views are the new surface and cannot be clicked from here, so
// each one is rendered directly with a realistic settings object. A view that
// throws on first paint would otherwise look exactly like a view that never
// mounted.
const settingsStub = {
  host: 'duzhong.local', user: 'duzhong', port: 22, sshAlias: '',
  vncHost: '', vncPort: 5901, vncMode: 'direct', vncPassword: '',
  filesRoot: '/home/duzhong', filesShortcuts: [{ label: '主目录', path: '/home/duzhong' }],
}
const storeStub = { get: () => ({ phase: 'idle', probe: { state: 'ready' } }), subscribe: () => () => {}, set: () => {} }
const { makeVncView, makeFilesView, makeCarPanel, makeVncTabShell, makeFilesTabShell } = client.__test
const tKey = key => key
for (const [label, component] of [
  ['vnc', makeVncView(ctxClient, tKey)({ settings: settingsStub, origin: 'http://127.0.0.1:19387' })],
  // Also without an origin: the panel gets that value from an API response, and a
  // missing one must render the "no address" notice instead of throwing.
  ['vnc-without-origin', makeVncView(ctxClient, tKey)({ settings: settingsStub, origin: undefined })],
  ['files', makeFilesView(ctxClient, tKey)({ settings: settingsStub })],
  ['panel', makeCarPanel(ctxClient, storeStub, tKey)({})],
  // The right-column tab bodies: first paint is the loading note (their status
  // fetch resolves later), so this proves the shell itself does not throw.
  ['tab:vnc', makeVncTabShell(ctxClient, tKey)({})],
  ['tab:files', makeFilesTabShell(ctxClient, tKey)({})],
]) {
  try {
    const tree = component
    console.log(`  render(view:${label}) -> ${tree === undefined ? 'undefined' : 'element'}`)
    if (tree === undefined) renderFailures.push(`render(view:${label}) returned nothing`)
  } catch (error) {
    renderFailures.push(`render(view:${label}) threw: ${error?.message ?? error}`)
  }
}

const phases = []
const result = await client.__test.connect(ctxClient, { closest: () => null }, phase => phases.push(phase))
console.log('\nconnect result:', JSON.stringify(result))
console.log('phases:', phases.join(' -> '))
console.log('writes:', JSON.stringify(writes))
console.log('renames:', JSON.stringify(renames))
console.log('diag steps:', diagPayloads.map(entry => entry.step).join(', '))

const failures = [...renderFailures]
if (result.ok !== true) failures.push(`connect did not succeed: ${JSON.stringify(result)}`)
if (writes.length !== 1) failures.push(`expected exactly one write, got ${writes.length}`)
if (!String(writes[0] ?? '').startsWith('ssh ')) failures.push(`the written command is not ssh: ${writes[0]}`)
if (!String(writes[0] ?? '').endsWith('\r')) failures.push('the command was not submitted with Enter')
if (renames.length !== 1) failures.push(`expected one rename, got ${renames.length}`)
const steps = diagPayloads.map(entry => entry.step)
for (const required of ['connect.start', 'connect.probe', 'connect.command', 'connect.target', 'connect.tab', 'connect.view', 'connect.writable', 'connect.wrote']) {
  if (!steps.includes(required)) failures.push(`missing diagnostic step ${required}`)
}
// The regression this suite exists for: starting from "plugin panel active"
// (commandTarget undefined), the flow must switch back to the Conversation and
// resolve there — not give up, and not just reveal an empty right column.
if (!panelHistory.includes(null)) {
  failures.push(`flow never switched to the Conversation (selectPanel history: ${JSON.stringify(panelHistory)})`)
}
if (!steps.includes('connect.reveal')) failures.push('missing diagnostic step connect.reveal')
const firstTarget = diagPayloads.find(entry => entry.step === 'connect.target')
if (firstTarget?.data?.found !== false) {
  failures.push('stub expected to start unreachable (found:false), the switch path was not exercised')
}

// ── the tool-list → right-column-tab path (the new architecture) ──────────────
// Simulate the user back on the plugin panel, then open the files tool the way
// its list row does: switch to the Conversation, open the tab type.
activePanel = 'dzrobot-car'
panelHistory.length = 0
const toolResult = await client.__test.openToolTab(ctxClient, 'dzr-files')
console.log('openToolTab result:', JSON.stringify(toolResult), '| opened:', JSON.stringify(openedKinds))

const vncType = tabTypes.find(definition => definition.kind === 'dzr-vnc')
const filesType = tabTypes.find(definition => definition.kind === 'dzr-files')
const paneRegistrations = registered['sidebar.right.pane.tab'] ?? []

if (toolResult.ok !== true) failures.push(`openToolTab did not succeed: ${JSON.stringify(toolResult)}`)
if (!openedKinds.includes('dzr-files')) failures.push(`openTab never called with the tool kind (opened: ${JSON.stringify(openedKinds)})`)
if (!panelHistory.includes(null)) failures.push('tool path did not switch to the Conversation first')
if (vncType === undefined || vncType.id !== 'dsh-dzrobot-ssh/vnc') {
  failures.push(`vnc tab type missing or wrong id: ${JSON.stringify(vncType)}`)
} else if (vncType.multiple !== false || vncType.keepMounted !== true) {
  failures.push(`vnc tab type flags wrong (multiple:${vncType.multiple} keepMounted:${vncType.keepMounted})`)
}
if (filesType === undefined || filesType.id !== 'dsh-dzrobot-ssh/files' || filesType.multiple !== false) {
  failures.push(`files tab type missing or wrong: ${JSON.stringify(filesType)}`)
}
// TabSlot dispatches through entryKey: definition.id — the slot key must be the
// type id, or the body renders the "type no longer available" fallback.
for (const id of ['dsh-dzrobot-ssh/vnc', 'dsh-dzrobot-ssh/files']) {
  if (!paneRegistrations.some(registration => registration.definition.key === id)) {
    failures.push(`no sidebar.right.pane.tab body registered under key ${id}`)
  }
}
if (client.__test.TOOLS.length !== 3) failures.push(`tool list should have 3 rows, has ${client.__test.TOOLS.length}`)

// ── the generic VNC connector panel ────────────────────────────────────────────
const connector = client.__test
if (connector.detectRuntime() !== 'browser') {
  failures.push(`headless node should report runtime 'browser', got ${connector.detectRuntime()}`)
}
const defaults = connector.normalizeConnectorConfig({})
if (defaults.scheme !== 'ws' || defaults.port !== '6080' || defaults.path !== 'websockify' || defaults.resolution !== 'auto' || defaults.scale !== true) {
  failures.push(`normalizeConnectorConfig defaults wrong: ${JSON.stringify(defaults)}`)
}
const sanitized = connector.normalizeConnectorConfig({ scheme: 'wss', host: 'x', port: '', path: '/custom', password: 'p', resolution: '1920x1080', scale: false })
if (sanitized.scheme !== 'wss' || sanitized.port !== '6080' || sanitized.path !== '/custom' || sanitized.password !== 'p' || sanitized.scale !== false) {
  failures.push(`normalizeConnectorConfig sanitize wrong: ${JSON.stringify(sanitized)}`)
}
const garbage = connector.normalizeConnectorConfig({ scheme: 'ftp', resolution: '9999x9999', port: 12 })
if (garbage.scheme !== 'ws' || garbage.resolution !== 'auto') {
  failures.push(`normalizeConnectorConfig must coerce junk, got ${JSON.stringify(garbage)}`)
}
// The connector replaced the car VNC tab, so it must NOT register its own
// sidebar icon or main panel anymore — the one VNC surface is the tab body.
const connectorPanes = registered.main ?? []
if (connectorPanes.some(entry => entry.definition.key === connector.CONNECTOR_ID)) {
  failures.push(`the connector must not register its own main panel anymore (${connector.CONNECTOR_ID}) — it replaced the car VNC tab`)
}
const connectorIcons = registered['sidebar.panellist'] ?? []
if (connectorIcons.some(entry => entry.definition.id === connector.CONNECTOR_ID)) {
  failures.push(`the connector must not register its own sidebar icon anymore (${connector.CONNECTOR_ID}) — it replaced the car VNC tab`)
}
// The VNC tab body must now host the connector panel, not the legacy view. A
// static render only shows the loading note (the status fetch resolves later),
// so the wiring is asserted at source level: the tab shell mounts the connector
// and no longer mounts the legacy view.
const clientSource = fs.readFileSync(new URL('../client.js', import.meta.url), 'utf8')
const tabShellSource = clientSource.slice(clientSource.indexOf('function makeVncTabShell'), clientSource.indexOf('function makeFilesTabShell'))
if (tabShellSource.includes('makeVncView(')) {
  failures.push('the VNC tab body still mounts the legacy car VNC view instead of the connector')
}
if (!tabShellSource.includes('makeVncConnectorPanel')) {
  failures.push('the VNC tab body does not mount the connector panel')
}
try {
  const tree = connector.makeVncConnectorPanel(ctxClient, tKey)({})
  console.log(`  render(connector panel) -> ${tree === undefined ? 'undefined' : 'element'}`)
  if (tree === undefined) failures.push('render(connector panel) returned nothing')
} catch (error) {
  failures.push(`render(connector panel) threw: ${error?.message ?? error}`)
}
if (!Array.isArray(connector.VNC_RESOLUTIONS) || !connector.VNC_RESOLUTIONS.some(item => item.id === 'auto') || !connector.VNC_RESOLUTIONS.some(item => item.id === '2560x1440')) {
  failures.push(`VNC_RESOLUTIONS missing entries: ${JSON.stringify(connector.VNC_RESOLUTIONS)}`)
}

// ── the car bridge: the one-click way back to a target that always works ───────
// The failure this guards was reported as a bare `1006` with the form pointed at
// a websockify that was never installed, and no way back but retyping every
// field. The prefill and the fallback button must therefore be the *same*
// computation, or they can drift and the escape hatch lands somewhere else.
const bridgeHttp = connector.bridgeConnectorConfig('http://127.0.0.1:19387', connector.normalizeConnectorConfig({}), 'carvnc')
if (bridgeHttp.scheme !== 'ws' || bridgeHttp.host !== '127.0.0.1' || bridgeHttp.port !== '19387'
  || bridgeHttp.path !== 'api/dzrobot-ssh/vnc' || bridgeHttp.password !== 'carvnc') {
  failures.push(`bridgeConnectorConfig (http) wrong: ${JSON.stringify(bridgeHttp)}`)
}
const bridgeHttps = connector.bridgeConnectorConfig('https://dsh.example', connector.normalizeConnectorConfig({}), 'carvnc')
if (bridgeHttps.scheme !== 'wss' || bridgeHttps.port !== '443') {
  failures.push(`bridgeConnectorConfig must map https -> wss on 443: ${JSON.stringify(bridgeHttps)}`)
}
const bridgeKeepsPassword = connector.bridgeConnectorConfig('http://127.0.0.1:9', connector.normalizeConnectorConfig({ password: 'typed' }), 'carvnc')
if (bridgeKeepsPassword.password !== 'typed') {
  failures.push('bridgeConnectorConfig must not overwrite a password the user typed')
}
let bridgeThrew = false
try { connector.bridgeConnectorConfig('not a url', connector.normalizeConnectorConfig({}), '') } catch { bridgeThrew = true }
if (!bridgeThrew) {
  failures.push('bridgeConnectorConfig must throw on an unparseable origin so the caller can show a notice')
}
// A static render cannot reach the failed phase (the harness stubs React state),
// so this half is asserted at source level, the same way the tab wiring is.
const connectorPanelSource = clientSource.slice(
  clientSource.indexOf('function makeVncConnectorPanel'), clientSource.indexOf('exports.__test'))
if (!connectorPanelSource.includes('useBridge')) {
  failures.push('the connector must offer the one-click car-bridge fallback')
}
if (!connectorPanelSource.includes('connHintNoListener') || !connectorPanelSource.includes('connHintBridgeFailed')) {
  failures.push('the connector must explain a 1006 in terms of the target it was pointed at')
}
// Both copies of every new string: a key added to one language only shows up as
// a raw identifier in the other, which no other test would catch.
for (const key of ['connCarBridge', 'connCarBridgeHint', 'connCarBridgeDone', 'connHintNoListener', 'connHintBridgeFailed']) {
  for (const lang of ['zh', 'en']) {
    const value = connector.DICT?.[lang]?.[key]
    if (typeof value !== 'string' || value === '') failures.push(`DICT.${lang}.${key} is missing`)
  }
}

server.close()
if (failures.length > 0) {
  console.log('\nFAILURES:')
  for (const failure of failures) console.log(' -', failure)
  process.exitCode = 1
} else {
  console.log('\nCLIENT TEST PASSED')
}
