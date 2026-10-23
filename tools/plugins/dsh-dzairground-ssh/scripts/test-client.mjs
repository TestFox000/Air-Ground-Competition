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
// car address in ~/.dsh/dzairground-ssh/settings.json.
process.env.DSH_HOME = fs.mkdtempSync(path.join(os.tmpdir(), 'dzairground-ssh-client-test-'))

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
      // A real tree rather than an opaque token. The float's contract is
      // structural now — two cells each carrying 名称/数值/曲线, a close button,
      // an opacity slider, a drag-guarded header — and `() => ({})` would make
      // all of that unassertable. Children arrive either spread or as one
      // array, so the walkers below flatten.
      createElement: (type, props, ...children) => ({ type, props: props ?? {}, children }),
      // React calls a function initial value once to produce the first state;
      // the float relies on that for its saved position, so the stub does too.
      useState: value => [typeof value === 'function' ? value() : value, () => {}],
      useEffect: () => {},
      useRef: value => ({ current: value }),
      useCallback: fn => fn,
    }
  }
  throw new Error(`unexpected require(${name})`)
})

console.log('client exports:', Object.keys(client).join(', '), '| inject:', JSON.stringify(client.inject))
if (client.name !== 'dzairground-ssh') throw new Error('wrong plugin name')

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
let activePanel = 'dzairground-ops'
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
//
// The car half also gets a local stand-in, the same idea as test-host's TCP VNC
// stand-in: only /car/probe and /car/status are answered, so connect() runs its
// whole flow (probe → command → reveal → tab → view → write) without a robot.
// /drone/* is deliberately NOT stubbed — dzsmart.local is honestly unreachable
// here, and the S7 drone assertions depend on that.
const CAR_STUB = {
  probe: { state: 'ready', detail: 'stub', latencyMs: 1, command: 'ssh -T duzhong@stub.local' },
  status: {
    settings: { host: 'stub.local', user: 'duzhong', sshAlias: '' },
    command: 'ssh -T duzhong@stub.local',
  },
}
const carStub = (url) => {
  const path = new URL(String(url), 'http://localhost').pathname  // client fetches a relative path; wrapper #1 makes it absolute
  if (path === '/api/dzairground-ssh/car/probe') return CAR_STUB.probe
  if (path === '/api/dzairground-ssh/car/status') return CAR_STUB.status
  return undefined
}
const fetchWithDiag = globalThis.fetch
globalThis.fetch = async (url, options) => {
  if (String(url).includes('/diag')) {
    try { diagPayloads.push(JSON.parse(options.body)) } catch { /* ignore */ }
  }
  const stubbed = carStub(url)
  if (stubbed !== undefined) {
    return { ok: true, status: 200, async text() { return JSON.stringify(stubbed) } }
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
const { makeVncView, makeFilesView, makeCarPanel, makeVncTabShell, makeFilesTabShell, makeFloatWindow } = client.__test
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
  ['tab:vnc-car', makeVncTabShell(ctxClient, tKey, 'car')({})],
  ['tab:vnc-drone', makeVncTabShell(ctxClient, tKey, 'drone')({})],
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

// The status float, in both states. Closed must be a deliberate `null` (an
// intentional nothing-to-show, distinct from a render that returned
// `undefined`), and open must paint without a Session, an origin or a reading —
// those all arrive later, and a first paint that throws would show an empty
// corner with no error anywhere.
const floatStoreStub = { get: () => ({ floatOpen: true }), subscribe: () => () => {}, set: () => {} }
for (const [label, component] of [
  ['float:closed', makeFloatWindow(ctxClient, storeStub, tKey)],
  ['float:open', makeFloatWindow(ctxClient, floatStoreStub, tKey)],
]) {
  try {
    const tree = component({})
    const closed = label.endsWith('closed')
    const expected = closed ? tree === null : tree !== undefined && tree !== null
    console.log(`  render(view:${label}) -> ${tree === undefined || tree === null ? String(tree) : 'element'}`)
    if (tree === undefined) renderFailures.push(`render(view:${label}) returned undefined (only null is an intentional empty)`)
    else if (!expected) renderFailures.push(`render(view:${label}) should have ${closed ? 'null (closed)' : 'an element (open)'}`)
  } catch (error) {
    renderFailures.push(`render(view:${label}) threw: ${error?.message ?? error}`)
  }
}

// The open float in full detail: two parallel cells (电量, CPU), each the pair
// of blue boxes — 名称 + 数值 over an axes-bearing plot; the uptime on the
// header line instead of in the body; a close button wired to the store; an
// opacity slider — and a header that refuses to start a drag when the press
// lands on a button (pointer capture would retarget that click to the header
// itself: the reported "关不掉").
const floatWalk = (node, visit) => {
  if (Array.isArray(node)) { node.forEach(item => floatWalk(item, visit)); return }
  if (node === null || node === undefined || typeof node !== 'object') return
  visit(node)
  floatWalk(node.children, visit)
}
const floatFind = (tree, predicate) => {
  const found = []
  floatWalk(tree, node => { if (predicate(node)) found.push(node) })
  return found
}
const floatSets = []
const floatOpenStore = {
  get: () => ({ floatOpen: true, floatOpacity: 60 }),
  subscribe: () => () => {},
  set: patch => { floatSets.push(patch) },
}
try {
  const tree = makeFloatWindow(ctxClient, floatOpenStore, tKey)({})

  // ── the header line: 名称 | 开机时间：--:-- ────────────────────────────────
  const titleGroups = floatFind(tree, node => node.props?.className === 'dzag_ftitle')
  if (titleGroups.length !== 1) renderFailures.push(`expected one title group, got ${titleGroups.length}`)
  else {
    const parts = (titleGroups[0].children ?? []).flat()
    const [name, sep, up] = parts
    if (name?.type !== 'b' || name.children?.[0] !== 'floatTitle') {
      renderFailures.push(`title should lead with the name, got ${JSON.stringify(name)}`)
    }
    if (sep?.type !== 'i' || sep.children?.[0] !== '|') {
      renderFailures.push(`title separator should be '|', got ${JSON.stringify(sep)}`)
    }
    const upText = Array.isArray(up?.children) ? up.children.join('') : String(up?.children ?? '')
    if (up?.type !== 'span' || upText !== 'floatUp：--:--') {
      renderFailures.push(`header uptime should read floatUp：--:-- before a reading, got ${JSON.stringify(upText)}`)
    }
    if (up?.props?.className !== 'dzag_fup') renderFailures.push('header uptime missing the dzag_fup class')
  }

  // ── the body: capability-driven cells (决议③ 一张卡多格) ─────────────────
  // Car row: battery + cpu. Drone row: temp + cpu + the FC battery, whose
  // value box carries the three-state word before any reading exists (S3).
  const cells = floatFind(tree, node => node.props?.className === 'dzag_fcell')
  if (cells.length !== 5) renderFailures.push(`the float should render 5 capability cells (car 2 + drone 3), got ${cells.length}`)
  const kinds = cells.map(cell => cell.props['data-kind'])
  if (kinds.join(',') !== 'battery,cpu,temp,droneCpu,droneBattery') {
    renderFailures.push(`cells should be battery,cpu,temp,droneCpu,droneBattery, got ${kinds.join(',')}`)
  }
  // One node row per node (决议③): the cells are never ambiguous about which
  // machine they describe.
  const nodeRows = floatFind(tree, node => node.props?.className === 'dzag_fnode')
  if (nodeRows.length !== 2) renderFailures.push(`the float should render 2 node rows, got ${nodeRows.length}`)
  for (const cell of cells) {
    const boxes = (cell.children ?? []).flat()
    if (boxes.length !== 2) { renderFailures.push(`cell ${cell.props['data-kind']} should be 2 boxes, got ${boxes.length}`); continue }
    const [valueBox, plotBox] = boxes
    if (valueBox?.props?.className !== 'dzag_fbox') renderFailures.push(`cell ${cell.props['data-kind']}: first box lost the dzag_fbox class`)
    const valueParts = (valueBox?.children ?? []).flat()
    if (valueParts[0]?.type !== 'span' || valueParts[1]?.type !== 'b') {
      renderFailures.push(`cell ${cell.props['data-kind']}: 名称+数值 box should be span then b, got ${JSON.stringify(valueParts.map(part => part?.type))}`)
    }
    if (!String(plotBox?.props?.className ?? '').includes('dzag_fplot')) {
      renderFailures.push(`cell ${cell.props['data-kind']}: second box should be the plot box`)
    }
    const charts = floatFind(plotBox, node => node.type === 'svg' && node.props?.className === 'dzag_fchart')
    if (charts.length !== 1) renderFailures.push(`cell ${cell.props['data-kind']} should carry exactly one plot, got ${charts.length}`)
    else if (charts[0].props.viewBox !== '0 0 144 78') renderFailures.push(`plot viewBox wrong: ${charts[0].props.viewBox}`)
    else {
      // First paint has no reading, so the plot must be its empty state: both
      // axes drawn plus the dashed baseline — no curve, no invented ticks.
      const kids = (charts[0].children ?? []).flat()
      const classes = kids.map(kid => String(kid?.props?.className ?? ''))
      if (classes.filter(name => name === 'dzag_faxis').length !== 2) {
        renderFailures.push(`cell ${cell.props['data-kind']}: empty plot should draw both axes, got ${JSON.stringify(classes)}`)
      }
      if (classes.includes('dzag_fno') !== true) renderFailures.push(`cell ${cell.props['data-kind']}: empty plot should draw the dashed baseline`)
      if (kids.some(kid => kid?.type === 'polyline')) renderFailures.push(`cell ${cell.props['data-kind']}: drew a curve with no reading`)
      if (kids.some(kid => kid?.type === 'circle')) renderFailures.push(`cell ${cell.props['data-kind']}: drew the red dot with no reading`)
    }
  }

  // The close path: a click on × must flip the store the overlay card reads.
  const closeButtons = floatFind(tree, node => node.type === 'button' && node.props?.title === 'floatClose')
  if (closeButtons.length !== 1) renderFailures.push(`expected exactly one close button, got ${closeButtons.length}`)
  else {
    closeButtons[0].props.onClick()
    if (!floatSets.some(patch => patch.floatOpen === false)) {
      renderFailures.push('clicking × did not close the float (store never saw floatOpen:false)')
    }
  }

  // The drag guard, both directions: a button press must not be captured, a
  // press on the title bar must start a drag as before.
  const heads = floatFind(tree, node => node.props?.className === 'dzag_fhead')
  if (heads.length !== 1) renderFailures.push(`expected one draggable header, got ${heads.length}`)
  else {
    let captures = 0
    const press = target => ({ target, clientX: 8, clientY: 8, pointerId: 1, currentTarget: { setPointerCapture: () => { captures += 1 } } })
    heads[0].props.onPointerDown(press({ closest: selector => (selector === 'button' ? {} : null) }))
    if (captures !== 0) renderFailures.push('pressing a header button started a drag — its click would be retargeted (关不掉 regression)')
    heads[0].props.onPointerDown(press({ closest: () => null }))
    if (captures !== 1) renderFailures.push('a press on the title bar should start a drag')
  }

  // Opacity: 60% in the store must reach the card as --dzag-op, and the slider
  // must be a clamped 30..100 range whose changes reach the store.
  const style = tree.props?.style ?? {}
  if (style['--dzag-op'] !== '0.6') renderFailures.push(`opacity 60 should render --dzag-op:"0.6", got ${JSON.stringify(style['--dzag-op'])}`)
  const sliders = floatFind(tree, node => node.type === 'input' && node.props?.type === 'range')
  if (sliders.length !== 1) renderFailures.push(`expected one opacity slider, got ${sliders.length}`)
  else {
    const slider = sliders[0].props
    if (slider.min !== 30 || slider.max !== 100 || slider.value !== 60) {
      renderFailures.push(`opacity slider wrong: min=${slider.min} max=${slider.max} value=${slider.value}`)
    }
    slider.onChange({ target: { value: '55' } })
    if (!floatSets.some(patch => patch.floatOpacity === 55)) {
      renderFailures.push('the opacity slider change never reached the store')
    }
  }

  // The refresh button: one manual re-poll without closing the card.
  if (floatFind(tree, node => node.type === 'button' && node.props?.title === 'floatRefresh').length !== 1) {
    renderFailures.push('expected exactly one refresh button in the footer')
  }
} catch (error) {
  renderFailures.push(`render(view:float:detail) threw: ${error?.message ?? error}`)
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
if (result.warn !== undefined) failures.push(`a ready probe must not carry a warn: ${JSON.stringify(result.warn)}`)
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

// ── S7: the drone terminal channel resolves the drone node ────────────────────
// dzsmart.local is honestly unreachable from the lab. Since TOOLS-PLUGIN-TERMGATE
// (2026-10-23) a failed probe no longer blocks the terminal: the click must STILL
// open a tab and type the DRONE's own ssh line, with the probe verdict attached
// as `warn`. What this proves is WHICH node was probed and whose command was
// typed — the channel, not some hardcoded car target.
{
  const fetched = []
  const prevFetch = globalThis.fetch
  globalThis.fetch = (url, options) => { fetched.push(String(url)); return prevFetch(url, options) }
  const dronePhases = []
  const writesBefore = writes.length
  const droneResult = await client.__test.connect(ctxClient, { closest: () => null }, phase => dronePhases.push(phase), 'drone')
  globalThis.fetch = prevFetch
  if (!fetched.some(url => url.includes('/api/dzairground-ssh/drone/probe'))) {
    failures.push(`S7: drone connect did not probe /drone/probe (fetched ${JSON.stringify(fetched)})`)
  }
  if (fetched.some(url => url.endsWith('/api/dzairground-ssh/probe'))) {
    failures.push('S7: the drone channel fell back to the unscoped (car) probe')
  }
  const start = diagPayloads.filter(entry => entry.step === 'connect.start').pop()
  if (start?.data?.role !== 'drone') failures.push(`S7: drone connect.start missing role tag: ${JSON.stringify(start?.data)}`)
  if (droneResult.ok !== true) failures.push(`S7: drone connect must open the terminal despite the failed probe: ${JSON.stringify(droneResult)}`)
  if (droneResult.warn === undefined) failures.push('S7: an unreachable probe must surface as warn, not vanish')
  const droneWrites = writes.slice(writesBefore)
  if (droneWrites.length !== 1 || !String(droneWrites[0]).includes('dzsmart.local')) {
    failures.push(`S7: the typed command is not the drone's ssh line: ${JSON.stringify(droneWrites)}`)
  }
  if (!dronePhases.includes('probing')) failures.push(`S7: drone flow never reached probing (phases ${JSON.stringify(dronePhases)})`)
}

// ── the tool-list → right-column-tab path (the new architecture) ──────────────
// Simulate the user back on the plugin panel, then open the files tool the way
// its list row does: switch to the Conversation, open the tab type.
activePanel = 'dzairground-ops'
panelHistory.length = 0
const toolResult = await client.__test.openToolTab(ctxClient, 'dzag-files')
console.log('openToolTab result:', JSON.stringify(toolResult), '| opened:', JSON.stringify(openedKinds))

const vncCarType = tabTypes.find(definition => definition.kind === 'dzag-vnc-car')
const vncDroneType = tabTypes.find(definition => definition.kind === 'dzag-vnc-drone')
const filesType = tabTypes.find(definition => definition.kind === 'dzag-files')
const paneRegistrations = registered['sidebar.right.pane.tab'] ?? []

if (toolResult.ok !== true) failures.push(`openToolTab did not succeed: ${JSON.stringify(toolResult)}`)
if (!openedKinds.includes('dzag-files')) failures.push(`openTab never called with the tool kind (opened: ${JSON.stringify(openedKinds)})`)
if (!panelHistory.includes(null)) failures.push('tool path did not switch to the Conversation first')
for (const [type, expectedId] of [[vncCarType, 'dsh-dzairground-ssh/vnc-car'], [vncDroneType, 'dsh-dzairground-ssh/vnc-drone']]) {
  if (type === undefined || type.id !== expectedId) {
    failures.push(`vnc tab type missing or wrong id (want ${expectedId}): ${JSON.stringify(type)}`)
  } else if (type.multiple !== false || type.keepMounted !== true) {
    failures.push(`vnc tab type flags wrong (multiple:${type.multiple} keepMounted:${type.keepMounted})`)
  }
}
if (filesType === undefined || filesType.id !== 'dsh-dzairground-ssh/files' || filesType.multiple !== false) {
  failures.push(`files tab type missing or wrong: ${JSON.stringify(filesType)}`)
}
// TabSlot dispatches through entryKey: definition.id — the slot key must be the
// type id, or the body renders the "type no longer available" fallback.
for (const id of ['dsh-dzairground-ssh/vnc-car', 'dsh-dzairground-ssh/vnc-drone', 'dsh-dzairground-ssh/files']) {
  if (!paneRegistrations.some(registration => registration.definition.key === id)) {
    failures.push(`no sidebar.right.pane.tab body registered under key ${id}`)
  }
}
// Seven rows since S7: terminal and VNC exist once per node (car/drone), the
// files tool stays shared, the float opens as an overlay card, and the sim
// bench is mode:"external" — client.js TOOLS[] is the single source of truth.
if (client.__test.TOOLS.length !== 7) failures.push(`tool list should have 7 rows, has ${client.__test.TOOLS.length}`)
const floatRow = client.__test.TOOLS.find(tool => tool.id === 'dzag-float')
if (floatRow === undefined || floatRow.mode !== 'float') {
  failures.push(`the float tool row is missing or not mode:"float": ${JSON.stringify(floatRow)}`)
}
// The float lives on the frame-wide overlay seat; if that registration ever
// stops happening the card has nowhere to mount and the tool row silently
// toggles a flag nothing renders.
const overlayRegistrations = registered['shell.overlay'] ?? []
if (!overlayRegistrations.some(entry => entry.definition.id === 'dzairground-float')) {
  failures.push(`no shell.overlay registration under id dzairground-float (got ${JSON.stringify(overlayRegistrations.map(entry => entry.definition.id))})`)
}

// ── S7: the four channels are distinct and dial distinct targets ──────────────
{
  const ids = client.__test.TOOLS.map(tool => tool.id)
  const channelIds = ['dzag-terminal-car', 'dzag-terminal-drone', 'dzag-vnc-car', 'dzag-vnc-drone']
  for (const id of channelIds) {
    if (!ids.includes(id)) failures.push(`S7: channel id ${id} missing from TOOLS`)
  }
  if (new Set(channelIds).size !== 4) failures.push('S7: channel ids are not pairwise distinct')
  const rowOf = id => client.__test.TOOLS.find(tool => tool.id === id)
  const kinds = channelIds.map(id => rowOf(id)?.kind)
  if (kinds[0] !== 'terminal' || kinds[1] !== 'terminal') {
    failures.push(`S7: both terminal channels must keep the stock kind (shell-owned PTY), got ${JSON.stringify(kinds.slice(0, 2))}`)
  }
  if (kinds[2] !== 'dzag-vnc-car' || kinds[3] !== 'dzag-vnc-drone') {
    failures.push(`S7: vnc channel kinds wrong: ${JSON.stringify(kinds.slice(2))}`)
  }
  const roles = channelIds.map(id => rowOf(id)?.role)
  if (roles.join(',') !== 'car,drone,car,drone') failures.push(`S7: channel roles wrong: ${JSON.stringify(roles)}`)
  const keys = client.__test.connectorStoreKeys
  if (!keys || keys.car === keys.drone) failures.push('S7: connector storage keys must differ per channel')
  const carCfg = client.__test.bridgeConnectorConfig('http://127.0.0.1:1', {}, '', 'car')
  const droneCfg = client.__test.bridgeConnectorConfig('http://127.0.0.1:1', {}, '', 'drone')
  if (carCfg.path !== 'api/dzairground-ssh/car/vnc') failures.push(`S7: car bridge path wrong: ${carCfg.path}`)
  if (droneCfg.path !== 'api/dzairground-ssh/drone/vnc') failures.push(`S7: drone bridge path wrong: ${droneCfg.path}`)
}

// ── B-1 (决议⑦): the panel's main view folds the sim bench away ───────────────
// Render the panel for real and walk it: the coordination entry is a main-view
// row, the sim bench + stress bench live inside a <details> that must NOT carry
// an `open` prop (default collapsed), and no sim row sits outside the drawer.
{
  const walk = (node, visit) => {
    if (Array.isArray(node)) { node.forEach(item => walk(item, visit)); return }
    if (node === null || node === undefined || typeof node !== 'object') return
    visit(node)
    walk(node.children, visit)
  }
  try {
    const panelStore = { get: () => ({ phase: 'idle', probe: undefined }), subscribe: () => () => {}, set: () => {} }
    const panelTree = client.__test.makeCarPanel(ctxClient, panelStore, tKey)({})
    const detailsNodes = []
    const coordRows = []
    walk(panelTree, node => {
      if (node.type === 'details' && node.props?.className === 'dzag_advanced') detailsNodes.push(node)
      if (node.type === 'div' && node.props?.className === 'dzag_toolrow' && node.props?.key === 'dzag-coord') coordRows.push(node)
    })
    if (detailsNodes.length !== 1) {
      failures.push(`B-1: expected exactly one advanced <details>, got ${detailsNodes.length}`)
    } else if (detailsNodes[0].props?.open !== undefined) {
      failures.push('B-1: the advanced drawer must not carry an open prop (default collapsed)')
    } else {
      const ids = []
      walk(detailsNodes[0], node => { if (node.props?.className === 'dzag_toolrow' && node.props?.key) ids.push(node.props.key) })
      if (!(ids.includes('dzag-sim') && ids.includes('dzag-stress'))) {
        failures.push(`B-1: the advanced drawer should hold the sim + stress rows, got ${JSON.stringify(ids)}`)
      }
      if (ids.includes('dzag-coord')) failures.push('B-1: the coordination entry must not be inside the advanced drawer')
    }
    if (coordRows.length !== 1) failures.push(`B-1: the coordination entry should be one main-view row, got ${coordRows.length}`)
  } catch (error) {
    failures.push(`B-1 panel render threw: ${error?.message ?? error}`)
  }
}

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
  || bridgeHttp.path !== 'api/dzairground-ssh/vnc' || bridgeHttp.password !== 'carvnc') {
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

// ── the float's pure logic ────────────────────────────────────────────────────
// Uptime must stay readable across all three magnitudes (a freshly booted car
// counts seconds, one left on for weeks counts days), every unit the gauge
// prints must exist in both dictionaries or the card renders raw translation
// keys in the corner of the screen, and the title-bar stopwatch is m:ss with
// --:-- before the first reading.
const unitT = key => ({ unitDay: 'd', unitHour: 'h', unitMin: 'm', unitSec: 's' }[key] ?? key)
for (const [input, expected] of [[45, '45s'], [90, '1m 30s'], [3671, '1h 1m'], [90061, '1d 1h'], [NaN, '—'], [-3, '—']]) {
  const actual = client.__test.formatDuration(input, unitT)
  if (actual !== expected) failures.push(`formatDuration(${input}) = ${JSON.stringify(actual)}, want ${JSON.stringify(expected)}`)
}
for (const [input, expected] of [[45, '0:45'], [90, '1:30'], [3671, '61:11'], [90061, '1501:01'], [NaN, '--:--'], [-3, '--:--']]) {
  const actual = client.__test.formatUptime(input)
  if (actual !== expected) failures.push(`formatUptime(${input}) = ${JSON.stringify(actual)}, want ${JSON.stringify(expected)}`)
}
for (const dict of [client.__test.DICT.zh, client.__test.DICT.en]) {
  for (const key of ['viewFloat', 'floatToolDesc', 'floatTitle', 'floatBattery', 'floatCpu', 'floatUp', 'floatDown',
    'floatSync', 'floatAxisX', 'fieldFloatOpacity', 'opacityHint', 'unitDay', 'unitHour', 'unitMin', 'unitSec',
    'floatBattNone', 'floatBattVolts', 'floatBattDerived', 'floatLoading', 'floatError', 'floatDrag', 'floatClose',
    'floatRefresh', 'floatSince', 'floatLoad', 'toolClose',
    // S3: the merged card's drone cells, the three-state words (决议②) and
    // the unified absence vocabulary (t4 §2.4) must all be present in copy.
    'floatTemp', 'floatZones', 'fcNoFc', 'fcNoData', 'battBad', 'stateStale', 'probeAgo',
    'viewCoord', 'viewStress', 'nodeCar', 'nodeDrone',
    'reasonNoStack', 'reasonNoRos', 'reasonNoFc', 'reasonDdsSilent', 'reasonBadValue',
    'reasonUnreachable', 'reasonNeedsPassword', 'reasonHostKey', 'reasonNoSample', 'reasonNotSupported']) {
    if (typeof dict[key] !== 'string' || dict[key] === '') failures.push(`dictionary is missing the float key ${key}`)
  }
}

// The disconnect contract (reported: the header kept ticking from a dead
// anchor, so it neither froze honestly nor matched the car's real uptime):
// live only from a fresh car reading, 小车未连接 the moment the poll fails or
// the reading ages past the bound, --:-- before anything is tried — and a
// reconnect replaces the anchor with the car's own numbers.
const uptimeView = client.__test.uptimeView
{
  const at = 1_000_000
  const reading = { ok: true, uptimeSec: 600, at, bootEpoch: 400 }
  const live = uptimeView(reading, '', at + 3000)
  if (live.state !== 'live' || Math.abs(live.uptime - 603) > 1e-9) {
    failures.push(`uptimeView live should be /proc/uptime + age (603), got ${JSON.stringify(live)}`)
  }
  if (live.startedAt !== 400_000) {
    failures.push(`uptimeView should prefer the car's btime boot instant, got ${live.startedAt}`)
  }
  // No btime (older host answer): fall back to host-clock arithmetic.
  const noBtime = uptimeView({ ok: true, uptimeSec: 600, at }, '', at)
  if (noBtime.state !== 'live' || noBtime.startedAt !== at - 600_000) {
    failures.push(`uptimeView fallback boot instant wrong: ${JSON.stringify(noBtime)}`)
  }
  // A failed poll: down, and no uptime number to render from the stale sample.
  const down = uptimeView(reading, '连接超时(4511ms)', at + 3000)
  if (down.state !== 'down' || down.uptime !== undefined) {
    failures.push(`uptimeView with a failed poll must answer down with no number, got ${JSON.stringify(down)}`)
  }
  // A silent page (no polls ran): an old reading may not claim liveness.
  const stale = uptimeView(reading, '', at + client.__test.UPTIME_STALE_MS + 1)
  if (stale.state !== 'down') failures.push(`a reading older than the stale bound must be down, got ${JSON.stringify(stale)}`)
  // Just inside the bound still ticks.
  const edge = uptimeView(reading, '', at + client.__test.UPTIME_STALE_MS)
  if (edge.state !== 'live') failures.push(`a reading inside the bound must stay live, got ${JSON.stringify(edge)}`)
  // Nothing yet: the placeholder, not the down text.
  if (uptimeView(undefined, '', at).state !== 'loading') failures.push('before the first attempt the state must be loading')
}

// The plot scales — asserted numerically, because every line of this math is
// invisible in a screenshot until it is wrong: x ticks every 5 seconds across
// now±30 s with HH:MM:SS labels only at the ends and the centre, y ticks
// anchored on the current value (±10 points in 5 % steps, clamped into 0..100),
// the latest sample landing on the centre line as the red dot, and no NaN
// anywhere. Both gauges are percentages, so both share the same shape.
const chartModel = client.__test.chartModel
const chartNow = new Date(2024, 4, 1, 10, 0, 0).getTime()  // 10:00 local

const battModel = chartModel('battery', [{ at: chartNow - 20000, value: 40 }, { at: chartNow, value: 48.2 }], chartNow, 48.2)
if (battModel === null) failures.push('chartModel(battery) with a reading must not answer null')
else {
  if (Math.abs(battModel.lo - 38.2) > 1e-9 || Math.abs(battModel.hi - 58.2) > 1e-9) {
    failures.push(`battery y range should be current±10, got ${battModel.lo}..${battModel.hi}`)
  }
  if (battModel.yTicks.map(tick => tick.label).join(',') !== '38,43,48,53,58') {
    failures.push(`battery ticks should step 5 across ±10, got ${battModel.yTicks.map(tick => tick.label).join(',')}`)
  }
  if (battModel.unit !== '%') failures.push(`battery y axis should be labelled %, got ${battModel.unit}`)
  if (battModel.xTicks.length !== 13) failures.push(`x axis should have 13 ticks (5-s steps over ±30 s), got ${battModel.xTicks.length}`)
  const gap = battModel.xTicks[1].x - battModel.xTicks[0].x
  if (Math.abs(gap - (client.__test.CHART.r - client.__test.CHART.l) / 12) > 1e-9) {
    failures.push(`x tick spacing wrong: ${gap}`)
  }
  const xLabels = battModel.xTicks.map(tick => tick.label)
  if (xLabels.filter(label => label !== null).join(',') !== '09:59:30,10:00:00,10:00:30') {
    failures.push(`x labels should be −30 s/now/+30 s with second precision, got ${JSON.stringify(xLabels.filter(label => label !== null))}`)
  }
  if (!Number.isFinite(battModel.dot.x) || Math.abs(battModel.dot.x - (client.__test.CHART.l + client.__test.CHART.r) / 2) > 1e-9) {
    failures.push(`the dot at "now" should sit on the centre line, got x=${battModel.dot.x}`)
  }
  if (Math.abs(battModel.dot.y - (client.__test.CHART.t + client.__test.CHART.b) / 2) > 1e-9) {
    failures.push(`the dot for the current value should sit mid-box, got y=${battModel.dot.y}`)
  }
  if (battModel.points === '' || battModel.points.includes('NaN')) {
    failures.push(`battery polyline wrong: ${JSON.stringify(battModel.points)}`)
  } else if (!battModel.points.split(' ').at(-1).endsWith(',31.0')) {
    failures.push(`the polyline should end at the current value (y=31), got ${JSON.stringify(battModel.points)}`)
  }
}

// CPU: same shape in percent, but the range is clamped into 0..100 — the spec's
// ±10/5 % cannot print a negative floor or a >100 ceiling.
for (const [current, label, wantLo, wantHi, wantTicks] of [
  [30, 'mid-range', 20, 40, '20,25,30,35,40'],
  [95, 'near the ceiling', 85, 100, '85,90,95,100'],
  [5, 'near the floor', 0, 15, '0,5,10,15'],
]) {
  const model = chartModel('cpu', [{ at: chartNow, value: current }], chartNow, current)
  if (model === null) { failures.push(`chartModel(cpu) at ${current}% must not answer null`); continue }
  if (model.lo !== wantLo || model.hi !== wantHi) failures.push(`cpu range at ${current}% (${label}) should be ${wantLo}..${wantHi}, got ${model.lo}..${model.hi}`)
  const ticks = model.yTicks.map(tick => tick.label).join(',')
  if (ticks !== wantTicks) failures.push(`cpu ticks at ${current}% (${label}) should be ${wantTicks}, got ${ticks}`)
  if (model.yTicks.some(tick => !Number.isFinite(tick.y) || tick.y < client.__test.CHART.t - 1e-9 || tick.y > client.__test.CHART.b + 1e-9)) {
    failures.push(`cpu tick outside the plot box at ${current}%: ${JSON.stringify(model.yTicks)}`)
  }
}

// Degenerate inputs: no reading answers null (the renderer draws an empty
// frame), samples outside the ±30 s window or without a finite value are
// dropped rather than dragged onto the axis, and a flat series keeps the dot on
// the centre line instead of reading as a collapse.
if (chartModel('battery', [], chartNow, null) !== null || chartModel('cpu', [], chartNow, NaN) !== null) {
  failures.push('chartModel must answer null when the current value is not finite')
}
const base = chartModel('battery', [{ at: chartNow - 5000, value: 47 }, { at: chartNow, value: 48.2 }], chartNow, 48.2)
const polluted = chartModel('battery', [
  { at: chartNow - 45000, value: 10 },   // 45 s old: just outside the ±30 s window
  { at: chartNow - 700000, value: 10 },  // 11.7 min old: far outside it
  { at: chartNow, value: NaN },           // a poll that read no battery
  { at: chartNow - 5000, value: 47 },
  { at: chartNow, value: 48.2 },
], chartNow, 48.2)
if (base === null || polluted === null) failures.push('chartModel answered null for finite inputs')
else if (base.points !== polluted.points) failures.push(`out-of-window/NaN samples changed the curve: ${polluted.points} vs ${base.points}`)
const flat = chartModel('battery', [{ at: chartNow, value: 48.2 }, { at: chartNow - 5000, value: 48.2 }], chartNow, 48.2)
if (flat === null || flat.points === '' || flat.points.includes('NaN')) failures.push(`flat series should still plot: ${JSON.stringify(flat?.points)}`)
else if (flat.dot.y !== (client.__test.CHART.t + client.__test.CHART.b) / 2) failures.push(`flat series dot should sit mid-box, got y=${flat.dot.y}`)

// The float's stylesheet lives in a template literal, so `node --check` only
// proves the JavaScript parses — a missing `}` in the CSS is invisible to it
// and silently swallows every later rule (the card would render unstyled, and
// nothing reports an error). Assert balance and the selectors that carry the
// float's contract: two cells, the drag/close chrome, the opacity variable.
// Truncation by a stray backtick is already covered by loading this module —
// the leftover would be a syntax error and no test would run at all.
const cssLiteral = clientSource.match(/const CSS = `([\s\S]*?)`/)?.[1] ?? ''
const cssOpen = (cssLiteral.match(/{/g) ?? []).length
const cssClose = (cssLiteral.match(/}/g) ?? []).length
if (cssOpen !== cssClose || cssOpen === 0) {
  failures.push(`CSS braces unbalanced: ${cssOpen} '{' vs ${cssClose} '}'`)
}
for (const selector of ['.dzag_float', '.dzag_fcell', '.dzag_fhead', '.dzag_ftitle', '.dzag_fplot', '.dzag_fchart', 'repeat(2,1fr)', '--dzag-op']) {
  if (!cssLiteral.includes(selector)) failures.push(`CSS is missing the float rule/variable ${selector}`)
}
// The plot's gridline class is dzag_fgridline, not dzag_fgrid: that class styles
// the body grid (display:grid), and a stray match on an SVG line would paint
// that layout onto it.
if (!cssLiteral.includes('.dzag_fchart .dzag_fgridline')) {
  failures.push('CSS is missing the SVG gridline rule .dzag_fchart .dzag_fgridline')
}
if (/\.dzag_fchart[^{]*\.dzag_fgrid\b/.test(cssLiteral)) {
  failures.push('SVG gridline class collides with the body grid class dzag_fgrid')
}

server.close()
if (failures.length > 0) {
  console.log('\nFAILURES:')
  for (const failure of failures) console.log(' -', failure)
  process.exitCode = 1
} else {
  console.log('\nCLIENT TEST PASSED')
}
