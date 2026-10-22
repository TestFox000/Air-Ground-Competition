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
// drone address in ~/.dsh/dzdrone-ssh/settings.json.
process.env.DSH_HOME = fs.mkdtempSync(path.join(os.tmpdir(), 'dzdrone-ssh-client-test-'))

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
      // structural now — three cells each carrying 名称/数值/曲线, a close
      // button, an opacity slider, no standalone-window button — and `() => ({})`
      // would make all of that unassertable. Children arrive either spread or
      // as one array, so the walkers below flatten.
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
if (client.name !== 'dzdrone-ssh') throw new Error('wrong plugin name')

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
let activePanel = 'dzdrone-drone'
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
  host: 'dzsmart.local', user: 'duzhong', port: 22, sshAlias: '',
  vncHost: '', vncPort: 5901, vncMode: 'direct', vncPassword: '',
  filesRoot: '/home/duzhong', filesShortcuts: [{ label: '主目录', path: '/home/duzhong' }],
}
const storeStub = { get: () => ({ phase: 'idle', probe: { state: 'ready' } }), subscribe: () => () => {}, set: () => {} }
const { makeVncView, makeFilesView, makeDronePanel, makeVncTabShell, makeFilesTabShell, makeFloatWindow } = client.__test
const tKey = key => key
for (const [label, component] of [
  ['vnc', makeVncView(ctxClient, tKey)({ settings: settingsStub, origin: 'http://127.0.0.1:19387' })],
  // Also without an origin: the panel gets that value from an API response, and a
  // missing one must render the "no address" notice instead of throwing.
  ['vnc-without-origin', makeVncView(ctxClient, tKey)({ settings: settingsStub, origin: undefined })],
  ['files', makeFilesView(ctxClient, tKey)({ settings: settingsStub })],
  ['panel', makeDronePanel(ctxClient, storeStub, tKey)({})],
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

// The open float in full detail: two parallel cells (temperature, CPU), each
// the pair of blue boxes — 名称 + 数值 over an axes-bearing plot; the uptime on
// the header line instead of in the body; a close button wired to the store; an
// opacity slider; no trace of the removed standalone-window button — and a
// header that refuses to start a drag when the press lands on a button (pointer
// capture would retarget that click to the header itself: the reported
// "关不掉").
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

  // ── the header line: 名称 | 开机时间：**:** ────────────────────────────────
  const titleGroups = floatFind(tree, node => node.props?.className === 'dzd_ftitle')
  if (titleGroups.length !== 1) renderFailures.push(`expected one title group, got ${titleGroups.length}`)
  else {
    const parts = (titleGroups[0].children ?? []).flat()
    const [name, sep, up, batt] = parts
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
    if (up?.props?.className !== 'dzd_fup') renderFailures.push('header uptime missing the dzd_fup class')
    // The battery rides the header too; before a reading it reads 电量：—.
    const battText = Array.isArray(batt?.children) ? batt.children.join('') : String(batt?.children ?? '')
    if (batt?.type !== 'span' || battText !== 'floatBattery：—') {
      renderFailures.push(`header battery should read floatBattery：— before a reading, got ${JSON.stringify(battText)}`)
    }
    if (batt?.props?.className !== 'dzd_fbatt') renderFailures.push('header battery missing the dzd_fbatt class')
  }

  // ── the body: two cells, each 名称+数值 box over its plot box ─────────────
  const cells = floatFind(tree, node => node.props?.className === 'dzd_fcell')
  if (cells.length !== 2) renderFailures.push(`the float should render 2 parallel cells, got ${cells.length}`)
  const kinds = cells.map(cell => cell.props['data-kind'])
  if (kinds.join(',') !== 'temp,cpu') renderFailures.push(`cells should be temp,cpu (uptime left the body), got ${kinds.join(',')}`)
  for (const cell of cells) {
    const boxes = (cell.children ?? []).flat()
    if (boxes.length !== 2) { renderFailures.push(`cell ${cell.props['data-kind']} should be 2 boxes, got ${boxes.length}`); continue }
    const [valueBox, plotBox] = boxes
    if (valueBox?.props?.className !== 'dzd_fbox') renderFailures.push(`cell ${cell.props['data-kind']}: first box lost the dzd_fbox class`)
    const valueParts = (valueBox?.children ?? []).flat()
    if (valueParts[0]?.type !== 'span' || valueParts[1]?.type !== 'b') {
      renderFailures.push(`cell ${cell.props['data-kind']}: 名称+数值 box should be span then b, got ${JSON.stringify(valueParts.map(part => part?.type))}`)
    }
    if (!String(plotBox?.props?.className ?? '').includes('dzd_fplot')) {
      renderFailures.push(`cell ${cell.props['data-kind']}: second box should be the plot box`)
    }
    const charts = floatFind(plotBox, node => node.type === 'svg' && node.props?.className === 'dzd_fchart')
    if (charts.length !== 1) renderFailures.push(`cell ${cell.props['data-kind']} should carry exactly one plot, got ${charts.length}`)
    else if (charts[0].props.viewBox !== '0 0 144 78') renderFailures.push(`plot viewBox wrong: ${charts[0].props.viewBox}`)
    else {
      // First paint has no reading, so the plot must be its empty state: both
      // axes drawn plus the dashed baseline — no curve, no invented ticks.
      const kids = (charts[0].children ?? []).flat()
      const classes = kids.map(kid => String(kid?.props?.className ?? ''))
      if (classes.filter(name => name === 'dzd_faxis').length !== 2) {
        renderFailures.push(`cell ${cell.props['data-kind']}: empty plot should draw both axes, got ${JSON.stringify(classes)}`)
      }
      if (classes.includes('dzd_fno') !== true) renderFailures.push(`cell ${cell.props['data-kind']}: empty plot should draw the dashed baseline`)
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
  const heads = floatFind(tree, node => node.props?.className === 'dzd_fhead')
  if (heads.length !== 1) renderFailures.push(`expected one draggable header, got ${heads.length}`)
  else {
    let captures = 0
    const press = target => ({ target, clientX: 8, clientY: 8, pointerId: 1, currentTarget: { setPointerCapture: () => { captures += 1 } } })
    heads[0].props.onPointerDown(press({ closest: selector => (selector === 'button' ? {} : null) }))
    if (captures !== 0) renderFailures.push('pressing a header button started a drag — its click would be retargeted (关不掉 regression)')
    heads[0].props.onPointerDown(press({ closest: () => null }))
    if (captures !== 1) renderFailures.push('a press on the title bar should start a drag')
  }

  // The standalone-window button is gone.
  if (floatFind(tree, node => node.props?.title === 'floatPopup').length !== 0) {
    renderFailures.push('the standalone-window (独立小窗) button should have been removed')
  }

  // Opacity: 60% in the store must reach the card as --dzd-op, and the slider
  // must be a clamped 30..100 range whose changes reach the store.
  const style = tree.props?.style ?? {}
  if (style['--dzd-op'] !== '0.6') renderFailures.push(`opacity 60 should render --dzd-op:"0.6", got ${JSON.stringify(style['--dzd-op'])}`)
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
} catch (error) {
  renderFailures.push(`render(view:float:detail) threw: ${error?.message ?? error}`)
}

// The float's pure logic: the duration formatting and the dictionary keys are
// asserted below, once the shared `failures` list exists.

const phases = []
const result = await client.__test.connect(ctxClient, { closest: () => null }, phase => phases.push(phase))
console.log('\nconnect result:', JSON.stringify(result))
console.log('phases:', phases.join(' -> '))
console.log('writes:', JSON.stringify(writes))
console.log('renames:', JSON.stringify(renames))
console.log('diag steps:', diagPayloads.map(entry => entry.step).join(', '))

const failures = [...renderFailures]

// The float's pure logic: uptime must stay readable across all three magnitudes
// (a fresh Jetson counts seconds, a board left on for weeks counts days), and
// every unit the gauge prints must exist in both dictionaries or the card
// renders raw translation keys in the corner of the screen.
const unitT = key => ({ unitDay: 'd', unitHour: 'h', unitMin: 'm', unitSec: 's' }[key] ?? key)
for (const [input, expected] of [[45, '45s'], [90, '1m 30s'], [3671, '1h 1m'], [90061, '1d 1h'], [NaN, '—'], [-3, '—']]) {
  const actual = client.__test.formatDuration(input, unitT)
  if (actual !== expected) failures.push(`formatDuration(${input}) = ${JSON.stringify(actual)}, want ${JSON.stringify(expected)}`)
}
for (const dict of [client.__test.DICT.zh, client.__test.DICT.en]) {
  for (const key of ['viewFloat', 'floatToolDesc', 'floatTitle', 'floatTemp', 'floatCpu', 'floatUp', 'floatBattery', 'floatNoFc', 'floatBatteryV', 'floatDown', 'floatSync', 'floatAxisX', 'fieldFloatOpacity', 'opacityHint', 'unitDay', 'unitHour', 'unitMin', 'unitSec']) {
    if (typeof dict[key] !== 'string' || dict[key] === '') failures.push(`dictionary is missing the float key ${key}`)
  }
  for (const key of ['floatPopup', 'floatPopupHint']) {
    if (key in dict) failures.push(`dictionary still carries the removed standalone-window key ${key}`)
  }
}

// The title-bar uptime: m:ss for any elapsed time — minutes are the drone's
// total elapsed (no hours: it is a stopwatch, so 90061 s reads 1501:01), and
// --:-- until the first reading exists.
const uptimeT = key => key
for (const [input, expected] of [[45, '0:45'], [90, '1:30'], [3671, '61:11'], [90061, '1501:01'], [NaN, '--:--'], [-3, '--:--']]) {
  const actual = client.__test.formatUptime(input, uptimeT)
  if (actual !== expected) failures.push(`formatUptime(${input}) = ${JSON.stringify(actual)}, want ${JSON.stringify(expected)}`)
}

// The disconnect contract (reported: the header kept ticking from a dead
// anchor, so it neither froze honestly nor matched the drone's real uptime):
// live only from a fresh drone reading, 无人机未连接 the moment the poll fails
// or the reading ages past the bound, --:-- before anything is tried — and a
// reconnect replaces the anchor with the drone's own numbers.
const uptimeView = client.__test.uptimeView
{
  const at = 1_000_000
  const reading = { ok: true, uptimeSec: 600, at, bootEpoch: 400 }
  const live = uptimeView(reading, '', at + 3000)
  if (live.state !== 'live' || Math.abs(live.uptime - 603) > 1e-9) {
    failures.push(`uptimeView live should be /proc/uptime + age (603), got ${JSON.stringify(live)}`)
  }
  if (live.startedAt !== 400_000) {
    failures.push(`uptimeView should prefer the drone's btime boot instant, got ${live.startedAt}`)
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

// The battery verdict: the drone's battery lives in the PX4 flight controller,
// so "missing" is 无飞控 (FC unplugged), not a fake 0%; only a fresh reading
// with a sane percent claims a number; a disconnected drone collapses to a
// dash rather than stale digits.
const batteryView = client.__test.batteryView
{
  const at = 1_000_000
  // A live battery: percent + voltage.
  const live = batteryView({ ok: true, at, battery: { percent: 65.5, voltage: 11.33 } }, '', at + 2000)
  if (live.state !== 'live' || Math.abs(live.percent - 65.5) > 1e-9 || Math.abs(live.voltage - 11.33) > 1e-9) {
    failures.push(`batteryView live should carry percent+voltage, got ${JSON.stringify(live)}`)
  }
  // A live battery with no voltage (older FC firmware): still live, voltage absent.
  const noVolt = batteryView({ ok: true, at, battery: { percent: 90 } }, '', at)
  if (noVolt.state !== 'live' || noVolt.voltage !== undefined) {
    failures.push(`batteryView without voltage should stay live with voltage undefined, got ${JSON.stringify(noVolt)}`)
  }
  // FC absent: drone fine, no battery field → nofc, not 0%.
  const noFc = batteryView({ ok: true, at }, '', at)
  if (noFc.state !== 'nofc') failures.push(`batteryView with no battery field should be nofc, got ${JSON.stringify(noFc)}`)
  // An implausible battery is also nofc: never paint a fabricated percent.
  const bad = batteryView({ ok: true, at, battery: { percent: -1 } }, '', at)
  if (bad.state !== 'nofc') failures.push(`batteryView with an out-of-range percent should be nofc, got ${JSON.stringify(bad)}`)
  // A failed poll: down (the header already says 无人机未连接).
  const down = batteryView({ ok: true, at, battery: { percent: 50, voltage: 11 } }, '连接超时', at)
  if (down.state !== 'down') failures.push(`batteryView with a failed poll should be down, got ${JSON.stringify(down)}`)
  // A stale sample may not claim the drone's battery.
  const stale = batteryView({ ok: true, at, battery: { percent: 50 } }, '', at + client.__test.UPTIME_STALE_MS + 1)
  if (stale.state !== 'down') failures.push(`batteryView with a stale sample should be down, got ${JSON.stringify(stale)}`)
  // Before the first attempt: loading.
  if (batteryView(undefined, '', at).state !== 'loading') failures.push('batteryView before the first attempt should be loading')
}

// The plot scales — asserted numerically, because every line of this math is
// invisible in a screenshot until it is wrong: x ticks every 5 seconds across
// now±30 s with HH:MM:SS labels only at the ends and the centre, y ticks
// anchored on the current value (±5 °C in 2.5° steps; ±10 points in 5% steps,
// clamped into 0..100), the latest sample landing on the centre line as the red
// dot, and no NaN anywhere.
const chartModel = client.__test.chartModel
const now = new Date(2024, 4, 1, 10, 0, 0).getTime()  // 10:00 local

const tempModel = chartModel('temp', [{ at: now - 20000, value: 40 }, { at: now, value: 48.2 }], now, 48.2)
if (tempModel === null) failures.push('chartModel(temp) with a reading must not answer null')
else {
  if (Math.abs(tempModel.lo - 43.2) > 1e-9 || Math.abs(tempModel.hi - 53.2) > 1e-9) {
    failures.push(`temp y range should be current±5, got ${tempModel.lo}..${tempModel.hi}`)
  }
  if (tempModel.yTicks.map(tick => tick.label).join(',') !== '43.2,45.7,48.2,50.7,53.2') {
    failures.push(`temp ticks should step 2.5 across ±5, got ${tempModel.yTicks.map(tick => tick.label).join(',')}`)
  }
  if (tempModel.unit !== '°C') failures.push(`temp y axis should be labelled °C, got ${tempModel.unit}`)
  if (tempModel.xTicks.length !== 13) failures.push(`x axis should have 13 ticks (5-s steps over ±30 s), got ${tempModel.xTicks.length}`)
  const gap = tempModel.xTicks[1].x - tempModel.xTicks[0].x
  if (Math.abs(gap - (client.__test.CHART.r - client.__test.CHART.l) / 12) > 1e-9) {
    failures.push(`x tick spacing wrong: ${gap}`)
  }
  const xLabels = tempModel.xTicks.map(tick => tick.label)
  if (xLabels.filter(label => label !== null).join(',') !== '09:59:30,10:00:00,10:00:30') {
    failures.push(`x labels should be −30 s/now/+30 s with second precision, got ${JSON.stringify(xLabels.filter(label => label !== null))}`)
  }
  if (!Number.isFinite(tempModel.dot.x) || Math.abs(tempModel.dot.x - (client.__test.CHART.l + client.__test.CHART.r) / 2) > 1e-9) {
    failures.push(`the dot at "now" should sit on the centre line, got x=${tempModel.dot.x}`)
  }
  if (Math.abs(tempModel.dot.y - (client.__test.CHART.t + client.__test.CHART.b) / 2) > 1e-9) {
    failures.push(`the dot for the current value should sit mid-box, got y=${tempModel.dot.y}`)
  }
  if (tempModel.points === '' || tempModel.points.includes('NaN')) {
    failures.push(`temp polyline wrong: ${JSON.stringify(tempModel.points)}`)
  } else if (!tempModel.points.split(' ').at(-1).endsWith(',31.0')) {
    failures.push(`the polyline should end at the current value (y=31), got ${JSON.stringify(tempModel.points)}`)
  }
}

// CPU: same shape in percent, but the range is clamped into 0..100 — the spec's
// ±10/5% cannot print a negative floor or a >100 ceiling.
for (const [current, label, wantLo, wantHi, wantTicks] of [
  [30, 'mid-range', 20, 40, '20,25,30,35,40'],
  [95, 'near the ceiling', 85, 100, '85,90,95,100'],
  [5, 'near the floor', 0, 15, '0,5,10,15'],
]) {
  const model = chartModel('cpu', [{ at: now, value: current }], now, current)
  if (model === null) { failures.push(`chartModel(cpu) at ${current}% must not answer null`); continue }
  if (model.lo !== wantLo || model.hi !== wantHi) failures.push(`cpu range at ${current}% (${label}) should be ${wantLo}..${wantHi}, got ${model.lo}..${model.hi}`)
  const ticks = model.yTicks.map(tick => tick.label).join(',')
  if (ticks !== wantTicks) failures.push(`cpu ticks at ${current}% (${label}) should be ${wantTicks}, got ${ticks}`)
  if (model.unit !== '%') failures.push(`cpu y axis should be labelled %, got ${model.unit}`)
  if (model.yTicks.some(tick => !Number.isFinite(tick.y) || tick.y < client.__test.CHART.t - 1e-9 || tick.y > client.__test.CHART.b + 1e-9)) {
    failures.push(`cpu tick outside the plot box at ${current}%: ${JSON.stringify(model.yTicks)}`)
  }
}

// Degenerate inputs: no reading answers null (the renderer draws an empty
// frame), samples outside the ±30 s window or without a finite value are
// dropped rather than dragged onto the axis, and a flat series keeps the dot on
// the centre line instead of reading as a collapse.
if (chartModel('temp', [], now, null) !== null || chartModel('cpu', [], now, NaN) !== null) {
  failures.push('chartModel must answer null when the current value is not finite')
}
const base = chartModel('temp', [{ at: now - 5000, value: 47 }, { at: now, value: 48.2 }], now, 48.2)
const polluted = chartModel('temp', [
  { at: now - 45000, value: 10 },   // 45 s old: just outside the ±30 s window
  { at: now - 700000, value: 10 },  // 11.7 min old: far outside it
  { at: now, value: NaN },           // a poll that read no temperature
  { at: now - 5000, value: 47 },
  { at: now, value: 48.2 },
], now, 48.2)
if (base === null || polluted === null) failures.push('chartModel answered null for finite inputs')
else if (base.points !== polluted.points) failures.push(`out-of-window/NaN samples changed the curve: ${polluted.points} vs ${base.points}`)
const flat = chartModel('temp', [{ at: now, value: 48.2 }, { at: now - 5000, value: 48.2 }], now, 48.2)
if (flat === null || flat.points === '' || flat.points.includes('NaN')) failures.push(`flat series should still plot: ${JSON.stringify(flat?.points)}`)
else if (flat.dot.y !== (client.__test.CHART.t + client.__test.CHART.b) / 2) failures.push(`flat series dot should sit mid-box, got y=${flat.dot.y}`)

// The float's stylesheet lives in a template literal, so `node --check` only
// proves the JavaScript parses — a missing `}` in the CSS is invisible to it
// and silently swallows every later rule (the card would render unstyled, and
// nothing reports an error). Assert balance and the selectors that carry the
// float's contract: three cells, the drag/close chrome, the opacity variable.
// Truncation by a stray backtick is already covered by loading this module —
// the leftover would be a syntax error and no test would run at all.
const cssSource = fs.readFileSync(new URL('../client.js', import.meta.url), 'utf8')
const cssLiteral = cssSource.match(/const CSS = `([\s\S]*?)`/)?.[1] ?? ''
const cssOpen = (cssLiteral.match(/{/g) ?? []).length
const cssClose = (cssLiteral.match(/}/g) ?? []).length
if (cssOpen !== cssClose || cssOpen === 0) {
  failures.push(`CSS braces unbalanced: ${cssOpen} '{' vs ${cssClose} '}'`)
}
for (const selector of ['.dzd_float', '.dzd_fcell', '.dzd_fhead', '.dzd_ftitle', '.dzd_fplot', '.dzd_fchart', 'repeat(2,1fr)', '--dzd-op']) {
  if (!cssLiteral.includes(selector)) failures.push(`CSS is missing the float rule/variable ${selector}`)
}
// The plot's gridline class is dzd_fgridline, not dzd_fgrid: that class styles
// the body grid (display:grid), and a stray match on an SVG line would paint
// that layout onto it.
if (!cssLiteral.includes('.dzd_fchart .dzd_fgridline')) {
  failures.push('CSS is missing the SVG gridline rule .dzd_fchart .dzd_fgridline')
}
if (/\.dzd_fchart[^{]*\.dzd_fgrid\b/.test(cssLiteral)) {
  failures.push('SVG gridline class collides with the body grid class dzd_fgrid')
}

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
activePanel = 'dzdrone-drone'
panelHistory.length = 0
const toolResult = await client.__test.openToolTab(ctxClient, 'dzd-files')
console.log('openToolTab result:', JSON.stringify(toolResult), '| opened:', JSON.stringify(openedKinds))

const vncType = tabTypes.find(definition => definition.kind === 'dzd-vnc')
const filesType = tabTypes.find(definition => definition.kind === 'dzd-files')
const paneRegistrations = registered['sidebar.right.pane.tab'] ?? []

if (toolResult.ok !== true) failures.push(`openToolTab did not succeed: ${JSON.stringify(toolResult)}`)
if (!openedKinds.includes('dzd-files')) failures.push(`openTab never called with the tool kind (opened: ${JSON.stringify(openedKinds)})`)
if (!panelHistory.includes(null)) failures.push('tool path did not switch to the Conversation first')
if (vncType === undefined || vncType.id !== 'dsh-dzdrone-ssh/vnc') {
  failures.push(`vnc tab type missing or wrong id: ${JSON.stringify(vncType)}`)
} else if (vncType.multiple !== false || vncType.keepMounted !== true) {
  failures.push(`vnc tab type flags wrong (multiple:${vncType.multiple} keepMounted:${vncType.keepMounted})`)
}
if (filesType === undefined || filesType.id !== 'dsh-dzdrone-ssh/files' || filesType.multiple !== false) {
  failures.push(`files tab type missing or wrong: ${JSON.stringify(filesType)}`)
}
// TabSlot dispatches through entryKey: definition.id — the slot key must be the
// type id, or the body renders the "type no longer available" fallback.
for (const id of ['dsh-dzdrone-ssh/vnc', 'dsh-dzdrone-ssh/files']) {
  if (!paneRegistrations.some(registration => registration.definition.key === id)) {
    failures.push(`no sidebar.right.pane.tab body registered under key ${id}`)
  }
}
// Four rows now: the three tab tools plus the float, which opens as an overlay
// card rather than a tab.
if (client.__test.TOOLS.length !== 4) failures.push(`tool list should have 4 rows, has ${client.__test.TOOLS.length}`)
const floatRow = client.__test.TOOLS.find(tool => tool.id === 'float')
if (floatRow === undefined || floatRow.mode !== 'float') {
  failures.push(`the float tool row is missing or not mode:"float": ${JSON.stringify(floatRow)}`)
}
// The float lives on the frame-wide overlay seat; if that registration ever
// stops happening the card has nowhere to mount and the tool row silently
// toggles a flag nothing renders.
const overlayRegistrations = registered['shell.overlay'] ?? []
if (!overlayRegistrations.some(entry => entry.definition.id === 'dzdrone-float')) {
  failures.push(`no shell.overlay registration under id dzdrone-float (got ${JSON.stringify(overlayRegistrations.map(entry => entry.definition.id))})`)
}

// ── the generic VNC connector panel ────────────────────────────────────────────
// The VNC tab no longer mounts the fixed vnc-app view: it hosts this panel,
// which is one surface serving both the drone (prefilled with the plugin's own
// bridge, no websockify on board) and any other ws://host:port/websockify
// endpoint — the same design the car plugin settled on.
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
// The connector replaced the drone VNC view, so it must NOT register its own
// sidebar icon or main panel — the one VNC surface is the tab body.
const connectorPanes = registered.main ?? []
if (connectorPanes.some(entry => entry.definition.key === connector.CONNECTOR_ID)) {
  failures.push(`the connector must not register its own main panel (${connector.CONNECTOR_ID}) — it lives in the VNC tab`)
}
const connectorIcons = registered['sidebar.panellist'] ?? []
if (connectorIcons.some(entry => entry.definition.id === connector.CONNECTOR_ID)) {
  failures.push(`the connector must not register its own sidebar icon (${connector.CONNECTOR_ID}) — it lives in the VNC tab`)
}
// The VNC tab body must host the connector panel, not the legacy view. A
// static render only shows the loading note (the status fetch resolves later),
// so the wiring is asserted at source level: the tab shell mounts the connector
// and no longer mounts the legacy view.
const clientSource = fs.readFileSync(new URL('../client.js', import.meta.url), 'utf8')
const tabShellSource = clientSource.slice(clientSource.indexOf('function makeVncTabShell'), clientSource.indexOf('function makeFilesTabShell'))
if (tabShellSource.includes('makeVncView(')) {
  failures.push('the VNC tab body still mounts the legacy drone VNC view instead of the connector')
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

// ── the drone bridge: the one-click way back to a target that always works ─────
// The failure this guards was reported as a bare `1006` with the form pointed at
// a websockify that was never installed, and no way back but retyping every
// field. The prefill and the fallback button must therefore be the *same*
// computation, or they can drift and the escape hatch lands somewhere else.
// The VNC 主机 it fills is `localhost` (the bridge dials this machine's own
// host half), not the origin's raw loopback IP.
const bridgeHttp = connector.bridgeConnectorConfig('http://127.0.0.1:19387', connector.normalizeConnectorConfig({}), 'dronevnc')
if (bridgeHttp.scheme !== 'ws' || bridgeHttp.host !== 'localhost' || bridgeHttp.port !== '19387'
  || bridgeHttp.path !== 'api/dzdrone-ssh/vnc' || bridgeHttp.password !== 'dronevnc') {
  failures.push(`bridgeConnectorConfig (http) wrong: ${JSON.stringify(bridgeHttp)}`)
}
// A non-loopback origin must keep its own hostname — localhost would name the
// wrong machine there.
const bridgeLan = connector.bridgeConnectorConfig('http://192.168.1.10:19387', connector.normalizeConnectorConfig({}), 'x')
if (bridgeLan.host !== '192.168.1.10') {
  failures.push(`bridgeConnectorConfig must keep a non-loopback origin host, got ${JSON.stringify(bridgeLan)}`)
}
const bridgeHttps = connector.bridgeConnectorConfig('https://dsh.example', connector.normalizeConnectorConfig({}), 'dronevnc')
if (bridgeHttps.scheme !== 'wss' || bridgeHttps.port !== '443') {
  failures.push(`bridgeConnectorConfig must map https -> wss on 443: ${JSON.stringify(bridgeHttps)}`)
}
const bridgeKeepsPassword = connector.bridgeConnectorConfig('http://127.0.0.1:9', connector.normalizeConnectorConfig({ password: 'typed' }), 'dronevnc')
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
  failures.push('the connector must offer the one-click drone-bridge fallback')
}
if (!connectorPanelSource.includes('connHintNoListener') || !connectorPanelSource.includes('connHintBridgeFailed')) {
  failures.push('the connector must explain a 1006 in terms of the target it was pointed at')
}
if (!connectorPanelSource.includes('api/dzdrone-ssh/vnc-connect.html')) {
  failures.push('the connector must load its document from this plugin\'s own origin')
}
// Both copies of every new string: a key added to one language only shows up as
// a raw identifier in the other, which no other test would catch.
for (const key of ['connDroneBridge', 'connDroneBridgeHint', 'connDroneBridgeDone', 'connHintNoListener', 'connHintBridgeFailed', 'vncConnectorTitle']) {
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
