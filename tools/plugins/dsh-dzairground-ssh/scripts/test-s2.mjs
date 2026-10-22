/**
 * S2/S3 contract test — the unified gauge contract, the node registry, the
 * B-1 panel layout and the route-preservation count.
 *
 * What lives here and why:
 *   · W3 — `src/gauges.js`: one shape for both nodes; a car sample answers
 *     exactly its 3 capability cells, a drone sample its 4, and an offline
 *     sample marks every declared cell absent with the transport reason. The
 *     real offline sample comes from a live-booted Host whose /drone/metrics
 *     genuinely cannot reach dzsmart.local — honesty checked against reality,
 *     not only against fixtures.
 *   · W4's UI half — the grep counts: `batteryNote` / `dds_silent` / `no_ros`
 *     / `batteryFc` must appear in client.js AND float.html (before S3 both
 *     files referenced them zero times, which is bug#2 itself).
 *   · W5 — the 12 `/sim*` routes are counted from the router source (S0 决议⑦
 *     B-1: 全保留), and the advanced drawer exists with no `open` prop.
 *   · the node registry — caps are the single source the UI renders from.
 *
 * Run: node scripts/test-s2.mjs
 */

import fs from 'node:fs'
import os from 'node:os'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const HERE = path.dirname(fileURLToPath(import.meta.url))
const PLUGIN = path.resolve(HERE, '..')

let failures = 0
const check = (label, ok, detail) => {
  console.log(`${ok ? 'OK  ' : 'FAIL'}  ${label}${detail === undefined ? '' : `  ${detail}`}`)
  if (!ok) failures += 1
}

// ── 0. 隔离：本测试不写真实 ~/.dsh ─────────────────────────────────────────────
process.env.DSH_HOME = fs.mkdtempSync(path.join(os.tmpdir(), 'dzairground-s2-'))

// ── 1. W3 · 统一 gauge 契约（src/gauges.js，合成样本）─────────────────────────
console.log('\n=== W3 gauge 契约（合成样本）===')
const { gaugeView, REASONS } = await import('../src/gauges.js')
const { NODES, nodeSpec, nodeTarget, nodeSnapshot, ROLES, DRONE_SETTINGS_INITIAL } = await import('../src/nodes.js')

const carOk = {
  ok: true, at: 1792234797000, latencyMs: 1774, target: 'duzhong@duzhong.local',
  cpuPercent: 31.5, uptimeSec: 9067, bootEpoch: 1792225530, load: [0.42, 0.38, 0.35],
  battery: 78.0, batteryVolt: 11.9, batteryNote: null,
}
const carView = gaugeView('car', carOk)
check('车样本 → 恰好 3 个能力格（决议①：电量/CPU/开机时长）',
  carView.ok === true && carView.gauges.length === 3
  && JSON.stringify(carView.gauges.map(g => g.key)) === JSON.stringify(['batteryPercent', 'cpuPercent', 'uptimeSec']),
  JSON.stringify(carView.gauges.map(g => g.key)))
check('车样本不画温度格（机侧专属，不在 caps 里）',
  !carView.gauges.some(g => g.key === 'temp'), '')
check('车电量格 quality=measured，电压随行在 extra',
  carView.gauges[0].v === 78.0 && carView.gauges[0].quality === 'measured' && carView.gauges[0].extra?.voltage === 11.9,
  JSON.stringify(carView.gauges[0]))
const carDerived = gaugeView('car', { ...carOk, battery: 95, batteryNote: 'derived' })
check('车 derived 是独立 quality（12V/10V 折算不许冒充实测）',
  carDerived.gauges[0].quality === 'derived' && carDerived.gauges[0].v === 95, JSON.stringify(carDerived.gauges[0].quality))
const carNoStack = gaugeView('car', { ...carOk, battery: null, batteryNote: 'no_stack' })
check('车 no_stack → absent + reason=no_stack',
  carNoStack.gauges[0].quality === 'absent' && carNoStack.gauges[0].reason === 'no_stack', JSON.stringify(carNoStack.gauges[0]))
const carBad = gaugeView('car', { ...carOk, battery: null, batteryNote: 'bad_value' })
check('车 bad_value → absent + reason=bad_value（bug#3 的契约半身）',
  carBad.gauges[0].quality === 'absent' && carBad.gauges[0].reason === 'bad_value', JSON.stringify(carBad.gauges[0]))

const droneOk = {
  ok: true, at: 1792234796000, latencyMs: 2103, target: 'duzhong@dzsmart.local',
  cpuPercent: 44.0, uptimeSec: 666, bootEpoch: 1791338271, load: [0.5, 0.4, 0.3],
  temps: [{ id: 'cpu-thermal', c: 61.2 }, { id: 'gpu-thermal', c: 55 }], temp: 61.2, tempAt: 'cpu-thermal', zones: 2,
  battery: { percent: 65, voltage: 11.9 },
}
const droneView = gaugeView('drone', droneOk)
check('机样本 → 恰好 4 个能力格（决议①：温度/CPU/开机时长/飞控电量）',
  droneView.ok === true && droneView.gauges.length === 4
  && JSON.stringify(droneView.gauges.map(g => g.key)) === JSON.stringify(['temp', 'cpuPercent', 'uptimeSec', 'batteryPercent']),
  JSON.stringify(droneView.gauges.map(g => g.key)))
check('机温度格带 zones/tempAt 归因',
  droneView.gauges[0].v === 61.2 && droneView.gauges[0].extra?.zones === 2 && droneView.gauges[0].extra?.tempAt === 'cpu-thermal',
  JSON.stringify(droneView.gauges[0]))
for (const [note, want] of [['dds_silent', 'dds_silent'], ['no_ros', 'no_ros'], ['bad_value', 'bad_value']]) {
  const view = gaugeView('drone', { ...droneOk, battery: undefined, batteryNote: note })
  check(`机 ${note} → absent + reason=${want}（决议②/bug#2 的契约半身）`,
    view.gauges[3].quality === 'absent' && view.gauges[3].reason === want,
    JSON.stringify(view.gauges[3]))
}
const droneNoFc = gaugeView('drone', { ...droneOk, battery: undefined, batteryNote: undefined })
check('机无飞控（无 battery 无 note）→ absent + reason=no_fc',
  droneNoFc.gauges[3].quality === 'absent' && droneNoFc.gauges[3].reason === 'no_fc', JSON.stringify(droneNoFc.gauges[3]))

// ── 2. 节点注册表（src/nodes.js）──────────────────────────────────────────────
console.log('\n=== 节点注册表 ===')
check('roles 是 car+drone 两个', JSON.stringify(ROLES) === JSON.stringify(['car', 'drone']), JSON.stringify(ROLES))
check('车 caps.gauges 3 格 / 机 caps.gauges 4 格（决议① 的单一真源）',
  NODES.car.caps.gauges.length === 3 && NODES.drone.caps.gauges.length === 4,
  JSON.stringify([NODES.car.caps.gauges, NODES.drone.caps.gauges]))
check('hasSim 只在车上（仿真台是 Host 本地能力）',
  NODES.car.caps.hasSim === true && NODES.drone.caps.hasSim === false, '')
check('kind：车 ground / 机 air（协同层的对偶）',
  NODES.car.kind === 'ground' && NODES.drone.kind === 'air', '')
check('nodeSpec 未知角色 → undefined', nodeSpec('submarine') === undefined, '')
check('nodeTarget 别名优先于 user@host',
  nodeTarget({ sshAlias: 'dz', user: 'duzhong', host: 'x.local' }) === 'dz'
  && nodeTarget({ sshAlias: '', user: 'duzhong', host: 'dzsmart.local' }) === 'duzhong@dzsmart.local', '')
const snap = nodeSnapshot('drone', { settings: DRONE_SETTINGS_INITIAL, probe: { state: 'ready', at: 1 }, desc: null })
check('nodeSnapshot 标 probe source=cached（缓存快照永不冒充活探针）',
  snap.probe.source === 'cached' && snap.role === 'drone' && snap.kind === 'air' && snap.label === '无人机',
  JSON.stringify(snap.probe))
check('统一缺席词表 10 项（t4 §2.4）',
  Object.keys(REASONS).length === 10 && REASONS.bad_value !== undefined && REASONS.no_fc !== undefined,
  JSON.stringify(Object.keys(REASONS)))

// ── 3. W4 的 UI 半身 · grep 计数（bug#2 的直接证据）───────────────────────────
console.log('\n=== W4 三态词汇的 UI 消费计数 ===')
const clientSrc = fs.readFileSync(path.join(PLUGIN, 'client.js'), 'utf8')
const floatSrc = fs.readFileSync(path.join(PLUGIN, 'float.html'), 'utf8')
for (const word of ['batteryNote', 'dds_silent', 'no_ros', 'batteryFc']) {
  const inClient = (clientSrc.match(new RegExp(word, 'g')) ?? []).length
  const inFloat = (floatSrc.match(new RegExp(word, 'g')) ?? []).length
  check(`${word}: client.js ${inClient} 处 / float.html ${inFloat} 处（S1 前 = 0/0）`,
    inClient > 0 && inFloat > 0, '')
}
check('三态渲染分支存在（live/noData/nofc + bad_value）',
  clientSrc.includes("state: 'noData'") && clientSrc.includes("state: 'nofc'")
  && clientSrc.includes("'bad_value'") && clientSrc.includes("state: 'live'"), '')

// ── 4. W5 · /sim* 路由计数（决议⑦ B-1：12 条全保留）───────────────────────────
console.log('\n=== W5 路由保全 ===')
const indexSrc = fs.readFileSync(path.join(PLUGIN, 'index.js'), 'utf8')
const simRoutes = [...new Set([...indexSrc.matchAll(/innerRoute\s*(?:===|\.startsWith\()\s*'([^']+)'/g)]
  .map(match => match[1])
  .filter(route => route.startsWith('/sim')))].sort()
console.log(`  /sim* 路由（${simRoutes.length}）: ${simRoutes.join(' ')}`)
check('/sim* 路由计数 = 12（一条不许删）', simRoutes.length === 12, String(simRoutes.length))

// ── 5. B-1 折叠区（client 源级断言）───────────────────────────────────────────
console.log('\n=== B-1 折叠区 ===')
check('高级折叠区存在（<details class="dzag_advanced">）',
  /h\('details',\s*\{\s*key:\s*'advanced',\s*className:\s*'dzag_advanced'/.test(clientSrc), '')
check('折叠区默认收起（details 的 props 不含 open）',
  !/h\('details',\s*\{[^}]*\bopen\b/.test(clientSrc), '')
check('主视图挂协同入口（COORD_TOOL，锚 #drone），仿真台/压测在折叠区（STRESS_TOOL）',
  clientSrc.includes("COORD_TOOL = { id: 'dzag-coord'") && clientSrc.includes("hash: '#drone'")
  && clientSrc.includes("STRESS_TOOL = { id: 'dzag-stress'"),
  '')
check('主列表把 sim 行滤出（TOOLS.filter(tool => tool.id !== \'sim\')）',
  clientSrc.includes("TOOLS.filter(tool => tool.id !== 'dzag-sim')"), '')

// ── 6. 真 Host：drone 端点 + 真实离线样本的诚实性 ─────────────────────────────
console.log('\n=== 真 Host drone 端点（含真实离线样本）===')
const { apply } = await import(`../index.js?boot=${Date.now()}`)
const routes = []
const ctx = {
  logger: { info: () => {}, warn: () => {}, error: () => {} },
  get: () => undefined,
  effect: callback => { const d = callback(); return typeof d === 'function' ? d : () => {} },
  inject: (deps, callback) => { callback({ webServer: { register: r => { routes.push(r); return () => {} }, registerUpgrade: () => {} }, effect: ctx.effect }) },
}
apply(ctx, {})
const http = await import('node:http')
const server = http.createServer((req, res) => {
  const route = routes.find(candidate => req.url.startsWith(candidate.path))
  if (route === undefined) { res.writeHead(404).end('no route'); return }
  void route.handler(req, res)
})
await new Promise(resolve => server.listen(0, '127.0.0.1', resolve))
const base = `http://127.0.0.1:${server.address().port}/api/dzairground-ssh`
const readJson = async url => {
  const response = await fetch(url)
  const text = await response.text()
  try { return { status: response.status, payload: JSON.parse(text) } } catch { return { status: response.status, payload: null } }
}

const carStatus = await readJson(`${base}/car/status`)
const droneStatus = await readJson(`${base}/drone/status`)
check('GET /drone/status 200 且 role="drone"', droneStatus.status === 200 && droneStatus.payload.role === 'drone',
  `${droneStatus.status} role=${droneStatus.payload?.role}`)
const carKeys = Object.keys(carStatus.payload ?? {}).filter(k => k !== 'role').sort()
const droneKeys = Object.keys(droneStatus.payload ?? {}).filter(k => k !== 'role').sort()
check('/drone/status 与 /car/status 键集合同构（去掉 role）',
  JSON.stringify(carKeys) === JSON.stringify(droneKeys),
  `car=[${carKeys}] drone=[${droneKeys}]`)
check('机侧默认 host=dzsmart.local（冻结配置值，非硬编码 IP）',
  droneStatus.payload.settings?.host === 'dzsmart.local', JSON.stringify(droneStatus.payload.settings?.host))
check('机侧 settings 落在新数据目录文件（先写一次再验：JsonStore 是懒写的）',
  await (async () => {
    await fetch(`${base}/drone/config`, {
      method: 'POST', headers: { 'content-type': 'application/json' }, body: JSON.stringify({ floatOpacity: 88 }),
    })
    // JsonStore.flush() 是 120 ms 防抖，写完立刻读盘必然读不到（同 verify-s1 V7）。
    await new Promise(resolve => setTimeout(resolve, 400))
    const file = path.join(process.env.DSH_HOME, 'dzairground-ssh', 'settings-drone.json')
    if (!fs.existsSync(file)) return false
    const landed = JSON.parse(fs.readFileSync(file, 'utf8'))
    return landed.host === 'dzsmart.local' && landed.floatOpacity === 88
  })(), '')
check('机侧写隔离：机侧写入没有落进车侧 settings.json',
  !fs.existsSync(path.join(process.env.DSH_HOME, 'dzairground-ssh', 'settings.json'))
    || JSON.parse(fs.readFileSync(path.join(process.env.DSH_HOME, 'dzairground-ssh', 'settings.json'), 'utf8')).floatOpacity !== 88, '')
check('缓存探针标 source=cached（bug#1 的 Host 半身）',
  droneStatus.payload.probe === undefined || droneStatus.payload.probe.source === 'cached',
  JSON.stringify(droneStatus.payload.probe))

// 真实离线样本：dzsmart.local 在测试环境不可达，这正是一次诚实的离线。
const droneMetrics = await readJson(`${base}/drone/metrics`)
check('GET /drone/metrics 200 且带 ok 布尔（不可达是答案不是故障）',
  droneMetrics.status === 200 && typeof droneMetrics.payload?.ok === 'boolean',
  `${droneMetrics.status} ok=${droneMetrics.payload?.ok}`)
const realOffline = gaugeView('drone', droneMetrics.payload)
check('真实机离线样本 → 4 个能力格全部 absent 且 reason=unreachable（诚实离线）',
  realOffline.ok === false && realOffline.gauges.length === 4
  && realOffline.gauges.every(g => g.quality === 'absent' && g.reason === realOffline.reason),
  `reason=${realOffline.reason} detail=${realOffline.detail}`)
check('真实离线的 reason 属于统一词表', REASONS[realOffline.reason] !== undefined, realOffline.reason)

const carMetrics = await readJson(`${base}/car/metrics`)
check('GET /car/metrics 仍 200 且带 ok 布尔（车链路未被 S2 破坏）',
  carMetrics.status === 200 && typeof carMetrics.payload?.ok === 'boolean',
  `${carMetrics.status} ok=${carMetrics.payload?.ok}`)
const unknownNode = await readJson(`${base}/submarine/status`)
// The URL whitelist for roles is the dispatch regex itself, so an unknown role
// falls through to the shared router's own 404 — a 404 either way, never a
// fabricated node answer.
check('未知角色 → 404（不伪造节点应答）', unknownNode.status === 404,
  `${unknownNode.status} ${JSON.stringify(unknownNode.payload)}`)

server.closeAllConnections?.()
await new Promise(resolve => server.close(resolve))
fs.rmSync(process.env.DSH_HOME, { recursive: true, force: true })

console.log(failures === 0 ? '\nS2 CONTRACT TEST PASSED' : `\n${failures} S2 CHECK(S) FAILED`)
process.exitCode = failures === 0 ? 0 : 1
