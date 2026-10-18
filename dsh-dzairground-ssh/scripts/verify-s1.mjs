/**
 * S1 验收取证：新包（dzairground-ssh）与旧车插件（dzrobot-ssh）的**同构**证明。
 *
 * 为什么单独写这个脚本，而不是只跑 scripts/test-host.mjs：
 *   test-host.mjs 证明「新包的 Host 半身自己跑通了」，但它**只测新包**。
 *   S1 的验收判据是「`/car/status` 与旧插件 `/status` **同构**」——那需要**同时**
 *   把两个插件都挂起来，用同一个 loopback 服，逐键比对。这是本脚本唯一的存在理由。
 *
 * 硬约束（踩了会假绿）：
 *   · 两个插件的 DSH_HOME 都指向**同一个临时目录**：这样数据目录隔离才有意义，
 *     也保证绝不碰用户真实的 ~/.dsh/dzrobot-ssh。
 *   · 旧插件从**冻结目录只读 import**，本脚本不写它一个字节。
 *   · sim.html 必须**真 fetch**：它的 6 个 test-sim-*.mjs 不引用 client.js、
 *     DOM 桩 fetch 直接返回空对象，前缀改错时它们照样全绿。
 *
 * 跑：node scripts/verify-s1.mjs
 */

import http from 'node:http'
import fs from 'node:fs'
import os from 'node:os'
import path from 'node:path'
import { fileURLToPath, pathToFileURL } from 'node:url'

const HERE = path.dirname(fileURLToPath(import.meta.url))
const PLUGIN = path.resolve(HERE, '..')
const OLD_PLUGIN = path.resolve(
  'D:/User/Desktop/比赛存档/v1/dzrobot_v2/dsh-plugin-dzrobot-ssh',
)

let failures = 0
const check = (label, ok, detail) => {
  console.log(`${ok ? 'OK  ' : 'FAIL'}  ${label}${detail === undefined ? '' : `  ${detail}`}`)
  if (!ok) failures += 1
}

// ── 0. 真实数据目录快照（V7 的前后对照）─────────────────────────────────────
const REAL_DSH = path.join(os.homedir(), '.dsh')
const REAL_OLD_DATA = path.join(REAL_DSH, 'dzrobot-ssh')
const snapshotDir = dir => {
  if (!fs.existsSync(dir)) return { exists: false, entries: [] }
  const entries = []
  const walk = current => {
    for (const name of fs.readdirSync(current).sort()) {
      const full = path.join(current, name)
      const stat = fs.statSync(full)
      if (stat.isDirectory()) walk(full)
      else entries.push(`${path.relative(dir, full)}|${stat.size}|${Math.round(stat.mtimeMs)}`)
    }
  }
  walk(dir)
  return { exists: true, entries }
}
const realOldBefore = snapshotDir(REAL_OLD_DATA)
const realNewDataExistedBefore = fs.existsSync(path.join(REAL_DSH, 'dzairground-ssh'))

// 两个插件共用这一个临时 DSH_HOME ⇒ 数据目录隔离才可观测。
process.env.DSH_HOME = fs.mkdtempSync(path.join(os.tmpdir(), 'dzairground-s1-'))

/** 把某个插件的 apply() 挂到一个 loopback server 上，返回 { base, routes, upgrades, logs, dispose }。 */
async function boot(entryPath, label) {
  const routes = []
  const upgrades = []
  const logs = []
  const disposers = []
  // Windows 上绝对路径不能直接 import，必须先转成 file:// URL；
  // 查询串只用来绕开 ESM 的按 URL 缓存（两个插件各加载一次）。
  const entryUrl = `${pathToFileURL(path.join(entryPath, 'index.js')).href}?boot=${label}`
  const mod = await import(entryUrl)
  const ctx = {
    logger: {
      info: message => logs.push(`info ${message}`),
      warn: message => logs.push(`warn ${message}`),
      error: message => logs.push(`error ${message}`),
    },
    get: () => undefined,
    effect: (callback) => {
      const disposer = callback()
      const wrapped = typeof disposer === 'function' ? disposer : () => {}
      disposers.push(wrapped)
      return wrapped
    },
    inject: (deps, callback) => {
      callback({
        webServer: {
          register: route => { routes.push(route); return () => {} },
          registerUpgrade: route => { upgrades.push(route); return () => {} },
        },
        effect: ctx.effect,
      })
    },
  }
  mod.apply(ctx, {})
  const server = http.createServer((req, res) => {
    const route = routes.find(candidate => req.url.startsWith(candidate.path))
    if (route === undefined) { res.writeHead(404).end('no route'); return }
    void route.handler(req, res)
  })
  server.on('upgrade', (req, socket, head) => {
    const route = upgrades.find(candidate => candidate.path === new URL(req.url, 'http://x').pathname)
    if (route === undefined) { socket.destroy(); return }
    void route.handler(req, socket, head)
  })
  await new Promise(resolve => server.listen(0, '127.0.0.1', resolve))
  const port = server.address().port
  return {
    label, mod, port, routes, upgrades, logs,
    base: `http://127.0.0.1:${port}${routes[0].path}`,
    origin: `http://127.0.0.1:${port}`,
    close: async () => {
      server.closeAllConnections?.()
      await new Promise(resolve => server.close(resolve))
      for (const dispose of disposers.reverse()) { try { dispose() } catch { /* best effort */ } }
    },
  }
}

async function readJson(url, options) {
  const response = await fetch(url, options)
  const text = await response.text()
  let payload
  try { payload = JSON.parse(text) } catch { payload = { __unparsable: text.slice(0, 200) } }
  return { status: response.status, payload, text, headers: response.headers }
}

const keySet = object => (object !== null && typeof object === 'object' && !Array.isArray(object)
  ? Object.keys(object).sort()
  : [`<not-an-object:${Array.isArray(object) ? 'array' : typeof object}>`])
const sameKeys = (a, b) => a.length === b.length && a.every((key, index) => key === b[index])

// ── 1. 两个插件同时挂起来 ───────────────────────────────────────────────────
const newHost = await boot(PLUGIN, 'new')
const oldHost = await boot(OLD_PLUGIN, 'old')

console.log('\n=== 注册面 ===')
console.log(`new  name=${newHost.mod.name}  routes=${newHost.routes.map(r => `${r.kind}:${r.path}`).join(',')}  upgrade=${newHost.upgrades.map(r => r.path).join(',')}`)
console.log(`old  name=${oldHost.mod.name}  routes=${oldHost.routes.map(r => `${r.kind}:${r.path}`).join(',')}  upgrade=${oldHost.upgrades.map(r => r.path).join(',')}`)

check('新包 name 是 dzairground-ssh', newHost.mod.name === 'dzairground-ssh', newHost.mod.name)
check('旧包 name 仍是 dzrobot-ssh（冻结目录没被改）', oldHost.mod.name === 'dzrobot-ssh', oldHost.mod.name)
check('新包路由前缀是 /api/dzairground-ssh',
  newHost.routes.length === 1 && newHost.routes[0].path === '/api/dzairground-ssh',
  newHost.routes.map(r => r.path).join(','))
check('新包 VNC upgrade 座位是 /api/dzairground-ssh/vnc',
  newHost.upgrades.length === 1 && newHost.upgrades[0].path === '/api/dzairground-ssh/vnc',
  newHost.upgrades.map(r => r.path).join(','))

// ── 2. V4：/car/status 与旧 /status 同构 ────────────────────────────────────
console.log('\n=== V4 同构（键集合比对，忽略新增 role）===')
const oldStatus = await readJson(`${oldHost.base}/status`)
const newStatus = await readJson(`${newHost.base}/status`)
const newCarStatus = await readJson(`${newHost.base}/car/status`)

console.log(`old      /status        ${oldStatus.status}  keys=${keySet(oldStatus.payload).length}`)
console.log(`new      /status        ${newStatus.status}  keys=${keySet(newStatus.payload).length}`)
console.log(`new      /car/status    ${newCarStatus.status}  keys=${keySet(newCarStatus.payload).length}`)

check('旧 /status 200', oldStatus.status === 200, String(oldStatus.status))
check('新 /status 200', newStatus.status === 200, String(newStatus.status))
check('新 /car/status 200', newCarStatus.status === 200, String(newCarStatus.status))

const oldKeys = keySet(oldStatus.payload)
const newKeys = keySet(newStatus.payload)
const carKeys = keySet(newCarStatus.payload).filter(key => key !== 'role')

console.log(`keys old      : ${oldKeys.join(' ')}`)
console.log(`keys new      : ${newKeys.join(' ')}`)
console.log(`keys /car     : ${keySet(newCarStatus.payload).join(' ')}`)

// S2 的唯一键改名：单节点时代的描述键 `car` 按统一节点模型（t4 §1.2 NodeSnapshot）
// 改成角色中立的 `desc`——`/drone/status` 不能把无人机的自述放在一个叫 car 的键里。
// 比对时把旧键集合里的 `car` 映射为 `desc` 再比，其余键必须一字不差。
const withRename = keys => keys.map(key => (key === 'car' ? 'desc' : key)).sort()

console.log(`keys old (car→desc): ${withRename(oldKeys).join(' ')}`)
check('新 /status 键集合 == 旧 /status 键集合（S2 唯一改名 car→desc）',
  sameKeys(withRename(oldKeys), newKeys),
  `old=${oldKeys.length} new=${newKeys.length}`)
check('新 /car/status 键集合（去掉 role）== 旧 /status 键集合（同上改名）',
  sameKeys(withRename(oldKeys), carKeys),
  `old=${oldKeys.length} car=${carKeys.length}`)
check('/car/status 带 role="car"', newCarStatus.payload.role === 'car', JSON.stringify(newCarStatus.payload.role))
check('/status（无节点前缀）不带 role', newStatus.payload.role === undefined, JSON.stringify(newStatus.payload.role))

// 子对象也要同构：settings / vnc / files / local / reload 都是对象或标量，
// 只比顶层键会漏掉「顶层对了但里面少一个字段」的情况。
const deepSame = (a, b) => {
  const ka = Object.keys(a ?? {}).sort()
  const kb = Object.keys(b ?? {}).sort()
  return sameKeys(ka, kb)
}
for (const field of ['settings', 'vnc', 'files', 'local', 'reload']) {
  check(`嵌套 ${field} 键集合一致（旧 /status vs 新 /car/status）`,
    deepSame(oldStatus.payload[field], newCarStatus.payload[field]),
    `old=${keySet(oldStatus.payload[field]).join(',')} || new=${keySet(newCarStatus.payload[field]).join(',')}`)
}

// ── 3. 路由剥离：/car/<rest> 必须打到原来的 <rest> ─────────────────────────
console.log('\n=== 路由剥离 /car/<rest> ===')
for (const suffix of ['/config', '/diag', '/graph']) {
  const oldOne = await readJson(`${oldHost.base}${suffix}`)
  const newOne = await readJson(`${newHost.base}${suffix}`)
  const newScoped = await readJson(`${newHost.base}/car${suffix}`)
  check(`${suffix}: 旧/新/scoped 三者同状态码，且 scoped 键集合 == 旧`,
    oldOne.status === newOne.status && newOne.status === newScoped.status
      && sameKeys(keySet(oldOne.payload), keySet(newScoped.payload).filter(key => key !== 'role')),
    `${oldOne.status}/${newOne.status}/${newScoped.status}`)
}

// ── 4. drone 节点（S2 已接入：同构 + 诚实离线）───────────────────────────────
// S1 时这里断言 404 node_not_ready；S2 的语义变化就是「drone 节点存在」，所以
// 本段改为：同构键集合 + 机侧默认值 + 不冒充活探针（bug#1 的 source 标记）。
console.log('\n=== drone 节点（S2 已接入）===')
const droneStatus = await readJson(`${newHost.base}/drone/status`)
check('/drone/status 是 200', droneStatus.status === 200, String(droneStatus.status))
check('/drone/status 带 role="drone"', droneStatus.payload.role === 'drone', JSON.stringify(droneStatus.payload.role))
const droneKeys = keySet(droneStatus.payload).filter(key => key !== 'role')
console.log(`keys /drone   : ${keySet(droneStatus.payload).join(' ')}`)
check('/drone/status 键集合（去掉 role）== /car/status 键集合（去掉 role）',
  sameKeys(carKeys, droneKeys), `car=${carKeys.length} drone=${droneKeys.length}`)
// 机侧默认值 = 冻结 Host 配置（~/.dsh/dzdrone-ssh/settings.json 的只读快照值）；
// dzsmart.local 是主机名不是 IP —— 地址是设置项，不许硬编码。
check('机侧默认 host=dzsmart.local（来自冻结配置，未硬编码 IP）',
  droneStatus.payload.settings?.host === 'dzsmart.local', JSON.stringify(droneStatus.payload.settings?.host))
check('机侧默认 user=duzhong / filesRoot=/home/duzhong / vncPort=5901',
  droneStatus.payload.settings?.user === 'duzhong'
  && droneStatus.payload.settings?.filesRoot === '/home/duzhong'
  && droneStatus.payload.settings?.vncPort === 5901,
  JSON.stringify({ user: droneStatus.payload.settings?.user, root: droneStatus.payload.settings?.filesRoot, vnc: droneStatus.payload.settings?.vncPort }))
check('/drone/status 的 probe 带 source="cached"（缓存探针绝不冒充活探针）',
  droneStatus.payload.probe === undefined || droneStatus.payload.probe.source === 'cached',
  JSON.stringify(droneStatus.payload.probe))
check('/drone/status 没有描述时如实缺席（undefined/null，不造 desc）',
  droneStatus.payload.desc === undefined || droneStatus.payload.desc === null,
  JSON.stringify(droneStatus.payload.desc))

// /drone/metrics：200 + ok 布尔 + 缺席可解释（无人机不可达是答案，不是故障）。
const droneMetrics = await readJson(`${newHost.base}/drone/metrics`)
check('/drone/metrics 200 且带 ok 布尔',
  droneMetrics.status === 200 && typeof droneMetrics.payload?.ok === 'boolean',
  `${droneMetrics.status} ok=${droneMetrics.payload?.ok}`)

// ── 5. V5：sim.html —— 真 fetch，不信测试全绿 ──────────────────────────────
console.log('\n=== V5 sim.html 前缀（真 fetch）===')
const simResponse = await fetch(`${newHost.base}/sim.html`)
const simBody = await simResponse.text()
const simApiLine = /var API = '([^']*)'/.exec(simBody)
console.log(`GET /api/dzairground-ssh/sim.html -> ${simResponse.status} ${simResponse.headers.get('content-type')} ${simBody.length}B`)
console.log(`  正文里的 API 常量: var API = '${simApiLine?.[1]}'`)

check('GET /api/dzairground-ssh/sim.html 是 200', simResponse.status === 200, String(simResponse.status))
check('sim.html 正文含 var API = \'/api/dzairground-ssh\'',
  simApiLine !== null && simApiLine[1] === '/api/dzairground-ssh', JSON.stringify(simApiLine?.[1]))
check('sim.html 正文**不再含** dzrobot-ssh', !simBody.includes('dzrobot-ssh'),
  `命中次数=${(simBody.match(/dzrobot-ssh/g) ?? []).length}`)

// 旧插件的 sim.html 仍然服务自己 —— 证明两个前缀各走各的，没串台。
const oldSim = await fetch(`${oldHost.base}/sim.html`)
const oldSimBody = await oldSim.text()
check('旧插件 /api/dzrobot-ssh/sim.html 仍 200（两个前缀并存不冲突）', oldSim.status === 200, String(oldSim.status))
check('旧 sim.html 的 API 常量仍是 /api/dzrobot-ssh',
  /var API = '\/api\/dzrobot-ssh'/.test(oldSimBody))

// ── 6. V6：files 白名单 —— 双向（声明了的在 + 该有的都声明了）───────────────
console.log('\n=== V6 package.json files 白名单 ===')
const pkg = JSON.parse(fs.readFileSync(path.join(PLUGIN, 'package.json'), 'utf8'))
console.log(`name=${pkg.name} version=${pkg.version}`)
console.log(`files=${JSON.stringify(pkg.files)}`)
check('包名是 dsh-dzairground-ssh', pkg.name === 'dsh-dzairground-ssh', pkg.name)
check('版本是 0.6.0-s2', pkg.version === '0.6.0-s2', pkg.version)

const REQUIRED = ['index.js', 'client.js', 'src', 'cordis.patch.yml', 'vnc-app.html', 'vnc-connect.html', 'float.html', 'sim.html', 'sim-root.json', 'novnc']
for (const entry of REQUIRED) {
  check(`files 声明了 ${entry}`, pkg.files.includes(entry), undefined)
  check(`${entry} 在磁盘上真实存在`, fs.existsSync(path.join(PLUGIN, entry)), undefined)
}

// 「该有的都声明了」：反查 —— Host 半身 require/import 到的、以及 index.js 里
// fileURLToPath 指向的运行期文件，必须全部落在 files 白名单覆盖范围内。
const covered = entry => pkg.files.some(declared => declared === entry || entry.startsWith(`${declared}/`))
const runtimeFiles = [
  'index.js', 'client.js', 'cordis.patch.yml',
  'vnc-app.html', 'vnc-connect.html', 'float.html', 'sim.html', 'sim-root.json',
]
const topLevelFiles = fs.readdirSync(PLUGIN, { withFileTypes: true })
  .filter(entry => entry.isFile())
  .map(entry => entry.name)
  .filter(name => !name.startsWith('.'))
  .sort()
console.log(`顶层文件: ${topLevelFiles.join(' ')}`)
const uncoveredTopLevel = topLevelFiles.filter(name => runtimeFiles.includes(name) ? !covered(name) : !covered(name))
console.log(`顶层文件未被 files 覆盖的: ${uncoveredTopLevel.length === 0 ? '(无)' : uncoveredTopLevel.join(' ')}`)

// 已知且有意不打包的顶层文件（测试/文档/脚本不进安装副本）。
const NOT_SHIPPED = new Set(['package.json', 'README.md', 'scripts'])
const unexplained = uncoveredTopLevel.filter(name => !NOT_SHIPPED.has(name))
check('没有「未声明也未归类」的顶层运行文件', unexplained.length === 0, unexplained.join(',') || '(无)')

// src/ 下每个模块都必须被白名单里的 `src` 目录覆盖。
const srcModules = fs.readdirSync(path.join(PLUGIN, 'src')).filter(name => name.endsWith('.js'))
const uncoveredSrc = srcModules.filter(name => !covered(`src/${name}`))
check(`src/ 下 ${srcModules.length} 个模块全部被 files 覆盖`, uncoveredSrc.length === 0, uncoveredSrc.join(',') || '(全部覆盖)')

// simbench.js 的运行期依赖：sim-root.json 必须在白名单里（约束 3）。
check('sim-root.json 在白名单里（simbench 指针文件）', pkg.files.includes('sim-root.json'), undefined)

// ── 7. V7：数据目录隔离 ───────────────────────────────────────────────────
// 只比对「目录被创建了」是不够的 —— 两个 store 都是懒写的，目录建好但一个文件
// 都不落盘时，上面那条断言照样通过。必须**真的写一次**，再证明它落在新目录、
// 且**没有**落到旧目录。
console.log('\n=== V7 数据目录隔离 ===')
const tempHome = process.env.DSH_HOME
const newData = path.join(tempHome, 'dzairground-ssh')
const oldData = path.join(tempHome, 'dzrobot-ssh')
console.log(`临时 DSH_HOME = ${tempHome}`)

// 用一个可辨认的值写入新插件的 settings，然后看它落在哪。
const MARKER_PORT = 2299
const written = await readJson(`${newHost.base}/car/config`, {
  method: 'POST',
  headers: { 'content-type': 'application/json' },
  body: JSON.stringify({ port: MARKER_PORT }),
})
check('POST /car/config 写入成功', written.status === 200 && written.payload.settings?.port === MARKER_PORT,
  `${written.status} port=${written.payload.settings?.port}`)
check('写入的响应带 role="car"', written.payload.role === 'car', JSON.stringify(written.payload.role))

// JsonStore.flush() 是 120 ms 防抖（src/store.js:154-164，合并突发写入用的），
// 所以写完立刻读盘必然读不到。这里**等过防抖窗口**再断言 —— 直接读会得到一个
// 假的「没落盘」，那是本脚本的时序错误，不是产品的 bug。
await new Promise(resolve => setTimeout(resolve, 400))

const newSettingsFile = path.join(newData, 'settings.json')
const oldSettingsFile = path.join(oldData, 'settings.json')
console.log(`  <DSH_HOME>/dzairground-ssh/settings.json 存在: ${fs.existsSync(newSettingsFile)}`)
console.log(`  <DSH_HOME>/dzrobot-ssh/settings.json      存在: ${fs.existsSync(oldSettingsFile)}`)
check('新插件把 settings.json 写进了 <DSH_HOME>/dzairground-ssh/', fs.existsSync(newSettingsFile), newSettingsFile)
check('新插件**没有**写 <DSH_HOME>/dzrobot-ssh/settings.json', !fs.existsSync(oldSettingsFile), oldSettingsFile)

const landed = fs.existsSync(newSettingsFile)
  ? JSON.parse(fs.readFileSync(newSettingsFile, 'utf8'))
  : {}
console.log(`  落盘的 port = ${landed.port}`)
check('落盘的值就是刚才 POST 的那个（证明读写的确实是这个目录）',
  landed.port === MARKER_PORT, JSON.stringify(landed.port))

const newDataFiles = fs.existsSync(newData) ? fs.readdirSync(newData).sort() : []
const oldDataFiles = fs.existsSync(oldData) ? fs.readdirSync(oldData).sort() : []
console.log(`  新目录内容: ${newDataFiles.join(' ') || '(空)'}`)
console.log(`  旧目录内容: ${oldDataFiles.join(' ') || '(空)'}`)
check('新插件目录里没有任何 dzrobot-ssh 命名的文件',
  !newDataFiles.some(name => name.includes('dzrobot')), newDataFiles.join(',') || '(空)')

// 真实 home 里的 dzairground-ssh 目录**必须不存在** —— 本次验证全程只碰临时
// DSH_HOME，若这里出现目录，说明 DSH_HOME 没有生效（那 V7 的结论就全是假的）。
const realNewData = path.join(REAL_DSH, 'dzairground-ssh')
const realNewExistedBefore = fs.existsSync(realNewData)
check('真实 ~/.dsh/dzairground-ssh/ 未被本次验证创建（DSH_HOME 确实生效）',
  realNewExistedBefore === realNewDataExistedBefore, `运行前=${realNewDataExistedBefore} 运行后=${fs.existsSync(realNewData)}`)

const realOldAfter = snapshotDir(REAL_OLD_DATA)
console.log(`  真实 ~/.dsh/dzrobot-ssh 条目数 before=${realOldBefore.entries.length} after=${realOldAfter.entries.length}`)
check('真实 ~/.dsh/dzrobot-ssh 未被本次验证触碰（mtime+size 全等）',
  realOldBefore.exists === realOldAfter.exists
  && realOldBefore.entries.length === realOldAfter.entries.length
  && realOldBefore.entries.every((line, index) => line === realOldAfter.entries[index]),
  `${realOldBefore.entries.length} vs ${realOldAfter.entries.length}`)

// ── 8. 收尾 ────────────────────────────────────────────────────────────────
await newHost.close()
await oldHost.close()
fs.rmSync(tempHome, { recursive: true, force: true })

console.log('\n=== 日志（新包）===')
for (const line of newHost.logs.slice(0, 8)) console.log(`  ${line}`)

console.log(failures === 0 ? '\nS1 VERIFY PASSED' : `\n${failures} S1 CHECK(S) FAILED`)
process.exitCode = failures === 0 ? 0 : 1
