#!/usr/bin/env node
/* scripts/test-sim-parity.mjs —— t1 ③ 长期守卫：**页面重规划 ↔ 引擎口径**对账 + 注入幂等 + 确定性
 *
 * 为什么需要这个脚本（t73「地图复杂就乱算+叠加+乱跑」的引擎一半）：
 *   页面「重新规划」此前调 `tools/plan_route.py` **不传 --engine** ⇒ CLI 默认 `tour`，而车端
 *   `config/robot.yaml` 配的是 `tracker-safe` ⇒ 同一张图页面和车算出**两条不同的路线**。
 *   修法：页面以车端配置为唯一真源（plannerEngineOf 读 `planner.engine`），planRoute 拼上 --engine。
 *
 * ★ 2026-10-18（第六张图口径）—— 本脚本改为**双态守卫**，因为现场已进入「手动路线」态：
 *   车端 `planner.auto_plan_enabled=false`（robot.yaml 注释：tracker-safe 在本图 K7 进洞形态下
 *   无解，legs0=0），实际发车路线 = 手动 `route.tokens`（68 token）。因此：
 *     · 现场图上页面按车端引擎重规划**必然失败** —— 断言它「显式失败、不静默给假路线」，
 *       而不是把"无解"当红；
 *     · 「页面 == 显式 CLI」的**成功路径**改在**引擎可解夹具**上钉
 *       （scripts/fixtures/map_fifth_solvable.yaml：历史第五张图几何，tracker-safe 44 token），
 *       证明 argv 拼装/引擎透传在真能解时逐 token 对齐，而不是只测失败路径；
 *     · 车端手动 route.tokens 单独做形状守卫（非空 / 全合法格名 / 起终点对齐）。
 *
 * 跑法（任何目录都行，路径按脚本自身位置推导）：
 *     node scripts/test-sim-parity.mjs
 *   退出码：0 = 全绿；1 = 断言失败；2 = 环境前置不满足（缺 python / 缺 workspace 配置）。
 *
 * 写域：**只读**。只读 config/*.yaml、tools/plan_route.py、src/simbench.js、scripts/fixtures/*；
 *   临时 YAML 落在 os.tmpdir() 并在退出前删除。不写 config、不跑仿真、不连车。
 */
import { spawn, spawnSync } from 'node:child_process'
import crypto from 'node:crypto'
import fs from 'node:fs'
import os from 'node:os'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const HERE = path.dirname(fileURLToPath(import.meta.url))
const PLUGIN = path.resolve(HERE, '..')                       // dsh-plugin-dzrobot-ssh
const SIMBENCH = path.join(PLUGIN, 'src', 'simbench.js')

const pass = { ok: 0, fail: 0, skip: 0 }
const check = (name, ok, note = '') => {
  if (ok) pass.ok += 1
  else pass.fail += 1
  console.log(`  ${ok ? 'PASS' : 'FAIL'} ${name}${note ? ' —— ' + note : ''}`)
}
/** 仅记录、不判失败：用于"本快照区分不出来"这类口径说明。 */
const info = (msg) => console.log(`  INFO ${msg}`)
const skip = (name, why) => { pass.skip += 1; console.log(`  SKIP ${name}${why ? ' —— ' + why : ''}`) }
const h = (t) => console.log(`\n\u001b[36m${t}\u001b[0m`)

/* ── 0. 环境前置 ─────────────────────────────────────────────────── */
if (!fs.existsSync(SIMBENCH)) {
  console.error(`✗ 找不到 ${SIMBENCH}（脚本必须放在 dsh-plugin-dzrobot-ssh/scripts/ 下）`)
  process.exit(2)
}
const sb = await import('file:///' + SIMBENCH.replace(/\\/g, '/'))
const {
  SIM_ROOT, fieldSpec, routeTokens, planRoute, plannerEngineOf, plannerFlagOf,
  applyFieldToMapText, injectDroneAvoidToMapText, normalizeCells,
} = sb
const { FIXTURE_MAP } = await import('file:///' + path.join(HERE, 'fixtures', 'solvable-map.mjs').replace(/\\/g, '/'))

const CFG = path.join(SIM_ROOT, 'config', 'robot.yaml')
const MAP = path.join(SIM_ROOT, 'config', 'map.yaml')
const CLI = path.join(SIM_ROOT, 'tools', 'plan_route.py')
const md5 = f => crypto.createHash('md5').update(fs.readFileSync(f)).digest('hex')

for (const [label, file] of [['robot.yaml', CFG], ['map.yaml', MAP], ['plan_route.py', CLI], ['夹具图', FIXTURE_MAP]]) {
  if (!fs.existsSync(file)) { console.error(`✗ 环境前置缺失：${label} → ${file}`); process.exit(2) }
}
const PY = process.env.PYTHON || 'python'
const pyProbe = spawnSync(PY, ['--version'], { encoding: 'utf8' })
if (pyProbe.status !== 0) {
  console.error(`✗ 环境前置缺失：跑不了 \`${PY}\`（${pyProbe.error?.message ?? pyProbe.stderr?.trim() ?? ''}）`)
  console.error('  本守卫要对账 plan_route.py 的真实输出，没有 python 只能算"未验证"。')
  process.exit(2)
}

console.log('SIM_ROOT       =', SIM_ROOT)
console.log('config 快照    = robot.yaml md5 %s | map.yaml md5 %s', md5(CFG), md5(MAP))
console.log('python        =', pyProbe.stdout.trim() || pyProbe.stderr.trim())

/* 车端引擎/自规划开关真源 = robot.yaml；页面白名单真源 = plan_route.py 的 argparse choices */
const engine = plannerEngineOf(CFG)
const autoPlan = plannerFlagOf(CFG, 'auto_plan_enabled')
const cliChoices = (() => {
  const text = fs.readFileSync(CLI, 'utf8')
  const hit = text.match(/add_argument\(\s*["']--engine["']\s*,\s*choices\s*=\s*\(([^)]*)\)/)
  return hit ? (hit[1].match(/["'][^"']+["']/g) ?? []).map(s => s.slice(1, -1)) : []
})()

/* ── 1. 引擎解析口径（页面从 robot.yaml 取什么就传什么）──────────────── */
h('1. plannerEngineOf：车端配置 → 页面要传的 --engine')
check('1.1 现役 robot.yaml 取到引擎且属 CLI 白名单',
  engine !== null && cliChoices.includes(engine),
  `engine=${engine} ｜ CLI choices=[${cliChoices.join(', ')}]`)
check('1.2 车端自规划开关可读（planner.auto_plan_enabled；缺键 ⇒ null 不猜）',
  autoPlan === true || autoPlan === false, String(autoPlan))

const tmpYaml = (name, text) => {
  const p = path.join(os.tmpdir(), `t1_parity_${name}.yaml`)
  fs.writeFileSync(p, text, 'utf8')
  return p
}
// 未知值必须**动态**取：车侧以后加了新引擎名，写死的假值会变成合法值
const UNKNOWN = `__not-an-engine-${Date.now()}__`
const probes = {
  quoted: 'planner:\n  engine: "tracker-safe"\nroute:\n  tokens: ["N10"]\n',
  inline: 'planner:\n  engine: tracker-safe   # 车上自规划\nroute:\n  tokens: ["N10"]\n',
  unknown: `planner:\n  engine: ${UNKNOWN}\nroute:\n  tokens: ["N10"]\n`,
  none: 'route:\n  tokens: ["N10"]\n',
  planOff: 'planner:\n  auto_plan_enabled: false   # 手动路线\nroute:\n  tokens: ["N10"]\n',
}
const created = []
for (const [k, text] of Object.entries(probes)) created.push(tmpYaml(k, text))
const [pQuoted, pInline, pUnknown, pNone, pPlanOff] = created
check('1.3 YAML 引号也认（旧实现误判未知 ⇒ 静默退回 tour）', plannerEngineOf(pQuoted) === 'tracker-safe')
check('1.4 行内注释不污染取值', plannerEngineOf(pInline) === 'tracker-safe')
check('1.5 白名单外的引擎值 ⇒ null（绝不给 argparse 塞未知值）', plannerEngineOf(pUnknown) === null, UNKNOWN)
check('1.6 没有 planner 块 ⇒ null（保持历史默认 tour，与改动前逐字节一致）', plannerEngineOf(pNone) === null)
check('1.7 auto_plan_enabled=false 原样读到 false（行内注释不污染）', plannerFlagOf(pPlanOff, 'auto_plan_enabled') === false)
for (const p of created) fs.rmSync(p, { force: true })

/* ── 2. 现场图：页面 online 路径重规划（双态）─────────────────────── */
h('2. planRoute()：现场图按车端引擎重规划（第六张图预期显式无解）')
const page = await planRoute({ configPath: CFG, mapPath: MAP })
const liveSolved = page.exitCode === 0 && page.tokens.length > 0
console.log('    exit=%d timedOut=%s engine=%s tokens=%d checkExit=%s',
  page.exitCode, page.timedOut, page.engine, page.tokens.length, page.check?.exit)
console.log('    token:', page.tokens.join(' '))
check('2.0 planRoute 记下的 engine == robot.yaml 的 planner.engine', page.engine === engine, `${page.engine} / ${engine}`)
if (liveSolved) {
  check('2.1 plan_route.py 退出码 0（页面自己没算，是调 CLI）', page.exitCode === 0 && !page.timedOut, page.stderrTail || '')
  check('2.3 route_check 通过（--check 退出 0）', page.check?.exit === 0)
  check('2.4 重规划出非空 token 列表', page.tokens.length > 0, `${page.tokens.length} token`)
} else {
  check('2.1 车端引擎对现场图 ⇒ **显式失败**（exit≠0 + 0 token，不是静默给假路线）',
    page.exitCode !== 0 && page.tokens.length === 0 && !page.timedOut,
    `exit=${page.exitCode} tokens=${page.tokens.length}`)
  check('2.2 失败原因可读（stderr 尾行含「无解 / 失败 / PlannerError」，不是空串）',
    /无解|失败|PlannerError/.test(page.stderrTail), JSON.stringify(page.stderrTail.slice(0, 120)))
  check('2.3 失败时没有 route_check 假绿（check=null 或 exit≠0）',
    !page.check || page.check.exit !== 0, JSON.stringify(page.check))
  check('2.4 失败时不产出 token（不会把半截路线当结果）', page.tokens.length === 0, `${page.tokens.length} token`)
}

/* ── 3. 车端路线（第六张图 = 手动 route.tokens）───────────────────── */
h('3. 车端 route.tokens（手动路线态的形状守卫）')
const car = routeTokens()
console.log('    车端 token:', car.join(' '))
const tokenCell = t => String(t).trim().replace(/^R:/i, '').replace(/[,:].*$/, '').replace(/\*$/, '').trim().toUpperCase()
const carCells = car.map(tokenCell)
const CELL_RE = /^[A-N](?:1[01]|[1-9])$/
const specLive = fieldSpec()
check('3.1 车端 route.tokens 非空（车端自规划关闭时，这条就是实发路线）', car.length > 0, `${car.length} token`)
check('3.2 车端 token 全部可解析为合法格名（允许 ,x/,y 后缀与 * / :PIVOT 修饰）',
  carCells.every(c => CELL_RE.test(c)), carCells.filter(c => !CELL_RE.test(c)).join(' ') || 'all ok')
check('3.3 车端路线首点 == map.start、末点 == map.end',
  carCells[0] === specLive.start && carCells[carCells.length - 1] === specLive.end,
  `${carCells[0]} → ${carCells[carCells.length - 1]}（start/end=${specLive.start}/${specLive.end}）`)
if (autoPlan === true && liveSolved) {
  let firstDiff = -1
  for (let i = 0; i < Math.max(car.length, page.tokens.length); i += 1) {
    if (car[i] !== page.tokens[i]) { firstDiff = i; break }
  }
  check('3.4 页面重规划 == 车端 route.tokens（逐 token，自规划开启态）', firstDiff < 0, `首个不同下标 ${firstDiff}`)
} else {
  info(`页面重规划 vs 车端手动路线不做逐 token 对账：auto_plan_enabled=${autoPlan}，liveSolved=${liveSolved}`
    + '（第六张图 tracker-safe 无解是车端已记录的现场态；成功路径对账见 4b 夹具）')
}

/* ── 4. 判别性证据 + 引擎可解夹具的逐 token 对账 ─────────────────── */
h('4. 判别性证据：同一份 CLI，传 / 不传 --engine')
const runCli = (extra, mapFile = MAP) => new Promise((resolve) => {
  const argv = ['tools/plan_route.py', '--config', CFG, '--map', mapFile, '--emit', 'json', '--check', '--quiet', ...extra]
  const child = spawn(PY, argv, {
    cwd: SIM_ROOT,
    env: { ...process.env, PYTHONPATH: path.join(SIM_ROOT, 'src', 'dzbrain'), PYTHONIOENCODING: 'utf-8', PYTHONUNBUFFERED: '1' },
    windowsHide: true,
  })
  let out = ''; let err = ''
  child.stdout.on('data', (c) => { out += c })
  child.stderr.on('data', (c) => { err += c })
  child.on('error', e => resolve({ code: -1, tokens: [], err: String(e?.message ?? e) }))
  child.on('close', code => {
    let tokens = []
    try { tokens = JSON.parse(out.slice(out.indexOf('{'))).plans?.[0]?.tokens ?? [] } catch { /* 解析失败按空算 */ }
    resolve({ code: code ?? 1, tokens, err })
  })
})

const explicit = engine ? await runCli(['--engine', engine]) : null
const legacy = await runCli([])
if (explicit) {
  if (liveSolved) {
    check('4.1 显式 --engine <车端引擎> 的 CLI 输出 == planRoute 输出（argv 拼装没走样）',
      explicit.code === 0 && explicit.tokens.join(' ') === page.tokens.join(' '),
      `${explicit.tokens.length} vs ${page.tokens.length}`)
  } else {
    check('4.1 现场图无解时，页面与显式 CLI **失败口径一致**（同 exit≠0、同 0 token）',
      explicit.code !== 0 && explicit.tokens.length === 0 && page.tokens.length === 0,
      `CLI exit=${explicit.code} tokens=${explicit.tokens.length} ｜ 页面 exit=${page.exitCode} tokens=${page.tokens.length}`)
  }
} else {
  skip('4.1 显式 --engine 对照', 'robot.yaml 无 planner.engine（页面也不传，两边都是 CLI 默认）')
}
console.log('    不带 --engine（= 改动前的页面行为）: %d token，exit=%d', legacy.tokens.length, legacy.code)
if (legacy.tokens.length === 0) {
  skip('4.2 反证', 'CLI 不带 --engine 没跑出结果')
} else if (legacy.tokens.join(' ') === car.join(' ')) {
  info('本快照两引擎同解（不带 --engine 也与车端相同）⇒ 反证不具判别性，但正面证据已成立；不算失败')
} else {
  let d = -1
  for (let i = 0; i < Math.max(car.length, legacy.tokens.length); i += 1) {
    if (car[i] !== legacy.tokens[i]) { d = i; break }
  }
  check('4.2 反证：不带 --engine ⇒ 与车端手动路线不同（旧行为确实是另一条路线）', d >= 0,
    `首个不同下标 ${d}；${legacy.tokens.length} vs ${car.length} token`)
}

/* ── 4b. 引擎可解夹具：页面 == 显式 CLI（逐 token 成功路径）─────────────
 *   为什么必须另立夹具：现场图 tracker-safe 无解（车端已记录），如果只测失败路径，
 *   「argv 拼装把 --engine/--map 传错」这类退化会因为两边都失败而假绿。夹具是历史第五张图
 *   几何（barrier/start/end 与现场相同，点位不同），tracker-safe 能解 ⇒ 成功路径有牙。 */
h('4b. 引擎可解夹具：页面 planRoute == 显式 CLI（逐 token 成功路径）')
const fixPage = await planRoute({ configPath: CFG, mapPath: FIXTURE_MAP })
const fixExplicit = engine ? await runCli(['--engine', engine], FIXTURE_MAP) : null
console.log('    夹具：页面 exit=%d tokens=%d checkExit=%s ｜ 显式 CLI exit=%d tokens=%d',
  fixPage.exitCode, fixPage.tokens.length, fixPage.check?.exit, fixExplicit?.code, fixExplicit?.tokens.length)
check('4b.1 夹具上页面重规划 exit 0 + route_check 通过 + 非空 token（引擎拼装真能跑通）',
  fixPage.exitCode === 0 && fixPage.check?.exit === 0 && fixPage.tokens.length > 0,
  fixPage.stderrTail || `${fixPage.tokens.length} token`)
check('4b.2 夹具上页面产物 == 显式 CLI 产物（逐 token，argv/引擎真没走样）',
  Boolean(fixExplicit) && fixExplicit.code === 0 && fixExplicit.tokens.join(' ') === fixPage.tokens.join(' '),
  fixExplicit ? `${fixExplicit.tokens.length} vs ${fixPage.tokens.length}` : '无显式 CLI 对照')
check('4b.3 夹具产物 != 车端手动路线（证明确实跑了重规划，不是直接读 route.tokens）',
  fixPage.tokens.length > 0 && fixPage.tokens.join(' ') !== car.join(' '),
  `${fixPage.tokens.length} vs ${car.length}`)

/* ── 5. 点位/无人机注入：连着做两次不叠加 ─────────────────────────── */
h('5. 服务端注入幂等（"叠加"的服务器一半）')
const mapText = fs.readFileSync(MAP, 'utf8')
const field = { mines: ['A1', 'B2'], obstacles: ['C3'], bridges: ['K5'], targets: ['B11', 'C11'] }
const once = applyFieldToMapText(mapText, field)
const twice = applyFieldToMapText(once, field)
const lineOf = (text, key) => (text.match(new RegExp(`^\\s*${key}:\\s*\\[[^\\]]*\\]`, 'm')) ?? [''])[0]
const cellsIn = (text, key) => normalizeCells(String(lineOf(text, key).replace(/^[^[]*\[/, '').replace(/\]$/, '')).replace(/["']/g, ''))
check('5.1 同一份点位连做两次 ⇒ 逐字节相同（格不翻倍）', once === twice)
check('5.2 注入后 mines 行就是 2 格（不是 4 格）', cellsIn(once, 'mines').length === 2, cellsIn(once, 'mines').join(' '))
const baseObs = cellsIn(once, 'obstacles').length
const drone1 = injectDroneAvoidToMapText(once, ['N8', 'M2'])
const drone2 = injectDroneAvoidToMapText(drone1, ['N8', 'M2'])
check('5.3 无人机起降格注两次 ⇒ 逐字节相同（幂等）', drone1 === drone2)
check('5.4 恰 +2 格（基线+2，不是 +4）',
  cellsIn(drone1, 'obstacles').length === baseObs + 2 && cellsIn(drone2, 'obstacles').length === baseObs + 2,
  `${baseObs} → ${cellsIn(drone1, 'obstacles').length} → ${cellsIn(drone2, 'obstacles').length}`)
const injected = cellsIn(drone1, 'obstacles')
check('5.5 obstacles 行里 N8 / M2 各出现恰 1 次',
  injected.filter(c => c === 'N8').length === 1 && injected.filter(c => c === 'M2').length === 1)

/* ── 6. /sim/map 快照无状态残留 + 真源计数 ────────────────────────── */
h('6. fieldSpec() 无状态残留（连读三次不变）+ 真源要素计数')
const specs = [fieldSpec(), fieldSpec(), fieldSpec()]
const dump = s => JSON.stringify({ mines: s.mines, obstacles: s.obstacles, targets: s.targets, bridges: s.bridges, target: s.target, drone: s.drone, planner: s.planner })
check('6.1 三次读数逐字节一致（TARGET_CACHE 不会越读越叠）',
  dump(specs[0]) === dump(specs[1]) && dump(specs[1]) === dump(specs[2]))
check('6.2 场地读得动（ok:true，不是错误对象）', specs[0].ok === true, specs[0].error ?? '')

/* 与 YAML 原文独立对账：字段计数随真源走（换图不假红） */
const yamlLists = {}
for (const key of ['mines', 'obstacles', 'targets', 'bridges']) {
  yamlLists[key] = normalizeCells(String(lineOf(mapText, key).replace(/^[^[]*\[/, '').replace(/\]$/, '')).replace(/["']/g, ''))
}
check('6.3 fieldSpec 四类要素计数 == map.yaml 原文计数',
  ['mines', 'obstacles', 'targets', 'bridges'].every(k => specs[0][k].length === yamlLists[k].length),
  `spec ${specs[0].mines.length}/${specs[0].obstacles.length}/${specs[0].bridges.length}/${specs[0].targets.length}`
  + ` vs yaml ${yamlLists.mines.length}/${yamlLists.obstacles.length}/${yamlLists.bridges.length}/${yamlLists.targets.length}`)
const SIXTH_MAP = '2d2f658daceedc7fb04e080fa5458918'
const OFFICIAL_MAP = 'c9bdb660637430a9dd00f518301090bc'
if (md5(MAP) === SIXTH_MAP) {
  check('6.4 第六张图：雷8 / 障8 / 通2 / 靶2（竖向 D5+D6）+ 起 N10 终 N5 + 隔板 D/6/7',
    specs[0].mines.length === 8 && specs[0].obstacles.length === 8
    && specs[0].bridges.length === 2 && specs[0].targets.length === 2
    && specs[0].target.yaw === 'v'
    && specs[0].start === 'N10' && specs[0].end === 'N5'
    && specs[0].barrier.colFrom === 'D' && specs[0].barrier.rowHi === 6 && specs[0].barrier.rowLo === 7,
    `${specs[0].mines.length}/${specs[0].obstacles.length}/${specs[0].bridges.length}/${specs[0].targets.length} yaw=${specs[0].target.yaw}`)
  check('6.5 无人机起降格取注释真源（N8 / M2，source=map-comment；不是内建兜底）',
    specs[0].drone.takeoff === 'N8' && specs[0].drone.land === 'M2' && specs[0].drone.source === 'map-comment',
    JSON.stringify(specs[0].drone))
} else if (md5(MAP) === OFFICIAL_MAP) {
  check('6.4 官方第五张图：雷 8 / 障 8 / 通 2 / 靶 2',
    specs[0].mines.length === 8 && specs[0].obstacles.length === 8
    && specs[0].bridges.length === 2 && specs[0].targets.length === 2,
    `${specs[0].mines.length}/${specs[0].obstacles.length}/${specs[0].bridges.length}/${specs[0].targets.length}`)
} else {
  info(`map.yaml md5=${md5(MAP).slice(0, 8)}… 不是已知快照 ⇒ 只做"计数 == 原文"对账（6.3）`)
}

/* ── 7. 路线冗余形状（"重复绘制"的服务端迹象）────────────────────── */
h('7. 车端路线无「整段复制 / 零位移」式重复')
const consecutiveDup = car.some((t, i) => i > 0 && t === car[i - 1])
const seenPair = new Set()
let repeatedPair = ''
for (let i = 1; i < car.length; i += 1) {
  const key = car[i - 1] + '>' + car[i]
  if (seenPair.has(key) && !repeatedPair) repeatedPair = key
  seenPair.add(key)
}
check('7.1 无相邻重复 token（A→A 零位移步）', !consecutiveDup, `${car.length} token`)
check('7.2 无重复相邻对（同一 A→B 段不会出现两次 ⇒ 不是整段复制）',
  !repeatedPair, repeatedPair ? `重复段 ${repeatedPair}` : `${seenPair.size} 个相邻对全唯一`)
if (car.length > 0) {
  const uniqueCells = new Set(carCells).size
  check('7.3 回访比例合理（手动 U 形路线允许回访走廊；unique/total ≥ 0.6，防整图乱复制）',
    uniqueCells / car.length >= 0.6, `${uniqueCells}/${car.length} = ${(uniqueCells / car.length).toFixed(2)}`)
}

/* ── 8. 重规划确定性（"乱跑/乱算"的排除项）────────────────────────── */
h('8. 重规划确定性：同输入同输出')
const page2 = await planRoute({ configPath: CFG, mapPath: MAP })
check('8.1 两次现场图重规划逐 token 相同（稳定；本图两次都显式失败也算稳定）',
  page2.tokens.join(' ') === page.tokens.join(' '), `${page.tokens.length} / ${page2.tokens.length}`)
check('8.2 两次的成败口径一致（不会一次成功一次失败）',
  (page2.exitCode === 0) === (page.exitCode === 0), `exit ${page.exitCode} / ${page2.exitCode}`)
check('8.3 台账取证字段 engine 与 robot.yaml 一致', page2.engine === engine, String(page2.engine))
const fixPage2 = await planRoute({ configPath: CFG, mapPath: FIXTURE_MAP })
check('8.4 夹具重规划确定性：两次逐 token 相同且都含合法首末点',
  fixPage2.tokens.join(' ') === fixPage.tokens.join(' ')
  && fixPage2.tokens[0] === 'N10' && /^N5/.test(String(fixPage2.tokens[fixPage2.tokens.length - 1])),
  `${fixPage.tokens.length} / ${fixPage2.tokens.length}`)

/* ── 汇总 ───────────────────────────────────────────────────────── */
const total = pass.ok + pass.fail
console.log('')
console.log(`结果：${pass.ok}/${total} 通过${pass.skip ? `，${pass.skip} 跳过` : ''}`
  + `${pass.fail === 0 ? ' —— 全部通过 ✅' : `，${pass.fail} 失败 ❌`}`)
process.exit(pass.fail === 0 ? 0 : 1)
