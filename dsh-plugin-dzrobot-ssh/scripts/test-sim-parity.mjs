#!/usr/bin/env node
/* scripts/test-sim-parity.mjs —— t1 ③ 长期守卫：**页面重规划 ↔ 车端路线** 对账 + 引擎口径 + 注入幂等 + 确定性
 *
 * 为什么需要这个脚本（t73「地图复杂就乱算+叠加+乱跑」的引擎一半）：
 *   页面「重新规划」此前调 `tools/plan_route.py` **不传 --engine** ⇒ CLI 默认 `tour`，而车端
 *   `config/robot.yaml` 配的是 `tracker-safe` ⇒ 同一张图页面和车算出**两条不同的路线**。
 *   修法：页面以车端配置为唯一真源（plannerEngineOf 读 `planner.engine`），planRoute 拼上 `--engine`。
 *   本脚本钉住三件事，缺一条就等于"修完又复发"：
 *     ① **页面 online 路径**（simbench.js 导出的 planRoute，不是测试里复制的 argv）算出的 token
 *        **逐 token 等于**车端 `route.tokens`；
 *     ② 该结果**只可能**来自车端同引擎：直接显式 `--engine <车端引擎>` 跑 CLI 与之逐 token 相同，
 *        且「不传 --engine」（= 改动前的页面行为）在能区分的快照上给出**不同的**路线；
 *     ③ 引擎取值白名单只有一个真源（plan_route.py 的 argparse choices），插件不再自抄第二份。
 *   另外并入 t1 服务端侧的两条"重复应用"钉子：点位注入/无人机避让注入**连做两次不叠加**、
 *   `fieldSpec()` 连读三次无状态残留；以及重规划**确定性**（同输入同输出 ⇒ 不存在"随机乱算"）。
 *
 * 跑法（任何目录都行，路径按脚本自身位置推导）：
 *     node scripts/test-sim-parity.mjs
 *   退出码：0 = 全绿；1 = 断言失败；2 = 环境前置不满足（缺 python / 缺 workspace 配置）。
 *
 * 写域：**只读**。只读 config/*.yaml、tools/plan_route.py、src/simbench.js；临时 YAML 落在
 *   os.tmpdir() 并在退出前删除。不写 config、不跑仿真、不连车。
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
  SIM_ROOT, fieldSpec, routeTokens, planRoute, plannerEngineOf,
  applyFieldToMapText, injectDroneAvoidToMapText, normalizeCells,
} = sb

const CFG = path.join(SIM_ROOT, 'config', 'robot.yaml')
const MAP = path.join(SIM_ROOT, 'config', 'map.yaml')
const CLI = path.join(SIM_ROOT, 'tools', 'plan_route.py')
const md5 = f => crypto.createHash('md5').update(fs.readFileSync(f)).digest('hex')

for (const [label, file] of [['robot.yaml', CFG], ['map.yaml', MAP], ['plan_route.py', CLI]]) {
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

/* 车端引擎真源：robot.yaml；页面白名单真源：plan_route.py 的 argparse choices */
const engine = plannerEngineOf(CFG)
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
}
const created = []
for (const [k, text] of Object.entries(probes)) created.push(tmpYaml(k, text))
const [pQuoted, pInline, pUnknown, pNone] = created
check('1.2 YAML 引号也认（旧实现误判未知 ⇒ 静默退回 tour）', plannerEngineOf(pQuoted) === 'tracker-safe')
check('1.3 行内注释不污染取值', plannerEngineOf(pInline) === 'tracker-safe')
check('1.4 白名单外的引擎值 ⇒ null（绝不给 argparse 塞未知值）', plannerEngineOf(pUnknown) === null, UNKNOWN)
check('1.5 没有 planner 块 ⇒ null（保持历史默认 tour，与改动前逐字节一致）', plannerEngineOf(pNone) === null)
for (const p of created) fs.rmSync(p, { force: true })

/* ── 2. 页面 online 路径重规划 ────────────────────────────────────── */
h('2. planRoute()（页面「重新规划」走的就是这一条 argv 拼装）')
const page = await planRoute({ configPath: CFG, mapPath: MAP })
console.log('    exit=%d timedOut=%s engine=%s tokens=%d checkExit=%s', page.exitCode, page.timedOut,
  page.engine, page.tokens.length, page.check?.exit)
console.log('    token:', page.tokens.join(' '))
check('2.1 plan_route.py 退出码 0（页面自己没算，是调 CLI）', page.exitCode === 0 && !page.timedOut,
  page.stderrTail || '')
check('2.2 planRoute 记下的 engine == robot.yaml 的 planner.engine', page.engine === engine, `${page.engine} / ${engine}`)
check('2.3 route_check 通过（--check 退出 0）', page.check?.exit === 0)
check('2.4 重规划出非空 token 列表', page.tokens.length > 0, `${page.tokens.length} token`)

/* ── 3. 页面 ↔ 车端 对账 ─────────────────────────────────────────── */
h('3. 页面重规划 ↔ 车端 route.tokens（逐 token）')
const car = routeTokens()
console.log('    车端 token:', car.join(' '))
let firstDiff = -1
for (let i = 0; i < Math.max(car.length, page.tokens.length); i += 1) {
  if (car[i] !== page.tokens[i]) { firstDiff = i; break }
}
check('3.1 逐 token 完全相等（不只是数量相同）', firstDiff < 0, firstDiff < 0 ? '' : `首个不同下标 ${firstDiff}`)
check('3.2 token 数量相等', car.length === page.tokens.length, `${page.tokens.length} vs ${car.length}`)

/* 快照锚：唯一的"具体条数"断言只对官方快照生效；车侧改标定后口径自动转为"页面==车端" */
const SNAPSHOT_44 = '4ffc53d634a301128cff5d6e71d442a8'
if (md5(CFG) === SNAPSHOT_44) {
  check('3.3 官方 44 token 快照（robot.yaml md5 4FFC53D6…）⇒ 恰 44 token',
    page.tokens.length === 44, `${page.tokens.length}`)
} else {
  info(`44 token 验收：本快照 robot.yaml md5=${md5(CFG).slice(0, 8)}… 已被车侧改过（如 t3 射击距离），`
    + `路线合法重钉为 ${page.tokens.length} token；验收口径 = 3.1/3.2「页面 == 车端」（与条数无关）`)
}

/* ── 4. 判别性证据：同一份 CLI，传 / 不传 --engine ─────────────────── */
h('4. 判别性证据：3.1 的相等只可能来自"同引擎"')
const runCli = (extra) => new Promise((resolve) => {
  const argv = ['tools/plan_route.py', '--config', CFG, '--map', MAP, '--emit', 'json', '--check', '--quiet', ...extra]
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
  check('4.1 显式 --engine <车端引擎> 的 CLI 输出 == planRoute 输出（argv 拼装没走样）',
    explicit.code === 0 && explicit.tokens.join(' ') === page.tokens.join(' '),
    `${explicit.tokens.length} vs ${page.tokens.length}`)
} else {
  skip('4.1 显式 --engine 对照', 'robot.yaml 无 planner.engine（页面也不传，两边都是 CLI 默认）')
}
console.log('    不带 --engine（= 改动前的页面行为）: %d token，exit=%d', legacy.tokens.length, legacy.code)
if (legacy.tokens.length === 0) {
  skip('4.2 反证', 'CLI 不带 --engine 没跑出结果')
} else if (legacy.tokens.join(' ') === car.join(' ')) {
  info('本快照两引擎同解（不带 --engine 也与车端相同）⇒ 反证不具判别性，但正面证据 3.1/4.1 已成立；不算失败')
} else {
  let d = -1
  for (let i = 0; i < Math.max(car.length, legacy.tokens.length); i += 1) {
    if (car[i] !== legacy.tokens[i]) { d = i; break }
  }
  check('4.2 反证：不带 --engine ⇒ 与车端路线不同（旧行为确实是另一条路线）', d >= 0,
    `首个不同下标 ${d}；${legacy.tokens.length} vs ${car.length} token`)
}

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

/* ── 6. /sim/map 快照无状态残留 + 官方图计数 ──────────────────────── */
h('6. fieldSpec() 无状态残留（连读三次不变）+ 官方第五张图要素计数')
const specs = [fieldSpec(), fieldSpec(), fieldSpec()]
const dump = s => JSON.stringify({ mines: s.mines, obstacles: s.obstacles, targets: s.targets, bridges: s.bridges, target: s.target })
check('6.1 三次读数逐字节一致（TARGET_CACHE 不会越读越叠）',
  dump(specs[0]) === dump(specs[1]) && dump(specs[1]) === dump(specs[2]))
check('6.2 场地读得动（ok:true，不是错误对象）', specs[0].ok === true, specs[0].error ?? '')

/* 与 YAML 原文独立对账：字段计数随真源走（换图不假红），官方快照另钉 8/8/2/2 */
const yamlLists = {}
for (const key of ['mines', 'obstacles', 'targets', 'bridges']) {
  yamlLists[key] = normalizeCells(String(lineOf(mapText, key).replace(/^[^[]*\[/, '').replace(/\]$/, '')).replace(/["']/g, ''))
}
check('6.3 fieldSpec 四类要素计数 == map.yaml 原文计数',
  ['mines', 'obstacles', 'targets', 'bridges'].every(k => specs[0][k].length === yamlLists[k].length),
  `spec ${specs[0].mines.length}/${specs[0].obstacles.length}/${specs[0].bridges.length}/${specs[0].targets.length}`
  + ` vs yaml ${yamlLists.mines.length}/${yamlLists.obstacles.length}/${yamlLists.bridges.length}/${yamlLists.targets.length}`)
const OFFICIAL_MAP = 'c9bdb660637430a9dd00f518301090bc'
if (md5(MAP) === OFFICIAL_MAP) {
  check('6.4 官方第五张图：雷 8 / 障 8 / 通 2 / 靶 2',
    specs[0].mines.length === 8 && specs[0].obstacles.length === 8
    && specs[0].bridges.length === 2 && specs[0].targets.length === 2,
    `${specs[0].mines.length}/${specs[0].obstacles.length}/${specs[0].bridges.length}/${specs[0].targets.length}`)
} else {
  info(`map.yaml md5=${md5(MAP).slice(0, 8)}… 不是官方第五张图快照（${OFFICIAL_MAP.slice(0, 8)}…）⇒ 跳过 8/8/2/2 钉`)
}

/* ── 7. 路线无重复 token（"重复绘制"的服务端迹象）──────────────────── */
h('7. 车端路线无重复项')
check('7.1 车端 tokens 无重复（重复 token = 重复绘制式的冗余）',
  car.length > 0 && new Set(car).size === car.length, `${car.length} token / 去重 ${new Set(car).size}`)

/* ── 8. 重规划确定性（"乱跑/乱算"的排除项）────────────────────────── */
h('8. 重规划确定性：同输入同输出')
const page2 = await planRoute({ configPath: CFG, mapPath: MAP })
check('8.1 两次重规划逐 token 相同', page2.tokens.join(' ') === page.tokens.join(' '),
  `${page.tokens.length} / ${page2.tokens.length}`)
check('8.2 两次都与车端相等（"稳定但不一致"也不行）',
  page2.tokens.join(' ') === car.join(' ') && page.tokens.join(' ') === car.join(' '))
check('8.3 台账取证字段 engine 与 robot.yaml 一致', page2.engine === engine, String(page2.engine))

/* ── 汇总 ───────────────────────────────────────────────────────── */
const total = pass.ok + pass.fail
console.log('')
console.log(`结果：${pass.ok}/${total} 通过${pass.skip ? `，${pass.skip} 跳过` : ''}`
  + `${pass.fail === 0 ? ' —— 全部通过 ✅' : `，${pass.fail} 失败 ❌`}`)
process.exit(pass.fail === 0 ? 0 : 1)
