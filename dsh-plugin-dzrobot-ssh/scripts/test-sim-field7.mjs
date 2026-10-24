#!/usr/bin/env node
/* scripts/test-sim-field7.mjs —— 仿真台「场地来源覆盖 + 手工路线 + 显式引擎（R1）」守卫。
 *
 * 背景：2026-10-18 实车换第七张图；仓库 config/ 仍保留第六张图做回归基线，所以仿真台
 *   必须支持"运行期只读源覆盖"（source dir），而不是去改 config/。
 *   本脚本钉住四类语义：
 *     A. resolveFieldSource：工作区内目录/文件可读；越界/缺文件显式失败（绝不静默回落）。
 *     B. fieldSpec({mapPath, configPath})：解析任意一张图（含第七张图的逐格要素）。
 *     C. 41 token 倒车路线：normalizeRouteTokens 保留 R: 前缀 / * 后缀 / ,x，只拒非法 token。
 *     D. sanitizeParams / 页面：source/route/engine 只在带键时出现；非法值给错误不回落；
 *        页面有「场地来源目录」+「显式引擎（what-if 红字）」。
 *     E. 真源第七张图（_tmp_session/field7，若存在）：逐格对齐 + 无人机起降格如实报
 *        builtin-fallback（该图 map.yaml 没有起降注释）。
 *
 * 跑法：node scripts/test-sim-field7.mjs（只读；临时目录只写 _tmp_session/sim_t29/tmp_*）
 */
import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const HERE = path.dirname(fileURLToPath(import.meta.url))
const PLUGIN = path.resolve(HERE, '..')
const WS = path.resolve(PLUGIN, '..')
const FIELD7 = path.resolve(WS, '..', '_tmp_session', 'field7')
const TMP_ROOT = path.resolve(WS, '..', '_tmp_session', 'sim_t29')

const pass = { ok: 0, fail: 0, skip: 0 }
const check = (name, ok, note = '') => {
  if (ok) pass.ok += 1
  else pass.fail += 1
  console.log('  ' + (ok ? 'PASS' : 'FAIL') + ' ' + name + (note ? ' —— ' + note : ''))
}
const skip = (name, why) => { pass.skip += 1; console.log('  SKIP ' + name + (why ? ' —— ' + why : '')) }
const h = (t) => console.log('\n\u001b[36m' + t + '\u001b[0m')
const sortCells = list => list.slice().sort((a, b) => (a[0] === b[0] ? Number(a.slice(1)) - Number(b.slice(1)) : a[0].localeCompare(b[0])))
const sameSet = (a, b) => JSON.stringify(sortCells(a)) === JSON.stringify(sortCells(b))

const sb = await import('file:///' + path.join(PLUGIN, 'src', 'simbench.js').replace(/\\/g, '/'))
const { resolveFieldSource, fieldSpec, normalizeRouteTokens, sanitizeParams, routeTokensOf, routeTokensFromFile, planRoute } = sb

/* ── A. resolveFieldSource ─────────────────────────────────────── */
h('A. 场地来源：工作区内可读，越界/缺文件显式失败')
fs.mkdirSync(TMP_ROOT, { recursive: true })
const tmpDir = fs.mkdtempSync(path.join(TMP_ROOT, 'tmp_source_'))
const tmpMap = path.join(tmpDir, 'map.yaml')
const tmpRobot = path.join(tmpDir, 'robot.yaml')
fs.writeFileSync(tmpMap, 'map:\n  name: "tmp"\n  start: "N10"\n  end: "N5"\n  barrier_col_from: "D"\n  barrier_row_hi: 6\n  barrier_row_lo: 7\n  mines: ["A1"]\n  obstacles: []\n  targets: ["B2","C2"]\n  bridges: ["K7"]\n', 'utf8')
fs.writeFileSync(tmpRobot, 'planner:\n  engine: tracker-safe\n  auto_plan_enabled: false\nroute:\n  tokens: ["N10","M10"]\n', 'utf8')
const srcOk = resolveFieldSource({ dir: tmpDir })
check('A1 dir 指向含 robot.yaml+map.yaml 的目录 → 解析成功且路径存在',
  srcOk && !srcOk.error && fs.existsSync(srcOk.configPath) && fs.existsSync(srcOk.mapPath), JSON.stringify(srcOk))
check('A2 解析结果保留 dir 绝对路径（台账可追溯）', srcOk && srcOk.dir === path.resolve(tmpDir))
const srcBad = resolveFieldSource({ dir: path.join(tmpDir, 'nope') })
check('A3 缺文件 → 显式 error（不回落 config/）', srcBad && typeof srcBad.error === 'string' && srcBad.error.includes('缺文件'), srcBad && srcBad.error)
const srcOut = resolveFieldSource({ dir: 'C:\\Windows' })
check('A4 工作区外目录 → 显式拒绝', srcOut && typeof srcOut.error === 'string', srcOut && srcOut.error)
check('A5 完全不传 source → null（普通跑行为不变）', resolveFieldSource(undefined) === null)
fs.writeFileSync(path.join(tmpDir, 'route.tokens.txt'), 'N10\nM10\nR:H9\n', 'utf8')
const srcRoute = resolveFieldSource({ dir: tmpDir })
check('A6 目录带 route.tokens.txt → routePath 认出，且 token 解析为 3 段',
  srcRoute && srcRoute.routePath && path.resolve(srcRoute.routePath) === path.resolve(tmpDir, 'route.tokens.txt')
  && routeTokensFromFile(srcRoute.routePath).length === 3, srcRoute && srcRoute.routePath)
fs.writeFileSync(path.join(tmpDir, 'route.tokens.txt'), 'N10\nBAD\n', 'utf8')
check('A7 route 文件含非法 token → routeTokensFromFile 返空（调用方显式报错，不半用）',
  routeTokensFromFile(path.join(tmpDir, 'route.tokens.txt')).length === 0)

/* ── B. fieldSpec 任意图 ───────────────────────────────────────── */
h('B. fieldSpec({mapPath,configPath}) 解析任意图（含 planner 口径）')
const spec = fieldSpec({ mapPath: tmpMap, configPath: tmpRobot })
check('B1 解析成功 + source 指向指定 map', spec.ok === true && path.resolve(spec.source) === path.resolve(tmpMap), spec.error || spec.source)
check('B2 点位按指定图解析（雷 A1 / 靶 B2+C2 横）', sameSet(spec.mines, ['A1']) && sameSet(spec.targets, ['B2', 'C2']) && spec.target.yaw === 'h', spec.mines.join(' ') + ' / ' + spec.targets.join('+') + ' / ' + spec.target.yaw)
check('B3 planner 口径来自指定 robot.yaml（tracker-safe + auto_plan_enabled=false）',
  spec.planner && spec.planner.engine === 'tracker-safe' && spec.planner.autoPlanEnabled === false, JSON.stringify(spec.planner))
check('B4 routeTokensOf 读指定 robot.yaml 的 route.tokens', sameSet(routeTokensOf(tmpRobot), ['M10', 'N10']), routeTokensOf(tmpRobot).join(' '))

/* ── C. 41 token 倒车路线 token 白名单 ─────────────────────────── */
h('C. 手工路线 token 白名单（41 token / 3 段 R: / 末点 ,x）')
const TOKENS = 'N10 M10 L10 K10 J10 I10 H10 H8 R:H9 G9 F9* E9 E8 C8 C7 C6 C5 C4 C3 C2 A2 A1 B1 C1 C3 E3 F3 R:F5 R:F4 F3 F2 G2 H2 I2 J2 J4 K4 L4 M4 N4 N5,x'.split(/\s+/)
const norm = normalizeRouteTokens(TOKENS)
check('C1 41 token 全通过白名单', norm.tokens.length === 41 && norm.rejected.length === 0, 'tokens=' + norm.tokens.length + ' rejected=' + norm.rejected.join(' '))
check('C2 3 段倒车 R: 保留', norm.tokens.filter(t => /^R:/.test(t)).length === 3, norm.tokens.filter(t => /^R:/.test(t)).join(' '))
check('C3 射击 * 与末点 ,x 保留（大小写归一为 ,x）', norm.tokens.includes('F9*') && norm.tokens[norm.tokens.length - 1] === 'N5,x', norm.tokens[norm.tokens.length - 1])
const bad = normalizeRouteTokens(['N10', 'x', 'N5,x', 'Z9', 'R:'])
check('C4 非法 token 显式收集（x/Z9/R:）', bad.rejected.length === 3 && bad.tokens.length === 2, bad.rejected.join(' '))

/* ── D. sanitizeParams / 页面接线 ─────────────────────────────── */
h('D. 请求体形状：带键才挂 + 非法值显式报错 + 页面有来源/引擎控件')
const plain = sanitizeParams({})
check('D1 普通请求 7 键形状不变（无 source/route/engine 键）',
  Object.keys(plain).length === 7 && !('source' in plain) && !('route' in plain) && !('engine' in plain), Object.keys(plain).join(','))
const full = sanitizeParams({ source: { dir: tmpDir }, route: { tokens: TOKENS }, engine: 'tour' })
check('D2 带 source/route/engine → 三者都挂且值合法',
  full.source && full.route && full.route.tokens.length === 41 && full.engine === 'tour' && !full.sourceError && !full.routeError && !full.engineError, JSON.stringify({ e: full.engine, n: full.route && full.route.tokens.length }))
const broken = sanitizeParams({ source: { dir: path.join(tmpDir, 'nope') }, route: 'N10 x', engine: 'bogus' })
check('D3 非法 source/route/engine → 三个 error 字段，不静默回落',
  broken.source === null && broken.route === null && broken.engine === null
  && !!broken.sourceError && !!broken.routeError && !!broken.engineError,
  [broken.sourceError, broken.routeError, broken.engineError].join(' | ').slice(0, 160))
const html = fs.readFileSync(path.join(PLUGIN, 'sim.html'), 'utf8')
const indexSrc = fs.readFileSync(path.join(PLUGIN, 'index.js'), 'utf8')
check('D4 页面有「场地来源目录」输入 + loadField 走 fieldMapUrl()',
  /id="sourceDir"/.test(html) && /function fieldMapUrl\(\)/.test(html) && /api\(fieldMapUrl\(\)\)/.test(html))
check('D5 页面有「显式引擎」选择 + what-if 红字（该路线不是车端将走的路线）',
  /id="engine"/.test(html) && /该路线不是车端将走的路线/.test(html) && /body\.engine = engineSel\.value/.test(html))
check('D6 /sim/map?source= 走 resolveFieldSource、失败显式 400、有 route.tokens.txt 时优先用它',
  /url\.searchParams\.get\('source'\)/.test(indexSrc) && /resolveFieldSource\(\{ dir: sourceRaw \}\)/.test(indexSrc) && /send\(400/.test(indexSrc)
  && /resolved\.routePath/.test(indexSrc) && /routeTokensFromFile\(resolved\.routePath\)/.test(indexSrc))
check('D7 runSimJob 支持 source（复制源图/配置 + md5 核对）与 manual route',
  /record\.source = \{/.test(indexSrc + fs.readFileSync(path.join(PLUGIN, 'src', 'simbench.js'), 'utf8'))
  && /manual\(field-source\)/.test(fs.readFileSync(path.join(PLUGIN, 'src', 'simbench.js'), 'utf8')))
// R1 成功路径 E2E：显式 tour 必须真的进 argv（不是只在 sanitizeParams 里留个字段）。
const fixtureMap = path.join(PLUGIN, 'scripts', 'fixtures', 'map_fifth_solvable.yaml')
const repoRobot = path.join(WS, 'config', 'robot.yaml')
if (fs.existsSync(fixtureMap) && fs.existsSync(repoRobot)) {
  const planned = await planRoute({ configPath: repoRobot, mapPath: fixtureMap, engine: 'tour', timeoutMs: 120000 })
  check('D8 显式 tour → planRoute.argv 带 --engine tour，engineSource=explicit',
    planned.exitCode === 0 && planned.engine === 'tour' && planned.engineSource === 'explicit'
    && planned.argv.includes('tour') && planned.tokens.length > 0,
    'exit=' + planned.exitCode + ' engine=' + planned.engine + ' source=' + planned.engineSource + ' tokens=' + planned.tokens.length)
} else {
  skip('D8 显式 tour E2E', '缺夹具图或 config/robot.yaml')
}

/* ── E. 真源第七张图（有则逐格对齐）──────────────────────────── */
h('E. 真源第七张图（_tmp_session/field7）逐格对齐')
if (!fs.existsSync(path.join(FIELD7, 'map.yaml')) || !fs.existsSync(path.join(FIELD7, 'robot.yaml'))) {
  skip('E1~E6 第七张图逐格', '_tmp_session/field7 不存在（非本机/已清理）')
} else {
  const f7 = fieldSpec({ mapPath: path.join(FIELD7, 'map.yaml'), configPath: path.join(FIELD7, 'robot.yaml') })
  check('E1 第七张图可读（source 指向 field7/map.yaml）', f7.ok === true && path.resolve(f7.source) === path.resolve(FIELD7, 'map.yaml'))
  check('E2 雷 8 格 == 现场图', sameSet(f7.mines, ['H8', 'G9', 'A2', 'B1', 'C3', 'E3', 'F5', 'I2']), f7.mines.join(' '))
  check('E3 障 8 格 == 现场图', sameSet(f7.obstacles, ['D5', 'I6', 'B7', 'F8', 'C9', 'E10', 'G10', 'A11']), f7.obstacles.join(' '))
  check('E4 靶 D7+E7 横向整块 / 桥洞 K10(下)+K4(上) / 起终 N10→N5',
    sameSet(f7.targets, ['D7', 'E7']) && f7.target.yaw === 'h'
    && sameSet(f7.bridges, ['K4', 'K10'])
    && f7.start === 'N10' && f7.end === 'N5', f7.targets.join('+') + ' ' + f7.target.yaw + ' / ' + f7.bridges.join(' '))
  check('E5 隔板 D 列起、行 6/7；车端引擎 tracker-safe + 手动路线态',
    f7.barrier.colFrom === 'D' && f7.barrier.rowHi === 6 && f7.barrier.rowLo === 7
    && f7.planner.engine === 'tracker-safe' && f7.planner.autoPlanEnabled === false, JSON.stringify(f7.planner))
  check('E6 无人机起降格如实报 builtin-fallback（第七张图 map.yaml 无起降注释，值是兜底不是真值）',
    f7.drone.takeoff === 'N8' && f7.drone.land === 'M2' && f7.drone.source === 'builtin-fallback', JSON.stringify(f7.drone))
}

/* ── 汇总 ─────────────────────────────────────────────────────── */
try { fs.rmSync(tmpDir, { recursive: true, force: true }) } catch {}
const total = pass.ok + pass.fail
console.log('')
console.log('结果：' + pass.ok + '/' + total + ' 通过' + (pass.skip ? '，' + pass.skip + ' 跳过' : '')
  + (pass.fail === 0 ? ' —— 全部通过 ✅' : '，' + pass.fail + ' 失败 ❌'))
process.exit(pass.fail === 0 ? 0 : 1)
