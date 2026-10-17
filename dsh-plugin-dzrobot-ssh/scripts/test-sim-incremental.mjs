#!/usr/bin/env node
/* scripts/test-sim-incremental.mjs —— t12 长期守卫：仿真台**增量规划透传**（from_cell / already_covered）
 *
 * 背景：车端 t9 在 **planner 层**加了 `plan_route_tracker_safe(..., from_cell=, already_covered=)`。
 *   仿真台要把它们透传上去，口径必须与 `--engine` 完全同链路：
 *     · 真源 = `tools/plan_route.py` 的 argparse（`plannerIncrementalSupport()` 现场读，不抄第二份）；
 *     · **声明了才传** —— 给 argparse 塞未知参数会 exit 2，页面只看到"改点位后规划失败(422)"，更难查；
 *     · 缺省不传 ⇒ argv 与改动前**逐字节相同**；
 *     · 诉求被降级（未声明/非法格名）时进 `incremental.dropped`，**不静默吞**。
 *
 * ⚠ 状态史：t12 交付时 `tools/plan_route.py` 里**还没有**这两条 add_argument（t9 只到
 *   `planner.py` + `tests/test_incremental_plan.py`），当时现役 CLI `supported = []`，E2E 由
 *   **影子 CLI**（%TEMP% 副本，只补两条 argparse + 两行传参）钉住。t12 收口时队长已给
 *   车端 CLI 接线（`--from-cell` / `--already-covered`，仅 tracker-safe 引擎）：
 *   现役 CLI `supported = 2/2`，E 段自动转为实跑、G 段对账走现役 CLI、D 段影子自动 SKIP；
 *   C 段的"未声明"档改由桩 CLI（无这两条声明的临时文件）承担，降级路径守卫不随接线消失。
 *
 * 跑法：node scripts/test-sim-incremental.mjs
 *   退出码：0 = 全绿；1 = 断言失败；2 = 环境前置不满足（缺 sim.html 之外的 python/配置）。
 *
 * 写域：**只读**工作区（只读 config/*.yaml、tools/plan_route.py、src/*.js）；唯一写入 =
 *   os.tmpdir() 下的影子 CLI，退出前清理。
 */
import crypto from 'node:crypto'
import fs from 'node:fs'
import os from 'node:os'
import path from 'node:path'
import { execFileSync } from 'node:child_process'
import { fileURLToPath } from 'node:url'

const HERE = path.dirname(fileURLToPath(import.meta.url))
const PLUGIN = path.resolve(HERE, '..')
const SIMBENCH = path.join(PLUGIN, 'src', 'simbench.js')

const pass = { ok: 0, fail: 0, skip: 0 }
const check = (name, ok, note = '') => {
  if (ok) pass.ok += 1
  else pass.fail += 1
  console.log(`  ${ok ? 'PASS' : 'FAIL'} ${name}${note ? ' —— ' + note : ''}`)
}
const info = (msg) => console.log(`  INFO ${msg}`)
const skip = (name, why) => { pass.skip += 1; console.log(`  SKIP ${name}${why ? ' —— ' + why : ''}`) }
const h = (t) => console.log(`\n\u001b[36m${t}\u001b[0m`)

if (!fs.existsSync(SIMBENCH)) {
  console.error(`✗ 找不到 ${SIMBENCH}（脚本必须放在 dsh-plugin-dzrobot-ssh/scripts/ 下）`)
  process.exit(2)
}
const sb = await import('file:///' + SIMBENCH.replace(/\\/g, '/'))
const {
  SIM_ROOT, planRoute, routeTokens, plannerIncrementalSupport, appendIncrementalArgs, fieldSpec,
} = sb
const { truthPlanEvidence } = await import('file:///' + path.join(PLUGIN, 'src', 'dronesim.js').replace(/\\/g, '/'))

const CFG = path.join(SIM_ROOT, 'config', 'robot.yaml')
const MAP = path.join(SIM_ROOT, 'config', 'map.yaml')
const CLI = path.join(SIM_ROOT, 'tools', 'plan_route.py')
for (const [label, file] of [['robot.yaml', CFG], ['map.yaml', MAP], ['plan_route.py', CLI]]) {
  if (!fs.existsSync(file)) { console.error(`✗ 环境前置缺失：${label} → ${file}`); process.exit(2) }
}
const md5 = (f) => crypto.createHash('md5').update(fs.readFileSync(f)).digest('hex')
console.log('SIM_ROOT =', SIM_ROOT)
console.log('config   = robot.yaml md5 %s | map.yaml md5 %s', md5(CFG), md5(MAP))

const car = routeTokens()

/* ── 影子 CLI：只补两条 argparse + 两行传参，其余逐字节照抄 ──────────────── */
const PATCH_ANCHOR_ARGS = '    args = ap.parse_args(argv)'
const PATCH_LINES_FLAGS = [
  '    ap.add_argument("--from-cell", default=None, help="影子 CLI（t12 透传取证）：增量续算起点格")',
  '    ap.add_argument("--already-covered", default=None, help="影子 CLI（t12 透传取证）：已压格，逗号分隔")',
]
const PATCH_ANCHOR_CALL = 'turned_mines=turned,'
const PATCH_LINES_CALL = [
  '            from_cell=getattr(args, "from_cell", None),',
  '            already_covered=tuple((args.already_covered or "").split(",")) if getattr(args, "already_covered", None) else (),',
]
const shadowDir = path.join(os.tmpdir(), `t12_shadow_${process.pid}`)
const shadowCli = path.join(shadowDir, 'tools', 'plan_route.py')

function buildShadow() {
  const srcText = fs.readFileSync(CLI, 'utf8')
  if (!srcText.includes(PATCH_ANCHOR_ARGS) || !srcText.includes(PATCH_ANCHOR_CALL)) return null
  let text = srcText.replace(PATCH_ANCHOR_ARGS, [...PATCH_LINES_FLAGS, PATCH_ANCHOR_ARGS].join('\n'))
  text = text.replace(PATCH_ANCHOR_CALL, [PATCH_ANCHOR_CALL, ...PATCH_LINES_CALL].join('\n'))
  fs.mkdirSync(path.join(shadowDir, 'tools'), { recursive: true })
  fs.writeFileSync(shadowCli, text, 'utf8')
  // --check 会子进程调 route_check.py（plan_route 按 __file__ 找同目录），一并拷过去
  const rc = path.join(SIM_ROOT, 'tools', 'route_check.py')
  if (fs.existsSync(rc)) fs.copyFileSync(rc, path.join(shadowDir, 'tools', 'route_check.py'))
  return { srcText, text }
}

const shadow = buildShadow()

h('A. CLI 事实：谁声明了增量开关（真源 = argparse）')
const liveSupport = plannerIncrementalSupport()
const CANON = ['--from-cell', '--already-covered']
console.log('  现役 CLI supported =', JSON.stringify(liveSupport.supported.map(s => s.option)),
  '｜ missing =', JSON.stringify(liveSupport.missing))
check('A1 现役 CLI 的 support 结构合法（supported ⊆ 规范两条、missing 为补集、cli 指向 tools/plan_route.py）',
  liveSupport.supported.every(item => CANON.includes(item.option) && ['fromCell', 'alreadyCovered'].includes(item.key))
  && liveSupport.missing.length === CANON.length - liveSupport.supported.length
  && path.resolve(liveSupport.cli) === path.resolve(CLI),
  `supported=${liveSupport.supported.length}/2 missing=${liveSupport.missing.join(',')}`)

h('B. 缺省不传：argv 与改动前逐字节相同（整线规划语义不变）')
const baseline = await planRoute({ configPath: CFG, mapPath: MAP })
console.log('    argv:', baseline.argv.join(' '))
check('B1 默认调用 argv 不含任何增量开关（也不含重复的 --engine）',
  !baseline.argv.includes('--from-cell') && !baseline.argv.includes('--already-covered')
  && baseline.argv.filter(a => a === '--engine').length === 1, `${baseline.argv.length} 项`)
check('B2 台账 incremental：flags 为空、requested 为空、supported 如实',
  Array.isArray(baseline.incremental.flags) && baseline.incremental.flags.length === 0
  && baseline.incremental.dropped.length === 0,
  JSON.stringify({ flags: baseline.incremental.flags, dropped: baseline.incremental.dropped.length }))
check('B3 缺省产物 == 车端 route.tokens（逐 token）', baseline.tokens.join(' ') === car.join(' '),
  `${baseline.tokens.length} vs ${car.length}`)

h('C. 白名单化：未声明 ⇒ 一个字节不加，但诉求如实进 dropped（不静默吞）')
// 现役 CLI 接线后，"未声明"档改由桩 CLI 承担（白名单降级路径不随接线消失）
const unwiredStubCli = path.join(shadowDir, 'tools', 'plan_route_unwired_stub.py')
fs.mkdirSync(path.dirname(unwiredStubCli), { recursive: true })
fs.writeFileSync(unwiredStubCli, 'ap.add_argument("--engine", choices=("tour", "tracker-safe"))\n', 'utf8')
const liveProbe = appendIncrementalArgs(['python', 'plan_route.py'],
  { fromCell: 'M9', alreadyCovered: ['C1', 'A3'] }, { cliPath: unwiredStubCli })
check('C1 未声明时不追加任何参数（argv 长度不变）', liveProbe.argv.length === 2, `argv=${JSON.stringify(liveProbe.argv)}`)
check('C2 两条诉求都进 dropped 且 reason=cli-unsupported（如实降级，不是消失）',
  liveProbe.applied.dropped.length === 2
  && liveProbe.applied.dropped.every(d => d.reason === 'cli-unsupported'),
  JSON.stringify(liveProbe.applied.dropped.map(d => d.key + ':' + d.reason)))
const badProbe = appendIncrementalArgs(['python', 'plan_route.py'], { fromCell: 'ZZ9', alreadyCovered: ['M99', ''] })
check('C3 非法格名不塞给 argparse（reason=invalid-cell），不让用户看到参数解析错',
  badProbe.argv.length === 2 && badProbe.applied.dropped.every(d => d.reason === 'invalid-cell'),
  JSON.stringify(badProbe.applied.dropped.map(d => d.key + ':' + d.reason)))

const liveWired = liveSupport.supported.length === CANON.length
let inc = null
let covered = null
if (liveWired) {
  skip('D1-D9 影子 CLI E2E', '现役 CLI 已接线——影子只为"未接线"过渡期存在，E 段实跑承接')
  inc = await planRoute({ configPath: CFG, mapPath: MAP, fromCell: 'M9' })
  covered = await planRoute({ configPath: CFG, mapPath: MAP, fromCell: 'M9', alreadyCovered: ['A3', 'C1'] })
} else if (!shadow) {
  console.error('✗ 影子 CLI 构造失败（tools/plan_route.py 的 argparse/传参锚点变了）——本守卫的 E2E 段无法成立')
  process.exit(2)
} else {
  h('D. 影子 CLI（声明了才传）：追加顺序/取值 + 产物 == 车端 planner')
  const shadowSupport = plannerIncrementalSupport(shadowCli)
  check('D1 影子 CLI 的 support == 两条规范选项', shadowSupport.supported.map(s => s.option).join(' ') === CANON.join(' '),
    JSON.stringify(shadowSupport.supported.map(s => s.option)))
  const argvProbe = ['python', 'plan_route.py']
  appendIncrementalArgs(argvProbe, { fromCell: 'm9', alreadyCovered: ['c1', 'A3', 'C1'] }, { cliPath: shadowCli })
  check('D2 追加恰 +4 项、顺序固定、值已归一（M9 / A3,C1 去重排序、小写转大写）',
    argvProbe.slice(2).join('|') === '--from-cell|M9|--already-covered|A3,C1',
    JSON.stringify(argvProbe.slice(2)))

  // 影子 CLI 可信度：除这两处 patch 外与源文件逐字节相同（不是"另一个规划器"）
  const reverted = shadow.text
    .replace([...PATCH_LINES_FLAGS, PATCH_ANCHOR_ARGS].join('\n'), PATCH_ANCHOR_ARGS)
    .replace([PATCH_ANCHOR_CALL, ...PATCH_LINES_CALL].join('\n'), PATCH_ANCHOR_CALL)
  check('D3 影子 CLI == 源文件 + 仅 4 行接线（回滚 patch 后逐字节相同）', reverted === shadow.srcText,
    `+${shadow.text.split('\n').length - shadow.srcText.split('\n').length} 行`)

  inc = await planRoute({ configPath: CFG, mapPath: MAP, cliPath: shadowCli, fromCell: 'M9' })
  const shadowBase = await planRoute({ configPath: CFG, mapPath: MAP, cliPath: shadowCli })
  console.log('    增量: exit=%d tokens=%d 首点=%s checkExit=%s ｜ 基线: tokens=%d',
    inc.exitCode, inc.tokens.length, inc.tokens[0], inc.check?.exit, shadowBase.tokens.length)
  check('D4 增量重规划 exit 0 且 route_check 通过', inc.exitCode === 0 && inc.check?.exit === 0,
    inc.stderrTail || '')
  check('D5 路线首点 == from_cell（真的从 M9 续算，不是从地图起点整线）', inc.tokens[0] === 'M9', String(inc.tokens[0]))
  check('D6 增量产物 != 基线产物（参数确实生效，不是被忽略）',
    inc.tokens.join(' ') !== shadowBase.tokens.join(' '),
    `${inc.tokens.length} vs ${shadowBase.tokens.length}`)
  check('D7 页面（planRoute）增量产物 == 显式 CLI 直跑产物（逐 token）', (() => {
    const raw = execFileSync('python', [shadowCli, '--config', CFG, '--map', MAP, '--emit', 'json',
      '--check', '--quiet', '--engine', baseline.engine, '--from-cell', 'M9'], {
      cwd: SIM_ROOT, encoding: 'utf8', maxBuffer: 64 * 1024 * 1024,
      env: { ...process.env, PYTHONPATH: path.join(SIM_ROOT, 'src', 'dzbrain'), PYTHONIOENCODING: 'utf-8', PYTHONUNBUFFERED: '1' },
    })
    const payload = JSON.parse(raw.slice(raw.indexOf('{')))
    const explicit = payload.plans?.[0]?.tokens ?? []
    return explicit.length > 0 && explicit.join(' ') === inc.tokens.join(' ')
  })(), `${inc.tokens.length} token`)

  covered = await planRoute({ configPath: CFG, mapPath: MAP, cliPath: shadowCli, fromCell: 'M9', alreadyCovered: ['A3', 'C1'] })
  console.log('    +already_covered(A3,C1): tokens=%d 首点=%s', covered.tokens.length, covered.tokens[0])
  check('D8 already_covered 生效：路线不比"只给 from_cell"更长，且 argv 两项俱全',
    covered.tokens.length <= inc.tokens.length
    && covered.argv.includes('--already-covered') && covered.argv.includes('--from-cell')
    && covered.incremental.alreadyCovered.join(',') === 'A3,C1',
    `${covered.tokens.length} ≤ ${inc.tokens.length}`)
  check('D9 增量请求下 dropped 为空（声明了就一定传，不再降级）', covered.incremental.dropped.length === 0)
}

/* ── G. 三方对账：页面 == CLI（影子或现役）== 车端 planner 直调（t9 的 API 语义）── */
h('G. 三方对账：页面 planRoute == CLI == 车端 planner 直调')
{
  const probePath = path.join(shadowDir, 'planner_probe.py')
  fs.writeFileSync(probePath, [
    'import json, sys',
    'ROOT = sys.argv[1]',
    'sys.path.insert(0, ROOT + "/src/dzbrain")',
    'import yaml',
    'from dzbrain.core.config import Config',
    'from dzbrain.route import planner as PL',
    'cfg = Config.load(ROOT + "/config/robot.yaml")',
    'map_cfg = yaml.safe_load(open(ROOT + "/config/map.yaml", encoding="utf-8")) or {}',
    'a = PL.plan_route_tracker_safe(map_cfg, cfg, from_cell="M9")',
    'b = PL.plan_route_tracker_safe(map_cfg, cfg, from_cell="M9", already_covered=["A3", "C1"])',
    'print(json.dumps({"from_cell": list(a.tokens), "covered": list(b.tokens)}))',
  ].join('\n'), 'utf8')
  const probe = JSON.parse(execFileSync('python', [probePath, SIM_ROOT], {
    cwd: SIM_ROOT, encoding: 'utf8', maxBuffer: 64 * 1024 * 1024,
    env: { ...process.env, PYTHONPATH: path.join(SIM_ROOT, 'src', 'dzbrain'), PYTHONIOENCODING: 'utf-8', PYTHONUNBUFFERED: '1' },
  }))
  console.log('    planner 直调: from_cell=%d token ｜ +already_covered=%d token',
    probe.from_cell.length, probe.covered.length)
  check('G1 车端 planner 直调（from_cell=M9）== 页面增量产物（逐 token）',
    probe.from_cell.length > 0 && probe.from_cell.join(' ') === inc.tokens.join(' '),
    `${probe.from_cell.length} vs ${inc.tokens.length}`)
  check('G2 车端 planner 直调（+already_covered A3,C1）== 页面产物（逐 token）',
    probe.covered.length > 0 && probe.covered.join(' ') === covered.tokens.join(' '),
    `${probe.covered.length} vs ${covered.tokens.length}`)
}

h('E. 现役 CLI 分支（车端接线后自动从 SKIP 转为实跑）')
if (liveWired) {
  check('E1 现役 CLI 增量重规划 exit 0 / check 通过 / 首点 == from_cell',
    inc.exitCode === 0 && inc.check?.exit === 0 && inc.tokens[0] === 'M9',
    `exit=${inc.exitCode} 首点=${inc.tokens[0]}`)
  check('E2 现役 CLI 增量产物 != 整线基线（参数生效）', inc.tokens.join(' ') !== car.join(' '),
    `${inc.tokens.length} vs ${car.length}`)
} else {
  skip('E1/E2 现役 CLI 增量 E2E',
    `tools/plan_route.py 还没有 ${liveSupport.missing.map(k => (k === 'fromCell' ? '--from-cell' : '--already-covered')).join(' / ')}`
    + '（t9 只改到 planner 层）⇒ 透传按白名单如实降级为"不传"，E2E 由 D 段影子 CLI 钉住')
}

/* ── F. dronesim.truthPlanEvidence 的运行期透传（与页面同一套 helper）────────── */
h('F. dronesim truthPlanEvidence（无人机时间线的 truth 重规划取证）')
{
  const droneSrc = fs.readFileSync(path.join(PLUGIN, 'src', 'dronesim.js'), 'utf8')
  check('F1 取证 argv 走 appendIncrementalArgs 且仍拼 --engine（同链路，不另抄一套）',
    /appendIncrementalArgs\(argv,\s*\{\s*fromCell,\s*alreadyCovered\s*\}\)/.test(droneSrc)
    && /argv\.push\('--engine', engine\)/.test(droneSrc))
  const spec = fieldSpec()
  const wired = liveSupport.supported.length === CANON.length
  const withInc = await truthPlanEvidence({
    board: spec, truth: spec.bridges, id: `t12_inc_${Date.now()}`, label: 'inc', drone: false,
    fromCell: 'M9', alreadyCovered: ['C1'],
  })
  const plain = await truthPlanEvidence({
    board: spec, truth: spec.bridges, id: `t12_plain_${Date.now()}`, label: 'plain', drone: false,
  })
  console.log('    增量取证 exit=%d tokens=%d ｜ 不带增量 tokens=%d ｜ CLI 已接线=%s',
    withInc.exitCode, withInc.tokens.length, plain.tokens.length, wired)
  check('F2 增量取证 exit 0（未接线时不塞未知参数、不炸 argparse）', withInc.exitCode === 0,
    withInc.stderrTail || '')
  check('F3 产物随"CLI 是否声明"如实变化（未接线 ⇒ 与不带增量逐 token 相同；已接线 ⇒ 从 M9 续算）',
    wired
      ? withInc.tokens[0] === 'M9' && withInc.tokens.join(' ') !== plain.tokens.join(' ')
      : withInc.tokens.join(' ') === plain.tokens.join(' '),
    wired ? `首点=${withInc.tokens[0]}` : `${withInc.tokens.length} vs ${plain.tokens.length}`)
  check('F4 台账如实反映实际参数（未接线 ⇒ argv 无增量开关、dropped 两条 cli-unsupported）',
    Array.isArray(withInc.incremental?.supported) && (wired
      ? withInc.argv.includes('--from-cell') && withInc.incremental.dropped.length === 0
      : !withInc.argv.includes('--from-cell')
        && withInc.incremental.dropped.length === 2
        && withInc.incremental.dropped.every(d => d.reason === 'cli-unsupported')),
    JSON.stringify({ flags: withInc.incremental?.flags, dropped: withInc.incremental?.dropped?.length }))
}

/* ── 清理影子目录 ─────────────────────────────────────────────────── */
try { fs.rmSync(shadowDir, { recursive: true, force: true }) } catch { /* 清理失败不影响结论 */ }

const total = pass.ok + pass.fail
console.log('')
console.log(`结果：${pass.ok}/${total} 通过${pass.skip ? `，${pass.skip} 跳过` : ''}`
  + `${pass.fail === 0 ? ' —— 全部通过 ✅' : `，${pass.fail} 失败 ❌`}`)
process.exit(pass.fail === 0 ? 0 : 1)
