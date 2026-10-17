#!/usr/bin/env node
/* ★ 「乱算 + 地图叠加/乱跑」修复的断言式自检（纯 node，无测试框架）。
 *
 * 两个根因（修复对应关系）：
 *   R1 乱算  —— simbench.planRoute 不传 --engine ⇒ plan_route.py 默认 tour，而车端
 *      robot.yaml 配 tracker-safe ⇒ 同一张图页面和车算出两条不同路线。
 *      修法：planRoute 读 planner.engine（plannerEngineOf）传入，台账落 record.plan.engine。
 *   R2 叠加/乱跑 —— renderMap 只重写 #map.innerHTML，旧层（#anim 回放折线 + TRACE 的
 *      参考线/绿轨迹/车身位姿）不随「棋盘变更」或「422 plan-failed」清除 ⇒ 上一局图层
 *      叠在新棋盘上。修法：afterFieldChange 统一清层（invalidateStaleRun）+ 422 分支清 TRACE。
 *
 * 跑法：node scripts/test-sim-overlay-fix.mjs → 末行「结果：N/N」，全过 exit 0。
 */
import fs from 'node:fs'
import os from 'node:os'
import path from 'node:path'
import { spawn } from 'node:child_process'
import { fileURLToPath, pathToFileURL } from 'node:url'

const HERE = path.dirname(fileURLToPath(import.meta.url))
const PLUGIN = path.resolve(HERE, '..')
const html = fs.readFileSync(path.join(PLUGIN, 'sim.html'), 'utf8')
const benchSrc = fs.readFileSync(path.join(PLUGIN, 'src', 'simbench.js'), 'utf8')

let pass = 0
let fail = 0
function check(name, ok, note = '') {
  if (ok) { pass += 1 } else { fail += 1 }
  console.log(`  ${ok ? 'PASS' : 'FAIL'} ${name}${note ? ' —— ' + note : ''}`)
}
function section(title) { console.log(''); console.log(title) }

/** 找 dzrobot_v2 工作区根（插件目录向上找 src/dzbrain + config/robot.yaml）。 */
function findWorkspaceRoot() {
  let dir = PLUGIN
  for (let i = 0; i < 4; i++) {
    if (fs.existsSync(path.join(dir, 'src', 'dzbrain')) && fs.existsSync(path.join(dir, 'config', 'robot.yaml'))) return dir
    dir = path.dirname(dir)
  }
  return null
}

/** 按花括号配平提取一个顶层函数声明（这些函数体的字符串/注释里没有裸花括号）。 */
function extractFn(src, name) {
  const start = src.indexOf(`function ${name}(`)
  if (start < 0) return ''
  const open = src.indexOf('{', start)
  let depth = 0
  for (let i = open; i < src.length; i++) {
    if (src[i] === '{') depth++
    else if (src[i] === '}') { depth--; if (depth === 0) return src.slice(start, i + 1) }
  }
  return ''
}

/** 沙箱：with + Proxy 把自由变量全部落到 state 上（赋值可观测、调用可记录）。 */
function makeCtx() {
  const els = {}
  const el = id => els[id] || (els[id] = {
    id, value: '', checked: false, disabled: false, innerHTML: '', textContent: '',
    style: {}, addEventListener() {}, getElementsByTagName: () => [{ innerHTML: '' }],
  })
  el('anim').innerHTML = '<polyline points="1,2 3,4"/>'   // 预置旧层：清没清一眼可断
  const state = {
    // with+Proxy 的 has 恒真会把内置对象也劫持成 undefined —— 显式播种还原
    JSON, Date, Math, String, Number, Boolean, isFinite, isNaN, parseFloat, parseInt,
    calls: [],
    animStateText: '',
    TRACE: { route: ['N10', 'M10', 'L10'], points: [[1, 2], [3, 4], [5, 6]], poses: [[1, 2, 0.5]] },
    FIELD: {
      ok: true, route: ['N10', 'M10'], start: 'N10', end: 'N5',
      mines: [], obstacles: [], bridges: [], targets: [],
      grid: { cols: 14, rows: 11 }, drone: { takeoff: 'N8', land: 'M2' },
    },
    renderMap() { state.calls.push('renderMap') },
    startReplay(r) { state.calls.push('startReplay:' + (r && r.id ? r.id : 'null')) },
    stopReplay() { state.calls.push('stopReplay') },
    loadHistory() { state.calls.push('loadHistory'); return Promise.resolve(3) },
    refreshFieldHints() { state.calls.push('refreshFieldHints') },
    api() {
      state.calls.push('api')
      return Promise.reject({ status: 422, body: { record: { id: 'r-422', status: 'plan-failed' } },
        message: '改点位后规划/校验未过' })
    },
    fieldInputErrors() { return [] },
    readForm() { return {} },
    renderResult() {},
    message() {},
    text(target, v) { state.animStateText = String(v) },
    num(v) { return v },
    droppedSummary() { return '' },
    $: el,
  }
  return new Proxy(state, { has: () => true, get: (t, k) => t[k], set: (t, k, v) => { t[k] = v; return true } })
}

/* ── A. 脚本提取与语法 ─────────────────────────────────────────── */
section('A. sim.html 脚本提取与语法')
const scriptMatch = html.match(/<script>([\s\S]*)<\/script>/)
check('A1 <script> 恰一段可提取', !!scriptMatch && scriptMatch[1].length > 1000)
const script = scriptMatch ? scriptMatch[1] : ''
let syntaxOk = true
try { new Function(script) } catch (e) { syntaxOk = false; console.log('    语法错误:', e.message) }
check('A2 整段脚本 new Function 解析通过（改动没写坏语法）', syntaxOk)

/* ── B. 422 plan-failed 分支：清 TRACE 再回放（R2 之一） ───────── */
section('B. 422 plan-failed 分支：清 TRACE 再回放')
{
  const srcRun = extractFn(script, 'runSimulation')
  check('B1 提取到 runSimulation 源码', srcRun.length > 200)
  const ctx = makeCtx()
  const factory = new Function('ctx', `with (ctx) { ${srcRun}\n; return runSimulation; }`)
  await factory(ctx)()
  // runSimulation 不 return promise 链（历史形状）——用一次宏任务排空等微任务结算完
  await new Promise(resolve => setTimeout(resolve, 25))
  const calls = ctx.calls
  check('B2 422 后上一局 TRACE 被清（null，旧参考线/轨迹/位姿不再叠图）', ctx.TRACE === null)
  check('B3 清层后重绘（renderMap 被调）', calls.includes('renderMap'))
  check('B4 plan-failed 仍回放（startReplay 收到 422 的 record）', calls.includes('startReplay:r-422'))
  check('B5 顺序正确：先 renderMap 后 startReplay（先清后画）',
    calls.indexOf('renderMap') >= 0 && calls.indexOf('renderMap') < calls.indexOf('startReplay:r-422'))
  check('B6 plan-failed 仍进台账（loadHistory 被调，t7 语义不回退）', calls.includes('loadHistory'))
}

/* ── C. afterFieldChange：棋盘变更统一清旧层（R2 之二） ────────── */
section('C. afterFieldChange：棋盘变更统一清旧层')
{
  const srcInv = extractFn(script, 'invalidateStaleRun')
  const srcAfter = extractFn(script, 'afterFieldChange')
  check('C1 提取到 invalidateStaleRun / afterFieldChange 源码', srcInv.length > 100 && srcAfter.length > 20)
  const ctx = makeCtx()
  const factory = new Function('ctx', `with (ctx) { ${srcInv}\n${srcAfter}\n; return afterFieldChange; }`)
  factory(ctx)()
  check('C2 TRACE 被清（null）', ctx.TRACE === null)
  check('C3 旧回放被停（stopReplay 被调）', ctx.calls.includes('stopReplay'))
  check('C4 #anim 覆盖层被清空（旧折线不再叠图）', ctx.$('anim').innerHTML === '')
  check('C5 animState 有可见说明', /棋盘已改动/.test(String(ctx.animStateText)))
  check('C6 原有链保留（refreshFieldHints + renderMap 都被调）',
    ctx.calls.includes('refreshFieldHints') && ctx.calls.includes('renderMap'))
}

/* ── D. 变更路径全部走统一咽喉（静态接线） ─────────────────────── */
section('D. 变更路径全部走统一咽喉（静态接线）')
{
  const at422 = script.indexOf('if (problem.status === 422)')
  const seg422 = at422 >= 0 ? script.slice(at422, at422 + 700) : ''
  check('D1 422 分支存在', at422 >= 0)
  check('D2 422 分支先清 TRACE 再 startReplay',
    /TRACE\s*=\s*null/.test(seg422) && seg422.indexOf('TRACE = null') >= 0
    && seg422.indexOf('TRACE = null') < seg422.indexOf('startReplay'))
  const inputBinds = script.match(/addEventListener\('input'[^)]*\)/g) || []
  check('D3 镜像 input 绑定全部走 afterFieldChange（无旁路）',
    inputBinds.length > 0 && inputBinds.every(b => /afterFieldChange/.test(b)),
    `${inputBinds.length} 处 input 绑定`)
  const msgAt = script.indexOf('已恢复现场默认值')
  const restoreSeg = msgAt >= 0 ? script.slice(Math.max(0, msgAt - 350), msgAt + 50) : ''
  check('D4 恢复默认走 afterFieldChange（含清层）',
    msgAt >= 0 && restoreSeg.indexOf('afterFieldChange();') >= 0)
  const resetAt = script.indexOf("$('resetAll')")
  const resetSeg = resetAt >= 0 ? script.slice(resetAt, resetAt + 400) : ''
  check('D5 整图恢复先清旧层（invalidateStaleRun）', /invalidateStaleRun\(\)/.test(resetSeg))
  check('D6 targetYaw 只重绘不误清层（渲染项不进清层咽喉）',
    /\$\('targetYaw'\)\.addEventListener\('change',\s*function/.test(script))
}

/* ── E. 页面与车端同引擎（R1） ────────────────────────────────── */
section('E. 页面与车端同引擎（修「乱算」）')
{
  check('E1 simbench 导出 plannerEngineOf', /export function plannerEngineOf/.test(benchSrc))
  check('E2 planRoute 把 planner.engine 传给 --engine',
    /const engine = plannerEngineOf\(configPath\)/.test(benchSrc)
    && /argv\.push\('--engine', engine\)/.test(benchSrc))
  check('E3 台账落 record.plan.engine（取证字段）', /engine: planned\.engine/.test(benchSrc))
  const root = findWorkspaceRoot()
  check('E4 找到 dzrobot_v2 工作区（真 config 可测）', !!root)
  if (root) {
    const { plannerEngineOf } = await import(pathToFileURL(path.join(PLUGIN, 'src', 'simbench.js')).href)
    const engine = plannerEngineOf(path.join(root, 'config', 'robot.yaml'))
    check('E5 从现役 robot.yaml 读到 tracker-safe（与车端一致）', engine === 'tracker-safe', `读到 ${engine}`)
    const tmpNo = path.join(os.tmpdir(), 't-overlay-no-planner.yaml')
    fs.writeFileSync(tmpNo, 'route:\n  tokens: ["N10"]\n', 'utf8')
    check('E6 无 planner 块 → null（不传 --engine，保持历史默认）', plannerEngineOf(tmpNo) === null)
    fs.rmSync(tmpNo, { force: true })
    const tmpBad = path.join(os.tmpdir(), 't-overlay-bad-engine.yaml')
    fs.writeFileSync(tmpBad, 'planner:\n  engine: bogus\nroute:\n  tokens: ["N10"]\n', 'utf8')
    check('E7 非法引擎值 → null（不给 argparse 塞未知值）', plannerEngineOf(tmpBad) === null)
    fs.rmSync(tmpBad, { force: true })
    const tmpTour = path.join(os.tmpdir(), 't-overlay-tour.yaml')
    fs.writeFileSync(tmpTour, 'planner:\n  engine: tour      # 历史引擎\nroute:\n  tokens: ["N10"]\n', 'utf8')
    check('E8 tour 值原样透传（行内注释不污染取值）', plannerEngineOf(tmpTour) === 'tour')
    fs.rmSync(tmpTour, { force: true })
  }
}

/* ── F. 实跑 plan_route.py（与 planRoute 同款 argv，证明引擎值可用） ─ */
section('F. 实跑 plan_route.py（同款 argv）')
{
  const root = findWorkspaceRoot()
  if (!root) check('F0 有工作区可实跑', false)
  else {
    const run = await new Promise(resolve => {
      const child = spawn('python', ['tools/plan_route.py', '--config', 'config/robot.yaml',
        '--map', 'config/map.yaml', '--emit', 'json', '--check', '--quiet', '--engine', 'tracker-safe'], {
        cwd: root,
        env: {
          ...process.env,
          PYTHONPATH: path.join(root, 'src', 'dzbrain'),
          PYTHONIOENCODING: 'utf-8',
          PYTHONUNBUFFERED: '1',
        },
        windowsHide: true,
      })
      let out = ''
      let err = ''
      child.stdout.on('data', c => { out += c })
      child.stderr.on('data', c => { err += c })
      const timer = setTimeout(() => { try { child.kill() } catch { /* already gone */ } }, 120000)
      child.on('error', e => { clearTimeout(timer); resolve({ code: -1, out, err: String(e?.message ?? e) }) })
      child.on('close', code => { clearTimeout(timer); resolve({ code: code ?? 1, out, err }) })
    })
    check('F1 tracker-safe 引擎实跑 exit 0', run.code === 0,
      run.code === 0 ? '' : run.err.split('\n').filter(Boolean).slice(-2).join(' | '))
    const open = run.out.indexOf('{')
    let tokens = 0
    let fallback = null
    if (open >= 0) {
      try {
        const plan = JSON.parse(run.out.slice(open))?.plans?.[0]
        tokens = Array.isArray(plan?.tokens) ? plan.tokens.length : 0
        fallback = plan?.fallback ?? null
      } catch { /* JSON 断在截断处按失败算 */ }
    }
    check('F2 产出非空 token（JSON plans[0]）', tokens > 0, `${tokens} tokens${fallback ? '（fallback: ' + fallback + '）' : ''}`)
  }
}

console.log('')
console.log(`结果：${pass}/${pass + fail} ${fail === 0 ? '全部通过 ✅' : '存在失败 ❌'}`)
process.exit(fail === 0 ? 0 : 1)
