#!/usr/bin/env node
/* scripts/test-sim-overlay-ab.mjs —— t1 ② 长期守卫：「地图复杂就**叠加/乱跑**」渲染层重复应用的**判定性 A/B**
 *
 * 根因（已定位，本脚本就是不变量）：`renderMap()` 只重写 `#map.innerHTML`，**有意不碰** `#anim`
 *   （t10 为了不打断回放）。所以上一局的图层不会随重绘消失：
 *     · `#anim` 里上一局的回放折线；
 *     · `TRACE` 里的参考线 / 绿轨迹 / 车身位姿；
 *   它们会**叠在新棋盘上** —— 复杂图规划失败率高、连续换图、连续 422 时最刺眼，用户看到的就是
 *   "越点越乱、车身还在旧轨迹上跑"。修法：棋盘一改就走唯一咽喉
 *   `afterFieldChange() → invalidateStaleRun()`（清 TRACE + stopReplay + 清 #anim），
 *   422（plan-failed）分支同样先清旧层再回放。
 *
 * 判定性实验（可证伪）：从**同一份现版 sim.html** 取真实源码喂同一套 DOM 桩，走同一条
 *   「上一局图层在场 → 改棋盘 → 重绘」路径，数画面上到底有几条折线 / 几个车身位姿。
 *   旧臂必须复现叠加：Σ 折线 ≥ 2、车身位姿 ≥ 1、#anim 仍有旧折线；
 *   现版必须消除叠加：Σ 折线 = 1、车身位姿 = 0、TRACE=null、#anim 空。
 *
 * 两个旧臂，职责不同（都不是"编造的旧代码"）：
 *   ① **历史臂**：`git show HEAD:dsh-plugin-dzrobot-ssh/sim.html` —— 修复还没提交时的真历史，
 *      用来复现用户看到的叠加现象（HEAD == 工作区时自动跳过）。
 *   ② **变异体臂**：从**现版源码**只删掉两处清层（统一咽喉里的 `invalidateStaleRun()`、
 *      422 分支里的 `TRACE = null`），其余逐字节不动。这一臂是**因果反证**：渲染逻辑与现版
 *      同一份（逐字节校验），唯一差异是清层 ⇒ 叠加立刻复现，证明清层就是承重件。
 *
 * 跑法（任何目录都行，路径按脚本自身位置推导）：
 *     node scripts/test-sim-overlay-ab.mjs
 *   退出码：0 = 全绿；1 = 断言失败；2 = 环境前置不满足（找不到 sim.html / 变异体构造失败）。
 *
 * 写域：**只读**。只读 sim.html（+ 只读 `git show`），不写任何文件，不起服务、不跑 python、不连车。
 */
import { spawnSync } from 'node:child_process'
import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const HERE = path.dirname(fileURLToPath(import.meta.url))
const PLUGIN = path.resolve(HERE, '..')                 // dsh-plugin-dzrobot-ssh
const SIM_HTML = path.join(PLUGIN, 'sim.html')

const pass = { ok: 0, fail: 0 }
const check = (name, ok, note = '') => {
  if (ok) pass.ok += 1
  else pass.fail += 1
  console.log(`  ${ok ? 'PASS' : 'FAIL'} ${name}${note ? ' —— ' + note : ''}`)
}
const info = (msg) => console.log(`  INFO ${msg}`)
const h = (t) => console.log(`\n\u001b[36m${t}\u001b[0m`)

/* ── 源码提取工具 ─────────────────────────────────────────────────── */
const scriptOf = (html) => {
  const m = html.match(/<script>([\s\S]*)<\/script>/)
  return m ? m[1] : ''
}
/** 花括号配平：从 `at` 处的 `{` 起，返回配对 `}` 之后的下标。 */
function blockEnd(src, at) {
  const open = src.indexOf('{', at)
  if (open < 0) return -1
  let depth = 0
  for (let i = open; i < src.length; i += 1) {
    if (src[i] === '{') depth += 1
    else if (src[i] === '}') { depth -= 1; if (depth === 0) return i + 1 }
  }
  return -1
}
/** 按花括号配平提取一个顶层函数声明。 */
function extractFn(src, name) {
  const at = src.indexOf(`function ${name}(`)
  if (at < 0) return ''
  const end = blockEnd(src, at)
  return end < 0 ? '' : src.slice(at, end)
}
/** 提取"某条注册语句后面的函数体"（用于 resetAll 这类内联处理器）。 */
function handlerBodyOf(src, needle) {
  const at = src.indexOf(needle)
  if (at < 0) return ''
  const end = blockEnd(src, at)
  return end < 0 ? '' : src.slice(at, end)
}
/** 摘出**棋盘改动镜像输入**那条 input 监听（各臂真实注册语句）。
 *  { line, body }：line = 注册语句原文；body = 可直接求值的函数体。
 *  旧版是内联函数表达式；新版注册的是顶层函数 afterFieldChange（统一咽喉）。 */
function fieldInputHandler(script) {
  const hit = script.split('\n').find(l => /\$\(key\)\.addEventListener\('input'/.test(l))
  if (!hit) return { line: '', body: '' }
  const at = script.indexOf(hit)
  if (/afterFieldChange/.test(hit)) {
    return {
      line: hit.trim(),
      body: `${extractFn(script, 'invalidateStaleRun')}\n${extractFn(script, 'afterFieldChange')}\nafterFieldChange();`,
    }
  }
  const end = blockEnd(script, at)
  const open = script.indexOf('{', at)
  return { line: hit.trim(), body: end < 0 ? '' : script.slice(open + 1, end - 1) }
}
/** 统一咽喉 afterFieldChange **函数体**里是否真调了清层（不是函数定义里出现名字）。 */
function chokepointClears(script) {
  const fn = extractFn(script, 'afterFieldChange')
  return fn.length > 0 && (/invalidateStaleRun\s*\(/.test(fn) || /TRACE\s*=\s*null/.test(fn))
}
/** 该臂的「棋盘改动」路径是否带清层（覆盖两种注册形态：内联处理器 / 统一咽喉）。 */
function handlerClears(script, handler) {
  if (!handler.body) return false
  if (/afterFieldChange/.test(handler.line)) return chokepointClears(script)
  return /invalidateStaleRun\s*\(/.test(handler.body) || /TRACE\s*=\s*null/.test(handler.body)
}

/* ── DOM 桩（with+Proxy：has 恒真会劫持内置对象，故显式播种）────────── */
function makeCtx() {
  const els = {}
  const el = id => els[id] || (els[id] = {
    id, value: '', checked: true, disabled: false, innerHTML: '', textContent: '',
    style: {}, addEventListener() {}, getElementsByTagName: () => [{ innerHTML: '' }],
  })
  const state = {
    JSON, Math, String, Number, Boolean, isFinite, isNaN, parseFloat, parseInt,
    els,
    MAP_S: 68,
    FIELD_GRID_BLOCK: 0.35,
    CAR_LEN_PX: 27,
    TARGET_HALF: ['#4d7fff', '#e5484d'],
    FIELD_CELL_RE: /^[A-N](?:[1-9]|1[0-1])$/,
    FIELD: {
      ok: true, md5: 'c9bdb660637430a9dd00f518301090bc', source: 'config/map.yaml',
      grid: { cols: 14, rows: 11, blockM: 0.35, colLetters: 'ABCDEFGHIJKLMN' },
      start: 'N10', end: 'N5',
      mines: ['B1', 'G2'], obstacles: ['D1', 'F4'], bridges: ['K9', 'K5'], targets: ['B11', 'C11'],
      barrier: { colFrom: 'D', rowHi: 6, rowLo: 7 },
      drone: { takeoff: 'N8', land: 'M2' },
      bridgeEntryRedundancyM: 0.0875,
      route: ['N10', 'M10', 'M9', 'L9', 'K9'],
    },
    TRACE: { route: ['N10', 'M10', 'M9'], points: [[1, 1], [2, 2], [3, 3]], poses: [[1, 1, 0]] },
    // 渲染依赖桩（统一到两版同名）
    esc: s => String(s),
    cellCenter: () => [10, 10],
    tokenCell: token => String(token == null ? '' : token).trim()
      .replace(/^R:/i, '').replace(/[,:].*$/, '').replace(/\*$/, '').trim().toUpperCase(),
    colIdxOf: () => 3,
    tokenFlags: () => ({ cell: 'A1', pivot: false, shoot: false }),
    routePolyline: cells => (cells || []).map((_, i) => [i, i]),
    carPolygon: () => [[0, 0], [1, 0], [1, 1]],
    headVec: () => [1, 0],
    carYawTilt: () => 0,
    targetBoardRect: () => null,
    targetYawMode: () => '',
    fieldLists: () => ({ mines: ['B1', 'G2'], obstacles: ['D1', 'F4'], bridges: ['K9', 'K5'], targets: ['B11', 'C11'] }),
    refreshFieldHints() {},
    stopReplay() { state.calls.push('stopReplay') },
    calls: [],
    text(target, v) { if (target && typeof target === 'object') target.textContent = String(v) },
    $: el,
  }
  return new Proxy(state, {
    has: () => true,
    get: (t, k) => t[k],
    set: (t, k, v) => { t[k] = v; return true },
  })
}

/** 用某一版的 renderMap（已挂到 ctx）渲染一次，数出画面里的图层元素。 */
function renderWith(ctx) {
  ctx.renderMap()
  const mapHtml = String(ctx.$('map').innerHTML ?? '')
  const animHtml = String(ctx.$('anim').innerHTML ?? '')
  const count = (html, tag) => (html.match(new RegExp(`<${tag}[ >]`, 'g')) || []).length
  return {
    mapPolylines: count(mapHtml, 'polyline'),
    mapPosePaths: (mapHtml.match(/<path d="M [^"]*"[^>]*fill="#1f2933"/g) || []).length,
    animPolylines: count(animHtml, 'polyline'),
    mapHasSvg: mapHtml.includes('<svg'),
  }
}

/** 跑一臂：上一局图层在场 → 渲染 → 走该版真实的「棋盘改动」处理器 → 再渲染 → 数图层。 */
function runArm(script, handler) {
  const ctx = makeCtx()
  ctx.renderMap = new Function('ctx', `with (ctx) { ${extractFn(script, 'renderMap')}\n; return renderMap; }`)(ctx)
  // 上一局遗留：#anim 里还留着上一局的回放折线（只重写 #map 的实现不会清它）
  ctx.$('anim').innerHTML = '<svg viewBox="0 0 10 10"><polyline points="0,0 1,1 2,2"/></svg>'
  ctx.$('showRoute').checked = true
  const before = renderWith(ctx)
  new Function('ctx', `with (ctx) { ${handler.body} }`)(ctx)
  const after = renderWith(ctx)
  return { before, after, traceKept: ctx.TRACE !== null && ctx.TRACE !== undefined, anim: ctx.$('anim').innerHTML }
}

/* ── 0. 实验素材 ─────────────────────────────────────────────────── */
if (!fs.existsSync(SIM_HTML)) {
  console.error(`✗ 找不到 ${SIM_HTML}（脚本必须放在 dsh-plugin-dzrobot-ssh/scripts/ 下）`)
  process.exit(2)
}
const newHtml = fs.readFileSync(SIM_HTML, 'utf8')
const newScript = scriptOf(newHtml)

/** 变异体：只删掉两处清层动作，其余逐字节保持现版。 */
function preFixVariant(script) {
  let out = script.replace(/function afterFieldChange\(\)\s*\{[^}]*\}/,
    'function afterFieldChange() { refreshFieldHints(); renderMap(); }')
  const at = out.indexOf('if (problem.status === 422) {')
  if (at >= 0) {
    const end = blockEnd(out, at)
    if (end > at) {
      const block = out.slice(at, end).replace(/^[ \t]*TRACE = null;[ \t]*\r?\n/gm, '')
      out = out.slice(0, at) + block + out.slice(end)
    }
  }
  return out
}
const mutantScript = preFixVariant(newScript)

/* 历史臂：git HEAD（修复未提交时才存在且有意义） */
let headScript = ''
let headLabel = 'HEAD == 工作区（修复已提交）⇒ 无历史臂'
const gitTop = spawnSync('git', ['-C', PLUGIN, 'rev-parse', '--show-toplevel'], { encoding: 'utf8' })
if (gitTop.status === 0) {
  const repoRoot = gitTop.stdout.trim()
  const rel = path.relative(repoRoot, PLUGIN).replace(/\\/g, '/')
  const show = spawnSync('git', ['-C', PLUGIN, 'show', `HEAD:${rel}/sim.html`], {
    encoding: 'utf8', maxBuffer: 64 * 1024 * 1024,
  })
  if (show.status === 0) {
    const candidate = scriptOf(show.stdout)
    if (candidate && candidate !== newScript) {
      headScript = candidate
      headLabel = `git HEAD:${rel}/sim.html`
    } else if (candidate) {
      headLabel = 'HEAD 与工作区 sim.html 逐字节相同（修复已提交）⇒ 跳过历史臂'
    }
  } else {
    headLabel = `git show 失败（${String(show.stderr || '').trim().split('\n')[0] || '无 sim.html 历史'}）⇒ 跳过历史臂`
  }
} else {
  headLabel = '不在 git 仓库内 ⇒ 跳过历史臂'
}

const headHandler = headScript ? fieldInputHandler(headScript) : null
const mutantHandler = fieldInputHandler(mutantScript)
const newHandler = fieldInputHandler(newScript)

h('0. 实验素材')
console.log('  现版 script %d B ｜ 变异体 %d B ｜ 历史臂：%s', newScript.length, mutantScript.length, headLabel)
check('0.1 现版 script 与 renderMap 都提取到',
  newScript.length > 1000 && extractFn(newScript, 'renderMap').length > 500)
check('0.2 变异体的 renderMap 与现版**逐字节相同**（因果前提：两臂唯一差异是清层）',
  extractFn(mutantScript, 'renderMap') === extractFn(newScript, 'renderMap')
  && extractFn(newScript, 'renderMap').length > 500)
check('0.3 变异体确实没有清层动作（否则本实验证不了任何东西）',
  mutantHandler.body.length > 0 && !handlerClears(mutantScript, mutantHandler))
check('0.4 现版仍走统一咽喉（afterFieldChange 函数体里调 invalidateStaleRun + 清 TRACE）',
  handlerClears(newScript, newHandler) && chokepointClears(newScript)
  && /TRACE\s*=\s*null/.test(extractFn(newScript, 'invalidateStaleRun')))
if (headHandler) {
  check('0.5 历史臂里也没有清层动作（= 用户当时看到的实现）', !handlerClears(headScript, headHandler),
    JSON.stringify(headHandler.line))
} else {
  info(`历史臂跳过：${headLabel}`)
}

h('1. 棋盘改动那条 input 监听（各臂真实注册语句）')
if (headHandler) console.log('  历史臂注册:', JSON.stringify(headHandler.line))
console.log('  变异体/现版注册:', JSON.stringify(newHandler.line))
check('1.1 现版注册语句就是统一咽喉 afterFieldChange', /afterFieldChange/.test(newHandler.line))

/* ── 2. 判定性 A/B ────────────────────────────────────────────────── */
const show = (label, arm) => {
  console.log(`  ${label} 改动前：折线 ${arm.before.mapPolylines}（#map）+ ${arm.before.animPolylines}（#anim）· 车身位姿 ${arm.before.mapPosePaths}`)
  console.log(`  ${label} 改动后：折线 ${arm.after.mapPolylines}（#map）+ ${arm.after.animPolylines}（#anim）· 车身位姿 ${arm.after.mapPosePaths}`
    + ` · TRACE ${arm.traceKept ? '仍是上一局' : '已清 null'} · #anim ${arm.anim === '' ? '空' : '仍有旧折线'}`)
}
h('2. 同一场景（上一局图层在场 → 改棋盘 → 重绘）三臂对比')
const headArm = headScript ? runArm(headScript, headHandler) : null
const mutantArm = runArm(mutantScript, mutantHandler)
const newArm = runArm(newScript, newHandler)
if (headArm) show('历史臂', headArm)
show('变异体', mutantArm)
show('现版  ', newArm)

const totalOf = (arm) => arm.after.mapPolylines + arm.after.animPolylines
const sameBefore = (arm) => arm.before.mapPolylines === newArm.before.mapPolylines
  && arm.before.animPolylines === newArm.before.animPolylines
  && arm.before.mapPosePaths === newArm.before.mapPosePaths

check('2.1 实验前提：改动前各臂画面一致（都带着上一局图层）',
  mutantArm && sameBefore(mutantArm) && (!headArm || sameBefore(headArm)),
  `变异体 Σ=${totalOf(mutantArm)} · 现版 Σ=${totalOf(newArm)}`)
check('2.2 变异体复现「叠加」：改棋盘后仍 ≥2 条折线（上一局图层没走）', totalOf(mutantArm) >= 2, `Σ=${totalOf(mutantArm)}`)
check('2.3 变异体仍画上一局车身位姿（棋盘已变、位姿停在旧轨迹上）', mutantArm.after.mapPosePaths >= 1,
  `${mutantArm.after.mapPosePaths} 个`)
check('2.4 变异体 #anim 里上一局回放折线没清', mutantArm.anim !== '' && mutantArm.after.animPolylines >= 1)
check('2.5 因果对照：同一份 renderMap、唯一差异是清层，两臂画面必须不同（叠加 vs 干净）',
  totalOf(mutantArm) > totalOf(newArm), `变异体 Σ=${totalOf(mutantArm)} > 现版 Σ=${totalOf(newArm)}`)
if (headArm) {
  check('2.6 历史臂同样复现叠加（用户当时看到的现象）', totalOf(headArm) >= 2 && headArm.after.mapPosePaths >= 1,
    `Σ=${totalOf(headArm)} · 位姿 ${headArm.after.mapPosePaths}`)
} else {
  info('历史臂复现：跳过（见 0 段原因）')
}
check('2.7 现版消除叠加：改棋盘后 Σ 折线 = 1（只剩新棋盘的参考线）', totalOf(newArm) === 1, `Σ=${totalOf(newArm)}`)
check('2.8 现版不再画上一局车身位姿', newArm.after.mapPosePaths === 0, `${newArm.after.mapPosePaths} 个`)
check('2.9 现版 TRACE 置 null 且 #anim 清空', !newArm.traceKept && newArm.anim === '')

/* ── 3. 422（plan-failed）分支：叠加的第二条路径 ──────────────────── */
const seg422 = (src) => {
  const at = src.indexOf('if (problem.status === 422)')
  if (at < 0) return ''
  const end = blockEnd(src, at)
  return end < 0 ? src.slice(at, at + 900) : src.slice(at, end)
}
const new422 = seg422(newScript)
const mutant422 = seg422(mutantScript)
h('3. 422 plan-failed 分支（连续换图失败时"旧层永不清"的那条路）')
console.log('  现版分支含 TRACE = null ? %s ｜ 变异体 ? %s', /TRACE\s*=\s*null/.test(new422), /TRACE\s*=\s*null/.test(mutant422))
check('3.1 现版 422 分支清 TRACE 且**先清后画**（清 < renderMap < startReplay）',
  /TRACE\s*=\s*null/.test(new422)
  && new422.indexOf('TRACE = null') < new422.indexOf('renderMap')
  && new422.indexOf('TRACE = null') < new422.indexOf('startReplay'))
check('3.2 变异体的 422 分支没有清层（第二叠加路径在旧实现里同样缺）',
  mutant422 !== '' && !/TRACE\s*=\s*null/.test(mutant422))
if (headHandler) {
  const head422 = seg422(headScript)
  check('3.3 历史臂 422 分支也不清上一局 TRACE', head422 !== '' && !/TRACE\s*=\s*null/.test(head422))
} else {
  info('历史臂 422 分支：跳过')
}

/* ── 4. 排除两条候选根因：实例重复初始化 / 路线重复绘制 ───────────── */
h('4. 候选根因②「实例重复初始化 / 路线重复绘制」的判定（当前实现不成立）')
{
  const ctx = makeCtx()
  ctx.renderMap = new Function('ctx', `with (ctx) { ${extractFn(newScript, 'renderMap')}\n; return renderMap; }`)(ctx)
  ctx.$('showRoute').checked = true
  // 4.1：TRACE 在位、没有轨迹点 ⇒ 参考路线只该画 1 条（重复绘制就会是 2）
  ctx.TRACE = { route: ['N10', 'M10', 'M9', 'L9', 'K9'], points: [], poses: [] }
  ctx.renderMap()
  const polylines = (String(ctx.$('map').innerHTML).match(/<polyline[ >]/g) || []).length
  check('4.1 参考路线只画 1 条折线（无路线重复绘制）', polylines === 1, `${polylines} 条`)

  // 4.2/4.3：连点两次「回放」是否叠出第二条动画
  const timers = new Set()
  let seq = 0
  ctx.setInterval = () => { const id = ++seq; timers.add(id); return id }
  ctx.clearInterval = (id) => { timers.delete(id) }
  ctx.ANIM = { points: [[1, 1], [2, 2], [3, 3], [4, 4]], mode: 'traj', idx: 0, timer: null, playing: false, lastRecord: null }
  ctx.animBlockSize = () => ({ L: 10, W: 6 })
  ctx.animModeLabel = () => '本次轨迹'
  ctx.$('replay').disabled = false
  const animSrc = ['stopAnimTimer', 'stopReplay', 'animFrame', 'animPlay', 'animFinish']
    .map(name => extractFn(newScript, name)).join('\n')
  const anim = new Function('ctx', `with (ctx) { ${animSrc}\n; return { animPlay, animFrame }; }`)(ctx)
  anim.animPlay()
  const firstTimer = ctx.ANIM.timer
  anim.animPlay()                                  // 再点一次「回放」
  check('4.2 连点两次回放只留 1 个定时器（无重复实例）',
    timers.size === 1 && ctx.ANIM.timer !== null && ctx.ANIM.timer !== firstTimer, `活动定时器 ${timers.size}`)
  anim.animFrame(2)
  const animHtml = String(ctx.$('anim').innerHTML)
  check('4.3 动画层每帧只有 1 条 trail（覆盖式重写，不是追加）',
    (animHtml.match(/<polyline[ >]/g) || []).length === 1)
}

/* ── 5. 反重复注册 + 根因成立的前提 ───────────────────────────────── */
h('5. 反重复注册 / 根因前提')
const inputRegs = (newScript.match(/addEventListener\('input'/g) || []).length
check('5.1 场地输入只有 1 条 input 监听（多注册 = 一次改动应用两遍）', inputRegs === 1, `${inputRegs} 条`)
const resetAll = handlerBodyOf(newScript, `$('resetAll').addEventListener('click'`)
check('5.2 整图还原也走清层（invalidateStaleRun / afterFieldChange 至少其一）',
  /invalidateStaleRun\s*\(|afterFieldChange\s*\(/.test(resetAll))
const renderMapSrc = extractFn(newScript, 'renderMap')
check('5.3 根因前提：renderMap 不碰 #anim（所以清层必须显式做，不能在重绘里顺带完成）',
  renderMapSrc.length > 500 && !/\(\s*['"]anim['"]\s*\)/.test(renderMapSrc))

/* ── 汇总 ───────────────────────────────────────────────────────── */
const total = pass.ok + pass.fail
console.log('')
console.log('结论：叠加根因 =「棋盘变更 / 422 后旧图层（TRACE + #anim）不清」，由统一咽喉 invalidateStaleRun 修复；'
  + '旧臂复现、现版消除。')
console.log(`结果：${pass.ok}/${total} ${pass.fail === 0 ? '全部通过 ✅' : `，${pass.fail} 失败 ❌`}`)
process.exit(pass.fail === 0 ? 0 : 1)
