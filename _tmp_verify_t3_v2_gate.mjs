/**
 * _tmp_verify_t3_v2_gate.mjs — verifier2 的独立门禁（只读；不写仓库，只在 %TEMP% 落一个探针副本）
 *
 * 做四件事，全部自己重算，不引用 t1/t2 的任何自报数字：
 *   ① 复刻官方 DOM 桩 + 真 renderMap()，数 SVG 里 `>靶<` / `>靶板<` / stroke / 朝向标注；
 *      并在内存里把 per-cell 靶字循环放回去，证明当前断言不是空转（F1 的反向对照）。
 *   ② 独立复算三组实现：routePolyline（逐格补点，用"每步只能一格"作独立判据）、
 *      carPolygon（车头朝向 vs 真实 traj.csv 的位移方向 —— 口径一致性用数据说话）、
 *      targetBoardRect（auto/h/v 三态 + 渲染层标注切换）。
 *   ③ 透传链：把**真** simbench.js 的 sanitizeParams 抽到 %TEMP% 副本里 import 出来实测。
 *   ④ 现状描述（不作为判据）：#targetYaw 事件绑定。
 */
import fs from 'node:fs'
import path from 'node:path'
import os from 'node:os'
import { fileURLToPath, pathToFileURL } from 'node:url'

const HERE = path.dirname(fileURLToPath(import.meta.url))
const ROOT = path.join(HERE, 'dsh-plugin-dzrobot-ssh')
const HTML = path.join(ROOT, 'sim.html')
const SIMBENCH = path.join(ROOT, 'src', 'simbench.js')
const { fieldSpec, routeTokens, sanitizeParams } = await import(pathToFileURL(SIMBENCH).href)

let bad = 0
let total = 0
const chk = (name, ok, got) => {
  total += 1
  if (!ok) bad += 1
  console.log((ok ? '  PASS ' : '  FAIL ') + name + (ok ? '' : '   → ' + JSON.stringify(got)))
}
const info = s => console.log('  ···· ' + s)

console.log('=== ⓪ 源文件现状 ===')
const rawHtml = fs.readFileSync(HTML, 'utf8')
const LEGACY = `  FIELD.targets.forEach(function (cell) {
    var p = cellCenter(cell); if (!p) return;
    svg.push(text(p[0], p[1] + 4, '靶', 11, '#8a2f5f', 'bold', 'middle'));
  });
`
info('sim.html 行数 = ' + rawHtml.split('\n').length)
info('遗留 per-cell 靶字循环仍在源码里 = ' + rawHtml.includes(LEGACY))
info('传说 legend 项「靶（跨格板）」仍在 = ' + rawHtml.includes('靶（跨格板）'))

// ── DOM 桩（与 scripts/test-sim-map-render.mjs 同一套，逐字复用口径）──────────
const els = {}
function el(id) {
  if (!els[id]) {
    els[id] = {
      id, value: '', checked: id === 'showRoute', innerHTML: '', textContent: '', className: '',
      style: {}, parentNode: null, addEventListener() {},
      getElementsByTagName() { return [{ innerHTML: '' }] },
      querySelectorAll() { return [] },
      getAttribute() { return '' },
    }
  }
  return els[id]
}
const field = fieldSpec()
el('mines').value = field.mines.join(' ')
el('obstacles').value = field.obstacles.join(' ')
el('bridges').value = field.bridges.join(' ')
field.route = routeTokens()

function makeApi(bodySrc, opts = {}) {
  const body = bodySrc.match(/<script>([\s\S]*?)<\/script>/)[1]
  const doc = { getElementById: opts.el || el, querySelectorAll() { return [] }, addEventListener() {} }
  const win = { addEventListener() {} }
  const fet = opts.fetch || (() => Promise.resolve({ json: () => Promise.resolve({}), text: () => Promise.resolve('') }))
  return new Function('document', 'window', 'fetch', 'location',
    'return (function(){' + body + '\n; return { renderMap: renderMap, setField: function(f){ FIELD = f; },'
    + ' cellCenter: cellCenter, codeToScreen: codeToScreen, routePolyline: routePolyline,'
    + ' carPolygon: carPolygon, targetBoardRect: targetBoardRect, targetYawMode: targetYawMode,'
    + ' loadTrace: loadTrace, getTrace: function(){ return TRACE; } }; })();'
  )(doc, win, fet, { hash: '' })
}

console.log('\n=== ① F1/F4：靶板重叠 bug（真 renderMap()） ===')
const api = makeApi(rawHtml)
api.setField(field)
api.renderMap()
const svg = el('map').innerHTML
const count = (s, re) => (s.match(re) || []).length
const cTargetBoard = count(svg, />靶板</g)
const cTarget = count(svg, />靶</g)
const cStroke = count(svg, /stroke="#8a2f5f"/g)
const cYawLabel = count(svg, /横向|竖向/g)
info(`>靶板<=${cTargetBoard}  >靶<=${cTarget}  stroke#8a2f5f=${cStroke}  横向|竖向=${cYawLabel}`)
// 语义依据：text() 产出 `>` + esc(body) + `</text>`。
//   '靶板' → `>靶板</text>`，被 />靶板</ 匹配、**不被** />靶</ 匹配（「靶」后是「板」不是「<」）。
//   所以删除 per-cell 靶字后，`>靶<` 必须为 0；期望 1 是把「靶板」错当成能匹配 />靶</ 的误判。
chk('F1 靶板：>靶板< 恰好 1 次', cTargetBoard === 1, cTargetBoard)
chk('F1 靶板：>靶< 恰好 0 次（text() 产出 >靶板</text>，不匹配 />靶</）', cTarget === 0, cTarget)
chk('F1 靶板：并板描边 stroke="#8a2f5f" 在位且唯一', cStroke === 1, cStroke)
chk('F1 靶板：朝向标注（横向/竖向）唯一', cYawLabel === 1, cYawLabel)

// 反向对照（内存替换）：把遗留循环放回去 → 当前断言必须变红，否则断言是空转
const svgLegacy = (() => {
  els.map = undefined
  const a2 = makeApi(rawHtml.replace('  // ★ t28：靶板按"一块并板"画', LEGACY + '  // ★ t28：靶板按"一块并板"画'))
  a2.setField(field)
  a2.renderMap()
  return el('map').innerHTML
})()
const legacyTarget = count(svgLegacy, />靶</g)
info(`反向对照：放回遗留循环后 >靶<=${legacyTarget}（应为 2）`)
chk('F1 反向对照：放回遗留循环 → >靶< 变 2 ⇒ 当前 AssertionOf(===0) 有牙',
  legacyTarget === 2 && !(legacyTarget === 0), legacyTarget)

console.log('\n=== ②A 路径折线 routePolyline（独立判据：每步只允许跨一格） ===')
const cellsOf = pts => pts.map(p => [p[0] / 34 + 0.5, p[1] / 34 + 0.5])
const stepOk = (pts) => {
  for (let i = 1; i < pts.length; i += 1) {
    const dc = Math.abs(pts[i][0] / 34 - pts[i - 1][0] / 34)     // 列方向格数
    const dr = Math.abs(pts[i][1] / 34 - pts[i - 1][1] / 34)     // 行方向格数
    if (dc > 1.0001 || dr > 1.0001) return false                 // 任何一步跨 >1 格 = 没有补点
    if (dc < 1e-6 && dr < 1e-6) return false                     // 原地重复点
  }
  return true
}
const polyAD = api.routePolyline(['A1', 'D1'])
const polyNN = api.routePolyline(['N11', 'N8'])
info('routePolyline([A1,D1]) = ' + JSON.stringify(polyAD))
info('routePolyline([N11,N8]) = ' + JSON.stringify(polyNN))
chk('A1 折线：A1→D1（跨 3 列）每一步只跨一格（= 逐格补 2 个中间格心）', stepOk(polyAD), polyAD)
chk('A1 折线：A1→D1 首末点仍是格心 A1(17,17) / D1(119,17)',
  JSON.stringify(polyAD[0]) === '[17,17]' && JSON.stringify(polyAD[polyAD.length - 1]) === '[119,17]', polyAD)
chk('A2 折线：N11→N8（跨 3 行）每一步只跨一格，且 x 恒为 459（列不变）',
  stepOk(polyNN) && polyNN.every(p => p[0] === 459), polyNN)
// 独立判据：折线点集必须覆盖每个途经格心（cellCenter 同一口径）
const cols = 'ABCDEFGHIJKLMN'
const routeCells = field.route.map(t => String(t).trim().replace(/^R:/i, '').replace(/[,:].*$/, '')
  .replace(/\*$/, '').trim().toUpperCase()).filter(Boolean)
const polyRoute = api.routePolyline(routeCells)
const centers = routeCells.map(c => api.cellCenter(c)).filter(Boolean)
const ptSet = new Set(polyRoute.map(p => p[0] + ',' + p[1]))
const missing = centers.filter(p => !ptSet.has(p[0] + ',' + p[1]))
info(`真实路线 token=${field.route.length} 格心=${centers.length} 折线点=${polyRoute.length} 缺格心=${missing.length}`)
chk('A3 折线：真实路线的每个格心都在折线点集里（口径与 cellCenter 一致）', missing.length === 0, missing)
chk('A4 折线：真实路线（54 token 全逐格正交）每步 ≤1 格且不原地重复', stepOk(polyRoute), polyRoute.length)
// 对角分支（当前真实路线 0 次触发）——如实报出实现与注释不符
const polyDiag = api.routePolyline(['A1', 'C3'])
info('routePolyline([A1,C3]) 对角分支产物 = ' + JSON.stringify(polyDiag)
  + '（注释写「切角过渡点」，实际是直角折点 [p.x, pc.y]；真实路线该分支 0 次触发）')
const diagTriggered = routeCells.slice(1).filter((c, i) => {
  const p = routeCells[i]
  return Math.abs(cols.indexOf(p[0]) - cols.indexOf(c[0])) > 1
    && Math.abs(Number(p.slice(1)) - Number(c.slice(1))) > 1
}).length
info('对角分支在真实路线上的触发次数 = ' + diagTriggered)

console.log('\n=== ②B 小车位姿 carPolygon（尺寸 + 车头方向） ===')
const CAR_LEN_PX = 0.28 * 34 / 0.35
const CAR_WID_PX = 0.135 * 34 / 0.35
const bbox = pts => [Math.max(...pts.map(p => p[0])) - Math.min(...pts.map(p => p[0])),
                     Math.max(...pts.map(p => p[1])) - Math.min(...pts.map(p => p[1]))]
const car0 = api.carPolygon(459, 323, 0)
const car90 = api.carPolygon(459, 323, Math.PI / 2)
const edge = (p, q) => Math.hypot(p[0] - q[0], p[1] - q[1])
info('carPolygon(459,323,0)  = ' + JSON.stringify(car0.map(p => p.map(v => +v.toFixed(2)))))
info(`车长/车宽像素 = ${CAR_LEN_PX.toFixed(3)} / ${CAR_WID_PX.toFixed(3)}（0.28m×0.135m ÷0.35m ×34px）`)
chk('B1 位姿：车体是对边为车长/车宽的矩形（对边等长 + 邻边 == 长宽）',
  Math.abs(edge(car0[0], car0[1]) - CAR_WID_PX) < 0.01 && Math.abs(edge(car0[1], car0[2]) - CAR_LEN_PX) < 0.01
  && Math.abs(edge(car0[2], car0[3]) - CAR_WID_PX) < 0.01 && Math.abs(edge(car0[3], car0[0]) - CAR_LEN_PX) < 0.01,
  car0.map(p => edge(p, car0[(car0.indexOf(p) + 1) % 4])))
chk('B2 位姿：矩形中心 = 传入的 (sx,sy)（与 cellCenter / codeToScreen 同一锚点）',
  Math.abs((car0[0][0] + car0[2][0]) / 2 - 459) < 1e-6 && Math.abs((car0[0][1] + car0[2][1]) / 2 - 323) < 1e-6,
  [(car0[0][0] + car0[2][0]) / 2, (car0[0][1] + car0[2][1]) / 2])
chk('B3 位姿：yaw 转 90° 后长宽互换（顶点真的动了）',
  Math.abs(bbox(car90)[0] - CAR_LEN_PX) < 0.01 && Math.abs(bbox(car90)[1] - CAR_WID_PX) < 0.01,
  bbox(car90))

// ★ 口径一致性（数据驱动）：真 traj.csv 的位移方向 vs carPolygon 的**车头边中线**方向
const trajCandidates = ['artifacts/sim/1792055476450_t2yaw-v/traj.csv',
                        'artifacts/sim/t12_rerun_0.35/traj.csv'].map(p => path.join(HERE, p))
const trajPath = trajCandidates.find(p => fs.existsSync(p))
if (!trajPath) {
  info('找不到 traj.csv，跳过数据驱动的车头方向核对')
} else {
  // carPolygon 前两个顶点 = 车头那一条边；其中心 - 车心 = 车头方向（不依赖内部公式）
  const headOfPoly = (pts, c) => {
    const mid = [(pts[0][0] + pts[1][0]) / 2, (pts[0][1] + pts[1][1]) / 2]
    return [mid[0] - c[0], mid[1] - c[1]]
  }
  const h0 = headOfPoly(car0, [459, 323])
  info('carPolygon 车头边中线方向（yaw=0）= ' + JSON.stringify(h0.map(v => +v.toFixed(2))) + '（SVG：+y = 屏幕下）')
  const lines = fs.readFileSync(trajPath, 'utf8').split(/\r?\n/).filter(Boolean)
  const head = lines[0].split(',').map(s => s.trim())
  const ix = head.indexOf('truth_x'); const iy = head.indexOf('truth_y'); const iw = head.indexOf('truth_yaw')
  const rows = lines.slice(1).map(l => l.split(',')).filter(r => r.length > Math.max(ix, iy, iw))
  const scr = r => [(13.5 - Number(r[iy]) / 0.35) * 34, (10.5 - Number(r[ix]) / 0.35) * 34]
  let n = 0; let sum = 0; let pos = 0; const angles = []
  for (let i = 0; i + 1 < rows.length; i += 1) {
    const a = scr(rows[i]); const b = scr(rows[i + 1])
    const dx = b[0] - a[0]; const dy = b[1] - a[1]
    const d = Math.hypot(dx, dy)
    if (d < 0.5) continue                       // 原地 PIVOT / 静止拍不计
    const yaw = Number(rows[i][iw]); if (!isFinite(yaw)) continue
    const hp = headOfPoly(api.carPolygon(a[0], a[1], yaw), a)   // 用被测实现算车头方向
    const hn = Math.hypot(...hp)
    const c = (dx * hp[0] + dy * hp[1]) / (d * hn)
    n += 1; sum += c
    if (c > 0) pos += 1
    angles.push(Math.acos(Math.max(-1, Math.min(1, c))) * 180 / Math.PI)
  }
  angles.sort((x, y) => x - y)
  info(`traj=${path.relative(HERE, trajPath)} 参与核对 ${n} 拍（位移>0.5px）`)
  info(`carPolygon 车头方向 · 平均 cos=${(sum / n).toFixed(3)}（同向占 ${(100 * pos / n).toFixed(1)}%）`
    + ` · 夹角中位数 ${angles[Math.floor(angles.length / 2)].toFixed(1)}°`)
  chk('B4 位姿：carPolygon 的车头方向与真实 traj.csv 的位移方向一致（平均 cos > 0.9）',
    sum / n > 0.9, { meanCos: +(sum / n).toFixed(3), sameDirPct: +(100 * pos / n).toFixed(1) })
}

// ★ 口径一致性（静态）：cellCenter(cell) == codeToScreen(该格心的代码坐标)，154 格全查
{
  const colLetters = field.grid.colLetters
  let worst = 0; let worstCell = ''
  for (let r = 1; r <= field.grid.rows; r += 1) {
    for (let ci = 0; ci < colLetters.length; ci += 1) {
      const cell = colLetters[ci] + r
      const cx = (10.5 - (r - 1)) * 0.35          // 官方 geometry：行 1 → 3.675
      const cy = (13.5 - ci) * 0.35               // 官方 geometry：A 列 → 4.725
      const a = api.cellCenter(cell)
      const b = api.codeToScreen(cx, cy)
      const d = Math.max(Math.abs(a[0] - b[0]), Math.abs(a[1] - b[1]))
      if (d > worst) { worst = d; worstCell = cell + ' cellCenter=' + JSON.stringify(a) + ' codeToScreen=' + JSON.stringify(b) }
    }
  }
  info('cellCenter vs codeToScreen 全 154 格最大偏差 = ' + worst.toFixed(4) + ' px  ' + (worst > 0.001 ? worstCell : ''))
  chk('B8 位姿：codeToScreen 与 cellCenter 是同一口径（154 格最大偏差 < 0.01 px）', worst < 0.01, worst)
}

console.log('\n=== ②B2 位姿标注整体落地（合成 traj.csv 跑真 loadTrace+renderMap） ===')
{
  const rows = []
  for (let i = 0; i < 400; i += 1) {
    const x = 0.35 + (i / 399) * 3.0
    rows.push([i + 1, x.toFixed(4), '4.9', '1.5708', '0.30'].join(','))
  }
  const CSV = 'step,truth_x,truth_y,truth_yaw,truth_v\n' + rows.join('\n')
  const tEls = {}
  const tEl = id => (tEls[id] = tEls[id] || el(id))
  const tFetch = url => String(url || '').includes('traj.csv')
    ? Promise.resolve({ ok: true, text: () => Promise.resolve(CSV) })
    : Promise.resolve({ json: () => Promise.resolve({}), text: () => Promise.resolve('') })
  const tApi = makeApi(rawHtml, { el: tEl, fetch: tFetch })
  tApi.setField(field)
  await tApi.loadTrace({ id: 'synthetic', route: field.route })
  const tSvg = tEl('map').innerHTML
  const trace = tApi.getTrace()
  const poses = trace && trace.poses
  const lines = CSV.split('\n')
  // 独立判据：每个位姿都必须能对齐到 CSV 里的**某一行真值**经 codeToScreen 的换算
  const matched = (poses || []).map(p => {
    for (let li = 1; li < lines.length; li += 1) {
      const c = tApi.codeToScreen(Number(lines[li].split(',')[1]), Number(lines[li].split(',')[2]))
      if (Math.abs(c[0] - p[0]) < 1e-6 && Math.abs(c[1] - p[1]) < 1e-6) return li
    }
    return null
  })
  info('位姿对齐到的 CSV 行号 = ' + JSON.stringify(matched) + '（首拍必画 ⇒ 必含 1）')
  chk('B5 位姿：每个位姿都 = 某拍真值经 codeToScreen 的换算（逐点对齐 CSV 原行，含首拍）',
    Array.isArray(poses) && poses.length >= 2 && matched.every(x => x !== null) && matched[0] === 1,
    { poseCount: poses && poses.length, matchedLines: matched })
  chk('B5b 位姿：loadTrace 的轨迹点 = 每 4 拍一点（401 行 → 100 点）',
    trace && trace.points.length === 100, trace && trace.points.length)
  chk('B6 位姿：渲染出车身 path + 车头箭头 + 「起点位姿」标注',
    count(tSvg, /fill="#1f2933"/g) > 0 && count(tSvg, /<line[^>]*stroke="#0b1727"/g) > 0
    && count(tSvg, />起点位姿</g) === 1,
    { bodies: count(tSvg, /fill="#1f2933"/g), arrows: count(tSvg, /<line[^>]*stroke="#0b1727"/g), label: count(tSvg, />起点位姿</g) })
  // 箭头终点方向 vs 首拍车头方向（真值）
  const tip = (tSvg.match(/<line x1="([\d.]+)" y1="([\d.]+)" x2="([\d.]+)" y2="([\d.]+)" stroke="#0b1727"/) || [])
  if (tip.length === 5) {
    const v = [+tip[3] - +tip[1], +tip[4] - +tip[2]]
    const u = [Math.cos(poses[0][2] + Math.PI / 2), Math.sin(poses[0][2] + Math.PI / 2)]
    const cos = (v[0] * u[0] + v[1] * u[1]) / Math.hypot(...v)
    info(`首拍箭头向量 ${JSON.stringify(v.map(x => +x.toFixed(2)))} · carPolygon 车头 ${JSON.stringify(u.map(x => +x.toFixed(2)))} · cos=${cos.toFixed(3)}`)
    chk('B7 位姿：SVG 里的车头箭头与 carPolygon 的车头方向一致', cos > 0.99, cos)
  }
}

console.log('\n=== ②C 靶板朝向 targetBoardRect + 渲染标注切换 ===')
const boardAutoH = api.targetBoardRect(['E5', 'F5'], '')
const boardAutoV = api.targetBoardRect(['E5', 'E6'], '')
info('auto E5+F5 = ' + JSON.stringify(boardAutoH) + ' · auto E5+E6 = ' + JSON.stringify(boardAutoV))
info('显式 h/v（E5+F5） = ' + JSON.stringify(api.targetBoardRect(['E5', 'F5'], 'h'))
  + ' / ' + JSON.stringify(api.targetBoardRect(['E5', 'F5'], 'v')))
chk('C1 靶朝向：auto 由两格几何判定 —— 左右相邻 E5+F5 → 68×34 横向',
  JSON.stringify(boardAutoH) === JSON.stringify([136, 136, 68, 34]), boardAutoH)
chk('C2 靶朝向：auto 由两格几何判定 —— 上下相邻 E5+E6 → 34×68 竖向',
  JSON.stringify(boardAutoV) === JSON.stringify([136, 136, 34, 68]), boardAutoV)
const labelOf = s => (s.match(/<text[^>]*>(横向|竖向)[^<]*<\/text>/) || ['', ''])[1]
const renderWith = v => { el('targetYaw').value = v; api.renderMap(); return el('map').innerHTML }
const svgV = renderWith('v')
const svgH = renderWith('h')
const svgX = renderWith('x')
chk('C3 靶朝向：select 设 v → 标注「竖向」且无「横向」', labelOf(svgV) === '竖向' && !/横向/.test(svgV), labelOf(svgV))
chk('C4 靶朝向：select 设 h → 标注「横向」且无「竖向」', labelOf(svgH) === '横向' && !/竖向/.test(svgH), labelOf(svgH))
chk('C5 靶朝向：非法值回落「跟随地图」（= auto 横向）', labelOf(svgX) === '横向', labelOf(svgX))
chk('C6 靶朝向：h/v 只改标注不改矩形尺寸（t28 语义：尺寸由两格几何决定）',
  JSON.stringify(api.targetBoardRect(['E5', 'F5'], 'h')) === JSON.stringify(api.targetBoardRect(['E5', 'F5'], 'v')), 'h/v 尺寸不同')
chk('C7 靶朝向：宿主 fieldSpec() 的 target.yaw 与页面 auto 判定一致（E5+F5 → h）',
  field.target && field.target.yaw === 'h'
  && api.targetBoardRect(field.targets, '')[2] >= api.targetBoardRect(field.targets, '')[3], field.target)

console.log('\n=== ③ 透传链：真 sanitizeParams（直接从工作区模块 import） ===')
const probe = { sanitizeParams }
const cases = [
  ['h', 'h', 'body.targetYaw="h"'],
  ['v', 'v', 'body.targetYaw="v"'],
  ['', null, 'body.targetYaw=""'],
  ['H', null, 'body.targetYaw="H"（大写）'],
  ['x', null, 'body.targetYaw="x"'],
  ['h;rm -rf /', null, '注入串'],
  [undefined, null, '不带 targetYaw 键'],
  [0, null, '数字 0'],
  [null, null, 'null'],
]
let chainBad = 0
for (const [input, expect, label] of cases) {
  const body = {}
  if (input !== undefined) body.targetYaw = input
  const out = probe.sanitizeParams(body)
  const ok = out.targetYaw === expect
  if (!ok) chainBad += 1
  console.log((ok ? '  PASS ' : '  FAIL ') + `sanitizeParams ${label} → targetYaw=${JSON.stringify(out.targetYaw)}（期望 ${JSON.stringify(expect)}）`)
}
total += cases.length; bad += chainBad
const ret = probe.sanitizeParams({ targetYaw: 'v' })
chk('③ 返回值：sanitizeParams 的返回对象带 targetYaw 键（:351）',
  Object.prototype.hasOwnProperty.call(ret, 'targetYaw') && ret.targetYaw === 'v', Object.keys(ret))
chk('③ 返回值：返回对象的三类点位/参数键与改动前同形（overrides/inject/label/maxSteps/seed/field）',
  ['overrides', 'inject', 'label', 'maxSteps', 'seed', 'targetYaw', 'field'].every(k => k in ret), Object.keys(ret))
const src = fs.readFileSync(SIMBENCH, 'utf8')
chk('③ 落账：record 对象里带 targetYaw（simbench.js:470）', /^\s*targetYaw: params\.targetYaw,$/m.test(src), 'regex miss')
chk('③ 落账：appendMeta 写 run_meta.jsonl，且 record 经 appendMeta 落盘（≥2 处调用）',
  /appendFileSync\(META_FILE/.test(src) && (src.match(/appendMeta\(record\)/g) || []).length >= 2, (src.match(/appendMeta\(record\)/g) || []).length)
const metaFile = path.join(HERE, 'artifacts', 'sim', 'run_meta.jsonl')
if (fs.existsSync(metaFile)) {
  const metaLines = fs.readFileSync(metaFile, 'utf8').split(/\r?\n/).filter(Boolean)
  const withYaw = metaLines.map(l => { try { return JSON.parse(l) } catch { return null } }).filter(r => r && 'targetYaw' in r)
  const values = withYaw.map(r => JSON.stringify(r.targetYaw))
  info(`已落盘 run_meta.jsonl 行数=${metaLines.length}，带 targetYaw 的行=${withYaw.length}，取值集合=${JSON.stringify([...new Set(values)])}`)
  chk('③ 落账：run_meta.jsonl 里确有 targetYaw 字段且非法值落 null（历史实跑产物）',
    withYaw.length > 0 && values.includes('"v"') && values.includes('"h"') && values.includes('null'), [...new Set(values)])
  const sample = withYaw.find(r => r.targetYaw === 'v')
  chk('③ 落账：名单历史可原样回读（listHistory 读同一文件）', Boolean(sample && sample.id), sample && sample.id)
} else {
  info('没有 run_meta.jsonl，落账仅静态核验')
}

console.log('\n=== ④ 现状描述（★ 不作为本卡判据；F3 归 t6） ===')
const hasChangeListener = /\$\('targetYaw'\)\.addEventListener/.test(rawHtml)
const inInputArray = /\['mines', 'obstacles', 'bridges', 'targetYaw'\]/.test(rawHtml)
info('#targetYaw 绑了 change 监听 = ' + hasChangeListener)
info('#targetYaw 被塞进 input 数组 = ' + inInputArray)
info('data-reset="targetYaw" 按钮存在 = ' + rawHtml.includes('data-reset="targetYaw"')
  + ' ⇒ reset 走 $(key).value = (FIELD[key]||[]).join(" ")，对 select 设成 "" = 默认项「跟随地图」，行为正确')
info('结论：' + (hasChangeListener
  ? 'change 监听已在位（本卡窗口内 t1 落地）—— F3 的两部分都到位了'
  : '改 select 不会即时重绘（要等别的控件触发 renderMap）；渲染函数本身正确 —— F3 遗留'))

console.log('\n=== 汇总 ===')
console.log(`独立门禁：${total - bad}/${total} 通过` + (bad ? `（${bad} 条 FAIL）` : ''))
process.exit(bad ? 1 : 0)
