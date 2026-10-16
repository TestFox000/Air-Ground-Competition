// t5 独立取证：变异测试（破坏实验）。
// 目的：证明 36/36 里的每条新断言**真有牙**——把 sim.html 的被测实现改坏，对应断言必须 FAIL。
// 手段（官方测试文件 :226-242 注释里明确点名的方法）：只读读 sim.html → 内存里替换 body →
// 用官方同一个 DOM 桩跑真 renderMap()/真函数 → 对每个变异体重新求值断言的谓词。
// 只读，零仓库写入。
import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath, pathToFileURL } from 'node:url'

const HERE = path.dirname(fileURLToPath(import.meta.url))
const ROOT = 'D:/User/Desktop/比赛存档/v1/dzrobot_v2/dsh-plugin-dzrobot-ssh'
const HTML = path.join(ROOT, 'sim.html')
const SIMBENCH = path.join(ROOT, 'src/simbench.js')
const { fieldSpec, routeTokens } = await import(pathToFileURL(SIMBENCH).href)

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

const raw = fs.readFileSync(HTML, 'utf8')
const CAR_LEN_PX = 0.28 * 34 / 0.35
const CAR_WID_PX = 0.135 * 34 / 0.35

// ── 谓词：与官方 test-sim-map-render.mjs 的断言同口径 ────────────────────────
function predA(api) {
  const p = api.routePolyline(['A1', 'D1'])
  const q = api.routePolyline(['N11', 'N8'])
  const cells = field.route.map(t => String(t).trim().replace(/^R:/i, '').replace(/[,:].*$/, '')
    .replace(/\*$/, '').trim().toUpperCase()).filter(Boolean)
  const pts = api.routePolyline(cells)
  const expect = cells.length + cells.slice(1).reduce((n, cell, i) => {
    const prev = cells[i]
    const dc = Math.abs(prev.charCodeAt(0) - cell.charCodeAt(0))
    const dr = Math.abs(Number(prev.slice(1)) - Number(cell.slice(1)))
    if (dc > 1 && dr > 1) return n + 1
    if (dr > 1) return n + (dr - 1)
    if (dc > 1) return n + (dc - 1)
    return n
  }, 0)
  return (p.length === 4 && JSON.stringify(p) === JSON.stringify([[17, 17], [51, 17], [85, 17], [119, 17]]))
    && (q.length === 4 && q.every(t => t[0] === 459))
    && pts.length === field.route.length
    && pts.length === expect
}
const bbox = pts => [Math.max(...pts.map(p => p[0])) - Math.min(...pts.map(p => p[0])),
                     Math.max(...pts.map(p => p[1])) - Math.min(...pts.map(p => p[1]))]
function predB(api) {
  const car0 = api.carPolygon(459, 323, 0)
  const car90 = api.carPolygon(459, 323, Math.PI / 2)
  return car0.length === 4
    && Math.abs(bbox(car0)[0] - CAR_WID_PX) < 0.01 && Math.abs(bbox(car0)[1] - CAR_LEN_PX) < 0.01
    && Math.abs(bbox(car90)[0] - CAR_LEN_PX) < 0.01 && Math.abs(bbox(car90)[1] - CAR_WID_PX) < 0.01
    && JSON.stringify(car0.map(p => p.map(v => v.toFixed(2)))) !== JSON.stringify(car90.map(p => p.map(v => v.toFixed(2))))
}
function predC(api) {
  const h = api.targetBoardRect(['E5', 'F5'], '')
  const v = api.targetBoardRect(['E5', 'E6'], '')
  return JSON.stringify(h) === JSON.stringify([136, 136, 68, 34])
    && JSON.stringify(v) === JSON.stringify([136, 136, 34, 68])
}
function predTarget(svg) {
  const count = re => (svg.match(re) || []).length
  return count(/>靶板</g) === 1 && count(/>靶</g) === 0 && /stroke="#8a2f5f"/.test(svg)
}

// ── 变异体：对 sim.html 源码做一处"退化成旧/坏实现"的文本替换 ────────────────
const INTERP_BLOCK = [
  '    if (Math.abs(dc) > 1 && Math.abs(dr) > 1) {   // 对角：补一个切角过渡点',
  '      out.push([p[0], pc[1]]);',
  '    } else if (Math.abs(dr) > 1) {                // 纵向多格：逐格补点',
  '      var stepR = dr > 0 ? MAP_S : -MAP_S;',
  '      for (var k = 1; k < Math.abs(dr) / MAP_S; k += 1) out.push([pc[0], pc[1] + stepR * k]);',
  '    } else if (Math.abs(dc) > 1) {                // 横向多格：逐格补点',
  '      var stepC = dc > 0 ? MAP_S : -MAP_S;',
  '      for (var m = 1; m < Math.abs(dc) / MAP_S; m += 1) out.push([pc[0] + stepC * m, pc[1]]);',
  '    }',
].join('\n')

const MUTANTS = []
MUTANTS.push({
  id: 'M1', name: 'A 折线：插值段短路成"直线连格心"',
  pred: predA,
  from: INTERP_BLOCK,
  to: '    if (false) { }',
})
MUTANTS.push({
  id: 'M2', name: 'A 折线：每段重复 push 当前点（点数倒退）',
  pred: predA,
  from: '    out.push(p);\n  }\n  return out;\n}',
  to: '    out.push(p); out.push(p);\n  }\n  return out;\n}',
})
MUTANTS.push({
  id: 'M3', name: 'B 位姿：车身尺寸退回整格方块 34×34',
  pred: predB,
  from: '  var L = CAR_LEN_PX / 2, W = CAR_WID_PX / 2;',
  to: '  var L = MAP_S / 2, W = MAP_S / 2;',
})
MUTANTS.push({
  id: 'M4', name: 'B 位姿：忽略 yawCode（尺寸对但其实没转）',
  pred: predB,
  from: '  var th = yawCode + Math.PI / 2;                  // 代码 yaw → 屏幕角（与官方渲染器一致）',
  to: '  var th = 0 + Math.PI / 2;',
})
MUTANTS.push({
  id: 'M5', name: 'C 靶朝向：auto 判定恒为横向（不区分横竖）',
  pred: predC,
  from: "  if (!mode) mode = (x1 - x0) >= (y1 - y0) ? 'h' : 'v';",
  to: "  if (!mode) mode = 'h';",
})
MUTANTS.push({
  id: 'M6', name: '靶板重叠 bug：恢复 t28 前"每格一个靶字"老实现',
  pred: (api, svg) => predTarget(svg),
  needsSvg: true,
  from: '  // ★ t28：靶板按"一块并板"画（横/竖朝向可选），不再每格一个「靶」字\n  (function () {',
  to: '  FIELD.targets.forEach(function (cell) {\n    var p = cellCenter(cell); if (!p) return;\n    svg.push(text(p[0], p[1] + 4, \'靶\', 11, \'#8a2f5f\', \'bold\', \'middle\'));\n  });\n  (function () {',
})

function buildApi(bodySrc) {
  const body = bodySrc.match(/<script>([\s\S]*?)<\/script>/)[1]
  const documentStub = { getElementById: el, querySelectorAll() { return [] }, addEventListener() {} }
  const windowStub = { addEventListener() {} }
  const fetchStub = () => Promise.resolve({ json: () => Promise.resolve({}), text: () => Promise.resolve('') })
  return new Function('document', 'window', 'fetch',
    'return (function(){' + body + '\n; return { renderMap: renderMap, setField: function(f){ FIELD = f; },'
    + ' cellCenter: cellCenter, fieldFromForm: fieldFromForm, routePolyline: routePolyline,'
    + ' carPolygon: carPolygon, targetBoardRect: targetBoardRect }; })();'
  )(documentStub, windowStub, fetchStub)
}

let bad = 0
const lines = []

function evalPred(src, m) {
  const api = buildApi(src)
  field.route = routeTokens()
  api.setField(field)
  if (m.needsSvg) { api.renderMap(); return { ok: m.pred(api, el('map').innerHTML) } }
  return { ok: m.pred(api) }
}

for (const m of MUTANTS) {
  // 基线：未变异实现，谓词必须 PASS
  let base
  try { base = evalPred(raw, m).ok } catch (e) { base = 'threw: ' + e.message }
  const baseOk = base === true
  lines.push('  ' + (baseOk ? 'OK   ' : 'BAD  ') + m.id + ' 基线（未变异）          谓词=' + (baseOk ? 'PASS' : base))
  if (!baseOk) bad += 1

  // 变异：改坏实现，谓词必须 FAIL
  if (!raw.includes(m.from)) {
    lines.push('  BAD  ' + m.id + ' 变异                   补丁锚点未命中')
    bad += 1
    continue
  }
  let mut
  let note = ''
  try {
    mut = evalPred(raw.replace(m.from, m.to), m).ok
  } catch (e) {
    mut = false
    note = 'threw: ' + e.message
  }
  const caught = mut === false
  lines.push('  ' + (caught ? 'OK   ' : 'BAD  ') + m.id + ' 变异：' + m.name + '  → ' + (caught ? '断言成功 FAIL ⇒ 有牙' : '断言未 FAIL ⇒ 空转') + (note ? ' [' + note + ']' : ''))
  if (!caught) bad += 1
}

console.log(lines.join('\n'))
console.log('\n变异测试：' + (bad === 0 ? '全部符合预期 ✓（6 条基线 PASS + 6 个变异体全部被断言抓住）' : bad + ' 项不符合预期 ✗'))
process.exit(bad ? 1 : 0)
