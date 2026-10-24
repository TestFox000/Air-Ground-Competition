#!/usr/bin/env node
/* scripts/test-sim-scout.mjs —— 仿真台「步进侦察 + 十批步进成果」守卫（T28/T29/T34 消费侧）。
 *
 * 钉住四类语义（全部只读）：
 *   A. runScoutSim：真跑 tools/scout_sim.py --json，覆盖率 154/154、相邻帧重叠 ≥30%、
 *      version 严格递增、known 单调、末版 target_cell 非空；非法 carBlock / 越界 mapPath 显式 400。
 *   B. stepLoopResults：读 T34 batch10 的 metrics/batches/analysis，关键指标对账、越界 dir 拒绝。
 *   C. stepLoopArtifact：只允许产物清单内的文件名（../ 越界 → null）。
 *   D. 接线：页面两个面板 + 三个路由都在 index.js / sim.html 里。
 *
 * 跑法：node scripts/test-sim-scout.mjs（只读；不写仓库 config/，不连车）
 */
import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const HERE = path.dirname(fileURLToPath(import.meta.url))
const PLUGIN = path.resolve(HERE, '..')
const pass = { ok: 0, fail: 0 }
const check = (name, ok, note = '') => {
  if (ok) pass.ok += 1
  else pass.fail += 1
  console.log('  ' + (ok ? 'PASS' : 'FAIL') + ' ' + name + (note ? ' —— ' + note : ''))
}
const h = (title) => console.log('\n\u001b[36m' + title + '\u001b[0m')
const seen = (list) => (Array.isArray(list) ? list : [])
const nums = (list) => seen(list).map((item) => Number(item))

const sb = await import('file:///' + path.join(PLUGIN, 'src', 'scoutsim.js').replace(/\\/g, '/'))
const { runScoutSim, stepLoopResults, stepLoopArtifact } = sb

/* ── A. 机端侦察仿真 ───────────────────────────────────────────── */
h('A. runScoutSim：tools/scout_sim.py --json（T28）')
const scout = await runScoutSim({ startT: -30, frameHz: 2, heartbeatS: 1 })
check('A1 返回 ok（exit 0 + JSON）', scout.ok === true, scout.error || '')
const stats = scout.stats || {}
check('A2 覆盖率 154/154', stats.coverage_cells === stats.cells_total && stats.cells_total === 154,
  'coverage=' + stats.coverage_cells + '/' + stats.cells_total)
check('A3 相邻帧面积重叠 min ≥ 30%', Number(stats.overlap_area_min) >= 0.3 - 1e-9,
  String(stats.overlap_area_min))
check('A4 布局 version 严格递增', nums(seen(scout.layouts).map((l) => l.version)).every((v, i, arr) => i === 0 || v > arr[i - 1]),
  nums(seen(scout.layouts).map((l) => l.version)).slice(0, 4).join(','))
check('A5 known 单调不减', nums(seen(scout.layouts).map((l) => l.knownCount)).every((v, i, arr) => i === 0 || v >= arr[i - 1]),
  nums(seen(scout.layouts).map((l) => l.knownCount)).slice(-3).join(','))
check('A6 末版 target_cell 非空', Boolean(scout.layouts && scout.layouts.length && scout.layouts[scout.layouts.length - 1].targetCell),
  String(scout.layouts && scout.layouts.length ? scout.layouts[scout.layouts.length - 1].targetCell : ''))
check('A7 心跳 > 0 且 known=154', Number(scout.heartbeatCount) > 0 && Number(scout.knownCount) === 154,
  'hb=' + scout.heartbeatCount + ' known=' + scout.knownCount)
const cmp = scout.comparison || {}
check('A8 comparison 三臂齐全', Boolean(cmp.generated && cmp.user_route && cmp.user_route_same_step),
  Object.keys(cmp).join(','))
const badBlock = await runScoutSim({ carBlock: 'Z9' })
check('A9 非法 carBlock → 400（不传给 python 猜）', badBlock.ok === false && badBlock.code === 400, badBlock.error)
const badMap = await runScoutSim({ mapPath: 'C:\\Windows\\win.ini' })
check('A10 越界 mapPath → 400', badMap.ok === false && badMap.code === 400, badMap.error)

/* ── B. 十批步进成果 ───────────────────────────────────────────── */
h('B. stepLoopResults：T34 batch10（只读）')
const step = stepLoopResults()
check('B1 返回 ok 且 dir 指向 batch10', step.ok === true && String(step.dir || '').endsWith('batch10'), step.error || step.dir)
const m = step.metrics || {}
check('B2 metrics 关键值（时长/等待/雷/停站/未知格）',
  Math.abs(Number(m.total_time_s) - 288.53) < 0.01 && Math.abs(Number(m.wait_total_s) - 89.62) < 0.01
  && Number(m.mines_pressed_count) === 8 && Number(m.mines_total) === 8
  && Number(m.shoot_stops_visited) === 2
  && Number(m.unknown_entries_plan) === 0 && Number(m.unknown_entries_truth) === 0,
  'time=' + m.total_time_s + ' wait=' + m.wait_total_s + ' mines=' + m.mines_pressed_count + '/' + m.mines_total)
const batches = seen(step.batches)
check('B3 batches 10 行 + known 递增', batches.length === 10
  && nums(batches.map((row) => row.known_count)).every((v, i, arr) => i === 0 || v >= arr[i - 1]),
  'rows=' + batches.length)
check('B4 analysis 非空且含等待归因', typeof step.analysis === 'string'
  && step.analysis.length > 100 && step.analysis.includes('等待归因'), String(step.analysis || '').length)
check('B5 产物清单含 gantt.png / traj_batch.png', seen(step.files).includes('gantt.png') && seen(step.files).includes('traj_batch.png'),
  seen(step.files).join(','))
const outDir = stepLoopResults({ dir: '../../..' })
check('B6 ../ 越界 dir 被拒', outDir.ok === false && outDir.code === 400, outDir.error)
const missing = stepLoopResults({ dir: 'nope_xyz' })
check('B7 不存在的目录 → 404', missing.ok === false && missing.code === 404, missing.error)

/* ── C. 产物白名单 ─────────────────────────────────────────────── */
h('C. stepLoopArtifact：产物白名单 + 目录边界')
const png = stepLoopArtifact({ name: 'gantt.png' })
check('C1 gantt.png 可下载 + content-type=image/png', Boolean(png && png.type === 'image/png'), JSON.stringify(png))
check('C2 ../../robot.yaml 被拒（null）', stepLoopArtifact({ name: '../../robot.yaml' }) === null)
check('C3 不存在的文件名 → null', stepLoopArtifact({ name: 'nope.txt' }) === null)

/* ── D. 页面 / 路由接线 ───────────────────────────────────────── */
h('D. 接线：sim.html 两个面板 + index.js 三个路由')
const html = fs.readFileSync(path.join(PLUGIN, 'sim.html'), 'utf8')
const idx = fs.readFileSync(path.join(PLUGIN, 'index.js'), 'utf8')
check('D1 页面有「步进侦察」与「十批步进成果」面板', html.includes('步进侦察') && html.includes('十批步进成果'))
check('D2 页面调 /sim/scout 与 /sim/step-loop', html.includes("'/sim/scout'") && html.includes("'/sim/step-loop'"))
check('D3 index.js 注册 POST /sim/scout', idx.includes("route === '/sim/scout'"))
check('D4 index.js 注册 GET /sim/step-loop + /artifact',
  idx.includes("route === '/sim/step-loop'") && idx.includes("route === '/sim/step-loop/artifact'"))

const total = pass.ok + pass.fail
console.log('\n' + '='.repeat(70))
console.log('结果：' + pass.ok + '/' + total + (pass.fail === 0 ? ' 全部通过 ✅' : '，' + pass.fail + ' 失败 ❌'))
console.log('='.repeat(70))
process.exit(pass.fail === 0 ? 0 : 1)
