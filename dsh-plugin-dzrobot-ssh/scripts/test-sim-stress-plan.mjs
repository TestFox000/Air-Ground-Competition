#!/usr/bin/env node
/* scripts/test-sim-stress-plan.mjs —— 仿真台压测「规划模式 + 引擎选择 + 补图统计」守卫（T26 仿真台）。
 *
 * 钉住五类语义（只读源码；会真跑 plan_route，但只写系统临时目录 + artifacts/sim/stress/）：
 *   A. checkPlan：同一张官方图上 tour 可规划、tracker-safe 结构性无解（:PIVOT 图与
 *      tracker-safe「禁 180°/禁紧凑 U」冲突）——这是「64/64 plan-failed」的根因证据。
 *   B. runStress 规划模式：每张图只跑一次 plan_route，不进入仿真；engine 显式透传；
 *      成功/失败都落 plan 摘要；规划模式强制不补图（否则成功率被选择偏差吃掉）。
 *   C. 非法引擎 → 400（不静默回落 config）。
 *   D. stressStats 规划口径：planned / plan-failed / generator-unsolvable 分母显式，
 *      generator-unsolvable 不并入规划失败率。
 *   E. 页面接线：模式/引擎/补图控件 + 引擎冲突提示存在。
 *
 * 跑法：node scripts/test-sim-stress-plan.mjs（约 5~10 s；不连车、不改 config/）
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
const sb = await import('file:///' + path.join(PLUGIN, 'src', 'simbench.js').replace(/\\/g, '/'))
const st = await import('file:///' + path.join(PLUGIN, 'src', 'stressbench.js').replace(/\\/g, '/'))
const { checkPlan } = sb
const { runStress, stressResult, stressStats } = st

/* ── A. checkPlan：官方图 tour vs tracker-safe ──────────────────── */
h('A. checkPlan：官方第六张图（68 token / 3 个 :PIVOT）')
const tour = await checkPlan({ engine: 'tour' })
check('A1 tour 能规划出路线', tour.ok === true && tour.tokens.length > 0,
  'ok=' + tour.ok + ' tokens=' + tour.tokens.length + ' ' + (tour.error || ''))
check('A2 tour 引擎显式透传（engine=tour/source=explicit）',
  tour.engine === 'tour' && tour.engineSource === 'explicit', tour.engine + '/' + tour.engineSource)
check('A3 tour 产物 token 数 ≥ 50（与现役 68 点同量级）', tour.tokens.length >= 50, String(tour.tokens.length))
const safe = await checkPlan({ engine: 'tracker-safe' })
check('A4 tracker-safe 对同一张图结构性无解（:PIVOT 冲突）',
  safe.ok === false && /无解|禁 180|禁紧凑/.test(String(safe.error || '')), String(safe.error || '').slice(0, 120))
check('A5 tracker-safe 的 engine 字段仍如实记录', safe.engine === 'tracker-safe', String(safe.engine))

/* ── B. runStress 规划模式 ──────────────────────────────────────── */
h('B. runStress：mode=plan（只规划，不跑 sim）')
const planBatch = await runStress({ count: 2, seed: 20261018, mode: 'plan', engine: 'tour' })
check('B1 批次 ok 且 2 条记录', planBatch.ok === true && (planBatch.records || []).length === 2,
  planBatch.error || 'n=' + (planBatch.records || []).length)
const recs = planBatch.records || []
check('B2 记录全带 plan 摘要（engine=tour / engineSource=explicit / ms≥0）',
  recs.every((r) => r.plan && r.plan.engine === 'tour' && r.plan.engineSource === 'explicit'
    && Number.isFinite(r.plan.ms) && r.plan.ms >= 0),
  recs.map((r) => r.plan && r.plan.engine).join(','))
check('B3 状态只可能是 planned / plan-failed（不进入仿真）',
  recs.every((r) => r.status === 'planned' || r.status === 'plan-failed'),
  recs.map((r) => r.status).join(','))
check('B4 规划压测统计分母显式：ok+fail = n', planBatch.stats.plan.ok + planBatch.stats.plan.fail === 2,
  planBatch.stats.plan.ok + '+' + planBatch.stats.plan.fail)
check('B5 至少 1 张规划成功（同 seed 确定）', planBatch.stats.plan.ok >= 1, String(planBatch.stats.plan.ok))
check('B6 规划模式强制 resample=false（计划里显式写死）', await (async () => {
  const r = await stressResult(planBatch.batchId)
  return r.ok === true && r.batch && r.batch.mode === 'plan' && r.batch.resample === false
})(), 'summary.batch.resample')
const named = await runStress({ count: 1, seed: 20261018, mode: 'plan', engine: 'tour', label: 'plan_smoke' })
check('B7 label 含下划线时 result 仍可回读（batchId 允许多下划线）',
  named.ok === true && await (async () => (await stressResult(named.batchId)).ok === true)(), named.batchId)

/* ── C. 非法引擎 ────────────────────────────────────────────────── */
h('C. 引擎白名单')
const bad = await runStress({ count: 1, seed: 1, mode: 'plan', engine: 'nope' })
check('C1 非法引擎 → 400（不静默回落 config）', bad.ok === false && bad.code === 400, bad.error || '')

/* ── D. stressStats 规划口径（纯函数） ──────────────────────────── */
h('D. stressStats：planned / plan-failed / generator-unsolvable')
const stats = stressStats([
  { status: 'planned', plan: { ms: 100, tokenCount: 68, engine: 'tour' } },
  { status: 'plan-failed', plan: { ms: 80, tokenCount: 0, engine: 'tracker-safe', stderrTail: '· tracker-safe 引擎无解' } },
  { status: 'generator-unsolvable', plan: { ms: 70, tokenCount: 0, engine: 'tour' }, planRejects: 2 },
])
check('D1 plan.ok/fail/genUnsolved 分开计数', stats.plan.ok === 1 && stats.plan.fail === 1 && stats.plan.genUnsolved === 1,
  JSON.stringify({ ok: stats.plan.ok, fail: stats.plan.fail, gen: stats.plan.genUnsolved }))
check('D2 okRate = planned / n（分母含 generator-unsolvable）', stats.plan.okRate === 0.3333, String(stats.plan.okRate))
check('D3 token 均值只取规划成功', stats.plan.tokens.avg === 68 && stats.plan.tokens.min === 68, JSON.stringify(stats.plan.tokens))
check('D4 失败主因去前缀、可展示', stats.plan.topReasons.length === 1
  && stats.plan.topReasons[0].reason === 'tracker-safe 引擎无解', JSON.stringify(stats.plan.topReasons))
check('D5 引擎分布照实', stats.plan.engines.tour === 2 && stats.plan.engines['tracker-safe'] === 1,
  JSON.stringify(stats.plan.engines))

/* ── E. 页面接线 ────────────────────────────────────────────────── */
h('E. sim.html 接线')
const html = fs.readFileSync(path.join(PLUGIN, 'sim.html'), 'utf8')
for (const id of ['stressMode', 'stressEngine', 'stressResample', 'stressMaxAttempts', 'stressEngineWarn']) {
  check('E.' + id + ' 控件存在', html.includes('id="' + id + '"'))
}
check('E.6 stressConfig 透传 mode/engine/resample/maxAttempts',
  /mode: mode/.test(html) && /resample: resample/.test(html) && /maxAttempts: maxAttempts/.test(html)
  && /if \(engine\) body\.engine = engine/.test(html))
check('E.7 引擎冲突提示函数在 loadField 后刷新', /refreshStressEngineHint\(\);    \/\/ 压测面板/.test(html))
check('E.8 规划模式不会画仿真指标（卡内按 mode=plan 走 token 行）', /record\.mode\) === 'plan'/.test(html))
check('E.9 压测输入与「场地来源」解耦（loadStressBase + 固定仓库提示）',
  /function loadStressBase/.test(html) && /压测固定使用仓库/.test(html) && /loadStressBase\(\);/.test(html))

console.log('\n结果：' + pass.ok + '/' + (pass.ok + pass.fail) + (pass.fail === 0 ? ' 全部通过 ✅' : '，' + pass.fail + ' 失败 ❌'))
process.exit(pass.fail === 0 ? 0 : 1)
