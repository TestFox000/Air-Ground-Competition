import { spawn } from 'node:child_process'
import crypto from 'node:crypto'
import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const HERE = path.dirname(fileURLToPath(import.meta.url))
const PLUGIN = path.resolve(HERE, '..', '..', 'dzrobot_v2', 'dsh-plugin-dzrobot-ssh')
const LOGDIR = path.join(HERE, 'logs')
const TAG = process.argv[2] || 'baseline'
const TESTS = [
  'test-sim-map-render.mjs',
  'test-sim-parity.mjs',
  'test-sim-incremental.mjs',
  'test-sim-overlay-ab.mjs',
  'test-sim-overlay-fix.mjs',
  'test-sim-drone-render.mjs',
  'test-sim-sixth-map.mjs',
]

const sha = (file) => crypto.createHash('sha256').update(fs.readFileSync(file)).digest('hex')
const hashTargets = ['sim.html', 'src/simbench.js', 'src/dronesim.js', ...TESTS.map(t => 'scripts/' + t)]
const hashes = Object.fromEntries(hashTargets.map(f => [f, sha(path.join(PLUGIN, f))]))
fs.writeFileSync(path.join(LOGDIR, `_hashes_${TAG}.json`), JSON.stringify(hashes, null, 2))

function runOne(name) {
  return new Promise(resolve => {
    const started = Date.now()
    const child = spawn(process.execPath, [path.join(PLUGIN, 'scripts', name)], {
      cwd: PLUGIN,
      env: { ...process.env, PYTHONIOENCODING: 'utf-8', PYTHONUNBUFFERED: '1' },
      windowsHide: true,
    })
    let stdout = ''
    let stderr = ''
    let timedOut = false
    const timer = setTimeout(() => { timedOut = true; try { child.kill() } catch {} }, 15 * 60 * 1000)
    child.stdout.on('data', c => { stdout += c })
    child.stderr.on('data', c => { stderr += c })
    child.on('error', err => { clearTimeout(timer); resolve({ name, exitCode: -1, stdout, stderr: stderr + String(err), timedOut, ms: Date.now() - started }) })
    child.on('close', code => { clearTimeout(timer); resolve({ name, exitCode: code ?? 1, stdout, stderr, timedOut, ms: Date.now() - started }) })
  })
}

const results = []
for (const name of TESTS) {
  console.log('RUN ' + name)
  const r = await runOne(name)
  const log = path.join(LOGDIR, `${TAG}__${name.replace(/\.mjs$/, '')}.log`)
  const header = [
    'CMD: node ' + path.join(PLUGIN, 'scripts', name),
    'CWD: ' + PLUGIN,
    'TAG: ' + TAG,
    'START: ' + new Date().toISOString(),
  ].join('\n')
  fs.writeFileSync(log, header + '\n\n--- STDOUT ---\n' + r.stdout + '\n--- STDERR ---\n' + r.stderr + '\n--- END ---\nexitCode=' + r.exitCode + ' timedOut=' + r.timedOut + ' ms=' + r.ms + '\n')
  const passMatches = (r.stdout.match(/PASS/g) || []).length
  const failMatches = (r.stdout.match(/FAIL/g) || []).length
  const ratio = [...r.stdout.matchAll(/(\d+)\s*\/\s*(\d+)\s*(?:通过|passed|PASS)/g)].pop()
  const tail = (r.stdout + r.stderr).trim().split(/\r?\n/).slice(-3).join(' | ')
  results.push({ name, exitCode: r.exitCode, timedOut: r.timedOut, seconds: Math.round(r.ms / 100) / 10, passMatches, failMatches, ratio: ratio ? ratio[0] : '', tail: tail.slice(0, 400), log })
  console.log(`  -> exit=${r.exitCode} timedOut=${r.timedOut} ${(r.ms/1000).toFixed(1)}s PASSx${passMatches} FAILx${failMatches} ${ratio ? ratio[0] : ''}`)
}
fs.writeFileSync(path.join(LOGDIR, `_summary_${TAG}.json`), JSON.stringify(results, null, 2))
console.log('\n=== SUMMARY ' + TAG + ' ===')
for (const r of results) console.log(`${r.exitCode === 0 ? 'OK  ' : 'BAD '} ${r.name.padEnd(32)} exit=${r.exitCode} ${r.seconds}s PASSx${r.passMatches} FAILx${r.failMatches} ${r.ratio}`)
process.exit(results.some(r => r.exitCode !== 0) ? 1 : 0)
