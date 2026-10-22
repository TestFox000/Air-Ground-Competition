/**
 * Guard against the failure mode that cost us the plugin twice: two trees that
 * both look authoritative, and the one DSH installs from quietly being the
 * other one.
 *
 * Two things are asserted, and neither is a style opinion:
 *
 *  1. The profile's `file:` dependency points at *this* tree. Otherwise the next
 *     `pnpm install` (triggered by installing anything else) repacks this
 *     plugin from a stale sibling directory and overwrites the running copy —
 *     silently, because pnpm honours the profile, not the working directory.
 *  2. Any other tree that still looks like a copy of this plugin is reported
 *     with its real divergence, so nobody discovers it by losing work again.
 *
 * Run: node scripts/check-divergence.mjs
 */

import fs from 'node:fs'
import path from 'node:path'
import os from 'node:os'
import { fileURLToPath } from 'node:url'

const root = path.join(path.dirname(fileURLToPath(import.meta.url)), '..')
const dshHome = process.env.DSH_HOME ?? path.join(process.env.USERPROFILE ?? '', '.dsh')
const profile = process.env.DZR_PROFILE ?? 'desktop'
const pkg = JSON.parse(fs.readFileSync(path.join(root, 'package.json'), 'utf8'))

let failures = 0
const check = (label, ok, detail = '') => {
  console.log(`${ok ? 'OK  ' : 'FAIL'}  ${label}${detail ? `  ${detail}` : ''}`)
  if (!ok) failures += 1
}

// ── 1. the profile installs from THIS tree ───────────────────────────────────
const profilePkg = path.join(dshHome, 'profiles', profile, 'package.json')
if (fs.existsSync(profilePkg)) {
  const declared = JSON.parse(fs.readFileSync(profilePkg, 'utf8')).dependencies?.[pkg.name]
  const here = path.resolve(root).replace(/\\/g, '/')
  const points = typeof declared === 'string' && declared.startsWith('file:')
    ? declared.slice(5).replace(/\\/g, '/').replace(/\/+$/, '')
    : ''
  const match = points !== '' && path.resolve(points).replace(/\\/g, '/') === here
  check(`profile declares ${pkg.name} from this tree`, match,
    match ? points : `declared=${JSON.stringify(declared)} expected=file:${here}`)
} else {
  console.log(`SKIP  no profile manifest at ${profilePkg}`)
}

// A pinned profile whose lockfile still names a different tree is worse than
// unpinned: pnpm will read one directory and pack another.
for (const rel of ['pnpm-lock.yaml', path.join('node_modules', '.modules.yaml'), path.join('node_modules', '.pnpm', 'lock.yaml')]) {
  const file = path.join(dshHome, 'profiles', profile, rel)
  if (!fs.existsSync(file)) continue
  const text = fs.readFileSync(file, 'utf8')
  const refs = [...new Set([...text.matchAll(/file:([^\s'"']*dsh-plugin-dzairground-ssh)/g)].map(m => m[1]))]
  const strays = refs.filter(ref => path.resolve(ref).replace(/\\/g, '/') !== path.resolve(root).replace(/\\/g, '/'))
  check(`${rel} names no foreign copy of this plugin`, strays.length === 0,
    strays.length === 0 ? `${refs.length} ref(s) OK` : `foreign: ${JSON.stringify(strays)}`)
}

// ── 2. the archive tree is reported, not silently trusted ────────────────────
// Known sibling copies. This is a *report*, not a hard failure: a stale archive
// is a fact worth printing every run, but it must not block the work.
const siblings = [
  // profile 安装副本 —— DSH 实际运行的那一份（S5 切换后唯一镜像）；不存在时只报 NOTE。
  path.join(process.env.USERPROFILE ?? '', '.dsh', 'profiles', 'desktop', 'node_modules', 'dsh-dzairground-ssh'),
].map(p => path.resolve(p)).filter(p => p !== path.resolve(root))

const walk = (dir, base = dir, acc = []) => {
  if (!fs.existsSync(dir)) return acc
  for (const entry of fs.readdirSync(dir, { withFileTypes: true })) {
    const full = path.join(dir, entry.name)
    if (entry.isDirectory()) walk(full, base, acc)
    else acc.push(path.relative(base, full).replace(/\\/g, '/'))
  }
  return acc
}

const here = new Set(walk(root))
for (const sib of siblings) {
  if (!fs.existsSync(sib)) { console.log(`NOTE  archive tree is gone: ${sib}`); continue }
  const there = new Set(walk(sib))
  const onlyHere = [...here].filter(f => !there.has(f))
  const onlyThere = [...there].filter(f => !here.has(f))
  const differing = [...here].filter(f => there.has(f) &&
    !fs.readFileSync(path.join(root, f)).equals(fs.readFileSync(path.join(sib, f))))
  console.log(`\nNOTE  archive tree ${sib}`)
  console.log(`      only-here=${onlyHere.length} only-there=${onlyThere.length} differing=${differing.length}`)
  if (onlyThere.length > 0) console.log(`      ⚠ present ONLY in the archive: ${onlyThere.slice(0, 12).join(', ')}`)
  // Only files that exist solely in the archive are a real risk: a feature
  // that lives in one tree and not the other is the exact thing we lost.
  if (onlyThere.length > 0) {
    console.log('      → resolve before the next pnpm install, or that work is unreachable')
  }
}

if (failures > 0) {
  console.log(`\nDIVERGENCE CHECK FAILED (${failures})`)
  process.exit(1)
}
console.log('\nDIVERGENCE CHECK PASSED')