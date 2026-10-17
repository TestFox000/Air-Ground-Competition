/**
 * Everything the Host serves at runtime must be declared in `package.json#files`
 * *and* actually present in the installed copy.
 *
 * Why this exists: the profile installs this package as a `file:` dependency, and
 * that materializes as a packed copy which honours `files`. `novnc/` and
 * `vnc-app.html` were missing from that list, so an install silently produced a
 * plugin whose VNC page and every noVNC module answered 404 — while every test
 * that boots from the source tree kept passing, because the source tree has the
 * files. The tests were validating the checkout, not the artifact the browser
 * actually loads. This is the check that closes that gap.
 *
 * Run: node scripts/test-package-files.mjs
 */

import fs from 'node:fs'
import os from 'node:os'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const here = path.dirname(fileURLToPath(import.meta.url))
const root = path.join(here, '..')
const pkg = JSON.parse(fs.readFileSync(path.join(root, 'package.json'), 'utf8'))
const declared = Array.isArray(pkg.files) ? pkg.files : []

const failures = []
const check = (label, run) => {
  try {
    run()
    console.log(`OK    ${label}`)
  } catch (error) {
    failures.push(`${label}: ${error?.message ?? error}`)
    console.log(`FAIL  ${label}\n      ${error?.message ?? error}`)
  }
}

/**
 * Paths `index.js` hands to the browser at runtime.
 *
 * `vnc-app.html` is the fixed bridge's iframe document, `vnc-connect.html` the
 * generic connector's, `float.html` the standalone status float, and `novnc/`
 * is the whole ES module graph both documents import. All are read from disk
 * per request, so a missing one is a 404 in the browser and nothing else.
 */
const RUNTIME_ASSETS = ['vnc-app.html', 'vnc-connect.html', 'float.html', 'novnc']

/** Files whose installed copy must be byte-identical to the source tree. */
const EXACT_FILES = [
  'index.js', 'client.js', 'float.html',
  'vnc-app.html', 'vnc-connect.html',
  'src/vnc.js', 'src/ws.js', 'src/metrics.js', 'src/files.js',
]

/** Whether a `files` entry ships `relative` (a path or anything under it). */
const isDeclared = relative => declared.some(entry => {
  const clean = String(entry).replace(/^\.\//, '').replace(/\/+$/, '')
  return clean === relative || relative.startsWith(`${clean}/`)
})

// ── 1. the matcher discriminates, so the checks below cannot pass vacuously ───
check('the declaration matcher rejects an entry that is not shipped', () => {
  if (isDeclared('scripts')) throw new Error('scripts/ is not shipped, so the matcher must not accept it')
  if (!isDeclared('index.js')) throw new Error('index.js is shipped, so the matcher must accept it')
})

// ── 2. each runtime asset exists in the source tree ──────────────────────────
for (const asset of RUNTIME_ASSETS) {
  check(`source tree contains ${asset}`, () => {
    if (!fs.existsSync(path.join(root, asset))) {
      throw new Error(`${asset} is missing from the source tree, so nothing can serve it`)
    }
  })
}

// ── 3. each runtime asset is declared, so an install will carry it ───────────
for (const asset of RUNTIME_ASSETS) {
  check(`package.json#files declares ${asset}`, () => {
    if (!isDeclared(asset)) {
      throw new Error(`${asset} is not covered by "files"; a packed install would drop it and the browser would 404`)
    }
  })
}

// ── 4. the installed copy actually has them ──────────────────────────────────
const dshHome = process.env.DSH_HOME ?? path.join(os.homedir(), '.dsh')
const profileName = process.env.DZD_PROFILE ?? 'desktop'
const installed = path.join(dshHome, 'profiles', profileName, 'node_modules', pkg.name)

if (!fs.existsSync(installed)) {
  console.log(`SKIP  no install at ${installed} (nothing deployed to check)`)
} else {
  for (const asset of RUNTIME_ASSETS) {
    check(`installed copy contains ${asset}`, () => {
      if (!fs.existsSync(path.join(installed, asset))) {
        throw new Error(`${asset} is absent from ${installed}; the running plugin cannot serve it`)
      }
    })
  }
  // The documents must also be the ones in the source tree — a stale copy is
  // the same failure with a different cause. The Host half loads from here, so
  // a stale src/ would mean the fixes under test are not the ones running.
  // client.js is included because the browser half is consumed as-is here —
  // there is no bundling step to hide a stale copy.
  check('the installed plugin matches the source tree', () => {
    for (const relative of EXACT_FILES) {
      const source = fs.readFileSync(path.join(root, relative))
      const shippedPath = path.join(installed, relative)
      if (!fs.existsSync(shippedPath)) throw new Error(`${relative} is absent from the install`)
      if (!source.equals(fs.readFileSync(shippedPath))) {
        throw new Error(`${relative} differs from the source tree; the running plugin is stale (reinstall)`)
      }
    }
  })
}

if (failures.length > 0) {
  console.log('\nFAILURES:')
  for (const failure of failures) console.log(' -', failure)
  process.exit(1)
}
console.log('\nPACKAGE FILES TEST PASSED')
