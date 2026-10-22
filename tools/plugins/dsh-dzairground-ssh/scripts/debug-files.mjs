/**
 * Pinpoint why the directory listing fails, by running the exact remote command
 * the plugin builds and showing the raw exit code, stderr and stdout.
 * Run: node scripts/debug-files.mjs
 */

import { listDirectory, quote, remote } from '../src/files.js'

const settings = { host: process.argv[2] ?? 'duzhong.local', user: process.argv[3] ?? 'duzhong', port: 22, sshAlias: '' }
const target = '/home/duzhong'

const variants = {
  'with -L (current)': `cd -- ${quote(target)} 2>/dev/null || { echo __DSH_ERR__; exit 3; }; find -L . -mindepth 1 -maxdepth 1 -printf '%f\\0%y\\0%s\\0%T@\\0' 2>/dev/null | base64 -w0; echo; pwd`,
  'without -L (previous)': `cd -- ${quote(target)} 2>/dev/null || { echo __DSH_ERR__; exit 3; }; find . -mindepth 1 -maxdepth 1 -printf '%f\\0%y\\0%s\\0%T@\\0' 2>/dev/null | base64 -w0; echo; pwd`,
  'find -L exit code alone': `find -L . -mindepth 1 -maxdepth 1 -printf '%f\\n' >/dev/null 2>&1; echo "find exit=$?"`,
  'broken links inside?': `find -L . -maxdepth 1 -type l -printf '%p -> %l\\n' 2>&1 | head -10; echo "---"; find . -maxdepth 1 -type l | head -10`,
}

for (const [label, command] of Object.entries(variants)) {
  const result = await remote(settings, command, { timeoutMs: 25000 })
  const stdout = result.stdout.toString('utf8')
  console.log(`\n### ${label}`)
  console.log(`  code=${result.code} timedOut=${result.timedOut === true} bytes=${result.stdout.length}`)
  if (result.stderr.trim() !== '') console.log(`  stderr=${result.stderr.trim().slice(0, 300)}`)
  console.log(`  stdout(head)=${JSON.stringify(stdout.slice(0, 140))}`)
}

console.log('\n### listDirectory()')
const listing = await listDirectory(settings, target)
console.log(' ', JSON.stringify({ ok: listing.ok, error: listing.error, entries: listing.entries?.length }))
