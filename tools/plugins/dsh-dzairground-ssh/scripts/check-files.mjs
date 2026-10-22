/**
 * Verify the remote file layer against the real car.
 * Run: node scripts/check-files.mjs [host] [user]
 */

import { homeShortcuts, listDirectory, readTextFile, remote, statPath } from '../src/files.js'

const settings = {
  host: process.argv[2] ?? 'duzhong.local',
  user: process.argv[3] ?? 'duzhong',
  port: 22,
  sshAlias: '',
}

let failures = 0
const check = (label, ok, detail) => {
  console.log(`${ok ? 'OK  ' : 'FAIL'}  ${label}${detail === undefined ? '' : `  ${detail}`}`)
  if (!ok) failures += 1
}

const root = await listDirectory(settings, '/home/duzhong')
check('list /home/duzhong', root.ok === true, root.error ?? `${root.entries.length} entries, path=${root.path}`)
if (root.ok) {
  const dirs = root.entries.filter(entry => entry.type === 'directory').map(entry => entry.name)
  console.log('      first entries:', root.entries.slice(0, 8).map(entry => `${entry.type === 'directory' ? 'd' : '-'}${entry.name}`).join('  '))
  check('directories sort first', root.entries.length === 0 || root.entries[0].type === 'directory' || dirs.length === 0)
}

// A name with a space, a quote and a double quote is the whole point of the NUL
// framing. The fixture is created from a *literal* here-doc script over stdin —
// deliberately NOT through files.js's own `quote()`, so a quoting bug cannot
// cancel itself out and make this check pass.
const NASTY = "weird 'name\" with space"
const { execFileSync } = await import('node:child_process')
const fixture = `mkdir -p /tmp/dsh-names && rm -f /tmp/dsh-names/* && cd /tmp/dsh-names && touch -- ${JSON.stringify(NASTY)}\n`
execFileSync('ssh', ['-o', 'BatchMode=yes', `${settings.user}@${settings.host}`, 'bash -s'], { input: fixture })

const nasty = await listDirectory(settings, '/tmp/dsh-names')
check('list a directory holding a quoted name', nasty.ok === true, nasty.error ?? '')
if (nasty.ok) {
  const found = nasty.entries.some(entry => entry.name === NASTY)
  check('awkward filename survives intact', found, nasty.entries.map(entry => JSON.stringify(entry.name)).join(', '))
}

const missing = await listDirectory(settings, '/definitely/not/here')
check('missing directory is an error, not an empty list', missing.ok === false, missing.error)

const info = await statPath(settings, '/etc/hostname')
check('stat /etc/hostname', info.ok === true && info.type === 'file' && info.size === 8, JSON.stringify(info))

// Regression guard: /usr/bin/Xvnc is a symlink chain. Without dereferencing, its
// reported size is the 22-byte length of the link's target path, which defeats
// the size guard and lets a 2.7 MB binary be read in full.
const link = await statPath(settings, '/usr/bin/Xvnc')
check('stat dereferences a symlink to its real size', link.ok === true && link.size > 1000000,
  `size=${link.size} type=${link.type}`)
check('a symlink to a huge file is refused by the size guard',
  (await readTextFile(settings, '/usr/bin/Xvnc')).ok === false, '')

const text = await readTextFile(settings, '/etc/hostname')
check('read /etc/hostname as text', text.ok === true && text.binary === false, JSON.stringify(String(text.text ?? '').trim()))

// Two deterministic binary fixtures, because the interesting cases differ: one
// contains NUL bytes, the other is binary *without* any NUL but is not valid
// UTF-8 either. Random bytes would flake on the second one (~21% of 400 random
// bytes contain no NUL at all), which is exactly the weakness being covered.
const { execFileSync: runRemote } = await import('node:child_process')
const fixtures = `python3 -c "
import sys
open('/tmp/dsh-bin-nul.bin','wb').write(bytes([0,1,2,3,0,255,7,0,65,0,66]))
open('/tmp/dsh-bin-nonul.bin','wb').write(bytes([0xC3,0x28,0xA0,0x1B])*100)
"
`
runRemote('ssh', ['-o', 'BatchMode=yes', `${settings.user}@${settings.host}`, 'bash -s'], { input: fixtures })

const withNul = await readTextFile(settings, '/tmp/dsh-bin-nul.bin')
check('a binary file with NUL bytes is detected, not rendered',
  withNul.ok === true && withNul.binary === true && withNul.text === undefined,
  JSON.stringify({ ok: withNul.ok, binary: withNul.binary, size: withNul.size }))

const withoutNul = await readTextFile(settings, '/tmp/dsh-bin-nonul.bin')
check('a binary file without NUL bytes is detected too',
  withoutNul.ok === true && withoutNul.binary === true && withoutNul.text === undefined,
  JSON.stringify({ ok: withoutNul.ok, binary: withoutNul.binary, size: withoutNul.size }))

// The other side of the same coin: real text must still come back as text.
const realText = await readTextFile(settings, '/home/duzhong/.bashrc')
check('a real text file is still returned as text',
  realText.ok === true && realText.binary === false && typeof realText.text === 'string' && realText.text.length > 0,
  `size=${realText.size}`)

const dirAsFile = await readTextFile(settings, '/etc')
check('directory refuses to open as a file', dirAsFile.ok === false, dirAsFile.error)

console.log('\nshortcuts:', homeShortcuts().map(item => `${item.label}=${item.path}`).join('  '))

// Leave the car as it was found: these fixtures live in /tmp on a real robot.
await remote(settings, 'rm -rf /tmp/dsh-names /tmp/dsh-bin-probe.bin /tmp/dsh-bin-nul.bin /tmp/dsh-bin-nonul.bin')

console.log(failures === 0 ? '\nFILE LAYER PASSED' : `\n${failures} CHECK(S) FAILED`)
process.exitCode = failures === 0 ? 0 : 1
