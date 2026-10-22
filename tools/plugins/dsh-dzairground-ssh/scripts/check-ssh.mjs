/**
 * Standalone check of the SSH layer against a real host.
 *
 * Run: node scripts/check-ssh.mjs [host] [user] [port]
 * Exercises splitOptions, interactiveArgv, probe and describe without a dsh
 * process, so a broken argv or a wrong assumption about the car shows up here
 * rather than as a broken sidebar button.
 */

import { describe as probeDescribe, interactiveArgv, probe, splitOptions } from '../src/ssh.js'

const host = process.argv[2] ?? 'duzhong.local'
const user = process.argv[3] ?? 'duzhong'
const port = Number(process.argv[4] ?? 22)

console.log('splitOptions:', JSON.stringify(splitOptions('-o "ProxyCommand nc %h %p" -p 22 -X foo')))
console.log('interactiveArgv:', JSON.stringify(interactiveArgv({ host, user, port, sshOptions: '-o ServerAliveInterval=30' })))

const settings = { host, user, port, sshAlias: '', probeTimeoutMs: 4000, sshOptions: '' }

console.log('\nprobing', `${user}@${host}:${port}`, '…')
const result = await probe(settings, { logger: console })
console.log('probe ->', JSON.stringify(result, null, 2))

if (result.state === 'ready') {
  console.log('\ndescribing …')
  console.log('describe ->', JSON.stringify(await probeDescribe(settings, { logger: console }), null, 2))
}