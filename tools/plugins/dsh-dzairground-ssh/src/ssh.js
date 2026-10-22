/**
 * The SSH layer: everything that actually talks to the car.
 *
 * Two commands, deliberately kept distinct because they answer different
 * questions and have opposite failure modes:
 *
 *   - `probe()`  — can we reach the car? One short-lived `ssh -o BatchMode=yes`
 *     that prints a marker. It never asks for a password, so it can never hang
 *     on a prompt, and it tells us "reachable", not "connected".
 *   - `probeLogin()` — can we *log in* without a password prompt? Same shape, but
 *     the exit status distinguishes "authenticated" from "reached it".
 *
 * The interactive shell is not built here: it is handed to the host's terminal
 * service as a command line, so the user gets a real PTY with echo, history and
 * Ctrl-C exactly as `ssh` would give in their own console. That also means the
 * plugin never has to parse a terminal stream.
 *
 * Authentication note: the reachability path is `BatchMode=yes`, so a car that
 * needs a password is reported as "needs a password" rather than silently
 * blocking on an invisible prompt. The interactive shell that actually gets
 * opened carries no such restriction, so a password car still works — the user
 * simply types the password in the terminal that opens.
 */

import { spawn } from 'node:child_process'
import { hostname as osHostname } from 'node:os'

/** Marker the remote command prints on success, so "empty output" is not proof. */
const LOGIN_MARKER = 'DSH_DZAIRGROUND_OK'

/** Split a settings `sshOptions` string into argv, honouring simple quoting. */
export function splitOptions(raw) {
  const text = String(raw ?? '').trim()
  if (text === '') return []
  // Split on whitespace but keep "double quoted runs" and 'single quoted runs'
  // together, so `-o "ProxyCommand nc %h %p"` survives as one argument.
  const out = []
  let current = ''
  let quote = null
  let started = false
  for (const char of text) {
    if (quote !== null) {
      if (char === quote) { quote = null; continue }
      current += char
      continue
    }
    if (char === '"' || char === "'") { quote = char; started = true; continue }
    if (/\s/.test(char)) {
      if (started || current !== '') out.push(current)
      current = ''
      started = false
      continue
    }
    current += char
    started = true
  }
  if (started || current !== '') out.push(current)
  return out
}

/**
 * The command that opens an interactive login shell on the car.
 *
 * Returned as argv, never as a shell string: the host spawns this directly, so
 * a host or user containing shell metacharacters cannot turn into command
 * injection. `-tt` forces a PTY on the far side so interactive programs (ros2,
 * sudo, editors) behave.
 */
export function interactiveArgv(settings) {
  const argv = ['ssh']
  argv.push('-tt')
  if (Number.isInteger(settings.port) && settings.port !== 22) argv.push('-p', String(settings.port))
  for (const option of splitOptions(settings.sshOptions)) argv.push(option)
  const alias = String(settings.sshAlias ?? '').trim()
  if (alias !== '') {
    argv.push(alias)
    return argv
  }
  argv.push(`${settings.user}@${settings.host}`)
  return argv
}

/**
 * The same login as {@link interactiveArgv}, as one line a shell can run.
 *
 * This is what the sidebar button types into a DSH terminal. It carries no
 * quoting for the target because a host and a user name are resolved by ssh
 * itself; the only free-form part is `sshOptions`, which is asked for by a user
 * who already owns the machine, and is passed through as-is for the same reason
 * their own `~/.ssh/config` is.
 */
export function commandLine(settings) {
  const alias = String(settings.sshAlias ?? '').trim()
  const parts = ['ssh']
  if (Number.isInteger(settings.port) && settings.port !== 22) parts.push('-p', String(settings.port))
  const options = String(settings.sshOptions ?? '').trim()
  if (options !== '') parts.push(options)
  parts.push(alias !== '' ? alias : `${settings.user}@${settings.host}`)
  return parts.join(' ')
}

/**
 * Run a command to completion, capturing output. Never rejects on non-zero.
 *
 * @param maxStdoutBytes - bound the captured stdout. `0` means unbounded (the
 *   probes emit a marker each); a metrics read sets it because the float polls
 *   forever and a remote shell that streamed forever would otherwise grow the
 *   buffer without limit. Exceeding the bound kills the child and settles the
 *   promise — a truncated answer is reported as a failure, never parsed.
 */
export function run(argv, { timeoutMs, input, maxStdoutBytes = 0 } = {}) {
  return new Promise(resolve => {
    let child
    try {
      child = spawn(argv[0], argv.slice(1), { stdio: ['pipe', 'pipe', 'pipe'], windowsHide: true })
    } catch (error) {
      resolve({ code: -1, stdout: '', stderr: String(error?.message ?? error), spawnFailed: true })
      return
    }
    let stdout = ''
    let stderr = ''
    let settled = false
    const finish = result => {
      if (settled) return
      settled = true
      clearTimeout(timer)
      resolve(result)
    }
    const timer = setTimeout(() => {
      try { child.kill() } catch { /* already gone */ }
      finish({ code: -1, stdout, stderr: stderr || 'timed out', timedOut: true })
    }, timeoutMs)
    timer.unref?.()

    child.stdout?.on('data', chunk => {
      stdout += chunk.toString()
      if (maxStdoutBytes > 0 && stdout.length > maxStdoutBytes) {
        stdout = ''
        try { child.kill() } catch { /* already gone */ }
        finish({ code: -1, stdout, stderr: stderr || `stdout exceeded ${maxStdoutBytes} bytes`, bounded: true })
      }
    })
    child.stderr?.on('data', chunk => { stderr += chunk.toString() })
    child.on('error', error => finish({ code: -1, stdout, stderr: String(error?.message ?? error), spawnFailed: true }))
    child.on('close', code => finish({ code: code ?? -1, stdout, stderr }))
    if (input !== undefined) { try { child.stdin?.end(input) } catch { /* not a pipe */ } }
    else { try { child.stdin?.end() } catch { /* not a pipe */ } }
  })
}

/**
 * Read a failed ssh attempt rather than guessing at it.
 *
 * These refusals look identical in the exit code alone and lead the user to
 * four different fixes, so the table lives in one exported place: the
 * reachability probe and the metrics probe both answer with it, and neither is
 * allowed to invent its own wording for the same underlying failure.
 *
/**
 * @param result - the shape `run()` settles with (code/stdout/stderr/timedOut).
 * @param options.label - which machine the failure is about ('车' by default;
 *   the drone node passes '无人机' so a drone outage never reads as a car one).
 * @returns `{ state, detail, latencyMs, target }` — always a refusal, never ok.
 */
export function refusalFor(result, { ms = 0, target = '', logger, label = '车' } = {}) {
  const text = `${result.stderr ?? ''}\n${result.stdout ?? ''}`.trim()
  const low = text.toLowerCase()
  if (result.timedOut) return { state: 'unreachable', detail: `连接超时（${ms}ms）`, latencyMs: ms, target }
  if (result.spawnFailed) return { state: 'error', detail: `无法启动 ssh：${text.split('\n')[0] ?? 'unknown'}`, latencyMs: ms, target }
  if (/could not resolve|unknown host|name or service not known|no such host|nodename nor servname/.test(low)) {
    return { state: 'unreachable', detail: '无法解析主机名', latencyMs: ms, target }
  }
  if (/connection refused/.test(low)) return { state: 'unreachable', detail: `端口被拒绝（${label}已开但 sshd 没起？）`, latencyMs: ms, target }
  if (/connection timed out|operation timed out|no route to host|network is unreachable|unreachable/.test(low)) {
    return { state: 'unreachable', detail: `网络不可达（${label}还没连上 WiFi？）`, latencyMs: ms, target }
  }
  if (/permission denied|publickey|password/.test(low)) {
    return { state: 'needs-password', detail: '能连上，但需要密码或密钥', latencyMs: ms, target }
  }
  if (/host key verification failed|remote host identification/.test(low)) {
    return { state: 'host-key', detail: '主机密钥与记录不符', latencyMs: ms, target }
  }
  logger?.info?.(`dzairground-ssh: ssh refused for ${target} -> ${result.code} (${ms}ms)`)
  return { state: 'error', detail: text.split('\n').filter(line => line.trim() !== '')[0] ?? `ssh 退出码 ${result.code}`, latencyMs: ms, target }
}

/**
 * One reachability + login check.
 *
 * `BatchMode=yes` is the whole design: it disables every interactive prompt, so
 * a car that wants a password answers with a permission error instead of a
 * prompt we would have to answer from a web page. `ConnectTimeout` bounds the
 * TCP handshake. `StrictHostKeyChecking=accept-new` keeps a first connection
 * from dying on an unknown host while still refusing a changed key.
 */
export async function probe(settings, { logger, label = '车' } = {}) {
  const started = Date.now()
  const target = `${settings.user}@${settings.host}`
  const alias = String(settings.sshAlias ?? '').trim()
  const argv = ['ssh', '-o', 'BatchMode=yes', '-o', `ConnectTimeout=${Math.max(1, Math.round((settings.probeTimeoutMs ?? 2500) / 1000))}`, '-o', 'StrictHostKeyChecking=accept-new']
  if (Number.isInteger(settings.port) && settings.port !== 22) argv.push('-p', String(settings.port))
  argv.push(alias !== '' ? alias : target)
  argv.push(`printf %s ${LOGIN_MARKER}`)

  const result = await run(argv, { timeoutMs: (settings.probeTimeoutMs ?? 2500) + 2000 })
  const ms = Date.now() - started
  const authenticated = result.code === 0 && result.stdout.includes(LOGIN_MARKER)

  if (authenticated) {
    return {
      state: 'ready',
      detail: '免密登录可用',
      latencyMs: ms,
      target: alias !== '' ? alias : target,
    }
  }

  // Read the refusal rather than guessing: these four cases look identical from
  // the exit code alone and lead the user to four different fixes.
  return refusalFor(result, { ms, target, logger, label })
}

/** A short human summary of the car, used by the sidebar tooltip. */
export async function describe(settings, { logger } = {}) {
  const command = [
    'echo "user=$(whoami)"',
    'echo "host=$(hostname)"',
    'echo "kernel=$(uname -r)"',
    "echo \"up=$(uptime -p 2>/dev/null | sed 's/^up //')\"",
  ].join('; ')
  const alias = String(settings.sshAlias ?? '').trim()
  const argv = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5', '-o', 'StrictHostKeyChecking=accept-new']
  if (Number.isInteger(settings.port) && settings.port !== 22) argv.push('-p', String(settings.port))
  argv.push(alias !== '' ? alias : `${settings.user}@${settings.host}`)
  argv.push(command)
  const result = await run(argv, { timeoutMs: 9000 })
  if (result.code !== 0) return { ok: false }
  const facts = {}
  for (const line of result.stdout.split('\n')) {
    const index = line.indexOf('=')
    if (index <= 0) continue
    facts[line.slice(0, index).trim()] = line.slice(index + 1).trim()
  }
  return {
    ok: true,
    user: facts.user ?? '',
    host: facts.host ?? '',
    kernel: facts.kernel ?? '',
    up: facts.up ?? '',
    at: Date.now(),
  }
}

/** The local machine's name, so the settings page can say what it runs on. */
export function localName() {
  try { return osHostname() } catch { return '' }
}