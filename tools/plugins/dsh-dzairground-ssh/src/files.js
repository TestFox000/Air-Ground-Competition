/**
 * The car's filesystem, reached over the SSH channel that is already trusted.
 *
 * SFTP would be the obvious vehicle, but it needs a client library the host does
 * not ship. `ssh` itself is already installed, already authenticated, and — this
 * is the part that matters — already known to work against this car. So every
 * operation here is one `ssh … command` with a machine-readable answer:
 *
 *   · listing:  `find -printf` with NUL separators, base64-wrapped. Parsing `ls`
 *     would break on the filenames a robot project actually has; NUL framing
 *     plus one base64 envelope means a name with spaces, newlines or quotes
 *     survives the shell, the transport and the parser untouched.
 *   · content:  `base64 -w0` after a size check, so a viewer never pulls a
 *     multi-gigabyte bag file into a JSON response.
 *   · download: `cat`, streamed to the caller as an octet stream.
 *   · upload:   `cat >`, streamed from the caller.
 *
 * Every remote value is quoted for the remote shell with {@link quote}, which
 * is the only place a caller-supplied path touches a command line.
 *
 * @module src/files.js
 */

import { spawn } from 'node:child_process'

/** Largest file the text viewer will read. */
export const MAX_READ_BYTES = 2 * 1024 * 1024

/** Largest upload this plugin will forward in one request. */
export const MAX_UPLOAD_BYTES = 512 * 1024 * 1024

/**
 * Quote one value for a POSIX shell.
 *
 * Single quotes make every byte literal; the only character that cannot appear
 * inside them is a single quote itself, which is closed, escaped and reopened.
 * This is the single chokepoint for caller-supplied text.
 */
export function quote(value) {
  return `'${String(value).replaceAll("'", "'\\''")}'`
}

/**
 * The `ssh` argv for one remote command.
 *
 * `BatchMode=yes` keeps a password prompt from hanging a request that has no
 * terminal to answer it; `-T` allocates no PTY, because these are data commands,
 * not interactive ones.
 */
export function remoteArgv(settings, command) {
  const argv = ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=accept-new']
  if (Number.isInteger(settings.port) && settings.port !== 22) argv.push('-p', String(settings.port))
  const alias = String(settings.sshAlias ?? '').trim()
  const destination = alias !== '' ? alias : `${settings.user}@${settings.host}`
  argv.push('-o', 'ConnectTimeout=8', destination, command)
  return argv
}

/** Run one remote command, capturing bounded output. Never rejects on non-zero. */
export function remote(settings, command, { timeoutMs = 20000, maxBytes = 8 * 1024 * 1024, signal, input } = {}) {
  return new Promise(resolve => {
    let child
    try {
      child = spawn(remoteArgv(settings, command)[0], remoteArgv(settings, command).slice(1), {
        stdio: [input === undefined ? 'ignore' : 'pipe', 'pipe', 'pipe'],
        windowsHide: true,
      })
    } catch (error) {
      resolve({ code: -1, stdout: Buffer.alloc(0), stderr: String(error?.message ?? error), spawnFailed: true })
      return
    }

    const chunks = []
    let bytes = 0
    let stderr = ''
    let settled = false
    const finish = result => {
      if (settled) return
      settled = true
      clearTimeout(timer)
      signal?.removeEventListener?.('abort', onAbort)
      resolve(result)
    }
    const onAbort = () => { try { child.kill() } catch { /* gone */ } ; finish({ code: -1, stdout: Buffer.concat(chunks), stderr: 'aborted', aborted: true }) }
    const timer = setTimeout(() => {
      try { child.kill() } catch { /* gone */ }
      finish({ code: -1, stdout: Buffer.concat(chunks), stderr: stderr || 'timed out', timedOut: true })
    }, timeoutMs)
    timer.unref?.()
    signal?.addEventListener?.('abort', onAbort, { once: true })

    child.stdout?.on('data', chunk => {
      bytes += chunk.length
      if (bytes > maxBytes) {
        try { child.kill() } catch { /* gone */ }
        finish({ code: -1, stdout: Buffer.concat(chunks), stderr: `output exceeded ${maxBytes} bytes`, tooLarge: true })
        return
      }
      chunks.push(chunk)
    })
    child.stderr?.on('data', chunk => { if (stderr.length < 8192) stderr += chunk.toString() })
    child.on('error', error => finish({ code: -1, stdout: Buffer.concat(chunks), stderr: String(error?.message ?? error), spawnFailed: true }))
    child.on('close', code => finish({ code: code ?? -1, stdout: Buffer.concat(chunks), stderr }))
    if (input !== undefined) {
      try { child.stdin.end(input) } catch { /* not a pipe */ }
    }
  })
}

/**
 * Turn a transport failure into something the user can act on.
 *
 * A phone-hotspot car flaps: it disappears from mDNS and comes back. The raw
 * `ssh` text for that is a Windows-encoded "could not resolve hostname", which
 * tells the operator nothing about what to check, so the four cases that
 * actually happen here are named explicitly.
 */
const FAILURES = [
  [/could not resolve hostname|name or service not known|nodename nor servname|temporary failure in name resolution/i,
    '找不到小车主机名（mDNS 解析失败）。确认小车已上电、和本机连的是同一个 WiFi。'],
  [/connection timed out|operation timed out|timed out|no route to host|network is unreachable/i,
    '小车不可达（连接超时）。它可能刚掉线，稍后重试或先测一下连通性。'],
  [/connection refused/i, '小车拒绝了连接：sshd 可能没在运行。'],
  [/permission denied|publickey|password/i, 'SSH 认证失败：把本机公钥装到小车，或检查用户名。'],
  [/host key verification failed|remote host identification/i, '主机密钥与已知记录不符，拒绝连接。'],
]

/**
 * Classify one failed remote command: transport, or the path itself.
 *
 * `ok: false` alone cannot tell the two apart and the HTTP layer must: pointing
 * the browser at a missing directory and finding the car asleep are different
 * failures with different fixes. Transport is ssh's own stderr matching a known
 * FAILURES pattern, or a spawn that never reached the remote at all; everything
 * else is the remote *answering* that the path is wrong.
 *
 * @returns `{ transport, error }` — `transport: true` maps to 503 downstream.
 */
function classifyFailure(result, fallback) {
  if (result.spawnFailed === true) {
    return { transport: true, error: `无法启动 ssh：${String(result.stderr ?? '').split('\n')[0] ?? 'unknown'}` }
  }
  const text = String(result.stderr ?? '').split('\n').map(line => line.trim()).filter(Boolean)
    .filter(line => !/^(warning: permanently added|debug\d*:)/i.test(line))
  const raw = text.join(' ')
  for (const [pattern, message] of FAILURES) {
    if (pattern.test(raw)) return { transport: true, error: message }
  }
  const message = text.slice(-1)[0]
  return { transport: false, error: message === undefined || message === '' ? fallback : message }
}

/**
 * List one directory.
 *
 * @param settings - car address.
 * @param path - absolute remote directory.
 * @returns `{ ok, path, entries, parent, error }`; entries are sorted with
 *   directories first, because that is how a file browser is read.
 */
export async function listDirectory(settings, path, { signal } = {}) {
  const target = String(path ?? '').trim() || '.'
  // `find` prints one record per entry as four NUL-separated fields; `-mindepth 1`
  // excludes the directory itself, and the trailing `||` reports a directory the
  // user cannot enter as an error rather than an empty listing. `-L` makes a
  // symlink report its target's type and size, so a link to a directory is
  // navigable and a link to a large file is not mis-sized; `-maxdepth 1` keeps
  // that from turning into a walk.
  const command = `cd -- ${quote(target)} 2>/dev/null || { echo __DSH_ERR__; exit 3; }; find -L . -mindepth 1 -maxdepth 1 -printf '%f\\0%y\\0%s\\0%T@\\0' 2>/dev/null | base64 -w0; echo; pwd`
  const result = await remote(settings, command, { signal, timeoutMs: 20000 })
  if (result.code !== 0) {
    return { ok: false, ...classifyFailure(result, '目录不可访问'), path: target }
  }

  const text = result.stdout.toString('utf8')
  const newline = text.indexOf('\n')
  const encoded = newline === -1 ? text : text.slice(0, newline)
  const printedPath = newline === -1 ? '' : text.slice(newline + 1).trim()

  let decoded = ''
  try { decoded = Buffer.from(encoded, 'base64').toString('utf8') } catch { decoded = '' }
  const fields = decoded.split('\0')
  const entries = []
  for (let index = 0; index + 3 < fields.length; index += 4) {
    const name = fields[index]
    if (name === '' && fields[index + 1] === '') break
    const kind = fields[index + 1]
    const size = Number(fields[index + 2])
    const mtime = Number(fields[index + 3])
    entries.push({
      name,
      type: kind === 'd' ? 'directory' : kind === 'l' ? 'link' : 'file',
      size: Number.isFinite(size) ? size : 0,
      mtime: Number.isFinite(mtime) ? Math.round(mtime * 1000) : 0,
    })
  }
  entries.sort((left, right) => {
    const leftDir = left.type === 'directory' ? 0 : 1
    const rightDir = right.type === 'directory' ? 0 : 1
    if (leftDir !== rightDir) return leftDir - rightDir
    return left.name.localeCompare(right.name, 'zh')
  })

  const resolved = printedPath === '' ? target : printedPath
  return {
    ok: true,
    path: resolved,
    parent: resolved === '/' ? null : resolved.replace(/\/[^/]+\/?$/, '') || '/',
    entries,
  }
}

/**
 * Stat one remote path.
 *
 * `find -printf` rather than `stat -c`, because GNU `stat`'s format does not
 * expand `\0` into a NUL — a separator that silently does not exist makes every
 * field land in one string and every size read as zero. `-maxdepth 0` limits the
 * walk to the path itself.
 *
 * `-L` dereferences symlinks, and it is not optional: without it `%s` reports the
 * length of a symlink's target *path* — `/usr/bin/Xvnc` comes back as 22 bytes
 * when the file it points at is 2.7 MB. A size guard reading a link's own size
 * does not merely mis-label the file, it lets the reader pull the whole target
 * through. The display name is therefore taken from the argument, not from
 * `%f`, which under `-L` would name the link's target instead.
 */
export async function statPath(settings, path, { signal } = {}) {
  const command = `find -L ${quote(path)} -maxdepth 0 -printf '%y\\0%s\\0%T@\\0' 2>/dev/null | base64 -w0`
  const result = await remote(settings, command, { signal, timeoutMs: 12000 })
  if (result.code !== 0) return { ok: false, ...classifyFailure(result, '文件不存在') }
  const decoded = Buffer.from(result.stdout.toString('utf8').trim(), 'base64').toString('utf8')
  const fields = decoded.split('\0')
  if (fields[0] === '' || fields.length < 3) return { ok: false, error: '文件不存在' }
  const [kind, size, mtime] = fields
  const trimmed = String(path).replace(/\/+$/, '')
  const name = trimmed === '' ? '/' : (trimmed.split('/').pop() || '/')
  return {
    ok: true,
    path: name,
    type: kind === 'd' ? 'directory' : 'file',
    size: Number(size) || 0,
    mtime: Math.round((Number(mtime) || 0) * 1000),
    broken: kind === 'l',
  }
}

/**
 * Read a text file for the viewer.
 *
 * A binary file is refused rather than rendered as mojibake: the caller gets
 * `binary: true` and can offer a download instead.
 */
export async function readTextFile(settings, path, { signal, maxBytes = MAX_READ_BYTES } = {}) {
  const info = await statPath(settings, path, { signal })
  if (!info.ok) return info
  if (info.type === 'directory') return { ok: false, error: '这是一个目录' }
  if (info.size > maxBytes) return { ok: false, error: `文件过大（${info.size} 字节），请下载后查看`, tooLarge: true, size: info.size }

  const result = await remote(settings, `base64 -w0 -- ${quote(path)}`, { signal, timeoutMs: 25000 })
  if (result.code !== 0) return { ok: false, ...classifyFailure(result, '读取失败') }
  const buffer = Buffer.from(result.stdout.toString('utf8').trim(), 'base64')

  // A NUL byte is the cheapest reliable binary tell, but it is not the only one:
  // a few hundred random bytes often contain no NUL at all. So the decoded prefix
  // is also judged on its shape — replacement characters (invalid UTF-8) and
  // control bytes are what actually distinguishes "not text" from "text".
  const prefix = buffer.subarray(0, 8000)
  if (prefix.includes(0)) return { ok: true, binary: true, size: buffer.length, path: info.path }

  const text = buffer.toString('utf8')
  const head = text.slice(0, 8000)
  let suspicious = 0
  for (const character of head) {
    const code = character.codePointAt(0)
    if (code === 0xfffd) suspicious += 2
    else if (code < 9 || (code > 13 && code < 32) || code === 127) suspicious += 1
  }
  if (suspicious * 100 > head.length * 2) {
    return { ok: true, binary: true, size: buffer.length, path: info.path }
  }

  return {
    ok: true,
    binary: false,
    path: info.path,
    size: buffer.length,
    mtime: info.mtime,
    text,
  }
}

/**
 * Stream one remote file to a Node writable.
 *
 * @returns `{ ok, code, stderr }`; on failure nothing has been written to the
 *   destination beyond what already arrived, which the caller reports as a
 *   broken download rather than a complete one.
 */
export function downloadFile(settings, path, destination, { signal, timeoutMs = 15 * 60 * 1000 } = {}) {
  return new Promise(resolve => {
    let child
    try {
      child = spawn('ssh', remoteArgv(settings, `cat -- ${quote(path)}`).slice(1), { stdio: ['ignore', 'pipe', 'pipe'], windowsHide: true })
    } catch (error) {
      resolve({ ok: false, code: -1, stderr: String(error?.message ?? error) })
      return
    }
    let stderr = ''
    let settled = false
    const finish = result => { if (!settled) { settled = true; clearTimeout(timer); resolve(result) } }
    const onAbort = () => { try { child.kill() } catch { /* gone */ } ; finish({ ok: false, code: -1, stderr: 'aborted' }) }
    // A watchdog, because a stalled transfer would otherwise hold the response
    // (and its socket) open indefinitely.
    const timer = setTimeout(() => {
      try { child.kill() } catch { /* gone */ }
      finish({ ok: false, code: -1, stderr: `transfer exceeded ${Math.round(timeoutMs / 1000)}s` })
    }, timeoutMs)
    timer.unref?.()
    signal?.addEventListener?.('abort', onAbort, { once: true })
    child.stderr?.on('data', chunk => { if (stderr.length < 4096) stderr += chunk.toString() })
    child.stdout?.pipe(destination)
    child.on('error', error => finish({ ok: false, code: -1, stderr: String(error?.message ?? error) }))
    child.on('close', code => {
      signal?.removeEventListener?.('abort', onAbort)
      finish({ ok: code === 0, code: code ?? -1, stderr })
    })
  })
}

/**
 * Stream an upload into a remote path.
 *
 * `cat >` truncates on open, so a failed transfer leaves a short file rather
 * than a half-merged one; the caller is told which happened.
 */
export function uploadFile(settings, path, source, { signal, mode = '0644', timeoutMs = 15 * 60 * 1000 } = {}) {
  return new Promise(resolve => {
    let child
    try {
      child = spawn('ssh', remoteArgv(settings, `cat > ${quote(path)} && chmod ${mode} ${quote(path)}`).slice(1), {
        stdio: ['pipe', 'ignore', 'pipe'],
        windowsHide: true,
      })
    } catch (error) {
      resolve({ ok: false, code: -1, stderr: String(error?.message ?? error) })
      return
    }
    let stderr = ''
    let settled = false
    const finish = result => { if (!settled) { settled = true; clearTimeout(timer); resolve(result) } }
    const onAbort = () => { try { child.kill() } catch { /* gone */ } ; finish({ ok: false, code: -1, stderr: 'aborted' }) }
    const timer = setTimeout(() => {
      try { child.kill() } catch { /* gone */ }
      finish({ ok: false, code: -1, stderr: `upload exceeded ${Math.round(timeoutMs / 1000)}s` })
    }, timeoutMs)
    timer.unref?.()
    signal?.addEventListener?.('abort', onAbort, { once: true })
    child.stderr?.on('data', chunk => { if (stderr.length < 4096) stderr += chunk.toString() })
    child.on('error', error => finish({ ok: false, code: -1, stderr: String(error?.message ?? error) }))
    child.on('close', code => {
      signal?.removeEventListener?.('abort', onAbort)
      finish({ ok: code === 0, code: code ?? -1, stderr })
    })
    source.on('error', () => { try { child.kill() } catch { /* gone */ } ; finish({ ok: false, code: -1, stderr: 'upload stream failed' }) })
    source.pipe(child.stdin)
  })
}

/**
 * Common places a node keeps its code, used as browse shortcuts.
 *
 * The car's workspace is `dzrobot_ws`; the drone's are `dzpx4` (PX4 source)
 * and `dzuav` (the UAV workspace) — the drone plugin's own list, so a drone
 * file browser never offers a car path that does not exist on it.
 */
export function homeShortcuts(role = 'car') {
  const common = [
    { label: '主目录', path: '/home/duzhong' },
    { label: '根目录', path: '/' },
  ]
  const project = role === 'drone'
    ? [
        { label: 'PX4 源码', path: '/home/duzhong/dzpx4' },
        { label: 'UAV 工作区', path: '/home/duzhong/dzuav' },
      ]
    : [
        { label: '工作区源码', path: '/home/duzhong/dzrobot_ws' },
      ]
  return [...common, ...project, { label: '临时目录', path: '/tmp' }, { label: '配置', path: '/home/duzhong/.config' }]
}
