/**
 * The VNC bridge: one browser WebSocket ↔ one TCP connection to the car's VNC
 * server.
 *
 * Two ways to reach it, both supported because the car's own configuration can
 * change underneath us:
 *
 *   · `direct`  — connect straight to `<host>:<port>`. What the car does today:
 *     TigerVNC listens on 0.0.0.0:5901.
 *   · `tunnel`  — spawn `ssh -N -L 127.0.0.1:<port>:127.0.0.1:<vncPort>` and
 *     connect to the local end. This is the only thing that works when the VNC
 *     server binds loopback only (the safer car configuration), and it carries
 *     the desktop through the SSH channel that is already trusted.
 *
 * The bridge is a byte pump on purpose: it never inspects the RFB stream, so
 * encodings, extensions and future protocol revisions are noVNC's business
 * rather than something to keep in sync here.
 *
 * @module src/vnc.js
 */

import net from 'node:net'
import dns from 'node:dns'
import { spawn } from 'node:child_process'

/** Hosts/ports a caller may be pointed at without it being a mistake. */
function normalize(settings) {
  const host = String(settings.vncHost ?? '').trim() || String(settings.host ?? '').trim()
  const raw = Number(settings.vncPort)
  const port = Number.isInteger(raw) && raw >= 1 && raw <= 65535 ? raw : 5901
  const mode = settings.vncMode === 'tunnel' ? 'tunnel' : 'direct'
  // The tunnel rides the same SSH target the rest of the plugin uses.
  const sshUser = String(settings.user ?? '').trim()
  const sshPort = Number(settings.port)
  const sshAlias = String(settings.sshAlias ?? '').trim()
  return { host, port, mode, sshUser, sshPort, sshAlias }
}

/** Wait for a TCP port to accept, bounded; used after spawning an SSH tunnel. */
function waitForPort(port, { host = '127.0.0.1', timeoutMs = 8000, signal } = {}) {
  const started = Date.now()
  return new Promise((resolve, reject) => {
    const attempt = () => {
      if (signal?.aborted === true) { reject(new Error('tunnel cancelled')); return }
      const socket = net.connect({ host, port })
      socket.setTimeout(1200)
      const giveUp = error => {
        socket.destroy()
        if (Date.now() - started >= timeoutMs) { reject(error ?? new Error('tunnel did not open in time')); return }
        setTimeout(attempt, 120)
      }
      socket.once('connect', () => { socket.destroy(); resolve() })
      socket.once('timeout', () => giveUp(new Error('tunnel port timed out')))
      socket.once('error', error => giveUp(error))
    }
    attempt()
  })
}

/**
 * The last address that actually answered, per host, with its timestamp.
 *
 * `duzhong.local` is mDNS, and this hotspot's answers drop in and out — a
 * lookup can fail for seconds while the car itself is perfectly up. When the
 * lookup fails, the address that answered most recently is tried first, so a
 * resolution flap degrades to "use what worked" instead of "give up".
 *
 * Only literal addresses that completed a TCP connect are cached, and only for
 * a limited time — a stale entry costs one fast refusal, not a wrong car.
 *
 * The cache is verified against the car before it is trusted, because this car
 * roams between networks (observed: 192.168.43.17 and 10.230.128.188 on the same
 * day). A cached address can therefore be reused by an unrelated host on the new
 * network, and this bridge pumps raw desktop bytes — so a cached address that
 * cannot authenticate as the car is discarded, never connected.
 */
const lastGoodAddress = new Map()
const ADDRESS_CACHE_TTL_MS = 10 * 60_000

/**
 * Dial a TCP port, preferring IPv4.
 *
 * The car's TigerVNC blacklist is keyed by source address, and on this network
 * every entry in it was an IPv6 address — Windows' *temporary* IPv6 addresses,
 * which rotate on their own schedule, so a machine that got blacklisted once
 * stays blacklisted across rotations until the penalty expires. A direct
 * `net.connect` lets the OS pick the family, and on a dual-stack LAN it often
 * picks the blacklisted one. Ordering candidates IPv4-first skips that whole
 * class of refusal without touching the car's own settings.
 *
 * Each candidate gets a short budget so a dead address cannot eat the whole
 * timeout before the healthy family is tried.
 */
/**
 * Seed the address cache for a host.
 *
 * Exported for tests only: the cache is otherwise written solely by a dial that
 * proved the address reachable, which a test cannot do for a name that must
 * fail to resolve (that is the very condition the cache exists to survive).
 * Production code must not call this — a primed-but-unverified address is
 * exactly the risk #awaitBanner exists to catch.
 */
export function __primeAddress(host, address) {
  lastGoodAddress.set(host, { address, at: Date.now() })
}

async function dialOrdered(targetHost, targetPort, { timeoutMs = 8000, signal } = {}) {
  let candidates = [targetHost]
  // Whether the address we are about to use came from the cache rather than from
  // a fresh answer — the caller verifies those before trusting them (see
  // VncBridge#awaitBanner), because a cached address can name a different host
  // once the car moves to another network.
  let suspect = false
  try {
    const resolved = await dns.promises.lookup(targetHost, { all: true })
    if (resolved.length > 0) {
      candidates = [
        ...resolved.filter(entry => entry.family === 4).map(entry => entry.address),
        ...resolved.filter(entry => entry.family === 6).map(entry => entry.address),
      ]
    }
  } catch {
    // Resolution flaked (mDNS). Fall back to the address that answered most
    // recently, then to the raw host — net.connect may still resolve where this
    // call did not, and if neither works the error below says so.
    const cached = lastGoodAddress.get(targetHost)
    if (cached !== undefined && Date.now() - cached.at < ADDRESS_CACHE_TTL_MS) {
      candidates = [cached.address, targetHost]
      suspect = true
    }
  }

  const perAttempt = Math.max(1500, Math.floor(timeoutMs / candidates.length))
  let lastError
  for (const address of candidates) {
    signal?.throwIfAborted?.()
    try {
      const socket = await new Promise((resolve, reject) => {
        const probe = net.connect({ host: address, port: targetPort })
        const fail = error => {
          probe.destroy()
          reject(error ?? new Error('connect failed'))
        }
        probe.setTimeout(perAttempt)
        probe.once('connect', () => { probe.setTimeout(0); resolve(probe) })
        probe.once('timeout', () => fail(new Error(`connect timeout (${address})`)))
        probe.once('error', fail)
      })
      // Remember only literal addresses that proved reachable.
      if (net.isIPv4(address) || net.isIPv6(address)) {
        lastGoodAddress.set(targetHost, { address, at: Date.now() })
      }
      return { socket, address, suspect: suspect && address === candidates[0] }
    } catch (error) {
      lastError = error
      // A connect refusal on one family is the signal to try the other, not to
      // give up — but if the user cancelled, stop entirely.
      if (signal?.aborted === true) throw error
    }
  }
  throw new Error(`无法连接 ${targetHost}:${targetPort}（${candidates.length} 个地址均失败）：${lastError?.message ?? '未知错误'}`)
}

/** Ask the OS for a free loopback port by binding and immediately releasing it. */
function freePort() {
  return new Promise((resolve, reject) => {
    const probe = net.createServer()
    probe.unref()
    probe.once('error', reject)
    probe.listen(0, '127.0.0.1', () => {
      const { port } = probe.address()
      probe.close(() => resolve(port))
    })
  })
}

/**
 * One open VNC bridge.
 *
 * `open()` resolves with a handle exposing `send`, `onData` and `close`. The
 * caller (the WebSocket side) owns the framing; this owns the bytes and, in
 * tunnel mode, the SSH process that must not outlive the connection.
 */
export class VncBridge {
  #settings
  #log
  #socket
  #tunnel
  #closed = false
  #onData
  #onClose
  /** Bytes held back by the banner check, flushed once the browser is listening. */
  #pending

  constructor(settings, { logger } = {}) {
    this.#settings = normalize(settings)
    this.#log = logger ?? {}
  }

  /** What this bridge will connect to, for the status line. */
  describe() {
    const { host, port, mode } = this.#settings
    return mode === 'tunnel' ? `ssh 隧道 → ${host}:${port}` : `${host}:${port}`
  }

  async open({ signal } = {}) {
    const { host, port, mode } = this.#settings
    if (host === '') throw new Error('没有配置 VNC 主机')

    let targetHost = host
    let targetPort = port

    if (mode === 'tunnel') {
      const local = await freePort()
      const current = this.#settings
      const sshArgs = ['-N', '-o', 'BatchMode=yes', '-o', 'ExitOnForwardFailure=yes', '-o', 'ServerAliveInterval=30', '-o', 'StrictHostKeyChecking=accept-new']
      if (Number.isInteger(current.sshPort) && current.sshPort !== 22) sshArgs.push('-p', String(current.sshPort))
      const user = String(current.sshUser ?? '').trim()
      const destination = String(current.sshAlias ?? '').trim() || (user === '' ? host : `${user}@${host}`)
      sshArgs.push('-L', `127.0.0.1:${local}:127.0.0.1:${port}`, destination)

      const child = spawn('ssh', sshArgs, { stdio: ['ignore', 'ignore', 'pipe'], windowsHide: true })
      let stderr = ''
      child.stderr?.on('data', chunk => { stderr += chunk.toString() })
      child.on('error', error => this.#log.warn?.(`dzrobot-ssh vnc: ssh tunnel failed (${error?.message ?? error})`))
      this.#tunnel = child

      try {
        await waitForPort(local, { signal })
      } catch (error) {
        const detail = stderr.split('\n').map(line => line.trim()).filter(Boolean).slice(-2).join('; ')
        this.#dispose()
        throw new Error(`SSH 隧道未建立：${detail || (error?.message ?? error)}`)
      }
      targetHost = '127.0.0.1'
      targetPort = local
    }

    // IPv4-first dial (see dialOrdered): on this LAN the blacklisted addresses
    // were all IPv6, so family order decides whether the car accepts us at all.
    let dialed
    try {
      dialed = await dialOrdered(targetHost, targetPort, { timeoutMs: 8000, signal })
    } catch (error) {
      // A dial failure in tunnel mode must not leave the spawned ssh running.
      this.#dispose()
      throw error
    }
    const { socket, address, suspect } = dialed
    socket.setNoDelay?.(true)

    // Verify the peer before forwarding a single byte, whenever the address is a
    // literal rather than the name the user configured. A configured hostname is
    // the user's own assertion about where the car is; a bare address is not,
    // and this car roams between networks (observed: 192.168.43.17 and
    // 10.230.128.188 on one day), so an old address can name a different host.
    // The banner check is also what retires a stale cache entry.
    if (suspect || address !== targetHost) {
      this.#socket = socket
      try {
        await this.#awaitBanner(address, { signal })
      } catch (error) {
        this.#dispose()
        throw error
      }
    }

    this.#socket = socket
    socket.on('data', chunk => this.#onData?.(chunk))
    socket.on('error', error => {
      this.#log.warn?.(`dzrobot-ssh vnc: socket error (${error?.message ?? error})`)
      this.#finish()
    })
    socket.on('close', () => this.#finish())

    // Release what the banner check held, now that the socket is actually
    // wired for forwarding. A server never repeats its banner, so skipping this
    // would stall the RFB handshake on a healthy connection.
    if (this.#pending !== undefined) {
      const held = this.#pending
      this.#pending = undefined
      try { this.#onData?.(held) } catch { /* browser gone */ }
    }

    this.#log.info?.(`dzrobot-ssh vnc: bridged ${this.describe()} via ${address}`)
    return this
  }

  /**
   * Read the RFB version banner before releasing any bytes to the browser.
   *
   * Every VNC server sends it first, so receiving it proves a VNC server is on
   * the other end — enough to discard a stale cached address that a different
   * host has since taken. This is deliberately not a stronger identity check:
   * the security handshake is what proves the *car*, and it happens in the
   * browser through noVNC. A peer that fails this gate gets nothing forwarded.
   */
  #awaitBanner(address, { signal, timeoutMs = 5000 } = {}) {
    const socket = this.#socket
    return new Promise((resolve, reject) => {
      let seen = ''
      // A peer that accepts the TCP connection and then says nothing would
      // otherwise hold the browser's WebSocket open forever with no desktop and
      // no error, so the wait is bounded like every other dial step.
      const timer = setTimeout(() => fail(new Error('等待 RFB 横幅超时')), timeoutMs)
      const fail = error => {
        cleanup()
        clearTimeout(timer)
        lastGoodAddress.delete(this.#settings.host)
        reject(new Error(`地址 ${address} 不是这台车的 VNC 服务（缓存已失效）：${error?.message ?? error}`))
      }
      const onData = chunk => {
        seen += chunk.toString('latin1')
        const end = seen.indexOf('\n')
        if (end === -1) {
          if (seen.length > 32) fail(new Error('没有收到 RFB 版本行'))
          return
        }
        const line = seen.slice(0, end).trim()
        if (!/^RFB \d{3}\.\d{3}$/.test(line)) {
          fail(new Error(`收到 ${JSON.stringify(line)} 而非 RFB 横幅`))
          return
        }
        cleanup()
        // RFB servers never repeat their banner, and noVNC waits for it before
        // sending its own version, so the held bytes must not be lost.
        //
        // They are only buffered, never sent here: the forwarding listener is
        // attached after this check returns, and the Host registers its sink
        // before calling open(). Buffering is therefore the only ordering that
        // cannot drop them — onData() flushes once the socket is wired.
        this.#pending = Buffer.from(seen.slice(0, end + 1), 'latin1')
        resolve()
      }
      const onEnd = () => fail(new Error('对方未发送 RFB 横幅就关闭了连接'))
      const onError = error => fail(new Error(`连接在握手前出错：${error?.message ?? error}`))
      const cleanup = () => {
        socket.off('data', onData)
        socket.off('end', onEnd)
        socket.off('close', onEnd)
        // Removed, not replaced: #dispose() destroys the socket right after a
        // failed check, and that reset must not escape as an unhandled 'error'.
        socket.off('error', onError)
        signal?.removeEventListener?.('abort', onEnd)
      }
      socket.on('data', onData)
      socket.once('end', onEnd)
      socket.once('close', onEnd)
      socket.once('error', onError)
      signal?.addEventListener?.('abort', onEnd, { once: true })
    })
  }

  /** Forward browser bytes to the car. */
  send(payload) {
    if (this.#closed || this.#socket === undefined) return true
    return this.#socket.write(payload)
  }

  /**
 * Register where socket bytes go.
 *
 * The sink must exist before open() is called; the host wires it first so that
 * nothing can arrive unhandled. Bytes held back by the banner check are flushed
 * by open() itself, after the socket is wired — not here, because this method
 * normally runs before that socket even exists.
 */
onData(handler) { this.#onData = handler }
  onClose(handler) { this.#onClose = handler }

  close() {
    this.#dispose()
  }

  #dispose() {
    const socket = this.#socket
    this.#socket = undefined
    this.#pending = undefined
    if (socket !== undefined) {
      // Destroying a socket that the peer already reset raises ECONNRESET; with
      // no listener that becomes an unhandled 'error' event and kills the Host,
      // so the throw-away socket is silenced before it is torn down.
      socket.on('error', () => {})
      try { socket.destroy() } catch { /* already gone */ }
    }
    try { this.#tunnel?.kill() } catch { /* already gone */ }
    this.#tunnel = undefined
  }

  #finish() {
    if (this.#closed) return
    this.#closed = true
    this.#dispose()
    this.#onClose?.()
  }
}

/** Resolve the VNC target from settings, for the status API. */
export function vncTarget(settings) {
  const { host, port, mode } = normalize(settings)
  return { host, port, mode, label: mode === 'tunnel' ? `ssh 隧道 → ${host}:${port}` : `${host}:${port}` }
}
