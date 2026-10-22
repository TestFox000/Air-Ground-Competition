/**
 * A tiny JSON-file store: the settings the sidebar reads and the settings page
 * writes, persisted under the dsh home so a reload keeps them.
 *
 * Deliberately dependency-free and synchronous on write — the values are a few
 * dozen bytes and the whole point of the page is that a change is visible to the
 * next connect attempt immediately.
 */

import fs from 'node:fs'
import os from 'node:os'
import path from 'node:path'

/** dsh's home directory, mirroring how the host resolves its own. */
export function resolveDshHome() {
  const fromEnv = process.env.DSH_HOME
  if (typeof fromEnv === 'string' && fromEnv.trim() !== '') return fromEnv.trim()
  return path.join(os.homedir(), '.dsh')
}

export const DATA_DIR_NAME = 'dzrobot-ssh'

/**
 * Defaults describe the machine this plugin was written on: the car answers to
 * `duzhong.local`, the user is `duzhong`, and the key that reaches it is the
 * ordinary `~/.ssh/id_rsa` — so a first install connects with no setup at all.
 */
export const SETTINGS_INITIAL = {
  version: 1,
  host: 'duzhong.local',
  port: 22,
  user: 'duzhong',
  /** Empty means "use the user's own ssh config / key", i.e. plain `ssh <target>`. */
  sshAlias: '',
  /** Seconds before a connect attempt is declared unreachable. */
  probeTimeoutMs: 2500,
  /** Extra ssh options, space-separated, appended verbatim to the command line. */
  sshOptions: '-o ServerAliveInterval=30 -o ServerAliveCountMax=6',
  /**
   * VNC: the car runs TigerVNC on display :1, i.e. port 5901 — not the 5900 a
   * default setup would suggest. Empty `vncHost` follows the SSH host.
   */
  vncHost: '',
  vncPort: 5901,
  /**
   * `direct` connects straight to the VNC port (what the car does today, it
   * binds 0.0.0.0); `tunnel` goes through `ssh -L`, which is the only mode that
   * works when that server binds loopback only.
   */
  vncMode: 'direct',
  /** Optional VNC password; empty leaves noVNC to prompt for it. */
  vncPassword: '',
  /** Where the file browser starts. */
  filesRoot: '/home/duzhong',
  /**
   * Status float opacity, percent (100 = solid). A saved choice, not a per-session
   * tweak: the card's slider and the settings page both write it through `/config`.
   * The floor is 30 — below that the card's own controls become hard to hit.
   */
  floatOpacity: 100,
  lastConnectedAt: 0,
}

export class JsonStore {
  #file
  #state
  #log
  #timer = null

  constructor(file, initial, { log } = {}) {
    this.#file = file
    this.#log = log ?? (() => {})
    this.#state = JsonStore.read(file, initial, this.#log)
  }

  static read(file, initial, log) {
    try {
      const raw = fs.readFileSync(file, 'utf8')
      const parsed = JSON.parse(raw)
      if (parsed !== null && typeof parsed === 'object' && !Array.isArray(parsed)) {
        // Shallow-merge onto the initial so a file written by an older version
        // still answers every field the current code reads.
        return { ...initial, ...parsed }
      }
      log(`dzrobot-ssh: ${file} is not an object; starting from defaults`)
    } catch (error) {
      if (error?.code !== 'ENOENT') log(`dzrobot-ssh: could not read ${file} (${error?.message ?? error}); starting from defaults`)
    }
    return { ...initial }
  }

  get() {
    return this.#state
  }

  /** Returns the merged view of a partial patch, without writing. */
  peek(patch) {
    return { ...this.#state, ...patch }
  }

  update(patch) {
    this.#state = { ...this.#state, ...patch }
    this.flush()
    return this.#state
  }

  /**
   * Coerce a posted patch.
   *
   * The settings page is the only writer and it is a text form, so a cleared
   * numeric field posts `''`. NaN would reach ssh as the string "NaN"; a port
   * outside 1..65535 is a refusal rather than something to clamp, so the caller
   * rejects it and the user sees which field was wrong.
   */
  sanitize(patch) {
    const next = { ...patch }
    if (next.host !== undefined) next.host = String(next.host ?? '').trim()
    if (next.user !== undefined) next.user = String(next.user ?? '').trim()
    if (next.sshAlias !== undefined) next.sshAlias = String(next.sshAlias ?? '').trim()
    if (next.sshOptions !== undefined) next.sshOptions = String(next.sshOptions ?? '').trim()
    if (next.vncHost !== undefined) next.vncHost = String(next.vncHost ?? '').trim()
    if (next.vncMode !== undefined) next.vncMode = next.vncMode === 'tunnel' ? 'tunnel' : 'direct'
    if (next.vncPassword !== undefined) next.vncPassword = String(next.vncPassword ?? '')
    if (next.filesRoot !== undefined) {
      const root = String(next.filesRoot ?? '').trim()
      // The browser joins this with a relative name, so it has to be absolute and
      // free of traversal before it is ever handed to a remote shell.
      next.filesRoot = root.startsWith('/') && !root.includes('..') ? root.replace(/\/+$/, '') || '/' : SETTINGS_INITIAL.filesRoot
    }
    if (next.vncPort !== undefined) {
      const vncPort = Number(next.vncPort)
      next.vncPort = Number.isInteger(vncPort) && vncPort >= 1 && vncPort <= 65535 ? vncPort : SETTINGS_INITIAL.vncPort
    }
    if (next.port !== undefined) {
      const port = Number(next.port)
      next.port = Number.isInteger(port) && port >= 1 && port <= 65535 ? port : SETTINGS_INITIAL.port
    }
    if (next.probeTimeoutMs !== undefined) {
      const value = Number(next.probeTimeoutMs)
      next.probeTimeoutMs = Number.isFinite(value) && value > 0 ? Math.min(60_000, Math.trunc(value)) : SETTINGS_INITIAL.probeTimeoutMs
    }
    if (next.floatOpacity !== undefined) {
      // A slider posts a number, but `/config` takes arbitrary JSON: an
      // out-of-range or non-numeric value falls back to the default instead of
      // rendering a card at `opacity: NaN` (i.e. invisible-but-clickable).
      const value = Number(next.floatOpacity)
      next.floatOpacity = Number.isFinite(value)
        ? Math.min(100, Math.max(30, Math.round(value)))
        : SETTINGS_INITIAL.floatOpacity
    }
    return next
  }

  flush() {
    if (this.#timer !== null) return
    // Coalesce bursts (a settings POST that touches several keys at once) into
    // one write, but never let the value the caller just read be older than disk
    // for longer than a tick.
    this.#timer = setTimeout(() => {
      this.#timer = null
      this.#writeNow()
    }, 120)
    this.#timer.unref?.()
  }

  #writeNow() {
    try {
      fs.mkdirSync(path.dirname(this.#file), { recursive: true })
      const tmp = `${this.#file}.tmp`
      fs.writeFileSync(tmp, JSON.stringify(this.#state, null, 2), 'utf8')
      fs.renameSync(tmp, this.#file)
    } catch (error) {
      this.#log(`dzrobot-ssh: could not persist settings (${error?.message ?? error})`)
    }
  }

  dispose() {
    if (this.#timer !== null) {
      clearTimeout(this.#timer)
      this.#timer = null
      this.#writeNow()
    }
  }
}