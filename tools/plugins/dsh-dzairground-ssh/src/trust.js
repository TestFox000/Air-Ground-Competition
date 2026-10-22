/**
 * The trust fence for this plugin's HTTP routes.
 *
 * `/api/dzairground-ssh/*` runs shell commands, so it must answer only callers the
 * app itself would admit. Two checks, in order:
 *
 *   1. the connection service's own admission, when the composition mounts it —
 *      the same decision the kernel applies to `/api`; and
 *   2. a structural fallback for a composition without that service: a loopback
 *      peer only.
 *
 * The fallback is deliberately the stricter of the two. `connection` is looked
 * up per request because plugins load before the browser-facing half publishes
 * it, and a one-shot read at apply time would freeze in "absent" — which here
 * would silently downgrade the fence for the process lifetime.
 */

/** True when an address is loopback (IPv4, IPv6, or a mapped IPv6 form). */
export function isLoopback(address) {
  const value = String(address ?? '').trim().toLowerCase()
  if (value === '') return false
  if (value === '::1' || value === '::ffff:127.0.0.1') return true
  if (value.startsWith('::ffff:')) return isLoopback(value.slice(7))
  return value === '127.0.0.1' || value.startsWith('127.')
}

/**
 * Decide one request.
 *
 * @param req - node request; `socket.remoteAddress` is the peer.
 * @param connection - the connection service, or a getter that answers it now.
 * @returns `undefined` to admit, otherwise the HTTP status to refuse with.
 */
export function rejectionFor(req, connection) {
  const service = typeof connection === 'function' ? connection() : connection
  if (service !== undefined && typeof service.admit === 'function') {
    try {
      const admission = service.admit(req)
      // The service answers either a boolean or an object with `admitted`.
      if (typeof admission === 'boolean') return admission ? undefined : 403
      if (admission !== null && typeof admission === 'object') {
        if (admission.admitted === true) return undefined
        if (admission.admitted === false) return admission.status ?? 403
      }
    } catch {
      // A throwing admission is a refusal, never a pass.
      return 403
    }
  }
  const peer = req?.socket?.remoteAddress ?? req?.connection?.remoteAddress
  return isLoopback(peer) ? undefined : 403
}
