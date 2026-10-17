/**
 * A minimal RFC 6455 WebSocket server, written because the host's upgrade seat
 * hands over a raw TCP socket rather than a framed connection.
 *
 * `webServer.registerUpgrade` gives the plugin `(req, socket, head)` and says
 * the handler owns negotiation plus socket use — so the handshake and the frame
 * codec are the plugin's to implement. This is deliberately the small surface a
 * byte-stream tunnel needs:
 *
 *   · the handshake, echoing the client's sub-protocol (noVNC asks for none by
 *     default, but asks for `binary` when configured to);
 *   · binary frames in both directions, with 7/16/64-bit lengths;
 *   · continuation frames, because a large framebuffer update can arrive split;
 *   · ping/pong/close, so a dead peer is noticed rather than half-open forever.
 *
 * Compression is not negotiated: the client must not assume it, and a VNC
 * framebuffer is already the wrong thing to inflate twice.
 *
 * @module src/ws.js
 */

import crypto from 'node:crypto'

const GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'

/** Frame opcodes this implementation acts on. */
const OP_CONTINUATION = 0x0
const OP_TEXT = 0x1
const OP_BINARY = 0x2
const OP_CLOSE = 0x8
const OP_PING = 0x9
const OP_PONG = 0xa

/** Largest single message this tunnel will assemble, as a runaway guard. */
const MAX_MESSAGE_BYTES = 64 * 1024 * 1024

/**
 * Complete the opening handshake.
 *
 * @param req - the upgrade request.
 * @param socket - the raw socket to answer on.
 * @returns the negotiated sub-protocol, or `null` when the request is not a
 *   usable WebSocket upgrade (the socket is already destroyed in that case).
 */
export function acceptUpgrade(req, socket) {
  const key = req.headers['sec-websocket-key']
  const version = Number(req.headers['sec-websocket-version'] ?? 0)
  const upgrade = String(req.headers.upgrade ?? '').toLowerCase()
  if (upgrade !== 'websocket' || typeof key !== 'string' || key === '' || version !== 13) {
    socket.write('HTTP/1.1 400 Bad Request\r\nConnection: close\r\n\r\n')
    socket.destroy()
    return null
  }

  // Only the sub-protocols the caller offered may be echoed back: replying with
  // one the client did not ask for is a protocol violation browsers enforce.
  const offered = String(req.headers['sec-websocket-protocol'] ?? '')
    .split(',')
    .map(value => value.trim())
    .filter(value => value !== '')
  const chosen = offered.includes('binary') ? 'binary' : (offered.includes('base64') ? 'base64' : null)

  const accept = crypto.createHash('sha1').update(key + GUID).digest('base64')
  const headers = [
    'HTTP/1.1 101 Switching Protocols',
    'Upgrade: websocket',
    'Connection: Upgrade',
    `Sec-WebSocket-Accept: ${accept}`,
  ]
  if (chosen !== null) headers.push(`Sec-WebSocket-Protocol: ${chosen}`)
  socket.write(`${headers.join('\r\n')}\r\n\r\n`)
  socket.setNoDelay?.(true)
  return chosen ?? ''
}

/**
 * A framed channel over one accepted socket.
 *
 * Constructed with `new WebSocketChannel(socket, protocol, head)`; `head` is the
 * bytes the HTTP parser had already read past the headers, which must be fed to
 * the frame decoder first or the first VNC message is lost.
 */
export class WebSocketChannel {
  #socket
  #protocol
  #buffer
  #fragments = []
  #fragmentBytes = 0
  #closed = false
  #onMessage
  #onClose
  #waiting = null

  constructor(socket, protocol, head) {
    this.#socket = socket
    this.#protocol = protocol
    this.#buffer = head !== undefined && head.length > 0 ? Buffer.from(head) : Buffer.alloc(0)
    socket.on('data', chunk => {
      this.#buffer = this.#buffer.length === 0 ? chunk : Buffer.concat([this.#buffer, chunk])
      this.#drain()
    })
    socket.on('error', () => this.#finish())
    socket.on('close', () => this.#finish())
  }

  /** The negotiated sub-protocol (empty string when none was offered). */
  get protocol() {
    return this.#protocol
  }

  get closed() {
    return this.#closed
  }

  onMessage(handler) {
    this.#onMessage = handler
    this.#drain()
  }

  onClose(handler) {
    this.#onClose = handler
  }

  /**
   * Send one binary message.
   *
   * Returns `false` when the kernel buffer is full; the caller must then await
   * {@link drained} before sending more, or a slow VNC client would make the
   * drone's framebuffer queue up in this process without bound.
   */
  send(payload) {
    if (this.#closed) return true
    const body = Buffer.isBuffer(payload) ? payload : Buffer.from(payload)
    const header = frameHeader(body.length)
    try {
      return this.#socket.write(header) && this.#socket.write(body)
    } catch {
      this.#finish()
      return false
    }
  }

  /** Resolves when the socket has flushed its backlog. */
  drained() {
    if (this.#closed) return Promise.resolve()
    return new Promise(resolve => this.#socket.once('drain', resolve))
  }

  /** Close with a normal-closure status, once. */
  close(code = 1000, reason = '') {
    if (this.#closed) return
    const text = Buffer.from(String(reason))
    const payload = Buffer.alloc(2 + text.length)
    payload.writeUInt16BE(code, 0)
    text.copy(payload, 2)
    try {
      this.#socket.write(Buffer.concat([frameHeader(payload.length, OP_CLOSE), payload]))
    } catch { /* peer already gone */ }
    this.#finish()
    try { this.#socket.end() } catch { /* already ended */ }
    try { this.#socket.destroy() } catch { /* already destroyed */ }
  }

  #finish() {
    if (this.#closed) return
    this.#closed = true
    this.#buffer = Buffer.alloc(0)
    this.#fragments = []
    try { this.#socket.destroy() } catch { /* already destroyed */ }
    this.#onClose?.()
  }

  /** Decode whatever whole frames the buffer currently holds. */
  #drain() {
    while (!this.#closed) {
      const frame = decodeFrame(this.#buffer)
      if (frame === null) return
      this.#buffer = this.#buffer.subarray(frame.consumed)
      this.#handleFrame(frame)
    }
  }

  #handleFrame(frame) {
    switch (frame.opcode) {
      case OP_PING:
        try { this.#socket.write(Buffer.concat([frameHeader(frame.payload.length, OP_PONG), frame.payload])) } catch { /* gone */ }
        return
      case OP_PONG:
        return
      case OP_CLOSE:
        this.close(1000, '')
        return
      case OP_TEXT:
      case OP_BINARY:
        if (frame.fin) {
          this.#deliver(frame.payload)
          return
        }
        this.#fragments = [frame.payload]
        this.#fragmentBytes = frame.payload.length
        return
      case OP_CONTINUATION: {
        if (this.#fragments.length === 0) return
        this.#fragments.push(frame.payload)
        this.#fragmentBytes += frame.payload.length
        if (this.#fragmentBytes > MAX_MESSAGE_BYTES) { this.close(1009, 'message too large'); return }
        if (frame.fin) {
          const joined = Buffer.concat(this.#fragments, this.#fragmentBytes)
          this.#fragments = []
          this.#fragmentBytes = 0
          this.#deliver(joined)
        }
        return
      }
      default:
        return
    }
  }

  #deliver(payload) {
    if (this.#onMessage === undefined) return
    let value = payload
    if (this.#protocol === 'base64') {
      // websockify's text fallback: each message is base64 of the raw bytes.
      const text = payload.toString('utf8')
      if (text === '') return
      value = Buffer.from(text, 'base64')
    }
    try { this.#onMessage(value) } catch { /* a consumer fault must not kill the tunnel */ }
  }
}

/** Header bytes for one unmasked server-to-client frame. */
function frameHeader(length, opcode = OP_BINARY) {
  if (length < 126) {
    const header = Buffer.alloc(2)
    header[0] = 0x80 | opcode
    header[1] = length
    return header
  }
  if (length < 65536) {
    const header = Buffer.alloc(4)
    header[0] = 0x80 | opcode
    header[1] = 126
    header.writeUInt16BE(length, 2)
    return header
  }
  const header = Buffer.alloc(10)
  header[0] = 0x80 | opcode
  header[1] = 127
  header.writeBigUInt64BE(BigInt(length), 2)
  return header
}

/**
 * Decode one frame from the front of `buffer`.
 *
 * Returns `null` while the frame is still incomplete, otherwise the payload plus
 * how many bytes it consumed. Client frames must be masked; an unmasked one is
 * treated as incomplete rather than trusted, which is what the RFC requires a
 * server to do.
 */
function decodeFrame(buffer) {
  if (buffer.length < 2) return null
  const first = buffer[0]
  const second = buffer[1]
  const fin = (first & 0x80) !== 0
  const opcode = first & 0x0f
  const masked = (second & 0x80) !== 0
  let length = second & 0x7f
  let offset = 2

  if (length === 126) {
    if (buffer.length < offset + 2) return null
    length = buffer.readUInt16BE(offset)
    offset += 2
  } else if (length === 127) {
    if (buffer.length < offset + 8) return null
    const big = buffer.readBigUInt64BE(offset)
    if (big > BigInt(MAX_MESSAGE_BYTES)) return null
    length = Number(big)
    offset += 8
  }
  if (!masked) return null
  if (buffer.length < offset + 4) return null
  const mask = buffer.subarray(offset, offset + 4)
  offset += 4
  if (buffer.length < offset + length) return null

  const payload = Buffer.allocUnsafe(length)
  const source = buffer.subarray(offset, offset + length)
  for (let index = 0; index < length; index += 1) payload[index] = source[index] ^ mask[index & 3]
  return { fin, opcode, payload, consumed: offset + length }
}
