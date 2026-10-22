/**
 * Self hot-reload: replace the running plugin with the code now on disk, without
 * restarting the application.
 *
 * Why this exists: Node caches an ES module by resolved URL for the life of the
 * process, so editing this package's host half changes nothing until the app is
 * restarted. The kernel's HMR service reloads *its own* packages by watching
 * files; it ships with that watching off and offers no "reload this plugin now"
 * API. So the sequence is performed here, against the same primitives the HMR
 * service itself uses:
 *
 *   1. snapshot the fiber(s) of this runtime and the config each was created with;
 *   2. evict this package's entries from the loader's ESM load cache (and from
 *      `require.cache`, where a module also exists on the CJS side);
 *   3. re-import the entry file so the new sources evaluate — falling back to a
 *      one-off query string when the eviction did not take, which is the case on
 *      Node versions whose load cache is no longer a plain Map;
 *   4. dispose the old fibers and register the replacement under the same config;
 *   5. on any failure, restore the caches and the previous registration, so the
 *      swap ends with either the old version or the new one running, never neither.
 *
 * @module src/reload.js
 */

import fs from 'node:fs'
import { fileURLToPath } from 'node:url'
import { createRequire } from 'node:module'

let inFlight = null

/** Whether a reload is currently running; guards against overlapping swaps. */
export function isReloading() {
  return inFlight !== null
}

/** This package's directory, as a `file://` URL with a trailing slash. */
export function packageUrlOf(entryModuleUrl) {
  return new URL('../', entryModuleUrl).href
}

/** This package's entry file, as an absolute `file://` URL. */
export function entryUrlOf(entryModuleUrl) {
  return new URL('../index.js', entryModuleUrl).href
}

/**
 * Every load-cache key living inside the package.
 *
 * Comparison tolerates case and percent-encoding drift between Node's resolver
 * and `new URL`, because a key that does not match is a module that silently
 * stays cached — the reload would then appear to succeed while running old code.
 */
export function packageCacheKeys(loadCache, packageUrl) {
  const base = `${decodeURIComponent(packageUrl).replace(/\/+$/, '').toLowerCase()}/`
  const keys = []
  for (const key of loadCache.keys()) {
    if (typeof key !== 'string') continue
    let normalized
    try { normalized = decodeURIComponent(key).toLowerCase() } catch { normalized = key.toLowerCase() }
    if (normalized.startsWith(base)) keys.push(key)
  }
  return keys
}

/**
 * Back up and remove this package's modules from both module caches.
 *
 * The load cache is accessed duck-typed: newer Node releases replaced the Map
 * subclass with their own structure, so `instanceof Map` is false there even
 * though `keys`/`get`/`delete` still behave like one. Where it really is a Map,
 * `Map.prototype` methods are used directly — Node 24's own `LoadCache.delete`
 * only clears a type slot and leaves the module in place.
 *
 * @returns `{ esm, cjs }` backups, restorable by {@link restoreCaches}.
 */
export function purgeCaches(internal, packageUrl) {
  const backup = { esm: new Map(), cjs: new Map() }
  const cache = internal?.loadCache
  if (cache !== null && cache !== undefined
    && typeof cache.keys === 'function' && typeof cache.get === 'function' && typeof cache.delete === 'function') {
    const read = key => {
      try { return Map.prototype.get.call(cache, key) } catch { try { return cache.get(key) } catch { return undefined } }
    }
    const drop = key => {
      try { Map.prototype.delete.call(cache, key); return } catch { /* not a real Map */ }
      try { cache.delete(key) } catch { /* best effort */ }
    }
    for (const key of packageCacheKeys(cache, packageUrl)) {
      backup.esm.set(key, read(key))
      drop(key)
    }
  }
  const require = createRequire(import.meta.url)
  for (const key of backup.esm.keys()) {
    try {
      const filePath = fileURLToPath(key)
      if (require.cache[filePath] !== undefined) {
        backup.cjs.set(filePath, require.cache[filePath])
        delete require.cache[filePath]
      }
    } catch {
      // a non-file: URL has no CJS side to purge
    }
  }
  return backup
}

/** Put a purge's backup back (the rollback path). */
export function restoreCaches(internal, backup) {
  const cache = internal?.loadCache
  for (const [key, job] of backup.esm) {
    try { Map.prototype.set.call(cache, key, job) } catch { try { cache.set(key, job) } catch { /* best effort */ } }
  }
  const require = createRequire(import.meta.url)
  for (const [filePath, module] of backup.cjs) require.cache[filePath] = module
}

/**
 * Reload the calling plugin instance in place.
 *
 * Runs from a plain call outside the fiber being disposed, so disposing this
 * plugin's own effects cannot cancel the swap half-way.
 *
 * @param ctx - the context `apply` received.
 * @returns the outcome; `ok: false` always leaves the previous version running.
 */
export function selfReload(ctx, options = {}) {
  if (inFlight !== null) return inFlight
  inFlight = (async () => {
    const logger = options.logger ?? console
    const packageUrl = options.packageUrl ?? packageUrlOf(import.meta.url)
    const entryUrl = options.entryUrl ?? entryUrlOf(import.meta.url)
    const loader = ctx.loader
    const registry = ctx.registry
    if (loader === undefined || registry === undefined) {
      return { ok: false, error: 'this context exposes no loader/registry to reload through', restored: false }
    }

    // The plugin identity is the exports object the registry keyed this runtime by.
    const previous = ctx.fiber?.runtime?.callback
    if (previous === undefined || previous === null) {
      return { ok: false, error: 'this plugin was not registered the ordinary way; nothing to swap', restored: false }
    }
    const runtime = registry.get(previous)
    const fibers = [...(runtime?.fibers ?? [])].map(fiber => {
      const entry = fiber.entry?.fiber?.uid === fiber.uid ? fiber.entry : undefined
      return { fiber, entry, config: entry === undefined ? fiber._config : entry.options.config }
    })
    if (fibers.length === 0) return { ok: false, error: 'this runtime has no live fiber', restored: false }

    const internal = loader.internal
    const backup = purgeCaches(internal, packageUrl)
    logger.info?.(`dzdrone-ssh: hot reload evicted ${backup.esm.size} cached module(s)`)

    const importFresh = async url => loader.unwrapExports(await loader.import(url, () => []))
    let replacement
    let busted = false
    try {
      replacement = await importFresh(entryUrl)
      if (replacement === undefined || typeof replacement.apply !== 'function') {
        throw new Error('the reloaded module does not export apply()')
      }
      if (replacement === previous) {
        // The eviction did not take on this kernel, so force a fresh evaluation
        // of the entry under a one-off query; its relative imports still resolve
        // to the evicted URLs, which is what makes this a real reload.
        replacement = await importFresh(`${entryUrl}?dzr=${Date.now()}`)
        busted = true
        if (replacement === undefined || typeof replacement.apply !== 'function') {
          throw new Error('the reloaded module does not export apply()')
        }
      }
    } catch (error) {
      restoreCaches(internal, backup)
      return { ok: false, error: `re-import failed, the old code is still running (${error?.message ?? error})`, restored: true }
    }
    globalThis[Symbol.for('dzdrone-ssh.last-reload')] = { at: Date.now(), evicted: backup.esm.size, busted }

    try {
      registry.delete(previous)
      await Promise.all(fibers.map(({ fiber }) => fiber.await()))
    } catch (error) {
      // The old fibers are gone either way; the new registration below is still
      // the right move, so this is reported rather than propagated.
      logger.warn?.(`dzdrone-ssh: hot reload dispose reported ${error?.message ?? error}`)
    }

    try {
      const created = []
      for (const { fiber, entry, config } of fibers) {
        const registered = fiber.parent.registry.plugin(replacement, config)
        const fresh = registered.ctx.fiber
        if (entry !== undefined) {
          fresh.entry = entry
          entry.fiber = fresh
        }
        created.push(fresh)
      }
      await Promise.all(created.map(fiber => fiber.await()))
      logger.info?.(`dzdrone-ssh: hot reload complete, ${created.length} fiber(s) re-registered`)
      return { ok: true, fibers: created.length, evicted: backup.esm.size, busted }
    } catch (error) {
      restoreCaches(internal, backup)
      try { registry.delete(replacement) } catch { /* may never have started */ }
      try {
        const restored = []
        for (const { fiber, entry, config } of fibers) {
          const registered = fiber.parent.registry.plugin(previous, config)
          const fresh = registered.ctx.fiber
          if (entry !== undefined) {
            fresh.entry = entry
            entry.fiber = fresh
          }
          restored.push(fresh)
        }
        await Promise.all(restored.map(fiber => fiber.await()))
        return { ok: false, error: `the new code failed to start, the previous version is restored (${error?.message ?? error})`, restored: true }
      } catch (restoreError) {
        return { ok: false, error: `the new code failed and restoring the old one failed too: ${restoreError?.message ?? restoreError}`, restored: false }
      }
    }
  })()
  return inFlight.finally(() => { inFlight = null })
}

/**
 * Watch this package and reload on any source change.
 *
 * Opt-in, and off by default: a normal install does not rewrite its own files.
 * It exists because the alternative — restarting the whole app to pick up an
 * edit — is the single most expensive step in working on this plugin.
 *
 * @returns a stop function.
 */
export function watchPackage(packageDir, { onChange, logger = console, debounceMs = 600 } = {}) {
  let timer
  let paused = false
  let watcher
  try {
    watcher = fs.watch(packageDir, { recursive: true })
  } catch (error) {
    logger.warn?.(`dzdrone-ssh: could not watch ${packageDir} (${error?.message ?? error})`)
    return () => {}
  }
  const schedule = () => {
    if (paused) return
    clearTimeout(timer)
    timer = setTimeout(() => {
      timer = undefined
      if (!paused) onChange?.()
    }, debounceMs)
    timer.unref?.()
  }
  watcher.on('change', schedule)
  watcher.on('error', error => logger.warn?.(`dzdrone-ssh: watcher error (${error?.message ?? error})`))
  return () => {
    paused = true
    clearTimeout(timer)
    try { watcher.close() } catch { /* already closed */ }
  }
}
