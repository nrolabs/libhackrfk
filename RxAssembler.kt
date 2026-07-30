/*
 * libhackrfk - Kotlin driver for the HackRF SDR (Android USB host)
 *
 * Receive-stream assembler: raw bulk-IN bytes -> fixed-size float IQ blocks,
 * preserving every byte and the I/Q phase alignment across transfers.
 *
 * Kotlin port: Copyright (C) 2026 Isak Ruas <isakruas@gmail.com>.
 * All rights reserved.
 *
 * Dual-licensed: GPLv2+ only as distributed within the iSDR Drivers
 * application; all other uses require a separate license from the copyright
 * holder. See LICENSE at the root of this module.
 */
package com.isaklab.libhackrfk

import java.util.concurrent.TimeUnit
import java.util.concurrent.locks.ReentrantLock
import kotlin.concurrent.withLock

/**
 * Byte ring between the USB thread and the DSP thread of the receive path.
 *
 * The USB thread [feed]s whatever each bulk-IN transfer delivered — full
 * transfers, short transfers, even odd byte counts — and the DSP thread
 * [awaitBlock]s whole blocks of [blockPairs] IQ pairs as floats. Two
 * properties are the whole point:
 *
 * - **Nothing is discarded.** A short transfer is stream data like any
 *   other; dropping it does not merely lose samples, it tears the phase of
 *   everything after it. Bytes accumulate until a block is complete.
 * - **I/Q alignment is positional, not per-transfer.** The stream is
 *   I,Q,I,Q… from the moment the mode starts; an odd-length transfer simply
 *   leaves one byte in the ring, and the next transfer's first byte is that
 *   pair's Q. Blocks are always an even number of bytes, so the parity of
 *   the stream is preserved by construction.
 *
 * When the DSP side stalls long enough to fill the ring, the OLDEST whole
 * blocks are dropped (block-granular, so parity survives) and counted in
 * [dropEvents] — the host-side twin of the firmware's shortfall counter, and
 * reported through the same telemetry.
 *
 * Android-free and JVM-testable; allocation-free after construction.
 */
class RxAssembler(
    private val blockPairs: Int = HackRfProtocol.RX_BLOCK_PAIRS,
    capacityBytes: Int = HackRfProtocol.RX_RING_BYTES,
) {
    private val blockBytes = blockPairs * 2
    private val buf = ByteArray(capacityBytes)
    private val lock = ReentrantLock()
    private val blockReady = lock.newCondition()
    private var head = 0 // next read index
    private var size = 0

    /** Times the ring overflowed and dropped old data (each is a real gap). */
    var dropEvents = 0L
        private set

    /** Total bytes dropped by overflow, for diagnostics. */
    var droppedBytes = 0L
        private set

    /** Bytes currently queued (test/diagnostic visibility). */
    val pending: Int get() = lock.withLock { size }

    /**
     * Append [length] bytes of [src]. Called from the USB thread; the lock is
     * held for a memcpy only. Overflow drops the oldest whole blocks so the
     * newest data — the samples closest to real time — always survives.
     */
    fun feed(src: ByteArray, length: Int) {
        if (length <= 0) return
        lock.lock()
        try {
            var off = 0
            var n = length
            if (n > buf.size) { // pathological; keep only what fits
                off = n - buf.size
                droppedBytes += off
                dropEvents++
                n = buf.size
            }
            val overflow = size + n - buf.size
            if (overflow > 0) {
                // Round the drop UP to whole blocks: dropping a partial block
                // would shift the stream by an odd amount relative to the
                // block grid and, worse, potentially by an odd byte count.
                val drop = ((overflow + blockBytes - 1) / blockBytes * blockBytes)
                    .coerceAtMost(size)
                head = (head + drop) % buf.size
                size -= drop
                droppedBytes += drop
                dropEvents++
            }
            var tail = (head + size) % buf.size
            var s = off
            var rem = n
            while (rem > 0) {
                val chunk = minOf(rem, buf.size - tail)
                System.arraycopy(src, s, buf, tail, chunk)
                tail = (tail + chunk) % buf.size
                s += chunk
                rem -= chunk
            }
            size += n
            if (size >= blockBytes) blockReady.signal()
        } finally {
            lock.unlock()
        }
    }

    // The claimed block is memcpy'd here under the lock; once head moves the
    // region counts as free space and a concurrent feed() may overwrite it,
    // so the s8->float arithmetic works on this private copy instead.
    private val scratch = ByteArray(blockBytes)

    /**
     * Wait up to [timeoutMs] for a whole block, convert it to interleaved
     * floats in [out] (length >= [blockPairs] * 2) and return true. Only a
     * memcpy happens under the lock; the signed-8-bit conversion runs
     * outside it, so the USB thread is never blocked behind the arithmetic.
     */
    fun awaitBlock(out: FloatArray, timeoutMs: Long): Boolean {
        lock.lock()
        try {
            if (size < blockBytes) {
                try {
                    blockReady.await(timeoutMs, TimeUnit.MILLISECONDS)
                } catch (_: InterruptedException) {
                    Thread.currentThread().interrupt()
                    return false
                }
                if (size < blockBytes) return false
            }
            val first = minOf(blockBytes, buf.size - head)
            System.arraycopy(buf, head, scratch, 0, first)
            if (first < blockBytes) {
                System.arraycopy(buf, 0, scratch, first, blockBytes - first)
            }
            head = (head + blockBytes) % buf.size
            size -= blockBytes
        } finally {
            lock.unlock()
        }
        for (i in 0 until blockBytes) {
            out[i] = HackRfProtocol.s8ToFloat(scratch[i])
        }
        return true
    }

    /** Discard buffered data and counters for a fresh stream. */
    fun reset() {
        lock.lock()
        try {
            head = 0
            size = 0
            dropEvents = 0
            droppedBytes = 0
        } finally {
            lock.unlock()
        }
    }

    /** Wake a parked [awaitBlock] (used at shutdown). */
    fun wake() {
        lock.lock()
        try {
            blockReady.signal()
        } finally {
            lock.unlock()
        }
    }
}
