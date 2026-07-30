/*
 * libhackrfk - Kotlin driver for the HackRF SDR (Android USB host)
 *
 * Transmit block renderer: 48 kSps modulator IQ -> board-rate signed 8-bit
 * blocks, with the queue-cushion control that keeps the DAC fed.
 *
 * Kotlin port: Copyright (C) 2026 Isak Ruas <isakruas@gmail.com>.
 * All rights reserved.
 *
 * Dual-licensed: GPLv2+ only as distributed within the iSDR Drivers
 * application; all other uses require a separate license from the copyright
 * holder. See LICENSE at the root of this module.
 */
package com.isaklab.libhackrfk

import com.isaklab.isdrdrivers.core.FloatRing

/**
 * Renders one USB block of transmit samples per call and, in the same pass,
 * regulates the depth of the 48 kSps input queue.
 *
 * The regulation is the point. A transmit shortfall on this board is not a
 * dropout: the M0 SGPIO loop writes **zeroes** to the DAC whenever the M4
 * hasn't advanced its byte count in time (firmware `hackrf_usb/sgpio_m0.s`),
 * so a host that lets the queue sit at zero hard-gates the envelope at an
 * audio rate and sprays splatter across the band. Two mechanisms prevent it:
 *
 * - **Cushion.** The board clocks its DAC off its own TCXO while the host
 *   produces off the Android audio clock. Those two never agree exactly, so a
 *   queue that starts empty drifts straight into starvation. [drain] holds the
 *   queue at [targetPairs] by swallowing one pair when it runs long and taking
 *   one fewer (repeating the last sample) when it runs short — asynchronous
 *   sample-rate correction, one pair at a time, far below audibility and good
 *   for ±3900 ppm at one correction per block.
 * - **Fade.** If the queue does run dry anyway, [render] ramps the last sample
 *   to zero over [fadePairs] instead of stepping to it. The step is what
 *   splatters; a 1 ms edge does not.
 *
 * Deliberately Android-free so every rule above is JVM-testable, the same way
 * [HackRfProtocol] covers the wire codec. Allocation-free after construction.
 */
class TxBlockRenderer(
    private val blockPairs: Int = HackRfProtocol.TX_BLOCK_PAIRS,
    private val targetPairs: Int = HackRfProtocol.TX_TARGET_PAIRS,
    private val slackPairs: Int = HackRfProtocol.TX_SLACK_PAIRS,
    private val fadePairs: Int = HackRfProtocol.TX_FADE_PAIRS,
) {
    private val interp = TxInterpolator()
    private val upsampled = FloatArray(HackRfProtocol.TX_UPSAMPLE * 2)
    private val inBuf = FloatArray(blockPairs * 2)

    // Carried across blocks: a starve that spans a block boundary must keep
    // fading, not restart the ramp at full amplitude.
    private var lastI = 0f
    private var lastQ = 0f
    private var starve = 0

    // Unkey ramp state: -1 while keyed, counts down from [unkeyRampPairs]
    // to 0 once beginUnkey() is called. Volatile because unkey is requested
    // from the control thread while render() runs on the render thread.
    @Volatile private var unkeyRemaining = -1
    private var unkeyRampPairs = HackRfProtocol.TX_UNKEY_FADE_PAIRS

    /** Pairs the board was fed from a dry queue (fade or silence). */
    var starvedPairs = 0L
        private set

    /** Pairs dropped to walk an over-long queue back to [targetPairs]. */
    var trimmedPairs = 0L
        private set

    /** Pairs repeated to walk an over-short queue back to [targetPairs]. */
    var heldPairs = 0L
        private set

    /** Bytes one rendered block occupies. */
    val blockBytes: Int = blockPairs * HackRfProtocol.TX_UPSAMPLE * 2

    /** Clear filter history, fade state and counters before a new transmission. */
    fun reset() {
        interp.reset()
        lastI = 0f
        lastQ = 0f
        starve = 0
        starvedPairs = 0
        trimmedPairs = 0
        heldPairs = 0
        unkeyRemaining = -1
    }

    /**
     * Begin the unkey ramp: from the next rendered pair on, output is
     * multiplied by a linear fade to zero over [rampPairs] pairs, draining
     * whatever the queue still holds under it. The step this replaces is the
     * classic key click — an envelope discontinuity radiated as a wideband
     * transient at every release of PTT. Idempotent while a ramp is running.
     */
    fun beginUnkey(rampPairs: Int = HackRfProtocol.TX_UNKEY_FADE_PAIRS) {
        if (unkeyRemaining >= 0) return
        unkeyRampPairs = rampPairs.coerceAtLeast(1)
        unkeyRemaining = unkeyRampPairs
    }

    /** True once the unkey ramp has fully reached zero (only zeros follow). */
    val unkeyComplete: Boolean get() = unkeyRemaining == 0

    /**
     * Take up to one block of pairs out of [queue], applying the cushion
     * correction. The caller holds whatever lock guards the ring; this does no
     * arithmetic beyond the copy, so the lock is held for a memcpy and not for
     * the interpolation.
     *
     * @return how many pairs were actually taken (0..[blockPairs]).
     */
    fun drain(queue: FloatRing): Int {
        val availPairs = queue.size / 2
        var want = blockPairs
        if (availPairs > targetPairs + slackPairs && queue.size >= 2) {
            queue.read()
            queue.read()
            trimmedPairs++
        } else if (availPairs in 1 until (targetPairs - slackPairs)) {
            want -= 1
            heldPairs++
        }
        var got = 0
        while (got < want && queue.size >= 2) {
            inBuf[got * 2] = queue.read()
            inBuf[got * 2 + 1] = queue.read()
            got++
        }
        return got
    }

    /**
     * Interpolate and quantize the [got] pairs taken by [drain] into [out],
     * filling any shortfall with the fade described in the class docs. Writes
     * exactly [blockBytes] bytes and touches no shared state, so it runs
     * outside the queue lock.
     */
    fun render(got: Int, out: ByteArray) {
        var n = 0
        for (p in 0 until blockPairs) {
            var i: Float
            var q: Float
            if (p < got) {
                i = inBuf[p * 2]
                q = inBuf[p * 2 + 1]
                lastI = i
                lastQ = q
                starve = 0
            } else {
                starve++
                starvedPairs++
                val g = if (starve >= fadePairs) 0f else 1f - starve.toFloat() / fadePairs
                i = lastI * g
                q = lastQ * g
            }
            // Unkey ramp on top of whatever the pair already is (live audio,
            // starvation fade or silence): a single multiply keeps the two
            // fades composable and the envelope monotonic to zero.
            val remaining = unkeyRemaining
            if (remaining >= 0) {
                val g = if (remaining == 0) 0f else remaining.toFloat() / unkeyRampPairs
                i *= g
                q *= g
                if (remaining > 0) unkeyRemaining = remaining - 1
            }
            interp.process(i, q, upsampled, 0)
            for (v in upsampled) out[n++] = HackRfProtocol.toS8(v)
        }
    }
}
