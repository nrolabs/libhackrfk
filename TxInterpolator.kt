/*
 * libhackrfk - Kotlin driver for the HackRF SDR (Android USB host)
 *
 * Polyphase FIR interpolator for the transmit path (48 kS/s modulator IQ
 * up to the board sample rate).
 *
 * Kotlin port: Copyright (C) 2026 Isak Ruas <isakruas@gmail.com>.
 * All rights reserved.
 *
 * Dual-licensed: GPLv2+ only as distributed within the iSDR Drivers
 * application; all other uses require a separate license from the copyright
 * holder. See LICENSE at the root of this module.
 */
package com.isaklab.libhackrfk

import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

/**
 * Polyphase FIR interpolator for the HackRF transmit path: expands the 48 kS/s
 * modulator IQ to the [HackRfProtocol.TX_BOARD_RATE] board rate. A plain
 * sample-and-interpolate stage would transmit spectral images at every 48 kHz
 * multiple around the carrier at roughly −30 dBc; this Blackman-windowed sinc
 * (8 taps per phase, 400-tap prototype) pushes them below −60 dBc.
 *
 * Each polyphase branch is normalized to unit DC gain, so a constant input —
 * a keyed CW carrier — reproduces EXACTLY at the output, with zero ripple.
 * Pure Kotlin and stateful-but-allocation-free on the hot path; the spectral
 * contract is pinned by TxInterpolatorTest.
 */
class TxInterpolator(
    private val factor: Int = HackRfProtocol.TX_UPSAMPLE,
    private val tapsPerPhase: Int = TAPS_PER_PHASE,
) {
    companion object {
        /**
         * Prototype taps per polyphase branch — also the depth of the input
         * history, i.e. how many input samples of tail the output carries
         * after the last non-zero sample.
         */
        const val TAPS_PER_PHASE = 8
    }

    /** [factor] branches × [tapsPerPhase] taps of the prototype lowpass. */
    private val phases: Array<FloatArray>

    // Newest-first input history, one per channel.
    private val histI = FloatArray(tapsPerPhase)
    private val histQ = FloatArray(tapsPerPhase)

    init {
        val n = factor * tapsPerPhase
        val center = (n - 1) / 2.0
        val proto = DoubleArray(n)
        for (i in 0 until n) {
            val x = (i - center) / factor
            val sinc = if (x == 0.0) 1.0 else sin(PI * x) / (PI * x)
            // Blackman window: ~-74 dB sidelobes, transition width is
            // irrelevant here (speech stops at 3 kHz vs the 24 kHz cutoff).
            val w = 0.42 -
                0.5 * cos(2.0 * PI * i / (n - 1)) +
                0.08 * cos(4.0 * PI * i / (n - 1))
            proto[i] = sinc * w
        }
        phases = Array(factor) { phase ->
            var sum = 0.0
            for (k in 0 until tapsPerPhase) sum += proto[k * factor + phase]
            FloatArray(tapsPerPhase) { k -> (proto[k * factor + phase] / sum).toFloat() }
        }
    }

    /**
     * Resets the internal polyphase history buffers. This should be called
     * prior to beginning a new transmission stream to ensure no residual
     * samples from previous keying events pollute the output.
     */
    fun reset() {
        histI.fill(0f)
        histQ.fill(0f)
    }

    /**
     * Feeds one 48 kS/s complex IQ pair into the interpolator and writes [factor]
     * upsampled, board-rate pairs as interleaved floats into the provided [out] array.
     * The polyphase branch convolutions are completely allocation-free.
     *
     * @param i Real (in-phase) part of the input sample.
     * @param q Imaginary (quadrature) part of the input sample.
     * @param out The destination array for the upsampled samples.
     * @param offset The starting index in the destination array.
     * @return The updated index offset within the destination array.
     */
    fun process(i: Float, q: Float, out: FloatArray, offset: Int): Int {
        for (k in tapsPerPhase - 1 downTo 1) {
            histI[k] = histI[k - 1]
            histQ[k] = histQ[k - 1]
        }
        histI[0] = i
        histQ[0] = q
        var n = offset
        for (p in 0 until factor) {
            val taps = phases[p]
            var accI = 0f
            var accQ = 0f
            for (k in 0 until tapsPerPhase) {
                accI += histI[k] * taps[k]
                accQ += histQ[k] * taps[k]
            }
            out[n++] = accI
            out[n++] = accQ
        }
        return n
    }
}
