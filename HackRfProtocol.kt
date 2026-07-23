/*
 * libhackrfk - Kotlin driver for the HackRF SDR (Android USB host)
 *
 * Wire codec: vendor request ids, payload encodings, gain/filter rules and
 * sample conversion.
 *
 * Ported faithfully from the reference host library libhackrf:
 *   Copyright (c) 2012-2026 Great Scott Gadgets <info@greatscottgadgets.com>
 *   Copyright (c) 2012, Jared Boone <jared@sharebrained.com>
 *   Copyright (c) 2013, Benjamin Vernoux <titanmkd@gmail.com>
 *   (BSD 3-clause; this notice is retained per its terms)
 *   https://github.com/greatscottgadgets/hackrf
 *
 * Kotlin port: Copyright (C) 2026 Isak Ruas <isakruas@gmail.com>.
 * All rights reserved.
 *
 * Dual-licensed: GPLv2+ only as distributed within the iSDR Drivers
 * application; all other uses require a separate license from the copyright
 * holder. See LICENSE at the root of this module.
 */
package com.isaklab.libhackrfk

/**
 * Android-free wire codec for the HackRF host protocol — a line-for-line port
 * of the encodings in the reference `host/libhackrf/src/hackrf.c` (in-repo at
 * `hackrf/`), kept pure Kotlin so every rule is JVM-testable, the same way
 * `Hl2Protocol` covers the Hermes-Lite 2.
 */
object HackRfProtocol {

    // ---- identity -----------------------------------------------------------

    const val USB_VID = 0x1d50
    val USB_PIDS = intArrayOf(0x6089, 0x604b, 0xcc15) // One, Jawbreaker, rad1o

    /** True when (vendorId, productId) is a known HackRF board. */
    fun isKnownDevice(vendorId: Int, productId: Int): Boolean =
        vendorId == USB_VID && productId in USB_PIDS

    // ---- vendor request ids (hackrf.c enum, subset used) --------------------

    const val REQ_SET_TRANSCEIVER_MODE = 1
    const val REQ_SAMPLE_RATE_SET = 6
    const val REQ_BASEBAND_FILTER_BW_SET = 7
    const val REQ_BOARD_ID_READ = 14
    const val REQ_VERSION_STRING_READ = 15
    const val REQ_SET_FREQ = 16
    const val REQ_AMP_ENABLE = 17
    const val REQ_SET_LNA_GAIN = 19
    const val REQ_SET_VGA_GAIN = 20
    const val REQ_SET_TXVGA_GAIN = 21
    const val REQ_ANTENNA_ENABLE = 23

    /**
     * Reading the buffer size once also clears the firmware's auto-tx-flush
     * flag, making TX→RX mode switches immediate instead of blocking until
     * the whole transmit buffer drains (firmware usb_api_transceiver.c).
     */
    const val REQ_GET_BUFFER_SIZE = 61

    const val REQ_INIT_SWEEP = 26

    const val MODE_OFF = 0
    const val MODE_RECEIVE = 1
    const val MODE_TRANSMIT = 2
    const val MODE_RX_SWEEP = 5

    /** bmRequestType vendor|device, host→device / device→host. */
    const val TYPE_VENDOR_OUT = 0x40
    const val TYPE_VENDOR_IN = 0xC0

    // ---- payload encodings ---------------------------------------------------

    /**
     * hackrf_set_freq payload: uint32 LE whole-MHz part followed by uint32 LE
     * Hz remainder (0..999_999).
     */
    fun freqParams(hz: Long): ByteArray {
        val mhz = (hz / 1_000_000L).toInt()
        val rem = (hz - mhz * 1_000_000L).toInt()
        val data = ByteArray(8)
        putLeInt(data, 0, mhz)
        putLeInt(data, 4, rem)
        return data
    }

    /** hackrf_set_sample_rate_manual payload: uint32 LE Hz + uint32 LE divider. */
    fun sampleRateParams(hz: Int, divider: Int = 1): ByteArray {
        val data = ByteArray(8)
        putLeInt(data, 0, hz)
        putLeInt(data, 4, divider)
        return data
    }

    /** BASEBAND_FILTER_BANDWIDTH_SET splits the Hz across wValue/wIndex. */
    fun filterBwValue(bandwidthHz: Int): Int = bandwidthHz and 0xFFFF
    fun filterBwIndex(bandwidthHz: Int): Int = bandwidthHz ushr 16

    // ---- rules ----------------------------------------------------------------

    /** MAX2837 anti-alias filter table (hackrf.c max2837_ft), ascending Hz. */
    val BASEBAND_FILTERS_HZ = intArrayOf(
        1_750_000, 2_500_000, 3_500_000, 5_000_000, 5_500_000, 6_000_000,
        7_000_000, 8_000_000, 9_000_000, 10_000_000, 12_000_000, 14_000_000,
        15_000_000, 20_000_000, 24_000_000, 28_000_000,
    )

    /**
     * hackrf_compute_baseband_filter_bw: the first table entry ≥ the request,
     * rounded DOWN to the previous entry when strictly greater (unless the
     * request is below the smallest entry). Requests beyond the table clamp to
     * the widest filter (the reference walks onto its 0 sentinel there, which
     * would program a nonsensical 0 Hz — sample-rate setting never reaches it,
     * since it always asks for 75 % of ≤ 20 MHz).
     */
    fun basebandFilterFor(bandwidthHz: Int): Int {
        val idx = BASEBAND_FILTERS_HZ.indexOfFirst { it >= bandwidthHz }
        if (idx < 0) return BASEBAND_FILTERS_HZ.last()
        if (idx == 0) return BASEBAND_FILTERS_HZ[0]
        return if (BASEBAND_FILTERS_HZ[idx] > bandwidthHz) {
            BASEBAND_FILTERS_HZ[idx - 1]
        } else {
            BASEBAND_FILTERS_HZ[idx]
        }
    }

    /** Anti-alias bandwidth auto-picked on rate changes: 75 % of the rate. */
    fun basebandFilterForSampleRate(sampleRateHz: Int): Int =
        basebandFilterFor((0.75 * sampleRateHz).toInt())

    /** LNA (IF) gain: 0–40 dB, 8 dB steps (hackrf.c masks with ~0x07). */
    fun lnaGainMasked(db: Int): Int = db.coerceIn(0, 40) and 0x07.inv()

    /** VGA (baseband) gain: 0–62 dB, 2 dB steps (hackrf.c masks with ~0x01). */
    fun vgaGainMasked(db: Int): Int = db.coerceIn(0, 62) and 0x01.inv()

    /** TXVGA gain: 0–47 dB, 1 dB steps. */
    fun txVgaGain(db: Int): Int = db.coerceIn(0, 47)

    /** The app's 0–255 drive scale mapped onto the 0–47 dB TXVGA range. */
    fun driveToTxVga(level: Int): Int = level.coerceIn(0, 255) * 47 / 255

    // ---- sweep mode (usb_api_sweep.c) -------------------------------------------

    /** Sweep data arrives in fixed blocks: 10-byte header + s8 IQ samples. */
    const val SWEEP_BLOCK_SIZE = 16_384
    const val SWEEP_HEADER_SIZE = 10
    const val SWEEP_MAGIC_0 = 0x7F.toByte()
    const val SWEEP_MAGIC_1 = 0x7F.toByte()

    const val SWEEP_STYLE_LINEAR = 0
    const val SWEEP_STYLE_INTERLEAVED = 1

    /**
     * INIT_SWEEP payload: step_width u32 LE, offset u32 LE, style u8, then up
     * to 10 [start, stop] range pairs as u16 LE whole MHz. The dwell size in
     * bytes per tuning (a multiple of [SWEEP_BLOCK_SIZE]) rides in
     * wValue/wIndex — see [sweepDwellValue]/[sweepDwellIndex].
     */
    fun initSweepParams(
        rangesMHz: List<Pair<Int, Int>>,
        stepWidthHz: Int,
        offsetHz: Int,
        style: Int,
    ): ByteArray {
        require(rangesMHz.isNotEmpty() && rangesMHz.size <= 10) { "1..10 ranges" }
        val data = ByteArray(9 + rangesMHz.size * 4)
        putLeInt(data, 0, stepWidthHz)
        putLeInt(data, 4, offsetHz)
        data[8] = style.toByte()
        rangesMHz.forEachIndexed { i, (start, stop) ->
            val base = 9 + i * 4
            data[base] = (start and 0xFF).toByte()
            data[base + 1] = ((start ushr 8) and 0xFF).toByte()
            data[base + 2] = (stop and 0xFF).toByte()
            data[base + 3] = ((stop ushr 8) and 0xFF).toByte()
        }
        return data
    }

    fun sweepDwellValue(bytesPerTune: Int): Int = bytesPerTune and 0xFFFF
    fun sweepDwellIndex(bytesPerTune: Int): Int = bytesPerTune ushr 16

    /** True when a sweep block starts at [offset] (0x7F 0x7F marker). */
    fun isSweepBlock(buf: ByteArray, offset: Int): Boolean =
        buf.size >= offset + SWEEP_HEADER_SIZE &&
            buf[offset] == SWEEP_MAGIC_0 && buf[offset + 1] == SWEEP_MAGIC_1

    /** Tuned frequency of the sweep block at [offset]: u64 little-endian Hz. */
    fun sweepBlockFreqHz(buf: ByteArray, offset: Int): Long {
        var v = 0L
        for (k in 7 downTo 0) {
            v = (v shl 8) or (buf[offset + 2 + k].toLong() and 0xFF)
        }
        return v
    }

    // ---- sample conversion -----------------------------------------------------

    /** Float [-1,1] → the board's signed 8-bit sample. */
    fun toS8(v: Float): Byte = (v.coerceIn(-1f, 1f) * 127f).toInt().toByte()

    /** The board's signed 8-bit sample → float in (-1,1]. */
    fun s8ToFloat(b: Byte): Float = b / 128.0f

    /** TX board rate and the exact-integer upsampling from the 48 k input. */
    const val TX_INPUT_RATE = 48_000
    const val TX_BOARD_RATE = 2_400_000
    const val TX_UPSAMPLE = TX_BOARD_RATE / TX_INPUT_RATE

    // ---- helpers ---------------------------------------------------------------

    private fun putLeInt(dst: ByteArray, offset: Int, value: Int) {
        dst[offset] = (value and 0xFF).toByte()
        dst[offset + 1] = ((value ushr 8) and 0xFF).toByte()
        dst[offset + 2] = ((value ushr 16) and 0xFF).toByte()
        dst[offset + 3] = ((value ushr 24) and 0xFF).toByte()
    }
}
