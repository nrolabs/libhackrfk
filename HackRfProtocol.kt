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
    // Operational extensions (reference hackrf.c; service/factory ops like
    // SPIFLASH/CPLD/FPGA are deliberately out of an app's scope):
    const val REQ_BOARD_PARTID_SERIALNO_READ = 18
    /** SGPIO loop state on the M0 core (hackrf.c:1447, USB API 0x0106+). */
    const val REQ_GET_M0_STATE = 41
    const val REQ_OPERACAKE_GET_BOARDS = 27
    const val REQ_OPERACAKE_SET_PORTS = 28
    const val REQ_RESET = 30
    const val REQ_CLKOUT_ENABLE = 32
    const val REQ_OPERACAKE_SET_MODE = 38
    const val REQ_SET_TX_UNDERRUN_LIMIT = 42
    const val REQ_SET_RX_OVERRUN_LIMIT = 43
    const val REQ_GET_CLKIN_STATUS = 44
    const val REQ_BOARD_REV_READ = 45

    /**
     * Bias-tee per-mode options word (hackrf.c hackrf_set_user_bias_t_opts):
     * three 3-bit groups — off[2:0]=update|changeOnEntry|enabled at bits 2..0,
     * rx at 5..3, tx at 8..6.
     */
    fun biasTOptsWord(
        offUpdate: Boolean, offEnabled: Boolean,
        rxUpdate: Boolean, rxEnabled: Boolean,
        txUpdate: Boolean, txEnabled: Boolean,
    ): Int {
        var state = 0
        if (offUpdate) state = state or 0x4 or 0x2 or (if (offEnabled) 1 else 0)
        if (rxUpdate) state = state or 0x20 or 0x10 or (if (rxEnabled) 1 shl 3 else 0)
        if (txUpdate) state = state or 0x100 or 0x80 or (if (txEnabled) 1 shl 6 else 0)
        return state
    }
    const val REQ_SET_USER_BIAS_T_OPTS = 48

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
     * Computes the appropriate baseband filter bandwidth for a given requested bandwidth.
     * Evaluates the first table entry ≥ the request, rounding down to the previous entry 
     * if strictly greater. Clamps to the widest available filter if out of bounds.
     * This avoids programming a nonsensical 0 Hz baseband filter.
     *
     * @param bandwidthHz The requested anti-alias bandwidth in Hz.
     * @return The exact filter bandwidth in Hz to configure on the MAX2837.
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

    /** 
     * Calculates VGA (baseband) gain encoding. Range: 0–62 dB, 2 dB steps.
     * Enforces the ~0x01 mask expected by the hardware.
     */
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

    /** 
     * Converts a normalized Float [-1,1] continuous sample to the board's signed 8-bit discrete format. 
     * Clamps values out of bounds to prevent integer overflow wraparound.
     */
    fun toS8(v: Float): Byte = (v.coerceIn(-1f, 1f) * 127f).toInt().toByte()

    /** 
     * Converts the board's signed 8-bit discrete format to a normalized float in (-1,1].
     * Divides by 128.0f to maintain consistent peak amplitudes.
     */
    fun s8ToFloat(b: Byte): Float = b / 128.0f

    /** TX board rate and the exact-integer upsampling from the 48 k input. */
    const val TX_INPUT_RATE = 48_000
    const val TX_BOARD_RATE = 2_400_000
    const val TX_UPSAMPLE = TX_BOARD_RATE / TX_INPUT_RATE

    // ---- transmit pipeline sizing ------------------------------------------------
    //
    // Everything below is derived from the firmware's own numbers, because a
    // transmit shortfall is not a soft failure on this board: the M0 SGPIO
    // loop writes ZEROES to the DAC whenever the M4 hasn't advanced its count
    // in time (firmware `hackrf_usb/sgpio_m0.s`, "In TX mode, zeroes are
    // written to SGPIO"). A starved host therefore hard-gates the envelope at
    // an audio rate, which on the air is broadband splatter — not a dropout.

    /** Bulk-out block the firmware schedules at a time (usb_api_transceiver.c). */
    const val USB_TRANSFER_SIZE = 0x4000

    /** Sample buffer the firmware fills BEFORE enabling baseband streaming. */
    const val USB_SAMP_BUFFER_SIZE = 0x8000

    /** Bulk buffer kept in flight behind it. */
    const val USB_BULK_BUFFER_SIZE = 0x8000

    /**
     * Total bytes the board can hold while transmitting — 64 KiB, which at
     * [TX_BOARD_RATE] is only **13.65 ms**. That is the entire budget the host
     * has to miss its deadline by, and it is why the transmit path is split
     * into a render thread and a USB thread instead of doing both in series.
     */
    const val DEVICE_TX_BUFFER_BYTES = USB_SAMP_BUFFER_SIZE + USB_BULK_BUFFER_SIZE

    /**
     * Input pairs rendered per USB block. 256 × [TX_UPSAMPLE] × 2 = 25 600
     * bytes — a whole number of 512-byte USB packets (so the stream never
     * carries a short packet) and 5.33 ms of RF.
     */
    const val TX_BLOCK_PAIRS = 256
    const val TX_BLOCK_BYTES = TX_BLOCK_PAIRS * TX_UPSAMPLE * 2

    /**
     * Rendered blocks kept in the pool. The reference host library keeps
     * `TRANSFER_COUNT` = 4 transfers permanently queued on the endpoint
     * (hackrf.c:144); this is the same idea with the same depth — the USB
     * thread always has the next block ready the instant the previous one
     * completes, so no per-block compute ever lands between transfers.
     */
    const val TX_BLOCKS_IN_FLIGHT = 4

    /**
     * Cushion of un-rendered 48 kSps pairs the driver aims to keep queued —
     * 30 ms. The board consumes off its own TCXO while the host produces off
     * the Android audio clock; without a cushion the queue sits at zero and
     * roughly every other block is part silence.
     */
    const val TX_TARGET_PAIRS = 1_440

    /**
     * How far the cushion may stray (±10 ms) before the renderer drops or
     * repeats a single pair to pull it back. One pair at 48 kSps is inaudible,
     * and at most one correction per block it can track ±3900 ppm — orders of
     * magnitude more than the two clocks will ever differ.
     */
    const val TX_SLACK_PAIRS = 480

    /**
     * When the queue genuinely runs dry, the last sample is faded out over
     * this many pairs (1 ms) instead of stepping to zero. A step IS the
     * splatter; a 1 ms raised edge is not.
     */
    const val TX_FADE_PAIRS = 48

    // ---- M0 SGPIO loop state (hackrf.c hackrf_get_m0_state) ----------------------

    /** Byte length of the `hackrf_m0_state` struct on the wire. */
    const val M0_STATE_SIZE = 40

    /**
     * Decoded `hackrf_m0_state` (hackrf.h) — the board's own account of how
     * well it was fed. [numShortfalls] staying put across a transmission is
     * the proof that the host never starved the DAC.
     */
    data class M0State(
        val requestedMode: Int,
        val requestFlag: Int,
        val activeMode: Int,
        val m0Count: Int,
        val m4Count: Int,
        val numShortfalls: Int,
        val longestShortfall: Int,
        val shortfallLimit: Int,
        val threshold: Int,
        val nextMode: Int,
        val error: Int,
    )

    /** Parse the 40-byte little-endian reply of [REQ_GET_M0_STATE]. */
    fun parseM0State(buf: ByteArray): M0State? {
        if (buf.size < M0_STATE_SIZE) return null
        return M0State(
            requestedMode = leShort(buf, 0),
            requestFlag = leShort(buf, 2),
            activeMode = leInt(buf, 4),
            m0Count = leInt(buf, 8),
            m4Count = leInt(buf, 12),
            numShortfalls = leInt(buf, 16),
            longestShortfall = leInt(buf, 20),
            shortfallLimit = leInt(buf, 24),
            threshold = leInt(buf, 28),
            nextMode = leInt(buf, 32),
            error = leInt(buf, 36),
        )
    }

    // ---- helpers ---------------------------------------------------------------

    private fun leShort(src: ByteArray, offset: Int): Int =
        (src[offset].toInt() and 0xFF) or ((src[offset + 1].toInt() and 0xFF) shl 8)

    private fun leInt(src: ByteArray, offset: Int): Int =
        (src[offset].toInt() and 0xFF) or
            ((src[offset + 1].toInt() and 0xFF) shl 8) or
            ((src[offset + 2].toInt() and 0xFF) shl 16) or
            ((src[offset + 3].toInt() and 0xFF) shl 24)

    private fun putLeInt(dst: ByteArray, offset: Int, value: Int) {
        dst[offset] = (value and 0xFF).toByte()
        dst[offset + 1] = ((value ushr 8) and 0xFF).toByte()
        dst[offset + 2] = ((value ushr 16) and 0xFF).toByte()
        dst[offset + 3] = ((value ushr 24) and 0xFF).toByte()
    }
}
