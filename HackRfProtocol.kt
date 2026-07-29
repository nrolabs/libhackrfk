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
 * Android-free wire codec for the HackRF control protocol: vendor request
 * ids, payload layouts, gain and filter rules, and sample conversion.
 *
 * Kept free of Android types so every encoding rule is verifiable in a plain
 * JVM test rather than only on hardware, the same way `Hl2Protocol` covers
 * the Hermes-Lite 2.
 */
object HackRfProtocol {

    // ---- identity -----------------------------------------------------------

    const val USB_VID = 0x1d50
    val USB_PIDS = intArrayOf(0x6089, 0x604b, 0xcc15) // One, Jawbreaker, rad1o

    /** True when (vendorId, productId) is a known HackRF board. */
    fun isKnownDevice(vendorId: Int, productId: Int): Boolean =
        vendorId == USB_VID && productId in USB_PIDS

    // ---- vendor request ids -------------------------------------------------
    //
    // The board's entire control vocabulary, not a subset: a request this
    // codec does not name is a control the driver cannot reach, so any gap
    // here is a missing feature on the radio.

    const val REQ_SET_TRANSCEIVER_MODE = 1
    const val REQ_MAX283X_WRITE = 2
    const val REQ_MAX283X_READ = 3
    const val REQ_SI5351C_WRITE = 4
    const val REQ_SI5351C_READ = 5
    const val REQ_SAMPLE_RATE_SET = 6
    const val REQ_BASEBAND_FILTER_BW_SET = 7
    const val REQ_RFFC5071_WRITE = 8
    const val REQ_RFFC5071_READ = 9
    const val REQ_SPIFLASH_ERASE = 10
    const val REQ_SPIFLASH_WRITE = 11
    const val REQ_SPIFLASH_READ = 12
    const val REQ_BOARD_ID_READ = 14
    const val REQ_VERSION_STRING_READ = 15
    const val REQ_SET_FREQ = 16
    const val REQ_AMP_ENABLE = 17
    const val REQ_SET_LNA_GAIN = 19
    const val REQ_SET_VGA_GAIN = 20
    const val REQ_SET_TXVGA_GAIN = 21
    const val REQ_ANTENNA_ENABLE = 23
    const val REQ_SET_FREQ_EXPLICIT = 24
    const val REQ_SET_HW_SYNC_MODE = 29
    const val REQ_OPERACAKE_SET_RANGES = 31
    const val REQ_SPIFLASH_STATUS = 33
    const val REQ_SPIFLASH_CLEAR_STATUS = 34
    const val REQ_OPERACAKE_GPIO_TEST = 35
    const val REQ_CPLD_CHECKSUM = 36
    const val REQ_UI_ENABLE = 37
    const val REQ_OPERACAKE_GET_MODE = 39
    const val REQ_OPERACAKE_SET_DWELL_TIMES = 40
    const val REQ_SUPPORTED_PLATFORM_READ = 46
    const val REQ_SET_LEDS = 47
    const val REQ_FPGA_WRITE_REG = 49
    const val REQ_FPGA_READ_REG = 50
    const val REQ_P2_CTRL = 51
    const val REQ_P1_CTRL = 52
    const val REQ_SET_NARROWBAND_FILTER = 53
    const val REQ_SET_FPGA_BITSTREAM = 54
    const val REQ_CLKIN_CTRL = 55
    const val REQ_READ_SELFTEST = 56
    const val REQ_READ_ADC = 57
    const val REQ_TEST_RTC_OSC = 58
    const val REQ_RADIO_WRITE_REG = 59
    const val REQ_RADIO_READ_REG = 60

    /**
     * Reading the buffer size once also clears the firmware's auto-tx-flush
     * flag, making TX→RX mode switches immediate instead of blocking until
     * the whole transmit buffer drains.
     */
    const val REQ_GET_BUFFER_SIZE = 61

    const val REQ_INIT_SWEEP = 26
    const val REQ_BOARD_PARTID_SERIALNO_READ = 18
    /** State of the sample-transfer loop on the M0 core. */
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
     * One mode's bias-tee policy.
     *
     * The two flags are NOT the same switch. [doUpdate] says "write this
     * mode's setting at all"; [changeOnModeEntry] says "and when the radio
     * enters this mode, force the bias tee to [enabled]". Updating WITHOUT
     * changing on entry is how a mode is handed back to the firmware default;
     * collapsing the pair into one flag makes that state unreachable and turns
     * every bias-tee write into a permanent override.
     */
    data class BiasTSetting(
        val doUpdate: Boolean = false,
        val changeOnModeEntry: Boolean = false,
        val enabled: Boolean = false,
    )

    /**
     * Bias-tee per-mode options word: three groups of three bits — OFF at
     * 2..0, RX at 5..3, TX at 8..6, each as update | changeOnEntry | enabled.
     */
    fun biasTOptsWord(off: BiasTSetting, rx: BiasTSetting, tx: BiasTSetting): Int {
        var state = 0
        if (off.doUpdate) {
            state = state or 0x4
            if (off.changeOnModeEntry) state = state or 0x2 or (if (off.enabled) 1 else 0)
        }
        if (rx.doUpdate) {
            state = state or 0x20
            if (rx.changeOnModeEntry) state = state or 0x10 or (if (rx.enabled) 1 shl 3 else 0)
        }
        if (tx.doUpdate) {
            state = state or 0x100
            if (tx.changeOnModeEntry) state = state or 0x80 or (if (tx.enabled) 1 shl 6 else 0)
        }
        return state
    }
    const val REQ_SET_USER_BIAS_T_OPTS = 48

    const val MODE_OFF = 0
    const val MODE_RECEIVE = 1
    const val MODE_TRANSMIT = 2
    const val MODE_SS = 3
    const val MODE_CPLD_UPDATE = 4
    const val MODE_RX_SWEEP = 5

    // ---- firmware capability gating -----------------------------------------
    //
    // The board reports its control-protocol version in the USB descriptor's
    // bcdDevice field. A request the running firmware predates must not be
    // issued: an unknown request id is answered with a STALL, and a stalled
    // control endpoint fails the next legitimate request too — so one probe
    // on older firmware breaks an unrelated tuning command.

    const val API_OPERACAKE_PORTS = 0x0102
    const val API_HW_SYNC = 0x0102
    const val API_OPERACAKE_RANGES = 0x0103
    const val API_SPIFLASH_STATUS = 0x0103
    const val API_CPLD_CHECKSUM = 0x0103
    const val API_UI_ENABLE = 0x0104
    const val API_OPERACAKE_MODE = 0x0105
    const val API_OPERACAKE_DWELL = 0x0105
    const val API_M0_STATE = 0x0106
    const val API_OVERRUN_LIMITS = 0x0106
    const val API_CLKIN_STATUS = 0x0106
    const val API_BOARD_REV = 0x0106
    const val API_SUPPORTED_PLATFORM = 0x0106
    const val API_SET_LEDS = 0x0107
    const val API_BIAS_T_OPTS = 0x0108
    const val API_PRO_SIGNALS = 0x0109
    const val API_RADIO_REGS = 0x0111
    const val API_BUFFER_SIZE = 0x0112

    /** True when firmware reporting [usbApiVersion] implements [required]. */
    fun apiAtLeast(usbApiVersion: Int, required: Int): Boolean =
        usbApiVersion >= required

    /**
     * Human form of a bcdDevice word: 0x0112 -> "1.12".
     *
     * The nibbles are the digits — it is a BCD field, so 0x12 reads as twelve,
     * not eighteen.
     */
    fun usbApiVersionName(v: Int): String =
        "${((v shr 8) and 0xFF).toString(16)}.${(v and 0xFF).toString(16).padStart(2, '0')}"

    // ---- board identity -----------------------------------------------------

    const val BOARD_ID_JELLYBEAN = 0
    const val BOARD_ID_JAWBREAKER = 1
    const val BOARD_ID_HACKRF1_OG = 2
    const val BOARD_ID_RAD1O = 3
    const val BOARD_ID_HACKRF1_R9 = 4
    const val BOARD_ID_PRALINE = 5
    const val BOARD_ID_UNRECOGNIZED = 0xFE
    const val BOARD_ID_UNDETECTED = 0xFF

    /** Marketing name for a board id byte. */
    fun boardIdName(id: Int): String = when (id) {
        BOARD_ID_JELLYBEAN -> "Jellybean"
        BOARD_ID_JAWBREAKER -> "Jawbreaker"
        BOARD_ID_HACKRF1_OG -> "HackRF One OG"
        BOARD_ID_RAD1O -> "rad1o"
        BOARD_ID_HACKRF1_R9 -> "HackRF One R9"
        BOARD_ID_PRALINE -> "Praline"
        BOARD_ID_UNRECOGNIZED -> "unrecognized"
        BOARD_ID_UNDETECTED -> "undetected"
        else -> "unknown"
    }

    /**
     * Revision name for a revision byte. Values at or above 0x80 mark a
     * Great Scott Gadgets board; the same revision below that came from
     * another maker.
     */
    fun boardRevName(rev: Int): String = when (rev and 0x7F) {
        0 -> "older than r6"
        1 -> "r6"
        2 -> "r7"
        3 -> "r8"
        4 -> "r9"
        5 -> "r10"
        6 -> "Praline r0.1"
        7 -> "Praline r0.2"
        8 -> "Praline r0.3"
        9 -> "Praline r1.0"
        10 -> "Praline r1.1"
        11 -> "Praline r1.2"
        else -> if (rev == 0xFF) "undetected" else "unrecognized"
    }

    /** True when [rev] identifies a genuine Great Scott Gadgets board. */
    fun boardRevIsGsg(rev: Int): Boolean = rev in 0x80..0xFD

    const val PLATFORM_JAWBREAKER = 1 shl 0
    const val PLATFORM_HACKRF1_OG = 1 shl 1
    const val PLATFORM_RAD1O = 1 shl 2
    const val PLATFORM_HACKRF1_R9 = 1 shl 3
    const val PLATFORM_PRALINE = 1 shl 4

    /** Readable list of the board families this firmware supports. */
    fun platformName(bits: Int): String {
        val names = buildList {
            if (bits and PLATFORM_JAWBREAKER != 0) add("Jawbreaker")
            if (bits and PLATFORM_HACKRF1_OG != 0) add("HackRF One OG")
            if (bits and PLATFORM_RAD1O != 0) add("rad1o")
            if (bits and PLATFORM_HACKRF1_R9 != 0) add("HackRF One R9")
            if (bits and PLATFORM_PRALINE != 0) add("Praline")
        }
        return if (names.isEmpty()) "unknown" else names.joinToString("/")
    }

    /** Decode the platform word, which arrives big-endian unlike every
     *  other multi-byte reply on this device. */
    fun parsePlatform(buf: ByteArray): Int {
        if (buf.size < 4) return 0
        return ((buf[0].toInt() and 0xFF) shl 24) or
            ((buf[1].toInt() and 0xFF) shl 16) or
            ((buf[2].toInt() and 0xFF) shl 8) or
            (buf[3].toInt() and 0xFF)
    }

    /** Part id and serial number: two little-endian u32 then four more. */
    data class PartIdSerial(val partId: LongArray, val serialNo: LongArray) {
        /** The serial as 32 hex digits, the form printed on the board. */
        fun serialHex(): String = serialNo.joinToString("") {
            it.toString(16).padStart(8, '0')
        }

        override fun equals(other: Any?): Boolean =
            other is PartIdSerial && partId.contentEquals(other.partId) &&
                serialNo.contentEquals(other.serialNo)

        override fun hashCode(): Int =
            partId.contentHashCode() * 31 + serialNo.contentHashCode()
    }

    fun parsePartIdSerial(buf: ByteArray): PartIdSerial? {
        if (buf.size < 24) return null
        val part = LongArray(2) { leInt(buf, it * 4).toLong() and 0xFFFFFFFFL }
        val serial = LongArray(4) { leInt(buf, 8 + it * 4).toLong() and 0xFFFFFFFFL }
        return PartIdSerial(part, serial)
    }

    // ---- explicit tuning ----------------------------------------------------

    const val RF_PATH_FILTER_BYPASS = 0
    const val RF_PATH_FILTER_LOW_PASS = 1
    const val RF_PATH_FILTER_HIGH_PASS = 2

    /** Readable name of an RF path filter selection. */
    fun filterPathName(path: Int): String = when (path) {
        RF_PATH_FILTER_BYPASS -> "mixer bypass"
        RF_PATH_FILTER_LOW_PASS -> "low pass filter"
        RF_PATH_FILTER_HIGH_PASS -> "high pass filter"
        else -> "invalid filter path"
    }

    /**
     * True when this explicit tuning is one the hardware can actually reach.
     *
     * The IF must land inside the MAX2837's passband, and outside BYPASS the
     * RFFC5072 mixer needs an LO it can synthesise. Anything else is refused
     * rather than programmed, because a synthesiser that fails to lock
     * reports no error — the board simply receives noise.
     */
    fun freqExplicitValid(ifFreqHz: Long, loFreqHz: Long, path: Int): Boolean {
        if (ifFreqHz < 2_000_000_000L || ifFreqHz > 3_000_000_000L) return false
        if (path != RF_PATH_FILTER_BYPASS &&
            (loFreqHz < 84_375_000L || loFreqHz > 5_400_000_000L)
        ) {
            return false
        }
        return path in RF_PATH_FILTER_BYPASS..RF_PATH_FILTER_HIGH_PASS
    }

    /** Explicit tuning payload: u64 LE IF, u64 LE LO, u8 path (17 bytes). */
    fun freqExplicitParams(ifFreqHz: Long, loFreqHz: Long, path: Int): ByteArray {
        val data = ByteArray(17)
        putLeLong(data, 0, ifFreqHz)
        putLeLong(data, 8, loFreqHz)
        data[16] = path.toByte()
        return data
    }

    // ---- Opera Cake antenna switcher ----------------------------------------

    const val OPERACAKE_MAX_BOARDS = 8
    const val OPERACAKE_MAX_DWELL_TIMES = 16
    const val OPERACAKE_MAX_FREQ_RANGES = 8

    const val OPERACAKE_PA1 = 0
    const val OPERACAKE_PA2 = 1
    const val OPERACAKE_PA3 = 2
    const val OPERACAKE_PA4 = 3
    const val OPERACAKE_PB1 = 4
    const val OPERACAKE_PB2 = 5
    const val OPERACAKE_PB3 = 6
    const val OPERACAKE_PB4 = 7

    const val OPERACAKE_MODE_MANUAL = 0
    const val OPERACAKE_MODE_FREQUENCY = 1
    const val OPERACAKE_MODE_TIME = 2

    fun operacakeValidAddress(address: Int): Boolean =
        address in 0 until OPERACAKE_MAX_BOARDS

    /** Port label as printed on the switch board (A1..A4, B1..B4). */
    fun operacakePortName(port: Int): String =
        if (port in OPERACAKE_PA1..OPERACAKE_PB4) {
            (if (port <= OPERACAKE_PA4) "A" else "B") + ((port % 4) + 1)
        } else {
            "invalid"
        }

    /**
     * The two ports must sit on OPPOSITE sides of the switch: one A, one B.
     * Both on the same side is a routing the hardware cannot make, so it is
     * refused instead of applied by halves.
     */
    fun operacakePortsValid(portA: Int, portB: Int): Boolean {
        if (portA > OPERACAKE_PB4 || portB > OPERACAKE_PB4) return false
        if (portA < 0 || portB < 0) return false
        val aSideA = portA <= OPERACAKE_PA4
        val aSideB = portB <= OPERACAKE_PA4
        return aSideA != aSideB
    }

    /** Port-select index word: port A in the low byte, port B in the high. */
    fun operacakePortsIndex(portA: Int, portB: Int): Int =
        (portA and 0xFF) or ((portB and 0xFF) shl 8)

    /**
     * Frequency-range table (mode FREQUENCY): five bytes per entry — min MHz
     * big-endian u16, max MHz big-endian u16, port u8. The byte order is not
     * a slip: this payload is swapped relative to every other one the board
     * accepts.
     */
    fun operacakeFreqRangesParams(ranges: List<Triple<Int, Int, Int>>): ByteArray {
        require(ranges.size in 1..OPERACAKE_MAX_FREQ_RANGES) {
            "1..$OPERACAKE_MAX_FREQ_RANGES ranges"
        }
        val data = ByteArray(ranges.size * 5)
        ranges.forEachIndexed { i, (minMHz, maxMHz, port) ->
            val p = i * 5
            data[p] = ((minMHz ushr 8) and 0xFF).toByte()
            data[p + 1] = (minMHz and 0xFF).toByte()
            data[p + 2] = ((maxMHz ushr 8) and 0xFF).toByte()
            data[p + 3] = (maxMHz and 0xFF).toByte()
            data[p + 4] = port.toByte()
        }
        return data
    }

    /** Dwell-time table (mode TIME): u32 LE sample count then u8 port. */
    fun operacakeDwellTimesParams(dwells: List<Pair<Int, Int>>): ByteArray {
        require(dwells.size in 1..OPERACAKE_MAX_DWELL_TIMES) {
            "1..$OPERACAKE_MAX_DWELL_TIMES dwell times"
        }
        val data = ByteArray(dwells.size * 5)
        dwells.forEachIndexed { i, (dwell, port) ->
            putLeInt(data, i * 5, dwell)
            data[i * 5 + 4] = port.toByte()
        }
        return data
    }

    // ---- clock, trigger and LED control -------------------------------------

    /** LED state byte: bit 0 USB, bit 1 RX, bit 2 TX (0 = firmware default). */
    const val LED_USB = 1
    const val LED_RX = 2
    const val LED_TX = 4

    const val CLKIN_SIGNAL_P1 = 0
    const val CLKIN_SIGNAL_P22 = 1

    const val P1_SIGNAL_TRIGGER_IN = 0
    const val P1_SIGNAL_AUX_CLK1 = 1
    const val P1_SIGNAL_CLKIN = 2
    const val P1_SIGNAL_TRIGGER_OUT = 3
    const val P1_SIGNAL_P22_CLKIN = 4
    const val P1_SIGNAL_P2_5 = 5
    const val P1_SIGNAL_NC = 6
    const val P1_SIGNAL_AUX_CLK2 = 7

    const val P2_SIGNAL_CLK3 = 0
    const val P2_SIGNAL_TRIGGER_IN = 2
    const val P2_SIGNAL_TRIGGER_OUT = 3

    /** Steps of the real-time-clock oscillator test, carried in wIndex. */
    const val RTC_START_OSCILLATOR = 0
    const val RTC_START_FREQ_MONITOR = 1
    const val RTC_READ_FREQ_MONITOR = 2
    const val RTC_STOP_OSCILLATOR = 3

    /** The 32 kHz oscillator passed when the frequency monitor reports 1. */
    fun rtcOscPassed(count: Int): Boolean = count == 1

    /** ADC channel travels in wIndex; bit 7 is a mode flag, not part of it. */
    fun adcChannelValid(channel: Int): Boolean = (channel and 0x80.inv()) <= 7

    /** Self-test reply: a pass flag then a NUL-terminated ASCII message. */
    const val SELFTEST_SIZE = 4096

    data class SelfTest(val pass: Boolean, val message: String)

    fun parseSelfTest(buf: ByteArray): SelfTest? {
        if (buf.isEmpty()) return null
        // The pass flag occupies byte 0; the message starts at 1 and is
        // NUL-terminated. The terminator must be searched for FROM byte 1 —
        // searching the whole buffer finds the pass flag itself whenever the
        // test failed, and the message then runs to the end of the reply,
        // carrying kilobytes of NUL padding into the UI.
        var end = buf.size
        for (i in 1 until buf.size) {
            if (buf[i] == 0.toByte()) {
                end = i
                break
            }
        }
        val msg = if (end > 1) String(buf, 1, end - 1, Charsets.US_ASCII) else ""
        return SelfTest(buf[0].toInt() != 0, msg.trim())
    }

    // ---- register access ----------------------------------------------------
    //
    // Direct access to the front-end chips, for diagnostics. Reads put the
    // register in wIndex and answer in the payload; writes carry the VALUE in
    // wValue and the register in wIndex. That asymmetry is the hardware's, and
    // it belongs in one tested place rather than at each call site.

    /** Radio register write payload: u8 register then u64 LE value. */
    fun radioRegWriteParams(register: Int, value: Long): ByteArray {
        val data = ByteArray(9)
        data[0] = register.toByte()
        putLeLong(data, 1, value)
        return data
    }

    fun leU16(buf: ByteArray, offset: Int = 0): Int =
        if (buf.size < offset + 2) -1 else leShort(buf, offset)

    fun leU32(buf: ByteArray, offset: Int = 0): Long =
        if (buf.size < offset + 4) -1L else leInt(buf, offset).toLong() and 0xFFFFFFFFL

    fun leU64(buf: ByteArray, offset: Int = 0): Long {
        if (buf.size < offset + 8) return -1L
        var v = 0L
        for (k in 7 downTo 0) v = (v shl 8) or (buf[offset + k].toLong() and 0xFF)
        return v
    }

    /** bmRequestType vendor|device, host→device / device→host. */
    const val TYPE_VENDOR_OUT = 0x40
    const val TYPE_VENDOR_IN = 0xC0

    // ---- payload encodings ---------------------------------------------------

    /** Tuning payload: uint32 LE whole MHz, then uint32 LE Hz remainder. */
    fun freqParams(hz: Long): ByteArray {
        val mhz = (hz / 1_000_000L).toInt()
        val rem = (hz - mhz * 1_000_000L).toInt()
        val data = ByteArray(8)
        putLeInt(data, 0, mhz)
        putLeInt(data, 4, rem)
        return data
    }

    /** Sample-rate payload: uint32 LE Hz + uint32 LE divider. */
    fun sampleRateParams(hz: Int, divider: Int = 1): ByteArray {
        val data = ByteArray(8)
        putLeInt(data, 0, hz)
        putLeInt(data, 4, divider)
        return data
    }

    /**
     * Pick the (frequency, divider) PAIR that lands on the requested sample
     * rate, instead of assuming a divider of 1.
     *
     * The Si5351C cannot synthesise an arbitrary rate directly, so the rate is
     * expressed as a rational n/d whose numerator the synthesiser can make.
     * Pinning the divider at 1 quietly rounds any rate it cannot hit exactly:
     * the host's idea of the sample rate and the board's then differ, and
     * every frequency the DSP derives from it is wrong by that same ratio.
     *
     * The search walks the fractional part of the rate through repeated
     * addition and stops at the first multiplier whose accumulated fraction
     * fits the mantissa bits that matter at this exponent.
     */
    fun sampleRateAuto(freqHz: Double): Pair<Int, Int> {
        val maxN = 32
        val freqFrac = 1.0 + freqHz - freqHz.toLong()
        val exponent = ((java.lang.Double.doubleToRawLongBits(freqHz) ushr 52).toInt() and 0x7FF) - 1023
        var mask = (1L shl 52) - 1
        val frac = java.lang.Double.doubleToRawLongBits(freqFrac) and mask
        // Every rate the board accepts (<= 20 MHz) keeps this shift inside
        // 32 bits; the clamp only guards against a nonsense argument.
        val shift = (exponent + 4).coerceIn(0, 62)
        mask = mask and ((1L shl shift) - 1).inv()
        var acc = 0L
        var i = 1
        while (i < maxN) {
            acc += frac
            if ((acc and mask) == 0L || (acc.inv() and mask) == 0L) break
            i++
        }
        if (i == maxN) i = 1
        return Pair((freqHz * i + 0.5).toInt(), i)
    }

    /** BASEBAND_FILTER_BANDWIDTH_SET splits the Hz across wValue/wIndex. */
    fun filterBwValue(bandwidthHz: Int): Int = bandwidthHz and 0xFFFF
    fun filterBwIndex(bandwidthHz: Int): Int = bandwidthHz ushr 16

    // ---- rules ----------------------------------------------------------------

    /** MAX2837 anti-alias filter steps, ascending Hz. */
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

    /**
     * Strictly-below filter choice: steps down a rung even on an exact hit.
     *
     * The difference is not cosmetic. At 8 MSps the 75 % rule asks for exactly
     * 6 MHz, which IS a table entry; selecting it puts the filter corner right
     * at the edge of the retained band, so the transition region folds back
     * into the top of the spectrum. Stepping down keeps the corner inside the
     * band it is protecting.
     */
    fun basebandFilterRoundDownLt(bandwidthHz: Int): Int {
        val idx = BASEBAND_FILTERS_HZ.indexOfFirst { it >= bandwidthHz }
        if (idx < 0) return BASEBAND_FILTERS_HZ.last()
        if (idx == 0) return BASEBAND_FILTERS_HZ[0]
        return BASEBAND_FILTERS_HZ[idx - 1]
    }

    /**
     * Anti-alias bandwidth picked automatically on a rate change: 75 % of the
     * sample rate, rounded strictly down.
     */
    fun basebandFilterForSampleRate(sampleRateHz: Int): Int =
        basebandFilterRoundDownLt((0.75 * sampleRateHz).toInt())

    /** LNA (IF) gain: 0–40 dB in 8 dB steps; the low bits are masked off. */
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

    // ---- sweep mode -------------------------------------------------------------

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

    /**
     * Spectrum resolution for a given sample rate, in bins.
     *
     * A fixed bin count is wrong for this board specifically: it is the only
     * radio here that spans 20 MHz, and 800 bins across that span is 25 kHz
     * per bin — wide enough to smear a whole FM channel into one column, on a
     * screen with more pixels than that across. The count follows the span so
     * the bin stays in the same order of magnitude at every rate, and stops
     * at 4096 because past roughly one bin per pixel nothing more is visible.
     */
    fun spectrumBinsFor(sampleRateHz: Int): Int = when {
        sampleRateHz <= 2_500_000 -> 1024
        sampleRateHz <= 5_000_000 -> 2048
        else -> 4096
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
    // in time. A starved host therefore hard-gates the envelope at
    // an audio rate, which on the air is broadband splatter — not a dropout.

    /** Bulk-out block the firmware schedules at a time. */
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
     * Rendered blocks kept in the pool. Four is enough depth for the USB
     * thread to always have the next block ready the instant the previous one
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

    // ---- sample-transfer loop state ---------------------------------------------

    /** Byte length of the `hackrf_m0_state` struct on the wire. */
    const val M0_STATE_SIZE = 40

    /**
     * The board's own account of how well it was fed. [numShortfalls] staying
     * put across a transmission is the proof that the host never starved the
     * DAC.
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

    /** Decode the 40-byte little-endian transfer-loop state reply. */
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

    private fun putLeLong(dst: ByteArray, offset: Int, value: Long) {
        for (k in 0 until 8) {
            dst[offset + k] = ((value ushr (8 * k)) and 0xFF).toByte()
        }
    }

    private fun putLeInt(dst: ByteArray, offset: Int, value: Int) {
        dst[offset] = (value and 0xFF).toByte()
        dst[offset + 1] = ((value ushr 8) and 0xFF).toByte()
        dst[offset + 2] = ((value ushr 16) and 0xFF).toByte()
        dst[offset + 3] = ((value ushr 24) and 0xFF).toByte()
    }
}
