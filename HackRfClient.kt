/*
 * libhackrfk - Kotlin driver for the HackRF SDR (Android USB host)
 *
 * USB transport, streaming and control lifecycle. The wire format itself
 * lives in the Android-free [HackRfProtocol] codec.
 *
 * Includes material under the following copyright notices:
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
import com.isaklab.isdrdrivers.core.LnaGainCapable
import com.isaklab.isdrdrivers.core.AnalogFilterCapable
import com.isaklab.isdrdrivers.core.AntennaPowerCapable
import com.isaklab.isdrdrivers.core.TransmitCapable
import com.isaklab.isdrdrivers.core.RadioClient

import android.app.PendingIntent
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.hardware.usb.UsbConstants
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbDeviceConnection
import android.hardware.usb.UsbEndpoint
import android.hardware.usb.UsbInterface
import android.hardware.usb.UsbManager
import android.os.Build
import android.util.Log
import com.isaklab.isdrproto.BoardControls
import com.isaklab.isdrdrivers.core.FFTProcessor
import com.isaklab.isdrdrivers.core.FloatRing
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.TimeUnit
import kotlin.concurrent.thread
import kotlin.coroutines.resume
import kotlinx.coroutines.suspendCancellableCoroutine

/**
 * HackRF USB client (half-duplex RX + TX). The wire encodings live in the
 * Android-free [HackRfProtocol]; this class owns the USB transport: vendor
 * control requests over endpoint 0, signed 8-bit interleaved IQ from bulk-IN
 * 0x81 (RX) and to bulk-OUT 0x02 (TX).
 *
 * Host contract matches the other clients: interleaved float IQ in [-1,1]
 * plus a power spectrum (skippable via [spectrumEnabled]); TX input is
 * 48 kS/s IQ, raised to [HackRfProtocol.TX_BOARD_RATE] by the polyphase
 * [TxInterpolator].
 *
 * Transmit runs on TWO threads. The bulk-OUT endpoint must never wait on
 * computation: the board holds only [HackRfProtocol.DEVICE_TX_BUFFER_BYTES]
 * (13.65 ms) while transmitting, and every byte it misses is clocked out as a
 * zero — a gate in the envelope at audio rate, which on the air is splatter,
 * not a dropout. So [txRenderLoop] always has the next block ready before
 * [txWriteLoop] asks for it. This is correctness, not optimisation.
 */
class HackRfClient(
    /** Null only under test, where no USB stack exists to connect through. */
    private val context: Context?,
    /** (power spectrum in dB, interleaved IQ samples i0,q0,i1,q1,... in [-1,1]) */
    private val onDataReceived: (FloatArray, FloatArray) -> Unit,
    private val onConnectionStatusChanged: (Boolean, String) -> Unit,
    /**
     * Receive-gap telemetry: cumulative count of stream discontinuities
     * (firmware shortfalls plus host-side ring drops) since RX started.
     * Called from the RX processing thread, at most about once per second
     * and only when the count changes.
     */
    private val onRxGaps: ((Long) -> Unit)? = null,
) : RadioClient, TransmitCapable, AntennaPowerCapable, AnalogFilterCapable, LnaGainCapable {
    companion object {
        private const val TAG = "HackRfClient"
        private val EMPTY_SPECTRUM = FloatArray(0)
        private const val PERMISSION_ACTION = "com.isaklab.libhackrfk.USB_PERMISSION"
        private const val CONTROL_TIMEOUT_MS = 1000
        private const val BULK_TIMEOUT_MS = 500

        /** True when (vendorId, productId) is a known HackRF board. */
        fun findKnownDevice(vendorId: Int, productId: Int): Boolean =
            HackRfProtocol.isKnownDevice(vendorId, productId)
    }

    /**
     * When false, IQ blocks are delivered with an empty spectrum and the FFT
     * is skipped (the host has no visible spectrum consumer).
     */
    @Volatile override var spectrumEnabled: Boolean = true

    private var connection: UsbDeviceConnection? = null
    private var iface: UsbInterface? = null
    @Volatile private var transport: HackRfTransport? = null
    @Volatile private var running = false
    @Volatile private var rxStarted = false
    @Volatile private var transmitting = false
    /**
     * True between PTT release and the moment the tail has actually left the
     * board: new samples are refused, the renderer runs its unkey ramp and
     * the mode switch waits for the DAC to drain. See [drainAndStopTx].
     */
    @Volatile private var txDraining = false
    private var rxUsbThread: Thread? = null
    private var rxDspThread: Thread? = null
    private var txRenderThread: Thread? = null
    private var txWriteThread: Thread? = null
    @Volatile private var txRenderer: TxBlockRenderer? = null

    // RX reassembly and gap accounting. The assembler carries every received
    // byte across transfer boundaries (short and odd-length transfers
    // included), so a hiccup on the wire can shift latency but never tear
    // I/Q alignment or silently discard samples.
    private val rxAssembler = RxAssembler()
    /** Firmware shortfall counter at RX start, for the running delta. */
    @Volatile private var rxStartShortfalls = -1
    /** Cumulative RX gap count last published through [onRxGaps]. */
    @Volatile private var lastReportedGaps = -1L
    private var fft: FFTProcessor? = null
    /** Bin count the current [fft] was built for. */
    @Volatile private var fftBins = 0

    /** RX sample rate to restore after a TX-over (half-duplex switch). */
    @Volatile private var rxSampleRate = 4_000_000

    /** Operator's anti-alias filter width in Hz; 0 follows the sample rate. */
    @Volatile private var manualFilterHz = 0

    /**
     * RX and TX frequencies are tracked SEPARATELY even though the board has a
     * single LO, because the host does not ask for the same one in both
     * directions: it tunes the receiver deliberately off the operator's VFO to
     * keep the wanted signal away from the zero-IF DC spike and corrects the
     * offset in its own NCO, while the transmitter must put the carrier
     * exactly on the VFO. Collapsing the two into one sticky value made the
     * first transmission overwrite the receiver's tuning, so returning to RX
     * came back offset by the guard — audible as a fixed dial error.
     */
    @Volatile private var rxFreqHz = 100_000_000L
    @Volatile private var txFreqHz = 100_000_000L

    // Last front-end state, re-asserted after every transceiver-mode change:
    // the firmware drops the antenna bias power when returning through IDLE,
    // so it can never stay enabled across modes. Re-sending the gains costs
    // nothing and removes any ambiguity about what survived the switch.
    @Volatile private var lastLnaGain = HackRfProtocol.DEFAULT_LNA_DB
    @Volatile private var lastVgaGain = HackRfProtocol.DEFAULT_VGA_DB
    @Volatile private var lastTxVgaGain = HackRfProtocol.DEFAULT_TXVGA_DB
    @Volatile private var lastAmpEnable = false
    @Volatile private var lastAntennaPower = false

    // TX queue: interleaved 48 kS/s float IQ, drop-oldest beyond 2 s. A
    // primitive ring, not an ArrayDeque<Float> — the deque boxed ~100k Floats
    // a second while keyed, and a GC pause on the transmit path is a shortfall
    // (see HackRfProtocol.DEVICE_TX_BUFFER_BYTES: the board holds 13.65 ms).
    private val txQueue = FloatRing(HackRfProtocol.TX_INPUT_RATE * 2 * 2)
    private val txLock = Any()

    // Rendered board-rate blocks travelling between the render thread and the
    // USB thread, and back. Bounded, so the renderer can never run away and
    // the pool never allocates on the hot path.
    private val txFree = ArrayBlockingQueue<ByteArray>(HackRfProtocol.TX_BLOCKS_IN_FLIGHT)
    private val txReady = ArrayBlockingQueue<ByteArray>(HackRfProtocol.TX_BLOCKS_IN_FLIGHT)

    /** Shortfall count read off the board when TX started, for the delta at unkey. */
    @Volatile private var txStartShortfalls = -1

    /**
     * Control-protocol version the firmware reports in bcdDevice.
     *
     * Every request added after the original firmware is gated on this.
     * Sending one the firmware does not implement is not a no-op: the control
     * endpoint STALLS, and a stalled endpoint fails the NEXT request too — so
     * one unsupported query on an older board takes an unrelated, perfectly
     * valid tuning command down with it.
     */
    @Volatile private var usbApiVersion = 0

    /** Board identity, read once at connect. */
    @Volatile var boardId: Int = HackRfProtocol.BOARD_ID_UNDETECTED
        private set
    @Volatile var boardRevision: Int = -1
        private set
    @Volatile var supportedPlatform: Int = 0
        private set
    @Volatile var firmwareVersion: String = ""
        private set
    @Volatile var serialNumber: String = ""
        private set

    /** True when the running firmware implements [required] (bcdDevice gate). */
    fun supportsApi(required: Int): Boolean =
        HackRfProtocol.apiAtLeast(usbApiVersion, required)

    /** The board's control-protocol version, e.g. "1.09" (empty if unknown). */
    fun usbApiVersionName(): String =
        if (usbApiVersion == 0) "" else HackRfProtocol.usbApiVersionName(usbApiVersion)

    /**
     * Which controls this board can actually honour, as the wire bitmask.
     *
     * Computed HERE because the rule is a firmware-version comparison, and
     * the version is only meaningful to the side that speaks USB and gets
     * stalled when it guesses wrong. The app receives the answer, never the
     * criterion.
     */
    fun supportedControls(): Int {
        var m = 0
        fun add(bit: Int, required: Int) {
            if (supportsApi(required)) m = m or bit
        }
        add(BoardControls.ANTENNA_SWITCH_PORTS, HackRfProtocol.API_OPERACAKE_PORTS)
        add(BoardControls.ANTENNA_SWITCH_MODE, HackRfProtocol.API_OPERACAKE_MODE)
        add(BoardControls.ANTENNA_SWITCH_TABLES, HackRfProtocol.API_OPERACAKE_RANGES)
        add(BoardControls.HARDWARE_SYNC, HackRfProtocol.API_HW_SYNC)
        add(BoardControls.BOARD_UI, HackRfProtocol.API_UI_ENABLE)
        add(BoardControls.STREAM_COUNTERS, HackRfProtocol.API_M0_STATE)
        add(BoardControls.WATCHDOG_LIMITS, HackRfProtocol.API_OVERRUN_LIMITS)
        add(BoardControls.CLOCK_OUTPUT, HackRfProtocol.API_CPLD_CHECKSUM)
        add(BoardControls.CLOCK_INPUT_SELECT, HackRfProtocol.API_PRO_SIGNALS)
        add(BoardControls.PANEL_LEDS, HackRfProtocol.API_SET_LEDS)
        add(BoardControls.ANTENNA_POWER_PER_MODE, HackRfProtocol.API_BIAS_T_OPTS)
        add(BoardControls.SELF_TEST, HackRfProtocol.API_PRO_SIGNALS)
        add(BoardControls.NARROWBAND_FILTER, HackRfProtocol.API_PRO_SIGNALS)
        add(BoardControls.RESET, HackRfProtocol.API_HW_SYNC)
        return m
    }

    /**
     * Everything the board says about itself, gathered at connect so the UI
     * can name the hardware instead of showing a generic "HackRF".
     */
    data class BoardInfo(
        val boardId: Int,
        val boardName: String,
        val boardRevision: Int,
        val revisionName: String,
        /** False when the firmware is too old to report a revision at all. */
        val revisionKnown: Boolean,
        val isGenuineGsg: Boolean,
        val platformBits: Int,
        val platformName: String,
        val firmwareVersion: String,
        val usbApiVersion: Int,
        val usbApiName: String,
        val serialNumber: String,
    )

    /**
     * True only when the board actually reported a revision AND it is not a
     * Great Scott Gadgets one. Firmware older than the revision request
     * reports nothing, and "I could not ask" must never be presented as
     * "this board is a fake".
     */
    fun revisionIsKnown(): Boolean = boardRevision in 0..0xFD

    fun boardInfo(): BoardInfo = BoardInfo(
        boardId = boardId,
        boardName = HackRfProtocol.boardIdName(boardId),
        boardRevision = boardRevision,
        revisionName = if (boardRevision < 0) "" else HackRfProtocol.boardRevName(boardRevision),
        revisionKnown = revisionIsKnown(),
        isGenuineGsg = !revisionIsKnown() || HackRfProtocol.boardRevIsGsg(boardRevision),
        platformBits = supportedPlatform,
        platformName = HackRfProtocol.platformName(supportedPlatform),
        firmwareVersion = firmwareVersion,
        usbApiVersion = usbApiVersion,
        usbApiName = usbApiVersionName(),
        serialNumber = serialNumber,
    )

    /**
     * Claims the USB interface, locates the bulk IN/OUT endpoints, starts the 
     * transceiver, and prepares the client for operation.
     * 
     * Requires USB permission to have been granted. Clears the firmware auto-tx-flush
     * to avoid blocking on subsequent mode switches.
     *
     * @return true if successfully connected and initialized, false otherwise.
     */
    override suspend fun connect(): Boolean {
        val ctx = context
        if (ctx == null) {
            onConnectionStatusChanged(false, "No USB context")
            return false
        }
        val usb = ctx.getSystemService(Context.USB_SERVICE) as UsbManager
        val device = usb.deviceList.values
            .firstOrNull { findKnownDevice(it.vendorId, it.productId) }
        if (device == null) {
            onConnectionStatusChanged(false, "No HackRF found")
            return false
        }
        if (!awaitPermission(usb, device)) {
            onConnectionStatusChanged(false, "USB permission denied")
            return false
        }
        val conn = usb.openDevice(device)
        if (conn == null) {
            onConnectionStatusChanged(false, "Could not open HackRF")
            return false
        }
        val itf = device.getInterface(0)
        if (!conn.claimInterface(itf, true)) {
            conn.close()
            onConnectionStatusChanged(false, "Could not claim HackRF interface")
            return false
        }
        var bulkIn: UsbEndpoint? = null
        var bulkOut: UsbEndpoint? = null
        for (i in 0 until itf.endpointCount) {
            val ep = itf.getEndpoint(i)
            if (ep.type == UsbConstants.USB_ENDPOINT_XFER_BULK) {
                if (ep.direction == UsbConstants.USB_DIR_IN) bulkIn = ep
                else bulkOut = ep
            }
        }
        if (bulkIn == null) {
            conn.releaseInterface(itf)
            conn.close()
            onConnectionStatusChanged(false, "HackRF RX endpoint missing")
            return false
        }
        connection = conn
        iface = itf
        transport = AndroidUsbTransport(conn, bulkIn, bulkOut)
        fftBins = HackRfProtocol.spectrumBinsFor(rxSampleRate)
        fft = FFTProcessor(fftBins)
        running = true
        // Clears the firmware's auto-tx-flush so leaving TX is immediate
        // (otherwise mode-off blocks until the transmit buffer fully drains).
        usbApiVersion = readUsbApiVersion(conn)
        if (supportsApi(HackRfProtocol.API_BUFFER_SIZE)) {
            vendorIn(HackRfProtocol.REQ_GET_BUFFER_SIZE, 0, 0, 4)
        }
        readIdentity()
        onConnectionStatusChanged(true, connectedStatusLine())
        return true
    }

    /** bcdDevice out of the raw USB device descriptor; 0 when unreadable. */
    private fun readUsbApiVersion(conn: UsbDeviceConnection): Int {
        val desc = try {
            conn.rawDescriptors
        } catch (_: Exception) {
            null
        } ?: return 0
        if (desc.size < 14) return 0
        return (desc[12].toInt() and 0xFF) or ((desc[13].toInt() and 0xFF) shl 8)
    }

    /** Read board id, revision, platform, serial and firmware string once. */
    private fun readIdentity() {
        firmwareVersion = vendorIn(HackRfProtocol.REQ_VERSION_STRING_READ, 0, 0, 64)
            ?.toString(Charsets.US_ASCII)?.trim() ?: ""
        boardId = vendorIn(HackRfProtocol.REQ_BOARD_ID_READ, 0, 0, 1)
            ?.firstOrNull()?.toInt()?.and(0xFF) ?: HackRfProtocol.BOARD_ID_UNDETECTED
        serialNumber = partIdSerial()?.serialHex() ?: ""
        boardRevision = if (supportsApi(HackRfProtocol.API_BOARD_REV)) {
            vendorIn(HackRfProtocol.REQ_BOARD_REV_READ, 0, 0, 1)
                ?.firstOrNull()?.toInt()?.and(0xFF) ?: -1
        } else {
            -1
        }
        supportedPlatform = if (supportsApi(HackRfProtocol.API_SUPPORTED_PLATFORM)) {
            vendorIn(HackRfProtocol.REQ_SUPPORTED_PLATFORM_READ, 0, 0, 4)
                ?.let { HackRfProtocol.parsePlatform(it) } ?: 0
        } else {
            0
        }
        // A counterfeit board is worth saying out loud once: it is the usual
        // explanation for a radio that tunes but performs nothing like the
        // datasheet, and the revision byte is the only place it shows.
        if (boardRevision in 0..0x7F) {
            Log.w(
                TAG,
                "board revision ${HackRfProtocol.boardRevName(boardRevision)} is not a " +
                    "Great Scott Gadgets board",
            )
        }
    }

    /** Status line naming the actual hardware, not a generic "HackRF". */
    private fun connectedStatusLine(): String {
        val parts = buildList {
            if (boardId != HackRfProtocol.BOARD_ID_UNDETECTED) {
                add(HackRfProtocol.boardIdName(boardId))
            }
            if (boardRevision >= 0) add(HackRfProtocol.boardRevName(boardRevision))
            if (firmwareVersion.isNotBlank()) add("fw $firmwareVersion")
        }
        return if (parts.isEmpty()) "Connected" else "Connected " + parts.joinToString(" ")
    }

    /**
     * Releases USB interfaces, terminates running TX/RX threads, and places the 
     * transceiver in the OFF state. Safe to call idempotently or when partially initialized.
     */
    override fun disconnect() {
        running = false
        rxStarted = false
        transmitting = false
        // The sweep loop reads the same endpoint as the RX loop and was the
        // one thread disconnect never stopped: it kept calling bulkTransfer
        // on a connection about to be closed.
        sweeping = false
        try {
            setTransceiverMode(HackRfProtocol.MODE_OFF)
        } catch (e: Exception) {
            Log.w(TAG, "could not park the transceiver on disconnect", e)
        }
        transport?.cancelRx()
        rxAssembler.wake()
        rxUsbThread?.join(1000)
        rxUsbThread = null
        rxDspThread?.join(1000)
        rxDspThread = null
        sweepThread?.join(1000)
        sweepThread = null
        stopTxThreads()
        transport = null
        iface?.let { connection?.releaseInterface(it) }
        connection?.close()
        connection = null
        iface = null
        onConnectionStatusChanged(false, "Disconnected")
    }

    // ==================== control requests ====================

    /**
     * Retunes the receiver. Automatically resets the smoothing filter for the
     * FFT to prevent cross-fading artifacts on screen. Supported range depends
     * on the MAX2837/RFFC5072 limit (typically 1 MHz to 6 GHz).
     *
     * While keyed this only records the frequency: the single LO belongs to
     * the transmitter until unkey, and [startRx] applies it on the way back.
     */
    override fun setFrequency(hz: Long) {
        // Moving the dial invalidates an explicit override: its oscillator
        // and filter were chosen for the OLD frequency, and silently keeping
        // them would tune the front end somewhere the operator did not ask
        // for. Automatic tuning takes over again.
        if (hz != rxFreqHz) explicitTuning = null
        rxFreqHz = hz
        if (transmitting) return
        applyRxTuning()
    }

    /**
     * Retunes the transmitter. The host puts the carrier exactly here, with
     * none of the receiver's zero-IF guard offset — see [rxFreqHz]. While
     * receiving this only records the frequency; [setPtt] applies it on key.
     */
    override fun setTxFrequency(hz: Long) {
        txFreqHz = hz
        if (!transmitting) return
        tuneTo(hz)
    }

    /**
     * An explicit IF/oscillator/filter choice in force on the receive path,
     * or null when the front end follows the automatic mapping.
     *
     * It has to be re-applied, not merely sent once: every plain retune
     * reprograms all three from the requested frequency, so leaving TX,
     * leaving a sweep or re-asserting the receiver would otherwise throw the
     * override away without a word.
     */
    @Volatile private var explicitTuning: Triple<Long, Long, Int>? = null

    /** Tune the receiver, honouring an explicit override when one is set. */
    private fun applyRxTuning() {
        val explicit = explicitTuning
        if (explicit == null) {
            tuneTo(rxFreqHz)
            return
        }
        val (ifHz, loHz, path) = explicit
        vendorOut(
            HackRfProtocol.REQ_SET_FREQ_EXPLICIT, 0, 0,
            HackRfProtocol.freqExplicitParams(ifHz, loHz, path),
        )
        fft?.resetSmoothing()
    }

    private fun tuneTo(hz: Long) {
        vendorOut(HackRfProtocol.REQ_SET_FREQ, 0, 0, HackRfProtocol.freqParams(hz))
        fft?.resetSmoothing()
    }

    /**
     * Sets the baseband ADC/DAC sample rate. Also automatically adjusts the
     * analog baseband filter bandwidth to 75% of the requested rate to prevent aliasing.
     * Restored automatically upon returning from TX.
     */
    override fun setSampleRate(hz: Int) {
        rxSampleRate = hz
        // Resolution follows the span. Keeping a fixed bin count would make
        // the display coarser every time the operator widens the rate, which
        // is the opposite of why the rate was widened.
        val bins = HackRfProtocol.spectrumBinsFor(hz)
        if (bins != fftBins) {
            fftBins = bins
            fft = FFTProcessor(bins)
        }
        // Deferred while keyed or sweeping, same semantics as setFrequency:
        // while transmitting the converters are clocked at the TX rate, and
        // reprogramming them mid-over shifts the whole transmitted spectrum
        // on the air; during a sweep the firmware owns the rate it was
        // initialised with. The recorded value is applied on the way back to
        // RX (setPtt/stopSweep both re-send it).
        if (transmitting || sweeping) return
        sendSampleRate(hz)
    }

    private fun sendSampleRate(hz: Int) {
        // (frequency, divider) pair, not a bare rate with divider 1: the
        // Si5351C cannot synthesise every rate directly, and a fixed divider
        // silently rounds the ones it cannot hit — after which the board and
        // the host disagree about the sample rate, and every frequency the
        // DSP computes from it is off by that same ratio.
        val (rateHz, divider) = HackRfProtocol.sampleRateAuto(hz.toDouble())
        vendorOut(
            HackRfProtocol.REQ_SAMPLE_RATE_SET, 0, 0,
            HackRfProtocol.sampleRateParams(rateHz, divider),
        )
        // Manual choice wins; 0 means follow the rate automatically.
        val bw = manualFilterHz.takeIf { it > 0 }
            ?.let { HackRfProtocol.basebandFilterFor(it) }
            ?: HackRfProtocol.basebandFilterForSampleRate(hz)
        vendorOut(
            HackRfProtocol.REQ_BASEBAND_FILTER_BW_SET,
            HackRfProtocol.filterBwValue(bw),
            HackRfProtocol.filterBwIndex(bw),
            null,
        )
    }

    /**
     * Anti-alias filter width, in Hz. Zero restores the automatic choice
     * (a fixed fraction of the sample rate).
     *
     * Worth overriding in both directions: narrowing it below the automatic
     * value keeps a strong neighbour out of the converter when watching a
     * quiet channel, and widening it stops the filter from cutting a signal
     * wider than the app expected. The value is snapped to one of the filter
     * steps the hardware actually has.
     */
    override fun setAnalogFilterHz(hz: Int) = setBasebandFilterBandwidth(hz)

    fun setBasebandFilterBandwidth(hz: Int) {
        manualFilterHz = hz.coerceAtLeast(0)
        // Deferred while keyed or sweeping, for the same reason as the
        // sample rate: the analog filter is shared by both directions, and
        // narrowing it under a live transmission clips the signal being
        // radiated. sendSampleRate re-asserts the filter when RX resumes,
        // so the recorded choice is never lost.
        if (transmitting || sweeping) return
        val bw = if (manualFilterHz > 0) {
            HackRfProtocol.basebandFilterFor(manualFilterHz)
        } else {
            HackRfProtocol.basebandFilterForSampleRate(rxSampleRate)
        }
        vendorOut(
            HackRfProtocol.REQ_BASEBAND_FILTER_BW_SET,
            HackRfProtocol.filterBwValue(bw),
            HackRfProtocol.filterBwIndex(bw),
            null,
        )
    }

    /** Filter width actually in force, in Hz. */
    fun basebandFilterHz(): Int = if (manualFilterHz > 0) {
        HackRfProtocol.basebandFilterFor(manualFilterHz)
    } else {
        HackRfProtocol.basebandFilterForSampleRate(rxSampleRate)
    }

    /**
     * Gain requests are the one control family the firmware answers with a
     * DATA STAGE: a vendor IN transfer whose single payload byte is the
     * accept/reject verdict. Sending them as OUT with no data stage is a
     * direction mismatch — the firmware queues a 1-byte IN response the host
     * never reads, and the rejection verdict is silently lost, so a refused
     * gain leaves the host believing a value the hardware never applied.
     *
     * @return true when the firmware confirmed the gain.
     */
    private fun sendGain(request: Int, gain: Int): Boolean {
        val status = vendorIn(request, 0, gain, 1)
        val ok = status != null && status.size == 1 && status[0].toInt() != 0
        if (!ok) Log.w(TAG, "gain request $request value $gain rejected by firmware")
        return ok
    }

    /**
     * Sets the LNA (IF) gain. Range: 0-40 dB in 8 dB steps.
     * Affects only RX path. The firmware masks invalid values to the closest step.
     */
    override fun setLnaGain(db: Int) {
        lastLnaGain = HackRfProtocol.lnaGainMasked(db)
        sendGain(HackRfProtocol.REQ_SET_LNA_GAIN, lastLnaGain)
    }

    /**
     * Sets the VGA (baseband) gain. Range: 0-62 dB in 2 dB steps.
     * Affects only RX path. Valid values are strictly enforced by the protocol mask.
     *
     * @return false when the firmware refused the value.
     */
    fun setVgaGain(db: Int): Boolean {
        lastVgaGain = HackRfProtocol.vgaGainMasked(db)
        return sendGain(HackRfProtocol.REQ_SET_VGA_GAIN, lastVgaGain)
    }

    /**
     * Sets the transmit VGA gain. Range: 0-47 dB in 1 dB steps.
     * Governs the TX power prior to the RF amplifier.
     *
     * @return false when the firmware refused the value.
     */
    fun setTxVgaGain(db: Int): Boolean {
        lastTxVgaGain = HackRfProtocol.txVgaGain(db)
        return sendGain(HackRfProtocol.REQ_SET_TXVGA_GAIN, lastTxVgaGain)
    }

    /**
     * RF amplifier, shared by RX and TX: bypassed when off, ~11 dB when on
     * (less at higher frequencies). Not 14 dB — that figure is the MGA-81563's
     * advertised OUTPUT POWER, not its gain; the two are routinely confused.
     */
    fun setAmpEnable(on: Boolean) {
        lastAmpEnable = on
        vendorOut(HackRfProtocol.REQ_AMP_ENABLE, if (on) 1 else 0, 0, null)
    }

    /** Antenna-port bias power (the HackRF equivalent of a bias tee). */
    override fun setAntennaPower(on: Boolean) {
        lastAntennaPower = on
        vendorOut(HackRfProtocol.REQ_ANTENNA_ENABLE, if (on) 1 else 0, 0, null)
    }

    /** Reboot the board. */
    fun reset() {
        if (!supportsApi(HackRfProtocol.API_HW_SYNC)) return
        vendorOut(HackRfProtocol.REQ_RESET, 0, 0, null)
    }

    /** CLKOUT: 10 MHz reference output on the CLKOUT SMA. */
    fun setClkoutEnable(on: Boolean) {
        if (!supportsApi(HackRfProtocol.API_CPLD_CHECKSUM)) return
        vendorOut(HackRfProtocol.REQ_CLKOUT_ENABLE, if (on) 1 else 0, 0, null)
    }

    /** True when an external clock is detected on CLKIN. */
    fun clkinStatus(): Boolean {
        if (!supportsApi(HackRfProtocol.API_CLKIN_STATUS)) return false
        return (vendorIn(HackRfProtocol.REQ_GET_CLKIN_STATUS, 0, 0, 1)
            ?.firstOrNull()?.toInt() ?: 0) != 0
    }

    /** Board hardware revision byte; -1 when the firmware cannot report it. */
    fun boardRev(): Int {
        if (!supportsApi(HackRfProtocol.API_BOARD_REV)) return -1
        return vendorIn(HackRfProtocol.REQ_BOARD_REV_READ, 0, 0, 1)
            ?.firstOrNull()?.toInt()?.and(0xFF) ?: -1
    }

    /** MCU part id + serial number (2×u32 part, 4×u32 serial). */
    fun partIdSerial(): HackRfProtocol.PartIdSerial? =
        vendorIn(HackRfProtocol.REQ_BOARD_PARTID_SERIALNO_READ, 0, 0, 24)
            ?.let { HackRfProtocol.parsePartIdSerial(it) }

    /**
     * Explicit IF/LO tuning with a chosen RF path filter.
     *
     * Ordinary tuning lets the firmware derive the intermediate frequency, the
     * local oscillator and the image-reject filter from the requested
     * frequency. This overrides all three. That matters when the automatic
     * choice puts a mixer spur or an image inside the span being watched:
     * moving the IF, or taking the mixer out of the path with BYPASS, shifts
     * the artefact somewhere harmless without moving the signal.
     */
    fun setFreqExplicit(ifFreqHz: Long, loFreqHz: Long, path: Int): Boolean {
        if (!HackRfProtocol.freqExplicitValid(ifFreqHz, loFreqHz, path)) {
            Log.w(TAG, "rejected explicit tuning if=$ifFreqHz lo=$loFreqHz path=$path")
            return false
        }
        explicitTuning = Triple(ifFreqHz, loFreqHz, path)
        if (!transmitting) applyRxTuning()
        return true
    }

    /** Drop an explicit override and go back to automatic tuning. */
    fun clearFreqExplicit() {
        if (explicitTuning == null) return
        explicitTuning = null
        if (!transmitting) applyRxTuning()
    }

    /** True while an explicit IF/oscillator/filter choice is in force. */
    fun hasExplicitTuning(): Boolean = explicitTuning != null

    /** TX underrun / RX overrun watchdog limits (0 disables). */
    fun setTxUnderrunLimit(v: Int) {
        if (!supportsApi(HackRfProtocol.API_OVERRUN_LIMITS)) return
        vendorOut(HackRfProtocol.REQ_SET_TX_UNDERRUN_LIMIT, v and 0xFFFF, v ushr 16, null)
    }

    fun setRxOverrunLimit(v: Int) {
        if (!supportsApi(HackRfProtocol.API_OVERRUN_LIMITS)) return
        vendorOut(HackRfProtocol.REQ_SET_RX_OVERRUN_LIMIT, v and 0xFFFF, v ushr 16, null)
    }

    /**
     * Per-mode bias-tee policy from plain wire values.
     *
     * The facade the host talks to: it has no reason to know this lib's own
     * types, so the three-flags-per-mode shape is assembled here rather than
     * built by the caller.
     */
    fun setBiasTeeOptions(
        offUpdate: Boolean, offOnEntry: Boolean, offEnabled: Boolean,
        rxUpdate: Boolean, rxOnEntry: Boolean, rxEnabled: Boolean,
        txUpdate: Boolean, txOnEntry: Boolean, txEnabled: Boolean,
    ) = setBiasTOpts(
        HackRfProtocol.BiasTSetting(offUpdate, offOnEntry, offEnabled),
        HackRfProtocol.BiasTSetting(rxUpdate, rxOnEntry, rxEnabled),
        HackRfProtocol.BiasTSetting(txUpdate, txOnEntry, txEnabled),
    )

    /** Per-mode bias-tee policy: what the antenna power does in each mode. */
    fun setBiasTOpts(
        off: HackRfProtocol.BiasTSetting,
        rx: HackRfProtocol.BiasTSetting,
        tx: HackRfProtocol.BiasTSetting,
    ) {
        if (!supportsApi(HackRfProtocol.API_BIAS_T_OPTS)) return
        vendorOut(
            HackRfProtocol.REQ_SET_USER_BIAS_T_OPTS,
            HackRfProtocol.biasTOptsWord(off, rx, tx), 0, null,
        )
    }

    /**
     * Hold the transceiver until the trigger input fires, so several boards
     * start sampling on the same edge. This is the only way to phase-align
     * two receivers.
     */
    fun setHwSyncMode(on: Boolean) {
        if (!supportsApi(HackRfProtocol.API_HW_SYNC)) return
        vendorOut(HackRfProtocol.REQ_SET_HW_SYNC_MODE, if (on) 1 else 0, 0, null)
    }

    /** PortaPack (or other add-on) display: let the board drive its own UI. */
    fun setUiEnable(on: Boolean) {
        if (!supportsApi(HackRfProtocol.API_UI_ENABLE)) return
        vendorOut(HackRfProtocol.REQ_UI_ENABLE, if (on) 1 else 0, 0, null)
    }

    /** Front-panel LEDs; [state] is a mask of LED_USB/LED_RX/LED_TX. */
    fun setLeds(state: Int) {
        if (!supportsApi(HackRfProtocol.API_SET_LEDS)) return
        vendorOut(HackRfProtocol.REQ_SET_LEDS, state and 0xFF, 0, null)
    }

    /** Narrowband front-end filter (newer hardware only). */
    fun setNarrowbandFilter(on: Boolean) {
        if (!supportsApi(HackRfProtocol.API_PRO_SIGNALS)) return
        vendorOut(HackRfProtocol.REQ_SET_NARROWBAND_FILTER, if (on) 1 else 0, 0, null)
    }

    /** Which physical pin the external clock input is taken from. */
    fun setClkinCtrl(signal: Int) {
        if (!supportsApi(HackRfProtocol.API_PRO_SIGNALS)) return
        vendorOut(HackRfProtocol.REQ_CLKIN_CTRL, signal and 0xFF, 0, null)
    }

    /** Expansion header P1 signal routing (trigger in/out, aux clocks). */
    fun setP1Ctrl(signal: Int) {
        if (!supportsApi(HackRfProtocol.API_PRO_SIGNALS)) return
        vendorOut(HackRfProtocol.REQ_P1_CTRL, signal and 0xFF, 0, null)
    }

    /** Expansion header P2 signal routing. */
    fun setP2Ctrl(signal: Int) {
        if (!supportsApi(HackRfProtocol.API_PRO_SIGNALS)) return
        vendorOut(HackRfProtocol.REQ_P2_CTRL, signal and 0xFF, 0, null)
    }

    /** Firmware self-test: pass flag plus the board's own diagnostic text. */
    fun readSelfTest(): HackRfProtocol.SelfTest? {
        if (!supportsApi(HackRfProtocol.API_PRO_SIGNALS)) return null
        return vendorIn(HackRfProtocol.REQ_READ_SELFTEST, 0, 0, HackRfProtocol.SELFTEST_SIZE)
            ?.let { HackRfProtocol.parseSelfTest(it) }
    }

    /**
     * 32 kHz real-time-clock oscillator test: start the oscillator, let it
     * settle, start the frequency monitor, read its verdict, then stop the
     * oscillator again.
     */
    fun testRtcOsc(): Boolean? {
        if (!supportsApi(HackRfProtocol.API_PRO_SIGNALS)) return null
        vendorOut(HackRfProtocol.REQ_TEST_RTC_OSC, 0, HackRfProtocol.RTC_START_OSCILLATOR, null)
        try {
            Thread.sleep(1000)
        } catch (_: InterruptedException) {
            return null
        }
        vendorOut(HackRfProtocol.REQ_TEST_RTC_OSC, 0, HackRfProtocol.RTC_START_FREQ_MONITOR, null)
        try {
            Thread.sleep(2)
        } catch (_: InterruptedException) {
            return null
        }
        val buf = vendorIn(HackRfProtocol.REQ_TEST_RTC_OSC, 0, HackRfProtocol.RTC_READ_FREQ_MONITOR, 2)
            ?: return null
        val pass = HackRfProtocol.rtcOscPassed(HackRfProtocol.leU16(buf))
        if (pass) {
            vendorOut(HackRfProtocol.REQ_TEST_RTC_OSC, 0, HackRfProtocol.RTC_STOP_OSCILLATOR, null)
        }
        return pass
    }

    /** CPLD bitstream CRC, for verifying the board's programmable logic. */
    fun cpldChecksum(): Long {
        if (!supportsApi(HackRfProtocol.API_CPLD_CHECKSUM)) return -1L
        return vendorIn(HackRfProtocol.REQ_CPLD_CHECKSUM, 0, 0, 4)
            ?.let { HackRfProtocol.leU32(it) } ?: -1L
    }

    // ---- Opera Cake antenna switcher ----------------------------------------

    /** Addresses of the attached Opera Cake boards (0xFF = empty slot). */
    fun operacakeBoards(): IntArray {
        if (!supportsApi(HackRfProtocol.API_OPERACAKE_MODE)) return IntArray(0)
        val buf = vendorIn(HackRfProtocol.REQ_OPERACAKE_GET_BOARDS, 0, 0, 8) ?: return IntArray(0)
        return buf.map { it.toInt() and 0xFF }
            .filter { it != 0xFF }
            .toIntArray()
    }

    /**
     * Manual port select. The two ports must sit on opposite sides of the
     * switch; the hardware cannot route both to the same side, so such a
     * request is refused rather than applied by halves.
     */
    fun operacakeSetPorts(address: Int, portA: Int, portB: Int): Boolean {
        if (!supportsApi(HackRfProtocol.API_OPERACAKE_PORTS)) return false
        if (!HackRfProtocol.operacakeValidAddress(address)) return false
        if (!HackRfProtocol.operacakePortsValid(portA, portB)) {
            Log.w(TAG, "rejected Opera Cake ports A=$portA B=$portB (same side)")
            return false
        }
        vendorOut(
            HackRfProtocol.REQ_OPERACAKE_SET_PORTS, address and 0xFF,
            HackRfProtocol.operacakePortsIndex(portA, portB), null,
        )
        return true
    }

    /** Switching mode: manual, by frequency, or by dwell time. */
    fun operacakeSetMode(address: Int, mode: Int): Boolean {
        if (!supportsApi(HackRfProtocol.API_OPERACAKE_MODE)) return false
        if (!HackRfProtocol.operacakeValidAddress(address)) return false
        vendorOut(
            HackRfProtocol.REQ_OPERACAKE_SET_MODE, address and 0xFF, mode and 0xFF, null,
        )
        return true
    }

    /** Current switching mode of the board at [address]; -1 if unknown. */
    fun operacakeGetMode(address: Int): Int {
        if (!supportsApi(HackRfProtocol.API_OPERACAKE_MODE)) return -1
        if (!HackRfProtocol.operacakeValidAddress(address)) return -1
        return vendorIn(HackRfProtocol.REQ_OPERACAKE_GET_MODE, address and 0xFF, 0, 1)
            ?.firstOrNull()?.toInt()?.and(0xFF) ?: -1
    }

    /**
     * Frequency-range table for mode FREQUENCY: (minMHz, maxMHz, port). The
     * switch then follows the VFO by itself, which makes a multi-band antenna
     * setup automatic instead of a manual step per band change.
     */
    fun operacakeSetFreqRanges(ranges: List<Triple<Int, Int, Int>>): Boolean {
        if (!supportsApi(HackRfProtocol.API_OPERACAKE_RANGES)) return false
        if (ranges.isEmpty() || ranges.size > HackRfProtocol.OPERACAKE_MAX_FREQ_RANGES) return false
        vendorOut(
            HackRfProtocol.REQ_OPERACAKE_SET_RANGES, 0, 0,
            HackRfProtocol.operacakeFreqRangesParams(ranges),
        )
        return true
    }

    /** Dwell-time table for mode TIME: (dwell in samples, port). */
    fun operacakeSetDwellTimes(dwells: List<Pair<Int, Int>>): Boolean {
        if (!supportsApi(HackRfProtocol.API_OPERACAKE_DWELL)) return false
        if (dwells.isEmpty() || dwells.size > HackRfProtocol.OPERACAKE_MAX_DWELL_TIMES) return false
        vendorOut(
            HackRfProtocol.REQ_OPERACAKE_SET_DWELL_TIMES, 0, 0,
            HackRfProtocol.operacakeDwellTimesParams(dwells),
        )
        return true
    }

    /** GPIO self-test on an Opera Cake board; -1 when unsupported. */
    fun operacakeGpioTest(address: Int): Int {
        if (!supportsApi(HackRfProtocol.API_OPERACAKE_RANGES)) return -1
        if (!HackRfProtocol.operacakeValidAddress(address)) return -1
        return vendorIn(HackRfProtocol.REQ_OPERACAKE_GPIO_TEST, address and 0xFF, 0, 2)
            ?.let { HackRfProtocol.leU16(it) } ?: -1
    }

    /** Re-send the front-end state after a transceiver-mode change. */
    private fun reassertFrontEnd(forTx: Boolean) {
        if (forTx) {
            sendGain(HackRfProtocol.REQ_SET_TXVGA_GAIN, lastTxVgaGain)
        } else {
            sendGain(HackRfProtocol.REQ_SET_LNA_GAIN, lastLnaGain)
            sendGain(HackRfProtocol.REQ_SET_VGA_GAIN, lastVgaGain)
        }
        vendorOut(HackRfProtocol.REQ_AMP_ENABLE, if (lastAmpEnable) 1 else 0, 0, null)
        vendorOut(
            HackRfProtocol.REQ_ANTENNA_ENABLE, if (lastAntennaPower) 1 else 0, 0, null,
        )
    }

    // ==================== RX ====================

    /** Assert RECEIVE mode and start the streaming thread (idempotent). */
    fun startRx() {
        // A sweep owns the same bulk-IN endpoint. Starting the RX loop
        // alongside it would have two threads splitting one stream: the sweep
        // loses its block marker and the receiver treats sweep headers as IQ,
        // and neither side reports an error.
        if (!running || rxStarted || transmitting || sweeping) return
        rxStarted = true
        reassertFrontEnd(forTx = false)
        // Back to the RECEIVER's tuning, which is not where the last
        // transmission left the oscillator — and which is the explicit
        // override, if one is in force.
        applyRxTuning()
        setTransceiverMode(HackRfProtocol.MODE_RECEIVE)
        rxAssembler.reset()
        lastReportedGaps = -1L
        // Baseline for the firmware's overrun counter: it is cumulative
        // across the board's whole uptime, so only the delta from here
        // belongs to this receive session.
        rxStartShortfalls = m0State()?.numShortfalls ?: -1
        // Two threads, mirroring the transmit path. The USB thread does
        // nothing but keep bulk-IN requests posted; every cycle it would
        // have spent on conversion or an FFT is a window with no posted
        // buffer, which at 20 MSps is a silent overrun inside the firmware.
        rxDspThread = thread(name = "hackrf-rx-dsp") { rxDspLoop() }
        rxUsbThread = thread(name = "hackrf-rx-usb") { rxUsbLoop() }
    }

    private fun stopRx() {
        rxStarted = false
        transport?.cancelRx()
        rxAssembler.wake()
        rxUsbThread?.join(1000)
        rxUsbThread = null
        rxDspThread?.join(1000)
        rxDspThread = null
    }

    /**
     * USB side of the receive stream: keep [HackRfProtocol.RX_REQUESTS_IN_FLIGHT]
     * transfers posted and shovel every completed one into the assembler.
     * Short and odd-length transfers are fed as-is — the assembler carries
     * the residue, so no byte is ever discarded and I/Q parity survives.
     */
    private fun rxUsbLoop() {
        urgentAudioPriority()
        val t = transport ?: return
        t.rxStream(
            HackRfProtocol.RX_TRANSFER_BYTES,
            HackRfProtocol.RX_REQUESTS_IN_FLIGHT,
            { running && rxStarted },
        ) { bytes, n -> rxAssembler.feed(bytes, n) }
    }

    /**
     * DSP side: fixed-size float blocks out of the assembler, spectrum on a
     * throttle, gap telemetry on a slower one. All arrays are reused — the
     * only steady-state allocation is the spectrum the FFT itself returns.
     */
    private fun rxDspLoop() {
        val iq = FloatArray(HackRfProtocol.RX_BLOCK_PAIRS * 2)
        var lastFftMs = 0L
        var lastGapPollMs = 0L
        while (running && rxStarted) {
            if (!rxAssembler.awaitBlock(iq, 100)) continue
            val now = System.currentTimeMillis()
            if (spectrumEnabled && now - lastFftMs >= 80) {
                lastFftMs = now
                val spectrum = fft?.computePowerSpectrum(iq, HackRfProtocol.RX_BLOCK_PAIRS)
                onDataReceived(spectrum ?: EMPTY_SPECTRUM, iq)
            } else {
                onDataReceived(EMPTY_SPECTRUM, iq)
            }
            if (now - lastGapPollMs >= 1000) {
                lastGapPollMs = now
                publishRxGaps()
            }
        }
    }

    /**
     * Fold the firmware's own overrun counter into the host-side drop count
     * and publish the total when it moved. The M0 counter is the only
     * witness to samples lost INSIDE the board (bus stalls, host scheduling)
     * — the stream itself shows nothing at the point of the loss.
     */
    private fun publishRxGaps() {
        val cb = onRxGaps ?: return
        var gaps = rxAssembler.dropEvents
        val base = rxStartShortfalls
        if (base >= 0) {
            val now = m0State()?.numShortfalls
            if (now != null && now >= base) gaps += (now - base).toLong()
        }
        if (gaps != lastReportedGaps) {
            lastReportedGaps = gaps
            cb(gaps)
        }
    }

    // ==================== TX (half-duplex) ====================

    /**
     * Key/unkey the transmitter. Keying stops RX, switches DIRECTLY to
     * TRANSMIT at [HackRfProtocol.TX_BOARD_RATE] and streams the queue;
     * unkeying returns to RECEIVE and restores the RX rate. Direct RX↔TX
     * switches preserve the bias-tee and RF amp, while passing through OFF
     * drops them (firmware idle bank) — so OFF is never used here.
     * Rate/frequency changes apply live in either mode (radio_update loop).
     */
    override fun setPtt(on: Boolean) {
        if (!running || on == transmitting) return
        if (!on && txDraining) return // unkey already in progress
        // Transmitting while the board is sweeping means both a mode fight and
        // two owners of the stream; the sweep has to be stopped first.
        if (on && sweeping) {
            Log.w(TAG, "refusing to key while sweeping")
            return
        }
        if (on) {
            stopRx()
            synchronized(txLock) { txQueue.clear() }
            txFree.clear()
            txReady.clear()
            repeat(HackRfProtocol.TX_BLOCKS_IN_FLIGHT) {
                txFree.offer(ByteArray(HackRfProtocol.TX_BLOCK_BYTES))
            }
            sendSampleRate(HackRfProtocol.TX_BOARD_RATE)
            reassertFrontEnd(forTx = true)
            tuneTo(txFreqHz)
            // Baseline BEFORE keying: the M0 counter is cumulative, and asking
            // for it while the firmware is prefilling its sample buffer is
            // needless traffic on the one path that must not be disturbed.
            // Null on firmware older than USB API 0x0106 — then we simply
            // don't report, rather than guessing.
            txStartShortfalls = m0State()?.numShortfalls ?: -1
            setTransceiverMode(HackRfProtocol.MODE_TRANSMIT)
            transmitting = true
            // Two threads on purpose. Doing both in one loop — write, then
            // interpolate 410k MACs, then write — leaves that compute gap on
            // the wire between transfers, and the board only tolerates
            // 13.65 ms of silence before it starts clocking out zeroes.
            txWriteThread = thread(name = "hackrf-tx-usb") { txWriteLoop() }
            txRenderThread = thread(name = "hackrf-tx-render") { txRenderLoop() }
        } else {
            drainAndStopTx()
            reportShortfalls()
            sendSampleRate(rxSampleRate)
            startRx()
        }
    }

    /**
     * Unkey without a key click. Killing the transmit threads at the instant
     * of PTT release truncated the envelope mid-sample — every over ended in
     * a step, radiated as a wideband transient. Instead: stop accepting new
     * audio, let the renderer drain what remains under a short fade-out ramp
     * ([HackRfProtocol.TX_UNKEY_FADE_PAIRS]), then wait for the board's own
     * consumption counter to catch up with what was sent, so the mode switch
     * happens over a DAC that is genuinely at zero. Every wait is bounded by
     * [HackRfProtocol.TX_UNKEY_TIMEOUT_MS]: a board that stops answering
     * must never leave the transmitter hung keyed.
     */
    private fun drainAndStopTx() {
        txDraining = true
        try {
            val deadline = System.currentTimeMillis() + HackRfProtocol.TX_UNKEY_TIMEOUT_MS
            val renderer = txRenderer
            renderer?.beginUnkey()
            // Phase 1: the render thread keeps producing blocks until the
            // ramp has fully reached zero (it drains the queue underneath).
            while (renderer != null && !renderer.unkeyComplete &&
                running && System.currentTimeMillis() < deadline
            ) {
                sleepQuietly(2)
            }
            // Phase 2: let the USB thread flush the already-rendered tail,
            // then wait for the board to consume its buffered bytes. The M0
            // count is the DAC's own read pointer; when it stops advancing
            // the air is actually silent, not merely our queue.
            while (running && txReady.isNotEmpty() &&
                System.currentTimeMillis() < deadline
            ) {
                sleepQuietly(2)
            }
            var lastCount = -1
            while (running && System.currentTimeMillis() < deadline) {
                val count = m0State()?.m0Count ?: break
                if (count == lastCount) break
                lastCount = count
                sleepQuietly(5)
            }
        } finally {
            stopTxThreads()
            txDraining = false
        }
    }

    private fun sleepQuietly(ms: Long) {
        try {
            Thread.sleep(ms)
        } catch (_: InterruptedException) {
            Thread.currentThread().interrupt()
        }
    }

    private fun stopTxThreads() {
        transmitting = false
        txRenderThread?.join(1000)
        txRenderThread = null
        txWriteThread?.join(1000)
        txWriteThread = null
        txRenderer = null
    }

    override fun isTransmitting(): Boolean = transmitting

    /** Queue interleaved 48 kS/s float IQ for transmission (drop-oldest). */
    override fun submitTxIq(iq: FloatArray) {
        // After PTT release only the tail already queued may go to air; new
        // audio arriving during the drain would extend the transmission past
        // the operator's unkey.
        if (txDraining) return
        synchronized(txLock) { txQueue.write(iq) }   // ring drops oldest itself
    }

    /**
     * Render loop: pull 48 kS/s pairs, polyphase-interpolate to the board rate,
     * quantize to signed 8-bit and hand whole blocks to [txWriteLoop]. Never
     * touches USB, so a slow control transfer can't stall the maths and a slow
     * block of maths can't stall the endpoint.
     *
     * Two things keep the stream continuous where the old single loop could
     * not. First a **cushion**: the board clocks its DAC off its own TCXO
     * while the host produces off the Android audio clock, so the renderer
     * waits for [HackRfProtocol.TX_TARGET_PAIRS] before the first block and
     * then drops or repeats one single pair per block to hold the queue there
     * — asynchronous rate correction, one sample at a time, inaudible. Second
     * a **fade**: if the queue does run dry the last sample is ramped to zero
     * over [HackRfProtocol.TX_FADE_PAIRS] rather than stepped, because a step
     * in the envelope is precisely what sprays splatter across the band.
     */
    private fun txRenderLoop() {
        urgentAudioPriority()
        val renderer = TxBlockRenderer()
        renderer.reset()
        txRenderer = renderer
        awaitPrefill()
        while (running && transmitting) {
            val out = txFree.poll(100, TimeUnit.MILLISECONDS) ?: continue
            // The lock covers the copy out of the ring only — the 410k MACs of
            // interpolation run outside it, so submitTxIq is never blocked by
            // the maths.
            val got = synchronized(txLock) { renderer.drain(txQueue) }
            renderer.render(got, out)
            if (!txReady.offer(out)) txFree.offer(out)
        }
        if (renderer.starvedPairs > 0) {
            Log.w(TAG, "TX queue starved for ${renderer.starvedPairs} pairs")
        }
        Log.i(
            TAG,
            "TX cushion trims=${renderer.trimmedPairs} holds=${renderer.heldPairs}",
        )
    }

    /**
     * Wait for the transmit cushion to build before the first byte leaves.
     * The firmware does the same on its side: it fills the whole 32 KiB sample
     * buffer before enabling baseband streaming. Starting with an empty host
     * queue is what left every other block part silence.
     */
    private fun awaitPrefill() {
        val need = HackRfProtocol.TX_TARGET_PAIRS * 2
        val deadline = System.currentTimeMillis() + 500
        while (running && transmitting && System.currentTimeMillis() < deadline) {
            if (synchronized(txLock) { txQueue.size } >= need) return
            try {
                Thread.sleep(2)
            } catch (_: InterruptedException) {
                return
            }
        }
    }

    /**
     * USB loop: nothing but bulk writes. Blocks are already rendered, so the
     * only thing between one transfer completing and the next being submitted
     * is a queue poll.
     */
    private fun txWriteLoop() {
        urgentAudioPriority()
        val t = transport ?: return
        if (!t.hasTxEndpoint) return
        while (running && transmitting) {
            val block = txReady.poll(100, TimeUnit.MILLISECONDS) ?: continue
            var offset = 0
            while (offset < block.size && running && transmitting) {
                val w = t.bulkWrite(block, offset, block.size - offset, BULK_TIMEOUT_MS)
                if (w < 0) break
                offset += w
            }
            txFree.offer(block)
        }
    }

    /** Read the board's sample-transfer loop state. */
    fun m0State(): HackRfProtocol.M0State? {
        if (!supportsApi(HackRfProtocol.API_M0_STATE)) return null
        return vendorIn(HackRfProtocol.REQ_GET_M0_STATE, 0, 0, HackRfProtocol.M0_STATE_SIZE)
            ?.let { HackRfProtocol.parseM0State(it) }
    }

    /**
     * Log how many transmit shortfalls the board counted over the last
     * transmission. This is the board's own verdict on whether the host kept
     * the DAC fed — a non-zero delta is audible splatter, not a statistic.
     */
    private fun reportShortfalls() {
        val before = txStartShortfalls
        txStartShortfalls = -1
        if (before < 0) return
        val after = m0State()?.numShortfalls ?: return
        val delta = after - before
        if (delta > 0) Log.w(TAG, "TX underruns during last transmission: $delta")
        else Log.i(TAG, "TX clean: no underruns reported by the M0")
    }

    private fun urgentAudioPriority() {
        try {
            android.os.Process.setThreadPriority(
                android.os.Process.THREAD_PRIORITY_URGENT_AUDIO
            )
        } catch (_: Throwable) {
        }
    }

    private fun setTransceiverMode(mode: Int) {
        vendorOut(HackRfProtocol.REQ_SET_TRANSCEIVER_MODE, mode, 0, null)
    }

    // ==================== sweep mode ====================

    @Volatile private var sweeping = false
    private var sweepThread: Thread? = null

    /**
     * Start a firmware-driven spectrum sweep over [startMHz, stopMHz]: the
     * board retunes itself in [stepWidthHz] steps and streams fixed 16 KiB
     * blocks, each tagged with its tuned frequency; [onSweepBlock] receives
     * (lowerEdgeHz, interleaved float IQ) per block. LINEAR style with the
     * LO offset at half the sample rate, so a block covers
     * [lowerEdgeHz, lowerEdgeHz + sample rate]. Mutually exclusive with
     * plain RX and with TX; [stopSweep] restores normal RX.
     */
    fun startSweep(
        startMHz: Int,
        stopMHz: Int,
        sampleRateHz: Int,
        stepWidthHz: Int,
        onSweepBlock: (Long, FloatArray) -> Unit,
    ) {
        if (!running || transmitting || sweeping) return
        stopRx()
        sendSampleRate(sampleRateHz)
        val params = HackRfProtocol.initSweepParams(
            listOf(startMHz to stopMHz),
            stepWidthHz,
            sampleRateHz / 2,
            HackRfProtocol.SWEEP_STYLE_LINEAR,
        )
        // One block per tuning step; INIT_SWEEP must precede the mode switch.
        vendorOut(
            HackRfProtocol.REQ_INIT_SWEEP,
            HackRfProtocol.sweepDwellValue(HackRfProtocol.SWEEP_BLOCK_SIZE),
            HackRfProtocol.sweepDwellIndex(HackRfProtocol.SWEEP_BLOCK_SIZE),
            params,
        )
        setTransceiverMode(HackRfProtocol.MODE_RX_SWEEP)
        reassertFrontEnd(forTx = false)
        sweeping = true
        sweepThread = thread(name = "hackrf-sweep") { sweepLoop(onSweepBlock) }
    }

    /** Leave sweep mode and resume normal RX streaming. */
    fun stopSweep() {
        if (!sweeping) return
        sweeping = false
        sweepThread?.join(1000)
        sweepThread = null
        sendSampleRate(rxSampleRate)
        startRx()
    }

    fun isSweeping(): Boolean = sweeping

    /**
     * Accumulate bulk-IN data into whole 16 KiB sweep blocks (transfers may
     * split at 512-byte packet boundaries), validate the marker and hand the
     * samples over as floats.
     */
    private fun sweepLoop(onSweepBlock: (Long, FloatArray) -> Unit) {
        // Same deal as the RX loop: losing CPU to rendering while blocks
        // arrive back-to-back is an overrun on the board's side.
        urgentAudioPriority()
        val t = transport ?: return
        val block = ByteArray(HackRfProtocol.SWEEP_BLOCK_SIZE)
        var filled = 0
        val chunk = ByteArray(HackRfProtocol.SWEEP_BLOCK_SIZE)
        // One reusable output array: block size is fixed for the whole
        // sweep, so per-block allocation was pure GC pressure on a loop that
        // must keep pace with the bus.
        val pairs = (HackRfProtocol.SWEEP_BLOCK_SIZE - HackRfProtocol.SWEEP_HEADER_SIZE) / 2
        val iq = FloatArray(pairs * 2)
        while (running && sweeping) {
            val n = t.bulkRead(chunk, chunk.size, BULK_TIMEOUT_MS)
            if (n <= 0) continue
            var consumed = 0
            while (consumed < n) {
                val take = minOf(n - consumed, block.size - filled)
                System.arraycopy(chunk, consumed, block, filled, take)
                consumed += take
                filled += take
                if (filled < block.size) continue
                filled = 0
                if (!HackRfProtocol.sweepHeaderPlausible(block, 0)) {
                    // Lost sync: hunt for the next header inside this block.
                    // The 0x7F 0x7F marker alone is NOT a safe re-lock
                    // target — 0x7F is also a positive full-scale sample, so
                    // saturated captures contain the marker in the data
                    // itself and a bare-marker hunt shifts every following
                    // block by a constant error. A candidate only counts
                    // when its full header parses to a frequency the
                    // synthesiser can produce.
                    var sync = 1
                    while (sync <= block.size - HackRfProtocol.SWEEP_HEADER_SIZE &&
                        !HackRfProtocol.sweepHeaderPlausible(block, sync)
                    ) {
                        sync++
                    }
                    if (sync <= block.size - HackRfProtocol.SWEEP_HEADER_SIZE) {
                        System.arraycopy(block, sync, block, 0, block.size - sync)
                        filled = block.size - sync
                    }
                    continue
                }
                val freqHz = HackRfProtocol.sweepBlockFreqHz(block, 0)
                for (i in 0 until pairs * 2) {
                    iq[i] = HackRfProtocol.s8ToFloat(
                        block[HackRfProtocol.SWEEP_HEADER_SIZE + i]
                    )
                }
                onSweepBlock(freqHz, iq)
            }
        }
    }

    // ==================== USB plumbing ====================

    /**
     * Control transfers are serialised. Diagnostics (self-test, oscillator
     * check) run off the session thread by design, so without this two
     * requests could be in flight on endpoint 0 at once and the replies would
     * be attributed to the wrong caller.
     */
    private val controlLock = Any()

    private fun vendorOut(request: Int, value: Int, index: Int, data: ByteArray?) = synchronized(controlLock) {
        val t = transport ?: return
        val r = t.controlTransfer(
            HackRfProtocol.TYPE_VENDOR_OUT, request, value, index, data,
            data?.size ?: 0, CONTROL_TIMEOUT_MS,
        )
        if (r < 0) Log.w(TAG, "vendor request $request failed ($r)")
    }

    /** Generic vendor IN read; returns the received bytes or null on failure. */
    private fun vendorIn(request: Int, value: Int, index: Int, length: Int): ByteArray? = synchronized(controlLock) {
        val t = transport ?: return null
        val buf = ByteArray(length)
        val r = t.controlTransfer(
            HackRfProtocol.TYPE_VENDOR_IN, request, value, index, buf, length,
            CONTROL_TIMEOUT_MS,
        )
        return if (r >= 0) buf.copyOf(r) else null
    }

    // ==================== test seam ====================

    /**
     * Wire a fake transport in place of a USB device, for JVM tests only:
     * everything above the [HackRfTransport] boundary — state guards, gain
     * status handling, stream reassembly, the unkey drain — runs exactly the
     * production code path, minus the hardware.
     */
    internal fun attachTransportForTest(t: HackRfTransport) {
        transport = t
        running = true
        fftBins = HackRfProtocol.spectrumBinsFor(rxSampleRate)
        fft = FFTProcessor(fftBins)
    }

    /** Test visibility into the deferred-state guards. */
    internal fun forceTransmittingForTest(on: Boolean) {
        transmitting = on
    }

    internal fun forceSweepingForTest(on: Boolean) {
        sweeping = on
    }

    private suspend fun awaitPermission(usb: UsbManager, device: UsbDevice): Boolean {
        val appCtx = context ?: return false
        if (usb.hasPermission(device)) return true
        return suspendCancellableCoroutine { cont ->
            val receiver = object : BroadcastReceiver() {
                override fun onReceive(ctx: Context, intent: Intent) {
                    try {
                        appCtx.unregisterReceiver(this)
                    } catch (_: Exception) {
                    }
                    if (cont.isActive) {
                        cont.resume(
                            intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)
                        )
                    }
                }
            }
            androidx.core.content.ContextCompat.registerReceiver(
                appCtx, receiver, IntentFilter(PERMISSION_ACTION),
                androidx.core.content.ContextCompat.RECEIVER_NOT_EXPORTED,
            )
            val flags =
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                    PendingIntent.FLAG_MUTABLE or PendingIntent.FLAG_UPDATE_CURRENT
                } else {
                    PendingIntent.FLAG_UPDATE_CURRENT
                }
            usb.requestPermission(
                device,
                PendingIntent.getBroadcast(
                    appCtx, 0,
                    Intent(PERMISSION_ACTION).setPackage(appCtx.packageName), flags,
                ),
            )
            cont.invokeOnCancellation {
                try {
                    appCtx.unregisterReceiver(receiver)
                } catch (_: Exception) {
                }
            }
        }
    }
}
