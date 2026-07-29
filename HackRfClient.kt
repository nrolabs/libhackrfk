/*
 * libhackrfk - Kotlin driver for the HackRF SDR (Android USB host)
 *
 * USB transport, streaming and control lifecycle. The wire format itself
 * lives in the Android-free [HackRfProtocol] codec.
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
import com.isaklab.isdrdrivers.core.FFTProcessor
import com.isaklab.isdrdrivers.core.FloatRing
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.TimeUnit
import kotlin.concurrent.thread
import kotlin.coroutines.resume
import kotlinx.coroutines.suspendCancellableCoroutine

/**
 * HackRF USB client (half-duplex RX + TX). The wire encodings live in the
 * Android-free [HackRfProtocol] (a faithful port of the reference
 * `host/libhackrf/src/hackrf.c`, in-repo at `hackrf/`); this class owns the
 * USB transport: vendor control requests over endpoint 0, signed 8-bit
 * interleaved IQ from bulk-IN 0x81 (RX) and to bulk-OUT 0x02 (TX).
 *
 * Host contract matches the other clients: interleaved float IQ in [-1,1]
 * plus a power spectrum (skippable via [spectrumEnabled]); TX input is
 * 48 kS/s IQ, raised to [HackRfProtocol.TX_BOARD_RATE] by the polyphase
 * [TxInterpolator].
 *
 * The transmit path is modelled on the reference host library's structure
 * rather than its API: libhackrf keeps four USB transfers permanently queued
 * on the bulk-OUT endpoint and renders inside their completion callbacks
 * (`hackrf.c` prepare_transfers / hackrf_libusb_transfer_callback), so the
 * endpoint is never idle while the host computes. Android has no equivalent
 * callback, so the same invariant is bought with two threads — [txRenderLoop]
 * always has the next block ready before [txWriteLoop] asks for it. This is
 * not an optimisation: the board holds only
 * [HackRfProtocol.DEVICE_TX_BUFFER_BYTES] (13.65 ms) while transmitting, and
 * every byte it misses is emitted as a zero by the M0, i.e. as splatter.
 */
class HackRfClient(
    private val context: Context,
    /** (power spectrum in dB, interleaved IQ samples i0,q0,i1,q1,... in [-1,1]) */
    private val onDataReceived: (FloatArray, FloatArray) -> Unit,
    private val onConnectionStatusChanged: (Boolean, String) -> Unit,
) {
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
    @Volatile var spectrumEnabled: Boolean = true

    private var connection: UsbDeviceConnection? = null
    private var iface: UsbInterface? = null
    private var rxEndpoint: UsbEndpoint? = null
    private var txEndpoint: UsbEndpoint? = null
    @Volatile private var running = false
    @Volatile private var rxStarted = false
    @Volatile private var transmitting = false
    private var rxThread: Thread? = null
    private var txRenderThread: Thread? = null
    private var txWriteThread: Thread? = null
    private var fft: FFTProcessor? = null

    /** RX sample rate to restore after a TX-over (half-duplex switch). */
    @Volatile private var rxSampleRate = 4_000_000

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
    // the firmware drops the antenna bias power when returning through IDLE
    // (documented in hackrf.h — it can never stay enabled across modes), and
    // re-sending the gains costs nothing and removes any reset ambiguity.
    @Volatile private var lastLnaGain = 16
    @Volatile private var lastVgaGain = 20
    @Volatile private var lastTxVgaGain = 0
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
     * Claims the USB interface, locates the bulk IN/OUT endpoints, starts the 
     * transceiver, and prepares the client for operation.
     * 
     * Requires USB permission to have been granted. Clears the firmware auto-tx-flush
     * to avoid blocking on subsequent mode switches.
     *
     * @return true if successfully connected and initialized, false otherwise.
     */
    suspend fun connect(): Boolean {
        val usb = context.getSystemService(Context.USB_SERVICE) as UsbManager
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
        rxEndpoint = bulkIn
        txEndpoint = bulkOut
        fft = FFTProcessor(800)
        running = true
        // Clears the firmware's auto-tx-flush so leaving TX is immediate
        // (otherwise mode-off blocks until the transmit buffer fully drains).
        vendorIn(HackRfProtocol.REQ_GET_BUFFER_SIZE, 0, 0, 4)
        val version = vendorIn(HackRfProtocol.REQ_VERSION_STRING_READ, 0, 0, 64)
            ?.toString(Charsets.US_ASCII)?.trimEnd(' ')
        onConnectionStatusChanged(
            true,
            if (version.isNullOrBlank()) "Connected" else "Connected · fw $version",
        )
        return true
    }

    /**
     * Releases USB interfaces, terminates running TX/RX threads, and places the 
     * transceiver in the OFF state. Safe to call idempotently or when partially initialized.
     */
    fun disconnect() {
        running = false
        rxStarted = false
        transmitting = false
        try {
            setTransceiverMode(HackRfProtocol.MODE_OFF)
        } catch (_: Exception) {
        }
        rxThread?.join(1000)
        rxThread = null
        stopTxThreads()
        iface?.let { connection?.releaseInterface(it) }
        connection?.close()
        connection = null
        iface = null
        rxEndpoint = null
        txEndpoint = null
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
    fun setFrequency(hz: Long) {
        rxFreqHz = hz
        if (transmitting) return
        tuneTo(hz)
    }

    /**
     * Retunes the transmitter. The host puts the carrier exactly here, with
     * none of the receiver's zero-IF guard offset — see [rxFreqHz]. While
     * receiving this only records the frequency; [setPtt] applies it on key.
     */
    fun setTxFrequency(hz: Long) {
        txFreqHz = hz
        if (!transmitting) return
        tuneTo(hz)
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
    fun setSampleRate(hz: Int) {
        rxSampleRate = hz
        sendSampleRate(hz)
    }

    private fun sendSampleRate(hz: Int) {
        vendorOut(
            HackRfProtocol.REQ_SAMPLE_RATE_SET, 0, 0,
            HackRfProtocol.sampleRateParams(hz),
        )
        val bw = HackRfProtocol.basebandFilterForSampleRate(hz)
        vendorOut(
            HackRfProtocol.REQ_BASEBAND_FILTER_BW_SET,
            HackRfProtocol.filterBwValue(bw),
            HackRfProtocol.filterBwIndex(bw),
            null,
        )
    }

    /**
     * Sets the LNA (IF) gain. Range: 0-40 dB in 8 dB steps.
     * Affects only RX path. The firmware masks invalid values to the closest step.
     */
    fun setLnaGain(db: Int) {
        lastLnaGain = HackRfProtocol.lnaGainMasked(db)
        vendorInByte(HackRfProtocol.REQ_SET_LNA_GAIN, 0, lastLnaGain)
    }

    /**
     * Sets the VGA (baseband) gain. Range: 0-62 dB in 2 dB steps.
     * Affects only RX path. Valid values are strictly enforced by the protocol mask.
     */
    fun setVgaGain(db: Int) {
        lastVgaGain = HackRfProtocol.vgaGainMasked(db)
        vendorInByte(HackRfProtocol.REQ_SET_VGA_GAIN, 0, lastVgaGain)
    }

    /**
     * Sets the transmit VGA gain. Range: 0-47 dB in 1 dB steps.
     * Governs the TX power prior to the RF amplifier.
     */
    fun setTxVgaGain(db: Int) {
        lastTxVgaGain = HackRfProtocol.txVgaGain(db)
        vendorInByte(HackRfProtocol.REQ_SET_TXVGA_GAIN, 0, lastTxVgaGain)
    }

    /**
     * RF amplifier, shared by RX and TX: bypassed when off, ~11 dB when on
     * (less at higher frequencies). Not 14 dB — that number is the MGA-81563's
     * advertised OUTPUT POWER, not its gain, and the reference documentation
     * calls the confusion out by name in `docs/source/setting_gain.rst`.
     */
    fun setAmpEnable(on: Boolean) {
        lastAmpEnable = on
        vendorOut(HackRfProtocol.REQ_AMP_ENABLE, if (on) 1 else 0, 0, null)
    }

    /** Antenna-port bias power (the HackRF equivalent of a bias tee). */
    fun setAntennaPower(on: Boolean) {
        lastAntennaPower = on
        vendorOut(HackRfProtocol.REQ_ANTENNA_ENABLE, if (on) 1 else 0, 0, null)
    }

    /** Reboot the board (reference hackrf_reset). */
    fun reset() = vendorOut(HackRfProtocol.REQ_RESET, 0, 0, null)

    /** CLKOUT: 10 MHz reference output on the CLKOUT SMA. */
    fun setClkoutEnable(on: Boolean) =
        vendorOut(HackRfProtocol.REQ_CLKOUT_ENABLE, if (on) 1 else 0, 0, null)

    /** True when an external clock is detected on CLKIN. */
    fun clkinStatus(): Boolean =
        (vendorIn(HackRfProtocol.REQ_GET_CLKIN_STATUS, 0, 0, 1)?.get(0)?.toInt() ?: 0) != 0

    /** Board revision byte (reference hackrf_board_rev_read). */
    fun boardRev(): Int =
        vendorIn(HackRfProtocol.REQ_BOARD_REV_READ, 0, 0, 1)?.get(0)?.toInt()?.and(0xFF) ?: -1

    /** MCU part id + serial number (24 bytes: 2×u32 part, 4×u32 serial). */
    fun partIdSerial(): ByteArray? =
        vendorIn(HackRfProtocol.REQ_BOARD_PARTID_SERIALNO_READ, 0, 0, 24)

    /** TX underrun / RX overrun watchdog limits (0 disables). */
    fun setTxUnderrunLimit(v: Int) = vendorOut(
        HackRfProtocol.REQ_SET_TX_UNDERRUN_LIMIT, v and 0xFFFF, v ushr 16, null,
    )
    fun setRxOverrunLimit(v: Int) = vendorOut(
        HackRfProtocol.REQ_SET_RX_OVERRUN_LIMIT, v and 0xFFFF, v ushr 16, null,
    )

    /** Per-mode bias-tee policy (reference hackrf_set_user_bias_t_opts). */
    fun setBiasTOpts(
        offUpdate: Boolean, offEnabled: Boolean,
        rxUpdate: Boolean, rxEnabled: Boolean,
        txUpdate: Boolean, txEnabled: Boolean,
    ) = vendorOut(
        HackRfProtocol.REQ_SET_USER_BIAS_T_OPTS,
        HackRfProtocol.biasTOptsWord(offUpdate, offEnabled, rxUpdate, rxEnabled, txUpdate, txEnabled),
        0, null,
    )

    /** Opera Cake antenna switcher: 8-byte board-address list (0xFF = none). */
    fun operacakeBoards(): ByteArray? =
        vendorIn(HackRfProtocol.REQ_OPERACAKE_GET_BOARDS, 0, 0, 8)

    /** Opera Cake manual port select: A0 side 0..3 (PA1-4), B0 side 4..7. */
    fun operacakeSetPorts(address: Int, portA: Int, portB: Int) = vendorOut(
        HackRfProtocol.REQ_OPERACAKE_SET_PORTS, address and 0xFF,
        (portA and 0xFF) or ((portB and 0xFF) shl 8), null,
    )

    /** Opera Cake switching mode: 0 manual, 1 frequency, 2 time. */
    fun operacakeSetMode(address: Int, mode: Int) = vendorOut(
        HackRfProtocol.REQ_OPERACAKE_SET_MODE, address and 0xFF, mode and 0xFF, null,
    )

    /** Re-send the front-end state after a transceiver-mode change. */
    private fun reassertFrontEnd(forTx: Boolean) {
        if (forTx) {
            vendorInByte(HackRfProtocol.REQ_SET_TXVGA_GAIN, 0, lastTxVgaGain)
        } else {
            vendorInByte(HackRfProtocol.REQ_SET_LNA_GAIN, 0, lastLnaGain)
            vendorInByte(HackRfProtocol.REQ_SET_VGA_GAIN, 0, lastVgaGain)
        }
        vendorOut(HackRfProtocol.REQ_AMP_ENABLE, if (lastAmpEnable) 1 else 0, 0, null)
        vendorOut(
            HackRfProtocol.REQ_ANTENNA_ENABLE, if (lastAntennaPower) 1 else 0, 0, null,
        )
    }

    // ==================== RX ====================

    /** Assert RECEIVE mode and start the streaming thread (idempotent). */
    fun startRx() {
        if (!running || rxStarted || transmitting) return
        rxStarted = true
        reassertFrontEnd(forTx = false)
        // Back to the RECEIVER's frequency, which is not where the last
        // transmission left the LO.
        tuneTo(rxFreqHz)
        setTransceiverMode(HackRfProtocol.MODE_RECEIVE)
        rxThread = thread(name = "hackrf-rx") { rxLoop() }
    }

    private fun stopRx() {
        rxStarted = false
        rxThread?.join(1000)
        rxThread = null
    }

    private fun rxLoop() {
        // Audio priority: block delivery must not lose CPU to rendering.
        urgentAudioPriority()
        val ep = rxEndpoint ?: return
        val buffer = ByteArray(16 * 1024) // reduced from 128KB to prevent JNI GC lock stalls
        var lastFftMs = 0L
        while (running && rxStarted) {
            val conn = connection ?: break
            val n = conn.bulkTransfer(ep, buffer, buffer.size, BULK_TIMEOUT_MS)
            if (n <= 0) continue
            val pairs = n / 2
            if (pairs < 256) continue
            /* signed 8-bit interleaved IQ → interleaved floats in [-1, 1] */
            val iq = FloatArray(pairs * 2)
            for (i in 0 until pairs * 2) {
                iq[i] = HackRfProtocol.s8ToFloat(buffer[i])
            }
            if (!spectrumEnabled) {
                onDataReceived(EMPTY_SPECTRUM, iq)
                continue
            }
            val now = System.currentTimeMillis()
            if (now - lastFftMs >= 80) {
                lastFftMs = now
                val spectrum = fft?.computePowerSpectrum(iq, pairs)
                if (spectrum != null) {
                    onDataReceived(spectrum, iq)
                    continue
                }
            }
            onDataReceived(EMPTY_SPECTRUM, iq)
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
    fun setPtt(on: Boolean) {
        if (!running || on == transmitting) return
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
            // Two threads on purpose. The reference host library keeps four
            // USB transfers permanently queued on the endpoint and renders in
            // its completion callback (hackrf.c prepare_transfers /
            // hackrf_libusb_transfer_callback), so the endpoint is never idle
            // while the host computes. Doing both in one loop — write, then
            // interpolate 410k MACs, then write — leaves exactly that compute
            // gap on the wire, and the board only tolerates 13.65 ms of it.
            txWriteThread = thread(name = "hackrf-tx-usb") { txWriteLoop() }
            txRenderThread = thread(name = "hackrf-tx-render") { txRenderLoop() }
        } else {
            stopTxThreads()
            reportShortfalls()
            sendSampleRate(rxSampleRate)
            startRx()
        }
    }

    private fun stopTxThreads() {
        transmitting = false
        txRenderThread?.join(1000)
        txRenderThread = null
        txWriteThread?.join(1000)
        txWriteThread = null
    }

    fun isTransmitting(): Boolean = transmitting

    /** Queue interleaved 48 kS/s float IQ for transmission (drop-oldest). */
    fun submitTxIq(iq: FloatArray) {
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
     * The firmware does exactly this on its side — `tx_mode` fills the whole
     * 32 KiB sample buffer over bulk-OUT and only then calls
     * `baseband_streaming_enable` (usb_api_transceiver.c) — and starting with
     * an empty host queue is what left every other block part silence.
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
        val ep = txEndpoint ?: return
        while (running && transmitting) {
            val block = txReady.poll(100, TimeUnit.MILLISECONDS) ?: continue
            val conn = connection
            if (conn == null) {
                txFree.offer(block)
                break
            }
            var offset = 0
            while (offset < block.size && running && transmitting) {
                val w = conn.bulkTransfer(
                    ep, block, offset, block.size - offset, BULK_TIMEOUT_MS,
                )
                if (w < 0) break
                offset += w
            }
            txFree.offer(block)
        }
    }

    /** Read the M0 SGPIO loop state (reference hackrf_get_m0_state). */
    fun m0State(): HackRfProtocol.M0State? =
        vendorIn(HackRfProtocol.REQ_GET_M0_STATE, 0, 0, HackRfProtocol.M0_STATE_SIZE)
            ?.let { HackRfProtocol.parseM0State(it) }

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
        val ep = rxEndpoint ?: return
        val block = ByteArray(HackRfProtocol.SWEEP_BLOCK_SIZE)
        var filled = 0
        val chunk = ByteArray(HackRfProtocol.SWEEP_BLOCK_SIZE)
        while (running && sweeping) {
            val conn = connection ?: break
            val n = conn.bulkTransfer(ep, chunk, chunk.size, BULK_TIMEOUT_MS)
            if (n <= 0) continue
            var consumed = 0
            while (consumed < n) {
                val take = minOf(n - consumed, block.size - filled)
                System.arraycopy(chunk, consumed, block, filled, take)
                consumed += take
                filled += take
                if (filled < block.size) continue
                filled = 0
                if (!HackRfProtocol.isSweepBlock(block, 0)) {
                    // Lost sync: hunt for the next marker inside this block.
                    var sync = 1
                    while (sync < block.size - 1 &&
                        !(block[sync] == HackRfProtocol.SWEEP_MAGIC_0 &&
                            block[sync + 1] == HackRfProtocol.SWEEP_MAGIC_1)
                    ) {
                        sync++
                    }
                    if (sync < block.size - 1) {
                        System.arraycopy(block, sync, block, 0, block.size - sync)
                        filled = block.size - sync
                    }
                    continue
                }
                val freqHz = HackRfProtocol.sweepBlockFreqHz(block, 0)
                val pairs = (block.size - HackRfProtocol.SWEEP_HEADER_SIZE) / 2
                val iq = FloatArray(pairs * 2)
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

    private fun vendorOut(request: Int, value: Int, index: Int, data: ByteArray?) {
        val conn = connection ?: return
        val r = conn.controlTransfer(
            HackRfProtocol.TYPE_VENDOR_OUT, request, value, index, data,
            data?.size ?: 0, CONTROL_TIMEOUT_MS,
        )
        if (r < 0) Log.w(TAG, "vendor request $request failed ($r)")
    }

    /** Generic vendor IN read; returns the received bytes or null on failure. */
    private fun vendorIn(request: Int, value: Int, index: Int, length: Int): ByteArray? {
        val conn = connection ?: return null
        val buf = ByteArray(length)
        val r = conn.controlTransfer(
            HackRfProtocol.TYPE_VENDOR_IN, request, value, index, buf, length,
            CONTROL_TIMEOUT_MS,
        )
        return if (r >= 0) buf.copyOf(r) else null
    }

    private fun vendorInByte(request: Int, value: Int, index: Int) {
        val conn = connection ?: return
        val retval = ByteArray(1)
        val r = conn.controlTransfer(
            HackRfProtocol.TYPE_VENDOR_IN, request, value, index, retval, 1,
            CONTROL_TIMEOUT_MS,
        )
        if (r != 1 || retval[0].toInt() == 0) {
            Log.w(TAG, "vendor request $request rejected (r=$r)")
        }
    }

    private suspend fun awaitPermission(usb: UsbManager, device: UsbDevice): Boolean {
        if (usb.hasPermission(device)) return true
        return suspendCancellableCoroutine { cont ->
            val receiver = object : BroadcastReceiver() {
                override fun onReceive(ctx: Context, intent: Intent) {
                    try {
                        context.unregisterReceiver(this)
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
                context, receiver, IntentFilter(PERMISSION_ACTION),
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
                    context, 0,
                    Intent(PERMISSION_ACTION).setPackage(context.packageName), flags,
                ),
            )
            cont.invokeOnCancellation {
                try {
                    context.unregisterReceiver(receiver)
                } catch (_: Exception) {
                }
            }
        }
    }
}
