/*
 * libhackrfk - Kotlin driver for the HackRF SDR (Android USB host)
 *
 * Thin USB transport seam: the exact set of operations the client performs
 * on the wire, so streaming and control logic can be exercised on the JVM
 * against a fake while the Android implementation owns the UsbRequest
 * machinery.
 *
 * Kotlin port: Copyright (C) 2026 Isak Ruas <isakruas@gmail.com>.
 * All rights reserved.
 *
 * Dual-licensed: GPLv2+ only as distributed within the iSDR Drivers
 * application; all other uses require a separate license from the copyright
 * holder. See LICENSE at the root of this module.
 */
package com.isaklab.libhackrfk

import android.hardware.usb.UsbDeviceConnection
import android.hardware.usb.UsbEndpoint
import android.hardware.usb.UsbRequest
import android.os.Build
import android.util.Log
import java.nio.ByteBuffer
import java.util.concurrent.TimeoutException

/**
 * The wire operations [HackRfClient] needs, and nothing more. One real
 * implementation talks to a [UsbDeviceConnection]; tests substitute a fake
 * that scripts transfer sizes and control-status bytes.
 */
interface HackRfTransport {
    /** Endpoint-0 transfer; returns bytes moved or a negative error. */
    fun controlTransfer(
        requestType: Int, request: Int, value: Int, index: Int,
        data: ByteArray?, length: Int, timeoutMs: Int,
    ): Int

    /** Bulk-OUT write from [data] at [offset]; bytes written or negative. */
    fun bulkWrite(data: ByteArray, offset: Int, length: Int, timeoutMs: Int): Int

    /** Single synchronous bulk-IN read (sweep path); bytes read or negative. */
    fun bulkRead(data: ByteArray, length: Int, timeoutMs: Int): Int

    /** True when the device exposes a bulk-OUT endpoint (TX possible). */
    val hasTxEndpoint: Boolean

    /**
     * Run the receive stream until [keepGoing] returns false or the link
     * fails: keep [requests] bulk-IN transfers of [transferBytes] bytes
     * posted at ALL times and call [onBytes] with each completed transfer's
     * payload (any length, including short and odd — the caller reassembles).
     * The buffer passed to [onBytes] is reused; the callback must copy out
     * before returning. Blocks for the duration of the stream.
     */
    fun rxStream(
        transferBytes: Int,
        requests: Int,
        keepGoing: () -> Boolean,
        onBytes: (ByteArray, Int) -> Unit,
    )

    /** Abort a blocked [rxStream] wait (used at shutdown). */
    fun cancelRx()
}

/**
 * Real transport over Android's USB host API.
 *
 * The receive stream uses asynchronous [UsbRequest]s where the platform can
 * (API 26+): several large transfers stay posted with the host controller
 * simultaneously, and a completed one is re-queued before its data is even
 * converted. A single blocking bulkTransfer leaves the bus idle between the
 * transfer completing and the next call being issued — at 20 MSps that idle
 * window is an overrun inside the firmware, which reports it only as a
 * counter while the stream silently loses samples. On older platforms the
 * async API cannot report short-transfer lengths, so the code falls back to
 * the synchronous loop there rather than guess.
 */
class AndroidUsbTransport(
    private val conn: UsbDeviceConnection,
    private val rxEndpoint: UsbEndpoint,
    private val txEndpoint: UsbEndpoint?,
) : HackRfTransport {
    private companion object {
        const val TAG = "HackRfUsb"
        const val WAIT_TIMEOUT_MS = 250L
        const val SYNC_READ_BYTES = 16 * 1024
        const val SYNC_TIMEOUT_MS = 500
        const val DRAIN_WAIT_MS = 50L
        const val DRAIN_BUDGET_MS = 500L
    }

    @Volatile private var rxCancelled = false
    private val rxRequests = ArrayList<UsbRequest>()

    override fun controlTransfer(
        requestType: Int, request: Int, value: Int, index: Int,
        data: ByteArray?, length: Int, timeoutMs: Int,
    ): Int = conn.controlTransfer(requestType, request, value, index, data, length, timeoutMs)

    override fun bulkWrite(data: ByteArray, offset: Int, length: Int, timeoutMs: Int): Int {
        val ep = txEndpoint ?: return -1
        return conn.bulkTransfer(ep, data, offset, length, timeoutMs)
    }

    override fun bulkRead(data: ByteArray, length: Int, timeoutMs: Int): Int =
        conn.bulkTransfer(rxEndpoint, data, length, timeoutMs)

    override val hasTxEndpoint: Boolean get() = txEndpoint != null

    override fun rxStream(
        transferBytes: Int,
        requests: Int,
        keepGoing: () -> Boolean,
        onBytes: (ByteArray, Int) -> Unit,
    ) {
        rxCancelled = false
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            rxStreamAsync(transferBytes, requests, keepGoing, onBytes)
        } else {
            rxStreamSync(keepGoing, onBytes)
        }
    }

    @androidx.annotation.RequiresApi(Build.VERSION_CODES.O)
    private fun rxStreamAsync(
        transferBytes: Int,
        requests: Int,
        keepGoing: () -> Boolean,
        onBytes: (ByteArray, Int) -> Unit,
    ) {
        synchronized(rxRequests) {
            var failed = false
            repeat(requests) {
                if (failed) return@repeat
                val req = UsbRequest()
                if (!req.initialize(conn, rxEndpoint)) {
                    req.close()
                    failed = true
                    return@repeat
                }
                val buf = ByteBuffer.allocate(transferBytes)
                req.clientData = buf
                if (!req.queue(buf)) {
                    req.close()
                    failed = true
                    return@repeat
                }
                rxRequests.add(req)
            }
            if (failed) {
                rxRequests.forEach {
                    try {
                        it.cancel()
                    } catch (_: Exception) {
                    }
                    it.close()
                }
                rxRequests.clear()
            }
        }
        if (rxRequests.isEmpty()) {
            // The controller refused async posting; a degraded stream is
            // still better than no stream.
            Log.w(TAG, "async bulk-IN unavailable, using synchronous reads")
            rxStreamSync(keepGoing, onBytes)
            return
        }
        try {
            while (keepGoing() && !rxCancelled) {
                val done = try {
                    conn.requestWait(WAIT_TIMEOUT_MS)
                } catch (_: TimeoutException) {
                    continue
                } catch (e: Exception) {
                    if (keepGoing() && !rxCancelled) Log.w(TAG, "requestWait failed", e)
                    break
                } ?: break
                val buf = done.clientData as? ByteBuffer ?: continue
                val n = buf.position()
                if (n > 0) onBytes(buf.array(), n)
                buf.clear()
                // Requeue IMMEDIATELY: the point of the pool is that the
                // controller never runs out of posted buffers.
                if (!done.queue(buf)) {
                    Log.w(TAG, "bulk-IN requeue failed")
                    break
                }
            }
        } finally {
            synchronized(rxRequests) {
                rxRequests.forEach {
                    try {
                        it.cancel()
                    } catch (_: Exception) {
                    }
                }
                // DRAIN before closing: a cancelled or completed request stays
                // in the connection's completion queue until requestWait
                // dequeues it. Closing first leaves a stale native entry that
                // the NEXT rx session's requestWait dequeues — a closed
                // request (endpoint null -> NPE) at best, a freed one
                // (SIGSEGV in libusbhost) at worst. Seen in the field as the
                // drivers process dying right after every transmission.
                var drained = 0
                val deadline = System.currentTimeMillis() + DRAIN_BUDGET_MS
                while (drained < rxRequests.size && System.currentTimeMillis() < deadline) {
                    val done = try {
                        conn.requestWait(DRAIN_WAIT_MS)
                    } catch (_: TimeoutException) {
                        continue
                    } catch (_: Exception) {
                        break
                    } ?: break
                    if (rxRequests.contains(done)) drained++
                }
                rxRequests.forEach { it.close() }
                rxRequests.clear()
            }
        }
    }

    private fun rxStreamSync(keepGoing: () -> Boolean, onBytes: (ByteArray, Int) -> Unit) {
        val buffer = ByteArray(SYNC_READ_BYTES)
        while (keepGoing() && !rxCancelled) {
            val n = conn.bulkTransfer(rxEndpoint, buffer, buffer.size, SYNC_TIMEOUT_MS)
            if (n > 0) onBytes(buffer, n)
        }
    }

    override fun cancelRx() {
        rxCancelled = true
        synchronized(rxRequests) {
            rxRequests.forEach {
                try {
                    it.cancel()
                } catch (_: Exception) {
                }
            }
        }
    }
}
