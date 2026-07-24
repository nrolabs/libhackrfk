# libhackrfk

Kotlin driver for the **HackRF** SDR (HackRF One, Jawbreaker, rad1o), speaking
the **libhackrf** USB host protocol over the Android USB host API. Pure Kotlin
— no NDK, no native libraries, no root. Built for Android but the wire codec
is Android-free and JVM-testable.

Maintained by Isak — **PU3IAR**. Brought to you by [id.qsl.br](https://id.qsl.br),
a platform with tools for amateur radio operators.
YouTube: [@qraisak](https://www.youtube.com/@qraisak)

## Features

- **`HackRfProtocol`** — the libhackrf wire codec, free of any Android
  dependency and unit-tested on the JVM:
  - Vendor **request ids** and payload encodings: frequency (MHz+Hz
    little-endian split), fractional **sample rate** (2–20 Msps) with the
    matching **MAX2837 baseband filter** selection, transceiver modes.
  - **Gain rules** exactly as the reference: LNA 0–40 dB in 8 dB steps,
    VGA 0–62 dB in 2 dB steps, TXVGA 0–47 dB, drive-scale mapping.
  - Signed 8-bit IQ **sample conversion** and the **sweep** block format
    (0x7F7F marker + u64 LE tuned frequency).
- **`HackRfClient`** — USB transport and streaming lifecycle:
  - Device identity (VID `0x1d50`), permission handling, **RX** streaming
    from the bulk-IN endpoint delivered as `FloatArray` (`i0,q0,…` in
    `[-1,1]`) with an optional power spectrum, through the same callback
    contract as the other iSDR clients.
  - Half-duplex **TX**: direct RX↔TX mode switches (never through OFF, which
    would drop the bias tee and RF amp), front-end state re-asserted across
    switches, and the transmit queue streamed to the bulk-OUT endpoint.
  - Firmware-driven **sweep** for wideband spectrum scans, with block
    reassembly and marker resynchronization.
  - `setFrequency` / `setSampleRate` / `setLnaGain` / `setVgaGain` /
    `setAmpEnable` / `setAntennaPower`, and TX: `setTxVgaGain` / `setPtt` /
    `submitTxIq`; sweep: `startSweep` / `stopSweep`.
- **`TxInterpolator`** — polyphase Blackman-sinc interpolation raising the
  48 kS/s modulator IQ to the board rate with unit DC gain per branch.

## Testing without hardware

The wire codec and the interpolator are pinned by JVM contract tests in the
parent project ([`isdr`](https://github.com/nrolabs/isdr-app):
`HackRfProtocolTest`, `TxInterpolatorTest`) — byte-exact payloads, filter and
gain rules, sweep block parsing, and the measured spectral contract of the TX
interpolation, all validated against the reference sources.

## Credits and references

Faithful port of the reference host library and firmware semantics:

- **HackRF / libhackrf** — Great Scott Gadgets and contributors:
  <https://github.com/greatscottgadgets/hackrf>
- `libhackrf` is **BSD 3-clause**; its copyright notice is retained in the
  file headers and in [`COPYING.md`](COPYING.md).

Request semantics, mode-switch behaviour and streaming alignment follow the
`hackrf_usb` firmware sources and the `libhackrf` host library.

## License

Dual-licensed — GPLv2+ only as part of the iSDR Drivers application; all
other rights reserved. See [`LICENSE`](LICENSE). The upstream `libhackrf`
BSD 3-clause notice is retained there and in [`COPYING.md`](COPYING.md).
Kotlin port © 2026 Isak Ruas <isakruas@gmail.com>.
