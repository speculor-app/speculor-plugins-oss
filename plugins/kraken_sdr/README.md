# kraken_sdr — KrakenSDR 5-channel coherent source

Streams five coherent I/Q channels from a [KrakenSDR](https://www.krakenrf.com/)
(KrakenRF) — a 5-channel coherent RTL-SDR array (5× R820T2 + RTL2832U sharing one
28.8 MHz TCXO). It opens the five tuners directly (no external daemon), reusing the
`rtl_sdr` device layer, and emits one `SPC_DATA_SIGNAL` port per channel.

`Signal/SDR/Passive Radar/Sources` · data-source plugin · maturity **PREVIEW**.

## Coherence scope — read this first

The shared clock gives **frequency coherence only**. It does **not** give sample or
phase alignment:

- the five USB streams start at unknown times → a **fixed integer-sample offset**
  between channels;
- each R820T2 PLL locks at a **random phase** per tune, and drifts slowly.

This plugin deliberately does **not** perform the noise-source cross-correlation
sample-sync + eigen-decomposition phase/amplitude calibration that KrakenRF's
Heimdall DAQ does. What it guarantees is *clean, frequency-coherent capture*:

- all five tuners on the same center frequency, sample rate, gain and PPM;
- **dithering forced off** (dithering breaks phase coherence);
- the five async reads started back-to-back (near-simultaneous);
- **lock-stepped, equal-size blocks** — a block is emitted only when *every* channel
  has a full batch buffered, and all five ports carry the **same `frame_number` and
  `timestamp_ns`** so a downstream node can pair samples of the same block.

### Why this is fine for passive radar

A passive-radar range-Doppler correlator cross-correlates a **reference** channel
against **surveillance** channels and self-calibrates the range origin from the
direct-path peak; a constant per-channel phase offset simply rides along on the
correlation peak. So this driver is *PR-ready capture*. **What it does not include is
the range-Doppler correlator itself** — that is a separate downstream node. For
direction finding you additionally need a phase-calibration node (also separate).

**Channel 0 = serial 1000 = the reference / control dongle** (it owns the noise-source
and bias-tee GPIO). Wire your reference antenna to that SMA port.

## Parameters

All tuning/gain parameters are applied **identically to all five tuners** (the Kraken
uses one universal gain).

| Parameter | Type | Default | Notes |
|---|---|---|---|
| `center_freq` | int, Hz | 100 MHz | 24 MHz – 1.766 GHz. FM-band default suits PR illuminators. |
| `sample_rate` | int, Hz | 2 400 000 | 2.4 MSPS is the Kraken recommended max. |
| `bandwidth` | int, Hz | 0 (auto) | IF bandwidth; 0 = matched to sample rate. |
| `agc_enabled` | bool | false | Leave off for coherent capture (manual gain). |
| `mgc_gain` | float, dB | 30 | Universal manual gain, snapped to the nearest hardware step (active when AGC is off). |
| `cal_gain_db` | float, dB | −1 (auto) | Gain applied to all five tuners **while the noise source is on**, restored when it turns off. The burst replaces the antennas, so the operating gain — set for weak off-air signals — rails the 8-bit ADC (measured 34.8 % clipped samples) and degrades the calibrator's phase estimate. −1 = automatic per-band value from heimdall's calibration gain table (8.7 dB at 100 MHz … 49.6 dB at 700 MHz+). |
| `freq_correction` | int, PPM | 0 | Correction for the shared TCXO. |
| `dithering` | bool | false | R820T2 PLL sigma-delta dithering. **Leave off for a coherent array**: with it on, each tuner's phase drifts independently and the array decorrelates itself (A/B-verified on hardware: off locks at 0.85+, on drops one channel to 0.02). Applied before tuning — librtlsdr only latches it into the next PLL programming. R820T-only; a V4's R828D rejects the call. |
| `noise_source` | bool | false | Toggle the onboard calibration noise source (control-dongle **GPIO 0**) — a common tone into all channels, for a calibration node. |
| `bias_tee_ch0..4` | bool ×5 | false | 4.5 V bias-tee per SMA port (control-dongle **GPIO 1..5**). **PREVIEW — verify routing.** |
| `relax_serial_match` | bool | false | If serials 1000–1004 aren't found, use the first 5 RTL devices (reflashed units; reference channel not guaranteed). |

Single-tuner debug knobs (`direct_sampling`, `offset_tuning`, `if_gain`, `test_mode`)
are intentionally omitted — they are per-tuner HF/debug features irrelevant to a
coherent VHF/UHF array.

Changing `sample_rate` or `bandwidth` while streaming logs a warning: a rate change
soft-resets each channel's demodulator at a different instant, and a bandwidth change
re-tunes each PLL (the filter moves the IF) at a new random phase — either way the
array's alignment/phase calibration is stale until the calibrator's next pass. A
`center_freq` change is detected by the calibrator itself (the new frequency arrives
in the block metadata) and forces a recalibration automatically.

## Outputs

Five signal ports `iq_ch0 … iq_ch4`, each `{i:int16, q:int16}` (interleaved I/Q, one
record per complex sample). Per-block metadata carries `sample_rate_hz`,
`center_freq_hz`, `bandwidth_hz`, `gain_db`, `agc_enabled`, `bit_depth` (8, RTL
native), and the shared `frame_number` / `timestamp_ns`.

## Device detection

The plugin detects a single Kraken by matching the five RTL-SDR serial strings
`"1000".."1004"` and mapping **channel k → serial 1000+k**. If not all five are
present it produces nothing and logs the shortfall; a **Scan** re-runs detection.
`relax_serial_match` is a best-effort fallback for reflashed units (uses the first
five enumerated devices, in which case the reference-channel mapping is not
guaranteed). Multiple Krakens on one host are unsupported (their serials collide).

## GPIO map (verified)

All GPIO is driven on the **channel-0 control dongle** via `rtlsdr_set_bias_tee_gpio`:
noise source on GPIO 0, per-channel bias tees on GPIO 1–5. This matches KrakenRF's own
DAQ firmware (heimdall `rtl_daq.c` drives the noise source as GPIO 0 and channel-m
bias tees as GPIO m+1, all on the control device), and the noise-source path is
hardware-verified — the calibrator locks through it. The per-port bias-tee routing
remains PREVIEW: confirm each `bias_tee_chN` energizes the right SMA port.

Bias tees are re-applied from parameters at every **Start** (not explicitly cleared at
Stop; Stop clears each dongle's own GPIO 0 as part of the guarded teardown).

## Coherent capture notes / limits

- **No sample/phase calibration** (by design). The fixed startup integer-sample offset
  is recovered downstream (a PR correlator does this from the direct-path peak).
- **USB / power.** Five channels at 2.4 MSPS ≈ 24 MB/s over the Kraken's internal USB
  2.0 hub (fits), but the unit needs a solid **5 V / 2.4 A+** supply; under-power or a
  weak hub causes dropped samples. Lower `sample_rate` if you see drops.
- **Overflow is loud, because it must be.** If the pipeline stalls long enough for a
  channel's ring to fill (~0.9 s at 2.4 MSPS), samples drop — and each channel drops a
  *different* amount, which silently breaks the fixed inter-channel alignment. The
  plugin counts refused samples per channel and logs an ERROR when it happens; the
  calibrator's next recalibration measures the shifted delays and re-aligns.
- **Read-back verification.** After configuring, each tuner is asked what it actually
  did (`rtlsdr_get_center_freq`/`_get_sample_rate`) — a channel that silently refused
  its frequency is otherwise indistinguishable from one with a dead antenna. Note the
  library reports the *requested* frequency back; the PLL's actual grid snap (up to
  ~±220 Hz, identical across all five tuners, cancelling in any cross-correlation) is
  not visible through this API.
- Opening the array is all-or-nothing: if any of the five channels fails to open,
  `start()` fails rather than streaming a partial (useless) array.

## Passive-radar pipeline

```
kraken_sdr ──iq_ch0 (ref)──┐
           ──iq_ch1..4─────┴──► range_doppler ──► 2D CFAR / detection ──► renderer
```

`range_doppler` (the PR correlator) is a separate plugin; `kraken_sdr`'s aligned
five-port output is its input. A Heimdall-like `coherent_calibrator` node is only
needed for direction finding, not passive radar.

## Licensing

Apache-2.0 plugin code; loads **librtlsdr (LGPL-2.1+)** at runtime (same as `rtl_sdr`).
See `LICENSE` and the repository-root `THIRD_PARTY_NOTICES.md`.
