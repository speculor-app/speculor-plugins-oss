# nesdr — NooElec NESDR I/Q source

Streams interleaved I/Q from a NooElec NESDR receiver into the pipeline as a
`SPC_DATA_SIGNAL` table, and gates every hardware control on what the selected
model's board actually provides.

> ## ⚠ Experimental — never run against a NESDR
>
> **No part of this plugin has been tested with real hardware.** Not one model,
> not the R820T2 bodies, not the E4000 ones. It has never opened a dongle,
> tuned a PLL, or produced a sample from silicon.
>
> What *is* verified is the software contract: the plugin loads, is ABI-honest,
> survives the no-device lifecycle paths, and its capability table gates the
> right controls for the right models (three automated tests, all hardware-free
> — see [Testing](#testing)). The streaming path is inherited essentially
> unchanged from `rtl_sdr`, which *is* hardware-validated, but "inherited from
> working code" is an argument, not a test.
>
> Everything below marked **assumed** comes from NooElec's published product
> pages or from reading librtlsdr, not from measurement. The descriptor declares
> `SPC_MATURITY_EXPERIMENTAL` accordingly. Treat first contact with a real
> dongle as a bring-up exercise, and see
> [First hardware bring-up](#first-hardware-bring-up) for what to check.
>
> `rtl_sdr` remains the tested path for any RTL2832U dongle, including every
> NESDR, if this one misbehaves.

| | |
|---|---|
| **Plugin id** | `nesdr` |
| **Category** | `Signal/SDR/Sources` |
| **Maturity** | `SPC_MATURITY_EXPERIMENTAL` — untested against hardware |
| **Output** | `iq_out` — `SPC_DATA_SIGNAL`, fields `i`/`q` (`INT16`) |
| **Inputs** | none; the implicit control input is port 0 |
| **Flags** | `data_source`, `streaming`, `device_scan` |
| **Runtime dep** | `librtlsdr`, loaded via `LoadLibrary`/`dlopen` |

## Why this exists alongside `rtl_sdr`

Every NESDR is an RTL2832U dongle, so `rtl_sdr` enumerates and streams from one
today. What it cannot do is tell the models apart — and four of its assumptions
are wrong on NooElec boards, three of them in ways where the ungated call does
something actively harmful rather than nothing:

- **`offset_tuning` switches the bias tee.** On an R820T/R828D, librtlsdr routes
  `rtlsdr_set_offset_tuning()` straight to `rtlsdr_set_bias_tee()` and returns
  −2 (`src/librtlsdr.c`). Since almost every NESDR is R820T2, ticking "Offset
  Tuning" puts voltage on the antenna port. Real offset tuning exists only on
  the E4000, so this plugin enables the control on XTR bodies and nowhere else.
- **Direct sampling below 24 MHz is forced on.** `rtl_sdr` treats "not an
  RTL-SDR Blog V4" as "V3, so auto-enable the Q-ADC for HF". Of the NESDR line
  only the SMArt v5 has a wired Q-branch; on the rest that tunes the receiver to
  a pin connected to nothing *and* re-initialises the tuner on the way. Every
  other model needs an upconverter, so the control stays disabled there.
- **The bias tee is not a GPIO on a SMArTee.** SMArTee bodies carry a
  hardware-permanent 4.5 V supply that no software call can switch; Mini, Nano,
  SMArt and XTR bodies have no bias-tee circuit at all. Only the generic
  profiles expose a switchable one.
- **The E4000 is a different tuner.** XTR models reach 2300 MHz (with a
  device-specific L-band gap), span −1.0…42.0 dB across 14 gain steps rather
  than the R820T2's 0…49.6 dB across 29, and have six IF gain stages —
  `rtl_sdr` caps out at 1.766 GHz and hardcodes stage 1.

`rtl_sdr` is deliberately left untouched: it stays the validated path for
RTL-SDR Blog V3/V4 hardware, and the escape hatch if a profile here misbehaves.
Both plugins compile the same `rtl_sdr_device.{h,cpp}`, so the hard-won
librtlsdr handling (heap-corruption mitigation, bounded two-phase stop,
abandon-on-fault) lives in one place.

## Model resolution

A NESDR appears in **both** plugins' device lists — only one process can open
it, so pick one. This plugin lists every RTL2832U device rather than filtering
to NooElec: Mini and Nano bodies frequently ship with stock Realtek EEPROM
strings, and a strict filter would show those owners an empty list.

The profile is resolved in three steps:

1. **At scan**, from the USB manufacturer/product strings — all that is
   available before the device is opened. A match names the device in the list
   (`NESDR Nano 3 [00000001]`); no match leaves it as reported
   (`Generic RTL2832U OEM [00000001]`), still selectable.
2. **At open**, `rtlsdr_get_tuner_type()` confirms the tuner family. This reads
   the silicon rather than a string `rtl_eeprom` can rewrite, so on a
   disagreement the tuner wins and the conflict is logged — unless you pinned a
   model by hand, which is honoured with a warning.
3. **The `model` parameter** pins a profile outright. Use it for a dongle with
   generic EEPROM strings.

A bare `NESDR SMArt` string resolves to the **v4** profile, not the v5: the
string cannot say which, and guessing v5 would re-introduce exactly the
forced-HF-direct-sampling bug above. v5 owners should pin the model.

The resolved profile is logged once at start:

```
NESDR: NESDR Nano 3 (tuner R820T2, profile nano3, bias tee none,
       direct sampling not wired, 0.5 PPM TCXO)
```

## Profiles

| Model | Tuner | Range | Direct sampling | Bias tee | TCXO |
|---|---|---|---|---|---|
| Generic RTL2832U (R820T2) | R820T/R820T2 | 24 M–1766 MHz | control available | switchable | — |
| Generic RTL2832U (E4000) | E4000 | 52 M–2200 MHz † | — | switchable | — |
| NESDR Mini | R820T | 25 M–1750 MHz | — | none | — |
| NESDR Mini 2 / 2+ | R820T2 | 25 M–1750 MHz | — | none | 2+ only |
| NESDR Nano 2 / 2+ | R820T2 | 25 M–1750 MHz | — | none | 2+ only |
| NESDR Nano 3 | R820T2 | 25 M–1700 MHz | — | none | 0.5 PPM |
| NESDR SMArt v4 | R820T2 | 25 M–1750 MHz | — | none | 0.5 PPM |
| NESDR SMArt v5 | R820T2/R860 | **100 kHz**–1750 MHz | **Q-branch wired** | none | 0.5 PPM |
| NESDR SMArTee v2 | R820T2 | 25 M–1750 MHz | — | **always-on 4.5 V** | 0.5 PPM |
| NESDR SMArt XTR | E4000 | 55 M–2300 MHz † | — | none | 0.5 PPM |
| NESDR SMArTee XTR | E4000 | 55 M–2300 MHz † | — | **always-on 4.5 V** | 0.5 PPM |
| NESDR XTR / XTR+ | E4000 | 65 M–2300 MHz † | — | none | XTR+ only |

† E4000 bodies have a device-specific L-band gap around 1100–1250 MHz where the
PLL will not lock. Its exact edges vary between individual tuners, so a request
inside it is reported rather than corrected — the I/Q metadata shows where the
tuner actually landed. This is also why NooElec does not recommend XTR models
for ADS-B.

**The `center_freq` parameter tops out at 2.147 GHz, not 2.3 GHz.**
`SPC_PARAM_INT` is an `int32_t`, so a frequency in Hz cannot go higher.
Widening it would mean leaving the `center_freq INT Hz` contract in
`sdr_params.h`, which is what lets a generic controller (`sdr_control`,
`radio_tuner`) drive any SDR source interchangeably — not a trade worth making
for the top 150 MHz of one tuner family.

## Parameters

Standard `sdr_params.h` names throughout, so a saved pipeline can swap this
source for any other SDR source, and `sdr_control` drives it over the control
port with no special-casing.

**Tuning** — `center_freq`, `sample_rate`, `bandwidth`.
**Gain** — `agc_enabled`, `mgc_gain` (dB, snapped to the nearest hardware step).
**Hardware** — `model`, `direct_sampling`, `offset_tuning`, `bias_tee`,
`freq_correction`, `dithering`, `test_mode`, `if_gain_stage`, `if_gain`.

A control the resolved board does not have is reported `SPC_PARAM_FLAG_DISABLED`
by `get_parameter`, and `set_parameter` answers `SPC_ERR_NOT_FOUND` **without
storing the value** — answering "unsupported" while quietly keeping it would
hand the next `start()` a setting the caller was told was refused.

`center_freq` is clamped to the profile's range when applied, with a warning
logged once per distinct request rather than once per retune. The requested
value is what `get_parameter` returns; the **I/Q table metadata carries the
truth**, stamped from the hardware's own `rtlsdr_get_center_freq()` /
`rtlsdr_get_sample_rate()` rather than from the request, because the tuner snaps
to what its PLL can synthesise.

## Gain, AGC and the cold-start trap

`mgc_gain` is a device-independent dB float snapped to the nearest step the open
device reports, so a saved value survives a change of dongle (the R820T2 and
E4000 gain tables share almost no steps).

Enabling AGC **seeds** the tuner's LNA/mixer gain-code registers with the manual
gain first, then switches to auto. librtlsdr's auto branch never writes those
code bits, and a cold start leaves them at the init array's minimum — "AGC on
from cold" otherwise sits at minimum RF gain with the digital AGC amplifying the
quantisation floor, and stays there until a manual gain is set once. Seeding
makes AGC-on behave the same from a cold start as from a live toggle.

## Testing

```
ctest --test-dir build -R nesdr --output-on-failure
```

Three tests, all hardware-free:

| Test | Covers |
|---|---|
| `conformance.nesdr` | loadable, ABI-honest descriptor and parameters |
| `nohardware.nesdr` | scan with nothing attached, repeated scans, start with no device, two-phase stop, restart |
| `models.nesdr` | the capability table — 15 profiles × 6 gates |

`models.nesdr` is the one that matters here, since the gating *is* the plugin.
It drives the exported vtable only, restates the expected table independently of
the plugin's own (a test deriving expectations from the code it checks would
pass for any table), and asserts both halves of each gate: the return code, and
that a rejected set left the stored value untouched.

**What these tests cannot tell you.** They prove the plugin is internally
consistent — that it gates the controls its table says it should. They say
nothing about whether the *table* matches reality, which is the entire content
of [Assumptions](#assumptions-this-plugin-is-built-on). A model profiled with
the wrong tuner or a bias tee it does not have passes all three tests. Nothing
here opens a device, tunes, or reads a sample.

## Assumptions this plugin is built on

Every one of these is an input to the capability table, and every one is
**unverified against hardware**. If a model behaves oddly, start here.

**From NooElec's published product pages, not measurement:**

1. **Model → tuner mapping.** Mini is R820T; Mini 2/2+, Nano 2/2+/3, SMArt v4
   and SMArTee v2 are R820T2; SMArt v5 is R820T2/R860; every XTR body is E4000.
2. **Frequency ranges** per model, including the Nano 3's 1700 MHz ceiling
   against 1750 MHz for the other R820T2 bodies.
3. **Only the SMArt v5 has a wired Q-branch.** NooElec documents HF down to
   100 kHz for the v5 alone. Every other model is assumed to have no direct-
   sampling path — an inference from *absence* of a claim, not a stated absence.
   This is the assumption most likely to be wrong for a model I have not listed.
4. **Bias tee presence.** SMArTee bodies have one; nothing else does. Same
   inference-from-absence caveat: a bias tee is a headline feature NooElec
   advertises, so silence is taken to mean "not fitted".
5. **A SMArTee's bias tee is hardware-permanent, not on the RTL2832U GPIO.**
   Follows from NooElec describing it as needing "no hardware or software
   modifications", at 4.5 V / 250 mA with a self-resetting fuse.
6. **The E4000 L-band gap sits around 1100–1250 MHz.** Its real edges vary per
   tuner; reports range from ~1100–1200 to ~1105–1268 MHz. The plugin only warns
   on it, never corrects, precisely because the figure is not trustworthy.

**From reading librtlsdr 0.9.0, not from running it:**

7. **`rtlsdr_set_offset_tuning()` on an R820T/R828D switches the bias tee** and
   returns −2 (`src/librtlsdr.c`, the "RTL-SDR-BLOG Hack" branch). This is the
   sharpest assumption in the plugin — it is why `offset_tuning` is disabled on
   every R820T2 model. Read from source; not observed on a scope.
8. **`rtlsdr_set_direct_sampling()` re-initialises the whole tuner** on every
   call including a no-op, discarding bandwidth and gain register state — hence
   the transition guard and the re-apply after it. Inherited from `rtl_sdr`,
   where it was found the hard way.
9. **Gain step tables**: R820T2 spans 0…49.6 dB over 29 steps, E4000 −1.0…42.0
   over 14 (`r82xx_gains[]` / `e4k_gains[]`). The plugin queries the real table
   from the device at open and snaps to it, so a wrong constant here only
   affects the declared parameter range, not what is programmed.
10. **`rtlsdr_get_tuner_type()` cannot distinguish R820T from R820T2** — both
    report `RTLSDR_TUNER_R820T` — so tuner detection resolves a *family*, and
    the specific chip name is model metadata only.

**Structural, and safe under either outcome:**

11. **A rejected parameter never reaches hardware.** Beyond rejecting the set,
    `start()` re-checks the profile before every hardware call, so no stored
    value — however a saved `.speculor` restores it, and whatever order the
    engine applies parameters in — can drive a control the board lacks. This is
    the property that makes assumptions 3–5 fail *safe*: if a model is
    mis-profiled, a control is needlessly greyed out rather than a wrong
    register written.
12. **`RtlSdrDevice::close()`'s unconditional bias-T-off write is inert on a
    SMArTee**, because the supply is not on that GPIO. Reasoning from
    assumption 5, not an observation. If it is wrong, the symptom is a SMArTee's
    bias tee dropping on pipeline stop.

Auto-detect falls back to a tuner-derived generic profile rather than guessing a
model, so an unlisted or mis-stringed unit degrades to *correct but
unspecialised* instead of *confidently wrong*.

## First hardware bring-up

In order, because each step gates the next:

1. **Does it enumerate?** Scan and read the log:
   ```
   NESDR: found 1 device(s)
   NESDR:   [1] NESDR Nano 3 [00000001]
   ```
   A `Generic RTL2832U OEM [...]` label instead means the EEPROM carries stock
   Realtek strings — expected on many Mini/Nano bodies, not a fault. Set the
   `model` parameter by hand and carry on.
2. **Does the profile resolve as expected?** At start:
   ```
   NESDR: NESDR Nano 3 (tuner R820T2, profile nano3, bias tee none,
          direct sampling not wired, 0.5 PPM TCXO)
   ```
   A `trusting the tuner` or `honouring the pin` warning here means the USB
   strings and the silicon disagree — worth investigating before anything else,
   since every gate downstream keys off this line. Checks assumptions 1 and 10.
3. **Does it stream?** Tune to a strong local FM broadcast into `sdr_spectrum`
   and confirm a carrier in the expected bin. Then check the I/Q metadata
   `center_freq_hz` / `sample_rate_hz` — these are stamped from the hardware's
   own read-back, so a mismatch against what you asked for is informative, not
   a bug.
4. **Are the gates right for this body?** On an R820T2 model, `offset_tuning`,
   `bias_tee`, `direct_sampling`, `if_gain` and `if_gain_stage` should all read
   disabled, and `dithering` live. Checks assumptions 3, 4 and 7.
5. **Below the floor.** Tune under the model's minimum (e.g. 20 MHz on a
   Nano 3): it should clamp with a warning and **not** flip to Q-ADC direct
   sampling. That is the specific `rtl_sdr` behaviour this plugin exists to
   avoid.
6. **Teardown.** Start/stop/start a few times — that path is where the
   librtlsdr heap corruption and the unbounded-stop bug both lived. On a
   SMArTee, watch whether the bias tee survives a stop (assumption 12).
7. **E4000 only:** sweep across 1100–1250 MHz and note where lock is actually
   lost, then check the six IF gain stages respond. Nothing in the XTR path has
   ever run.

## Installing librtlsdr

`librtlsdr` is `dlopen`/`LoadLibrary`-loaded at runtime, never linked. The
plugin therefore always loads; without the library it simply reports **no
devices** and logs a one-time hint. An empty device list is the symptom to
look for.

### Windows

`rtlsdr.dll` ships in this bundle's `vendor/` folder and is found automatically
via the plugin's DLL search path — nothing to install. If the list is empty:

- Confirm `plugins/oss/vendor/rtlsdr.dll` exists; re-extract the bundle if not.
- **Install the WinUSB driver.** Windows binds RTL2832U devices to its DVB-T
  driver by default, which librtlsdr cannot open. Use
  [Zadig](https://zadig.akeo.ie/): *Options → List All Devices*, select the
  `Bulk-In, Interface (Interface 0)` entry for the dongle, choose **WinUSB**,
  and click Replace Driver. This is per physical USB port on some systems.
- A dongle already open in SDR#, SDRSharp, rtl_tcp or another Speculor node
  cannot be opened twice — including by `rtl_sdr` in the same pipeline.

The bundled DLL is upstream's `rtlsdr-bin-w64_static` build, which links libusb
statically, so no further runtime is needed.

### Linux

Install from your package manager, then restart the app:

```bash
sudo apt install librtlsdr0      # Debian / Ubuntu
sudo dnf install rtl-sdr         # Fedora
sudo pacman -S rtl-sdr           # Arch
```

Two further steps are usually needed:

- **udev rules for non-root access.** The distro package normally installs
  `/etc/udev/rules.d/rtl-sdr.rules`; if the device only works under `sudo`, that
  file is missing or the rules were not reloaded. Reload and replug:
  ```bash
  sudo udevadm control --reload-rules && sudo udevadm trigger
  ```
- **Blacklist the DVB-T kernel driver.** Linux binds `dvb_usb_rtl28xxu` on plug,
  which holds the device against librtlsdr:
  ```bash
  echo 'blacklist dvb_usb_rtl28xxu' | sudo tee /etc/modprobe.d/blacklist-rtl.conf
  sudo rmmod dvb_usb_rtl28xxu      # for the current session
  ```

The plugin looks for `librtlsdr.so.0` first, then `librtlsdr.so`, so a
`-dev`-only install without the runtime SONAME will not be found.

### macOS

```bash
brew install librtlsdr
```

Nothing is vendored for macOS, and no NESDR has been tested there.

