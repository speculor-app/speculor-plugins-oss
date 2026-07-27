# Speculor Open-Source Plugins

Open-source plugins for [Speculor](https://speculor.app) — the real-time
multi-camera / signal-processing platform. This repository is the public home for
Speculor plugins that carry **open-source obligations**: code derived from
permissively-licensed upstreams that should be readable in source form, and plugins
that link copyleft libraries. They build against the public Speculor SDK exactly like
any other plugin.

> This is distinct from
> [speculor-plugin-examples](https://github.com/speculor-app/speculor-plugin-examples)
> (minimal teaching templates that vendor no third-party code). The plugins here are
> production algorithms with real third-party provenance.

## Plugins

| Plugin | In → Out | What it is | License notes |
|--------|----------|-----------|---------------|
| **vibe_bgs** | frame → frame | ViBe background subtraction (CPU + Vulkan GPU) | Apache-2.0; LITIV-derived; **ViBe is patent-encumbered** in some jurisdictions |
| **subsense_bgs** | frame → frame | SuBSENSE background subtraction (CPU + Vulkan GPU) | Apache-2.0; LITIV-derived |
| **rtl_sdr** | — → signal | RTL-SDR I/Q source (data-source plugin — captured by session recording, driven by reinjection replay) | Apache-2.0 plugin code; links **librtlsdr (LGPL-2.1+)** at runtime |
| **kraken_sdr** | — → 5× signal | KrakenSDR 5-channel coherent RTL-SDR array source (data-source plugin; reuses the `rtl_sdr` device layer) | Apache-2.0 plugin code; links **librtlsdr (LGPL-2.1+)** at runtime |

> **Linux: `rtl_sdr` and `kraken_sdr` need librtlsdr installed separately.** It
> is `dlopen`ed at runtime rather than linked, so it is in no Speculor archive
> and the plugins load fine without it — they simply report no devices. Install
> it with `sudo apt install librtlsdr0` (Debian/Ubuntu), `sudo dnf install
> rtl-sdr` (Fedora) or `sudo pacman -S rtl-sdr` (Arch). The plugins log the same
> hint when it is missing. Windows ships `rtlsdr.dll` in the bundle's `vendor/`
> folder, so nothing extra is needed there.

The ViBe/SuBSENSE CPU algorithms live under `common/bgs/` (derived from the
[LITIV Framework](https://github.com/plstcharles/litiv), © 2015 Pierre-Luc
St-Charles, Apache-2.0). They compile into the plugins here and link the Speculor
SDK's `spclib` only for the shared `CoreBgs` base and OpenCV — so the SDK itself
ships no LITIV-derived code.

## Quick start

1. **Download the SDK bundle** from
   [speculor-sdk-dist releases](https://github.com/speculor-app/speculor-sdk-dist/releases)
   and extract it. It is self-contained (ships OpenCV, FFmpeg, Vulkan, spclib + the
   CMake config/helpers) — you need only CMake ≥ 3.24, Ninja and a C++20 compiler.
   The current plugins require an SDK bundle **≥ 0.20.0** (`rtl_sdr` declares the
   `SPC_PLUGIN_DATA_SOURCE` descriptor flag, added in SDK 0.20.0).

2. **Build**, pointing `CMAKE_PREFIX_PATH` at the extracted folder:

   ```bash
   cmake -S . -B build -G Ninja \
     -DCMAKE_PREFIX_PATH=/path/to/SpeculorSDK-<ver>-<platform>
   cmake --build build
   ```

   > **Linux:** the BGS plugins link `spclib`, which pulls the SDK's VA-API-enabled
   > FFmpeg — install the system libraries first:
   > ```bash
   > sudo apt-get install -y libva-dev libdrm-dev
   > ```
   > `std::execution::par` in ViBe needs TBB on libstdc++ (`libtbb-dev`).

   > **rtl_sdr:** `RtlSdr.cmake` downloads `rtl-sdr.h` (and, on Windows, the prebuilt
   > `rtlsdr.dll`) at configure time. Offline / no system header → the plugin self-skips.

3. **Copy the built plugins** from `build/plugins/` into Speculor's `plugins/`
   directory and restart the app.

## License

This repository is **mixed-license**. Repository-original code (plugin wrappers,
scaffolding, build files) is **Apache-2.0** — see the root [`LICENSE`](LICENSE) and
[`NOTICE`](NOTICE). Individual plugins carry their own obligations:

- `vibe_bgs`, `subsense_bgs` — Apache-2.0, contain LITIV-derived code; **ViBe is
  patent-encumbered** in some jurisdictions.
- `rtl_sdr`, `kraken_sdr` — Apache-2.0 plugin code linking **librtlsdr (LGPL-2.1+)** at runtime.

See each `plugins/<name>/LICENSE` and the root
[`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md). The plugins load at runtime against
the Speculor engine (proprietary) through the documented plugin C ABI and link the
Speculor Plugin SDK (proprietary); those carry their own licenses, and the grants in
this repository do not extend to them.
