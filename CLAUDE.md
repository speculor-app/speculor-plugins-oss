# CLAUDE.md — Speculor Open-Source Plugins

## Workflow rules

- **Never commit or push** unless the user explicitly asks.
- **Update docs before committing** — keep README.md, this file, NOTICE and
  THIRD_PARTY_NOTICES.md in sync with plugin/licensing changes.
- This is a **public, mixed-license** repo. Treat licensing carefully: every
  third-party-derived or copyleft-linked addition needs a `plugins/<name>/LICENSE`
  and a `THIRD_PARTY_NOTICES.md` entry. Putting code in a public repo does **not**
  by itself make bundling copyleft (e.g. GPL) code safe — that needs separate review.

## What this repo is

The public home for Speculor plugins that carry **open-source obligations**, kept
separate from the closed first-party `speculor-plugins` catalog. It builds against
the public [Speculor SDK bundle](https://github.com/speculor-app/speculor-sdk-dist/releases)
(source: private `speculor-sdk`) like `speculor-plugin-examples`.

## Layout

```
common/bgs/        # LITIV-derived CPU algorithm sources (Apache-2.0, © 2015 P-L St-Charles)
  vibe/            #   Vibe.{hpp,cpp}, VibeUtils.hpp
  subsense/        #   SuBSENSE.{hpp,cpp}, SuBSENSEUtils.hpp, LBSP.hpp
plugins/
  vibe_bgs/        # ViBe BGS plugin (CPU + Vulkan GPU)
  subsense_bgs/    # SuBSENSE BGS plugin (CPU + Vulkan GPU)
  rtl_sdr/         # RTL-SDR I/Q source (links librtlsdr LGPL-2.1+ at runtime)
  cmake/           # FindRtlSdr.cmake (header locator) + RtlSdr.cmake (downloader)
```

## Build

```bash
cmake -S . -B build -G Ninja -DCMAKE_PREFIX_PATH=/path/to/SpeculorSDK-<ver>-<platform>
cmake --build build
```

Output: `build/plugins/` (set via `SPC_PLUGIN_OUTPUT_DIR`). No vcpkg — the SDK bundle
is self-contained.

## How the BGS plugins consume the SDK

The CPU algorithm under `common/bgs/` is compiled into each plugin via
`target_sources(... ${SPC_OSS_COMMON_DIR}/bgs/<algo>/<Algo>.cpp)`. The shared
`CoreBgs`/`CoreParameters` base, the `coreUtils`/`pcg32` headers and OpenCV come from
`SpeculorSDK::spclib` (kept linked). The moved headers include the base via SDK
angle-includes (`<bgs/CoreBgs.hpp>`, `<include/pcg32.hpp>`); `${SPC_OSS_COMMON_DIR}`
on the include path resolves the plugin's own `<bgs/vibe/Vibe.hpp>`. The two `bgs/`
include trees are disjoint, so there is no collision.

## CI

`.github/workflows/build.yml` downloads the latest published SDK bundle from
`speculor-sdk-dist` and builds the plugins on Windows + Linux.
