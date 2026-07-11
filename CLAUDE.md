# CLAUDE.md — Speculor Open-Source Plugins

## Scope & Platform Constraints

Speculor — and everything under it (`speculor-app`, `speculor-sdk`, `speculor-plugins`, `speculor-plugins-oss`) — is a **generic, multi-purpose pipeline engine for multi-sensor / multi-camera fusion**, not a single-machine tool. Design, optimize, and review accordingly:

- **Must run on any hardware.** Performance has to hold across the whole range — low-end CPUs and integrated GPUs through high-end discrete GPUs, plus headless / edge boxes. **Never assume a powerful machine.** "It runs fine on the dev's box" is not the bar; the constrained low end and many-stream load are first-class targets.
- **Many sensors at once.** The common deployment joins *many* concurrent cameras/sensors into one pipeline, so every per-stream cost is multiplied. Avoid redundant work, per-frame allocations, busy-waits, and contention on shared resources (GPU queues, VRAM, memory bandwidth, threads) — they all scale with sensor count.
- **Optimize for the constrained case.** Weigh designs against the low-end / high-fan-in deployment, not the developer's high-end GPU. If a choice trades generality or low-end performance for peak-machine speed, surface the tradeoff rather than assuming the strong hardware.

## Workflow rules

- **Never commit or push** unless the user explicitly asks.
- **Commit messages and PR titles follow Conventional Commits** (`type(scope): subject` — full rules in `CONTRIBUTING.md`, enforced by CI on PRs). Release notes are generated from `feat`/`fix`/`perf` subjects, so write them for a user reading the changelog. Applies to all four Speculor repos.
- **Never merge with a red Build check.** CI builds against the latest *published* SDK bundle from `speculor-sdk-dist`, so a PR that needs unreleased SDK API stays open until that SDK release is published (this exact skew broke main on 2026-07-04: `sdr_source_helpers.h` was merged before any bundle contained it). Build (Linux) / Build (Windows) / Conventional commits are required checks on `main`.
- **Update docs before committing** — keep README.md, this file, NOTICE and
  THIRD_PARTY_NOTICES.md in sync with plugin/licensing changes.
- This is a **public, mixed-license** repo. Treat licensing carefully: every
  third-party-derived or copyleft-linked addition needs a `plugins/<name>/LICENSE`
  and a `THIRD_PARTY_NOTICES.md` entry. Putting code in a public repo does **not**
  by itself make bundling copyleft (e.g. GPL) code safe — that needs separate review.
  - **Comments**: Add a comment only when it explains something the code itself cannot show — a non-obvious *why*, an invariant, or a gotcha. Don't restate what the code already says, and never narrate changes, fixes, or history in comments — that belongs in the git commit message.
  - **Don't add co-authored to commits, merge and PRs description.** Keep comments and PRs description clean of co-authored messages.

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
