# CLAUDE.md — Speculor Open-Source Plugins

## Scope & Platform Constraints

Speculor — and everything under it (`speculor-app`, `speculor-sdk`, `speculor-plugins`, `speculor-plugins-oss`) — is a **generic, multi-purpose pipeline engine for multi-sensor / multi-camera fusion**, not a single-machine tool. Design, optimize, and review accordingly:

- **Must run on any hardware.** Performance has to hold across the whole range — low-end CPUs and integrated GPUs through high-end discrete GPUs, plus headless / edge boxes. **Never assume a powerful machine.** "It runs fine on the dev's box" is not the bar; the constrained low end and many-stream load are first-class targets.
- **Many sensors at once.** The common deployment joins *many* concurrent cameras/sensors into one pipeline, so every per-stream cost is multiplied. Avoid redundant work, per-frame allocations, busy-waits, and contention on shared resources (GPU queues, VRAM, memory bandwidth, threads) — they all scale with sensor count.
- **Optimize for the constrained case.** Weigh designs against the low-end / high-fan-in deployment, not the developer's high-end GPU. If a choice trades generality or low-end performance for peak-machine speed, surface the tradeoff rather than assuming the strong hardware.

## Workflow rules

- **Never commit or push** unless the user explicitly asks.
- **Commit messages and PR titles follow Conventional Commits** (`type(scope): subject` — full rules in `CONTRIBUTING.md`, enforced by CI on PRs). Release notes are generated from `feat`/`fix`/`perf` subjects, so write them for a user reading the changelog. Applies to all four Speculor repos.
- **ALWAYS merge PRs with a merge commit — never squash, never rebase.** `gh pr merge <n> --merge --delete-branch`. Applies to every Speculor repo, whatever a given repo's existing history happens to show, and regardless of whether the PR has one commit or twenty. Squashing throws away the per-commit messages, which are the record of *why* each change was made. After merging: `git checkout main && git pull`, then drop the local branch.
- **Never merge with a red Build check.** CI builds against the latest *published* SDK bundle from `speculor-sdk-dist`, so a PR that needs unreleased SDK API stays open until that SDK release is published (this exact skew broke main on 2026-07-04: `sdr_source_helpers.h` was merged before any bundle contained it). Build (Linux) / Build (Windows) / Conventional commits are required checks on `main`.
- **Every new plugin ships with tests, and must be added to `tests/CMakeLists.txt` by name.** Unlike `speculor-plugins`, the list here is explicit — a plugin missing from it is never tested, which is indistinguishable from a plugin that passes. Add it to `_candidates` for conformance, then add a behavioural test for whatever contract it actually has. Verify with `ctest -R <name>` rather than assuming registration worked. See **Tests** below.
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
  rtl_sdr/         # RTL-SDR I/Q source — data-source plugin (SPC_PLUGIN_DATA_SOURCE, needs SDK ≥ 0.20.0); links librtlsdr LGPL-2.1+ at runtime
  kraken_sdr/      # KrakenSDR 5-channel coherent RTL-SDR array source (5 signal ports); compiles rtl_sdr's device layer; links librtlsdr LGPL-2.1+ at runtime
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

## Tests

`-DSPECULOR_BUILD_TESTS=ON` builds `tests/`; run with `ctest --test-dir build --output-on-failure`. Ten tests run per platform in CI, and `Build (Linux)` / `Build (Windows)` are required checks.

**The plugin list is explicit, and that is the trap.** `tests/CMakeLists.txt` holds a `_candidates` list filtered through `if(TARGET ...)` — the filter exists so a plugin held back by the release gate drops out instead of failing the configure, *not* to discover new plugins. Nothing warns you about a plugin you never added; it is simply never tested, which reads exactly like a plugin that passes. (`speculor-plugins` auto-discovers instead, because at ~140 plugins a hand-written list would be stale within a week.)

Four tiers, each registered per plugin so a failure names the plugin:

| Tier | Runner | Covers |
|---|---|---|
| `conformance.<plugin>` | `conformance_runner.cpp` | every plugin — loadable, ABI-honest |
| `behaviour.<plugin>` | `bgs_behaviour_runner.cpp` | BGS pair — image in, foreground mask out |
| `nohardware.<plugin>` | `sdr_nohardware_runner.cpp` | SDR sources — no-device paths |
| `display.<plugin>` | `adsb_display_runner.cpp` | waypoint round-trip, hostile inputs |

Conformance is the floor, not the goal: it proves the plugin loads and answers the ABI honestly, never that it works. Pick the tier that matches the plugin's contract — and where no hardware exists in CI, the no-device paths are still worth asserting, since the librtlsdr heap corruption and the unbounded-stop bugs both lived there.

The harness ships in the bundle at `include/speculor_common/testing/`, which `spc_add_plugin_test` puts on the include path — so a runner includes it as `<testing/conformance.h>`, `<testing/plugin_under_test.h>`, `<testing/fake_host.h>`. `PluginUnderTest` loads the built plugin through its exported `spc_plugin_vtable`, `FakeHost` stands in for `SpcHostServices`, and `run_conformance()` returns a report rather than asserting.

It arrives with the bundle CI already downloads — but a harness fix only reaches this repo when a **new bundle is published**, since the build resolves the latest published release rather than a pinned version.

## CI

`.github/workflows/build.yml` downloads the latest published SDK bundle from
`speculor-sdk-dist`, builds the plugins on Windows + Linux, and runs `ctest`.
