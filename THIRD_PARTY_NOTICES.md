# Third-Party Notices — Speculor Open-Source Plugins

The plugins in this repository incorporate or link the third-party software listed
below. Each component is governed by its own license terms; nothing in `LICENSE`
modifies those terms. Per-plugin specifics are also recorded in each
`plugins/<name>/LICENSE`.

When compiled, every plugin additionally links the Speculor Plugin SDK and, through
it, the third-party libraries bundled in the SDK (OpenCV, FFmpeg — an LGPL-only
build — Vulkan, Eigen, libjpeg-turbo, and others). Those are reproduced in
`THIRD_PARTY_NOTICES.md` inside the SDK distribution you build against:
<https://github.com/speculor-app/speculor-sdk-dist/releases>.

## Summary

| Component | Version | License | SPDX |
|---|---|---|---|
| LITIV Framework (ViBe, SuBSENSE, LBSP) | git, © 2015 Pierre-Luc St-Charles | Apache License 2.0 | Apache-2.0 |
| librtlsdr (librtlsdr/librtlsdr fork) | 0.9.0 | GNU Lesser General Public License v2.1 or later | LGPL-2.1-or-later |
| libusb (statically linked inside the shipped librtlsdr binary) | per upstream release | GNU Lesser General Public License v2.1 or later | LGPL-2.1-or-later |

## LITIV Framework (ViBe, SuBSENSE, LBSP)

- Project: <https://github.com/plstcharles/litiv>
- Author: Pierre-Luc St-Charles and contributors
- Copyright: © 2015 Pierre-Luc St-Charles
- License: Apache License 2.0.

The `vibe_bgs` and `subsense_bgs` plugins build on background-subtraction code under
`common/bgs/` that derives from the LITIV Framework: the ViBe sampler, the SuBSENSE
change detector, and the Local Binary Similarity Pattern (LBSP) descriptor. Each
derived source file carries the LITIV copyright and Apache-2.0 banner. Some LITIV
source is itself reimplemented from OpenCV; the OpenCV license (Apache-2.0) therefore
also applies to those portions.

**Patent note.** The ViBe algorithm (Barnich & Van Droogenbroeck, University of
Liège) is covered by patents in some jurisdictions. The Apache-2.0 patent grant in
the LITIV license is made by the LITIV authors and does not extend to the underlying
ViBe patent held by third parties. Recipients are responsible for determining whether
use of the ViBe-based motion detector requires a patent license in their jurisdiction.

## librtlsdr

- Project: <https://github.com/librtlsdr/librtlsdr>
- License: GNU Lesser General Public License, version 2.1 or later (LGPL-2.1-or-later).

This fork rather than another because it is the only one exporting `rtlsdr_set_dithering`,
which a coherent array needs in order to stop the R820T2's sigma-delta modulator drifting
each tuner's phase independently. The Windows build shipped is the upstream
`rtlsdr-bin-w64_static` release, which links libusb statically.

The `rtl_sdr`, `nesdr` and `kraken_sdr` plugins use the RTL-SDR API (the NooElec
NESDR line is RTL2832U-based; the KrakenSDR is a five-channel RTL-SDR array).
The library (`rtlsdr.dll` / `librtlsdr.so`)
is **loaded at runtime** via `LoadLibrary`/`dlopen`; only the `rtl-sdr.h` header is
used at compile time. Because the library is dynamically loaded rather than statically
linked, the LGPL's relinking requirement is satisfied by the runtime-replaceable
library; redistributing the plugin alongside `rtlsdr.dll`/`librtlsdr.so` carries the
LGPL attribution and source-availability obligations for that library.

The Windows binary is upstream's static build, which has **libusb** (also
LGPL-2.1-or-later) linked into it, so redistributing it carries libusb's obligations
too. The same relinking argument covers it: the whole library is loaded at runtime and
a recipient can replace it wholesale.

The full license text for each component is distributed with its upstream project and
applies unmodified.
