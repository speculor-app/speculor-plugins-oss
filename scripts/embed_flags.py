#!/usr/bin/env python3
"""
Generate embedded_flags.h from the downloaded flag PNG files.

Reads all PNGs from plugins/renderers/spatial/adsb_display/data/flags/
and produces a C++ header with the raw bytes as constexpr arrays.

Usage:
    python scripts/embed_flags.py
"""

import os
import glob
import sys


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    flags_dir = os.path.join(
        project_root,
        "plugins", "renderers", "spatial", "adsb_display", "data", "flags"
    )
    out_path = os.path.join(
        project_root,
        "plugins", "renderers", "spatial", "adsb_display", "embedded_flags.h"
    )

    files = sorted(glob.glob(os.path.join(flags_dir, "*.png")))
    if not files:
        print(f"No PNG files found in {flags_dir}", file=sys.stderr)
        print("Run scripts/download_flags.py first.", file=sys.stderr)
        sys.exit(1)

    lines = []
    lines.append("#pragma once")
    lines.append("")
    lines.append("// auto-generated — do not edit")
    lines.append("// embedded country flag PNG data (generated from data/flags/*.png)")
    lines.append("//")
    lines.append("// regenerate with: python scripts/download_flags.py && python scripts/embed_flags.py")
    lines.append("")
    lines.append("#include <cstdint>")
    lines.append("#include <cstddef>")
    lines.append("")
    lines.append("namespace embedded_flags {")
    lines.append("")

    entries = []
    for f in files:
        code = os.path.splitext(os.path.basename(f))[0]
        with open(f, "rb") as fh:
            data = fh.read()

        var_name = f"k_flag_{code}"
        hex_bytes = ", ".join(f"0x{b:02x}" for b in data)
        lines.append(f"static constexpr uint8_t {var_name}[] = {{{hex_bytes}}};")
        entries.append((code, var_name, len(data)))

    lines.append("")
    lines.append("struct EmbeddedFlag {")
    lines.append("    char code[3];          // ISO 3166-1 alpha-2")
    lines.append("    const uint8_t* data;")
    lines.append("    size_t size;")
    lines.append("};")
    lines.append("")
    lines.append("static constexpr EmbeddedFlag k_all_flags[] = {")
    for code, var_name, size in entries:
        lines.append(f'    {{"{code}", {var_name}, {size}}},')
    lines.append("};")
    lines.append("")
    lines.append(f"static constexpr size_t k_flag_count = {len(entries)};")
    lines.append("")
    lines.append("} // namespace embedded_flags")
    lines.append("")

    with open(out_path, "w", newline="\n") as f:
        f.write("\n".join(lines))

    total_bytes = sum(size for _, _, size in entries)
    print(f"Generated {out_path}")
    print(f"  {len(entries)} flags, {total_bytes} bytes ({total_bytes / 1024:.1f} KB)")


if __name__ == "__main__":
    main()
