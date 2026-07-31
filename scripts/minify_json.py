#!/usr/bin/env python3
"""Strip insignificant whitespace from a JSON file, in place.

The OpenAIP data is assembled by cmake/deps/OpenAIP.cmake out of `string(JSON
... GET)` results, and that command re-serialises pretty-printed — about 47% of
the bytes in the airspace/airports files are indentation. Those files are the
dominant input to both release compressors (the -full archive and the adsb
bundle), so the whitespace is paid for twice on every release build.

Works a line at a time rather than loading the file: a raw newline cannot appear
inside a JSON string (it has to be escaped as \\n), so splitting on newlines can
never cut a string literal in half, and each line can be rewritten on its own.
Within a line the regex matches whole string literals first, so spaces that are
part of a value are preserved and only the whitespace between tokens is dropped.

Usage: minify_json.py <file> [<file> ...]
"""
from __future__ import annotations

import os
import re
import sys

# A whole string literal (so its contents are handed back untouched), or a run
# of whitespace between tokens (dropped). Alternation order matters.
_TOKEN = re.compile(rb'("(?:[^"\\]|\\.)*")|[ \t\r\n]+')

# If the first chunk has no newline the file is already minified (or was never
# pretty-printed); rewriting it would mean holding the whole thing as one line.
_PROBE = 1 << 20


def _keep_strings(m: "re.Match[bytes]") -> bytes:
    return m.group(1) or b""


def minify(path: str) -> tuple[int, int]:
    """Rewrite `path` without insignificant whitespace. Returns (before, after)."""
    before = os.path.getsize(path)

    with open(path, "rb") as f:
        if b"\n" not in f.read(_PROBE):
            return before, before  # already minified — nothing to do

    tmp = path + ".min.tmp"
    try:
        with open(path, "rb") as fi, open(tmp, "wb") as fo:
            for line in fi:
                fo.write(_TOKEN.sub(_keep_strings, line))
        os.replace(tmp, path)
    except BaseException:
        if os.path.exists(tmp):
            os.remove(tmp)
        raise

    return before, os.path.getsize(path)


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print(__doc__, file=sys.stderr)
        return 2
    for path in argv[1:]:
        if not os.path.isfile(path):
            print(f"minify_json: no such file: {path}", file=sys.stderr)
            return 1
        before, after = minify(path)
        if after < before:
            print(f"[minify] {os.path.basename(path)}: "
                  f"{before / 1048576:.1f} MB -> {after / 1048576:.1f} MB "
                  f"({100 - after * 100 // before}% smaller)")
        else:
            print(f"[minify] {os.path.basename(path)}: already minified")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
