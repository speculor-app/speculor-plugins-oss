#!/usr/bin/env python3
"""
Download small country flag PNG images for the ADS-B display plugin.

Downloads 20x15 flag images from flagcdn.com and saves them to
plugins/renderers/spatial/adsb_display/data/flags/

Usage:
    python scripts/download_flags.py
"""

import os
import sys
import urllib.request
import time

# ISO 3166-1 alpha-2 country codes used in the ICAO database
COUNTRY_CODES = [
    "ad", "ae", "af", "ag", "al", "am", "ao", "ar", "at", "au",
    "az", "ba", "bb", "bd", "be", "bf", "bg", "bh", "bi", "bj",
    "bn", "bo", "br", "bs", "bt", "bw", "by", "bz", "ca", "cd",
    "cf", "cg", "ch", "ci", "cl", "cm", "cn", "co", "cr", "cu",
    "cv", "cy", "cz", "de", "dj", "dk", "do", "dz", "ec", "ee",
    "eg", "es", "et", "fi", "fj", "fm", "fr", "ga", "gb", "gd",
    "ge", "gh", "gm", "gn", "gq", "gr", "gt", "gw", "gy", "hn",
    "hr", "ht", "hu", "id", "ie", "il", "in", "iq", "ir", "is",
    "it", "jm", "jo", "jp", "ke", "kg", "kh", "km", "kp", "kr",
    "kw", "kz", "la", "lb", "lk", "lr", "ls", "lt", "lu", "lv",
    "ly", "ma", "mc", "md", "me", "mg", "mh", "mk", "ml", "mm",
    "mn", "mr", "mt", "mu", "mv", "mw", "mx", "my", "mz", "ne",
    "ng", "ni", "nl", "no", "np", "nr", "nz", "om", "pa", "pe",
    "pg", "ph", "pk", "pl", "ps", "pt", "pw", "py", "qa", "ro",
    "rs", "ru", "rw", "sa", "sb", "sc", "sd", "se", "sg", "si",
    "sk", "sl", "sm", "sn", "so", "sr", "st", "sv", "sy", "sz",
    "td", "tg", "th", "tj", "tl", "tm", "tn", "to", "tr", "tt",
    "tw", "tz", "ua", "ug", "us", "uy", "uz", "vc", "ve", "vn",
    "vu", "ws", "ye", "za", "zm", "zw",
]

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    flags_dir = os.path.join(
        project_root,
        "plugins", "renderers", "spatial", "adsb_display", "data", "flags"
    )

    os.makedirs(flags_dir, exist_ok=True)

    print(f"Downloading {len(COUNTRY_CODES)} flag images to {flags_dir}")

    success = 0
    failed = 0

    for code in COUNTRY_CODES:
        url = f"https://flagcdn.com/20x15/{code}.png"
        out_path = os.path.join(flags_dir, f"{code}.png")

        if os.path.exists(out_path):
            success += 1
            continue

        try:
            req = urllib.request.Request(url, headers={
                "User-Agent": "Speculor/0.1 (flag downloader)"
            })
            with urllib.request.urlopen(req, timeout=10) as resp:
                data = resp.read()
                with open(out_path, "wb") as f:
                    f.write(data)
            success += 1
            sys.stdout.write(".")
            sys.stdout.flush()
            # be polite to the CDN
            time.sleep(0.1)
        except Exception as e:
            failed += 1
            print(f"\n  Failed: {code} - {e}")

    print(f"\nDone: {success} downloaded, {failed} failed")


if __name__ == "__main__":
    main()
