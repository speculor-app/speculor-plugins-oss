#!/usr/bin/env python3
"""
Convert aircraft SVG shapes from tar1090's markers.js into C++ polygon point arrays.

Downloads markers.js from the tar1090 GitHub repository, extracts SVG path data
for selected aircraft shapes, converts bezier curves to polyline points using
de Casteljau subdivision, and generates aircraft_shapes.h.

Usage:
    python scripts/convert_shapes.py

Output:
    plugins/adsb_display/aircraft_shapes.h

Source: https://github.com/wiedehopf/tar1090 (GPL-3.0)
Some icons CC BY 4.0 (Peter Lowden), some by pimlie.
"""

import json
import math
import os
import re
import sys
import urllib.request

MARKERS_URL = "https://raw.githubusercontent.com/wiedehopf/tar1090/master/html/markers.js"

# shapes to extract (tar1090 shape name)
SELECTED_SHAPES = [
    # commercial
    "airliner", "a319", "a320", "a321", "a332", "a359", "a380", "heavy_4e", "heavy_2e",
    "b707", "b737", "b738", "b739", "md11", "beluga",
    # regional / turboprop
    "jet_nonswept", "twin_small", "twin_large", "c130",
    # general aviation
    "cessna", "cirrus_sr22", "pa24", "single_turbo", "gyrocopter",
    # business jets
    "jet_swept", "e390",
    # military - fighters
    "f18", "f35", "typhoon", "tornado", "rafale", "hi_perf",
    "mirage", "f5_tiger", "a10",
    # military - bombers / special
    "b52", "u2", "c17", "c5", "a400", "p8", "p3_orion", "e3awacs",
    # helicopters
    "helicopter", "apache", "blackhawk", "chinook", "mil24", "s61",
    # other
    "uav", "glider", "balloon", "blimp", "v22_fast",
    "ground_square", "ground_service", "ground_emergency",
]

# bezier subdivision resolution (points per curve segment)
BEZIER_STEPS = 6


def fetch_markers_js():
    """Download markers.js from tar1090 repo."""
    print("Downloading markers.js from tar1090...")
    req = urllib.request.Request(MARKERS_URL, headers={
        "User-Agent": "Speculor/0.1 (shape converter)"
    })
    with urllib.request.urlopen(req, timeout=30) as resp:
        return resp.read().decode("utf-8")


def extract_shapes(js_content):
    """Extract shape definitions from markers.js JavaScript."""
    shapes = {}

    # find each shape: 'name': { ... } by matching balanced braces
    for name in SELECTED_SHAPES:
        # find the shape definition start
        pattern = re.compile(rf"'{re.escape(name)}'\s*:\s*\{{", re.DOTALL)
        m = pattern.search(js_content)
        if not m:
            continue

        # find the matching closing brace (handle nested braces)
        start = m.end()
        depth = 1
        pos = start
        while pos < len(js_content) and depth > 0:
            if js_content[pos] == '{':
                depth += 1
            elif js_content[pos] == '}':
                depth -= 1
            pos += 1
        block = js_content[start:pos - 1]

        # Extract path. Two upstream forms: a single quoted string, or an array
        # of them for shapes drawn as several subpaths (chinook, a10,
        # gyrocopter). Concatenating the array is correct here because each
        # element begins with a moveto, which parse_svg_path already treats as
        # the start of a new subpath — and only the largest subpath is kept as
        # the outline further down.
        path_match = re.search(r"path\s*:\s*\[(.*?)\]", block, re.S)
        if path_match:
            parts = re.findall(r"['\"]([^'\"]+)['\"]", path_match.group(1))
            if not parts:
                continue
            path_data = " ".join(parts)
        else:
            path_match = re.search(r"path\s*:\s*['\"]([^'\"]+)['\"]", block)
            if not path_match:
                continue
            path_data = path_match.group(1)

        # extract viewBox
        vb_match = re.search(r"viewBox\s*:\s*'([^']+)'", block)
        if not vb_match:
            vb_match = re.search(r'viewBox\s*:\s*"([^"]+)"', block)
        viewbox = vb_match.group(1).split() if vb_match else None

        shapes[name] = {
            "path": path_data,
            "viewBox": [float(v) for v in viewbox] if viewbox else None,
        }

    return shapes


_NUM_RE = re.compile(r'[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?')

# parameters consumed by one group of each command
_ARITY = {'m': 2, 'l': 2, 'h': 1, 'v': 1, 'c': 6, 's': 4, 'q': 4, 't': 2, 'a': 7, 'z': 0}


def tokenize_path(d):
    """Split SVG path data into command letters and parameter tokens.

    A plain number scanner cannot do this correctly. In an elliptical arc the
    large-arc and sweep flags are single digits that minifiers emit with no
    separator: "a18.3 18.3 0 001.72-1.54" means flags 0 and 0 followed by
    x=1.72, but a number scanner reads "001.72" as one value. The arc then has
    five parameters instead of seven, so it consumes two tokens from whatever
    command follows and dies on a letter — which is what silently dropped
    tornado, b52, c5 and s61 from the generated header.

    Scanning with each command's arity in hand fixes it: inside an arc, take
    parameters 4 and 5 as single characters.
    """
    tokens = []
    i, n = 0, len(d)
    cmd = None
    while i < n:
        ch = d[i]
        if ch in 'MmLlHhVvCcSsQqTtAaZz':
            tokens.append(ch)
            cmd = ch
            i += 1
            continue
        if ch in ' ,\t\r\n':
            i += 1
            continue
        if cmd is None or _ARITY.get(cmd.lower(), 0) == 0:
            i += 1          # stray token with no command to own it
            continue
        # One parameter group for the command in force. A bare group with no
        # letter in front of it is an implicit repeat, which this loop handles
        # without extra work.
        for k in range(_ARITY[cmd.lower()]):
            while i < n and d[i] in ' ,\t\r\n':
                i += 1
            if i >= n:
                break
            if cmd in 'Aa' and k in (3, 4):
                tokens.append(d[i])       # flag: exactly one character
                i += 1
                continue
            m = _NUM_RE.match(d, i)
            if not m:
                i += 1
                break
            tokens.append(m.group())
            i = m.end()
        # after a moveto, further coordinate pairs are implicit linetos
        if cmd == 'M':
            cmd = 'L'
        elif cmd == 'm':
            cmd = 'l'
    return tokens


def parse_svg_path(d):
    """Parse SVG path data into a list of subpaths, each a list of commands."""
    tokens = tokenize_path(d)

    subpaths = []
    current = []
    cx, cy = 0.0, 0.0
    start_x, start_y = 0.0, 0.0
    i = 0

    def next_num():
        nonlocal i
        i += 1
        return float(tokens[i])

    while i < len(tokens):
        cmd = tokens[i]

        if cmd in ('M', 'm'):
            if current:
                subpaths.append(current)
                current = []
            x = next_num()
            y = next_num()
            if cmd == 'm':
                x += cx; y += cy
            cx, cy = x, y
            start_x, start_y = x, y
            current.append(('M', x, y))
            # implicit lineto after moveto
            while i + 1 < len(tokens) and tokens[i + 1] not in 'MmLlHhVvCcSsQqTtAaZz':
                x = next_num()
                y = next_num()
                if cmd == 'm':
                    x += cx; y += cy
                cx, cy = x, y
                current.append(('L', x, y))

        elif cmd in ('L', 'l'):
            while i + 1 < len(tokens) and tokens[i + 1] not in 'MmLlHhVvCcSsQqTtAaZz':
                x = next_num()
                y = next_num()
                if cmd == 'l':
                    x += cx; y += cy
                cx, cy = x, y
                current.append(('L', x, y))

        elif cmd == 'H':
            while i + 1 < len(tokens) and tokens[i + 1] not in 'MmLlHhVvCcSsQqTtAaZz':
                x = next_num()
                cx = x
                current.append(('L', cx, cy))

        elif cmd == 'h':
            while i + 1 < len(tokens) and tokens[i + 1] not in 'MmLlHhVvCcSsQqTtAaZz':
                dx = next_num()
                cx += dx
                current.append(('L', cx, cy))

        elif cmd == 'V':
            while i + 1 < len(tokens) and tokens[i + 1] not in 'MmLlHhVvCcSsQqTtAaZz':
                y = next_num()
                cy = y
                current.append(('L', cx, cy))

        elif cmd == 'v':
            while i + 1 < len(tokens) and tokens[i + 1] not in 'MmLlHhVvCcSsQqTtAaZz':
                dy = next_num()
                cy += dy
                current.append(('L', cx, cy))

        elif cmd in ('C', 'c'):
            while i + 1 < len(tokens) and tokens[i + 1] not in 'MmLlHhVvCcSsQqTtAaZz':
                x1 = next_num(); y1 = next_num()
                x2 = next_num(); y2 = next_num()
                x = next_num(); y = next_num()
                if cmd == 'c':
                    x1 += cx; y1 += cy
                    x2 += cx; y2 += cy
                    x += cx; y += cy
                current.append(('C', cx, cy, x1, y1, x2, y2, x, y))
                cx, cy = x, y

        elif cmd in ('S', 's'):
            while i + 1 < len(tokens) and tokens[i + 1] not in 'MmLlHhVvCcSsQqTtAaZz':
                x2 = next_num(); y2 = next_num()
                x = next_num(); y = next_num()
                if cmd == 's':
                    x2 += cx; y2 += cy
                    x += cx; y += cy
                # reflect previous control point
                if current and current[-1][0] == 'C':
                    prev = current[-1]
                    x1 = 2 * cx - prev[5]
                    y1 = 2 * cy - prev[6]
                else:
                    x1, y1 = cx, cy
                current.append(('C', cx, cy, x1, y1, x2, y2, x, y))
                cx, cy = x, y

        elif cmd in ('Q', 'q'):
            while i + 1 < len(tokens) and tokens[i + 1] not in 'MmLlHhVvCcSsQqTtAaZz':
                x1 = next_num(); y1 = next_num()
                x = next_num(); y = next_num()
                if cmd == 'q':
                    x1 += cx; y1 += cy
                    x += cx; y += cy
                # convert quadratic to cubic
                cx1 = cx + 2/3 * (x1 - cx)
                cy1 = cy + 2/3 * (y1 - cy)
                cx2 = x + 2/3 * (x1 - x)
                cy2 = y + 2/3 * (y1 - y)
                current.append(('C', cx, cy, cx1, cy1, cx2, cy2, x, y))
                cx, cy = x, y

        elif cmd in ('Z', 'z'):
            current.append(('Z',))
            cx, cy = start_x, start_y

        elif cmd in ('A', 'a'):
            # arc — approximate with line to endpoint
            while i + 1 < len(tokens) and tokens[i + 1] not in 'MmLlHhVvCcSsQqTtAaZz':
                next_num()  # rx
                next_num()  # ry
                next_num()  # x-rotation
                next_num()  # large-arc
                next_num()  # sweep
                x = next_num()
                y = next_num()
                if cmd == 'a':
                    x += cx; y += cy
                cx, cy = x, y
                current.append(('L', x, y))

        i += 1

    if current:
        subpaths.append(current)

    return subpaths


def cubic_bezier(p0, p1, p2, p3, steps):
    """Sample cubic bezier curve using de Casteljau."""
    points = []
    for i in range(1, steps + 1):
        t = i / steps
        u = 1 - t
        x = u*u*u*p0[0] + 3*u*u*t*p1[0] + 3*u*t*t*p2[0] + t*t*t*p3[0]
        y = u*u*u*p0[1] + 3*u*u*t*p1[1] + 3*u*t*t*p2[1] + t*t*t*p3[1]
        points.append((x, y))
    return points


def subpath_to_points(subpath):
    """Convert a subpath (list of commands) to a polygon point list."""
    points = []
    for cmd in subpath:
        if cmd[0] == 'M':
            points.append((cmd[1], cmd[2]))
        elif cmd[0] == 'L':
            points.append((cmd[1], cmd[2]))
        elif cmd[0] == 'C':
            p0 = (cmd[1], cmd[2])
            p1 = (cmd[3], cmd[4])
            p2 = (cmd[5], cmd[6])
            p3 = (cmd[7], cmd[8])
            points.extend(cubic_bezier(p0, p1, p2, p3, BEZIER_STEPS))
        elif cmd[0] == 'Z':
            pass  # close path
    return points


def normalize_points(points, viewbox):
    """Normalize points to [-1, 1] range, centered at origin, Y pointing up."""
    if not points:
        return [], 1.0

    if viewbox:
        vx, vy, vw, vh = viewbox
    else:
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        vx, vy = min(xs), min(ys)
        vw, vh = max(xs) - vx, max(ys) - vy

    if vw == 0 or vh == 0:
        return [], 1.0

    cx = vx + vw / 2
    cy = vy + vh / 2
    scale = max(vw, vh) / 2

    normalized = []
    for x, y in points:
        nx = (x - cx) / scale
        ny = (y - cy) / scale  # SVG Y is down, keep as-is (heading 0 = north = -Y)
        normalized.append((nx, ny))

    aspect = vw / vh
    return normalized, aspect


def deduplicate_close_points(points, threshold=0.01):
    """Remove consecutive nearly-duplicate points."""
    if not points:
        return points
    result = [points[0]]
    for p in points[1:]:
        dx = p[0] - result[-1][0]
        dy = p[1] - result[-1][1]
        if dx*dx + dy*dy > threshold * threshold:
            result.append(p)
    return result


def generate_header(shapes_data, output_path):
    """Generate aircraft_shapes.h from processed shape data."""
    lines = []
    lines.append("#pragma once")
    lines.append("")
    lines.append("// auto-generated from tar1090's markers.js — do not edit")
    lines.append("// source: https://github.com/wiedehopf/tar1090 (GPL-3.0)")
    lines.append("// some icons CC BY 4.0 (Peter Lowden), some by pimlie")
    lines.append("//")
    lines.append("// regenerate with: python scripts/convert_shapes.py")
    lines.append("")
    lines.append("#include <cstdint>")
    lines.append("#include <cstring>")
    lines.append("")
    lines.append("namespace aircraft_shapes {")
    lines.append("")

    # emit point arrays
    shape_vars = {}
    for name, data in sorted(shapes_data.items()):
        points = data["points"]
        aspect = data["aspect"]
        var = f"k_pts_{name}"
        shape_vars[name] = (var, len(points), aspect)

        coords = ", ".join(f"{p[0]:.4f}f,{p[1]:.4f}f" for p in points)
        lines.append(f"static constexpr float {var}[] = {{{coords}}};")

    lines.append("")

    # shape struct
    lines.append("struct Shape {")
    lines.append("    const char* name;")
    lines.append("    const float* points;      // interleaved x,y pairs, normalized [-1,1]")
    lines.append("    int point_count;")
    lines.append("    float aspect_ratio;        // width / height")
    lines.append("};")
    lines.append("")

    # shape table
    lines.append(f"static constexpr Shape k_shapes[] = {{")
    for name in sorted(shapes_data.keys()):
        var, count, aspect = shape_vars[name]
        lines.append(f'    {{"{name}", {var}, {count}, {aspect:.3f}f}},')
    lines.append("};")
    lines.append(f"static constexpr int k_shape_count = {len(shapes_data)};")
    lines.append("")

    # lookup by name
    lines.append("inline const Shape* find_shape(const char* name) {")
    lines.append("    for (int i = 0; i < k_shape_count; ++i)")
    lines.append("        if (std::strcmp(k_shapes[i].name, name) == 0)")
    lines.append("            return &k_shapes[i];")
    lines.append("    return nullptr;")
    lines.append("}")
    lines.append("")

    # type code prefix mapping
    lines.append("// ICAO type code prefix -> shape name")
    lines.append("struct TypeMapping { const char* prefix; const char* shape; };")
    lines.append("static constexpr TypeMapping k_type_map[] = {")
    type_mappings = [
        # ── super heavy / 4-engine ──
        ("A388", "a380"), ("B748", "a380"), ("B744", "a380"), ("B742", "a380"),
        ("B74S", "a380"), ("A225", "a380"),
        ("A342", "heavy_4e"), ("A343", "heavy_4e"), ("A345", "heavy_4e"), ("A346", "heavy_4e"),
        ("IL62", "heavy_4e"), ("IL96", "heavy_4e"),
        # ── wide-body twin ──
        ("A332", "a332"), ("A333", "a332"), ("A338", "a332"), ("A339", "a332"),
        ("B762", "a332"), ("B763", "a332"), ("B764", "a332"),
        ("B772", "a332"), ("B773", "a332"), ("B77W", "a332"), ("B77L", "a332"), ("B779", "a332"),
        ("B788", "a332"), ("B789", "a332"), ("B78X", "a332"),
        # ── wide-body twin (A350 specific) ──
        ("A359", "a359"), ("A35K", "a359"),
        # ── tri-engine ──
        ("MD11", "md11"), ("DC10", "md11"), ("L101", "md11"),
        # ── narrow-body (A320 family) ──
        ("A318", "a319"), ("A319", "a319"), ("A19N", "a319"),
        ("A320", "a320"), ("A20N", "a320"),
        ("A321", "a321"), ("A21N", "a321"),
        # ── narrow-body (B737 family) ──
        ("B712", "b737"), ("B733", "b737"), ("B734", "b737"), ("B735", "b737"),
        ("B737", "b737"), ("B38M", "b738"),
        ("B738", "b738"),
        ("B739", "b739"), ("B39M", "b739"),
        # ── older narrow-body / B757 ──
        ("B703", "b707"), ("B707", "b707"), ("B722", "b707"),
        ("B752", "b707"), ("B753", "b707"), ("DC87", "b707"),
        # ── beluga / special cargo ──
        ("A3ST", "beluga"), ("BLCF", "beluga"),
        # ── regional jets ──
        ("E170", "jet_nonswept"), ("E175", "jet_nonswept"),
        ("E190", "jet_nonswept"), ("E195", "jet_nonswept"),
        ("CRJ2", "jet_nonswept"), ("CRJ7", "jet_nonswept"),
        ("CRJ9", "jet_nonswept"), ("CRJX", "jet_nonswept"),
        ("E35L", "jet_nonswept"), ("BCS1", "jet_nonswept"), ("BCS3", "jet_nonswept"),
        # ── twin turboprop ──
        ("AT45", "twin_small"), ("AT75", "twin_small"), ("ATP", "twin_small"),
        ("DH8C", "twin_small"), ("DH8D", "twin_small"), ("D328", "twin_small"),
        ("SF34", "twin_small"), ("B190", "twin_small"), ("J328", "twin_small"),
        ("D228", "twin_small"),
        # ── military transport (turboprop) ──
        ("C130", "c130"), ("C295", "c130"), ("C160", "c130"), ("CN35", "c130"),
        ("A400", "a400"),
        # ── military transport (jet) ──
        ("C17", "c17"), ("C5M", "c5"), ("C2", "c17"), ("IL76", "c5"),
        ("KC46", "a332"), ("KC2", "a332"), ("K35E", "b707"),
        # ── maritime patrol / AWACS ──
        ("P8", "p8"), ("P3", "p3_orion"), ("E737", "a320"),
        # ── light single-engine ──
        ("C172", "cessna"), ("C152", "cessna"), ("C182", "cessna"),
        ("P28A", "cessna"), ("PA28", "cessna"), ("PA18", "cessna"),
        ("DR40", "cessna"), ("C150", "cessna"),
        # ── light single (modern) ──
        ("SR20", "cirrus_sr22"), ("SR22", "cirrus_sr22"),
        # ── light twin ──
        ("DA42", "pa24"), ("PA34", "pa24"), ("BE20", "pa24"),
        ("P180", "pa24"), ("PA44", "pa24"), ("BE58", "pa24"),
        ("BE9L", "pa24"), ("C310", "pa24"), ("PA31", "pa24"),
        # ── single turboprop ──
        ("PC12", "single_turbo"), ("TBM", "single_turbo"), ("C208", "single_turbo"),
        ("PC6T", "single_turbo"), ("B350", "single_turbo"),
        # ── business jets (swept wing) ──
        ("C25B", "jet_swept"), ("C25C", "jet_swept"), ("C510", "jet_swept"),
        ("C525", "jet_swept"), ("C560", "jet_swept"), ("C680", "jet_swept"),
        ("C750", "jet_swept"), ("LJ35", "jet_swept"), ("LJ45", "jet_swept"),
        ("GL5T", "jet_swept"), ("GLF6", "jet_swept"), ("GLEX", "jet_swept"),
        ("FA7X", "jet_swept"), ("FA8X", "jet_swept"), ("F900", "jet_swept"),
        ("CL30", "jet_swept"), ("CL35", "jet_swept"), ("CL60", "jet_swept"),
        ("E390", "e390"), ("E50P", "e390"),
        # ── fighters ──
        ("F16", "f18"), ("F15", "f18"), ("F5", "f5_tiger"),
        ("F18S", "f18"), ("F18H", "f18"),
        ("F35", "f35"),
        ("EUFI", "typhoon"), ("RFAL", "rafale"),
        ("MIRA", "mirage"), ("MIR2", "mirage"),
        ("TOR", "tornado"),
        ("A10", "a10"),
        # ── bombers / reconnaissance ──
        ("B52", "b52"), ("U2", "u2"), ("B1", "hi_perf"),
        # ── trainer / light attack ──
        ("HAWK", "hi_perf"), ("PC9", "hi_perf"),
        ("M326", "hi_perf"), ("T38", "hi_perf"),
        ("L159", "hi_perf"), ("SB39", "hi_perf"),
        # ── helicopters ──
        ("EC35", "helicopter"), ("EC45", "helicopter"), ("EC20", "helicopter"),
        ("AS32", "helicopter"), ("AS65", "helicopter"),
        ("R44", "helicopter"), ("R22", "helicopter"),
        ("NH90", "helicopter"), ("GAZL", "helicopter"),
        ("H60", "blackhawk"), ("S70", "blackhawk"),
        ("H64", "apache"), ("AH64", "apache"),
        ("H47", "chinook"), ("CH47", "chinook"),
        ("MI24", "mil24"), ("MI28", "mil24"),
        ("S61", "s61"), ("S92", "s61"),
        # ── tiltrotor ──
        ("V22", "v22_fast"),
        # ── gyrocopter ──
        ("GYRO", "gyrocopter"),
    ]
    for prefix, shape in type_mappings:
        lines.append(f'    {{"{prefix.rstrip()}", "{shape}"}},')
    lines.append("};")
    lines.append(f"static constexpr int k_type_map_count = {len(type_mappings)};")
    lines.append("")

    # category fallback mapping
    lines.append("// ADS-B category -> fallback shape")
    lines.append("inline const char* category_to_shape(uint8_t cat) {")
    lines.append("    switch (cat) {")
    lines.append('        case 0xA1: return "cessna";')
    lines.append('        case 0xA2: return "single_turbo";')
    lines.append('        case 0xA3: return "a320";')
    lines.append('        case 0xA4: return "a332";')
    lines.append('        case 0xA5: return "a332";')
    lines.append('        case 0xA6: return "hi_perf";')
    lines.append('        case 0xA7: return "helicopter";')
    lines.append('        case 0xB1: return "glider";')
    lines.append('        case 0xB2: return "balloon";')
    lines.append('        case 0xB4: return "cessna";')
    lines.append('        case 0xB6: return "uav";')
    lines.append('        case 0xC1: case 0xC2: return "ground_square";')
    lines.append('        default:   return "airliner";')
    lines.append("    }")
    lines.append("}")
    lines.append("")

    # unified lookup
    lines.append("// lookup shape by type code (with prefix match) then category fallback")
    lines.append("inline const Shape* lookup(const char* type_code, uint8_t category) {")
    lines.append("    if (type_code && type_code[0] != '\\0') {")
    lines.append("        int len = 0;")
    lines.append("        while (type_code[len] && len < 4) ++len;")
    lines.append("        // try prefix match (longest first)")
    lines.append("        for (int i = 0; i < k_type_map_count; ++i) {")
    lines.append("            const char* p = k_type_map[i].prefix;")
    lines.append("            int plen = 0;")
    lines.append("            while (p[plen]) ++plen;")
    lines.append("            if (plen <= len) {")
    lines.append("                bool match = true;")
    lines.append("                for (int j = 0; j < plen; ++j)")
    lines.append("                    if (type_code[j] != p[j]) { match = false; break; }")
    lines.append("                if (match)")
    lines.append("                    return find_shape(k_type_map[i].shape);")
    lines.append("            }")
    lines.append("        }")
    lines.append("    }")
    lines.append("    return find_shape(category_to_shape(category));")
    lines.append("}")
    lines.append("")
    lines.append("} // namespace aircraft_shapes")
    lines.append("")

    with open(output_path, "w", newline="\n", encoding="utf-8") as f:
        f.write("\n".join(lines))


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    output_path = os.path.join(
        project_root,
        "plugins", "adsb_display", "aircraft_shapes.h"
    )

    js_content = fetch_markers_js()
    print(f"Downloaded {len(js_content)} bytes")

    shapes_raw = extract_shapes(js_content)
    print(f"Extracted {len(shapes_raw)} / {len(SELECTED_SHAPES)} selected shapes")

    missing = set(SELECTED_SHAPES) - set(shapes_raw.keys())
    if missing:
        print(f"  Missing shapes: {missing}")

    shapes_data = {}
    for name, raw in shapes_raw.items():
        try:
            subpaths = parse_svg_path(raw["path"])
        except (ValueError, IndexError) as e:
            print(f"  Skipping {name}: parse error: {e}")
            continue
        # use the largest subpath (main outline)
        all_points = []
        for sp in subpaths:
            pts = subpath_to_points(sp)
            if len(pts) > len(all_points):
                all_points = pts

        normalized, aspect = normalize_points(all_points, raw["viewBox"])
        normalized = deduplicate_close_points(normalized)

        if len(normalized) < 3:
            print(f"  Skipping {name}: only {len(normalized)} points")
            continue

        shapes_data[name] = {"points": normalized, "aspect": aspect}
        print(f"  {name}: {len(normalized)} points, aspect={aspect:.2f}")

    generate_header(shapes_data, output_path)
    print(f"\nGenerated {output_path} with {len(shapes_data)} shapes")

    rc = validate_header(output_path, shapes_data)
    sys.exit(rc)


def validate_header(output_path, shapes_data):
    """Refuse to leave a header whose lookup tables point at absent shapes.

    A k_type_map entry naming a shape that was not generated is worse than no
    entry at all: lookup() prefix-matches it, find_shape() returns nullptr, and
    the aircraft draws the generic fallback — where without the entry it would
    have reached its emitter category and drawn a real silhouette. Seven entries
    were in that state from March until 2026-07-31 because the converter only
    warned about dropped shapes and nobody re-read the output.
    """
    with open(output_path, encoding="utf-8") as f:
        header = f.read()

    have = set(shapes_data)
    problems = []

    type_map = re.search(r"k_type_map\[\]\s*=\s*\{(.*?)\n\};", header, re.S)
    if type_map:
        for shape in sorted(set(re.findall(r',\s*"([a-z0-9_]+)"\s*\}', type_map.group(1)))):
            if shape not in have:
                problems.append(f"k_type_map -> {shape}")

    cat = re.search(r"category_to_shape[^{]*\{(.*?)\n\}", header, re.S)
    if cat:
        for shape in sorted(set(re.findall(r'return\s+"([a-z0-9_]+)"', cat.group(1)))):
            if shape not in have:
                problems.append(f"category_to_shape -> {shape}")

    if problems:
        print("\nERROR: lookup tables reference shapes that were not generated:")
        for p in problems:
            print(f"  {p}")
        print("\nEither the shape failed to convert (check the warnings above) or it\n"
              "should be dropped from the mapping. Leaving it dangling makes those\n"
              "aircraft render worse than having no mapping at all.")
        return 1

    print("Validated: every k_type_map and category_to_shape target resolves.")
    return 0


if __name__ == "__main__":
    main()
