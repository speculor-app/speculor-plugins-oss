#pragma once

// Color mapping used by on-map rendering: the tar1090-style altitude
// gradient and the map-style-aware contrast/halo pair. Header-only.

#include <spc_map_tiles.h>

#include <opencv2/core.hpp>
#include <cstdint>

// ── map-style presets ─────────────────────────────────────────────────
// Defined in the shared tile engine so every map plugin offers the same styles.
// Pulled in unqualified here for the call sites that predate the move.

using spc::map::MAP_STYLE_STANDARD;
using spc::map::MAP_STYLE_HUMANITARIAN;
using spc::map::MAP_STYLE_DARK;
using spc::map::MAP_STYLE_LIGHT;
using spc::map::MAP_STYLE_VOYAGER;
using spc::map::MAP_STYLE_TOPO;
using spc::map::MAP_STYLE_CUSTOM;
using spc::map::MAP_STYLE_COUNT;
using spc::map::k_tile_servers;
using spc::map::resolve_tile_server;
using spc::map::is_dark_map;
using spc::map::map_contrast_color;

// halo color behind outlines/labels (opposite of contrast color).
// On uniform maps the halo blends with the background; on mixed maps
// (topo) both layers are visible where needed.
inline cv::Scalar map_halo_color(int32_t style)
{
    return is_dark_map(style)
        ? cv::Scalar(0, 0, 0)           // dark halo behind white outline
        : cv::Scalar(255, 255, 255);    // white halo behind dark outline
}

// ── altitude gradient (tar1090 convention) ───────────────────────────

// Altitude-based color: warm colors near the ground draw the eye to
// low/relevant traffic, cooler colors identify cruise-altitude airliners.
// Continuous linear interpolation across 6 stops keeps altitude trails
// free of stair-steps. Canvas uses RGB order.
inline cv::Scalar altitude_color(int32_t alt_ft)
{
    struct Stop { int ft; uint8_t r, g, b; };
    static constexpr Stop stops[] = {
        {    0,   0, 200,   0},   // ground: green
        { 5000, 255, 230,   0},   // low:    yellow
        {10000, 255, 140,   0},   // mid:    orange
        {20000, 240,  50,  50},   // upper:  red
        {30000, 200,  60, 200},   // high:   magenta
        {40000,  80, 180, 255},   // cruise: cyan/blue
    };
    constexpr int n = std::size(stops);

    if (alt_ft <= stops[0].ft)
        return {double(stops[0].r), double(stops[0].g), double(stops[0].b)};
    if (alt_ft >= stops[n-1].ft)
        return {double(stops[n-1].r), double(stops[n-1].g), double(stops[n-1].b)};

    int i = 0;
    while (i + 1 < n && alt_ft >= stops[i + 1].ft) ++i;
    double t = double(alt_ft - stops[i].ft) / double(stops[i + 1].ft - stops[i].ft);
    double r = stops[i].r + t * (stops[i + 1].r - stops[i].r);
    double g = stops[i].g + t * (stops[i + 1].g - stops[i].g);
    double b = stops[i].b + t * (stops[i + 1].b - stops[i].b);
    return {r, g, b};
}
