#pragma once

// Click info panel for user-placed waypoints. Anchored to the waypoint
// marker; shows name, type, lat/lon, and altitude. Mirrors the layout
// conventions of airport_tooltip / airspace_tooltip so all click panels
// feel uniform.

#include "adsb_display_state.h"
#include "unit_format.h"
#include "aircraft_tooltip.h"   // reuse PanelCtx, TT_PAD, TT_OFFSET

#include <spc_ui_theme.h>

#include <algorithm>
#include <cstdint>
#include <format>
#include <opencv2/imgproc.hpp>
#include <string>

namespace adsb_waypoint_tooltip {

inline const char* type_label(int32_t type)
{
    switch (type) {
        case 0:  return "Waypoint";
        case 1:  return "Airport";
        case 2:  return "Military Base";
        case 3:  return "VOR";
        case 4:  return "NDB";
        default: return "Waypoint";
    }
}

// Match the on-map marker color so the panel reads as "this thing you clicked".
inline cv::Scalar accent_for(int32_t type, int32_t map_style)
{
    switch (type) {
        case 1:  return {50, 180, 255};   // airport
        case 2:  return {255, 60, 60};    // military
        case 3:  return {50, 200, 50};    // VOR
        case 4:  return {255, 180, 50};   // NDB
        default: return map_contrast_color(map_style);  // waypoint
    }
}

inline void render_waypoint_tooltip(
    cv::Mat& canvas,
    int32_t waypoint_idx,
    float anchor_x, float anchor_y,
    const std::vector<MapDisplayState::Waypoint>& waypoints,
    int32_t map_style,
    float font_size,
    float opacity = 0.92f,
    int32_t unit_system = 0)
{
    if (waypoint_idx < 0
        || static_cast<size_t>(waypoint_idx) >= waypoints.size()) return;
    const auto& wp = waypoints[waypoint_idx];

    static const spc::ui::Theme theme;

    double fs = static_cast<double>(font_size);
    double fs_big = fs * 1.3;
    int line_h = static_cast<int>(15.0 * (fs / 0.30));
    int line_h_big = static_cast<int>(line_h * 1.2);

    bool has_name = wp.name[0] != '\0';
    bool has_alt  = wp.alt != 0.0;

    int tt_w = static_cast<int>(220.0 * (fs / 0.30));
    int content_h = adsb_tooltip::TT_PAD;
    content_h += line_h_big;        // name (big) + type tag
    content_h += 6;                 // separator
    content_h += line_h;            // POS
    if (has_alt) content_h += line_h;
    content_h += adsb_tooltip::TT_PAD;
    int tt_h = content_h;

    int tt_x = static_cast<int>(anchor_x) + adsb_tooltip::TT_OFFSET;
    int tt_y = static_cast<int>(anchor_y) + adsb_tooltip::TT_OFFSET;
    if (tt_x + tt_w > canvas.cols - 5)
        tt_x = static_cast<int>(anchor_x) - adsb_tooltip::TT_OFFSET - tt_w;
    if (tt_y + tt_h > canvas.rows - 5)
        tt_y = static_cast<int>(anchor_y) - adsb_tooltip::TT_OFFSET - tt_h;
    tt_x = std::clamp(tt_x, 2, std::max(2, canvas.cols - tt_w - 2));
    tt_y = std::clamp(tt_y, 2, std::max(2, canvas.rows - tt_h - 2));

    cv::Mat roi = canvas(cv::Rect(tt_x, tt_y, tt_w, tt_h));
    cv::Mat overlay(tt_h, tt_w, CV_8UC3, theme.panel_bg);
    cv::addWeighted(overlay, static_cast<double>(opacity),
                    roi, 1.0 - static_cast<double>(opacity), 0, roi);
    cv::rectangle(canvas, {tt_x, tt_y, tt_w, tt_h}, theme.surface2, 1);

    cv::Scalar accent = accent_for(wp.type, map_style);
    cv::rectangle(canvas, {tt_x, tt_y, 3, tt_h}, accent, cv::FILLED);

    adsb_tooltip::PanelCtx ctx{canvas, tt_x, tt_w, tt_y + adsb_tooltip::TT_PAD,
                               fs, line_h, theme};

    // Header: name (big) on the left, type tag right-aligned in accent color.
    {
        std::string left = has_name ? std::string{wp.name} : std::string{"<unnamed>"};
        if (left.size() > 24) left = left.substr(0, 21) + "...";
        spc::ui::draw_text_vcenter(canvas,
            tt_x + adsb_tooltip::TT_PAD, ctx.cy, line_h_big,
            left.c_str(), theme.text, fs_big);

        const char* tl = type_label(wp.type);
        auto tl_sz = spc::ui::measure_text(tl, fs);
        spc::ui::draw_text_vcenter(canvas,
            tt_x + tt_w - adsb_tooltip::TT_PAD - tl_sz.width,
            ctx.cy, line_h_big, tl, accent, fs);
        ctx.cy += line_h_big;
    }

    ctx.separator();

    ctx.row1("POS", std::format("{:.4f}, {:.4f}", wp.lat, wp.lon));

    if (has_alt) {
        ctx.row1("ALT", format_altitude(static_cast<int32_t>(wp.alt), unit_system));
    }
}

} // namespace adsb_waypoint_tooltip
