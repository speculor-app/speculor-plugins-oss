#pragma once

// Click info panel for airspace polygons. Anchored to whichever polygon the
// user clicked, shows the airspace name, class tag (CTR/TMA/FIR/etc),
// and floor/ceiling altitudes.

#include "adsb_display_state.h"
#include "unit_format.h"
#include "aircraft_tooltip.h"   // reuse PanelCtx, TT_PAD, TT_OFFSET

#include <spc_ui_theme.h>

#include <algorithm>
#include <cstdint>
#include <format>
#include <string>

namespace adsb_airspace_tooltip {

inline const char* class_label(AirspaceFeature::Class k)
{
    switch (k) {
        case AirspaceFeature::Class::Controlled:  return "Controlled";
        case AirspaceFeature::Class::Restricted:  return "Restricted";
        default:                                   return "Other";
    }
}

// Format a ceiling / floor altitude: feet or FL above FL100, "GND" at 0,
// "UNL" when the loader didn't populate a bound (ceiling_ft == 0 on a
// non-ground polygon).
inline std::string format_airspace_alt(int ft, bool is_ceiling, int32_t unit_system)
{
    if (ft <= 0) {
        return is_ceiling ? "UNL" : "GND";
    }
    if (ft >= 18000 && ft % 100 == 0) {
        return std::format("FL{}", ft / 100);
    }
    return format_altitude(ft, unit_system);
}

inline void render_airspace_tooltip(
    cv::Mat& canvas,
    int32_t airspace_idx,
    float anchor_x, float anchor_y,
    const std::vector<AirspaceFeature>& airspaces,
    float font_size,
    float opacity = 0.92f,
    int32_t unit_system = 0)
{
    if (airspace_idx < 0
        || static_cast<size_t>(airspace_idx) >= airspaces.size()) return;
    const auto& a = airspaces[airspace_idx];

    static const spc::ui::Theme theme;

    double fs = static_cast<double>(font_size);
    double fs_big = fs * 1.2;
    int line_h = static_cast<int>(15.0 * (fs / 0.30));
    int line_h_big = static_cast<int>(line_h * 1.2);

    bool has_name = !a.name.empty();
    bool has_type = !a.type_label.empty();

    // ── compute panel height ──────────────────────────────────────────
    int tt_w = static_cast<int>(240.0 * (fs / 0.30));
    int content_h = adsb_tooltip::TT_PAD;
    content_h += line_h_big;       // name (big) + type badge right-aligned
    content_h += 6;                // separator
    content_h += line_h;           // class row
    content_h += line_h;           // altitude range row
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

    // background
    cv::Mat roi = canvas(cv::Rect(tt_x, tt_y, tt_w, tt_h));
    cv::Mat overlay(tt_h, tt_w, CV_8UC3, theme.panel_bg);
    cv::addWeighted(overlay, static_cast<double>(opacity),
                    roi, 1.0 - static_cast<double>(opacity), 0, roi);
    cv::rectangle(canvas, {tt_x, tt_y, tt_w, tt_h}, theme.surface2, 1);

    // left accent — red for restricted/military, blue-gray for controlled,
    // matches the on-map outline color for that class.
    cv::Scalar accent = (a.klass == AirspaceFeature::Class::Restricted)
        ? cv::Scalar(200, 60, 60)
        : cv::Scalar(100, 160, 240);
    cv::rectangle(canvas, {tt_x, tt_y, 3, tt_h}, accent, cv::FILLED);

    adsb_tooltip::PanelCtx ctx{canvas, tt_x, tt_w, tt_y + adsb_tooltip::TT_PAD,
                               fs, line_h, theme};

    // Header: name (big) on the left, type tag on the right.
    {
        std::string left = has_name ? a.name : std::string{"<unnamed airspace>"};
        // Truncate at 40 chars — long FIR / sector names can be unbounded
        if (left.size() > 40) left = left.substr(0, 37) + "...";
        spc::ui::draw_text_vcenter(canvas,
            tt_x + adsb_tooltip::TT_PAD, ctx.cy, line_h_big,
            left.c_str(), theme.text, fs_big);

        if (has_type) {
            auto tsz = spc::ui::measure_text(a.type_label.c_str(), fs);
            spc::ui::draw_text_vcenter(canvas,
                tt_x + tt_w - adsb_tooltip::TT_PAD - tsz.width,
                ctx.cy, line_h_big, a.type_label.c_str(), accent, fs);
        }
        ctx.cy += line_h_big;
    }

    ctx.separator();

    ctx.row1("CLASS", class_label(a.klass));

    auto floor_s = format_airspace_alt(a.floor_ft, false, unit_system);
    auto ceil_s  = format_airspace_alt(a.ceiling_ft, true, unit_system);
    ctx.row1("ALT", std::format("{}  -  {}", floor_s, ceil_s));
}

} // namespace adsb_airspace_tooltip
