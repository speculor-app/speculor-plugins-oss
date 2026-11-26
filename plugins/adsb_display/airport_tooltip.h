#pragma once

// Click info panel for airports. Anchored to the clicked airport marker,
// shows the same style of panel as aircraft_tooltip but with airport
// fields (name, codes, elevation, coordinates, size / military).

#include "adsb_display_state.h"
#include "unit_format.h"
#include "aircraft_tooltip.h"  // reuse PanelCtx, TT_PAD, TT_OFFSET
#include "country_flags.h"
#include "draw_helpers.h"

#include <spc_ui_theme.h>

#include <algorithm>
#include <cmath>
#include <format>
#include <string>

namespace adsb_airport_tooltip {

inline const char* size_label(AirportFeature::Size s)
{
    switch (s) {
        case AirportFeature::Size::Large:    return "Large airport";
        case AirportFeature::Size::Medium:   return "Medium airport";
        case AirportFeature::Size::Small:    return "Small airport";
        case AirportFeature::Size::Heliport: return "Heliport";
        case AirportFeature::Size::Seaplane: return "Seaplane base";
        default:                             return "Airport";
    }
}

inline void render_airport_tooltip(
    adsb_draw::DrawList& list, int32_t font, int canvas_w, int canvas_h,
    int32_t airport_idx,
    float anchor_x, float anchor_y,
    const std::vector<AirportFeature>& airports,
    const CountryFlagCache& flags,
    float font_size,
    float opacity = 0.92f,
    int32_t unit_system = 0)
{
    if (airport_idx < 0
        || static_cast<size_t>(airport_idx) >= airports.size()) return;
    const auto& a = airports[airport_idx];

    static const spc::ui::Theme theme;

    double fs = static_cast<double>(font_size);
    double fs_big = fs * 1.3;
    int line_h = static_cast<int>(15.0 * (fs / 0.30));
    int line_h_big = static_cast<int>(line_h * 1.2);

    bool has_iata = !a.iata.empty();
    bool has_icao = !a.ident.empty() && a.ident.size() <= 4;
    bool has_name = !a.name.empty();
    bool has_muni = !a.municipality.empty();
    bool has_country = !a.country.empty();
    bool has_elev = a.elevation_ft != 0;

    // ── compute panel height ──────────────────────────────────────────
    int tt_w = static_cast<int>(220.0 * (fs / 0.30));
    int content_h = adsb_tooltip::TT_PAD;
    content_h += line_h_big;                               // header row (IATA / ICAO)
    if (has_name) content_h += line_h;                     // name
    content_h += 6;                                        // separator
    if (has_muni) content_h += line_h;                     // city (country lives in header)
    if (has_elev) content_h += line_h;                     // elevation
    content_h += line_h;                                   // coords
    content_h += 6;                                        // separator
    content_h += line_h;                                   // size / military
    content_h += adsb_tooltip::TT_PAD;
    int tt_h = content_h;

    // ── position, flip near edges ─────────────────────────────────────
    int tt_x = static_cast<int>(anchor_x) + adsb_tooltip::TT_OFFSET;
    int tt_y = static_cast<int>(anchor_y) + adsb_tooltip::TT_OFFSET;
    if (tt_x + tt_w > canvas_w - 5)
        tt_x = static_cast<int>(anchor_x) - adsb_tooltip::TT_OFFSET - tt_w;
    if (tt_y + tt_h > canvas_h - 5)
        tt_y = static_cast<int>(anchor_y) - adsb_tooltip::TT_OFFSET - tt_h;
    tt_x = std::clamp(tt_x, 2, std::max(2, canvas_w - tt_w - 2));
    tt_y = std::clamp(tt_y, 2, std::max(2, canvas_h - tt_h - 2));

    // ── background ────────────────────────────────────────────────────
    adsb_draw::fill_px(list, tt_x, tt_y, tt_x + tt_w - 1, tt_y + tt_h - 1,
                       adsb_draw::rgb(theme.panel_bg, opacity));
    adsb_draw::stroke_px(list, tt_x, tt_y, tt_x + tt_w - 1, tt_y + tt_h - 1,
                         adsb_draw::rgb(theme.surface2), 1);

    // left accent — red for military, yellow otherwise (matches the map
    // marker convention so the panel visually links back to the icon)
    cv::Scalar accent = a.military ? cv::Scalar(230, 60, 60)
                                   : cv::Scalar(255, 200, 40);
    adsb_draw::fill_px(list, tt_x, tt_y, tt_x + 2, tt_y + tt_h - 1, adsb_draw::rgb(accent));

    // ── content ───────────────────────────────────────────────────────
    adsb_tooltip::PanelCtx ctx{list, font, tt_x, tt_w, tt_y + adsb_tooltip::TT_PAD,
                               fs, line_h, theme};

    // Header: IATA (or fallback) big on the left; on the right, when present:
    // [ICAO]  [flag]  [CC]  — matching the aircraft-tooltip convention so
    // country reads at the same position regardless of which panel you're
    // looking at.
    {
        std::string left;
        if (has_iata)      left = a.iata;
        else if (has_icao) left = a.ident;
        else if (has_name) left = a.name.substr(0, 10);
        else               left = "---";

        adsb_draw::text_vcenter(list, tt_x + adsb_tooltip::TT_PAD, ctx.cy, line_h_big, left,
                                adsb_draw::style(font, fs_big, theme.text, true));

        int rx = tt_x + tt_w - adsb_tooltip::TT_PAD;

        // right-to-left: [country name] [flag] [ICAO]
        // Country name resolved from the ISO code via the ADS-B country DB;
        // falls back to the ISO code when the DB doesn't know the country.
        if (has_country && a.country.size() == 2) {
            char cc_upper[3] = {
                static_cast<char>(std::toupper(static_cast<unsigned char>(a.country[0]))),
                static_cast<char>(std::toupper(static_cast<unsigned char>(a.country[1]))),
                '\0'
            };
            const char* cname = country_name_from_iso(cc_upper);
            const char* cc_display = cname ? cname : cc_upper;

            const auto cc_st = adsb_draw::style(font, fs, theme.subtext, true);
            rx -= adsb_tooltip::text_width_px(cc_display, cc_st);
            adsb_draw::text_vcenter(list, rx, ctx.cy, line_h_big, cc_display, cc_st);

            rx -= CountryFlagCache::FLAG_W + 3;
            int fy = ctx.cy + (line_h_big - CountryFlagCache::FLAG_H) / 2;
            if (rx >= tt_x && fy >= tt_y)
                draw_flag(list, font, flags, cc_upper, rx, fy);
            rx -= 6;
        }

        // ICAO (only if IATA already filled the big left-side token)
        if (has_icao && has_iata) {
            const auto ic_st = adsb_draw::style(font, fs, theme.subtext, true);
            rx -= adsb_tooltip::text_width_px(a.ident, ic_st);
            adsb_draw::text_vcenter(list, rx, ctx.cy, line_h_big, a.ident, ic_st);
        }

        ctx.cy += line_h_big;
    }

    if (has_name) {
        // full name dimmed below the header
        adsb_draw::text_vcenter(list, tt_x + adsb_tooltip::TT_PAD, ctx.cy, line_h, a.name,
                                adsb_draw::style(font, fs, theme.subtext, true));
        ctx.cy += line_h;
    }

    ctx.separator();

    // Country moved to the header row with the flag — LOC row now shows
    // just the city (keeping the row skipped entirely when even that is
    // empty, since the header already makes the country clear).
    if (has_muni) {
        ctx.row1("LOC", a.municipality);
    }

    if (has_elev) {
        ctx.row1("ELEV", format_altitude(a.elevation_ft, unit_system));
    }

    ctx.row1("POS", std::format("{:.4f}, {:.4f}", a.lat, a.lon));

    ctx.separator();

    // Size bucket on the left, MIL badge on the right if applicable.
    const char* sz = size_label(a.size);
    adsb_draw::text_vcenter(list, tt_x + adsb_tooltip::TT_PAD, ctx.cy, line_h, sz,
                            adsb_draw::style(font, fs, theme.subtext, true));
    if (a.military) {
        const auto mil_st = adsb_draw::style(font, fs, cv::Scalar(240, 80, 80), true);
        adsb_draw::text_vcenter(list,
            tt_x + tt_w - adsb_tooltip::TT_PAD - adsb_draw::text_width("MILITARY", mil_st),
            ctx.cy, line_h, "MILITARY", mil_st);
    }
    ctx.cy += line_h;
}

} // namespace adsb_airport_tooltip
