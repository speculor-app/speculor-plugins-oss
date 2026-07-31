#pragma once

// click info panel: detailed aircraft information anchored to an aircraft icon

#include "adsb_display_state.h"
#include <spc_ui_theme.h>

#include <chrono>
#include <cmath>
#include <format>

namespace adsb_tooltip {

static constexpr int TT_PAD    = 8;
static constexpr int TT_OFFSET = 15;  // offset from anchor point

// ── signal strength helpers ──────────────────────────────────────────
// all sources output RSSI in dBFS (negative values, 0 = full scale)

// signal level: 0=none, 1=very weak, 2=weak, 3=fair, 4=good, 5=strong
inline int rssi_level(float dbfs)
{
    if (dbfs == 0.0f) return 0;
    if (dbfs >= -3.0f)  return 5;
    if (dbfs >= -10.0f) return 4;
    if (dbfs >= -20.0f) return 3;
    if (dbfs >= -30.0f) return 2;
    return 1;
}

inline cv::Scalar rssi_color(int level)
{
    switch (level) {
        case 5: return {0x70, 0xe3, 0xa6};  // green
        case 4: return {0x70, 0xe3, 0xa6};  // green
        case 3: return {0xf9, 0xe7, 0x72};  // yellow
        case 2: return {0xf9, 0xb8, 0x72};  // peach/orange
        case 1: return {0xf3, 0x6b, 0x62};  // red
        default: return {0xa9, 0xad, 0xa6}; // gray
    }
}

// draw 5-bar signal indicator (like WiFi/cell bars)
inline void draw_signal_bars(cv::Mat& canvas, int x, int y, int h,
                             int level, const cv::Scalar& color)
{
    constexpr int NUM_BARS = 5;
    constexpr int BAR_W = 3;
    constexpr int BAR_GAP = 2;

    for (int i = 0; i < NUM_BARS; ++i) {
        int bar_h = (h * (i + 1)) / NUM_BARS;
        int bx = x + i * (BAR_W + BAR_GAP);
        int by = y + h - bar_h;
        auto c = (i < level) ? color : cv::Scalar(0x45, 0x42, 0x58);  // dim unfilled
        cv::rectangle(canvas, {bx, by, BAR_W, bar_h}, c, cv::FILLED);
    }
}

// ── helpers ───────────────────────────────────────────────────────────

inline const char* type_label(uint8_t category, uint8_t db_flags)
{
    switch (adsb_classify_type(category, db_flags)) {
        case 0:  return "Civilian";
        case 1:  return "Military";
        default: return "Unknown";
    }
}

inline cv::Scalar type_color(uint8_t category, uint8_t db_flags)
{
    static const spc::ui::Theme th;
    switch (adsb_classify_type(category, db_flags)) {
        case 0:  return th.green;
        case 1:  return th.red;
        default: return th.yellow;
    }
}

inline std::string format_squawk(uint16_t squawk)
{
    if (squawk == 0) return "----";
    return std::format("{:04o}", squawk);
}

inline const char* emergency_name(uint8_t code)
{
    switch (code) {
        case 1: return "GENERAL EMERGENCY";
        case 2: return "LIFEGUARD / MEDEVAC";
        case 3: return "MINIMUM FUEL";
        case 4: return "NO COMMUNICATIONS";
        case 5: return "UNLAWFUL INTERFERENCE";
        case 6: return "DOWNED AIRCRAFT";
        default: return nullptr;
    }
}

// ── two-column row helper ─────────────────────────────────────────────

struct PanelCtx
{
    cv::Mat& canvas;
    int x, w;            // panel left edge and width
    int cy;              // current y cursor (advances after each row)
    double fs;           // base font scale
    int line_h;          // row height
    const spc::ui::Theme& th;

    // Offset from a row's left edge to its value, so values line up down the card
    // instead of each starting right after its own label.
    //
    // The shared column is sized for the grid labels only. A longer label (AIRLINE,
    // TYPE) pushes just its own value clear of itself rather than widening — and so
    // shifting right — every numeric row. Sizing the column to the longest label the
    // card can show is what let AIRLINE's value land on top of its label.
    int grid_label_w = -1;
    int value_x_off(const char* label)
    {
        if (grid_label_w < 0) {
            static constexpr const char* kGridLabels[] = {
                "ALT", "VS", "GS", "HDG", "TGT", "ICAO",
                "SQK", "SIG", "TRK", "DST", "BRG", "FROM", "TO",
            };
            grid_label_w = 0;
            for (const char* l : kGridLabels)
                grid_label_w = std::max(grid_label_w, spc::ui::measure_text(l, fs).width);
        }
        // gap scales with the font so it stays readable at any card size
        const int gap = std::max(6, spc::ui::measure_text("00", fs).width);
        return std::max(grid_label_w, spc::ui::measure_text(label, fs).width) + gap;
    }

    // draw a label: value     label2: value2 row
    void row2(const char* l1, const std::string& v1,
              const char* l2, const std::string& v2)
    {
        int half = w / 2;
        auto lc = th.subtext;
        auto vc = th.text;
        // left column
        spc::ui::draw_text_vcenter(canvas, x + TT_PAD, cy, line_h, l1, lc, fs);
        spc::ui::draw_text_vcenter(canvas, x + TT_PAD + value_x_off(l1), cy, line_h,
                                   v1.c_str(), vc, fs);
        // right column
        spc::ui::draw_text_vcenter(canvas, x + half, cy, line_h, l2, lc, fs);
        spc::ui::draw_text_vcenter(canvas, x + half + value_x_off(l2), cy, line_h,
                                   v2.c_str(), vc, fs);
        cy += line_h;
    }

    void row1(const char* l1, const std::string& v1)
    {
        auto lc = th.subtext;
        auto vc = th.text;
        spc::ui::draw_text_vcenter(canvas, x + TT_PAD, cy, line_h, l1, lc, fs);
        spc::ui::draw_text_vcenter(canvas, x + TT_PAD + value_x_off(l1), cy, line_h,
                                   v1.c_str(), vc, fs);
        cy += line_h;
    }

    void separator()
    {
        cy += 2;
        cv::line(canvas, {x + TT_PAD, cy}, {x + w - TT_PAD, cy}, th.surface1, 1);
        cy += 4;
    }
};

// ── rendering ─────────────────────────────────────────────────────────

inline void render_tooltip(
    cv::Mat& canvas,
    uint32_t target_icao,
    float anchor_x, float anchor_y,
    const std::vector<AircraftView>& aircraft,
    CountryFlagCache& flags,
    const std::unordered_map<uint32_t, std::chrono::steady_clock::time_point>& first_seen,
    const std::unordered_map<uint32_t, cv::Mat>& photo_cache,
    float font_size,
    float opacity = 0.92f,
    bool has_gps = false, double gps_lat = 0.0, double gps_lon = 0.0,
    int32_t unit_system = 0)
{
    if (target_icao == 0) return;

    const AircraftView* ac = nullptr;
    for (const auto& a : aircraft) {
        if (a.icao == target_icao) { ac = &a; break; }
    }
    if (!ac) return;

    static const spc::ui::Theme theme;
    auto now = std::chrono::steady_clock::now();

    double fs = static_cast<double>(font_size);
    double fs_big = fs * 1.3;
    int line_h = static_cast<int>(15.0 * (fs / 0.30));
    int line_h_big = static_cast<int>(line_h * 1.2);

    bool is_emergency = ac->emergency > 0 && ac->emergency != 0xFF;
    bool has_ias_tas = ac->ias > 0.1f || ac->tas > 0.1f || ac->mach_num > 0.001f;
    bool has_reg = !ac->registration.empty();
    // Nav-target altitude row appears when the pilot has dialed in a target
    // that differs from current baro alt (>=100 ft tolerance to avoid flicker
    // from rounding near level-off).
    int32_t nav_tgt = ac->nav_alt_mcp > 0 ? ac->nav_alt_mcp :
                      ac->nav_alt_fms > 0 ? ac->nav_alt_fms : 0;
    const char* nav_src = ac->nav_alt_mcp > 0 ? "MCP" :
                          ac->nav_alt_fms > 0 ? "FMS" : "";
    bool has_nav_target = nav_tgt > 0 && std::abs(nav_tgt - ac->alt) >= 100;
    bool has_flags = ac->spi || ac->alert;
    bool has_rssi = ac->rssi < 0.0f;
    bool has_trk_row = first_seen.count(ac->icao) > 0;
    bool has_dist_row = has_gps && ac->lat != 0.0 && ac->lon != 0.0;
    bool has_route = !ac->origin_icao.empty() || !ac->dest_icao.empty();
    bool has_airline = !ac->airline_name.empty();
    bool has_manufacturer = !ac->manufacturer.empty() || !ac->aircraft_type.empty();
    bool has_photo = false;
    const cv::Mat* photo = nullptr;
    {
        auto pit = photo_cache.find(ac->icao);
        if (pit != photo_cache.end() && !pit->second.empty()) {
            photo = &pit->second;
            has_photo = true;
        }
    }

    // ── compute panel height ──────────────────────────────────────────
    int tt_w = static_cast<int>(230.0 * (fs / 0.30));
    int content_h = TT_PAD;
    if (is_emergency) content_h += line_h;           // emergency banner
    content_h += line_h_big;                          // callsign row
    if (has_reg) content_h += line_h;                 // registration below callsign
    content_h += line_h;                              // category + type row
    content_h += 6;                                   // separator
    content_h += line_h * 2;                          // alt/vs + gs/hdg
    if (has_nav_target) content_h += line_h;          // target altitude row
    if (has_ias_tas) content_h += line_h;             // ias/tas/mach
    content_h += 6;                                   // separator
    content_h += line_h;                              // icao + squawk
    if (has_flags) content_h += line_h;               // ident/alert badges
    if (has_rssi || has_trk_row) content_h += line_h;  // signal + tracked time
    if (has_dist_row) content_h += line_h;            // distance + bearing
    if (has_route || has_airline) content_h += 6;     // separator before route
    if (has_airline) content_h += line_h;             // airline
    if (has_route) content_h += line_h;                  // origin + destination (two columns)
    if (has_manufacturer) content_h += line_h;        // manufacturer + type
    if (has_photo) content_h += 6 + photo->rows + 4; // separator + photo + pad
    if (ac->on_ground) content_h += line_h;           // ground status
    content_h += TT_PAD;
    int tt_h = content_h;

    // ── position: offset from anchor, flip if near edge ───────────────
    int tt_x = static_cast<int>(anchor_x) + TT_OFFSET;
    int tt_y = static_cast<int>(anchor_y) + TT_OFFSET;

    if (tt_x + tt_w > canvas.cols - 5)
        tt_x = static_cast<int>(anchor_x) - TT_OFFSET - tt_w;
    if (tt_y + tt_h > canvas.rows - 5)
        tt_y = static_cast<int>(anchor_y) - TT_OFFSET - tt_h;

    tt_x = std::clamp(tt_x, 2, std::max(2, canvas.cols - tt_w - 2));
    tt_y = std::clamp(tt_y, 2, std::max(2, canvas.rows - tt_h - 2));

    // ── draw background ───────────────────────────────────────────────
    cv::Mat roi = canvas(cv::Rect(tt_x, tt_y, tt_w, tt_h));
    cv::Mat overlay(tt_h, tt_w, CV_8UC3, theme.panel_bg);
    cv::addWeighted(overlay, static_cast<double>(opacity), roi, 1.0 - static_cast<double>(opacity), 0, roi);

    // border: red 2px for emergency, normal 1px otherwise
    if (is_emergency)
        cv::rectangle(canvas, {tt_x, tt_y, tt_w, tt_h}, theme.red, 2);
    else
        cv::rectangle(canvas, {tt_x, tt_y, tt_w, tt_h}, theme.surface2, 1);

    // accent bar on the left (skip for emergency — red border is enough)
    if (!is_emergency) {
        auto accent = type_color(ac->category, ac->db_flags);
        cv::rectangle(canvas, {tt_x, tt_y, 3, tt_h}, accent, cv::FILLED);
    }

    // ── draw content ──────────────────────────────────────────────────
    PanelCtx ctx{canvas, tt_x, tt_w, tt_y + TT_PAD, fs, line_h, theme};

    // emergency banner
    if (is_emergency) {
        auto* ename = emergency_name(ac->emergency);
        if (ename) {
            spc::ui::draw_text_vcenter(canvas, tt_x + TT_PAD, ctx.cy, line_h,
                                       ename, theme.red, fs);
        }
        ctx.cy += line_h;
    }

    // ── header: callsign (big) + type code + flag right-aligned ───────
    {
        // callsign or ICAO hex as fallback
        std::string cs = ac->callsign.empty()
            ? std::format("{:06X}", ac->icao) : ac->callsign;
        spc::ui::draw_text_vcenter(canvas, tt_x + TT_PAD, ctx.cy, line_h_big,
                                   cs.c_str(), theme.text, fs_big);

        // right side: [type_code] [flag] CC
        int rx = tt_x + tt_w - TT_PAD;

        // country code text
        if (ac->country_code) {
            char cc_upper[3] = {
                static_cast<char>(std::toupper(static_cast<unsigned char>(ac->country_code[0]))),
                static_cast<char>(std::toupper(static_cast<unsigned char>(ac->country_code[1]))),
                '\0'
            };
            auto cc_sz = spc::ui::measure_text(cc_upper, fs);
            rx -= cc_sz.width;
            spc::ui::draw_text_vcenter(canvas, rx, ctx.cy, line_h_big,
                                       cc_upper, theme.subtext, fs);

            // flag image
            const auto& flag_img = flags.get(ac->country_code);
            rx -= flag_img.cols + 3;
            int fy = ctx.cy + (line_h_big - flag_img.rows) / 2;
            if (rx >= tt_x && fy >= tt_y &&
                rx + flag_img.cols <= canvas.cols && fy + flag_img.rows <= canvas.rows) {
                flag_img.copyTo(canvas(cv::Rect(rx, fy, flag_img.cols, flag_img.rows)));
            }
            rx -= 4;
        }

        // type code (e.g., "A333")
        if (!ac->type_code_str.empty()) {
            auto tc_sz = spc::ui::measure_text(ac->type_code_str.c_str(), fs);
            rx -= tc_sz.width;
            spc::ui::draw_text_vcenter(canvas, rx, ctx.cy, line_h_big,
                                       ac->type_code_str.c_str(), theme.blue, fs);
        }

        ctx.cy += line_h_big;
    }

    // ── registration (below callsign, smaller + dimmer) ──────────────
    if (has_reg) {
        spc::ui::draw_text_vcenter(canvas, tt_x + TT_PAD, ctx.cy, line_h,
                                   ac->registration.c_str(), theme.subtext, fs);
        ctx.cy += line_h;
    }

    // ── category description + civilian/military ──────────────────────
    {
        auto* cat_desc = adsb_category_description(ac->category);
        std::string left = (cat_desc && cat_desc[0] != '\0') ? cat_desc : "";
        const char* tl = type_label(ac->category, ac->db_flags);
        auto tc = type_color(ac->category, ac->db_flags);

        if (!left.empty())
            spc::ui::draw_text_vcenter(canvas, tt_x + TT_PAD, ctx.cy, line_h,
                                       left.c_str(), theme.subtext, fs);

        // right-align type label
        auto tl_sz = spc::ui::measure_text(tl, fs);
        spc::ui::draw_text_vcenter(canvas, tt_x + tt_w - TT_PAD - tl_sz.width,
                                   ctx.cy, line_h, tl, tc, fs);
        ctx.cy += line_h;
    }

    // ── separator ─────────────────────────────────────────────────────
    ctx.separator();

    // ── flight data ───────────────────────────────────────────────────
    {
        std::string alt_v = ac->on_ground ? "GND" :
            (ac->alt != 0) ? format_altitude(ac->alt, unit_system) : "---";
        std::string vs_v;
        if (ac->baro_rate != 0)
            vs_v = format_vspeed(ac->baro_rate, unit_system);
        else if (ac->geom_rate != 0)
            vs_v = format_vspeed(ac->geom_rate, unit_system);
        else
            vs_v = "---";
        ctx.row2("ALT", alt_v, "VS", vs_v);

        std::string gs_v = (ac->gs > 0.1f) ? format_speed(ac->gs, unit_system) : "---";
        float hdg = (ac->true_heading > 0.1f) ? ac->true_heading :
                    (ac->mag_heading > 0.1f) ? ac->mag_heading : ac->track;
        std::string hdg_v = (hdg > 0.1f) ? std::format("{:.0f}deg", hdg) : "---";
        ctx.row2("GS", gs_v, "HDG", hdg_v);

        if (has_nav_target) {
            auto tgt_v = std::format("{} ({})",
                                     format_altitude(nav_tgt, unit_system), nav_src);
            ctx.row2("TGT", tgt_v, "", "");
        }
    }

    // IAS/TAS/Mach (only if available)
    if (has_ias_tas) {
        std::string ias_s = (ac->ias > 0.1f)
            ? std::format("{:.0f}", unit_system == 1 ? kts_to_kmh(ac->ias) : ac->ias)
            : "---";
        std::string tas_s = (ac->tas > 0.1f)
            ? std::format("{:.0f}", unit_system == 1 ? kts_to_kmh(ac->tas) : ac->tas)
            : "---";
        std::string mach_s = (ac->mach_num > 0.001f) ? std::format("{:.2f}", ac->mach_num) : "---";
        auto combined = std::format("IAS {}  TAS {}  M {}", ias_s, tas_s, mach_s);
        spc::ui::draw_text_vcenter(canvas, tt_x + TT_PAD, ctx.cy, line_h,
                                   combined.c_str(), theme.subtext, fs);
        ctx.cy += line_h;
    }

    // ── separator ─────────────────────────────────────────────────────
    ctx.separator();

    // ── identifiers ───────────────────────────────────────────────────
    ctx.row2("ICAO", std::format("{:06X}", ac->icao), "SQK", format_squawk(ac->squawk));

    // Cockpit indicator flags (SPI = ident pressed, Alert = mode-S alert).
    // Colored so they stand out from the neutral rows above.
    if (has_flags) {
        std::string tokens;
        if (ac->spi)   tokens += "IDENT ";
        if (ac->alert) tokens += "ALERT";
        cv::Scalar flag_col = ac->alert ? cv::Scalar(240, 80, 80)
                                        : cv::Scalar(240, 200, 60);
        spc::ui::draw_text_vcenter(canvas, tt_x + TT_PAD, ctx.cy, line_h,
                                   tokens.c_str(), flag_col, fs);
        ctx.cy += line_h;
    }

    // signal strength + tracked time (shared row)
    if (has_rssi || has_trk_row) {
        // left: signal bars + dB
        if (has_rssi) {
            float db = ac->rssi;
            int level = rssi_level(ac->rssi);
            auto color = rssi_color(level);

            spc::ui::draw_text_vcenter(canvas, tt_x + TT_PAD, ctx.cy, line_h,
                                       "SIG", theme.subtext, fs);
            // same value column as the rows above/below, not "SIG" plus its own width
            int bar_x = tt_x + TT_PAD + ctx.value_x_off("SIG");
            int bar_h = line_h - 4;
            draw_signal_bars(canvas, bar_x, ctx.cy + 2, bar_h, level, color);

            auto db_str = std::format("{:.0f} dB", db);
            int bars_w = 5 * 3 + 4 * 2;  // 5 bars * width + 4 gaps
            spc::ui::draw_text_vcenter(canvas, bar_x + bars_w + 4, ctx.cy, line_h,
                                       db_str.c_str(), color, fs);
        }

        // right: tracked time
        if (has_trk_row) {
            std::string trk_v = "---";
            auto it = first_seen.find(ac->icao);
            if (it != first_seen.end()) {
                auto secs = std::chrono::duration_cast<std::chrono::seconds>(now - it->second).count();
                if (secs < 60)
                    trk_v = std::format("{}s", secs);
                else if (secs < 3600)
                    trk_v = std::format("{}m {}s", secs / 60, secs % 60);
                else
                    trk_v = std::format("{}h {}m", secs / 3600, (secs % 3600) / 60);
            }
            int half = tt_w / 2;
            spc::ui::draw_text_vcenter(canvas, tt_x + half, ctx.cy, line_h,
                                       "TRK", theme.subtext, fs);
            spc::ui::draw_text_vcenter(canvas, tt_x + half + ctx.value_x_off("TRK"),
                                       ctx.cy, line_h, trk_v.c_str(), theme.text, fs);
        }

        ctx.cy += line_h;
    }

    // distance + bearing from GPS
    if (has_dist_row) {
        double dist_km = haversine_km(gps_lat, gps_lon, ac->lat, ac->lon);
        double brg = bearing_deg(gps_lat, gps_lon, ac->lat, ac->lon);
        std::string dist_v = (dist_km < 1.0)
            ? std::format("{:.0f} m", dist_km * 1000.0)
            : std::format("{:.1f} km", dist_km);
        std::string brg_v = std::format("{:.0f}deg", brg);
        ctx.row2("DST", dist_v, "BRG", brg_v);
    }

    // ── route & airline info ──────────────────────────────────────────
    if (has_route || has_airline) ctx.separator();

    if (has_airline) {
        std::string airline_v = ac->airline_name;
        if (!ac->airline_icao.empty())
            airline_v += " (" + ac->airline_icao + ")";
        ctx.row1("AIRLINE", airline_v);
    }

    if (has_route) {
        // compact format: "BCN Barcelona" (IATA + city)
        auto fmt_airport = [](const std::string& icao, const std::string& iata,
                              const std::string& city) -> std::string {
            if (icao.empty() && iata.empty()) return "---";
            std::string code = !iata.empty() ? iata : icao;
            if (!city.empty()) code += " " + city;
            return code;
        };
        ctx.row2("FROM", fmt_airport(ac->origin_icao, ac->origin_iata, ac->origin_name),
                 "TO",   fmt_airport(ac->dest_icao, ac->dest_iata, ac->dest_name));
    }

    if (has_manufacturer) {
        std::string mfr = ac->manufacturer;
        if (!ac->aircraft_type.empty()) {
            if (!mfr.empty()) mfr += " ";
            mfr += ac->aircraft_type;
        }
        ctx.row1("TYPE", mfr);
    }

    // ── photo thumbnail ─────────────────────────────────────────────
    if (has_photo && photo) {
        ctx.separator();
        // center the thumbnail horizontally
        int px = tt_x + (tt_w - photo->cols) / 2;
        int py = ctx.cy;
        if (px >= 0 && py >= 0 &&
            px + photo->cols <= canvas.cols && py + photo->rows <= canvas.rows) {
            photo->copyTo(canvas(cv::Rect(px, py, photo->cols, photo->rows)));
        }
        ctx.cy += photo->rows + 4;
    }

    // ground status (only when on ground)
    if (ac->on_ground) {
        spc::ui::draw_text_vcenter(canvas, tt_x + TT_PAD, ctx.cy, line_h,
                                   "ON GROUND", theme.yellow, fs);
        ctx.cy += line_h;
    }
}

} // namespace adsb_tooltip
