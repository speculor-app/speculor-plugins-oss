#pragma once

// aircraft list panel: scrollable, sortable table rendered on the right side of the map

#include "adsb_display_state.h"
#include "aircraft_tooltip.h"
#include <spc_ui_theme.h>

#include <algorithm>
#include <chrono>
#include <format>

namespace adsb_list {

// ── layout constants ──────────────────────────────────────────────────

static constexpr int HEADER_H    = 22;
static constexpr int ROW_H       = 28;   // taller rows for two-line callsign+reg
static constexpr int PAD_X       = 4;
static constexpr int STATUS_BAR  = 20;  // bottom status bar
static constexpr double FONT     = 0.28;
static constexpr double FONT_SM  = 0.24;  // smaller font for registration line
static constexpr double FONT_HDR = 0.30;

// column definitions
struct ColDef { const char* name; int width; };
static constexpr ColDef k_columns[] = {
    {"",    30},   // 0: flag
    {"CS",  50},   // 1: callsign + registration
    {"RTE", 30},   // 2: route (origin→dest)
    {"S",   20},   // 3: status dot
    {"ALT", 60},   // 4: altitude
    {"SPD", 40},   // 5: speed
    {"SIG", 30},   // 6: signal strength
};
static constexpr int NUM_COLS = sizeof(k_columns) / sizeof(k_columns[0]);

// sort column indices
static constexpr int SORT_COUNTRY  = 0;
static constexpr int SORT_CALLSIGN = 1;
static constexpr int SORT_ROUTE    = 2;
static constexpr int SORT_STATUS   = 3;
static constexpr int SORT_ALT      = 4;
static constexpr int SORT_SPEED    = 5;
static constexpr int SORT_SIGNAL   = 6;

// ── helpers ───────────────────────────────────────────────────────────

// status: 0=airborne, 1=ground, 2=stale(>10s), 3=lost(>30s)
inline int aircraft_status(const AircraftView& ac)
{
    if (ac.seen > 30.0f) return 3;
    if (ac.seen > 10.0f) return 2;
    if (ac.on_ground) return 1;
    return 0;
}

inline cv::Scalar status_color(int status)
{
    switch (status) {
        case 0: return {0xa6, 0xe3, 0x70};  // green — airborne
        case 1: return {0xa6, 0xad, 0xa9};  // gray — ground
        case 2: return {0xf9, 0xe7, 0x72};  // yellow — stale
        case 3: return {0xf3, 0x6b, 0x62};  // red — lost
        default: return {0xa6, 0xad, 0xa9};
    }
}

// ── sorting ───────────────────────────────────────────────────────────

inline void sort_aircraft(std::vector<AircraftView>& aircraft,
                          const ListPanelState& state)
{
    auto cmp = [&](const AircraftView& a, const AircraftView& b) -> bool {
        int result = 0;
        switch (state.sort_column) {
            case SORT_COUNTRY: {
                const char* ca = a.country_code ? a.country_code : "";
                const char* cb = b.country_code ? b.country_code : "";
                result = std::strcmp(ca, cb);
                break;
            }
            case SORT_CALLSIGN:
                result = a.callsign.compare(b.callsign);
                break;
            case SORT_ROUTE: {
                auto ra = a.origin_icao + a.dest_icao;
                auto rb = b.origin_icao + b.dest_icao;
                result = ra.compare(rb);
                break;
            }
            case SORT_STATUS:
                result = aircraft_status(a) - aircraft_status(b);
                break;
            case SORT_ALT:
                result = (a.alt < b.alt) ? -1 : (a.alt > b.alt) ? 1 : 0;
                break;
            case SORT_SPEED:
                result = (a.gs < b.gs) ? -1 : (a.gs > b.gs) ? 1 : 0;
                break;
            case SORT_SIGNAL:
                result = (a.rssi < b.rssi) ? -1 : (a.rssi > b.rssi) ? 1 : 0;
                break;
            default:
                result = (a.icao < b.icao) ? -1 : (a.icao > b.icao) ? 1 : 0;
                break;
        }
        return state.sort_ascending ? (result < 0) : (result > 0);
    };
    std::sort(aircraft.begin(), aircraft.end(), cmp);
}

// ── rendering ─────────────────────────────────────────────────────────

inline void render_aircraft_list(
    cv::Mat& canvas,
    int panel_x, int panel_y, int panel_w, int panel_h,
    std::vector<AircraftView>& aircraft,
    ListPanelState& state,
    CountryFlagCache& flags,
    const std::unordered_map<uint32_t, std::chrono::steady_clock::time_point>& /*first_seen*/,
    std::chrono::steady_clock::time_point /*now*/,
    int32_t unit_system = 0)
{
    if (panel_w < 100 || panel_h < HEADER_H + ROW_H) return;

    static const spc::ui::Theme theme;

    // panel background
    cv::Mat panel_roi = canvas(cv::Rect(panel_x, panel_y, panel_w, panel_h));
    panel_roi.setTo(theme.panel_bg);

    // vertical separator line
    cv::line(canvas, {panel_x, panel_y}, {panel_x, panel_y + panel_h},
             theme.surface1, 1);

    // sort aircraft
    sort_aircraft(aircraft, state);

    // compute column positions (scale to fit panel width)
    int total_def_w = 0;
    for (int i = 0; i < NUM_COLS; ++i) total_def_w += k_columns[i].width;
    float scale = static_cast<float>(panel_w - PAD_X * 2) / static_cast<float>(total_def_w);
    if (scale > 1.5f) scale = 1.5f;

    int col_x[NUM_COLS + 1];
    col_x[0] = panel_x + PAD_X;
    for (int i = 0; i < NUM_COLS; ++i)
        col_x[i + 1] = col_x[i] + static_cast<int>(k_columns[i].width * scale);

    // ── header row ────────────────────────────────────────────────────
    int hdr_y = panel_y;
    cv::rectangle(canvas, {panel_x, hdr_y, panel_w, HEADER_H},
                  theme.surface1, cv::FILLED);

    for (int i = 0; i < NUM_COLS; ++i) {
        if (k_columns[i].name[0] == '\0') continue;
        auto col = (state.sort_column == i) ? theme.blue : theme.subtext;
        spc::ui::draw_text_vcenter(canvas, col_x[i], hdr_y, HEADER_H,
                                   k_columns[i].name, col, FONT_HDR);

        // sort indicator arrow
        if (state.sort_column == i) {
            int ax = col_x[i] + static_cast<int>(k_columns[i].width * scale) - 8;
            int ay = hdr_y + HEADER_H / 2;
            if (state.sort_ascending) {
                cv::Point pts[3] = {{ax, ay + 3}, {ax + 4, ay - 3}, {ax + 8, ay + 3}};
                cv::fillConvexPoly(canvas, pts, 3, theme.blue);
            } else {
                cv::Point pts[3] = {{ax, ay - 3}, {ax + 4, ay + 3}, {ax + 8, ay - 3}};
                cv::fillConvexPoly(canvas, pts, 3, theme.blue);
            }
        }
    }

    // ── data rows ─────────────────────────────────────────────────────
    int data_y = hdr_y + HEADER_H;
    int data_h = panel_h - HEADER_H - STATUS_BAR;
    int visible_rows = data_h / ROW_H;
    int total_rows = static_cast<int>(aircraft.size());

    // clamp scroll
    int max_scroll = std::max(0, total_rows - visible_rows);
    state.scroll_offset = std::clamp(state.scroll_offset, 0, max_scroll);

    for (int row = 0; row < visible_rows; ++row) {
        int idx = state.scroll_offset + row;
        if (idx >= total_rows) break;

        const auto& ac = aircraft[idx];
        int ry = data_y + row * ROW_H;

        // row background (alternating + selection + hover)
        cv::Scalar row_bg;
        if (ac.icao == state.selected_icao)
            row_bg = theme.surface1;
        else if (ac.icao == state.hovered_row_icao)
            row_bg = cv::Scalar(0x38, 0x35, 0x40);
        else if (row % 2 == 0)
            row_bg = theme.panel_bg;
        else
            row_bg = cv::Scalar(0x2e, 0x28, 0x38);

        cv::rectangle(canvas, {panel_x + 1, ry, panel_w - 1, ROW_H}, row_bg, cv::FILLED);

        auto text_col = (ac.icao == state.selected_icao) ? theme.text : theme.subtext;

        // col 0: flag
        if (ac.country_code) {
            const auto& flag_img = flags.get(ac.country_code);
            int fx = col_x[0];
            int fy = ry + (ROW_H - CountryFlagCache::FLAG_H) / 2;
            if (fx >= 0 && fy >= 0 &&
                fx + flag_img.cols <= canvas.cols && fy + flag_img.rows <= canvas.rows) {
                flag_img.copyTo(canvas(cv::Rect(fx, fy, flag_img.cols, flag_img.rows)));
            }
        }

        // col 1: callsign-or-ICAO (top line) + registration (bottom line,
        // dimmer). Top falls back to the ICAO hex when callsign is empty
        // so the row never renders blank. Registration stays on the bottom
        // line independent of what's above it.
        {
            std::string top = !ac.callsign.empty()
                ? ac.callsign
                : std::format("{:06X}", ac.icao);
            int cs_y = ry + 2;
            spc::ui::draw_text_vcenter(canvas, col_x[1], cs_y, ROW_H / 2,
                                       top.c_str(), text_col, FONT);

            if (!ac.registration.empty()) {
                auto reg_col = theme.subtext;
                int reg_y = ry + ROW_H / 2 - 1;
                spc::ui::draw_text_vcenter(canvas, col_x[1], reg_y, ROW_H / 2,
                                           ac.registration.c_str(), reg_col, FONT_SM);
            }
        }

        // col 2: route (origin / dest) on two lines, prefer IATA codes
        {
            if (!ac.origin_icao.empty() || !ac.dest_icao.empty()) {
                auto route_col = text_col;
                int top_y = ry + 2;
                int bot_y = ry + ROW_H / 2 - 1;
                auto orig = !ac.origin_iata.empty() ? ac.origin_iata : ac.origin_icao;
                auto dest = !ac.dest_iata.empty() ? ac.dest_iata : ac.dest_icao;
                if (!orig.empty())
                    spc::ui::draw_text_vcenter(canvas, col_x[2], top_y, ROW_H / 2,
                                               orig.c_str(), route_col, FONT_SM);
                if (!dest.empty())
                    spc::ui::draw_text_vcenter(canvas, col_x[2], bot_y, ROW_H / 2,
                                               dest.c_str(), route_col, FONT_SM);
            }
        }

        // col 3: status dot
        {
            int st = aircraft_status(ac);
            auto sc = status_color(st);
            int cx = col_x[3] + 7;
            int cy = ry + ROW_H / 2;
            cv::circle(canvas, {cx, cy}, 4, sc, cv::FILLED, cv::LINE_AA);
        }

        // col 4: altitude
        {
            std::string alt_str;
            if (ac.on_ground)
                alt_str = "GND";
            else if (ac.alt != 0)
                alt_str = format_altitude_value(ac.alt, unit_system);
            else
                alt_str = "---";
            spc::ui::draw_text_vcenter(canvas, col_x[4], ry, ROW_H,
                                       alt_str.c_str(), text_col, FONT);
        }

        // col 5: speed
        {
            std::string spd_str;
            if (ac.gs > 0.1f)
                spd_str = format_speed_value(ac.gs, unit_system);
            else
                spd_str = "---";
            spc::ui::draw_text_vcenter(canvas, col_x[5], ry, ROW_H,
                                       spd_str.c_str(), text_col, FONT);
        }

        // col 6: signal strength bars
        {
            int level = adsb_tooltip::rssi_level(ac.rssi);
            auto color = adsb_tooltip::rssi_color(level);
            int bar_h = ROW_H - 10;
            adsb_tooltip::draw_signal_bars(canvas, col_x[6] + 2, ry + 5, bar_h, level, color);
        }
    }

    // ── scroll indicator ──────────────────────────────────────────────
    if (total_rows > visible_rows) {
        int track_y = data_y;
        int track_h = data_h;
        float thumb_frac = static_cast<float>(visible_rows) / total_rows;
        int thumb_h = std::max(8, static_cast<int>(track_h * thumb_frac));
        float scroll_frac = (max_scroll > 0) ?
            static_cast<float>(state.scroll_offset) / max_scroll : 0.0f;
        int thumb_y = track_y + static_cast<int>((track_h - thumb_h) * scroll_frac);
        int bar_x = panel_x + panel_w - 4;
        cv::rectangle(canvas, {bar_x, thumb_y, 3, thumb_h},
                      theme.surface2, cv::FILLED);
    }

    // ── bottom status bar ─────────────────────────────────────────────
    int bar_y = panel_y + panel_h - STATUS_BAR;
    cv::rectangle(canvas, {panel_x, bar_y, panel_w, STATUS_BAR},
                  theme.surface1, cv::FILLED);
    auto count_str = std::format(" {} aircraft", total_rows);
    spc::ui::draw_text_vcenter(canvas, panel_x + PAD_X, bar_y, STATUS_BAR,
                               count_str.c_str(), theme.subtext, FONT);
}

// ── event handling ────────────────────────────────────────────────────

// returns 0 if event consumed, -1 if not
// if a row is clicked, sets out_lat/out_lon to center on that aircraft
inline int handle_list_event(
    const SpcInputEvent* event,
    int panel_x, int panel_y, int panel_w, int panel_h,
    int canvas_w, int canvas_h,
    ListPanelState& state,
    const std::vector<AircraftView>& aircraft,
    double* out_lat, double* out_lon)
{
    // convert normalized coords to pixel coords
    float mx = event->mouse.x * canvas_w;
    float my = event->mouse.y * canvas_h;

    // check if cursor is inside the panel
    if (mx < panel_x || mx >= panel_x + panel_w ||
        my < panel_y || my >= panel_y + panel_h)
        return -1;

    int data_h = panel_h - HEADER_H - STATUS_BAR;
    int visible_rows = data_h / ROW_H;

    switch (event->type) {
        case SPC_INPUT_EVENT_MOUSE_WHEEL: {
            int delta = (event->mouse.wheel_delta > 0) ? -3 : 3;
            state.scroll_offset += delta;
            int max_scroll = std::max(0, static_cast<int>(aircraft.size()) - visible_rows);
            state.scroll_offset = std::clamp(state.scroll_offset, 0, max_scroll);
            return 0;
        }

        case SPC_INPUT_EVENT_MOUSE_PRESS: {
            if (!(event->mouse.button & SPC_MOUSE_BUTTON_LEFT))
                return 0;

            int local_y = static_cast<int>(my) - panel_y;

            // header click: change sort column
            if (local_y < HEADER_H) {
                int total_def_w = 0;
                for (int i = 0; i < NUM_COLS; ++i) total_def_w += k_columns[i].width;
                float scale = static_cast<float>(panel_w - PAD_X * 2) / static_cast<float>(total_def_w);
                if (scale > 1.5f) scale = 1.5f;

                int cx = panel_x + PAD_X;
                int clicked_col = -1;
                for (int i = 0; i < NUM_COLS; ++i) {
                    int cw = static_cast<int>(k_columns[i].width * scale);
                    if (mx >= cx && mx < cx + cw) { clicked_col = i; break; }
                    cx += cw;
                }

                if (clicked_col >= 0) {
                    if (state.sort_column == clicked_col)
                        state.sort_ascending = !state.sort_ascending;
                    else {
                        state.sort_column = clicked_col;
                        state.sort_ascending = true;
                    }
                }
                return 0;
            }

            // data row click: toggle selection (matches map-click UX) and
            // center map only on select-on, never on toggle-off, so
            // deselecting doesn't make the map jump.
            if (local_y >= HEADER_H && local_y < HEADER_H + data_h) {
                int row = (local_y - HEADER_H) / ROW_H;
                int idx = state.scroll_offset + row;
                if (idx >= 0 && idx < static_cast<int>(aircraft.size())) {
                    uint32_t click_icao = aircraft[idx].icao;
                    if (state.selected_icao == click_icao) {
                        state.selected_icao = 0;
                    } else {
                        state.selected_icao = click_icao;
                        if (out_lat && out_lon) {
                            *out_lat = aircraft[idx].lat;
                            *out_lon = aircraft[idx].lon;
                        }
                    }
                }
                return 0;
            }
            return 0;
        }

        case SPC_INPUT_EVENT_MOUSE_MOVE: {
            int local_y = static_cast<int>(my) - panel_y;
            if (local_y >= HEADER_H && local_y < HEADER_H + data_h) {
                int row = (local_y - HEADER_H) / ROW_H;
                int idx = state.scroll_offset + row;
                if (idx >= 0 && idx < static_cast<int>(aircraft.size()))
                    state.hovered_row_icao = aircraft[idx].icao;
                else
                    state.hovered_row_icao = 0;
            } else {
                state.hovered_row_icao = 0;
            }
            return 0;
        }

        default:
            return 0;
    }
}

} // namespace adsb_list
