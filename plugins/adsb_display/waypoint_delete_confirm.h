#pragma once

// Confirmation modal for right-click deletion of a waypoint. Shared between
// the event handler (hit-tests the buttons) and the renderer (draws the
// popup) so the geometry stays in lock-step regardless of where the click
// lands relative to the waypoint marker.

#include "adsb_display_state.h"
#include "draw_helpers.h"
#include <spc_ui_theme.h>

#include <algorithm>
#include <cstdio>

namespace adsb_waypoint_delete {

constexpr int MODAL_W   = 260;
constexpr int MODAL_H   = 100;
constexpr int BTN_H     = 28;
constexpr int BTN_W     = 110;
constexpr int BTN_PAD   = 10;
constexpr int ANCHOR_GAP = 20;  // px between marker and modal edge

struct Geometry {
    cv::Rect modal;
    cv::Rect cancel_btn;
    cv::Rect delete_btn;
};

// Centered-on-anchor placement. Prefers below the marker; flips above when
// the bottom would clip. Clamped to the canvas so the modal never renders
// off-screen even if the user right-clicked near a corner.
inline Geometry compute_geometry(float anchor_x, float anchor_y,
                                 int canvas_w, int canvas_h)
{
    int x = static_cast<int>(anchor_x) - MODAL_W / 2;
    int y = static_cast<int>(anchor_y) + ANCHOR_GAP;
    if (y + MODAL_H > canvas_h - 5)
        y = static_cast<int>(anchor_y) - ANCHOR_GAP - MODAL_H;
    x = std::clamp(x, 5, std::max(5, canvas_w - MODAL_W - 5));
    y = std::clamp(y, 5, std::max(5, canvas_h - MODAL_H - 5));

    Geometry g;
    g.modal = cv::Rect(x, y, MODAL_W, MODAL_H);
    int by = y + MODAL_H - BTN_PAD - BTN_H;
    g.cancel_btn = cv::Rect(x + BTN_PAD, by, BTN_W, BTN_H);
    g.delete_btn = cv::Rect(x + MODAL_W - BTN_PAD - BTN_W, by, BTN_W, BTN_H);
    return g;
}

inline bool contains(const cv::Rect& r, float px, float py)
{
    return px >= r.x && py >= r.y
        && px < r.x + r.width && py < r.y + r.height;
}

inline void render(adsb_draw::DrawList& list, int32_t font, int canvas_w, int canvas_h,
                   float anchor_x, float anchor_y, const char* wp_name)
{
    static const spc::ui::Theme theme;

    adsb_draw::fill_px(list, 0, 0, canvas_w - 1, canvas_h - 1, adsb_draw::rgb({0, 0, 0}, 0.45));

    auto geo = compute_geometry(anchor_x, anchor_y, canvas_w, canvas_h);

    const auto& m = geo.modal;
    adsb_draw::fill_px(list, m.x, m.y, m.x + m.width - 1, m.y + m.height - 1,
                       adsb_draw::rgb(theme.panel_bg, 0.96));
    adsb_draw::stroke_px(list, m.x, m.y, m.x + m.width - 1, m.y + m.height - 1,
                         adsb_draw::rgb(theme.surface2), 1);

    // Title + subtitle (waypoint name)
    int line_h = 18;
    int tx = geo.modal.x + 12;
    int ty = geo.modal.y + 10;
    adsb_draw::text_vcenter(list, tx, ty, line_h, "Delete waypoint?",
                            adsb_draw::style(font, 0.42, theme.text, true));

    char subtitle[SPC_LIST_CELL_STRING_MAX + 8];
    if (wp_name && wp_name[0] != '\0') {
        std::snprintf(subtitle, sizeof(subtitle), "\"%s\"", wp_name);
    } else {
        std::snprintf(subtitle, sizeof(subtitle), "(unnamed)");
    }
    adsb_draw::text_vcenter(list, tx, ty + line_h, line_h, subtitle,
                            adsb_draw::style(font, 0.36, theme.subtext, true));

    auto draw_button = [&](const cv::Rect& r, const char* label,
                           const cv::Scalar& bg, const cv::Scalar& fg) {
        adsb_draw::fill_px(list, r.x, r.y, r.x + r.width - 1, r.y + r.height - 1, adsb_draw::rgb(bg));
        adsb_draw::stroke_px(list, r.x, r.y, r.x + r.width - 1, r.y + r.height - 1,
                             adsb_draw::rgb(theme.surface2), 1);
        const auto st = adsb_draw::style(font, 0.42, fg, true);
        adsb_draw::text_vcenter(list, r.x + (r.width - adsb_draw::text_width(label, st)) / 2,
                                r.y, r.height, label, st);
    };

    draw_button(geo.cancel_btn, "Cancel", theme.surface1, theme.text);
    // BGR: red destructive accent
    draw_button(geo.delete_btn, "Delete",
                cv::Scalar(60, 60, 220), cv::Scalar(255, 255, 255));
}

} // namespace adsb_waypoint_delete
