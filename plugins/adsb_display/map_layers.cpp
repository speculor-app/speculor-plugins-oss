#include "map_layers.h"
#include "adsb_display_state.h"
#include "geo_helpers.h"

#include <spc_ui_panel.h>
#include <opencv2/imgproc.hpp>

#include <spc_ui_text.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <format>
#include <numbers>
#include <string>

namespace {

// draw a circle with the given line style (0=solid, 1=dashed, 2=dotted).
// Uses a fixed angular step so the dash pattern doesn't jitter as the radius
// changes — otherwise animated rings would flicker.
void draw_styled_circle(cv::Mat& canvas, cv::Point center, int radius,
                        cv::Scalar color, int thickness, int style)
{
    if (style == 0) {
        cv::circle(canvas, center, radius, color, thickness, cv::LINE_AA);
        return;
    }

    double seg_angle = (style == 1) ? 12.0 : 6.0;
    double on_frac   = (style == 1) ? 0.667 : 0.5;
    int num_segs = static_cast<int>(360.0 / seg_angle);

    for (int i = 0; i < num_segs; ++i) {
        double start_angle = i * seg_angle;
        double arc_angle = seg_angle * on_frac;
        cv::ellipse(canvas, center, {radius, radius}, 0,
                    start_angle, start_angle + arc_angle,
                    color, thickness, cv::LINE_AA);
    }
}

} // namespace

void render_range_rings(MapDisplayState* s,
                        double gps_lat, double gps_lon,
                        double zoom, double origin_tx, double origin_ty,
                        int w, int h)
{
    if (!s->cur.show_range_rings) return;

    uint32_t rgba = s->cur.range_ring_color;
    cv::Scalar ring_color((rgba >> 24) & 0xFF,
                          (rgba >> 16) & 0xFF,
                          (rgba >> 8) & 0xFF);
    int thickness = s->cur.range_ring_thickness;
    int style = s->cur.range_ring_style;
    float opacity = std::clamp(s->cur.range_ring_opacity, 0.0f, 1.0f);
    float interval_km = s->cur.range_ring_interval;
    if (interval_km < 1.0f) interval_km = 1.0f;

    auto center_px = geo_to_pixel(gps_lat, gps_lon, zoom, origin_tx, origin_ty);
    cv::Point center_pt(static_cast<int>(center_px.x), static_cast<int>(center_px.y));

    // 1 degree latitude ~= 110.574 km — derive pixels-per-km at the current lat/zoom
    double one_km_deg = 1.0 / 110.574;
    auto north_px = geo_to_pixel(gps_lat + one_km_deg, gps_lon,
                                 zoom, origin_tx, origin_ty);
    double px_per_km = std::abs(center_px.y - north_px.y);
    if (px_per_km < 0.1) return;

    double max_visible_px = std::sqrt(static_cast<double>(w * w + h * h));

    cv::Rect map_rect(0, 0, w, h);
    cv::Mat map_roi = s->canvas(map_rect);
    cv::Mat overlay;
    if (opacity < 0.99f)
        overlay = map_roi.clone();

    cv::Mat& target = (opacity < 0.99f) ? overlay : map_roi;

    for (float dist_km = interval_km; ; dist_km += interval_km) {
        int r_px = static_cast<int>(dist_km * px_per_km);
        if (r_px < 2) continue;
        if (r_px > max_visible_px) break;

        draw_styled_circle(target, center_pt, r_px, ring_color, thickness, style);

        int label_x = center_pt.x + r_px + 4;
        int label_y = center_pt.y + 4;

        if (label_x > 0 && label_x < w - 30 && label_y > 10 && label_y < h - 10) {
            std::string label;
            if (dist_km >= 10.0f)
                label = std::format("{:.0f} km", dist_km);
            else
                label = std::format("{:.1f} km", dist_km);

            double rfs = static_cast<double>(s->cur.range_ring_font_size);
            spc::ui::draw_text_outlined(target, label, {label_x, label_y}, rfs, ring_color);
        }
    }

    int ch = 6;
    cv::line(target, {center_pt.x - ch, center_pt.y},
             {center_pt.x + ch, center_pt.y}, ring_color, thickness, cv::LINE_AA);
    cv::line(target, {center_pt.x, center_pt.y - ch},
             {center_pt.x, center_pt.y + ch}, ring_color, thickness, cv::LINE_AA);

    if (opacity < 0.99f)
        cv::addWeighted(overlay, opacity, map_roi, 1.0 - opacity, 0, map_roi);
}

void render_max_range(MapDisplayState* s,
                      double gps_lat, double gps_lon,
                      double zoom, double origin_tx, double origin_ty,
                      int w, int h)
{
    if (!s->cur.show_max_range || s->max_range_km < 0.1) return;

    uint32_t rgba = s->cur.max_range_color;
    cv::Scalar color((rgba >> 24) & 0xFF,
                     (rgba >> 16) & 0xFF,
                     (rgba >> 8) & 0xFF);
    float opacity = std::clamp(s->cur.max_range_opacity, 0.0f, 1.0f);

    auto center_px = geo_to_pixel(gps_lat, gps_lon, zoom, origin_tx, origin_ty);
    cv::Point center_pt(static_cast<int>(center_px.x), static_cast<int>(center_px.y));

    double one_km_deg = 1.0 / 110.574;
    auto north_px = geo_to_pixel(gps_lat + one_km_deg, gps_lon,
                                 zoom, origin_tx, origin_ty);
    double px_per_km = std::abs(center_px.y - north_px.y);
    if (px_per_km < 0.1) return;

    int r_px = static_cast<int>(s->max_range_km * px_per_km);
    if (r_px < 2) return;

    cv::Rect map_rect(0, 0, w, h);
    cv::Mat map_roi = s->canvas(map_rect);
    cv::Mat overlay;
    if (opacity < 0.99f) overlay = map_roi.clone();
    cv::Mat& target = (opacity < 0.99f) ? overlay : map_roi;

    draw_styled_circle(target, center_pt, r_px, color, 2, 2);  // dotted

    auto label = std::format("MAX {:.1f} km", s->max_range_km);
    double mfs = static_cast<double>(s->cur.max_range_font_size);
    auto label_sz = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, mfs, 1, nullptr);
    int label_x = center_pt.x - r_px - label_sz.width - 4;
    int label_y = center_pt.y + 4;
    if (label_x > 0 && label_x < w - 10 && label_y > 10 && label_y < h - 10) {
        spc::ui::draw_text_outlined(target, label, {label_x, label_y}, mfs, color);
    }

    if (opacity < 0.99f)
        cv::addWeighted(overlay, opacity, map_roi, 1.0 - opacity, 0, map_roi);
}

void render_compass_rose(MapDisplayState* s, int map_w, int h)
{
    static const spc::ui::Theme th;

    int radius = 28;
    int ext = radius + 12;
    int margin = ext + 2;
    int cx = map_w - margin;
    int cy = margin;

    if (map_w < margin * 2 + 10 || h < margin * 2 + 10)
        return;

    int x0 = cx - ext, y0 = cy - ext;
    int bw = ext * 2, bh = ext * 2;
    x0 = std::max(0, x0); y0 = std::max(0, y0);
    bw = std::min(bw, map_w - x0); bh = std::min(bh, h - y0);
    cv::Mat roi = s->canvas(cv::Rect(x0, y0, bw, bh));
    cv::Mat overlay;
    roi.copyTo(overlay);
    cv::circle(overlay, {cx - x0, cy - y0}, ext, th.bg, cv::FILLED, cv::LINE_AA);
    cv::addWeighted(overlay, 0.75, roi, 0.25, 0, roi);

    cv::circle(s->canvas, {cx, cy}, radius, th.surface2, 1, cv::LINE_AA);

    for (int i = 0; i < 8; ++i) {
        double angle = i * 45.0 * std::numbers::pi / 180.0;
        bool cardinal = (i % 2 == 0);
        int inner = cardinal ? radius - 7 : radius - 4;
        int outer = radius - 1;
        int ix = cx + static_cast<int>(std::sin(angle) * inner);
        int iy = cy - static_cast<int>(std::cos(angle) * inner);
        int ox = cx + static_cast<int>(std::sin(angle) * outer);
        int oy = cy - static_cast<int>(std::cos(angle) * outer);
        auto col = cardinal ? th.text : th.surface2;
        cv::line(s->canvas, {ix, iy}, {ox, oy}, col, 1, cv::LINE_AA);
    }

    // north pointer (filled, red)
    {
        int tip = radius - 9;
        int base = 5;
        int tail = 4;
        cv::Point pts[3] = {
            {cx, cy - tip},
            {cx - base, cy - tail},
            {cx + base, cy - tail}
        };
        cv::fillConvexPoly(s->canvas, pts, 3, th.red, cv::LINE_AA);
    }

    // south pointer (smaller, subtle)
    {
        int tip = radius - 9;
        int base = 4;
        int tail = 3;
        cv::Point pts[3] = {
            {cx, cy + tip},
            {cx - base, cy + tail},
            {cx + base, cy + tail}
        };
        cv::fillConvexPoly(s->canvas, pts, 3, th.surface2, cv::LINE_AA);
    }

    cv::circle(s->canvas, {cx, cy}, 2, th.text, cv::FILLED, cv::LINE_AA);

    double fs = 0.30;
    int baseline = 0;
    int label_r = radius + 8;

    auto n_sz = cv::getTextSize("N", cv::FONT_HERSHEY_SIMPLEX, fs, 1, &baseline);
    cv::putText(s->canvas, "N",
               {cx - n_sz.width / 2, cy - label_r + n_sz.height / 2},
               cv::FONT_HERSHEY_SIMPLEX, fs, th.red, 1, cv::LINE_AA);

    auto s_sz = cv::getTextSize("S", cv::FONT_HERSHEY_SIMPLEX, fs, 1, &baseline);
    cv::putText(s->canvas, "S",
               {cx - s_sz.width / 2, cy + label_r + s_sz.height / 2},
               cv::FONT_HERSHEY_SIMPLEX, fs, th.subtext, 1, cv::LINE_AA);

    auto e_sz = cv::getTextSize("E", cv::FONT_HERSHEY_SIMPLEX, fs, 1, &baseline);
    cv::putText(s->canvas, "E",
               {cx + label_r - e_sz.width / 2, cy + e_sz.height / 2},
               cv::FONT_HERSHEY_SIMPLEX, fs, th.subtext, 1, cv::LINE_AA);

    auto w_sz = cv::getTextSize("W", cv::FONT_HERSHEY_SIMPLEX, fs, 1, &baseline);
    cv::putText(s->canvas, "W",
               {cx - label_r - w_sz.width / 2, cy + w_sz.height / 2},
               cv::FONT_HERSHEY_SIMPLEX, fs, th.subtext, 1, cv::LINE_AA);
}
