#include "map_layers.h"
#include "adsb_display_state.h"
#include "draw_helpers.h"
#include "geo_helpers.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <format>
#include <numbers>
#include <string>

using namespace adsb_draw;

namespace {

// draw a circle with the given line style (0=solid, 1=dashed, 2=dotted).
// Uses a fixed angular step so the dash pattern doesn't jitter as the radius
// changes — otherwise animated rings would flicker.
void draw_styled_circle(DrawList& list, Point center, float radius,
                        Color color, float thickness, int style)
{
    if (style == 0) {
        list.stroke_circle(center, radius, thickness, color);
        return;
    }

    float seg_angle = (style == 1) ? 12.0f : 6.0f;
    float on_frac   = (style == 1) ? 0.667f : 0.5f;
    int num_segs = static_cast<int>(360.0f / seg_angle);

    for (int i = 0; i < num_segs; ++i) {
        float start_angle = static_cast<float>(i) * seg_angle;
        arc(list, center, radius, start_angle, start_angle + seg_angle * on_frac, thickness, color);
    }
}

cv::Scalar color_of(uint32_t rgba)
{
    return {static_cast<double>((rgba >> 24) & 0xFF), static_cast<double>((rgba >> 16) & 0xFF),
            static_cast<double>((rgba >> 8) & 0xFF)};
}

// a label outlined in black, both at `opacity`
TextStyle faded_label(int32_t font, double scale, const cv::Scalar& color, float opacity)
{
    TextStyle st = style(font, scale, color);
    st.color = rgb(color, opacity);
    st.outline = rgb({0, 0, 0}, opacity);
    st.outline_width = 1;
    return st;
}

} // namespace

void render_range_rings(MapDisplayState* s, DrawList& list,
                        double gps_lat, double gps_lon,
                        double zoom, double origin_tx, double origin_ty,
                        int w, int h)
{
    if (!s->cur.show_range_rings) return;

    const cv::Scalar ring_scalar = color_of(s->cur.range_ring_color);
    float thickness = static_cast<float>(s->cur.range_ring_thickness);
    int style = s->cur.range_ring_style;
    float opacity = std::clamp(s->cur.range_ring_opacity, 0.0f, 1.0f);
    const Color ring_color = rgb(ring_scalar, opacity);
    float interval_km = s->cur.range_ring_interval;
    if (interval_km < 1.0f) interval_km = 1.0f;

    auto center_px = geo_to_pixel(gps_lat, gps_lon, zoom, origin_tx, origin_ty);
    const int cx = static_cast<int>(center_px.x), cy = static_cast<int>(center_px.y);
    const Point centre = px(cx, cy);

    // 1 degree latitude ~= 110.574 km — derive pixels-per-km at the current lat/zoom
    double one_km_deg = 1.0 / 110.574;
    auto north_px = geo_to_pixel(gps_lat + one_km_deg, gps_lon,
                                 zoom, origin_tx, origin_ty);
    double px_per_km = std::abs(center_px.y - north_px.y);
    if (px_per_km < 0.1) return;

    double max_visible_px = std::sqrt(static_cast<double>(w * w + h * h));
    const TextStyle label_style = faded_label(s->cur.font, s->cur.range_ring_font_size, ring_scalar, opacity);

    for (float dist_km = interval_km; ; dist_km += interval_km) {
        int r_px = static_cast<int>(dist_km * px_per_km);
        if (r_px < 2) continue;
        if (r_px > max_visible_px) break;

        draw_styled_circle(list, centre, static_cast<float>(r_px), ring_color, thickness, style);

        int label_x = cx + r_px + 4;
        int label_y = cy + 4;

        if (label_x > 0 && label_x < w - 30 && label_y > 10 && label_y < h - 10) {
            std::string label;
            if (dist_km >= 10.0f)
                label = std::format("{:.0f} km", dist_km);
            else
                label = std::format("{:.1f} km", dist_km);
            text(list, label_x, label_y, label, label_style);
        }
    }

    int ch = 6;
    list.line(px(cx - ch, cy), px(cx + ch, cy), thickness, ring_color);
    list.line(px(cx, cy - ch), px(cx, cy + ch), thickness, ring_color);
}

void render_max_range(MapDisplayState* s, DrawList& list,
                      double gps_lat, double gps_lon,
                      double zoom, double origin_tx, double origin_ty,
                      int w, int h)
{
    if (!s->cur.show_max_range || s->max_range_km < 0.1) return;

    const cv::Scalar color = color_of(s->cur.max_range_color);
    float opacity = std::clamp(s->cur.max_range_opacity, 0.0f, 1.0f);

    auto center_px = geo_to_pixel(gps_lat, gps_lon, zoom, origin_tx, origin_ty);
    const int cx = static_cast<int>(center_px.x), cy = static_cast<int>(center_px.y);

    double one_km_deg = 1.0 / 110.574;
    auto north_px = geo_to_pixel(gps_lat + one_km_deg, gps_lon,
                                 zoom, origin_tx, origin_ty);
    double px_per_km = std::abs(center_px.y - north_px.y);
    if (px_per_km < 0.1) return;

    int r_px = static_cast<int>(s->max_range_km * px_per_km);
    if (r_px < 2) return;

    draw_styled_circle(list, px(cx, cy), static_cast<float>(r_px), rgb(color, opacity), 2, 2);  // dotted

    auto label = std::format("MAX {:.1f} km", s->max_range_km);
    const TextStyle label_style = faded_label(s->cur.font, s->cur.max_range_font_size, color, opacity);
    int label_x = cx - r_px - static_cast<int>(std::lround(text_width(label, label_style))) - 4;
    int label_y = cy + 4;
    if (label_x > 0 && label_x < w - 10 && label_y > 10 && label_y < h - 10)
        text(list, label_x, label_y, label, label_style);
}

void render_compass_rose(MapDisplayState* s, DrawList& list, int map_w, int h)
{
    static const spc::ui::Theme th;

    int radius = 28;
    int ext = radius + 12;
    int margin = ext + 2;
    int cx = map_w - margin;
    int cy = margin;

    if (map_w < margin * 2 + 10 || h < margin * 2 + 10)
        return;

    const Point centre = px(cx, cy);
    list.fill_circle(centre, static_cast<float>(ext) + 0.5f, rgb(th.bg, 0.75));
    list.stroke_circle(centre, static_cast<float>(radius), 1, rgb(th.surface2));

    for (int i = 0; i < 8; ++i) {
        double angle = i * 45.0 * std::numbers::pi / 180.0;
        bool cardinal = (i % 2 == 0);
        int inner = cardinal ? radius - 7 : radius - 4;
        int outer = radius - 1;
        int ix = cx + static_cast<int>(std::sin(angle) * inner);
        int iy = cy - static_cast<int>(std::cos(angle) * inner);
        int ox = cx + static_cast<int>(std::sin(angle) * outer);
        int oy = cy - static_cast<int>(std::cos(angle) * outer);
        list.line(px(ix, iy), px(ox, oy), 1, rgb(cardinal ? th.text : th.surface2));
    }

    // north pointer (filled, red)
    {
        int tip = radius - 9;
        int base = 5;
        int tail = 4;
        const Point pts[3] = {px(cx, cy - tip), px(cx - base, cy - tail), px(cx + base, cy - tail)};
        list.fill_polygon(pts, rgb(th.red));
    }

    // south pointer (smaller, subtle)
    {
        int tip = radius - 9;
        int base = 4;
        int tail = 3;
        const Point pts[3] = {px(cx, cy + tip), px(cx - base, cy + tail), px(cx + base, cy + tail)};
        list.fill_polygon(pts, rgb(th.surface2));
    }

    list.fill_circle(centre, 2.5f, rgb(th.text));

    const double fs = 0.30;
    const int label_r = radius + 8;
    const auto letter = [&](const char* l, int x, int y, const cv::Scalar& color) {
        const TextStyle st = style(s->cur.font, fs, color);
        text(list, x - text_width(l, st) / 2, y + text_height(st) / 2, l, st);
    };
    letter("N", cx, cy - label_r, th.red);
    letter("S", cx, cy + label_r, th.subtext);
    letter("E", cx + label_r, cy, th.subtext);
    letter("W", cx - label_r, cy, th.subtext);
}
