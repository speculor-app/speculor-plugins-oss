#include "aircraft_layer.h"
#include "adsb_display_state.h"
#include "aircraft_shapes.h"
#include "draw_helpers.h"
#include "geo_helpers.h"
#include "color_palette.h"
#include "unit_format.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <format>
#include <string>
#include <unordered_set>
#include <vector>

using namespace adsb_draw;

void render_aircraft_layer(MapDisplayState* s, DrawList& list,
                           const std::vector<AircraftView>& aircraft,
                           double zoom, double origin_tx, double origin_ty,
                           int w, int h)
{
    // prepare screen position buffer for hover hit-testing
    int build_idx = 1 - s->screen_pos_active.load(std::memory_order_relaxed);
    auto& build_pos = s->screen_pos[build_idx];
    build_pos.clear();
    build_pos.reserve(aircraft.size());

    // draw altitude-colored fading trails (only for aircraft that passed the
    // filter — trail history keeps every observed track, but a filtered-out
    // plane shouldn't leave a phantom trail behind on the map). The fade is
    // monotonic with index, so 8 brightness levels across a trail suffice.
    std::unordered_set<uint32_t> visible_icaos;
    if (s->cur.show_trails) {
        visible_icaos.reserve(aircraft.size());
        for (const auto& ac : aircraft) visible_icaos.insert(ac.icao);

        constexpr int N_BUCKETS = 8;
        std::vector<Point> trail_pts;
        trail_pts.reserve(s->trail_history.max_length);

        for (const auto& [icao, trail] : s->trail_history.trails) {
            if (!visible_icaos.contains(icao)) continue;
            if (trail.size() < 2) continue;
            int n = static_cast<int>(trail.size());

            trail_pts.clear();
            for (const auto& pt : trail) {
                auto p = geo_to_pixel(pt.lat, pt.lon, zoom, origin_tx, origin_ty);
                trail_pts.push_back(px(static_cast<int>(p.x), static_cast<int>(p.y)));
            }

            int n_segments = n - 1;
            for (int b = 0; b < N_BUCKETS; ++b) {
                int seg_start = (b * n_segments) / N_BUCKETS;
                int seg_end   = ((b + 1) * n_segments) / N_BUCKETS;  // exclusive
                if (seg_end <= seg_start) continue;

                // representative trail point for the bucket's fade + color
                int rep = std::min(n - 1, (seg_start + seg_end) / 2 + 1);
                double fade = static_cast<double>(rep) / n;
                auto col = altitude_color(trail[rep].alt);
                const Color faded = rgb({col[0] * fade, col[1] * fade, col[2] * fade});
                for (int k = seg_start; k < seg_end; ++k) list.line(trail_pts[k], trail_pts[k + 1], 1, faded);
            }
        }
    }

    // pulsing frame counter for emergency highlights
    s->frame_number_local++;
    double pulse = 0.5 + 0.5 * std::sin(s->frame_number_local * 0.15);

    auto outline = map_contrast_color(s->cur.map_style);
    auto halo = map_halo_color(s->cur.map_style);
    const int32_t font = s->cur.font;

    uint32_t selected_icao = s->info_panel_icao.load(std::memory_order_relaxed);

    for (const auto& ac : aircraft) {
        auto pos = geo_to_pixel(ac.lat, ac.lon, zoom, origin_tx, origin_ty);

        if (pos.x < -20 || pos.x > w + 20 || pos.y < -20 || pos.y > h + 20)
            continue;

        cv::Scalar color = ac.on_ground ? cv::Scalar(150, 150, 150)
                                        : altitude_color(ac.alt);
        int base_size = ac.on_ground ? 8 : 13;
        int icon_size = std::max(4, static_cast<int>(base_size * s->cur.icon_scale));

        // staleness fade: blend color toward map halo (background) as `seen`
        // grows past 5s, clamped at 40% so the plane doesn't vanish before
        // the engine actually expires the track.
        float fade = 1.0f;
        if (s->cur.show_staleness_fade && ac.seen > 5.0f) {
            fade = std::max(0.4f, 1.0f - (ac.seen - 5.0f) / 25.0f);
            for (int i = 0; i < 3; ++i)
                color[i] = color[i] * fade + halo[i] * (1.0 - fade);
        }

        const int ix = static_cast<int>(pos.x), iy = static_cast<int>(pos.y);
        const Point centre = px(ix, iy);

        // record screen position for hit-testing
        build_pos.push_back({ac.icao, ix, iy, icon_size});

        // trend vector: short line projecting the aircraft's position forward
        // by `trend_seconds`, same color as the icon. Skip on ground or when
        // ground-speed is negligible so parked aircraft don't show a hairline.
        if (s->cur.show_trend_vector && !ac.on_ground && ac.gs >= 20.0f) {
            double dist_km = ac.gs * 1.852 * (s->cur.trend_seconds / 3600.0);
            auto [end_lat, end_lon] = dest_point(ac.lat, ac.lon, ac.track, dist_km);
            auto end_px = geo_to_pixel(end_lat, end_lon, zoom, origin_tx, origin_ty);
            const Point end = px(static_cast<int>(end_px.x), static_cast<int>(end_px.y));
            list.line(centre, end, 3, rgb(halo));
            list.line(centre, end, 1, rgb(color));
        }

        // selected-aircraft ring: distinct on-map marker for whichever plane
        // the info panel is currently showing, so the list<->map link is obvious.
        if (selected_icao != 0 && ac.icao == selected_icao) {
            const float sel_r = static_cast<float>(icon_size + 6);
            list.stroke_circle(centre, sel_r, 3, rgb(halo));
            list.stroke_circle(centre, sel_r, 1, rgb(outline));
        }

        // MLAT source indicator: a dashed circle outside the icon signals that
        // the position came from multilateration (~50-500 m accuracy) rather
        // than the aircraft's own GPS. At low zoom the dashes blur into a solid
        // ring anyway, so it is drawn solid there.
        if (ac.msg_source == 5) {
            const float dr = static_cast<float>(icon_size + 2);
            if (zoom < 11.0) {
                list.stroke_circle(centre, dr, 1, rgb(halo));
            } else {
                constexpr int n_dashes = 12;
                constexpr int dash_deg = 360 / (n_dashes * 2);
                for (int k = 0; k < n_dashes; ++k) {
                    const float a0 = static_cast<float>(360 * k / n_dashes);
                    arc(list, centre, dr, a0, a0 + dash_deg, 2, rgb(halo));
                }
            }
        }

        // emergency alert: pulsing red ring behind the icon
        if (is_emergency(ac)) {
            const float ring_r = static_cast<float>(icon_size + 4 + static_cast<int>(pulse * 4));
            const double red = 180 + 75 * pulse;
            list.stroke_circle(centre, ring_r, 2, rgb({red, 0, 0}));
        }

        auto* shape = aircraft_shapes::lookup(
            ac.type_code_str.c_str(), ac.category);
        draw_aircraft(list, centre, ac.track, color, icon_size, shape, halo);

        if (s->cur.show_labels) {
            // Cached label strings populated in input_parser::read_aircraft_data —
            // ac.cs_label, ac.alt_label, ac.vs_label.
            int label_offset = std::max(12, static_cast<int>(15 * s->cur.icon_scale));
            cv::Scalar label_color = is_emergency(ac)
                ? cv::Scalar(255, 0, 0) : outline;
            double fs = static_cast<double>(s->cur.font_size);
            const TextStyle st = style(font, fs, label_color);
            int lx = ix + label_offset;
            int ly = iy + 4;

            text_outlined(list, lx, ly, ac.cs_label, st, halo);

            if (!ac.alt_label.empty()) {
                const auto m = spc::draw::measure_text(ac.cs_label, st);
                int alt_y = ly + static_cast<int>(std::lround(m.ascent + m.descent)) + 2;
                text_outlined(list, lx, alt_y, ac.alt_label, st, halo);

                // vertical-speed glyph: small green-up / red-down triangle plus
                // the rate magnitude, after the altitude text. Prefer geom rate
                // (GPS-derived) and fall back to baro rate, matching the tooltip.
                // ac.vs_label is empty when the indicator shouldn't render —
                // gate covers (show_vs_indicator off) and (|vs| < 300).
                if (!ac.vs_label.empty()) {
                    int vs_fpm = ac.geom_rate != 0 ? ac.geom_rate : ac.baro_rate;
                    int alt_w = static_cast<int>(std::lround(text_width(ac.alt_label, st)));
                    int gx = lx + alt_w + 6;
                    int th_sz = std::max(5, static_cast<int>(fs * 18.0));
                    int tw = th_sz;
                    bool climbing = vs_fpm > 0;
                    Point tri[3];
                    if (climbing) {
                        tri[0] = px(gx + tw / 2, alt_y - th_sz);
                        tri[1] = px(gx,          alt_y);
                        tri[2] = px(gx + tw,     alt_y);
                    } else {
                        tri[0] = px(gx,          alt_y - th_sz);
                        tri[1] = px(gx + tw,     alt_y - th_sz);
                        tri[2] = px(gx + tw / 2, alt_y);
                    }
                    list.fill_polygon(tri, climbing ? Color{40, 220, 40} : Color{240, 60, 60});

                    text_outlined(list, gx + tw + 3, alt_y, ac.vs_label, st, halo);
                }
            }
        }
    }

    // publish screen positions for event handler
    s->screen_pos_active.store(build_idx, std::memory_order_release);
}
