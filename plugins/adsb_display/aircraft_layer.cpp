#include "aircraft_layer.h"
#include "adsb_display_state.h"
#include "aircraft_shapes.h"
#include "geo_helpers.h"
#include "color_palette.h"
#include "unit_format.h"

#include <opencv2/imgproc.hpp>

#include <spc_ui_text.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <format>
#include <string>
#include <unordered_set>
#include <vector>

void render_aircraft_layer(MapDisplayState* s,
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
    // plane shouldn't leave a phantom trail behind on the map).
    //
    // Render each trail as N_BUCKETS cv::polylines calls instead of one
    // cv::line per segment: the fade is monotonic with index so consecutive
    // segments fall into contiguous buckets, and 8 brightness levels across
    // a 30-point trail is below perceptual JND. Saves ~75% of the dispatch
    // overhead vs the old per-segment loop while keeping cv::LINE_AA on for
    // visual quality.
    std::unordered_set<uint32_t> visible_icaos;
    if (s->cur.show_trails) {
        visible_icaos.reserve(aircraft.size());
        for (const auto& ac : aircraft) visible_icaos.insert(ac.icao);

        constexpr int N_BUCKETS = 8;
        std::vector<cv::Point> trail_pts;
        trail_pts.reserve(s->trail_history.max_length);

        for (const auto& [icao, trail] : s->trail_history.trails) {
            if (!visible_icaos.contains(icao)) continue;
            if (trail.size() < 2) continue;
            int n = static_cast<int>(trail.size());

            // project all trail points to pixels once per aircraft
            trail_pts.clear();
            for (const auto& pt : trail) {
                auto px = geo_to_pixel(pt.lat, pt.lon, zoom, origin_tx, origin_ty);
                trail_pts.emplace_back(static_cast<int>(px.x),
                                       static_cast<int>(px.y));
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
                cv::Scalar faded(col[0] * fade, col[1] * fade, col[2] * fade);

                // polyline through points seg_start..seg_end (inclusive)
                // covers segments [seg_start, seg_end). C-style overload
                // takes a slice of the projected-point buffer with no copy.
                const cv::Point* slice = trail_pts.data() + seg_start;
                int npts = seg_end - seg_start + 1;
                cv::polylines(s->canvas, &slice, &npts, 1, false,
                              faded, 1, cv::LINE_AA);
            }
        }
    }

    // pulsing frame counter for emergency highlights
    s->frame_number_local++;
    double pulse = 0.5 + 0.5 * std::sin(s->frame_number_local * 0.15);

    auto outline = map_contrast_color(s->cur.map_style);
    auto halo = map_halo_color(s->cur.map_style);

    uint32_t selected_icao = s->info_panel_icao.load(std::memory_order_relaxed);

    for (const auto& ac : aircraft) {
        auto px = geo_to_pixel(ac.lat, ac.lon, zoom, origin_tx, origin_ty);

        if (px.x < -20 || px.x > w + 20 || px.y < -20 || px.y > h + 20)
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

        // record screen position for hit-testing
        build_pos.push_back({ac.icao,
                             static_cast<int>(px.x), static_cast<int>(px.y),
                             icon_size});

        // trend vector: short line projecting the aircraft's position forward
        // by `trend_seconds`, same color as the icon. Skip on ground or when
        // ground-speed is negligible so parked aircraft don't show a hairline.
        if (s->cur.show_trend_vector && !ac.on_ground && ac.gs >= 20.0f) {
            double dist_km = ac.gs * 1.852 * (s->cur.trend_seconds / 3600.0);
            auto [end_lat, end_lon] = dest_point(ac.lat, ac.lon, ac.track, dist_km);
            auto end_px = geo_to_pixel(end_lat, end_lon, zoom, origin_tx, origin_ty);
            cv::Point p0{static_cast<int>(px.x), static_cast<int>(px.y)};
            cv::Point p1{static_cast<int>(end_px.x), static_cast<int>(end_px.y)};
            cv::line(s->canvas, p0, p1, halo,  3, cv::LINE_AA);
            cv::line(s->canvas, p0, p1, color, 1, cv::LINE_AA);
        }

        // selected-aircraft ring: distinct on-map marker for whichever plane
        // the info panel is currently showing, so the list<->map link is obvious.
        if (selected_icao != 0 && ac.icao == selected_icao) {
            int sel_r = icon_size + 6;
            cv::circle(s->canvas, {static_cast<int>(px.x), static_cast<int>(px.y)},
                       sel_r, halo,    3, cv::LINE_AA);
            cv::circle(s->canvas, {static_cast<int>(px.x), static_cast<int>(px.y)},
                       sel_r, outline, 1, cv::LINE_AA);
        }

        // MLAT source indicator: a dashed circle outside the icon signals that
        // the position came from multilateration (~50-500 m accuracy) rather
        // than the aircraft's own GPS. Drawn as 12 short arc segments for a
        // visible dash pattern (OpenCV has no dashed-circle primitive).
        // At low zoom the dashes blur into a solid ring anyway — fall back
        // to a single cv::circle so 50+ MLAT aircraft don't cost ~600
        // ellipse calls per frame at continental zoom.
        if (ac.msg_source == 5) {
            int ix = static_cast<int>(px.x);
            int iy = static_cast<int>(px.y);
            int dr = icon_size + 2;
            if (zoom < 11.0) {
                cv::circle(s->canvas, {ix, iy}, dr, halo, 1);
            } else {
                constexpr int n_dashes = 12;
                constexpr int dash_deg = 360 / (n_dashes * 2);
                for (int k = 0; k < n_dashes; ++k) {
                    int a0 = 360 * k / n_dashes;
                    cv::ellipse(s->canvas, {ix, iy}, {dr, dr}, 0,
                                a0, a0 + dash_deg, halo, 2, cv::LINE_AA);
                }
            }
        }

        // emergency alert: pulsing red ring behind the icon
        if (is_emergency(ac)) {
            int ring_r = icon_size + 4 + static_cast<int>(pulse * 4);
            int alpha_i = static_cast<int>(180 + 75 * pulse);
            cv::Scalar red(static_cast<double>(alpha_i), 0, 0);
            cv::circle(s->canvas, {static_cast<int>(px.x), static_cast<int>(px.y)},
                       ring_r, red, 2, cv::LINE_AA);
        }

        auto* shape = aircraft_shapes::lookup(
            ac.type_code_str.c_str(), ac.category);
        draw_aircraft(s->canvas, px, ac.track, color, icon_size, shape, halo);

        if (s->cur.show_labels) {
            // Cached label strings populated in input_parser::read_aircraft_data —
            // ac.cs_label, ac.alt_label, ac.vs_label — so this loop avoids
            // std::format and the MLAT-prefix branching repeated for halo+main.
            int label_offset = std::max(12, static_cast<int>(15 * s->cur.icon_scale));
            cv::Scalar label_color = is_emergency(ac)
                ? cv::Scalar(255, 0, 0) : outline;
            double fs = static_cast<double>(s->cur.font_size);
            int lx = static_cast<int>(px.x) + label_offset;
            int ly = static_cast<int>(px.y) + 4;

            spc::ui::draw_text_outlined(s->canvas, ac.cs_label, {lx, ly}, fs,
                                        label_color, halo);

            if (!ac.alt_label.empty()) {
                int baseline = 0;
                auto text_sz = cv::getTextSize(ac.cs_label, cv::FONT_HERSHEY_SIMPLEX,
                                                fs, 1, &baseline);
                int line_gap = text_sz.height + baseline + 2;
                int alt_y = ly + line_gap;
                spc::ui::draw_text_outlined(s->canvas, ac.alt_label, {lx, alt_y}, fs,
                                            label_color, halo);

                // vertical-speed glyph: small green-up / red-down triangle plus
                // the rate magnitude, after the altitude text. Prefer geom rate
                // (GPS-derived) and fall back to baro rate, matching the tooltip.
                // ac.vs_label is empty when the indicator shouldn't render —
                // gate covers (show_vs_indicator off) and (|vs| < 300).
                if (!ac.vs_label.empty()) {
                    int vs_fpm = ac.geom_rate != 0 ? ac.geom_rate : ac.baro_rate;
                    int alt_baseline = 0;
                    int alt_w = cv::getTextSize(ac.alt_label, cv::FONT_HERSHEY_SIMPLEX,
                                                 fs, 1, &alt_baseline).width;
                    int gx = lx + alt_w + 6;
                    int th_sz = std::max(5, static_cast<int>(fs * 18.0));
                    int tw = th_sz;
                    bool climbing = vs_fpm > 0;
                    cv::Point tri[3];
                    if (climbing) {
                        tri[0] = {gx + tw / 2, alt_y - th_sz};
                        tri[1] = {gx,          alt_y};
                        tri[2] = {gx + tw,     alt_y};
                    } else {
                        tri[0] = {gx,          alt_y - th_sz};
                        tri[1] = {gx + tw,     alt_y - th_sz};
                        tri[2] = {gx + tw / 2, alt_y};
                    }
                    cv::Scalar tri_col = climbing ? cv::Scalar(40, 220, 40)
                                                  : cv::Scalar(240, 60, 60);
                    cv::fillConvexPoly(s->canvas, tri, 3, tri_col, cv::LINE_AA);

                    cv::Point vs_org{gx + tw + 3, alt_y};
                    spc::ui::draw_text_outlined(s->canvas, ac.vs_label, vs_org, fs,
                                                label_color, halo);
                }
            }
        }
    }

    // publish screen positions for event handler
    s->screen_pos_active.store(build_idx, std::memory_order_release);
}
