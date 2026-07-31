#include "airports_layer.h"
#include "adsb_display_state.h"
#include "color_palette.h"
#include "geo_helpers.h"

#include <opencv2/imgproc.hpp>

#include <spc_ui_text.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace {

// Pick the most useful label text for an airport, in order of preference:
// IATA (short + most recognizable to civilians) > ICAO (precise) > first
// word of name (when neither code is populated for small strips).
std::string label_for(const AirportFeature& a)
{
    if (!a.iata.empty()) return a.iata;
    if (!a.ident.empty() && a.ident.size() <= 4) return a.ident;  // ICAO
    if (!a.name.empty()) {
        auto sp = a.name.find(' ');
        return sp == std::string::npos ? a.name : a.name.substr(0, sp);
    }
    return {};
}

// Zoom at which labels start appearing for each size bucket. Markers show
// earlier than labels so users see dots at continental scale and names at
// regional scale. Military airports get labeled one tier earlier so they
// pop — matching the same "military should stand out" principle as the
// red accent ring.
double label_min_zoom(AirportFeature::Size size, bool military)
{
    double z;
    switch (size) {
        case AirportFeature::Size::Large:    z = 6.0;  break;
        case AirportFeature::Size::Medium:   z = 8.0;  break;
        case AirportFeature::Size::Small:    z = 10.0; break;
        case AirportFeature::Size::Heliport:
        case AirportFeature::Size::Seaplane: z = 10.5; break;
        default:                             z = 12.0; break;
    }
    if (military) z -= 1.0;
    return z;
}

// Marker size per airport bucket, in pixels.
int radius_for(AirportFeature::Size s)
{
    switch (s) {
        case AirportFeature::Size::Large:    return 6;
        case AirportFeature::Size::Medium:   return 4;
        case AirportFeature::Size::Small:    return 2;
        case AirportFeature::Size::Heliport: return 3;
        case AirportFeature::Size::Seaplane: return 3;
        default:                             return 2;
    }
}

// Honor airport_min_type AND zoom-aware auto-visibility: small airports
// and heliports only render when zoomed in enough to justify the clutter.
bool visible_at(AirportFeature::Size size, int32_t min_type, double zoom)
{
    switch (size) {
        case AirportFeature::Size::Large:    return true;
        case AirportFeature::Size::Medium:   return min_type >= AIRPORT_MIN_MEDIUM;
        case AirportFeature::Size::Small:    return min_type >= AIRPORT_MIN_ALL && zoom >= 9.0;
        case AirportFeature::Size::Heliport:
        case AirportFeature::Size::Seaplane: return min_type >= AIRPORT_MIN_ALL && zoom >= 10.0;
        default:                             return false;
    }
}

// Draw a single airport. Civilian airports render as a hollow ring in a
// muted tone so they read as landmarks, not as traffic — filled white
// markers on a dark map read as loud as the aircraft icons and crowd the
// eye at continental zoom. Size tier is conveyed by ring radius + a small
// center dot for large/medium.
//
// Military keeps the red accent ring (slightly dimmed so it's not alarming
// on busy airspace) and the selection accent stays bright yellow so
// origin/dest / clicked airports still pop.
void draw_airport(cv::Mat& canvas, cv::Point2d center_px,
                  const AirportFeature& a, int32_t map_style,
                  const cv::Scalar& halo, bool accented)
{
    int r = radius_for(a.size);
    if (accented) r += 2;
    cv::Point c(static_cast<int>(center_px.x), static_cast<int>(center_px.y));

    cv::Scalar marker = is_dark_map(map_style)
        ? cv::Scalar(140, 160, 180)      // muted steel-blue on dark maps
        : cv::Scalar( 90, 100, 110);     // muted charcoal on light maps

    // halo underneath for visibility on busy tile backgrounds
    cv::circle(canvas, c, r + 1, halo, cv::FILLED, cv::LINE_AA);

    // hollow ring — main marker
    cv::circle(canvas, c, r, marker, 1, cv::LINE_AA);

    // center dot — size-tier cue
    if (a.size == AirportFeature::Size::Large) {
        cv::circle(canvas, c, 2, marker, cv::FILLED, cv::LINE_AA);
    } else if (a.size == AirportFeature::Size::Medium) {
        cv::circle(canvas, c, 1, marker, cv::FILLED, cv::LINE_AA);
    }

    if (a.military) {
        cv::circle(canvas, c, r + 3, cv::Scalar(200, 70, 70), 1, cv::LINE_AA);
    }

    if (accented) {
        cv::circle(canvas, c, r + 5, cv::Scalar(255, 200, 40), 2, cv::LINE_AA);
    }
}

const AirportFeature* lookup_airport(const MapDisplayState* s,
                                     const std::string& icao)
{
    if (icao.empty()) return nullptr;
    auto it = s->airport_by_icao.find(icao);
    return it == s->airport_by_icao.end() ? nullptr : it->second;
}

} // namespace

void render_airports_layer(MapDisplayState* s,
                           double zoom, double origin_tx, double origin_ty,
                           int map_w, int h)
{
    if (!s->cur.show_airports) return;
    if (!s->reference_loaded.load(std::memory_order_acquire)) return;
    if (s->reference.airports.empty()) return;

    // viewport bbox for cheap point-in-view reject
    double tl_lat, tl_lon, br_lat, br_lon;
    pixel_to_geo(0.0,             0.0,   zoom, origin_tx, origin_ty, tl_lat, tl_lon);
    pixel_to_geo(static_cast<double>(map_w),
                 static_cast<double>(h), zoom, origin_tx, origin_ty, br_lat, br_lon);
    double view_min_lat = std::min(tl_lat, br_lat);
    double view_max_lat = std::max(tl_lat, br_lat);
    double view_min_lon = std::min(tl_lon, br_lon);
    double view_max_lon = std::max(tl_lon, br_lon);

    auto outline = map_contrast_color(s->cur.map_style);
    auto halo    = map_halo_color(s->cur.map_style);

    // prepare the back-buffer for airport hit-testing (same double-buffer
    // pattern as aircraft)
    int build_idx = 1 - s->airport_screen_pos_active.load(std::memory_order_relaxed);
    auto& build_pos = s->airport_screen_pos[build_idx];
    build_pos.clear();
    build_pos.reserve(256);

    // resolve origin/destination airports for the selected aircraft (drives
    // the accent rings on origin/dest airport markers — the leader line
    // moved out to process() since it depends on aircraft pixel position).
    const AirportFeature* origin_ap = nullptr;
    const AirportFeature* dest_ap   = nullptr;
    uint32_t selected_icao = s->info_panel_icao.load(std::memory_order_relaxed);
    if (s->cur.show_origin_dest_highlight && selected_icao != 0) {
        for (const auto& ac : s->cached_aircraft) {
            if (ac.icao != selected_icao) continue;
            origin_ap = lookup_airport(s, ac.origin_icao);
            dest_ap   = lookup_airport(s, ac.dest_icao);
            break;
        }
    }

    int32_t info_airport_idx = s->info_panel_airport_idx.load(std::memory_order_relaxed);
    double fs = 0.32;
    int label_baseline = 0;
    auto label_probe = cv::getTextSize("A", cv::FONT_HERSHEY_SIMPLEX, fs, 1, &label_baseline);
    int label_h = label_probe.height;

    for (size_t i = 0; i < s->reference.airports.size(); ++i) {
        const auto& a = s->reference.airports[i];
        if (!visible_at(a.size, s->cur.airport_min_type, zoom)) continue;
        if (a.lat < view_min_lat || a.lat > view_max_lat) continue;
        if (a.lon < view_min_lon || a.lon > view_max_lon) continue;

        auto px = geo_to_pixel(a.lat, a.lon, zoom, origin_tx, origin_ty);
        bool accented = (origin_ap == &a) || (dest_ap == &a)
                        || info_airport_idx == static_cast<int32_t>(i);
        draw_airport(s->canvas, px, a, s->cur.map_style, halo, accented);

        int ix = static_cast<int>(px.x);
        int iy = static_cast<int>(px.y);

        // record for hit-testing regardless of whether a label renders
        build_pos.push_back({
            static_cast<int32_t>(i), ix, iy, radius_for(a.size) + 3
        });

        // label — zoom-gated per size bucket so continental views stay
        // readable. Drawn halo-under / contrast-over, same pattern as the
        // aircraft callsign so styling matches.
        if (zoom >= label_min_zoom(a.size, a.military)) {
            auto text = label_for(a);
            if (!text.empty()) {
                int lx = ix + radius_for(a.size) + 5;
                int ly = iy + label_h / 2;
                spc::ui::draw_text_outlined(s->canvas, text, {lx, ly}, fs, outline, halo);
            }
        }
    }

    s->airport_screen_pos_active.store(build_idx, std::memory_order_release);

    // Leader line (selected aircraft → destination airport) is drawn live
    // by process() AFTER the aircraft layer — it depends on the selected
    // aircraft's current pixel position, which moves between frames, so
    // it can't be baked into the static-layer cache.
}
