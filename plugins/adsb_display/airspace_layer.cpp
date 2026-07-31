#include "airspace_layer.h"
#include "adsb_display_state.h"
#include "color_palette.h"
#include "geo_helpers.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstdint>
#include <vector>

namespace {

// Per-class fill / outline colors. RGB (canvas is RGB, not BGR).
// Fills end up at airspace_opacity strength after alpha blend; outlines
// draw at full opacity on top so boundaries stay legible.
struct ClassStyle { cv::Scalar fill; cv::Scalar outline; };

ClassStyle style_for(AirspaceFeature::Class k)
{
    switch (k) {
        case AirspaceFeature::Class::Controlled:
            return {cv::Scalar(100, 160, 240), cv::Scalar( 70, 130, 210)};  // blue
        case AirspaceFeature::Class::Restricted:
            return {cv::Scalar(230,  90,  90), cv::Scalar(200,  60,  60)};  // red
        default:
            return {cv::Scalar(160, 160, 160), cv::Scalar(120, 120, 120)};  // neutral gray
    }
}

// Respect the `airspace_classes` enum: filter out polygons whose class
// the user doesn't want to see.
bool class_enabled(AirspaceFeature::Class k, int32_t enum_val)
{
    switch (enum_val) {
        case AIRSPACE_CLASSES_NONE:       return false;
        case AIRSPACE_CLASSES_CONTROLLED: return k == AirspaceFeature::Class::Controlled;
        case AIRSPACE_CLASSES_RESTRICTED: return k == AirspaceFeature::Class::Restricted;
        case AIRSPACE_CLASSES_ALL:
        default:                          return true;
    }
}

// Zoom-gating designed so the overlay thins progressively with zoom-in:
//   Large   (FIR/UIR/CTA/ACC)  — hide when zoomed past the country level
//   Medium  (TMA/ADIZ)          — visible at most zooms (regional ATC)
//   Small   (CTR/ATZ/MATZ/P/R/D/TRA/TSA/Airway/MTR/…) — only at airport
//                                  detail; keeps continental views from
//                                  turning into overlapping-circles soup
bool visible_at_zoom(AirspaceFeature::Size size, double zoom)
{
    switch (size) {
        case AirspaceFeature::Size::Large:  return zoom <= 9.0;
        case AirspaceFeature::Size::Medium: return zoom >= 5.0;
        case AirspaceFeature::Size::Small:  return zoom >= 10.0;
        default:                            return zoom >= 11.0;
    }
}

// Altitude filter: when the user has set an altitude, only keep polygons
// whose floor/ceiling bracket it. ceiling_ft == 0 means "no known upper
// limit" (OpenAIP didn't provide one) — treat as unlimited so we don't
// hide FIRs etc. Same for floor.
bool altitude_matches(const AirspaceFeature& a, int32_t alt_ft)
{
    if (alt_ft <= 0) return true;  // filter disabled
    if (a.floor_ft > 0   && a.floor_ft   > alt_ft) return false;
    if (a.ceiling_ft > 0 && a.ceiling_ft < alt_ft) return false;
    return true;
}

} // namespace

void render_airspace_layer(MapDisplayState* s,
                           double zoom, double origin_tx, double origin_ty,
                           int map_w, int h)
{
    if (!s->cur.show_airspace) return;
    if (!s->reference_loaded.load(std::memory_order_acquire)) return;
    if (s->reference.airspaces.empty()) return;
    if (s->cur.airspace_classes == AIRSPACE_CLASSES_NONE) return;

    // viewport bbox in lat/lon — used to cheaply reject polygons that
    // can't possibly be visible before we bother transforming their rings
    double tl_lat, tl_lon, br_lat, br_lon;
    pixel_to_geo(0.0,             0.0,   zoom, origin_tx, origin_ty, tl_lat, tl_lon);
    pixel_to_geo(static_cast<double>(map_w),
                 static_cast<double>(h), zoom, origin_tx, origin_ty, br_lat, br_lon);
    double view_min_lat = std::min(tl_lat, br_lat);
    double view_max_lat = std::max(tl_lat, br_lat);
    double view_min_lon = std::min(tl_lon, br_lon);
    double view_max_lon = std::max(tl_lon, br_lon);

    float opacity = std::clamp(s->cur.airspace_opacity, 0.0f, 1.0f);

    cv::Rect map_rect(0, 0, map_w, h);
    cv::Mat map_roi = s->canvas(map_rect);
    cv::Mat overlay;
    bool blend = opacity < 0.99f;
    if (blend) overlay = map_roi.clone();
    cv::Mat& fill_target = blend ? overlay : map_roi;

    // Stash the projected pixel rings as we go so the outline pass can
    // reuse them without re-projecting every vertex, and publish the
    // indices of airspaces that actually render so click hit-testing
    // only considers visible polygons.
    struct Projected {
        std::vector<cv::Point> pts;
        cv::Scalar outline;
    };
    std::vector<Projected> visible;
    visible.reserve(32);

    int build_idx = 1 - s->visible_airspace_active.load(std::memory_order_relaxed);
    auto& visible_idx = s->visible_airspace_idx[build_idx];
    visible_idx.clear();

    for (size_t i = 0; i < s->reference.airspaces.size(); ++i) {
        const auto& a = s->reference.airspaces[i];
        if (!class_enabled(a.klass, s->cur.airspace_classes)) continue;
        if (!visible_at_zoom(a.size, zoom)) continue;
        if (!altitude_matches(a, s->cur.airspace_altitude_ft)) continue;
        // axis-aligned bbox reject
        if (a.max_lat < view_min_lat || a.min_lat > view_max_lat) continue;
        if (a.max_lon < view_min_lon || a.min_lon > view_max_lon) continue;

        Projected pr;
        pr.pts.reserve(a.ring_lat_lon.size());
        for (const auto& [lat, lon] : a.ring_lat_lon) {
            auto px = geo_to_pixel(lat, lon, zoom, origin_tx, origin_ty);
            pr.pts.emplace_back(static_cast<int>(px.x), static_cast<int>(px.y));
        }
        if (pr.pts.size() < 3) continue;

        auto style = style_for(a.klass);
        pr.outline = style.outline;

        // Controlled airspace gets an outline only — with FIRs + TMAs + CTRs
        // stacking, filling them all turns the map into an opaque blob.
        // Restricted / P-R-D / MATZ still fill so they pop as "watch out".
        if (a.klass == AirspaceFeature::Class::Restricted) {
            cv::fillPoly(fill_target, std::vector<std::vector<cv::Point>>{pr.pts},
                         style.fill, cv::LINE_AA);
        }
        visible.push_back(std::move(pr));
        visible_idx.push_back(static_cast<int32_t>(i));
    }

    if (blend)
        cv::addWeighted(overlay, opacity, map_roi, 1.0 - opacity, 0, map_roi);

    // Outlines drawn directly on the canvas at full opacity so polygon
    // boundaries stay crisp regardless of the fill opacity.
    for (const auto& pr : visible) {
        cv::polylines(map_roi, pr.pts, true, pr.outline, 1, cv::LINE_AA);
    }

    s->visible_airspace_active.store(build_idx, std::memory_order_release);
}
