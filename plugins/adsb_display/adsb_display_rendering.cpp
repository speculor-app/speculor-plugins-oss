#include "adsb_display_state.h"
#include "aircraft_shapes.h"
#include "draw_helpers.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <string>
#include <unordered_map>
#include <vector>

using namespace adsb_draw;

namespace {

// the halo's width across the silhouette's edge, in pixels
constexpr float k_halo_px = 2.5f;
// points closer than this to a kept edge are dropped, in pixels at the drawn size
constexpr float k_simplify_px = 0.3f;

// Fallback silhouette used when the ICAO type code has no mapped shape.
constexpr float k_fallback[] = {
     0.00f, -1.20f,  0.12f, -0.80f,  0.15f, -0.30f,
     0.90f, -0.10f,  0.85f,  0.10f,  0.15f,  0.00f,
     0.12f,  0.60f,  0.45f,  0.70f,  0.40f,  0.90f,
     0.10f,  0.85f,  0.00f,  1.00f, -0.10f,  0.85f,
    -0.40f,  0.90f, -0.45f,  0.70f, -0.12f,  0.60f,
    -0.15f,  0.00f, -0.85f,  0.10f, -0.90f, -0.10f,
    -0.15f, -0.30f, -0.12f, -0.80f
};

Point at(const float* pts, int i)
{
    return {pts[i * 2], pts[i * 2 + 1]};
}

// Douglas-Peucker: keeps the points of (first, last) farther than tol from the chord
void simplify(const float* pts, int first, int last, float tol, std::vector<char>& keep)
{
    if (last - first < 2) return;
    const Point a = at(pts, first), b = at(pts, last);
    const float dx = b.x - a.x, dy = b.y - a.y;
    const float len2 = dx * dx + dy * dy;
    float worst = -1;
    int worst_i = -1;
    for (int i = first + 1; i < last; ++i) {
        const Point p = at(pts, i);
        float d;
        if (len2 <= 0) {
            d = std::hypot(p.x - a.x, p.y - a.y);
        } else {
            const float t = std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / len2, 0.f, 1.f);
            d = std::hypot(p.x - a.x - t * dx, p.y - a.y - t * dy);
        }
        if (d > worst) {
            worst = d;
            worst_i = i;
        }
    }
    if (worst <= tol) return;
    keep[static_cast<size_t>(worst_i)] = 1;
    simplify(pts, first, worst_i, tol, keep);
    simplify(pts, worst_i, last, tol, keep);
}

// The shape's outline in shape units, with no more points than `size` pixels can show.
// Per thread: each node draws on its own worker.
const std::vector<Point>& outline_for(const aircraft_shapes::Shape* shape, int size)
{
    thread_local std::unordered_map<std::string, std::vector<Point>> cache;
    const float* pts = shape ? shape->points : k_fallback;
    const int n = shape ? shape->point_count : static_cast<int>(std::size(k_fallback) / 2);
    std::string key = shape ? shape->name : "__fallback__";
    key += '/';
    key += std::to_string(size);
    auto it = cache.find(key);
    if (it != cache.end()) return it->second;

    std::vector<char> keep(static_cast<size_t>(n), 0);
    keep[0] = 1;
    // split the closed outline at its farthest point from the first, so each half has a chord
    int far_i = 0;
    float far_d = -1;
    for (int i = 1; i < n; ++i) {
        const float d = std::hypot(at(pts, i).x - pts[0], at(pts, i).y - pts[1]);
        if (d > far_d) {
            far_d = d;
            far_i = i;
        }
    }
    if (far_i > 0) {
        const float tol = k_simplify_px / static_cast<float>(std::max(size, 1));
        keep[static_cast<size_t>(far_i)] = 1;
        simplify(pts, 0, far_i, tol, keep);
        std::vector<float> wrapped(pts, pts + static_cast<size_t>(n) * 2);
        wrapped.push_back(pts[0]);
        wrapped.push_back(pts[1]);
        keep.push_back(0);
        simplify(wrapped.data(), far_i, n, tol, keep);
        keep.pop_back();
    }
    std::vector<Point> out;
    for (int i = 0; i < n; ++i)
        if (keep[static_cast<size_t>(i)]) out.push_back(at(pts, i));
    return cache.emplace(std::move(key), std::move(out)).first->second;
}

}  // namespace

void draw_aircraft(DrawList& list, Point pos, float track_deg, const cv::Scalar& color, int size,
                   const aircraft_shapes::Shape* shape, const cv::Scalar& halo_color)
{
    const auto& outline = outline_for(shape, size);
    if (outline.size() < 3) return;

    const double angle = track_deg * std::numbers::pi / 180.0;
    const auto sn = static_cast<float>(std::sin(angle) * size);
    const auto cs = static_cast<float>(std::cos(angle) * size);
    const auto place = [&](Point p) { return Point{pos.x + cs * p.x - sn * p.y, pos.y + sn * p.x + cs * p.y}; };

    thread_local std::vector<Point> poly;
    poly.clear();
    for (const Point& p : outline) poly.push_back(place(p));

    stroke_polygon(list, poly, k_halo_px, rgb(halo_color));
    list.fill_polygon(poly, rgb(color));
    const cv::Scalar light(std::min(255.0, color[0] * 1.4), std::min(255.0, color[1] * 1.4),
                           std::min(255.0, color[2] * 1.4));
    list.line(place({0, -0.8f}), place({0, 0.7f}), 1, rgb(light));
}
