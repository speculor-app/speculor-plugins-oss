#include "adsb_display_state.h"
#include "aircraft_shapes.h"

#include <array>
#include <numbers>
#include <string>
#include <unordered_map>

namespace {

// Pre-rasterization as a mipmap chain. warpAffine has no INTER_AREA, so a
// single large template ends up aliased under the 5× downsample typical
// icons need. Instead we rasterize the shape at several scales and pick the
// level closest to the target size, keeping the per-frame warp near 1:1.
constexpr int k_margin     = 6;      // halo width + AA bleed room
constexpr int k_halo_px    = 3;      // halo polyline thickness, baked per level
constexpr int k_fp_shift   = 4;
constexpr double k_fp_scale = 1 << k_fp_shift;

// Chosen to cover the icon_size range (~4–65 px) with ≤~2× warp ratios.
constexpr std::array<int, 4> k_level_half_extents = {64, 32, 16, 8};

struct Level {
    cv::Mat masks;       // CV_8UC3: (fill, halo, highlight) alpha
    int half_extent;     // shape coord 1.0 maps to this many pixels
};

struct Sprite {
    std::array<Level, k_level_half_extents.size()> levels;
};

cv::Point fp(double x, double y) {
    return {
        static_cast<int>(x * k_fp_scale + 0.5),
        static_cast<int>(y * k_fp_scale + 0.5)
    };
}

void build_level(const float* points, int point_count, int H, Level& out) {
    out.half_extent = H;
    const int dim = 2 * (H + k_margin);
    const double center = H + k_margin;

    std::vector<cv::Point> pts;
    pts.reserve(point_count);
    for (int i = 0; i < point_count; ++i) {
        double dx = points[i * 2];
        double dy = points[i * 2 + 1];
        pts.push_back(fp(center + dx * H, center + dy * H));
    }

    cv::Mat fill(dim, dim, CV_8UC1, cv::Scalar(0));
    cv::Mat halo(dim, dim, CV_8UC1, cv::Scalar(0));
    cv::Mat hl  (dim, dim, CV_8UC1, cv::Scalar(0));

    cv::fillPoly(fill, pts, cv::Scalar(255), cv::LINE_AA, k_fp_shift);

    // halo = (fill ∪ k_halo_px polyline) − fill, a clean outer ring
    cv::Mat grown(dim, dim, CV_8UC1, cv::Scalar(0));
    cv::polylines(grown, pts, true, cv::Scalar(255), k_halo_px, cv::LINE_AA, k_fp_shift);
    cv::fillPoly(grown, pts, cv::Scalar(255), cv::LINE_AA, k_fp_shift);
    halo = grown - fill;  // uchar saturating subtract

    cv::line(hl,
             fp(center, center - 0.8 * H),
             fp(center, center + 0.7 * H),
             cv::Scalar(255), 1, cv::LINE_AA, k_fp_shift);

    cv::merge(std::vector<cv::Mat>{fill, halo, hl}, out.masks);
}

void build_sprite_from_points(const float* points, int point_count, Sprite& out) {
    for (size_t i = 0; i < k_level_half_extents.size(); ++i) {
        build_level(points, point_count, k_level_half_extents[i], out.levels[i]);
    }
}

// Fallback silhouette used when the ICAO type code has no mapped shape.
void build_fallback_sprite(Sprite& out) {
    static constexpr float body[] = {
         0.00f, -1.20f,  0.12f, -0.80f,  0.15f, -0.30f,
         0.90f, -0.10f,  0.85f,  0.10f,  0.15f,  0.00f,
         0.12f,  0.60f,  0.45f,  0.70f,  0.40f,  0.90f,
         0.10f,  0.85f,  0.00f,  1.00f, -0.10f,  0.85f,
        -0.40f,  0.90f, -0.45f,  0.70f, -0.12f,  0.60f,
        -0.15f,  0.00f, -0.85f,  0.10f, -0.90f, -0.10f,
        -0.15f, -0.30f, -0.12f, -0.80f
    };
    build_sprite_from_points(body, 20, out);
}

// Keyed by shape name — `const char*` identity isn't stable across TUs since
// k_shapes is `static constexpr`, so a pointer from plugin.cpp won't match.
const std::unordered_map<std::string, Sprite>& sprite_cache() {
    static const auto cache = [] {
        std::unordered_map<std::string, Sprite> m;
        m.reserve(aircraft_shapes::k_shape_count + 1);
        for (int i = 0; i < aircraft_shapes::k_shape_count; ++i) {
            const auto& s = aircraft_shapes::k_shapes[i];
            build_sprite_from_points(s.points, s.point_count, m[s.name]);
        }
        build_fallback_sprite(m["__fallback__"]);
        return m;
    }();
    return cache;
}

const Sprite& get_sprite(const aircraft_shapes::Shape* shape) {
    const auto& cache = sprite_cache();
    if (shape) {
        if (auto it = cache.find(shape->name); it != cache.end()) return it->second;
    }
    return cache.at("__fallback__");
}

// Smallest level whose half-extent is still ≥ target size, so the per-frame
// warp is at most ~2× downscale (or an upscale for icons larger than L0).
const Level& pick_level(const Sprite& sp, int size) {
    size_t i = 0;
    while (i + 1 < sp.levels.size() && sp.levels[i + 1].half_extent >= size) ++i;
    return sp.levels[i];
}

} // namespace

void draw_aircraft(cv::Mat& canvas, cv::Point2d pos, float track_deg,
                   cv::Scalar color, int size,
                   const aircraft_shapes::Shape* shape,
                   cv::Scalar halo_color)
{
    const Sprite& sprite = get_sprite(shape);
    const Level& lv = pick_level(sprite, size);
    const double sprite_center = lv.half_extent + k_margin;

    double angle = track_deg * std::numbers::pi / 180.0;
    double sn = std::sin(angle);
    double cs = std::cos(angle);
    double s = static_cast<double>(size) / lv.half_extent;

    // Forward affine: sprite coord -> canvas coord.
    // dst = pos + scale * R * (src - sprite_center)
    double M00 =  s * cs, M01 = -s * sn;
    double M10 =  s * sn, M11 =  s * cs;
    double tx = pos.x - (M00 * sprite_center + M01 * sprite_center);
    double ty = pos.y - (M10 * sprite_center + M11 * sprite_center);

    // Transformed bbox of the sprite on the canvas
    const double W = lv.masks.cols;
    const double H = lv.masks.rows;
    const double cx[4] = {0, W, W, 0};
    const double cy[4] = {0, 0, H, H};
    double min_x = 1e18, max_x = -1e18, min_y = 1e18, max_y = -1e18;
    for (int i = 0; i < 4; ++i) {
        double x = M00 * cx[i] + M01 * cy[i] + tx;
        double y = M10 * cx[i] + M11 * cy[i] + ty;
        min_x = std::min(min_x, x); max_x = std::max(max_x, x);
        min_y = std::min(min_y, y); max_y = std::max(max_y, y);
    }
    int x0 = static_cast<int>(std::floor(min_x)) - 1;
    int y0 = static_cast<int>(std::floor(min_y)) - 1;
    int x1 = static_cast<int>(std::ceil(max_x))  + 1;
    int y1 = static_cast<int>(std::ceil(max_y))  + 1;

    cv::Rect dst_bbox(x0, y0, x1 - x0, y1 - y0);
    cv::Rect canvas_rect(0, 0, canvas.cols, canvas.rows);
    cv::Rect roi = dst_bbox & canvas_rect;
    if (roi.width <= 0 || roi.height <= 0) return;

    // Warp into a small ROI-local buffer with the forward transform shifted
    // so (0,0) of the output is at (roi.x, roi.y) on the canvas.
    cv::Matx23d M_roi(M00, M01, tx - roi.x,
                      M10, M11, ty - roi.y);
    cv::Mat warped(roi.height, roi.width, CV_8UC3);
    cv::warpAffine(lv.masks, warped, cv::Mat(M_roi), roi.size(),
                   cv::INTER_CUBIC, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

    cv::Mat canvas_roi = canvas(roi);
    const int cr0 = static_cast<int>(color[0]);
    const int cr1 = static_cast<int>(color[1]);
    const int cr2 = static_cast<int>(color[2]);
    const int hr0 = static_cast<int>(halo_color[0]);
    const int hr1 = static_cast<int>(halo_color[1]);
    const int hr2 = static_cast<int>(halo_color[2]);
    const int lr0 = std::min(255, static_cast<int>(color[0] * 1.4));
    const int lr1 = std::min(255, static_cast<int>(color[1] * 1.4));
    const int lr2 = std::min(255, static_cast<int>(color[2] * 1.4));

    // Composite: halo first (outer ring), fill on top (body), highlight last.
    // halo/fill alphas are disjoint by construction (halo = grown - fill), so
    // order only matters for the highlight, which should sit on the body.
    for (int y = 0; y < warped.rows; ++y) {
        const cv::Vec3b* wrow = warped.ptr<cv::Vec3b>(y);
        cv::Vec3b* drow = canvas_roi.ptr<cv::Vec3b>(y);
        for (int x = 0; x < warped.cols; ++x) {
            int fa = wrow[x][0];
            int ha = wrow[x][1];
            int la = wrow[x][2];
            if ((fa | ha | la) == 0) continue;

            int d0 = drow[x][0], d1 = drow[x][1], d2 = drow[x][2];
            if (ha) {
                int ia = 255 - ha;
                d0 = (d0 * ia + hr0 * ha) / 255;
                d1 = (d1 * ia + hr1 * ha) / 255;
                d2 = (d2 * ia + hr2 * ha) / 255;
            }
            if (fa) {
                int ia = 255 - fa;
                d0 = (d0 * ia + cr0 * fa) / 255;
                d1 = (d1 * ia + cr1 * fa) / 255;
                d2 = (d2 * ia + cr2 * fa) / 255;
            }
            if (la) {
                int ia = 255 - la;
                d0 = (d0 * ia + lr0 * la) / 255;
                d1 = (d1 * ia + lr1 * la) / 255;
                d2 = (d2 * ia + lr2 * la) / 255;
            }
            drow[x][0] = static_cast<uchar>(d0);
            drow[x][1] = static_cast<uchar>(d1);
            drow[x][2] = static_cast<uchar>(d2);
        }
    }
}
