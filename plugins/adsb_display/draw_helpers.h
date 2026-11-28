#pragma once

// DrawList helpers for the ADS-B Display layers and panels. The plugin's
// colours are cv::Scalar in RGB order (Theme's), and its layout works
// in OpenCV's integer pixels: (x, y) is the pixel whose centre is (x + 0.5, y + 0.5).

#include <draw/draw_list.h>
#include <draw/image.h>

#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <span>
#include <string_view>

namespace adsb_draw {

using spc::draw::Color;
using spc::draw::DrawList;
using spc::draw::Point;
using spc::draw::TextStyle;

// text size in pixels per unit of the OpenCV font scale the parameters were set in
inline constexpr float kPxPerFontScale = 28.f;

// Catppuccin Mocha
struct Theme {
    cv::Scalar bg{0x1e, 0x1e, 0x2e};
    cv::Scalar panel_bg{0x30, 0x2a, 0x3b};
    cv::Scalar surface1{0x45, 0x47, 0x5a};
    cv::Scalar surface2{0x58, 0x5c, 0x6e};
    cv::Scalar text{0xcd, 0xd6, 0xf4};
    cv::Scalar subtext{0xa6, 0xad, 0xc8};
    cv::Scalar green{0xa6, 0xe3, 0x70};
    cv::Scalar blue{0x89, 0xba, 0xf3};
    cv::Scalar peach{0xfa, 0xb4, 0x7a};
    cv::Scalar red{0xf3, 0x6b, 0x62};
    cv::Scalar yellow{0xf9, 0xe7, 0x72};
};

// the map's buttons
inline constexpr int kMargin = 10;
inline constexpr int kButtonHeight = 26;
inline constexpr int kButtonGap = 4;
inline constexpr double kButtonFont = 0.35;

inline Color rgb(const cv::Scalar& c, double opacity = 1.0)
{
    const auto ch = [](double v) { return static_cast<uint8_t>(std::clamp(v, 0.0, 255.0) + 0.5); };
    return {ch(c[0]), ch(c[1]), ch(c[2]), ch(std::clamp(opacity, 0.0, 1.0) * 255.0)};
}

// the centre of OpenCV pixel (x, y)
inline Point px(double x, double y)
{
    return {static_cast<float>(x) + 0.5f, static_cast<float>(y) + 0.5f};
}

inline TextStyle style(int32_t font, double scale, const cv::Scalar& color, bool bold = false)
{
    TextStyle st;
    st.font = static_cast<spc::draw::Font>(std::clamp(font, 0, 2));
    st.size = static_cast<float>(scale) * kPxPerFontScale;
    st.color = rgb(color);
    st.bold = bold;
    return st;
}

inline float text_width(std::string_view text, const TextStyle& st)
{
    return spc::draw::measure_text(text, st).width;
}

// what cv::getTextSize called the height: the face's ascent
inline float text_height(const TextStyle& st)
{
    return spc::draw::measure_text("H", st).ascent;
}

// baseline at y, as cv::putText takes its origin
inline void text(DrawList& list, double x, double y, std::string_view s, const TextStyle& st)
{
    list.text({static_cast<float>(x), static_cast<float>(y)}, s, st);
}

// top at y
inline void text_top(DrawList& list, double x, double y, std::string_view s, const TextStyle& st)
{
    text(list, x, y + text_height(st), s, st);
}

// centred in [y, y + h)
inline void text_vcenter(DrawList& list, double x, double y, double h, std::string_view s, const TextStyle& st)
{
    const float th = text_height(st);
    text(list, x, y + (h - th) / 2 + th, s, st);
}

// right-aligned in [x, x + col_w), centred in [y, y + h), never left of min_x
inline void text_right(DrawList& list, double x, double col_w, double y, double h, std::string_view s,
                       const TextStyle& st, double min_x = 0)
{
    const double tx = std::max(x + col_w - text_width(s, st), min_x);
    text_vcenter(list, tx, y, h, s, st);
}

// a halo round the glyphs, for labels over an unknown background
inline void text_outlined(DrawList& list, double x, double y, std::string_view s, TextStyle st,
                          const cv::Scalar& halo, float outline_px = 1.f)
{
    st.outline = rgb(halo);
    st.outline_width = outline_px;
    text(list, x, y, s, st);
}

// OpenCV's inclusive pixel rectangle [x0, x1] x [y0, y1], as cv::rectangle takes it
inline void fill_px(DrawList& list, double x0, double y0, double x1, double y1, Color c)
{
    list.fill_rect(static_cast<float>(std::min(x0, x1)), static_cast<float>(std::min(y0, y1)),
                   static_cast<float>(std::max(x0, x1)) + 1, static_cast<float>(std::max(y0, y1)) + 1, c);
}

// its outline, through the centres of its edge pixels (cv::rectangle with a thickness)
inline void stroke_px(DrawList& list, double x0, double y0, double x1, double y1, Color c, float width = 1)
{
    list.stroke_rect(static_cast<float>(x0) + 0.5f, static_cast<float>(y0) + 0.5f,
                     static_cast<float>(x1) + 0.5f, static_cast<float>(y1) + 0.5f, width, c);
}

inline void stroke_polygon(DrawList& list, std::span<const Point> pts, float width, Color c)
{
    for (size_t i = 0; i < pts.size(); ++i) list.line(pts[i], pts[(i + 1) % pts.size()], width, c);
}

inline void dashed_line(DrawList& list, Point a, Point b, float dash, float gap, float width, Color c)
{
    const float dx = b.x - a.x, dy = b.y - a.y;
    const float len = std::sqrt(dx * dx + dy * dy);
    if (len <= 0) return;
    const float ux = dx / len, uy = dy / len;
    for (float d = 0; d < len; d += dash + gap) {
        const float d1 = std::min(d + dash, len);
        list.line({a.x + ux * d, a.y + uy * d}, {a.x + ux * d1, a.y + uy * d1}, width, c);
    }
}

// an arc from a0 to a1 degrees, clockwise from +x on screen as cv::ellipse
// measures it, in chords no more than half a pixel off the curve
inline void arc(DrawList& list, Point centre, float radius, float a0, float a1, float width, Color c)
{
    if (radius <= 0 || a1 <= a0) return;
    const float span = (a1 - a0) * static_cast<float>(std::numbers::pi) / 180.f;
    const float step = 2.f * std::acos(std::max(0.f, 1.f - 0.5f / radius));
    const int n = std::clamp(static_cast<int>(std::ceil(span / std::max(step, 1e-3f))), 1, 256);
    const float t0 = a0 * static_cast<float>(std::numbers::pi) / 180.f;
    Point prev{centre.x + radius * std::cos(t0), centre.y + radius * std::sin(t0)};
    for (int i = 1; i <= n; ++i) {
        const float t = t0 + span * static_cast<float>(i) / static_cast<float>(n);
        const Point p{centre.x + radius * std::cos(t), centre.y + radius * std::sin(t)};
        list.line(prev, p, width, c);
        prev = p;
    }
}

// cv::drawMarker's shapes
enum class Marker { Diamond, Square, Star, TiltedCross, TriangleUp };

inline void marker(DrawList& list, Point c, Marker kind, float size, float width, Color col)
{
    const float h = size / 2;
    switch (kind) {
    case Marker::Diamond: {
        const Point p[] = {{c.x, c.y - h}, {c.x + h, c.y}, {c.x, c.y + h}, {c.x - h, c.y}};
        stroke_polygon(list, p, width, col);
        break;
    }
    case Marker::Square: {
        const Point p[] = {{c.x - h, c.y - h}, {c.x + h, c.y - h}, {c.x + h, c.y + h}, {c.x - h, c.y + h}};
        stroke_polygon(list, p, width, col);
        break;
    }
    case Marker::Star:
        list.line({c.x - h, c.y}, {c.x + h, c.y}, width, col);
        list.line({c.x, c.y - h}, {c.x, c.y + h}, width, col);
        list.line({c.x - h, c.y - h}, {c.x + h, c.y + h}, width, col);
        list.line({c.x + h, c.y - h}, {c.x - h, c.y + h}, width, col);
        break;
    case Marker::TiltedCross:
        list.line({c.x - h, c.y - h}, {c.x + h, c.y + h}, width, col);
        list.line({c.x + h, c.y - h}, {c.x - h, c.y + h}, width, col);
        break;
    case Marker::TriangleUp: {
        const Point p[] = {{c.x, c.y - h}, {c.x + h, c.y + h}, {c.x - h, c.y + h}};
        stroke_polygon(list, p, width, col);
        break;
    }
    }
}

// an image at its own size, its top-left at OpenCV pixel (x, y)
inline void image_at(DrawList& list, const spc::draw::Image& img, double x, double y)
{
    const auto w = static_cast<float>(img.width()), h = static_cast<float>(img.height());
    const auto fx = static_cast<float>(x), fy = static_cast<float>(y);
    list.image(img, 0, 0, w, h, fx, fy, fx + w, fy + h, spc::draw::Filter::Nearest);
}

}  // namespace adsb_draw
