#pragma once

// Geodesy + web-Mercator tile projection used by every on-map rendering
// layer. Header-only, no plugin-state dependencies.

#include <opencv2/core.hpp>

#include <cmath>
#include <numbers>
#include <utility>

inline constexpr int k_tile_size = 256;

struct TileCoord { int x, y, z; };

// Web-Mercator forward projection: WGS84 -> tile space at a given zoom.
inline double lon_to_tile_x(double lon, double zoom)
{
    return (lon + 180.0) / 360.0 * std::exp2(zoom);
}

inline double lat_to_tile_y(double lat, double zoom)
{
    double lat_rad = lat * std::numbers::pi / 180.0;
    return (1.0 - std::log(std::tan(lat_rad) + 1.0 / std::cos(lat_rad))
            / std::numbers::pi) / 2.0 * std::exp2(zoom);
}

// tile space -> WGS84 (inverse of the two above).
inline double tile_x_to_lon(double tx, double zoom)
{
    return tx / std::exp2(zoom) * 360.0 - 180.0;
}

inline double tile_y_to_lat(double ty, double zoom)
{
    double n = std::numbers::pi - 2.0 * std::numbers::pi * ty / std::exp2(zoom);
    return 180.0 / std::numbers::pi * std::atan(0.5 * (std::exp(n) - std::exp(-n)));
}

// WGS84 -> canvas-pixel position, given the top-left tile-space origin
// of the current viewport.
inline cv::Point2d geo_to_pixel(double lat, double lon, double zoom,
                                double origin_tx, double origin_ty)
{
    double tx = lon_to_tile_x(lon, zoom);
    double ty = lat_to_tile_y(lat, zoom);
    return {(tx - origin_tx) * k_tile_size,
            (ty - origin_ty) * k_tile_size};
}

// canvas-pixel -> WGS84 (out-parameters because we return two values).
inline void pixel_to_geo(double px, double py, double zoom,
                         double origin_tx, double origin_ty,
                         double& lat, double& lon)
{
    double tx = origin_tx + px / k_tile_size;
    double ty = origin_ty + py / k_tile_size;
    lon = tile_x_to_lon(tx, zoom);
    lat = tile_y_to_lat(ty, zoom);
}

// great-circle distance in km between two lat/lon points.
inline double haversine_km(double lat1, double lon1, double lat2, double lon2)
{
    constexpr double R = 6371.0;
    double dlat = (lat2 - lat1) * std::numbers::pi / 180.0;
    double dlon = (lon2 - lon1) * std::numbers::pi / 180.0;
    double a = std::sin(dlat / 2) * std::sin(dlat / 2)
             + std::cos(lat1 * std::numbers::pi / 180.0)
             * std::cos(lat2 * std::numbers::pi / 180.0)
             * std::sin(dlon / 2) * std::sin(dlon / 2);
    return R * 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
}

// destination point given origin lat/lon, bearing (deg from N, CW), distance (km).
// Great-circle formula.
inline std::pair<double, double> dest_point(double lat, double lon,
                                            double bearing_deg, double dist_km)
{
    constexpr double R = 6371.0;
    double br = bearing_deg * std::numbers::pi / 180.0;
    double la1 = lat * std::numbers::pi / 180.0;
    double lo1 = lon * std::numbers::pi / 180.0;
    double d = dist_km / R;
    double la2 = std::asin(std::sin(la1) * std::cos(d)
                           + std::cos(la1) * std::sin(d) * std::cos(br));
    double lo2 = lo1 + std::atan2(std::sin(br) * std::sin(d) * std::cos(la1),
                                  std::cos(d) - std::sin(la1) * std::sin(la2));
    return {la2 * 180.0 / std::numbers::pi, lo2 * 180.0 / std::numbers::pi};
}

// bearing in degrees (0=N, 90=E) from point 1 to point 2.
inline double bearing_deg(double lat1, double lon1, double lat2, double lon2)
{
    double dlon = (lon2 - lon1) * std::numbers::pi / 180.0;
    double la1 = lat1 * std::numbers::pi / 180.0;
    double la2 = lat2 * std::numbers::pi / 180.0;
    double y = std::sin(dlon) * std::cos(la2);
    double x = std::cos(la1) * std::sin(la2)
             - std::sin(la1) * std::cos(la2) * std::cos(dlon);
    double brg = std::atan2(y, x) * 180.0 / std::numbers::pi;
    return std::fmod(brg + 360.0, 360.0);
}
