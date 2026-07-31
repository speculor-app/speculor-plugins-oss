#include "view_math.h"
#include "adsb_display_state.h"
#include "geo_helpers.h"

#include <algorithm>
#include <cmath>
#include <numbers>

double zoom_for_span(double lat_span, double lon_span, int w, int h)
{
    if (lat_span < 0.001) lat_span = 0.001;
    if (lon_span < 0.001) lon_span = 0.001;

    // solve span / 360 * 2^z * tile_size <= viewport
    double z_lon = std::log2(w / (lon_span / 360.0 * k_tile_size));
    double z_lat = std::log2(h / (lat_span / 180.0 * k_tile_size));
    return std::clamp(std::min(z_lon, z_lat), 2.0, 18.0);
}

void radius_to_span(double radius_km, double lat_deg,
                    double& lat_span, double& lon_span)
{
    // 1 deg latitude ~= 110.574 km, 1 deg longitude ~= 111.32 * cos(lat) km
    lat_span = (radius_km * 2.0) / 110.574;
    double cos_lat = std::cos(lat_deg * std::numbers::pi / 180.0);
    if (cos_lat < 0.01) cos_lat = 0.01;
    lon_span = (radius_km * 2.0) / (111.32 * cos_lat);
}

void sync_radius_from_zoom(MapDisplayState* s, int h)
{
    double zoom = static_cast<double>(s->zoom.load(std::memory_order_relaxed));
    double lat_span = h / (std::exp2(zoom) * k_tile_size) * 180.0;
    s->radius.store(static_cast<float>(lat_span / 2.0 * 110.574),
                    std::memory_order_relaxed);
}

void sync_zoom_from_radius(MapDisplayState* s, int w, int h)
{
    double lat_span, lon_span;
    radius_to_span(static_cast<double>(s->radius.load(std::memory_order_relaxed)),
                   s->center_lat.load(std::memory_order_relaxed), lat_span, lon_span);
    s->zoom.store(static_cast<float>(zoom_for_span(lat_span, lon_span, w, h)),
                  std::memory_order_relaxed);
}
